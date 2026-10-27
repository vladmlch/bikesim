// test_native_state_bundle.cpp — C++ port of
// tests/reference/test_native_state_bundle.py (T53 batch 3.27).
//
// The Python suite pins the BIKEST02 writer/reader/resolver contracts of
// tools/proto_native_bench/state_format.py. Most records are already
// covered through the real consumer — the native_bench executable spawned
// by reference.native_benchmark (3.24a port) exercises resolve_artifacts
// and proto_native_bench::load_state end to end; those records are `drop`
// in the port map with evidence pointing at the covering benchmark case.
//
// This file ports only the records whose claim is NOT exercised there:
//   * the 80-byte header constant and the serialized field layout
//     (test_header_is_exactly_80_bytes) — a structural pin;
//   * per-section decode values on a well-formed file
//     (test_read_state_roundtrip_values) — benchmark cases only exercise
//     rejection paths and whole-run success, never the decoded fields;
//   * the out-of-domain u64 dimension sweep over ALL five header fields
//     (test_out_of_domain_dimension_is_rejected) — the CLI test samples
//     nq only; the per-field call sites each carry their own offset, so
//     the sweep pins all five;
//   * the declared model_crc64 lie
//     (test_model_checksum_corruption_is_rejected) — the benchmark covers
//     the state-crc lie and the mixed-generation pair, not a corrupted
//     declared model checksum against intact model bytes;
//   * the hostile bundle-name sweep including '.' and '..' —
//     (test_resolve_pointer_name_cannot_escape) — the benchmark sweeps
//     '../escape', '/abs' and 'a/b' through the charset check; the
//     exact-match '.'/'..' rejection branch stays uncovered there;
//   * resolution of a nonexistent bundle directory
//     (test_resolve_missing_dir_is_rejected) — the benchmark covers a
//     regular-file root; the source record names a path that does not
//     exist at all.
//
// The reader side runs in-process against proto_native_bench::load_state
// (the same inline the benchmark's child process links); the resolver
// side keeps the argv boundary — resolve_artifacts exists only inside
// the native_bench executable, exactly like test_native_benchmark.cpp.
//
// The bundle writer records (pointer publication, name immutability,
// manifest provenance, npz/bin equality, serialize extent/nonfinite
// rejection) are Python-side tooling contracts: the C++ consumer reads
// state.bin + model.mjb and only checks the other members' existence, so
// no native owner exists — those records are `drop` in the port map.
#include <mujoco/mujoco.h>

#include "../../../tools/proto_native_bench/state_format.hpp"
#include "model/builder.hpp"
#include "platform/process.hpp"
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "support/golden.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <format>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "bike_native_bench_path.hpp"

#ifndef BIKE_NATIVE_BENCH_PATH
#error "BIKE_NATIVE_BENCH_PATH must be supplied by the generated bike_native_bench_path.hpp"
#endif

namespace {

using bike::model::ModelOwner;

// BIKEST02 header field offsets (struct '<8s2I8Q', 80 bytes) — the layout
// the source module's HDR_* constants pin.
constexpr std::size_t kHdrVersion = 8;
constexpr std::size_t kHdrFlags = 12;
constexpr std::size_t kHdrDimNq = 16;
constexpr std::size_t kHdrDimNv = 24;
constexpr std::size_t kHdrDimNa = 32;
constexpr std::size_t kHdrDimNu = 40;
constexpr std::size_t kHdrDimNbody = 48;
constexpr std::size_t kHdrPayloadBytes = 56;
constexpr std::size_t kHdrModelCrc = 64;
constexpr std::size_t kHdrStateCrc = 72;
constexpr std::size_t kHeaderBytes = 80;

// ---------------------------------------------------------------------------
// Paths + subprocess boundary (only the resolver sweep needs the executable)
// ---------------------------------------------------------------------------

[[nodiscard]] const std::filesystem::path &repo_root() {
    static const std::filesystem::path root = std::filesystem::weakly_canonical(
        bt::oracle::root() / ".." / ".." / "..");
    return root;
}

[[nodiscard]] const std::filesystem::path &bench_path() {
    static const std::filesystem::path path{BIKE_NATIVE_BENCH_PATH};
    return path;
}

struct BenchResult {
    int code = -1;
    std::string out;
    std::string err;
};

[[nodiscard]] BenchResult
run_bench(std::initializer_list<std::string> args) {
    std::vector<std::string> argv;
    argv.reserve(args.size() + 1);
    argv.push_back(bench_path().string());
    argv.insert(argv.end(), args.begin(), args.end());
    bike::platform::ProcessOptions options;
    options.timeout_s = 120.0;
    const bike::platform::ProcessResult result =
        bike::platform::run(argv, repo_root(), options);
    return {.code = result.exit_code, .out = result.out, .err = result.err};
}

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

void expect_exit(const BenchResult &result, int expected,
                 std::string_view context) {
    bt::require(result.code == expected,
                std::string(context) + ": native_bench exits " +
                    std::to_string(expected) + " (got " +
                    std::to_string(result.code) + "; stderr: " + result.err +
                    ")");
}

void expect_stderr_any(const BenchResult &result,
                       std::initializer_list<std::string_view> needles,
                       std::string_view context) {
    bool found = false;
    for (const std::string_view needle : needles)
        found |= contains(result.err, needle);
    bt::require(found, std::string(context) + ": stderr names the failure "
                       "(stderr: " + result.err + ")");
}

template <class F>
void throws_with(std::string_view needle, std::string_view context, F &&action) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::exception &failure) {
        rejected = true;
        bt::require(contains(failure.what(), needle),
                    std::string(context) + ": diagnostic names '" +
                        std::string(needle) + "' (got: " + failure.what() +
                        ")");
    }
    bt::require(rejected, std::string(context) + ": the reader rejects");
}

// ---------------------------------------------------------------------------
// Deterministic chain model + BIKEST02 fixture serialization — the same
// field-level twins test_native_benchmark.cpp carries (the two files must
// stay byte-identical in spirit: the serializer mirrors
// state_format.py::serialize_state, the model builder mirrors
// _chain_model).
// ---------------------------------------------------------------------------

[[nodiscard]] std::string chain_xml(int dofs, int actuators) {
    std::string bodies;
    for (int i = 0; i < dofs; ++i)
        bodies += std::format(
            "<body name=\"link{0}\" pos=\"{0} 0 0\">"
            "<joint name=\"joint{0}\" type=\"slide\" axis=\"1 0 0\"/>"
            "<geom type=\"sphere\" size=\"0.1\" mass=\"0.5\"/></body>",
            i);
    std::string motors;
    for (int i = 0; i < (dofs != 0 ? actuators : 0); ++i)
        motors += std::format(
            "<motor name=\"motor{0}\" joint=\"joint{1}\"/>", i, i % dofs);
    const std::string actuator_xml =
        motors.empty() ? std::string{} : "<actuator>" + motors + "</actuator>";
    return "<mujoco><worldbody>" + bodies + "</worldbody>" + actuator_xml +
           "</mujoco>";
}

[[nodiscard]] ModelOwner chain_model(int dofs, int actuators = 1) {
    return bike::model::compile_mjcf(chain_xml(dofs, actuators));
}

[[nodiscard]] std::vector<double> linspace(double start, double stop,
                                           std::size_t count) {
    std::vector<double> values(count);
    if (count == 0) return values;
    if (count == 1) {
        values.front() = start;
        return values;
    }
    const double step =
        (stop - start) / static_cast<double>(count - 1);
    for (std::size_t i = 0; i + 1 < count; ++i)
        values[i] = start + static_cast<double>(i) * step;
    values.back() = stop;
    return values;
}

// _arrays(model): the deterministic payload dict the Python helper emits.
struct StateArrays {
    double time = 0.125;
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> act;
    std::vector<double> ctrl;
    std::vector<double> qfrc;
    std::vector<double> xfrc;
    std::vector<double> qacc;
};

[[nodiscard]] StateArrays state_arrays(const mjModel &model) {
    StateArrays arrays;
    const auto nv = static_cast<std::size_t>(model.nv);
    const auto nbody = static_cast<std::size_t>(model.nbody);
    arrays.qpos = linspace(0.01, 0.02, static_cast<std::size_t>(model.nq));
    arrays.qvel = linspace(-0.01, 0.01, nv);
    arrays.act.assign(static_cast<std::size_t>(model.na), 0.0);
    arrays.ctrl = linspace(0.1, 0.2, static_cast<std::size_t>(model.nu));
    arrays.qfrc = linspace(0.001, 0.002, nv);
    arrays.xfrc.assign(6 * nbody, 0.0);
    arrays.qacc = linspace(0.0, 0.004, nv);
    return arrays;
}

void append_bytes(std::vector<char> &out, std::span<const std::byte> bytes) {
    for (const std::byte element : bytes)
        out.push_back(static_cast<char>(element));
}

void put_u32(std::vector<char> &out, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFFU));
}

void put_u64(std::vector<char> &out, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFFU));
}

void put_f64(std::vector<char> &out, double value) {
    append_bytes(out, std::as_bytes(std::span{&value, std::size_t{1}}));
}

void append_doubles(std::vector<char> &out, std::span<const double> values) {
    append_bytes(out, std::as_bytes(values));
}

// serialize_state: dims derive from the payload arrays; header fields land
// at the documented offsets; both checksums are honest.
[[nodiscard]] std::vector<char>
serialize_state(const StateArrays &arrays,
                std::span<const std::byte> model_bytes) {
    const std::size_t nq = arrays.qpos.size();
    const std::size_t nv = arrays.qvel.size();
    const std::size_t na = arrays.act.size();
    const std::size_t nu = arrays.ctrl.size();
    const std::size_t nbody = arrays.xfrc.size() / 6;
    bt::require(arrays.xfrc.size() % 6 == 0,
                "xfrc_applied element count is a multiple of 6");
    bt::require(arrays.qfrc.size() == nv && arrays.qacc.size() == nv,
                "qfrc_applied/qacc_warmstart extents equal nv");
    std::vector<char> payload;
    payload.reserve(8 * (1 + nq + 3 * nv + na + nu + 6 * nbody));
    put_f64(payload, arrays.time);
    append_doubles(payload, arrays.qpos);
    append_doubles(payload, arrays.qvel);
    append_doubles(payload, arrays.act);
    append_doubles(payload, arrays.ctrl);
    append_doubles(payload, arrays.qfrc);
    append_doubles(payload, arrays.xfrc);
    append_doubles(payload, arrays.qacc);
    const std::uint64_t model_crc =
        proto_native_bench::detail::crc64_ecma(model_bytes);
    const std::uint64_t state_crc = proto_native_bench::detail::crc64_ecma(
        std::as_bytes(std::span{payload}));
    std::vector<char> image;
    image.reserve(kHeaderBytes + payload.size());
    image.insert(image.end(), {'B', 'I', 'K', 'E', 'S', 'T', '0', '2'});
    put_u32(image, 2);
    put_u32(image, 0);
    for (const std::uint64_t dim : {static_cast<std::uint64_t>(nq),
                                    static_cast<std::uint64_t>(nv),
                                    static_cast<std::uint64_t>(na),
                                    static_cast<std::uint64_t>(nu),
                                    static_cast<std::uint64_t>(nbody)})
        put_u64(image, dim);
    put_u64(image, static_cast<std::uint64_t>(payload.size()));
    put_u64(image, model_crc);
    put_u64(image, state_crc);
    image.insert(image.end(), payload.begin(), payload.end());
    return image;
}

void write_bytes(const std::filesystem::path &path,
                 std::span<const char> bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    bt::require(file.is_open(), "fixture write opens " + path.string());
    if (!bytes.empty())
        file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    bt::require(file.good(), "fixture write completes " + path.string());
}

void write_text(const std::filesystem::path &path, std::string_view text) {
    write_bytes(path, std::span{text.data(), text.size()});
}

// A state.bin + model.mjb pair the reader accepts: the chain model saved
// through mj_saveModel plus the serialized arrays — _corrupt's baseline.
struct StatePair {
    std::filesystem::path state_path;
    std::filesystem::path model_path;
    StateArrays arrays;
};

[[nodiscard]] StatePair write_state_pair(
    const std::filesystem::path &dir, const mjModel &model) {
    std::filesystem::create_directories(dir);
    const std::filesystem::path model_path = dir / "model.mjb";
    mj_saveModel(&model, model_path.string().c_str(), nullptr, 0);
    const std::string model_bytes = bt::text(model_path);
    StatePair pair{.state_path = dir / "state.bin",
                   .model_path = model_path,
                   .arrays = state_arrays(model)};
    write_bytes(pair.state_path,
                serialize_state(pair.arrays,
                                std::as_bytes(std::span{model_bytes})));
    return pair;
}

void rewrite_state(const std::filesystem::path &state_path,
                   const std::vector<char> &image) {
    write_bytes(state_path, std::span{image});
}

void patch_u64(std::string &raw, std::size_t offset, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        raw.at(offset + shift / 8) =
            static_cast<char>((value >> shift) & 0xFFU);
}

[[nodiscard]] std::vector<char> patched_u64(std::string_view raw,
                                            std::size_t offset,
                                            std::uint64_t value) {
    std::string copy(raw);
    patch_u64(copy, offset, value);
    return {copy.begin(), copy.end()};
}

[[nodiscard]] std::size_t dim_offset(std::string_view name) {
    if (name == "nq") return kHdrDimNq;
    if (name == "nv") return kHdrDimNv;
    if (name == "na") return kHdrDimNa;
    if (name == "nu") return kHdrDimNu;
    bt::require(name == "nbody",
                "state dimension is one of the five header fields");
    return kHdrDimNbody;
}

void require_bitwise(std::span<const double> actual,
                     std::span<const double> expected,
                     std::string_view context) {
    bt::require(actual.size() == expected.size(),
                std::string(context) + ": widths agree");
    for (std::size_t i = 0; i < actual.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(actual[i]) ==
                        std::bit_cast<std::uint64_t>(expected[i]),
                    std::string(context) + ": element " + std::to_string(i) +
                        " is bitwise identical");
}

// ---------------------------------------------------------------------------
// Structural pins
// ---------------------------------------------------------------------------

// tests/reference/test_native_state_bundle.py::test_header_is_exactly_80_bytes
void test_header_is_exactly_80_bytes() {
    static_assert(proto_native_bench::detail::kHeaderBytes == 80,
                  "the BIKEST02 header is exactly 80 bytes");
    static_assert(sizeof(proto_native_bench::detail::kStateMagic) == 8,
                  "the magic is eight bytes");
    // The serialized image puts every field at its documented offset —
    // decode the fixed header of a generated file and check each slot.
    const bt::TemporaryDirectory dir("state_hdr");
    const auto model = chain_model(3, 2);
    const StatePair pair = write_state_pair(dir.path(), *model);
    const std::string raw = bt::text(pair.state_path);
    bt::require(raw.size() > kHeaderBytes, "the image carries a payload");
    const std::span<const char> bytes{raw};
    const std::span<const char> header = bytes.first(kHeaderBytes);
    bt::require(std::ranges::equal(header.first<8>(),
                                   std::span<const char>{
                                       proto_native_bench::detail::
                                           kStateMagic}),
                "magic occupies bytes 0..8");
    bt::require(proto_native_bench::detail::read_u32(
                    header.subspan(kHdrVersion, 4)) == 2 &&
                    proto_native_bench::detail::read_u32(
                        header.subspan(kHdrFlags, 4)) == 0,
                "version=2 and flags=0 occupy bytes 8..16");
    for (const auto &[name, offset, expected] :
         {std::tuple{"nq", kHdrDimNq, model->nq},
          {"nv", kHdrDimNv, model->nv},
          {"na", kHdrDimNa, model->na},
          {"nu", kHdrDimNu, model->nu},
          {"nbody", kHdrDimNbody, model->nbody}})
        bt::require(proto_native_bench::detail::read_u64(
                        header.subspan(offset, 8)) ==
                        static_cast<std::uint64_t>(expected),
                    std::string(name) + " occupies its header slot");
    const std::uint64_t payload = proto_native_bench::detail::read_u64(
        header.subspan(kHdrPayloadBytes, 8));
    bt::require(payload == raw.size() - kHeaderBytes,
                "payload_bytes names the byte extent after the header");
    const std::string model_bytes = bt::text(pair.model_path);
    bt::require(
        proto_native_bench::detail::read_u64(header.subspan(kHdrModelCrc, 8)) ==
            proto_native_bench::detail::crc64_ecma(
                std::as_bytes(std::span<const char>{model_bytes})),
        "model_crc64 occupies bytes 64..72");
    bt::require(
        proto_native_bench::detail::read_u64(header.subspan(kHdrStateCrc, 8)) ==
            proto_native_bench::detail::crc64_ecma(
                std::as_bytes(bytes.subspan(kHeaderBytes))),
        "state_crc64 occupies bytes 72..80");
}

// tests/reference/test_native_state_bundle.py::test_read_state_roundtrip_values
void test_read_state_roundtrip_values(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(3, 2);
    const StatePair pair = write_state_pair(dir.path(), *model);
    const proto_native_bench::State state = proto_native_bench::load_state(
        pair.state_path, pair.model_path, *model);
    bt::require(std::bit_cast<std::uint64_t>(state.time) ==
                    std::bit_cast<std::uint64_t>(pair.arrays.time),
                "time round-trips bitwise");
    require_bitwise(state.qpos, pair.arrays.qpos, "qpos round-trip");
    require_bitwise(state.qvel, pair.arrays.qvel, "qvel round-trip");
    require_bitwise(state.act, pair.arrays.act, "act round-trip");
    require_bitwise(state.ctrl, pair.arrays.ctrl, "ctrl round-trip");
    require_bitwise(state.qfrc, pair.arrays.qfrc, "qfrc_applied round-trip");
    require_bitwise(state.xfrc, pair.arrays.xfrc, "xfrc_applied round-trip");
    bt::require(state.xfrc.size() ==
                    6 * static_cast<std::size_t>(model->nbody),
                "xfrc_applied decodes as a flat 6*nbody section");
    require_bitwise(state.qacc_warmstart, pair.arrays.qacc,
                    "qacc_warmstart round-trip");
    bt::require(state.qacc_warmstart.size() ==
                    static_cast<std::size_t>(model->nv),
                "qacc_warmstart width equals nv");
    // The declared checksums survive the read as provenance.
    const std::string model_bytes = bt::text(pair.model_path);
    bt::require(state.model_crc64 ==
                    proto_native_bench::detail::crc64_ecma(
                        std::as_bytes(std::span<const char>{model_bytes})),
                "the declared model checksum is retained");
    const std::string raw = bt::text(pair.state_path);
    bt::require(state.state_crc64 ==
                    proto_native_bench::detail::crc64_ecma(std::as_bytes(
                        std::span<const char>{raw}.subspan(kHeaderBytes))),
                "the declared payload checksum is retained");
}

// ---------------------------------------------------------------------------
// Rejection contracts the benchmark port does not already exercise
// ---------------------------------------------------------------------------

// tests/reference/test_native_state_bundle.py::test_out_of_domain_dimension_is_rejected[nq|nv|na|nu|nbody]
void test_out_of_domain_dimension_is_rejected(const bt::TemporaryDirectory &dir) {
    // Each of the five u64 header fields gets its own reader call site — the
    // sweep pins all five, not just the nq slot the CLI variant samples. The
    // rejection fires before any model comparison or allocation.
    for (const std::string_view name : {"nq", "nv", "na", "nu", "nbody"}) {
        const auto model = chain_model(3, 2);
        const StatePair pair =
            write_state_pair(dir.path() / std::string(name), *model);
        const std::string raw = bt::text(pair.state_path);
        rewrite_state(pair.state_path,
                      patched_u64(raw, dim_offset(name), 1ULL << 63U));
        throws_with("domain", "2^63 " + std::string(name) + " header field",
                    [&] {
                        (void)proto_native_bench::load_state(
                            pair.state_path, pair.model_path, *model);
                    });
    }
}

// tests/reference/test_native_state_bundle.py::test_model_checksum_corruption_is_rejected
void test_model_checksum_corruption_is_rejected(const bt::TemporaryDirectory &dir) {
    // The declared model_crc64 no longer matches the intact model.mjb bytes —
    // the benchmark covers the state-crc lie and the mixed-generation pair,
    // this direction pins a corrupted declared value outright.
    const auto model = chain_model(3, 2);
    const StatePair pair = write_state_pair(dir.path(), *model);
    const std::string raw = bt::text(pair.state_path);
    rewrite_state(pair.state_path,
                  patched_u64(raw, kHdrModelCrc, 0xDEADBEEF));
    throws_with("checksum", "declared model_crc64 lie", [&] {
        (void)proto_native_bench::load_state(pair.state_path, pair.model_path,
                                             *model);
    });
}

// tests/reference/test_native_state_bundle.py::test_resolve_pointer_name_cannot_escape[hostile]
// resolve_artifacts exists only inside native_bench — keep the argv
// boundary. The benchmark sweeps '../escape', '/abs', 'a/b'; this port
// covers the full five-name source parametrization including the
// exact-match '.'/'..' rejections the charset check would otherwise allow.
void test_resolve_pointer_name_cannot_escape(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    // A minimal resolvable bundle: the pointer rejection fires before the
    // member check, so only the names need to exist for the exit code.
    const std::filesystem::path root = dir.path() / "artifacts";
    std::filesystem::create_directories(root / "bundle");
    write_bytes(root / "bundle" / "model.mjb", std::span{std::string_view{"x"}});
    for (const std::string_view hostile :
         {"../escape", "/abs", "a/b", "..", "."}) {
        write_text(root / "current.json",
                   std::format("{{\"format\": 1, \"bundle\": \"{}\"}}",
                               hostile));
        const BenchResult result =
            run_bench({"bare", "--artifacts", root.string()});
        expect_exit(result, 1,
                    "hostile bundle name '" + std::string(hostile) + "'");
        expect_stderr_any(result, {"bundle name"},
                          "hostile bundle name '" + std::string(hostile) +
                              "' diagnostic");
    }
}

// tests/reference/test_native_state_bundle.py::test_resolve_missing_dir_is_rejected
void test_resolve_missing_dir_is_rejected(const bt::TemporaryDirectory &dir) {
    // A path that does not exist at all still names model.mjb — member
    // existence is checked uniformly, missing-or-not-a-directory alike.
    const std::filesystem::path missing = dir.path() / "nonexistent";
    const BenchResult result =
        run_bench({"bare", "--artifacts", missing.string()});
    expect_exit(result, 1, "nonexistent bundle root");
    expect_stderr_any(result, {"model.mjb"}, "nonexistent root diagnostic");
}

template <typename Fn> void with_tmp_path(const char *label, Fn &&fn) {
    const bt::TemporaryDirectory dir(label);
    fn(dir);
}

void check() {
    test_header_is_exactly_80_bytes();
    with_tmp_path("state_roundtrip", test_read_state_roundtrip_values);
    with_tmp_path("state_dims", test_out_of_domain_dimension_is_rejected);
    with_tmp_path("state_crc", test_model_checksum_corruption_is_rejected);
    with_tmp_path("state_resolve", test_resolve_pointer_name_cannot_escape);
    with_tmp_path("state_resolve", test_resolve_missing_dir_is_rejected);
}

} // namespace

int main() { return bt::run(check); }
