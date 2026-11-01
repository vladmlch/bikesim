// test_native_benchmark.cpp — C++ port of
// tests/reference/test_native_benchmark.py (T52 batch 3.24a).
//
// The unit under test is the real `native_bench` executable from the same
// build (resolved through the generated bike_native_bench_path.hpp macro and
// executed through bike::platform::run — the bounded argv boundary: no
// shell, no parent-cwd mutation, concurrent stdout/stderr drain, killpg
// cleanup, deadline + byte bound). The Python suite runs each case through
// subprocess.run(cwd=REPO_ROOT, timeout=120); this port spawns with
// cwd=<repo root> and timeout_s=120 identically. platform::run encodes a
// signal death as 128+signal where Python reports -signal — the ported
// assertions only ever require exit code 0 or 1, so the encodings are
// equivalent here.
//
// Bundle fixtures are generated in test code, mirroring the Python helpers
// byte-for-byte: the same slide-joint chain XML through the same compiler,
// mj_saveModel for model.mjb, the BIKEST02 serializer duplicated at the
// field level (tools/proto_native_bench/state_format.hpp crc64_ecma is the
// shared checksum authority), a genuine .npz member archive through
// bike::io::encode_npz, and the {"format":1,"bundle":...} pointer file.
// Only the matched-workload conformance node replays a captured oracle:
// the recorded Python report is consumed through bytes_input("input.report")
// and every native-vs-oracle comparison from the pytest body is re-asserted
// verbatim — including the Python `==` number-value semantics across
// int/float lexemes (1e-06 == 9.9999999999999995e-07 is equality there).
//
// Sanitizer note: the Python suite injects the selected sanitizer runtime
// into the child's preload list because pytest itself runs uninstrumented.
// The root build LINKS sanitizer runtimes into every binary — including
// this test and native_bench — so verbatim environment inheritance inside
// platform::run reproduces the contract without a preload shim.
#include <mujoco/mujoco.h>

#include "../../../tools/proto_native_bench/state_format.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "io/value.hpp"
#include "model/builder.hpp"
#include "platform/process.hpp"
#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "bike_native_bench_path.hpp"

#ifndef BIKE_NATIVE_BENCH_PATH
#error "BIKE_NATIVE_BENCH_PATH must be supplied by the generated bike_native_bench_path.hpp"
#endif

namespace {

using bike::io::Number;
using bike::io::Value;

constexpr std::string_view kNodeFile =
    "tests/reference/test_native_benchmark.py";
constexpr std::int64_t kMaxCount = 10'000'000;
constexpr std::int32_t kLegacyStateMagic = 0xBEA0;

// BIKEST02 header field offsets (struct '<8s2I8Q', 80 bytes).
constexpr std::size_t kHdrVersion = 8;
constexpr std::size_t kHdrFlags = 12;
constexpr std::size_t kHdrDimNq = 16;
constexpr std::size_t kHdrDimNv = 24;
constexpr std::size_t kHdrDimNa = 32;
constexpr std::size_t kHdrDimNu = 40;
constexpr std::size_t kHdrDimNbody = 48;
constexpr std::size_t kHdrPayloadBytes = 56;
constexpr std::size_t kHdrStateCrc = 72;
constexpr std::size_t kHeaderBytes = 80;

// ---------------------------------------------------------------------------
// Paths + subprocess boundary
// ---------------------------------------------------------------------------

// REPO_ROOT in the pytest module: the checkout root that holds
// tools/proto_native_bench/artifacts. Derived from the CMake-pinned golden
// root (<repo>/tests/fixtures/golden) — never the process working directory.
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

// _run_bench: subprocess.run([bench, *args], cwd=REPO_ROOT, timeout=120,
// capture_output=True) — argv boundary, same cwd, same 120 s deadline.
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

// ---------------------------------------------------------------------------
// Assertion spellings mirroring the pytest asserts
// ---------------------------------------------------------------------------

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

[[nodiscard]] std::string lowercase(std::string_view text) {
    std::string lowered(text);
    std::ranges::transform(lowered, lowered.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return lowered;
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
    for (const std::string_view needle : needles) found |= contains(result.err, needle);
    bt::require(found, std::string(context) + ": stderr names the failure " +
                           "(stderr: " + result.err + ")");
}

// ---------------------------------------------------------------------------
// Deterministic chain models — _chain_model / _history_model
// ---------------------------------------------------------------------------

// _chain_model: nq = nv = dofs, nu = actuators, nbody = dofs + welded + 1.
// `actuators` may exceed dofs (motors stack on joints cyclically) — the
// nu > nv case the actuator->velocity mapping must handle. dofs == 0
// cannot carry motors.
[[nodiscard]] std::string chain_xml(int dofs, int actuators, int welded) {
    std::string bodies;
    for (int i = 0; i < dofs; ++i)
        bodies += std::format(
            "<body name=\"link{0}\" pos=\"{0} 0 0\">"
            "<joint name=\"joint{0}\" type=\"slide\" axis=\"1 0 0\"/>"
            "<geom type=\"sphere\" size=\"0.1\" mass=\"0.5\"/></body>",
            i);
    for (int i = 0; i < welded; ++i)
        bodies += std::format(
            "<body name=\"weld{0}\" pos=\"0 0 {1}\">"
            "<geom type=\"box\" size=\"0.1 0.1 0.1\"/></body>",
            i, -1 - i);
    std::string motors;
    for (int i = 0; i < (dofs != 0 ? actuators : 0); ++i)
        motors += std::format(
            "<motor name=\"motor{0}\" joint=\"joint{1}\"/>", i, i % dofs);
    const std::string actuator_xml =
        motors.empty() ? std::string{} : "<actuator>" + motors + "</actuator>";
    return "<mujoco><worldbody>" + bodies + "</worldbody>" + actuator_xml +
           "</mujoco>";
}

[[nodiscard]] bike::model::ModelOwner chain_model(int dofs, int actuators = 1,
                                                  int welded = 0) {
    return bike::model::compile_mjcf(chain_xml(dofs, actuators, welded));
}

// Model whose mjData allocation the engine boundary must reject: nsample>0
// gives nhistory>0; a nonpositive timestep then trips the guarded makeData
// precondition — the case that would reach MuJoCo's fatal 'history buffers
// require positive timestep' without the E1 check.
[[nodiscard]] bike::model::ModelOwner history_model() {
    auto model = bike::model::compile_mjcf(
        "<mujoco><worldbody><body name=\"link\">"
        "<joint name=\"j0\" type=\"slide\"/>"
        "<geom type=\"sphere\" size=\"0.1\" mass=\"0.5\"/></body></worldbody>"
        "<sensor><jointpos joint=\"j0\" nsample=\"3\"/></sensor></mujoco>");
    model->opt.timestep = -0.5;
    return model;
}

// ---------------------------------------------------------------------------
// BIKEST02 fixture serialization — serialize_state/_write_bundle twins
// ---------------------------------------------------------------------------

// np.linspace semantics: y[i] = start + i*step, step = (stop-start)/(n-1),
// last element pinned to `stop`; n==1 returns [start].
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

struct StateArrays {
    double time = 0.25;
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> act;
    std::vector<double> ctrl;
    std::vector<double> qfrc;
    std::vector<double> xfrc;
    std::vector<double> qacc;
};

// _state_arrays: deterministic payload arrays for the model's dimensions —
// no MjData needed, so models whose data allocation would fail can still
// produce fixtures.
[[nodiscard]] StateArrays state_arrays(const mjModel &model,
                                       bool nonfinite = false) {
    const std::size_t nq = static_cast<std::size_t>(model.nq);
    const std::size_t nv = static_cast<std::size_t>(model.nv);
    const std::size_t na = static_cast<std::size_t>(model.na);
    const std::size_t nu = static_cast<std::size_t>(model.nu);
    const std::size_t nbody = static_cast<std::size_t>(model.nbody);
    StateArrays arrays;
    arrays.qpos = linspace(0.01, 0.02, nq);
    if (nonfinite) {
        if (nq != 0)
            arrays.qpos.front() = std::numeric_limits<double>::quiet_NaN();
        else
            arrays.qpos = {std::numeric_limits<double>::quiet_NaN()};
        if (nq == 0) arrays.time = std::numeric_limits<double>::quiet_NaN();
    }
    arrays.qvel = linspace(-0.01, 0.01, nv);
    arrays.act.assign(na, 0.0);
    arrays.ctrl = linspace(0.1, 0.2, nu);
    arrays.qfrc = linspace(0.001, 0.002, nv);
    arrays.xfrc.assign(6 * nbody, 0.0);
    arrays.qacc = linspace(0.0, 0.004, nv);
    return arrays;
}

void put_u32(std::vector<char> &out, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFFU));
}

void put_u64(std::vector<char> &out, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFFU));
}

void put_i32(std::vector<char> &out, std::int32_t value) {
    put_u32(out, static_cast<std::uint32_t>(value));
}

void append_bytes(std::vector<char> &out, std::span<const std::byte> bytes) {
    out.reserve(out.size() + bytes.size());
    for (const std::byte element : bytes)
        out.push_back(static_cast<char>(element));
}

void put_f64(std::vector<char> &out, double value) {
    append_bytes(out,
                 std::as_bytes(std::span{&value, std::size_t{1}}));
}

void append_doubles(std::vector<char> &out, std::span<const double> values) {
    append_bytes(out, std::as_bytes(values));
}

// serialize_state: dims are derived from the payload arrays (the
// _dim_from_arrays contract), then header + payload in section order
// time, qpos, qvel, act, ctrl, qfrc_applied, xfrc_applied, qacc_warmstart.
[[nodiscard]] std::vector<char>
serialize_state(const StateArrays &arrays,
                std::span<const std::byte> model_bytes) {
    bt::require(arrays.xfrc.size() % 6 == 0,
                "xfrc_applied element count is a multiple of 6");
    const std::size_t nq = arrays.qpos.size();
    const std::size_t nv = arrays.qvel.size();
    const std::size_t na = arrays.act.size();
    const std::size_t nu = arrays.ctrl.size();
    const std::size_t nbody = arrays.xfrc.size() / 6;
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
    bt::require(payload.size() == 8 * (1 + nq + 3 * nv + na + nu + 6 * nbody),
                "payload assembly drifted from dimensions");
    const std::uint64_t model_crc =
        proto_native_bench::detail::crc64_ecma(model_bytes);
    const std::uint64_t state_crc = proto_native_bench::detail::crc64_ecma(
        std::as_bytes(std::span{payload}));
    std::vector<char> image;
    image.reserve(kHeaderBytes + payload.size());
    image.insert(image.end(), {'B', 'I', 'K', 'E', 'S', 'T', '0', '2'});
    put_u32(image, 2); // version
    put_u32(image, 0); // flags
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

// _legacy_state_bytes: the retired 0xBEA0 payload — kept only to prove the
// reader rejects it. '<6i' header (magic + five dims), then the double
// sections in the retired order (no qacc_warmstart member existed).
[[nodiscard]] std::vector<char> legacy_state_bytes(const mjModel &model) {
    std::vector<char> image;
    for (const std::int32_t field : {kLegacyStateMagic,
                                     static_cast<std::int32_t>(model.nq),
                                     static_cast<std::int32_t>(model.nv),
                                     static_cast<std::int32_t>(model.na),
                                     static_cast<std::int32_t>(model.nu),
                                     static_cast<std::int32_t>(model.nbody)})
        put_i32(image, field);
    put_f64(image, 0.25);
    append_doubles(image, linspace(0.01, 0.02,
                                   static_cast<std::size_t>(model.nq)));
    append_doubles(image, linspace(-0.01, 0.01,
                                   static_cast<std::size_t>(model.nv)));
    append_doubles(image, std::vector<double>(
                              static_cast<std::size_t>(model.na), 0.0));
    append_doubles(image, linspace(0.1, 0.2,
                                   static_cast<std::size_t>(model.nu)));
    append_doubles(image, linspace(0.001, 0.002,
                                   static_cast<std::size_t>(model.nv)));
    append_doubles(image, std::vector<double>(
                              6 * static_cast<std::size_t>(model.nbody), 0.0));
    return image;
}

// _manifest_stub: minimal manifest for synthetic bundles — the reader only
// requires a readable file; real generation provenance is covered by the
// tracked artifacts and dump_model.py.
[[nodiscard]] std::string manifest_stub(const mjModel &model) {
    return std::format(
        "{{\"format\": 1, \"model\": {{\"nq\": {}, \"nv\": {}, \"na\": {}, "
        "\"nu\": {}, \"nbody\": {}, \"timestep\": {:.17g}}}}}",
        model.nq, model.nv, model.na, model.nu, model.nbody,
        static_cast<double>(model.opt.timestep));
}

void write_bytes(const std::filesystem::path &path,
                 std::span<const char> bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    bt::require(file.is_open(), "fixture write opens " + path.string());
    if (!bytes.empty())
        file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    bt::require(file.good(), "fixture write completes " + path.string());
}

void write_bytes(const std::filesystem::path &path,
                 std::span<const std::byte> bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    bt::require(file.is_open(), "fixture write opens " + path.string());
    for (const std::byte element : bytes)
        file.put(static_cast<char>(element));
    bt::require(file.good(), "fixture write completes " + path.string());
}

void write_text(const std::filesystem::path &path, std::string_view text) {
    write_bytes(path, std::span{text.data(), text.size()});
}

// np.savez equivalent: a genuine STORED-member ZIP32 archive of the eight
// payload arrays (PAYLOAD_KEYS order, xfrc as (nbody, 6)).
void write_npz(const std::filesystem::path &path, const StateArrays &arrays) {
    bike::io::NpzArchive archive;
    const auto member = [&archive](std::string name,
                                   std::span<const double> values,
                                   std::vector<std::size_t> shape) {
        archive.emplace_back(std::move(name),
                             bike::io::double_array(values, std::move(shape)));
    };
    member("time", std::span{&arrays.time, std::size_t{1}}, {1});
    member("qpos", arrays.qpos, {arrays.qpos.size()});
    member("qvel", arrays.qvel, {arrays.qvel.size()});
    member("act", arrays.act, {arrays.act.size()});
    member("ctrl", arrays.ctrl, {arrays.ctrl.size()});
    member("qfrc_applied", arrays.qfrc, {arrays.qfrc.size()});
    member("xfrc_applied", arrays.xfrc,
           {arrays.xfrc.size() / 6, std::size_t{6}});
    member("qacc_warmstart", arrays.qacc, {arrays.qacc.size()});
    write_bytes(path, bike::io::encode_npz(archive));
}

struct BundleSpec {
    std::optional<std::vector<char>> state;
    bool nonfinite = false;
    // _generated_bundle snapshots a fresh MjData: time stays 0.0; every
    // other field equals the _state_arrays defaults.
    bool snapshot_time_zero = false;
};

// _write_bundle: a concrete bundle directory with model.mjb + state.bin +
// state.npz + manifest.json, self-consistent unless state/nonfinite corrupt
// it. `root` IS the bundle directory, matching the Python helper.
[[nodiscard]] std::filesystem::path
write_bundle(const std::filesystem::path &root, const mjModel &model,
             const BundleSpec &spec = {}) {
    std::filesystem::create_directories(root);
    const std::filesystem::path model_path = root / "model.mjb";
    mj_saveModel(&model, model_path.string().c_str(), nullptr, 0);
    const std::string model_bytes = bt::text(model_path);
    StateArrays arrays = state_arrays(model, spec.nonfinite);
    if (spec.snapshot_time_zero) arrays.time = 0.0;
    const std::vector<char> state =
        spec.state.has_value()
            ? *spec.state
            : serialize_state(arrays,
                              std::as_bytes(std::span{model_bytes}));
    write_bytes(root / "state.bin", state);
    write_npz(root / "state.npz", arrays);
    write_text(root / "manifest.json", manifest_stub(model));
    return root;
}

// _generated_bundle: a versioned bundle through the production layout —
// root/<name>/ members plus the root/current.json pointer publication.
// The bundle path is returned for callers that run against it directly;
// pointer-root tests call it only for the side effect.
std::filesystem::path
generated_bundle(const std::filesystem::path &root, const mjModel &model) {
    std::filesystem::create_directories(root);
    const std::filesystem::path bundle =
        write_bundle(root / "test-bundle", model,
                     {.state = std::nullopt,
                      .nonfinite = false,
                      .snapshot_time_zero = true});
    write_text(root / "current.json",
               "{\"format\":1,\"bundle\":\"test-bundle\"}");
    return bundle;
}

void rewrite_state(const std::filesystem::path &bundle,
                   const std::function<std::vector<char>(std::string_view)> &transform) {
    const std::filesystem::path path = bundle / "state.bin";
    const std::string raw = bt::text(path);
    write_bytes(path, transform(raw));
}

void patch_u64(std::string &raw, std::size_t offset, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        raw.at(offset + shift / 8) =
            static_cast<char>((value >> shift) & 0xFFU);
}

void patch_u32(std::string &raw, std::size_t offset, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
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

[[nodiscard]] std::vector<char> patched_u32(std::string_view raw,
                                            std::size_t offset,
                                            std::uint32_t value) {
    std::string copy(raw);
    patch_u32(copy, offset, value);
    return {copy.begin(), copy.end()};
}

[[nodiscard]] mjtSize model_dim(const mjModel &model, std::string_view name) {
    if (name == "nq") return model.nq;
    if (name == "nv") return model.nv;
    if (name == "na") return model.na;
    if (name == "nu") return model.nu;
    bt::require(name == "nbody", "state dimension is one of the five header fields");
    return model.nbody;
}

[[nodiscard]] std::size_t dim_offset(std::string_view name) {
    if (name == "nq") return kHdrDimNq;
    if (name == "nv") return kHdrDimNv;
    if (name == "na") return kHdrDimNa;
    if (name == "nu") return kHdrDimNu;
    bt::require(name == "nbody", "state dimension is one of the five header fields");
    return kHdrDimNbody;
}

// ---------------------------------------------------------------------------
// Emitted JSON report accessors + Python-equality comparison
// ---------------------------------------------------------------------------

// Required-member access: an absent member is a structural contract
// violation — the [[noreturn]] spelling keeps the failure loud AND the
// pointer proof explicit (Python dict[key] raises KeyError the same way).
[[noreturn]] void report_missing(std::string_view field) {
    throw std::runtime_error("benchmark report member " + std::string(field) +
                             " is missing");
}

[[nodiscard]] const Value &member(const Value::Object &object,
                                  std::string_view key, std::string_view path) {
    const Value *found = bike::io::find_member(object, key);
    if (found == nullptr)
        report_missing(std::string(path) + "." + std::string(key));
    // report_missing is [[noreturn]] — the dereference is unreachable on the
    // null path.
    // cppcheck-suppress nullPointerRedundantCheck
    return *found;
}

[[nodiscard]] const Value &member(const Value &value, std::string_view key,
                                  std::string_view path) {
    return member(bike::io::as_object(value, path), key, path);
}

[[nodiscard]] double report_double(const Value &value,
                                   std::string_view path) {
    return bike::io::number_double(bike::io::as_number(value, path), path);
}

[[nodiscard]] std::int64_t report_int(const Value &value,
                                      std::string_view path) {
    return bike::io::number_i64(bike::io::as_number(value, path), path);
}

// Python `==` between two decoded JSON numbers: integer lexemes compare as
// integers; anything else compares as binary64 values (CPython float()
// coercion). A lexeme that cannot produce a binary64 fails the equality.
[[nodiscard]] bool numeric_equal(const Number &a, const Number &b) {
    if (bike::io::is_integer_lexeme(a.lexeme) &&
        bike::io::is_integer_lexeme(b.lexeme)) {
        std::int64_t ai = 0;
        std::int64_t bi = 0;
        const char *aend = std::to_address(a.lexeme.cend());
        const char *bend = std::to_address(b.lexeme.cend());
        const auto pa = std::from_chars(a.lexeme.data(), aend, ai);
        const auto pb = std::from_chars(b.lexeme.data(), bend, bi);
        if (pa.ec == std::errc() && pa.ptr == aend && pb.ec == std::errc() &&
            pb.ptr == bend)
            return ai == bi;
    }
    bool oka = false;
    bool okb = false;
    const double da = bike::io::detail::float_from_lexeme(a.lexeme, oka);
    const double db = bike::io::detail::float_from_lexeme(b.lexeme, okb);
    return oka && okb && da == db;
}

// Python `==` across the decoded value tree: dict==dict needs identical key
// sets and == values; list==list is elementwise; numbers compare by VALUE
// (int and float spellings are interchangeable); strings/bool/null compare
// structurally. Bytes/Tagged never appear in native_bench reports.
[[nodiscard]] bool python_equal(const Value &a, const Value &b) {
    if (const auto *na = std::get_if<Number>(&a.data)) {
        if (const auto *nb = std::get_if<Number>(&b.data))
            return numeric_equal(*na, *nb);
        if (const auto *bb = std::get_if<bool>(&b.data)) {
            const Number boolean{.lexeme = *bb ? "1" : "0"};
            return numeric_equal(*na, boolean);
        }
        return false;
    }
    if (const auto *ba = std::get_if<bool>(&a.data)) {
        if (const auto *bb = std::get_if<bool>(&b.data)) return *ba == *bb;
        if (const auto *nb = std::get_if<Number>(&b.data)) {
            const Number boolean{.lexeme = *ba ? "1" : "0"};
            return numeric_equal(boolean, *nb);
        }
        return false;
    }
    if (const auto *sa = std::get_if<std::string>(&a.data)) {
        const auto *sb = std::get_if<std::string>(&b.data);
        return sb != nullptr && *sa == *sb;
    }
    if (std::holds_alternative<std::monostate>(a.data))
        return std::holds_alternative<std::monostate>(b.data);
    if (const auto *aa = std::get_if<Value::Array>(&a.data)) {
        const auto *ab = std::get_if<Value::Array>(&b.data);
        if (ab == nullptr || aa->size() != ab->size()) return false;
        for (std::size_t i = 0; i < aa->size(); ++i)
            if (!python_equal((*aa)[i], (*ab)[i])) return false;
        return true;
    }
    if (const auto *oa = std::get_if<Value::Object>(&a.data)) {
        const auto *ob = std::get_if<Value::Object>(&b.data);
        if (ob == nullptr || oa->size() != ob->size()) return false;
        for (const auto &[key, value] : *oa) {
            const Value *other = bike::io::find_member(*ob, key);
            if (other == nullptr || !python_equal(value, *other)) return false;
        }
        return true;
    }
    return false;
}

// np.isclose(a, b, rtol=1e-12, atol=1e-12) — never a bitwise shortcut.
[[nodiscard]] bool isclose(double a, double b) {
    return std::abs(a - b) <= 1e-12 + 1e-12 * std::abs(b);
}

[[nodiscard]] Value emit_report(const bt::TemporaryDirectory &dir,
                                const std::string &mode,
                                const std::filesystem::path &bundle) {
    const std::filesystem::path emit = dir.path() / ("native_" + mode + ".json");
    const BenchResult result =
        run_bench({mode, "6", "2", "--artifacts", bundle.string(),
                   "--emit-json", emit.string()});
    expect_exit(result, 0, mode + " emit-json run");
    return bike::io::parse_json(bt::text(emit));
}

[[nodiscard]] bt::oracle::Node conformance_node(std::string_view mode) {
    return bt::oracle::Node(
        std::string(kNodeFile) +
        "::test_matched_workload_python_native_conformance[" +
        std::string(mode) + "]");
}

// ---------------------------------------------------------------------------
// CLI contract — test_native_benchmark.py lines 261-365
// ---------------------------------------------------------------------------

// tests/reference/test_native_benchmark.py::test_help_prints_usage
void test_help_prints_usage() {
    const BenchResult result = run_bench({"--help"});
    expect_exit(result, 0, "--help");
    bt::require(contains(lowercase(result.out), "usage"),
                "--help prints usage text on stdout");
}

// tests/reference/test_native_benchmark.py::test_selftest_prints_crc64_check_vector
void test_selftest_prints_crc64_check_vector() {
    // CRC64-ECMA('123456789') == 0x6C40DF5F0B497347 — cross-checks the
    // native implementation against the published parameter set.
    const BenchResult result = run_bench({"--selftest"});
    expect_exit(result, 0, "--selftest");
    bt::require(contains(result.out, "0x6c40df5f0b497347"),
                "--selftest prints the CRC64-ECMA check vector");
}

// tests/reference/test_native_benchmark.py::test_mode_is_required
void test_mode_is_required() {
    const BenchResult result = run_bench({});
    expect_exit(result, 1, "no arguments");
    expect_stderr_any(result, {"mode"}, "missing mode diagnostic");
}

// tests/reference/test_native_benchmark.py::test_unknown_mode_is_rejected[Glue|spin|0|--mode]
void test_unknown_mode_is_rejected() {
    for (const std::string mode : {"Glue", "spin", "0", "--mode"}) {
        const bt::TemporaryDirectory dir("bench_mode");
        const BenchResult result =
            run_bench({mode, "--artifacts", dir.path().string()});
        expect_exit(result, 1, "unknown mode '" + mode + "'");
        bt::require(!result.err.empty(),
                    "unknown mode '" + mode + "' writes a diagnostic");
    }
}

// tests/reference/test_native_benchmark.py::test_malformed_steps_are_rejected[abc|1x|1.5|0|-3|+5|10000001|99999999999999999999]
void test_malformed_steps_are_rejected() {
    const std::vector<std::string> cases{
        "abc", "1x", "1.5", "0", "-3", "+5",
        std::to_string(kMaxCount + 1), "99999999999999999999"};
    for (const std::string &steps : cases) {
        const bt::TemporaryDirectory dir("bench_steps");
        const BenchResult result =
            run_bench({"bare", steps, "--artifacts", dir.path().string()});
        expect_exit(result, 1, "malformed steps '" + steps + "'");
        expect_stderr_any(result, {"steps"},
                          "malformed steps '" + steps + "' diagnostic");
    }
}

// tests/reference/test_native_benchmark.py::test_invalid_rewind_is_rejected[0|-1|abc|1x|1.5|+2|10000001]
void test_invalid_rewind_is_rejected() {
    const std::vector<std::string> cases{"0", "-1", "abc", "1x",
                                         "1.5", "+2",
                                         std::to_string(kMaxCount + 1)};
    for (const std::string &rewind : cases) {
        const bt::TemporaryDirectory dir("bench_rewind");
        const BenchResult result = run_bench(
            {"glue", "2", rewind, "--artifacts", dir.path().string()});
        expect_exit(result, 1, "invalid rewind '" + rewind + "'");
        expect_stderr_any(result, {"rewind"},
                          "invalid rewind '" + rewind + "' diagnostic");
    }
}

// tests/reference/test_native_benchmark.py::test_missing_artifacts_flag_is_rejected
void test_missing_artifacts_flag_is_rejected() {
    const BenchResult result = run_bench({"bare", "5", "2"});
    expect_exit(result, 1, "missing --artifacts flag");
    expect_stderr_any(result, {"artifacts"}, "missing --artifacts diagnostic");
}

// tests/reference/test_native_benchmark.py::test_artifacts_requires_a_directory
void test_artifacts_requires_a_directory() {
    const BenchResult result = run_bench({"bare", "--artifacts"});
    expect_exit(result, 1, "--artifacts without a value");
    expect_stderr_any(result, {"artifacts"}, "--artifacts value diagnostic");
}

// tests/reference/test_native_benchmark.py::test_artifacts_must_be_a_directory
void test_artifacts_must_be_a_directory(const bt::TemporaryDirectory &dir) {
    const std::filesystem::path not_a_dir = dir.path() / "plain.bin";
    write_bytes(not_a_dir, std::span{std::string_view{"x"}});
    const BenchResult result =
        run_bench({"bare", "--artifacts", not_a_dir.string()});
    expect_exit(result, 1, "--artifacts naming a regular file");
    expect_stderr_any(result, {"model.mjb"},
                      "regular-file root fails at model.mjb");
}

// tests/reference/test_native_benchmark.py::test_unknown_option_is_rejected
void test_unknown_option_is_rejected(const bt::TemporaryDirectory &dir) {
    const BenchResult result = run_bench(
        {"bare", "--speed", "--artifacts", dir.path().string()});
    expect_exit(result, 1, "unknown option '--speed'");
    expect_stderr_any(result, {"option", "unknown"},
                      "unknown option diagnostic");
}

// tests/reference/test_native_benchmark.py::test_duplicate_artifacts_flag_is_rejected
void test_duplicate_artifacts_flag_is_rejected(const bt::TemporaryDirectory &dir) {
    const BenchResult result =
        run_bench({"bare", "--artifacts", dir.path().string(), "--artifacts",
                   dir.path().string()});
    expect_exit(result, 1, "duplicate --artifacts flag");
}

// tests/reference/test_native_benchmark.py::test_extra_positional_is_rejected
void test_extra_positional_is_rejected(const bt::TemporaryDirectory &dir) {
    const BenchResult result = run_bench(
        {"bare", "1", "2", "3", "--artifacts", dir.path().string()});
    expect_exit(result, 1, "extra positional argument");
}

// ---------------------------------------------------------------------------
// Artifact root / pointer / bundle-member validation — lines 372-511
// ---------------------------------------------------------------------------

// tests/reference/test_native_benchmark.py::test_missing_state_bin_is_rejected
void test_missing_state_bin_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    mj_saveModel(model.get(), (dir.path() / "model.mjb").string().c_str(),
                 nullptr, 0);
    const BenchResult result =
        run_bench({"bare", "--artifacts", dir.path().string()});
    expect_exit(result, 1, "bundle without state.bin");
    expect_stderr_any(result, {"state.bin"}, "missing state.bin diagnostic");
}

// tests/reference/test_native_benchmark.py::test_missing_state_npz_is_rejected
void test_missing_state_npz_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    std::filesystem::remove(bundle / "state.npz");
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "bundle without state.npz");
    expect_stderr_any(result, {"state.npz"}, "missing state.npz diagnostic");
}

// tests/reference/test_native_benchmark.py::test_missing_manifest_is_rejected
void test_missing_manifest_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    std::filesystem::remove(bundle / "manifest.json");
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "bundle without manifest.json");
    expect_stderr_any(result, {"manifest.json"},
                      "missing manifest.json diagnostic");
}

// tests/reference/test_native_benchmark.py::test_missing_model_mjb_is_rejected
void test_missing_model_mjb_is_rejected(const bt::TemporaryDirectory &dir) {
    write_bytes(dir.path() / "state.bin", std::span{std::string_view{"x"}});
    const BenchResult result =
        run_bench({"bare", "--artifacts", dir.path().string()});
    expect_exit(result, 1, "bundle without model.mjb");
    expect_stderr_any(result, {"model.mjb"}, "missing model.mjb diagnostic");
}

// tests/reference/test_native_benchmark.py::test_pointer_root_resolves_to_bundle
void test_pointer_root_resolves_to_bundle(const bt::TemporaryDirectory &dir) {
    // --artifacts may name the artifacts ROOT: current.json is resolved once.
    const auto model = chain_model(3);
    generated_bundle(dir.path() / "artifacts", *model);
    const BenchResult result = run_bench(
        {"bare", "3", "2", "--artifacts", (dir.path() / "artifacts").string()});
    expect_exit(result, 0, "artifacts root resolves through current.json");
    bt::require(contains(result.out, "mode=bare"),
                "pointer-root run reports mode=bare");
}

// tests/reference/test_native_benchmark.py::test_concrete_bundle_dir_resolves_directly
void test_concrete_bundle_dir_resolves_directly(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(3);
    const auto bundle = generated_bundle(dir.path() / "artifacts", *model);
    const BenchResult result = run_bench(
        {"glue", "3", "2", "--artifacts", bundle.string()});
    expect_exit(result, 0, "concrete bundle dir resolves directly");
    bt::require(contains(result.out, "mode=glue"),
                "concrete-dir run reports mode=glue");
}

// tests/reference/test_native_benchmark.py::test_malformed_pointer_is_rejected
void test_malformed_pointer_is_rejected(const bt::TemporaryDirectory &dir) {
    // A half-written current.json must fail loudly, never fall back to a
    // guess — interrupted publication is part of the contract.
    const auto model = chain_model(2);
    generated_bundle(dir.path() / "artifacts", *model);
    write_text(dir.path() / "artifacts" / "current.json", "{not json");
    const BenchResult result = run_bench(
        {"bare", "--artifacts", (dir.path() / "artifacts").string()});
    expect_exit(result, 1, "malformed current.json");
    expect_stderr_any(result, {"current.json"},
                      "malformed pointer diagnostic");
}

// tests/reference/test_native_benchmark.py::test_pointer_to_missing_bundle_is_rejected
void test_pointer_to_missing_bundle_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    generated_bundle(dir.path() / "artifacts", *model);
    write_text(dir.path() / "artifacts" / "current.json",
               "{\"format\": 1, \"bundle\": \"deleted-generation\"}");
    const BenchResult result = run_bench(
        {"bare", "--artifacts", (dir.path() / "artifacts").string()});
    expect_exit(result, 1, "pointer to a missing bundle dir");
    expect_stderr_any(result, {"missing bundle"},
                      "missing bundle diagnostic");
}

// tests/reference/test_native_benchmark.py::test_pointer_name_cannot_escape_root
void test_pointer_name_cannot_escape_root(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    generated_bundle(dir.path() / "artifacts", *model);
    for (const std::string_view hostile : {"../escape", "/abs", "a/b"}) {
        write_text(dir.path() / "artifacts" / "current.json",
                   std::format("{{\"format\": 1, \"bundle\": \"{}\"}}",
                               hostile));
        const BenchResult result = run_bench(
            {"bare", "--artifacts", (dir.path() / "artifacts").string()});
        expect_exit(result, 1,
                    "hostile bundle name '" + std::string(hostile) + "'");
        expect_stderr_any(result, {"bundle name"},
                          "hostile bundle name '" + std::string(hostile) +
                              "' diagnostic");
    }
}

// tests/reference/test_native_benchmark.py::test_corrupt_model_mjb_is_rejected
void test_corrupt_model_mjb_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    // The MJB leading int (54321) guards the size block; corrupting it must
    // be a clean load failure through the engine boundary, not an abort.
    std::string raw = bt::text(bundle / "model.mjb");
    patch_u32(raw, 0, 42);
    write_bytes(bundle / "model.mjb", std::span{raw});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "corrupt model.mjb magic");
}

// tests/reference/test_native_benchmark.py::test_corrupt_model_dimensions_are_rejected
void test_corrupt_model_dimensions_are_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    // mjtSize nv slot in the MJB size block: 5-i32 header, nv is sizes[1].
    std::string raw = bt::text(bundle / "model.mjb");
    patch_u64(raw, 20 + 8, 9'999'999);
    write_bytes(bundle / "model.mjb", std::span{raw});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "corrupt model.mjb dimension");
}

// tests/reference/test_native_benchmark.py::test_make_data_failure_is_recoverable
void test_make_data_failure_is_recoverable(const bt::TemporaryDirectory &dir) {
    const auto model = history_model();
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "mj_makeData precondition failure");
    expect_stderr_any(result, {"timestep", "history"},
                      "makeData failure is a recoverable diagnostic");
}

// ---------------------------------------------------------------------------
// state.bin validation — lines 518-714
// ---------------------------------------------------------------------------

// tests/reference/test_native_benchmark.py::test_legacy_state_format_is_rejected_with_hint
void test_legacy_state_format_is_rejected_with_hint(const bt::TemporaryDirectory &dir) {
    // A 0xBEA0 file must be rejected with a regeneration instruction.
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model,
                                     {.state = legacy_state_bytes(*model)});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "legacy 0xBEA0 state.bin");
    expect_stderr_any(result, {"retired", "regenerate"},
                      "legacy state rejection names regeneration");
}

// tests/reference/test_native_benchmark.py::test_bad_state_magic_is_rejected
void test_bad_state_magic_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        std::vector<char> patched(raw.begin(), raw.end());
        std::ranges::fill_n(patched.begin(), 8, '\0');
        return patched;
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "corrupted state magic");
    expect_stderr_any(result, {"format", "magic"}, "bad magic diagnostic");
}

// tests/reference/test_native_benchmark.py::test_unknown_state_version_is_rejected
void test_unknown_state_version_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u32(raw, kHdrVersion, 7);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "unknown state version");
    expect_stderr_any(result, {"version"}, "state version diagnostic");
}

// tests/reference/test_native_benchmark.py::test_unknown_state_flags_are_rejected
void test_unknown_state_flags_are_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u32(raw, kHdrFlags, 1);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "unknown state flags");
    expect_stderr_any(result, {"flags"}, "state flags diagnostic");
}

// tests/reference/test_native_benchmark.py::test_truncated_state_is_rejected
void test_truncated_state_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        const std::string_view kept = raw.substr(0, raw.size() - 16);
        return std::vector<char>(kept.begin(), kept.end());
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "state payload truncated by 16 bytes");
}

// tests/reference/test_native_benchmark.py::test_truncated_state_header_is_rejected
void test_truncated_state_header_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        const std::string_view kept = raw.substr(0, 20);
        return std::vector<char>(kept.begin(), kept.end());
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "state truncated inside the header");
    expect_stderr_any(result, {"truncated"}, "truncated header diagnostic");
}

// tests/reference/test_native_benchmark.py::test_trailing_state_bytes_are_rejected
void test_trailing_state_bytes_are_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        std::vector<char> patched(raw.begin(), raw.end());
        patched.insert(patched.end(), 8, '\0');
        return patched;
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "trailing bytes after the state payload");
    expect_stderr_any(result, {"expected"}, "trailing-bytes diagnostic");
}

// tests/reference/test_native_benchmark.py::test_huge_header_count_is_rejected
void test_huge_header_count_is_rejected(const bt::TemporaryDirectory &dir) {
    // u64-max nq overflows the signed mjtSize domain — rejected before any
    // allocation, never fed to a sized buffer.
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u64(raw, kHdrDimNq, 0xFFFFFFFFFFFFFFFFULL);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "u64-max nq header field");
    expect_stderr_any(result, {"domain", "does not match", "mismatch"},
                      "huge dimension diagnostic");
}

// tests/reference/test_native_benchmark.py::test_excessive_header_count_is_rejected
void test_excessive_header_count_is_rejected(const bt::TemporaryDirectory &dir) {
    // A huge positive count mismatches the model and must be rejected
    // before any buffer is sized — never fed to an allocation.
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u64(raw, kHdrDimNq, 2'000'000'000ULL);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "2e9 nq header field");
    expect_stderr_any(result, {"does not match", "mismatch"},
                      "excessive dimension diagnostic");
}

// tests/reference/test_native_benchmark.py::test_state_model_dimension_mismatch_is_rejected[nq|nv|na|nu|nbody]
void test_state_model_dimension_mismatch_is_rejected(const bt::TemporaryDirectory &dir) {
    for (const std::string_view label : {"nq", "nv", "na", "nu", "nbody"}) {
        const auto model = chain_model(4, 2);
        const auto bundle =
            write_bundle(dir.path() / std::string(label), *model);
        const std::uint64_t patched =
            static_cast<std::uint64_t>(model_dim(*model, label) + 1);
        rewrite_state(bundle, [patched, label](std::string_view raw) {
            return patched_u64(raw, dim_offset(label), patched);
        });
        const BenchResult result =
            run_bench({"bare", "--artifacts", bundle.string()});
        expect_exit(result, 1,
                    "state/model dimension mismatch on " + std::string(label));
        expect_stderr_any(result, {"does not match", "mismatch"},
                          "dimension mismatch on " + std::string(label));
    }
}

// tests/reference/test_native_benchmark.py::test_declared_payload_length_lie_is_rejected
void test_declared_payload_length_lie_is_rejected(const bt::TemporaryDirectory &dir) {
    // payload_bytes is independently mandatory: patching only the declared
    // field (file length still correct) must still fail.
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u64(raw, kHdrPayloadBytes, 8);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "declared payload_bytes lie");
    expect_stderr_any(result, {"expected", "does not match"},
                      "payload_bytes diagnostic");
}

// tests/reference/test_native_benchmark.py::test_payload_checksum_corruption_is_rejected
void test_payload_checksum_corruption_is_rejected(const bt::TemporaryDirectory &dir) {
    // Flipping one payload byte breaks state_crc64 — corruption caught
    // before decode, not by a downstream finiteness accident.
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        std::vector<char> patched(raw.begin(), raw.end());
        patched.back() = static_cast<char>(patched.back() ^ 0xFF);
        return patched;
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "payload byte flip");
    expect_stderr_any(result, {"checksum"}, "payload checksum diagnostic");
}

// tests/reference/test_native_benchmark.py::test_declared_state_checksum_lie_is_rejected
void test_declared_state_checksum_lie_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    rewrite_state(bundle, [](std::string_view raw) {
        return patched_u64(raw, kHdrStateCrc, 0);
    });
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "declared state_crc64 lie");
    expect_stderr_any(result, {"checksum"}, "state crc diagnostic");
}

// tests/reference/test_native_benchmark.py::test_state_from_other_generation_is_rejected
void test_state_from_other_generation_is_rejected(const bt::TemporaryDirectory &dir) {
    // Mixed-generation mixing: same dims, different model bytes — the
    // model_crc64 binds state.bin to its own model.mjb.
    const auto model = chain_model(3);
    const auto bundle = write_bundle(dir.path() / "main", *model);
    const auto other_model = chain_model(3);
    // same dims, different model bytes
    model_access::mutable_buffer(other_model->body_mass, other_model->nbody,
                                 "body_mass")[1] = 0.9;
    const auto other = write_bundle(dir.path() / "other", *other_model);
    const std::string foreign = bt::text(other / "state.bin");
    write_bytes(bundle / "state.bin", std::span{foreign});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "state.bin from another generation");
    expect_stderr_any(result, {"checksum"},
                      "cross-generation checksum diagnostic");
}

// tests/reference/test_native_benchmark.py::test_state_from_other_model_dims_is_rejected
void test_state_from_other_model_dims_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto other_model = chain_model(5);
    const auto other = write_bundle(dir.path() / "other", *other_model);
    const auto model = chain_model(3);
    const auto bundle = write_bundle(dir.path() / "main", *model);
    const std::string foreign = bt::text(other / "state.bin");
    write_bytes(bundle / "state.bin", std::span{foreign});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "state.bin from a different-dimension model");
}

// tests/reference/test_native_benchmark.py::test_nonfinite_state_scalar_is_rejected
void test_nonfinite_state_scalar_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle =
        write_bundle(dir.path() / "bundle", *model,
                     {.state = std::nullopt,
                      .nonfinite = true,
                      .snapshot_time_zero = false});
    const BenchResult result =
        run_bench({"bare", "--artifacts", bundle.string()});
    expect_exit(result, 1, "non-finite stored scalar");
    expect_stderr_any(result, {"finite"}, "finiteness diagnostic");
}

// ---------------------------------------------------------------------------
// Workload domains + valid controls — lines 721-901
// ---------------------------------------------------------------------------

// tests/reference/test_native_benchmark.py::test_bare_accepts_empty_model
void test_bare_accepts_empty_model(const bt::TemporaryDirectory &dir) {
    // nv=0/nu=0/nbody=1 — bare steps an empty model if the engine permits it.
    const auto model = chain_model(0, 0);
    bt::require(model->nv == 0 && model->nu == 0 && model->nbody == 1,
                "empty chain model has nv=0, nu=0, nbody=1");
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"bare", "3", "2", "--artifacts", bundle.string()});
    expect_exit(result, 0, "bare on the empty model");
    bt::require(contains(result.out, "mode=bare"),
                "empty-model run reports mode=bare");
    bt::require(contains(result.out, "nv=0"),
                "empty-model run reports nv=0");
}

// tests/reference/test_native_benchmark.py::test_non_bare_modes_reject_empty_model[glue|heavy]
void test_non_bare_modes_reject_empty_model(const bt::TemporaryDirectory &dir) {
    for (const std::string mode : {"glue", "heavy"}) {
        const auto model = chain_model(0, 0);
        const auto bundle =
            write_bundle(dir.path() / ("empty_" + mode), *model);
        const BenchResult result =
            run_bench({mode, "2", "--artifacts", bundle.string()});
        expect_exit(result, 1, mode + " on the empty model");
        expect_stderr_any(result, {"requires"},
                          mode + " empty-model rejection names the requirement");
    }
}

// tests/reference/test_native_benchmark.py::test_glue_rejects_no_actuators
void test_glue_rejects_no_actuators(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(4, 0);
    bt::require(model->nu == 0, "chain(4, actuators=0) has nu=0");
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"glue", "2", "--artifacts", bundle.string()});
    expect_exit(result, 1, "glue without actuators");
    expect_stderr_any(result, {"nu"}, "glue nu=0 rejection names nu");
}

// tests/reference/test_native_benchmark.py::test_heavy_rejects_too_few_attachment_bodies[6|9]
void test_heavy_rejects_too_few_attachment_bodies(const bt::TemporaryDirectory &dir) {
    // nbody = dofs + 1 < 11 — the heavy attachment list cannot resolve.
    for (const int dofs : {6, 9}) {
        const auto model = chain_model(dofs);
        const auto bundle =
            write_bundle(dir.path() / std::to_string(dofs), *model);
        const BenchResult result =
            run_bench({"heavy", "2", "--artifacts", bundle.string()});
        expect_exit(result, 1,
                    "heavy with nbody=" + std::to_string(dofs + 1));
        expect_stderr_any(result, {"nbody"},
                          "heavy body-count rejection names nbody");
    }
}

// tests/reference/test_native_benchmark.py::test_heavy_rejects_narrow_model
void test_heavy_rejects_narrow_model(const bt::TemporaryDirectory &dir) {
    // nbody >= 11 but nv < 3: welded shells add bodies without dofs.
    const auto model = chain_model(1, 1, 11);
    bt::require(model->nbody == 13 && model->nv == 1,
                "welded model has nbody=13, nv=1");
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"heavy", "2", "--artifacts", bundle.string()});
    expect_exit(result, 1, "heavy with nv=1");
    expect_stderr_any(result, {"nv"}, "heavy nv rejection names nv");
}

// tests/reference/test_native_benchmark.py::test_bare_dimension_sweep[0|1|33|65|128]
void test_bare_dimension_sweep(const bt::TemporaryDirectory &dir) {
    for (const int nv : {0, 1, 33, 65, 128}) {
        const auto model = chain_model(nv);
        const auto bundle =
            write_bundle(dir.path() / ("nv" + std::to_string(nv)), *model);
        const BenchResult result =
            run_bench({"bare", "3", "2", "--artifacts", bundle.string()});
        expect_exit(result, 0, "bare dimension sweep nv=" + std::to_string(nv));
        bt::require(contains(result.out, "nv=" + std::to_string(nv)),
                    "bare sweep run reports nv=" + std::to_string(nv));
    }
}

// tests/reference/test_native_benchmark.py::test_glue_dimension_sweep[1|33|65|128]
void test_glue_dimension_sweep(const bt::TemporaryDirectory &dir) {
    // nv=65 exceeds the prototype's fixed 64-element Jacobian cap; nv=33
    // crosses the 3*nv shared-buffer reuse defect. Both must succeed.
    for (const int nv : {1, 33, 65, 128}) {
        const auto model = chain_model(nv);
        const auto bundle =
            write_bundle(dir.path() / ("nv" + std::to_string(nv)), *model);
        const BenchResult result =
            run_bench({"glue", "3", "2", "--artifacts", bundle.string()});
        expect_exit(result, 0, "glue dimension sweep nv=" + std::to_string(nv));
        bt::require(contains(result.out, "nv=" + std::to_string(nv)),
                    "glue sweep run reports nv=" + std::to_string(nv));
    }
}

// tests/reference/test_native_benchmark.py::test_glue_with_nu_greater_than_nv
void test_glue_with_nu_greater_than_nv(const bt::TemporaryDirectory &dir) {
    // nu > nv: the actuator->velocity mapping must go through the
    // engine-projected qfrc_actuator (length nv), never qvel[:nu].
    const auto model = chain_model(2, 4);
    bt::require(model->nu > model->nv, "chain(2, actuators=4) has nu > nv");
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"glue", "3", "2", "--artifacts", bundle.string()});
    expect_exit(result, 0, "glue with nu > nv");
    bt::require(contains(result.out, "mode=glue"),
                "nu>nv run reports mode=glue");
    bt::require(!contains(result.out, "nan"),
                "nu>nv run reports no nan checksum");
}

// tests/reference/test_native_benchmark.py::test_heavy_nv33_paired_jacobians
void test_heavy_nv33_paired_jacobians(const bt::TemporaryDirectory &dir) {
    // ASan control: nv=33 makes the prototype's 64-double shared Jacobian
    // buffer overflow; model-sized separate buffers must not.
    const auto model = chain_model(33);
    const auto bundle = write_bundle(dir.path() / "nv33", *model);
    const BenchResult result =
        run_bench({"heavy", "3", "2", "--artifacts", bundle.string()});
    expect_exit(result, 0, "heavy nv=33 paired Jacobians");
    bt::require(contains(result.out, "nv=33"), "heavy run reports nv=33");
    bt::require(!contains(result.out, "nan"), "heavy run reports no nan");
}

// tests/reference/test_native_benchmark.py::test_heavy_boundary_model
void test_heavy_boundary_model(const bt::TemporaryDirectory &dir) {
    // nbody == 11 is the smallest supported heavy model.
    const auto model = chain_model(10);
    bt::require(model->nbody == 11, "chain(10) has nbody=11");
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const BenchResult result =
        run_bench({"heavy", "3", "2", "--artifacts", bundle.string()});
    expect_exit(result, 0, "heavy on the boundary model");
}

// tests/reference/test_native_benchmark.py::test_operation_mix_mode_runs
void test_operation_mix_mode_runs(const bt::TemporaryDirectory &dir) {
    // operation_mix keeps its R2 approximation — it runs, reports its own
    // claim, and is NOT among the parity-checked workloads.
    const auto model = chain_model(12);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const std::filesystem::path emit = dir.path() / "mix.json";
    const BenchResult result =
        run_bench({"operation_mix", "4", "2", "--artifacts", bundle.string(),
                   "--emit-json", emit.string()});
    expect_exit(result, 0, "operation_mix run");
    bt::require(contains(result.out, "mode=operation_mix"),
                "operation_mix reports its mode");
    bt::require(!contains(result.out, "nan"),
                "operation_mix reports no nan checksum");
    const Value report = bike::io::parse_json(bt::text(emit));
    bt::require(bike::io::as_string(member(report, "comparison", "report"),
                                    "report.comparison") == "operation_mix",
                "operation_mix report claims operation_mix comparison");
    const Value &workload = member(report, "workload", "report");
    bt::require(bike::io::as_string(member(workload, "claim", "report.workload"),
                                    "report.workload.claim") == "operation_mix",
                "operation_mix workload claims operation_mix");
}

// tests/reference/test_native_benchmark.py::test_tracked_artifacts_smoke[bare-args0|glue-args1|heavy-args2|operation_mix-args3]
void test_tracked_artifacts_smoke() {
    const std::filesystem::path tracked =
        repo_root() / "tools" / "proto_native_bench" / "artifacts";
    bt::require(std::filesystem::is_directory(tracked),
                "tracked artifacts bundle directory exists");
    struct Smoke {
        const char *mode;
        const char *steps;
        const char *rewind;
    };
    for (const Smoke smoke : {Smoke{"bare", "10", "5"},
                              Smoke{"glue", "5", "2"},
                              Smoke{"heavy", "3", "2"},
                              Smoke{"operation_mix", "4", "2"}}) {
        const BenchResult result = run_bench(
            {smoke.mode, smoke.steps, smoke.rewind, "--artifacts",
             tracked.string()});
        const std::string label =
            std::string("tracked smoke ") + smoke.mode;
        expect_exit(result, 0, label);
        bt::require(contains(result.out, "mode=" + std::string(smoke.mode)),
                    label + " reports its mode");
        bt::require(contains(result.out, "RTF="),
                    label + " reports the RTF line");
        bt::require(!contains(result.out, "nan"),
                    label + " reports no nan checksum");
    }
}

// tests/reference/test_native_benchmark.py::test_rewind_boundaries
void test_rewind_boundaries(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(3);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    // rewind == steps: restore runs exactly once; rewind > steps is fine.
    BenchResult result = run_bench(
        {"glue", "4", "4", "--artifacts", bundle.string()});
    expect_exit(result, 0, "rewind == steps");
    result = run_bench({"glue", "4", "99", "--artifacts", bundle.string()});
    expect_exit(result, 0, "rewind > steps");
}

// ---------------------------------------------------------------------------
// Emitted JSON report + paired conformance — lines 930-1027
// ---------------------------------------------------------------------------

// tests/reference/test_native_benchmark.py::test_emit_json_report_shape
void test_emit_json_report_shape(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(12);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const Value report = emit_report(dir, "glue", bundle);
    bt::require(report_int(member(report, "format", "report"),
                         "report.format") == 1,
                "report format is 1");
    bt::require(bike::io::as_string(member(report, "mode", "report"),
                                    "report.mode") == "glue",
                "report mode is glue");
    bt::require(bike::io::as_string(member(report, "comparison", "report"),
                                    "report.comparison") == "matched",
                "report comparison is matched");
    const Value &workload = member(report, "workload", "report");
    bt::require(
        bike::io::as_string(member(workload, "seed", "report.workload"),
                            "report.workload.seed") == "0x42f0e1eba9ea3693",
        "workload seed is the published splitmix64 seed");
    const Value &counts = member(report, "operation_counts", "report");
    bt::require(report_int(member(counts, "engine_steps", "operation_counts"),
                           "operation_counts.engine_steps") > 0 &&
                report_int(member(counts, "forwards", "operation_counts"),
                           "operation_counts.forwards") > 0,
                "engine_steps and forwards are positive");
    bt::require(report_int(member(counts, "jacobian_calls", "operation_counts"),
                           "operation_counts.jacobian_calls") > 0 &&
                report_int(member(counts, "name_lookups", "operation_counts"),
                           "operation_counts.name_lookups") > 0,
                "jacobian_calls and name_lookups are positive");
    const Value &checksums = member(report, "stage_checksums", "report");
    const auto &checksum_object =
        bike::io::as_object(checksums, "stage_checksums");
    for (const std::string_view key : {"trajectory_time", "forces", "energy",
                                       "jacobians", "channels", "name_ids"})
        bt::require(bike::io::find_member(checksum_object, key) != nullptr,
                    "stage_checksums." + std::string(key) + " is present");
    // np.isfinite(report['checksum']): a nonfinite lexeme fails the
    // finite-double accessor instead of passing silently.
    (void)report_double(member(report, "checksum", "report"), "report.checksum");
    const Value &provenance = member(report, "provenance", "report");
    bt::require(bike::io::as_string(member(provenance, "model_crc64",
                                           "report.provenance"),
                                    "report.provenance.model_crc64")
                    .starts_with("0x"),
                "provenance.model_crc64 is a hex string");
}

// tests/reference/test_native_benchmark.py::test_emit_json_unwritable_path_is_rejected
void test_emit_json_unwritable_path_is_rejected(const bt::TemporaryDirectory &dir) {
    const auto model = chain_model(2);
    const auto bundle = write_bundle(dir.path() / "bundle", *model);
    const std::filesystem::path missing_dir =
        dir.path() / "no-such-dir" / "report.json";
    const BenchResult result =
        run_bench({"glue", "2", "2", "--artifacts", bundle.string(),
                   "--emit-json", missing_dir.string()});
    expect_exit(result, 1, "emit-json into a missing directory");
}

// tests/reference/test_native_benchmark.py::test_matched_workload_python_native_conformance[bare|glue|heavy]
void test_matched_workload_python_native_conformance(const bt::TemporaryDirectory &dir) {
    // The matched workloads must produce the same operation counts, the same
    // stage checksums, and solver outputs inside rtol=atol=1e-12 in both
    // languages. Exact-integer stages (name_ids) must be byte-equal; float
    // stages may differ only by platform libm noise bounded by the solver
    // tolerance. The oracle emission is replayed from the captured
    // input.report entry; every comparison below mirrors the pytest body.
    for (const std::string mode : {"bare", "glue", "heavy"}) {
        auto golden = conformance_node(mode);
        const std::vector<std::byte> recorded = golden.bytes_input("input.report");
        std::string recorded_text;
        recorded_text.reserve(recorded.size());
        for (const std::byte element : recorded)
            recorded_text.push_back(static_cast<char>(element));
        const Value python = bike::io::parse_json(recorded_text);

        const auto model = chain_model(12);
        const auto bundle =
            generated_bundle(dir.path() / ("artifacts_" + mode), *model);
        const Value native = emit_report(dir, mode, bundle);

        bt::require(
            bike::io::as_string(member(native, "comparison", "native"),
                                "native.comparison") == "matched",
            mode + ": native comparison is matched");
        bt::require(
            bike::io::as_string(member(python, "comparison", "python"),
                                "python.comparison") == "matched",
            mode + ": oracle comparison is matched");
        bt::require(python_equal(member(native, "workload", "native"),
                                 member(python, "workload", "python")),
                    mode + ": workload blocks are equal");
        bt::require(
            python_equal(member(native, "operation_counts", "native"),
                         member(python, "operation_counts", "python")),
            mode + ": operation_counts blocks are equal");
        const Value &native_prov = member(native, "provenance", "native");
        const Value &python_prov = member(python, "provenance", "python");
        bt::require(
            bike::io::as_string(
                member(native_prov, "model_crc64", "native.provenance"),
                "native.provenance.model_crc64") ==
                bike::io::as_string(
                    member(python_prov, "model_crc64", "python.provenance"),
                    "python.provenance.model_crc64"),
            mode + ": model_crc64 provenance matches the oracle");
        bt::require(
            bike::io::as_string(
                member(native_prov, "state_crc64", "native.provenance"),
                "native.provenance.state_crc64") ==
                bike::io::as_string(
                    member(python_prov, "state_crc64", "python.provenance"),
                    "python.provenance.state_crc64"),
            mode + ": state_crc64 provenance matches the oracle");

        const auto &native_ck = bike::io::as_object(
            member(native, "stage_checksums", "native"), "native.stage_checksums");
        const auto &python_ck = bike::io::as_object(
            member(python, "stage_checksums", "python"), "python.stage_checksums");
        bt::require(native_ck.size() == python_ck.size(),
                    mode + ": stage_checksums key counts agree");
        // Scalar stages that must be byte-equal.
        bt::require(
            python_equal(member(native_ck, "name_ids", "native.stage_checksums"),
                         member(python_ck, "name_ids", "python.stage_checksums")),
            mode + ": name_ids is exactly equal");
        bt::require(
            python_equal(member(native_ck, "trajectory_time",
                                "native.stage_checksums"),
                         member(python_ck, "trajectory_time",
                                "python.stage_checksums")),
            mode + ": trajectory_time is exactly equal");
        for (const auto &[key, native_value] : native_ck) {
            const Value &python_value =
                member(python_ck, key, "python.stage_checksums");
            const double nv =
                report_double(native_value, "native.stage_checksums." + key);
            const double pv =
                report_double(python_value, "python.stage_checksums." + key);
            bt::require(std::isfinite(nv) && std::isfinite(pv),
                        mode + ": stage_checksums." + key + " is finite");
            bt::require(isclose(nv, pv),
                        mode + ": stage_checksums[" + key + "]: " +
                            std::format("{:.17g}", nv) + " vs " +
                            std::format("{:.17g}", pv));
        }

        const auto &native_solve = bike::io::as_array(
            member(native, "solve_outputs", "native"), "native.solve_outputs");
        const auto &python_solve = bike::io::as_array(
            member(python, "solve_outputs", "python"), "python.solve_outputs");
        bt::require(native_solve.size() == python_solve.size(),
                    mode + ": solve_outputs lengths are equal");
        for (std::size_t i = 0; i < native_solve.size(); ++i) {
            const double nv = report_double(
                native_solve[i], "native.solve_outputs");
            const double pv = report_double(
                python_solve[i], "python.solve_outputs");
            bt::require(std::isfinite(nv),
                        mode + ": solve_outputs[" + std::to_string(i) +
                            "] is finite");
            bt::require(isclose(nv, pv),
                        mode + ": solve_outputs[" + std::to_string(i) +
                            "]: " + std::format("{:.17g}", nv) + " vs " +
                            std::format("{:.17g}", pv));
        }

        const double native_checksum =
            report_double(member(native, "checksum", "native"), "native.checksum");
        const double python_checksum =
            report_double(member(python, "checksum", "python"), "python.checksum");
        bt::require(std::isfinite(native_checksum) && std::isfinite(python_checksum),
                    mode + ": checksum is finite on both sides");
        bt::require(isclose(native_checksum, python_checksum),
                    mode + ": checksum: " +
                        std::format("{:.17g}", native_checksum) + " vs " +
                        std::format("{:.17g}", python_checksum));
        golden.done();
    }
}

// pytest tmp_path isolation: each test below gets a fresh directory —
// never a dir shared across tests, since several write members directly
// into dir.path() and leftovers would mask the missing-member checks.
template <typename Fn> void with_tmp_path(const char *label, Fn &&fn) {
    const bt::TemporaryDirectory dir(label);
    fn(dir);
}

void check() {
    // CLI contract
    test_help_prints_usage();
    test_selftest_prints_crc64_check_vector();
    test_mode_is_required();
    test_unknown_mode_is_rejected();
    test_malformed_steps_are_rejected();
    test_invalid_rewind_is_rejected();
    test_missing_artifacts_flag_is_rejected();
    test_artifacts_requires_a_directory();
    // Artifact root / pointer / bundle members
    with_tmp_path("bench_cli", test_artifacts_must_be_a_directory);
    with_tmp_path("bench_cli", test_unknown_option_is_rejected);
    with_tmp_path("bench_cli", test_duplicate_artifacts_flag_is_rejected);
    with_tmp_path("bench_cli", test_extra_positional_is_rejected);
    with_tmp_path("bench_bundle", test_missing_state_bin_is_rejected);
    with_tmp_path("bench_bundle", test_missing_state_npz_is_rejected);
    with_tmp_path("bench_bundle", test_missing_manifest_is_rejected);
    with_tmp_path("bench_bundle", test_missing_model_mjb_is_rejected);
    with_tmp_path("bench_bundle", test_pointer_root_resolves_to_bundle);
    with_tmp_path("bench_bundle", test_concrete_bundle_dir_resolves_directly);
    with_tmp_path("bench_bundle", test_malformed_pointer_is_rejected);
    with_tmp_path("bench_bundle", test_pointer_to_missing_bundle_is_rejected);
    with_tmp_path("bench_bundle", test_pointer_name_cannot_escape_root);
    with_tmp_path("bench_bundle", test_corrupt_model_mjb_is_rejected);
    with_tmp_path("bench_bundle", test_corrupt_model_dimensions_are_rejected);
    with_tmp_path("bench_bundle", test_make_data_failure_is_recoverable);
    // state.bin validation
    with_tmp_path("bench_state", test_legacy_state_format_is_rejected_with_hint);
    with_tmp_path("bench_state", test_bad_state_magic_is_rejected);
    with_tmp_path("bench_state", test_unknown_state_version_is_rejected);
    with_tmp_path("bench_state", test_unknown_state_flags_are_rejected);
    with_tmp_path("bench_state", test_truncated_state_is_rejected);
    with_tmp_path("bench_state", test_truncated_state_header_is_rejected);
    with_tmp_path("bench_state", test_trailing_state_bytes_are_rejected);
    with_tmp_path("bench_state", test_huge_header_count_is_rejected);
    with_tmp_path("bench_state", test_excessive_header_count_is_rejected);
    with_tmp_path("bench_state", test_state_model_dimension_mismatch_is_rejected);
    with_tmp_path("bench_state", test_declared_payload_length_lie_is_rejected);
    with_tmp_path("bench_state", test_payload_checksum_corruption_is_rejected);
    with_tmp_path("bench_state", test_declared_state_checksum_lie_is_rejected);
    with_tmp_path("bench_state", test_state_from_other_generation_is_rejected);
    with_tmp_path("bench_state", test_state_from_other_model_dims_is_rejected);
    with_tmp_path("bench_state", test_nonfinite_state_scalar_is_rejected);
    // Workload domains + valid controls
    with_tmp_path("bench_domains", test_bare_accepts_empty_model);
    with_tmp_path("bench_domains", test_non_bare_modes_reject_empty_model);
    with_tmp_path("bench_domains", test_glue_rejects_no_actuators);
    with_tmp_path("bench_domains", test_heavy_rejects_too_few_attachment_bodies);
    with_tmp_path("bench_domains", test_heavy_rejects_narrow_model);
    with_tmp_path("bench_domains", test_bare_dimension_sweep);
    with_tmp_path("bench_domains", test_glue_dimension_sweep);
    with_tmp_path("bench_domains", test_glue_with_nu_greater_than_nv);
    with_tmp_path("bench_domains", test_heavy_nv33_paired_jacobians);
    with_tmp_path("bench_domains", test_heavy_boundary_model);
    with_tmp_path("bench_domains", test_operation_mix_mode_runs);
    test_tracked_artifacts_smoke();
    with_tmp_path("bench_domains", test_rewind_boundaries);
    // Emitted JSON report + oracle conformance
    with_tmp_path("bench_report", test_emit_json_report_shape);
    with_tmp_path("bench_report", test_emit_json_unwritable_path_is_rejected);
    with_tmp_path("bench_report", test_matched_workload_python_native_conformance);
}

} // namespace

int main() { return bt::run(check); }
