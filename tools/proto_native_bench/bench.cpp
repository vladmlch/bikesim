// bench.cpp — native prototype benchmark for the bike_native Python-binding
// workload.
//
// A first stress-test build of a native implementation of the Python API
// being ported. The benchmark replays a recorded BIKEST02 state bundle
// through mj_step under selectable workloads:
//   - bare:  engine stepping only (frozen replayed state, periodic rewind)
//   - glue:  bare + the matched engine-call surface (force assemblies,
//            forward passes, paired point Jacobians, name lookups, energy
//            probe, channel writes) — mirrored stage-for-stage in bench.py
//   - heavy: glue + matched telemetry/accounting volume (efc-row unpack,
//            attachment wrench solves, energy ledger, record churn,
//            period-batch solve)
//   - operation_mix: the retained R2 approximation — same engine surface but
//            a per-step mix that intentionally does NOT match bench.py and
//            carries no parity claim
//
// bare/glue/heavy are the *matched* workloads: both languages execute the
// same warmup/rewind phases, the same stage order, the same operation
// counts, the same attachment Jacobian + regularized 3x3 solve sequence, and
// the same actuator->velocity mapping (the engine-projected qfrc_actuator —
// never qvel[:nu] slicing). --emit-json writes operation counts, per-stage
// checksum contributions, solver outputs, and provenance; the conformance
// suite compares them against bench.py's emission.
//
// Every engine call crosses the bike_native_engine boundary (E1) so MuJoCo
// warnings/fatals surface as recoverable EngineFailure exceptions; the CLI
// reports them and returns nonzero instead of terminating. All extents are
// validated (bundle resolution, state header vs. model, checksums, workload
// preconditions, checked products/sums) before any buffer is sized or
// copied.
//
// See README.md for the build, CLI syntax, bundle layout, and the honest
// list of comparison-claim boundaries.
//
// Artifact resolution: --artifacts names an artifacts root (a current.json
// pointer resolved once to a versioned bundle directory) or a concrete
// bundle directory containing model.mjb + state.npz + state.bin +
// manifest.json directly.
#include <mujoco/mujoco.h>

#include "../../native/src/engine_call.hpp"
#include "../../native/src/model_access.hpp"
#include "state_format.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

using proto_native_bench::State;

constexpr std::string_view kUsage =
    "usage: native_bench {bare,glue,heavy,operation_mix} [steps] [rewind] "
    "--artifacts <directory> [--emit-json <path>]\n"
    "  bare:  mj_step only (frozen replayed state, periodic rewind)\n"
    "  glue:  bare + matched engine-call surface (8 qfrc assemblies, forward\n"
    "         passes, paired point Jacobians, name lookups, 100 channels)\n"
    "  heavy: glue + matched telemetry volume (efc unpack, 5 attachment\n"
    "         wrench solves, energy ledger, record churn, batch solve)\n"
    "  operation_mix: retained R2 approximation mix — no parity claim\n"
    "  steps/rewind: positive integers in [1, 10000000] "
    "(defaults 4000 and 250)\n"
    "  --artifacts: artifacts root (current.json pointer) or a concrete\n"
    "         bundle dir holding model.mjb/state.npz/state.bin/manifest.json\n"
    "  --emit-json: write the operation/checksum/provenance report\n"
    "  --selftest: print the CRC64-ECMA check vector and exit\n";

enum class Mode : std::uint8_t { bare, glue, heavy, operation_mix };

// Hard bound on step/rewind counts: keeps count * sizeof(double)-style
// products and loop planning inside checked-arithmetic range and rejects
// absurd run lengths early.
constexpr long kMaxCount = 10'000'000;
constexpr long kDefaultSteps = 4'000;
constexpr long kDefaultRewind = 250;

// Canonical matched-workload constants — mirrored in bench.py and recorded
// verbatim in the emitted workload JSON (and in the bundle manifest's
// "workload" block). The seed feeds a splitmix64 generator implemented
// identically in both languages.
constexpr long kWarmupSteps = 500;
constexpr std::size_t kWriterCount = 8;
constexpr std::size_t kChannelCount = 100;
constexpr std::size_t kAttachmentCount = 5;
constexpr std::size_t kNameLookups = 10;
constexpr std::size_t kRecordChurn = 6;
constexpr std::size_t kEfcBuckets = 8;
constexpr std::size_t kEfcRowsMax = 200;
constexpr long kBatchPeriod = 10;
constexpr long kBatchBatch = 6;
constexpr long kBatchSolve = 16;
constexpr std::size_t kBatchWidth = 3;
constexpr int kForwardsPerStep = 2;
constexpr double kSolveEta = 1e-9;
constexpr double kRegularization = 1e-6;
constexpr std::uint64_t kWorkloadSeed = 0x42F0E1EBA9EA3693ULL;

// heavy/operation_mix map attachment pairs onto physical bodies
// 1..2*kAttachmentCount; a model needs at least that many bodies (world +
// links).
constexpr int kHeavyMinBodies = 1 + 2 * static_cast<int>(kAttachmentCount);
constexpr int kHeavyMinNv = 3;
constexpr int kPointsPerAttachment = 7;

// Element ceiling for every model-derived extent, mirroring
// state_format.hpp: element counts must stay byte-representable and inside
// the signed mjtSize domain used by mjData arrays.
constexpr std::size_t kMaxElements =
    std::numeric_limits<std::size_t>::max() / sizeof(mjtNum);

// Model dimensions are mjtSize (int64); negative extents are rejected before
// any size_t-sized buffer is formed from them.
[[nodiscard]] std::size_t model_extent(mjtSize dimension,
                                       std::string_view name) {
    if (dimension < 0)
        throw std::invalid_argument(std::string(name) + ": negative extent");
    return static_cast<std::size_t>(dimension);
}

[[nodiscard]] std::size_t jac_elements(std::size_t nv) {
    return model_access::checked_product(std::size_t{3}, nv, kMaxElements);
}

[[nodiscard]] std::size_t xfrc_elements(const mjModel &model) {
    return model_access::checked_product(
        std::size_t{6}, model_extent(model.nbody, "nbody"), kMaxElements);
}

[[nodiscard]] std::size_t point_offset(std::size_t body) {
    return model_access::checked_product(body, std::size_t{3}, kMaxElements);
}

struct CliOptions {
    Mode mode = Mode::bare;
    long steps = kDefaultSteps;
    long rewind = kDefaultRewind;
    std::filesystem::path artifacts;
    std::filesystem::path emit_json;
};

[[nodiscard]] const char *mode_name(Mode mode) {
    switch (mode) {
        case Mode::bare: return "bare";
        case Mode::glue: return "glue";
        case Mode::heavy: return "heavy";
        case Mode::operation_mix: return "operation_mix";
    }
    return "unknown"; // unreachable: every Mode enumerant returns above
}

// The matched workloads carry a cross-language parity claim; operation_mix
// is the retained R2 approximation and makes none.
[[nodiscard]] bool is_matched(Mode mode) { return mode != Mode::operation_mix; }

// Integer CLI fields: std::from_chars over the string_view's own iterator
// bounds — std::to_address keeps the end bound inside iterator arithmetic
// instead of raw pointer math. Full consumption, positivity, and the hard
// count bound are all required.
[[nodiscard]] long parse_count(std::string_view value, std::string_view field) {
    long parsed = 0;
    const char *end = std::to_address(value.end());
    const auto result =
        std::from_chars(std::to_address(value.begin()), end, parsed);
    if (result.ec != std::errc() || result.ptr != end || parsed < 1 ||
        parsed > kMaxCount)
        throw std::invalid_argument(std::string(field) +
                                    " must be an integer in [1, " +
                                    std::to_string(kMaxCount) + "], got '" +
                                    std::string(value) + "'");
    return parsed;
}

[[nodiscard]] Mode parse_mode(std::string_view requested) {
    if (requested == "bare") return Mode::bare;
    if (requested == "glue") return Mode::glue;
    if (requested == "heavy") return Mode::heavy;
    if (requested == "operation_mix") return Mode::operation_mix;
    throw std::invalid_argument(
        "unknown mode '" + std::string(requested) +
        "'; expected bare, glue, heavy, or operation_mix");
}

// CLI: mode, then [steps] [rewind], then required --artifacts <dir> plus
// optional --emit-json <path>. std::from_chars with full-consumption and
// bounded range checks replaces the prototype's atoi/std::exit handling;
// malformed input throws.
[[nodiscard]] CliOptions parse_cli(std::span<const char *const> args) {
    if (args.empty())
        throw std::invalid_argument(
            "mode is required: native_bench {bare,glue,heavy,operation_mix} "
            "[steps] [rewind] --artifacts <directory>");

    const Mode mode = parse_mode(std::string_view{args.front()});
    CliOptions options{.mode = mode,
                       .steps = kDefaultSteps,
                       .rewind = kDefaultRewind,
                       .artifacts = {},
                       .emit_json = {}};
    int positionals = 0;
    bool have_artifacts = false;
    bool have_emit = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view text{args[i]};
        if (text == "--artifacts" || text == "--emit-json") {
            if (i + 1 >= args.size())
                throw std::invalid_argument(std::string(text) +
                                            " requires a value");
            const std::string_view value{args[i + 1]};
            if (value.empty())
                throw std::invalid_argument(std::string(text) +
                                            " requires a value");
            if (text == "--artifacts") {
                if (have_artifacts)
                    throw std::invalid_argument("duplicate --artifacts");
                options.artifacts = value;
                have_artifacts = true;
            } else {
                if (have_emit)
                    throw std::invalid_argument("duplicate --emit-json");
                options.emit_json = value;
                have_emit = true;
            }
            ++i;
            continue;
        }
        if (text.starts_with("--"))
            throw std::invalid_argument("unknown option '" +
                                        std::string(text) + "'");
        switch (positionals) {
            case 0:
                options.steps = parse_count(text, "steps");
                break;
            case 1:
                options.rewind = parse_count(text, "rewind");
                break;
            default:
                throw std::invalid_argument("unexpected positional argument '" +
                                            std::string(text) + "'");
        }
        ++positionals;
    }
    if (!have_artifacts)
        throw std::invalid_argument(
            "missing required --artifacts <directory>: an artifacts root with "
            "a current.json pointer or a concrete bundle directory");
    return options;
}

// ---------------------------------------------------------------------------
// Artifact resolution: current.json pointer or concrete bundle directory
// ---------------------------------------------------------------------------

// Minimal field extraction for the generated {"format":1,"bundle":"name"}
// pointer file. The bundle name is then restricted to a safe directory-name
// character set, so a hostile or corrupt pointer cannot escape the artifacts
// root — resolution only ever names a direct child directory.
// `key` stays const char* deliberately: a distinct parameter type keeps the
// call sites (json, "literal") impossible to swap by mistake.
[[nodiscard]] std::optional<std::string>
json_string_field(std::string_view json, const char *key) {
    const std::string quoted = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = json.find(quoted);
    if (key_pos == std::string_view::npos)
        return std::nullopt;
    std::size_t pos = json.find(':', key_pos + quoted.size());
    if (pos == std::string_view::npos)
        return std::nullopt;
    ++pos;
    while (pos < json.size() &&
           (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' ||
            json[pos] == '\r'))
        ++pos;
    if (pos >= json.size() || json[pos] != '"')
        return std::nullopt;
    ++pos;
    const std::size_t start = pos;
    while (pos < json.size() && json[pos] != '"') {
        // Escape sequences are rejected outright: the generator never emits
        // them, and refusing to interpret them keeps the name literal.
        if (json[pos] == '\\' ||
            static_cast<unsigned char>(json[pos]) < 0x20)
            return std::nullopt;
        ++pos;
    }
    if (pos >= json.size())
        return std::nullopt;
    return std::string{json.substr(start, pos - start)};
}

[[nodiscard]] std::optional<long long>
json_int_field(std::string_view json, const char *key) {
    const std::string quoted = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = json.find(quoted);
    if (key_pos == std::string_view::npos)
        return std::nullopt;
    std::size_t pos = json.find(':', key_pos + quoted.size());
    if (pos == std::string_view::npos)
        return std::nullopt;
    ++pos;
    while (pos < json.size() &&
           (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' ||
            json[pos] == '\r'))
        ++pos;
    long long value = 0;
    const std::string_view tail = json.substr(pos);
    const auto result = std::from_chars(std::to_address(tail.begin()),
                                        std::to_address(tail.end()), value);
    if (result.ec != std::errc() || result.ptr == std::to_address(tail.begin()))
        return std::nullopt;
    return value;
}

[[nodiscard]] bool safe_bundle_name(std::string_view name) {
    if (name.empty() || name == "." || name == "..")
        return false;
    return std::ranges::all_of(name, [](char ch) {
        return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
               (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' ||
               ch == '.';
    });
}

struct ResolvedBundle {
    std::filesystem::path dir;
    std::filesystem::path model;
    std::filesystem::path state;
    std::filesystem::path npz;
    std::filesystem::path manifest;
    bool via_pointer = false;
};

[[nodiscard]] ResolvedBundle
resolve_artifacts(const std::filesystem::path &root) {
    std::error_code error;
    ResolvedBundle bundle;
    const std::filesystem::path pointer = root / "current.json";
    if (std::filesystem::is_regular_file(pointer, error) && !error) {
        const std::string label = pointer.string();
        const std::vector<char> raw =
            proto_native_bench::detail::read_file(pointer, label);
        if (raw.size() > 4096)
            throw std::runtime_error(label +
                                     ": current.json pointer is oversized");
        const std::string_view text{raw.data(), raw.size()};
        const std::optional<long long> format = json_int_field(text, "format");
        if (!format.has_value() || *format != 1)
            throw std::runtime_error(label +
                                     ": unsupported current.json pointer "
                                     "format");
        const std::optional<std::string> name =
            json_string_field(text, "bundle");
        if (!name.has_value() || !safe_bundle_name(*name))
            throw std::runtime_error(label +
                                     ": unsafe or missing bundle name");
        bundle.dir = root / *name;
        bundle.via_pointer = true;
        if (!std::filesystem::is_directory(bundle.dir, error) || error)
            throw std::runtime_error(
                label + ": points at missing bundle directory '" + *name +
                "'");
    } else {
        bundle.dir = root;
    }
    bundle.model = bundle.dir / "model.mjb";
    bundle.state = bundle.dir / "state.bin";
    bundle.npz = bundle.dir / "state.npz";
    bundle.manifest = bundle.dir / "manifest.json";
    for (const auto &[member, tag] :
         {std::pair{&bundle.model, "model.mjb"},
          std::pair{&bundle.state, "state.bin"},
          std::pair{&bundle.npz, "state.npz"},
          std::pair{&bundle.manifest, "manifest.json"}}) {
        if (!std::filesystem::is_regular_file(*member, error) || error)
            throw std::runtime_error(member->string() + ": bundle member " +
                                     tag + " is not a readable file");
    }
    return bundle;
}

// Ownership for engine-loaded objects: mj_deleteModel/mj_deleteData instead
// of raw pointer leaks.
struct ModelDeleter {
    void operator()(mjModel *model) const noexcept { mj_deleteModel(model); }
};
struct DataDeleter {
    void operator()(mjData *data) const noexcept { mj_deleteData(data); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

// Copy a state section into an engine array only after exact extent
// validation: the source must equal the declared model extent, and the
// destination pointer must be nonnull for nonzero extents before the span
// is even formed.
void copy_exact(std::span<const mjtNum> source, mjtNum *destination,
                std::size_t extent, std::string_view field) {
    if (extent > static_cast<std::size_t>(std::numeric_limits<mjtSize>::max()))
        throw std::invalid_argument(std::string(field) +
                                    ": extent is not mjtSize-representable");
    const std::span<mjtNum> target = model_access::mutable_buffer(
        destination, static_cast<mjtSize>(extent), field);
    if (source.size() != target.size())
        throw std::invalid_argument(std::string(field) +
                                    ": state/model extent mismatch (" +
                                    std::to_string(source.size()) +
                                    " != " + std::to_string(target.size()) +
                                    ")");
    std::ranges::copy(source, target.begin());
}

void restore_state(const mjModel &model, mjData &data, const State &state) {
    data.time = state.time;
    copy_exact(state.qpos, data.qpos, model_extent(model.nq, "nq"), "qpos");
    copy_exact(state.qvel, data.qvel, model_extent(model.nv, "nv"), "qvel");
    copy_exact(state.act, data.act, model_extent(model.na, "na"), "act");
    copy_exact(state.ctrl, data.ctrl, model_extent(model.nu, "nu"), "ctrl");
    copy_exact(state.qfrc, data.qfrc_applied, model_extent(model.nv, "nv"),
               "qfrc_applied");
    copy_exact(state.xfrc, data.xfrc_applied, xfrc_elements(model),
               "xfrc_applied");
    copy_exact(state.qacc_warmstart, data.qacc_warmstart,
               model_extent(model.nv, "nv"), "qacc_warmstart");
    engine::forward(&model, &data);
}

// Engine-boundary trampolines for the non-step calls in the workload mix.
// engine::invoke accepts only trivial noexcept operations, so validation
// runs in throwing C++ before the frame is entered and each trampoline is
// a bare MuJoCo call — the same shape as engine_call.cpp's own operations.
struct JacobianCall {
    const mjModel *model;
    const mjData *data;
    mjtNum *jacp;
    mjtNum *jacr;
    const mjtNum *point;
    int body;
};
// engine::Operation fixes the void* signature; this call is read-only
// through its context, so the pointee-const hint cannot apply.
// NOLINTNEXTLINE(misc-const-correctness)
void jacobian_operation(void *raw) noexcept {
    const auto *call = static_cast<const JacobianCall *>(raw);
    mj_jac(call->model, call->data, call->jacp, call->jacr, call->point,
           call->body);
}

struct NameLookupCall {
    const mjModel *model;
    const char *name;
    int result;
};
void name_lookup_operation(void *raw) noexcept {
    auto *call = static_cast<NameLookupCall *>(raw);
    call->result = mj_name2id(call->model, mjOBJ_JOINT, call->name);
}

struct IdNameCall {
    const mjModel *model;
    int id;
    const char *result;
};
void id_name_operation(void *raw) noexcept {
    auto *call = static_cast<IdNameCall *>(raw);
    call->result = mj_id2name(call->model, mjOBJ_JOINT, call->id);
}

void invoke_or_throw(engine::Operation operation, void *context) {
    engine::ErrorBuffer error;
    if (!engine::invoke(operation, context, error))
        engine::throw_failure(error);
}

// Destination for a point-Jacobian call: jacp always holds 3*nv elements;
// jacr either holds the same extent or is empty (nullptr to mj_jac).
struct JacobianTarget {
    std::span<mjtNum> jacp;
    std::span<mjtNum> jacr;
};

// Checked point-Jacobian call: the destination must be exactly 3*nv, the
// body must be a valid model ID, and the point must be three finite
// coordinates — matching model_access::point_jacobian_into's contract, but
// the MuJoCo call itself is dispatched through the E1 frame.
void point_jacobian(const mjModel &model, const mjData &data,
                    JacobianTarget target, std::span<const mjtNum> point,
                    int body) {
    model_access::require_id(body, model.nbody, "Jacobian body");
    if (point.size() != std::size_t{3} ||
        !std::ranges::all_of(point, [](mjtNum value) {
            return std::isfinite(value);
        }))
        throw std::invalid_argument("invalid world point");
    if (target.jacp.size() != jac_elements(model_extent(model.nv, "nv")))
        throw std::invalid_argument("jacp destination must be 3*nv elements");
    if (!target.jacr.empty() && target.jacr.size() != target.jacp.size())
        throw std::invalid_argument("jacr destination must be 3*nv elements");
    JacobianCall call{.model = &model,
                      .data = &data,
                      .jacp = target.jacp.data(),
                      .jacr = target.jacr.empty() ? nullptr : target.jacr.data(),
                      .point = point.data(),
                      .body = body};
    invoke_or_throw(jacobian_operation, &call);
}

[[nodiscard]] const char *joint_name(const mjModel &model, int id) {
    IdNameCall call{.model = &model, .id = id, .result = nullptr};
    invoke_or_throw(id_name_operation, &call);
    return call.result;
}

[[nodiscard]] int lookup_joint(const mjModel &model, const char *name) {
    NameLookupCall call{.model = &model, .name = name, .result = -1};
    invoke_or_throw(name_lookup_operation, &call);
    return call.result;
}

// Supported workload domains are validated before any modulo, body index,
// or Jacobian buffer is sized — the prototype indexed attachment bodies and
// qpos[0] without ever checking the model had them.
void validate_workload(Mode mode, const mjModel &model) {
    if (mode == Mode::bare)
        return; // any engine-loadable model, including an empty one, may step

    if (model.nq <= 0)
        throw std::invalid_argument(std::format(
            "mode '{}' requires a model with nq > 0 (got {})",
            mode_name(mode), model.nq));
    if (model.nv <= 0)
        throw std::invalid_argument(std::format(
            "mode '{}' requires a model with nv > 0 (got {})",
            mode_name(mode), model.nv));
    if (model.nu <= 0)
        throw std::invalid_argument(std::format(
            "mode '{}' requires a model with nu > 0 (got {})",
            mode_name(mode), model.nu));
    if (model.nbody <= 1)
        throw std::invalid_argument(std::format(
            "mode '{}' requires at least one nonworld body "
            "(nbody > 1, got {})",
            mode_name(mode), model.nbody));
    if (mode == Mode::heavy || mode == Mode::operation_mix) {
        if (model.nbody < kHeavyMinBodies)
            throw std::invalid_argument(std::format(
                "mode '{}' requires nbody >= {} to resolve {} attachment "
                "body pairs (got {})",
                mode_name(mode), kHeavyMinBodies, kAttachmentCount,
                model.nbody));
        if (model.nv < kHeavyMinNv)
            throw std::invalid_argument(std::format(
                "mode '{}' requires nv >= {} (got {})", mode_name(mode),
                kHeavyMinNv, model.nv));
    }
}

struct Attachment {
    std::size_t body = 0;
    std::size_t peer = 0;
    double gain = 0;
};

// Representative record churn: the Python glue allocates per-step telemetry
// records; the benchmark materializes a few per timed step so the operation
// mix is visible without claiming parity with the Python cost model.
struct Record {
    long index = 0;
    double a = 0;
    double b = 0;
};

// ---------------------------------------------------------------------------
// Deterministic workload seeds: splitmix64, mirrored in bench.py so both
// languages fill the writer-source vectors with bit-identical doubles.
// ---------------------------------------------------------------------------
struct Splitmix64 {
    std::uint64_t state = 0;
    [[nodiscard]] std::uint64_t next_u64() noexcept {
        state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31U);
    }
    // [0,1) double via the standard (u64 >> 11) * 2^-53 reduction.
    [[nodiscard]] double next_f64() noexcept {
        return static_cast<double>(next_u64() >> 11U) *
               (1.0 / 9007199254740992.0);
    }
};

// Operation counts and per-stage checksum contributions, accumulated in a
// fixed order identical to bench.py's. These are emitted by --emit-json and
// compared field-by-field by the conformance suite.
struct OpCounts {
    std::int64_t engine_steps = 0;
    std::int64_t rewinds = 0;
    std::int64_t forwards = 0;
    std::int64_t jacobian_calls = 0;
    std::int64_t name_lookups = 0;
    std::int64_t qfrc_elements = 0;
    std::int64_t channel_writes = 0;
    std::int64_t energy_terms = 0;
    std::int64_t efc_rows = 0;
    std::int64_t attachment_solves = 0;
    std::int64_t point_evals = 0;
    std::int64_t ledger_terms = 0;
    std::int64_t records = 0;
    std::int64_t batch_events = 0;
    std::int64_t batch_iterations = 0;
};

struct StageChecksums {
    // trajectory_time is the "exact scalar" stage: data.time arrives as
    // bit-identical engine output, so sequential accumulation is byte-equal
    // across languages. name_ids is the exact integer stage.
    double trajectory_time = 0;
    double forces = 0;
    double energy = 0;
    double jacobians = 0;
    double attachments = 0;
    double channels = 0;
    double efc = 0;
    double ledger = 0;
    double records = 0;
    double batch = 0;
    std::int64_t name_ids = 0;
};

struct RunStats {
    OpCounts counts;
    StageChecksums ck;
    std::vector<double> solve_outputs;
    // operation_mix keeps its R2 aggregate sink — it claims no per-stage
    // parity, only finiteness.
    double mix_sink = 0;
};

// Model-sized scratch owned by the benchmark: eight separate nv-sized source
// vectors (the prototype reused one), nv-sized accumulator, and three
// independent 3*nv Jacobian buffers (the prototype shared one buffer and
// overflowed a fixed nv=64 assumption).
struct GlueContext {
    std::size_t nv = 0;
    std::size_t nq = 0;
    std::size_t nu = 0;
    std::size_t nbody = 0;
    std::array<std::vector<mjtNum>, kWriterCount> sources;
    std::vector<mjtNum> acc;
    std::vector<mjtNum> jacp;
    std::vector<mjtNum> jacr;
    std::vector<mjtNum> jacp2;
    std::vector<double> channels;
    std::vector<mjtNum> solve;
    std::array<Attachment, kAttachmentCount> attachments{};
    std::vector<const char *> names;
};

// Fill the writer-source vectors. Matched modes use the seeded stream so
// bench.py fills bit-identical values; operation_mix keeps the R2
// closed-form fill — the retained approximation is preserved literally.
void fill_sources(GlueContext &ctx, bool seeded) {
    Splitmix64 seed{.state = kWorkloadSeed};
    for (std::size_t w = 0; w < ctx.sources.size(); ++w) {
        std::vector<mjtNum> &source = ctx.sources[w];
        source.assign(ctx.nv, mjtNum{0});
        for (std::size_t j = 0; j < ctx.nv; ++j)
            source[j] = seeded ? 0.1 * seed.next_f64() - 0.05
                               : 0.01 * static_cast<mjtNum>(w + 1) *
                                     static_cast<mjtNum>(
                                         static_cast<long>(j % 7) - 3);
    }
}

void prepare_glue(const mjModel &model, GlueContext &ctx, bool seeded) {
    ctx.nv = model_extent(model.nv, "nv");
    ctx.nq = model_extent(model.nq, "nq");
    ctx.nu = model_extent(model.nu, "nu");
    ctx.nbody = model_extent(model.nbody, "nbody");
    ctx.acc.assign(ctx.nv, mjtNum{0});
    fill_sources(ctx, seeded);
    const std::size_t jac_size = jac_elements(ctx.nv);
    ctx.jacp.assign(jac_size, mjtNum{0});
    ctx.jacr.assign(jac_size, mjtNum{0});
    ctx.jacp2.assign(jac_size, mjtNum{0});
    ctx.channels.assign(kChannelCount, 0.0);
    for (int j = 1; j < model.njnt && ctx.names.size() < kNameLookups; ++j) {
        if (const char *name = joint_name(model, j))
            ctx.names.push_back(name);
    }
    if (ctx.names.empty())
        ctx.names.push_back("root_x");
}

void prepare_heavy(GlueContext &ctx) {
    ctx.solve.assign(kBatchSolve, mjtNum{0});
    for (std::size_t i = 0; i < ctx.attachments.size(); ++i)
        ctx.attachments[i] =
            Attachment{.body = i + 1,
                       .peer = i + 1 + kAttachmentCount,
                       .gain = 0.5 + 0.1 * static_cast<double>(i)};
}

[[nodiscard]] double sequential_sum(std::span<const mjtNum> values) {
    return std::ranges::fold_left(values, 0.0, std::plus{});
}

// Regularized SPD 3x3 solve via lower Cholesky + forward/back substitution —
// mirrored scalar-for-scalar in bench.py so both languages run the same
// solve sequence with the same rank policy (fixed full-rank regularization,
// never a rank-revealing fallback). A nonpositive pivot or non-finite result
// is an explicit failure, never a successful NaN sink.
[[nodiscard]] std::array<double, 3>
solve3x3_spd(const std::array<double, 9> &a, const std::array<double, 3> &b) {
    const double l11 = std::sqrt(a[0]);
    if (!(l11 > 0.0) || !std::isfinite(l11))
        throw std::runtime_error(
            "attachment normal matrix is singular after regularization");
    const double l21 = a[3] / l11;
    const double l31 = a[6] / l11;
    const double l22_sq = a[4] - l21 * l21;
    const double l22 = std::sqrt(l22_sq);
    if (!(l22 > 0.0) || !std::isfinite(l22))
        throw std::runtime_error(
            "attachment normal matrix is singular after regularization");
    const double l32 = (a[7] - l31 * l21) / l22;
    const double l33_sq = a[8] - l31 * l31 - l32 * l32;
    const double l33 = std::sqrt(l33_sq);
    if (!(l33 > 0.0) || !std::isfinite(l33))
        throw std::runtime_error(
            "attachment normal matrix is singular after regularization");
    const double y1 = b[0] / l11;
    const double y2 = (b[1] - l21 * y1) / l22;
    const double y3 = (b[2] - l31 * y1 - l32 * y2) / l33;
    const double x3 = y3 / l33;
    const double x2 = (y2 - l32 * x3) / l22;
    const double x1 = (y1 - l21 * x2 - l31 * x3) / l11;
    const std::array<double, 3> result{x1, x2, x3};
    if (!std::ranges::all_of(result,
                             [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("attachment wrench solve produced "
                                 "non-finite output");
    return result;
}

// The matched per-step workload. Stage order is fixed and identical to
// bench.py's: forces -> forward -> energy -> jacobians/solves -> names ->
// channels -> (heavy: efc, ledger, records) -> step -> forward ->
// (heavy: batch) -> trajectory probe.
void matched_step(const mjModel &model, mjData &data, const State &state,
                  GlueContext &ctx, Mode mode, long index, RunStats &stats) {
    const std::size_t nv = ctx.nv;
    const bool heavy = mode == Mode::heavy;

    // Stage 1 — applied-force assembly: eight writer sources summed into
    // qfrc_applied (elementwise, so identical-order in both languages), the
    // recorded xfrc re-staged, then a forward recompute.
    std::ranges::fill(ctx.acc, mjtNum{0});
    for (const std::vector<mjtNum> &source : ctx.sources)
        for (std::size_t j = 0; j < nv; ++j)
            ctx.acc[j] += source[j];
    copy_exact(ctx.acc, data.qfrc_applied, nv, "qfrc_applied");
    copy_exact(state.xfrc, data.xfrc_applied, xfrc_elements(model),
               "xfrc_applied");
    engine::forward(&model, &data);
    stats.counts.forwards += 1;
    stats.counts.qfrc_elements +=
        static_cast<std::int64_t>(kWriterCount * nv);
    stats.ck.forces += sequential_sum(ctx.acc);

    const std::span<const mjtNum> qpos =
        model_access::readonly_buffer(data.qpos, model.nq, "qpos");
    const std::span<const mjtNum> qvel =
        model_access::readonly_buffer(data.qvel, model.nv, "qvel");
    const std::span<const mjtNum> qfrc_constraint =
        model_access::readonly_buffer(data.qfrc_constraint, model.nv,
                                      "qfrc_constraint");
    const std::span<const mjtNum> qacc =
        model_access::readonly_buffer(data.qacc, model.nv, "qacc");
    const std::span<const mjtNum> qfrc_actuator =
        model_access::readonly_buffer(data.qfrc_actuator, model.nv,
                                      "qfrc_actuator");
    const std::span<const mjtNum> ctrl =
        model_access::readonly_buffer(data.ctrl, model.nu, "ctrl");
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        data.xpos, model.nbody * 3, "xpos");

    // Stage 2 — energy probe. The actuator side uses the engine-projected
    // qfrc_actuator (moment^T . actuator_force, length nv) — the explicit
    // supported actuator->velocity mapping, never qvel[:nu] slicing.
    double energy = 0.0;
    for (std::size_t j = 0; j < nv; ++j)
        energy += qfrc_constraint[j] * qvel[j];
    for (std::size_t j = 0; j < nv; ++j)
        energy += qfrc_actuator[j] * qvel[j];
    stats.ck.energy += energy;
    stats.counts.energy_terms += static_cast<std::int64_t>(2 * nv);

    // Stage 3 — point Jacobians (glue: one pair on body 1; heavy: five
    // attachment pairs) plus the heavy attachment wrench solves.
    if (!heavy) {
        const std::span<const mjtNum> point = xpos.subspan(3, 3);
        point_jacobian(model, data, {.jacp = ctx.jacp, .jacr = ctx.jacr},
                       point, 1);
        point_jacobian(model, data,
                       {.jacp = ctx.jacp2, .jacr = std::span<mjtNum>{}},
                       point, 1);
        stats.counts.jacobian_calls += 2;
        stats.ck.jacobians += sequential_sum(ctx.jacp) +
                              sequential_sum(ctx.jacr) +
                              sequential_sum(ctx.jacp2);
    } else {
        for (const Attachment &attachment : ctx.attachments) {
            point_jacobian(
                model, data, {.jacp = ctx.jacp, .jacr = ctx.jacr},
                xpos.subspan(point_offset(attachment.body), 3),
                static_cast<int>(attachment.body));
            point_jacobian(
                model, data,
                {.jacp = ctx.jacp2, .jacr = std::span<mjtNum>{}},
                xpos.subspan(point_offset(attachment.peer), 3),
                static_cast<int>(attachment.peer));
            stats.counts.jacobian_calls += 2;
            stats.ck.jacobians += sequential_sum(ctx.jacp) +
                                  sequential_sum(ctx.jacr) +
                                  sequential_sum(ctx.jacp2);

            // Attachment wrench solve: N = J1 J1^T + 1e-6*I (the declared
            // regularization), b = J1 * qfrc_constraint, w = solve(N, b).
            // Same reduction order in both languages; solver outputs are
            // compared at rtol=atol=1e-12.
            std::array<double, 9> normal{};
            std::array<double, 3> rhs{};
            for (std::size_t i = 0; i < 3; ++i) {
                for (std::size_t j = 0; j < 3; ++j) {
                    double sum = 0.0;
                    for (std::size_t k = 0; k < nv; ++k)
                        sum += ctx.jacp[i * nv + k] * ctx.jacp[j * nv + k];
                    normal[i * 3 + j] = sum;
                }
                double dot = 0.0;
                for (std::size_t k = 0; k < nv; ++k)
                    dot += ctx.jacp[i * nv + k] * qfrc_constraint[k];
                rhs[i] = dot;
            }
            for (std::size_t i = 0; i < 3; ++i)
                normal[i * 3 + i] += kRegularization;
            const std::array<double, 3> wrench = solve3x3_spd(normal, rhs);
            stats.counts.attachment_solves += 1;
            for (const double component : wrench)
                stats.solve_outputs.push_back(component);
            // Seven deterministic point evaluations per attachment.
            for (std::size_t k = 0;
                 k < static_cast<std::size_t>(kPointsPerAttachment); ++k)
                stats.ck.attachments += attachment.gain * wrench[k % 3];
            stats.counts.point_evals += kPointsPerAttachment;
        }
    }

    // Stage 4 — name lookups resolve through the engine boundary; the exact
    // integer id sum is the byte-equal scalar stage.
    for (const char *name : ctx.names) {
        stats.ck.name_ids += lookup_joint(model, name);
        stats.counts.name_lookups += 1;
    }

    // Stage 5 — 100 channel writes per step.
    for (std::size_t c = 0; c < kChannelCount; ++c)
        ctx.channels[c] = qpos[c % ctx.nq] * static_cast<double>(c + 1) +
                          static_cast<double>(index) * 1e-9;
    stats.counts.channel_writes +=
        static_cast<std::int64_t>(kChannelCount);
    stats.ck.channels += sequential_sum(ctx.channels);

    if (heavy) {
        // Stage 6 — efc-row unpack, batched; the span is sized from the
        // reported nefc only.
        const std::size_t n_read =
            std::min(static_cast<std::size_t>(std::max(data.nefc, 0)),
                     kEfcRowsMax);
        const std::span<const int> efc_id = model_access::readonly_buffer(
            data.efc_id, data.nefc, "efc_id");
        const std::span<const mjtNum> efc_pos = model_access::readonly_buffer(
            data.efc_pos, data.nefc, "efc_pos");
        const std::span<const mjtNum> efc_force =
            model_access::readonly_buffer(data.efc_force, data.nefc,
                                          "efc_force");
        std::array<long, kEfcBuckets> bucket{};
        std::array<mjtNum, kEfcBuckets> bpos{};
        for (std::size_t r = 0; r < n_read; ++r) {
            const std::size_t k =
                static_cast<std::size_t>(efc_id[r]) & (kEfcBuckets - 1);
            bucket[k]++;
            bpos[k] += efc_pos[r] + efc_force[r];
        }
        stats.ck.efc += static_cast<double>(bucket[0]) +
                        static_cast<double>(bpos[1]);
        stats.counts.efc_rows += static_cast<std::int64_t>(n_read);

        // Stage 7 — energy ledger: the six declared terms, each an nv-length
        // reduction in the declared order.
        const std::span<const mjtNum> qfrc_spring =
            model_access::readonly_buffer(data.qfrc_spring, model.nv,
                                          "qfrc_spring");
        const std::span<const mjtNum> qfrc_damper =
            model_access::readonly_buffer(data.qfrc_damper, model.nv,
                                          "qfrc_damper");
        const std::span<const mjtNum> qfrc_bias =
            model_access::readonly_buffer(data.qfrc_bias, model.nv,
                                          "qfrc_bias");
        const std::span<const mjtNum> qfrc_applied =
            model_access::readonly_buffer(data.qfrc_applied, model.nv,
                                          "qfrc_applied");
        double ledger_sum = 0.0;
        for (const auto &[a, b] : {std::pair{qfrc_spring, qvel},
                                   std::pair{qfrc_damper, qvel},
                                   std::pair{qfrc_bias, qvel},
                                   std::pair{qfrc_applied, qvel},
                                   std::pair{qfrc_constraint, qvel},
                                   std::pair{qfrc_actuator, qvel}}) {
            double term = 0.0;
            for (std::size_t j = 0; j < nv; ++j)
                term += a[j] * b[j];
            ledger_sum += term;
        }
        stats.ck.ledger += ledger_sum;
        stats.counts.ledger_terms += 6;

        // Stage 8 — record churn: six per-step telemetry records.
        std::vector<Record> records;
        records.reserve(kRecordChurn);
        for (std::size_t k = 0; k < kRecordChurn; ++k)
            records.push_back(
                Record{.index = index, .a = qpos[0], .b = ctrl[0]});
        records.back().b += ctx.channels[7];
        for (const Record &record : records)
            stats.ck.records += record.a + record.b;
        stats.counts.records += static_cast<std::int64_t>(kRecordChurn);
    }

    // Stage 9/10 — the engine step and the post-step telemetry forward.
    engine::step(&model, &data);
    stats.counts.engine_steps += 1;
    engine::forward(&model, &data);
    stats.counts.forwards += 1;

    if (heavy && index % kBatchPeriod == 0) {
        // Stage 11 — period-batch solve: 16 gradient-free iterations over
        // six dof triples; qacc spans are read through checked accessors.
        for (std::size_t c = 0; c < static_cast<std::size_t>(kBatchBatch);
             ++c) {
            for (std::size_t it = 0;
                 it < static_cast<std::size_t>(kBatchSolve); ++it) {
                double g = 0.0;
                for (std::size_t k2 = 0; k2 < kBatchWidth; ++k2)
                    g += qacc[(k2 + c) % nv];
                ctx.solve[it] -= kSolveEta * g;
            }
            stats.ck.batch += ctx.solve[c] * 1e-6;
        }
        for (const mjtNum value : ctx.solve)
            stats.solve_outputs.push_back(value);
        stats.counts.batch_events += 1;
        stats.counts.batch_iterations +=
            static_cast<std::int64_t>(kBatchBatch) * kBatchSolve;
    }

    // Stage 12 — trajectory probe: bit-identical engine output accumulated
    // in identical order, the declared byte-equal scalar stage.
    stats.ck.trajectory_time += data.time;
}

// The retained R2 operation mix: the same engine surface, a per-step mix
// that does NOT match bench.py's, and no parity claim. Preserved verbatim
// from R2 — including the property that it never calls mj_step — so the
// label stays honest.
void mix_step(const mjModel &model, const mjData &data, const State &state,
              GlueContext &ctx, long index, RunStats &stats) {
    const std::size_t nv = ctx.nv;

    std::ranges::fill(ctx.acc, mjtNum{0});
    for (const std::vector<mjtNum> &source : ctx.sources)
        for (std::size_t j = 0; j < nv; ++j)
            ctx.acc[j] += source[j];
    copy_exact(ctx.acc, data.qfrc_applied, nv, "qfrc_applied");
    copy_exact(state.xfrc, data.xfrc_applied, xfrc_elements(model),
               "xfrc_applied");

    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        data.xpos, model.nbody * 3, "xpos");
    const std::span<const mjtNum> qpos =
        model_access::readonly_buffer(data.qpos, model.nq, "qpos");
    const std::span<const mjtNum> qvel =
        model_access::readonly_buffer(data.qvel, model.nv, "qvel");
    const std::span<const mjtNum> qfrc_constraint =
        model_access::readonly_buffer(data.qfrc_constraint, model.nv,
                                      "qfrc_constraint");
    const std::span<const mjtNum> qacc =
        model_access::readonly_buffer(data.qacc, model.nv, "qacc");
    const std::span<const mjtNum> actuator_force =
        model_access::readonly_buffer(data.actuator_force, model.nu,
                                      "actuator_force");
    const std::span<const mjtNum> ctrl =
        model_access::readonly_buffer(data.ctrl, model.nu, "ctrl");

    std::vector<Record> records;
    records.reserve(kRecordChurn);

    const std::size_t n_read =
        std::min(static_cast<std::size_t>(std::max(data.nefc, 0)),
                 std::size_t{200});
    const std::span<const int> efc_id =
        model_access::readonly_buffer(data.efc_id, data.nefc, "efc_id");
    const std::span<const mjtNum> efc_pos =
        model_access::readonly_buffer(data.efc_pos, data.nefc, "efc_pos");
    const std::span<const mjtNum> efc_force = model_access::readonly_buffer(
        data.efc_force, data.nefc, "efc_force");
    std::array<long, 8> bucket{};
    std::array<mjtNum, 8> bpos{};
    for (std::size_t r = 0; r < n_read; ++r) {
        const std::size_t k = static_cast<std::size_t>(efc_id[r]) & 7U;
        bucket[k]++;
        bpos[k] += efc_pos[r] + efc_force[r];
    }
    stats.mix_sink +=
        static_cast<double>(bucket[0]) + static_cast<double>(bpos[1]);

    for (const Attachment &attachment : ctx.attachments) {
        point_jacobian(
            model, data, {.jacp = ctx.jacp, .jacr = ctx.jacr},
            xpos.subspan(point_offset(attachment.body), 3),
            static_cast<int>(attachment.body));
        point_jacobian(
            model, data, {.jacp = ctx.jacp2, .jacr = std::span<mjtNum>{}},
            xpos.subspan(point_offset(attachment.peer), 3),
            static_cast<int>(attachment.peer));
        for (std::size_t k = 0;
             k < static_cast<std::size_t>(kPointsPerAttachment); ++k) {
            const std::size_t row = model_access::checked_product(
                k % std::size_t{3}, nv, kMaxElements);
            const std::span<const mjtNum> jacp_row =
                std::span<const mjtNum>{ctx.jacp}.subspan(row, nv);
            const std::span<const mjtNum> jacp2_row =
                std::span<const mjtNum>{ctx.jacp2}.subspan(row, nv);
            const double a =
                1e-6 * std::ranges::fold_left(jacp_row, 0.0, std::plus{});
            const double b =
                1e-6 * std::ranges::fold_left(jacp2_row, 0.0, std::plus{});
            stats.mix_sink += a * b * attachment.gain;
        }
    }

    // R2's energy term — an approximate modulo mapping (actuator index wraps
    // nv); retained only under the operation_mix label.
    double energy = 0.0;
    for (std::size_t j = 0; j < nv; ++j)
        energy += qfrc_constraint[j] * qvel[j] +
                  actuator_force[j % ctx.nu] * qvel[j];
    stats.mix_sink += energy * 1e-9;

    for (std::size_t k = 0; k < kRecordChurn; ++k)
        records.push_back(Record{.index = index, .a = qpos[0], .b = ctrl[0]});

    if (index % kBatchPeriod == 0) {
        for (int c = 0; c < kBatchBatch; ++c) {
            const std::size_t c_idx = static_cast<std::size_t>(c);
            for (int it = 0; it < kBatchSolve; ++it) {
                double g = 0.0;
                for (int k2 = 0; k2 < 3; ++k2)
                    g += qacc[(static_cast<std::size_t>(k2) + c_idx) % nv];
                ctx.solve[static_cast<std::size_t>(it)] -= kSolveEta * g;
            }
            stats.mix_sink += ctx.solve[c_idx] * 1e-6;
        }
        for (const mjtNum value : ctx.solve)
            stats.solve_outputs.push_back(value);
        stats.counts.batch_events += 1;
    }

    for (const char *name : ctx.names) {
        stats.mix_sink += static_cast<double>(lookup_joint(model, name));
        stats.counts.name_lookups += 1;
    }

    for (std::size_t c = 0; c < kChannelCount; ++c)
        ctx.channels[c] = qpos[c % ctx.nq] * static_cast<double>(c + 1) +
                          static_cast<double>(index) * 1e-9;
    stats.counts.channel_writes += static_cast<std::int64_t>(kChannelCount);

    Record &last = records.back();
    last.b += ctx.channels[7];
    for (const Record &record : records)
        stats.mix_sink += record.a + record.b;
    stats.mix_sink += ctx.channels[0];
    stats.ck.trajectory_time += data.time;
}

void run_step(const mjModel &model, mjData &data, const State &state,
              GlueContext &ctx, Mode mode, long index, RunStats &stats) {
    if (mode == Mode::bare) {
        engine::step(&model, &data);
        stats.counts.engine_steps += 1;
        stats.ck.trajectory_time += data.time;
        return;
    }
    if (mode == Mode::operation_mix) {
        mix_step(model, data, state, ctx, index, stats);
        return;
    }
    matched_step(model, data, state, ctx, mode, index, stats);
}

void restore_counted(const mjModel &model, mjData &data, const State &state,
                     RunStats &stats) {
    restore_state(model, data, state);
    stats.counts.rewinds += 1;
    stats.counts.forwards += 1;
}

void run_phase(const mjModel &model, mjData &data, const State &state,
               GlueContext &ctx, const CliOptions &cli, long count,
               RunStats &stats) {
    for (long i = 0; i < count; ++i) {
        if (i % cli.rewind == 0)
            restore_counted(model, data, state, stats);
        run_step(model, data, state, ctx, cli.mode, i, stats);
    }
}

// ---------------------------------------------------------------------------
// Result report (--emit-json)
// ---------------------------------------------------------------------------

[[nodiscard]] std::string json_quote(std::string_view text) {
    std::string out{"\""};
    for (const char ch : text) {
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20)
                    out += std::format("\\u{:04x}",
                                       static_cast<unsigned char>(ch));
                else
                    out += ch;
        }
    }
    out += '"';
    return out;
}

// Full-precision double emission: 17 significant digits round-trip a
// binary64 exactly, so the parsed JSON value is bit-identical to the
// computed one. Non-finite values are rejected rather than emitted as
// invalid JSON.
[[nodiscard]] std::string json_f64(double value, std::string_view field) {
    if (!std::isfinite(value))
        throw std::runtime_error("report field '" + std::string(field) +
                                 "' is not finite — refusing to emit NaN/inf");
    return std::format("{:.17g}", value);
}

[[nodiscard]] std::string json_f64(double value) {
    return json_f64(value, "value");
}

[[nodiscard]] std::string json_u64_hex(std::uint64_t value) {
    return json_quote(std::format("0x{:016x}", value));
}

#if defined(__has_feature)
#define BENCH_HAS_FEATURE(x) __has_feature(x)
#else
#define BENCH_HAS_FEATURE(x) 0
#endif
#define BENCH_STRINGIFY2(x) #x
#define BENCH_STRINGIFY(x) BENCH_STRINGIFY2(x)

[[nodiscard]] std::string provenance_compiler() {
#if defined(__clang__)
    return "clang " + std::string(__clang_version__);
#elif defined(__GNUC__)
    return "gcc " + std::string(__VERSION__);
#else
    return "unknown";
#endif
}

[[nodiscard]] std::string provenance_hardening() {
    std::string out = "asan=" +
        std::to_string(BENCH_HAS_FEATURE(address_sanitizer)) +
        ";ubsan=" + std::to_string(BENCH_HAS_FEATURE(undefined_behavior_sanitizer)) +
        ";rtsan=" + std::to_string(BENCH_HAS_FEATURE(realtime_sanitizer));
#if defined(_FORTIFY_SOURCE)
    out += ";fortify=" BENCH_STRINGIFY(_FORTIFY_SOURCE);
#endif
#if defined(_LIBCPP_HARDENING_MODE)
    out += ";libcxx_hardening=" BENCH_STRINGIFY(_LIBCPP_HARDENING_MODE);
#endif
    return out;
}

[[nodiscard]] long jacobian_calls_per_step(Mode mode) {
    switch (mode) {
        case Mode::glue: return 2;
        case Mode::heavy:
        case Mode::operation_mix: return 2 * static_cast<long>(kAttachmentCount);
        case Mode::bare: return 0;
    }
    return 0;
}

// Serialize the run report. The "workload" block is the deterministic shared
// workload definition — bench.py emits the byte-equal-parsed document for
// the same mode/parameters, which is what the conformance suite compares.
[[nodiscard]] std::string
build_report(const CliOptions &cli, const mjModel &model,
             const ResolvedBundle &bundle, const State &state,
             const GlueContext &ctx, const RunStats &stats, double wall_us,
             double checksum) {
    const char *claim = is_matched(cli.mode) ? "matched" : "operation_mix";
    std::string out;
    out += "{\n" R"(  "format": 1,)" "\n" R"(  "comparison": )";
    out += json_quote(claim);
    out += ",\n" R"(  "mode": )";
    out += json_quote(mode_name(cli.mode));

    // Shared workload JSON: mode parameters + fixed seed, identical content
    // to bench.py's emission for the same mode.
    out += ",\n" R"(  "workload": {"format": 1, "mode": )";
    out += json_quote(mode_name(cli.mode));
    out += R"(, "seed": "0x42f0e1eba9ea3693", "claim": )";
    out += json_quote(claim);
    out += R"(, "warmup_steps": )";
    out += std::to_string(kWarmupSteps);
    out += R"(, "measured_steps": )";
    out += std::to_string(cli.steps);
    out += R"(, "rewind_period": )";
    out += std::to_string(cli.rewind);
    out += R"(, "writers": )";
    out += std::to_string(kWriterCount);
    out += R"(, "channels": )";
    out += std::to_string(kChannelCount);
    out += R"(, "name_lookups_max": )";
    out += std::to_string(kNameLookups);
    out += R"(, "names": [)";
    for (std::size_t i = 0; i < ctx.names.size(); ++i) {
        if (i != 0) out += ", ";
        out += json_quote(ctx.names[i]);
    }
    out += R"(], "efc_buckets": )";
    out += std::to_string(kEfcBuckets);
    out += R"(, "efc_rows_max": )";
    out += std::to_string(kEfcRowsMax);
    // The attachment table is a workload constant (mirrored in
    // prepare_heavy) — emit it unconditionally so the shared workload
    // document is identical for every mode.
    out += R"(, "attachments": [)";
    for (std::size_t i = 0; i < kAttachmentCount; ++i) {
        if (i != 0) out += ", ";
        out += R"({"body": )" + std::to_string(i + 1) +
               R"(, "peer": )" + std::to_string(i + 1 + kAttachmentCount) +
               R"(, "gain": )" +
               json_f64(0.5 + 0.1 * static_cast<double>(i)) + "}";
    }
    out += R"(], "points_per_attachment": )";
    out += std::to_string(kPointsPerAttachment);
    out += R"(, "record_churn": )";
    out += std::to_string(kRecordChurn);
    out += R"(, "ledger_terms": ["spring", "damper", "bias", )"
           R"("applied", "constraint", "actuator"])";
    out += R"(, "batch": {"period": )";
    out += std::to_string(kBatchPeriod);
    out += R"(, "batches": )";
    out += std::to_string(kBatchBatch);
    out += R"(, "iterations": )";
    out += std::to_string(kBatchSolve);
    out += R"(, "width": )";
    out += std::to_string(kBatchWidth);
    out += R"(}, "regularization": )";
    out += json_f64(kRegularization);
    out += R"(, "solve_eta": )";
    out += json_f64(kSolveEta);
    out += R"(, "forwards_per_step": )";
    out += std::to_string(kForwardsPerStep);
    out += R"(, "jacobian_calls_per_step": )";
    out += std::to_string(jacobian_calls_per_step(cli.mode));
    out += '}';

    const OpCounts &c = stats.counts;
    out += ",\n" R"(  "operation_counts": {)";
    bool first = true;
    for (const auto &[key, value] : std::array{
             std::pair{"engine_steps", c.engine_steps},
             std::pair{"rewinds", c.rewinds},
             std::pair{"forwards", c.forwards},
             std::pair{"jacobian_calls", c.jacobian_calls},
             std::pair{"name_lookups", c.name_lookups},
             std::pair{"qfrc_elements", c.qfrc_elements},
             std::pair{"channel_writes", c.channel_writes},
             std::pair{"energy_terms", c.energy_terms},
             std::pair{"efc_rows", c.efc_rows},
             std::pair{"attachment_solves", c.attachment_solves},
             std::pair{"point_evals", c.point_evals},
             std::pair{"ledger_terms", c.ledger_terms},
             std::pair{"records", c.records},
             std::pair{"batch_events", c.batch_events},
             std::pair{"batch_iterations", c.batch_iterations}}) {
        if (!first) out += ", ";
        first = false;
        out += json_quote(key);
        out += ": ";
        out += std::to_string(value);
    }
    out += '}';

    const StageChecksums &k = stats.ck;
    out += ",\n" R"(  "stage_checksums": {)";
    first = true;
    for (const auto &[key, value] : std::array{
             std::pair{"trajectory_time", k.trajectory_time},
             std::pair{"forces", k.forces},
             std::pair{"energy", k.energy},
             std::pair{"jacobians", k.jacobians},
             std::pair{"attachments", k.attachments},
             std::pair{"channels", k.channels},
             std::pair{"efc", k.efc},
             std::pair{"ledger", k.ledger},
             std::pair{"records", k.records},
             std::pair{"batch", k.batch}}) {
        if (!first) out += ", ";
        first = false;
        out += json_quote(key);
        out += ": ";
        out += json_f64(value, key);
    }
    out += R"(, "name_ids": )";
    out += std::to_string(k.name_ids);
    out += '}';

    out += ",\n" R"(  "solve_outputs": [)";
    for (std::size_t i = 0; i < stats.solve_outputs.size(); ++i) {
        if (i != 0) out += ", ";
        out += json_f64(stats.solve_outputs[i], "solve_outputs");
    }
    out += ']';

    const double us_per_step = wall_us / static_cast<double>(cli.steps);
    out += ",\n" R"(  "checksum": )";
    out += json_f64(checksum, "checksum");
    out += ",\n" R"(  "timing": {"wall_us": )";
    out += json_f64(wall_us, "wall_us");
    out += R"(, "us_per_step": )";
    out += json_f64(us_per_step, "us_per_step");
    out += R"(, "rtf": )";
    out += json_f64(1e6 * model.opt.timestep / us_per_step, "rtf");
    out += '}';

    out += ",\n" R"(  "provenance": {"language": "c++", "compiler": )";
    out += json_quote(provenance_compiler());
    out += R"(, "hardening": )";
    out += json_quote(provenance_hardening());
    out += R"(, "mujoco": )";
    out += json_quote(mj_versionString());
    out += R"(, "bundle": )";
    out += json_quote(bundle.dir.string());
    out += R"(, "via_pointer": )";
    out += bundle.via_pointer ? "true" : "false";
    out += R"(, "model_crc64": )";
    out += json_u64_hex(state.model_crc64);
    out += R"(, "state_crc64": )";
    out += json_u64_hex(state.state_crc64);
    out += "}\n}\n";
    return out;
}

// The declared exact/tolerance comparison policy documented for the
// conformance suite: "checksum" is the ordered accumulation of every stage
// checksum (float stages plus the integer name-id stage).
[[nodiscard]] double total_checksum(const RunStats &stats) {
    const StageChecksums &k = stats.ck;
    double total = k.trajectory_time + k.forces + k.energy + k.jacobians +
                   k.attachments + k.channels + k.efc + k.ledger + k.records +
                   k.batch + static_cast<double>(k.name_ids);
    total += stats.mix_sink;
    return total;
}

[[nodiscard]] int run(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() >= 2) {
        const std::string_view first{arguments[1]};
        if (first == "-h" || first == "--help") {
            std::cout << kUsage;
            return std::cout ? 0 : 1;
        }
        if (first == "--selftest") {
            // CRC64-ECMA check vector: "123456789" must produce
            // 0x6C40DF5F0B497347 — the Python-side crc64_ecma asserts the
            // same vector in the test suite.
            constexpr std::string_view kVector = "123456789";
            const std::uint64_t crc = proto_native_bench::detail::crc64_ecma(
                std::as_bytes(std::span<const char>{kVector}));
            std::cout << std::format(
                "crc64_ecma(\"123456789\")=0x{:016x} header_bytes={}\n", crc,
                proto_native_bench::detail::kHeaderBytes);
            if (crc != 0x6C40DF5F0B497347ULL)
                throw std::runtime_error("CRC64-ECMA check vector mismatch");
            return std::cout ? 0 : 1;
        }
    }
    const CliOptions cli = parse_cli(arguments.subspan(1));

    const ResolvedBundle bundle = resolve_artifacts(cli.artifacts);

    // Model load crosses the E1 boundary inside engine::load_model; a
    // MuJoCo fatal arrives as EngineFailure, a null return means the file
    // was unreadable/oversized, and plugin models are rejected up front.
    const std::string model_file = bundle.model.string();
    ModelPtr model{engine::load_model(model_file.c_str())};
    if (!model)
        throw std::runtime_error(model_file +
                                 ": model.mjb could not be loaded by MuJoCo");
    engine::require_supported_callbacks();
    DataPtr data{engine::make_data(model.get())};
    if (!data)
        throw std::runtime_error(model_file +
                                 ": mj_makeData returned null for model");
    validate_workload(cli.mode, *model);

    const State state = proto_native_bench::load_state(bundle.state,
                                                       bundle.model, *model);

    RunStats stats;
    if (cli.mode == Mode::heavy)
        stats.solve_outputs.reserve(
            static_cast<std::size_t>(kWarmupSteps + cli.steps) *
            (kAttachmentCount * 3 + kBatchSolve));

    GlueContext ctx;
    if (cli.mode != Mode::bare) {
        prepare_glue(*model, ctx, /*seeded=*/is_matched(cli.mode));
        if (cli.mode == Mode::heavy || cli.mode == Mode::operation_mix)
            prepare_heavy(ctx);
    }

    restore_counted(*model, *data, state, stats);
    run_phase(*model, *data, state, ctx, cli, kWarmupSteps, stats);
    const auto start = std::chrono::steady_clock::now();
    run_phase(*model, *data, state, ctx, cli, cli.steps, stats);
    const auto stop = std::chrono::steady_clock::now();

    const double wall_us =
        std::chrono::duration<double, std::micro>(stop - start).count();
    const double us_per_step = wall_us / static_cast<double>(cli.steps);
    const double steps_per_second =
        1e6 * static_cast<double>(cli.steps) / wall_us;
    const double rtf = 1e6 * model->opt.timestep / us_per_step;
    const double checksum = total_checksum(stats);
    if (!std::isfinite(checksum))
        throw std::runtime_error(
            "benchmark checksum is not finite — refusing to report a "
            "successful NaN sink");

    std::cout << std::format(
        "mode={} steps={} nv={} nefc~{} | {:.1f} us/step  {:.0f} steps/s  "
        "RTF={:.2f}x  (checksum {:.6e})\n",
        mode_name(cli.mode), cli.steps, model->nv, data->nefc, us_per_step,
        steps_per_second, rtf, checksum);
    if (!std::cout)
        throw std::runtime_error("failed to write benchmark result to stdout");

    if (!cli.emit_json.empty()) {
        const std::string report = build_report(cli, *model, bundle, state,
                                                ctx, stats, wall_us, checksum);
        std::ofstream out(cli.emit_json, std::ios::binary | std::ios::trunc);
        if (!out || !(out << report))
            throw std::runtime_error(cli.emit_json.string() +
                                     ": emit-json path is not writable");
    }
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "native_bench: " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "native_bench: unknown error\n";
        return 1;
    }
}
