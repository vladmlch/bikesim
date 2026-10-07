// bench.cpp — native prototype benchmark for the bike_native Python-binding
// workload.
//
// A first stress-test build of a native implementation of the Python API
// being ported. The benchmark replays a recorded state through mj_step under
// three workloads:
//   - bare:  engine stepping only (frozen replayed state, periodic rewind)
//   - glue:  bare + the engine-call surface of the Python glue (forward
//            passes, point Jacobians, name lookups, qfrc assemblies, channel
//            writes)
//   - heavy: glue + representative telemetry/accounting volume (efc-row
//            unpack, attachment wrench solves, energy ledger, record churn,
//            period-batch solve)
//
// Every engine call crosses the bike_native_engine boundary (E1) so MuJoCo
// warnings/fatals surface as recoverable EngineFailure exceptions; the CLI
// reports them and returns nonzero instead of terminating. All extents are
// validated (state header vs. model, workload preconditions, checked
// products/sums) before any buffer is sized or copied.
//
// See README.md for the build, CLI syntax, artifact layout, and the honest
// list of workload-approximation limitations.

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
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
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
    "usage: native_bench {bare,glue,heavy} [steps] [rewind] "
    "--artifacts <directory>\n"
    "  bare:  mj_step only (frozen replayed state, periodic rewind)\n"
    "  glue:  bare + engine-call surface (forward passes, point Jacobians,\n"
    "         name lookups, 8 qfrc assemblies, 100 channel writes per step)\n"
    "  heavy: glue + telemetry/accounting volume (efc unpack, 5 attachment\n"
    "         wrench solves, energy ledger, record churn, batch solve)\n"
    "  steps/rewind: positive integers in [1, 10000000] "
    "(defaults 4000 and 250)\n"
    "  --artifacts: directory containing model.mjb and state.bin\n";

enum class Mode : std::uint8_t { bare, glue, heavy };

// Hard bound on step/rewind counts: keeps count * sizeof(double)-style
// products and loop planning inside checked-arithmetic range and rejects
// absurd run lengths early.
constexpr long kMaxCount = 10'000'000;
constexpr long kDefaultSteps = 4'000;
constexpr long kDefaultRewind = 250;
constexpr long kWarmupSteps = 500;
constexpr std::size_t kWriterCount = 8;
constexpr std::size_t kChannelCount = 100;
constexpr std::size_t kAttachmentCount = 5;
constexpr std::size_t kNameLookups = 10;
constexpr std::size_t kRecordChurn = 6;
constexpr long kBatchPeriod = 10;
constexpr long kBatchBatch = 6;
constexpr long kBatchSolve = 16;
// heavy maps attachment pairs onto physical bodies 1..2*kAttachmentCount;
// a model needs at least that many bodies (world + links).
constexpr int kHeavyMinBodies = 1 + 2 * static_cast<int>(kAttachmentCount);
constexpr int kHeavyMinNv = 3;
constexpr int kPointsPerAttachment = 7;
constexpr double kSolveEta = 1e-9;

// Element ceiling for every model-derived extent, mirroring
// state_format.hpp: element counts must stay byte-representable and inside
// the signed mjtSize domain used by mjData arrays.
constexpr std::size_t kMaxElements =
    std::numeric_limits<std::size_t>::max() / sizeof(mjtNum);

// Model dimensions are mjtSize (int64); negative extents are rejected before
// any size_t-sized buffer is formed from them.
[[nodiscard]] std::size_t model_extent(mjtSize dimension, std::string_view name) {
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
};

[[nodiscard]] const char *mode_name(Mode mode) {
    switch (mode) {
        case Mode::bare: return "bare";
        case Mode::glue: return "glue";
        case Mode::heavy: return "heavy";
    }
    return "unknown"; // unreachable: every Mode enumerant returns above
}

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
    throw std::invalid_argument("unknown mode '" + std::string(requested) +
                                "'; expected bare, glue, or heavy");
}

// CLI: mode, then [steps] [rewind], then required --artifacts <dir>.
// std::from_chars with full-consumption and bounded range checks replaces
// the prototype's atoi/std::exit handling; malformed input throws.
[[nodiscard]] CliOptions parse_cli(std::span<const char *const> args) {
    if (args.empty())
        throw std::invalid_argument(
            "mode is required: native_bench {bare,glue,heavy} [steps] [rewind] "
            "--artifacts <directory>");

    const Mode mode = parse_mode(std::string_view{args.front()});
    CliOptions options{.mode = mode,
                       .steps = kDefaultSteps,
                       .rewind = kDefaultRewind,
                       .artifacts = {}};
    int positionals = 0;
    bool have_artifacts = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view text{args[i]};
        if (text == "--artifacts") {
            if (i + 1 >= args.size())
                throw std::invalid_argument("--artifacts requires a directory");
            if (have_artifacts)
                throw std::invalid_argument("duplicate --artifacts");
            options.artifacts = std::string_view{args[i + 1]};
            if (options.artifacts.empty())
                throw std::invalid_argument("--artifacts requires a directory");
            have_artifacts = true;
            ++i;
            continue;
        }
        if (text.starts_with("--"))
            throw std::invalid_argument("unknown option '" + std::string(text) + "'");
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
            "missing required --artifacts <directory> containing model.mjb "
            "and state.bin");
    return options;
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
    const std::span<mjtNum> target =
        model_access::mutable_buffer(destination, static_cast<mjtSize>(extent), field);
    if (source.size() != target.size())
        throw std::invalid_argument(std::string(field) +
                                    ": state/model extent mismatch (" +
                                    std::to_string(source.size()) + " != " +
                                    std::to_string(target.size()) + ")");
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
        !std::ranges::all_of(point,
                             [](mjtNum value) { return std::isfinite(value); }))
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
    if (mode == Mode::heavy) {
        if (model.nbody < kHeavyMinBodies)
            throw std::invalid_argument(std::format(
                "mode 'heavy' requires nbody >= {} to resolve {} attachment "
                "body pairs (got {})",
                kHeavyMinBodies, kAttachmentCount, model.nbody));
        if (model.nv < kHeavyMinNv)
            throw std::invalid_argument(std::format(
                "mode 'heavy' requires nv >= {} (got {})", kHeavyMinNv,
                model.nv));
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
    double sink = 0;
};

void prepare_glue(const mjModel &model, GlueContext &ctx) {
    ctx.nv = model_extent(model.nv, "nv");
    ctx.nq = model_extent(model.nq, "nq");
    ctx.nu = model_extent(model.nu, "nu");
    ctx.nbody = model_extent(model.nbody, "nbody");
    ctx.acc.assign(ctx.nv, mjtNum{0});
    for (std::size_t w = 0; w < ctx.sources.size(); ++w) {
        std::vector<mjtNum> &source = ctx.sources[w];
        source.assign(ctx.nv, mjtNum{0});
        for (std::size_t j = 0; j < ctx.nv; ++j)
            source[j] = 0.01 * static_cast<mjtNum>(w + 1) *
                        static_cast<mjtNum>(static_cast<long>(j % 7) - 3);
    }
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

void glue_work(const mjModel &model, const mjData &data, const State &state,
               GlueContext &ctx, bool heavy, long index) {
    const std::size_t nv = ctx.nv;

    // 8 independent source fields summed into the accumulator, then staged
    // into qfrc_applied through the checked copy.
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
    const std::span<const mjtNum> qfrc_constraint = model_access::readonly_buffer(
        data.qfrc_constraint, model.nv, "qfrc_constraint");
    const std::span<const mjtNum> qacc =
        model_access::readonly_buffer(data.qacc, model.nv, "qacc");
    const std::span<const mjtNum> actuator_force = model_access::readonly_buffer(
        data.actuator_force, model.nu, "actuator_force");
    const std::span<const mjtNum> ctrl =
        model_access::readonly_buffer(data.ctrl, model.nu, "ctrl");

    // Record churn is materialized only inside the heavy mix; the records
    // vector stays scoped here so the channel fold below can consume it.
    std::vector<Record> records;
    if (heavy) {
        records.reserve(kRecordChurn);

        // efc-row unpack, batched to keep the prototype's representative
        // volume; the span is sized from the reported nefc only.
        const std::size_t n_read =
            std::min(static_cast<std::size_t>(std::max(data.nefc, 0)),
                     std::size_t{200});
        const std::span<const int> efc_id =
            model_access::readonly_buffer(data.efc_id, data.nefc, "efc_id");
        const std::span<const mjtNum> efc_pos =
            model_access::readonly_buffer(data.efc_pos, data.nefc, "efc_pos");
        const std::span<const mjtNum> efc_force =
            model_access::readonly_buffer(data.efc_force, data.nefc, "efc_force");
        std::array<long, 8> bucket{};
        std::array<mjtNum, 8> bpos{};
        for (std::size_t r = 0; r < n_read; ++r) {
            const std::size_t k = static_cast<std::size_t>(efc_id[r]) & 7U;
            bucket[k]++;
            bpos[k] += efc_pos[r] + efc_force[r];
        }
        ctx.sink += static_cast<double>(bucket[0]) +
                    static_cast<double>(bpos[1]);

        // Five attachment wrench solves over paired Jacobians — jacp and
        // jacp2 are separate model-sized buffers, fixing the prototype's
        // shared-buffer truncation.
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
                // jacp holds 3 rows (3*nv); the 7 per-attachment evals reuse
                // the rows cyclically — the prototype indexed k*nv directly
                // and ran off the end of its fixed buffer once k >= 3.
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
                ctx.sink += a * b * attachment.gain;
            }
        }

        // Energy ledger.
        double energy = 0.0;
        for (std::size_t j = 0; j < nv; ++j)
            energy += qfrc_constraint[j] * qvel[j] +
                      actuator_force[j % ctx.nu] * qvel[j];
        ctx.sink += energy * 1e-9;

        for (std::size_t k = 0; k < kRecordChurn; ++k)
            records.push_back(
                Record{.index = index, .a = qpos[0], .b = ctrl[0]});

        // Period-batch solve: 16 gradient-free iterations over six dof
        // triples; qacc spans are read through checked accessors only.
        if (index % kBatchPeriod == 0) {
            for (int c = 0; c < kBatchBatch; ++c) {
                const std::size_t c_idx = static_cast<std::size_t>(c);
                for (int it = 0; it < kBatchSolve; ++it) {
                    double g = 0.0;
                    for (int k2 = 0; k2 < 3; ++k2)
                        g += qacc[(static_cast<std::size_t>(k2) + c_idx) % nv];
                    ctx.solve[static_cast<std::size_t>(it)] -=
                        kSolveEta * g;
                }
                ctx.sink += ctx.solve[c_idx] * 1e-6;
            }
        }
    } else {
        // glue: paired point Jacobians on body 1 (nbody > 1 by gate).
        const std::span<const mjtNum> point = xpos.subspan(3, 3);
        point_jacobian(model, data, {.jacp = ctx.jacp, .jacr = ctx.jacr},
                       point, 1);
        point_jacobian(model, data,
                       {.jacp = ctx.jacp2, .jacr = std::span<mjtNum>{}},
                       point, 1);
    }

    // Name lookups resolve through the engine boundary; unknown names return
    // -1 and feed the sink so the lookup cost stays in the measurement.
    for (const char *name : ctx.names)
        ctx.sink += static_cast<double>(lookup_joint(model, name));

    // 100 channel writes per step.
    for (std::size_t c = 0; c < kChannelCount; ++c)
        ctx.channels[c] = qpos[c % ctx.nq] * static_cast<double>(c + 1) +
                          static_cast<double>(index) * 1e-9;

    if (heavy) {
        Record &last = records.back();
        last.b += ctx.channels[7];
        for (const Record &record : records)
            ctx.sink += record.a + record.b;
    }
    ctx.sink += ctx.channels[0];
}

void run_step(const mjModel &model, mjData &data, const State &state,
              GlueContext &ctx, Mode mode, long index) {
    if (mode == Mode::bare) {
        engine::step(&model, &data);
        return;
    }
    glue_work(model, data, state, ctx, mode == Mode::heavy, index);
}

void run_phase(const mjModel &model, mjData &data, const State &state,
               GlueContext &ctx, const CliOptions &cli, long count) {
    for (long i = 0; i < count; ++i) {
        if (i % cli.rewind == 0)
            restore_state(model, data, state);
        run_step(model, data, state, ctx, cli.mode, i);
    }
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
    }
    const CliOptions cli = parse_cli(arguments.subspan(1));

    const std::filesystem::path model_path = cli.artifacts / "model.mjb";
    const std::filesystem::path state_path = cli.artifacts / "state.bin";
    std::error_code error;
    if (!std::filesystem::is_regular_file(model_path, error) || error)
        throw std::runtime_error(model_path.string() +
                                 ": model.mjb is not a readable file");
    if (!std::filesystem::is_regular_file(state_path, error) || error)
        throw std::runtime_error(state_path.string() +
                                 ": state.bin is not a readable file");
    const std::string model_file = model_path.string();
    const std::string state_file = state_path.string();

    // Model load crosses the E1 boundary inside engine::load_model; a
    // MuJoCo fatal arrives as EngineFailure, a null return means the file
    // was unreadable/oversized, and plugin models are rejected up front.
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

    const State state = proto_native_bench::load_state(state_file, *model);
    restore_state(*model, *data, state);

    GlueContext ctx;
    if (cli.mode != Mode::bare) {
        prepare_glue(*model, ctx);
        if (cli.mode == Mode::heavy)
            prepare_heavy(ctx);
    }

    run_phase(*model, *data, state, ctx, cli, kWarmupSteps);
    const auto start = std::chrono::steady_clock::now();
    run_phase(*model, *data, state, ctx, cli, cli.steps);
    const auto stop = std::chrono::steady_clock::now();

    const double wall_us =
        std::chrono::duration<double, std::micro>(stop - start).count();
    const double us_per_step = wall_us / static_cast<double>(cli.steps);
    const double steps_per_second =
        1e6 * static_cast<double>(cli.steps) / wall_us;
    const double rtf = 1e6 * model->opt.timestep / us_per_step;

    std::cout << std::format(
        "mode={} steps={} nv={} nefc~{} | {:.1f} us/step  {:.0f} steps/s  "
        "RTF={:.2f}x  (sink {:.1f})\n",
        mode_name(cli.mode), cli.steps, model->nv, data->nefc, us_per_step,
        steps_per_second, rtf, ctx.sink);
    if (!std::cout)
        throw std::runtime_error("failed to write benchmark result to stdout");
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
