// W53: source-authored experiment; execution and selection belong to V09.
//
// Exact locked SDK inspected without executing it:
// native/dependencies.lock.json -> mujoco 3.12.0, Apache-2.0.
// Linux wheel SHA256:
//   267c3fc36b5a0536073557760e36c464da2ad9a744b047893db73fe273931ad1
// mujoco/include/mujoco/mujoco.h SHA256:
//   c7378e471cb951f4ff4bd4801139be41495a3df82c5ae643c4134ded850fb7ef
// Exact declarations in that header (lines 19 and 1629):
//   #define mjVERSION_HEADER 3012000
//   MJAPI void mju_threadpool(mjData* d, int nthread);
// The obsolete mju_threadPoolCreate/mju_bindThreadPool API is not used.
//
// Upstream tag 3.12.0 resolves to 13827e9ee56f097f57acf69ae52b078f9839682d:
// https://github.com/google-deepmind/mujoco/blob/13827e9ee56f097f57acf69ae52b078f9839682d/src/engine/engine_thread.cc
// https://github.com/google-deepmind/mujoco/blob/13827e9ee56f097f57acf69ae52b078f9839682d/src/engine/engine_io.c
// Source inspection establishes nthread BACKGROUND workers plus the caller;
// zero removes the pool. mj_deleteData removes it before freeing mjData.
// The runtime's existing mjData owner therefore also owns this test-only pool.
// No application factory, engine ownership API or production flag is changed.
#include "support/migration_test.hpp"
#include "spatial/support/ride_qualification.hpp"
#include "runtime/spatial_checkpoint_internal.hpp"
#include "runtime/provenance.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
namespace ride = spatial_tests::ride;
namespace io = bike::io;
namespace sp = bike::spatial;
using namespace runtime::setupvalue;
using SteadyClock = std::chrono::steady_clock;
static_assert(mjVERSION_HEADER == 3012000, "W53 must be reviewed against a changed MuJoCo lock");
static_assert(std::is_same_v<decltype(&mju_threadpool), void (*)(mjData*, int)>);
static_assert(SteadyClock::is_steady);
constexpr double duration_s = 30.;
constexpr unsigned parallel_threads = 4;
constexpr unsigned repetitions = 3;

bool qualifies_engine_pool(double baseline_wall_s, double candidate_wall_s, bool same_bits) {
    if (!std::isfinite(baseline_wall_s) || baseline_wall_s <= 0. ||
        !std::isfinite(candidate_wall_s) || candidate_wall_s <= 0.)
        throw std::invalid_argument("engine_pool_probe: positive finite full-run times required");
    return same_bits && candidate_wall_s <= .90 * baseline_wall_s;
}

void helper_controls() {
    bt::require(!qualifies_engine_pool(10., 8., false), "speed cannot override byte mismatch");
    bt::require(qualifies_engine_pool(10., 9., true), "ten percent full-run gain qualifies");
    bt::require(!qualifies_engine_pool(10., 9.000001, true), "the ten percent threshold is not rounded");
    bt::invalid([] { (void)qualifies_engine_pool(10., 0., true); });
    bt::invalid([] { (void)qualifies_engine_pool(0., 1., true); });
    bt::invalid([] { (void)qualifies_engine_pool(std::numeric_limits<double>::quiet_NaN(), 1., true); });
    bt::invalid([] { (void)qualifies_engine_pool(1., std::numeric_limits<double>::infinity(), true); });
}

void set_engine_threads(mjData* data, unsigned total_threads) {
    if (!data || total_threads == 0 || total_threads > 64)
        throw std::invalid_argument("engine_pool_probe.engine_threads: expected 1..64 and owned data");
    // The C API is implemented in C++. Preserve a possible allocation/thread
    // exception outside the engine's noexcept error frame as well as MuJoCo's
    // fatal-error channel. No nontrivial automatic owner crosses that frame.
    struct Call { mjData* data; int workers; std::exception_ptr failure; };
    Call call{data, static_cast<int>(total_threads - 1), {}};
    engine::ErrorBuffer error;
    const bool ok = engine::invoke([](void* raw) noexcept {
        auto& c = *static_cast<Call*>(raw);
        try { mju_threadpool(c.data, c.workers); }
        catch (...) { c.failure = std::current_exception(); }
    }, &call, error);
    if (!ok) engine::throw_failure(error);
    if (call.failure) std::rethrow_exception(call.failure);
    bt::require((data->threadpool != 0) == (total_threads > 1),
        "engine serial control has no background pool and parallel control has an owned pool");
}

double seconds(SteadyClock::time_point start, SteadyClock::time_point end) {
    const double elapsed = std::chrono::duration<double>(end - start).count();
    if (!std::isfinite(elapsed) || elapsed <= 0.)
        throw std::invalid_argument("engine_pool_probe.wall_s: positive finite elapsed time required");
    return elapsed;
}

// Every byte belongs to a causal owner or a published physical result. Pool
// pointers, worker bookkeeping and wall clocks are deliberately not state.
// The complete W42 plant continuation adds the partial accounting period,
// histories, accepted truth/IMU, mutable engine rows and controller inventory.
std::vector<std::byte> complete_bytes(runtime::NativeRideRuntime& live) {
    auto result = spatial_tests::complete_physics_state_bytes(live);
    const auto append = [&](std::span<const std::byte> bytes) {
        const std::uint64_t size = bytes.size();
        const auto header = std::as_bytes(std::span{&size, std::size_t{1}});
        result.insert(result.end(), header.begin(), header.end());
        result.insert(result.end(), bytes.begin(), bytes.end());
    };
    const auto checkpoint = io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(live));
    append(std::as_bytes(std::span{checkpoint}));
    auto& stepper = ride::SpatialRideAccess::stepper(live);
    const auto* model = stepper.model();
    const auto* data = stepper.data();
    for (const double* force : {data->qfrc_applied, data->qfrc_constraint, data->qfrc_actuator,
                               data->qfrc_passive, data->qfrc_bias, data->qfrc_smooth})
        append(std::as_bytes(model_access::readonly_buffer(force, model->nv)));
    append(std::as_bytes(model_access::readonly_buffer(data->sensordata, model->nsensordata)));
    for (const auto& patches : stepper.spatial_tire_writer().state().patches) {
        append(std::as_bytes(std::span{patches.ids}));
        append(std::as_bytes(std::span{patches.slot_used}));
        append(std::as_bytes(std::span{patches.element_to_patch_slot}));
    }
    return result;
}

struct Block {
    std::int64_t target = 0;
    runtime::RideControl control;
    double brake = 0.;
};
struct Fixture {
    std::unique_ptr<runtime::RuntimeBootstrap> bootstrap;
    std::vector<Block> blocks;
    std::int64_t final_step = 0, checkpoint_stride = 0;
    Value identity;
};

Fixture prepare_fixture() {
    auto request = spatial_tests::make_startup_request(sp::RootKind::free6, sp::TyreTier::fast, true, 3.);
    const auto manifest = ride::example("research/spatial_flat.toml");
    const auto pin = ride::pin_profile(*request, manifest);
    const auto trace = ride::demand_trace(manifest);
    request->runtime.strict = true;
    request->runtime.async_telemetry = false;
    request->runtime.record_retention = runtime::RetentionMode::off;
    // All four cases reuse one serial-settled bootstrap. The W36 F model,
    // tyre/steering pins and demand trace stay fixed; its final demand is held
    // through 30 seconds for the predeclared performance duration.
    spatial_tests::OwnedStartup settled(std::move(*request));
    Fixture out;
    out.bootstrap = std::move(settled.bootstrap);
    const auto& config = out.bootstrap->config;
    const double dt = config.timestep_s;
    out.final_step = static_cast<std::int64_t>(std::llround(duration_s / dt));
    out.checkpoint_stride = static_cast<std::int64_t>(std::llround(5. / dt));
    const auto stride = static_cast<std::int64_t>(config.control_period_steps);
    bt::require(out.final_step > 0 && out.checkpoint_stride > 0 && stride > 0 &&
        out.final_step % stride == 0 && out.checkpoint_stride % stride == 0,
        "probe's control, checkpoint and duration boundaries use exact physical ticks");
    std::size_t at = 0;
    for (std::int64_t start = 0; start < out.final_step; start += stride) {
        const double time = static_cast<double>(start) * dt;
        while (at + 1 < trace.size() && trace[at + 1].time_s <= time) ++at;
        const auto demand = trace.empty() ? ride::Demand{} : trace[at];
        Block block;
        block.target = std::min(start + stride, out.final_step);
        block.control.motor_torque_nm = 85. * demand.torque_fraction;
        block.control.motor_limit_nm = 85.;
        block.control.human_torque_nm = 0.;
        block.brake = demand.brake_fraction;
        out.blocks.push_back(std::move(block));
    }
    const auto execution = runtime::runtime_identity();
    const auto manifest_bytes = spatial_tests::bytes(manifest);
    out.identity = Value{Object{
        {"fixture", text("W36/F: spatial_flat.toml; final demand extended to 30 s")},
        {"demand_and_profile_sha256", text(io::sha256(manifest_bytes))},
        {"model_sha256", text(out.bootstrap->model.sha256)},
        {"terrain_sha256", text(config.terrain3d->fingerprint_hex())},
        {"tyre_sha256", text(pin.tyre)}, {"steer_sha256", text(pin.steer)},
        {"root", text("free6")}, {"tier", text("fast")}, {"timestep_s", real(dt)},
        {"duration_s", real(duration_s)}, {"strict", Value{true}},
        {"record_retention", text("off")}, {"sensors", text("native truth/IMU; no research sensor pipeline")},
        {"mujoco_version", text(execution.mujoco_version)},
        {"native_source_sha256", text(execution.native_source_sha256)},
        {"build_context", io::parse_json(execution.build_context_json)},
        {"provider_policy", io::parse_json(runtime::numerics_provider_policy_json())}}};
    return out;
}

struct Run {
    double wall_s = 0., setup_s = 0., simulated_s = 0.;
    std::int64_t steps = 0;
    bool valid = false;
    std::string terminal_reason;
    std::vector<std::byte> final_state;
    std::vector<std::vector<std::byte>> checkpoints;
};

Run run(const Fixture& fixture, unsigned tyre_threads, unsigned engine_threads, bool checkpoints) {
    Run out;
    const auto setup_begin = SteadyClock::now();
    auto live = std::make_unique<runtime::NativeRideRuntime>(*fixture.bootstrap);
    auto& stepper = ride::SpatialRideAccess::stepper(*live);
    // Cold, test-only selection keeps all bootstrap/config/identity bytes
    // identical. The production --threads propagation is covered separately
    // by W52 thread_invariance. Actual worker count is checked, not assumed.
    stepper.spatial_tire_writer().set_worker_threads(tyre_threads);
    bt::require(stepper.spatial_tire_writer().model().worker_threads() == tyre_threads,
        "probe uses the requested tyre pool, not a serial substitute");
    set_engine_threads(stepper.data(), engine_threads);
    live->set_completion_step(fixture.final_step);
    const auto initial = live->snapshot();
    bt::require(initial.step == 0 && initial.spatial && !initial.spatial->hold.active &&
        initial.spatial->bootstrap_phase == sp::BootstrapPhase::scored,
        "each probe begins at the same scored, hold-free bootstrap");
    if (checkpoints) out.checkpoints.push_back(complete_bytes(*live));
    out.setup_s = seconds(setup_begin, SteadyClock::now());
    const auto begin = SteadyClock::now();
    for (const auto& block : fixture.blocks) {
        const auto result = live->advance(block.target, block.control, block.brake, block.brake, std::nullopt);
        out.steps = result.step;
        out.simulated_s = result.time_s;
        if (checkpoints && result.step % fixture.checkpoint_stride == 0 && result.step < fixture.final_step)
            out.checkpoints.push_back(complete_bytes(*live));
        if (result.outcome) { out.terminal_reason = *result.outcome; break; }
    }
    // Include the final accounting drain in the full-run timing. Construction,
    // settlement and cold comparison/export are outside the measurement.
    live->flush();
    out.wall_s = seconds(begin, SteadyClock::now());
    const auto final = live->snapshot();
    out.valid = out.steps == fixture.final_step && final.spatial &&
        final.spatial->outcome.latched.reason == sp::SpatialReason::requested_completion &&
        final.spatial->outcome.latched.valid && !final.spatial->hold.active &&
        !live->take_pending_warning();
    if (out.terminal_reason.empty()) out.terminal_reason = "no_terminal_outcome";
    if (!live->checkpoint_unavailable_reason()) {
        out.final_state = complete_bytes(*live);
        if (checkpoints) out.checkpoints.push_back(out.final_state);
    }
    return out;
}

struct Case {
    unsigned tyre_threads = 1, engine_threads = 1;
    Run warmup, comparison;
    std::vector<Run> runs;
    double median_wall_s = 0.;
    bool valid = false, repeat_same_bits = false, same_serial_bits = false;
};
Value row(const Run& run, bool warmup) {
    return Value{Object{{"wall_s", real(run.wall_s)}, {"setup_s", real(run.setup_s)},
        {"simulated_s", real(run.simulated_s)}, {"rtf", real(run.simulated_s / run.wall_s)},
        {"steps", integer(run.steps)}, {"valid", Value{run.valid}}, {"warmup", Value{warmup}},
        {"terminal_reason", text(run.terminal_reason)},
        {"complete_state_sha256", run.final_state.empty() ? Value{} : text(io::sha256(run.final_state))}}};
}
Value case_report(const Case& item) {
    Array rows{row(item.warmup, true)}, checkpoint_hashes;
    for (const auto& run : item.runs) rows.push_back(row(run, false));
    for (const auto& state : item.comparison.checkpoints) checkpoint_hashes.push_back(text(io::sha256(state)));
    return Value{Object{{"tyre_threads", integer(item.tyre_threads)},
        {"engine_threads", integer(item.engine_threads)}, {"engine_background_workers", integer(item.engine_threads - 1)},
        {"runs", Value{std::move(rows)}}, {"all_full_runs_valid", Value{item.valid}},
        {"repeat_same_bits", Value{item.repeat_same_bits}}, {"same_serial_bits", Value{item.same_serial_bits}},
        {"median_wall_s", item.valid ? real(item.median_wall_s) : Value{}},
        {"checkpoint_sha256", Value{std::move(checkpoint_hashes)}},
        {"checkpoint_run_timing", text("diagnostic: excluded from measured median")}}};
}

void probe() {
    const auto fixture = prepare_fixture();
    std::array<Case, 4> cases;
    cases[1].tyre_threads = parallel_threads;
    cases[2].engine_threads = parallel_threads;
    cases[3].tyre_threads = cases[3].engine_threads = parallel_threads;
    // Warm every combination first. Rotate measured-case order between rounds
    // so the engine candidate is not systematically the last/hottest workload.
    for (auto& item : cases) item.warmup = run(fixture, item.tyre_threads, item.engine_threads, false);
    for (unsigned round = 0; round < repetitions; ++round)
        for (std::size_t offset = 0; offset < cases.size(); ++offset) {
            auto& item = cases[(offset + round) % cases.size()];
            item.runs.push_back(run(fixture, item.tyre_threads, item.engine_threads, false));
        }
    for (auto& item : cases) {
        item.comparison = run(fixture, item.tyre_threads, item.engine_threads, true);
        item.valid = item.warmup.valid && item.comparison.valid;
        item.repeat_same_bits = !item.comparison.final_state.empty() &&
            item.warmup.final_state == item.comparison.final_state;
        std::vector<double> walls;
        for (const auto& measured : item.runs) {
            item.valid = item.valid && measured.valid;
            item.repeat_same_bits = item.repeat_same_bits && measured.final_state == item.comparison.final_state;
            if (measured.valid) walls.push_back(measured.wall_s);
        }
        if (item.valid && walls.size() == repetitions) {
            std::ranges::sort(walls);
            item.median_wall_s = walls[walls.size() / 2];
        }
    }
    for (auto& item : cases)
        item.same_serial_bits = item.repeat_same_bits && cases[0].repeat_same_bits &&
            item.comparison.checkpoints == cases[0].comparison.checkpoints;
    Array reports;
    for (const auto& item : cases) reports.push_back(case_report(item));
    Array decisions;
    // Isolate engine speedup at each fixed tyre count, then also compare to
    // the fastest byte-invariant, valid engine-serial CPU baseline. Otherwise
    // an engine win over a deliberately slower tyre setting would qualify.
    const Case* best_cpu = nullptr;
    for (const std::size_t index : {std::size_t{0}, std::size_t{1}}) {
        const auto& candidate = cases[index];
        if (candidate.valid && candidate.same_serial_bits &&
            (!best_cpu || candidate.median_wall_s < best_cpu->median_wall_s)) best_cpu = &candidate;
    }
    for (std::size_t index = 2; index < cases.size(); ++index) {
        const auto& candidate = cases[index];
        const auto& paired = cases[index - 2];
        const bool comparable = best_cpu && candidate.valid && paired.valid;
        const bool same_bits = candidate.same_serial_bits && paired.same_serial_bits;
        const bool gain_at_fixed_tyre = comparable &&
            qualifies_engine_pool(paired.median_wall_s, candidate.median_wall_s, same_bits);
        const bool qualifies = comparable && gain_at_fixed_tyre &&
            qualifies_engine_pool(best_cpu->median_wall_s, candidate.median_wall_s, same_bits);
        const std::string_view decision = !candidate.valid || !paired.valid
            ? "blocked-invalid-workload"
            : !best_cpu ? "blocked-no-invariant-cpu-baseline"
            : qualifies ? "eligible-for-V09-review" : "not_selected";
        decisions.push_back(Value{Object{{"tyre_threads", integer(candidate.tyre_threads)},
            {"engine_threads", integer(candidate.engine_threads)}, {"same_bits", Value{same_bits}},
            {"ten_percent_gain_at_fixed_tyre", comparable ? Value{gain_at_fixed_tyre} : Value{}},
            {"ten_percent_gain_over_best_cpu", comparable ? Value{qualifies} : Value{}},
            {"decision", text(decision)}}});
    }
    std::cout << io::canonical_json(Value{Object{{"suite", text("spatial.engine_pool_probe")},
        {"probe_schema", integer(1)}, {"api", text("mju_threadpool(mjData*, int)")},
        {"mjVERSION_HEADER", integer(mjVERSION_HEADER)}, {"identity", fixture.identity},
        {"duration_s", real(duration_s)}, {"measured_repetitions", integer(repetitions)},
        {"cases", Value{std::move(reports)}}, {"engine_decisions", Value{std::move(decisions)}},
        {"production_pool_enabled", Value{false}}}}) << '\n';
    bt::require(std::ranges::all_of(cases, [](const Case& item) { return item.valid; }),
        "engine pool evidence requires full valid episodes; a surviving prefix cannot qualify");
    // A byte mismatch or a lack of speedup is a successful negative probe,
    // represented as not_selected, never a reason to relax physics checks.
}
} // namespace

int main() { return bt::run([] { helper_controls(); probe(); }); }
