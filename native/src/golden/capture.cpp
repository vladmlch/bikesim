#include "episode.hpp"
#include "capture_testing.hpp"
#include "model_image.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/research.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <type_traits>

namespace bike::golden {
namespace {
using io::Value;
Value number(double value) {
    if (!std::isfinite(value)) return Value{std::string(std::isnan(value) ? "nan" : value < 0 ? "-inf" : "inf")};
    std::array<char, 64> text{};
    const auto end = std::to_chars(text.data(), std::to_address(text.end()), value);
    if (end.ec != std::errc{}) throw std::runtime_error("golden number formatting failed");
    std::string lexeme(text.data(), end.ptr);
    if (lexeme.find_first_of(".eE") == std::string::npos) lexeme += ".0";
    return Value{io::Number{std::move(lexeme)}};
}
Value integer(std::size_t value) { return Value{io::Number{std::to_string(value)}}; }
Value value_from_wire(const runtime::Wire &wire) {
    return std::visit([](const auto &value) -> Value {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, runtime::WireObject>) {
            Value::Object out;
            for (const auto &[name, child] : value) out.emplace_back(name, value_from_wire(child));
            return Value{std::move(out)};
        } else if constexpr (std::is_same_v<T, runtime::WireArray>) {
            Value::Array out;
            for (const auto &child : value) out.push_back(value_from_wire(child));
            return Value{std::move(out)};
        } else if constexpr (std::is_same_v<T, double>) return number(value);
        else if constexpr (std::is_same_v<T, std::int64_t>) return Value{io::Number{std::to_string(value)}};
        else return Value{value};
    }, wire.value);
}
class Guard {
public:
    explicit Guard(std::atomic<bool> &flag) : flag_(flag) {
        bool expected = false;
        if (!flag_.compare_exchange_strong(expected, true, std::memory_order_acquire))
            throw std::runtime_error("golden capture: runtime operation already in progress");
    }
    ~Guard() { flag_.store(false, std::memory_order_release); }
    Guard(const Guard &) = delete;
    Guard &operator=(const Guard &) = delete;
    Guard(Guard &&) = delete;
    Guard &operator=(Guard &&) = delete;
private:
    std::atomic<bool> &flag_;
};
Value snapshots(const runtime::GoldenForceObservation &observation) {
    Value::Object out;
    constexpr std::array<std::string_view, 2> names{"front", "rear"};
    for (std::size_t side = 0; side < names.size(); ++side) {
        if (!observation.tire_snapshots[side]) continue;
        const auto &snapshot = *observation.tire_snapshots[side]; // NOLINT(bugprone-unchecked-optional-access) guarded by the disengaged-continue directly above
        Value::Array patches;
        for (const auto &patch : snapshot.patches) patches.push_back(Value{Value::Object{
            {"normal_load_n", number(patch.normal_load_n)}, {"tangent_force_n", number(patch.tangent_force_n)},
            {"slip_mps", number(patch.slip_mps)}, {"working_surface", Value{patch.working_surface}}}});
        out.emplace_back(std::string(names[side]), Value{Value::Object{
            {"time_s", number(snapshot.time_s)}, {"interval_id", Value{io::Number{std::to_string(snapshot.interval_id)}}},
            {"backend", Value{snapshot.backend}}, {"geometric_contact", Value{snapshot.geometric_contact}},
            {"effective_radius_m", number(snapshot.effective_radius_m)}, {"patches", Value{std::move(patches)}}}});
    }
    return Value{std::move(out)};
}
const io::NpyArray &initial_field(const EpisodeArtifact &episode, std::string_view key) {
    for (const auto &[name, value] : episode.initial) if (name == key) return value;
    throw std::invalid_argument("golden replay: missing initial state " + std::string(key));
}
std::vector<double> initial_vector(const EpisodeArtifact &episode, std::string_view name, std::size_t width) {
    const auto &array = initial_field(episode, name);
    if (array.shape != std::vector<std::size_t>{width})
        throw std::invalid_argument("golden replay: initial " + std::string(name) + " shape differs from model");
    return array.doubles();
}
runtime::RideControl decode_control(const io::Value &value) {
    // episode_compare._control ignores surplus fields and fills dataclass
    // defaults, including nested RiderPosture defaults.
    auto defaults = io::as_object(::rider::encode_intent_control(runtime::RideControl{}), "control defaults");
    const auto &source = io::as_object(value, "recorded control");
    for (auto &[name, target] : defaults) {
        const auto *supplied = io::find_member(source, name);
        if (supplied) target = *supplied;
    }
    for (auto &[name, target] : defaults) if (name == "posture" && !io::is_null(target)) {
        auto posture = io::as_object(::rider::encode_rider_posture(::rider::RiderPosture{}), "posture defaults");
        const auto &supplied = io::as_object(target, "recorded posture");
        for (auto &[key, member] : posture) if (const auto *found = io::find_member(supplied, key)) member = *found;
        target = Value{std::move(posture)};
    }
    return ::rider::decode_intent_control(Value{std::move(defaults)}, "golden.control");
}
void arrays(EpisodeArtifact &episode, const std::vector<runtime::GoldenForceObservation> &records,
            const mjModel &model) {
    const auto count = records.size(), nq = static_cast<std::size_t>(model.nq), nv = static_cast<std::size_t>(model.nv);
    const auto na = static_cast<std::size_t>(model.na), nu = static_cast<std::size_t>(model.nu);
    const auto stack = [&](auto member, std::size_t width) {
        std::vector<double> values;
        values.reserve(count * width);
        for (const auto &record : records) {
            const auto &row = record.*member;
            if (row.size() != width) throw std::runtime_error("golden observation width changed");
            values.insert(values.end(), row.begin(), row.end());
        }
        return io::double_array(values, {count, width});
    };
    episode.state_qpos = stack(&runtime::GoldenForceObservation::qpos, nq);
    episode.state_qvel = stack(&runtime::GoldenForceObservation::qvel, nv);
    episode.state_act = stack(&runtime::GoldenForceObservation::act, na);
    episode.state_warmstart = stack(&runtime::GoldenForceObservation::warmstart, nv);
    episode.ctrl_written = stack(&runtime::GoldenForceObservation::ctrl, nu);
    episode.qfrc_applied = stack(&runtime::GoldenForceObservation::qfrc_applied, nv);
    std::vector<double> times;
    times.reserve(records.size());
for (const auto &record : records) times.push_back(record.time_s);
    episode.state_time = io::double_array(times, {count});
    for (const auto &record : records) {
        for (const auto &[name, unused] : record.components) {
            (void)unused;
            if (std::ranges::find(episode.force_names, name) == episode.force_names.end()) episode.force_names.push_back(name);
        }
        for (const auto &name : record.tire_state_names)
            if (std::ranges::find(episode.tire_state_names, name) == episode.tire_state_names.end()) episode.tire_state_names.push_back(name);
    }
    const auto force_width = episode.force_names.size(), tire_width = episode.tire_state_names.size();
    if (nv && force_width && count > io::kMaxNumpyBytes / sizeof(double) / nv / force_width)
        throw std::invalid_argument("golden force matrix exceeds resource bound");
    if (tire_width && count > io::kMaxNumpyBytes / sizeof(double) / tire_width)
        throw std::invalid_argument("golden tyre matrix exceeds resource bound");
    std::vector<double> forces(count * force_width * nv, 0.);
    std::vector<double> tire(count * tire_width, std::numeric_limits<double>::quiet_NaN());
    for (std::size_t i = 0; i < count; ++i) {
        for (const auto &[name, row] : records[i].components) {
            if (row.size() != nv) throw std::runtime_error("golden force component width differs");
            const auto column = static_cast<std::size_t>(std::ranges::find(episode.force_names, name) - episode.force_names.begin());
            std::ranges::copy(row, std::span(forces).subspan((i * force_width + column) * nv, nv).begin());
        }
        if (records[i].tire_state_names.size() != records[i].tire_state.size()) throw std::runtime_error("golden tyre state schema differs");
        for (std::size_t j = 0; j < records[i].tire_state_names.size(); ++j) {
            const auto column = static_cast<std::size_t>(std::ranges::find(episode.tire_state_names, records[i].tire_state_names[j]) - episode.tire_state_names.begin());
            tire[i * tire_width + column] = records[i].tire_state[j];
        }
    }
    episode.forces = io::double_array(forces, {count, force_width, nv});
    episode.tire_state = io::double_array(tire, {count, tire_width});
}
} // namespace

// A single cold privileged seam avoids exposing live model/data pointers.
// It uses the runtime's ordinary advance_interval/accounting path and holds
// both existing reentrancy guards. Only this path requests force observations.
class RuntimeAccess {
public:
    static EpisodeArtifact run(runtime::NativeResearchRuntime &env,
        std::span<const runtime::RideControl> controls, const EpisodeArtifact *replay,
        void *test_exit_context = nullptr, testing::ForceExitMutation test_exit_mutation = nullptr) {
        const Guard research_guard(env.busy_);
        env.require_open();
        auto &runtime = *env.runtime_;
        runtime.require_open();
        if (replay == nullptr && (runtime.committed_step_ != 0 || runtime.committed_time_s_ != 0.))
            throw std::invalid_argument("golden capture requires a fresh physical t=0 state");
        if (env.episode_->window || env.episode_->delivery_pending)
            throw std::invalid_argument("golden capture cannot interrupt a pending research control");
        auto  const&stepper = *runtime.stepper_;
        const auto &model = *stepper.model();
        const auto minimum_width = static_cast<std::size_t>(model.nq + 3 * model.nv + model.na + model.nu + 1);
        if (minimum_width && controls.size() > io::kMaxNumpyBytes / sizeof(double) / minimum_width)
            throw std::invalid_argument("golden capture state matrix exceeds resource bound");
        for (const auto &control : controls)
            runtime::control_validate_for(control, runtime.physical_->rider_present(), runtime.physical_->drive_mode());
        const auto initial = replay ? std::optional(prepare_initial(model, *replay)) : std::nullopt;
        // The Python tool resets env.sim, then calls its physical step
        // directly; it does not submit delayed research control windows.
        // Validate all inputs before that reset changes any runtime state.
        runtime.reset();
        const Guard physical_guard(runtime.advancing_);
        if (initial) restore_initial(runtime, *initial);
        auto result = std::make_unique<EpisodeArtifact>();
        capture_initial(*result, model, *stepper.data());
        std::vector<runtime::GoldenForceObservation> records;
        records.reserve(controls.size());
        Value::Array tire_snapshots, tire_diagnostics;
        for (const auto &control : controls) {
            records.emplace_back();
            records.back().test_exit_context_ = test_exit_context;
            records.back().test_exit_mutation_ = test_exit_mutation;
            runtime.advance_interval(control, 0., 0., &records.back());
            result->controls.push_back(::rider::encode_intent_control(runtime.physical_->applied_control()));
            append_samples(runtime, *result);
            tire_snapshots.push_back(snapshots(records.back()));
            tire_diagnostics.push_back(value_from_wire(runtime::Wire(records.back().tire_diagnostics)));
        }
        runtime.flush_accounting();
        append_samples(runtime, *result);
        if (const auto &failure = runtime.accounting_->state().monitor.first_failure)
            result->first_failure = value_from_wire(failure->as_wire());
        std::set<std::string> channels;
        for (const auto &row : result->stored_rows)
            for (const auto &[name, unused] : io::as_object(row, "stored row")) { (void)unused; channels.insert(name); }
        result->channel_names.assign(channels.begin(), channels.end());
        // Python save receives env.sim.model AFTER the physical episode.
        // Preserve its live mutable model coefficients, not the setup MJB.
        result->model_image = detail::model_image(model);
        result->manifest = Value{Value::Object{{"producer", Value{std::string("bike-recording golden")}},
            {"model_sha256", Value{io::sha256(result->model_image)}}, {"steps", integer(result->rows.size())},
            {"physics_steps", integer(records.size())}, {"dt_s", number(model.opt.timestep)},
            {"tire_snapshots", Value{std::move(tire_snapshots)}}, {"tire_diagnostics", Value{std::move(tire_diagnostics)}}}};
        arrays(*result, records, model);
        return std::move(*result);
    }
private:
    struct InitialState {
        std::vector<double> qpos, qvel, act, warmstart, ctrl, time;
    };
    static InitialState prepare_initial(const mjModel &model, const EpisodeArtifact &episode) {
        InitialState out;
        out.qpos = initial_vector(episode, "qpos", static_cast<std::size_t>(model.nq));
        out.qvel = initial_vector(episode, "qvel", static_cast<std::size_t>(model.nv));
        out.act = initial_vector(episode, "act", static_cast<std::size_t>(model.na));
        out.warmstart = initial_vector(episode, "qacc_warmstart", static_cast<std::size_t>(model.nv));
        out.ctrl = initial_vector(episode, "ctrl", static_cast<std::size_t>(model.nu));
        out.time = initial_vector(episode, "time", 1);
        if (out.time.front() != 0.) throw std::invalid_argument("golden replay initial time must be zero");
        return out;
    }
    static void capture_initial(EpisodeArtifact &episode, const mjModel &model, const mjData &data) {
        const auto add = [&](std::string name, auto values) {
            episode.initial.emplace_back(std::move(name), io::double_array(values, {values.size()}));
        };
        add("qpos", std::views::counted(data.qpos, model.nq));
        add("qvel", std::views::counted(data.qvel, model.nv));
        add("act", std::views::counted(data.act, model.na));
        add("ctrl", std::views::counted(data.ctrl, model.nu));
        add("qacc_warmstart", std::views::counted(data.qacc_warmstart, model.nv));
        const std::array time{data.time}; add("time", std::span(time));
    }
    static void append_samples(runtime::NativeRideRuntime &runtime, EpisodeArtifact &episode) {
        const auto batch = runtime.accounting_->prepare_samples();
        for (const auto &sample : batch.samples) {
            // PhysicalSampleData::as_wire mirrors PhysicalSample.as_dict,
            // including full-period extra channels (runtime/samples.cpp).
            episode.stored_rows.push_back(storage_row(value_from_wire(runtime::Wire(sample->as_wire()))));
            episode.rows.push_back(flatten_row(episode.stored_rows.back()));
        }
        runtime.accounting_->acknowledge_samples(batch);
    }
    static void restore_initial(runtime::NativeRideRuntime &runtime, const InitialState &initial) {
        auto &stepper = *runtime.stepper_;
        auto &model = *stepper.model(); auto &data = *stepper.data();
        // Reset restored model/writer/controller state above. Overlay the
        // artifact's explicit engine state before the first force call;
        // replay never substitutes a newly solved qpos or warmstart.
        stepper.mutate([&] {
            std::ranges::copy(initial.qpos, std::views::counted(data.qpos, model.nq).begin());
            std::ranges::copy(initial.qvel, std::views::counted(data.qvel, model.nv).begin());
            std::ranges::copy(initial.act, std::views::counted(data.act, model.na).begin());
            std::ranges::copy(initial.ctrl, std::views::counted(data.ctrl, model.nu).begin());
            std::ranges::copy(initial.warmstart, std::views::counted(data.qacc_warmstart, model.nv).begin());
            data.time = initial.time.front();
            stepper.note_posvel_stale();
            engine::forward(&model, &data);
            stepper.note_posvel_fresh();
            engine::get_state(&model, &data, runtime.committed_integration_state_.data(), static_cast<int>(mjSTATE_INTEGRATION));
            runtime.next_integration_state_ = runtime.committed_integration_state_;
        });
        runtime.physical_->capture_presentation(runtime.committed_presentation_);
        runtime.next_presentation_ = runtime.committed_presentation_;
        runtime.cache_research_boundary();
    }
};

namespace {
EpisodeArtifact capture_impl(runtime::NativeResearchRuntime &env, std::size_t steps,
    const runtime::RideControl &control, void *test_exit_context,
    testing::ForceExitMutation test_exit_mutation) {
    const auto state = env.snapshot();
    if (state.step != 0 || state.time_s != 0.)
        throw std::invalid_argument("physical initial states may only be transferred before the first step");
    if (steps > 1000000) throw std::invalid_argument("golden capture exceeds one million physics steps");
    std::vector<runtime::RideControl> controls(steps, control);
    return RuntimeAccess::run(env, controls, nullptr, test_exit_context, test_exit_mutation);
}
} // namespace

EpisodeArtifact capture_episode(runtime::NativeResearchRuntime &env, std::size_t steps,
                                 const runtime::RideControl &control) {
    return capture_impl(env, steps, control, nullptr, nullptr);
}
EpisodeArtifact testing::capture_with_force_exit_mutation(runtime::NativeResearchRuntime &env,
    std::size_t steps, const runtime::RideControl &control, void *context, testing::ForceExitMutation mutation) {
    if (context == nullptr || mutation == nullptr)
        throw std::invalid_argument("golden force-exit regression requires a callback and context");
    return capture_impl(env, steps, control, context, mutation);
}
std::vector<Row> replay_episode(runtime::NativeResearchRuntime &env, const EpisodeArtifact &episode) {
    std::vector<runtime::RideControl> controls;
    controls.reserve(episode.controls.size());
    for (const auto &value : episode.controls) controls.push_back(decode_control(value));
    return RuntimeAccess::run(env, controls, &episode).rows;
}
std::vector<std::string> compare_episode(const EpisodeArtifact &golden,
    runtime::NativeResearchRuntime &env, double rtol, double atol) {
    if (!std::isfinite(rtol) || !std::isfinite(atol) || rtol < 0. || atol < 0.)
        throw std::invalid_argument("golden comparison tolerances must be finite and nonnegative");
    auto differences = compare_rows(golden.rows, replay_episode(env, golden), rtol, atol);
    Value failure;
    if (const auto current = env.snapshot().first_failure) failure = value_from_wire(current->as_wire());
    if (!same_failure(golden.first_failure, failure, rtol, atol))
        differences.emplace_back("first_failure differs between golden and candidate");
    return differences;
}
} // namespace bike::golden
