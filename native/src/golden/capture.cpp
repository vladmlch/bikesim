#include "episode.hpp"
#include "capture_testing.hpp"
#include "spatial_compare.hpp"
#include "model_image.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/research.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/spatial_checkpoint_internal.hpp"
#include "../runtime/provenance.hpp"
#include "../platform/provenance.hpp"
#include "../engine_identity.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <map>
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
// NOLINTNEXTLINE(misc-no-recursion) bounded owned sample tree, before legacy number-to-string serialization
void require_finite_spatial_wire(const runtime::Wire& wire,const std::string& path) {
    std::visit([&](const auto& value) {
        using T=std::decay_t<decltype(value)>;
        if constexpr(std::is_same_v<T,double>) {
            if(!std::isfinite(value))
                throw std::invalid_argument("physical_invalid: "+path+" contains nonfinite numeric evidence");
        } else if constexpr(std::is_same_v<T,runtime::WireObject>) {
            for(const auto& [name,child]:value)require_finite_spatial_wire(child,std::string(path).append(".").append(name));
        } else if constexpr(std::is_same_v<T,runtime::WireArray>) {
            for(std::size_t i=0;i<value.size();++i)
                require_finite_spatial_wire(value[i],path+"["+std::to_string(i)+"]");
        }
    },wire.value);
}
Value capture_wire(const runtime::Wire& wire,bool spatial,const std::string& path) {
    // A real string such as "nan" is valid evidence. Inspect numeric doubles
    // while their type is still present; keep the planar serialization exact.
    if(spatial)require_finite_spatial_wire(wire,path);
    return value_from_wire(wire);
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
const Value& initial_comparison_checkpoint(const EpisodeArtifact& episode) {
    const auto& manifest=io::as_object(episode.manifest,"golden manifest");
    if(const auto* candidate=io::find_member(manifest,"spatial_startup_candidate"))return *candidate;
    return runtime::setupvalue::field(episode.manifest,"spatial_checkpoint_initial");
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
        void *test_exit_context = nullptr, testing::ForceExitMutation test_exit_mutation = nullptr,
        bool restore_spatial_reference = true) {
        const Guard research_guard(env.busy_);
        env.require_open();
        auto &target = *env.runtime_;
        target.require_open();
        if (replay == nullptr && (target.committed_step_ != 0 || target.committed_time_s_ != 0.))
            throw std::invalid_argument("golden capture requires a fresh physical t=0 state");
        if (env.episode_->window || env.episode_->delivery_pending)
            throw std::invalid_argument("golden capture cannot interrupt a pending research control");
        if(target.config_.spatial && (env.episode_->requested_count!=0 || !env.episode_->commands.empty()))
            throw std::invalid_argument("spatial golden requires an inactive research host; reset the research episode first");
        const auto& input_model=*target.stepper_->model();
        const auto minimum_width = static_cast<std::size_t>(input_model.nq + 3 * input_model.nv + input_model.na + input_model.nu + 1);
        if (minimum_width && controls.size() > io::kMaxNumpyBytes / sizeof(double) / minimum_width)
            throw std::invalid_argument("golden capture state matrix exceeds resource bound");
        for (const auto &control : controls)
            runtime::control_validate_for(control, target.physical_->rider_present(), target.physical_->drive_mode());
        const auto initial = replay ? std::optional(prepare_initial(input_model, *replay)) : std::nullopt;
        std::unique_ptr<runtime::NativeRideRuntime> staged_reference;
        std::optional<int> reference_generation;
        if(replay && target.config_.spatial) {
            const auto checkpoint=spatial_reference(*replay,env.config_.metadata);
            if(restore_spatial_reference) {
                if(io::as_string(runtime::setupvalue::field(replay->manifest,"source_identity"),"source_identity")!=
                   runtime::runtime_identity().native_source_sha256)
                    throw std::invalid_argument("incompatibility: spatial golden source identity differs");
                staged_reference=runtime::spatial::CheckpointAccess::prepare(checkpoint,target);
                if(staged_reference->committed_step_!=0 || staged_reference->committed_time_s_!=0.)
                    throw std::invalid_argument("golden replay: spatial checkpoint must describe t=0");
                validate_initial(*initial,*staged_reference->stepper_->model(),*staged_reference->stepper_->data()); // NOLINT(bugprone-unchecked-optional-access) initial is engaged iff replay, required by the enclosing guard; a prepared runtime always owns stepper_
                reference_generation=staged_reference->committed_generation_;
            } else {
                // Reviewed cross-platform comparison runs the target's own
                // complete startup. Validate redundant reference arrays on a
                // disposable data owner without importing foreign execution
                // identities or mixing its engine state into the target.
                // NOLINTNEXTLINE(modernize-make-unique) decode the complete bootstrap directly into cold heap storage
                const std::unique_ptr<runtime::BootstrapState> decoded{new runtime::BootstrapState(runtime::decode_bootstrap_state(checkpoint.bootstrap2,input_model))};
                const std::unique_ptr<mjData,decltype(&mj_deleteData)> data(engine::make_data(&input_model),&mj_deleteData);
                if(!data)throw std::runtime_error("golden replay: cannot allocate initial-state validation arena");
                engine::set_state(&input_model,data.get(),decoded->integration_state.data(),mjSTATE_INTEGRATION);
                if(decoded->step!=0)throw std::invalid_argument("golden replay: spatial checkpoint must describe t=0");
                validate_initial(*initial,input_model,*data); // NOLINT(bugprone-unchecked-optional-access) initial is engaged iff replay, required by the enclosing guard
                reference_generation=decoded->generation;
            }
        }
        // The Python tool resets env.sim, then calls its physical step
        // directly; it does not submit delayed research control windows.
        // Validate all inputs before that reset changes any runtime state.
        target.reset();
        if(reference_generation && target.committed_generation_!=*reference_generation) {
            // Compare the complete fresh startup in the reference epoch even
            // when this host has already replayed it. Rebuild only the empty
            // t=0 accounting/publication, preserving the target's own physics.
            const Guard generation_guard(target.advancing_);
            const int previous_generation=target.physical_->generation();
            target.physical_->set_generation(*reference_generation);
            try { target.initialize_accounting(); }
            catch(...) {
                // initialize_accounting stages before its no-throw commit.
                target.physical_->set_generation(previous_generation);
                throw;
            }
        }
        const auto startup_candidate=target.config_.spatial ? runtime::spatial::CheckpointAccess::capture_bootstrap(target) : Value{};
        if(staged_reference)env.runtime_.swap(staged_reference);
        // A full restore can replace all owners. Bind the simulation faces
        // only after publication, and keep their guard on the new runtime.
        auto& runtime=*env.runtime_;
        const auto& stepper=*runtime.stepper_;
        const auto& model=*stepper.model();
        const auto spatial_initial=runtime.config_.spatial ?
            (replay && restore_spatial_reference ? runtime::setupvalue::field(replay->manifest,"spatial_checkpoint_initial") : startup_candidate) : Value{};
        std::optional<Guard> physical_guard;physical_guard.emplace(runtime.advancing_);
        if (initial && !runtime.config_.spatial) restore_initial(runtime, *initial);
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
            tire_diagnostics.push_back(capture_wire(runtime::Wire(records.back().tire_diagnostics),
                runtime.config_.spatial.has_value(),"tire_diagnostics"));
        }
        runtime.flush_accounting();
        append_samples(runtime, *result);
        if (const auto &failure = runtime.accounting_->state().monitor.first_failure)
            result->first_failure = capture_wire(failure->as_wire(),runtime.config_.spatial.has_value(),"first_failure");
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
        physical_guard.reset();
        if(runtime.config_.spatial) {
            auto& manifest=std::get<Value::Object>(result->manifest.data);
            manifest.emplace_back("spatial_checkpoint_initial",spatial_initial);
            if(replay && restore_spatial_reference)
                manifest.emplace_back("spatial_startup_candidate",startup_candidate);
            manifest.emplace_back("spatial_checkpoint_final",runtime::spatial::CheckpointAccess::capture_bootstrap(runtime));
#if defined(__APPLE__) && defined(__aarch64__)
            manifest.emplace_back("spatial_platform",Value{std::string("macos-arm64")});
#elif defined(__linux__) && defined(__aarch64__)
            manifest.emplace_back("spatial_platform",Value{std::string("linux-aarch64")});
#else
            manifest.emplace_back("spatial_platform",Value{platform::build_identity().platform});
#endif
            manifest.emplace_back("source_identity",Value{runtime::runtime_identity().native_source_sha256});
            manifest.emplace_back("execution_identity",Value{platform::execution_identity()});
            manifest.emplace_back("provider",Value{engine::identity().numeric_provider});
            const auto& metadata=io::as_object(env.config_.metadata,"spatial golden configuration");
            const auto* config_hash=io::find_member(metadata,"configuration_sha256");
            if(!config_hash)throw std::invalid_argument("spatial golden requires captured configuration identity");
            manifest.emplace_back("config_sha256",*config_hash);
            manifest.emplace_back("calibration_sha256",Value{runtime.bootstrap_.spatial->calibration_sha256}); // NOLINT(bugprone-unchecked-optional-access) bootstrap_.spatial engagement follows config_.spatial at runtime construction
            manifest.emplace_back("terrain_sha256",Value{runtime.bootstrap_.spatial->terrain_sha256}); // NOLINT(bugprone-unchecked-optional-access) bootstrap_.spatial engagement follows config_.spatial at runtime construction
        }
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
    static runtime::spatial::SpatialCheckpoint spatial_reference(const EpisodeArtifact& episode,const Value& metadata) {
        namespace sv=runtime::setupvalue;
        if(episode.initial.size()!=6)
            throw std::invalid_argument("golden replay: spatial initial engine field set differs");
        if(io::canonical_json(sv::field(episode.manifest,"config_sha256"))!=
           io::canonical_json(sv::field(metadata,"configuration_sha256")))
            throw std::invalid_argument("incompatibility: spatial golden configuration identity differs");
        const auto& bootstrap=sv::field(episode.manifest,"spatial_checkpoint_initial");
        runtime::spatial::SpatialCheckpoint checkpoint{
            .bootstrap2=bootstrap,.research=runtime::spatial::inactive_research_checkpoint(),
            .sensor_v2=runtime::spatial::inactive_sensor_checkpoint(),
            .policy=Value{Value::Object{{"resumable",Value{false}},{"state",Value{}}}},
            .identity=Value{Value::Object{{"model_digest",sv::field(bootstrap,"model_digest")},
                {"config_sha256",sv::field(episode.manifest,"config_sha256")},
                {"terrain_sha256",sv::field(episode.manifest,"terrain_sha256")},
                {"calibration_sha256",sv::field(episode.manifest,"calibration_sha256")},
                {"solver",sv::field(episode.manifest,"execution_identity")},
                {"provider",sv::field(episode.manifest,"provider")},{"hold_removed",Value{true}}}}};
        (void)runtime::spatial::encode_checkpoint(checkpoint);
        return checkpoint;
    }
    static void validate_initial(const InitialState& initial,const mjModel& model,const mjData& data) {
        const auto equal=[&](const std::vector<double>& expected,const double* values,mjtSize count,std::string_view name) {
            const auto actual=std::views::counted(values,count);
            if(!std::ranges::equal(std::as_bytes(std::span{expected}),std::as_bytes(actual)))
                throw std::invalid_argument("golden replay: initial "+std::string(name)+" disagrees with spatial checkpoint");
        };
        equal(initial.qpos,data.qpos,model.nq,"qpos");equal(initial.qvel,data.qvel,model.nv,"qvel");
        equal(initial.act,data.act,model.na,"act");equal(initial.ctrl,data.ctrl,model.nu,"ctrl");
        equal(initial.warmstart,data.qacc_warmstart,model.nv,"qacc_warmstart");
        equal(initial.time,&data.time,1,"time");
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
            episode.stored_rows.push_back(storage_row(capture_wire(runtime::Wire(sample->as_wire()),
                runtime.config_.spatial.has_value(),"rows")));
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
        try {
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
                engine::get_state(&model, &data, runtime.next_integration_state_.data(), static_cast<int>(mjSTATE_INTEGRATION));
            });
            runtime.physical_->capture_presentation(runtime.next_presentation_);
            auto boundary = runtime.capture_boundary();
            runtime.commit_boundary(std::move(boundary));
        } catch (...) {
            runtime.boundary_recovery_required_ = true;
            throw;
        }
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

void testing::validate_spatial_numeric_evidence(const runtime::Wire& wire) {
    require_finite_spatial_wire(wire,"evidence");
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
    std::vector<runtime::RideControl> controls;controls.reserve(golden.controls.size());
    for(const auto& control:golden.controls)controls.push_back(decode_control(control));
    const bool spatial=env.snapshot().spatial.has_value();
    if(spatial) {
        const auto& manifest=io::as_object(golden.manifest,"golden manifest");
        const auto* source=io::find_member(manifest,"source_identity");
        const auto* execution=io::find_member(manifest,"execution_identity");
        if(!source||!execution||io::as_string(*source,"source_identity")!=runtime::runtime_identity().native_source_sha256 ||
            io::as_string(*execution,"execution_identity")!=platform::execution_identity())
            throw std::invalid_argument("incompatibility: spatial golden execution/source identity differs");
    }
    const auto candidate=RuntimeAccess::run(env,controls,&golden);
    auto differences = compare_rows(golden.rows,candidate.rows,rtol,atol);
    if(spatial) {
        for(auto key:{"spatial_checkpoint_initial","spatial_checkpoint_final","calibration_sha256","terrain_sha256"}) {
            const auto* expected=io::find_member(io::as_object(golden.manifest,"manifest"),key);
            const auto* actual=io::find_member(io::as_object(candidate.manifest,"manifest"),key);
            if(std::string_view(key)=="spatial_checkpoint_initial")actual=&initial_comparison_checkpoint(candidate);
            if(!expected||!actual||io::canonical_json(*expected)!=io::canonical_json(*actual))differences.push_back(std::string("regression: ")+key);
        }
    }
    Value failure;
    if (const auto current = env.snapshot().first_failure) failure = capture_wire(current->as_wire(),spatial,"first_failure");
    if (!same_failure(golden.first_failure, failure, rtol, atol))
        differences.emplace_back("first_failure differs between golden and candidate");
    return differences;
}

namespace {
const Value& required(const Value& value,std::string_view key) {
    const auto* member=io::find_member(io::as_object(value,"spatial golden"),key);
    if(!member)throw std::invalid_argument("incompatibility: spatial golden missing "+std::string(key));
    return *member;
}
std::string string(const Value& value,std::string_view key) {return io::as_string(required(value,key),key);}
struct Tolerance {double atol=0.,rtol=0.;bool used=false;};
class SpatialComparison {
public:
    SpatialComparison(const Value& entry,bool cross_platform):cross_(cross_platform) {
        if(cross_)for(const auto& [path,value]:io::as_object(required(required(entry,"comparison"),"cross_platform_tolerances"),"tolerances"))
            rules_.emplace(path,Tolerance{.atol=io::as_double(required(value,"atol"),"atol"),.rtol=io::as_double(required(value,"rtol"),"rtol")});
    }
    void number(double expected,double actual,const std::string& path,const std::string& label) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the reference API
        if(!difference_.empty())return;
        if(!std::isfinite(expected)||!std::isfinite(actual)) {
            difference_="physical_invalid: "+label+" contains nonfinite numeric evidence";return;
        }
        const auto rule=rules_.find(path);
        if(rule!=rules_.end()) {
            rule->second.used=true;
            if(std::isfinite(expected)&&std::isfinite(actual)&&
               std::abs(actual-expected)<=rule->second.atol+rule->second.rtol*std::abs(expected))return;
        } else if(std::bit_cast<std::uint64_t>(expected)==std::bit_cast<std::uint64_t>(actual))return;
        difference_="regression: "+label+" numeric evidence differs";
    }
    // NOLINTNEXTLINE(misc-no-recursion) bounded artifact value-tree comparison
    void value(const Value& expected,const Value& actual,const std::string& path,const std::string& label) {
        if(!difference_.empty())return;
        const auto nonfinite=[](const Value& value) {
            const auto* tag=std::get_if<io::Tagged>(&value.data);
            return tag && tag->kind=="float64";
        };
        // Recording float64 tags have an explicit numeric type and denote
        // only NaN/+Inf/-Inf. Ordinary string channels remain exact strings.
        if(nonfinite(expected)||nonfinite(actual)) {
            difference_="physical_invalid: "+label+" contains nonfinite numeric evidence";return;
        }
        if(cross_&&(path=="checkpoints.initial.runtime.continuation.execution"||path=="checkpoints.final.runtime.continuation.execution"))return;
        if(expected.data.index()!=actual.data.index()){difference_="regression: "+label+" value kind differs";return;}
        if(const auto* object=std::get_if<Value::Object>(&expected.data)) {
            const auto& other=std::get<Value::Object>(actual.data);
            if(object->size()!=other.size()){difference_="regression: "+label+" object keys differ";return;}
            for(const auto& [key,child]:*object) {
                const auto* found=io::find_member(other,key);
                if(!found){difference_="regression: ";difference_.append(label).append(" missing ").append(key);return;}
                value(child,*found,std::string(path).append(".").append(key),std::string(label).append(".").append(key));
            }
        } else if(const auto* array=std::get_if<Value::Array>(&expected.data)) {
            const auto& other=std::get<Value::Array>(actual.data);
            if(array->size()!=other.size()){difference_="regression: "+label+" array shape differs";return;}
            for(std::size_t i=0;i<array->size();++i)value((*array)[i],other[i],path+".*",label+"["+std::to_string(i)+"]");
        } else if(const auto* numeric=std::get_if<io::Number>(&expected.data)) {
            const auto& other=std::get<io::Number>(actual.data);
            if(io::is_integer_lexeme(numeric->lexeme)||io::is_integer_lexeme(other.lexeme)) {
                if(numeric->lexeme!=other.lexeme)difference_="regression: "+label+" exact integer differs";
            } else number(io::number_double(*numeric,label),io::number_double(other,label),path,label);
        } else if(io::canonical_json(expected)!=io::canonical_json(actual))difference_="regression: "+label+" exact evidence differs";
    }
    void array(const io::NpyArray& expected,const io::NpyArray& actual,const std::string& path) {
        if(!difference_.empty())return;
        if(expected.dtype!=actual.dtype||expected.shape!=actual.shape||expected.fortran_order!=actual.fortran_order){difference_="regression: "+path+" dtype/shape differs";return;}
        // Equal NaN/Inf payload bits can be useful diagnostic artifacts, but
        // cannot qualify a spatial physical run. Check both sides before the
        // same-platform byte fast path or a cross-platform tolerance rule.
        const auto x=expected.doubles(true),y=actual.doubles(true);
        const auto finite=[](double value){return std::isfinite(value);};
        if(!std::ranges::all_of(x,finite)||!std::ranges::all_of(y,finite)) {
            difference_="physical_invalid: "+path+" contains nonfinite numeric evidence";return;
        }
        if(!cross_){if(expected.bytes!=actual.bytes)difference_="regression: "+path+" bits differ";return;}
        for(std::size_t i=0;i<x.size();++i)number(x[i],y[i],path+".*",path+"["+std::to_string(i)+"]");
    }
    std::vector<std::string> finish() const {
        if(!difference_.empty())return {difference_};
        for(const auto& [path,rule]:rules_)if(!rule.used)throw std::invalid_argument("incompatibility: unused cross-platform numeric field tolerance "+path);
        return {};
    }
private:
    bool cross_;
    std::map<std::string,Tolerance> rules_;
    std::string difference_;
};
}

std::vector<std::string> compare_spatial_artifacts(const EpisodeArtifact& expected,
    const EpisodeArtifact& actual,const Value& entry,std::string_view current_platform) {
    const auto reference_platform=string(expected.manifest,"spatial_platform");
    if(string(actual.manifest,"spatial_platform")!=current_platform)throw std::invalid_argument("incompatibility: candidate platform differs");
    const auto& platforms=required(entry,"platforms");
    const auto& reference=required(platforms,reference_platform);const auto& current=required(platforms,current_platform);
    for(auto key:{"source_identity","execution_identity","provider"}) {
        if(string(expected.manifest,key)!=string(reference,key)||string(actual.manifest,key)!=string(current,key))
            throw std::invalid_argument("incompatibility: "+std::string(key)+" differs from reviewed platform");
    }
    if(string(expected.manifest,"source_identity")!=string(actual.manifest,"source_identity"))
        throw std::invalid_argument("incompatibility: stale source identity");
    for(auto key:{"config_sha256","terrain_sha256","calibration_sha256"})
        if(string(expected.manifest,key)!=string(entry,key)||string(actual.manifest,key)!=string(entry,key))
            throw std::invalid_argument("incompatibility: "+std::string(key)+" differs");
    if(!io::is_null(expected.first_failure)||!io::is_null(actual.first_failure))return {"physical_invalid: first_failure is present"};
    if(expected.channel_names!=actual.channel_names||expected.force_names!=actual.force_names||expected.tire_state_names!=actual.tire_state_names)
        return {"regression: channel/force/tyre names differ"};
    for(const auto& field:io::as_array(required(entry,"expected_channels"),"expected_channels"))
        if(std::ranges::find(expected.channel_names,io::as_string(field,"channel"))==expected.channel_names.end())
            throw std::invalid_argument("incompatibility: missing expected channel "+io::as_string(field,"channel"));
    const bool cross=reference_platform!=current_platform;
    SpatialComparison compare(entry,cross);
    compare.value(Value{expected.stored_rows},Value{actual.stored_rows},"rows","rows");
    compare.value(Value{expected.controls},Value{actual.controls},"controls","controls");
    compare.value(expected.first_failure,actual.first_failure,"first_failure","first_failure");
    for(auto key:{"initial","final"})compare.value(runtime::spatial::bootstrap_comparison_value(required(expected.manifest,std::string("spatial_checkpoint_")+key)),
        runtime::spatial::bootstrap_comparison_value(std::string_view(key)=="initial"?initial_comparison_checkpoint(actual):
            required(actual.manifest,"spatial_checkpoint_final")),std::string("checkpoints.")+key,std::string("checkpoints.")+key);
    if(expected.initial.size()!=actual.initial.size())return {"regression: initial engine state fields differ"};
    for(const auto& [name,value]:expected.initial) {
        // A named copy: capturing a structured binding in a lambda is only valid in
        // C++20 compilers with P1091/P1381, and this runs on both qualified hosts.
        const std::string& wanted=name;
        const auto found=std::ranges::find_if(actual.initial,[&wanted](const auto& p){return p.first==wanted;});
        if(found==actual.initial.end())return {"regression: missing initial engine field "+name};
        compare.array(value,found->second,"initial."+name);
    }
    for(const auto& [name,member]:std::array{
        std::pair{"state_qpos",&EpisodeArtifact::state_qpos},std::pair{"state_qvel",&EpisodeArtifact::state_qvel},
        std::pair{"state_act",&EpisodeArtifact::state_act},std::pair{"state_warmstart",&EpisodeArtifact::state_warmstart},
        std::pair{"state_time",&EpisodeArtifact::state_time},std::pair{"ctrl_written",&EpisodeArtifact::ctrl_written},
        std::pair{"qfrc_applied",&EpisodeArtifact::qfrc_applied},std::pair{"forces",&EpisodeArtifact::forces},std::pair{"tire_state",&EpisodeArtifact::tire_state}})
        compare.array(expected.*member,actual.*member,std::string("arrays.")+name);
    return compare.finish();
}
std::vector<std::string> compare_spatial_episode(const EpisodeArtifact& expected,
    runtime::NativeResearchRuntime& env,const Value& entry,std::string_view current_platform) {
    const auto& current=required(required(entry,"platforms"),current_platform);
    if(string(current,"source_identity")!=runtime::runtime_identity().native_source_sha256 ||
       string(current,"execution_identity")!=platform::execution_identity() || string(current,"provider")!=engine::identity().numeric_provider)
        throw std::invalid_argument("incompatibility: current execution/source/provider identity differs from reviewed platform");
    const auto reference_platform=string(expected.manifest,"spatial_platform");
    const auto& reference=required(required(entry,"platforms"),reference_platform);
    for(auto key:{"source_identity","execution_identity","provider"})
        if(string(expected.manifest,key)!=string(reference,key))
            throw std::invalid_argument("incompatibility: reference "+std::string(key)+" differs from reviewed platform");
    if(string(expected.manifest,"source_identity")!=string(current,"source_identity"))
        throw std::invalid_argument("incompatibility: stale source identity");
    for(auto key:{"config_sha256","terrain_sha256","calibration_sha256"})
        if(string(expected.manifest,key)!=string(entry,key))
            throw std::invalid_argument("incompatibility: reference "+std::string(key)+" differs");
    const auto state=env.snapshot();
    if(!state.spatial || state.spatial->terrain_sha256!=string(entry,"terrain_sha256") ||
       state.spatial->calibration_sha256!=string(entry,"calibration_sha256"))
        throw std::invalid_argument("incompatibility: candidate spatial terrain/calibration differs");
    std::vector<runtime::RideControl> controls;controls.reserve(expected.controls.size());
    for(const auto& control:expected.controls)controls.push_back(decode_control(control));
    const bool same_platform=reference_platform==current_platform;
    return compare_spatial_artifacts(expected,RuntimeAccess::run(env,controls,&expected,nullptr,nullptr,same_platform),entry,current_platform);
}
} // namespace bike::golden
