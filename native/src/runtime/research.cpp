#include "research.hpp"
#include "research_io.hpp"
#include "spatial_checkpoint_internal.hpp"
#include "setup_values.hpp"
#include "spatial_truth.hpp"
#include "value_readers.hpp"
#include "../io/json.hpp"
#include "../io/hash.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
#include "../stepper.hpp"
#include "../model_access.hpp"

namespace runtime {
namespace {
using namespace sample_wire;
class ResearchGuard {
public:
    explicit ResearchGuard(std::atomic<bool> &busy) : busy_(busy) {
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true, std::memory_order_acquire))
            throw std::runtime_error("native research operation already in progress");
    }
    ~ResearchGuard() { busy_.store(false, std::memory_order_release); }
    ResearchGuard(const ResearchGuard &) = delete;
    ResearchGuard &operator=(const ResearchGuard &) = delete;
    ResearchGuard(ResearchGuard &&) = delete;
    ResearchGuard &operator=(ResearchGuard &&) = delete;
private:
    std::atomic<bool> &busy_;
};
bool keeps_history(ResearchCaptureMode mode) noexcept {
    return mode == ResearchCaptureMode::full || mode == ResearchCaptureMode::legacy;
}
template<class T> void reserve_next(std::vector<T> &values) {
    if (values.size() == values.max_size()) throw std::length_error("research history row limit reached");
    const auto required = values.size() + 1;
    if (values.capacity() >= required) return;
    const auto grown = values.capacity() > values.max_size() / 2 ? required : values.capacity() * 2;
    values.reserve(std::max(required, grown));
}
Wire opt(std::optional<double> v) { return v ? Wire(*v) : Wire(nullptr); }
Wire opt(const std::optional<std::string> &v) { return v ? Wire(*v) : Wire(nullptr); }
const WireObject &section(const WireObject &v, std::string_view key) { return object(required(v, key)); }
double field(const WireObject &v, std::string_view key) { return number(required(v, key)); }
void validate_brake(double value) {
    if (!std::isfinite(value) || value < 0. || value > 1.)
        throw std::invalid_argument("brake demand must lie in [0, 1]");
}
// Steps must stay below 2^53 so every committed index is exactly real-representable.
constexpr std::int64_t kMaxExactStep = static_cast<std::int64_t>(std::uint64_t{1} << 52U);
std::size_t tape_rows(const ResearchNoiseTape& tape) { return std::visit([](const auto& value){return value.noise.size();},tape); }
std::string_view spatial_contact(const WireObject& truth) {
    const bool front=boolean(required(truth,"front_contact_valid")),rear=boolean(required(truth,"rear_contact_valid"));
    return front?(rear?"two_wheels":"rear_lift"):(rear?"front_lift":"flight");
}
bike::io::Value checkpoint_value(const Wire& wire) {
    return std::visit([](const auto& value)->bike::io::Value{
        using T=std::decay_t<decltype(value)>;
        if constexpr(std::is_same_v<T,std::monostate>)return {};
        else if constexpr(std::is_same_v<T,double>)return setupvalue::real(value);
        else if constexpr(std::is_same_v<T,std::int64_t>)return setupvalue::integer(value);
        else if constexpr(std::is_same_v<T,WireArray>){bike::io::Value::Array out;out.reserve(value.size());for(const auto& child:value)out.push_back(checkpoint_value(child));return bike::io::Value{std::move(out)};}
        else if constexpr(std::is_same_v<T,WireObject>){bike::io::Value::Object out;out.reserve(value.size());for(const auto& [key,child]:value)out.emplace_back(key,checkpoint_value(child));return bike::io::Value{std::move(out)};}
        else return bike::io::Value{value};
    },wire.value);
}
WireObject checked_spatial_truth(const WireObject& wire) {
    constexpr std::array<std::string_view,21> keys{
        "root_position_world_m","root_quat_world_from_body_wxyz","root_linear_velocity_world_mps","root_angular_velocity_world_rad_s",
        "attitude_valid","roll_rad","pitch_rad","heading_rad","steer_angle_rad","steer_rate_rad_s","steer_torque_nm","steer_work_j",
        "lateral_displacement_m","front_contact_valid","front_lateral_slip_mps","front_normal_load_n","front_support_normal_world",
        "rear_contact_valid","rear_lateral_slip_mps","rear_normal_load_n","rear_support_normal_world"};
    exact(wire,keys,"checkpoint.research.spatial_truth");
    const auto vector=[&](std::string_view key,std::size_t width,bool unit) {
        const auto& values=array(required(wire,key));
        if(values.size()!=width)throw std::invalid_argument("checkpoint.research.spatial_truth: vector width differs");
        double norm2=0.;for(const auto& value:values){const double x=number(value);norm2+=x*x;}
        if(unit&&std::abs(norm2-1.)>1e-9)throw std::invalid_argument("checkpoint.research.spatial_truth: nonunit orientation/normal");
    };
    vector("root_position_world_m",3,false);vector("root_quat_world_from_body_wxyz",4,true);
    vector("root_linear_velocity_world_mps",3,false);vector("root_angular_velocity_world_rad_s",3,false);
    const bool attitude=boolean(required(wire,"attitude_valid"));
    for(const auto name:{"roll_rad","pitch_rad","heading_rad"}) {
        const auto& angle=required(wire,name);
        if(attitude)(void)number(angle);
        else if(!is_none(angle))throw std::invalid_argument("checkpoint.research.spatial_truth: invalid attitude requires null angles");
    }
    for(const auto name:{"steer_angle_rad","steer_rate_rad_s","steer_torque_nm","steer_work_j","lateral_displacement_m"})(void)field(wire,name);
    for(const std::string_view side:{"front_","rear_"}) {
        const auto prefix=std::string(side);
        const bool contact=boolean(required(wire,prefix+"contact_valid"));
        if(contact) {
            (void)field(wire,prefix+"lateral_slip_mps");
            if(field(wire,prefix+"normal_load_n")<0.)throw std::invalid_argument("checkpoint.research.spatial_truth: negative normal load");
            vector(prefix+"support_normal_world",3,true);
        } else for(const auto suffix:{"lateral_slip_mps","normal_load_n","support_normal_world"})
            if(!is_none(required(wire,prefix+suffix)))throw std::invalid_argument("checkpoint.research.spatial_truth: invalid contact requires null evidence");
    }
    return wire;
}
std::string research_configuration_digest(const ResearchConfig& config) {
    if(!config.sensors_v2)throw std::invalid_argument("research configuration digest requires sensor schema2");
    WireArray rider,demand;
    for(const auto& frame:config.programs.rider_frames) {
        RideControl control;control.posture=frame.posture;control.human_torque_nm=frame.human_torque_nm;
        rider.emplace_back(WireObject{{"time_s",frame.time_s},{"control",research_control_wire(control)}});
    }
    for(const auto& frame:config.programs.demand_frames)demand.emplace_back(wire_array(frame));
    const WireObject value{{"timestep_s",config.timestep_s},{"track_length_m",config.track_length_m},{"start_position_m",config.start_position_m},
        {"maximum_energy_residual_ratio",config.maximum_energy_residual_ratio},{"wheelie_persistence_s",config.wheelie_persistence_s},
        {"control_steps",config.control_steps},{"delay_steps",config.delay_steps},{"max_steps",config.max_steps},{"sensor_steps",config.sensor_steps},
        {"record_decimation",config.record_decimation},{"capture_mode",static_cast<int>(config.capture_mode)},
        {"stop_on_model_violation",config.stop_on_model_violation},
        {"sensors_v2",sensor_config_v2_to_wire(*config.sensors_v2)},{"startup_control",research_control_wire(config.startup_control)},
        {"seed_decimal",config.seed_decimal},
        {"reaction_delay_s",config.programs.reaction_delay_s},{"rider_frames",std::move(rider)},{"demand_frames",std::move(demand)}};
    const auto json=bike::io::canonical_json(checkpoint_value(value));
    return bike::io::sha256(std::as_bytes(std::span{json}));
}
std::string research_tape_digest(const ResearchNoiseTape& tape) {
    const auto& value=std::get<spatial::NoiseTapeV2>(tape);
    // Each channel is a validated binary64 input on the qualified execution
    // lane; pin both arrays separately so their boundary cannot be ambiguous.
    const auto noise=bike::io::sha256(std::as_bytes(std::span{value.noise}));
    const auto dropout=bike::io::sha256(std::as_bytes(std::span{value.dropout_uniform}));
    const auto combined=noise+dropout;
    return bike::io::sha256(std::as_bytes(std::span{combined}));
}
ResearchTransition continuation_transition(const WireObject& wire) {
    constexpr std::array<std::string_view,10> keys{"observation","truth","contact_state","terminated","truncated","reason","physics_steps","numerically_valid","demand_nm","model_valid"};
    // Keep the exact historical transition spelling; schema is on observation.
    exact(wire,keys,"checkpoint.research.transition");
    ResearchTransition t;t.observation=research_observation_from_wire(object(required(wire,"observation")));
    if(!std::holds_alternative<spatial::SensorObservationV2>(t.observation))throw std::invalid_argument("checkpoint: spatial transition requires observation2");
    if(!is_none(required(wire,"truth")))t.spatial_truth=checked_spatial_truth(object(required(wire,"truth")));
    t.contact_state=string(required(wire,"contact_state"));t.terminated=boolean(required(wire,"terminated"));t.truncated=boolean(required(wire,"truncated"));
    t.numerically_valid=boolean(required(wire,"numerically_valid"));t.model_valid=boolean(required(wire,"model_valid"));
    t.physics_steps=integer(required(wire,"physics_steps"));if(t.physics_steps<0)throw std::invalid_argument("checkpoint.transition.physics_steps: negative");
    if(!is_none(required(wire,"reason")))t.reason=string(required(wire,"reason"));
    if(!is_none(required(wire,"demand_nm")))t.demand_nm=number(required(wire,"demand_nm"));
    if(t.spatial_truth&&t.contact_state!=spatial_contact(*t.spatial_truth))
        throw std::invalid_argument("checkpoint.transition.contact_state: differs from spatial contacts");
    if(t.terminated&&t.truncated)throw std::invalid_argument("checkpoint.transition: mutually exclusive outcome flags");
    if(t.demand_nm&&*t.demand_nm<0.)throw std::invalid_argument("checkpoint.transition.demand_nm: negative");
    return t;
}
ResearchCommand continuation_command(const WireObject& wire,bool requested) {
    const std::array<std::string_view,5> keys{"time_s","step","control","front_brake_demand","rear_brake_demand"};
    exact(wire,requested?std::span{keys}:std::span{keys}.first(3),"checkpoint.research.command");
    ResearchCommand command;command.time_s=number(required(wire,"time_s"));command.step=integer(required(wire,"step"));
    command.control=research_control_from_wire(object(required(wire,"control")));
    if(command.time_s<0.||command.step<0)throw std::invalid_argument("checkpoint.research.command: invalid clock");
    if(requested){command.front=number(required(wire,"front_brake_demand"));command.rear=number(required(wire,"rear_brake_demand"));validate_brake(command.front);validate_brake(command.rear);}
    return command;
}
}
WireObject research_observation_wire(const ResearchObservation& value) {
    return std::visit([](const auto& sample){return sample.as_wire();},value);
}
ResearchObservation research_observation_from_wire(const WireObject& value) {
    if(const auto* schema=sample_wire::find(value,"sensor_schema_version")) {
        if(sample_wire::integer(*schema)!=2)throw std::invalid_argument("research observation: unsupported sensor schema");
        return spatial::SensorObservationV2::from_wire(value);
    }
    return SensorObservation::from_wire(value);
}
spatial::SensorObservationV2 raw_sensor_observation_v2(const PhysicalSampleData& sample) {
    using namespace sample_wire;
    const auto& sensors=object(required(sample.channels,"sensors"));
    const auto& force=array(required(sensors,"specific_force_sensor_mps2"));
    const auto& gyro=array(required(sensors,"angular_rate_sensor_rad_s"));
    const auto& enc=object(required(sensors,"encoders_rad_s"));
    if(force.size()!=3||gyro.size()!=3)throw std::invalid_argument("sensors_v2: malformed captured IMU width");
    spatial::SensorObservationV2 out;out.time_s=out.source_time_s=sample.time_s;
    for(std::size_t i=0;i<3;++i){out.values[i]=number(force[i]);out.values[i+3]=number(gyro[i]);}
    out.values[6]=number(required(enc,"front_wheel"));out.values[7]=number(required(enc,"rear_wheel"));
    out.values[8]=number(required(enc,"crank"));out.values[9]=number(required(sensors,"motor_torque_nm"));
    out.values[10]=number(required(sensors,"human_torque_nm"));out.validate();return out;
}
WireObject ResearchCommand::as_wire(bool requested) const {
    WireObject result{{"time_s", time_s}, {"step", step}, {"control", research_control_wire(control)}};
    if (requested) {
        result.emplace_back("front_brake_demand", front);
        result.emplace_back("rear_brake_demand", rear);
    }
    return result;
}
WireObject ResearchTransition::as_wire() const {
    return {{"observation", research_observation_wire(observation)}, {"truth", spatial_truth?Wire(*spatial_truth):truth ? Wire(truth->as_wire()) : Wire(nullptr)},
        {"contact_state", contact_state}, {"terminated", terminated}, {"truncated", truncated},
        {"reason", opt(reason)}, {"physics_steps", physics_steps}, {"numerically_valid", numerically_valid},
        {"demand_nm", opt(demand_nm)}, {"model_valid", model_valid}};
}
ResearchEpisode::ResearchEpisode(const ResearchConfig &c, ResearchNoiseTape tape, ResearchSensorState startup)
    : sensors(c.sensors, std::holds_alternative<NoiseTape>(tape)?std::move(std::get<NoiseTape>(tape)):NoiseTape{}), tracker(c.wheelie_persistence_s),
      recorder(c.record_decimation, ColumnLayout{.root_x_qpos = c.geometry.root_x_qpos,
          .root_x_dof = c.geometry.root_x_dof, .root_pitch_qpos = c.geometry.root_pitch_qpos, .spatial=c.sensors_v2.has_value()}),
      next_sensor_step(c.sensor_steps), applied(c.startup_control), motor_applied(c.startup_control) {
    // Setup already pushed and delivered row zero. Never noise it twice.
    const bool valid_start=std::visit([](const auto& state){return state.cursor==1&&state.samples_attempted==1&&state.last_time&&*state.last_time==0.&&
        state.delivery_time&&*state.delivery_time==0.&&state.startup&&state.startup->time_s==0.;},startup);
    if (!valid_start)
        throw std::invalid_argument("research bootstrap requires the delivered startup row at t=0");
    if(c.sensors_v2) {
        if(!std::holds_alternative<spatial::NoiseTapeV2>(tape)||!std::holds_alternative<spatial::SensorStateV2>(startup))
            throw std::invalid_argument("research bootstrap: sensor schema2 requires tape11 and state2");
        sensors_v2.emplace(*c.sensors_v2,std::move(std::get<spatial::NoiseTapeV2>(tape)));
        sensors_v2->import_state(std::move(std::get<spatial::SensorStateV2>(startup)));
        observation=sensors_v2->read(0.);
    } else {
        if(!std::holds_alternative<SensorState>(startup))throw std::invalid_argument("research bootstrap: planar sensor schema mismatch");
        sensors.import_state(std::move(std::get<SensorState>(startup)));observation=sensors.read(0.);
    }
    if (keeps_history(c.capture_mode)) observations.push_back(observation);
    if (c.capture_mode != ResearchCaptureMode::off)
        applied_records.push_back(ResearchCommand{.time_s = 0., .step = 0, .control = applied,
            .front = 0., .rear = 0.});
    demand_nm = c.programs.demand_at(0.);
    if (demand_nm) demand_integral_nms = 0.;
}
NativeResearchRuntime::NativeResearchRuntime(std::unique_ptr<NativeRideRuntime> runtime,
    ResearchConfig config, ResearchNoiseTape tape, ResearchSensorState startup)
    : runtime_(std::move(runtime)), config_(std::move(config)), reset_tape_(tape), reset_startup_(startup) {
    if (!runtime_ || runtime_->closed() || runtime_->committed_step_ != 0 || runtime_->committed_time_s_ != 0.)
        throw std::invalid_argument("native research requires a fresh owned physical runtime");
    const auto &c = config_;
    if (!std::isfinite(c.timestep_s) || c.timestep_s <= 0. || c.timestep_s != runtime_->config_.timestep_s ||
        c.control_steps < 1 || c.sensor_steps < 1 || c.max_steps < 1 || c.delay_steps < 0 ||
        c.max_steps > kMaxExactStep || c.sensor_steps > kMaxExactStep ||
        c.delay_steps > std::numeric_limits<std::int64_t>::max() - c.max_steps)
        throw std::invalid_argument("invalid integer research deadlines or physics timestep");
    if (!std::isfinite(c.track_length_m) || c.track_length_m <= 0. || !std::isfinite(c.start_position_m) ||
        !std::isfinite(c.maximum_energy_residual_ratio) || c.maximum_energy_residual_ratio <= 0.)
        throw std::invalid_argument("invalid research geometry/energy limits");
    if (c.record_decimation != runtime_->config_.measurement_decimation ||
        c.stop_on_model_violation != runtime_->config_.strict)
        throw std::invalid_argument("research recording/strictness must match its physical runtime");
    const bool spatial_mode=runtime_->config_.spatial.has_value();
    if(spatial_mode)runtime_->completion_step_=c.max_steps;
    if(spatial_mode!=c.sensors_v2.has_value() || spatial_mode!=std::holds_alternative<spatial::NoiseTapeV2>(tape) ||
       spatial_mode!=std::holds_alternative<spatial::SensorStateV2>(startup))
        throw std::invalid_argument("research bootstrap: runtime/sensor/tape/state schema mismatch");
    if(spatial_mode) {
        c.sensors_v2->validate();
        if(std::abs(c.sensors_v2->sample_period_s/c.timestep_s-static_cast<double>(c.sensor_steps))>1e-8||
            static_cast<double>(c.control_steps)*c.timestep_s>c.sensors_v2->maximum_read_period_s)
            throw std::invalid_argument("research.sensors_v2: periods differ from integer acquisition/control clocks");
    }
    if(!spatial_mode) {
    c.geometry.validate(static_cast<std::size_t>(runtime_->stepper_->model()->nq),
                        static_cast<std::size_t>(runtime_->stepper_->model()->nv));
    if (c.geometry.root_x_qpos != runtime_->accounting_config_.columns.root_x_qpos ||
        c.geometry.root_x_dof != runtime_->accounting_config_.columns.root_x_dof ||
        c.geometry.root_pitch_qpos != runtime_->accounting_config_.columns.root_pitch_qpos ||
        c.start_position_m != runtime_->committed_position_m_)
        throw std::invalid_argument("research truth projection disagrees with physical bootstrap");
    const auto vertices = runtime_->physical_->terrain_vertices();
    if (vertices.size() != c.geometry.x.size())
        throw std::invalid_argument("research terrain width differs from physical bootstrap");
    for (std::size_t i = 0; i < vertices.size(); ++i)
        if (std::get<0>(vertices[i]) != c.geometry.x[i] || std::get<1>(vertices[i]) != c.geometry.z[i])
            throw std::invalid_argument("research terrain differs from physical bootstrap");
    const auto *model = runtime_->stepper_->model();
    const int pitch_joint = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    const auto dofs = model_access::readonly_buffer(model->jnt_dofadr, model->njnt,
                                                    "research pitch joint dof addresses");
    // Backend-agnostic presence + radii check: the writer view forwards
    // radii() for whichever tyre backend the config selected.
    const bool tire_mismatch = !runtime_->stepper_->has_tire() ||
        runtime_->stepper_->tire_writer().radii()[0] != c.geometry.front_radius ||
        runtime_->stepper_->tire_writer().radii()[1] != c.geometry.rear_radius;
    if (tire_mismatch || pitch_joint < 0 ||
        std::cmp_not_equal(dofs[static_cast<std::size_t>(pitch_joint)], c.geometry.root_pitch_dof))
        throw std::invalid_argument("research radii/pitch address differ from physical bootstrap");
    }
    c.programs.validate();
    control_validate_for(c.startup_control, runtime_->physical_->rider_present(), runtime_->physical_->drive_mode());
    if (c.seed_decimal.empty() || !std::ranges::all_of(c.seed_decimal,
            [](char ch) { return ch >= '0' && ch <= '9'; }))
        throw std::invalid_argument("research seed must be a nonnegative decimal integer");
    for (const double scale : c.noise_std)
        if (!std::isfinite(scale) || scale < 0.)
            throw std::invalid_argument("sensor noise scales must be finite and nonnegative");
    if (c.raw_initial) {
        c.raw_initial->validate();
        if (c.raw_initial->time_s != 0. || c.raw_initial->source_time_s != 0.)
            throw std::invalid_argument("research raw startup observation must belong to time zero");
    }
    if(c.raw_initial_v2) {
        c.raw_initial_v2->validate();
        if(!spatial_mode||c.raw_initial_v2->time_s!=0.||c.raw_initial_v2->source_time_s!=0.)
            throw std::invalid_argument("research.raw_initial_v2: requires schema2 source at time zero");
    }
    const auto required_rows = static_cast<std::size_t>(1 + c.max_steps / c.sensor_steps);
    if (tape_rows(tape) != required_rows)
        throw std::invalid_argument("research tape must contain 1 + floor(max_steps / sensor_steps) rows");
    initial_integration_state_ = runtime_->snapshot().integration_state;
    episode_ = std::make_unique<ResearchEpisode>(c, std::move(tape), std::move(startup));
    if(spatial_mode)initial_checkpoint_=checkpoint_unlocked();
}
NativeResearchRuntime::~NativeResearchRuntime() = default;
void NativeResearchRuntime::require_open() const {
    if (closed_.load(std::memory_order_acquire)) throw std::runtime_error("NativeResearchRuntime is closed");
}
bool NativeResearchRuntime::model_valid() const {
    return boolean(required(runtime_->accounting_->state().model_status.as_wire(), "model_valid"));
}
void NativeResearchRuntime::begin_control(const RideControl &control, double front, double rear) {
    const ResearchGuard guard(busy_);
    require_open();
    auto &e = *episode_;
    if (e.window || e.delivery_pending) throw std::runtime_error("an external research window is already active");
    if (e.done()) throw std::runtime_error("episode has ended; reset before stepping again");
    validate_brake(front);
    validate_brake(rear);
    config_.programs.validate_control(control);
    control_validate_for(control, runtime_->physical_->rider_present(), runtime_->physical_->drive_mode());
    const auto start = runtime_->committed_step_;
    if (start >= config_.max_steps) throw std::runtime_error("research duration already reached");
    const auto count = std::min(config_.control_steps, config_.max_steps - start);
    const ResearchWindow window{.control = control, .front = front, .rear = rear,
        .start_step = start, .target_step = start + count, .seen_demand = e.demand_nm};
    const ResearchCommand request{.time_s = runtime_->committed_time_s_, .step = start,
        .control = control, .front = front, .rear = rear};
    if (keeps_history(config_.capture_mode)) reserve_next(e.requested);
    // RideControl contains value-only fields. Queue allocation precedes the
    // publication of the accepted request and the active window.
    e.commands.emplace_back(start + config_.delay_steps, control);
    if (keeps_history(config_.capture_mode)) e.requested.push_back(request);
    e.last_requested = request;
    ++e.requested_count;
    e.window = window;
}
void NativeResearchRuntime::consume_sample(const SamplePtr &sample, bool stop_at_outcome) {
    auto &e = *episode_;
    const auto &s = *sample;
    std::optional<ResearchQuality> spatial_quality;
    if(find(s.channels,"spatial")) {
        spatial_quality=research_quality(section(s.channels,"energy"),config_.maximum_energy_residual_ratio);
        if(!e.spatial_metrics)e.spatial_metrics=SpatialEpisodeMetrics{
            .requested_duration_s=static_cast<double>(config_.max_steps)*config_.timestep_s,
            .start_position_m=config_.start_position_m};
        // Reject this interval from the valid prefix before accumulating its
        // duration/distance, including the final requested-completion interval.
        if(!spatial_quality->acceptable)e.spatial_metrics->valid=false;
        const auto* recorded=find(s.channels,"spatial_outcome");
        if(!recorded)throw std::invalid_argument("research.spatial: sample lacks its accepted outcome record");
        const auto& record=object(*recorded);
        bike::spatial::Outcome outcome;
        outcome.terminal=boolean(required(record,"terminal"));outcome.valid=boolean(required(record,"valid"));
        outcome.tick=integer(required(record,"tick"));outcome.time_s=number(required(record,"time_s"));
        const auto label=string(required(record,"reason"));bool recognized=false;
        for(const auto reason:{bike::spatial::SpatialReason::none,bike::spatial::SpatialReason::capsize,
            bike::spatial::SpatialReason::off_corridor,bike::spatial::SpatialReason::balance_lost,
            bike::spatial::SpatialReason::terrain_invalid,bike::spatial::SpatialReason::terrain_outside,
            bike::spatial::SpatialReason::numerical_failure,bike::spatial::SpatialReason::requested_completion})
            if(label==bike::spatial::to_string(reason)){outcome.reason=reason;recognized=true;break;}
        if(!recognized)throw std::invalid_argument("research.spatial: unknown accepted outcome reason");
        update_spatial_metrics(*e.spatial_metrics,s.interval_id,s.end_time_s,s.dt_s(),
            number(required(record,"accepted_position_m")),outcome);
        if(outcome.terminal && outcome.time_s<=s.end_time_s) {
            e.numerically_valid=e.numerically_valid && outcome.valid;
            e.terminated=outcome.valid;e.truncated=!outcome.valid;
            e.reason=label;
            // Even cleanup consumption cannot replace a spatial latch with a
            // later planar monitor, model-quality label or finish poll.
            // Still consume the final accepted interval's measurements below.
        }
    }
    const double ratio = s.time_s / config_.timestep_s;
    if (!std::isfinite(ratio) || ratio < 0. || ratio > static_cast<double>(config_.max_steps))
        throw std::runtime_error("invalid sensor acquisition time");
    const auto source_step = static_cast<std::int64_t>(std::nearbyint(ratio));
    if (source_step >= e.next_sensor_step) {
        if (source_step != e.next_sensor_step)
            throw std::runtime_error("sensor acquisition skipped a physical sample");
        if(e.sensors_v2)e.sensors_v2->push(raw_sensor_observation_v2(s));
        else e.sensors.push(raw_sensor_observation(s));
        e.next_sensor_step += config_.sensor_steps;
    }
    if(e.sensors_v2) {
        e.spatial_truth=object(required(s.channels,"spatial"));
        auto& metrics=*e.spatial_metrics; // NOLINT(bugprone-unchecked-optional-access) every sensors_v2 sample carries the spatial channel that assigns spatial_metrics above
        for(std::size_t w=0;w<2;++w){const auto prefix=w==0?"front_":"rear_";
            if(boolean(required(*e.spatial_truth,std::string(prefix)+"contact_valid")))
                metrics.lateral_slip_path_m[w]=spatial::integrate_abs_slip(metrics.lateral_slip_path_m[w],
                    number(required(*e.spatial_truth,std::string(prefix)+"lateral_slip_mps")),s.dt_s());}
        const bool front=boolean(required(*e.spatial_truth,"front_contact_valid")),rear=boolean(required(*e.spatial_truth,"rear_contact_valid"));
        if(!front)metrics.front_lift_s+=s.dt_s();
        if(!front&&!rear)metrics.flight_s+=s.dt_s();
        const auto& mass=section(s.channels,"mass");
        const auto& com=array(required(mass,"com_m"));
        const auto& geometry=section(s.channels,"spatial_geometry");
        const auto& front_position=array(required(geometry,"front_wheel_position_world_m"));
        const auto& rear_position=array(required(geometry,"rear_wheel_position_world_m"));
        if(com.size()!=3||front_position.size()!=3||rear_position.size()!=3)
            throw std::invalid_argument("research.spatial: malformed accepted geometry width");
        const double dx=number(front_position[0])-number(rear_position[0]);
        const double dy=number(front_position[1])-number(rear_position[1]);
        const double span2=dx*dx+dy*dy;
        const double front_share=span2>1e-20?std::clamp(
            ((number(com[0])-number(rear_position[0]))*dx+
             (number(com[1])-number(rear_position[1]))*dy)/span2,0.,1.):.5;
        const auto& gravity=runtime_->stepper_->model()->opt.gravity;
        const double gravity_magnitude=std::sqrt(gravity[0]*gravity[0]+gravity[1]*gravity[1]+gravity[2]*gravity[2]);
        const double front_load=front?number(required(*e.spatial_truth,"front_normal_load_n")):0.;
        if(front_load<.05*number(required(mass,"mass_kg"))*gravity_magnitude*front_share)
            metrics.front_unloaded_s+=s.dt_s();
    } else e.truth = research_truth(s, config_.geometry);
    const auto &suspension = section(s.channels, "suspension");
    e.max_shock_stroke_m = std::max(e.max_shock_stroke_m, std::abs(field(suspension, "shock_stroke_m")));
    e.max_fork_travel_m = std::max(e.max_fork_travel_m, std::abs(field(suspension, "fork_travel_m")));
    const auto &drive = section(s.channels, "drive");
    const auto *motor = find(drive, "motor_torque_nm");
    const double delivered = motor ? number(*motor) : 0.;
    if (e.demand_integral_nms) {
        const auto demand = config_.programs.demand_at(s.time_s);
        // Integral tracking is created only when a demand program exists.
        if (!demand) throw std::logic_error("demand integration requires a demand program");
        *e.demand_integral_nms += *demand * s.dt_s();
    }
    const auto &requested = required(section(s.channels, "control"), "motor_torque_nm");
    const auto applied = is_none(requested) ? std::nullopt : std::optional(number(requested));
    if (applied) e.torque_requested_nms += *applied * s.dt_s();
    e.torque_delivered_nms += delivered * s.dt_s();
    if(e.truth)e.tracker.update(*e.truth, s.dt_s(), delivered, applied);
    if (config_.capture_mode == ResearchCaptureMode::legacy) e.recorder.append(sample);
    const auto quality = spatial_quality ? *spatial_quality :
        research_quality(section(s.channels, "energy"), config_.maximum_energy_residual_ratio);
    e.max_energy_residual_ratio = std::max(e.max_energy_residual_ratio, quality.residual_ratio);
    if(e.spatial_metrics&&e.spatial_metrics->terminal_reason!=bike::spatial::SpatialReason::none) {
        // Preserve the physical terminal reason, but never let its priority
        // hide failed energy quality on the same accepted interval.
        if(!quality.acceptable) {
            e.numerically_valid=false;
            e.terminated=false;e.truncated=true;
        }
        return;
    }
    if (!quality.acceptable) {
        e.numerically_valid = false;
        if(e.spatial_metrics)e.spatial_metrics->valid=false;
        e.truncated = true;
        e.reason = "numerical_quality";
        if (stop_at_outcome) return;
    }
    if (config_.stop_on_model_violation && !boolean(required(section(s.channels, "model_status"), "model_valid"))) {
        e.truncated = true;
        e.reason = "model_violation";
        if (stop_at_outcome) return;
    }
    if(e.sensors_v2)return; // Terminal ownership is the physical spatial latch.
    const auto &crash = runtime_->committed_crash_;
    if (crash && crash->time_s <= s.end_time_s) {
        e.terminated = true;
        e.reason = crash->cause == "pitch_over"
            ? (crash->pitch_rad < 0. ? "crash:loop_out" : "crash:endo") : "crash:" + crash->cause;
        if (stop_at_outcome) return;
    }
    if (e.truth->position_m >= config_.track_length_m) { // NOLINT(bugprone-unchecked-optional-access) the non-spatial branch assigns e.truth in this same consume_sample pass
        e.terminated = true;
        e.reason = "finish";
    }
}
void NativeResearchRuntime::consume_samples(bool stop_at_outcome) {
    auto &e = *episode_;
    try {
        for (const auto &sample : runtime_->accounting_->pending_samples()) {
            if (sample->interval_id <= e.sample_cursor) continue;
            // Claim before side effects, as the Python invalid-run contract does.
            e.sample_cursor = sample->interval_id;
            consume_sample(sample, stop_at_outcome);
            if (stop_at_outcome && e.done()) break;
        }
    } catch (...) {
        runtime_->accounting_->acknowledge_through(e.sample_cursor);
        throw;
    }
    runtime_->accounting_->acknowledge_through(e.sample_cursor);
}
void NativeResearchRuntime::finish_window() {
    auto &e = *episode_;
    if (!e.window) throw std::logic_error("no research window to finish");
    if (!e.done() && runtime_->committed_step_ >= config_.max_steps) {
        e.truncated = true;
        e.reason = "duration";
    }
    if (keeps_history(config_.capture_mode)) {
        reserve_next(e.observations);
        reserve_next(e.transitions);
    }
    const auto demand = config_.programs.demand_at(runtime_->committed_time_s_);
    const ResearchObservation observation=e.sensors_v2?ResearchObservation{e.sensors_v2->read(runtime_->committed_time_s_)}:
        ResearchObservation{e.sensors.read(runtime_->committed_time_s_)};
    ResearchTransition result{.observation = observation, .truth = e.truth, .spatial_truth=e.spatial_truth,
        .contact_state = e.spatial_truth?std::string(spatial_contact(*e.spatial_truth)):e.tracker.state(), .terminated = e.terminated, .truncated = e.truncated,
        .numerically_valid = e.numerically_valid, .model_valid = model_valid(), .reason = e.reason,
        .physics_steps = runtime_->committed_step_ - e.window->start_step,
        .demand_nm = e.window->seen_demand};
    // Stage the deliverable before publishing the completed control prefix.
    // Python boxing may fail and retry without advancing any of the clocks.
    const auto pending = result;
    if (keeps_history(config_.capture_mode)) {
        e.transitions.push_back(std::move(result));
        e.observations.push_back(observation);
    }
    e.pending_transition = pending;
    ++e.completed_count;
    e.observation = observation;
    e.demand_nm = demand;
    e.delivery_pending = true;
}
void NativeResearchRuntime::handle_failure(const std::exception &failure) {
    auto &e = *episode_;
    e.terminated = true;
    const bool reference = dynamic_cast<const InvalidReferenceRun *>(&failure) != nullptr;
    std::string reason = reference ? "invalid_controller" : "simulation_error";
    std::optional<bike::spatial::SpatialReason> spatial_failure;
    if(runtime_->config_.spatial) {
        e.checkpoint_unavailable_reason="failed spatial research boundary: "+std::string(failure.what());
        if(!e.spatial_metrics)e.spatial_metrics=SpatialEpisodeMetrics{
            .requested_duration_s=static_cast<double>(config_.max_steps)*config_.timestep_s,
            .start_position_m=config_.start_position_m};
        e.numerically_valid=false;e.terminated=false;e.truncated=true;
        const auto& state=runtime_->physical_->spatial_state();
        spatial_failure=state && state->outcome.latched.terminal?state->outcome.latched.reason:
            bike::spatial::SpatialReason::numerical_failure;
        // Keep the failure local until all already accepted pending samples
        // have extended the valid prefix. A premature terminal latch would
        // make their metrics updates observational no-ops.
    }
    e.reason = reason;
    e.error = std::string(reference ? "InvalidReferenceRun: " : "RuntimeError: ") + failure.what();
    // Cleanup may fail independently; retain the original exception and all
    // successfully published evidence rather than manufacturing a transition.
    try { if (runtime_->accounting_->has_raws()) runtime_->flush_accounting(); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing interval check: ") + tail.what()); }
    try { consume_samples(false); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing sample consumption: ") + tail.what()); }
    if(e.spatial_metrics) {
        if(e.spatial_metrics->terminal_reason==bike::spatial::SpatialReason::none && spatial_failure)
            e.spatial_metrics->terminal_reason=*spatial_failure;
        reason=std::string(bike::spatial::to_string(e.spatial_metrics->terminal_reason));
        e.spatial_metrics->valid=false;e.numerically_valid=false;e.terminated=false;e.truncated=true;
    }
    e.reason = reason;
    e.window.reset();
    e.delivery_pending = false;
}
std::optional<ResearchTransition> NativeResearchRuntime::advance_control(
    std::optional<double> budget, std::optional<std::int64_t> target_step) {
    const ResearchGuard guard(busy_);
    require_open();
    if (budget && (!std::isfinite(*budget) || *budget < 0.))
        throw std::invalid_argument("wall_budget_s must be finite and nonnegative");
    if (target_step && *target_step < runtime_->committed_step_)
        throw std::invalid_argument("target_step is behind the committed step");
    auto &e = *episode_;
    if (e.delivery_pending) return e.pending_transition;
    if (!e.window) throw std::runtime_error("begin_control must precede advance_control");
    const auto start = std::chrono::steady_clock::now();
    try {
        runtime_->stepper_->refresh_time_callback_policy();
        while (runtime_->committed_step_ < e.window->target_step && !e.done()) {
            if (target_step && runtime_->committed_step_ >= *target_step)
                return std::nullopt; // Pacing yield; the external command stays latched.
            if (budget && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= *budget)
                return std::nullopt; // No flush, delivery, request or transition on a yield.
            while (!e.commands.empty() && e.commands.front().first <= runtime_->committed_step_) {
                e.motor_applied = e.commands.front().second;
                e.commands.pop_front();
            }
            const auto rider = config_.programs.apply(e.window->control, runtime_->committed_time_s_);
            RideControl effective = e.motor_applied;
            effective.human_torque_nm = rider.human_torque_nm;
            effective.crank_target_rate_rad_s = rider.crank_target_rate_rad_s;
            effective.posture = rider.posture;
            effective.rider_enabled = rider.rider_enabled;
            if (!controls_equal(effective, e.applied)) {
                if (config_.capture_mode != ResearchCaptureMode::off)
                    e.applied_records.push_back(ResearchCommand{.time_s = runtime_->committed_time_s_,
                        .step = runtime_->committed_step_, .control = effective, .front = 0., .rear = 0.});
                e.applied = effective;
            }
            runtime_->advance_interval(e.applied, e.window->front, e.window->rear);
            consume_samples();
        }
        if (runtime_->accounting_->has_raws()) {
            runtime_->flush_accounting();
            consume_samples();
        }
        finish_window();
    } catch (const std::exception &failure) {
        // Best-effort cleanup must not mask even an allocation exception.
        try { handle_failure(failure); }
        catch (...) {
            e.terminated = true;
            e.window.reset();
            e.delivery_pending = false;
        }
        throw;
    }
    return e.pending_transition;
}
void NativeResearchRuntime::acknowledge_control() {
    const ResearchGuard guard(busy_);
    require_open();
    auto &e = *episode_;
    if (!e.delivery_pending) throw std::runtime_error("no completed research transition to acknowledge");
    e.pending_transition.reset();
    e.delivery_pending = false;
    e.window.reset();
    if (e.stop_requested && !e.done()) { e.truncated = true; e.reason = "operator_stop"; }
    e.stop_requested = false;
}
RuntimeSnapshot NativeResearchRuntime::snapshot() const {
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->snapshot();
}
RenderFrame NativeResearchRuntime::render_frame() const {
    const ResearchGuard guard(busy_);
    require_open();
    auto frame = runtime_->render_frame();
    if (episode_->reason) frame.state.outcome = episode_->reason;
    return frame;
}
ModelImage NativeResearchRuntime::model_image() const {
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->model_image();
}
WireObject NativeResearchRuntime::status() const {
    const ResearchGuard guard(busy_);
    require_open();
    return status_unlocked();
}
WireObject NativeResearchRuntime::status_unlocked() const {
    const auto &e = *episode_;
    const auto &balance = runtime_->committed_balance_;
    const auto &crash = runtime_->committed_crash_;
    // SensorPipelineV2::state() returns a copy of its queue; read it once per status.
    const std::array<std::size_t, 3> sensor_counters = e.sensors_v2
        ? [&] { const auto state = e.sensors_v2->state(); return std::array<std::size_t, 3>{state.samples_attempted, state.samples_dropped, state.cursor}; }()
        : [&] { const auto &state = e.sensors.state(); return std::array<std::size_t, 3>{state.samples_attempted, state.samples_dropped, state.cursor}; }();
    WireObject result{{"step", runtime_->committed_step_}, {"time_s", runtime_->committed_time_s_},
        {"position_m", runtime_->committed_position_m_}, {"battery_energy_j", runtime_->committed_battery_energy_j_},
        {"observation", research_observation_wire(e.observation)}, {"demand_nm", opt(e.demand_nm)},
        {"truth", e.spatial_truth?Wire(*e.spatial_truth):e.truth ? Wire(e.truth->as_wire()) : Wire(nullptr)},
        {"contact_state", e.spatial_truth?std::string(spatial_contact(*e.spatial_truth)):e.tracker.state()},
        {"terminated", e.terminated}, {"truncated", e.truncated}, {"reason", opt(e.reason)}, {"error", opt(e.error)},
        {"numerically_valid", e.numerically_valid}, {"model_valid", model_valid()},
        {"max_energy_residual_ratio", e.max_energy_residual_ratio}, {"torque_delivered_nms", e.torque_delivered_nms},
        {"torque_requested_nms", e.torque_requested_nms}, {"demand_integral_nms", opt(e.demand_integral_nms)},
        {"max_shock_stroke_m", e.max_shock_stroke_m}, {"max_fork_travel_m", e.max_fork_travel_m},
        {"metrics", e.tracker.metrics()}, {"accounting", runtime_->accounting_->state_wire()},
        {"active", e.window.has_value()}, {"stop_requested", e.stop_requested}, {"delivery_pending", e.delivery_pending},
        {"samples_attempted", static_cast<std::int64_t>(sensor_counters[0])},
        {"samples_dropped", static_cast<std::int64_t>(sensor_counters[1])},
        {"noise_cursor", static_cast<std::int64_t>(sensor_counters[2])},
        {"recorded_rows", static_cast<std::int64_t>(config_.capture_mode == ResearchCaptureMode::full
            ? runtime_->accounting_->recorded_samples().size() : e.recorder.rows())},
        {"requested_count", e.requested_count}, {"completed_count", e.completed_count},
        {"applied_base", e.applied_base},
        {"cleanup_notes", strings(e.cleanup_notes)}};
    result.emplace_back("balance_event", balance ? Wire(WireObject{{"time_s", balance->time_s},
        {"position_m", balance->position_m}, {"speed_mps", balance->speed_mps}}) : Wire(nullptr));
    result.emplace_back("crash", crash ? Wire(WireObject{{"time_s", crash->time_s}, {"position_m", crash->position_m},
        {"pitch_rad", crash->pitch_rad}, {"cause", crash->cause}}) : Wire(nullptr));
    if(e.spatial_metrics)result.emplace_back("spatial",Wire(e.spatial_metrics->as_wire()));
    return result;
}
ResearchConfig NativeResearchRuntime::configuration() const {
    const ResearchGuard guard(busy_); require_open();
    return config_;
}
WireObject NativeResearchRuntime::continuation_state_unlocked() const {
    if(!config_.sensors_v2||!episode_->sensors_v2)throw std::invalid_argument("research continuation requires sensor schema2");
    const auto& e=*episode_;WireArray pending,requested,applied,observations,transitions;
    for(const auto& [step,control]:e.commands)pending.emplace_back(WireObject{{"step",step},{"control",research_control_wire(control)}});
    for(const auto& c:e.requested)requested.emplace_back(c.as_wire(true));
    for(const auto& c:e.applied_records)applied.emplace_back(c.as_wire(false));
    for(const auto& o:e.observations)observations.emplace_back(research_observation_wire(o));
    for(const auto& t:e.transitions)transitions.emplace_back(t.as_wire());
    Wire window;
    if(e.window)window=WireObject{{"control",research_control_wire(e.window->control)},{"front",e.window->front},{"rear",e.window->rear},
        {"start_step",e.window->start_step},{"target_step",e.window->target_step},{"seen_demand",opt(e.window->seen_demand)}};
    Wire metrics;
    if(e.spatial_metrics){auto fields=e.spatial_metrics->as_wire();
        fields.emplace_back("start_position_m",e.spatial_metrics->start_position_m);
        fields.emplace_back("accepted_origin_time_s",e.spatial_metrics->accepted_origin_time_s);
        fields.emplace_back("last_end_time_s",e.spatial_metrics->last_end_time_s);
        fields.emplace_back("last_interval",e.spatial_metrics->last_interval);metrics=std::move(fields);}
    WireObject episode{{"terminated",e.terminated},{"truncated",e.truncated},{"numerically_valid",e.numerically_valid},
        {"reason",opt(e.reason)},{"error",opt(e.error)},{"cleanup_notes",strings(e.cleanup_notes)},
        {"max_energy_residual_ratio",e.max_energy_residual_ratio},{"torque_delivered_nms",e.torque_delivered_nms},
        {"torque_requested_nms",e.torque_requested_nms},{"demand_integral_nms",opt(e.demand_integral_nms)},{"demand_nm",opt(e.demand_nm)},
        {"max_shock_stroke_m",e.max_shock_stroke_m},{"max_fork_travel_m",e.max_fork_travel_m},
        {"checkpoint_unavailable_reason",opt(e.checkpoint_unavailable_reason)}};
    return {{"clock",WireObject{{"step",runtime_->committed_step_},{"time_s",runtime_->committed_time_s_},
        {"next_sensor_step",e.next_sensor_step},{"sample_cursor",e.sample_cursor},{"seed_decimal",config_.seed_decimal},
        {"configuration_sha256",research_configuration_digest(config_)},{"noise_tape_sha256",research_tape_digest(reset_tape_)},
        {"applied",research_control_wire(e.applied)},{"motor_applied",research_control_wire(e.motor_applied)},
        {"last_requested",e.last_requested?Wire(e.last_requested->as_wire(true)):Wire(nullptr)}}},
        {"pending_commands",std::move(pending)},{"actuator_delay",WireObject{{"steps",config_.delay_steps}}},{"window",std::move(window)},
        {"ack",WireObject{{"delivery_pending",e.delivery_pending},{"stop_requested",e.stop_requested},
            {"pending_transition",e.pending_transition?Wire(e.pending_transition->as_wire()):Wire(nullptr)}}},
        {"metrics_cursors",WireObject{{"requested_count",e.requested_count},{"completed_count",e.completed_count},{"applied_base",e.applied_base},
            {"observation",research_observation_wire(e.observation)},{"spatial_metrics",std::move(metrics)},
            {"spatial_truth",e.spatial_truth?Wire(*e.spatial_truth):Wire(nullptr)},{"episode",std::move(episode)},
            {"requested",std::move(requested)},{"applied",std::move(applied)},{"observations",std::move(observations)},{"transitions",std::move(transitions)},
            {"recorder",e.recorder.continuation_state()}}},
        {"sensor_v2",WireObject{{"schema",std::int64_t{2}},{"config",sensor_config_v2_to_wire(*config_.sensors_v2)},
            {"state",e.sensors_v2->state_wire()}}}};
}
WireObject NativeResearchRuntime::continuation_state() const {
    const ResearchGuard guard(busy_);require_open();return continuation_state_unlocked();
}
std::unique_ptr<ResearchEpisode> NativeResearchRuntime::prepare_continuation(const WireObject& wire,const NativeRideRuntime& plant) const {
    if(!config_.sensors_v2)throw std::invalid_argument("research continuation requires schema2");
    const auto expected_step=plant.committed_step_;
    const auto expected_time=plant.committed_time_s_;
    constexpr std::array<std::string_view,7> keys{"clock","pending_commands","actuator_delay","window","ack","metrics_cursors","sensor_v2"};
    exact(wire,keys,"research continuation");
    const auto& clock=section(wire,"clock");
    constexpr std::array<std::string_view,10> clock_keys{"step","time_s","next_sensor_step","sample_cursor","seed_decimal","configuration_sha256","noise_tape_sha256","applied","motor_applied","last_requested"};
    exact(clock,clock_keys,"research continuation.clock");
    if(integer(required(clock,"step"))!=expected_step||number(required(clock,"time_s"))!=expected_time||
        string(required(clock,"seed_decimal"))!=config_.seed_decimal||
        string(required(clock,"configuration_sha256"))!=research_configuration_digest(config_)||
        string(required(clock,"noise_tape_sha256"))!=research_tape_digest(reset_tape_))
        throw std::invalid_argument("research continuation.clock: plant clock/seed identity differs");
    auto next=std::make_unique<ResearchEpisode>(config_,reset_tape_,reset_startup_);
    next->next_sensor_step=integer(required(clock,"next_sensor_step"));next->sample_cursor=integer(required(clock,"sample_cursor"));
    if(next->next_sensor_step<1||next->next_sensor_step>config_.max_steps+config_.sensor_steps||next->sample_cursor<-1||next->sample_cursor>=expected_step)
        throw std::invalid_argument("research continuation.clock: invalid acquisition/publication cursor");
    next->applied=research_control_from_wire(object(required(clock,"applied")));
    next->motor_applied=research_control_from_wire(object(required(clock,"motor_applied")));
    control_validate_for(next->applied,plant.physical_->rider_present(),plant.physical_->drive_mode());
    control_validate_for(next->motor_applied,plant.physical_->rider_present(),plant.physical_->drive_mode());
    next->last_requested.reset();if(!is_none(required(clock,"last_requested")))next->last_requested=continuation_command(object(required(clock,"last_requested")),true);
    const auto& delay=section(wire,"actuator_delay");constexpr std::array<std::string_view,1> delay_keys{"steps"};exact(delay,delay_keys,"research continuation.actuator_delay");
    if(integer(required(delay,"steps"))!=config_.delay_steps)throw std::invalid_argument("research continuation.actuator_delay: configuration differs");
    continuation_commands(*next,wire,plant,expected_step);
    continuation_metrics(*next,wire,plant,expected_step,expected_time);
    return next;
}
void NativeResearchRuntime::continuation_commands(ResearchEpisode& next,
    const WireObject& wire,const NativeRideRuntime& plant,std::int64_t expected_step) const {
    next.commands.clear();std::int64_t previous=-1;
    for(const auto& row:array(required(wire,"pending_commands"))){const auto& command=object(row);
        constexpr std::array<std::string_view,2> command_keys{"step","control"};exact(command,command_keys,"research continuation.pending_commands");
        const auto step=integer(required(command,"step"));if(step<expected_step||step<=previous||step>config_.max_steps+config_.delay_steps)
            throw std::invalid_argument("research continuation.pending_commands: inconsistent deadlines");
        const auto control=research_control_from_wire(object(required(command,"control")));
        control_validate_for(control,plant.physical_->rider_present(),plant.physical_->drive_mode());
        next.commands.emplace_back(step,control);previous=step;}
    next.window.reset();if(!is_none(required(wire,"window"))){const auto& window=object(required(wire,"window"));
        constexpr std::array<std::string_view,6> window_keys{"control","front","rear","start_step","target_step","seen_demand"};exact(window,window_keys,"research continuation.window");
        ResearchWindow value;value.control=research_control_from_wire(object(required(window,"control")));
        control_validate_for(value.control,plant.physical_->rider_present(),plant.physical_->drive_mode());
        value.front=field(window,"front");value.rear=field(window,"rear");validate_brake(value.front);validate_brake(value.rear);
        value.start_step=integer(required(window,"start_step"));value.target_step=integer(required(window,"target_step"));
        if(value.start_step<0||value.start_step>expected_step||value.target_step<expected_step||value.target_step>config_.max_steps||value.target_step<=value.start_step)
            throw std::invalid_argument("research continuation.window: inconsistent plant/control deadline");
        if(!is_none(required(window,"seen_demand")))value.seen_demand=field(window,"seen_demand");
        if(value.seen_demand&&*value.seen_demand<0.)throw std::invalid_argument("research continuation.window.seen_demand: negative");
        next.window=value;}
    const auto& ack=section(wire,"ack");constexpr std::array<std::string_view,3> ack_keys{"delivery_pending","stop_requested","pending_transition"};exact(ack,ack_keys,"research continuation.ack");
    next.delivery_pending=boolean(required(ack,"delivery_pending"));next.stop_requested=boolean(required(ack,"stop_requested"));
    next.pending_transition.reset();if(!is_none(required(ack,"pending_transition")))next.pending_transition=continuation_transition(object(required(ack,"pending_transition")));
    if(next.delivery_pending!=next.pending_transition.has_value()||(next.delivery_pending&&!next.window))
        throw std::invalid_argument("research continuation.ack: inconsistent pending transition/window");
}
void NativeResearchRuntime::continuation_metrics(ResearchEpisode& next,
    const WireObject& wire,const NativeRideRuntime& plant,std::int64_t expected_step,double expected_time) const {
    const auto& metrics=section(wire,"metrics_cursors");
    constexpr std::array<std::string_view,12> metric_keys{"requested_count","completed_count","applied_base","observation","spatial_metrics","spatial_truth","episode","requested","applied","observations","transitions","recorder"};
    exact(metrics,metric_keys,"research continuation.metrics_cursors");
    next.requested_count=integer(required(metrics,"requested_count"));next.completed_count=integer(required(metrics,"completed_count"));next.applied_base=integer(required(metrics,"applied_base"));
    if(next.requested_count<0||next.completed_count<0||next.completed_count>next.requested_count||next.applied_base<0)
        throw std::invalid_argument("research continuation.metrics_cursors: invalid counters");
    next.observation=research_observation_from_wire(object(required(metrics,"observation")));
    if(!std::holds_alternative<spatial::SensorObservationV2>(next.observation)||std::get<spatial::SensorObservationV2>(next.observation).time_s>expected_time)
        throw std::invalid_argument("research continuation.observation: schema/time differs");
    next.spatial_truth.reset();if(!is_none(required(metrics,"spatial_truth")))next.spatial_truth=checked_spatial_truth(object(required(metrics,"spatial_truth")));
    next.spatial_metrics.reset();if(!is_none(required(metrics,"spatial_metrics"))){const auto& values=object(required(metrics,"spatial_metrics"));
        constexpr std::array<std::string_view,16> spatial_keys{"spatial_schema","requested_duration_s","valid_duration_s","completed_distance_m","terminal_reason","terminal_tick",
            "front_lateral_slip_path_m","rear_lateral_slip_path_m","front_unloaded_s","front_lift_s","flight_s","valid","start_position_m","accepted_origin_time_s","last_end_time_s","last_interval"};
        exact(values,spatial_keys,"research continuation.spatial_metrics");if(integer(required(values,"spatial_schema"))!=1)throw std::invalid_argument("research continuation.spatial_metrics: unknown schema");
        SpatialEpisodeMetrics value;value.requested_duration_s=field(values,"requested_duration_s");value.valid_duration_s=field(values,"valid_duration_s");
        value.completed_distance_m=field(values,"completed_distance_m");value.start_position_m=field(values,"start_position_m");
        value.accepted_origin_time_s=field(values,"accepted_origin_time_s");value.last_end_time_s=field(values,"last_end_time_s");value.last_interval=integer(required(values,"last_interval"));
        value.valid=boolean(required(values,"valid"));value.terminal_tick=integer(required(values,"terminal_tick"));
        value.lateral_slip_path_m={field(values,"front_lateral_slip_path_m"),field(values,"rear_lateral_slip_path_m")};
        value.front_unloaded_s=field(values,"front_unloaded_s");value.front_lift_s=field(values,"front_lift_s");value.flight_s=field(values,"flight_s");
        const auto label=string(required(values,"terminal_reason"));bool known=false;
        for(const auto reason:{bike::spatial::SpatialReason::none,bike::spatial::SpatialReason::capsize,bike::spatial::SpatialReason::off_corridor,
            bike::spatial::SpatialReason::balance_lost,bike::spatial::SpatialReason::terrain_invalid,bike::spatial::SpatialReason::terrain_outside,
            bike::spatial::SpatialReason::numerical_failure,bike::spatial::SpatialReason::requested_completion})if(label==bike::spatial::to_string(reason)){known=true;value.terminal_reason=reason;break;}
        if(!known||value.last_interval!=next.sample_cursor||value.last_interval<0||value.last_end_time_s>expected_time||
            value.last_end_time_s<0.||value.accepted_origin_time_s<0.||value.accepted_origin_time_s>value.last_end_time_s||
            value.terminal_tick>=expected_step||value.terminal_tick<-1||
            ((value.terminal_reason==bike::spatial::SpatialReason::none)!=(value.terminal_tick==-1))||
            value.requested_duration_s!=static_cast<double>(config_.max_steps)*config_.timestep_s||value.start_position_m!=config_.start_position_m||
            value.valid_duration_s<0.||value.valid_duration_s>value.requested_duration_s||value.completed_distance_m<0.||
            value.lateral_slip_path_m[0]<0.||value.lateral_slip_path_m[1]<0.||value.front_unloaded_s<0.||value.front_lift_s<0.||value.flight_s<0.||
            value.flight_s>value.front_lift_s||value.front_lift_s>value.last_end_time_s-value.accepted_origin_time_s+config_.timestep_s)
            throw std::invalid_argument("research continuation.spatial_metrics: inconsistent state");
        next.spatial_metrics=value;}
    const auto& episode=section(metrics,"episode");constexpr std::array<std::string_view,14> episode_keys{"terminated","truncated","numerically_valid","reason","error","cleanup_notes",
        "max_energy_residual_ratio","torque_delivered_nms","torque_requested_nms","demand_integral_nms","demand_nm","max_shock_stroke_m","max_fork_travel_m","checkpoint_unavailable_reason"};exact(episode,episode_keys,"research continuation.episode");
    next.terminated=boolean(required(episode,"terminated"));next.truncated=boolean(required(episode,"truncated"));next.numerically_valid=boolean(required(episode,"numerically_valid"));
    const auto optional_text=[&](std::string_view key){const auto& v=required(episode,key);return is_none(v)?std::nullopt:std::optional(string(v));};
    const auto optional_number=[&](std::string_view key){const auto& v=required(episode,key);return is_none(v)?std::nullopt:std::optional(number(v));};
    next.reason=optional_text("reason");next.error=optional_text("error");next.demand_nm=optional_number("demand_nm");next.demand_integral_nms=optional_number("demand_integral_nms");
    next.checkpoint_unavailable_reason=optional_text("checkpoint_unavailable_reason");
    if(next.checkpoint_unavailable_reason)throw std::invalid_argument("research continuation: failed boundary is diagnostic evidence, not resumable state");
    next.max_energy_residual_ratio=field(episode,"max_energy_residual_ratio");next.torque_delivered_nms=field(episode,"torque_delivered_nms");next.torque_requested_nms=field(episode,"torque_requested_nms");
    next.max_shock_stroke_m=field(episode,"max_shock_stroke_m");next.max_fork_travel_m=field(episode,"max_fork_travel_m");
    if((next.terminated&&next.truncated)||next.max_energy_residual_ratio<0.||next.max_shock_stroke_m<0.||next.max_fork_travel_m<0.||
        (next.demand_nm&&*next.demand_nm<0.)||(next.demand_integral_nms&&*next.demand_integral_nms<0.))
        throw std::invalid_argument("research continuation.episode: invalid outcome or metric domain");
    next.cleanup_notes.clear();for(const auto& note:array(required(episode,"cleanup_notes")))next.cleanup_notes.push_back(string(note));
    next.requested.clear();next.applied_records.clear();next.observations.clear();next.transitions.clear();
    for(const auto& row:array(required(metrics,"requested")))next.requested.push_back(continuation_command(object(row),true));
    for(const auto& row:array(required(metrics,"applied")))next.applied_records.push_back(continuation_command(object(row),false));
    for(const auto& row:array(required(metrics,"observations"))){const auto value=research_observation_from_wire(object(row));
        if(!std::holds_alternative<spatial::SensorObservationV2>(value))throw std::invalid_argument("research continuation: history schema mismatch");
        next.observations.push_back(value);}
    for(const auto& row:array(required(metrics,"transitions")))next.transitions.push_back(continuation_transition(object(row)));
    const auto check_command_history=[&](const std::vector<ResearchCommand>& history,bool strict) {
        std::int64_t prior=-1;
        for(const auto& row:history) {
            if(row.step<prior||(strict&&row.step==prior)||row.step>expected_step||row.time_s>expected_time||
                std::abs(row.time_s-static_cast<double>(row.step)*config_.timestep_s)>
                    .25*config_.timestep_s)
                throw std::invalid_argument("research continuation.history: invalid command clock");
            control_validate_for(row.control,plant.physical_->rider_present(),plant.physical_->drive_mode());
            prior=row.step;
        }
    };
    check_command_history(next.requested,true);check_command_history(next.applied_records,false);
    if(next.last_requested)check_command_history(std::vector<ResearchCommand>{*next.last_requested},true);
    if(next.last_requested.has_value()!=(next.requested_count>0)||next.requested_count-next.completed_count>1||
        (next.window&&next.requested_count-next.completed_count!=(next.delivery_pending?0:1)))
        throw std::invalid_argument("research continuation.history: request/window counters disagree");
    if(keeps_history(config_.capture_mode)) {
        if(next.requested.size()!=static_cast<std::size_t>(next.requested_count)||
            next.transitions.size()!=static_cast<std::size_t>(next.completed_count)||
            next.observations.size()!=static_cast<std::size_t>(next.completed_count)+1)
            throw std::invalid_argument("research continuation.history: incomplete retained control prefix");
        if(bike::io::canonical_json(checkpoint_value(research_observation_wire(next.observations.back())))!=
           bike::io::canonical_json(checkpoint_value(research_observation_wire(next.observation))))
            throw std::invalid_argument("research continuation.history: current observation differs from delivered prefix");
        if(next.last_requested&&bike::io::canonical_json(checkpoint_value(next.last_requested->as_wire(true)))!=
           bike::io::canonical_json(checkpoint_value(next.requested.back().as_wire(true))))
            throw std::invalid_argument("research continuation.history: last request differs from retained prefix");
    } else if(!next.requested.empty()||!next.observations.empty()||!next.transitions.empty())
        throw std::invalid_argument("research continuation.history: retained rows conflict with capture mode");
    double previous_time=-1.;
    for(const auto& row:next.observations) {
        const auto time=std::get<spatial::SensorObservationV2>(row).time_s;
        if(time<previous_time||time>expected_time)throw std::invalid_argument("research continuation.history: observation clock differs");
        previous_time=time;
    }
    for(std::size_t i=0;i<next.transitions.size();++i) {
        const auto& transition=next.transitions[i];
        if(bike::io::canonical_json(checkpoint_value(research_observation_wire(transition.observation)))!=
            bike::io::canonical_json(checkpoint_value(research_observation_wire(next.observations[i+1]))))
            throw std::invalid_argument("research continuation.history: transition observation differs");
    }
    next.recorder.restore_continuation(section(metrics,"recorder"),plant.accounting_->recorded_samples(),next.sample_cursor);
    const auto& sensor=section(wire,"sensor_v2");constexpr std::array<std::string_view,3> sensor_keys{"schema","config","state"};exact(sensor,sensor_keys,"research continuation.sensor_v2");
    if(integer(required(sensor,"schema"))!=2||bike::io::canonical_json(checkpoint_value(required(sensor,"config")))!=
        bike::io::canonical_json(checkpoint_value(sensor_config_v2_to_wire(*config_.sensors_v2))))throw std::invalid_argument("research continuation.sensor_v2: configuration/schema differs"); // NOLINT(bugprone-unchecked-optional-access) prepare_continuation rejects a missing sensors_v2 before this decode path
    auto sensor_state=decode_sensor_state_v2(checkpoint_value(required(sensor,"state")));
    if((sensor_state.last_time&&*sensor_state.last_time>expected_time)||(sensor_state.delivery_time&&*sensor_state.delivery_time>expected_time))
        throw std::invalid_argument("research continuation.sensor_v2: sensor clock exceeds plant clock");
    const auto acquisitions=next.sample_cursor<0?std::int64_t{1}:next.sample_cursor/config_.sensor_steps+1;
    if(std::cmp_not_equal(sensor_state.cursor,acquisitions)||
        next.next_sensor_step!=acquisitions*config_.sensor_steps||!sensor_state.last_time||!sensor_state.delivery_time||
        std::abs(*sensor_state.last_time-static_cast<double>((acquisitions-1)*config_.sensor_steps)*config_.timestep_s)>
            .25*config_.timestep_s||
        *sensor_state.delivery_time!=std::get<spatial::SensorObservationV2>(next.observation).time_s)
        throw std::invalid_argument("research continuation.sensor_v2: acquisition/delivery cursor disagrees with consumed prefix");
    if(plant.completion_step_!=config_.max_steps)
        throw std::invalid_argument("research continuation: plant completion deadline differs");
    if(next.spatial_metrics.has_value()!=(next.sample_cursor>=0)||next.spatial_truth.has_value()!=(next.sample_cursor>=0))
        throw std::invalid_argument("research continuation: spatial truth/metrics differ from consumed prefix");
    next.sensors_v2->import_state(std::move(sensor_state)); // NOLINT(bugprone-unchecked-optional-access) the ResearchEpisode ctor emplaces sensors_v2 whenever config_.sensors_v2 holds, guarded in prepare_continuation
}
void NativeResearchRuntime::restore_continuation(const WireObject& wire) {
    const ResearchGuard guard(busy_);require_open();auto next=prepare_continuation(wire,*runtime_);
    episode_.swap(next);
}
bike::io::Value NativeResearchRuntime::checkpoint_unlocked() const {
    if(episode_->checkpoint_unavailable_reason)
        throw std::invalid_argument("checkpoint-unavailable: "+*episode_->checkpoint_unavailable_reason);
    auto state=continuation_state_unlocked();const auto sensor=checkpoint_value(required(state,"sensor_v2"));
    std::erase_if(state,[](const auto& member){return member.first=="sensor_v2";});
    using namespace setupvalue;
    return spatial::encode_checkpoint(spatial::capture_checkpoint(*runtime_,checkpoint_value(state),sensor,
        Value{Object{{"resumable",Value{false}},{"state",Value{}}}},spatial::checkpoint_identity(config_.metadata)));
}
bike::io::Value NativeResearchRuntime::checkpoint() const {const ResearchGuard guard(busy_);require_open();return checkpoint_unlocked();}
void NativeResearchRuntime::restore_checkpoint(const bike::io::Value& document) {
    const ResearchGuard guard(busy_);require_open();
    const auto decoded=spatial::decode_checkpoint(document);
    if(bike::io::canonical_json(decoded.identity)!=bike::io::canonical_json(spatial::checkpoint_identity(config_.metadata)))
        throw std::invalid_argument("research checkpoint: execution/input identity differs");
    if(bike::io::as_bool(setupvalue::field(decoded.policy,"resumable"),"policy.resumable"))
        throw std::invalid_argument("research checkpoint: policy continuation is not provided by this host");
    auto candidate=spatial::CheckpointAccess::prepare(decoded,*runtime_);
    auto continuation=object(vread::wire_from_value(decoded.research,"checkpoint.research"));
    continuation.emplace_back("sensor_v2",vread::wire_from_value(decoded.sensor_v2,"checkpoint.sensor_v2"));
    auto next=prepare_continuation(continuation,*candidate);
    runtime_.swap(candidate);episode_.swap(next);
}
OwnedResearchEpisode NativeResearchRuntime::evidence_unlocked(OwnedNativeEpisode physical) const {
    const auto& e = *episode_;
    auto unavailable=e.checkpoint_unavailable_reason;
    if(config_.sensors_v2&&runtime_->boundary_recovery_required_&&!unavailable)
        unavailable="failed physical boundary requires reset; accepted prefix has no complete resumable writer snapshot";
    bike::io::Value final_checkpoint;
    if(config_.sensors_v2&&!unavailable)final_checkpoint=checkpoint_unlocked();
    return OwnedResearchEpisode{.physical = std::move(physical),
        .requested = e.requested, .applied = e.applied_records,
        .observations = e.observations, .transitions = e.transitions,
        .evaluation = status_unlocked(), .final_snapshot = runtime_->snapshot(),
        .configuration = config_, .initial_integration_state = initial_integration_state_,
        .run_metadata = bike::io::Value{bike::io::Value::Object{}},
        .checkpoint_initial=initial_checkpoint_,.checkpoint_final=std::move(final_checkpoint),
        .checkpoint_unavailable_reason=std::move(unavailable)};
}
OwnedResearchEpisode NativeResearchRuntime::evidence_snapshot() const {
    const ResearchGuard guard(busy_); require_open();
    OwnedNativeEpisode physical;
    physical.generation = runtime_->committed_generation_;
    physical.columns = runtime_->accounting_config_.columns;
    if (config_.capture_mode == ResearchCaptureMode::full)
        physical = runtime_->accounting_->seal_episode();
    else if (config_.capture_mode == ResearchCaptureMode::legacy)
        physical = episode_->recorder.seal();
    return evidence_unlocked(std::move(physical));
}
std::size_t OwnedResearchEpisode::row_count(std::string_view stream) const {
    if (stream == "commands_requested") return requested.size();
    if (stream == "commands_applied") return applied.size();
    if (stream == "observations") return observations.size();
    if (stream == "transitions") return transitions.size();
    throw std::invalid_argument("unknown research stream");
}
WireArray OwnedResearchEpisode::batch(std::string_view stream, std::int64_t start,
                                      std::int64_t limit) const {
    const auto count = row_count(stream);
    if (start < 0 || limit <= 0 || std::cmp_greater(start, count))
        throw std::invalid_argument("invalid research batch range");
    const auto first = static_cast<std::size_t>(start);
    const auto size = std::min(count - first, static_cast<std::size_t>(limit));
    WireArray rows;
    rows.reserve(size);
    for (std::size_t index = first; index < first + size; ++index) {
        if (stream == "commands_requested") rows.emplace_back(requested[index].as_wire(true));
        else if (stream == "commands_applied") rows.emplace_back(applied[index].as_wire(false));
        else if (stream == "observations") rows.emplace_back(research_observation_wire(observations[index]));
        else rows.emplace_back(transitions[index].as_wire());
    }
    return rows;
}
OwnedResearchEpisode NativeResearchRuntime::seal_episode() {
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode != ResearchCaptureMode::full)
        throw std::runtime_error("sealing research evidence requires full capture");
    // Sealing never integrates or completes an external window. Frontends
    // drain a requested stop at the existing control boundary first.
    if (runtime_->accounting_->has_raws()) runtime_->flush_accounting();
    return evidence_unlocked(runtime_->accounting_->seal_episode());
}
OwnedResearchEpisode NativeResearchRuntime::legacy_snapshot() const {
    const ResearchGuard guard(busy_); require_open();
    if (!keeps_history(config_.capture_mode))
        throw std::runtime_error("legacy export requires legacy or full capture");
    const auto &e = *episode_;
    if (e.window || e.delivery_pending)
        throw std::runtime_error("finish the external control interval before exporting");
    return evidence_unlocked(config_.capture_mode == ResearchCaptureMode::full
        ? runtime_->accounting_->seal_episode() : e.recorder.seal());
}
Wire NativeResearchRuntime::last_requested() const {
    const ResearchGuard guard(busy_); require_open();
    return episode_->last_requested ? Wire(episode_->last_requested->as_wire(true)) : Wire(nullptr);
}
WireObject NativeResearchRuntime::prepare_applied_since(std::int64_t cursor) const {
    const ResearchGuard guard(busy_); require_open();
    const auto &e = *episode_;
    const auto end = e.applied_base + static_cast<std::int64_t>(e.applied_records.size());
    if (cursor < e.applied_base || cursor > end)
        throw std::invalid_argument("applied cursor is stale or beyond the committed prefix");
    WireArray rows;
    rows.reserve(static_cast<std::size_t>(end - cursor));
    for (auto index = cursor; index < end; ++index)
        rows.emplace_back(e.applied_records[static_cast<std::size_t>(index - e.applied_base)].as_wire(false));
    return {{"rows", std::move(rows)}, {"next_cursor", end}};
}
void NativeResearchRuntime::acknowledge_applied(std::int64_t cursor) {
    const ResearchGuard guard(busy_); require_open();
    auto &e = *episode_;
    const auto end = e.applied_base + static_cast<std::int64_t>(e.applied_records.size());
    if (cursor < e.applied_base || cursor > end)
        throw std::invalid_argument("applied acknowledgement is stale or beyond the committed prefix");
    if (config_.capture_mode == ResearchCaptureMode::verify) {
        // cursor and applied_base are std::int64_t — identical to the
        // container's difference_type on the supported LP64 toolchains, so
        // an explicit cast is redundant (GCC -Wuseless-cast fires on it).
        const auto count = cursor - e.applied_base;
        e.applied_records.erase(e.applied_records.begin(), e.applied_records.begin() + count);
        e.applied_base = cursor;
    }
}
WireArray NativeResearchRuntime::commands_requested() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->requested.size());
    for (const auto &c : episode_->requested) out.emplace_back(c.as_wire(true));
    return out;
}
WireArray NativeResearchRuntime::commands_applied() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->applied_records.size());
    for (const auto &c : episode_->applied_records) out.emplace_back(c.as_wire(false));
    return out;
}
WireArray NativeResearchRuntime::observations() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->observations.size());
    for (const auto &v : episode_->observations) out.emplace_back(research_observation_wire(v));
    return out;
}
WireArray NativeResearchRuntime::transitions() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->transitions.size());
    for (const auto &v : episode_->transitions) out.emplace_back(v.as_wire());
    return out;
}
SampleColumns NativeResearchRuntime::recorded_columns() const {
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode == ResearchCaptureMode::full)
        return make_columns(runtime_->accounting_->recorded_samples(), runtime_->accounting_config_.columns);
    return episode_->recorder.columns();
}
WireArray NativeResearchRuntime::recorded_intervals() const {
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode == ResearchCaptureMode::full) {
        WireArray rows;
        for (const auto &sample : runtime_->accounting_->recorded_samples()) rows.emplace_back(sample->as_wire());
        return rows;
    }
    return episode_->recorder.intervals();
}
void NativeResearchRuntime::reset(ResearchNoiseTape tape, ResearchSensorState startup,
                                  std::optional<std::string> seed_decimal,
                                  std::optional<bike::io::Value> metadata, ResearchAssets assets) {
    const ResearchGuard guard(busy_); require_open();
    reset_unlocked(std::move(tape), std::move(startup), std::move(seed_decimal), std::move(metadata), std::move(assets));
}
std::unique_ptr<NativeRideRuntime> NativeResearchRuntime::make_reset_plant() const {
    return std::make_unique<NativeRideRuntime>(RuntimeBootstrap{
        .model=runtime_->model_image_,.config=runtime_->config_,.state=runtime_->bootstrap_});
}
void NativeResearchRuntime::reset_unlocked(ResearchNoiseTape tape, ResearchSensorState startup,
    std::optional<std::string> seed_decimal, std::optional<bike::io::Value> metadata, ResearchAssets assets) {
    if (episode_->window || episode_->delivery_pending)
        throw std::runtime_error("finish the active external interval before reset");
    if (tape_rows(tape) != static_cast<std::size_t>(1 + config_.max_steps / config_.sensor_steps))
        throw std::invalid_argument("reset noise tape has the wrong number of rows");
    if (seed_decimal && (seed_decimal->empty() || !std::ranges::all_of(*seed_decimal,
            [](char c) { return c >= '0' && c <= '9'; })))
        throw std::invalid_argument("reset seed must be a nonnegative decimal integer");
    auto next_tape = tape;
    auto next_startup = startup;
    auto next_config = config_;
    if (seed_decimal) next_config.seed_decimal = std::move(*seed_decimal);
    if (metadata) next_config.metadata = std::move(*metadata);
    if (assets) next_config.assets = std::move(assets);
    if(next_config.sensors_v2) {
        if(runtime_->committed_generation_==std::numeric_limits<int>::max())
            throw std::overflow_error("native runtime generation overflow");
        // Construct from the original reset origin. Neither the accepted
        // plant nor the sensor seed/history changes until every new owner
        // and the new initial checkpoint have been captured successfully.
        auto plant=make_reset_plant();
        plant->physical_->set_generation(runtime_->committed_generation_+1);
        plant->initialize_accounting();
        auto staged=std::make_unique<NativeResearchRuntime>(std::move(plant),std::move(next_config),
            std::move(tape),std::move(startup));
        // Preserve the established reset contract: close the outgoing
        // accounting period first, so a strict rejection remains readable.
        runtime_->flush_accounting();
        staged->runtime_->pending_warning_=runtime_->pending_warning_;
        if(!staged->runtime_->pending_warning_) {
            auto monitor=runtime_->accounting_->state().monitor;
            staged->runtime_->pending_warning_=monitor.take_warning();
        }
        staged->initial_checkpoint_=staged->checkpoint_unlocked();
        runtime_.swap(staged->runtime_);episode_.swap(staged->episode_);
        using std::swap;
        swap(config_,staged->config_);reset_tape_.swap(staged->reset_tape_);
        reset_startup_.swap(staged->reset_startup_);
        initial_integration_state_.swap(staged->initial_integration_state_);
        initial_checkpoint_.data.swap(staged->initial_checkpoint_.data);
        return;
    }
    auto next = std::make_unique<ResearchEpisode>(next_config, std::move(tape), std::move(startup));
    // Physical reset owns its generation and poisoned-context recovery.
    runtime_->reset();
    episode_ = std::move(next);
    reset_tape_ = std::move(next_tape);
    reset_startup_ = std::move(next_startup);
    config_ = std::move(next_config);
}
void NativeResearchRuntime::reset() {
    const ResearchGuard guard(busy_); require_open();
    reset_unlocked(reset_tape_, reset_startup_, std::nullopt, std::nullopt, {});
}
void NativeResearchRuntime::stop() {
    const ResearchGuard guard(busy_); require_open();
    auto &e = *episode_;
    if (e.done()) return;
    if (e.window) e.stop_requested = true;
    else { e.truncated = true; e.reason = "operator_stop"; }
}
void NativeResearchRuntime::fail_policy(std::string message) {
    const ResearchGuard guard(busy_); require_open();
    if (episode_->window) throw std::runtime_error("cannot fail a policy inside an active native interval");
    if (!episode_->done()) {
        episode_->terminated = true;
        episode_->reason = "policy_error";
        episode_->error = std::move(message);
    }
}
void NativeResearchRuntime::set_error_text(std::string message) {
    const ResearchGuard guard(busy_); require_open();
    if (!episode_->terminated || episode_->delivery_pending)
        throw std::runtime_error("error text requires a failed research episode");
    episode_->error = std::move(message);
}
void NativeResearchRuntime::close() {
    const ResearchGuard guard(busy_);
    if (closed_.load(std::memory_order_acquire)) return;
    runtime_->close();
    closed_.store(true, std::memory_order_release);
}
void NativeResearchRuntime::test_fail_at_step(std::int64_t step) {
    const ResearchGuard guard(busy_); require_open();
    if (episode_->window) throw std::runtime_error("arm failure before beginning a research interval");
    runtime_->test_fail_at_step(step);
}
std::optional<std::string> NativeResearchRuntime::take_pending_warning() {
    // The pending non-strict rejection is pure runtime state; the
    // RuntimeWarning emission stays the adapter's job at the held-GIL
    // boundary — identical delegation to NativeRideRuntime.
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->take_pending_warning();
}
} // namespace runtime
