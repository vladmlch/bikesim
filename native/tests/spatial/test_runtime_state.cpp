// W17 authoring. Execution and all lifecycle qualifications belong to V03.
#include "support/migration_test.hpp"
#include "spatial/support/startup_fixture.hpp"
#include "spatial/bootstrap_state.hpp"
#include "spatial/brake_state.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/runtime.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include "diag.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

namespace {
namespace sp=bike::spatial;
namespace io=bike::io;
using namespace runtime::setupvalue;

template<class F> void invalid_path(F&& operation,std::string_view path) {
    bt::invalid([&] {
        try { operation(); }
        catch(const std::invalid_argument& error) {
            bt::require(std::string_view(error.what()).find(path)!=std::string_view::npos,
                "rejection must name "+std::string(path)+": "+error.what());
            throw;
        }
    });
}
void same(std::span<const double> a,std::span<const double> b,std::string_view message) {
    bt::require(a.size()==b.size(),std::string(message)+" width");
    for(std::size_t i=0;i<a.size();++i)
        bt::require(std::bit_cast<std::uint64_t>(a[i])==std::bit_cast<std::uint64_t>(b[i]),
            std::string(message)+" lane "+std::to_string(i));
}
void hold_roundtrip(const sp::HoldState& a) {
    const auto b=sp::decode_hold(sp::encode_hold(a));
    bt::require(std::tie(a.schema,a.active,a.reference_position_world_m,a.reference_quat_wxyz,
        a.last_force_world_n,a.last_couple_world_nm,a.stored_energy_j,a.work_j,a.phase,a.valid_mask)==
        std::tie(b.schema,b.active,b.reference_position_world_m,b.reference_quat_wxyz,
        b.last_force_world_n,b.last_couple_world_nm,b.stored_energy_j,b.work_j,b.phase,b.valid_mask),
        "hold roundtrip retains every causal field and quaternion sign");
}
void steer_roundtrip(const sp::SteerState& a) {
    const auto b=sp::decode_steer(sp::encode_steer(a));
    bt::require(std::tie(a.schema,a.controller,a.next_tick,a.delay_fifo_nm,a.fifo_head,a.fifo_count,
        a.profile_sha256,a.last_command_nm,a.last_applied_nm,a.saturated_ticks,a.work_j,a.valid_mask)==
        std::tie(b.schema,b.controller,b.next_tick,b.delay_fifo_nm,b.fifo_head,b.fifo_count,
        b.profile_sha256,b.last_command_nm,b.last_applied_nm,b.saturated_ticks,b.work_j,b.valid_mask),
        "steer roundtrip retains FIFO storage, clocks and work");
}
void brake_roundtrip(const sp::BrakeState& a) {
    const auto b=sp::decode_brake(sp::encode_brake(a));
    bt::require(std::tie(a.schema,a.interval_index,a.valid_mask)==std::tie(b.schema,b.interval_index,b.valid_mask),
        "brake roundtrip envelope");
    for(std::size_t i=0;i<2;++i) {
        const auto& x=a.wheel[i];const auto& y=b.wheel[i];
        bt::require(std::tie(x.engaged,x.latch_angle_rad,x.unwrapped_angle_rad,x.stored_energy_j,x.previous_demand)==
            std::tie(y.engaged,y.latch_angle_rad,y.unwrapped_angle_rad,y.stored_energy_j,y.previous_demand),
            "brake roundtrip retains each wheel latch independently");
    }
}
void outcome_roundtrip(const sp::OutcomeState& a) {
    const auto b=sp::decode_outcome(sp::encode_outcome(a));const auto& x=a.latched;const auto& y=b.latched;
    bt::require(std::tie(a.schema,a.capsize_dwell_ticks,a.corridor_dwell_ticks,a.balance_dwell_ticks,a.valid_mask)==
        std::tie(b.schema,b.capsize_dwell_ticks,b.corridor_dwell_ticks,b.balance_dwell_ticks,b.valid_mask),
        "outcome roundtrip retains dwell clocks");
    bt::require(std::tie(x.reason,x.terminal,x.valid,x.tick,x.time_s,x.position_world_m,x.diagnostic)==
        std::tie(y.reason,y.terminal,y.valid,y.tick,y.time_s,y.position_world_m,y.diagnostic),
        "outcome roundtrip retains latched event");
}
void codecs() {
    hold_roundtrip(sp::canonical_inactive_hold());
    steer_roundtrip(sp::canonical_inactive_steer());
    brake_roundtrip(sp::canonical_released_brake());
    outcome_roundtrip(sp::canonical_unlatched_outcome());
    sp::HoldState hold;
    hold.active=true;hold.reference_position_world_m={2.,-.25,1.5};hold.reference_quat_wxyz={-1.,0.,0.,0.};
    hold.last_force_world_n={1.,2.,-3.};hold.last_couple_world_nm={-.5,.25,.125};
    hold.stored_energy_j=3.25;hold.work_j=-2.5;hold.phase=sp::BootstrapPhase::release_check;hold.valid_mask=255;
    hold_roundtrip(hold);
    sp::SteerState steer;
    steer.controller=sp::SteerController::roll_heading;steer.next_tick=1001;steer.delay_fifo_nm={-2.,0.,1.25};
    steer.fifo_head=2;steer.fifo_count=2;steer.profile_sha256=std::string(64,'a');
    steer.last_command_nm=-4.;steer.last_applied_nm=-2.;steer.saturated_ticks=4;steer.work_j=-3.;steer.valid_mask=7;
    steer_roundtrip(steer);
    sp::BrakeState brake;brake.interval_index=28;brake.valid_mask=3;
    brake.wheel[0]={true,-1.25,20.,.125,.75};brake.wheel[1]={false,1.5,-11.,0.,0.};brake_roundtrip(brake);
    sp::OutcomeState outcome;outcome.valid_mask=7;outcome.capsize_dwell_ticks=9;
    outcome.corridor_dwell_ticks=3;outcome.balance_dwell_ticks=5;
    outcome.latched={sp::SpatialReason::terrain_outside,true,false,55,.275,{12.,1.6,.4},"outside finite corridor"};
    outcome_roundtrip(outcome);
    runtime::SpatialBootstrapState inventory;
    inventory.root_kind=sp::RootKind::free6;inventory.topology_fingerprint=std::string(64,'1');
    inventory.terrain_sha256=std::string(64,'2');inventory.calibration_sha256=std::string(64,'3');
    inventory.bootstrap_phase=hold.phase;inventory.hold=hold;inventory.steer=steer;inventory.brake=brake;inventory.outcome=outcome;
    const auto encoded=runtime::encode_spatial_bootstrap(inventory);
    bt::require(io::canonical_json(encoded)==io::canonical_json(runtime::encode_spatial_bootstrap(
        runtime::decode_spatial_bootstrap(encoded))),"complete spatial inventory roundtrip");
}
void invalid_codecs() {
    invalid_path([]{(void)sp::decode_hold(Value{});},"hold");
    invalid_path([]{(void)sp::decode_steer(Value{});},"steer");
    invalid_path([]{(void)sp::decode_brake(Value{});},"brake");
    invalid_path([]{(void)sp::decode_outcome(Value{});},"outcome");
    auto hold=sp::encode_hold(sp::canonical_inactive_hold());set(hold,"unexpected",integer(1));
    invalid_path([&]{(void)sp::decode_hold(hold);},"hold");
    hold=sp::encode_hold(sp::canonical_inactive_hold());set(hold,"reference_quat_wxyz",reals(std::array{2.,0.,0.,0.}));
    invalid_path([&]{(void)sp::decode_hold(hold);},"reference_quat_wxyz");
    hold=sp::encode_hold(sp::canonical_inactive_hold());set(hold,"last_force_world_n",Value{Array{Value{io::Number{"1e309"}},real(0.),real(0.)}});
    invalid_path([&]{(void)sp::decode_hold(hold);},"last_force_world_n");
    auto steer=sp::encode_steer(sp::canonical_inactive_steer());set(steer,"fifo_count",integer(1));
    invalid_path([&]{(void)sp::decode_steer(steer);},"fifo_count");
    steer=sp::encode_steer(sp::canonical_inactive_steer());set(steer,"fifo_head",integer(1));
    invalid_path([&]{(void)sp::decode_steer(steer);},"fifo_head");
    auto brake=sp::encode_brake(sp::canonical_released_brake());set(brake,"schema",integer(2));
    invalid_path([&]{(void)sp::decode_brake(brake);},"schema");
    auto outcome=sp::encode_outcome(sp::canonical_unlatched_outcome());set(outcome,"balance_dwell_ticks",integer(-1));
    invalid_path([&]{(void)sp::decode_outcome(outcome);},"balance_dwell_ticks");
    hold=sp::encode_hold(sp::canonical_inactive_hold());std::get<Object>(hold.data).emplace_back("schema",integer(1));
    invalid_path([&]{(void)sp::decode_hold(hold);},"schema");
}
void families_and_dims() {
    runtime::RuntimeConfig config;config.writer_config=Value{Object{{"schema",integer(1)}}};
    runtime::validate_schema_family(config,1);set(config.writer_config,"schema",integer(2));runtime::validate_schema_family(config,1);
    invalid_path([&]{runtime::validate_schema_family(config,2);},"schema_family");
    config.schema=2;config.spatial.emplace();config.spatial->mode=sp::Mode::spatial3d;config.spatial->root=sp::RootKind::constrained4;
    invalid_path([&]{runtime::validate_schema_family(config,2);},"schema_family");
    config.writer_config=Value{Object{{"schema",integer(3)}, {"spatial",Value{Object{
        {"spatial_contract",integer(1)},{"root_kind",text("constrained")},
        {"topology_fingerprint",text(std::string(64,'a'))},{"terrain_sha256",text(std::string(64,'b'))},
        {"numeric_profile",text("binary64")}}}, {"drive",Value{Object{}}},
        {"suspension",Value{Object{}}},{"resistance",Value{Object{}}},
        {"tire",Value{Object{{"backend",text("pneumatic_3d")},{"pneumatic_3d",Value{Object{}}}}}}};
    runtime::validate_schema_family(config,2);
    invalid_path([&]{runtime::validate_schema_family(config,1);},"schema_family");
    auto rig=spatial_tests::mixed_joint_rig();const auto before=spatial_tests::complete_state_bytes(rig);
    runtime::ModelDims dims;dims.nq=rig.model->nv;dims.nv=rig.model->nq;
    invalid_path([&]{runtime::verify_model_dims(dims,*rig.model);},"model_dims");
    bt::require(spatial_tests::complete_state_bytes(rig)==before,"mixed dimension rejection did not mutate model/data");
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void planar_lifecycle() {
    const auto directory=bt::fixture("bootstrap/welded");
    // Missing pre-existing binary fixture is a V03 failure, never an invented
    // baseline or a success/skip result in this authoring-only phase.
    const auto bootstrap=std::make_unique<runtime::RuntimeBootstrap>(runtime::read_bootstrap(directory));
    auto ride=std::make_unique<runtime::NativeRideRuntime>(*bootstrap);
    const auto fresh=std::make_unique<runtime::RuntimeSnapshot>(ride->snapshot());
    runtime::RideControl control;control.motor_torque_nm=25.;control.human_torque_nm=10.;
    (void)ride->probe_step_inputs(control,.25,.5);
    const auto probe=std::make_unique<runtime::RuntimeSnapshot>(ride->snapshot());
    same(fresh->integration_state,probe->integration_state,"probe integration unchanged");
    bt::require(fresh->model_sha256==probe->model_sha256 && fresh->tire_state_bytes==probe->tire_state_bytes,
        "probe preserved model identity and complete tyre material bytes");
    const auto publications=ride->prepare_samples().samples.size();
    auto wrong=std::make_unique<runtime::RuntimeBootstrap>(*bootstrap);wrong->state.model_digest=std::string(64,'0');
    invalid_path([&]{runtime::NativeRideRuntime rejected(std::move(*wrong));},"model_digest");
    const auto after_reject=std::make_unique<runtime::RuntimeSnapshot>(ride->snapshot());
    same(probe->integration_state,after_reject->integration_state,"rejected constructor cannot mutate existing runtime");
    bt::require(probe->tire_state_bytes==after_reject->tire_state_bytes &&
        ride->prepare_samples().samples.size()==publications,"rejection preserved material/publication state");
    (void)ride->advance(4,control,0.,0.,std::nullopt);
    ride->reset();const auto reset=std::make_unique<runtime::RuntimeSnapshot>(ride->snapshot());
    same(fresh->integration_state,reset->integration_state,"reset restores fresh integration bytes");
    bt::require(fresh->tire_state_bytes==reset->tire_state_bytes && fresh->model_sha256==reset->model_sha256,
        "reset restores fresh material and model identity");
    bt::require(reset->generation==fresh->generation+1,"reset advances only the lifetime generation");
    ride->close();bt::require(ride->closed(),"close releases runtime");
    bool closed_rejected=false;try{(void)ride->snapshot();}catch(const std::logic_error&){closed_rejected=true;}
    bt::require(closed_rejected,"snapshot of a closed runtime is rejected");
}
NATIVE_DIAG_POP
} // namespace

int main() {
    int failures=0;
    failures+=bt::run(codecs);failures+=bt::run(invalid_codecs);
    failures+=bt::run(families_and_dims);failures+=bt::run(planar_lifecycle);
    return failures?1:0;
}
