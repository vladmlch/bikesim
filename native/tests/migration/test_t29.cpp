// T29 analytic soft-constraint response and transactional state contracts.
// These regressions are authored only; runtime/bitwise gates remain pending.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "rider/constraint_dynamics.hpp"
#include "rider/activation_allocation.hpp"
#include "rider/force_requests.hpp"
#include "rider/response_codec.hpp"
#include "rider/state_codec.hpp"
#include "numerics/dense.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include <array>
#include <cmath>
#include <numbers>
#include <limits>
#include <set>

namespace {
namespace r=bike::rider;
namespace d=bike::numerics::dense;
r::ConstraintSystem scalar_constraint(int type,double known=-2.,double friction=0.) {
    return {.mass={.rows=1,.cols=1,.values={1.}},.jacobian={.rows=1,.cols=1,.values={1.}},.actuation={.rows=1,.cols=1,.values={1.}},.known_force={known},.resistance={1.},.reference_acceleration={0.},.frictionloss={friction},.types={type},.ids={0}};
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void affine_modes() {
    r::ConstraintDynamics dynamics(scalar_constraint(mjCNSTR_LIMIT_JOINT));
    const auto free=dynamics.linearize(std::array<double,1>{0.});
    bt::require(free.mode.empty()&&free.force_offset[0]==1.&&free.force_matrix.values[0]==-.5,"interior soft-constraint law");
    bt::require(free.acceleration_offset[0]==-1.&&free.acceleration_matrix.values[0]==.5,"constraint force and acceleration use same response");
    bt::require(d::multiply(free.domain_matrix,std::array<double,1>{4.})[0]>free.domain_bound[0],"affine interior cannot be used beyond unilateral domain");
    const auto released=dynamics.linearize(std::array<double,1>{4.});
    bt::require(released.mode==r::ConstraintMode({{0,0.}})&&released.force_offset[0]==0.&&released.acceleration_matrix.values[0]==1.,"unilateral force releases at zero");
    r::ConstraintDynamics const friction(scalar_constraint(mjCNSTR_FRICTION_TENDON,0.,1.));
    const auto saturated=friction.linearize(std::array<double,1>{4.});
    bt::require(saturated.mode==r::ConstraintMode({{0,-1.}})&&saturated.force_offset[0]==-1.,"friction bounds keep tendon loss finite");
    bt::invalid([&]{(void)dynamics.response_for_mode({{0,2.}});});
    bt::invalid([]{r::ConstraintDynamics const elliptical(scalar_constraint(mjCNSTR_CONTACT_ELLIPTIC));});
    const r::ModeSolver failed=[](const r::ConstraintResponse& response){return r::ModeCandidate{.allocation={.solution={0.},.feasible=false,.violation=response.mode.empty()?1.:2.},.payload_index=0};};
    const auto truncated=r::search_constraint_modes(dynamics,std::array<double,1>{0.},failed,{},1);
    bt::require(!truncated.complete&&truncated.modes_tried==1&&!truncated.allocation.feasible,"truncated search is not a mechanical impossibility certificate");
    const auto complete=r::search_constraint_modes(dynamics,std::array<double,1>{0.},failed,{},2);
    bt::require(complete.complete&&complete.modes_tried==2&&!complete.allocation.feasible,"last distinct mode exhausts queue without phantom duplicates");
    const auto across=r::search_constraint_modes(dynamics,std::array<double,1>{0.},[](const r::ConstraintResponse& response){return r::ModeCandidate{.allocation=response.mode.empty()?r::Allocation{.solution={0.},.feasible=false,.violation=1.}:r::Allocation{.solution={4.},.feasible=true,.violation=0.},.payload_index=1};});
    bt::require(across.allocation.feasible&&across.modes_tried==2&&across.response.mode==released.mode,"search crosses the wish's infeasible local mode");
}
[[gnu::noinline]] void activation_and_forces() {
    r::ActivationLaw const law({0.,0.},{1.,2.},{-100.,-100.},{100.,100.},std::numbers::ln2,1.,60.);
    const std::array<double,2> excitation{80.,80.};const auto latent=law.latent(excitation),delivered=law.delivered(excitation);
    bt::require(std::abs(latent[0]-40.)<1e-12&&std::abs(delivered[0]-20.)<1e-12&&std::abs(delivered[1]-20.)<1e-12,"latent activation is distinct from whole-body power-limited delivery");
    const auto derivative=law.derivative(excitation);const double epsilon=1e-5;
    for(std::size_t column=0;column<2;++column){auto plus=excitation,minus=excitation;plus[column]+=epsilon;minus[column]-=epsilon;const auto a=law.delivered(plus),b=law.delivered(minus);for(std::size_t row=0;row<2;++row)bt::require(std::abs((a[row]-b[row])/(2.*epsilon)-derivative.values[row*2+column])<1e-8,"power-normalized activation Jacobian");}
    const std::vector<std::size_t> ti{0,1},ei{2,3};const std::vector<double> scales{50.,50.,50.,50.};
    const auto point=law.project(std::array<double,4>{0.,0.,1.6,1.6},ti,ei,scales);const auto residual=law.constraint(ti,ei,scales).evaluate(point);
    bt::require(d::norm(residual)<1e-12,"post-solve activation projection satisfies its equality");
    const auto force=r::feasible_pedal_force_request({100.,0.,-30.},{0.,0.,1.},.5,20.);
    bt::require(force==r::Force3({10.,0.,-30.}),"foot request obeys measured-load friction without erasing compression");
    const auto requests=r::pedaling_force_requests(0.,20.,.2,{{{0.,0.,1.},{0.,0.,1.}}},{100.,100.},.8,0.);
    bt::require(std::abs(-.2*requests.force[0][2]+.2*requests.force[1][2]-20.)<1e-10,"two-foot request accounts for net shaft moment");
    bt::invalid([]{(void)r::feasible_pedal_force_request({0.,1.,0.},{0.,0.,1.},1.,1.);});
}
[[gnu::noinline]] void state_and_config() {
    ::rider::SpindleController::State state;state.active_state={4.,5.};state.pd_split={{"rider_hip_front",{1.,2.}}};state.pedal_recovery={{"front",{}},{"rear",{}}};
    state.allocation_memory=r::AllocationMemory{.branch=std::array<bool,2>{false,true},.solution=std::vector<double>{.1,.2}};
    state.sole_goal_diagnostics={{"front",::rider::decode_sole_goal(bike::io::parse_json(R"({"requested_compression_m":0.02,"applied_compression_m":0.01,"requested_shear_m":0.0,"applied_shear_m":0.0,"saturated":true,"limiting_reasons":["compression"]})"),"goal")},{"rear",::rider::decode_sole_goal(bike::io::parse_json(R"({"saturated":false,"limiting_reasons":[],"recovery":true})"),"goal")}};
    r::ResponseAllocation allocation;allocation.allocation={.solution={.1,.2},.feasible=false,.violation=.25};allocation.torque={3.,4.};allocation.excitation={5.,6.};allocation.activation_state={4.,5.};allocation.power={0.,0.};allocation.lower_torque_nm={-10.,-10.};allocation.upper_torque_nm={10.,10.};allocation.rider_acceleration={0.};allocation.base_acceleration={0.};
    state.allocation_diagnostics.present=true;state.allocation_diagnostics.invalid_controller=true;state.allocation_diagnostics.legacy=allocation;
    const auto document=::rider::encode_spindle_state(state);const auto restored=::rider::decode_spindle_state(document,"state.rider_controller");
    bt::require(restored.allocation_memory.value().branch==state.allocation_memory.value().branch&&restored.allocation_memory.value().solution==state.allocation_memory.value().solution,"warm branch and point roundtrip"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    bt::require(restored.active_state==state.active_state&&restored.pd_split==state.pd_split&&restored.allocation_diagnostics.legacy.value().activation_state==allocation.activation_state,"latent activation and PD components roundtrip"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    bt::require(bike::io::canonical_json(::rider::encode_spindle_state(restored))==bike::io::canonical_json(document),"state codec preserves source-shaped legacy records");
    const auto disabled=r::decode_response_allocation(bike::io::parse_json(R"({"feasible":true,"violation":0.0,"invalid_controller":false})"),"allocation");
    bt::require(disabled.disabled&&bike::io::as_object(r::encode_response_allocation(disabled),"allocation").size()==3,"disabled controller has no invented solver coordinates");
    auto stopped=restored;stopped.allocation_diagnostics.legacy=disabled;stopped.effort_diagnostics.present=true;stopped.effort_diagnostics.budget_present=false;stopped.effort_diagnostics.power_limit_present=false;stopped.effort_diagnostics.observation_present=false;stopped.effort_diagnostics.strength_violations=std::vector<std::string>{};::rider::JointTerms zero;zero.requested_present=false;stopped.last_terms={{"rider_hip_front",zero}};
    const auto stopped_document=::rider::encode_spindle_state(stopped);
    bt::require(bike::io::canonical_json(::rider::encode_spindle_state(::rider::decode_spindle_state(stopped_document,"rider")))==bike::io::canonical_json(stopped_document),"disabled effort and terms preserve omitted source fields");
    auto config=bike::configuration::default_physics();config.articulated.coupled_task_control=true;config.articulated.pedal_attachment="weld";config.articulated.saddle_attachment="weld";config.articulated.grip_attachment="connect";
    auto const full=bike::configuration::detail::plain_document(bike::configuration::to_value(config));const auto& object=bike::io::as_object(full,"config");auto rider=*bike::io::find_member(object,"articulated");auto& members=std::get<bike::io::Value::Object>(rider.data);members.emplace_back("strength",bike::io::Value{});members.emplace_back("strength_coordinates",bike::io::Value{bike::io::Value::Object{}});members.emplace_back("envelopes",bike::io::Value{bike::io::Value::Object{}});
    const auto decoded=::rider::decode_spindle_config(rider);
    bt::require(decoded.legacy&&decoded.legacy->coupled_task_control&&decoded.legacy->pedal_attachment=="weld","normal config path reaches coupled controller parameters");
}
}
int main(){return bt::run([]{affine_modes();activation_and_forces();state_and_config();});}
NATIVE_DIAG_POP
