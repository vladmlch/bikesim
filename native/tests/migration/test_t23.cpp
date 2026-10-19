// T23 analytic/control regressions for the owned dense TRF provider and
// physical sag acceptance. These do not claim SciPy/LAPACK bit identity.
#include "support/migration_test.hpp"
#include "drivetrain/state_codec.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/sag_fit.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "numerics/least_squares.hpp"
#include "rider/state_codec.hpp"
#include "runtime/runtime.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/profile.hpp"
#include "writer_config.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
namespace num=bike::numerics;
namespace init=bike::initialization;

struct PhysicalCase {
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
    std::unique_ptr<init::SetupContext> context;
};

// NOLINTBEGIN(modernize-make-unique) make_unique's inlined forwarding path keeps each
// by-value result on this frame and overflows the 8192-byte budget; new T(f()) elides.
std::unique_ptr<PhysicalCase> physical_case() {
    auto request=std::unique_ptr<init::SetupRequest>{new init::SetupRequest(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml")))};
    auto compiled=bike::model::compile_mjcf(bike::model::generate_mjcf(request->model));
    const auto field=static_cast<std::size_t>(bike::model::required_id(*compiled,mjOBJ_HFIELD,"road"));
    const auto address=std::views::counted(compiled->hfield_adr,compiled->nhfield)[field];
    const auto samples=bike::terrain::build_field_data(request->track,request->model.field);
    bt::require(address>=0 && std::cmp_less_equal(address,compiled->nhfielddata) &&
        samples.size()<=static_cast<std::size_t>(compiled->nhfielddata-address),
        "physical refinement fixture has valid compiled road storage");
    auto const heightfield=std::views::counted(compiled->hfield_data,compiled->nhfielddata)
        .subspan(static_cast<std::size_t>(address),samples.size());
    for(std::size_t i=0;i<samples.size();++i)heightfield[i]=static_cast<float>(samples[i]);
    auto config=std::unique_ptr<runtime::RuntimeConfig>{new runtime::RuntimeConfig(runtime::project_runtime(
        request->model,*compiled,request->track))};
    const auto image=init::capture_model_image(*compiled);
    auto out=std::make_unique<PhysicalCase>();
    auto const writer_config=std::unique_ptr<nativecfg::WriterConfig>{new nativecfg::WriterConfig(
        nativecfg::decode(config->writer_config))};
    out->stepper=std::make_unique<Stepper>(image.mjb,*writer_config);
    out->physical=runtime::make_physical_step(*out->stepper,*config);
    out->context=std::unique_ptr<init::SetupContext>{new init::SetupContext(init::SetupContext{
        .stepper=*out->stepper,.physical=*out->physical,.physics=request->model.physics.value()})}; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    auto& context=*out->context;
    context.ground_z_m=bike::geometry::compute_ground_z(request->model.bike)/1000.;
    context.frame_body_id=bike::model::required_id(*compiled,mjOBJ_BODY,"frame");
    context.front_geom_id=bike::model::required_id(*compiled,mjOBJ_GEOM,"geom_front_contact");
    context.rear_geom_id=bike::model::required_id(*compiled,mjOBJ_GEOM,"geom_rear_contact");
    context.rider_pose=rider::decode_spindle_pose(config->geometry.pose,"refinement.pose");
    init::initialize_pose(context);

    // Move both independently rooted assemblies together. In free fall no
    // internal spring/rider/drive adjustment can provide the missing road
    // reaction; tiny differences at budget=1 cannot meet the unchanged .05
    // acceptance threshold. This exercises real physical trial evaluations.
    auto const q=std::views::counted(out->stepper->data()->qpos,out->stepper->model()->nq);
    for(auto name:{"root_z","rider_root_z"})
        q[static_cast<std::size_t>(init::joint_address(context,name).qpos)]+=10.;
    out->stepper->note_posvel_stale();
    out->stepper->forward();
    auto tire=out->stepper->tire_state();
    const auto& names=out->stepper->tire_state_names();
    for(std::size_t i=0;i<names.size();++i)
        if(names[i].ends_with(".xi"))tire[i]=.0003;
    out->stepper->set_tire_state(names,tire);
    auto contacts=std::unique_ptr<writers::RiderContactsState>{new writers::RiderContactsState(out->stepper->rider_contacts().state())};
    for(auto& support:contacts->supports)support.xi=.0002;
    for(auto& grip:contacts->grip_xi_local){grip[0]=.0001;grip[2]=-.0001;}
    auto probe=out->stepper->rider_contacts().probe();
    out->stepper->rider_contacts().assign_state(std::move(*contacts),std::move(probe));
    init::apply_initial_forces(context,false);
    out->stepper->forward();
    const auto tires=out->stepper->tire_writer();
    bt::require(tires.probe_committed(),"airborne physical fixture evaluates both tire probes");
    for(const auto& wheel:tires.probe_snapshot_storage())
        for(const auto& patch:wheel.patches)
            bt::require(patch.normal_load_n==0.,"airborne refinement has no road support");
    return out;
}
// NOLINTEND(modernize-make-unique)

struct MaterialSnapshot {
    double time_s=0.;
    std::vector<double> qpos,qvel,tire;
    std::vector<std::string> tire_names;
    std::unique_ptr<drivetrain::DriveSnapshot> drive;
    std::unique_ptr<runtime::StepState> physical;
    std::unique_ptr<writers::RiderContactsState> rider;
    std::unique_ptr<std::optional<writers::RiderContactsProbe>> rider_probe;
};

MaterialSnapshot snapshot(const PhysicalCase& owner) {
    MaterialSnapshot out;
    out.time_s=owner.stepper->time();
    out.qpos.assign(owner.stepper->qpos().begin(),owner.stepper->qpos().end());
    out.qvel.assign(owner.stepper->qvel().begin(),owner.stepper->qvel().end());
    out.tire=owner.stepper->tire_state();
    out.tire_names=owner.stepper->tire_state_names();
    out.drive=std::make_unique<drivetrain::DriveSnapshot>(owner.stepper->drive().state());
    out.physical=std::make_unique<runtime::StepState>(owner.physical->state());
    out.rider=std::make_unique<writers::RiderContactsState>(owner.stepper->rider_contacts().state());
    out.rider_probe=std::make_unique<std::optional<writers::RiderContactsProbe>>(
        owner.stepper->rider_contacts().probe());
    return out;
}

void exact(std::span<const double> actual,std::span<const double> expected,std::string_view label) {
    bt::require(actual.size()==expected.size(),label);
    for(std::size_t i=0;i<actual.size();++i)
        bt::require(std::bit_cast<std::uint64_t>(actual[i])==std::bit_cast<std::uint64_t>(expected[i]),
            std::string(label)+" at "+std::to_string(i));
}
void exact_value(const bike::io::Value& actual,const bike::io::Value& expected,std::string_view label) {
    bt::require(bike::io::recording_json(actual)==bike::io::recording_json(expected),label);
}

void same_material(const MaterialSnapshot& actual,const MaterialSnapshot& expected) {
    bt::require(actual.time_s==expected.time_s && actual.physical->step==expected.physical->step,
        "refinement does not advance the physical clock");
    exact(actual.qpos,expected.qpos,"selected physical coordinates");
    exact(actual.qvel,expected.qvel,"selected physical velocities");
    bt::require(actual.tire_names==expected.tire_names,"canonical tire material columns");
    exact(actual.tire,expected.tire,"rejected candidates do not leak tire material");
    exact_value(drivetrain::encode_drive_snapshot(*actual.drive),drivetrain::encode_drive_snapshot(*expected.drive),
        "rejected candidates do not leak drive/freehub state");
    exact_value(rider::encode_rider_contacts_state(*actual.rider,*actual.rider_probe),
        rider::encode_rider_contacts_state(*expected.rider,*expected.rider_probe),
        "rejected candidates do not leak rider material or probe state");
    bt::require(actual.physical->rider_controller && expected.physical->rider_controller,
        "physical regression exercises the articulated controller");
    exact_value(rider::encode_spindle_state(actual.physical->rider_controller.value()), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
        rider::encode_spindle_state(expected.physical->rider_controller.value()), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
        "rejected candidates do not leak rider control state");
    exact_value(rider::encode_intent_state(actual.physical->rider_intent),
        rider::encode_intent_state(expected.physical->rider_intent),"rider intent remains isolated");
    exact_value(rider::encode_intent_signals(actual.physical->signals),
        rider::encode_intent_signals(expected.physical->signals),"rider feedback remains isolated");
}

void evaluate_selected_once(PhysicalCase& reference,const MaterialSnapshot& incoming,
                            const MaterialSnapshot& selected) {
    // Independent reference: take the incoming committed material and the
    // reported optimization variables, then evaluate exactly one physical
    // point. Never copy the selected drive, controller, tangents, anchors,
    // clocks, diagnostics or energy into the reference owner.
    reference.physical->restore(*incoming.physical);
    reference.stepper->mutate([&]{reference.stepper->drive().restore(*incoming.drive);});
    auto tire=incoming.tire;
    for(std::size_t i=0;i<tire.size();++i)
        if(incoming.tire_names[i].ends_with(".xi"))tire[i]=selected.tire[i];
    reference.stepper->set_tire_state(incoming.tire_names,tire);
    auto contacts=std::make_unique<writers::RiderContactsState>(*incoming.rider);
    for(std::size_t i=0;i<contacts->supports.size();++i)
        contacts->supports[i].xi=selected.rider->supports[i].xi;
    for(std::size_t side=0;side<contacts->grip_xi_local.size();++side)
        for(std::size_t const axis:{0U,2U})contacts->grip_xi_local[side][axis]=selected.rider->grip_xi_local[side][axis];
    auto probe=*incoming.rider_probe;
    reference.stepper->rider_contacts().assign_state(std::move(*contacts),std::move(probe));
    auto const* data=reference.stepper->data();const auto* model=reference.stepper->model();
    std::ranges::copy(selected.qpos,std::views::counted(data->qpos,model->nq).begin());
    std::ranges::fill(std::views::counted(data->qvel,model->nv),0.);
    reference.stepper->note_posvel_stale();
    init::apply_initial_forces(*reference.context,false);
    reference.stepper->forward();
}

void repeated_insufficient_physical_refinement() {
    auto tested=physical_case(),reference=physical_case();
    same_material(snapshot(*tested),snapshot(*reference));
    const auto root_x=static_cast<std::size_t>(init::joint_address(*tested->context,"root_x").qpos);
    for(int attempt=0;attempt<2;++attempt) {
        const auto incoming=snapshot(*tested);
        // The provider still evaluates the initial three-point Jacobian at
        // max_evaluations=1: many candidates are probed, then exactly one
        // best candidate is retained without claiming physical acceptance.
        const double residual=init::refine_equilibrium(*tested->context,1,.05);
        bt::require(std::isfinite(residual) && residual>.05,
            "insufficient physical refinement cannot satisfy the .05 acceptance gate");
        const auto selected=snapshot(*tested);
        exact(std::span{selected.qpos}.subspan(root_x,1),std::span{incoming.qpos}.subspan(root_x,1),
            "refinement holds root_x at the accepted datum");
        bt::require(!tested->context->completed && !tested->context->equilibrium,
            "insufficient candidate cannot publish a completed equilibrium");
        evaluate_selected_once(*reference,incoming,selected);
        same_material(selected,snapshot(*reference));
    }
}

void analytic_solutions() {
    const std::array<double,2> initial{-.2,.7},lower{-3.,-3.},upper{3.,3.};
    const num::Residual residual=[](std::span<const double> x) {
        return std::vector<double>{2.*(x[0]-1.),x[1]+.5,.25*(x[0]+x[1]-.5)};
    };
    for(auto const difference:{num::DifferenceScheme::two_point,num::DifferenceScheme::three_point}) {
        num::LeastSquaresOptions options;options.difference=difference;
        const auto result=num::bounded_least_squares(residual,initial,lower,upper,options);
        bt::require(result.converged && result.cost<1e-14 &&
                    std::abs(result.x[0]-1.)<1e-6 && std::abs(result.x[1]+.5)<1e-6,
                    "dense TRF solves the overdetermined analytical linear system");
        bt::require(result.residual_calls>result.evaluations && result.jacobian_evaluations>0,
                    "finite-difference evaluations are accounted separately from max_nfev");
    }
    const num::Residual singular=[](std::span<const double> x) {
        return std::vector<double>{x[0]+x[1]-2.,2.*(x[0]+x[1]-2.)};
    };
    const auto rank_one=num::bounded_least_squares(singular,initial,lower,upper,80);
    bt::require(rank_one.converged && std::abs(rank_one.x[0]+rank_one.x[1]-2.)<1e-6,
                "rank-deficient Jacobian is solved without an inverse normal matrix");
    const num::Residual outside=[](std::span<const double> x){return std::vector<double>{x[0]-4.};};
    const auto constrained=num::bounded_least_squares(outside,std::array{0.},std::array{-1.},std::array{1.},80);
    bt::require(constrained.converged && constrained.x[0]<=1. &&
                constrained.x[0]>1.-1e-6 && constrained.cost>4.,
                "a bound-constrained optimum remains inside the declared domain");
}

void invalid_and_budget() {
    const num::Residual valid=[](std::span<const double> x){return std::vector<double>{x[0]-1.,x[1]+2.};};
    const std::array<double,2> x{0.,0.},lower{-3.,-3.},upper{3.,3.};
    const auto exhausted=num::bounded_least_squares(valid,x,lower,upper,1);
    bt::require(!exhausted.converged && exhausted.status==0 && exhausted.evaluations==1,
                "one evaluation is a budget stop, not numerical convergence");
    bt::invalid([&]{(void)num::bounded_least_squares(valid,x,lower,upper,0);});
    bt::invalid([&]{(void)num::bounded_least_squares(valid,std::array{4.,0.},lower,upper,80);});
    bt::invalid([&]{(void)num::bounded_least_squares(valid,x,x,x,80);});
    bt::invalid([&]{(void)num::bounded_least_squares([](std::span<const double>){return std::vector<double>{std::numeric_limits<double>::quiet_NaN()};},x,lower,upper,80);});
    int calls=0;
    bt::invalid([&]{(void)num::bounded_least_squares([&](std::span<const double>){return ++calls==1?std::vector<double>{1.,2.}:std::vector<double>{1.};},x,lower,upper,80);});
    bool original=false;
    try {(void)num::bounded_least_squares([][[noreturn]](std::span<const double>)->std::vector<double>{throw std::runtime_error("physical residual failed");},x,lower,upper,80);}
    catch(const std::runtime_error& error){original=std::string(error.what())=="physical residual failed";}
    bt::require(original,"residual failures propagate with their physical diagnostic");
}

void sag_acceptance() {
    const auto answer=bike::initialization::fit_sag(
        [](double pressure,double rate){return std::array{18000./pressure,50000./rate};},
        std::array{120.,50.},std::array{100.,800.},std::array{40.,100.},std::array{300.,3000.});
    bt::require(std::abs(answer[0]-150.)<.01 && std::abs(answer[1]-1000.)<.01,
                "log-parameter sag fit reports pressure and coil rate in source units");
    bool rejected=false;
    try {(void)bike::initialization::fit_sag([](double,double){return std::array{1.,1.};},
        std::array{120.,50.},std::array{100.,800.},std::array{40.,100.},std::array{300.,3000.});}
    catch(const std::runtime_error&){rejected=true;}
    bt::require(rejected,"optimizer convergence cannot bless an unreachable sag target");
    bt::invalid([&]{(void)bike::initialization::fit_sag([](double,double){return std::array{1.,1.};},
        std::array{0.,50.},std::array{100.,800.},std::array{40.,100.},std::array{300.,3000.});});
}
} // namespace

int main(){return bt::run([]{analytic_solutions();invalid_and_budget();sag_acceptance();repeated_insufficient_physical_refinement();});}
