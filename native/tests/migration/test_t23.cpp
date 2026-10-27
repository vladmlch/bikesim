// T23 analytic/control regressions for the owned dense TRF provider and
// physical sag acceptance. These do not claim SciPy/LAPACK bit identity.
#include "support/migration_test.hpp"
#include "contact/laws.hpp"
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
#include "rider/support_geometry.hpp"
#include "runtime/runtime.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/profile.hpp"
#include "writers/cruise.hpp"
#include "writer_config.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
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

// ---- wave-review findings F2/F3/F5 --------------------------------------
// PhysicalStep::apply_forces parity with physical_runtime.py:305-395. Every
// fixture rides the real setup seam (request -> generate -> compile ->
// project_runtime -> Stepper -> make_physical_step -> initialize_pose), so a
// failure is a defect in the shipped step body, not in scaffolding. Each
// check fails on the pre-correction source: F3's resistance fold is absent
// without the self.tire is None branch, F2's ctrl row reads 0. without the
// sim.cruise.compute call, and F5's stage transition is exactly the flag the
// old active && !initializing_ expression masked.

struct ReviewRig {
    std::unique_ptr<init::SetupRequest> request;
    std::unique_ptr<runtime::RuntimeConfig> config;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
};

// NOLINTBEGIN(modernize-make-unique) same inlined-forwarding frame budget as physical_case.
std::unique_ptr<ReviewRig> review_rig(std::string_view rider_variant,
    const std::function<void(bike::configuration::SimulationPhysicsConfig&)> &edit) {
    auto rig=std::make_unique<ReviewRig>();
    // welded_flat's validated bike/track/effort documents are the base;
    // skip_initial_state because its initial-state payload is pinned to the
    // welded model contract and these rigs intentionally diverge from it.
    rig->request=std::unique_ptr<init::SetupRequest>{new init::SetupRequest(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml"),true))};
    auto& model=rig->request->model;
    model.rider=bike::model::rider_from_args(rider_variant);
    edit(model.physics.value()); // NOLINT(bugprone-unchecked-optional-access) request schema engages physics; .value() reports violations loudly
    bike::configuration::post_init(model.physics.value()); // NOLINT(bugprone-unchecked-optional-access) same contract — revalidates the mutation
    auto compiled=bike::model::compile_mjcf(bike::model::generate_mjcf(model),
        rig->request->asset_root);
    const auto field=static_cast<std::size_t>(
        bike::model::required_id(*compiled,mjOBJ_HFIELD,"road"));
    const auto address=std::views::counted(compiled->hfield_adr,compiled->nhfield)[field];
    const auto samples=bike::terrain::build_field_data(rig->request->track,model.field);
    bt::require(address>=0 && std::cmp_less_equal(address,compiled->nhfielddata) &&
        samples.size()<=static_cast<std::size_t>(compiled->nhfielddata-address),
        "review fixture has valid compiled road storage");
    auto const heightfield=std::views::counted(compiled->hfield_data,compiled->nhfielddata)
        .subspan(static_cast<std::size_t>(address),samples.size());
    for(std::size_t i=0;i<samples.size();++i)heightfield[i]=static_cast<float>(samples[i]);
    rig->config=std::unique_ptr<runtime::RuntimeConfig>{new runtime::RuntimeConfig(
        runtime::project_runtime(model,*compiled,rig->request->track))};
    const auto image=init::capture_model_image(*compiled);
    auto const writers=std::unique_ptr<nativecfg::WriterConfig>{new nativecfg::WriterConfig(
        nativecfg::decode(rig->config->writer_config))};
    rig->stepper=std::make_unique<Stepper>(image.mjb,*writers);
    rig->physical=runtime::make_physical_step(*rig->stepper,*rig->config);
    init::SetupContext context{.stepper=*rig->stepper,.physical=*rig->physical,
        .physics=model.physics.value()}; // NOLINT(bugprone-unchecked-optional-access) same contract
    context.ground_z_m=bike::geometry::compute_ground_z(model.bike)/1000.;
    context.frame_body_id=bike::model::required_id(*compiled,mjOBJ_BODY,"frame");
    context.front_geom_id=bike::model::required_id(*compiled,mjOBJ_GEOM,"geom_front_contact");
    context.rear_geom_id=bike::model::required_id(*compiled,mjOBJ_GEOM,"geom_rear_contact");
    if(rig->physical->rider_present())
        context.rider_pose=rider::decode_spindle_pose(rig->config->geometry.pose,"review.pose");
    init::initialize_pose(context);
    return rig;
}
// NOLINTEND(modernize-make-unique)

int joint_qpos(const mjModel& m,std::string_view name) {
    const int id=bike::model::required_id(m,mjOBJ_JOINT,name);
    return std::views::counted(m.jnt_qposadr,m.njnt)[static_cast<std::size_t>(id)];
}
int joint_dof(const mjModel& m,std::string_view name) {
    const int id=bike::model::required_id(m,mjOBJ_JOINT,name);
    return std::views::counted(m.jnt_dofadr,m.njnt)[static_cast<std::size_t>(id)];
}
void reflow(const ReviewRig& rig) {
    rig.stepper->note_posvel_stale();
    rig.stepper->forward();
}
const std::vector<double>* component(const runtime::PhysicalStep::ProbeResult& probe,
                                     std::string_view name) {
    for(const auto& entry:probe.components)
        if(entry.first==name)return &entry.second;
    return nullptr;
}
double norm2(std::span<const double> values) {
    double out=0.;for(const double v:values)out+=v*v;return out;
}

// F2+F3: physical ideal_speed_control on the native_reference contact
// route — the exact capability combination the review flagged. A lumped
// rider keeps the fixture minimal; the force tail under test is identical.
void native_reference_resistance_and_cruise() {
    auto rig=review_rig("lumped",[](bike::configuration::SimulationPhysicsConfig& p){
        p.tires.backend="native_reference";
        // surface_mode 'track' resolves material names through a tire
        // writer — the no-writer route must declare 'configured'.
        p.tires.surface_mode="configured";
        p.drive_mode="ideal_speed_control";
        p.seated_climb.enabled=false;
        // capabilities.cpp:32-34 — a non-articulated topology must carry
        // the declared default attachments; the welded fixture's
        // spindle/pin/connect selection is articulated-only.
        p.articulated.pedal_attachment="flat";
        p.articulated.saddle_attachment="flat";
        p.articulated.grip_attachment="spring";
    });
    const mjModel* m=rig->stepper->model();
    const mjData* d=rig->stepper->data();
    bt::require(!rig->stepper->has_tire() && rig->stepper->has_cruise() &&
        rig->stepper->has_resistance(),
        "native_reference fixture has no tire writer but keeps the "
        "cruise/resistance owners");
    // initialize_pose leaves the wheel geoms ~5 mm above the road; press
    // the chassis into a real contact load so the engine contacts carry
    // resting weight through refresh_contacts.
    auto const q=std::views::counted(d->qpos,m->nq);
    q[static_cast<std::size_t>(joint_qpos(*m,"root_z"))]-=.012;
    reflow(*rig);
    const runtime::RideControl control{};
    // contacts() mirrors the COMMITTED member — a probe's returned
    // RuntimeContacts is the observable channel on the apply_forces seam.
    const auto settled=rig->physical->apply_forces(true,true,0.,0.,
        std::nullopt,control);
    const auto resting=rig->physical->probe_step_inputs(control,0.,0.);
    bt::require(settled.rear_load_n>1. && settled.front_load_n>1.,
        "the settled fixture loads both wheels through engine contacts");
    // F3 zero-speed control: the named components fold even when they
    // evaluate to nothing (physical_resistance.py:42 returns both rows
    // unconditionally).
    const auto* resting_aero=component(resting,"aerodynamic");
    const auto* resting_roll=component(resting,"road_rolling");
    bt::require(resting_aero!=nullptr && resting_roll!=nullptr,
        "native_reference probes fold the named resistance components");
    bt::require(resting_aero!=nullptr && resting_roll!=nullptr &&
        norm2(*resting_aero)<=1e-9 && norm2(*resting_roll)<=1e-9,
        "a stationary bike carries no resistance work — the valid control");
    // F2 active branch: stationary chassis, maximum positive speed error,
    // grounded rear wheel -> the PI write lands on the rear_drive row.
    const int rear_drive=mj_name2id(m,mjOBJ_ACTUATOR,"rear_drive");
    bt::require(rear_drive>=0,"the ride model keeps its rear_drive actuator");
    const auto adr=static_cast<std::size_t>(rear_drive);
    bt::require(resting.ctrl[adr]>0. && resting.ctrl[adr]<=150.,
        "a grounded rear wheel and positive error write the cruise "
        "torque into rear_drive (physical_runtime.py:390-394)");
    // F2 suppression: a positive brake demand cancels a positive torque
    // and clears the committed torque_nm scalar a second time.
    const auto braking=rig->physical->probe_step_inputs(control,.5,0.);
    bt::require(braking.ctrl[adr]==0.,
        "a front-brake demand suppresses the positive cruise torque");
    bt::require(rig->stepper->cruise().state().torque_nm==0.,
        "suppression clears the committed cruise torque scalar");
    // F2 zero-error control: chassis exactly at the projected target with
    // a cleared integrator commands nothing.
    rig->stepper->mutate([&]{
        auto cs=rig->stepper->cruise().state();
        cs.integral_mps_s=0.;cs.torque_nm=0.;cs.engaged=false;
        rig->stepper->cruise().set_state(cs);
    });
    auto const qv=std::views::counted(d->qvel,m->nv);
    qv[static_cast<std::size_t>(joint_dof(*m,"root_x"))]=
        rig->stepper->cruise().state().target_speed_mps;
    reflow(*rig);
    const auto balanced=rig->physical->probe_step_inputs(control,0.,0.);
    bt::require(balanced.ctrl[adr]==0.,
        "a zero speed error writes zero torque — the response control");
    // F3 moving case: 5 m/s chassis speed plus counter-rotating wheels;
    // drag opposes the chassis velocity and each rolling moment opposes
    // its wheel's omega (component[dof] * qvel[dof] < 0 is dissipation).
    qv[static_cast<std::size_t>(joint_dof(*m,"root_x"))]=5.;
    const int front_spin=joint_dof(*m,"front_wheel_spin"),
              rear_spin=joint_dof(*m,"rear_wheel_spin");
    qv[static_cast<std::size_t>(front_spin)]=-14.;
    qv[static_cast<std::size_t>(rear_spin)]=-14.;
    reflow(*rig);
    const auto moving=rig->physical->probe_step_inputs(control,0.,0.);
    const auto* aero=component(moving,"aerodynamic");
    const auto* roll=component(moving,"road_rolling");
    bt::require(aero!=nullptr && roll!=nullptr,
        "the moving probe keeps both named resistance components");
    bt::require((*aero)[static_cast<std::size_t>(joint_dof(*m,"root_x"))]<-1e-6,
        "aerodynamic drag opposes the +5 m/s chassis speed on root_x");
    bt::require(roll->at(static_cast<std::size_t>(front_spin))*-14.<0. &&
        roll->at(static_cast<std::size_t>(rear_spin))*-14.<0.,
        "the rolling moment dissipates each spinning wheel's power");
}

// F5: flat-pedal articulated rigs differ only in seated_climb.enabled.
// observe_recovery's "none"->"release" gate (legacy_controller.cpp:104)
// needs a sole pad pressing the pedal's bottom face while the sole site
// sits below the pedal origin — estimate()'s force_z is that pad-contact
// force projected on world z, replicated here field-for-field.
::rider::Vec3 as_vec(const mjtNum* p,mjtSize count,int index) {
    const auto cell=std::views::counted(p,static_cast<std::ptrdiff_t>(count)*3)
        .subspan(static_cast<std::size_t>(index)*3,3);
    return {cell[0],cell[1],cell[2]};
}
void as_mat(const mjtNum* p,mjtSize count,int index,::rider::Mat3& out) {
    const auto cell=std::views::counted(p,static_cast<std::ptrdiff_t>(count)*9)
        .subspan(static_cast<std::size_t>(index)*9,9);
    std::ranges::copy(cell,out.begin());
}
double recovery_force_z(const mjModel& m,const mjData& d,std::string_view side,
    double patch_half,double pad_radius,double support_k,double pedal_c) { // NOLINT(bugprone-easily-swappable-parameters) mirrors the rider_contacts field order the gate reads
    const std::string s{side};
    const int geom=bike::model::required_id(m,mjOBJ_GEOM,"geom_pedal_"+s);
    const int sole=bike::model::required_id(m,mjOBJ_SITE,"site_rider_sole_"+s);
    const int foot=bike::model::required_id(m,mjOBJ_BODY,"rider_foot_"+s);
    const auto origin=as_vec(d.geom_xpos,m.ngeom,geom),
        half=as_vec(m.geom_size,m.ngeom,geom);
    ::rider::Mat3 rot,foot_r;
    as_mat(d.geom_xmat,m.ngeom,geom,rot);as_mat(d.xmat,m.nbody,foot,foot_r);
    const auto sole_p=as_vec(d.site_xpos,m.nsite,sole);
    double force_z=0.;
    for(const double sign:{-1.,1.}) {
        const auto center=::rider::add(sole_p,::rider::matvec(foot_r,
            {sign*patch_half,0.,pad_radius}));
        const auto contact=::rider::box_pad_contact(center,pad_radius,origin,rot,half);
        if(!contact.within_footprint||contact.gap_m>=0.)continue;
        const auto body_ids=std::views::counted(m.geom_bodyid,m.ngeom);
        const auto relative=::rider::subtract(
            bike::model::point_velocity(m,d,foot,contact.point_m),
            bike::model::point_velocity(m,d,
                body_ids[static_cast<std::size_t>(geom)],contact.point_m));
        force_z+=contactlaw::normal_contact(-contact.gap_m,
            -::rider::dot(relative,contact.normal),support_k/2.,pedal_c/2.).first
            *contact.normal[2];
    }
    return force_z;
}
bool recovery_gate(const mjModel& m,const mjData& d,std::string_view side,
    double patch_half,double pad_radius,double support_k,double pedal_c) {
    const std::string s{side};
    const int geom=bike::model::required_id(m,mjOBJ_GEOM,"geom_pedal_"+s);
    const int sole=bike::model::required_id(m,mjOBJ_SITE,"site_rider_sole_"+s);
    const auto sites=std::views::counted(d.site_xpos,3*m.nsite);
    const auto geoms=std::views::counted(d.geom_xpos,3*m.ngeom);
    return recovery_force_z(m,d,side,patch_half,pad_radius,support_k,pedal_c)<-1. &&
        sites[static_cast<std::size_t>(sole)*3+2]+pad_radius<
        geoms[static_cast<std::size_t>(geom)*3+2];
}
std::string pedal_stage(const ReviewRig& rig,std::string_view side) {
    const auto state=rig.physical->state();
    bt::require(state.rider_controller.has_value(),
        "the articulated fixture keeps a rider controller state");
    for(const auto& entry:
        state.rider_controller->pedal_recovery) // NOLINT(bugprone-unchecked-optional-access) require() above engaged the state
        if(entry.first==side)return entry.second.stage;
    bt::require(false,"pedal_recovery entries name the front/rear sides");
    return {};
}
void run_recovery_scenario(ReviewRig& on,ReviewRig& off,double angle,const std::string& side);
void seated_climb_recovery_switch() {
    const auto flat=[](bool enabled){
        return review_rig("articulated_planar",[enabled](
            bike::configuration::SimulationPhysicsConfig& p){
            p.articulated.pedal_attachment="flat";
            p.seated_climb.enabled=enabled;});
    };
    auto on=flat(true),off=flat(false);
    const mjModel* m=on->stepper->model();
    const mjData* d=on->stepper->data();
    const auto& a=on->request->model.physics->articulated; // NOLINT(bugprone-unchecked-optional-access) fixture contract
    // The assembled pose is the non-triggering control: soles rest on the
    // pedal top faces, so no pad reaches a bottom face.
    for(const char* side:{"front","rear"})
        bt::require(!recovery_gate(*m,*d,side,a.pedal_patch_half_length_m,
            a.support_pad_radius_m,a.support_k_n_m,a.pedal_c_ns_m),
            "the assembled flat-pedal pose is a valid non-triggering control");
    // Rotate the crank with the pedals counter-spun level; somewhere on
    // the circle the lifted pedal's bottom face overlaps a sole pad.
    const int crank=joint_qpos(*m,"crank_spin"),
        pf=joint_qpos(*m,"pedal_front_spin"),pr=joint_qpos(*m,"pedal_rear_spin");
    auto const q=std::views::counted(d->qpos,m->nq);
    const double base=q[static_cast<std::size_t>(crank)];
    std::optional<std::pair<double,std::string>> armed;
    for(int k=-1000;k<=1000&&!armed.has_value();++k) {
        const double delta=.001*static_cast<double>(k);
        q[static_cast<std::size_t>(crank)]=base+delta;
        q[static_cast<std::size_t>(pf)]=-(base+delta);
        q[static_cast<std::size_t>(pr)]=-(base+delta);
        reflow(*on);
        for(const char* side:{"front","rear"})
            if(recovery_gate(*m,*d,side,a.pedal_patch_half_length_m,
                a.support_pad_radius_m,a.support_k_n_m,a.pedal_c_ns_m))
                {armed.emplace(base+delta,side);break;}
    }
    bt::require(armed.has_value(),
        "the flat-pedal fixture reaches the recovery gate geometry");
    const auto [angle,side]=
        *armed; // NOLINT(bugprone-unchecked-optional-access) require() above engaged it
    run_recovery_scenario(*on,*off,angle,side);
}
void run_recovery_scenario(ReviewRig& on,ReviewRig& off,double angle,const std::string& side) {
    const auto on_state=on.physical->state(),off_state=off.physical->state();
    // advance_physics is the committed interval: its control tick is where
    // compute() receives the pedal_recovery flag, and stepping the
    // integrator hands each replay a fresh timestamp (the drivetrain
    // advances once per timestamp by contract). A probe would roll the
    // recovery state back — legacy_controller.cpp:387 restores
    // pedal_recovery_ verbatim on !advance.
    const auto scenario=[&](ReviewRig& rig,const runtime::StepState& fresh,
                            bool initializing){
        auto stage=fresh;stage.initializing=initializing;
        rig.physical->restore(stage);
        const mjModel* rm=rig.stepper->model();const mjData* rd=rig.stepper->data();
        auto const rq=std::views::counted(rd->qpos,rm->nq);
        rq[static_cast<std::size_t>(joint_qpos(*rm,"crank_spin"))]=angle;
        rq[static_cast<std::size_t>(joint_qpos(*rm,"pedal_front_spin"))]=-angle;
        rq[static_cast<std::size_t>(joint_qpos(*rm,"pedal_rear_spin"))]=-angle;
        reflow(rig);
        (void)rig.physical->advance_physics(0.,0.,std::nullopt,
            runtime::RideControl{});
        return pedal_stage(rig,side);
    };
    // The oracle passes cfg.seated_climb.enabled into every compute —
    // including the init ticks where recovery geometry is first observed.
    bt::require(scenario(on,on_state,true)=="release",
        "seated_climb enabled: the armed gate advances the recovery "
        "state machine on an init tick");
    bt::require(scenario(on,on_state,false)=="release",
        "seated_climb enabled: the same gate fires on a live tick");
    // The pre-fix expression (active && !initializing_) is true on a live
    // tick — this is exactly the case it silently enabled with the switch
    // configured off.
    bt::require(scenario(off,off_state,false)=="none",
        "seated_climb disabled: no recovery transition on a live tick "
        "even when the geometry would trigger it");
    bt::require(scenario(off,off_state,true)=="none",
        "seated_climb disabled: init ticks stay clean as well");
}
} // namespace

int main(){return bt::run([]{analytic_solutions();invalid_and_budget();sag_acceptance();repeated_insufficient_physical_refinement();native_reference_resistance_and_cruise();seated_climb_recovery_switch();});}
