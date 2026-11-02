// Ports tests/reference/test_coupled_rider_task.py:
// test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step —
// the coupled-task invariant that the allocator's crank task coordinate
// equals the first-step solved weld torque (same multipliers, same J^T
// column). F4 regression: the runtime must refresh the forward state after
// staging qfrc_applied and before rider compute(), mirroring
// physical_runtime.py's mj_forward — otherwise ConstraintDynamics snapshots
// the previous interval's qfrc_actuator and the weld projection drifts.
#include "support/fixture_test.hpp"
#include "configuration/physical_config.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/finalize.hpp"
#include "initialization/setup_owner.hpp"
#include "kinematics/solver.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "research/environment.hpp"
#include "rider/response_allocation.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/state_codec.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/profile.hpp"
#include "writers/suspension.hpp"
#include "writer_config.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {
namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace rcfg = ::runtime;

struct WeldedRig {
    std::unique_ptr<init::SetupRequest> request;
    std::unique_ptr<rcfg::RuntimeConfig> config;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<rcfg::PhysicalStep> physical;
};

// NOLINTBEGIN(modernize-make-unique) same inlined-forwarding frame budget as
// test_native_rider_allocation.cpp's welded_rig.
std::unique_ptr<WeldedRig> coupled_rig() {
    auto rig = std::make_unique<WeldedRig>();
    rig->request = std::unique_ptr<init::SetupRequest>{new init::SetupRequest(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml"),
                                 true))};
    auto &model = rig->request->model;
    auto &physics = model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) the request schema engages physics
    physics.articulated.pedal_attachment = "weld";
    physics.articulated.saddle_attachment = "weld";
    physics.articulated.grip_attachment = "weld";
    // The coupled-task contract pair the oracle test's in-place rewrite
    // carries (coupled_rider_task/_environment).
    physics.articulated.coupled_task_control = true;
    physics.articulated.joint_passive_damping_nms_rad = .5;
    cfg::post_init(physics);
    const auto resolved_rider = bike::model::resolve_rider(
        model.rider ? bike::model::RiderArg{*model.rider}
                    : bike::model::RiderArg{std::monostate{}},
        model.include_rider, "lumped");
    model.rider = resolved_rider;
    model.crank_joint = true;
    auto compiled = bike::model::compile_mjcf(
        bike::model::generate_mjcf(model), rig->request->asset_root);
    const auto field = static_cast<std::size_t>(
        bike::model::required_id(*compiled, mjOBJ_HFIELD, "road"));
    const auto address = std::views::counted(compiled->hfield_adr,
                                             compiled->nhfield)[field];
    const auto samples =
        bike::terrain::build_field_data(rig->request->track, model.field);
    bt::require(address >= 0 &&
                    std::cmp_less_equal(address, compiled->nhfielddata) &&
                    samples.size() <= static_cast<std::size_t>(
                                          compiled->nhfielddata - address),
                "the welded fixture has valid compiled road storage");
    const auto heightfield = std::views::counted(
        compiled->hfield_data, compiled->nhfielddata)
                                 .subspan(static_cast<std::size_t>(address),
                                          samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
        heightfield[i] = static_cast<float>(samples[i]);
    rig->config = std::unique_ptr<rcfg::RuntimeConfig>{
        new rcfg::RuntimeConfig(
            rcfg::project_runtime(model, *compiled, rig->request->track))};
    const auto image = init::capture_model_image(*compiled);
    const auto writers = std::unique_ptr<nativecfg::WriterConfig>{
        new nativecfg::WriterConfig(
            nativecfg::decode(rig->config->writer_config))};
    rig->stepper = std::make_unique<Stepper>(image.mjb, *writers);
    rig->physical =
        rcfg::make_physical_step(*rig->stepper, *rig->config);
    init::SetupContext context{.stepper = *rig->stepper,
                               .physical = *rig->physical,
                               .physics = physics};
    context.start_x_m = rig->request->start_x_m;
    context.ground_z_m =
        bike::geometry::compute_ground_z(model.bike) / 1000.;
    context.frame_body_id =
        bike::model::required_id(*compiled, mjOBJ_BODY, "frame");
    context.front_geom_id =
        bike::model::required_id(*compiled, mjOBJ_GEOM, "geom_front_contact");
    context.rear_geom_id = bike::model::required_id(
        *compiled, mjOBJ_GEOM, "geom_rear_contact");
    if (rig->physical->rider_present())
        context.rider_pose =
            ::rider::decode_spindle_pose(rig->config->geometry.pose,
                                         "welded.pose");
    auto solver =
        model.solver
            ? std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  *model.solver)
            : std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  model.bike);
    context.rear_travel_from_shock_mm =
        [solver = std::move(solver)](double stroke) {
            return solver->solve_state_from_shock_stroke(stroke).wheel_travel;
        };
    init::initialize_pose(context);
    init::EquilibriumResult result;
    result.residual_qacc = init::acceleration_residual(context);
    const auto settled_qpos = rig->stepper->qpos();
    result.fork_travel_mm =
        1000. * settled_qpos[static_cast<std::size_t>(
            rig->stepper->suspension().fork_qposadr())];
    result.shock_stroke_mm =
        1000. * settled_qpos[static_cast<std::size_t>(
            rig->stepper->suspension().shock_qposadr())];
    result.rear_travel_mm =
        context.rear_travel_from_shock_mm(result.shock_stroke_mm);
    result.root_z_m = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "root_z").qpos)];
    result.pitch_rad = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "root_pitch").qpos)];
    result.initial_crank_phase_rad = physics.drive.crank_phase_rad;
    result.settled_crank_phase_rad = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "crank_spin").qpos)];
    context.equilibrium = result;
    init::finalize_initial_state(context, *rig->config);
    context.initializing = false;
    context.completed = true;
    return rig;
}
// NOLINTEND(modernize-make-unique)

// env.step(RideControl()): no caller torque, motor assist unbounded,
// brakes released — the automatic seated-climb effort owns the crank task.
rcfg::RideControl ride_control() { return {}; }

// The oracle test reads samples[0].channels['rider_welds']
// ['crank_torque_nm']; natively the period-close channel is assembled from
// the interval's diagnostics wire, so the equivalent first-step read is
// raw.diagnostics['rider_welds']['crank_torque_nm'] — the same value
// accounting publishes (step.cpp's wire_settled_welds emission). It is NOT
// delivered_crank_torque_nm, which evaluate() deliberately republishes
// one step stale from the previous settle latch.
const rcfg::Wire *find_wire(const rcfg::WireObject &object,
                            std::string_view key) {
    for (const auto &[name, wire] : object)
        if (name == key) return &wire;
    return nullptr;
}

double weld_crank_torque_nm(const rcfg::RawStep &raw) {
    const rcfg::Wire *welds = find_wire(raw.diagnostics, "rider_welds");
    bt::require(welds != nullptr,
                "the interval diagnostics carry rider_welds");
    const auto *entries = std::get_if<rcfg::WireObject>(&welds->value);
    bt::require(entries != nullptr, "rider_welds is a wire object");
    const rcfg::Wire *torque = find_wire(*entries, "crank_torque_nm");
    bt::require(torque != nullptr,
                "welded pedals append crank_torque_nm");
    const auto *value = std::get_if<double>(&torque->value);
    bt::require(value != nullptr, "crank_torque_nm is a scalar");
    return *value;
}

// .6 s of advances like the oracle test; on each control tick the
// allocator's crank task coordinate must equal the solved weld torque of
// the interval's first step, within the oracle's relative bound. This is
// the F4 regression: the runtime must refresh the forward state between
// staging qfrc_applied and rider compute() (physical_runtime.py's
// mj_forward), otherwise ConstraintDynamics snapshots the previous
// interval's qfrc_actuator and the weld projection drifts by that
// actuator-force delta.
void planned_torque_is_the_solved_weld_torque() {
    auto rig = coupled_rig();
    rcfg::PhysicalStep &physical = *rig->physical;
    const auto &clock = physical.control_clock();
    const std::int64_t steps =
        std::llround(.6 / clock.timestep_s);
    int checked = 0;
    for (std::int64_t step = 0; step < steps; ++step) {
        const bool tick = clock.is_tick(physical.step());
        const rcfg::RawStep raw = physical.advance_physics(
            0., 0., std::nullopt, ride_control());
        if (!tick || !physical.rider_present()) continue;
        const auto &alloc =
            physical.rider_control().state().allocation_diagnostics.legacy;
        if (!alloc.has_value() || !alloc->predicted_crank_torque_nm ||
            alloc->effort_scale == 0.)
            continue;
        const double planned = *alloc->predicted_crank_torque_nm;
        const double solved = weld_crank_torque_nm(raw);
        ++checked;
        bt::require(
            std::abs(planned - solved) <
                1e-3 * std::max(1., std::abs(planned)),
            std::format("step {} t={:.4f}: planned crank torque {:.6f} "
                        "N.m vs first-step solved weld torque {:.6f} N.m",
                        static_cast<int>(step), rig->stepper->data()->time,
                        planned, solved));
    }
    bt::require(checked >= 20,
                "the run covers at least 20 control periods");
}
} // namespace

int main() {
    return bt::run(planned_torque_is_the_solved_weld_torque);
}
