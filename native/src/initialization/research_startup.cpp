#include "research_startup.hpp"
#include "context.hpp"
#include "../model/mapping.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/step.hpp"
#include "../runtime/research.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"
#include <algorithm>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
using namespace runtime::setupvalue;

Value compiled_physical_metadata(const mjModel& model) {
    Object contacts;
    const auto solref = std::views::counted(model.geom_solref, model.ngeom * mjNREF);
    const auto solimp = std::views::counted(model.geom_solimp, model.ngeom * mjNIMP);
    const auto friction = std::views::counted(model.geom_friction, model.ngeom * 3);
    const auto condim = std::views::counted(model.geom_condim, model.ngeom);
    const auto contype = std::views::counted(model.geom_contype, model.ngeom);
    const auto affinity = std::views::counted(model.geom_conaffinity, model.ngeom);
    const auto size = std::views::counted(model.geom_size, model.ngeom * 3);
    Object pedals;
    for (const auto side : {"front", "rear"}) {
        const auto index = static_cast<std::size_t>(bike::model::required_id(
            model, mjOBJ_GEOM, std::string("geom_") + side + "_contact"));
        contacts.emplace_back(side, Value{Object{
            {"solref", reals(solref.subspan(index * mjNREF, mjNREF))},
            {"solimp", reals(solimp.subspan(index * mjNIMP, mjNIMP))},
            {"friction", reals(friction.subspan(index * 3, 3))},
            {"condim", integer(condim[index])},
            {"native_enabled", Value{contype[index] != 0 || affinity[index] != 0}},
            {"units", text("MuJoCo solver parameters; not tire material stiffness or damping")}}});
        const int pedal = mj_name2id(&model, mjOBJ_GEOM, (std::string("geom_pedal_") + side).c_str());
        if (pedal >= 0) pedals.emplace_back(side, real(size[static_cast<std::size_t>(pedal) * 3]));
    }
    Array eq_ref, eq_imp;
    const auto refs = std::views::counted(model.eq_solref, model.neq * mjNREF);
    const auto imps = std::views::counted(model.eq_solimp, model.neq * mjNIMP);
    for (int i = 0; i < model.neq; ++i) {
        eq_ref.push_back(reals(refs.subspan(static_cast<std::size_t>(i) * mjNREF, mjNREF)));
        eq_imp.push_back(reals(imps.subspan(static_cast<std::size_t>(i) * mjNIMP, mjNIMP)));
    }
    return Value{Object{{"compiled_contact_solver", Value{std::move(contacts)}},
        {"compiled_equality_solref", Value{std::move(eq_ref)}},
        {"compiled_equality_solimp", Value{std::move(eq_imp)}},
        {"pedal_support_half_length_m", Value{std::move(pedals)}}}};
}
ResearchStartup prepare_research_startup(SetupContext& context) {
    if (!context.completed || !context.equilibrium || context.stepper.time() != 0.)
        throw std::invalid_argument("research startup requires completed t=0 equilibrium");
    auto& stepper = context.stepper;
    auto& physical = context.physical;
    if (physical.drive_mode() != "crank_effort" && physical.drive_mode() != "articulated_effort")
        throw std::invalid_argument("research startup requires a physical effort drive");
    auto const* data = stepper.data();
    const auto* model = stepper.model();
    // ResearchEnvironment._begin_episode: holding brakes do not enter the
    // policy episode. This is a non-advancing force evaluation and one forward.
    std::ranges::fill(std::views::counted(data->qacc_warmstart, model->nv), 0.);
    runtime::RideControl safe;
    safe.motor_torque_nm = 0.;
    safe.human_torque_nm = 0.;
    (void)physical.apply_forces(true, false, 0., 0., std::nullopt, safe);
    stepper.forward();
    physical.refresh_spatial_observation_cache();
    auto channels = physical.sensor_channels(stepper.qvel(), stepper.drive().diagnostics(true).to_map());
    if (context.physics.seated_climb.enabled) {
        auto signals = channels;
        signals.emplace_back("tires", runtime::Wire{physical.tire_channels(physical.snapshots_, stepper.qvel())});
        stepper.mutate([&] { physical.update_intent_signals(signals); });
    }
    runtime::PhysicalSampleData initial;
    initial.channels.emplace_back("sensors", runtime::Wire{std::move(channels)});
    ResearchStartup result;
    if(context.topology) result.observation_v2=runtime::raw_sensor_observation_v2(initial);
    else result.observation = runtime::raw_sensor_observation(initial);
    for (const auto& vertex : physical.terrain_vertices()) {
        result.geometry.x.push_back(vertex[0]);
        result.geometry.z.push_back(vertex[1]);
    }
    const auto radii = std::views::counted(model->geom_size, model->ngeom * 3);
    result.geometry.front_radius = radii[static_cast<std::size_t>(context.front_geom_id) * 3];
    result.geometry.rear_radius = radii[static_cast<std::size_t>(context.rear_geom_id) * 3];
    if(!context.topology) {
    const auto x = joint_address(context, "root_x"), pitch = joint_address(context, "root_pitch");
    result.geometry.root_x_qpos = static_cast<std::size_t>(x.qpos);
    result.geometry.root_x_dof = static_cast<std::size_t>(x.dof);
    result.geometry.root_pitch_qpos = static_cast<std::size_t>(pitch.qpos);
    result.geometry.root_pitch_dof = static_cast<std::size_t>(pitch.dof);
    result.position_m = std::views::counted(data->qpos, model->nq)[result.geometry.root_x_qpos];
    } else result.position_m=physical.longitudinal_position_m();
    result.compiled_metadata = compiled_physical_metadata(*model);
    if(context.terrain3d) {
        auto& metadata=std::get<Object>(result.compiled_metadata.data);
        metadata.emplace_back("physical_terrain_sha256",text(context.terrain3d->fingerprint_hex()));
        metadata.emplace_back("terrain_query_source",text("canonical_terrain3d"));
        if(context.render3d)metadata.emplace_back("terrain_render_resolution",Value{Object{
            {"nrow",integer(context.render3d->nrow)},{"ncol",integer(context.render3d->ncol)}}});
    }
    return result;
}
} // namespace bike::initialization
