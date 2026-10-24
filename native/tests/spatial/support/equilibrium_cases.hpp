#pragma once
#include "startup_cases.hpp"
#include "initialization/context.hpp"
#include "initialization/capture.hpp"
#include "initialization/spatial_equilibrium.hpp"
#include "geometry/specs.hpp"
#include "kinematics/solver.hpp"
#include "rider/state_codec.hpp"
#include "runtime/spatial_tire_factory.hpp"
#include "stepper.hpp"

namespace spatial_tests {
// A cold, unsealed setup arena for ordered/unordered probe tests. It uses the
// identical projection and physical owner construction as SetupOwner.
struct EquilibriumArena {
    std::unique_ptr<bike::initialization::SetupRequest> request;
    runtime::RuntimeConfig config;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
    std::unique_ptr<bike::initialization::SetupContext> context;
    EquilibriumArena(bike::spatial::RootKind root,bike::spatial::TyreTier tier,bool articulated)
        :request(make_startup_request(root,tier,articulated,0.)) {
        namespace b=bike;
        b::configuration::post_init(*request->model.physics);
        runtime::resolve_spatial_runtime_profiles(*request->model.physics);
        request->model.spatial=request->model.physics->spatial;
        auto compiled=b::model::compile_mjcf(b::model::generate_mjcf(request->model),request->asset_root);
        b::model::populate_render_heightfield(*compiled,b::terrain::make_render_grid(*request->model.terrain3d,
            {static_cast<std::uint32_t>(request->model.field.nrow),static_cast<std::uint32_t>(request->model.field.ncol)}));
        config=runtime::project_runtime(request->model,*compiled,request->track,request->runtime);
        auto image=b::initialization::capture_model_image(*compiled);
        stepper=std::make_unique<Stepper>(image.mjb,nativecfg::decode(config.writer_config));
        physical=runtime::make_physical_step(*stepper,config);
        context=std::make_unique<b::initialization::SetupContext>(b::initialization::SetupContext{
            .stepper=*stepper,.physical=*physical,.physics=*request->model.physics});
        context->start_x_m=request->start_x_m;
        context->ground_z_m=b::geometry::compute_ground_z(request->model.bike)/1000.;
        context->frame_body_id=mj_name2id(stepper->model(),mjOBJ_BODY,"frame");
        context->front_geom_id=mj_name2id(stepper->model(),mjOBJ_GEOM,"geom_front_contact");
        context->rear_geom_id=mj_name2id(stepper->model(),mjOBJ_GEOM,"geom_rear_contact");
        context->terrain3d=request->model.terrain3d;
        context->topology=b::spatial::bind_topology(*stepper->model(),*request->model.spatial);
        context->model_sha256=image.sha256;
        if(physical->rider_present())context->rider_pose=::rider::decode_spatial_spindle_pose(config.geometry.pose,"fixture.pose");
        const auto solver=std::make_shared<b::kinematics::HorstLinkageSolver>(request->model.bike);
        context->rear_travel_from_shock_mm=[solver](double stroke){return solver->solve_state_from_shock_stroke(stroke).wheel_travel;};
        b::initialization::initialize_pose(*context);
    }
};
} // namespace spatial_tests
