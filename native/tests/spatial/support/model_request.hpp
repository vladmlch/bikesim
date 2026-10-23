#pragma once

#include "model/builder.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/source3d.hpp"
#include "support/migration_test.hpp"

#include <memory>
#include <string>
#include <utility>

namespace spatial_tests {

// Analytic input, independent of the renderer/generator under test. This
// ground covers the complete bicycle and is immutable for each model owner.
inline std::shared_ptr<const bike::terrain::Terrain3dSource> flat_model_ground() {
    bike::terrain::SurfaceSpec3d surface{.id=1,.name="analytic-model-ground",
        .mu_peak=1.,.mu_slide=.8,.slip_stiffness_per_load=12.,
        .stribeck_speed_mps=.1,.description="flat authored model fixture"};
    auto source=bike::terrain::ProfileExtrusion3d::create(
        {-4.,4.},{0.,0.},{1},{surface},2.,
        bike::io::Value{bike::io::Value::Object{{"fixture",bike::io::Value{std::string("flat-model-ground")}}}});
    bt::require(source.has_value(),"analytic model terrain is well formed");
    return std::make_shared<const bike::terrain::Terrain3dSource>(std::move(*source));
}

inline std::unique_ptr<bike::model::BuildRequest> model_request(bike::spatial::RootKind kind) {
    auto request=std::make_unique<bike::model::BuildRequest>();
    request->mode="ride";
    request->physics.emplace();
    request->physics->physics_mode="physical";
    request->physics->drive_mode="crank_effort";
    request->physics->tires.backend="pneumatic_3d";
    request->physics->drive.transmission_model="ideal_mid_drive";
    request->physics->drive.shifting.enabled=false;
    request->spatial.emplace();
    request->spatial->mode=bike::spatial::Mode::spatial3d;
    request->spatial->root=kind;
    request->spatial->steer.controller=bike::spatial::SteerController::off;
    request->spatial->steer.torque_cap_nm=12.25;
    request->terrain3d=flat_model_ground();
    request->field.nrow=9;
    request->field.ncol=17;
    return request;
}

} // namespace spatial_tests
