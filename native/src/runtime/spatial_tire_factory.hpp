#pragma once
#include "configuration/model_config.hpp"
#include "pneumatic3d/model.hpp"
#include "rider/steer_balance.hpp"
#include "runtime/config.hpp"
class Stepper;
namespace runtime {
// Cold identity resolution, shared by fresh setup and saved runtime. Measured
// qualification is independent from these deterministic content hashes.
void resolve_spatial_runtime_profiles(bike::configuration::SimulationPhysicsConfig&);
bike::io::Value project_spatial_tire_config(const bike::configuration::SimulationPhysicsConfig&);
bike::pneumatic3d::Pneumatic3dSpec decode_spatial_tire_config(const bike::io::Value&);
bike::rider::SteerProfile resolved_steer_profile(const bike::spatial::SteerConfig&);
void install_spatial_tire(Stepper&,const RuntimeConfig&);
}
