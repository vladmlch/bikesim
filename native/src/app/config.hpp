#pragma once
#include "../configuration/suspension_spec.hpp"
#include "../configuration/rider_spec.hpp"
#include "../geometry/specs.hpp"
#include "../model/mass.hpp"

namespace bike {
// config.py BikeConfig: independently owned defaults for every nested
// component. The aggregate imposes no new cross-field validation rules.
struct BikeConfig {
    ::bike::geometry::FrameGeometrySpecs geometry;
    ::bike::geometry::SuspensionHardwareSpecs suspension;
    configuration::AirSpringSpecs air_spring;
    configuration::DamperClickConfig damper_clicks;
    model::BikeMassSpecs mass;
    configuration::RiderSpecs rider;
    configuration::DrivetrainSpecs drivetrain;
};
} // namespace bike
