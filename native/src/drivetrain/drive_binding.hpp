#pragma once
#include "../writers/drivetrain.hpp"
#include <nanobind/nanobind.h>
class Stepper;
drivetrain::DriveConfig parse_drive_config(const nanobind::dict &config);
void bind_drivetrain(nanobind::module_ &module, nanobind::class_<Stepper> &cls);
