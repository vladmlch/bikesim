#pragma once
#include "../writers/drivetrain.hpp"
#include <nanobind/nanobind.h>
class Stepper;

drivetrain::DriveConfig parse_drive_config(const nanobind::dict &config);

// Schema-only decode of the flattened drive state dict — the same reader
// 'set_drive_state' uses. Pure: no writer or model mutation.
[[nodiscard]] drivetrain::DriveSnapshot
parse_drive_snapshot(nanobind::handle state, std::string_view path);

void bind_drivetrain(nanobind::module_ &module, nanobind::class_<Stepper> &cls);
