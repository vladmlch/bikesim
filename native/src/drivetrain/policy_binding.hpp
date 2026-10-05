#pragma once
#include <nanobind/nanobind.h>
#include "policy_config.hpp"

drivetrain::DrivePolicyConfig parse_drive_policy_config(const nanobind::dict &config);

void bind_drive_policies(nanobind::module_ &module);
