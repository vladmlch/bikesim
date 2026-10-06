#pragma once
#include <nanobind/nanobind.h>
#include "policy_config.hpp"
#include <span>
#include <string_view>

drivetrain::DrivePolicyConfig parse_drive_policy_config(const nanobind::dict &config, std::span<const std::string_view> extra_root_keys = {});

void bind_drive_policies(nanobind::module_ &module);
