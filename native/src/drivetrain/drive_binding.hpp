// drive_binding.hpp — the nanobind ADAPTER face for the drivetrain wires.
// The state wire's schema decode/encode is owned by the Python-free codec
// (drivetrain/state_codec.hpp, T08); this header declares only the
// nanobind-object adapters that keep the interpreter boundary's strictness
// (numpy gates, nb type checks). Nothing pure-core includes this header.
#pragma once
#include "../writers/drivetrain.hpp"
#include <nanobind/nanobind.h>
class Stepper;

drivetrain::DriveConfig parse_drive_config(const nanobind::dict &config);

// Schema-only decode of the flattened drive state dict — the same reader
// 'set_drive_state' uses. Pure: no writer or model mutation. The
// bike::io::Value twin is drivetrain::decode_drive_snapshot.
[[nodiscard]] drivetrain::DriveSnapshot
parse_drive_snapshot(nanobind::handle state, std::string_view path);

void bind_drivetrain(nanobind::module_ &module, nanobind::class_<Stepper> &cls);
