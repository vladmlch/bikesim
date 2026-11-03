// configuration/legacy_battery.hpp — reading documents written before the
// [drive.battery] energy model was removed.
//
// The battery survives only as bike mass (mass budget, component
// provenance, the battery-pack geom). Recordings and initial-state files
// written earlier still carry a resolved `physics.drive.battery` section.
// Every shipped config disabled it, and a disabled battery never touched the
// dynamics: assist torque had no energy ceiling and the electrical channels
// were telemetry only. Such a section is inert and readers may drop it. An
// enabled recorded battery could have clamped assist torque once its energy
// ran out; that run is not reproducible and readers must reject it.
#pragma once

#include "fields.hpp"
#include "../io/value.hpp"

#include <string_view>

namespace bike::configuration::legacy {

inline constexpr std::string_view kBatteryKey = "battery";

// True when a recorded `drive.battery` section is the inert disabled one.
// Only `enabled` decides: with the battery disabled the remaining members
// (capacity, loss coefficients) fed nothing but telemetry.
[[nodiscard]] inline bool battery_inert(const io::Value& section) {
    const auto& object = io::as_object(section, "drive.battery");
    const auto* enabled = io::find_member(object, "enabled");
    return enabled != nullptr && !io::as_bool(*enabled, "drive.battery.enabled");
}

// The section exactly as pre-removal configs serialized a disabled battery
// with the shipped defaults, in declaration order. Legacy digests (the
// initial-state physical contract) are recomputed over it.
[[nodiscard]] inline io::Value disabled_battery_section() {
    const auto real = [](double value) {
        return io::Value{io::Number{detail::repr_double(value)}};
    };
    return io::Value{io::Value::Object{
        {"enabled", io::Value{false}},
        {"energy_j", real(1800000.)},
        {"copper_w_per_nm2", real(.02)},
        {"speed_w_per_rad_s2", real(0.)},
        {"idle_w", real(5.)}}};
}

} // namespace bike::configuration::legacy
