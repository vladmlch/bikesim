// strictness_reference_support.hpp — the shared neutral-raw stand-ins the
// batch 3.23 ports consume, mirroring the oracle's import relationship:
// tests/reference/test_energy_ledger.py does
//   from test_monitor_strictness import runtime_for_raws, neutral_raw
// so both ports must run the identical fixture body. test_monitor_strictness
// keeps these helpers on its own file's namespace in the oracle; here they
// live in this header so the energy-ledger port sees the same statements.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "runtime/accounting.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"

namespace strictness_support {

constexpr double kDt = .0005;

// RawStep carries a matched ConstraintSnapshot: the native period close
// verifies snapshot identity against the interval (accounting.cpp:672-676)
// where the oracle's optional None simply skipped the check.
//
// Like the oracle's neutral_raw, components named in the numerical set also
// publish their force@qvel into numerical_constraint_power_w — the key is the
// component name itself, so a 'closure' component lands in the numerical
// bucket (split_constraint_powers only lifts 'equality:*' rows).
[[nodiscard]] inline runtime::RawStep neutral_raw(
    std::int64_t i, std::vector<std::string> violations = {},
    runtime::ForceComponents components = {},
    std::optional<std::vector<double>> qvel = std::nullopt) {
    runtime::RawStep raw;
    raw.interval_id = i;
    raw.time_s = static_cast<double>(i) * kDt;
    raw.end_time_s = static_cast<double>(i + 1) * kDt;
    raw.qpos = {0.};
    raw.qvel = qvel.has_value() ? *qvel : std::vector<double>{0.};
    // numerical_constraint_power_w={name:float(force@qvel) for name,force
    // in components.items() if name in (...)} — same insertion order.
    for (const auto &[name, force] : components) {
        if (name == "joint_limits" || name == "shock_solver_limit" ||
            name == "closure" || name == "ideal_transmission") {
            double power = 0.;
            for (std::size_t k = 0; k < force.size() && k < raw.qvel.size(); ++k)
                power += force[k] * raw.qvel[k];
            raw.numerical_constraint_power_w.emplace_back(name, power);
        }
    }
    raw.components = std::move(components);
    raw.qfrc_passive = {0.};
    raw.loss_step_j = 0.;
    // Python neutral_raw always measured (mechanical 0., elastic {});
    // the unmeasured legs clear the optionals below.
    raw.mechanical_energy_j = 0.;
    raw.elastic_energy_j = runtime::WireObject{};
    raw.attachment_errors = std::move(violations);
    raw.full = false;
    raw.constraint_snapshot.interval_start_s = raw.time_s;
    raw.constraint_snapshot.interval_end_s = raw.end_time_s;
    raw.constraint_snapshot.qvel_start = raw.qvel;
    runtime::WireObject front_tire{{"unloaded_radius_m", runtime::Wire(.35)},
                                   {"patches", runtime::Wire(runtime::WireArray{})}};
    raw.channels = {{"tires", runtime::Wire(runtime::WireObject{
                                {"front", runtime::Wire(front_tire)},
                                {"rear", runtime::Wire(front_tire)}})}};
    raw.diagnostics = {
        {"rider", runtime::Wire(runtime::WireObject{})},
        {"rider_welds", runtime::Wire(runtime::WireObject{})},
        {"endpoint_mass", runtime::Wire(nullptr)},
        {"rider_intent", runtime::Wire(nullptr)},
        {"rider_allocation", runtime::Wire(runtime::WireObject{})},
        {"rider_ik_saturation", runtime::Wire(runtime::WireObject{})},
        {"rider_support_targets", runtime::Wire(runtime::WireObject{})},
    };
    return raw;
}

// runtime_for_raws(): a detached PeriodAccounting — no controller, no
// contact writer, legacy retention, generation 1.
// strict=True/decimation=1 are the oracle defaults.
[[nodiscard]] inline std::unique_ptr<runtime::PeriodAccounting>
runtime_for_raws(bool strict = true, int decimation = 1) {
    runtime::AccountingConfig config;
    config.timestep_s = kDt;
    config.period_steps = 10;
    config.measurement_decimation = decimation;
    config.retention = runtime::RetentionMode::legacy;
    runtime::AccountingState initial;
    initial.energy.initial_energy_j = 0.;
    initial.energy.energy_scale_j = 1.;
    initial.monitor.strict = strict;
    return std::make_unique<runtime::PeriodAccounting>(
        config, initial, /*generation=*/1,
        /*controller=*/nullptr, /*contacts=*/nullptr);
}

} // namespace strictness_support
