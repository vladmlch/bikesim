#pragma once
#include "policy_config.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace drivetrain {
    // Closed pedaling-mode domain — the ordinal doubles as the rider_mode
    // telemetry label index (rider_mode_labels in writer_types.hpp lists
    // the wire strings in the same order).
    enum class PedalMode : std::uint8_t { disabled, pedaling, coasting };

    [[nodiscard]] constexpr std::string_view
    pedal_mode_name(PedalMode mode) noexcept {
        switch (mode) {
        case PedalMode::disabled: return "disabled";
        case PedalMode::pedaling: return "pedaling";
        case PedalMode::coasting: return "coasting";
        }
        return {};
    }

    [[nodiscard]] constexpr std::optional<PedalMode>
    pedal_mode(std::string_view name) noexcept {
        if (name == "disabled") return PedalMode::disabled;
        if (name == "pedaling") return PedalMode::pedaling;
        if (name == "coasting") return PedalMode::coasting;
        return std::nullopt;
    }

    // Closed coast-reason domain — `none` serializes as the empty string
    // (pedaling results carry no reason), matching the wire values
    // update() produced before the enum port.
    enum class CoastReason : std::uint8_t {
        none,
        braking,
        no_effort,
        cadence,
        disabled,
        reposition_requested,
        reposition_stall
    };

    [[nodiscard]] constexpr std::string_view
    coast_reason_name(CoastReason reason) noexcept {
        switch (reason) {
        case CoastReason::none: return "";
        case CoastReason::braking: return "braking";
        case CoastReason::no_effort: return "no_effort";
        case CoastReason::cadence: return "cadence";
        case CoastReason::disabled: return "disabled";
        case CoastReason::reposition_requested: return "reposition_requested";
        case CoastReason::reposition_stall: return "reposition_stall";
        }
        return {};
    }

    [[nodiscard]] constexpr std::optional<CoastReason>
    coast_reason(std::string_view name) noexcept {
        if (name == "") return CoastReason::none;
        if (name == "braking") return CoastReason::braking;
        if (name == "no_effort") return CoastReason::no_effort;
        if (name == "cadence") return CoastReason::cadence;
        if (name == "disabled") return CoastReason::disabled;
        if (name == "reposition_requested") return CoastReason::reposition_requested;
        if (name == "reposition_stall") return CoastReason::reposition_stall;
        return std::nullopt;
    }

    [[nodiscard]] constexpr bool is_reposition(CoastReason r) noexcept {
        return r == CoastReason::reposition_requested || r == CoastReason::reposition_stall;
    }
    // The longest label coast_reason_name can emit
    // ("reposition_requested", 20 chars) — the capacity every
    // coasting_reason open-lane pre-reserve must meet so no set_open or
    // snapshot copy reallocs inside the warm core. The assert walks the
    // whole underlying domain, so a longer future label cannot add
    // itself without tripping it.
    inline constexpr std::size_t kMaxCoastReasonLength = 20;
    static_assert([] {
        for (int i = 0; i < 256; ++i)
            if (coast_reason_name(static_cast<CoastReason>(i)).size() > kMaxCoastReasonLength)
                return false;
        return true;
    }(), "kMaxCoastReasonLength must bound every CoastReason label");
    // Crank qpos of the first power phase (Task 1 pins it; multiples of pi
    // from it are the two horizontal crank poses).
    inline constexpr double kPowerPhaseOffsetRad = 0.;

    // POD per-tick pedaling result — no string members, so copying into a
    // transaction bank is a plain move. String materialization happens at
    // the FFI boundary through the name functions above.
    struct PedalingState {
        PedalMode mode = PedalMode::disabled;
        CoastReason reason = CoastReason::disabled;
        double effort_nm{}, required_cadence_rpm{};
        std::optional<double> target_phase_rad;
        double target_rate_rad_s{};
    };

    class PedalingPolicy {
    public:
        explicit PedalingPolicy(PedalingConfig config) : config_(config) {
            validate(config_);
        }

        void reset() noexcept { state_ = {}; }

        [[nodiscard]] PedalingState update(double phase_rad, double rate_rad_s, double required_cadence_rpm,
                                           double effort_nm, double dt, bool enabled = true, bool braking = false,
                                           bool reposition_request = false,
                                           std::optional<double> shaft_rad_s = std::nullopt,
                                           std::optional<double> ceiling_nm = std::nullopt,
                                           double torque_factor = 1., bool cut_after_ceiling = false);

        [[nodiscard]] const PedalingSnapshot &state() const noexcept { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(PedalingSnapshot state);

    private:
        PedalingConfig config_;
        PedalingSnapshot state_;
    };

    [[nodiscard]] double human_crank_torque(double mean_nm, double phase_rad, double ripple = .35);
}
