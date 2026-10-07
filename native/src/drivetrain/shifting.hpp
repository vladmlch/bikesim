#pragma once
#include "policy_config.hpp"
#include "../engaged.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace drivetrain {
    // The closed serialization domain of ShiftingSnapshot.direction — the
    // snapshot keeps the Python wire strings ("none"/"up"/"down"), while
    // the update path dispatches on this enum so the valid set is
    // compiler-visible.
    enum class ShiftDirection : std::uint8_t { none, up, down };

    // Closed-set parse for serialized shift directions: returns the
    // enumerator for "none"/"up"/"down" and nullopt for anything else.
    [[nodiscard]] constexpr std::optional<ShiftDirection>
    shift_direction(std::string_view label) noexcept {
        if (label == "none") {
            return ShiftDirection::none;
        }
        if (label == "up") {
            return ShiftDirection::up;
        }
        if (label == "down") {
            return ShiftDirection::down;
        }
        return std::nullopt;
    }

    // The wire string for a direction — the exhaustive switch is pinned to
    // the enum by -Wswitch-enum, with std::unreachable() instead of a
    // hidden default.
    [[nodiscard]] constexpr std::string_view
    shift_direction_name(ShiftDirection direction) noexcept {
        switch (direction) {
        case ShiftDirection::none:
            return "none";
        case ShiftDirection::up:
            return "up";
        case ShiftDirection::down:
            return "down";
        }
        std::unreachable();
    }

    // The closed set of upshift slip modes accepted by
    // ShiftingConfig.upshift_slip_mode ("legacy_signed" | "magnitude").
    // The config keeps the wire string; the shifter resolves it once at
    // construction so update() never re-parses per tick.
    enum class SlipMode : std::uint8_t { legacy_signed, magnitude };

    // Closed-set parse for slip modes: enumerator for a known label,
    // nullopt for anything else.
    [[nodiscard]] constexpr std::optional<SlipMode>
    slip_mode(std::string_view label) noexcept {
        if (label == "legacy_signed") {
            return SlipMode::legacy_signed;
        }
        if (label == "magnitude") {
            return SlipMode::magnitude;
        }
        return std::nullopt;
    }

    class CadenceShifter {
    public:
        CadenceShifter(GearingConfig gearing, ShiftingConfig config) : gearing_(gearing), config_(std::move(config)) {
            validate(gearing_);
            validate(config_);
            // validate() already closed the slip-mode domain; resolving once
            // here keeps per-tick updates on the enum. engaged() turns a
            // bypassed gate into the same logic_error any other violated
            // invariant raises.
            slip_mode_ = engaged(slip_mode(config_.upshift_slip_mode));
            if (config_.enabled && std::ranges::find(config_.cassette, gearing_.rear_teeth) == config_.cassette.end())
                throw std::invalid_argument("CadenceShifter.rear_teeth");
            reset();
        }

        // NOT noexcept: ShiftingSnapshot{} constructs its `direction` wire
        // string ("none"), so the reset can allocate — bugprone-exception-
        // escape correctly flags a noexcept claim here.
        void reset() {
            state_ = {};
            state_.rear_teeth = gearing_.rear_teeth;
            state_.from_teeth = state_.rear_teeth;
        }

        [[nodiscard]] bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                                  bool pedaling = true, bool braking = false, bool rear_in_contact = true,
                                  std::optional<double> rear_slip_mps = std::nullopt);

        [[nodiscard]] double gear_ratio() const noexcept {
            return static_cast<double>(gearing_.front_teeth) / state_.rear_teeth;
        }
        [[nodiscard]] double torque_factor() const noexcept {
            return state_.cut_remaining_s > 0. ? config_.torque_factor : 1.;
        }
        [[nodiscard]] const ShiftingSnapshot &state() const noexcept { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move (the direction string never re-allocates).
        void set_state(ShiftingSnapshot state);

    private:
        GearingConfig gearing_;
        ShiftingConfig config_;
        ShiftingSnapshot state_;
        // Resolved from config_.upshift_slip_mode at construction — engaged
        // for every valid config because validate() rejects anything else.
        SlipMode slip_mode_ = SlipMode::legacy_signed;
    };
}
