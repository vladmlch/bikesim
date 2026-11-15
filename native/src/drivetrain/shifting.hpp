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

    // Launch part 1, task 5 — the rider-owned request carried into
    // update(). owns=true hands shifting to the rider for that tick: the
    // band never selects and an engaged request moves exactly the
    // adjacent cog in its direction, sharing the automaton's admission
    // gate and checked commit. owns=false is the automatic path, and a
    // request without ownership is an inconsistency update() rejects.
    struct RiderShift {
        bool owns = false;
        std::optional<ShiftDirection> request = std::nullopt;
        std::optional<int> stationary_target_teeth = std::nullopt;
    };

    // The closed serialization domain of ShiftingSnapshot.rider_response
    // — the label every update() writes: "none" outside an owned request
    // (including an owned tick with no request), "accepted" on the shared
    // commit, "deferred" when admission or an in-flight cut declines the
    // request without judging the target, and "rejected" when the
    // cassette end, slip limit or landing veto declines the target.
    [[nodiscard]] constexpr bool
    rider_response_label(std::string_view label) noexcept {
        return label == "none" || label == "accepted" || label == "deferred" ||
               label == "rejected";
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

    // The rider-effort factor in force while a shift cut runs, keyed to the
    // direction of the shift in progress (ruling of 2026-10-03): a downshift
    // lands on a LARGER cog (easier gear), so the rider keeps
    // torque_factor_larger_cog of the effort; an upshift lands on a SMALLER
    // cog (harder gear) and keeps torque_factor_smaller_cog. Outside a cut —
    // or on a snapshot that carries no direction — the factor is 1.
    [[nodiscard]] inline double
    shift_torque_factor(const ShiftingConfig &config,
                        const ShiftingSnapshot &state) noexcept {
        if (state.cut_remaining_s <= 0.) {
            return 1.;
        }
        const auto direction = shift_direction(state.direction);
        if (direction == ShiftDirection::down) {
            return config.torque_factor_larger_cog;
        }
        if (direction == ShiftDirection::up) {
            return config.torque_factor_smaller_cog;
        }
        return 1.;
    }

    class CadenceShifter {
    public:
        CadenceShifter(GearingConfig gearing, ShiftingConfig config) : gearing_(gearing), config_(std::move(config)) {
            validate(gearing_);
            validate(config_);
            // ShiftingConfig.__post_init__ (physical_config.py) normalizes the
            // cassette to ascending order on every construction — including a
            // directly-constructed config that never sees the wire decoder or
            // post_init(). Mirror that ordering guarantee here or the
            // nearest-first scans land on the wrong cog.
            std::ranges::sort(config_.cassette);
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
                                  std::optional<double> rear_slip_mps = std::nullopt) {
            return update(cadence_rpm, required_cadence_rpm, dt, pedaling, braking,
                          rear_in_contact, rear_slip_mps, RiderShift{});
        }

        // The task-5 signature: rider stays defaulted-out on the old
        // positional call so existing callers keep the pure automaton.
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror shifting.py
        [[nodiscard]] bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                                  bool pedaling, bool braking, bool rear_in_contact,
                                  std::optional<double> rear_slip_mps, RiderShift rider);

        [[nodiscard]] double gear_ratio() const noexcept {
            return static_cast<double>(gearing_.front_teeth) / state_.rear_teeth;
        }
        [[nodiscard]] double torque_factor() const noexcept {
            return shift_torque_factor(config_, state_);
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
