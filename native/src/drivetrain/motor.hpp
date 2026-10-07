#pragma once
#include "policy_config.hpp"
#include "../engaged.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

namespace drivetrain {
    // The closed set of profiled assist modes — MotorProfile.mode_gains in
    // motor_profile.py carries exactly these keys. The config field stays an
    // owned string at the wire boundary because a profile-less controller
    // accepts arbitrary custom labels; the enum exists only for controllers
    // that carry a profile table, which resolve it once at construction.
    enum class ProfileMode : std::uint8_t { eco, tour, emtb, turbo };

    // Closed-set parse for profiled modes: returns the enumerator for one
    // of the four profile labels and nullopt for anything else. Only
    // profiled configs go through this parser — custom profile-less labels
    // never reach it and keep their open-string semantics.
    [[nodiscard]] constexpr std::optional<ProfileMode>
    profile_mode(std::string_view label) noexcept {
        if (label == "eco") {
            return ProfileMode::eco;
        }
        if (label == "tour") {
            return ProfileMode::tour;
        }
        if (label == "emtb") {
            return ProfileMode::emtb;
        }
        if (label == "turbo") {
            return ProfileMode::turbo;
        }
        return std::nullopt;
    }

    // assist_gain (motor_profile.py:79-89): fixed per-mode support, except
    // eMTB which ramps between emtb_low and emtb_high with rider torque.
    // The switch is exhaustive by construction — -Wswitch-enum pins every
    // enumerator to a branch, and std::unreachable() marks the post-switch
    // point instead of a hidden default.
    [[nodiscard]] constexpr double
    profile_gain(const MotorProfile &p, ProfileMode mode, double sensed) noexcept {
        switch (mode) {
        case ProfileMode::eco:
            return p.eco;
        case ProfileMode::tour:
            return p.tour;
        case ProfileMode::emtb: {
            const double fraction =
                    std::min(1., std::max(0., sensed) / p.emtb_full_gain_at_nm);
            return p.emtb_low + (p.emtb_high - p.emtb_low) * fraction;
        }
        case ProfileMode::turbo:
            return p.turbo;
        }
        std::unreachable();
    }

    class AssistController {
    public:
        explicit AssistController(AssistConfig config) : config_(std::move(config)) {
            validate(config_);
            // Resolve the closed profiled mode once at the boundary — the
            // config is owned so the enum can never desync, and step()
            // dispatches on the enum instead of re-parsing the wire label
            // per tick. validate() already rejected unknown profiled modes;
            // engaged() turns a bypassed gate into the same logic_error any
            // other violated invariant raises, never a silent wrong branch.
            if (config_.profile) {
                profile_mode_ = engaged(profile_mode(config_.mode));
            }
        }

        void reset() noexcept { state_ = {}; }

        [[nodiscard]] std::pair<double, double> ceiling(double shaft_rpm, double speed_mps) const;

        double step(double human_nm, double cadence_rpm, double speed_mps, bool braking, double dt,
                    std::optional<double> torque_request_nm = std::nullopt,
                    std::optional<double> shaft_rpm = std::nullopt);

        [[nodiscard]] const AssistSnapshot &state() const noexcept { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(AssistSnapshot state);

        // noexcept publication for staged candidates that were validated
        // while staging (PreparedSettlement::commit is noexcept — no
        // validation may run there).
        void publish(AssistSnapshot state) noexcept { state_ = state; }

    private:
        AssistConfig config_;
        AssistSnapshot state_;
        // Engaged exactly when config_.profile is — step() switches on this
        // closed enum, so the hot path never re-parses the wire label.
        std::optional<ProfileMode> profile_mode_;
    };

    class Battery {
    public:
        explicit Battery(double energy_j) : state_{
            .initial_energy_j = nonnegative(energy_j, "battery energy"), .energy_j = energy_j, .drawn_energy_j = 0.
        } {
        }

        void reset() noexcept {
            state_.energy_j = state_.initial_energy_j;
            state_.drawn_energy_j = 0.;
        }

        double draw(double requested_power_w, double dt);

        // Pure candidate form of draw(): identical arithmetic, but the debit
        // lands on a detached snapshot so a settlement can stage the draw
        // before any live state moves.
        [[nodiscard]] std::pair<double, BatterySnapshot>
        debit(double requested_power_w, double dt) const;

        [[nodiscard]] const BatterySnapshot &state() const noexcept { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(BatterySnapshot state);

        // noexcept publication for staged candidates that were validated
        // while staging (PreparedSettlement::commit is noexcept — no
        // validation may run there).
        void publish(BatterySnapshot state) noexcept { state_ = state; }

    private:
        BatterySnapshot state_;
    };

    [[nodiscard]] double motor_electrical_power(double torque, double omega, double a, double b, double idle, bool enabled);

    [[nodiscard]] double limit_torque_by_energy(double request, double omega, double a, double b, double idle, double budget_w);
}
