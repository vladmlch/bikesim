// contact_config.hpp — the typed, resolved rider-contact setup the
// RiderContactWriter consumes, plus the closed attachment discriminators.
//
// The wire section ('rider_contacts' in tools/native_schema.py) carries the
// ArticulatedConfig fields RiderContactApplier reads
// (sim/ride/rider_contacts.py) and the arm reach the applier resolved from
// the seated pose (arm_reach_m). Domains mirror
// ArticulatedConfig.__post_init__ (physics/physical_config.py): every scalar
// is finite and nonnegative, the structural lengths and stiffnesses are
// positive, and grip_pair_force_limit_n is either absent or positive.
//
// No nanobind/Python here: dict→typed conversion lives in the binding layer,
// which maps the wire strings through the constexpr parsers below.
#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>

#include "../validation.hpp"

namespace rider {
    // Closed attachment domains (ArticulatedConfig.__post_init__):
    // pedals 'flat'|'weld'|'spindle', saddle 'flat'|'weld'|'pin', grip
    // 'spring'|'connect'. Python normalizes the legacy 'weld' grip label to
    // 'connect' before projection, so it is never a wire value.
    enum class PedalAttachment : std::uint8_t { flat, weld, spindle };
    enum class SaddleAttachment : std::uint8_t { flat, weld, pin };
    enum class GripAttachment : std::uint8_t { spring, connect };

    // Closed-set parses: the enumerator for a declared wire label, nullopt
    // for anything else — same contract as drivetrain::slip_mode.
    [[nodiscard]] constexpr std::optional<PedalAttachment>
    pedal_attachment(std::string_view label) noexcept {
        if (label == "flat") return PedalAttachment::flat;
        if (label == "weld") return PedalAttachment::weld;
        if (label == "spindle") return PedalAttachment::spindle;
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<SaddleAttachment>
    saddle_attachment(std::string_view label) noexcept {
        if (label == "flat") return SaddleAttachment::flat;
        if (label == "weld") return SaddleAttachment::weld;
        if (label == "pin") return SaddleAttachment::pin;
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<GripAttachment>
    grip_attachment(std::string_view label) noexcept {
        if (label == "spring") return GripAttachment::spring;
        if (label == "connect") return GripAttachment::connect;
        return std::nullopt;
    }

    // Resolved construction-time setup; mutable contact state (the enabled
    // map, per-pad xi/tangent, grip xi/anchors, energies and latches reset()
    // establishes in rider_contacts.py:98-124) is snapshot state, not config.
    struct RiderContactsConfig {
        double arm_reach_m{};
        double saddle_patch_half_length_m{};
        double pedal_patch_half_length_m{};
        double support_pad_radius_m{};
        double support_k_n_m{};
        double support_c_ns_m{};
        double pedal_c_ns_m{};
        double support_tangent_k_n_m{};
        double support_mu{};
        double support_length_m{};
        double grip_k_n_m{};
        double grip_c_ns_m{};
        double grip_release_distance_m{};
        std::optional<double> grip_pair_force_limit_n;
        double grip_capture_distance_m{};
        double grip_capture_speed_mps{};
        PedalAttachment pedal_attachment = PedalAttachment::flat;
        SaddleAttachment saddle_attachment = SaddleAttachment::flat;
        GripAttachment grip_attachment = GripAttachment::spring;
    };

    // The ArticulatedConfig.__post_init__ domain, on the resolved subset the
    // contact applier reads: scalar() minimum=0 everywhere, positive for the
    // structural lengths/stiffnesses its second loop names, and positive for
    // the resolved reach — arm_reach_fraction is strictly inside (0, 1) of
    // positive arm segment lengths, so a nonpositive reach is a model defect.
    inline void validate(const RiderContactsConfig &c) {
        validation::positive(c.arm_reach_m, "config.rider_contacts.arm_reach_m");
        validation::nonnegative(c.saddle_patch_half_length_m,
                                "config.rider_contacts.saddle_patch_half_length_m");
        validation::nonnegative(c.pedal_patch_half_length_m,
                                "config.rider_contacts.pedal_patch_half_length_m");
        validation::positive(c.support_pad_radius_m,
                             "config.rider_contacts.support_pad_radius_m");
        validation::positive(c.support_k_n_m, "config.rider_contacts.support_k_n_m");
        validation::nonnegative(c.support_c_ns_m, "config.rider_contacts.support_c_ns_m");
        validation::nonnegative(c.pedal_c_ns_m, "config.rider_contacts.pedal_c_ns_m");
        validation::positive(c.support_tangent_k_n_m,
                             "config.rider_contacts.support_tangent_k_n_m");
        validation::nonnegative(c.support_mu, "config.rider_contacts.support_mu");
        validation::positive(c.support_length_m, "config.rider_contacts.support_length_m");
        validation::positive(c.grip_k_n_m, "config.rider_contacts.grip_k_n_m");
        validation::nonnegative(c.grip_c_ns_m, "config.rider_contacts.grip_c_ns_m");
        validation::positive(c.grip_release_distance_m,
                             "config.rider_contacts.grip_release_distance_m");
        if (c.grip_pair_force_limit_n)
            validation::positive(*c.grip_pair_force_limit_n,
                                 "config.rider_contacts.grip_pair_force_limit_n");
        validation::nonnegative(c.grip_capture_distance_m,
                                "config.rider_contacts.grip_capture_distance_m");
        validation::nonnegative(c.grip_capture_speed_mps,
                                "config.rider_contacts.grip_capture_speed_mps");
        // Enums cannot express absence, but a cast may smuggle a value
        // outside the closed set — membership is a runtime check.
        if (c.pedal_attachment != PedalAttachment::flat &&
            c.pedal_attachment != PedalAttachment::weld &&
            c.pedal_attachment != PedalAttachment::spindle)
            throw std::invalid_argument("config.rider_contacts.pedal_attachment");
        if (c.saddle_attachment != SaddleAttachment::flat &&
            c.saddle_attachment != SaddleAttachment::weld &&
            c.saddle_attachment != SaddleAttachment::pin)
            throw std::invalid_argument("config.rider_contacts.saddle_attachment");
        if (c.grip_attachment != GripAttachment::spring &&
            c.grip_attachment != GripAttachment::connect)
            throw std::invalid_argument("config.rider_contacts.grip_attachment");
    }
} // namespace rider
