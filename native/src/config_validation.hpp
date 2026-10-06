#pragma once
#include "config_types.hpp"
#include "validation.hpp"
#include <algorithm>

namespace nativecfg {
    inline bool nonblank(std::string_view text) {
        // Match Python str.strip's Unicode whitespace set at the UTF-8 boundary.
        constexpr std::array<std::string_view, 29> spaces{
            "\x09", "\x0a", "\x0b", "\x0c", "\x0d", "\x1c", "\x1d", "\x1e", "\x1f", " ",
            "\xc2\x85", "\xc2\xa0", "\xe1\x9a\x80", "\xe2\x80\x80", "\xe2\x80\x81",
            "\xe2\x80\x82", "\xe2\x80\x83", "\xe2\x80\x84", "\xe2\x80\x85", "\xe2\x80\x86",
            "\xe2\x80\x87", "\xe2\x80\x88", "\xe2\x80\x89", "\xe2\x80\x8a", "\xe2\x80\xa8",
            "\xe2\x80\xa9", "\xe2\x80\xaf", "\xe2\x81\x9f", "\xe3\x80\x80"
        };
        while (!text.empty()) {
            const auto found = std::ranges::find_if(spaces, [&](std::string_view space) { return text.starts_with(space); });
            if (found == spaces.end()) return true;
            text.remove_prefix(found->size());
        }
        return false;
    }
    inline void validate(const AirSpringSpec &s) {
        validation::positive(s.stanchion_inner_diam_mm, "config.suspension.air_spring.stanchion_inner_diam_mm");
        validation::positive(s.total_travel_mm, "config.suspension.air_spring.total_travel_mm");
        validation::positive(s.pos_chamber_length_mm, "config.suspension.air_spring.pos_chamber_length_mm");
        validation::positive(s.neg_chamber_length_mm, "config.suspension.air_spring.neg_chamber_length_mm");
        validation::nonnegative(s.token_volume_cm3, "config.suspension.air_spring.token_volume_cm3");
        if (s.max_tokens < 0) throw std::invalid_argument("config.suspension.air_spring.max_tokens");
        validation::positive(s.gamma, "config.suspension.air_spring.gamma");
        validation::positive(s.atm_pressure_pa, "config.suspension.air_spring.atm_pressure_pa");
    }

    inline void validate(const AirSpringConfig &s) {
        validate(s.specs);
        validation::nonnegative(s.gauge_pressure_psi, "config.suspension.air_spring.gauge_pressure_psi");
    }

    inline void validate(const DamperCore &s, std::string_view path = "config.suspension.damper") {
        if (s.max_hsc <= 0) throw std::invalid_argument(std::string(path) + ".max_hsc");
        if (s.max_lsc <= 0) throw std::invalid_argument(std::string(path) + ".max_lsc");
        if (s.max_reb <= 0) throw std::invalid_argument(std::string(path) + ".max_reb");
        validation::nonnegative(s.c_lsc_min, std::string(path) + ".c_lsc_min");
        validation::nonnegative(s.c_lsc_max, std::string(path) + ".c_lsc_max");
        validation::nonnegative(s.c_hsc_min, std::string(path) + ".c_hsc_min");
        validation::nonnegative(s.c_hsc_max, std::string(path) + ".c_hsc_max");
        validation::nonnegative(s.c_reb_min, std::string(path) + ".c_reb_min");
        validation::nonnegative(s.c_reb_max, std::string(path) + ".c_reb_max");
        if (s.c_lsc_min > s.c_lsc_max || s.c_hsc_min > s.c_hsc_max || s.c_reb_min > s.c_reb_max)
            throw std::invalid_argument(std::string(path) + ": unordered damping bounds");
        validation::positive(s.v_knee_comp, std::string(path) + ".v_knee_comp");
        validation::positive(s.v_knee_reb, std::string(path) + ".v_knee_reb");
    }

    inline void validate(const Charger3Config &s) {
        validate(s.core, "config.suspension.fork_damper");
        validation::positive(s.total_travel_mm, "config.suspension.fork_damper.total_travel_mm");
        validation::nonnegative(s.hbo_start_mm, "config.suspension.fork_damper.hbo_start_mm");
        validation::nonnegative(s.c_hbo_base, "config.suspension.fork_damper.c_hbo_base");
        if (!s.legacy_behavior && s.hbo_start_mm >= s.total_travel_mm)
            throw std::invalid_argument("config.suspension.fork_damper.hbo_start_mm");
    }

    inline void validate(const SuperDeluxeConfig &s) {
        validate(s.core, "config.suspension.shock_damper");
        validation::positive(s.total_stroke_mm, "config.suspension.shock_damper.total_stroke_mm");
        validation::nonnegative(s.hbo_start_mm, "config.suspension.shock_damper.hbo_start_mm");
        if (s.max_hbo <= 0) throw std::invalid_argument("config.suspension.shock_damper.max_hbo");
        validation::nonnegative(s.c_hbo_min, "config.suspension.shock_damper.c_hbo_min");
        validation::nonnegative(s.c_hbo_max, "config.suspension.shock_damper.c_hbo_max");
        validation::nonnegative(s.lockout_preload_n, "config.suspension.shock_damper.lockout_preload_n");
        validation::nonnegative(s.lockout_stiffness, "config.suspension.shock_damper.lockout_stiffness");
        if (s.c_hbo_min > s.c_hbo_max || (!s.legacy_behavior && s.hbo_start_mm >= s.total_stroke_mm))
            throw std::invalid_argument("config.suspension.shock_damper: invalid HBO bounds");
    }

    inline void validate(const CoilConfig &s) {
        validation::positive(s.rate_n_m, "config.suspension.coil.rate_n_m");
        validation::nonnegative(s.preload_mm, "config.suspension.coil.preload_mm");
        validation::positive(s.stroke_mm, "config.suspension.coil.stroke_mm");
        validation::positive(s.bumper_length_mm, "config.suspension.coil.bumper_length_mm");
        validation::positive(s.bumper_peak_n, "config.suspension.coil.bumper_peak_n");
        if (s.bumper_length_mm > s.stroke_mm) throw std::invalid_argument("config.suspension.coil.bumper_length_mm");
    }

    inline void validate(const EndStopConfig &s) {
        validation::nonnegative(s.stiffness_n_m, "config.suspension.end_stops.stiffness_n_m");
        validation::nonnegative(s.damping_n_s_m, "config.suspension.end_stops.damping_n_s_m");
    }

    inline void validate(const SuspensionConfig &s) {
        if (s.physics_mode != "legacy" && s.physics_mode != "physical")
            throw std::invalid_argument("config.suspension.physics_mode");
        validate(s.air_spring);
        validate(s.fork_damper);
        validate(s.shock_damper);
        validate(s.coil);
        validate(s.end_stops);
    }
}

namespace drivetrain {
    inline void validate(const GearingConfig &s) {
        if (s.front_teeth < 3 || s.rear_teeth < 3) throw std::invalid_argument("config.drive.gearing.teeth");
        validation::positive(s.chain_pitch_m, "config.drive.gearing.chain_pitch_m");
    }
    inline void validate(const PedalingConfig &s) {
        validation::positive(s.coast_above_rpm, "config.drive.pedaling.coast_above_rpm");
        validation::nonnegative(s.resume_below_rpm, "config.drive.pedaling.resume_below_rpm");
        validation::positive(s.stop_time_s, "config.drive.pedaling.stop_time_s");
        validation::nonnegative(s.coast_cadence_tau_s, "config.drive.pedaling.coast_cadence_tau_s");
        validation::nonnegative(s.mash_cadence_rpm, "config.drive.pedaling.mash_cadence_rpm");
        validation::nonnegative(s.mash_torque_nm, "config.drive.pedaling.mash_torque_nm");
        validation::nonnegative(s.effort_slew_nm_s, "config.drive.pedaling.effort_slew_nm_s");
        if (s.resume_below_rpm >= s.coast_above_rpm || (s.mash_torque_nm > 0. && s.mash_cadence_rpm <= 0.))
            throw std::invalid_argument("config.drive.pedaling.cadence_band");
    }
    inline void validate(const ShiftingConfig &s) {
        if (s.cassette.empty()) throw std::invalid_argument("config.drive.shifting.cassette");
        for (std::size_t i = 0; i < s.cassette.size(); ++i) {
            if (s.cassette[i] < 3 || std::ranges::count(s.cassette, s.cassette[i]) != 1)
                throw std::invalid_argument("config.drive.shifting.cassette: unique teeth required");
        }
        validation::positive(s.target_cadence_min_rpm, "config.drive.shifting.target_cadence_min_rpm");
        validation::positive(s.target_cadence_max_rpm, "config.drive.shifting.target_cadence_max_rpm");
        validation::nonnegative(s.shift_cooldown_s, "config.drive.shifting.shift_cooldown_s");
        validation::nonnegative(s.shift_cut_duration_s, "config.drive.shifting.shift_cut_duration_s");
        validation::nonnegative(s.torque_factor, "config.drive.shifting.torque_factor");
        validation::nonnegative(s.cadence_smoothing_tau_s, "config.drive.shifting.cadence_smoothing_tau_s");
        validation::nonnegative(s.upshift_slip_limit_mps, "config.drive.shifting.upshift_slip_limit_mps");
        if (s.target_cadence_max_rpm <= s.target_cadence_min_rpm || s.shift_cut_duration_s > s.shift_cooldown_s || s.torque_factor > 1.)
            throw std::invalid_argument("config.drive.shifting.timing_band");
        if (s.upshift_slip_mode != "legacy_signed" && s.upshift_slip_mode != "magnitude")
            throw std::invalid_argument("config.drive.shifting.upshift_slip_mode");
    }
    inline void validate(const MotorProfile &s) {
        validation::nonnegative(s.eco, "config.drive.assist.profile.mode_gains.eco");
        validation::nonnegative(s.tour, "config.drive.assist.profile.mode_gains.tour");
        validation::nonnegative(s.emtb_low, "config.drive.assist.profile.mode_gains.emtb.0");
        validation::nonnegative(s.emtb_high, "config.drive.assist.profile.mode_gains.emtb.1");
        validation::nonnegative(s.turbo, "config.drive.assist.profile.mode_gains.turbo");
        validation::positive(s.emtb_full_gain_at_nm, "config.drive.assist.profile.emtb_full_gain_at_nm");
        if (s.emtb_low > s.emtb_high) throw std::invalid_argument("config.drive.assist.profile.emtb_gains");
    }
    inline void validate(const AssistConfig &s) {
        validation::nonnegative(s.gain, "config.drive.assist.gain");
        validation::nonnegative(s.max_torque, "config.drive.assist.max_torque");
        validation::nonnegative(s.max_power, "config.drive.assist.max_power");
        validation::positive(s.tau, "config.drive.assist.tau");
        validation::positive(s.slew, "config.drive.assist.slew");
        validation::nonnegative(s.engage_torque_nm, "config.drive.assist.engage_torque_nm");
        validation::nonnegative(s.gate_min_crank_rad_s, "config.drive.assist.gate_min_crank_rad_s");
        validation::nonnegative(s.cutoff_mps, "config.drive.assist.cutoff_mps");
        validation::positive(s.taper_width_mps, "config.drive.assist.taper_width_mps");
        if (s.taper_width_mps > s.cutoff_mps) throw std::invalid_argument("config.drive.assist.taper_width_mps");
        if (s.profile) {
            validate(*s.profile);
            if (s.mode != "eco" && s.mode != "tour" && s.mode != "emtb" && s.mode != "turbo")
                throw std::invalid_argument("config.drive.assist.mode");
        }
        if (s.torque_curve) {
            if (s.torque_curve->size() < 2) throw std::invalid_argument("config.drive.assist.torque_curve");
            double previous = -1.;
            for (const auto &row: *s.torque_curve) {
                validation::nonnegative(row[0], "config.drive.assist.torque_curve.rpm");
                validation::nonnegative(row[1], "config.drive.assist.torque_curve.torque");
                if (row[0] <= previous) throw std::invalid_argument("config.drive.assist.torque_curve.rpm_order");
                previous = row[0];
            }
        }
    }
    inline void validate(const BatteryConfig &s) {
        validation::nonnegative(s.energy_j, "config.drive.battery.energy_j");
        validation::nonnegative(s.copper_w_per_nm2, "config.drive.battery.copper_w_per_nm2");
        validation::nonnegative(s.speed_w_per_rad_s2, "config.drive.battery.speed_w_per_rad_s2");
        validation::nonnegative(s.idle_w, "config.drive.battery.idle_w");
    }
    inline void validate(const DrivePolicyConfig &s) {
        validate(s.gearing); validate(s.pedaling); validate(s.shifting); validate(s.assist); validate(s.battery);
        validation::positive(s.hub_stiffness_nm_rad, "config.drive.hub_stiffness_nm_rad");
        validation::nonnegative(s.hub_damping_nm_s, "config.drive.hub_damping_nm_s");
        if (s.shifting.enabled && std::ranges::find(s.shifting.cassette, s.gearing.rear_teeth) == s.shifting.cassette.end())
            throw std::invalid_argument("config.drive.gearing.rear_teeth: enabled gear must belong to cassette");
    }
    inline void validate(const DriveConfig &s) {
        validate(s.policies);
        validation::nonnegative(s.human_torque_nm, "config.drive.human_torque_nm");
        validation::nonnegative(s.torque_ripple, "config.drive.torque_ripple");
        if (s.torque_ripple >= 1.) throw std::invalid_argument("config.drive.torque_ripple");
        validation::finite(s.crank_phase_rad, "config.drive.crank_phase_rad");
        validation::positive(s.chain_k_n_m, "config.drive.chain_k_n_m");
        validation::nonnegative(s.chain_c_ns_m, "config.drive.chain_c_ns_m");
        validation::nonnegative(s.bearing_c_nms_rad, "config.drive.bearing_c_nms_rad");
        validation::nonnegative(s.rotor_inertia_kgm2, "config.drive.rotor_inertia_kgm2");
        if (s.drive_mode != "crank_effort" && s.drive_mode != "articulated_effort" &&
            s.drive_mode != "coast" && s.drive_mode != "ideal_speed_control")
            throw std::invalid_argument("config.drive.drive_mode");
        if (s.transmission_model != "elastic_chain" && s.transmission_model != "ideal_mid_drive" &&
            s.transmission_model != "geometric_ideal_mid_drive")
            throw std::invalid_argument("config.drive.transmission_model");
        if ((s.motor_clutch && s.rotor_inertia_kgm2 > 0.) ||
            (s.transmission_model == "elastic_chain" && (s.motor_clutch || s.rotor_inertia_kgm2 > 0. || s.policies.shifting.enabled)))
            throw std::invalid_argument("config.drive.transmission_model: incompatible clutch, rotor or shifting");
    }
}

namespace nativecfg {
    inline void validate(const BrakeConfig &s) {
        validation::nonnegative(s.torque_ceiling_nm, "config.brake.torque_ceiling_nm");
        validation::positive(s.taper_radps, "config.brake.taper_radps");
    }
    inline void validate(const CruiseConfig &s) {
        validation::positive(s.kp_nm_per_mps, "config.cruise.kp_nm_per_mps");
        validation::positive(s.ki_nm_per_mps_s, "config.cruise.ki_nm_per_mps_s");
        validation::positive(s.torque_ceiling_nm, "config.cruise.torque_ceiling_nm");
        validation::finite(s.target_speed_kmh, "config.cruise.target_speed_kmh");
        if (s.target_speed_kmh < 15. || s.target_speed_kmh > 45.) throw std::invalid_argument("config.cruise.target_speed_kmh");
    }
    inline void validate(const RiderPathConfig &s, std::string_view path = "config.rider_forces.paths") {
        validation::nonnegative(s.stiffness_n_m, std::string(path) + ".stiffness_n_m");
        validation::nonnegative(s.damping_ns_m, std::string(path) + ".damping_ns_m");
        validation::nonnegative(s.preload_deflection_m, std::string(path) + ".preload_deflection_m");
        validation::finite(s.offset_m, std::string(path) + ".offset_m");
    }
    inline void validate(const RiderForcesConfig &s) {
        for (std::size_t i = 0; i < s.paths.size(); ++i)
            validate(s.paths[i], "config.rider_forces.paths." + std::to_string(i));
    }
    inline void validate(const ResistanceConfig &s) {
        validation::nonnegative(s.crr, "config.resistance.crr");
        validation::nonnegative(s.rho_kg_m3, "config.resistance.rho_kg_m3");
        validation::nonnegative(s.cda_m2, "config.resistance.cda_m2");
        validation::positive(s.rolling_taper_rad_s, "config.resistance.rolling_taper_rad_s");
        for (const double value: s.wind_world_mps) validation::finite(value, "config.resistance.wind_world_mps");
        for (const double value: s.point_body_m) validation::finite(value, "config.resistance.point_body_m");
        if (s.wind_world_mps[1] != 0. || s.point_body_m[1] != 0.) throw std::invalid_argument("config.resistance: vectors must be planar");
    }
    inline void validate(const TireMaterial &s, std::string_view path = "config.tire.material") {
        validation::positive(s.radial_k_n_m, std::string(path) + ".radial_k_n_m");
        validation::nonnegative(s.radial_c_ns_m, std::string(path) + ".radial_c_ns_m");
        validation::nonnegative(s.pressure_pa_gauge, std::string(path) + ".pressure_pa_gauge");
        validation::nonnegative(s.valid_load_range_n[0], std::string(path) + ".valid_load_range_n");
        validation::positive(s.valid_load_range_n[1], std::string(path) + ".valid_load_range_n");
        if (s.valid_load_range_n[0] >= s.valid_load_range_n[1])
            throw std::invalid_argument(std::string(path) + ".valid_load_range_n");
        if (!nonblank(s.provenance)) throw std::invalid_argument(std::string(path) + ".provenance");
    }
    inline void validate(const TireParams &s, std::string_view path = "config.tire") {
        validate(s.material, std::string(path) + ".material");
        validation::positive(s.tangent_k_n_m, std::string(path) + ".tangent_k_n_m");
        validation::nonnegative(s.mu, std::string(path) + ".mu");
        validation::positive(s.relaxation_length_m, std::string(path) + ".relaxation_length_m");
    }
    inline void validate(const SurfaceSpec &s, std::string_view path = "config.tire.surface") {
        validation::positive(s.mu_peak, std::string(path) + ".mu_peak");
        validation::positive(s.mu_slide, std::string(path) + ".mu_slide");
        validation::positive(s.slip_stiffness_per_load, std::string(path) + ".slip_stiffness_per_load");
        validation::positive(s.stribeck_speed_mps, std::string(path) + ".stribeck_speed_mps");
        if (s.mu_slide > s.mu_peak) throw std::invalid_argument(std::string(path) + ".mu_slide");
    }
    inline void validate(const TireConfig &s) {
        validate(s.front, "config.tire.front"); validate(s.rear, "config.tire.rear");
        validation::nonnegative(s.significant_delta_m, "config.tire.significant_delta_m");
        validation::nonnegative(s.significance_fraction, "config.tire.significance_fraction");
        validation::positive(s.distinct_normal_deg, "config.tire.distinct_normal_deg");
        if (s.significance_fraction > 1. || s.distinct_normal_deg >= 180.) throw std::invalid_argument("config.tire.support_thresholds");
        if (s.backend != "compliant_2d" || (s.surface_mode != "track" && s.surface_mode != "configured"))
            throw std::invalid_argument("config.tire.backend_or_surface_mode");
        if (s.surface_mode == "track" && !s.surface_map) throw std::invalid_argument("config.tire.surface_map");
        if (s.surface_map) {
            validate(s.surface_map->surface, "config.tire.surface_map.surface");
            for (std::size_t i = 0; i < s.surface_map->sections.size(); ++i) {
                const auto &section = s.surface_map->sections[i];
                const std::string path = "config.tire.surface_map.sections." + std::to_string(i);
                validation::nonnegative(section.start_m, path + ".start_m");
                validation::positive(section.end_m, path + ".end_m");
                if (section.start_m >= section.end_m) throw std::invalid_argument(path + ": unordered interval");
                validate(section.surface, path + ".surface");
            }
            for (std::size_t i = 0; i < s.surface_map->sections.size(); ++i)
                for (std::size_t j = i + 1; j < s.surface_map->sections.size(); ++j)
                    if (std::max(s.surface_map->sections[i].start_m, s.surface_map->sections[j].start_m) <
                        std::min(s.surface_map->sections[i].end_m, s.surface_map->sections[j].end_m))
                        throw std::invalid_argument("config.tire.surface_map: overlapping sections");
        }
    }
}
