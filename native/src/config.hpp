// config.hpp — the config bridge: nanobind dict → typed writer configs.
//
// `tools.native_config.project(env)` emits {'schema': 2, '<writer>': {...}}
// carrying ONLY the fields the ported writers read. An empty dict leaves
// every writer disabled (the Stepper(path) contract stays intact). Every
// required key that is missing raises std::invalid_argument naming the
// key — nanobind's default exception translator surfaces it as ValueError.
// Readers validate exact keys, original scalar/sequence types and finiteness.
// Physical invariants also remain enforced by each typed writer constructor.
// Schema 1 migrates resolved fork metadata without reconstructing defaults.
#pragma once

#include <algorithm>
#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include "diag.hpp"
#include "config_types.hpp"
#include "binding_readers.hpp"
#include "drivetrain/drive_binding.hpp"

namespace nb = nanobind;

namespace nativecfg {
    // --- dict readers ----------------------------------------------------------

    namespace detail {
        inline void section_keys(const nb::dict &d, std::string_view name, std::string_view public_path = {}) {
            if (name == "config") {
                constexpr auto required = wire::keys("schema");
                constexpr auto optional = wire::keys("drive", "cruise", "suspension", "brake", "resistance", "tire", "rider_forces", "rider_contacts");
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension") {
                constexpr auto required = wire::keys("physics_mode", "joints", "air_spring", "fork_damper", "shock_damper", "coil", "end_stops");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.joints") {
                constexpr auto required = wire::keys("fork", "shock");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.air_spring") {
                constexpr auto required = wire::keys("stanchion_inner_diam_mm", "total_travel_mm", "pos_chamber_length_mm", "neg_chamber_length_mm", "token_volume_cm3", "max_tokens", "gamma", "atm_pressure_pa", "num_tokens", "gauge_pressure_psi");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.fork_damper") {
                constexpr auto required = wire::keys("max_hsc", "max_lsc", "max_reb", "hsc_clicks", "lsc_clicks", "rebound_clicks", "c_lsc_min", "c_lsc_max", "c_hsc_min", "c_hsc_max", "c_reb_min", "c_reb_max", "v_knee_comp", "v_knee_reb", "total_travel_mm", "hbo_start_mm", "c_hbo_base");
                constexpr auto optional = wire::keys("legacy_behavior");
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.shock_damper") {
                constexpr auto required = wire::keys("max_hsc", "max_lsc", "max_reb", "hsc_clicks", "lsc_clicks", "rebound_clicks", "c_lsc_min", "c_lsc_max", "c_hsc_min", "c_hsc_max", "c_reb_min", "c_reb_max", "v_knee_comp", "v_knee_reb", "total_stroke_mm", "max_hbo", "hbo_clicks", "lockout_firm", "legacy_behavior", "hbo_start_mm", "c_hbo_min", "c_hbo_max", "lockout_preload_n", "lockout_stiffness");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.coil") {
                constexpr auto required = wire::keys("rate_n_m", "preload_mm", "stroke_mm", "bumper_length_mm", "bumper_peak_n", "legacy_behavior");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "suspension.end_stops") {
                constexpr auto required = wire::keys("stiffness_n_m", "damping_n_s_m");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "brake") {
                constexpr auto required = wire::keys("torque_ceiling_nm", "taper_radps");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "cruise") {
                constexpr auto required = wire::keys("target_speed_kmh", "kp_nm_per_mps", "ki_nm_per_mps_s", "torque_ceiling_nm");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "resistance") {
                constexpr auto required = wire::keys("crr", "rolling_taper_rad_s", "rho_kg_m3", "cda_m2", "wind_world_mps", "point_body_m", "bodies");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "resistance.bodies") {
                constexpr auto required = wire::keys("frame", "front_wheel", "rear_wheel");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "tire") {
                constexpr auto required = wire::keys("backend", "surface_mode", "front", "rear", "significant_delta_m", "significance_fraction", "distinct_normal_deg");
                constexpr auto optional = wire::keys("surface_map");
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "tire.front" || name == "tire.rear") {
                constexpr auto required = wire::keys("material", "tangent_k_n_m", "mu", "relaxation_length_m");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "tire.front.material" || name == "tire.rear.material") {
                constexpr auto required = wire::keys("radial_k_n_m", "radial_c_ns_m", "pressure_pa_gauge", "provenance", "valid_load_range_n");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name.ends_with(".surface")) {
                constexpr auto required = wire::keys("name", "mu_peak", "mu_slide", "slip_stiffness_per_load", "stribeck_speed_mps");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "tire.surface_map") {
                constexpr auto required = wire::keys("surface", "sections");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "tire.surface_map.sections") {
                constexpr auto required = wire::keys("start_m", "end_m", "surface");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "rider_forces") {
                constexpr auto required = wire::keys("paths");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "rider_forces.paths") {
                constexpr auto required = wire::keys("joint", "stiffness_n_m", "damping_ns_m", "preload_deflection_m", "offset_m", "unilateral");
                constexpr auto optional = wire::keys();
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
            if (name == "rider_contacts") {
                constexpr auto required = wire::keys("arm_reach_m", "saddle_patch_half_length_m", "pedal_patch_half_length_m", "support_pad_radius_m", "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m", "support_tangent_k_n_m", "support_mu", "support_length_m", "grip_k_n_m", "grip_c_ns_m", "grip_release_distance_m", "grip_capture_distance_m", "grip_capture_speed_mps", "pedal_attachment", "saddle_attachment", "grip_attachment");
                constexpr auto optional = wire::keys("grip_pair_force_limit_n");
                wire::exact_keys(d, required, optional, public_path.empty() ? (name == "config" ? "config" : "config." + std::string(name)) : std::string(public_path));
                return;
            }
        }

        inline nb::object req(const nb::dict &d, const char *section,
                              const char *key) {
            if (!d.contains(key))
                throw std::invalid_argument(
                    "native config: missing key '" + std::string(section) + "." +
                    key + "'");
            return d[key];
        }

        inline nb::dict req_dict(const nb::dict &d, const char *section,
                                 const char *key) {
            nb::object const v = req(d, section, key);
            if (!nb::isinstance<nb::dict>(v))
                throw std::invalid_argument(
                    "native config: '" + std::string(section) + "." + key +
                    "' must be a dict");
            const auto result = nb::borrow<nb::dict>(v);
            const std::string name = std::string(section) == "config" ? key : std::string(section) + "." + key;
            section_keys(result, name);
            return result;
        }

        inline std::string path(const char *s, const char *k) {
            return std::string(s).starts_with("config") ? std::string(s) + "." + k : "config." + std::string(s) + "." + k;
        }

        inline double req_f64(const nb::dict &d, const char *s, const char *k) {
            return wire::finite_real(req(d, s, k), path(s, k));
        }
        inline int req_int(const nb::dict &d, const char *s, const char *k) {
            return wire::integer32(req(d, s, k), path(s, k));
        }
        inline bool req_bool(const nb::dict &d, const char *s, const char *k) {
            return wire::boolean(req(d, s, k), path(s, k));
        }
        inline std::string req_str(const nb::dict &d, const char *s, const char *k) {
            return wire::string(req(d, s, k), path(s, k));
        }
        inline std::array<double, 3> req_vec3(const nb::dict &d, const char *s, const char *k) {
            return wire::fixed<3>(req(d, s, k), path(s, k));
        }
        inline std::array<double, 2> req_vec2(const nb::dict &d, const char *s, const char *k) {
            return wire::fixed<2>(req(d, s, k), path(s, k));
        }

        // Returns the parsed struct by value deliberately; -Wlarge-by-value-copy
        // exists to flag by-value *parameters*, and clang gives no param/return split.
        NATIVE_DIAG_PUSH
        NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
        inline DamperCore damper_core(const nb::dict &d, const char *s) {
            DamperCore c{};
            c.max_hsc = req_int(d, s, "max_hsc");
            c.max_lsc = req_int(d, s, "max_lsc");
            c.max_reb = req_int(d, s, "max_reb");
            // BaseDamper.__init__ clamps clicks into range; project() emits the
            // resolved attributes, so this is a no-op for well-formed input and
            // keeps hand-built dicts on the same code path.
            c.hsc_clicks = std::max(0, std::min(c.max_hsc, req_int(d, s, "hsc_clicks")));
            c.lsc_clicks = std::max(0, std::min(c.max_lsc, req_int(d, s, "lsc_clicks")));
            c.rebound_clicks =
                    std::max(0, std::min(c.max_reb, req_int(d, s, "rebound_clicks")));
            c.c_lsc_min = req_f64(d, s, "c_lsc_min");
            c.c_lsc_max = req_f64(d, s, "c_lsc_max");
            c.c_hsc_min = req_f64(d, s, "c_hsc_min");
            c.c_hsc_max = req_f64(d, s, "c_hsc_max");
            c.c_reb_min = req_f64(d, s, "c_reb_min");
            c.c_reb_max = req_f64(d, s, "c_reb_max");
            c.v_knee_comp = req_f64(d, s, "v_knee_comp");
            c.v_knee_reb = req_f64(d, s, "v_knee_reb");
            return c;
        }

        NATIVE_DIAG_POP
    } // namespace detail

    inline SuspensionConfig suspension_from_dict(const nb::dict &top, int schema) {
        const char *s = "suspension";
        const nb::dict d = detail::req_dict(top, "config", s);
        SuspensionConfig c;
        c.physics_mode = detail::req_str(d, s, "physics_mode");
        if (c.physics_mode != "legacy" && c.physics_mode != "physical")
            throw std::invalid_argument(
                "config.suspension.physics_mode must be 'legacy' or "
                "'physical'");
        {
            const nb::dict j = detail::req_dict(d, s, "joints");
            c.fork_joint = detail::req_str(j, "suspension.joints", "fork");
            c.shock_joint = detail::req_str(j, "suspension.joints", "shock");
        }
        {
            const nb::dict a = detail::req_dict(d, s, "air_spring");
            const char *sa = "suspension.air_spring";
            c.air_spring.specs = {
                .stanchion_inner_diam_mm = detail::req_f64(a, sa, "stanchion_inner_diam_mm"),
                .total_travel_mm = detail::req_f64(a, sa, "total_travel_mm"),
                .pos_chamber_length_mm = detail::req_f64(a, sa, "pos_chamber_length_mm"),
                .neg_chamber_length_mm = detail::req_f64(a, sa, "neg_chamber_length_mm"),
                .token_volume_cm3 = detail::req_f64(a, sa, "token_volume_cm3"),
                .max_tokens = detail::req_int(a, sa, "max_tokens"),
                .gamma = detail::req_f64(a, sa, "gamma"),
                .atm_pressure_pa = detail::req_f64(a, sa, "atm_pressure_pa"),
            };
            // ForkAirSpring.__init__ clamps num_tokens into [0, max_tokens].
            c.air_spring.num_tokens = std::max(
                0, std::min(c.air_spring.specs.max_tokens,
                            detail::req_int(a, sa, "num_tokens")));
            c.air_spring.gauge_pressure_psi =
                    detail::req_f64(a, sa, "gauge_pressure_psi");
        }
        {
            const nb::dict f = detail::req_dict(d, s, "fork_damper");
            const char *sf = "suspension.fork_damper";
            c.fork_damper.core = detail::damper_core(f, sf);
            c.fork_damper.total_travel_mm = detail::req_f64(f, sf, "total_travel_mm");
            c.fork_damper.hbo_start_mm = detail::req_f64(f, sf, "hbo_start_mm");
            c.fork_damper.c_hbo_base = detail::req_f64(f, sf, "c_hbo_base");
            if (schema == 1 && f.contains("legacy_behavior"))
                wire::invalid("config.suspension.fork_damper.legacy_behavior", "unknown schema-1 key");
            // Schema 1 retains the historical resolved branch; never reconstruct defaults.
            c.fork_damper.legacy_behavior = schema == 1
                ? c.fork_damper.hbo_start_mm == 160.0
                : detail::req_bool(f, sf, "legacy_behavior");
        }
        {
            const nb::dict h = detail::req_dict(d, s, "shock_damper");
            const char *sh = "suspension.shock_damper";
            c.shock_damper.core = detail::damper_core(h, sh);
            c.shock_damper.total_stroke_mm = detail::req_f64(h, sh, "total_stroke_mm");
            c.shock_damper.max_hbo = detail::req_int(h, sh, "max_hbo");
            c.shock_damper.hbo_clicks = std::max(
                0, std::min(c.shock_damper.max_hbo,
                            detail::req_int(h, sh, "hbo_clicks")));
            c.shock_damper.lockout_firm = detail::req_bool(h, sh, "lockout_firm");
            c.shock_damper.legacy_behavior =
                    detail::req_bool(h, sh, "legacy_behavior");
            c.shock_damper.hbo_start_mm = detail::req_f64(h, sh, "hbo_start_mm");
            c.shock_damper.c_hbo_min = detail::req_f64(h, sh, "c_hbo_min");
            c.shock_damper.c_hbo_max = detail::req_f64(h, sh, "c_hbo_max");
            c.shock_damper.lockout_preload_n =
                    detail::req_f64(h, sh, "lockout_preload_n");
            c.shock_damper.lockout_stiffness =
                    detail::req_f64(h, sh, "lockout_stiffness");
        }
        {
            const nb::dict k = detail::req_dict(d, s, "coil");
            const char *sk = "suspension.coil";
            c.coil = {
                .rate_n_m = detail::req_f64(k, sk, "rate_n_m"),
                .preload_mm = detail::req_f64(k, sk, "preload_mm"),
                .stroke_mm = detail::req_f64(k, sk, "stroke_mm"),
                .bumper_length_mm = detail::req_f64(k, sk, "bumper_length_mm"),
                .bumper_peak_n = detail::req_f64(k, sk, "bumper_peak_n"),
                .legacy_behavior = detail::req_bool(k, sk, "legacy_behavior"),
            };
        }
        {
            const nb::dict e = detail::req_dict(d, s, "end_stops");
            const char *se = "suspension.end_stops";
            c.end_stops = {
                .stiffness_n_m = detail::req_f64(e, se, "stiffness_n_m"),
                .damping_n_s_m = detail::req_f64(e, se, "damping_n_s_m"),
            };
        }
        return c;
    }

    inline BrakeConfig brake_from_dict(const nb::dict &top) {
        const char *s = "brake";
        const nb::dict d = detail::req_dict(top, "config", s);
        BrakeConfig c{};
        c.torque_ceiling_nm = detail::req_f64(d, s, "torque_ceiling_nm");
        c.taper_radps = detail::req_f64(d, s, "taper_radps");
        return c;
    }

    inline ResistanceConfig resistance_from_dict(const nb::dict &top) {
        const char *s = "resistance";
        const nb::dict d = detail::req_dict(top, "config", s);
        ResistanceConfig c;
        c.crr = detail::req_f64(d, s, "crr");
        c.rolling_taper_rad_s = detail::req_f64(d, s, "rolling_taper_rad_s");
        c.rho_kg_m3 = detail::req_f64(d, s, "rho_kg_m3");
        c.cda_m2 = detail::req_f64(d, s, "cda_m2");
        c.wind_world_mps = detail::req_vec3(d, s, "wind_world_mps");
        c.point_body_m = detail::req_vec3(d, s, "point_body_m");
        const nb::dict b = detail::req_dict(d, s, "bodies");
        const char *sb = "resistance.bodies";
        c.frame_body = detail::req_str(b, sb, "frame");
        c.front_wheel_body = detail::req_str(b, sb, "front_wheel");
        c.rear_wheel_body = detail::req_str(b, sb, "rear_wheel");
        return c;
    }

    inline SurfaceSpec surface_spec_from_dict(const nb::dict &d, const char *s) {
        detail::section_keys(d, s);
        SurfaceSpec c;
        c.name = detail::req_str(d, s, "name");
        c.mu_peak = detail::req_f64(d, s, "mu_peak");
        c.mu_slide = detail::req_f64(d, s, "mu_slide");
        c.slip_stiffness_per_load =
                detail::req_f64(d, s, "slip_stiffness_per_load");
        c.stribeck_speed_mps = detail::req_f64(d, s, "stribeck_speed_mps");
        return c;
    }

    inline TireParams tire_params_from_dict(const nb::dict &d, const char *s) {
        detail::section_keys(d, s);
        TireParams p;
        {
            const nb::dict m = detail::req_dict(d, s, "material");
            const std::string sm_path = std::string(s) + ".material";
            const char *sm = sm_path.c_str();
            p.material = {
                .radial_k_n_m = detail::req_f64(m, sm, "radial_k_n_m"),
                .radial_c_ns_m = detail::req_f64(m, sm, "radial_c_ns_m"),
                .pressure_pa_gauge = detail::req_f64(m, sm, "pressure_pa_gauge"),
                .provenance = detail::req_str(m, sm, "provenance"),
                .valid_load_range_n = detail::req_vec2(m, sm, "valid_load_range_n"),
            };
        }
        p.tangent_k_n_m = detail::req_f64(d, s, "tangent_k_n_m");
        p.mu = detail::req_f64(d, s, "mu");
        p.relaxation_length_m = detail::req_f64(d, s, "relaxation_length_m");
        return p;
    }

    inline TireConfig tire_from_dict(const nb::dict &top) {
        const char *s = "tire";
        const nb::dict d = detail::req_dict(top, "config", s);
        TireConfig c;
        c.backend = detail::req_str(d, s, "backend");
        // Only compliant_2d is ported; the config layer is the honest place to
        // say so (project() never emits another backend).
        if (c.backend != "compliant_2d")
            throw std::invalid_argument(
                "config.tire.backend must be 'compliant_2d'");
        c.surface_mode = detail::req_str(d, s, "surface_mode");
        if (c.surface_mode != "configured" && c.surface_mode != "track")
            throw std::invalid_argument(
                "config.tire.surface_mode must be 'configured' or "
                "'track'");
        c.front = tire_params_from_dict(
            detail::req_dict(d, s, "front"), "tire.front");
        c.rear = tire_params_from_dict(
            detail::req_dict(d, s, "rear"), "tire.rear");
        c.significant_delta_m = detail::req_f64(d, s, "significant_delta_m");
        c.significance_fraction =
                detail::req_f64(d, s, "significance_fraction");
        c.distinct_normal_deg = detail::req_f64(d, s, "distinct_normal_deg");
        if (c.surface_mode == "track" || d.contains("surface_map")) {
            const nb::dict sm =
                    detail::req_dict(d, s, "surface_map");
            const char *smn = "tire.surface_map";
            SurfaceMap map;
            map.surface = surface_spec_from_dict(
                detail::req_dict(sm, smn, "surface"),
                "tire.surface_map.surface");
            const nb::object raw = detail::req(sm, smn, "sections");
            const auto sections = wire::sequence(raw, "config.tire.surface_map.sections");
            std::size_t section_index = 0;
            for (nb::handle const item: sections) {
                if (!nb::isinstance<nb::dict>(item))
                    throw std::invalid_argument(
                        "native config: 'tire.surface_map.sections' entries "
                        "must be dicts");
                const nb::dict sec = nb::borrow<nb::dict>(item);
                const std::string section_path = "tire.surface_map.sections." + std::to_string(section_index++);
                const char *ss = section_path.c_str();
                detail::section_keys(sec, "tire.surface_map.sections", "config." + section_path);
                map.sections.push_back({
                    .start_m = detail::req_f64(sec, ss, "start_m"),
                    .end_m = detail::req_f64(sec, ss, "end_m"),
                    .surface = surface_spec_from_dict(
                        detail::req_dict(sec, ss, "surface"),
                        (section_path + ".surface").c_str()),
                });
            }
            c.surface_map = std::move(map);
        }
        return c;
    }

    inline RiderForcesConfig rider_forces_from_dict(const nb::dict &top) {
        const char *s = "rider_forces";
        const nb::dict d = detail::req_dict(top, "config", s);
        RiderForcesConfig c;
        const nb::object raw = detail::req(d, s, "paths");
        const auto paths = wire::sequence(raw, "config.rider_forces.paths");
        std::size_t path_index = 0;
        for (nb::handle const item: paths) {
            if (!nb::isinstance<nb::dict>(item))
                throw std::invalid_argument(
                    "native config: 'rider_forces.paths' entries must be dicts");
            const nb::dict p = nb::borrow<nb::dict>(item);
            const std::string path = "rider_forces.paths." + std::to_string(path_index++);
            const char *sp = path.c_str();
            detail::section_keys(p, "rider_forces.paths", "config." + path);
            c.paths.push_back({
                .joint = detail::req_str(p, sp, "joint"),
                .stiffness_n_m = detail::req_f64(p, sp, "stiffness_n_m"),
                .damping_ns_m = detail::req_f64(p, sp, "damping_ns_m"),
                .preload_deflection_m = detail::req_f64(p, sp, "preload_deflection_m"),
                .offset_m = detail::req_f64(p, sp, "offset_m"),
                .unilateral = detail::req_bool(p, sp, "unilateral"),
            });
        }
        return c;
    }

    // rider_contacts (T3b-1): the ArticulatedConfig subset
    // RiderContactApplier reads, plus the pose-resolved arm_reach_m. The
    // closed-set parses name the field on rejection like the
    // ArticulatedConfig.__post_init__ membership gates they mirror.
    inline rider::RiderContactsConfig rider_contacts_from_dict(const nb::dict &top) {
        const char *s = "rider_contacts";
        const nb::dict d = detail::req_dict(top, "config", s);
        rider::RiderContactsConfig c;
        c.arm_reach_m = detail::req_f64(d, s, "arm_reach_m");
        c.saddle_patch_half_length_m = detail::req_f64(d, s, "saddle_patch_half_length_m");
        c.pedal_patch_half_length_m = detail::req_f64(d, s, "pedal_patch_half_length_m");
        c.support_pad_radius_m = detail::req_f64(d, s, "support_pad_radius_m");
        c.support_k_n_m = detail::req_f64(d, s, "support_k_n_m");
        c.support_c_ns_m = detail::req_f64(d, s, "support_c_ns_m");
        c.pedal_c_ns_m = detail::req_f64(d, s, "pedal_c_ns_m");
        c.support_tangent_k_n_m = detail::req_f64(d, s, "support_tangent_k_n_m");
        c.support_mu = detail::req_f64(d, s, "support_mu");
        c.support_length_m = detail::req_f64(d, s, "support_length_m");
        c.grip_k_n_m = detail::req_f64(d, s, "grip_k_n_m");
        c.grip_c_ns_m = detail::req_f64(d, s, "grip_c_ns_m");
        c.grip_release_distance_m = detail::req_f64(d, s, "grip_release_distance_m");
        c.grip_capture_distance_m = detail::req_f64(d, s, "grip_capture_distance_m");
        c.grip_capture_speed_mps = detail::req_f64(d, s, "grip_capture_speed_mps");
        if (d.contains("grip_pair_force_limit_n"))
            c.grip_pair_force_limit_n = wire::optional_real(
                d["grip_pair_force_limit_n"],
                "config.rider_contacts.grip_pair_force_limit_n");
        const auto pedal =
            rider::pedal_attachment(detail::req_str(d, s, "pedal_attachment"));
        if (!pedal)
            wire::invalid("config.rider_contacts.pedal_attachment",
                          "unknown attachment");
        c.pedal_attachment = *pedal;
        const auto saddle =
            rider::saddle_attachment(detail::req_str(d, s, "saddle_attachment"));
        if (!saddle)
            wire::invalid("config.rider_contacts.saddle_attachment",
                          "unknown attachment");
        c.saddle_attachment = *saddle;
        const auto grip =
            rider::grip_attachment(detail::req_str(d, s, "grip_attachment"));
        if (!grip)
            wire::invalid("config.rider_contacts.grip_attachment",
                          "unknown attachment");
        c.grip_attachment = *grip;
        return c;
    }

    // Whole-config reader: an empty dict disables every writer (checked by the
    // caller before this runs); a non-empty one must name the schema and may
    // carry each writer's section.
    inline NativeConfig native_config_from_dict(const nb::dict &d) {
        detail::section_keys(d, "config");
        const int schema = detail::req_int(d, "config", "schema");
        if (schema != 1 && schema != 2)
            throw std::invalid_argument(
                "config.schema: unsupported schema " + std::to_string(schema));
        NativeConfig c;
        if (d.contains("drive"))
            c.drive = parse_drive_config(detail::req_dict(d, "config", "drive"));
        if (d.contains("cruise")) {
            const auto section = detail::req_dict(d, "config", "cruise");
            c.cruise = CruiseConfig{
                .target_speed_kmh = detail::req_f64(section, "cruise", "target_speed_kmh"),
                .kp_nm_per_mps = detail::req_f64(section, "cruise", "kp_nm_per_mps"),
                .ki_nm_per_mps_s = detail::req_f64(section, "cruise", "ki_nm_per_mps_s"),
                .torque_ceiling_nm = detail::req_f64(section, "cruise", "torque_ceiling_nm")
            };
        }
        if (d.contains("suspension"))
            c.suspension = suspension_from_dict(d, schema);
        if (d.contains("brake"))
            c.brake = brake_from_dict(d);
        if (d.contains("resistance"))
            c.resistance = resistance_from_dict(d);
        if (d.contains("tire"))
            c.tire = tire_from_dict(d);
        if (d.contains("rider_forces"))
            c.rider_forces = rider_forces_from_dict(d);
        if (d.contains("rider_contacts"))
            c.rider_contacts = rider_contacts_from_dict(d);
        return c;
    }
} // namespace nativecfg
