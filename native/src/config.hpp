// config.hpp — the config bridge: nanobind dict → typed writer configs.
//
// `tools.native_config.project(env)` emits {'schema': 1, '<writer>': {...}}
// carrying ONLY the fields the ported writers read. An empty dict leaves
// every writer disabled (the Stepper(path) contract stays intact). Every
// required key that is missing raises std::invalid_argument naming the
// key — nanobind's default exception translator surfaces it as ValueError.
// Type/semantic validation beyond key presence lives in the writers'
// constructors, mirroring where Python puts it (dataclass vs __init__).
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
#include "drivetrain/drive_binding.hpp"

namespace nb = nanobind;

namespace nativecfg {
    // --- typed configs (mirror the Python objects' used attributes) -----------

    // AirSpringSpecs fields the force path reads; the "properties" are the same
    // expressions as physics/air_spring.py so derived values are bitwise-equal.
    struct AirSpringSpec {
        double stanchion_inner_diam_mm;
        double total_travel_mm;
        double pos_chamber_length_mm;
        double neg_chamber_length_mm;
        double token_volume_cm3;
        int max_tokens;
        double gamma;
        double atm_pressure_pa;
    };

    struct AirSpringConfig {
        AirSpringSpec specs;
        int num_tokens;
        double gauge_pressure_psi;
    };

    // BaseDamper resolved fields. `total_travel_mm` is NOT here: Charger 3 reads
    // it, Super Deluxe reads `total_stroke_mm` instead — each subclass section
    // carries the name its methods use, like the Python classes do.
    struct DamperCore {
        int max_hsc, max_lsc, max_reb;
        int hsc_clicks, lsc_clicks, rebound_clicks; // ctor-clamped below
        double c_lsc_min, c_lsc_max;
        double c_hsc_min, c_hsc_max;
        double c_reb_min, c_reb_max;
        double v_knee_comp, v_knee_reb;
    };

    struct Charger3Config {
        DamperCore core;
        double total_travel_mm;
        double hbo_start_mm;
        double c_hbo_base;
    };

    struct SuperDeluxeConfig {
        DamperCore core;
        double total_stroke_mm;
        int max_hbo, hbo_clicks;
        bool lockout_firm;
        bool legacy_behavior;
        double hbo_start_mm;
        double c_hbo_min, c_hbo_max;
        double lockout_preload_n, lockout_stiffness;
    };

    struct CoilConfig {
        double rate_n_m, preload_mm, stroke_mm;
        double bumper_length_mm, bumper_peak_n;
        bool legacy_behavior;

        [[nodiscard]] double bumper_engage_mm() const {
            return stroke_mm - bumper_length_mm;
        }
    };

    struct EndStopConfig {
        double stiffness_n_m, damping_n_s_m;
    };

    struct SuspensionConfig {
        std::string physics_mode;
        std::string fork_joint, shock_joint;
        AirSpringConfig air_spring{};
        Charger3Config fork_damper{};
        SuperDeluxeConfig shock_damper{};
        CoilConfig coil{};
        EndStopConfig end_stops{};
    };

    // BrakeController fields (braking.py:39-65): ceiling and taper band. The
    // wheel/actuator names it resolves are literals in the Python source, so
    // they live in the writer, not the config.
    struct BrakeConfig {
        double torque_ceiling_nm;
        double taper_radps;
    };

    struct CruiseConfig {
        double target_speed_kmh;
        double kp_nm_per_mps;
        double ki_nm_per_mps_s;
        double torque_ceiling_nm;
    };

    // ResistanceConfig fields ExternalResistanceApplier reads (physical_config.
    // py:266-284), plus the body names its __init__ resolves — literals there
    // too, emitted so a differently-named model stays describable.
    struct ResistanceConfig {
        double crr{};
        double rolling_taper_rad_s{};
        double rho_kg_m3{};
        double cda_m2{};
        std::array<double, 3> wind_world_mps{};
        std::array<double, 3> point_body_m{};
        std::string frame_body, front_wheel_body, rear_wheel_body;
    };

    // TireSpec fields (physics/tire.py:31-50) — the linear radial material the
    // compliant_2d path reads, plus the load-range declaration its validity flag
    // consumes. `provenance`/`pressure_pa_gauge` ride along for completeness; the
    // writer only validates them the way TireSpec.__post_init__ does.
    struct TireMaterial {
        double radial_k_n_m{}, radial_c_ns_m{}, pressure_pa_gauge{};
        std::string provenance;
        std::array<double, 2> valid_load_range_n{};
    };

    // TireParameters (physical_config.py:23-35).
    struct TireParams {
        TireMaterial material;
        double tangent_k_n_m{}, mu{}, relaxation_length_m{};
    };

    // SurfaceSpec (terrain/surface.py:31-88) with the registry name already
    // resolved — the writer only ever calls mu(slip) and reads .name.
    struct SurfaceSpec {
        std::string name;
        double mu_peak{}, mu_slide{}, slip_stiffness_per_load{}, stribeck_speed_mps{};
    };

    // SurfaceSection — a half-open material interval on the track axis.
    struct SurfaceSection {
        double start_m, end_m;
        SurfaceSpec surface;
    };

    // SurfaceMap (terrain/surface.py:160-206): a default material plus sorted
    // non-overlapping sections; `at` scans sections then falls back to default.
    struct SurfaceMap {
        SurfaceSpec surface;
        std::vector<SurfaceSection> sections;
    };

    // TireBackendConfig (physical_config.py:38-63), restricted to the backend the
    // native port implements: 'compliant_2d'.
    struct TireConfig {
        std::string backend;
        std::string surface_mode; // 'configured' | 'track'
        TireParams front, rear;
        double significant_delta_m{}, significance_fraction{}, distinct_normal_deg{};
        std::optional<SurfaceMap> surface_map;
    };

    // RiderForceApplier per-path state (rider_forces.py:35-76): the body's
    // resolved spring params plus the construction-time pedal offset
    // (set_pedal_offsets is applied before project() runs — the config carries
    // the effective offset_m, not the crank derivation). `joint` re-resolves to
    // qposadr/dofadr through _resolve_slide's checks.
    struct RiderPathConfig {
        std::string joint;
        double stiffness_n_m, damping_ns_m;
        double preload_deflection_m, offset_m;
        bool unilateral;
    };

    struct RiderForcesConfig {
        std::vector<RiderPathConfig> paths; // _paths order
    };

    struct NativeConfig {
        std::optional<drivetrain::DriveConfig> drive;
        std::optional<CruiseConfig> cruise;
        std::optional<SuspensionConfig> suspension;
        std::optional<BrakeConfig> brake;
        std::optional<ResistanceConfig> resistance;
        std::optional<TireConfig> tire;
        std::optional<RiderForcesConfig> rider_forces;
    };

    // --- dict readers ----------------------------------------------------------

    namespace detail {
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
            return nb::borrow<nb::dict>(v);
        }

        inline double req_f64(const nb::dict &d, const char *s, const char *k) {
            return nb::cast<double>(req(d, s, k));
        }

        inline int req_int(const nb::dict &d, const char *s, const char *k) {
            return nb::cast<int>(req(d, s, k));
        }

        inline bool req_bool(const nb::dict &d, const char *s, const char *k) {
            return nb::cast<bool>(req(d, s, k));
        }

        inline std::string req_str(const nb::dict &d, const char *s, const char *k) {
            return nb::cast<std::string>(req(d, s, k));
        }

        // Exactly-three-floats sequence reader (wind_world_mps / point_body_m). A
        // non-sequence or wrong length raises naming the key, like req_* above.
        inline std::array<double, 3> req_vec3(const nb::dict &d, const char *s,
                                              const char *k) {
            std::vector<double> v;
            try {
                v = nb::cast<std::vector<double> >(req(d, s, k));
            } catch (const nb::cast_error &) {
                throw std::invalid_argument(
                    "native config: '" + std::string(s) + "." + k +
                    "' must be a sequence of 3 floats");
            }
            if (v.size() != 3)
                throw std::invalid_argument(
                    "native config: '" + std::string(s) + "." + k +
                    "' must have exactly 3 elements");
            return {v[0], v[1], v[2]};
        }

        // Two-float sequence reader (tire valid_load_range_n).
        inline std::array<double, 2> req_vec2(const nb::dict &d, const char *s,
                                              const char *k) {
            std::vector<double> v;
            try {
                v = nb::cast<std::vector<double> >(req(d, s, k));
            } catch (const nb::cast_error &) {
                throw std::invalid_argument(
                    "native config: '" + std::string(s) + "." + k +
                    "' must be a sequence of 2 floats");
            }
            if (v.size() != 2)
                throw std::invalid_argument(
                    "native config: '" + std::string(s) + "." + k +
                    "' must have exactly 2 elements");
            return {v[0], v[1]};
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

    inline SuspensionConfig suspension_from_dict(const nb::dict &top) {
        const char *s = "suspension";
        const nb::dict d = detail::req_dict(top, "config", s);
        SuspensionConfig c;
        c.physics_mode = detail::req_str(d, s, "physics_mode");
        if (c.physics_mode != "legacy" && c.physics_mode != "physical")
            throw std::invalid_argument(
                "native config: suspension.physics_mode must be 'legacy' or "
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
        TireParams p;
        {
            const nb::dict m = detail::req_dict(d, s, "material");
            const char *sm = "tire.material";
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
                "native config: tire.backend must be 'compliant_2d'");
        c.surface_mode = detail::req_str(d, s, "surface_mode");
        if (c.surface_mode != "configured" && c.surface_mode != "track")
            throw std::invalid_argument(
                "native config: tire.surface_mode must be 'configured' or "
                "'track'");
        c.front = tire_params_from_dict(
            detail::req_dict(d, s, "front"), "tire.front");
        c.rear = tire_params_from_dict(
            detail::req_dict(d, s, "rear"), "tire.rear");
        c.significant_delta_m = detail::req_f64(d, s, "significant_delta_m");
        c.significance_fraction =
                detail::req_f64(d, s, "significance_fraction");
        c.distinct_normal_deg = detail::req_f64(d, s, "distinct_normal_deg");
        if (c.surface_mode == "track") {
            const nb::dict sm =
                    detail::req_dict(d, s, "surface_map");
            const char *smn = "tire.surface_map";
            SurfaceMap map;
            map.surface = surface_spec_from_dict(
                detail::req_dict(sm, smn, "surface"),
                "tire.surface_map.surface");
            const nb::object raw = detail::req(sm, smn, "sections");
            if (!nb::isinstance<nb::list>(raw) &&
                !nb::isinstance<nb::tuple>(raw))
                throw std::invalid_argument(
                    "native config: 'tire.surface_map.sections' must be a "
                    "sequence of dicts");
            for (nb::handle const item: nb::borrow<nb::sequence>(raw)) {
                if (!nb::isinstance<nb::dict>(item))
                    throw std::invalid_argument(
                        "native config: 'tire.surface_map.sections' entries "
                        "must be dicts");
                const nb::dict sec = nb::borrow<nb::dict>(item);
                const char *ss = "tire.surface_map.sections";
                map.sections.push_back({
                    .start_m = detail::req_f64(sec, ss, "start_m"),
                    .end_m = detail::req_f64(sec, ss, "end_m"),
                    .surface = surface_spec_from_dict(
                        detail::req_dict(sec, ss, "surface"),
                        "tire.surface_map.sections.surface"),
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
        if (!nb::isinstance<nb::list>(raw) && !nb::isinstance<nb::tuple>(raw))
            throw std::invalid_argument(
                "native config: 'rider_forces.paths' must be a sequence of "
                "dicts");
        for (nb::handle const item: nb::borrow<nb::sequence>(raw)) {
            if (!nb::isinstance<nb::dict>(item))
                throw std::invalid_argument(
                    "native config: 'rider_forces.paths' entries must be dicts");
            const nb::dict p = nb::borrow<nb::dict>(item);
            const char *sp = "rider_forces.paths";
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

    // Whole-config reader: an empty dict disables every writer (checked by the
    // caller before this runs); a non-empty one must name the schema and may
    // carry each writer's section.
    inline NativeConfig native_config_from_dict(const nb::dict &d) {
        const int schema = detail::req_int(d, "config", "schema");
        if (schema != 1)
            throw std::invalid_argument(
                "native config: unsupported schema " + std::to_string(schema));
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
            c.suspension = suspension_from_dict(d);
        if (d.contains("brake"))
            c.brake = brake_from_dict(d);
        if (d.contains("resistance"))
            c.resistance = resistance_from_dict(d);
        if (d.contains("tire"))
            c.tire = tire_from_dict(d);
        if (d.contains("rider_forces"))
            c.rider_forces = rider_forces_from_dict(d);
        return c;
    }
} // namespace nativecfg
