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
    int hsc_clicks, lsc_clicks, rebound_clicks;   // ctor-clamped below
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
    AirSpringConfig air_spring;
    Charger3Config fork_damper;
    SuperDeluxeConfig shock_damper;
    CoilConfig coil;
    EndStopConfig end_stops;
};

// BrakeController fields (braking.py:39-65): ceiling and taper band. The
// wheel/actuator names it resolves are literals in the Python source, so
// they live in the writer, not the config.
struct BrakeConfig {
    double torque_ceiling_nm;
    double taper_radps;
};

// ResistanceConfig fields ExternalResistanceApplier reads (physical_config.
// py:266-284), plus the body names its __init__ resolves — literals there
// too, emitted so a differently-named model stays describable.
struct ResistanceConfig {
    double crr;
    double rolling_taper_rad_s;
    double rho_kg_m3;
    double cda_m2;
    std::array<double, 3> wind_world_mps;
    std::array<double, 3> point_body_m;
    std::string frame_body, front_wheel_body, rear_wheel_body;
};

struct NativeConfig {
    std::optional<SuspensionConfig> suspension;
    std::optional<BrakeConfig> brake;
    std::optional<ResistanceConfig> resistance;
};

// --- dict readers ----------------------------------------------------------

namespace detail {

inline nb::object req(const nb::dict& d, const char* section,
                      const char* key) {
    if (!d.contains(key))
        throw std::invalid_argument(
            "native config: missing key '" + std::string(section) + "." +
            key + "'");
    return d[key];
}

inline nb::dict req_dict(const nb::dict& d, const char* section,
                         const char* key) {
    nb::object v = req(d, section, key);
    if (!nb::isinstance<nb::dict>(v))
        throw std::invalid_argument(
            "native config: '" + std::string(section) + "." + key +
            "' must be a dict");
    return nb::borrow<nb::dict>(v);
}

inline double req_f64(const nb::dict& d, const char* s, const char* k) {
    return nb::cast<double>(req(d, s, k));
}
inline int req_int(const nb::dict& d, const char* s, const char* k) {
    return nb::cast<int>(req(d, s, k));
}
inline bool req_bool(const nb::dict& d, const char* s, const char* k) {
    return nb::cast<bool>(req(d, s, k));
}
inline std::string req_str(const nb::dict& d, const char* s, const char* k) {
    return nb::cast<std::string>(req(d, s, k));
}

// Exactly-three-floats sequence reader (wind_world_mps / point_body_m). A
// non-sequence or wrong length raises naming the key, like req_* above.
inline std::array<double, 3> req_vec3(const nb::dict& d, const char* s,
                                      const char* k) {
    std::vector<double> v;
    try {
        v = nb::cast<std::vector<double>>(req(d, s, k));
    } catch (const nb::cast_error&) {
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

inline DamperCore damper_core(const nb::dict& d, const char* s) {
    DamperCore c;
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

}  // namespace detail

inline SuspensionConfig suspension_from_dict(const nb::dict& top) {
    const char* s = "suspension";
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
        const char* sa = "suspension.air_spring";
        c.air_spring.specs = {
            detail::req_f64(a, sa, "stanchion_inner_diam_mm"),
            detail::req_f64(a, sa, "total_travel_mm"),
            detail::req_f64(a, sa, "pos_chamber_length_mm"),
            detail::req_f64(a, sa, "neg_chamber_length_mm"),
            detail::req_f64(a, sa, "token_volume_cm3"),
            detail::req_int(a, sa, "max_tokens"),
            detail::req_f64(a, sa, "gamma"),
            detail::req_f64(a, sa, "atm_pressure_pa"),
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
        const char* sf = "suspension.fork_damper";
        c.fork_damper.core = detail::damper_core(f, sf);
        c.fork_damper.total_travel_mm = detail::req_f64(f, sf, "total_travel_mm");
        c.fork_damper.hbo_start_mm = detail::req_f64(f, sf, "hbo_start_mm");
        c.fork_damper.c_hbo_base = detail::req_f64(f, sf, "c_hbo_base");
    }
    {
        const nb::dict h = detail::req_dict(d, s, "shock_damper");
        const char* sh = "suspension.shock_damper";
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
        const char* sk = "suspension.coil";
        c.coil = {
            detail::req_f64(k, sk, "rate_n_m"),
            detail::req_f64(k, sk, "preload_mm"),
            detail::req_f64(k, sk, "stroke_mm"),
            detail::req_f64(k, sk, "bumper_length_mm"),
            detail::req_f64(k, sk, "bumper_peak_n"),
            detail::req_bool(k, sk, "legacy_behavior"),
        };
    }
    {
        const nb::dict e = detail::req_dict(d, s, "end_stops");
        const char* se = "suspension.end_stops";
        c.end_stops = {
            detail::req_f64(e, se, "stiffness_n_m"),
            detail::req_f64(e, se, "damping_n_s_m"),
        };
    }
    return c;
}

inline BrakeConfig brake_from_dict(const nb::dict& top) {
    const char* s = "brake";
    const nb::dict d = detail::req_dict(top, "config", s);
    BrakeConfig c;
    c.torque_ceiling_nm = detail::req_f64(d, s, "torque_ceiling_nm");
    c.taper_radps = detail::req_f64(d, s, "taper_radps");
    return c;
}

inline ResistanceConfig resistance_from_dict(const nb::dict& top) {
    const char* s = "resistance";
    const nb::dict d = detail::req_dict(top, "config", s);
    ResistanceConfig c;
    c.crr = detail::req_f64(d, s, "crr");
    c.rolling_taper_rad_s = detail::req_f64(d, s, "rolling_taper_rad_s");
    c.rho_kg_m3 = detail::req_f64(d, s, "rho_kg_m3");
    c.cda_m2 = detail::req_f64(d, s, "cda_m2");
    c.wind_world_mps = detail::req_vec3(d, s, "wind_world_mps");
    c.point_body_m = detail::req_vec3(d, s, "point_body_m");
    const nb::dict b = detail::req_dict(d, s, "bodies");
    const char* sb = "resistance.bodies";
    c.frame_body = detail::req_str(b, sb, "frame");
    c.front_wheel_body = detail::req_str(b, sb, "front_wheel");
    c.rear_wheel_body = detail::req_str(b, sb, "rear_wheel");
    return c;
}

// Whole-config reader: an empty dict disables every writer (checked by the
// caller before this runs); a non-empty one must name the schema and may
// carry each writer's section.
inline NativeConfig native_config_from_dict(const nb::dict& d) {
    const int schema = detail::req_int(d, "config", "schema");
    if (schema != 1)
        throw std::invalid_argument(
            "native config: unsupported schema " + std::to_string(schema));
    NativeConfig c;
    if (d.contains("suspension"))
        c.suspension = suspension_from_dict(d);
    if (d.contains("brake"))
        c.brake = brake_from_dict(d);
    if (d.contains("resistance"))
        c.resistance = resistance_from_dict(d);
    return c;
}

}  // namespace nativecfg
