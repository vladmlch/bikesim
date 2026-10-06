// writers/suspension.cpp — expression-for-expression port of the ride-mode
// suspension force path. Every function mirrors the Python source it names;
// comments cite file:line at the time of porting. `a ** b` goes through
// pyfloat::pow so the call reaches libm pow() like CPython float.__pow__
// (see pyfloat.hpp); `max`/`min` map to std::max/std::min, which agree with
// Python's two-argument builtins element-for-element (incl. ±0/NaN cases).
#include "suspension.hpp"
#include "../config_validation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <format>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>

#include "pyfloat.hpp"

namespace {
    // physics/air_spring.py:14 — module constant.
    constexpr double kPsiToPa = 6894.757293168;

    // --- AirSpringSpecs derived quantities (air_spring.py properties) ----------

    // piston_area_m2: `np.pi * (r_m ** 2)` — np.pi and std::numbers::pi are the
    // same double; `r_m ** 2` stays a libm pow call.
    double piston_area_m2(const nativecfg::AirSpringSpec &s) {
        const double r_m = (s.stanchion_inner_diam_mm / 2.0) / 1000.0;
        return std::numbers::pi * pyfloat::pow(r_m, 2.0);
    }

    double token_volume_m3(const nativecfg::AirSpringSpec &s) {
        return s.token_volume_cm3 * 1e-6;
    }

    double base_pos_volume_m3(const nativecfg::AirSpringSpec &s) {
        return piston_area_m2(s) * (s.pos_chamber_length_mm / 1000.0);
    }

    double base_neg_volume_m3(const nativecfg::AirSpringSpec &s) {
        return piston_area_m2(s) * (s.neg_chamber_length_mm / 1000.0);
    }

    // ForkAirSpring.compute_axial_force (air_spring.py:142-154) — inlines
    // compute_pressures_pa:113-140 and compute_volumes:94-126 on their behalf.
    double air_axial_force(const nativecfg::AirSpringConfig &air,
                           double travel_mm) {
        const nativecfg::AirSpringSpec &s = air.specs;
        const int tokens =
                std::max(0, std::min(s.max_tokens, air.num_tokens));
        const double travel_m =
                std::max(0.0, std::min(s.total_travel_mm, travel_mm)) / 1000.0;
        const double area = piston_area_m2(s);
        const double v_pos_0 = base_pos_volume_m3(s) - tokens * token_volume_m3(s);
        const double v_neg_0 = base_neg_volume_m3(s);
        const double v_disp = validation::derived(area * travel_m, "ForkAirSpring.displaced_volume");
        validation::derived(v_pos_0, "ForkAirSpring.initial_positive_volume");
        validation::derived(v_neg_0, "ForkAirSpring.initial_negative_volume");
        if (v_pos_0 <= 0. || v_neg_0 <= 0. || v_disp >= v_pos_0)
            throw std::invalid_argument(std::format(
                "pneumatic volume exhausted at travel={:.1f} mm (displaced "
                "volume {:.1f} cm³ >= initial chamber volume {:.1f} cm³)",
                travel_mm, v_disp * 1e6, v_pos_0 * 1e6));
        const double v_pos = v_pos_0 - v_disp;
        const double v_neg = v_neg_0 + v_disp;
        validation::derived(v_pos, "ForkAirSpring.positive_volume");
        validation::derived(v_neg, "ForkAirSpring.negative_volume");
        const double p_abs_0 =
                air.gauge_pressure_psi * kPsiToPa + s.atm_pressure_pa;
        validation::derived(p_abs_0, "ForkAirSpring.absolute_pressure");
        if (v_pos <= 0. || v_neg <= 0.) throw std::invalid_argument("ForkAirSpring.volumes");
        const double p_pos = p_abs_0 * pyfloat::pow(v_pos_0 / v_pos, s.gamma);
        const double p_neg = p_abs_0 * pyfloat::pow(v_neg_0 / v_neg, s.gamma);
        validation::derived(p_pos, "ForkAirSpring.positive_pressure");
        validation::derived(p_neg, "ForkAirSpring.negative_pressure");
        return std::max(0.0, validation::derived(area * (p_pos - p_neg), "ForkAirSpring.axial_force"));
    }

    // --- damper helpers (physics/damper.py BaseDamper) -------------------------

    struct Coeffs {
        double c_lsc, c_hsc, c_reb;
    };

    // BaseDamper._calc_fractions_and_coeffs (damper.py:122-131); the frac_*
    // intermediates are kept so the statement sequence is identical.
    Coeffs calc_fractions_and_coeffs(const nativecfg::DamperCore &c) {
        const double frac_lsc = std::clamp(c.lsc_clicks, 0, c.max_lsc) / static_cast<double>(c.max_lsc);
        const double frac_hsc = std::clamp(c.hsc_clicks, 0, c.max_hsc) / static_cast<double>(c.max_hsc);
        const double frac_reb = std::clamp(c.rebound_clicks, 0, c.max_reb) / static_cast<double>(c.max_reb);
        const double c_lsc = c.c_lsc_min + (c.c_lsc_max - c.c_lsc_min) *
                             pyfloat::pow(frac_lsc, 1.25);
        const double c_hsc = c.c_hsc_min + (c.c_hsc_max - c.c_hsc_min) *
                             pyfloat::pow(frac_hsc, 1.15);
        const double c_reb = c.c_reb_min + (c.c_reb_max - c.c_reb_min) *
                             pyfloat::pow(frac_reb, 1.20);
        return {.c_lsc = c_lsc, .c_hsc = c_hsc, .c_reb = c_reb};
    }

    // BaseDamper._compute_base_damping (damper.py:133-150).
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror damper.py
    double base_damping(const nativecfg::DamperCore &c, double v, double c_lsc,
                        double c_hsc, double c_reb) {
        if (v >= 0.0) {
            if (v <= c.v_knee_comp)
                return c_lsc * v * (1.0 + 0.35 * (v / c.v_knee_comp));
            const double f_knee = c_lsc * c.v_knee_comp * 1.35;
            const double v_excess = v - c.v_knee_comp;
            return f_knee + c_hsc * pyfloat::pow(v_excess, 0.88) *
                   pyfloat::pow(c.v_knee_comp, 0.12);
        }
        const double v_abs = -v;
        if (v_abs <= c.v_knee_reb)
            return -c_reb * v_abs * (1.0 + 0.25 * (v_abs / c.v_knee_reb));
        const double f_knee = c_reb * c.v_knee_reb * 1.25;
        const double v_excess = v_abs - c.v_knee_reb;
        const double c_hs_reb = c_reb * 0.45;
        return -(f_knee + c_hs_reb * pyfloat::pow(v_excess, 0.90) *
                 pyfloat::pow(c.v_knee_reb, 0.10));
    }

    // Charger3Damper.compute_damping_force (damper.py:214-224).
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    double charger3_damping_force(const nativecfg::Charger3Config &d,
                                  double velocity_mps, double travel_mm) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const Coeffs k = calc_fractions_and_coeffs(d.core);
        const double v = velocity_mps;
        double f_damp = base_damping(d.core, v, k.c_lsc, k.c_hsc, k.c_reb);
        if (travel_mm > d.hbo_start_mm && v > 0.0) {
            if (d.total_travel_mm == d.hbo_start_mm) throw std::invalid_argument("Charger3Damper.hbo_start_mm: zero denominator");
            const double hbo_prog = std::max(
                0.0, std::min(1.0, (travel_mm - d.hbo_start_mm) /
                                   (d.total_travel_mm - d.hbo_start_mm)));
            f_damp += d.c_hbo_base * pyfloat::pow(hbo_prog, 2.0) * v;
        }
        return validation::derived(f_damp, "SuspensionWriter.damper_force");
    }

    struct DampingParts {
        double base_n, hbo_n;
    };

    // SuperDeluxeDamper._compute_legacy_damping_force (damper.py:362-377).
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    double sd_legacy_damping_force(const nativecfg::SuperDeluxeConfig &d,
                                   double velocity_mps, double stroke_mm) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const Coeffs k = calc_fractions_and_coeffs(d.core);
        // get_effective_coefficients (damper.py:312-328) HBO terms.
        const double frac_hbo = std::clamp(d.hbo_clicks, 0, d.max_hbo) / static_cast<double>(d.max_hbo);
        const double c_hbo =
                d.c_hbo_min + (d.c_hbo_max - d.c_hbo_min) * pyfloat::pow(frac_hbo, 1.30);
        const double v = velocity_mps;
        if (d.lockout_firm && v > 0.0) {
            if (v < 0.03)
                return d.lockout_stiffness * v;
            return d.lockout_preload_n + (k.c_hsc * 1.8) * (v - 0.03);
        }
        double f_damp = base_damping(d.core, v, k.c_lsc, k.c_hsc, k.c_reb);
        if (stroke_mm > d.hbo_start_mm && v > 0.0) {
            if (d.total_stroke_mm == d.hbo_start_mm) throw std::invalid_argument("SuperDeluxeDamper.hbo_start_mm: zero denominator");
            const double hbo_prog = std::max(
                0.0, std::min(1.0, (stroke_mm - d.hbo_start_mm) /
                                   (d.total_stroke_mm - d.hbo_start_mm)));
            f_damp += c_hbo * pyfloat::pow(hbo_prog, 2.0) * v;
        }
        return validation::derived(f_damp, "SuspensionWriter.damper_force");
    }

    // SuperDeluxeDamper.compute_damping_components (damper.py:338-360).
    DampingParts sd_damping_components(const nativecfg::SuperDeluxeConfig &d,
                                       double velocity_mps, double stroke_mm) {
        if (d.legacy_behavior)
            return {.base_n = sd_legacy_damping_force(d, velocity_mps, stroke_mm), .hbo_n = 0.0};
        const double v = velocity_mps;
        if (!std::isfinite(v) || !std::isfinite(stroke_mm))
            throw std::invalid_argument("non-finite shock damper state");
        const Coeffs k = calc_fractions_and_coeffs(d.core);
        const double frac_hbo = std::clamp(d.hbo_clicks, 0, d.max_hbo) / static_cast<double>(d.max_hbo);
        const double c_hbo =
                d.c_hbo_min + (d.c_hbo_max - d.c_hbo_min) * pyfloat::pow(frac_hbo, 1.30);
        double f_damp = std::numeric_limits<double>::quiet_NaN();
        if (d.lockout_firm && v > 0.0) {
            const double knee = 0.03;
            const double low = d.lockout_stiffness;
            const double high = 1.8 * k.c_hsc;
            f_damp = (v < knee) ? low * v : low * knee + high * (v - knee);
        } else {
            f_damp = base_damping(d.core, v, k.c_lsc, k.c_hsc, k.c_reb);
        }
        double hbo = 0.0;
        if (stroke_mm > d.hbo_start_mm && v > 0.0) {
            if (d.total_stroke_mm == d.hbo_start_mm) throw std::invalid_argument("SuperDeluxeDamper.hbo_start_mm: zero denominator");
            const double fraction =
                    std::min(1.0, (stroke_mm - d.hbo_start_mm) /
                                  (d.total_stroke_mm - d.hbo_start_mm));
            hbo = c_hbo * pyfloat::pow(fraction, 2.0) * v;
        }
        return {.base_n = validation::derived(f_damp, "SuperDeluxeDamper.base_force"), .hbo_n = validation::derived(hbo, "SuperDeluxeDamper.hbo_force")};
    }

    // SuperDeluxeDamper.compute_damping_force (damper.py:330-336).
    double sd_damping_force(const nativecfg::SuperDeluxeConfig &d,
                            double velocity_mps, double stroke_mm) {
        if (d.legacy_behavior)
            return sd_legacy_damping_force(d, velocity_mps, stroke_mm);
        const DampingParts p =
                sd_damping_components(d, velocity_mps, stroke_mm);
        return validation::derived(p.base_n + p.hbo_n, "SuperDeluxeDamper.force");
    }

    // CoilShock.compute_spring_force (coil_shock.py:77-90).
    double coil_spring_force(const nativecfg::CoilConfig &s, double stroke_mm) {
        double compression_mm = stroke_mm + s.preload_mm;
        if (!s.legacy_behavior)
            compression_mm = std::max(0.0, compression_mm);
        return validation::derived(s.rate_n_m * compression_mm / 1000.0, "CoilShock.spring_force");
    }

    // CoilShock.compute_bumper_force (coil_shock.py:92-104).
    double coil_bumper_force(const nativecfg::CoilConfig &s, double stroke_mm) {
        const double excess_mm =
                std::max(0.0, stroke_mm - s.bumper_engage_mm());
        return validation::derived(s.bumper_peak_n *
               pyfloat::pow(excess_mm / s.bumper_length_mm, 2.0), "CoilShock.bumper_force");
    }

    // physics/stops.py:6-46 — end_stop, including its parameter validation.
    std::pair<double, double> end_stop(double q, double v, double lo, double hi,
                                       double k, double c,
                                       double upper_boundary_force_n = 0.0,
                                       double upper_boundary_energy_j = 0.0) {
        if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(lo) ||
            !std::isfinite(hi) || !std::isfinite(k) || !std::isfinite(c) ||
            !std::isfinite(upper_boundary_force_n) ||
            !std::isfinite(upper_boundary_energy_j) || hi <= lo || k < 0.0 ||
            c < 0.0 || upper_boundary_force_n < 0.0 ||
            upper_boundary_energy_j < 0.0)
            throw std::invalid_argument("invalid end-stop parameters");

        const double low_depth = std::max(lo - q, 0.0);
        if (low_depth > 0.0)
            return {
                std::max(0.0, validation::derived(k * low_depth - c * v, "end_stop.raw_force")),
                0.5 * k * pyfloat::pow(low_depth, 2.0)
            };

        const double high_depth = std::max(q - hi, 0.0);
        if (high_depth > 0.0 || (q == hi && upper_boundary_force_n > 0.0)) {
            const double elastic_force = upper_boundary_force_n + k * high_depth;
            const double force = -std::max(0.0, validation::derived(elastic_force + c * v, "end_stop.raw_force"));
            const double energy = upper_boundary_energy_j +
                                  upper_boundary_force_n * high_depth +
                                  0.5 * k * pyfloat::pow(high_depth, 2.0);
            return {force, energy};
        }
        return {0.0, 0.0};
    }

    // forces.py:67-99 — _resolve_compression_joint, same checks and order.
    std::pair<int, int> resolve_compression_joint(const mjModel *m,
                                                  const std::string &joint_name,
                                                  bool allow_negative_lower) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (jid < 0)
            throw std::invalid_argument(
                "model has no joint '" + joint_name +
                "'; suspension forces cannot be applied");

        // views::counted everywhere — -Wunsafe-buffer-usage rejects raw
        // indexing into mjModel pointer fields (same pattern as stepper.hpp).
        const std::span<const mjtNum> range =
                std::views::counted(m->jnt_range, 2 * m->njnt);
        const double lo = range[2 * static_cast<std::size_t>(jid)];
        const double hi = range[2 * static_cast<std::size_t>(jid) + 1];
        const std::span<const mjtBool> limited =
                std::views::counted(m->jnt_limited, m->njnt);
        const bool lower_valid =
                std::abs(lo) <= 1e-9 ||
                (allow_negative_lower && joint_name == "shock_stroke" && lo < 0.0);
        if (!limited[static_cast<std::size_t>(jid)] || !lower_valid || hi <= 0.0)
            throw std::invalid_argument(std::format(
                "joint '{}' has range [{:.4f}, {:.4f}]; the suspension force "
                "path requires a compression-positive limited range",
                joint_name, lo, hi));

        const std::span<const int> qposadr =
                std::views::counted(m->jnt_qposadr, m->njnt);
        const std::span<const int> dofadr =
                std::views::counted(m->jnt_dofadr, m->njnt);
        return {
            qposadr[static_cast<std::size_t>(jid)],
            dofadr[static_cast<std::size_t>(jid)]
        };
    }
} // namespace

SuspensionWriter::SuspensionWriter(const mjModel *m,
                                   nativecfg::SuspensionConfig config)
    : cfg_(std::move(config)),
      physical_(cfg_.physics_mode == "physical"),
      nq_(m->nq),
      nv_(m->nv) {
    nativecfg::validate(cfg_);
    // forces.py:49-52 — only the physical shock may start below zero
    // (top-out travel).
    std::tie(fork_qposadr_, fork_dofadr_) =
            resolve_compression_joint(m, cfg_.fork_joint, false);
    std::tie(shock_qposadr_, shock_dofadr_) =
            resolve_compression_joint(m, cfg_.shock_joint, physical_);

    // Component-constructor validation, mirroring the Python __init__s.
    // CoilShock.__init__ (coil_shock.py:64-75).
    const nativecfg::CoilConfig &s = cfg_.coil;
    if (!std::isfinite(s.rate_n_m) || !std::isfinite(s.preload_mm) ||
        !std::isfinite(s.stroke_mm) || !std::isfinite(s.bumper_length_mm) ||
        !std::isfinite(s.bumper_peak_n) || s.preload_mm < 0.0 ||
        s.rate_n_m <= 0.0 || s.stroke_mm <= 0.0 ||
        !(0.0 < s.bumper_length_mm && s.bumper_length_mm <= s.stroke_mm) ||
        s.bumper_peak_n <= 0.0)
        throw std::invalid_argument("invalid coil shock specifications");

    // Charger3Damper.__init__ (damper.py:167-189): the resolved-zone
    // invariant is what survives once hbo_start_mm is projected resolved —
    // the legacy-vs-modern choice was made upstream in Python.
    const nativecfg::Charger3Config &f = cfg_.fork_damper;
    if (!std::isfinite(f.total_travel_mm) || f.total_travel_mm <= 0.0)
        throw std::invalid_argument(
            "fork travel must be finite and positive");
    if (!std::isfinite(f.hbo_start_mm) || f.hbo_start_mm < 0.0)
        throw std::invalid_argument("config.suspension.fork_damper.hbo_start_mm: invalid HBO start");
    if (!f.legacy_behavior && !(0.0 < f.total_travel_mm - f.hbo_start_mm &&
          f.total_travel_mm - f.hbo_start_mm <= f.total_travel_mm))
        throw std::invalid_argument(
            "fork HBO zone must be positive and no longer than travel");

    // SuperDeluxeDamper.__init__ (damper.py:254-281).
    const nativecfg::SuperDeluxeConfig &h = cfg_.shock_damper;
    if (!std::isfinite(h.total_stroke_mm) || h.total_stroke_mm <= 0.0)
        throw std::invalid_argument(
            "shock stroke must be finite and positive");
    if (!h.legacy_behavior &&
        !(0.0 < h.total_stroke_mm - h.hbo_start_mm &&
          h.total_stroke_mm - h.hbo_start_mm <= h.total_stroke_mm))
        throw std::invalid_argument(
            "shock HBO zone must be positive and no longer than stroke");
}

std::vector<SuspensionWriter::Component>
SuspensionWriter::components(const mjData *d) const {
    // forces.py:127-131 — SuspensionController.compute_fork_force
    // (controllers.py:32-42). counted spans, not raw indexing
    // (-Wunsafe-buffer-usage).
    const std::span<const mjtNum> qpos = std::views::counted(d->qpos, nq_);
    const std::span<const mjtNum> qvel = std::views::counted(d->qvel, nv_);
    const double travel_mm =
            qpos[static_cast<std::size_t>(fork_qposadr_)] * 1000.0;
    const double fork_velocity_mps =
            qvel[static_cast<std::size_t>(fork_dofadr_)];
    const double fork_spring = air_axial_force(cfg_.air_spring, travel_mm);
    const double fork_damper =
            charger3_damping_force(cfg_.fork_damper, fork_velocity_mps, travel_mm);
    const double fork_total = fork_spring + fork_damper;

    // forces.py:133-185 — the shock side.
    const double stroke_mm =
            qpos[static_cast<std::size_t>(shock_qposadr_)] * 1000.0;
    const double shock_velocity_mps =
            qvel[static_cast<std::size_t>(shock_dofadr_)];
    const double shock_spring = coil_spring_force(cfg_.coil, stroke_mm);
    const double stroke_limit_mm = cfg_.coil.stroke_mm;
    const double shock_bumper =
            (physical_ && stroke_mm > stroke_limit_mm)
                ? 0.0
                : coil_bumper_force(cfg_.coil, stroke_mm);
    const double shock_damper =
            sd_damping_force(cfg_.shock_damper, shock_velocity_mps, stroke_mm);
    const double shock_hbo =
            physical_
                ? sd_damping_components(cfg_.shock_damper,
                                        shock_velocity_mps, stroke_mm)
                .hbo_n
                : 0.0;

    double top_out_force = 0.0, top_out_energy = 0.0;
    double upper_force = 0.0, upper_energy = 0.0;
    double bumper_energy = 0.0;
    double coil_energy = 0.0;
    if (physical_) {
        // forces.py:154-179.
        const nativecfg::EndStopConfig &stop = cfg_.end_stops;
        const double stroke_m = stroke_mm / 1000.0;
        const double limit_m = stroke_limit_mm / 1000.0;
        if (stroke_m < 0.0) {
            const auto [f, e] =
                    end_stop(stroke_m, shock_velocity_mps, 0.0, limit_m,
                             stop.stiffness_n_m, stop.damping_n_s_m);
            top_out_force = f;
            top_out_energy = e;
        }
        const double bumper_length_m = cfg_.coil.bumper_length_mm / 1000.0;
        const double bumper_peak_n = cfg_.coil.bumper_peak_n;
        const double bumper_full_energy =
                bumper_peak_n * bumper_length_m / 3.0;
        if (stroke_m <= limit_m) {
            const double bumper_depth_m =
                    std::max(0.0, stroke_m - (limit_m - bumper_length_m));
            bumper_energy =
                    bumper_peak_n * pyfloat::pow(bumper_depth_m, 3.0) /
                    (3.0 * pyfloat::pow(bumper_length_m, 2.0));
        } else {
            const auto [f, e] =
                    end_stop(stroke_m, shock_velocity_mps, 0.0, limit_m,
                             stop.stiffness_n_m, stop.damping_n_s_m, bumper_peak_n,
                             bumper_full_energy);
            upper_force = f;
            upper_energy = e;
        }
        const double coil_compression_m =
                std::max(0.0, (stroke_mm + cfg_.coil.preload_mm) / 1000.0);
        coil_energy =
                0.5 * cfg_.coil.rate_n_m * pyfloat::pow(coil_compression_m, 2.0);
    }

    const double shock_top_out = -top_out_force;
    const double shock_upper_stop = -upper_force;
    const double shock_total = shock_spring + shock_bumper + shock_damper +
                               shock_top_out + shock_upper_stop;

    // Last-call telemetry mirrors SuspensionForceApplier's self.*_n fields
    // and potential_energy_j (forces.py:207-221); kept as a snapshot for a
    // future telemetry surface — it does not feed the components.
    for (const double value: {fork_total, shock_total, top_out_force, top_out_energy, upper_force, upper_energy, bumper_energy, coil_energy})
        validation::derived(value, "SuspensionWriter.force_or_energy");
    last_ = Telemetry{
        .fork_spring_n = fork_spring, .fork_damper_n = fork_damper, .fork_total_n = fork_total,
        .shock_spring_n = shock_spring, .shock_bumper_n = shock_bumper, .shock_damper_n = shock_damper,
        .shock_top_out_n = shock_top_out, .shock_upper_stop_n = shock_upper_stop, .shock_total_n = shock_total,
        .potential_energy_j = {
            .shock_coil = coil_energy, .shock_bumper = bumper_energy, .shock_top_out = top_out_energy,
            .shock_upper_stop = upper_energy
        }
    };

    // vector(dofadr, force) — compression-positive coordinates take the
    // negated force (forces.py:189-192).
    const auto vector = [this](int dofadr, double force_n) {
        std::vector<double> qfrc(static_cast<std::size_t>(nv_), 0.0);
        qfrc[static_cast<std::size_t>(dofadr)] = -force_n;
        return qfrc;
    };

    std::vector<Component> out;
    out.reserve(physical_ ? 8 : 7);
    out.emplace_back("fork_spring", vector(fork_dofadr_, fork_spring));
    out.emplace_back("fork_damper", vector(fork_dofadr_, fork_damper));
    out.emplace_back("shock_coil", vector(shock_dofadr_, shock_spring));
    out.emplace_back("shock_bumper", vector(shock_dofadr_, shock_bumper));
    out.emplace_back("shock_damper",
                     vector(shock_dofadr_, shock_damper - shock_hbo));
    out.emplace_back("shock_top_out", vector(shock_dofadr_, shock_top_out));
    out.emplace_back("shock_upper_stop",
                     vector(shock_dofadr_, shock_upper_stop));
    if (physical_)
        out.emplace_back("shock_hbo", vector(shock_dofadr_, shock_hbo));
    return out;
}
