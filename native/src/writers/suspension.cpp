// writers/suspension.cpp — expression-for-expression port of the ride-mode
// suspension force path. Every function mirrors the Python source it names;
// comments cite file:line at the time of porting. `a ** b` goes through
// pyfloat::pow so the call reaches libm pow() like CPython float.__pow__
// (see pyfloat.hpp); `max`/`min` map to std::max/std::min, which agree with
// Python's two-argument builtins element-for-element (incl. ±0/NaN cases).
#include "suspension.hpp"
#include "../config_validation.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/suspension.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <new>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string_view>
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

    bike::physics_core::AirSpringParams<double> to_core_air(
            const nativecfg::AirSpringConfig &air) noexcept {
        return {
            .stanchion_inner_diam_mm = air.specs.stanchion_inner_diam_mm,
            .total_travel_mm = air.specs.total_travel_mm,
            .pos_chamber_length_mm = air.specs.pos_chamber_length_mm,
            .neg_chamber_length_mm = air.specs.neg_chamber_length_mm,
            .token_volume_cm3 = air.specs.token_volume_cm3,
            .max_tokens = air.specs.max_tokens,
            .gamma = air.specs.gamma,
            .atm_pressure_pa = air.specs.atm_pressure_pa,
            .num_tokens = air.num_tokens,
            .gauge_pressure_psi = air.gauge_pressure_psi,
        };
    }

    bike::physics_core::DamperCoreParams<double> to_core_damper(
            const nativecfg::DamperCore &in) noexcept {
        return {
            .max_hsc = in.max_hsc,
            .max_lsc = in.max_lsc,
            .max_reb = in.max_reb,
            .hsc_clicks = in.hsc_clicks,
            .lsc_clicks = in.lsc_clicks,
            .rebound_clicks = in.rebound_clicks,
            .c_lsc_min = in.c_lsc_min,
            .c_lsc_max = in.c_lsc_max,
            .c_hsc_min = in.c_hsc_min,
            .c_hsc_max = in.c_hsc_max,
            .c_reb_min = in.c_reb_min,
            .c_reb_max = in.c_reb_max,
            .v_knee_comp = in.v_knee_comp,
            .v_knee_reb = in.v_knee_reb,
        };
    }

    bike::physics_core::SuspensionParams<double> to_core_suspension(
            const nativecfg::SuspensionConfig &cfg) noexcept {
        return {
            .air_spring = to_core_air(cfg.air_spring),
            .fork_damper = {
                .core = to_core_damper(cfg.fork_damper.core),
                .total_travel_mm = cfg.fork_damper.total_travel_mm,
                .hbo_start_mm = cfg.fork_damper.hbo_start_mm,
                .c_hbo_base = cfg.fork_damper.c_hbo_base,
            },
            .shock_damper = {
                .core = to_core_damper(cfg.shock_damper.core),
                .total_stroke_mm = cfg.shock_damper.total_stroke_mm,
                .max_hbo = cfg.shock_damper.max_hbo,
                .hbo_clicks = cfg.shock_damper.hbo_clicks,
                .lockout_firm = static_cast<bike::physics_core::u8>(
                    cfg.shock_damper.lockout_firm ? 1 : 0),
                .hbo_start_mm = cfg.shock_damper.hbo_start_mm,
                .c_hbo_min = cfg.shock_damper.c_hbo_min,
                .c_hbo_max = cfg.shock_damper.c_hbo_max,
                .lockout_preload_n = cfg.shock_damper.lockout_preload_n,
                .lockout_stiffness = cfg.shock_damper.lockout_stiffness,
            },
            .coil = {
                .rate_n_m = cfg.coil.rate_n_m,
                .preload_mm = cfg.coil.preload_mm,
                .stroke_mm = cfg.coil.stroke_mm,
                .bumper_length_mm = cfg.coil.bumper_length_mm,
                .bumper_peak_n = cfg.coil.bumper_peak_n,
            },
            .end_stops = {
                .stiffness_n_m = cfg.end_stops.stiffness_n_m,
                .damping_n_s_m = cfg.end_stops.damping_n_s_m,
            },
        };
    }

    [[noreturn]] void throw_suspension_error(
            bike::physics_core::SuspDiagField field,
            double travel_mm,
            double v_disp,
            double v_pos_0) {
        using bike::physics_core::SuspDiagField;
        switch (field) {
            case SuspDiagField::air_displaced_volume:
                throw std::overflow_error("ForkAirSpring.displaced_volume must be finite");
            case SuspDiagField::air_initial_pos_volume:
                throw std::overflow_error("ForkAirSpring.initial_positive_volume must be finite");
            case SuspDiagField::air_initial_neg_volume:
                throw std::overflow_error("ForkAirSpring.initial_negative_volume must be finite");
            case SuspDiagField::air_volume_exhausted:
                throw std::invalid_argument(std::format(
                    "pneumatic volume exhausted at travel={:.1f} mm (displaced "
                    "volume {:.1f} cm³ >= initial chamber volume {:.1f} cm³)",
                    travel_mm, v_disp * 1e6, v_pos_0 * 1e6));
            case SuspDiagField::air_pos_volume:
                throw std::overflow_error("ForkAirSpring.positive_volume must be finite");
            case SuspDiagField::air_neg_volume:
                throw std::overflow_error("ForkAirSpring.negative_volume must be finite");
            case SuspDiagField::air_abs_pressure:
                throw std::overflow_error("ForkAirSpring.absolute_pressure must be finite");
            case SuspDiagField::air_volumes_nonpositive:
                throw std::invalid_argument("ForkAirSpring.volumes");
            case SuspDiagField::air_pos_pressure:
                throw std::overflow_error("ForkAirSpring.positive_pressure must be finite");
            case SuspDiagField::air_neg_pressure:
                throw std::overflow_error("ForkAirSpring.negative_pressure must be finite");
            case SuspDiagField::air_axial_force:
                throw std::overflow_error("ForkAirSpring.axial_force must be finite");
            case SuspDiagField::charger3_zero_hbo_denom:
                throw std::invalid_argument("Charger3Damper.hbo_start_mm: zero denominator");
            case SuspDiagField::charger3_damper_force:
                throw std::overflow_error("SuspensionWriter.damper_force must be finite");
            case SuspDiagField::coil_spring_force:
                throw std::overflow_error("CoilShock.spring_force must be finite");
            case SuspDiagField::coil_bumper_force:
                throw std::overflow_error("CoilShock.bumper_force must be finite");
            case SuspDiagField::sd_nonfinite_state:
                throw std::invalid_argument("non-finite shock damper state");
            case SuspDiagField::sd_zero_hbo_denom:
                throw std::invalid_argument("SuperDeluxeDamper.hbo_start_mm: zero denominator");
            case SuspDiagField::sd_base_force:
                throw std::overflow_error("SuperDeluxeDamper.base_force must be finite");
            case SuspDiagField::sd_hbo_force:
                throw std::overflow_error("SuperDeluxeDamper.hbo_force must be finite");
            case SuspDiagField::sd_total_force:
                throw std::overflow_error("SuperDeluxeDamper.force must be finite");
            case SuspDiagField::end_stop_invalid_params:
                throw std::invalid_argument("invalid end-stop parameters");
            case SuspDiagField::end_stop_raw_force:
                throw std::overflow_error("end_stop.raw_force must be finite");
            case SuspDiagField::telemetry_force_or_energy:
                throw std::overflow_error("SuspensionWriter.force_or_energy must be finite");
            case SuspDiagField::invalid_damper_clicks_denom:
                throw std::invalid_argument("invalid damper click range");
            case SuspDiagField::none:
                throw std::invalid_argument("suspension evaluation failed");
        }
    }

    // ForkAirSpring.compute_axial_force (air_spring.py:142-154) — delegates
    // to shared physics_core::eval_air_axial_force<double, CpuExactToolkit>.
    double air_axial_force(const nativecfg::AirSpringConfig &air,
                           double travel_mm) {
        double out_force = 0.0;
        double v_disp = 0.0;
        double v_pos_0 = 0.0;
        auto err = bike::physics_core::ErrCode::none;
        auto diag = bike::physics_core::SuspDiagField::none;
        const auto air_params = to_core_air(air);
        const auto st = bike::physics_core::eval_air_axial_force<
            double, bike::physics_core::CpuExactToolkit>(
            &air_params, travel_mm, out_force, v_disp, v_pos_0, err, diag);
        if (st != bike::physics_core::Status::ok) {
            throw_suspension_error(diag, travel_mm, v_disp, v_pos_0);
        }
        return out_force;
    }

    // --- stored-energy helpers (physical_energy.py) ----------------------------

    // gas_chamber_energy (physical_energy.py:12-21) — scalar(positive)
    // gates on all four inputs, then the PV**gamma integral.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror physical_energy.py
    double gas_chamber_energy(double pressure_pa, double initial_volume_m3,
                              double volume_m3, double gamma) {
        const double p = validation::positive(pressure_pa, "absolute pressure");
        const double v0 = validation::positive(initial_volume_m3, "reference volume");
        const double v = validation::positive(volume_m3, "volume");
        const double g = validation::positive(gamma, "polytropic exponent");
        const double ratio_log = std::log(v / v0);
        if (std::abs(g - 1.) < 1e-10) return -p * v0 * ratio_log;
        return p * v0 * std::expm1((1. - g) * ratio_log) / (g - 1.);
    }

    // fork_air_energy (physical_energy.py:24-42): integral of the
    // travel-clamped axial force — NOT air_axial_force's clamped output;
    // above nominal travel the terminal force's linear continuation is
    // added explicitly. The ArithmeticError reaches C++ as
    // std::overflow_error — OverflowError is ArithmeticError's subclass
    // on the Python side (same convention as contact/laws.hpp).
    double fork_air_energy(const nativecfg::AirSpringConfig &air,
                           double travel_m) {
        const double x = validation::finite(travel_m, "fork travel");
        const nativecfg::AirSpringSpec &s = air.specs;
        const double limit = s.total_travel_mm / 1000.;
        const double inside = std::min(std::max(x, 0.), limit);
        const double vp0 = base_pos_volume_m3(s) -
                           air.num_tokens * token_volume_m3(s);
        const double vn0 = base_neg_volume_m3(s);
        const double displacement = piston_area_m2(s) * inside;
        // ForkAirSpring.abs_pressure_pa (air_spring.py:86-90).
        validation::nonnegative(air.gauge_pressure_psi,
                                "ForkAirSpring.gauge_pressure_psi");
        const double p0 = validation::derived(
            air.gauge_pressure_psi * kPsiToPa + s.atm_pressure_pa,
            "ForkAirSpring.abs_pressure_pa");
        double energy = gas_chamber_energy(p0, vp0, vp0 - displacement,
                                           s.gamma);
        energy += gas_chamber_energy(p0, vn0, vn0 + displacement, s.gamma);
        if (x > limit)
            energy += air_axial_force(air, s.total_travel_mm) * (x - limit);
        if (energy < -1e-9 || !std::isfinite(energy))
            throw std::overflow_error("invalid fork elastic energy");
        return std::max(energy, 0.);
    }

    // stored-energy helpers (physical_energy.py) ----------------------------

    // forces.py:67-99 — _resolve_compression_joint, same checks and order.
    std::pair<int, int> resolve_compression_joint(const mjModel *m,
                                                  const std::string &joint_name,
                                                  bool allow_negative_lower) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (jid < 0)
            throw std::invalid_argument(
                "model has no joint '" + joint_name +
                "'; suspension forces cannot be applied");

        // Checked extents everywhere — -Wunsafe-buffer-usage rejects raw
        // indexing into mjModel pointer fields (same pattern as stepper.hpp).
        model_access::require_id(jid, m->njnt, "suspension joint");
        const std::span<const mjtNum> range =
                model_access::readonly_buffer(m->jnt_range, 2 * m->njnt);
        const double lo = range[2 * static_cast<std::size_t>(jid)];
        const double hi = range[2 * static_cast<std::size_t>(jid) + 1];
        const std::span<const mjtBool> limited =
                model_access::readonly_buffer(m->jnt_limited, m->njnt);
        const bool lower_valid =
                std::abs(lo) <= 1e-9 ||
                (allow_negative_lower && joint_name == "shock_stroke" && lo < 0.0);
        if (!limited[static_cast<std::size_t>(jid)] || !lower_valid || hi <= 0.0)
            throw std::invalid_argument(std::format(
                "joint '{}' has range [{:.4f}, {:.4f}]; the suspension force "
                "path requires a compression-positive limited range",
                joint_name, lo, hi));

        const std::span<const int> qposadr =
                model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
        const std::span<const int> dofadr =
                model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
        const int qadr = qposadr[static_cast<std::size_t>(jid)];
        const int vadr = dofadr[static_cast<std::size_t>(jid)];
        // The stored addresses index d->qpos[nq] / d->qvel[nv] on every
        // components() call — a corrupt model table is rejected here, not
        // at the read.
        model_access::require_id(qadr, m->nq, "suspension qpos address");
        model_access::require_id(vadr, m->nv, "suspension dof address");
        return {qadr, vadr};
    }
    // The member-init list runs before the ctor body could, so the typed
    // config is validated in the init expression itself — same pattern as
    // drivetrain::checked_config. validate() accepts physics_mode
    // "physical" only and rejects the removed legacy damper curve.
    nativecfg::SuspensionConfig checked_config(
            nativecfg::SuspensionConfig config) {
        nativecfg::validate(config);
        return config;
    }

    // compute_qfrc_components' dict keys in insertion order — serialized
    // only at the Python-boxing surface; the typed interface carries
    // ForceKind::suspension instead.
    constexpr std::array<std::string_view,
                         SuspensionWriter::kPhysicalComponentCount>
            kComponentNames = {
        "fork_spring", "fork_damper", "shock_coil", "shock_bumper",
        "shock_damper", "shock_top_out", "shock_upper_stop", "shock_hbo"
    };
} // namespace

SuspensionWriter::SuspensionWriter(const mjModel *m,
                                   nativecfg::SuspensionConfig config)
    : cfg_(checked_config(std::move(config))),
      nq_(m->nq),
      nv_(m->nv) {
    // forces.py:49-52 — the physical shock may start below zero
    // (top-out travel).
    std::tie(fork_qposadr_, fork_dofadr_) =
            resolve_compression_joint(m, cfg_.fork_joint, false);
    std::tie(shock_qposadr_, shock_dofadr_) =
            resolve_compression_joint(m, cfg_.shock_joint, true);

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
    // invariant is what survives once hbo_start_mm is projected resolved.
    const nativecfg::Charger3Config &f = cfg_.fork_damper;
    if (!std::isfinite(f.total_travel_mm) || f.total_travel_mm <= 0.0)
        throw std::invalid_argument(
            "fork travel must be finite and positive");
    if (!std::isfinite(f.hbo_start_mm) || f.hbo_start_mm < 0.0)
        throw std::invalid_argument("config.suspension.fork_damper.hbo_start_mm: invalid HBO start");
    if (!(0.0 < f.total_travel_mm - f.hbo_start_mm &&
          f.total_travel_mm - f.hbo_start_mm <= f.total_travel_mm))
        throw std::invalid_argument(
            "fork HBO zone must be positive and no longer than travel");

    // SuperDeluxeDamper.__init__ (damper.py:254-281).
    const nativecfg::SuperDeluxeConfig &h = cfg_.shock_damper;
    if (!std::isfinite(h.total_stroke_mm) || h.total_stroke_mm <= 0.0)
        throw std::invalid_argument(
            "shock stroke must be finite and positive");
    if (!(0.0 < h.total_stroke_mm - h.hbo_start_mm &&
          h.total_stroke_mm - h.hbo_start_mm <= h.total_stroke_mm))
        throw std::invalid_argument(
            "shock HBO zone must be positive and no longer than stroke");

    // R1: the former per-call component nv-vectors are construction-sized
    // members — compute fills this capacity, never resizes.
    components_.resize(kPhysicalComponentCount);
    for (std::vector<double> &v: components_)
        v.assign(static_cast<std::size_t>(nv_), 0.0);
}

// Caller precondition (raw boundary): `d` is the live mjData of the model
// this writer was built on, driven by the single owning Stepper — widths
// are validated, model/data pairing is not derivable from them.
void SuspensionWriter::components_into(const mjData *d,
                                       std::span<ForceComponentView> out) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "suspension compute needs a live mjData");
    if (out.size() != components_.size())
        throw std::invalid_argument("suspension component view count");
    compute_validated(d);
    export_views(out);
}

// The warm-core twin of components_into: the same precondition gates
// report CoreStatus::invalid_input instead of throwing, and residual
// kernel rejections are caught and mapped so no exception escapes —
// noexcept is proved by the catch-all. The kernel's own throw
// construction stays on the failure path only; a valid tick allocates
// nothing.
CoreStatus SuspensionWriter::try_components_into(
        const mjData *d, std::span<ForceComponentView> out) const noexcept {
    if (d == nullptr || out.size() != components_.size())
        return CoreStatus::invalid_input;
    try {
        compute_validated(d);
    } catch (const std::bad_alloc &) {
        return CoreStatus::engine_failure;
    } catch (const engine::EngineFailure &) {
        // EngineFailure derives std::exception — it must be caught before
        // the generic mapping classifies an engine error as bad input.
        return CoreStatus::engine_failure;
    } catch (const std::exception &) {
        return CoreStatus::invalid_input;
    } catch (...) {
        return CoreStatus::engine_failure;
    }
    export_views(out);
    return CoreStatus::ok;
}

std::span<const std::string_view>
SuspensionWriter::component_names() const noexcept {
    return std::span<const std::string_view>(kComponentNames)
            .first(components_.size());
}

void SuspensionWriter::export_views(
        std::span<ForceComponentView> out) const noexcept {
    for (std::size_t i = 0; i < components_.size(); ++i)
        out[i] = {.kind = ForceKind::suspension, .values = components_[i]};
}

// Python-boxed surface: the typed compute plus the named-component copy —
// identical values and insertion order to the original components().
std::vector<SuspensionWriter::Component>
SuspensionWriter::components(const mjData *d) const {
    compute_validated(d);
    std::vector<Component> out;
    out.reserve(components_.size());
    for (std::size_t i = 0; i < components_.size(); ++i)
        out.emplace_back(std::string(kComponentNames[i]), components_[i]);
    return out;
}

// Extracted unchanged from the original components() body — including the
// model-table require_id guards and the derived() overflow gates, whose
// exception types are part of the observable contract. The `vector`
// lambda's zero+negated-write fills the persistent member buffers in the
// same insertion order.
void SuspensionWriter::compute_validated(const mjData *d) const {
    // forces.py:127-131 — SuspensionController.compute_fork_force
    // (controllers.py:32-42). counted spans, not raw indexing
    // (-Wunsafe-buffer-usage).
    const std::span<const mjtNum> qpos =
            model_access::readonly_buffer(d->qpos, nq_);
    const std::span<const mjtNum> qvel =
            model_access::readonly_buffer(d->qvel, nv_);
    model_access::require_id(fork_qposadr_, nq_, "fork qpos address");
    model_access::require_id(fork_dofadr_, nv_, "fork dof address");
    model_access::require_id(shock_qposadr_, nq_, "shock qpos address");
    model_access::require_id(shock_dofadr_, nv_, "shock dof address");
    const double fork_qpos_m =
            qpos[static_cast<std::size_t>(fork_qposadr_)];
    const double fork_velocity_mps =
            qvel[static_cast<std::size_t>(fork_dofadr_)];
    const double shock_qpos_m =
            qpos[static_cast<std::size_t>(shock_qposadr_)];
    const double shock_velocity_mps =
            qvel[static_cast<std::size_t>(shock_dofadr_)];

    bike::physics_core::SuspensionResult<double> res{};
    auto err = bike::physics_core::ErrCode::none;
    const auto susp_params = to_core_suspension(cfg_);
    const auto st = bike::physics_core::eval_suspension<
        double, bike::physics_core::CpuExactToolkit>(
        &susp_params, fork_qpos_m, fork_velocity_mps,
        shock_qpos_m, shock_velocity_mps, res, err);
    if (st != bike::physics_core::Status::ok) {
        throw_suspension_error(
            res.diag_field, fork_qpos_m * 1000.0,
            res.err_v_disp_m3, res.err_v_pos0_m3);
    }
    shock_total_n_ = res.shock_total;

    // vector(dofadr, force) — compression-positive coordinates take the
    // negated force (forces.py:189-192). The member buffer takes the same
    // zero-fill + negated write as the original fresh vector.
    const auto vector = [this](std::size_t slot, // NOLINT(bugprone-easily-swappable-parameters) slot+dofadr order is pinned by the call sites below
                               int dofadr, double force_n) {
        std::vector<double> &qfrc = components_[slot];
        std::ranges::fill(qfrc, 0.0);
        qfrc[static_cast<std::size_t>(dofadr)] = -force_n;
    };

    vector(0, fork_dofadr_, res.fork_spring);
    vector(1, fork_dofadr_, res.fork_damper);
    vector(2, shock_dofadr_, res.shock_spring);
    vector(3, shock_dofadr_, res.shock_bumper);
    vector(4, shock_dofadr_, res.shock_damper - res.shock_hbo);
    vector(5, shock_dofadr_, res.shock_top_out);
    vector(6, shock_dofadr_, res.shock_upper_stop);
    vector(7, shock_dofadr_, res.shock_hbo);
}

// suspension_energy (physical_energy.py:70-89) — the terms dict in its
// insertion order. qpos reads are float() (ungated); only fork_air's
// scalar() gate rejects non-finite travel.
SuspensionStoredTerms SuspensionWriter::stored_terms(const mjData *d) const {
    if (d == nullptr)
        throw std::invalid_argument("suspension stored_terms needs a live mjData");
    const std::span<const mjtNum> qpos = model_access::readonly_buffer(
        d->qpos, nq_);
    const nativecfg::CoilConfig &coil = cfg_.coil;
    const double x = qpos[static_cast<std::size_t>(shock_qposadr_)];
    const double nominal = coil.stroke_mm / 1000.;
    const double bumper_length = coil.bumper_length_mm / 1000.;
    const double bumper_depth =
            std::min(std::max(x - (nominal - bumper_length), 0.),
                     bumper_length);
    const nativecfg::EndStopConfig &stop = cfg_.end_stops;
    const double over = std::max(x - nominal, 0.);
    return {
        .fork_air = fork_air_energy(
            cfg_.air_spring,
            qpos[static_cast<std::size_t>(fork_qposadr_)]),
        .shock_coil = .5 * coil.rate_n_m *
                      pyfloat::pow(
                          std::max(x + coil.preload_mm / 1000., 0.), 2.0),
        .shock_bumper = coil.bumper_peak_n *
                        pyfloat::pow(bumper_depth, 3.0) /
                        (3. * pyfloat::pow(bumper_length, 2.0)),
        .shock_top_out = .5 * stop.stiffness_n_m *
                         pyfloat::pow(std::min(x, 0.), 2.0),
        .shock_upper_stop = coil.bumper_peak_n * over +
                            .5 * stop.stiffness_n_m *
                                pyfloat::pow(over, 2.0),
    };
}
