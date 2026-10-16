// model/suspension_setup.hpp — T15 suspension boundary:
// physics/air_spring.py's ForkAirSpring model, physics/coil_shock.py's
// CoilShock, physics/damper.py's BaseDamper/Charger3Damper/
// SuperDeluxeDamper/BikeSuspensionSystem, and physics/tuning.py's
// static-sag calculators.
//
// Bitwise contract: same rules as model/mass.hpp — the oracle's operation
// order is kept, `x ** y` reaches libm pow() through pyfloat::pow,
// np.linspace/np.cumsum/np.diff keep their distinct accumulation orders,
// builtin sum() goes through numeric::PythonSum, and np.linalg.lstsq's
// DGELSD call goes through rider::LeastSquaresWorkspace (the same
// Accelerate ILP64 entry numpy's gufunc calls, with rcond=None resolving
// to eps * max(m, n) exactly like numpy's default).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../configuration/rider_spec.hpp" // configuration::RiderSpecs
#include "../geometry/specs.hpp"           // geometry::BikeSpecs, Vec3
#include "../kinematics/solver.hpp"        // kinematics::HorstLinkageSolver
#include "mass.hpp"                        // BikeMassSpecs

namespace bike::model {

// ---------------------------------------------------------------------------
// physics/air_spring.py — unit constants (declaration order verbatim).
// ---------------------------------------------------------------------------
inline constexpr double PSI_TO_PA = 6894.757293168;
inline constexpr double PA_TO_PSI = 1.0 / PSI_TO_PA;
inline constexpr double BAR_TO_PA = 100000.0;
inline constexpr double PA_TO_BAR = 1.0 / BAR_TO_PA;
inline constexpr double ATM_PA = 101325.0;

// AirSpringSpecs frozen dataclass — field order verbatim; the
// __post_init__ (domain air_specs()) runs through
// validate_air_spring_specs, which every consumer invokes exactly like
// the oracle (construction and again inside ForkAirSpring.__init__).
struct AirSpringSpecs {
    double stanchion_inner_diam_mm = 24.0;
    double total_travel_mm = 180.0;
    double pos_chamber_length_mm = 280.0;
    double neg_chamber_length_mm = 55.0;
    double token_volume_cm3 = 8.0;
    int max_tokens = 4;
    int default_tokens = 2;
    double gamma = 1.40;
    double atm_pressure_pa = ATM_PA;

    [[nodiscard]] double piston_area_m2() const;
    [[nodiscard]] double token_volume_m3() const;
    [[nodiscard]] double base_pos_volume_m3() const;
    [[nodiscard]] double base_neg_volume_m3() const;
};

// AirSpringSpecs.__post_init__ — domain_validation.air_specs.
void validate_air_spring_specs(const AirSpringSpecs &specs);

// ForkAirSpring.compute_force_curve() result dict — key order verbatim.
struct AirSpringForceCurve {
    std::vector<double> travel_mm;
    std::vector<double> axial_force_n;
    std::vector<double> stiffness_n_mm;
    std::vector<double> pos_pressure_psi;
    std::vector<double> neg_pressure_psi;
    std::vector<double> energy_j;
    double max_force_n;
    double max_energy_j;
    double stiffness_at_0_n_mm;
    double stiffness_at_sag_n_mm;
    double stiffness_at_bottom_n_mm;
    double progressivity_pct;
};

class ForkAirSpring {
public:
    // __init__(specs=None, num_tokens=None, gauge_pressure_psi=82.0):
    // nullopt specs -> AirSpringSpecs{} validated inside.
    explicit ForkAirSpring(
            const AirSpringSpecs &specs,
            std::optional<int> num_tokens = std::nullopt,
            double gauge_pressure_psi = 82.0);
    explicit ForkAirSpring(
            std::optional<int> num_tokens = std::nullopt,
            double gauge_pressure_psi = 82.0);

    [[nodiscard]] const AirSpringSpecs &specs() const noexcept {
        return specs_;
    }
    // Public mutable attributes in the oracle — accessed directly.
    [[nodiscard]] int num_tokens() const noexcept { return num_tokens_; }
    [[nodiscard]] double gauge_pressure_psi() const noexcept {
        return gauge_pressure_psi_;
    }
    void set_gauge_pressure_psi(double v) { gauge_pressure_psi_ = v; }

    [[nodiscard]] double abs_pressure_pa() const;
    [[nodiscard]] double gauge_pressure_bar() const;

    void set_tokens(int n_tokens);
    void set_pressure_psi(double psi);

    // compute_volumes(travel_mm, num_tokens=None) -> (v_pos, v_neg).
    [[nodiscard]] std::pair<double, double>
    compute_volumes(double travel_mm,
                    std::optional<int> num_tokens = std::nullopt) const;
    // compute_pressures_pa(travel_mm, p_gauge_psi=None, num_tokens=None)
    // -> (p_pos, p_neg) absolute Pa.
    [[nodiscard]] std::pair<double, double>
    compute_pressures_pa(double travel_mm,
                         std::optional<double> p_gauge_psi = std::nullopt,
                         std::optional<int> num_tokens = std::nullopt) const;
    // compute_axial_force(travel_mm, p_gauge_psi=None, num_tokens=None).
    [[nodiscard]] double
    compute_axial_force(double travel_mm,
                        std::optional<double> p_gauge_psi = std::nullopt,
                        std::optional<int> num_tokens = std::nullopt) const;
    // compute_instantaneous_stiffness(travel_mm, p_gauge_psi=None,
    // num_tokens=None, delta_mm=0.5).
    [[nodiscard]] double compute_instantaneous_stiffness(
            double travel_mm, std::optional<double> p_gauge_psi = std::nullopt,
            std::optional<int> num_tokens = std::nullopt,
            double delta_mm = 0.5) const;
    // calibrate_psi_for_sag(target_sag_mm=54.0,
    // target_axial_force_n=322.2, num_tokens=None) — sets
    // gauge_pressure_psi and returns it.
    double calibrate_psi_for_sag(double target_sag_mm = 54.0,
                                 double target_axial_force_n = 322.2,
                                 std::optional<int> num_tokens = std::nullopt);
    // compute_force_curve(n_points=101, p_gauge_psi=None,
    // num_tokens=None).
    [[nodiscard]] AirSpringForceCurve
    compute_force_curve(int n_points = 101,
                        std::optional<double> p_gauge_psi = std::nullopt,
                        std::optional<int> num_tokens = std::nullopt) const;
    // fit_polynomial_mjcf(order=3, num_tokens=None, p_gauge_psi=None) —
    // DGELSD lstsq over column_stack([travel_m**i for i in 1..order]).
    [[nodiscard]] std::vector<double>
    fit_polynomial_mjcf(int order = 3,
                        std::optional<int> num_tokens = std::nullopt,
                        std::optional<double> p_gauge_psi = std::nullopt) const;

private:
    AirSpringSpecs specs_;
    int num_tokens_;
    double gauge_pressure_psi_;
};

// ---------------------------------------------------------------------------
// physics/coil_shock.py
// ---------------------------------------------------------------------------
struct CoilShockSpecs {
    double rate_n_m = 114600.0;
    double preload_mm = 0.0;
    double stroke_mm = 65.0;
    double bumper_length_mm = 10.0;
    double bumper_peak_n = 7000.0;

    [[nodiscard]] double bumper_engage_mm() const {
        return stroke_mm - bumper_length_mm;
    }
};
// CoilShockSpecs.__post_init__ — domain_validation.coil_specs.
void validate_coil_shock_specs(const CoilShockSpecs &specs);

class CoilShock {
public:
    explicit CoilShock(const CoilShockSpecs &specs = {},
                       bool legacy_behavior = false);

    [[nodiscard]] const CoilShockSpecs &specs() const noexcept {
        return specs_;
    }
    [[nodiscard]] bool legacy_behavior() const noexcept {
        return legacy_behavior_;
    }
    void set_legacy_behavior(bool v) { legacy_behavior_ = v; }

    [[nodiscard]] double compute_spring_force(double stroke_mm) const;
    [[nodiscard]] double compute_bumper_force(double stroke_mm) const;
    [[nodiscard]] double compute_axial_force(double stroke_mm) const;

private:
    CoilShockSpecs specs_;
    bool legacy_behavior_;
};

// ---------------------------------------------------------------------------
// physics/damper.py — preset enum + click config + damper classes.
// ---------------------------------------------------------------------------

// DamperPreset — member order = enum declaration order.
enum class DamperPreset : std::uint8_t {
    base,   // "Base Tune (Factory Neutral)"
    plush,  // "Plush / Wet Traction"
    enduro, // "Enduro Race (Supportive)"
    park,   // "Bike Park / Jumps (Firm)"
    custom, // "Custom Rider Tune"
};
[[nodiscard]] std::string_view damper_preset_label(DamperPreset p) noexcept;

// DamperClickConfig — field order verbatim, including preset_name.
struct DamperClickConfig {
    int fork_hsc = 2;
    int fork_lsc = 7;
    int fork_rebound = 9;
    int shock_hsc = 2;
    int shock_lsc = 7;
    int shock_rebound = 7;
    int shock_hbo = 2;
    bool shock_lockout = false;
    std::string preset_name = "Base Tune (Factory Neutral)";
};

// DAMPER_PRESETS — contains BASE/PLUSH/ENDURO/PARK only (CUSTOM is
// absent — apply_preset on it is a no-op).
[[nodiscard]] bool damper_preset_has_config(DamperPreset preset) noexcept;
[[nodiscard]] DamperClickConfig damper_preset_config(DamperPreset preset);

// Shared BaseDamper records — the constructor parameters become fields
// (the oracle assigns them verbatim; max_*/c_* bounds/knees are mutable
// attributes re-validated by damper()/damper_core on set_clicks).
struct BaseDamper {
    int max_hsc;
    int max_lsc;
    int max_reb;
    double total_travel_mm;
    int hsc_clicks;
    int lsc_clicks;
    int rebound_clicks;
    double c_lsc_min;
    double c_lsc_max;
    double c_hsc_min;
    double c_hsc_max;
    double c_reb_min;
    double c_reb_max;
    double v_knee_comp;
    double v_knee_reb;

    // _calc_fractions_and_coeffs -> (frac_lsc, frac_hsc, frac_reb,
    // c_lsc, c_hsc, c_reb).
    struct FractionsCoeffs {
        double frac_lsc, frac_hsc, frac_reb;
        double c_lsc, c_hsc, c_reb;
    };
    [[nodiscard]] FractionsCoeffs calc_fractions_and_coeffs() const;
    // _compute_base_damping(v, c_lsc, c_hsc, c_reb).
    [[nodiscard]] double compute_base_damping(double v, double c_lsc,
                                              double c_hsc,
                                              double c_reb) const;
};

// Charger3Damper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=9,
//                total_travel_mm=180.0, *, legacy_behavior=False).
struct Charger3Damper : BaseDamper {
    bool legacy_behavior = false;
    double hbo_start_mm = 0.0;
    double c_hbo_base = 3500.0;

    // The _arg-suffixed parameter names keep GCC -Wshadow from flagging
    // them against the same-named public members.
    explicit Charger3Damper(int hsc_clicks_arg = 2, int lsc_clicks_arg = 7,
                            int rebound_clicks_arg = 9,
                            double total_travel_mm_arg = 180.0,
                            bool legacy_behavior_arg = false);

    // set_clicks(hsc=None, lsc=None, reb=None) — damper(self) validates
    // the CURRENT record first, then clamped integer assignments.
    void set_clicks(std::optional<int> hsc, std::optional<int> lsc,
                    std::optional<int> reb);
    // get_effective_coefficients — {"c_lsc","c_hsc","c_reb","frac_lsc",
    // "frac_hsc","frac_reb"} order.
    [[nodiscard]] std::vector<std::pair<std::string, double>>
    get_effective_coefficients() const;
    // compute_damping_force(velocity_mps, travel_mm=50.0).
    [[nodiscard]] double compute_damping_force(double velocity_mps,
                                               double travel_mm = 50.0) const;
    // compute_dyno_curve(v_max=2.5, n_points=101, travel_mm=90.0).
    struct DynoCurve {
        std::vector<double> velocity_mps;
        std::vector<double> force_n;
        int hsc_clicks;
        int lsc_clicks;
        int rebound_clicks;
    };
    [[nodiscard]] DynoCurve compute_dyno_curve(double v_max = 2.5,
                                               int n_points = 101,
                                               double travel_mm = 90.0) const;
};

// SuperDeluxeDamper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=7,
//                   hbo_clicks=2, lockout_firm=False, total_stroke_mm=65.0,
//                   legacy_behavior=False) — all positional-capable.
struct SuperDeluxeDamper : BaseDamper {
    int max_hbo = 4;
    double total_stroke_mm = 65.0;
    int hbo_clicks = 2;
    bool lockout_firm = false;
    bool legacy_behavior = false;
    double hbo_start_mm = 0.0;
    double c_hbo_min = 4000.0;
    double c_hbo_max = 18000.0;
    double lockout_preload_n = 480.0;
    double lockout_stiffness = 15000.0;

    // Same _arg convention as Charger3Damper (params shadow members).
    explicit SuperDeluxeDamper(int hsc_clicks_arg = 2, int lsc_clicks_arg = 7,
                               int rebound_clicks_arg = 7,
                               int hbo_clicks_arg = 2,
                               bool lockout_firm_arg = false,
                               double total_stroke_mm_arg = 65.0,
                               bool legacy_behavior_arg = false);

    void set_clicks(std::optional<int> hsc, std::optional<int> lsc,
                    std::optional<int> reb, std::optional<int> hbo,
                    std::optional<bool> lockout);
    // toggle_lockout() -> new state.
    bool toggle_lockout();
    // get_effective_coefficients — {"c_lsc","c_hsc","c_reb","c_hbo",
    // "frac_lsc","frac_hsc","frac_reb","frac_hbo","lockout_firm"} order;
    // lockout_firm is the bool recorded as 0.0/1.0 in the numeric map.
    struct Coefficients {
        double c_lsc, c_hsc, c_reb, c_hbo;
        double frac_lsc, frac_hsc, frac_reb, frac_hbo;
        bool lockout_firm;
    };
    [[nodiscard]] Coefficients get_effective_coefficients() const;
    // compute_damping_components(velocity_mps, stroke_mm=20.0) ->
    // {"base_n","hbo_n"}.
    struct Components {
        double base_n;
        double hbo_n;
    };
    [[nodiscard]] Components
    compute_damping_components(double velocity_mps,
                               double stroke_mm = 20.0) const;
    // compute_damping_force(velocity_mps, stroke_mm=20.0).
    [[nodiscard]] double compute_damping_force(double velocity_mps,
                                               double stroke_mm = 20.0) const;
    // _compute_legacy_damping_force(velocity_mps, stroke_mm=20.0).
    [[nodiscard]] double
    compute_legacy_damping_force(double velocity_mps,
                                 double stroke_mm = 20.0) const;
    // compute_dyno_curve(v_max=2.0, n_points=101, stroke_mm=30.0).
    struct DynoCurve {
        std::vector<double> velocity_mps;
        std::vector<double> force_n;
        int hsc_clicks;
        int lsc_clicks;
        int rebound_clicks;
        int hbo_clicks;
        bool lockout;
    };
    [[nodiscard]] DynoCurve compute_dyno_curve(double v_max = 2.0,
                                               int n_points = 101,
                                               double stroke_mm = 30.0) const;
};

// BikeSuspensionSystem(click_config=None, *, fork_travel_mm=180.0,
// shock_stroke_mm=65.0, legacy_behavior=False).
struct BikeSuspensionSystem {
    DamperPreset current_preset = DamperPreset::base;
    DamperClickConfig config;
    Charger3Damper fork_damper;
    SuperDeluxeDamper shock_damper;

    explicit BikeSuspensionSystem(
            const std::optional<DamperClickConfig> &click_config =
                    std::nullopt,
            double fork_travel_mm = 180.0, double shock_stroke_mm = 65.0,
            bool legacy_behavior = false);

    // apply_preset(preset) — a preset absent from DAMPER_PRESETS (custom)
    // returns without touching state.
    void apply_preset(DamperPreset preset);
    // cycle_preset(forward=True) -> the newly applied DamperPreset.
    DamperPreset cycle_preset(bool forward = true);
};

// ---------------------------------------------------------------------------
// physics/tuning.py
// ---------------------------------------------------------------------------

[[nodiscard]] double reflected_shock_mass(double mass_kg,
                                          double leverage_ratio);

// compute_suspension_tuning_for_sag's result dict — merge order:
// total_system_mass_kg, front_load_fraction, then the fork dict keys,
// then the shock dict keys.
struct SuspensionTuning {
    double total_system_mass_kg;
    double front_load_fraction;
    // _compute_fork_tuning keys, order verbatim:
    double target_front_sag_mm;
    double fork_stiffness_n_m;
    double fork_damping_n_s_m;
    int fork_air_tokens;
    double fork_calibrated_psi;
    double fork_air_force_full_travel_n;
    double fork_air_bottom_out_g;
    double fork_air_progressivity_pct;
    double front_spring_force_full_travel_n;
    double front_bottom_out_g;
    // _compute_shock_tuning keys:
    double target_rear_sag_mm;
    double shock_stroke_at_sag_mm;
    double lr_at_sag;
    double shock_stiffness_n_m;
    double shock_damping_n_s_m;
    double rear_spring_force_full_travel_n;
    double rear_bottom_out_g;
};

// compute_suspension_tuning_for_sag(specs=None, mass_specs=None,
// rider_specs=None, rider_mass_kg=None, target_front_sag_pct=30.0,
// target_rear_sag_pct=30.0, front_weight_fraction=None).
// The oracle's default rider is lumped at rider_mass_kg or 80 kg.
[[nodiscard]] SuspensionTuning compute_suspension_tuning_for_sag(
        const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
        const configuration::RiderSpecs *rider_specs = nullptr,
        std::optional<double> rider_mass_kg = std::nullopt,
        double target_front_sag_pct = 30.0, double target_rear_sag_pct = 30.0,
        std::optional<double> front_weight_fraction = std::nullopt);

} // namespace bike::model
