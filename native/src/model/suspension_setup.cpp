// model/suspension_setup.cpp — T15 suspension boundary implementations.
//
// numpy-provider notes kept inline at each site. Scalar `x ** y` routes
// through pyfloat::float_pow (CPython float_pow's complete ladder);
// ndarray `x ** i` inside fit_polynomial_mjcf reproduces the DOUBLE_power
// ufunc's scalar-exponent fastops (1 → x, 2 → x*x, else npy_pow ==
// opaque libm pow). np.linspace goes through
// geometry::detail::np_linspace; np.diff/np.cumsum are sequential by
// construction; np.linalg.lstsq(rcond=None) resolves the cutoff to
// eps * max(m, n) and drives ::rider::LeastSquaresWorkspace's DGELSD call
// — the same Accelerate ILP64 entry numpy's gufunc uses.
#include "suspension_setup.hpp"

#include "../diag.hpp"
#include "../rider/least_squares.hpp"
#include "../writers/pyfloat.hpp"
#include "../numeric_sincos.hpp"
#include "checks.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The curve/tuning records are returned by value deliberately (the
// dict/tuple results the oracle returns); -Wlarge-by-value-copy exists
// to flag by-value *parameters* and clang gives no param/return split —
// same convention as kinematics/solver.cpp.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")

namespace {

namespace gd = bike::geometry::detail;
using bike::model::checks::boolean;
using bike::model::checks::derived;
using bike::model::checks::derived_array;
using bike::model::checks::integer;
using bike::model::checks::scalar;

// Python scalar `x ** y` — CPython float_pow's complete ladder.
[[nodiscard]] inline double py_pow(double base, double exponent) {
    return pyfloat::float_pow(base, exponent);
}
// ndarray `x ** y` — DOUBLE_power's non-fastop lane (npy_pow → libm).
[[nodiscard]] inline double npy_pow(double base, double exponent) {
    return pyfloat::pow(base, exponent);
}

// Python builtin min/max on floats: min(a, b) returns b iff b < a.
[[nodiscard]] inline double py_min(double a, double b) noexcept {
    return b < a ? b : a;
}
[[nodiscard]] inline double py_max(double a, double b) noexcept {
    return b > a ? b : a;
}
// int min/max — the click clamps.
[[nodiscard]] inline int py_imin(int a, int b) noexcept {
    return b < a ? b : a;
}
[[nodiscard]] inline int py_imax(int a, int b) noexcept {
    return b > a ? b : a;
}

// ---------------------------------------------------------------------
// domain_validation.py — the per-record validators, message text kept.
// ---------------------------------------------------------------------
void air_specs_validate(const bike::model::AirSpringSpecs &specs) {
    using bike::model::checks::scalar;
    scalar(specs.stanchion_inner_diam_mm,
           "AirSpringSpecs.stanchion_inner_diam_mm", std::nullopt, true);
    scalar(specs.total_travel_mm, "AirSpringSpecs.total_travel_mm",
           std::nullopt, true);
    scalar(specs.pos_chamber_length_mm,
           "AirSpringSpecs.pos_chamber_length_mm", std::nullopt, true);
    scalar(specs.neg_chamber_length_mm,
           "AirSpringSpecs.neg_chamber_length_mm", std::nullopt, true);
    scalar(specs.gamma, "AirSpringSpecs.gamma", std::nullopt, true);
    scalar(specs.atm_pressure_pa, "AirSpringSpecs.atm_pressure_pa",
           std::nullopt, true);
    scalar(specs.token_volume_cm3, "AirSpringSpecs.token_volume_cm3", 0.0);
    integer(specs.max_tokens, "AirSpringSpecs.max_tokens", 0);
    integer(specs.default_tokens, "AirSpringSpecs.default_tokens");
}

void coil_specs_validate(const bike::model::CoilShockSpecs &specs) {
    scalar(specs.rate_n_m, "CoilShockSpecs.rate_n_m", std::nullopt, true);
    scalar(specs.stroke_mm, "CoilShockSpecs.stroke_mm", std::nullopt, true);
    scalar(specs.bumper_length_mm, "CoilShockSpecs.bumper_length_mm",
           std::nullopt, true);
    scalar(specs.bumper_peak_n, "CoilShockSpecs.bumper_peak_n",
           std::nullopt, true);
    scalar(specs.preload_mm, "CoilShockSpecs.preload_mm", 0.0);
    if (specs.bumper_length_mm > specs.stroke_mm)
        throw std::invalid_argument(
                "CoilShockSpecs.bumper_length_mm: exceeds stroke");
}

// damper_core(damper) — the shared six-band validation; `name` is the
// f"{type(damper).__name__}" interpolation of the leaf class.
void damper_core_validate(const bike::model::BaseDamper &d,
                          std::string_view name) {
    const std::string n(name);
    integer(d.max_hsc, n + ".max_hsc", std::nullopt, true);
    integer(d.max_lsc, n + ".max_lsc", std::nullopt, true);
    integer(d.max_reb, n + ".max_reb", std::nullopt, true);
    integer(d.hsc_clicks, n + ".hsc_clicks");
    integer(d.lsc_clicks, n + ".lsc_clicks");
    integer(d.rebound_clicks, n + ".rebound_clicks");
    // std::array keeps the bound in the type — raw C-array subscripts
    // trip -Wunsafe-buffer-usage.
    const std::array<std::string_view, 3> bands{"lsc", "hsc", "reb"};
    const std::array<double, 3> lows{d.c_lsc_min, d.c_hsc_min,
                                     d.c_reb_min};
    const std::array<double, 3> highs{d.c_lsc_max, d.c_hsc_max,
                                      d.c_reb_max};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::string low = n + ".c_" + std::string(bands[i]) + "_min";
        const std::string high =
                n + ".c_" + std::string(bands[i]) + "_max";
        scalar(lows[i], low, 0.0);
        scalar(highs[i], high, 0.0);
        if (lows[i] > highs[i])
            throw std::invalid_argument(low +
                                        ": damping bounds must be ordered");
    }
    scalar(d.v_knee_comp, n + ".v_knee_comp", std::nullopt, true);
    scalar(d.v_knee_reb, n + ".v_knee_reb", std::nullopt, true);
    scalar(d.total_travel_mm, n + ".total_travel_mm", std::nullopt, true);
}

// damper(damper) — the Charger3 leaf (has c_hbo_base).
void damper_validate(const bike::model::Charger3Damper &d) {
    damper_core_validate(d, "Charger3Damper");
    constexpr std::string_view name = "Charger3Damper";
    boolean(d.legacy_behavior, "Charger3Damper.legacy_behavior");
    const double start =
            scalar(d.hbo_start_mm, "Charger3Damper.hbo_start_mm", 0.0);
    // getattr(damper, 'total_stroke_mm', damper.total_travel_mm) — the
    // Charger3 record carries no total_stroke_mm.
    scalar(d.total_travel_mm, "Charger3Damper.travel", std::nullopt, true);
    if (!d.legacy_behavior && !(start < d.total_travel_mm))
        throw std::invalid_argument(std::string(name) +
                                    ".hbo_start_mm: modern HBO must "
                                    "start within travel");
    scalar(d.c_hbo_base, "Charger3Damper.c_hbo_base", 0.0);
}

// damper(damper) — the SuperDeluxe leaf (has max_hbo/lockout fields).
void damper_validate(const bike::model::SuperDeluxeDamper &d) {
    damper_core_validate(d, "SuperDeluxeDamper");
    boolean(d.legacy_behavior, "SuperDeluxeDamper.legacy_behavior");
    const double start =
            scalar(d.hbo_start_mm, "SuperDeluxeDamper.hbo_start_mm", 0.0);
    scalar(d.total_stroke_mm, "SuperDeluxeDamper.travel", std::nullopt,
           true);
    if (!d.legacy_behavior && !(start < d.total_stroke_mm))
        throw std::invalid_argument(
                "SuperDeluxeDamper.hbo_start_mm: modern HBO must start "
                "within travel");
    integer(d.max_hbo, "SuperDeluxeDamper.max_hbo", std::nullopt, true);
    integer(d.hbo_clicks, "SuperDeluxeDamper.hbo_clicks");
    boolean(d.lockout_firm, "SuperDeluxeDamper.lockout_firm");
    scalar(d.c_hbo_min, "SuperDeluxeDamper.c_hbo_min", 0.0);
    scalar(d.c_hbo_max, "SuperDeluxeDamper.c_hbo_max", 0.0);
    scalar(d.lockout_preload_n, "SuperDeluxeDamper.lockout_preload_n",
           0.0);
    scalar(d.lockout_stiffness, "SuperDeluxeDamper.lockout_stiffness",
           0.0);
    if (d.c_hbo_min > d.c_hbo_max)
        throw std::invalid_argument(
                "SuperDeluxeDamper.c_hbo_min: HBO bounds must be ordered");
}

// BaseDamper.__init__ — assignment in the oracle's statement order, the
// leaf name carried for the trailing damper_core call.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) fixed positional
// Python signature — the diagnostic anchors mid-parameter-list.
bike::model::BaseDamper
make_base_damper(int max_hsc, int max_lsc, int max_reb, int hsc_clicks,
                 int lsc_clicks, int rebound_clicks,
                 double total_travel_mm, double c_lsc_min,
                 double c_lsc_max, double c_hsc_min, double c_hsc_max,
                 double c_reb_min, double c_reb_max, double v_knee_comp,
                 double v_knee_reb, std::string_view leaf_name) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    bike::model::BaseDamper d{};
    d.max_hsc = integer(max_hsc, "BaseDamper.max_hsc", std::nullopt, true);
    d.max_lsc = integer(max_lsc, "BaseDamper.max_lsc", std::nullopt, true);
    d.max_reb = integer(max_reb, "BaseDamper.max_reb", std::nullopt, true);
    d.total_travel_mm =
            scalar(total_travel_mm, "BaseDamper.total_travel_mm",
                   std::nullopt, true);
    d.hsc_clicks = py_imax(0, py_imin(d.max_hsc,
                                     integer(hsc_clicks,
                                             "BaseDamper.hsc_clicks")));
    d.lsc_clicks = py_imax(0, py_imin(d.max_lsc,
                                     integer(lsc_clicks,
                                             "BaseDamper.lsc_clicks")));
    d.rebound_clicks =
            py_imax(0, py_imin(d.max_reb,
                              integer(rebound_clicks,
                                      "BaseDamper.rebound_clicks")));
    d.c_lsc_min = c_lsc_min;
    d.c_lsc_max = c_lsc_max;
    d.c_hsc_min = c_hsc_min;
    d.c_hsc_max = c_hsc_max;
    d.c_reb_min = c_reb_min;
    d.c_reb_max = c_reb_max;
    d.v_knee_comp = v_knee_comp;
    d.v_knee_reb = v_knee_reb;
    damper_core_validate(d, leaf_name);
    return d;
}

} // namespace

namespace bike::model {

// ---------------------------------------------------------------------
// physics/air_spring.py
// ---------------------------------------------------------------------
void validate_air_spring_specs(const AirSpringSpecs &specs) {
    air_specs_validate(specs);
}
void validate_coil_shock_specs(const CoilShockSpecs &specs) {
    coil_specs_validate(specs);
}

double AirSpringSpecs::piston_area_m2() const {
    const double r_m = (stanchion_inner_diam_mm / 2.0) / 1000.0;
    return derived(M_PI * py_pow(r_m, 2.0), "AirSpringSpecs.piston_area_m2");
}
double AirSpringSpecs::token_volume_m3() const {
    return derived(token_volume_cm3 * 1e-6,
                   "AirSpringSpecs.token_volume_m3");
}
double AirSpringSpecs::base_pos_volume_m3() const {
    const double l_pos_m = pos_chamber_length_mm / 1000.0;
    return derived(piston_area_m2() * l_pos_m,
                   "AirSpringSpecs.base_pos_volume_m3");
}
double AirSpringSpecs::base_neg_volume_m3() const {
    const double l_neg_m = neg_chamber_length_mm / 1000.0;
    return derived(piston_area_m2() * l_neg_m,
                   "AirSpringSpecs.base_neg_volume_m3");
}

ForkAirSpring::ForkAirSpring(const AirSpringSpecs &specs,
                             std::optional<int> num_tokens,
                             double gauge_pressure_psi)
        : specs_(specs),
          // The clamp cannot throw on an int input, so hoisting it into
          // the member-init list cannot reorder a failure ahead of
          // air_specs_validate below.
          num_tokens_(py_imax(
                  0, py_imin(specs_.max_tokens,
                             integer(num_tokens.value_or(
                                             specs_.default_tokens),
                                     "ForkAirSpring.num_tokens")))) {
    air_specs_validate(specs_); // __init__'s air_specs(self.specs)
    gauge_pressure_psi_ =
            scalar(gauge_pressure_psi, "ForkAirSpring.gauge_pressure_psi",
                   0.0);
}
ForkAirSpring::ForkAirSpring(std::optional<int> num_tokens,
                             double gauge_pressure_psi)
        : ForkAirSpring(AirSpringSpecs{}, num_tokens, gauge_pressure_psi) {
}

double ForkAirSpring::abs_pressure_pa() const {
    scalar(gauge_pressure_psi_, "ForkAirSpring.gauge_pressure_psi", 0.0);
    return derived(
            gauge_pressure_psi_ * PSI_TO_PA + specs_.atm_pressure_pa,
            "ForkAirSpring.abs_pressure_pa");
}
double ForkAirSpring::gauge_pressure_bar() const {
    scalar(gauge_pressure_psi_, "ForkAirSpring.gauge_pressure_psi", 0.0);
    return derived(gauge_pressure_psi_ * PSI_TO_PA * PA_TO_BAR,
                   "ForkAirSpring.gauge_pressure_bar");
}
void ForkAirSpring::set_tokens(int n_tokens) {
    num_tokens_ = py_imax(
            0, py_imin(specs_.max_tokens,
                       integer(n_tokens, "ForkAirSpring.num_tokens")));
}
void ForkAirSpring::set_pressure_psi(double psi) {
    gauge_pressure_psi_ =
            py_max(10.0, scalar(psi, "ForkAirSpring.gauge_pressure_psi",
                                0.0));
}

std::pair<double, double>
ForkAirSpring::compute_volumes(double travel_mm,
                               std::optional<int> num_tokens) const {
    const int tokens =
            py_imax(0, py_imin(specs_.max_tokens,
                              integer(num_tokens.value_or(num_tokens_),
                                      "ForkAirSpring.num_tokens")));
    const double travel_m =
            py_max(0.0,
                   py_min(specs_.total_travel_mm,
                          scalar(travel_mm, "ForkAirSpring.travel_mm"))) /
            1000.0;
    const double v_pos_0 =
            specs_.base_pos_volume_m3() -
            (tokens * specs_.token_volume_m3());
    const double v_neg_0 = specs_.base_neg_volume_m3();
    const double v_disp = specs_.piston_area_m2() * travel_m;
    derived(v_disp, "ForkAirSpring.displaced_volume");
    derived(v_pos_0, "ForkAirSpring.initial_positive_volume");
    derived(v_neg_0, "ForkAirSpring.initial_negative_volume");
    if (v_pos_0 <= 0. || v_neg_0 <= 0. || v_disp >= v_pos_0)
        throw std::invalid_argument(
                "Pneumatic volume exhausted at travel=" +
                gd::py_fixed(travel_mm, 1) + " mm with " +
                std::to_string(tokens) +
                " tokens (displaced volume " +
                gd::py_fixed(v_disp * 1e6, 1) +
                " cm³ >= initial chamber volume " +
                gd::py_fixed(v_pos_0 * 1e6, 1) + " cm³).");
    const double v_pos = v_pos_0 - v_disp;
    const double v_neg = v_neg_0 + v_disp;
    if (v_pos <= 0. || v_neg <= 0.)
        throw std::invalid_argument(
                "ForkAirSpring.volumes: chamber volumes must be "
                "positive");
    return {derived(v_pos, "ForkAirSpring.positive_volume"),
            derived(v_neg, "ForkAirSpring.negative_volume")};
}

std::pair<double, double>
ForkAirSpring::compute_pressures_pa(
        double travel_mm, std::optional<double> p_gauge_psi,
        std::optional<int> num_tokens) const {
    const int tokens =
            py_imax(0, py_imin(specs_.max_tokens,
                              integer(num_tokens.value_or(num_tokens_),
                                      "ForkAirSpring.num_tokens")));
    const double psi = p_gauge_psi.value_or(gauge_pressure_psi_);
    scalar(psi, "ForkAirSpring.gauge_pressure_psi", 0.0);
    const double p_abs_0 = psi * PSI_TO_PA + specs_.atm_pressure_pa;
    derived(p_abs_0, "ForkAirSpring.absolute_pressure");
    const double v_pos_0 =
            specs_.base_pos_volume_m3() -
            (tokens * specs_.token_volume_m3());
    const double v_neg_0 = specs_.base_neg_volume_m3();
    const auto [v_pos_x, v_neg_x] = compute_volumes(travel_mm, tokens);
    const double gamma = specs_.gamma;
    const double p_pos = p_abs_0 * py_pow(v_pos_0 / v_pos_x, gamma);
    const double p_neg = p_abs_0 * py_pow(v_neg_0 / v_neg_x, gamma);
    return {derived(p_pos, "ForkAirSpring.positive_pressure"),
            derived(p_neg, "ForkAirSpring.negative_pressure")};
}

double ForkAirSpring::compute_axial_force(
        double travel_mm, std::optional<double> p_gauge_psi,
        std::optional<int> num_tokens) const {
    const auto [p_pos, p_neg] =
            compute_pressures_pa(travel_mm, p_gauge_psi, num_tokens);
    const double area = specs_.piston_area_m2();
    const double net_force = area * (p_pos - p_neg);
    return py_max(0.0, derived(net_force, "ForkAirSpring.axial_force"));
}

double ForkAirSpring::compute_instantaneous_stiffness(
        double travel_mm, std::optional<double> p_gauge_psi,
        std::optional<int> num_tokens, double delta_mm) const {
    scalar(delta_mm, "ForkAirSpring.delta_mm", std::nullopt, true);
    const double x = scalar(travel_mm, "ForkAirSpring.travel_mm");
    const double x1 = py_max(0.0, x - delta_mm);
    const double x2 = py_min(specs_.total_travel_mm, x + delta_mm);
    const double f1 =
            compute_axial_force(x1, p_gauge_psi, num_tokens);
    const double f2 =
            compute_axial_force(x2, p_gauge_psi, num_tokens);
    if (x2 == x1)
        throw std::invalid_argument(
                "ForkAirSpring.delta_mm: zero finite difference "
                "interval");
    return derived((f2 - f1) / (x2 - x1), "ForkAirSpring.stiffness");
}

double ForkAirSpring::calibrate_psi_for_sag(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double target_sag_mm, double target_axial_force_n,
        std::optional<int> num_tokens) {
    scalar(target_axial_force_n, "ForkAirSpring.target_axial_force_n",
           0.0);
    const int tokens =
            py_imax(0, py_imin(specs_.max_tokens,
                              integer(num_tokens.value_or(num_tokens_),
                                      "ForkAirSpring.num_tokens")));
    const double v_pos_0 =
            specs_.base_pos_volume_m3() -
            (tokens * specs_.token_volume_m3());
    const double v_neg_0 = specs_.base_neg_volume_m3();
    const auto [v_pos_sag, v_neg_sag] =
            compute_volumes(target_sag_mm, tokens);
    const double gamma = specs_.gamma;
    const double pos_ratio = py_pow(v_pos_0 / v_pos_sag, gamma);
    const double neg_ratio = py_pow(v_neg_0 / v_neg_sag, gamma);
    const double delta_ratio = pos_ratio - neg_ratio;
    derived(delta_ratio, "ForkAirSpring.calibration_ratio");
    if (delta_ratio <= 0.0)
        throw std::invalid_argument(
                "Invalid sag calibration at target_sag=" +
                gd::py_fixed(target_sag_mm, 1) +
                " mm: negative chamber expansion ratio (" +
                gd::py_fixed(neg_ratio, 4) + ") >= positive ratio (" +
                gd::py_fixed(pos_ratio, 4) + ").");
    const double area = specs_.piston_area_m2();
    const double denominator = area * delta_ratio;
    derived(denominator, "ForkAirSpring.calibration_denominator");
    const double p_abs_0_required = target_axial_force_n / denominator;
    derived(p_abs_0_required,
            "ForkAirSpring.calibration_absolute_pressure");
    const double p_gauge_pa = p_abs_0_required - specs_.atm_pressure_pa;
    derived(p_gauge_pa, "ForkAirSpring.calibration_gauge_pressure");
    const double calibrated_psi =
            py_max(10.0, derived(p_gauge_pa * PA_TO_PSI,
                                 "ForkAirSpring.calibration_raw_psi"));
    derived(calibrated_psi, "ForkAirSpring.calibrated_pressure");
    gauge_pressure_psi_ = calibrated_psi;
    return calibrated_psi;
}

AirSpringForceCurve ForkAirSpring::compute_force_curve(
        int n_points, std::optional<double> p_gauge_psi,
        std::optional<int> num_tokens) const {
    const std::vector<double> travel_array =
            gd::np_linspace(0.0, specs_.total_travel_mm,
                            static_cast<std::size_t>(n_points));
    std::vector<double> force_list, stiffness_list, pos_press_list,
            neg_press_list;
    force_list.reserve(travel_array.size());
    stiffness_list.reserve(travel_array.size());
    pos_press_list.reserve(travel_array.size());
    neg_press_list.reserve(travel_array.size());
    for (const double x: travel_array) {
        const double f =
                compute_axial_force(x, p_gauge_psi, num_tokens);
        const double k =
                compute_instantaneous_stiffness(x, p_gauge_psi, num_tokens);
        const auto [p_pos, p_neg] =
                compute_pressures_pa(x, p_gauge_psi, num_tokens);
        force_list.push_back(f);
        stiffness_list.push_back(k);
        pos_press_list.push_back(
                (p_pos - specs_.atm_pressure_pa) * PA_TO_PSI);
        neg_press_list.push_back(
                (p_neg - specs_.atm_pressure_pa) * PA_TO_PSI);
    }
    // travel_m = travel_array / 1000.0 — elementwise.
    std::vector<double> travel_m(travel_array.size());
    for (std::size_t i = 0; i < travel_array.size(); ++i)
        travel_m[i] = travel_array[i] / 1000.0;
    // np.cumsum(np.diff(travel_m) * (force[1:] + force[:-1]) * 0.5) —
    // diff/sub/add per lane, cumsum's sequential accumulate.
    std::vector<double> energy(travel_array.size());
    if (!travel_m.empty()) {
        energy[0] = 0.0;
        double acc = 0.0;
        for (std::size_t i = 1; i < travel_m.size(); ++i) {
            const double d = travel_m[i] - travel_m[i - 1];
            const double s = force_list[i] + force_list[i - 1];
            acc += d * s * 0.5;
            energy[i] = acc;
        }
    }
    derived_array(energy, "ForkAirSpring.energy_j");
    return {.travel_mm = travel_array,
            .axial_force_n = force_list,
            .stiffness_n_mm = stiffness_list,
            .pos_pressure_psi = pos_press_list,
            .neg_pressure_psi = neg_press_list,
            .energy_j = energy,
            .max_force_n = force_list.back(),
            .max_energy_j = energy.back(),
            .stiffness_at_0_n_mm = stiffness_list.front(),
            .stiffness_at_sag_n_mm = compute_instantaneous_stiffness(
                     54.0, p_gauge_psi, num_tokens),
            .stiffness_at_bottom_n_mm = stiffness_list.back(),
            .progressivity_pct = derived(
                    ((stiffness_list.back() - stiffness_list.front()) /
                     py_max(0.1, stiffness_list.front())) *
                            100.0,
                    "ForkAirSpring.progressivity_pct")};
}

std::vector<double> ForkAirSpring::fit_polynomial_mjcf(
        int order, std::optional<int> num_tokens,
        std::optional<double> p_gauge_psi) const {
    const AirSpringForceCurve curve =
            compute_force_curve(101, p_gauge_psi, num_tokens);
    const std::size_t m = curve.travel_mm.size();
    // travel_m = curve["travel_mm"] / 1000.0 — elementwise.
    std::vector<double> travel_m(m);
    for (std::size_t i = 0; i < m; ++i)
        travel_m[i] = curve.travel_mm[i] / 1000.0;
    // A = np.column_stack([travel_m ** i for i in 1..order]) — for
    // order < 1 the list is empty and column_stack raises
    // "need at least one array to concatenate" before lstsq runs.
    if (order < 1)
        throw std::invalid_argument(
                "need at least one array to concatenate");
    // DOUBLE_power scalar fastops: i==1 -> x, i==2 -> x*x, else
    // npy_pow (libm pow).
    std::vector<double> a(m * static_cast<std::size_t>(order));
    for (int j = 0; j < order; ++j) {
        const int exponent = j + 1;
        for (std::size_t i = 0; i < m; ++i) {
            double v = 0.0;
            if (exponent == 1)
                v = travel_m[i];
            else if (exponent == 2)
                v = travel_m[i] * travel_m[i];
            else
                v = npy_pow(travel_m[i],
                            static_cast<double>(exponent));
            a[i * static_cast<std::size_t>(order) +
              static_cast<std::size_t>(j)] = v;
        }
    }
    // np.linalg.lstsq(A, force_n, rcond=None): rcond=None resolves to
    // eps * max(m, n) inside the gufunc — the resolved value is passed
    // to DGELSD here.
    ::rider::LeastSquaresWorkspace ws(
            static_cast<::rider::lapack_int>(m),
            static_cast<::rider::lapack_int>(order), 1);
    const double rcond =
            std::numeric_limits<double>::epsilon() *
            static_cast<double>(std::max(m,
                                         static_cast<std::size_t>(order)));
    const ::rider::LeastSquaresResult result = ws.solve(
            a, static_cast<::rider::lapack_int>(m),
            static_cast<::rider::lapack_int>(order), curve.axial_force_n, 1,
            rcond);
    return result.solution;
}

// ---------------------------------------------------------------------
// physics/coil_shock.py
// ---------------------------------------------------------------------
CoilShock::CoilShock(const CoilShockSpecs &specs, bool legacy_behavior)
        : specs_(specs),
          legacy_behavior_(
                  boolean(legacy_behavior, "CoilShock.legacy_behavior")) {
    coil_specs_validate(specs_);
    const auto &s = specs_;
    if (!(std::isfinite(s.rate_n_m) && std::isfinite(s.preload_mm) &&
          std::isfinite(s.stroke_mm) &&
          std::isfinite(s.bumper_length_mm) &&
          std::isfinite(s.bumper_peak_n)) ||
        s.preload_mm < 0 || s.rate_n_m <= 0 || s.stroke_mm <= 0 ||
        !(0 < s.bumper_length_mm && s.bumper_length_mm <= s.stroke_mm) ||
        s.bumper_peak_n <= 0)
        throw std::invalid_argument("invalid coil shock specifications");
}

double CoilShock::compute_spring_force(double stroke_mm) const {
    coil_specs_validate(specs_);
    boolean(legacy_behavior_, "CoilShock.legacy_behavior");
    double compression_mm =
            scalar(stroke_mm, "CoilShock.stroke_mm") + specs_.preload_mm;
    if (!legacy_behavior_) compression_mm = py_max(0.0, compression_mm);
    return derived(specs_.rate_n_m * compression_mm / 1000.0,
                   "CoilShock.spring_force");
}

double CoilShock::compute_bumper_force(double stroke_mm) const {
    coil_specs_validate(specs_);
    const double excess_mm =
            py_max(0.0, scalar(stroke_mm, "CoilShock.stroke_mm") -
                                specs_.bumper_engage_mm());
    return derived(specs_.bumper_peak_n *
                           py_pow(excess_mm / specs_.bumper_length_mm,
                                  2.0),
                   "CoilShock.bumper_force");
}

double CoilShock::compute_axial_force(double stroke_mm) const {
    return derived(
            compute_spring_force(stroke_mm) +
                    compute_bumper_force(stroke_mm),
            "CoilShock.axial_force");
}

// ---------------------------------------------------------------------
// physics/damper.py — presets.
// ---------------------------------------------------------------------
std::string_view damper_preset_label(DamperPreset p) noexcept {
    switch (p) {
        case DamperPreset::base: return "Base Tune (Factory Neutral)";
        case DamperPreset::plush: return "Plush / Wet Traction";
        case DamperPreset::enduro: return "Enduro Race (Supportive)";
        case DamperPreset::park: return "Bike Park / Jumps (Firm)";
        case DamperPreset::custom: return "Custom Rider Tune";
    }
    return {};
}

bool damper_preset_has_config(DamperPreset preset) noexcept {
    return preset != DamperPreset::custom;
}

DamperClickConfig damper_preset_config(DamperPreset preset) {
    DamperClickConfig cfg;
    switch (preset) {
        case DamperPreset::base:
            break; // the field defaults ARE the base tune
        case DamperPreset::plush:
            cfg = {.fork_hsc = 0,
                   .fork_lsc = 3,
                   .fork_rebound = 5,
                   .shock_hsc = 0,
                   .shock_lsc = 3,
                   .shock_rebound = 4,
                   .shock_hbo = 1,
                   .shock_lockout = false,
                   .preset_name = "Plush / Wet Traction"};
            break;
        case DamperPreset::enduro:
            cfg = {.fork_hsc = 3,
                   .fork_lsc = 10,
                   .fork_rebound = 11,
                   .shock_hsc = 3,
                   .shock_lsc = 10,
                   .shock_rebound = 9,
                   .shock_hbo = 3,
                   .shock_lockout = false,
                   .preset_name = "Enduro Race (Supportive)"};
            break;
        case DamperPreset::park:
            cfg = {.fork_hsc = 4,
                   .fork_lsc = 13,
                   .fork_rebound = 14,
                   .shock_hsc = 4,
                   .shock_lsc = 13,
                   .shock_rebound = 12,
                   .shock_hbo = 4,
                   .shock_lockout = false,
                   .preset_name = "Bike Park / Jumps (Firm)"};
            break;
        case DamperPreset::custom:
            throw std::out_of_range("no DamperClickConfig for CUSTOM");
    }
    return cfg;
}

// ---------------------------------------------------------------------
// BaseDamper — the shared calculators.
// ---------------------------------------------------------------------
BaseDamper::FractionsCoeffs
BaseDamper::calc_fractions_and_coeffs() const {
    const double frac_lsc =
            static_cast<double>(py_imax(0, py_imin(max_lsc, lsc_clicks))) /
            static_cast<double>(max_lsc);
    const double frac_hsc =
            static_cast<double>(py_imax(0, py_imin(max_hsc, hsc_clicks))) /
            static_cast<double>(max_hsc);
    const double frac_reb =
            static_cast<double>(py_imax(
                    0, py_imin(max_reb, rebound_clicks))) /
            static_cast<double>(max_reb);
    const double c_lsc = c_lsc_min + (c_lsc_max - c_lsc_min) *
                                             py_pow(frac_lsc, 1.25);
    const double c_hsc = c_hsc_min + (c_hsc_max - c_hsc_min) *
                                             py_pow(frac_hsc, 1.15);
    const double c_reb = c_reb_min + (c_reb_max - c_reb_min) *
                                             py_pow(frac_reb, 1.20);
    return FractionsCoeffs{.frac_lsc = frac_lsc,
                           .frac_hsc = frac_hsc,
                           .frac_reb = frac_reb,
                           .c_lsc = c_lsc,
                           .c_hsc = c_hsc,
                           .c_reb = c_reb};
}

double BaseDamper::compute_base_damping(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double v, double c_lsc, double c_hsc, double c_reb) const {
    if (v >= 0.0) {
        if (v <= v_knee_comp)
            return c_lsc * v * (1.0 + 0.35 * (v / v_knee_comp));
        const double f_knee = c_lsc * v_knee_comp * 1.35;
        const double v_excess = v - v_knee_comp;
        return f_knee + c_hsc * py_pow(v_excess, 0.88) *
                                py_pow(v_knee_comp, 0.12);
    }
    const double v_abs = -v;
    if (v_abs <= v_knee_reb)
        return -c_reb * v_abs * (1.0 + 0.25 * (v_abs / v_knee_reb));
    const double f_knee = c_reb * v_knee_reb * 1.25;
    const double v_excess = v_abs - v_knee_reb;
    const double c_hs_reb = c_reb * 0.45;
    return -(f_knee + c_hs_reb * py_pow(v_excess, 0.90) *
                             py_pow(v_knee_reb, 0.10));
}

// ---------------------------------------------------------------------
// Charger3Damper
// ---------------------------------------------------------------------
Charger3Damper::Charger3Damper(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        int hsc_clicks_arg, int lsc_clicks_arg, int rebound_clicks_arg,
        double total_travel_mm_arg, bool legacy_behavior_arg)
        : BaseDamper(make_base_damper(
                  4, 14, 17, hsc_clicks_arg, lsc_clicks_arg,
                  rebound_clicks_arg,
                  [&] {
                      scalar(total_travel_mm_arg,
                             "Charger3Damper.total_travel_mm",
                             std::nullopt, true);
                      if (!std::isfinite(total_travel_mm_arg) ||
                          total_travel_mm_arg <= 0.0)
                          throw std::invalid_argument(
                                  "fork travel must be finite and "
                                  "positive");
                      if (!legacy_behavior_arg &&
                          total_travel_mm_arg < 20.0)
                          throw std::invalid_argument(
                                  "fork travel must contain the 20 mm "
                                  "HBO zone");
                      return total_travel_mm_arg;
                  }(),
                  260.0, 1150.0, 180.0, 520.0, 350.0, 1550.0, 0.18, 0.22,
                  "Charger3Damper")),
          legacy_behavior(
                  boolean(legacy_behavior_arg,
                          "Charger3Damper.legacy_behavior")),
          hbo_start_mm(legacy_behavior_arg ? 160.0
                                           : total_travel_mm_arg - 20.0) {
    if (!legacy_behavior_arg &&
        !(0.0 < total_travel_mm_arg - hbo_start_mm &&
          total_travel_mm_arg - hbo_start_mm <= total_travel_mm_arg))
        throw std::invalid_argument(
                "fork HBO zone must be positive and no longer than "
                "travel");
}

void Charger3Damper::set_clicks(std::optional<int> hsc,
                                std::optional<int> lsc,
                                std::optional<int> reb) {
    damper_validate(*this);
    if (hsc.has_value())
        hsc_clicks =
                py_imax(0, py_imin(max_hsc,
                                  integer(*hsc, "set_clicks.hsc")));
    if (lsc.has_value())
        lsc_clicks =
                py_imax(0, py_imin(max_lsc,
                                  integer(*lsc, "set_clicks.lsc")));
    if (reb.has_value())
        rebound_clicks =
                py_imax(0, py_imin(max_reb,
                                  integer(*reb, "set_clicks.reb")));
}

std::vector<std::pair<std::string, double>>
Charger3Damper::get_effective_coefficients() const {
    const FractionsCoeffs fc = calc_fractions_and_coeffs();
    return {{"c_lsc", fc.c_lsc},     {"c_hsc", fc.c_hsc},
            {"c_reb", fc.c_reb},     {"frac_lsc", fc.frac_lsc},
            {"frac_hsc", fc.frac_hsc}, {"frac_reb", fc.frac_reb}};
}

double Charger3Damper::compute_damping_force(double velocity_mps,
                                             double travel_mm) const {
    const double v =
            scalar(velocity_mps, "Charger3Damper.velocity_mps");
    const FractionsCoeffs coeffs = calc_fractions_and_coeffs();
    double f_damp = compute_base_damping(v, coeffs.c_lsc, coeffs.c_hsc,
                                         coeffs.c_reb);
    scalar(travel_mm, "Charger3Damper.travel_mm");
    if (travel_mm > hbo_start_mm && v > 0.0) {
        if (total_travel_mm == hbo_start_mm)
            throw std::invalid_argument(
                    "Charger3Damper.hbo_start_mm: activated zero HBO "
                    "denominator");
        const double raw =
                (travel_mm - hbo_start_mm) /
                (total_travel_mm - hbo_start_mm);
        const double hbo_prog = py_max(0.0, py_min(1.0, raw));
        f_damp += c_hbo_base * py_pow(hbo_prog, 2.0) * v;
    }
    return derived(f_damp, "Charger3Damper.force");
}

Charger3Damper::DynoCurve
Charger3Damper::compute_dyno_curve(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double v_max, int n_points, double travel_mm) const {
    const std::vector<double> vel_arr = gd::np_linspace(
            -v_max, v_max, static_cast<std::size_t>(n_points));
    std::vector<double> force_arr;
    force_arr.reserve(vel_arr.size());
    for (const double v: vel_arr)
        force_arr.push_back(compute_damping_force(v, travel_mm));
    return {.velocity_mps = vel_arr,
            .force_n = std::move(force_arr),
            .hsc_clicks = hsc_clicks,
            .lsc_clicks = lsc_clicks,
            .rebound_clicks = rebound_clicks};
}

// ---------------------------------------------------------------------
// SuperDeluxeDamper
// ---------------------------------------------------------------------
SuperDeluxeDamper::SuperDeluxeDamper(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        int hsc_clicks_arg, int lsc_clicks_arg, int rebound_clicks_arg,
        int hbo_clicks_arg, bool lockout_firm_arg,
        double total_stroke_mm_arg, bool legacy_behavior_arg)
        : BaseDamper(make_base_damper(
                  4, 14, 14, hsc_clicks_arg, lsc_clicks_arg,
                  rebound_clicks_arg,
                  [&] {
                      scalar(total_stroke_mm_arg,
                             "SuperDeluxeDamper.total_stroke_mm",
                             std::nullopt, true);
                      if (!std::isfinite(total_stroke_mm_arg) ||
                          total_stroke_mm_arg <= 0.0)
                          throw std::invalid_argument(
                                  "shock stroke must be finite and "
                                  "positive");
                      return total_stroke_mm_arg;
                  }(),
                  650.0, 2900.0, 450.0, 1450.0, 900.0, 8000.0, 0.16, 0.20,
                  "SuperDeluxeDamper")),
          total_stroke_mm(total_stroke_mm_arg),
          // self.hbo_clicks = the clamped member, not the shadowing
          // parameter; max_hbo is declared earlier so it is already
          // initialized here.
          hbo_clicks(py_imax(0, py_imin(max_hbo,
                                      integer(hbo_clicks_arg,
                                              "BaseDamper.hbo_clicks")))),
          lockout_firm(
                  boolean(lockout_firm_arg,
                          "SuperDeluxeDamper.lockout_firm")),
          legacy_behavior(
                  boolean(legacy_behavior_arg,
                          "SuperDeluxeDamper.legacy_behavior")),
          hbo_start_mm(legacy_behavior_arg ? 52.0
                                           : 0.8 * total_stroke_mm_arg) {
    if (!legacy_behavior_arg &&
        !(0.0 < total_stroke_mm_arg - hbo_start_mm &&
          total_stroke_mm_arg - hbo_start_mm <= total_stroke_mm_arg))
        throw std::invalid_argument(
                "shock HBO zone must be positive and no longer than "
                "stroke");
}

void SuperDeluxeDamper::set_clicks(std::optional<int> hsc,
                                   std::optional<int> lsc,
                                   std::optional<int> reb,
                                   std::optional<int> hbo,
                                   std::optional<bool> lockout) {
    damper_validate(*this);
    if (hsc.has_value())
        hsc_clicks =
                py_imax(0, py_imin(max_hsc,
                                  integer(*hsc, "set_clicks.hsc")));
    if (lsc.has_value())
        lsc_clicks =
                py_imax(0, py_imin(max_lsc,
                                  integer(*lsc, "set_clicks.lsc")));
    if (reb.has_value())
        rebound_clicks =
                py_imax(0, py_imin(max_reb,
                                  integer(*reb, "set_clicks.reb")));
    if (hbo.has_value())
        hbo_clicks =
                py_imax(0, py_imin(max_hbo,
                                  integer(*hbo, "set_clicks.hbo")));
    if (lockout.has_value())
        lockout_firm =
                boolean(*lockout, "SuperDeluxeDamper.lockout_firm");
}

bool SuperDeluxeDamper::toggle_lockout() {
    damper_validate(*this);
    lockout_firm = !lockout_firm;
    return lockout_firm;
}

SuperDeluxeDamper::Coefficients
SuperDeluxeDamper::get_effective_coefficients() const {
    const FractionsCoeffs fc = calc_fractions_and_coeffs();
    const double frac_hbo =
            static_cast<double>(py_imax(0, py_imin(max_hbo, hbo_clicks))) /
            static_cast<double>(max_hbo);
    const double c_hbo = c_hbo_min + (c_hbo_max - c_hbo_min) *
                                             py_pow(frac_hbo, 1.30);
    return {.c_lsc = fc.c_lsc,
            .c_hsc = fc.c_hsc,
            .c_reb = fc.c_reb,
            .c_hbo = c_hbo,
            .frac_lsc = fc.frac_lsc,
            .frac_hsc = fc.frac_hsc,
            .frac_reb = fc.frac_reb,
            .frac_hbo = frac_hbo,
            .lockout_firm = lockout_firm};
}

SuperDeluxeDamper::Components
SuperDeluxeDamper::compute_damping_components(double velocity_mps,
                                              double stroke_mm) const {
    if (legacy_behavior)
        return {.base_n = compute_legacy_damping_force(velocity_mps,
                                                       stroke_mm),
                .hbo_n = 0.0};
    const double v =
            scalar(velocity_mps, "SuperDeluxeDamper.velocity_mps");
    if (!std::isfinite(v) || !std::isfinite(stroke_mm))
        throw std::invalid_argument("non-finite shock damper state");
    const Coefficients coeffs = get_effective_coefficients();
    double f_damp = 0.0;
    if (lockout_firm && v > 0.0) {
        const double knee = 0.03;
        const double low = lockout_stiffness;
        const double high = 1.8 * coeffs.c_hsc;
        f_damp = v < knee ? low * v : low * knee + high * (v - knee);
    } else {
        f_damp = compute_base_damping(v, coeffs.c_lsc, coeffs.c_hsc,
                                      coeffs.c_reb);
    }
    double hbo = 0.0;
    scalar(stroke_mm, "SuperDeluxeDamper.stroke_mm");
    if (stroke_mm > hbo_start_mm && v > 0.0) {
        const double fraction =
                py_min(1.0, (stroke_mm - hbo_start_mm) /
                                    (total_stroke_mm - hbo_start_mm));
        hbo = coeffs.c_hbo * py_pow(fraction, 2.0) * v;
    }
    return {.base_n = derived(f_damp, "SuperDeluxeDamper.base_n"),
            .hbo_n = derived(hbo, "SuperDeluxeDamper.hbo_n")};
}

double SuperDeluxeDamper::compute_damping_force(double velocity_mps,
                                                double stroke_mm) const {
    if (legacy_behavior)
        return compute_legacy_damping_force(velocity_mps, stroke_mm);
    const Components parts =
            compute_damping_components(velocity_mps, stroke_mm);
    return parts.base_n + parts.hbo_n;
}

double SuperDeluxeDamper::compute_legacy_damping_force(
        double velocity_mps, double stroke_mm) const {
    const double v =
            scalar(velocity_mps, "SuperDeluxeDamper.velocity_mps");
    const Coefficients coeffs = get_effective_coefficients();
    if (lockout_firm && v > 0.0) {
        if (v < 0.03)
            return derived(lockout_stiffness * v,
                           "SuperDeluxeDamper.lockout_force");
        return derived(lockout_preload_n +
                               (coeffs.c_hsc * 1.8) * (v - 0.03),
                       "SuperDeluxeDamper.lockout_force");
    }
    double f_damp = compute_base_damping(v, coeffs.c_lsc, coeffs.c_hsc,
                                         coeffs.c_reb);
    scalar(stroke_mm, "SuperDeluxeDamper.stroke_mm");
    if (stroke_mm > hbo_start_mm && v > 0.0) {
        if (total_stroke_mm == hbo_start_mm)
            throw std::invalid_argument(
                    "SuperDeluxeDamper.hbo_start_mm: activated zero HBO "
                    "denominator");
        const double raw =
                (stroke_mm - hbo_start_mm) /
                (total_stroke_mm - hbo_start_mm);
        const double hbo_prog = py_max(0.0, py_min(1.0, raw));
        f_damp += coeffs.c_hbo * py_pow(hbo_prog, 2.0) * v;
    }
    return derived(f_damp, "SuperDeluxeDamper.force");
}

SuperDeluxeDamper::DynoCurve
SuperDeluxeDamper::compute_dyno_curve(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double v_max, int n_points, double stroke_mm) const {
    const std::vector<double> vel_arr = gd::np_linspace(
            -v_max, v_max, static_cast<std::size_t>(n_points));
    std::vector<double> force_arr;
    force_arr.reserve(vel_arr.size());
    for (const double v: vel_arr)
        force_arr.push_back(compute_damping_force(v, stroke_mm));
    return {.velocity_mps = vel_arr,
            .force_n = std::move(force_arr),
            .hsc_clicks = hsc_clicks,
            .lsc_clicks = lsc_clicks,
            .rebound_clicks = rebound_clicks,
            .hbo_clicks = hbo_clicks,
            .lockout = lockout_firm};
}

// ---------------------------------------------------------------------
// BikeSuspensionSystem
// ---------------------------------------------------------------------
BikeSuspensionSystem::BikeSuspensionSystem(
        const std::optional<DamperClickConfig> &click_config,
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double fork_travel_mm, double shock_stroke_mm,
        bool legacy_behavior)
        : config(click_config.value_or(
                  damper_preset_config(DamperPreset::base))),
          fork_damper(config.fork_hsc, config.fork_lsc,
                      config.fork_rebound, fork_travel_mm,
                      legacy_behavior),
          shock_damper(config.shock_hsc, config.shock_lsc,
                       config.shock_rebound, config.shock_hbo,
                       config.shock_lockout, shock_stroke_mm,
                       legacy_behavior) {}

void BikeSuspensionSystem::apply_preset(DamperPreset preset) {
    if (!damper_preset_has_config(preset)) return;
    current_preset = preset;
    config = damper_preset_config(preset);
    fork_damper.set_clicks(config.fork_hsc, config.fork_lsc,
                           config.fork_rebound);
    shock_damper.set_clicks(config.shock_hsc, config.shock_lsc,
                            config.shock_rebound, config.shock_hbo,
                            config.shock_lockout);
}

DamperPreset BikeSuspensionSystem::cycle_preset(bool forward) {
    static constexpr std::array<DamperPreset, 4> kPresetList{
            DamperPreset::base, DamperPreset::plush,
            DamperPreset::enduro, DamperPreset::park};
    int curr_idx = 0;
    for (std::size_t i = 0; i < kPresetList.size(); ++i)
        if (kPresetList[i] == current_preset) {
            curr_idx = static_cast<int>(i);
            break;
        }
    const std::size_t next_idx =
            forward ? (static_cast<std::size_t>(curr_idx) + 1) % 4
                    : (static_cast<std::size_t>(curr_idx) + 3) % 4;
    const DamperPreset new_preset = kPresetList[next_idx];
    apply_preset(new_preset);
    return new_preset;
}

// ---------------------------------------------------------------------
// physics/tuning.py
// ---------------------------------------------------------------------
double reflected_shock_mass(double mass_kg, double leverage_ratio) {
    if (!(std::isfinite(mass_kg) && std::isfinite(leverage_ratio)) ||
        mass_kg <= 0 || leverage_ratio <= 0)
        throw std::invalid_argument(
                "mass and leverage ratio must be finite and positive");
    return mass_kg * py_pow(leverage_ratio, 2.0);
}

SuspensionTuning compute_suspension_tuning_for_sag(
        const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
        const configuration::RiderSpecs *rider_specs,
        std::optional<double> rider_mass_kg,
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double target_front_sag_pct, double target_rear_sag_pct,
        std::optional<double> front_weight_fraction) {
    // rider_specs is None -> the historical standing lumped rider.
    configuration::RiderSpecs default_rider;
    if (rider_specs == nullptr) {
        default_rider.variant = "lumped";
        default_rider.mass_kg = rider_mass_kg.value_or(80.0);
        configuration::detail::post_init(default_rider);
        rider_specs = &default_rider;
    }

    const kinematics::HorstLinkageSolver solver(specs);
    const StaticSystemCg cg_info = compute_static_system_cg(
            specs, mass_specs, rider_specs, &solver);

    const double total_mass = cg_info.total_mass_kg;
    const double g = 9.81;
    const double total_weight_n = total_mass * g;

    const double fwf =
            front_weight_fraction.value_or(cg_info.front_load_pct / 100.0);

    const double f_front_vert = total_weight_n * fwf;
    const double f_rear_vert = total_weight_n * (1.0 - fwf);

    // ---- _compute_fork_tuning ---------------------------------------
    const double theta = gd::np_radians(specs.head_angle_deg);
    const double target_fork_travel_m =
            (target_front_sag_pct / 100.0) * (specs.fork_travel / 1000.0);
    const double f_fork_axial = f_front_vert * numeric::np_sin(theta);
    const double k_fork = f_fork_axial / target_fork_travel_m;

    const double m_sprung_front =
            total_mass * fwf -
            (mass_specs.fork_lowers_mass + mass_specs.front_wheel_mass);
    const double c_crit_front =
            2.0 * std::sqrt(k_fork *
                            (1.0 > m_sprung_front ? 1.0 : m_sprung_front));
    const double c_fork = 0.50 * c_crit_front;

    AirSpringSpecs air_specs;
    air_specs.total_travel_mm = specs.fork_travel;
    air_specs_validate(air_specs); // AirSpringSpecs.__post_init__
    const int tokens = specs.fork_air_tokens;
    ForkAirSpring air_spring(air_specs, tokens);
    const double calibrated_psi = air_spring.calibrate_psi_for_sag(
            target_fork_travel_m * 1000.0, f_fork_axial, tokens);
    const AirSpringForceCurve air_curve =
            air_spring.compute_force_curve(101, std::nullopt, tokens);
    const double air_max_axial_force = air_curve.max_force_n;
    const double air_front_vert_max =
            air_max_axial_force / numeric::np_sin(theta);
    const double air_front_g_bottom = air_front_vert_max / f_front_vert;

    const double f_fork_axial_max =
            k_fork * (specs.fork_travel / 1000.0);
    const double f_front_vert_max = f_fork_axial_max / numeric::np_sin(theta);
    const double front_g_bottom = f_front_vert_max / f_front_vert;

    // ---- _compute_shock_tuning --------------------------------------
    const double target_rear_wheel_travel_mm =
            (target_rear_sag_pct / 100.0) * specs.rear_wheel_travel;
    const kinematics::KinematicState st_sag =
            solver.solve_state_from_wheel_travel(
                    target_rear_wheel_travel_mm);
    const double shock_stroke_sag_m = st_sag.shock_stroke / 1000.0;
    const double lr_at_sag = st_sag.leverage_ratio;

    const double f_shock_sag = f_rear_vert * lr_at_sag;
    const double k_shock = f_shock_sag / shock_stroke_sag_m;

    const double m_sprung_rear =
            total_mass * (1.0 - fwf) -
            (mass_specs.chainstay_mass + mass_specs.seatstay_mass +
             mass_specs.rear_wheel_mass);
    const double m_eff_shock =
            reflected_shock_mass(m_sprung_rear, lr_at_sag);
    const double c_crit_shock =
            2.0 * std::sqrt(k_shock *
                            (1.0 > m_eff_shock ? 1.0 : m_eff_shock));
    const double c_shock = 0.65 * c_crit_shock;

    const kinematics::KinematicState st180 =
            solver.solve_state_from_wheel_travel(specs.rear_wheel_travel);
    const double f_shock_max = k_shock * (st180.shock_stroke / 1000.0);
    const double f_rear_wheel_max = f_shock_max / st180.leverage_ratio;
    const double rear_g_bottom = f_rear_wheel_max / f_rear_vert;

    return {.total_system_mass_kg = total_mass,
            .front_load_fraction = fwf,
            .target_front_sag_mm = target_fork_travel_m * 1000.0,
            .fork_stiffness_n_m = k_fork,
            .fork_damping_n_s_m = c_fork,
            .fork_air_tokens = tokens,
            .fork_calibrated_psi = calibrated_psi,
            .fork_air_force_full_travel_n = air_front_vert_max,
            .fork_air_bottom_out_g = air_front_g_bottom,
            .fork_air_progressivity_pct = air_curve.progressivity_pct,
            .front_spring_force_full_travel_n = f_front_vert_max,
            .front_bottom_out_g = front_g_bottom,
            .target_rear_sag_mm = target_rear_wheel_travel_mm,
            .shock_stroke_at_sag_mm = shock_stroke_sag_m * 1000.0,
            .lr_at_sag = lr_at_sag,
            .shock_stiffness_n_m = k_shock,
            .shock_damping_n_s_m = c_shock,
            .rear_spring_force_full_travel_n = f_rear_wheel_max,
            .rear_bottom_out_g = rear_g_bottom};
}

NATIVE_DIAG_POP

} // namespace bike::model
