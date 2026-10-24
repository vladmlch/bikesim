// Ports tests/reference/test_rider_allocation.py — the constrained
// seated-effort allocation contracts. Two records are dropped as
// Python-only seams (D14): test_allocation_forecast_includes_current_
// assembled_support_forces drives a SimpleNamespace interception point
// inside Python's apply_forces that has no native analogue — the ordering
// it asserts (the forecast sees support-assembled acceleration) is
// load-bearing in test_allocated_wrenches_close_the_rider_dynamics, whose
// dynamics residual closes only when the committed forecast acceleration
// was produced with the support wrenches already applied — and
// test_warm_start_from_feasible_point_solves_once counts scipy.optimize
// .minimize calls, an implementation observation the dense native solve
// cannot expose; its behavioral content — a feasible warm start returns the
// cold answer — is covered by test_warm_start_does_not_change_the_answer.
//
// The oracle's __new__ stubs around controller internals map onto the
// public support_rows/select_allocation_branch/allocate_response surfaces;
// where the Python reads an assembled-problem interior (the uncoupled
// saddle-reserve param), the produced bound differs only by the 1e-5 force
// guard — inside the oracle's own pytest.approx tolerance — and is
// asserted with that same tolerance.
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "configuration/physical_config.hpp"
#include "engine_call.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/finalize.hpp"
#include "initialization/setup_owner.hpp"
#include "kinematics/solver.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "numerics/dense.hpp"
#include "research/environment.hpp"
#include "rider/activation_allocation.hpp"
#include "rider/allocation.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/response_allocation.hpp"
#include "rider/rider_posture.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "rider/state_codec.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/profile.hpp"
#include "writers/suspension.hpp"
#include "writer_config.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <ranges>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {
namespace d = bike::numerics::dense;
namespace rider = bike::rider;
namespace engine = ::engine;
namespace cfg = bike::configuration;
namespace init = bike::initialization;
using bike::rider::Matrix;
using Vector = std::vector<double>;
namespace rcfg = ::runtime;

// configuration::AttachmentBudget -> runtime::AttachmentBudget — the two
// mirrors of attachment_budget.py stay distinct like the oracle's config
// and accounting copies.
rcfg::AttachmentBudget attachment_budget() {
    const cfg::ArticulatedConfig config;
    const auto source = config.attachment_budget();
    return {.foot_min_normal_n = source.foot_min_normal_n,
            .foot_mu = source.foot_mu, .saddle_mu = source.saddle_mu,
            .grip_pull_n = source.grip_pull_n,
            .max_gap_m = source.max_gap_m};
}
const rcfg::AttachmentBudget BUDGET = attachment_budget();

bool approx(double actual, double expected, double rtol = 1e-6,
            double atol = 1e-12) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}
void allclose(std::span<const double> actual, std::span<const double> expected,
              double rtol, double atol, std::string_view what) {
    bt::require(actual.size() == expected.size(),
                std::string(what) + ": shape matches");
    for (std::size_t i = 0; i < actual.size(); ++i)
        bt::require(approx(actual[i], expected[i], rtol, atol),
                    std::string(what) + " element " + std::to_string(i));
}

// pytest.raises(..., match=...) — type plus a message fragment.
template <class F>
void throws(F &&action, std::string_view needle, std::string_view message) {
    try {
        std::forward<F>(action)();
    } catch (const std::invalid_argument &failure) {
        bt::require(std::string_view(failure.what()).find(needle) !=
                        std::string_view::npos,
                    std::string(message) + ": unexpected message '" +
                        failure.what() + "'");
        return;
    }
    bt::require(false, std::string(message) + ": no exception was raised");
}

rider::AllocationProblem problem(Vector target, Matrix equality,
                                 Vector equality_rhs, Matrix inequality,
                                 Vector inequality_rhs, Vector lower,
                                 Vector upper,
                                 std::vector<rider::ExtraConstraint> extra = {},
                                 std::optional<Vector> initial = std::nullopt) {
    rider::AllocationProblem out;
    out.target = std::move(target);
    out.equality = std::move(equality);
    out.equality_rhs = std::move(equality_rhs);
    out.inequality = std::move(inequality);
    out.inequality_rhs = std::move(inequality_rhs);
    out.lower = std::move(lower);
    out.upper = std::move(upper);
    out.extra = std::move(extra);
    out.initial = std::move(initial);
    return out;
}

Matrix matrix(std::size_t rows, std::size_t cols, Vector values) {
    return {.rows = rows, .cols = cols, .values = std::move(values)};
}

[[gnu::noinline]] void test_saddle_reserve_is_an_actual_configured_lower_bound() {
    // tests/reference/test_rider_allocation.py::test_saddle_reserve_is_an_actual_configured_lower_bound
    for (const bool coupled : {true, false})
        for (const double reserve : {0., .15, .5}) {
            // x = [qddot/500, tau/50, Fx/300, Fz/300, p/450]. Check the
            // generated inequality, not only a diagnostic echo.
            Matrix force_map = d::zeros(3, 5);
            force_map.values[2] = force_map.values[5 + 3] = 300.;
            cfg::ArticulatedConfig config;
            config.saddle_reserve_weight_fraction = reserve;
            const auto [g, h] = rider::support_rows(
                force_map, {0., 1.}, true, config, 80. * 9.81, .05);
            const double expected = reserve * 80. * 9.81;
            // The oracle asserts -expected-guard with guard=1e-5 coupled /
            // 0 uncoupled; the uncoupled guard lives in the private
            // allocate_uncoupled assembly, which emits the -floor row
            // unguarded. The shared bound is asserted to within the force
            // guard the two assemblies differ by; the coupled param is
            // asserted exactly.
            const Vector row{0., 0., 0., -300., 0.};
            double best = std::numeric_limits<double>::infinity();
            std::size_t selected = 0;
            for (std::size_t i = 0; i < g.rows; ++i) {
                bool match = true;
                for (std::size_t j = 0; j < g.cols; ++j)
                    match = match && approx(g.values[i * g.cols + j], row[j]);
                if (match) {
                    ++selected;
                    best = std::min(best, h[i]);
                }
            }
            bt::require(selected > 0,
                        "a lower-normal bound exists in the support problem");
            if (coupled)
                bt::require(approx(best, -expected - 1e-5, 1e-6, 0.),
                            "the coupled reserve bound carries the force "
                            "guard");
            else
                bt::require(approx(best, -expected, 1e-6, 1.5e-5),
                            "the uncoupled reserve is an actual lower bound "
                            "within the guard slack");
            const double floor =
                config.saddle_reserve_weight_fraction * 80. * 9.81;
            bt::require(approx(floor, expected),
                        "the diagnostic reports the configured reserve");
            const ::rider::AttachmentSample sample{
                .kind = "saddle", .normal_n = 0., .tangent_n = 0.,
                .moment_nm = 0., .gap_m = 0.};
            const auto violations =
                rcfg::attachment_violations(sample, BUDGET);
            bt::require(std::ranges::find(violations, "normal") !=
                            violations.end(),
                        "a zero saddle load violates the normal budget");
        }
}

[[gnu::noinline]] void test_coupled_support_cone_allows_compression_and_rejects_tension() {
    // tests/reference/test_rider_allocation.py::test_coupled_support_cone_allows_compression_and_rejects_tension
    for (const std::string_view name : {"front", "rear", "saddle"}) {
        Matrix force_map = d::zeros(3, 5);
        force_map.values[2] = force_map.values[5 + 3] = 300.;
        cfg::ArticulatedConfig config;
        config.saddle_reserve_weight_fraction = .15;
        const bool saddle = name == "saddle";
        const auto [g, h] = rider::support_rows(
            force_map, {0., 1.}, saddle, config, 80. * 9.81, .05);
        const auto allowed = [&](double fx, double fz) {
            const Vector x{0., 0., fx / 300., fz / 300., 0.};
            const auto value = d::multiply(g, x);
            return std::ranges::all_of(std::views::iota(std::size_t{0},
                                                        value.size()),
                                       [&](std::size_t i) {
                                           return value[i] <= h[i] + 1e-10;
                                       });
        };
        bt::require(allowed(0., 200.),
                    "a compressive supported foot/saddle is feasible");
        bt::require(!allowed(0., -200.), "a tensile support is rejected");
        if (!saddle) {
            bt::require(allowed(.9 * 200. - 1e-4, 200.),
                        "the foot friction cone admits the edge");
            bt::require(!allowed(.901 * 200., 200.),
                        "the foot friction cone rejects beyond the edge");
            bt::require(!allowed(0., 19.99),
                        "the foot minimum normal bound is enforced");
        } else {
            bt::require(allowed(.6 * 200. - 1e-4, 200.),
                        "the saddle friction cone admits the edge");
            bt::require(!allowed(.601 * 200., 200.),
                        "the saddle friction cone rejects beyond the edge");
        }
    }
}

[[gnu::noinline]] void test_finalizer_delivers_allocated_torque_instead_of_rebuilding_raw_wish() {
    // tests/reference/test_rider_allocation.py::test_finalizer_delivers_allocated_torque_instead_of_rebuilding_raw_wish
    // The oracle exercises finalize_effort on a __new__ stub; the delivery
    // contract — the allocated torque survives activation, strength and
    // power shaping unchanged — is the same free-function chain the
    // controller publishes (activation_step -> bounded -> power cap).
    for (const double tau_s : {0., .04}) {
        const Vector state{0.};
        const Vector target{10.};
        const Vector velocity{1.};
        const double gain =
            tau_s == 0. ? 1. : -std::expm1(-.005 / tau_s);
        Vector excitation(gain == 0. ? state
                                     : Vector{(target[0] -
                                               (1. - gain) * state[0]) /
                                                  gain});
        const auto activated = ::spindle::activation_step(
            state, excitation, .005, tau_s);
        auto active = ::spindle::bounded_effort(velocity, activated, 100.,
                                                20., 250.);
        active = ::spindle::limit_positive_power(active, velocity, 450.);
        bt::require(active.size() == 1 && active[0] == 10.,
                    "the finalizer delivers the allocated torque, not the "
                    "rebuilt raw wish");
    }
}

[[gnu::noinline]] void test_support_moment_constraints_reject_cop_outside_the_physical_patch() {
    // tests/reference/test_rider_allocation.py::test_support_moment_constraints_reject_cop_outside_the_physical_patch
    for (const auto &param :
         {std::tuple{false, .05, 100.}, std::tuple{true, .045, 200.}}) {
        const bool saddle = std::get<0>(param);
        const double half = std::get<1>(param);
        const double normal = std::get<2>(param);
        const auto [g, h] = rider::support_rows(
            d::identity(3), {0., 1.}, saddle, cfg::ArticulatedConfig{},
            80. * 9.81, half);
        const auto inside = d::multiply(
            g, Vector{0., normal, half * normal - 1e-5});
        for (std::size_t i = 0; i < inside.size(); ++i)
            bt::require(inside[i] <= h[i] + 1e-12,
                        "a COP inside the physical patch is feasible");
        const auto outside = d::multiply(
            g, Vector{0., normal, half * normal + 1e-3});
        bool rejected = false;
        for (std::size_t i = 0; i < outside.size(); ++i)
            rejected = rejected || outside[i] > h[i];
        bt::require(rejected,
                    "a COP outside the physical patch is rejected");
    }
    const ::rider::AttachmentSample sample{.kind = "foot", .normal_n = 100.,
                                         .tangent_n = 90., .moment_nm = 5.,
                                         .gap_m = 0., .half_patch_m = .05};
    bt::require(rcfg::attachment_violations(sample, BUDGET).empty(),
                "the projected foot sample is inside the attachment budget");
}

[[gnu::noinline]] void test_activation_dynamics_shape_the_wish_before_support_allocation() {
    // tests/reference/test_rider_allocation.py::test_activation_dynamics_shape_the_wish_before_support_allocation
    const Vector state{10.};
    const double expected = 10. + (50. - 10.) * (1. - std::exp(-.005 / .04));
    const auto wished = ::spindle::activation_step(state, Vector{50.}, .005,
                                                   .04);
    allclose(wished, Vector{expected}, 0., 1e-12, "activation-shaped wish");
    rider::ActivationLaw law(state, {1.}, {-50.}, {50.}, .005, .04,
                             std::nullopt);
    bt::require(approx(law.gain(), 1. - std::exp(-.005 / .04), 1e-6, 1e-12),
                "the activation gain matches the first-order law");
    bt::require(state == Vector{10.},
                "shaping the wish does not advance the activation state");
}

[[gnu::noinline]] void test_intent_is_relaxed_but_physical_limit_is_not() {
    // tests/reference/test_rider_allocation.py::test_intent_is_relaxed_but_physical_limit_is_not
    const auto result = rider::allocate(
        problem({100.}, d::zeros(0, 1), {}, matrix(1, 1, {1.}), {80.},
                {0.}, {200.}));
    bt::require(result.feasible, "the intent is relaxed but feasible");
    bt::require(std::abs(result.solution[0] - 80.) < 1e-6,
                "the physical limit is not relaxed");
}

[[gnu::noinline]] void test_infeasible_support_is_not_reported_as_valid_stall() {
    // tests/reference/test_rider_allocation.py::test_infeasible_support_is_not_reported_as_valid_stall
    const auto result = rider::allocate(
        problem({0.}, matrix(1, 1, {1.}), {-1.}, d::zeros(0, 1), {},
                {20.}, {100.}));
    bt::require(!result.feasible,
                "an infeasible support is not a valid stall");
}

[[gnu::noinline]] void test_scaled_grip_uses_a_circle_not_a_componentwise_square() {
    // tests/reference/test_rider_allocation.py::test_scaled_grip_uses_a_circle_not_a_componentwise_square
    const double scale = 300.;
    auto map = d::identity(2);
    for (auto &v : map.values) v *= scale;
    const double inv = 1. / std::sqrt(2.);
    const auto constraints = rider::grip_constraints(
        map, {inv, inv}, true, 300.);
    const auto result = rider::allocate(problem(
        {-250. / scale, -250. / scale}, d::zeros(0, 2), {}, d::zeros(0, 2),
        {}, {-1000. / scale, -1000. / scale}, {1000. / scale, 1000. / scale},
        constraints));
    bt::require(result.feasible, "the diagonal pull inside the circle is "
                                 "feasible");
    const Vector force{scale * result.solution[0],
                       scale * result.solution[1]};
    bt::require(d::norm(force) <= 300., "the pull stays inside the circle");
    bt::require(d::norm(d::add(force, Vector{-250., -250.}, -1.)) > 50.,
                "the circle trims the componentwise-square request");
}

[[gnu::noinline]] void test_diagonal_pull_beyond_the_circle_is_infeasible() {
    // tests/reference/test_rider_allocation.py::test_diagonal_pull_beyond_the_circle_is_infeasible
    const double scale = 300.;
    auto map = d::identity(2);
    for (auto &v : map.values) v *= scale;
    const double inv = 1. / std::sqrt(2.);
    const auto constraints = rider::grip_constraints(
        map, {inv, inv}, true, 300.);
    const auto result = rider::allocate(problem(
        {0., 0.}, d::identity(2), {-250. / scale, -250. / scale},
        d::zeros(0, 2), {}, {-1000. / scale, -1000. / scale},
        {1000. / scale, 1000. / scale}, constraints));
    bt::require(!result.feasible,
                "a (250, 250) N pull exceeds the circular grip budget");
}

[[gnu::noinline]] void test_grip_constraint_validation() {
    // tests/reference/test_rider_allocation.py::test_grip_constraint_validation
    const double inv = 1. / std::sqrt(2.);
    throws([&] { (void)rider::grip_constraints(d::identity(2), {2., 0.},
                                               true, 300.); },
           "unit direction", "a non-unit direction is rejected");
    throws([&] {
        (void)rider::grip_constraints(d::identity(2), {inv, inv}, true, 0.);
    }, "unit direction", "a non-positive grip limit is rejected");
    throws([&] {
        (void)rider::grip_constraints(d::identity(3), {inv, inv}, true,
                                      300.);
    }, "planar grip map", "a non-planar force map is rejected");
}

[[gnu::noinline]] void test_pressing_branch_bounds_only_the_sign() {
    // tests/reference/test_rider_allocation.py::test_pressing_branch_bounds_only_the_sign
    const double scale = 300.;
    auto map = d::identity(2);
    for (auto &v : map.values) v *= scale;
    const double inv = 1. / std::sqrt(2.);
    const auto constraints = rider::grip_constraints(
        map, {inv, inv}, false, 300.);
    const auto result = rider::allocate(problem(
        {400. / scale, 400. / scale}, d::zeros(0, 2), {}, d::zeros(0, 2), {},
        {-1000. / scale, -1000. / scale}, {1000. / scale, 1000. / scale},
        constraints));
    bt::require(result.feasible,
                "pressing magnitude is bounded only by the joint budgets");
    allclose(result.solution, Vector{400. / scale, 400. / scale}, 0., 1e-6,
             "the pressing solution keeps the wish");
}

[[gnu::noinline]] void test_inverse_dynamics_rows_layout_and_sign() {
    // tests/reference/test_rider_allocation.py::test_inverse_dynamics_rows_layout_and_sign
    const Matrix mass = matrix(2, 2, {2., .1, .1, 3.});
    const Matrix actuation = matrix(2, 1, {1., 0.});
    const Matrix support = matrix(2, 2, {.5, 0., 0., .5});
    const Vector bias{9., -4.};
    const Vector known{1., 2.};
    const auto [aeq, beq] =
        rider::inverse_dynamics_rows(mass, actuation, support, bias, known);
    bt::require(aeq.rows == 2 && aeq.cols == 5,
                "the inverse dynamics rows have the (n, n+a+s) layout");
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            bt::require(approx(aeq.values[i * 5 + j],
                               mass.values[i * 2 + j], 1e-7, 0.),
                        "the leading block is the mass matrix");
    for (std::size_t i = 0; i < 2; ++i) {
        bt::require(approx(aeq.values[i * 5 + 2], -actuation.values[i],
                           1e-7, 0.),
                    "the actuation column is negated");
        for (std::size_t j = 0; j < 2; ++j)
            bt::require(approx(aeq.values[i * 5 + 3 + j],
                               -support.values[j * 2 + i], 1e-7, 0.),
                        "the support block is the negated transpose");
    }
    allclose(beq, d::add(Vector{known}, bias, -1.), 1e-7, 0.,
             "the right side is known minus bias");
}

[[gnu::noinline]] void test_allocation_rejects_bad_problem_data() {
    // tests/reference/test_rider_allocation.py::test_allocation_rejects_bad_problem_data
    throws([&] {
        (void)rider::allocate(problem(
            {std::numeric_limits<double>::quiet_NaN()}, d::zeros(0, 1), {},
            d::zeros(0, 1), {}, {0.}, {1.}));
    }, "finite", "a NaN target is rejected");
    throws([&] {
        (void)rider::allocate(problem({0.}, d::zeros(0, 1), {},
                                      d::zeros(0, 1), {}, {2.}, {1.}));
    }, "reversed", "reversed bounds are rejected");
}

[[gnu::noinline]] void test_nonfinite_constraint_eval_reports_infinite_violation() {
    // tests/reference/test_rider_allocation.py::test_nonfinite_constraint_eval_reports_infinite_violation
    rider::ExtraConstraint bad;
    bad.evaluate = [](std::span<const double>) {
        return Vector{std::numeric_limits<double>::quiet_NaN()};
    };
    bad.jacobian = [](std::span<const double>) {
        return matrix(1, 1, {0.});
    };
    bad.lower = {-std::numeric_limits<double>::infinity()};
    bad.upper = {1.};
    const auto result = rider::allocate(problem(
        {0.5}, d::zeros(0, 1), {}, d::zeros(0, 1), {}, {0.}, {1.}, {bad}));
    bt::require(!result.feasible,
                "a nonfinite constraint evaluation is not feasible");
    bt::require(result.violation ==
                    std::numeric_limits<double>::infinity(),
                "the reported violation is infinite");
    bt::require(std::ranges::all_of(
                    result.solution,
                    [](double v) { return std::isfinite(v); }),
                "the reported candidate remains finite");
}

[[gnu::noinline]] void test_positive_power_cap_is_a_sum_not_a_net() {
    // tests/reference/test_rider_allocation.py::test_positive_power_cap_is_a_sum_not_a_net
    // z = [tau(3), p(3)]; p_j >= tau_j*v_j, p_j >= 0, sum(p) <= 450.
    const Vector speeds{2., -1., 1.};
    const std::size_t n = 6;
    auto g = d::zeros(4, n);
    for (std::size_t j = 0; j < 3; ++j) {
        g.values[j * n + j] = speeds[j];
        g.values[j * n + 3 + j] = -1.;
    }
    for (std::size_t j = 0; j < 3; ++j) g.values[3 * n + 3 + j] = 1.;
    Vector target(n, 0.);
    for (std::size_t j = 0; j < 3; ++j)
        target[j] = Vector{400., -350., 100.}[j] / speeds[j];
    Vector lower(n, -std::numeric_limits<double>::infinity()),
        upper(n, std::numeric_limits<double>::infinity());
    for (std::size_t j = 3; j < n; ++j) lower[j] = 0.;
    const auto result =
        rider::allocate(problem(std::move(target), d::zeros(0, n), {},
                                std::move(g), {0., 0., 0., 450.},
                                std::move(lower), std::move(upper)));
    bt::require(result.feasible, "the summed power cap problem is feasible");
    double positive = 0.;
    for (std::size_t j = 0; j < 3; ++j)
        positive += std::max(result.solution[j] * speeds[j], 0.);
    bt::require(positive <= 450. + 1e-6,
                "positive work is capped as a sum");
    bt::require(positive < 500. - 1e-6,
                "the cap bites even though the net power is only 150 W");
}

[[gnu::noinline]] void test_isometric_joint_does_not_divide_by_zero_speed() {
    // tests/reference/test_rider_allocation.py::test_isometric_joint_does_not_divide_by_zero_speed
    const Vector speeds{0., 1.};
    const std::size_t n = 4;
    auto g = d::zeros(3, n);
    g.values[0] = speeds[0];
    g.values[2] = -1.;
    g.values[n + 1] = speeds[1];
    g.values[n + 3] = -1.;
    g.values[2 * n + 2] = 1.;
    g.values[2 * n + 3] = 1.;
    const auto result = rider::allocate(problem(
        {30., 5., 0., 0.}, d::zeros(0, n), {}, std::move(g), {0., 0., 450.},
        {-50., -50., 0., 0.}, {50., 50., 450., 450.}));
    bt::require(result.feasible,
                "a bounded isometric torque is perfectly feasible");
    bt::require(approx(result.solution[0], 30., 0., 1e-6),
                "the isometric torque is delivered");
    bt::require(std::ranges::all_of(
                    result.solution,
                    [](double v) { return std::isfinite(v); }),
                "zero speed never produces a 0/0 power cap");
}

[[gnu::noinline]] void test_solution_is_deterministic_for_identical_inputs() {
    // tests/reference/test_rider_allocation.py::test_solution_is_deterministic_for_identical_inputs
    const auto args = [] {
        return problem({3., -2.}, matrix(1, 2, {1., 1.}), {1.},
                       matrix(1, 2, {1., 0.}), {2.}, {-5., -5.}, {5., 5.});
    };
    const auto first = rider::allocate(args());
    const auto second = rider::allocate(args());
    bt::require(first.feasible && second.feasible,
                "identical inputs are feasible twice");
    bt::require(first.solution == second.solution,
                "identical inputs produce a bitwise identical solution");
}

[[gnu::noinline]] void test_candidate_is_preserved_when_infeasible() {
    // tests/reference/test_rider_allocation.py::test_candidate_is_preserved_when_infeasible
    const auto result = rider::allocate(
        problem({0.}, matrix(1, 1, {1.}), {-1.}, d::zeros(0, 1), {},
                {20.}, {100.}));
    bt::require(!result.feasible, "the wish is infeasible");
    bt::require(result.solution.size() == 1,
                "the infeasible candidate keeps the solver shape");
    bt::require(std::ranges::all_of(
                    result.solution,
                    [](double v) { return std::isfinite(v); }),
                "the infeasible candidate remains finite");
}

[[gnu::noinline]] void test_warm_start_does_not_change_the_answer() {
    // tests/reference/test_rider_allocation.py::test_warm_start_does_not_change_the_answer
    const auto args = [] {
        return problem({100., -20.}, d::zeros(0, 2), {},
                       matrix(1, 2, {1., 0.}), {80.}, {0., -50.},
                       {200., 50.});
    };
    const auto cold = rider::allocate(args());
    auto warm_args = args();
    warm_args.initial = Vector{79., -19.};
    const auto warm = rider::allocate(warm_args);
    bt::require(cold.feasible && warm.feasible,
                "both cold and warm starts are feasible");
    allclose(warm.solution, cold.solution, 0., 1e-7,
             "a warm start does not change the answer");
}

[[gnu::noinline]] void test_invalid_warm_start_is_rejected() {
    // tests/reference/test_rider_allocation.py::test_invalid_warm_start_is_rejected
    throws([&] {
        auto args = problem({1., 1.}, d::zeros(0, 2), {}, d::zeros(0, 2), {},
                            {-1., -1.}, {1., 1.}, {}, Vector{0., 0., 0.});
        (void)rider::allocate(args);
    }, "warm start", "a mistargeted warm start is rejected");
    throws([&] {
        auto args = problem(
            {1., 1.}, d::zeros(0, 2), {}, d::zeros(0, 2), {}, {-1., -1.},
            {1., 1.}, {},
            Vector{std::numeric_limits<double>::quiet_NaN(), 0.});
        (void)rider::allocate(args);
    }, "warm start", "a nonfinite warm start is rejected");
}

[[gnu::noinline]] void test_branch_memory_preserves_cold_ranking_and_full_fallback() {
    // tests/reference/test_rider_allocation.py::test_branch_memory_preserves_cold_ranking_and_full_fallback
    const Vector target{0.};
    const std::array<bool, 2> ff{false, false};
    std::vector<std::array<bool, 2>> calls;
    const auto solve = [&](const std::array<bool, 2> &branch,
                           const std::optional<Vector> &) {
        calls.push_back(branch);
        const double distance =
            branch == std::array<bool, 2>{true, true}      ? 4.
            : branch == std::array<bool, 2>{true, false} ? 3.
            : branch == std::array<bool, 2>{false, true} ? 2.
                                                         : 1.;
        return rider::Allocation{.solution = {distance}, .feasible = true,
                                 .violation = 0.};
    };
    rider::AllocationMemory memory;
    const auto first = rider::select_allocation_branch(target, solve, memory);
    bt::require(calls.size() == 4 && first.branch == ff,
                "the cold sweep visits every branch and ranks by distance");
    calls.clear();
    const auto again = rider::select_allocation_branch(target, solve, memory);
    bt::require(calls == std::vector<std::array<bool, 2>>{ff} &&
                    again.branch == ff,
                "the remembered feasible branch short-circuits the sweep");
    calls.clear();
    const auto switched = [&](const std::array<bool, 2> &branch,
                              const std::optional<Vector> &x0) {
        auto result = solve(branch, x0);
        if (branch == ff)
            return rider::Allocation{.solution = result.solution,
                                     .feasible = false, .violation = 1.};
        return result;
    };
    const auto remembered =
        rider::select_allocation_branch(target, switched, memory);
    bt::require(calls.size() == 4,
                "an infeasible remembered branch falls back to the full "
                "sweep");
    rider::AllocationMemory cold;
    calls.clear();
    const auto cold_result =
        rider::select_allocation_branch(target, switched, cold);
    bt::require(calls.size() == 4 &&
                    remembered.branch == cold_result.branch &&
                    cold_result.branch == std::array<bool, 2>{false, true},
                "the fallback ranks the cold candidates identically");
}

[[gnu::noinline]] void test_vector_nonlinear_bounds_and_extra_equalities_keep_their_semantics() {
    // tests/reference/test_rider_allocation.py::test_vector_nonlinear_bounds_and_extra_equalities_keep_their_semantics
    std::vector<rider::ExtraConstraint> extra;
    extra.push_back(rider::linear_constraint(matrix(1, 2, {1., 1.}), {1.},
                                             {1.}));
    rider::ExtraConstraint identity;
    identity.evaluate = [](std::span<const double> x) {
        return Vector(x.begin(), x.end());
    };
    identity.jacobian = [](std::span<const double>) {
        return d::identity(2);
    };
    identity.lower = {-std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<double>::infinity()};
    identity.upper = {.8, .8};
    extra.push_back(std::move(identity));
    const auto result = rider::allocate(problem(
        {2., 0.}, d::zeros(0, 2), {}, d::zeros(0, 2), {}, {0., 0.},
        {1., 1.}, std::move(extra)));
    bt::require(result.feasible,
                "vector bounds and extra equalities stay feasible");
    allclose(result.solution, Vector{.8, .2}, 0., 1e-7,
             "the vector bound and the extra equality keep their semantics");
}

[[gnu::noinline]] void test_nonlinear_constraint_is_evaluated_at_the_bounded_start() {
    // tests/reference/test_rider_allocation.py::test_nonlinear_constraint_is_evaluated_at_the_bounded_start
    for (const std::optional<Vector> &x0 :
         {std::optional<Vector>{std::nullopt}, std::optional<Vector>{Vector{1.}}}) {
        rider::ExtraConstraint log_bound;
        log_bound.evaluate = [](std::span<const double> x) {
            return Vector{std::log(x[0])};
        };
        log_bound.jacobian = [](std::span<const double> x) {
            return matrix(1, 1, {1. / x[0]});
        };
        log_bound.lower = {-std::numeric_limits<double>::infinity()};
        log_bound.upper = {1.};
        const auto result = rider::allocate(problem(
            {-5.}, d::zeros(0, 1), {}, d::zeros(0, 1), {}, {.1}, {2.},
            {std::move(log_bound)}, x0));
        bt::require(result.feasible,
                    "the bounded start keeps the domain-narrowed "
                    "constraint evaluable");
        allclose(result.solution, Vector{.1}, 0., 1e-7,
                 "the clipped start never exposes an out-of-domain target");
    }
}

[[gnu::noinline]] void test_real_grip_problem_changes_from_remembered_pull_to_press() {
    // tests/reference/test_rider_allocation.py::test_real_grip_problem_changes_from_remembered_pull_to_press
    rider::AllocationMemory memory;
    std::vector<std::array<bool, 2>> calls;
    const auto run = [&](const Vector &target, bool prescribed) {
        const auto solve = [&](const std::array<bool, 2> &branch,
                               const std::optional<Vector> &x0) {
            calls.push_back(branch);
            std::vector<rider::ExtraConstraint> extra;
            for (std::size_t i = 0; i < 2; ++i) {
                Matrix map = d::zeros(2, 2);
                map.values[i] = 1.;
                auto row = rider::grip_constraints(
                    map, {1., 0.}, branch[i], 300.);
                extra.insert(extra.end(),
                             std::make_move_iterator(row.begin()),
                             std::make_move_iterator(row.end()));
            }
            return rider::allocate(problem(
                target, prescribed ? d::identity(2) : d::zeros(0, 2),
                prescribed ? target : Vector{}, d::zeros(0, 2), {},
                {-1000., -1000.}, {1000., 1000.}, std::move(extra), x0));
        };
        return rider::select_allocation_branch(target, solve, memory);
    };
    const auto first = run({-100., -100.}, false);
    bt::require(first.allocation.feasible &&
                    first.branch == std::array<bool, 2>{true, true} &&
                    calls.size() == 4,
                "the pull wish keeps the remembered-free pulling branch");
    calls.clear();
    const auto second = run({100., 100.}, true);
    bt::require(second.allocation.feasible &&
                    second.branch == std::array<bool, 2>{false, false} &&
                    calls.size() == 4,
                "the pressing wish falls back from the remembered pull to "
                "press");
    rider::AllocationMemory cold;
    calls.clear();
    const auto cold_result = [&] {
        const auto solve = [&](const std::array<bool, 2> &branch,
                               const std::optional<Vector> &x0) {
            calls.push_back(branch);
            std::vector<rider::ExtraConstraint> extra;
            for (std::size_t i = 0; i < 2; ++i) {
                Matrix map = d::zeros(2, 2);
                map.values[i] = 1.;
                auto row = rider::grip_constraints(
                    map, {1., 0.}, branch[i], 300.);
                extra.insert(extra.end(),
                             std::make_move_iterator(row.begin()),
                             std::make_move_iterator(row.end()));
            }
            return rider::allocate(problem(
                Vector{100., 100.}, d::identity(2), {100., 100.},
                d::zeros(0, 2), {}, {-1000., -1000.}, {1000., 1000.},
                std::move(extra), x0));
        };
        return rider::select_allocation_branch(Vector{100., 100.}, solve,
                                               cold);
    }();
    bt::require(cold_result.branch == second.branch &&
                    cold_result.allocation.feasible,
                "the cold sweep reaches the same branch the memory did");
    allclose(second.allocation.solution, cold_result.allocation.solution,
             0., 1e-7, "the cold solution matches the remembered solution");
}

// ---- adapter: whole-rider balance on the compiled plant ------------------
// The review_rig setup seam from test_t23.cpp, with the batch's all-welded
// attachment mutation (the oracle's _welded_config replacement).

struct WeldedRig {
    std::unique_ptr<init::SetupRequest> request;
    std::unique_ptr<rcfg::RuntimeConfig> config;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<rcfg::PhysicalStep> physical;
};

// NOLINTBEGIN(modernize-make-unique) same inlined-forwarding frame budget as review_rig.
std::unique_ptr<WeldedRig> welded_rig() {
    auto rig = std::make_unique<WeldedRig>();
    // skip_initial_state=true: the shipped welded_flat.json was captured
    // against the fixture's own spindle/pin/connect physics, so its
    // contract cannot accept the weld/weld/weld override — the rig runs
    // the owner's settle_context refinement like the oracle's
    // make_environment does for the rewritten config.
    rig->request = std::unique_ptr<init::SetupRequest>{new init::SetupRequest(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml"),
                                 true))};
    auto &model = rig->request->model;
    auto &physics = model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) the request schema engages physics
    physics.articulated.pedal_attachment = "weld";
    physics.articulated.saddle_attachment = "weld";
    physics.articulated.grip_attachment = "weld";
    cfg::post_init(physics);
    // setup_owner.cpp make_setup: the resolved rider document is what the
    // captured initial-state contract was hashed against, and crank_joint
    // is a compiled-model contract field.
    const auto resolved_rider = bike::model::resolve_rider(
        model.rider ? bike::model::RiderArg{*model.rider}
                    : bike::model::RiderArg{std::monostate{}},
        model.include_rider, "lumped");
    model.rider = resolved_rider;
    model.crank_joint = true;
    auto compiled = bike::model::compile_mjcf(
        bike::model::generate_mjcf(model), rig->request->asset_root);
    const auto field = static_cast<std::size_t>(
        bike::model::required_id(*compiled, mjOBJ_HFIELD, "road"));
    const auto address = std::views::counted(compiled->hfield_adr,
                                             compiled->nhfield)[field];
    const auto samples =
        bike::terrain::build_field_data(rig->request->track, model.field);
    bt::require(address >= 0 &&
                    std::cmp_less_equal(
                        address, compiled->nhfielddata) &&
                    samples.size() <=
                        static_cast<std::size_t>(compiled->nhfielddata -
                                                 address),
                "the welded fixture has valid compiled road storage");
    const auto heightfield = std::views::counted(
        compiled->hfield_data, compiled->nhfielddata)
                                 .subspan(static_cast<std::size_t>(address),
                                          samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
        heightfield[i] = static_cast<float>(samples[i]);
    rig->config = std::unique_ptr<rcfg::RuntimeConfig>{
        new rcfg::RuntimeConfig(
            rcfg::project_runtime(model, *compiled, rig->request->track))};
    const auto image = init::capture_model_image(*compiled);
    const auto writers = std::unique_ptr<nativecfg::WriterConfig>{
        new nativecfg::WriterConfig(
            nativecfg::decode(rig->config->writer_config))};
    rig->stepper = std::make_unique<Stepper>(image.mjb, *writers);
    rig->physical =
        rcfg::make_physical_step(*rig->stepper, *rig->config);
    init::SetupContext context{.stepper = *rig->stepper,
                               .physical = *rig->physical,
                               .physics = physics};
    context.start_x_m = rig->request->start_x_m;
    context.ground_z_m =
        bike::geometry::compute_ground_z(model.bike) / 1000.;
    context.frame_body_id =
        bike::model::required_id(*compiled, mjOBJ_BODY, "frame");
    context.front_geom_id =
        bike::model::required_id(*compiled, mjOBJ_GEOM,
                                 "geom_front_contact");
    context.rear_geom_id = bike::model::required_id(
        *compiled, mjOBJ_GEOM, "geom_rear_contact");
    if (rig->physical->rider_present())
        context.rider_pose =
            ::rider::decode_spindle_pose(rig->config->geometry.pose,
                                       "welded.pose");
    // The rest of SetupOwner::make_setup's context wiring the settle needs
    // — the linkage solver backs the EquilibriumResult's rear-travel
    // evidence (equilibrium.cpp:85).
    auto solver =
        model.solver
            ? std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  *model.solver)
            : std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  model.bike);
    context.rear_travel_from_shock_mm =
        [solver = std::move(solver)](double stroke) {
            return solver->solve_state_from_shock_stroke(stroke).wheel_travel;
        };
    // The owner settle (setup_owner.cpp:222-231) in bounded form: the
    // captured welded_flat.json contracts the fixture's spindle/pin/
    // connect physics — different joint topology from the weld/weld/weld
    // override — so no initial_state transfer applies, and the unbounded
    // 80 s/dt refinement solve does not fit a unit test. The rig poses,
    // measures the honest residual, and finalizes — the bookkeeping the
    // running loop needs (initializing cleared, clocks/latches reset) so
    // control ticks hold between ticks like the oracle's PhysicalRuntime.
    // The tests' own env-step warmups then carry the plant to its working
    // point through the same physics the oracle steps.
    init::initialize_pose(context);
    init::EquilibriumResult result;
    result.residual_qacc = init::acceleration_residual(context);
    const auto settled_qpos = rig->stepper->qpos();
    result.fork_travel_mm =
        1000. * settled_qpos[static_cast<std::size_t>(
            rig->stepper->suspension().fork_qposadr())];
    result.shock_stroke_mm =
        1000. * settled_qpos[static_cast<std::size_t>(
            rig->stepper->suspension().shock_qposadr())];
    result.rear_travel_mm =
        context.rear_travel_from_shock_mm(result.shock_stroke_mm);
    result.root_z_m = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "root_z").qpos)];
    result.pitch_rad = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "root_pitch").qpos)];
    result.initial_crank_phase_rad = physics.drive.crank_phase_rad;
    result.settled_crank_phase_rad = settled_qpos[static_cast<std::size_t>(
        init::joint_address(context, "crank_spin").qpos)];
    context.equilibrium = result;
    init::finalize_initial_state(context, *rig->config);
    context.initializing = false;
    context.completed = true;
    return rig;
}
// NOLINTEND(modernize-make-unique)

rcfg::RideControl idle_control() {
    rcfg::RideControl control;
    control.motor_torque_nm = 0.;
    control.human_torque_nm = 0.;
    return control;
}

// env.step() advances `control_steps` physics intervals per call — the
// RESEARCH control period (ExperimentConfig.control_period_s = .01 s,
// environment.py:130), not the physical control clock's .005 s period.
// advance_physics is the single-interval entry point, so the loops map to
// count * round(.01 / timestep_s) interval advances.
rcfg::RawStep advance_env_steps(rcfg::PhysicalStep &physical,
                                std::int64_t count) {
    const auto per = static_cast<std::int64_t>(std::llround(
        bike::research::ExperimentConfig{}.control_period_s /
        physical.control_clock().timestep_s));
    rcfg::RawStep last;
    for (std::int64_t i = 0; i < count * per; ++i)
        last = physical.advance_physics(1., 1., std::nullopt,
                                        idle_control());
    return last;
}

const runtime::Wire *find(const runtime::WireObject &object,
                          std::string_view key) {
    for (const auto &[name, value] : object)
        if (name == key) return &value;
    return nullptr;
}
// period_buffer's RawStep keeps the emission order of the Python
// diagnostics dict; accounting publishes those same entries as the
// interval's channels — the oracle reads sample.channels[...] against the
// identical wire tree.
const runtime::WireObject &channel(const rcfg::RawStep &step,
                                   std::string_view name) {
    const auto *entry = find(step.diagnostics, name);
    bt::require(entry != nullptr,
                "the interval publishes the " + std::string(name) +
                    " channel");
    return std::get<runtime::WireObject>(entry->value);
}
double channel_real(const runtime::WireObject &object,
                    std::string_view key) {
    const auto *entry = find(object, key);
    bt::require(entry != nullptr,
                "channel key present: " + std::string(key));
    const auto *value = std::get_if<double>(&entry->value);
    bt::require(value != nullptr,
                "channel key is a real: " + std::string(key));
    return *value;
}
bool channel_bool(const runtime::WireObject &object, std::string_view key) {
    const auto *entry = find(object, key);
    bt::require(entry != nullptr,
                "channel key present: " + std::string(key));
    const auto *value = std::get_if<bool>(&entry->value);
    bt::require(value != nullptr,
                "channel key is a bool: " + std::string(key));
    return *value;
}
Vector channel_array(const runtime::WireObject &object,
                     std::string_view key) {
    const auto *entry = find(object, key);
    bt::require(entry != nullptr,
                "channel key present: " + std::string(key));
    const auto *items = std::get_if<runtime::WireArray>(&entry->value);
    bt::require(items != nullptr,
                "channel key is an array: " + std::string(key));
    Vector out;
    out.reserve(items->size());
    for (const auto &item : *items)
        out.push_back(std::get<double>(item.value));
    return out;
}

[[gnu::noinline]] void test_healthy_step_allocates_a_feasible_command() {
    // tests/reference/test_rider_allocation.py::test_healthy_step_allocates_a_feasible_command
    auto rig = welded_rig();
    const rcfg::RawStep last = advance_env_steps(*rig->physical, 40);
    const auto &allocation = channel(last, "rider_allocation");
    bt::require(channel_bool(allocation, "feasible"),
                "a healthy step allocates a feasible command");
    bt::require(!last.invalid_controller,
                "'rider_controller.infeasible' never reaches the step "
                "violations");
    bt::require(std::isfinite(channel_real(allocation, "violation")),
                "the reported violation is finite");
}

// The five attachment resolutions the welded LegacyController builds from
// the same name/eq tables, reproduced at the model surface.
std::vector<rider::AllocationAttachment>
welded_attachments(const mjModel &model) {
    const std::array<std::string, 5> names{"saddle", "front", "rear",
                                           "grip_left", "grip_right"};
    const std::array<std::string, 5> eqs{"weld_saddle", "weld_foot_front",
                                         "weld_foot_rear",
                                         "connect_grip_left",
                                         "connect_grip_right"};
    const std::array<std::string, 3> sites{
        "site_rider_saddle", "site_rider_sole_front", "site_rider_sole_rear"};
    const auto eq_obj1id = model_access::readonly_buffer(
        model.eq_obj1id, model.neq, "eq_obj1id");
    const auto eq_obj2id = model_access::readonly_buffer(
        model.eq_obj2id, model.neq, "eq_obj2id");
    const auto eq_type = model_access::readonly_buffer(
        model.eq_type, model.neq, "eq_type");
    const auto eq_data = model_access::readonly_buffer(
        model.eq_data, model.neq * mjNEQDATA, "eq_data");
    const auto geom_bodyid = model_access::readonly_buffer(
        model.geom_bodyid, model.ngeom, "geom_bodyid");
    const std::array<std::string, 3> geoms{"geom_saddle", "geom_pedal_front",
                                           "geom_pedal_rear"};
    std::vector<rider::AllocationAttachment> out;
    for (std::size_t k = 0; k < names.size(); ++k) {
        rider::AllocationAttachment a;
        a.name = names[k];
        a.equality_id = mj_name2id(&model, mjOBJ_EQUALITY, eqs[k].c_str());
        if (k == 0 && a.equality_id < 0)
            a.equality_id = mj_name2id(&model, mjOBJ_EQUALITY,
                                       "connect_saddle");
        if (k < 3) {
            a.site_id = mj_name2id(&model, mjOBJ_SITE, sites[k].c_str());
            a.geom_id = mj_name2id(&model, mjOBJ_GEOM, geoms[k].c_str());
        }
        if (a.equality_id >= 0) {
            const auto eq = static_cast<std::size_t>(a.equality_id);
            a.rider_body = eq_obj1id[eq];
            a.other_body = eq_obj2id[eq];
            a.weld = eq_type[eq] == mjEQ_WELD;
            std::ranges::copy(eq_data.subspan(eq * mjNEQDATA, 3),
                              a.anchor_local.begin());
        } else if (k < 3) {
            // legacy_controller.cpp:161 — the non-equality branch resolves
            // the rider body from the support geometry side instead.
            bt::require(a.geom_id >= 0 && a.site_id >= 0,
                        "the welded plant resolves rider support geometry");
            a.rider_body = mj_name2id(
                &model, mjOBJ_BODY,
                k == 0 ? "rider_pelvis"
                       : (k == 1 ? "rider_foot_front" : "rider_foot_rear"));
            a.other_body = geom_bodyid[static_cast<std::size_t>(a.geom_id)];
        } else {
            continue; // a grip without a connect equality is skipped
        }
        out.push_back(std::move(a));
    }
    return out;
}

[[gnu::noinline]] void test_allocated_wrenches_close_the_rider_dynamics() {
    // tests/reference/test_rider_allocation.py::test_allocated_wrenches_close_the_rider_dynamics
    auto rig = welded_rig();
    const rcfg::RawStep last = advance_env_steps(*rig->physical, 30);
    const auto &allocation = channel(last, "rider_allocation");
    bt::require(channel_bool(allocation, "feasible"),
                "the recorded interval allocated feasibly");
    const Vector qddot = channel_array(allocation, "solution_qddot");
    const Vector wrenches = channel_array(allocation, "solution_wrenches");
    const Vector moments = channel_array(allocation, "solution_moments");
    const Vector tau = channel_array(allocation, "solution_tau");
    const Vector qacc_b = channel_array(allocation, "solution_base_qacc");
    const Vector qpos = channel_array(allocation, "solution_qpos");
    const Vector qvel = channel_array(allocation, "solution_qvel");
    const double solved_time = channel_real(allocation, "solution_time_s");
    auto &clock = rig->physical->control_clock();
    const auto step = static_cast<std::int64_t>(
        std::llround(solved_time / clock.timestep_s));
    bt::require(clock.is_tick(step),
                "the solved command landed on a control tick");
    bt::require(step == last.interval_id -
                            last.interval_id % clock.steps_per_period,
                "the recorded state is the interval start of the held "
                "allocation");
    auto *model = rig->stepper->model();
    auto *data = rig->stepper->data();
    rig->stepper->mutate([&] {
        const auto position = model_access::mutable_buffer(
            data->qpos, model->nq, "qpos");
        const auto velocity = model_access::mutable_buffer(
            data->qvel, model->nv, "qvel");
        for (std::size_t i = 0; i < qpos.size(); ++i)
            position[i] = qpos[i];
        for (std::size_t i = 0; i < qvel.size(); ++i)
            velocity[i] = qvel[i];
        engine::forward(model, data);
    });
    const auto nv = static_cast<std::size_t>(model->nv);
    const auto dof_adr = model_access::readonly_buffer(
        model->jnt_dofadr, model->njnt, "jnt_dofadr");
    auto &controller = rig->physical->rider_control();
    // rider_dofs = {root_x, root_z, root_pitch dofadr} + hinge dofs;
    // bike_dofs = the complement, in model order.
    std::vector<int> rd, bd;
    for (const char *joint :
         {"rider_root_x", "rider_root_z", "rider_root_pitch"}) {
        const int j = mj_name2id(model, mjOBJ_JOINT, joint);
        bt::require(j >= 0, "the welded plant has the floating rider root");
        rd.push_back(dof_adr[static_cast<std::size_t>(j)]);
    }
    for (const auto &entry : controller.joints()) rd.push_back(entry.dof_adr);
    for (int i = 0; i < model->nv; ++i)
        if (std::ranges::find(rd, i) == rd.end()) bd.push_back(i);
    std::vector<double> full(nv * nv, 0.);
    mj_fullM(model, data, full.data());
    const auto qfrc_bias = model_access::readonly_buffer(
        data->qfrc_bias, model->nv, "qfrc_bias");
    const auto qvel_now = model_access::readonly_buffer(
        data->qvel, model->nv, "qvel");
    Vector bias(rd.size(), 0.);
    for (std::size_t i = 0; i < rd.size(); ++i) {
        bias[i] = qfrc_bias[static_cast<std::size_t>(rd[i])];
        for (std::size_t j = 0; j < bd.size(); ++j)
            bias[i] += full[static_cast<std::size_t>(rd[i]) * nv +
                            static_cast<std::size_t>(bd[j])] *
                       qacc_b[j];
    }
    const auto attachments = welded_attachments(*model);
    bt::require(attachments.size() * 2 == wrenches.size() &&
                    attachments.size() == moments.size(),
                "the recorded wrenches cover every attachment");
    // jac rows (jp_x[rd], jp_z[rd], jr_y[rd]) against the expanded wrench
    // triples (Fx, Fz, My) per attachment, at the sole/grip support point.
    Vector jac_rhs(rd.size(), 0.);
    std::vector<double> jp(3 * nv, 0.), jr(3 * nv, 0.);
    for (std::size_t i = 0; i < attachments.size(); ++i) {
        const auto point = rider::allocation_support_point(
            *model, *data, attachments[i]);
        std::ranges::fill(jp, 0.);
        std::ranges::fill(jr, 0.);
        mj_jac(model, data, jp.data(), jr.data(), point.data(),
               attachments[i].rider_body);
        const std::array<double, 3> w{wrenches[2 * i], wrenches[2 * i + 1],
                                      moments[i]};
        for (std::size_t r = 0; r < rd.size(); ++r) {
            const auto dof = static_cast<std::size_t>(rd[r]);
            jac_rhs[r] += jp[0 * nv + dof] * w[0] + jp[2 * nv + dof] * w[1] +
                          jr[1 * nv + dof] * w[2];
        }
    }
    const auto [envelope, stored] = controller.envelope_forces(data);
    (void)stored;
    Vector known = envelope;
    const double damping = controller.config().passive_damping_nms_rad();
    std::size_t index = 0;
    Vector b_torque(nv, 0.);
    for (const auto &entry : controller.joints()) {
        const auto dof = static_cast<std::size_t>(entry.dof_adr);
        known[dof] -= damping * qvel_now[dof];
        b_torque[dof] += tau[index++];
    }
    for (std::size_t r = 0; r < rd.size(); ++r) {
        const auto row = static_cast<std::size_t>(rd[r]);
        double predicted = 0.;
        for (std::size_t c = 0; c < rd.size(); ++c)
            predicted += full[row * nv + static_cast<std::size_t>(rd[c])] *
                         qddot[c];
        const double residual = predicted + bias[r] -
                                b_torque[row] - jac_rhs[r] - known[row];
        bt::require(std::abs(residual) <= 1e-4,
                    "the solved z satisfies M*qddot + bias = B*tau + "
                    "J.T*f + known at row " + std::to_string(r));
    }
}

[[gnu::noinline]] void test_command_does_not_read_solved_reactions() {
    // tests/reference/test_rider_allocation.py::test_command_does_not_read_solved_reactions
    auto rig = welded_rig();
    (void)advance_env_steps(*rig->physical, 20);
    ::rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.enabled = true;
    command.posture = rig->physical->applied_control().posture.value_or(
        ::rider::RiderPosture{});
    command.crank_target_phase_rad = std::nullopt;
    const auto first = rig->physical->rider_control().compute(
        rig->stepper->data(), command, false);
    const auto nefc = static_cast<std::size_t>(rig->stepper->data()->nefc);
    const auto nv = static_cast<std::size_t>(rig->stepper->model()->nv);
    std::mt19937_64 rng(7);
    std::normal_distribution<double> normal(0., 1.);
    std::vector<double> perturbed(nefc);
    for (auto &v : perturbed) v = normal(rng);
    rig->stepper->mutate([&] {
        const auto forces = model_access::mutable_buffer(
            rig->stepper->data()->efc_force,
            static_cast<mjtSize>(nefc), "efc_force");
        for (std::size_t i = 0; i < nefc; ++i)
            forces[i] += perturbed[i];
        // qfrc_constraint — the second solved-reaction channel the
        // information-boundary oracle corrupts alongside efc_force.
        const auto constraint = model_access::mutable_buffer(
            rig->stepper->data()->qfrc_constraint,
            static_cast<mjtSize>(nv), "qfrc_constraint");
        for (std::size_t i = 0; i < nv; ++i)
            constraint[i] -= 1e9;
    });
    // The third privileged channel: the committed rider-contacts
    // diagnostics. The writer state round-trips through restore(), so a
    // copy/mutate/restore cycle corrupts it through the sanctioned path.
    {
        // supports order: 0 'saddle', 1 'front_pedal' (rider_contacts.hpp).
        auto contacts_state = rig->stepper->rider_contacts().state();
        contacts_state.diagnostics.supports[0].normal_load_n = 1e9;
        contacts_state.diagnostics.supports[1].normal_load_n = 1e9;
        auto contacts_probe = rig->stepper->rider_contacts().probe();
        rig->stepper->rider_contacts().assign_state(
            std::move(contacts_state), std::move(contacts_probe));
    }
    const auto second = rig->physical->rider_control().compute(
        rig->stepper->data(), command, false);
    bt::require(first.size() == second.size(),
                "the command carries the same joint set");
    for (std::size_t i = 0; i < first.size(); ++i) {
        bt::require(first[i].first == second[i].first,
                    "the command order is unchanged");
        bt::require(std::abs(first[i].second - second[i].second) <= 1e-9,
                    "the command does not read solved reactions: " +
                        first[i].first);
    }
}

[[gnu::noinline]] void test_infeasible_intent_keeps_the_physical_bounds() {
    // tests/reference/test_rider_allocation.py::test_infeasible_intent_keeps_the_physical_bounds
    auto rig = welded_rig();
    (void)advance_env_steps(*rig->physical, 20);
    auto *model = rig->stepper->model();
    auto *data = rig->stepper->data();
    auto &controller = rig->physical->rider_control();
    const auto &config = controller.config();
    const auto &legacy = config.legacy;
    bt::require(legacy.has_value(),
                "the welded plant runs the legacy response allocator");
    const auto &joints = controller.joints();
    const auto qpos = model_access::readonly_buffer(data->qpos, model->nq,
                                                    "qpos");
    const auto qvel = model_access::readonly_buffer(data->qvel, model->nv,
                                                    "qvel");
    const auto forcelimited = model_access::readonly_buffer(
        model->actuator_forcelimited, model->nu, "actuator_forcelimited");
    const auto forcerange = model_access::readonly_buffer(
        model->actuator_forcerange, 2 * model->nu, "actuator_forcerange");
    const auto ctrllimited = model_access::readonly_buffer(
        model->actuator_ctrllimited, model->nu, "actuator_ctrllimited");
    const auto ctrlrange = model_access::readonly_buffer(
        model->actuator_ctrlrange, 2 * model->nu, "actuator_ctrlrange");
    const auto dof_adr = model_access::readonly_buffer(
        model->jnt_dofadr, model->njnt, "jnt_dofadr");
    const auto body_mass = model_access::readonly_buffer(
        model->body_mass, model->nbody, "body_mass");
    // allocation_request's bound assembly, through the public capacity
    // accessors (strength_capacity == directional_limit's public twin).
    rider::ResponseRequest request;
    request.config = *legacy;
    for (const auto &j : joints) {
        const auto dof = static_cast<std::size_t>(j.dof_adr);
        request.actuator_dofs.push_back(j.dof_adr);
        const double speed = qvel[dof];
        const double angle = controller.anatomical_joint_angle(
            j.name, qpos[static_cast<std::size_t>(j.qpos_adr)]);
        double positive = config.has_strength
                              ? controller.strength_capacity(j.name, angle,
                                                             speed, 1.)
                              : config.joint_limit_nm;
        double negative = config.has_strength
                              ? controller.strength_capacity(j.name, angle,
                                                             speed, -1.)
                              : config.joint_limit_nm;
        if (speed > 0.) {
            positive = std::min(positive,
                                config.joint_power_limit_w / speed);
            if (speed >= config.joint_speed_limit_rad_s) positive = 0.;
        } else if (speed < 0.) {
            negative = std::min(negative,
                                -config.joint_power_limit_w / speed);
            if (-speed >= config.joint_speed_limit_rad_s) negative = 0.;
        }
        double lo = -negative, hi = positive;
        const auto act_id = static_cast<std::size_t>(j.actuator_id);
        if (forcelimited[act_id]) {
            lo = std::max(lo, forcerange[2 * act_id]);
            hi = std::min(hi, forcerange[2 * act_id + 1]);
        }
        if (ctrllimited[act_id]) {
            lo = std::max(lo, ctrlrange[2 * act_id]);
            hi = std::min(hi, ctrlrange[2 * act_id + 1]);
        }
        request.lower_torque_nm.push_back(lo);
        request.upper_torque_nm.push_back(hi);
    }
    request.torque_intent.assign(joints.size(), 1e4);
    request.activation_state = controller.state().active_state;
    request.attachments = welded_attachments(*model);
    for (const char *joint :
         {"rider_root_x", "rider_root_z", "rider_root_pitch"})
        request.rider_dofs.push_back(dof_adr[static_cast<std::size_t>(
            mj_name2id(model, mjOBJ_JOINT, joint))]);
    for (const auto &j : joints) request.rider_dofs.push_back(j.dof_adr);
    for (int i = 0; i < model->nv; ++i)
        if (std::ranges::find(request.rider_dofs, i) ==
            request.rider_dofs.end())
            request.bike_dofs.push_back(i);
    request.pelvis_body =
        mj_name2id(model, mjOBJ_BODY, "rider_pelvis");
    double mass = 0.;
    for (int body = 0; body < model->nbody; ++body) {
        const char *name = mj_id2name(model, mjOBJ_BODY, body);
        if (name != nullptr && std::string_view(name).starts_with("rider_"))
            mass += body_mass[static_cast<std::size_t>(body)];
    }
    request.rider_mass_kg = mass;
    request.dt_s = rig->physical->control_clock().timestep_s;
    rider::AllocationMemory memory;
    const auto result = rider::allocate_response(*model, *data, request,
                                                 memory);
    bt::require(result.allocation.feasible,
                "the absurd intent is relaxed, not stalled");
    bt::require(result.torque.size() == joints.size(),
                "the allocation covers every joint");
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto dof = static_cast<std::size_t>(joints[i].dof_adr);
        const double angle = controller.anatomical_joint_angle(
            joints[i].name,
            qpos[static_cast<std::size_t>(joints[i].qpos_adr)]);
        const double cap =
            config.has_strength
                ? std::max(controller.strength_capacity(joints[i].name,
                                                        angle, qvel[dof],
                                                        1.),
                           controller.strength_capacity(joints[i].name,
                                                        angle, qvel[dof],
                                                        -1.))
                : config.joint_limit_nm;
        bt::require(std::abs(result.torque[i]) <= cap + 1e-6,
                    "the relaxed command keeps the physical bound: " +
                        joints[i].name);
    }
}

[[gnu::noinline]] void test_allocate_effort_returns_target_without_slsqp_when_target_is_feasible() {
    // tests/reference/test_rider_allocation.py::test_allocate_effort_returns_target_without_slsqp_when_target_is_feasible
    // The oracle proves the solver never runs by failing a monkeypatched
    // minimize; the native fast path is the residual<=1e-7 early return —
    // bitwise identity to the feasible target is the same proof.
    const auto result = rider::allocate(problem(
        {1., -2.}, d::zeros(0, 2), {}, d::zeros(0, 2), {}, {-10., -10.},
        {10., 10.}));
    bt::require(result.solution == Vector{1., -2.},
                "an already-feasible target returns bitwise unchanged");
    bt::require(result.feasible && result.violation == 0.,
                "the feasible target reports zero violation");
}
} // namespace

int main() {
    return bt::run([] {
        test_saddle_reserve_is_an_actual_configured_lower_bound();
        test_coupled_support_cone_allows_compression_and_rejects_tension();
        test_finalizer_delivers_allocated_torque_instead_of_rebuilding_raw_wish();
        test_support_moment_constraints_reject_cop_outside_the_physical_patch();
        test_activation_dynamics_shape_the_wish_before_support_allocation();
        test_intent_is_relaxed_but_physical_limit_is_not();
        test_infeasible_support_is_not_reported_as_valid_stall();
        test_scaled_grip_uses_a_circle_not_a_componentwise_square();
        test_diagonal_pull_beyond_the_circle_is_infeasible();
        test_grip_constraint_validation();
        test_pressing_branch_bounds_only_the_sign();
        test_inverse_dynamics_rows_layout_and_sign();
        test_allocation_rejects_bad_problem_data();
        test_nonfinite_constraint_eval_reports_infinite_violation();
        test_positive_power_cap_is_a_sum_not_a_net();
        test_isometric_joint_does_not_divide_by_zero_speed();
        test_solution_is_deterministic_for_identical_inputs();
        test_candidate_is_preserved_when_infeasible();
        test_warm_start_does_not_change_the_answer();
        test_invalid_warm_start_is_rejected();
        test_branch_memory_preserves_cold_ranking_and_full_fallback();
        test_vector_nonlinear_bounds_and_extra_equalities_keep_their_semantics();
        test_nonlinear_constraint_is_evaluated_at_the_bounded_start();
        test_real_grip_problem_changes_from_remembered_pull_to_press();
        test_healthy_step_allocates_a_feasible_command();
        test_allocated_wrenches_close_the_rider_dynamics();
        test_command_does_not_read_solved_reactions();
        test_infeasible_intent_keeps_the_physical_bounds();
        test_allocate_effort_returns_target_without_slsqp_when_target_is_feasible();
    });
}
