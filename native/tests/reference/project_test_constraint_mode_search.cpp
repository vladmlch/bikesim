// Ports all six source test functions in
// project/tests/reference/test_constraint_mode_search.py — the bounded
// mechanical-mode search contracts. `budget` is typed std::size_t: the
// -1/1.5/True/None parameter rows of
// test_invalid_mode_budget_fails_before_evaluating_dynamics are compile-time
// rejections in this signature; only 0 reaches the same runtime check.
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "numerics/dense.hpp"
#include "rider/constraint_dynamics.hpp"
#include "rider/response_allocation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace d = bike::numerics::dense;
namespace rider = bike::rider;
namespace engine = ::engine;
using bike::rider::Matrix;
using Vector = std::vector<double>;

bt::ModelFixture stand(const char *kind) {
    const bool friction = std::string_view(kind) == "friction";
    const std::string xml =
        std::string("<mujoco><option gravity=\"0 0 0\" timestep=\".0005\" tolerance=\"1e-12\"/>"
                    "<worldbody>"
                    "<body><joint name=\"a\" type=\"slide\" axis=\"1 0 0\"") +
        (friction ? " frictionloss=\"2\"" : "") +
        "/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body>"
        "<body><joint name=\"b\" type=\"slide\" axis=\"1 0 0\"/>"
        "<inertial mass=\"2\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body></worldbody>" +
        (friction ? "" :
         "<tendon><fixed limited=\"true\" range=\"-10 0\">"
         "<joint joint=\"a\" coef=\"1\"/><joint joint=\"b\" coef=\"-1\"/>"
         "</fixed></tendon>") +
        "</mujoco>";
    bt::ModelFixture rig(xml);
    if (!friction)
        model_access::mutable_buffer(rig.data->qpos, rig.model->nq,
                                     "qpos")[0] = 1e-5;
    engine::forward(rig.model.get(), rig.data.get());
    return rig;
}

bool approx(double actual, double expected, double rtol, double atol) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}
void allclose(std::span<const double> actual, std::span<const double> expected,
              double rtol, double atol, std::string_view what) {
    bt::require(actual.size() == expected.size(),
                std::string(what) + ": shape matches the forward solve");
    for (std::size_t i = 0; i < actual.size(); ++i)
        bt::require(approx(actual[i], expected[i], rtol, atol),
                    std::string(what) + " element " + std::to_string(i) +
                        " tracks the forward solve");
}

[[gnu::noinline]] void test_command_crosses_mode_boundaries_and_matches_current_forward() {
    // project/tests/reference/test_constraint_mode_search.py::test_command_crosses_mode_boundaries_and_matches_current_forward
    struct Param { const char *kind; double wish, boundary; };
    for (const auto &param : {Param{"tendon", 20., -20.}, {"tendon", -20., 20.},
                              {"friction", 20., 1.}, {"friction", -20., -1.},
                              {"friction", 20., -20.}, {"friction", -20., 20.}}) {
        auto rig = stand(param.kind);
        std::vector<int> dof{0};
        rider::ConstraintDynamics dynamics(*rig.model, *rig.data, dof);
        const Vector wish{param.wish};
        const double direction = param.wish > param.boundary ? 1. : -1.;
        std::vector<rider::ConstraintMode> payloads;
        const auto solve = [&](const rider::ConstraintResponse &response) {
            const std::array<double, 1> boundary_row{direction};
            auto g = response.domain_matrix;
            d::append_row(g, boundary_row);
            auto h = response.domain_bound;
            h.push_back(direction * param.boundary);
            rider::AllocationProblem problem;
            problem.target = wish;
            problem.equality = d::zeros(0, 1);
            problem.inequality = std::move(g);
            problem.inequality_rhs = std::move(h);
            problem.lower = {-50.};
            problem.upper = {50.};
            auto result = rider::allocate(problem);
            payloads.push_back(response.mode);
            return rider::ModeCandidate{.allocation = std::move(result),
                                        .payload_index = payloads.size() - 1};
        };
        const auto first = dynamics.linearize(wish);
        bt::require(!solve(first).allocation.feasible,
                    "the wish's local mode is infeasible on this boundary");
        const auto result = rider::search_constraint_modes(dynamics, wish, solve);
        bt::require(result.allocation.feasible && result.complete &&
                        result.modes_tried > 1,
                    "a feasible mode is reached across the boundary");
        bt::require(payloads[result.payload_index] == result.response.mode,
                    "the payload belongs to the returned response");
        bt::require(std::abs(result.allocation.solution[0] - param.boundary) <= 1e-7,
                    "the crossing command lands on the mode boundary");
        const auto domain = d::multiply(result.response.domain_matrix,
                                        result.allocation.solution);
        for (std::size_t i = 0; i < domain.size(); ++i)
            bt::require(domain[i] <= result.response.domain_bound[i] + 1e-7,
                        "the solved command satisfies the returned mode domain");
        model_access::mutable_buffer(rig.data->qfrc_applied, rig.model->nv,
                                     "qfrc_applied")[0] =
            result.allocation.solution[0];
        engine::forward(rig.model.get(), rig.data.get());
        const auto predicted_a =
            d::add(result.response.acceleration_offset,
                   d::multiply(result.response.acceleration_matrix,
                               result.allocation.solution));
        allclose(predicted_a,
                 model_access::readonly_buffer(rig.data->qacc, rig.model->nv,
                                               "qacc"),
                 1e-10, 1e-9, "predicted acceleration");
        const auto predicted_f =
            d::add(result.response.force_offset,
                   d::multiply(result.response.force_matrix,
                               result.allocation.solution));
        allclose(predicted_f,
                 model_access::readonly_buffer(rig.data->efc_force,
                                               rig.data->nefc, "efc_force"),
                 1e-10, 1e-9, "predicted constraint reactions");
    }
}

[[gnu::noinline]] void test_exact_mode_budget_distinguishes_exhaustion_from_truncation() {
    // project/tests/reference/test_constraint_mode_search.py::test_exact_mode_budget_distinguishes_exhaustion_from_truncation
    struct Param { const char *kind; std::size_t mode_count; bool truncate; };
    for (const auto &param : {Param{"tendon", 2, false}, {"tendon", 2, true},
                              {"friction", 3, false}, {"friction", 3, true}}) {
        auto rig = stand(param.kind);
        std::vector<int> dof{0};
        rider::ConstraintDynamics dynamics(*rig.model, *rig.data, dof);
        const Vector wish{20.};
        const std::size_t budget = param.truncate ? param.mode_count - 1
                                                  : param.mode_count;
        struct Payload { rider::ConstraintMode mode; std::size_t call; };
        std::vector<rider::ConstraintMode> calls;
        std::vector<Payload> payloads;
        std::vector<rider::Allocation> results;
        const auto solve = [&](const rider::ConstraintResponse &response) {
            calls.push_back(response.mode);
            const double violation =
                std::array{2., .5, 1.}[calls.size() - 1];
            rider::Allocation result{
                .solution = {param.kind == std::string_view("tendon") ? -20. : 0.},
                .feasible = false,
                .violation = violation};
            results.push_back(result);
            payloads.push_back({.mode = response.mode, .call = calls.size()});
            return rider::ModeCandidate{.allocation = result,
                                        .payload_index = payloads.size() - 1};
        };
        const auto result = rider::search_constraint_modes(
            dynamics, wish, solve, {}, budget);
        bt::require(result.modes_tried == budget && calls.size() == budget &&
                        std::set<rider::ConstraintMode>(calls.begin(),
                                                        calls.end())
                                .size() == budget,
                    "the budget bounds the distinct evaluated modes exactly");
        bt::require(result.complete == !param.truncate,
                    "complete distinguishes exhaustion from truncation");
        const auto best = static_cast<std::size_t>(std::min_element(
            results.begin(), results.end(),
            [](const auto &a, const auto &b) { return a.violation < b.violation; }) -
            results.begin());
        bt::require(!result.allocation.feasible &&
                        result.allocation.violation == results[best].violation &&
                        result.payload_index == best,
                    "the returned candidate is the lowest-violation iterate");
        bt::require(payloads[best].mode == calls[best] &&
                        payloads[best].call == best + 1,
                    "the payload belongs to that same iterate");
        bt::require(result.response.mode == calls[best],
                    "the returned response belongs to that same iterate");
        bt::require(wish == Vector{20.}, "the wish is not mutated by the search");
    }
}

[[gnu::noinline]] void test_feasible_first_mode_stops_without_exploring_neighbors() {
    // project/tests/reference/test_constraint_mode_search.py::test_feasible_first_mode_stops_without_exploring_neighbors
    auto rig = stand("tendon");
    std::vector<int> dof{0};
    rider::ConstraintDynamics dynamics(*rig.model, *rig.data, dof);
    const Vector wish{20.};
    std::vector<rider::ConstraintMode> calls;
    std::vector<std::string> payloads;
    const auto solve = [&](const rider::ConstraintResponse &response) {
        calls.push_back(response.mode);
        payloads.emplace_back("feasible");
        return rider::ModeCandidate{
            .allocation = {.solution = wish, .feasible = true, .violation = 0.},
            .payload_index = payloads.size() - 1};
    };
    const auto result = rider::search_constraint_modes(dynamics, wish, solve, {}, 1);
    bt::require(result.allocation.feasible &&
                    payloads[result.payload_index] == "feasible",
                "the feasible iterate is returned with its payload");
    bt::require(calls.size() == 1 && result.response.mode == calls[0] &&
                    result.modes_tried == 1 && result.complete,
                "a feasible command stops the mode search immediately");
}

[[gnu::noinline]] void test_invalid_mode_budget_fails_before_evaluating_dynamics() {
    // project/tests/reference/test_constraint_mode_search.py::test_invalid_mode_budget_fails_before_evaluating_dynamics
    // 0 is the only parameter expressible in a std::size_t budget; the
    // remaining source rows (-1, 1.5, True, None) are compile-time type
    // rejections. The empty solver proves the budget check precedes every
    // other evaluation, like the oracle's type gate before dynamics.
    auto rig = stand("tendon");
    std::vector<int> dof{0};
    rider::ConstraintDynamics dynamics(*rig.model, *rig.data, dof);
    const Vector wish{0.};
    bool rejected = false;
    try {
        (void)rider::search_constraint_modes(dynamics, wish, {}, {}, 0);
    } catch (const std::invalid_argument &error) {
        rejected = std::string_view(error.what()).find("positive integer") !=
                   std::string_view::npos;
        bt::require(rejected,
                    "a zero budget fails as a non-positive integer budget");
    }
    bt::require(rejected, "a zero candidate budget is rejected before any solve");
}

[[gnu::noinline]] void test_guarded_grip_branch_preserves_sign_and_circle_budgets() {
    // project/tests/reference/test_constraint_mode_search.py::test_guarded_grip_branch_preserves_sign_and_circle_budgets
    for (const bool pulling : {false, true}) {
        const auto constraints = rider::allocation_grip_constraints(
            d::identity(2), {1., 0.}, pulling, 300.);
        const auto &sign = constraints[0];
        bt::require(sign.linear.has_value() && sign.linear->rows == 1 &&
                        sign.linear->cols == 2,
                    "the sign constraint stays linear");
        bt::require(sign.linear->values[0] == (pulling ? -1. : 1.) &&
                        sign.linear->values[1] == 0.,
                    "the guarded sign row matches the branch");
        bt::require(sign.lower == Vector{pulling ? 0. :
                                                  rider::support_force_guard_n},
                    "the sign lower bound keeps the press guard");
        bt::require(sign.upper.size() == 1 &&
                        sign.upper[0] == std::numeric_limits<double>::infinity(),
                    "the sign upper bound stays open");
        if (pulling) {
            bt::require(constraints.size() == 2,
                        "the pulling branch adds the guarded circle");
            const double radius = 300. * (1. - rider::grip_radius_guard_fraction);
            const std::array<double, 2> guarded{-radius, 0.};
            bt::require(approx(constraints[1].evaluate(guarded)[0], 1.,
                               1e-6, 1e-12),
                        "the circle budget is met at the guarded radius");
            const std::array<double, 2> unguarded{-300., 0.};
            bt::require(constraints[1].evaluate(unguarded)[0] > 1.,
                        "the unguarded radius still violates the circle");
        } else {
            bt::require(constraints.size() == 1,
                        "press magnitude remains a joint budget, not a circle");
            bt::require(sign.linear->values[0] * 400. >= sign.lower[0],
                        "a large pressing force satisfies the sign row");
            bt::require(sign.linear->values[0] * -1. < sign.lower[0],
                        "a pulling force violates the pressing sign row");
        }
    }
}

[[gnu::noinline]] void test_guarded_grip_search_falls_back_from_remembered_pull_to_press() {
    // project/tests/reference/test_constraint_mode_search.py::test_guarded_grip_search_falls_back_from_remembered_pull_to_press
    std::vector<std::array<bool, 2>> calls;
    const Vector target{100., 100.};
    const auto solve = [&](const std::array<bool, 2> &branch,
                           const std::optional<Vector> &x0) {
        calls.push_back(branch);
        std::vector<rider::ExtraConstraint> extra;
        for (std::size_t i = 0; i < 2; ++i) {
            Matrix map = d::zeros(2, 2);
            map.values[i] = 1.;
            auto branch_constraints = rider::allocation_grip_constraints(
                map, {1., 0.}, branch[i], 300.);
            extra.insert(extra.end(),
                         std::make_move_iterator(branch_constraints.begin()),
                         std::make_move_iterator(branch_constraints.end()));
        }
        rider::AllocationProblem problem;
        problem.target = target;
        problem.equality = d::identity(2);
        problem.equality_rhs = target;
        problem.inequality = d::zeros(0, 2);
        problem.lower = {-1000., -1000.};
        problem.upper = {1000., 1000.};
        problem.extra = std::move(extra);
        problem.initial = x0;
        return rider::allocate(problem);
    };
    rider::AllocationMemory memory{.branch = {{true, true}},
                                   .solution = Vector{-100., -100.}};
    const auto result =
        rider::select_allocation_branch(target, solve, memory);
    bt::require(result.allocation.feasible &&
                    result.branch == std::array<bool, 2>{false, false},
                "the remembered pull falls back to the pressing branch");
    bt::require(calls.size() == 4 && calls[0] == std::array<bool, 2>{true, true} &&
                    std::set<std::array<bool, 2>>(calls.begin(), calls.end()).size() == 4,
                "the remembered branch is tried first, then every branch once");
    for (std::size_t i = 0; i < 2; ++i)
        bt::require(std::abs(result.allocation.solution[i] - target[i]) <= 1e-7,
                    "the pressing solution reaches the wished force");
}
} // namespace

int main() {
    return bt::run([] {
        test_command_crosses_mode_boundaries_and_matches_current_forward();
        test_exact_mode_budget_distinguishes_exhaustion_from_truncation();
        test_feasible_first_mode_stops_without_exploring_neighbors();
        test_invalid_mode_budget_fails_before_evaluating_dynamics();
        test_guarded_grip_branch_preserves_sign_and_circle_budgets();
        test_guarded_grip_search_falls_back_from_remembered_pull_to_press();
    });
}
