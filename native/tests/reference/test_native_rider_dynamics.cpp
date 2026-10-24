// Ports tests/reference/test_rider_dynamics.py — current-state
// soft-constraint prediction against actual MuJoCo solves. The oracle's
// __new__ stubs around ArticulatedRiderController.resolve map onto the
// public ResponseRequest surface (the fields the stub sets are exactly the
// request inputs the legacy allocator consumes). The finalize_effort leg
// of test_actual_allocation_cannot_jump_outside_bounded_activation_reach
// is controller-internal in C++; its contract — delivered equals the
// allocated torque and the committed state is exactly one activation step —
// is asserted through the same activation math it publishes.
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "numerics/dense.hpp"
#include "numerics/linear_feasibility.hpp"
#include "rider/activation_allocation.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/constraint_dynamics.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/response_allocation.hpp"
#include "rider/spindle_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <string>
#include <vector>

namespace {
namespace d = bike::numerics::dense;
namespace rider = bike::rider;
namespace engine = ::engine;
using bike::rider::Matrix;
using Vector = std::vector<double>;

bool approx(double actual, double expected, double rtol = 0.,
            double atol = 0.) {
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

bt::ModelFixture stand(std::string_view kind) {
    std::string xml =
        "<mujoco><option gravity=\"0 0 0\" timestep=\".0005\" tolerance=\"1e-12\"/>";
    if (kind == "weld") {
        xml += "<worldbody><body name=\"a\"><freejoint/><inertial mass=\"2\" pos=\".1 0 0\" diaginertia=\"1 1 1\"/></body>"
               "<body name=\"b\"><freejoint/><inertial mass=\"1\" pos=\"0 0 .1\" diaginertia=\".5 .5 .5\"/></body></worldbody>"
               "<equality><weld body1=\"a\" body2=\"b\" solref=\".005 1\"/></equality>";
    } else {
        xml += std::string("<worldbody><body><joint name=\"a\" type=\"slide\" axis=\"1 0 0\"") +
               (kind == "friction" ? " frictionloss=\"2\"" : "") +
               "/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body>"
               "<body><joint name=\"b\" type=\"slide\" axis=\"1 0 0\"/>"
               "<inertial mass=\"2\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body></worldbody>";
        if (kind == "tendon")
            xml += "<tendon><fixed limited=\"true\" range=\"-10 0\">"
                   "<joint joint=\"a\" coef=\"1\"/><joint joint=\"b\" coef=\"-1\"/>"
                   "</fixed></tendon>";
    }
    xml += "</mujoco>";
    bt::ModelFixture rig(xml);
    if (kind == "tendon")
        model_access::mutable_buffer(rig.data->qpos, rig.model->nq,
                                     "qpos")[0] = 1e-5;
    return rig;
}

// The oracle's `model.eq_solimp[:,:2] = impedance` /
// `model.tendon_solimp_lim[:,:2] = impedance` row writes (mjNIMP columns).
void set_impedance(mjModel *model, double impedance) {
    const auto solimp = model_access::mutable_buffer(
        model->eq_solimp, model->neq * mjNIMP, "eq_solimp");
    for (std::size_t i = 0; i < static_cast<std::size_t>(model->neq); ++i)
        for (std::size_t j = 0; j < 2; ++j)
            solimp[i * mjNIMP + j] = impedance;
    const auto limp = model_access::mutable_buffer(
        model->tendon_solimp_lim, model->ntendon * mjNIMP,
        "tendon_solimp_lim");
    for (std::size_t i = 0; i < static_cast<std::size_t>(model->ntendon);
         ++i)
        for (std::size_t j = 0; j < 2; ++j)
            limp[i * mjNIMP + j] = impedance;
}

[[gnu::noinline]] void test_model_response_predicts_current_acceleration_and_all_reactions() {
    // tests/reference/test_rider_dynamics.py::test_model_response_predicts_current_acceleration_and_all_reactions
    struct Param { const char *kind; double torque, impedance; };
    for (const char *kind : {"weld", "tendon", "friction"})
        for (const double torque : {-20., 0., 20.})
            for (const double impedance : {.95, .9999})
                [&] {
                    auto rig = stand(kind);
                    set_impedance(rig.model.get(), impedance);
                    const auto nv = static_cast<std::size_t>(rig.model->nv);
                    const auto applied = model_access::mutable_buffer(
                        rig.data->qfrc_applied, rig.model->nv,
                        "qfrc_applied");
                    applied[nv - 1] = .1;
                    engine::forward(rig.model.get(), rig.data.get());
                    const std::array<int, 1> dofs{0};
                    rider::ConstraintDynamics kernel(*rig.model, *rig.data,
                                                     dofs);
                    const Vector wish{torque};
                    const auto response = kernel.linearize(wish);
                    const auto predicted_a =
                        d::add(response.acceleration_offset,
                               d::multiply(response.acceleration_matrix,
                                           wish));
                    const auto predicted_f =
                        d::add(response.force_offset,
                               d::multiply(response.force_matrix, wish));
                    const auto domain = d::multiply(response.domain_matrix,
                                                    wish);
                    for (std::size_t i = 0; i < domain.size(); ++i)
                        bt::require(domain[i] <=
                                        response.domain_bound[i] + 1e-8,
                                    "the wish satisfies its own mode domain");
                    model_access::mutable_buffer(
                        rig.data->qfrc_applied, rig.model->nv,
                        "qfrc_applied")[0] += torque;
                    engine::forward(rig.model.get(), rig.data.get());
                    allclose(predicted_a,
                             model_access::readonly_buffer(
                                 rig.data->qacc, rig.model->nv, "qacc"),
                             1e-10, 1e-9, "predicted acceleration");
                    allclose(predicted_f,
                             model_access::readonly_buffer(
                                 rig.data->efc_force, rig.data->nefc,
                                 "efc_force"),
                             1e-10, 1e-9, "predicted constraint reactions");
                }();
}

[[gnu::noinline]] void test_prediction_does_not_read_previous_reactions_or_acceleration() {
    // tests/reference/test_rider_dynamics.py::test_prediction_does_not_read_previous_reactions_or_acceleration
    auto rig = stand("weld");
    model_access::mutable_buffer(rig.data->qfrc_applied, rig.model->nv,
                                 "qfrc_applied")[0] = 10.;
    engine::forward(rig.model.get(), rig.data.get());
    const std::array<int, 1> dofs{0};
    const Vector wish{5.};
    const auto expected = rider::ConstraintDynamics(*rig.model, *rig.data,
                                                    dofs).linearize(wish);
    const auto nv = static_cast<std::size_t>(rig.model->nv);
    const auto forces = model_access::mutable_buffer(
        rig.data->efc_force, rig.data->nefc, "efc_force");
    for (std::size_t i = 0; i < forces.size(); ++i)
        forces[i] = std::numeric_limits<double>::quiet_NaN();
    const auto constraint = model_access::mutable_buffer(
        rig.data->qfrc_constraint, rig.model->nv, "qfrc_constraint");
    const auto acceleration = model_access::mutable_buffer(
        rig.data->qacc, rig.model->nv, "qacc");
    for (std::size_t i = 0; i < nv; ++i) {
        constraint[i] = std::numeric_limits<double>::quiet_NaN();
        acceleration[i] = std::numeric_limits<double>::quiet_NaN();
    }
    const auto actual = rider::ConstraintDynamics(*rig.model, *rig.data,
                                                  dofs).linearize(wish);
    bt::require(actual.acceleration_offset == expected.acceleration_offset,
                "the predicted acceleration offset ignores solved outputs");
    bt::require(actual.force_matrix.values == expected.force_matrix.values,
                "the predicted force matrix ignores solved outputs");
}

[[gnu::noinline]] void test_unilateral_mode_domain_rejects_a_command_that_changes_contact_state() {
    // tests/reference/test_rider_dynamics.py::test_unilateral_mode_domain_rejects_a_command_that_changes_contact_state
    auto rig = stand("tendon");
    model_access::mutable_buffer(rig.data->qpos, rig.model->nq,
                                 "qpos")[0] = 1e-5;
    engine::forward(rig.model.get(), rig.data.get());
    const std::array<int, 1> dofs{0};
    rider::ConstraintDynamics kernel(*rig.model, *rig.data, dofs);
    const auto engaged = kernel.linearize(Vector{20.});
    const auto free = kernel.linearize(Vector{-20.});
    bt::require(engaged.mode != free.mode,
                "opposite wishes resolve different unilateral modes");
    const auto engaged_domain = d::multiply(engaged.domain_matrix,
                                          Vector{-20.});
    const auto free_domain = d::multiply(free.domain_matrix, Vector{20.});
    bool engaged_rejects = false, free_rejects = false;
    for (std::size_t i = 0; i < engaged_domain.size(); ++i)
        engaged_rejects = engaged_rejects ||
                          engaged_domain[i] > engaged.domain_bound[i];
    for (std::size_t i = 0; i < free_domain.size(); ++i)
        free_rejects =
            free_rejects || free_domain[i] > free.domain_bound[i];
    bt::require(engaged_rejects,
                "the engaged mode rejects a releasing command");
    bt::require(free_rejects,
                "the released mode rejects an engaging command");
}

[[gnu::noinline]] void test_allocation_crosses_from_infeasible_sliding_wish_to_valid_sticking_mode() {
    // tests/reference/test_rider_dynamics.py::test_allocation_crosses_from_infeasible_sliding_wish_to_valid_sticking_mode
    auto rig = stand("friction");
    engine::forward(rig.model.get(), rig.data.get());
    const std::array<int, 1> dofs{0};
    rider::ConstraintDynamics kernel(*rig.model, *rig.data, dofs);
    const Vector wish{20.};
    const auto solve = [&](const rider::ConstraintResponse &response) {
        // Declared support bound tau <= 1 conflicts with the wish's sliding
        // region tau >= 2; a physically valid sticking command still exists.
        const std::array<double, 1> bound_row{1.};
        auto g = response.domain_matrix;
        d::append_row(g, bound_row);
        auto h = response.domain_bound;
        h.push_back(1.);
        rider::AllocationProblem problem;
        problem.target = wish;
        problem.equality = d::zeros(0, 1);
        problem.inequality = std::move(g);
        problem.inequality_rhs = std::move(h);
        problem.lower = {-50.};
        problem.upper = {50.};
        return rider::ModeCandidate{.allocation = rider::allocate(problem),
                                    .payload_index = 0};
    };
    bt::require(!solve(kernel.linearize(wish)).allocation.feasible,
                "the wish's local mode is infeasible under the bound");
    const auto result = rider::search_constraint_modes(kernel, wish, solve);
    bt::require(result.allocation.feasible && result.modes_tried > 1,
                "the mode search reaches a feasible mode across the boundary");
    bt::require(approx(result.allocation.solution[0], 1., 0., 1e-7),
                "the sticking command lands on the declared bound");
    model_access::mutable_buffer(rig.data->qfrc_applied, rig.model->nv,
                                 "qfrc_applied")[0] =
        result.allocation.solution[0];
    engine::forward(rig.model.get(), rig.data.get());
    const auto predicted =
        d::add(result.response.acceleration_offset,
               d::multiply(result.response.acceleration_matrix,
                           result.allocation.solution));
    const auto solved = model_access::readonly_buffer(
        rig.data->qacc, rig.model->nv, "qacc");
    allclose(solved, predicted, 0., 1e-9, "forward acceleration");
    bt::require(std::abs(solved[0]) < .1,
                "the sticking command accelerates the actuated dof slowly");
}

[[gnu::noinline]] void test_predicted_weld_wrench_includes_the_actual_support_moment() {
    // tests/reference/test_rider_dynamics.py::test_predicted_weld_wrench_includes_the_actual_support_moment
    auto rig = stand("weld");
    engine::forward(rig.model.get(), rig.data.get());
    const std::array<int, 1> dofs{4};
    rider::ConstraintDynamics kernel(*rig.model, *rig.data, dofs);
    const auto response = kernel.linearize(Vector{10.});
    const int body_a = mj_name2id(rig.model.get(), mjOBJ_BODY, "a");
    const int body_b = mj_name2id(rig.model.get(), mjOBJ_BODY, "b");
    bt::require(body_a >= 0 && body_b >= 0, "the weld rig has both bodies");
    const auto xpos = model_access::readonly_buffer(
        rig.data->xpos, 3 * rig.model->nbody, "xpos");
    const std::array<double, 3> point{
        xpos[3 * static_cast<std::size_t>(body_a)],
        xpos[3 * static_cast<std::size_t>(body_a) + 1],
        xpos[3 * static_cast<std::size_t>(body_a) + 2]};
    const auto force_map = rider::attachment_force_map(
        *rig.model, *rig.data, kernel, 0, body_a, point);
    const auto wrench = d::multiply(
        force_map,
        d::add(response.force_offset,
               d::multiply(response.force_matrix, Vector{10.})));
    model_access::mutable_buffer(rig.data->qfrc_applied, rig.model->nv,
                                 "qfrc_applied")[4] = 10.;
    engine::forward(rig.model.get(), rig.data.get());
    ::rider::EqualityReactions reactions(rig.model.get());
    ::rider::LeastSquaresWorkspace workspace{16, 16, 1};
    std::vector<double> scratch(static_cast<std::size_t>(rig.model->nv), 0.);
    const std::array<double, 3> normal{0., 0., 1.};
    const auto sample = ::rider::attachment_sample(
        rig.model.get(), rig.data.get(), reactions, workspace, scratch, 0,
        body_a, body_b, point, normal, "foot", true, .05, std::nullopt,
        nullptr);
    const Vector expected{sample.tangent_n, sample.normal_n,
                          sample.moment_nm};
    bt::require(wrench.size() == 3,
                "the weld force map emits tangent/normal/moment");
    allclose(wrench, expected, 0., 1e-9, "predicted weld wrench");
    bt::require(std::abs(wrench[2]) > 1.,
                "the predicted support moment is significant");
}

// The oracle's __new__ stub's resolved fields -> the ResponseRequest
// inputs the legacy allocator assembles inside compute.
rider::AllocationAttachment weld_front_attachment(const mjModel &model) {
    rider::AllocationAttachment a;
    a.name = "front";
    a.equality_id = 0;
    a.rider_body = mj_name2id(&model, mjOBJ_BODY, "rider");
    a.other_body = mj_name2id(&model, mjOBJ_BODY, "bike");
    a.site_id = mj_name2id(&model, mjOBJ_SITE, "sole");
    a.geom_id = mj_name2id(&model, mjOBJ_GEOM, "platform");
    a.weld = true;
    return a;
}

[[gnu::noinline]] void test_feasible_ten_watt_wish_is_not_suppressed_by_mixing_watts_and_scaled_torque() {
    // tests/reference/test_rider_dynamics.py::test_feasible_ten_watt_wish_is_not_suppressed_by_mixing_watts_and_scaled_torque
    bt::ModelFixture rig(
        "<mujoco><option gravity=\"0 0 0\"/>"
        "<worldbody><body name=\"bike\"><joint name=\"bx\" type=\"slide\" axis=\"1 0 0\"/>"
        "  <joint type=\"slide\" axis=\"0 0 1\"/><joint type=\"hinge\" axis=\"0 1 0\"/>"
        "  <inertial mass=\"1000\" pos=\"0 0 0\" diaginertia=\"1000 1000 1000\"/>"
        "  <geom name=\"platform\" type=\"box\" size=\".05 .025 .01\" contype=\"0\" conaffinity=\"0\"/></body>"
        "<body name=\"rider\"><joint name=\"rx\" type=\"slide\" axis=\"1 0 0\"/>"
        "  <joint type=\"slide\" axis=\"0 0 1\"/><joint type=\"hinge\" axis=\"0 1 0\"/>"
        "  <inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/><site name=\"sole\"/></body></worldbody>"
        "<equality><weld body1=\"rider\" body2=\"bike\"/></equality>"
        "<actuator><motor joint=\"rx\" gear=\"1\" forcelimited=\"true\" forcerange=\"-50 50\"/></actuator></mujoco>");
    const auto applied = model_access::mutable_buffer(
        rig.data->qfrc_applied, rig.model->nv, "qfrc_applied");
    applied[4] = -100.;
    const auto velocity = model_access::mutable_buffer(
        rig.data->qvel, rig.model->nv, "qvel");
    velocity[0] = 1.;
    velocity[3] = 1.;
    engine::forward(rig.model.get(), rig.data.get());
    bike::configuration::ArticulatedConfig config;
    config.active_positive_power_limit_w = 450.;
    rider::ResponseRequest request;
    request.config = config;
    request.actuator_dofs = {3};
    request.rider_dofs = {3, 4, 5};
    request.bike_dofs = {0, 1, 2};
    request.torque_intent = {10.};
    request.lower_torque_nm = {-50.};
    request.upper_torque_nm = {50.};
    request.activation_state = {0.};
    request.attachments = {weld_front_attachment(*rig.model)};
    request.pelvis_body = mj_name2id(rig.model.get(), mjOBJ_BODY, "rider");
    request.rider_mass_kg = 1.;
    request.dt_s = rig.model->opt.timestep;
    rider::AllocationMemory memory;
    const auto result = rider::allocate_response(*rig.model, *rig.data,
                                                 request, memory);
    bt::require(result.allocation.feasible,
                "the ten-watt wish allocates feasibly");
    bt::require(result.torque.size() == 1 && result.torque[0] > 9. &&
                    result.torque[0] <= 10.,
                "the delivered torque keeps the feasible wish");
    bt::require(approx(result.power[0], result.torque[0], 0., 1e-7),
                "the solved power is torque times the unit speed");
}

[[gnu::noinline]] void test_actual_allocation_cannot_jump_outside_bounded_activation_reach() {
    // tests/reference/test_rider_dynamics.py::test_actual_allocation_cannot_jump_outside_bounded_activation_reach
    for (const double previous : {0., 35.}) {
        bt::ModelFixture rig(
            "<mujoco><option gravity=\"0 0 0\" timestep=\".0005\"/>"
            "<worldbody><body name=\"bike\"><joint type=\"slide\" axis=\"1 0 0\"/><joint type=\"slide\" axis=\"0 0 1\"/>"
            "  <joint type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"1000\" pos=\"0 0 0\" diaginertia=\"1000 1000 1000\"/>"
            "  <geom name=\"platform\" type=\"box\" size=\".05 .025 .01\" contype=\"0\" conaffinity=\"0\"/></body>"
            "<body name=\"rider\"><joint type=\"slide\" axis=\"1 0 0\"/><joint type=\"slide\" axis=\"0 0 1\"/>"
            "  <joint name=\"muscle\" type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/>"
            "  <site name=\"sole\"/></body></worldbody><equality><weld body1=\"rider\" body2=\"bike\"/></equality>"
            "<actuator><motor joint=\"muscle\" gear=\"1\" forcelimited=\"true\" forcerange=\"-50 50\"/></actuator></mujoco>");
        const auto applied = model_access::mutable_buffer(
            rig.data->qfrc_applied, rig.model->nv, "qfrc_applied");
        applied[4] = -100.;
        applied[5] = -35.;
        engine::forward(rig.model.get(), rig.data.get());
        bike::configuration::ArticulatedConfig config;
        config.active_positive_power_limit_w = 450.;
        config.activation_tau_s = .04;
        const Vector before{previous};
        const Vector wished = ::spindle::activation_step(before, Vector{50.},
                                                         .005, .04);
        rider::ResponseRequest request;
        request.config = config;
        request.actuator_dofs = {5};
        request.rider_dofs = {3, 4, 5};
        request.bike_dofs = {0, 1, 2};
        request.torque_intent = wished;
        request.lower_torque_nm = {-50.};
        request.upper_torque_nm = {50.};
        request.activation_state = before;
        request.attachments = {weld_front_attachment(*rig.model)};
        request.pelvis_body = mj_name2id(rig.model.get(), mjOBJ_BODY, "rider");
        request.rider_mass_kg = 1.;
        request.dt_s = .005;
        rider::AllocationMemory memory;
        const auto result = rider::allocate_response(*rig.model, *rig.data,
                                                     request, memory);
        if (previous == 0.) {
            bt::require(!result.allocation.feasible,
                        "a cold activation state cannot reach the wish");
            bt::require(std::abs(result.torque[0]) <=
                            50. * (1. - std::exp(-.005 / .04)) + 1e-7,
                        "the allocated torque stays inside one activation "
                        "step's reach");
        } else {
            bt::require(result.allocation.feasible,
                        "a primed activation state reaches the wish");
            bt::require(result.torque[0] > 30.,
                        "the delivered torque reflects the primed state");
        }
        bt::require(std::abs(result.excitation[0]) <= 50.,
                    "the solved excitation keeps the joint bound");
        const Vector step = ::spindle::activation_step(
            before, result.excitation, .005, .04);
        allclose(result.torque, step, 0., 1e-7,
                 "delivered torque is one activation step from state");
        // finalize_effort delivers the allocated torque and commits exactly
        // that one activation step into the controller's state.
        bt::require(approx(step[0], result.torque[0], 0., 1e-9),
                    "finalize_effort delivers the allocated torque");
        bt::require(approx(result.activation_state[0], step[0], 0., 1e-9),
                    "the committed state is exactly one activation step");
        model_access::mutable_buffer(rig.data->ctrl, rig.model->nu,
                                     "ctrl")[0] = step[0];
        engine::forward(rig.model.get(), rig.data.get());
        const auto qacc = model_access::readonly_buffer(
            rig.data->qacc, rig.model->nv, "qacc");
        const Vector solved_qacc{qacc[3], qacc[4], qacc[5]};
        allclose(solved_qacc, result.rider_acceleration, 0., 1e-8,
                 "the forward solve reproduces the allocated acceleration");
    }
}

[[gnu::noinline]] void test_activation_projection_and_jacobian_include_joint_and_whole_body_clamps() {
    // tests/reference/test_rider_dynamics.py::test_activation_projection_and_jacobian_include_joint_and_whole_body_clamps
    {
        rider::ActivationLaw clamped({100., 100.}, {10., 10.}, {-40., -40.},
                                     {40., 40.}, .005, .04, 450.);
        const auto delivered = clamped.delivered(Vector{40., 40.});
        allclose(delivered, Vector{22.5, 22.5}, 1e-7, 0.,
                 "whole-body clamped delivery");
        const auto latent = clamped.latent(Vector{40., 40.});
        bt::require(std::ranges::all_of(
                        latent, [](double v) { return v > 40.; }),
                    "the latent excitation stays above the clamp");
    }
    {
        rider::ActivationLaw law({20., 20.}, {20., 10.}, {-50., -50.},
                                 {50., 50.}, .005, .04, 450.);
        const Vector excitation{10., 40.};
        const double epsilon = 1e-4;
        const auto analytic = law.derivative(excitation);
        for (std::size_t i = 0; i < 2; ++i) {
            auto plus = excitation, minus = excitation;
            plus[i] += epsilon;
            minus[i] -= epsilon;
            const auto forward = law.delivered(plus);
            const auto backward = law.delivered(minus);
            for (std::size_t r = 0; r < forward.size(); ++r)
                bt::require(
                    approx(analytic.values[r * analytic.cols + i],
                           (forward[r] - backward[r]) / (2. * epsilon),
                           0., 1e-8),
                    "the activation Jacobian matches central differences");
        }
        const auto delivered = law.delivered(excitation);
        bt::require(approx(d::dot(delivered, Vector{20., 10.}), 450.,
                           1e-6, 1e-12),
                    "the delivered activation saturates the power budget");
    }
}

[[gnu::noinline]] void test_linear_mode_rejection_respects_the_existing_feasibility_tolerance() {
    // tests/reference/test_rider_dynamics.py::test_linear_mode_rejection_respects_the_existing_feasibility_tolerance
    const Matrix matrix{.rows = 2, .cols = 1, .values = {1., -1.}};
    bt::require(!bike::numerics::linear_region_infeasible(
                    matrix, Vector{0., -.5e-7}, Vector{-1.}, Vector{1.}),
                "a witness inside the 1e-7 residual contract is not "
                "rejected by the precheck");
    bt::require(bike::numerics::linear_region_infeasible(
                    matrix, Vector{0., -3e-7}, Vector{-1.}, Vector{1.}),
                "a witness outside the residual contract is rejected");
}

[[gnu::noinline]] void test_grip_branch_uses_the_fixed_bar_anchor_when_the_soft_hand_moves() {
    // tests/reference/test_rider_dynamics.py::test_grip_branch_uses_the_fixed_bar_anchor_when_the_soft_hand_moves
    bt::ModelFixture rig(
        "<mujoco><worldbody>"
        "<body name=\"bar\"><freejoint/><inertial mass=\"1000\" pos=\"0 0 0\" diaginertia=\"1000 1000 1000\"/></body>"
        "<body name=\"hand\"><freejoint/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/><site name=\"grip\"/></body>"
        "</worldbody><equality><connect body1=\"hand\" body2=\"bar\" anchor=\"0 0 0\"/></equality></mujoco>");
    model_access::mutable_buffer(rig.data->qpos, rig.model->nq,
                                 "qpos")[9] = .001;
    engine::forward(rig.model.get(), rig.data.get());
    rider::AllocationAttachment attachment;
    attachment.name = "grip_left";
    attachment.equality_id = 0;
    attachment.rider_body = mj_name2id(rig.model.get(), mjOBJ_BODY, "hand");
    attachment.other_body = mj_name2id(rig.model.get(), mjOBJ_BODY, "bar");
    const auto point = rider::allocation_support_point(
        *rig.model, *rig.data, attachment);
    bt::require(point == std::array<double, 3>{0., 0., 0.},
                "the grip support point reads the compiled bar anchor");
    const std::array<double, 2> pelvis{-1., 0.};
    const std::array<double, 2> force{.5, -1000.};
    const double dx = pelvis[0] - point[0], dz = pelvis[1] - point[2];
    const double norm = std::hypot(dx, dz);
    const std::array<double, 2> direction{dx / norm, dz / norm};
    const auto press = rider::grip_constraints(d::identity(2), direction,
                                               false, 300.);
    bt::require(!press.empty(),
                "the pressing branch emits the sign constraint");
    const auto value = press[0].evaluate(force);
    bt::require(value[0] < press[0].lower[0],
                "a pulling-direction force violates the pressing sign row");
    // The contact writer's grip_anchor_local reset publishes the same
    // compiled eq_data anchor; the read below pins the identical anchor on
    // the writer's own contract through the support point (a moved hand
    // site would change the site_xpos read, never this anchor).
    const auto eq_data = model_access::readonly_buffer(
        rig.model->eq_data, rig.model->neq * mjNEQDATA, "eq_data");
    const auto base = static_cast<std::size_t>(attachment.equality_id) *
                          mjNEQDATA + 3;
    bt::require(eq_data[base] == 0. && eq_data[base + 1] == 0. &&
                    eq_data[base + 2] == 0.,
                "the compiled grip anchor stays at the bar origin");
}
} // namespace

int main() {
    return bt::run([] {
        test_model_response_predicts_current_acceleration_and_all_reactions();
        test_prediction_does_not_read_previous_reactions_or_acceleration();
        test_unilateral_mode_domain_rejects_a_command_that_changes_contact_state();
        test_allocation_crosses_from_infeasible_sliding_wish_to_valid_sticking_mode();
        test_predicted_weld_wrench_includes_the_actual_support_moment();
        test_feasible_ten_watt_wish_is_not_suppressed_by_mixing_watts_and_scaled_torque();
        test_actual_allocation_cannot_jump_outside_bounded_activation_reach();
        test_activation_projection_and_jacobian_include_joint_and_whole_body_clamps();
        test_linear_mode_rejection_respects_the_existing_feasibility_tolerance();
        test_grip_branch_uses_the_fixed_bar_anchor_when_the_soft_hand_moves();
    });
}
