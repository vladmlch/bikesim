// Ports all seven source test functions in
// tests/reference/test_leg_loop.py — loop closure and virtual-work
// derivatives in the native +y convention. The five pure records exercise
// spindle_math's leg-loop helpers directly; the two slow records replay the
// _compiled/_model pinned plants from test_pinned_topology.py through
// rider_control_reference_support.hpp.
#include "support/fixture_test.hpp"
#include "rider_control_reference_support.hpp"
#include "rider/spindle_math.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace {
namespace rr = rider_control_reference;
using spindle::Vec2;

// _geometry() (test_leg_loop.py:12-18): the oracle constructs
// LegLoopGeometry field-by-field over the same norms/angles
// leg_loop_geometry computes — including knee_sign, which resolves to 1.
// for this triangle — so the helper produces the bitwise-identical struct.
const Vec2 kHip{0., .70}, kKnee{.25, .35}, kPedal{.17, -.10};
spindle::LegLoopGeometry geometry() {
    return spindle::leg_loop_geometry(kHip, kKnee, kPedal, true);
}

// leg_loop.py's two-argument leg_loop_geometry(pose, side) — pose-field
// selection plus the same geometric reduction.
spindle::LegLoopGeometry pose_geometry(const rider::SpindlePose &pose,
                                       bool front) {
    const auto &knee = front ? pose.knee_front : pose.knee_rear;
    const auto &pedal = front ? pose.pedal_front : pose.pedal_rear;
    return spindle::leg_loop_geometry({pose.hip[0], pose.hip[2]},
                                      {knee[0], knee[2]},
                                      {pedal[0], pedal[2]}, front);
}

[[gnu::noinline]] void test_build_pose_gives_zero_q() {
    // leg_loop/build_pose_gives_zero_q
    const auto g = geometry();
    const auto q = spindle::leg_joint_q(g, kHip, kPedal, 0.);
    bt::require(rr::approx(q[0], 0., 1e-6, 1e-9) &&
                    rr::approx(q[1], 0., 1e-6, 1e-9),
                "the build pose solves to zero joint coordinates");
}

[[gnu::noinline]] void test_pelvis_pitch_shifts_hip_only() {
    // leg_loop/pelvis_pitch_shifts_hip_only
    const auto g = geometry();
    const auto q = spindle::leg_joint_q(g, kHip, kPedal, .1);
    bt::require(rr::approx(q[0], -.1, 1e-6, 1e-9) &&
                    rr::approx(q[1], 0., 1e-6, 1e-9),
                "pelvis pitch enters the hip coordinate only");
}

[[gnu::noinline]] void test_jacobian_matches_finite_difference_of_ik() {
    // leg_loop/jacobian_matches_finite_difference_of_ik
    const auto g = geometry();
    const Vec2 crank{0., 0.};
    const double phi = .7, d = 1e-5;
    const auto jac =
        spindle::leg_loop_jacobian(g, kHip, crank, .17, phi, 0.);
    const auto qp = spindle::leg_joint_q(
        g, kHip, spindle::spindle_xz(crank, .17, phi + d, 1.), 0.);
    const auto qm = spindle::leg_joint_q(
        g, kHip, spindle::spindle_xz(crank, .17, phi - d, 1.), 0.);
    const std::array<double, 2> manual{(qp[0] - qm[0]) / (2. * d),
                                       (qp[1] - qm[1]) / (2. * d)};
    rr::allclose(jac, manual, 1e-5, 1e-6,
                 "the analytic Jacobian matches the IK finite difference");
}

[[gnu::noinline]] void test_unreachable_raises() {
    // leg_loop/unreachable_raises
    const auto g = geometry();
    for (const Vec2 point : {Vec2{0., -5.}, Vec2{0., .7}})
        rr::expect_invalid(
            [&] { (void)spindle::leg_joint_q(g, kHip, point, 0.); },
            "unreachable");
}

[[gnu::noinline]] void test_ik_derivative_unwraps_near_angle_branch_cut() {
    // leg_loop/ik_derivative_unwraps_near_angle_branch_cut
    const spindle::LegLoopGeometry g{.thigh_m = 1.,
                                     .link_m = 1.,
                                     .knee_sign = 1.,
                                     .thigh_angle0_rad = 0.,
                                     .link_angle0_rad = 0.,
                                     .side_sign = 1.};
    const auto jac =
        spindle::leg_loop_jacobian(g, Vec2{0., 0.}, Vec2{0., 0.}, 1.,
                                   -std::numbers::pi / 2., 0.);
    bt::require(rr::approx(jac[0], 1., 1e-6, 1e-8) &&
                    rr::approx(jac[1], 0., 1e-6, 1e-8),
                "the derivative unwraps across the -pi/pi branch cut");
}

[[gnu::noinline]] void
test_native_spindle_and_leg_pose_including_bike_pitch() {
    // leg_loop/native_spindle_and_leg_pose_including_bike_pitch
    auto const plant = rr::pinned_model();
    mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    const auto S = [](int i) { return static_cast<std::size_t>(i); };
    auto const qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto adrs =
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const int crank = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    const int root_pitch = mj_name2id(m, mjOBJ_JOINT, "root_pitch");
    bt::require(crank >= 0 && root_pitch >= 0,
                "the pinned plant carries crank_spin and root_pitch");
    const auto xanchor = model_access::readonly_buffer(
        d->xanchor, m->njnt * 3, "xanchor");
    const auto xpos =
        model_access::readonly_buffer(d->xpos, m->nbody * 3, "xpos");
    for (const double bike_pitch : {0., .2}) {
        qpos[S(adrs[S(root_pitch)])] = bike_pitch;
        for (const double phase : {0., .7, 2.2}) {
            qpos[S(adrs[S(crank)])] = phase;
            engine::forward(m, d);
            const Vec2 anchor{xanchor[3 * S(crank)],
                              xanchor[3 * S(crank) + 2]};
            for (const auto &[side, sign] :
                 {std::pair{"front", 1.}, std::pair{"rear", -1.}}) {
                const int body = mj_name2id(
                    m, mjOBJ_BODY, (std::string("pedal_") + side).c_str());
                const std::array<double, 2> actual{xpos[3 * S(body)],
                                                   xpos[3 * S(body) + 2]};
                const auto expected = spindle::spindle_xz(
                    anchor, .165, phase, sign, bike_pitch);
                rr::allclose(expected, actual, 1e-5, 1e-6,
                             std::string("spindle position, ") + side +
                                 " pedal");
            }
        }
    }
    // Read the native foot endpoint at a native pose; the pure IK must
    // recover the joint coordinates, including pelvis pitch and
    // build-angle offsets.
    const auto qpos0 =
        model_access::readonly_buffer(m->qpos0, m->nq, "qpos0");
    for (std::size_t i = 0; i < qpos.size(); ++i)
        qpos[i] = qpos0[i];
    const auto address_of = [&](const char *name) {
        const int id = mj_name2id(m, mjOBJ_JOINT, name);
        bt::require(id >= 0,
                    std::string("the pinned plant carries ") + name);
        return S(adrs[S(id)]);
    };
    qpos[address_of("rider_root_pitch")] = .12;
    for (const char *side : {"front", "rear"}) {
        qpos[address_of((std::string("rider_hip_") + side).c_str())] = .15;
        qpos[address_of((std::string("rider_knee_") + side).c_str())] = -.1;
    }
    engine::forward(m, d);
    const auto xmat =
        model_access::readonly_buffer(d->xmat, m->nbody * 9, "xmat");
    const auto eq_data = model_access::readonly_buffer(
        m->eq_data, m->neq * mjNEQDATA, "eq_data");
    const int pelvis = mj_name2id(m, mjOBJ_BODY, "rider_pelvis");
    const Vec2 hip{xpos[3 * S(pelvis)], xpos[3 * S(pelvis) + 2]};
    for (const bool front : {true, false}) {
        const std::string side = front ? "front" : "rear";
        const int eq = mj_name2id(m, mjOBJ_EQUALITY,
                                  ("connect_foot_" + side).c_str());
        const int foot = mj_name2id(m, mjOBJ_BODY,
                                    ("rider_foot_" + side).c_str());
        const std::size_t eqo =
            static_cast<std::size_t>(eq) * static_cast<std::size_t>(mjNEQDATA);
        const std::size_t xf = 9 * S(foot);
        const Vec2 endpoint{
            xpos[3 * S(foot)] + xmat[xf] * eq_data[eqo] +
                xmat[xf + 1] * eq_data[eqo + 1] +
                xmat[xf + 2] * eq_data[eqo + 2],
            xpos[3 * S(foot) + 2] + xmat[xf + 6] * eq_data[eqo] +
                xmat[xf + 7] * eq_data[eqo + 1] +
                xmat[xf + 8] * eq_data[eqo + 2]};
        const auto g = pose_geometry(plant.pose, front);
        const auto q = spindle::leg_joint_q(g, hip, endpoint, .12);
        bt::require(rr::approx(q[0], .15, 1e-6, 1e-8) &&
                        rr::approx(q[1], -.1, 1e-6, 1e-8),
                    "IK recovers the " + side + " leg joint coordinates");
    }
}

[[gnu::noinline]] void test_leg_ik_matches_settled_native_joints() {
    // leg_loop/leg_ik_matches_settled_native_joints
    auto rig = rr::pinned_rig();
    mjModel const *m = rig->stepper->model();
    mjData const *d = rig->stepper->data();
    const auto xpos =
        model_access::readonly_buffer(d->xpos, m->nbody * 3, "xpos");
    const auto xmat =
        model_access::readonly_buffer(d->xmat, m->nbody * 9, "xmat");
    const auto site_xpos = model_access::readonly_buffer(
        d->site_xpos, m->nsite * 3, "site_xpos");
    const auto qpos =
        model_access::readonly_buffer(d->qpos, m->nq, "qpos");
    const auto adrs =
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const auto S = [](int i) { return static_cast<std::size_t>(i); };
    const int pelvis = mj_name2id(m, mjOBJ_BODY, "rider_pelvis");
    const Vec2 hip{xpos[3 * S(pelvis)], xpos[3 * S(pelvis) + 2]};
    const double pitch =
        std::atan2(xmat[9 * S(pelvis) + 2], xmat[9 * S(pelvis)]);
    for (const bool front : {true, false}) {
        const std::string side = front ? "front" : "rear";
        const int site = mj_name2id(m, mjOBJ_SITE,
                                    ("site_pedal_" + side).c_str());
        const Vec2 spindle_pos{site_xpos[3 * S(site)],
                               site_xpos[3 * S(site) + 2]};
        const std::array<double, 2> expected{
            qpos[S(adrs[S(mj_name2id(m, mjOBJ_JOINT,
                                    ("rider_hip_" + side).c_str()))])],
            qpos[S(adrs[S(mj_name2id(m, mjOBJ_JOINT,
                                    ("rider_knee_" + side).c_str()))])]};
        const auto q = spindle::leg_joint_q(
            pose_geometry(rig->pose, front), hip, spindle_pos, pitch);
        bt::require(rr::approx(q[0], expected[0], 1e-6, .02) &&
                        rr::approx(q[1], expected[1], 1e-6, .02),
                    "IK tracks the settled " + side + " joint coordinates");
    }
}
} // namespace

int main() {
    return bt::run([] {
        test_build_pose_gives_zero_q();
        test_pelvis_pitch_shifts_hip_only();
        test_jacobian_matches_finite_difference_of_ik();
        test_unreachable_raises();
        test_ik_derivative_unwraps_near_angle_branch_cut();
        test_native_spindle_and_leg_pose_including_bike_pitch();
        test_leg_ik_matches_settled_native_joints();
    });
}
