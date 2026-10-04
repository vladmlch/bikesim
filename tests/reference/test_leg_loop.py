"""Loop closure and virtual-work derivatives in the native +y convention."""
import math

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.leg_loop import (LegLoopGeometry, leg_joint_q,
    leg_loop_geometry, leg_loop_jacobian, planar_angle, spindle_xz)


def _geometry():
    hip = np.array([0., .70])
    knee = np.array([.25, .35])
    pedal = np.array([.17, -.10])
    return hip, knee, pedal, LegLoopGeometry(
        float(np.linalg.norm(knee - hip)), float(np.linalg.norm(pedal - knee)), 1.,
        planar_angle(knee - hip), planar_angle(pedal - knee), 1.)


def test_build_pose_gives_zero_q():
    hip, _, pedal, g = _geometry()
    assert leg_joint_q(g, hip, pedal, 0.) == pytest.approx((0., 0.), abs=1e-9)


def test_pelvis_pitch_shifts_hip_only():
    hip, _, pedal, g = _geometry()
    assert leg_joint_q(g, hip, pedal, .1) == pytest.approx((-.1, 0.), abs=1e-9)


def test_jacobian_matches_finite_difference_of_ik():
    hip, _, _, g = _geometry()
    crank = np.zeros(2)
    phi, d = .7, 1e-5
    jac = leg_loop_jacobian(g, hip, crank, .17, phi, 0.)
    qp = leg_joint_q(g, hip, spindle_xz(crank, .17, phi+d, 1.), 0.)
    qm = leg_joint_q(g, hip, spindle_xz(crank, .17, phi-d, 1.), 0.)
    assert np.allclose(jac, (np.array(qp)-qm)/(2*d), atol=1e-6)


@pytest.mark.parametrize('point', [[0., -5.], [0., .7]])
def test_unreachable_raises(point):
    hip, _, _, g = _geometry()
    with pytest.raises(ValueError, match='unreachable'):
        leg_joint_q(g, hip, point, 0.)


def test_ik_derivative_unwraps_near_angle_branch_cut():
    g = LegLoopGeometry(1., 1., 1., 0., 0., 1.)
    jac = leg_loop_jacobian(g, np.zeros(2), np.zeros(2), 1., -math.pi/2, 0.)
    assert jac == pytest.approx((1., 0.), abs=1e-8)


@pytest.mark.slow
def test_native_spindle_and_leg_pose_including_bike_pitch(tmp_path):
    from test_pinned_topology import _compiled
    model, controller = _compiled(tmp_path)
    data = mujoco.MjData(model)
    crank = model.joint('crank_spin')
    for bike_pitch in (0., .2):
        data.qpos[model.joint('root_pitch').qposadr[0]] = bike_pitch
        for phase in (0., .7, 2.2):
            data.qpos[crank.qposadr[0]] = phase
            mujoco.mj_forward(model, data)
            anchor = data.xanchor[crank.id][[0, 2]]
            for side, sign in [('front', 1.), ('rear', -1.)]:
                actual = data.xpos[model.body('pedal_'+side).id][[0, 2]]
                assert np.allclose(spindle_xz(anchor, .165, phase, sign,
                                            frame_pitch_rad=bike_pitch), actual, atol=1e-6)
    # Read the native foot endpoint at a native pose; the pure IK must recover
    # the joint coordinates, including pelvis pitch and build-angle offsets.
    data.qpos[:] = model.qpos0
    data.qpos[model.joint('rider_root_pitch').qposadr[0]] = .12
    for side in ('front', 'rear'):
        data.qpos[model.joint('rider_hip_'+side).qposadr[0]] = .15
        data.qpos[model.joint('rider_knee_'+side).qposadr[0]] = -.1
    mujoco.mj_forward(model, data)
    hip = data.xpos[model.body('rider_pelvis').id][[0, 2]]
    for side in ('front', 'rear'):
        eq = model.equality('connect_foot_'+side).id
        foot = model.body('rider_foot_'+side).id
        endpoint = data.xpos[foot] + data.xmat[foot].reshape(3,3) @ model.eq_data[eq,:3]
        g = leg_loop_geometry(controller.pose, side)
        assert leg_joint_q(g, hip, endpoint[[0,2]], .12) == pytest.approx((.15, -.1), abs=1e-8)


@pytest.mark.slow
def test_leg_ik_matches_settled_native_joints(tmp_path):
    from test_pinned_topology import _model
    _, env = _model(tmp_path)
    model, data = env.sim.model, env.sim.data
    controller = env.sim.physical.rider_control
    hip = data.xpos[model.body('rider_pelvis').id][[0, 2]]
    rotation = data.xmat[model.body('rider_pelvis').id].reshape(3,3)
    pitch = math.atan2(rotation[0,2], rotation[0,0])
    for side in ('front', 'rear'):
        spindle = data.site_xpos[model.site('site_pedal_'+side).id][[0,2]]
        expected = [data.qpos[model.joint('rider_'+joint+'_'+side).qposadr[0]] for joint in ('hip','knee')]
        assert leg_joint_q(leg_loop_geometry(controller.pose, side), hip, spindle, pitch) == pytest.approx(expected, abs=.02)
