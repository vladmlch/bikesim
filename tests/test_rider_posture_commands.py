import mujoco
import numpy as np
import pytest
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.rider_control import RiderCommand
from tests.test_rider_posture_tracking import rig


@pytest.mark.parametrize('kwargs', [dict(torso_lean_rad=2.), dict(pelvis_pitch_rad=float('nan')),
    dict(pelvis_offset_m=(0., 1.)), dict(use_saddle=1), dict(pelvis_offset_m=(0.,))])
def test_bad_posture_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RiderPosture(**kwargs)


def test_posture_changes_only_internal_goals(rig):
    m, d, c = rig
    q, v = d.qpos.copy(), d.qvel.copy()
    neutral = c.compute(m, d, RiderCommand())
    posture = RiderPosture(torso_lean_rad=.12, pelvis_offset_m=(.02, .03))
    moved = c.compute(m, d, RiderCommand(posture=posture))
    np.testing.assert_array_equal(d.qpos, q)
    np.testing.assert_array_equal(d.qvel, v)
    assert moved.keys() == neutral.keys()
    assert any(abs(moved[k]-neutral[k]) > 1e-3 for k in moved)
    assert max(abs(value) for value in moved.values()) <= c.config.joint_limit_nm
    c.write(d, moved)
    mujoco.mj_forward(m, d)
    root = int(m.joint('root_pitch').dofadr[0])
    assert d.qfrc_actuator[root] == 0.


def test_standing_goal_does_not_disable_actual_contact(rig):
    m, d, c = rig
    c.compute(m, d, RiderCommand(posture=RiderPosture.standing()),
              contact_loads={'front': 100., 'rear': 100., 'saddle': 200., 'grip': True})
    assert c.support_diagnostics['requested_vertical_forces_n']['saddle'] == 0.
    assert c.support_diagnostics['posture']['use_saddle'] is False
