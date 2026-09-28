import numpy as np
import pytest
from math import pi

from bike_sim.physics.drivetrain import ripple_shape
from bike_sim.physics.rider import (
    ANKLE_ABOVE_PEDAL_M, RiderSpecs, crank_torque_share, pedal_spindle_pos,
    solve_leg_qpos, solve_leg_joints,
)


@pytest.fixture(scope="module")
def pose():
    return RiderSpecs(variant="seated", legs="articulated").seated_pose()


def test_articulated_pose_has_leg_chains_no_leg_bodies(pose):
    assert len(pose.leg_chains) == 2
    names = {b.name for b in pose.bodies}
    assert names == {"rider_pelvis", "rider_torso", "rider_arms"}


def test_rigid_pose_unchanged():
    pose = RiderSpecs(variant="seated").seated_pose()  # legs defaults to "rigid"
    assert pose.leg_chains == ()
    assert {b.name for b in pose.bodies} == {
        "rider_pelvis", "rider_torso", "rider_arms", "rider_leg_front", "rider_leg_rear"}


def test_design_pose_is_zero_qpos(pose):
    # The model is built at phase 0 (horizontal cranks): IK must reproduce qpos=0,
    # which is what makes the weld datum consistent at build time.
    for chain in pose.leg_chains:
        q = solve_leg_qpos(chain, phase_rad=0.0)
        np.testing.assert_allclose(q, np.zeros(3), atol=1e-9)


def test_ankle_tracks_pedal_circle(pose):
    phases = np.linspace(0.0, 2.0 * pi, 73)
    for chain in pose.leg_chains:
        for phi in phases:
            # must never be unreachable through a revolution
            knee, ankle = solve_leg_joints(chain, phi)
            pedal = pedal_spindle_pos(phi, chain.lateral_y_m, chain.crank_len_m)
            np.testing.assert_allclose(
                [ankle[0], ankle[2]], [pedal[0], pedal[2] + ANKLE_ABOVE_PEDAL_M],
                atol=1e-9)


def test_knee_flexion_stays_in_human_range(pose):
    # 20-60 deg is the accepted BDC band (physics/rider.py); allow margin for the
    # top of the stroke but refuse folded-back or locked-out knees.
    for chain in pose.leg_chains:
        flexions = []
        for phi in np.linspace(0.0, 2.0 * pi, 37):
            knee, ankle = solve_leg_joints(chain, phi)
            hip = chain.hip
            v1, v2 = hip - knee, ankle - knee
            flexions.append(180.0 - np.degrees(
                np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))))
        assert min(flexions) > 15.0 and max(flexions) < 75.0


def test_pelvis_slide_moves_the_hip(pose):
    # The hip anchor rides the pelvis slide joint: raising the pelvis must move
    # the IK knee up (ankle stays welded to its pedal circle).
    chain = pose.leg_chains[0]
    knee0, _ = solve_leg_joints(chain, pi / 2, pelvis_z_m=0.0)
    knee1, _ = solve_leg_joints(chain, pi / 2, pelvis_z_m=0.03)
    assert knee1[2] > knee0[2]


def test_torque_shares_sum_to_ripple():
    for depth in (0.0, 0.5, 0.85, 1.0):
        for phi in np.linspace(0.0, 2.0 * pi, 65):
            total = crank_torque_share(phi, depth) + crank_torque_share(phi + pi, depth)
            assert total == pytest.approx(ripple_shape(phi, depth), abs=1e-12)
        # each leg averages half the mean torque over a revolution
        mean = np.mean([crank_torque_share(p, depth) for p in np.linspace(0, 2 * pi, 1001)])
        assert mean == pytest.approx(0.5, abs=1e-3)


def test_leg_masses_match_rigid_path(pose):
    # The articulated leg carries the same pedal-path mass the slide leg did.
    rigid = RiderSpecs(variant="seated").seated_pose()
    rigid_leg = rigid.body("rider_leg_front").mass
    chain = pose.leg_chains[0]
    assert chain.thigh_mass_kg + chain.shank_mass_kg + chain.foot_mass_kg == pytest.approx(rigid_leg)
