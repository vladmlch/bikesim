import numpy as np
import pytest
from math import pi

from bike_sim.physics.drivetrain import ripple_shape
from bike_sim.physics.rider import (
    ANKLE_ABOVE_PEDAL_M, RiderSpecs, crank_torque_share, fk_leg, leg_jacobian,
    pedal_spindle_pos, solve_leg_qpos, solve_leg_joints,
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
    # 20-60 deg is the accepted BDC band (physics/rider.py); at top of stroke the
    # geometry reaches ~106 deg, which is the correct human value there. The 115 deg
    # ceiling still refuses folded-back knees while allowing the real TDC flexion.
    for chain in pose.leg_chains:
        flexions = []
        for phi in np.linspace(0.0, 2.0 * pi, 37):
            knee, ankle = solve_leg_joints(chain, phi)
            hip = chain.hip
            v1, v2 = hip - knee, ankle - knee
            flexions.append(180.0 - np.degrees(
                np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))))
        assert min(flexions) > 15.0 and max(flexions) < 115.0


def test_pelvis_slide_moves_the_hip(pose):
    # The hip anchor rides the pelvis slide joint: raising the pelvis must move
    # the IK knee up (ankle stays welded to its pedal circle).
    chain = pose.leg_chains[0]
    knee0, _ = solve_leg_joints(chain, pi, pelvis_z_m=0.0)
    knee1, _ = solve_leg_joints(chain, pi, pelvis_z_m=0.03)
    assert knee1[2] > knee0[2]


def test_torque_shares_sum_to_ripple():
    for depth in (0.0, 0.5, 0.85, 1.0):
        for phi in np.linspace(0.0, 2.0 * pi, 65):
            total = crank_torque_share(phi, depth) + crank_torque_share(phi + pi, depth)
            assert total == pytest.approx(ripple_shape(phi, depth), abs=1e-12)
        # each leg averages half the mean torque over a revolution
        mean = np.mean([crank_torque_share(p, depth) for p in np.linspace(0, 2 * pi, 1000, endpoint=False)])
        assert mean == pytest.approx(0.5, abs=1e-3)


def test_fk_leg_round_trips_the_ik(pose):
    # fk_leg is the Jacobian's geometry source: it must reproduce the IK points
    # exactly (x/z; fk lives on the sagittal plane so y is omitted) for the qpos
    # that solve_leg_qpos produces at every crank phase.
    for chain in pose.leg_chains:
        for phi in np.linspace(0.0, 2.0 * pi, 37):
            knee_fk, ankle_fk, _ = fk_leg(chain, solve_leg_qpos(chain, phi), pelvis_z_m=0.0)
            knee_ik, ankle_ik = solve_leg_joints(chain, phi)
            np.testing.assert_allclose(
                [knee_fk[0], knee_fk[2]], [knee_ik[0], knee_ik[2]], atol=1e-9)
            np.testing.assert_allclose(
                [ankle_fk[0], ankle_fk[2]], [ankle_ik[0], ankle_ik[2]], atol=1e-9)


def test_leg_jacobian_matches_finite_difference(pose):
    # leg_jacobian rows are hinge torques per unit pedal-spindle force, i.e. the
    # transpose of the pedal point's position Jacobian w.r.t. the hinge angles.
    # Check against central differences of fk_leg's pedal point mid-stroke.
    eps = 1e-6
    for chain in pose.leg_chains:
        q = solve_leg_qpos(chain, pi / 4)
        J = leg_jacobian(chain, q)
        J_fd = np.empty((3, 2))
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = eps
            _, _, p_fwd = fk_leg(chain, q + dq)
            _, _, p_bwd = fk_leg(chain, q - dq)
            J_fd[i] = ((p_fwd - p_bwd) / (2.0 * eps))[[0, 2]]
        np.testing.assert_allclose(J, J_fd, rtol=1e-4, atol=1e-4)


def test_leg_jacobian_propulsive_couple_signs(pose):
    # A downward push on a horizontal forward pedal is the propulsive stroke: it
    # must load the hip positively and flex the knee negatively (about +y).
    down = np.array([0.0, -1.0])
    for chain in pose.leg_chains:
        tau = leg_jacobian(chain, np.zeros(3)) @ down
        assert tau[0] > 0.0 and tau[1] < 0.0


def test_leg_masses_match_rigid_path(pose):
    # The articulated leg carries the same pedal-path mass the slide leg did.
    rigid = RiderSpecs(variant="seated").seated_pose()
    rigid_leg = rigid.body("rider_leg_front").mass
    chain = pose.leg_chains[0]
    assert chain.thigh_mass_kg + chain.shank_mass_kg + chain.foot_mass_kg == pytest.approx(rigid_leg)


def test_articulated_total_mass_and_interface_loads(pose):
    # Articulated leg mass counts toward the rider total (equilibrium consumers
    # read it), while the pedal interface's *spring* load is zero: the joint/weld
    # path, not a slide spring, carries the legs.
    rigid = RiderSpecs(variant="seated").seated_pose()
    assert pose.total_mass_kg == pytest.approx(rigid.total_mass_kg)
    assert pose.interface_loads_n()["pedals"] == 0.0
