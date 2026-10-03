"""Two-arm reference rider: anatomical split, per-hand grip, 11-joint registry.

The paired-arm collapse (`upper_arm_pair`/`forearm_pair`, one midline grip
point) is replaced by two planar arm chains, each with its own shoulder,
elbow, forearm body and grip site. The split creates no mass or strength:
each side carries exactly half of its paired segment, and the hand's mass
stays inside its forearm.
"""
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.reference_rider import reference_joint_names
from bike_sim.physics.rider_segments import (
    ARM_LATERAL_OFFSET_M, segment_masses, split_paired_arm_masses,
)
from bike_sim.physics.rider_envelope import (
    RIDER_JOINTS, load_joint_envelopes,
)

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'
ENVELOPE = ROOT / 'examples' / 'research' / 'rider_joint_envelope_anatomical.json'


def test_split_preserves_the_anatomical_budget():
    paired = segment_masses(80., .5)
    split = split_paired_arm_masses(paired)
    assert set(split) == (set(paired) - {'upper_arm_pair', 'forearm_pair'}) | {
        'upper_arm_left', 'upper_arm_right', 'forearm_left', 'forearm_right'}
    assert sum(split.values()) == pytest.approx(sum(paired.values()), abs=1e-12)
    for part in ('upper_arm', 'forearm'):
        for side in ('left', 'right'):
            assert split[f'{part}_{side}'] == pytest.approx(paired[part + '_pair'] / 2.)


def test_arm_split_does_not_add_mass_or_strength():
    # Plan task R2, step 1: identical total, halved per-hand shares, and the
    # shared shoulder DOF is gone from the registry.
    old = segment_masses(80., 0.)
    new = split_paired_arm_masses(old)
    assert math.isclose(sum(new.values()), 80., abs_tol=1e-10)
    for part in ('upper_arm', 'forearm'):
        assert math.isclose(new[f'{part}_left'] + new[f'{part}_right'],
                            old[part + '_pair'], abs_tol=1e-10)
    names = reference_joint_names()
    assert len(names) == 11
    assert 'rider_shoulder' not in names
    assert 'rider_shoulder_left' in names and 'rider_shoulder_right' in names


def test_split_preserves_com_and_sagittal_inertia():
    # Two half-mass segments at the same geometry keep the paired COM and the
    # same sagittal moment of inertia about the shoulder pivot.
    from bike_sim.physics.rider import RiderSpecs
    rider = RiderSpecs(variant='articulated_planar')
    props = rider.compute_rider_centers_of_mass()
    arms = [key for key in props if 'arm' in key]
    assert set(arms) == {'rider_upper_arm_left', 'rider_forearm_left',
                         'rider_upper_arm_right', 'rider_forearm_right'}
    mass = sum(props[key][1] for key in arms)
    com_xz = (sum(props[key][1] * props[key][0][[0, 2]] for key in arms) / mass)
    inertia = sum(props[key][1] * float(props[key][0][[0, 2]] @ props[key][0][[0, 2]])
                  for key in arms)
    # A single equivalent pair at the midline would give the same totals.
    paired = segment_masses(rider.mass_kg, rider.helmet_mass_kg)
    pair_mass = paired['upper_arm_pair'] + paired['forearm_pair']
    assert mass == pytest.approx(pair_mass, rel=1e-12)
    for key in arms:
        side = -1. if key.endswith('_left') else 1.
        assert props[key][0][1] == pytest.approx(side * ARM_LATERAL_OFFSET_M)
    assert np.isfinite(com_xz).all() and inertia > 0.


def test_hand_mass_stays_inside_each_forearm():
    # No wrist segment: each forearm body carries forearm+hand, halved.
    paired = segment_masses(80., .5)
    split = split_paired_arm_masses(paired)
    body = 80. - .5  # the helmet's mass lives in the head, not the body sum
    f = {'forearm': .0162, 'hand': .0061}
    for side in ('left', 'right'):
        assert split[f'forearm_{side}'] == pytest.approx((f['forearm'] + f['hand']) * body)


def test_registry_names_eleven_internal_joints():
    expected = ('rider_torso_hinge',
                'rider_shoulder_left', 'rider_elbow_left',
                'rider_shoulder_right', 'rider_elbow_right',
                'rider_hip_front', 'rider_knee_front', 'rider_ankle_front',
                'rider_hip_rear', 'rider_knee_rear', 'rider_ankle_rear')
    assert reference_joint_names() == expected
    assert RIDER_JOINTS == expected
    assert not any(name.startswith('rider_root') for name in expected)


def test_envelope_schema_two_requires_the_two_arm_topology():
    envelopes = load_joint_envelopes(str(ENVELOPE))
    assert set(envelopes) == set(RIDER_JOINTS)
    for side in ('left', 'right'):
        for joint in ('shoulder', 'elbow'):
            name = f'rider_{joint}_{side}'
            assert (envelopes[name].minimum_anatomical_rad
                    < envelopes[name].neutral_anatomical_rad
                    < envelopes[name].maximum_anatomical_rad)
    payload = json.loads(ENVELOPE.read_text())
    payload['topology'] = 'articulated_planar'
    bad = ENVELOPE.parent / '_bad_topology.json'
    try:
        bad.write_text(json.dumps(payload))
        with pytest.raises(ValueError, match='different rider topology'):
            load_joint_envelopes(str(bad))
    finally:
        bad.unlink(missing_ok=True)


def test_envelope_schema_missing_arm_fails_closed(tmp_path):
    payload = json.loads(ENVELOPE.read_text())
    del payload['joints']['rider_shoulder_left']
    target = tmp_path / 'missing_arm.json'
    target.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match='internal rider joints'):
        load_joint_envelopes(str(target))


def _environment(tmp_path, config=WELDED):
    from bike_sim.cli import research as research_cli
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', '1.', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0',
        '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


@pytest.mark.slow
def test_compiled_topology_has_two_arms_and_two_grip_sites(tmp_path):
    env = _environment(tmp_path)
    model = env.sim.model
    for side, sign in (('left', -1.), ('right', 1.)):
        for part in ('upper_arm', 'forearm'):
            body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                         f'rider_{part}_{side}'))
            assert body >= 0
        site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                     f'site_rider_grip_{side}'))
        assert site >= 0
        eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                   f'connect_grip_{side}'))
        assert eq >= 0
        assert int(model.eq_obj1id[eq]) == int(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f'rider_forearm_{side}'))
        assert int(model.eq_obj2id[eq]) == int(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, 'steer'))
        for joint in ('shoulder', 'elbow'):
            name = f'rider_{joint}_{side}'
            assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)) >= 0
            assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                       f'act_{name}')) >= 0
    # The midline pair bodies and single grip point are gone.
    for name in ('rider_upper_arm_pair', 'rider_forearm_pair'):
        assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)) < 0
    assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'site_rider_grip')) < 0
    # Each hand grasps its own half-width point on the bar.
    data = env.sim.data
    for side, sign in (('left', -1.), ('right', 1.)):
        site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                     f'site_rider_grip_{side}'))
        assert data.site_xpos[site][1] == pytest.approx(sign * ARM_LATERAL_OFFSET_M, abs=.01)
    # Eleven internal actuated joints; no actuator drives a root coordinate.
    internal = set(reference_joint_names())
    assert internal <= {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        for j in range(model.njnt)}
    root_dofs = {int(model.joint(n).dofadr[0])
                 for n in ('rider_root_x', 'rider_root_z', 'rider_root_pitch')}
    for a in range(model.nu):
        assert int(model.actuator_trnid[a, 0]) not in {
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n))
            for n in ('rider_root_x', 'rider_root_z', 'rider_root_pitch')}


@pytest.mark.slow
def test_each_hand_jacobian_loads_only_its_own_arm(tmp_path):
    """A force at site_rider_grip_<side> torques only that side's arm chain.

    tau_elbow = (r x F)_y with r the elbow->site lever; the shoulder carries
    the moment of the whole arm. The opposite arm's actuator columns are
    identically zero -- contact ownership is not shared.
    """
    env = _environment(tmp_path)
    model, data = env.sim.model, env.sim.data
    for side, other in (('left', 'right'), ('right', 'left')):
        forearm = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                        f'rider_forearm_{side}'))
        site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                     f'site_rider_grip_{side}'))
        point = data.site_xpos[site]
        jac = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jac, None, point, forearm)
        force = np.array([0., 0., -100.])  # one hand pressing down on the bar
        torques = jac.T @ force
        elbow = int(model.joint(f'rider_elbow_{side}').dofadr[0])
        shoulder = int(model.joint(f'rider_shoulder_{side}').dofadr[0])
        elbow_anchor = data.xpos[int(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f'rider_forearm_{side}'))]
        r_elbow = point - elbow_anchor
        expected_elbow = np.cross(r_elbow, force)[1]
        assert torques[elbow] == pytest.approx(expected_elbow, rel=1e-9)
        # The shoulder sees the same force through the whole-arm lever.
        shoulder_anchor = data.xpos[int(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f'rider_upper_arm_{side}'))]
        r_shoulder = point - shoulder_anchor
        assert torques[shoulder] == pytest.approx(
            np.cross(r_shoulder, force)[1], rel=1e-9)
        for joint in ('shoulder', 'elbow'):
            dof = int(model.joint(f'rider_{joint}_{other}').dofadr[0])
            assert torques[dof] == pytest.approx(0., abs=1e-12)


@pytest.mark.slow
def test_welded_hands_report_finite_per_hand_reactions(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(40):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    assert physical.attachment_errors == ()
    samples = physical.attachment_samples
    assert {'grip_left', 'grip_right'} <= set(samples)
    for side in ('left', 'right'):
        sample = samples[f'grip_{side}']
        assert sample.kind == 'grip'
        assert math.isfinite(sample.normal_n) and math.isfinite(sample.tangent_n)
        assert math.isfinite(sample.gap_m) and sample.gap_m < .005
    channel = physical.sample.channels['rider_welds']
    assert {'grip_left', 'grip_right'} <= set(channel)
    left = np.asarray(channel['grip_left']['force_on_rider_n'])
    right = np.asarray(channel['grip_right']['force_on_rider_n'])
    assert np.isfinite(left).all() and np.isfinite(right).all()
    # Symmetric pose, symmetric road: the two hands split the bar load evenly.
    # In-plane components agree; the out-of-plane residual is antisymmetric
    # (each hand braces against its own half-width direction) and negligible.
    np.testing.assert_allclose(left[[0, 2]], right[[0, 2]], rtol=.01)
    np.testing.assert_allclose(left[1], -right[1], atol=1e-6)


@pytest.mark.slow
def test_initial_state_round_trip_restores_per_hand_anchors(tmp_path):
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    env = _environment(tmp_path)
    state = PhysicalInitialState.capture(env.sim)
    anchors = state.payload['grip_anchor_local']
    assert set(anchors) == {'left', 'right'}
    env.sim.physical_initial_state = state
    env.sim.physical.reset()
    restored = env.sim.physical.rider_contacts.grip_anchor_local
    assert set(restored) == {'left', 'right'}
    for side in ('left', 'right'):
        np.testing.assert_allclose(restored[side], anchors[side], atol=1e-12)
