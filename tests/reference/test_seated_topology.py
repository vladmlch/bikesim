"""Reference seated topology: a saddle pin frees pelvis pitch, platforms spin."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.reference_rider import add_saddle_pin
from bike_sim.physics.rider_envelope import (
    JointEnvelope, anatomical_angle, joint_q_range, load_joint_envelopes,
    reference_rom_check, RIDER_JOINTS,
)

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'
ENVELOPE = ROOT / 'examples' / 'research' / 'rider_joint_envelope_anatomical.json'


def test_saddle_connection_does_not_constrain_orientation():
    root = ET.fromstring('<mujoco><worldbody><body name="rider_pelvis">'
        '<site name="site_rider_saddle" pos="0 0 -0.060"/>'
        '</body><body name="frame"/></worldbody><equality/></mujoco>')
    add_saddle_pin(root, 0.0025)
    eq = root.find("./equality/connect[@name='connect_saddle']")
    assert eq is not None
    assert eq.get('body1') == 'rider_pelvis'
    assert eq.get('body2') == 'frame'
    assert eq.get('anchor') == '0 0 -0.060'
    assert root.find("./equality/weld[@name='weld_saddle']") is None


def test_saddle_pin_replaces_a_legacy_weld_and_validates():
    root = ET.fromstring('<mujoco><worldbody><body name="rider_pelvis">'
        '<site name="site_rider_saddle" pos="0 0 -0.060"/>'
        '</body><body name="frame"/></worldbody><equality>'
        '<weld name="weld_saddle" body1="rider_pelvis" body2="frame"/></equality></mujoco>')
    add_saddle_pin(root, 0.0025)
    assert root.find("./equality/weld[@name='weld_saddle']") is None
    assert root.find("./equality/connect[@name='connect_saddle']") is not None
    bare = ET.fromstring('<mujoco><worldbody/></mujoco>')
    with pytest.raises(ValueError):
        add_saddle_pin(bare, 0.0025)
    with pytest.raises(ValueError):
        add_saddle_pin(root, 0.)


def test_anatomical_angle_inverts_the_declared_range():
    envelope = JointEnvelope(neutral_anatomical_rad=.3, direction=-1,
        minimum_anatomical_rad=-.2, maximum_anatomical_rad=1.1,
        provenance='synthetic-unit-fixture')
    lo, hi = joint_q_range(envelope)
    # The q-bound endpoints map to the two declared anatomical extremes,
    # swapped under a negative direction convention.
    assert {round(anatomical_angle(envelope, q), 12) for q in (lo, hi)} == {
        round(-.2, 12), round(1.1, 12)}
    assert anatomical_angle(envelope, 0.) == pytest.approx(.3)
    forward = JointEnvelope(neutral_anatomical_rad=.3, direction=1,
        minimum_anatomical_rad=-.2, maximum_anatomical_rad=1.1,
        provenance='synthetic-unit-fixture')
    flo, fhi = joint_q_range(forward)
    assert anatomical_angle(forward, flo) == pytest.approx(-.2)
    assert anatomical_angle(forward, fhi) == pytest.approx(1.1)
    # A wrapped coordinate cannot fake being inside the range.
    assert anatomical_angle(envelope, 2*math.pi) == pytest.approx(.3)


def test_rom_check_names_out_of_range_joints():
    envelopes = {name: JointEnvelope(neutral_anatomical_rad=0., direction=1,
        minimum_anatomical_rad=-1., maximum_anatomical_rad=1.,
        provenance='synthetic-unit-fixture') for name in RIDER_JOINTS}
    inside = {name: 0. for name in RIDER_JOINTS}
    assert reference_rom_check(inside, envelopes) == ()
    outside = dict(inside, rider_torso_hinge=2.)
    assert reference_rom_check(outside, envelopes) == ('rider_torso_hinge',)


def _pin_config(tmp_path):
    target = tmp_path / 'pinned.toml'
    text = WELDED.read_text().replace(
        'saddle_attachment = "weld"', 'saddle_attachment = "pin"')
    # Config-relative paths resolve against the TOML's own directory.
    text = text.replace('joint_envelope_path = "rider_joint_envelope_anatomical.json"',
                        f'joint_envelope_path = "{ENVELOPE}"')
    target.write_text(text)
    return target


def _environment(tmp_path, config):
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
def test_pin_rows_free_pelvis_pitch_and_carry_no_moment(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.weld_pedals import equality_rows

    env = _environment(tmp_path, _pin_config(tmp_path))
    model, data = env.sim.model, env.sim.data
    eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'connect_saddle'))
    assert eq >= 0
    assert int(model.eq_type[eq]) == int(mujoco.mjtEq.mjEQ_CONNECT)
    assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'weld_saddle')) < 0
    pitch = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'rider_root_pitch'))
    assert pitch >= 0
    # No actuator may write to any rider root degree of freedom.
    root_dofs = {int(model.joint(name).dofadr[0]) for name in
                 ('rider_root_x', 'rider_root_z', 'rider_root_pitch')}
    for a in range(model.nu):
        assert int(model.actuator_trnid[a, 0]) not in {
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n))
            for n in ('rider_root_x', 'rider_root_z', 'rider_root_pitch')}

    env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
             front_brake_demand=1., rear_brake_demand=1.)
    rows = equality_rows(data)[eq]
    assert len(rows) == 3
    sample = env.sim.physical.rider_contacts.last_attachment_samples['saddle']
    assert sample.moment_nm == 0.
    assert sample.gap_m <= .005
    assert np.isfinite([sample.normal_n, sample.tangent_n]).all()


@pytest.mark.slow
def test_pedal_platform_keeps_its_spin_joint(tmp_path):
    env = _environment(tmp_path, _pin_config(tmp_path))
    model = env.sim.model
    for side in ('front', 'rear'):
        joint = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                      f'pedal_{side}_spin'))
        assert joint >= 0
        assert int(model.jnt_type[joint]) == int(mujoco.mjtJoint.mjJNT_HINGE)
        assert not bool(model.jnt_limited[joint])
        weld = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                     f'weld_foot_{side}'))
        assert weld >= 0
        # The foot is welded to the free-spinning platform, never to the
        # crank arm or the frame.
        body2 = int(model.eq_obj2id[weld])
        assert body2 == int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                            f'pedal_{side}'))


@pytest.mark.slow
def test_built_pose_anatomical_angles_inside_envelope(tmp_path):
    env = _environment(tmp_path, _pin_config(tmp_path))
    model, data = env.sim.model, env.sim.data
    envelopes = load_joint_envelopes(str(ENVELOPE))
    angles = {}
    for name in RIDER_JOINTS:
        joint = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        assert joint >= 0
        angles[name] = float(data.qpos[model.jnt_qposadr[joint]])
    assert reference_rom_check(angles, envelopes) == ()
    # Every joint's assembled-pose angle is a real anatomical read, not a
    # round-trip of its own declared range.
    for name, envelope in envelopes.items():
        angle = anatomical_angle(envelope, angles[name])
        assert envelope.minimum_anatomical_rad <= angle <= envelope.maximum_anatomical_rad
