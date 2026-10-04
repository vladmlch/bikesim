"""R3: directional joint strength curves, provenance gate, and the 450 W cap.

Each internal rider joint owns two directional torque envelopes: isometric
capacity over the anatomical angle, Hill-scaled by velocity in the torque's
own direction. Positive whole-body active power is capped at the configured
engineering budget; passive damping and eccentric work never add to it.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from bike_sim.mujoco.reference_rider import reference_joint_names
from bike_sim.physics.joint_strength import (
    TorqueCurve, directional_capacity, load_strength_coordinates,
    load_strength_profile,
)
from bike_sim.physics.rider_activation import limit_positive_power
from bike_sim.physics.rider_envelope import load_joint_envelopes

ROOT = Path(__file__).resolve().parents[2]
STRENGTH = ROOT / 'examples' / 'research' / 'rider_strength_reference.json'
ENVELOPE = ROOT / 'examples' / 'research' / 'rider_joint_envelope_anatomical.json'
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'
JOINTS = reference_joint_names()


def _curve(**overrides):
    base = dict(angles_rad=(-1., 0., 1.), torques_nm=(50., 100., 50.),
                vmax_rad_s=10., hill_c=.25, eccentric_ratio=1.4,
                source='synthetic-unit-fixture-not-human-data')
    return TorqueCurve(**{**base, **overrides})


def test_curve_validation():
    with pytest.raises(ValueError):
        _curve(angles_rad=(0.,), torques_nm=(1.,))
    with pytest.raises(ValueError):
        _curve(angles_rad=(0., 1., 2.), torques_nm=(1., 2.))
    with pytest.raises(ValueError):
        _curve(torques_nm=(50., -1., 50.))
    with pytest.raises(ValueError):
        _curve(angles_rad=(0., 0., 1.))
    with pytest.raises(ValueError):
        _curve(vmax_rad_s=0.)
    with pytest.raises(ValueError):
        _curve(hill_c=-.1)
    with pytest.raises(ValueError):
        _curve(eccentric_ratio=.9)
    with pytest.raises(ValueError):
        _curve(source='   ')
    with pytest.raises(ValueError):
        _curve(angles_rad=(0., float('nan')))


def test_plan_fixture_ordering_and_velocity_limits():
    curve = _curve()
    iso = directional_capacity(curve, 0., 0., 1)
    concentric = directional_capacity(curve, 0., 5., 1)
    eccentric = directional_capacity(curve, 0., -5., 1)
    assert 0 <= concentric < iso <= eccentric <= 140.
    assert directional_capacity(curve, 0., 10., 1) == 0.
    assert directional_capacity(curve, 0., -10., -1) == 0.
    assert 100. * -5. < 0


def test_isometric_interpolation_between_knots():
    curve = _curve()
    assert directional_capacity(curve, 0., 0., 1) == pytest.approx(100.)
    assert directional_capacity(curve, .5, 0., 1) == pytest.approx(75.)
    assert directional_capacity(curve, -.5, 0., 1) == pytest.approx(75.)
    # Static requests at zero speed are never throttled by a division.
    assert np.isfinite(directional_capacity(curve, 0., 0., -1))


def test_concentric_decays_eccentric_is_bounded_and_finite():
    curve = _curve()
    speeds = [0., 2., 5., 8.]
    capacities = [directional_capacity(curve, 0., v, 1) for v in speeds]
    assert all(b < a for a, b in zip(capacities, capacities[1:]))
    # Slightly above vmax nothing remains; exactly at vmax is already zero.
    assert directional_capacity(curve, 0., 10.1, 1) == 0.
    # The eccentric branch saturates at eccentric_ratio * isometric, never more.
    deep = directional_capacity(curve, 0., -50., 1)
    assert deep <= 100. * 1.4 + 1e-9
    assert np.isfinite(deep)
    # Eccentric torque against motion only absorbs: its power is negative.
    assert directional_capacity(curve, 0., -5., 1) * -5. < 0.


def test_direction_sign_uses_the_torque_direction():
    curve = _curve()
    # A negative-q torque sees its concentric side at negative velocity.
    assert directional_capacity(curve, 0., -5., -1) < \
        directional_capacity(curve, 0., 0., -1)
    assert directional_capacity(curve, 0., 5., -1) > \
        directional_capacity(curve, 0., -5., -1)
    with pytest.raises(ValueError):
        directional_capacity(curve, 0., 0., 0)
    with pytest.raises(ValueError):
        directional_capacity(curve, 2., 0., 1)
    with pytest.raises(ValueError):
        directional_capacity(curve, 0., float('inf'), 1)


def test_positive_power_budget_scales_active_torque_only():
    # 100 N*m at 5 rad/s on five 'joints' requests 2500 W of positive work;
    # the 450 W budget must scale the accelerating set to exactly 450 W.
    torque = np.full(5, 100.)
    speed = np.full(5, 5.)
    limited = limit_positive_power(torque, speed, 450.)
    assert np.maximum(limited * speed, 0.).sum() == pytest.approx(450.)
    # An eccentric joint (tau*v < 0) does not consume the positive budget and
    # is not scaled down -- braking capacity stays intact.
    torque = np.array([100., -100.])
    speed = np.array([5., 5.])
    limited = limit_positive_power(torque, speed, 300.)
    assert np.maximum(limited * speed, 0.).sum() == pytest.approx(300.)
    assert limited[1] == -100.
    # Zero velocity divides nothing: the request passes through untouched.
    assert np.array_equal(limit_positive_power(torque, np.zeros(2), 450.), torque)


def test_reference_profile_covers_exactly_the_registry():
    profile = load_strength_profile(str(STRENGTH), JOINTS, require_verified=False)
    assert set(profile) == set(JOINTS)
    for name in JOINTS:
        assert set(profile[name]) == {1, -1}
        for curve in profile[name].values():
            assert curve.vmax_rad_s > 0. and curve.eccentric_ratio >= 1.
            assert curve.source.strip()


def test_verified_gate_rejects_the_provisional_profile():
    # Every shipped curve is verified=false; a gate demanding provenance must
    # fail loudly rather than accept placeholder data by silence.
    with pytest.raises(ValueError, match='not verified'):
        load_strength_profile(str(STRENGTH), JOINTS, require_verified=True)
    # Marking one curve verified still leaves ten others unverified.
    payload = json.loads(STRENGTH.read_text())
    payload['joints']['rider_torso_hinge']['directions']['positive']['verified'] = True
    with pytest.raises(ValueError, match='not verified'):
        _load(payload, JOINTS, require_verified=True)


def _load(payload, joints, require_verified, tmp_path=None):
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
        json.dump(payload, handle)
        path = handle.name
    return load_strength_profile(path, joints, require_verified=require_verified)


def test_profile_rejects_incomplete_or_alien_coverage(tmp_path):
    payload = json.loads(STRENGTH.read_text())
    del payload['joints']['rider_ankle_rear']
    with pytest.raises(ValueError, match='internal rider joints'):
        _load(payload, JOINTS, require_verified=False)
    payload = json.loads(STRENGTH.read_text())
    payload['joints']['rider_knee_front']['directions'].pop('negative')
    with pytest.raises(ValueError, match='two torque directions'):
        _load(payload, JOINTS, require_verified=False)
    payload = json.loads(STRENGTH.read_text())
    payload['topology'] = 'articulated_planar_single_arm'
    with pytest.raises(ValueError, match='different rider topology'):
        _load(payload, JOINTS, require_verified=False)


def test_curve_knots_must_cover_the_used_anatomical_range():
    payload = json.loads(STRENGTH.read_text())
    entry = payload['joints']['rider_knee_front']['directions']['negative']
    entry['angles_rad'] = [0.5, 1.0, 2.0]
    with pytest.raises(ValueError, match='used ROM'):
        _load(payload, JOINTS, require_verified=False)


def test_strength_coordinates_agree_with_the_joint_envelope():
    coordinates = load_strength_coordinates(str(STRENGTH), JOINTS)
    envelopes = load_joint_envelopes(str(ENVELOPE))
    assert set(coordinates) == set(JOINTS) == set(envelopes)
    for name in JOINTS:
        coordinate, envelope = coordinates[name], envelopes[name]
        assert coordinate.direction == envelope.direction
        assert coordinate.neutral_anatomical_rad == pytest.approx(
            envelope.neutral_anatomical_rad)


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
def test_actuators_are_pure_motors_with_dof_damping(tmp_path):
    import mujoco
    env = _environment(tmp_path)
    model = env.sim.model
    controller = env.sim.physical.rider_control
    assert controller.strength is not None
    present={name for name in JOINTS if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)>=0}
    assert set(controller.strength) == present
    for name, (_, dof, aid) in controller.joints.items():
        assert int(model.actuator_trntype[aid]) == int(mujoco.mjtTrn.mjTRN_JOINT)
        # No affine bias: actuator_force is purely the commanded muscle torque.
        assert np.allclose(model.actuator_biasprm[aid], 0.)
        assert model.dof_damping[dof] == pytest.approx(
            controller.config.passive_damping_nms_rad)


@pytest.mark.slow
def test_stepped_effort_respects_strength_and_the_power_budget(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(200):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=30.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    diagnostics = physical.rider_control.effort_diagnostics
    assert diagnostics['rider_positive_power_w'] <= 450. + 1e-6
    assert diagnostics['rider_strength_violations'] == ()
    # Passive work is reported separately and never feeds the positive budget:
    # tissue damping can only remove power, so its signed power is <= 0.
    assert diagnostics['rider_passive_power_w'] <= 1e-9
    controller = physical.rider_control
    kd = controller.config.passive_damping_nms_rad
    sample = physical.sample
    for name, (_, dof, aid) in controller.joints.items():
        # The actuator is a pure motor: solved force == commanded muscle
        # torque == the energy-ledger actuator row.
        delivered = controller.last_terms[name]['solved_active_nm']
        assert delivered == pytest.approx(
            float(sample.forces[f'act_{name}'][dof]))
        # Tissue damping is a DOF-damping row: recorded at the interval's
        # incoming velocity, moved wholly into rider_passive_damping, and
        # removed from engine_passive so it is never counted twice.
        passive = controller.last_terms[name]['solved_passive_nm']
        assert passive == pytest.approx(
            float(sample.forces['rider_passive_damping'][dof]))
        assert passive == pytest.approx(-kd * float(sample.qvel[dof]))
        assert float(sample.forces['engine_passive'][dof]) == pytest.approx(0., abs=1e-9)


@pytest.mark.slow
def test_directional_capacity_bounds_the_commanded_torque(tmp_path):
    # Directly exercise the limiter: a huge active request is clipped to the
    # directional capacity at the current anatomical state, joint by joint.
    env = _environment(tmp_path)
    controller = env.sim.physical.rider_control
    data = env.sim.data
    names = tuple(controller.joints)
    request = np.full(len(names), 1e4)
    clipped, limited = controller.strength_limited(request, data.qpos, data.qvel)
    assert set(limited) == set(names)
    for index, (name, (qa, dof, _)) in enumerate(controller.joints.items()):
        capacity = controller.strength_capacity(name,
            controller.anatomical_joint_angle(name, data.qpos[qa]),
            data.qvel[dof], 1e4)
        assert clipped[index] == pytest.approx(capacity)
        assert capacity < 1e4


def test_leg_curves_span_a_120_rpm_pedalling_envelope():
    """At 120 rpm the knee peaks near 7.7 rad/s and must still hold ~40 N.m; the hip ~60 N.m at 5 rad/s."""
    # {joint: {+1: curve, -1: curve}} keyed by the sign of the bounded q-torque;
    # JSON "positive" -> +1, "negative" -> -1.
    profile = load_strength_profile(str(STRENGTH), JOINTS, require_verified=False)
    for side in ('front', 'rear'):
        for joint in ('hip', 'knee', 'ankle'):
            for direction in (1, -1):
                curve = profile[f'rider_{joint}_{side}'][direction]
                assert curve.vmax_rad_s >= 20., (joint, side, direction)
                assert curve.hill_c >= .3, (joint, side, direction)
                assert 'unverified' in curve.source
        knee = profile[f'rider_knee_{side}'][-1]          # knee extension ("negative")
        assert directional_capacity(knee, 1.0, -7.7, -1) >= 40.
        hip = profile[f'rider_hip_{side}'][1]             # hip extension ("positive")
        assert directional_capacity(hip, .9, 5., 1) >= 60.
    torso = profile['rider_torso_hinge'][1]
    assert torso.vmax_rad_s == 8.                          # untouched
