from dataclasses import replace
from math import cos, sin

import pytest

from bike_sim.physics.seated_climb import (
    SeatedClimbConfig, SeatedClimbPolicy, SeatedClimbSignals, crank_effort_ceiling,
)
from bike_sim.physics.resolution import resolve_physics_config


def signals(angle=0., rate=8., acceleration_scale=1., gyro=0.):
    return SeatedClimbSignals(gyro,
        (9.81 * acceleration_scale * sin(angle), 0., 9.81 * acceleration_scale * cos(angle)),
        rate, 20.)


def test_sustained_uphill_estimate_changes_only_seated_torso_with_bounded_speed():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True))
    previous = 0.
    for interval_index in range(400):
        intent = policy.update(signals(.2), .01)
        assert intent.posture.use_saddle
        assert intent.posture.pelvis_offset_m is None
        assert intent.posture.pelvis_pitch_rad == 0.
        assert abs(intent.posture.torso_lean_rad - previous) <= .005 + 1e-12
        assert 0. <= intent.effort_ceiling_nm <= 60.
        previous = intent.posture.torso_lean_rad
    assert previous == pytest.approx(.2, abs=.001)


def test_reaction_delay_does_not_expose_future_input():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True))
    for interval_index in range(15):
        intent = policy.update(signals(.3), .01)
        assert intent.effort_ceiling_nm == 0.
        assert intent.posture.torso_lean_rad == 0.
    assert policy.update(signals(.3), .01).effort_ceiling_nm > 0.


def test_impact_does_not_turn_accelerometer_spike_into_lean():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, reaction_delay_s=0.))
    for interval_index in range(100):
        intent = policy.update(signals(.7, acceleration_scale=2.), .01)
    assert intent.posture.torso_lean_rad == 0.


def test_torso_angles_remain_within_seated_bounds():
    for angle, expected in ((.8, .35), (-.8, -.1)):
        policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, reaction_delay_s=0.))
        for interval_index in range(400):
            intent = policy.update(signals(angle), .01)
        assert intent.posture.torso_lean_rad == pytest.approx(expected)


def test_gyro_preserves_orientation_when_acceleration_correction_is_unavailable():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, reaction_delay_s=0.))
    for interval_index in range(10):
        intent = policy.update(signals(0., acceleration_scale=0., gyro=1.), .01)
    assert intent.posture.torso_lean_rad > 0.


def test_reset_reproduces_delay_and_bounded_effort():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True))
    first = [policy.update(signals(.3), .01) for interval_index in range(50)]
    policy.reset()
    assert first == [policy.update(signals(.3), .01) for interval_index in range(50)]


@pytest.mark.parametrize('rate, expected', [(0., 60.), (-8., 60.), (8., 28.125), (20., 11.25)])
def test_shaft_power_request_is_finite_at_rest_and_reverse_rotation(rate, expected):
    assert crank_effort_ceiling(225., 60., rate) == pytest.approx(expected)


def test_config_resolver_round_trip_preserves_seated_intention():
    config = resolve_physics_config({'physics_mode': 'physical', 'drive_mode': 'articulated_effort',
        'seated_climb': {'enabled': True, 'target_crank_power_w': 240.}})
    assert config.seated_climb.enabled
    policy = SeatedClimbPolicy(replace(config.seated_climb, reaction_delay_s=0.))
    for interval_index in range(20):
        intent = policy.update(signals(rate=10.), .01)
    assert intent.effort_ceiling_nm == pytest.approx(24.)


@pytest.mark.parametrize('name, value', [
    ('period_s', 0.), ('reaction_delay_s', -.1), ('target_crank_power_w', float('nan')),
    ('max_crank_torque_nm', -1.), ('enabled', 1),
])
def test_invalid_human_parameters_are_rejected(name, value):
    with pytest.raises(ValueError):
        SeatedClimbConfig(**{name: value})


def test_disabled_intent_is_neutral():
    policy = SeatedClimbPolicy(SeatedClimbConfig())
    intent = policy.update(signals(.3), .01)
    assert intent.effort_ceiling_nm == 0.
    assert intent.posture.torso_lean_rad == 0.


def test_engine_inertial_site_reports_uphill_with_the_expected_lean_sign():
    from types import SimpleNamespace
    import mujoco
    from bike_sim.sim.ride.physical_observations import sensor_channels
    from bike_sim.sim.ride.rider_intent import signals_from_channels

    model = mujoco.MjModel.from_xml_string('''<mujoco><compiler angle="radian"/>
      <option gravity="0 0 -9.81"/>
      <worldbody><body name="frame" euler="0 -0.2 0">
        <joint name="frame_pitch" axis="0 1 0"/><geom size=".1" mass="10"/>
        <site name="imu"/>
        <body><joint name="crank_spin" axis="0 1 0"/><geom size=".1" mass="1"/></body>
        <body pos="1 0 0"><joint name="front_wheel_spin" axis="0 1 0"/><geom size=".1" mass="1"/></body>
        <body pos="-1 0 0"><joint name="rear_wheel_spin" axis="0 1 0"/><geom size=".1" mass="1"/></body>
      </body></worldbody>
      <sensor><accelerometer name="sensor_frame_accel" site="imu"/>
        <gyro name="sensor_frame_gyro" site="imu"/></sensor>
    </mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = SimpleNamespace(sim=SimpleNamespace(data=data), drive=SimpleNamespace(last={}),
        address=lambda name: (int(model.joint(name).qposadr[0]), int(model.joint(name).dofadr[0])))
    sensed = signals_from_channels(sensor_channels(runtime))
    assert sensed.specific_force_body_mps2[0] > 0.
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, reaction_delay_s=0.))
    for interval_index in range(400):
        intent = policy.update(sensed, .01)
    assert intent.posture.torso_lean_rad == pytest.approx(.2, abs=.001)
