import copy
from dataclasses import replace

import numpy as np
import pytest

from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.replay import integration_state, replay_episode
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.validation.rider_replay import build_sim


@pytest.fixture(scope='module')
def seated_sim():
    return build_sim('examples/research/viewer_physics_fast.toml',
        'examples/research/rider_resume_flat.toml', .000625,
        physics_overrides={'seated_climb': {'enabled': True}})


def environment(sim):
    sim.reset()
    return ResearchEnvironment(sim, ExperimentConfig(duration_s=.3, actuator_delay_s=0.,
        record_decimation=16), replace(SensorConfig.ideal(), sample_period_s=.005))


def test_shared_runtime_delays_effort_and_remains_seated(seated_sim):
    env = environment(seated_sim)
    saw_effort = False
    while not env.done:
        env.step(RideControl(motor_torque_nm=0.))
        sample = env.sim.physical.sample
        control = sample.channels['control']
        assert control['posture']['use_saddle']
        assert control['posture']['pelvis_offset_m'] is None
        if sample.time_s < .15 - 1e-9:
            assert control['human_torque_nm'] == 0.
        saw_effort = saw_effort or control['human_torque_nm'] > 0.
    assert env.sim.time_s == pytest.approx(.3)
    assert saw_effort
    assert env.sim.physical.model_status.as_dict()['model_valid']


def test_force_probes_do_not_consume_rider_reaction_state(seated_sim):
    env = environment(seated_sim)
    before = copy.deepcopy(vars(env.sim.physical.rider_intent.policy))
    position = env.sim.data.qpos.copy()
    velocity = env.sim.data.qvel.copy()
    for interval_index in range(3):
        env.sim.physical.apply_forces(active=True, advance=False, control=RideControl())
    assert vars(env.sim.physical.rider_intent.policy) == before
    np.testing.assert_array_equal(env.sim.data.qpos, position)
    np.testing.assert_array_equal(env.sim.data.qvel, velocity)


def test_seated_intention_replays_after_the_reaction_delay(seated_sim, tmp_path):
    env = environment(seated_sim)
    while not env.done:
        env.step(RideControl(motor_torque_nm=0.))
    final = integration_state(env.sim)
    destination = tmp_path / 'episode'
    env.save(destination)
    report = replay_episode(destination)
    assert report['passed']
    assert report['replayed_control_steps'] == 30
    assert report['final_state_max_abs_error'] < 1e-9
    assert np.isfinite(final).all()


def test_motor_transport_does_not_postpone_automatic_rider_reaction(seated_sim):
    seated_sim.reset()
    env = ResearchEnvironment(seated_sim, ExperimentConfig(duration_s=.35,
        actuator_delay_s=.3, record_decimation=1),
        replace(SensorConfig.ideal(), sample_period_s=.005))
    while not env.done:
        env.step(RideControl(motor_torque_nm=40.))
    before_motor = [sample for sample in env.recorder.samples
                    if 320 <= sample.interval_id < 480]
    assert before_motor
    assert any(sample.channels['control']['human_torque_nm'] > 0. for sample in before_motor)
    assert all(sample.channels['control']['motor_torque_nm'] == 0. for sample in before_motor)


def test_initial_rider_and_policy_sensors_use_the_same_solved_input(seated_sim):
    env = environment(seated_sim)
    signals = seated_sim.physical.rider_intent_signals
    np.testing.assert_allclose(signals.specific_force_body_mps2,
        env.observation.specific_force_body_mps2, rtol=0., atol=1e-12)
