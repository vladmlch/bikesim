import pytest
from bike_sim.cli.research import parser, make_environment
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.rider_behavior import RiderSignals
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset

CONFIG = ExperimentConfig(duration_s=.03, actuator_delay_s=0., record_decimation=1)


class Recorder:
    """Stub behavior: records every call; proves the hook, not a reflex model."""
    def __init__(self, posture=None):
        self.posture, self.calls, self.resets = posture, [], []

    def reset(self, seed):
        self.resets.append(seed)

    def act(self, time_s, signals):
        self.calls.append((time_s, signals))
        return self.posture


@pytest.fixture(scope='module')
def lumped():
    return RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))


@pytest.fixture(scope='module')
def articulated():
    args = parser().parse_args(['--scenario', 'flat', '--duration', '.03', '--ideal-sensors'])
    return make_environment(args).sim  # articulated plant build is slow: share it


def _fresh(sim):
    sim.reset()
    return sim


def test_act_is_called_once_per_control_step_with_sensory_signals(lumped):
    behavior = Recorder()
    env = ResearchEnvironment(_fresh(lumped), CONFIG, SensorConfig.ideal(), rider_behavior=behavior)
    assert behavior.resets == [env.seed]
    steps = 0
    while not env.done:
        env.step(RideControl(motor_torque_nm=40.)); steps += 1
    assert len(behavior.calls) == steps == 3
    assert [t for t, _ in behavior.calls] == pytest.approx([0., .01, .02])
    first, later = behavior.calls[0][1], behavior.calls[-1][1]
    assert isinstance(later, RiderSignals)
    assert first == RiderSignals(0., (0., 0., 0.), 0., 0., 0.)   # nothing sensed before the first interval
    assert later.specific_force_body_mps2 != (0., 0., 0.)
    assert not hasattr(later, 'front_load_n') and not hasattr(later, 'road_pitch_rad')  # no terrain truth


def test_reset_reseeds_behavior(lumped):
    behavior = Recorder()
    env = ResearchEnvironment(_fresh(lumped), CONFIG, SensorConfig.ideal(), rider_behavior=behavior)
    env.reset(seed=7)
    assert behavior.resets == [0, 7]


def test_policy_posture_takes_precedence_over_behavior(articulated):
    behavior = Recorder(RiderPosture(torso_lean_rad=.2))
    env = ResearchEnvironment(_fresh(articulated), CONFIG, SensorConfig.ideal(), rider_behavior=behavior)
    env.step(RideControl(motor_torque_nm=0., posture=RiderPosture(torso_lean_rad=.05)))
    assert behavior.calls == []
    assert env.commands_applied[-1]['control']['posture']['torso_lean_rad'] == .05


def test_behavior_posture_is_applied_to_the_command(articulated):
    behavior = Recorder(RiderPosture(torso_lean_rad=.1))
    env = ResearchEnvironment(_fresh(articulated), CONFIG, SensorConfig.ideal(), rider_behavior=behavior)
    env.step(RideControl(motor_torque_nm=0.))
    assert env.commands_requested[-1]['control']['posture']['torso_lean_rad'] == .1
    assert env.commands_applied[-1]['control']['posture']['torso_lean_rad'] == .1


def test_behavior_and_posture_owning_program_are_exclusive(lumped):
    program = RiderProgram((RiderKeyframe(0.),))
    with pytest.raises(ValueError, match='rider_behavior'):
        ResearchEnvironment(_fresh(lumped), CONFIG, SensorConfig.ideal(),
                            rider_behavior=Recorder(), rider_program=program)


def test_behavior_must_implement_protocol(lumped):
    with pytest.raises(ValueError, match='RiderBehavior'):
        ResearchEnvironment(_fresh(lumped), CONFIG, SensorConfig.ideal(), rider_behavior=object())


def test_rider_program_is_applied_and_saved(articulated, tmp_path):
    import json
    program = RiderProgram((RiderKeyframe(0., RiderPosture(torso_lean_rad=.12), human_torque_nm=15.),))
    env = ResearchEnvironment(_fresh(articulated), CONFIG, SensorConfig.ideal(), rider_program=program)
    env.step(RideControl(motor_torque_nm=0.))
    applied = env.commands_applied[-1]['control']
    assert applied['posture']['torso_lean_rad'] == .12 and applied['human_torque_nm'] == 15.
    with pytest.raises(ValueError, match='owns'):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
    env.save(tmp_path/'run')
    summary = json.loads((tmp_path/'run'/'summary.json').read_text())
    assert RiderProgram.from_dict(summary['research']['rider_program']) == program


def test_rider_program_evolves_at_physics_rate_without_motor_transport_delay(articulated):
    program = RiderProgram((RiderKeyframe(0., human_torque_nm=10.),
        RiderKeyframe(.01, RiderPosture(torso_lean_rad=.1), human_torque_nm=20.)))
    env = ResearchEnvironment(_fresh(articulated),
        ExperimentConfig(duration_s=.02, actuator_delay_s=.02, record_decimation=1),
        SensorConfig.ideal(), rider_program=program)
    env.step(RideControl(motor_torque_nm=40.))
    middle = min(env.recorder.samples, key=lambda sample: abs(sample.time_s-.005))
    assert middle.channels['control']['human_torque_nm'] == pytest.approx(15.)
    assert middle.channels['control']['posture']['torso_lean_rad'] == pytest.approx(.05)
    assert middle.channels['control']['motor_torque_nm'] == 0.
