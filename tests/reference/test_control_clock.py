import os
from pathlib import Path

import numpy as np
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.control_clock import ControlClock


# bike::research::make_environment request mirroring the make_environment
# call below: flat 200 m track, dt=.0005, welded pinned plant, diagnostic
# policy. The settled t=0 state transfers through initial_state.json.
_ENV_REQUEST = '''\
request_schema = 1
physics_file = "physics.toml"
track_file = "track.toml"
model_mode = "ride"
rider_variant = "articulated_planar"
speed_kph = 0.0
seed_decimal = "0"
use_equilibrium_cache = false
duration_s = 0.2
control_period_s = 0.01
actuator_delay_s = 0.005
sensor_delay_s = 0.01
check_policy = "diagnostic"
capture_mode = "legacy"
initial_state_file = "initial_state.json"

[overrides]
timestep_s = 0.0005

[experiment]
record_decimation = 80
'''


def _fixture_save_env_request(env, physics_path, track_path):
    """tests/fixtures/golden/<platform>/control_clock/env_request/ — the
    bundle the C++ port feeds bike::research::make_environment: pinned
    physics TOML (asset paths rewritten relative to the bundle), the flat
    track, request.toml and the settled t=0 state."""
    root = os.environ.get('BIKE_GOLDEN_CAPTURE')
    if root is None:
        return
    import re
    import _golden
    target = Path(root) / _golden.platform_key() / 'control_clock' / 'env_request'
    target.mkdir(parents=True, exist_ok=True)
    physics_text = Path(physics_path).read_text()
    for key in ('joint_envelope_path', 'joint_strength_path'):
        physics_text = re.sub(
            rf'{key} = "([^"]+)"',
            lambda m: (f'{key} = "'
                       f'{os.path.relpath((Path(physics_path).parent / m.group(1)).resolve(), target)}"'),
            physics_text)
    (target / 'physics.toml').write_text(physics_text)
    (target / 'track.toml').write_text(Path(track_path).read_text())
    (target / 'request.toml').write_text(_ENV_REQUEST)
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    (target / 'initial_state.json').write_text(
        PhysicalInitialState.capture(env.sim).payload_json)


def test_period_must_be_an_integer_multiple_of_the_timestep():
    clock = ControlClock(.0005, .005)
    assert clock.steps_per_period == 10
    assert [clock.is_tick(s) for s in range(12)] == [True] + [False]*9 + [True, False]
    with pytest.raises(ValueError, match='multiple'):
        ControlClock(.0005, .0012)
    with pytest.raises(ValueError, match='multiple'):
        SimulationPhysicsConfig(physics_mode='physical', timestep_s=.0005,
                                control_period_s=.0012)


def test_held_command_is_returned_unchanged_between_ticks():
    clock = ControlClock(.0005, .005)
    with pytest.raises(RuntimeError, match='no command'):
        clock.held()
    clock.hold({'rider_knee_front': 12.5, 'rider_hip_front': -3.})
    held = clock.held()
    held['rider_knee_front'] = 0.
    assert clock.held() == {'rider_knee_front': 12.5, 'rider_hip_front': -3.}
    clock.reset()
    with pytest.raises(RuntimeError, match='no command'):
        clock.held()


@pytest.mark.parametrize('dt,period', [(0., .005), (.0005, np.inf), (.0005, -.005)])
def test_clock_rejects_invalid_durations(dt, period):
    with pytest.raises(ValueError):
        ControlClock(dt, period)


@pytest.mark.slow
def test_runtime_holds_ctrl_but_observes_solved_effort_each_step(tmp_path, monkeypatch):
    from pathlib import Path
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    from bike_sim.sim.ride.control import RideControl

    original = ArticulatedRiderController.compute
    calls = []
    def counting(self, *args, **kwargs):
        calls.append((kwargs.get('advance', True), kwargs.get('dt_s')))
        return original(self, *args, **kwargs)
    monkeypatch.setattr(ArticulatedRiderController, 'compute', counting)
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 200.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(Path(__file__).resolve().parents[2] /
                                'examples/research/viewer_physics_welded.toml'),
        '--track-file', str(track), '--dt', '.0005', '--duration', '.2',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    _fixture_save_env_request(
        env, Path(__file__).resolve().parents[2] /
        'examples/research/viewer_physics_welded.toml', track)
    sim = env.sim
    start = len(calls)
    ctrl = None
    powers = []
    for i in range(403):
        sim.step(control=RideControl(motor_torque_nm=0., human_torque_nm=0.))
        if i % 10:
            np.testing.assert_array_equal(sim.data.ctrl, ctrl)
        ctrl = sim.data.ctrl.copy()
        for sample in sim.physical.completed_samples:
            assert sample.channels['rider_effort_observation'] == 'solved_actuator_force_at_incoming_interval'
            powers.append(sample.channels['rider_positive_power_w'])
    advancing = [dt for advance, dt in calls[start:] if advance]
    assert len(advancing) == 41
    assert advancing == [.005]*41
    assert len(powers) == 400
    assert np.isfinite(powers).all()
    sim.physical.flush()
    assert len(sim.physical.completed_samples) == 3
    before_reset = len(calls)
    sim.reset()
    assert sim.steps == 0
    assert all(dt == .0005 for _, dt in calls[before_reset:])
    sim.step(control=RideControl(motor_torque_nm=0., human_torque_nm=0.))
    assert calls[-1] == (True, .005)
