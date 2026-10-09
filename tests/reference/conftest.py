"""Track-B regression fixtures. Importing these tests never builds an artifact."""
from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from numbers import Real
from pathlib import Path
import json
import os
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))


def _assert_tree(actual, expected, *, atol=1e-9, rtol=1e-9):
    if is_dataclass(actual):
        actual = asdict(actual)
    if is_dataclass(expected):
        expected = asdict(expected)
    if isinstance(expected, Mapping):
        assert isinstance(actual, Mapping)
        assert set(actual) == set(expected)
        for key in expected:
            _assert_tree(actual[key], expected[key], atol=atol, rtol=rtol)
    elif isinstance(expected, np.ndarray):
        np.testing.assert_allclose(actual, expected, atol=atol, rtol=rtol)
    elif isinstance(expected, (tuple, list)):
        assert len(actual) == len(expected)
        for left, right in zip(actual, expected):
            _assert_tree(left, right, atol=atol, rtol=rtol)
    elif expected is None or isinstance(expected, (str, bool)):
        assert type(actual) is type(expected)
        assert actual == expected
    elif isinstance(expected, Real):
        assert not isinstance(actual, bool)
        np.testing.assert_allclose(actual, expected, atol=atol, rtol=rtol)
    else:
        assert actual == expected


@pytest.fixture
def assert_tree():
    return _assert_tree


@pytest.fixture(scope='session')
def native_module():
    # Match the A-track artifact selector. A missing/mismatched extension is a
    # failed gate, not a skip and never an invitation to compile implicitly.
    from bike_sim.native.artifact import load_native_extension
    selected = os.environ.get('NATIVE_TEST_BUILD_PATH')
    if selected is not None:
        path = Path(selected)
        if not path.is_absolute():
            pytest.fail('NATIVE_TEST_BUILD_PATH must be absolute')
        application = os.environ.get('BIKE_NATIVE_BUILD_PATH')
        if application is not None and Path(application).resolve() != path.resolve():
            pytest.fail('test and application native artifact selectors disagree')
        os.environ['BIKE_NATIVE_BUILD_PATH'] = str(path.resolve())
    return load_native_extension()


@pytest.fixture
def raw_sensor():
    from bike_sim.sim.research.sensors import SensorObservation

    def make(time_s=0.):
        return SensorObservation(time_s=time_s, source_time_s=time_s, valid=True,
            specific_force_body_mps2=(.25, -1.5, 9.81), pitch_rate_up_rad_s=.03,
            front_wheel_rad_s=2., rear_wheel_rad_s=3., crank_rad_s=4.,
            motor_torque_nm=5., human_torque_nm=6.)
    return make


@pytest.fixture
def environment_factory(tmp_path):
    """The supplied spindle/pin/connect, ideal-drive flat-road recipe."""
    from bike_sim.cli.research import make_environment, parser
    environments = []

    def create(*, duration=.04, seed=19, decimation=1, actuator_delay=.005,
               program=None, demand=None, strict=False):
        arguments = ['--physics-config', str(ROOT/'examples/research/viewer_physics_welded.toml'),
            '--scenario', 'flat', '--duration', str(duration), '--seed', str(seed),
            '--record-decimation', str(decimation), '--actuator-delay', str(actuator_delay),
            ]
        if not strict:
            arguments.append('--diagnostic-model-limits')
        index = len(environments)
        if program is not None:
            lines = [f'reaction_delay_s = {program.reaction_delay_s!r}']
            for frame in program.keyframes:
                lines += ['', '[[keyframes]]', f'time_s = {frame.time_s!r}']
                if frame.human_torque_nm is not None:
                    lines.append(f'human_torque_nm = {frame.human_torque_nm!r}')
                lines.append('[keyframes.posture]')
                for key, value in asdict(frame.posture).items():
                    if value is not None:
                        lines.append(f'{key} = {json.dumps(value)}')
            path = tmp_path/f'rider-{index}.toml'
            path.write_text('\n'.join(lines)+'\n', encoding='utf-8')
            arguments += ['--rider-program', str(path)]
        if demand is not None:
            lines = []
            for time_s, torque in demand.keyframes:
                lines += ['[[keyframes]]', f'time_s = {time_s!r}', f'torque_nm = {torque!r}', '']
            path = tmp_path/f'demand-{index}.toml'
            path.write_text('\n'.join(lines), encoding='utf-8')
            arguments += ['--demand-file', str(path)]
        environment = make_environment(parser().parse_args(arguments))
        environments.append(environment)
        return environment

    yield create
    for environment in reversed(environments):
        if not environment.control_pending:
            environment.close()


@pytest.fixture
def environment_pair(environment_factory, native_module):
    from bike_sim.native.research import create_native_research
    native_environments = []

    def create(*, behavior=None, **kwargs):
        reference = environment_factory(**kwargs)
        if behavior is not None:
            reference.rider_behavior = behavior
            behavior.reset(reference.seed)
        native = create_native_research(reference)
        native_environments.append(native)
        return reference, native

    yield create
    for native in reversed(native_environments):
        if native.control_pending:
            # Cleanup must not finish a pending physical interval as a hidden
            # extra simulation. Explicit destruction only, after assertions.
            native._native.close()
            if native._directory is not None:
                native._directory.cleanup()
        else:
            native.close()
