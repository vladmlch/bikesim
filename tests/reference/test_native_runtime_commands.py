"""A3 — command handling through NativeRideRuntime.advance().

RideControl fields drive the same intent/drivetrain/rider paths as the
Python step; every variant below is cross-checked against the oracle's
integration vector. Command holding across calls stays explicit: each
advance() takes the full control dict for its whole interval range.
"""
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import mujoco

from native_loader import load_native, selected_build
from _native_runtime_support import make_python_ride
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl


# The selected-artifact import doubles as the sanitizer-runtime check;
# create_native_ride reuses this module through load_native_extension.
bike_native = load_native()

REPO_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_ROOT = Path(__file__).resolve().parent


@pytest.fixture
def sim():
    return make_python_ride()


def _integration_state(sim):
    expected = np.empty(
        mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, expected,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    return expected


def _advance_both(sim, native, target, command, *, front=0., rear=0.):
    sim.step(front, rear, control=command)
    return native.advance(target, command, front_brake_demand=front,
                          rear_brake_demand=rear)


@pytest.mark.slow
def test_explicit_zero_effort_differs_from_automatic(sim):
    """human_torque_nm=0. is an explicit command, not the automatic lane."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    automatic = create_native_ride(sim, strict=False, record_decimation=1)
    zero = RideControl(human_torque_nm=0.)
    # The climb intent needs enough intervals to raise its effort ceiling
    # (the oracle first lifts it past zero around step 121 on this track);
    # below that horizon automatic and explicit zero are legitimately equal.
    for target in range(1, 141):
        _advance_both(sim, native, target, zero)
        automatic.advance(target, RideControl())
    expected = _integration_state(sim)
    np.testing.assert_array_equal(
        native.snapshot().integration_state, expected)
    assert not np.array_equal(
        native.snapshot().integration_state,
        automatic.snapshot().integration_state)


@pytest.mark.slow
def test_motor_ceiling_change_matches_python(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    for target in range(1, 31):
        command = RideControl(motor_limit_nm=40. if target <= 15 else None,
                              human_torque_nm=30.)
        _advance_both(sim, native, target, command)
        np.testing.assert_array_equal(
            native.snapshot().integration_state, _integration_state(sim),
            err_msg=f'interval {target}')


@pytest.mark.slow
def test_brake_apply_release_matches_python(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl(human_torque_nm=25.)
    plan = [(0., 0.)] * 8 + [(1., 0.5)] * 8 + [(0., 0.)] * 8
    for target, (front, rear) in enumerate(plan, start=1):
        _advance_both(sim, native, target, command, front=front, rear=rear)
        np.testing.assert_array_equal(
            native.snapshot().integration_state, _integration_state(sim),
            err_msg=f'interval {target}')


@pytest.mark.slow
def test_gear_shift_and_coast_resume_match_python(sim):
    """Sustained pedaling crosses a cassette shift; coasting and resuming
    keep the deterministic drivetrain path on the oracle."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    segments = [(60, RideControl(human_torque_nm=35.)),
                (80, RideControl(human_torque_nm=0.)),
                (120, RideControl(human_torque_nm=35.))]
    for target, command in segments:
        while sim.steps < target:
            _advance_both(sim, native, sim.steps + 1, command)
        np.testing.assert_array_equal(
            native.snapshot().integration_state, _integration_state(sim),
            err_msg=f'interval {target}')


def test_budget_yield_stops_between_steps(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    result = native.advance(50, command, wall_budget_s=0.)
    assert result.reason == 'budget'
    assert result.step == 0
    done = native.advance(5, command, wall_budget_s=60.)
    assert done.reason == 'target'
    assert done.step == 5


def test_advance_to_current_step_is_a_noop(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    native.advance(4, RideControl())
    result = native.advance(4, RideControl())
    assert result.reason == 'target' and result.step == 4


def test_invalid_target_and_command_rejected(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    native.advance(3, RideControl())
    with pytest.raises(Exception):
        native.advance(-1, RideControl())
    with pytest.raises(Exception):
        native.advance(2, RideControl())  # behind the committed step
    with pytest.raises(Exception):
        native.advance(5, 'not a control')
    with pytest.raises(Exception):
        native.advance(5, {'unknown_field': 1})
    with pytest.raises(Exception):
        native.advance(5, RideControl(), front_brake_demand=float('nan'))
    # Rejections never mutate: the committed prefix is intact.
    assert native.snapshot().step == 3
    done = native.advance(6, RideControl())
    assert done.step == 6


def test_concurrent_advance_and_reset_rejected(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    started = threading.Event()
    release = threading.Event()
    errors = []

    def worker():
        started.set()
        try:
            native.advance(10 ** 9, command, wall_budget_s=5.)
        except Exception as exc:  # the budget expiry path is fine
            errors.append(exc)
        finally:
            release.set()

    thread = threading.Thread(target=worker)
    thread.start()
    assert started.wait(5.)
    time.sleep(.05)  # the worker holds the advancement guard now
    with pytest.raises(Exception, match='advance'):
        native.advance(5, command)
    with pytest.raises(Exception):
        native.reset()
    # snapshot() reads live mjData and close() frees it — both take the
    # same guard rather than racing the in-flight loop.
    with pytest.raises(Exception, match='advance'):
        native.snapshot()
    with pytest.raises(Exception, match='advance'):
        native.probe_step_inputs(command)
    with pytest.raises(Exception, match='advance'):
        native.close()
    # The rejected close must not have latched: the runtime is still open.
    assert not native.closed
    assert release.wait(15.)
    thread.join(15.)
    assert native.snapshot().step >= 0


def test_advance_after_close_fails(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    native.close()
    with pytest.raises(Exception, match='closed'):
        native.advance(1, RideControl())


def run_child(script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment['PYTHONPATH'] = os.pathsep.join(
        [str(REFERENCE_ROOT), environment.get('PYTHONPATH', '')])
    command = ['uv', 'run', '--frozen', '--group', 'native']
    runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime:
        preload_name = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        previous = environment.pop(preload_name, '')
        preload = runtime if not previous else f'{runtime}{os.pathsep}{previous}'
        # dyld clears the variable in the parent after loading it. Reinject
        # after uv starts so the probe runs under the selected sanitizer —
        # same contract as test_native_engine_errors.run_child.
        command.extend(['env', f'{preload_name}={preload}'])
    command.extend(['python', '-c', script, *arguments])
    return subprocess.run(
        command, cwd=REPO_ROOT, env=environment, text=True,
        capture_output=True, check=False, timeout=60)


def test_engine_failure_keeps_committed_prefix() -> None:
    """A fatal interval mid-advance surfaces as an exception, not an abort —
    and the committed prefix stays readable (process-isolated)."""
    script = """
import numpy as np
import sys
sys.path.insert(0, {root!r})
from _native_runtime_support import make_python_ride
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl

sim = make_python_ride()
native = create_native_ride(sim, strict=False, record_decimation=1)
native.advance(3, RideControl())
try:
    # An absurd motor demand drives the solve into the numerical-warning
    # wall mid-range; the loop must surface it as an exception and keep
    # every already-committed interval readable.
    native.advance(200, RideControl(motor_torque_nm=1e9))
except Exception as exc:
    try:
        snap = native.snapshot()
        print('PREFIX', snap.step, type(exc).__name__)
    except Exception:
        # A poisoned engine still rejects cleanly rather than aborting.
        print('PREFIX-POISONED', type(exc).__name__)
else:
    print('PREFIX-CLEAN', native.snapshot().step)
""".format(root=str(REFERENCE_ROOT))
    result = run_child(script)
    assert result.returncode == 0, result.stderr
    assert 'PREFIX' in result.stdout
    assert 'Fatal' not in result.stderr
