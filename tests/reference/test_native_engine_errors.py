"""Fatal MuJoCo errors must cross the native boundary without exiting Python."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import subprocess
import sys

import mujoco
import pytest

from native_loader import load_native, selected_build

bike_native = load_native()


REPO_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_ROOT = Path(__file__).resolve().parent


@pytest.fixture
def invalid_solver_model(tmp_path: Path) -> Path:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco>
          <worldbody>
            <geom type="plane" size="1 1 .1"/>
            <body pos="0 0 .09">
              <freejoint/>
              <geom type="sphere" size=".1" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"""
    )
    model.opt.solver = 99
    path = tmp_path / 'invalid-solver.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    return path


def run_child(script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment['PYTHONPATH'] = os.pathsep.join(
        [str(REFERENCE_ROOT), environment.get('PYTHONPATH', '')]
    )
    command = ['uv', 'run', '--frozen', '--group', 'native']
    runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime:
        preload_name = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        previous = environment.pop(preload_name, '')
        preload = runtime if not previous else f'{runtime}{os.pathsep}{previous}'
        # dyld clears the variable in the parent after loading it. Reinject
        # after uv starts, then let native_loader verify the mapped runtime.
        command.extend(['env', f'{preload_name}={preload}'])
    command.extend(['python', '-c', script, *arguments])
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )


def test_stock_binding_raises_fatal_error_and_survives(invalid_solver_model: Path) -> None:
    child = run_child(
        """import mujoco, sys
model = mujoco.MjModel.from_binary_path(sys.argv[1])
data = mujoco.MjData(model)
try:
    mujoco.mj_forward(model, data)
except mujoco.FatalError:
    print('stock FatalError; process alive')
else:
    raise AssertionError('invalid solver was accepted')
""",
        str(invalid_solver_model),
    )
    assert child.returncode == 0, child.stderr
    assert 'stock FatalError; process alive' in child.stdout


@pytest.mark.parametrize('import_order', ['mujoco_first', 'native_first'])
def test_native_fatal_constructor_preserves_process(
    invalid_solver_model: Path, import_order: str
) -> None:
    child = run_child(
        """import sys
if sys.argv[2] == 'mujoco_first':
    import mujoco
    from native_loader import load_native
    bike_native = load_native()
else:
    from native_loader import load_native
    bike_native = load_native()
    import mujoco
try:
    bike_native.Stepper(sys.argv[1])
except mujoco.FatalError:
    assert bike_native.FatalError is mujoco.FatalError
    print('caught FatalError; process alive')
else:
    raise AssertionError('native constructor accepted an invalid solver')
""",
        str(invalid_solver_model),
        import_order,
    )
    assert child.returncode == 0, child.stderr
    assert 'caught FatalError; process alive' in child.stdout


def test_valid_solver_still_constructs_and_forwards(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
    path = tmp_path / 'valid.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import sys
from native_loader import load_native
stepper = load_native().Stepper(sys.argv[1])
stepper.forward()
stepper.step()
print('valid forward completed')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'valid forward completed' in child.stdout


def test_runtime_fatal_poisons_stepper_until_reset(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'runtime-fatal.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
try:
    bike_native._engine_test_forward_failure(stepper)
except mujoco.FatalError:
    pass
else:
    raise AssertionError('runtime engine failure was accepted')
for operation in (stepper.forward, stepper.step):
    try:
        operation()
    except RuntimeError as error:
        assert 'poisoned' in str(error)
    else:
        raise AssertionError('poisoned Stepper accepted a mutation')
assert len(stepper.qpos) == 7  # read-only diagnostics remain accessible
stepper.reset()
stepper.forward()
print('runtime failure recovered after reset')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'runtime failure recovered after reset' in child.stdout


def test_set_const_fatal_poisons_owner_and_preserves_process(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'setconst-fatal.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
try:
    bike_native._engine_test_set_const_failure(stepper)
except mujoco.FatalError as error:
    assert 'simple' in str(error)
else:
    raise AssertionError('setConst failure was accepted')
try:
    stepper.forward()
except RuntimeError as error:
    assert 'poisoned' in str(error)
else:
    raise AssertionError('setConst failure did not poison owner')
assert len(stepper.qpos) == 7
stepper.reset()
stepper.forward()
print('setConst fatal recovered')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'setConst fatal recovered' in child.stdout


def test_valid_set_state_can_recover_after_rejected_restore(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'restore-after-fatal.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, numpy as np, sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
qpos = stepper.qpos.copy()
qvel = stepper.qvel.copy()
warmstart = np.zeros_like(qvel)
act = np.empty(0, dtype=np.float64)
try:
    bike_native._engine_test_forward_failure(stepper)
except mujoco.FatalError:
    pass
else:
    raise AssertionError('fatal injection was accepted')
try:
    stepper.set_state(qpos[:-1], qvel, act, warmstart, 0.)
except ValueError:
    pass
else:
    raise AssertionError('invalid restore was accepted')
try:
    stepper.forward()
except RuntimeError as error:
    assert 'poisoned' in str(error)
else:
    raise AssertionError('rejected restore cleared poison')
stepper.set_state(qpos, qvel, act, warmstart, 0.)
stepper.forward()
print('valid set_state recovered')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'valid set_state recovered' in child.stdout


@pytest.mark.parametrize('recovery', ['reset', 'set_state'])
def test_configured_owner_recovery_clears_pending_drive_state(
    tmp_path: Path, recovery: str
) -> None:
    child = run_child(
        """import mujoco, sys
from pathlib import Path
from test_native_drivetrain import pair, bike_native
model, data, _, stepper = pair(Path(sys.argv[1]))
stepper.drive_components({}, .002, 2.)
before = stepper.drive_state()
assert before['last_time_s'] is not None
assert before['pending_actuation'] is not None
try:
    bike_native._engine_test_forward_failure(stepper)
except mujoco.FatalError:
    pass
else:
    raise AssertionError('configured fatal injection was accepted')
if sys.argv[2] == 'reset':
    stepper.reset()
else:
    stepper.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, data.time)
after = stepper.drive_state()
assert after['last_time_s'] is None
assert after['pending_actuation'] is None
stepper.drive_components({}, .002, 2.)
print('configured owner recovered coherently')
""",
        str(tmp_path),
        recovery,
    )
    assert child.returncode == 0, child.stderr
    assert 'configured owner recovered coherently' in child.stdout


@pytest.mark.parametrize('operation', ['forward', 'step'])
def test_realtime_status_path_reports_fatal_without_throwing_inside_marked_region(
    tmp_path: Path, operation: str
) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'realtime-status.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
success, message = bike_native._engine_test_try_status(stepper, sys.argv[2] == 'step')
assert success is False
assert 'unknown solver type 99' in message
try:
    stepper.forward()
except RuntimeError as error:
    assert 'poisoned' in str(error)
else:
    raise AssertionError('status failure did not poison owner')
stepper.reset()
stepper.forward()
print('realtime status path recovered')
""",
        str(path),
        operation,
    )
    assert child.returncode == 0, child.stderr
    assert 'realtime status path recovered' in child.stdout


def test_installed_python_callback_is_rejected_before_native_construction(
    tmp_path: Path,
) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
mujoco.set_mjcb_control(lambda model, data: None)
try:
    try:
        bike_native.Stepper(sys.argv[1])
    except ValueError as error:
        assert 'unsupported installed MuJoCo callback: mjcb_control' in str(error)
    else:
        raise AssertionError('native constructor accepted an installed callback')
finally:
    mujoco.set_mjcb_control(None)
model = mujoco.MjModel.from_binary_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print('callback rejection left stock binding usable')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'callback rejection left stock binding usable' in child.stdout


def test_stock_time_callback_error_preserves_python_exception(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'time-callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stock_model = mujoco.MjModel.from_binary_path(sys.argv[1])
stock_data = mujoco.MjData(stock_model)
def fail_time():
    raise ValueError('time callback marker')
mujoco.set_mjcb_time(fail_time)
try:
    try:
        mujoco.mj_forward(stock_model, stock_data)
    except ValueError as error:
        assert 'time callback marker' in str(error)
    else:
        raise AssertionError('stock callback error was accepted')
    try:
        bike_native.Stepper(sys.argv[1])
    except ValueError as error:
        assert 'time callback marker' in str(error)
        print('Python callback error survived')
    else:
        raise AssertionError('callback error was accepted')
finally:
    mujoco.set_mjcb_time(None)
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'Python callback error survived' in child.stdout


def test_installed_callback_rejects_runtime_call_without_poison(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    path = tmp_path / 'runtime-callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
before = stepper.time
mujoco.set_mjcb_control(lambda model, data: None)
try:
    try:
        stepper.forward()
    except ValueError as error:
        assert 'unsupported installed MuJoCo callback: mjcb_control' in str(error)
    else:
        raise AssertionError('runtime call accepted callback')
finally:
    mujoco.set_mjcb_control(None)
assert stepper.time == before
stepper.forward()
print('callback rejection left owner healthy')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'callback rejection left owner healthy' in child.stdout


def test_python_time_callback_is_rejected_before_marked_runtime_step(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    path = tmp_path / 'runtime-time-callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
stepper = load_native().Stepper(sys.argv[1])
before = stepper.time
mujoco.set_mjcb_time(lambda: 1.0)
try:
    try:
        stepper.step()
    except ValueError as error:
        assert 'unsupported installed MuJoCo callback: mjcb_time' in str(error)
    else:
        raise AssertionError('Python timer callback entered marked step')
finally:
    mujoco.set_mjcb_time(None)
assert stepper.time == before
stepper.step()
print('runtime timer callback rejected without poison')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'runtime timer callback rejected without poison' in child.stdout


def test_ctypes_time_callback_is_rejected_before_native_construction(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    path = tmp_path / 'ctypes-time-callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import ctypes, mujoco, sys
from native_loader import load_native
bike_native = load_native()
calls = [0]
@ctypes.CFUNCTYPE(ctypes.c_double)
def foreign_timer():
    calls[0] += 1
    return 0.0
mujoco.set_mjcb_time(foreign_timer)
try:
    try:
        bike_native.Stepper(sys.argv[1])
    except ValueError as error:
        assert 'unsupported installed MuJoCo callback: mjcb_time' in str(error)
    else:
        raise AssertionError('native constructor accepted a ctypes timer')
finally:
    mujoco.set_mjcb_time(None)
assert calls[0] == 0, calls
print('ctypes timer rejected before native construction')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'ctypes timer rejected before native construction' in child.stdout


def test_ctypes_time_callback_is_rejected_before_native_recovery(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body pos=".1 0 0">'
        '<geom type="sphere" size=".1"/></body></worldbody></mujoco>'
    )
    path = tmp_path / 'ctypes-recovery-callback.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import ctypes, mujoco, numpy as np, sys
from native_loader import load_native
bike_native = load_native()
stepper = bike_native.Stepper(sys.argv[1])
stepper.step()
before = stepper.time
calls = [0]
@ctypes.CFUNCTYPE(ctypes.c_double)
def foreign_timer():
    calls[0] += 1
    return 0.0
mujoco.set_mjcb_time(foreign_timer)
try:
    for operation in (
        stepper.reset,
        lambda: stepper.set_state(
            np.asarray(stepper.qpos).copy(), np.asarray(stepper.qvel).copy(),
            np.empty(0), np.zeros_like(stepper.qvel), before),
        lambda: bike_native._engine_test_set_const_failure(stepper),
    ):
        try:
            operation()
        except ValueError as error:
            assert 'unsupported installed MuJoCo callback: mjcb_time' in str(error)
        else:
            raise AssertionError('native recovery accepted a ctypes timer')
finally:
    mujoco.set_mjcb_time(None)
assert calls[0] == 0, calls
assert stepper.time == before
stepper.step()
print('ctypes timer rejected before native recovery')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'ctypes timer rejected before native recovery' in child.stdout


def test_unsupported_callback_rejected_before_drive_restore(tmp_path: Path) -> None:
    from test_native_drive_policies import assert_tree
    from test_native_drivetrain import pair

    _, _, _, stepper = pair(tmp_path, 'geometric_ideal_mid_drive')
    before = stepper.drive_state()
    attempted = copy.deepcopy(before)
    attempted['ideal_hub']['prepared'] = None
    mujoco.set_mjcb_control(lambda model, data: None)
    try:
        with pytest.raises(ValueError, match='unsupported installed MuJoCo callback: mjcb_control'):
            stepper.set_drive_state(attempted)
    finally:
        mujoco.set_mjcb_control(None)
    assert_tree(stepper.drive_state(), before)


def test_stock_timer_installed_after_native_construction_is_allowed(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    path = tmp_path / 'late-stock-timer.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
stepper = load_native().Stepper(sys.argv[1])
stock_model = mujoco.MjModel.from_binary_path(sys.argv[1])
stock_data = mujoco.MjData(stock_model)
assert stock_data is not None
assert mujoco.get_mjcb_time() is None
stepper.step()
print('late stock timer allowed')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'late stock timer allowed' in child.stdout


def test_stock_fatal_handler_nests_inside_native_timer_callback(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>"""
    )
    path = tmp_path / 'nested-stock-native.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stock_model = mujoco.MjModel.from_binary_path(sys.argv[1])
stock_data = mujoco.MjData(stock_model)
normal_solver = stock_model.opt.solver
stock_model.opt.solver = 99
active = [False]
caught = [0]
def nested_timer():
    if active[0]:
        return 0.0
    active[0] = True
    try:
        try:
            mujoco.mj_forward(stock_model, stock_data)
        except mujoco.FatalError:
            caught[0] += 1
        else:
            raise AssertionError('nested stock call accepted solver 99')
    finally:
        active[0] = False
    return 0.0
mujoco.set_mjcb_time(nested_timer)
try:
    stepper = bike_native.Stepper(sys.argv[1])
finally:
    mujoco.set_mjcb_time(None)
assert caught[0] > 0
stock_model.opt.solver = normal_solver
mujoco.mj_forward(stock_model, stock_data)
stepper.forward()
try:
    bike_native._engine_test_forward_failure(stepper)
except mujoco.FatalError:
    pass
else:
    raise AssertionError('native handler was not restored after nested stock call')
stepper.reset()
print('nested stock/native handlers restored')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'nested stock/native handlers restored' in child.stdout


def test_nested_stock_native_warning_does_not_recurse(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    valid_path = tmp_path / 'nested-warning-valid.mjb'
    invalid_path = tmp_path / 'nested-warning-invalid.mjb'
    mujoco.mj_saveModel(model, str(valid_path), None)
    invalid_path.write_bytes(b'not an MJB')
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
bike_native = load_native()
stock_model = mujoco.MjModel.from_binary_path(sys.argv[1])
stock_data = mujoco.MjData(stock_model)
depth = [0]
warnings = [0]
def nested_timer():
    previous = depth[0]
    depth[0] += 1
    try:
        if previous == 0:
            mujoco.mj_forward(stock_model, stock_data)
        elif previous == 1:
            try:
                bike_native.Stepper(sys.argv[2])
            except RuntimeError:
                warnings[0] += 1
            else:
                raise AssertionError('invalid nested MJB accepted')
    finally:
        depth[0] -= 1
    return 0.0
mujoco.set_mjcb_time(nested_timer)
try:
    bike_native.Stepper(sys.argv[1])
finally:
    mujoco.set_mjcb_time(None)
assert warnings[0] > 0
print('nested warning forwarded once')
""",
        str(valid_path), str(invalid_path),
    )
    assert child.returncode == 0, child.stderr
    assert 'nested warning forwarded once' in child.stdout


def test_plugin_model_is_rejected_before_native_engine_loading(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><extension><plugin plugin="mujoco.pid"><instance name="pid">'
        '<config key="kp" value="1"/></instance></plugin></extension>'
        '<worldbody><body><joint name="j"/><geom size=".1"/></body></worldbody>'
        '<actuator><plugin joint="j" plugin="mujoco.pid" instance="pid"/>'
        '</actuator></mujoco>'
    )
    assert model.nplugin == 1
    path = tmp_path / 'plugin.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
try:
    load_native().Stepper(sys.argv[1])
except ValueError as error:
    assert 'unsupported MuJoCo plugin model' in str(error)
    print('plugin model rejected before native load')
else:
    raise AssertionError('native owner accepted a plugin model')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'plugin model rejected before native load' in child.stdout


def test_invalid_history_timestep_rejected_before_make_data(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint name="j" type="slide"/>'
        '<geom size=".1"/></body></worldbody><actuator>'
        '<motor joint="j" delay=".05" nsample="5"/></actuator></mujoco>'
    )
    assert model.nhistory > 0
    model.opt.timestep = 0
    path = tmp_path / 'invalid-history-timestep.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    child = run_child(
        """import sys
from native_loader import load_native
try:
    load_native().Stepper(sys.argv[1])
except ValueError as error:
    assert 'history requires positive timestep' in str(error)
    print('invalid history timestep rejected before data allocation')
else:
    raise AssertionError('invalid history timestep accepted')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'invalid history timestep rejected before data allocation' in child.stdout


@pytest.mark.parametrize('payload', [None, b'', b'not an MJB'])
def test_bad_model_file_raises_without_exiting(tmp_path: Path, payload: bytes | None) -> None:
    path = tmp_path / 'bad.mjb'
    if payload is not None:
        path.write_bytes(payload)
    child = run_child(
        """import mujoco, sys
from native_loader import load_native
try:
    load_native().Stepper(sys.argv[1])
except mujoco.FatalError:
    raise AssertionError('ordinary bad MJB became a fatal engine error')
except RuntimeError:
    print('bad MJB rejected; process alive')
else:
    raise AssertionError('bad MJB was accepted')
""",
        str(path),
    )
    assert child.returncode == 0, child.stderr
    assert 'bad MJB rejected; process alive' in child.stdout


@pytest.mark.parametrize(
    'operation', [
        '--load-allocation-failure',
        '--make-data-allocation-failure',
        '--late-data-allocation-failure',
    ]
)
def test_actual_engine_allocation_error_is_caught_in_c_contract(
    tmp_path: Path, operation: str
) -> None:
    assert Path(bike_native.__file__).resolve().parent == selected_build(os.environ).resolve()
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    path = tmp_path / 'valid-for-allocation.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    executable = selected_build(os.environ) / 'native_contract_tests'
    environment = os.environ.copy()
    runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime:
        preload_name = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        previous = environment.get(preload_name, '')
        environment[preload_name] = runtime if not previous else f'{runtime}{os.pathsep}{previous}'
    child = subprocess.run(
        [str(executable), operation, str(path)],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )
    assert child.returncode == 0, child.stderr
    assert 'fatal allocation' in child.stdout


def test_configure_rejects_unsupported_installed_mujoco_version(tmp_path: Path) -> None:
    fake_package = tmp_path / 'fake-python-path' / 'mujoco'
    fake_package.mkdir(parents=True)
    (fake_package / '__init__.py').write_text("__version__ = '3.15.0'\n")
    environment = os.environ.copy()
    environment['PYTHONPATH'] = str(fake_package.parent)
    configured = subprocess.run(
        [
            'uv', 'run', '--frozen', '--group', 'native', 'cmake',
            '-S', str(REPO_ROOT / 'native'),
            '-B', str(tmp_path / 'unsupported-build'),
            f'-DPython_EXECUTABLE={sys.executable}',
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=60,
    )
    assert configured.returncode != 0
    assert 'requires the pinned MuJoCo 3.12.0 error ABI' in configured.stdout + configured.stderr
