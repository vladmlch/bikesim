"""Fatal MuJoCo probes must be run only in an isolated child process."""
from pathlib import Path
import json
import os
import subprocess
import sys

from native_loader import load_native


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_engine_failure_preserves_prefix_and_reset_recovers(native_module):
    root = Path(__file__).resolve().parents[2]
    script = r'''
from pathlib import Path
import json
import sys
from bike_sim.cli.research import make_environment, parser
from bike_sim.native.research import create_native_research
from bike_sim.sim.ride.control import RideControl
root = Path(sys.argv[1])
reference = make_environment(parser().parse_args([
    '--physics-config', str(root/'examples/research/viewer_physics_welded.toml'),
    '--scenario', 'flat', '--duration', '.04', '--diagnostic-model-limits',
    '--record-decimation', '1']))
env = create_native_research(reference)
initial = env.snapshot()
env._native._test_fail_at_step(3)
env.begin_control(RideControl(0.))
try:
    env.advance_control()
except Exception as error:
    failure = dict(exception=type(error).__name__, message=str(error),
        step=env.sim.steps, reason=env.reason, error=env.error,
        pending=env.control_pending, transitions=len(env.trace),
        requests=len(env.commands_requested), rows=env.recorder.rows)
else:
    raise AssertionError('fatal engine probe did not raise')
assert failure['step'] == 3
assert failure['reason'] == 'simulation_error'
assert not failure['pending'] and failure['transitions'] == 0
assert failure['requests'] == 1 and failure['rows'] == 3
assert failure['error'] is not None
assert env.snapshot().step == 3
env.reset(seed=29)
assert env.sim.steps == 0 and not env.done
assert env.snapshot().generation > initial.generation
assert env.step(RideControl(0.)).physics_steps == env.control_steps
env.close()
reference.close()
print(json.dumps(failure))
'''
    child_environment = os.environ.copy()
    child_environment['PYTHONPATH'] = str(root/'src') + os.pathsep + child_environment.get('PYTHONPATH', '')
    result = subprocess.run([sys.executable, '-c', script, str(root)], env=child_environment,
                            capture_output=True, text=True, timeout=180, check=False)
    assert result.returncode == 0, result.stdout+'\n'+result.stderr
    report = json.loads(result.stdout.splitlines()[-1])
    assert report['step'] == report['rows'] == 3
