"""Self-contained checked replay; JSON/NPZ only, never pickle or executed code.

Rebuild static equilibrium from the saved physical configuration and verify the
initial integration state. Replay every control/brake request and compare every
policy observation and ground-truth transition as well as the final engine state.
The initial state is checked, not teleported over an uninitialized material state.
"""
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import mujoco
import numpy as np
from bike_sim.sim.ride.physical_samples import plain

FILES = ('summary.json', 'commands_requested.jsonl', 'commands_applied.jsonl',
         'observations.jsonl', 'transitions.jsonl', 'states.npz',
         'terrain_vertices.npy', 'track.toml')


def integration_state(sim):
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(sim.model, signature))
    mujoco.mj_getState(sim.model, sim.data, state, signature)
    return state


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save_replay_files(env, path):
    path = Path(path)
    np.savez_compressed(path/'states.npz', initial=env.initial_integration_state,
                        final=integration_state(env.sim))
    with (path/'transitions.jsonl').open('w', encoding='utf-8') as stream:
        for result in env.trace:
            stream.write(json.dumps(plain(asdict(result)), sort_keys=True, allow_nan=False)+'\n')
    manifest = dict(schema_version=1, start_x_m=env.sim.start_x_m,
                    heightfield=asdict(env.sim.field),
                    state_signature='mjSTATE_INTEGRATION',
                    file_sha256={name: _sha256(path/name) for name in FILES})
    (path/'replay.json').write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False)+'\n')


def validate_recording(directory):
    """Validate fixed local paths and checksums before constructing a simulator."""
    path = Path(directory)
    manifest = json.loads((path/'replay.json').read_text())
    if not isinstance(manifest, dict) or manifest.get('schema_version') != 1:
        raise ValueError('unsupported replay schema')
    if manifest.get('state_signature') != 'mjSTATE_INTEGRATION':
        raise ValueError('unsupported replay state signature')
    hashes = manifest.get('file_sha256', {})
    if set(hashes) != set(FILES):
        raise ValueError('replay manifest must cover the complete fixed recording file set')
    for name in FILES:
        if _sha256(path/name) != hashes[name]:
            raise ValueError(f'recording checksum mismatch: {name}')
    summary = json.loads((path/'summary.json').read_text())
    return summary, manifest


def _check_runtime(summary):
    from bike_sim.validation.environment import source_fingerprint
    current = {'python': platform.python_version(),
               **{name: importlib.metadata.version(name) for name in ('mujoco', 'numpy', 'scipy')}}
    if current != summary['versions']:
        raise ValueError(f'replay requires the recorded runtime versions: {summary["versions"]}; found {current}')
    source_root = Path(__file__).resolve().parents[2]
    if source_fingerprint(source_root) != summary['model_source_sha256']:
        raise ValueError('replay source hash differs from the recorded plant')
    research = summary['research']
    if research.get('source_changed_during_run'):
        raise ValueError('source changed during recording; exact replay is not a valid claim')
    if research.get('error'):
        raise ValueError('recording contains a failed solve; preserved evidence is not a completed replayable transition')


def _state_error(actual, expected, label, *, atol=1e-9, rtol=1e-9):
    if actual.shape != expected.shape or not np.isfinite(expected).all():
        raise ValueError(f'{label}: incompatible or nonfinite state')
    error = float(np.max(np.abs(actual-expected))) if actual.size else 0.
    if not np.allclose(actual, expected, rtol=rtol, atol=atol):
        raise ValueError(f'{label} differs (maximum absolute error {error:.9g})')
    return error


def _rebuild(path, summary, manifest):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.physics.mass import BikeMassSpecs
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.resolution import resolve_physics_config
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
    from bike_sim.sim.research.rider_program import RiderProgram
    from bike_sim.sim.research.sensors import SensorConfig
    from bike_sim.terrain.heightfield import HeightFieldSpec
    from bike_sim.terrain.trackfile import load_track
    _check_runtime(summary)
    data = summary['resolved_config']
    cfg = resolve_physics_config(data['physics'])
    sim = RideSimulation(track=load_track(path/'track.toml'),
        specs=BikeSpecs(**data['geometry_and_suspension']), mass_specs=BikeMassSpecs(**data['mass_budget']),
        rider=RiderSpecs(**data['rider']), physics_config=cfg,
        field=HeightFieldSpec(**manifest['heightfield']), start_x_m=manifest['start_x_m'])
    research = summary['research']
    program = research.get('rider_program')
    experiment = dict(research['config'], seed=research['actual_sensor_seed'])
    env = ResearchEnvironment(sim, ExperimentConfig(**experiment), SensorConfig(**research['sensor_config']),
        rider_program=None if program is None else RiderProgram.from_dict(program))
    if env.metadata['configuration_sha256'] != summary['configuration_sha256']:
        raise ValueError('reconstructed configuration differs; custom runtime/suspension overrides are not reproducible from this recipe')
    if env.metadata['terrain_sha256'] != summary['terrain_sha256']:
        raise ValueError('reconstructed terrain differs from the recorded compiled road')
    vertices = np.load(path/'terrain_vertices.npy', allow_pickle=False)
    _state_error(sim.physical.vertices, vertices, 'compiled terrain', atol=0., rtol=0.)
    with np.load(path/'states.npz', allow_pickle=False) as states:
        _state_error(env.initial_integration_state, states['initial'], 'initial integration state')
    return env


def rebuild_environment(directory):
    """Return the checked fresh plant for replay or a new, paired torque policy."""
    path = Path(directory)
    summary, manifest = validate_recording(path)
    return _rebuild(path, summary, manifest)


def _rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _compare(actual, expected, label):
    """Recursive finite comparison of JSON-compatible state/measurement trees."""
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(actual) != set(expected):
            raise ValueError(f'{label}: different fields')
        for key in expected:
            _compare(actual[key], expected[key], label+'.'+key)
    elif isinstance(expected, list):
        if not isinstance(actual, (tuple, list)) or len(actual) != len(expected):
            raise ValueError(f'{label}: different array length')
        for i, (a, b) in enumerate(zip(actual, expected)):
            _compare(a, b, f'{label}[{i}]')
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if (isinstance(actual, bool) or not isinstance(actual, (int, float))
            or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)):
            raise ValueError(f'{label}: numerical replay mismatch ({actual!r} vs {expected!r})')
    elif actual != expected:
        raise ValueError(f'{label}: replay mismatch ({actual!r} vs {expected!r})')


def replay_episode(directory):
    """Raise on incompatible or diverging replay; return measured errors on success."""
    from bike_sim.physics.rider_posture import RiderPosture
    from bike_sim.sim.ride.control import RideControl
    path = Path(directory)
    summary, manifest = validate_recording(path)
    env = _rebuild(path, summary, manifest)
    commands = _rows(path/'commands_requested.jsonl')
    transitions = _rows(path/'transitions.jsonl')
    observations = _rows(path/'observations.jsonl')
    if len(commands) != len(transitions) or len(observations) != len(commands)+1:
        raise ValueError('inconsistent recorded command/transition counts')
    _compare(plain(asdict(env.observation)), observations[0], 'initial observation')
    for i, (command, expected) in enumerate(zip(commands, transitions)):
        if env.done or command['step'] != env.sim.steps:
            raise ValueError(f'command {i}: episode ended early or step does not align')
        _compare(env.sim.time_s, command['time_s'], f'command {i} time')
        value = dict(command['control'])
        if value.get('posture') is not None:
            value['posture'] = RiderPosture(**value['posture'])
        result = env.step(RideControl(**value), front_brake_demand=command['front_brake_demand'],
                          rear_brake_demand=command['rear_brake_demand'])
        _compare(plain(asdict(result)), expected, f'transition {i}')
        _compare(plain(asdict(env.observation)), observations[i+1], f'observation {i+1}')
    _compare(plain(env.commands_applied), _rows(path/'commands_applied.jsonl'), 'applied commands')
    _compare(env.reason, summary['outcome']['reason'] if env.done else None, 'outcome')
    _compare(env.tracker.metrics, summary['research']['metrics'], 'event metrics')
    with np.load(path/'states.npz', allow_pickle=False) as states:
        final_error = _state_error(integration_state(env.sim), states['final'], 'final integration state')
    return dict(passed=True, replayed_control_steps=len(commands), physics_steps=env.sim.steps,
        final_state_max_abs_error=final_error, outcome=env.reason, numerically_valid=env.numerically_valid,
        model_valid=env.model_valid, source_sha256=env.metadata['model_source_sha256'],
        scope='Deterministic software replay under the recorded source/runtime, not physical calibration.')
