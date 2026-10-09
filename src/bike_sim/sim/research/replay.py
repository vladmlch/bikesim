"""Checked recording recipes: fixed local JSON/NPZ data, never executed policy code.

Schema 1 remains readable with its original checksum set. Schema 2 identifies
its backend and covers all exported evidence and referenced rider parameter
bundles. Checked native replay/viewer execution is a separate Track C contract.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import re
import mujoco
import numpy as np
from bike_sim.sim.ride.physical_samples import plain

FILES = ('summary.json', 'commands_requested.jsonl', 'commands_applied.jsonl',
         'observations.jsonl', 'transitions.jsonl', 'states.npz',
         'terrain_vertices.npy', 'track.toml')
SCHEMA2_FILES = FILES + ('trace.csv', 'episode_metrics.json', 'telemetry.csv', 'intervals.jsonl')
ENVELOPE_FILE = 'rider_joint_envelope.json'
STRENGTH_FILE = 'rider_joint_strength.json'


def integration_state(sim):
    if getattr(sim, 'backend', None) == 'native':
        return sim.integration_state()
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


def _json_read(path):
    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON field: {key}')
            result[key] = value
        return result

    def nonfinite(token):
        raise ValueError(f'nonfinite JSON constant: {token}')

    def finite_float(token):
        value = float(token)
        if not math.isfinite(value):
            raise ValueError("nonfinite JSON number")
        return value

    return json.loads(Path(path).read_text(encoding='utf-8'),
                      object_pairs_hook=object_pairs, parse_constant=nonfinite, parse_float=finite_float)


def save_replay_files(env, path):
    """Write the manifest only after states, transitions and all bundles exist."""
    from bike_sim.native.artifact import validate_execution
    path = Path(path)
    execution = validate_execution(deepcopy(env.execution))
    np.savez_compressed(path/'states.npz', initial=env.initial_integration_state,
                        final=integration_state(env.sim))
    with (path/'transitions.jsonl').open('w', encoding='utf-8') as stream:
        for result in env.trace:
            stream.write(json.dumps(plain(asdict(result)), sort_keys=True, allow_nan=False)+'\n')
    manifest = dict(schema_version=2, start_x_m=env.sim.start_x_m,
                    heightfield=asdict(env.sim.field), state_signature='mjSTATE_INTEGRATION',
                    execution=execution, file_sha256={name: _sha256(path/name) for name in SCHEMA2_FILES})
    for key, name in (('joint_envelope', ENVELOPE_FILE), ('joint_strength', STRENGTH_FILE)):
        source = getattr(env.sim.physics_config.articulated, key+'_path')
        if source is not None:
            contents = Path(source).read_bytes()
            digest = hashlib.sha256(contents).hexdigest()
            if digest != env.metadata['resolved_config'].get(key+'_sha256'):
                raise ValueError(f'{key} contents changed after research setup')
            (path/name).write_bytes(contents)
            manifest[key+'_file'] = name
            manifest['file_sha256'][name] = digest
    (path/'replay.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')


def validate_recording(directory):
    """Validate fixed files before any recipe can construct a simulator.

    This function never loads an extension, imports a recorded policy or opens
    any pathname contained in a recorded configuration/build context.
    """
    from bike_sim.native.artifact import validate_execution
    path = Path(directory)
    if (path/'replay.json').is_symlink():
        raise ValueError('recording manifest must not be a symbolic link')
    manifest = _json_read(path/'replay.json')
    if (not isinstance(manifest, dict) or type(manifest.get('schema_version')) is not int
            or manifest['schema_version'] not in (1, 2)):
        raise ValueError('unsupported replay schema')
    version = manifest['schema_version']
    mandatory = {'schema_version', 'start_x_m', 'heightfield', 'state_signature', 'file_sha256'}
    optional = {'joint_envelope_file'}
    if version == 2:
        mandatory.add('execution')
        optional.add('joint_strength_file')
    if not mandatory <= set(manifest) or set(manifest)-mandatory-optional:
        raise ValueError('unsupported replay manifest fields')
    if manifest['state_signature'] != 'mjSTATE_INTEGRATION':
        raise ValueError('unsupported replay state signature')
    start = manifest['start_x_m']
    try:
        valid_start = type(start) in (float, int) and math.isfinite(start)
    except OverflowError:
        valid_start = False
    if not valid_start or not isinstance(manifest['heightfield'], dict):
        raise ValueError('invalid replay start/heightfield recipe')
    if version == 2:
        validate_execution(manifest['execution'])
    hashes = manifest['file_sha256']
    if not isinstance(hashes, dict):
        raise ValueError('replay file_sha256 must be an object')
    required = set(FILES if version == 1 else SCHEMA2_FILES)
    for key, name in (('joint_envelope_file', ENVELOPE_FILE), ('joint_strength_file', STRENGTH_FILE)):
        if key in manifest:
            if manifest[key] != name:
                raise ValueError(f'unsupported replay {key} path')
            required.add(name)
    if set(hashes) != required:
        raise ValueError('replay manifest must cover the complete fixed recording file set')
    for name in sorted(required):
        expected = hashes[name]
        if not isinstance(expected, str) or re.fullmatch('[0-9a-f]{64}', expected) is None:
            raise ValueError(f'invalid recording checksum: {name}')
        if (path/name).is_symlink() or not (path/name).is_file():
            raise ValueError(f'recording requires a regular local file: {name}')
        if _sha256(path/name) != expected:
            raise ValueError(f'recording checksum mismatch: {name}')
    summary = _json_read(path/'summary.json')
    if not isinstance(summary, dict):
        raise ValueError('recording summary must be an object')
    if version == 2:
        research = summary.get('research')
        if not isinstance(research, dict) or research.get('execution') != manifest['execution']:
            raise ValueError('summary and manifest execution identities disagree')
        resolved = summary.get("resolved_config")
        if not isinstance(resolved, dict) or not isinstance(resolved.get("physics"), dict):
            raise ValueError("recording resolved physics must be an object")
        articulated = resolved["physics"].get("articulated")
        if not isinstance(articulated, dict):
            raise ValueError("recording articulated recipe must be an object")
        for key, name in (('joint_envelope', ENVELOPE_FILE), ('joint_strength', STRENGTH_FILE)):
            declared = articulated.get(key+'_path') is not None
            bundled = key+'_file' in manifest
            if declared != bundled or (bundled and resolved.get(key+'_sha256') != hashes[name]):
                raise ValueError(f'inconsistent recorded {key} bundle')
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
    if research.get("execution_changed_during_run"):
        raise ValueError("source or binary identity changed during recording")
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
    if manifest["schema_version"] == 2:
        if manifest["execution"]["backend"] == "native":
            raise ValueError("checked native replay requires Track C3; refusing a Python-backend substitution")
        from bike_sim.native.artifact import execution_provenance
        if execution_provenance("python") != manifest["execution"]:
            raise ValueError("replay execution identity differs from this runtime")
    _check_runtime(summary)
    data = deepcopy(summary["resolved_config"])
    for key, filename in (("joint_envelope", ENVELOPE_FILE), ("joint_strength", STRENGTH_FILE)):
        declared = data["physics"]["articulated"].get(key+"_path")
        bundled = manifest.get(key+"_file")
        if declared is not None and bundled is None:
            raise ValueError(f"legacy recording lacks a bundled {key}; external recipe paths are not trusted")
        data["physics"]["articulated"][key+"_path"] = (
            None if bundled is None else str((path/filename).resolve()))
    cfg = resolve_physics_config(data['physics'])
    sim = RideSimulation(track=load_track(path/'track.toml'),
        specs=BikeSpecs(**data['geometry_and_suspension']), mass_specs=BikeMassSpecs(**data['mass_budget']),
        rider=RiderSpecs(**data['rider']), physics_config=cfg,
        field=HeightFieldSpec(**manifest['heightfield']), start_x_m=manifest['start_x_m'])
    research = summary['research']
    program = research.get('rider_program')
    from bike_sim.sim.research.demand import DemandProgram
    demand = research.get('demand_program')
    experiment = dict(research['config'], seed=research['actual_sensor_seed'])
    env = ResearchEnvironment(sim, ExperimentConfig(**experiment), SensorConfig(**research['sensor_config']),
        rider_program=None if program is None else RiderProgram.from_dict(program),
        demand=None if demand is None else DemandProgram.from_dict(demand))
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
    if summary['research'].get('rider_command_recording') != 'program_inputs':
        env.rider_program = None
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
    if summary['outcome']['reason'] == 'operator_stop' and not env.done:
        env.stop()
    _compare(env.reason, summary['outcome']['reason'] if env.done else None, 'outcome')
    _compare(env.tracker.metrics, summary['research']['metrics'], 'event metrics')
    with np.load(path/'states.npz', allow_pickle=False) as states:
        final_error = _state_error(integration_state(env.sim), states['final'], 'final integration state')
    return dict(passed=True, replayed_control_steps=len(commands), physics_steps=env.sim.steps,
        final_state_max_abs_error=final_error, outcome=env.reason, numerically_valid=env.numerically_valid,
        model_valid=env.model_valid, source_sha256=env.metadata['model_source_sha256'],
        scope='Deterministic software replay under the recorded source/runtime, not physical calibration.')
