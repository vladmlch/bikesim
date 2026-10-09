"""Checked recording recipes: fixed local JSON/NPZ data, never executed policy code.

Schema 1 remains readable with its original checksum set. Schema 2 identifies
its backend and covers all exported evidence and referenced rider parameter
bundles. ReplaySession supplies the same incremental verifier to headless and
visual replay, using the recorded backend without loading a policy factory.
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


def _json_loads(text):
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

    return json.loads(text,
                      object_pairs_hook=object_pairs, parse_constant=nonfinite, parse_float=finite_float)


def _json_read(path):
    return _json_loads(Path(path).read_text(encoding='utf-8'))


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
    if actual.shape != expected.shape or not np.isfinite(expected).all() or not np.isfinite(actual).all():
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
    backend = 'python' if manifest['schema_version'] == 1 else manifest['execution']['backend']
    if manifest["schema_version"] == 2:
        from bike_sim.native.artifact import execution_provenance
        if execution_provenance(backend) != manifest["execution"]:
            raise ValueError("replay execution identity differs from this runtime")
    data = deepcopy(summary["resolved_config"])
    for key, filename in (("joint_envelope", ENVELOPE_FILE), ("joint_strength", STRENGTH_FILE)):
        declared = data["physics"]["articulated"].get(key+"_path")
        bundled = manifest.get(key+"_file")
        if declared is not None and bundled is None:
            raise ValueError(f"legacy recording lacks a bundled {key}; external recipe paths are not trusted")
        data["physics"]["articulated"][key+"_path"] = (
            None if bundled is None else str((path/filename).resolve()))
    cfg = resolve_physics_config(data['physics'])
    rider = RiderSpecs(**data['rider'])
    from bike_sim.sim.backend import require_backend
    require_backend(backend, cfg, rider)
    research = summary['research']
    program = research.get('rider_program')
    if backend == 'native' and program is not None and research.get('rider_command_recording') != 'program_inputs':
        raise ValueError('native replay requires the recorded rider program input contract')
    sim = RideSimulation(track=load_track(path/'track.toml'),
        specs=BikeSpecs(**data['geometry_and_suspension']), mass_specs=BikeMassSpecs(**data['mass_budget']),
        rider=rider, physics_config=cfg,
        field=HeightFieldSpec(**manifest['heightfield']), start_x_m=manifest['start_x_m'])
    from bike_sim.sim.research.demand import DemandProgram
    demand = research.get('demand_program')
    experiment = dict(research['config'], seed=research['actual_sensor_seed'])
    env = ResearchEnvironment(sim, ExperimentConfig(**experiment), SensorConfig(**research['sensor_config']),
        rider_program=None if program is None else RiderProgram.from_dict(program),
        demand=None if demand is None else DemandProgram.from_dict(demand))
    try:
        if env.metadata['configuration_sha256'] != summary['configuration_sha256']:
            raise ValueError('reconstructed configuration differs; custom runtime/suspension overrides are not reproducible from this recipe')
        if env.metadata['terrain_sha256'] != summary['terrain_sha256']:
            raise ValueError('reconstructed terrain differs from the recorded compiled road')
        vertices = np.load(path/'terrain_vertices.npy', allow_pickle=False)
        _state_error(sim.physical.vertices, vertices, 'compiled terrain', atol=0., rtol=0.)
        with np.load(path/'states.npz', allow_pickle=False) as states:
            initial = states['initial'].copy()
        _state_error(env.initial_integration_state, initial, 'initial integration state')
        if backend == 'python':
            return env
        from bike_sim.native.research import create_native_research
        native = create_native_research(env)
        try:
            _state_error(native.snapshot().integration_state, initial, 'native initial integration state')
        except BaseException:
            native.close(discard_pending=True)
            raise
        env.close()
        return native
    except BaseException:
        env.close(discard_pending=True)
        raise


def rebuild_environment(directory):
    """Return the checked fresh plant for replay or a new, paired torque policy."""
    path = Path(directory)
    summary, manifest = validate_recording(path)
    return _rebuild(path, summary, manifest)


def _rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [_json_loads(line) for line in stream if line.strip()]


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
            or not math.isfinite(actual)
            or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)):
            raise ValueError(f'{label}: numerical replay mismatch ({actual!r} vs {expected!r})')
    elif type(actual) is not type(expected) or actual != expected:
        raise ValueError(f'{label}: replay mismatch ({actual!r} vs {expected!r})')


def replay_episode(directory):
    """Synchronous convenience over the exact same verifier used by the viewer."""
    from bike_sim.sim.research.replay_session import ReplaySession
    session = ReplaySession(directory)
    try:
        while not session.advance():
            pass
        return session.report()
    finally:
        session.close()
