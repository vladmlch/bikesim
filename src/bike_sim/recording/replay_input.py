"""Immutable, data-only inputs to the single checked replay implementation.

Container schema 1 and replay contract 3 are deliberately independent. A
legacy directory retains its original schema-1/2 checksum and shape rules.
Neither adapter imports a policy, opens a recorded pathname, or builds a plant.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from io import BytesIO
import math
from pathlib import Path

import numpy as np

from bike_sim.recording.contracts import TableBatch, freeze
from bike_sim.recording.encoding import decode_rows, encode_rows
from bike_sim.recording.json_codec import loads, thaw
from bike_sim.recording.reader import Recording, RecordingError
from bike_sim.recording.schema import BATCH_ROWS, research_tables, quote


@dataclass(frozen=True)
class ReplayInput:
    summary: object
    manifest: object
    assets: object
    requested: tuple
    applied: tuple
    observations: tuple
    transitions: tuple
    initial_state: np.ndarray
    final_state: np.ndarray
    terrain_vertices: np.ndarray
    metrics: object
    episode_id: int
    container_schema_version: int | None

    def __post_init__(self):
        for name in ('summary', 'manifest', 'assets', 'requested', 'applied',
                     'observations', 'transitions', 'initial_state', 'final_state',
                     'terrain_vertices', 'metrics'):
            object.__setattr__(self, name, freeze(getattr(self, name)))

    @property
    def replay_contract_version(self):
        return self.manifest['schema_version']


def _command_shapes(requested, transitions, observations):
    if len(requested) != len(transitions) or len(observations) != len(requested) + 1:
        raise RecordingError('inconsistent recorded command/transition counts')
    for index, command in enumerate(requested):
        if (not isinstance(command, dict) or set(command) != {
                'step', 'time_s', 'control', 'front_brake_demand', 'rear_brake_demand'}
                or type(command['step']) is not int or command['step'] < 0
                or not isinstance(command['control'], dict)):
            raise RecordingError(f'command {index}: invalid requested-command record')


def _terrain(content):
    value = np.load(BytesIO(content), allow_pickle=False)
    if (not isinstance(value, np.ndarray) or value.dtype != np.dtype('float64')
            or value.ndim != 2 or value.shape[1] != 2 or not np.isfinite(value).all()):
        raise RecordingError('terrain must be a finite float64 N-by-2 array')
    return value


class LegacyReplayInput:
    @staticmethod
    def load(directory, *, episode_id=None):
        # Keep the independent old fixed-file contract. Validation never reads
        # paths from a recipe, and this snapshot closes the validate/read race.
        from bike_sim.sim.research.replay import validate_recording, _json_loads
        path = Path(directory)
        if episode_id not in (None, 0) or isinstance(episode_id, bool):
            raise RecordingError('legacy directories contain only episode 0')
        _, manifest = validate_recording(path)
        assets = {}
        for name, digest in manifest['file_sha256'].items():
            contents = (path / name).read_bytes()
            if hashlib.sha256(contents).hexdigest() != digest:
                raise RecordingError(f'recording changed while reading: {name}')
            assets[name] = contents
        summary = _json_loads(assets['summary.json'].decode('utf-8'))
        def rows(name):
            return tuple(_json_loads(line) for line in assets[name].decode('utf-8').splitlines()
                         if line.strip())
        requested, applied = rows('commands_requested.jsonl'), rows('commands_applied.jsonl')
        observations, transitions = rows('observations.jsonl'), rows('transitions.jsonl')
        _command_shapes(requested, transitions, observations)
        with np.load(BytesIO(assets['states.npz']), allow_pickle=False) as states:
            initial, final = states['initial'].copy(), states['final'].copy()
        metrics = (_json_loads(assets['episode_metrics.json'].decode('utf-8'))
                   if manifest['schema_version'] == 2 else None)
        # Legacy arrays retain the old verifier's shape/finite checks, rather
        # than silently redefining old valid data at the format boundary.
        vertices = np.load(BytesIO(assets['terrain_vertices.npy']), allow_pickle=False)
        return ReplayInput(summary, manifest, assets, requested, applied,
                           observations, transitions, initial, final, vertices, metrics, 0, None)


def _stream(connection, spec, episode_id):
    identity = spec.keys[1].key
    result = connection.execute(f'SELECT * FROM {quote(spec.name)} WHERE episode_id=? '
                                f'ORDER BY {quote(identity)}', [episode_id])
    names = tuple(column[0] for column in result.description)
    while values := result.fetchmany(BATCH_ROWS):
        batch = TableBatch(spec.name, {name: tuple(row[index] for row in values)
                                      for index, name in enumerate(names)}, len(values))
        decoded = decode_rows(spec, batch)
        encode_rows(spec, decoded, episode_id=episode_id)
        for row in decoded:
            row.pop('episode_id')
            row.pop(identity)
            yield row


def _validate_recipe(summary, recipe, assets, manifest, selected):
    from bike_sim.native.artifact import validate_execution
    required = {'replay_contract_version', 'start_x_m', 'heightfield', 'state_signature', 'execution'}
    optional = {'joint_envelope_file', 'joint_strength_file'}
    if (not isinstance(recipe, dict) or not required <= set(recipe)
            or set(recipe) - required - optional or type(recipe['replay_contract_version']) is not int
            or recipe['replay_contract_version'] != 3
            or recipe['state_signature'] != 'mjSTATE_INTEGRATION'
            or type(recipe['start_x_m']) not in (int, float)
            or not math.isfinite(recipe['start_x_m']) or not isinstance(recipe['heightfield'], dict)):
        raise RecordingError('invalid replay contract 3 recipe')
    execution = validate_execution(recipe['execution'])
    research = summary.get('research', {})
    if research.get('execution') != execution:
        raise RecordingError('summary and recipe execution identities disagree')
    for key in ('source_changed_during_run', 'configuration_changed_during_run',
                'execution_changed_during_run'):
        if research.get(key) is not False:
            raise RecordingError(f'exact replay requires a checked unchanged identity: {key}')
    if research.get('error') or research.get('run_metadata', {}).get('policy_source_changed_during_run'):
        raise RecordingError('failed or identity-changing recording is not replay eligible')
    resolved = loads(assets['resolved_config.json'].decode('utf-8'))
    if resolved != summary.get('resolved_config'):
        raise RecordingError('bundled and summarized resolved configurations disagree')
    articulated = resolved.get('physics', {}).get('articulated')
    if not isinstance(articulated, dict):
        raise RecordingError('recording articulated recipe must be an object')
    expected_assets = {'track.toml', 'terrain_vertices.npy', 'resolved_config.json', 'recipe.json'}
    for key, name in (('joint_envelope', 'rider_joint_envelope.json'),
                      ('joint_strength', 'rider_joint_strength.json')):
        declared = articulated.get(key + '_path') is not None
        bundled = key + '_file' in recipe
        if declared != bundled or (bundled and (recipe[key + '_file'] != name
                or name not in assets or hashlib.sha256(assets[name]).hexdigest()
                != resolved.get(key + '_sha256'))):
            raise RecordingError(f'inconsistent recorded {key} bundle')
        if bundled:
            expected_assets.add(name)
    if set(assets) != expected_assets:
        raise RecordingError('replay recipe must own exactly its fixed input assets')
    extents = summary.get('recording_extents', {})
    for name, value in (('physical_rows', selected['physical_rows']),
                        ('complete_control_steps', selected['complete_control_steps']),
                        ('replay_eligible', True)):
        if extents.get(name) != value:
            raise RecordingError(f'summary and database extents disagree: {name}')
    # This is a logical verifier recipe, not a rewritten legacy bundle.
    return dict(schema_version=3, start_x_m=recipe['start_x_m'], heightfield=recipe['heightfield'],
                state_signature=recipe['state_signature'], execution=execution,
                **{key: recipe[key] for key in optional if key in recipe},
                container_schema_version=manifest['schema_version'], container_manifest=manifest,
                episode_id=selected['episode_id'])


class DuckDBReplayInput:
    @staticmethod
    def load(directory, *, episode_id=None):
        with Recording(directory) as recording:
            selected = recording.episode(episode_id, research=True)
            if not selected['replay_eligible']:
                raise RecordingError('failure/incomplete-prefix evidence is analyzable but not completed replay')
            eid = selected['episode_id']
            summary = loads(selected['summary_json'])
            assets = {row['name']: bytes(row['content']) for row in recording.rows(
                'SELECT name,content FROM assets WHERE episode_id=? ORDER BY name', [eid])}
            required = {'track.toml', 'terrain_vertices.npy', 'resolved_config.json', 'recipe.json'}
            if not required <= set(assets):
                raise RecordingError('recording is missing fixed replay input assets')
            recipe = loads(assets['recipe.json'].decode('utf-8'))
            manifest = _validate_recipe(summary, recipe, assets, recording.manifest, selected)
            streams = {spec.name: tuple(_stream(recording.connection, spec, eid))
                       for spec in research_tables()}
            observations = streams['observations']
            transitions = []
            for index, value in enumerate(streams['transitions']):
                row = dict(value)
                if (row.pop('request_id'), row.pop('previous_observation_id'),
                    row.pop('observation_id')) != (index, index, index + 1):
                    raise RecordingError('inconsistent transition observation/request references')
                row['observation'] = observations[index + 1]
                transitions.append(row)
            requested, applied = streams['commands_requested'], streams['commands_applied']
            _command_shapes(requested, transitions, observations)
            states = {row['phase']: np.asarray(row['state'], dtype='float64') for row in recording.rows(
                'SELECT phase,state FROM integration_states WHERE episode_id=?', [eid])}
            initial, final = states['initial'], states['final']
            if initial.ndim != 1 or final.shape != initial.shape or not initial.size \
                    or not np.isfinite(initial).all() or not np.isfinite(final).all():
                raise RecordingError('invalid initial/final integration state for checked replay')
            vertices = _terrain(assets['terrain_vertices.npy'])
            if hashlib.sha256(vertices.astype('<f8', copy=False).tobytes()).hexdigest() != summary.get('terrain_sha256'):
                raise RecordingError('terrain identity differs from the bundled compiled road')
            completed_steps = sum(row['physics_steps'] for row in transitions)
            if completed_steps != selected['physical_rows'] or completed_steps != summary['outcome']['steps']:
                raise RecordingError('completed controls do not cover the saved physical prefix')
            expected_step = 0
            for index, command in enumerate(requested):
                if command['step'] != expected_step:
                    raise RecordingError('requested controls do not cover consecutive physical windows')
                expected_step += transitions[index]['physics_steps']
            return ReplayInput(summary, manifest, assets, requested, applied, observations,
                               tuple(transitions), initial, final, vertices,
                               summary.get('episode_metrics'), eid, recording.schema_version)


def load_replay_input(path, *, episode_id=None):
    path = Path(path)
    # Never fall back to an old directory when a new, unsealed database exists.
    if any((path / name).exists() or (path / name).is_symlink()
           for name in ('manifest.json', 'run.duckdb', 'UNSEALED')):
        return DuckDBReplayInput.load(path, episode_id=episode_id)
    return LegacyReplayInput.load(path, episode_id=episode_id)


# ---------------------------------------------------------------------------
# Portable recorded-state playback — inspection data, never a checked recipe
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlaybackInput:
    """Recorded-state playback bundle for a sealed run container.

    Shares the container's data-only validation (manifest, checksums, recipe
    shape, bundled-asset consistency) while deliberately dropping the
    checked-replay run-integrity gates: eligibility, completed-prefix coverage
    and identity-change flags exist for verification, not for displaying the
    frames that were actually recorded. ``frames`` are ``physics`` rows —
    ``interval_id``, ``time_s``, ``interval_end_s``, ``dt_s``, ``qpos``,
    ``qvel`` — for the selected episode only. This object is never an input
    to checked stepping and carries no verdict about physical replay.
    """

    summary: object
    recipe: object
    assets: object
    frames: tuple
    episode_id: int

    def __post_init__(self):
        for name in ('summary', 'recipe', 'assets'):
            object.__setattr__(self, name, freeze(getattr(self, name)))
        object.__setattr__(self, 'frames',
                         tuple(freeze(row) for row in self.frames))


def _validate_playback_recipe(summary, recipe, assets):
    """Structural recipe/asset checks shared with the checked-replay adapter.

    Unlike ``_validate_recipe`` this performs no eligibility, coverage or
    identity-change gating and computes no current-runtime identity —
    ``validate_execution`` only validates the recorded record's own shape.
    """
    required = {'replay_contract_version', 'start_x_m', 'heightfield', 'state_signature', 'execution'}
    optional = {'joint_envelope_file', 'joint_strength_file'}
    if (not isinstance(recipe, dict) or not required <= set(recipe)
            or set(recipe) - required - optional or type(recipe['replay_contract_version']) is not int
            or recipe['replay_contract_version'] != 3
            or recipe['state_signature'] != 'mjSTATE_INTEGRATION'
            or type(recipe['start_x_m']) not in (int, float)
            or not math.isfinite(recipe['start_x_m']) or not isinstance(recipe['heightfield'], dict)):
        raise RecordingError('invalid replay contract 3 recipe')
    from bike_sim.native.artifact import validate_execution
    execution = validate_execution(recipe['execution'])
    research = summary.get('research', {})
    if not isinstance(research, dict) or research.get('execution') != execution:
        raise RecordingError('summary and recipe execution identities disagree')
    bundled = assets.get('resolved_config.json')
    if bundled is None:
        raise RecordingError('recording is missing the bundled resolved configuration')
    if loads(bundled.decode('utf-8')) != summary.get('resolved_config'):
        raise RecordingError('bundled and summarized resolved configurations disagree')
    resolved = summary['resolved_config']
    physics = resolved.get('physics') if isinstance(resolved, dict) else None
    articulated = physics.get('articulated') if isinstance(physics, dict) else None
    if not isinstance(articulated, dict):
        raise RecordingError('recording articulated recipe must be an object')
    expected_assets = {'track.toml', 'terrain_vertices.npy', 'resolved_config.json', 'recipe.json'}
    for key, name in (('joint_envelope', 'rider_joint_envelope.json'),
                      ('joint_strength', 'rider_joint_strength.json')):
        declared = articulated.get(key + '_path') is not None
        bundled = key + '_file' in recipe
        if declared != bundled or (bundled and (recipe[key + '_file'] != name
                or name not in assets or hashlib.sha256(assets[name]).hexdigest()
                != resolved.get(key + '_sha256'))):
            raise RecordingError(f'inconsistent recorded {key} bundle')
        if bundled:
            expected_assets.add(name)
    if set(assets) != expected_assets:
        raise RecordingError('playback recipe must own exactly its fixed input assets')


def load_playback_input(directory, *, episode_id=None):
    """Load the portable playback bundle for one episode of a sealed run.

    Read-only container validation; no extension, policy, recorded path or
    recorded runtime identity is ever loaded or compared. Episodes that are
    not replay-eligible still yield their recorded frames — failure evidence
    is inspectable data, and playback output is labeled unverified.
    """
    try:
        recording = Recording(directory)
    except RecordingError:
        raise
    except Exception as error:
        raise RecordingError(
            f'recording cannot be opened for playback: {type(error).__name__}: {error}') from error
    with recording:
        selected = recording.episode(episode_id)
        eid = selected['episode_id']
        summary = loads(selected['summary_json'])
        assets = {row['name']: bytes(row['content']) for row in recording.rows(
            'SELECT name,content FROM assets WHERE episode_id=? ORDER BY name', [eid])}
        if 'recipe.json' not in assets:
            raise RecordingError(
                'recording does not bundle the authored replay recipe; '
                'recorded-state playback requires the fixed input assets')
        recipe = loads(assets['recipe.json'].decode('utf-8'))
        _validate_playback_recipe(summary, recipe, assets)
        frames = tuple(recording.rows(
            'SELECT interval_id,time_s,interval_end_s,dt_s,qpos,qvel FROM physics '
            'WHERE episode_id=? ORDER BY interval_id', [eid]))
    return PlaybackInput(summary, recipe, assets, frames, eid)
