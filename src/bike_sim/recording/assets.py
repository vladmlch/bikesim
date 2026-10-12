"""Capture recipe bytes when setup identity is established, not at export."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
from io import BytesIO
from pathlib import Path

from bike_sim.recording.contracts import Asset
from bike_sim.recording.json_codec import dumps, loads

RIDER_ASSETS = (('joint_envelope', 'rider_joint_envelope.json'),
                ('joint_strength', 'rider_joint_strength.json'))


def asset(name, media_type, content):
    if not isinstance(content, bytes):
        raise TypeError('asset content must be immutable bytes')
    return Asset(name, media_type, hashlib.sha256(content).hexdigest(), content)


def snapshot_assets(*, track, vertices, metadata, recipe=None):
    """No output files. Caller supplies the already resolved model identity."""
    import numpy as np
    from bike_sim.terrain import dump_track
    terrain = BytesIO()
    np.save(terrain, np.asarray(vertices, dtype='float64'), allow_pickle=False)
    result = [asset('track.toml', 'application/toml', dump_track(track).encode('utf-8')),
              asset('terrain_vertices.npy', 'application/x-npy', terrain.getvalue()),
              asset('resolved_config.json', 'application/json',
                    (dumps(metadata['resolved_config']) + '\n').encode('utf-8'))]
    resolved = metadata['resolved_config']
    articulated = resolved['physics'].get('articulated', {})
    for key, name in RIDER_ASSETS:
        path = articulated.get(key + '_path')
        expected = resolved.get(key + '_sha256')
        if path is None:
            continue
        content = Path(path).read_bytes()
        bundled = asset(name, 'application/json', content)
        if expected is None or bundled.sha256 != expected:
            raise ValueError(f'{key} changed before recording setup could snapshot it')
        result.append(bundled)
    if recipe is not None:
        result.append(asset('recipe.json', 'application/json', (dumps(recipe) + '\n').encode('utf-8')))
    return tuple(result)


def research_recipe(env):
    return dict(replay_contract_version=3, start_x_m=env.sim.start_x_m,
                heightfield=asdict(env.sim.field), state_signature='mjSTATE_INTEGRATION',
                execution=env.execution,
                **{key + '_file': name for key, name in RIDER_ASSETS
                   if env.metadata['resolved_config']['physics']['articulated'].get(key + '_path') is not None})


def replace_recipe(assets, recipe):
    return tuple(value for value in assets if value.name != 'recipe.json') + (
        asset('recipe.json', 'application/json', (dumps(recipe) + '\n').encode('utf-8')),)


def identity_changes(metadata):
    """End-of-run status only. Changed content never replaces captured bytes."""
    from bike_sim.validation.environment import source_fingerprint
    from bike_sim.native.artifact import execution_provenance
    from bike_sim.sim import ride
    source_root = Path(ride.__file__).resolve().parents[2]
    # configuration_metadata hashes the bike_sim source tree, not the repository.
    current_source = source_fingerprint(source_root)
    backend = metadata.get('backend', metadata.get('execution', {}).get('backend', 'python'))
    changes = dict(source_changed_during_run=current_source != metadata.get('model_source_sha256'))
    if 'execution' in metadata:
        changes['execution_changed_during_run'] = execution_provenance(backend) != metadata['execution']
    resolved = metadata['resolved_config']
    articulated = resolved['physics'].get('articulated', {})
    inputs = {}
    for key, _ in RIDER_ASSETS:
        path = articulated.get(key + '_path')
        if path is None:
            continue
        try:
            digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            inputs[key] = {'status': 'unchanged' if digest == resolved.get(key + '_sha256') else 'changed',
                           'setup_sha256': resolved.get(key + '_sha256'), 'current_sha256': digest}
        except OSError as error:
            inputs[key] = {'status': 'unavailable', 'setup_sha256': resolved.get(key + '_sha256'),
                           'error': f'{type(error).__name__}: {error}'}
    changes['input_identity_at_finalization'] = inputs
    return changes
