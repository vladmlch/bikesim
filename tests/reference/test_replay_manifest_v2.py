"""Data-only reader tests use synthetic files, not valid physical rollouts."""
from copy import deepcopy
from hashlib import sha256
import json
import pytest
from bike_sim.native.artifact import EXECUTION_FIELDS, validate_execution
from bike_sim.sim.research.replay import (
    FILES, SCHEMA2_FILES, ENVELOPE_FILE, STRENGTH_FILE, validate_recording,
)


def identity(backend='native'):
    native = backend == 'native'
    return dict(backend=backend, runtime_schema=1, python_source_sha256='1'*64,
        native_source_sha256='2'*64 if native else None,
        extension_sha256='3'*64 if native else None, mujoco_version='3.12.0',
        mujoco_library_sha256='4'*64 if native else None,
        build_context=dict(build_type='Release', compiler_id='AppleClang',
            compiler_version='21.0.0', numerical_flags='-O3 -ffp-contract=off') if native else None)


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def reseal(path, name):
    manifest = json.loads((path/'replay.json').read_text())
    manifest['file_sha256'][name] = digest(path/name)
    write_json(path/'replay.json', manifest)


@pytest.fixture
def recording(tmp_path):
    def create(*, version=2, backend='native', bundles=True):
        path = tmp_path/f'v{version}-{backend}-{bundles}'
        path.mkdir()
        files = FILES if version == 1 else SCHEMA2_FILES
        for name in files:
            (path/name).write_bytes(b'opaque data-only reader fixture\n')
        execution = identity(backend)
        articulated = dict(joint_envelope_path=None, joint_strength_path=None)
        resolved = dict(physics=dict(articulated=articulated))
        summary = dict(resolved_config=resolved, research=dict(execution=execution,
            run_metadata=dict(policy=dict(reference='malicious_recorded_module:factory'))))
        manifest = dict(schema_version=version, start_x_m=.25, heightfield={},
                        state_signature='mjSTATE_INTEGRATION')
        if version == 2:
            manifest['execution'] = execution
        optional = []
        if bundles:
            choices = [('joint_envelope', ENVELOPE_FILE)]
            if version == 2:
                choices.append(('joint_strength', STRENGTH_FILE))
            for key, name in choices:
                (path/name).write_bytes(b'{"data_only":true}\n')
                articulated[key+'_path'] = '/untrusted/recipe/path/'+name
                resolved[key+'_sha256'] = digest(path/name)
                manifest[key+'_file'] = name
                optional.append(name)
        write_json(path/'summary.json', summary)
        manifest['file_sha256'] = {name: digest(path/name) for name in (*files, *optional)}
        write_json(path/'replay.json', manifest)
        return path
    return create


@pytest.mark.parametrize('version,backend', [(1, 'python'), (2, 'python'), (2, 'native')])
def test_fixed_schema_compatibility_without_loading_extension(recording, monkeypatch, version, backend):
    import bike_sim.native.artifact as artifact
    import importlib
    path = recording(version=version, backend=backend)
    def forbidden(*args, **kwargs):
        raise AssertionError('data-only reader attempted an import')
    monkeypatch.setattr(artifact, 'load_native_extension', forbidden)
    monkeypatch.setattr(importlib, 'import_module', forbidden)
    summary, manifest = validate_recording(path)
    assert manifest['schema_version'] == version
    assert summary['research']['run_metadata']['policy']['reference'].startswith('malicious_')


@pytest.mark.parametrize('name', SCHEMA2_FILES+(ENVELOPE_FILE, STRENGTH_FILE))
def test_every_schema2_file_is_authenticated(recording, name):
    path = recording()
    with (path/name).open('ab') as stream:
        stream.write(b'tamper')
    with pytest.raises(ValueError, match='checksum'):
        validate_recording(path)


@pytest.mark.parametrize('mutation', ['missing', 'extra', 'traversal', 'upper_hash', 'unknown_top', 'bool_schema'])
def test_manifest_rejects_unlisted_fields_and_paths(recording, mutation):
    path = recording()
    manifest = json.loads((path/'replay.json').read_text())
    if mutation == 'missing':
        del manifest['file_sha256']['trace.csv']
    elif mutation == 'extra':
        manifest['file_sha256']['other.json'] = '0'*64
    elif mutation == 'traversal':
        manifest['joint_strength_file'] = '../rider_joint_strength.json'
    elif mutation == 'upper_hash':
        manifest['file_sha256']['trace.csv'] = 'A'*64
    elif mutation == 'unknown_top':
        manifest['policy_module'] = 'untrusted'
    else:
        manifest['schema_version'] = True
    write_json(path/'replay.json', manifest)
    with pytest.raises(ValueError):
        validate_recording(path)


def test_schema1_does_not_silently_accept_schema2_checksum_set(recording):
    path = recording(version=1)
    manifest = json.loads((path/'replay.json').read_text())
    manifest['file_sha256']['telemetry.csv'] = '0'*64
    write_json(path/'replay.json', manifest)
    with pytest.raises(ValueError, match='fixed recording file set'):
        validate_recording(path)


@pytest.mark.parametrize('mutation', ['missing_bundle', 'bundle_digest', 'execution', 'malformed_physics'])
def test_summary_identity_and_referenced_bundles_are_cross_checked(recording, mutation):
    path = recording()
    summary = json.loads((path/'summary.json').read_text())
    if mutation == 'missing_bundle':
        summary['resolved_config']['physics']['articulated']['joint_strength_path'] = None
    elif mutation == 'bundle_digest':
        summary['resolved_config']['joint_strength_sha256'] = '0'*64
    elif mutation == 'execution':
        summary['research']['execution']['extension_sha256'] = '0'*64
    else:
        summary['resolved_config']['physics'] = []
    write_json(path/'summary.json', summary)
    reseal(path, 'summary.json')
    with pytest.raises(ValueError):
        validate_recording(path)


def test_symlinked_recorded_file_is_rejected(recording, tmp_path):
    path = recording()
    outside = tmp_path/'outside-trace.csv'
    outside.write_bytes((path/'trace.csv').read_bytes())
    (path/'trace.csv').unlink()
    (path/'trace.csv').symlink_to(outside)
    with pytest.raises(ValueError, match='regular local file'):
        validate_recording(path)


@pytest.mark.parametrize('payload', ['{"schema_version":2,"schema_version":1}',
                                    '{"schema_version":NaN}', '{"schema_version":1e999}'])
def test_duplicate_and_nonfinite_json_fields_are_rejected(recording, payload):
    path = recording()
    (path/'replay.json').write_text(payload)
    with pytest.raises(ValueError):
        validate_recording(path)


@pytest.mark.parametrize('backend', ['python', 'native'])
def test_execution_exact_shape_and_nullability(backend):
    expected = identity(backend)
    assert set(expected) == EXECUTION_FIELDS
    assert validate_execution(deepcopy(expected)) == expected
    for key in EXECUTION_FIELDS:
        bad = deepcopy(expected)
        del bad[key]
        with pytest.raises(ValueError):
            validate_execution(bad)
    for key, value in [('runtime_schema', True), ('backend', 'auto'),
                       ('python_source_sha256', 'not a digest'), ('mujoco_version', '')]:
        bad = dict(expected, **{key: value})
        with pytest.raises(ValueError):
            validate_execution(bad)
    bad = deepcopy(expected)
    if backend == 'native':
        bad['build_context']['injected'] = 'untracked'
    else:
        bad['extension_sha256'] = '0'*64
    with pytest.raises(ValueError):
        validate_execution(bad)


def test_native_source_manifest_has_stable_paths_and_tracks_headers(tmp_path):
    from bike_sim.native.artifact import native_source_fingerprint
    root = tmp_path/'native'
    (root/'src').mkdir(parents=True)
    (root/'cmake').mkdir()
    files = {'CMakeLists.txt': b'project(reference)\n', 'src/core.cpp': b'int value = 1;\n',
             'src/core.hpp': b'#pragma once\n', 'cmake/config.cmake': b'set(VALUE 1)\n'}
    for name, payload in files.items():
        (root/name).write_bytes(payload)
    manifest = ''.join(f'{name}={sha256(files[name]).hexdigest()}\n' for name in sorted(files))
    expected = sha256(manifest.encode()).hexdigest()
    assert native_source_fingerprint(root) == expected
    (root/'build').mkdir()
    (root/'build/bike_native.so').write_bytes(b'not a source input')
    assert native_source_fingerprint(root) == expected
    (root/'src/core.hpp').write_bytes(b'#pragma once\n// changed\n')
    assert native_source_fingerprint(root) != expected
