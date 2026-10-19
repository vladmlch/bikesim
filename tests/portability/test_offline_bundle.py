"""Offline source contracts with tiny, inert files; no builds or downloads.

Synthetic wheels and model bytes exercise inventories and dependency metadata,
not installation or MuJoCo qualification. Git source selection uses an explicit
index double so current, deleted, ignored and generated files remain distinct.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import PurePosixPath
from types import SimpleNamespace
from zipfile import ZipFile, ZipInfo

from packaging.tags import Tag
import pytest

from tools import offline_bundle as bundle
from tools.offline_manifest import include_source_path, source_digest, validate_bundle_manifest


TOOLS = ('pip', 'setuptools', 'uv', 'wheel')
APP_PINS = (('mujoco', '3.12.0'), ('numpy', '2.5.2'), ('scipy', '1.17.0'),
            ('nanobind', '1.0'), ('threadpoolctl', '1.0'), ('hypothesis', '1.0'))


def sha(content):
    return hashlib.sha256(content).hexdigest()


def put(root, name, content):
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content.encode() if isinstance(content, str) else content)
    return path


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')


def wheel(root, name, version='1.0', *, requires=(), extras=(), tag='py3-none-any',
          metadata_name=None, info_name=None, wheel_version='1.0', requires_python='>=3.13', added=()):
    root.mkdir(parents=True, exist_ok=True)
    dist = name.replace('-', '_')
    info = info_name or f'{dist}-{version}.dist-info'
    path = root / f'{dist}-{version}-{tag}.whl'
    metadata = (f'Metadata-Version: 2.4\nName: {metadata_name or name}\nVersion: {version}\n'
                f'Requires-Python: {requires_python}\n'
                + ''.join(f'Requires-Dist: {value}\n' for value in requires)
                + ''.join(f'Provides-Extra: {value}\n' for value in extras) + '\n')
    version_header = '' if wheel_version is None else f'Wheel-Version: {wheel_version}\n'
    with ZipFile(path, 'w') as archive:
        archive.writestr(f'{info}/METADATA', metadata)
        archive.writestr(f'{info}/WHEEL', version_header + f'Root-Is-Purelib: true\nTag: {tag}\n')
        archive.writestr(f'{info}/RECORD', '')
        if name == 'uv':
            archive.writestr(f'{info}/licenses/LICENSE', 'Synthetic test data; no third-party code.\n')
        archive.writestr(f'{dist}/__init__.py', "raise AssertionError('wheel code must never execute')\n")
        for member, content in added:
            archive.writestr(member, content)
    return path


@pytest.fixture
def target():
    return dict(system='Linux', machine='x86_64', python_minor='3.13',
        implementation='cpython', libc_floor='glibc>=2.28',
        accepted_wheel_tags=['cp313-cp313-manylinux_2_28_x86_64', 'py3-none-any'])


@pytest.fixture
def environment(target, monkeypatch):
    # Make the fixture independent of the test runner's OS and Python patch.
    monkeypatch.setattr(bundle, '_host_identity', lambda: ('Darwin', 'arm64', '3.14'))
    return bundle.marker_environment(target)


@pytest.fixture(autouse=True)
def no_execution(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('source validation started a process or download')
    monkeypatch.setattr(bundle.subprocess, 'run', forbidden)
    monkeypatch.setattr(bundle.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(bundle.urllib.request, 'urlopen', forbidden)


@pytest.fixture
def source(tmp_path):
    root = tmp_path / 'source'
    files = {
        'README.md': 'Synthetic source fixture.\n',
        'src/bike_sim/__init__.py': '# inert fixture\n',
        'native/CMakeLists.txt': 'add_library(core src/core.cpp)\ninclude(cmake/flags.cmake)\n',
        'native/src/core.cpp': '// no build is performed\n',
        'native/cmake/flags.cmake': '# fixture\n',
        'native/tests/smoke.cpp': '// fixture\n',
        'tools/dev.py': '# fixture\n',
        'tools/run_tests.sh': '# fixture\n',
        'tools/proto_native_bench/bench.cpp': '// fixture\n',
        'tools/proto_native_bench/state_format.hpp': '// fixture\n',
        'tools/proto_native_bench/artifacts/model.mjb': b'INDEPENDENT FLAT MODEL FIXTURE',
        'packaging/OFFLINE_README_full.md': 'Full source fixture; installation pending.\n',
        'packaging/OFFLINE_README_slim.md': 'Slim source fixture; installation pending.\n',
    }
    requirements = [f'{name}=={version}' for name, version in APP_PINS]
    files['pyproject.toml'] = (
        '[project]\nname="bike-sim"\nversion="0.1.0"\nrequires-python=">=3.13"\n'
        f'dependencies={json.dumps(requirements[:3])}\n'
        '[dependency-groups]\n'
        f'native={json.dumps(requirements[3:5])}\ndev={json.dumps(requirements[5:])}\n'
        '[build-system]\nrequires=["setuptools>=1.0"]\nbuild-backend="setuptools.build_meta"\n')
    files['uv.lock'] = 'version = 1\n'
    for name, content in files.items():
        put(root, name, content)
    base = 'tools/proto_native_bench/artifacts'
    put(root, f'{base}/current.json', json.dumps({'format': 1, 'bundle': 'selected'}))
    checksums = {}
    for filename, field in [('model.mjb', 'model_mjb_sha256'), ('state.bin', 'state_bin_sha256'),
                            ('state.npz', 'state_npz_sha256')]:
        content = f'INERT SELECTED {filename}'.encode()
        put(root, f'{base}/selected/{filename}', content)
        checksums[field] = sha(content)
    put(root, f'{base}/selected/manifest.json', json.dumps({'checksums': checksums}))
    return root


@pytest.fixture
def extracted_bundle(tmp_path, source, target, environment):
    """A real, internally consistent extracted source bundle, never installed."""
    root = tmp_path / 'bundle'
    root.mkdir()
    source.rename(root / 'project')
    source = root / 'project'
    records = [bundle.inspect_wheel(wheel(root / 'wheelhouse', name, version), target)
               for name, version in APP_PINS + tuple((name, '1.0') for name in TOOLS)]
    records.sort(key=lambda record: record['wheel']['distribution'])
    by_name = {record['wheel']['distribution']: record for record in records}
    lock_lines, requirements = ['version = 1'], []
    for name, version in APP_PINS:
        row = by_name[name]['wheel']
        lock_lines.extend(['[[package]]', f'name={json.dumps(name)}', f'version={json.dumps(version)}',
            'source={registry="https://example.invalid/simple"}',
            'wheels=[{url=' + json.dumps('https://example.invalid/' + row['filename'])
            + ',hash=' + json.dumps('sha256:' + row['sha256']) + '}]'])
        requirements.append(f'{name}=={version} --hash=sha256:{row["sha256"]}')
    put(source, 'uv.lock', '\n'.join(lock_lines) + '\n')
    requirements_file = put(root, 'requirements.txt', '\n'.join(requirements) + '\n')
    rows = [{'path': path.relative_to(source).as_posix(), 'sha256': sha(path.read_bytes())}
            for path in sorted(source.rglob('*')) if path.is_file()]
    manifest = dict(schema_version=1,
        project=dict(name='bike-sim', version='0.1.0', git_commit='a'*40, dirty=False,
            python_source_sha256=source_digest(rows, 'src/'), native_source_sha256=source_digest(rows, 'native/')),
        target=target, mode='full',
        capabilities=dict(python=True, native_source_build=True, development=True, gui=False),
        requirements=dict(lock_sha256=sha((source / 'uv.lock').read_bytes()),
            requirements_sha256=sha(requirements_file.read_bytes()), groups=['native', 'dev']),
        wheels=[record['wheel'] for record in records], source_files=rows,
        prerequisites=dict(python='CPython 3.13.*', compiler_feature_profile='bike-cxx23-strict-local-probes-required',
            cmake_minimum='3.27', uv_version='1.0', system_libraries=['glibc>=2.28', 'preinstalled compiler']),
        slim_external=[])
    roots = [f'{name}=={version}' for name, version in APP_PINS]
    roots += [f'{name}==1.0' for name in TOOLS] + ['setuptools>=1.0']
    closure = dict(schema_version=1, target=target, marker_environment=environment, roots=roots,
                   wheels=records, dependency_edges={name: [] for name in by_name})
    tools = dict(schema_version=1, target=target, uv_version='1.0',
                 wheels=[by_name[name]['wheel'] for name in TOOLS])
    write_json(root / 'manifest.json', manifest)
    write_json(root / 'offline-tools.json', tools)
    write_json(root / 'dependency-closure.json', closure)
    write_json(root / 'assembly-report.json', dict(schema_version=1, status='assembled-not-installed',
        installation_qualification='pending-T25', native_extension_built=False,
        tool_lock_sha256=sha((root / 'offline-tools.json').read_bytes()),
        dependency_closure_sha256=sha((root / 'dependency-closure.json').read_bytes()),
        source=dict(source_modes={row['path']: (source / row['path']).stat().st_mode & 0o777 for row in rows})))
    put(root, 'README.md', 'Synthetic bundle; installation has not been qualified.\n')
    return root


@pytest.fixture
def manifest(extracted_bundle):
    return json.loads((extracted_bundle / 'manifest.json').read_text())


def test_source_digest_ignores_row_order_and_unrelated_source_groups():
    rows = [{'path': 'src/bike_sim/a.py', 'sha256': '1'*64},
            {'path': 'native/src/a.cpp', 'sha256': '2'*64}]
    assert source_digest(rows, 'src/') == source_digest(rows[::-1], 'src/')
    changed = deepcopy(rows)
    changed[1]['sha256'] = '3'*64
    assert source_digest(rows, 'src/') == source_digest(changed, 'src/')
    assert source_digest(rows, 'native/') != source_digest(changed, 'native/')
    assert source_digest(rows + [{'path': 'native', 'sha256': '4'*64}], 'native/') \
        == source_digest(rows, 'native/')


@pytest.mark.parametrize('name', ['native/CMakeLists.txt', 'native/cmake/BikeNumerics.cmake',
    'src/bike_sim/__init__.py', 'uv.lock', 'tests/fixtures/input.json',
    'tools/proto_native_bench/artifacts/model.mjb'])
def test_authored_sources_and_required_fixtures_are_included(name):
    assert include_source_path(PurePosixPath(name))


@pytest.mark.parametrize('name', ['native/build/release/bike_native.so', 'native/build/asan/CMakeCache.txt',
    '.venv/bin/python', '.venv-macos/bin/python', '.git/config', '.aws/credentials',
    'output/run/run.duckdb', 'dist/package.whl', '.env', 'src/__pycache__/module.pyc', '../outside'])
def test_generated_private_and_escaping_paths_are_excluded(name):
    assert not include_source_path(PurePosixPath(name))


def test_valid_native_manifest_and_actual_payload_are_nonexecuting(manifest, extracted_bundle, no_execution):
    assert validate_bundle_manifest(manifest) == manifest
    assert bundle.check_bundle(extracted_bundle) == manifest


@pytest.mark.parametrize('schema', [0, 2, True])
def test_manifest_schema_is_explicit_and_not_a_boolean(manifest, schema):
    manifest['schema_version'] = schema
    with pytest.raises(ValueError, match='schema_version'):
        validate_bundle_manifest(manifest)


@pytest.mark.parametrize('missing', ['uv.lock', 'pyproject.toml', 'native/CMakeLists.txt',
    'native/src/core.cpp', 'native/cmake/flags.cmake', 'native/tests/smoke.cpp', 'tools/dev.py'])
def test_manifest_requires_lock_native_sources_and_development_tools(manifest, missing):
    manifest['source_files'] = [row for row in manifest['source_files'] if row['path'] != missing]
    with pytest.raises(ValueError, match='source'):
        validate_bundle_manifest(manifest)


@pytest.mark.parametrize('missing', ['numpy', 'mujoco', 'scipy', 'nanobind', 'hypothesis',
                                    'threadpoolctl', *TOOLS])
def test_manifest_requires_exact_scientific_development_and_tool_closure(manifest, missing):
    manifest['wheels'] = [row for row in manifest['wheels'] if row['distribution'] != missing]
    with pytest.raises(ValueError, match=missing):
        validate_bundle_manifest(manifest)


@pytest.mark.parametrize('field,value', [('system', 'Windows'), ('machine', 'arm64'),
    ('python_minor', '3.12'), ('implementation', 'pypy'),
    ('accepted_wheel_tags', ['cp314-cp314-manylinux_2_28_x86_64']),
    ('accepted_wheel_tags', ['cp313-cp313-macosx_14_0_arm64'])])
def test_manifest_rejects_wrong_platform_python_and_tags(manifest, field, value):
    manifest['target'][field] = value
    with pytest.raises(ValueError, match='target'):
        validate_bundle_manifest(manifest)


@pytest.mark.parametrize('mutation,match', [
    (lambda value: value['requirements'].update(groups=['native']), 'groups'),
    (lambda value: value['requirements'].update(lock_sha256='f'*64), 'uv.lock'),
    (lambda value: value['project'].update(python_source_sha256='f'*64), 'source.*hash'),
    (lambda value: value['wheels'].append(deepcopy(value['wheels'][0])), 'duplicate'),
    (lambda value: value['wheels'][0].update(sha256='not-a-hash'), 'SHA256'),
    (lambda value: value['wheels'][0].update(filename='../outside.whl'), 'unsafe'),
    (lambda value: value['source_files'].append(dict(path='../outside', sha256='a'*64)), 'unsafe'),
    (lambda value: value['source_files'].append(dict(path='.aws/credentials', sha256='a'*64)), 'private'),
    (lambda value: value['capabilities'].update(native_extension=True), 'fields'),
    (lambda value: value['capabilities'].update(development=False), 'development'),
    (lambda value: value['capabilities'].update(gui=True), 'rendering'),
])
def test_manifest_rejects_inconsistent_or_overstated_claims(manifest, mutation, match):
    mutation(manifest)
    with pytest.raises(ValueError, match=match):
        validate_bundle_manifest(manifest)


def test_python_only_manifest_requires_matching_pure_project_wheel(manifest, tmp_path, target):
    manifest['capabilities'].update(native_source_build=False, development=False)
    manifest['requirements']['groups'] = []
    manifest['prerequisites'].update(cmake_minimum=None, compiler_feature_profile=None)
    project = bundle.inspect_wheel(wheel(tmp_path, 'bike-sim', '0.1.0'), target)['wheel']
    manifest['wheels'].append(project)
    assert validate_bundle_manifest(manifest) == manifest
    manifest['wheels'][-1] = bundle.inspect_wheel(
        wheel(tmp_path, 'bike-sim', '0.1.0', tag='cp313-cp313-manylinux_2_28_x86_64'), target)['wheel']
    with pytest.raises(ValueError, match='pure Python'):
        validate_bundle_manifest(manifest)


def test_slim_bundle_keeps_exact_omissions_and_pending_qualification(manifest, extracted_bundle, no_execution):
    omitted = next(row for row in manifest['wheels'] if row['distribution'] == 'numpy')
    manifest['wheels'].remove(omitted)
    manifest['mode'] = 'slim'
    manifest['slim_external'] = [dict(distribution='numpy', version='2.5.2',
                                    qualification_profile='native-exact-provider-pending-T25')]
    (extracted_bundle / 'wheelhouse' / omitted['filename']).unlink()
    write_json(extracted_bundle / 'manifest.json', manifest)
    assert bundle.check_bundle(extracted_bundle) == manifest
    manifest['slim_external'][0]['qualification_profile'] = 'qualified'
    with pytest.raises(ValueError, match='qualification'):
        validate_bundle_manifest(manifest)


@pytest.mark.parametrize('name', ['project/src/bike_sim/__init__.py',
                                 'project/tools/proto_native_bench/artifacts/model.mjb', 'requirements.txt',
                                 'wheelhouse/numpy-2.5.2-py3-none-any.whl'])
def test_payload_tampering_is_detected(extracted_bundle, no_execution, name):
    path = extracted_bundle / name
    path.write_bytes(path.read_bytes() + b'\nchanged')
    with pytest.raises(ValueError, match='hash|SHA256'):
        bundle.check_bundle(extracted_bundle)


@pytest.mark.parametrize('name', ['project/forgotten.txt', 'wheelhouse/forgotten.txt', 'private.txt'])
def test_undeclared_payload_is_rejected(extracted_bundle, no_execution, name):
    put(extracted_bundle, name, 'not inventoried')
    with pytest.raises(ValueError, match='undeclared'):
        bundle.check_bundle(extracted_bundle)


def test_bundle_readme_cannot_be_a_directory_hiding_unlisted_files(extracted_bundle):
    (extracted_bundle / 'README.md').unlink()
    put(extracted_bundle, 'README.md/undeclared.txt', 'not a bundle instruction file')
    with pytest.raises(ValueError, match='README.md: expected a regular file'):
        bundle.check_bundle(extracted_bundle)


def test_bundle_requires_requested_host_match(extracted_bundle, environment, no_execution):
    with pytest.raises(ValueError, match='current OS/architecture/Python'):
        bundle.check_bundle(extracted_bundle, require_host=True)


def test_extracted_bundle_rejects_symlink_and_changed_source_mode(extracted_bundle, no_execution):
    path = extracted_bundle / 'project/tools/run_tests.sh'
    original_mode = path.stat().st_mode & 0o777
    path.chmod(original_mode ^ 0o100)
    with pytest.raises(ValueError, match='mode mismatch'):
        bundle.check_bundle(extracted_bundle)
    path.chmod(original_mode)
    readme = extracted_bundle / 'README.md'
    readme.unlink()
    readme.symlink_to(extracted_bundle / 'project/README.md')
    with pytest.raises(ValueError, match='symlink'):
        bundle.check_bundle(extracted_bundle)


def test_dependency_evidence_cannot_invent_an_edge(extracted_bundle, no_execution):
    path = extracted_bundle / 'dependency-closure.json'
    closure = json.loads(path.read_text())
    closure['dependency_edges']['numpy'] = ['pip']
    write_json(path, closure)
    report_path = extracted_bundle / 'assembly-report.json'
    report = json.loads(report_path.read_text())
    report['dependency_closure_sha256'] = sha(path.read_bytes())
    write_json(report_path, report)
    with pytest.raises(ValueError, match='edge evidence'):
        bundle.check_bundle(extracted_bundle)


@pytest.mark.parametrize('options,match', [
    ({'metadata_name': 'different'}, 'name/version'),
    ({'info_name': 'different-1.0.dist-info'}, 'dist-info'),
    ({'info_name': 'nested/demo-1.0.dist-info'}, 'dist-info'),
    ({'wheel_version': None}, 'Wheel-Version'),
    ({'wheel_version': '2.0'}, 'Wheel-Version'),
    ({'requires': ('helper @ https://example.invalid/helper.whl',)}, 'direct'),
    ({'added': (('../escape.py', 'bad'),)}, 'unsafe'),
    ({'added': (('DEMO/__init__.py', 'collision'),)}, 'collid'),
])
def test_wheel_metadata_and_member_rejections_are_specific(tmp_path, target, options, match):
    path = wheel(tmp_path, 'demo', **options)
    with pytest.raises(ValueError, match=match):
        bundle.inspect_wheel(path, target)


def test_wheel_symlink_member_is_rejected(tmp_path, target):
    member = ZipInfo('demo/link')
    member.create_system = 3
    member.external_attr = 0o120777 << 16
    path = wheel(tmp_path, 'demo', added=((member, 'elsewhere'),))
    with pytest.raises(ValueError, match='symlink'):
        bundle.inspect_wheel(path, target)


def test_wheel_requires_target_tags(tmp_path, target):
    path = wheel(tmp_path, 'demo', tag='cp313-cp313-macosx_14_0_arm64')
    with pytest.raises(ValueError, match='target tags'):
        bundle.inspect_wheel(path, target)


def test_dependency_extras_retain_their_parent_marker_context(tmp_path, target, environment):
    root = bundle.inspect_wheel(wheel(tmp_path, 'root', requires=('helper>=1; extra == "fast"',),
                                      extras=('fast',)), target)
    helper = bundle.inspect_wheel(wheel(tmp_path, 'helper'), target)
    assert bundle.validate_dependency_closure([root], ['root==1.0'], environment) == {'root': []}
    with pytest.raises(ValueError, match='missing helper'):
        bundle.validate_dependency_closure([root], ['root[fast]==1.0'], environment)
    assert bundle.validate_dependency_closure([root, helper], ['root[fast]==1.0'], environment) == {
        'root': ['helper'], 'helper': []}


def test_dependency_closure_rejects_version_conflicts_and_unreachable_wheels(tmp_path, target, environment):
    record = bundle.inspect_wheel(wheel(tmp_path, 'helper'), target)
    with pytest.raises(ValueError, match='bundled 1.0'):
        bundle.validate_dependency_closure([record], ['helper>=2'], environment)
    with pytest.raises(ValueError, match='unreachable'):
        bundle.validate_dependency_closure([record], [], environment)


@pytest.mark.parametrize('case,match', [('python', 'excludes the host Python'),
    ('host-dependency', 'host build-tool closure.*missing host-helper')])
def test_cross_target_tool_wheels_also_require_author_compatibility(
        tmp_path, target, environment, monkeypatch, case, match):
    records = []
    for name in TOOLS:
        requires_python = '>=3.13' if name == 'pip' and case == 'python' else '>=3.12'
        requires = ('host-helper>=1; sys_platform == "darwin"',) if name == 'pip' and case == 'host-dependency' else ()
        records.append(bundle.inspect_wheel(wheel(tmp_path, name, requires=requires,
                                                  requires_python=requires_python), target))
    project = {'build-system': {'requires': ['setuptools>=1.0']}}
    roots = [f'{name}==1.0' for name in TOOLS] + project['build-system']['requires']
    assert bundle.validate_dependency_closure(records, roots, environment) == {name: [] for name in TOOLS}
    host = dict(environment, platform_system='Darwin', platform_machine='arm64', sys_platform='darwin',
                platform_release='24.0', python_version='3.12', python_full_version='3.12.8',
                implementation_version='3.12.8')
    monkeypatch.setattr(bundle, 'default_environment', lambda: host)
    monkeypatch.setattr(bundle, 'sys_tags', lambda: iter([Tag('py3', 'none', 'any')]))
    with pytest.raises(ValueError, match=match):
        bundle._host_build_tools(records, project)


def test_frozen_export_requires_exact_hashed_active_pins(environment):
    text = f'# frozen export\nhelper==1.0 \\\n    --hash=sha256:{"a"*64}\n'
    rows = bundle.parse_frozen_requirements(text, environment)
    assert [(row['name'], row['version'], row['hashes']) for row in rows] == [('helper', '1.0', ['a'*64])]
    for invalid in ['helper>=1.0 --hash=sha256:' + 'a'*64, 'helper==1.0', text + text,
                    'helper @ https://example.invalid/helper.whl --hash=sha256:' + 'a'*64]:
        with pytest.raises(ValueError):
            bundle.parse_frozen_requirements(invalid, environment)


def test_native_target_selection_uses_actual_host_tags(monkeypatch):
    tags = [Tag('cp313', 'cp313', 'manylinux_2_28_x86_64'), Tag('py3', 'none', 'any')]
    monkeypatch.setattr(bundle, '_host_identity', lambda: ('Linux', 'x86_64', '3.13'))
    monkeypatch.setattr(bundle.platform, 'python_implementation', lambda: 'CPython')
    monkeypatch.setattr(bundle.platform, 'libc_ver', lambda: ('glibc', '2.28'))
    monkeypatch.setattr(bundle, 'sys_tags', lambda: iter(tags))
    selected = bundle.select_target('host', '3.13', True)
    assert selected['accepted_wheel_tags'] == [str(tag) for tag in tags]
    with pytest.raises(ValueError, match='target/host mismatch'):
        bundle.select_target('macos-arm64', '3.13', True)
    with pytest.raises(ValueError, match='actual interpreter'):
        bundle.select_target('host', '3.14', True)


class GitIndex:
    def __init__(self, root, names=None, *, status='', stages=None):
        self.root, self.status, self.stages = root, status, stages or {}
        self.names = names if names is not None else [path.relative_to(root).as_posix()
            for path in root.rglob('*') if path.is_file()]
        self.commands = []

    def run(self, argv, cwd, *, binary=False, env=None):
        assert cwd == self.root and argv[0] == 'git'
        self.commands.append(argv)
        if argv[1:] == ['rev-parse', '--show-toplevel']:
            return str(self.root) + '\n'
        if argv[1:] == ['rev-parse', 'HEAD']:
            return 'a'*40 + '\n'
        if argv[1:] == ['status', '--porcelain=v1', '--untracked-files=normal']:
            return self.status
        if argv[1:] == ['status', '--porcelain=v1', '--untracked-files=no']:
            return ''.join(line for line in self.status.splitlines(keepends=True) if not line.startswith('?? '))
        assert binary and argv[1:] == ['ls-files', '--stage', '-z']
        return b''.join(f'{self.stages.get(name, "100644 " + "0"*40 + " 0")}\t{name}\0'.encode()
                        for name in self.names)


def test_source_selection_uses_current_bytes_and_preserves_dirty_deletion(source, no_execution):
    put(source, 'old.txt', 'previously tracked')
    log = GitIndex(source, status=' M src/bike_sim/__init__.py\n D old.txt\n?? private-note.txt\n')
    (source / 'old.txt').unlink()
    put(source, 'src/bike_sim/__init__.py', '# current uncommitted bytes\n').chmod(0o755)
    put(source, 'private-note.txt', 'untracked; not approved')
    rows, evidence = bundle.collect_source_files(source, log, native=False, assets=[])
    selected = {name: (path, mode) for name, path, mode in rows}
    assert selected['src/bike_sim/__init__.py'][0].read_text() == '# current uncommitted bytes\n'
    assert selected['src/bike_sim/__init__.py'][1] == 0o755
    assert not {'old.txt', 'private-note.txt'} & selected.keys()
    assert evidence['dirty'] and evidence['git_commit'] == 'a'*40


def test_source_selection_excludes_host_binaries_and_arbitrary_build_roots(source, no_execution):
    for name in ['native/build/libmujoco.dylib', '.venv/bin/python', 'alternate/CMakeCache.txt',
                 'alternate/generated.py', 'another/native_check_context.json', 'another/generated.py']:
        put(source, name, 'must not ship')
    rows, evidence = bundle.collect_source_files(source, GitIndex(source), native=False, assets=[])
    selected = {name for name, _, _ in rows}
    assert 'native/CMakeLists.txt' in selected
    assert not any(name.startswith(('native/build/', '.venv/', 'alternate/', 'another/')) for name in selected)
    assert 'native/build/libmujoco.dylib' in evidence['excluded']


@pytest.mark.parametrize('relative', ['uv.lock', 'README.md'])
def test_dirty_deletion_of_required_source_is_rejected(source, no_execution, relative):
    log = GitIndex(source, status=f' D {relative}\n')
    (source / relative).unlink()
    with pytest.raises(ValueError, match='required source'):
        bundle.collect_source_files(source, log, native=False, assets=[])


@pytest.mark.parametrize('destination', ['outside', 'private', 'ordinary'])
def test_source_symlinks_are_confined_and_cannot_disguise_private_files(source, tmp_path, no_execution, destination):
    target = (put(tmp_path, 'outside.txt', 'outside') if destination == 'outside'
              else put(source, '.venv/token.txt', 'private') if destination == 'private'
              else source / 'README.md')
    link = source / 'alias.txt'
    link.symlink_to(target)
    log = GitIndex(source)
    if destination == 'ordinary':
        rows, _ = bundle.collect_source_files(source, log, native=False, assets=[])
        assert next(path for name, path, _ in rows if name == 'alias.txt') == target
    else:
        with pytest.raises(ValueError, match='escaping|excluded'):
            bundle.collect_source_files(source, log, native=False, assets=[])


def test_native_assets_include_flat_and_versioned_models_without_equating_them(source, no_execution):
    required = bundle.required_native_assets(source)
    flat = 'tools/proto_native_bench/artifacts/model.mjb'
    selected = 'tools/proto_native_bench/artifacts/selected/model.mjb'
    assert {flat, selected, 'tools/proto_native_bench/artifacts/selected/state.bin',
            'tools/proto_native_bench/artifacts/selected/state.npz'} <= required
    assert (source / flat).read_bytes() != (source / selected).read_bytes()
    tracked = [path.relative_to(source).as_posix() for path in source.rglob('*')
               if path.is_file() and path.relative_to(source).as_posix() not in required]
    rows, evidence = bundle.collect_source_files(source, GitIndex(source, tracked), native=True, assets=[])
    assert required <= {name for name, _, _ in rows}
    assert evidence['dirty'] and required <= set(evidence['approved_assets'])


@pytest.mark.parametrize('relative', ['tools/proto_native_bench/artifacts/model.mjb',
    'tools/proto_native_bench/artifacts/selected/model.mjb',
    'tools/proto_native_bench/artifacts/selected/state.bin',
    'tools/proto_native_bench/artifacts/selected/state.npz'])
def test_missing_required_binary_assets_cannot_be_fabricated(source, no_execution, relative):
    (source / relative).unlink()
    with pytest.raises(ValueError, match='asset|missing'):
        bundle.required_native_assets(source)


def test_required_binary_asset_hash_is_enforced(source, no_execution):
    put(source, 'tools/proto_native_bench/artifacts/selected/state.bin', b'tampered')
    with pytest.raises(ValueError, match='hash mismatch'):
        bundle.required_native_assets(source)


@pytest.mark.parametrize('stage', ['160000 ' + '0'*40 + ' 0', '100644 ' + '0'*40 + ' 1'])
def test_submodule_and_conflicted_index_require_an_explicit_resolution(source, no_execution, stage):
    with pytest.raises(ValueError, match='submodule|conflicts'):
        bundle.collect_source_files(source, GitIndex(source, stages={'README.md': stage}), native=False, assets=[])


def test_assembler_rejects_missing_lock_before_output_or_external_commands(source, tmp_path, no_execution):
    (source / 'uv.lock').unlink()
    output = tmp_path / 'not-created'
    with pytest.raises(ValueError, match='uv.lock is absent'):
        bundle.assemble(SimpleNamespace(project=source, output=output))
    assert not output.exists()


def test_assembler_rejects_missing_tool_lock_before_output_or_git(
        source, tmp_path, target, environment, no_execution, monkeypatch):
    monkeypatch.setattr(bundle, 'select_target', lambda *args: target)
    output = tmp_path / 'not-created'
    args = SimpleNamespace(project=source, output=output, target='host', python='3.13', native=True,
                           tool_lock=None, tool_wheelhouse=None)
    with pytest.raises(ValueError, match='missing exact build/tool lock'):
        bundle.assemble(args)
    assert not output.exists()
