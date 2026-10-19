"""T25 regression tests: schema-1 bundle installation preflight and e2e paths.

Fixture bundles are real extracted schema-1 layouts assembled locally from the
*actual* installer sources (the fixture project copies tools/install_offline.py,
tools/offline_bundle.py, tools/offline_manifest.py and the packaging scripts),
so the installer subprocess under test is the shipping code itself.

Every wheel is a tiny inert or repacked host package — nothing is downloaded,
and only python-only installs are exercised end to end on this host (native
compilation needs the real locked scientific/native wheels, which this host
cannot provide; see the module docstring and T25 status notes).
"""
from __future__ import annotations

import hashlib
import importlib.metadata as md
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
from zipfile import ZipFile

import pytest
from packaging.tags import sys_tags

from tools import offline_bundle as bundle
from tools.offline_manifest import source_digest, validate_bundle_manifest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import offline_install_probe as probe  # noqa: E402

TOOLS = ('pip', 'setuptools', 'uv', 'wheel')
UV_VERSION = subprocess.run(['uv', '--version'], capture_output=True,
                            text=True).stdout.strip().split()[1]


def sha(content) -> str:
    if isinstance(content, str):
        content = content.encode()
    return hashlib.sha256(content).hexdigest()


def put(root: Path, name: str, content) -> Path:
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, str):
        content = content.encode()
    path.write_bytes(content)
    return path


def write_json(path: Path, value) -> Path:
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')
    return path


def _machine() -> str:
    system = platform.system()
    machine = platform.machine()
    return {'AMD64': 'x86_64',
            'aarch64': 'arm64' if system == 'Darwin' else 'aarch64'}.get(machine, machine)


@pytest.fixture
def target():
    return dict(
        system=platform.system(), machine=_machine(),
        python_minor=f'{sys.version_info.major}.{sys.version_info.minor}',
        implementation='cpython',
        accepted_wheel_tags=[str(next(iter(sys_tags()))), 'py3-none-any'],
        libc_floor='glibc>=2.28' if platform.system() == 'Linux' else None)


def wheel(root: Path, name: str, version: str = '1.0', *, tag='py3-none-any',
          requires=(), init=None, extra_members=()):
    """Inert wheel whose metadata/RECORD are genuinely installable by uv."""
    dist = name.replace('-', '_')
    info = f'{dist}-{version}.dist-info'
    path = root / f'{dist}-{version}-{tag}.whl'
    members = [
        (f'{info}/METADATA',
         f'Metadata-Version: 2.4\nName: {name}\nVersion: {version}\n'
         f'Requires-Python: >=3.13\n'
         + ''.join(f'Requires-Dist: {value}\n' for value in requires) + '\n'),
        (f'{info}/WHEEL', f'Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: {tag}\n'),
        (f'{info}/RECORD', ''),
        (f'{dist}/__init__.py', init if init is not None
         else f"raise AssertionError('{name} wheel code must never execute')\n"),
    ]
    if name == 'uv':
        members.append((f'{info}/licenses/LICENSE', 'Synthetic test data.\n'))
    members += list(extra_members)
    members = [(member, content) for member, content in members
               if not member.endswith('/RECORD')]
    record = ''.join(f'{member},,\n' for member, _ in members)
    members.append((f'{info}/RECORD', record + f'{info}/RECORD,,\n'))
    path.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(path, 'w') as archive:
        for member, content in members:
            archive.writestr(member, content)
    return path


def uv_executable_wheel(root: Path, version: str = UV_VERSION) -> Path:
    script = f'#!/bin/sh\necho "uv {version}"\n'
    return wheel(root, 'uv', version, init='"""uv shim"""\n', extra_members=[
        ('uv/bin/uv', script),
        (f'uv-{version}.data/scripts/uv', script)])


def repack_installed_wheel(root: Path, dist_name: str) -> Path:
    """A real wheel rebuilt from the installed distribution's own files."""
    dist = md.distribution(dist_name)
    version = dist.version
    site = Path(str(dist.locate_file('')))
    normalized = dist_name.replace('-', '_')
    info_dirs = [path for path in site.iterdir()
                 if path.name.startswith(f'{normalized}-')
                 and path.name.endswith('.dist-info')]
    assert len(info_dirs) == 1, f'no installed dist-info for {dist_name}'
    pkg = site / normalized
    assert pkg.is_dir(), f'expected a package directory for {dist_name}'
    path = root / f'{normalized}-{version}-py3-none-any.whl'
    with ZipFile(path, 'w') as archive:
        for source in sorted(pkg.rglob('*')):
            if source.is_file():
                archive.write(source, source.relative_to(site).as_posix())
        for source in sorted(info_dirs[0].rglob('*')):
            if source.is_file():
                archive.write(source, source.relative_to(site).as_posix())
    return path


REAL_FILES = (
    'tools/install_offline.py', 'tools/offline_bundle.py',
    'tools/offline_manifest.py', 'tools/dev.py', 'tools/run_tests.sh',
    'tools/check_offline_sources.py',
    'packaging/install_full.sh', 'packaging/install_slim.sh',
    'tools/install_offline.sh',
)


def _copy_real(project: Path, names=REAL_FILES) -> None:
    for relative in names:
        put(project, relative, (REPO / relative).read_bytes())


PYPROJECT = (
    '[project]\nname="bike-sim"\nversion="0.1.0"\nrequires-python=">=3.13"\n'
    'dependencies={deps}\n'
    '{groups}'
    '[build-system]\nrequires=["setuptools>=1.0"]\nbuild-backend="setuptools.build_meta"\n')


def _project_tree(project: Path, *, native: bool, dependencies: list[str],
                  groups: dict[str, list[str]] | None = None) -> None:
    group_text = ''
    if groups:
        group_text = '[dependency-groups]\n' + ''.join(
            f'{name}={json.dumps(values)}\n' for name, values in groups.items())
    files = {
        'README.md': 'Synthetic source fixture.\n',
        'src/bike_sim/__init__.py': 'BIKE_SIM_FIXTURE = True\n',
        'pyproject.toml': PYPROJECT.format(deps=json.dumps(dependencies),
                                           groups=group_text),
        'uv.lock': 'placeholder, rewritten after wheels exist\n',
    }
    for name, content in files.items():
        put(project, name, content)
    _copy_real(project)
    if native:
        put(project, 'native/CMakeLists.txt',
            'add_library(core src/core.cpp)\ninclude(cmake/flags.cmake)\n')
        put(project, 'native/src/core.cpp', '// fixture\n')
        put(project, 'native/cmake/flags.cmake', '# fixture\n')
        put(project, 'native/tests/smoke.cpp', '// fixture\n')
        put(project, 'tools/proto_native_bench/bench.cpp', '// fixture\n')
        put(project, 'tools/proto_native_bench/state_format.hpp', '// fixture\n')
        base = 'tools/proto_native_bench/artifacts'
        put(project, f'{base}/current.json', json.dumps({'format': 1, 'bundle': 'selected'}))
        checksums = {}
        for filename, field in [('model.mjb', 'model_mjb_sha256'),
                                ('state.bin', 'state_bin_sha256'),
                                ('state.npz', 'state_npz_sha256')]:
            content = f'INERT SELECTED {filename}'.encode()
            put(project, f'{base}/selected/{filename}', content)
            checksums[field] = sha(content)
        put(project, f'{base}/selected/manifest.json', json.dumps({'checksums': checksums}))
        put(project, f'{base}/model.mjb', b'INDEPENDENT FLAT MODEL FIXTURE')


def _make_bundle(tmp_path: Path, target: dict, *, mode='full', native=False,
                 name='bundle') -> Path:
    """An internally consistent extracted schema-1 bundle for the host."""
    root = tmp_path / name
    project = root / 'project'
    wheelhouse = root / 'wheelhouse'
    wheelhouse.mkdir(parents=True)
    packaging = repack_installed_wheel(wheelhouse, 'packaging')
    packaging_version = md.version('packaging')

    app = [('mujoco', '3.12.0'), ('packaging', packaging_version), ('fakehelper', '1.0')]
    if native:
        app += [('numpy', '2.5.2'), ('scipy', '1.17.0'), ('nanobind', '1.0'),
                ('hypothesis', '1.0'), ('threadpoolctl', '1.0')]
        groups = {'native': ['mujoco==3.12.0', 'numpy==2.5.2', 'nanobind==1.0',
                             'threadpoolctl==1.0'],
                  'dev': ['hypothesis==1.0']}
    else:
        groups = None
    externals = [('fakeride', '1.0')] if mode == 'slim' else []
    deps = [f'{dist}=={version}' for dist, version in app + externals]
    _project_tree(project, native=native, dependencies=deps, groups=groups)

    created: dict[str, tuple] = {}
    for dist, version in app:
        if dist == 'packaging':
            continue
        init = f'__version__ = "{version}"\n' if dist == 'mujoco' else None
        created[dist] = (version, wheel(wheelhouse, dist, version, init=init))
    created['packaging'] = (packaging_version, packaging)
    for dist, version in externals:
        created[dist] = (version, wheel(tmp_path / 'external-wheels', dist, version,
                                      init=f'__version__ = "{version}"\n'))
    for dist in TOOLS:
        version = {'uv': UV_VERSION, 'setuptools': '82.0.1'}.get(dist, '1.0')
        created[dist] = (version, uv_executable_wheel(wheelhouse, version)
                         if dist == 'uv' else wheel(wheelhouse, dist, version))
    if not native:
        source_init = (project / 'src/bike_sim/__init__.py').read_bytes()
        created['bike-sim'] = ('0.1.0', wheel(
            wheelhouse, 'bike-sim', '0.1.0',
            init=source_init.decode()))

    environment = bundle.marker_environment(target)
    records = {}
    for dist, (version, path) in created.items():
        records[dist] = bundle.inspect_wheel(path, target)

    # Frozen requirements cover every app dependency (externals included).
    lock_lines, requirements = ['version = 1'], []
    for dist, version in app + externals:
        row = records[dist]['wheel']
        url = 'https://example.invalid/' + row['filename']
        wheel_entry = 'wheels=[{url=' + json.dumps(url) \
            + ',hash=' + json.dumps('sha256:' + row['sha256']) + '}]'
        lock_lines += ['[[package]]', f'name={json.dumps(dist)}',
                       f'version={json.dumps(version)}',
                       'source={registry="https://example.invalid/simple"}',
                       wheel_entry]
        requirements.append(f'{dist}=={version} --hash=sha256:{row["sha256"]}')
    put(project, 'uv.lock', '\n'.join(lock_lines) + '\n')
    requirements_path = put(root, 'requirements.txt',
                            '\n'.join(requirements) + '\n')

    rows = [{'path': path.relative_to(project).as_posix(),
             'sha256': sha(path.read_bytes())}
            for path in sorted(project.rglob('*')) if path.is_file()]
    bundled = [records[dist]['wheel'] for dist, _ in app if dist not in
               [name for name, _ in externals]] \
        + [records[dist]['wheel'] for dist in TOOLS]
    if not native:
        bundled.append(records['bike-sim']['wheel'])
    bundled.sort(key=lambda row: row['distribution'])
    manifest = dict(
        schema_version=1,
        project=dict(name='bike-sim', version='0.1.0', git_commit='a' * 40,
                     dirty=False,
                     python_source_sha256=source_digest(rows, 'src/'),
                     native_source_sha256=source_digest(rows, 'native/')),
        target=target, mode=mode,
        capabilities=dict(python=True, native_source_build=native,
                          development=native, gui=False),
        requirements=dict(lock_sha256=sha((project / 'uv.lock').read_bytes()),
                          requirements_sha256=sha(requirements_path.read_bytes()),
                          groups=(['native', 'dev'] if native else [])),
        wheels=bundled, source_files=rows,
        prerequisites=dict(
            python=f'CPython {target["python_minor"]}.*',
            compiler_feature_profile=('bike-cxx23-strict-local-probes-required'
                                      if native else None),
            cmake_minimum='3.27' if native else None,
            uv_version=UV_VERSION,
            system_libraries=['test system library']),
        slim_external=[dict(distribution=dist, version=version,
                            qualification_profile=('native-exact-provider-pending-T25'
                                                   if native else
                                                   'python-exact-import-pending-T25'))
                       for dist, version in externals])
    closure_records = [records[dist] for dist, _ in app + externals] \
        + [records[dist] for dist in TOOLS]
    if not native:
        closure_records.append(records['bike-sim'])
    roots = list(deps)
    if native:
        roots += groups['native'] + groups['dev']
    roots += [f'{name}=={created[name][0]}' for name in sorted(TOOLS)] \
        + ['setuptools>=1.0']
    if not native:
        roots.append('bike-sim==0.1.0')
    closure = dict(schema_version=1, target=target,
                   marker_environment=environment, roots=roots,
                   wheels=closure_records,
                   dependency_edges={dist: [] for dist in records})
    tools = dict(schema_version=1, target=target, uv_version=UV_VERSION,
                 wheels=[records[dist]['wheel'] for dist in TOOLS])
    write_json(root / 'manifest.json', manifest)
    write_json(root / 'offline-tools.json', tools)
    write_json(root / 'dependency-closure.json', closure)
    write_json(root / 'assembly-report.json', dict(
        schema_version=1, status='assembled-not-installed',
        installation_qualification='pending-T25', native_extension_built=False,
        tool_lock_sha256=sha((root / 'offline-tools.json').read_bytes()),
        dependency_closure_sha256=sha((root / 'dependency-closure.json').read_bytes()),
        source=dict(source_modes={
            row['path']: (project / row['path']).stat().st_mode & 0o777
            for row in rows})))
    put(root, 'README.md', 'Synthetic bundle; installation pending.\n')
    return root


@pytest.fixture
def full_bundle(tmp_path, target):
    return _make_bundle(tmp_path, target, mode='full', native=False, name='bundle a')


@pytest.fixture
def slim_bundle(tmp_path, target):
    return _make_bundle(tmp_path, target, mode='slim', native=False, name='bundle slim')


@pytest.fixture
def native_bundle(tmp_path, target):
    return _make_bundle(tmp_path, target, mode='full', native=True, name='bundle native')


def run_installer(project: Path, *argv, env=None):
    return subprocess.run(
        [sys.executable, str(project / 'tools' / 'install_offline.py'), *argv],
        capture_output=True, text=True, env=env)


def run_install_sh(project: Path, mode: str, *argv, env=None):
    script = project / 'packaging' / f'install_{mode}.sh'
    return subprocess.run(['bash', str(script), *argv],
                          capture_output=True, text=True, env=env)


def provider_injection(base: Path, name: str, version: str) -> dict:
    """A fake 'preinstalled' provider for the *base* interpreter: importable,
    version-visible, and ordered after venv site-packages (addsitedir)."""
    provider = base / 'provider'
    provider.mkdir(parents=True)
    module = name.replace('-', '_')
    (provider / module).mkdir()
    (provider / module / '__init__.py').write_text(
        f'__version__ = "{version}"\n')
    info = provider / f'{module}-{version}.dist-info'
    info.mkdir()
    (info / 'METADATA').write_text(
        f'Metadata-Version: 2.1\nName: {name}\nVersion: {version}\n')
    (info / 'RECORD').write_text('')
    (base / 'sitecustomize.py').write_text(
        'import pathlib, site\n'
        'site.addsitedir(str(pathlib.Path(__file__).resolve().parent / "provider"))\n')
    return {'PYTHONPATH': str(base)}


# ---------------------------------------------------------------------------
# Plan-mandated control: schema rejection stays in the manifest validator.
# ---------------------------------------------------------------------------

def test_wrong_manifest_schema_is_not_legacy_native():
    with pytest.raises(ValueError, match="schema"):
        validate_bundle_manifest({
            "schema_version": 2,
            "capabilities": {"native_source_build": True},
        })


def test_fixture_bundle_passes_full_check(full_bundle):
    manifest = json.loads((full_bundle / 'manifest.json').read_text())
    assert bundle.check_bundle(full_bundle, require_host=True) == manifest


# ---------------------------------------------------------------------------
# Interpreter / uv / venv selection units.
# ---------------------------------------------------------------------------

def test_select_python_returns_matching_base(target):
    from tools import install_offline as install
    selected = install._select_python(target, None)
    assert Path(selected).is_file()


def test_select_python_explicit_mismatch(target):
    from tools import install_offline as install
    with pytest.raises(ValueError, match='does not satisfy'):
        install._select_python(target, '/bin/false')


def test_select_python_unsatisfiable_minor(target):
    from tools import install_offline as install
    target['python_minor'] = '3.99'
    with pytest.raises(ValueError, match='3\\.99'):
        install._select_python(target, sys.executable)
    with pytest.raises(ValueError, match='3\\.99'):
        install._select_python(target, None)


def test_select_uv_prefers_matching_system(target, tmp_path):
    from tools import install_offline as install
    manifest = {'prerequisites': {'uv_version': UV_VERSION}, 'wheels': []}
    selected = install._select_uv(manifest, tmp_path, tmp_path / 'tools')
    assert selected == Path(shutil.which('uv'))


def test_select_uv_extracts_verified_bundled_wheel(target, tmp_path, monkeypatch):
    from tools import install_offline as install
    wheelhouse = tmp_path / 'wheelhouse'
    wheelhouse.mkdir()
    wheel = uv_executable_wheel(wheelhouse, '9.9.9')
    manifest = {'prerequisites': {'uv_version': '9.9.9'},
                'wheels': [bundle.inspect_wheel(wheel, target)['wheel']]}
    monkeypatch.setattr(install.shutil, 'which', lambda name: None)
    selected = install._select_uv(manifest, wheelhouse, tmp_path / 'tools')
    assert install._uv_version(selected) == '9.9.9'


def test_select_uv_rejects_wrong_bundled_version(target, tmp_path, monkeypatch):
    from tools import install_offline as install
    wheelhouse = tmp_path / 'wheelhouse'
    wheelhouse.mkdir()
    wheel = uv_executable_wheel(wheelhouse, '9.9.9')
    manifest = {'prerequisites': {'uv_version': '8.8.8'},
                'wheels': [bundle.inspect_wheel(wheel, target)['wheel']]}
    monkeypatch.setattr(install.shutil, 'which', lambda name: None)
    with pytest.raises(ValueError, match='8\\.8\\.8'):
        install._select_uv(manifest, wheelhouse, tmp_path / 'tools')


def test_select_venv_rejects_non_venv_and_payload_dirs(full_bundle, tmp_path):
    from tools import install_offline as install
    project = full_bundle / 'project'
    occupied = tmp_path / 'occupied'
    occupied.mkdir()
    (occupied / 'file.txt').write_text('x')
    with pytest.raises(ValueError, match='non-venv'):
        install._select_venv(occupied, full_bundle, project)
    with pytest.raises(ValueError, match='payloads'):
        install._select_venv(project / 'nested', full_bundle, project)
    with pytest.raises(ValueError, match='payloads'):
        install._select_venv(full_bundle / 'wheelhouse' / 'inner', full_bundle, project)
    default = install._select_venv(None, full_bundle, project)
    assert default == (full_bundle / '.venv').resolve()


def test_activation_wrapper_exports_exact_environment(tmp_path):
    from tools import install_offline as install
    venv = tmp_path / 'venv dir with spaces'
    (venv / 'bin').mkdir(parents=True)
    wrapper = install._write_activation(venv, tmp_path / 'native' / 'build' / 'release')
    result = subprocess.run(
        ['bash', '-c', f'source "{wrapper}" && env'],
        capture_output=True, text=True)
    assert result.returncode == 0
    env = dict(line.split('=', 1) for line in result.stdout.splitlines()
               if '=' in line)
    assert env['VIRTUAL_ENV'] == str(venv)
    assert env['UV_PROJECT_ENVIRONMENT'] == str(venv)
    assert env['BIKE_NATIVE_BUILD_PATH'] == str(tmp_path / 'native/build/release')
    assert env['PATH'].split(os.pathsep)[0] == str(venv / 'bin')


# ---------------------------------------------------------------------------
# Slim external provider contract: exact version + unchanged import path.
# ---------------------------------------------------------------------------

def test_slim_provider_requires_exact_version(tmp_path, monkeypatch):
    from tools import install_offline as install
    manifest = {'slim_external': [
        {'distribution': 'fakeride', 'version': '1.0',
         'qualification_profile': 'python-exact-import-pending-T25'}]}
    provider_injection(tmp_path / 'wrong', 'fakeride', '1.1')
    monkeypatch.setenv('PYTHONPATH', str(tmp_path / 'wrong'))
    with pytest.raises(ValueError, match=r'fakeride.*exactly 1\.0.*1\.1'):
        install._verify_slim_externals(manifest, Path(sys.executable))
    provider_injection(tmp_path / 'right', 'fakeride', '1.0')
    monkeypatch.setenv('PYTHONPATH', str(tmp_path / 'right'))
    origins = install._verify_slim_externals(manifest, Path(sys.executable))
    assert origins['fakeride'].name == '__init__.py'


def test_slim_provider_postcheck_detects_shadowing(tmp_path, target):
    """A venv-local fakeride must shadow the validated provider visibly."""
    from tools import install_offline as install
    manifest = {'slim_external': [
        {'distribution': 'fakeride', 'version': '1.0',
         'qualification_profile': 'python-exact-import-pending-T25'}]}
    provider_injection(tmp_path / 'inj', 'fakeride', '1.0')
    venv = tmp_path / 'venv'
    subprocess.run([sys.executable, '-m', 'venv', '--without-pip', str(venv)],
                   check=True)
    # A conflicting same-version fakeride inside the venv's own site-packages.
    site = Path(subprocess.check_output(
        [str(venv / 'bin/python'), '-c',
         'import sysconfig; print(sysconfig.get_path("purelib"))'],
        text=True).strip())
    (site / 'fakeride').mkdir()
    (site / 'fakeride' / '__init__.py').write_text('__version__="1.0"\n')
    info = site / 'fakeride-1.0.dist-info'
    info.mkdir()
    (info / 'METADATA').write_text(
        'Metadata-Version: 2.1\nName: fakeride\nVersion: 1.0\n')
    origins = {'fakeride': (tmp_path / 'inj' / 'provider' / 'fakeride'
                           / '__init__.py').resolve()}
    env = {**os.environ, 'PYTHONPATH': str(tmp_path / 'inj')}
    with pytest.raises(ValueError, match='shadow'):
        install._confirm_slim_externals(manifest, venv / 'bin' / 'python',
                                        origins, env)


# ---------------------------------------------------------------------------
# Preflight ordering: every named failure precedes environment mutation.
# ---------------------------------------------------------------------------

def _assert_no_environment(venv: Path):
    assert not (venv / 'pyvenv.cfg').exists(), \
        f'preflight failure still created an environment at {venv}'


def test_preflight_wrong_system_fails_before_mutation(full_bundle, tmp_path):
    manifest = json.loads((full_bundle / 'manifest.json').read_text())
    manifest['target']['system'] = 'Linux' if platform.system() != 'Linux' else 'Darwin'
    manifest['target']['machine'] = 'x86_64' if manifest['target']['system'] == 'Linux' else 'arm64'
    write_json(full_bundle / 'manifest.json', manifest)
    venv = tmp_path / 'venv target'
    result = run_installer(full_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert 'OFFLINE INSTALL ERROR' in result.stderr
    _assert_no_environment(venv)


def test_preflight_wrong_python_minor_fails_before_mutation(full_bundle, tmp_path):
    manifest = json.loads((full_bundle / 'manifest.json').read_text())
    manifest['target']['python_minor'] = '3.99'
    write_json(full_bundle / 'manifest.json', manifest)
    venv = tmp_path / 'venv target'
    result = run_installer(full_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert '3.99' in result.stderr
    _assert_no_environment(venv)


def test_preflight_corrupted_wheel_fails_before_mutation(full_bundle, tmp_path):
    wheel_path = next((full_bundle / 'wheelhouse').glob('mujoco-*.whl'))
    wheel_path.write_bytes(wheel_path.read_bytes() + b'corrupted')
    venv = tmp_path / 'venv target'
    result = run_installer(full_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert 'OFFLINE INSTALL ERROR' in result.stderr
    _assert_no_environment(venv)


def test_preflight_missing_native_source_fails(native_bundle, tmp_path):
    (native_bundle / 'project' / 'native' / 'CMakeLists.txt').unlink()
    venv = tmp_path / 'venv target'
    result = run_installer(native_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert 'source' in result.stderr or 'OFFLINE INSTALL ERROR' in result.stderr
    _assert_no_environment(venv)


def test_preflight_mismatched_lock_fails(full_bundle, tmp_path):
    lock = full_bundle / 'project' / 'uv.lock'
    lock.write_text(lock.read_text() + '\n# drifted\n')
    venv = tmp_path / 'venv target'
    result = run_installer(full_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert 'OFFLINE INSTALL ERROR' in result.stderr
    _assert_no_environment(venv)


def test_preflight_missing_compiler_fails(native_bundle, tmp_path, monkeypatch):
    venv = tmp_path / 'venv target'
    env = dict(os.environ)
    env['PATH'] = '/nonexistent-path-only'
    result = run_installer(native_bundle / 'project', str(venv), env=env)
    assert result.returncode == 1
    assert 'cmake' in result.stderr
    _assert_no_environment(venv)


def test_mode_mismatch_fails_before_mutation(full_bundle, tmp_path):
    venv = tmp_path / 'venv target'
    result = run_installer(full_bundle / 'project', str(venv), '--mode', 'slim')
    assert result.returncode == 1
    assert 'mode' in result.stderr
    _assert_no_environment(venv)


def test_legacy_bundle_layout_is_not_schema1(tmp_path):
    root = tmp_path / 'legacy'
    project = root / 'project'
    project.mkdir(parents=True)
    _copy_real(project, ('tools/install_offline.py',))
    put(root, 'requirements.txt', 'demo==1.0\n')
    (root / 'wheelhouse').mkdir()
    result = run_installer(project, str(tmp_path / 'venv'))
    assert result.returncode == 1
    # On this host the documented legacy gate refuses before any venv work.
    assert 'Linux x86_64' in result.stderr or 'CPython 3.13' in result.stderr
    assert not (tmp_path / 'venv').exists()


def test_missing_bundle_layout_fails_cleanly(tmp_path):
    project = tmp_path / 'orphan' / 'project'
    project.mkdir(parents=True)
    _copy_real(project, ('tools/install_offline.py',))
    result = run_installer(project, str(tmp_path / 'venv'))
    assert result.returncode == 1
    assert 'offline bundle' in result.stderr


# ---------------------------------------------------------------------------
# Real end-to-end installs on this host (python-only capability only).
# ---------------------------------------------------------------------------

def test_full_python_only_install_end_to_end(full_bundle, tmp_path):
    venv = tmp_path / 'venv dir with space'
    result = run_install_sh(full_bundle / 'project', 'full', str(venv))
    assert result.returncode == 0, result.stderr
    report = json.loads((venv / 'bikesim-install-report.json').read_text())
    assert report['mode'] == 'full'
    assert report['native_source_build'] is False
    assert report['network_used'] is False
    assert report['verify']['passed'] is True
    wrapper = venv / 'bin' / 'activate-bikesim'
    assert wrapper.is_file()
    probe_result = subprocess.run(
        [str(venv / 'bin' / 'python'), '-c',
         'import bike_sim, mujoco, packaging; import sys; '
         'print(bike_sim.BIKE_SIM_FIXTURE, mujoco.__version__, packaging.__name__)'],
        capture_output=True, text=True)
    assert probe_result.returncode == 0, probe_result.stderr
    assert 'True' in probe_result.stdout
    # The venv's own uv entry point exists after install.
    assert (venv / 'bin' / 'uv').is_file()


def test_slim_python_only_install_end_to_end(slim_bundle, tmp_path):
    injection = provider_injection(tmp_path / 'providers', 'fakeride', '1.0')
    env = {**os.environ, **injection}
    venv = tmp_path / 'slim venv dir'
    result = run_install_sh(slim_bundle / 'project', 'slim', str(venv), env=env)
    assert result.returncode == 0, result.stderr
    report = json.loads((venv / 'bikesim-install-report.json').read_text())
    assert report['mode'] == 'slim'
    providers = report['slim_external']
    assert providers['fakeride'].startswith(str(tmp_path / 'providers'))
    # The venv must not have installed its own copy of the provider.
    site = subprocess.check_output(
        [str(venv / 'bin' / 'python'), '-c',
         'import sysconfig; print(sysconfig.get_path("purelib"))'],
        text=True).strip()
    assert not (Path(site) / 'fakeride').exists()


def test_slim_wrong_provider_version_is_rejected(slim_bundle, tmp_path):
    injection = provider_injection(tmp_path / 'providers', 'fakeride', '1.17.1')
    env = {**os.environ, **injection}
    venv = tmp_path / 'slim venv dir'
    result = run_install_sh(slim_bundle / 'project', 'slim', str(venv), env=env)
    assert result.returncode == 1
    assert 'exactly 1.0' in result.stderr
    _assert_no_environment(venv)


def test_native_fixture_fails_without_success_claim(native_bundle, tmp_path):
    """The fixture's inert wheels cannot perform a real native build: the
    install must fail and never print a success banner."""
    venv = tmp_path / 'native venv'
    result = run_installer(native_bundle / 'project', str(venv))
    assert result.returncode == 1
    assert 'Installed bike-sim' not in result.stdout
    assert not (venv / 'bin' / 'activate-bikesim').exists()
    assert not (venv / 'bikesim-install-report.json').exists()


# ---------------------------------------------------------------------------
# Probe selection: exactly one matching manifest; archives + dirs counted.
# ---------------------------------------------------------------------------

def test_probe_selects_exactly_one_matching_bundle(full_bundle, tmp_path):
    container = tmp_path / 'bundles'
    container.mkdir()
    shutil.copytree(full_bundle, container / 'one')
    kind, path, manifest = probe.select_bundle(container, 'full')
    assert kind == 'dir'
    assert manifest['mode'] == 'full'


def test_probe_rejects_zero_and_ambiguous_matches(full_bundle, slim_bundle, tmp_path):
    container = tmp_path / 'bundles'
    container.mkdir()
    shutil.copytree(full_bundle, container / 'one')
    shutil.copytree(full_bundle, container / 'two')
    with pytest.raises(ValueError, match='ambiguous'):
        probe.select_bundle(container, 'full')
    with pytest.raises(ValueError, match='no bundle manifest'):
        probe.select_bundle(container, 'slim')
    shutil.rmtree(container / 'one')
    shutil.copytree(slim_bundle, container / 'slim one')
    kind, path, manifest = probe.select_bundle(container, 'slim')
    assert manifest['mode'] == 'slim'


def test_probe_discovers_archive_members(slim_bundle, tmp_path):
    container = tmp_path / 'bundles'
    container.mkdir()
    archive_path = container / 'slim-bundle.tar.gz'
    with tarfile.open(archive_path, 'w:gz') as tar:
        for path in sorted(slim_bundle.rglob('*')):
            if path.is_file():
                tar.add(path, arcname='slim-bundle/' + path.relative_to(slim_bundle).as_posix())
    kind, path, manifest = probe.select_bundle(container, 'slim')
    assert kind == 'tar'
    assert manifest['slim_external'][0]['distribution'] == 'fakeride'
