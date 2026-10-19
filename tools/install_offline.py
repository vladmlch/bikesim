#!/usr/bin/env python3
"""Install an extracted bike-sim offline bundle without any network access.

Two bundle layouts are supported:

* Schema-1 bundles assembled by ``tools/offline_bundle.py``.  The extracted
  bundle root contains ``manifest.json``, ``project/``, ``wheelhouse/``,
  ``requirements.txt``, ``offline-tools.json``, ``dependency-closure.json``,
  ``assembly-report.json`` and ``README.md``.  Every payload is hash- and
  metadata-validated before any environment is mutated.
* Legacy manifest-less bundles (``requirements.txt`` + ``wheelhouse/`` next to
  ``project/``).  These keep the documented Python-only slim installation and
  are never presented as native-capable.

Schema-1 flow — every step resolves from validated local paths:

1. Read ``manifest.json`` and select an interpreter matching the declared
   target exactly (system, machine, CPython minor) from ``--python``, the
   running process or ``PATH``.
2. Run the complete bundle preflight (manifest schema, source inventory,
   wheel metadata/hashes, requirements/lock hashes, dependency and tool
   closures, wheel-tag compatibility) under the selected interpreter —
   all before any environment mutation.
3. Select ``uv``: a preinstalled uv exactly matching ``uv_version``, else the
   hash-verified bundled uv wheel extracted to a transient directory.
4. Native bundles additionally require preinstalled ``cmake`` and a C++
   compiler (the feature profile is probed by the local CMake configure).
5. Slim bundles verify every ``slim_external`` provider — exact version and
   importable origin — on the selected base interpreter, before creating the
   ``--system-site-packages`` environment.
6. Create the venv only at the explicit/default path; an existing non-venv
   directory is never deleted or repurposed.
7. Install the frozen hashed closure:

   * full → ``uv pip sync --require-hashes -r requirements.txt`` then the
     project itself (editable install for native bundles, the bundled pure
     Python wheel otherwise);
   * slim → ``uv pip install --no-deps`` of exactly the bundled wheels
     (externals are never installed into the venv) plus the editable/native
     or wheel project payload.

   ``uv sync`` cannot consume a flat wheelhouse — locked wheels pin their
   registry URLs — so the frozen export plus ``uv pip`` is the faithful
   offline equivalent.  PEP 517 build isolation resolves the backend
   (setuptools) from the same wheelhouse and fails closed when absent.
8. Slim bundles re-verify provider versions and import origins inside the new
   environment; a shadowed provider is a rejection, never a silent swap.
9. Native bundles configure/build/doctor through ``tools/dev.py`` under
   ``uv run --frozen --group native --no-sync --offline`` with
   ``UV_PROJECT_ENVIRONMENT`` pinned to the new venv.
10. Emit an activation wrapper exporting the exact environment and selected
    build path, a JSON report, and a success banner — only after every step
    passed.  A native build failure leaves diagnostics and prints no banner.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT.parent


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _requirement_cls():
    """packaging is a bundled dep; legacy/verify paths may still need the wheel."""
    try:
        from packaging.requirements import Requirement
    except ImportError:
        candidates = sorted((BUNDLE / 'wheelhouse').glob('packaging-*.whl'))
        if candidates:
            sys.path.insert(0, str(candidates[0]))
            from packaging.requirements import Requirement
        else:
            raise
    return Requirement


# ---------------------------------------------------------------------------
# Interpreter selection — the declared target is matched exactly, never
# coerced.  The selected interpreter's real base executable is what the venv
# and (for slim) the external providers bind to.
# ---------------------------------------------------------------------------

_PY_PROBE = (
    "import json, platform, sys\n"
    "print(json.dumps({"
    "'executable': sys.executable,"
    "'base': getattr(sys, '_base_executable', None) or sys.executable,"
    "'version': platform.python_version(),"
    "'minor': f'{sys.version_info.major}.{sys.version_info.minor}',"
    "'implementation': platform.python_implementation(),"
    "'system': platform.system(),"
    "'machine': platform.machine(),"
    "}))"
)


def _normal_machine(machine: str, system: str) -> str:
    aliases = {'AMD64': 'x86_64', 'arm64': 'arm64' if system == 'Darwin' else 'aarch64'}
    return aliases.get(machine, machine)


def _clean_generated_sources(project: Path) -> list[str]:
    """Delete install-generated artifact directories so re-installation is
    idempotent: a prior PEP 517 editable build leaves ``*.egg-info`` under
    ``src/`` and interpreter imports leave ``__pycache__``.  Neither is ever
    declared bundle payload — the assembler never ships them — so removing
    them restores the pre-install state the manifest inventory describes.
    """
    removed = []
    for directory in sorted(
            path for path in project.rglob('*')
            if path.is_dir()
            and (path.name == '__pycache__' or path.name.endswith('.egg-info'))):
        shutil.rmtree(directory)
        removed.append(directory.relative_to(project).as_posix())
    return removed


def _probe_python(executable) -> dict | None:
    try:
        result = subprocess.run(
            [str(executable), '-P', '-c', _PY_PROBE],
            capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    try:
        probe = json.loads(result.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return None
    return probe if isinstance(probe, dict) else None


def _probe_matches(probe: dict, target: dict) -> bool:
    return (
        probe.get('system') == target['system']
        and _normal_machine(probe.get('machine', ''), probe.get('system', '')) == target['machine']
        and probe.get('minor') == target['python_minor']
        and probe.get('implementation', '').lower() == target['implementation'].lower()
    )


def _select_python(target: dict, explicit: str | None) -> Path:
    """One interpreter satisfying the manifest target; error names the need."""
    minor = target['python_minor']
    need = f"CPython {minor}.* for {target['system']}/{target['machine']}"
    if explicit is not None:
        probe = _probe_python(explicit)
        if probe is None or not _probe_matches(probe, target):
            found = 'unusable' if probe is None else (
                f"{probe.get('implementation')} {probe.get('version')} "
                f"on {probe.get('system')}/{probe.get('machine')}")
            raise ValueError(
                f"--python {explicit} does not satisfy the bundle target: "
                f"requires {need}; found {found}")
        return Path(probe['base'])
    candidates = [sys.executable, shutil.which(f'python{minor}'),
                  shutil.which('python3'), shutil.which('python')]
    seen: set[str] = set()
    for candidate in candidates:
        if candidate is None:
            continue
        key = str(Path(candidate).resolve())
        if key in seen:
            continue
        seen.add(key)
        probe = _probe_python(candidate)
        if probe is not None and _probe_matches(probe, target):
            return Path(probe['base'])
    raise ValueError(
        f"the bundle requires {need}; none of the candidate interpreters "
        f"({', '.join(c for c in candidates if c) or 'none'}) satisfies it. "
        "Provision the interpreter declared by the bundle prerequisites "
        "instead of letting uv download one")


# ---------------------------------------------------------------------------
# Bundle validation — runs under the *selected* interpreter in a child so the
# installer front-end itself stays stdlib-only.  ``check_bundle`` performs the
# manifest schema check, target/host match, wheel-tag compatibility, source
# inventory hashes/modes, requirements+lock hashes, dependency and tool
# closures and wheelhouse inventory.
# ---------------------------------------------------------------------------

_VALIDATE_CHILD = (
    "import sys\n"
    "try:\n"
    "    import packaging  # noqa: F401\n"
    "except ImportError:\n"
    "    if sys.argv[3]:\n"
    "        sys.path.insert(0, sys.argv[3])\n"
    "        import packaging  # noqa: F401\n"
    "sys.path.insert(0, sys.argv[2])\n"
    "import json\n"
    "from pathlib import Path\n"
    "from tools.offline_bundle import check_bundle\n"
    "result = check_bundle(Path(sys.argv[1]), require_host=True)\n"
    "sys.stdout.write(json.dumps(result, sort_keys=True))\n"
)


def _packaging_wheel(manifest: dict, wheelhouse: Path) -> Path | None:
    candidates = sorted(
        path for path in wheelhouse.glob('packaging-*.whl') if path.is_file())
    if not candidates:
        return None
    wheel = candidates[0]
    rows = manifest.get('wheels') if isinstance(manifest, dict) else None
    row = next((row for row in rows
                if isinstance(row, dict) and row.get('distribution') == 'packaging'),
               None) if isinstance(rows, list) else None
    if (row is not None and isinstance(row.get('sha256'), str)
            and _hash_file(wheel) != row['sha256']):
        raise ValueError(f'packaging wheel hash mismatch: {wheel.name}')
    return wheel


def _validate_bundle(python: Path, bundle: Path, project: Path) -> dict:
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('UV_', 'PIP_'))}
    # Importing tools.offline_bundle must not write __pycache__ into the
    # payload — the inventory check counts undeclared files.
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    wheelhouse = bundle / 'wheelhouse'
    raw = json.loads((bundle / 'manifest.json').read_text(encoding='utf-8'))
    wheel = _packaging_wheel(raw, wheelhouse) if wheelhouse.is_dir() else None
    child = subprocess.run(
        [str(python), '-P', '-c', _VALIDATE_CHILD, str(bundle), str(project),
         str(wheel) if wheel else ''],
        capture_output=True, text=True, env=env)
    if child.returncode != 0:
        detail = (child.stderr.strip().splitlines() or ['unknown failure'])[-1]
        raise ValueError(f'bundle preflight failed: {detail}')
    try:
        return json.loads(child.stdout)
    except json.JSONDecodeError as error:
        raise ValueError(f'bundle preflight returned no manifest: {error}') from error


# ---------------------------------------------------------------------------
# uv selection — the manifest's exact uv version from PATH, else the
# hash-verified bundled platform wheel (extracted, never executed from the
# wheelhouse directly).
# ---------------------------------------------------------------------------

def _uv_version(executable) -> str | None:
    try:
        result = subprocess.run([str(executable), '--version'],
                                capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    match = re.search(r'uv\s+([0-9][\w.\-+]*)', result.stdout.strip())
    return match.group(1) if match else None


def _extract_uv(wheel_path: Path, tools_dir: Path) -> Path:
    tools_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(wheel_path) as archive:
        members = [name for name in archive.namelist()
                   if PurePosixPath(name).name == 'uv' and not name.endswith('/')]
        if not members:
            raise ValueError(f'bundled uv wheel has no uv executable: {wheel_path.name}')
        members.sort(key=lambda name: (
            'scripts' not in name and 'bin' not in name, name))
        target = tools_dir / 'uv'
        target.write_bytes(archive.read(members[0]))
    target.chmod(target.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return target


def _select_uv(manifest: dict, wheelhouse: Path, tools_dir: Path) -> Path:
    required = manifest['prerequisites']['uv_version']
    system = shutil.which('uv')
    if system is not None and _uv_version(system) == required:
        return Path(system)
    row = next((row for row in manifest['wheels']
                if row['distribution'] == 'uv'), None)
    if row is None:
        raise ValueError(
            f'manifest requires uv=={required} but bundles no uv wheel; '
            'install that exact uv version')
    wheel = wheelhouse / row['filename']
    if not wheel.is_file() or _hash_file(wheel) != row['sha256']:
        raise ValueError(f'bundled uv wheel missing or hash mismatch: {row["filename"]}')
    extracted = _extract_uv(wheel, tools_dir)
    actual = _uv_version(extracted)
    if actual != required:
        raise ValueError(
            f'bundled uv executable reports {actual!r}; the manifest requires '
            f'uv=={required}')
    return extracted


# ---------------------------------------------------------------------------
# Slim external providers — exact version plus a real import origin on the
# base interpreter (before) and inside the created venv (after).  The path
# comparison detects venv/bundled shadowing of the validated system package.
# ---------------------------------------------------------------------------

_EXTERNAL_PROBE = (
    "import importlib.metadata as md, importlib.util, json, sys\n"
    "names = json.loads(sys.argv[1])\n"
    "result = {}\n"
    "for name in names:\n"
    "    entry = {'version': None, 'origin': None}\n"
    "    try:\n"
    "        entry['version'] = md.version(name)\n"
    "        modules = []\n"
    "        top = md.distribution(name).read_text('top_level.txt')\n"
    "        if top:\n"
    "            modules = [line.strip() for line in top.splitlines() if line.strip()]\n"
    "        modules.append(name.replace('-', '_'))\n"
    "    except md.PackageNotFoundError:\n"
    "        modules = [name.replace('-', '_')]\n"
    "    for module in modules:\n"
    "        try:\n"
    "            spec = importlib.util.find_spec(module)\n"
    "        except (ImportError, ValueError):\n"
    "            spec = None\n"
    "        if spec is not None and spec.origin:\n"
    "            entry['origin'] = spec.origin\n"
    "            break\n"
    "    result[name] = entry\n"
    "print(json.dumps(result))\n"
)


def _probe_externals(python: Path, names, env: dict | None = None) -> dict:
    child_env = env if env is not None else dict(os.environ)
    # Probing must not write __pycache__ into the provider's site-packages.
    child_env['PYTHONDONTWRITEBYTECODE'] = '1'
    child = subprocess.run(
        [str(python), '-P', '-c', _EXTERNAL_PROBE, json.dumps(sorted(names))],
        capture_output=True, text=True, env=child_env)
    if child.returncode != 0:
        detail = (child.stderr.strip().splitlines() or ['unknown failure'])[-1]
        raise ValueError(f'external provider probe failed: {detail}')
    return json.loads(child.stdout)


def _verify_slim_externals(manifest: dict, base_python: Path) -> dict[str, Path]:
    """Exact-version + importable-provider precheck on the base interpreter."""
    required = {row['distribution']: row['version']
                for row in manifest['slim_external']}
    probed = _probe_externals(base_python, required.keys())
    problems, origins = [], {}
    for name, version in required.items():
        entry = probed.get(name) or {}
        if entry.get('version') != version:
            problems.append(
                f"{name}: the bundle requires exactly {version}; found "
                f"{entry.get('version') or 'not installed'}")
            continue
        if not entry.get('origin'):
            problems.append(
                f'{name}=={version} is installed but not importable')
            continue
        origins[name] = Path(entry['origin']).resolve()
    if problems:
        raise ValueError(
            'slim bundle requires the exact declared external providers: '
            + '; '.join(problems)
            + '. Provision them on the selected interpreter, or use the '
              'full bundle which carries them')
    return origins


def _confirm_slim_externals(manifest: dict, venv_python: Path,
                            origins: dict[str, Path], env: dict) -> dict[str, str]:
    """Post-install check: same version, same resolved provider path."""
    required = {row['distribution']: row['version']
                for row in manifest['slim_external']}
    probed = _probe_externals(venv_python, origins.keys(), env=env)
    problems = []
    for name, expected_path in origins.items():
        entry = probed.get(name) or {}
        if entry.get('version') != required[name]:
            problems.append(
                f"{name}: the new environment reports {entry.get('version')}, "
                f"required {required[name]}")
            continue
        actual = (Path(entry['origin']).resolve()
                  if entry.get('origin') else None)
        if actual != expected_path:
            problems.append(
                f'{name}: provider would be shadowed — validated provider '
                f'{expected_path} but the environment resolves {actual}')
    if problems:
        raise ValueError(
            'external provider verification failed inside the created '
            'environment: ' + '; '.join(problems)
            + '. The slim environment cannot claim the validated providers; '
              'use the full bundle')
    return {name: str(path) for name, path in origins.items()}


# ---------------------------------------------------------------------------
# Environment target + child process environment.  All uv/pip subprocesses
# run with scrubbed ambient package-manager config, an installer-owned cache
# (no warm-cache reliance) and an explicit PATH for the selected tools.
# ---------------------------------------------------------------------------

def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _select_venv(arg, bundle: Path, project: Path) -> Path:
    venv = (Path(arg).expanduser() if arg else bundle / '.venv').resolve()
    wheelhouse = bundle / 'wheelhouse'
    if venv in (bundle, project) or _is_within(venv, project) or _is_within(venv, wheelhouse):
        raise ValueError(
            f'the venv target must live beside the bundle payloads, not inside '
            f'them: {venv}')
    if venv.exists():
        if not (venv / 'pyvenv.cfg').is_file():
            if any(venv.iterdir()):
                raise ValueError(
                    f'refusing to reuse an existing non-venv directory: {venv}')
        elif not (venv / 'bin' / 'python').is_file():
            raise ValueError(
                f'existing venv has no python executable: {venv}')
    return venv


def _venv_python(venv: Path) -> Path:
    return venv / 'bin' / 'python'


def _install_env(venv: Path, tools_dir: Path | None) -> dict:
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('UV_', 'PIP_', 'VIRTUAL_ENV'))}
    path_parts = [str(venv / 'bin')]
    if tools_dir is not None:
        path_parts.append(str(tools_dir))
    path_parts.append(os.environ.get('PATH', os.defpath))
    env.update({
        'UV_OFFLINE': '1',
        'UV_NO_CONFIG': '1',
        'UV_PYTHON_DOWNLOADS': 'never',
        'UV_PROJECT_ENVIRONMENT': str(venv),
        'UV_CACHE_DIR': str((tools_dir or venv) / 'uv-cache'),
        'PATH': os.pathsep.join(path_parts),
        # Keep imports of the editable source from writing __pycache__ into
        # the bundle payload — it must stay byte-identical for re-checks.
        'PYTHONDONTWRITEBYTECODE': '1',
    })
    return env


def _run(argv: list, *, cwd: Path, env: dict) -> None:
    result = subprocess.run([str(item) for item in argv], cwd=str(cwd), env=env)
    if result.returncode != 0:
        raise ValueError(f'command failed (exit {result.returncode}): '
                         + ' '.join(str(a) for a in argv))


def _run_capture(argv: list, *, cwd: Path, env: dict) -> str:
    result = subprocess.run([str(item) for item in argv], cwd=str(cwd), env=env,
                            capture_output=True, text=True)
    if result.returncode != 0:
        detail = (result.stderr.strip().splitlines()
                  or result.stdout.strip().splitlines() or ['unknown failure'])[-1]
        raise ValueError(f'command failed (exit {result.returncode}): {detail}\n'
                         + ' '.join(str(a) for a in argv))
    return result.stdout


def _bundled_wheel_files(manifest: dict, wheelhouse: Path) -> list[Path]:
    files = []
    for row in manifest['wheels']:
        path = wheelhouse / row['filename']
        if not path.is_file() or _hash_file(path) != row['sha256']:
            raise ValueError(f'wheelhouse wheel missing or hash mismatch: '
                             f'{row["filename"]}')
        files.append(path)
    return files


def _install_payload(manifest: dict, uv: Path, venv: Path, bundle: Path,
                     project: Path, env: dict) -> None:
    wheelhouse = bundle / 'wheelhouse'
    python = _venv_python(venv)
    native = manifest['capabilities']['native_source_build']
    if manifest['mode'] == 'full':
        # The manifest-pinned, hash-annotated frozen export is the contract:
        # `uv sync` cannot consume a flat wheelhouse (locked wheels pin their
        # registry URLs), while `uv pip sync` verifies every hash and installs
        # exclusively from --find-links.
        _run([uv, 'pip', 'sync', '--python', python, '--offline', '--no-index',
              '--require-hashes', '--find-links', wheelhouse,
              bundle / 'requirements.txt'], cwd=project, env=env)
        if native:
            _run([uv, 'pip', 'install', '--python', python, '--offline',
                  '--no-index', '--no-deps', '--find-links', wheelhouse,
                  '--editable', project], cwd=project, env=env)
        else:
            bike = wheelhouse / next(row['filename'] for row in manifest['wheels']
                                     if row['distribution'] == 'bike-sim')
            _run([uv, 'pip', 'install', '--python', python, '--offline',
                  '--no-index', '--no-deps', '--find-links', wheelhouse, bike],
                 cwd=project, env=env)
    else:
        # Slim installs exactly the bundled wheels — externals are provided by
        # the system interpreter and must never be installed into the venv.
        _run([uv, 'pip', 'install', '--python', python, '--offline',
              '--no-index', '--no-deps', '--find-links', wheelhouse]
             + _bundled_wheel_files(manifest, wheelhouse), cwd=project, env=env)
        if native:
            _run([uv, 'pip', 'install', '--python', python, '--offline',
                  '--no-index', '--no-deps', '--find-links', wheelhouse,
                  '--editable', project], cwd=project, env=env)
    # Persist the verified uv inside the venv: dev.py's own `uv run` children
    # and the activation wrapper resolve it from the venv before PATH.
    uv_row = next(row for row in manifest['wheels'] if row['distribution'] == 'uv')
    _run([uv, 'pip', 'install', '--python', python, '--offline', '--no-index',
          '--no-deps', wheelhouse / uv_row['filename']], cwd=project, env=env)


def _native_build(manifest: dict, uv: Path, venv: Path, project: Path,
                  build_arg: str, env: dict) -> dict:
    prefix = [uv, 'run', '--frozen', '--group', 'native', '--no-sync',
              '--offline', 'python', project / 'tools' / 'dev.py']
    _run(prefix + ['configure', '--build', build_arg, '--config', 'Release'],
         cwd=project, env=env)
    _run(prefix + ['build', '--build', build_arg], cwd=project, env=env)
    output = _run_capture(prefix + ['doctor', '--build', build_arg, '--numerics'],
                          cwd=project, env=env)
    try:
        doctor = json.loads(output)
    except json.JSONDecodeError:
        doctor = {'raw': output.strip()}
    return doctor


def _write_activation(venv: Path, build_dir: Path | None) -> Path:
    wrapper = venv / 'bin' / 'activate-bikesim'
    lines = [
        '# Generated by tools/install_offline.py for the bike-sim offline bundle.',
        '# Source this file; it pins uv to this exact environment regardless of',
        '# any other activated venv or ambient uv configuration.',
        'VIRTUAL_ENV="%s"' % venv,
        'export VIRTUAL_ENV',
        'case ":$PATH:" in',
        '  *":$VIRTUAL_ENV/bin:"*) ;;',
        '  *) PATH="$VIRTUAL_ENV/bin:$PATH" ;;',
        'esac',
        'export PATH',
        'UV_PROJECT_ENVIRONMENT="%s"' % venv,
        'export UV_PROJECT_ENVIRONMENT',
    ]
    if build_dir is not None:
        lines += [
            'BIKE_NATIVE_BUILD_PATH="%s"' % build_dir,
            'export BIKE_NATIVE_BUILD_PATH',
            'NATIVE_TEST_BUILD_PATH="%s"' % build_dir,
            'export NATIVE_TEST_BUILD_PATH',
        ]
    wrapper.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return wrapper


def install_schema1(destination, *, mode: str | None, python_arg: str | None,
                    build_arg: str) -> dict:
    bundle, project = BUNDLE, ROOT
    wheelhouse = bundle / 'wheelhouse'
    raw_path = bundle / 'manifest.json'
    try:
        raw = json.loads(raw_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'cannot read {raw_path}: {error}') from error
    target = raw.get('target') if isinstance(raw, dict) else None
    if not isinstance(target, dict) or not isinstance(target.get('python_minor'), str):
        raise ValueError('manifest.json has no usable target declaration')
    if mode is not None and isinstance(raw, dict) and raw.get('mode') not in (None, mode):
        raise ValueError(
            f"this bundle is mode {raw.get('mode')!r}; the requested installer "
            f'mode {mode!r} does not apply — use the matching script')

    # --- preflight: nothing below this line may create or mutate a venv ---
    python = _select_python(target, python_arg)
    cleaned = _clean_generated_sources(project)
    if cleaned:
        print('Removed install-generated artifacts before preflight: '
              + ', '.join(cleaned[:8]) + (' ...' if len(cleaned) > 8 else ''))
    manifest = _validate_bundle(python, bundle, project)
    native = manifest['capabilities']['native_source_build']
    venv = _select_venv(destination, bundle, project)
    if native:
        for tool in ('cmake', 'c++'):
            if shutil.which(tool) is None:
                raise ValueError(
                    f'native bundles require a preinstalled {tool}; provision '
                    'the compiler toolchain declared in prerequisites.system_libraries')
        cmake_out = _run_capture(['cmake', '--version'], cwd=project,
                                 env=_install_env(venv, None))
        match = re.search(r'version\s+([0-9]+(?:\.[0-9]+)+)', cmake_out)
        minimum = tuple(int(part) for part in manifest['prerequisites']['cmake_minimum'].split('.'))
        if match is None or tuple(int(part) for part in match.group(1).split('.')) < minimum:
            raise ValueError(
                f"cmake>={manifest['prerequisites']['cmake_minimum']} is required; "
                f'found {cmake_out.strip().splitlines()[:1]}')
    providers: dict[str, Path] = {}
    if manifest['mode'] == 'slim':
        providers = _verify_slim_externals(manifest, python)

    # --- environment creation at the validated target ---
    if not (venv / 'pyvenv.cfg').is_file():
        command = [str(python), '-m', 'venv', '--without-pip']
        if manifest['mode'] == 'slim':
            command.append('--system-site-packages')
        command.append(str(venv))
        try:
            subprocess.run(command, check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            raise ValueError(
                f'venv creation failed for {venv}: {error}. On Debian/Ubuntu '
                'install the matching python*-venv package') from error
    env_python = _venv_python(venv)
    probe = _probe_python(env_python)
    if probe is None or not _probe_matches(probe, target):
        raise ValueError(f'the venv interpreter does not match the bundle target: {venv}')

    # The selected uv (system match or the hash-verified bundled wheel) is
    # extracted to a transient directory; the verified wheel is additionally
    # installed into the venv so `uv` persists after this process exits.
    with tempfile.TemporaryDirectory(prefix='bikesim-uv-') as tools_dir:
        uv = _select_uv(manifest, wheelhouse, Path(tools_dir))
        env = _install_env(venv, Path(tools_dir))
        _install_payload(manifest, uv, venv, bundle, project, env)

        provider_evidence = None
        if manifest['mode'] == 'slim':
            provider_evidence = _confirm_slim_externals(manifest, env_python,
                                                        providers, env)
        build_dir = None
        doctor = None
        if native:
            build_dir = (project / build_arg).resolve()
            doctor = _native_build(manifest, uv, venv, project, build_arg, env)

        result = _run_capture([env_python, Path(__file__).resolve(), '--verify'],
                              cwd=project, env=env)
        try:
            verify = json.loads(result.strip().splitlines()[-1])
        except (json.JSONDecodeError, IndexError):
            verify = {'raw': result.strip()}
    wrapper = _write_activation(venv, build_dir)
    venv_uv = venv / 'bin' / 'uv'
    report = {
        'schema_version': 1,
        'mode': manifest['mode'],
        'native_source_build': native,
        'bundle': str(bundle),
        'project': str(project),
        'venv': str(venv),
        'python': {'executable': str(python),
                   'version': (probe or {}).get('version')},
        'uv': {'executable': str(venv_uv if venv_uv.is_file() else uv),
               'version': manifest['prerequisites']['uv_version']},
        'target': manifest['target'],
        'build': str(build_dir) if build_dir is not None else None,
        'slim_external': provider_evidence,
        'doctor': doctor,
        'activation': str(wrapper),
        'verify': verify,
        'network_used': False,
    }
    (venv / 'bikesim-install-report.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(f"Installed bike-sim {manifest['project']['version']} "
          f"({manifest['mode']}, "
          f"{'native source build' if native else 'Python-only'}) "
          'using only the bundled wheelhouse.')
    if manifest['capabilities'].get('gui'):
        print('GUI viewers still require the host display/graphics libraries '
              'listed in prerequisites.system_libraries.')
    print(f'Activate: source {wrapper}')
    if build_dir is not None:
        print(f'Native extension: {build_dir}')
    return report


# ---------------------------------------------------------------------------
# Legacy manifest-less Python-only path — retained verbatim semantics: Linux
# x86_64 CPython 3.13 wheels, wheel/source comparison, inherited site-packages
# for the slim scientific stack.  Never advertised as native-capable.
# ---------------------------------------------------------------------------

def _project():
    import tomllib
    with (ROOT / 'pyproject.toml').open('rb') as stream:
        return tomllib.load(stream)['project']


def _check(requirement):
    Requirement = _requirement_cls()
    import importlib.metadata as md
    req = Requirement(requirement)
    try:
        version = md.version(req.name)
    except md.PackageNotFoundError as exc:
        raise ValueError(f'missing preinstalled package {requirement}') from exc
    if not req.specifier.contains(version, prereleases=True):
        raise ValueError(f'{requirement} is required; found {req.name}=={version}')
    return version


def verify_install():
    import importlib.metadata as md
    Requirement = _requirement_cls()
    project = _project()
    versions = {Requirement(r).name: _check(r) for r in project['dependencies']}
    if md.version(project['name']) != project['version']:
        raise ValueError('installed bike-sim wheel version does not match the source')
    # Verify the dependency closure, including etils[epath], against the actual
    # interpreter search path. uv's environment metadata alone does not describe
    # packages intentionally reused through an inherited-site .pth file.
    pending = [(Requirement(r).name, set(Requirement(r).extras)) for r in project['dependencies']]
    seen = set()
    while pending:
        name, extras = pending.pop()
        key = (name.lower().replace('_', '-'), tuple(sorted(extras)))
        if key in seen:
            continue
        seen.add(key)
        for text in md.requires(name) or ():
            req = Requirement(text)
            if req.marker and not any(req.marker.evaluate({'extra': e}) for e in (extras or {''})):
                continue
            versions[req.name] = _check(str(req))
            pending.append((req.name, set(req.extras)))
    import bike_sim
    import mujoco
    return dict(passed=True, python=sys.version.split()[0], executable=sys.executable,
        project_version=md.version('bike-sim'), package_path=list(bike_sim.__path__),
        versions=dict(sorted(versions.items())), network_used=False,
        scope='Installed project dependency closure, not a claim that unrelated system packages are consistent.')


def _check_wheel_source(wheelhouse):
    Requirement = _requirement_cls()
    wheels = list(wheelhouse.glob('bike_sim-*.whl'))
    if len(wheels) != 1:
        raise ValueError('wheelhouse must contain exactly one rebuilt bike_sim wheel')
    with zipfile.ZipFile(wheels[0]) as archive:
        metadata_name = next(n for n in archive.namelist() if n.endswith('.dist-info/METADATA'))
        from email.parser import BytesParser
        metadata = BytesParser().parsebytes(archive.read(metadata_name))
        if metadata['Version'] != _project()['version']:
            raise ValueError('project wheel is stale: rebuild the bundle with tools/offline_bundle.py')
        files = {p.relative_to(ROOT / 'src').as_posix(): p for p in (ROOT / 'src/bike_sim').rglob('*.py')}
        packaged = {n for n in archive.namelist() if n.startswith('bike_sim/') and n.endswith('.py')}
        if set(files) != packaged or any(archive.read(name) != p.read_bytes() for name, p in files.items()):
            raise ValueError('project wheel content differs from source; rebuild it before installing')


def install_legacy(destination, mode=None):
    """Documented legacy manifest-less path: Python-only, Linux cp313."""
    Requirement = _requirement_cls()
    if sys.version_info[:2] != (3, 13) or sys.platform != 'linux' or platform.machine() not in ('x86_64', 'AMD64'):
        raise ValueError('the manifest-less bundle supplies binary wheels for '
                         'Linux x86_64 and CPython 3.13 only')
    for requirement in _project()['dependencies']:
        if Requirement(requirement).name.lower() not in ('mujoco', 'glfw'):
            _check(requirement)
    wheelhouse = BUNDLE / 'wheelhouse'
    _check_wheel_source(wheelhouse)
    destination = Path(destination).resolve()
    env = dict(os.environ, UV_OFFLINE='1', PIP_NO_INDEX='1', PIP_DISABLE_PIP_VERSION_CHECK='1')
    uv = shutil.which('uv')
    system_site = mode != 'full'
    if not (destination / 'pyvenv.cfg').exists():
        command = ([uv, 'venv', '--offline', '--python', sys.executable, str(destination)]
                   if uv else [sys.executable, '-m', 'venv', str(destination)])
        if system_site:
            command.insert(-1, '--system-site-packages')
        subprocess.run(command, check=True, env=env)
    python = destination / 'bin/python'
    child_version = subprocess.check_output([str(python), '-c', 'import sys; print(sys.version_info[:2])'], text=True)
    if child_version.strip() != '(3, 13)':
        raise ValueError('existing target environment is not CPython 3.13')
    site_path = subprocess.check_output([str(python), '-c',
        'import sysconfig; print(sysconfig.get_path("purelib"))'], text=True).strip()
    if system_site:
        inherited = sorted({str(Path(p).resolve()) for p in sys.path
                            if p and Path(p).is_dir() and Path(p).name in ('site-packages', 'dist-packages')})
        (Path(site_path) / 'bikesim-inherited-scientific-packages.pth').write_text('\n'.join(inherited) + '\n')
    common = ['--no-index', '--no-deps', '--require-hashes', '--only-binary', ':all:',
              '--find-links', str(wheelhouse), '-r', str(BUNDLE / 'requirements.txt')]
    command = ([uv, 'pip', 'install', '--offline', '--python', str(python), '--reinstall-package', 'bike-sim', *common]
               if uv else [str(python), '-m', 'pip', 'install', '--force-reinstall', *common])
    subprocess.run(command, check=True, env=env)
    result = subprocess.check_output([str(python), str(Path(__file__).resolve()), '--verify'], text=True, env=env)
    report = json.loads(result)
    (destination / 'bikesim-install-report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Installed bike-sim {report["project_version"]} using only local wheels.')
    print(f'Activate: source {destination}/bin/activate')
    print('Commands: bike-sim, bike-ride, bike-research, bike-replay')


def _layout() -> str:
    if (BUNDLE / 'manifest.json').is_file():
        return 'schema1'
    if (BUNDLE / 'requirements.txt').is_file() and (BUNDLE / 'wheelhouse').is_dir():
        return 'legacy'
    raise ValueError(
        f'{BUNDLE} is not an extracted bike-sim offline bundle: expected '
        'manifest.json (schema 1) or requirements.txt + wheelhouse/ (legacy) '
        'beside the project/ source tree')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', nargs='?', type=Path, default=None,
                        help='venv install directory (default: <bundle>/.venv)')
    parser.add_argument('--mode', choices=('full', 'slim'), default=None,
                        help='expected bundle mode; a mismatch fails the install')
    parser.add_argument('--python', default=None, metavar='PATH',
                        help='explicit interpreter for the target environment')
    parser.add_argument('--build', default='native/build/release', metavar='PATH',
                        help='native build directory selector relative to project/')
    parser.add_argument('--verify', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        if args.verify:
            print(json.dumps(verify_install(), sort_keys=True, allow_nan=False))
            return 0
        layout = _layout()
        destination = args.destination if args.destination is not None else BUNDLE / '.venv'
        if layout == 'schema1':
            install_schema1(destination, mode=args.mode,
                            python_arg=args.python, build_arg=args.build)
        else:
            install_legacy(destination, mode=args.mode)
        return 0
    except (ValueError, OSError, subprocess.CalledProcessError,
            importlib.metadata.PackageNotFoundError) as exc:
        # verify_install() reads bike-sim's metadata directly; a missing
        # distribution must surface as the same clean diagnostic the
        # legacy verifier printed, not as a traceback.
        print(f'OFFLINE INSTALL ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
