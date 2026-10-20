"""Platform linkage evidence for the native target graph.

These controls run against the selected native build and prove the target
graph carries its own runtime search behavior:

* every CTest executable runs directly with the library-search environment
  scrubbed — no dependency may be supplied accidentally through
  ``LD_LIBRARY_PATH``/``DYLD_*`` rescue variables;
* on Darwin every engine consumer records the selected absolute MuJoCo
  install name (``otool -L``), never the wheel's unanchored @rpath name;
* on Linux every binary's ``DT_NEEDED``/``DT_RUNPATH`` resolves the engine
  image and the numerics provider (``readelf -d`` / ``ldd``), with no
  "not found" remainder;
* the ``bike_native`` extension imports with the same scrubbed
  environment.

Sanitizer runtime injection (``NATIVE_TEST_SANITIZER_RUNTIME`` →
``LD_PRELOAD``/``DYLD_INSERT_LIBRARIES``) is instrumentation, not a
dependency-location mechanism, so it is preserved while the search-path
variables are removed.
"""

from __future__ import annotations

import json
import os
import platform
import re
import shlex
import subprocess
import sys
from pathlib import Path

import pytest

from native_loader import selected_build


_SEARCH_ENV_VARS = (
    'LD_LIBRARY_PATH',
    'DYLD_LIBRARY_PATH',
    'DYLD_FALLBACK_LIBRARY_PATH',
)

# CTestTestfile.cmake serializes the legacy signature — quoted or
# bracket-quoted name depending on the generator version:
#   add_test("<name>" "<command>" ["<arg>" ...])
#   add_test([=[<name>]=] "<command>" ...)
_ADD_TEST = re.compile(
    r'add_test\(\s*(?:"|\[=\[)?([A-Za-z0-9_]+)(?:"|\]=\])?\s+([^\n)]+)\)',
    re.IGNORECASE,
)

# CTest registrations that name an interpreter or script rather than a
# built binary are excluded — only native executables are smoke-tested.
# An empty list means the registered command must be the executable named
# after the test; otherwise it must be one of the listed executables.
_EXPECTED_CTESTS = {
    'native_contract_tests': [],
    'native_distance_tests': [],
    'native_station_tests': [],
    'native_platform_blas_tests': [],
    'native_platform_lapack_tests': [],
    'native_bench_bare': ['native_bench'],
    'native_bench_glue': ['native_bench'],
    'native_bench_heavy': ['native_bench'],
}


@pytest.fixture(scope='module')
def build_dir() -> Path:
    build = selected_build(dict(os.environ)).resolve()
    if not build.is_dir():
        pytest.fail(f'selected native build does not exist: {build}')
    return build


@pytest.fixture(scope='module')
def scrubbed_env() -> dict[str, str]:
    environment = os.environ.copy()
    for name in _SEARCH_ENV_VARS:
        environment.pop(name, None)
    return environment


def _resolve_executable(build: Path, name: str) -> Path:
    candidates = [build / name]
    context_dir = build / 'native_check_context'
    if context_dir.is_dir():
        for context_file in context_dir.glob('*.json'):
            candidates.append(build / context_file.stem / name)
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    pytest.fail(f'missing native executable {name} under {build}')


def _resolve_extension(build: Path) -> Path:
    """The built bike_native extension module — an engine consumer too."""
    import importlib.machinery

    candidates = [
        build / f'bike_native{suffix}'
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
    ]
    extension = next((path for path in candidates if path.is_file()), None)
    if extension is None:
        pytest.fail(f'no bike_native extension in {build}')
    return extension.resolve()


def _ctest_commands(build: Path) -> dict[str, list[str]]:
    ctest_file = build / 'CTestTestfile.cmake'
    if not ctest_file.is_file():
        pytest.fail(f'missing CTest registration file: {ctest_file}')
    commands: dict[str, list[str]] = {}
    for match in _ADD_TEST.finditer(ctest_file.read_text()):
        test_name, command_text = match.group(1), match.group(2).strip()
        commands[test_name] = shlex.split(command_text)
    missing = [
        name for name in _EXPECTED_CTESTS if name not in commands
    ]
    if missing:
        pytest.fail(f'CTest registration is incomplete; missing: {missing}')
    wrong = [
        name
        for name, expected in _EXPECTED_CTESTS.items()
        if Path(commands[name][0]).name not in (expected or [name])
    ]
    if wrong:
        pytest.fail(
            'CTest registrations run unexpected executables: '
            + '; '.join(
                f'{name} runs {commands[name][0]}'
                for name in wrong
            )
        )
    return commands


def test_every_ctest_executable_runs_directly(
    build_dir: Path, scrubbed_env: dict[str, str]
) -> None:
    """Direct-exec every registered CTest case without search-path rescue."""
    commands = _ctest_commands(build_dir)
    failures: list[str] = []
    for test_name in sorted(_EXPECTED_CTESTS):
        argv = commands[test_name]
        executable = _resolve_executable(build_dir, Path(argv[0]).name)
        result = subprocess.run(
            [str(executable), *argv[1:]],
            cwd=build_dir,
            env=scrubbed_env,
            text=True,
            capture_output=True,
            timeout=600,
            check=False,
        )
        if result.returncode != 0:
            failures.append(
                f'{test_name} exited {result.returncode}: '
                f'{result.stderr.strip() or result.stdout.strip() or "no output"}'
            )
    if failures:
        pytest.fail('; '.join(failures))


def _darwin_install_names(executable: Path) -> list[str]:
    result = subprocess.run(
        ['otool', '-L', str(executable)],
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(f'otool -L failed on {executable}: {result.stderr.strip()}')
    lines = [line.strip() for line in result.stdout.splitlines()[1:]]
    return [line.split(' ')[0] for line in lines if line]


def _linux_dynamic_tags(executable: Path) -> tuple[list[str], list[str]]:
    result = subprocess.run(
        ['readelf', '-d', str(executable)],
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(f'readelf -d failed on {executable}: {result.stderr.strip()}')
    needed = re.findall(r'NEEDED.*\[(.+?)\]', result.stdout)
    runpaths: list[str] = []
    for match in re.finditer(
        r'(?:RUNPATH|RPATH).*\[(.+?)\]', result.stdout
    ):
        runpaths.extend(match.group(1).split(':'))
    return needed, runpaths


def _ldd_unresolved(executable: Path, env: dict[str, str]) -> list[str]:
    result = subprocess.run(
        ['ldd', str(executable)],
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(f'ldd failed on {executable}: {result.stderr.strip()}')
    return [
        line.strip()
        for line in result.stdout.splitlines()
        if 'not found' in line
    ]


def test_engine_consumers_record_resolvable_engine_image(
    build_dir: Path, scrubbed_env: dict[str, str]
) -> None:
    """Every engine consumer resolves the selected engine image by itself."""
    context_path = build_dir / 'native_check_context.json'
    if not context_path.is_file():
        contexts = sorted((build_dir / 'native_check_context').glob('*.json'))
        if len(contexts) != 1:
            pytest.fail('cannot select the native check context for linkage checks')
        context_path = contexts[0]
    context = json.loads(context_path.read_text())
    mujoco_paths = [
        entry for entry in context.get('library_paths', [])
        if 'mujoco' in Path(entry).name.lower() or 'libmujoco' in entry
    ]
    if not mujoco_paths:
        pytest.fail('context records no engine image in library_paths')
    mujoco_image = Path(mujoco_paths[0])

    # bike_native links the engine image like the executables do — inspect
    # its recorded dependencies structurally, not only through import.
    engine_consumers = [
        ('native_contract_tests', _resolve_executable(build_dir, 'native_contract_tests')),
        ('native_distance_tests', _resolve_executable(build_dir, 'native_distance_tests')),
        ('native_station_tests', _resolve_executable(build_dir, 'native_station_tests')),
        ('native_bench', _resolve_executable(build_dir, 'native_bench')),
        ('bike_native', _resolve_extension(build_dir)),
    ]
    system = platform.system()
    failures: list[str] = []
    for name, executable in engine_consumers:
        if system == 'Darwin':
            names = _darwin_install_names(executable)
            if str(mujoco_image) not in names:
                failures.append(
                    f'{name}: records no absolute engine install name '
                    f'({mujoco_image}); names={names}'
                )
            if any('mujoco.framework' in recorded for recorded in names):
                failures.append(
                    f'{name}: still records the unanchored @rpath framework name'
                )
        elif system == 'Linux':
            needed, runpaths = _linux_dynamic_tags(executable)
            soname = mujoco_image.name
            # libmujoco.so.3.12.0 records a versioned SONAME — accept any
            # needed entry sharing the libmujoco.so stem.
            if not any(entry.startswith('libmujoco.so') for entry in needed):
                failures.append(f'{name}: no libmujoco DT_NEEDED entry')
            if str(mujoco_image.parent) not in runpaths:
                failures.append(
                    f'{name}: RUNPATH lacks the engine image directory '
                    f'{mujoco_image.parent}; runpaths={runpaths}'
                )
            unresolved = _ldd_unresolved(executable, scrubbed_env)
            if unresolved:
                failures.append(f'{name}: ldd unresolved: {unresolved}')
        else:
            pytest.fail(f'unsupported host platform: {system}')
    if failures:
        pytest.fail('; '.join(failures))


def test_extension_imports_without_environment_rescue(
    build_dir: Path, scrubbed_env: dict[str, str]
) -> None:
    """import bike_native with no library-search environment variables."""
    _resolve_extension(build_dir)
    result = subprocess.run(
        [
            sys.executable,
            '-c',
            'import sys; sys.path.insert(0, sys.argv[1]); import bike_native',
            str(build_dir),
        ],
        env=scrubbed_env,
        text=True,
        capture_output=True,
        timeout=120,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(
            f'import bike_native failed without environment rescue: '
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
