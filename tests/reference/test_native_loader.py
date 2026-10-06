"""Selected native build resolution and import provenance controls."""

from __future__ import annotations

import importlib.machinery
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from native_loader import selected_build


REPO_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_ROOT = Path(__file__).resolve().parent


def _child_env(
    overrides: dict[str, str] | None = None, *, clear_selectors: bool = True
) -> dict[str, str]:
    environment = os.environ.copy()
    if clear_selectors:
        environment.pop('NATIVE_TEST_BUILD_PATH', None)
        environment.pop('NATIVE_TEST_BUILD_DIR', None)
    environment.pop('NATIVE_TEST_PROVENANCE_PATH', None)
    current_pythonpath = environment.get('PYTHONPATH', '')
    pythonpath = [str(REFERENCE_ROOT)]
    if current_pythonpath:
        pythonpath.append(current_pythonpath)
    environment['PYTHONPATH'] = os.pathsep.join(pythonpath)
    if overrides:
        environment.update(overrides)
    return environment


def _run_child(code: str, environment: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return _run_python_command(['python', '-c', code], environment)


def _run_python_command(
    python_arguments: list[str], environment: dict[str, str]
) -> subprocess.CompletedProcess[str]:
    runtime_variables = (
        'DYLD_INSERT_LIBRARIES',
        'LD_PRELOAD',
        'ASAN_OPTIONS',
        'PYTHONMALLOC',
        'NATIVE_TEST_SANITIZER_RUNTIME',
    )
    runtime_assignments = [
        f'{name}={environment[name]}' for name in runtime_variables if name in environment
    ]
    selected_runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    preload_variable = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
    if selected_runtime and not environment.get(preload_variable):
        runtime_assignments.append(f'{preload_variable}={selected_runtime}')
    command = ['uv', 'run']
    if runtime_assignments:
        command.extend(['env', *runtime_assignments])
    command.extend(python_arguments)
    process_environment = environment.copy()
    for name in runtime_variables:
        process_environment.pop(name, None)
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=process_environment,
        text=True,
        capture_output=True,
        check=False,
    )


def _selected_extension(build: Path) -> Path:
    extensions = [
        candidate
        for candidate in build.glob('bike_native*')
        if any(candidate.name.endswith(suffix) for suffix in importlib.machinery.EXTENSION_SUFFIXES)
    ]
    assert len(extensions) == 1, f'expected one built extension in {build}, found {extensions}'
    return extensions[0]


def _copy_selected_extension(destination: Path) -> Path:
    source_build = selected_build(os.environ)
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / _selected_extension(source_build).name
    shutil.copy2(_selected_extension(source_build), target)
    return target


def _selected_asan_runtime() -> Path:
    context = json.loads((selected_build(os.environ) / 'native_check_context.json').read_text())
    compiler = Path(context['compiler']['path'])
    runtime_name = (
        'libclang_rt.asan_osx_dynamic.dylib'
        if sys.platform == 'darwin'
        else f'libclang_rt.asan-{os.uname().machine}.so'
    )
    if sys.platform.startswith('linux') and 'clang' not in compiler.name:
        runtime_name = 'libasan.so'
    result = subprocess.run(
        [str(compiler), f'-print-file-name={runtime_name}'],
        text=True,
        capture_output=True,
        check=False,
    )
    runtime = Path(result.stdout.strip())
    assert result.returncode == 0 and runtime.is_file(), result.stderr or result.stdout
    return runtime.resolve()


def test_selected_build_loads_the_exact_imported_extension() -> None:
    expected_build = selected_build(os.environ)
    result = _run_child(
        "import json, os; from native_loader import load_native, selected_build; "
        "module = load_native(); print(json.dumps([str(selected_build(os.environ)), module.__file__]))",
        _child_env(clear_selectors=False),
    )

    assert result.returncode == 0, result.stderr
    selected, imported = json.loads(result.stdout)
    assert Path(selected).resolve() == expected_build.resolve()
    assert Path(imported).resolve().parent == expected_build.resolve()
    assert Path(imported).resolve() == _selected_extension(expected_build).resolve()


def test_load_native_records_selected_build_and_imported_extension(tmp_path: Path) -> None:
    provenance_path = tmp_path / 'test-native-provenance.json'
    result = _run_child(
        "from native_loader import load_native; load_native()",
        _child_env(
            {'NATIVE_TEST_PROVENANCE_PATH': str(provenance_path)},
            clear_selectors=False,
        ),
    )

    assert result.returncode == 0, result.stderr
    provenance = json.loads(provenance_path.read_text())
    expected_build = selected_build(os.environ).resolve()
    assert provenance['selected_build'] == str(expected_build)
    assert Path(provenance['imported_extension']).resolve() == _selected_extension(expected_build).resolve()


def test_pytest_reporter_records_selected_native_count_and_provenance(tmp_path: Path) -> None:
    provenance_path = tmp_path / 'pytest-native-provenance.json'
    environment = _child_env(
        {'NATIVE_TEST_PROVENANCE_PATH': str(provenance_path)},
        clear_selectors=False,
    )
    result = _run_python_command(
        [
            'python',
            '-m',
            'pytest',
            '-p',
            'native_test_reporter',
            'tests/reference/test_native_stepper.py',
            '--collect-only',
            '-q',
        ],
        environment,
    )

    assert result.returncode == 0, result.stderr
    provenance = json.loads(provenance_path.read_text())
    assert provenance['native_test_count'] > 0
    assert Path(provenance['imported_extension']).resolve().parent == selected_build(os.environ).resolve()

    followup = _run_child('from native_loader import load_native; load_native()', environment)

    assert followup.returncode == 0, followup.stderr
    preserved = json.loads(provenance_path.read_text())
    assert preserved['native_test_count'] == provenance['native_test_count']


def test_pytest_reporter_does_not_count_native_tool_tests() -> None:
    result = _run_python_command(
        [
            'python',
            '-m',
            'pytest',
            '-p',
            'native_test_reporter',
            'tests/reference/test_native_contract_checks.py',
            '--collect-only',
            '-q',
        ],
        _child_env(clear_selectors=False),
    )

    assert result.returncode == pytest.ExitCode.NO_TESTS_COLLECTED
    assert 'native extension test items collected: 0' in result.stdout


def test_repeated_load_after_collection_preserves_native_count(tmp_path: Path) -> None:
    provenance_path = tmp_path / 'repeated-load-provenance.json'
    test_path = tmp_path / 'test_native_repeated_load.py'
    test_path.write_text(
        'from native_loader import load_native\n'
        'bike_native = load_native()\n'
        '\n'
        'def test_load_after_collection_preserves_count():\n'
        '    import json\n'
        '    import os\n'
        '    from pathlib import Path\n'
        '    load_native()\n'
        '    provenance_path = Path(os.environ["NATIVE_TEST_PROVENANCE_PATH"])\n'
        '    provenance = json.loads(provenance_path.read_text())\n'
        '    assert provenance["native_test_count"] == 1\n'
    )
    result = _run_python_command(
        ['python', '-m', 'pytest', '-p', 'native_test_reporter', str(test_path), '-q'],
        _child_env(
            {'NATIVE_TEST_PROVENANCE_PATH': str(provenance_path)},
            clear_selectors=False,
        ),
    )

    assert result.returncode == 0, result.stdout + result.stderr
    provenance = json.loads(provenance_path.read_text())
    assert provenance['native_test_count'] == 1


@pytest.mark.parametrize(
    ('stale_build_matches', 'stale_extension_matches', 'stale_count'),
    [
        (False, True, 7),
        (True, False, 7),
        (True, True, True),
    ],
)
def test_loader_discards_invalid_or_other_artifact_test_counts(
    tmp_path: Path,
    stale_build_matches: bool,
    stale_extension_matches: bool,
    stale_count: object,
) -> None:
    build = selected_build(os.environ).resolve()
    extension = _selected_extension(build).resolve()
    provenance_path = tmp_path / 'prior-run-provenance.json'
    provenance_path.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'selected_build': str(build if stale_build_matches else build / 'different-build'),
                'imported_extension': str(
                    extension if stale_extension_matches else extension.with_name('different-extension.so')
                ),
                'native_test_count': stale_count,
            }
        )
    )

    result = _run_child(
        'from native_loader import load_native; load_native()',
        _child_env(
            {'NATIVE_TEST_PROVENANCE_PATH': str(provenance_path)},
            clear_selectors=False,
        ),
    )

    assert result.returncode == 0, result.stderr
    refreshed = json.loads(provenance_path.read_text())
    assert 'native_test_count' not in refreshed


def test_custom_absolute_build_path_loads_its_extension(tmp_path: Path) -> None:
    extension = _copy_selected_extension(tmp_path / 'custom build')
    result = _run_child(
        "import json, os; from native_loader import load_native, selected_build; "
        "module = load_native(); print(json.dumps([str(selected_build(os.environ)), module.__file__]))",
        _child_env({'NATIVE_TEST_BUILD_PATH': str(extension.parent)}),
    )

    assert result.returncode == 0, result.stderr
    selected, imported = json.loads(result.stdout)
    assert Path(selected).resolve() == extension.parent.resolve()
    assert Path(imported).resolve() == extension.resolve()


@pytest.mark.parametrize(
    ('legacy_selector', 'relative_build'),
    [('', 'build'), ('asan', 'build/asan'), ('rtsan', 'build/rtsan'), ('coverage', 'build/coverage')],
)
def test_legacy_build_selector_resolves_only_known_builds(
    legacy_selector: str, relative_build: str
) -> None:
    result = _run_child(
        "import json, os; from native_loader import selected_build; "
        "print(json.dumps(str(selected_build(os.environ))))",
        _child_env({'NATIVE_TEST_BUILD_DIR': legacy_selector}),
    )

    assert result.returncode == 0, result.stderr
    assert Path(json.loads(result.stdout)).resolve() == (REPO_ROOT / 'native' / relative_build).resolve()


def test_conflicting_build_selectors_fail_in_a_child_process(tmp_path: Path) -> None:
    result = _run_child(
        "from native_loader import selected_build; import os; selected_build(os.environ)",
        _child_env(
            {
                'NATIVE_TEST_BUILD_PATH': str(tmp_path),
                'NATIVE_TEST_BUILD_DIR': 'asan',
            }
        ),
    )

    assert result.returncode != 0
    assert 'conflicting native build selectors' in result.stderr


def test_relative_build_path_is_rejected_in_a_child_process(tmp_path: Path) -> None:
    result = _run_child(
        "from native_loader import selected_build; import os; selected_build(os.environ)",
        _child_env({'NATIVE_TEST_BUILD_PATH': str(tmp_path.name)}),
    )

    assert result.returncode != 0
    assert 'must be absolute' in result.stderr


def test_preimported_module_from_another_build_is_rejected(tmp_path: Path) -> None:
    selected = tmp_path / 'selected'
    selected.mkdir()
    suffix = importlib.machinery.EXTENSION_SUFFIXES[0]
    (selected / f'bike_native{suffix}').write_bytes(b'not a loadable extension')
    wrong_module = tmp_path / 'wrong' / 'bike_native.so'
    result = _run_child(
        "import os, sys, types; from native_loader import load_native; "
        f"module = types.ModuleType('bike_native'); module.__file__ = {str(wrong_module)!r}; "
        "sys.modules['bike_native'] = module; load_native()",
        _child_env({'NATIVE_TEST_BUILD_PATH': str(selected)}),
    )

    assert result.returncode != 0
    assert 'pre-imported bike_native module' in result.stderr


def test_missing_extension_fails_without_import_fallback(tmp_path: Path) -> None:
    result = _run_child(
        "from native_loader import load_native; load_native()",
        _child_env({'NATIVE_TEST_BUILD_PATH': str(tmp_path)}),
    )

    assert result.returncode != 0
    assert 'expected exactly one bike_native extension' in result.stderr


def test_ambiguous_extensions_fail_before_import(tmp_path: Path) -> None:
    suffixes = list(dict.fromkeys(importlib.machinery.EXTENSION_SUFFIXES))
    assert len(suffixes) >= 2
    for suffix in (suffixes[0], suffixes[-1]):
        (tmp_path / f'bike_native{suffix}').write_bytes(b'not a loadable extension')
    result = _run_child(
        "from native_loader import load_native; load_native()",
        _child_env({'NATIVE_TEST_BUILD_PATH': str(tmp_path)}),
    )

    assert result.returncode != 0
    assert 'expected exactly one bike_native extension' in result.stderr


def test_rider_forces_import_rejects_an_earlier_wrong_build(tmp_path: Path) -> None:
    extension = _copy_selected_extension(tmp_path / 'selected build')
    code = (
        "import importlib, os, sys, types; "
        f"module = types.ModuleType('bike_native'); module.__file__ = {str(tmp_path / 'wrong' / 'bike_native.so')!r}; "
        "sys.modules['bike_native'] = module; "
        f"os.environ['NATIVE_TEST_BUILD_PATH'] = {str(extension.parent)!r}; "
        "importlib.import_module('test_native_rider_forces')"
    )
    result = _run_child(code, _child_env())

    assert result.returncode != 0
    assert 'pre-imported bike_native module' in result.stderr
    assert 'test_native_rider_forces' in result.stderr


def test_native_contract_require_control_fails_in_selected_build() -> None:
    build = selected_build(os.environ)
    executable = build / 'native_contract_tests'
    assert executable.is_file(), f'native contract executable has not been built: {executable}'

    result = subprocess.run(
        [str(executable), '--require-failure'],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'deliberate require failure' in result.stderr


def test_native_contract_executable_lists_registered_cases() -> None:
    executable = selected_build(os.environ) / 'native_contract_tests'

    result = subprocess.run(
        [str(executable), '--list'],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'human_crank_torque',
        'pedaling_policy_valid_transition',
        'pedaling_ctor_domain',
        'shifter_ctor_domain',
        'assist_ctor_domain',
        'battery_ctor_domain',
        'freehub_ctor_domain',
        'writer_config_domains',
        'transmission_lifecycle_invariants',
        'typed_ctor_matches_validator_domain',
        'engine_fatal_status_and_reuse',
        'nested_engine_frames',
        'thread_local_engine_frames',
        'previous_tls_handler_restored',
        'owned_staging_owner_failure_frees_storage',
    ]


def test_contract_executable_source_is_in_cmake_manifest() -> None:
    build = selected_build(os.environ)
    manifest = json.loads((build / 'native_sources.json').read_text())
    contract_source = (REPO_ROOT / 'native' / 'tests' / 'test_contracts.cpp').resolve()
    entries = [
        entry for entry in manifest['sources'] if Path(entry['path']).resolve() == contract_source
    ]

    assert len(entries) == 1
    assert entries[0]['target'] == 'native_contract_tests'


@pytest.mark.parametrize(
    'selection',
    ['test_empty_compilation_database_fails_for_expected_sources', 'codex_no_such_native_item'],
)
def test_native_profile_reporter_rejects_zero_native_items(selection: str) -> None:
    result = _run_python_command(
        [
            'python',
            '-m',
            'pytest',
            '-p',
            'native_test_reporter',
            'tests/reference/test_native_check_tools.py',
            '-k',
            selection,
            '--collect-only',
            '-q',
        ],
        _child_env(),
    )

    assert result.returncode != 0
    assert 'native/full profile collected zero native extension test items' in result.stdout + result.stderr


@pytest.mark.parametrize('with_existing_preload', [False, True])
def test_child_confirms_selected_asan_runtime_is_loaded(with_existing_preload: bool) -> None:
    runtime = _selected_asan_runtime()
    runtime_variable = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
    unrelated_preload = '/usr/lib/libSystem.B.dylib' if sys.platform == 'darwin' else 'libc.so.6'
    preload = (
        f'{unrelated_preload}{os.pathsep}{runtime}' if with_existing_preload else str(runtime)
    )
    result = _run_child(
        "from native_loader import verify_sanitizer_runtime; "
        "print(verify_sanitizer_runtime())",
        _child_env(
            {
                'NATIVE_TEST_SANITIZER_RUNTIME': str(runtime),
                runtime_variable: preload,
                'ASAN_OPTIONS': 'detect_leaks=0',
                'PYTHONMALLOC': 'malloc',
            }
        ),
    )

    assert result.returncode == 0, result.stderr
    assert str(runtime) in result.stdout


def test_child_rejects_wrong_preload_when_selected_runtime_is_missing() -> None:
    runtime = _selected_asan_runtime()
    runtime_variable = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
    wrong_preload = '/usr/lib/libSystem.B.dylib' if sys.platform == 'darwin' else 'libc.so.6'
    result = _run_child(
        "from native_loader import verify_sanitizer_runtime; verify_sanitizer_runtime()",
        _child_env(
            {
                'NATIVE_TEST_SANITIZER_RUNTIME': str(runtime),
                runtime_variable: wrong_preload,
                'ASAN_OPTIONS': 'detect_leaks=0',
                'PYTHONMALLOC': 'malloc',
            }
        ),
    )

    assert result.returncode != 0
    assert 'selected sanitizer runtime is not loaded in this Python process' in result.stderr
