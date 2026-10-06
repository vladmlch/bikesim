#!/usr/bin/env python3
"""Fail closed on native build structure, selected-artifact use, and CTest contracts."""

from __future__ import annotations

import argparse
import ast
import fnmatch
import importlib.machinery
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

from native_checks import CheckError as NativeCheckError
from native_checks import load_context, load_entries


class ContractError(ValueError):
    """A configured native contract is absent or malformed."""


_SUPPORT_TEST_MODULES = {
    'test_native_check_tools.py',
    'test_native_contract_checks.py',
    'test_native_loader.py',
}


def _read_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as error:
        raise ContractError(f'missing {label}: {path}') from error
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'malformed {label}: {path}: {error}') from error


def _extensions(build: Path) -> list[Path]:
    return [
        (build / f'bike_native{suffix}').resolve()
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (build / f'bike_native{suffix}').is_file()
    ]


def _selected_extension(build: Path) -> Path:
    extensions = _extensions(build)
    if len(extensions) != 1:
        raise ContractError(
            f'expected exactly one bike_native extension in {build}, found {len(extensions)}'
        )
    return extensions[0]


def _validate_provenance(build: Path, extension: Path, environment: dict[str, str]) -> dict[str, Any]:
    selected_value = environment.get('NATIVE_TEST_BUILD_PATH')
    if selected_value is not None:
        selected_path = Path(selected_value)
        if not selected_path.is_absolute() or selected_path.resolve() != build:
            raise ContractError(
                f'NATIVE_TEST_BUILD_PATH differs from requested build: {selected_value} != {build}'
            )
    path_value = environment.get('NATIVE_TEST_PROVENANCE_PATH')
    provenance_path = (
        Path(path_value).expanduser().resolve()
        if path_value is not None
        else build / 'native_test_provenance.json'
    )
    provenance = _read_json(provenance_path, 'native loader provenance')
    if (
        not isinstance(provenance, dict)
        or isinstance(provenance.get('schema_version'), bool)
        or provenance.get('schema_version') != 1
    ):
        raise ContractError(f'native loader provenance has an unsupported schema: {provenance_path}')
    if provenance.get('selected_build') != str(build):
        raise ContractError(
            'native loader provenance selected build does not match: '
            f"{provenance.get('selected_build')!r} != {str(build)!r}"
        )
    imported = provenance.get('imported_extension')
    if (
        not isinstance(imported, str)
        or not Path(imported).is_absolute()
        or Path(imported).resolve() != extension
    ):
        raise ContractError(
            'native loader provenance imported extension does not match selected artifact: '
            f'{imported!r} != {str(extension)!r}'
        )
    native_test_count = provenance.get('native_test_count')
    if native_test_count is not None and (
        isinstance(native_test_count, bool)
        or not isinstance(native_test_count, int)
        or native_test_count < 0
    ):
        raise ContractError('native loader provenance has an invalid native_test_count')
    return provenance


def _validate_language_and_hardening(context: dict[str, Any], sources: dict[str, Any]) -> dict[str, Any]:
    target_contexts = context.get('target_contexts')
    entries = sources.get('sources')
    if not isinstance(target_contexts, dict) or not isinstance(entries, list) or not entries:
        raise ContractError('native target context or source manifest is missing')
    targets = {entry.get('target') for entry in entries if isinstance(entry, dict)}
    if None in targets or not targets:
        raise ContractError('native source manifest contains an invalid target list')
    hardening = context.get('libcpp_hardening')
    if not isinstance(hardening, dict):
        raise ContractError('native check context has no libc++ hardening record')
    mode = hardening.get('mode')
    supported = hardening.get('supported')
    if not isinstance(supported, bool):
        raise ContractError('native libc++ hardening support must be boolean')
    required_definition = None
    if supported:
        if mode not in {'EXTENSIVE', 'FAST'}:
            raise ContractError(f'unsupported configured libc++ hardening mode: {mode!r}')
        required_definition = f'_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_{mode}'

    violations: list[str] = []
    for target in sorted(targets):
        target_context = target_contexts.get(target)
        if not isinstance(target_context, dict):
            violations.append(f'{target}: no configured target context')
            continue
        if target_context.get('cxx_standard') != 23:
            violations.append(f'{target}: C++ standard is not ISO C++23')
        if target_context.get('cxx_extensions') is not False:
            violations.append(f'{target}: C++ language extensions are enabled')
        definitions = target_context.get('compile_definitions')
        if not isinstance(definitions, list):
            violations.append(f'{target}: compile definitions are malformed')
        elif required_definition is not None and required_definition not in definitions:
            violations.append(f'{target}: missing libc++ hardening definition {required_definition}')
    if violations:
        raise ContractError('; '.join(violations))
    return {'targets': sorted(targets), 'cxx_standard': 23, 'extensions': False, 'hardening_mode': mode}


def _validate_tidy_policy(repo_root: Path) -> dict[str, Any]:
    path = repo_root / 'native' / '.clang-tidy'
    try:
        content = path.read_text()
    except OSError as error:
        raise ContractError(f'cannot read native clang-tidy policy {path}: {error}') from error
    checks_value = _yaml_setting(content, 'Checks')
    header_filter = _yaml_setting(content, 'HeaderFilterRegex')
    if checks_value is None or header_filter is None:
        raise ContractError('native clang-tidy policy must declare Checks and HeaderFilterRegex')
    tokens = [token.strip() for token in checks_value.split(',') if token.strip()]
    move_enabled = False
    for token in tokens:
        disabled = token.startswith('-')
        pattern = token[1:] if disabled else token
        if fnmatch.fnmatchcase('clang-analyzer-cplusplus.Move', pattern):
            move_enabled = not disabled
    if not move_enabled:
        raise ContractError('clang-tidy must enable first-party clang-analyzer-cplusplus.Move findings')

    try:
        pattern = re.compile(header_filter)
    except re.error as error:
        raise ContractError(f'native clang-tidy header filter is invalid: {error}') from error
    required_paths = (
        'native/src/contract.hpp',
        'native/tests/test_contracts.cpp',
        'tools/proto_native_bench/bench.cpp',
    )
    if any(pattern.search(path) is None for path in required_paths):
        raise ContractError('native clang-tidy header filter omits a portable first-party path')
    if any(pattern.search(f'/checkout/{path}') is None for path in required_paths):
        raise ContractError('native clang-tidy header filter is not checkout portable')
    return {'move_analysis': 'enabled', 'header_filter': header_filter}


def _yaml_setting(content: str, key: str) -> str | None:
    lines = content.splitlines()
    for index, line in enumerate(lines):
        match = re.match(rf'^\s*{re.escape(key)}\s*:\s*(.*?)\s*$', line)
        if match is None:
            continue
        value = match.group(1).strip()
        if value in {'>', '|', '>-', '|-'}:
            setting_indent = len(line) - len(line.lstrip())
            block: list[str] = []
            for block_line in lines[index + 1 :]:
                if not block_line.strip():
                    continue
                indentation = len(block_line) - len(block_line.lstrip())
                if indentation <= setting_indent:
                    break
                block.append(block_line.strip())
            return ' '.join(block)
        return value.strip("'\"")
    return None


def _validate_test_loaders(repo_root: Path) -> dict[str, Any]:
    reference_root = repo_root / 'tests' / 'reference'
    test_files = sorted(
        path
        for path in reference_root.glob('test_native_*.py')
        if path.name not in _SUPPORT_TEST_MODULES
    )
    if not test_files:
        raise ContractError(f'no native extension tests found under {reference_root}')
    violations: list[str] = []
    for path in test_files:
        try:
            tree = ast.parse(path.read_text(), filename=str(path))
        except (OSError, SyntaxError) as error:
            violations.append(f'{path.name}: cannot inspect module: {error}')
            continue
        imports_loader = any(
            isinstance(node, ast.ImportFrom)
            and node.module == 'native_loader'
            and any(alias.name == 'load_native' for alias in node.names)
            for node in ast.walk(tree)
        )
        calls_loader = any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'load_native'
            for node in ast.walk(tree)
        )
        imports_raw_module = any(
            (isinstance(node, ast.Import) and any(alias.name == 'bike_native' for alias in node.names))
            or (isinstance(node, ast.ImportFrom) and node.module == 'bike_native')
            for node in ast.walk(tree)
        )
        if not imports_loader or not calls_loader or imports_raw_module:
            violations.append(f'{path.name}: native tests must use the selected-artifact loader')
    if violations:
        raise ContractError('; '.join(violations))
    return {'module_count': len(test_files), 'modules': [path.name for path in test_files]}


def _validate_contract_manifest(sources: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    entries = sources.get('sources')
    contract_source = (repo_root / 'native' / 'tests' / 'test_contracts.cpp').resolve()
    if not isinstance(entries, list):
        raise ContractError('native source manifest has no source entries')
    matches = [
        entry
        for entry in entries
        if isinstance(entry, dict)
        and Path(str(entry.get('path', ''))).resolve() == contract_source
        and entry.get('target') == 'native_contract_tests'
    ]
    if len(matches) != 1:
        raise ContractError('native contract source is not uniquely manifested in native_contract_tests')
    return {'source': str(contract_source), 'target': 'native_contract_tests'}


def _contract_executable(build: Path) -> Path:
    candidates = [
        build / 'native_contract_tests',
        build / 'Debug' / 'native_contract_tests',
        build / 'Release' / 'native_contract_tests',
    ]
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise ContractError(f'missing executable native contract test binary in {build}')


def _run_contract_case(executable: Path, arguments: list[str], label: str, build: Path) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    runtime_value = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime_value is not None:
        runtime = Path(runtime_value)
        if not runtime.is_absolute() or not runtime.is_file():
            raise ContractError(f'selected sanitizer runtime is not an absolute file: {runtime}')
        preload_name = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        existing = environment.get(preload_name, '').split(os.pathsep)
        if str(runtime) not in existing:
            environment[preload_name] = os.pathsep.join(
                [value for value in existing if value] + [str(runtime)]
            )
    try:
        result = subprocess.run(
            [str(executable), *arguments],
            cwd=build,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError as error:
        raise ContractError(f'cannot run native contract executable: {error}') from error
    log_directory = build / 'native_check_logs' / 'contracts'
    log_directory.mkdir(parents=True, exist_ok=True)
    safe_label = re.sub(r'[^A-Za-z0-9_.-]+', '_', label)
    (log_directory / f'{safe_label}.stdout.log').write_text(result.stdout)
    (log_directory / f'{safe_label}.stderr.log').write_text(result.stderr)
    if result.returncode != 0:
        raise ContractError(
            f'native contract executable {label} exited {result.returncode}: '
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
    return result


def _validate_ctest_contracts(build: Path) -> dict[str, Any]:
    ctest_file = build / 'CTestTestfile.cmake'
    try:
        ctest_contents = ctest_file.read_text()
    except OSError as error:
        raise ContractError(f'cannot read CTest registration file {ctest_file}: {error}') from error
    registration = re.compile(
        r'add_test\([^\n)]*native_contract_tests[^\n)]*native_contract_tests[^\n)]*\)',
        re.IGNORECASE,
    )
    if registration.search(ctest_contents) is None:
        raise ContractError('native_contract_tests is not registered with CTest')

    executable = _contract_executable(build)
    listed = _run_contract_case(executable, ['--list'], 'case-list', build)
    names = [line.strip() for line in listed.stdout.splitlines() if line.strip()]
    if not names or len(set(names)) != len(names):
        raise ContractError('native contract executable lists no cases or contains duplicate case names')
    invalid = [name for name in names if re.fullmatch(r'[A-Za-z0-9_]+', name) is None]
    if invalid:
        raise ContractError(f'native contract case names are malformed: {invalid}')
    for name in names:
        _run_contract_case(executable, [name], f'case-{name}', build)
    return {'executable': str(executable), 'cases': names, 'ctest_registered': True}


def _check_pending_rules(repo_root: Path, sources: dict[str, Any]) -> dict[str, dict[str, str]]:
    source_entries = sources.get('sources', [])
    native_src = repo_root / 'native' / 'src'
    engine_source = native_src / 'engine_call.cpp'
    engine_header = native_src / 'engine_call.hpp'
    engine_abi = native_src / 'engine_abi_312.hpp'
    engine_files = (engine_source, engine_header, engine_abi)
    if any(path.exists() for path in engine_files):
        if not all(path.is_file() for path in engine_files):
            raise ContractError('installed engine-call boundary is missing a source or ABI header')
        matches = [
            entry for entry in source_entries
            if isinstance(entry, dict)
            and Path(str(entry.get('path', ''))).resolve() == engine_source.resolve()
            and entry.get('target') == 'bike_native_engine'
        ]
        if len(matches) != 1:
            raise ContractError('engine_call.cpp is not uniquely manifested in bike_native_engine')
        header_text = engine_header.read_text()
        required_wrappers = (
            'invoke', 'load_model', 'make_data', 'forward', 'step',
            'reset_data', 'set_const', 'try_forward', 'try_step',
        )
        missing = [
            name for name in required_wrappers
            if re.search(rf'\b{re.escape(name)}\s*\(', header_text) is None
        ]
        if missing:
            raise ContractError('engine-call interface is missing wrappers: ' + ', '.join(missing))
        fatal_calls = re.compile(r'\bmj_(?:loadModel(?:Buffer)?|makeData|forward|step|resetData|setConst)\s*\(')
        unguarded = [
            str(path.relative_to(repo_root)) for path in native_src.rglob('*.cpp')
            if path.resolve() != engine_source.resolve()
            and fatal_calls.search(path.read_text()) is not None
        ]
        if unguarded:
            raise ContractError('unguarded fatal-capable MuJoCo calls: ' + ', '.join(unguarded))
        engine_status = {
            'status': 'passed',
            'detail': 'pinned engine-call wrappers are manifested; direct fatal-capable calls are confined to engine_call.cpp',
        }
    else:
        engine_status = {
            'status': 'pending',
            'detail': 'no engine-call wrapper declaration manifest exists yet',
        }
    benchmark_targets = {
        entry.get('target')
        for entry in source_entries
        if isinstance(entry, dict) and entry.get('target') == 'native_bench'
    }
    if benchmark_targets:
        benchmark_sources = {
            path.resolve()
            for path in (repo_root / 'tools' / 'proto_native_bench').glob('*.cpp')
            if path.is_file()
        }
        if not benchmark_sources:
            raise ContractError('native_bench target has no C++ sources under tools/proto_native_bench')
        manifested = {
            Path(entry['path']).resolve()
            for entry in source_entries
            if isinstance(entry, dict) and entry.get('target') in benchmark_targets
        }
        missing = sorted(str(path) for path in benchmark_sources - manifested)
        if missing:
            raise ContractError('benchmark sources are missing from CMake manifest: ' + ', '.join(missing))
        benchmark_status = {'status': 'passed', 'detail': 'configured benchmark sources are manifested'}
    else:
        benchmark_status = {
            'status': 'pending',
            'detail': 'no configured proto_native_bench target exists yet',
        }
    return {
        'engine_call_wrappers': engine_status,
        'benchmark_manifest_membership': benchmark_status,
    }


def _write_summary(build: Path, summary: dict[str, Any]) -> None:
    summary_directory = build / 'native_check_summaries'
    summary_directory.mkdir(parents=True, exist_ok=True)
    summary_path = summary_directory / 'contracts.json'
    temporary_path = summary_path.with_name(f'{summary_path.name}.tmp')
    temporary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    os.replace(temporary_path, summary_path)


def check_native_contracts(build: Path, repo_root: Path, environment: dict[str, str]) -> dict[str, Any]:
    build = build.expanduser().resolve()
    repo_root = repo_root.expanduser().resolve()
    if not build.is_dir():
        raise ContractError(f'build directory does not exist: {build}')
    context = load_context(build)
    sources = _read_json(build / 'native_sources.json', 'native source manifest')
    if not isinstance(sources, dict) or sources.get('schema_version') != 1:
        raise ContractError('native source manifest has an unsupported schema')
    source_entries = sources.get('sources')
    if not isinstance(source_entries, list) or not source_entries:
        raise ContractError('native source manifest has an empty source set')
    expected: set[Path] = set()
    for index, entry in enumerate(source_entries):
        if (
            not isinstance(entry, dict)
            or not isinstance(entry.get('path'), str)
            or not isinstance(entry.get('target'), str)
            or not isinstance(entry.get('context'), str)
        ):
            raise ContractError(f'native source manifest entry {index} is malformed')
        expected.add(Path(entry['path']).resolve())
    try:
        compiled_entries = load_entries(build, expected)
    except NativeCheckError as error:
        raise ContractError(f'native source manifest differs from selected translation units: {error}') from error

    extension = _selected_extension(build)
    checks: dict[str, Any] = {
        'manifest_selection': {
            'status': 'passed',
            'source_count': len(expected),
            'translation_unit_context_count': len(compiled_entries),
        },
    }
    checks['loader_provenance'] = {
        'status': 'passed',
        'provenance': _validate_provenance(build, extension, environment),
    }
    checks['iso_language_mode_and_hardening'] = {
        'status': 'passed',
        **_validate_language_and_hardening(context, sources),
    }
    checks['analysis_policy'] = {'status': 'passed', **_validate_tidy_policy(repo_root)}
    checks['selected_artifact_assertions'] = {
        'status': 'passed',
        **_validate_test_loaders(repo_root),
    }
    checks['contract_source_manifest_membership'] = {
        'status': 'passed',
        **_validate_contract_manifest(sources, repo_root),
    }
    checks['ctest_contract_cases'] = {'status': 'passed', **_validate_ctest_contracts(build)}
    checks.update(_check_pending_rules(repo_root, sources))
    return {
        'schema_version': 1,
        'status': 'passed',
        'build': str(build),
        'checks': checks,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True)
    parser.add_argument('--repo-root', default=str(Path(__file__).resolve().parents[1]))
    arguments = parser.parse_args(argv)
    build = Path(arguments.build).expanduser().resolve()
    try:
        summary = check_native_contracts(build, Path(arguments.repo_root), os.environ.copy())
    except (ContractError, NativeCheckError, KeyError, TypeError, OSError) as error:
        summary = {
            'schema_version': 1,
            'status': 'failed',
            'build': str(build),
            'failures': [str(error)],
        }
        if build.is_dir():
            _write_summary(build, summary)
        print(f'native contract checks failed: {error}', file=sys.stderr)
        return 1
    _write_summary(build, summary)
    print(f'native contract checks passed: {build}')
    for name, result in summary['checks'].items():
        print(f"  {name}: {result['status']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
