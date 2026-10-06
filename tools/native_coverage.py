#!/usr/bin/env python3
"""Merge one native test run's LLVM profiles and record artifact provenance."""

from __future__ import annotations

import argparse
import importlib.machinery
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


class CoverageError(ValueError):
    """A selected build, profile set, or LLVM report is invalid."""


def _read_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as error:
        raise CoverageError(f'missing {label}: {path}') from error
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CoverageError(f'malformed {label}: {path}: {error}') from error


def _extension_artifacts(build: Path) -> list[Path]:
    return [
        (build / f'bike_native{suffix}').resolve()
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (build / f'bike_native{suffix}').is_file()
    ]


def _executable(value: str, label: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise CoverageError(f'{label} is not an executable file: {path}')
    return path


def _run_logged(
    argv: list[str], *, run_directory: Path, name: str
) -> subprocess.CompletedProcess[str]:
    logs = run_directory / 'logs'
    logs.mkdir(parents=True, exist_ok=True)
    try:
        result = subprocess.run(
            argv,
            cwd=run_directory,
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError as error:
        (logs / f'{name}.stdout.log').write_text('')
        (logs / f'{name}.stderr.log').write_text(str(error) + '\n')
        raise CoverageError(
            f'could not launch {name}: {error}; logs retained in {logs}'
        ) from error
    (logs / f'{name}.stdout.log').write_text(result.stdout)
    (logs / f'{name}.stderr.log').write_text(result.stderr)
    (logs / f'{name}.command.json').write_text(json.dumps(argv, indent=2) + '\n')
    if result.returncode != 0:
        raise CoverageError(
            f'{name} exited {result.returncode}; logs retained in {logs}: '
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
    return result


def _tool_version(
    executable: Path, *, run_directory: Path, name: str
) -> dict[str, str]:
    result = _run_logged(
        [str(executable), '--version'], run_directory=run_directory, name=f'{name}-version'
    )
    version = result.stdout.strip() or result.stderr.strip()
    if not version:
        raise CoverageError(f'{name} returned an empty version string')
    return {'path': str(executable), 'version': version.splitlines()[0]}


def _load_test_provenance(run_directory: Path, build: Path, extension: Path) -> tuple[dict[str, Any], int]:
    path = run_directory / 'test_native_provenance.json'
    provenance = _read_json(path, 'native test provenance')
    if (
        not isinstance(provenance, dict)
        or isinstance(provenance.get('schema_version'), bool)
        or provenance.get('schema_version') != 1
    ):
        raise CoverageError(f'native test provenance has an unsupported schema: {path}')
    if provenance.get('selected_build') != str(build):
        raise CoverageError(
            'native test provenance selected build does not match requested build: '
            f"selected={provenance.get('selected_build')!r}, requested={str(build)!r}"
        )
    imported_value = provenance.get('imported_extension')
    if (
        not isinstance(imported_value, str)
        or not Path(imported_value).is_absolute()
        or Path(imported_value).resolve() != extension
    ):
        raise CoverageError(
            'imported extension does not match selected build artifact: '
            f'imported={imported_value!r}, expected={extension}'
        )
    native_test_count = provenance.get('native_test_count')
    if (
        isinstance(native_test_count, bool)
        or not isinstance(native_test_count, int)
        or native_test_count <= 0
    ):
        raise CoverageError('native test provenance must contain a positive native_test_count')
    return provenance, native_test_count


def finalize_coverage(
    *, build_value: str, run_directory_value: str, llvm_profdata_value: str, llvm_cov_value: str
) -> dict[str, Any]:
    build = Path(build_value).expanduser().resolve()
    run_directory = Path(run_directory_value).expanduser().resolve()
    if not build.is_dir():
        raise CoverageError(f'build directory does not exist: {build}')
    if not run_directory.is_dir():
        raise CoverageError(f'coverage run directory does not exist: {run_directory}')
    if not run_directory.is_relative_to(build):
        raise CoverageError(f'coverage run directory must be inside the selected build: {run_directory}')
    cache = build / 'CMakeCache.txt'
    try:
        cache_text = cache.read_text()
    except (OSError, UnicodeDecodeError) as error:
        raise CoverageError(f'cannot read selected build cache: {cache}: {error}') from error
    if 'NATIVE_COVERAGE:BOOL=ON' not in cache_text:
        raise CoverageError(f'selected build is not configured with NATIVE_COVERAGE=ON: {build}')

    extensions = _extension_artifacts(build)
    if len(extensions) != 1:
        raise CoverageError(
            f'expected exactly one bike_native extension in {build}, found {len(extensions)}'
        )
    extension = extensions[0]
    _test_provenance, native_test_count = _load_test_provenance(run_directory, build, extension)

    profile_directory = run_directory / 'profiles'
    profiles = sorted(
        path for path in profile_directory.glob('*.profraw') if path.is_file() and path.stat().st_size > 0
    ) if profile_directory.is_dir() else []
    if not profiles:
        raise CoverageError(f'no nonempty profraw files from this run in {profile_directory}')

    llvm_profdata = _executable(llvm_profdata_value, 'llvm-profdata')
    llvm_cov = _executable(llvm_cov_value, 'llvm-cov')
    tool_versions = {
        'llvm_profdata': _tool_version(
            llvm_profdata, run_directory=run_directory, name='llvm-profdata'
        ),
        'llvm_cov': _tool_version(llvm_cov, run_directory=run_directory, name='llvm-cov'),
    }

    merged_profile = run_directory / 'coverage.profdata'
    _run_logged(
        [
            str(llvm_profdata),
            'merge',
            '-sparse',
            *(str(path) for path in profiles),
            '-o',
            str(merged_profile),
        ],
        run_directory=run_directory,
        name='llvm-profdata-merge',
    )
    if not merged_profile.is_file() or merged_profile.stat().st_size == 0:
        raise CoverageError(f'llvm-profdata produced no merged profile: {merged_profile}')

    text_report = run_directory / 'coverage_report.txt'
    report = _run_logged(
        [str(llvm_cov), 'report', str(extension), f'-instr-profile={merged_profile}'],
        run_directory=run_directory,
        name='llvm-cov-report',
    ).stdout
    if not report.strip() or 'TOTAL' not in report:
        raise CoverageError('llvm-cov report is empty or has no TOTAL summary')
    text_report.write_text(report)

    export_report = run_directory / 'coverage_export.json'
    exported = _run_logged(
        [str(llvm_cov), 'export', str(extension), f'-instr-profile={merged_profile}'],
        run_directory=run_directory,
        name='llvm-cov-export',
    ).stdout
    try:
        export_value = json.loads(exported)
    except json.JSONDecodeError as error:
        raise CoverageError(f'llvm-cov export did not return valid JSON: {error}') from error
    data = export_value.get('data') if isinstance(export_value, dict) else None
    if not isinstance(data, list) or not data or not any(
        isinstance(unit, dict) and isinstance(unit.get('files'), list) and unit['files']
        for unit in data
    ):
        raise CoverageError('llvm-cov export contains no file coverage records')
    export_report.write_text(json.dumps(export_value, indent=2, sort_keys=True) + '\n')

    report_paths = {'text': str(text_report.resolve()), 'export': str(export_report.resolve())}
    result: dict[str, Any] = {
        'schema_version': 1,
        'selected_build': str(build),
        'imported_extension': str(extension),
        'native_test_count': native_test_count,
        'profile_count': len(profiles),
        'profile_directory': str(profile_directory.resolve()),
        'merged_profile': str(merged_profile.resolve()),
        'object_path': str(extension),
        'report_object': str(extension),
        'tool_versions': tool_versions,
        'report_paths': report_paths,
        'test_provenance': str((run_directory / 'test_native_provenance.json').resolve()),
    }
    provenance_path = run_directory / 'coverage_provenance.json'
    temporary_path = provenance_path.with_name(f'{provenance_path.name}.tmp')
    temporary_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    os.replace(temporary_path, provenance_path)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--llvm-profdata', required=True)
    parser.add_argument('--llvm-cov', required=True)
    arguments = parser.parse_args(argv)
    try:
        result = finalize_coverage(
            build_value=arguments.build,
            run_directory_value=arguments.run_dir,
            llvm_profdata_value=arguments.llvm_profdata,
            llvm_cov_value=arguments.llvm_cov,
        )
    except (CoverageError, OSError) as error:
        print(f'native coverage failed: {error}', file=sys.stderr)
        return 1
    report_path = Path(result['report_paths']['text'])
    print(report_path.read_text(), end='')
    print(f"coverage provenance: {Path(arguments.run_dir).resolve() / 'coverage_provenance.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
