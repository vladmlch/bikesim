"""Coverage provenance and native structural-contract controls."""

from __future__ import annotations

import importlib.machinery
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
COVERAGE_TOOL = REPO_ROOT / 'tools' / 'native_coverage.py'
CONTRACT_TOOL = REPO_ROOT / 'tools' / 'check_native_contracts.py'


def _run_uv_python(script: Path, arguments: list[str], environment: dict[str, str]) -> subprocess.CompletedProcess[str]:
    child_environment = environment.copy()
    child_environment.setdefault('UV_CACHE_DIR', '/private/tmp/cpp-port-v4-uv')
    return subprocess.run(
        ['uv', 'run', 'python', str(script), *arguments],
        cwd=REPO_ROOT,
        env=child_environment,
        text=True,
        capture_output=True,
        check=False,
    )


def _write_fake_llvm_tools(directory: Path) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    profdata = directory / 'fake-llvm-profdata'
    profdata.write_text(
        """#!/bin/sh
set -eu
if [ "$1" = "--version" ]; then
  printf 'fake llvm-profdata 1\\n'
  exit 0
fi
if [ "$1" = "merge" ]; then
  shift
  output=''
  while [ "$#" -gt 0 ]; do
    if [ "$1" = "-o" ]; then
      shift
      output="$1"
    fi
    shift
  done
  [ -n "$output" ] || exit 4
  printf 'merged profile\\n' > "$output"
  exit 0
fi
exit 5
"""
    )
    profdata.chmod(0o755)

    cov = directory / 'fake-llvm-cov'
    cov.write_text(
        """#!/bin/sh
set -eu
if [ "$1" = "--version" ]; then
  printf 'fake llvm-cov 1\\n'
  exit 0
fi
if [ "$1" = "report" ]; then
  printf 'Filename Regions Missed Regions Cover Functions Missed Functions Cover Lines Missed Lines Cover Branches Missed Branches Cover\\n'
  printf '%s 2 0 100%% 1 0 100%% 4 1 75%% 2 1 50%%\\n' "$2"
  printf 'TOTAL 2 0 100%% 1 0 100%% 4 1 75%% 2 1 50%%\\n'
  exit 0
fi
if [ "$1" = "export" ]; then
  printf '{"data":[{"files":[{"filename":"coverage-control.cpp","summary":{"lines":{"count":4,"covered":3},"branches":{"count":2,"covered":1}}}]}]}\\n'
  exit 0
fi
exit 5
"""
    )
    cov.chmod(0o755)
    return profdata, cov


def _write_coverage_fixture(
    tmp_path: Path,
    *,
    extension_count: int = 1,
    profile_count: int = 1,
    imported_extension: Path | None = None,
    native_test_count: int | None = 5,
) -> tuple[Path, Path, list[Path]]:
    build = tmp_path / 'custom coverage build'
    build.mkdir()
    (build / 'CMakeCache.txt').write_text('NATIVE_COVERAGE:BOOL=ON\n')
    suffixes = list(dict.fromkeys(importlib.machinery.EXTENSION_SUFFIXES))
    if extension_count > len(suffixes):
        raise ValueError('test requested more suffixes than this Python provides')
    extensions = [build / f'bike_native{suffix}' for suffix in suffixes[:extension_count]]
    for extension in extensions:
        extension.write_bytes(b'extension fixture; never import')

    run_directory = build / 'native_coverage_runs' / 'run-control'
    profile_directory = run_directory / 'profiles'
    profile_directory.mkdir(parents=True)
    profiles = [profile_directory / f'{index}.profraw' for index in range(profile_count)]
    for profile in profiles:
        profile.write_bytes(b'profile fixture')

    provenance: dict[str, object] = {
        'schema_version': 1,
        'selected_build': str(build.resolve()),
        'imported_extension': str(
            imported_extension.resolve() if imported_extension is not None else extensions[0].resolve()
        ) if extensions else str(imported_extension or (build / 'missing.so')),
    }
    if native_test_count is not None:
        provenance['native_test_count'] = native_test_count
    (run_directory / 'test_native_provenance.json').write_text(json.dumps(provenance))
    return build, run_directory, profiles


def _run_coverage_control(
    build: Path,
    run_directory: Path,
    fake_tools: tuple[Path, Path],
) -> subprocess.CompletedProcess[str]:
    profdata, cov = fake_tools
    return _run_uv_python(
        COVERAGE_TOOL,
        [
            '--build',
            str(build),
            '--run-dir',
            str(run_directory),
            '--llvm-profdata',
            str(profdata),
            '--llvm-cov',
            str(cov),
        ],
        os.environ.copy(),
    )


def test_coverage_provenance_supports_a_custom_build_and_exact_report_object(tmp_path: Path) -> None:
    build, run_directory, _ = _write_coverage_fixture(tmp_path)
    fake_tools = _write_fake_llvm_tools(tmp_path / 'fake-llvm')

    result = _run_coverage_control(build, run_directory, fake_tools)

    assert result.returncode == 0, result.stderr
    provenance = json.loads((run_directory / 'coverage_provenance.json').read_text())
    extension = next(
        path
        for path in build.glob('bike_native*')
        if any(path.name.endswith(suffix) for suffix in importlib.machinery.EXTENSION_SUFFIXES)
    )
    assert Path(provenance['selected_build']).resolve() == build.resolve()
    assert Path(provenance['imported_extension']).resolve() == extension.resolve()
    assert Path(provenance['report_object']).resolve() == extension.resolve()
    assert provenance['profile_count'] == 1
    assert provenance['native_test_count'] == 5
    assert Path(provenance['report_paths']['text']).is_file()
    assert Path(provenance['report_paths']['export']).is_file()
    assert 'TOTAL' in Path(provenance['report_paths']['text']).read_text()


@pytest.mark.parametrize('extension_count', [0, 2])
def test_coverage_requires_exactly_one_extension(tmp_path: Path, extension_count: int) -> None:
    build, run_directory, _ = _write_coverage_fixture(tmp_path, extension_count=extension_count)
    fake_tools = _write_fake_llvm_tools(tmp_path / 'fake-llvm')

    result = _run_coverage_control(build, run_directory, fake_tools)

    assert result.returncode != 0
    assert 'expected exactly one bike_native extension' in result.stderr


def test_coverage_requires_profiles_from_this_run(tmp_path: Path) -> None:
    build, run_directory, _ = _write_coverage_fixture(tmp_path, profile_count=0)
    fake_tools = _write_fake_llvm_tools(tmp_path / 'fake-llvm')

    result = _run_coverage_control(build, run_directory, fake_tools)

    assert result.returncode != 0
    assert 'no nonempty profraw files' in result.stderr


def test_coverage_rejects_a_different_imported_binary(tmp_path: Path) -> None:
    wrong_build = tmp_path / 'wrong-build'
    wrong_build.mkdir()
    wrong_extension = wrong_build / f'bike_native{importlib.machinery.EXTENSION_SUFFIXES[0]}'
    wrong_extension.write_bytes(b'wrong extension; never import')
    build, run_directory, _ = _write_coverage_fixture(
        tmp_path,
        imported_extension=wrong_extension,
    )
    fake_tools = _write_fake_llvm_tools(tmp_path / 'fake-llvm')

    result = _run_coverage_control(build, run_directory, fake_tools)

    assert result.returncode != 0
    assert 'imported extension does not match selected build artifact' in result.stderr


def test_coverage_requires_a_collected_native_test_count(tmp_path: Path) -> None:
    build, run_directory, _ = _write_coverage_fixture(tmp_path, native_test_count=None)
    fake_tools = _write_fake_llvm_tools(tmp_path / 'fake-llvm')

    result = _run_coverage_control(build, run_directory, fake_tools)

    assert result.returncode != 0
    assert 'native_test_count' in result.stderr


def _write_contract_fixture(tmp_path: Path) -> dict[str, Path]:
    repository = tmp_path / 'repo'
    native = repository / 'native'
    source_root = native / 'src'
    test_root = native / 'tests'
    reference_root = repository / 'tests' / 'reference'
    source_root.mkdir(parents=True)
    test_root.mkdir()
    reference_root.mkdir(parents=True)
    (source_root / 'one.cpp').write_text('int native_source() { return 1; }\n')
    contract_source = test_root / 'test_contracts.cpp'
    contract_source.write_text('int contract_source() { return 2; }\n')
    (native / 'analysis_suppressions.json').write_text(
        json.dumps({'schema_version': 1, 'suppressions': []})
    )
    (native / '.clang-tidy').write_text(
        "Checks: 'bugprone-*,clang-analyzer-cplusplus.Move'\n"
        "HeaderFilterRegex: '(^|/)(native/(src|tests)|tools/proto_native_bench)/'\n"
    )
    (reference_root / 'test_native_probe.py').write_text(
        'from native_loader import load_native\n'
        'bike_native = load_native()\n'
    )

    build = tmp_path / 'build'
    build.mkdir()
    compiler = Path(sys.executable).resolve()
    sdk = tmp_path.resolve()
    hardening_definition = '_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE'
    source_entries = [
        {
            'path': str((source_root / 'one.cpp').resolve()),
            'target': 'bike_native',
            'context': 'bike_native:fixture',
        },
        {
            'path': str(contract_source.resolve()),
            'target': 'native_contract_tests',
            'context': 'native_contract_tests:fixture',
        },
    ]
    (build / 'native_sources.json').write_text(
        json.dumps(
            {
                'schema_version': 1,
                'first_party_roots': [str(native.resolve())],
                'sources': source_entries,
            }
        )
    )

    target_contexts = {
        target: {
            'configuration': 'fixture',
            'cxx_standard': 23,
            'cxx_extensions': False,
            'include_directories': [],
            'compile_definitions': [hardening_definition],
            'compile_options': [],
            'link_directories': [],
            'link_libraries': [],
        }
        for target in ('bike_native', 'native_contract_tests')
    }
    context = {
        'schema_version': 1,
        'compiler': {'path': str(compiler), 'id': 'fixture', 'version': '1', 'launcher': []},
        'sdk': str(sdk),
        'sdk_requested': str(sdk),
        'sdk_source': 'explicit',
        'libcpp_hardening': {
            'supported': True,
            'mode': 'EXTENSIVE',
            'requested_mode': 'EXTENSIVE',
        },
        'analysis_suppressions_manifest': str((native / 'analysis_suppressions.json').resolve()),
        'include_paths': [],
        'library_paths': [],
        'required_files': [],
        'source_roots': [str(native.resolve())],
        'python_executable': str(compiler),
        'tool_defaults': {},
        'tool_overrides': {},
        'target_contexts': target_contexts,
    }
    (build / 'native_check_context.json').write_text(json.dumps(context))
    compile_entries = []
    for source, target in (
        (source_root / 'one.cpp', 'bike_native'),
        (contract_source, 'native_contract_tests'),
    ):
        output = f'CMakeFiles/{target}.dir/{source.name}.o'
        compile_entries.append(
            {
                'directory': str(tmp_path.resolve()),
                'file': str(source.resolve()),
                'output': output,
                'arguments': [
                    str(compiler),
                    '-std=c++23',
                    '-isysroot',
                    str(sdk),
                    f'-D{hardening_definition}',
                    '-c',
                    str(source.resolve()),
                    '-o',
                    output,
                ],
            }
        )
    (build / 'compile_commands.json').write_text(json.dumps(compile_entries))
    (build / 'CMakeCache.txt').write_text('NATIVE_COVERAGE:BOOL=OFF\n')
    (build / 'CTestTestfile.cmake').write_text(
        'add_test([=[native_contract_tests]=] "native_contract_tests")\n'
    )
    contract_binary = build / 'native_contract_tests'
    contract_binary.write_text(
        "#!/bin/sh\n"
        "if [ \"${1:-}\" = \"--list\" ]; then printf 'human_crank_torque\\npedaling_policy_valid_transition\\n'; fi\n"
    )
    contract_binary.chmod(0o755)
    extension = build / f'bike_native{importlib.machinery.EXTENSION_SUFFIXES[0]}'
    extension.write_bytes(b'fixture extension; checker must not import it')
    provenance = build / 'native_test_provenance.json'
    provenance.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'selected_build': str(build.resolve()),
                'imported_extension': str(extension.resolve()),
                'native_test_count': 1,
            }
        )
    )
    return {'repository': repository, 'build': build, 'extension': extension, 'contract_source': contract_source}


def _run_contract_checker(fixture: dict[str, Path]) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment['NATIVE_TEST_BUILD_PATH'] = str(fixture['build'].resolve())
    environment['NATIVE_TEST_PROVENANCE_PATH'] = str(
        (fixture['build'] / 'native_test_provenance.json').resolve()
    )
    return _run_uv_python(
        CONTRACT_TOOL,
        ['--build', str(fixture['build']), '--repo-root', str(fixture['repository'])],
        environment,
    )


def test_contract_checker_requires_manifested_ctest_contract_cases(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    ctest_file = fixture['build'] / 'CTestTestfile.cmake'
    ctest_file.write_text('# no contract case registered\n')

    result = _run_contract_checker(fixture)

    assert result.returncode != 0
    assert 'native_contract_tests is not registered with CTest' in result.stderr


def test_contract_checker_rejects_empty_native_contract_case_registry(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    executable = fixture['build'] / 'native_contract_tests'
    executable.write_text('#!/bin/sh\nexit 0\n')
    executable.chmod(0o755)

    result = _run_contract_checker(fixture)

    assert result.returncode != 0
    assert 'lists no cases' in result.stderr


def test_contract_checker_marks_future_engine_and_benchmark_rules_pending(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)

    result = _run_contract_checker(fixture)

    assert result.returncode == 0, result.stderr
    summary = json.loads((fixture['build'] / 'native_check_summaries' / 'contracts.json').read_text())
    assert summary['status'] == 'passed'
    assert summary['checks']['engine_call_wrappers']['status'] == 'pending'
    assert summary['checks']['benchmark_manifest_membership']['status'] == 'pending'


def test_contract_checker_rejects_missing_configured_hardening_definition(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    hardening_definition = '_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE'

    context_path = fixture['build'] / 'native_check_context.json'
    context = json.loads(context_path.read_text())
    for target in context['target_contexts'].values():
        target['compile_definitions'].remove(hardening_definition)
    context_path.write_text(json.dumps(context))

    commands_path = fixture['build'] / 'compile_commands.json'
    commands = json.loads(commands_path.read_text())
    for command in commands:
        command['arguments'].remove(f'-D{hardening_definition}')
    commands_path.write_text(json.dumps(commands))

    result = _run_contract_checker(fixture)

    assert result.returncode != 0
    assert 'missing libc++ hardening definition' in result.stderr


def test_contract_checker_rejects_disabled_move_analysis(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    (fixture['repository'] / 'native' / '.clang-tidy').write_text(
        "Checks: 'bugprone-*,-clang-analyzer-cplusplus.Move'\n"
        "HeaderFilterRegex: '(^|/)(native/(src|tests)|tools/proto_native_bench)/'\n"
    )

    result = _run_contract_checker(fixture)

    assert result.returncode != 0
    assert 'must enable first-party clang-analyzer-cplusplus.Move' in result.stderr


def test_contract_checker_rejects_native_tests_without_selected_loader(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    (fixture['repository'] / 'tests' / 'reference' / 'test_native_probe.py').write_text(
        'import bike_native\n'
    )

    result = _run_contract_checker(fixture)

    assert result.returncode != 0
    assert 'must use the selected-artifact loader' in result.stderr


def test_contract_checker_accepts_multiple_target_contexts_for_one_source(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    build = fixture['build']
    source = (fixture['repository'] / 'native' / 'src' / 'one.cpp').resolve()

    source_manifest_path = build / 'native_sources.json'
    source_manifest = json.loads(source_manifest_path.read_text())
    source_manifest['sources'].append(
        {
            'path': str(source),
            'target': 'native_analysis_probe',
            'context': 'native_analysis_probe:fixture',
        }
    )
    source_manifest_path.write_text(json.dumps(source_manifest))

    context_path = build / 'native_check_context.json'
    context = json.loads(context_path.read_text())
    context['target_contexts']['native_analysis_probe'] = dict(
        context['target_contexts']['bike_native']
    )
    context_path.write_text(json.dumps(context))

    compile_commands_path = build / 'compile_commands.json'
    compile_commands = json.loads(compile_commands_path.read_text())
    bike_entry = next(entry for entry in compile_commands if Path(entry['file']).resolve() == source)
    output = 'CMakeFiles/native_analysis_probe.dir/one.cpp.o'
    arguments = list(bike_entry['arguments'])
    arguments[-1] = output
    compile_commands.append(
        {
            **bike_entry,
            'output': output,
            'arguments': arguments,
        }
    )
    compile_commands_path.write_text(json.dumps(compile_commands))

    result = _run_contract_checker(fixture)

    assert result.returncode == 0, result.stderr
    summary = json.loads((build / 'native_check_summaries' / 'contracts.json').read_text())
    assert summary['checks']['manifest_selection']['source_count'] == 2
    assert summary['checks']['manifest_selection']['translation_unit_context_count'] == 3


def test_contract_checker_enforces_manifest_membership_for_native_bench_target(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    build = fixture['build']
    repository = fixture['repository']
    benchmark_root = repository / 'tools' / 'proto_native_bench'
    benchmark_root.mkdir(parents=True)
    benchmark_source = benchmark_root / 'bench.cpp'
    benchmark_source.write_text('int benchmark_source() { return 3; }\n')

    source_manifest_path = build / 'native_sources.json'
    source_manifest = json.loads(source_manifest_path.read_text())
    source_manifest['first_party_roots'].append(str(benchmark_root.resolve()))
    source_manifest['sources'].append(
        {
            'path': str(benchmark_source.resolve()),
            'target': 'native_bench',
            'context': 'native_bench:fixture',
        }
    )
    source_manifest_path.write_text(json.dumps(source_manifest))

    context_path = build / 'native_check_context.json'
    context = json.loads(context_path.read_text())
    context['source_roots'].append(str(benchmark_root.resolve()))
    context['target_contexts']['native_bench'] = dict(context['target_contexts']['bike_native'])
    context_path.write_text(json.dumps(context))

    compile_commands_path = build / 'compile_commands.json'
    compile_commands = json.loads(compile_commands_path.read_text())
    bike_entry = next(entry for entry in compile_commands if Path(entry['file']).resolve().name == 'one.cpp')
    output = 'CMakeFiles/native_bench.dir/bench.cpp.o'
    arguments = list(bike_entry['arguments'])
    source_index = next(index for index, argument in enumerate(arguments) if argument.endswith('/one.cpp'))
    arguments[source_index] = str(benchmark_source.resolve())
    arguments[-1] = output
    compile_commands.append(
        {
            **bike_entry,
            'file': str(benchmark_source.resolve()),
            'output': output,
            'arguments': arguments,
        }
    )
    compile_commands_path.write_text(json.dumps(compile_commands))

    result = _run_contract_checker(fixture)

    assert result.returncode == 0, result.stderr
    summary = json.loads((build / 'native_check_summaries' / 'contracts.json').read_text())
    assert summary['checks']['benchmark_manifest_membership']['status'] == 'passed'


def test_contract_checker_parses_repository_folded_tidy_policy(tmp_path: Path) -> None:
    fixture = _write_contract_fixture(tmp_path)
    shutil.copy2(REPO_ROOT / 'native' / '.clang-tidy', fixture['repository'] / 'native' / '.clang-tidy')

    result = _run_contract_checker(fixture)

    assert result.returncode == 0, result.stderr


def test_real_one_file_llvm_coverage_report(tmp_path: Path) -> None:
    """Exercise real clang instrumentation, profile merging, and llvm-cov report output."""
    assert sys.platform == 'darwin', 'native coverage is supported on the Apple toolchain'
    context = json.loads((REPO_ROOT / 'native' / 'build' / 'native_check_context.json').read_text())
    compiler = Path(context['compiler']['path'])
    source = tmp_path / 'coverage_probe.cpp'
    source.write_text(
        'extern "C" int coverage_probe(int value) {\n'
        '  if (value > 0) return value + 1;\n'
        '  return -value;\n'
        '}\n'
    )

    build = tmp_path / 'instrumented-build'
    build.mkdir()
    (build / 'CMakeCache.txt').write_text('NATIVE_COVERAGE:BOOL=ON\n')
    extension = build / f'bike_native{importlib.machinery.EXTENSION_SUFFIXES[0]}'
    compile_library = subprocess.run(
        [
                str(compiler),
                '-std=c++23',
                '-isysroot',
                context['sdk'],
                '-dynamiclib',
            '-fPIC',
            '-fprofile-instr-generate',
            '-fcoverage-mapping',
            str(source),
            '-o',
            str(extension),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert compile_library.returncode == 0, compile_library.stderr

    driver = tmp_path / 'coverage_driver.cpp'
    driver.write_text(
        '#include <dlfcn.h>\n'
        'using CoverageFunction = int (*)(int);\n'
        'int main(int argc, char **argv) {\n'
        '  if (argc != 2) return 2;\n'
        '  void *handle = dlopen(argv[1], RTLD_NOW);\n'
        '  if (!handle) return 3;\n'
        '  auto function = reinterpret_cast<CoverageFunction>(dlsym(handle, "coverage_probe"));\n'
        '  if (!function) return 4;\n'
        '  const int result = function(3);\n'
        '  dlclose(handle);\n'
        '  return result == 4 ? 0 : 5;\n'
        '}\n'
    )
    executable = tmp_path / 'coverage_driver'
    compile_driver = subprocess.run(
        [str(compiler), '-std=c++23', '-isysroot', context['sdk'], str(driver), '-o', str(executable)],
        text=True,
        capture_output=True,
        check=False,
    )
    assert compile_driver.returncode == 0, compile_driver.stderr

    run_directory = build / 'native_coverage_runs' / 'real-one-file'
    profiles = run_directory / 'profiles'
    profiles.mkdir(parents=True)
    driver_environment = os.environ.copy()
    driver_environment['LLVM_PROFILE_FILE'] = str(profiles / '%p.profraw')
    run_driver = subprocess.run(
        [str(executable), str(extension)],
        env=driver_environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert run_driver.returncode == 0, run_driver.stderr

    (run_directory / 'test_native_provenance.json').write_text(
        json.dumps(
            {
                'schema_version': 1,
                'selected_build': str(build.resolve()),
                'imported_extension': str(extension.resolve()),
                'native_test_count': 1,
            }
        )
    )
    profdata = subprocess.run(
        ['xcrun', '--find', 'llvm-profdata'], text=True, capture_output=True, check=False
    )
    cov = subprocess.run(
        ['xcrun', '--find', 'llvm-cov'], text=True, capture_output=True, check=False
    )
    assert profdata.returncode == 0 and Path(profdata.stdout.strip()).is_file(), profdata.stderr
    assert cov.returncode == 0 and Path(cov.stdout.strip()).is_file(), cov.stderr
    result = _run_uv_python(
        COVERAGE_TOOL,
        [
            '--build',
            str(build),
            '--run-dir',
            str(run_directory),
            '--llvm-profdata',
            profdata.stdout.strip(),
            '--llvm-cov',
            cov.stdout.strip(),
        ],
        os.environ.copy(),
    )

    assert result.returncode == 0, result.stderr
    provenance = json.loads((run_directory / 'coverage_provenance.json').read_text())
    report = Path(provenance['report_paths']['text'])
    assert 'coverage_probe.cpp' in report.read_text()
    assert 'TOTAL' in report.read_text()
    exported = json.loads(Path(provenance['report_paths']['export']).read_text())
    file_coverage = next(
        item
        for item in exported['data'][0]['files']
        if item['filename'].endswith('coverage_probe.cpp')
    )
    lines = file_coverage['summary']['lines']
    branches = file_coverage['summary']['branches']
    assert 0 < lines['covered'] < lines['count']
    assert 0 < branches['covered'] < branches['count']
