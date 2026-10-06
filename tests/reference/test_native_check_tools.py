from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

import pytest

import tools.native_checks as native_checks
from tools.native_checks import load_context, load_entries, run_sweep


def write_fixture(
    tmp_path: Path,
    *,
    sources: list[tuple[Path, str, str]] | None = None,
    entries: list[dict[str, object]] | None = None,
    include_paths: list[Path] | None = None,
) -> tuple[Path, set[Path]]:
    source_root = tmp_path / "native" / "src"
    source_root.mkdir(parents=True, exist_ok=True)
    build = tmp_path / "build"
    build.mkdir(exist_ok=True)
    source = source_root / "one.cpp"
    source.write_text("int first_party() { return 0; }\n")

    manifest_sources = sources or [(source, "bike_native", "default")]
    expected = {item[0].resolve() for item in manifest_sources}
    (build / "native_sources.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "first_party_roots": [str(source_root.resolve())],
                "sources": [
                    {"path": str(path.resolve()), "target": target, "context": context}
                    for path, target, context in manifest_sources
                ],
            }
        )
    )
    suppression_manifest = source_root / "analysis_suppressions.json"
    suppression_manifest.write_text(json.dumps({"schema_version": 1, "suppressions": []}))
    context = {
        "schema_version": 1,
        "source_roots": [str(source_root.resolve())],
        "compiler": {"path": sys.executable, "id": "fixture", "version": "1"},
        "sdk": str(tmp_path.resolve()),
        "sdk_requested": str(tmp_path.resolve()),
        "sdk_source": "explicit",
        "libcpp_hardening": {
            "supported": True,
            "mode": "EXTENSIVE",
            "requested_mode": "EXTENSIVE",
        },
        "include_paths": [str(path.resolve()) for path in include_paths or []],
        "library_paths": [],
        "required_files": [],
        "python_executable": sys.executable,
        "analysis_suppressions_manifest": str(suppression_manifest.resolve()),
        "tool_defaults": {
            "tidy": "clang-tidy",
            "analyzer": "clang++",
            "cppcheck": "cppcheck",
            "gxx": "g++-16",
        },
        "tool_overrides": {},
        "target_contexts": {
            target: {
                "cxx_standard": 23,
                "cxx_extensions": False,
                "include_directories": [],
                "compile_definitions": [],
                "compile_options": [],
            }
            for target in {item[1] for item in manifest_sources}
        },
    }
    (build / "native_check_context.json").write_text(json.dumps(context))

    if entries is None:
        entries = [compile_entry(source, tmp_path, arguments=True)]
    (build / "compile_commands.json").write_text(json.dumps(entries))
    return build, expected


def compile_entry(
    source: Path,
    directory: Path,
    *,
    arguments: bool,
    relative_file: bool = False,
    extra: list[str] | None = None,
    target: str = "bike_native",
) -> dict[str, object]:
    source_arg = os.path.relpath(source, directory) if relative_file else str(source)
    output = f"CMakeFiles/{target}.dir/{source.name}.o"
    argv = [
        sys.executable,
        "-std=c++23",
        "-isysroot",
        str(directory),
        *(extra or []),
        "-c",
        source_arg,
        "-o",
        output,
    ]
    entry: dict[str, object] = {
        "directory": str(directory),
        "file": source_arg,
        "output": output,
    }
    if arguments:
        entry["arguments"] = argv
    else:
        entry["command"] = shlex.join(argv)
    return entry


@pytest.mark.parametrize("arguments", [True, False], ids=["arguments", "command"])
def test_load_entries_accepts_both_compilation_database_forms(
    tmp_path: Path, arguments: bool
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    build, expected = write_fixture(
        tmp_path,
        entries=[compile_entry(source, tmp_path, arguments=arguments)],
    )

    selected = load_entries(build, expected)

    assert len(selected) == 1
    assert selected[0]["source"] == source.resolve()


def test_load_entries_resolves_relative_file_from_entry_directory(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    build, expected = write_fixture(
        tmp_path,
        entries=[
            compile_entry(
                source,
                tmp_path,
                arguments=True,
                relative_file=True,
            )
        ],
    )

    selected = load_entries(build, expected)

    assert selected[0]["source"] == source.resolve()


def test_duplicate_identical_translation_units_fail_selection(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True)
    build, expected = write_fixture(tmp_path, entries=[entry, entry.copy()])

    with pytest.raises(ValueError, match="duplicate.*translation unit"):
        load_entries(build, expected)


def test_shared_source_with_distinct_target_contexts_is_selected_twice(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text("int first_party() { return 0; }\n")
    variants = [
        compile_entry(
            source,
            tmp_path,
            arguments=True,
            extra=["-DVARIANT=1"],
            target="bike_native",
        ),
        compile_entry(
            source,
            tmp_path,
            arguments=True,
            extra=["-DVARIANT=2"],
            target="native_contract_tests",
        ),
    ]
    build, expected = write_fixture(
        tmp_path,
        sources=[
            (source, "bike_native", "default"),
            (source, "native_contract_tests", "default"),
        ],
        entries=variants,
    )

    selected = load_entries(build, expected)

    assert len(selected) == 2
    assert {entry["source"] for entry in selected} == {source.resolve()}


@pytest.mark.parametrize("include_flag", ["-I", "-isystem", "-iquote", "-idirafter"])
def test_target_context_accepts_joined_include_flags(tmp_path: Path, include_flag: str) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    include_directory = tmp_path / "target-include"
    include_directory.mkdir()
    entry = compile_entry(
        source,
        tmp_path,
        arguments=True,
        extra=[f"{include_flag}{include_directory}"],
    )
    build, expected = write_fixture(tmp_path, entries=[entry])
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["target_contexts"]["bike_native"]["include_directories"] = [
        str(include_directory.resolve())
    ]
    context_path.write_text(json.dumps(context))

    selected = load_entries(build, expected)

    assert len(selected) == 1
    assert selected[0]["source"] == source.resolve()


def test_empty_compilation_database_fails_for_expected_sources(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    build, expected = write_fixture(tmp_path, entries=[])

    with pytest.raises(ValueError, match="empty.*selection|missing.*one.cpp"):
        load_entries(build, expected)


def test_missing_manifest_source_file_fails_before_tool_selection(tmp_path: Path) -> None:
    missing = tmp_path / "native" / "src" / "missing.cpp"
    build, expected = write_fixture(
        tmp_path,
        sources=[(missing, "bike_native", "default")],
        entries=[],
    )

    with pytest.raises(ValueError, match="missing source file.*missing.cpp"):
        load_entries(build, expected)


def test_unexpected_first_party_translation_unit_fails_selection(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    unexpected = source.parent / "unexpected.cpp"
    source.parent.mkdir(parents=True, exist_ok=True)
    unexpected.write_text("int unexpected() { return 0; }\n")
    build, expected = write_fixture(
        tmp_path,
        entries=[
            compile_entry(source, tmp_path, arguments=True),
            compile_entry(unexpected, tmp_path, arguments=True),
        ],
    )

    with pytest.raises(ValueError, match="unexpected first-party.*unexpected.cpp"):
        load_entries(build, expected)


def test_unparseable_cmake_output_cannot_bypass_target_context_validation(
    tmp_path: Path,
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True)
    entry["output"] = "one.o"
    entry["arguments"][-1] = "one.o"
    build, expected = write_fixture(tmp_path, entries=[entry])

    with pytest.raises(ValueError, match="target context.*unavailable"):
        load_entries(build, expected)


def test_missing_dependency_include_path_fails_selection(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    missing_include = tmp_path / "missing-include"
    build, expected = write_fixture(
        tmp_path,
        entries=[
            compile_entry(
                source,
                tmp_path,
                arguments=True,
                extra=["-I", str(missing_include)],
            )
        ],
    )

    with pytest.raises(ValueError, match="dependency path.*missing-include"):
        load_entries(build, expected)


def test_compile_database_sysroot_must_match_configured_context(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    mismatched_sysroot = tmp_path / "other-sdk"
    mismatched_sysroot.mkdir()
    entry = compile_entry(
        source,
        tmp_path,
        arguments=True,
        extra=["-isysroot", str(mismatched_sysroot)],
    )
    build, expected = write_fixture(tmp_path, entries=[entry])

    with pytest.raises(ValueError, match="sysroot.*configured SDK"):
        load_entries(build, expected)


def test_compile_database_compiler_must_match_configured_context(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True)
    entry["arguments"][0] = "/bin/sh"
    build, expected = write_fixture(tmp_path, entries=[entry])

    with pytest.raises(ValueError, match="compiler differs from configured"):
        load_entries(build, expected)


def test_context_include_paths_must_be_directories(tmp_path: Path) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    build, _ = write_fixture(tmp_path, include_paths=[source])

    with pytest.raises(ValueError, match="include_paths entry.*not a directory"):
        load_context(build)


def test_context_library_path_directories_must_be_frameworks(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path)
    library_directory = tmp_path / "ordinary-library-directory"
    library_directory.mkdir()
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["library_paths"] = [str(library_directory)]
    context_path.write_text(json.dumps(context))

    with pytest.raises(ValueError, match="library_paths entry.*framework directory"):
        load_context(build)


def test_context_preserves_cpp_driver_symlink_spelling(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path)
    driver = tmp_path / 'clang++'
    driver.symlink_to(sys.executable)
    context_path = build / 'native_check_context.json'
    context = json.loads(context_path.read_text())
    context['compiler']['path'] = str(driver)
    context_path.write_text(json.dumps(context))

    assert load_context(build)['compiler']['path'] == str(driver)


def test_missing_build_directory_fails_before_selection(tmp_path: Path) -> None:
    missing_build = tmp_path / "not-configured"

    assert run_sweep("tidy", missing_build) == 1


def test_tidy_wrapper_routes_missing_build_to_fail_closed_controller(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    wrapper = repository / "tools" / "check_native_tidy.sh"
    missing_build = tmp_path / "not-configured"

    completed = subprocess.run(
        ["bash", str(wrapper), str(missing_build), str(repository / "native" / "src")],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode != 0
    assert "build directory does not exist" in completed.stderr


def write_fake_tool(path: Path, *, stable_rc: int = 0, stable_text: str = "", alpha_rc: int = 0, alpha_text: str = "") -> None:
    def shell_value(value: str) -> str:
        return shlex.quote(value)

    path.write_text(
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--version" ]; then echo "mock analysis tool 1"; exit 0; fi\n'
        'case "$*" in\n'
        '  *-analyzer-checker=alpha*)\n'
        f'    printf "%s\\n" {shell_value(alpha_text)} >&2; exit {alpha_rc} ;;\n'
        '  *)\n'
        f'    printf "%s\\n" {shell_value(stable_text)} >&2; exit {stable_rc} ;;\n'
        'esac\n'
    )
    path.chmod(0o755)


def test_stable_sweep_fails_closed_on_tool_health_and_diagnostics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cases = [
        (127, "tool not found", "none"),
        (2, "error: analysis failed", "blocking"),
        (139, "compiler crash", "none"),
        (0, "warning: unsafe operation", "blocking"),
    ]
    for index, (returncode, diagnostic, finding_status) in enumerate(cases):
        case_root = tmp_path / str(index)
        build, _ = write_fixture(case_root)
        tool = case_root / "mock-tidy"
        write_fake_tool(tool, stable_rc=returncode, stable_text=diagnostic)
        monkeypatch.setenv("CLANG_TIDY", str(tool))

        assert run_sweep("tidy", build) == 1
        summary = json.loads(
            (build / "native_check_summaries" / "tidy.json").read_text()
        )
        assert summary["tool_health"] == ("failed" if returncode != 0 else "ok")
        assert summary["finding_status"] == finding_status


def test_alpha_warning_is_report_only_but_alpha_crash_fails_health(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    report_root = tmp_path / "report-only"
    report_build, _ = write_fixture(report_root)
    report_tool = report_root / "mock-analyzer"
    write_fake_tool(
        report_tool,
        alpha_text="warning: experimental finding",
    )
    monkeypatch.setenv("ANALYZER_CLANG", str(report_tool))

    assert run_sweep("analyzer", report_build) == 0
    report_summary = json.loads(
        (report_build / "native_check_summaries" / "analyzer.json").read_text()
    )
    assert report_summary["tool_health"] == "ok"
    assert report_summary["finding_status"] == "report-only"
    assert report_summary["translation_unit_count"] == 1

    crash_root = tmp_path / "crash"
    crash_build, _ = write_fixture(crash_root)
    crash_tool = crash_root / "mock-analyzer"
    write_fake_tool(crash_tool, alpha_rc=139, alpha_text="segmentation fault")
    monkeypatch.setenv("ANALYZER_CLANG", str(crash_tool))

    assert run_sweep("analyzer", crash_build) == 1
    crash_summary = json.loads(
        (crash_build / "native_check_summaries" / "analyzer.json").read_text()
    )
    assert crash_summary["tool_health"] == "failed"


def test_positive_control_records_one_selected_translation_unit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build, expected = write_fixture(tmp_path)
    tool = tmp_path / "mock-tidy"
    write_fake_tool(tool)
    monkeypatch.setenv("CLANG_TIDY", str(tool))

    assert run_sweep("tidy", build) == 0
    summary = json.loads(
        (build / "native_check_summaries" / "tidy.json").read_text()
    )

    assert summary["tool_health"] == "ok"
    assert summary["finding_status"] == "none"
    assert summary["unique_source_count"] == 1
    assert summary["translation_unit_count"] == 1
    assert summary["selected_sources"] == [str(next(iter(expected)))]
    assert summary["records"][0]["returncode"] == 0


def test_context_tool_default_runs_without_environment_override(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build, _ = write_fixture(tmp_path)
    tool = tmp_path / "configured-tidy"
    write_fake_tool(tool)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["tool_defaults"]["tidy"] = str(tool)
    context_path.write_text(json.dumps(context))
    monkeypatch.delenv("CLANG_TIDY", raising=False)

    assert run_sweep("tidy", build) == 0


@pytest.mark.parametrize("kind", ["frontends", "odr"])
def test_gxx_context_default_runs_without_environment_override(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, kind: str
) -> None:
    build, _ = write_fixture(tmp_path)
    tool = tmp_path / "configured-gxx"
    write_fake_tool(tool)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["tool_defaults"]["gxx"] = str(tool)
    context_path.write_text(json.dumps(context))
    monkeypatch.delenv("GXX", raising=False)

    assert run_sweep(kind, build) == 0


def run_header_control(root: Path, *, diagnostic_in_header: bool = True) -> subprocess.CompletedProcess[str]:
    repository = Path(__file__).resolve().parents[2]
    source_dir = root / "native" / "src"
    source_dir.mkdir(parents=True, exist_ok=True)
    header = source_dir / "control.hpp"
    source = source_dir / "control.cpp"
    if diagnostic_in_header:
        header.write_text(
            "namespace control { inline double divide(int a, int b) { return a / b; } }\n"
        )
        source.write_text(
            '#include "control.hpp"\nint main() { return static_cast<int>(control::divide(1, 2)); }\n'
        )
    else:
        header.write_text("namespace control { inline int value() { return 0; } }\n")
        source.write_text(
            '#include "control.hpp"\ndouble divide(int a, int b) { return a / b; }\n'
            'int main() { return static_cast<int>(divide(1, 2) + control::value()); }\n'
        )

    configured_build = repository / "native" / "build"
    context = json.loads((configured_build / "native_check_context.json").read_text())
    database = json.loads((configured_build / "compile_commands.json").read_text())
    configured_source = repository / "native" / "src" / "binding.cpp"
    compile_entry = next(
        entry
        for entry in database
        if Path(entry["file"]).resolve() == configured_source.resolve()
    )
    argv = (
        list(compile_entry["arguments"])
        if "arguments" in compile_entry
        else shlex.split(compile_entry["command"])
    )
    object_path = "CMakeFiles/bike_native.dir/src/control.cpp.o"
    relocated_args: list[str] = []
    position = 0
    while position < len(argv):
        argument = argv[position]
        if argument == "-o" and position + 1 < len(argv):
            relocated_args.extend(["-o", object_path])
            position += 2
            continue
        if not argument.startswith("-"):
            try:
                if Path(argument).resolve() == configured_source.resolve():
                    relocated_args.append(str(source.resolve()))
                    position += 1
                    continue
            except OSError:
                pass
        relocated_args.append(argument)
        position += 1
    relocated_args.extend(["-I", str(source_dir.resolve())])

    build = root / "build"
    build.mkdir(parents=True, exist_ok=True)
    (build / "native_sources.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "first_party_roots": [str((root / "native").resolve())],
                "sources": [
                    {
                        "path": str(source.resolve()),
                        "target": "bike_native",
                        "context": "bike_native:relocated-header-control",
                    }
                ],
            }
        )
    )
    context["source_roots"] = [str((root / "native").resolve())]
    context["include_paths"] = [
        *context["include_paths"],
        str(source_dir.resolve()),
    ]
    (build / "native_check_context.json").write_text(json.dumps(context))
    (build / "compile_commands.json").write_text(
        json.dumps(
            [
                {
                    "directory": compile_entry["directory"],
                    "file": str(source.resolve()),
                    "output": object_path,
                    "arguments": relocated_args,
                }
            ]
        )
    )

    configured_tidy = re.search(
        r"^HeaderFilterRegex:\s*(.+)$",
        (repository / "native" / ".clang-tidy").read_text(),
        re.MULTILINE,
    )
    assert configured_tidy is not None
    (root / "native" / ".clang-tidy").write_text(
        "Checks: 'bugprone-integer-division'\n"
        f"HeaderFilterRegex: {configured_tidy.group(1)}\n"
    )

    environment = os.environ.copy()
    environment["CLANG_TIDY"] = (
        context["tool_overrides"].get("CLANG_TIDY")
        or context["tool_defaults"]["tidy"]
    )
    return subprocess.run(
        [
            "uv",
            "run",
            "python",
            str(repository / "tools" / "native_checks.py"),
            "--kind",
            "tidy",
            "--build",
            str(build),
        ],
        cwd=repository,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )


@pytest.mark.parametrize("root_name", ["original-root", "renamed-root"])
def test_header_filter_is_portable(tmp_path: Path, root_name: str) -> None:
    result = run_header_control(tmp_path / root_name)

    assert result.returncode != 0
    assert "bugprone-integer-division" in result.stdout + result.stderr


def test_header_filter_keeps_main_file_diagnostics(tmp_path: Path) -> None:
    result = run_header_control(tmp_path / "main-file-control", diagnostic_in_header=False)

    assert result.returncode != 0
    assert "bugprone-integer-division" in result.stdout + result.stderr


def configure_native(
    tmp_path: Path, *cmake_options: str
) -> tuple[subprocess.CompletedProcess[str], Path]:
    repository = Path(__file__).resolve().parents[2]
    build = tmp_path / "native-configure"
    command = [
        "uv",
        "run",
        "cmake",
        "-S",
        str(repository / "native"),
        "-B",
        str(build),
        *cmake_options,
    ]
    completed = subprocess.run(
        command,
        cwd=repository,
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )
    return completed, build


def test_explicit_sdk_alias_is_retained_and_canonicalized(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )
    current_sdk = Path(current_context["sdk"])
    explicit_alias = current_sdk.parent / "MacOSX26.5.sdk"
    assert explicit_alias.is_dir()

    configured, build = configure_native(
        tmp_path,
        f"-DCMAKE_OSX_SYSROOT={explicit_alias}",
        f"-DCMAKE_CXX_COMPILER={current_context['compiler']['path']}",
    )

    assert configured.returncode == 0, configured.stdout + configured.stderr
    context = json.loads((build / "native_check_context.json").read_text())
    assert context["sdk_requested"] == str(explicit_alias)
    assert Path(context["sdk"]).resolve() == explicit_alias.resolve()
    compilation_commands = json.loads((build / "compile_commands.json").read_text())
    selected_sysroots = []
    for entry in compilation_commands:
        if not Path(entry["file"]).resolve().is_relative_to(repository / "native"):
            continue
        argv = entry.get("arguments") or shlex.split(entry["command"])
        for index, argument in enumerate(argv[:-1]):
            if argument == "-isysroot":
                selected_sysroots.append(Path(argv[index + 1]).resolve())
    assert selected_sysroots
    assert set(selected_sysroots) == {explicit_alias.resolve()}


def test_missing_explicit_sdk_fails_without_default_fallback(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )
    missing_sdk = tmp_path / "missing-explicit-sdk"

    configured, _ = configure_native(
        tmp_path,
        f"-DCMAKE_OSX_SYSROOT={missing_sdk}",
        f"-DCMAKE_CXX_COMPILER={context['compiler']['path']}",
    )

    assert configured.returncode != 0
    assert str(missing_sdk) in configured.stdout + configured.stderr


def test_default_sdk_is_resolved_from_xcrun(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )
    xcrun = subprocess.run(
        ["xcrun", "--sdk", "macosx", "--show-sdk-path"],
        text=True,
        capture_output=True,
        check=True,
    )

    configured, build = configure_native(
        tmp_path,
    )

    assert configured.returncode == 0, configured.stdout + configured.stderr
    context = json.loads((build / "native_check_context.json").read_text())
    assert context["sdk_source"] == "default"
    assert Path(context["sdk"]).resolve() == Path(xcrun.stdout.strip()).resolve()
    assert Path(context["compiler"]["path"]).resolve() == Path(
        current_context["compiler"]["path"]
    ).resolve()


def test_empty_build_type_defaults_to_extensive_hardening(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )

    configured, build = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={current_context['compiler']['path']}",
        f"-DCMAKE_OSX_SYSROOT={current_context['sdk']}",
    )

    assert configured.returncode == 0, configured.stdout + configured.stderr
    context = json.loads((build / "native_check_context.json").read_text())
    assert context["libcpp_hardening"]["mode"] == "EXTENSIVE"
    assert context["libcpp_hardening"]["supported"] is True


def test_fast_hardening_is_available_as_an_explicit_comparison(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )

    configured, build = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={current_context['compiler']['path']}",
        f"-DCMAKE_OSX_SYSROOT={current_context['sdk']}",
        "-DBIKE_LIBCPP_HARDENING=FAST",
    )

    assert configured.returncode == 0, configured.stdout + configured.stderr
    context = json.loads((build / "native_check_context.json").read_text())
    assert context["libcpp_hardening"]["mode"] == "FAST"
    entries = json.loads((build / "compile_commands.json").read_text())
    native_entries = [
        entry
        for entry in entries
        if Path(entry["file"]).resolve().is_relative_to(repository / "native")
    ]
    assert native_entries
    assert all(
        "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST"
        in (entry.get("arguments") or shlex.split(entry["command"]))
        for entry in native_entries
    )


def test_first_party_compile_commands_use_iso_cpp23(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (repository / "native" / "build" / "native_check_context.json").read_text()
    )

    configured, build = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={current_context['compiler']['path']}",
        f"-DCMAKE_OSX_SYSROOT={current_context['sdk']}",
    )

    assert configured.returncode == 0, configured.stdout + configured.stderr
    entries = json.loads((build / "compile_commands.json").read_text())
    native_entries = [
        entry
        for entry in entries
        if Path(entry["file"]).resolve().is_relative_to(repository / "native")
    ]
    assert native_entries
    for entry in native_entries:
        argv = entry.get("arguments") or shlex.split(entry["command"])
        assert "-std=c++23" in argv
        assert "-std=gnu++23" not in argv


def test_unsupported_hardening_name_fails_configuration(tmp_path: Path) -> None:
    configured, _ = configure_native(tmp_path, "-DBIKE_LIBCPP_HARDENING=INVALID")

    assert configured.returncode != 0
    assert "BIKE_LIBCPP_HARDENING" in configured.stdout + configured.stderr


def run_controller_kind(kind: str, *, timeout: int = 180) -> subprocess.CompletedProcess[str]:
    repository = Path(__file__).resolve().parents[2]
    return subprocess.run(
        [
            "uv",
            "run",
            "python",
            str(repository / "tools" / "native_checks.py"),
            "--kind",
            kind,
            "--build",
            str(repository / "native" / "build"),
        ],
        cwd=repository,
        text=True,
        capture_output=True,
        check=False,
        timeout=timeout,
    )


def test_standalone_header_sweep_compiles_first_party_headers() -> None:
    result = run_controller_kind("headers", timeout=240)

    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads(
        (
            Path(__file__).resolve().parents[2]
            / "native"
            / "build"
            / "native_check_summaries"
            / "headers.json"
        ).read_text()
    )
    assert summary["tool_health"] == "ok"
    assert summary["finding_status"] == "none"
    assert summary["header_count"] > 0


def test_diagnostic_controls_cover_warning_families_and_move() -> None:
    result = run_controller_kind("diagnostic-controls", timeout=240)

    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads(
        (
            Path(__file__).resolve().parents[2]
            / "native"
            / "build"
            / "native_check_summaries"
            / "diagnostic-controls.json"
        ).read_text()
    )
    assert summary["tool_health"] == "ok"
    assert {control["name"] for control in summary["controls"]} == {
        "discarded_nodiscard",
        "bitwise_instead_of_logical",
        "extra_semicolon",
        "signed_bounds",
        "first_party_use_after_move",
        "libcpp_probe_rejects_missing_mode",
    }
    move = next(
        control for control in summary["controls"] if control["name"] == "first_party_use_after_move"
    )
    assert any("cplusplus.Move" in line for line in move["diagnostics"])
    assert move["suppressed_diagnostics"] == []


def test_context_control_records_sdk_hardening_and_timing_evidence() -> None:
    result = run_controller_kind("context", timeout=360)

    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads(
        (
            Path(__file__).resolve().parents[2]
            / "native"
            / "build"
            / "native_check_summaries"
            / "context.json"
        ).read_text()
    )
    assert summary["tool_health"] == "ok"
    assert summary["sdk_controls"]["explicit_sdk"] == "passed"
    assert summary["sdk_controls"]["missing_sdk"] == "rejected"
    assert summary["sdk_controls"]["default_sdk"] == "passed"
    assert {item["build_type"] for item in summary["hardening_evidence"]} == {
        "Debug",
        "Release",
        "default",
    }
    assert all(
        item["sizeof"]["EXTENSIVE"] == item["sizeof"]["FAST"]
        for item in summary["hardening_evidence"]
    )
    assert all(
        all(item["timings_ns"][mode] for mode in ("EXTENSIVE", "FAST"))
        for item in summary["hardening_evidence"]
    )
    assert all(item["assertion_active"]["EXTENSIVE"] for item in summary["hardening_evidence"])
    assert all("FAST" in item["assertion_active"] for item in summary["hardening_evidence"])
    assert all(
        item["assertion_evidence"]["EXTENSIVE"]["signal"] == "SIGTRAP"
        and item["assertion_evidence"]["EXTENSIVE"]["category"]
        == "libc++ std::vector element access"
        for item in summary["hardening_evidence"]
    )


def test_analyzer_suppression_manifest_has_complete_records() -> None:
    repository = Path(__file__).resolve().parents[2]
    manifest = json.loads(
        (repository / "native" / "analysis_suppressions.json").read_text()
    )

    assert manifest["schema_version"] == 1
    assert isinstance(manifest["suppressions"], list)
    required_fields = {
        "checker",
        "origin",
        "message",
        "tool_version",
        "reason",
        "reproducer",
        "remove_when",
    }
    assert all(required_fields <= suppression.keys() for suppression in manifest["suppressions"])


def test_suppression_requires_matching_checker_origin_message_and_tool_version() -> None:
    filter_diagnostics = getattr(native_checks, "apply_suppressions", None)
    assert callable(filter_diagnostics)
    suppression = {
        "checker": "cplusplus.Move",
        "origin": r"/nanobind/include/nanobind/stl/detail/nb_list\.h$",
        "message": "documented array-caster lifetime false positive",
        "tool_version": "23.1.2",
        "reason": "reproduced false positive in a third-party nanobind header",
        "reproducer": "tests/reference/test_native_check_tools.py::test_diagnostic_controls_cover_warning_families_and_move",
        "remove_when": "the installed analyzer no longer reports this diagnostic",
    }
    matching = (
        "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: "
        "warning: documented array-caster lifetime false positive [cplusplus.Move]"
    )
    first_party = (
        "/repo/native/src/binding.cpp:67:4: warning: documented array-caster lifetime false positive [cplusplus.Move]"
    )
    other_message = (
        "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: "
        "warning: another move diagnostic [cplusplus.Move]"
    )
    other_checker = (
        "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: "
        "warning: documented array-caster lifetime false positive [bugprone-use-after-move]"
    )

    remaining, suppressed = filter_diagnostics(
        [matching, first_party, other_message, other_checker],
        "Homebrew LLVM version 23.1.2",
        [suppression],
    )

    assert [item["diagnostic"] for item in suppressed] == [matching]
    assert remaining == [first_party, other_message, other_checker]
    version_mismatch, version_suppressed = filter_diagnostics(
        [matching], "Homebrew LLVM version 24.0.0", [suppression]
    )
    assert version_mismatch == [matching]
    assert version_suppressed == []


def test_clang_tidy_move_suppression_is_limited_to_nanobind_origin(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build, expected = write_fixture(tmp_path)
    source = next(iter(expected))
    third_party = (
        "/venv/lib/python3.14/site-packages/nanobind/include/nanobind/stl/detail/nb_array.h:34:17: "
        "warning: Method called on moved-from object 'value' of type 'std::array' "
        "[clang-analyzer-cplusplus.Move]"
    )
    first_party = (
        f"{source}:4:5: warning: Method called on moved-from object 'value' "
        "of type 'std::array' [clang-analyzer-cplusplus.Move]"
    )
    suppression_path = build.parent / "native" / "src" / "analysis_suppressions.json"
    suppression_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "suppressions": [
                    {
                        "checker": "clang-analyzer-cplusplus.Move",
                        "origin": r"(?:^|/)nanobind/include/nanobind/stl/detail/nb_array\.h$",
                        "message": "Method called on moved-from object 'value' of type 'std::array'",
                        "tool_version": "1",
                        "reason": "specific third-party false-positive control",
                        "reproducer": "test_clang_tidy_move_suppression_is_limited_to_nanobind_origin",
                        "remove_when": "the installed checker no longer emits this diagnostic",
                    }
                ],
            }
        )
    )
    fake_tidy = tmp_path / "mock-tidy"
    write_fake_tool(
        fake_tidy,
        stable_text=f"{third_party}\n{first_party}",
    )
    monkeypatch.setenv("CLANG_TIDY", str(fake_tidy))

    assert run_sweep("tidy", build) == 1
    summary = json.loads((build / "native_check_summaries" / "tidy.json").read_text())
    assert summary["suppression_counts"] == {"clang-analyzer-cplusplus.Move": 1}
    assert summary["records"][0]["diagnostics"] == [third_party, first_party]
    assert summary["records"][0]["unsuppressed_diagnostics"] == [first_party]
