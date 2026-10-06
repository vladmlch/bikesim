from __future__ import annotations

import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

import pytest

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
    context = {
        "schema_version": 1,
        "source_roots": [str(source_root.resolve())],
        "compiler": {"path": sys.executable, "id": "fixture", "version": "1"},
        "sdk": str(tmp_path.resolve()),
        "include_paths": [str(path.resolve()) for path in include_paths or []],
        "library_paths": [],
        "required_files": [],
        "python_executable": sys.executable,
        "tool_defaults": {
            "tidy": "clang-tidy",
            "analyzer": "clang++",
            "cppcheck": "cppcheck",
            "gxx": "g++-16",
        },
        "tool_overrides": {},
        "target_contexts": {
            target: {
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
