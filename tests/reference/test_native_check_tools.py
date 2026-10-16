from __future__ import annotations

import json
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from native_loader import selected_build
import tools.native_checks as native_checks
import tools.native_toolchain as native_toolchain
from tools.native_checks import load_context, load_entries, run_sweep


def configured_build_path() -> Path:
    """The build directory this test run selected (NATIVE_TEST_BUILD_PATH)."""
    return selected_build(os.environ)


def _schema2_records(
    tmp_path: Path,
    *,
    platform_system: str,
    stdlib_name: str,
    sysroot: str | None,
) -> dict[str, object]:
    """The records schema 2 adds over schema 1, honest to the arguments."""
    stdlib_modes = {"libc++": "EXTENSIVE", "libstdc++": "ASSERTIONS"}
    if platform_system == "Darwin":
        sdk_fields: dict[str, object] = {
            "sdk": str(tmp_path.resolve()),
            "sdk_requested": str(tmp_path.resolve()),
            "sdk_source": "explicit",
            "sysroot": str(tmp_path.resolve()),
        }
    else:
        sdk_fields = {
            "sdk": None,
            "sdk_requested": None,
            "sdk_source": "not-applicable",
            "sysroot": sysroot,
        }
    return {
        "platform": {
            "system": platform_system,
            "host_system": platform_system,
            "machine": platform.machine(),
            "pointer_bits": 64,
        },
        "stdlib": {
            "name": stdlib_name,
            "version": "17",
            "hardening": {
                "mode": stdlib_modes[stdlib_name],
                "effective": True,
            },
        },
        "numerics": {"backend": "TEST_PROVIDER"},
        "policy": {
            "required_flags": ["-ffp-contract=off"],
            "accepted_warnings": ["-Wall"],
            "rejected_warnings": [
                {"flag": "-Wnot-a-warning", "reason": "probe failed"}
            ],
        },
        "capabilities": {
            "asan": {
                "requested": False,
                "supported": True,
                "enabled": False,
                "runtime": None,
                "compiler_id": "fixture",
                "target": None,
                "evidence": {"control": "probe"},
            },
            "rtsan": {
                "requested": False,
                "supported": False,
                "enabled": False,
                "runtime": None,
                "compiler_id": "fixture",
                "target": None,
                "evidence": {"control": "probe failed"},
            },
            "coverage": {
                "requested": False,
                "supported": True,
                "enabled": False,
                "runtime": None,
                "compiler_id": "fixture",
                "target": None,
                "evidence": {"control": "probe"},
            },
            "frontend_tools": {},
        },
        **sdk_fields,
    }


def write_fixture(
    tmp_path: Path,
    *,
    sources: list[tuple[Path, str, str]] | None = None,
    entries: list[dict[str, object]] | None = None,
    include_paths: list[Path] | None = None,
    schema_version: int = 1,
    platform_system: str | None = None,
    stdlib_name: str = "libc++",
    sysroot: str | None = None,
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
    libcpp_hardening = (
        {"supported": True, "mode": "EXTENSIVE", "requested_mode": "EXTENSIVE"}
        if stdlib_name == "libc++"
        else {"supported": False, "mode": "NOT_APPLICABLE", "requested_mode": "EXTENSIVE"}
    )
    context = {
        "schema_version": schema_version,
        "source_roots": [str(source_root.resolve())],
        "compiler": {"path": sys.executable, "id": "fixture", "version": "1"},
        "sdk": str(tmp_path.resolve()),
        "sdk_requested": str(tmp_path.resolve()),
        "sdk_source": "explicit",
        "libcpp_hardening": libcpp_hardening,
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
    if schema_version == 2:
        context.update(
            _schema2_records(
                tmp_path,
                platform_system=platform_system or platform.system(),
                stdlib_name=stdlib_name,
                sysroot=sysroot,
            )
        )
    (build / "native_check_context.json").write_text(json.dumps(context))

    if entries is None:
        linux_no_sysroot = schema_version == 2 and platform_system == "Linux"
        entries = [
            compile_entry(
                source, tmp_path, arguments=True, with_sysroot=not linux_no_sysroot
            )
        ]
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
    with_sysroot: bool = True,
) -> dict[str, object]:
    source_arg = os.path.relpath(source, directory) if relative_file else str(source)
    output = f"CMakeFiles/{target}.dir/{source.name}.o"
    sysroot_args = ["-isysroot", str(directory)] if with_sysroot else []
    argv = [
        sys.executable,
        "-std=c++23",
        *sysroot_args,
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

    with pytest.raises(ValueError, match="sysroot.*configured sysroot"):
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


def _retarget_fixture_compiler(build: Path, compiler: Path, compiler_id: str) -> None:
    """Point the fixture's context and compile database at a different
    configured compiler — load_entries re-validates the pairing, so both
    records must move together."""
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["compiler"]["path"] = str(compiler)
    context["compiler"]["id"] = compiler_id
    context_path.write_text(json.dumps(context))
    database_path = build / "compile_commands.json"
    database = json.loads(database_path.read_text())
    for entry in database:
        argv = (
            list(entry["arguments"])
            if "arguments" in entry
            else shlex.split(entry["command"])
        )
        argv[0] = str(compiler)
        if "arguments" in entry:
            entry["arguments"] = argv
        else:
            entry["command"] = shlex.join(argv)
    database_path.write_text(json.dumps(database))


def test_tidy_prefers_configured_compiler_sibling_over_default(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A clang-tidy from the configured compiler's own toolchain wins over
    the recorded default/PATH spelling — the sweep shares the compiler's
    toolchain view rather than an unrelated installation."""
    build, _ = write_fixture(tmp_path)
    toolchain_dir = tmp_path / "llvm-18" / "bin"
    toolchain_dir.mkdir(parents=True)
    compiler = toolchain_dir / "clang++-18"
    sibling = toolchain_dir / "clang-tidy-18"
    write_fake_tool(compiler)
    write_fake_tool(sibling)
    unrelated = tmp_path / "unrelated-clang-tidy"
    write_fake_tool(unrelated)
    _retarget_fixture_compiler(build, compiler, "Clang")
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["tool_defaults"]["tidy"] = str(unrelated)
    context_path.write_text(json.dumps(context))
    monkeypatch.delenv("CLANG_TIDY", raising=False)

    assert run_sweep("tidy", build) == 0
    summary = json.loads(
        (build / "native_check_summaries" / "tidy.json").read_text()
    )
    assert summary["tool"] == str(sibling)


def test_gxx_prefers_gnu_compiler_sibling_over_default(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The secondary frontend under a GNU build is the configured compiler's
    own spelling — not whatever g++ a default points at."""
    build, _ = write_fixture(tmp_path)
    toolchain_dir = tmp_path / "gcc-13" / "bin"
    toolchain_dir.mkdir(parents=True)
    compiler = toolchain_dir / "g++-13"
    write_fake_tool(compiler)
    unrelated = tmp_path / "unrelated-gxx"
    write_fake_tool(unrelated)
    _retarget_fixture_compiler(build, compiler, "GNU")
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["tool_defaults"]["gxx"] = str(unrelated)
    context_path.write_text(json.dumps(context))
    monkeypatch.delenv("GXX", raising=False)

    assert run_sweep("frontends", build) == 0
    summary = json.loads(
        (build / "native_check_summaries" / "frontends.json").read_text()
    )
    assert summary["tool"] == str(compiler)


@pytest.mark.parametrize(
    ("kind", "env_name"),
    [
        ("tidy", "CLANG_TIDY"),
        ("analyzer", "ANALYZER_CLANG"),
        ("cppcheck", "CPPCHECK"),
        ("frontends", "GXX"),
        ("odr", "GXX"),
    ],
)
def test_broken_required_override_fails_before_sweep(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, kind: str, env_name: str
) -> None:
    """A deliberately nonexistent required override must fail at resolution —
    before any empty sweep could be labeled passing."""
    build, _ = write_fixture(tmp_path)
    monkeypatch.setenv(env_name, str(tmp_path / f"missing-{env_name.lower()}"))

    assert run_sweep(kind, build) == 1
    summary = json.loads(
        (build / "native_check_summaries" / f"{kind}.json").read_text()
    )
    assert summary["status"] == "failed"
    assert any("override" in error for error in summary["errors"])
    assert summary["translation_unit_count"] == 0


def test_wrong_family_tool_is_a_selection_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A real executable that identifies as the wrong toolchain family is a
    selection error, not a passing sweep."""
    build, _ = write_fixture(tmp_path)
    compiler = shutil.which("c++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("no host C++ compiler to misuse as the tidy tool")
    monkeypatch.setenv("CLANG_TIDY", compiler)

    assert run_sweep("tidy", build) == 1
    summary = json.loads(
        (build / "native_check_summaries" / "tidy.json").read_text()
    )
    assert summary["tool_health"] == "failed"
    assert any("identifies as" in error for error in summary["errors"])
    assert summary["tool_inspection"]["family"] in {"clang", "gcc"}


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

    configured_build = configured_build_path()
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
            sys.executable,
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
    tmp_path: Path, *cmake_options: str, environ: dict[str, str] | None = None
) -> tuple[subprocess.CompletedProcess[str], Path]:
    repository = Path(__file__).resolve().parents[2]
    build = tmp_path / "native-configure"
    command = [
        "cmake",
        "-S",
        str(repository / "native"),
        "-B",
        str(build),
        # Pin the interpreter running the tests — FindPython otherwise picks
        # up whatever python3 lands first on PATH, which may not carry the
        # dependencies (threadpoolctl) the numerics probe requires.
        f"-DPython_EXECUTABLE={sys.executable}",
        *cmake_options,
    ]
    completed = subprocess.run(
        command,
        cwd=repository,
        env=environ,
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )
    return completed, build


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="CMAKE_OSX_SYSROOT is Darwin-only"
)
def test_explicit_sdk_alias_is_retained_and_canonicalized(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (configured_build_path() / "native_check_context.json").read_text()
    )
    current_sdk = Path(current_context["sdk"])
    explicit_alias = current_sdk.parent / "MacOSX26.5.sdk"
    if not explicit_alias.is_dir():
        pytest.skip(
            f"sibling SDK {explicit_alias} is not installed next to "
            f"{current_sdk} — the explicit-alias control requires a host "
            "carrying MacOSX26.5.sdk under that SDK directory"
        )

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


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="CMAKE_OSX_SYSROOT is Darwin-only"
)
def test_missing_explicit_sdk_fails_without_default_fallback(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    context = json.loads(
        (configured_build_path() / "native_check_context.json").read_text()
    )
    missing_sdk = tmp_path / "missing-explicit-sdk"

    configured, _ = configure_native(
        tmp_path,
        f"-DCMAKE_OSX_SYSROOT={missing_sdk}",
        f"-DCMAKE_CXX_COMPILER={context['compiler']['path']}",
    )

    assert configured.returncode != 0
    assert str(missing_sdk) in configured.stdout + configured.stderr


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="xcrun SDK resolution is Darwin-only"
)
def test_default_sdk_is_resolved_from_xcrun(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (configured_build_path() / "native_check_context.json").read_text()
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


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="libc++ EXTENSIVE default is Darwin-only"
)
def test_empty_build_type_defaults_to_extensive_hardening(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (configured_build_path() / "native_check_context.json").read_text()
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


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="libc++ FAST comparison is Darwin-only"
)
def test_fast_hardening_is_available_as_an_explicit_comparison(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    current_context = json.loads(
        (configured_build_path() / "native_check_context.json").read_text()
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
        (configured_build_path() / "native_check_context.json").read_text()
    )
    sdk_options = (
        [f"-DCMAKE_OSX_SYSROOT={current_context['sdk']}"]
        if platform.system() == "Darwin"
        else []
    )

    configured, build = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={current_context['compiler']['path']}",
        *sdk_options,
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


# ---------------------------------------------------------------------------
# T16 — RTSan qualification gates
#
# The realtime-sanitizer lane is qualified per toolchain: the flag and the
# BIKE_NONBLOCKING annotation must compile+link together, the annotation
# must demonstrably expand (the function-effects diagnostic must fire), and
# the violation control must produce the real RealtimeSanitizer report when
# run. A requested-but-unsupported lane is a configure error naming both
# -fsanitize=realtime and the selected compiler; an unrequested unsupported
# lane records supported=false — explicitly unavailable, never a green skip.
# ---------------------------------------------------------------------------

RTSAN_CONTROL_SOURCE = (
    Path(__file__).resolve().parents[2] / "native" / "tests" / "rtsan_control.cpp"
)


def _driver_accepts_rtsan(driver: Path, work_dir: Path) -> bool:
    """Probe: does this driver compile+link a TU under -fsanitize=realtime?"""
    work_dir.mkdir(parents=True, exist_ok=True)
    source = work_dir / "rtsan_accept_probe.cpp"
    source.write_text("int main() { return 0; }\n")
    completed = subprocess.run(
        [
            str(driver), "-std=c++23", "-fsanitize=realtime",
            str(source), "-o", str(work_dir / "rtsan_accept_probe"),
        ],
        text=True, capture_output=True, check=False, timeout=120,
    )
    return completed.returncode == 0


def _configured_compiler() -> Path | None:
    """The compiler the selected build was configured with, if loadable."""
    try:
        context = load_context(configured_build_path())
    except Exception:
        return None
    path = context.get("compiler", {}).get("path")
    return Path(path) if isinstance(path, str) and path else None


def _compiler_candidates() -> list[Path]:
    """Real C++ drivers to probe, most relevant first."""
    names: list[str] = []
    configured = _configured_compiler()
    if configured is not None:
        names.append(str(configured))
    names.extend(
        [
            os.environ.get("CXX", ""),
            "c++",
            "clang++",
            "g++",
            "/usr/bin/clang++",                      # AppleClang on macOS
            "/opt/homebrew/opt/llvm/bin/clang++",    # brew LLVM (the documented macOS RTSan lane)
            "/usr/local/opt/llvm/bin/clang++",       # brew LLVM on Intel macs
        ]
    )
    # Versioned distro spellings (clang++-21/-20/-19/-18, g++-14/-13...).
    for directory in ("/usr/bin", "/usr/local/bin", "/opt/homebrew/bin"):
        root = Path(directory)
        if root.is_dir():
            names.extend(
                str(path)
                for path in sorted(root.glob("clang++-*"), reverse=True)
            )
            names.extend(
                str(path)
                for path in sorted(root.glob("g++-*"), reverse=True)
            )
    found: list[Path] = []
    seen: set[str] = set()
    for name in names:
        if not name:
            continue
        located = Path(name) if Path(name).is_file() else (
            Path(shutil.which(name)) if shutil.which(name) else None
        )
        if located is None:
            continue
        key = str(located.resolve())
        if key not in seen:
            seen.add(key)
            found.append(located)
    return found


def _rtsan_capable_driver(tmp_path: Path) -> Path | None:
    """The first real driver whose compile+link accepts -fsanitize=realtime."""
    for index, candidate in enumerate(_compiler_candidates()):
        if _driver_accepts_rtsan(candidate, tmp_path / f"probe{index}"):
            return candidate
    return None


def _rtsan_lacking_driver(tmp_path: Path) -> Path | None:
    """The first real driver measured to reject -fsanitize=realtime."""
    for index, candidate in enumerate(_compiler_candidates()):
        if not _driver_accepts_rtsan(candidate, tmp_path / f"probe{index}"):
            return candidate
    return None


def _write_flag_rejecting_driver(
    tmp_path: Path, real_driver: Path, *, rejected_flag: str, label: str
) -> Path:
    """A compiler-probe fixture: forwards everything to the real driver but
    rejects one capability flag — a requested-lane configure must fail
    naming the flag and this selected compiler."""
    driver = tmp_path / f"{label}-c++"
    driver.write_text(
        "#!/bin/sh\n"
        'for arg in "$@"; do\n'
        '  case "$arg" in\n'
        f"    *{rejected_flag}*)\n"
        f'      echo "{label}-c++: unsupported argument'
        f' {rejected_flag}" >&2\n'
        "      exit 1 ;;\n"
        "  esac\n"
        "done\n"
        f'exec {shlex.quote(str(real_driver))} "$@"\n'
    )
    driver.chmod(0o755)
    return driver


def _write_runtime_shadowing_driver(tmp_path: Path, real_driver: Path) -> Path:
    """Forwards everything except rtsan -print-file-name queries, which echo
    the bare name back unresolved — a toolchain missing its RTSan runtime."""
    driver = tmp_path / "no-rtsan-runtime-c++"
    driver.write_text(
        "#!/bin/sh\n"
        'for arg in "$@"; do\n'
        '  case "$arg" in\n'
        "    -print-file-name=*rtsan*)\n"
        '      echo "${arg#-print-file-name=}"\n'
        "      exit 0 ;;\n"
        "  esac\n"
        "done\n"
        f'exec {shlex.quote(str(real_driver))} "$@"\n'
    )
    driver.chmod(0o755)
    return driver


def test_requested_rtsan_fails_on_a_flag_rejecting_compiler_fixture(
    tmp_path: Path,
) -> None:
    """The brief's compiler-probe fixture: C++23 works, -fsanitize=realtime
    is rejected; -DNATIVE_RTSAN=ON must fail naming flag and compiler."""
    real = _configured_compiler()
    if real is None:
        candidates = _compiler_candidates()
        if not candidates:
            pytest.skip("no host C++ compiler")
        real = candidates[0]
    fake = _write_flag_rejecting_driver(
        tmp_path, real, rejected_flag="-fsanitize=realtime", label="no-rtsan"
    )
    configured, _ = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={fake}", "-DNATIVE_RTSAN=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "-fsanitize=realtime" in output
    assert str(fake) in output


def test_requested_rtsan_fails_on_a_known_lacking_compiler(
    tmp_path: Path,
) -> None:
    """Step 4's negative path: explicitly request RTSan in a separate build
    directory on a compiler measured to lack the flag — must fail naming
    both the flag and the selected compiler."""
    driver = _rtsan_lacking_driver(tmp_path)
    if driver is None:
        pytest.skip("every candidate compiler on this host accepts -fsanitize=realtime")
    configured, _ = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={driver}", "-DNATIVE_RTSAN=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "-fsanitize=realtime" in output
    assert str(driver) in output


def test_unrequested_rtsan_capability_is_explicitly_unsupported(
    tmp_path: Path,
) -> None:
    """An unrequested lane on a lacking compiler configures fine and records
    {requested:false, supported:false, enabled:false, runtime:null} with
    evidence — explicitly unavailable, not a silent skip."""
    driver = _rtsan_lacking_driver(tmp_path)
    if driver is None:
        pytest.skip("every candidate compiler on this host accepts -fsanitize=realtime")
    configured, build = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={driver}"
    )
    assert configured.returncode == 0, configured.stdout + configured.stderr
    context = json.loads((build / "native_check_context.json").read_text())
    rtsan = context["capabilities"]["rtsan"]
    assert rtsan["requested"] is False
    assert rtsan["supported"] is False
    assert rtsan["enabled"] is False
    assert rtsan["runtime"] is None
    assert rtsan["compiler_id"] == context["compiler"]["id"]
    assert rtsan["evidence"]["control"], "missing probe evidence"


def test_requested_rtsan_fails_when_the_runtime_is_unresolvable(
    tmp_path: Path,
) -> None:
    """A capable compiler missing its RTSan runtime installation is a setup
    failure — configure must fail on the unresolved runtime, distinct from
    the flag-rejection path."""
    driver = _rtsan_capable_driver(tmp_path)
    if driver is None:
        pytest.skip("no RTSan-capable compiler on this host")
    shadowed = _write_runtime_shadowing_driver(tmp_path, driver)
    configured, _ = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={shadowed}", "-DNATIVE_RTSAN=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "rtsan" in output.lower()
    assert str(shadowed) in output


def test_qualified_rtsan_configure_enables_the_lane(tmp_path: Path) -> None:
    """On an RTSan-capable toolchain the requested lane configures, records
    enabled with a resolved runtime and run-control evidence, and the built
    control binary behaves as contracted."""
    driver = _rtsan_capable_driver(tmp_path)
    if driver is None:
        pytest.skip("no RTSan-capable compiler on this host")
    configured, build = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={driver}", "-DNATIVE_RTSAN=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode == 0, output
    context = json.loads((build / "native_check_context.json").read_text())
    rtsan = context["capabilities"]["rtsan"]
    assert rtsan["requested"] is True
    assert rtsan["supported"] is True
    assert rtsan["enabled"] is True
    assert rtsan["runtime"] is not None and Path(rtsan["runtime"]).is_file()
    assert "rtsan" in Path(rtsan["runtime"]).name
    evidence = rtsan["evidence"]
    assert evidence["control"]
    assert evidence["run_control"]
    # T16's deeper gates: the flag+annotation compile control, the
    # load-bearing macro check and the violation run control all recorded.
    assert evidence["annotation_control"]
    assert evidence["macro_control"]
    assert "RealtimeSanitizer" in evidence["violation_control"]
    # The built control target runs both modes as contracted: clean exits 0,
    # the violation dies with the RealtimeSanitizer diagnostic.
    built = subprocess.run(
        ["cmake", "--build", str(build), "--target", "native_rtsan_control"],
        text=True, capture_output=True, check=False, timeout=300,
    )
    assert built.returncode == 0, built.stdout + built.stderr
    binary = build / "native_rtsan_control"
    clean = subprocess.run(
        [str(binary)], text=True, capture_output=True, check=False,
        timeout=60,
    )
    assert clean.returncode == 0, clean.stderr
    violation = subprocess.run(
        [str(binary), "forbidden-allocation"],
        text=True, capture_output=True, check=False, timeout=60,
    )
    assert violation.returncode != 0
    assert "RealtimeSanitizer" in violation.stderr
    assert "malloc" in violation.stderr


def configure_policy_fixture(
    tmp_path: Path, *cmake_options: str, environ: dict[str, str] | None = None
) -> tuple[subprocess.CompletedProcess[str], Path]:
    """A standalone project including only BikeCompilerPolicy.cmake — the
    coverage/family gates without the BikePlatform SDK layer, which rejects
    a GNU driver before the policy even runs on Darwin."""
    repository = Path(__file__).resolve().parents[2]
    source = tmp_path / "policy-fixture"
    source.mkdir(parents=True)
    (source / "main.cpp").write_text("int main() { return 0; }\n")
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.27)\n"
        "project(policy_fixture CXX)\n"
        "set(CMAKE_CXX_STANDARD 23)\n"
        "set(CMAKE_CXX_STANDARD_REQUIRED ON)\n"
        "set(CMAKE_CXX_EXTENSIONS OFF)\n"
        f'include("{repository / "native" / "cmake" / "BikeCompilerPolicy.cmake"}")\n'
        "add_executable(probe main.cpp)\n"
        "bike_apply_native_policy(probe)\n"
    )
    build = tmp_path / "policy-fixture-build"
    completed = subprocess.run(
        ["cmake", "-S", str(source), "-B", str(build), *cmake_options],
        env=environ,
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )
    return completed, build


def _coverage_capable_driver() -> Path | None:
    """The first real clang-family driver whose coverage toolchain fully
    qualifies — the same ``resolve_coverage_tools`` the CMake policy runs,
    so a returned driver configures ``NATIVE_COVERAGE=ON`` cleanly."""
    for candidate in _compiler_candidates():
        try:
            if native_toolchain.inspect_tool(candidate)["family"] != "clang":
                continue
            native_toolchain.resolve_coverage_tools(candidate, {})
        except ValueError:
            continue
        return candidate
    return None


def _gnu_family_driver() -> Path | None:
    """A real GNU-family C++ driver, when the host carries one."""
    for candidate in _compiler_candidates():
        try:
            if native_toolchain.inspect_tool(candidate)["family"] == "gcc":
                return candidate
        except ValueError:
            continue
    return None


def _write_broken_profdata(tmp_path: Path) -> Path:
    """An llvm-profdata stand-in that identifies and launches but refuses
    every merge — a resolved-but-incompatible coverage tool."""
    tool = tmp_path / "broken-llvm-profdata"
    tool.write_text(
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--version" ]; then\n'
        "  printf 'broken LLVM version 99.0.0\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"${1:-}\" = \"merge\" ]; then\n"
        "  printf 'broken-llvm-profdata: incompatible profile format\\n' >&2\n"
        "  exit 7\n"
        "fi\n"
        "exit 5\n"
    )
    tool.chmod(0o755)
    return tool


def test_requested_coverage_fails_on_a_flag_rejecting_compiler_fixture(
    tmp_path: Path,
) -> None:
    """The compiler-probe fixture for the coverage lane: C++23 works but
    -fprofile-instr-generate is rejected. A requested coverage configure
    must fail naming the flag and the selected compiler — never configure
    an uninstrumented build."""
    real = _configured_compiler()
    if real is None:
        candidates = _compiler_candidates()
        if not candidates:
            pytest.skip("no host C++ compiler")
        real = candidates[0]
    fake = _write_flag_rejecting_driver(
        tmp_path,
        real,
        rejected_flag="-fprofile-instr-generate",
        label="no-coverage",
    )
    configured, _ = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={fake}", "-DNATIVE_COVERAGE=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "-fprofile-instr-generate" in output
    assert str(fake) in output


def test_requested_coverage_fails_for_a_non_clang_compiler(tmp_path: Path) -> None:
    """llvm-cov reads the clang source-based instrumentation format — a GNU
    C++ compiler in the requested coverage lane must be rejected at
    configure, not waved through to empty gcov output. The standalone
    policy fixture exercises the gate directly; the platform layer would
    reject the same driver earlier on the full project."""
    driver = _gnu_family_driver()
    if driver is None:
        pytest.skip("no GNU-family C++ compiler on this host")
    configured, _ = configure_policy_fixture(
        tmp_path, f"-DCMAKE_CXX_COMPILER={driver}", "-DNATIVE_COVERAGE=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "clang" in output.lower()
    assert str(driver) in output


def test_requested_coverage_fails_on_an_unresolvable_tool_override(
    tmp_path: Path,
) -> None:
    """An explicit LLVM_PROFDATA override that does not resolve is a hard
    configure failure — never a silent fallthrough to a discoverable tool."""
    driver = _coverage_capable_driver()
    if driver is None:
        pytest.skip("no clang toolchain with qualified coverage tools on this host")
    missing = tmp_path / "missing-llvm-profdata"
    configured, _ = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={driver}",
        "-DNATIVE_COVERAGE=ON",
        environ={**os.environ, "LLVM_PROFDATA": str(missing)},
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "LLVM_PROFDATA" in output
    assert str(missing) in output


def test_requested_coverage_fails_when_the_tool_chain_cannot_merge(
    tmp_path: Path,
) -> None:
    """A resolved llvm-profdata that cannot merge the compiler's instrumented
    control is a format-incompatibility configure failure naming the merge
    command — not an empty coverage success."""
    driver = _coverage_capable_driver()
    if driver is None:
        pytest.skip("no clang toolchain with qualified coverage tools on this host")
    broken = _write_broken_profdata(tmp_path)
    configured, _ = configure_native(
        tmp_path,
        f"-DCMAKE_CXX_COMPILER={driver}",
        "-DNATIVE_COVERAGE=ON",
        environ={**os.environ, "LLVM_PROFDATA": str(broken)},
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode != 0
    assert "merge" in output.lower()
    assert str(broken) in output


def test_qualified_coverage_configure_records_tool_evidence(tmp_path: Path) -> None:
    """A requested coverage configure on a qualified toolchain records the
    resolved tools and the compile/run/merge/export probe as capability
    evidence in the build context."""
    driver = _coverage_capable_driver()
    if driver is None:
        pytest.skip("no clang toolchain with qualified coverage tools on this host")
    configured, build = configure_native(
        tmp_path, f"-DCMAKE_CXX_COMPILER={driver}", "-DNATIVE_COVERAGE=ON"
    )
    output = configured.stdout + configured.stderr
    assert configured.returncode == 0, output
    context = json.loads((build / "native_check_context.json").read_text())
    coverage = context["capabilities"]["coverage"]
    assert coverage["requested"] is True
    assert coverage["supported"] is True
    assert coverage["enabled"] is True
    evidence = coverage["evidence"]
    assert evidence["control"], "missing flag-probe evidence"
    assert "llvm-profdata" in evidence["tools"]
    assert "llvm-cov" in evidence["tools"]
    assert "export" in evidence["format_probe"]


def test_rtsan_control_source_reports_the_sanitizer_diagnostic(
    tmp_path: Path,
) -> None:
    """Subprocess control: compile native/tests/rtsan_control.cpp with the
    toolchain's real RTSan flags; the violation must die with the selected
    RealtimeSanitizer diagnostic (not a loader error or unrelated signal),
    and the same source without the flag must exit cleanly — the annotation
    is load-bearing, not decorative."""
    driver = _rtsan_capable_driver(tmp_path)
    if driver is None:
        pytest.skip("no RTSan-capable compiler on this host")

    instrumented = tmp_path / "rtsan_control"
    compiled = subprocess.run(
        [
            str(driver), "-std=c++23", "-g", "-O0", "-fsanitize=realtime",
            str(RTSAN_CONTROL_SOURCE), "-o", str(instrumented),
        ],
        text=True, capture_output=True, check=False, timeout=300,
    )
    assert compiled.returncode == 0, compiled.stderr

    clean = subprocess.run(
        [str(instrumented)], text=True, capture_output=True, check=False,
        timeout=60,
    )
    assert clean.returncode == 0, clean.stderr

    violation = subprocess.run(
        [str(instrumented), "forbidden-allocation"],
        text=True, capture_output=True, check=False, timeout=60,
    )
    assert violation.returncode != 0
    output = violation.stdout + violation.stderr
    assert "RealtimeSanitizer" in output
    assert "malloc" in output

    # Load-bearing annotation control: the identical source without
    # -fsanitize=realtime expands BIKE_NONBLOCKING to nothing — malloc runs
    # unmarked, the process exits 0 and no sanitizer diagnostic appears.
    plain = tmp_path / "rtsan_control_plain"
    compiled_plain = subprocess.run(
        [
            str(driver), "-std=c++23", "-g", "-O0",
            str(RTSAN_CONTROL_SOURCE), "-o", str(plain),
        ],
        text=True, capture_output=True, check=False, timeout=300,
    )
    assert compiled_plain.returncode == 0, compiled_plain.stderr
    uninstrumented = subprocess.run(
        [str(plain), "forbidden-allocation"],
        text=True, capture_output=True, check=False, timeout=60,
    )
    assert uninstrumented.returncode == 0
    assert "RealtimeSanitizer" not in uninstrumented.stdout + uninstrumented.stderr


def run_controller_kind(kind: str, *, timeout: int = 180) -> subprocess.CompletedProcess[str]:
    repository = Path(__file__).resolve().parents[2]
    return subprocess.run(
        [
            sys.executable,
            str(repository / "tools" / "native_checks.py"),
            "--kind",
            kind,
            "--build",
            str(configured_build_path()),
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
        (configured_build_path() / "native_check_summaries" / "headers.json").read_text()
    )
    assert summary["tool_health"] == "ok"
    assert summary["finding_status"] == "none"
    assert summary["header_count"] > 0


def test_diagnostic_controls_cover_warning_families_and_move() -> None:
    result = run_controller_kind("diagnostic-controls", timeout=240)

    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads(
        (
            configured_build_path()
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
        (configured_build_path() / "native_check_summaries" / "context.json").read_text()
    )
    assert summary["tool_health"] == "ok"
    # Common selection-integrity controls run identically on both platforms.
    assert summary["common_controls"]["stale_dependency"] == "rejected"
    assert summary["common_controls"]["wrong_artifact"] == "rejected"
    assert summary["common_controls"]["source_list_change"] == "rejected"
    assert summary["common_controls"]["duplicate_tu"] == "rejected"
    assert summary["common_controls"]["target_context_drift"] == "rejected"
    assert summary["common_controls"]["explicit_paths"] == "passed"
    if platform.system() == "Darwin":
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
        assert all(
            item["assertion_active"]["EXTENSIVE"]
            for item in summary["hardening_evidence"]
        )
        assert all(
            "FAST" in item["assertion_active"]
            for item in summary["hardening_evidence"]
        )
        assert all(
            item["assertion_evidence"]["EXTENSIVE"]["signal"] == "SIGTRAP"
            and item["assertion_evidence"]["EXTENSIVE"]["category"]
            == "libc++ std::vector element access"
            for item in summary["hardening_evidence"]
        )
    else:
        assert summary["sdk_controls"]["no_sdk"] == "passed"
        assert summary["sdk_controls"]["bad_sysroot"] == "rejected"
        assert summary["sdk_controls"]["rejected_osx_sysroot"] == "rejected"
        assert summary["sdk_controls"]["no_apple_tool"] == "passed"
        assert summary["sdk_controls"]["sysroot_preservation"] == "passed"
        assert summary["sdk_controls"]["stdlib_hardening"] == "passed"
        assert summary["hardening_evidence"], "missing libstdc++ hardening evidence"
        assert all(
            item["stdlib"] == "libstdc++" and item["mode"] == "ASSERTIONS"
            for item in summary["hardening_evidence"]
        )
        assert all(
            item["assertion_active"]["ASSERTIONS"]
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


# ---------------------------------------------------------------------------
# Schema-2 platform-context validation
# ---------------------------------------------------------------------------


def _patch_host(monkeypatch: pytest.MonkeyPatch, system: str) -> None:
    """A synthetic context only validates against its claimed host."""
    monkeypatch.setattr(native_checks.platform, "system", lambda: system)


def test_schema2_context_loads_on_matching_host(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context = load_context(build)
    assert context["platform"]["system"] == platform.system()
    assert context["stdlib"]["name"] == "libc++" or context["stdlib"]["name"] == "libstdc++"
    if platform.system() == "Darwin":
        assert context["sdk"] is not None
        assert context["sysroot"] == context["sdk"]
    else:
        assert context["sdk"] is None
        assert context["sysroot"] is None


def test_schema1_context_remains_supported(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=1)
    context = load_context(build)
    # Schema-1 records are Darwin-shaped; the SDK stays a resolved path.
    assert context["sdk"] is not None
    assert Path(context["sdk"]).is_dir()


def test_schema2_linux_context_loads_and_records_no_sdk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    build, _ = write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        stdlib_name="libstdc++",
    )
    context = load_context(build)
    assert context["sdk"] is None
    assert context["sdk_requested"] is None
    assert context["sdk_source"] == "not-applicable"
    assert context["sysroot"] is None
    assert context["stdlib"]["name"] == "libstdc++"
    assert context["stdlib"]["hardening"]["mode"] == "ASSERTIONS"


def test_schema2_linux_entries_without_sysroot_validate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    build, expected = write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        stdlib_name="libstdc++",
    )
    entries = load_entries(build, expected)
    assert len(entries) == 1


def test_schema2_linux_sysroot_argument_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """An -isysroot the context did not record must fail selection."""
    _patch_host(monkeypatch, "Linux")
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=True)
    build, expected = write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        stdlib_name="libstdc++",
        entries=[entry],
    )
    with pytest.raises(native_checks.CheckError, match="configures none"):
        load_entries(build, expected)


def test_schema2_explicit_sysroot_directory_is_accepted(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    sysroot_dir = tmp_path / "sysroot"
    sysroot_dir.mkdir()
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    # Re-attach the sysroot argument pointing at the configured sysroot.
    entry["arguments"] = [
        entry["arguments"][0],
        "-std=c++23",
        "-isysroot",
        str(sysroot_dir.resolve()),
        *entry["arguments"][2:],
    ]
    build, expected = write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        stdlib_name="libstdc++",
        sysroot=str(sysroot_dir.resolve()),
        entries=[entry],
    )
    entries = load_entries(build, expected)
    assert len(entries) == 1


def test_schema2_missing_platform_record_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    del context["platform"]
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="platform"):
        load_context(build)


def test_schema2_platform_host_mismatch_fails(tmp_path: Path) -> None:
    claimed = "Linux" if platform.system() == "Darwin" else "Darwin"
    build, _ = write_fixture(
        tmp_path, schema_version=2, platform_system=claimed
    )
    with pytest.raises(native_checks.CheckError, match="host"):
        load_context(build)


def test_schema2_linux_context_must_not_record_sdk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    build, _ = write_fixture(
        tmp_path, schema_version=2, platform_system="Linux"
    )
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["sdk"] = str(tmp_path.resolve())
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="Apple SDK"):
        load_context(build)


def test_schema2_linux_wrong_sdk_source_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    build, _ = write_fixture(
        tmp_path, schema_version=2, platform_system="Linux"
    )
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["sdk_source"] = "explicit"
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="not-applicable"):
        load_context(build)


def test_schema2_linux_missing_sysroot_path_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_host(monkeypatch, "Linux")
    build, _ = write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        sysroot=str(tmp_path / "nonexistent-sysroot"),
    )
    with pytest.raises(native_checks.CheckError, match="sysroot"):
        load_context(build)


@pytest.mark.skipif(
    platform.system() != "Darwin", reason="Darwin SDK semantics"
)
def test_schema2_darwin_sysroot_drift_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    drifted = tmp_path / "drifted-sdk"
    drifted.mkdir()
    context["sysroot"] = str(drifted.resolve())
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="sysroot"):
        load_context(build)


def test_schema2_invalid_stdlib_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["stdlib"]["name"] = "msvc-stl"
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="stdlib"):
        load_context(build)


def test_schema2_stdlib_mode_mismatch_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(
        tmp_path, schema_version=2, stdlib_name="libstdc++"
    )
    if platform.system() == "Darwin":
        pytest.skip("Darwin requires libc++ — the stdlib check fails earlier")
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["stdlib"]["hardening"]["mode"] = "FAST"
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="hardening"):
        load_context(build)


def test_schema2_capability_enabled_without_support_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["capabilities"]["rtsan"]["enabled"] = True
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="without support"):
        load_context(build)


def _mutate_context(build: Path, *path: str, value: object = None) -> Path:
    """Set ``value`` at ``path`` (or delete the key when value is MISSING)."""
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    node = context
    for key in path[:-1]:
        node = node[key]
    if value is MISSING:
        del node[path[-1]]
    else:
        node[path[-1]] = value
    context_path.write_text(json.dumps(context))
    return context_path


MISSING = object()


def test_schema2_enabled_sanitizer_runtime_must_be_recorded(
    tmp_path: Path,
) -> None:
    """An enabled asan lane without a runtime record is a context the
    runtime helper could never preload — reject it at load."""
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "requested", value=True
    )
    _mutate_context(
        build, "capabilities", "asan", "enabled", value=True
    )
    with pytest.raises(native_checks.CheckError, match="runtime"):
        load_context(build)


def test_schema2_sanitizer_runtime_record_required(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(build, "capabilities", "asan", "runtime", value=MISSING)
    with pytest.raises(native_checks.CheckError, match="runtime"):
        load_context(build)


def test_schema2_sanitizer_runtime_must_exist(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "runtime",
        value=str(tmp_path / "absent" / "libasan.so"),
    )
    with pytest.raises(native_checks.CheckError, match="runtime"):
        load_context(build)


def test_schema2_sanitizer_runtime_must_be_absolute(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "runtime", value="libasan.so"
    )
    with pytest.raises(native_checks.CheckError, match="runtime"):
        load_context(build)


def test_schema2_sanitizer_runtime_must_be_a_file(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    directory = tmp_path / "libasan.so"
    directory.mkdir()
    _mutate_context(
        build, "capabilities", "asan", "runtime",
        value=str(directory.resolve()),
    )
    with pytest.raises(native_checks.CheckError, match="runtime"):
        load_context(build)


def test_schema2_enabled_sanitizer_records_a_real_runtime(
    tmp_path: Path,
) -> None:
    """The green path: an enabled lane whose runtime file exists loads."""
    build, _ = write_fixture(tmp_path, schema_version=2)
    runtime = tmp_path / "libasan.so"
    runtime.write_bytes(b"runtime image")
    _mutate_context(
        build, "capabilities", "asan", "requested", value=True
    )
    _mutate_context(
        build, "capabilities", "asan", "enabled", value=True
    )
    _mutate_context(
        build, "capabilities", "asan", "runtime",
        value=str(runtime.resolve()),
    )
    context = load_context(build)
    assert context["capabilities"]["asan"]["enabled"] is True
    assert context["capabilities"]["asan"]["runtime"] == str(runtime.resolve())


def test_schema2_enabled_sanitizer_requires_requested(tmp_path: Path) -> None:
    """enabled without requested is an inconsistent record — fail closed."""
    build, _ = write_fixture(tmp_path, schema_version=2)
    runtime = tmp_path / "libasan.so"
    runtime.write_bytes(b"runtime image")
    _mutate_context(
        build, "capabilities", "asan", "enabled", value=True
    )
    _mutate_context(
        build, "capabilities", "asan", "runtime",
        value=str(runtime.resolve()),
    )
    with pytest.raises(native_checks.CheckError, match="requested"):
        load_context(build)


def test_schema2_sanitizer_compiler_identity_required(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "compiler_id", value=MISSING
    )
    with pytest.raises(native_checks.CheckError, match="compiler"):
        load_context(build)


def test_schema2_sanitizer_target_may_be_null_or_triple(
    tmp_path: Path,
) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "target", value=42
    )
    with pytest.raises(native_checks.CheckError, match="target"):
        load_context(build)


def test_schema2_capability_evidence_rejects_empty_object(
    tmp_path: Path,
) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    _mutate_context(
        build, "capabilities", "asan", "evidence", value={}
    )
    with pytest.raises(native_checks.CheckError, match="evidence"):
        load_context(build)


def test_schema2_missing_numerics_fails(tmp_path: Path) -> None:
    build, _ = write_fixture(tmp_path, schema_version=2)
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    del context["numerics"]
    context_path.write_text(json.dumps(context))
    with pytest.raises(native_checks.CheckError, match="numerics"):
        load_context(build)


# ---------------------------------------------------------------------------
# Schema-2 Linux negative controls — the same fail-closed checks the Darwin
# fixtures exercise, run through the no-SDK platform record. Every case goes
# through load_context/load_entries, not a narrower subvalidator.
# ---------------------------------------------------------------------------


def _linux_fixture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    **kwargs: object,
) -> tuple[Path, set[Path]]:
    """A schema-2 Linux build context: no SDK, null sysroot, libstdc++."""
    _patch_host(monkeypatch, "Linux")
    return write_fixture(
        tmp_path,
        schema_version=2,
        platform_system="Linux",
        stdlib_name="libstdc++",
        **kwargs,
    )


def test_schema2_linux_stale_compiler_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    entry["arguments"][0] = "/bin/sh"
    build, expected = _linux_fixture(tmp_path, monkeypatch, entries=[entry])

    with pytest.raises(
        native_checks.CheckError, match="compiler differs from configured"
    ):
        load_entries(build, expected)


@pytest.mark.parametrize(
    "library_entry", ["missing-file", "plain-directory"]
)
def test_schema2_linux_invalid_library_path_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, library_entry: str
) -> None:
    build, _ = _linux_fixture(tmp_path, monkeypatch)
    library_path = tmp_path / "libprovider.so"
    if library_entry == "plain-directory":
        library_path.mkdir()
    context_path = build / "native_check_context.json"
    context = json.loads(context_path.read_text())
    context["library_paths"] = [str(library_path)]
    context_path.write_text(json.dumps(context))

    with pytest.raises(native_checks.CheckError, match="library_paths entry"):
        load_context(build)


def test_schema2_linux_duplicate_translation_unit_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    build, expected = _linux_fixture(
        tmp_path, monkeypatch, entries=[entry, entry.copy()]
    )

    with pytest.raises(
        native_checks.CheckError, match="duplicate.*translation unit"
    ):
        load_entries(build, expected)


def test_schema2_linux_missing_translation_unit_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source_root = tmp_path / "native" / "src"
    source_root.mkdir(parents=True)
    missing_tu = source_root / "two.cpp"
    missing_tu.write_text("int second_party() { return 0; }\n")
    source = source_root / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    build, expected = _linux_fixture(
        tmp_path,
        monkeypatch,
        sources=[
            (source, "bike_native", "default"),
            (missing_tu, "bike_native", "default"),
        ],
        entries=[entry],
    )

    with pytest.raises(
        native_checks.CheckError,
        match="missing expected translation units.*two.cpp",
    ):
        load_entries(build, expected)


def test_schema2_linux_wrong_target_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    entry["output"] = "CMakeFiles/other_target.dir/one.cpp.o"
    entry["arguments"][-1] = entry["output"]
    build, expected = _linux_fixture(tmp_path, monkeypatch, entries=[entry])

    with pytest.raises(
        native_checks.CheckError, match="unexpected target context"
    ):
        load_entries(build, expected)


@pytest.mark.parametrize(
    "flag_case", ["extensions-std", "omitted-option"]
)
def test_schema2_linux_wrong_selected_flags_fail(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, flag_case: str
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    if flag_case == "extensions-std":
        entry["arguments"][1] = "-std=gnu++23"
        match = r"not strict ISO C\+\+23"
    else:
        match = "omits target bike_native compile option"
    build, expected = _linux_fixture(tmp_path, monkeypatch, entries=[entry])
    if flag_case == "omitted-option":
        context_path = build / "native_check_context.json"
        context = json.loads(context_path.read_text())
        context["target_contexts"]["bike_native"]["compile_options"] = [
            "-ffp-contract=off"
        ]
        context_path.write_text(json.dumps(context))

    with pytest.raises(native_checks.CheckError, match=match):
        load_entries(build, expected)


def test_schema2_linux_relative_dependency_path_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A relative -I resolves against the entry directory — and must exist."""
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(
        source,
        tmp_path,
        arguments=True,
        with_sysroot=False,
        extra=["-I", "relative-include"],
    )
    build, expected = _linux_fixture(tmp_path, monkeypatch, entries=[entry])

    with pytest.raises(
        native_checks.CheckError, match="missing dependency path.*relative-include"
    ):
        load_entries(build, expected)


@pytest.mark.parametrize(
    "malformed", ["bad-arguments", "bad-command", "no-command"]
)
def test_schema2_linux_malformed_command_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, malformed: str
) -> None:
    source = tmp_path / "native" / "src" / "one.cpp"
    entry = compile_entry(source, tmp_path, arguments=True, with_sysroot=False)
    if malformed == "bad-arguments":
        entry["arguments"] = "not-a-list"
        match = "malformed arguments"
    elif malformed == "bad-command":
        del entry["arguments"]
        entry["command"] = f"{sys.executable} 'unterminated"
        match = "malformed command"
    else:
        del entry["arguments"]
        match = "needs command or arguments"
    build, expected = _linux_fixture(tmp_path, monkeypatch, entries=[entry])

    with pytest.raises(native_checks.CheckError, match=match):
        load_entries(build, expected)


def test_schema2_linux_absent_compile_database_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build, expected = _linux_fixture(tmp_path, monkeypatch)
    (build / "compile_commands.json").unlink()

    with pytest.raises(
        native_checks.CheckError, match="missing compilation database"
    ):
        load_entries(build, expected)
