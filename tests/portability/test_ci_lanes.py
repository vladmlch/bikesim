"""Contract tests for tools.ci_portability lane definitions (T28).

Python-only: these tests inspect the generated argv arrays and the checked-in
workflow/toolchain documents; they never execute a lane (lanes run real
builds). The two tests below mirror plan step 1 verbatim.
"""
from __future__ import annotations

import json
import platform
import re
from pathlib import Path

import pytest

from tools import ci_portability
from tools.ci_portability import lane_commands

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = Path("/work/project")
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "portability.yml"
MATRIX_DOC = REPO_ROOT / "docs" / "development" / "toolchain-matrix.json"

REQUIRED_LANES = (
    "linux-clang-release",
    "linux-gcc-debug",
    "macos-appleclang-release",
    "linux-asan",
    "macos-asan",
    "linux-rtsan",
    "macos-rtsan",
    "linux-coverage",
    "macos-coverage",
    "linux-full",
    "macos-full",
)


def _lane_text(name: str) -> str:
    return " ".join(" ".join(command) for command in lane_commands(name, ROOT))


def _flatten(commands):
    return [token for command in commands for token in command]


# --- plan step 1, verbatim -------------------------------------------------


def test_linux_lane_does_not_use_apple_sdk():
    commands = lane_commands("linux-clang-release", Path("/work/project"))
    text = " ".join(" ".join(command) for command in commands)
    assert "--group native" in text
    assert "CMAKE_OSX_SYSROOT" not in text
    assert "run_tests.sh" in text


def test_macos_lane_is_not_removed():
    commands = lane_commands("macos-appleclang-release", Path("/work/project"))
    assert commands


# --- lane inventory and platform contract -----------------------------------


def test_every_required_lane_exists_plus_trusted_lanes():
    for name in REQUIRED_LANES:
        assert name in ci_portability.LANES, f"missing required lane {name}"
    trusted = set(ci_portability.TRUSTED_LANES)
    assert {"linux-release", "macos-release"} <= trusted
    assert {"linux-offline", "macos-offline"} <= trusted
    assert any("manual" in name for name in trusted)


def test_every_lane_asserts_platform_first():
    for name in ci_portability.LANES:
        first = lane_commands(name, ROOT)[0]
        assert first[0] == "python3"
        assert first[-2:] == ["--assert-platform", name], first


def test_lane_spec_expectations_match_name():
    for name, spec in ci_portability.LANES.items():
        if name.startswith("linux"):
            assert spec.system == "Linux"
        elif name.startswith("macos"):
            assert spec.system == "Darwin"
        else:  # pragma: no cover - naming drift guard
            raise AssertionError(f"lane name lacks os prefix: {name}")
        assert spec.machines, name


@pytest.mark.parametrize("name", sorted(ci_portability.LANES))
def test_lane_commands_are_absolute_argv_arrays(name):
    for command in lane_commands(name, ROOT):
        assert command and all(isinstance(token, str) and token for token in command)
        assert Path(command[0]).name in {"python3", "uv", "env", "bash"}


def test_platform_assertion_rejects_wrong_os():
    """On this Darwin host a linux lane must fail the platform assert."""
    if platform.system() != "Darwin":
        pytest.skip("host is not macOS; inversion covered by inspection")
    spec = ci_portability.lane_spec("linux-clang-release")
    assert ci_portability._assert_platform(spec) != 0


@pytest.mark.skipif(platform.system() != "Darwin", reason="macOS lane check")
def test_macos_lane_platform_assertion_passes_on_macos():
    spec = ci_portability.lane_spec("macos-appleclang-release")
    assert spec.machines == ("arm64",)
    assert ci_portability._assert_platform(spec) == 0


# --- toolchain / build contract ----------------------------------------------


def test_unique_build_path_per_lane():
    builds = {}
    for name in ci_portability.LANES:
        spec = ci_portability.lane_spec(name)
        build = ci_portability.build_path(ROOT, spec)
        assert build not in builds, f"{name} shares {build} with {builds[build]}"
        builds[build] = name
        assert build.name == name  # build dir identity is the lane name


def test_every_lane_uses_frozen_native_group():
    for name in ci_portability.LANES:
        text = _lane_text(name)
        assert "--frozen" in text, name
        assert "--group native" in text, name


def test_no_lane_invokes_uv_lock_or_upgrades():
    for name in ci_portability.LANES:
        tokens = _flatten(lane_commands(name, ROOT))
        for index, token in enumerate(tokens[:-1]):
            assert not (token == "uv" and tokens[index + 1] == "lock"), name
        assert "--upgrade" not in tokens
        assert "--locked" not in tokens or "sync" not in tokens  # only bootstrap syncs


def test_no_lane_uses_apple_sdk_options_on_linux():
    for name in ci_portability.LANES:
        if not name.startswith("linux"):
            continue
        text = _lane_text(name)
        assert "CMAKE_OSX_SYSROOT" not in text
        assert "xcrun" not in text
        assert "-isysroot" not in text


def test_declared_compiler_per_native_lane():
    for name, spec in ci_portability.LANES.items():
        if spec.kind != "native":
            continue
        assert spec.compiler is not None and spec.compiler.candidates, name
        steps = [c for c in lane_commands(name, ROOT) if "--configure" in c]
        assert len(steps) == 1, name
        step = steps[0]
        index = step.index("--compiler")
        assert step[index + 1] == spec.compiler.candidates[0]


def test_declared_compiler_for_coverage_lanes():
    for name, spec in ci_portability.LANES.items():
        if spec.kind != "coverage":
            continue
        assert spec.compiler is not None, name
        text = _lane_text(name)
        assert "--run-coverage" in text and spec.compiler.candidates[0] in text


def test_compiler_resolution_prefers_declared_then_candidates(monkeypatch):
    spec = ci_portability.lane_spec("linux-clang-release")
    seen = []

    def fake(candidate):
        seen.append(candidate)
        return Path("/opt/llvm/bin/" + candidate) if candidate.endswith("-18") else None

    monkeypatch.setattr(ci_portability, "_resolve_candidate", fake)
    monkeypatch.delenv("CXX", raising=False)
    resolved = ci_portability._resolve_compiler(spec, preferred="clang++")
    assert resolved == Path("/opt/llvm/bin/clang++-18")
    assert seen[0] == "clang++"  # declared spelling tried first


def test_compiler_resolution_cxx_override_wins(monkeypatch, tmp_path):
    clang = tmp_path / "clang++"
    clang.write_text("#!/bin/sh\n")
    clang.chmod(0o755)
    spec = ci_portability.lane_spec("linux-clang-release")
    monkeypatch.setenv("CXX", str(clang))
    assert ci_portability._resolve_compiler(spec) == clang


def test_compiler_resolution_fails_closed(monkeypatch):
    spec = ci_portability.lane_spec("linux-clang-release")
    monkeypatch.setattr(ci_portability, "_resolve_candidate", lambda c: None)
    monkeypatch.delenv("CXX", raising=False)
    with pytest.raises(ValueError, match="unresolved"):
        ci_portability._resolve_compiler(spec)


def test_required_analysis_tools_checked():
    required = {
        "cmake", "ctest", "ninja", "shellcheck", "cppcheck", "clang-tidy",
        "analyzer-clang++", "g++", "uv", "python3",
    }
    for name, spec in ci_portability.LANES.items():
        steps = [c for c in lane_commands(name, ROOT) if "--check-tools" in c]
        assert len(steps) == 1, f"{name} lacks a tool check step"
        names = {tool.name for tool in spec.required_tools}
        if spec.kind in {"native", "coverage"}:
            missing = required - names
            assert not missing, f"{name} does not require {sorted(missing)}"
        for tool in spec.required_tools:
            assert tool.candidates, f"{name}.{tool.name} has no candidates"


def test_coverage_lanes_require_llvm_tools():
    for name in ("linux-coverage", "macos-coverage"):
        spec = ci_portability.lane_spec(name)
        names = {tool.name for tool in spec.required_tools}
        assert {"llvm-profdata", "llvm-cov"} <= names, name


def test_building_lanes_select_their_build_for_tests():
    for name, spec in ci_portability.LANES.items():
        if spec.kind != "native":
            continue
        build = str(ci_portability.build_path(ROOT, spec))
        steps = [c for c in lane_commands(name, ROOT) if "run_tests.sh" in " ".join(c)]
        assert len(steps) == 1, name
        step = steps[0]
        assert f"NATIVE_TEST_BUILD_PATH={build}" in step
        assert f"BIKE_NATIVE_BUILD_PATH={build}" in step
        assert step[-1] == spec.profile


def test_linux_lanes_do_not_remove_macos_expectations():
    """Darwin lanes keep SDK/xcrun/Accelerate toolchain evidence."""
    spec = ci_portability.lane_spec("macos-appleclang-release")
    assert any(tool.name == "xcrun" for tool in spec.required_tools)


# --- runner semantics ---------------------------------------------------------


def test_unsupported_lane_name_fails():
    with pytest.raises(ValueError, match="unsupported lane"):
        lane_commands("windows-msvc-2019", ROOT)


def test_cli_rejects_unsupported_lane(capsys):
    rc = ci_portability.main(["--lane", "windows-msvc-2019"], root=ROOT)
    assert rc == 2
    assert "unsupported lane" in capsys.readouterr().err


def test_failed_command_stops_the_lane(monkeypatch):
    calls = []

    def fake(argv, *, cwd, env=None):
        calls.append(list(argv))
        return 23

    monkeypatch.setattr(ci_portability, "_run", fake)
    rc = ci_portability.main(["--lane", "linux-clang-release"], root=ROOT)
    assert rc == 23
    assert len(calls) == 1, "the lane must stop at the first failed command"


def test_lane_runs_every_command_when_all_pass(monkeypatch):
    calls = []
    monkeypatch.setattr(
        ci_portability, "_run",
        lambda argv, *, cwd, env=None: (calls.append(list(argv)), 0)[1],
    )
    rc = ci_portability.main(["--lane", "macos-appleclang-release"], root=ROOT)
    assert rc == 0
    assert len(calls) == len(lane_commands("macos-appleclang-release", ROOT))


def test_lane_failure_exit_code_is_preserved(monkeypatch):
    """A failed test command propagates its own nonzero status."""

    def fake(argv, *, cwd, env=None):
        return 7 if "run_tests.sh" in " ".join(argv) else 0

    monkeypatch.setattr(ci_portability, "_run", fake)
    rc = ci_portability.main(["--lane", "linux-full"], root=ROOT)
    assert rc == 7


def test_lane_commands_do_not_execute(monkeypatch):
    """lane_commands is a pure generator — no subprocess may spawn."""

    def explode(*args, **kwargs):  # pragma: no cover - must not run
        raise AssertionError("subprocess spawned while listing commands")

    monkeypatch.setattr(ci_portability.subprocess, "run", explode)
    for name in ci_portability.LANES:
        assert lane_commands(name, ROOT)


def test_offline_lane_uses_checked_in_offline_tools():
    for name in ("linux-offline", "macos-offline"):
        text = _lane_text(name)
        assert "tools/offline_bundle.py" in text
        assert "--mode full" in text and "--mode slim" in text
        assert "--native" in text  # native-source bundle rehearsal retained
        assert "--offline-install" in text


def test_lane_names_match_toolchain_matrix():
    document = json.loads(MATRIX_DOC.read_text())
    assert document["schema_version"] == 1
    assert set(document["lanes"]) == set(ci_portability.LANES)
    for name, entry in document["lanes"].items():
        spec = ci_portability.lane_spec(name)
        assert entry["system"] == spec.system
        assert entry["python_minor"] == spec.python_minor
        assert set(entry["machines"]) == set(spec.machines)


# --- workflow contract --------------------------------------------------------


def _workflow():
    yaml = pytest.importorskip(
        "yaml", reason="workflow validation needs pyyaml (uv run --with pyyaml)"
    )
    return yaml.safe_load(WORKFLOW.read_text())


def _uses_refs(document):
    refs = []
    for job in document["jobs"].values():
        for step in job.get("steps", []):
            if "uses" in step:
                refs.append(str(step["uses"]))
    return refs


def test_workflow_yaml_parses_and_every_use_is_full_sha():
    document = _workflow()
    assert document["jobs"], "workflow has no jobs"
    sha = re.compile(r"^[\w./-]+@[0-9a-f]{40}$")
    refs = _uses_refs(document)
    assert refs, "workflow uses no actions — a stray tag ref would be invisible"
    for ref in refs:
        assert sha.match(ref), f"action not pinned to a full SHA: {ref}"


def test_workflow_triggers_and_permissions():
    document = _workflow()
    on = document.get("on", document.get(True))  # YAML 1.1 parses `on:` as True
    assert on and "pull_request" in on and "push" in on
    assert "workflow_dispatch" in on
    assert "pull_request_target" not in on
    assert document["permissions"]["contents"] == "read"


def test_workflow_matrix_covers_both_oses_and_required_lanes():
    document = _workflow()
    lanes_by_runner = {}
    scheduled_or_dispatch = set()
    for job in document["jobs"].values():
        gated = job.get("if", "")
        gated_text = json.dumps(gated)
        for entry in job["strategy"]["matrix"]["include"]:
            runner, lane = entry["runner"], entry["lane"]
            assert lane in ci_portability.LANES, f"workflow runs undeclared lane {lane}"
            runner_key = runner if isinstance(runner, str) else "self-hosted"
            lanes_by_runner.setdefault(runner_key, set()).add(lane)
            if "schedule" in gated_text or "workflow_dispatch" in gated_text:
                scheduled_or_dispatch.add(lane)
    ubuntu_lanes = lanes_by_runner.get("ubuntu-24.04", set())
    macos_lanes = lanes_by_runner.get("macos-15", set())
    assert {"linux-clang-release", "linux-gcc-debug"} <= ubuntu_lanes
    assert "macos-appleclang-release" in macos_lanes
    # Runtime-intensive lanes exist and are schedule/dispatch-gated — release
    # acceptance must not omit them.
    intensive = {
        "linux-asan", "macos-asan", "linux-rtsan", "macos-rtsan",
        "linux-coverage", "macos-coverage", "linux-full", "macos-full",
    }
    assert intensive <= ubuntu_lanes | macos_lanes
    assert intensive <= scheduled_or_dispatch


def test_workflow_runs_the_lane_runner_and_uploads_evidence():
    document = _workflow()
    for job_name, job in document["jobs"].items():
        runs = [step.get("run", "") for step in job["steps"]]
        assert any(
            "ci_portability.py" in run and "--lane" in run for run in runs
        ), f"{job_name} does not execute the lane runner"
        uploads = [
            step for step in job["steps"]
            if "upload-artifact" in str(step.get("uses", ""))
        ]
        assert uploads, f"{job_name} uploads no evidence"
        for step in uploads:
            assert "always()" in str(step.get("if", "")), (
                f"{job_name} evidence upload must run on failure too"
            )
            path_block = str(step["with"]["path"])
            assert ".venv" not in path_block and "venv" not in path_block
            assert "secrets" not in path_block


def test_workflow_never_runs_fork_code_on_trusted_runners():
    document = _workflow()
    for job_name, job in document["jobs"].items():
        entries = job["strategy"]["matrix"]["include"]
        trusted = any(
            not isinstance(entry["runner"], str)
            for entry in entries
        ) or any(
            ci_portability.lane_spec(entry["lane"]).trusted for entry in entries
        )
        if trusted:
            gate = str(job.get("if", ""))
            assert "workflow_dispatch" in gate, (
                f"{job_name}: trusted lanes must be dispatch/environment-gated"
            )
