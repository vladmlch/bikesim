"""Contract tests for tools.dev command orchestration (Python-only).

These tests never import the native extension and never spawn real builds:
every subprocess goes through ``tools.dev._run``, which tests stub out. The
quick profile must be able to run them on a host where no native artifact
exists. Runtime qualification is separate from command orchestration.
"""
from __future__ import annotations

import ast
import importlib.machinery
import inspect
import json
from pathlib import Path

import pytest

from tools import dev
from tools.dev import build_commands

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def _default_project_environment(monkeypatch):
    monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)


class _Completed:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def _record_run(calls, returncode=0):
    """Stub for dev._run that records invocations instead of spawning."""

    def fake(argv, *, cwd, env=None):
        calls.append({"argv": list(argv), "cwd": cwd, "env": env})
        return returncode

    return fake


def test_configure_uses_one_interpreter_and_preserves_path_tokens(tmp_path):
    root = tmp_path / "source tree"
    build = root / "native/build/release"
    commands = build_commands(root, build, "Release", "/opt/llvm tools/clang++")
    configure = commands[0]
    assert configure[:5] == ["uv", "run", "--frozen", "--group", "native"]
    assert str(build) in configure
    assert "-DCMAKE_CXX_COMPILER=/opt/llvm tools/clang++" in configure
    assert all(isinstance(token, str) for token in configure)


def test_build_commands_returns_configure_then_compile(tmp_path):
    root = tmp_path.resolve()
    build = root / "native" / "build" / "debug"
    commands = build_commands(root, build, "Debug", None)
    assert len(commands) == 2
    configure, compile_cmd = commands
    # Configure addresses the checkout's native source dir and the build dir.
    assert configure[:5] == ["uv", "run", "--frozen", "--group", "native"]
    assert "--no-sync" in configure
    assert "-S" in configure and str(root / "native") in configure
    assert "-B" in configure and str(build) in configure
    assert "-DCMAKE_BUILD_TYPE=Debug" in configure
    assert not any(t.startswith("-DCMAKE_CXX_COMPILER=") for t in configure)
    # Compile is a plain parallel build of the same directory.
    assert compile_cmd[:5] == ["uv", "run", "--frozen", "--group", "native"]
    assert compile_cmd[-3:] == ["--build", str(build), "--parallel"]


def test_unsupported_config_is_rejected(tmp_path):
    root = tmp_path.resolve()
    build = root / "native" / "build"
    for bad in ("RelWithDebInfo", "release", "", "DEBUG"):
        with pytest.raises(ValueError, match="Debug or Release"):
            build_commands(root, build, bad, None)


def test_relative_paths_are_rejected_by_command_builder(tmp_path):
    root = tmp_path.resolve()
    build = root / "native" / "build" / "release"
    with pytest.raises(ValueError, match="absolute"):
        build_commands(root, Path("native/build/release"), "Release", None)
    with pytest.raises(ValueError, match="absolute"):
        build_commands(Path("relative/root"), build, "Release", None)


def test_cli_resolves_relative_build_against_checkout(tmp_path):
    """A relative --build selector resolves against the checkout, not cwd."""
    root = tmp_path / "source tree"
    resolved = dev.resolve_build_path(root, "native/build/release")
    assert resolved == (root / "native/build/release").resolve()
    assert resolved.is_absolute()
    absolute = tmp_path / "elsewhere" / "build dir"
    assert dev.resolve_build_path(root, str(absolute)) == absolute.resolve()


def test_subprocess_failure_exit_code_propagates(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(dev, "_run", _record_run(calls, returncode=23))
    monkeypatch.setattr(dev, "_require_project_interpreter", lambda root: "/selected/python")
    monkeypatch.setattr(dev, "_check_build_cache", lambda root, build: None)
    rc = dev.main(["build", "--build", "native/build/release"], root=tmp_path)
    assert rc == 23
    assert len(calls) == 1
    assert calls[0]["cwd"] == tmp_path.resolve()


def test_bootstrap_missing_lock_fails_before_any_sync(tmp_path, monkeypatch, capsys):
    def explode(argv, *, cwd, env=None):  # pragma: no cover - must not run
        raise AssertionError("subprocess spawned despite missing uv.lock")

    monkeypatch.setattr(dev, "_run", explode)
    rc = dev.main(["bootstrap"], root=tmp_path)
    assert rc != 0
    assert "uv.lock" in capsys.readouterr().err


def test_bootstrap_is_explicit_locked_sync(tmp_path, monkeypatch):
    (tmp_path / "uv.lock").write_text("version = 1\n")
    calls = []
    monkeypatch.setattr(dev, "_run", _record_run(calls))
    assert dev.main(["bootstrap"], root=tmp_path) == 0
    assert calls[0]["argv"] == [
        "uv", "sync", "--locked", "--group", "native", "--group", "dev",
    ]
    assert calls[0]["cwd"] == tmp_path.resolve()
    assert calls[0]["env"]["UV_PROJECT_ENVIRONMENT"] == str(tmp_path / ".venv")


def test_bootstrap_python_and_offline_wheelhouse(tmp_path, monkeypatch):
    """Offline bootstrap is the wheelhouse-safe equivalent of `uv sync`:

    `uv sync --locked` pins registry URLs that are unreachable offline, so
    the offline path creates the explicit venv, exports (or reuses) the
    frozen requirements, then `uv pip sync --require-hashes` against
    --find-links with no index, and finally installs the project editable
    with --no-deps from the same wheelhouse.
    """
    (tmp_path / "uv.lock").write_text("version = 1\n")
    wheelhouse = tmp_path / "wheelhouse"
    wheelhouse.mkdir()
    calls = []
    monkeypatch.setattr(dev, "_run", _record_run(calls))
    argv_in = [
        "bootstrap", "--python", "3.13",
        "--offline", "--wheelhouse", str(wheelhouse),
    ]
    assert dev.main(argv_in, root=tmp_path) == 0
    venv = (tmp_path / ".venv").resolve()
    python = str(venv / "bin" / "python")

    venv_argv = calls[0]["argv"]
    assert venv_argv[:3] == ["uv", "venv", "--offline"]
    assert venv_argv[3] == str(venv)
    assert "--python" in venv_argv and "3.13" in venv_argv

    # No requirements.txt sits beside the project root, so a frozen export
    # is produced before the sync.
    export_argv = calls[1]["argv"]
    assert export_argv[:4] == [
        "uv", "export", "--frozen", "--offline",
    ]
    assert "--no-emit-project" in export_argv
    assert export_argv[export_argv.index("-o") + 1].endswith(
        "requirements.txt")

    # uv's supported controls restrict resolution to the wheelhouse only.
    common = ["--python", python, "--offline", "--no-index",
              "--find-links", str(wheelhouse)]
    sync_argv = calls[2]["argv"]
    assert sync_argv[:4] == ["uv", "pip", "sync", "--require-hashes"]
    assert sync_argv[4:-1] == common
    assert sync_argv[-1].endswith("requirements.txt")

    install_argv = calls[3]["argv"]
    assert install_argv[:4] == ["uv", "pip", "install", "--no-deps"]
    assert install_argv[4:4 + len(common)] == common
    assert install_argv[-2:] == ["--editable", str(tmp_path.resolve())]
    assert len(calls) == 4


def test_bootstrap_offline_requires_wheelhouse(tmp_path):
    (tmp_path / "uv.lock").write_text("version = 1\n")
    with pytest.raises(SystemExit):
        dev.main(["bootstrap", "--offline"], root=tmp_path)
    with pytest.raises(SystemExit):
        dev.main(["bootstrap", "--wheelhouse", str(tmp_path)], root=tmp_path)


def test_test_profile_sets_absolute_selectors(tmp_path, monkeypatch):
    """`test` exports ONE absolute build path under both selector names and
    delegates the profile logic to tools/run_tests.sh unchanged."""
    calls = []
    monkeypatch.setattr(dev, "_run", _record_run(calls, returncode=5))
    monkeypatch.setattr(dev, "_require_project_interpreter", lambda root: "/selected/python")
    monkeypatch.setenv("ASAN_OPTIONS", "detect_leaks=0")
    rc = dev.main(
        ["test", "--build", "native/build/release", "--profile", "quick"],
        root=tmp_path,
    )
    assert rc == 5
    call = calls[0]
    assert call["argv"] == [
        "bash",
        str((tmp_path / "tools" / "run_tests.sh").resolve()),
        "quick",
    ]
    expected = str((tmp_path / "native/build/release").resolve())
    env = call["env"]
    assert env["BIKE_NATIVE_BUILD_PATH"] == expected
    assert env["NATIVE_TEST_BUILD_PATH"] == expected
    # User-provided tool/sanitizer environment survives verbatim.
    assert env["ASAN_OPTIONS"] == "detect_leaks=0"


def test_no_command_uses_shell_true():
    """Every subprocess is an argument array: no shell=True anywhere."""
    tree = ast.parse(inspect.getsource(dev))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for keyword in node.keywords:
            assert not (
                keyword.arg == "shell"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value is True
            ), "shell=True is forbidden in tools/dev.py"


def _base_check_names(report):
    return [check["name"] for check in report["checks"]]


def test_doctor_schema_and_base_checks(tmp_path):
    """Bare doctor runs the unconditional checks; a missing lock is a named
    failure, and no native check is reported unless requested."""
    report = dev.doctor_report(tmp_path)
    assert report["schema_version"] == 1
    assert report["status"] in {"pass", "fail"}
    names = _base_check_names(report)
    assert names == ["interpreter", "lock", "packages", "compilers"]
    for check in report["checks"]:
        assert check["status"] in {"pass", "fail"}
        assert "evidence" in check
        assert "remediation" in check
    # tmp_path has no uv.lock: that check must fail by name.
    lock = next(c for c in report["checks"] if c["name"] == "lock")
    assert lock["status"] == "fail"
    assert "uv.lock" in lock["remediation"]


def test_doctor_missing_artifact_is_not_failed_unless_requested(tmp_path):
    """Spec: a missing native binary is a failure only when a native build
    check was requested."""
    report = dev.doctor_report(tmp_path)
    assert "selected_artifact" not in _base_check_names(report)
    build = tmp_path / "native" / "build" / "release"
    report = dev.doctor_report(tmp_path, build=build)
    artifact = next(
        c for c in report["checks"] if c["name"] == "selected_artifact"
    )
    assert artifact["status"] == "fail"
    assert "bike_native" in artifact["evidence"]


def test_doctor_unavailable_probes_fail_when_requested(tmp_path):
    """Pre-T05/T06 the dependency/numerics probes are requested-only named
    failures — never passing stubs."""
    build = tmp_path / "native" / "build" / "release"
    report = dev.doctor_report(tmp_path, build=build, numerics=True)
    names = _base_check_names(report)
    assert "dependency_probe" in names
    assert "numerical_profile" in names
    probe = next(c for c in report["checks"] if c["name"] == "dependency_probe")
    numerics = next(c for c in report["checks"] if c["name"] == "numerical_profile")
    assert probe["status"] == "fail"
    assert numerics["status"] == "fail"
    assert "unavailable" in probe["evidence"]
    assert "unavailable" in numerics["evidence"]
    assert report["status"] == "fail"
    # Without the flags the unavailable checks are not even listed.
    assert "dependency_probe" not in _base_check_names(dev.doctor_report(tmp_path))
    assert "numerical_profile" not in _base_check_names(dev.doctor_report(tmp_path))


def test_doctor_cli_writes_json_stdout_and_status(tmp_path, capsys):
    rc = dev.main(["doctor", "--build", "native/build/release"], root=tmp_path)
    captured = capsys.readouterr()
    report = json.loads(captured.out)
    assert report["schema_version"] == 1
    assert report["status"] == "fail"
    assert rc == 1
    # Human diagnostics go to stderr, not into the structured document.
    assert "selected_artifact" in captured.err


def test_doctor_selected_artifact_passes_with_one_extension(tmp_path):
    build = tmp_path / "native" / "build" / "release"
    build.mkdir(parents=True)
    suffix = importlib.machinery.EXTENSION_SUFFIXES[0]
    (build / f"bike_native{suffix}").write_bytes(b"artifact")
    report = dev.doctor_report(tmp_path, build=build)
    artifact = next(
        c for c in report["checks"] if c["name"] == "selected_artifact"
    )
    assert artifact["status"] == "pass"
    assert artifact["evidence"] == str(
        (build / f"bike_native{suffix}").resolve()
    )


def test_main_help_lists_every_command(capsys):
    with pytest.raises(SystemExit) as excinfo:
        dev.main(["--help"])
    assert excinfo.value.code == 0
    out = capsys.readouterr().out
    for command in ("bootstrap", "configure", "build", "doctor", "test"):
        assert command in out
