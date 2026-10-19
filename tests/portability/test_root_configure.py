"""The standalone root configure, run against the real ``CMakeLists.txt``.

The module-level tests (test_cmake_platform.py, test_numerics_discovery.py)
include the shared CMake modules from a scratch project. They cannot show what
the root itself does: whether its configure starts an interpreter, whether its
include chain refuses a foreign toolchain, and whether BIKE_BUILD_PYTHON_BINDINGS=ON
fails closed. These tests configure the repository root, so they pin what a
developer or CI lane executes.

The root qualifies only the Darwin host profile (BikeNumerics RECORD_ONLY fails
closed elsewhere), so the successful configure and the bindings guard, which
comes after the numerics step, are Darwin-only. The cross-compilation refusal
runs after project() and before any numerics step, so it runs on every host.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CMAKE = shutil.which("cmake")

pytestmark = pytest.mark.skipif(CMAKE is None, reason="cmake executable unavailable")

CONFIGURE_TIMEOUT = 900
# Every interpreter name a PATH lookup can resolve, plus uv. Each stub records
# its call and exits 97, so a configure that starts one fails and leaves a record.
INTERPRETER_NAMES = ("python", "python3", "python3.13", "python3.14", "python3.15", "uv")
CROSS_REFUSAL = "Cross-compilation is not supported by this native-build workflow"
BINDINGS_REFUSAL = "is not wired to the cmake/Bike*.cmake platform layer yet"


def _require_dependency_tree() -> None:
    if not (REPO_ROOT / ".deps").is_dir():
        pytest.skip("the locked .deps tree is not acquired")


def _configure_root(build: Path, *, bindings: str = "OFF", extra: tuple[str, ...] = (),
                    env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    """Configure the repository root the way tools/run_tests.sh documents it."""
    _require_dependency_tree()
    return subprocess.run(
        [CMAKE, "-S", str(REPO_ROOT), "-B", str(build),
         "-DCMAKE_BUILD_TYPE=Release", f"-DBIKE_BUILD_PYTHON_BINDINGS={bindings}",
         "-DBIKE_BUILD_GUI=OFF", f"-DBIKE_DEPENDENCY_ROOT={REPO_ROOT / '.deps'}",
         *extra],
        env=env, capture_output=True, text=True, timeout=CONFIGURE_TIMEOUT)


def _squashed(text: str) -> str:
    """CMake wraps long messages at 80 columns; compare the words, not the lines."""
    return " ".join(text.split())


def _interpreter_shim(tmp_path: Path) -> tuple[dict[str, str], Path]:
    """An environment whose PATH starts with logging stubs for every interpreter.

    The stubs come before the system directories, so a PATH lookup by name lands
    on a stub. The returned log is the evidence; it exists only if something ran.
    """
    bin_dir = tmp_path / "interpreter-shim"
    bin_dir.mkdir()
    calls = tmp_path / "interpreter-calls.log"
    for name in INTERPRETER_NAMES:
        stub = bin_dir / name
        stub.write_text(f'#!/bin/sh\necho "{name} $*" >> "{calls}"\nexit 97\n',
                        encoding="utf-8")
        stub.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(bin_dir), "/usr/bin", "/bin"])
    return env, calls


def test_interpreter_shim_records_a_spawn_by_name(tmp_path: Path) -> None:
    """Control: the shim records a PATH spawn, so the zero-call assertion below can fail."""
    env, calls = _interpreter_shim(tmp_path)
    script = tmp_path / "spawn.cmake"
    script.write_text(
        "execute_process(COMMAND python3 -c pass RESULT_VARIABLE rc)\n"
        'if(NOT rc EQUAL 97)\n  message(FATAL_ERROR "shim did not run: ${rc}")\nendif()\n',
        encoding="utf-8")
    result = subprocess.run([CMAKE, "-P", str(script)], env=env,
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert calls.read_text(encoding="utf-8") == "python3 -c pass\n"


@pytest.mark.skipif(sys.platform != "darwin",
                    reason="the root graph qualifies only the Darwin host profile")
def test_root_configure_starts_no_interpreter(tmp_path: Path) -> None:
    """The real root configure succeeds with no interpreter reachable and records none.

    The cache check also covers a Python lookup that resolves through an absolute
    framework path and so would bypass the PATH stubs.
    """
    env, calls = _interpreter_shim(tmp_path)
    build = tmp_path / "root-build"
    result = _configure_root(build, env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not calls.exists(), calls.read_text(encoding="utf-8")
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
    assert not re.search(r"^_?P(?:ython|YTHON)\d*_", cache, re.MULTILINE), \
        "the root configure cached a Python package lookup"


def test_root_refuses_a_cross_compiling_toolchain(tmp_path: Path) -> None:
    """A foreign CMAKE_SYSTEM_NAME is refused by the root's own include chain.

    The refusal comes from BikeHostTarget.cmake, which the root includes after
    project(), and configure stops before "Configuring done".
    """
    result = _configure_root(tmp_path / "root-build", extra=("-DCMAKE_SYSTEM_NAME=Windows",))
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert CROSS_REFUSAL in _squashed(output), output
    assert "BikeHostTarget.cmake" in output, output
    assert "Configuring done" not in output, output


@pytest.mark.skipif(sys.platform != "darwin",
                    reason="the bindings guard is reached only after the Darwin-only numerics step")
def test_root_fails_closed_on_python_bindings(tmp_path: Path) -> None:
    """BIKE_BUILD_PYTHON_BINDINGS=ON from the root is refused by native/CMakeLists.txt.

    The root never builds the bindings itself; the transition extension is
    configured with cmake -S native. The refusal keeps the two dependency policies
    from mixing in one configure.
    """
    result = _configure_root(tmp_path / "root-build", bindings="ON")
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert BINDINGS_REFUSAL in _squashed(output), output
