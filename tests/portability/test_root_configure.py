"""The standalone root configure, run against the real ``CMakeLists.txt``.

The module-level tests (test_cmake_platform.py, test_numerics_discovery.py)
include the shared CMake modules from a scratch project. They cannot show what
the root itself does: whether its configure starts an interpreter, whether its
include chain refuses a foreign toolchain, and whether BIKE_BUILD_PYTHON_BINDINGS=ON
fails closed. These tests configure the repository root, so they pin what a
developer or CI lane executes.

The successful configure and the bindings guard cover the supported Darwin
arm64 and Linux aarch64 root profiles, using the acquired dependency tree and
that platform's committed numerics record. Foreign-system dispatch is refused
before project(), before any compiler or numerics probe can run.
"""
from __future__ import annotations

import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from interpreter_shim import interpreter_shim_env, squashed

REPO_ROOT = Path(__file__).resolve().parents[2]
CMAKE = shutil.which("cmake")
ROOT_HOST_SUPPORTED = (
    (sys.platform == "darwin" and platform.machine() == "arm64") or
    (sys.platform == "linux" and platform.machine() == "aarch64")
)

pytestmark = pytest.mark.skipif(CMAKE is None, reason="cmake executable unavailable")

CONFIGURE_TIMEOUT = 900
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


def test_interpreter_shim_records_a_spawn_by_name(tmp_path: Path) -> None:
    """Control: the shim records a PATH spawn, so the zero-call assertion below can fail."""
    env, calls = interpreter_shim_env(tmp_path)
    script = tmp_path / "spawn.cmake"
    script.write_text(
        "execute_process(COMMAND python3 -c pass RESULT_VARIABLE rc)\n"
        'if(NOT rc EQUAL 97)\n  message(FATAL_ERROR "shim did not run: ${rc}")\nendif()\n',
        encoding="utf-8")
    result = subprocess.run([CMAKE, "-P", str(script)], env=env,
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert calls.read_text(encoding="utf-8") == "python3 -c pass\n"


@pytest.mark.skipif(not ROOT_HOST_SUPPORTED,
                    reason="the root lane requires Darwin arm64 or Linux aarch64")
def test_root_configure_starts_no_interpreter(tmp_path: Path) -> None:
    """The real root configure succeeds with no interpreter reachable and records none.

    The cache check also covers a Python lookup that resolves through an absolute
    framework path and so would bypass the PATH stubs.
    """
    env, calls = interpreter_shim_env(tmp_path)
    build = tmp_path / "root-build"
    result = _configure_root(build, env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not calls.exists(), calls.read_text(encoding="utf-8")
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
    assert not re.search(r"^_?P(?:ython|YTHON)\d*_", cache, re.MULTILINE), \
        "the root configure cached a Python package lookup"


def test_root_refuses_a_cross_compiling_toolchain(tmp_path: Path) -> None:
    """A foreign CMAKE_SYSTEM_NAME is refused by the root's own include chain.

    The refusal comes from BikeHostTarget.cmake through the root's pre-project
    BikePlatform dispatch, and configure stops before "Configuring done".
    """
    result = _configure_root(tmp_path / "root-build", extra=("-DCMAKE_SYSTEM_NAME=Windows",))
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert CROSS_REFUSAL in squashed(output), output
    assert "BikeHostTarget.cmake" in output, output
    assert "Configuring done" not in output, output


@pytest.mark.skipif(not ROOT_HOST_SUPPORTED,
                    reason="the root lane requires Darwin arm64 or Linux aarch64")
def test_root_fails_closed_on_python_bindings(tmp_path: Path) -> None:
    """BIKE_BUILD_PYTHON_BINDINGS=ON from the root is refused by native/CMakeLists.txt.

    The root never builds the bindings itself; the transition extension is
    configured with cmake -S native. The refusal keeps the two dependency policies
    from mixing in one configure.
    """
    result = _configure_root(tmp_path / "root-build", bindings="ON")
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert BINDINGS_REFUSAL in squashed(output), output
