"""Selection and validation contract for tools.native_dependencies.

The fixture-dir tests exercise pure library selection and schema validation
with package-shaped temporary directories; they never import third-party
packages (mujoco/nanobind are only imported inside the probe's own child
interpreter). The subprocess controls at the bottom cover the fail-closed
CLI contract that tools/dev.py doctor and BikeDependencies.cmake consume.
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tools.native_dependencies import (
    REQUIRED_MUJOCO_VERSION,
    probe_dependencies,
    select_mujoco_library,
    validate_dependencies,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PROBE_SCRIPT = REPO_ROOT / "tools" / "native_dependencies.py"

CANONICAL = {"Darwin": "libmujoco.3.12.0.dylib", "Linux": "libmujoco.so.3.12.0"}
ALIAS = {"Darwin": "libmujoco.dylib", "Linux": "libmujoco.so"}


# --------------------------------------------------------------------------
# select_mujoco_library — pure selection over package-shaped directories
# --------------------------------------------------------------------------


def test_linux_library_selection_is_not_a_dylib_glob(tmp_path):
    library = tmp_path / "libmujoco.so.3.12.0"
    library.write_bytes(b"fixture")
    (tmp_path / "libmujoco.3.12.0.dylib").write_bytes(b"wrong platform")
    assert select_mujoco_library(tmp_path, "Linux", "3.12.0") == library.resolve()


def test_missing_exact_engine_is_fatal(tmp_path):
    (tmp_path / "libmujoco.so.3.15.0").write_bytes(b"wrong version")
    with pytest.raises(ValueError, match="3.12.0"):
        select_mujoco_library(tmp_path, "Linux", "3.12.0")


def test_darwin_selects_the_versioned_dylib(tmp_path):
    library = tmp_path / "libmujoco.3.12.0.dylib"
    library.write_bytes(b"fixture")
    (tmp_path / "libmujoco.so.3.12.0").write_bytes(b"wrong platform")
    assert select_mujoco_library(tmp_path, "Darwin", "3.12.0") == library.resolve()


def test_unversioned_alias_resolves_to_the_same_image(tmp_path):
    """A linker-name symlink next to the real image is one selection, not two."""
    library = tmp_path / CANONICAL["Linux"]
    library.write_bytes(b"real image")
    (tmp_path / ALIAS["Linux"]).symlink_to(library.name)
    assert select_mujoco_library(tmp_path, "Linux", "3.12.0") == library.resolve()


def test_versioned_name_itself_may_be_a_symlink(tmp_path):
    """The canonical name may be a symlink; the real image is returned."""
    real = tmp_path / "real" / CANONICAL["Linux"]
    real.parent.mkdir()
    real.write_bytes(b"real image")
    (tmp_path / CANONICAL["Linux"]).symlink_to(real)
    assert select_mujoco_library(tmp_path, "Linux", "3.12.0") == real.resolve()


def test_conflicting_real_images_are_fatal(tmp_path):
    """An alias resolving to a *different* image than the canonical name is a
    named conflict, never a coin toss."""
    canonical = tmp_path / CANONICAL["Linux"]
    canonical.write_bytes(b"local image")
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    other = elsewhere / CANONICAL["Linux"]
    other.write_bytes(b"other image")
    (tmp_path / ALIAS["Linux"]).symlink_to(other)
    with pytest.raises(ValueError, match="exactly one"):
        select_mujoco_library(tmp_path, "Linux", "3.12.0")


def test_unversioned_real_library_cannot_prove_the_version(tmp_path):
    """A bare libmujoco.so real file could be any version — reject it even
    though the filename matches an accepted alias."""
    (tmp_path / ALIAS["Linux"]).write_bytes(b"unversioned image")
    with pytest.raises(ValueError, match="3.12.0"):
        select_mujoco_library(tmp_path, "Linux", "3.12.0")


def test_alias_to_a_different_version_is_fatal(tmp_path):
    (tmp_path / "libmujoco.so.3.15.0").write_bytes(b"wrong version")
    (tmp_path / ALIAS["Linux"]).symlink_to("libmujoco.so.3.15.0")
    with pytest.raises(ValueError, match="3.12.0"):
        select_mujoco_library(tmp_path, "Linux", "3.12.0")


def test_nonexistent_package_directory_is_fatal(tmp_path):
    missing = tmp_path / "no-such-package"
    with pytest.raises(ValueError, match="missing"):
        select_mujoco_library(missing, "Linux", "3.12.0")


def test_package_directory_with_spaces_selects_exactly(tmp_path):
    spaced = tmp_path / "site packages with spaces"
    spaced.mkdir()
    library = spaced / CANONICAL["Darwin"]
    library.write_bytes(b"fixture")
    assert (
        select_mujoco_library(spaced, "Darwin", "3.12.0") == library.resolve()
    )


def test_wrong_version_is_rejected_before_lookup(tmp_path):
    """The version gate runs first: even a correctly named image under a
    non-3.12.0 package is refused with the pinned version in the message."""
    (tmp_path / CANONICAL["Linux"]).write_bytes(b"correct name, wrong package")
    with pytest.raises(ValueError, match="3.12.0"):
        select_mujoco_library(tmp_path, "Linux", "3.15.0")


def test_unsupported_platform_is_named(tmp_path):
    (tmp_path / "mujoco.dll").write_bytes(b"windows")
    with pytest.raises(ValueError, match="unsupported native platform: Windows"):
        select_mujoco_library(tmp_path, "Windows", "3.12.0")


# --------------------------------------------------------------------------
# validate_dependencies — schema-1 type/existence contract (pure stdlib data)
# --------------------------------------------------------------------------


def _fixture_document(tmp_path: Path) -> dict:
    """A schema-1 document whose paths are real fixture files."""
    package = tmp_path / "site-packages"
    mujoco_inc = package / "mujoco" / "include" / "mujoco"
    mujoco_inc.mkdir(parents=True, exist_ok=True)
    (mujoco_inc / "mujoco.h").write_text("/* fixture */\n")
    library = package / "mujoco" / CANONICAL["Linux"]
    library.write_bytes(b"fixture image")
    python_include = tmp_path / "python-include"
    python_include.mkdir(exist_ok=True)
    nb_cmake = tmp_path / "nanobind" / "cmake"
    nb_include = tmp_path / "nanobind" / "include"
    nb_cmake.mkdir(parents=True, exist_ok=True)
    nb_include.mkdir(parents=True, exist_ok=True)
    return {
        "schema_version": 1,
        "platform": {"system": "Linux", "machine": "aarch64", "libc": "glibc 2.39"},
        "python": {
            "executable": sys.executable,
            "version": "3.13.16",
            "soabi": "cpython-313-aarch64-linux-gnu",
            "include_dir": str(python_include),
        },
        "mujoco": {
            "version": REQUIRED_MUJOCO_VERSION,
            "include_dir": str(mujoco_inc.parent),
            "library_path": str(library),
        },
        "nanobind": {
            "cmake_dir": str(nb_cmake),
            "include_dir": str(nb_include),
        },
        "numerics": None,
    }


def test_validate_dependencies_accepts_a_complete_document(tmp_path):
    document = _fixture_document(tmp_path)
    assert validate_dependencies(document) is document


def test_validate_dependencies_rejects_non_object_and_wrong_schema(tmp_path):
    with pytest.raises(ValueError, match="object"):
        validate_dependencies(["not", "a", "dict"])
    document = _fixture_document(tmp_path)
    document["schema_version"] = 2
    with pytest.raises(ValueError, match="schema_version"):
        validate_dependencies(document)


def test_validate_dependencies_enforces_the_engine_pin(tmp_path):
    document = _fixture_document(tmp_path)
    document["mujoco"]["version"] = "3.15.0"
    with pytest.raises(ValueError, match="3.12.0"):
        validate_dependencies(document)


def test_validate_dependencies_requires_absolute_existing_paths(tmp_path):
    document = _fixture_document(tmp_path)
    document["mujoco"]["library_path"] = "relative/libmujoco.so.3.12.0"
    with pytest.raises(ValueError, match="absolute"):
        validate_dependencies(document)
    document = _fixture_document(tmp_path)
    document["mujoco"]["library_path"] = str(tmp_path / "libmujoco.so.3.12.0")
    with pytest.raises(ValueError, match="exist"):
        validate_dependencies(document)


def test_validate_dependencies_checks_library_name_against_platform(tmp_path):
    """A .dylib path on a Linux record is inconsistent — fail closed."""
    document = _fixture_document(tmp_path)
    dylib = tmp_path / "site-packages" / "mujoco" / CANONICAL["Darwin"]
    dylib.write_bytes(b"wrong platform image")
    document["mujoco"]["library_path"] = str(dylib)
    with pytest.raises(ValueError, match="libmujoco"):
        validate_dependencies(document)


def test_validate_dependencies_requires_real_sections(tmp_path):
    document = _fixture_document(tmp_path)
    del document["nanobind"]
    with pytest.raises(ValueError, match="nanobind"):
        validate_dependencies(document)
    document = _fixture_document(tmp_path)
    document["python"]["executable"] = str(tmp_path / "missing-python")
    with pytest.raises(ValueError, match="executable"):
        validate_dependencies(document)
    document = _fixture_document(tmp_path)
    document["numerics"] = "not-an-object"
    with pytest.raises(ValueError, match="numerics"):
        validate_dependencies(document)


# --------------------------------------------------------------------------
# CLI / probe controls — run through subprocesses, never in-process imports
# --------------------------------------------------------------------------


def _run_cli(*argv: str, env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(PROBE_SCRIPT), *argv],
        capture_output=True,
        text=True,
        env=env,
        timeout=300,
    )


def test_cli_rejects_a_nonexistent_interpreter(tmp_path):
    result = _run_cli("--python", str(tmp_path / "missing-python"))
    assert result.returncode != 0
    assert "missing-python" in result.stderr


def test_cli_rejects_a_relative_interpreter_path():
    result = _run_cli("--python", "python3")
    assert result.returncode != 0
    assert "absolute" in result.stderr


def test_cli_reports_named_error_for_a_wrong_mujoco_version(tmp_path):
    """An isolated site tree with a fake non-3.12.0 mujoco fails with the pin
    named — before any build could start, never a fallback."""
    packages = tmp_path / "fake-site-packages"
    package = packages / "mujoco"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text('__version__ = "3.15.0"\n')
    env = dict(os.environ)
    env["PYTHONPATH"] = str(packages) + os.pathsep + env.get("PYTHONPATH", "")
    result = _run_cli("--python", sys.executable, env=env)
    assert result.returncode != 0
    assert "3.12.0" in result.stderr
    assert "3.15.0" in result.stderr


def test_cli_fails_closed_when_mujoco_is_absent(tmp_path):
    """A bare venv interpreter (no third-party packages) gets a named
    dependency error, not an implicit environment mutation or a silent skip."""
    venv = tmp_path / "bare-venv"
    creation = subprocess.run(
        [sys.executable, "-m", "venv", "--without-pip", str(venv)],
        capture_output=True,
        text=True,
        timeout=120,
    )
    if creation.returncode != 0:
        pytest.skip(f"venv creation unavailable on this host: {creation.stderr}")
    interpreter = venv / "bin" / "python"
    result = _run_cli("--python", str(interpreter))
    assert result.returncode != 0
    assert "mujoco" in result.stderr


@pytest.mark.skipif(
    importlib.util.find_spec("mujoco") is None
    or importlib.util.find_spec("nanobind") is None,
    reason="real probe needs the selected interpreter's mujoco+nanobind",
)
def test_probe_dependencies_reports_this_environment():
    document = probe_dependencies(Path(sys.executable))
    assert document["schema_version"] == 1
    assert document["mujoco"]["version"] == REQUIRED_MUJOCO_VERSION
    library = Path(document["mujoco"]["library_path"])
    assert library.is_file()
    assert library.name == CANONICAL[document["platform"]["system"]]
    assert Path(document["python"]["executable"]).resolve() == Path(
        sys.executable
    ).resolve()


@pytest.mark.skipif(
    importlib.util.find_spec("mujoco") is None
    or importlib.util.find_spec("nanobind") is None,
    reason="real probe needs the selected interpreter's mujoco+nanobind",
)
def test_cli_stdout_is_pure_schema_json_for_doctor():
    """tools/dev.py doctor json.loads()es stdout verbatim — nothing but the
    document may be printed there."""
    result = _run_cli("--python", sys.executable)
    assert result.returncode == 0, result.stderr
    document = json.loads(result.stdout)
    assert document["schema_version"] == 1
    assert document["mujoco"]["library_path"]
    assert document["mujoco"]["include_dir"]
    assert document["nanobind"]["cmake_dir"]
