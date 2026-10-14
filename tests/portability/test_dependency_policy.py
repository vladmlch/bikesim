"""Dependency-pin policy for the native scientific environment.

These tests never import the native extension; they exercise the declared
requirements and the artifact-side enforcement helpers only, so the quick
profile can run them on hosts without a built ``bike_native``.
"""
from __future__ import annotations

import tomllib
from pathlib import Path

import pytest

from bike_sim.native import artifact

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_native_scientific_versions_are_pinned():
    project = tomllib.loads(Path("pyproject.toml").read_text())
    native = project["dependency-groups"]["native"]
    assert "mujoco==3.12.0" in native
    assert "numpy==2.5.2" in native
    assert "mujoco==3.12.0" in project["project"]["dependencies"]


def test_preserved_scientific_pins_and_python_floor():
    """Untouched constraints stay: SciPy/DuckDB/Arrow exact pins, the numba
    floor, and the declared interpreter floor (no raise to satisfy pins)."""
    project = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    dependencies = project["project"]["dependencies"]
    for pin in ("scipy==1.17.0", "duckdb==1.5.6", "pyarrow==26.0.0"):
        assert pin in dependencies
    assert "numba>=0.68" in dependencies
    assert project["project"]["requires-python"] == ">=3.13"


def test_threadpoolctl_is_a_tooling_floor_not_an_exact_pin():
    """threadpoolctl is provider-discovery/diagnostics tooling: it joins the
    native group as a ``>=`` floor and must NOT enter the enforced exact-pin
    map that ``check_dependencies`` applies before loading the extension."""
    project = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    native = project["dependency-groups"]["native"]
    assert "threadpoolctl>=3.6" in native
    assert all(not str(entry).startswith("threadpoolctl==") for entry in native)
    assert "threadpoolctl" not in artifact.required_dependencies()


def test_required_dependencies_returns_direct_exact_native_pins():
    """The enforced map is exactly the ``==`` pins of the native group."""
    assert artifact.required_dependencies() == {
        "mujoco": "3.12.0",
        "numpy": "2.5.2",
    }


def test_check_dependencies_enforces_exact_native_pins():
    """Match passes; a wrong version or a missing package names the package."""
    required = artifact.required_dependencies()
    assert artifact.check_dependencies(installed=dict(required)) == required
    for name in required:
        mismatched = dict(required, **{name: "0.0.0"})
        with pytest.raises(ValueError, match=name):
            artifact.check_dependencies(installed=mismatched)
        missing = {key: value for key, value in required.items() if key != name}
        with pytest.raises(ValueError, match=name):
            artifact.check_dependencies(installed=missing)


def test_required_dependencies_fallback_is_retained(monkeypatch):
    """Without the checkout layout the full exact-pin set still applies."""
    monkeypatch.setattr(artifact, "_REPO_ROOT", Path("/nonexistent-checkout"))
    assert artifact.required_dependencies() == {
        "mujoco": "3.12.0",
        "numpy": "2.5.2",
    }


def test_fallback_mirrors_the_exact_native_pins():
    """A new ``==`` pin in the native group must update the bundled fallback
    in the same change; drift is caught here, not silently in deployments."""
    assert artifact._FALLBACK_DEPENDENCIES == artifact.required_dependencies()
