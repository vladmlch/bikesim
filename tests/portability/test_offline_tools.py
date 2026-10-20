"""Local tool selection controls; no resolver or installed project is required."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
from zipfile import ZipFile

import pytest

from tools import prepare_offline_tools as prepare
from tools.offline_bundle import load_tool_lock


@pytest.fixture
def target():
    return {
        "system": "Linux", "machine": "x86_64", "python_minor": "3.13",
        "implementation": "cpython", "accepted_wheel_tags": ["py3-none-any"],
        "libc_floor": "glibc>=2.28",
    }


def write_wheel(directory: Path, name: str, *, version: str = "1.0",
                requires: tuple[str, ...] = (), license_file: bool = True) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    distribution = name.replace("-", "_")
    path = directory / f"{distribution}-{version}-py3-none-any.whl"
    info = f"{distribution}-{version}.dist-info"
    metadata = (
        f"Metadata-Version: 2.4\nName: {name}\nVersion: {version}\n"
        "Requires-Python: >=3.13\n"
        + "".join(f"Requires-Dist: {requirement}\n" for requirement in requires)
        + "\n"
    )
    with ZipFile(path, "w") as archive:
        archive.writestr(f"{info}/METADATA", metadata)
        archive.writestr(f"{info}/WHEEL", "Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
        archive.writestr(f"{info}/RECORD", "")
        if license_file:
            archive.writestr(f"{info}/licenses/LICENSE", "Synthetic test wheel; no third-party bytes.\n")
        archive.writestr(f"{distribution}/__init__.py", "raise AssertionError('a wheel was executed')\n")
    return path


@pytest.fixture
def project(tmp_path):
    root = tmp_path / "project"
    root.mkdir()
    (root / "pyproject.toml").write_text(
        '[build-system]\nrequires = ["setuptools>=1.0"]\nbuild-backend = "setuptools.build_meta"\n',
        encoding="utf-8",
    )
    return root


@pytest.fixture
def wheelhouse(tmp_path):
    root = tmp_path / "tools"
    for name in ("uv", "pip", "setuptools", "wheel"):
        write_wheel(root, name)
    return root


def test_freezes_selected_wheel_bytes_without_launching_any_tool(project, wheelhouse, target, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("tool lock preparation must not start a process")
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    lock = prepare.prepare_tool_lock(project, wheelhouse, target)
    assert set(lock) == {"schema_version", "target", "uv_version", "wheels"}
    assert lock["schema_version"] == 1
    assert lock["target"] == target
    assert lock["uv_version"] == "1.0"
    assert {row["distribution"] for row in lock["wheels"]} == {"uv", "pip", "setuptools", "wheel"}
    for row in lock["wheels"]:
        assert row["version"] == "1.0"
        assert row["tags"] == ["py3-none-any"]
        assert row["sha256"] == hashlib.sha256((wheelhouse / row["filename"]).read_bytes()).hexdigest()


@pytest.mark.parametrize("name", ("uv", "pip", "setuptools", "wheel"))
def test_missing_required_tool_is_rejected(project, wheelhouse, target, name):
    (wheelhouse / f"{name}-1.0-py3-none-any.whl").unlink()
    with pytest.raises(ValueError, match=name):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_two_versions_of_one_distribution_are_ambiguous(project, wheelhouse, target):
    write_wheel(wheelhouse, "pip", version="1.1")
    with pytest.raises(ValueError, match="more than one wheel for pip"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_transitive_tool_requirement_must_be_locally_present(project, wheelhouse, target):
    write_wheel(wheelhouse, "wheel", requires=("packaging>=1.0",))
    with pytest.raises(ValueError, match="packaging"):
        prepare.prepare_tool_lock(project, wheelhouse, target)
    write_wheel(wheelhouse, "packaging")
    lock = prepare.prepare_tool_lock(project, wheelhouse, target)
    assert len(lock["wheels"]) == 5


def test_real_project_build_requirement_is_checked(project, wheelhouse, target):
    (project / "pyproject.toml").write_text(
        '[build-system]\nrequires = ["setuptools>=2.0"]\n', encoding="utf-8"
    )
    with pytest.raises(ValueError, match="setuptools"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_unrelated_wheel_does_not_become_an_implicit_tool_root(project, wheelhouse, target):
    write_wheel(wheelhouse, "unrelated")
    with pytest.raises(ValueError, match="unrelated"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_active_dependency_marker_requires_dependency(project, wheelhouse, target):
    write_wheel(wheelhouse, "wheel", requires=('packaging>=1.0; sys_platform == "linux"',))
    with pytest.raises(ValueError, match="packaging"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_inactive_dependency_marker_does_not_require_another_platform_tool(project, wheelhouse, target):
    write_wheel(wheelhouse, "wheel", requires=('colorama>=1.0; sys_platform == "win32"',))
    assert len(prepare.prepare_tool_lock(project, wheelhouse, target)["wheels"]) == 4


def test_uv_license_must_travel_with_the_selected_tool(project, wheelhouse, target):
    write_wheel(wheelhouse, "uv", license_file=False)
    with pytest.raises(ValueError, match="[Ll]icen[cs]e"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_symlinked_tool_wheel_is_not_a_frozen_local_input(project, wheelhouse, target, tmp_path):
    selected = wheelhouse / "pip-1.0-py3-none-any.whl"
    moved = tmp_path / selected.name
    selected.rename(moved)
    selected.symlink_to(moved)
    with pytest.raises(ValueError, match="non-symlink"):
        prepare.prepare_tool_lock(project, wheelhouse, target)


def test_cli_writes_a_new_lock_and_preserves_existing_pins(project, wheelhouse, target, tmp_path, monkeypatch):
    monkeypatch.setattr(prepare, "select_target", lambda name, version, native: target)
    output = tmp_path / "pins" / "tools.json"
    args = ["--project", str(project), "--wheelhouse", str(wheelhouse), "--output", str(output), "--native"]
    assert prepare.main(args) == 0
    initial = output.read_bytes()
    assert json.loads(initial)["uv_version"] == "1.0"
    assert prepare.main(args) == 2
    assert output.read_bytes() == initial


def test_prepared_lock_is_consumed_by_the_assembler(project, wheelhouse, target, tmp_path):
    lock = prepare.prepare_tool_lock(project, wheelhouse, target)
    path = tmp_path / "tools.json"
    path.write_text(json.dumps(lock), encoding="utf-8")
    restored, records = load_tool_lock(
        path, wheelhouse, target, {"build-system": {"requires": ["setuptools>=1.0"]}}
    )
    assert restored == lock
    assert [record["wheel"] for record in records] == lock["wheels"]


def test_failed_preparation_writes_no_output(project, wheelhouse, target, tmp_path, monkeypatch):
    monkeypatch.setattr(prepare, "select_target", lambda name, version, native: target)
    (wheelhouse / "uv-1.0-py3-none-any.whl").unlink()
    output = tmp_path / "failed" / "tools.json"
    assert prepare.main(["--project", str(project), "--wheelhouse", str(wheelhouse), "--output", str(output)]) == 2
    assert not output.exists()
    assert not output.parent.exists()


def test_cli_rejects_corrupt_wheel_without_writing_a_lock(project, wheelhouse, target, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(prepare, "select_target", lambda name, version, native: target)
    (wheelhouse / "pip-1.0-py3-none-any.whl").write_bytes(b"not a ZIP archive")
    output = tmp_path / "tools.json"
    assert prepare.main(["--project", str(project), "--wheelhouse", str(wheelhouse), "--output", str(output)]) == 2
    assert "OFFLINE TOOL LOCK ERROR" in capsys.readouterr().err
    assert not output.exists()
