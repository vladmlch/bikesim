"""Contract tests for tools.portability_baseline (Python-only).

These tests never import the native extension. The quick profile must be
able to run them on a host where no native artifact exists — including
Linux, where native configure currently fails before dependency
discovery.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from tools.portability_baseline import collect_baseline

REPO_ROOT = Path(__file__).resolve().parents[2]

REQUIRED_SECTIONS = (
    "schema_version",
    "git",
    "python",
    "platform",
    "packages",
    "numpy_config",
    "tools",
    "missing_inputs",
)


def test_report_has_required_sections(tmp_path):
    report = collect_baseline(tmp_path)
    for key in REQUIRED_SECTIONS:
        assert key in report, f"missing baseline section {key!r}"


def test_missing_checkout_inputs_are_explicit(tmp_path):
    (tmp_path / "pyproject.toml").write_text('[project]\nname="probe"\n')
    report = collect_baseline(tmp_path)
    assert report["schema_version"] == 1
    assert "uv.lock" in report["missing_inputs"]
    assert report["git"]["commit"] is None
    assert report["git"]["verified"] is False


@pytest.mark.skipif(shutil.which("git") is None, reason="git executable unavailable")
def test_git_checkout_reports_dirty_without_mutation(tmp_path):
    repo = tmp_path / "checkout"
    repo.mkdir()

    def git(*args):
        return subprocess.run(
            ["git", *args], cwd=repo, text=True, capture_output=True, check=True
        )

    git("init", "-q", "-b", "main")
    git("config", "user.email", "probe@example.invalid")
    git("config", "user.name", "probe")
    tracked = repo / "tracked.txt"
    tracked.write_text("committed\n")
    git("add", "tracked.txt")
    git("commit", "-q", "-m", "initial")
    tracked.write_text("edited after commit\n")

    report = collect_baseline(repo)

    assert report["git"]["verified"] is True
    assert report["git"]["commit"] is not None
    assert len(report["git"]["commit"]) == 40
    assert report["git"]["dirty"] is True
    # Collection must not modify, stash, or clean the uncommitted change.
    assert tracked.read_text() == "edited after commit\n"
    status = git("status", "--porcelain").stdout
    assert " M tracked.txt" in status


def test_real_checkout_baseline(tmp_path):
    """The recovered real checkout: lock present, git identity verifiable."""
    report = collect_baseline(REPO_ROOT)
    assert report["schema_version"] == 1
    assert "uv.lock" not in report["missing_inputs"]
    assert report["git"]["verified"] is True
    assert report["git"]["commit"] is not None
    assert len(report["git"]["commit"]) == 40
    # The native dependency contract pins MuJoCo exactly; the locked
    # environment must satisfy it for this baseline to be meaningful.
    declared = report["packages"]["declared"]
    assert declared["mujoco"]["version"] == "3.12.0"
    assert declared["numpy"]["version"] == "2.5.2"
    # Genuinely absent inputs are named, not silently dropped.
    assert ".python-version" in report["missing_inputs"]
    # T28 adds .github/workflows/portability.yml, so .github is no longer
    # an absent input; the remaining entries below are still missing.
    assert ".github" not in report["missing_inputs"]
    assert (
        "docs/superpowers/specs/2026-10-06-native-safety.md"
        in report["missing_inputs"]
    )
    assert (
        "docs/superpowers/plans/2026-10-06-native-safety-master.md"
        in report["missing_inputs"]
    )
