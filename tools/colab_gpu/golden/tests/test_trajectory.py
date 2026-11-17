"""Tests for BGTRAJ01 trajectory writer/reader."""
from __future__ import annotations

import hashlib
import json
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parents[4]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.colab_gpu.golden.trajectory import MAGIC, VERSION, WHEEL_COLS, read_trajectory, write_trajectory


@dataclass
class _FakeEpisode:
    dt_s: float
    times_s: np.ndarray
    qpos: np.ndarray
    qvel: np.ndarray


@dataclass
class _FakeRun:
    mode: str
    window_s: float
    frame_stride: int
    frame_steps: np.ndarray
    frame_qpos: np.ndarray
    frame_qvel: np.ndarray
    wheels: np.ndarray


def test_write_and_read_synthetic_trajectory(tmp_path: Path) -> None:
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    fake_mjb = b"synthetic-mjb-bytes-for-hash"
    (bundle_dir / "model.mjb").write_bytes(fake_mjb)

    ep = _FakeEpisode(
        dt_s=0.005,
        times_s=np.array([0.0, 0.005, 0.010, 0.015, 0.020]),
        qpos=np.array([[0.0, 0.5, 0.0]] * 5),
        qvel=np.array([[2.0, 0.0]] * 5),
    )
    run = _FakeRun(
        mode="fixed",
        window_s=0.02,
        frame_stride=2,
        frame_steps=np.array([2, 4]),
        frame_qpos=np.array([[0.02, 0.51, 0.01], [0.04, 0.52, 0.02]]),
        frame_qvel=np.array([[2.1, 0.1], [2.2, 0.2]]),
        wheels=np.array([
            [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]],
            [[0.5, 0.0, 12.0, 400.0], [-0.5, 0.0, 8.0, 350.0]],
            [[0.6, 0.0, 13.0, 410.0], [-0.4, 0.0, 9.0, 360.0]],
            [[0.7, 0.0, 14.0, 420.0], [-0.3, 0.0, 10.0, 370.0]],
        ]),
    )

    out_dir = tmp_path / "traj"
    write_trajectory(out_dir, bundle_dir, ep, run)

    raw = (out_dir / "trajectory.bin").read_bytes()
    magic, version, nq, nv, wheel_cols, frames, sha = struct.unpack_from("<8sIIIIQ64s", raw, 0)
    assert magic == MAGIC
    assert version == VERSION
    assert nq == 3
    assert nv == 2
    assert wheel_cols == WHEEL_COLS
    assert frames == 3
    assert sha.decode("ascii") == hashlib.sha256(fake_mjb).hexdigest()

    row_cols = 2 + nq + nv + wheel_cols
    assert len(raw) == 96 + frames * row_cols * 8
    rows = np.frombuffer(raw, dtype="<f8", offset=96).reshape(frames, row_cols)
    times = rows[:, 0]
    ends = rows[:, 1]
    assert np.all(np.diff(times) > 0.0)
    assert ends[-1] > times[-1]
    np.testing.assert_allclose(times, [0.0, 0.010, 0.020])
    np.testing.assert_allclose(ends, [0.010, 0.020, 0.025])
    np.testing.assert_allclose(rows[0, 2 + nq + nv :], np.zeros(8))
    np.testing.assert_allclose(rows[1, 2 + nq + nv :], [0.5, 0.0, 12.0, 400.0, -0.5, 0.0, 8.0, 350.0])
    np.testing.assert_allclose(rows[2, 2 + nq + nv :], [0.7, 0.0, 14.0, 420.0, -0.3, 0.0, 10.0, 370.0])

    parsed = read_trajectory(out_dir)
    assert parsed.frames == 3
    assert parsed.nq == 3
    assert parsed.nv == 2
    np.testing.assert_array_equal(parsed.qpos, np.vstack([ep.qpos[0], run.frame_qpos]))
    np.testing.assert_array_equal(parsed.qvel, np.vstack([ep.qvel[0], run.frame_qvel]))

    meta = json.loads((out_dir / "trajectory.json").read_text(encoding="utf-8"))
    assert meta["format"] == "BGTRAJ01"
    assert meta["frames"] == 3
    assert meta["mode"] == "fixed"


def test_read_trajectory_rejects_corruption(tmp_path: Path) -> None:
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    (bundle_dir / "model.mjb").write_bytes(b"mjb-1")

    ep = _FakeEpisode(
        dt_s=0.01,
        times_s=np.array([0.0, 0.01, 0.02]),
        qpos=np.array([[0.0, 0.0]] * 3),
        qvel=np.array([[1.0]] * 3),
    )
    run = _FakeRun(
        mode="fixed",
        window_s=0.02,
        frame_stride=1,
        frame_steps=np.array([1, 2]),
        frame_qpos=np.array([[0.1, 0.0], [0.2, 0.0]]),
        frame_qvel=np.array([[1.0], [1.0]]),
        wheels=np.zeros((2, 2, 4)),
    )

    out_dir = tmp_path / "traj"
    write_trajectory(out_dir, bundle_dir, ep, run)

    # Truncated payload
    raw = (out_dir / "trajectory.bin").read_bytes()
    (out_dir / "trajectory.bin").write_bytes(raw[:-8])
    with pytest.raises(ValueError, match="size mismatch"):
        read_trajectory(out_dir)

    # Hash mismatch
    (out_dir / "trajectory.bin").write_bytes(raw)
    (out_dir / "model.mjb").write_bytes(b"different-mjb")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        read_trajectory(out_dir)

    # Bad magic
    (out_dir / "model.mjb").write_bytes(b"mjb-1")
    (out_dir / "trajectory.bin").write_bytes(b"BADMAGIC" + raw[8:])
    with pytest.raises(ValueError, match="invalid trajectory magic"):
        read_trajectory(out_dir)
