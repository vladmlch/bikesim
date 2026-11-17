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

from tools.colab_gpu.golden.trajectory import (
    MAGIC,
    VERSION,
    WHEEL_COLS,
    read_trajectory,
    write_trajectory,
    write_trajectory_arrays,
)


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

    # Trailing bytes / oversized payload
    (out_dir / "trajectory.bin").write_bytes(raw + b"\x00" * 8)
    with pytest.raises(ValueError, match="size mismatch"):
        read_trajectory(out_dir)

    # Nonfinite values in qpos, qvel, wheels rejected on read_trajectory
    header = raw[:96]
    valid_rows = np.frombuffer(raw, dtype="<f8", offset=96).reshape(3, 2 + 2 + 1 + 8).copy()
    for col, label in ((2, "qpos"), (4, "qvel"), (5, "wheels")):
        for bad_val in (np.nan, np.inf):
            bad_rows = valid_rows.copy()
            bad_rows[1, col] = bad_val
            (out_dir / "trajectory.bin").write_bytes(header + bad_rows.tobytes())
            with pytest.raises(ValueError, match=f"{label} contains nonfinite"):
                read_trajectory(out_dir)

    # Domain errors in binary payload rejected on read_trajectory
    bad_time_rows = valid_rows.copy()
    bad_time_rows[0, 0] = -0.01
    (out_dir / "trajectory.bin").write_bytes(header + bad_time_rows.tobytes())
    with pytest.raises(ValueError, match="nonnegative"):
        read_trajectory(out_dir)

    noninc_rows = valid_rows.copy()
    noninc_rows[2, 0] = noninc_rows[1, 0]
    (out_dir / "trajectory.bin").write_bytes(header + noninc_rows.tobytes())
    with pytest.raises(ValueError, match="strictly increasing"):
        read_trajectory(out_dir)

    bad_end_rows = valid_rows.copy()
    bad_end_rows[1, 1] = bad_end_rows[1, 0]
    (out_dir / "trajectory.bin").write_bytes(header + bad_end_rows.tobytes())
    with pytest.raises(ValueError, match="ends_s must exceed times_s"):
        read_trajectory(out_dir)

    one_frame_header = struct.pack(
        "<8sIIIIQ64s",
        MAGIC,
        VERSION,
        2,
        1,
        WHEEL_COLS,
        1,
        hashlib.sha256(b"mjb-1").hexdigest().encode("ascii"),
    )
    (out_dir / "trajectory.bin").write_bytes(one_frame_header + valid_rows[:1].tobytes())
    with pytest.raises(ValueError, match="at least two frames"):
        read_trajectory(out_dir)


def test_write_trajectory_arrays_validation(tmp_path: Path) -> None:
    mjb_path = tmp_path / "model.mjb"
    mjb_path.write_bytes(b"mjb-1")

    times = np.array([0.0, 0.01, 0.02])
    ends = np.array([0.01, 0.02, 0.03])
    qpos = np.zeros((3, 2))
    qvel = np.ones((3, 1))
    wheels = np.zeros((3, WHEEL_COLS))

    with pytest.raises(ValueError, match="at least two frames"):
        write_trajectory_arrays(tmp_path / "one", mjb_path, times[:1], ends[:1], qpos[:1], qvel[:1], wheels[:1])

    with pytest.raises(ValueError, match="nonnegative"):
        write_trajectory_arrays(
            tmp_path / "neg_t", mjb_path, np.array([-0.01, 0.01, 0.02]), ends, qpos, qvel, wheels
        )

    with pytest.raises(ValueError, match="strictly increasing"):
        write_trajectory_arrays(
            tmp_path / "noninc", mjb_path, np.array([0.0, 0.01, 0.01]), ends, qpos, qvel, wheels
        )

    with pytest.raises(ValueError, match="ends_s must exceed times_s"):
        write_trajectory_arrays(
            tmp_path / "bad_end", mjb_path, times, np.array([0.01, 0.02, 0.02]), qpos, qvel, wheels
        )

    for bad_val in (np.nan, np.inf):
        bad_qpos = qpos.copy()
        bad_qpos[1, 0] = bad_val
        with pytest.raises(ValueError, match="qpos contains nonfinite"):
            write_trajectory_arrays(tmp_path / "bad_qpos", mjb_path, times, ends, bad_qpos, qvel, wheels)

        bad_qvel = qvel.copy()
        bad_qvel[1, 0] = bad_val
        with pytest.raises(ValueError, match="qvel contains nonfinite"):
            write_trajectory_arrays(tmp_path / "bad_qvel", mjb_path, times, ends, qpos, bad_qvel, wheels)

        bad_wheels = wheels.copy()
        bad_wheels[1, 0] = bad_val
        with pytest.raises(ValueError, match="wheels contains nonfinite"):
            write_trajectory_arrays(tmp_path / "bad_wheels", mjb_path, times, ends, qpos, qvel, bad_wheels)

        bad_times = times.copy()
        bad_times[1] = bad_val
        with pytest.raises(ValueError, match="times_s contains nonfinite"):
            write_trajectory_arrays(tmp_path / "bad_times", mjb_path, bad_times, ends, qpos, qvel, wheels)

        bad_ends = ends.copy()
        bad_ends[1] = bad_val
        with pytest.raises(ValueError, match="ends_s contains nonfinite"):
            write_trajectory_arrays(tmp_path / "bad_ends", mjb_path, times, bad_ends, qpos, qvel, wheels)
