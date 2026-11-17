"""Write and read the BGTRAJ01 directory consumed by `bike-replay --view --trajectory`."""
from __future__ import annotations

import hashlib
import json
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

MAGIC = b"BGTRAJ01"
VERSION = 1
WHEEL_COLS = 8
_HEADER_STRUCT = struct.Struct("<8sIIIIQ64s")
assert _HEADER_STRUCT.size == 96


@dataclass(frozen=True)
class TrajectoryData:
    """Parsed BGTRAJ01 trajectory payload."""

    version: int
    nq: int
    nv: int
    wheel_cols: int
    frames: int
    model_sha256: str
    times_s: np.ndarray
    ends_s: np.ndarray
    qpos: np.ndarray
    qvel: np.ndarray
    wheels: np.ndarray
    metadata: dict[str, Any] | None = None


def write_trajectory_arrays(
    out_dir: Path,
    model_mjb_path: Path,
    times_s: np.ndarray,
    ends_s: np.ndarray,
    qpos: np.ndarray,
    qvel: np.ndarray,
    wheels: np.ndarray | None = None,
    metadata: dict[str, Any] | None = None,
) -> None:
    """Write `trajectory.bin`, `model.mjb`, and `trajectory.json` into `out_dir`."""
    out_dir = Path(out_dir)
    model_mjb_path = Path(model_mjb_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    mjb = model_mjb_path.read_bytes()
    dst_mjb = out_dir / "model.mjb"
    if model_mjb_path.resolve() != dst_mjb.resolve():
        shutil.copyfile(model_mjb_path, dst_mjb)
    sha_str = hashlib.sha256(mjb).hexdigest()
    sha = sha_str.encode("ascii")

    times = np.asarray(times_s, dtype="<f8")
    ends = np.asarray(ends_s, dtype="<f8")
    qpos_arr = np.asarray(qpos, dtype="<f8")
    qvel_arr = np.asarray(qvel, dtype="<f8")

    if times.ndim != 1 or ends.ndim != 1 or times.shape != ends.shape:
        raise ValueError("times_s and ends_s must be 1-D arrays with equal length")
    frames = int(times.shape[0])
    if qpos_arr.ndim != 2 or qvel_arr.ndim != 2:
        raise ValueError("qpos and qvel must be 2-D arrays")
    if qpos_arr.shape[0] != frames or qvel_arr.shape[0] != frames:
        raise ValueError("qpos and qvel row counts must match times_s")

    nq = int(qpos_arr.shape[1])
    nv = int(qvel_arr.shape[1])

    if wheels is None:
        wheel_rows = np.zeros((frames, WHEEL_COLS), dtype="<f8")
    else:
        wheel_rows = np.asarray(wheels, dtype="<f8")
        if wheel_rows.ndim == 3 and wheel_rows.shape == (frames, 2, 4):
            wheel_rows = wheel_rows.reshape(frames, WHEEL_COLS)
        if wheel_rows.shape != (frames, WHEEL_COLS):
            raise ValueError(f"wheels must have shape ({frames}, {WHEEL_COLS}), got {wheel_rows.shape}")

    header = _HEADER_STRUCT.pack(MAGIC, VERSION, nq, nv, WHEEL_COLS, frames, sha)
    rows = np.hstack([times[:, None], ends[:, None], qpos_arr, qvel_arr, wheel_rows]).astype("<f8", copy=False)
    (out_dir / "trajectory.bin").write_bytes(header + rows.tobytes())

    meta = {
        "format": "BGTRAJ01",
        "frames": frames,
        "nq": nq,
        "nv": nv,
        "model_sha256": sha_str,
    }
    if metadata:
        meta.update(metadata)
    (out_dir / "trajectory.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")


def write_trajectory(out_dir: Path, bundle_dir: Path, ep: Any, run: Any) -> None:
    """Write a BGTRAJ01 directory from a golden episode `ep` and GPU `run`."""
    steps = np.concatenate([[0], np.asarray(run.frame_steps, dtype=np.int64)])
    qpos = np.vstack([ep.qpos[0], run.frame_qpos])
    qvel = np.vstack([ep.qvel[0], run.frame_qvel])
    times = np.asarray(ep.times_s, dtype=np.float64)[steps]
    ends = np.append(times[1:], times[-1] + float(ep.dt_s))
    wheel_rows = np.zeros((steps.size, WHEEL_COLS), dtype=np.float64)
    for r, s in enumerate(steps[1:], start=1):
        wheel_rows[r] = np.asarray(run.wheels[int(s) - 1], dtype=np.float64).reshape(WHEEL_COLS)

    write_trajectory_arrays(
        out_dir=out_dir,
        model_mjb_path=Path(bundle_dir) / "model.mjb",
        times_s=times,
        ends_s=ends,
        qpos=qpos,
        qvel=qvel,
        wheels=wheel_rows,
        metadata={
            "mode": run.mode,
            "window_s": run.window_s,
            "frame_stride": run.frame_stride,
        },
    )


def read_trajectory(traj_dir: Path) -> TrajectoryData:
    """Read and validate a BGTRAJ01 directory."""
    traj_dir = Path(traj_dir)
    mjb = (traj_dir / "model.mjb").read_bytes()
    actual_sha = hashlib.sha256(mjb).hexdigest()
    raw = (traj_dir / "trajectory.bin").read_bytes()
    if len(raw) < _HEADER_STRUCT.size:
        raise ValueError("trajectory.bin smaller than 96-byte header")

    magic, version, nq, nv, wheel_cols, frames, sha_bytes = _HEADER_STRUCT.unpack_from(raw, 0)
    if magic != MAGIC:
        raise ValueError(f"invalid trajectory magic: {magic!r}")
    if version != VERSION:
        raise ValueError(f"unsupported trajectory version: {version}")
    if wheel_cols != WHEEL_COLS:
        raise ValueError(f"trajectory wheel_cols must be {WHEEL_COLS}, got {wheel_cols}")

    header_sha = sha_bytes.decode("ascii")
    if header_sha != actual_sha:
        raise ValueError("trajectory model SHA-256 mismatch")

    row_cols = 2 + nq + nv + wheel_cols
    expected_bytes = _HEADER_STRUCT.size + frames * row_cols * 8
    if len(raw) != expected_bytes:
        raise ValueError(f"trajectory.bin size mismatch: expected {expected_bytes}, got {len(raw)}")

    table = np.frombuffer(raw, dtype="<f8", offset=_HEADER_STRUCT.size).reshape(frames, row_cols)
    times_s = table[:, 0].copy()
    ends_s = table[:, 1].copy()
    qpos = table[:, 2 : 2 + nq].copy()
    qvel = table[:, 2 + nq : 2 + nq + nv].copy()
    wheels = table[:, 2 + nq + nv :].copy()

    meta_path = traj_dir / "trajectory.json"
    metadata = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else None
    return TrajectoryData(
        version=version,
        nq=nq,
        nv=nv,
        wheel_cols=wheel_cols,
        frames=frames,
        model_sha256=header_sha,
        times_s=times_s,
        ends_s=ends_s,
        qpos=qpos,
        qvel=qvel,
        wheels=wheels,
        metadata=metadata,
    )
