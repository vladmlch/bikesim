"""Tests for SP2-B Golden Bundle + 5-lane FP32 attribution runner."""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from tools.colab_gpu.golden.trajectory import read_trajectory
from tools.fp32_attribution.build_and_run import (
    available_mjwarp_devices,
    export_lane_ghost_trajectory,
    find_mujoco_f32_root,
    find_repo_root,
    format_markdown,
    load_step_closure_trace,
    run_attribution,
)


@pytest.fixture(scope="module")
def welded_bundle(tmp_path_factory: pytest.TempPathFactory) -> Path:
    repo = find_repo_root()
    env_bin = os.environ.get("BIKE_GPU_EXPORT_BIN")
    gpu_export = (
        Path(env_bin)
        if env_bin
        else repo / "build" / "native-release" / "bin" / "bike-gpu-export"
    )
    if not gpu_export.is_file():
        pytest.skip(f"bike-gpu-export not built at {gpu_export}")
    out_dir = tmp_path_factory.mktemp("sp2b-bundle") / "welded"
    subprocess.run(
        [
            str(gpu_export),
            "--physics-config",
            str(repo / "examples" / "research" / "viewer_physics_welded.toml"),
            "--track",
            str(repo / "examples" / "research" / "rough_uphill_extreme.toml"),
            "--steps",
            "260",
            "--verify",
            "--out",
            str(out_dir),
        ],
        check=True,
    )
    return out_dir


def test_bundle_layout_and_component_sum(welded_bundle: Path) -> None:
    tr = load_step_closure_trace(welded_bundle)
    assert tr.n_steps == 260
    assert tr.nq == 27 and tr.nv == 27 and tr.nu == 12 and tr.ntendon == 2
    assert (welded_bundle / "model_12gears.xml").is_file()
    assert (welded_bundle / "model.mjb").is_file()
    assert (welded_bundle / "physics_config.json").is_file()
    mjb_sha = hashlib.sha256((welded_bundle / "model.mjb").read_bytes()).hexdigest()
    assert tr.manifest["model_mjb_sha256"] == mjb_sha

    comp_sum = (
        tr.qfrc_tires
        + tr.qfrc_suspension
        + tr.qfrc_resistance
        + tr.qfrc_rider_joint_envelope
        + tr.qfrc_brake_hold
        + tr.qfrc_injected_other
    )
    np.testing.assert_allclose(comp_sum, tr.qfrc_applied, rtol=0.0, atol=1e-12)


def test_attribution_cpp_lanes_and_optional_mjwarp(welded_bundle: Path) -> None:
    repo = find_repo_root()
    report = run_attribution(welded_bundle, max_parity_rows=50, window_steps=80)
    md = format_markdown(report)
    assert "FP32 Attribution Report" in md

    ratio_pairs = report["one_step_ratio"]["pairs"]
    assert "cpp_f64_vs_cpp_f64_in_f32" in ratio_pairs
    assert ratio_pairs["cpp_f64_vs_cpp_f64_in_f32"]["nefc_match"] == 1.0
    assert ratio_pairs["cpp_f64_vs_cpp_f64_in_f32"]["dqvel_max"] < 1e-2

    if find_mujoco_f32_root(repo) is not None:
        assert "cpp_f32" in report["lanes_available"]["ratio_1step"]
        assert "cpp_f64_vs_cpp_f32" in ratio_pairs
        assert ratio_pairs["cpp_f64_vs_cpp_f32"]["nefc_match"] == 1.0
        assert ratio_pairs["cpp_f64_vs_cpp_f32"]["dqpos_max"] < 1e-4
        assert ratio_pairs["cpp_f64_vs_cpp_f32"]["dqvel_max"] < 2e-2

        win_ratio = report["window_ratio"]["pairs"]
        assert "cpp_f64_vs_cpp_f32" in win_ratio
        assert win_ratio["cpp_f64_vs_cpp_f32"]["dqpos_max"] < 1e-3
        assert win_ratio["cpp_f64_vs_cpp_f32"]["dqvel_max"] < 5e-2

    devices = available_mjwarp_devices()
    if "cpu" in devices:
        assert "mjwarp_cpu" in report["lanes_available"]["steady_12gears_1step"]
    if "metal:0" in devices:
        assert "mjwarp_metal" in report["lanes_available"]["steady_12gears_1step"]
        p12 = report["one_step_steady_12gears"]["pairs"]
        assert p12["cpp_f64_vs_mjwarp_metal"]["nefc_match"] == 1.0
        assert p12["cpp_f64_vs_mjwarp_metal"]["dqvel_max"] < 1e-1


def test_ghost_trajectory_export_and_replay(welded_bundle: Path, tmp_path: Path) -> None:
    devices = available_mjwarp_devices()
    target_device = next(
        (dev for dev in ("metal:0", "cuda:0", "cpu") if dev in devices),
        None,
    )
    if target_device is None:
        pytest.skip("mujoco_warp device unavailable")

    tr = load_step_closure_trace(welded_bundle)
    traj_dir = tmp_path / "ghost_traj"
    export_lane_ghost_trajectory(
        welded_bundle,
        traj_dir,
        device=target_device,
        start_step=0,
        n_steps=40,
    )

    loaded = read_trajectory(traj_dir)
    assert loaded.frames == 41
    assert loaded.nq == tr.nq == 27
    assert loaded.nv == tr.nv == 27
    assert loaded.times_s[0] == pytest.approx(float(tr.time_s[0]), abs=1e-12)
    assert np.all(np.isfinite(loaded.qpos))
    assert np.all(np.isfinite(loaded.qvel))
    assert np.max(np.abs(loaded.qpos[1] - tr.next_qpos[0])) < 1e-4

    meta = json.loads((traj_dir / "trajectory.json").read_text(encoding="utf-8"))
    assert meta["format"] == "BGTRAJ01"
    assert meta["device"] == target_device
    assert meta["roll_steps"] == 40
    assert (traj_dir / "model.mjb").is_file()
    assert (traj_dir / "trajectory.bin").is_file()
    assert (traj_dir / "model.mjb").read_bytes() == (welded_bundle / "model.mjb").read_bytes()

    # Fallback path when model.mjb is absent must still emit a MuJoCo 3.12.0 2-tendon
    # model.mjb via C++ bundle_runner_f64 (never Python mujoco.mj_saveModel(m12, ...)).
    stripped_bundle = tmp_path / "stripped_bundle"
    shutil.copytree(welded_bundle, stripped_bundle)
    (stripped_bundle / "model.mjb").unlink()
    fallback_traj_dir = tmp_path / "ghost_traj_fallback"
    export_lane_ghost_trajectory(
        stripped_bundle,
        fallback_traj_dir,
        device=target_device,
        start_step=0,
        n_steps=40,
    )
    fallback_loaded = read_trajectory(fallback_traj_dir)
    assert fallback_loaded.frames == 41
    assert (fallback_traj_dir / "model.mjb").is_file()

    # End-to-end C++ bike-replay --view --trajectory --dry-run verification against
    # a matching sealed bike-ride --record run on the same physics-config and track.
    repo = find_repo_root()
    bin_dir = (
        Path(os.environ["BIKE_GPU_EXPORT_BIN"]).parent
        if os.environ.get("BIKE_GPU_EXPORT_BIN")
        else repo / "build" / "native-release" / "bin"
    )
    bike_ride = Path(os.environ.get("BIKE_RIDE_BIN", bin_dir / "bike-ride"))
    bike_replay = Path(os.environ.get("BIKE_REPLAY_BIN", bin_dir / "bike-replay"))
    if not (bike_ride.is_file() and bike_replay.is_file()):
        pytest.skip(f"bike-ride / bike-replay not built under {bin_dir}")

    rec_root = tmp_path / "ride_rec_out"
    ride_res = subprocess.run(
        [
            str(bike_ride),
            "--physics-config",
            str(repo / "examples" / "research" / "viewer_physics_welded.toml"),
            "--track",
            str(repo / "examples" / "research" / "rough_uphill_extreme.toml"),
            "--duration",
            "0.1",
            "--headless",
            "--record",
            "--no-plots",
            "--out",
            str(rec_root),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rec_dirs = [p for p in rec_root.iterdir() if p.is_dir() and (p / "manifest.json").is_file()]
    assert len(rec_dirs) == 1
    rec_dir = rec_dirs[0]

    replay_res = subprocess.run(
        [
            str(bike_replay),
            "--view",
            "--trajectory",
            str(traj_dir),
            "--dry-run",
            str(rec_dir),
        ],
        capture_output=True,
        text=True,
    )
    assert replay_res.returncode == 0, (
        f"bike-replay failed for {traj_dir}:\nstdout={replay_res.stdout}\nstderr={replay_res.stderr}"
    )
    assert "ok" in replay_res.stdout
