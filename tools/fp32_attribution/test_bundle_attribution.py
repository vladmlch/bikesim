"""Tests for SP2-B Golden Bundle + 5-lane FP32 attribution runner."""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import numpy as np
import pytest

from tools.fp32_attribution.build_and_run import (
    available_mjwarp_devices,
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
    assert (welded_bundle / "physics_config.json").is_file()

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
