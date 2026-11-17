#!/usr/bin/env python3
"""Standalone 5-lane FP32 attribution runner for SP2-B GPU Golden Bundles.

Reads `step_closure.bin` (BGSC0001) + `model.xml` + `model_12gears.xml` +
`hfield_data.bin` and decomposes 1-step and 80-step (0.1 s) parity across:
  1. cpp_f64        : MuJoCo 3.12.0 FP64 (.deps/mujoco-3.12.0)
  2. cpp_f64_in_f32 : MuJoCo 3.12.0 FP64 with float32-quantized inputs
  3. cpp_f32        : MuJoCo 3.12.0 FP32 (-DmjUSESINGLE -ffp-contract=off)
  4. mjwarp_cpu     : mujoco_warp clean_split on cpu (when available)
  5. mjwarp_metal   : mujoco_warp clean_split on metal:0 (when available)
  6. mjwarp_cuda    : mujoco_warp clean_split on cuda:0 (when available)

If BIKE_MUJOCO_F32_ROOT / .worktrees/fp32-physics/third-party/mujoco-312-install-f32
is absent on a machine, only the cpp_f32 lane is skipped gracefully.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

HEADER_STRUCT = struct.Struct("<8sIIQIIIIIIiiiiiIQ")
assert HEADER_STRUCT.size == 80
MAGIC = b"BGSC0001"


@dataclass(frozen=True)
class StepClosureTrace:
    """Decoded BGSC0001 step_closure.bin + manifest."""

    bundle_dir: Path
    manifest: dict[str, Any]
    n_steps: int
    row_doubles: int
    nq: int
    nv: int
    na: int
    nu: int
    ntendon: int
    front_wheel_dof: int
    rear_wheel_dof: int
    freehub_wrap_adr: int
    freehub_tendon_id: int
    clutch_tendon_id: int
    dt_s: float
    step_index: np.ndarray
    time_s: np.ndarray
    qpos: np.ndarray
    qvel: np.ndarray
    act: np.ndarray
    warmstart: np.ndarray
    ctrl: np.ndarray
    qfrc_applied: np.ndarray
    qfrc_tires: np.ndarray
    qfrc_suspension: np.ndarray
    qfrc_resistance: np.ndarray
    qfrc_rider_joint_envelope: np.ndarray
    qfrc_brake_hold: np.ndarray
    qfrc_injected_other: np.ndarray
    dof_frictionloss_wheels: np.ndarray
    wrap_prm_freehub: np.ndarray
    tendon_range: np.ndarray
    cog_index: np.ndarray
    ncon: np.ndarray
    nefc: np.ndarray
    solver_niter: np.ndarray
    next_qpos: np.ndarray
    next_qvel: np.ndarray
    next_qacc: np.ndarray


def load_step_closure_trace(bundle_dir: Path) -> StepClosureTrace:
    """Load and validate step_closure.bin and manifest.json from bundle_dir."""
    bundle_dir = Path(bundle_dir)
    manifest = json.loads((bundle_dir / "manifest.json").read_text())
    raw_bytes = (bundle_dir / "step_closure.bin").read_bytes()
    if len(raw_bytes) < HEADER_STRUCT.size:
        raise ValueError("step_closure.bin shorter than 80-byte header")
    (
        magic,
        version,
        header_bytes,
        n_steps,
        row_doubles,
        nq,
        nv,
        na,
        nu,
        ntendon,
        fw_dof,
        rw_dof,
        wrap_adr,
        fh_tid,
        cl_tid,
        _res0,
        _res1,
    ) = HEADER_STRUCT.unpack_from(raw_bytes, 0)
    if magic != MAGIC or version != 1 or header_bytes != HEADER_STRUCT.size:
        raise ValueError(f"Unexpected BGSC header: magic={magic!r}, version={version}")
    arr = np.frombuffer(raw_bytes, dtype="<f8", offset=header_bytes)
    if arr.size != n_steps * row_doubles:
        raise ValueError("step_closure.bin payload size mismatch")
    mat = arr.reshape((n_steps, row_doubles))

    off = 0

    def take(width: int) -> np.ndarray:
        nonlocal off
        s = mat[:, off : off + width]
        off += width
        return s

    step_index = take(1)[:, 0].astype(np.int64)
    time_s = take(1)[:, 0]
    dt_arr = take(1)[:, 0]
    qpos = take(nq)
    qvel = take(nv)
    act = take(na)
    warmstart = take(nv)
    ctrl = take(nu)
    qfrc_applied = take(nv)
    qfrc_tires = take(nv)
    qfrc_suspension = take(nv)
    qfrc_resistance = take(nv)
    qfrc_env = take(nv)
    qfrc_brake = take(nv)
    qfrc_other = take(nv)
    dfl_wheels = take(2)
    wrap_prm = take(1)[:, 0]
    tendon_range = take(2 * ntendon).reshape((n_steps, ntendon, 2))
    cog_index = take(1)[:, 0].astype(np.int64)
    ncon = take(1)[:, 0].astype(np.int64)
    nefc = take(1)[:, 0].astype(np.int64)
    solver_niter = take(1)[:, 0].astype(np.int64)
    next_qpos = take(nq)
    next_qvel = take(nv)
    next_qacc = take(nv)
    if off != row_doubles:
        raise ValueError(f"Decoded {off} doubles, expected {row_doubles}")

    return StepClosureTrace(
        bundle_dir=bundle_dir,
        manifest=manifest,
        n_steps=int(n_steps),
        row_doubles=int(row_doubles),
        nq=int(nq),
        nv=int(nv),
        na=int(na),
        nu=int(nu),
        ntendon=int(ntendon),
        front_wheel_dof=int(fw_dof),
        rear_wheel_dof=int(rw_dof),
        freehub_wrap_adr=int(wrap_adr),
        freehub_tendon_id=int(fh_tid),
        clutch_tendon_id=int(cl_tid),
        dt_s=float(np.median(dt_arr)),
        step_index=step_index,
        time_s=time_s,
        qpos=qpos,
        qvel=qvel,
        act=act,
        warmstart=warmstart,
        ctrl=ctrl,
        qfrc_applied=qfrc_applied,
        qfrc_tires=qfrc_tires,
        qfrc_suspension=qfrc_suspension,
        qfrc_resistance=qfrc_resistance,
        qfrc_rider_joint_envelope=qfrc_env,
        qfrc_brake_hold=qfrc_brake,
        qfrc_injected_other=qfrc_other,
        dof_frictionloss_wheels=dfl_wheels,
        wrap_prm_freehub=wrap_prm,
        tendon_range=tendon_range,
        cog_index=cog_index,
        ncon=ncon,
        nefc=nefc,
        solver_niter=solver_niter,
        next_qpos=next_qpos,
        next_qvel=next_qvel,
        next_qacc=next_qacc,
    )


def find_repo_root() -> Path:
    cur = Path(__file__).resolve().parent
    for parent in [cur, *cur.parents]:
        if (parent / "CMakeLists.txt").is_file() and (parent / ".deps").is_dir():
            return parent
    raise RuntimeError("Cannot locate repository root containing CMakeLists.txt and .deps")


def find_mujoco_library(root: Path) -> Path:
    lib_dir = root / "lib"
    for name in (
        "libmujoco.3.12.0.dylib",
        "libmujoco.dylib",
        "libmujoco.so.3.12.0",
        "libmujoco.so",
    ):
        cand = lib_dir / name
        if cand.exists():
            return cand
    matches = sorted(lib_dir.glob("libmujoco*"))
    if not matches:
        raise FileNotFoundError(f"No libmujoco shared library found under {lib_dir}")
    return matches[0]


def find_mujoco_f32_root(repo_root: Path) -> Path | None:
    env = os.environ.get("BIKE_MUJOCO_F32_ROOT")
    if env:
        p = Path(env)
        if (p / "include" / "mujoco" / "mujoco.h").is_file():
            return p
    for base in (repo_root, repo_root.parent.parent):
        cand = base / ".worktrees" / "fp32-physics" / "third-party" / "mujoco-312-install-f32"
        if (cand / "include" / "mujoco" / "mujoco.h").is_file():
            return cand
    return None


def compile_runner(
    source_cpp: Path,
    mujoco_root: Path,
    out_binary: Path,
    *,
    f32: bool,
) -> Path:
    cxx = os.environ.get("CXX") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise RuntimeError("No C++ compiler found (clang++ / g++)")
    lib_path = find_mujoco_library(mujoco_root)
    cmd = [
        cxx,
        "-std=c++23",
        "-O2",
        "-ffp-contract=off",
        "-Wall",
        "-Wextra",
        "-Werror",
    ]
    if f32:
        cmd.append("-DmjUSESINGLE")
    cmd += [
        f"-I{mujoco_root / 'include'}",
        str(source_cpp),
        str(lib_path),
        f"-Wl,-rpath,{lib_path.parent}",
    ]
    if sys.platform == "darwin":
        cmd.append("-Wl,-headerpad_max_install_names")
    cmd += ["-o", str(out_binary)]
    subprocess.run(cmd, check=True)
    if sys.platform == "darwin":
        otool = subprocess.run(
            ["otool", "-L", str(out_binary)],
            check=True,
            capture_output=True,
            text=True,
        )
        for line in otool.stdout.splitlines()[1:]:
            dep = line.strip().split(" ", 1)[0]
            if dep.startswith("@rpath/") and "libmujoco" in dep:
                subprocess.run(
                    ["install_name_tool", "-change", dep, str(lib_path.resolve()), str(out_binary)],
                    check=True,
                )
    return out_binary


def compile_runners(build_dir: Path, repo_root: Path) -> dict[str, Path]:
    build_dir.mkdir(parents=True, exist_ok=True)
    src = Path(__file__).resolve().parent / "bundle_runner.cpp"
    f64_root = repo_root / ".deps" / "mujoco-3.12.0"
    runners = {
        "f64": compile_runner(src, f64_root, build_dir / "bundle_runner_f64", f32=False),
    }
    f32_root = find_mujoco_f32_root(repo_root)
    if f32_root is not None:
        runners["f32"] = compile_runner(
            src, f32_root, build_dir / "bundle_runner_f32", f32=True
        )
    return runners


def run_cpp_lane(
    binary: Path,
    tr: StepClosureTrace,
    model_path: Path,
    jobs: list[tuple[int, int]],
    mode: str,
    scratch_dir: Path,
    *,
    quantize_f32: bool = False,
    use_ratio: bool = False,
    reconstruct_forces: bool = False,
) -> dict[str, Any]:
    tag = f"{binary.name}_{mode}_{'q32' if quantize_f32 else 'raw'}_{'ratio' if use_ratio else '12g'}"
    jobs_path = scratch_dir / f"jobs_{tag}.bin"
    out_path = scratch_dir / f"out_{tag}.bin"
    with open(jobs_path, "wb") as f:
        for a, b in jobs:
            f.write(struct.pack("<ii", int(a), int(b)))
    cmd = [
        str(binary),
        "--model",
        str(model_path),
        "--hfield",
        str(tr.bundle_dir / "hfield_data.bin"),
        "--trace",
        str(tr.bundle_dir / "step_closure.bin"),
        "--jobs",
        str(jobs_path),
        "--out",
        str(out_path),
        "--mode",
        mode,
    ]
    if quantize_f32:
        cmd.append("--quantize-f32")
    if use_ratio:
        cmd.append("--use-ratio")
    if reconstruct_forces:
        cmd.append("--reconstruct-forces")
    t0 = time.perf_counter()
    subprocess.run(cmd, check=True)
    elapsed_s = time.perf_counter() - t0
    raw = np.fromfile(out_path, dtype="<f8").reshape(len(jobs), 5 + tr.nq + tr.nv)
    return {
        "elapsed_s": elapsed_s,
        "ncon": raw[:, 2].astype(np.int64),
        "nefc": raw[:, 3].astype(np.int64),
        "niter": raw[:, 4].astype(np.int64),
        "qpos": raw[:, 5 : 5 + tr.nq].copy(),
        "qvel": raw[:, 5 + tr.nq :].copy(),
    }


def _mjwarp_modules():
    import mujoco
    import mujoco_warp as mjw
    import warp as wp
    from mujoco_warp._src import collision_driver as mjw_collision
    from mujoco_warp._src import constraint as mjw_constraint
    from mujoco_warp._src import forward as mjw_fwd
    from mujoco_warp._src import smooth as mjw_smooth
    from mujoco_warp._src import solver as mjw_solver

    return (
        mujoco,
        mjw,
        wp,
        mjw_fwd,
        mjw_smooth,
        mjw_collision,
        mjw_constraint,
        mjw_solver,
    )


def _mjwarp_clean_split_step(m_wp, d_wp, mods) -> None:
    _, _, _, mjw_fwd, mjw_smooth, mjw_collision, mjw_constraint, mjw_solver = mods
    mjw_fwd.fwd_kinematics(m_wp, d_wp)
    mjw_smooth.crb(m_wp, d_wp)
    mjw_smooth.tendon_armature(m_wp, d_wp)
    mjw_smooth.factor_m(m_wp, d_wp)
    mjw_collision.collision(m_wp, d_wp)
    mjw_smooth.transmission(m_wp, d_wp)
    mjw_fwd.fwd_velocity(m_wp, d_wp)
    mjw_constraint.make_constraint(m_wp, d_wp)
    mjw_fwd.fwd_actuation(m_wp, d_wp)
    mjw_fwd.fwd_acceleration(m_wp, d_wp, factorize=False)
    mjw_solver.solve(m_wp, d_wp)
    mjw_fwd.implicit(m_wp, d_wp)


def _make_mjwarp_inputs(
    tr: StepClosureTrace, rows: np.ndarray, cogs: np.ndarray, nt12: int, clutch_tid: int
) -> dict[str, np.ndarray]:
    nw = len(rows)
    trng = np.tile(np.array([[-1e12, 1e12]], np.float32), (nw, nt12, 1))
    for w in range(nw):
        cog = int(cogs[w])
        if 0 <= cog < nt12 - (1 if clutch_tid >= 0 else 0):
            trng[w, cog] = tr.tendon_range[rows[w], 0].astype(np.float32)
        if clutch_tid >= 0 and tr.ntendon >= 2:
            trng[w, clutch_tid] = tr.tendon_range[rows[w], 1].astype(np.float32)
    dfl = np.zeros((nw, tr.nv), dtype=np.float32)
    dfl[:, tr.front_wheel_dof] = tr.dof_frictionloss_wheels[rows, 0].astype(np.float32)
    dfl[:, tr.rear_wheel_dof] = tr.dof_frictionloss_wheels[rows, 1].astype(np.float32)
    return {
        "qpos": tr.qpos[rows].astype(np.float32),
        "qvel": tr.qvel[rows].astype(np.float32),
        "ws": tr.warmstart[rows].astype(np.float32),
        "ctrl": tr.ctrl[rows].astype(np.float32),
        "qfa": tr.qfrc_applied[rows].astype(np.float32),
        "dfl": dfl,
        "trng": trng,
    }


def run_mjwarp_1step(
    m_mj, tr: StepClosureTrace, rows: list[int], cogs: list[int], device: str
) -> dict[str, Any]:
    mujoco, mjw, wp, *_ = mods = _mjwarp_modules()
    rows_arr = np.asarray(rows, dtype=np.int64)
    cogs_arr = np.asarray(cogs, dtype=np.int64)
    clutch_tid = mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_TENDON, "crank_clutch")
    inp = _make_mjwarp_inputs(tr, rows_arr, cogs_arr, m_mj.ntendon, clutch_tid)

    t0 = time.perf_counter()
    with wp.ScopedDevice(device):
        m_wp = mjw.put_model(m_mj)
        d = mjw.make_data(m_mj, nworld=len(rows))
        d.qpos.assign(wp.array(inp["qpos"], device=device))
        d.qvel.assign(wp.array(inp["qvel"], device=device))
        d.qacc_warmstart.assign(wp.array(inp["ws"], device=device))
        d.ctrl.assign(wp.array(inp["ctrl"], device=device))
        d.qfrc_applied.assign(wp.array(inp["qfa"], device=device))
        m_wp.tendon_range = wp.array(inp["trng"], dtype=wp.vec2f, device=device)
        m_wp.dof_frictionloss = wp.array(inp["dfl"], device=device)
        _mjwarp_clean_split_step(m_wp, d, mods)
        wp.synchronize()
        return {
            "elapsed_s": time.perf_counter() - t0,
            "ncon": np.full(len(rows), -1, dtype=np.int64),
            "nefc": d.nefc.numpy().copy().astype(np.int64),
            "niter": d.solver_niter.numpy().copy().astype(np.int64),
            "qpos": d.qpos.numpy().copy().astype(np.float64),
            "qvel": d.qvel.numpy().copy().astype(np.float64),
        }


def run_mjwarp_window(
    m_mj, tr: StepClosureTrace, anchors: list[int], nsteps: int, device: str
) -> dict[str, Any]:
    mujoco, mjw, wp, *_ = mods = _mjwarp_modules()
    init_rows = np.asarray(anchors, dtype=np.int64)
    clutch_tid = mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_TENDON, "crank_clutch")
    inp0 = _make_mjwarp_inputs(
        tr, init_rows, tr.cog_index[init_rows], m_mj.ntendon, clutch_tid
    )

    t0 = time.perf_counter()
    with wp.ScopedDevice(device):
        m_wp = mjw.put_model(m_mj)
        d = mjw.make_data(m_mj, nworld=len(anchors))
        d.qpos.assign(wp.array(inp0["qpos"], device=device))
        d.qvel.assign(wp.array(inp0["qvel"], device=device))
        d.qacc_warmstart.assign(wp.array(inp0["ws"], device=device))
        for s in range(nsteps):
            rows = init_rows + s
            inp = _make_mjwarp_inputs(
                tr, rows, tr.cog_index[rows], m_mj.ntendon, clutch_tid
            )
            d.ctrl.assign(wp.array(inp["ctrl"], device=device))
            d.qfrc_applied.assign(wp.array(inp["qfa"], device=device))
            m_wp.tendon_range = wp.array(inp["trng"], dtype=wp.vec2f, device=device)
            m_wp.dof_frictionloss = wp.array(inp["dfl"], device=device)
            _mjwarp_clean_split_step(m_wp, d, mods)
        wp.synchronize()
        return {
            "elapsed_s": time.perf_counter() - t0,
            "ncon": np.full(len(anchors), -1, dtype=np.int64),
            "nefc": d.nefc.numpy().copy().astype(np.int64),
            "niter": d.solver_niter.numpy().copy().astype(np.int64),
            "qpos": d.qpos.numpy().copy().astype(np.float64),
            "qvel": d.qvel.numpy().copy().astype(np.float64),
        }


def compute_pair_stats(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    dqp = np.abs(a["qpos"] - b["qpos"]).max(axis=1)
    dqv = np.abs(a["qvel"] - b["qvel"]).max(axis=1)
    return {
        "dqpos_p50": float(np.median(dqp)),
        "dqpos_p95": float(np.percentile(dqp, 95)),
        "dqpos_max": float(dqp.max()),
        "dqvel_p50": float(np.median(dqv)),
        "dqvel_p95": float(np.percentile(dqv, 95)),
        "dqvel_max": float(dqv.max()),
        "nefc_match": float((a["nefc"] == b["nefc"]).mean()),
        "niter_mean_a": float(a["niter"].mean()),
        "niter_mean_b": float(b["niter"].mean()),
        "argmax_dqvel_idx": int(np.argmax(dqv)),
    }


def available_mjwarp_devices() -> list[str]:
    try:
        _, _, wp, *_ = _mjwarp_modules()
        wp.init()
        return [d.alias for d in wp.get_devices()]
    except Exception:
        return []


def run_attribution(
    bundle_dir: Path,
    *,
    max_parity_rows: int = 100,
    window_steps: int = 80,
    build_dir: Path | None = None,
    enable_mjwarp: bool = True,
) -> dict[str, Any]:
    repo_root = find_repo_root()
    tr = load_step_closure_trace(bundle_dir)

    with tempfile.TemporaryDirectory(prefix="bike-fp32-attr-") as tmp_str:
        tmp_dir = Path(tmp_str)
        runner_dir = build_dir if build_dir is not None else tmp_dir / "runners"
        runners = compile_runners(runner_dir, repo_root)

        rows_1step = np.linspace(
            0, tr.n_steps - 1, min(max_parity_rows, tr.n_steps), dtype=int
        ).tolist()
        jobs_ratio = [(r, -1) for r in rows_1step]
        cogs_matched = [int(tr.cog_index[r]) for r in rows_1step]
        jobs_12g = list(zip(rows_1step, cogs_matched))

        # C++ ratio path (model.xml) covers shifting + steady steps cleanly.
        ratio_lanes: dict[str, dict[str, Any]] = {
            "cpp_f64": run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model.xml", jobs_ratio, "parity", tmp_dir,
                quantize_f32=False, use_ratio=True,
            ),
            "cpp_f64_in_f32": run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model.xml", jobs_ratio, "parity", tmp_dir,
                quantize_f32=True, use_ratio=True,
            ),
        }
        if "f32" in runners:
            ratio_lanes["cpp_f32"] = run_cpp_lane(
                runners["f32"], tr, bundle_dir / "model.xml", jobs_ratio, "parity", tmp_dir,
                quantize_f32=False, use_ratio=True,
            )

        # Matched steady-cog rows on model_12gears.xml across all 5/6 lanes.
        steady_mask = []
        front_teeth = int(tr.manifest.get("front_teeth", 22))
        cassette = [int(t) for t in tr.manifest.get("cassette", [])]
        for r in rows_1step:
            cog = int(tr.cog_index[r])
            if 0 <= cog < len(cassette):
                steady_ratio = front_teeth / cassette[cog]
                steady_mask.append(abs(float(tr.wrap_prm_freehub[r]) - steady_ratio) <= 1e-12)
            else:
                steady_mask.append(False)
        steady_rows = [r for r, ok in zip(rows_1step, steady_mask) if ok]
        if not steady_rows:
            steady_rows = rows_1step[:1]
        steady_cogs = [int(tr.cog_index[r]) for r in steady_rows]
        steady_jobs = list(zip(steady_rows, steady_cogs))

        lanes_12g: dict[str, dict[str, Any]] = {
            "cpp_f64": run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model_12gears.xml", steady_jobs, "parity", tmp_dir,
                quantize_f32=False, use_ratio=False,
            ),
            "cpp_f64_in_f32": run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model_12gears.xml", steady_jobs, "parity", tmp_dir,
                quantize_f32=True, use_ratio=False,
            ),
        }
        if "f32" in runners:
            lanes_12g["cpp_f32"] = run_cpp_lane(
                runners["f32"], tr, bundle_dir / "model_12gears.xml", steady_jobs, "parity", tmp_dir,
                quantize_f32=False, use_ratio=False,
            )

        devices = available_mjwarp_devices() if enable_mjwarp else []
        if devices:
            import mujoco

            m12 = mujoco.MjModel.from_xml_path(str(bundle_dir / "model_12gears.xml"))
            m12.hfield_data[:] = np.fromfile(bundle_dir / "hfield_data.bin", dtype="<f8")
            for dev, key in [("cpu", "mjwarp_cpu"), ("metal:0", "mjwarp_metal"), ("cuda:0", "mjwarp_cuda")]:
                if dev in devices:
                    lanes_12g[key] = run_mjwarp_1step(m12, tr, steady_rows, steady_cogs, dev)

        # 80-step (0.1 s) closed-loop windowed rollouts.
        max_anchor = tr.n_steps - window_steps
        anchors_ratio = [a for a in (0, 150, max(0, max_anchor)) if 0 <= a <= max_anchor]
        anchors_ratio = sorted(set(anchors_ratio))
        window_ratio_lanes: dict[str, dict[str, Any]] = {}
        window_12g_lanes: dict[str, dict[str, Any]] = {}
        if anchors_ratio:
            wjobs = [(a, window_steps) for a in anchors_ratio]
            window_ratio_lanes["cpp_f64"] = run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model.xml", wjobs, "window", tmp_dir,
                quantize_f32=False, use_ratio=True,
            )
            window_ratio_lanes["cpp_f64_in_f32"] = run_cpp_lane(
                runners["f64"], tr, bundle_dir / "model.xml", wjobs, "window", tmp_dir,
                quantize_f32=True, use_ratio=True,
            )
            if "f32" in runners:
                window_ratio_lanes["cpp_f32"] = run_cpp_lane(
                    runners["f32"], tr, bundle_dir / "model.xml", wjobs, "window", tmp_dir,
                    quantize_f32=False, use_ratio=True,
                )

            # Find steady-cog windows for model_12gears.xml across C++ and mjwarp.
            steady_anchors: list[int] = []
            for a in anchors_ratio:
                ok_win = True
                for s in range(window_steps):
                    rk = a + s
                    cog = int(tr.cog_index[rk])
                    if not (0 <= cog < len(cassette)) or abs(float(tr.wrap_prm_freehub[rk]) - front_teeth / cassette[cog]) > 1e-12:
                        ok_win = False
                        break
                if ok_win:
                    steady_anchors.append(a)
            if steady_anchors:
                wjobs_12g = [(a, window_steps) for a in steady_anchors]
                window_12g_lanes["cpp_f64"] = run_cpp_lane(
                    runners["f64"], tr, bundle_dir / "model_12gears.xml", wjobs_12g, "window", tmp_dir,
                    quantize_f32=False, use_ratio=False,
                )
                window_12g_lanes["cpp_f64_in_f32"] = run_cpp_lane(
                    runners["f64"], tr, bundle_dir / "model_12gears.xml", wjobs_12g, "window", tmp_dir,
                    quantize_f32=True, use_ratio=False,
                )
                if "f32" in runners:
                    window_12g_lanes["cpp_f32"] = run_cpp_lane(
                        runners["f32"], tr, bundle_dir / "model_12gears.xml", wjobs_12g, "window", tmp_dir,
                        quantize_f32=False, use_ratio=False,
                    )
                if devices:
                    import mujoco

                    m12 = mujoco.MjModel.from_xml_path(str(bundle_dir / "model_12gears.xml"))
                    m12.hfield_data[:] = np.fromfile(bundle_dir / "hfield_data.bin", dtype="<f8")
                    for dev, key in [("cpu", "mjwarp_cpu"), ("metal:0", "mjwarp_metal"), ("cuda:0", "mjwarp_cuda")]:
                        if dev in devices:
                            window_12g_lanes[key] = run_mjwarp_window(
                                m12, tr, steady_anchors, window_steps, dev
                            )

    def build_pairs(lanes: dict[str, dict[str, Any]]) -> dict[str, Any]:
        ordered = [
            ("cpp_f64", "cpp_f64_in_f32"),
            ("cpp_f64_in_f32", "cpp_f32"),
            ("cpp_f64", "cpp_f32"),
            ("cpp_f32", "mjwarp_cpu"),
            ("mjwarp_cpu", "mjwarp_metal"),
            ("mjwarp_cpu", "mjwarp_cuda"),
            ("cpp_f32", "mjwarp_metal"),
            ("cpp_f64", "mjwarp_cpu"),
            ("cpp_f64", "mjwarp_metal"),
            ("cpp_f64", "mjwarp_cuda"),
        ]
        return {
            f"{ka}_vs_{kb}": compute_pair_stats(lanes[ka], lanes[kb])
            for ka, kb in ordered
            if ka in lanes and kb in lanes
        }

    return {
        "bundle": str(bundle_dir),
        "steps": tr.n_steps,
        "dt_s": tr.dt_s,
        "lanes_available": {
            "ratio_1step": list(ratio_lanes.keys()),
            "steady_12gears_1step": list(lanes_12g.keys()),
            "window_ratio": list(window_ratio_lanes.keys()),
            "window_12gears": list(window_12g_lanes.keys()),
        },
        "one_step_ratio": {
            "n_rows": len(jobs_ratio),
            "timings_s": {k: round(v["elapsed_s"], 4) for k, v in ratio_lanes.items()},
            "pairs": build_pairs(ratio_lanes),
        },
        "one_step_steady_12gears": {
            "n_rows": len(steady_jobs),
            "timings_s": {k: round(v["elapsed_s"], 4) for k, v in lanes_12g.items()},
            "pairs": build_pairs(lanes_12g),
        },
        "window_ratio": {
            "window_steps": window_steps,
            "anchors": anchors_ratio,
            "pairs": build_pairs(window_ratio_lanes),
        },
        "window_steady_12gears": {
            "window_steps": window_steps,
            "pairs": build_pairs(window_12g_lanes),
        },
    }


def format_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# FP32 Attribution Report",
        "",
        f"- **Bundle:** `{report['bundle']}`",
        f"- **Steps:** `{report['steps']}` (`dt = {report['dt_s']} s`)",
        f"- **1-Step Steady Lanes:** `{', '.join(report['lanes_available']['steady_12gears_1step'])}`",
        "",
        "## 1-Step Attribution (`model_12gears.xml`, steady cog)",
        "",
        "| Pair | `dqpos` p50 / p95 / max | `dqvel` p50 / p95 / max | `nefc` match |",
        "|---|---:|---:|---:|",
    ]
    for name, s in report["one_step_steady_12gears"]["pairs"].items():
        lines.append(
            f"| `{name}` | `{s['dqpos_p50']:.2e}` / `{s['dqpos_p95']:.2e}` / `{s['dqpos_max']:.2e}` "
            f"| `{s['dqvel_p50']:.2e}` / `{s['dqvel_p95']:.2e}` / `{s['dqvel_max']:.2e}` "
            f"| `{s['nefc_match']:.4f}` |"
        )
    lines += [
        "",
        "## 1-Step Ratio Attribution (`model.xml --use-ratio`)",
        "",
        "| Pair | `dqpos` p50 / p95 / max | `dqvel` p50 / p95 / max | `nefc` match |",
        "|---|---:|---:|---:|",
    ]
    for name, s in report["one_step_ratio"]["pairs"].items():
        lines.append(
            f"| `{name}` | `{s['dqpos_p50']:.2e}` / `{s['dqpos_p95']:.2e}` / `{s['dqpos_max']:.2e}` "
            f"| `{s['dqvel_p50']:.2e}` / `{s['dqvel_p95']:.2e}` / `{s['dqvel_max']:.2e}` "
            f"| `{s['nefc_match']:.4f}` |"
        )
    if report["window_steady_12gears"]["pairs"]:
        lines += [
            "",
            f"## {report['window_steady_12gears']['window_steps']}-Step Windowed Rollout (`model_12gears.xml`)",
            "",
            "| Pair | `dqpos` max | `dqvel` max | `nefc` match |",
            "|---|---:|---:|---:|",
        ]
        for name, s in report["window_steady_12gears"]["pairs"].items():
            lines.append(
                f"| `{name}` | `{s['dqpos_max']:.2e}` | `{s['dqvel_max']:.2e}` | `{s['nefc_match']:.4f}` |"
            )
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description="Run 5-lane FP32 attribution on a Golden Bundle.")
    ap.add_argument("--bundle", type=Path, required=True)
    ap.add_argument("--max-rows", type=int, default=100)
    ap.add_argument("--window-steps", type=int, default=80)
    ap.add_argument("--no-mjwarp", action="store_true")
    ap.add_argument("--out-json", type=Path)
    ap.add_argument("--out-md", type=Path)
    args = ap.parse_args()

    report = run_attribution(
        args.bundle,
        max_parity_rows=args.max_rows,
        window_steps=args.window_steps,
        enable_mjwarp=not args.no_mjwarp,
    )
    md = format_markdown(report)
    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(report, indent=2))
    if args.out_md:
        args.out_md.parent.mkdir(parents=True, exist_ok=True)
        args.out_md.write_text(md)
    print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
