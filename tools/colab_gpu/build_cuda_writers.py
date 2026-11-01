"""Compile the allocation-free C++/CUDA force writers into a shared library and bind ctypes."""

from __future__ import annotations

import ctypes
import json
from pathlib import Path
import subprocess
from typing import Any, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GPU_SRC_DIR = REPO_ROOT / "native" / "src" / "gpu"
DEFAULT_OUT_SO_PATH = Path("/tmp/libbike_cuda_writers.so")


class GpuModelIndicesStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuModelIndices`` in ``native/src/gpu/common.hpp``."""

    _fields_ = [
        ("nq", ctypes.c_int),
        ("nv", ctypes.c_int),
        ("nu", ctypes.c_int),
        ("nbody", ctypes.c_int),
        # Body IDs
        ("frame_bid", ctypes.c_int),
        ("front_wheel_bid", ctypes.c_int),
        ("rear_wheel_bid", ctypes.c_int),
        # Joint qpos addresses
        ("root_x_qpos", ctypes.c_int),
        ("root_z_qpos", ctypes.c_int),
        ("root_pitch_qpos", ctypes.c_int),
        ("crank_spin_qpos", ctypes.c_int),
        ("fork_travel_qpos", ctypes.c_int),
        ("front_wheel_spin_qpos", ctypes.c_int),
        ("rear_wheel_spin_qpos", ctypes.c_int),
        ("shock_stroke_qpos", ctypes.c_int),
        ("rider_qpos", ctypes.c_int * 6),
        # Joint dof addresses
        ("root_x_dof", ctypes.c_int),
        ("root_z_dof", ctypes.c_int),
        ("root_pitch_dof", ctypes.c_int),
        ("crank_spin_dof", ctypes.c_int),
        ("fork_travel_dof", ctypes.c_int),
        ("front_wheel_spin_dof", ctypes.c_int),
        ("rear_wheel_spin_dof", ctypes.c_int),
        ("shock_stroke_dof", ctypes.c_int),
        ("rider_dof", ctypes.c_int * 6),
        # Actuator ctrl addresses
        ("rider_ctrl", ctypes.c_int * 6),
    ]


def configure_cuda_writers_cdll(lib: ctypes.CDLL) -> ctypes.CDLL:
    """Configure ``.argtypes`` and ``.restype`` for all ``bike_gpu_*`` C API entry points."""
    lib.bike_gpu_sizeof_world_config.argtypes = []
    lib.bike_gpu_sizeof_world_config.restype = ctypes.c_int

    lib.bike_gpu_sizeof_world_state.argtypes = []
    lib.bike_gpu_sizeof_world_state.restype = ctypes.c_int

    lib.bike_gpu_sizeof_model_indices.argtypes = []
    lib.bike_gpu_sizeof_model_indices.restype = ctypes.c_int

    lib.bike_gpu_init_default_config.argtypes = [ctypes.c_void_p]
    lib.bike_gpu_init_default_config.restype = None

    lib.bike_gpu_build_sweep_configs.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.c_int,
    ]
    lib.bike_gpu_build_sweep_configs.restype = None

    lib.bike_gpu_launch_init_states.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    lib.bike_gpu_launch_init_states.restype = None

    lib.bike_gpu_launch_writers.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
    ]
    lib.bike_gpu_launch_writers.restype = None

    return lib


def compile_cuda_writers(
    gpu_src_dir: Path = DEFAULT_GPU_SRC_DIR,
    out_so_path: Path = DEFAULT_OUT_SO_PATH,
) -> ctypes.CDLL:
    """Compile ``cuda_kernels.cu`` with ``nvcc`` and load the resulting shared library via ``ctypes.CDLL``."""
    gpu_src_dir = Path(gpu_src_dir).resolve()
    out_so_path = Path(out_so_path).resolve()
    cu_path = gpu_src_dir / "cuda_kernels.cu"
    if not cu_path.is_file():
        raise FileNotFoundError(f"CUDA kernel source not found: {cu_path}")

    out_so_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "nvcc",
        "-std=c++20",
        "-O3",
        "--use_fast_math",
        "--shared",
        "-Xcompiler",
        "-fPIC",
        str(cu_path),
        "-o",
        str(out_so_path),
    ]
    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"nvcc compilation failed (exit {proc.returncode}):\n"
            f"Command: {' '.join(cmd)}\n"
            f"STDOUT:\n{proc.stdout}\n"
            f"STDERR:\n{proc.stderr}"
        )

    lib = ctypes.CDLL(str(out_so_path))
    return configure_cuda_writers_cdll(lib)


def _lookup_body_id(m: Any, candidate_names: Sequence[str], default_id: int) -> int:
    if m is None:
        return default_id
    try:
        import mujoco

        for name in candidate_names:
            bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name))
            if bid >= 0:
                return bid
    except Exception:
        pass
    return default_id


def _lookup_joint_addrs(
    m: Any,
    candidate_names: Sequence[str],
    default_qpos: int,
    default_dof: int,
) -> tuple[int, int]:
    if m is None:
        return default_qpos, default_dof
    try:
        import mujoco

        for name in candidate_names:
            jid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name))
            if jid >= 0:
                return int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid])
    except Exception:
        pass
    return default_qpos, default_dof


def _lookup_actuator_id(m: Any, candidate_names: Sequence[str], default_id: int) -> int:
    if m is None:
        return default_id
    try:
        import mujoco

        for name in candidate_names:
            aid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            if aid >= 0:
                return aid
    except Exception:
        pass
    return default_id


def resolve_gpu_model_indices(m: Any) -> bytes:
    """Populate the ``GpuModelIndices`` C struct from a ``mujoco.MjModel`` and return its raw bytes."""
    nq = int(getattr(m, "nq", 26)) if m is not None else 26
    nv = int(getattr(m, "nv", 26)) if m is not None else 26
    nu = int(getattr(m, "nu", 12)) if m is not None else 12
    nbody = int(getattr(m, "nbody", 23)) if m is not None else 23

    frame_bid = _lookup_body_id(m, ("frame", "main_frame"), 1)
    front_wheel_bid = _lookup_body_id(m, ("front_wheel",), 7)
    rear_wheel_bid = _lookup_body_id(m, ("rear_wheel",), 10)

    root_x_qpos, root_x_dof = _lookup_joint_addrs(m, ("root_x",), 0, 0)
    root_z_qpos, root_z_dof = _lookup_joint_addrs(m, ("root_z",), 1, 1)
    root_pitch_qpos, root_pitch_dof = _lookup_joint_addrs(m, ("root_pitch",), 2, 2)
    crank_spin_qpos, crank_spin_dof = _lookup_joint_addrs(m, ("crank_spin",), 3, 3)
    fork_travel_qpos, fork_travel_dof = _lookup_joint_addrs(m, ("fork_travel",), 6, 6)
    front_wheel_spin_qpos, front_wheel_spin_dof = _lookup_joint_addrs(
        m, ("front_wheel_spin",), 7, 7
    )
    rear_wheel_spin_qpos, rear_wheel_spin_dof = _lookup_joint_addrs(
        m, ("rear_wheel_spin",), 10, 10
    )
    shock_stroke_qpos, shock_stroke_dof = _lookup_joint_addrs(
        m, ("shock_stroke",), 13, 13
    )

    rider_joint_specs = (
        (("rider_root_pitch", "rider_hip", "hip"), 16, 16),
        (("rider_torso_hinge", "rider_knee", "knee"), 17, 17),
        (("rider_shoulder_left", "rider_ankle", "ankle"), 18, 18),
        (("rider_knee_front", "rider_waist_pitch", "waist_pitch"), 23, 23),
        (("rider_hip_rear", "rider_shoulder_pitch", "shoulder_pitch"), 24, 24),
        (("rider_knee_rear", "rider_elbow", "elbow"), 25, 25),
    )
    rider_qpos_list: list[int] = []
    rider_dof_list: list[int] = []
    for names, def_q, def_d in rider_joint_specs:
        q_adr, d_adr = _lookup_joint_addrs(m, names, def_q, def_d)
        rider_qpos_list.append(q_adr)
        rider_dof_list.append(d_adr)

    rider_act_specs = (
        (("act_rider_shoulder_right", "rider_hip_motor", "rider_hip"), 6),
        (("act_rider_elbow_right", "rider_knee_motor", "rider_knee"), 7),
        (("act_rider_hip_front", "rider_ankle_motor", "rider_ankle"), 8),
        (("act_rider_knee_front", "rider_waist_pitch_motor", "rider_waist_pitch"), 9),
        (("act_rider_hip_rear", "rider_shoulder_pitch_motor", "rider_shoulder_pitch"), 10),
        (("act_rider_knee_rear", "rider_elbow_motor", "rider_elbow"), 11),
    )
    rider_ctrl_list = [
        _lookup_actuator_id(m, names, def_ctrl) for names, def_ctrl in rider_act_specs
    ]

    idx_struct = GpuModelIndicesStruct(
        nq=nq,
        nv=nv,
        nu=nu,
        nbody=nbody,
        frame_bid=frame_bid,
        front_wheel_bid=front_wheel_bid,
        rear_wheel_bid=rear_wheel_bid,
        root_x_qpos=root_x_qpos,
        root_z_qpos=root_z_qpos,
        root_pitch_qpos=root_pitch_qpos,
        crank_spin_qpos=crank_spin_qpos,
        fork_travel_qpos=fork_travel_qpos,
        front_wheel_spin_qpos=front_wheel_spin_qpos,
        rear_wheel_spin_qpos=rear_wheel_spin_qpos,
        shock_stroke_qpos=shock_stroke_qpos,
        rider_qpos=(ctypes.c_int * 6)(*rider_qpos_list),
        root_x_dof=root_x_dof,
        root_z_dof=root_z_dof,
        root_pitch_dof=root_pitch_dof,
        crank_spin_dof=crank_spin_dof,
        fork_travel_dof=fork_travel_dof,
        front_wheel_spin_dof=front_wheel_spin_dof,
        rear_wheel_spin_dof=rear_wheel_spin_dof,
        shock_stroke_dof=shock_stroke_dof,
        rider_dof=(ctypes.c_int * 6)(*rider_dof_list),
        rider_ctrl=(ctypes.c_int * 6)(*rider_ctrl_list),
    )
    return bytes(idx_struct)


def build_default_config_bytes(lib: ctypes.CDLL) -> bytes:
    """Populate a single default ``GpuWorldConfig`` struct on the host and return its raw bytes."""
    cfg_size = int(lib.bike_gpu_sizeof_world_config())
    buf = ctypes.create_string_buffer(cfg_size)
    lib.bike_gpu_init_default_config(ctypes.cast(buf, ctypes.c_void_p))
    return buf.raw


def _as_float32_ptr(
    arr: np.ndarray | None,
    nworld: int,
) -> tuple[np.ndarray | None, ctypes.POINTER(ctypes.c_float) | None]:
    if arr is None:
        return None, None
    contig = np.ascontiguousarray(arr, dtype=np.float32)
    if contig.size < nworld:
        raise ValueError(f"Expected scale array of length >= {nworld}, got {contig.size}")
    return contig, contig.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def build_sweep_configs_bytes(
    lib: ctypes.CDLL,
    nworld: int,
    fork_psi_scale: np.ndarray | None = None,
    rebound_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
    tire_stiff_scale: np.ndarray | None = None,
    num_stations: int = 32,
    num_lateral_ribs: int = 1,
) -> bytes:
    """Populate ``nworld`` ``GpuWorldConfig`` structs on the host and return their raw bytes."""
    cfg_size = int(lib.bike_gpu_sizeof_world_config())
    buf = ctypes.create_string_buffer(nworld * cfg_size)

    _, fork_ptr = _as_float32_ptr(fork_psi_scale, nworld)
    _, reb_ptr = _as_float32_ptr(rebound_scale, nworld)
    _, shock_ptr = _as_float32_ptr(shock_rate_scale, nworld)
    _, tire_ptr = _as_float32_ptr(tire_stiff_scale, nworld)

    lib.bike_gpu_build_sweep_configs(
        ctypes.cast(buf, ctypes.c_void_p),
        int(nworld),
        fork_ptr,
        reb_ptr,
        shock_ptr,
        tire_ptr,
        int(num_stations),
        int(num_lateral_ribs),
    )
    return buf.raw


if __name__ == "__main__":
    so_lib = compile_cuda_writers()
    print(
        json.dumps(
            {
                "sizeof_world_config": int(so_lib.bike_gpu_sizeof_world_config()),
                "sizeof_world_state": int(so_lib.bike_gpu_sizeof_world_state()),
                "sizeof_model_indices": int(so_lib.bike_gpu_sizeof_model_indices()),
            },
            indent=2,
        )
        if "json" in globals()
        else {
            "sizeof_world_config": int(so_lib.bike_gpu_sizeof_world_config()),
            "sizeof_world_state": int(so_lib.bike_gpu_sizeof_world_state()),
            "sizeof_model_indices": int(so_lib.bike_gpu_sizeof_model_indices()),
        }
    )
