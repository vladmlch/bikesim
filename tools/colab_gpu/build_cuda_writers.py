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
        ("frame_body", ctypes.c_int),
        ("steer_body", ctypes.c_int),
        ("fork_lower_body", ctypes.c_int),
        ("front_wheel_body", ctypes.c_int),
        ("chainstay_body", ctypes.c_int),
        ("seatstay_body", ctypes.c_int),
        ("rear_wheel_body", ctypes.c_int),
        # Joint qpos addresses
        ("root_x_qpos", ctypes.c_int),
        ("root_z_qpos", ctypes.c_int),
        ("root_pitch_qpos", ctypes.c_int),
        ("crank_spin_qpos", ctypes.c_int),
        ("fork_travel_qpos", ctypes.c_int),
        ("front_wheel_spin_qpos", ctypes.c_int),
        ("rear_wheel_spin_qpos", ctypes.c_int),
        ("shock_stroke_qpos", ctypes.c_int),
        ("rider_qpos", ctypes.c_int * 9),
        # Joint dof addresses
        ("root_x_dof", ctypes.c_int),
        ("root_z_dof", ctypes.c_int),
        ("root_pitch_dof", ctypes.c_int),
        ("crank_spin_dof", ctypes.c_int),
        ("fork_travel_dof", ctypes.c_int),
        ("front_wheel_spin_dof", ctypes.c_int),
        ("rear_wheel_spin_dof", ctypes.c_int),
        ("shock_stroke_dof", ctypes.c_int),
        ("rider_dof", ctypes.c_int * 9),
        # Actuator ctrl addresses
        ("rider_ctrl", ctypes.c_int * 9),
        # Suspension pivot DOFs in model.xml (main_pivot=8, horst_pivot=9)
        ("main_pivot_dof", ctypes.c_int),
        ("horst_pivot_dof", ctypes.c_int),
        # Tendon index in model.xml (ideal_mid_drive_freehub=0)
        ("freehub_tendon", ctypes.c_int),
    ]

    @property
    def frame_bid(self) -> int:
        return int(self.frame_body)

    @frame_bid.setter
    def frame_bid(self, value: int) -> None:
        self.frame_body = int(value)

    @property
    def front_wheel_bid(self) -> int:
        return int(self.front_wheel_body)

    @front_wheel_bid.setter
    def front_wheel_bid(self, value: int) -> None:
        self.front_wheel_body = int(value)

    @property
    def rear_wheel_bid(self) -> int:
        return int(self.rear_wheel_body)

    @rear_wheel_bid.setter
    def rear_wheel_bid(self, value: int) -> None:
        self.rear_wheel_body = int(value)


def configure_cuda_writers_cdll(lib: ctypes.CDLL) -> ctypes.CDLL:
    """Configure ``.argtypes`` and ``.restype`` for all ``bike_gpu_*`` C API entry points."""
    lib.bike_gpu_sizeof_world_config.argtypes = []
    lib.bike_gpu_sizeof_world_config.restype = ctypes.c_int

    lib.bike_gpu_sizeof_world_state.argtypes = []
    lib.bike_gpu_sizeof_world_state.restype = ctypes.c_int

    lib.bike_gpu_sizeof_model_indices.argtypes = []
    lib.bike_gpu_sizeof_model_indices.restype = ctypes.c_int

    expected_idx_size = ctypes.sizeof(GpuModelIndicesStruct)
    actual_idx_size = int(lib.bike_gpu_sizeof_model_indices())
    if actual_idx_size != expected_idx_size:
        raise RuntimeError(
            f"GpuModelIndicesStruct size mismatch: ctypes has {expected_idx_size} bytes, "
            f"C++ library expects {actual_idx_size} bytes"
        )

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


def _lookup_joint_id_and_addrs(
    m: Any,
    candidate_names: Sequence[str],
    default_qpos: int,
    default_dof: int,
) -> tuple[int, int, int]:
    if m is None:
        return -1, default_qpos, default_dof
    try:
        import mujoco

        for name in candidate_names:
            jid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name))
            if jid >= 0:
                return jid, int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid])
    except Exception:
        pass
    return -1, default_qpos, default_dof


def _lookup_joint_addrs(
    m: Any,
    candidate_names: Sequence[str],
    default_qpos: int,
    default_dof: int,
) -> tuple[int, int]:
    _, qpos_adr, dof_adr = _lookup_joint_id_and_addrs(
        m, candidate_names, default_qpos, default_dof
    )
    return qpos_adr, dof_adr


def _lookup_actuator_id(
    m: Any,
    candidate_names: Sequence[str],
    default_id: int,
    target_jid: int = -1,
) -> int:
    if m is None:
        return default_id
    aid = -1
    try:
        import mujoco

        for name in candidate_names:
            found = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            if found >= 0:
                aid = found
                break
    except Exception:
        pass

    if target_jid >= 0:
        try:
            trnid = getattr(m, "actuator_trnid", None)
            if trnid is not None:
                n_act = int(trnid.shape[0])
                if 0 <= aid < n_act and int(trnid[aid, 0]) == target_jid:
                    return aid
                for candidate_aid in range(n_act):
                    if int(trnid[candidate_aid, 0]) == target_jid:
                        return candidate_aid
        except Exception:
            pass

    return aid if aid >= 0 else default_id


def _lookup_tendon_id(m: Any, candidate_names: Sequence[str], default_id: int) -> int:
    if m is None:
        return default_id
    try:
        import mujoco

        for name in candidate_names:
            tid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name))
            if tid >= 0:
                return tid
    except Exception:
        pass
    return default_id


def resolve_gpu_model_indices(m: Any) -> bytes:
    """Populate the ``GpuModelIndices`` C struct from a ``mujoco.MjModel`` and return its raw bytes."""
    nq = int(getattr(m, "nq", 26)) if m is not None else 26
    nv = int(getattr(m, "nv", 26)) if m is not None else 26
    nu = int(getattr(m, "nu", 12)) if m is not None else 12
    nbody = int(getattr(m, "nbody", 28)) if m is not None else 28

    frame_body = _lookup_body_id(m, ("frame", "main_frame", "chassis"), 1)
    steer_body = _lookup_body_id(m, ("steer", "fork_upper", "steering"), 5)
    fork_lower_body = _lookup_body_id(m, ("fork_lower", "front_fork_lower"), 6)
    front_wheel_body = _lookup_body_id(m, ("front_wheel", "wheel_front"), 7)
    chainstay_body = _lookup_body_id(m, ("chainstay", "rear_arm"), 8)
    seatstay_body = _lookup_body_id(m, ("seatstay", "rear_seatstay"), 9)
    rear_wheel_body = _lookup_body_id(m, ("rear_wheel", "wheel_rear"), 10)

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

    _, main_pivot_dof = _lookup_joint_addrs(m, ("main_pivot",), 8, 8)
    _, horst_pivot_dof = _lookup_joint_addrs(m, ("horst_pivot",), 9, 9)
    freehub_tendon = _lookup_tendon_id(m, ("ideal_mid_drive_freehub",), 0)

    joint_candidates = [
        ("rider_hip_front", "rider_hip", "hip_pitch", "rider_hip_pitch", "hip"),
        ("rider_knee_front", "rider_knee", "knee_pitch", "rider_knee_pitch", "knee"),
        ("rider_hip_rear", "rider_ankle_front", "rider_ankle", "ankle_pitch", "ankle"),
        ("rider_knee_rear",),
        ("rider_torso_hinge", "rider_waist_pitch", "waist_pitch", "torso_pitch", "waist"),
        ("rider_shoulder_left",),
        ("rider_elbow_left",),
        ("rider_shoulder_right", "rider_shoulder_pitch", "shoulder_pitch", "shoulder"),
        ("rider_elbow_right", "rider_elbow", "elbow_pitch", "rider_elbow_pitch", "elbow"),
    ]
    act_candidates = [
        ("act_rider_hip_front", "rider_hip_motor", "act_rider_hip", "hip_motor", "hip"),
        ("act_rider_knee_front", "rider_knee_motor", "act_rider_knee", "knee_motor", "knee"),
        ("act_rider_hip_rear", "rider_ankle_motor", "act_rider_ankle", "ankle_motor", "ankle"),
        ("act_rider_knee_rear",),
        ("act_rider_torso_hinge", "rider_waist_motor", "act_rider_waist_pitch", "waist_motor", "waist_pitch"),
        ("act_rider_shoulder_left",),
        ("act_rider_elbow_left",),
        ("act_rider_shoulder_right", "rider_shoulder_motor", "act_rider_shoulder_pitch", "shoulder_motor", "shoulder_pitch"),
        ("act_rider_elbow_right", "rider_elbow_motor", "act_rider_elbow", "elbow_motor", "elbow"),
    ]
    default_qpos = (22, 23, 24, 25, 17, 18, 19, 20, 21)
    default_dof = (22, 23, 24, 25, 17, 18, 19, 20, 21)
    default_ctrl = (8, 9, 10, 11, 3, 4, 5, 6, 7)

    rider_qpos_list: list[int] = []
    rider_dof_list: list[int] = []
    rider_ctrl_list: list[int] = []
    for j in range(9):
        jid, q_adr, d_adr = _lookup_joint_id_and_addrs(
            m, joint_candidates[j], default_qpos[j], default_dof[j]
        )
        rider_qpos_list.append(q_adr)
        rider_dof_list.append(d_adr)
        rider_ctrl_list.append(
            _lookup_actuator_id(m, act_candidates[j], default_ctrl[j], target_jid=jid)
        )

    idx_struct = GpuModelIndicesStruct(
        nq=nq,
        nv=nv,
        nu=nu,
        nbody=nbody,
        frame_body=frame_body,
        steer_body=steer_body,
        fork_lower_body=fork_lower_body,
        front_wheel_body=front_wheel_body,
        chainstay_body=chainstay_body,
        seatstay_body=seatstay_body,
        rear_wheel_body=rear_wheel_body,
        root_x_qpos=root_x_qpos,
        root_z_qpos=root_z_qpos,
        root_pitch_qpos=root_pitch_qpos,
        crank_spin_qpos=crank_spin_qpos,
        fork_travel_qpos=fork_travel_qpos,
        front_wheel_spin_qpos=front_wheel_spin_qpos,
        rear_wheel_spin_qpos=rear_wheel_spin_qpos,
        shock_stroke_qpos=shock_stroke_qpos,
        rider_qpos=(ctypes.c_int * 9)(*rider_qpos_list),
        root_x_dof=root_x_dof,
        root_z_dof=root_z_dof,
        root_pitch_dof=root_pitch_dof,
        crank_spin_dof=crank_spin_dof,
        fork_travel_dof=fork_travel_dof,
        front_wheel_spin_dof=front_wheel_spin_dof,
        rear_wheel_spin_dof=rear_wheel_spin_dof,
        shock_stroke_dof=shock_stroke_dof,
        rider_dof=(ctypes.c_int * 9)(*rider_dof_list),
        rider_ctrl=(ctypes.c_int * 9)(*rider_ctrl_list),
        main_pivot_dof=main_pivot_dof,
        horst_pivot_dof=horst_pivot_dof,
        freehub_tendon=freehub_tendon,
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
    info = {
        "sizeof_world_config": int(so_lib.bike_gpu_sizeof_world_config()),
        "sizeof_world_state": int(so_lib.bike_gpu_sizeof_world_state()),
        "sizeof_model_indices": int(so_lib.bike_gpu_sizeof_model_indices()),
    }
    print(json.dumps(info, indent=2))
