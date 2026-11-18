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
        ("drive_shaft_spin_qpos", ctypes.c_int),
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
        ("drive_shaft_spin_dof", ctypes.c_int),
        ("rider_dof", ctypes.c_int * 9),
        # Actuator ctrl addresses
        ("rider_ctrl", ctypes.c_int * 9),
        # Suspension pivot DOFs in welded.xml (main_pivot=8, horst_pivot=9)
        ("main_pivot_dof", ctypes.c_int),
        ("horst_pivot_dof", ctypes.c_int),
        # Tendon indices in welded.xml (ideal_mid_drive_freehub=0, crank_clutch=1)
        ("freehub_tendon", ctypes.c_int),
        ("crank_clutch_tendon", ctypes.c_int),
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


def _assert_struct_size(lib: ctypes.CDLL, fn_name: str, struct_cls: type[ctypes.Structure]) -> None:
    expected = ctypes.sizeof(struct_cls)
    actual = int(getattr(lib, fn_name)())
    if actual != expected:
        raise RuntimeError(
            f"{struct_cls.__name__} size mismatch: ctypes has {expected} bytes, "
            f"C++ library ({fn_name}) expects {actual} bytes"
        )


class GpuTerrainTableDescStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::physics_core::GpuTerrainTableDesc``."""

    _fields_ = [
        ("polyline_vertex_count", ctypes.c_int32),
        ("px_offset", ctypes.c_int32),
        ("pz_offset", ctypes.c_int32),
        ("seg_x_offset", ctypes.c_int32),
        ("seg_z_offset", ctypes.c_int32),
        ("seg_len_sq_offset", ctypes.c_int32),
        ("height_changes_offset", ctypes.c_int32),
        ("left_x_offset", ctypes.c_int32),
        ("left_z_offset", ctypes.c_int32),
        ("right_x_offset", ctypes.c_int32),
        ("right_z_offset", ctypes.c_int32),
        ("maximum_z", ctypes.c_float),
        ("significant_delta_m", ctypes.c_float),
        ("significance_fraction", ctypes.c_float),
        ("normal_cosine", ctypes.c_float),
        ("signed_vertex_count", ctypes.c_int32),
        ("sp_x_offset", ctypes.c_int32),
        ("sp_z_offset", ctypes.c_int32),
        ("sp_edge_x_offset", ctypes.c_int32),
        ("sp_edge_z_offset", ctypes.c_int32),
        ("sp_length2_offset", ctypes.c_int32),
        ("sp_normal_x_offset", ctypes.c_int32),
        ("sp_normal_z_offset", ctypes.c_int32),
        ("f32_count", ctypes.c_int32),
        ("i32_count", ctypes.c_int32),
    ]


class GpuChainJointStruct(ctypes.Structure):
    _fields_ = [
        ("joint_id", ctypes.c_int32),
        ("joint_type", ctypes.c_int32),
        ("dof_adr", ctypes.c_int32),
    ]


class GpuWheelChainStruct(ctypes.Structure):
    _fields_ = [
        ("wheel_body_id", ctypes.c_int32),
        ("njnt", ctypes.c_int32),
        ("joint_count", ctypes.c_int32),
        ("joints", GpuChainJointStruct * 8),
    ]


class CompliantMaterialViewStruct(ctypes.Structure):
    _fields_ = [
        ("radial_k_n_m", ctypes.c_float),
        ("loss_factor", ctypes.c_float),
        ("hysteresis_rate_eps_mps", ctypes.c_float),
        ("valid_load_min_n", ctypes.c_float),
        ("valid_load_max_n", ctypes.c_float),
        ("has_tabulated_radial", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 3),
    ]


class HingeDensityViewStruct(ctypes.Structure):
    _fields_ = [
        ("knots_m", ctypes.c_float * 8),
        ("stiffness_n_m", ctypes.c_float * 8),
        ("knot_count", ctypes.c_int32),
        ("damping_ns_m", ctypes.c_float),
        ("loss_factor", ctypes.c_float),
        ("hysteresis_rate_eps_mps", ctypes.c_float),
    ]


class TyreWheelSpecStruct(ctypes.Structure):
    _fields_ = [
        ("chain", GpuWheelChainStruct),
        ("geom_id", ctypes.c_int32),
        ("radius_m", ctypes.c_float),
        ("tangent_k_n_m", ctypes.c_float),
        ("tire_mu", ctypes.c_float),
        ("relaxation_length_m", ctypes.c_float),
        ("compliant", CompliantMaterialViewStruct),
        ("distributed_density", HingeDensityViewStruct),
        ("valid_deflection_min_m", ctypes.c_float),
        ("valid_deflection_max_m", ctypes.c_float),
        ("local_xyz", ctypes.c_float * (3 * 128)),
        ("station_weight", ctypes.c_float),
    ]


class SurfaceSpecPodStruct(ctypes.Structure):
    _fields_ = [
        ("mu_peak", ctypes.c_float),
        ("mu_slide", ctypes.c_float),
        ("slip_stiffness_per_load", ctypes.c_float),
        ("stribeck_speed_mps", ctypes.c_float),
        ("surface_id", ctypes.c_int32),
    ]


class SurfaceSectionViewStruct(ctypes.Structure):
    _fields_ = [
        ("start_m", ctypes.c_float),
        ("end_m", ctypes.c_float),
        ("surface", SurfaceSpecPodStruct),
    ]


class SurfaceMapViewStruct(ctypes.Structure):
    _fields_ = [
        ("default_surface", SurfaceSpecPodStruct),
        ("sections", SurfaceSectionViewStruct * 16),
        ("section_count", ctypes.c_int32),
        ("has_surface_map", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuTyreModelStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::physics_core::GpuTyreModel``."""

    _fields_ = [
        ("backend", ctypes.c_int32),
        ("nq", ctypes.c_int32),
        ("nv", ctypes.c_int32),
        ("nbody", ctypes.c_int32),
        ("njnt", ctypes.c_int32),
        ("station_count", ctypes.c_int32),
        ("distinct_normal_cosine", ctypes.c_float),
        ("wheels", TyreWheelSpecStruct * 2),
        ("frame_chain", GpuWheelChainStruct),
        ("surface_map", SurfaceMapViewStruct),
    ]


def configure_cuda_writers_cdll(lib: ctypes.CDLL) -> ctypes.CDLL:
    """Configure ``.argtypes`` and ``.restype`` for all ``bike_gpu_*`` C API entry points."""
    lib.bike_gpu_sizeof_world_config.argtypes = []
    lib.bike_gpu_sizeof_world_config.restype = ctypes.c_int

    lib.bike_gpu_sizeof_world_state.argtypes = []
    lib.bike_gpu_sizeof_world_state.restype = ctypes.c_int

    lib.bike_gpu_sizeof_model_indices.argtypes = []
    lib.bike_gpu_sizeof_model_indices.restype = ctypes.c_int

    lib.bike_gpu_sizeof_tyre_model.argtypes = []
    lib.bike_gpu_sizeof_tyre_model.restype = ctypes.c_int

    lib.bike_gpu_sizeof_terrain_desc.argtypes = []
    lib.bike_gpu_sizeof_terrain_desc.restype = ctypes.c_int

    _assert_struct_size(lib, "bike_gpu_sizeof_model_indices", GpuModelIndicesStruct)
    _assert_struct_size(lib, "bike_gpu_sizeof_tyre_model", GpuTyreModelStruct)
    _assert_struct_size(lib, "bike_gpu_sizeof_terrain_desc", GpuTerrainTableDescStruct)

    lib.bike_gpu_init_default_config.argtypes = [ctypes.c_void_p]
    lib.bike_gpu_init_default_config.restype = ctypes.c_int

    lib.bike_gpu_build_sweep_configs.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.bike_gpu_build_sweep_configs.restype = ctypes.c_int

    lib.bike_gpu_launch_init_states.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    lib.bike_gpu_launch_init_states.restype = ctypes.c_int

    writers_argtypes = [
        ctypes.c_void_p,  # cuda_stream
        ctypes.c_int,     # nworld
        ctypes.c_float,   # dt_s
        ctypes.c_void_p,  # d_idx
        ctypes.c_void_p,  # d_tyre_model
        ctypes.c_void_p,  # d_terrain_desc
        ctypes.c_void_p,  # d_terrain_f32
        ctypes.c_void_p,  # d_terrain_i32
        ctypes.c_void_p,  # d_configs
        ctypes.c_int,     # per_world_config
        ctypes.c_void_p,  # d_states
        ctypes.c_void_p,  # d_qpos
        ctypes.c_void_p,  # d_qvel
        ctypes.c_void_p,  # d_xpos
        ctypes.c_void_p,  # d_xmat
        ctypes.c_void_p,  # d_xanchor
        ctypes.c_void_p,  # d_xaxis
        ctypes.c_void_p,  # d_qfrc_applied
        ctypes.c_void_p,  # d_ctrl
        ctypes.c_void_p,  # d_tendon_range
        ctypes.c_int,     # tendon_range_world_stride
    ]
    lib.bike_gpu_launch_writers.argtypes = writers_argtypes
    lib.bike_gpu_launch_writers.restype = ctypes.c_int
    if hasattr(lib, "bike_gpu_launch_step_writers"):
        lib.bike_gpu_launch_step_writers.argtypes = writers_argtypes
        lib.bike_gpu_launch_step_writers.restype = ctypes.c_int

    return lib


def compile_cuda_writers(
    gpu_src_dir: Path = DEFAULT_GPU_SRC_DIR,
    out_so_path: Path = DEFAULT_OUT_SO_PATH,
) -> ctypes.CDLL:
    """Compile ``cuda_kernels.cu`` with ``nvcc`` and load the resulting shared library via ``ctypes.CDLL``."""
    gpu_src_dir = Path(gpu_src_dir).resolve()
    native_src_dir = gpu_src_dir.parent
    out_so_path = Path(out_so_path).resolve()
    cu_path = gpu_src_dir / "cuda_kernels.cu"
    if not cu_path.is_file():
        raise FileNotFoundError(f"CUDA kernel source not found: {cu_path}")

    out_so_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "nvcc",
        "-std=c++20",
        "-O3",
        "--fmad=false",
        "--shared",
        "-Xcompiler",
        "-fPIC",
        f"-I{native_src_dir}",
        f"-I{gpu_src_dir}",
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
    nq = int(getattr(m, "nq", 27)) if m is not None else 27
    nv = int(getattr(m, "nv", 27)) if m is not None else 27
    nu = int(getattr(m, "nu", 12)) if m is not None else 12
    nbody = int(getattr(m, "nbody", 29)) if m is not None else 29

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
    drive_shaft_spin_qpos, drive_shaft_spin_dof = _lookup_joint_addrs(
        m, ("drive_shaft_spin",), -1 if m is not None else 14, -1 if m is not None else 14
    )

    _, main_pivot_dof = _lookup_joint_addrs(m, ("main_pivot",), 8, 8)
    _, horst_pivot_dof = _lookup_joint_addrs(m, ("horst_pivot",), 9, 9)
    freehub_tendon = _lookup_tendon_id(m, ("ideal_mid_drive_freehub",), 0 if m is None else -1)
    crank_clutch_tendon = _lookup_tendon_id(m, ("crank_clutch",), 1 if m is None else -1)

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
    default_qpos = (23, 24, 25, 26, 18, 19, 20, 21, 22)
    default_dof = (23, 24, 25, 26, 18, 19, 20, 21, 22)
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
        drive_shaft_spin_qpos=drive_shaft_spin_qpos,
        rider_qpos=(ctypes.c_int * 9)(*rider_qpos_list),
        root_x_dof=root_x_dof,
        root_z_dof=root_z_dof,
        root_pitch_dof=root_pitch_dof,
        crank_spin_dof=crank_spin_dof,
        fork_travel_dof=fork_travel_dof,
        front_wheel_spin_dof=front_wheel_spin_dof,
        rear_wheel_spin_dof=rear_wheel_spin_dof,
        shock_stroke_dof=shock_stroke_dof,
        drive_shaft_spin_dof=drive_shaft_spin_dof,
        rider_dof=(ctypes.c_int * 9)(*rider_dof_list),
        rider_ctrl=(ctypes.c_int * 9)(*rider_ctrl_list),
        main_pivot_dof=main_pivot_dof,
        horst_pivot_dof=horst_pivot_dof,
        freehub_tendon=freehub_tendon,
        crank_clutch_tendon=crank_clutch_tendon,
    )
    return bytes(idx_struct)


def build_default_config_bytes(lib: ctypes.CDLL) -> bytes:
    """Populate a single default ``GpuWorldConfig`` struct on the host and return its raw bytes."""
    cfg_size = int(lib.bike_gpu_sizeof_world_config())
    buf = ctypes.create_string_buffer(cfg_size)
    rc = int(lib.bike_gpu_init_default_config(ctypes.cast(buf, ctypes.c_void_p)))
    if rc != 0:
        raise RuntimeError(f"bike_gpu_init_default_config failed with return code {rc}")
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
) -> bytes:
    """Populate ``nworld`` ``GpuWorldConfig`` structs on the host and return their raw bytes."""
    cfg_size = int(lib.bike_gpu_sizeof_world_config())
    buf = ctypes.create_string_buffer(nworld * cfg_size)

    _, fork_ptr = _as_float32_ptr(fork_psi_scale, nworld)
    _, reb_ptr = _as_float32_ptr(rebound_scale, nworld)
    _, shock_ptr = _as_float32_ptr(shock_rate_scale, nworld)
    _, tire_ptr = _as_float32_ptr(tire_stiff_scale, nworld)

    rc = int(
        lib.bike_gpu_build_sweep_configs(
            ctypes.cast(buf, ctypes.c_void_p),
            int(nworld),
            fork_ptr,
            reb_ptr,
            shock_ptr,
            tire_ptr,
        )
    )
    if rc != 0:
        raise RuntimeError(f"bike_gpu_build_sweep_configs failed with return code {rc}")
    return buf.raw


def configure_host_tyre_builder(lib: ctypes.CDLL, symbol_name: str) -> Any:
    """Bind ``bike_{cpu,metal}_build_tyre_and_terrain`` argtypes and restype."""
    fn = getattr(lib, symbol_name)
    fn.argtypes = [
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int),
    ]
    fn.restype = ctypes.c_int
    return fn


def build_tyre_and_terrain_buffers(
    host_lib: ctypes.CDLL,
    xml_path: Path | str,
    backend: str = "compliant_2d",
    station_count: int = 64,
    idx_bytes: bytes | None = None,
) -> tuple[bytes, bytes, np.ndarray, np.ndarray]:
    """Resolve ``(tyre_model_bytes, terrain_desc_bytes, terrain_f32, terrain_i32)`` via host C++."""
    if hasattr(host_lib, "bike_cpu_build_tyre_and_terrain"):
        builder = configure_host_tyre_builder(host_lib, "bike_cpu_build_tyre_and_terrain")
    elif hasattr(host_lib, "bike_metal_build_tyre_and_terrain"):
        builder = configure_host_tyre_builder(host_lib, "bike_metal_build_tyre_and_terrain")
    else:
        raise AttributeError(
            "host_lib exposes neither bike_cpu_build_tyre_and_terrain nor bike_metal_build_tyre_and_terrain"
        )

    if backend == "compliant_2d":
        backend_kind = 0
    elif backend == "distributed_2d_reference":
        backend_kind = 1
    else:
        raise ValueError(f"Unknown tyre backend '{backend}'")

    model_buf = ctypes.create_string_buffer(int(host_lib.bike_gpu_sizeof_tyre_model()))
    desc_buf = ctypes.create_string_buffer(int(host_lib.bike_gpu_sizeof_terrain_desc()))
    idx_buf = ctypes.create_string_buffer(idx_bytes) if idx_bytes is not None else None
    idx_ptr = ctypes.cast(idx_buf, ctypes.c_void_p) if idx_buf is not None else None

    f32_cap = ctypes.c_int(0)
    i32_cap = ctypes.c_int(0)
    xml_b = str(Path(xml_path).resolve()).encode("utf-8")
    rc = int(
        builder(
            xml_b,
            int(backend_kind),
            int(station_count),
            idx_ptr,
            ctypes.cast(model_buf, ctypes.c_void_p),
            ctypes.cast(desc_buf, ctypes.c_void_p),
            None,
            ctypes.byref(f32_cap),
            None,
            ctypes.byref(i32_cap),
        )
    )
    if rc != 0:
        raise RuntimeError(f"Host tyre/terrain size query failed with return code {rc}")

    f32_arr = np.zeros(max(1, int(f32_cap.value)), dtype=np.float32)
    i32_arr = np.zeros(max(1, int(i32_cap.value)), dtype=np.int32)
    f32_cap.value = int(f32_arr.size)
    i32_cap.value = int(i32_arr.size)

    rc = int(
        builder(
            xml_b,
            int(backend_kind),
            int(station_count),
            idx_ptr,
            ctypes.cast(model_buf, ctypes.c_void_p),
            ctypes.cast(desc_buf, ctypes.c_void_p),
            f32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            ctypes.byref(f32_cap),
            i32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
            ctypes.byref(i32_cap),
        )
    )
    if rc != 0:
        raise RuntimeError(f"Host tyre/terrain build failed with return code {rc}")

    return (
        model_buf.raw,
        desc_buf.raw,
        np.ascontiguousarray(f32_arr[: int(f32_cap.value)], dtype=np.float32),
        np.ascontiguousarray(i32_arr[: int(i32_cap.value)], dtype=np.int32),
    )


if __name__ == "__main__":
    so_lib = compile_cuda_writers()
    info = {
        "sizeof_world_config": int(so_lib.bike_gpu_sizeof_world_config()),
        "sizeof_world_state": int(so_lib.bike_gpu_sizeof_world_state()),
        "sizeof_model_indices": int(so_lib.bike_gpu_sizeof_model_indices()),
        "sizeof_tyre_model": int(so_lib.bike_gpu_sizeof_tyre_model()),
        "sizeof_terrain_desc": int(so_lib.bike_gpu_sizeof_terrain_desc()),
    }
    print(json.dumps(info, indent=2))
