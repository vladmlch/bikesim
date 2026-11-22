"""Scenario 2: Full Apple Metal GPU Pipeline (`warp-metal` + `mujoco_warp` + Custom Metal Writers).

Executes the full GPU simulation loop on Apple Silicon (`device="metal:0"`) via SP1 `clean_split`:
  - Custom bicycle force writers (`bike_wp_writers_metal_kernel`) compiled via
    `wp_metal_load_library` and dispatched via `wp_metal_launch_kernel` with `MTLMathModeSafe`
    and `#pragma METAL fp contract(off)`.
  - Rigid-body constraint dynamics, kinematics, and collision (`clean_split` schedule:
    `fwd_kinematics -> crb -> tendon_armature -> factor_m -> collision -> transmission ->
    fwd_velocity -> writers -> make_constraint -> fwd_actuation -> fwd_acceleration ->
    solve -> sensor_acc -> implicit`) compiled and dispatched on Apple Metal GPU via `warp-metal`.
  - All stages fused inside a single `wp.ScopedCapture()` Metal Graph (`wp.capture_launch(graph)`).

Benchmarks:
  1. Experiment A (`run_mjwarp_metal_parameter_sweep`):
     Heterogeneous `nworld = 2048` parameter sweep across `fork_psi_scale`, `rebound_scale`,
     `shock_rate_scale`, and `tire_stiff_scale`.
  2. Experiment B (`run_mjwarp_metal_tyre_backend_scaling_benchmark`):
     Shared `tyre_gpu_step.hpp` backend scaling across `compliant_2d` and
     `distributed_2d_reference` (`N=32/64/128`) at `nworld = 2048` and `nworld = 1`.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
from pathlib import Path
import platform
import sys
import tempfile
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WARP_CACHE_DIR = REPO_ROOT / "build" / "warp_cache"

try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None  # type: ignore[assignment]

try:
    import warp_metal  # noqa: F401
    try:
        import warp_metal._bootstrap  # noqa: F401
    except Exception:
        pass
    import warp as wp
    import mujoco_warp as mjwarp
except Exception:  # pragma: no cover
    wp = None  # type: ignore[assignment]
    mjwarp = None  # type: ignore[assignment]

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        GpuModelIndicesStruct,
        build_sp3_sweep_configs_bytes,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        expand_mjwarp_tendon_range_and_frictionloss,
        materialize_model_12gears_xml,
        resolve_gpu_model_indices,
        unpack_sp4_world_metrics,
    )
    from tools.colab_gpu.metal_hybrid_sweep import compile_metal_batch_runner
    from tools.colab_gpu.model_assets import (
        load_model_with_assets,
        materialize_welded_assets_dir,
    )
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_ENVELOPE_JSON,
        DEFAULT_MODEL_XML,
        DEFAULT_STRENGTH_JSON,
        TYRE_BACKEND_TIERS,
        _allocate_and_copy_device_bytes,
        _build_sp4_config_bytes,
        _build_sp4_s1s4_scenarios,
        _build_sp4_xz_rebase_ctx,
        _compute_steady90_v0,
        _disable_contacts_for_sp4,
        _ensure_newton_solver,
        _export_sp4_scenario_bundle,
        export_sp4_bundles,
        _mjwarp_clean_split_step,
        _mjwarp_sp3_clean_split_step,
        _mjwarp_sp4_clean_split_step,
        _prepare_initial_mjdata,
        _resolve_clean_split_ops,
        _resolve_dof_frictionloss_binding,
        _resolve_sp3_post_solve_bindings,
        _resolve_tendon_range_binding,
        _summarize_distribution,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        GpuModelIndicesStruct,
        build_sp3_sweep_configs_bytes,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        expand_mjwarp_tendon_range_and_frictionloss,
        materialize_model_12gears_xml,
        resolve_gpu_model_indices,
        unpack_sp4_world_metrics,
    )
    from metal_hybrid_sweep import compile_metal_batch_runner  # type: ignore[no-redef]
    from model_assets import (  # type: ignore[no-redef]
        load_model_with_assets,
        materialize_welded_assets_dir,
    )
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_ENVELOPE_JSON,
        DEFAULT_MODEL_XML,
        DEFAULT_STRENGTH_JSON,
        TYRE_BACKEND_TIERS,
        _allocate_and_copy_device_bytes,
        _build_sp4_config_bytes,
        _build_sp4_s1s4_scenarios,
        _build_sp4_xz_rebase_ctx,
        _compute_steady90_v0,
        _disable_contacts_for_sp4,
        _ensure_newton_solver,
        _export_sp4_scenario_bundle,
        export_sp4_bundles,
        _mjwarp_clean_split_step,
        _mjwarp_sp3_clean_split_step,
        _mjwarp_sp4_clean_split_step,
        _prepare_initial_mjdata,
        _resolve_clean_split_ops,
        _resolve_dof_frictionloss_binding,
        _resolve_sp3_post_solve_bindings,
        _resolve_tendon_range_binding,
        _summarize_distribution,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )


class _BikeWpBounds(ctypes.Structure):
    _fields_ = [
        ("shape0", ctypes.c_int32),
        ("_pad0", ctypes.c_int32),
        ("size", ctypes.c_size_t),
        ("coord_mult", ctypes.c_size_t),
    ]


class _BikeWpInitArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("per_world_config", ctypes.c_int32),
    ]


class _BikeWpWritersArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("tyre_model", ctypes.c_uint64),
        ("terrain_desc", ctypes.c_uint64),
        ("terrain_f32", ctypes.c_uint64),
        ("terrain_i32", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("xanchor", ctypes.c_uint64),
        ("xaxis", ctypes.c_uint64),
        ("qfrc_applied", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
    ]


_INIT_PTR_OFFSETS = (ctypes.c_size_t * 4)(0, 8, 16, 24)
_WRITERS_PTR_OFFSETS_15 = (ctypes.c_size_t * 15)(*range(0, 120, 8))
_WRITERS_PTR_OFFSETS_16 = (ctypes.c_size_t * 16)(*range(0, 128, 8))


class _BikeWpSp3InitArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("world_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("per_world_config", ctypes.c_int32),
        ("init_grade", ctypes.c_float),
        ("ntendon", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
        ("has_world_states", ctypes.c_int32),
        ("has_qpos", ctypes.c_int32),
        ("has_tendon_range", ctypes.c_int32),
    ]


class _BikeWpSp3PreSolveArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("tyre_model", ctypes.c_uint64),
        ("terrain_desc", ctypes.c_uint64),
        ("terrain_f32", ctypes.c_uint64),
        ("terrain_i32", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("world_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("xanchor", ctypes.c_uint64),
        ("xaxis", ctypes.c_uint64),
        ("qfrc_applied", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("dof_frictionloss", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("ntendon", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
        ("dof_frictionloss_world_stride", ctypes.c_int32),
        ("has_world_states", ctypes.c_int32),
        ("has_tendon_range", ctypes.c_int32),
        ("has_dof_frictionloss", ctypes.c_int32),
    ]


class _BikeWpSp3PostSolveArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("sensordata", ctypes.c_uint64),
        ("nefc", ctypes.c_uint64),
        ("efc_type", ctypes.c_uint64),
        ("efc_id", ctypes.c_uint64),
        ("efc_J", ctypes.c_uint64),
        ("efc_force", ctypes.c_uint64),
        ("nacon", ctypes.c_uint64),
        ("contact_worldid", ctypes.c_uint64),
        ("contact_geom", ctypes.c_uint64),
        ("contact_efc_address", ctypes.c_uint64),
        ("contact_dim", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("nsensordata", ctypes.c_int32),
        ("njmax", ctypes.c_int32),
        ("efc_J_row_stride", ctypes.c_int32),
        ("nconmax", ctypes.c_int32),
        ("contact_efc_stride", ctypes.c_int32),
        ("has_sensordata", ctypes.c_int32),
        ("has_efc", ctypes.c_int32),
        ("has_contacts", ctypes.c_int32),
        ("has_contact_worldid", ctypes.c_int32),
        ("has_contact_dim", ctypes.c_int32),
    ]


_SP3_INIT_PTR_OFFSETS = (ctypes.c_size_t * 6)(*range(0, 48, 8))
_SP3_PRESOLVE_PTR_OFFSETS = (ctypes.c_size_t * 18)(*range(0, 144, 8))
_SP3_POSTSOLVE_PTR_OFFSETS = (ctypes.c_size_t * 17)(*range(0, 136, 8))


class _BikeWpSp4InitArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("sp4_states", ctypes.c_uint64),
        ("world_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("per_world_config", ctypes.c_int32),
        ("init_grade", ctypes.c_float),
        ("ntendon", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
        ("has_world_states", ctypes.c_int32),
        ("has_qpos", ctypes.c_int32),
        ("has_tendon_range", ctypes.c_int32),
    ]


class _BikeWpSp4PreSolveArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("tyre_model", ctypes.c_uint64),
        ("terrain_desc", ctypes.c_uint64),
        ("terrain_f32", ctypes.c_uint64),
        ("terrain_i32", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("spindle_params", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("sp4_states", ctypes.c_uint64),
        ("world_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("qfrc_bias", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("xanchor", ctypes.c_uint64),
        ("xaxis", ctypes.c_uint64),
        ("site_xpos", ctypes.c_uint64),
        ("subtree_com", ctypes.c_uint64),
        ("body_subtreemass", ctypes.c_uint64),
        ("M_rowadr", ctypes.c_uint64),
        ("M_rownnz", ctypes.c_uint64),
        ("M_colind", ctypes.c_uint64),
        ("d_M", ctypes.c_uint64),
        ("qfrc_applied", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("dof_frictionloss", ctypes.c_uint64),
        ("qacc_warmstart", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("qM_row_stride", ctypes.c_int32),
        ("M_world_stride", ctypes.c_int32),
        ("ntendon", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
        ("dof_frictionloss_world_stride", ctypes.c_int32),
        ("has_body_subtreemass", ctypes.c_int32),
        ("has_tendon_range", ctypes.c_int32),
        ("has_dof_frictionloss", ctypes.c_int32),
        ("has_qacc_warmstart", ctypes.c_int32),
    ]


class _BikeWpSp4PassArgs(ctypes.Structure):
    _fields_ = [
        ("spindle_params", ctypes.c_uint64),
        ("sp4_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("qfrc_bias", ctypes.c_uint64),
        ("qacc", ctypes.c_uint64),
        ("M_rowadr", ctypes.c_uint64),
        ("M_rownnz", ctypes.c_uint64),
        ("M_colind", ctypes.c_uint64),
        ("d_M", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("qacc_warmstart", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("nq", ctypes.c_int32),
        ("nv", ctypes.c_int32),
        ("nu", ctypes.c_int32),
        ("qM_row_stride", ctypes.c_int32),
        ("M_world_stride", ctypes.c_int32),
        ("has_qacc_warmstart", ctypes.c_int32),
    ]


class _BikeWpSp4PostSolveArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("spindle_params", ctypes.c_uint64),
        ("sp3_states", ctypes.c_uint64),
        ("sp4_states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("xanchor", ctypes.c_uint64),
        ("sensordata", ctypes.c_uint64),
        ("nefc", ctypes.c_uint64),
        ("efc_type", ctypes.c_uint64),
        ("efc_id", ctypes.c_uint64),
        ("efc_J", ctypes.c_uint64),
        ("efc_force", ctypes.c_uint64),
        ("nacon", ctypes.c_uint64),
        ("contact_worldid", ctypes.c_uint64),
        ("contact_geom", ctypes.c_uint64),
        ("contact_efc_address", ctypes.c_uint64),
        ("contact_dim", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("nsensordata", ctypes.c_int32),
        ("njmax", ctypes.c_int32),
        ("efc_J_row_stride", ctypes.c_int32),
        ("nconmax", ctypes.c_int32),
        ("contact_efc_stride", ctypes.c_int32),
        ("has_xpos", ctypes.c_int32),
        ("has_xanchor", ctypes.c_int32),
        ("has_sensordata", ctypes.c_int32),
        ("has_efc", ctypes.c_int32),
        ("has_contacts", ctypes.c_int32),
        ("has_contact_worldid", ctypes.c_int32),
        ("has_contact_dim", ctypes.c_int32),
    ]


_SP4_INIT_PTR_OFFSETS = (ctypes.c_size_t * 7)(*range(0, 56, 8))
_SP4_PRESOLVE_PTR_OFFSETS = (ctypes.c_size_t * 29)(*range(0, 232, 8))
_SP4_PASS_PTR_OFFSETS = (ctypes.c_size_t * 12)(*range(0, 96, 8))
_SP4_POSTSOLVE_PTR_OFFSETS = (ctypes.c_size_t * 21)(*range(0, 168, 8))


class WarpMetalWritersRuntime:
    """Loads custom bicycle Metal force writer kernels into `warp-metal`'s native runtime."""

    def __init__(self, lib: ctypes.CDLL, ordinal: int = 0) -> None:
        self.lib = lib
        self.ordinal = int(ordinal)
        self.core = wp._src.context.runtime.core

        msl_bytes = self.lib.bike_metal_get_shader_source(
            str(DEFAULT_GPU_SRC_DIR).encode("utf-8")
        )
        if not msl_bytes:
            raise RuntimeError("bike_metal_get_shader_source returned empty source")

        self.mtl_lib = self.core.wp_metal_load_library(self.ordinal, msl_bytes)
        if not self.mtl_lib:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_load_library failed:\n{err}")

        self.k_init = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_init_states_kernel")
        self.k_writers = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_writers_metal_kernel")
        self.k_sp3_init = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_sp3_init_kernel")
        self.k_sp3_pre_solve = self.core.wp_metal_get_kernel(
            self.mtl_lib, b"bike_wp_sp3_pre_solve_kernel"
        )
        self.k_sp3_post_solve = self.core.wp_metal_get_kernel(
            self.mtl_lib, b"bike_wp_sp3_post_solve_kernel"
        )
        self.k_sp4_init = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_sp4_init_kernel")
        self.k_sp4_pre_solve = self.core.wp_metal_get_kernel(
            self.mtl_lib, b"bike_wp_sp4_pre_solve_and_pass0_kernel"
        )
        self.k_sp4_pass1 = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_sp4_pass1_kernel")
        self.k_sp4_pass2 = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_sp4_pass2_kernel")
        self.k_sp4_pass3 = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_sp4_pass3_kernel")
        self.k_sp4_post_solve = self.core.wp_metal_get_kernel(
            self.mtl_lib, b"bike_wp_sp4_post_solve_kernel"
        )
        if (
            not self.k_init
            or not self.k_writers
            or not self.k_sp3_init
            or not self.k_sp3_pre_solve
            or not self.k_sp3_post_solve
            or not self.k_sp4_init
            or not self.k_sp4_pre_solve
            or not self.k_sp4_pass1
            or not self.k_sp4_pass2
            or not self.k_sp4_pass3
            or not self.k_sp4_post_solve
        ):
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_get_kernel failed:\n{err}")

        dev_name = f"metal:{self.ordinal}"
        self._dummy_f32 = wp.zeros(4, dtype=wp.float32, device=dev_name)
        self._dummy_i32 = wp.zeros(4, dtype=wp.int32, device=dev_name)
        ws_sz = max(64, int(self.lib.bike_gpu_sizeof_world_state()))
        self._dummy_ws = wp.zeros(ws_sz, dtype=wp.uint8, device=dev_name)

    def launch_init_states(
        self,
        nworld: int,
        d_idx_ptr: int,
        d_configs_ptr: int,
        per_world_config: int,
        d_states_ptr: int,
        d_qpos_ptr: int,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        for name, ptr in (
            ("d_idx_ptr", d_idx_ptr),
            ("d_configs_ptr", d_configs_ptr),
            ("d_states_ptr", d_states_ptr),
            ("d_qpos_ptr", d_qpos_ptr),
        ):
            if int(ptr) == 0:
                raise ValueError(f"{name} must be non-zero")
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpInitArgs(
            idx=int(d_idx_ptr),
            configs=int(d_configs_ptr),
            states=int(d_states_ptr),
            qpos=int(d_qpos_ptr),
            nworld=int(nworld),
            per_world_config=int(per_world_config),
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_init,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _INIT_PTR_OFFSETS,
            4,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_init_states_kernel) failed: {err}")

    def launch_writers(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_tyre_model_ptr: int,
        d_terrain_desc_ptr: int,
        d_terrain_f32_ptr: int,
        d_terrain_i32_ptr: int,
        d_configs_ptr: int,
        per_world_config: int,
        d_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xpos_ptr: int,
        d_xmat_ptr: int,
        d_xanchor_ptr: int,
        d_xaxis_ptr: int,
        d_qfrc_ptr: int,
        d_ctrl_ptr: int,
        tendon_range_ptr: int,
        tendon_range_stride: int,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        if not math.isfinite(float(dt_s)) or float(dt_s) <= 0.0:
            raise ValueError(f"dt_s must be finite and > 0, got {dt_s}")
        if int(tendon_range_stride) < 0:
            raise ValueError(f"tendon_range_stride must be >= 0, got {tendon_range_stride}")
        for name, ptr in (
            ("d_idx_ptr", d_idx_ptr),
            ("d_tyre_model_ptr", d_tyre_model_ptr),
            ("d_terrain_desc_ptr", d_terrain_desc_ptr),
            ("d_terrain_f32_ptr", d_terrain_f32_ptr),
            ("d_terrain_i32_ptr", d_terrain_i32_ptr),
            ("d_configs_ptr", d_configs_ptr),
            ("d_states_ptr", d_states_ptr),
            ("d_qpos_ptr", d_qpos_ptr),
            ("d_qvel_ptr", d_qvel_ptr),
            ("d_xpos_ptr", d_xpos_ptr),
            ("d_xmat_ptr", d_xmat_ptr),
            ("d_xanchor_ptr", d_xanchor_ptr),
            ("d_xaxis_ptr", d_xaxis_ptr),
            ("d_qfrc_ptr", d_qfrc_ptr),
            ("d_ctrl_ptr", d_ctrl_ptr),
        ):
            if int(ptr) == 0:
                raise ValueError(f"{name} must be non-zero")
        num_threads = int(nworld)
        bounds = _BikeWpBounds(shape0=num_threads, _pad0=0, size=num_threads, coord_mult=1)
        args = _BikeWpWritersArgs(
            idx=int(d_idx_ptr),
            tyre_model=int(d_tyre_model_ptr),
            terrain_desc=int(d_terrain_desc_ptr),
            terrain_f32=int(d_terrain_f32_ptr),
            terrain_i32=int(d_terrain_i32_ptr),
            configs=int(d_configs_ptr),
            states=int(d_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xpos=int(d_xpos_ptr),
            xmat=int(d_xmat_ptr),
            xanchor=int(d_xanchor_ptr),
            xaxis=int(d_xaxis_ptr),
            qfrc_applied=int(d_qfrc_ptr),
            ctrl=int(d_ctrl_ptr),
            tendon_range=int(tendon_range_ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            tendon_range_world_stride=int(tendon_range_stride),
        )
        if tendon_range_ptr != 0:
            offsets = _WRITERS_PTR_OFFSETS_16
            n_offsets = 16
        else:
            offsets = _WRITERS_PTR_OFFSETS_15
            n_offsets = 15

        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_writers,
            num_threads,
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            offsets,
            n_offsets,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_writers_metal_kernel) failed: {err}")

    def launch_sp3_init(
        self,
        nworld: int,
        d_idx_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_sp3_states_ptr: int,
        d_world_states_ptr: int = 0,
        d_qpos_ptr: int = 0,
        init_grade: float = 0.0,
        ntendon: int = 0,
        d_tendon_range_ptr: int = 0,
        tendon_range_world_stride: int = 0,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        for name, ptr in (
            ("d_idx_ptr", d_idx_ptr),
            ("d_sp3_configs_ptr", d_sp3_configs_ptr),
            ("d_sp3_states_ptr", d_sp3_states_ptr),
        ):
            if int(ptr) == 0:
                raise ValueError(f"{name} must be non-zero")
        has_ws = 1 if int(d_world_states_ptr) != 0 else 0
        has_qpos = 1 if int(d_qpos_ptr) != 0 else 0
        has_tr = (
            1
            if (
                int(d_tendon_range_ptr) != 0
                and int(ntendon) > 0
                and (int(tendon_range_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp3InitArgs(
            idx=int(d_idx_ptr),
            configs=int(d_sp3_configs_ptr),
            sp3_states=int(d_sp3_states_ptr),
            world_states=int(d_world_states_ptr) if has_ws else int(self._dummy_ws.ptr),
            qpos=int(d_qpos_ptr) if has_qpos else int(self._dummy_f32.ptr),
            tendon_range=int(d_tendon_range_ptr) if has_tr else int(self._dummy_f32.ptr),
            nworld=int(nworld),
            per_world_config=int(per_world_config),
            init_grade=float(init_grade),
            ntendon=int(ntendon),
            tendon_range_world_stride=int(tendon_range_world_stride),
            has_world_states=has_ws,
            has_qpos=has_qpos,
            has_tendon_range=has_tr,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp3_init,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP3_INIT_PTR_OFFSETS,
            6,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_sp3_init_kernel) failed: {err}")

    def launch_sp3_pre_solve(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_tyre_model_ptr: int,
        d_terrain_desc_ptr: int,
        d_terrain_f32_ptr: int,
        d_terrain_i32_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_sp3_states_ptr: int,
        d_world_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xpos_ptr: int,
        d_xmat_ptr: int,
        d_xanchor_ptr: int,
        d_xaxis_ptr: int,
        d_qfrc_ptr: int,
        d_ctrl_ptr: int,
        ntendon: int = 0,
        d_tendon_range_ptr: int = 0,
        tendon_range_world_stride: int = 0,
        d_dof_frictionloss_ptr: int = 0,
        dof_frictionloss_world_stride: int = 0,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        has_tr = (
            1
            if (
                int(d_tendon_range_ptr) != 0
                and int(ntendon) > 0
                and (int(tendon_range_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        has_fl = (
            1
            if (
                int(d_dof_frictionloss_ptr) != 0
                and (int(dof_frictionloss_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        has_ws = 1 if int(d_world_states_ptr) != 0 else 0
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp3PreSolveArgs(
            idx=int(d_idx_ptr),
            tyre_model=int(d_tyre_model_ptr),
            terrain_desc=int(d_terrain_desc_ptr),
            terrain_f32=int(d_terrain_f32_ptr),
            terrain_i32=int(d_terrain_i32_ptr),
            configs=int(d_sp3_configs_ptr),
            sp3_states=int(d_sp3_states_ptr),
            world_states=int(d_world_states_ptr) if has_ws else int(self._dummy_ws.ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xpos=int(d_xpos_ptr),
            xmat=int(d_xmat_ptr),
            xanchor=int(d_xanchor_ptr),
            xaxis=int(d_xaxis_ptr),
            qfrc_applied=int(d_qfrc_ptr),
            ctrl=int(d_ctrl_ptr),
            tendon_range=int(d_tendon_range_ptr) if has_tr else int(self._dummy_f32.ptr),
            dof_frictionloss=(
                int(d_dof_frictionloss_ptr) if has_fl else int(self._dummy_f32.ptr)
            ),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            ntendon=int(ntendon),
            tendon_range_world_stride=int(tendon_range_world_stride),
            dof_frictionloss_world_stride=int(dof_frictionloss_world_stride),
            has_world_states=has_ws,
            has_tendon_range=has_tr,
            has_dof_frictionloss=has_fl,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp3_pre_solve,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP3_PRESOLVE_PTR_OFFSETS,
            18,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(
                f"wp_metal_launch_kernel(bike_wp_sp3_pre_solve_kernel) failed: {err}"
            )

    def launch_sp3_post_solve(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_sp3_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xmat_ptr: int,
        nsensordata: int = 0,
        d_sensordata_ptr: int = 0,
        d_nefc_ptr: int = 0,
        njmax: int = 0,
        efc_J_row_stride: int = 0,
        d_efc_type_ptr: int = 0,
        d_efc_id_ptr: int = 0,
        d_efc_J_ptr: int = 0,
        d_efc_force_ptr: int = 0,
        d_nacon_ptr: int = 0,
        nconmax: int = 0,
        contact_efc_stride: int = 1,
        d_contact_worldid_ptr: int = 0,
        d_contact_geom_ptr: int = 0,
        d_contact_efc_address_ptr: int = 0,
        d_contact_dim_ptr: int = 0,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        has_sd = 1 if (int(d_sensordata_ptr) != 0 and int(nsensordata) > 0) else 0
        has_efc = 1 if (int(d_nefc_ptr) != 0 and int(njmax) > 0) else 0
        has_con = (
            1
            if (
                int(d_nacon_ptr) != 0
                and int(nconmax) > 0
                and int(d_contact_geom_ptr) != 0
                and int(d_contact_efc_address_ptr) != 0
            )
            else 0
        )
        has_cwid = 1 if (has_con and int(d_contact_worldid_ptr) != 0) else 0
        has_cdim = 1 if (has_con and int(d_contact_dim_ptr) != 0) else 0
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp3PostSolveArgs(
            idx=int(d_idx_ptr),
            configs=int(d_sp3_configs_ptr),
            sp3_states=int(d_sp3_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xmat=int(d_xmat_ptr),
            sensordata=int(d_sensordata_ptr) if has_sd else int(self._dummy_f32.ptr),
            nefc=int(d_nefc_ptr) if has_efc else int(self._dummy_i32.ptr),
            efc_type=int(d_efc_type_ptr) if (has_efc and int(d_efc_type_ptr) != 0) else int(self._dummy_i32.ptr),
            efc_id=int(d_efc_id_ptr) if (has_efc and int(d_efc_id_ptr) != 0) else int(self._dummy_i32.ptr),
            efc_J=int(d_efc_J_ptr) if (has_efc and int(d_efc_J_ptr) != 0) else int(self._dummy_f32.ptr),
            efc_force=int(d_efc_force_ptr) if (has_efc and int(d_efc_force_ptr) != 0) else int(self._dummy_f32.ptr),
            nacon=int(d_nacon_ptr) if has_con else int(self._dummy_i32.ptr),
            contact_worldid=int(d_contact_worldid_ptr) if has_cwid else int(self._dummy_i32.ptr),
            contact_geom=int(d_contact_geom_ptr) if has_con else int(self._dummy_i32.ptr),
            contact_efc_address=int(d_contact_efc_address_ptr) if has_con else int(self._dummy_i32.ptr),
            contact_dim=int(d_contact_dim_ptr) if has_cdim else int(self._dummy_i32.ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            nsensordata=int(nsensordata),
            njmax=int(njmax),
            efc_J_row_stride=int(efc_J_row_stride),
            nconmax=int(nconmax),
            contact_efc_stride=max(1, int(contact_efc_stride)),
            has_sensordata=has_sd,
            has_efc=has_efc,
            has_contacts=has_con,
            has_contact_worldid=has_cwid,
            has_contact_dim=has_cdim,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp3_post_solve,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP3_POSTSOLVE_PTR_OFFSETS,
            17,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(
                f"wp_metal_launch_kernel(bike_wp_sp3_post_solve_kernel) failed: {err}"
            )

    def launch_sp4_init(
        self,
        nworld: int,
        d_idx_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_sp3_states_ptr: int,
        d_sp4_states_ptr: int,
        d_world_states_ptr: int = 0,
        d_qpos_ptr: int = 0,
        init_grade: float = 0.0,
        ntendon: int = 0,
        d_tendon_range_ptr: int = 0,
        tendon_range_world_stride: int = 0,
    ) -> None:
        has_ws = 1 if int(d_world_states_ptr) != 0 else 0
        has_qpos = 1 if int(d_qpos_ptr) != 0 else 0
        has_tr = (
            1
            if (
                int(d_tendon_range_ptr) != 0
                and int(ntendon) > 0
                and (int(tendon_range_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp4InitArgs(
            idx=int(d_idx_ptr),
            configs=int(d_sp3_configs_ptr),
            sp3_states=int(d_sp3_states_ptr),
            sp4_states=int(d_sp4_states_ptr),
            world_states=int(d_world_states_ptr) if has_ws else int(self._dummy_ws.ptr),
            qpos=int(d_qpos_ptr) if has_qpos else int(self._dummy_f32.ptr),
            tendon_range=int(d_tendon_range_ptr) if has_tr else int(self._dummy_f32.ptr),
            nworld=int(nworld),
            per_world_config=int(per_world_config),
            init_grade=float(init_grade),
            ntendon=int(ntendon),
            tendon_range_world_stride=int(tendon_range_world_stride),
            has_world_states=has_ws,
            has_qpos=has_qpos,
            has_tendon_range=has_tr,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp4_init,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP4_INIT_PTR_OFFSETS,
            7,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_sp4_init_kernel) failed: {err}")

    def launch_sp4_pre_solve_and_pass0(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_tyre_model_ptr: int,
        d_terrain_desc_ptr: int,
        d_terrain_f32_ptr: int,
        d_terrain_i32_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_spindle_params_ptr: int,
        d_sp3_states_ptr: int,
        d_sp4_states_ptr: int,
        d_world_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_qfrc_bias_ptr: int,
        d_xpos_ptr: int,
        d_xmat_ptr: int,
        d_xanchor_ptr: int,
        d_xaxis_ptr: int,
        d_site_xpos_ptr: int,
        d_subtree_com_ptr: int,
        d_body_subtreemass_ptr: int,
        qM_row_stride: int,
        M_world_stride: int,
        d_M_rowadr_ptr: int,
        d_M_rownnz_ptr: int,
        d_M_colind_ptr: int,
        d_M_ptr: int,
        d_qfrc_ptr: int,
        d_ctrl_ptr: int,
        ntendon: int = 0,
        d_tendon_range_ptr: int = 0,
        tendon_range_world_stride: int = 0,
        d_dof_frictionloss_ptr: int = 0,
        dof_frictionloss_world_stride: int = 0,
        d_qacc_warmstart_ptr: int = 0,
    ) -> None:
        has_submass = 1 if int(d_body_subtreemass_ptr) != 0 else 0
        has_tr = (
            1
            if (
                int(d_tendon_range_ptr) != 0
                and int(ntendon) > 0
                and (int(tendon_range_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        has_fl = (
            1
            if (
                int(d_dof_frictionloss_ptr) != 0
                and (int(dof_frictionloss_world_stride) > 0 or int(nworld) == 1)
            )
            else 0
        )
        has_ws = 1 if int(d_qacc_warmstart_ptr) != 0 else 0
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp4PreSolveArgs(
            idx=int(d_idx_ptr),
            tyre_model=int(d_tyre_model_ptr),
            terrain_desc=int(d_terrain_desc_ptr),
            terrain_f32=int(d_terrain_f32_ptr),
            terrain_i32=int(d_terrain_i32_ptr),
            configs=int(d_sp3_configs_ptr),
            spindle_params=int(d_spindle_params_ptr),
            sp3_states=int(d_sp3_states_ptr),
            sp4_states=int(d_sp4_states_ptr),
            world_states=int(d_world_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            qfrc_bias=int(d_qfrc_bias_ptr),
            xpos=int(d_xpos_ptr),
            xmat=int(d_xmat_ptr),
            xanchor=int(d_xanchor_ptr),
            xaxis=int(d_xaxis_ptr),
            site_xpos=int(d_site_xpos_ptr),
            subtree_com=int(d_subtree_com_ptr),
            body_subtreemass=(
                int(d_body_subtreemass_ptr) if has_submass else int(self._dummy_f32.ptr)
            ),
            M_rowadr=int(d_M_rowadr_ptr) if int(d_M_rowadr_ptr) != 0 else int(self._dummy_i32.ptr),
            M_rownnz=int(d_M_rownnz_ptr) if int(d_M_rownnz_ptr) != 0 else int(self._dummy_i32.ptr),
            M_colind=int(d_M_colind_ptr) if int(d_M_colind_ptr) != 0 else int(self._dummy_i32.ptr),
            d_M=int(d_M_ptr),
            qfrc_applied=int(d_qfrc_ptr),
            ctrl=int(d_ctrl_ptr),
            tendon_range=int(d_tendon_range_ptr) if has_tr else int(self._dummy_f32.ptr),
            dof_frictionloss=(
                int(d_dof_frictionloss_ptr) if has_fl else int(self._dummy_f32.ptr)
            ),
            qacc_warmstart=int(d_qacc_warmstart_ptr) if has_ws else int(self._dummy_f32.ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            qM_row_stride=int(qM_row_stride),
            M_world_stride=int(M_world_stride),
            ntendon=int(ntendon),
            tendon_range_world_stride=int(tendon_range_world_stride),
            dof_frictionloss_world_stride=int(dof_frictionloss_world_stride),
            has_body_subtreemass=has_submass,
            has_tendon_range=has_tr,
            has_dof_frictionloss=has_fl,
            has_qacc_warmstart=has_ws,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp4_pre_solve,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP4_PRESOLVE_PTR_OFFSETS,
            29,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(
                f"wp_metal_launch_kernel(bike_wp_sp4_pre_solve_and_pass0_kernel) failed: {err}"
            )

    def launch_sp4_pass(
        self,
        stage: int,
        nworld: int,
        nq: int,
        nv: int,
        nu: int,
        d_spindle_params_ptr: int,
        d_sp4_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_qfrc_bias_ptr: int,
        d_qacc_ptr: int,
        qM_row_stride: int,
        M_world_stride: int,
        d_M_rowadr_ptr: int,
        d_M_rownnz_ptr: int,
        d_M_colind_ptr: int,
        d_M_ptr: int,
        d_ctrl_ptr: int,
        d_qacc_warmstart_ptr: int = 0,
    ) -> None:
        kernel_map = {1: self.k_sp4_pass1, 2: self.k_sp4_pass2, 3: self.k_sp4_pass3}
        k_pass = kernel_map[int(stage)]
        has_ws = 1 if int(d_qacc_warmstart_ptr) != 0 else 0
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp4PassArgs(
            spindle_params=int(d_spindle_params_ptr),
            sp4_states=int(d_sp4_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            qfrc_bias=int(d_qfrc_bias_ptr),
            qacc=int(d_qacc_ptr),
            M_rowadr=int(d_M_rowadr_ptr) if int(d_M_rowadr_ptr) != 0 else int(self._dummy_i32.ptr),
            M_rownnz=int(d_M_rownnz_ptr) if int(d_M_rownnz_ptr) != 0 else int(self._dummy_i32.ptr),
            M_colind=int(d_M_colind_ptr) if int(d_M_colind_ptr) != 0 else int(self._dummy_i32.ptr),
            d_M=int(d_M_ptr),
            ctrl=int(d_ctrl_ptr),
            qacc_warmstart=int(d_qacc_warmstart_ptr) if has_ws else int(self._dummy_f32.ptr),
            nworld=int(nworld),
            nq=int(nq),
            nv=int(nv),
            nu=int(nu),
            qM_row_stride=int(qM_row_stride),
            M_world_stride=int(M_world_stride),
            has_qacc_warmstart=has_ws,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            k_pass,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP4_PASS_PTR_OFFSETS,
            12,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(
                f"wp_metal_launch_kernel(bike_wp_sp4_pass{stage}_kernel) failed: {err}"
            )

    def launch_sp4_post_solve(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_sp3_configs_ptr: int,
        per_world_config: int,
        d_spindle_params_ptr: int,
        d_sp3_states_ptr: int,
        d_sp4_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xpos_ptr: int,
        d_xmat_ptr: int,
        d_xanchor_ptr: int,
        nsensordata: int = 0,
        d_sensordata_ptr: int = 0,
        d_nefc_ptr: int = 0,
        njmax: int = 0,
        efc_J_row_stride: int = 0,
        d_efc_type_ptr: int = 0,
        d_efc_id_ptr: int = 0,
        d_efc_J_ptr: int = 0,
        d_efc_force_ptr: int = 0,
        d_nacon_ptr: int = 0,
        nconmax: int = 0,
        contact_efc_stride: int = 1,
        d_contact_worldid_ptr: int = 0,
        d_contact_geom_ptr: int = 0,
        d_contact_efc_address_ptr: int = 0,
        d_contact_dim_ptr: int = 0,
    ) -> None:
        has_xp = 1 if int(d_xpos_ptr) != 0 else 0
        has_xa = 1 if int(d_xanchor_ptr) != 0 else 0
        has_sd = 1 if (int(d_sensordata_ptr) != 0 and int(nsensordata) > 0) else 0
        has_efc = 1 if (int(d_nefc_ptr) != 0 and int(njmax) > 0) else 0
        has_con = (
            1
            if (
                int(d_nacon_ptr) != 0
                and int(nconmax) > 0
                and int(d_contact_geom_ptr) != 0
                and int(d_contact_efc_address_ptr) != 0
            )
            else 0
        )
        has_cwid = 1 if (has_con and int(d_contact_worldid_ptr) != 0) else 0
        has_cdim = 1 if (has_con and int(d_contact_dim_ptr) != 0) else 0
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpSp4PostSolveArgs(
            idx=int(d_idx_ptr),
            configs=int(d_sp3_configs_ptr),
            spindle_params=int(d_spindle_params_ptr),
            sp3_states=int(d_sp3_states_ptr),
            sp4_states=int(d_sp4_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xpos=int(d_xpos_ptr) if has_xp else int(self._dummy_f32.ptr),
            xmat=int(d_xmat_ptr),
            xanchor=int(d_xanchor_ptr) if has_xa else int(self._dummy_f32.ptr),
            sensordata=int(d_sensordata_ptr) if has_sd else int(self._dummy_f32.ptr),
            nefc=int(d_nefc_ptr) if has_efc else int(self._dummy_i32.ptr),
            efc_type=(
                int(d_efc_type_ptr) if (has_efc and int(d_efc_type_ptr) != 0) else int(self._dummy_i32.ptr)
            ),
            efc_id=(
                int(d_efc_id_ptr) if (has_efc and int(d_efc_id_ptr) != 0) else int(self._dummy_i32.ptr)
            ),
            efc_J=(
                int(d_efc_J_ptr) if (has_efc and int(d_efc_J_ptr) != 0) else int(self._dummy_f32.ptr)
            ),
            efc_force=(
                int(d_efc_force_ptr) if (has_efc and int(d_efc_force_ptr) != 0) else int(self._dummy_f32.ptr)
            ),
            nacon=int(d_nacon_ptr) if has_con else int(self._dummy_i32.ptr),
            contact_worldid=int(d_contact_worldid_ptr) if has_cwid else int(self._dummy_i32.ptr),
            contact_geom=int(d_contact_geom_ptr) if has_con else int(self._dummy_i32.ptr),
            contact_efc_address=(
                int(d_contact_efc_address_ptr) if has_con else int(self._dummy_i32.ptr)
            ),
            contact_dim=int(d_contact_dim_ptr) if has_cdim else int(self._dummy_i32.ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            nsensordata=int(nsensordata),
            njmax=int(njmax),
            efc_J_row_stride=int(efc_J_row_stride),
            nconmax=int(nconmax),
            contact_efc_stride=max(1, int(contact_efc_stride)),
            has_xpos=has_xp,
            has_xanchor=has_xa,
            has_sensordata=has_sd,
            has_efc=has_efc,
            has_contacts=has_con,
            has_contact_worldid=has_cwid,
            has_contact_dim=has_cdim,
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_sp4_post_solve,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _SP4_POSTSOLVE_PTR_OFFSETS,
            21,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(
                f"wp_metal_launch_kernel(bike_wp_sp4_post_solve_kernel) failed: {err}"
            )


def _init_metal_device() -> str:
    if wp is None or mujoco is None or mjwarp is None:
        raise RuntimeError("warp-lang, warp-metal, mujoco, and mujoco_warp are required.")
    DEFAULT_WARP_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    wp.config.kernel_cache_dir = str(DEFAULT_WARP_CACHE_DIR.resolve())
    wp.init()
    wp.set_device("metal:0")
    return str(wp.get_device("metal:0").name)


def run_mjwarp_metal_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    warmup_steps: int = 25,
    seed: int = 42,
    peak_sample_stride: int = 25,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    nconmax: int = 64,
    njmax: int = 128,
    backend: str = "compliant_2d",
    station_count: int = 64,
) -> dict[str, Any]:
    """Experiment A: Heterogeneous parameter sweep on Apple Metal (`clean_split`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    m = load_model_with_assets(model_xml_path)
    _ensure_newton_solver(m)
    d0 = _prepare_initial_mjdata(m, model_xml_path)
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
    configs_bytes = build_sweep_configs_bytes(
        lib=lib,
        nworld=nworld,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
    )
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)
    (
        d_tyre_model,
        d_terrain_desc,
        _d_terrain_f32,
        _d_terrain_i32,
        terrain_f32_ptr,
        terrain_i32_ptr,
    ) = _upload_tyre_and_terrain_device(
        lib, model_xml_path, backend, station_count, idx_bytes, device=device
    )

    m_wp = mjwarp.put_model(m)
    if m.ntendon > 0 and hasattr(m_wp, "tendon_range"):
        tr_np = np.tile(m.tendon_range.astype(np.float32)[None, :, :], (nworld, 1, 1))
        m_wp.tendon_range = wp.array(tr_np, dtype=wp.vec2f, device=device)

    eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
    eff_njmax = max(int(njmax), int(d0.nefc) + 64)

    def _init_data() -> Any:
        data_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
        mjwarp.forward(m_wp, data_b)
        tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
        wm_rt.launch_init_states(
            nworld=nworld,
            d_idx_ptr=int(d_idx.ptr),
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=1,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(data_b.qpos.ptr),
        )
        wp.synchronize()
        return data_b, tr_ptr, tr_stride

    def _make_writer_fn(data_b: Any, tr_ptr: int, tr_stride: int) -> Any:
        def _launch_writers() -> None:
            wm_rt.launch_writers(
                nworld=nworld,
                dt_s=dt_s,
                d_idx_ptr=int(d_idx.ptr),
                d_tyre_model_ptr=int(d_tyre_model.ptr),
                d_terrain_desc_ptr=int(d_terrain_desc.ptr),
                d_terrain_f32_ptr=terrain_f32_ptr,
                d_terrain_i32_ptr=terrain_i32_ptr,
                d_configs_ptr=int(d_configs.ptr),
                per_world_config=1,
                d_states_ptr=int(d_states.ptr),
                d_qpos_ptr=int(data_b.qpos.ptr),
                d_qvel_ptr=int(data_b.qvel.ptr),
                d_xpos_ptr=int(data_b.xpos.ptr),
                d_xmat_ptr=int(data_b.xmat.ptr),
                d_xanchor_ptr=int(data_b.xanchor.ptr),
                d_xaxis_ptr=int(data_b.xaxis.ptr),
                d_qfrc_ptr=int(data_b.qfrc_applied.ptr),
                d_ctrl_ptr=int(data_b.ctrl.ptr),
                tendon_range_ptr=tr_ptr,
                tendon_range_stride=tr_stride,
            )

        return _launch_writers

    # Warmup 1 uncaptured step to ensure all mjwarp kernels are compiled before capture
    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    _mjwarp_clean_split_step(ops, m_wp, d_b, _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride))
    wp.synchronize()

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph

    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph
    wp.synchronize()

    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph

    fork_qpos_idx = int(idx_struct.fork_travel_qpos)
    shock_qpos_idx = int(idx_struct.shock_stroke_qpos)
    root_x_qpos_idx = int(idx_struct.root_x_qpos)

    peak_fork_mm = np.zeros(nworld, dtype=np.float32)
    peak_shock_mm = np.zeros(nworld, dtype=np.float32)
    stride = max(1, int(peak_sample_stride))

    steps_done = 0
    while steps_done < n_steps:
        chunk = min(stride, n_steps - steps_done)
        for _ in range(chunk):
            wp.capture_launch(graph)
        wp.synchronize()
        qpos_host = d_b.qpos.numpy()
        peak_fork_mm = np.maximum(peak_fork_mm, qpos_host[:, fork_qpos_idx] * 1000.0)
        peak_shock_mm = np.maximum(peak_shock_mm, qpos_host[:, shock_qpos_idx] * 1000.0)
        steps_done += chunk

    final_qpos = d_b.qpos.numpy()
    forward_distance_m = final_qpos[:, root_x_qpos_idx] - float(d0.qpos[root_x_qpos_idx])
    finite_all = bool(np.all(np.isfinite(final_qpos)))

    total_world_steps = int(nworld) * int(n_steps)
    us_per_step_per_world = (wall_time_s * 1e6) / float(total_world_steps)
    steps_per_s = float(total_world_steps) / max(wall_time_s, 1e-12)
    rtf = (float(total_world_steps) * dt_s) / max(wall_time_s, 1e-12)

    return {
        "experiment": "A_mjwarp_metal_parameter_sweep",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "schedule": "clean_split",
        "tyre_backend": backend,
        "gpu_device": gpu_name,
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "wall_time_ms": wall_time_s * 1000.0,
        "us_per_step_per_world": us_per_step_per_world,
        "steps_per_s": steps_per_s,
        "realtime_factor_x": rtf,
        "all_finite": finite_all,
        "sweep_ranges": {
            "fork_psi_scale": [0.80, 1.25],
            "rebound_scale": [0.70, 1.40],
            "shock_rate_scale": [0.85, 1.20],
            "tire_stiff_scale": [0.85, 1.20],
        },
        "metrics": {
            "peak_fork_travel_mm": _summarize_distribution(peak_fork_mm),
            "peak_shock_stroke_mm": _summarize_distribution(peak_shock_mm),
            "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
        },
    }


def run_mjwarp_metal_tyre_backend_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    warmup_steps: int = 20,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    nconmax: int = 64,
    njmax: int = 128,
) -> dict[str, Any]:
    """Experiment B: Shared tyre backend scaling on Apple Metal (`clean_split`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    m = load_model_with_assets(model_xml_path)
    _ensure_newton_solver(m)
    d0 = _prepare_initial_mjdata(m, model_xml_path)
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    m_wp = mjwarp.put_model(m)
    if m.ntendon > 0 and hasattr(m_wp, "tendon_range"):
        tr_np = np.tile(m.tendon_range.astype(np.float32)[None, :, :], (nworld, 1, 1))
        m_wp.tendon_range = wp.array(tr_np, dtype=wp.vec2f, device=device)
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    configs_bytes = build_sweep_configs_bytes(lib=lib, nworld=1)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)

    tier_results: list[dict[str, Any]] = []
    baseline_us: float | None = None

    eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
    eff_njmax = max(int(njmax), int(d0.nefc) + 64)

    for tier in TYRE_BACKEND_TIERS:
        backend = str(tier["backend"])
        station_count = int(tier["station_count"])
        units_per_world = int(tier["units_per_world"])
        (
            d_tyre_model,
            d_terrain_desc,
            _d_terrain_f32,
            _d_terrain_i32,
            terrain_f32_ptr,
            terrain_i32_ptr,
        ) = _upload_tyre_and_terrain_device(
            lib, model_xml_path, backend, station_count, idx_bytes, device=device
        )

        d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
        mjwarp.forward(m_wp, d_b)

        wm_rt.launch_init_states(
            nworld=nworld,
            d_idx_ptr=int(d_idx.ptr),
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=0,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
        )
        wp.synchronize()

        def _launch_writers() -> None:
            wm_rt.launch_writers(
                nworld=nworld,
                dt_s=dt_s,
                d_idx_ptr=int(d_idx.ptr),
                d_tyre_model_ptr=int(d_tyre_model.ptr),
                d_terrain_desc_ptr=int(d_terrain_desc.ptr),
                d_terrain_f32_ptr=terrain_f32_ptr,
                d_terrain_i32_ptr=terrain_i32_ptr,
                d_configs_ptr=int(d_configs.ptr),
                per_world_config=0,
                d_states_ptr=int(d_states.ptr),
                d_qpos_ptr=int(d_b.qpos.ptr),
                d_qvel_ptr=int(d_b.qvel.ptr),
                d_xpos_ptr=int(d_b.xpos.ptr),
                d_xmat_ptr=int(d_b.xmat.ptr),
                d_xanchor_ptr=int(d_b.xanchor.ptr),
                d_xaxis_ptr=int(d_b.xaxis.ptr),
                d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
                d_ctrl_ptr=int(d_b.ctrl.ptr),
                tendon_range_ptr=tendon_range_ptr,
                tendon_range_stride=tendon_range_stride,
            )

        _mjwarp_clean_split_step(ops, m_wp, d_b, _launch_writers)
        wp.synchronize()

        with wp.ScopedCapture() as capture:
            _mjwarp_clean_split_step(ops, m_wp, d_b, _launch_writers)
        graph = capture.graph

        for _ in range(warmup_steps):
            wp.capture_launch(graph)
        wp.synchronize()

        t0 = time.perf_counter()
        for _ in range(n_steps):
            wp.capture_launch(graph)
        wp.synchronize()
        wall_time_s = time.perf_counter() - t0

        qpos_host = d_b.qpos.numpy()
        finite_all = bool(np.all(np.isfinite(qpos_host)))

        total_world_steps = int(nworld) * int(n_steps)
        us_per_step_per_world = (wall_time_s * 1e6) / float(total_world_steps)
        steps_per_s = float(total_world_steps) / max(wall_time_s, 1e-12)
        rtf = (float(total_world_steps) * dt_s) / max(wall_time_s, 1e-12)

        if baseline_us is None:
            baseline_us = us_per_step_per_world
        delta_us = us_per_step_per_world - baseline_us
        pct_over = ((us_per_step_per_world / baseline_us) - 1.0) * 100.0 if baseline_us > 0.0 else 0.0

        tier_results.append(
            {
                "tier_id": tier["tier_id"],
                "label": tier["label"],
                "backend": backend,
                "station_count": station_count,
                "units_per_wheel": int(tier["units_per_wheel"]),
                "units_per_world": units_per_world,
                "wall_time_ms": wall_time_s * 1000.0,
                "us_per_step_per_world": us_per_step_per_world,
                "delta_us_vs_compliant": delta_us,
                "pct_over_compliant": pct_over,
                "steps_per_s": steps_per_s,
                "realtime_factor_x": rtf,
                "all_finite": finite_all,
            }
        )

    return {
        "experiment": "B_mjwarp_metal_tyre_backend_scaling",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "schedule": "clean_split",
        "gpu_device": gpu_name,
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "tiers": tier_results,
    }


def run_mjwarp_metal_sp3_autonomous_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 64,
    n_steps: int = 80,
    warmup_steps: int = 4,
    seed: int = 42,
    use_12gears: bool = True,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    nconmax: int = 64,
    njmax: int = 128,
    backend: str = "compliant_2d",
    station_count: int = 64,
) -> dict[str, Any]:
    """Autonomous SP3-C closed-loop sweep inside `wp.ScopedCapture()` on Apple Metal."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    with tempfile.TemporaryDirectory(prefix="bike-sp3-metal-") as sp3_tmp:
        resolved_xml = Path(model_xml_path).resolve()
        if use_12gears:
            xml12 = Path(sp3_tmp) / "model_12gears.xml"
            m_src = load_model_with_assets(resolved_xml)
            if int(m_src.nhfielddata) > 0:
                (Path(sp3_tmp) / "hfield_data.bin").write_bytes(
                    np.asarray(m_src.hfield_data, dtype="<f8").tobytes()
                )
            cb = resolved_xml.parent / "captured_bootstrap.json"
            if cb.is_file():
                (Path(sp3_tmp) / "captured_bootstrap.json").write_text(
                    cb.read_text(encoding="utf-8"), encoding="utf-8"
                )
            resolved_xml = materialize_model_12gears_xml(resolved_xml, xml12)

        m = load_model_with_assets(resolved_xml)
        _ensure_newton_solver(m)
        d0 = _prepare_initial_mjdata(
            m, model_xml_path if not use_12gears else resolved_xml
        )
        dt_s = float(m.opt.timestep)
        ops = _resolve_clean_split_ops(mjwarp)

        idx_bytes = resolve_gpu_model_indices(m)
        idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
        scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
        sp3_configs_bytes = build_sp3_sweep_configs_bytes(
            lib=lib,
            nworld=nworld,
            m_mj=m,
            fork_psi_scale=scales["fork_psi_scale"],
            rebound_scale=scales["rebound_scale"],
            shock_rate_scale=scales["shock_rate_scale"],
            tire_stiff_scale=scales["tire_stiff_scale"],
        )
        sizeof_ws = int(lib.bike_gpu_sizeof_world_state())
        sizeof_sp3 = int(lib.bike_gpu_sizeof_sp3_world_state())

        d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
        d_sp3_configs = _allocate_and_copy_device_bytes(sp3_configs_bytes, device=device)
        d_world_states = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
        d_sp3_states = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
        (
            d_tyre_model,
            d_terrain_desc,
            _d_tf32,
            _d_ti32,
            terrain_f32_ptr,
            terrain_i32_ptr,
        ) = _upload_tyre_and_terrain_device(
            lib, resolved_xml, backend, station_count, idx_bytes, device=device
        )

        m_wp = mjwarp.put_model(m)
        expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)

        eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
        eff_njmax = max(int(njmax), int(d0.nefc) + 64)

        def _alloc_and_init() -> tuple[Any, int, int, int, int, dict[str, int]]:
            d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
            mjwarp.forward(m_wp, d_b)
            wp.synchronize()
            tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
            fl_ptr, fl_stride = _resolve_dof_frictionloss_binding(m_wp, nworld)
            post_b = _resolve_sp3_post_solve_bindings(d_b)
            wm_rt.launch_sp3_init(
                nworld=nworld,
                d_idx_ptr=int(d_idx.ptr),
                d_sp3_configs_ptr=int(d_sp3_configs.ptr),
                per_world_config=1,
                d_sp3_states_ptr=int(d_sp3_states.ptr),
                d_world_states_ptr=int(d_world_states.ptr),
                d_qpos_ptr=int(d_b.qpos.ptr),
                init_grade=0.0,
                ntendon=int(m.ntendon),
                d_tendon_range_ptr=tr_ptr,
                tendon_range_world_stride=tr_stride,
            )
            wp.synchronize()
            return d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b

        def _make_step_fns(
            d_b: Any,
            tr_ptr: int,
            tr_stride: int,
            fl_ptr: int,
            fl_stride: int,
            post_b: dict[str, int],
        ) -> tuple[Any, Any]:
            def _pre() -> None:
                wm_rt.launch_sp3_pre_solve(
                    nworld=nworld,
                    dt_s=dt_s,
                    d_idx_ptr=int(d_idx.ptr),
                    d_tyre_model_ptr=int(d_tyre_model.ptr),
                    d_terrain_desc_ptr=int(d_terrain_desc.ptr),
                    d_terrain_f32_ptr=terrain_f32_ptr,
                    d_terrain_i32_ptr=terrain_i32_ptr,
                    d_sp3_configs_ptr=int(d_sp3_configs.ptr),
                    per_world_config=1,
                    d_sp3_states_ptr=int(d_sp3_states.ptr),
                    d_world_states_ptr=int(d_world_states.ptr),
                    d_qpos_ptr=int(d_b.qpos.ptr),
                    d_qvel_ptr=int(d_b.qvel.ptr),
                    d_xpos_ptr=int(d_b.xpos.ptr),
                    d_xmat_ptr=int(d_b.xmat.ptr),
                    d_xanchor_ptr=int(d_b.xanchor.ptr),
                    d_xaxis_ptr=int(d_b.xaxis.ptr),
                    d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
                    d_ctrl_ptr=int(d_b.ctrl.ptr),
                    ntendon=int(m.ntendon),
                    d_tendon_range_ptr=tr_ptr,
                    tendon_range_world_stride=tr_stride,
                    d_dof_frictionloss_ptr=fl_ptr,
                    dof_frictionloss_world_stride=fl_stride,
                )

            def _post() -> None:
                wm_rt.launch_sp3_post_solve(
                    nworld=nworld,
                    dt_s=dt_s,
                    d_idx_ptr=int(d_idx.ptr),
                    d_sp3_configs_ptr=int(d_sp3_configs.ptr),
                    per_world_config=1,
                    d_sp3_states_ptr=int(d_sp3_states.ptr),
                    d_qpos_ptr=int(d_b.qpos.ptr),
                    d_qvel_ptr=int(d_b.qvel.ptr),
                    d_xmat_ptr=int(d_b.xmat.ptr),
                    nsensordata=int(post_b["nsensordata"]),
                    d_sensordata_ptr=int(post_b["sensordata_ptr"]),
                    d_nefc_ptr=int(post_b["nefc_ptr"]),
                    njmax=int(post_b["njmax"]),
                    efc_J_row_stride=int(post_b["efc_J_row_stride"]),
                    d_efc_type_ptr=int(post_b["efc_type_ptr"]),
                    d_efc_id_ptr=int(post_b["efc_id_ptr"]),
                    d_efc_J_ptr=int(post_b["efc_J_ptr"]),
                    d_efc_force_ptr=int(post_b["efc_force_ptr"]),
                    d_nacon_ptr=int(post_b["nacon_ptr"]),
                    nconmax=int(post_b["nconmax"]),
                    contact_efc_stride=int(post_b["contact_efc_stride"]),
                    d_contact_worldid_ptr=int(post_b["contact_worldid_ptr"]),
                    d_contact_geom_ptr=int(post_b["contact_geom_ptr"]),
                    d_contact_efc_address_ptr=int(post_b["contact_efc_address_ptr"]),
                    d_contact_dim_ptr=int(post_b["contact_dim_ptr"]),
                )

            return _pre, _post

        # Warmup uncaptured step so all Metal kernels are compiled before capture
        d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b = _alloc_and_init()
        pre_fn, post_fn = _make_step_fns(d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b)
        _mjwarp_sp3_clean_split_step(ops, m_wp, d_b, pre_fn, post_fn)
        wp.synchronize()

        # Re-init cleanly and capture graph
        d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b = _alloc_and_init()
        pre_fn, post_fn = _make_step_fns(d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b)
        with wp.ScopedCapture() as capture:
            _mjwarp_sp3_clean_split_step(ops, m_wp, d_b, pre_fn, post_fn)
        graph = capture.graph
        for _ in range(warmup_steps):
            wp.capture_launch(graph)
        wp.synchronize()

        # Re-init cleanly for timed closed-loop rollout
        d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b = _alloc_and_init()
        pre_fn, post_fn = _make_step_fns(d_b, tr_ptr, tr_stride, fl_ptr, fl_stride, post_b)
        with wp.ScopedCapture() as capture:
            _mjwarp_sp3_clean_split_step(ops, m_wp, d_b, pre_fn, post_fn)
        graph = capture.graph
        wp.synchronize()

        t0 = time.perf_counter()
        for _ in range(n_steps):
            wp.capture_launch(graph)
        wp.synchronize()
        wall_time_s = time.perf_counter() - t0

        final_qpos = d_b.qpos.numpy()
        final_qvel = d_b.qvel.numpy()
        all_finite = bool(np.all(np.isfinite(final_qpos)) and np.all(np.isfinite(final_qvel)))
        forward_distance_m = final_qpos[:, int(idx_struct.root_x_qpos)] - float(
            d0.qpos[int(idx_struct.root_x_qpos)]
        )
        total_world_steps = int(nworld) * int(n_steps)
        return {
            "experiment": "SP3_C_metal_autonomous",
            "backend": "metal_mjwarp_full_gpu_fp32",
            "schedule": "clean_split",
            "gpu_device": gpu_name,
            "ntendon": int(m.ntendon),
            "nworld": int(nworld),
            "n_steps": int(n_steps),
            "dt_s": dt_s,
            "sizeof_sp3_world_state_bytes": sizeof_sp3,
            "wall_time_ms": wall_time_s * 1000.0,
            "us_per_step_per_world": (wall_time_s * 1e6) / float(total_world_steps),
            "steps_per_s": float(total_world_steps) / max(wall_time_s, 1e-12),
            "all_finite": all_finite,
            "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
            "final_qpos_world0": final_qpos[0].tolist(),
            "final_qvel_world0": final_qvel[0].tolist(),
        }


def _run_metal_sp4_batch(
    *,
    lib: ctypes.CDLL,
    wm_rt: WarpMetalWritersRuntime,
    xml_path: Path,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    qacc_ws0: np.ndarray | None,
    nworld: int,
    n_steps: int,
    warmup_steps: int,
    surface_label: int,
    backend: str,
    station_count: int,
    control_period_steps: int,
    cfg_bytes: bytes,
    njmax: int = 48,
    device: str = "metal:0",
) -> dict[str, Any]:
    m = load_model_with_assets(xml_path)
    _ensure_newton_solver(m)
    d0 = mujoco.MjData(m)
    d0.qpos[:] = qpos0
    d0.qvel[:] = qvel0
    mujoco.mj_forward(m, d0)
    if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
        d0.qacc_warmstart[:] = qacc_ws0
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
    spindle_bytes = build_spindle_params_bytes(
        host_lib=lib,
        xml_path=xml_path,
        strength_json_path=DEFAULT_STRENGTH_JSON,
        envelope_json_path=DEFAULT_ENVELOPE_JSON,
        control_period_steps=control_period_steps,
    )
    sizeof_ws = int(lib.bike_gpu_sizeof_world_state())
    sizeof_sp3 = int(lib.bike_gpu_sizeof_sp3_world_state())
    sizeof_sp4 = int(lib.bike_gpu_sizeof_sp4_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_sp3_configs = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_spindle_params = _allocate_and_copy_device_bytes(spindle_bytes, device=device)
    d_world_states = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
    d_sp3_states = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
    d_sp4_states = wp.zeros(nworld * sizeof_sp4, dtype=wp.uint8, device=device)
    (
        d_tyre_model,
        d_terrain_desc,
        _d_tf32,
        _d_ti32,
        terrain_f32_ptr,
        terrain_i32_ptr,
    ) = _upload_tyre_and_terrain_device(
        lib,
        xml_path,
        backend,
        station_count,
        idx_bytes,
        device=device,
        surface_label=surface_label,
    )

    m_wp = _disable_contacts_for_sp4(m, device=device)
    expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
    init_tr_np = m_wp.tendon_range.numpy().copy()
    init_fl_np = m_wp.dof_frictionloss.numpy().copy()
    eff_njmax = max(int(njmax), int(d0.nefc) + 16)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=0, njmax=eff_njmax)
    mjwarp.forward(m_wp, d_b)
    if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
        ws_np = np.tile(qacc_ws0.astype(np.float32)[None, :], (nworld, 1))
        d_b.qacc_warmstart.assign(wp.array(ws_np, dtype=wp.float32, device=device))
    wp.synchronize()

    tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
    fl_ptr, fl_stride = _resolve_dof_frictionloss_binding(m_wp, nworld)
    post_b = _resolve_sp3_post_solve_bindings(d_b)
    submass_arr = getattr(m_wp, "body_subtreemass", None)
    submass_ptr = int(submass_arr.ptr) if submass_arr is not None else 0
    m_rowadr_ptr = int(m_wp.M_rowadr.ptr)
    m_rownnz_ptr = int(m_wp.M_rownnz.ptr)
    m_colind_ptr = int(m_wp.M_colind.ptr)
    m_world_stride = int(d_b.M.shape[1])
    xz_ctx = _build_sp4_xz_rebase_ctx(m, nworld, device)

    def _init_states() -> None:
        if xz_ctx is not None:
            xz_ctx["xz_off"].zero_()
        d_world_states.zero_()
        d_sp3_states.zero_()
        d_sp4_states.zero_()
        m_wp.tendon_range.assign(
            wp.array(init_tr_np, dtype=m_wp.tendon_range.dtype, device=device)
        )
        m_wp.dof_frictionloss.assign(
            wp.array(init_fl_np, dtype=m_wp.dof_frictionloss.dtype, device=device)
        )
        assert int(m_wp.tendon_range.ptr) == tr_ptr
        assert int(m_wp.dof_frictionloss.ptr) == fl_ptr
        d_b.qpos.assign(
            wp.array(np.tile(qpos0.astype(np.float32)[None, :], (nworld, 1)), dtype=wp.float32, device=device)
        )
        d_b.qvel.assign(
            wp.array(np.tile(qvel0.astype(np.float32)[None, :], (nworld, 1)), dtype=wp.float32, device=device)
        )
        if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
            d_b.qacc_warmstart.assign(
                wp.array(
                    np.tile(qacc_ws0.astype(np.float32)[None, :], (nworld, 1)),
                    dtype=wp.float32,
                    device=device,
                )
            )
        else:
            d_b.qacc_warmstart.zero_()
        d_b.ctrl.zero_()
        d_b.qfrc_applied.zero_()
        d_b.time.zero_()
        if int(m.na) > 0:
            d_b.act.zero_()
        mjwarp.forward(m_wp, d_b)
        wp.synchronize()
        wm_rt.launch_sp4_init(
            nworld=nworld,
            d_idx_ptr=int(d_idx.ptr),
            d_sp3_configs_ptr=int(d_sp3_configs.ptr),
            per_world_config=1,
            d_sp3_states_ptr=int(d_sp3_states.ptr),
            d_sp4_states_ptr=int(d_sp4_states.ptr),
            d_world_states_ptr=int(d_world_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            init_grade=0.0,
            ntendon=int(m.ntendon),
            d_tendon_range_ptr=tr_ptr,
            tendon_range_world_stride=tr_stride,
        )
        wp.synchronize()

    def _pre() -> None:
        wm_rt.launch_sp4_pre_solve_and_pass0(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_tyre_model_ptr=int(d_tyre_model.ptr),
            d_terrain_desc_ptr=int(d_terrain_desc.ptr),
            d_terrain_f32_ptr=terrain_f32_ptr,
            d_terrain_i32_ptr=terrain_i32_ptr,
            d_sp3_configs_ptr=int(d_sp3_configs.ptr),
            per_world_config=1,
            d_spindle_params_ptr=int(d_spindle_params.ptr),
            d_sp3_states_ptr=int(d_sp3_states.ptr),
            d_sp4_states_ptr=int(d_sp4_states.ptr),
            d_world_states_ptr=int(d_world_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_qfrc_bias_ptr=int(d_b.qfrc_bias.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_xmat_ptr=int(d_b.xmat.ptr),
            d_xanchor_ptr=int(d_b.xanchor.ptr),
            d_xaxis_ptr=int(d_b.xaxis.ptr),
            d_site_xpos_ptr=int(d_b.site_xpos.ptr),
            d_subtree_com_ptr=int(d_b.subtree_com.ptr),
            d_body_subtreemass_ptr=submass_ptr,
            qM_row_stride=0,
            M_world_stride=m_world_stride,
            d_M_rowadr_ptr=m_rowadr_ptr,
            d_M_rownnz_ptr=m_rownnz_ptr,
            d_M_colind_ptr=m_colind_ptr,
            d_M_ptr=int(d_b.M.ptr),
            d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
            d_ctrl_ptr=int(d_b.ctrl.ptr),
            ntendon=int(m.ntendon),
            d_tendon_range_ptr=tr_ptr,
            tendon_range_world_stride=tr_stride,
            d_dof_frictionloss_ptr=fl_ptr,
            dof_frictionloss_world_stride=fl_stride,
            d_qacc_warmstart_ptr=int(d_b.qacc_warmstart.ptr),
        )

    def _make_pass(stage: int) -> Any:
        def _run() -> None:
            wm_rt.launch_sp4_pass(
                stage=stage,
                nworld=nworld,
                nq=int(m.nq),
                nv=int(m.nv),
                nu=int(m.nu),
                d_spindle_params_ptr=int(d_spindle_params.ptr),
                d_sp4_states_ptr=int(d_sp4_states.ptr),
                d_qpos_ptr=int(d_b.qpos.ptr),
                d_qvel_ptr=int(d_b.qvel.ptr),
                d_qfrc_bias_ptr=int(d_b.qfrc_bias.ptr),
                d_qacc_ptr=int(d_b.qacc.ptr),
                qM_row_stride=0,
                M_world_stride=m_world_stride,
                d_M_rowadr_ptr=m_rowadr_ptr,
                d_M_rownnz_ptr=m_rownnz_ptr,
                d_M_colind_ptr=m_colind_ptr,
                d_M_ptr=int(d_b.M.ptr),
                d_ctrl_ptr=int(d_b.ctrl.ptr),
                d_qacc_warmstart_ptr=int(d_b.qacc_warmstart.ptr),
            )

        return _run

    p1 = _make_pass(1)
    p2 = _make_pass(2)
    p3 = _make_pass(3)

    def _post() -> None:
        wm_rt.launch_sp4_post_solve(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_sp3_configs_ptr=int(d_sp3_configs.ptr),
            per_world_config=1,
            d_spindle_params_ptr=int(d_spindle_params.ptr),
            d_sp3_states_ptr=int(d_sp3_states.ptr),
            d_sp4_states_ptr=int(d_sp4_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_xmat_ptr=int(d_b.xmat.ptr),
            d_xanchor_ptr=int(d_b.xanchor.ptr),
            nsensordata=int(post_b["nsensordata"]),
            d_sensordata_ptr=int(post_b["sensordata_ptr"]),
            d_nefc_ptr=int(post_b["nefc_ptr"]),
            njmax=int(post_b["njmax"]),
            efc_J_row_stride=int(post_b["efc_J_row_stride"]),
            d_efc_type_ptr=int(post_b["efc_type_ptr"]),
            d_efc_id_ptr=int(post_b["efc_id_ptr"]),
            d_efc_J_ptr=int(post_b["efc_J_ptr"]),
            d_efc_force_ptr=int(post_b["efc_force_ptr"]),
            d_nacon_ptr=int(post_b["nacon_ptr"]),
            nconmax=int(post_b["nconmax"]),
            contact_efc_stride=int(post_b["contact_efc_stride"]),
            d_contact_worldid_ptr=int(post_b["contact_worldid_ptr"]),
            d_contact_geom_ptr=int(post_b["contact_geom_ptr"]),
            d_contact_efc_address_ptr=int(post_b["contact_efc_address_ptr"]),
            d_contact_dim_ptr=int(post_b["contact_dim_ptr"]),
        )

    # Warmup uncaptured step so Metal shaders compile before ScopedCapture
    _init_states()
    _mjwarp_sp4_clean_split_step(
        ops, m_wp, d_b, _pre, p1, p2, p3, _post, run_collision=False, xz_ctx=xz_ctx
    )
    wp.synchronize()

    _init_states()
    with wp.ScopedCapture() as capture:
        _mjwarp_sp4_clean_split_step(
            ops, m_wp, d_b, _pre, p1, p2, p3, _post, run_collision=False, xz_ctx=xz_ctx
        )
    graph = capture.graph
    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    _init_states()
    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    final_qpos = d_b.qpos.numpy()
    final_qvel = d_b.qvel.numpy()
    all_finite = bool(np.all(np.isfinite(final_qpos)) and np.all(np.isfinite(final_qvel)))
    forward_distance_m = final_qpos[:, int(idx_struct.root_x_qpos)] - float(
        qpos0[int(idx_struct.root_x_qpos)]
    )
    sp4_raw = d_sp4_states.numpy().tobytes()
    sp3_raw = d_sp3_states.numpy().tobytes()
    world0_metrics = unpack_sp4_world_metrics(sp4_raw[:sizeof_sp4])
    sp3_buf0 = ctypes.create_string_buffer(sp3_raw[:sizeof_sp3], sizeof_sp3)
    has_failure0 = int(lib.bike_gpu_sp3_state_has_failure(ctypes.cast(sp3_buf0, ctypes.c_void_p)))
    total_world_steps = int(nworld) * int(n_steps)
    sim_time_s = float(n_steps) * dt_s
    return {
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "backend": backend,
        "station_count": int(station_count),
        "control_period_steps": int(control_period_steps),
        "wall_time_ms": wall_time_s * 1000.0,
        "ms_per_step": (wall_time_s * 1000.0) / max(1, int(n_steps)),
        "us_per_step_per_world": (wall_time_s * 1e6) / max(1, total_world_steps),
        "steps_per_s": float(total_world_steps) / max(wall_time_s, 1e-12),
        "realtime_factor_x": (sim_time_s * float(nworld)) / max(wall_time_s, 1e-12),
        "all_finite": all_finite,
        "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
        "world0_metrics": world0_metrics,
        "world0_first_failure": "violation" if has_failure0 != 0 else "none",
    }


def run_mjwarp_metal_sp4_closed_loop_sweep(
    nworld: int = 2048,
    n_steps: int = 200,
    warmup_steps: int = 5,
    seed: int = 42,
    backend: str = "both",
    station_count: int = 64,
    control_period_steps: int = 1,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    sp4_bundle_dir: Path | None = None,
) -> dict[str, Any]:
    """Full SP4 closed-loop 4x-solve Metal Graph throughput sweep across tyre backends."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    backends = (
        ("compliant_2d", "distributed_2d_reference")
        if backend == "both"
        else (backend,)
    )
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
    runs = []
    with tempfile.TemporaryDirectory(prefix="bike-sp4-metal-") as td:
        xml12, qpos0, qvel0, qacc_ws0 = _export_sp4_scenario_bundle(
            "tests/fixtures/tracks/flat.toml",
            0.0,
            Path(td),
            sp4_bundle_dir=sp4_bundle_dir,
            scenario_id="S1_flat",
        )
        m = load_model_with_assets(xml12)
        cfg_bytes = _build_sp4_config_bytes(
            lib=lib,
            m_mj=m,
            nworld=nworld,
            surface_label=1,
            v0=0.0,
            fork_psi_scale=scales["fork_psi_scale"],
            rebound_scale=scales["rebound_scale"],
            shock_rate_scale=scales["shock_rate_scale"],
            tire_stiff_scale=scales["tire_stiff_scale"],
        )
        for bk in backends:
            res = _run_metal_sp4_batch(
                lib=lib,
                wm_rt=wm_rt,
                xml_path=xml12,
                qpos0=qpos0,
                qvel0=qvel0,
                qacc_ws0=qacc_ws0,
                nworld=nworld,
                n_steps=n_steps,
                warmup_steps=warmup_steps,
                surface_label=1,
                backend=bk,
                station_count=station_count if bk == "distributed_2d_reference" else 0,
                control_period_steps=control_period_steps,
                cfg_bytes=cfg_bytes,
                device=device,
            )
            runs.append(res)
    return {
        "experiment": "SP4_closed_loop_4solve_metal",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "gpu_device": gpu_name,
        "runs": runs,
    }


def run_mjwarp_metal_sp4_s1s4_acceptance(
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    sp4_bundle_dir: Path | None = None,
) -> dict[str, Any]:
    """Run closed-loop S1-S4 acceptance scenarios on Apple Metal (`4x solve`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    scenario_results = []
    all_passed = True
    for sc in _build_sp4_s1s4_scenarios(sp4_bundle_dir=sp4_bundle_dir):
        with tempfile.TemporaryDirectory(prefix=f"bike-s1s4-metal-{sc['id']}-") as td:
            xml12, qpos0, qvel0, qacc_ws0 = _export_sp4_scenario_bundle(
                sc["track"],
                sc["v0"],
                Path(td),
                sp4_bundle_dir=sp4_bundle_dir,
                scenario_id=sc["id"],
            )
            m = load_model_with_assets(xml12)
            cfg_bytes = _build_sp4_config_bytes(
                lib=lib,
                m_mj=m,
                nworld=1,
                surface_label=sc["surface_label"],
                v0=sc["v0"],
                human_torque_nm=sc["human_torque_nm"],
                motor_torque_nm=sc["motor_torque_nm"],
            )
            n_steps = int(round(sc["duration_s"] / float(m.opt.timestep)))
            res = _run_metal_sp4_batch(
                lib=lib,
                wm_rt=wm_rt,
                xml_path=xml12,
                qpos0=qpos0,
                qvel0=qvel0,
                qacc_ws0=qacc_ws0,
                nworld=1,
                n_steps=n_steps,
                warmup_steps=2,
                surface_label=sc["surface_label"],
                backend="compliant_2d",
                station_count=0,
                control_period_steps=1,
                cfg_bytes=cfg_bytes,
                device=device,
            )
            wm = res["world0_metrics"]
            dist = float(wm["distance_m"])
            m0_dist = float(sc["m0_distance_m"])
            rel_err = abs(dist - m0_dist) / max(1.0, abs(m0_dist))
            min_press = min(float(wm["min_press_front_n"]), float(wm["min_press_rear_n"]))
            max_rise_f = wm["max_rising_front_excess_n"]
            max_rise_r = wm["max_rising_rear_excess_n"]
            max_rise = (
                max(float(max_rise_f), float(max_rise_r))
                if (max_rise_f is not None and max_rise_r is not None)
                else (
                    float(max_rise_f)
                    if max_rise_f is not None
                    else (float(max_rise_r) if max_rise_r is not None else None)
                )
            )
            min_saddle = float(wm["min_saddle_force_n"])
            max_saddle = float(wm["max_saddle_force_n"])
            passed = (
                res["all_finite"]
                and res["world0_first_failure"] == "none"
                and min_press >= -6.0
                and (max_rise is not None and max_rise <= 6.0)
                and min_saddle >= -6.0
                and max_saddle <= 1750.0
                and rel_err <= 0.002
            )
            if not passed:
                all_passed = False
            scenario_results.append(
                {
                    "id": sc["id"],
                    "passed": bool(passed),
                    "first_failure": res["world0_first_failure"],
                    "distance_m": dist,
                    "m0_distance_m": m0_dist,
                    "distance_rel_err": rel_err,
                    "end_speed_mps": float(wm["end_speed_mps"]),
                    "min_press_front_n": float(wm["min_press_front_n"]),
                    "min_press_rear_n": float(wm["min_press_rear_n"]),
                    "min_press_n": min_press,
                    "max_rising_front_excess_n": max_rise_f,
                    "max_rising_rear_excess_n": max_rise_r,
                    "max_rising_excess_n": max_rise,
                    "min_saddle_force_n": min_saddle,
                    "max_saddle_force_n": max_saddle,
                    "wall_time_ms": res["wall_time_ms"],
                    "all_finite": res["all_finite"],
                }
            )
    return {
        "experiment": "SP4_S1S4_metal",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "gpu_device": gpu_name,
        "all_passed": bool(all_passed),
        "scenarios": scenario_results,
    }


def _print_report(results: dict[str, Any]) -> None:
    if "parameter_sweep" in results:
        sw = results["parameter_sweep"]
        print(
            f"\n=== Scenario 2 Experiment A: Full Metal GPU Parameter Sweep ({sw['gpu_device']}) ==="
        )
        print(
            f"nworld = {sw['nworld']} | n_steps = {sw['n_steps']} | "
            f"wall = {sw['wall_time_ms']:.2f} ms | "
            f"{sw['us_per_step_per_world']:.3f} us/step/world | "
            f"{sw['steps_per_s']:,.0f} steps/s | "
            f"RTF = {sw['realtime_factor_x']:.1f}x | finite = {sw['all_finite']}"
        )
        for metric_name, stats in sw["metrics"].items():
            print(
                f"  {metric_name}: "
                f"min={stats['min']:.3f}, p10={stats['p10']:.3f}, "
                f"median={stats['median']:.3f}, mean={stats['mean']:.3f}, "
                f"p90={stats['p90']:.3f}, max={stats['max']:.3f}"
            )

    for key in ("tyre_backend_scaling_batch", "tyre_backend_scaling_single"):
        if key in results:
            tr = results[key]
            print(
                f"\n=== Scenario 2 Experiment B: Tyre Backend Scaling "
                f"(nworld={tr['nworld']}, {tr['gpu_device']}) ==="
            )
            for tier in tr["tiers"]:
                print(
                    f"  {tier['label']:<35} | "
                    f"stations={tier['station_count']:3d} | "
                    f"wall={tier['wall_time_ms']:7.2f} ms | "
                    f"{tier['us_per_step_per_world']:6.3f} us/step/world "
                    f"({tier['pct_over_compliant']:+6.1f}%) | "
                    f"{tier['steps_per_s']:10,.0f} steps/s | "
                    f"RTF={tier['realtime_factor_x']:6.1f}x"
                )

    if "sp4_closed_loop" in results:
        sp4 = results["sp4_closed_loop"]
        print(f"\n=== SP4 Closed-Loop 4x-Solve Metal Throughput ({sp4['gpu_device']}) ===")
        for r in sp4["runs"]:
            print(
                f"  {r['backend']:<26} | nworld={r['nworld']} | n_steps={r['n_steps']} | "
                f"{r['ms_per_step']:.3f} ms/step | {r['steps_per_s']:10,.0f} ws/s | "
                f"RTF={r['realtime_factor_x']:.1f}x | finite={r['all_finite']}"
            )

    if "sp4_s1s4" in results:
        s1s4 = results["sp4_s1s4"]
        print(
            f"\n=== SP4 S1-S4 Metal Closed-Loop Acceptance "
            f"({s1s4['gpu_device']}, all_passed={s1s4['all_passed']}) ==="
        )
        for sc in s1s4["scenarios"]:
            mr = sc["max_rising_excess_n"]
            mr_str = f"{mr:+.2f}" if mr is not None else "None"
            print(
                f"  [{sc['id']}] pass={sc['passed']} fail={sc['first_failure']} "
                f"dist={sc['distance_m']:.4f} m0={sc['m0_distance_m']:.4f} "
                f"rel={sc['distance_rel_err'] * 100.0:.4f}% "
                f"min_press={sc['min_press_n']:+.2f} max_rise={mr_str} "
                f"saddle={sc['min_saddle_force_n']:+.1f}..{sc['max_saddle_force_n']:.1f}"
            )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run Scenario 2: Full Apple Metal GPU (warp-metal + clean_split mujoco_warp) benchmarks."
    )
    parser.add_argument(
        "--model-xml",
        type=Path,
        default=DEFAULT_MODEL_XML,
        help="Path to the MuJoCo XML model.",
    )
    parser.add_argument(
        "--nworld",
        type=int,
        default=2048,
        help="Number of parallel worlds in the GPU batch (default: 2048).",
    )
    parser.add_argument(
        "--n-steps",
        type=int,
        default=1000,
        help="Number of steps for Experiment A parameter sweep (default: 1000).",
    )
    parser.add_argument(
        "--tier-steps",
        type=int,
        default=500,
        help="Number of steps per tier for Experiment B tyre backend scaling (default: 500).",
    )
    parser.add_argument(
        "--experiment",
        choices=(
            "all",
            "sweep",
            "tire_scaling",
            "sp3_autonomous",
            "sp4_closed_loop",
            "sp4_s1s4",
            "sp4_all",
        ),
        default="all",
        help="Which benchmark experiment to execute.",
    )
    parser.add_argument(
        "--backend",
        choices=("both", "compliant_2d", "distributed_2d_reference"),
        default="both",
        help="Tyre backend for sp4_closed_loop sweep (default: both).",
    )
    parser.add_argument(
        "--sp4-bundle-dir",
        type=Path,
        default=None,
        help="Directory containing pre-exported S1-S4 scenario bundles (avoids bike-gpu-export).",
    )
    parser.add_argument(
        "--export-sp4-bundles",
        type=Path,
        default=None,
        help="Export standalone S1-S4 scenario bundles + manifest.json to DIR and exit.",
    )
    parser.add_argument(
        "--use-12gears",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Expand freehub tendon into 12 static cassette tendons for SP3 autonomous run.",
    )
    parser.add_argument(
        "--skip-single-world",
        action="store_true",
        help="Skip the nworld=1 single-world scaling run.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write JSON benchmark report.",
    )
    parser.add_argument(
        "--synthesize-welded-assets",
        action="store_true",
        help="Synthesize flat-road hfield_data.bin and settled welded initial state when missing.",
    )
    args = parser.parse_args(argv)

    if args.export_sp4_bundles is not None:
        out_root = export_sp4_bundles(args.export_sp4_bundles)
        print(f"Exported SP4 S1-S4 bundles to {out_root}")
        return 0

    _init_metal_device()
    lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    wm_rt = WarpMetalWritersRuntime(lib)

    results: dict[str, Any] = {
        "host_info": {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "machine": platform.machine(),
            "gpu_device": str(wp.get_device("metal:0").name),
            "warp_version": wp.__version__,
            "mujoco_warp_version": getattr(mjwarp, "__version__", "3.11.0"),
        }
    }

    with tempfile.TemporaryDirectory(prefix="bike-metal-assets-") as tmp_dir:
        model_xml_path = (
            materialize_welded_assets_dir(args.model_xml, tmp_dir)
            if args.synthesize_welded_assets
            else args.model_xml
        )
        if args.experiment in ("all", "sweep"):
            results["parameter_sweep"] = run_mjwarp_metal_parameter_sweep(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.n_steps,
                lib=lib,
                wm_rt=wm_rt,
            )

        if args.experiment in ("all", "tire_scaling"):
            results["tyre_backend_scaling_batch"] = run_mjwarp_metal_tyre_backend_scaling_benchmark(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.tier_steps,
                lib=lib,
                wm_rt=wm_rt,
            )
            if not args.skip_single_world:
                results["tyre_backend_scaling_single"] = run_mjwarp_metal_tyre_backend_scaling_benchmark(
                    model_xml_path=model_xml_path,
                    nworld=1,
                    n_steps=args.tier_steps,
                    lib=lib,
                    wm_rt=wm_rt,
                )

        if args.experiment == "sp3_autonomous":
            results["sp3_autonomous"] = run_mjwarp_metal_sp3_autonomous_sweep(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.n_steps,
                use_12gears=args.use_12gears,
                lib=lib,
                wm_rt=wm_rt,
            )

        if args.experiment in ("sp4_closed_loop", "sp4_all"):
            results["sp4_closed_loop"] = run_mjwarp_metal_sp4_closed_loop_sweep(
                nworld=args.nworld,
                n_steps=args.n_steps,
                backend=args.backend,
                lib=lib,
                wm_rt=wm_rt,
                sp4_bundle_dir=args.sp4_bundle_dir,
            )

        if args.experiment in ("sp4_s1s4", "sp4_all"):
            results["sp4_s1s4"] = run_mjwarp_metal_sp4_s1s4_acceptance(
                lib=lib,
                wm_rt=wm_rt,
                sp4_bundle_dir=args.sp4_bundle_dir,
            )

    _print_report(results)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
