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


class AirSpringParamsStruct(ctypes.Structure):
    _fields_ = [
        ("stanchion_inner_diam_mm", ctypes.c_float),
        ("total_travel_mm", ctypes.c_float),
        ("pos_chamber_length_mm", ctypes.c_float),
        ("neg_chamber_length_mm", ctypes.c_float),
        ("token_volume_cm3", ctypes.c_float),
        ("max_tokens", ctypes.c_int32),
        ("gamma", ctypes.c_float),
        ("atm_pressure_pa", ctypes.c_float),
        ("num_tokens", ctypes.c_int32),
        ("gauge_pressure_psi", ctypes.c_float),
    ]


class DamperCoreParamsStruct(ctypes.Structure):
    _fields_ = [
        ("max_hsc", ctypes.c_int32),
        ("max_lsc", ctypes.c_int32),
        ("max_reb", ctypes.c_int32),
        ("hsc_clicks", ctypes.c_int32),
        ("lsc_clicks", ctypes.c_int32),
        ("rebound_clicks", ctypes.c_int32),
        ("c_lsc_min", ctypes.c_float),
        ("c_lsc_max", ctypes.c_float),
        ("c_hsc_min", ctypes.c_float),
        ("c_hsc_max", ctypes.c_float),
        ("c_reb_min", ctypes.c_float),
        ("c_reb_max", ctypes.c_float),
        ("v_knee_comp", ctypes.c_float),
        ("v_knee_reb", ctypes.c_float),
    ]


class Charger3ParamsStruct(ctypes.Structure):
    _fields_ = [
        ("core", DamperCoreParamsStruct),
        ("total_travel_mm", ctypes.c_float),
        ("hbo_start_mm", ctypes.c_float),
        ("c_hbo_base", ctypes.c_float),
    ]


class SuperDeluxeParamsStruct(ctypes.Structure):
    _fields_ = [
        ("core", DamperCoreParamsStruct),
        ("total_stroke_mm", ctypes.c_float),
        ("max_hbo", ctypes.c_int32),
        ("hbo_clicks", ctypes.c_int32),
        ("lockout_firm", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
        ("hbo_start_mm", ctypes.c_float),
        ("c_hbo_min", ctypes.c_float),
        ("c_hbo_max", ctypes.c_float),
        ("lockout_preload_n", ctypes.c_float),
        ("lockout_stiffness", ctypes.c_float),
    ]


class CoilParamsStruct(ctypes.Structure):
    _fields_ = [
        ("rate_n_m", ctypes.c_float),
        ("preload_mm", ctypes.c_float),
        ("stroke_mm", ctypes.c_float),
        ("bumper_length_mm", ctypes.c_float),
        ("bumper_peak_n", ctypes.c_float),
    ]


class EndStopParamsStruct(ctypes.Structure):
    _fields_ = [
        ("stiffness_n_m", ctypes.c_float),
        ("damping_n_s_m", ctypes.c_float),
    ]


class SuspensionParamsStruct(ctypes.Structure):
    _fields_ = [
        ("air_spring", AirSpringParamsStruct),
        ("fork_damper", Charger3ParamsStruct),
        ("shock_damper", SuperDeluxeParamsStruct),
        ("coil", CoilParamsStruct),
        ("end_stops", EndStopParamsStruct),
    ]


class GpuSuspensionParamsStruct(ctypes.Structure):
    _fields_ = [
        ("params", SuspensionParamsStruct),
        ("fork_sag_fraction", ctypes.c_float),
        ("shock_sag_fraction", ctypes.c_float),
        ("fork_preload_n", ctypes.c_float),
        ("shock_preload_n", ctypes.c_float),
    ]


class ResistanceParamsStruct(ctypes.Structure):
    _fields_ = [
        ("crr", ctypes.c_float),
        ("rolling_taper_rad_s", ctypes.c_float),
        ("rho_kg_m3", ctypes.c_float),
        ("cda_m2", ctypes.c_float),
        ("wind_world_mps", ctypes.c_float * 3),
        ("point_body_m", ctypes.c_float * 3),
    ]


class RiderJointEnvelopeSpecStruct(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_int32),
        ("qpos_adrs", ctypes.c_int32 * 16),
        ("dof_adrs", ctypes.c_int32 * 16),
        ("lower", ctypes.c_float * 16),
        ("upper", ctypes.c_float * 16),
        ("stiffness_nm_rad", ctypes.c_float),
        ("margin_rad", ctypes.c_float),
    ]


class GpuRiderParamsStruct(ctypes.Structure):
    _fields_ = [
        ("envelope", RiderJointEnvelopeSpecStruct),
        ("kp_posture", ctypes.c_float * 9),
        ("kd_posture", ctypes.c_float * 9),
        ("target_qpos_rad", ctypes.c_float * 9),
        ("rider_enabled", ctypes.c_int32),
    ]


class BrakeHoldParamsStruct(ctypes.Structure):
    _fields_ = [
        ("torque_nm", ctypes.c_float),
        ("stiffness_nm_rad", ctypes.c_float),
        ("damping_nms_rad", ctypes.c_float),
        ("engage_radps", ctypes.c_float),
        ("release_radps", ctypes.c_float),
    ]


class ShiftingParamsStruct(ctypes.Structure):
    _fields_ = [
        ("front_teeth", ctypes.c_int32),
        ("initial_rear_teeth", ctypes.c_int32),
        ("cog_count", ctypes.c_int32),
        ("cassette", ctypes.c_int32 * 12),
        ("cassette_tendon_ids", ctypes.c_int32 * 12),
        ("target_cadence_min_rpm", ctypes.c_float),
        ("target_cadence_max_rpm", ctypes.c_float),
        ("shift_cooldown_ticks", ctypes.c_int32),
        ("shift_cut_duration_ticks", ctypes.c_int32),
        ("torque_factor_larger_cog", ctypes.c_float),
        ("torque_factor_smaller_cog", ctypes.c_float),
        ("cadence_smoothing_tau_s", ctypes.c_float),
        ("upshift_slip_limit_mps", ctypes.c_float),
        ("upshift_slip_mode", ctypes.c_uint8),
        ("enabled", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class PedalingParamsStruct(ctypes.Structure):
    _fields_ = [
        ("coast_above_rpm", ctypes.c_float),
        ("resume_below_rpm", ctypes.c_float),
        ("stop_time_s", ctypes.c_float),
        ("coast_cadence_tau_s", ctypes.c_float),
        ("mash_cadence_rpm", ctypes.c_float),
        ("mash_torque_nm", ctypes.c_float),
        ("effort_slew_nm_s", ctypes.c_float),
        ("reposition_min_effort_nm", ctypes.c_float),
        ("reposition_stall_window_ticks", ctypes.c_int32),
        ("reposition_stall_progress_rad", ctypes.c_float),
        ("reposition_cooldown_ticks", ctypes.c_int32),
        ("reposition_back_rate_rad_s", ctypes.c_float),
        ("reposition_timeout_ticks", ctypes.c_int32),
        ("reposition_phase_tolerance_rad", ctypes.c_float),
        ("reposition_noop_rad", ctypes.c_float),
        ("enabled", ctypes.c_uint8),
        ("reposition_on_stall", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class MotorProfileParamsStruct(ctypes.Structure):
    _fields_ = [
        ("eco", ctypes.c_float),
        ("tour", ctypes.c_float),
        ("emtb_low", ctypes.c_float),
        ("emtb_high", ctypes.c_float),
        ("turbo", ctypes.c_float),
        ("emtb_full_gain_at_nm", ctypes.c_float),
    ]


class AssistParamsStruct(ctypes.Structure):
    _fields_ = [
        ("gain", ctypes.c_float),
        ("max_torque", ctypes.c_float),
        ("max_power", ctypes.c_float),
        ("tau", ctypes.c_float),
        ("slew", ctypes.c_float),
        ("engage_torque_nm", ctypes.c_float),
        ("release_hold_ticks", ctypes.c_int32),
        ("cutoff_mps", ctypes.c_float),
        ("taper_width_mps", ctypes.c_float),
        ("profile", MotorProfileParamsStruct),
        ("curve_rpm", ctypes.c_float * 8),
        ("curve_nm", ctypes.c_float * 8),
        ("curve_len", ctypes.c_int32),
        ("profile_mode", ctypes.c_uint8),
        ("has_profile", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class IntentParamsStruct(ctypes.Structure):
    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
        ("period_s", ctypes.c_float),
        ("period_steps", ctypes.c_int32),
        ("delay_ticks", ctypes.c_int32),
        ("dead_time_ticks", ctypes.c_int32),
        ("target_crank_power_w", ctypes.c_float),
        ("max_crank_torque_nm", ctypes.c_float),
        ("lean_gain", ctypes.c_float),
        ("max_forward_lean_rad", ctypes.c_float),
        ("max_backward_lean_rad", ctypes.c_float),
        ("lean_rate_rad_s", ctypes.c_float),
        ("orientation_tau_s", ctypes.c_float),
        ("surge_power_w", ctypes.c_float),
        ("surge_grade", ctypes.c_float),
        ("surge_max_subticks", ctypes.c_int32),
        ("surge_recover_subticks", ctypes.c_int32),
        ("front_load_share_target", ctypes.c_float),
        ("lean_trim_gain_rad_s", ctypes.c_float),
        ("lean_trim_limit_rad", ctypes.c_float),
        ("min_elbow_flexion_deg", ctypes.c_float),
        ("climb_effort_cap_count", ctypes.c_int32),
        ("climb_effort_cap_grades", ctypes.c_float * 16),
        ("climb_effort_cap_grade_nm", ctypes.c_float * 16),
        ("climb_effort_rise_speed_mps", ctypes.c_float),
        ("climb_effort_full_speed_mps", ctypes.c_float),
    ]


class FrontLiftParamsStruct(ctypes.Structure):
    _fields_ = [
        ("hand_drop_ratio", ctypes.c_float),
        ("pitch_rate_up_rad_s", ctypes.c_float),
        ("pitch_rad", ctypes.c_float),
        ("ease_fraction", ctypes.c_float),
        ("ease_rate_per_s", ctypes.c_float),
        ("lean_add_rad", ctypes.c_float),
        ("lean_rate_rad_s", ctypes.c_float),
        ("period_s", ctypes.c_float),
        ("window_ticks", ctypes.c_int32),
        ("lag_ticks", ctypes.c_int32),
        ("decision_ticks", ctypes.c_int32),
        ("clear_ticks", ctypes.c_int32),
        ("ease_enabled", ctypes.c_uint8),
        ("lean_enabled", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class LaunchParamsStruct(ctypes.Structure):
    _fields_ = [
        ("start_cog_count", ctypes.c_int32),
        ("start_cog_grade_breaks", ctypes.c_float * 16),
        ("start_cog_teeth", ctypes.c_int32 * 16),
        ("start_pedal_angle_rad", ctypes.c_float),
        ("start_lean_count", ctypes.c_int32),
        ("start_lean_grades", ctypes.c_float * 16),
        ("start_lean_rad", ctypes.c_float * 16),
        ("plan_enabled", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
        ("standstill_speed_mps", ctypes.c_float),
        ("standstill_crank_rpm", ctypes.c_float),
        ("effort_rate_nm_s", ctypes.c_float),
        ("effort_cap_asphalt_nm", ctypes.c_float),
        ("effort_cap_hardpack_nm", ctypes.c_float),
        ("effort_cap_loose_nm", ctypes.c_float),
        ("effort_cap_wet_nm", ctypes.c_float),
        ("effort_cap_scree_nm", ctypes.c_float),
        ("effort_cap_count", ctypes.c_int32),
        ("effort_cap_grades", ctypes.c_float * 16),
        ("effort_cap_grade_nm", ctypes.c_float * 16),
        ("effort_rise_speed_mps", ctypes.c_float),
        ("release_torque_count", ctypes.c_int32),
        ("release_torque_grades", ctypes.c_float * 16),
        ("release_torque_nm", ctypes.c_float * 16),
        ("upshift_crank_rpm", ctypes.c_float),
        ("end_speed_mps", ctypes.c_float),
        ("end_crank_rpm", ctypes.c_float),
        ("period_s", ctypes.c_float),
        ("release_ticks", ctypes.c_int32),
        ("max_hold_ticks", ctypes.c_int32),
        ("end_hold_ticks", ctypes.c_int32),
        ("max_launch_ticks", ctypes.c_int32),
        ("handover_ticks", ctypes.c_int32),
        ("retry_ticks", ctypes.c_int32),
    ]


class RollbackBrakeConfigStruct(ctypes.Structure):
    _fields_ = [
        ("rollback_demand", ctypes.c_float),
        ("rollback_engage_mps", ctypes.c_float),
        ("rollback_release_mps", ctypes.c_float),
        ("rollback_brake_enabled", ctypes.c_uint8),
        ("effort_drive_mode", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class GroundedFilterConfigStruct(ctypes.Structure):
    _fields_ = [("hold_ticks", ctypes.c_int32)]


class BridgedLoadConfigStruct(ctypes.Structure):
    _fields_ = [
        ("dropout_steps", ctypes.c_int32),
        ("threshold_n", ctypes.c_float),
    ]


class BalanceMonitorConfigStruct(ctypes.Structure):
    _fields_ = [
        ("floor_mps", ctypes.c_float),
        ("dwell_ticks", ctypes.c_int32),
        ("grace_ticks", ctypes.c_int32),
        ("period_s", ctypes.c_float),
    ]


class EfcScanConfigStruct(ctypes.Structure):
    _fields_ = [
        ("grip_left_eq_id", ctypes.c_int32),
        ("grip_right_eq_id", ctypes.c_int32),
        ("saddle_eq_id", ctypes.c_int32),
        ("foot_front_eq_id", ctypes.c_int32),
        ("foot_rear_eq_id", ctypes.c_int32),
        ("frame_body_id", ctypes.c_int32),
        ("crank_dof", ctypes.c_int32),
        ("front_wheel_dof", ctypes.c_int32),
        ("rear_wheel_dof", ctypes.c_int32),
        ("freehub_tendon_id", ctypes.c_int32),
        ("crank_clutch_tendon_id", ctypes.c_int32),
        ("handlebar_geom_id", ctypes.c_int32),
        ("catch_geom_id", ctypes.c_int32),
        ("terrain_geom_count", ctypes.c_int32),
        ("terrain_geom_ids", ctypes.c_int32 * 4),
        ("rider_geom_count", ctypes.c_int32),
        ("rider_geom_ids", ctypes.c_int32 * 16),
        ("load_threshold_n", ctypes.c_float),
        ("pitch_limit_rad", ctypes.c_float),
    ]


class SurfaceSectionCoreStruct(ctypes.Structure):
    _fields_ = [
        ("start_m", ctypes.c_float),
        ("end_m", ctypes.c_float),
        ("label", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuSp3ConfigStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuSp3Config``."""

    _fields_ = [
        ("suspension", GpuSuspensionParamsStruct),
        ("tire_stiff_scale", ctypes.c_float),
        ("resistance", ResistanceParamsStruct),
        ("rider", GpuRiderParamsStruct),
        ("static_brake_ceiling_nm", ctypes.c_float),
        ("brake_hold", BrakeHoldParamsStruct),
        ("front_max_brake_torque_nm", ctypes.c_float),
        ("rear_max_brake_torque_nm", ctypes.c_float),
        ("brake_taper_rad_s", ctypes.c_float),
        ("shifting", ShiftingParamsStruct),
        ("pedaling", PedalingParamsStruct),
        ("assist", AssistParamsStruct),
        ("intent", IntentParamsStruct),
        ("front_lift", FrontLiftParamsStruct),
        ("launch", LaunchParamsStruct),
        ("rollback", RollbackBrakeConfigStruct),
        ("grounded_filter", GroundedFilterConfigStruct),
        ("bridged_load", BridgedLoadConfigStruct),
        ("balance", BalanceMonitorConfigStruct),
        ("efc_scan", EfcScanConfigStruct),
        ("road_lookahead_m", ctypes.c_float),
        ("road_sample_spacing_m", ctypes.c_float),
        ("surface_sections", SurfaceSectionCoreStruct * 16),
        ("surface_section_count", ctypes.c_int32),
        ("default_surface_label", ctypes.c_uint8),
        ("pad0", ctypes.c_uint8 * 3),
        ("motor_actuator_idx", ctypes.c_int32),
        ("motor_ctrl_min_nm", ctypes.c_float),
        ("motor_ctrl_max_nm", ctypes.c_float),
        ("accel_sensor_adr", ctypes.c_int32),
        ("gyro_sensor_adr", ctypes.c_int32),
        ("crank_torque_ripple", ctypes.c_float),
        ("caller_front_brake", ctypes.c_float),
        ("caller_rear_brake", ctypes.c_float),
        ("caller_motor_torque_nm", ctypes.c_float),
        ("caller_motor_limit_nm", ctypes.c_float),
        ("caller_human_torque_nm", ctypes.c_float),
        ("lean_limit_rad", ctypes.c_float),
        ("min_lean_rad", ctypes.c_float),
        ("has_lean_limit", ctypes.c_uint8),
        ("has_min_lean", ctypes.c_uint8),
        ("effort_mode", ctypes.c_uint8),
        ("rider_present", ctypes.c_uint8),
        ("caller_braking", ctypes.c_uint8),
        ("apply_direct_crank_torque", ctypes.c_uint8),
        ("cut_after_ceiling", ctypes.c_uint8),
        ("seed_lean_from_launch", ctypes.c_uint8),
        ("arm_standstill_start", ctypes.c_uint8),
        ("use_bridged_load", ctypes.c_uint8),
        ("reposition_request", ctypes.c_uint8),
        ("pad1", ctypes.c_uint8),
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

    if hasattr(lib, "bike_gpu_sizeof_sp3_config"):
        lib.bike_gpu_sizeof_sp3_config.argtypes = []
        lib.bike_gpu_sizeof_sp3_config.restype = ctypes.c_int
        _assert_struct_size(lib, "bike_gpu_sizeof_sp3_config", GpuSp3ConfigStruct)

    if hasattr(lib, "bike_gpu_sizeof_sp3_world_state"):
        lib.bike_gpu_sizeof_sp3_world_state.argtypes = []
        lib.bike_gpu_sizeof_sp3_world_state.restype = ctypes.c_int
        sp3_st_bytes = int(lib.bike_gpu_sizeof_sp3_world_state())
        if sp3_st_bytes <= 0 or sp3_st_bytes > 16384:
            raise RuntimeError(f"Invalid GpuSp3WorldState size: {sp3_st_bytes} bytes")

    if hasattr(lib, "bike_gpu_init_default_sp3_config"):
        lib.bike_gpu_init_default_sp3_config.argtypes = [ctypes.c_void_p]
        lib.bike_gpu_init_default_sp3_config.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_build_sp3_sweep_configs"):
        lib.bike_gpu_build_sp3_sweep_configs.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        lib.bike_gpu_build_sp3_sweep_configs.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp3_init"):
        lib.bike_gpu_launch_sp3_init.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.bike_gpu_launch_sp3_init.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp3_pre_solve"):
        lib.bike_gpu_launch_sp3_pre_solve.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.bike_gpu_launch_sp3_pre_solve.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp3_post_solve"):
        lib.bike_gpu_launch_sp3_post_solve.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.bike_gpu_launch_sp3_post_solve.restype = ctypes.c_int

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


CASSETTE_12S_TEETH: tuple[int, ...] = (10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51)


def _replace_xml_range_attr(attrs: str) -> str:
    key = 'range="'
    pos = attrs.find(key)
    if pos < 0:
        return attrs + ' range="-1e12 1e12"'
    val_start = pos + len(key)
    val_end = attrs.find('"', val_start)
    if val_end < 0:
        raise ValueError("Malformed range attribute in ideal_mid_drive_freehub tag")
    return attrs[:val_start] + "-1e12 1e12" + attrs[val_end:]


def expand_model_12gears_xml_text(
    model_xml: str,
    front_teeth: int = 22,
    cassette: Sequence[int] = CASSETTE_12S_TEETH,
    motor_clutch: bool = True,
) -> str:
    """Expand ``ideal_mid_drive_freehub`` into static ``freehub_cog_00..11`` cassette tendons."""
    needle = '<fixed name="ideal_mid_drive_freehub"'
    start = model_xml.find(needle)
    if start < 0:
        raise ValueError("model XML does not contain ideal_mid_drive_freehub tendon")
    head_end = model_xml.find(">", start)
    if head_end < 0:
        raise ValueError("malformed ideal_mid_drive_freehub opening tag")
    close_tag = "</fixed>"
    block_end = model_xml.find(close_tag, head_end)
    if block_end < 0:
        raise ValueError("unclosed ideal_mid_drive_freehub block")

    attrs = _replace_xml_range_attr(model_xml[start + len(needle) : head_end])
    driver = "drive_shaft_spin" if motor_clutch else "crank_spin"
    blocks: list[str] = []
    for idx, teeth in enumerate(cassette):
        coef = float(front_teeth) / float(teeth)
        prefix = "" if idx == 0 else "    "
        blocks.append(
            f'{prefix}<fixed name="freehub_cog_{idx:02d}"{attrs}>\n'
            f'      <joint joint="{driver}" coef="{coef:.17g}"/>\n'
            f'      <joint joint="rear_wheel_spin" coef="-1"/>\n'
            f"    </fixed>"
        )
    replacement = "\n".join(blocks)
    return model_xml[:start] + replacement + model_xml[block_end + len(close_tag) :]


def materialize_model_12gears_xml(
    src_xml_path: Path | str,
    out_xml_path: Path | str,
    front_teeth: int = 22,
    cassette: Sequence[int] = CASSETTE_12S_TEETH,
    motor_clutch: bool = True,
) -> Path:
    """Materialize a 12-tendon cassette MuJoCo XML alongside the base model directory."""
    src_path = Path(src_xml_path)
    out_path = Path(out_xml_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    xml_text = expand_model_12gears_xml_text(
        src_path.read_text(encoding="utf-8"),
        front_teeth=front_teeth,
        cassette=cassette,
        motor_clutch=motor_clutch,
    )
    out_path.write_text(xml_text, encoding="utf-8")
    return out_path


def expand_mjwarp_tendon_range_and_frictionloss(
    m_mj: Any,
    m_wp: Any,
    nworld: int,
    device: Any,
) -> None:
    """Expand ``m_wp.tendon_range`` to ``(nworld, ntendon)`` and ``dof_frictionloss`` to ``(nworld, nv)``."""
    import warp as wp

    nworld = int(nworld)
    ntendon = int(m_mj.ntendon)
    nv = int(m_mj.nv)

    if ntendon > 0:
        tr_host = np.broadcast_to(
            np.asarray(m_mj.tendon_range, dtype=np.float32).reshape(1, ntendon, 2),
            (nworld, ntendon, 2),
        ).copy()
        m_wp.tendon_range = wp.from_numpy(tr_host, dtype=wp.vec2f, device=device)

    if nv > 0:
        fl_host = np.broadcast_to(
            np.asarray(m_mj.dof_frictionloss, dtype=np.float32).reshape(1, nv),
            (nworld, nv),
        ).copy()
        m_wp.dof_frictionloss = wp.from_numpy(fl_host, dtype=wp.float32, device=device)


def build_default_sp3_config_struct(lib: ctypes.CDLL) -> GpuSp3ConfigStruct:
    cfg = GpuSp3ConfigStruct()
    rc = int(lib.bike_gpu_init_default_sp3_config(ctypes.byref(cfg)))
    if rc != 0:
        raise RuntimeError(f"bike_gpu_init_default_sp3_config failed with return code {rc}")
    return cfg


def build_sp3_config_struct_from_mjmodel(lib: ctypes.CDLL, m_mj: Any) -> GpuSp3ConfigStruct:
    """Build a ``GpuSp3ConfigStruct`` with equality/sensor/actuator/tendon IDs resolved from ``m_mj``."""
    import mujoco

    cfg = build_default_sp3_config_struct(lib)
    if m_mj is None:
        return cfg

    def _name_id(obj_type: Any, name: str, fallback: int = -1) -> int:
        oid = int(mujoco.mj_name2id(m_mj, obj_type, name))
        return oid if oid >= 0 else fallback

    cfg.efc_scan.grip_left_eq_id = _name_id(mujoco.mjtObj.mjOBJ_EQUALITY, "connect_grip_left")
    cfg.efc_scan.grip_right_eq_id = _name_id(mujoco.mjtObj.mjOBJ_EQUALITY, "connect_grip_right")
    cfg.efc_scan.saddle_eq_id = _name_id(mujoco.mjtObj.mjOBJ_EQUALITY, "connect_saddle")
    cfg.efc_scan.foot_front_eq_id = _name_id(mujoco.mjtObj.mjOBJ_EQUALITY, "connect_foot_front")
    cfg.efc_scan.foot_rear_eq_id = _name_id(mujoco.mjtObj.mjOBJ_EQUALITY, "connect_foot_rear")
    cfg.efc_scan.frame_body_id = _lookup_body_id(m_mj, ("frame", "main_frame", "chassis"), 1)
    _, cfg.efc_scan.crank_dof = _lookup_joint_addrs(m_mj, ("crank_spin",), 3, 3)
    _, cfg.efc_scan.front_wheel_dof = _lookup_joint_addrs(m_mj, ("front_wheel_spin",), 7, 7)
    _, cfg.efc_scan.rear_wheel_dof = _lookup_joint_addrs(m_mj, ("rear_wheel_spin",), 10, 10)
    cfg.efc_scan.handlebar_geom_id = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "handlebar")
    cfg.efc_scan.catch_geom_id = -1
    terrain_gid = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "terrain")
    if terrain_gid >= 0:
        cfg.efc_scan.terrain_geom_ids[0] = terrain_gid
        cfg.efc_scan.terrain_geom_count = 1

    motor_act = _lookup_actuator_id(m_mj, ("mid_drive",), -1)
    cfg.motor_actuator_idx = motor_act
    if 0 <= motor_act < int(m_mj.nu):
        cfg.motor_ctrl_min_nm = float(m_mj.actuator_ctrlrange[motor_act, 0])
        cfg.motor_ctrl_max_nm = float(m_mj.actuator_ctrlrange[motor_act, 1])

    accel_id = _name_id(mujoco.mjtObj.mjOBJ_SENSOR, "sensor_frame_accel")
    gyro_id = _name_id(mujoco.mjtObj.mjOBJ_SENSOR, "sensor_frame_gyro")
    cfg.accel_sensor_adr = int(m_mj.sensor_adr[accel_id]) if accel_id >= 0 else -1
    cfg.gyro_sensor_adr = int(m_mj.sensor_adr[gyro_id]) if gyro_id >= 0 else -1

    if int(m_mj.ntendon) >= 12:
        for c in range(12):
            tid = _name_id(mujoco.mjtObj.mjOBJ_TENDON, f"freehub_cog_{c:02d}", c)
            cfg.shifting.cassette_tendon_ids[c] = tid
        cfg.efc_scan.freehub_tendon_id = int(cfg.shifting.cassette_tendon_ids[0])
    else:
        cfg.efc_scan.freehub_tendon_id = _lookup_tendon_id(
            m_mj, ("ideal_mid_drive_freehub",), 0
        )
    cfg.efc_scan.crank_clutch_tendon_id = _lookup_tendon_id(m_mj, ("crank_clutch",), 1)
    return cfg


def build_sp3_sweep_configs_bytes(
    lib: ctypes.CDLL,
    nworld: int,
    m_mj: Any = None,
    fork_psi_scale: np.ndarray | None = None,
    rebound_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
    tire_stiff_scale: np.ndarray | None = None,
) -> bytes:
    """Build ``nworld`` ``GpuSp3Config`` structs resolved against ``m_mj`` with optional sweep scales."""
    cfg_size = int(lib.bike_gpu_sizeof_sp3_config())
    buf = ctypes.create_string_buffer(nworld * cfg_size)
    _, fork_ptr = _as_float32_ptr(fork_psi_scale, nworld)
    _, reb_ptr = _as_float32_ptr(rebound_scale, nworld)
    _, shock_ptr = _as_float32_ptr(shock_rate_scale, nworld)
    _, tire_ptr = _as_float32_ptr(tire_stiff_scale, nworld)

    rc = int(
        lib.bike_gpu_build_sp3_sweep_configs(
            ctypes.cast(buf, ctypes.c_void_p),
            int(nworld),
            fork_ptr,
            reb_ptr,
            shock_ptr,
            tire_ptr,
        )
    )
    if rc != 0:
        raise RuntimeError(f"bike_gpu_build_sp3_sweep_configs failed with return code {rc}")

    if m_mj is not None:
        base_resolved = build_sp3_config_struct_from_mjmodel(lib, m_mj)
        arr_type = GpuSp3ConfigStruct * int(nworld)
        arr = arr_type.from_buffer(buf)
        for w in range(int(nworld)):
            arr[w].efc_scan = base_resolved.efc_scan
            arr[w].motor_actuator_idx = base_resolved.motor_actuator_idx
            arr[w].motor_ctrl_min_nm = base_resolved.motor_ctrl_min_nm
            arr[w].motor_ctrl_max_nm = base_resolved.motor_ctrl_max_nm
            arr[w].accel_sensor_adr = base_resolved.accel_sensor_adr
            arr[w].gyro_sensor_adr = base_resolved.gyro_sensor_adr
            arr[w].shifting.cassette_tendon_ids = base_resolved.shifting.cassette_tendon_ids
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
