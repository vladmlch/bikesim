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
        ("use_tabulated", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
        ("knot_count", ctypes.c_int32),
        ("deflection_m", ctypes.c_float * 64),
        ("force_n", ctypes.c_float * 64),
        ("slopes", ctypes.c_float * 63),
        ("integral", ctypes.c_float * 64),
    ]


class PneumaticMaterialViewStruct(ctypes.Structure):
    _fields_ = [
        ("outer_radius_mm", ctypes.c_float),
        ("rim_radius_mm", ctypes.c_float),
        ("section_height_mm", ctypes.c_float),
        ("pressure_bar", ctypes.c_float),
        ("crr_reference", ctypes.c_float),
        ("casing_width_mm", ctypes.c_float),
        ("compressed_casing_mm", ctypes.c_float),
        ("area_factor", ctypes.c_float),
        ("carcass_stiffness_n_mm2", ctypes.c_float),
        ("contact_length_factor", ctypes.c_float),
        ("rate_stiffening", ctypes.c_float),
        ("rate_relaxation_s", ctypes.c_float),
        ("loss_factor", ctypes.c_float),
        ("hysteresis_rate_eps_mps", ctypes.c_float),
        ("relaxation_length_mm", ctypes.c_float),
        ("rim_stiffness_n_mm2", ctypes.c_float),
        ("tread_loss_crr", ctypes.c_float),
        ("n_rays", ctypes.c_int32),
        ("discretised_brush", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
        ("half_angle_rad", ctypes.c_float),
        ("dtheta_rad", ctypes.c_float),
        ("theta_rad", ctypes.c_float * 256),
        ("ux", ctypes.c_float * 256),
        ("uz", ctypes.c_float * 256),
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
        ("pneumatic", PneumaticMaterialViewStruct),
    ]


class SurfaceSpecPodStruct(ctypes.Structure):
    _fields_ = [
        ("mu_peak", ctypes.c_float),
        ("mu_slide", ctypes.c_float),
        ("slip_stiffness_per_load", ctypes.c_float),
        ("stribeck_speed_mps", ctypes.c_float),
        ("crr_delta", ctypes.c_float),
        ("load_sensitivity", ctypes.c_float),
        ("surface_id", ctypes.c_int32),
    ]


class SurfaceSectionViewStruct(ctypes.Structure):
    _fields_ = [
        ("start_m", ctypes.c_float),
        ("end_m", ctypes.c_float),
        ("surface", SurfaceSpecPodStruct),
        ("blend_m", ctypes.c_float),
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
        ("road_x0_m", ctypes.c_float),
        ("road_dx_m", ctypes.c_float),
        ("road_is_uniform_grid", ctypes.c_uint8),
        ("pad_road", ctypes.c_uint8 * 3),
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


class GpuDrivetrainParamsStruct(ctypes.Structure):
    _fields_ = [
        ("chainring_teeth", ctypes.c_float),
        ("cog_teeth", ctypes.c_float),
        ("freehub_stiffness_nm_rad", ctypes.c_float),
        ("pawl_engage_rad", ctypes.c_float),
        ("max_rider_cadence_rad_s", ctypes.c_float),
        ("target_cadence_rad_s", ctypes.c_float),
        ("nominal_crank_torque_nm", ctypes.c_float),
        ("front_max_brake_torque_nm", ctypes.c_float),
        ("rear_max_brake_torque_nm", ctypes.c_float),
        ("brake_taper_rad_s", ctypes.c_float),
        ("static_brake_ceiling_nm", ctypes.c_float),
        ("brake_hold", BrakeHoldParamsStruct),
    ]


class GpuWorldConfigStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuWorldConfig``."""

    _fields_ = [
        ("suspension", GpuSuspensionParamsStruct),
        ("tire_stiff_scale", ctypes.c_float),
        ("resistance", ResistanceParamsStruct),
        ("drivetrain", GpuDrivetrainParamsStruct),
        ("rider", GpuRiderParamsStruct),
        ("pedal_demand", ctypes.c_float),
        ("front_brake_demand", ctypes.c_float),
        ("rear_brake_demand", ctypes.c_float),
    ]


class GpuCompliantWheelStateStruct(ctypes.Structure):
    _fields_ = [
        ("xi", ctypes.c_float),
        ("tangent", ctypes.c_float * 3),
        ("point", ctypes.c_float * 3),
        ("center", ctypes.c_float * 3),
        ("segment", ctypes.c_int32),
        ("has_tangent", ctypes.c_uint8),
        ("has_point", ctypes.c_uint8),
        ("has_center", ctypes.c_uint8),
        ("has_segment", ctypes.c_uint8),
    ]


class GpuStationStateStruct(ctypes.Structure):
    _fields_ = [
        ("xi", ctypes.c_float),
        ("tangent", ctypes.c_float * 3),
        ("active", ctypes.c_uint8),
        ("has_tangent", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class PneumaticCarcassStatePodStruct(ctypes.Structure):
    _fields_ = [
        ("delta_prev_m", ctypes.c_float * 256),
        ("elastic_prev_n", ctypes.c_float * 256),
        ("maxwell_force_n", ctypes.c_float * 256),
        ("rim_event_x_m", ctypes.c_float),
        ("rim_event_speed_mps", ctypes.c_float),
        ("rim_event_peak_load_n", ctypes.c_float),
        ("rim_event_peak_force_n", ctypes.c_float),
        ("rim_event_energy_j", ctypes.c_float),
        ("n_rays", ctypes.c_int32),
        ("initialized", ctypes.c_uint8),
        ("rim_strike_active", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class GpuPneumaticPatchMetaStruct(ctypes.Structure):
    _fields_ = [
        ("start", ctypes.c_int32),
        ("stop", ctypes.c_int32),
        ("kappa_prime", ctypes.c_float),
        ("detailed_kappa_prime", ctypes.c_float),
        ("has_detailed", ctypes.c_uint8),
        ("active", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
    ]


class GpuPneumaticWheelStateStruct(ctypes.Structure):
    _fields_ = [
        ("carcass", PneumaticCarcassStatePodStruct),
        ("bristle_deflection_m", ctypes.c_float * 256),
        ("patches", GpuPneumaticPatchMetaStruct * 16),
        ("patch_count", ctypes.c_int32),
    ]


class RatchetBoundStateStruct(ctypes.Structure):
    _fields_ = [
        ("boundary", ctypes.c_float),
        ("ratio", ctypes.c_float),
        ("has_boundary", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuDrivetrainStateStruct(ctypes.Structure):
    _fields_ = [
        ("freehub", RatchetBoundStateStruct),
        ("crank_clutch", RatchetBoundStateStruct),
        ("front_brake_latched", ctypes.c_uint8),
        ("rear_brake_latched", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 2),
        ("front_brake_latch_theta", ctypes.c_float),
        ("rear_brake_latch_theta", ctypes.c_float),
        ("crank_target_phase_rad", ctypes.c_float),
    ]


class GpuWorldStateStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuWorldState``."""

    _fields_ = [
        ("compliant_wheels", GpuCompliantWheelStateStruct * 2),
        ("distributed_wheels", (GpuStationStateStruct * 128) * 2),
        ("pneumatic_wheels", GpuPneumaticWheelStateStruct * 2),
        ("drivetrain", GpuDrivetrainStateStruct),
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


class CruiseConfigPodStruct(ctypes.Structure):
    _fields_ = [
        ("target_speed_kmh", ctypes.c_float),
        ("kp_nm_per_mps", ctypes.c_float),
        ("ki_nm_per_mps_s", ctypes.c_float),
        ("torque_ceiling_nm", ctypes.c_float),
    ]


class GpuProgramSegmentStruct(ctypes.Structure):
    _fields_ = [
        ("t_end_s", ctypes.c_float),
        ("channel_mask", ctypes.c_uint32),
        ("human_torque_nm", ctypes.c_float),
        ("motor_torque_nm", ctypes.c_float),
        ("motor_limit_nm", ctypes.c_float),
        ("crank_rate_rad_s", ctypes.c_float),
        ("front_brake", ctypes.c_float),
        ("rear_brake", ctypes.c_float),
        ("cruise_speed_kmh", ctypes.c_float),
        ("posture_lean_rad", ctypes.c_float),
    ]


class GpuRideProgramStruct(ctypes.Structure):
    _fields_ = [
        ("segments", GpuProgramSegmentStruct * 32),
        ("segment_count", ctypes.c_int32),
        ("reaction_delay_s", ctypes.c_float),
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
        ("cruise", CruiseConfigPodStruct),
        ("ride_program", GpuRideProgramStruct),
        ("control_period_steps", ctypes.c_int32),
        ("cruise_enabled", ctypes.c_uint8),
        ("program_enabled", ctypes.c_uint8),
        ("freeze_on_crash", ctypes.c_uint8),
        ("track_energy_ledger", ctypes.c_uint8),
    ]


class ShiftingStateStruct(ctypes.Structure):
    _fields_ = [
        ("rear_teeth", ctypes.c_int32),
        ("from_teeth", ctypes.c_int32),
        ("active_cog_idx", ctypes.c_int32),
        ("shift_count", ctypes.c_int32),
        ("cooldown_ticks", ctypes.c_int32),
        ("cut_remaining_ticks", ctypes.c_int32),
        ("cadence_ema", ctypes.c_float),
        ("required_ema", ctypes.c_float),
        ("direction", ctypes.c_uint8),
        ("rider_response", ctypes.c_uint8),
        ("has_ema", ctypes.c_uint8),
        ("pad", ctypes.c_uint8),
    ]


class RepositionStateStruct(ctypes.Structure):
    _fields_ = [
        ("crank_rad", ctypes.c_float),
        ("wheel_rad", ctypes.c_float),
        ("goal_rad", ctypes.c_float),
        ("stall_elapsed_ticks", ctypes.c_int32),
        ("elapsed_ticks", ctypes.c_int32),
        ("cooldown_ticks", ctypes.c_int32),
        ("active", ctypes.c_uint8),
        ("armed", ctypes.c_uint8),
        ("reason", ctypes.c_uint8),
        ("has_goal", ctypes.c_uint8),
    ]


class PedalingStateStruct(ctypes.Structure):
    _fields_ = [
        ("target_phase_rad", ctypes.c_float),
        ("target_rate_rad_s", ctypes.c_float),
        ("deceleration_rad_s2", ctypes.c_float),
        ("effort", ctypes.c_float),
        ("cadence_ema", ctypes.c_float),
        ("reposition", RepositionStateStruct),
        ("coasting", ctypes.c_uint8),
        ("has_target_phase", ctypes.c_uint8),
        ("has_cadence_ema", ctypes.c_uint8),
        ("pad", ctypes.c_uint8),
    ]


class AssistStateStruct(ctypes.Structure):
    _fields_ = [
        ("torque", ctypes.c_float),
        ("last_gain", ctypes.c_float),
        ("hold_power_w", ctypes.c_float),
        ("hold_torque_nm", ctypes.c_float),
        ("release_hold_ticks", ctypes.c_int32),
        ("pedaling", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class EfcScanResultStruct(ctypes.Structure):
    _fields_ = [
        ("hand_press_n", ctypes.c_float),
        ("left_hand_press_n", ctypes.c_float),
        ("right_hand_press_n", ctypes.c_float),
        ("pelvis_saddle_force_n", ctypes.c_float),
        ("foot_front_press_n", ctypes.c_float),
        ("foot_rear_press_n", ctypes.c_float),
        ("delivered_crank_torque_nm", ctypes.c_float),
        ("front_brake_friction_torque_nm", ctypes.c_float),
        ("rear_brake_friction_torque_nm", ctypes.c_float),
        ("freehub_tendon_force_n", ctypes.c_float),
        ("crank_clutch_tendon_force_n", ctypes.c_float),
        ("handlebar_load_n", ctypes.c_float),
        ("saddle_separating", ctypes.c_uint8),
        ("handlebar_in_contact", ctypes.c_uint8),
        ("contact_crash_cause", ctypes.c_uint8),
        ("has_grip_left", ctypes.c_uint8),
        ("has_grip_right", ctypes.c_uint8),
        ("has_saddle", ctypes.c_uint8),
        ("has_foot_front", ctypes.c_uint8),
        ("has_foot_rear", ctypes.c_uint8),
    ]


class BrakeResolutionCoreStruct(ctypes.Structure):
    _fields_ = [
        ("final_front", ctypes.c_float),
        ("final_rear", ctypes.c_float),
        ("legs_coast", ctypes.c_uint8),
        ("motor_braking", ctypes.c_uint8),
        ("shift_blocked", ctypes.c_uint8),
        ("riding", ctypes.c_uint8),
    ]


class CruiseStatePodStruct(ctypes.Structure):
    _fields_ = [
        ("target_speed_mps", ctypes.c_float),
        ("integral_mps_s", ctypes.c_float),
        ("torque_nm", ctypes.c_float),
        ("gain_scale", ctypes.c_float),
        ("engaged", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class StallRepositionStatePodStruct(ctypes.Structure):
    _fields_ = [
        ("crank_rad", ctypes.c_float),
        ("wheel_rad", ctypes.c_float),
        ("stall_elapsed_s", ctypes.c_float),
        ("goal_rad", ctypes.c_float),
        ("elapsed_s", ctypes.c_float),
        ("cooldown_s", ctypes.c_float),
        ("active", ctypes.c_uint8),
        ("armed", ctypes.c_uint8),
        ("reason", ctypes.c_uint8),
        ("has_stall_elapsed", ctypes.c_uint8),
        ("has_goal", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuEnergyLedgerStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::physics_core::GpuEnergyLedger<float>``."""

    _fields_ = [
        ("E_mech_initial", ctypes.c_float),
        ("E_mech", ctypes.c_float),
        ("E_scale_baseline", ctypes.c_float),
        ("E_scale_min", ctypes.c_float),
        ("W_muscle_signed", ctypes.c_float),
        ("W_muscle_pos", ctypes.c_float),
        ("W_motor_signed", ctypes.c_float),
        ("W_motor_pos", ctypes.c_float),
        ("W_cruise_signed", ctypes.c_float),
        ("W_cruise_pos", ctypes.c_float),
        ("W_external_signed", ctypes.c_float),
        ("D_tyre_radial", ctypes.c_float),
        ("D_tyre_brush_roll", ctypes.c_float),
        ("D_suspension", ctypes.c_float),
        ("D_resistance", ctypes.c_float),
        ("D_brake", ctypes.c_float),
        ("D_rider_end_stop", ctypes.c_float),
        ("W_constraint_signed", ctypes.c_float),
        ("W_constraint_abs", ctypes.c_float),
        ("W_closure_signed", ctypes.c_float),
        ("W_closure_abs", ctypes.c_float),
        ("source_j", ctypes.c_float),
        ("source_positive_j", ctypes.c_float),
        ("delta_E_mech", ctypes.c_float),
        ("net_supply", ctypes.c_float),
        ("residual_j", ctypes.c_float),
        ("energy_scale_j", ctypes.c_float),
        ("rel_residual", ctypes.c_float),
        ("max_rel_residual", ctypes.c_float),
        ("step_count", ctypes.c_int32),
        ("initialized", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuSp3WorldStateStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuSp3WorldState``."""

    _fields_ = [
        ("sp3", ctypes.c_uint8 * 12108),
        ("shifting", ShiftingStateStruct),
        ("pedaling", PedalingStateStruct),
        ("assist", AssistStateStruct),
        ("freehub", RatchetBoundStateStruct),
        ("crank_clutch", RatchetBoundStateStruct),
        ("last_efc", EfcScanResultStruct),
        ("last_brakes", BrakeResolutionCoreStruct),
        ("cruise", CruiseStatePodStruct),
        ("stall_reposition", StallRepositionStatePodStruct),
        ("energy_ledger", GpuEnergyLedgerStruct),
        ("step_index", ctypes.c_int32),
        ("last_wheel_load_n", ctypes.c_float * 2),
        ("last_wheel_slip_mps", ctypes.c_float * 2),
        ("last_wheel_eff_radius_m", ctypes.c_float * 2),
        ("last_front_load_share", ctypes.c_float),
        ("last_commanded_crank_torque_nm", ctypes.c_float),
        ("last_motor_torque_nm", ctypes.c_float),
        ("last_cruise_torque_nm", ctypes.c_float),
        ("last_front_hold_nm", ctypes.c_float),
        ("last_rear_hold_nm", ctypes.c_float),
        ("last_front_frictionloss_nm", ctypes.c_float),
        ("last_rear_frictionloss_nm", ctypes.c_float),
        ("last_step_u_elastic_pre_j", ctypes.c_float),
        ("last_step_d_tyre_radial_j", ctypes.c_float),
        ("last_step_d_tyre_brush_roll_j", ctypes.c_float),
        ("last_step_d_susp_j", ctypes.c_float),
        ("last_step_d_res_j", ctypes.c_float),
        ("last_pitch_rate_up_rad_s", ctypes.c_float),
        ("last_specific_force_body_mps2", ctypes.c_float * 3),
        ("cached_posture_grade", ctypes.c_float),
        ("cached_preview_grade", ctypes.c_float),
        ("cached_surface_front", ctypes.c_uint8),
        ("cached_surface_ahead", ctypes.c_uint8),
        ("has_cached_road_state", ctypes.c_uint8),
        ("reposition_consumed", ctypes.c_uint8),
        ("last_wheel_grounded", ctypes.c_uint8 * 2),
        ("last_has_front_load_share", ctypes.c_uint8),
        ("using_cassette_tendons", ctypes.c_uint8),
        ("frozen", ctypes.c_uint8),
        ("cached_surface_rear", ctypes.c_uint8),
        ("_tail_pad", ctypes.c_uint8 * 2),
    ]


class PressStateCoreStruct(ctypes.Structure):
    _fields_ = [
        ("f_passive", ctypes.c_float * 2),
        ("max_total_press_n", ctypes.c_float),
    ]


class SpindleM2K2StateStruct(ctypes.Structure):
    _fields_ = [
        ("coast_hold_nm", ctypes.c_float),
        ("held_torques", ctypes.c_float * 9),
        ("prev_ps", PressStateCoreStruct),
        ("enabled", ctypes.c_uint8),
        ("command_enabled", ctypes.c_uint8),
        ("has_held_torques", ctypes.c_uint8),
        ("has_prev_ps", ctypes.c_uint8),
    ]


class SpindleM2K2StepScratchStruct(ctypes.Structure):
    _fields_ = [
        ("frame_r", ctypes.c_float * 9),
        ("crank_world", ctypes.c_float * 3),
        ("frame_pitch_anchor", ctypes.c_float * 3),
        ("rider_pitch_anchor", ctypes.c_float * 3),
        ("hip_world", (ctypes.c_float * 3) * 2),
        ("delta_hip", (ctypes.c_float * 3) * 2),
        ("phase", ctypes.c_float),
        ("rate", ctypes.c_float),
        ("pelvis_pitch", ctypes.c_float),
        ("shares", ctypes.c_float * 2),
        ("jacobians", (ctypes.c_float * 2) * 2),
        ("djacs", (ctypes.c_float * 2) * 2),
        ("gammas", (ctypes.c_float * 2) * 2),
        ("gammas_x", (ctypes.c_float * 2) * 2),
        ("f_weight", ctypes.c_float * 2),
        ("qdot_leg_kin", (ctypes.c_float * 2) * 2),
        ("leg_cap_pos", (ctypes.c_float * 2) * 2),
        ("leg_cap_neg", (ctypes.c_float * 2) * 2),
        ("press_per_nm", ctypes.c_float * 2),
        ("upper_req", ctypes.c_float * 5),
        ("arm_gamma_sh", ctypes.c_float * 2),
        ("arm_gamma_el", ctypes.c_float * 2),
        ("arm_tau_sh", ctypes.c_float * 2),
        ("arm_tau_el", ctypes.c_float * 2),
        ("arm_valid", ctypes.c_uint8 * 2),
        ("has_jacobian", ctypes.c_uint8 * 2),
        ("incoming_coast_hold_nm", ctypes.c_float),
        ("mean_crank_torque_nm", ctypes.c_float),
        ("crank_target_rate_rad_s", ctypes.c_float),
        ("weld_crank_torque_nm", ctypes.c_float),
        ("has_weld_crank_torque", ctypes.c_uint8),
        ("command_enabled", ctypes.c_uint8),
        ("crank_reposition", ctypes.c_uint8),
        ("steady_state", ctypes.c_uint8),
        ("advance", ctypes.c_uint8),
        ("active_newton", ctypes.c_uint8),
        ("converged", ctypes.c_uint8),
        ("perturbed0", ctypes.c_uint8),
        ("perturbed1", ctypes.c_uint8),
        ("adaptive_mode", ctypes.c_uint8),
        ("torso_saturated", ctypes.c_uint8),
        ("arms_saturated", ctypes.c_uint8),
        ("pad_flags", ctypes.c_uint8 * 4),
        ("ps0", PressStateCoreStruct),
        ("P", PressStateCoreStruct),
        ("P0", PressStateCoreStruct),
        ("ps_trial", PressStateCoreStruct),
        ("base_leg_req", ctypes.c_float * 4),
        ("base_leg_prio", ctypes.c_float * 4),
        ("crank_task_nm", ctypes.c_float),
        ("last_req", ctypes.c_float * 9),
        ("last_exc", ctypes.c_float * 9),
    ]


class SpindleCommandStruct(ctypes.Structure):
    _fields_ = [
        ("mean_crank_torque_nm", ctypes.c_float),
        ("torso_lean_rad", ctypes.c_float),
        ("crank_target_rate_rad_s", ctypes.c_float),
        ("weld_crank_torque_nm", ctypes.c_float),
        ("has_weld_crank_torque", ctypes.c_uint8),
        ("enabled", ctypes.c_uint8),
        ("crank_reposition", ctypes.c_uint8),
        ("steady_state", ctypes.c_uint8),
        ("advance", ctypes.c_uint8),
        ("pad", ctypes.c_uint8 * 3),
    ]


class GpuSp4WorldStateStruct(ctypes.Structure):
    """Exact ctypes layout matching ``bike::gpu::GpuSp4WorldState``."""

    _fields_ = [
        ("spindle", SpindleM2K2StateStruct),
        ("scratch", SpindleM2K2StepScratchStruct),
        ("last_cmd", SpindleCommandStruct),
        ("saved_qacc_warmstart", ctypes.c_float * 32),
        ("last_posture_min_lean_rad", ctypes.c_float),
        ("min_press_front_n", ctypes.c_float),
        ("min_press_rear_n", ctypes.c_float),
        ("max_rising_front_excess_n", ctypes.c_float),
        ("max_rising_rear_excess_n", ctypes.c_float),
        ("min_saddle_force_n", ctypes.c_float),
        ("max_saddle_force_n", ctypes.c_float),
        ("sum_delivered_crank_torque_nm", ctypes.c_float),
        ("sum_commanded_crank_torque_nm", ctypes.c_float),
        ("start_root_x_m", ctypes.c_float),
        ("end_root_x_m", ctypes.c_float),
        ("end_speed_mps", ctypes.c_float),
        ("max_speed_mps", ctypes.c_float),
        ("energy_residual_j", ctypes.c_float),
        ("energy_scale_j", ctypes.c_float),
        ("max_rel_energy_residual", ctypes.c_float),
        ("sample_count", ctypes.c_int32),
        ("max_m0_passes_used", ctypes.c_int32),
        ("current_m0_passes", ctypes.c_int32),
        ("has_posture_min_lean", ctypes.c_uint8),
        ("has_start_root_x", ctypes.c_uint8),
        ("active_control_tick", ctypes.c_uint8),
        ("last_cmd_enabled", ctypes.c_uint8),
        ("energy_contract_ok", ctypes.c_uint8),
        ("solver_mode", ctypes.c_uint8),
        ("pad_sp4", ctypes.c_uint8 * 2),
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

    _assert_struct_size(lib, "bike_gpu_sizeof_world_config", GpuWorldConfigStruct)
    _assert_struct_size(lib, "bike_gpu_sizeof_world_state", GpuWorldStateStruct)
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
        _assert_struct_size(lib, "bike_gpu_sizeof_sp3_world_state", GpuSp3WorldStateStruct)

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

    if hasattr(lib, "bike_gpu_sizeof_sp4_world_state"):
        lib.bike_gpu_sizeof_sp4_world_state.argtypes = []
        lib.bike_gpu_sizeof_sp4_world_state.restype = ctypes.c_int
        _assert_struct_size(lib, "bike_gpu_sizeof_sp4_world_state", GpuSp4WorldStateStruct)

    if hasattr(lib, "bike_gpu_sizeof_spindle_params"):
        lib.bike_gpu_sizeof_spindle_params.argtypes = []
        lib.bike_gpu_sizeof_spindle_params.restype = ctypes.c_int
        spindle_bytes = int(lib.bike_gpu_sizeof_spindle_params())
        if spindle_bytes <= 0 or spindle_bytes > 65536:
            raise RuntimeError(f"Invalid SpindleM2K2Params size: {spindle_bytes} bytes")

    if hasattr(lib, "bike_gpu_launch_sp4_init"):
        lib.bike_gpu_launch_sp4_init.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.bike_gpu_launch_sp4_init.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_pre_solve_and_pass0"):
        lib.bike_gpu_launch_sp4_pre_solve_and_pass0.argtypes = [
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
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        lib.bike_gpu_launch_sp4_pre_solve_and_pass0.restype = ctypes.c_int

    sp4_pass_argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_void_p,
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
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    for fn_name in (
        "bike_gpu_launch_sp4_pass1",
        "bike_gpu_launch_sp4_pass2",
        "bike_gpu_launch_sp4_pass3",
    ):
        if hasattr(lib, fn_name):
            fn = getattr(lib, fn_name)
            fn.argtypes = sp4_pass_argtypes
            fn.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_post_solve"):
        lib.bike_gpu_launch_sp4_post_solve.argtypes = [
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
        lib.bike_gpu_launch_sp4_post_solve.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_sp3_state_has_failure"):
        lib.bike_gpu_sp3_state_has_failure.argtypes = [ctypes.c_void_p]
        lib.bike_gpu_sp3_state_has_failure.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_sp3_state_failure_flags"):
        lib.bike_gpu_sp3_state_failure_flags.argtypes = [ctypes.c_void_p]
        lib.bike_gpu_sp3_state_failure_flags.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_sizeof_pneumatic_scratch"):
        lib.bike_gpu_sizeof_pneumatic_scratch.argtypes = []
        lib.bike_gpu_sizeof_pneumatic_scratch.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_sizeof_spindle_m0_scratch"):
        lib.bike_gpu_sizeof_spindle_m0_scratch.argtypes = []
        lib.bike_gpu_sizeof_spindle_m0_scratch.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_m0_init"):
        lib.bike_gpu_launch_sp4_m0_init.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.bike_gpu_launch_sp4_m0_init.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_m0_pre_solve_and_begin"):
        lib.bike_gpu_launch_sp4_m0_pre_solve_and_begin.argtypes = [
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
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.bike_gpu_launch_sp4_m0_pre_solve_and_begin.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_m0_step"):
        lib.bike_gpu_launch_sp4_m0_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
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
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.bike_gpu_launch_sp4_m0_step.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_m0_post_solve"):
        lib.bike_gpu_launch_sp4_m0_post_solve.argtypes = [
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
            ctypes.c_void_p,
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
        lib.bike_gpu_launch_sp4_m0_post_solve.restype = ctypes.c_int

    if hasattr(lib, "bike_gpu_launch_sp4_energy_post_step"):
        lib.bike_gpu_launch_sp4_energy_post_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
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
            ctypes.c_int,
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
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.bike_gpu_launch_sp4_energy_post_step.restype = ctypes.c_int

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
        "-arch=native",
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


def _apply_backend_sag_defaults(
    arr: Any,
    nworld: int,
    backend: str,
    fork_psi_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
) -> None:
    """Apply backend-specific 30/30% sag suspension defaults across ``nworld`` configs."""
    if backend not in _BACKEND_DEFAULT_SUSPENSION_SAG or backend == "compliant_2d":
        return
    psi_def, rate_def = _BACKEND_DEFAULT_SUSPENSION_SAG[backend]
    for w in range(int(nworld)):
        psi_s = float(fork_psi_scale[w]) if fork_psi_scale is not None else 1.0
        rate_s = float(shock_rate_scale[w]) if shock_rate_scale is not None else 1.0
        arr[w].suspension.params.air_spring.gauge_pressure_psi = float(psi_def) * psi_s
        arr[w].suspension.params.coil.rate_n_m = float(rate_def) * rate_s
        recompute_suspension_preloads(arr[w].suspension)


def build_sweep_configs_bytes(
    lib: ctypes.CDLL,
    nworld: int,
    fork_psi_scale: np.ndarray | None = None,
    rebound_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
    tire_stiff_scale: np.ndarray | None = None,
    backend: str = "compliant_2d",
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
    if backend in _BACKEND_DEFAULT_SUSPENSION_SAG and backend != "compliant_2d":
        arr = (GpuWorldConfigStruct * int(nworld)).from_buffer(buf)
        _apply_backend_sag_defaults(
            arr,
            nworld,
            backend,
            fork_psi_scale=fork_psi_scale,
            shock_rate_scale=shock_rate_scale,
        )
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
    hbar_gid = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "geom_handlebar")
    if hbar_gid < 0:
        hbar_gid = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "handlebar")
    cfg.efc_scan.handlebar_geom_id = hbar_gid
    catch_gid = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "catch_plane")
    cfg.efc_scan.catch_geom_id = catch_gid
    t_cnt = 0
    terrain_gid = _name_id(mujoco.mjtObj.mjOBJ_GEOM, "terrain")
    if terrain_gid >= 0 and t_cnt < 4:
        cfg.efc_scan.terrain_geom_ids[t_cnt] = terrain_gid
        t_cnt += 1
    if catch_gid >= 0 and t_cnt < 4:
        cfg.efc_scan.terrain_geom_ids[t_cnt] = catch_gid
        t_cnt += 1
    cfg.efc_scan.terrain_geom_count = t_cnt

    r_cnt = 0
    for gid in range(int(m_mj.ngeom)):
        gname = mujoco.mj_id2name(m_mj, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if gname and gname.startswith("geom_rider_") and r_cnt < 16:
            cfg.efc_scan.rider_geom_ids[r_cnt] = int(gid)
            r_cnt += 1
    cfg.efc_scan.rider_geom_count = r_cnt

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

    # Populate rider joint envelope qpos/dof addresses and MuJoCo jnt_range limits
    # in canonical idx.rider_qpos order:
    #   [hip_front, knee_front, hip_rear, knee_rear, torso,
    #    shoulder_left, elbow_left, shoulder_right, elbow_right]
    rider_joint_names = (
        "rider_hip_front",
        "rider_knee_front",
        "rider_hip_rear",
        "rider_knee_rear",
        "rider_torso_hinge",
        "rider_shoulder_left",
        "rider_elbow_left",
        "rider_shoulder_right",
        "rider_elbow_right",
    )
    for slot in range(16):
        cfg.rider.envelope.qpos_adrs[slot] = -1
        cfg.rider.envelope.dof_adrs[slot] = -1
        cfg.rider.envelope.lower[slot] = 0.0
        cfg.rider.envelope.upper[slot] = 0.0
    for j, jname in enumerate(rider_joint_names):
        jid = _name_id(mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0 and int(m_mj.jnt_limited[jid]) != 0:
            cfg.rider.envelope.qpos_adrs[j] = int(m_mj.jnt_qposadr[jid])
            cfg.rider.envelope.dof_adrs[j] = int(m_mj.jnt_dofadr[jid])
            cfg.rider.envelope.lower[j] = float(m_mj.jnt_range[jid, 0])
            cfg.rider.envelope.upper[j] = float(m_mj.jnt_range[jid, 1])
    cfg.rider.envelope.count = len(rider_joint_names)
    return cfg


_SURFACE_LABEL_IDS: dict[str, int] = {
    "asphalt": 0,
    "hardpack": 1,
    "loose": 2,
    "wet": 3,
    "scree": 4,
}

_SURFACE_SPECS: dict[str, dict[str, float | int]] = {
    "asphalt": {
        "mu_peak": 1.05,
        "mu_slide": 0.75,
        "slip_stiffness_per_load": 15.0,
        "stribeck_speed_mps": 4.5,
        "crr_delta": 0.0,
        "load_sensitivity": 0.06,
        "surface_id": 0,
    },
    "hardpack": {
        "mu_peak": 0.80,
        "mu_slide": 0.60,
        "slip_stiffness_per_load": 12.0,
        "stribeck_speed_mps": 4.5,
        "crr_delta": 0.0025,
        "load_sensitivity": 0.08,
        "surface_id": 1,
    },
    "loose": {
        "mu_peak": 0.55,
        "mu_slide": 0.45,
        "slip_stiffness_per_load": 7.0,
        "stribeck_speed_mps": 4.5,
        "crr_delta": 0.0060,
        "load_sensitivity": 0.12,
        "surface_id": 2,
    },
    "wet": {
        "mu_peak": 0.50,
        "mu_slide": 0.40,
        "slip_stiffness_per_load": 10.0,
        "stribeck_speed_mps": 4.5,
        "crr_delta": 0.0035,
        "load_sensitivity": 0.12,
        "surface_id": 3,
    },
    "scree": {
        "mu_peak": 0.45,
        "mu_slide": 0.38,
        "slip_stiffness_per_load": 4.5,
        "stribeck_speed_mps": 4.5,
        "crr_delta": 0.0080,
        "load_sensitivity": 0.15,
        "surface_id": 4,
    },
}

_BACKEND_DEFAULT_SUSPENSION_SAG: dict[str, tuple[float, float]] = {
    "compliant_2d": (97.0, 81550.0),
    "distributed_2d_reference": (90.8, 84372.0),
    "pneumatic": (113.8, 73557.0),
}

DEFAULT_EFFORT_CAP_GRADE_NM: tuple[float, ...] = (
    36.0,
    15.0,
    14.5,
    13.2,
    14.0,
    15.5,
    19.5,
    18.5,
    21.0,
    20.0,
)


def _as_seq(val: Any) -> list[Any] | None:
    if isinstance(val, dict) and "$tuple" in val:
        inner = val["$tuple"]
        return list(inner) if isinstance(inner, (list, tuple)) else None
    if isinstance(val, (list, tuple)):
        return list(val)
    return None


def _surface_id_from_value(val: Any, default: int = 1) -> int:
    if isinstance(val, (int, np.integer)):
        return int(val)
    if isinstance(val, str):
        key = val.strip().lower()
        if key in _SURFACE_LABEL_IDS:
            return _SURFACE_LABEL_IDS[key]
    return int(default)


def recompute_suspension_preloads(susp: GpuSuspensionParamsStruct) -> None:
    """Recompute static sag preloads matching ``bike::gpu::gpu_init_suspension_preloads``."""
    import math

    air = susp.params.air_spring
    coil = susp.params.coil
    fork_sag_m = (float(air.total_travel_mm) * 1e-3) * float(susp.fork_sag_fraction)
    shock_sag_m = (float(coil.stroke_mm) * 1e-3) * float(susp.shock_sag_fraction)

    r_m = (float(air.stanchion_inner_diam_mm) * 0.5) * 1e-3
    area = math.pi * r_m * r_m
    tokens = max(0, min(int(air.num_tokens), int(air.max_tokens)))
    v_pos_0 = area * (float(air.pos_chamber_length_mm) * 1e-3) - float(tokens) * (float(air.token_volume_cm3) * 1e-6)
    v_neg_0 = area * (float(air.neg_chamber_length_mm) * 1e-3)
    v_disp = area * max(0.0, min(float(air.total_travel_mm) * 1e-3, fork_sag_m))
    if v_pos_0 > 0.0 and v_neg_0 > 0.0 and v_disp < v_pos_0:
        p_abs_0 = float(air.gauge_pressure_psi) * 6894.757293168 + float(air.atm_pressure_pa)
        v_pos = v_pos_0 - v_disp
        v_neg = v_neg_0 + v_disp
        gamma = float(air.gamma)
        susp.fork_preload_n = float(area * (p_abs_0 * ((v_pos_0 / v_pos) ** gamma) - p_abs_0 * ((v_neg_0 / v_neg) ** gamma)))
    else:
        susp.fork_preload_n = 0.0

    stroke_m = float(coil.stroke_mm) * 1e-3
    if 0.0 <= shock_sag_m <= stroke_m:
        susp.shock_preload_n = float(float(coil.rate_n_m) * (float(coil.preload_mm) * 1e-3 + shock_sag_m))
    else:
        susp.shock_preload_n = 0.0


def apply_manifest_json_to_sp3_config(
    cfg: GpuSp3ConfigStruct,
    manifest: dict[str, Any],
) -> None:
    """Apply ``manifest.json`` (``resolved_bike``, ``track_spec``, ``ride_program``) onto ``GpuSp3ConfigStruct``."""
    rb = manifest.get("resolved_bike")
    if isinstance(rb, dict):
        if "fork_initial_psi" in rb and rb["fork_initial_psi"] is not None:
            cfg.suspension.params.air_spring.gauge_pressure_psi = float(rb["fork_initial_psi"])
        if "shock_stiffness" in rb and rb["shock_stiffness"] is not None:
            cfg.suspension.params.coil.rate_n_m = float(rb["shock_stiffness"])
        recompute_suspension_preloads(cfg.suspension)

    ts = manifest.get("track_spec")
    if isinstance(ts, dict):
        if "surface" in ts and ts["surface"] is not None:
            cfg.default_surface_label = _surface_id_from_value(
                ts["surface"], int(cfg.default_surface_label)
            )
        sections = _as_seq(ts.get("surface_sections")) or _as_seq(ts.get("sections"))
        if sections is not None:
            cnt = min(len(sections), 16)
            cfg.surface_section_count = cnt
            for i in range(cnt):
                sec = sections[i]
                if not isinstance(sec, dict):
                    continue
                cfg.surface_sections[i].start_m = float(sec.get("start_m", 0.0))
                cfg.surface_sections[i].end_m = float(sec.get("end_m", 0.0))
                cfg.surface_sections[i].label = _surface_id_from_value(
                    sec.get("surface", sec.get("label", cfg.default_surface_label)),
                    int(cfg.default_surface_label),
                )

    rp = manifest.get("ride_program")
    if isinstance(rp, dict):
        _apply_ride_program_dict_to_sp3_config(cfg, rp)


def _apply_ride_program_dict_to_sp3_config(
    cfg: GpuSp3ConfigStruct,
    rp: dict[str, Any],
) -> None:
    segs = _as_seq(rp.get("segments"))
    if segs is None:
        return
    cnt = min(len(segs), 32)
    cfg.ride_program.segment_count = cnt
    cfg.ride_program.reaction_delay_s = float(rp.get("reaction_delay_s", 0.0))
    for i in range(cnt):
        seg = segs[i]
        if not isinstance(seg, dict):
            continue
        dst = cfg.ride_program.segments[i]
        dst.t_end_s = float(seg.get("t_end_s", 0.0))
        mask = int(seg.get("channel_mask", 0))
        if mask == 0:
            if seg.get("human_torque_nm") is not None:
                mask |= 1 << 0
            if seg.get("motor_torque_nm") is not None:
                mask |= 1 << 1
            if seg.get("motor_limit_nm") is not None:
                mask |= 1 << 2
            if seg.get("crank_rate_rad_s") is not None:
                mask |= 1 << 3
            if seg.get("front_brake") is not None:
                mask |= 1 << 4
            if seg.get("rear_brake") is not None:
                mask |= 1 << 5
            if seg.get("cruise_speed_kmh") is not None:
                mask |= 1 << 6
            if seg.get("posture_lean_rad") is not None:
                mask |= 1 << 7
            if seg.get("linear_ramp"):
                mask |= 1 << 8
        dst.channel_mask = mask
        dst.human_torque_nm = float(seg.get("human_torque_nm") or 0.0)
        dst.motor_torque_nm = float(seg.get("motor_torque_nm") or 0.0)
        dst.motor_limit_nm = float(seg.get("motor_limit_nm") or 0.0)
        dst.crank_rate_rad_s = float(seg.get("crank_rate_rad_s") or 0.0)
        dst.front_brake = float(seg.get("front_brake") or 0.0)
        dst.rear_brake = float(seg.get("rear_brake") or 0.0)
        dst.cruise_speed_kmh = float(seg.get("cruise_speed_kmh") or 0.0)
        dst.posture_lean_rad = float(seg.get("posture_lean_rad") or 0.0)
    cfg.program_enabled = 1 if bool(rp.get("enabled", cnt > 0)) and cnt > 0 else 0


def apply_physics_config_json_to_sp3_config(
    cfg: GpuSp3ConfigStruct,
    phys_cfg: dict[str, Any],
    manifest: dict[str, Any] | None = None,
) -> None:
    """Apply exported ``physics_config.json`` (and optional ``manifest.json``) onto ``GpuSp3ConfigStruct``."""
    dt_s = float(phys_cfg.get("timestep_s", 0.00125))
    if dt_s <= 0.0:
        dt_s = 0.00125

    def _ticks(sec: float, step_s: float = dt_s) -> int:
        return max(0, int(round(float(sec) / float(step_s))))

    if "control_period_s" in phys_cfg and float(phys_cfg["control_period_s"]) > 0.0:
        cfg.control_period_steps = max(1, _ticks(float(phys_cfg["control_period_s"])))

    res = phys_cfg.get("resistance")
    if isinstance(res, dict):
        if "crr" in res:
            cfg.resistance.crr = float(res["crr"])
        if "rolling_taper_rad_s" in res:
            cfg.resistance.rolling_taper_rad_s = float(res["rolling_taper_rad_s"])
        if "rho_kg_m3" in res:
            cfg.resistance.rho_kg_m3 = float(res["rho_kg_m3"])
        if "cda_m2" in res:
            cfg.resistance.cda_m2 = float(res["cda_m2"])
        wind = _as_seq(res.get("wind_world_mps"))
        if wind is not None and len(wind) == 3:
            for i in range(3):
                cfg.resistance.wind_world_mps[i] = float(wind[i])
        pt = _as_seq(res.get("point_body_m"))
        if pt is not None and len(pt) == 3:
            for i in range(3):
                cfg.resistance.point_body_m[i] = float(pt[i])

    tires = phys_cfg.get("tires")
    if not isinstance(tires, dict):
        tires = phys_cfg.get("tire")
    backend_name = str(tires.get("backend", "compliant_2d")) if isinstance(tires, dict) else "compliant_2d"
    if backend_name in ("distributed_2d_reference", "pneumatic"):
        cfg.resistance.crr = 0.0
    if backend_name in _BACKEND_DEFAULT_SUSPENSION_SAG:
        psi_def, rate_def = _BACKEND_DEFAULT_SUSPENSION_SAG[backend_name]
        cfg.suspension.params.air_spring.gauge_pressure_psi = float(psi_def)
        cfg.suspension.params.coil.rate_n_m = float(rate_def)
    if isinstance(tires, dict):
        pneu = tires.get("pneumatic")
        if isinstance(pneu, dict) and pneu.get("surface") is not None:
            cfg.default_surface_label = _surface_id_from_value(
                pneu["surface"], int(cfg.default_surface_label)
            )

    end_stops = phys_cfg.get("end_stops")
    if isinstance(end_stops, dict):
        ref_f = float(end_stops.get("reference_force_n", 0.0))
        ref_d = float(end_stops.get("reference_deflection_m", 0.0))
        if ref_f > 0.0 and ref_d > 0.0:
            cfg.suspension.params.end_stops.stiffness_n_m = float(ref_f / ref_d)
        if "damping_n_s_m" in end_stops:
            cfg.suspension.params.end_stops.damping_n_s_m = float(end_stops["damping_n_s_m"])

    susp = phys_cfg.get("suspension")
    if isinstance(susp, dict):
        air = susp.get("air_spring")
        if isinstance(air, dict):
            for k in (
                "stanchion_inner_diam_mm",
                "total_travel_mm",
                "pos_chamber_length_mm",
                "neg_chamber_length_mm",
                "token_volume_cm3",
                "gamma",
                "atm_pressure_pa",
                "gauge_pressure_psi",
            ):
                if k in air:
                    setattr(cfg.suspension.params.air_spring, k, float(air[k]))
            for k in ("max_tokens", "num_tokens"):
                if k in air:
                    setattr(cfg.suspension.params.air_spring, k, int(air[k]))
        coil = susp.get("coil")
        if isinstance(coil, dict):
            for k in ("rate_n_m", "preload_mm", "stroke_mm", "bumper_length_mm", "bumper_peak_n"):
                if k in coil:
                    setattr(cfg.suspension.params.coil, k, float(coil[k]))
        if "fork_sag_fraction" in susp:
            cfg.suspension.fork_sag_fraction = float(susp["fork_sag_fraction"])
        if "shock_sag_fraction" in susp:
            cfg.suspension.shock_sag_fraction = float(susp["shock_sag_fraction"])
    recompute_suspension_preloads(cfg.suspension)

    drive = phys_cfg.get("drive")
    if isinstance(drive, dict):
        if "brake_ceiling_nm" in drive:
            cfg.static_brake_ceiling_nm = float(drive["brake_ceiling_nm"])
        if "brake_hold_torque_nm" in drive:
            cfg.brake_hold.torque_nm = float(drive["brake_hold_torque_nm"])
        if "brake_hold_stiffness_nm_rad" in drive:
            cfg.brake_hold.stiffness_nm_rad = float(drive["brake_hold_stiffness_nm_rad"])
        if "brake_hold_damping_nms_rad" in drive:
            cfg.brake_hold.damping_nms_rad = float(drive["brake_hold_damping_nms_rad"])
        if "brake_hold_engage_radps" in drive:
            cfg.brake_hold.engage_radps = float(drive["brake_hold_engage_radps"])
        if "brake_hold_release_radps" in drive:
            cfg.brake_hold.release_radps = float(drive["brake_hold_release_radps"])
        if "torque_ripple" in drive:
            cfg.crank_torque_ripple = float(drive["torque_ripple"])

        gearing = drive.get("gearing")
        if isinstance(gearing, dict):
            if "front_teeth" in gearing:
                cfg.shifting.front_teeth = int(gearing["front_teeth"])
            if "rear_teeth" in gearing:
                cfg.shifting.initial_rear_teeth = int(gearing["rear_teeth"])

        sh = drive.get("shifting")
        if isinstance(sh, dict):
            if "enabled" in sh:
                cfg.shifting.enabled = 1 if bool(sh["enabled"]) else 0
            cass = _as_seq(sh.get("cassette"))
            if cass is not None and 1 <= len(cass) <= 12:
                cfg.shifting.cog_count = len(cass)
                for i, t in enumerate(cass):
                    cfg.shifting.cassette[i] = int(t)
            if "target_cadence_min_rpm" in sh:
                cfg.shifting.target_cadence_min_rpm = float(sh["target_cadence_min_rpm"])
            if "target_cadence_max_rpm" in sh:
                cfg.shifting.target_cadence_max_rpm = float(sh["target_cadence_max_rpm"])
            if "shift_cooldown_s" in sh and float(sh["shift_cooldown_s"]) >= 0.0:
                cfg.shifting.shift_cooldown_ticks = _ticks(float(sh["shift_cooldown_s"]))
            if "shift_cut_duration_s" in sh and float(sh["shift_cut_duration_s"]) >= 0.0:
                cfg.shifting.shift_cut_duration_ticks = max(
                    1, _ticks(float(sh["shift_cut_duration_s"]))
                )
            if "torque_factor_larger_cog" in sh:
                cfg.shifting.torque_factor_larger_cog = float(sh["torque_factor_larger_cog"])
            if "torque_factor_smaller_cog" in sh:
                cfg.shifting.torque_factor_smaller_cog = float(sh["torque_factor_smaller_cog"])
            if "cadence_smoothing_tau_s" in sh:
                cfg.shifting.cadence_smoothing_tau_s = float(sh["cadence_smoothing_tau_s"])
            if "upshift_slip_limit_mps" in sh:
                cfg.shifting.upshift_slip_limit_mps = float(sh["upshift_slip_limit_mps"])
            if "upshift_slip_mode" in sh:
                mode_s = str(sh["upshift_slip_mode"]).strip().lower()
                cfg.shifting.upshift_slip_mode = 1 if mode_s == "magnitude" else 0

        assist = drive.get("assist")
        if isinstance(assist, dict):
            if "engage_torque_nm" in assist:
                cfg.assist.engage_torque_nm = float(assist["engage_torque_nm"])
            if "release_hold_s" in assist and float(assist["release_hold_s"]) >= 0.0:
                cfg.assist.release_hold_ticks = _ticks(float(assist["release_hold_s"]))
            prof = assist.get("profile")
            prof_key = prof.strip().lower() if isinstance(prof, str) else ""
            if prof_key in ("bosch_cx_gen4", "bosch_performance_line_cx_gen4"):
                pmap = {"eco": 0, "tour": 1, "emtb": 2, "turbo": 3}
                mode_key = str(assist.get("mode", "turbo")).strip().lower()
                pmode = pmap.get(mode_key, 3)
                cfg.assist.profile_mode = pmode
                cfg.assist.has_profile = 1
                cfg.assist.profile.eco = 0.6
                cfg.assist.profile.tour = 1.4
                cfg.assist.profile.emtb_low = 1.4
                cfg.assist.profile.emtb_high = 3.4
                cfg.assist.profile.turbo = 3.4
                cfg.assist.profile.emtb_full_gain_at_nm = 40.0
                cfg.assist.gain = {0: 0.6, 1: 1.4, 2: 3.4, 3: 3.4}[pmode]
                cfg.assist.max_torque = 85.0
                cfg.assist.max_power = 600.0
                cfg.assist.tau = 0.04
                cfg.assist.slew = 85.0 / 0.04
                cfg.assist.cutoff_mps = 25.0 / 3.6
                cfg.assist.taper_width_mps = 2.0 / 3.6
            else:
                if prof is None and "profile" in assist:
                    cfg.assist.has_profile = 0
                elif prof_key in ("eco", "tour", "emtb", "turbo"):
                    pmap = {"eco": 0, "tour": 1, "emtb": 2, "turbo": 3}
                    cfg.assist.profile_mode = pmap[prof_key]
                    cfg.assist.has_profile = 1
                for k in (
                    "gain",
                    "max_torque",
                    "max_power",
                    "tau",
                    "slew",
                    "cutoff_mps",
                    "taper_width_mps",
                ):
                    if k in assist:
                        setattr(cfg.assist, k, float(assist[k]))
            tc = _as_seq(assist.get("torque_curve"))
            if tc is not None and len(tc) > 0:
                cnt = min(len(tc), 8)
                cfg.assist.curve_len = cnt
                for i in range(cnt):
                    pair = _as_seq(tc[i]) or tc[i]
                    cfg.assist.curve_rpm[i] = float(pair[0])
                    cfg.assist.curve_nm[i] = float(pair[1])

        ped = drive.get("pedaling")
        if isinstance(ped, dict):
            if "enabled" in ped:
                cfg.pedaling.enabled = 1 if bool(ped["enabled"]) else 0
            for k in (
                "coast_above_rpm",
                "resume_below_rpm",
                "stop_time_s",
                "coast_cadence_tau_s",
                "mash_cadence_rpm",
                "mash_torque_nm",
                "effort_slew_nm_s",
                "reposition_min_effort_nm",
                "reposition_stall_progress_rad",
                "reposition_back_rate_rad_s",
                "reposition_phase_tolerance_rad",
                "reposition_noop_rad",
            ):
                if k in ped and float(ped[k]) >= 0.0:
                    setattr(cfg.pedaling, k, float(ped[k]))
            if "reposition_on_stall" in ped:
                cfg.pedaling.reposition_on_stall = 1 if bool(ped["reposition_on_stall"]) else 0
            if "reposition_stall_window_s" in ped and float(ped["reposition_stall_window_s"]) >= 0.0:
                cfg.pedaling.reposition_stall_window_ticks = _ticks(
                    float(ped["reposition_stall_window_s"])
                )
            if "reposition_cooldown_s" in ped and float(ped["reposition_cooldown_s"]) >= 0.0:
                cfg.pedaling.reposition_cooldown_ticks = _ticks(
                    float(ped["reposition_cooldown_s"])
                )
            if "reposition_timeout_s" in ped and float(ped["reposition_timeout_s"]) >= 0.0:
                cfg.pedaling.reposition_timeout_ticks = _ticks(
                    float(ped["reposition_timeout_s"])
                )
            if "rollback_brake" in ped:
                cfg.rollback.rollback_brake_enabled = 1 if bool(ped["rollback_brake"]) else 0
            if "rollback_demand" in ped:
                cfg.rollback.rollback_demand = float(ped["rollback_demand"])
            if "rollback_engage_mps" in ped:
                cfg.rollback.rollback_engage_mps = float(ped["rollback_engage_mps"])
            if "rollback_release_mps" in ped:
                cfg.rollback.rollback_release_mps = float(ped["rollback_release_mps"])

    art = phys_cfg.get("articulated")
    if isinstance(art, dict):
        if "pedal_torque_ripple" in art:
            cfg.crank_torque_ripple = float(art["pedal_torque_ripple"])
        if "joint_envelope_soft_k_nm_rad" in art:
            cfg.rider.envelope.stiffness_nm_rad = float(art["joint_envelope_soft_k_nm_rad"])
        if "joint_envelope_soft_margin_rad" in art:
            cfg.rider.envelope.margin_rad = float(art["joint_envelope_soft_margin_rad"])
        if "road_lookahead_m" in art:
            cfg.road_lookahead_m = float(art["road_lookahead_m"])
        if "balance_floor_kmh" in art:
            cfg.balance.floor_mps = float(art["balance_floor_kmh"]) / 3.6
        if "balance_dwell_s" in art:
            cfg.balance.dwell_ticks = _ticks(float(art["balance_dwell_s"]))
        if "balance_grace_s" in art:
            cfg.balance.grace_ticks = _ticks(float(art["balance_grace_s"]))
        cfg.balance.period_s = dt_s

    sc = phys_cfg.get("seated_climb")
    if isinstance(sc, dict):
        if "enabled" in sc:
            cfg.intent.enabled = 1 if bool(sc["enabled"]) else 0
        period_s = float(sc.get("period_s", cfg.intent.period_s))
        if period_s > 0.0:
            cfg.intent.period_s = period_s
            cfg.intent.period_steps = max(1, _ticks(period_s))
            if "reaction_delay_s" in sc:
                cfg.intent.delay_ticks = _ticks(float(sc["reaction_delay_s"]), period_s)
            if "trim_dead_time_s" in sc:
                cfg.intent.dead_time_ticks = _ticks(float(sc["trim_dead_time_s"]), period_s)
            if "surge_budget_s" in sc:
                cfg.intent.surge_max_subticks = int(round(float(sc["surge_budget_s"]) / period_s * 600.0))
        if "surge_recovery_rate" in sc:
            cfg.intent.surge_recover_subticks = int(round(float(sc["surge_recovery_rate"]) * 600.0))
        for k in (
            "target_crank_power_w",
            "max_crank_torque_nm",
            "lean_gain",
            "max_forward_lean_rad",
            "max_backward_lean_rad",
            "lean_rate_rad_s",
            "orientation_tau_s",
            "surge_power_w",
            "surge_grade",
            "front_load_share_target",
            "lean_trim_gain_rad_s",
            "lean_trim_limit_rad",
            "min_elbow_flexion_deg",
            "climb_effort_rise_speed_mps",
            "climb_effort_full_speed_mps",
        ):
            if k in sc:
                setattr(cfg.intent, k, float(sc[k]))
        c_grades = _as_seq(sc.get("climb_effort_cap_grades"))
        c_vals = _as_seq(sc.get("climb_effort_cap_grade_nm"))
        if c_grades is not None and c_vals is not None and len(c_grades) == len(c_vals):
            cnt = min(len(c_grades), 16)
            cfg.intent.climb_effort_cap_count = cnt
            for i in range(cnt):
                cfg.intent.climb_effort_cap_grades[i] = float(c_grades[i])
                cfg.intent.climb_effort_cap_grade_nm[i] = float(c_vals[i])

    rl = phys_cfg.get("rider_launch")
    if isinstance(rl, dict):
        if "plan_enabled" in rl:
            cfg.launch.plan_enabled = 1 if bool(rl["plan_enabled"]) else 0
        for k in (
            "start_pedal_angle_rad",
            "standstill_speed_mps",
            "standstill_crank_rpm",
            "effort_rate_nm_s",
            "effort_cap_asphalt_nm",
            "effort_cap_hardpack_nm",
            "effort_cap_loose_nm",
            "effort_cap_wet_nm",
            "effort_cap_scree_nm",
            "effort_rise_speed_mps",
            "upshift_crank_rpm",
            "end_speed_mps",
            "end_crank_rpm",
        ):
            if k in rl:
                setattr(cfg.launch, k, float(rl[k]))
        cfg.launch.period_s = dt_s
        if "brake_release_s" in rl:
            cfg.launch.release_ticks = _ticks(float(rl["brake_release_s"]))
        if "max_hold_s" in rl:
            cfg.launch.max_hold_ticks = _ticks(float(rl["max_hold_s"]))
        if "end_hold_s" in rl:
            cfg.launch.end_hold_ticks = _ticks(float(rl["end_hold_s"]))
        if "max_launch_s" in rl:
            cfg.launch.max_launch_ticks = _ticks(float(rl["max_launch_s"]))
        if "handover_s" in rl:
            cfg.launch.handover_ticks = _ticks(float(rl["handover_s"]))
        if "shift_retry_s" in rl:
            cfg.launch.retry_ticks = _ticks(float(rl["shift_retry_s"]))
        cog_b = _as_seq(rl.get("start_cog_grade_breaks"))
        cog_t = _as_seq(rl.get("start_cog_teeth"))
        if (
            cog_b is not None
            and cog_t is not None
            and len(cog_b) + 1 == len(cog_t)
            and 1 <= len(cog_t) <= 16
        ):
            cfg.launch.start_cog_count = len(cog_t)
            for i in range(len(cog_b)):
                cfg.launch.start_cog_grade_breaks[i] = float(cog_b[i])
            for i in range(len(cog_t)):
                cfg.launch.start_cog_teeth[i] = int(cog_t[i])
        lean_g = _as_seq(rl.get("start_lean_grades"))
        lean_r = _as_seq(rl.get("start_lean_rad"))
        if lean_g is not None and lean_r is not None and len(lean_g) == len(lean_r):
            cnt = min(len(lean_g), 16)
            cfg.launch.start_lean_count = cnt
            for i in range(cnt):
                cfg.launch.start_lean_grades[i] = float(lean_g[i])
                cfg.launch.start_lean_rad[i] = float(lean_r[i])
        ec_g = _as_seq(rl.get("effort_cap_grades"))
        ec_v = _as_seq(rl.get("effort_cap_grade_nm"))
        if ec_g is not None and ec_v is not None and len(ec_g) == len(ec_v):
            cnt = min(len(ec_g), 16)
            cfg.launch.effort_cap_count = cnt
            for i in range(cnt):
                cfg.launch.effort_cap_grades[i] = float(ec_g[i])
                cfg.launch.effort_cap_grade_nm[i] = float(ec_v[i])
        rt_g = _as_seq(rl.get("release_torque_grades"))
        rt_v = _as_seq(rl.get("release_torque_nm"))
        if rt_g is not None and rt_v is not None and len(rt_g) == len(rt_v):
            cnt = min(len(rt_g), 16)
            cfg.launch.release_torque_count = cnt
            for i in range(cnt):
                cfg.launch.release_torque_grades[i] = float(rt_g[i])
                cfg.launch.release_torque_nm[i] = float(rt_v[i])
    if cfg.launch.effort_cap_count == 0:
        cfg.launch.effort_cap_count = len(DEFAULT_EFFORT_CAP_GRADE_NM)
        for i, val in enumerate(DEFAULT_EFFORT_CAP_GRADE_NM):
            cfg.launch.effort_cap_grade_nm[i] = float(val)

    fl = phys_cfg.get("front_lift_reaction")
    if isinstance(fl, dict):
        fl_period = float(cfg.intent.period_s if cfg.intent.period_s > 0.0 else dt_s)
        cfg.front_lift.period_s = fl_period
        for k in (
            "hand_drop_ratio",
            "pitch_rate_up_rad_s",
            "pitch_rad",
            "ease_fraction",
            "ease_rate_per_s",
            "lean_add_rad",
            "lean_rate_rad_s",
        ):
            if k in fl:
                setattr(cfg.front_lift, k, float(fl[k]))
        if "hand_window_s" in fl:
            cfg.front_lift.window_ticks = _ticks(float(fl["hand_window_s"]), fl_period)
        if "hand_lag_s" in fl:
            cfg.front_lift.lag_ticks = _ticks(float(fl["hand_lag_s"]), fl_period)
        if "decision_s" in fl:
            cfg.front_lift.decision_ticks = _ticks(float(fl["decision_s"]), fl_period)
        if "clear_s" in fl:
            cfg.front_lift.clear_ticks = _ticks(float(fl["clear_s"]), fl_period)
        if "ease_enabled" in fl:
            cfg.front_lift.ease_enabled = 1 if bool(fl["ease_enabled"]) else 0
        if "lean_enabled" in fl:
            cfg.front_lift.lean_enabled = 1 if bool(fl["lean_enabled"]) else 0

    mon = phys_cfg.get("monitors")
    if isinstance(mon, dict):
        if "grounded_hold_s" in mon:
            cfg.grounded_filter.hold_ticks = _ticks(float(mon["grounded_hold_s"]))
        if "contact_dropout_s" in mon:
            cfg.bridged_load.dropout_steps = _ticks(float(mon["contact_dropout_s"]))

    cruise = phys_cfg.get("cruise")
    if isinstance(cruise, dict):
        for k in ("target_speed_kmh", "kp_nm_per_mps", "ki_nm_per_mps_s", "torque_ceiling_nm"):
            if k in cruise:
                setattr(cfg.cruise, k, float(cruise[k]))
        if "enabled" in cruise:
            cfg.cruise_enabled = 1 if bool(cruise["enabled"]) else 0

    rp = phys_cfg.get("ride_program")
    if isinstance(rp, dict):
        _apply_ride_program_dict_to_sp3_config(cfg, rp)

    if isinstance(manifest, dict):
        apply_manifest_json_to_sp3_config(cfg, manifest)


def load_bundle_sp3_config(
    lib: ctypes.CDLL,
    m_mj: Any,
    bundle_dir: Path | str | None = None,
    phys_cfg: dict[str, Any] | None = None,
    manifest: dict[str, Any] | None = None,
) -> GpuSp3ConfigStruct:
    """Build a ``GpuSp3ConfigStruct`` from ``m_mj`` plus bundle ``physics_config.json`` & ``manifest.json``."""
    cfg = build_sp3_config_struct_from_mjmodel(lib, m_mj)
    bdir = Path(bundle_dir) if bundle_dir is not None else None
    if phys_cfg is None and bdir is not None:
        pc_path = bdir / "physics_config.json"
        if pc_path.is_file():
            phys_cfg = json.loads(pc_path.read_text(encoding="utf-8"))
    if manifest is None and bdir is not None:
        mf_path = bdir / "manifest.json"
        if mf_path.is_file():
            manifest = json.loads(mf_path.read_text(encoding="utf-8"))
    if isinstance(phys_cfg, dict):
        apply_physics_config_json_to_sp3_config(cfg, phys_cfg, manifest=manifest)
    elif isinstance(manifest, dict):
        apply_manifest_json_to_sp3_config(cfg, manifest)
    return cfg


def build_sp3_sweep_configs_bytes(
    lib: ctypes.CDLL,
    nworld: int,
    m_mj: Any = None,
    fork_psi_scale: np.ndarray | None = None,
    rebound_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
    tire_stiff_scale: np.ndarray | None = None,
    backend: str = "compliant_2d",
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

    need_arr = m_mj is not None or (backend in _BACKEND_DEFAULT_SUSPENSION_SAG and backend != "compliant_2d")
    if need_arr:
        arr_type = GpuSp3ConfigStruct * int(nworld)
        arr = arr_type.from_buffer(buf)
        _apply_backend_sag_defaults(
            arr,
            nworld,
            backend,
            fork_psi_scale=fork_psi_scale,
            shock_rate_scale=shock_rate_scale,
        )
        if m_mj is not None:
            base_resolved = build_sp3_config_struct_from_mjmodel(lib, m_mj)
            for w in range(int(nworld)):
                arr[w].efc_scan = base_resolved.efc_scan
                arr[w].rider.envelope = base_resolved.rider.envelope
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


def configure_host_tyre_for_surface_builder(lib: ctypes.CDLL, symbol_name: str) -> Any:
    """Bind ``bike_{cpu,metal}_build_tyre_and_terrain_for_surface`` argtypes and restype."""
    fn = getattr(lib, symbol_name)
    fn.argtypes = [
        ctypes.c_char_p,
        ctypes.c_int,
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
    surface_label: int = -2,
) -> tuple[bytes, bytes, np.ndarray, np.ndarray]:
    """Resolve ``(tyre_model_bytes, terrain_desc_bytes, terrain_f32, terrain_i32)`` via host C++."""
    use_surface = int(surface_label) != -2
    builder = None
    if use_surface:
        if hasattr(host_lib, "bike_cpu_build_tyre_and_terrain_for_surface"):
            builder = configure_host_tyre_for_surface_builder(
                host_lib, "bike_cpu_build_tyre_and_terrain_for_surface"
            )
        elif hasattr(host_lib, "bike_metal_build_tyre_and_terrain_for_surface"):
            builder = configure_host_tyre_for_surface_builder(
                host_lib, "bike_metal_build_tyre_and_terrain_for_surface"
            )
        else:
            raise AttributeError(
                "host_lib exposes neither bike_cpu_build_tyre_and_terrain_for_surface "
                "nor bike_metal_build_tyre_and_terrain_for_surface"
            )
    else:
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
    elif backend == "pneumatic":
        backend_kind = 2
    else:
        raise ValueError(f"Unknown tyre backend '{backend}'")

    model_buf = ctypes.create_string_buffer(int(host_lib.bike_gpu_sizeof_tyre_model()))
    desc_buf = ctypes.create_string_buffer(int(host_lib.bike_gpu_sizeof_terrain_desc()))
    idx_buf = ctypes.create_string_buffer(idx_bytes) if idx_bytes is not None else None
    idx_ptr = ctypes.cast(idx_buf, ctypes.c_void_p) if idx_buf is not None else None

    f32_cap = ctypes.c_int(0)
    i32_cap = ctypes.c_int(0)
    xml_b = str(Path(xml_path).resolve()).encode("utf-8")

    def _call_builder(f32_ptr: Any, i32_ptr: Any) -> int:
        if use_surface:
            return int(
                builder(
                    xml_b,
                    int(backend_kind),
                    int(station_count),
                    int(surface_label),
                    idx_ptr,
                    ctypes.cast(model_buf, ctypes.c_void_p),
                    ctypes.cast(desc_buf, ctypes.c_void_p),
                    f32_ptr,
                    ctypes.byref(f32_cap),
                    i32_ptr,
                    ctypes.byref(i32_cap),
                )
            )
        return int(
            builder(
                xml_b,
                int(backend_kind),
                int(station_count),
                idx_ptr,
                ctypes.cast(model_buf, ctypes.c_void_p),
                ctypes.cast(desc_buf, ctypes.c_void_p),
                f32_ptr,
                ctypes.byref(f32_cap),
                i32_ptr,
                ctypes.byref(i32_cap),
            )
        )

    rc = _call_builder(None, None)
    if rc != 0:
        raise RuntimeError(f"Host tyre/terrain size query failed with return code {rc}")

    f32_arr = np.zeros(max(1, int(f32_cap.value)), dtype=np.float32)
    i32_arr = np.zeros(max(1, int(i32_cap.value)), dtype=np.int32)
    f32_cap.value = int(f32_arr.size)
    i32_cap.value = int(i32_arr.size)

    rc = _call_builder(
        f32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        i32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
    )
    if rc != 0:
        raise RuntimeError(f"Host tyre/terrain build failed with return code {rc}")

    cand_manifest = Path(xml_path).resolve().parent / "manifest.json"
    if cand_manifest.is_file() and len(model_buf) == ctypes.sizeof(GpuTyreModelStruct):
        mf = json.loads(cand_manifest.read_text(encoding="utf-8"))
        ts = mf.get("track_spec") if isinstance(mf, dict) and isinstance(mf.get("track_spec"), dict) else {}
        secs = _as_seq(ts.get("surface_sections")) or _as_seq(ts.get("sections")) or []
        if len(secs) > 0:
            tm = GpuTyreModelStruct.from_buffer(model_buf)
            def_surf = ts.get("surface", int(surface_label) if use_surface and int(surface_label) >= 0 else "hardpack")
            _fill_surface_spec_pod(tm.surface_map.default_surface, def_surf)
            sec_cnt = min(len(secs), 16)
            tm.surface_map.section_count = sec_cnt
            for i in range(sec_cnt):
                sec = secs[i]
                if not isinstance(sec, dict):
                    continue
                tm.surface_map.sections[i].start_m = float(sec.get("start_m", 0.0))
                tm.surface_map.sections[i].end_m = float(sec.get("end_m", 0.0))
                tm.surface_map.sections[i].blend_m = float(sec.get("blend_m", 0.0))
                _fill_surface_spec_pod(
                    tm.surface_map.sections[i].surface,
                    sec.get("surface", sec.get("label", def_surf)),
                )
            tm.surface_map.has_surface_map = 1

    return (
        model_buf.raw,
        desc_buf.raw,
        np.ascontiguousarray(f32_arr[: int(f32_cap.value)], dtype=np.float32),
        np.ascontiguousarray(i32_arr[: int(i32_cap.value)], dtype=np.int32),
    )


def _fill_surface_spec_pod(dst: SurfaceSpecPodStruct, surface_name_or_id: Any) -> None:
    if isinstance(surface_name_or_id, (int, np.integer)):
        sid = int(surface_name_or_id)
        name = next((k for k, v in _SURFACE_LABEL_IDS.items() if v == sid), "hardpack")
    else:
        name = str(surface_name_or_id).strip().lower()
        if name not in _SURFACE_SPECS:
            name = "hardpack"
    spec = _SURFACE_SPECS[name]
    dst.mu_peak = float(spec["mu_peak"])
    dst.mu_slide = float(spec["mu_slide"])
    dst.slip_stiffness_per_load = float(spec["slip_stiffness_per_load"])
    dst.stribeck_speed_mps = float(spec["stribeck_speed_mps"])
    dst.crr_delta = float(spec["crr_delta"])
    dst.load_sensitivity = float(spec["load_sensitivity"])
    dst.surface_id = int(spec["surface_id"])


def _fill_wheel_chain(m_mj: Any, body_id: int, dst: GpuWheelChainStruct) -> None:
    dst.wheel_body_id = int(body_id)
    dst.njnt = int(m_mj.njnt)
    chain_jnts: list[tuple[int, int, int]] = []
    b = int(body_id)
    while b > 0:
        jnum = int(m_mj.body_jntnum[b])
        jadr = int(m_mj.body_jntadr[b])
        for k in range(jnum):
            jid = jadr + k
            chain_jnts.append((jid, int(m_mj.jnt_type[jid]), int(m_mj.jnt_dofadr[jid])))
        b = int(m_mj.body_parentid[b])
    chain_jnts.reverse()
    cnt = min(len(chain_jnts), 8)
    dst.joint_count = cnt
    for i in range(cnt):
        dst.joints[i].joint_id = chain_jnts[i][0]
        dst.joints[i].joint_type = chain_jnts[i][1]
        dst.joints[i].dof_adr = chain_jnts[i][2]


def _fill_pneumatic_material(
    dst: PneumaticMaterialViewStruct,
    tyre_dict: dict[str, Any] | None,
    default_spec: dict[str, float],
    n_rays: int,
    discretised_brush: bool,
    half_angle_deg: float = 75.0,
) -> None:
    import math

    src = tyre_dict if isinstance(tyre_dict, dict) else {}
    for k, v in default_spec.items():
        setattr(dst, k, float(src.get(k, v)))
    dst.n_rays = int(n_rays)
    dst.discretised_brush = 1 if discretised_brush else 0
    half_angle_rad = float(half_angle_deg) * (math.pi / 180.0)
    dst.half_angle_rad = float(half_angle_rad)
    step = (2.0 * half_angle_rad) / float(n_rays - 1)
    for i in range(n_rays):
        theta = -half_angle_rad + float(i) * step
        if i == n_rays - 1:
            theta = half_angle_rad
        dst.theta_rad[i] = float(theta)
        dst.ux[i] = float(math.sin(theta))
        dst.uz[i] = float(-math.cos(theta))
    dst.dtheta_rad = float(dst.theta_rad[1] - dst.theta_rad[0])


def build_gpu_tyre_model_struct_from_mjmodel(
    m_mj: Any,
    phys_cfg: dict[str, Any] | None = None,
    manifest: dict[str, Any] | None = None,
    backend: str | None = None,
    station_count: int | None = None,
    surface_label: int | None = None,
) -> tuple[GpuTyreModelStruct, dict[str, Any]]:
    """Build ``GpuTyreModelStruct`` purely in Python from ``m_mj``, ``physics_config.json``, and ``manifest.json``."""
    import math
    import mujoco

    pc = phys_cfg or {}
    tires = pc.get("tires")
    if not isinstance(tires, dict):
        tires = pc.get("tire") if isinstance(pc.get("tire"), dict) else {}

    be_name = str(backend or tires.get("backend", "compliant_2d"))
    if be_name == "compliant_2d":
        backend_kind = 0
    elif be_name == "distributed_2d_reference":
        backend_kind = 1
    elif be_name == "pneumatic":
        backend_kind = 2
    else:
        raise ValueError(f"Unknown tyre backend '{be_name}'")

    model = GpuTyreModelStruct()
    model.backend = int(backend_kind)
    model.nq = int(m_mj.nq)
    model.nv = int(m_mj.nv)
    model.nbody = int(m_mj.nbody)
    model.njnt = int(m_mj.njnt)

    distinct_deg = float(tires.get("distinct_normal_deg", 10.0))
    model.distinct_normal_cosine = float(math.cos(math.radians(distinct_deg)))

    pneu_cfg = tires.get("pneumatic") if isinstance(tires.get("pneumatic"), dict) else {}
    tier_str = str(pneu_cfg.get("tier", "fast")).strip().lower()
    if be_name == "pneumatic":
        if station_count is None:
            station_count = 256 if tier_str == "detailed" else 64
        discretised_brush = (int(station_count) >= 256) or (tier_str == "detailed")
        model.station_count = int(station_count)
    elif be_name == "distributed_2d_reference":
        dist_cfg = tires.get("distributed") if isinstance(tires.get("distributed"), dict) else {}
        model.station_count = int(station_count or dist_cfg.get("station_count", 64))
        discretised_brush = False
    else:
        model.station_count = int(station_count or 1)
        discretised_brush = False

    wheel_body_names = (("front_wheel",), ("rear_wheel",))
    wheel_geom_names = (
        ("geom_front_contact", "front_contact", "front_tire", "front_wheel"),
        ("geom_rear_contact", "rear_contact", "rear_tire", "rear_wheel"),
    )
    default_radii = (0.372, 0.352)
    rb = manifest.get("resolved_bike") if isinstance(manifest, dict) and isinstance(manifest.get("resolved_bike"), dict) else {}
    rb_radii = (
        float(rb["front_wheel_radius_mm"]) * 1e-3 if rb.get("front_wheel_radius_mm") else None,
        float(rb["rear_wheel_radius_mm"]) * 1e-3 if rb.get("rear_wheel_radius_mm") else None,
    )

    pneu_defaults = (
        {
            "outer_radius_mm": 372.0,
            "rim_radius_mm": 320.0,
            "section_height_mm": 55.0,
            "pressure_bar": 1.7,
            "crr_reference": 0.011,
            "casing_width_mm": 60.0,
            "compressed_casing_mm": 6.0,
            "area_factor": 0.4435,
            "carcass_stiffness_n_mm2": 0.1194,
            "contact_length_factor": 0.8589,
            "rate_stiffening": 0.2484,
            "rate_relaxation_s": 0.2,
            "loss_factor": 0.0662,
            "hysteresis_rate_eps_mps": 0.01,
            "relaxation_length_mm": 60.0,
            "rim_stiffness_n_mm2": 28.0,
            "tread_loss_crr": 0.00586,
        },
        {
            "outer_radius_mm": 352.0,
            "rim_radius_mm": 300.0,
            "section_height_mm": 55.0,
            "pressure_bar": 1.7,
            "crr_reference": 0.0103,
            "casing_width_mm": 60.0,
            "compressed_casing_mm": 6.0,
            "area_factor": 0.4576,
            "carcass_stiffness_n_mm2": 0.1225,
            "contact_length_factor": 0.8801,
            "rate_stiffening": 0.2498,
            "rate_relaxation_s": 0.2,
            "loss_factor": 0.069,
            "hysteresis_rate_eps_mps": 0.01,
            "relaxation_length_mm": 60.0,
            "rim_stiffness_n_mm2": 28.0,
            "tread_loss_crr": 0.00481,
        },
    )

    frame_bid = _lookup_body_id(m_mj, ("frame", "main_frame", "chassis"), 1)
    _fill_wheel_chain(m_mj, frame_bid, model.frame_chain)

    side_keys = ("front", "rear")
    dist_cfg = tires.get("distributed") if isinstance(tires.get("distributed"), dict) else {}
    for s in range(2):
        w = model.wheels[s]
        bid = _lookup_body_id(m_mj, wheel_body_names[s], 1)
        _fill_wheel_chain(m_mj, bid, w.chain)
        gid = -1
        for gname in wheel_geom_names[s]:
            cand = int(mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_GEOM, gname))
            if cand >= 0:
                gid = cand
                break
        w.geom_id = gid

        geom_r = float(m_mj.geom_size[gid, 0]) if gid >= 0 else default_radii[s]
        w.radius_m = float(rb_radii[s] or geom_r or default_radii[s])

        tp = tires.get(side_keys[s]) if isinstance(tires.get(side_keys[s]), dict) else {}
        w.tangent_k_n_m = float(tp.get("tangent_k_n_m", 48000.0))
        w.tire_mu = float(tp.get("mu", 0.90))
        w.relaxation_length_m = float(tp.get("relaxation_length_m", 0.08))

        mat = tp.get("material") if isinstance(tp.get("material"), dict) else {}
        defl_seq = _as_seq(mat.get("deflection_m"))
        force_seq = _as_seq(mat.get("force_n"))
        if defl_seq is not None and force_seq is not None and len(defl_seq) == len(force_seq) and 2 <= len(defl_seq) <= 64:
            n_k = len(defl_seq)
            w.compliant.has_tabulated_radial = 1
            w.compliant.use_tabulated = 1
            w.compliant.knot_count = n_k
            for i in range(n_k):
                w.compliant.deflection_m[i] = float(defl_seq[i])
                w.compliant.force_n[i] = float(force_seq[i])
            w.compliant.integral[0] = 0.0
            for i in range(n_k - 1):
                dx = float(defl_seq[i + 1]) - float(defl_seq[i])
                df = float(force_seq[i + 1]) - float(force_seq[i])
                w.compliant.slopes[i] = float(df / dx) if dx > 0.0 else 0.0
                w.compliant.integral[i + 1] = float(
                    w.compliant.integral[i] + 0.5 * (float(force_seq[i]) + float(force_seq[i + 1])) * dx
                )
            w.compliant.radial_k_n_m = float(w.compliant.slopes[0]) if n_k >= 2 else float(mat.get("radial_k_n_m", 120000.0))
        else:
            w.compliant.radial_k_n_m = float(mat.get("radial_k_n_m", 120000.0))
            w.compliant.has_tabulated_radial = 0
            w.compliant.use_tabulated = 0
        w.compliant.loss_factor = float(mat.get("loss_factor", 0.05))
        w.compliant.hysteresis_rate_eps_mps = float(mat.get("hysteresis_rate_eps_mps", 0.01))
        vlr = _as_seq(mat.get("valid_load_range_n"))
        if vlr is not None and len(vlr) == 2:
            w.compliant.valid_load_min_n = float(vlr[0])
            w.compliant.valid_load_max_n = float(vlr[1])

        dmat = dist_cfg.get(f"{side_keys[s]}_material") if isinstance(dist_cfg.get(f"{side_keys[s]}_material"), dict) else dist_cfg
        dens = dmat.get("density") if isinstance(dmat.get("density"), dict) else {}
        knots_seq = _as_seq(dens.get("knots_m")) or [0.0, 0.025]
        stiff_seq = _as_seq(dens.get("stiffness_n_m")) or [1.2e6, 1.2e6]
        kcnt = min(len(knots_seq), len(stiff_seq), 8)
        w.distributed_density.knot_count = kcnt
        for i in range(kcnt):
            w.distributed_density.knots_m[i] = float(knots_seq[i])
            w.distributed_density.stiffness_n_m[i] = float(stiff_seq[i])
        w.distributed_density.damping_ns_m = float(dens.get("damping_ns_m", 0.0))
        w.distributed_density.loss_factor = float(dens.get("loss_factor", 0.05))
        w.distributed_density.hysteresis_rate_eps_mps = float(dens.get("hysteresis_rate_eps_mps", 0.01))
        vdr = _as_seq(dmat.get("valid_deflection_range_m"))
        if vdr is not None and len(vdr) == 2:
            w.valid_deflection_min_m = float(vdr[0])
            w.valid_deflection_max_m = float(vdr[1])
        else:
            w.valid_deflection_max_m = float(w.radius_m * 0.45)

        if be_name == "distributed_2d_reference":
            scnt = max(1, min(int(model.station_count), 128))
            w.station_weight = float((2.0 * math.pi) / float(scnt))
            for st in range(scnt):
                ang = (2.0 * math.pi * float(st)) / float(scnt)
                w.local_xyz[3 * st + 0] = float(w.radius_m * math.cos(ang))
                w.local_xyz[3 * st + 1] = 0.0
                w.local_xyz[3 * st + 2] = float(w.radius_m * math.sin(ang))

        p_side = pneu_cfg.get(side_keys[s]) if isinstance(pneu_cfg.get(side_keys[s]), dict) else {}
        _fill_pneumatic_material(
            w.pneumatic,
            p_side,
            pneu_defaults[s],
            int(model.station_count) if be_name == "pneumatic" else 64,
            discretised_brush,
            half_angle_deg=float(pneu_cfg.get("half_angle_deg", 75.0)),
        )

    # Surface map (Stage A)
    ts = manifest.get("track_spec") if isinstance(manifest, dict) and isinstance(manifest.get("track_spec"), dict) else {}
    def_surf = surface_label if surface_label is not None else ts.get("surface", pneu_cfg.get("surface", "hardpack"))
    _fill_surface_spec_pod(model.surface_map.default_surface, def_surf)
    secs = _as_seq(ts.get("surface_sections")) or _as_seq(ts.get("sections")) or []
    sec_cnt = min(len(secs), 16)
    model.surface_map.section_count = sec_cnt
    for i in range(sec_cnt):
        sec = secs[i]
        if not isinstance(sec, dict):
            continue
        model.surface_map.sections[i].start_m = float(sec.get("start_m", 0.0))
        model.surface_map.sections[i].end_m = float(sec.get("end_m", 0.0))
        model.surface_map.sections[i].blend_m = float(sec.get("blend_m", 0.0))
        _fill_surface_spec_pod(
            model.surface_map.sections[i].surface,
            sec.get("surface", sec.get("label", def_surf)),
        )
    model.surface_map.has_surface_map = 1 if (be_name == "pneumatic" or sec_cnt > 0 or surface_label is not None) else 0

    meta = {
        "backend": be_name,
        "station_count": int(model.station_count),
        "discretised_brush": bool(discretised_brush),
        "significant_delta_m": float(tires.get("significant_delta_m", 0.002)),
        "significance_fraction": float(tires.get("significance_fraction", 0.25)),
        "distinct_normal_cosine": float(model.distinct_normal_cosine),
    }
    return model, meta


def build_terrain_buffers_from_mjmodel(
    m_mj: Any,
    model_struct: GpuTyreModelStruct,
    significant_delta_m: float = 0.002,
    significance_fraction: float = 0.25,
) -> tuple[bytes, bytes, np.ndarray, np.ndarray]:
    """Build ``(tyre_model_bytes, terrain_desc_bytes, terrain_f32, terrain_i32)`` from ``m_mj`` in Python."""
    import mujoco

    terrain_gid = int(mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_GEOM, "terrain"))
    if terrain_gid < 0:
        raise ValueError("MuJoCo model has no 'terrain' geom")

    gtype = int(m_mj.geom_type[terrain_gid])
    gpos = np.asarray(m_mj.geom_pos[terrain_gid], dtype=np.float64)
    rmat = np.asarray(m_mj.geom_mat[terrain_gid], dtype=np.float64) if hasattr(m_mj, "geom_mat") else np.eye(3)

    if gtype == int(mujoco.mjtGeom.mjGEOM_HFIELD):
        hf_id = int(m_mj.geom_dataid[terrain_gid])
        nrow = int(m_mj.hfield_nrow[hf_id])
        ncol = int(m_mj.hfield_ncol[hf_id])
        adr = int(m_mj.hfield_adr[hf_id])
        size = np.asarray(m_mj.hfield_size[hf_id], dtype=np.float64)  # (hx, hy, z_top, z_base)
        raw = np.asarray(m_mj.hfield_data[adr : adr + nrow * ncol], dtype=np.float64).reshape(nrow, ncol)
        row_idx = nrow // 2
        z_samples = gpos[2] + raw[row_idx, :] * size[2]
        x_local = np.linspace(-size[0], size[0], ncol, dtype=np.float64)
        px = gpos[0] + x_local
        pz = z_samples
        step = float(px[1] - px[0]) if ncol >= 2 else 0.0
        if step > 0.0 and np.allclose(rmat.reshape(3, 3), np.eye(3), atol=1e-9):
            model_struct.road_x0_m = float(px[0])
            model_struct.road_dx_m = float(step)
            model_struct.road_is_uniform_grid = 1
        else:
            model_struct.road_x0_m = 0.0
            model_struct.road_dx_m = 0.0
            model_struct.road_is_uniform_grid = 0
    else:
        # Flat or tilted plane geom
        px = np.array([-1000.0, 5000.0], dtype=np.float64)
        pz = np.array([float(gpos[2]), float(gpos[2])], dtype=np.float64)
        model_struct.road_x0_m = float(px[0])
        model_struct.road_dx_m = float(px[1] - px[0])
        model_struct.road_is_uniform_grid = 1

    px32 = np.asarray(px, dtype=np.float32)
    pz32 = np.asarray(pz, dtype=np.float32)
    n_pts = int(px32.size)
    if n_pts < 2:
        raise ValueError(f"Terrain polyline requires at least 2 vertices, got {n_pts}")
    dx = px32[1:] - px32[:-1]
    if not bool(np.all(dx > 0.0)):
        raise ValueError("Terrain profile x coordinates must be strictly increasing")
    dz = pz32[1:] - pz32[:-1]
    len_sq = dx * dx + dz * dz

    left_x = np.zeros(n_pts, dtype=np.float32)
    left_z = np.zeros(n_pts, dtype=np.float32)
    left_x[1:] = -dx
    left_z[1:] = -dz
    right_x = np.zeros(n_pts, dtype=np.float32)
    right_z = np.zeros(n_pts, dtype=np.float32)
    right_x[:-1] = dx
    right_z[:-1] = dz

    height_changes = np.zeros(n_pts, dtype=np.int32)
    height_changes[1:] = np.cumsum((dz != 0.0).astype(np.int32), dtype=np.int32)

    # Signed profile shares the strictly monotonic polyline vertices (tyre_upload.cpp)
    sp_x = px32
    sp_z = pz32
    n_sp = n_pts
    sp_ex = dx
    sp_ez = dz
    sp_len2 = len_sq
    sp_len = np.sqrt(np.maximum(sp_len2, 1e-12))
    sp_nx = (-sp_ez / sp_len).astype(np.float32)
    sp_nz = (sp_ex / sp_len).astype(np.float32)

    f32_parts = [
        px32,
        pz32,
        dx,
        dz,
        len_sq,
        left_x,
        left_z,
        right_x,
        right_z,
        sp_x,
        sp_z,
        sp_ex,
        sp_ez,
        sp_len2,
        sp_nx,
        sp_nz,
    ]
    offsets: list[int] = []
    cur = 0
    for part in f32_parts:
        offsets.append(cur)
        cur += int(part.size)
    f32_arr = np.ascontiguousarray(np.concatenate(f32_parts), dtype=np.float32)

    i32_arr = np.ascontiguousarray(height_changes, dtype=np.int32)
    desc = GpuTerrainTableDescStruct()
    desc.polyline_vertex_count = n_pts
    desc.px_offset = offsets[0]
    desc.pz_offset = offsets[1]
    desc.seg_x_offset = offsets[2]
    desc.seg_z_offset = offsets[3]
    desc.seg_len_sq_offset = offsets[4]
    desc.height_changes_offset = 0
    desc.left_x_offset = offsets[5]
    desc.left_z_offset = offsets[6]
    desc.right_x_offset = offsets[7]
    desc.right_z_offset = offsets[8]
    desc.maximum_z = float(np.max(pz32))
    desc.significant_delta_m = float(significant_delta_m)
    desc.significance_fraction = float(significance_fraction)
    desc.normal_cosine = float(model_struct.distinct_normal_cosine)
    desc.signed_vertex_count = n_sp
    desc.sp_x_offset = offsets[9]
    desc.sp_z_offset = offsets[10]
    desc.sp_edge_x_offset = offsets[11]
    desc.sp_edge_z_offset = offsets[12]
    desc.sp_length2_offset = offsets[13]
    desc.sp_normal_x_offset = offsets[14]
    desc.sp_normal_z_offset = offsets[15]
    desc.f32_count = int(f32_arr.size)
    desc.i32_count = int(i32_arr.size)

    return bytes(model_struct), bytes(desc), f32_arr, i32_arr


def configure_host_spindle_builder(lib: ctypes.CDLL, symbol_name: str) -> Any:
    """Bind ``bike_{cpu,metal}_build_spindle_params`` argtypes and restype."""
    fn = getattr(lib, symbol_name)
    fn.argtypes = [
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_void_p,
    ]
    fn.restype = ctypes.c_int
    return fn


def build_spindle_params_bytes(
    host_lib: ctypes.CDLL,
    xml_path: Path | str,
    strength_json_path: Path | str,
    envelope_json_path: Path | str,
    control_period_steps: int = 1,
) -> bytes:
    """Resolve ``SpindleM2K2Params<float>`` bytes via host/metal C++ builder."""
    if hasattr(host_lib, "bike_cpu_build_spindle_params"):
        builder = configure_host_spindle_builder(host_lib, "bike_cpu_build_spindle_params")
    elif hasattr(host_lib, "bike_metal_build_spindle_params"):
        builder = configure_host_spindle_builder(host_lib, "bike_metal_build_spindle_params")
    else:
        raise AttributeError(
            "host_lib exposes neither bike_cpu_build_spindle_params nor bike_metal_build_spindle_params"
        )

    size_fn = None
    for cand in ("bike_gpu_sizeof_spindle_params", "bike_metal_sizeof_spindle_params"):
        if hasattr(host_lib, cand):
            size_fn = getattr(host_lib, cand)
            size_fn.argtypes = []
            size_fn.restype = ctypes.c_int
            break
    if size_fn is None:
        raise AttributeError("host_lib exposes neither bike_gpu_sizeof_spindle_params nor bike_metal_sizeof_spindle_params")

    params_size = int(size_fn())
    buf = ctypes.create_string_buffer(params_size)
    rc = int(
        builder(
            str(Path(xml_path).resolve()).encode("utf-8"),
            str(Path(strength_json_path).resolve()).encode("utf-8"),
            str(Path(envelope_json_path).resolve()).encode("utf-8"),
            int(control_period_steps),
            ctypes.cast(buf, ctypes.c_void_p),
        )
    )
    if rc != 0:
        raise RuntimeError(f"Host spindle params build failed with return code {rc}")
    return buf.raw


def unpack_sp3_energy_ledger(raw_bytes: bytes | bytearray | memoryview) -> dict[str, Any]:
    """Decode ``GpuEnergyLedger`` fields from one ``GpuSp3WorldState`` struct."""
    st = GpuSp3WorldStateStruct.from_buffer_copy(raw_bytes)
    el = st.energy_ledger
    return {
        "E_mech_initial": float(el.E_mech_initial),
        "E_mech": float(el.E_mech),
        "E_scale_baseline": float(el.E_scale_baseline),
        "E_scale_min": float(el.E_scale_min),
        "W_muscle_signed": float(el.W_muscle_signed),
        "W_muscle_pos": float(el.W_muscle_pos),
        "W_motor_signed": float(el.W_motor_signed),
        "W_motor_pos": float(el.W_motor_pos),
        "W_cruise_signed": float(el.W_cruise_signed),
        "W_cruise_pos": float(el.W_cruise_pos),
        "W_external_signed": float(el.W_external_signed),
        "D_tyre_radial": float(el.D_tyre_radial),
        "D_tyre_brush_roll": float(el.D_tyre_brush_roll),
        "D_suspension": float(el.D_suspension),
        "D_resistance": float(el.D_resistance),
        "D_brake": float(el.D_brake),
        "D_rider_end_stop": float(el.D_rider_end_stop),
        "W_constraint_signed": float(el.W_constraint_signed),
        "W_constraint_abs": float(el.W_constraint_abs),
        "W_closure_signed": float(el.W_closure_signed),
        "W_closure_abs": float(el.W_closure_abs),
        "source_j": float(el.source_j),
        "source_positive_j": float(el.source_positive_j),
        "delta_E_mech": float(el.delta_E_mech),
        "net_supply": float(el.net_supply),
        "residual_j": float(el.residual_j),
        "energy_scale_j": float(el.energy_scale_j),
        "rel_residual": float(el.rel_residual),
        "max_rel_residual": float(el.max_rel_residual),
        "step_count": int(el.step_count),
        "initialized": int(el.initialized),
    }


def unpack_sp4_world_metrics(raw_bytes: bytes | bytearray | memoryview) -> dict[str, Any]:
    """Decode the scalar metrics tail from one ``GpuSp4WorldState`` struct."""
    import struct

    tail_fmt = "<16f3i8B"
    tail_size = struct.calcsize(tail_fmt)
    if len(raw_bytes) < tail_size:
        raise ValueError(f"Expected >= {tail_size} bytes for GpuSp4WorldState, got {len(raw_bytes)}")
    vals = struct.unpack_from(tail_fmt, raw_bytes, len(raw_bytes) - tail_size)
    max_rise_f = float(vals[3])
    max_rise_r = float(vals[4])
    return {
        "last_posture_min_lean_rad": float(vals[0]),
        "min_press_front_n": float(vals[1]),
        "min_press_rear_n": float(vals[2]),
        "max_rising_front_excess_n": None if max_rise_f <= -1e8 else max_rise_f,
        "max_rising_rear_excess_n": None if max_rise_r <= -1e8 else max_rise_r,
        "min_saddle_force_n": float(vals[5]),
        "max_saddle_force_n": float(vals[6]),
        "sum_delivered_crank_torque_nm": float(vals[7]),
        "sum_commanded_crank_torque_nm": float(vals[8]),
        "start_root_x_m": float(vals[9]),
        "end_root_x_m": float(vals[10]),
        "distance_m": float(vals[10] - vals[9]),
        "end_speed_mps": float(vals[11]),
        "max_speed_mps": float(vals[12]),
        "energy_residual_j": float(vals[13]),
        "energy_scale_j": float(vals[14]),
        "max_rel_energy_residual": float(vals[15]),
        "sample_count": int(vals[16]),
        "max_m0_passes_used": int(vals[17]),
        "current_m0_passes": int(vals[18]),
        "has_posture_min_lean": int(vals[19]),
        "has_start_root_x": int(vals[20]),
        "active_control_tick": int(vals[21]),
        "last_cmd_enabled": int(vals[22]),
        "energy_contract_ok": int(vals[23]),
        "solver_mode": int(vals[24]),
    }


if __name__ == "__main__":
    so_lib = compile_cuda_writers()
    info = {
        "sizeof_world_config": int(so_lib.bike_gpu_sizeof_world_config()),
        "sizeof_world_state": int(so_lib.bike_gpu_sizeof_world_state()),
        "sizeof_model_indices": int(so_lib.bike_gpu_sizeof_model_indices()),
        "sizeof_tyre_model": int(so_lib.bike_gpu_sizeof_tyre_model()),
        "sizeof_terrain_desc": int(so_lib.bike_gpu_sizeof_terrain_desc()),
        "sizeof_sp3_config": int(so_lib.bike_gpu_sizeof_sp3_config()),
        "sizeof_sp3_world_state": int(so_lib.bike_gpu_sizeof_sp3_world_state()),
        "sizeof_sp4_world_state": int(so_lib.bike_gpu_sizeof_sp4_world_state()),
        "sizeof_pneumatic_scratch": int(so_lib.bike_gpu_sizeof_pneumatic_scratch()),
        "sizeof_spindle_m0_scratch": int(so_lib.bike_gpu_sizeof_spindle_m0_scratch()),
    }
    print(json.dumps(info, indent=2))
