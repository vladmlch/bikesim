"""
Simulation Telemetry Provider.

Computes live suspension telemetry, kinematic metrics, air/damper forces,
axle loads, and center of gravity coordinates from MuJoCo state.
"""

from dataclasses import dataclass
from typing import Any, Dict
import mujoco
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.physics.air_spring import ForkAirSpring
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.sim.camera import CameraManager
from bike_sim.sim.controllers import SuspensionController


@dataclass
class SimModelHandles:
    """Cached joint, actuator, and site IDs in the MuJoCo model."""
    jnt_main_pivot: int = -1
    jnt_shock: int = -1
    jnt_fork: int = -1
    site_p3_ss: int = -1
    site_p3_roc: int = -1
    site_bb: int = -1
    site_pfa: int = -1
    site_pra: int = -1

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> "SimModelHandles":
        """Resolves and caches all relevant joint and site IDs from model."""
        return cls(
            jnt_main_pivot=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "main_pivot"),
            jnt_shock=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke"),
            jnt_fork=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fork_travel"),
            site_p3_ss=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_P3_ss"),
            site_p3_roc=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_P3_rocker"),
            site_bb=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_BB"),
            site_pfa=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_PFA"),
            site_pra=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_PRA"),
        )


class TelemetryProvider:
    """
    Computes real-time telemetry metrics for the bicycle suspension simulation.
    """

    def __init__(
        self,
        specs: BikeSpecs,
        solver: HorstLinkageSolver,
        air_spring: ForkAirSpring,
        suspension_system: BikeSuspensionSystem,
        camera_manager: CameraManager,
    ) -> None:
        self.specs = specs
        self.solver = solver
        self.air_spring = air_spring
        self.suspension_system = suspension_system
        self.camera_manager = camera_manager
        # Single owner of the spring+damper force model; see sim/controllers.py.
        self._controller = SuspensionController(
            specs=specs, air_spring=air_spring, suspension_system=suspension_system
        )

    def compute_telemetry(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        target_rear_travel_mm: float,
        include_rider: bool,
        handles: SimModelHandles,
    ) -> Dict[str, Any]:
        """
        Computes all instant kinematic and dynamic telemetry values from current simulation state.
        """
        kin = self._compute_kinematics(model, data, target_rear_travel_mm, handles)
        forces = self._compute_forces(model, data, kin["fork_travel_mm"], kin["shock_stroke_mm"], handles)
        res = self._compute_residuals(data, handles)
        balance = self._compute_mass_and_balance(model, data, handles)

        fd = self.suspension_system.fork_damper
        sd = self.suspension_system.shock_damper

        return {
            "speed_kmh": 0.0,
            "speed_mps": 0.0,
            "pos_x_m": 0.0,
            "camera_mode": self.camera_manager.mode_name,
            "camera_mode_id": self.camera_manager.active_mode,
            "rider_active": include_rider,
            **kin,
            **forces,
            "fork_air_psi": float(self.air_spring.gauge_pressure_psi),
            "fork_tokens": int(self.air_spring.num_tokens),
            "fork_hsc": fd.hsc_clicks,
            "fork_lsc": fd.lsc_clicks,
            "fork_reb": fd.rebound_clicks,
            "shock_hsc": sd.hsc_clicks,
            "shock_lsc": sd.lsc_clicks,
            "shock_reb": sd.rebound_clicks,
            "shock_hbo": sd.hbo_clicks,
            "shock_lockout": sd.lockout_firm,
            "damper_preset": self.suspension_system.current_preset.name,
            **res,
            **balance,
        }

    def _compute_kinematics(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        target_rear_travel_mm: float,
        h: SimModelHandles,
    ) -> Dict[str, Any]:
        """Resolves rear travel, shock stroke, fork travel, and instantaneous leverage ratio."""
        if h.jnt_main_pivot >= 0:
            q_pivot = data.qpos[model.jnt_qposadr[h.jnt_main_pivot]]
            theta_cs_rad = float(self.solver.theta_cs_0 - q_pivot)
            kin_state = self.solver.solve_state_from_chainstay_angle(theta_cs_rad)
            rear_travel = max(0.0, min(float(self.specs.rear_wheel_travel), float(kin_state["wheel_travel"])))
            shock_stroke = max(0.0, min(float(self.specs.shock_stroke), float(kin_state["shock_stroke"])))
            leverage_ratio = float(kin_state["leverage_ratio"])
        elif h.jnt_shock >= 0:
            q_shock = data.qpos[model.jnt_qposadr[h.jnt_shock]]
            shock_stroke = max(0.0, min(float(self.specs.shock_stroke), q_shock * 1000.0))
            kin_state = self.solver.solve_state_from_shock_stroke(shock_stroke)
            rear_travel = float(kin_state["wheel_travel"])
            leverage_ratio = float(kin_state["leverage_ratio"])
        else:
            kin_state = self.solver.solve_state_from_wheel_travel(target_rear_travel_mm)
            rear_travel = target_rear_travel_mm
            shock_stroke = float(kin_state["shock_stroke"])
            leverage_ratio = float(kin_state["leverage_ratio"])

        fork_travel = max(0.0, data.qpos[model.jnt_qposadr[h.jnt_fork]] * 1000.0) if h.jnt_fork >= 0 else 0.0
        shock_pct = min(100.0, (shock_stroke / float(self.specs.shock_stroke)) * 100.0)

        return {
            "rear_travel_mm": rear_travel,
            "fork_travel_mm": fork_travel,
            "shock_stroke_mm": shock_stroke,
            "shock_compression_pct": shock_pct,
            "leverage_ratio": leverage_ratio,
        }

    def _compute_forces(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        fork_travel_mm: float,
        shock_stroke_mm: float,
        h: SimModelHandles,
    ) -> Dict[str, Any]:
        """Calculates instantaneous spring, damper, and total suspension forces."""
        v_shock = float(data.qvel[model.jnt_dofadr[h.jnt_shock]]) if h.jnt_shock >= 0 else 0.0
        v_fork = float(data.qvel[model.jnt_dofadr[h.jnt_fork]]) if h.jnt_fork >= 0 else 0.0

        fork_total, _f_air, f_d_fork = self._controller.compute_fork_force(fork_travel_mm, v_fork)
        shock_total, _f_spring, f_d_shock = self._controller.compute_shock_force(shock_stroke_mm, v_shock)
        fork_k = self.air_spring.compute_instantaneous_stiffness(fork_travel_mm)

        return {
            "v_fork_mps": v_fork,
            "v_shock_mps": v_shock,
            "fork_force_n": fork_total,
            "fork_damper_n": f_d_fork,
            "fork_k_n_mm": float(fork_k),
            "shock_force_n": shock_total,
            "shock_damper_n": f_d_shock,
        }

    def _compute_residuals(self, data: mujoco.MjData, h: SimModelHandles) -> Dict[str, Any]:
        """Calculates 4-bar kinematic loop closure residual at Horst pivot P3."""
        if h.site_p3_ss >= 0 and h.site_p3_roc >= 0:
            residual = float(np.linalg.norm(data.site_xpos[h.site_p3_ss] - data.site_xpos[h.site_p3_roc]) * 1000.0)
        else:
            residual = 0.0
        return {"p3_residual_mm": residual}

    def _compute_mass_and_balance(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        h: SimModelHandles,
    ) -> Dict[str, Any]:
        """Calculates subtree center of gravity and front/rear static load distribution."""
        frame_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
        if frame_id >= 0:
            total_mass = float(sum(model.body_mass[frame_id:]))
            com = data.subtree_com[frame_id]
        else:
            total_mass = float(sum(model.body_mass))
            com = data.subtree_com[0]

        bb_pos = data.site_xpos[h.site_bb] if h.site_bb >= 0 else np.zeros(3)
        cg_rel_bb = com - bb_pos

        x_fa = data.site_xpos[h.site_pfa][0] if h.site_pfa >= 0 else 0.833
        x_ra = data.site_xpos[h.site_pra][0] if h.site_pra >= 0 else -0.448
        wb = max(0.5, x_fa - x_ra)

        front_pct = float(np.clip(((com[0] - x_ra) / wb) * 100.0, 0.0, 100.0))
        rear_pct = float(100.0 - front_pct)
        total_weight_n = total_mass * 9.81

        return {
            "total_mass_kg": total_mass,
            "cg_x_mm": float(cg_rel_bb[0] * 1000.0),
            "cg_z_mm": float(cg_rel_bb[2] * 1000.0),
            "front_load_pct": front_pct,
            "rear_load_pct": rear_pct,
            "front_load_n": total_weight_n * (front_pct / 100.0),
            "rear_load_n": total_weight_n * (rear_pct / 100.0),
        }
