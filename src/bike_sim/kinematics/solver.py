"""
Horst-Link 4-Bar Suspension Kinematics Solver.

This module implements an exact, closed-form analytical kinematics solver for the
Horst-Link + Yoke rear suspension mechanism in the MuJoCo 3D coordinate system
(+X: Forward, +Y: Lateral/Left, +Z: Up, Origin at Bottom Bracket BB (0,0,0)).
"""

import functools
from typing import Any, Callable, Dict, List, Optional, Tuple
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import (
    compute_front_axle,
    compute_headtube_points,
    compute_rear_axle,
    get_fixed_frame_points,
)


from bike_sim.kinematics.linkage_math import (
    brentq_pure,
    circle_circle_intersection_2d,
)


class HorstLinkageSolver:
    """
    Exact analytical closed-form kinematics solver for the Horst-Link + Yoke rear suspension.

    All lengths are in millimeters (mm), angles in radians/degrees, coordinate system
    is MuJoCo 3D (+X: Forward, +Y: Left, +Z: Up, Origin at BB (0,0,0)).
    """

    def __init__(
        self,
        specs: Optional[BikeSpecs] = None,
        p0: Optional[np.ndarray] = None,
        p5: Optional[np.ndarray] = None,
        p7: Optional[np.ndarray] = None,
        p1_0: Optional[np.ndarray] = None,
        p2_0: Optional[np.ndarray] = None,
        p3_0: Optional[np.ndarray] = None,
        p4_0: Optional[np.ndarray] = None,
        p6_0: Optional[np.ndarray] = None,
        p12_0: Optional[np.ndarray] = None,
    ) -> None:
        self.specs = specs or BikeSpecs()
        fixed_points = get_fixed_frame_points(self.specs)

        # Fixed Frame Pivots (mm)
        self.p0 = np.array(p0 if p0 is not None else fixed_points["P0"], dtype=float)
        self.p5 = np.array(p5 if p5 is not None else fixed_points["P5"], dtype=float)
        self.p7 = np.array(p7 if p7 is not None else fixed_points["P7"], dtype=float)
        self.bb = np.array(fixed_points["BB"], dtype=float)
        self.p_fa = compute_front_axle(self.specs)
        p11, _ = compute_headtube_points(self.specs)
        self.p11 = p11

        # Uncompressed Reference Coordinates (mm)
        self.p1_0 = np.array(
            p1_0 if p1_0 is not None else compute_rear_axle(self.specs), dtype=float
        )
        self.p2_0 = np.array(
            p2_0 if p2_0 is not None else [-379.118647, 0.0, -8.442119], dtype=float
        )
        self.p3_0 = np.array(
            p3_0 if p3_0 is not None else [-71.715477, 0.0, 206.598467], dtype=float
        )
        self.p4_0 = np.array(
            p4_0 if p4_0 is not None else [-38.545800, 0.0, 191.462565], dtype=float
        )
        self.p6_0 = np.array(
            p6_0 if p6_0 is not None else [28.408048, 0.0, 258.549831], dtype=float
        )
        self.p12_0 = np.array(
            p12_0 if p12_0 is not None else [-414.000688, 0.0, 59.986864], dtype=float
        )

        # Rigid Link Length Invariants (mm)
        self.l_cs = float(np.linalg.norm(self.p2_0 - self.p0))
        self.l_ss = float(np.linalg.norm(self.p3_0 - self.p2_0))
        self.r_53 = float(np.linalg.norm(self.p3_0 - self.p5))
        self.r_54 = float(np.linalg.norm(self.p4_0 - self.p5))
        self.r_21 = float(np.linalg.norm(self.p1_0 - self.p2_0))
        self.r_2_12 = float(np.linalg.norm(self.p12_0 - self.p2_0))
        self.l_dropout_height = float(np.linalg.norm(self.p1_0 - self.p12_0))
        self.l_ss_tube = float(np.linalg.norm(self.p3_0 - self.p12_0))
        self.l_yoke = float(np.linalg.norm(self.p6_0 - self.p4_0))
        self.l_shock_0 = float(np.linalg.norm(self.p7 - self.p6_0))

        # Reference Angles (radians)
        self.theta_cs_0 = float(
            np.arctan2(self.p2_0[2] - self.p0[2], self.p2_0[0] - self.p0[0])
        )

        # Rocker link rigid angular offset beta = angle(P4-P5) - angle(P3-P5)
        v53_0 = self.p3_0[[0, 2]] - self.p5[[0, 2]]
        v54_0 = self.p4_0[[0, 2]] - self.p5[[0, 2]]
        self.beta_rocker = float(
            np.arctan2(v54_0[1], v54_0[0]) - np.arctan2(v53_0[1], v53_0[0])
        )

        # Seatstay link rigid angular offsets:
        v23_0 = self.p3_0[[0, 2]] - self.p2_0[[0, 2]]
        v21_0 = self.p1_0[[0, 2]] - self.p2_0[[0, 2]]
        v2_12_0 = self.p12_0[[0, 2]] - self.p2_0[[0, 2]]
        self.gamma_ra = float(
            np.arctan2(v21_0[1], v21_0[0]) - np.arctan2(v23_0[1], v23_0[0])
        )
        self.gamma_12 = float(
            np.arctan2(v2_12_0[1], v2_12_0[0]) - np.arctan2(v23_0[1], v23_0[0])
        )

    @functools.cached_property
    def _stroke_table(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Monotonic travel <-> stroke lookup table for exact O(1) inverse solving.

        Built lazily: only solve_state_from_shock_stroke needs it, and most
        solver instances never call that, so the 101 root-finds would be wasted.
        """
        travel_samples = np.linspace(0.0, float(self.specs.rear_wheel_travel), 101)
        stroke_samples = [
            self.solve_state_from_wheel_travel(float(tr))["shock_stroke"]
            for tr in travel_samples
        ]
        return (
            np.array(travel_samples, dtype=float),
            np.array(stroke_samples, dtype=float),
        )

    def solve_state_from_shock_stroke(self, shock_stroke_mm: float) -> Dict[str, Any]:
        """
        Solves complete 3D kinematic state given shock stroke in mm by exact
        monotonic 1D interpolation against the kinematic trajectory.
        """
        sample_travel, sample_stroke = self._stroke_table
        stroke_clamped = float(np.clip(shock_stroke_mm, 0.0, float(self.specs.shock_stroke)))
        wheel_travel_est = float(np.interp(stroke_clamped, sample_stroke, sample_travel))
        return self.solve_state_from_wheel_travel(wheel_travel_est)

    def _solve_pivots_from_theta_cs(self, theta_cs: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
        """Calculates 3D coordinates for all moving pivots given chainstay angle theta_cs."""
        # 1. Horst Pivot P2 on Circle(P0, L_cs)
        p2_x = self.p0[0] + self.l_cs * np.cos(theta_cs)
        p2_z = self.p0[2] + self.l_cs * np.sin(theta_cs)
        p2 = np.array([p2_x, 0.0, p2_z], dtype=float)

        # 2. Circle-Circle Intersection for P3: Circle(P2, L_ss) & Circle(P5, R_53)
        sol1, sol2 = circle_circle_intersection_2d(p2, self.l_ss, self.p5, self.r_53)
        p3_xz = sol1 if sol1[1] >= sol2[1] else sol2
        p3 = np.array([p3_xz[0], 0.0, p3_xz[1]], dtype=float)

        # 3. Rocker-Yoke Pivot P4 on Rocker Body (Rotating about P5)
        v53 = p3_xz - self.p5[[0, 2]]
        u53 = v53 / np.linalg.norm(v53)
        cos_b, sin_b = np.cos(self.beta_rocker), np.sin(self.beta_rocker)
        u54 = np.array([cos_b * u53[0] - sin_b * u53[1], sin_b * u53[0] + cos_b * u53[1]], dtype=float)
        p4_xz = self.p5[[0, 2]] + self.r_54 * u54
        p4 = np.array([p4_xz[0], 0.0, p4_xz[1]], dtype=float)

        # 4. Rear Axle P1 and Dropout Upper Kink P12 on Seatstay Body (Attached to P2-P3)
        v23 = p3_xz - p2[[0, 2]]
        u23 = v23 / np.linalg.norm(v23)
        cos_g, sin_g = np.cos(self.gamma_ra), np.sin(self.gamma_ra)
        u21 = np.array([cos_g * u23[0] - sin_g * u23[1], sin_g * u23[0] + cos_g * u23[1]], dtype=float)
        p1_xz = p2[[0, 2]] + self.r_21 * u21
        p1 = np.array([p1_xz[0], 0.0, p1_xz[1]], dtype=float)

        cos_g12, sin_g12 = np.cos(self.gamma_12), np.sin(self.gamma_12)
        u2_12 = np.array([cos_g12 * u23[0] - sin_g12 * u23[1], sin_g12 * u23[0] + cos_g12 * u23[1]], dtype=float)
        p12_xz = p2[[0, 2]] + self.r_2_12 * u2_12
        p12 = np.array([p12_xz[0], 0.0, p12_xz[1]], dtype=float)

        # 5. Shock Lower Eyelet P6 along Line of Action (P4 -> P7)
        v47 = self.p7[[0, 2]] - p4_xz
        d47 = float(np.linalg.norm(v47))
        u47 = v47 / d47
        p6_xz = p4_xz + self.l_yoke * u47
        p6 = np.array([p6_xz[0], 0.0, p6_xz[1]], dtype=float)

        # 7. Transmission angle
        cos_mu = float(np.dot(v23, v53) / (np.linalg.norm(v23) * np.linalg.norm(v53)))
        transmission_angle_deg = float(np.degrees(np.arccos(np.clip(cos_mu, -1.0, 1.0))))

        return p1, p2, p3, p4, p6, p12, u47, d47, transmission_angle_deg

    def solve_state_from_chainstay_angle(self, theta_cs: float) -> Dict[str, Any]:
        """
        Solves the complete 3D kinematic state of the Horst-Link rear suspension
        given the chainstay rotation angle theta_cs (in radians).
        """
        p1, p2, p3, p4, p6, p12, u47, d47, transmission_angle_deg = self._solve_pivots_from_theta_cs(theta_cs)

        shock_length = float(d47 - self.l_yoke)
        shock_stroke = float(self.l_shock_0 - shock_length)
        wheel_travel = float(p1[2] - self.p1_0[2])

        lr_instantaneous = self._compute_instantaneous_lr(
            theta_cs=theta_cs,
            p1=p1,
            p2=p2,
            p3=p3,
            p4=p4,
            u47=u47,
        )

        link_errors = {
            "chainstay": float(abs(np.linalg.norm(p2 - self.p0) - self.l_cs)),
            "seatstay": float(abs(np.linalg.norm(p3 - p2) - self.l_ss)),
            "rocker_p5_p3": float(abs(np.linalg.norm(p3 - self.p5) - self.r_53)),
            "rocker_p5_p4": float(abs(np.linalg.norm(p4 - self.p5) - self.r_54)),
            "axle_offset": float(abs(np.linalg.norm(p1 - p2) - self.r_21)),
            "dropout_upper": float(abs(np.linalg.norm(p12 - p2) - self.r_2_12)),
            "dropout_height": float(abs(np.linalg.norm(p1 - p12) - self.l_dropout_height)),
            "yoke": float(abs(np.linalg.norm(p6 - p4) - self.l_yoke)),
        }

        return {
            "P0": self.p0,
            "P1": p1,
            "P2": p2,
            "P3": p3,
            "P4": p4,
            "P5": self.p5,
            "P6": p6,
            "P7": self.p7,
            "P12": p12,
            "P_FA": self.p_fa,
            "P11": self.p11,
            "BB": self.bb,
            # State scalars
            "theta_cs": float(theta_cs),
            "theta_cs_deg": float(np.degrees(theta_cs)),
            "wheel_travel": wheel_travel,
            "shock_stroke": shock_stroke,
            "shock_length": shock_length,
            "transmission_angle_deg": transmission_angle_deg,
            "leverage_ratio": lr_instantaneous,
            "link_errors": link_errors,
            "max_link_error": max(link_errors.values()),
        }

    def _compute_instantaneous_lr(
        self,
        theta_cs: float,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
        p4: np.ndarray,
        u47: np.ndarray,
    ) -> float:
        """
        Computes the instantaneous analytical leverage ratio d(wheel_travel) / d(shock_stroke)
        via 2D planar velocity kinematics (Jacobian virtual work formulation).
        """
        v2 = np.array([-self.l_cs * np.sin(theta_cs), self.l_cs * np.cos(theta_cs)])

        r23 = p3[[0, 2]] - p2[[0, 2]]
        r53 = p3[[0, 2]] - self.p5[[0, 2]]
        r54 = p4[[0, 2]] - self.p5[[0, 2]]
        r21 = p1[[0, 2]] - p2[[0, 2]]

        cross_53_23 = -r53[1] * r23[0] + r53[0] * r23[1]
        if abs(cross_53_23) < 1e-12:
            return 0.0

        omega_roc = float(np.dot(v2, r23) / cross_53_23)
        v3 = omega_roc * np.array([-r53[1], r53[0]])

        omega_ss = float(
            (-(v3[0] - v2[0]) * r23[1] + (v3[1] - v2[1]) * r23[0]) / (self.l_ss**2)
        )

        v1 = v2 + omega_ss * np.array([-r21[1], r21[0]])
        v4 = omega_roc * np.array([-r54[1], r54[0]])

        d_stroke = float(np.dot(u47, v4))
        d_travel_z = float(v1[1])

        if abs(d_stroke) < 1e-12:
            return 0.0

        return float(d_travel_z / d_stroke)

    def solve_state_from_wheel_travel(self, target_travel_z: float) -> Dict[str, Any]:
        """
        Solves the complete kinematic state matching a specified rear wheel vertical travel (in mm).
        """
        def objective(th: float) -> float:
            # Only the rear-axle height is needed to bracket the root; building the
            # full state dict (leverage-ratio Jacobian + link-error norms) per
            # iteration discards ~40% of the work.
            p1 = self._solve_pivots_from_theta_cs(th)[0]
            return float(p1[2] - self.p1_0[2]) - target_travel_z

        bracket_a = self.theta_cs_0 - 0.70  # ~40 deg upward rotation
        bracket_b = self.theta_cs_0 + 0.10  # ~5.7 deg downward extension

        theta_opt = brentq_pure(objective, bracket_a, bracket_b, xtol=1e-12)
        return self.solve_state_from_chainstay_angle(theta_opt)

    def solve_trajectory(
        self,
        n_points: int = 101,
        max_travel: float = 180.0,
    ) -> Dict[str, Any]:
        """
        Evaluates the suspension kinematics trajectory across uniform wheel travel steps.
        """
        travel_array = np.linspace(0.0, max_travel, n_points)
        states = [self.solve_state_from_wheel_travel(float(z)) for z in travel_array]

        wheel_travel = np.array(travel_array, dtype=float)
        shock_stroke = np.array([st["shock_stroke"] for st in states], dtype=float)
        shock_length = np.array([st["shock_length"] for st in states], dtype=float)
        leverage_ratio = np.array([st["leverage_ratio"] for st in states], dtype=float)
        transmission_angle_deg = np.array([st["transmission_angle_deg"] for st in states], dtype=float)
        axle_path_x = np.array([st["P1"][0] for st in states], dtype=float)
        axle_path_z = np.array([st["P1"][2] for st in states], dtype=float)
        chainstay_angle_deg = np.array([st["theta_cs_deg"] for st in states], dtype=float)
        link_length_errors = np.array([st["max_link_error"] for st in states], dtype=float)

        overall_max_link_error = float(np.max(link_length_errors))
        lr_init = float(leverage_ratio[0])
        lr_final = float(leverage_ratio[-1])
        progressivity_pct = float((lr_init - lr_final) / lr_init * 100.0)

        point_names = ["P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P12", "P_FA", "P11"]
        arrays = {k: np.array([st[k] for st in states]) for k in point_names}

        return {
            "wheel_travel": wheel_travel,
            "shock_stroke": shock_stroke,
            "shock_length": shock_length,
            "leverage_ratio": leverage_ratio,
            "transmission_angle_deg": transmission_angle_deg,
            "axle_path_x": axle_path_x,
            "axle_path_z": axle_path_z,
            "chainstay_angle_deg": chainstay_angle_deg,
            "link_length_errors": link_length_errors,
            "max_link_error": overall_max_link_error,
            "initial_leverage_ratio": lr_init,
            "final_leverage_ratio": lr_final,
            "progressivity_pct": progressivity_pct,
            "max_stroke": float(shock_stroke[-1]),
            "P0": arrays["P0"],
            "P1": arrays["P1"],
            "P2": arrays["P2"],
            "P3": arrays["P3"],
            "P4": arrays["P4"],
            "P5": arrays["P5"],
            "P6": arrays["P6"],
            "P7": arrays["P7"],
            "P12": arrays["P12"],
            "P_FA": arrays["P_FA"],
            "P11": arrays["P11"],
            "states": states,
        }
