"""
Unit tests for 4-bar Horst-link kinematics invariants, shock stroke, and leverage curves.
"""

import unittest
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver


class TestLinkageKinematics(unittest.TestCase):
    """Verifies Horst-link 4-bar kinematics invariants, shock stroke, and leverage curve."""

    def setUp(self) -> None:
        self.specs = BikeSpecs()
        self.solver = HorstLinkageSolver(self.specs)
        self.n_steps = 101
        self.max_travel = 180.0
        self.traj = self.solver.solve_trajectory(
            n_points=self.n_steps, max_travel=self.max_travel
        )

    def test_rigid_length_invariance(self) -> None:
        """
        Verifies that all rigid link lengths and interior link angles remain invariant
        across the complete 101-step suspension travel trajectory (0 to 180 mm).
        Max tolerance error must be < 10^-10 mm (or radians).
        """
        # Overall trajectory invariant error
        self.assertLess(
            self.traj["max_link_error"],
            1e-10,
            msg=f"Trajectory max link error ({self.traj['max_link_error']:.3e}) exceeds 1e-10 mm",
        )

        # Detailed per-state link length and angle verification
        for idx, st in enumerate(self.traj["states"]):
            p0 = st["P0"]
            p1 = st["P1"]
            p2 = st["P2"]
            p3 = st["P3"]
            p4 = st["P4"]
            p5 = st["P5"]
            p6 = st["P6"]

            # 1. Chainstay length: ||P2 - P0|| == L_CS
            l_cs_current = float(np.linalg.norm(p2 - p0))
            self.assertAlmostEqual(
                l_cs_current,
                self.solver.l_cs,
                delta=1e-10,
                msg=f"Step {idx}: Chainstay length changed by {abs(l_cs_current - self.solver.l_cs):.3e} mm",
            )

            # 2. Seatstay length: ||P3 - P2|| == L_SS
            l_ss_current = float(np.linalg.norm(p3 - p2))
            self.assertAlmostEqual(
                l_ss_current,
                self.solver.l_ss,
                delta=1e-10,
                msg=f"Step {idx}: Seatstay length changed by {abs(l_ss_current - self.solver.l_ss):.3e} mm",
            )

            # 3. Rear Axle offset: ||P1 - P2|| == R_21
            r_21_current = float(np.linalg.norm(p1 - p2))
            self.assertAlmostEqual(
                r_21_current,
                self.solver.r_21,
                delta=1e-10,
                msg=f"Step {idx}: Rear axle offset changed by {abs(r_21_current - self.solver.r_21):.3e} mm",
            )

            # 4. Rocker arm P5->P3: ||P3 - P5|| == R_53
            r_53_current = float(np.linalg.norm(p3 - p5))
            self.assertAlmostEqual(
                r_53_current,
                self.solver.r_53,
                delta=1e-10,
                msg=f"Step {idx}: Rocker P5-P3 changed by {abs(r_53_current - self.solver.r_53):.3e} mm",
            )

            # 5. Rocker arm P5->P4: ||P4 - P5|| == R_54
            r_54_current = float(np.linalg.norm(p4 - p5))
            self.assertAlmostEqual(
                r_54_current,
                self.solver.r_54,
                delta=1e-10,
                msg=f"Step {idx}: Rocker P5-P4 changed by {abs(r_54_current - self.solver.r_54):.3e} mm",
            )

            # 6. Rocker rigid interior angle beta
            v53 = p3[[0, 2]] - p5[[0, 2]]
            v54 = p4[[0, 2]] - p5[[0, 2]]
            beta_current = float(
                np.arctan2(v54[1], v54[0]) - np.arctan2(v53[1], v53[0])
            )
            angle_diff_rocker = (
                beta_current - self.solver.beta_rocker + np.pi
            ) % (2.0 * np.pi) - np.pi
            self.assertAlmostEqual(
                angle_diff_rocker,
                0.0,
                delta=1e-10,
                msg=f"Step {idx}: Rocker interior angle changed by {abs(angle_diff_rocker):.3e} rad",
            )

            # 7. Seatstay rigid interior angle gamma_ra
            v23 = p3[[0, 2]] - p2[[0, 2]]
            v21 = p1[[0, 2]] - p2[[0, 2]]
            gamma_current = float(
                np.arctan2(v21[1], v21[0]) - np.arctan2(v23[1], v23[0])
            )
            angle_diff_ss = (
                gamma_current - self.solver.gamma_ra + np.pi
            ) % (2.0 * np.pi) - np.pi
            self.assertAlmostEqual(
                angle_diff_ss,
                0.0,
                delta=1e-10,
                msg=f"Step {idx}: Seatstay triangle angle changed by {abs(angle_diff_ss):.3e} rad",
            )

            # 8. Dropout P12 invariants
            p12 = st["P12"]
            r_2_12_current = float(np.linalg.norm(p12 - p2))
            self.assertAlmostEqual(
                r_2_12_current,
                self.solver.r_2_12,
                delta=1e-10,
                msg=f"Step {idx}: Dropout P2-P12 changed by {abs(r_2_12_current - self.solver.r_2_12):.3e} mm",
            )
            l_dropout_h_current = float(np.linalg.norm(p1 - p12))
            self.assertAlmostEqual(
                l_dropout_h_current,
                self.solver.l_dropout_height,
                delta=1e-10,
                msg=f"Step {idx}: Dropout height P1-P12 changed by {abs(l_dropout_h_current - self.solver.l_dropout_height):.3e} mm",
            )

            # 9. Yoke length: ||P6 - P4|| == L_yoke
            l_yoke_current = float(np.linalg.norm(p6 - p4))
            self.assertAlmostEqual(
                l_yoke_current,
                self.solver.l_yoke,
                delta=1e-10,
                msg=f"Step {idx}: Yoke length changed by {abs(l_yoke_current - self.solver.l_yoke):.3e} mm",
            )

    def test_shock_stroke_and_travel(self) -> None:
        """
        Verifies:
        - At 0 mm travel: trunnion eye-to-eye = 205.00 mm ± 0.05 mm, stroke = 0.00 mm ± 0.05 mm.
        - At 180 mm travel: trunnion eye-to-eye = 140.00 mm ± 0.05 mm, stroke = 65.00 mm ± 0.05 mm.
        - Shock stroke is strictly monotonic with respect to rear wheel travel.
        """
        st_0 = self.traj["states"][0]
        st_180 = self.traj["states"][-1]

        # Initial uncompressed state (0 mm travel)
        self.assertAlmostEqual(
            st_0["shock_length"],
            205.00,
            delta=0.05,
            msg=f"Initial shock eye-to-eye ({st_0['shock_length']:.3f} mm) != 205.00 ± 0.05 mm",
        )
        self.assertAlmostEqual(
            st_0["shock_stroke"],
            0.00,
            delta=0.05,
            msg=f"Initial shock stroke ({st_0['shock_stroke']:.3f} mm) != 0.00 ± 0.05 mm",
        )

        # Fully compressed state (180 mm travel)
        self.assertAlmostEqual(
            st_180["shock_length"],
            140.00,
            delta=0.05,
            msg=f"Compressed shock eye-to-eye ({st_180['shock_length']:.3f} mm) != 140.00 ± 0.05 mm",
        )
        self.assertAlmostEqual(
            st_180["shock_stroke"],
            65.00,
            delta=0.05,
            msg=f"Compressed shock stroke ({st_180['shock_stroke']:.3f} mm) != 65.00 ± 0.05 mm",
        )

        # Strictly monotonic stroke progression
        stroke_diffs = np.diff(self.traj["shock_stroke"])
        self.assertTrue(
            bool(np.all(stroke_diffs > 0.0)),
            msg="Shock stroke is not strictly monotonically increasing with wheel travel",
        )

    def test_transmission_angle_safe_range(self) -> None:
        """
        Verifies that transmission angle mu(t) remains within [50.0°, 140.0°] across all 101 steps.
        Guarantees smooth mechanical transmission free from toggling, locking, or singularities.
        """
        transmission_angles = self.traj["transmission_angle_deg"]
        min_mu = float(np.min(transmission_angles))
        max_mu = float(np.max(transmission_angles))

        self.assertGreaterEqual(
            min_mu,
            50.0,
            msg=f"Minimum transmission angle ({min_mu:.2f}°) is below safe limit 50.0°",
        )
        self.assertLessEqual(
            max_mu,
            140.0,
            msg=f"Maximum transmission angle ({max_mu:.2f}°) is above safe limit 140.0°",
        )

    def test_leverage_ratio_progressive(self) -> None:
        """
        Characterizes the photo-fitted leverage ratio curve:
        - Initial leverage ratio: 3.3495 ± 0.02
        - Final leverage ratio: 2.4118 ± 0.02
        - Progressivity: 27.995% ± 0.10
        """
        lr_start = self.traj["initial_leverage_ratio"]
        lr_end = self.traj["final_leverage_ratio"]
        prog_pct = self.traj["progressivity_pct"]

        self.assertAlmostEqual(
            lr_start,
            3.3495,
            delta=0.02,
            msg=f"Initial leverage ratio ({lr_start:.4f}) outside 3.3495 ± 0.02",
        )

        self.assertAlmostEqual(
            lr_end,
            2.4118,
            delta=0.02,
            msg=f"Final leverage ratio ({lr_end:.4f}) outside 2.4118 ± 0.02",
        )

        self.assertAlmostEqual(
            prog_pct,
            27.995,
            delta=0.10,
            msg=f"Progressivity percentage ({prog_pct:.3f}%) outside 27.995 ± 0.10",
        )


if __name__ == "__main__":
    unittest.main()
