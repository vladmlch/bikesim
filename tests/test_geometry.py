"""
Unit tests for bicycle geometry, frame specifications, and published invariants.
"""

import unittest
import numpy as np

from bike_sim.geometry.hardpoints import (
    compute_frame_angles,
    compute_front_axle,
    compute_headtube_points,
    compute_rear_axle,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver


class TestBicycleGeometry(unittest.TestCase):
    """Verifies analytical frame geometry, steering parameters, and fixed pivot points."""

    def setUp(self) -> None:
        self.specs = BikeSpecs()

    def test_wheelbase_and_bb_drop(self) -> None:
        """Verifies front/rear axle height (BB drop / Mullet) and exact wheelbase calculation."""
        p_fa = compute_front_axle(self.specs)
        p_ra = compute_rear_axle(self.specs, front_axle=p_fa)

        # Front axle sits at Z = bb_drop (22.5 mm above BB origin Z=0)
        self.assertAlmostEqual(
            float(p_fa[2]),
            22.5,
            places=6,
            msg=f"Front axle Z ({p_fa[2]}) != 22.5 mm",
        )
        # Rear axle in Mullet (27.5" rear, 352mm vs 29" front 372mm) sits at Z = 2.5 mm
        self.assertAlmostEqual(
            float(p_ra[2]),
            2.5,
            places=6,
            msg=f"Rear axle Z ({p_ra[2]}) != 2.5 mm",
        )

        # Y coordinates should be on the centerline (0.0 mm)
        self.assertAlmostEqual(float(p_fa[1]), 0.0, places=6)
        self.assertAlmostEqual(float(p_ra[1]), 0.0, places=6)

        # Wheelbase = P_FA[X] - P_RA[X] == specs.wheelbase (exact within 1e-4 mm)
        wheelbase_computed = float(p_fa[0] - p_ra[0])
        self.assertAlmostEqual(
            wheelbase_computed,
            self.specs.wheelbase,
            places=4,
            msg=f"Wheelbase ({wheelbase_computed}) != {self.specs.wheelbase} mm",
        )

        # Distance BB -> Rear Axle P_RA should be exactly 447.50 mm
        dist_bb_ra = float(np.linalg.norm(p_ra))
        self.assertAlmostEqual(
            dist_bb_ra,
            447.50,
            delta=0.01,
            msg=f"BB -> Rear Axle distance ({dist_bb_ra:.3f} mm) != 447.50 mm",
        )

    def test_steering_geometry_and_offset(self) -> None:
        """Verifies top headtube Reach/Stack, head angle, fork offset, A2C, and trail metrics."""
        p11, p_ht_bot = compute_headtube_points(self.specs)

        # Top headtube P11 corresponds to Reach (X=480.0 mm) and Stack (Z=646.0 mm)
        self.assertAlmostEqual(
            float(p11[0]),
            480.0,
            places=6,
            msg=f"P11 Reach ({p11[0]}) != 480.0 mm",
        )
        self.assertAlmostEqual(
            float(p11[1]),
            0.0,
            places=6,
            msg=f"P11 Lateral Y ({p11[1]}) != 0.0 mm",
        )
        self.assertAlmostEqual(
            float(p11[2]),
            646.0,
            places=6,
            msg=f"P11 Stack ({p11[2]}) != 646.0 mm",
        )

        # Head angle and fork offset specifications
        self.assertAlmostEqual(float(self.specs.head_angle_deg), 64.0, places=6)
        self.assertAlmostEqual(float(self.specs.fork_offset), 44.0, places=6)

        # Compute trail metrics
        trail_metrics = compute_trail(self.specs)

        # Axle-to-Crown distance ≈ 595.2 mm ± 1.0 mm
        self.assertAlmostEqual(
            trail_metrics["axle_to_crown"],
            595.2,
            delta=1.0,
            msg=f"Axle-to-Crown ({trail_metrics['axle_to_crown']:.2f} mm) outside expected 595.2 ± 1.0 mm",
        )

        # Ground Trail ≈ 132.5 mm ± 0.5 mm
        self.assertAlmostEqual(
            trail_metrics["ground_trail"],
            132.5,
            delta=0.5,
            msg=f"Ground Trail ({trail_metrics['ground_trail']:.2f} mm) outside expected 132.5 ± 0.5 mm",
        )

        # Mechanical Trail ≈ 119.1 mm ± 0.5 mm
        self.assertAlmostEqual(
            trail_metrics["mechanical_trail"],
            119.1,
            delta=0.5,
            msg=f"Mechanical Trail ({trail_metrics['mechanical_trail']:.2f} mm) outside expected 119.1 ± 0.5 mm",
        )

    def test_fixed_frame_points(self) -> None:
        """Verifies existence and numerical coordinates of all fixed frame reference points."""
        fixed = get_fixed_frame_points(self.specs)
        required_keys = ["P0", "P5", "P7", "P8", "P9", "P10", "P11", "P_HT_bot", "BB"]

        for key in required_keys:
            self.assertIn(key, fixed, msg=f"Missing fixed point key: {key}")
            self.assertIsInstance(
                fixed[key],
                np.ndarray,
                msg=f"Fixed point {key} is not a np.ndarray",
            )
            self.assertEqual(
                fixed[key].shape,
                (3,),
                msg=f"Fixed point {key} shape is {fixed[key].shape}, expected (3,)",
            )

        # Check explicit reference coordinate values
        np.testing.assert_allclose(
            fixed["P0"], [-44.902156, 0.0, 46.392796], atol=1e-3, err_msg="P0 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P5"], [41.321097, 0.0, 184.456762], atol=1e-3, err_msg="P5 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P7"], [173.220585, 0.0, 403.650931], atol=1e-3, err_msg="P7 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P8"], [410.0, 0.0, 635.0], atol=1e-3, err_msg="P8 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P9"], [-85.0, 0.0, 490.0], atol=1e-3, err_msg="P9 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P10"], [-60.0, 0.0, 400.0], atol=1e-3, err_msg="P10 mismatch"
        )
        np.testing.assert_allclose(
            fixed["P11"], [480.0, 0.0, 646.0], atol=1e-3, err_msg="P11 mismatch"
        )
        np.testing.assert_allclose(
            fixed["BB"], [0.0, 0.0, 0.0], atol=1e-6, err_msg="BB mismatch"
        )

        # Bottom headtube P_HT_bot = P11 + headtube_length * u_steer
        theta = np.radians(self.specs.head_angle_deg)
        expected_ht_bot = np.array(
            [
                480.0 + self.specs.headtube_length * np.cos(theta),
                0.0,
                646.0 - self.specs.headtube_length * np.sin(theta),
            ]
        )
        np.testing.assert_allclose(
            fixed["P_HT_bot"],
            expected_ht_bot,
            atol=1e-3,
            err_msg="P_HT_bot mismatch",
        )

    def test_frame_angles(self) -> None:
        """Verifies frame tube inclination angles and Angle 1."""
        angles = compute_frame_angles(self.specs)

        # Tube 1 inclination angle with X ≈ 26.57°
        self.assertAlmostEqual(angles["angle_x_tube1_deg"], 26.57, delta=1.5)

        # Tube 2 inclination angle with X ≈ 45.30°
        self.assertAlmostEqual(angles["angle_x_tube2_deg"], 45.30, delta=0.5)

        self.assertAlmostEqual(angles["effective_seat_tube_angle_deg"], 74.476, delta=0.05)
        self.assertAlmostEqual(angles["angle_1_deg"], 78.959, delta=0.05)


class TestPublishedGeometryInvariants(unittest.TestCase):
    """
    Locks the published geometry table. These values are user-fixed: a future
    hardpoint refit must never move them silently.
    """

    def setUp(self) -> None:
        self.specs = BikeSpecs()
        self.solver = HorstLinkageSolver(self.specs)

    def test_frame_table_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.reach, 480.0, places=6)
        self.assertAlmostEqual(self.specs.stack, 646.0, places=6)
        self.assertAlmostEqual(self.specs.head_angle_deg, 64.0, places=6)
        self.assertAlmostEqual(self.specs.effective_seat_angle_deg, 77.0, places=6)
        self.assertAlmostEqual(self.specs.bb_drop, 22.5, places=6)
        self.assertAlmostEqual(self.specs.wheelbase, 1280.55, places=6)

    def test_wheel_and_shock_hardware_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.front_wheel_radius, 372.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_radius, 352.0, places=6)
        self.assertAlmostEqual(self.specs.shock_eye_to_eye, 205.0, places=6)
        self.assertAlmostEqual(self.specs.shock_stroke, 65.0, places=6)
        self.assertAlmostEqual(self.specs.fork_travel, 180.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_travel, 180.0, places=6)

    def test_chainstay_length_is_447_5(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        chainstay = float(np.linalg.norm(st0["P1"] - np.zeros(3)))
        self.assertAlmostEqual(chainstay, 447.5, delta=0.1)

    def test_full_travel_consumes_exactly_the_65mm_stroke(self) -> None:
        traj = self.solver.solve_trajectory(n_points=101, max_travel=180.0)
        self.assertAlmostEqual(traj["max_stroke"], 65.0, delta=0.01)

    def test_shock_eye_to_eye_is_205_at_full_extension(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        e2e = float(np.linalg.norm(st0["P7"] - st0["P6"]))
        self.assertAlmostEqual(e2e, 205.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
