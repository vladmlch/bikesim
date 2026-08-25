"""
Unit tests for MuJoCo MJCF XML export, topology, loop closure, and JSON coordinate serialization.
"""

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import export_json


class TestMuJoCoExport(unittest.TestCase):
    """Verifies MuJoCo MJCF XML model export integrity, kinematic closure, and JSON serialization."""

    @classmethod
    def setUpClass(cls) -> None:
        # Solver construction is not cheap and the inputs are identical for every
        # test method, so build it once for the class.
        cls.specs = BikeSpecs()
        cls.solver = HorstLinkageSolver(cls.specs)

    def _compute_site_world_position(
        self, root: ET.Element, site_name: str
    ) -> Optional[np.ndarray]:
        """
        Helper method to compute the world position of a named site by traversing
        the MJCF XML body hierarchy from worldbody root.
        """
        def find_site_recursive(
            body_elem: ET.Element, parent_world_pos: np.ndarray
        ) -> Optional[np.ndarray]:
            raw_body_pos = body_elem.attrib.get("pos", "0 0 0").split()
            body_pos = np.array([float(x) for x in raw_body_pos], dtype=float)
            current_body_world_pos = parent_world_pos + body_pos

            for site_elem in body_elem.findall("site"):
                if site_elem.attrib.get("name") == site_name:
                    raw_site_pos = site_elem.attrib.get("pos", "0 0 0").split()
                    site_pos = np.array([float(x) for x in raw_site_pos], dtype=float)
                    return current_body_world_pos + site_pos

            for child_body in body_elem.findall("body"):
                res = find_site_recursive(child_body, current_body_world_pos)
                if res is not None:
                    return res
            return None

        worldbody = root.find("worldbody")
        if worldbody is None:
            return None

        for root_body in worldbody.findall("body"):
            res = find_site_recursive(root_body, np.zeros(3, dtype=float))
            if res is not None:
                return res
        return None

    def test_mujoco_xml_validity(self) -> None:
        """
        Verifies:
        - Generated XML parses cleanly with ElementTree.
        - Exactly 11 kinematic bodies are present.
        - Exactly 9 kinematic joints are present.
        - Loop closure equality constraints connect site_P3 and site_P7.
        - Near-zero initial constraint violation between sites in nominal pose.
        """
        xml_string = generate_mujoco_xml(specs=self.specs, solver=self.solver)
        self.assertIsInstance(xml_string, str)
        self.assertGreater(len(xml_string), 500)

        root = ET.fromstring(xml_string)
        self.assertEqual(root.tag, "mujoco")

        expected_bodies = {
            "frame",
            "steer",
            "fork_lower",
            "front_wheel",
            "chainstay",
            "seatstay",
            "rear_wheel",
            "rocker",
            "shock_yoke",
            "shock_shaft",
            "shock_body",
        }
        bodies_found = {b.attrib.get("name") for b in root.iter("body")}
        self.assertEqual(
            bodies_found,
            expected_bodies,
            msg=f"Body topology mismatch. Found: {bodies_found}, expected: {expected_bodies}",
        )
        self.assertEqual(len(bodies_found), 11)

        expected_joints = {
            "steer_joint",
            "fork_travel",
            "front_wheel_spin",
            "main_pivot",
            "horst_pivot",
            "rear_wheel_spin",
            "rocker_frame_pivot",
            "yoke_pivot",
            "shock_stroke",
        }
        joints_found = {j.attrib.get("name") for j in root.iter("joint")}
        self.assertEqual(
            joints_found,
            expected_joints,
            msg=f"Joint topology mismatch. Found: {joints_found}, expected: {expected_joints}",
        )
        self.assertEqual(len(joints_found), 9)

        equality_elem = root.find("equality")
        self.assertIsNotNone(equality_elem, msg="Missing <equality> section in XML")

        connect_elems = equality_elem.findall("connect")
        self.assertEqual(
            len(connect_elems),
            2,
            msg=f"Expected 2 <connect> equality constraints, found {len(connect_elems)}",
        )

        connect_map = {c.attrib.get("name"): c.attrib for c in connect_elems}
        self.assertIn("seatstay_rocker_joint", connect_map)
        self.assertEqual(connect_map["seatstay_rocker_joint"]["site1"], "site_P3_ss")
        self.assertEqual(connect_map["seatstay_rocker_joint"]["site2"], "site_P3_rocker")

        self.assertIn("shock_frame_joint", connect_map)
        self.assertEqual(connect_map["shock_frame_joint"]["site1"], "site_P7_shock")
        self.assertEqual(connect_map["shock_frame_joint"]["site2"], "site_P7")

        pos_p3_ss = self._compute_site_world_position(root, "site_P3_ss")
        pos_p3_rocker = self._compute_site_world_position(root, "site_P3_rocker")
        self.assertIsNotNone(pos_p3_ss, "site_P3_ss not found in XML hierarchy")
        self.assertIsNotNone(pos_p3_rocker, "site_P3_rocker not found in XML hierarchy")

        p3_violation_mm = float(np.linalg.norm(pos_p3_ss - pos_p3_rocker) * 1000.0)
        self.assertLess(
            p3_violation_mm,
            0.002,
            msg=f"Initial site_P3 constraint violation ({p3_violation_mm:.3e} mm) >= 2e-3 mm",
        )

        pos_p7_shock = self._compute_site_world_position(root, "site_P7_shock")
        pos_p7_frame = self._compute_site_world_position(root, "site_P7")
        self.assertIsNotNone(pos_p7_shock, "site_P7_shock not found in XML hierarchy")
        self.assertIsNotNone(pos_p7_frame, "site_P7 not found in XML hierarchy")

        p7_violation_mm = float(np.linalg.norm(pos_p7_shock - pos_p7_frame) * 1000.0)
        self.assertLess(
            p7_violation_mm,
            1e-6,
            msg=f"Initial site_P7 constraint violation ({p7_violation_mm:.3e} mm) >= 1e-6 mm",
        )

    def test_json_export_structure(self) -> None:
        """
        Verifies JSON export parsing, top-level keys, and numerical coordinates.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            json_file = Path(tmp_dir) / "test_coordinates.json"
            json_str = export_json(
                output_path=json_file, specs=self.specs, solver=self.solver
            )

            self.assertTrue(json_file.exists())
            self.assertIsInstance(json_str, str)

            data = json.loads(json_str)

            required_keys = [
                "coordinate_frame",
                "uncompressed_points_mm",
                "compressed_points_mm",
                "link_lengths_mm",
                "geometry_summary",
            ]
            for key in required_keys:
                self.assertIn(key, data, msg=f"JSON missing required key: {key}")

            self.assertEqual(
                data["coordinate_frame"]["origin"], "Bottom Bracket (BB) [0, 0, 0]"
            )

            for p_name in ["P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P12", "BB"]:
                self.assertIn(
                    p_name,
                    data["uncompressed_points_mm"],
                    msg=f"Uncompressed points missing {p_name}",
                )
                self.assertEqual(
                    len(data["uncompressed_points_mm"][p_name]),
                    3,
                    msg=f"Point {p_name} does not have 3 coordinates",
                )
                self.assertIn(
                    p_name,
                    data["compressed_points_mm"],
                    msg=f"Compressed points missing {p_name}",
                )

            self.assertIn("chainstay_p0_p2_mm", data["link_lengths_mm"])
            self.assertIn("seatstay_p2_p3_mm", data["link_lengths_mm"])
            self.assertIn("seatstay_tube_p3_p12_mm", data["link_lengths_mm"])
            self.assertIn("rocker_p5_p3_mm", data["link_lengths_mm"])
            self.assertIn("shock_stroke_mm", data["link_lengths_mm"])
            self.assertAlmostEqual(
                data["link_lengths_mm"]["shock_stroke_mm"], 65.0, delta=0.1
            )

            self.assertEqual(data["geometry_summary"]["reach_mm"], 480.0)
            self.assertEqual(data["geometry_summary"]["stack_mm"], 646.0)
            self.assertEqual(data["geometry_summary"]["wheelbase_mm"], self.specs.wheelbase)
            self.assertAlmostEqual(
                data["geometry_summary"]["progressivity_pct"], 27.995, delta=1.0
            )


class TestConfigAndInverseSolver(unittest.TestCase):
    """Verifies modular BikeConfig structure and inverse shock stroke kinematic solver."""

    def test_bike_config_composition(self) -> None:
        from bike_sim.config import BikeConfig
        from bike_sim.geometry.specs import FrameGeometrySpecs, SuspensionHardwareSpecs

        cfg = BikeConfig()
        self.assertIsInstance(cfg.geometry, FrameGeometrySpecs)
        self.assertIsInstance(cfg.suspension, SuspensionHardwareSpecs)
        self.assertEqual(cfg.geometry.reach, 480.0)
        self.assertEqual(cfg.suspension.fork_travel, 180.0)

    def test_shock_stroke_inverse_solver(self) -> None:
        specs = BikeSpecs()
        solver = HorstLinkageSolver(specs)

        for target_stroke in [0.0, 15.0, 30.0, 45.0, 60.0, 65.0]:
            st = solver.solve_state_from_shock_stroke(target_stroke)
            self.assertAlmostEqual(st["shock_stroke"], target_stroke, delta=0.08)
            self.assertTrue(2.40 <= st["leverage_ratio"] <= 3.36)


if __name__ == "__main__":
    unittest.main()
