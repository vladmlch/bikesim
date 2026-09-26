"""
Automated Golden Snapshot Regression Tests.

Guarantees that procedural MuJoCo MJCF XML builders and JSON hardpoint exporters
remain 100% byte-for-byte and numerically equivalent to verified golden baselines.
"""

import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import export_json, export_playground_models


class TestGoldenBaselines(unittest.TestCase):
    """
    Validates generated MJCF XML and JSON coordinates against canonical golden baselines.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.golden_dir = Path(__file__).parent / "golden"
        cls.specs = BikeSpecs()
        cls.solver = HorstLinkageSolver(cls.specs)

    def _assert_text_equal(self, actual: str, baseline_filename: str) -> None:
        baseline_path = self.golden_dir / baseline_filename
        self.assertTrue(
            baseline_path.exists(),
            msg=f"Golden baseline file {baseline_filename} does not exist in {self.golden_dir}",
        )
        expected = baseline_path.read_text(encoding="utf-8")
        self.assertEqual(
            actual,
            expected,
            msg=f"Generated output diverges from golden baseline {baseline_filename}",
        )

    def test_baseline_bike_model_xml(self) -> None:
        """Standard bike model XML matches baseline_bike_model.xml."""
        xml_content = generate_mujoco_xml(specs=self.specs, solver=self.solver, mode="standard")
        self._assert_text_equal(xml_content, "baseline_bike_model.xml")

        # Verify it parses as valid XML
        root = ET.fromstring(xml_content)
        self.assertEqual(root.tag, "mujoco")

    def test_baseline_bike_playground_xml(self) -> None:
        """Playground bike model XML matches baseline_bike_playground.xml."""
        xml_content = generate_mujoco_xml(
            specs=self.specs,
            solver=self.solver,
            mode="playground",
            debug_markers=True,
        )
        self._assert_text_equal(xml_content, "baseline_bike_playground.xml")
        root = ET.fromstring(xml_content)
        self.assertEqual(root.tag, "mujoco")

    def test_baseline_bike_playground_stand_xml(self) -> None:
        """Test stand mode model XML matches baseline_bike_playground_stand.xml."""
        xml_content = generate_mujoco_xml(specs=self.specs, solver=self.solver, mode="stand")
        self._assert_text_equal(xml_content, "baseline_bike_playground_stand.xml")
        root = ET.fromstring(xml_content)
        self.assertEqual(root.tag, "mujoco")

    def test_baseline_bike_ride_xml(self) -> None:
        """Ride mode model XML with the default (seated) rider matches baseline_bike_ride.xml."""
        xml_content = generate_mujoco_xml(
            specs=self.specs,
            solver=self.solver,
            mode="ride",
            rider="seated",
        )
        self._assert_text_equal(xml_content, "baseline_bike_ride.xml")
        root = ET.fromstring(xml_content)
        self.assertEqual(root.tag, "mujoco")

    def test_baseline_bike_ride_lumped_xml(self) -> None:
        """Ride mode model XML with the lumped rider matches baseline_bike_ride_lumped.xml."""
        xml_content = generate_mujoco_xml(
            specs=self.specs,
            solver=self.solver,
            mode="ride",
            include_rider=True,
        )
        self._assert_text_equal(xml_content, "baseline_bike_ride_lumped.xml")
        root = ET.fromstring(xml_content)
        self.assertEqual(root.tag, "mujoco")

    def test_baseline_coordinates_json(self) -> None:
        """Exported hardpoints JSON matches baseline_coordinates.json."""
        baseline_path = self.golden_dir / "baseline_coordinates.json"
        self.assertTrue(baseline_path.exists())
        expected_raw = baseline_path.read_text(encoding="utf-8")

        # Use export_json directly to a temp location
        import tempfile
        with tempfile.TemporaryDirectory() as tmp_dir:
            json_file = Path(tmp_dir) / "coordinates.json"
            actual_raw = export_json(output_path=json_file, specs=self.specs, solver=self.solver)

            self.assertEqual(actual_raw, expected_raw)

            # Also verify JSON structural equivalence
            actual_obj = json.loads(actual_raw)
            expected_obj = json.loads(expected_raw)
            self.assertEqual(actual_obj, expected_obj)

    def test_compiled_mujoco_models_validity(self) -> None:
        """Verifies all golden baseline XMLs compile without warnings or errors in MuJoCo runtime."""
        try:
            import mujoco
        except ImportError:
            self.skipTest("mujoco not installed")

        for baseline_name in [
            "baseline_bike_model.xml",
            "baseline_bike_playground.xml",
            "baseline_bike_playground_stand.xml",
            "baseline_bike_ride.xml",
            "baseline_bike_ride_lumped.xml",
        ]:
            path = self.golden_dir / baseline_name
            model = mujoco.MjModel.from_xml_path(str(path))
            self.assertIsNotNone(model)
            data = mujoco.MjData(model)
            self.assertIsNotNone(data)


if __name__ == "__main__":
    unittest.main()
