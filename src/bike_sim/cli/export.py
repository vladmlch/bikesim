"""
CLI Exporter for MuJoCo MJCF Models and JSON Hardpoints.
"""

import argparse
from pathlib import Path

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.exporter import export_json, export_playground_models


def main() -> None:
    """CLI Entrypoint for exporting MuJoCo models and coordinate data."""
    parser = argparse.ArgumentParser(description="Export MuJoCo MJCF XML models and JSON geometry data.")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="output/models",
        help="Target output directory for generated MJCF XML and JSON files (default: output/models).",
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Export only coordinates.json without generating XML models.",
    )
    parser.add_argument(
        "--xml-only",
        action="store_true",
        help="Export only MuJoCo XML models without exporting JSON data.",
    )

    args = parser.parse_args()
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)

    if not args.xml_only:
        json_path = out_dir / "coordinates.json"
        export_json(specs=specs, solver=solver, output_path=str(json_path))

    if not args.json_only:
        export_playground_models(output_dir=str(out_dir), specs=specs, solver=solver)


if __name__ == "__main__":
    main()
