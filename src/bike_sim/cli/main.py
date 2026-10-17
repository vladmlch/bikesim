"""
Main CLI Orchestrator for Bicycle Kinematics, Analysis, and Simulation.
"""

import argparse
import os
from pathlib import Path

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.exporter import export_json, export_playground_models
from bike_sim.sim.playground import run_interactive_playground
from bike_sim.viz.dyno_plot import plot_damper_dyno_curves
from bike_sim.viz.plots import (
    plot_leverage_ratio,
    plot_linkage_geometry,
    plot_suspension_compressed_comparison,
)
from bike_sim.viz.tables import print_tables


def parse_args() -> argparse.Namespace:
    """
    Configures and parses CLI arguments.
    """
    parser = argparse.ArgumentParser(
        description="Bicycle Frame Geometry and Horst-Link Suspension Kinematics CLI Runner."
    )
    parser.add_argument(
        "--print-table",
        action="store_true",
        help="Print formatted ASCII tables of uncompressed and compressed hardpoint coordinates and geometry metrics.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate 3 publication-quality kinematic and geometry plots (PNG).",
    )
    parser.add_argument(
        "--plot-damper",
        action="store_true",
        help="Generate publication-quality RockShox ZEB Ultimate & Super Deluxe damper dyno curves.",
    )
    parser.add_argument(
        "--export-json",
        action="store_true",
        help="Export all hardpoint coordinates and geometry metadata to JSON.",
    )
    parser.add_argument(
        "--export-mujoco",
        action="store_true",
        help="Export complete MuJoCo MJCF XML models.",
    )
    parser.add_argument(
        "--playground",
        action="store_true",
        help="Launch interactive 2D MuJoCo suspension test stand viewer.",
    )
    parser.add_argument(
        "--output-plots-dir",
        default="output/plots",
        help="Directory where plot images will be saved (default: output/plots).",
    )
    parser.add_argument(
        "--output-models-dir",
        default="output/models",
        help="Directory where MJCF XML and JSON files will be saved (default: output/models).",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Execute all operations: print tables, generate plots, export JSON, and export MuJoCo models.",
    )

    args = parser.parse_args()

    if not (args.print_table or args.plot or args.plot_damper or args.export_json or args.export_mujoco or args.playground or args.all):
        args.print_table = True
        args.plot = True

    return args


def main() -> None:
    """Main execution function for the CLI runner."""
    args = parse_args()

    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)

    plots_dir = Path(args.output_plots_dir)
    models_dir = Path(args.output_models_dir)
    plots_dir.mkdir(parents=True, exist_ok=True)
    models_dir.mkdir(parents=True, exist_ok=True)

    do_all = args.all

    if do_all or args.print_table:
        print_tables(specs, solver)

    if do_all or args.plot:
        print("\n--- Generating Kinematic Plots ---")
        plot_linkage_geometry(specs, solver, str(plots_dir / "linkage_geometry.png"))
        plot_leverage_ratio(specs, solver, str(plots_dir / "leverage_ratio.png"))
        plot_suspension_compressed_comparison(specs, solver, str(plots_dir / "suspension_compressed_comparison.png"))

    if do_all or args.plot or args.plot_damper:
        print("\n--- Generating Damper Dyno Plots (RockShox ZEB & Super Deluxe) ---")
        plot_damper_dyno_curves(str(plots_dir / "damper_dyno_curves.png"))

    if do_all or args.export_json:
        print("\n--- Exporting Hardpoints to JSON ---")
        export_json(output_path=str(models_dir / "coordinates.json"), specs=specs, solver=solver)

    if do_all or args.export_mujoco:
        print("\n--- Exporting MuJoCo MJCF XML Models ---")
        export_playground_models(output_dir=str(models_dir), specs=specs, solver=solver)

    if args.playground:
        print("\n--- Launching MuJoCo 2D Suspension Test Stand ---")
        run_interactive_playground()


if __name__ == "__main__":
    main()
