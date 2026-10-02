"""
Visualization & Reporting Subpackage.
"""

from bike_sim.viz.dyno_plot import plot_damper_dyno_curves
from bike_sim.viz.theme import setup_matplotlib, LINKAGE_COLORS, COMPARISON_COLORS
from bike_sim.viz.plots import (
    plot_leverage_ratio,
    plot_linkage_geometry,
    plot_suspension_compressed_comparison,
)
from bike_sim.viz.tables import POINT_DESCRIPTIONS, get_all_points, print_tables

__all__ = [
    "plot_damper_dyno_curves",
    "plot_leverage_ratio",
    "plot_linkage_geometry",
    "plot_suspension_compressed_comparison",
    "setup_matplotlib",
    "LINKAGE_COLORS",
    "COMPARISON_COLORS",
    "POINT_DESCRIPTIONS",
    "get_all_points",
    "print_tables",
]
