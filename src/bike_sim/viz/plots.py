"""
Publication-Quality Plots for Bicycle Suspension Kinematics & Geometry (Facade).

Re-exports specialized plotters for complete backward compatibility:
- plot_linkage_geometry (from bike_sim.viz.geometry_plot)
- plot_leverage_ratio (from bike_sim.viz.leverage_plot)
- plot_suspension_compressed_comparison (from bike_sim.viz.comparison_plot)
- _setup_matplotlib (from bike_sim.viz.theme)
"""

from bike_sim.viz.theme import setup_matplotlib as _setup_matplotlib
from bike_sim.viz.geometry_plot import plot_linkage_geometry
from bike_sim.viz.leverage_plot import plot_leverage_ratio
from bike_sim.viz.comparison_plot import plot_suspension_compressed_comparison

__all__ = [
    "_setup_matplotlib",
    "plot_linkage_geometry",
    "plot_leverage_ratio",
    "plot_suspension_compressed_comparison",
]
