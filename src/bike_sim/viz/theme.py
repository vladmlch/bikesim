"""
Visualization Styling, Themes, and Matplotlib Setup.

Provides standard palettes and lazy Matplotlib initialization to avoid headless environment issues.
"""

import os
from typing import Any, Tuple


def setup_matplotlib() -> Tuple[Any, Any]:
    """Lazily configure and import matplotlib to avoid import-time side effects."""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_cache")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as patches
    import matplotlib.pyplot as plt
    return plt, patches


# Color Palettes
LINKAGE_COLORS = {
    "frame": "#2C3E50",
    "chainstay": "#27AE60",
    "seatstay": "#E67E22",
    "rocker": "#C0392B",
    "yoke": "#8E44AD",
    "shock": "#2980B9",
    "fork": "#7F8C8D",
    "ground": "#BDC3C7",
    "wheel": "#95A5A6",
    "grid": "#D5D8DC",
}

COMPARISON_COLORS = {
    "frame": "#34495E",
    "uncompressed": "#2980B9",
    "compressed": "#E74C3C",
    "arc": "#95A5A6",
    "ground": "#BDC3C7",
    "frame_main": "#1C2833",
    "frame_dark": "#1A252F",
    "frame_gray": "#5D6D7E",
}
