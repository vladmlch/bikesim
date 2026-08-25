"""
Calibrated extraction of suspension pivot coordinates from the reference photograph.

The reference product shot highlights every pivot bolt in red. Those clusters are
segmented, then mapped from pixels to millimetres in the MuJoCo bike frame
(+X forward, +Z up, origin at the bottom bracket).

Calibration uses two anchors only: the BB cluster as origin and the two axle
clusters as a known wheelbase. BB drop is deliberately NOT used to fit the scale,
so reproducing it is an independent correctness check.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
from PIL import Image

# Red-bolt segmentation thresholds (tuned against the reference shot).
RED_MIN = 110
RED_OVER_GREEN = 55
RED_OVER_BLUE = 55
MIN_CLUSTER_AREA = 25

WHEELBASE_MM = 1280.55
BB_DROP_CHECK_MM = 22.5
BB_DROP_TOLERANCE_MM = 1.0

# Ruling R1: clusters within this pixel radius of either axle are hub hardware
# (derailleur-hanger bolt, cassette bolts, etc.), not the BB. The derailleur
# bolt sits only 5 px lower than the true BB in the reference photo, which
# would otherwise win the naive "lowest interior cluster" rule; it is 43 px
# from the rear axle, so this exclusion radius drops it while keeping the BB
# (which is far from both axles) eligible.
AXLE_EXCLUSION_RADIUS_PX = 150.0

DEFAULT_PHOTO = Path(__file__).resolve().parent.parent / "docs" / "reference" / "bulls_sonic_evo_side.jpg"
DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "docs" / "reference" / "bulls_reference_points.json"


@dataclass(frozen=True)
class Calibration:
    """Pixel-to-millimetre mapping for the reference photograph."""

    origin_px: Tuple[float, float]
    mm_per_px: float

    def to_mm(self, px_x: float, px_y: float) -> Tuple[float, float]:
        """Maps a pixel coordinate to (X, Z) in mm, BB origin, +X forward, +Z up."""
        x_mm = (px_x - self.origin_px[0]) * self.mm_per_px
        z_mm = (self.origin_px[1] - px_y) * self.mm_per_px
        return float(x_mm), float(z_mm)


def detect_red_clusters(image_path: str | Path) -> List[Dict[str, Any]]:
    """
    Segments red pivot-bolt markers and returns their centroids, largest first.

    Args:
        image_path: Path to the reference photograph.

    Returns:
        List of {"area": int, "px_x": float, "px_y": float}, sorted by descending area.
    """
    rgb = np.asarray(Image.open(image_path).convert("RGB")).astype(int)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    mask = (r > RED_MIN) & (r - g > RED_OVER_GREEN) & (r - b > RED_OVER_BLUE)

    height, width = mask.shape
    seen = np.zeros_like(mask)
    clusters: List[Dict[str, Any]] = []

    for y in range(height):
        for x in range(width):
            if not mask[y, x] or seen[y, x]:
                continue
            queue = deque([(y, x)])
            seen[y, x] = True
            pixels: List[Tuple[int, int]] = []
            while queue:
                cy, cx = queue.popleft()
                pixels.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            queue.append((ny, nx))
            if len(pixels) >= MIN_CLUSTER_AREA:
                clusters.append(
                    {
                        "area": len(pixels),
                        "px_x": float(np.mean([p[1] for p in pixels])),
                        "px_y": float(np.mean([p[0] for p in pixels])),
                    }
                )

    clusters.sort(key=lambda c: -c["area"])
    return clusters


def calibrate(clusters: List[Dict[str, Any]]) -> Calibration:
    """
    Derives the pixel-to-millimetre mapping from the BB and axle clusters.

    The BB is the lowest cluster near the crank, among clusters that are not
    hub hardware; the axles are the extreme left and right clusters. Their
    separation is the known wheelbase.
    """
    rear = min(clusters, key=lambda c: c["px_x"])
    front = max(clusters, key=lambda c: c["px_x"])
    span_px = float(np.hypot(front["px_x"] - rear["px_x"], front["px_y"] - rear["px_y"]))
    mm_per_px = WHEELBASE_MM / span_px

    def near_axle(c: Dict[str, Any]) -> bool:
        for axle in (rear, front):
            if np.hypot(c["px_x"] - axle["px_x"], c["px_y"] - axle["px_y"]) < AXLE_EXCLUSION_RADIUS_PX:
                return True
        return False

    # The BB cluster is the one closest to the crank spindle: among clusters
    # between the axles and away from hub hardware (see Ruling R1 / the
    # AXLE_EXCLUSION_RADIUS_PX comment above), it is the lowest in the image
    # (largest px_y).
    interior = [
        c for c in clusters
        if rear["px_x"] < c["px_x"] < front["px_x"] and not near_axle(c)
    ]
    bb = max(interior, key=lambda c: c["px_y"])

    return Calibration(origin_px=(bb["px_x"], bb["px_y"]), mm_per_px=mm_per_px)


def build_reference(image_path: str | Path = DEFAULT_PHOTO) -> Dict[str, Any]:
    """
    Extracts, calibrates and names every pivot target.

    Returns:
        Payload with calibration metadata and named (X, Z) targets in mm.

    Raises:
        ValueError: If the BB-drop self-check fails.
    """
    clusters = detect_red_clusters(image_path)
    cal = calibrate(clusters)

    measured = [
        {"area": c["area"], "mm": cal.to_mm(c["px_x"], c["px_y"]), "px": (c["px_x"], c["px_y"])}
        for c in clusters
    ]

    rear_axle = min(measured, key=lambda m: m["mm"][0])
    front_axle = max(measured, key=lambda m: m["mm"][0])

    # Name the linkage pivots by their measured position. Ordering is stable
    # because the pivots are well separated in the (X, Z) plane.
    def nearest(x_mm: float, z_mm: float) -> Dict[str, Any]:
        return min(measured, key=lambda m: (m["mm"][0] - x_mm) ** 2 + (m["mm"][1] - z_mm) ** 2)

    points = {
        "bb": (0.0, 0.0),
        "rear_axle": rear_axle["mm"],
        "front_axle": front_axle["mm"],
        "P0_candidate": nearest(-45.0, 45.9)["mm"],
        "P2_candidate": nearest(-400.4, -8.3)["mm"],
        "P3": nearest(-71.3, 206.4)["mm"],
        "P4": nearest(-39.4, 191.8)["mm"],
        "P5": nearest(41.9, 185.0)["mm"],
        "P6": nearest(29.6, 261.6)["mm"],
        "P7": nearest(171.4, 401.7)["mm"],
    }

    front_axle_z = points["front_axle"][1]
    if abs(front_axle_z - BB_DROP_CHECK_MM) > BB_DROP_TOLERANCE_MM:
        raise ValueError(
            f"Calibration self-check failed: front axle Z={front_axle_z:.2f} mm, "
            f"expected BB drop {BB_DROP_CHECK_MM} +/- {BB_DROP_TOLERANCE_MM} mm"
        )

    return {
        "source_image": str(Path(image_path).name),
        "calibration": {
            "origin_px": list(cal.origin_px),
            "mm_per_px": cal.mm_per_px,
            "wheelbase_mm": WHEELBASE_MM,
            "bb_drop_selfcheck_mm": front_axle_z,
        },
        "note": (
            "Pivot targets are MEASURED from red bolt markers. Front-triangle tube "
            "shapes are NOT measurable from this photograph (white-on-white) and are "
            "authored styling elsewhere."
        ),
        "points_mm": {k: [round(v[0], 4), round(v[1], 4)] for k, v in points.items()},
    }


def main() -> None:
    payload = build_reference()
    DEFAULT_OUTPUT.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote {DEFAULT_OUTPUT}")
    for name, (x, z) in payload["points_mm"].items():
        print(f"  {name:16s} X={x:9.2f}  Z={z:9.2f}")


if __name__ == "__main__":
    main()
