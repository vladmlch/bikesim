"""
Kinematic Curves & Analysis Utilities.

Provides calculation of leverage ratio, anti-squat curves, instant centers, and axle paths.
"""

from typing import Any, Dict
import numpy as np

from bike_sim.kinematics.solver import HorstLinkageSolver


def compute_instant_centers(solver: HorstLinkageSolver, n_points: int = 101) -> Dict[str, np.ndarray]:
    """
    Computes the Instant Center (IC) trajectory of the rear axle across wheel travel.

    For a 4-bar linkage, the instant center of the coupler (seatstay) relative to
    the frame is the intersection of the line (P0-P2) and (P5-P3).

    Args:
        solver: HorstLinkageSolver instance.
        n_points: Number of travel samples.

    Returns:
        Dictionary with 'travel', 'ic_x', and 'ic_z' arrays.
    """
    traj = solver.solve_trajectory(n_points=n_points)
    travel = traj["wheel_travel"]
    p0 = traj["P0"]
    p2 = traj["P2"]
    p5 = traj["P5"]
    p3 = traj["P3"]

    ic_x = []
    ic_z = []

    for i in range(len(travel)):
        # Line 1: P0 -> P2
        # Line 2: P5 -> P3
        p0_i, p2_i = p0[i][[0, 2]], p2[i][[0, 2]]
        p5_i, p3_i = p5[i][[0, 2]], p3[i][[0, 2]]

        d1 = p2_i - p0_i
        d2 = p3_i - p5_i

        # Solve p0_i + t * d1 = p5_i + u * d2
        A = np.column_stack([d1, -d2])
        b = p5_i - p0_i

        try:
            params = np.linalg.solve(A, b)
            ic = p0_i + params[0] * d1
            ic_x.append(ic[0])
            ic_z.append(ic[1])
        except np.linalg.LinAlgError:
            ic_x.append(np.nan)
            ic_z.append(np.nan)

    return {
        "travel": travel,
        "ic_x": np.array(ic_x, dtype=float),
        "ic_z": np.array(ic_z, dtype=float),
    }
