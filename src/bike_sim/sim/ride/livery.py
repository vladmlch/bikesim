"""
Ride-Mode Model Livery.

The one thing an interactive session changes about a *compiled* model rather than about the
run: whether the yellow pivot markers are visible. It is applied in place, because a MuJoCo
passive viewer holds the model and data it was launched with and cannot be handed
replacements.

The rider used to be toggled here too, by rewriting the `frame` body's mass. That worked only
while the rider was rigid mass inside `frame`; the seated rider is its own bodies and joints,
so the variant is fixed at compile time and chosen on the command line (`bike-ride --rider`).
"""

from typing import Dict, List, Tuple

import mujoco
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver

MARKER_GEOM_PREFIX = "marker_"

# Size a hidden marker is shrunk to. Zero alpha alone leaves the sphere in the collision and
# bounding-volume bookkeeping at full size; the markers carry no collision, but the test stand
# shrinks them too and the two modes should hide them the same way.
HIDDEN_MARKER_SIZE_M = 0.0001


class ModelLivery:
    """Applies the marker toggle to one compiled ride model, in place."""

    def __init__(self, model: mujoco.MjModel, specs: BikeSpecs, solver: HorstLinkageSolver) -> None:
        """
        Args:
            model: Compiled ride-mode model to mutate.
            specs: Bicycle geometry the model was built from.
            solver: Linkage solver the model was built from.
        """
        self.model = model
        self.specs = specs
        self.solver = solver
        self.marker_geom_ids, self._marker_sizes = _resolve_markers(model)

    @property
    def has_markers(self) -> bool:
        """Whether the model was compiled with the pivot-marker geoms at all."""
        return bool(self.marker_geom_ids)

    def set_markers(self, visible: bool) -> None:
        """
        Shows or hides the pivot markers.

        Args:
            visible: Whether the markers should be drawn.
        """
        for gid in self.marker_geom_ids:
            if visible:
                self.model.geom_rgba[gid, 3] = 1.0
                self.model.geom_size[gid] = self._marker_sizes[gid]
            else:
                self.model.geom_rgba[gid, 3] = 0.0
                self.model.geom_size[gid] = [HIDDEN_MARKER_SIZE_M] * 3


def _resolve_markers(model: mujoco.MjModel) -> Tuple[List[int], Dict[int, np.ndarray]]:
    """
    Finds the pivot-marker geoms and remembers their compiled sizes.

    Args:
        model: Compiled ride-mode model.

    Returns:
        Tuple of (marker geom ids, map from id to its compiled size vector).
    """
    ids: List[int] = []
    sizes: Dict[int, np.ndarray] = {}
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if name and name.startswith(MARKER_GEOM_PREFIX):
            ids.append(gid)
            sizes[gid] = np.array(model.geom_size[gid], copy=True)
    return ids, sizes


def body_id(model: mujoco.MjModel, name: str) -> int:
    """
    Resolves a named body's id.

    Args:
        model: Compiled MuJoCo model.
        name: Body name.

    Returns:
        The body's index.

    Raises:
        ValueError: If the model has no body of that name.
    """
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise ValueError(f"model has no body '{name}'; the ride livery needs it")
    return int(bid)


__all__ = [
    "ModelLivery",
    "body_id",
    "MARKER_GEOM_PREFIX",
    "HIDDEN_MARKER_SIZE_M",
]
