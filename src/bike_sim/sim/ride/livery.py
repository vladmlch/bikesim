"""
Ride-Mode Model Livery.

The two things an interactive session changes about a *compiled* model rather than about the
run: whether the rider's 80 kg is present, and whether the yellow pivot markers are visible.
Both are applied in place, because a MuJoCo passive viewer holds the model and data it was
launched with and cannot be handed replacements.

**The rider's values are read from the builder, not written here.** Toggling the rider off has
to reproduce exactly what `generate_mujoco_xml(include_rider=False)` compiles to -- frame mass,
inertial frame, diagonal inertia, and the three rider capsules' size, colour and collision
flags. Those are therefore taken from a model compiled with that flag, once per state, instead
of being restated as constants that would silently drift from the builder.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple

import mujoco
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml

# Rider geoms whose visibility and collision follow the rider toggle.
RIDER_GEOMS: Tuple[str, ...] = ("geom_rider_torso", "geom_rider_legs", "geom_rider_arms")

MARKER_GEOM_PREFIX = "marker_"

# Size a hidden marker is shrunk to. Zero alpha alone leaves the sphere in the collision and
# bounding-volume bookkeeping at full size; the markers carry no collision, but the test stand
# shrinks them too and the two modes should hide them the same way.
HIDDEN_MARKER_SIZE_M = 0.0001


@dataclass(frozen=True)
class RiderVariant:
    """
    Everything the rider toggle has to write into a compiled model.

    Attributes:
        frame_mass: Mass of the `frame` body, in kg.
        frame_ipos: Inertial-frame position of the `frame` body.
        frame_inertia: Diagonal inertia of the `frame` body.
        geoms: Per-geom `(size, rgba, contype, conaffinity)` for the rider capsules.
    """

    frame_mass: float
    frame_ipos: np.ndarray
    frame_inertia: np.ndarray
    geoms: Dict[str, Tuple[np.ndarray, np.ndarray, int, int]]


class ModelLivery:
    """
    Applies the rider and marker toggles to one compiled ride model, in place.

    Rider variants are compiled lazily: a session that never touches the rider toggle never
    pays for the second compile.
    """

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
        self._variants: Dict[bool, RiderVariant] = {}

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

    def set_rider(self, data: mujoco.MjData, include_rider: bool) -> None:
        """
        Writes one rider state's mass and geometry into the model.

        The caller is responsible for re-solving the starting equilibrium afterwards: changing
        80 kg of a 104 kg system means the state the model was in is no longer one.

        Args:
            data: Simulation state, needed to recompute the model's derived constants.
            include_rider: Whether the rider should be present.

        Raises:
            ValueError: If the model has no `frame` body.
        """
        variant = self.variant(include_rider)
        fid = body_id(self.model, "frame")
        self.model.body_mass[fid] = variant.frame_mass
        self.model.body_ipos[fid] = variant.frame_ipos
        self.model.body_inertia[fid] = variant.frame_inertia

        for name, (size, rgba, contype, conaffinity) in variant.geoms.items():
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if gid < 0:
                continue
            self.model.geom_size[gid] = size
            self.model.geom_rgba[gid] = rgba
            self.model.geom_contype[gid] = contype
            self.model.geom_conaffinity[gid] = conaffinity

        mujoco.mj_setConst(self.model, data)

    def variant(self, include_rider: bool) -> RiderVariant:
        """
        Returns the compiled mass and geometry of one rider state, compiling it on first use.

        Args:
            include_rider: Rider state to describe.

        Returns:
            The variant's frame mass properties and rider geom livery.

        Raises:
            ValueError: If the compiled model has no `frame` body.
        """
        if include_rider not in self._variants:
            model = mujoco.MjModel.from_xml_string(
                generate_mujoco_xml(
                    specs=self.specs,
                    solver=self.solver,
                    mode="ride",
                    include_rider=include_rider,
                )
            )
            fid = body_id(model, "frame")
            geoms: Dict[str, Tuple[np.ndarray, np.ndarray, int, int]] = {}
            for name in RIDER_GEOMS:
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                if gid < 0:
                    continue
                geoms[name] = (
                    np.array(model.geom_size[gid], copy=True),
                    np.array(model.geom_rgba[gid], copy=True),
                    int(model.geom_contype[gid]),
                    int(model.geom_conaffinity[gid]),
                )
            self._variants[include_rider] = RiderVariant(
                frame_mass=float(model.body_mass[fid]),
                frame_ipos=np.array(model.body_ipos[fid], copy=True),
                frame_inertia=np.array(model.body_inertia[fid], copy=True),
                geoms=geoms,
            )
        return self._variants[include_rider]


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
    "RiderVariant",
    "body_id",
    "RIDER_GEOMS",
    "MARKER_GEOM_PREFIX",
    "HIDDEN_MARKER_SIZE_M",
]
