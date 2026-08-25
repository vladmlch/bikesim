"""
Ride-Mode Terrain Contact Queries.

One place that answers, per step, which of the tracked geoms are touching the road and with
what normal load. Cruise gating, rolling resistance, the virtual rider and the crash
detector all need that answer, and all four must see the *same* answer within a step, so
the query is made once per step and the snapshot is passed around.

The normal load comes from `mj_contactForce`, not from a static axle-load estimate. Rolling
resistance is proportional to the instantaneous load (docs/RIDE.md section 4), and the whole
reason for modelling it that way is that the load is not static: a G-out roughly doubles it
and a crest takes it to zero.

The forces read here are those of the last `mj_forward`/`mj_step`, so within one control step
they lag the state by a single 0.5 ms timestep. That is the normal cost of reading constraint
forces from a controller and is negligible against every time constant in the model.

**Single-step collision dropouts are bridged.** MuJoCo's sphere-heightfield collision
intermittently reports no contact at all for a wheel that is demonstrably loaded: the sphere
rests about 10 um into the surface, so whether a prism registers a contact is decided at the
edge of floating-point resolution. Measured over the `single_edge` run-up, one wheel drops
out on 13 % of steps and both drop out simultaneously on 1.7 % of them, for exactly one step
on flat road and for at most four in the aftermath of the square edge. Taken at face value
that makes the bike briefly airborne on flat ground, which gates the drive torque off and
fires the virtual rider. Each wheel's last known load is therefore held across a short gap;
see `CONTACT_DROPOUT_STEPS`.
"""

from dataclasses import dataclass
from typing import Dict, Tuple

import mujoco
import numpy as np

# Worldbody collision geoms that make up the ground: the road heightfield and the runaway
# catch plane far below it.
TERRAIN_GEOMS: Tuple[str, ...] = ("terrain", "catch_plane")

# Contact sphere of each wheel, and the bar that a crash puts into the dirt.
FRONT_CONTACT_GEOM = "geom_front_contact"
REAR_CONTACT_GEOM = "geom_rear_contact"
HANDLEBAR_GEOM = "geom_handlebar"

# Normal load below which a wheel counts as off the ground. MuJoCo registers a contact as
# soon as the geoms are within margin, so a wheel that has just left the road can still own
# a contact row carrying nothing. One newton is 0.1 % of the 1024 N static weight -- far
# below any load that transmits usable traction, far above solver noise.
CONTACT_LOAD_THRESHOLD_N = 1.0

# Consecutive unloaded steps a wheel is allowed before it counts as airborne. Measured
# dropouts are one step long on flat ground and at most four in the aftermath of the square
# edge; the genuine flight phase over that 90 mm edge lasts 220 steps. Ten steps -- 5 ms --
# is an order of magnitude clear of both, so it removes every dropout without blunting the
# start of a real flight by more than 2 % of its duration.
CONTACT_DROPOUT_STEPS = 10


@dataclass(frozen=True)
class TerrainContacts:
    """
    Snapshot of the bike's contact with the road at one instant.

    Attributes:
        front_load_n: Normal force on the front contact sphere, in newtons, held across a
            collision dropout shorter than `CONTACT_DROPOUT_STEPS`.
        rear_load_n: Normal force on the rear contact sphere, in newtons, held the same way.
        handlebar_load_n: Normal force on the handlebar geom, in newtons. Not held: a
            handlebar contact lasts far longer than a dropout, and a missed step only delays
            the crash detector by 0.5 ms.
    """

    front_load_n: float
    rear_load_n: float
    handlebar_load_n: float

    @property
    def front_in_contact(self) -> bool:
        """Whether the front wheel is carrying load."""
        return self.front_load_n > CONTACT_LOAD_THRESHOLD_N

    @property
    def rear_in_contact(self) -> bool:
        """Whether the rear wheel is carrying load."""
        return self.rear_load_n > CONTACT_LOAD_THRESHOLD_N

    @property
    def airborne(self) -> bool:
        """Whether both wheels are off the ground, which is the virtual rider's gate."""
        return not self.front_in_contact and not self.rear_in_contact

    @property
    def handlebar_in_contact(self) -> bool:
        """Whether the handlebar is touching the ground, which the crash detector treats as a crash."""
        return self.handlebar_load_n > CONTACT_LOAD_THRESHOLD_N


class TerrainContactQuery:
    """
    Sums the normal contact load on each tracked geom against the terrain.

    Geom ids are resolved once at construction; the per-step query is a single pass over
    `data.contact` with a reused force buffer, so it allocates nothing beyond the snapshot.

    The query is stateful, because bridging a collision dropout requires remembering the
    previous steps. Advance it once per simulation step -- querying the same state twice
    charges it twice against the dropout budget -- and `reset` it when the run restarts.
    """

    def __init__(self, model: mujoco.MjModel, dropout_steps: int = CONTACT_DROPOUT_STEPS) -> None:
        """
        Args:
            model: Compiled ride-mode model.
            dropout_steps: Consecutive unloaded steps a wheel may have before it is reported
                as airborne.

        Raises:
            ValueError: If `dropout_steps` is negative, or a tracked geom is missing.
        """
        if dropout_steps < 0:
            raise ValueError(f"dropout_steps must be non-negative, got {dropout_steps}")

        self.terrain_ids = frozenset(_geom_id(model, name) for name in TERRAIN_GEOMS)
        self.front_id = _geom_id(model, FRONT_CONTACT_GEOM)
        self.rear_id = _geom_id(model, REAR_CONTACT_GEOM)
        self.handlebar_id = _geom_id(model, HANDLEBAR_GEOM)

        self._force = np.zeros(6, dtype=float)
        self._front_load = _BridgedLoad(dropout_steps)
        self._rear_load = _BridgedLoad(dropout_steps)

    def query(self, model: mujoco.MjModel, data: mujoco.MjData) -> TerrainContacts:
        """
        Reads the current contact normal load on each tracked geom.

        Args:
            model: Compiled ride-mode model.
            data: Simulation state, after a `mj_forward` or `mj_step` has populated its
                contacts and constraint forces.

        Returns:
            The load on each tracked geom, with the two wheel loads held across a short
            collision dropout.
        """
        raw = self._raw_loads(model, data)
        return TerrainContacts(
            front_load_n=self._front_load.update(raw[self.front_id]),
            rear_load_n=self._rear_load.update(raw[self.rear_id]),
            handlebar_load_n=raw[self.handlebar_id],
        )

    def reset(self) -> None:
        """Discards the held loads, so a fresh run does not start off the previous one."""
        self._front_load.reset()
        self._rear_load.reset()

    def _raw_loads(self, model: mujoco.MjModel, data: mujoco.MjData) -> Dict[int, float]:
        """
        Sums the unfiltered normal load on each tracked geom for the current step.

        Args:
            model: Compiled ride-mode model.
            data: Simulation state with populated contacts.

        Returns:
            Normal load in newtons keyed by geom id. A wheel with several contact rows
            against adjacent heightfield prisms is summed over all of them: MuJoCo routinely
            loads only one row of such a group and leaves its siblings at exactly zero.
        """
        loads: Dict[int, float] = {self.front_id: 0.0, self.rear_id: 0.0, self.handlebar_id: 0.0}

        for i in range(data.ncon):
            contact = data.contact[i]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.terrain_ids:
                tracked = geom2
            elif geom2 in self.terrain_ids:
                tracked = geom1
            else:
                continue
            if tracked not in loads:
                continue
            mujoco.mj_contactForce(model, data, i, self._force)
            loads[tracked] += abs(float(self._force[0]))

        return loads


class _BridgedLoad:
    """
    One wheel's contact load, held across a run of steps that report nothing.

    Attributes:
        dropout_steps: Consecutive unloaded steps tolerated before the load is released.
        held_load_n: Load currently being reported, in newtons.
        unloaded_steps: Length of the current run of unloaded steps.
    """

    def __init__(self, dropout_steps: int) -> None:
        self.dropout_steps = int(dropout_steps)
        self.held_load_n = 0.0
        self.unloaded_steps = 0

    def update(self, raw_load_n: float) -> float:
        """
        Folds this step's raw load into the held value.

        Args:
            raw_load_n: Load summed from this step's contact rows, in newtons.

        Returns:
            The load to report for this step.
        """
        if raw_load_n > CONTACT_LOAD_THRESHOLD_N:
            self.held_load_n = raw_load_n
            self.unloaded_steps = 0
        else:
            self.unloaded_steps += 1
            if self.unloaded_steps > self.dropout_steps:
                self.held_load_n = 0.0
        return self.held_load_n

    def reset(self) -> None:
        """Clears the held load and the dropout counter."""
        self.held_load_n = 0.0
        self.unloaded_steps = 0


def _geom_id(model: mujoco.MjModel, geom_name: str) -> int:
    """
    Resolves a named geom's id.

    Args:
        model: Compiled MuJoCo model.
        geom_name: Name of the geom to resolve.

    Returns:
        The geom's index in the model.

    Raises:
        ValueError: If the model has no geom of that name, which would leave the contact
            query silently reporting zero load rather than the load it was asked for.
    """
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if gid < 0:
        raise ValueError(f"model has no geom '{geom_name}'; ride-mode contact queries need it")
    return int(gid)


__all__ = [
    "TerrainContacts",
    "TerrainContactQuery",
    "TERRAIN_GEOMS",
    "FRONT_CONTACT_GEOM",
    "REAR_CONTACT_GEOM",
    "HANDLEBAR_GEOM",
    "CONTACT_LOAD_THRESHOLD_N",
    "CONTACT_DROPOUT_STEPS",
]
