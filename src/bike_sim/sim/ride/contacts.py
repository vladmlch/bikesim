"""
Ride-Mode Terrain Contact Queries.

One place that answers, per step, which of the tracked geoms are touching the road and with
what normal load. Cruise gating, rolling resistance, the virtual rider and the crash
detector all need that answer, and all four must see the *same* answer within a step, so
the query is made once per step and the snapshot is passed around.

Every load comes from `mj_contactForce`, never from a static axle-load estimate. Rolling
resistance is proportional to the instantaneous load (docs/RIDE.md section 4), and the whole
reason for modelling it that way is that the load is not static: a G-out roughly doubles it
and a crest takes it to zero.

The forces read here are those of the last `mj_forward`/`mj_step`, so within one control step
they lag the state by a single 0.5 ms timestep. That is the normal cost of reading constraint
forces from a controller and is negligible against every time constant in the model.

**Each wheel gets two load channels, because two questions are being asked.**

*Did the legacy contact bridge report load?* -- `front_load_n` / `rear_load_n`: the
scalar sum of the normal-force magnitudes of every contact row the wheel owns against the
terrain, **bridged** across a short collision dropout. Magnitudes are summed rather than
vectors precisely so that rows with opposing normals cannot cancel and read as airborne; a
wheel jammed against two faces is emphatically in contact.

*How hard is this wheel pressed into the road?* -- `front_support_n` / `rear_support_n`: the
vertical component of the vector sum of the same rows' normal forces, **unbridged**. This is
the only channel `Crr . N . r` may use. It is vertical because a sphere on a square edge
touches the ~87 degree face and the ledge top at once, and adding those magnitudes
double-counts a load that is mostly horizontal -- an impact, not weight on a contact patch.
It is unbridged because bridging inflates a spec-reported sink: measured over the flat
`single_edge` run-up, the bridged pair averages 111.7 % of the 1023.7 N system weight while
the raw pair averages 100.2 %. The 13 % of steps with no contact row carry near-zero load, so
bridging *introduces* the error in this channel instead of removing it. Bridging the driving
gate is still right, because a false airborne there fires the +/-80 N.m rider moment and gates
150 N.m of drive torque off.

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
from typing import Collection, Dict, Optional, Protocol, Tuple

import mujoco
import numpy as np

from bike_sim.sim.ride.contact_filter import GroundedFilter
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot
from bike_sim.sim.ride.tyre.model import WheelOutputs
from bike_sim.sim.ride.wheel_kinematics import wheel_point_velocity

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

# Controller-only debounce duration. Physical load and force channels never use this value.
CONTROLLER_GROUNDED_HOLD_S = 0.005


class WheelLoadProvider(Protocol):
    """Per-wheel normal load and support supplied by a non-contact tyre model."""

    front_outputs: WheelOutputs
    rear_outputs: WheelOutputs


@dataclass(frozen=True)
class TerrainContacts:
    """
    Snapshot of the bike's contact with the road at one instant.

    Attributes:
        front_load_n: Total normal-force magnitude on the front contact sphere, in newtons,
            held across a collision dropout shorter than `CONTACT_DROPOUT_STEPS`. The driving
            and gating signal.
        rear_load_n: The same for the rear contact sphere.
        front_support_n: Vertical component of the front wheel's normal load this step, in
            newtons, never held and never negative. The load `Crr . N . r` may use.
        rear_support_n: The same for the rear wheel.
        handlebar_load_n: Normal-force magnitude on the handlebar geom, in newtons. Not held:
            a handlebar contact lasts far longer than a dropout, and a missed step only delays
            the crash detector by 0.5 ms.
        front_snapshot: Raw, immutable physical contact state of the front wheel, when queried.
        rear_snapshot: The corresponding raw state of the rear wheel.
        front_controller_grounded: Time-filtered working-road load above the contact threshold
            for the front controller.
        rear_controller_grounded: The corresponding rear controller signal. Neither field is a
            normal load; they are absent for legacy pneumatic outputs and synthetic snapshots.
    """

    front_load_n: float
    rear_load_n: float
    front_support_n: float
    rear_support_n: float
    handlebar_load_n: float
    front_snapshot: WheelContactSnapshot | None = None
    rear_snapshot: WheelContactSnapshot | None = None
    front_controller_grounded: bool | None = None
    rear_controller_grounded: bool | None = None

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
    `data.contact` with a reused MuJoCo force buffer.

    The query is stateful, because bridging a collision dropout requires remembering the
    previous steps. The legacy load bridge advances per query; the separate controller
    filter is idempotent at the same timestamp. Advance the query once per simulation step
    and `reset` it when the run restarts.
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

        self.terrain_geom_names = {
            _geom_id(model, name): name for name in TERRAIN_GEOMS
        }
        self.terrain_ids = frozenset(self.terrain_geom_names)
        self.front_id = _geom_id(model, FRONT_CONTACT_GEOM)
        self.rear_id = _geom_id(model, REAR_CONTACT_GEOM)
        self.handlebar_id = _geom_id(model, HANDLEBAR_GEOM)

        self._force = np.zeros(6, dtype=float)
        self._front_load = _BridgedLoad(dropout_steps)
        self._rear_load = _BridgedLoad(dropout_steps)
        self._front_controller_grounded = GroundedFilter(CONTROLLER_GROUNDED_HOLD_S)
        self._rear_controller_grounded = GroundedFilter(CONTROLLER_GROUNDED_HOLD_S)

    def query(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        wheel_load_provider: Optional[WheelLoadProvider] = None,
        *, time_s: float | None = None, qvel: np.ndarray | None = None,
    ) -> TerrainContacts:
        """
        Reads the current contact normal load on each tracked geom.

        Args:
            model: Compiled ride-mode model.
            data: Simulation state, after a `mj_forward` or `mj_step` has populated its
                contacts and constraint forces.
            wheel_load_provider: Optional pneumatic tyre applier. Its raw wheel loads and
                vertical support replace the sphere contacts; the handlebar still uses
                MuJoCo contact.

        Returns:
            Both legacy load channels for each wheel -- bridged magnitude and raw vertical
            support -- plus the handlebar's raw magnitude and separate controller booleans.
        """
        sample_time = float(data.time) if time_s is None else float(time_s)
        magnitude_n, vertical_n, wheel_patches = self._sum_normal_loads(
            model, data, include_wheel_contacts=wheel_load_provider is None, qvel=qvel
        )
        interval_id = round(sample_time / float(model.opt.timestep))
        front_axis = data.geom_xpos[self.front_id]
        rear_axis = data.geom_xpos[self.rear_id]
        if wheel_load_provider is not None:
            front = wheel_load_provider.front_outputs
            rear = wheel_load_provider.rear_outputs
            return TerrainContacts(
                front_load_n=max(0.0, front.normal_load_n),
                rear_load_n=max(0.0, rear.normal_load_n),
                front_support_n=max(0.0, front.support_n),
                rear_support_n=max(0.0, rear.support_n),
                handlebar_load_n=magnitude_n[self.handlebar_id],
                front_snapshot=_legacy_tyre_snapshot(
                    model, data, self.front_id, front, interval_id
                ),
                rear_snapshot=_legacy_tyre_snapshot(
                    model, data, self.rear_id, rear, interval_id
                ),
            )
        front_snapshot = WheelContactSnapshot(
            time_s=sample_time, patches=wheel_patches[self.front_id],
            geometric_contact=bool(wheel_patches[self.front_id]),
            interval_id=interval_id, backend="native_reference", wheel_axis_m=front_axis,
        )
        rear_snapshot = WheelContactSnapshot(
            time_s=sample_time, patches=wheel_patches[self.rear_id],
            geometric_contact=bool(wheel_patches[self.rear_id]),
            interval_id=interval_id, backend="native_reference", wheel_axis_m=rear_axis,
        )
        return TerrainContacts(
            front_load_n=self._front_load.update(magnitude_n[self.front_id]),
            rear_load_n=self._rear_load.update(magnitude_n[self.rear_id]),
            front_support_n=max(0.0, vertical_n[self.front_id]),
            rear_support_n=max(0.0, vertical_n[self.rear_id]),
            handlebar_load_n=magnitude_n[self.handlebar_id],
            front_snapshot=front_snapshot,
            rear_snapshot=rear_snapshot,
            front_controller_grounded=self._front_controller_grounded.update(
                _working_road_grounded(front_snapshot), sample_time
            ),
            rear_controller_grounded=self._rear_controller_grounded.update(
                _working_road_grounded(rear_snapshot), sample_time
            ),
        )

    def reset(self) -> None:
        """Discard legacy held loads and timestamp-filter history for a fresh run."""
        self._front_load.reset()
        self._rear_load.reset()
        self._front_controller_grounded.reset()
        self._rear_controller_grounded.reset()

    def handlebar_load(self, model: mujoco.MjModel, data: mujoco.MjData) -> float:
        """
        Normal-force magnitude on the handlebar geom against the terrain.

        Identical to ``query(...).handlebar_load_n`` -- the same pass over
        `data.contact` with `mj_contactForce`, restricted to handlebar rows --
        for tire backends whose wheel channels replace every other field. No
        bridge or filter state advances here, so it must not be the sole call
        when the native wheel loads are consumed.
        """
        load_n = 0.0
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if not (
                (geom1 == self.handlebar_id and geom2 in self.terrain_ids)
                or (geom2 == self.handlebar_id and geom1 in self.terrain_ids)
            ):
                continue
            mujoco.mj_contactForce(model, data, i, self._force)
            load_n += abs(float(self._force[0]))
        return load_n

    def _sum_normal_loads(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        include_wheel_contacts: bool = True,
        qvel: np.ndarray | None = None,
    ) -> Tuple[Dict[int, float], Dict[int, float], Dict[int, tuple[ContactPatch, ...]]]:
        """
        Accumulates this step's normal contact load on each tracked geom, two ways.

        A wheel routinely owns several contact rows at once -- adjacent heightfield prisms, of
        which MuJoCo loads only one and leaves its coplanar siblings at exactly zero, and at a
        square edge the near-vertical face and the ledge top together. Both sums pass over all
        of them, but they combine them differently on purpose; see the module docstring.

        Args:
            model: Compiled ride-mode model.
            data: Simulation state with populated contacts.

        Returns:
            Tuple of (magnitudes, verticals, wheel patches), keyed by geom id. `magnitudes`
            adds the scalar normal force of every row, so opposing normals reinforce rather
            than cancel. `verticals` adds the world-Z component of each row's normal force
            vector, signed so that support on the tracked geom is positive, so a horizontal
            face impact contributes almost nothing.
        """
        magnitude_n: Dict[int, float] = {self.front_id: 0.0, self.rear_id: 0.0, self.handlebar_id: 0.0}
        vertical_n: Dict[int, float] = {self.front_id: 0.0, self.rear_id: 0.0, self.handlebar_id: 0.0}
        wheel_patches: Dict[int, list[ContactPatch]] = {self.front_id: [], self.rear_id: []}

        for i in range(data.ncon):
            contact = data.contact[i]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if not (
                (geom1 in self.terrain_ids and geom2 in magnitude_n)
                or (geom2 in self.terrain_ids and geom1 in magnitude_n)
            ):
                continue
            mujoco.mj_contactForce(model, data, i, self._force)
            result = _tracked_wrench(
                geom1, geom2, self.terrain_ids, magnitude_n.keys(),
                contact.frame.reshape(3, 3), self._force,
            )
            if result is None:
                continue
            tracked, normal, force_world, couple_world = result
            if not include_wheel_contacts and tracked in wheel_patches:
                continue
            normal_force_n = float(self._force[0])
            magnitude_n[tracked] += abs(normal_force_n)
            vertical_n[tracked] += normal_force_n * float(normal[2])
            if tracked in wheel_patches:
                source_id = geom1 if geom1 in self.terrain_ids else geom2
                # MuJoCo's sphere/heightfield rows can tilt a few microradians out of
                # the constrained X-Z bicycle plane. Project their direction into our
                # planar contact contract while retaining the full world wrench above.
                planar_normal = np.array([normal[0], 0.0, normal[2]])
                planar_normal /= np.linalg.norm(planar_normal)
                tangent = np.array([planar_normal[2], 0.0, -planar_normal[0]])
                body_id = int(model.geom_bodyid[tracked])
                point = np.asarray(contact.pos, dtype=float)
                slip_mps = float(np.dot(
                    wheel_point_velocity(model, data, body_id, point, qvel=qvel), tangent
                ))
                wheel_patches[tracked].append(ContactPatch(
                    point_m=point,
                    normal=planar_normal,
                    normal_load_n=max(0.0, normal_force_n),
                    tangent_force_n=float(np.dot(force_world, tangent)),
                    slip_mps=slip_mps,
                    couple_world_nm=couple_world,
                    native_world_force_n=force_world,
                    source_geom=self.terrain_geom_names[source_id],
                ))

        return magnitude_n, vertical_n, {
            geom_id: tuple(patches) for geom_id, patches in wheel_patches.items()
        }


def _working_road_grounded(snapshot: WheelContactSnapshot) -> bool:
    """Convert only current terrain normal load to the controller's raw boolean."""
    return sum(
        patch.normal_load_n for patch in snapshot.patches if patch.working_surface
    ) > CONTACT_LOAD_THRESHOLD_N


def _tracked_wrench(
    geom1: int,
    geom2: int,
    terrain_ids: frozenset[int],
    tracked_ids: Collection[int],
    frame: np.ndarray,
    contact_force: np.ndarray,
) -> tuple[int, np.ndarray, np.ndarray, np.ndarray] | None:
    """Convert MuJoCo's contact-frame wrench to force on a tracked geom.

    The normal points from geom1 to geom2. MuJoCo's reported wrench acts on
    geom2; reversing the geom order reverses both force and contact couple.
    """
    if geom1 in terrain_ids and geom2 in tracked_ids:
        tracked, sign = geom2, 1.0
    elif geom2 in terrain_ids and geom1 in tracked_ids:
        tracked, sign = geom1, -1.0
    else:
        return None
    rotation = np.asarray(frame, dtype=float).reshape(3, 3).T
    force = np.asarray(contact_force, dtype=float)
    return (
        tracked,
        sign * rotation[:, 0],
        sign * (rotation @ force[:3]),
        sign * (rotation @ force[3:6]),
    )


def _legacy_tyre_snapshot(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
    outputs: WheelOutputs,
    interval_id: int,
) -> WheelContactSnapshot:
    """Copy the pre-existing pneumatic output into the physical snapshot contract."""
    body_id = int(model.geom_bodyid[geom_id])
    patches = []
    for output in outputs.patches:
        normal = output.normal_force_world_n / output.normal_load_n
        tangent = np.array([normal[2], 0.0, -normal[0]])
        point = output.centroid_world_m
        patches.append(ContactPatch(
            point_m=point,
            normal=normal,
            normal_load_n=output.normal_load_n,
            tangent_force_n=output.tangential_force_n,
            slip_mps=float(np.dot(wheel_point_velocity(model, data, body_id, point), tangent)),
        ))
    return WheelContactSnapshot(
        time_s=float(data.time), patches=tuple(patches),
        geometric_contact=not outputs.airborne,
        interval_id=interval_id, backend="legacy_pneumatic",
        wheel_axis_m=data.geom_xpos[geom_id],
    )


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
    "CONTROLLER_GROUNDED_HOLD_S",
    "WheelLoadProvider",
]
