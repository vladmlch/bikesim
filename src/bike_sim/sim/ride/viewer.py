"""
Interactive Ride-Mode Viewer.

The live rolling run: a MuJoCo passive viewer, a real-time pacer, and the loop that ties them
to a `RideSession`. Everything that can be checked without a window lives elsewhere --
termination in `termination.py`, key dispatch in `input.py`, HUD formatting in `hud.py`, camera
tracking in `sim/camera.py`, session state in `session.py` -- so this module is a loop that
paces, syncs and prints, and the only untestable part of the mode is that loop.

**The viewer must be paced, not stepped once per frame.** Ride mode integrates at 0.5 ms for
`sphere`/`fast` and 0.25 ms for `pneumatic/detailed` (docs/RIDE.md sections 3.1 and 10), so a
loop that took one step per rendered frame would run the track far below real time. The pacer
converts elapsed wall clock into whole timesteps and caps the arrears it will chase, so a stall
in the window manager cannot make the simulation sprint.
"""

import time
from typing import TYPE_CHECKING, Optional, Union

import mujoco

from bike_sim.physics.drivetrain import DrivetrainSpecs
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.cruise import DEFAULT_TARGET_SPEED_KMH
from bike_sim.sim.ride.session import RideSession
from bike_sim.sim.ride.termination import RunOutcome
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import TrackSpec

if TYPE_CHECKING:
    # Import-time only: mujoco.viewer pulls in glfw, which every headless consumer of this
    # package would otherwise pay for.
    import mujoco.viewer

# Longest arrears the pacer will chase, in seconds of simulated time. A frame that took longer
# than this loses the excess rather than making it up: dropping simulated time keeps the run
# honest about being behind, whereas sprinting through 200 ms of terrain in one frame hides it.
MAX_CATCHUP_S = 0.05

# Viewer refresh intervals, in seconds of real time. Rendering and printing are decoupled from
# the physics because the pacer takes up to a hundred steps per frame and neither the window nor
# the terminal benefits from being redrawn per step.
SYNC_INTERVAL_S = 1.0 / 120.0
HUD_REFRESH_INTERVAL_S = 0.08
FRAME_SLEEP_S = 0.001


class RealTimePacer:
    """
    Converts elapsed wall clock into whole simulation steps.

    Keeps the sub-timestep remainder so that a sequence of frames advances the simulation at
    exactly real time on average rather than losing a fraction of a step per frame.
    """

    def __init__(self, timestep_s: float, max_catchup_s: float = MAX_CATCHUP_S) -> None:
        """
        Args:
            timestep_s: Integration timestep of the compiled model, in seconds.
            max_catchup_s: Longest arrears the pacer will chase, in seconds.

        Raises:
            ValueError: If the timestep is not positive, or the cap is shorter than one step --
                which would return zero steps forever and freeze the run.
        """
        if timestep_s <= 0.0:
            raise ValueError(f"timestep_s must be positive, got {timestep_s}")
        if max_catchup_s < timestep_s:
            raise ValueError(
                f"max_catchup_s {max_catchup_s} is shorter than one timestep {timestep_s}"
            )
        self.timestep_s = float(timestep_s)
        self.max_catchup_s = float(max_catchup_s)
        self.arrears_s = 0.0

    def steps_for(self, elapsed_s: float) -> int:
        """
        Reports how many steps to take for a frame of the given duration.

        Args:
            elapsed_s: Wall-clock seconds since the previous call. Negative values are treated
                as zero, so a non-monotonic clock cannot rewind the accumulator.

        Returns:
            Number of simulation steps to take now.
        """
        self.arrears_s = min(self.arrears_s + max(0.0, float(elapsed_s)), self.max_catchup_s)
        steps = int(self.arrears_s / self.timestep_s)
        self.arrears_s -= steps * self.timestep_s
        return steps

    def reset(self) -> None:
        """Discards the accumulated arrears, so a restarted run does not inherit them."""
        self.arrears_s = 0.0


def run_interactive_ride(
    track: Optional[TrackSpec] = None,
    specs: Optional[BikeSpecs] = None,
    target_speed_kmh: float = DEFAULT_TARGET_SPEED_KMH,
    include_rider: bool = True,
    rider: Optional[Union[RiderSpecs, str]] = None,
    tyre: Optional[TyreConfig] = None,
    drive_mode: str = "motor",
    assist: str = "tour",
    drivetrain: Optional[DrivetrainSpecs] = None,
    legs: Optional[str] = None,
    visual_pedalling: bool = False,
) -> Optional[RunOutcome]:
    """
    Launches the live interactive ride viewer.

    When the run terminates the reason is printed once and the viewer stays open, so the
    finishing state can be inspected and `R` restarts.

    Args:
        track: Track to ride. Defaults to the shipped default preset.
        specs: Bicycle geometry. Defaults to the shipped `BikeSpecs`.
        target_speed_kmh: Initial cruise target, inside the 15-45 km/h band.
        include_rider: Legacy switch; False rides the bike alone. Ignored when ``rider`` is given.
        rider: The rider variant or `RiderSpecs`; fixed for the session (see `session.py`).
        tyre: Wheel-contact model; defaults to the existing sphere model.
        drive_mode: ``motor``, ``pedal`` or ``pedelec``; see `RideSimulation`.
        assist: Assist level the mid-drive starts in; `E` cycles it while riding.
        drivetrain: Gearing and drivetrain ceilings. Defaults to the shipped 32x14 eMTB.
        legs: ``articulated`` or ``rigid`` legs for the seated rider; None follows the
            crankset (see `RideSimulation`).
        visual_pedalling: Motor mode only: spin the crankset so the rider visibly
            pedals while the wheel actuator drives (see `RideSimulation`).

    Returns:
        The run's outcome if it terminated before the window was closed, else None.

    Raises:
        RuntimeError: If the starting equilibrium does not converge.
    """
    import mujoco.viewer

    from bike_sim.sim.playground import ensure_macos_mjpython

    ensure_macos_mjpython()

    sim = RideSimulation(
        track=track,
        specs=specs,
        target_speed_kmh=target_speed_kmh,
        include_rider=include_rider,
        rider=rider,
        tyre=tyre,
        debug_markers=True,
        drive_mode=drive_mode,
        assist=assist,
        drivetrain=drivetrain,
        legs=legs,
        visual_pedalling=visual_pedalling,
    )
    session = RideSession(sim)

    print("\n" + "=" * 80)
    print(f"  MUJOCO RIDE MODE - {sim.track.name.upper()} ({sim.track.length_m:.0f} m)")
    print("=" * 80)
    print(f"  {sim.track.description}")
    session.print_help()

    pacer = RealTimePacer(float(sim.model.opt.timestep))
    site_bb = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "site_BB")

    with mujoco.viewer.launch_passive(
        sim.model,
        sim.data,
        key_callback=session.handle_key,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        session.viewer = viewer
        session.camera.reset_preset()

        last_frame = time.monotonic()
        last_sync = last_hud = last_frame
        announced = False

        while viewer.is_running():
            now = time.monotonic()
            elapsed, last_frame = now - last_frame, now

            session.process_pending_keys()
            for _ in range(pacer.steps_for(elapsed)):
                if session.step() is not None:
                    pacer.reset()
                    break

            if session.outcome is None:
                announced = False
            elif not announced:
                print(f"\n[RUN ENDED] {session.outcome.describe()}")
                announced = True

            if now - last_sync >= SYNC_INTERVAL_S:
                bb = sim.data.site_xpos[site_bb] if site_bb >= 0 else (0.0, 0.0, 0.0)
                session.camera.update_viewer(viewer, bike_x=float(bb[0]), bike_z=float(bb[2]))
                viewer.sync()
                last_sync = now

            if session.show_telemetry and now - last_hud >= HUD_REFRESH_INTERVAL_S:
                session.print_hud()
                last_hud = now

            time.sleep(FRAME_SLEEP_S)

    return session.outcome


__all__ = [
    "RealTimePacer",
    "run_interactive_ride",
    "MAX_CATCHUP_S",
    "SYNC_INTERVAL_S",
    "HUD_REFRESH_INTERVAL_S",
]
