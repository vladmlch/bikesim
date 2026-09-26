"""
Ride-Mode Interactive Session.

The state an interactive run has beyond the simulation itself: the camera, the brake toggle,
the display flags, and the run's termination. The key map acts on this object and nothing else,
so every binding is exercised without a window -- `RideSession` is fully usable headlessly, and
that is how it is tested.

**The outcome is latched.** When a run terminates the session stops stepping and keeps the
reason, so the viewer can hold the finishing state on screen and report why it ended instead of
quietly continuing to integrate past the end of the track.

**Keys are queued.** The passive viewer calls its key callback from its own thread, and the
actions behind those keys write damper clicks and air pressure -- state `mj_step` is reading.
Events are therefore enqueued and drained on the physics thread, exactly as the test stand
does. With no viewer attached they are applied immediately, so a headless session behaves
synchronously.

**The rider is not a key.** The seated rider has its own bodies and joints, so switching rider
variants changes the compiled model's coordinate count, which a passive viewer cannot follow
in place. The rider is chosen on the command line (`bike-ride --rider`); comparing variants is
two headless runs on the same seed.
"""

import queue
import threading
from typing import TYPE_CHECKING, Optional

import mujoco

from bike_sim.physics.air_spring import ForkAirSpring
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.sim.camera import CameraManager
from bike_sim.sim.ride.cruise import MAX_TARGET_SPEED_KMH, MIN_TARGET_SPEED_KMH
from bike_sim.sim.ride.hud import RideHUD
from bike_sim.sim.ride.input import RideInputHandler
from bike_sim.sim.ride.livery import ModelLivery
from bike_sim.sim.ride.termination import RunLimits, RunOutcome, RunTerminator

if TYPE_CHECKING:
    # Type-only on both counts: importing the orchestrator at runtime would make the ride
    # package import itself through `ride_sim`, and `mujoco.viewer` pulls in glfw.
    import mujoco.viewer

    from bike_sim.sim.ride_sim import RideSimulation

# Brake lever position the toggle applies before the strength keys touch it. Half of the
# 200 N.m per-wheel ceiling is a firm but controllable lever; full demand stops the bike from
# 25 km/h in 4.25 m.
DEFAULT_BRAKE_STRENGTH = 0.5


class RideSession:
    """
    One interactive ride: the simulation, the camera, the brake toggle and the run's outcome.

    Attributes:
        sim: The simulation being ridden.
        camera: Camera perspective manager.
        hud: Terminal HUD formatter.
        input: Key dispatcher bound to this session.
        braking: Whether the brake toggle is engaged.
        brake_strength: Lever position the toggle applies, in [0, 1].
        show_telemetry: Whether the viewer prints the HUD line.
        debug_markers: Whether the pivot markers are visible.
        viewer: The attached passive viewer handle, or None while headless.
        limits: Bounds the current run is judged against.
        outcome: The finished run's outcome, or None while it is still live.
    """

    def __init__(
        self,
        sim: "RideSimulation",
        camera: Optional[CameraManager] = None,
        show_telemetry: bool = True,
    ) -> None:
        """
        Args:
            sim: Simulation to ride. The session takes it as given rather than building it, so
                a caller stays in charge of the track, the springs and the marker geoms.
            camera: Camera perspective manager. Defaults to the 2D side view.
            show_telemetry: Whether the HUD line is printed while riding.
        """
        self.sim = sim
        self.camera = camera if camera is not None else CameraManager(default_mode="2d")
        self.hud = RideHUD(sim.model)
        self.input = RideInputHandler(self)
        self.livery = ModelLivery(sim.model, sim.specs, sim.solver)

        self.braking = False
        self.brake_strength = DEFAULT_BRAKE_STRENGTH
        self.show_telemetry = show_telemetry
        self.debug_markers = False
        self.viewer: Optional["mujoco.viewer.Handle"] = None

        self.limits: RunLimits = sim.default_limits()
        self.terminator = RunTerminator(self.limits)
        self.outcome: Optional[RunOutcome] = None

        self._key_queue: "queue.Queue[int]" = queue.Queue()
        self._lock = threading.RLock()

        self.livery.set_markers(self.debug_markers)
        # The simulation solved its own equilibrium at construction, so the session arms a run
        # rather than resetting into a second nine-thousand-step relaxation.
        self._start_run()

    # --- What the key map reaches for ---

    @property
    def suspension_system(self) -> BikeSuspensionSystem:
        """The fork and shock damper models the damping keys adjust."""
        return self.sim.controller.suspension_system

    @property
    def air_spring(self) -> ForkAirSpring:
        """The fork air spring the token and pressure keys adjust."""
        return self.sim.controller.air_spring

    def adjust_target_speed(self, delta_kmh: float) -> float:
        """
        Steps the cruise target, clamped to the section 6 band.

        Clamping rather than raising is deliberate: the controller's setter rejects an
        out-of-band target because a mis-specified experiment should fail loudly, but a key
        press that walks into the edge of the band is not a mis-specified experiment, and an
        exception raised inside the viewer's key handler would take the run down with it.

        Args:
            delta_kmh: Change in km/h.

        Returns:
            The new target in km/h.
        """
        target = self.sim.cruise.target_speed_kmh + float(delta_kmh)
        self.sim.cruise.target_speed_kmh = max(
            MIN_TARGET_SPEED_KMH, min(MAX_TARGET_SPEED_KMH, target)
        )
        return self.sim.cruise.target_speed_kmh

    def toggle_braking(self) -> bool:
        """
        Toggles the brakes.

        Returns:
            Whether the brakes are now engaged.
        """
        self.braking = not self.braking
        return self.braking

    def adjust_brake_strength(self, delta: float) -> float:
        """
        Steps the lever position the brake toggle applies.

        Args:
            delta: Change in lever position, in units of full demand.

        Returns:
            The new lever position, in [0, 1].
        """
        self.brake_strength = max(0.0, min(1.0, self.brake_strength + float(delta)))
        return self.brake_strength

    def toggle_debug_markers(self) -> bool:
        """
        Toggles the yellow pivot-marker livery.

        Returns:
            Whether the markers are now visible.
        """
        self.debug_markers = not self.debug_markers
        self.livery.set_markers(self.debug_markers)
        mujoco.mj_forward(self.sim.model, self.sim.data)
        if not self.livery.has_markers:
            print("\n[KEY G] No marker geoms in this model; build it with debug_markers=True.")
        else:
            print(f"\n[KEY G] Debug pivot markers: {'ON' if self.debug_markers else 'OFF'}")
        return self.debug_markers

    def print_help(self) -> None:
        """Displays the ride-mode control help in the terminal."""
        self.hud.print_help()

    # --- Running ---

    def reset_run(self) -> None:
        """
        Returns the bike to the solved static equilibrium and starts a fresh run.

        Raises:
            RuntimeError: If the equilibrium solve does not converge.
        """
        self.sim.reset()
        self._start_run()

    def _start_run(self) -> None:
        """Arms a fresh terminator for the state the simulation is currently in."""
        self.limits = self.sim.default_limits()
        self.terminator = RunTerminator(self.limits)
        self.terminator.start()
        self.outcome = None
        self.braking = False

    def step(self) -> Optional[RunOutcome]:
        """
        Checks termination and, if the run is still live, advances it by one timestep.

        Returns:
            The run's outcome once it has terminated, or None while it is still running. The
            outcome is latched, so a terminated session can be held on screen without the
            physics carrying on behind it.
        """
        with self._lock:
            if self.outcome is not None:
                return self.outcome

            reason = self.terminator.reason(self.sim.position_m, self.sim.steps, self.sim.crash)
            if reason is not None:
                self.outcome = self.terminator.outcome(
                    reason, self.sim.steps, self.sim.time_s, self.sim.position_m, self.sim.crash
                )
                return self.outcome

            demand = self.brake_strength if self.braking else 0.0
            self.sim.step(demand, demand)
            return None

    def hud_line(self) -> str:
        """Returns the live HUD line for the session's current state."""
        return self.hud.line(self.sim, self.braking, self.brake_strength)

    def print_hud(self) -> None:
        """Writes the live HUD line over the current terminal line."""
        self.hud.print_line(self.sim, self.braking, self.brake_strength)

    # --- Key plumbing ---

    def handle_key(self, keycode: int) -> None:
        """
        Accepts a keycode from the viewer thread.

        Args:
            keycode: GLFW keycode. Queued for the physics thread while a viewer is attached,
                and applied immediately when there is none.
        """
        self._key_queue.put(keycode)
        if self.viewer is None:
            self.process_pending_keys()

    def process_pending_keys(self) -> None:
        """Drains every queued key event on the calling thread."""
        with self._lock:
            while True:
                try:
                    keycode = self._key_queue.get_nowait()
                except queue.Empty:
                    return
                self.input.handle_key(keycode)


__all__ = ["RideSession", "DEFAULT_BRAKE_STRENGTH"]
