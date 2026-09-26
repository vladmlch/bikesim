"""
Ride-Mode Input & Key Event Dispatcher.

Maps GLFW keycodes to ride-session actions through one declarative lookup table, in the same
shape as `sim/input_handler.py` does for the test stand.

**Every damper and air-spring key keeps its playground binding**, deliberately: the two modes
adjust the same two hardware models, and a rider who has learnt `J`/`H` on the stand should
not have to relearn it on the track. Only keys with no counterpart on a static stand are new
-- `W`/`S` for the cruise target, `Space` for the brakes, `,`/`.` for brake strength -- and
they take over the three stand-only bindings (travel up/down, auto-sweep).

**Braking is a toggle, not a hold.** MuJoCo's passive viewer delivers key *press* events, not
key state (docs/RIDE.md section 6), so a held lever cannot be represented faithfully and
pretending otherwise would give a brake that releases itself on the next repeat event.
"""

from typing import Any, Callable, Dict, List, Tuple

# Steps the new bindings take. One km/h spans the 15-45 km/h band in thirty presses, and 10 %
# of the 200 N.m per-wheel brake ceiling is 20 N.m -- fine enough to trail the brakes through a
# rock garden, coarse enough to reach a locked wheel in ten.
TARGET_SPEED_STEP_KMH = 1.0
BRAKE_STRENGTH_STEP = 0.1


class RideInputHandler:
    """
    Dispatches keyboard events to a ride session.

    The handler holds no state of its own: every key mutates the session, which owns the
    simulation, the camera and the brake toggle. That keeps the dispatch table a pure map from
    keycode to intent, and keeps this module out of the physics.
    """

    def __init__(self, session: Any) -> None:
        """
        Args:
            session: Ride session the keys act on. It supplies `sim`, `camera`, and the
                interactive toggles; see `sim/ride/viewer.py`.
        """
        self.session = session
        self._dispatch_map: Dict[int, Callable[[], None]] = self._build_dispatch_map()

    def handle_key(self, keycode: int) -> bool:
        """
        Dispatches one keycode.

        Args:
            keycode: GLFW keycode delivered by the passive viewer.

        Returns:
            True if the key was bound and its action ran, False if the key is unmapped.
        """
        handler = self._dispatch_map.get(keycode)
        if handler is None:
            return False
        handler()
        return True

    @property
    def bound_keycodes(self) -> Tuple[int, ...]:
        """Every keycode the handler answers to, in ascending order."""
        return tuple(sorted(self._dispatch_map))

    def _build_dispatch_map(self) -> Dict[int, Callable[[], None]]:
        """
        Builds the keycode-to-action table.

        Returns:
            Map from GLFW keycode to a zero-argument action. Letter keys are bound for both
            cases, because the viewer reports the physical key without applying shift.
        """
        s = self.session
        actions: List[Tuple[Tuple[int, ...], Callable[[], None]]] = [
            # Speed, brakes and run control -- the bindings with no test-stand counterpart
            ((87, 119), lambda: self._adjust_target_speed(+TARGET_SPEED_STEP_KMH, "W")),
            ((83, 115), lambda: self._adjust_target_speed(-TARGET_SPEED_STEP_KMH, "S")),
            ((32,), self._on_toggle_braking),
            ((46,), lambda: self._adjust_brake_strength(+BRAKE_STRENGTH_STEP, ".")),
            ((44,), lambda: self._adjust_brake_strength(-BRAKE_STRENGTH_STEP, ",")),
            ((82, 114), self._on_reset_run),
            # System / views. `B` is deliberately unbound in ride mode: the rider variant
            # changes the compiled model's coordinates and is a command-line choice.
            ((67, 99), self._on_cycle_camera),
            ((49, 321), lambda: self._on_set_camera("2d", "2D SIDE VIEW")),
            ((50, 322), lambda: self._on_set_camera("3d", "3D ISOMETRIC VIEW")),
            ((80, 112), self._on_cycle_preset),
            # Fork Charger 3 damping adjustments
            ((76, 108), lambda: self._adjust_fork("lsc", +1, "L", "LSC")),
            ((75, 107), lambda: self._adjust_fork("lsc", -1, "K", "LSC")),
            ((74, 106), lambda: self._adjust_fork("hsc", +1, "J", "HSC")),
            ((72, 104), lambda: self._adjust_fork("hsc", -1, "H", "HSC")),
            ((85, 117), lambda: self._adjust_fork("rebound", +1, "U", "Rebound")),
            ((89, 121), lambda: self._adjust_fork("rebound", -1, "Y", "Rebound")),
            # Shock Super Deluxe damping adjustments and lockout
            ((88, 120), self._on_shock_toggle_lockout),
            ((48,), lambda: self._adjust_shock("lsc", +1, "0", "LSC")),
            ((57,), lambda: self._adjust_shock("lsc", -1, "9", "LSC")),
            ((56,), lambda: self._adjust_shock("hsc", +1, "8", "HSC")),
            ((55,), lambda: self._adjust_shock("hsc", -1, "7", "HSC")),
            ((54,), lambda: self._adjust_shock("rebound", +1, "6", "Rebound")),
            ((53,), lambda: self._adjust_shock("rebound", -1, "5", "Rebound")),
            ((52,), lambda: self._adjust_shock("hbo", +1, "4", "HBO")),
            ((51,), lambda: self._adjust_shock("hbo", -1, "3", "HBO")),
            # Air spring tuning
            ((91,), lambda: self._adjust_tokens(-1, "[", "Less end-stroke ramp-up")),
            ((93,), lambda: self._adjust_tokens(+1, "]", "More ramp-up / bottom-out resistance")),
            ((45,), lambda: self._adjust_psi(-2.0, "-")),
            ((61,), lambda: self._adjust_psi(+2.0, "=")),
            # Display
            ((84, 116), self._on_toggle_telemetry),
            ((71, 103), s.toggle_debug_markers),
            ((47, 63), s.print_help),
        ]

        mapping: Dict[int, Callable[[], None]] = {}
        for keys, action in actions:
            for k in keys:
                mapping[k] = action
        return mapping

    # --- Parameterized helpers ---

    # The click attribute is `<param>_clicks`, but set_clicks names the rebound circuit `reb`;
    # everything else uses the same spelling for both.
    _SET_CLICKS_KWARG = {"rebound": "reb"}

    def _adjust_damper(
        self,
        damper: Any,
        damper_label: str,
        param: str,
        delta: int,
        key: str,
        label: str,
    ) -> None:
        """
        Steps one damper circuit by one click and reports the new setting.

        Args:
            damper: Damper model to adjust.
            damper_label: Human-readable damper name for the printed line.
            param: Circuit name (`hsc`, `lsc`, `rebound`, `hbo`).
            delta: Click change, positive or negative.
            key: Key label for the printed line.
            label: Circuit label for the printed line.
        """
        # set_clicks owns the clamp, so write through it rather than assigning the raw value
        # first and correcting afterwards.
        kwarg = self._SET_CLICKS_KWARG.get(param, param)
        damper.set_clicks(**{kwarg: getattr(damper, f"{param}_clicks") + delta})
        val = getattr(damper, f"{param}_clicks")
        max_val = getattr(damper, f"max_{param[:3]}")
        print(f"\n[KEY {key}] {damper_label} {label} -> Click {val}/{max_val}")

    def _adjust_fork(self, param: str, delta: int, key: str, label: str) -> None:
        """Steps a fork damper circuit; see `_adjust_damper`."""
        self._adjust_damper(
            self.session.suspension_system.fork_damper, "Fork Charger 3", param, delta, key, label
        )

    def _adjust_shock(self, param: str, delta: int, key: str, label: str) -> None:
        """Steps a shock damper circuit; see `_adjust_damper`."""
        self._adjust_damper(
            self.session.suspension_system.shock_damper,
            "Shock Super Deluxe",
            param,
            delta,
            key,
            label,
        )

    def _adjust_tokens(self, delta: int, key: str, note: str) -> None:
        """
        Steps the fork's bottomless token count.

        Args:
            delta: Token change, positive or negative.
            key: Key label for the printed line.
            note: Short explanation of what the change does.
        """
        air = self.session.air_spring
        new_tokens = max(0, min(air.specs.max_tokens, air.num_tokens + delta))
        air.set_tokens(new_tokens)
        print(f"\n[KEY {key}] Fork Volume Tokens: {new_tokens} ({note})")

    def _adjust_psi(self, delta: float, key: str) -> None:
        """
        Steps the fork's air pressure.

        Args:
            delta: Pressure change in psi.
            key: Key label for the printed line.
        """
        air = self.session.air_spring
        new_psi = max(10.0, min(200.0, air.gauge_pressure_psi + delta))
        air.set_pressure_psi(new_psi)
        print(f"\n[KEY {key}] Fork Air Pressure: {new_psi:.1f} PSI ({air.gauge_pressure_bar:.2f} bar)")

    def _adjust_target_speed(self, delta_kmh: float, key: str) -> None:
        """
        Steps the cruise target speed and reports it.

        Args:
            delta_kmh: Target change in km/h.
            key: Key label for the printed line.
        """
        new_kmh = self.session.adjust_target_speed(delta_kmh)
        print(f"\n[KEY {key}] Cruise Target -> {new_kmh:.1f} km/h")

    def _adjust_brake_strength(self, delta: float, key: str) -> None:
        """
        Steps the brake lever position the toggle applies.

        Args:
            delta: Change in lever position, in units of full demand.
            key: Key label for the printed line.
        """
        strength = self.session.adjust_brake_strength(delta)
        print(f"\n[KEY {key}] Brake Strength -> {strength * 100.0:.0f} % of the 200 N.m ceiling")

    def _on_toggle_braking(self) -> None:
        """Toggles the brakes and reports the new state."""
        braking = self.session.toggle_braking()
        strength = self.session.brake_strength
        print(f"\n[SPACE] Brakes: {'ON' if braking else 'OFF'} ({strength * 100.0:.0f} % demand)")

    def _on_reset_run(self) -> None:
        """Restarts the run from the solved static equilibrium."""
        self.session.reset_run()
        print("\n[KEY R] Run restarted from the solved static equilibrium.")

    def _on_cycle_camera(self) -> None:
        """Cycles the camera preset."""
        self.session.camera.cycle_mode()
        print(f"\n[KEY C] Camera View -> {self.session.camera.mode_name.upper()}")

    def _on_set_camera(self, mode: str, label: str) -> None:
        """
        Selects a camera preset.

        Args:
            mode: Camera preset key.
            label: Human-readable preset name for the printed line.
        """
        self.session.camera.set_mode(mode)
        print(f"\n[KEY {'1' if mode == '2d' else '2'}] Camera View -> {label}")

    def _on_cycle_preset(self) -> None:
        """Cycles the factory damper preset and reports every resulting click count."""
        new_preset = self.session.suspension_system.cycle_preset(forward=True)
        fd = self.session.suspension_system.fork_damper
        sd = self.session.suspension_system.shock_damper
        print(f"\n[KEY P] Damper Factory Preset -> {new_preset.value}")
        print(
            f"        Fork ZEB: HSC={fd.hsc_clicks}/4, LSC={fd.lsc_clicks}/14, "
            f"Reb={fd.rebound_clicks}/17"
        )
        print(
            f"        Shock SD: HSC={sd.hsc_clicks}/4, LSC={sd.lsc_clicks}/14, "
            f"Reb={sd.rebound_clicks}/14, HBO={sd.hbo_clicks}/4"
        )

    def _on_shock_toggle_lockout(self) -> None:
        """Toggles the shock's threshold lockout."""
        sd = self.session.suspension_system.shock_damper
        is_locked = sd.toggle_lockout()
        print(
            f"\n[KEY X] Rear Shock Threshold Lockout -> "
            f"{'FIRM / PEDAL PLATFORM' if is_locked else 'OPEN / DESCENT'}"
        )

    def _on_toggle_telemetry(self) -> None:
        """Toggles the live HUD stream."""
        self.session.show_telemetry = not self.session.show_telemetry
        print(f"\n[KEY T] Live Telemetry HUD: {'ON' if self.session.show_telemetry else 'OFF'}")


__all__ = ["RideInputHandler", "TARGET_SPEED_STEP_KMH", "BRAKE_STRENGTH_STEP"]
