"""
Simulation Input & Key Event Dispatcher.

Dispatches GLFW / MuJoCo keyboard interactions to simulator actions via a clean lookup table.
"""

from typing import Any, Callable, Dict, Tuple
import time


class PlaygroundInputHandler:
    """
    Handles and dispatches GLFW keyboard events to the simulation playground.
    """

    def __init__(self, playground: Any) -> None:
        self.pg = playground
        self._dispatch_map: Dict[int, Callable[[], None]] = self._build_dispatch_map()

    def handle_key(self, keycode: int) -> bool:
        """Dispatches a keycode to its corresponding action. Returns True if handled."""
        handler = self._dispatch_map.get(keycode)
        if handler:
            handler()
            return True
        return False

    def _build_dispatch_map(self) -> Dict[int, Callable[[], None]]:
        mapping: Dict[int, Callable[[], None]] = {}

        # Define declarative action definitions: (tuple_of_keys, action_callable)
        actions: list[Tuple[Tuple[int, ...], Callable[[], None]]] = [
            # System / Rider / Views
            ((66, 98), self.pg.toggle_rider),
            ((67, 99), self._on_cycle_camera),
            ((49, 321), lambda: self._on_set_camera("2d", "2D SIDE VIEW")),
            ((50, 322), lambda: self._on_set_camera("3d", "3D ISOMETRIC VIEW")),
            ((80, 112), self._on_cycle_preset),
            # Fork Charger 3 Damping Adjustments
            ((76, 108), lambda: self._adjust_fork("lsc", +1, "L", "LSC")),
            ((75, 107), lambda: self._adjust_fork("lsc", -1, "K", "LSC")),
            ((74, 106), lambda: self._adjust_fork("hsc", +1, "J", "HSC")),
            ((72, 104), lambda: self._adjust_fork("hsc", -1, "H", "HSC")),
            ((85, 117), lambda: self._adjust_fork("rebound", +1, "U", "Rebound")),
            ((89, 121), lambda: self._adjust_fork("rebound", -1, "Y", "Rebound")),
            # Shock Super Deluxe Damping Adjustments & Lockout
            ((88, 120), self._on_shock_toggle_lockout),
            ((48,), lambda: self._adjust_shock("lsc", +1, "0", "LSC")),
            ((57,), lambda: self._adjust_shock("lsc", -1, "9", "LSC")),
            ((56,), lambda: self._adjust_shock("hsc", +1, "8", "HSC")),
            ((55,), lambda: self._adjust_shock("hsc", -1, "7", "HSC")),
            ((54,), lambda: self._adjust_shock("rebound", +1, "6", "Rebound")),
            ((53,), lambda: self._adjust_shock("rebound", -1, "5", "Rebound")),
            ((52,), lambda: self._adjust_shock("hbo", +1, "4", "HBO")),
            ((51,), lambda: self._adjust_shock("hbo", -1, "3", "HBO")),
            # Stand Travel Adjustments
            ((87, 119), lambda: self._adjust_travel("target_rear_travel_mm", +5.0, "W", "Rear Wheel Lift")),
            ((83, 115), lambda: self._adjust_travel("target_rear_travel_mm", -5.0, "S", "Rear Wheel Lower")),
            ((265,), lambda: self._adjust_travel("target_fork_travel_mm", +5.0, "UP", "Fork Compress")),
            ((264,), lambda: self._adjust_travel("target_fork_travel_mm", -5.0, "DOWN", "Fork Extend")),
            # Air Spring Tuning
            ((91,), lambda: self._adjust_tokens(-1, "[", "Less end-stroke ramp-up")),
            ((93,), lambda: self._adjust_tokens(+1, "]", "More ramp-up / bottom-out resistance")),
            ((45,), lambda: self._adjust_psi(-2.0, "-")),
            ((61,), lambda: self._adjust_psi(+2.0, "=")),
            # Automation & State
            ((32,), self._on_toggle_auto_sweep),
            ((82, 114), self.pg.reset_state),
            ((84, 116), self._on_toggle_telemetry),
            ((71, 103), self.pg.toggle_debug_markers),
            ((47, 63), self.pg.print_help),
        ]

        for keys, action in actions:
            for k in keys:
                mapping[k] = action

        return mapping

    # --- Parameterized Helpers ---

    # The click attribute is `<param>_clicks`, but set_clicks names the rebound
    # circuit `reb`; everything else uses the same spelling for both.
    _SET_CLICKS_KWARG = {"rebound": "reb"}

    def _adjust_damper(self, damper: Any, damper_label: str, param: str, delta: int, key: str, label: str) -> None:
        # set_clicks owns the clamp, so write through it rather than assigning the
        # raw value first and correcting afterwards.
        kwarg = self._SET_CLICKS_KWARG.get(param, param)
        damper.set_clicks(**{kwarg: getattr(damper, f"{param}_clicks") + delta})
        val = getattr(damper, f"{param}_clicks")
        max_val = getattr(damper, f"max_{param[:3]}")
        print(f"\n[KEY {key}] {damper_label} {label} -> Click {val}/{max_val}")

    def _adjust_fork(self, param: str, delta: int, key: str, label: str) -> None:
        self._adjust_damper(
            self.pg.suspension_system.fork_damper, "Fork Charger 3", param, delta, key, label
        )

    def _adjust_shock(self, param: str, delta: int, key: str, label: str) -> None:
        self._adjust_damper(
            self.pg.suspension_system.shock_damper, "Shock Super Deluxe", param, delta, key, label
        )

    def _adjust_travel(self, target_attr: str, delta: float, key: str, label: str) -> None:
        self.pg.auto_sweep = False
        val = min(180.0, max(0.0, getattr(self.pg, target_attr) + delta))
        setattr(self.pg, target_attr, val)
        print(f"\n[KEY {key}] {label} -> {val:.1f} mm")

    def _adjust_tokens(self, delta: int, key: str, note: str) -> None:
        air = self.pg.air_spring
        new_tokens = max(0, min(air.specs.max_tokens, air.num_tokens + delta))
        air.set_tokens(new_tokens)
        print(f"\n[KEY {key}] Fork Volume Tokens: {new_tokens} ({note})")

    def _adjust_psi(self, delta: float, key: str) -> None:
        air = self.pg.air_spring
        new_psi = max(10.0, min(200.0, air.gauge_pressure_psi + delta))
        air.set_pressure_psi(new_psi)
        print(f"\n[KEY {key}] Fork Air Pressure: {new_psi:.1f} PSI ({air.gauge_pressure_bar:.2f} bar)")

    def _on_cycle_camera(self) -> None:
        self.pg.camera_manager.cycle_mode()
        print(f"\n[KEY C] Camera View -> {self.pg.camera_manager.mode_name.upper()}")

    def _on_set_camera(self, mode: str, label: str) -> None:
        self.pg.camera_manager.set_mode(mode)
        print(f"\n[KEY {'1' if mode == '2d' else '2'}] Camera View -> {label}")

    def _on_cycle_preset(self) -> None:
        new_preset = self.pg.suspension_system.cycle_preset(forward=True)
        fd = self.pg.suspension_system.fork_damper
        sd = self.pg.suspension_system.shock_damper
        print(f"\n[KEY P] Damper Factory Preset -> {new_preset.value}")
        print(f"        Fork ZEB: HSC={fd.hsc_clicks}/4, LSC={fd.lsc_clicks}/14, Reb={fd.rebound_clicks}/17")
        print(f"        Shock SD: HSC={sd.hsc_clicks}/4, LSC={sd.lsc_clicks}/14, Reb={sd.rebound_clicks}/14, HBO={sd.hbo_clicks}/4")

    def _on_shock_toggle_lockout(self) -> None:
        sd = self.pg.suspension_system.shock_damper
        is_locked = sd.toggle_lockout()
        print(f"\n[KEY X] Rear Shock Threshold Lockout -> {'FIRM / PEDAL PLATFORM' if is_locked else 'OPEN / DESCENT'}")

    def _on_toggle_auto_sweep(self) -> None:
        self.pg.auto_sweep = not self.pg.auto_sweep
        self.pg.sweep_start_time = time.time()
        state_str = "ENABLED (0 -> 180 mm sinusoidal cycle)" if self.pg.auto_sweep else "DISABLED"
        print(f"\n[SPACE] Auto-Sweep: {state_str}")

    def _on_toggle_telemetry(self) -> None:
        self.pg.show_telemetry = not self.pg.show_telemetry
        print(f"\n[KEY T] Live Telemetry HUD: {'ON' if self.pg.show_telemetry else 'OFF'}")
