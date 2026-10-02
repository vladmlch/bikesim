"""
Simulation HUD and Display Manager.

Provides terminal HUD formatting and interactive control help text.
"""

import sys
from typing import Any, Dict


class PlaygroundHUDManager:
    """
    Manages terminal HUD output and interactive controls documentation.
    """

    @staticmethod
    def print_hud(tel: Dict[str, Any], auto_sweep: bool = False) -> None:
        """Prints a single-line live telemetry HUD to terminal."""
        lock_str = "FIRM" if tel["shock_lockout"] else "OPEN"
        damp_str = f"ZEB:[H{tel['fork_hsc']} L{tel['fork_lsc']} R{tel['fork_reb']}] Shk:[H{tel['shock_hsc']} L{tel['shock_lsc']} R{tel['shock_reb']} B{tel['shock_hbo']}|{lock_str}]"
        preset_str = tel["damper_preset"]
        rider_str = "Rider:ON" if tel["rider_active"] else "Rider:OFF"
        sweep_str = " [AUTO-SWEEP ON]" if auto_sweep else ""

        hud_line = (
            f"\r[STAND{sweep_str}|{rider_str}|{preset_str}] "
            f"Fork:{tel['fork_travel_mm']:5.1f}mm (Air:{tel['fork_air_psi']:4.1f}PSI, {tel['fork_tokens']}tok, Fd:{tel['fork_damper_n']:+4.0f}N) | "
            f"Shock:{tel['shock_stroke_mm']:4.1f}/65mm ({tel['shock_compression_pct']:4.1f}%, Fd:{tel['shock_damper_n']:+4.0f}N) | "
            f"{damp_str}"
        )
        sys.stdout.write(hud_line)
        sys.stdout.flush()

    @staticmethod
    def get_help_text() -> str:
        """Returns interactive control help string."""
        return """
================================================================================
           2D BIKE SUSPENSION PLAYGROUND - TEST STAND CONTROLS
================================================================================
  --- SUSPENSION TRAVEL CONTROLS ---
    W / S           : Rear Wheel Travel UP / DOWN (+/- 5 mm)
    UP / DOWN       : Front Fork Travel COMPRESS / EXTEND (+/- 5 mm)
    SPACE           : Toggle Auto-Sweep (Sinusoidal 0 -> 180 mm sweep)
    R               : Reset Stand to 0 mm Travel (Uncompressed)

  --- PNEUMATIC AIR SPRING (FORK) ---
    [ / ]           : Bottomless Tokens DECREASE / INCREASE (Ramp-up)
    - / =           : Fork Air Pressure -/+ 2.0 PSI

  --- ROCKSHOX CHARGER 3 DAMPER (FORK) ---
    J / H           : Fork High-Speed Compression (HSC) + / - (0 to 4 clicks)
    L / K           : Fork Low-Speed Compression (LSC) + / - (0 to 14 clicks)
    U / Y           : Fork Rebound (Reb) Slower / Faster (0 to 17 clicks)

  --- ROCKSHOX SUPER DELUXE DAMPER (REAR SHOCK) ---
    8 / 7           : Shock High-Speed Compression (HSC) + / - (0 to 4 clicks)
    0 / 9           : Shock Low-Speed Compression (LSC) + / - (0 to 14 clicks)
    6 / 5           : Shock Rebound (Reb) Slower / Faster (0 to 14 clicks)
    4 / 3           : Shock Hydraulic Bottom-Out (HBO) + / - (0 to 4 clicks)
    X               : Shock Threshold Lockout TOGGLE (Open / Firm Platform)
    P               : Factory Damper Preset CYCLE (Base / Plush / Firm / Climb)

  --- VISUALIZATION & VIEWPORT ---
    C / 1 / 2       : Camera Angle (Cycle / 2D Side View / 3D Isometric)
    B               : Toggle Rider Mass & Model (Stand Sag Load: 75 kg)
    G               : Toggle Pivot Markers (Yellow sphere debug indicators)
    T               : Toggle Live Telemetry Stream
    ? / /           : Show This Help Screen
    ESC             : Close Viewer & Exit Playground
================================================================================
"""

    @classmethod
    def print_help(cls) -> None:
        """Displays interactive control help in terminal."""
        print(cls.get_help_text())
