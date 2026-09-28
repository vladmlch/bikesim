"""
Hydrodynamic Suspension Damper Simulation Module for Mountain Bike Suspensions.

This module models high-performance mountain bike hydraulic dampers:
1. RockShox ZEB Ultimate Fork Damper (Charger 3 / 3.1 RC2)
2. RockShox Super Deluxe Ultimate Rear Damper (RC2T)
"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
import numpy as np


class DamperPreset(Enum):
    BASE = "Base Tune (Factory Neutral)"
    PLUSH = "Plush / Wet Traction"
    ENDURO = "Enduro Race (Supportive)"
    PARK = "Bike Park / Jumps (Firm)"
    CUSTOM = "Custom Rider Tune"


@dataclass
class DamperClickConfig:
    """Dataclass holding discrete click adjustments for a suspension damper."""

    # Fork (Charger 3 RC2)
    fork_hsc: int = 2          # 0..4 (0=Open, 2=Neutral, 4=Firm)
    fork_lsc: int = 7          # 0..14 (0=Open, 7=Neutral, 14=Firm)
    fork_rebound: int = 9      # 0..17 (0=Fast/Open, 9=Neutral, 17=Slow/Firm)

    # Shock (Super Deluxe RC2T)
    shock_hsc: int = 2         # 0..4 (0=Open, 2=Neutral, 4=Firm)
    shock_lsc: int = 7         # 0..14 (0=Open, 7=Neutral, 14=Firm)
    shock_rebound: int = 7     # 0..14 (0=Fast/Open, 7=Neutral, 14=Slow/Firm)
    shock_hbo: int = 2         # 0..4 (0=Min HBO, 2=Neutral, 4=Max HBO)
    shock_lockout: bool = False  # False=Open, True=Firm Pedal Platform

    preset_name: str = DamperPreset.BASE.value


DAMPER_PRESETS: Dict[DamperPreset, DamperClickConfig] = {
    # BASE is the factory tune: exactly the DamperClickConfig field defaults.
    DamperPreset.BASE: DamperClickConfig(),
    DamperPreset.PLUSH: DamperClickConfig(
        fork_hsc=0,
        fork_lsc=3,
        fork_rebound=5,
        shock_hsc=0,
        shock_lsc=3,
        shock_rebound=4,
        shock_hbo=1,
        shock_lockout=False,
        preset_name=DamperPreset.PLUSH.value,
    ),
    DamperPreset.ENDURO: DamperClickConfig(
        fork_hsc=3,
        fork_lsc=10,
        fork_rebound=11,
        shock_hsc=3,
        shock_lsc=10,
        shock_rebound=9,
        shock_hbo=3,
        shock_lockout=False,
        preset_name=DamperPreset.ENDURO.value,
    ),
    DamperPreset.PARK: DamperClickConfig(
        fork_hsc=4,
        fork_lsc=13,
        fork_rebound=14,
        shock_hsc=4,
        shock_lsc=13,
        shock_rebound=12,
        shock_hbo=4,
        shock_lockout=False,
        preset_name=DamperPreset.PARK.value,
    ),
}


class BaseDamper:
    """Base class for hydraulic suspension dampers with click-adjustable damping circuits."""

    def __init__(
        self,
        max_hsc: int,
        max_lsc: int,
        max_reb: int,
        hsc_clicks: int,
        lsc_clicks: int,
        rebound_clicks: int,
        total_travel_mm: float,
        c_lsc_min: float,
        c_lsc_max: float,
        c_hsc_min: float,
        c_hsc_max: float,
        c_reb_min: float,
        c_reb_max: float,
        v_knee_comp: float,
        v_knee_reb: float,
    ) -> None:
        self.max_hsc = max_hsc
        self.max_lsc = max_lsc
        self.max_reb = max_reb
        self.total_travel_mm = float(total_travel_mm)

        self.hsc_clicks = max(0, min(self.max_hsc, int(hsc_clicks)))
        self.lsc_clicks = max(0, min(self.max_lsc, int(lsc_clicks)))
        self.rebound_clicks = max(0, min(self.max_reb, int(rebound_clicks)))

        self.c_lsc_min = c_lsc_min
        self.c_lsc_max = c_lsc_max
        self.c_hsc_min = c_hsc_min
        self.c_hsc_max = c_hsc_max
        self.c_reb_min = c_reb_min
        self.c_reb_max = c_reb_max

        self.v_knee_comp = v_knee_comp
        self.v_knee_reb = v_knee_reb

    def _calc_fractions_and_coeffs(self) -> Tuple[float, float, float, float, float, float]:
        """Calculates click ratios and interpolated damping rates."""
        frac_lsc = self.lsc_clicks / float(self.max_lsc)
        frac_hsc = self.hsc_clicks / float(self.max_hsc)
        frac_reb = self.rebound_clicks / float(self.max_reb)

        c_lsc = self.c_lsc_min + (self.c_lsc_max - self.c_lsc_min) * (frac_lsc ** 1.25)
        c_hsc = self.c_hsc_min + (self.c_hsc_max - self.c_hsc_min) * (frac_hsc ** 1.15)
        c_reb = self.c_reb_min + (self.c_reb_max - self.c_reb_min) * (frac_reb ** 1.20)
        return frac_lsc, frac_hsc, frac_reb, float(c_lsc), float(c_hsc), float(c_reb)

    def _compute_base_damping(self, v: float, c_lsc: float, c_hsc: float, c_reb: float) -> float:
        """Computes low/high-speed compression or rebound damping force."""
        if v >= 0.0:
            if v <= self.v_knee_comp:
                return c_lsc * v * (1.0 + 0.35 * (v / self.v_knee_comp))
            else:
                f_knee = c_lsc * self.v_knee_comp * 1.35
                v_excess = v - self.v_knee_comp
                return f_knee + c_hsc * (v_excess ** 0.88) * (self.v_knee_comp ** 0.12)
        else:
            v_abs = -v
            if v_abs <= self.v_knee_reb:
                return -c_reb * v_abs * (1.0 + 0.25 * (v_abs / self.v_knee_reb))
            else:
                f_knee = c_reb * self.v_knee_reb * 1.25
                v_excess = v_abs - self.v_knee_reb
                c_hs_reb = c_reb * 0.45
                return -(f_knee + c_hs_reb * (v_excess ** 0.90) * (self.v_knee_reb ** 0.10))


class Charger3Damper(BaseDamper):
    """
    RockShox Charger 3 / 3.1 RC2 Damper Model for ZEB Ultimate Fork.
    """

    def __init__(
        self,
        hsc_clicks: int = 2,
        lsc_clicks: int = 7,
        rebound_clicks: int = 9,
        total_travel_mm: float = 180.0,
    ) -> None:
        super().__init__(
            max_hsc=4,
            max_lsc=14,
            max_reb=17,
            hsc_clicks=hsc_clicks,
            lsc_clicks=lsc_clicks,
            rebound_clicks=rebound_clicks,
            total_travel_mm=total_travel_mm,
            c_lsc_min=260.0,
            c_lsc_max=1150.0,
            c_hsc_min=180.0,
            c_hsc_max=520.0,
            c_reb_min=350.0,
            c_reb_max=1550.0,
            v_knee_comp=0.18,
            v_knee_reb=0.22,
        )
        self.hbo_start_mm = 160.0
        self.c_hbo_base = 3500.0

    def set_clicks(self, hsc: Optional[int] = None, lsc: Optional[int] = None, reb: Optional[int] = None) -> None:
        """Sets damper click adjustments."""
        if hsc is not None:
            self.hsc_clicks = max(0, min(self.max_hsc, int(hsc)))
        if lsc is not None:
            self.lsc_clicks = max(0, min(self.max_lsc, int(lsc)))
        if reb is not None:
            self.rebound_clicks = max(0, min(self.max_reb, int(reb)))

    def get_effective_coefficients(self) -> Dict[str, float]:
        """Calculates instantaneous damping rates based on active clicks."""
        frac_lsc, frac_hsc, frac_reb, c_lsc, c_hsc, c_reb = self._calc_fractions_and_coeffs()
        return {
            "c_lsc": c_lsc,
            "c_hsc": c_hsc,
            "c_reb": c_reb,
            "frac_lsc": float(frac_lsc),
            "frac_hsc": float(frac_hsc),
            "frac_reb": float(frac_reb),
        }

    def compute_damping_force(self, velocity_mps: float, travel_mm: float = 50.0) -> float:
        """Computes instantaneous hydraulic damping force in Newtons."""
        v = float(velocity_mps)
        coeffs = self.get_effective_coefficients()
        f_damp = self._compute_base_damping(v, coeffs["c_lsc"], coeffs["c_hsc"], coeffs["c_reb"])

        if travel_mm > self.hbo_start_mm and v > 0.0:
            hbo_prog = max(0.0, min(1.0, (travel_mm - self.hbo_start_mm) / (self.total_travel_mm - self.hbo_start_mm)))
            f_damp += self.c_hbo_base * (hbo_prog ** 2.0) * v

        return float(f_damp)

    def compute_dyno_curve(self, v_max: float = 2.5, n_points: int = 101, travel_mm: float = 90.0) -> Dict[str, np.ndarray]:
        """Generates full force vs velocity dyno sweep curve across [-v_max, +v_max]."""
        vel_arr = np.linspace(-v_max, v_max, n_points)
        force_arr = np.array([self.compute_damping_force(v, travel_mm) for v in vel_arr], dtype=float)
        return {
            "velocity_mps": vel_arr,
            "force_n": force_arr,
            "hsc_clicks": self.hsc_clicks,
            "lsc_clicks": self.lsc_clicks,
            "rebound_clicks": self.rebound_clicks,
        }


class SuperDeluxeDamper(BaseDamper):
    """
    RockShox Super Deluxe Ultimate RC2T Rear Shock Damper Model.
    """

    def __init__(
        self,
        hsc_clicks: int = 2,
        lsc_clicks: int = 7,
        rebound_clicks: int = 7,
        hbo_clicks: int = 2,
        lockout_firm: bool = False,
        total_stroke_mm: float = 65.0,
        legacy_behavior: bool = False,
    ) -> None:
        super().__init__(
            max_hsc=4,
            max_lsc=14,
            max_reb=14,
            hsc_clicks=hsc_clicks,
            lsc_clicks=lsc_clicks,
            rebound_clicks=rebound_clicks,
            total_travel_mm=total_stroke_mm,
            c_lsc_min=650.0,
            c_lsc_max=2900.0,
            c_hsc_min=450.0,
            c_hsc_max=1450.0,
            c_reb_min=900.0,
            c_reb_max=8000.0,
            v_knee_comp=0.16,
            v_knee_reb=0.20,
        )
        self.max_hbo = 4
        self.total_stroke_mm = float(total_stroke_mm)
        self.hbo_clicks = max(0, min(self.max_hbo, int(hbo_clicks)))
        self.lockout_firm = bool(lockout_firm)
        self.legacy_behavior = bool(legacy_behavior)

        self.hbo_start_mm = 52.0
        self.c_hbo_min = 4000.0
        self.c_hbo_max = 18000.0
        self.lockout_preload_n = 480.0
        self.lockout_stiffness = 15000.0

    def set_clicks(
        self,
        hsc: Optional[int] = None,
        lsc: Optional[int] = None,
        reb: Optional[int] = None,
        hbo: Optional[int] = None,
        lockout: Optional[bool] = None,
    ) -> None:
        """Sets rear shock click adjustments."""
        if hsc is not None:
            self.hsc_clicks = max(0, min(self.max_hsc, int(hsc)))
        if lsc is not None:
            self.lsc_clicks = max(0, min(self.max_lsc, int(lsc)))
        if reb is not None:
            self.rebound_clicks = max(0, min(self.max_reb, int(reb)))
        if hbo is not None:
            self.hbo_clicks = max(0, min(self.max_hbo, int(hbo)))
        if lockout is not None:
            self.lockout_firm = bool(lockout)

    def toggle_lockout(self) -> bool:
        """Toggles threshold lockout state between Open and Firm."""
        self.lockout_firm = not self.lockout_firm
        return self.lockout_firm

    def get_effective_coefficients(self) -> Dict[str, float]:
        """Calculates instantaneous damping rates based on active clicks."""
        frac_lsc, frac_hsc, frac_reb, c_lsc, c_hsc, c_reb = self._calc_fractions_and_coeffs()
        frac_hbo = self.hbo_clicks / float(self.max_hbo)
        c_hbo = self.c_hbo_min + (self.c_hbo_max - self.c_hbo_min) * (frac_hbo ** 1.30)

        return {
            "c_lsc": c_lsc,
            "c_hsc": c_hsc,
            "c_reb": c_reb,
            "c_hbo": float(c_hbo),
            "frac_lsc": float(frac_lsc),
            "frac_hsc": float(frac_hsc),
            "frac_reb": float(frac_reb),
            "frac_hbo": float(frac_hbo),
            "lockout_firm": self.lockout_firm,
        }

    def compute_damping_force(self, velocity_mps: float, stroke_mm: float = 20.0) -> float:
        """Computes instantaneous rear shock damping force in Newtons."""
        if self.legacy_behavior:
            return self._compute_legacy_damping_force(velocity_mps, stroke_mm)

        v = float(velocity_mps)
        coeffs = self.get_effective_coefficients()

        if self.lockout_firm and v > 0.0:
            knee = 0.03
            low = self.lockout_stiffness
            high = 1.8 * coeffs["c_hsc"]
            f_damp = low * v if v < knee else low * knee + high * (v - knee)
        else:
            f_damp = self._compute_base_damping(v, coeffs["c_lsc"], coeffs["c_hsc"], coeffs["c_reb"])

        if stroke_mm > self.hbo_start_mm and v > 0.0:
            fraction = min(1.0, (stroke_mm - self.hbo_start_mm) / (self.total_stroke_mm - self.hbo_start_mm))
            f_damp += coeffs["c_hbo"] * fraction ** 2 * v

        return float(f_damp)

    def _compute_legacy_damping_force(self, velocity_mps: float, stroke_mm: float = 20.0) -> float:
        v = float(velocity_mps)
        coeffs = self.get_effective_coefficients()

        if self.lockout_firm and v > 0.0:
            if v < 0.03:
                return float(self.lockout_stiffness * v)
            return float(self.lockout_preload_n + (coeffs["c_hsc"] * 1.8) * (v - 0.03))

        f_damp = self._compute_base_damping(v, coeffs["c_lsc"], coeffs["c_hsc"], coeffs["c_reb"])

        if stroke_mm > self.hbo_start_mm and v > 0.0:
            hbo_prog = max(0.0, min(1.0, (stroke_mm - self.hbo_start_mm) / (self.total_stroke_mm - self.hbo_start_mm)))
            f_damp += coeffs["c_hbo"] * (hbo_prog ** 2.0) * v

        return float(f_damp)

    def compute_dyno_curve(self, v_max: float = 2.0, n_points: int = 101, stroke_mm: float = 30.0) -> Dict[str, np.ndarray]:
        """Generates full force vs velocity dyno sweep curve across [-v_max, +v_max]."""
        vel_arr = np.linspace(-v_max, v_max, n_points)
        force_arr = np.array([self.compute_damping_force(v, stroke_mm) for v in vel_arr], dtype=float)
        return {
            "velocity_mps": vel_arr,
            "force_n": force_arr,
            "hsc_clicks": self.hsc_clicks,
            "lsc_clicks": self.lsc_clicks,
            "rebound_clicks": self.rebound_clicks,
            "hbo_clicks": self.hbo_clicks,
            "lockout": self.lockout_firm,
        }



class BikeSuspensionSystem:
    """
    Combined suspension system holding both Fork (ZEB Ultimate) and Rear Shock (Super Deluxe Ultimate).
    """

    def __init__(self, click_config: Optional[DamperClickConfig] = None) -> None:
        if click_config is None:
            click_config = DAMPER_PRESETS[DamperPreset.BASE]

        self.current_preset = DamperPreset.BASE
        self.config = click_config
        self.fork_damper = Charger3Damper(
            hsc_clicks=click_config.fork_hsc,
            lsc_clicks=click_config.fork_lsc,
            rebound_clicks=click_config.fork_rebound,
        )
        self.shock_damper = SuperDeluxeDamper(
            hsc_clicks=click_config.shock_hsc,
            lsc_clicks=click_config.shock_lsc,
            rebound_clicks=click_config.shock_rebound,
            hbo_clicks=click_config.shock_hbo,
            lockout_firm=click_config.shock_lockout,
        )

    def apply_preset(self, preset: DamperPreset) -> None:
        """Applies a factory preset configuration."""
        if preset not in DAMPER_PRESETS:
            return
        self.current_preset = preset
        cfg = DAMPER_PRESETS[preset]
        self.config = cfg
        self.fork_damper.set_clicks(hsc=cfg.fork_hsc, lsc=cfg.fork_lsc, reb=cfg.fork_rebound)
        self.shock_damper.set_clicks(
            hsc=cfg.shock_hsc,
            lsc=cfg.shock_lsc,
            reb=cfg.shock_rebound,
            hbo=cfg.shock_hbo,
            lockout=cfg.shock_lockout,
        )

    def cycle_preset(self, forward: bool = True) -> DamperPreset:
        """Cycles to next / previous preset."""
        preset_list = [DamperPreset.BASE, DamperPreset.PLUSH, DamperPreset.ENDURO, DamperPreset.PARK]
        try:
            curr_idx = preset_list.index(self.current_preset)
        except ValueError:
            curr_idx = 0

        next_idx = (curr_idx + 1) % len(preset_list) if forward else (curr_idx - 1) % len(preset_list)
        new_preset = preset_list[next_idx]
        self.apply_preset(new_preset)
        return new_preset
