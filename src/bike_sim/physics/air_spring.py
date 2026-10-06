"""
Pneumatic Air Spring Simulation Module for Suspension Fork.

This module models a high-performance mountain bike / eMTB air spring (e.g. Fox 38 / RockShox ZEB)
with dual air chambers (positive and negative), automatic equalization at top-out, volume spacers
(tokens), and polytropic/adiabatic gas compression dynamics (P * V^gamma = const).
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from bike_sim.physics.checks import derived, derived_array, integer, scalar
from bike_sim.physics.domain_validation import air_specs


PSI_TO_PA = 6894.757293168
PA_TO_PSI = 1.0 / PSI_TO_PA
BAR_TO_PA = 100000.0
PA_TO_BAR = 1.0 / BAR_TO_PA
ATM_PA = 101325.0


@dataclass
class AirSpringSpecs:
    """Dataclass holding geometric and thermodynamic specifications for a suspension air spring."""

    stanchion_inner_diam_mm: float = 24.0   # Inner air shaft bore inside 38mm stanchion ~ 24.0 mm
    total_travel_mm: float = 180.0          # Max travel in mm
    pos_chamber_length_mm: float = 280.0    # Baseline positive chamber length in mm (280mm air column)
    neg_chamber_length_mm: float = 55.0     # Negative chamber length in mm (~20% of total)
    token_volume_cm3: float = 8.0           # Volume displaced by one volume spacer token (8 cm^3)
    max_tokens: int = 4                     # Maximum allowable volume spacers
    default_tokens: int = 2                 # Default recommended tokens
    gamma: float = 1.40                     # Adiabatic index for diatomic air
    atm_pressure_pa: float = ATM_PA         # Atmospheric reference pressure in Pa

    def __post_init__(self):
        air_specs(self)

    @property
    def piston_area_m2(self) -> float:
        """Piston cross-sectional area in square meters."""
        air_specs(self)
        r_m = (self.stanchion_inner_diam_mm / 2.0) / 1000.0
        return derived(np.pi * (r_m ** 2), "AirSpringSpecs.piston_area_m2")

    @property
    def token_volume_m3(self) -> float:
        """Volume of one token in cubic meters."""
        air_specs(self)
        return derived(self.token_volume_cm3 * 1e-6, 'AirSpringSpecs.token_volume_m3')

    @property
    def base_pos_volume_m3(self) -> float:
        """Nominal uncompressed positive chamber volume with 0 tokens in m^3."""
        air_specs(self)
        l_pos_m = self.pos_chamber_length_mm / 1000.0
        return derived(self.piston_area_m2 * l_pos_m, 'AirSpringSpecs.base_pos_volume_m3')

    @property
    def base_neg_volume_m3(self) -> float:
        """Nominal uncompressed negative chamber volume at top-out in m^3."""
        air_specs(self)
        l_neg_m = self.neg_chamber_length_mm / 1000.0
        return derived(self.piston_area_m2 * l_neg_m, 'AirSpringSpecs.base_neg_volume_m3')


class ForkAirSpring:
    """
    Thermodynamic pneumatic air spring model with positive & negative chambers and volume tokens.
    """

    def __init__(
        self,
        specs: Optional[AirSpringSpecs] = None,
        num_tokens: Optional[int] = None,
        gauge_pressure_psi: float = 82.0,
    ) -> None:
        self.specs = specs if specs is not None else AirSpringSpecs()
        air_specs(self.specs)
        self.num_tokens = num_tokens if num_tokens is not None else self.specs.default_tokens
        self.num_tokens = max(0, min(self.specs.max_tokens, integer(self.num_tokens, "ForkAirSpring.num_tokens")))
        self.gauge_pressure_psi = scalar(gauge_pressure_psi, "ForkAirSpring.gauge_pressure_psi", minimum=0.)

    @property
    def abs_pressure_pa(self) -> float:
        """Initial equalized absolute pressure at top-out in Pa."""
        air_specs(self.specs)
        scalar(self.gauge_pressure_psi, 'ForkAirSpring.gauge_pressure_psi', minimum=0.)
        return derived(self.gauge_pressure_psi * PSI_TO_PA + self.specs.atm_pressure_pa, 'ForkAirSpring.abs_pressure_pa')

    @property
    def gauge_pressure_bar(self) -> float:
        """Gauge pressure in bar."""
        air_specs(self.specs)
        scalar(self.gauge_pressure_psi, 'ForkAirSpring.gauge_pressure_psi', minimum=0.)
        return derived(self.gauge_pressure_psi * PSI_TO_PA * PA_TO_BAR, 'ForkAirSpring.gauge_pressure_bar')

    def set_tokens(self, n_tokens: int) -> None:
        """Sets active token count constrained within [0, max_tokens]."""
        air_specs(self.specs)
        self.num_tokens = max(0, min(self.specs.max_tokens, integer(n_tokens, "ForkAirSpring.num_tokens")))

    def set_pressure_psi(self, psi: float) -> None:
        """Sets gauge air pressure in PSI."""
        air_specs(self.specs)
        self.gauge_pressure_psi = max(10.0, scalar(psi, "ForkAirSpring.gauge_pressure_psi", minimum=0.))

    def compute_volumes(
        self,
        travel_mm: float,
        num_tokens: Optional[int] = None,
    ) -> Tuple[float, float]:
        """
        Computes instantaneous positive and negative chamber volumes in m^3 at travel x (mm).
        """
        air_specs(self.specs)
        tokens = max(0, min(self.specs.max_tokens, integer(self.num_tokens if num_tokens is None else num_tokens, "ForkAirSpring.num_tokens")))
        travel_m = max(0.0, min(self.specs.total_travel_mm, scalar(travel_mm, "ForkAirSpring.travel_mm"))) / 1000.0

        v_pos_0 = self.specs.base_pos_volume_m3 - (tokens * self.specs.token_volume_m3)
        v_neg_0 = self.specs.base_neg_volume_m3

        v_disp = self.specs.piston_area_m2 * travel_m
        derived(v_disp, 'ForkAirSpring.displaced_volume')

        derived(v_pos_0, "ForkAirSpring.initial_positive_volume")
        derived(v_neg_0, "ForkAirSpring.initial_negative_volume")
        if v_pos_0 <= 0. or v_neg_0 <= 0. or v_disp >= v_pos_0:
            raise ValueError(
                f"Pneumatic volume exhausted at travel={travel_mm:.1f} mm with {tokens} tokens "
                f"(displaced volume {v_disp*1e6:.1f} cm³ >= initial chamber volume {v_pos_0*1e6:.1f} cm³)."
            )

        v_pos = v_pos_0 - v_disp
        v_neg = v_neg_0 + v_disp
        if v_pos <= 0. or v_neg <= 0.:
            raise ValueError("ForkAirSpring.volumes: chamber volumes must be positive")
        return derived(v_pos, "ForkAirSpring.positive_volume"), derived(v_neg, "ForkAirSpring.negative_volume")

    def compute_pressures_pa(
        self,
        travel_mm: float,
        p_gauge_psi: Optional[float] = None,
        num_tokens: Optional[int] = None,
    ) -> Tuple[float, float]:
        """
        Computes absolute pressures P_pos and P_neg in Pa at travel x (mm).
        """
        air_specs(self.specs)
        tokens = max(0, min(self.specs.max_tokens, integer(self.num_tokens if num_tokens is None else num_tokens, "ForkAirSpring.num_tokens")))
        psi = self.gauge_pressure_psi if p_gauge_psi is None else p_gauge_psi
        scalar(psi, "ForkAirSpring.gauge_pressure_psi", minimum=0.)
        p_abs_0 = psi * PSI_TO_PA + self.specs.atm_pressure_pa
        derived(p_abs_0, "ForkAirSpring.absolute_pressure")

        v_pos_0 = self.specs.base_pos_volume_m3 - (tokens * self.specs.token_volume_m3)
        v_neg_0 = self.specs.base_neg_volume_m3

        v_pos_x, v_neg_x = self.compute_volumes(travel_mm, tokens)

        gamma = self.specs.gamma
        p_pos = p_abs_0 * ((v_pos_0 / v_pos_x) ** gamma)
        p_neg = p_abs_0 * ((v_neg_0 / v_neg_x) ** gamma)
        return derived(p_pos, "ForkAirSpring.positive_pressure"), derived(p_neg, "ForkAirSpring.negative_pressure")

    def compute_axial_force(
        self,
        travel_mm: float,
        p_gauge_psi: Optional[float] = None,
        num_tokens: Optional[int] = None,
    ) -> float:
        """
        Computes the net spring axial force (N) acting along the fork stanchions.
        """
        p_pos, p_neg = self.compute_pressures_pa(travel_mm, p_gauge_psi, num_tokens)
        area = self.specs.piston_area_m2
        net_force = area * (p_pos - p_neg)
        return max(0.0, derived(net_force, "ForkAirSpring.axial_force"))

    def compute_instantaneous_stiffness(
        self,
        travel_mm: float,
        p_gauge_psi: Optional[float] = None,
        num_tokens: Optional[int] = None,
        delta_mm: float = 0.5,
    ) -> float:
        """
        Computes the instantaneous tangent spring rate k(x) = dF/dx in N/mm (or kN/m).
        """
        scalar(delta_mm, "ForkAirSpring.delta_mm", positive=True)
        x = scalar(travel_mm, "ForkAirSpring.travel_mm")
        x1 = max(0.0, x - delta_mm)
        x2 = min(self.specs.total_travel_mm, x + delta_mm)
        f1 = self.compute_axial_force(x1, p_gauge_psi, num_tokens)
        f2 = self.compute_axial_force(x2, p_gauge_psi, num_tokens)
        if x2 == x1:
            raise ValueError("ForkAirSpring.delta_mm: zero finite difference interval")
        return derived((f2 - f1) / (x2 - x1), "ForkAirSpring.stiffness")

    def calibrate_psi_for_sag(
        self,
        target_sag_mm: float = 54.0,
        target_axial_force_n: float = 322.2,
        num_tokens: Optional[int] = None,
    ) -> float:
        """
        Analytically calibrates and sets the required gauge PSI to achieve exact target sag.
        """
        air_specs(self.specs)
        scalar(target_axial_force_n, 'ForkAirSpring.target_axial_force_n', minimum=0.)
        tokens = max(0, min(self.specs.max_tokens, integer(self.num_tokens if num_tokens is None else num_tokens, "ForkAirSpring.num_tokens")))

        v_pos_0 = self.specs.base_pos_volume_m3 - (tokens * self.specs.token_volume_m3)
        v_neg_0 = self.specs.base_neg_volume_m3
        v_pos_sag, v_neg_sag = self.compute_volumes(target_sag_mm, tokens)

        gamma = self.specs.gamma
        pos_ratio = (v_pos_0 / v_pos_sag) ** gamma
        neg_ratio = (v_neg_0 / v_neg_sag) ** gamma
        delta_ratio = pos_ratio - neg_ratio
        derived(delta_ratio, 'ForkAirSpring.calibration_ratio')

        if delta_ratio <= 0.0:
            raise ValueError(
                f"Invalid sag calibration at target_sag={target_sag_mm:.1f} mm: "
                f"negative chamber expansion ratio ({neg_ratio:.4f}) >= positive ratio ({pos_ratio:.4f})."
            )

        area = self.specs.piston_area_m2
        denominator = area * delta_ratio
        derived(denominator, 'ForkAirSpring.calibration_denominator')
        p_abs_0_required = target_axial_force_n / denominator
        derived(p_abs_0_required, 'ForkAirSpring.calibration_absolute_pressure')
        p_gauge_pa = p_abs_0_required - self.specs.atm_pressure_pa
        derived(p_gauge_pa, 'ForkAirSpring.calibration_gauge_pressure')
        calibrated_psi = max(10.0, derived(p_gauge_pa * PA_TO_PSI, 'ForkAirSpring.calibration_raw_psi'))

        derived(calibrated_psi, "ForkAirSpring.calibrated_pressure")
        self.gauge_pressure_psi = calibrated_psi
        return calibrated_psi

    def compute_force_curve(
        self,
        n_points: int = 101,
        p_gauge_psi: Optional[float] = None,
        num_tokens: Optional[int] = None,
    ) -> Dict[str, np.ndarray]:
        """
        Generates comprehensive force, pressure, stiffness, and energy curves across 0..180 mm travel.
        """
        travel_array = np.linspace(0.0, self.specs.total_travel_mm, n_points)
        force_list = []
        stiffness_list = []
        pos_press_list = []
        neg_press_list = []

        for x in travel_array:
            f = self.compute_axial_force(x, p_gauge_psi, num_tokens)
            k = self.compute_instantaneous_stiffness(x, p_gauge_psi, num_tokens)
            p_pos, p_neg = self.compute_pressures_pa(x, p_gauge_psi, num_tokens)

            force_list.append(f)
            stiffness_list.append(k)
            pos_press_list.append((p_pos - self.specs.atm_pressure_pa) * PA_TO_PSI)
            neg_press_list.append((p_neg - self.specs.atm_pressure_pa) * PA_TO_PSI)

        force_arr = np.array(force_list, dtype=float)
        travel_m = travel_array / 1000.0
        # Cumulative trapezoid: equivalent to re-integrating every prefix, but O(n)
        # instead of O(n^2).
        energy_arr = np.concatenate((
            [0.0],
            np.cumsum(np.diff(travel_m) * (force_arr[1:] + force_arr[:-1]) * 0.5),
        ))
        derived_array(energy_arr, 'ForkAirSpring.energy_j')

        return {
            "travel_mm": travel_array,
            "axial_force_n": force_arr,
            "stiffness_n_mm": np.array(stiffness_list, dtype=float),
            "pos_pressure_psi": np.array(pos_press_list, dtype=float),
            "neg_pressure_psi": np.array(neg_press_list, dtype=float),
            "energy_j": energy_arr,
            "max_force_n": float(force_arr[-1]),
            "max_energy_j": float(energy_arr[-1]),
            "stiffness_at_0_n_mm": float(stiffness_list[0]),
            "stiffness_at_sag_n_mm": float(self.compute_instantaneous_stiffness(54.0, p_gauge_psi, num_tokens)),
            "stiffness_at_bottom_n_mm": float(stiffness_list[-1]),
            "progressivity_pct": derived(((stiffness_list[-1] - stiffness_list[0]) / max(0.1, stiffness_list[0])) * 100.0, 'ForkAirSpring.progressivity_pct'),
        }

    def fit_polynomial_mjcf(
        self,
        order: int = 3,
        num_tokens: Optional[int] = None,
        p_gauge_psi: Optional[float] = None,
    ) -> np.ndarray:
        """
        Fits a polynomial F(x) = c1*x + c2*x^2 + c3*x^3 (with F(0)=0) to match the pneumatic force curve.
        """
        curve = self.compute_force_curve(n_points=101, p_gauge_psi=p_gauge_psi, num_tokens=num_tokens)
        travel_m = curve["travel_mm"] / 1000.0
        force_n = curve["axial_force_n"]

        A = np.column_stack([travel_m ** i for i in range(1, order + 1)])
        coeffs, _, _, _ = np.linalg.lstsq(A, force_n, rcond=None)
        return coeffs
