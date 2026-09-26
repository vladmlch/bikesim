"""
Pneumatic Tyre Specifications and Literature Targets.

The parameters of the `pneumatic` tyre model (docs/RIDE.md sections 3.1 and 4.1) and the
measured quantities it is fitted to and tested against (section 11.2). A pure calculator in
this package's convention -- millimetres and bar at the public boundary, no MuJoCo. The
per-step force kernels live in `bike_sim.sim.ride.tyre` and work in metres.

Three groups of numbers live here:

- **`TyreSpecs`** -- one tyre: geometry, pressure, and the model constants. The fitted ones
  (`area_factor`, `carcass_stiffness_n_mm2`, `contact_length_factor`, `rate_stiffening`,
  `loss_factor`) are set by the calibration of plan task 5 and recorded in RIDE.md §3.1.
- **`TyreConfig`** -- which tyre model a run uses, at which tier, on which tyres, with an
  optional surface override. The one object the simulation and the CLI pass around.
- **Targets** -- the literature values (static stiffness law, contact length, dynamic
  stiffness ratio, damping band, rim-strike band, drum rolling resistance), with the
  tolerances the tests hold the model to.
"""

from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

BAR_TO_PA = 100000.0

TYRE_MODELS: Tuple[str, ...] = ("sphere", "pneumatic")
DEFAULT_TYRE_MODEL = "sphere"

# Live pressure adjustment from the viewer (docs/RIDE.md, Usage: viewer key map).
PRESSURE_MIN_BAR = 0.8
PRESSURE_MAX_BAR = 3.0
PRESSURE_STEP_BAR = 0.05


@dataclass(frozen=True)
class TyreSpecs:
    """
    One pneumatic tyre on its rim.

    Attributes:
        name: Tyre, for reports.
        outer_radius_mm: Unloaded outer radius; equals the wheel radius in `BikeSpecs`.
        rim_radius_mm: Radius of the rim the tyre sits on; equals the builder's rim geom.
        section_height_mm: Nominal section height, used only to place the rim-strike
            deflection against the literature band.
        pressure_bar: Gauge inflation pressure.
        crr_reference: Drum rolling-resistance coefficient at `CRR_REFERENCE` conditions.
        casing_width_mm: Casing width `W_c`; the loaded contact width saturates at it.
        compressed_casing_mm: Casing plus tread left between rim and road at rim strike.
        area_factor: Effective-area factor `c_A` on the pressure term (fitted).
        carcass_stiffness_n_mm2: Carcass stiffness `k_c` per unit arc length (fitted).
        contact_length_factor: `c_L`, geometric footprint span to measured length (fitted).
        rate_stiffening: `k_r`, Maxwell-branch stiffness as a fraction of the elastic
            element stiffness (fitted).
        rate_relaxation_s: Maxwell-branch relaxation time `tau`.
        loss_factor: Rate-independent hysteresis `eta` (fitted).
        hysteresis_rate_eps_mps: Regularisation rate `delta_dot_eps` of the hysteresis sign.
        relaxation_length_mm: Longitudinal relaxation length `sigma`.
        rim_stiffness_n_mm2: Rim-contact stiffness `k_rim` per unit arc length.
        tread_loss_crr: Tread-loss term, zero unless the section 3.1 fallback is needed.
    """

    name: str
    outer_radius_mm: float
    rim_radius_mm: float
    section_height_mm: float
    pressure_bar: float
    crr_reference: float
    casing_width_mm: float = 60.0
    compressed_casing_mm: float = 6.0
    area_factor: float = 0.46
    carcass_stiffness_n_mm2: float = 0.10
    contact_length_factor: float = 0.86
    rate_stiffening: float = 0.25
    rate_relaxation_s: float = 0.2
    loss_factor: float = 0.07
    hysteresis_rate_eps_mps: float = 0.01
    relaxation_length_mm: float = 90.0
    rim_stiffness_n_mm2: float = 28.0
    tread_loss_crr: float = 0.0

    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If a dimension or constant is out of its physical range.
        """
        positive = (
            "outer_radius_mm", "rim_radius_mm", "section_height_mm", "pressure_bar",
            "crr_reference", "casing_width_mm", "compressed_casing_mm", "area_factor",
            "contact_length_factor", "rate_relaxation_s", "hysteresis_rate_eps_mps",
            "relaxation_length_mm", "rim_stiffness_n_mm2",
        )
        for name in positive:
            if not getattr(self, name) > 0.0:
                raise ValueError(f"tyre '{self.name}': {name} must be positive, got {getattr(self, name)}")
        for name in ("carcass_stiffness_n_mm2", "rate_stiffening", "loss_factor", "tread_loss_crr"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"tyre '{self.name}': {name} must not be negative, got {getattr(self, name)}")
        if self.rim_radius_mm >= self.outer_radius_mm:
            raise ValueError(f"tyre '{self.name}': rim radius {self.rim_radius_mm} mm is not inside "
                             f"the outer radius {self.outer_radius_mm} mm")
        if self.rim_strike_deflection_mm <= 0.0:
            raise ValueError(f"tyre '{self.name}': compressed casing {self.compressed_casing_mm} mm "
                             f"leaves no travel above the rim")
        if self.area_factor > 1.0 or self.contact_length_factor > 1.0:
            raise ValueError(f"tyre '{self.name}': area and contact-length factors are at most 1")
        if self.loss_factor >= 1.0:
            raise ValueError(f"tyre '{self.name}': loss factor must be below 1, got {self.loss_factor}")

    @property
    def tyre_height_mm(self) -> float:
        """Height of the tyre above the rim, unloaded."""
        return self.outer_radius_mm - self.rim_radius_mm

    @property
    def rim_strike_deflection_mm(self) -> float:
        """Element deflection at which the rim engages: `R - R_rim - t_c` (RIDE.md §3.1)."""
        return self.tyre_height_mm - self.compressed_casing_mm

    @property
    def pressure_pa(self) -> float:
        """Gauge pressure in pascals."""
        return self.pressure_bar * BAR_TO_PA

    def with_pressure(self, pressure_bar: float) -> "TyreSpecs":
        """Returns a copy at another inflation pressure."""
        return replace(self, pressure_bar=float(pressure_bar))


# Plan D5: stock Magic Mary front, Hans Dampf rear, tubeless, no insert, 1.5 / 1.7 bar. Radii
# match `BikeSpecs` (372 / 352 mm) and the builder's rim geoms (320 / 300 mm), asserted by test.
FRONT_TYRE = TyreSpecs(
    name="Schwalbe Magic Mary 29x2.4",
    outer_radius_mm=372.0,
    rim_radius_mm=320.0,
    section_height_mm=57.0,
    pressure_bar=1.5,
    crr_reference=0.011,
    area_factor=0.4435,
    carcass_stiffness_n_mm2=0.1194,
    contact_length_factor=0.8589,
    rate_stiffening=0.2484,
    loss_factor=0.0662,
)
REAR_TYRE = TyreSpecs(
    name="Schwalbe Hans Dampf 27.5x2.4",
    outer_radius_mm=352.0,
    rim_radius_mm=300.0,
    section_height_mm=55.0,
    pressure_bar=1.7,
    crr_reference=0.0103,
    area_factor=0.4576,
    carcass_stiffness_n_mm2=0.1225,
    contact_length_factor=0.8801,
    rate_stiffening=0.2498,
    loss_factor=0.0690,
)


@dataclass(frozen=True)
class TierSpec:
    """
    Resolution of one fidelity tier (RIDE.md §3.1, *Tiers*).

    Attributes:
        name: ``fast`` or ``detailed``.
        n_rays: Radial elements per wheel.
        half_angle_deg: Coverage either side of the downward vertical.
        timestep_s: Integration timestep the tier runs at.
        discretised_brush: True for per-element bristles, False for the lumped brush.
    """

    name: str
    n_rays: int
    half_angle_deg: float
    timestep_s: float
    discretised_brush: bool


TIERS: Dict[str, TierSpec] = {
    "fast": TierSpec("fast", n_rays=64, half_angle_deg=75.0, timestep_s=0.0005, discretised_brush=False),
    "detailed": TierSpec("detailed", n_rays=256, half_angle_deg=75.0, timestep_s=0.00025,
                         discretised_brush=True),
}
TYRE_TIERS: Tuple[str, ...] = tuple(TIERS)
DEFAULT_TYRE_TIER = "fast"


@dataclass(frozen=True)
class TyreConfig:
    """
    The tyre a run uses.

    Attributes:
        model: ``sphere`` (default, RIDE.md §3.0) or ``pneumatic`` (§3.1).
        tier: ``fast`` or ``detailed``; ignored by ``sphere``.
        front: Front tyre; ignored by ``sphere``.
        rear: Rear tyre; ignored by ``sphere``.
        surface: Surface name overriding the track's own; None keeps the track's.
    """

    model: str = DEFAULT_TYRE_MODEL
    tier: str = DEFAULT_TYRE_TIER
    front: TyreSpecs = field(default=FRONT_TYRE)
    rear: TyreSpecs = field(default=REAR_TYRE)
    surface: Optional[str] = None

    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: On an unknown model, tier or surface.
        """
        if self.model not in TYRE_MODELS:
            raise ValueError(f"unknown tyre model '{self.model}'; available: {', '.join(TYRE_MODELS)}")
        if self.tier not in TIERS:
            raise ValueError(f"unknown tyre tier '{self.tier}'; available: {', '.join(TYRE_TIERS)}")
        if self.surface is not None:
            from bike_sim.terrain.surface import SURFACES  # pure data, no cycle at import time

            if self.surface not in SURFACES:
                raise ValueError(f"unknown surface '{self.surface}'; available: {', '.join(SURFACES)}")

    @property
    def pneumatic(self) -> bool:
        """Whether the run uses the pneumatic tyre."""
        return self.model == "pneumatic"

    @property
    def tier_spec(self) -> TierSpec:
        """The tier's resolution."""
        return TIERS[self.tier]

    def with_pressures(self, front_bar: float, rear_bar: float) -> "TyreConfig":
        """Returns a copy with both tyres at new pressures."""
        return replace(self, front=self.front.with_pressure(front_bar), rear=self.rear.with_pressure(rear_bar))


# --------------------------------------------------------------------------------------
# Literature targets (docs/RIDE.md section 11.2)
# --------------------------------------------------------------------------------------

# Static radial stiffness vs pressure, derived from Dressel & Sadauckas 2020 [T1]:
# k(p) = 22 + 24 p[bar] N/mm, +/-15 %, over 1.0-2.0 bar, near a 418 N load.
STATIC_STIFFNESS_INTERCEPT_N_MM = 22.0
STATIC_STIFFNESS_SLOPE_N_MM_PER_BAR = 24.0
STIFFNESS_TOLERANCE = 0.15
STIFFNESS_PRESSURE_RANGE_BAR = (1.0, 2.0)
REFERENCE_LOAD_N = 418.0

# Footprint length at REFERENCE_LOAD_N [T1 Fig. 16], +/-15 %.
CONTACT_LENGTH_TARGETS_MM: Dict[float, float] = {1.38: 133.0, 1.72: 122.0}
CONTACT_LENGTH_TOLERANCE = 0.15

# Share of the load carried by the pressure term at nominal pressure [T1].
PRESSURE_SHARE_BAND = (0.70, 1.00)

# Drop sled [T2]: 435 N sled; dynamic / static stiffness and settled damping ratio.
DROP_SLED_MASS_KG = 435.0 / 9.81
DYNAMIC_STIFFNESS_RATIO_BAND = (1.16, 1.35)
DAMPING_RATIO_BAND = (0.02, 0.055)

# Rim strike as a fraction of section height [T1, est].
RIM_STRIKE_FRACTION_BAND = (0.80, 0.85)

# Drum rolling resistance [T4]: 1.5 bar, 490.5 N (50 kg), 20 km/h; Crr ~ p^-0.3 [T3].
CRR_REFERENCE_PRESSURE_BAR = 1.5
CRR_REFERENCE_LOAD_N = 490.5
CRR_REFERENCE_SPEED_KMH = 20.0
CRR_PRESSURE_EXPONENT = -0.3
CRR_TOLERANCE = 0.15


def static_stiffness_target_n_mm(pressure_bar: float) -> float:
    """Static radial stiffness the literature law gives at a pressure, in N/mm."""
    return STATIC_STIFFNESS_INTERCEPT_N_MM + STATIC_STIFFNESS_SLOPE_N_MM_PER_BAR * float(pressure_bar)


def crr_target(tyre: TyreSpecs, pressure_bar: Optional[float] = None) -> float:
    """
    Drum rolling-resistance coefficient expected for a tyre.

    Args:
        tyre: The tyre, carrying its reference coefficient.
        pressure_bar: Pressure to evaluate at; defaults to the tyre's own.

    Returns:
        ``crr_reference . (p / 1.5 bar) ** -0.3``.
    """
    p = tyre.pressure_bar if pressure_bar is None else float(pressure_bar)
    return tyre.crr_reference * (p / CRR_REFERENCE_PRESSURE_BAR) ** CRR_PRESSURE_EXPONENT


def clamp_pressure_bar(pressure_bar: float) -> float:
    """Clamps a live pressure adjustment into the viewer's allowed band."""
    return min(PRESSURE_MAX_BAR, max(PRESSURE_MIN_BAR, float(pressure_bar)))


__all__ = [
    "BAR_TO_PA",
    "TYRE_MODELS",
    "DEFAULT_TYRE_MODEL",
    "TYRE_TIERS",
    "DEFAULT_TYRE_TIER",
    "PRESSURE_MIN_BAR",
    "PRESSURE_MAX_BAR",
    "PRESSURE_STEP_BAR",
    "TyreSpecs",
    "FRONT_TYRE",
    "REAR_TYRE",
    "TierSpec",
    "TIERS",
    "TyreConfig",
    "static_stiffness_target_n_mm",
    "crr_target",
    "clamp_pressure_bar",
]
