"""
Pedal Drivetrain Physics.

Pure calculators for a pedalled mid-drive eMTB: the crank torque a rider produces over a
revolution, the ceilings that bound it, and the assist a Bosch-style mid-drive adds on top.
Nothing here touches MuJoCo -- `sim/ride/drivetrain.py` reads the model state, asks these
functions for a torque, and writes it into `ctrl`, the same division of labour the rest of
`sim/ride/` uses.

**Why the torque pulses.** Two legs 180 degrees apart on flat pedals that can be pushed but
not pulled: the tangential force is a downstroke only, so the crank torque over one
revolution follows |cos(phi)|, peaking with an arm horizontal and vanishing with the cranks
vertical. `phi = 0` is the right arm forward -- the 3 o'clock position the model is built in,
so a run that never pedals stands exactly where the pre-drivetrain model stood.

**Ripple depth is one knob, and it does not move the mean.**
``shape(phi) = (1 - d) + d * (pi/2) * |cos(phi)|`` has mean 1 for every depth, because
|cos| averages 2/pi. The depth therefore reshapes a revolution without changing its average:
at d = 0.85 the dead centre keeps 15 % of the mean and the peak reaches 1.48x, and d = 0 is a
perfectly smooth crank -- the setting that reproduces the pre-pedalling motor baseline.

**Assist law.** A mid-drive measures the rider's crank torque and adds `support_factor` times
it, so the motor repeats the ripple instead of smoothing it: in Turbo the pulse arriving at
the wheel is 4.4x the rider's own. Support is bounded by a torque ceiling, a power ceiling,
and the legal cutoff speed it tapers into.
"""

from dataclasses import dataclass
from math import cos, pi
from typing import Dict, Tuple

# Support factors as a fraction of the rider's own crank torque, in the shape of a
# Performance Line CX: Eco 60 %, Tour 140 %, Sport 240 %, Turbo 340 %.
ASSIST_MODES: Dict[str, float] = {
    "off": 0.0,
    "eco": 0.60,
    "tour": 1.40,
    "sport": 2.40,
    "turbo": 3.40,
}

# Cycling order for the interactive key, off first so a tap always reaches a known state.
ASSIST_ORDER: Tuple[str, ...] = ("off", "eco", "tour", "sport", "turbo")

# `motor` is the pre-pedalling ideal torque source kept for the baseline, `pedal` is the
# human alone, `pedelec` is the human plus the mid-drive.
DRIVE_MODES: Tuple[str, ...] = ("motor", "pedal", "pedelec")

# Crank-side actuator ceiling. The arithmetic that matters is the *peak*, not the mean: a
# 60 N.m mean rider torque peaks at 1.48x -- 88.9 N.m -- because the ripple has unit mean,
# and the motor tracks that pulse up to its own 85 N.m ceiling, so the crank sees 174 N.m at
# the top of a Turbo stroke. 180 N.m covers it with headroom for the torque that holds the
# cranks level while freewheeling.
CRANK_TORQUE_CEILING_NM = 180.0

RPM_PER_RADPS = 60.0 / (2.0 * pi)
KMH_PER_MPS = 3.6


@dataclass(frozen=True)
class DrivetrainSpecs:
    """
    Gearing, rider limits and assist limits of the pedalled drivetrain.

    Attributes:
        chainring_teeth / cog_teeth: The single gear. 32x14 is 2.286, which puts 25 km/h at
            82 rpm on the 352 mm rear wheel -- the middle of a normal cadence.
        ripple_depth: Depth of the |cos| pulse, in [0, 1]. 0 is a smooth crank.
        rider_torque_ceiling_nm: Sustained mean crank torque a rider holds. 60 N.m is a
            strong rider standing on it; the pulse peaks 1.48x higher.
        rider_power_ceiling_w: Sustained rider power. Whichever ceiling binds first wins,
            which is the torque ceiling below ~48 rpm and the power ceiling above it.
        assist_torque_ceiling_nm / assist_power_ceiling_w: Mid-drive limits, in the shape of
            a full-power eMTB: 85 N.m and 600 W peak (250 W nominal).
        assist_response_s: First-order lag of the torque sensor and controller.
        cutoff_speed_kmh / cutoff_taper_kmh: Legal cutoff and the band it fades over.
        crank_phase_deg: Crank angle at the start of a run; 0 is the built 3/9 o'clock pose.
        crank_hold_stiffness_nm_rad / crank_hold_damping_nms_rad: The rider's legs holding
            the cranks still while freewheeling. Without them a disengaged crank is the
            unactuated pendulum the original rigid crankset existed to avoid.
    """

    chainring_teeth: int = 32
    cog_teeth: int = 14
    ripple_depth: float = 0.85
    rider_torque_ceiling_nm: float = 60.0
    rider_power_ceiling_w: float = 300.0
    assist_torque_ceiling_nm: float = 85.0
    assist_power_ceiling_w: float = 600.0
    assist_response_s: float = 0.30
    cutoff_speed_kmh: float = 25.0
    cutoff_taper_kmh: float = 2.0
    crank_phase_deg: float = 0.0
    crank_hold_stiffness_nm_rad: float = 30.0
    crank_hold_damping_nms_rad: float = 3.0

    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If the gearing, a ceiling or the ripple depth is out of range.
        """
        if self.chainring_teeth < 20 or self.cog_teeth < 9:
            raise ValueError(
                f"implausible gearing {self.chainring_teeth}x{self.cog_teeth}; "
                f"a chainring has at least 20 teeth and a cog at least 9"
            )
        if not 0.0 <= self.ripple_depth <= 1.0:
            raise ValueError(f"ripple_depth must be in [0, 1], got {self.ripple_depth}")
        for name in (
            "rider_torque_ceiling_nm",
            "rider_power_ceiling_w",
            "assist_torque_ceiling_nm",
            "assist_power_ceiling_w",
            "assist_response_s",
            "cutoff_speed_kmh",
            "cutoff_taper_kmh",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        rider_peak = self.rider_torque_ceiling_nm * peak_shape(self.ripple_depth)
        if rider_peak + self.assist_torque_ceiling_nm > CRANK_TORQUE_CEILING_NM:
            raise ValueError(
                f"peak crank torque {rider_peak + self.assist_torque_ceiling_nm:.1f} N.m exceeds the "
                f"{CRANK_TORQUE_CEILING_NM:.0f} N.m crank actuator ceiling; lower a ceiling or the depth"
            )

    @property
    def gear_ratio(self) -> float:
        """Wheel revolutions per crank revolution."""
        return float(self.chainring_teeth) / float(self.cog_teeth)

    def cadence_rpm_at(self, speed_mps: float, rear_wheel_radius_m: float) -> float:
        """
        Cadence the gearing gives at a road speed.

        Args:
            speed_mps: Road speed.
            rear_wheel_radius_m: Rolling radius of the driven wheel.

        Returns:
            Cadence in rpm.
        """
        wheel_radps = speed_mps / rear_wheel_radius_m
        return wheel_radps / self.gear_ratio * RPM_PER_RADPS

    def support_factor(self, assist_mode: str) -> float:
        """
        Support factor of a named assist mode.

        Args:
            assist_mode: One of `ASSIST_MODES`.

        Returns:
            Motor torque as a multiple of rider torque.

        Raises:
            ValueError: If the mode is unknown.
        """
        try:
            return ASSIST_MODES[assist_mode]
        except KeyError:
            raise ValueError(
                f"unknown assist mode {assist_mode!r}; available: {', '.join(ASSIST_ORDER)}"
            ) from None


def peak_shape(depth: float) -> float:
    """
    Peak of the unit-mean ripple shape.

    Args:
        depth: Ripple depth in [0, 1].

    Returns:
        Ratio of peak to mean crank torque.
    """
    return (1.0 - depth) + depth * (pi / 2.0)


def ripple_shape(phase_rad: float, depth: float) -> float:
    """
    Unit-mean crank torque shape over a revolution.

    Args:
        phase_rad: Crank angle; 0 is the right arm forward, horizontal.
        depth: Ripple depth in [0, 1]; 0 is a smooth crank.

    Returns:
        Multiplier on the mean crank torque, mean 1 over a revolution for any depth.
    """
    return (1.0 - depth) + depth * (pi / 2.0) * abs(cos(phase_rad))


def limited_rider_torque(
    demand_nm: float,
    crank_radps: float,
    specs: DrivetrainSpecs,
) -> Tuple[float, str]:
    """
    Clamps a mean crank torque demand to what a rider can actually hold.

    Args:
        demand_nm: Mean crank torque the speed controller is asking for, N.m.
        crank_radps: Crank angular velocity, used for the power ceiling.
        specs: Drivetrain limits.

    Returns:
        Tuple of (mean crank torque, binding limit: ``none``, ``torque`` or ``power``).
    """
    if demand_nm <= 0.0:
        return 0.0, "none"
    limit = "none"
    torque = demand_nm
    if torque > specs.rider_torque_ceiling_nm:
        torque = specs.rider_torque_ceiling_nm
        limit = "torque"
    if crank_radps > 1e-3:
        power_limited = specs.rider_power_ceiling_w / crank_radps
        if power_limited < torque:
            torque = power_limited
            limit = "power"
    return torque, limit


def cutoff_factor(speed_mps: float, specs: DrivetrainSpecs) -> float:
    """
    Fraction of the nominal support still available at a road speed.

    Args:
        speed_mps: Road speed.
        specs: Drivetrain limits, read for the cutoff speed and its taper band.

    Returns:
        1 below the taper, 0 above the cutoff, linear in between.
    """
    speed_kmh = speed_mps * KMH_PER_MPS
    taper_start = specs.cutoff_speed_kmh - specs.cutoff_taper_kmh
    if speed_kmh <= taper_start:
        return 1.0
    if speed_kmh >= specs.cutoff_speed_kmh:
        return 0.0
    return (specs.cutoff_speed_kmh - speed_kmh) / specs.cutoff_taper_kmh


def assist_target_torque(
    rider_torque_nm: float,
    support_factor: float,
    crank_radps: float,
    specs: DrivetrainSpecs,
) -> float:
    """
    Motor torque the controller is aiming for, before its response lag.

    Args:
        rider_torque_nm: The rider's instantaneous crank torque, ripple included.
        support_factor: Support of the selected mode after the cutoff taper.
        crank_radps: Crank angular velocity, used for the power ceiling.
        specs: Drivetrain limits.

    Returns:
        Target motor torque at the crank, N.m.
    """
    if rider_torque_nm <= 0.0 or support_factor <= 0.0:
        return 0.0
    target = min(support_factor * rider_torque_nm, specs.assist_torque_ceiling_nm)
    if crank_radps > 1e-3:
        target = min(target, specs.assist_power_ceiling_w / crank_radps)
    return target


def first_order_step(current: float, target: float, dt_s: float, tau_s: float) -> float:
    """
    One explicit step of a first-order lag.

    Args:
        current: Present value.
        target: Value being approached.
        dt_s: Timestep.
        tau_s: Time constant.

    Returns:
        The value after `dt_s`.
    """
    if tau_s <= 0.0:
        return target
    alpha = dt_s / (tau_s + dt_s)
    return current + alpha * (target - current)


__all__ = [
    "ASSIST_MODES",
    "ASSIST_ORDER",
    "DRIVE_MODES",
    "CRANK_TORQUE_CEILING_NM",
    "DrivetrainSpecs",
    "assist_target_torque",
    "cutoff_factor",
    "first_order_step",
    "limited_rider_torque",
    "peak_shape",
    "ripple_shape",
]
