"""
Bicycle Specifications & Data Models.

Defines the geometric and suspension hardware specifications (BikeSpecs).
"""

from dataclasses import dataclass


@dataclass
class FrameGeometrySpecs:
    """Dataclass holding frame geometry and wheel dimensions (all dimensions in mm/deg)."""

    reach: float = 480.0
    stack: float = 646.0
    head_angle_deg: float = 64.0
    effective_seat_angle_deg: float = 77.0
    bb_drop: float = 22.5
    wheelbase: float = 1280.55  # Gives exact BB->P1 distance of 447.50 mm
    headtube_length: float = 120.0
    front_wheel_radius: float = 372.0  # 29" outer radius ~ 744mm diameter
    rear_wheel_radius: float = 352.0  # 27.5" outer radius ~ 704mm diameter
    wheel_radius: float = 372.0  # Legacy compatibility alias (Front 29")


@dataclass
class SuspensionHardwareSpecs:
    """Dataclass holding suspension hardware dimensions, baseline rates, and damper travels."""

    fork_travel: float = 180.0
    fork_offset: float = 44.0
    rear_wheel_travel: float = 180.0
    shock_eye_to_eye: float = 205.0  # Trunnion mount 205x65 (= 230x65 standard-mount equivalent)
    shock_stroke: float = 65.0
    fork_stiffness: float = 6000.0  # N/m (6.0 N/mm) -> 30% sag (54mm) under 80kg rider
    fork_damping: float = 450.0  # Ns/m -> damping ratio ~0.50 (linear baseline)
    shock_stiffness: float = 114600.0  # N/m (114.6 N/mm ~654 lbs/in) -> 30% rear sag
    shock_damping: float = 1200.0  # Ns/m -> damping ratio ~0.65 (linear baseline)


@dataclass
class BikeSpecs:
    """Composite bike specifications combining frame geometry and suspension hardware."""

    # Frame Geometry
    reach: float = 480.0
    stack: float = 646.0
    head_angle_deg: float = 64.0
    effective_seat_angle_deg: float = 77.0
    bb_drop: float = 22.5
    wheelbase: float = 1280.55
    headtube_length: float = 120.0
    front_wheel_radius: float = 372.0
    rear_wheel_radius: float = 352.0
    wheel_radius: float = 372.0

    # Suspension Hardware
    fork_travel: float = 180.0
    fork_offset: float = 44.0
    rear_wheel_travel: float = 180.0
    shock_eye_to_eye: float = 205.0  # Trunnion mount 205x65 (= 230x65 standard-mount equivalent)
    shock_stroke: float = 65.0
    fork_stiffness: float = 6000.0
    fork_damping: float = 450.0
    shock_stiffness: float = 114600.0
    shock_damping: float = 1200.0

    # Fork Damper (RockShox Charger 3 RC2) & DebonAir+ Pneumatic Spring Settings
    fork_hsc: int = 2
    fork_lsc: int = 7
    fork_rebound: int = 9
    fork_initial_psi: float = 85.2
    fork_air_tokens: int = 2

    # Rear Shock Damper (RockShox Super Deluxe Ultimate RC2T + HBO) Settings
    shock_hsc: int = 2
    shock_lsc: int = 7
    shock_rebound: int = 7
    shock_hbo: int = 2
    shock_lockout: bool = False

    @property
    def geometry(self) -> FrameGeometrySpecs:
        return FrameGeometrySpecs(
            reach=self.reach,
            stack=self.stack,
            head_angle_deg=self.head_angle_deg,
            effective_seat_angle_deg=self.effective_seat_angle_deg,
            bb_drop=self.bb_drop,
            wheelbase=self.wheelbase,
            headtube_length=self.headtube_length,
            front_wheel_radius=self.front_wheel_radius,
            rear_wheel_radius=self.rear_wheel_radius,
            wheel_radius=self.wheel_radius,
        )

    @property
    def suspension(self) -> SuspensionHardwareSpecs:
        return SuspensionHardwareSpecs(
            fork_travel=self.fork_travel,
            fork_offset=self.fork_offset,
            rear_wheel_travel=self.rear_wheel_travel,
            shock_eye_to_eye=self.shock_eye_to_eye,
            shock_stroke=self.shock_stroke,
            fork_stiffness=self.fork_stiffness,
            fork_damping=self.fork_damping,
            shock_stiffness=self.shock_stiffness,
            shock_damping=self.shock_damping,
        )
