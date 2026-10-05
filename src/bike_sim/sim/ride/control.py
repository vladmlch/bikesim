"""Immutable, one-interval commands for the physical plant."""
from dataclasses import dataclass
from bike_sim.physics.checks import scalar
from bike_sim.physics.rider_posture import RiderPosture


@dataclass(frozen=True)
class RideControl:
    """Torques are N*m at the crank/mid-drive shaft, NOT the rear wheel.

    None motor_torque_nm selects the configured pedelec assistance. A numeric
    value is an external ceiling on that assistance; rider torque and forward
    crank motion still gate it. Motor lag, slew, torque/power/speed and battery
    limits remain active. motor_limit_nm is an immediate safety ceiling after the lag
    and before energy limiting. Its removal restarts the normal bounded ramp.
    Commands are not latched by RideSimulation.step; wrappers may hold them.
    """
    motor_torque_nm: float | None = None
    motor_limit_nm: float | None = None
    human_torque_nm: float | None = None
    crank_target_rate_rad_s: float | None = None
    posture: RiderPosture | None = None
    rider_enabled: bool = True

    def __post_init__(self):
        for name in ('motor_torque_nm', 'motor_limit_nm', 'human_torque_nm', 'crank_target_rate_rad_s'):
            value = getattr(self, name)
            if value is not None:
                scalar(value, name, minimum=0.)
        if self.posture is not None and not isinstance(self.posture, RiderPosture):
            raise ValueError('posture must be a RiderPosture')
        if not isinstance(self.rider_enabled, bool):
            raise ValueError('rider_enabled must be a bool')

    def validate_for(self, config, rider_variant):
        if config.physics_mode != 'physical':
            raise ValueError('RideControl requires physical mode')
        if any(v is not None for v in (self.motor_torque_nm, self.motor_limit_nm, self.human_torque_nm)):
            if config.drive_mode not in ('crank_effort', 'articulated_effort'):
                raise ValueError('torque commands require a physical effort drive mode')
        if (self.posture is not None or not self.rider_enabled
                or self.crank_target_rate_rad_s is not None) and rider_variant != 'articulated_planar':
            raise ValueError('rider commands require articulated_planar')
