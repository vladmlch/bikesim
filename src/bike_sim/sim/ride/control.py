"""Immutable, one-interval commands for the physical plant."""
from dataclasses import dataclass
from bike_sim.physics.checks import scalar
from bike_sim.physics.rider_posture import RiderPosture


@dataclass(frozen=True)
class RideControl:
    """Torques are N*m at the crank/mid-drive shaft, NOT the rear wheel.

    None motor_torque_nm selects the configured pedelec assistance. A numeric
    value explicitly selects an external research setpoint, bypassing only
    human/cadence demand gating, never motor lag, slew, torque/power/speed or
    battery limits. motor_limit_nm is an immediate safety ceiling after the lag
    and before energy limiting. Its removal restarts the normal bounded ramp.
    Commands are not latched by RideSimulation.step; wrappers may hold them.
    """
    motor_torque_nm: float | None = None
    motor_limit_nm: float | None = None
    human_torque_nm: float | None = None
    posture: RiderPosture | None = None
    rider_enabled: bool = True
    # One crank-reposition maneuver per rising edge: the rider backpedals to
    # the top of the power stroke while the motor shaft keeps driving. Needs
    # the articulated rider and the drive.motor_clutch topology.
    crank_reposition: bool = False

    def __post_init__(self):
        for name in ('motor_torque_nm', 'motor_limit_nm', 'human_torque_nm'):
            value = getattr(self, name)
            if value is not None:
                scalar(value, name, minimum=0.)
        if self.posture is not None and not isinstance(self.posture, RiderPosture):
            raise ValueError('posture must be a RiderPosture')
        if not isinstance(self.rider_enabled, bool):
            raise ValueError('rider_enabled must be a bool')
        if not isinstance(self.crank_reposition, bool):
            raise ValueError('crank_reposition must be a bool')

    def validate_for(self, config, rider_variant):
        if config.physics_mode != 'physical':
            raise ValueError('RideControl requires physical mode')
        if any(v is not None for v in (self.motor_torque_nm, self.motor_limit_nm, self.human_torque_nm)):
            if config.drive_mode not in ('crank_effort', 'articulated_effort'):
                raise ValueError('torque commands require a physical effort drive mode')
        if (self.posture is not None or not self.rider_enabled) and rider_variant != 'articulated_planar':
            raise ValueError('rider commands require articulated_planar')
        if self.crank_reposition:
            if rider_variant != 'articulated_planar' or config.drive_mode != 'articulated_effort':
                raise ValueError('crank reposition requires articulated_planar + articulated_effort')
            if not config.drive.motor_clutch:
                raise ValueError('crank reposition requires drive.motor_clutch')
