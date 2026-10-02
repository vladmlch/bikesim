"""Resolve bicycle suspension settings into active physical components."""

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.damper import BikeSuspensionSystem, DamperClickConfig
from bike_sim.sim.controllers import SuspensionController


def build_suspension_components(
    specs: BikeSpecs, *, preload_mm: float = 0.0
) -> tuple[SuspensionController, CoilShock]:
    """Build the fork, dampers and rear coil from one bicycle specification."""
    clicks = DamperClickConfig(
        fork_hsc=specs.fork_hsc,
        fork_lsc=specs.fork_lsc,
        fork_rebound=specs.fork_rebound,
        shock_hsc=specs.shock_hsc,
        shock_lsc=specs.shock_lsc,
        shock_rebound=specs.shock_rebound,
        shock_hbo=specs.shock_hbo,
        shock_lockout=specs.shock_lockout,
    )
    dampers = BikeSuspensionSystem(
        clicks,
        fork_travel_mm=specs.fork_travel,
        shock_stroke_mm=specs.shock_stroke,
    )
    air = ForkAirSpring(
        AirSpringSpecs(total_travel_mm=specs.fork_travel),
        num_tokens=specs.fork_air_tokens,
        gauge_pressure_psi=specs.fork_initial_psi,
    )
    coil = CoilShock(
        CoilShockSpecs(
            rate_n_m=specs.shock_stiffness,
            preload_mm=preload_mm,
            stroke_mm=specs.shock_stroke,
        )
    )
    return SuspensionController(specs, air, dampers), coil
