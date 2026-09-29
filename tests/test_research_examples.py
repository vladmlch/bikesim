from pathlib import Path
from bike_sim.cli.research import parser
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.tire_curve import TabulatedTireSpec
from bike_sim.sim.research.configuration import resolve_research_physics
from bike_sim.sim.research.rider_program import RiderProgram
from bike_sim.sim.ride.control import RideControl

ROOT = Path(__file__).resolve().parents[1]


def test_nonlinear_example_is_loadable_and_explicitly_synthetic():
    args = parser().parse_args(['--physics-config', str(ROOT/'examples/research/nonlinear_tires.toml')])
    config = resolve_research_physics(SimulationPhysicsConfig('physical'), args)
    for tire in (config.tires.front, config.tires.rear):
        assert isinstance(tire.material, TabulatedTireSpec)
        assert 'synthetic' in tire.material.provenance
        assert tire.material.is_load_in_valid_range(1000.)


def test_rider_example_keeps_motor_authority_with_the_policy():
    program = RiderProgram.load(ROOT/'examples/research/rider_shift.toml')
    intent = program.apply(RideControl(motor_torque_nm=42.), 2.)
    assert intent.motor_torque_nm == 42.
    assert intent.human_torque_nm == 0.
    assert intent.posture.pelvis_offset_m[1] == .1
    assert intent.posture.use_saddle is False
