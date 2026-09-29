from types import SimpleNamespace
import pytest
from bike_sim.sim.research.validity import model_violations
from bike_sim.sim.research.environment import ExperimentConfig, ResearchStep


def sample(**changes):
    tire = dict(multi_support=False, penetration_m=.005, unloaded_radius_m=.35,
                outside_material_load_range=False, normal_load_n=500., patches=[])
    return SimpleNamespace(channels={'tires': {'front': tire | changes, 'rear': dict(tire)},
                                    'suspension': {'linkage_closure_max_m': 1e-6}})


def test_separate_model_limits_without_clipping_forces():
    s = sample(multi_support=True, outside_material_load_range=True, penetration_m=.08)
    before = dict(s.channels['tires']['front'])
    assert model_violations(s, .15) == ('front:multi_support', 'front:material_load_range', 'front:tire_compression')
    assert s.channels['tires']['front'] == before
    assert model_violations(sample(), .15) == ()


def test_airborne_zero_force_is_not_a_material_range_violation():
    assert model_violations(sample(normal_load_n=0., penetration_m=-.02,
                                  outside_material_load_range=True), .15) == ()


def test_linkage_error_and_catch_plane_are_explicit():
    s = sample(patches=[{'source_geom': 'catch_plane', 'normal_load_n': 20.}])
    s.channels['suspension']['linkage_closure_max_m'] = .003
    assert model_violations(s, .15) == ('front:catch_plane', 'linkage:closure_error')


@pytest.mark.parametrize('values', [ {'enforce_model_limits': 'yes'},
    {'maximum_tire_compression_fraction': 1.}, {'maximum_tire_compression_fraction': 0.},
    {'maximum_linkage_error_m': -1.} ])
def test_bad_applicability_configuration(values):
    with pytest.raises(ValueError):
        ExperimentConfig(**values)


def test_model_gate_is_latched_separately_from_energy(monkeypatch, tmp_path):
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import TireBackendConfig
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.research.environment import ResearchEnvironment
    import bike_sim.sim.research.environment as module
    sim = RideSimulation(rider='lumped', physics_config=SimulationPhysicsConfig('physical',
        drive_mode='crank_effort', tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))
    env = ResearchEnvironment(sim, ExperimentConfig(duration_s=.01))
    monkeypatch.setattr(module, 'model_violations', lambda *a: ('front:multi_support',))
    result = env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
    assert result.reason == 'model_applicability' and result.truncated
    assert result.numerically_valid and not result.model_valid and not result.valid_for_learning
    assert result.physics_steps == 1
    assert env.model_violation_time_s['front:multi_support'] == pytest.approx(sim.model.opt.timestep)
    with pytest.raises(RuntimeError):
        env.step(RideControl())
