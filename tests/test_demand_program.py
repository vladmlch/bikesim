import json
import csv
import pytest
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_constant_and_keyframes():
    assert DemandProgram.constant(60.).at(5.) == 60.
    d = DemandProgram(keyframes=((0., 0.), (1., 80.)))
    assert d.at(0.) == 0. and abs(d.at(1.)-80.) < 1e-9 and 0. < d.at(.5) < 80.
    assert d.at(9.) == 80.


def test_quintic_smoothstep_matches_rider_program_blend():
    d = DemandProgram(keyframes=((0., 0.), (2., 100.)))
    assert d.at(1.) == pytest.approx(50.)           # blend(0.5) = 0.5
    assert d.at(.5) == pytest.approx(100.*(10*.25**3-15*.25**4+6*.25**5))


@pytest.mark.parametrize('frames', [(), ((1., 10.),), ((0., 10.), (0., 20.)), ((0., -1.),),
                                    ((0., float('nan')),), ((0., 1., 2.),)])
def test_invalid_keyframes_rejected(frames):
    with pytest.raises(ValueError):
        DemandProgram(keyframes=frames)


def test_dict_and_toml_round_trip(tmp_path):
    d = DemandProgram(keyframes=((0., 10.), (1.5, 70.)))
    assert DemandProgram.from_dict(d.to_dict()) == d
    (tmp_path/'d.toml').write_text('[[keyframes]]\ntime_s = 0.0\ntorque_nm = 10.0\n'
                                   '[[keyframes]]\ntime_s = 1.5\ntorque_nm = 70.0\n')
    assert DemandProgram.load(tmp_path/'d.toml') == d
    with pytest.raises(ValueError):
        DemandProgram.from_dict({'keyframes': [], 'teleport': 1})


@pytest.fixture(scope='module')
def plant():
    return RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))


def test_env_exposes_demand_as_command_echo_and_records_it(plant, tmp_path):
    plant.reset()
    demand = DemandProgram(keyframes=((0., 0.), (.02, 50.)))
    env = ResearchEnvironment(plant, ExperimentConfig(duration_s=.03, record_decimation=1),
                              SensorConfig.ideal(), demand=demand)
    assert env.demand_nm == 0.
    seen = []
    while not env.done:
        result = env.step(RideControl(motor_torque_nm=env.demand_nm, human_torque_nm=0.))
        seen.append(result.demand_nm)
    assert seen[0] == 0. and seen[1] == pytest.approx(demand.at(.01))
    env.save(tmp_path/'run')
    rows = list(csv.DictReader((tmp_path/'run'/'trace.csv').open()))
    assert float(rows[1]['demand_nm']) == pytest.approx(seen[1])
    summary = json.loads((tmp_path/'run'/'summary.json').read_text())
    assert summary['research']['demand_program'] == demand.to_dict()


def test_env_without_demand_reports_none(plant):
    plant.reset()
    env = ResearchEnvironment(plant, ExperimentConfig(duration_s=.01), SensorConfig.ideal())
    assert env.demand_nm is None
    assert env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.)).demand_nm is None


def test_env_rejects_wrong_demand_type(plant):
    plant.reset()
    with pytest.raises(ValueError, match='DemandProgram'):
        ResearchEnvironment(plant, ExperimentConfig(), SensorConfig.ideal(), demand=60.)
