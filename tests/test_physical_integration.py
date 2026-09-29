import dataclasses
import mujoco
import numpy as np
import pytest
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.mass import BikeMassSpecs

@pytest.mark.parametrize('drive', ['coast','ideal_speed_control','crank_effort','articulated_effort'])
def test_physical_topology_compiles_and_preserves_mass(drive):
    rider = 'articulated_planar' if drive == 'articulated_effort' else 'none'
    cfg = SimulationPhysicsConfig(physics_mode='physical', drive_mode=drive)
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride', rider=rider, physics_config=cfg))
    for name in ('crank_spin','cassette_spin','pedal_front_spin','pedal_rear_spin'):
        assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name) >= 0
    assert model.body_mass.sum() == pytest.approx(BikeMassSpecs().total_bike_mass + (80 if rider != 'none' else 0),abs=1e-8)
    assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY,'chain') < 0
    human = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'human_crank')
    assert (human >= 0) == (drive == 'crank_effort')
    if rider != 'none':
        pelvis = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_pelvis')
        assert model.body_parentid[pelvis] == 0
        assert model.body_jntnum[pelvis] == 3


def test_articulated_requires_physical_mode():
    with pytest.raises(ValueError, match='physical'):
        generate_mujoco_xml(mode='ride',rider='articulated_planar')
