"""Public API boundaries and reporting regressions discovered during release review."""
from dataclasses import replace
import json
import mujoco
import numpy as np
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.ride.recorder import read_csv


def test_anatomical_reference_com_matches_compiled_pose():
    rider=RiderSpecs(variant='articulated_planar')
    points=rider.compute_rider_centers_of_mass(BikeSpecs())
    model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,
        physics_config=SimulationPhysicsConfig(physics_mode='physical')))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert sum(mass for _,mass in points.values())==pytest.approx(rider.mass_kg)
    for name,(point,mass) in points.items():
        body=model.body(name).id
        np.testing.assert_allclose(point,data.xipos[body],atol=1e-10)
        assert model.body_mass[body]==pytest.approx(mass,abs=1e-10)


def test_builder_rejects_legacy_tire_in_physical_mode():
    with pytest.raises(ValueError,match='pneumatic'):
        generate_mujoco_xml(mode='ride',tyre_model='pneumatic',
            physics_config=SimulationPhysicsConfig(physics_mode='physical'))


def test_csv_absent_patch_is_not_zero_load(tmp_path):
    file=tmp_path/'patches.csv'
    file.write_text('time_s,patch_force_n\n0,10\n.001,\n')
    values=read_csv(file)
    assert np.isnan(values['patch_force_n'][1])


def test_validation_failure_is_recorded_and_not_skipped(tmp_path,monkeypatch):
    from bike_sim.validation import benchmarks
    def fail(dt):raise RuntimeError('broken stand')
    monkeypatch.setitem(benchmarks.REGISTRY,'broken',fail)
    report=benchmarks.run_suite(tmp_path,[.0005],cases=['broken'])
    assert not report['passed']
    row=json.loads((tmp_path/'report.json').read_text())['cases'][0]
    assert row['status']=='error' and not row['passed'] and row['mandatory']


@pytest.mark.parametrize('surface,cause',[('catch_plane','catch_plane_contact'),('terrain','rider_ground_contact')])
def test_actual_emergency_contacts_are_classified(surface,cause):
    from bike_sim.sim.ride.physical_crash import physical_contact_crash
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
    <geom name="{surface}" type="plane" size="1 1 .1"/>
    <body pos="0 0 .099"><joint type="slide" axis="0 0 1"/>
    <geom name="geom_rider_head" type="sphere" size=".1" mass="5"/>
    </body></worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert physical_contact_crash(model,data)==cause
    data.qpos[0]=1.;mujoco.mj_forward(model,data)
    assert physical_contact_crash(model,data) is None


@pytest.mark.parametrize('rider',['none','lumped','seated','articulated_planar'])
def test_fast_momentum_matches_independent_com_jacobians(rider):
    from bike_sim.sim.ride.physical_energy import mass_observations
    from bike_sim.sim.ride.energy import system_momentum
    model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,
        physics_config=SimulationPhysicsConfig('physical')))
    data=mujoco.MjData(model);rng=np.random.default_rng(123)
    for _ in range(8):
        data.qpos[:]=rng.uniform(-.02,.02,model.nq)
        data.qvel[:]=rng.normal(size=model.nv)
        expected_p,expected_l=system_momentum(model,data)
        solved=data.qfrc_constraint.copy()
        result=mass_observations(model,data)
        np.testing.assert_allclose(result['linear_momentum_kg_mps'],expected_p,atol=1e-10)
        np.testing.assert_allclose(result['angular_momentum_kg_m2_s'],expected_l,atol=1e-10)
        np.testing.assert_array_equal(data.qfrc_constraint,solved)


def test_linkage_reference_does_not_change_with_time_step_refinement():
    responses=[]
    for dt in (.0005,.00025,.000125):
        model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider='none',
            physics_config=SimulationPhysicsConfig('physical',timestep_s=dt)))
        np.testing.assert_array_equal(model.eq_solref[:,0],.001)
        data=mujoco.MjData(model)
        data.qpos[model.joint('root_z').qposadr[0]]=2.
        data.qpos[model.joint('main_pivot').qposadr[0]]=.001
        mujoco.mj_forward(model,data)
        responses.append(data.qacc.copy())
    for response in responses[1:]:
        np.testing.assert_allclose(response,responses[0],atol=1e-8,rtol=1e-10)
    with pytest.raises(ValueError,match='closure_time_constant'):
        SimulationPhysicsConfig('physical',timestep_s=.001)
