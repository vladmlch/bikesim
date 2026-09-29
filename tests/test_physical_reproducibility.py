"""End-to-end API/CLI, reset, and declared initial-condition contracts."""
from dataclasses import replace
import json
import numpy as np
import pytest

from bike_sim.cli.ride import main
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import PhysicalDriveConfig, TireBackendConfig
from bike_sim.sim.ride.physical_mapping import point_velocity
from bike_sim.sim.ride.physical_session import configuration_metadata, physical_summary
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset, TrackSpec, HeightFieldSpec
from bike_sim.validation.ride_cases import Incline


def test_physical_api_and_cli_have_identical_resolved_run(tmp_path):
    config=tmp_path/'physical.toml'
    config.write_text('physics_mode="physical"\ndrive_mode="crank_effort"\ninitial_speed_mps=1.0\n[tires]\nbackend="compliant_2d"\n[drive]\nhuman_torque_nm=20.0\n')
    assert main(['--physics-config',str(config),'--track','flat','--rider','none',
        '--duration','.01','--headless','--no-plots','--out',str(tmp_path/'cli')])==0
    saved=json.loads(next((tmp_path/'cli').glob('*/summary.json')).read_text())
    cfg=SimulationPhysicsConfig('physical',drive_mode='crank_effort',initial_speed_mps=1.,
        tires=TireBackendConfig(backend='compliant_2d'),drive=PhysicalDriveConfig(human_torque_nm=20.))
    sim=RideSimulation(track=get_preset('flat'),rider='none',physics_config=cfg)
    metadata=configuration_metadata(sim)
    for _ in range(20):sim.step()
    direct=physical_summary(sim,metadata,'duration_reached')
    assert saved['configuration_sha256']==direct['configuration_sha256']
    assert saved['resolved_config']==direct['resolved_config']
    for key in ('outcome','equilibrium','energy','component_work_j','airtime_threshold_s'):
        assert saved[key]==direct[key], key


def test_physical_reset_reproduces_states_events_and_work_exactly():
    cfg=SimulationPhysicsConfig('physical',drive_mode='crank_effort',initial_speed_mps=1.,
        tires=TireBackendConfig(backend='compliant_2d'),drive=PhysicalDriveConfig(human_torque_nm=20.))
    sim=RideSimulation(track=get_preset('flat'),rider='none',physics_config=cfg)
    def trace():
        rows=[]
        for _ in range(24):
            sim.step()
            rows.append(json.dumps(sim.physical.sample.as_dict(),sort_keys=True,allow_nan=False))
        return rows,sim.data.qpos.copy(),sim.data.qvel.copy()
    first,q,v=trace()
    sim.reset()
    second,q2,v2=trace()
    assert first==second
    np.testing.assert_array_equal(q,q2)
    np.testing.assert_array_equal(v,v2)


def test_inclined_initial_speed_does_not_start_with_a_normal_impact():
    track=TrackSpec('incline_initial',8.,[Incline(0.)])
    field=HeightFieldSpec(ncol=801,radius_x_m=4.,elevation_m=2.,datum_z_m=.1)
    cfg=SimulationPhysicsConfig('physical',initial_speed_mps=2.,initial_front_brake=1.,initial_rear_brake=1.,
        tires=TireBackendConfig(backend='compliant_2d'))
    sim=RideSimulation(track=track,field=field,rider='none',physics_config=cfg)
    tangent=sum((p.normal_load_n*p.tangent for snapshot in sim.physical.snapshots.values()
                 for p in snapshot.patches),np.zeros(3))
    translation=2.*tangent/np.linalg.norm(tangent)
    for side,snapshot in sim.physical.snapshots.items():
        body=sim.physical.tire.bodies[side]
        for patch in snapshot.patches:
            velocity=point_velocity(sim.model,sim.data,body,patch.point_m)
            # Float32 rasterization can give the two supports slightly different
            # slopes. Only that geometric projection, not a spin/pose impulse,
            # may contribute normal velocity at the declared initial condition.
            assert float(velocity@patch.normal)==pytest.approx(float(translation@patch.normal),abs=1e-9)
            assert float(velocity@patch.tangent)==pytest.approx(0.,abs=1e-9)
