from dataclasses import replace
import copy
import numpy as np
import pytest
import mujoco
from bike_sim.validation.contact_manifold_rigs import wheel_stand,distributed_flat_rig
from bike_sim.physics.resolution import resolve_physics_config


@pytest.mark.parametrize('count',[128,256,512])
def test_static_multicontact_supports_one_external_load(count):
    metrics,bounds=distributed_flat_rig(.000625,count,600.)
    assert metrics['normal_force_n']==pytest.approx(600.,rel=.02)
    assert metrics['native_wheel_contact_count']==0
    assert metrics['energy_residual_ratio']<.02


def test_probe_is_read_only_and_work_uses_same_points():
    m,d,tire=wheel_stand();d.qpos[1]=-.012;d.qvel[:]=[.2,-.1,.4];mujoco.mj_forward(m,d)
    q=d.qpos.copy();v=d.qvel.copy()
    probe=tire.compute_qfrc(m,d,.000625,advance=False)
    assert tire.last_time_s is None and tire.states=={} and tire.snapshots=={}
    force=tire.compute_qfrc(m,d,.000625)
    np.testing.assert_allclose(probe,force)
    np.testing.assert_array_equal(q,d.qpos);np.testing.assert_array_equal(v,d.qvel)
    power=0.
    for side,s in tire.snapshots.items():
        for patch in s.patches:
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,patch.point_m,tire.bodies[side])
            power+=float(patch.world_force_n@(jp@d.qvel))
    assert float(force@d.qvel)==pytest.approx(power,abs=1e-10)
    assert tire.snapshots['front'].wheel_axis_moment_nm==pytest.approx(force[2],abs=1e-10)
    with pytest.raises(ValueError):tire.compute_qfrc(m,d,.000625)


def test_liftoff_releases_existing_shear_energy_without_cloning_patches():
    m,d,tire=wheel_stand();d.qpos[1]=-.015;d.qvel[0]=.1;mujoco.mj_forward(m,d)
    tire.compute_qfrc(m,d,.000625)
    before=sum(.5*tire.config.front.tangent_k_n_m*tire.weights[key[1]]*state.xi**2 for key,state in tire.states.items())
    assert before>0 and all(isinstance(k,tuple) for k in tire.states)
    d.qpos[1]=.2;d.time+=.000625;mujoco.mj_forward(m,d)
    assert tire.stored_energy(m,d)==pytest.approx(before)
    tire.compute_qfrc(m,d,.000625)
    assert tire.brush_loss_step_j==pytest.approx(before)
    assert tire.states=={} and tire.stored_energy(m,d)==0


def test_configuration_constructs_density_and_cannot_self_certify():
    cfg=resolve_physics_config({'tires':{'backend':'distributed_2d_reference','distributed':{
        'station_count':256,'density':{'knots_m':[0.,.004],'stiffness_n_m':[1e5,2e5],'provenance':'synthetic'}}}})
    assert cfg.tires.distributed.density.knots_m==(0.,.004)
    with pytest.raises(ValueError):
        replace(cfg.tires.distributed,calibration_status='validated')
