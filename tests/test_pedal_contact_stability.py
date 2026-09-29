"""A light pedal's finite-edge contact must remain stable at the declared dt."""
import mujoco
import numpy as np
import pytest
from bike_sim.physics.physical_config import ArticulatedConfig
from bike_sim.physics.tire import normal_contact
from bike_sim.sim.ride.support_geometry import box_pad_contact


@pytest.mark.parametrize('dt', [.0005, .00025, .000125])
def test_unforced_pedal_with_loaded_edge_pads_relaxes_without_numerical_flip(dt):
    cfg = ArticulatedConfig()
    # Test the shipped law, including compatibility with the pre-separation
    # material profile. Restoring the old 500 N s/m shared damping must fail.
    damping = getattr(cfg, 'pedal_c_ns_m', cfg.support_c_ns_m)
    m = mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt}"
      gravity="0 0 0" integrator="implicitfast"/>
      <worldbody><body><joint axis="0 1 0" damping="0.005"/>
      <inertial pos="0 0 0" mass="0.175" diaginertia="0.0000970667 0.0001495667 0.0002391667"/>
      </body></worldbody></mujoco>''')
    d = mujoco.MjData(m)
    d.qpos[0] = .003
    peak = abs(float(d.qpos[0]))
    jp,jr = np.zeros((3,1)),np.zeros((3,1))
    for _ in range(round(.2/dt)):
        mujoco.mj_forward(m,d)
        d.qfrc_applied.fill(0.)
        for x in (-.045, .045):
            pad = box_pad_contact([x,0.,.023],cfg.support_pad_radius_m,
                                  d.xpos[1],d.xmat[1].reshape(3,3),[.05,.04,.008])
            mujoco.mj_jac(m,d,jp,jr,pad.point_m,1)
            delta_dot = float((jp@d.qvel)@pad.normal)
            force,_ = normal_contact(-pad.gap_m,delta_dot,cfg.support_k_n_m/2,damping/2)
            mujoco.mj_applyFT(m,d,-force*pad.normal,np.zeros(3),pad.point_m,1,d.qfrc_applied)
        mujoco.mj_step(m,d)
        peak=max(peak,abs(float(d.qpos[0])))
    assert peak < .0031, 'unforced contact must not amplify a small pedal tilt'
    assert abs(d.qpos[0]) < 1e-7
