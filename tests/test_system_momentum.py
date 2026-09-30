import numpy as np
import pytest
import mujoco
from bike_sim.validation.system_momentum import system_momentum


def body_fixture():
    m=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/><worldbody>
    <body><joint name="x" type="slide" axis="1 0 0"/><joint name="z" type="slide" axis="0 0 1"/>
    <joint name="pitch" type="hinge" axis="0 1 0"/><inertial mass="2" pos=".2 0 .1" diaginertia=".2 .3 .2"/>
    <body pos=".4 0 .2"><joint name="internal" type="hinge" axis="0 1 0"/><inertial mass="1" pos=".1 0 0" diaginertia=".02 .03 .02"/></body>
    </body></worldbody></mujoco>''')
    d=mujoco.MjData(m);d.qpos[:]=[.2,.3,.4,.5];d.qvel[:]=[1.,2.,3.,4.]
    mujoco.mj_forward(m,d);return m,d


def test_common_translation_and_independent_linear_sum():
    m,d=body_fixture();com,p,l=system_momentum(m,d)
    d.qpos[0]+=100.;d.qpos[1]+=100.;mujoco.mj_forward(m,d)
    c2,p2,l2=system_momentum(m,d)
    np.testing.assert_allclose(c2-com,[100,0,100],atol=1e-12)
    np.testing.assert_allclose(p2,p,atol=1e-11);np.testing.assert_allclose(l2,l,atol=1e-11)
    delta=np.zeros(m.nv);delta[0]=1.;delta[1]=-1.;d.qvel+=delta
    _,p3,l3=system_momentum(m,d)
    np.testing.assert_allclose(p3-p2,[3,0,-3],atol=1e-11)
    np.testing.assert_allclose(l3,l2,atol=1e-11)


def test_internal_impulse_preserves_total_momentum_to_integrator_accuracy():
    m,d=body_fixture();d.qvel.fill(0.);m.opt.timestep=1e-5;mujoco.mj_forward(m,d)
    c,p,l=system_momentum(m,d);d.qfrc_applied[3]=10.
    mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    _,p1,l1=system_momentum(m,d)
    np.testing.assert_allclose(p1,p,atol=1e-9);np.testing.assert_allclose(l1,l,atol=1e-9)
