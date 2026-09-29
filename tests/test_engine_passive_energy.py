"""Native spring storage must not be charged again as dissipative work."""
import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride import physical_energy


@pytest.mark.parametrize('velocity', [-2., 2.])
@pytest.mark.parametrize('joint_type', ['hinge', 'slide'])
def test_native_spring_plus_damping_counts_only_damping(velocity, joint_type):
    m = mujoco.MjModel.from_xml_string(f'''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint type="{joint_type}" axis="0 1 0"
      stiffness="100" damping="3" springref="0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      </body></worldbody></mujoco>''')
    d = mujoco.MjData(m)
    d.qpos[0] = .2
    d.qvel[0] = velocity
    mujoco.mj_forward(m, d)
    result = physical_energy.engine_passive_loss_power(m, d.qpos, d.qvel, d.qfrc_passive)
    assert result == pytest.approx(3*velocity**2, abs=1e-12)


def test_undamped_native_spring_does_not_dissipate():
    m = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint stiffness="10" springref="30"/>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      </body></worldbody></mujoco>''')
    d = mujoco.MjData(m)
    d.qpos[0] = 1.
    d.qvel[0] = 2.
    mujoco.mj_forward(m, d)
    assert physical_energy.engine_passive_loss_power(m, d.qpos, d.qvel, d.qfrc_passive) == pytest.approx(0., abs=1e-12)
