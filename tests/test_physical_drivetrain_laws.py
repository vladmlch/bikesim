import math
import numpy as np
import pytest
from bike_sim.physics.chain import DrivetrainSpecs, chain_extension, chain_jacobian
from bike_sim.physics.freehub import Freehub
from bike_sim.physics.motor import AssistController
from bike_sim.physics.pedaling import human_crank_torque
from bike_sim.physics.battery import Battery, motor_electrical_power, limit_torque_by_energy


def test_chain_radii_and_equal_sprockets():
    gear = DrivetrainSpecs()
    assert gear.front_radius_m == pytest.approx(.0127*34/(2*math.pi))
    assert chain_extension([0,0],[-.45,0],.05,.05,.2,0,.45) == pytest.approx(.01)


@pytest.mark.parametrize('angle', [-3.5,-2.,-.2,0.,.2,2.,3.5])
def test_chain_global_rotation_is_not_stretch(angle):
    cf, cr = np.array([0.,0.]), np.array([-.45,0.])
    baseline = chain_extension(cf,cr,.07,.04,0,0,0,psi_reference=math.pi/2)
    R = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    actual = chain_extension(R@cf,R@cr,.07,.04,-angle,-angle,baseline,
                             up_xz=R@np.array([0.,1.]),psi_reference=math.pi/2+angle)
    assert actual == pytest.approx(0.,abs=1e-12)


def test_chain_jacobian_virtual_work_and_phase_ratio():
    q = np.array([.1,-.02,.03])
    def extension(q):
        return chain_extension([0,0],[-.45,q[2]],.07,.04,q[0],q[1],.45)
    J = chain_jacobian(q,extension)
    np.testing.assert_allclose(J[:2],[.07,-.04],atol=1e-9)
    v = np.array([2.,-1.,.3])
    h = 1e-6
    rate = (extension(q+h*v)-extension(q-h*v))/(2*h)
    assert (-100*J)@v == pytest.approx(-100*rate,abs=1e-7)


def test_freehub_overrun_engagement_reverse_and_reset():
    hub = Freehub(1000.,.5)
    assert hub.update(0,0,0,10) == 0
    assert hub.update(0,1,0,10) == 0
    assert hub.update(.02,1,2,0) == pytest.approx(21)
    assert hub.energy_j == pytest.approx(.2)
    assert hub.update(-.1,1,-2,0) == 0
    assert hub.energy_j == 0
    hub.reset()
    assert hub.energy_j == 0


def test_assist_brake_priority_and_power_cap_after_cadence_jump():
    motor = AssistController()
    for _ in range(200):
        torque = motor.step(100,20,2,False,.005)
    assert torque > 0
    torque = motor.step(100,500,2,False,.005)
    assert torque*500*2*math.pi/60 <= 500+1e-9
    assert motor.step(100,60,2,True,.005) == 0
    assert motor.step(100,-1,2,False,.005) == 0


def test_assist_stop_delay_cutoff_table_and_atomic_validation():
    motor = AssistController(torque_curve=((0.,80.),(100.,40.),(200.,0.)))
    for _ in range(200):
        torque = motor.step(40,100,2,False,.005)
    assert torque <= 40
    assert motor.step(40,60,25/3.6,False,.005) == 0
    for _ in range(200):
        motor.step(40,60,2,False,.005)
    old = motor.torque
    with pytest.raises(ValueError):
        motor.step(float('nan'),60,2,False,.005)
    assert motor.torque == old
    for _ in range(25):
        torque = motor.step(0,0,2,False,.005)
    assert torque == 0


def test_human_torque_mean_over_full_cycle():
    phase = np.linspace(0,2*math.pi,1000,endpoint=False)
    assert np.mean([human_crank_torque(20,p) for p in phase]) == pytest.approx(20)


def test_battery_and_stall_loss():
    battery = Battery(10)
    assert battery.draw(100,.2) == 50
    assert battery.energy_j == 0
    assert motor_electrical_power(20,0,.02,0,5,True) == 13
    assert motor_electrical_power(20,10,.02,0,5,False) == 0


@pytest.mark.parametrize('omega', [0.,.1,2.,100.,-2.])
@pytest.mark.parametrize('budget', [0.,1.,5.,6.,20.,100.,1000.])
def test_energy_limiter_never_exceeds_budget(omega,budget):
    torque = limit_torque_by_energy(80,omega,.02,.001,5,budget)
    assert 0 <= torque <= 80
    if torque > 0:
        assert motor_electrical_power(torque,omega,.02,.001,5,True) <= budget+1e-9


@pytest.mark.parametrize('bad', [float('nan'),float('inf'),-1])
def test_invalid_inputs_do_not_mutate_battery(bad):
    battery = Battery(10)
    with pytest.raises(ValueError):
        battery.draw(bad,.1)
    assert battery.energy_j == 10
    with pytest.raises(ValueError):
        Freehub(bad,.5)
    with pytest.raises(ValueError):
        DrivetrainSpecs(front_teeth=bad)
