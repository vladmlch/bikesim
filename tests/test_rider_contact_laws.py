import numpy as np
import pytest
from bike_sim.sim.ride.rider_contacts import grip_step
from bike_sim.sim.ride.static_braking import StaticBrakeApplier
from types import SimpleNamespace


@pytest.mark.parametrize('xi',([0.,0.,0.],[.01,0.,-.02]))
@pytest.mark.parametrize('u',([1.,0.,2.],[-2.,0.,-3.],[0.,0.,0.]))
def test_grip_has_exact_discrete_work_identity(xi,u):
    k,c,dt=4000.,150.,.0005
    new,force,energy,loss=grip_step(xi,u,k,c,dt)
    old=.5*k*np.dot(xi,xi)
    assert np.dot(force,u)*dt+energy-old+loss==pytest.approx(0.,abs=1e-12)
    assert loss>=0.


def test_grip_rotational_objectivity():
    angle=.3
    R=np.array([[np.cos(angle),0.,np.sin(angle)],[0.,1.,0.],[-np.sin(angle),0.,np.cos(angle)]])
    xi=np.array([.01,0.,-.02]); u=np.array([1.,0.,2.])
    a=grip_step(xi,u,4000.,150.,.0005)
    b=grip_step(R@xi,R@u,4000.,150.,.0005)
    np.testing.assert_allclose(R@a[0],b[0]); np.testing.assert_allclose(R@a[1],b[1])
    assert a[2:]==pytest.approx(b[2:])


def test_brake_configuration_atomic_and_not_a_velocity_write():
    model=SimpleNamespace(nv=4,dof_frictionloss=np.zeros(4))
    data=SimpleNamespace(qvel=np.ones(4))
    brake=StaticBrakeApplier(1,3,200.)
    brake.apply(model,data,1.,.5)
    np.testing.assert_array_equal(model.dof_frictionloss,[0.,200.,0.,100.])
    np.testing.assert_array_equal(data.qvel,np.ones(4))
    with pytest.raises(ValueError): brake.apply(model,data,0.,float('nan'))
    np.testing.assert_array_equal(model.dof_frictionloss,[0.,200.,0.,100.])
    brake.apply(model,data,0.,0.)
    assert not model.dof_frictionloss.any()
