import numpy as np
import pytest
from bike_sim.physics.rider_segments import segment_masses, segment_inertia
from bike_sim.sim.ride.rider_control import stance_force, bounded_joint_torque, two_link_ik


@pytest.mark.parametrize('mass',[60.,80.,100.])
@pytest.mark.parametrize('helmet',[0.,.4])
def test_anatomical_mass_has_one_helmet_and_symmetric_legs(mass,helmet):
    result = segment_masses(mass,helmet)
    assert sum(result.values()) == pytest.approx(mass,abs=1e-10)
    assert result['head'] == pytest.approx(.0694*(mass-helmet)+helmet)
    assert result['thigh_front'] == result['thigh_rear']
    assert all(v > 0 for v in result.values())


def test_segment_tensor_is_physical_and_rotates():
    a = segment_inertia(5.,[.4,0,0],.04)
    b = segment_inertia(5.,[0,0,.4],.04)
    eig = np.linalg.eigvalsh(a)
    assert eig[0] > 0
    assert eig[-1] <= sum(eig[:-1])
    np.testing.assert_allclose(np.diag(a),np.diag(b)[::-1])


def test_stance_does_not_pull_return_pedal_and_mean_torque_is_correct():
    assert stance_force(0,20,.165)[2] < 0
    np.testing.assert_allclose(stance_force(np.pi,20,.165),[0,0,0],atol=1e-12)
    moments = []
    for phase in np.linspace(0,2*np.pi,2000,endpoint=False):
        total = 0.
        for p in (phase,phase+np.pi):
            r = .165*np.array([np.cos(p),0,-np.sin(p)])
            total += np.cross(r,stance_force(p,20,.165))[1]
        moments.append(total)
    assert np.mean(moments) == pytest.approx(20,rel=1e-5)


def test_joint_sum_is_bounded_and_bad_shapes_fail():
    np.testing.assert_allclose(bounded_joint_torque([0],[0],[2],100,10,30),[30])
    with pytest.raises(ValueError):
        bounded_joint_torque([0],[0,1],[1],100,10,30)


@pytest.mark.parametrize('target',[[.2,-.6],[.5,-.2],[-.2,-.5]])
def test_two_link_ik_matches_forward_kinematics(target):
    angles,saturated = two_link_ik(target,.42,.43,elbow_sign=1)
    assert not saturated
    q1,q2 = angles
    forward = .42*np.array([np.cos(q1),np.sin(q1)])+.43*np.array([np.cos(q1+q2),np.sin(q1+q2)])
    np.testing.assert_allclose(forward,target,atol=1e-10)


def test_unreachable_ik_is_reported_and_remains_finite():
    angles,saturated = two_link_ik([2.,0],.42,.43)
    assert saturated
    assert np.isfinite(angles).all()
    angles,saturated = two_link_ik([0.,0],.42,.43)
    assert saturated
    assert np.isfinite(angles).all()


def test_final_effort_has_power_and_speed_bounds_without_removing_braking():
    from bike_sim.sim.ride.rider_control import bounded_effort
    qd=np.array([30.,-30.,30.,-30.,5.,-5.,0.])
    requested=np.array([100.,-100.,-100.,100.,100.,-100.,100.])
    result=bounded_effort(qd,requested,100.,20.,250.)
    np.testing.assert_allclose(result,[0.,0.,-100.,100.,50.,-50.,100.])
    assert np.max(result*qd)<=250.
