"""Support-share preferences must not impose an unbalanced anatomical wrench."""
import numpy as np
import pytest
from bike_sim.sim.ride.rider_support import gravity_support_targets


def test_projected_supports_balance_weight_moment_and_coasting_crank():
    points=np.array([-.18,.165,-.165,.52])
    forces,status=gravity_support_targets(800.,.05,points,0.,.33,.12,[True]*4)
    vector=np.array(list(forces.values()))
    assert np.sum(vector)==pytest.approx(800.,abs=1e-9)
    assert points@vector==pytest.approx(800.*.05,abs=1e-9)
    assert .165*(forces['front']-forces['rear'])==pytest.approx(0.,abs=1e-9)
    assert np.min(vector[:3])>=0
    assert status['feasible']
    # Unlike mass routing, preferences can alter requests without changing any
    # physical segment mass or bypassing unilateral contact laws.
    different,_=gravity_support_targets(800.,.05,points,0.,.2,.2,[True]*4)
    assert different!=forces


def test_support_solution_is_translation_invariant():
    args=(800.,.05,np.array([-.18,.165,-.165,.52]),0.,.33,.12,[True]*4)
    reference,_=gravity_support_targets(*args)
    shifted,_=gravity_support_targets(args[0],args[1]+100,args[2]+100,100.,*args[4:])
    np.testing.assert_allclose(list(reference.values()),list(shifted.values()),rtol=1e-12,atol=1e-9)


def test_lost_supports_never_produce_a_hidden_request():
    forces,status=gravity_support_targets(800.,.05,[-.18,.165,-.165,.52],0.,.33,.12,[False]*4)
    assert all(value==0. for value in forces.values())
    assert not status['feasible']
    assert status['vertical_force_error_n']==-800.
    forces,status=gravity_support_targets(800.,1.,[-.18,.165,-.165,.52],0.,.33,.12,[True,False,False,False])
    assert forces['saddle']>=0.
    assert forces['front']==forces['rear']==forces['grip']==0.
    assert not status['feasible']


def test_postural_pitch_request_is_realized_by_contact_force_moments():
    points=np.array([-.2,.165,-.165,.5])
    forces,info=gravity_support_targets(800.,.1,points,0.,.33,.12,[True]*4,pitch_moment_nm=-20.)
    loads=np.array(list(forces.values()))
    assert np.sum(loads)==pytest.approx(800.)
    assert -np.dot(points-.1,loads)==pytest.approx(-20.)
    assert info['feasible']


def test_pedaling_support_requests_do_not_add_weight_twice():
    from bike_sim.sim.ride.rider_support import pedaling_support_targets
    points=np.array([[-.2,0.,.7],[.165,0.,0.],[-.165,0.,0.],[.5,0.,.8]])
    com=np.array([.1,0.,.8])
    requested={'front':np.array([-50.,0.,-150.]),'rear':np.zeros(3)}
    forces,info=pedaling_support_targets(800.,com,points,0.,.33,.12,[True]*4,requested,pitch_moment_nm=-20.)
    reactions=-np.array(list(forces.values()))
    np.testing.assert_allclose(np.sum(reactions,axis=0),[0.,0.,800.],atol=1e-9)
    moment=np.cross(points-com,reactions).sum(axis=0)
    assert moment[1]==pytest.approx(-20.,abs=1e-9)
    assert np.cross(points[1],forces['front'])[1]+np.cross(points[2],forces['rear'])[1]==pytest.approx(24.75,abs=1e-9)
    assert info['feasible']
