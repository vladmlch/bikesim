import numpy as np
import pytest
from bike_sim.physics.distributed_tire import HingeDensity, density_response, station_angles, normal_station_response, fit_density, flat_load
from bike_sim.terrain.profile_distance import SignedProfile


def test_force_is_potential_derivative():
    m=HingeDensity((0.,.004),(10000.,20000.),0.,'synthetic-test')
    d=np.array([.003,.007]);eps=1e-7
    f,u=density_response(d,m)
    np.testing.assert_allclose(f,(density_response(d+eps,m)[1]-density_response(d-eps,m)[1])/(2*eps),rtol=1e-6)
    assert np.all(u>=0)


@pytest.mark.parametrize('count',[128,256,512])
def test_quadrature_and_unilateral(count):
    a,w=station_angles(count)
    assert len(a)==count and w.sum()==pytest.approx(2*np.pi)
    f,u=normal_station_response(np.full(count,.002),-100.,w,HingeDensity())
    assert np.all(f==0) and np.all(u>0)
    assert normal_station_response(-.1,100.,1.,HingeDensity())[0]==0


@pytest.mark.parametrize('q', [[.013,.359,.02],[.02,.359,.17]])
def test_x_z_spin_potential_gradient(q):
    q=np.array(q);a,w=station_angles(128);R=.37
    road=SignedProfile([[-2.,0.],[0.,0.],[.025,.005],[2.,.005]])
    material=HingeDensity((0.,),(300000.,),0.,'synthetic')
    def evaluate(state):
        x,z,theta=state
        arm=np.c_[R*np.cos(a-theta),R*np.sin(a-theta)]
        p=arm+[x,z];d=road.query(p)
        f,u=normal_station_response(-d.distance,0.,w,material)
        world=f[:,None]*d.normal
        # Positive rotation about world Y rotates X toward negative Z.
        return np.array([world[:,0].sum(),world[:,1].sum(),np.sum(arm[:,1]*world[:,0]-arm[:,0]*world[:,1])]),u.sum()
    force,_=evaluate(q);eps=1e-7
    gradient=np.array([(evaluate(q+eps*np.eye(3)[i])[1]-evaluate(q-eps*np.eye(3)[i])[1])/(2*eps) for i in range(3)])
    np.testing.assert_allclose(force,-gradient,rtol=2e-5,atol=1e-5)


def test_fit_recovers_known_density_and_rejects_incompatible_radial_curve():
    m=HingeDensity((0.,.004),(230000.,300000.),0.,'synthetic target')
    d=np.linspace(.001,.018,20);loads=flat_load(d,.37,m)[0]
    fitted,report=fit_density(d,loads,.37,knots_m=(0.,.004),dataset_id='synthetic-density')
    assert report['accepted'] and report['max_relative_error']<1e-10
    _,bad=fit_density(np.array([100.,300.,600.,1000.])/130000.,[100.,300.,600.,1000.],.37,
                      dataset_id='synthetic-original-radial-law')
    assert not bad['accepted']
