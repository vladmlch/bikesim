import numpy as np
import pytest
from bike_sim.terrain.profile_distance import SignedProfile, signed_profile_distance


def test_plane_incline_and_projection():
    v = np.array([[-2., -1.], [0., 0.], [2., 1.]])
    p = np.array([[.2, .7], [.2, -.5]])
    d,n,c,s = signed_profile_distance(v,p)
    np.testing.assert_allclose(d, [.6/np.sqrt(1.25), -.6/np.sqrt(1.25)])
    np.testing.assert_allclose(n, np.tile([-1/np.sqrt(5), 2/np.sqrt(5)], (2,1)))
    np.testing.assert_allclose(p-c, d[:,None]*n, atol=1e-14)


def test_valley_tie_and_vertex_are_explicit():
    road=SignedProfile([[-1.,1.],[0.,0.],[1.,1.]])
    result=road.query([[0., .1],[0.,0.]])
    assert result.segment[0] == 0
    assert result.nonsmooth.all()


@pytest.mark.parametrize('p', [[-2.,0.],[2.,0.],[float('nan'),0.]])
def test_outside_not_extrapolated(p):
    with pytest.raises(ValueError):
        SignedProfile([[-1.,0.],[1.,0.]]).query([p])


def test_step_uses_same_raster_and_stable_mesh_subdivisions():
    v=np.array([[-1.,0.],[0.,0.],[.005,.04],[1.,.04]])
    road=SignedProfile(v)
    p=np.array([[.002,.01],[.1,.02],[-.1,.03]])
    fine=np.vstack([a+(b-a)*t for a,b in zip(v[:-1],v[1:]) for t in (0.,.5)]+[v[-1]])
    coarse=road.query(p); refined=SignedProfile(fine).query(p)
    np.testing.assert_allclose(coarse.distance, refined.distance, atol=1e-14)
    np.testing.assert_allclose(coarse.normal, refined.normal, atol=1e-12)
    assert coarse.distance[0]<0 and coarse.distance[1]<0 and coarse.distance[2]>0


def test_bounded_search_matches_all_segments():
    rng=np.random.default_rng(412)
    x=np.linspace(-2,2,401);v=np.c_[x,.05*np.sin(7*x)]
    p=np.c_[rng.uniform(-1.9,1.9,100),rng.uniform(-.1,.2,100)]
    answer=SignedProfile(v).query(p)
    edge=np.diff(v,axis=0)
    for point,d in zip(p,answer.distance):
        f=np.clip(np.sum((point-v[:-1])*edge,axis=1)/np.sum(edge**2,axis=1),0,1)
        reference=np.min(np.linalg.norm(point-(v[:-1]+f[:,None]*edge),axis=1))
        assert abs(d)==pytest.approx(reference, abs=1e-12)
