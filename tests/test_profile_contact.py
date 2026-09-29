import numpy as np
import pytest
from bike_sim.terrain.contact_profile import ProfileQuery, closest_profile_contact


def test_flat_contact_does_not_depend_on_segment_count():
    c = [0.1, 0.34]
    a = closest_profile_contact(c, 0.35, [[-1, 0], [1, 0]])
    for count in (11, 101, 1001):
        vertices = np.column_stack([np.linspace(-1, 1, count), np.zeros(count)])
        b = closest_profile_contact(c, 0.35, vertices)
        assert b.delta == pytest.approx(0.01, abs=1e-12)
        np.testing.assert_allclose(b.normal, [0, 1], atol=1e-12)
        np.testing.assert_allclose(b.point, a.point, atol=1e-12)
        assert not b.multi_support


def test_slope_contact_and_convex_edge_normal():
    n = np.array([-0.5, np.sqrt(3) / 2])
    c = np.array([0.0, 0.0]) + 0.34 * n
    contact = closest_profile_contact(c, 0.35, [[-1, -1/np.sqrt(3)], [1, 1/np.sqrt(3)]])
    np.testing.assert_allclose(contact.normal, n)
    assert contact.delta == pytest.approx(0.01)
    edge = closest_profile_contact([0.02, 0.34], 0.35, [[-1, -1], [0, 0], [1, -1]])
    np.testing.assert_allclose(edge.point, [0, 0])
    np.testing.assert_allclose(edge.normal, np.array([0.02, 0.34])/np.hypot(0.02, 0.34))


def test_valley_reports_two_separated_supports_and_preserves_tie_branch():
    vertices = np.array([[-1., 1.], [0., 0.], [1., 1.]])
    a = closest_profile_contact([0, 0.4], 0.35, vertices, previous_segment=1)
    assert a.multi_support
    assert a.segment_id == 1
    b = closest_profile_contact([0, 0.4], 0.35, vertices, previous_segment=0)
    assert b.segment_id == 0


def test_airborne_is_not_loaded_and_endpoint_duplicates_are_not_multisupport():
    c = closest_profile_contact([0, 1.0], 0.35, [[-1, 0], [0, 0], [1, 0]])
    assert c.delta < 0
    assert not c.multi_support
    for name in ('point', 'normal'):
        with pytest.raises(ValueError):
            getattr(c, name).setflags(write=True)


@pytest.mark.parametrize('vertices,center,radius', [
    ([[0, 0], [0, 1]], [0, 1], .35),
    ([[0, 0], [1, np.nan]], [.5, 1], .35),
    ([[0, 0], [1, 0]], [.5, 0], .35),
    ([[0, 0], [1, 0]], [.5, -.1], .35),
    ([[0, 0], [1, 0]], [2, 1], .35),
    ([[0, 0], [1, 0]], [.5, 1], -1),
])
def test_invalid_or_unsupported_geometry_fails(vertices, center, radius):
    with pytest.raises(ValueError):
        closest_profile_contact(center, radius, vertices)


def test_local_search_matches_independent_full_segment_reference():
    x = np.linspace(-3, 3, 2001)
    vertices = np.column_stack([x, .08*np.cos(3*x)])
    query = ProfileQuery(vertices)
    for cx in np.linspace(-2.5, 2.5, 31):
        for height in (.34, .5):
            c = np.array([cx, height])
            a, b = vertices[:-1], vertices[1:]
            d = b-a
            t = np.clip(np.sum((c-a)*d, axis=1)/np.sum(d*d, axis=1), 0, 1)
            p = a+t[:, None]*d
            i = np.argmin(np.linalg.norm(c-p, axis=1))
            result = query.contact(c, .35)
            assert result.delta == pytest.approx(.35-np.linalg.norm(c-p[i]), abs=1e-12)
            np.testing.assert_allclose(result.point, p[i], atol=1e-12)


def test_query_copies_input_and_rejects_invalid_branch():
    v = np.array([[-1.,0], [1.,0]])
    query = ProfileQuery(v)
    v[:,1] = 10
    assert query.contact([0,.34],.35).delta == pytest.approx(.01)
    with pytest.raises(ValueError):
        query.contact([0,.34],.35, previous_segment=9)


def test_airborne_search_matches_full_independent_projection():
    """Even when a remote hill is closer, an airborne gap must be exact."""
    from bike_sim.terrain.contact_profile import ProfileQuery
    rng=np.random.default_rng(931)
    x=np.linspace(-60.,60.,24001)
    for z in (np.zeros_like(x),.2*np.sin(x/2),np.exp(-((x-8.)/3.)**2)*5.):
        vertices=np.column_stack((x,z));query=ProfileQuery(vertices)
        a,b=vertices[:-1],vertices[1:];direction=b-a
        for cx,cz in zip(rng.uniform(-10,10,8),rng.uniform(8,40,8)):
            center=np.array([cx,cz])
            fraction=np.clip(np.sum((center-a)*direction,axis=1)/np.sum(direction**2,axis=1),0,1)
            points=a+fraction[:,None]*direction
            index=np.argmin(np.linalg.norm(center-points,axis=1))
            contact=query.contact(center,.35)
            np.testing.assert_allclose(contact.point,points[index],atol=1e-10)
            assert contact.delta==pytest.approx(.35-np.linalg.norm(center-points[index]),abs=1e-10)
