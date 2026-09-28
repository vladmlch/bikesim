import numpy as np
import pytest
from bike_sim.physics.external_resistance import rolling_moment, drag_force


@pytest.mark.parametrize('omega', [-100., -10., -.01, 0., .01, 10., 100.])
def test_rolling_is_passive_in_both_directions(omega):
    torque = rolling_moment(.015, 500., .35, omega, .2)
    assert torque*omega <= 0
    assert rolling_moment(.015, 0., .35, omega, .2) == 0
    assert rolling_moment(.015, 500., .35, -omega, .2) == pytest.approx(-torque)


def test_known_normal_load_and_planar_drag():
    assert rolling_moment(.015, 500., .35, 10., .2) == pytest.approx(-2.625)
    v = np.array([7.,0.,1.])
    f = drag_force(v, 1.2, .5)
    assert f@v < 0
    np.testing.assert_array_equal(drag_force([0,0,0],1.2,.5), np.zeros(3))
    with pytest.raises(ValueError, match='planar'):
        drag_force([1,1,0],1.2,.5)


@pytest.mark.parametrize('bad', [float('nan'),float('inf'),-1])
def test_invalid_coefficients_are_rejected(bad):
    with pytest.raises(ValueError):
        rolling_moment(bad,100,.35,3,.2)
    with pytest.raises(ValueError):
        drag_force([3,0,0],1.2,bad)
