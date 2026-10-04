"""Two-leg crank torque waveform: mean x (1 + ripple cos 2phi), ripple from config."""
import math

import numpy as np
import pytest

from bike_sim.sim.ride.rider_control import pedal_torque_waveform, pedaling_force_requests


def test_waveform_mean_and_ripple():
    phases = np.linspace(0., 2*math.pi, 3600, endpoint=False)
    values = np.array([pedal_torque_waveform(40., p, .5) for p in phases])
    assert values.mean() == pytest.approx(40., rel=1e-6)
    assert values.max() == pytest.approx(60.) and values.min() == pytest.approx(20.)
    assert pedal_torque_waveform(40., 0., 0.) == 40.


def test_waveform_rejects_invalid_ripple():
    for bad in (-.1, 1., 1.5, math.nan):
        with pytest.raises(ValueError):
            pedal_torque_waveform(40., 0., bad)


def test_force_requests_honour_the_ripple_argument():
    normals = {'front': (0., 0., 1.), 'rear': (0., 0., 1.)}
    loads = {'front': 1e4, 'rear': 1e4}
    lo, _ = pedaling_force_requests(0., 40., .175, normals, loads, 5., ripple=0.)
    hi, _ = pedaling_force_requests(0., 40., .175, normals, loads, 5., ripple=.5)
    total = lambda r: sum(np.linalg.norm(r[s]) for s in r)
    assert total(hi) > 1.3*total(lo)


@pytest.mark.parametrize('phase', [0., math.pi])
@pytest.mark.parametrize('mu', [0., .9, 5.])
def test_horizontal_power_stroke_requests_the_finite_crank_torque(phase, mu):
    normals = {'front': (0., 0., 1.), 'rear': (0., 0., 1.)}
    loads = {'front': 1e4, 'rear': 1e4}
    with np.errstate(invalid='raise'):
        forces, _ = pedaling_force_requests(phase, 40., .175, normals, loads, mu, ripple=0.)
    torque = 0.
    for side, offset in (('front', 0.), ('rear', math.pi)):
        lever = np.array([.175*math.cos(phase+offset), 0., -.175*math.sin(phase+offset)])
        assert np.isfinite(forces[side]).all()
        torque += np.cross(lever, forces[side])[1]
    assert torque == pytest.approx(40.)


def test_articulated_config_exposes_the_ripple():
    from bike_sim.physics.physical_config import ArticulatedConfig
    assert ArticulatedConfig().pedal_torque_ripple == .35
    assert ArticulatedConfig(pedal_torque_ripple=.5).pedal_torque_ripple == .5
    with pytest.raises(ValueError):
        ArticulatedConfig(pedal_torque_ripple=1.)


def test_return_foot_preload_is_a_nonnegative_force():
    from bike_sim.physics.physical_config import ArticulatedConfig
    assert ArticulatedConfig().return_foot_preload_n == 0.
    assert ArticulatedConfig(return_foot_preload_n=40.).return_foot_preload_n == 40.
    with pytest.raises(ValueError):
        ArticulatedConfig(return_foot_preload_n=-1.)
