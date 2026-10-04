"""The welded preview profile is the realistic pedelec default."""
from pathlib import Path

import pytest

from bike_sim.physics.resolution import load_physics_config

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'


@pytest.fixture(scope='module')
def cfg():
    return load_physics_config(str(WELDED), {})


def test_rigid_crank_topology_is_the_default(cfg):
    assert cfg.drive.transmission_model == 'ideal_mid_drive'
    assert cfg.drive.motor_clutch is False
    assert cfg.drive.rotor_inertia_kgm2 == 0.
    assert cfg.drive.human_torque_nm == 0.     # effort comes from seated_climb


def test_assist_is_bosch_turbo_with_the_declared_envelope(cfg):
    a = cfg.drive.assist
    assert a.profile == 'bosch_cx_gen4' and a.mode == 'turbo'
    assert a.torque_curve == ((0., 85.), (120., 85.), (120.1, 47.7), (180., 0.))
    assert a.engage_torque_nm == 4.


def test_rider_numbers(cfg):
    p = cfg.drive.pedaling
    assert (p.coast_above_rpm, p.resume_below_rpm) == (120., 105.)
    assert p.coast_cadence_tau_s == .35
    assert p.rollback_brake is True
    s = cfg.drive.shifting
    assert (s.target_cadence_min_rpm, s.target_cadence_max_rpm) == (75., 110.)
    assert s.torque_factor == .3
    assert cfg.seated_climb.enabled and cfg.seated_climb.target_crank_power_w == 250.
    assert cfg.seated_climb.max_crank_torque_nm == 60.
    assert cfg.articulated.pedal_torque_ripple == .5
    assert cfg.articulated.return_foot_preload_n == 40.
    assert cfg.articulated.active_positive_power_limit_w == 450.
