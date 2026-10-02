from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from bike_sim.physics.distributed_tire import HingeDensity, DistributedWheelMaterial, flat_load
from bike_sim.physics.tire_curve import TabulatedTireSpec
from bike_sim.physics.tire_calibration import TireCalibration
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.plant_profile import build_candidate, write_physics_toml, _toml_value


def calibration(side, radius):
    d = np.linspace(.001, .018, 20)
    density = HingeDensity((0., .004), (230000., 300000.), 0., 'synthetic-profile-test')
    loads = flat_load(d, radius, density)[0]
    domain = (0., float(loads[-1]))
    curve = TabulatedTireSpec((0., *d), (0., *loads), 0., 100000.,
                             'synthetic-profile-test', domain)
    c = TireCalibration('synthetic-' + side, 'synthetic', 'fixture only',
                        radius, .071, .035, curve)
    m = DistributedWheelMaterial(density, c.dataset_id, domain, (0., float(d[-1])))
    return c, m


def make_candidate():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    c = build_candidate(base, f, r, fm, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)
    return base, c


def test_candidate_is_explicit_and_does_not_rewrite_baseline(tmp_path):
    base, c = make_candidate()
    assert base.articulated.saddle_attachment == 'weld'
    assert c.articulated.saddle_attachment == 'flat'
    assert c.articulated.pedal_attachment == 'flat'
    assert c.articulated.grip_attachment == 'spring'
    assert c.drive.assist == base.drive.assist
    assert c.drive.shifting.upshift_slip_mode == 'magnitude'
    assert not c.seated_climb.enabled
    p = tmp_path / 'candidate.toml'
    write_physics_toml(c, p)
    assert load_physics_config(p) == c


def test_mismatched_density_cannot_be_used_as_a_calibrated_profile():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    bad = replace(fm, density=HingeDensity((0.,), (1e7,), 0., 'bad synthetic fit'))
    with pytest.raises(ValueError, match='static response'):
        build_candidate(base, f, r, bad, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)


def test_compiled_radii_mismatch_or_shape_rejected():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    with pytest.raises(ValueError, match='compiled front and rear radii are required'):
        build_candidate(base, f, r, fm, rm, compiled_radii_m=(.372,),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)

    with pytest.raises(ValueError, match='calibration radius does not match'):
        build_candidate(base, f, r, fm, rm, compiled_radii_m=(.380, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)


def test_closure_time_constant_guard():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    # base.closure_time_constant_s is 0.0025. 2 * 0.0013 = 0.0026 > 0.0025
    with pytest.raises(ValueError, match='closure compliance'):
        build_candidate(base, f, r, fm, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.,
                        timestep_s=.0013)


def test_mismatched_dataset_id_rejected():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    mismatched_m = replace(fm, fitting_dataset_id='different-id')
    with pytest.raises(ValueError, match='distributed material and radial record IDs differ'):
        build_candidate(base, f, r, mismatched_m, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)


def test_domain_exceeded_rejected():
    base = load_physics_config('examples/research/viewer_physics_welded.toml')
    f, fm = calibration('front', .372)
    r, rm = calibration('rear', .352)
    exceeded_load = replace(fm, valid_load_range_n=(0., f.material.valid_load_range_n[1] + 100.))
    with pytest.raises(ValueError, match='distributed load domain exceeds the radial record'):
        build_candidate(base, f, r, exceeded_load, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)

    exceeded_defl = replace(fm, valid_deflection_range_m=(0., f.material.deflection_m[-1] + .01))
    with pytest.raises(ValueError, match='distributed deflection domain exceeds the radial record'):
        build_candidate(base, f, r, exceeded_defl, rm, compiled_radii_m=(.372, .352),
                        rider_power_limit_w=600., grip_pair_force_limit_n=600.)


def test_write_physics_toml_existing_file_rejected(tmp_path):
    _, c = make_candidate()
    p = tmp_path / 'existing.toml'
    p.write_text('dummy')
    with pytest.raises(FileExistsError, match='must not overwrite'):
        write_physics_toml(c, p)


def test_toml_value_and_key_validation(monkeypatch, tmp_path):
    assert _toml_value([1, 2, "a", True]) == '[1, 2, "a", true]'
    with pytest.raises(ValueError, match='unsupported physics configuration scalar'):
        _toml_value(object())

    _, c = make_candidate()
    monkeypatch.setattr('bike_sim.physics.plant_profile.asdict', lambda _: {'invalid-key!': 1})
    with pytest.raises(ValueError, match='invalid configuration key'):
        write_physics_toml(c, tmp_path / 'out.toml')
