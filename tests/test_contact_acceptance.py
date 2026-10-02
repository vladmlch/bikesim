import numpy as np
import pytest

from bike_sim.physics.distributed_tire import HingeDensity, flat_load, fit_density
from bike_sim.validation.contact_acceptance import check_bounds, contact_matrix


def test_missing_or_nonfinite_metric_fails_closed():
    bounds = {'energy_residual_ratio': (0., .05)}
    assert check_bounds({}, bounds) == {'energy_residual_ratio': False}
    assert check_bounds({'energy_residual_ratio': float('nan')}, bounds) == {'energy_residual_ratio': False}
    assert check_bounds({'energy_residual_ratio': float('inf')}, bounds) == {'energy_residual_ratio': False}
    assert check_bounds({'energy_residual_ratio': .06}, bounds) == {'energy_residual_ratio': False}
    assert check_bounds({'energy_residual_ratio': .01}, bounds) == {'energy_residual_ratio': True}


def test_check_bounds_boundary_conditions():
    bounds = {
        'lower_and_upper': (0.0, 10.0),
        'exact_zero': (0.0, 0.0),
    }
    metrics = {
        'lower_and_upper': 0.0,
        'exact_zero': 0.0,
    }
    assert check_bounds(metrics, bounds) == {
        'lower_and_upper': True,
        'exact_zero': True,
    }
    assert check_bounds({'lower_and_upper': 10.0, 'exact_zero': 1e-9}, bounds) == {
        'lower_and_upper': True,
        'exact_zero': False,
    }
    # Non-numeric or missing metrics fail closed
    assert check_bounds({'lower_and_upper': 'bad_str', 'exact_zero': None}, bounds) == {
        'lower_and_upper': False,
        'exact_zero': False,
    }


def test_synthetic_compatible_data_fit_passes():
    material = HingeDensity((0., .004), (230000., 300000.), 0., 'synthetic-fit-fixture')
    deflections = np.linspace(.001, .018, 20)
    loads = flat_load(deflections, .35, material)[0]
    fitted, report = fit_density(
        deflections, loads, .35,
        knots_m=(0., .004), dataset_id='synthetic-fit-fixture'
    )
    assert report['accepted']
    assert report['max_relative_error'] < 1e-6
    assert report['calibration_status'] == 'synthetic_fit_only'


def test_incompatible_or_bad_curve_fit_fails_closed():
    # If the curve cannot be fit by the constitutive law within error gate,
    # fit_density rejects it without mutating model status.
    deflections = np.linspace(.002, .020, 10)
    # Concave / inversely-loaded curve that cannot be fit by convex progressive springs
    unphysical_loads = np.array([500.0, 400.0, 350.0, 300.0, 250.0, 200.0, 180.0, 150.0, 120.0, 100.0])
    # Note: fit_density requires positive loads; these are positive, but decreasing, which violates
    # progressive contact mechanics. Wait, let's see if fit_density raises ValueError or returns accepted=False:
    # fit_density checks np.any(np.diff(d) <= 0) but does NOT enforce np.diff(loads) > 0.
    fitted, report = fit_density(
        deflections, unphysical_loads, .35,
        knots_m=(0., .004, .012), dataset_id='synthetic-bad-curve',
        max_relative_error=0.05
    )
    assert not report['accepted']
    assert report['max_relative_error'] > 0.05
    assert report['calibration_status'] == 'synthetic_fit_only'


def test_contact_matrix_subset_execution_and_schema():
    rows = contact_matrix(
        dts=(0.00125,),
        station_counts=(128,),
        flat_loads=(600.,),
        step_dxs=(0.02,),
        incline_angle_deg=15.,
    )
    # Should produce 1 flat + 1 step + 1 incline = 3 rows
    assert len(rows) == 3
    cases = {r['case'] for r in rows}
    assert cases == {'flat', 'step', 'incline'}
    for r in rows:
        assert r['dt_s'] == 0.00125
        assert r['station_count'] == 128
        assert isinstance(r['metrics'], dict)
        assert isinstance(r['checks'], dict)
        assert isinstance(r['accepted'], bool)
        assert r['accepted'] is True
        for check_name, passed in r['checks'].items():
            assert passed is True
