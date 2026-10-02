from dataclasses import asdict
import numpy as np
import pytest
from bike_sim.physics.distributed_tire import HingeDensity, flat_load
from bike_sim.physics.resolution import resolve_physics_config


def test_front_and_rear_density_and_domains_are_independent():
    cfg = resolve_physics_config({'tires': {
        'backend': 'distributed_2d_reference', 'distributed': {
            'front_material': {'density': {'knots_m': [0.], 'stiffness_n_m': [2e5]},
                               'fitting_dataset_id': 'synthetic-front',
                               'valid_load_range_n': [0., 900.]},
            'rear_material': {'density': {'knots_m': [0.], 'stiffness_n_m': [3e5]},
                              'fitting_dataset_id': 'synthetic-rear',
                              'valid_load_range_n': [0., 1400.]}}}})
    front = cfg.tires.distributed.for_wheel('front')
    rear = cfg.tires.distributed.for_wheel('rear')
    assert front.density.stiffness_n_m != rear.density.stiffness_n_m
    assert front.valid_load_range_n == (0., 900.)
    assert rear.valid_load_range_n == (0., 1400.)
    assert resolve_physics_config(asdict(cfg)) == cfg


def test_legacy_fallback_and_optional_fields_survive_round_trip():
    cfg = resolve_physics_config({})
    assert cfg.tires.distributed.for_wheel('front') == cfg.tires.distributed.for_wheel('rear')
    assert resolve_physics_config(asdict(cfg)) == cfg


def test_distinct_density_changes_aggregate_flat_load():
    a = HingeDensity((0.,), (2e5,), 0., 'synthetic-front')
    b = HingeDensity((0.,), (3e5,), 0., 'synthetic-rear')
    fa = flat_load(np.array([.01]), .35, a)[0][0]
    fb = flat_load(np.array([.01]), .35, b)[0][0]
    assert fb > fa > 0.


def test_distributed_wheel_material_validations():
    from bike_sim.physics.distributed_tire import DistributedWheelMaterial, DistributedTireConfig

    with pytest.raises(ValueError, match='a material record cannot certify the plant'):
        DistributedWheelMaterial(calibration_status='certified')

    with pytest.raises(ValueError, match='an explicit density is required'):
        DistributedWheelMaterial(density=None)  # type: ignore

    with pytest.raises(ValueError, match='fitting dataset ID is required'):
        DistributedWheelMaterial(fitting_dataset_id='   ')

    with pytest.raises(ValueError, match='invalid per-wheel material domain'):
        DistributedWheelMaterial(valid_load_range_n=(500., 100.))

    with pytest.raises(ValueError, match='invalid per-wheel material domain'):
        DistributedWheelMaterial(valid_deflection_range_m=(float('nan'), 0.04))

    cfg = DistributedTireConfig()
    with pytest.raises(ValueError, match='wheel side must be front or rear'):
        cfg.for_wheel('side')

