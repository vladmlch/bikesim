import json
from pathlib import Path
import pytest
from bike_sim.physics.tire_calibration import (
    TireCalibration,
    calibration_from_record,
    load_calibration,
)
from bike_sim.physics.tire_curve import MaterialRangeError, TabulatedTireSpec


def record():
    return dict(
        dataset_id='synthetic-test-only',
        source_kind='synthetic',
        tyre_label='fixture, not user tyre',
        radius_m=.35,
        inflated_width_m=.071,
        rim_internal_width_m=.035,
        pressure_pa_gauge=100000.,
        radial_c_ns_m=100.,
        deflection_m=[0., .01, .02],
        force_n=[0., 400., 1000.],
        valid_load_range_n=[0., 1000.],
        provenance='authored unit-test curve, no measurements',
    )


def test_pressure_change_alone_does_not_change_force_law():
    a = calibration_from_record(record())
    r = record()
    r['pressure_pa_gauge'] = 80000.
    b = calibration_from_record(r)
    assert a.material.elastic_response(.005) == b.material.elastic_response(.005)
    assert a.material.pressure_pa_gauge != b.material.pressure_pa_gauge


@pytest.mark.parametrize('field', ['dataset_id', 'radius_m', 'provenance', 'source_kind'])
def test_missing_calibration_context_is_rejected(field):
    r = record()
    del r[field]
    with pytest.raises(ValueError):
        calibration_from_record(r)


def test_outside_curve_domain_is_not_extrapolated():
    c = calibration_from_record(record())
    with pytest.raises(MaterialRangeError):
        c.material.elastic_response(.021)


def test_extra_field_is_rejected():
    r = record()
    r['extra_field'] = 'invalid'
    with pytest.raises(ValueError, match='schema'):
        calibration_from_record(r)


def test_non_dict_record_is_rejected():
    with pytest.raises(ValueError, match='schema'):
        calibration_from_record("not a dict")


@pytest.mark.parametrize('bad_source', ['estimated', 'measured_and_verified', '', 'other'])
def test_invalid_source_kind_is_rejected(bad_source):
    r = record()
    r['source_kind'] = bad_source
    with pytest.raises(ValueError, match='source_kind'):
        calibration_from_record(r)


def test_measured_source_kind_is_accepted():
    r = record()
    r['source_kind'] = 'measured'
    c = calibration_from_record(r)
    assert c.source_kind == 'measured'


@pytest.mark.parametrize('field', ['dataset_id', 'tyre_label'])
def test_empty_string_field_is_rejected(field):
    r = record()
    r[field] = '   '
    with pytest.raises(ValueError, match=f'{field} is required'):
        calibration_from_record(r)


@pytest.mark.parametrize('geom_field', ['radius_m', 'inflated_width_m', 'rim_internal_width_m'])
@pytest.mark.parametrize('bad_val', [0., -0.05, float('inf'), float('nan')])
def test_non_positive_or_invalid_geometry_is_rejected(geom_field, bad_val):
    r = record()
    r[geom_field] = bad_val
    with pytest.raises(ValueError):
        calibration_from_record(r)


def test_load_calibration_from_file(tmp_path: Path):
    path = tmp_path / "calibration.json"
    r = record()
    path.write_text(json.dumps(r), encoding='utf-8')

    loaded = load_calibration(path)
    assert isinstance(loaded, TireCalibration)
    assert loaded.dataset_id == r['dataset_id']
    assert loaded.source_kind == r['source_kind']
    assert loaded.tyre_label == r['tyre_label']
    assert loaded.radius_m == pytest.approx(r['radius_m'])
    assert loaded.inflated_width_m == pytest.approx(r['inflated_width_m'])
    assert loaded.rim_internal_width_m == pytest.approx(r['rim_internal_width_m'])
    assert isinstance(loaded.material, TabulatedTireSpec)
    assert loaded.material.radial_c_ns_m == pytest.approx(r['radial_c_ns_m'])
    assert loaded.material.pressure_pa_gauge == pytest.approx(r['pressure_pa_gauge'])


def test_invalid_material_type_rejected():
    with pytest.raises(ValueError, match='explicit radial curve'):
        TireCalibration(
            dataset_id='test',
            source_kind='synthetic',
            tyre_label='test',
            radius_m=0.35,
            inflated_width_m=0.071,
            rim_internal_width_m=0.035,
            material="not_a_tabulated_spec",  # type: ignore[arg-type]
        )
