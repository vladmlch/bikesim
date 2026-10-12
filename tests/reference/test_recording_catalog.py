"""Reviewed field/coordinate catalog, presence and typed-batch encoding."""
import math
from collections.abc import Mapping

import numpy as np
import pytest

from bike_sim.recording.catalog import build_catalog
from bike_sim.recording.encoding import (
    MissingFieldError, UnknownFieldError, decode_rows, encode_rows, force_rows,
)

from _recording_support import (
    TreePhysicalSource, energy_channels, sample_row, support_catalog,
    synthetic_joints,
)


def _energy_row(**energy):
    base = energy_channels()
    base['energy'].update(energy)
    return {root: base[root] for root in ('energy', 'component_work_j')}


def _field(catalog, table, key):
    spec = catalog.table(table)
    for field in spec.fields:
        if field.key == key:
            return field
    raise AssertionError(f'{table}: no field {key!r}')


def test_catalog_tables_are_finite_and_fully_described():
    catalog = support_catalog()
    expected = {'physics', 'forces', 'channel_mass', 'channel_energy',
                'channel_sensors', 'channel_drive', 'channel_tires',
                'channel_suspension', 'channel_control', 'channel_rider',
                'channel_model_status'}
    assert {spec.name for spec in catalog.table_order()} == expected
    for spec in catalog.table_order():
        assert spec.keys, spec.name
        seen = set()
        for field in spec.fields:
            assert field.key not in seen, field.key
            seen.add(field.key)
            assert field.description, field.key
            assert field.unit, field.key
            assert field.measurement, field.key
            assert field.source, field.key
            assert field.phase, field.key


def test_coordinate_catalog_maps_qpos_and_dof_addresses():
    catalog = support_catalog()
    coordinates = catalog.coordinates
    assert len(coordinates.qpos) == 8
    assert len(coordinates.dof) == 8
    root_x = coordinates.qpos[0]
    assert root_x.joint == 'root_x' and root_x.unit == 'm'
    pitch = coordinates.qpos[2]
    assert pitch.joint == 'root_pitch' and pitch.unit == 'rad'
    assert coordinates.dof[0].unit == 'm/s'
    assert coordinates.dof[2].unit == 'rad/s'
    # Mixed generalized-force units follow the DOF map, not a single unit.
    assert coordinates.force_unit(0) == 'N'        # slide: newtons
    assert coordinates.force_unit(2) == 'N·m'      # hinge: newton-metres
    assert coordinates.force_unit(5) == 'N·m'      # crank_spin


def test_legacy_flat_aliases_resolve_to_canonical_fields():
    catalog = support_catalog()
    cases = {
        'interval_start_s': ('physics.time_s', None, None),
        'interval_dt_s': ('physics.dt_s', None, None),
        'x_m': ('physics.qpos', 0, None),
        'pitch_rad': ('physics.qpos', 2, None),
        'speed_mps': ('physics.qvel', 0, None),
        'prestep_qpos_5': ('physics.qpos', 5, None),
        'prestep_qvel_3': ('physics.qvel', 3, None),
        'qpos.4': ('physics.qpos', 4, None),
        'qvel.7': ('physics.qvel', 7, None),
        'compiled_mass_kg': ('channel_mass.mass_kg', None, None),
        'compiled_com_y_m': ('channel_mass.com_m', 1, None),
        'kinetic_energy_j': ('channel_mass.kinetic_energy_j', None, None),
        'gravitational_energy_j': ('channel_mass.gravitational_energy_j', None, None),
        'mechanical_energy_j': ('channel_energy.mechanical_energy_j', None, None),
        'energy_residual_j': ('channel_energy.residual_j', None, None),
        'powers_w.chain': ('forces.power_w', None, 'chain'),
        'power_chain_w': ('forces.power_w', None, 'chain'),
        'chain_power_w': ('forces.power_w', None, 'chain'),
        'chain_work_j': ('channel_energy.component_work_j', None, 'chain'),
        'component_work_j.tires': ('channel_energy.component_work_j', None, 'tires'),
        'drive.cadence_rpm': ('channel_drive.cadence_rpm', None, None),
        'tires.front.normal_load_n': ('channel_tires.front.normal_load_n', None, None),
        'sensors.encoders_rad_s.crank': ('channel_sensors.encoder_crank_rad_s', None, None),
    }
    for alias, (canonical, index, parameter) in cases.items():
        resolved = catalog.resolve_alias(alias)
        assert resolved is not None, alias
        assert resolved.canonical == canonical, alias
        assert resolved.index == index, alias
        assert resolved.parameter == parameter, alias
    assert catalog.resolve_alias('not_a_field') is None


def test_r10_presence_and_exceptional_values_round_trip():
    catalog = support_catalog()
    spec = catalog.table('channel_energy')
    rows = [
        _energy_row(residual_j=float('nan')),          # NaN stays a float NaN
        _energy_row(residual_j=None),                  # explicit null
        {root: dict(value) for root, value in          # missing key entirely
         ((k, v) for k, v in _energy_row().items())},
    ]
    del rows[2]['energy']['residual_j']
    batch = encode_rows(spec, rows)
    assert batch.table == 'channel_energy'
    assert batch.row_count == 3
    assert math.isnan(batch.columns['residual_j'][0])
    assert batch.columns['residual_j'][1] is None
    assert batch.columns['residual_j'][2] is None
    assert list(batch.columns['residual_j_present']) == [True, True, False]
    decoded = decode_rows(spec, batch)
    assert math.isnan(decoded[0]['energy']['residual_j'])
    assert decoded[1]['energy']['residual_j'] is None
    assert 'residual_j' not in decoded[2]['energy']


def test_false_zero_empty_and_signed_values_survive_encoding():
    catalog = support_catalog()
    spec = catalog.table('channel_energy')
    row = _energy_row(constraint_work_ok=False, loss_j=-0.0,
                      elastic_energy_j={}, residual_j=0.0)
    batch = encode_rows(spec, [row])
    assert list(batch.columns['constraint_work_ok']) == [False]
    value = batch.columns['loss_j'][0]
    assert value == 0.0 and math.copysign(1.0, value) < 0.0
    assert list(batch.columns['elastic_energy_j']) == [{}]
    decoded = decode_rows(spec, batch)[0]
    assert decoded['energy']['constraint_work_ok'] is False
    negative_zero = decoded['energy']['loss_j']
    assert negative_zero == 0.0 and math.copysign(1.0, negative_zero) < 0.0
    assert decoded['energy']['elastic_energy_j'] == {}
    assert decoded['energy']['residual_j'] == 0.0


def test_nonfinite_doubles_are_not_null():
    catalog = support_catalog()
    spec = catalog.table('channel_energy')
    row = _energy_row(mechanical_energy_j=float('inf'),
                      external_work_j=float('-inf'))
    batch = encode_rows(spec, [row])
    assert list(batch.columns['mechanical_energy_j']) == [float('inf')]
    assert list(batch.columns['external_work_j']) == [float('-inf')]
    decoded = decode_rows(spec, batch)[0]
    assert decoded['energy']['mechanical_energy_j'] == float('inf')
    assert decoded['energy']['external_work_j'] == float('-inf')


def test_nullable_parent_objects_carry_their_own_presence():
    catalog = support_catalog()
    spec = catalog.table('channel_mass')
    measured = {'mass_kg': 80.0, 'com_m': [0.0, 0.0, 0.5]}
    rows = [
        {'mass': measured,
         'endpoint_mass': dict(measured, kinetic_energy_j=2.0)},
        {'mass': measured, 'endpoint_mass': None},     # explicit null parent
        {'mass': measured},                            # parent key absent
    ]
    batch = encode_rows(spec, rows)
    assert list(batch.columns['endpoint_mass_present']) == [True, True, False]
    assert batch.columns['endpoint_mass'][0]['kinetic_energy_j'] == 2.0
    assert batch.columns['endpoint_mass'][1] is None
    assert batch.columns['endpoint_mass'][2] is None
    decoded = decode_rows(spec, batch)
    assert decoded[0]['endpoint_mass']['mass_kg'] == 80.0
    assert decoded[1]['endpoint_mass'] is None
    assert 'endpoint_mass' not in decoded[2]


def test_empty_lists_round_trip_separately_from_absent():
    catalog = support_catalog()
    spec = catalog.table('channel_model_status')
    status = {'model_status': {
        'model_valid': True, 'first_model_violation': None,
        'model_violation_counts': {}, 'numerically_valid': 'not_evaluated',
        'calibration_status': 'parameterized_unvalidated'}}
    batch = encode_rows(spec, [dict(status, attachment_violations=()),
                               dict(status)])
    assert list(batch.columns['attachment_violations']) == [[], None]
    assert list(batch.columns['attachment_violations_present']) == [True, False]
    decoded = decode_rows(spec, batch)
    assert decoded[0]['attachment_violations'] == []
    assert 'attachment_violations' not in decoded[1]


def test_vector_fields_keep_shape_axes_and_units():
    catalog = support_catalog()
    accel = _field(catalog, 'channel_sensors',
                   'frame_specific_force_body_mps2')
    assert accel.shape == (3,)
    assert accel.axes == ('x', 'y', 'z')
    assert accel.unit == 'm/s²'
    com = _field(catalog, 'channel_mass', 'com_m')
    assert com.shape == (3,) and com.axes == ('x', 'y', 'z')
    qpos = _field(catalog, 'physics', 'qpos')
    assert qpos.unit == 'per_coordinate'
    spec = catalog.table('channel_sensors')
    row = {'sensors': dict(_sensors(), **{
        'frame_specific_force_body_mps2': [1.0, 2.0]})}
    with pytest.raises(ValueError):
        encode_rows(spec, [row])   # wrong vector width is rejected


def _sensors():
    return {
        'frame_specific_force_body_mps2': [0.0, 0.0, 9.81],
        'frame_gyro_body_rad_s': [0.0, 0.0, 0.0],
        'encoders_rad_s': {'front_wheel': 1.0, 'rear_wheel': 2.0, 'crank': 3.0},
        'motor_torque_nm': 4.0,
        'human_torque_nm': 5.0,
    }


def test_unknown_fields_fail_with_a_named_error():
    catalog = support_catalog()
    spec = catalog.table('channel_energy')
    row = _energy_row()
    row['energy']['definitely_not_a_channel_leaf'] = 1.0
    with pytest.raises(UnknownFieldError, match='definitely_not_a_channel_leaf'):
        encode_rows(spec, [row])
    with pytest.raises(UnknownFieldError):
        list(TreePhysicalSource(
            [sample_row(0, channels={'mystery_domain': {'x': 1.0}})]).tables(
                batch_rows=4))
    good = _energy_row()
    del good['energy']['mechanical_energy_j']      # required key missing
    with pytest.raises(MissingFieldError, match='mechanical_energy_j'):
        encode_rows(spec, [good])


def test_source_backed_catalog_entries_for_reviewed_semantics():
    catalog = support_catalog()
    # Command ceiling versus requested target: distinct fields, distinct meaning.
    ceiling = _field(catalog, 'channel_drive', 'motor_limit_nm')
    setpoint = _field(catalog, 'channel_drive', 'motor_setpoint_nm')
    request = _field(catalog, 'channel_drive', 'motor_request_nm')
    for field in (ceiling, setpoint, request):
        assert field.unit == 'N·m'
    assert 'ceiling' in ceiling.description
    assert 'request' in setpoint.description or 'target' in setpoint.description
    assert ceiling.key != setpoint.key != request.key
    applied_ceiling = _field(catalog, 'channel_control', 'motor_limit_nm')
    assert 'ceiling' in applied_ceiling.description
    # Incoming state: qpos/qvel are interval-start ("prestep") state.
    for key in ('qpos', 'qvel'):
        field = _field(catalog, 'physics', key)
        assert field.phase == 'interval_start'
        assert 'incoming' in field.description
    # Specific acceleration: proper acceleration, gravity not subtracted again.
    accel = _field(catalog, 'channel_sensors', 'frame_specific_force_body_mps2')
    assert accel.unit == 'm/s²'
    assert 'specific force' in accel.description
    assert 'gravity' in accel.description
    assert accel.phase == 'interval_start'
    # Crank-side torque is named against wheel-side torque.
    sensed = _field(catalog, 'channel_drive', 'human_sensor_nm')
    assert sensed.unit == 'N·m'
    assert 'crank' in sensed.description and 'wheel' in sensed.description
    human = _field(catalog, 'channel_sensors', 'human_torque_nm')
    assert human.unit == 'N·m' and 'crank' in human.description


def test_tree_physical_source_emits_typed_batches_through_the_catalog():
    samples = [sample_row(i, start=0.001 * i) for i in range(3)]
    source = TreePhysicalSource(samples)
    batches = list(source.tables(batch_rows=2))
    by_table = {}
    for batch in batches:
        assert batch.row_count <= 2
        by_table.setdefault(batch.table, []).append(batch)
    physics = by_table['physics']
    assert [batch.row_count for batch in physics] == [2, 1]
    assert list(physics[0].columns['interval_id']) == [0, 1]
    assert list(physics[1].columns['interval_id']) == [2]
    for batch in by_table['channel_energy']:
        assert batch.row_count == len(batch.columns['interval_id'])
        assert all(isinstance(i, int) for i in batch.columns['interval_id'])
    # Tables that saw no rows still report an explicit empty batch.
    assert by_table.get('channel_rider') is not None


def test_forces_explode_per_component_with_vector_and_power():
    qvel = [1.0] + [0.0] * 7
    row = sample_row(0, qvel=qvel,
                     forces={'chain': [0.5] + [0.0] * 7,
                             'tires': [2.0] + [0.0] * 7},
                     powers={'chain': 0.5, 'tires': 2.0})
    rows = list(force_rows(row))
    assert {r['component'] for r in rows} == {'chain', 'tires'}
    chain = next(r for r in rows if r['component'] == 'chain')
    assert chain['power_w'] == 0.5
    assert chain['qfrc'] == [0.5] + [0.0] * 7
    catalog = support_catalog()
    spec = catalog.table('forces')
    batch = encode_rows(spec, rows)
    assert batch.row_count == 2
    assert list(batch.columns['component']) == ['chain', 'tires']
    decoded = decode_rows(spec, batch)
    assert decoded[0]['power_w'] == 0.5


def test_tree_source_rejects_batch_arguments_and_bad_intervals():
    source = TreePhysicalSource([sample_row(0)])
    with pytest.raises(ValueError):
        list(source.tables(batch_rows=0))
    spec = support_catalog().table('physics')
    with pytest.raises(ValueError):
        encode_rows(spec, [sample_row(0, qpos=[1.0, 'x'])])


def test_catalog_rejects_malformed_joint_descriptors():
    with pytest.raises(ValueError):
        build_catalog(({'name': 'x'},))          # not a JointCoords
    with pytest.raises(ValueError):
        build_catalog([type('J', (), {'name': 'a', 'joint_type': 'weird',
                                      'qposadr': 0, 'dofadr': 0})()])
