"""Differential tests: native columnar encoder vs the encode_rows oracle."""
import numpy as np
import pytest
from native_loader import load_native
import _encode_oracle as oracle
import _recording_support as support

pytestmark = pytest.mark.usefixtures('native_module')

bike_native = load_native()


def test_make_episode_builds_retained_samples():
    rows = [support.sample_row(interval_id=i, start=i * 0.001)
            for i in range(3)]
    episode = oracle.make_episode(rows)
    assert episode.row_count == 3
    assert episode.generation == 1
    # sample_batch, not batch: batch() computes the legacy flat-columns view,
    # which requires mass/energy channels that authored rows do not carry.
    batch = episode.sample_batch(0, 3)
    assert tuple(int(i) for i in batch.interval_ids) == (0, 1, 2)


def test_make_episode_preserves_authored_value_tree():
    """The constructor round-trips channels/forces/powers_w bit-for-value.

    This is the load-bearing path for every later parity test: if the
    authored tree were dropped or mangled, the episode-mode oracle and the
    native encoder would consume the same corrupted input and
    assert_identical would pass vacuously.
    """
    channels = {
        'drive': {'watts': [150.5, None], 'ok': np.bool_(True)},
        'meta': {'note': 'spin-up', 'tries': (1, 2)},
        'empty': {},
        'nothing': None,
    }
    rows = [support.sample_row(interval_id=0,
                               forces={'crank': [3.5, -1.25]},
                               powers={'drive': 250.5},
                               channels=channels)]
    sample = oracle.make_episode(rows).sample_batch(0, 1).samples[0]
    assert dict(sample.forces).keys() == {'crank'}
    assert list(sample.forces['crank']) == [3.5, -1.25]
    assert sample.powers_w == {'drive': 250.5}
    # np scalars unbox (np.bool_ -> bool), tuples/arrays become lists,
    # empty dicts and None survive.
    assert sample.channels == {
        'drive': {'watts': [150.5, None], 'ok': True},
        'meta': {'note': 'spin-up', 'tries': [1, 2]},
        'empty': {},
        'nothing': None,
    }


def test_make_episode_rejects_malformed_channels():
    row = support.sample_row()
    del row['channels']
    with pytest.raises(ValueError, match='channels'):
        oracle.make_episode([row])
    row = support.sample_row()
    row['channels'] = [1, 2]
    with pytest.raises(ValueError, match='channels'):
        oracle.make_episode([row])


def test_column_bits_detects_negative_zero_and_nan():
    import pyarrow as pa
    a = pa.array([0.0, float('nan')])
    b = pa.array([-0.0, float('nan')])
    assert oracle.column_bits(a) != oracle.column_bits(b)
    assert oracle.column_bits(a) == oracle.column_bits(pa.array([0.0, float('nan')]))


def test_physics_and_forces_bitwise_vs_oracle():
    cat = support.support_catalog()
    n = 8
    rows = [support.sample_row(
        interval_id=i, start=i * 0.001,
        qpos=np.linspace(0, 1, n) + i, qvel=np.full(n, -0.0 if i % 2 else 0.5),
        forces={'gravity': np.full(n, i), 'motor': np.zeros(n)},
        powers={'gravity': float(i) + 0.25, 'motor': float('nan')},
        channels={'energy': support.energy_channels()['energy']})
        for i in range(6)]
    for name in ('physics', 'forces'):
        oracle.assert_identical(cat.table(name), rows)


def test_compile_plan_contract():
    from bike_sim.recording.native_encode import compile_plan
    cat = support.support_catalog()
    assert compile_plan(cat.table('physics')) == {
        'table': 'physics', 'kind': 'physics'}
    assert compile_plan(cat.table('forces')) == {
        'table': 'forces', 'kind': 'forces'}
    energy = cat.table('channel_energy')
    with pytest.raises(ValueError, match='all_roots'):
        compile_plan(energy)
    # channel_energy's map<double> leaves compile; every physical table in
    # the catalog compiles now — including the struct-bound channel tables.
    assert compile_plan(
        energy, all_roots=sorted(cat.channel_roots())) is not None
    all_roots = sorted(cat.channel_roots())
    for spec in cat.table_order():
        assert compile_plan(spec, all_roots=all_roots) is not None, spec.name


def test_encode_table_piece_shape_and_domain():
    episode = oracle.make_episode([support.sample_row()])
    plan = {'table': 'physics', 'kind': 'physics'}
    count, pieces = episode.encode_table(plan, 0, 1)
    assert count == 1
    assert set(pieces) == {'interval_id', 'time_s', 'interval_end_s',
                           'dt_s', 'qpos', 'qvel'}
    ids = pieces['interval_id']
    assert ids['kind'] == 'scalar' and ids['valid'] is None
    assert np.asarray(ids['data']).dtype == np.int64
    qpos = pieces['qpos']
    assert qpos['kind'] == 'list' and qpos['valid'] is None
    assert np.asarray(qpos['offsets']).dtype == np.int32
    # start == row_count admits the empty suffix, mirroring sample_batch.
    count, _ = episode.encode_table(plan, 1, 1)
    assert count == 0


def test_encode_table_rejects_bad_plans_and_ranges():
    episode = oracle.make_episode([support.sample_row()])
    good = {'table': 'physics', 'kind': 'physics'}
    for plan in ({'kind': 'physics'},
                 {'table': 'physics', 'kind': 'forces'},
                 {'table': 'forces', 'kind': 'physics'},
                 {'table': 'physics', 'kind': 'physics', 'columns': ['qpos']},
                 {'table': 'physics', 'kind': 'physics', 'extra': 1}):
        with pytest.raises(ValueError):
            episode.encode_table(plan, 0, 1)
    for start, limit in ((-1, 1), (0, 0), (2, 1)):
        with pytest.raises(ValueError):
            episode.encode_table(good, start, limit)


def test_forces_missing_power_is_value_error():
    # Parity with force_rows' MissingFieldError: a component without its
    # powers_w entry rejects from the native path as a ValueError too.
    row = support.sample_row(forces={'ghost': np.zeros(8)}, powers={})
    episode = oracle.make_episode([row])
    plan = {'table': 'forces', 'kind': 'forces'}
    with pytest.raises(ValueError, match='powers_w'):
        episode.encode_table(plan, 0, 1)


def test_flat_channel_tables_bitwise():
    cat = support.support_catalog()
    rows = [support.sample_row(
        interval_id=i, start=i * 0.001,
        channels={**support.drive_channels(speed=float(i)),
                  **support.sensors_channels(speed=float(i))})
        for i in range(6)]
    for name in ('channel_drive', 'channel_sensors'):
        oracle.assert_identical(cat.table(name), rows, catalog=cat)


def test_optional_leaf_absent_vs_null():
    """An absent optional leaf emits ``_present=False``; an explicit None
    emits ``_present=True`` with a NULL value. ``channel_sensors`` has no
    optional leaves (the brief's literal table), so the distinction runs on
    ``channel_drive.coasting_reason`` — the same code path either way.
    """
    cat = support.support_catalog()
    spec = cat.table('channel_drive')
    absent = support.sample_row(interval_id=0,
                                channels=support.drive_channels())
    del absent['channels']['drive']['coasting_reason']
    nullv = support.sample_row(
        interval_id=1,
        channels=support.drive_channels(drive={'coasting_reason': None}))
    oracle.assert_identical(spec, [absent, nullv], catalog=cat)
    # The distinction must be real in the oracle output, not vacuously equal.
    batch = oracle.oracle_batch(spec, [
        {'interval_id': 0, 'drive': absent['channels']['drive']},
        {'interval_id': 1, 'drive': nullv['channels']['drive']}])
    assert tuple(batch.columns['coasting_reason_present']) == (False, True)
    assert tuple(batch.columns['coasting_reason']) == (None, None)


def test_parent_mask_absent_vs_null():
    """``__<parent>_present`` distinguishes a missing member from an
    explicit-null one; ``_valid`` is False for both."""
    cat = support.support_catalog()
    spec = cat.table('channel_sensors')
    absent = support.sample_row(interval_id=0,
                                channels=support.sensors_channels())
    del absent['channels']['sensors']['encoders_rad_s']
    nullv = support.sample_row(
        interval_id=1,
        channels=support.sensors_channels(
            sensors={'encoders_rad_s': None}))
    oracle.assert_identical(spec, [absent, nullv], catalog=cat)
    batch = oracle.oracle_batch(spec, [
        {'interval_id': 0, 'sensors': absent['channels']['sensors']},
        {'interval_id': 1, 'sensors': nullv['channels']['sensors']}])
    assert tuple(batch.columns['__sensors.encoders_rad_s_present']) == (False, True)
    assert tuple(batch.columns['__sensors.encoders_rad_s_valid']) == (False, False)


def test_required_leaf_missing_vs_null_rejects():
    """A required leaf missing under a present parent rejects; so does an
    explicit null leaf. An absent/null *parent* encodes NULL instead."""
    cat = support.support_catalog()
    spec = cat.table('channel_sensors')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    for channels in ({'sensors': {}},
                     {'sensors': {'motor_torque_nm': None}},
                     {'sensors': {'encoders_rad_s': {}}}):
        episode = oracle.make_episode(
            [support.sample_row(interval_id=0, channels=channels)])
        with pytest.raises(ValueError, match='required'):
            episode.encode_table(plan, 0, 1)


def test_leaf_type_and_shape_gates():
    cat = support.support_catalog()
    spec = cat.table('channel_sensors')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    bad_shape = support.sample_row(
        interval_id=0,
        channels=support.sensors_channels(
            sensors={'frame_specific_force_body_mps2': [1.0, 2.0]}))
    with pytest.raises(ValueError, match='3 elements'):
        oracle.make_episode([bad_shape]).encode_table(plan, 0, 1)
    bad_type = support.sample_row(
        interval_id=0,
        channels=support.sensors_channels(
            sensors={'motor_torque_nm': 'not-a-number'}))
    with pytest.raises(ValueError, match='real number'):
        oracle.make_episode([bad_type]).encode_table(plan, 0, 1)


def test_unknown_channel_root_rejected():
    cat = support.support_catalog()
    bad = support.sample_row(interval_id=0, channels={'forged_root': {}})
    episode = oracle.make_episode([bad])
    spec = cat.table('channel_drive')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    with pytest.raises(ValueError, match='forged_root'):
        episode.encode_table(plan, 0, 1)


def test_unknown_channel_member_rejected():
    cat = support.support_catalog()
    channels = support.drive_channels()
    channels['drive']['bogus_member'] = 1
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=channels)])
    spec = cat.table('channel_drive')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    with pytest.raises(ValueError, match='bogus_member'):
        episode.encode_table(plan, 0, 1)


def test_map_channel_tables_bitwise():
    """``channel_energy``/``channel_suspension`` ``map<double>`` leaves.

    Row 0 carries full maps; row 1 exercises the empty-vs-absent split —
    an empty dict at a *deeper* map path is a present empty map, while an
    empty dict at a claimed-root map path routes an empty remainder and so
    encodes the leaf absent (``_distribute`` never assigns it); row 2 drops
    every root.
    """
    cat = support.support_catalog()
    rows = [
        support.sample_row(interval_id=0, start=0.0, channels={
            **support.energy_channels(),
            'component_work_j': {'chain': 0.1, 'tires': -0.2},
            **support.suspension_channels()}),
        support.sample_row(interval_id=1, start=0.001, channels={
            **support.energy_channels(energy={'elastic_energy_j': {}}),
            'component_work_j': {},
            **support.suspension_channels(
                suspension={'generalized_force_components_n': {}})}),
        support.sample_row(interval_id=2, start=0.002, channels={}),
    ]
    for name in ('channel_energy', 'channel_suspension'):
        oracle.assert_identical(cat.table(name), rows, catalog=cat)
    # The empty-vs-absent distinction is real in the native output, not a
    # vacuous byte match on both sides.
    spec = cat.table('channel_energy')
    native_rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    assert native_rb.column('elastic_energy_j').to_pylist() == \
        [[('fork_spring', 0.25)], [], None]
    assert native_rb.column('elastic_energy_j_present').to_pylist() == \
        [True, True, False]
    assert native_rb.column('component_work_j').to_pylist() == \
        [[('chain', 0.1), ('tires', -0.2)], None, None]
    assert native_rb.column('component_work_j_present').to_pylist() == \
        [True, False, False]
    suspension_rb = oracle.native_record_batch(
        cat.table('channel_suspension'), oracle.make_episode(rows),
        catalog=cat)
    assert suspension_rb.column(
        'generalized_force_components_n').to_pylist() == [
            [('fork_slide', 12.5), ('shock_slide', -3.0)], [], None]


def test_map_leaf_rejects_non_dict_and_bad_member():
    """Map conversion gates, asserted on BOTH paths: the oracle raises
    EncodingError (a ValueError) and the native side must reject too."""
    cat = support.support_catalog()
    spec = cat.table('channel_suspension')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    for member in ('a-string', {'fork_slide': 'text'},
                   {'fork_slide': [1.0]}, 5):
        channels = support.suspension_channels(
            suspension={'generalized_force_components_n': member})
        with pytest.raises(ValueError):
            oracle.oracle_batch(
                spec, [{'interval_id': 0,
                        'suspension': channels['suspension']}])
        episode = oracle.make_episode(
            [support.sample_row(interval_id=0, channels=channels)])
        with pytest.raises(ValueError):
            episode.encode_table(plan, 0, 1)


def test_map_required_leaf_missing_rejects():
    """A required ``map<double>`` absent under a present parent rejects on
    both paths; an absent root still encodes NULL via the exemption."""
    cat = support.support_catalog()
    spec = cat.table('channel_suspension')
    channels = support.suspension_channels()
    del channels['suspension']['generalized_force_components_n']
    with pytest.raises(ValueError, match='required'):
        oracle.oracle_batch(
            spec, [{'interval_id': 0, 'suspension': channels['suspension']}])
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=channels)])
    with pytest.raises(ValueError, match='required'):
        episode.encode_table(plan, 0, 1)


def test_map_except_keys_routes_owned_member():
    """``except_keys`` remainder routing on a depth-1 map leaf.

    Mirrors ``channel_rider``'s ``rider_welds``/``weld_crank_torque_nm``
    split (still ``map<struct>``-bound until Task 5): a member claimed by
    a deeper sibling leaf is its own column and never a map entry, and a
    payload whose members are ALL owned leaves the map itself absent.
    """
    from bike_sim.recording.catalog import (
        Catalog, FieldSpec, TableSpec, _EPISODE_KEY, _INTERVAL_KEY)

    def leaf(key, dtype, path, optional=False, **kwargs):
        return FieldSpec(key, dtype, 'n/a', 'interval', optional,
                         'diagnostic', 'test', f'{key} leaf', path=path,
                         **kwargs)

    spec = TableSpec(
        'channel_owned_map', 'channels', (_EPISODE_KEY, _INTERVAL_KEY),
        (leaf('bag', 'map<double>', ('bag',), optional=True,
              except_keys=('owned',)),
         leaf('owned_value', 'double', ('bag', 'owned'), optional=True)),
        'map remainder routing', 'test', claims=frozenset({'bag'}))
    mini = Catalog((spec,), support.synthetic_joints(), ())
    rows = [
        support.sample_row(interval_id=0, channels={'bag': {
            'owned': 3.5, 'free_a': 1.5, 'free_b': -0.25}}),
        # Only the owned member: the map routes an empty remainder away.
        support.sample_row(interval_id=1, channels={'bag': {'owned': 1}}),
        # An empty payload is not an empty map either — the leaf is absent.
        support.sample_row(interval_id=2, channels={'bag': {}}),
    ]
    oracle.assert_identical(spec, rows, catalog=mini)
    native_rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=mini)
    assert native_rb.column('bag').to_pylist() == [
        [('free_a', 1.5), ('free_b', -0.25)], None, None]
    assert native_rb.column('bag_present').to_pylist() == [True, False, False]
    assert native_rb.column('owned_value').to_pylist() == [3.5, 1.0, None]
    assert native_rb.column('owned_value_present').to_pylist() == \
        [True, True, False]


def test_all_list_and_map_dtypes_bitwise():
    """The full leaf matrix of this task in one mini-spec: ``bigint[]``,
    ``varchar[]`` (root-level list lanes incl. a str payload exploding to
    code points) and ``map<boolean>``/``map<varchar>``/``map<double[]>``
    alongside ``map<double>``."""
    from bike_sim.recording.catalog import (
        Catalog, FieldSpec, TableSpec, _EPISODE_KEY, _INTERVAL_KEY)

    def leaf(key, dtype, optional=False):
        return FieldSpec(key, dtype, 'n/a', 'interval', optional,
                         'diagnostic', 'test', f'{key} leaf', path=(key,))

    spec = TableSpec(
        'channel_dtypes', 'channels', (_EPISODE_KEY, _INTERVAL_KEY),
        (leaf('ints', 'bigint[]'), leaf('tags', 'varchar[]'),
         leaf('map_d', 'map<double>', optional=True),
         leaf('map_b', 'map<boolean>', optional=True),
         leaf('map_s', 'map<varchar>', optional=True),
         leaf('map_v', 'map<double[]>', optional=True)),
        'list/map dtype matrix', 'test',
        claims=frozenset({'ints', 'tags', 'map_d', 'map_b', 'map_s',
                          'map_v'}))
    mini = Catalog((spec,), support.synthetic_joints(), ())
    rows = [
        support.sample_row(interval_id=0, channels={
            'ints': [1, 2, -3], 'tags': ['a', 'b'],
            'map_d': {'x': 1.5, 'y': -0.0}, 'map_b': {'on': True},
            'map_s': {'k': 'v'}, 'map_v': {'p': [1.0, 2.0], 'q': []}}),
        support.sample_row(interval_id=1, channels={
            'ints': [], 'tags': 'hé',
            'map_d': {}, 'map_b': {}, 'map_s': {}, 'map_v': {}}),
        support.sample_row(interval_id=2, channels={}),
    ]
    oracle.assert_identical(spec, rows, catalog=mini)
    native_rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=mini)
    assert native_rb.column('ints').to_pylist() == \
        [[1, 2, -3], [], None]
    assert native_rb.column('tags').to_pylist() == \
        [['a', 'b'], ['h', 'é'], None]
    assert native_rb.column('map_d').to_pylist() == \
        [[('x', 1.5), ('y', -0.0)], None, None]
    assert native_rb.column('map_b').to_pylist() == \
        [[('on', True)], None, None]
    assert native_rb.column('map_s').to_pylist() == \
        [[('k', 'v')], None, None]
    assert native_rb.column('map_v').to_pylist() == \
        [[('p', [1.0, 2.0]), ('q', [])], None, None]


def test_struct_field_bitwise_channel_mass():
    """``endpoint_mass``: a depth-1 optional ``struct`` field authored only
    on measured rows; its own optional members stay absent inside a present
    struct (member-level ``_present`` lanes)."""
    cat = support.support_catalog()
    spec = cat.table('channel_mass')
    rows = [support.sample_row(
        interval_id=i, start=i * 0.001,
        channels=support.mass_channels(measured=(i % 2 == 0)))
        for i in range(3)]
    oracle.assert_identical(spec, rows, catalog=cat)
    rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    assert rb.column('endpoint_mass_present').to_pylist() == \
        [True, False, True]
    endpoint = rb.column('endpoint_mass').to_pylist()
    assert endpoint[1] is None
    assert endpoint[0]['mass_kg'] == 95.1
    # Absent optional members encode NULL + a False companion inside a
    # present parent struct.
    assert endpoint[0]['angular_momentum_kg_m2_s'] is None
    assert endpoint[0]['angular_momentum_kg_m2_s_present'] is False
    assert endpoint[0]['kinetic_energy_j_present'] is True


def test_struct_field_optional_member_channel_control():
    """``control.posture`` — present struct with an absent optional
    ``pelvis_offset_m``, a present member, then the whole field absent."""
    cat = support.support_catalog()
    spec = cat.table('channel_control')
    rows = [
        support.sample_row(
            interval_id=0, channels=support.control_channels()),
        support.sample_row(
            interval_id=1,
            channels=support.control_channels(pelvis_offset=True)),
        support.sample_row(
            interval_id=2,
            channels=support.control_channels(posture=False)),
    ]
    oracle.assert_identical(spec, rows, catalog=cat)
    rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    assert rb.column('posture_present').to_pylist() == [True, True, False]
    posture = rb.column('posture').to_pylist()
    assert posture[0]['pelvis_offset_m'] is None
    assert posture[0]['pelvis_offset_m_present'] is False
    assert posture[1]['pelvis_offset_m'] == [0.02, 0.01]
    assert posture[1]['pelvis_offset_m_present'] is True
    assert posture[2] is None


def test_struct_nested_and_verdict_channel_model_status():
    """``model_status`` — a required struct carrying a nested optional
    struct (``first_model_violation``), a required ``map<double>`` member
    and the ``numerically_valid`` verdict leaf (bool -> text)."""
    cat = support.support_catalog()
    spec = cat.table('channel_model_status')
    rows = [
        support.sample_row(
            interval_id=0,
            channels=support.model_status_channels(verdict=True)),
        support.sample_row(
            interval_id=1,
            channels=support.model_status_channels(
                verdict=False, violation=True,
                contact_crash_cause='tire_blowout',
                attachment_violations=['left pedal'])),
        support.sample_row(
            interval_id=2,
            channels=support.model_status_channels(
                verdict='not_evaluated')),
    ]
    oracle.assert_identical(spec, rows, catalog=cat)
    rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    status = rb.column('model_status').to_pylist()
    assert [row['numerically_valid'] for row in status] == \
        ['true', 'false', 'not_evaluated']
    assert status[0]['first_model_violation'] is None
    assert status[0]['first_model_violation_present'] is False
    assert status[1]['first_model_violation']['interval_id'] == 4
    assert status[1]['first_model_violation']['reasons'] == \
        ['energy_closure']
    assert status[1]['first_model_violation_present'] is True
    assert rb.column('contact_crash_cause').to_pylist() == \
        [None, 'tire_blowout', None]
    assert rb.column('attachment_violations').to_pylist() == \
        [None, ['left pedal'], None]


def test_struct_list_patches_channel_tires():
    """``tires.front``/``tires.rear`` structs with a ``struct[]`` ``patches``
    member: two patches on front, zero on rear, then the root absent."""
    cat = support.support_catalog()
    spec = cat.table('channel_tires')
    rows = [
        support.sample_row(
            interval_id=0,
            channels=support.tires_channels(
                front_patches=[support.tire_patch(160.0),
                               support.tire_patch(95.0, couple_world=False)],
                rear_patches=[])),
        support.sample_row(
            interval_id=1,
            channels=support.tires_channels(
                front_patches=[support.tire_patch(300.0)],
                rear_patches=[support.tire_patch(180.0)])),
        support.sample_row(interval_id=2, channels={}),
    ]
    oracle.assert_identical(spec, rows, catalog=cat)
    rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    front = rb.column('front').to_pylist()
    assert len(front[0]['patches']) == 2
    assert front[0]['patches'][0]['couple_world_nm'] == [0.0, 0.05, 0.0]
    assert front[0]['patches'][0]['couple_world_nm_present'] is True
    assert front[0]['patches'][1]['couple_world_nm'] is None
    assert front[0]['patches'][1]['couple_world_nm_present'] is False
    rear = rb.column('rear').to_pylist()
    assert rear[0]['patches'] == []
    assert front[2] is None and rear[2] is None


def test_map_struct_except_keys_and_nesting_channel_rider():
    """``rider``/``rider_welds`` ``map<struct>`` roots: entries 'saddle' and
    'bar' with sparse optional members; ``crank_torque_nm`` is routed to the
    sibling ``weld_crank_torque_nm`` leaf (``except_keys``), never a map
    entry; ``rider_support_targets`` nests ``map<struct>`` and ``struct``
    members inside a top-level struct."""
    cat = support.support_catalog()
    spec = cat.table('channel_rider')
    welds_owned_only = support.rider_channels()
    welds_owned_only['rider_welds'] = {'crank_torque_nm': 4.0}
    rows = [
        support.sample_row(interval_id=0,
                           channels=support.rider_channels()),
        support.sample_row(interval_id=1, channels=welds_owned_only),
        support.sample_row(interval_id=2, channels={}),
    ]
    oracle.assert_identical(spec, rows, catalog=cat)
    rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=cat)
    assert rb.column('weld_crank_torque_nm').to_pylist() == \
        [6.25, 4.0, None]
    assert rb.column('weld_crank_torque_nm_present').to_pylist() == \
        [True, True, False]
    welds = rb.column('rider_welds').to_pylist()
    assert dict(welds[0])['pedal_left']['normal_n'] == 45.0
    assert 'crank_torque_nm' not in dict(welds[0])
    # An all-owned remainder encodes the map field absent, not empty.
    assert welds[1] is None
    assert rb.column('rider_welds_present').to_pylist() == \
        [True, False, False]
    rider = rb.column('rider').to_pylist()
    assert set(dict(rider[0])) == {'saddle', 'bar'}
    saddle, bar = dict(rider[0])['saddle'], dict(rider[0])['bar']
    assert saddle['normal_load_n'] == 310.0
    assert len(saddle['patches']) == 1
    assert saddle['force_on_rider_n'] is None
    assert saddle['force_on_rider_n_present'] is False
    assert bar['hand_gap_m'] == 0.005
    assert bar['normal_load_n'] is None
    assert bar['patches'] is None
    assert rider[2] is None
    targets = rb.column('rider_support_targets').to_pylist()
    left = dict(targets[0]['feet'])['left']
    assert left['ik_saturated'] is False
    assert left['goal_geometry']['weld'] is True
    assert left['actual_sole_position_m'] is None
    assert targets[0]['requested_vertical_forces_n'] == \
        [('saddle', 300.0)]
    assert rb.column('rider_control').to_pylist() == [None, None, None]


def test_struct_member_validation_parity():
    """Struct projection rejects exactly where the oracle does: unknown
    members, missing required members, non-dict payloads and bad map keys —
    on both the authored-oracle and the native paths."""
    cat = support.support_catalog()
    spec = cat.table('channel_mass')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))

    def as_row(channels):
        return {'interval_id': 0,
                **{root: channels[root] for root in spec.claims
                   if root in channels}}

    bad_member = support.mass_channels()
    bad_member['endpoint_mass']['bogus'] = 1
    with pytest.raises(ValueError, match='bogus'):
        oracle.oracle_batch(spec, [as_row(bad_member)])
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=bad_member)])
    with pytest.raises(ValueError, match='bogus'):
        episode.encode_table(plan, 0, 1)

    missing = support.mass_channels()
    del missing['endpoint_mass']['mass_kg']
    with pytest.raises(ValueError, match='required'):
        oracle.oracle_batch(spec, [as_row(missing)])
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=missing)])
    with pytest.raises(ValueError, match='required'):
        episode.encode_table(plan, 0, 1)

    non_dict = support.mass_channels()
    non_dict['endpoint_mass'] = [1.0, 2.0]
    with pytest.raises(ValueError):
        oracle.oracle_batch(spec, [as_row(non_dict)])
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=non_dict)])
    with pytest.raises(ValueError):
        episode.encode_table(plan, 0, 1)


def test_map_struct_bad_key_and_member_parity():
    """``map<struct>``: an empty key rejects via ``_check_map_keys`` and a
    missing required entry member rejects inside the entry struct."""
    cat = support.support_catalog()
    spec = cat.table('channel_rider')
    plan = oracle.compile_plan(spec, all_roots=sorted(cat.channel_roots()))

    def as_row(channels):
        return {'interval_id': 0,
                **{root: channels[root] for root in spec.claims
                   if root in channels}}

    bad_key = support.rider_channels()
    bad_key['rider_welds'] = {
        '': {'force_on_rider_n': [0.0, 0.0, -1.0]}}
    with pytest.raises(ValueError, match='nonempty'):
        oracle.oracle_batch(spec, [as_row(bad_key)])
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=bad_key)])
    with pytest.raises(ValueError, match='nonempty'):
        episode.encode_table(plan, 0, 1)

    missing_member = support.rider_channels()
    del missing_member['rider_welds']['pedal_left']['force_on_rider_n']
    with pytest.raises(ValueError, match='required'):
        oracle.oracle_batch(spec, [as_row(missing_member)])
    episode = oracle.make_episode(
        [support.sample_row(interval_id=0, channels=missing_member)])
    with pytest.raises(ValueError, match='required'):
        episode.encode_table(plan, 0, 1)


def test_struct_fragments_respect_batch_rows_bound():
    """Recursive pieces slice across fragment boundaries: row windows rebase
    ``struct[]``/``map<struct>`` offsets and the shared child arrays."""
    cat = support.support_catalog()
    rows = [
        support.sample_row(
            interval_id=i,
            channels={**support.tires_channels(
                front_patches=[support.tire_patch(float(i) + 1.0)]
                if i % 2 == 0 else [],
                rear_patches=[support.tire_patch(2.0 * i)] * i),
                **support.rider_channels()})
        for i in range(5)]
    episode = oracle.make_episode(rows)
    from bike_sim.recording.contracts import PieceBatch
    from bike_sim.recording.native import NativePhysicalSource
    from bike_sim.recording.native_encode import pieces_to_arrow
    import pyarrow as pa
    for name in ('channel_tires', 'channel_rider', 'channel_model_status'):
        spec = cat.table(name)
        source = NativePhysicalSource(episode, cat)
        batches = [batch for batch in source.tables(batch_rows=2)
                   if batch.table == name]
        assert batches and all(isinstance(b, PieceBatch) for b in batches)
        assert all(b.row_count <= 2 for b in batches)
        assert sum(b.row_count for b in batches) == 5
        assembled = pa.concat_batches(
            [pieces_to_arrow(spec, b.row_count, b.pieces, 0, pa)
             for b in batches])
        reference = oracle.oracle_record_batch_from_episode(
            spec, episode, 0, catalog=cat)
        assert reference.num_rows == assembled.num_rows
        for col in reference.schema.names:
            assert oracle.column_bits(reference.column(col)) == \
                oracle.column_bits(assembled.column(col)), col


def test_applied_rows_bitwise_vs_python_mirror():
    """``episode.applied_rows`` == the Python diff mirror, chunk-stable.

    Control changes at rows 0, 3, 6; batch_rows=4 splits the last change
    off the row-0/3 chunk, so the second call must seed both the previous
    tuple and ``application_id`` from the earlier rows.
    """
    from bike_sim.recording.catalog import Catalog
    from bike_sim.recording.native import NativePhysicalSource
    cat = support.support_catalog()
    mini = Catalog(
        tuple(cat.table(name) for name in
              ('physics', 'forces', 'channel_drive', 'channel_sensors',
               'channel_control')),
        cat.coordinates, ())
    rows = [support.sample_row(
        interval_id=i, start=i * 0.001,
        channels={**support.drive_channels(speed=0.0),
                  'control': {'motor_torque_nm': float(i // 3),
                              'rider_enabled': True}})
        for i in range(9)]
    episode = oracle.make_episode(rows)

    def applied(source):
        return [(batch.row_count, dict(batch.columns))
                for batch in source.tables(batch_rows=4)
                if batch.table == 'commands_applied']

    reference = applied(NativePhysicalSource(
        oracle._PythonPathEpisode(episode), mini, ordinary_commands=True))
    native = applied(NativePhysicalSource(
        episode, mini, ordinary_commands=True))
    assert native == reference
    steps = [step for batch in native
             for step in batch[1]['step']]
    ids = [app_id for batch in native for app_id in batch[1]['application_id']]
    assert (steps, ids) == ([0, 3, 6], [0, 1, 2])


def test_verdict_leaf_coerces_bool_to_text():
    """``model_status.numerically_valid`` publishes 'true'/'false' verdict
    text: the oracle coerces inside ``_canonical``, the native leaf through
    the plan's ``convert: 'verdict'`` flag. A real catalog table for that
    path is struct-covered (still oracle-bound), so a flat synthetic spec
    exercises the wiring directly.
    """
    from bike_sim.recording.catalog import (
        Catalog, FieldSpec, TableSpec, _EPISODE_KEY, _INTERVAL_KEY)

    def leaf(key, dtype, path, optional=False):
        return FieldSpec(key, dtype, 'n/a', 'interval', optional,
                         'diagnostic', 'test', f'{key} leaf', path=path)

    spec = TableSpec(
        'channel_verdict', 'channels', (_EPISODE_KEY, _INTERVAL_KEY),
        (leaf('numerically_valid', 'varchar',
              ('model_status', 'numerically_valid')),
         leaf('model_valid', 'boolean', ('model_status', 'model_valid')),
         # Optional varchar[] leaf: a str payload must explode into code
         # points exactly like Python's enumerate-based _convert.
         leaf('tags', 'varchar[]', ('model_status', 'tags'),
              optional=True)),
        'verdict coverage', 'test', claims=frozenset({'model_status'}))
    mini = Catalog((spec,), support.synthetic_joints(), ())
    rows = [
        support.sample_row(interval_id=0, channels={'model_status': {
            'numerically_valid': True, 'model_valid': False,
            'tags': 'héllo'}}),
        support.sample_row(interval_id=1, channels={'model_status': {
            'numerically_valid': False, 'model_valid': True}}),
        support.sample_row(interval_id=2, channels={'model_status': {
            'numerically_valid': 'not_evaluated', 'model_valid': True,
            'tags': ['explicit', 'list']}}),
    ]
    oracle.assert_identical(spec, rows, catalog=mini)
    # And the native batch publishes the literal verdict text — the parity
    # above is not vacuous.
    native_rb = oracle.native_record_batch(
        spec, oracle.make_episode(rows), catalog=mini)
    assert native_rb.column('numerically_valid').to_pylist() == \
        ['true', 'false', 'not_evaluated']
    assert native_rb.column('tags').to_pylist() == \
        [['h', 'é', 'l', 'l', 'o'], None, ['explicit', 'list']]
    assert native_rb.column('tags_present').to_pylist() == \
        [True, False, True]


def test_applied_rows_missing_control_rejects():
    cat = support.support_catalog()
    row = support.sample_row(interval_id=0,
                             channels=support.drive_channels())
    episode = oracle.make_episode([row])
    with pytest.raises(ValueError, match='ordinary applied control'):
        episode.applied_rows(0, 1)


def test_forces_fragments_respect_batch_rows_bound():
    """Exploded PieceBatches stay inside the _streams batch_rows bound."""
    cat = support.support_catalog()
    n = 8
    rows = [support.sample_row(
        interval_id=i, start=i * 0.001,
        forces={'a': np.zeros(n), 'b': np.ones(n), 'c': np.full(n, 2.)},
        powers={'a': 0.0, 'b': 1.0, 'c': 2.0})
        for i in range(5)]
    episode = oracle.make_episode(rows)
    from bike_sim.recording.contracts import PieceBatch
    from bike_sim.recording.native import NativePhysicalSource
    from bike_sim.recording.native_encode import pieces_to_arrow
    import pyarrow as pa
    spec = cat.table('forces')
    source = NativePhysicalSource(episode, cat)
    forces = [batch for batch in source.tables(batch_rows=4)
              if batch.table == 'forces']
    assert forces and all(isinstance(b, PieceBatch) for b in forces)
    assert all(b.row_count <= 4 for b in forces)
    assert sum(b.row_count for b in forces) == 15
    # The fragments reassemble into exactly the oracle's record batch.
    parts = [pieces_to_arrow(spec, b.row_count, b.pieces, 0, pa)
             for b in forces]
    assembled = pa.concat_batches(parts)
    reference = oracle.oracle_record_batch_from_episode(spec, episode, 0)
    assert reference.num_rows == assembled.num_rows
    for name in reference.schema.names:
        assert oracle.column_bits(reference.column(name)) == \
            oracle.column_bits(assembled.column(name))


def test_map_fragments_respect_batch_rows_bound():
    """Map/list piece slicing across fragment boundaries matches oracle.

    ``_slice_piece`` re-bases a map's key offsets and slices the shared
    items payload; the row boundaries land mid-entry-set so any off-by-one
    in the entry span shows up here.
    """
    cat = support.support_catalog()
    rows = [
        support.sample_row(interval_id=i, start=i * 0.001, channels={
            **support.energy_channels(energy={'elastic_energy_j': {
                f'k{i}': float(i), f'k{i}b': -float(i)}}),
            'component_work_j': {'only': 0.5} if i % 2 == 0 else {}})
        for i in range(5)]
    episode = oracle.make_episode(rows)
    from bike_sim.recording.contracts import PieceBatch
    from bike_sim.recording.native import NativePhysicalSource
    from bike_sim.recording.native_encode import pieces_to_arrow
    import pyarrow as pa
    spec = cat.table('channel_energy')
    source = NativePhysicalSource(episode, cat)
    batches = [batch for batch in source.tables(batch_rows=2)
               if batch.table == 'channel_energy']
    assert batches and all(isinstance(b, PieceBatch) for b in batches)
    assert all(b.row_count <= 2 for b in batches)
    assert sum(b.row_count for b in batches) == 5
    assembled = pa.concat_batches(
        [pieces_to_arrow(spec, b.row_count, b.pieces, 0, pa)
         for b in batches])
    reference = oracle.oracle_record_batch_from_episode(spec, episode, 0)
    assert reference.num_rows == assembled.num_rows
    for name in reference.schema.names:
        assert oracle.column_bits(reference.column(name)) == \
            oracle.column_bits(assembled.column(name)), name


def test_real_episode_all_tables_bitwise():
    """Every table in catalog.table_order() is emitted natively and
    byte-identical to the oracle — including the struct-bound channel
    tables this task unlocks."""
    from _native_runtime_support import make_python_ride
    from bike_sim.native.runtime import create_native_ride
    from bike_sim.recording.catalog import build_catalog
    from bike_sim.recording.native import model_joints
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='full')
    try:
        driver.advance(65)
        driver.flush()
        episode = driver._native.seal_episode()
        cat = build_catalog(model_joints(driver.make_render_model()))
        for spec in cat.table_order():
            oracle.assert_identical(spec, episode, catalog=cat)
    finally:
        driver.close()


def test_whole_episode_single_pass(monkeypatch):
    """A real episode streams every catalog table through ``encode_table``
    in one pass — ``encode_rows`` is never called for physical tables."""
    from _native_runtime_support import make_python_ride
    import bike_sim.recording.native as native_module
    from bike_sim.native.runtime import create_native_ride
    from bike_sim.recording.catalog import build_catalog
    from bike_sim.recording.contracts import PieceBatch
    from bike_sim.recording.native import NativePhysicalSource, model_joints
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='full')
    try:
        driver.advance(65)
        driver.flush()
        episode = driver._native.seal_episode()
        cat = build_catalog(model_joints(driver.make_render_model()))
        calls = []

        def spy(*args, **kwargs):
            calls.append(args)
            raise AssertionError('encode_rows must not run on the '
                                 'all-native path')

        monkeypatch.setattr(native_module, 'encode_rows', spy)
        source = NativePhysicalSource(episode, cat)
        batches = list(source.tables(batch_rows=8))
        assert calls == []
        emitted = {batch.table for batch in batches}
        assert emitted == {spec.name for spec in cat.table_order()}
        assert all(isinstance(batch, PieceBatch) for batch in batches)
        assert sum(batch.row_count for batch in batches
                   if batch.table == 'physics') == episode.row_count
    finally:
        driver.close()
