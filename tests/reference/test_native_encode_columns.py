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
    assert compile_plan(
        energy, all_roots=sorted(cat.channel_roots())) is None


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
