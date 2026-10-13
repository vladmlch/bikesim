"""Differential tests: native columnar encoder vs the encode_rows oracle."""
import numpy as np
import pytest
import _encode_oracle as oracle
import _recording_support as support

pytestmark = pytest.mark.usefixtures('native_module')


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
