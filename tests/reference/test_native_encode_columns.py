"""Differential tests: native columnar encoder vs the encode_rows oracle."""
import numpy as np
import pytest
from native_loader import load_native
import _encode_oracle as oracle
import _recording_support as support

bike_native = load_native()
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


def test_column_bits_detects_negative_zero_and_nan():
    import pyarrow as pa
    a = pa.array([0.0, float('nan')])
    b = pa.array([-0.0, float('nan')])
    assert oracle.column_bits(a) != oracle.column_bits(b)
    assert oracle.column_bits(a) == oracle.column_bits(pa.array([0.0, float('nan')]))
