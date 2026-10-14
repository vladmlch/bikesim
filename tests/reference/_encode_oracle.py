"""Shared oracle plumbing for native-encoder parity tests."""
import numpy as np
import pyarrow as pa
from native_loader import load_native
from _bits import assert_bitwise_equal
import _recording_support as support
from bike_sim.recording.encoding import encode_rows
from bike_sim.recording.schema import arrow_batch

bike_native = load_native()


def derived_plan():
    """A fresh copy of the plan ``iter_derived_events`` passes to
    ``episode.derived_rows`` — the tracked (kind, path) table stays
    authored in ``native.py::_DERIVED_FIELDS``."""
    from bike_sim.recording.native import _DERIVED_FIELDS
    return {'fields': [{'kind': kind, 'path': list(path)}
                       for kind, path in _DERIVED_FIELDS]}


def make_episode(rows, generation=1):
    return bike_native.runtime_make_episode(list(rows), generation)


def oracle_batch(spec, rows, episode_id=0):
    """TableBatch straight through the reviewed Python encoder."""
    return encode_rows(spec, rows, episode_id=episode_id)


def compile_plan(spec, all_roots=None):
    from bike_sim.recording.native_encode import compile_plan as _compile
    return _compile(spec, all_roots=all_roots)


def native_record_batch(spec, episode, episode_id=0, catalog=None):
    # Lazy import: this module exists in Task 1 before native_encode.py
    # lands in Task 2, and the module must not require pyarrow's siblings.
    from bike_sim.recording.native_encode import compile_plan, pieces_to_arrow
    all_roots = None if catalog is None else sorted(catalog.channel_roots())
    plan = compile_plan(spec, all_roots=all_roots)
    assert plan is not None, f'{spec.name}: no native plan (fell back?)'
    row_count, pieces = episode.encode_table(plan, 0, episode.row_count)
    return pieces_to_arrow(spec, row_count, pieces, episode_id, pa)


def oracle_record_batch(spec, rows, episode_id=0):
    return arrow_batch(spec, oracle_batch(spec, rows, episode_id),
                       episode_id, pa)


class _PythonPathEpisode:
    """Duck-typed read surface hiding ``encode_table``.

    ``NativePhysicalSource`` dispatches to the native projector whenever
    the wrapped episode exposes ``encode_table``; the oracle must stay on
    the reviewed canonical + ``encode_rows`` path to remain a differential
    reference, so this delegates only the sample/batch surface the Python
    path consumes.
    """

    def __init__(self, episode):
        self._episode = episode

    @property
    def generation(self):
        return self._episode.generation

    @property
    def row_count(self):
        return self._episode.row_count

    def batch(self, start, limit):
        # Exists only to satisfy NativePhysicalSource's constructor probe;
        # _sample_batch prefers sample_batch, so this is never invoked.
        return self._episode.batch(start, limit)

    def sample_batch(self, start, limit):
        return self._episode.sample_batch(start, limit)


def oracle_record_batch_from_episode(spec, episode, episode_id=0,
                                     catalog=None):
    """Oracle RecordBatch through the production ``NativePhysicalSource``.

    Small ``batch_rows`` exercises the chunked emission path; fragments for
    the requested table are reassembled into one RecordBatch.
    """
    from bike_sim.recording.native import NativePhysicalSource
    source = NativePhysicalSource(
        _PythonPathEpisode(episode),
        support.support_catalog() if catalog is None else catalog)
    parts = [arrow_batch(spec, batch, episode_id, pa)
             for batch in source.tables(batch_rows=7)
             if batch.table == spec.name]
    if not parts:
        # A spec can legitimately emit zero fragments (e.g. forces with no
        # authored components): the oracle answer is the empty typed batch.
        parts.append(arrow_batch(
            spec, encode_rows(spec, [], episode_id=episode_id),
            episode_id, pa))
    return pa.concat_batches(parts)


def column_bits(array):
    """Deterministic byte identity of one Arrow column.

    validity bitmap + offsets + value payload bytes, recursing into
    list/map/struct children — catches -0.0, NaN payloads, empty vs absent.
    """
    a = array.combine_chunks() if isinstance(array, pa.ChunkedArray) else array
    parts = []
    if a.null_count or isinstance(a, pa.StructArray):
        buf = a.buffers()[0]
        parts.append(b'' if buf is None else buf.to_pybytes())
    if isinstance(a, pa.MapArray):
        parts.append(np.asarray(a.offsets).view(np.int32).tobytes())
        parts.append(column_bits(a.keys))
        parts.append(column_bits(a.items))
    elif isinstance(a, pa.ListArray):
        parts.append(np.asarray(a.offsets).view(np.int32).tobytes())
        parts.append(column_bits(a.values))
    elif isinstance(a, pa.StructArray):
        for i in range(a.type.num_fields):
            parts.append(column_bits(a.field(i)))
    elif a.type in (pa.float64(), pa.int64()):
        # fill_null canonicalizes masked slots (their bits live in the
        # validity bitmap above) while valid payloads — NaN bit patterns,
        # -0.0, full-width int64s — stay bitwise-visible.
        values = a.fill_null(0) if a.null_count else a
        parts.append(np.asarray(values).view(np.uint64).tobytes())
    elif a.type == pa.bool_():
        # np.asarray yields object dtype once a bool column has nulls, so
        # encode the ternary contents directly.
        parts.append(b''.join(b'\x01' if v is True else b'\x00' if v is False
                              else b'\xff' for v in a.to_pylist()))
    else:
        # UTF-8 never emits 0xfe/0xff, so the tags keep nulls unambiguous.
        parts.append(b''.join(
            b'\xff' if x is None else
            b'\xfe' + (x.encode() if isinstance(x, str) else repr(x).encode())
            for x in a.to_pylist()))
    return b'|'.join(parts)


def assert_plain_bitwise(actual, expected, what='value'):
    """Recursive bitwise compare over plain JSON-shaped value trees.

    ``thaw`` normalizes the frozen InputEvent views (mappingproxy ->
    dict, tuple -> list) before the walk, so both the boxing path's and
    the oracle's event values compare in the same shape. Floats go
    through ``assert_bitwise_equal`` — NaN payloads and -0.0 stay
    distinct; ints/bools/strings compare by exact type+value.
    """
    from bike_sim.recording.json_codec import thaw
    a, e = thaw(actual), thaw(expected)
    if isinstance(e, dict):
        assert isinstance(a, dict) and list(a) == list(e), \
            f'{what}: {a!r} != {e!r}'
        for key in e:
            assert_plain_bitwise(a[key], e[key], f'{what}[{key!r}]')
        return
    if isinstance(e, list):
        assert isinstance(a, list) and len(a) == len(e), \
            f'{what}: {a!r} != {e!r}'
        for index, pair in enumerate(zip(a, e)):
            assert_plain_bitwise(pair[0], pair[1], f'{what}[{index}]')
        return
    if isinstance(e, float) or isinstance(a, float):
        assert_bitwise_equal(a, e, what)
        return
    assert type(a) is type(e) and a == e, f'{what}: {a!r} != {e!r}'


def assert_derived_events(actual, expected):
    """Two ``iter_derived_events`` InputEvent sequences are identical:
    identity (episode_id, event_id), kind, steps and received_wall_s
    equal, ``values`` equal with ``changes`` compared bitwise."""
    assert len(actual) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected)):
        label = f'event[{index}]'
        assert (got.episode_id, got.event_id, got.kind,
                got.received_wall_s, got.processed_step,
                got.applied_step) == \
               (want.episode_id, want.event_id, want.kind,
                want.received_wall_s, want.processed_step,
                want.applied_step), label
        assert set(got.values) == set(want.values), label
        for key in ('classification', 'derived_kind', 'derivation_rule',
                    'interval_id'):
            assert got.values[key] == want.values[key], \
                f'{label}.{key}'
        assert_plain_bitwise(got.values['changes'],
                             want.values['changes'],
                             f'{label}.changes')


def assert_identical(spec, episode_or_rows, episode_id=0, catalog=None):
    """Encode the same rows/episode through both paths; compare bytes.

    The oracle always reads the sealed episode through the production
    ``NativePhysicalSource``. Authored ``dt_s`` is advisory — the runtime
    never retains it and both ``PhysicalSampleData::dt_s`` and
    ``_canonical`` recompute ``end_time_s - time_s`` — so feeding raw rows
    to ``encode_rows`` would make the oracle disagree with both production
    and the native encoder in the last ulp. It would also fail outright on
    forces/channels specs, whose claims do not cover raw row keys
    (``qpos``, ``time_s``, ``channels`` → ``UnknownFieldError``); the
    canonical projection carries exactly each spec's keys.
    """
    if hasattr(episode_or_rows, 'row_count'):   # real OwnedNativeEpisode
        episode = episode_or_rows
    else:
        episode = make_episode(list(episode_or_rows))
    oracle_rb = oracle_record_batch_from_episode(spec, episode, episode_id,
                                                 catalog=catalog)
    native_rb = native_record_batch(spec, episode, episode_id,
                                    catalog=catalog)
    assert oracle_rb.schema.equals(native_rb.schema), spec.name
    assert oracle_rb.num_rows == native_rb.num_rows
    for name in oracle_rb.schema.names:
        a, b = oracle_rb.column(name), native_rb.column(name)
        assert column_bits(a) == column_bits(b), f'{spec.name}.{name}'
