"""Shared oracle plumbing for native-encoder parity tests."""
import numpy as np
import pyarrow as pa
from native_loader import load_native
import _recording_support as support
from bike_sim.recording.encoding import encode_rows
from bike_sim.recording.schema import arrow_batch

bike_native = load_native()


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
