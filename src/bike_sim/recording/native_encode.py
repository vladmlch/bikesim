"""Plan compiler + Arrow assembler for the native recording encoder.

``compile_plan`` turns a catalog ``TableSpec`` into the plain-data plan the
C++ projector (``OwnedNativeEpisode.encode_table``) executes; tables whose
spec uses a dtype the projector does not cover yet compile to ``None`` and
keep flowing through the reviewed ``encode_rows``/``arrow_batch`` oracle.
``pieces_to_arrow`` assembles the same ``pa.RecordBatch`` the oracle path
would produce, so publication output is bitwise-identical either way.

The module must stay importable without pyarrow — the assembler receives
``pa`` from its caller, matching the lazy-import convention in storage.py.
"""
from __future__ import annotations


def compile_plan(spec, all_roots=None):
    """TableSpec -> plain-data plan; None when a dtype is not covered yet.

    ``all_roots`` (the catalog's claimed channel roots) is required for
    channels specs — the projector enforces unknown-root rejection with it.
    """
    if spec.kind in ('physics', 'forces'):
        return {'table': spec.name, 'kind': spec.kind}
    if spec.kind == 'channels' and all_roots is None:
        raise ValueError('channels plans require all_roots')
    return None  # channels land in Tasks 3-5


_SCALAR = {'double': 'scalar', 'bigint': 'scalar', 'boolean': 'scalar',
           'varchar': 'scalar'}


def _assemble(node_dtype, piece, pa):
    """Piece dict -> pa.Array for one catalog column."""
    kind = piece['kind']
    if kind == 'scalar':
        import numpy as np
        raw = piece['data']
        # strings stay Python lists; numeric buffers are numpy already
        data = raw if node_dtype == 'varchar' else np.asarray(raw)
        if node_dtype == 'boolean' and getattr(data, 'dtype', None) != bool:
            data = np.asarray(data).astype(bool)
        mask = None
        if piece.get('valid') is not None:
            mask = ~np.asarray(piece['valid'], dtype=bool)
        return pa.array(data, type={
            'double': pa.float64(), 'bigint': pa.int64(),
            'boolean': pa.bool_(), 'varchar': pa.string()}[node_dtype],
            mask=mask)
    if kind == 'list':
        import numpy as np
        flat = piece['data']
        child = _assemble('double' if node_dtype == 'double[]' else
                          'bigint' if node_dtype == 'bigint[]' else 'varchar',
                          {'kind': 'scalar', 'data': flat}, pa)
        return pa.ListArray.from_arrays(
            pa.array(np.asarray(piece['offsets'], dtype=np.int32)), child)
    raise ValueError(f'unsupported piece kind {kind!r}')


def pieces_to_arrow(spec, row_count, pieces, episode_id, pa):
    from bike_sim.recording.schema import column_types
    expected = column_types(spec)
    names, arrays = [], []
    by_key = {f.key: f for f in spec.keys + spec.fields}
    for name in expected:
        if name == 'episode_id':
            arrays.append(pa.array([episode_id] * row_count, type=pa.int64()))
        else:
            arrays.append(_assemble(by_key[name].dtype, pieces[name], pa))
        names.append(name)
    return pa.RecordBatch.from_arrays(arrays, names=names)


def _slice_piece(piece, first, last):
    """Row window [first, last) of one piece dict, for bound honoring."""
    kind = piece['kind']
    out = {'kind': kind}
    if 'valid' in piece:
        valid = piece['valid']
        out['valid'] = None if valid is None else valid[first:last]
    if kind == 'scalar':
        out['data'] = piece['data'][first:last]
        return out
    if kind == 'list':
        import numpy as np
        offsets = np.asarray(piece['offsets'])
        low, high = int(offsets[first]), int(offsets[last])
        out['data'] = piece['data'][low:high]
        out['offsets'] = offsets[first:last + 1] - low
        return out
    raise ValueError(f'unsupported piece kind {kind!r}')


def piece_fragments(pieces, row_count, max_rows):
    """Split one ``encode_table`` piece set into <= ``max_rows`` fragments.

    The projector emits one piece set per interval window; an exploded
    table (``forces``) can exceed the ``batch_rows`` bound a yielded
    fragment must honor. Row-granular slices reproduce the oracle's flush
    boundaries: exactly ``max_rows`` per fragment plus one remainder, so
    fragment identity matches the Python path piece for piece.
    """
    if row_count <= max_rows:
        yield row_count, pieces
        return
    for first in range(0, row_count, max_rows):
        last = min(first + max_rows, row_count)
        yield last - first, {
            name: _slice_piece(piece, first, last)
            for name, piece in pieces.items()}
