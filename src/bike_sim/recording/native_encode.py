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


# Flat-leaf dtypes the channels projector covers; struct fields compile
# to None (whole table falls back to the oracle) until their emitter lands.
_LEAF_DTYPES = {'double', 'bigint', 'boolean', 'varchar',
                'double[]', 'bigint[]', 'varchar[]',
                'map<double>', 'map<boolean>', 'map<varchar>',
                'map<double[]>'}

# map<K> member dtype -> the leaf dtype the items piece assembles as.
_MAP_MEMBER = {'map<double>': 'double', 'map<boolean>': 'boolean',
               'map<varchar>': 'varchar', 'map<double[]>': 'double[]'}

# The runtime bool|str tri-state at this absolute channel path publishes as
# lowercase verdict text — the same coercion NativePhysicalSource._canonical
# applies before encoding; the leaf plan node carries it so the native
# emitter reproduces it verbatim.
_VERDICT_PATH = ('model_status', 'numerically_valid')


def compile_plan(spec, all_roots=None):
    """TableSpec -> plain-data plan; None when a dtype is not covered yet.

    ``all_roots`` (the catalog's claimed channel roots) is required for
    channels specs — the projector enforces unknown-root rejection with it.
    """
    if spec.kind in ('physics', 'forces'):
        return {'table': spec.name, 'kind': spec.kind}
    if all_roots is None:
        raise ValueError('channels plans require all_roots')
    columns, masks = [], []
    for f in spec.fields:
        if f.dtype not in _LEAF_DTYPES:
            return None
        columns.append({'name': f.key, 'dtype': f.dtype,
                        'path': list(f.path), 'optional': f.optional,
                        'shape': list(f.shape) if f.shape else None,
                        'except_keys': list(f.except_keys),
                        'convert': 'verdict' if tuple(f.path) == _VERDICT_PATH
                                   else None})
    # One mask pair per proper field-path prefix plus one per claimed root —
    # the same name set schema.parent_masks computes, in field order.
    for f in spec.fields:
        for depth in range(1, len(f.path)):
            prefix = f.path[:depth]
            joined = '.'.join(prefix)
            masks += [{'name': '__' + joined + '_present',
                       'path': list(prefix), 'op': 'present'},
                      {'name': '__' + joined + '_valid',
                       'path': list(prefix), 'op': 'valid'}]
    for root in sorted(spec.claims):
        masks += [{'name': '__' + root + '_present',
                   'path': [root], 'op': 'present'},
                  {'name': '__' + root + '_valid',
                   'path': [root], 'op': 'valid'}]
    # Dedupe while preserving order (roots are both claims and prefixes).
    out, have = [], set()
    for m in masks:
        if m['name'] not in have:
            have.add(m['name'])
            out.append(m)
    return {'table': spec.name, 'kind': 'channels',
            'roots': sorted(spec.claims), 'all_roots': list(all_roots),
            'columns': columns, 'masks': out}


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
        # Required x[] leaves can still be NULL when a parent root/prefix
        # is absent-or-null, so list pieces carry a validity mask too.
        mask = None
        if piece.get('valid') is not None:
            mask = pa.array(~np.asarray(piece['valid'], dtype=bool),
                            type=pa.bool_())
        return pa.ListArray.from_arrays(
            pa.array(np.asarray(piece['offsets'], dtype=np.int32)),
            child, mask=mask)
    if kind == 'map':
        import numpy as np
        keys = pa.array(list(piece['keys']), type=pa.string())
        items = _assemble(_MAP_MEMBER[node_dtype], piece['items'], pa)
        # Same absent-vs-null convention as list leaves: a NULL map slot
        # carries an empty key span plus a validity bit.
        mask = None
        if piece.get('valid') is not None:
            mask = pa.array(~np.asarray(piece['valid'], dtype=bool),
                            type=pa.bool_())
        return pa.MapArray.from_arrays(
            pa.array(np.asarray(piece['offsets'], dtype=np.int32)),
            keys, items, mask=mask)
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
            field = by_key.get(name)
            # Optional-leaf ``_present`` companions and ``__`` parent/root
            # masks are not FieldSpecs; arrow_batch falls back to bool_ for
            # them, so the matching pieces are boolean scalars.
            arrays.append(_assemble(
                field.dtype if field is not None else 'boolean',
                pieces[name], pa))
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
    if kind == 'map':
        import numpy as np
        offsets = np.asarray(piece['offsets'])
        low, high = int(offsets[first]), int(offsets[last])
        out['keys'] = piece['keys'][low:high]
        out['items'] = _slice_piece(piece['items'], low, high)
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
