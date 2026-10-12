"""Typed batch encoding of reviewed rows (R08).

Rows are ordinary Python data at documented reference values:

* ``physics`` rows address fields by their source name (``time_s``,
  ``end_time_s``, ``qpos``, ``qvel``, ...); ``channels``/``forces``/
  ``powers_w`` members are delegated to the other tables and exempted.
* ``forces`` rows are exploded per component (see ``force_rows``).
* channel-table rows carry their claimed ``channels`` roots at top level:
  ``{'energy': {...}, 'component_work_j': {...}}``. A root member is routed
  to the field whose ``path`` matches; a ``map<*>``/``struct`` field placed
  at the root itself receives the remaining members verbatim.

``encode_rows`` materializes a ``TableBatch`` fragment against a
``TableSpec``; ``decode_rows`` rebuilds rows for tests. Optional fields
project to ``<key>_present`` categories plus a value column so absence is
recorded explicitly rather than inferred from a padding value. Doubles are
preserved verbatim — NaN and ±inf round-trip and never become null — while
integral columns are validated as integers (a float NaN is rejected).
Unknown row keys raise ``UnknownFieldError`` naming the offender; a missing
required field raises ``MissingFieldError``.

Identity keys (``episode_id``, ``interval_id``) are fragment-optional: they
are typed when present and may be injected per batch via ``episode_id=``
(the publisher owns attaching episode identity when it assembles tables);
other declared keys such as ``component`` are required per row.
"""
from __future__ import annotations

import numpy as np

from bike_sim.recording.catalog import TableSpec
from bike_sim.recording.contracts import TableBatch


class EncodingError(ValueError):
    """A row cannot be represented by the reviewed catalog."""


class UnknownFieldError(EncodingError):
    """A row key has no reviewed canonical field — never silently dropped."""


class MissingFieldError(EncodingError):
    """A required reviewed field is absent from a row."""


_INJECTABLE_KEYS = frozenset({'episode_id', 'interval_id'})
_SEQUENCES = {'bigint[]', 'double[]', 'varchar[]'}
_MAP_MEMBER = {
    'map<bigint>': 'bigint', 'map<double>': 'double',
    'map<boolean>': 'boolean', 'map<varchar>': 'varchar',
    'map<double[]>': 'double[]', 'map<struct>': 'struct',
}
_STRUCTS = {'struct', 'struct[]', 'map<struct>'}


def _is_int(value) -> bool:
    return isinstance(value, (int, np.integer)) \
        and not isinstance(value, (bool, np.bool_))


def _is_real(value) -> bool:
    return isinstance(value, (float, int, np.integer, np.floating)) \
        and not isinstance(value, (bool, np.bool_))


def _convert_scalar(value, label, dtype):
    if dtype == 'bigint':
        if not _is_int(value):
            raise EncodingError(f'{label}: expected an integer, got {value!r}')
        return int(value)
    if dtype == 'double':
        if not _is_real(value):
            raise EncodingError(
                f'{label}: expected a real number, got {value!r}')
        return float(value)
    if dtype == 'boolean':
        if not isinstance(value, (bool, np.bool_)):
            raise EncodingError(f'{label}: expected a bool, got {value!r}')
        return bool(value)
    if dtype == 'varchar':
        if not isinstance(value, str):
            raise EncodingError(f'{label}: expected a string, got {value!r}')
        return value
    raise EncodingError(f'{label}: unsupported scalar dtype {dtype!r}')


def _convert(value, label, dtype, field=None):
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if dtype in _SEQUENCES:
        member = dtype[:-2]
        seq = [_convert_scalar(v, f'{label}[{i}]', member)
               for i, v in enumerate(value)]
        if field is not None and field.shape is not None \
                and len(seq) != field.shape[0]:
            raise EncodingError(
                f'{label}: expected {field.shape[0]} elements, got {len(seq)}')
        return seq
    if dtype.startswith('map<'):
        if not isinstance(value, dict):
            raise EncodingError(
                f'{label}: expected a dict, got {type(value).__name__}')
        member_dtype = _MAP_MEMBER[dtype]
        return {key: _convert(member, f'{label}[{key!r}]', member_dtype, field)
                for key, member in _check_map_keys(value, label).items()}
    if dtype == 'struct':
        return _convert_struct(value, label, field)
    if dtype == 'struct[]':
        return [_convert_struct(member, f'{label}[{i}]', field)
                for i, member in enumerate(value)]
    return _convert_scalar(value, label, dtype)


def _check_map_keys(value, label):
    for key in value:
        if not isinstance(key, str) or not key:
            raise EncodingError(
                f'{label}: map keys must be nonempty strings, got {key!r}')
    return value


def _convert_struct(value, label, field):
    if not isinstance(value, dict):
        raise EncodingError(
            f'{label}: expected a dict, got {type(value).__name__}')
    children = {child.key: child for child in field.children}
    unknown = set(value) - set(children)
    if unknown:
        raise UnknownFieldError(
            f'{label}: unknown struct members {sorted(unknown)}')
    row = {}
    for child in field.children:
        member = value.get(child.key)
        if member is None:
            if not child.optional:
                raise MissingFieldError(f'{label}.{child.key}: required')
            row[child.key] = None
        else:
            row[child.key] = _convert(member, f'{label}.{child.key}',
                                      child.dtype, child)
    return row


def _terminal_at(spec, prefix):
    found = [f for f in spec.fields if f.path == tuple(prefix)]
    if len(found) > 1:
        raise EncodingError(f'{spec.name}: ambiguous catalog at {prefix}')
    return found[0] if found else None


def _deeper_at(spec, prefix):
    depth = len(prefix)
    return [f for f in spec.fields
            if len(f.path) > depth and f.path[:depth] == tuple(prefix)]


def _distribute(spec, prefix, payload, view, label):
    """Route a ``channels`` payload at ``prefix`` into the flat field view."""
    terminal = _terminal_at(spec, prefix)
    deeper = _deeper_at(spec, prefix)
    if not isinstance(payload, dict):
        if terminal is None:
            raise UnknownFieldError(
                f'{label}: no reviewed field accepts {payload!r}')
        view[terminal.key] = payload
        return
    remainder = dict(payload)
    for member, value in payload.items():
        at_member = [f for f in deeper if f.path[len(prefix)] == member]
        if not at_member:
            continue
        remainder.pop(member)
        member_label = f'{label}.{member}'
        leaf = _terminal_at(spec, prefix + [member])
        if leaf is not None:
            view[leaf.key] = value
        elif isinstance(value, dict):
            _distribute(spec, prefix + [member], value, view, member_label)
        else:
            raise UnknownFieldError(
                f'{member_label}: no reviewed field accepts {value!r}')
    if remainder:
        if terminal is None:
            raise UnknownFieldError(
                f'{label}: unknown channel members {sorted(remainder)}')
        if terminal.dtype.startswith('map<') \
                or terminal.dtype in ('struct', 'struct[]'):
            view[terminal.key] = remainder
        else:
            raise UnknownFieldError(
                f'{label}: unknown channel members {sorted(remainder)}')


def _view(row, spec, row_index):
    """Flat ``{field_key: value}`` projection of one authored row."""
    label = f'{spec.name}[{row_index}]'
    key_names = {key.key for key in spec.keys}
    view = {}
    if spec.kind != 'channels':
        field_names = {f.path[0]: f for f in spec.fields}
        for name, value in row.items():
            if name in spec.exempt_keys:
                continue
            if name in key_names:
                view[name] = value
            elif name in field_names:
                view[field_names[name].key] = value
            else:
                raise UnknownFieldError(f'{label}: unknown field {name!r}')
        return view
    for root, payload in row.items():
        if root in key_names or root in spec.exempt_keys:
            view[root] = payload
        elif root in spec.claims:
            _distribute(spec, [root], payload, view, f'{label}.{root}')
        else:
            raise UnknownFieldError(
                f'{label}: unknown channel root {root!r}')
    return view


def encode_rows(spec: TableSpec, rows, *, episode_id=None) -> TableBatch:
    """Encode a row iterable into a typed ``TableBatch`` fragment."""
    if not isinstance(spec, TableSpec):
        raise TypeError('spec must be a TableSpec')
    if episode_id is not None and not _is_int(episode_id):
        raise EncodingError('episode_id must be an integer')
    columns = {}
    key_state = {key.key: [] for key in spec.keys}
    rows = list(rows)
    for row_index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise EncodingError(f'{spec.name}[{row_index}]: row must be a dict')
        view = _view(row, spec, row_index)
        label = f'{spec.name}[{row_index}]'
        present_roots = set(row) & spec.claims if spec.kind == 'channels' \
            else set()
        for key in spec.keys:
            value = view.get(key.key)
            if key.key == 'episode_id' and episode_id is not None:
                if value is not None and int(value) != int(episode_id):
                    raise EncodingError(
                        f'{label}.episode_id: row {value} disagrees with '
                        f'{episode_id}')
                value = episode_id
            if value is None and key.key not in _INJECTABLE_KEYS:
                raise MissingFieldError(f'{label}.{key.key}: required key')
            key_state[key.key].append(value)
        for field_spec in spec.fields:
            field_label = f'{label}.{field_spec.key}'
            if field_spec.key in view:
                value = view[field_spec.key]
                if value is None and not field_spec.optional:
                    raise MissingFieldError(
                        f'{field_label}: required field may not be null')
                encoded = None if value is None else _convert(
                    value, field_label, field_spec.dtype, field_spec)
                present = True
            elif field_spec.optional:
                encoded, present = None, False
            elif spec.kind == 'channels' \
                    and field_spec.path[0] not in present_roots:
                # The parent ``channels`` root is absent on this row: a
                # required member encodes NULL (requiredness binds within a
                # present parent; NULL in a required column therefore always
                # means "parent root absent").
                encoded, present = None, False
            else:
                raise MissingFieldError(f'{field_label}: required')
            columns.setdefault(field_spec.key, []).append(encoded)
            if field_spec.optional:
                columns.setdefault(field_spec.key + '_present',
                                   []).append(present)
    for key in spec.keys:
        values = key_state[key.key]
        if not values or all(v is None for v in values):
            continue
        if any(v is None for v in values):
            raise MissingFieldError(
                f'{spec.name}.{key.key}: key missing on some rows')
        columns[key.key] = [_convert_scalar(v, f'{spec.name}.{key.key}',
                                            key.dtype) for v in values]
    return TableBatch(table=spec.name, columns=columns,
                      row_count=len(rows))


def decode_rows(spec: TableSpec, batch: TableBatch) -> list:
    """Rebuild authored row views from a batch produced by ``encode_rows``."""
    if not isinstance(batch, TableBatch):
        raise TypeError('batch must be a TableBatch')
    n = batch.row_count
    for name, values in batch.columns.items():
        if len(values) != n:
            raise EncodingError(
                f'{spec.name}.{name}: column length {len(values)} != {n}')
    field_by_key = {f.key: f for f in spec.keys + spec.fields}
    extras = set(batch.columns) - set(field_by_key) \
        - {k + '_present' for k in field_by_key}
    if extras:
        raise UnknownFieldError(f'{spec.name}: unknown columns {sorted(extras)}')
    rows = []
    for index in range(n):
        row = {}
        for field_spec in spec.keys + spec.fields:
            if field_spec.key not in batch.columns:
                if field_spec.optional:
                    continue
                if field_spec.key in {k.key for k in spec.keys}:
                    continue
                raise MissingFieldError(
                    f'{spec.name}.{field_spec.key}: column missing')
            value = batch.columns[field_spec.key][index]
            if field_spec.optional:
                present = batch.columns.get(field_spec.key + '_present')
                if present is not None and not present[index]:
                    continue
            elif value is None:
                continue   # NULL in a required column: parent root absent
            _place(row, field_spec.path, _decode_value(value, field_spec))
        rows.append(row)
    return rows


def _place(row, path, value):
    if len(path) == 1:
        row[path[0]] = value
        return
    target = row.setdefault(path[0], {})
    for member in path[1:-1]:
        target = target.setdefault(member, {})
    target[path[-1]] = value


def _decode_value(value, field):
    if value is None:
        return None
    if field.dtype == 'struct':
        return _decode_struct(value, field)
    if field.dtype == 'struct[]':
        return [_decode_struct(member, field) for member in value]
    if field.dtype == 'map<struct>':
        return {key: _decode_struct(member, field)
                for key, member in value.items()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def _decode_struct(value, field):
    return {child.key: _decode_value(value.get(child.key), child)
            for child in field.children
            if value.get(child.key) is not None}


def force_rows(sample) -> list:
    """Explode one sample's ``forces``/``powers_w`` into per-component rows.

    Every force component must carry its recorded power; the encoder never
    recomputes a power the source did not measure.
    """
    forces = sample.get('forces') or {}
    powers = sample.get('powers_w') or {}
    rows = []
    for name, qfrc in forces.items():
        if name not in powers:
            raise MissingFieldError(
                f'forces[{name!r}] has no matching powers_w entry')
        rows.append({'interval_id': sample['interval_id'],
                     'component': name,
                     'qfrc': list(qfrc),
                     'power_w': powers[name]})
    return rows


__all__ = [
    'EncodingError', 'MissingFieldError', 'UnknownFieldError',
    'decode_rows', 'encode_rows', 'force_rows',
]
