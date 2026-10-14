"""Lossless JSON boundary for metadata and CLI results, never numeric storage.

Nonfinite binary64 values use tagged objects. Ordinary objects containing the
reserved tag are escaped, so application metadata cannot impersonate a float.
Strict parsing rejects duplicate keys and bare NaN/Infinity tokens.
"""
from __future__ import annotations

import base64
from collections.abc import Mapping
from datetime import date, datetime, time, timedelta
from decimal import Decimal
import json
import math
from uuid import UUID

TAG = '$bike_recording'


def to_json_value(value):
    if value is None or type(value) in (bool, int, str):
        return value
    if isinstance(value, float):
        if math.isnan(value):
            return {TAG: 'float64', 'value': 'nan'}
        if math.isinf(value):
            return {TAG: 'float64', 'value': '+inf' if value > 0 else '-inf'}
        return value  # json preserves -0.0
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise TypeError('JSON object keys must be strings')
        if TAG in value:
            return {TAG: 'object', 'items': [[k, to_json_value(v)] for k, v in value.items()]}
        return {key: to_json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_json_value(item) for item in value]
    if isinstance(value, bytes):
        return {TAG: 'bytes', 'value': base64.b64encode(value).decode('ascii')}
    if isinstance(value, Decimal):
        return {TAG: 'decimal', 'value': str(value)}
    if isinstance(value, (datetime, date, time)):
        return {TAG: type(value).__name__, 'value': value.isoformat()}
    if isinstance(value, timedelta):
        return {TAG: 'timedelta', 'days': value.days, 'seconds': value.seconds,
                'microseconds': value.microseconds}
    if isinstance(value, UUID):
        return str(value)
    # Avoid importing NumPy on the ordinary read-only metadata path.
    if type(value).__module__.startswith('numpy'):
        return to_json_value(value.tolist() if hasattr(value, 'tolist') else value.item())
    raise TypeError(f'unsupported recording JSON value: {type(value).__name__}')


def from_json_value(value):
    if isinstance(value, list):
        return [from_json_value(item) for item in value]
    if not isinstance(value, dict):
        return value
    if TAG not in value:
        return {key: from_json_value(item) for key, item in value.items()}
    kind = value[TAG]
    if kind == 'float64' and set(value) == {TAG, 'value'}:
        try:
            return {'nan': float('nan'), '+inf': float('inf'), '-inf': -float('inf')}[value['value']]
        except (KeyError, TypeError):
            raise ValueError('invalid tagged float64') from None
    if kind in ('decimal', 'datetime', 'date', 'time') and set(value) == {TAG, 'value'}:
        constructor = {'decimal': Decimal, 'datetime': datetime.fromisoformat,
                       'date': date.fromisoformat, 'time': time.fromisoformat}[kind]
        return constructor(value['value'])
    if kind == 'timedelta' and set(value) == {TAG, 'days', 'seconds', 'microseconds'}:
        if any(type(value[key]) is not int for key in ('days', 'seconds', 'microseconds')):
            raise ValueError('invalid tagged timedelta')
        return timedelta(days=value['days'], seconds=value['seconds'], microseconds=value['microseconds'])
    if kind == 'bytes' and set(value) == {TAG, 'value'}:
        return base64.b64decode(value['value'], validate=True)
    if kind == 'object' and set(value) == {TAG, 'items'}:
        result = {}
        for pair in value['items']:
            if not isinstance(pair, list) or len(pair) != 2 or not isinstance(pair[0], str):
                raise ValueError('invalid escaped JSON object')
            key, item = pair
            if key in result:
                raise ValueError(f'duplicate JSON field: {key}')
            result[key] = from_json_value(item)
        return result
    raise ValueError('invalid recording JSON tag')


def dumps(value, *, indent=None):
    return json.dumps(to_json_value(value), ensure_ascii=False, sort_keys=True,
                      allow_nan=False, indent=indent,
                      separators=(',', ':') if indent is None else None)


def loads(text):
    def pairs(items):
        result = {}
        for key, item in items:
            if key in result:
                raise ValueError(f'duplicate JSON field: {key}')
            result[key] = item
        return result

    def reject(token):
        raise ValueError(f'nonfinite JSON token must use a recording tag: {token}')

    def finite(token):
        result = float(token)
        if not math.isfinite(result):
            reject(token)
        return result

    return from_json_value(json.loads(text, object_pairs_hook=pairs,
                                     parse_constant=reject, parse_float=finite))


def thaw(value):
    """Mutable builtins for Arrow/API consumers, without JSON coercion."""
    if isinstance(value, Mapping):
        return {key: thaw(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [thaw(item) for item in value]
    if type(value).__module__.startswith('numpy'):
        return value.tolist() if hasattr(value, 'tolist') else value.item()
    return value
