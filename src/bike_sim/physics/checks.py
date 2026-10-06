"""Small shared validation helpers for the physical SI components."""
from math import isfinite
from numbers import Integral, Real
from operator import index
from collections.abc import Mapping, Sequence
import numpy as np


def integer(value, name: str, *, minimum=None, positive=False) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(f'{name}: expected integer')
    result = index(value)
    if (minimum is not None and result < minimum) or (positive and result <= 0):
        raise ValueError(f'{name}: invalid integer domain')
    return result


def boolean(value, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f'{name}: expected bool')
    return value


def derived(value, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise OverflowError(f'{name}: non-finite derived value')
    return result


def derived_array(value, name: str):
    if not np.isfinite(value).all():
        raise OverflowError(f'{name}: non-finite derived array')
    return value


def scalar(value, name: str, *, minimum=None, positive=False) -> float:
    # Fast path: plain and numpy floats make up nearly every call in the
    # per-step force evaluations; they can skip the abstract Real dispatch.
    if type(value) is float or type(value) is int or isinstance(value, np.floating):
        try:
            value = float(value)
        except OverflowError as exc:
            raise ValueError(f'{name}: real scalar is out of range') from exc
        if not isfinite(value) or (minimum is not None and value < minimum) or (positive and value <= 0):
            raise ValueError(f'invalid {name}')
        return value
    if isinstance(value,(bool,np.bool_)) or not isinstance(value,Real):
        raise ValueError(f'{name} must be a finite real scalar')
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f'{name}: real scalar is out of range') from exc
    if not isfinite(value) or (minimum is not None and value < minimum) or (positive and value <= 0):
        raise ValueError(f'invalid {name}')
    return value


def sequence(value, name: str) -> tuple:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, (Sequence, np.ndarray)):
        raise ValueError(f'{name}: expected ordered sequence')
    if isinstance(value, np.ndarray) and value.ndim == 0:
        raise ValueError(f'{name}: expected ordered sequence')
    return tuple(value)


def array(value, name: str, shape=None, *, readonly=False) -> np.ndarray:
    def original_types(item):
        if type(item) in (float, int) or isinstance(item, (np.floating, np.integer)):
            return
        if isinstance(item, Mapping):
            raise ValueError(f'{name}: expected numeric sequence, not mapping')
        if isinstance(item, np.ndarray) and item.dtype.kind in 'iuf':
            return  # numeric dtype proves the original element type without a Python loop
        if isinstance(item, np.ndarray) and item.dtype.kind == 'b':
            raise ValueError(f'{name}: booleans are not numeric inputs')
        if isinstance(item, np.ndarray) and item.ndim == 0:
            scalar(item[()], name)
        elif isinstance(item, (Sequence, np.ndarray)) and not isinstance(item, (str, bytes)):
            for child in item:
                original_types(child)
        else:
            if isinstance(item, (bool, np.bool_)) or not isinstance(item, Real):
                raise ValueError(f'{name}: expected real numeric elements')
    original_types(value)
    try:
        result = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'invalid {name}') from exc
    if (shape is not None and result.shape != shape) or not np.isfinite(result).all():
        raise ValueError(f'invalid shape or non-finite {name}')
    if readonly:
        # An immutable backing buffer also prevents callers re-enabling write access.
        result = np.frombuffer(result.tobytes(), dtype=float).reshape(result.shape)
    return result
