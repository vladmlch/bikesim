"""Bitwise equality for the native-port oracle contract.

``np.array_equal`` uses IEEE equality: ``-0.0 == +0.0`` and NaN compares by
value-class, not payload. The port contract is bit-identical output, so the
assert compares raw bytes — ``tobytes()`` distinguishes signed zeros and NaN
payloads. Scalars and array-likes are normalized through ``np.asarray`` (a
Python float becomes a 0-d float64 array, still byte-comparable).
"""
import numpy as np


def assert_bitwise_equal(actual, expected, msg=''):
    a = np.asarray(actual)
    b = np.asarray(expected)
    assert a.dtype == b.dtype and a.shape == b.shape and \
        a.tobytes() == b.tobytes(), (
            f'{msg}: bitwise mismatch\nactual   {a!r}\nexpected {b!r}')
