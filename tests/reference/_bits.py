"""Bitwise equality for the native-port oracle contract.

``np.array_equal`` uses IEEE equality: ``-0.0 == +0.0`` and NaN compares by
value-class, not payload. The port contract is bit-identical output, so the
assert compares raw bytes — ``tobytes()`` distinguishes signed zeros and NaN
payloads. Scalars and array-likes are normalized through ``np.asarray`` (a
Python float becomes a 0-d float64 array, still byte-comparable).
"""
import numpy as np
import _golden


def assert_bitwise_equal(actual, expected, msg=''):
    """Compare unchanged dtype, shape and raw bits; optionally freeze expected.

    A passing comparison has equal sides regardless of argument order. A
    failing comparison may leave capture output, which must never be committed.
    """
    a = np.asarray(actual)
    b = np.asarray(expected)
    _golden.record(b, msg)
    assert a.dtype == b.dtype and a.shape == b.shape and \
        a.tobytes() == b.tobytes(), (
            f'{msg}: bitwise mismatch\nactual   {a!r}\nexpected {b!r}')
