"""Bitwise qualification of every platform-sensitive numerical lowering.

Each production lowering is compared against the *local* oracle — the
pinned NumPy 2.5.2 / CPython of the running interpreter — on raw
``uint64`` bit patterns, never a tolerance.  The corpus is deterministic:
a fixed RNG seed plus a checked-in adversarial fixture (signed zeros,
subnormals, tanh LUT interval boundaries, hypot scale edges, infinities,
quiet and signaling NaNs).

Surfaces under test (all debug-only bindings that invoke the same
production helpers, never a second implementation):

* ``_debug_pneumatic_math(xs, ys)``   — ``numeric::np_tanh`` /
  ``std::hypot`` against ``np.tanh`` / ``np.hypot``.
* ``_debug_numeric_contract(name, xs, ys)`` with names
  ``python_hypot`` (``numeric::python_vector_norm`` vs ``math.hypot`` —
  NOT ``np.hypot``), ``minimum``/``maximum``
  (``numeric::np_minimum``/``numeric::np_maximum`` vs the exact
  production ``np.minimum``/``np.maximum`` call shapes), ``sum``
  (``np.sum`` — the reduce identity folded over
  ``numeric::numpy_pairwise_sum`` order), ``sin``/``cos``
  (``numeric::np_sin``/``numeric::np_cos`` vs ``np.sin``/``np.cos`` —
  the ufunc oracle; ``math.*`` mirrors use ``math_sin``/``math_cos``
  and are not this surface).

``expected`` arrays are always produced by NumPy/CPython — never by the
native implementation.  NaN payload expectations are the locally
observed ones; NumPy promises no cross-dispatch NaN-payload stability,
so the corpus asserts local behavior only.

The production bindings accept contiguous float64 1-D arrays only.
Strided/reversed/F-order/offset/misaligned layouts are exercised on the
*oracle* side: the ufunc's own loop must produce the same per-element
bits our scalar port produces on the contiguous copy of the same values.
"""

import math

import numpy as np
import pytest

from _bits import assert_bitwise_equal
from native_loader import load_native

bike_native = load_native()

SEED = 0xB1E4C0DE

# --------------------------------------------------------------------
# Checked-in adversarial fixtures (uint64 bit patterns — NaN-safe)
# --------------------------------------------------------------------

# Signed zeros, subnormal/normal boundary, infinities, NaNs.
ADVERSARIAL_UINT64 = [
    0x0000000000000000,  # +0.0
    0x8000000000000000,  # -0.0
    0x0000000000000001,  # smallest +subnormal
    0x8000000000000001,  # smallest -subnormal
    0x000FFFFFFFFFFFFF,  # largest +subnormal
    0x800FFFFFFFFFFFFF,  # largest -subnormal
    0x0010000000000000,  # smallest +normal (DBL_MIN)
    0x8010000000000000,  # smallest -normal
    0x7FEFFFFFFFFFFFFF,  # largest +finite (DBL_MAX)
    0xFFEFFFFFFFFFFFFF,  # largest -finite
    0x7FF0000000000000,  # +inf
    0xFFF0000000000000,  # -inf
    0x7FF8000000000000,  # canonical qNaN
    0xFFF8000000000000,  # -qNaN
    0x7FF8000000000001,  # qNaN payload 1
    0x7FF0000000000001,  # sNaN payload 1
    0x7FF4000000000000,  # sNaN (quiet-bit-adjacent payload)
    0xFFF123456789ABCD,  # negative NaN payload
]

# tanh lut18x16 interval bases b (rows 0..14) and the |x|>=32 fake
# saturation boundary, each with its two adjacent representable doubles.
_TANH_BOUNDARY_WORDS = [
    0x3FC0000000000000, 0x3FCC000000000000, 0x3FD4000000000000,
    0x3FDC000000000000, 0x3FE4000000000000, 0x3FEC000000000000,
    0x3FF4000000000000, 0x3FFC000000000000, 0x4004000000000000,
    0x400C000000000000, 0x4014000000000000, 0x401C000000000000,
    0x4024000000000000, 0x402C000000000000, 0x4034000000000000,
    0x4040000000000000,  # 32.0 — first index clamped to row 15
]
TANH_BOUNDARY_UINT64 = [
    w
    for base in _TANH_BOUNDARY_WORDS
    for w in (base - 1, base, base + 1)
]

# hypot scale edges: overflow/underflow regions and powers of two the
# scaled algorithms special-case.
_HYPOT_WORDS = [
    0x7E37E43C8800759C,  # ~1e300
    0x01E3D8F5A5A00000,  # ~1e-300
    0x5FF0000000000000,  # 2**512
    0x2000000000000000,  # 2**-513 (subnormal-ish scale)
    0x4330000000000000,  # 2**52
    0x3FF0000000000000,  # 1.0
    0x3FE0000000000000,  # 0.5
]
# sin/cos edges: exact +/- multiples of pi (where range reduction must
# be exact), the float64 nearest-pi values, huge arguments, and tiny
# magnitudes that must return x verbatim.
_SINCOS_WORDS = [
    0x400921FB54442D18,  # +pi
    0xC00921FB54442D18,  # -pi
    0x400921FB54442D19,  # nextafter(pi, inf)
    0x400921FB54442D17,  # nextafter(pi, -inf)
    0x3FF921FB54442D18,  # pi/2
    0x401921FB54442D18,  # 2*pi
    0x3E45798EE2308C3A,  # 1e-8
    0x4340000000000000,  # 2**54 — beyond all easy reductions
    0x43D0000000000000,  # 2**62
    0x7FE0000000000000,  # 2**1023 — still finite
    0x3CD0000000000000,  # 2**-50 — sin(x)==x, cos(x)==1 edge
]

ADVERSARIAL_POOL_UINT64 = (
    ADVERSARIAL_UINT64 + TANH_BOUNDARY_UINT64 + _HYPOT_WORDS + _SINCOS_WORDS
)


def _bits(values) -> np.ndarray:
    return np.asarray(values, dtype=np.uint64).view(np.float64)


def corpus(length: int, *, seed: int = SEED) -> np.ndarray:
    """Deterministic adversarial pool resized to ``length``."""
    rng = np.random.default_rng(seed)
    raw = rng.integers(0, 2 ** 63, 64, dtype=np.uint64)
    pool = np.concatenate(
        [_bits(ADVERSARIAL_POOL_UINT64), raw.view(np.float64)])
    return np.resize(pool, length).copy()


# --------------------------------------------------------------------
# np.tanh / np.hypot — the existing pneumatic debug surface
# --------------------------------------------------------------------

@pytest.mark.parametrize("length", [1, 2, 3, 7, 8, 9, 31, 32, 33, 65])
def test_tanh_and_hypot_match_local_array_ufunc(length):
    pool = np.array([-0.0, 0.0, -0.125, 0.125, -1.0, 1.0,
                     -7.25, 7.25, -19.0, 19.0, 1e-300, -1e-300],
                    dtype=np.float64)
    xs = np.resize(np.concatenate([pool, corpus(48)]), length).copy()
    ys = np.resize(np.concatenate([pool[::-1], corpus(40, seed=SEED ^ 1)]),
                   length).copy()
    got = bike_native._debug_pneumatic_math(xs, ys)
    with np.errstate(all="ignore"):
        expected = {"tanh": np.tanh(xs), "hypot": np.hypot(xs, ys)}
    for name in expected:
        assert got[name].dtype == expected[name].dtype
        np.testing.assert_array_equal(got[name].view(np.uint64),
                                      expected[name].view(np.uint64))


def test_tanh_and_hypot_adversarial_fixture():
    """The checked-in corpus: every LUT boundary, subnormal, inf, NaN."""
    xs = corpus(256)
    ys = corpus(256, seed=SEED ^ 2)
    got = bike_native._debug_pneumatic_math(xs, ys)
    with np.errstate(all="ignore"):
        assert_bitwise_equal(np.asarray(got["tanh"]), np.tanh(xs),
                             "np.tanh adversarial")
        assert_bitwise_equal(np.asarray(got["hypot"]), np.hypot(xs, ys),
                             "np.hypot adversarial")


@pytest.mark.parametrize("stride", [2, 3, -1, -2])
def test_tanh_and_hypot_strided_oracle(stride):
    """Non-unit-stride ufunc loops must produce the same per-element
    bits the contiguous production API is checked against."""
    xs = corpus(192)[::stride]
    ys = corpus(192, seed=SEED ^ 3)[::stride]
    got = bike_native._debug_pneumatic_math(
        np.ascontiguousarray(xs), np.ascontiguousarray(ys))
    with np.errstate(all="ignore"):
        assert_bitwise_equal(np.asarray(got["tanh"]), np.tanh(xs),
                             f"np.tanh stride={stride}")
        assert_bitwise_equal(np.asarray(got["hypot"]), np.hypot(xs, ys),
                             f"np.hypot stride={stride}")


def test_tanh_and_hypot_layout_views():
    """F-order column view, offset view and byte-misaligned view all
    feed the oracle's strided/misaligned loops; bits stay per-element."""
    base = corpus(160)
    f_order = np.asfortranarray(base.reshape(16, 10))
    views = {
        "f_order_column": f_order[:, 3],
        "offset": np.concatenate([np.zeros(1), base])[1:],
        "misaligned": np.ndarray(
            160, dtype=np.float64,
            buffer=b"\x00" + base.tobytes(), offset=1),
    }
    for label, xs in views.items():
        ys = corpus(xs.size, seed=SEED ^ 4)
        got = bike_native._debug_pneumatic_math(
            np.ascontiguousarray(xs), ys)
        with np.errstate(all="ignore"):
            assert_bitwise_equal(np.asarray(got["tanh"]), np.tanh(xs),
                                 f"np.tanh {label}")
            assert_bitwise_equal(np.asarray(got["hypot"]),
                                 np.hypot(xs, ys),
                                 f"np.hypot {label}")


# --------------------------------------------------------------------
# _debug_numeric_contract — production helpers vs their exact oracles
# --------------------------------------------------------------------

def _contract(name, xs, ys):
    return np.asarray(
        bike_native._debug_numeric_contract(
            name, np.ascontiguousarray(xs, dtype=np.float64),
            np.ascontiguousarray(ys, dtype=np.float64)))


def test_python_hypot_matches_math_hypot():
    """CPython vector_norm port vs ``math.hypot`` — the production
    ``py_hypot``/contact-norm oracle — NEVER ``np.hypot``."""
    xs = corpus(256)
    ys = corpus(256, seed=SEED ^ 5)
    got = _contract("python_hypot", xs, ys)
    expected = np.array(
        [math.hypot(float(x), float(y)) for x, y in zip(xs, ys)],
        dtype=np.float64)
    assert_bitwise_equal(got, expected, "math.hypot pairwise")


def test_python_hypot_scale_and_nan_edges():
    """Overflow, subnormal-rescale, inf-beats-NaN and sNaN canonical
    qNaN branches of CPython's vector_norm."""
    pairs = np.array([
        [1e300, 1e300], [5e-324, 5e-324], [1e-300, -1e-300],
        [np.inf, np.nan], [np.nan, np.inf], [np.inf, 0.0],
        [0.0, 0.0], [-0.0, -0.0], [np.finfo(np.float64).max, 1.0],
        [np.finfo(np.float64).tiny, np.finfo(np.float64).tiny],
        [np.uint64(0x7FF0000000000001).view(np.float64), 1.0],
        [np.nan, 1.0], [1.0, np.nan],
    ])
    got = _contract("python_hypot", pairs[:, 0], pairs[:, 1])
    expected = np.array(
        [math.hypot(float(x), float(y)) for x, y in pairs],
        dtype=np.float64)
    assert_bitwise_equal(got, expected, "math.hypot edges")


@pytest.mark.parametrize("name", ["minimum", "maximum"])
def test_minmax_matches_elementwise_ufunc(name):
    """NEON fmax/fmin semantics: NaN passthrough (payload + quiet bit)
    and the signed-zero tie rule, elementwise."""
    oracle = getattr(np, name)
    xs = corpus(256)
    ys = corpus(256, seed=SEED ^ 6)
    got = _contract(name, xs, ys)
    expected = oracle(xs, ys)
    assert_bitwise_equal(got, expected, f"np.{name} elementwise")


@pytest.mark.parametrize("name", ["minimum", "maximum"])
def test_minmax_signed_zero_and_nan_fixture(name):
    oracle = getattr(np, name)
    pos, neg = 0.0, -0.0
    snan = np.uint64(0x7FF0000000000001).view(np.float64)
    qnan = np.uint64(0x7FF80000000000EE).view(np.float64)
    pairs = np.array([
        [pos, neg], [neg, pos], [neg, neg],
        [snan, 1.0], [1.0, snan], [qnan, snan], [snan, qnan],
        [qnan, 1.0], [1.0, qnan],
    ])
    got = _contract(name, pairs[:, 0], pairs[:, 1])
    expected = oracle(pairs[:, 0], pairs[:, 1])
    assert_bitwise_equal(got, expected, f"np.{name} zero/NaN ties")


@pytest.mark.parametrize("name", ["minimum", "maximum"])
def test_minmax_scalar_broadcast_shape(name):
    """The production call shape is array-vs-scalar-0.0; the broadcast
    ufunc path must lower to the same per-element instruction."""
    oracle = getattr(np, name)
    xs = corpus(192)
    got = _contract(name, xs, np.zeros(xs.size))
    expected = oracle(xs, 0.0)
    assert_bitwise_equal(got, expected, f"np.{name}(a, 0.0) broadcast")


SUM_LENGTHS = [0, 1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33,
               63, 64, 65, 127, 128, 129, 130, 131, 135, 136,
               255, 256, 257, 511, 512, 513]


def _sum_vectors(length: int) -> list[np.ndarray]:
    rng = np.random.default_rng(SEED + length)
    vectors = [
        np.zeros(length),
        np.full(length, -0.0),
        corpus(length),
        rng.uniform(-1.0, 1.0, length),
        rng.integers(0, 2 ** 63, length, dtype=np.uint64).view(np.float64),
    ]
    if length >= 3:
        cancel = np.resize(
            np.array([1e300, 1.0, -1e300, 2.0 ** 53, 1.0, -2.0 ** 53]),
            length).copy()
        pow2 = 2.0 ** (np.arange(length) % 60) + 2.0 ** -53
        vectors += [cancel, pow2]
    return vectors


@pytest.mark.parametrize("length", SUM_LENGTHS)
def test_sum_matches_np_sum(length):
    """np.sum: the add-reduce identity 0.0 seeds the pairwise fold —
    an all-(-0.0) input must come back +0.0, not pairwise's -0.0.
    Lengths bracket the n<8 sequential, n<=128 eight-way and
    halving-recursion boundaries."""
    for vector in _sum_vectors(length):
        got = _contract("sum", vector, vector[:0])
        with np.errstate(all="ignore"):
            expected = np.asarray(np.sum(vector), dtype=np.float64)
        assert got.shape == (1,), f"sum len={length} shape {got.shape}"
        assert_bitwise_equal(got, expected.reshape(1),
                             f"np.sum len={length}")


def test_sum_cancellation_fixture():
    """Pow2+eps and signed cancellation around the unroll boundary."""
    vectors = [
        np.array([1e300, 1.0, -1e300]),
        np.concatenate([[2.0 ** 53, 1.0, -2.0 ** 53], np.zeros(125)]),
        np.resize(np.array([1.0, -1.0, 1e-300, -1e-300]), 128),
        np.resize(np.array([1.0, -1.0, 1e-300, -1e-300]), 129),
    ]
    for index, vector in enumerate(vectors):
        got = _contract("sum", vector, vector[:0])
        with np.errstate(all="ignore"):
            expected = np.asarray(np.sum(vector), dtype=np.float64)
        assert_bitwise_equal(got, expected.reshape(1),
                             f"np.sum cancellation {index}")


@pytest.mark.parametrize("name", ["sin", "cos"])
def test_sin_cos_match_local_ufunc(name):
    """DOUBLE_sin/cos lower to npy_sin/npy_cos (the f64 SIMD path is
    disabled upstream): the in-process libm plus, on Apple arm64, the
    upstream `x - x` non-finite workaround that keeps a negative NaN's
    sign — which is exactly what `numeric::np_sin`/`np_cos` mirror.
    `math.sin`/`math.cos` would NOT match here (libm canonicalizes the
    sign)."""
    oracle = getattr(np, name)
    xs = corpus(256)
    got = _contract(name, xs, xs[:0])
    with np.errstate(all="ignore"):
        expected = oracle(xs)
    assert_bitwise_equal(got, expected, f"np.{name} corpus")


@pytest.mark.parametrize("name", ["sin", "cos"])
def test_sin_cos_edge_fixture(name):
    oracle = getattr(np, name)
    xs = _bits(ADVERSARIAL_UINT64 + _SINCOS_WORDS)
    got = _contract(name, xs, xs[:0])
    with np.errstate(all="ignore"):
        expected = oracle(xs)
    assert_bitwise_equal(got, expected, f"np.{name} edges")


def test_contract_rejects_unknown_name():
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_numeric_contract(
            "definitely_not_an_op", np.zeros(1), np.zeros(1))
