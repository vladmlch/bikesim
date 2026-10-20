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
  production ``np.minimum``/``np.maximum`` call shapes),
  ``multiply_broadcast`` (``numeric::np_multiply_broadcast`` vs the
  wheel's scalar-broadcast ``np.multiply`` — sNaN-either-side and
  same-class operand-0 NaN precedence), ``sum``
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


def _multiply_broadcast_oracle(pairs) -> np.ndarray:
    """np.multiply(float scalar, ndarray element) per pair — run the
    real 64-lane broadcast kernel per lane so the oracle exposes the
    wheel's fmul operand order (scalar-first on the AppleClang wheel,
    element-first on manylinux), never a Python-level product."""
    return np.array(
        [np.multiply(float(x), np.full(64, y))[0] for x, y in pairs],
        dtype=np.float64)


def test_multiply_broadcast_matches_scalar_ufunc():
    """corpus lanes through numeric::np_multiply_broadcast vs the
    installed wheel's scalar-broadcast multiply."""
    xs = corpus(256)
    ys = corpus(256, seed=SEED ^ 6)
    pairs = np.stack([xs, ys], axis=1)
    got = _contract("multiply_broadcast", xs, ys)
    expected = _multiply_broadcast_oracle(pairs)
    assert_bitwise_equal(got, expected, "np.multiply(scalar, arr)")


def test_multiply_broadcast_nan_class_fixture():
    """Every NaN-class combination in both operand positions: an sNaN
    quiets and wins either side; a same-class tie resolves to the
    wheel's measured operand 0."""
    snan = np.uint64(0x7FF0000000000001).view(np.float64)
    snan2 = np.uint64(0xFFF4000000000002).view(np.float64)
    qnan = np.uint64(0x7FF8000000000000).view(np.float64)
    qnan_neg = np.uint64(0xFFF8000000000000).view(np.float64)
    pairs = np.array([
        [qnan, qnan_neg], [qnan_neg, qnan],      # qNaN vs qNaN
        [snan, qnan], [qnan, snan],              # sNaN vs qNaN
        [snan, snan2], [snan2, snan],            # sNaN vs sNaN
        [qnan, 1.0], [1.0, qnan],                # single NaN
        [snan, 1.0], [1.0, snan],
        [0.0, np.inf], [np.inf, 0.0],            # fresh invalid NaN
        [-0.0, 1.0], [2.0, -3.0],
    ])
    got = _contract("multiply_broadcast",
                    pairs[:, 0].copy(), pairs[:, 1].copy())
    expected = _multiply_broadcast_oracle(pairs)
    assert_bitwise_equal(got, expected, "multiply broadcast NaN classes")


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


# --------------------------------------------------------------------
# _debug_interp — biketyre::np_interp vs np.interp (general table)
# --------------------------------------------------------------------
# The oracle is the literal np.interp call: strictly increasing float64
# knot tables, default left/right clamping, exact-knot hits, interior
# queries (the slope*(x-xp)+fp lowering), the NaN retry and the
# constant-segment fallback — including non-finite fp lanes.

def _interp_tables() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    rng = np.random.default_rng(SEED ^ 0x5EED)
    tables = {}
    for n in (2, 3, 5, 9, 17):
        xs = np.cumsum(np.abs(rng.standard_normal(n)) + 0.25)
        tables[f"n{n}-random"] = (xs, rng.standard_normal(n))
    xs4 = np.asarray([0.25, 0.5, 0.75, 1.0])
    tables["n4-constant"] = (xs4, np.full(4, 3.5))
    # inf/-inf fp lanes push slope*(x-xp)+fp through the NaN retry.
    tables["n4-inf-y"] = (
        xs4, np.asarray([np.inf, -np.inf, 1.0, -1.0]))
    tables["n3-nan-y"] = (
        np.asarray([0.25, 0.5, 0.75]),
        np.asarray([np.nan, -1.0, 1.0]))
    return tables


_INTERP_TABLES = _interp_tables()


def _interp_queries(xs: np.ndarray) -> np.ndarray:
    rng = np.random.default_rng(SEED ^ 0x1E7E)
    span = xs[-1] - xs[0]
    interior = rng.uniform(xs[0], xs[-1], 96)
    edges = np.asarray([xs[0], xs[-1],
                        xs[0] - 0.125 * span,
                        xs[-1] + 0.125 * span,
                        np.inf, -np.inf])
    return np.concatenate([interior, xs, edges])


@pytest.mark.parametrize("label", sorted(_INTERP_TABLES))
def test_interp_matches_np_interp_general_table(label):
    xs, ys = _INTERP_TABLES[label]
    queries = _interp_queries(xs)
    got = np.asarray(
        [bike_native._debug_interp(float(q), xs, ys) for q in queries])
    with np.errstate(all="ignore"):
        expected = np.interp(queries, xs, ys)
    assert_bitwise_equal(got, expected, f"np.interp {label}")


def test_interp_rejects_mismatched_or_empty_tables():
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_interp(0.5, np.zeros(0), np.zeros(0))
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_interp(0.5, np.ones(3), np.ones(2))


# --------------------------------------------------------------------
# _debug_rider_linalg — spindle-controller mirrors vs numpy
# --------------------------------------------------------------------
# Every operation drives the SAME production helper the controller
# calls (rider/spindle_linalg.hpp) on tightly C-order float64 operands;
# the oracle is the literal NumPy expression the port mirrors
# (a @ v / a.T @ v / a @ b / a.T @ b / np.linalg.norm).

_RIDER_OPS_3D = {
    "matvec": lambda a, b: a @ b,
    "mat_t_vec": lambda a, b: a.T @ b,
    "mat_t_vec_xz": lambda a, b: (a.T @ b)[[0, 2]],
    "matmul": lambda a, b: a @ b,
    "mat_t_mul": lambda a, b: a.T @ b,
}
_RIDER_OPS_2D = {
    "matvec2": lambda a, b: a @ b,
    "mat_t_vec2": lambda a, b: a.T @ b,
}


def _rider_operands(rng, n: int, second_is_matrix: bool,
                    rounds: int) -> list:
    cases = []
    for _ in range(rounds):
        cases.append((rng.standard_normal((n, n)),
                      rng.standard_normal((n, n) if second_is_matrix
                                          else n)))
    # The checked-in adversarial pool supplies inf/NaN/subnormal lanes.
    cases.append((np.resize(_bits(ADVERSARIAL_POOL_UINT64), n * n)
                  .reshape(n, n).copy(),
                  np.resize(_bits(ADVERSARIAL_POOL_UINT64)[::-1],
                            n * n if second_is_matrix else n)
                  .reshape((n, n) if second_is_matrix else n).copy()))
    return cases


@pytest.mark.parametrize("op_name", sorted(_RIDER_OPS_3D))
def test_rider_linalg_3d_matches_numpy(op_name):
    oracle = _RIDER_OPS_3D[op_name]
    rng = np.random.default_rng(SEED ^ 0x2A3D)
    is_matmul = op_name in ("matmul", "mat_t_mul")
    for a, b in _rider_operands(rng, 3, is_matmul, 64):
        got = np.asarray(bike_native._debug_rider_linalg(op_name, a, b))
        with np.errstate(all="ignore"):
            expected = oracle(a, b)
        assert_bitwise_equal(got, np.asarray(expected),
                             f"rider {op_name} 3-D")


@pytest.mark.parametrize("op_name", sorted(_RIDER_OPS_2D))
def test_rider_linalg_2d_matches_numpy(op_name):
    oracle = _RIDER_OPS_2D[op_name]
    rng = np.random.default_rng(SEED ^ 0x2A3E)
    for a, b in _rider_operands(rng, 2, False, 64):
        got = np.asarray(bike_native._debug_rider_linalg(op_name, a, b))
        with np.errstate(all="ignore"):
            expected = oracle(a, b)
        assert_bitwise_equal(got, np.asarray(expected),
                             f"rider {op_name} 2-D")


@pytest.mark.parametrize("length", [1, 2, 3, 5, 8])
def test_rider_linalg_norm_matches_np_linalg_norm(length):
    """np.linalg.norm on a 1-D float64 vector — the sqrt(v.dot(v))
    fold, not math.hypot's scaled algorithm."""
    rng = np.random.default_rng(SEED ^ 0xA11F)
    vectors = [rng.standard_normal(length) for _ in range(48)]
    vectors.append(np.resize(_bits(ADVERSARIAL_POOL_UINT64), length))
    for v in vectors:
        v = np.ascontiguousarray(v, dtype=np.float64)
        got = np.float64(
            bike_native._debug_rider_linalg(
                "norm", v, np.zeros(0)))
        with np.errstate(all="ignore"):
            expected = np.float64(np.linalg.norm(v))
        assert_bitwise_equal(np.asarray(got), np.asarray(expected),
                             f"np.linalg.norm len={length}")


def test_rider_linalg_rejects_bad_inputs():
    good3 = np.eye(3)
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "bogus_op", good3, np.zeros(3))
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "matvec", np.eye(2), np.zeros(3))
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "matvec", good3, np.zeros(4))
    # F-order and strided layouts are rejected — the production helpers
    # only ever see packed C-order storage.
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "matvec", np.asfortranarray(good3), np.zeros(3))
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "matvec", good3, np.zeros(6)[::2])
    with pytest.raises((ValueError, RuntimeError)):
        bike_native._debug_rider_linalg(
            "norm", np.zeros((2, 2)), np.zeros(0))
