"""Bitwise oracle for the typed platform CBLAS adapters (T07).

``bike_native._debug_platform_blas(a, b, operation)`` must invoke the
same ``blas::ddot``/``blas::dgemv``/``blas::dgemm`` wrappers the
production callers use — never a second implementation — and return
numpy-``@`` bit-identical, owning results. 2-D operands may arrive C- or
F-contiguous (an F operand reaches the row-major Trans entry point, the
same call shape the writers' ``A.T @ x`` uses). Overlapping, strided or
non-contiguous layouts are rejected explicitly rather than re-interpreted.
"""
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal

DIMS = (0, 1, 2, 3, 7, 16, 17, 64, 127, 128, 129)


def test_blas_rectangular_matmul_matches_selected_numpy():
    rng = np.random.default_rng(6021)
    a = rng.normal(size=(3, 17))
    b = rng.normal(size=(17, 5))
    actual = bike_native._debug_platform_blas(a, b, "matmul")
    assert_bitwise_equal(actual, a @ b)


@pytest.mark.parametrize("n", DIMS)
def test_blas_dot_matches_numpy(n):
    rng = np.random.default_rng(9000 + n)
    a = rng.normal(size=n)
    b = rng.normal(size=n)
    actual = bike_native._debug_platform_blas(a, b, "dot")
    assert_bitwise_equal(actual, a @ b)


@pytest.mark.parametrize("m,n", [(0, 0), (0, 3), (1, 1), (2, 3), (3, 2),
                                 (3, 17), (16, 16), (17, 129), (64, 127),
                                 (128, 64), (129, 1)])
@pytest.mark.parametrize("layout", ["C", "F"])
def test_blas_matvec_matches_numpy(m, n, layout):
    rng = np.random.default_rng(1000 + 131 * m + n)
    a = rng.normal(size=(m, n))
    x = rng.normal(size=n)
    if layout == "F":
        a = np.asfortranarray(a)
    actual = bike_native._debug_platform_blas(a, x, "matvec")
    assert_bitwise_equal(actual, a @ x)


@pytest.mark.parametrize("m,k,n", [(0, 0, 0), (0, 3, 2), (2, 0, 3),
                                   (1, 1, 1), (2, 3, 4), (3, 2, 2),
                                   (3, 17, 5), (16, 17, 64), (127, 1, 2),
                                   (129, 64, 7)])
@pytest.mark.parametrize("layout_a,layout_b",
                         [("C", "C"), ("C", "F"), ("F", "C"), ("F", "F")])
def test_blas_matmul_matches_numpy(m, k, n,
                                   layout_a, layout_b):
    rng = np.random.default_rng(7000 + 131 * m + 17 * k + n)
    a = rng.normal(size=(m, k))
    b = rng.normal(size=(k, n))
    if layout_a == "F":
        a = np.asfortranarray(a)
    if layout_b == "F":
        b = np.asfortranarray(b)
    actual = bike_native._debug_platform_blas(a, b, "matmul")
    assert_bitwise_equal(actual, a @ b)


@pytest.mark.parametrize("layout_a", ["C", "F"])
def test_blas_matvec_transposed_view_matches_numpy(layout_a):
    """``a.T @ x`` — the F-view dgemv call shape the writers emit, plus
    the same product over a materialized C-order transpose."""
    rng = np.random.default_rng(4451)
    a = rng.normal(size=(17, 3))
    x = rng.normal(size=17)
    # a.T of a C array is F-contiguous; ascontiguousarray materializes the
    # same logical matrix in C order — two different adapter paths. The
    # oracle is ``transposed @ x`` (the provider call NumPy itself makes for
    # that stored layout), not ``a.T @ x``: an F-order operand takes the
    # column-major entry point whose reduction order differs from the
    # row-major NoTrans kernel, so cross-layout bitwise equality is not a
    # provider contract.
    transposed = a.T if layout_a == "F" else np.ascontiguousarray(a.T)
    actual = bike_native._debug_platform_blas(transposed, x, "matvec")
    assert_bitwise_equal(actual, transposed @ x)


def test_blas_matmul_transposed_view_matches_numpy():
    """``local @ R.T`` — the row-major NT dgemm call shape the writers emit."""
    rng = np.random.default_rng(8080)
    local = rng.normal(size=(4, 3))
    r = rng.normal(size=(3, 3))
    actual = bike_native._debug_platform_blas(local, r.T, "matmul")
    assert_bitwise_equal(actual, local @ r.T)


def test_blas_dot_zero_size():
    actual = bike_native._debug_platform_blas(
        np.empty(0), np.empty(0), "dot")
    assert_bitwise_equal(actual, np.empty(0) @ np.empty(0))


def test_blas_rejects_unknown_operation():
    with pytest.raises(Exception, match="operation|unsupported|unknown"):
        bike_native._debug_platform_blas(
            np.ones(2), np.ones(2), "ger")


def test_blas_rejects_shape_mismatch():
    with pytest.raises(Exception):
        bike_native._debug_platform_blas(
            np.ones(3), np.ones(4), "dot")
    with pytest.raises(Exception):
        bike_native._debug_platform_blas(
            np.ones((2, 3)), np.ones(4), "matvec")
    with pytest.raises(Exception):
        bike_native._debug_platform_blas(
            np.ones((2, 3)), np.ones((4, 2)), "matmul")


def test_blas_rejects_noncontiguous_operands():
    """Strided/overlapping storage must be rejected, never re-interpreted
    through invented strides."""
    a = np.arange(8.0)[::2]                 # stride 2, not contiguous
    b = np.ones(4)
    with pytest.raises(Exception, match="contig"):
        bike_native._debug_platform_blas(a, b, "dot")
    m = np.arange(24.0).reshape(4, 6)[:, ::2]
    with pytest.raises(Exception, match="contig"):
        bike_native._debug_platform_blas(m, np.ones(3), "matvec")
    with pytest.raises(Exception, match="contig"):
        bike_native._debug_platform_blas(m, np.ones((3, 2)), "matmul")
    broadcast = np.broadcast_to(np.ones(3), (3, 3))  # stride-0 overlap
    with pytest.raises(Exception, match="contig"):
        bike_native._debug_platform_blas(
            broadcast, np.ones((3, 2)), "matmul")


def test_blas_rejects_wrong_rank_and_dtype():
    with pytest.raises(Exception):
        bike_native._debug_platform_blas(
            np.ones((2, 2, 2)), np.ones((2, 2)), "matmul")
    with pytest.raises(Exception):
        bike_native._debug_platform_blas(
            np.ones(2, dtype=np.float32), np.ones(2), "dot")
    with pytest.raises(Exception):
        bike_native._debug_platform_blas([1.0, 2.0], np.ones(2), "dot")


@pytest.mark.parametrize("order", ["numpy-first", "native-first"])
def test_blas_debug_binding_import_order(order):
    """The adapter must not depend on which image maps the provider
    first — exercise both import orders in fresh subprocesses."""
    build = Path(bike_native.__file__).resolve().parent
    imports = ("import numpy; import bike_native" if order == "numpy-first"
               else "import bike_native; import numpy")
    code = (
        "import sys; sys.path.insert(0, sys.argv[1]); " + imports + "; "
        "import numpy as np; "
        "got = bike_native._debug_platform_blas("
        "np.array([1., 2.]), np.array([3., 4.]), 'dot'); "
        "assert got == 11.0, got; "
        "print('import-order-ok')"
    )
    result = subprocess.run(
        [sys.executable, "-c", code, str(build)],
        capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "import-order-ok" in result.stdout
