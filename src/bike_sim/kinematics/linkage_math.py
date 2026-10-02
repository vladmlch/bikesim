"""
Mathematical primitives for 2D linkage kinematics and root-finding.

Provides closed-form circle-circle intersection in the (X, Z) sagittal plane
and a pure Python/NumPy implementation of Brent's 1D root-finding method.
"""

from typing import Callable, Tuple
import numpy as np


def brentq_pure(
    f: Callable[[float], float],
    a: float,
    b: float,
    xtol: float = 1e-12,
    rtol: float = 1e-12,
    maxiter: int = 100,
) -> float:
    """
    Pure Python / NumPy implementation of Brent's method for 1D root-finding.

    Guarantees machine-precision root finding without external C-extensions.

    Args:
        f: Objective 1D scalar function.
        a: Lower bracket bound.
        b: Upper bracket bound.
        xtol: Absolute tolerance for convergence.
        rtol: Relative tolerance for convergence.
        maxiter: Maximum number of iterations.

    Returns:
        Root location x where f(x) == 0 within tolerance.
    """
    fa = f(a)
    fb = f(b)
    if fa * fb > 0:
        raise ValueError(f"Root is not bracketed: f({a})={fa:g}, f({b})={fb:g}")
    if fa == 0.0:
        return a
    if fb == 0.0:
        return b

    c = a
    fc = fa
    d = e = b - a

    for _ in range(maxiter):
        if (fb > 0 and fc > 0) or (fb < 0 and fc < 0):
            c = a
            fc = fa
            d = e = b - a
        if abs(fc) < abs(fb):
            a, b, c = b, c, b
            fa, fb, fc = fb, fc, fb

        tol = 2.0 * 1e-16 * abs(b) + 0.5 * xtol
        m = 0.5 * (c - b)

        if abs(m) <= tol or fb == 0.0:
            return b

        if abs(e) >= tol and abs(fa) > abs(fb):
            s = fb / fa
            if a == c:
                # Linear interpolation (Secant method)
                p = 2.0 * m * s
                q = 1.0 - s
            else:
                # Inverse quadratic interpolation
                q = fa / fc
                r = fb / fc
                p = s * (2.0 * m * q * (q - r) - (b - a) * (r - 1.0))
                q = (q - 1.0) * (r - 1.0) * (s - 1.0)
            if p > 0:
                q = -q
            else:
                p = -p
            if 2.0 * p < min(3.0 * m * q - abs(tol * q), abs(e * q)):
                e = d
                d = p / q
            else:
                d = e = m
        else:
            d = e = m

        a = b
        fa = fb
        if abs(d) > tol:
            b += d
        elif m > 0:
            b += tol
        else:
            b -= tol
        fb = f(b)

    return b


def circle_circle_intersection_2d(
    c1: np.ndarray,
    r1: float,
    c2: np.ndarray,
    r2: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes analytical 2D circle-circle intersection points in the (X, Z) plane.

    Args:
        c1: Center of Circle 1 [X, Z] or [X, Y, Z].
        r1: Radius of Circle 1.
        c2: Center of Circle 2 [X, Z] or [X, Y, Z].
        r2: Radius of Circle 2.

    Returns:
        Tuple of two intersection points (sol_branch_1, sol_branch_2) in 2D [X, Z].
    """
    p1 = np.array([c1[0], c1[2] if len(c1) == 3 else c1[1]], dtype=float)
    p2 = np.array([c2[0], c2[2] if len(c2) == 3 else c2[1]], dtype=float)

    d_vec = p2 - p1
    d = float(np.linalg.norm(d_vec))

    if d < 1e-12:
        raise ValueError("Circles are concentric; infinite or zero intersections.")
    if d > (r1 + r2) + 1e-9 or d < abs(r1 - r2) - 1e-9:
        raise ValueError(
            f"Circles do not intersect: distance {d:.4f} outside [{abs(r1-r2):.4f}, {r1+r2:.4f}]"
        )

    # Clamp d for numerical stability if tangent
    d_clamped = np.clip(d, abs(r1 - r2), r1 + r2)
    a = (r1**2 - r2**2 + d_clamped**2) / (2.0 * d_clamped)
    h_sq = r1**2 - a**2
    h = np.sqrt(max(0.0, h_sq))

    u = d_vec / d
    n = np.array([-u[1], u[0]], dtype=float)  # 90-degree CCW normal

    p_mid = p1 + a * u
    sol1 = p_mid + h * n
    sol2 = p_mid - h * n

    return sol1, sol2
