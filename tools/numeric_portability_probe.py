"""Bitwise numerical-portability probe for the native extension.

Loads exactly one ``bike_native`` artifact through the application loader
(``bike_sim.native.artifact.load_native_extension`` — never an implicit
build), runs the debug surfaces that invoke the production numerical
helpers, and compares raw ``uint64`` bit patterns against the *local*
oracles — the pinned NumPy 2.5.2 ufuncs and CPython ``math.hypot`` of the
interpreter running this probe.  No tolerance anywhere.

CLI:
    uv run --frozen --group native python tools/numeric_portability_probe.py \
        --build ABSOLUTE_BUILD --output PATH

The corpus is deterministic: a fixed RNG seed plus the checked-in
adversarial fixtures (signed zeros, subnormals, tanh LUT boundaries,
hypot scale edges, infinities, quiet/signaling NaNs).  Inputs are stored
as ``uint64`` lists so NaN payloads and signed zeros survive JSON.

Exit status is nonzero when any comparison differs bitwise, when a
required debug surface is absent, or when the artifact/environment fails
to load — fail closed, never a silent pass.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import platform
import sys
import sysconfig
from pathlib import Path

SCHEMA_VERSION = 1
SEED = 0xB1E4C0DE

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _entry in (str(_REPO_ROOT / "src"), str(_REPO_ROOT)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)


# --------------------------------------------------------------------
# Comparison core — preserves bit patterns verbatim (schema contract)
# --------------------------------------------------------------------

def compare_arrays(expected, actual):
    import numpy as np

    expected = np.ascontiguousarray(expected, dtype=np.float64)
    actual = np.ascontiguousarray(actual, dtype=np.float64)
    if actual.shape != expected.shape:
        return {"classification": "shape",
                "expected_shape": list(expected.shape),
                "actual_shape": list(actual.shape)}
    eb = expected.view(np.uint64).ravel()
    ab = actual.view(np.uint64).ravel()
    bad = np.flatnonzero(eb != ab)
    return {"classification": "equal" if not bad.size else "bits",
            "expected_uint64": [int(v) for v in eb],
            "actual_uint64": [int(v) for v in ab],
            "first_mismatch": None if not bad.size else int(bad[0])}


# --------------------------------------------------------------------
# Deterministic corpus (mirrors the checked-in pytest fixtures)
# --------------------------------------------------------------------

ADVERSARIAL_UINT64 = [
    0x0000000000000000, 0x8000000000000000,
    0x0000000000000001, 0x8000000000000001,
    0x000FFFFFFFFFFFFF, 0x800FFFFFFFFFFFFF,
    0x0010000000000000, 0x8010000000000000,
    0x7FEFFFFFFFFFFFFF, 0xFFEFFFFFFFFFFFFF,
    0x7FF0000000000000, 0xFFF0000000000000,
    0x7FF8000000000000, 0xFFF8000000000000,
    0x7FF8000000000001, 0x7FF0000000000001,
    0x7FF4000000000000, 0xFFF123456789ABCD,
]

_TANH_BOUNDARY_WORDS = [
    0x3FC0000000000000, 0x3FCC000000000000, 0x3FD4000000000000,
    0x3FDC000000000000, 0x3FE4000000000000, 0x3FEC000000000000,
    0x3FF4000000000000, 0x3FFC000000000000, 0x4004000000000000,
    0x400C000000000000, 0x4014000000000000, 0x401C000000000000,
    0x4024000000000000, 0x402C000000000000, 0x4034000000000000,
    0x4040000000000000,
]
TANH_BOUNDARY_UINT64 = [
    w for base in _TANH_BOUNDARY_WORDS for w in (base - 1, base, base + 1)
]

_HYPOT_WORDS = [
    0x7E37E43C8800759C, 0x01E3D8F5A5A00000, 0x5FF0000000000000,
    0x2000000000000000, 0x4330000000000000, 0x3FF0000000000000,
    0x3FE0000000000000,
]

_SINCOS_WORDS = [
    0x400921FB54442D18, 0xC00921FB54442D18,
    0x400921FB54442D19, 0x400921FB54442D17,
    0x3FF921FB54442D18, 0x401921FB54442D18,
    0x3E45798EE2308C3A, 0x4340000000000000,
    0x43D0000000000000, 0x7FE0000000000000,
    0x3CD0000000000000,
]

ADVERSARIAL_POOL_UINT64 = (
    ADVERSARIAL_UINT64 + TANH_BOUNDARY_UINT64 + _HYPOT_WORDS + _SINCOS_WORDS
)


def _corpus(np, length: int, seed: int = SEED):
    rng = np.random.default_rng(seed)
    raw = rng.integers(0, 2 ** 63, 64, dtype=np.uint64)
    pool = np.concatenate(
        [np.asarray(ADVERSARIAL_POOL_UINT64, dtype=np.uint64).view(
            np.float64), raw.view(np.float64)])
    return np.resize(pool, length).copy()


def _sum_cases(np):
    """np.sum probes: zero/one elements, cancellation, pow2+eps, and
    lengths bracketing the n<8 / n<=128 / halving-recursion folds."""
    cases = []
    for length in (0, 1, 2, 3, 7, 8, 9, 31, 32, 33, 127, 128, 129, 130,
                   255, 256, 257):
        cases.append((f"len{length}-corpus", _corpus(np, length)))
        cases.append((f"len{length}-negzero", np.full(length, -0.0)))
        if length >= 3:
            cases.append((
                f"len{length}-cancel",
                np.resize(np.array([1e300, 1.0, -1e300, 2.0 ** 53,
                                    1.0, -2.0 ** 53]), length).copy()))
    return cases


def _interp_cases(np):
    """(label, xs, ys, queries) — general-table np.interp probes.

    Strictly increasing float64 knot tables at production sizes plus
    constant/non-finite fp lanes; queries cover the interior
    slope*(x-xp)+fp lowering, exact knots, out-of-range endpoints and
    the +-inf clamps. NaN queries are out of contract (np.interp returns
    early; the port documents the same boundary upstream), so they are
    not part of the corpus.
    """
    rng = np.random.default_rng(SEED ^ 0x5EED)
    cases = []
    for n in (2, 3, 5, 9, 17):
        xs = np.cumsum(np.abs(rng.standard_normal(n)) + 0.25)
        ys = rng.standard_normal(n)
        interior = rng.uniform(xs[0], xs[-1], 96)
        span = xs[-1] - xs[0]
        queries = np.concatenate(
            [interior, xs,
             np.asarray([xs[0], xs[-1],
                         xs[0] - 0.125 * span,
                         xs[-1] + 0.125 * span,
                         np.inf, -np.inf])])
        cases.append((f"n{n}-random", xs, ys, queries))
    xs4 = np.asarray([0.25, 0.5, 0.75, 1.0])
    cases.append(("n4-constant", xs4, np.full(4, 3.5),
                  np.linspace(0.0, 1.25, 21)))
    cases.append(("n4-inf-y", xs4,
                  np.asarray([np.inf, -np.inf, 1.0, -1.0]),
                  np.linspace(0.3, 0.95, 17)))
    cases.append(("n3-nan-y",
                  np.asarray([0.25, 0.5, 0.75]),
                  np.asarray([np.nan, -1.0, 1.0]),
                  np.linspace(0.2, 0.8, 13)))
    return cases


def _blas_cases(np):
    """(operation, label, a, b, expected) — platform-BLAS lowering.

    `_debug_platform_blas` accepts only tightly C- or F-contiguous
    float64 operands; F operands exercise the row-major Trans entry —
    the same call shape the writers' `A.T @ x` / `A @ B.T` emit.
    """
    rng = np.random.default_rng(SEED ^ 0xB1A5)
    pool = np.asarray(ADVERSARIAL_POOL_UINT64, dtype=np.uint64).view(
        np.float64)
    cases = []
    for n in (0, 1, 2, 3, 5, 16):
        a = rng.standard_normal(n)
        b = np.resize(pool, n).copy() if n == 3 else (
            rng.standard_normal(n))
        # The oracle is `a @ b` — the matmul ufunc's DOUBLE_dot path
        # (cblas_ddot). np.dot's 1-D@1-D lowering is a DIFFERENT kernel
        # (PyArray_InnerProduct's sequential loop) and must not stand in
        # for it — they legitimately differ (e.g. -0.0 vs +0.0).
        cases.append(("dot", f"n{n}", a, b, a @ b))
    for m, n in ((3, 3), (4, 2), (2, 5), (7, 7)):
        a = rng.standard_normal((m, n))
        v = rng.standard_normal(n)
        cases.append(("matvec", f"c{m}x{n}", a, v, a @ v))
        # The oracle must run on the SAME F-order operand: numpy picks a
        # different cblas entry for F input and the result legitimately
        # differs bitwise from the C-order evaluation.
        af = np.asfortranarray(a)
        cases.append(("matvec", f"f{m}x{n}", af, v, af @ v))
    for m, k, n in ((3, 3, 3), (2, 3, 4), (5, 5, 5)):
        a = rng.standard_normal((m, k))
        b = rng.standard_normal((k, n))
        cases.append(("matmul", f"cc{m}x{k}x{n}", a, b, a @ b))
        af = np.asfortranarray(a)
        bf = np.asfortranarray(b)
        cases.append(("matmul", f"fc{m}x{k}x{n}", af, b, af @ b))
        cases.append(("matmul", f"cf{m}x{k}x{n}", a, bf, a @ bf))
        cases.append(("matmul", f"ff{m}x{k}x{n}", af, bf, af @ bf))
    return cases


def _rider_linalg_cases(np):
    """(operation, label, a, b, expected) — spindle-controller mirrors.

    The operations name the production helpers in
    rider/spindle_linalg.hpp; `expected` is the literal NumPy
    expression each one ports.
    """
    rng = np.random.default_rng(SEED ^ 0x11DE)
    pool = np.asarray(ADVERSARIAL_POOL_UINT64, dtype=np.uint64).view(
        np.float64)
    cases = []
    ops3 = (("matvec", lambda a, b: a @ b),
            ("mat_t_vec", lambda a, b: a.T @ b),
            ("mat_t_vec_xz", lambda a, b: (a.T @ b)[[0, 2]]),
            ("matmul", lambda a, b: a @ b),
            ("mat_t_mul", lambda a, b: a.T @ b))
    for round_ in range(4):
        a = rng.standard_normal((3, 3))
        v = rng.standard_normal(3)
        b = rng.standard_normal((3, 3))
        for operation, oracle in ops3:
            operand = b if operation in ("matmul", "mat_t_mul") else v
            cases.append((operation, f"r{round_}", a, operand,
                          oracle(a, operand)))
    a3 = np.resize(pool, 9).reshape(3, 3).copy()
    v3 = np.resize(pool[::-1], 3).copy()
    b3 = np.resize(pool[::-1], 9).reshape(3, 3).copy()
    for operation, oracle in ops3:
        operand = b3 if operation in ("matmul", "mat_t_mul") else v3
        cases.append((operation, "adversarial", a3, operand,
                      oracle(a3, operand)))
    for round_ in range(4):
        a = rng.standard_normal((2, 2))
        v = rng.standard_normal(2)
        cases.append(("matvec2", f"r{round_}", a, v, a @ v))
        cases.append(("mat_t_vec2", f"r{round_}", a, v, a.T @ v))
    a2 = np.resize(pool, 4).reshape(2, 2).copy()
    v2 = np.resize(pool[::-1], 2).copy()
    cases.append(("matvec2", "adversarial", a2, v2, a2 @ v2))
    cases.append(("mat_t_vec2", "adversarial", a2, v2, a2.T @ v2))
    for n in (1, 2, 3, 5):
        v = rng.standard_normal(n)
        cases.append(("norm", f"n{n}", v, np.zeros(0),
                      np.linalg.norm(v)))
    cases.append(("norm", "adversarial",
                  np.resize(pool, 5).copy(), np.zeros(0),
                  np.linalg.norm(np.resize(pool, 5))))
    return cases


# --------------------------------------------------------------------
# Environment record — interpreter / provider / CPU, verbatim
# --------------------------------------------------------------------

def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def _environment(np, module, build: Path, artifact: Path) -> dict:
    libc_name, libc_version = platform.libc_ver()
    cpu_features = {}
    try:
        from numpy._core import _multiarray_umath
        cpu_features = {
            name: bool(flag)
            for name, flag in _multiarray_umath.__cpu_features__.items()
        }
    except (ImportError, AttributeError) as exc:
        cpu_features = {"_error": repr(exc)}

    blas_pools = None
    try:
        import threadpoolctl
        np.zeros((8, 8)) @ np.zeros((8, 8))  # force provider load
        blas_pools = threadpoolctl.threadpool_info()
    except ImportError:
        blas_pools = [{"_error": "threadpoolctl not installed"}]
    except Exception as exc:  # noqa: BLE001 - record, never hide
        blas_pools = [{"_error": repr(exc)}]

    identity = {}
    for key in ("runtime_identity",):
        probe = getattr(module, key, None)
        if probe is None:
            continue
        try:
            identity[key] = _jsonable(probe())
        except Exception as exc:  # noqa: BLE001
            identity[key] = {"_error": repr(exc)}

    return {
        "interpreter": {
            "executable": sys.executable,
            "version": sys.version,
            "version_info": list(sys.version_info[:3]),
            "implementation": platform.python_implementation(),
            "compiler": platform.python_compiler(),
            "soabi": sysconfig.get_config_var("SOABI"),
        },
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "version": platform.version(),
            "machine": platform.machine(),
            "processor": platform.processor(),
            "libc": {"name": libc_name or None,
                     "version": libc_version or None},
        },
        "numpy": {
            "version": np.__version__,
            "config": _jsonable(getattr(np.__config__, "CONFIG", None)),
            "cpu_features_true": sorted(
                name for name, flag in cpu_features.items() if flag),
            "cpu_features_error": cpu_features.get("_error"),
            "blas_threadpools": _jsonable(blas_pools),
        },
        "artifact": {
            "selected_build": str(build),
            "extension": str(artifact),
            "extension_sha256": _sha256(artifact),
        },
        "extension_identity": identity,
        "modules_loaded_before_native": sorted(
            name for name in sys.modules
            if name.split(".")[0] in
            ("numpy", "scipy", "mujoco", "numba", "threadpoolctl")),
    }


# --------------------------------------------------------------------
# Operation table — each entry: (actual via debug surface, oracle)
# --------------------------------------------------------------------

def _operations(np, module):
    def pneum(xs, ys):
        return module._debug_pneumatic_math(xs, ys)

    def contract(name, xs, ys):
        return np.asarray(module._debug_numeric_contract(name, xs, ys))

    ops = [
        ("tanh",
         lambda xs, ys: pneum(xs, ys)["tanh"],
         lambda xs, ys: np.tanh(xs),
         "vector"),
        ("hypot",
         lambda xs, ys: pneum(xs, ys)["hypot"],
         lambda xs, ys: np.hypot(xs, ys),
         "pairs"),
        ("python_hypot",
         lambda xs, ys: contract("python_hypot", xs, ys),
         lambda xs, ys: np.array(
             [math.hypot(float(x), float(y)) for x, y in zip(xs, ys)]),
         "pairs"),
        ("minimum",
         lambda xs, ys: contract("minimum", xs, ys),
         lambda xs, ys: np.minimum(xs, ys),
         "pairs"),
        ("maximum",
         lambda xs, ys: contract("maximum", xs, ys),
         lambda xs, ys: np.maximum(xs, ys),
         "pairs"),
        # np.multiply(float scalar, ndarray element) per lane — the
        # wheel's broadcast-scalar kernel resolves a dual-NaN lane to
        # whichever operand it loads into fmul operand 0 (scalar on the
        # AppleClang wheel, array element on the manylinux wheel); an
        # sNaN quiets and wins from either position. The oracle runs
        # the real 64-lane broadcast per pair, never a Python-level
        # multiply — only the ufunc exposes the kernel's operand order.
        ("multiply_broadcast",
         lambda xs, ys: contract("multiply_broadcast", xs, ys),
         lambda xs, ys: np.array(
             [np.multiply(float(x), np.full(64, y))[0]
              for x, y in zip(xs, ys)]),
         "pairs"),
        ("sin",
         lambda xs, ys: contract("sin", xs, ys),
         lambda xs, ys: np.sin(xs),
         "vector"),
        ("cos",
         lambda xs, ys: contract("cos", xs, ys),
         lambda xs, ys: np.cos(xs),
         "vector"),
    ]
    return ops


# --------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------

def run(build: Path) -> dict:
    import numpy as np

    from bike_sim.native.artifact import (extension_artifacts,
                                         load_native_extension)

    artifacts = extension_artifacts(build)
    if len(artifacts) != 1:
        raise RuntimeError(
            f"expected exactly one bike_native extension in {build}, "
            f"found {len(artifacts)}")
    artifact = artifacts[0].resolve()
    module = load_native_extension(build)

    environment = _environment(np, module, build, artifact)

    records = []
    xs = _corpus(np, 256)
    ys = _corpus(np, 256, seed=SEED ^ 7)

    for name, actual_fn, expected_fn, shape in _operations(np, module):
        for label, (x_in, y_in) in {
            "corpus": (xs, ys),
            "strided": (xs[::3], ys[::3]),
            "reversed": (xs[::-1], ys[::-1]),
        }.items():
            x_run = np.ascontiguousarray(x_in)
            y_run = np.ascontiguousarray(y_in)
            record = {"operation": name, "case": label,
                      "input_uint64": [
                          int(v) for v in np.concatenate(
                              [np.ascontiguousarray(
                                  x_in).view(np.uint64).ravel(),
                               np.ascontiguousarray(
                                   y_in).view(np.uint64).ravel()])]}
            try:
                with np.errstate(all="ignore"):
                    expected = expected_fn(x_in, y_in)
                actual = actual_fn(x_run, y_run)
            except AttributeError as exc:
                record["classification"] = "binding-missing"
                record["error"] = str(exc)
            except Exception as exc:  # noqa: BLE001
                record["classification"] = "error"
                record["error"] = repr(exc)
            else:
                record.update(compare_arrays(expected, actual))
            records.append(record)

    # np.sum — its own shapes (reduction, not elementwise).
    for label, vector in _sum_cases(np):
        record = {"operation": "sum", "case": label,
                  "input_uint64": [
                      int(v) for v in np.ascontiguousarray(
                          vector).view(np.uint64).ravel()]}
        try:
            with np.errstate(all="ignore"):
                expected = np.asarray(np.sum(vector), dtype=np.float64)
            actual = np.asarray(module._debug_numeric_contract(
                "sum", np.ascontiguousarray(vector),
                np.zeros(0, dtype=np.float64)))
        except AttributeError as exc:
            record["classification"] = "binding-missing"
            record["error"] = str(exc)
        except Exception as exc:  # noqa: BLE001
            record["classification"] = "error"
            record["error"] = repr(exc)
        else:
            record.update(compare_arrays(expected, actual))
        records.append(record)

    # np.interp — the general-table compiled_base.c np_interp mirror
    # (biketyre::np_interp through _debug_interp). Interior queries hit
    # the slope*(x-xp)+fp lowering; knot/endpoint/inf-y lanes cover the
    # early-out, NaN retry and constant-segment fallback.
    for label, xs, ys, queries in _interp_cases(np):
        record = {"operation": "interp", "case": label,
                  "input_uint64": [
                      int(v) for v in np.concatenate(
                          [xs.view(np.uint64), ys.view(np.uint64),
                           queries.view(np.uint64)])]}
        try:
            with np.errstate(all="ignore"):
                expected = np.interp(queries, xs, ys)
            actual = np.asarray(
                [module._debug_interp(float(q), xs, ys)
                 for q in queries])
        except AttributeError as exc:
            record["classification"] = "binding-missing"
            record["error"] = str(exc)
        except Exception as exc:  # noqa: BLE001
            record["classification"] = "error"
            record["error"] = repr(exc)
        else:
            record.update(compare_arrays(expected, actual))
        records.append(record)

    # Platform BLAS lowering corpus — _debug_platform_blas drives the
    # same blas::ddot/dgemv/dgemm wrappers the production writers call.
    for operation, label, a, b, expected in _blas_cases(np):
        record = {"operation": f"blas-{operation}", "case": label,
                  "input_uint64": [
                      int(v) for v in np.concatenate(
                          [np.ascontiguousarray(a).view(
                              np.uint64).ravel(),
                           np.ascontiguousarray(b).view(
                               np.uint64).ravel()])]}
        try:
            with np.errstate(all="ignore"):
                expected = np.asarray(expected, dtype=np.float64)
            actual = np.asarray(
                module._debug_platform_blas(a, b, operation))
        except AttributeError as exc:
            record["classification"] = "binding-missing"
            record["error"] = str(exc)
        except Exception as exc:  # noqa: BLE001
            record["classification"] = "error"
            record["error"] = repr(exc)
        else:
            record.update(compare_arrays(expected, actual))
        records.append(record)

    # Rider spindle linear algebra — _debug_rider_linalg drives the
    # same rider:: helpers spindle_controller.cpp calls.
    for operation, label, a, b, expected in _rider_linalg_cases(np):
        record = {"operation": f"rider-{operation}", "case": label,
                  "input_uint64": [
                      int(v) for v in np.concatenate(
                          [np.ascontiguousarray(a).view(
                              np.uint64).ravel(),
                           np.ascontiguousarray(b).view(
                               np.uint64).ravel()])]}
        try:
            with np.errstate(all="ignore"):
                expected = np.asarray(expected, dtype=np.float64)
            actual = np.asarray(
                module._debug_rider_linalg(operation, a, b))
        except AttributeError as exc:
            record["classification"] = "binding-missing"
            record["error"] = str(exc)
        except Exception as exc:  # noqa: BLE001
            record["classification"] = "error"
            record["error"] = repr(exc)
        else:
            record.update(compare_arrays(expected, actual))
        records.append(record)

    mismatches = [r for r in records
                  if r.get("classification") not in (None, "equal")]
    return {
        "schema_version": SCHEMA_VERSION,
        "environment": environment,
        "operations": records,
        "summary": {
            "seed": SEED,
            "records": len(records),
            "equal": sum(1 for r in records
                         if r.get("classification") == "equal"),
            "mismatched": len(mismatches),
            "first_failures": [
                {"operation": r["operation"], "case": r["case"],
                 "classification": r.get("classification"),
                 "first_mismatch": r.get("first_mismatch"),
                 "error": r.get("error")}
                for r in mismatches[:8]
            ],
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build", required=True,
                        help="absolute path of the selected native build dir")
    parser.add_argument("--output", required=True,
                        help="JSON report path (created atomically)")
    args = parser.parse_args(argv)

    build = Path(args.build)
    if not build.is_absolute():
        parser.error("--build must be an absolute path")
    build = build.resolve()

    try:
        report = run(build)
    except Exception as exc:  # noqa: BLE001 - evidence, not a crash
        report = {
            "schema_version": SCHEMA_VERSION,
            "environment": {
                "interpreter": {"executable": sys.executable,
                                "version": sys.version},
                "platform": {"system": platform.system(),
                             "machine": platform.machine()},
            },
            "operations": [],
            "summary": {"seed": SEED, "records": 0, "equal": 0,
                        "mismatched": 1,
                        "first_failures": [
                            {"classification": "load-error",
                             "error": repr(exc)}]},
        }

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    temporary.write_text(json.dumps(report, indent=1) + "\n",
                         encoding="utf-8")
    os.replace(temporary, output)

    summary = report["summary"]
    print(json.dumps(summary, indent=1))
    return 0 if not summary["mismatched"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
