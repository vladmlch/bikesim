"""Tests for tools/colab_gpu/model_assets.py.

Run standalone:
    uv run --with numpy==2.5.2 --with mujoco==3.12.0 python tools/colab_gpu/test_model_assets.py
Also collectable by pytest (test_* functions).
"""

from __future__ import annotations

import ast
import json
import os
import pathlib
import shutil
import sys
import tempfile
import traceback
from collections.abc import Callable, Iterator
from contextlib import contextmanager

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import mujoco  # noqa: E402

import model_assets as ma  # noqa: E402

REPO_ROOT = _HERE.parents[1]
ASSET_ROOT = REPO_ROOT / "tools" / "fp32_probe" / "artifacts" / "common"
REAL_XML = ASSET_ROOT / "model.xml"
REAL_BIN = ASSET_ROOT / "hfield_data.bin"
REAL_JSON = ASSET_ROOT / "captured_bootstrap.json"
OLD_SWEEP_PY = _HERE / "mjwarp_batch_sweep.py"
SCRATCH_ROOT = pathlib.Path("/tmp/bike_orch/E")

NQ = 26
NV = 26


@contextmanager
def _scratch_dir() -> Iterator[pathlib.Path]:
    SCRATCH_ROOT.mkdir(parents=True, exist_ok=True)
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="model_assets_", dir=SCRATCH_ROOT))
    try:
        yield tmp
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _real_model() -> mujoco.MjModel:
    return ma.load_model_with_assets(REAL_XML)


def _f32_bits(a: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.float32).view(np.uint32)


def _f64_bits(a: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.float64).view(np.uint64)


def _no_hfield_model() -> mujoco.MjModel:
    xml = '<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>'
    return mujoco.MjModel.from_xml_string(xml)


# --------------------------------------------------------------------------- happy paths


def test_real_model_loads_with_hfield_bin() -> None:
    m = _real_model()
    assert int(m.nhfield) == 1
    assert int(m.nhfielddata) == int(m.hfield_ncol[0]) * 2, (m.nhfielddata, m.hfield_ncol[0])
    raw = np.fromfile(REAL_BIN, dtype="<f8")
    assert raw.size == int(m.nhfielddata)
    assert np.array_equal(_f64_bits(m.hfield_data), _f64_bits(raw)), "hfield_data != hfield_data.bin"


def test_extract_heightfield_1d_matches_independent_recompute() -> None:
    m = _real_model()
    heights, x_min, dx = ma.extract_heightfield_1d(m)

    ncol = int(m.hfield_ncol[0])
    size_x = np.float32(m.hfield_size[0, 0])
    size_z = np.float32(m.hfield_size[0, 2])
    gid = next(
        g for g in range(int(m.ngeom)) if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_HFIELD)
    )
    exp_x_min = np.float32(m.geom_pos[gid, 0]) - size_x
    z_off = np.float32(m.geom_pos[gid, 2])

    exp_heights = np.empty(ncol, dtype=np.float32)
    for i in range(ncol):  # scalar loop: independent of the vectorised implementation
        exp_heights[i] = np.float32(np.float32(m.hfield_data[i]) * size_z) + z_off

    if size_x > np.float32(0.0):
        exp_total = np.float32(2.0) * size_x
    else:
        exp_total = np.float32(ncol - 1) * np.float32(0.005)
    exp_dx = exp_total / np.float32(max(ncol - 1, 1))

    assert heights.dtype == np.float32 and heights.shape == (ncol,)
    assert np.array_equal(_f32_bits(heights), _f32_bits(exp_heights)), "heights not bit-identical"
    assert np.float32(x_min).view(np.uint32) == exp_x_min.view(np.uint32), (x_min, exp_x_min)
    assert np.float32(dx).view(np.uint32) == exp_dx.view(np.uint32), (dx, exp_dx)
    assert np.any(heights != 0.0), "heights are all zero; hfield_data.bin was not applied"
    print(f"  ncol={ncol} x_min={x_min!r} dx={dx!r} z_off={float(z_off)!r} size_x={float(size_x)!r}")


def test_extract_heightfield_1d_size_x_fallback_branch() -> None:
    m = _real_model()
    m.hfield_size[0, 0] = 0.0  # force the (ncol-1)*0.005f fallback
    ncol = int(m.hfield_ncol[0])
    _, _, dx = ma.extract_heightfield_1d(m)
    exp = np.float32(np.float32(ncol - 1) * np.float32(0.005)) / np.float32(max(ncol - 1, 1))
    assert np.float32(dx).view(np.uint32) == exp.view(np.uint32), (dx, exp)


def test_extract_heightfield_1d_rejects_model_without_hfield() -> None:
    try:
        ma.extract_heightfield_1d(_no_hfield_model())
    except ValueError:
        return
    raise AssertionError("expected ValueError for a model without a heightfield")


def test_load_hfield_bin_noop_without_hfield() -> None:
    m = _no_hfield_model()
    with _scratch_dir() as tmp:
        ma.load_hfield_bin(m, tmp / "model.xml")  # no bin exists here; must still be a no-op


def test_load_captured_initial_state_matches_json() -> None:
    qpos, qvel = ma.load_captured_initial_state(REAL_XML, NQ, NV)
    with REAL_JSON.open("r", encoding="utf-8") as fh:
        il = json.load(fh)["state"]["integration_state"]
    exp_qpos = np.asarray(il[1 : 1 + NQ], dtype=np.float64)
    exp_qvel = np.asarray(il[1 + NQ : 1 + NQ + NV], dtype=np.float64)
    assert qpos.dtype == np.float64 and qpos.shape == (NQ,)
    assert qvel.dtype == np.float64 and qvel.shape == (NV,)
    assert np.array_equal(_f64_bits(qpos), _f64_bits(exp_qpos))
    assert np.array_equal(_f64_bits(qvel), _f64_bits(exp_qvel))


def test_load_model_with_assets_missing_xml() -> None:
    with _scratch_dir() as tmp:
        try:
            ma.load_model_with_assets(tmp / "absent.xml")
        except FileNotFoundError:
            return
    raise AssertionError("expected FileNotFoundError for a missing XML")


# --------------------------------------------------------------------------- error paths


def test_load_hfield_bin_missing_bin_leaves_model_untouched() -> None:
    m = _real_model()
    before = _f64_bits(m.hfield_data).copy()
    with _scratch_dir() as tmp:
        shutil.copy(REAL_XML, tmp / "model.xml")  # no hfield_data.bin next to it
        try:
            ma.load_hfield_bin(m, tmp / "model.xml")
        except FileNotFoundError:
            pass
        else:
            raise AssertionError("expected FileNotFoundError for missing hfield_data.bin")
        try:
            ma.load_model_with_assets(tmp / "model.xml")
        except FileNotFoundError:
            pass
        else:
            raise AssertionError("expected FileNotFoundError from load_model_with_assets")
    assert np.array_equal(_f64_bits(m.hfield_data), before), "model modified on failure"


def test_load_hfield_bin_truncated_bin_raises_and_leaves_model_untouched() -> None:
    m = _real_model()
    before = _f64_bits(m.hfield_data).copy()
    nbytes = int(m.nhfielddata) * 8
    raw = REAL_BIN.read_bytes()
    assert len(raw) == nbytes
    with _scratch_dir() as tmp:
        shutil.copy(REAL_XML, tmp / "model.xml")
        (tmp / "hfield_data.bin").write_bytes(raw[: nbytes - 8])  # one value short
        try:
            ma.load_hfield_bin(m, tmp / "model.xml")
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for truncated hfield_data.bin")
        (tmp / "hfield_data.bin").write_bytes(raw[: nbytes - 3])  # not a multiple of 8
        try:
            ma.load_hfield_bin(m, tmp / "model.xml")
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for non-multiple-of-8 hfield_data.bin")
    assert np.array_equal(_f64_bits(m.hfield_data), before), "model modified on failure"


def test_load_hfield_bin_oversized_bin_raises() -> None:
    m = _real_model()
    raw = REAL_BIN.read_bytes()
    with _scratch_dir() as tmp:
        shutil.copy(REAL_XML, tmp / "model.xml")
        (tmp / "hfield_data.bin").write_bytes(raw + np.float64(0.0).tobytes())
        try:
            ma.load_hfield_bin(m, tmp / "model.xml")
        except ValueError:
            return
    raise AssertionError("expected ValueError for oversized hfield_data.bin")


def test_load_captured_initial_state_missing_json() -> None:
    with _scratch_dir() as tmp:
        shutil.copy(REAL_XML, tmp / "model.xml")
        try:
            ma.load_captured_initial_state(tmp / "model.xml", NQ, NV)
        except FileNotFoundError:
            return
    raise AssertionError("expected FileNotFoundError for missing captured_bootstrap.json")


def test_load_captured_initial_state_wrong_nq_or_nv() -> None:
    for nq, nv in ((NQ - 1, NV), (NQ, NV + 1)):
        try:
            ma.load_captured_initial_state(REAL_XML, nq, nv)
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for nq={nq}, nv={nv}")


def test_load_captured_initial_state_short_integration_state() -> None:
    doc = {
        "state": {
            "model_dims": {"nq": NQ, "nv": NV},
            "integration_state": [0.0] * (1 + NQ + NV - 1),
        }
    }
    with _scratch_dir() as tmp:
        (tmp / "model.xml").write_text("<mujoco/>", encoding="utf-8")
        (tmp / "captured_bootstrap.json").write_text(json.dumps(doc), encoding="utf-8")
        try:
            ma.load_captured_initial_state(tmp / "model.xml", NQ, NV)
        except ValueError:
            return
    raise AssertionError("expected ValueError for short integration_state")


# --------------------------------------------------------------------------- reporting only


def _old_settled_constants() -> dict[str, tuple[float, ...]] | None:
    """Read the legacy hardcoded tuples via AST (no import of the sweep module)."""
    if not OLD_SWEEP_PY.is_file():
        return None
    tree = ast.parse(OLD_SWEEP_PY.read_text(encoding="utf-8"))
    found: dict[str, tuple[float, ...]] = {}
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id in ("_SETTLED_QPOS0", "_SETTLED_QVEL0") and node.value is not None:
                found[node.target.id] = tuple(ast.literal_eval(node.value))
    return found or None


def test_report_old_settled_constants_vs_captured() -> None:
    """Print-only comparison: the legacy constants were hand-edited, so differences are expected."""
    old = _old_settled_constants()
    if old is None:
        print("  legacy _SETTLED_* constants not present in mjwarp_batch_sweep.py (already removed)")
        return
    qpos, qvel = ma.load_captured_initial_state(REAL_XML, NQ, NV)
    for name, captured in (("_SETTLED_QPOS0", qpos), ("_SETTLED_QVEL0", qvel)):
        if name not in old:
            print(f"  {name}: absent")
            continue
        old_arr = np.asarray(old[name], dtype=np.float64)
        print(f"  {name}: len old={old_arr.size} captured={captured.size}")
        if old_arr.size != captured.size:
            print("    length mismatch; no element-wise comparison")
            continue
        diff = np.abs(old_arr - captured)
        imax = int(np.argmax(diff))
        differing = [int(i) for i in np.flatnonzero(old_arr != captured)]
        print(f"    max|old-captured|={float(diff[imax]):.6e} at index {imax}")
        print(f"    indices with any difference (count={len(differing)}): {differing}")
        for i in differing:
            print(f"      [{i:2d}] old={old_arr[i]!r} captured={captured[i]!r} |d|={diff[i]:.3e}")


# --------------------------------------------------------------------------- runner

_TESTS: tuple[Callable[[], None], ...] = tuple(
    obj for name, obj in list(globals().items()) if name.startswith("test_") and callable(obj)
)


def main() -> int:
    print(f"mujoco {mujoco.__version__}, numpy {np.__version__}, python {sys.version.split()[0]}")
    print(f"scratch root: {SCRATCH_ROOT}")
    failures = 0
    for fn in _TESTS:
        print(f"RUN  {fn.__name__}")
        try:
            fn()
        except Exception:
            failures += 1
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
        else:
            print(f"PASS {fn.__name__}")
    print(f"{len(_TESTS) - failures}/{len(_TESTS)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
