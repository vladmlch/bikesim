"""Shared MuJoCo model-asset helpers for the tools/colab_gpu/ runners.

Consolidates logic previously duplicated across ``mjwarp_batch_sweep.py``,
``cpu_batch_sweep.py``, ``metal_hybrid_sweep.py`` and ``mjwarp_metal_sweep.py``:

* loading a ``MjModel`` and populating ``hfield_data`` from ``hfield_data.bin``;
* extracting the 1D terrain slice with the same float32 operation order as
  ``native/src/gpu/cpu_batch_runner.cpp``;
* reading the authoritative initial ``qpos``/``qvel`` from
  ``captured_bootstrap.json``.

Assets are expected next to the model XML (``asset_dir(xml_path)``).
This module imports ``mujoco`` lazily, so importing it does not require MuJoCo.
"""

from __future__ import annotations

import json
import os
import pathlib
import shutil
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import mujoco

HFIELD_BIN_NAME = "hfield_data.bin"
CAPTURED_BOOTSTRAP_NAME = "captured_bootstrap.json"

_FLOAT64_LE = np.dtype("<f8")
_FLOAT64_NBYTES = 8


def asset_dir(xml_path: str | os.PathLike) -> pathlib.Path:
    """Return the directory holding the assets that belong to ``xml_path``.

    The path is resolved (symlinks followed) to match the historical loader behaviour.
    """
    return pathlib.Path(os.fspath(xml_path)).resolve().parent


def load_hfield_bin(model: "mujoco.MjModel", xml_path: str | os.PathLike) -> None:
    """Fill ``model.hfield_data`` from ``asset_dir(xml_path)/hfield_data.bin``.

    The file must hold little-endian float64 values, exactly ``model.nhfielddata`` of them.
    A model without heightfield data (``nhfielddata == 0``) is left untouched.

    Raises:
        FileNotFoundError: the binary file does not exist.
        ValueError: the file size does not equal ``nhfielddata * 8`` bytes.
    On failure the model is not modified.
    """
    nhfielddata = int(model.nhfielddata)
    if nhfielddata == 0:
        return
    path = asset_dir(xml_path) / HFIELD_BIN_NAME
    if not path.is_file():
        raise FileNotFoundError(
            f"hfield data file not found: {path} (model has nhfielddata={nhfielddata})"
        )
    raw = path.read_bytes()
    expected_nbytes = nhfielddata * _FLOAT64_NBYTES
    if len(raw) != expected_nbytes:
        raise ValueError(
            f"{path}: size is {len(raw)} bytes, expected {expected_nbytes} bytes "
            f"({nhfielddata} float64 values)"
        )
    # Validation is complete; only now mutate the model.
    model.hfield_data[:] = np.frombuffer(raw, dtype=_FLOAT64_LE)


def load_model_with_assets(xml_path: str | os.PathLike) -> "mujoco.MjModel":
    """Load a ``MjModel`` from ``xml_path`` and apply ``hfield_data.bin`` when present.

    Raises:
        FileNotFoundError: the XML file does not exist.
    """
    import mujoco

    path = pathlib.Path(os.fspath(xml_path))
    if not path.is_file():
        raise FileNotFoundError(f"MuJoCo model XML not found: {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    load_hfield_bin(model, path)
    return model


def extract_heightfield_1d(model: "mujoco.MjModel") -> tuple[np.ndarray, float, float]:
    """Return the first heightfield row as ``(heights, x_min, dx)``.

    Mirrors ``native/src/gpu/cpu_batch_runner.cpp`` bit-for-bit: every intermediate is a
    float32 scalar operated on in the same order as the C++ source (the native build uses
    ``-ffp-contract=off``, so no fused multiply-add is involved).

    * ``heights[i] = float32(hfield_data[i]) * float32(size_z) + float32(geom_z)`` for
      ``i < ncol``, where ``geom_z`` is the z of the first HFIELD geom (0 if none).
    * ``x_min = float32(geom_x) - float32(size_x)`` for that geom (0 if none).
    * ``dx = total_len_x / float32(max(ncol - 1, 1))`` with
      ``total_len_x = 2 * size_x`` if ``size_x > 0`` else ``float32(ncol - 1) * 0.005f``.

    ``x_min`` and ``dx`` are returned as Python floats, which hold the float32 values exactly.

    Raises:
        ValueError: the model has no heightfield, or ``hfield_ncol[0]`` is outside
            ``[1, nhfielddata]``.
    """
    import mujoco

    nhfield = int(model.nhfield)
    if nhfield <= 0:
        raise ValueError("model has no heightfield (nhfield == 0)")
    nhfielddata = int(model.nhfielddata)
    ncol = int(model.hfield_ncol[0])
    if ncol <= 0 or ncol > nhfielddata:
        raise ValueError(
            f"hfield 0 has ncol={ncol}, which must be in [1, nhfielddata={nhfielddata}]"
        )

    size_x_half = np.float32(model.hfield_size[0, 0])
    scale_z = np.float32(model.hfield_size[0, 2])

    x_min = np.float32(0.0)
    z_offset = np.float32(0.0)
    for g in range(int(model.ngeom)):
        if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_HFIELD):
            x_min = np.float32(model.geom_pos[g, 0]) - size_x_half
            z_offset = np.float32(model.geom_pos[g, 2])
            break

    heights = np.asarray(model.hfield_data[:ncol], dtype=np.float32) * scale_z + z_offset

    if size_x_half > np.float32(0.0):
        total_len_x = np.float32(2.0) * size_x_half
    else:
        total_len_x = np.float32(ncol - 1) * np.float32(0.005)
    dx = total_len_x / np.float32(max(ncol - 1, 1))

    return (
        np.ascontiguousarray(heights, dtype=np.float32),
        float(x_min),
        float(dx),
    )


def load_captured_initial_state(
    xml_path: str | os.PathLike, nq: int, nv: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return the captured initial ``(qpos, qvel)`` as float64 arrays.

    Reads ``asset_dir(xml_path)/captured_bootstrap.json``. ``state.integration_state`` is laid
    out as ``[time, qpos[nq], qvel[nv], ...]``; ``state.model_dims`` must match ``nq``/``nv``.

    Raises:
        FileNotFoundError: the JSON file does not exist.
        ValueError: ``model_dims`` does not match ``nq``/``nv``, or ``integration_state`` is
            too short or missing.
    """
    path = asset_dir(xml_path) / CAPTURED_BOOTSTRAP_NAME
    if not path.is_file():
        raise FileNotFoundError(f"captured bootstrap not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        doc = json.load(fh)

    try:
        state = doc["state"]
        model_dims = state["model_dims"]
        integration_state = state["integration_state"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"{path}: missing state.model_dims or state.integration_state"
        ) from exc

    got_nq = model_dims.get("nq")
    got_nv = model_dims.get("nv")
    if got_nq != nq or got_nv != nv:
        raise ValueError(
            f"{path}: model_dims nq={got_nq}, nv={got_nv} do not match requested nq={nq}, nv={nv}"
        )

    needed = 1 + nq + nv
    if len(integration_state) < needed:
        raise ValueError(
            f"{path}: integration_state has {len(integration_state)} entries, "
            f"needs at least {needed} (time + qpos[{nq}] + qvel[{nv}])"
        )
    qpos = np.array(integration_state[1 : 1 + nq], dtype=np.float64)
    qvel = np.array(integration_state[1 + nq : needed], dtype=np.float64)
    return qpos, qvel


def _resolve_wheel_geom(model: "mujoco.MjModel", body_name: str, *geom_names: str) -> tuple[int, int]:
    import mujoco

    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
    if body_id < 0:
        raise ValueError(f"model has no body {body_name!r}")
    for gname in geom_names:
        gid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname))
        if gid >= 0:
            return body_id, gid
    for g in range(int(model.ngeom)):
        if (
            int(model.geom_bodyid[g]) == body_id
            and int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
        ):
            return body_id, g
    raise ValueError(f"model has no contact sphere geom on body {body_name!r}")


def synthesize_welded_assets(
    model: "mujoco.MjModel",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Synthesize a flat-road ``hfield_data`` array and settled ``(qpos0, qvel0)``.

    Matches ``test_gpu_batch_runners.cpp::{seed_flat_welded_road,build_welded_initial_state}``
    for ``tests/fixtures/model/welded.xml`` (``nq=27``) when sidecar assets are absent.
    """
    import mujoco

    hfield_data = np.asarray(model.hfield_data, dtype=np.float64).copy()
    hf_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "road"))
    terrain_geom = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain"))
    if hf_id >= 0 and terrain_geom >= 0 and int(model.nhfielddata) > 0:
        nrow = int(model.hfield_nrow[hf_id])
        ncol = int(model.hfield_ncol[hf_id])
        adr = int(model.hfield_adr[hf_id])
        sz = float(model.hfield_size[hf_id, 2])
        pos_z = float(model.geom_pos[terrain_geom, 2])
        flat_norm = (0.0 - pos_z) / sz
        hfield_data[adr : adr + nrow * ncol] = flat_norm
        model.hfield_data[:] = hfield_data

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    root_x = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x"))
    root_z = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_z"))
    rider_rx = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rider_root_x"))
    rider_rz = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rider_root_z"))
    fw_jnt = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "front_wheel_spin"))
    rw_jnt = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rear_wheel_spin"))
    crank_jnt = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "crank_spin"))
    ds_jnt = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "drive_shaft_spin"))
    for jname, jid in [
        ("root_x", root_x),
        ("root_z", root_z),
        ("rider_root_x", rider_rx),
        ("rider_root_z", rider_rz),
        ("front_wheel_spin", fw_jnt),
        ("rear_wheel_spin", rw_jnt),
        ("crank_spin", crank_jnt),
    ]:
        if jid < 0:
            raise ValueError(f"welded model missing required joint {jname!r}")

    data.qpos[int(model.jnt_qposadr[root_x])] = 2.0
    data.qpos[int(model.jnt_qposadr[rider_rx])] = 2.0
    mujoco.mj_forward(model, data)

    f_body, f_geom = _resolve_wheel_geom(model, "front_wheel", "front_contact", "geom_front_contact")
    r_body, r_geom = _resolve_wheel_geom(model, "rear_wheel", "rear_contact", "geom_rear_contact")
    f_r = float(model.geom_size[f_geom, 0])
    r_r = float(model.geom_size[r_geom, 0])
    min_clear = min(
        float(data.xpos[f_body, 2]) - f_r,
        float(data.xpos[r_body, 2]) - r_r,
    )
    dz = -0.015 - min_clear
    data.qpos[int(model.jnt_qposadr[root_z])] += dz
    data.qpos[int(model.jnt_qposadr[rider_rz])] += dz
    data.qvel[int(model.jnt_dofadr[root_x])] = 5.0
    data.qvel[int(model.jnt_dofadr[rider_rx])] = 5.0
    data.qvel[int(model.jnt_dofadr[fw_jnt])] = 5.0 / f_r
    data.qvel[int(model.jnt_dofadr[rw_jnt])] = 5.0 / r_r
    data.qvel[int(model.jnt_dofadr[crank_jnt])] = 6.0
    if ds_jnt >= 0:
        data.qvel[int(model.jnt_dofadr[ds_jnt])] = 6.0
    mujoco.mj_forward(model, data)

    return (
        np.ascontiguousarray(hfield_data, dtype=_FLOAT64_LE),
        np.ascontiguousarray(data.qpos, dtype=np.float64),
        np.ascontiguousarray(data.qvel, dtype=np.float64),
    )


def materialize_welded_assets_dir(
    xml_path: str | os.PathLike,
    out_dir: str | os.PathLike,
) -> pathlib.Path:
    """Return an asset-complete XML path, synthesizing sidecar files into ``out_dir`` if absent."""
    import mujoco

    src_xml = pathlib.Path(os.fspath(xml_path)).resolve()
    src_dir = src_xml.parent
    if (src_dir / HFIELD_BIN_NAME).is_file() and (src_dir / CAPTURED_BOOTSTRAP_NAME).is_file():
        return src_xml

    out_path = pathlib.Path(os.fspath(out_dir))
    out_path.mkdir(parents=True, exist_ok=True)
    dst_xml = out_path / src_xml.name
    shutil.copyfile(src_xml, dst_xml)

    model = mujoco.MjModel.from_xml_path(str(dst_xml))
    hfield_data, qpos0, qvel0 = synthesize_welded_assets(model)
    (out_path / HFIELD_BIN_NAME).write_bytes(hfield_data.astype(_FLOAT64_LE).tobytes())
    bootstrap_doc = {
        "state": {
            "model_dims": {"nq": int(model.nq), "nv": int(model.nv)},
            "integration_state": [0.0, *qpos0.tolist(), *qvel0.tolist()],
        }
    }
    (out_path / CAPTURED_BOOTSTRAP_NAME).write_text(
        json.dumps(bootstrap_doc, indent=2) + "\n", encoding="utf-8"
    )
    return dst_xml
