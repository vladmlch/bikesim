"""BIKEST02 versioned benchmark state bundle — generator and reader.

A bundle directory is one immutable generation of the benchmark inputs:

    model.mjb      compiled MjModel bytes
    state.npz      float64 arrays: time(1,), qpos(nq), qvel(nv), act(na),
                   ctrl(nu), qfrc_applied(nv), xfrc_applied(nbody,6),
                   qacc_warmstart(nv)
    state.bin      80-byte little-endian header + binary64 payload (below)
    manifest.json  provenance: model dims, timestep, source config/track,
                   warmup steps, generator commit, checksums, workload params

An artifacts ROOT additionally carries a small pointer file:

    current.json   {"format": 1, "bundle": "<bundle-dir-name>"}

`write_bundle` builds a NEW versioned directory under the root, validates every
written file, then publishes `current.json` atomically via os.replace. Readers
resolve the pointer once, so a bundle can never be observed mid-generation and
files from different generations can never be mixed. A concrete bundle
directory (one that directly contains the four files) is equally valid input —
tests use it to skip the pointer indirection.

state.bin header layout (exactly 80 bytes, little-endian):

    offset  0  magic8         "BIKEST02"
    offset  8  version        u32 = 2
    offset 12  flags          u32 = 0
    offset 16  nq, nv, na, nu, nbody   u64 each
    offset 56  payload_bytes  u64
    offset 64  model_crc64    u64  (CRC64-ECMA over model.mjb bytes)
    offset 72  state_crc64    u64  (CRC64-ECMA over the payload bytes)

Payload is binary64 little-endian, in order:
    time(1), qpos(nq), qvel(nv), act(na), ctrl(nu), qfrc_applied(nv),
    xfrc_applied flattened (6*nbody), qacc_warmstart(nv)

Required payload = 8 * (1 + nq + 3*nv + na + nu + 6*nbody) bytes; the reader
recomputes it with overflow checks and requires the file to end exactly there.
Dimensions, the CRCs, and the exact length are independently mandatory checks:
a dimension match does not excuse a checksum mismatch and vice versa.

CRC64-ECMA parameters: polynomial 0x42F0E1EBA9EA3693, init 0, no reflection,
no final xor. Check vector: b"123456789" -> 0x6C40DF5F0B497347 (asserted in
the tests). The checksums detect corruption and cross-generation mixing; they
are an integrity signal, not an authenticity/security guarantee.

The legacy 0xBEA0 layout (R2) is retired: files starting with that magic are
rejected with a regeneration instruction.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
from typing import TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:
    from collections.abc import Mapping


MAGIC = b"BIKEST02"
VERSION = 2
FLAGS = 0
HEADER_BYTES = 80
# magic8, version u32, flags u32, nq/nv/na/nu/nbody u64, payload_bytes u64,
# model_crc64 u64, state_crc64 u64.
HEADER = struct.Struct("<8s2I8Q")
assert HEADER.size == HEADER_BYTES

LEGACY_MAGIC_I32 = 0xBEA0
LEGACY_MAGIC_BYTES = struct.pack("<i", LEGACY_MAGIC_I32)

MODEL_FILE = "model.mjb"
NPZ_FILE = "state.npz"
BIN_FILE = "state.bin"
MANIFEST_FILE = "manifest.json"
POINTER_FILE = "current.json"
BUNDLE_MEMBERS = (MODEL_FILE, BIN_FILE, NPZ_FILE, MANIFEST_FILE)

DIM_FIELDS = ("nq", "nv", "na", "nu", "nbody")
# Payload section order: time, qpos, qvel, act, ctrl, qfrc_applied,
# xfrc_applied, qacc_warmstart.
PAYLOAD_KEYS = (
    "time", "qpos", "qvel", "act", "ctrl", "qfrc_applied", "xfrc_applied",
    "qacc_warmstart",
)

_MJT_SIZE_MAX = (1 << 63) - 1  # engine extents are signed mjtSize

# Canonical deterministic workload definition shared by bench.cpp and
# bench.py. The seed feeds a splitmix64 generator implemented identically in
# both languages; every parameter here is duplicated as compiled constants in
# the two benchmarks and re-emitted in each run's JSON so the conformance test
# can prove the workload definitions are equal.
WORKLOAD = {
    "format": 1,
    "seed": "0x42f0e1eba9ea3693",
    "warmup_steps": 500,
    "writers": 8,
    "channels": 100,
    "name_lookups_max": 10,
    "efc_buckets": 8,
    "efc_rows_max": 200,
    "attachment_count": 5,
    "points_per_attachment": 7,
    "record_churn": 6,
    "ledger_terms": [
        "spring", "damper", "bias", "applied", "constraint", "actuator",
    ],
    "batch": {"period": 10, "batches": 6, "iterations": 16, "width": 3},
    "regularization": 1e-6,
    "solve_eta": 1e-9,
    "forwards_per_step": 2,
}

_REPO_ROOT = Path(__file__).resolve().parents[2]


class StateFormatError(ValueError):
    """Raised for any malformed state.bin/bundle content."""


# ---------------------------------------------------------------------------
# CRC64-ECMA (MSB-first, poly 0x42F0E1EBA9EA3693, init 0, no reflection/xorout)
# ---------------------------------------------------------------------------

def _crc64_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 56
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC64_POLY) if crc & (1 << 63) else crc << 1
            crc &= 0xFFFFFFFFFFFFFFFF
        table.append(crc)
    return table


_CRC64_POLY = 0x42F0E1EBA9EA3693
_CRC64_TABLE = _crc64_table()


def crc64_ecma(data: bytes | bytearray | memoryview) -> int:
    """CRC64-ECMA-182 (init 0, refin/refout false, xorout 0)."""
    crc = 0
    table = _CRC64_TABLE
    for byte in data:
        crc = table[((crc >> 56) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFFFFFFFFFFFFFF)
    return crc


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------

def _dim_from_arrays(arrays: Mapping[str, np.ndarray]) -> dict[str, int]:
    """Model dimensions implied by the payload arrays, with sanity checks."""
    xfrc = np.asarray(arrays["xfrc_applied"])
    if xfrc.size % 6 != 0:
        raise StateFormatError(
            f"xfrc_applied has {xfrc.size} elements; expected a multiple of 6"
        )
    if np.asarray(arrays["time"]).size != 1:
        raise StateFormatError("time must hold exactly one scalar")
    dims = {
        "nq": int(np.asarray(arrays["qpos"]).size),
        "nv": int(np.asarray(arrays["qvel"]).size),
        "na": int(np.asarray(arrays["act"]).size),
        "nu": int(np.asarray(arrays["ctrl"]).size),
        "nbody": int(xfrc.size // 6),
    }
    for key, count in (
        ("qfrc_applied", dims["nv"]), ("qacc_warmstart", dims["nv"]),
    ):
        if np.asarray(arrays[key]).size != count:
            raise StateFormatError(
                f"{key} has {np.asarray(arrays[key]).size} elements; "
                f"expected nv={count}"
            )
    return dims


def required_payload_bytes(nq: int, nv: int, na: int, nu: int, nbody: int) -> int:
    """8 * (1 + nq + 3*nv + na + nu + 6*nbody) — the exact payload extent."""
    elements = 1 + nq + 3 * nv + na + nu + 6 * nbody
    if elements < 0:
        raise StateFormatError("negative element count")
    return 8 * elements


def serialize_state(arrays: Mapping[str, np.ndarray], model_bytes: bytes) -> bytes:
    """Assemble the versioned state.bin image (header + payload)."""
    dims = _dim_from_arrays(arrays)
    payload = b"".join(
        np.ascontiguousarray(arrays[key], dtype="<f8").tobytes()
        for key in PAYLOAD_KEYS
    )
    if len(payload) != required_payload_bytes(**dims):
        raise StateFormatError("payload assembly drifted from dimensions")
    header = HEADER.pack(
        MAGIC, VERSION, FLAGS,
        dims["nq"], dims["nv"], dims["na"], dims["nu"], dims["nbody"],
        len(payload), crc64_ecma(model_bytes), crc64_ecma(payload),
    )
    return header + payload


# ---------------------------------------------------------------------------
# Reader
# ---------------------------------------------------------------------------

def _require(condition: bool, message: str, label: str) -> None:
    if not condition:
        raise StateFormatError(f"{label}: {message}")


def _decode_sections(
    payload: bytes, dims: dict[str, int], label: str,
) -> dict[str, np.ndarray]:
    """Slice the validated payload into named arrays (same order as the writer)."""
    sizes = {
        "time": 1,
        "qpos": dims["nq"],
        "qvel": dims["nv"],
        "act": dims["na"],
        "ctrl": dims["nu"],
        "qfrc_applied": dims["nv"],
        "xfrc_applied": 6 * dims["nbody"],
        "qacc_warmstart": dims["nv"],
    }
    result: dict[str, np.ndarray] = {}
    offset = 0
    for key in PAYLOAD_KEYS:
        count = sizes[key]
        section = np.frombuffer(
            payload, dtype="<f8", count=count, offset=offset,
        ).astype(np.float64)
        if not np.isfinite(section).all():
            raise StateFormatError(f"{label}: non-finite {key} data")
        result[key] = section
        offset += 8 * count
    if offset != len(payload):
        raise StateFormatError(f"{label}: payload accounting drift")
    result["xfrc_applied"] = result["xfrc_applied"].reshape(dims["nbody"], 6)
    return result


def read_state(
    state_path: Path | str, model_path: Path | str,
) -> dict[str, np.ndarray]:
    """Read and validate a BIKEST02 state.bin against its bundle model.mjb.

    Checks, in order: size floor, magic, version, flags, u64 field ranges,
    dimension equality with the loaded model, declared vs required vs actual
    payload length, payload CRC64, model.mjb CRC64, and finiteness of every
    stored scalar. Returns the arrays keyed like state.npz.
    """
    state_path = Path(state_path)
    model_path = Path(model_path)
    label = str(state_path)
    raw = state_path.read_bytes()
    _require(len(raw) >= HEADER_BYTES, "truncated state header", label)
    magic = raw[:8]
    if magic != MAGIC:
        if raw[:4] == LEGACY_MAGIC_BYTES:
            raise StateFormatError(
                f"{label}: legacy 0xBEA0 state.bin is retired; regenerate the "
                "bundle with tools/proto_native_bench/dump_model.py"
            )
        raise StateFormatError(f"{label}: unsupported state format")
    (
        _magic, version, flags, nq, nv, na, nu, nbody,
        payload_bytes, model_crc64, state_crc64,
    ) = HEADER.unpack(raw[:HEADER_BYTES])
    _require(version == VERSION, f"unsupported state version {version}", label)
    _require(flags == FLAGS, f"unsupported state flags {flags}", label)

    header_dims = {"nq": nq, "nv": nv, "na": na, "nu": nu, "nbody": nbody}
    for name, value in header_dims.items():
        _require(
            value <= _MJT_SIZE_MAX,
            f"{name} header field {value} is outside the mjtSize domain", label,
        )
    _require(
        model_path.is_file(), "model.mjb is not a readable file", label,
    )
    model = mujoco.MjModel.from_binary_path(str(model_path))
    for name, value in header_dims.items():
        model_dim = int(getattr(model, name))
        _require(
            value == model_dim,
            f"state {name}={value} does not match model dimension {model_dim}",
            label,
        )
    required = required_payload_bytes(nq, nv, na, nu, nbody)
    _require(
        payload_bytes == required,
        f"payload_bytes field {payload_bytes} does not match required "
        f"{required}", label,
    )
    _require(
        len(raw) - HEADER_BYTES == required,
        f"state payload length is {len(raw) - HEADER_BYTES} bytes; expected "
        f"{required}", label,
    )
    payload = raw[HEADER_BYTES:]
    _require(
        crc64_ecma(payload) == state_crc64,
        "state payload checksum mismatch", label,
    )
    _require(
        model_path.is_file(), "model.mjb is not a readable file", label,
    )
    _require(
        crc64_ecma(model_path.read_bytes()) == model_crc64,
        "model checksum mismatch (bundle members from different generations?)",
        label,
    )
    return _decode_sections(payload, header_dims, label)


# ---------------------------------------------------------------------------
# Bundle writer + pointer publication
# ---------------------------------------------------------------------------

def _snapshot_arrays(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    """Freeze one consistent mjData snapshot; dims must match the model."""
    arrays = {
        "time": np.array([float(data.time)]),
        "qpos": np.array(data.qpos, dtype=np.float64),
        "qvel": np.array(data.qvel, dtype=np.float64),
        "act": np.array(data.act, dtype=np.float64),
        "ctrl": np.array(data.ctrl, dtype=np.float64),
        "qfrc_applied": np.array(data.qfrc_applied, dtype=np.float64),
        "xfrc_applied": np.array(data.xfrc_applied, dtype=np.float64),
        "qacc_warmstart": np.array(data.qacc_warmstart, dtype=np.float64),
    }
    dims = _dim_from_arrays(arrays)
    for name, value in dims.items():
        model_dim = int(getattr(model, name))
        if value != model_dim:
            raise StateFormatError(
                f"snapshot {name}={value} does not match model dimension "
                f"{model_dim}"
            )
    for key, array in arrays.items():
        if not np.isfinite(array).all():
            raise StateFormatError(f"snapshot {key} contains non-finite data")
    return arrays


def _generator_commit() -> dict[str, object]:
    """Best-effort provenance: HEAD commit and worktree dirty flag."""
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=_REPO_ROOT, check=True,
            capture_output=True, text=True, timeout=15,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain", "--untracked-files=no"],
                cwd=_REPO_ROOT, check=True, capture_output=True, text=True,
                timeout=15,
            ).stdout.strip()
        )
        return {"commit": commit, "worktree_dirty": dirty}
    except (OSError, subprocess.SubprocessError):
        return {"commit": None, "worktree_dirty": None}


def _sha256(path: Path) -> str:
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


def _validate_bundle(bundle: Path, snapshot: Mapping[str, np.ndarray]) -> None:
    """Re-read every written member; the bundle must be self-consistent
    before the pointer is published."""
    for member in BUNDLE_MEMBERS:
        if not (bundle / member).is_file():
            raise StateFormatError(
                f"generated bundle is incomplete: missing {member}"
            )
    binary = read_state(bundle / BIN_FILE, bundle / MODEL_FILE)
    for key in PAYLOAD_KEYS:
        if binary[key].tobytes() != np.ascontiguousarray(
            snapshot[key], dtype=np.float64,
        ).tobytes():
            raise StateFormatError(f"generated state.bin {key} differs from snapshot")
    with np.load(bundle / NPZ_FILE) as archive:
        if set(archive.files) != set(PAYLOAD_KEYS):
            raise StateFormatError("generated state.npz member set drifted")
        for key in PAYLOAD_KEYS:
            if archive[key].tobytes() != binary[key].tobytes():
                raise StateFormatError(
                    f"state.npz {key} is not byte-equal to state.bin"
                )
    manifest = json.loads((bundle / MANIFEST_FILE).read_text())
    if manifest.get("format") != 1 or "model" not in manifest or "checksums" not in manifest:
        raise StateFormatError("generated manifest.json failed to validate")


def _publish_pointer(root: Path, bundle_name: str) -> None:
    """Atomically replace root/current.json (write temp + fsync + rename)."""
    pointer = json.dumps(
        {"format": 1, "bundle": bundle_name}, separators=(",", ":"),
    )
    fd, tmp_name = tempfile.mkstemp(
        prefix=".current.", suffix=".json.tmp", dir=root,
    )
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(pointer)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, root / POINTER_FILE)
    except BaseException:
        Path(tmp_name).unlink(missing_ok=True)
        raise
    # Best-effort directory fsync so the rename itself is durable.
    try:
        dir_fd = os.open(root, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except OSError:
        pass


def write_bundle(
    root: Path | str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    name: str | None = None,
    source_track: str | None = None,
    source_config: str | None = None,
    warmup_steps: int | None = None,
) -> Path:
    """Serialize one immutable mjData snapshot as a versioned bundle.

    Creates ``root/<name>/`` (auto-named ``v2-<utc>-p<pid>-<seq>`` when unset),
    writes model.mjb/state.npz/state.bin/manifest.json, re-validates every
    file, then atomically publishes ``root/current.json``. Returns the bundle
    directory. A failed write removes the partial directory and never touches
    the pointer.
    """
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    snapshot = _snapshot_arrays(model, data)

    if name is None:
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        seq = 0
        while True:
            candidate = f"v2-{stamp}-p{os.getpid()}-{seq}"
            if not (root / candidate).exists():
                name = candidate
                break
            seq += 1
    bundle = root / name
    try:
        bundle.mkdir()
    except FileExistsError:
        raise FileExistsError(
            f"bundle directory already exists (bundles are immutable): {bundle}"
        ) from None

    try:
        model_path = bundle / MODEL_FILE
        mujoco.mj_saveModel(model, str(model_path))
        model_bytes = model_path.read_bytes()
        (bundle / BIN_FILE).write_bytes(serialize_state(snapshot, model_bytes))
        np.savez(bundle / NPZ_FILE, **snapshot)
        manifest = {
            "format": 1,
            "bundle_format": {"magic": MAGIC.decode(), "version": VERSION},
            "model": {
                "nq": model.nq, "nv": model.nv, "na": model.na,
                "nu": model.nu, "nbody": model.nbody,
                "timestep": float(model.opt.timestep),
            },
            "source": {
                "track": source_track,
                "physics_config": source_config,
                "warmup_steps": warmup_steps,
            },
            "generator": {
                "script": "tools/proto_native_bench/dump_model.py",
                "python": sys.version.split()[0],
                "mujoco": mujoco.__version__,
                **_generator_commit(),
            },
            "checksums": {
                "model_mjb_crc64": f"0x{crc64_ecma(model_bytes):016x}",
                "state_payload_crc64": (
                    f"0x{crc64_ecma((bundle / BIN_FILE).read_bytes()[HEADER_BYTES:]):016x}"
                ),
                "model_mjb_sha256": _sha256(model_path),
                "state_bin_sha256": _sha256(bundle / BIN_FILE),
                "state_npz_sha256": _sha256(bundle / NPZ_FILE),
            },
            "workload": WORKLOAD,
        }
        (bundle / MANIFEST_FILE).write_text(json.dumps(manifest, indent=2) + "\n")
        _validate_bundle(bundle, snapshot)
    except BaseException:
        # Never leave a partially written bundle behind; the pointer keeps
        # naming the last complete generation (or is absent entirely).
        import shutil

        shutil.rmtree(bundle, ignore_errors=True)
        raise

    _publish_pointer(root, name)
    return bundle


# ---------------------------------------------------------------------------
# Bundle resolution (consumer side)
# ---------------------------------------------------------------------------

def resolve_bundle(artifacts: Path | str) -> Path:
    """Resolve an --artifacts argument to a concrete bundle directory.

    If ``artifacts/current.json`` exists it is the pointer: the named member
    directory must be a complete bundle. Otherwise ``artifacts`` itself must
    directly contain the four bundle files. The pointer is read exactly once.
    """
    root = Path(artifacts)
    pointer = root / POINTER_FILE
    if pointer.is_file():
        try:
            document = json.loads(pointer.read_text())
        except (OSError, json.JSONDecodeError) as error:
            raise StateFormatError(
                f"{pointer}: malformed current.json pointer: {error}"
            ) from error
        if not isinstance(document, dict) or document.get("format") != 1:
            raise StateFormatError(
                f"{pointer}: unsupported current.json pointer format"
            )
        name = document.get("bundle")
        safe = (
            isinstance(name, str) and name != ""
            and all(ch.isalnum() or ch in "-_." for ch in name)
            and name not in (".", "..")
        )
        if not safe:
            raise StateFormatError(
                f"{pointer}: unsafe or missing bundle name {name!r}"
            )
        bundle = root / name
        if not bundle.is_dir():
            raise StateFormatError(
                f"{pointer}: points at missing bundle directory '{name}'"
            )
    else:
        bundle = root
    for member in BUNDLE_MEMBERS:
        if not (bundle / member).is_file():
            raise StateFormatError(
                f"{bundle}: bundle member {member} is not a readable file"
            )
    return bundle
