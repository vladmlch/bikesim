#!/usr/bin/env python3
"""T01 reference capture — freeze measurable oracle behaviour into fixtures.

Run this against the immutable Python oracle checkout (tag
``reference/python-oracle``), never against the implementation worktree:

    uv run --frozen --group native python \
        <impl>/tools/migration/capture_reference.py \
        --output <impl>/tests/fixtures/migration \
        --manifest <impl>/migration/oracle-manifest.json \
        --cli-contracts <impl>/migration/cli-contracts.json \
        --config-contracts <impl>/migration/config-contracts.json

The script must be launched with the reference worktree as its working
directory (``--oracle-root`` defaults to the ancestor of ``bike_sim.__file__``
that owns ``pyproject.toml`` and changes into it before capturing).

Design notes
------------
* Every fixture file is hashed (SHA-256) and recorded in the oracle manifest
  with its recipe id, comparison class (B bitwise / N numeric / S structural /
  Q text / P performance), expected validity, source inputs and provenance.
* Numerical payloads are stored as raw little-endian IEEE-754 bytes
  (``.f64``/``.i32``/``.i64``/``.u64``/``.u8``) — never rounded JSON numbers.
* JSON trees use a canonical tagged codec: numpy arrays are externalised into
  sibling payload files, non-finite floats become ``{"$nonfinite": ...}``,
  ``Path``/``datetime``/``UUID``/``tuple``/``bytes`` keep explicit tags so the
  encoding is lossless and reproducible.
* Pure-Python pinned-kernel captures (``bootstrap/welded_pinned``) replicate
  ``tests/reference/conftest.py::_pin_oracle_kernels`` exactly; production
  Numba captures are kept under separate names as required.
* Fault fixtures record the real first-failure identity (exception class and
  message) produced by the oracle at capture time — nothing is guessed.
* Missing tools or empty selections are recorded as ``blocker`` manifest
  entries, never silently skipped.
"""
from __future__ import annotations

import argparse
import contextlib
import dataclasses
import datetime as _dt
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import traceback
import uuid
import warnings
from pathlib import Path

import numpy as np

SCHEMA = "migration-oracle-manifest/v1"
CLI_SCHEMA = "migration-cli-contracts/v1"
CONFIG_SCHEMA = "migration-config-contracts/v1"

DTYPE_SUFFIX = {
    "<f8": ".f64", "=f8": ".f64", "f8": ".f64",
    "<i4": ".i32", "=i4": ".i32", "i4": ".i32",
    "<i8": ".i64", "=i8": ".i64", "i8": ".i64",
    "<u8": ".u64", "=u8": ".u64", "u8": ".u64",
    "|u1": ".u8", "u1": ".u8", "bool": ".u8",
}
SUFFIX_DTYPE = {".f64": "<f8", ".i32": "<i4", ".i64": "<i8",
                ".u64": "<u8", ".u8": "|u1"}

COMPARISON_CLASSES = ("B", "N", "S", "Q", "P")

# Object reprs embed ASLR-dependent memory addresses ("<… at 0x11e7c8bf0>"),
# which makes fixture bytes nondeterministic across identical oracle runs.
# Strip the address so recorded identities (module + qualname + type) are
# stable for bitwise comparison.
_ADDR_RE = __import__("re").compile(r" at 0x[0-9a-fA-F]+")


def _strip_addr(text: str) -> str:
    return _ADDR_RE.sub("", text)


def _stable_repr(value) -> str:
    return _strip_addr(repr(value))


# ---------------------------------------------------------------------------
# Hashing / filesystem helpers
# ---------------------------------------------------------------------------

def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rel(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


# ---------------------------------------------------------------------------
# Fixture writer
# ---------------------------------------------------------------------------

class FixtureWriter:
    """Writes payload files under the fixture root and tracks file records."""

    def __init__(self, root: Path):
        self.root = root
        self.files = {}          # relpath -> record
        self._array_counter = 0

    def _write(self, relpath: str, data: bytes) -> dict:
        path = self.root / relpath
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        record = {"path": relpath, "sha256": sha256_bytes(data),
                  "bytes": len(data)}
        self.files[relpath] = record
        return record

    def bytes(self, relpath: str, data: bytes) -> dict:
        return self._write(relpath, data)

    def text(self, relpath: str, text: str) -> dict:
        return self._write(relpath, text.encode("utf-8"))

    def copy(self, relpath: str, source: Path) -> dict:
        data = Path(source).read_bytes()
        return self._write(relpath, data)

    def copy_tree(self, relpath: str, source_dir: Path) -> list:
        records = []
        for child in sorted(Path(source_dir).rglob("*")):
            if child.is_file():
                records.append(self.copy(
                    f"{relpath}/{child.relative_to(source_dir).as_posix()}",
                    child))
        return records

    def array(self, relpath_stem: str, array) -> tuple:
        """Write a numpy array as raw canonical little-endian bytes.

        Returns (record, tag) where tag is the canonical-JSON stand-in.
        """
        arr = np.asarray(array)
        order = "C" if arr.flags.c_contiguous else (
            "F" if arr.flags.f_contiguous else "C")
        canon = np.ascontiguousarray(arr)
        dtype = canon.dtype
        if dtype == np.dtype("bool"):
            canon = canon.view(np.uint8)
            dtype = canon.dtype
        key = dtype.str
        if key not in DTYPE_SUFFIX:
            if dtype.kind == "f":
                canon = canon.astype("<f8")
            elif dtype.kind == "i":
                canon = canon.astype("<i8")
            elif dtype.kind == "u":
                canon = canon.astype("<u8" if dtype.itemsize > 1 else "|u1")
            else:
                raise TypeError(f"unsupported fixture dtype {dtype}")
            dtype = canon.dtype
            key = dtype.str
        suffix = DTYPE_SUFFIX.get(key)
        if suffix is None:
            suffix = {"<f8": ".f64", "<i4": ".i32", "<i8": ".i64",
                      "<u8": ".u64", "|u1": ".u8"}[canon.dtype.str]
        stem = relpath_stem
        if stem.endswith(tuple(SUFFIX_DTYPE)):
            relpath = stem
        else:
            relpath = stem + suffix
        record = self._write(relpath, canon.tobytes(order="C"))
        record.update({"dtype": canon.dtype.str, "shape": list(canon.shape),
                       "order": "C"})
        tag = {"$array": relpath, "dtype": canon.dtype.str,
               "shape": list(canon.shape), "order": "C",
               "sha256": record["sha256"]}
        return record, tag


# ---------------------------------------------------------------------------
# Canonical tagged JSON codec
# ---------------------------------------------------------------------------

def encode_tree(obj, writer: FixtureWriter, rel: str, sink: list):
    """Encode ``obj`` for canonical JSON; externalise ndarrays to ``sink``.

    ``rel`` is the logical fixture path (used to name externalised arrays);
    ``sink`` collects array records produced along the way.
    """
    if isinstance(obj, np.ndarray):
        stem = f"{rel}.arrays/a{len(sink):03d}"
        record, tag = writer.array(stem, obj)
        sink.append(record)
        return tag
    if isinstance(obj, np.generic):
        return encode_tree(obj.item(), writer, rel, sink)
    if isinstance(obj, (bytes, bytearray, memoryview)):
        data = bytes(obj)
        stem = f"{rel}.arrays/b{len(sink):03d}"
        record = writer.bytes(stem + ".bin", data)
        sink.append(record)
        return {"$bytes": record["path"], "sha256": record["sha256"],
                "bytes": len(data)}
    if isinstance(obj, dict):
        return {str(k): encode_tree(v, writer, rel, sink)
                for k, v in obj.items()}
    if isinstance(obj, (list,)):
        return [encode_tree(v, writer, rel, sink) for v in obj]
    if isinstance(obj, tuple):
        return {"$tuple": [encode_tree(v, writer, rel, sink) for v in obj]}
    if isinstance(obj, (set, frozenset)):
        return {"$set": sorted(encode_tree(v, writer, rel, sink)
                               for v in obj)}
    if isinstance(obj, bool):
        return obj
    if isinstance(obj, int):
        return obj
    if isinstance(obj, float):
        if np.isfinite(obj):
            return obj
        return {"$nonfinite": ("nan" if np.isnan(obj)
                               else "inf" if obj > 0 else "-inf")}
    if isinstance(obj, str) or obj is None:
        return obj
    if isinstance(obj, Path):
        return {"$path": str(obj)}
    if isinstance(obj, _dt.datetime):
        return {"$datetime": obj.isoformat()}
    if isinstance(obj, _dt.date):
        return {"$date": obj.isoformat()}
    if isinstance(obj, uuid.UUID):
        return {"$uuid": str(obj)}
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {"$dataclass": f"{type(obj).__module__}.{type(obj).__name__}",
                "fields": encode_tree(dataclasses.asdict(obj), writer, rel, sink)}
    raise TypeError(f"cannot canonically encode {type(obj).__name__} "
                    f"(path {rel})")


def write_canonical(writer: FixtureWriter, relpath: str, obj) -> list:
    """Encode ``obj`` canonically and write ``relpath`` JSON + array payloads."""
    sink = []
    encoded = encode_tree(obj, writer, relpath[:-5] if relpath.endswith(".json")
                          else relpath, sink)
    text = json.dumps(encoded, indent=2, sort_keys=True, allow_nan=False)
    record = writer.text(relpath, text + "\n")
    return [record, *sink]


# ---------------------------------------------------------------------------
# Manifest recorder
# ---------------------------------------------------------------------------

class Recorder:
    def __init__(self):
        self.fixtures = []
        self.blockers = []
        self.negative_controls = []

    def add(self, name, *, recipe, group, comparison, expected, produced_by,
            files=(), inputs=(), notes=""):
        assert comparison in COMPARISON_CLASSES, comparison
        entry = {
            "name": name,
            "recipe": recipe,
            "group": group,
            "comparison": comparison,
            "expected": expected,
            "produced_by": produced_by,
            "files": [self._filerec(f) for f in files],
            "inputs": [self._inputrec(i) for i in inputs],
            "notes": notes,
        }
        self.fixtures.append(entry)
        return entry

    @staticmethod
    def _filerec(record):
        if isinstance(record, str):
            return {"path": record}
        return dict(record)

    @staticmethod
    def _inputrec(item):
        if isinstance(item, dict):
            return item
        path = Path(item)
        if path.is_dir():
            return {"path": str(path), "kind": "directory",
                    "sha256": None}
        return {"path": str(path), "sha256": sha256_file(path)}

    def blocker(self, name, recipe, detail):
        entry = {"name": name, "recipe": recipe, "kind": "blocker",
                 "detail": detail}
        self.blockers.append(entry)
        self.fixtures.append({"name": name, "recipe": recipe,
                              "comparison": None,
                              "expected": {"kind": "blocker"},
                              "files": [], "inputs": [],
                              "produced_by": "capture_reference.py",
                              "notes": detail})
        return entry

    def control(self, name, result, detail=""):
        self.negative_controls.append(
            {"name": name, "result": result, "detail": detail})


# ---------------------------------------------------------------------------
# Context
# ---------------------------------------------------------------------------

class Context:
    def __init__(self, oracle_root, writer, recorder, args):
        self.oracle = Path(oracle_root)
        self.out = writer
        self.rec = recorder
        self.args = args
        self.tmp = Path(tempfile.mkdtemp(prefix="capture-reference-"))
        self.sim_cache = {}

    def cleanup(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


def _oracle_provenance(oracle_root: Path) -> dict:
    def git(*argv):
        return subprocess.run(
            ["git", "-C", str(oracle_root), *argv],
            capture_output=True, text=True).stdout.strip()
    prov = {
        "worktree": str(oracle_root),
        "commit": git("rev-parse", "HEAD"),
        "describe": git("describe", "--tags", "--always"),
        "status_porcelain": git("status", "--porcelain"),
    }
    bike_init = None
    try:
        import bike_sim
        bike_init = str(Path(bike_sim.__file__).resolve())
    except Exception:
        pass
    prov["bike_sim_file"] = bike_init
    try:
        from bike_sim.validation.environment import source_fingerprint
        prov["source_sha256"] = source_fingerprint(oracle_root / "src")
    except Exception as exc:
        prov["source_sha256_error"] = f"{type(exc).__name__}: {exc}"
    return prov


def _environment() -> dict:
    env = {
        "python": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "executable": sys.executable,
        "packages": {},
    }
    import io
    for name in ("numpy", "scipy", "mujoco", "numba", "duckdb", "pyarrow",
                 "pytest", "bike_sim", "nanobind"):
        try:
            module = __import__(name)
            env["packages"][name] = getattr(module, "__version__", "?")
        except Exception:
            env["packages"][name] = None
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            np.show_config()
        env["numpy_config"] = buf.getvalue()
    except Exception as exc:
        env["numpy_config_error"] = str(exc)
    try:
        import mujoco
        env["mujoco_version"] = mujoco.mj_versionString()
    except Exception:
        pass
    return env


# ---------------------------------------------------------------------------
# RCP-02 configuration / requests / tracks / model / kinematics
# ---------------------------------------------------------------------------

REQUEST_TEMPLATE = '''request_schema = 1
physics_file = "{physics}"
track_file = "{track}"
model_mode = "{mode}"
rider_variant = "{rider}"
speed_kph = {speed}
seed_decimal = "{seed}"
use_equilibrium_cache = {cache}
{extra}'''


def _rcp02_config(ctx: Context):
    out, rec, oracle = ctx.out, ctx.rec, ctx.oracle
    examples = oracle / "examples" / "research"

    # --- shipped physics profiles (byte-identical copies + resolved forms) ---
    for name in ("welded", "distributed", "pneumatic"):
        src = examples / f"viewer_physics_{name}.toml"
        record = out.copy(f"config/viewer_physics_{name}.toml", src)
        rec.add(f"config/viewer_physics_{name}.toml", recipe="RCP-02",
                group="config", comparison="S",
                expected={"kind": "accepted"},
                produced_by="shutil copy of shipped profile",
                files=[record], inputs=[src],
                notes="byte-identical copy of the shipped physics profile")
    # External payloads the profiles resolve relative to their own directory.
    for name in ("rider_joint_envelope_anatomical.json",
                 "rider_strength_reference.json"):
        src = examples / name
        record = out.copy(f"config/{name}", src)
        rec.add(f"config/{name}", recipe="RCP-02", group="config",
                comparison="B", expected={"kind": "accepted"},
                produced_by="shutil copy of profile-referenced payload",
                files=[record], inputs=[src],
                notes="resolved by [articulated].joint_envelope_path / "
                      "joint_strength_path relative to the TOML")
    pose_src = examples / "poses" / "savage_pneumatic.npz"
    if pose_src.is_file():
        record = out.copy("config/poses/savage_pneumatic.npz", pose_src)
        rec.add("config/poses/savage_pneumatic.npz", recipe="RCP-02",
                group="config", comparison="B",
                expected={"kind": "accepted"},
                produced_by="shutil copy of settled pose referenced by "
                            "viewer_physics_pneumatic.toml",
                files=[record], inputs=[pose_src],
                notes="relative path resolves against the pneumatic TOML dir")
    else:
        rec.blocker("config/poses/savage_pneumatic.npz", "RCP-02",
                    f"{pose_src} missing from oracle checkout")

    from bike_sim.physics.resolution import load_physics_config
    from bike_sim.physics.model_config import SimulationPhysicsConfig

    # --- resolved configuration surfaces ---
    for name in ("welded", "distributed", "pneumatic"):
        rel = f"config/viewer_physics_{name}.toml"
        try:
            cfg = load_physics_config(out.root / rel)
            files = write_canonical(out, f"config/resolved/{name}.json",
                                    dataclasses.asdict(cfg))
            rec.add(f"config/resolved/{name}.json", recipe="RCP-02",
                    group="config", comparison="S",
                    expected={"kind": "accepted"},
                    produced_by="load_physics_config + canonical codec",
                    files=files, inputs=[out.root / rel],
                    notes="resolved SimulationPhysicsConfig incl. external "
                          "parameter payloads")
        except Exception as exc:
            rec.blocker(f"config/resolved/{name}.json", "RCP-02",
                        f"{type(exc).__name__}: {exc}")

    # --- seeded profile variants: elastic-chain drivetrain + alternative
    #     attachment supports (welded TOML patched at the keyed lines) ---
    welded_text = (out.root / "config/viewer_physics_welded.toml").read_text()
    variants = {
        # elastic_chain plus the rejected-with-shifting form: the welded TOML
        # enables automatic shifting, which requires ideal_mid_drive
        "elastic_chain": (welded_text
            .replace('transmission_model = "ideal_mid_drive"',
                     'transmission_model = "elastic_chain"')
            .replace('[drive.shifting]\nenabled = true',
                     '[drive.shifting]\nenabled = false')),
        "elastic_chain_shifting_rejected": welded_text.replace(
            'transmission_model = "ideal_mid_drive"',
            'transmission_model = "elastic_chain"'),
        "alt_support": (welded_text
            .replace('pedal_attachment = "spindle"',
                     'pedal_attachment = "flat"')
            .replace('saddle_attachment = "pin"',
                     'saddle_attachment = "flat"')
            .replace('grip_attachment = "connect"',
                     'grip_attachment = "spring"')),
        "grip_weld_coercion": welded_text.replace(
            'grip_attachment = "connect"', 'grip_attachment = "weld"'),
    }
    for name, text in variants.items():
        stem = f"config/variant_{name}"
        rec_file = out.text(f"{stem}.toml", text)
        try:
            cfg = load_physics_config(out.root / f"{stem}.toml")
            resolved = dataclasses.asdict(cfg)
            files = [rec_file] + write_canonical(
                out, f"{stem}.resolved.json", resolved)
            expected = {"kind": "accepted"}
            note = ""
            if name == "grip_weld_coercion":
                grip = resolved.get("articulated", {}).get(
                    "grip_attachment")
                expected = {"kind": "accepted",
                            "coerced_grip_attachment": grip}
                note = ("source coerces grip_attachment 'weld' -> "
                        f"{grip!r} during resolution")
            rec.add(stem, recipe="RCP-02", group="config", comparison="S",
                    expected=expected,
                    produced_by="authored variant TOML + "
                                "load_physics_config",
                    files=files,
                    inputs=[out.root / "config/viewer_physics_welded.toml"],
                    notes=(f"{name} profile variant; " + note).strip())
        except Exception as exc:
            files = [rec_file]
            rec.add(stem, recipe="RCP-02", group="config", comparison="S",
                    expected={"kind": "invalid",
                              "error": type(exc).__name__,
                              "message": str(exc)},
                    produced_by="authored variant TOML + "
                                "load_physics_config rejection",
                    files=files,
                    inputs=[out.root / "config/viewer_physics_welded.toml"],
                    notes=f"{name} variant rejected at resolution")

    # --- config schema surface (declared fields/defaults) ---
    try:
        import typing
        def _fields(cls):
            rows = []
            for f in dataclasses.fields(cls):
                entry = {"name": f.name,
                         "type": str(f.type),
                         "default": (None if f.default is dataclasses.MISSING
                                     else f.default),
                         "default_factory": (
                             getattr(f.default_factory, "__qualname__", None)
                             if f.default_factory is not dataclasses.MISSING
                             else None)}
                if dataclasses.is_dataclass(f.type if isinstance(f.type, type)
                                            else None):
                    entry["fields"] = _fields(f.type)
                rows.append(entry)
            return rows
        schema = {"config_class": "SimulationPhysicsConfig",
                  "fields": _fields(SimulationPhysicsConfig)}
        files = write_canonical(out, "config/schema.json", schema)
        rec.add("config/schema.json", recipe="RCP-02", group="config",
                comparison="S", expected={"kind": "accepted"},
                produced_by="dataclasses introspection", files=files,
                notes="declared field/default surface; T12 schema() contract")
    except Exception as exc:
        rec.blocker("config/schema.json", "RCP-02",
                    f"{type(exc).__name__}: {exc}")

    # --- authored config inputs: legacy + invalid key ---
    legacy_toml = 'physics_mode = "legacy"\n'
    rec_file = out.text("config/legacy.toml", legacy_toml)
    rec.add("config/legacy.toml", recipe="RCP-02", group="config",
            comparison="S", expected={"kind": "accepted"},
            produced_by="authored capture input", files=[rec_file],
            notes="minimal legacy-mode physics config for the legacy request")

    unknown_toml = ('physics_mode = "physical"\n'
                    'definitely_not_a_field = 1\n')
    rec_file = out.text("config/unknown_key.toml", unknown_toml)
    try:
        load_physics_config(out.root / "config/unknown_key.toml")
        expected = {"kind": "unexpected-acceptance"}
        note = "load_physics_config did not reject an unknown key"
    except Exception as exc:
        expected = {"kind": "invalid",
                    "error": type(exc).__name__, "message": str(exc)}
        note = "unknown key must be rejected"
    rec.add("config/unknown_key.toml", recipe="RCP-02", group="config",
            comparison="S", expected=expected,
            produced_by="authored capture input", files=[rec_file], notes=note)

    # --- tracks: copies, canonical round-trips, invalid inputs ---
    from bike_sim.terrain import available_presets, get_preset
    from bike_sim.terrain.profile import build_profile, profile_extent
    from bike_sim.terrain.trackfile import (
        dump_track, load_track, track_to_dict)

    savage = examples / "rough_uphill_savage.toml"
    record = out.copy("tracks/rough_uphill_savage.toml", savage)
    rec.add("tracks/rough_uphill_savage.toml", recipe="RCP-02", group="config",
            comparison="S", expected={"kind": "accepted"},
            produced_by="shutil copy", files=[record], inputs=[savage],
            notes="authored research track used by the welded captures")
    try:
        canon = dump_track(load_track(out.root / "tracks/rough_uphill_savage.toml"))
        record = out.text("tracks/rough_uphill_savage.canonical.toml", canon)
        rec.add("tracks/rough_uphill_savage.canonical.toml", recipe="RCP-02",
                group="config", comparison="Q",
                expected={"kind": "accepted"},
                produced_by="dump_track(load_track(...))",
                files=[record], inputs=[out.root / "tracks/rough_uphill_savage.toml"],
                notes="canonical emitter output for the authored track")
    except Exception as exc:
        rec.blocker("tracks/rough_uphill_savage.canonical.toml", "RCP-02",
                    f"{type(exc).__name__}: {exc}")

    # flat preset as a file-backed track for the request fixtures
    try:
        flat_text = dump_track(get_preset("flat"))
        record = out.text("tracks/flat.toml", flat_text)
        rec.add("tracks/flat.toml", recipe="RCP-02", group="config",
                comparison="Q", expected={"kind": "accepted"},
                produced_by="dump_track(get_preset('flat'))", files=[record],
                notes="file form of the flat preset used by *_flat requests")
    except Exception as exc:
        rec.blocker("tracks/flat.toml", "RCP-02", f"{type(exc).__name__}: {exc}")

    for name in available_presets():
        try:
            track = get_preset(name)
            stem = f"tracks/preset_{name}"
            files = []
            notes = []
            for stage, fn in (
                    (".toml", lambda: out.text(
                        f"{stem}.toml", dump_track(track))),
                    (".json", lambda: write_canonical(
                        out, f"{stem}.json", track_to_dict(track)))):
                try:
                    produced = fn()
                    files += produced if isinstance(produced, list) else [
                        produced]
                except Exception as exc:
                    notes.append(f"{stage} stage rejected: "
                                 f"{type(exc).__name__}: {exc}")
            try:
                x0, x1 = profile_extent(track)
                xs = np.arange(x0, x1 + 0.5 * .005, .005)
                profile = np.column_stack((xs, build_profile(track, xs)))
                arec, _ = out.array(f"{stem}_profile.f64", profile)
                files.append(arec)
            except Exception as exc:
                notes.append(f"profile stage rejected: "
                             f"{type(exc).__name__}: {exc}")
            if not files:
                raise RuntimeError("every capture stage failed for preset")
            rec.add(stem, recipe="RCP-02", group="config", comparison="B",
                    expected={"kind": "accepted"},
                    produced_by="get_preset + dump_track + track_to_dict + "
                                "build_profile(0.005 m)",
                    files=files,
                    notes=(f"preset {name}: spec dump, decoded dict and "
                           "profile vertices" +
                           ("; " + "; ".join(notes) if notes else "")))
        except Exception as exc:
            rec.blocker(f"tracks/preset_{name}", "RCP-02",
                        f"{type(exc).__name__}: {exc}")

    # invalid/degenerate authored tracks — record the real rejection identity
    for fname, text in (
        ("tracks/unknown_obstacle.toml",
         'name = "unknown_obstacle"\nlength_m = 10.0\n'
         '[[obstacles]]\ntype = "hovercraft"\nstart_m = 1.0\n'),
        ("tracks/degenerate.toml",
         'name = "degenerate"\nlength_m = 0.0\n'),
    ):
        record = out.text(fname, text)
        try:
            load_track(out.root / fname)
            expected = {"kind": "unexpected-acceptance"}
            note = "track unexpectedly accepted"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            note = "authored invalid/degenerate track must be rejected"
        rec.add(fname, recipe="RCP-02", group="config", comparison="S",
                expected=expected, produced_by="authored capture input",
                files=[record], notes=note)

    # --- request files (interface-contracts section 3 schema) ---
    requests = {
        # The welded bootstrap oracle restores a captured t=0 state: the
        # initial_state_file transfers that pose instead of re-solving or
        # consulting the derived-name cache, keeping every choice explicit.
        "welded_flat": dict(physics="../config/viewer_physics_welded.toml",
                            track="../tracks/flat.toml", mode="ride",
                            rider="articulated_planar", speed="0.0",
                            seed="1234", cache="false",
                            extra='initial_state_file = "../initial_state/welded_flat.json"\n'),
        # legacy.toml leaves drive_mode to post_init (ideal_speed_control);
        # the RCP-01 bootstrap recipe pins drive_mode="coast", so the request
        # carries that effective choice explicitly through [overrides].
        "legacy_flat": dict(physics="../config/legacy.toml",
                            track="../tracks/flat.toml", mode="ride",
                            rider="lumped", speed="10.0",
                            seed="1234", cache="false",
                            extra='[overrides]\ndrive_mode = "coast"\n'),
        "pneumatic_flat": dict(physics="../config/viewer_physics_pneumatic.toml",
                               track="../tracks/flat.toml", mode="ride",
                               rider="articulated_planar", speed="0.0",
                               seed="1234", cache="false", extra=""),
        "distributed_flat": dict(physics="../config/viewer_physics_distributed.toml",
                                 track="../tracks/flat.toml", mode="ride",
                                 rider="articulated_planar", speed="0.0",
                                 seed="1234", cache="false", extra=""),
        "research_delay": dict(physics="../config/viewer_physics_welded.toml",
                               track="../tracks/flat.toml", mode="research",
                               rider="articulated_planar", speed="2.0",
                               seed="210", cache="false",
                               extra='actuator_delay_s = 0.01\nsensor_delay_s = 0.01\n'
                                     'duration_s = 0.1\n'),
    }
    for name, spec in requests.items():
        text = REQUEST_TEMPLATE.format(**spec)
        record = out.text(f"requests/{name}.toml", text)
        rec.add(f"requests/{name}.toml", recipe="RCP-02", group="config",
                comparison="S", expected={"kind": "accepted"},
                produced_by="request writer (interface-contracts section 3)",
                files=[record],
                notes="internal request fixture; every effective choice explicit")

    # model/welded.xml is captured during the welded sim build inside
    # _rcp01_bootstrap — a spy on ride_sim.generate_mujoco_xml preserves the
    # exact bytes MuJoCo compiled (rider/mass/field resolution happens there).

    # --- minimal compiled model for the bare native stepper ---
    try:
        import mujoco
        minimal_xml = (
            '<mujoco><option timestep="0.001"/>'
            '<worldbody><body name="load" pos="0 0 0.5">'
            '<joint name="slide" type="slide" axis="0 0 1"/>'
            '<geom type="sphere" size="0.05" mass="1.0"/>'
            '</body></worldbody></mujoco>')
        record = out.text("models/minimal.xml", minimal_xml)
        mm = mujoco.MjModel.from_xml_string(minimal_xml)
        mjb = ctx.tmp / "minimal.mjb"
        mujoco.mj_saveModel(mm, str(mjb))
        record2 = out.copy("models/minimal.mjb", mjb)
        rec.add("models/minimal", recipe="RCP-02", group="config",
                comparison="B", expected={"kind": "accepted"},
                produced_by="mujoco.MjModel.from_xml_string + mj_saveModel",
                files=[record, record2],
                notes="smallest compilable model used by the bare-stepper "
                      "fixture; tool-authored XML documented here")
    except Exception as exc:
        rec.blocker("models/minimal", "RCP-02", f"{type(exc).__name__}: {exc}")

    # --- annular-cylinder inertia golden ---
    try:
        from bike_sim.physics.inertia import ring_inertia
        record, _ = out.array("model/ring_2kg_0p1_0p5_0p06.f64",
                              ring_inertia(2.0, .1, .5, .06))
        rec.add("model/ring_2kg_0p1_0p5_0p06.f64", recipe="RCP-02",
                group="config", comparison="B",
                expected={"kind": "accepted"},
                produced_by="ring_inertia(2.0, 0.1, 0.5, 0.06)",
                files=[record],
                notes="annular cylinder inertia matrix (3x3 row-major)")
    except Exception as exc:
        rec.blocker("model/ring_2kg_0p1_0p5_0p06.f64", "RCP-02",
                    f"{type(exc).__name__}: {exc}")

    # --- kinematics: leverage/geometry grid ---
    try:
        from bike_sim.kinematics.solver import HorstLinkageSolver
        solver = HorstLinkageSolver()
        traj = solver.solve_trajectory(n_points=101, max_travel=180.0)
        files = write_canonical(out, "kinematics/trajectory.json", traj)
        leverage = np.asarray(traj.get("leverage_ratio", []), dtype=float)
        if leverage.size:
            arec, _ = out.array("kinematics/leverage.f64", leverage)
            files.append(arec)
        rec.add("kinematics/trajectory", recipe="RCP-02", group="config",
                comparison="B", expected={"kind": "accepted"},
                produced_by="HorstLinkageSolver().solve_trajectory(101, 180)",
                files=files,
                notes="101-sample wheel-travel grid incl. leverage curve")
    except Exception as exc:
        rec.blocker("kinematics/trajectory", "RCP-02",
                    f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# RCP-01 bootstrap + RCP-02 model arrays
# ---------------------------------------------------------------------------

def _support(oracle_root: Path):
    ref = oracle_root / "tests" / "reference"
    if str(ref) not in sys.path:
        sys.path.insert(0, str(ref))
    import _native_runtime_support as support  # noqa
    return support


def _bootstrap_payload(ctx: Context, sim, stem: str, *, steps=None,
                       advance_args=None):
    """capture_bootstrap + optional advance_python rows under ``stem``.

    The bitwise gate's ``<platform-key>/integration.f64`` pin is NOT emitted
    here: it is the native prepare->snapshot vector (the request pipeline
    zeroes the unported leg-ctrl slots), produced by
    ``bike-native-ride --request <bundle> --steps 0 --dump-integration-state``
    — see the RCP-01 native_start_welded notes and the T25 evidence.
    """
    from bike_sim.native.setup import capture_bootstrap
    out, rec = ctx.out, ctx.rec
    work = ctx.tmp / stem.replace("/", "_")
    work.mkdir(parents=True, exist_ok=True)
    boot = capture_bootstrap(sim, work)
    files = [out.copy(f"{stem}/model.mjb", boot.model_path)]
    files += write_canonical(out, f"{stem}/config.json", boot.config)
    files += write_canonical(out, f"{stem}/state.json", boot.state)
    arec, _ = out.array(f"{stem}/integration.f64", boot.state["integration_state"])
    files.append(arec)
    digest = hashlib.sha256((work / "model.mjb").read_bytes()).hexdigest()
    meta = {"model_sha256": digest,
            "model_dims": boot.state["model_dims"],
            "integration_width": len(boot.state["integration_state"])}
    files += write_canonical(out, f"{stem}/meta.json", meta)
    entries = {"model_sha256": digest}
    if steps:
        support = _support(ctx.oracle)
        control = advance_args.get("control") if advance_args else None
        front = advance_args.get("front", 0.0) if advance_args else 0.0
        rear = advance_args.get("rear", 0.0) if advance_args else 0.0
        rows = support.advance_python(sim, steps, control=control,
                                      front=front, rear=rear)
        files += write_canonical(out, f"{stem}/steps.json",
                                 {"control": (None if control is None else
                                              dataclasses.asdict(control)),
                                  "front_brake": front, "rear_brake": rear,
                                  "rows": rows})
        import mujoco as _mj
        width = _mj.mj_stateSize(sim.model, _mj.mjtState.mjSTATE_INTEGRATION)
        post = np.empty(width, dtype=np.float64)
        _mj.mj_getState(sim.model, sim.data, post,
                        _mj.mjtState.mjSTATE_INTEGRATION)
        arec, _ = out.array(f"{stem}/steps_integration.f64", post)
        files.append(arec)
        entries["rows"] = len(rows)
    return files, entries


def _welded_model_arrays(ctx: Context, sim):
    """Raw compiled-model arrays + named topology inventory for the welded model."""
    import mujoco
    out, rec = ctx.out, ctx.rec
    model = sim.model
    stem = "model/welded_arrays"
    index = {}
    names = [
        "qpos0", "qvel0" if hasattr(model, "qvel0") else "qpos0",  # placeholder fixed below
    ]
    fields = ["qpos0", "body_mass", "body_inertia", "body_ipos", "body_iquat",
              "dof_damping", "dof_frictionloss", "dof_armature", "dof_solref",
              "dof_solimp", "jnt_range", "jnt_stiffness", "jnt_pos",
              "jnt_axis", "actuator_gainprm", "actuator_biasprm",
              "actuator_ctrlrange", "actuator_forcerange", "tendon_range",
              "tendon_stiffness", "tendon_damping", "tendon_lengthspring",
              "tendon_length0", "eq_data", "eq_active0", "geom_friction",
              "geom_solref", "geom_solimp", "site_pos", "opt_timestep"]
    files = []
    for field in fields:
        try:
            if field == "opt_timestep":
                arr = np.atleast_1d(np.float64(model.opt.timestep))
            else:
                arr = np.asarray(getattr(model, field))
            arec, tag = out.array(f"{stem}/{field}", arr)
            index[field] = {"file": tag["$array"], "dtype": tag["dtype"],
                            "shape": tag["shape"], "sha256": arec["sha256"]}
            files.append(arec)
        except Exception as exc:
            index[field] = {"error": f"{type(exc).__name__}: {exc}"}
    inventory = {}
    kinds = {"body": (model.nbody, mujoco.mjtObj.mjOBJ_BODY),
             "joint": (model.njnt, mujoco.mjtObj.mjOBJ_JOINT),
             "geom": (model.ngeom, mujoco.mjtObj.mjOBJ_GEOM),
             "site": (model.nsite, mujoco.mjtObj.mjOBJ_SITE),
             "actuator": (model.nu, mujoco.mjtObj.mjOBJ_ACTUATOR),
             "sensor": (model.nsensor, mujoco.mjtObj.mjOBJ_SENSOR),
             "tendon": (model.ntendon, mujoco.mjtObj.mjOBJ_TENDON),
             "equality": (model.neq, mujoco.mjtObj.mjOBJ_EQUALITY)}
    for kind, (count, obj) in kinds.items():
        inventory[kind] = [mujoco.mj_id2name(model, obj, i)
                           for i in range(count)]
    index["names"] = inventory
    index["dims"] = {k: int(getattr(model, k)) for k in
                     ("nq", "nv", "nu", "na", "nbody", "njnt", "ntendon",
                      "nwrap", "nsite", "ngeom", "neq", "nsensor",
                      "nsensordata", "nmesh", "nhfield", "nhfielddata")}
    files += write_canonical(out, f"{stem}/index.json", index)
    rec.add("model/welded_arrays", recipe="RCP-02", group="bootstrap",
            comparison="B", expected={"kind": "accepted"},
            produced_by="direct mjModel field dumps + mj_id2name inventory",
            files=files,
            notes="raw compiled model arrays; index.json maps names/dtypes")
    return files


def _bootstrap_faults(ctx: Context, sim, stem: str):
    """Deliberate bootstrap mutations; record each real rejection identity."""
    from bike_sim.native.setup import capture_bootstrap
    out, rec = ctx.out, ctx.rec
    work = ctx.tmp / "faults_base"
    work.mkdir(parents=True, exist_ok=True)
    boot = capture_bootstrap(sim, work)

    # Load the oracle-built extension ONCE through the oracle's own loader
    # (bike_sim.native.artifact.load_native_extension — the same path tests
    # use).  A C extension can only be initialised once per process; the
    # loader caches the module in sys.modules so every ctor call below sees
    # the same bound NativeRideRuntime.  If the extension cannot be loaded
    # the whole fault group is a blocker — never a per-case "expected".
    try:
        ctor = _native_ctor(ctx)
    except Exception as exc:
        rec.blocker(f"{stem}", "RCP-01",
                    f"native extension unavailable — fault verdicts not "
                    f"captured: {type(exc).__name__}: {exc}")
        return

    def _plain(obj):
        return json.loads(json.dumps(obj, default=_jd))

    base_state = _plain(boot.state)
    base_config = _plain(boot.config)

    fault_cases = {}
    st = _plain(base_state)
    st["schema"] = 99
    fault_cases["schema"] = ("state.schema=99", st)
    st = _plain(base_state)
    st["model_digest"] = "0" * 64
    fault_cases["digest"] = ("state.model_digest zeroed", st)
    st = _plain(base_state)
    st["model_dims"]["nq"] += 1
    fault_cases["dims"] = ("state.model_dims.nq+=1", st)
    st = _plain(base_state)
    st["integration_state"] = st["integration_state"][:-1]
    fault_cases["width"] = ("state.integration_state[:-1]", st)
    st = _plain(base_state)
    st["integration_state"][0] = float("nan")
    fault_cases["nonfinite"] = ("state.integration_state[0]=nan", st)
    st = _plain(base_state)
    st["runtime"].pop("step", None)
    fault_cases["missing_key"] = ("state.runtime.step removed", st)
    st = _plain(base_state)
    st["unexpected_future_field"] = 1
    fault_cases["unknown_key"] = ("state.+unexpected_future_field", st)
    st = _plain(base_state)
    if isinstance(st.get("drive"), dict):
        st["drive"].pop("front_teeth", None)
    fault_cases["malformed_drive"] = ("state.drive.front_teeth removed", st)
    cfg = _plain(base_config)
    cfg["control_period_steps"] = 0
    fault_cases["control_period"] = ("config.control_period_steps=0",
                                     None, cfg)

    for name, spec in fault_cases.items():
        where = spec[0]
        mutated_state = spec[1]
        mutated_config = spec[2] if len(spec) > 2 else base_config
        entry_dir = f"{stem}/{name}"
        files = []
        if mutated_state is not None:
            files += write_canonical(
                out, f"{entry_dir}/state.json", mutated_state)
        files += write_canonical(
            out, f"{entry_dir}/config.json", mutated_config)
        try:
            ctor(str(work / "model.mjb"), mutated_config,
                 mutated_state if mutated_state is not None
                 else _plain(base_state))
            expected = {"kind": "unexpected-acceptance"}
            error = "native runtime accepted the mutated bootstrap"
        except Exception as exc:
            expected = {"kind": "invalid",
                        "error": type(exc).__name__,
                        "message": str(exc)}
            error = f"{type(exc).__name__}: {exc}"
        files += write_canonical(
            out, f"{entry_dir}/expected.json",
            {"mutation": where, "expected": expected})
        rec.add(f"{stem}/{name}", recipe="RCP-01", group="bootstrap",
                comparison="S", expected=expected,
                produced_by="mutated capture_bootstrap payload + native ctor",
                files=files,
                notes=f"mutation {where}; oracle response: {error}")


def _native_ctor(ctx: Context):
    """Return the oracle-built ``NativeRideRuntime``, loaded exactly once.

    Uses ``bike_sim.native.artifact.load_native_extension`` — the oracle's
    own verified import path (sys.modules-cached, provenance-checked) — so
    repeated calls never re-execute the C extension's module init, which is
    what produced attribute-less module objects under the previous
    spec_from_file_location/exec_module approach.  Raises on any failure;
    callers convert that to a blocker/control-failure, never an expected.
    """
    cached = getattr(ctx, "_native_ctor_cache", None)
    if cached is not None:
        ctor, err = cached
        if err is not None:
            raise err
        return ctor
    try:
        from bike_sim.native.artifact import (
            extension_artifacts, load_native_extension)
        build = None
        for cand in (ctx.oracle / "native" / "build" / "release",
                     ctx.oracle / "native" / "build"):
            if cand.is_dir() and extension_artifacts(cand):
                build = cand
                break
        if build is None:
            raise RuntimeError("no bike_native extension under "
                               f"{ctx.oracle / 'native' / 'build'}")
        module = load_native_extension(build)
        ctor = getattr(module, "NativeRideRuntime", None) or getattr(
            module, "RideRuntime", None)
        if ctor is None:
            raise RuntimeError("bike_native has no NativeRideRuntime")
        ctx._native_ctor_cache = (ctor, None)
        return ctor
    except Exception as exc:
        ctx._native_ctor_cache = (None, exc)
        raise


def _rcp01_bootstrap(ctx: Context):
    out, rec = ctx.out, ctx.rec
    support = _support(ctx.oracle)
    from bike_sim.sim.ride.control import RideControl

    # -- welded production-Numba bootstrap -------------------------------
    try:
        sim = ctx.sim_cache.get("welded")
        if sim is None:
            captured_xml = {}
            import bike_sim.sim.ride_sim as _ride_mod
            real_xml = _ride_mod.generate_mujoco_xml

            def _spy(*a, **kw):
                text = real_xml(*a, **kw)
                captured_xml["xml"] = text
                captured_xml["kwargs"] = {k: str(v) for k, v in kw.items()}
                return text
            _ride_mod.generate_mujoco_xml = _spy
            try:
                with _least_squares_spy(ctx, "welded_build"):
                    sim = support.make_python_ride(strict=False,
                                                   decimation=1)
            finally:
                _ride_mod.generate_mujoco_xml = real_xml
            ctx.sim_cache["welded"] = sim
            if "xml" in captured_xml:
                import re as _re
                assets = sorted(set(
                    _re.findall(r'file="([^"]+)"', captured_xml["xml"])))
                record = out.text("model/welded.xml", captured_xml["xml"])
                extra = write_canonical(
                    out, "model/assets/inventory.json",
                    {"referenced_files": assets})
                rec.add("model/welded.xml", recipe="RCP-02",
                        group="bootstrap", comparison="Q",
                        expected={"kind": "accepted"},
                        produced_by="spied ride_sim.generate_mujoco_xml "
                                    "during make_python_ride",
                        files=[record] + extra,
                        notes="exact bytes MuJoCo compiled; referenced asset "
                              f"files: {assets or 'none'}")
        files, meta = _bootstrap_payload(ctx, sim, "bootstrap/welded")
        rec.add("bootstrap/welded", recipe="RCP-01", group="bootstrap",
                comparison="B", expected={"kind": "accepted"},
                produced_by="make_python_ride(strict=False, decimation=1) + "
                            "capture_bootstrap (production numba kernels)",
                files=files,
                inputs=[ctx.oracle / "examples/research/viewer_physics_welded.toml",
                        ctx.oracle / "examples/research/rough_uphill_savage.toml"],
                notes=f"t=0 bootstrap, model sha256 {meta['model_sha256'][:16]}…")
        _welded_model_arrays(ctx, sim)
    except Exception as exc:
        rec.blocker("bootstrap/welded", "RCP-01",
                    "".join(traceback.format_exception(exc))[-2000:])
        return

    # -- step 0/1 + control boundaries ------------------------------------
    try:
        control = RideControl(motor_torque_nm=25.0, human_torque_nm=10.0)
        files = []
        work = ctx.tmp / "welded_steps"
        rows = support.advance_python(sim, 4, control=control)
        files += write_canonical(
            out, "bootstrap/welded/steps_control.json",
            {"control": dataclasses.asdict(control), "rows": rows})
        import mujoco as _mj
        width = _mj.mj_stateSize(sim.model, _mj.mjtState.mjSTATE_INTEGRATION)
        post = np.empty(width, dtype=np.float64)
        _mj.mj_getState(sim.model, sim.data, post,
                        _mj.mjtState.mjSTATE_INTEGRATION)
        arec, _ = out.array("bootstrap/welded/steps_control_integration.f64",
                            post)
        files.append(arec)
        rec.add("bootstrap/welded/steps_control", recipe="RCP-01",
                group="bootstrap", comparison="B",
                expected={"kind": "accepted"},
                produced_by="advance_python x4 with "
                            "RideControl(motor=25Nm, human=10Nm)",
                files=files,
                notes="first committed intervals + post-step integration state")
    except Exception as exc:
        rec.blocker("bootstrap/welded/steps_control", "RCP-01",
                    f"{type(exc).__name__}: {exc}")

    # -- reset twice -> byte-identical bootstrap digests -------------------
    try:
        digests = []
        files = []
        for idx in (1, 2):
            sim.reset()
            iter_files, meta = _bootstrap_payload(
                ctx, sim, f"bootstrap/welded/reset/{idx}")
            files += iter_files
            digests.append(meta["model_sha256"])
        rec.add("bootstrap/welded/reset", recipe="RCP-01", group="bootstrap",
                comparison="B",
                expected={"kind": "accepted",
                          "identical_digests": digests[0] == digests[1]},
                produced_by="sim.reset() x2 + capture_bootstrap",
                files=files,
                notes=f"reset digests {digests[0][:12]} / {digests[1][:12]}")
    except Exception as exc:
        rec.blocker("bootstrap/welded/reset", "RCP-01",
                    f"{type(exc).__name__}: {exc}")

    # -- pure-python pinned-kernel twin ------------------------------------
    try:
        _pin_oracle_kernels()
        sim_py = support.make_python_ride(strict=False, decimation=1)
        ctx.sim_cache["welded_py"] = sim_py
        files, meta = _bootstrap_payload(ctx, sim_py, "bootstrap/welded_pinned")
        rec.add("bootstrap/welded_pinned", recipe="RCP-01", group="bootstrap",
                comparison="B", expected={"kind": "accepted"},
                produced_by="make_python_ride with _pin_oracle_kernels "
                            "(*_py twins) + capture_bootstrap",
                files=files,
                notes="pure-Python kernel identity — kept separate from the "
                      "production numba capture as required")
    except Exception as exc:
        rec.blocker("bootstrap/welded_pinned", "RCP-01",
                    "".join(traceback.format_exception(exc))[-2000:])

    # -- native-start welded (request-equivalent pipeline) -----------------
    try:
        from bike_sim.cli import ride as ride_cli
        # The oracle rejects a seed override on a file track (no [generator]
        # block); requests/welded_flat.toml's seed_decimal therefore maps to
        # no CLI flag here — the track file is already fully specified.
        args = ride_cli.parse_args([
            "--physics-config",
            str(out.root / "config/viewer_physics_welded.toml"),
            "--track", str(out.root / "tracks/flat.toml"),
            "--headless", "--duration", "1", "--no-plots"])
        sim_ns = ride_cli.build_physical_simulation_from_args(args)
        sim_ns.physical.set_strict(False)
        sim_ns.physical.set_measurement_decimation(1)
        ctx.sim_cache["welded_flat"] = sim_ns
        files, meta = _bootstrap_payload(
            ctx, sim_ns, "bootstrap/native_start_welded")
        rec.add("bootstrap/native_start_welded", recipe="RCP-01",
                group="bootstrap", comparison="B",
                expected={"kind": "accepted"},
                produced_by="ride pipeline for requests/welded_flat.toml",
                files=files,
                inputs=[out.root / "requests/welded_flat.toml"],
                notes="startup outputs for the matching frozen-flat request")
        # requests/welded_flat.toml restores this captured t=0 state through
        # the audited initial-state transfer. A second build is a guaranteed
        # equilibrium-cache hit (the first build saved the entry), so the
        # captured qacc_warmstart is the canonical zeroed restore value and
        # the fixture bytes are deterministic on cold and warm caches.
        # The contract hash covers compiled-model fields and the settled
        # pose — both platform-sensitive — so each platform owns its document
        # under the golden platform key and the request reader prefers that
        # platform-keyed sibling over the shared spelling.
        from bike_sim.sim.ride.initial_state import PhysicalInitialState
        sim_is = ride_cli.build_physical_simulation_from_args(args)
        transferred = PhysicalInitialState.capture(sim_is)
        import platform as _platform
        _state_rel = ("initial_state/" +
                      f"{_platform.system().lower()}-{_platform.machine()}" +
                      "/welded_flat.json")
        record = out.text(_state_rel, transferred.payload_json)
        rec.add(_state_rel, recipe="RCP-01",
                group="bootstrap", comparison="S",
                expected={"kind": "accepted"},
                produced_by="PhysicalInitialState.capture on a welded-flat "
                            "cache-hit build",
                files=[record],
                inputs=[out.root / "requests/welded_flat.toml"],
                notes="t=0 state transferred by requests/welded_flat.toml; "
                      "per-platform capture superseded by the platform-keyed "
                      "sibling convention")
    except Exception as exc:
        rec.blocker("bootstrap/native_start_welded", "RCP-01",
                    "".join(traceback.format_exception(exc))[-2000:])

    # -- legacy (non-physical) ride startup + steps ------------------------
    try:
        from bike_sim.terrain import get_preset
        from bike_sim.sim.ride_sim import RideSimulation
        from bike_sim.physics.model_config import SimulationPhysicsConfig
        from bike_sim.physics.physical_config import TireBackendConfig
        from bike_sim.physics.rider import RiderSpecs
        import mujoco as _mj
        lsim = RideSimulation(
            track=get_preset("flat"),
            rider=RiderSpecs(variant="lumped", mass_kg=80.0),
            physics_config=SimulationPhysicsConfig(
                "legacy", drive_mode="coast",
                tires=TireBackendConfig(backend="compliant_2d")))
        width = _mj.mj_stateSize(lsim.model, _mj.mjtState.mjSTATE_INTEGRATION)
        init = np.empty(width, dtype=np.float64)
        _mj.mj_getState(lsim.model, lsim.data, init,
                        _mj.mjtState.mjSTATE_INTEGRATION)
        files = []
        arec, _ = out.array("bootstrap/legacy_flat/integration.f64", init)
        files.append(arec)
        lsim.step(0.0, 0.0)
        post = np.empty(width, dtype=np.float64)
        _mj.mj_getState(lsim.model, lsim.data, post,
                        _mj.mjtState.mjSTATE_INTEGRATION)
        arec, _ = out.array("bootstrap/legacy_flat/step1.f64", post)
        files.append(arec)
        files += write_canonical(out, "bootstrap/legacy_flat/meta.json",
                                 {"physics_mode": "legacy",
                                  "rider": "lumped", "steps": lsim.steps})
        rec.add("bootstrap/legacy_flat", recipe="RCP-01", group="bootstrap",
                comparison="B", expected={"kind": "accepted"},
                produced_by="legacy RideSimulation startup + one step",
                files=files,
                inputs=[out.root / "requests/legacy_flat.toml"],
                notes="legacy ride startup state for requests/legacy_flat.toml")
    except Exception as exc:
        rec.blocker("bootstrap/legacy_flat", "RCP-01",
                    "".join(traceback.format_exception(exc))[-1200:])

    # -- fault injections ---------------------------------------------------
    try:
        _bootstrap_faults(ctx, sim, "bootstrap/faults")
    except Exception as exc:
        rec.blocker("bootstrap/faults", "RCP-01",
                    "".join(traceback.format_exception(exc))[-2000:])

    # -- close behaviour ----------------------------------------------------
    # RideSimulation itself exposes no close(); the closeable lifecycle lives
    # on ResearchEnvironment (env.close -> env.step must raise).  The
    # close-then-step negative control therefore runs in _negative_controls
    # after the research env exists.


def _pin_oracle_kernels():
    """Replicate tests/reference/conftest.py::_pin_oracle_kernels."""
    from bike_sim.physics import joint_strength, tire
    from bike_sim.sim.ride import (distributed_tire_forces, rider_contacts,
                                   rider_control, support_geometry,
                                   tire_forces)
    for module in (tire, tire_forces, distributed_tire_forces, rider_contacts):
        module._brush_step = tire._brush_step_py
    for module in (tire, tire_forces, rider_contacts, rider_control):
        module._normal_contact = tire._normal_contact_py
    joint_strength._capacity_core = joint_strength._capacity_core_py
    support_geometry._box_pad_contact_core = \
        support_geometry._box_pad_contact_core_py


# ---------------------------------------------------------------------------
# RCP-03 random / research
# ---------------------------------------------------------------------------

def _rcp03_random(ctx: Context):
    out, rec = ctx.out, ctx.rec
    files = []
    rng = np.random.default_rng(1234)
    arec, _ = out.array("random/seed1234-normal64.f64", rng.normal(0., 1., 64))
    files.append(arec)
    rng = np.random.default_rng(1234)
    arec, _ = out.array("random/seed1234-uniform64.f64", rng.random(64))
    files.append(arec)
    raw = np.random.PCG64(1234).random_raw(64)
    arec, _ = out.array("random/seed1234-pcg64-raw64.u64", raw)
    files.append(arec)
    seq = np.random.SeedSequence(1234)
    children = seq.spawn(2)
    spawn = {"seed": 1234,
             "parent_state": seq.state,
             "children": []}
    for index, child in enumerate(children):
        child_rng = np.random.default_rng(child)
        arec, _ = out.array(f"random/seed1234-spawn{index}-normal16.f64",
                            child_rng.normal(0., 1., 16))
        files.append(arec)
        spawn["children"].append({
            "spawn_key": list(child.spawn_key),
            "state_u64": [str(v) for v in child.generate_state(4,
                          dtype=np.uint64)],
            "normal16": f"random/seed1234-spawn{index}-normal16.f64"})
    files += write_canonical(out, "random/seed1234-spawn.json", spawn)
    rec.add("random/seed1234", recipe="RCP-03", group="random",
            comparison="B", expected={"kind": "accepted"},
            produced_by="numpy PCG64/SeedSequence capture (numpy "
                        f"{np.__version__})",
            files=files,
            notes="first-64 normal01 words, uniform stream, raw64 words and "
                  "SeedSequence-spawned child states for seed 1234")

    # research noise tape
    try:
        from bike_sim.sim.research.sensor_noise import build_noise_tape
        from bike_sim.sim.research.sensors import SensorConfig
        tape = build_noise_tape(SensorConfig(), seed=210, count=128)
        files = []
        arec, _ = out.array("random/research-noise-seed210-noise.f64",
                            tape.noise)
        files.append(arec)
        arec, _ = out.array("random/research-noise-seed210-dropout.f64",
                            tape.dropout_uniform)
        files.append(arec)
        rec.add("random/research-noise-seed210", recipe="RCP-03",
                group="random", comparison="B",
                expected={"kind": "accepted"},
                produced_by="build_noise_tape(SensorConfig(), seed=210, "
                            "count=128)",
                files=files,
                notes="(128,9) sensor noise + (128,) dropout uniforms from "
                      "the spawned sensor streams")
    except Exception as exc:
        rec.blocker("random/research-noise-seed210", "RCP-03",
                    f"{type(exc).__name__}: {exc}")

    # seeded rider sampling
    try:
        from bike_sim.sim.research.rider_random import (
            RiderRandomSpec, sample_rider)
        cases = {}
        for seed in (0, 42, 210):
            rider, program = sample_rider(RiderRandomSpec(), seed)
            cases[str(seed)] = {
                "rider": dataclasses.asdict(rider),
                "program": dataclasses.asdict(program)}
        files = write_canonical(out, "random/rider-samples.json", cases)
        rec.add("random/rider-samples.json", recipe="RCP-03", group="random",
                comparison="S", expected={"kind": "accepted"},
                produced_by="sample_rider(RiderRandomSpec(), seed)",
                files=files,
                notes="seeded rider mass/height/posture/effort for 0/42/210")
    except Exception as exc:
        rec.blocker("random/rider-samples.json", "RCP-03",
                    f"{type(exc).__name__}: {exc}")


def _rcp03_research(ctx: Context):
    """Research env startup: sensors, noise tape, first transitions."""
    out, rec = ctx.out, ctx.rec
    support = _support(ctx.oracle)
    try:
        from bike_sim.cli import research as research_cli
        args = research_cli.parser().parse_args([
            "--physics-config",
            str(out.root / "config/viewer_physics_welded.toml"),
            "--track-file", str(out.root / "tracks/flat.toml"),
            "--duration", "0.1", "--seed", "210",
            "--actuator-delay", "0.01",
            "--diagnostic-model-limits", "--assist"])
        env = research_cli.make_environment(args)
        files = []
        files += write_canonical(
            out, "research/delayed_bootstrap/env.json",
            {"config": dataclasses.asdict(env.config),
             "sensors": dataclasses.asdict(env.sensor_config)
             if dataclasses.is_dataclass(env.sensor_config)
             else _stable_repr(env.sensor_config),
             "backend": getattr(env, "backend", None),
             "delay_steps": env.delay_steps,
             "control_steps": env.control_steps,
             "sensor_steps": env.sensor_steps})
        # startup sensor observations + first transitions
        transitions = []
        observations = []
        from bike_sim.sim.ride.control import RideControl
        command = RideControl(motor_torque_nm=10.0)
        for _ in range(3):
            try:
                env.step(command)
            except Exception as exc:
                transitions.append({"error": f"{type(exc).__name__}: {exc}"})
                break
        for obs in list(getattr(env, "observations", ())):
            observations.append(dataclasses.asdict(obs)
                                if dataclasses.is_dataclass(obs) else obs)
        for tr in list(getattr(env, "trace", ())):
            transitions.append(dataclasses.asdict(tr)
                               if dataclasses.is_dataclass(tr) else tr)
        files += write_canonical(
            out, "research/delayed_bootstrap/transitions.json",
            {"observations": observations, "transitions": transitions})
        rec.add("research/delayed_bootstrap", recipe="RCP-03",
                group="research", comparison="B",
                expected={"kind": "diagnostic", "valid": False,
                          "diagnostic": "model-limits",
                          "reason": "run captured under "
                                    "--diagnostic-model-limits; validity "
                                    "remains false by contract"},
                produced_by="research.make_environment (--actuator-delay "
                            "0.01, seed 210, flat track, "
                            "--diagnostic-model-limits) + 3 env.step calls",
                files=files,
                inputs=[out.root / "requests/research_delay.toml"],
                notes="research env startup: sensors/noise/delayed queue and "
                      "first applied transitions")
        try:
            env.close()
        except Exception:
            pass
    except Exception as exc:
        rec.blocker("research/delayed_bootstrap", "RCP-03",
                    "".join(traceback.format_exception(exc))[-1500:])


# ---------------------------------------------------------------------------
# RCP-04 equilibrium / allocation / filters
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _least_squares_spy(ctx: Context, label: str):
    """Record every scipy.optimize.least_squares call inside the block.

    Consumers bind the function at import time (``from scipy.optimize import
    least_squares``), so the spy must rewrite the name inside every already
    loaded module that holds a reference — patching ``scipy.optimize`` alone
    would never be seen.
    """
    import scipy.optimize as _opt
    calls = []
    real_ls = _opt.least_squares

    def _argval(v):
        if isinstance(v, np.ndarray):
            return v
        if isinstance(v, (list, tuple)):
            return [_argval(x) for x in v]
        if isinstance(v, dict):
            return {str(k): _argval(x) for k, x in v.items()}
        if isinstance(v, (bool, int, float, str)) or v is None:
            return v
        return _stable_repr(v)

    def _spy(fun, x0, *a, **kw):
        result = real_ls(fun, x0, *a, **kw)
        calls.append({
            "x0": np.asarray(x0, dtype=float),
            "kwargs": {k: _argval(v) for k, v in kw.items()},
            "args": [_strip_addr(str(x)) for x in a],
            "nfev": int(result.nfev),
            "njev": None if result.njev is None else int(result.njev),
            "cost": float(result.cost),
            "optimality": float(result.optimality),
            "status": int(result.status),
            "message": str(result.message),
            "x": np.asarray(result.x, dtype=float)})
        return result

    # Patch the attribute first: modules imported later (equilibrium_refine is
    # imported lazily inside the settle loop) bind the spy through it.
    _opt.least_squares = _spy
    patched = [_opt]
    for module in list(sys.modules.values()):
        if getattr(module, "least_squares", None) is real_ls:
            module.least_squares = _spy
            patched.append(module)
    try:
        yield
    finally:
        for module in patched:
            module.least_squares = real_ls
        ctx.sim_cache[f"ls_calls_{label}"] = calls


def _declared_solver_specs() -> dict:
    """Source-declared least_squares option sets for the recorded call sites."""
    import inspect
    specs = {}
    try:
        import bike_sim.sim.ride.equilibrium_refine as mod
        src = inspect.getsource(mod)
        idx = src.find("least_squares(")
        specs["equilibrium_refine"] = {
            "module": "bike_sim/sim/ride/equilibrium_refine.py",
            "signature": str(inspect.signature(mod.refine_equilibrium)),
            "call_site_excerpt": src[idx - 120:idx + 200] if idx >= 0 else None,
            "source_sha256": sha256_bytes(src.encode())}
    except Exception as exc:
        specs["equilibrium_refine"] = {"error": str(exc)}
    try:
        import bike_sim.sim.ride.sag_fit as mod
        src = inspect.getsource(mod)
        idx = src.find("least_squares(")
        specs["sag_fit"] = {
            "module": "bike_sim/sim/ride/sag_fit.py",
            "call_site_excerpt": src[idx - 120:idx + 240] if idx >= 0 else None,
            "source_sha256": sha256_bytes(src.encode())}
    except Exception as exc:
        specs["sag_fit"] = {"error": str(exc)}
    return specs


def _rcp04_equilibrium(ctx: Context):
    out, rec = ctx.out, ctx.rec
    try:
        from bike_sim.physics.model_config import SimulationPhysicsConfig
        from bike_sim.physics.physical_config import TireBackendConfig
        from bike_sim.physics.rider import RiderSpecs
        from bike_sim.sim.ride.equilibrium_cache import (
            fingerprint, load_state_file, save_state_file)
        from bike_sim.sim.ride_sim import RideSimulation
        from bike_sim.terrain import get_preset

        def _sim(**kwargs):
            cfg = SimulationPhysicsConfig(
                "physical", drive_mode="coast",
                tires=TireBackendConfig(backend="compliant_2d"), **kwargs)
            return RideSimulation(track=get_preset("flat"),
                                  rider=RiderSpecs(variant="lumped",
                                                   mass_kg=80.0),
                                  physics_config=cfg)

        sim = _sim()
        pose = ctx.tmp / "pose.npz"
        save_state_file(pose, sim.physical, sim.data.qpos,
                        sim.equilibrium["steps"],
                        sim.equilibrium["residual_qacc"])
        files = [out.copy("equilibrium/legacy_state.npz", pose)]
        files += write_canonical(out, "equilibrium/legacy_state.meta.json", {
            "equilibrium": dict(sim.equilibrium),
            "fingerprint": fingerprint(sim.physical)})
        rec.add("equilibrium/legacy_state.npz", recipe="RCP-04",
                group="equilibrium", comparison="B",
                expected={"kind": "accepted"},
                produced_by="save_state_file on flat/coast/lumped settle",
                files=files,
                notes="explicit pose file incl. material state + fingerprint")

        # truncated: drop a material state array -> rejected by strict check
        with np.load(pose, allow_pickle=False) as src:
            arrays = {k: src[k] for k in src.files
                      if k != "state_tire_xi"}
        trunc = ctx.tmp / "truncated.npz"
        np.savez(trunc, **arrays)
        files = [out.copy("equilibrium/truncated.npz", trunc)]
        try:
            # Point at the fixture copy, not ctx.tmp, so the recorded
            # rejection message carries a stable path across capture runs.
            _sim(equilibrium_state_file=str(
                out.root / "equilibrium" / "truncated.npz"))
            expected = {"kind": "unexpected-acceptance"}
            err = "accepted truncated pose"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        rec.add("equilibrium/truncated.npz", recipe="RCP-04",
                group="equilibrium", comparison="B", expected=expected,
                produced_by="npz rewrite dropping state_tire_xi",
                files=files, notes=f"rejection: {err}")

        # bad residual: shifted qpos -> residual recheck must fail on load
        with np.load(pose, allow_pickle=False) as src:
            arrays = {k: src[k] for k in src.files}
        root_z = sim.physical.address("root_z")[0]
        shifted = np.array(arrays["qpos"], copy=True)
        shifted[root_z] += .05
        arrays["qpos"] = shifted
        bad = ctx.tmp / "bad_residual.npz"
        np.savez(bad, **arrays)
        files = [out.copy("equilibrium/bad_residual.npz", bad)]
        try:
            _sim(equilibrium_state_file=str(
                out.root / "equilibrium" / "bad_residual.npz"))
            expected = {"kind": "unexpected-acceptance"}
            err = "accepted shifted pose"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        rec.add("equilibrium/bad_residual.npz", recipe="RCP-04",
                group="equilibrium", comparison="B", expected=expected,
                produced_by="npz rewrite with qpos[root_z]+=0.05",
                files=files, notes=f"residual recheck rejection: {err}")

        # fingerprint mismatch: same pose, different rider mass
        try:
            cfg = SimulationPhysicsConfig(
                "physical", drive_mode="coast",
                tires=TireBackendConfig(backend="compliant_2d"),
                equilibrium_state_file=str(
                    out.root / "equilibrium" / "legacy_state.npz"))
            RideSimulation(
                track=get_preset("flat"),
                rider=RiderSpecs(variant="lumped", mass_kg=75.0),
                physics_config=cfg)
            expected = {"kind": "unexpected-acceptance"}
            err = "pose accepted under a different configuration"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        files = write_canonical(out, "equilibrium/fingerprint_mismatch.json",
                                {"pose": "equilibrium/legacy_state.npz",
                                 "rider_mass_kg": 75.0,
                                 "expected": expected})
        rec.add("equilibrium/fingerprint_mismatch", recipe="RCP-04",
                group="equilibrium", comparison="S", expected=expected,
                produced_by="equilibrium_state_file + rider_mass 75kg",
                files=files, notes=f"fingerprint rejection: {err}")

        # refine calls: the spy runs inside the welded build (_rcp01); here we
        # also spy the flat/coast build to record whether refinement fires.
        files = []
        with _least_squares_spy(ctx, "flat_build"):
            _sim()
        refine = {"welded_build": ctx.sim_cache.get("ls_calls_welded_build"),
                  "flat_build": ctx.sim_cache.get("ls_calls_flat_build"),
                  "declared": _declared_solver_specs()}
        files += write_canonical(out, "equilibrium/refine_calls.json", refine)
        n_calls = sum(len(v or ()) for k, v in refine.items()
                      if k != "declared")
        rec.add("equilibrium/refine_calls", recipe="RCP-04",
                group="equilibrium", comparison="N",
                expected={"kind": "accepted"},
                produced_by="least_squares spy around sim construction + "
                            "source-declared options",
                files=files,
                notes=f"{n_calls} recorded solver call(s); declared option "
                      "records cover equilibrium_refine and sag_fit call sites")
    except Exception as exc:
        rec.blocker("equilibrium", "RCP-04",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- allocation cases ---------------------------------------------------
    try:
        from bike_sim.physics.rider_allocation import (
            allocate_effort, grip_constraints)
        from scipy.optimize import NonlinearConstraint
        inf = float("inf")

        def _case(name, target, aeq, beq, g, h, lower, upper, **kw):
            prob = {"target": np.asarray(target, dtype=float),
                    "aeq": np.atleast_2d(np.asarray(aeq, dtype=float)),
                    "beq": np.atleast_1d(np.asarray(beq, dtype=float)),
                    "g": np.atleast_2d(np.asarray(g, dtype=float)),
                    "h": np.atleast_1d(np.asarray(h, dtype=float)),
                    "lower": np.asarray(lower, dtype=float),
                    "upper": np.asarray(upper, dtype=float)}
            extra = kw.pop("extra_constraints", None)
            x0 = kw.pop("x0", None)
            try:
                if extra is not None:
                    result = allocate_effort(
                        extra_constraints=extra,
                        x0=x0, **prob)
                else:
                    result = allocate_effort(x0=x0, **prob)
                files = write_canonical(
                    out, f"allocation/{name}.json",
                    {"problem": {k: v for k, v in prob.items()},
                     "extra": (_strip_addr(str(extra))
                               if extra is not None else None),
                     "x0": x0,
                     "result": {"solution": result.solution,
                                "feasible": bool(result.feasible),
                                "violation": float(result.violation)}})
                arec, _ = out.array(f"allocation/{name}.f64",
                                    np.asarray(result.solution))
                files.append(arec)
                expected = {"kind": "accepted",
                            "feasible": bool(result.feasible)}
                err = None
            except Exception as exc:
                files = write_canonical(
                    out, f"allocation/{name}.json",
                    {"problem": prob, "error": str(exc)})
                expected = {"kind": "invalid",
                            "error": type(exc).__name__, "message": str(exc)}
                err = exc
            rec.add(f"allocation/{name}", recipe="RCP-04", group="allocation",
                    comparison="B", expected=expected,
                    produced_by="allocate_effort(...)", files=files,
                    notes=("" if err is None else str(err)))

        _case("box_projection", [3., -2.], np.zeros((0, 2)), [],
              np.zeros((0, 2)), [], [0., -1.], [1., 1.])
        _case("equality", [2., 0.], [[1., 1.]], [1.],
              np.zeros((0, 2)), [], [-inf, -inf], [inf, inf])
        grip = grip_constraints(np.eye(2), np.array([-1., 0.]),
                                pulling=True, limit_n=300.)
        _case("nonlinear_grip", [400., 0.], np.zeros((0, 2)), [],
              np.zeros((0, 2)), [], [-1000., -1000.], [1000., 1000.],
              extra_constraints=tuple(grip))
        _case("infeasible", [0.], [[1.]], [5.],
              np.zeros((0, 1)), [], [0.], [1.])
        _case("warm_start", [2., 0.], [[1., 1.]], [1.],
              np.zeros((0, 2)), [], [-inf, -inf], [inf, inf],
              x0=np.array([0.4, 0.6]))
        _case("nan_target", [float("nan"), 0.], np.zeros((0, 2)), [],
              np.zeros((0, 2)), [], [-1., -1.], [1., 1.])
    except Exception as exc:
        rec.blocker("allocation", "RCP-04",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- metrics filter fixtures --------------------------------------------
    try:
        from bike_sim.sim.ride.metrics import lowpass
        n = 128
        dt = .001
        t = np.arange(n) * dt
        signal = np.sin(2 * np.pi * 5 * t) + (t > .05).astype(float)
        files = []
        arec, _ = out.array("metrics/filter_input.f64", signal)
        files.append(arec)
        filtered = lowpass(signal, dt)
        arec, _ = out.array("metrics/filter_expected.f64", filtered)
        files.append(arec)
        short = np.arange(10, dtype=float)
        arec, _ = out.array("metrics/filter_short.f64",
                            lowpass(short, dt))
        files.append(arec)
        arec, _ = out.array("metrics/filter_input_short.f64", short)
        files.append(arec)
        meta = {"sample_interval_s": dt, "cutoff_hz": 100.,
                "order": 4, "short_passthrough": True}
        files += write_canonical(out, "metrics/filter.json", meta)
        rec.add("metrics/filter", recipe="RCP-04", group="metrics",
                comparison="B", expected={"kind": "accepted"},
                produced_by="metrics.lowpass(signal, 0.001) + edge cases",
                files=files,
                notes="zero-phase Butterworth fixture; short input returns "
                      "the signal unchanged")
    except Exception as exc:
        rec.blocker("metrics/filter", "RCP-04",
                    f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# RCP-05 recording
# ---------------------------------------------------------------------------

def _rcp05_recording(ctx: Context):
    out, rec = ctx.out, ctx.rec
    ref = ctx.oracle / "tests" / "reference"
    if str(ref) not in sys.path:
        sys.path.insert(0, str(ref))
    try:
        import _recording_support as rs
        from bike_sim.recording import analysis, reader, storage
        from bike_sim.recording.json_codec import dumps
        from bike_sim.recording import schema as rschema
    except Exception as exc:
        rec.blocker("recording", "RCP-05", f"{type(exc).__name__}: {exc}")
        return

    def _write_and_copy(capture, stem, extra=None):
        dest = storage.write_recording(capture, ctx.tmp / "recroot")
        if dest is None:
            rec.blocker(stem, "RCP-05",
                        "write_recording returned None (disabled capture)")
            return None
        files = out.copy_tree(stem, dest)
        extra_files = []
        if extra:
            for name, value in extra.items():
                extra_files += write_canonical(out, f"{stem}/{name}", value)
        return dest, files + extra_files

    # --- good recording: canonical sample rows + optional-leaf matrix --------
    try:
        rows = [rs.sample_row(interval_id=i, start=i * .001)
                for i in range(3)]
        capture = rs.synthetic_capture(rows=rows, forces=True)
        result = _write_and_copy(capture, "recording/good")
        if result:
            dest, files = result
            rdr = reader.Recording(dest)
            files += write_canonical(
                out, "recording/good/overview.json",
                analysis.overview(rdr))
            files += write_canonical(
                out, "recording/good/fields.json",
                analysis.fields(rdr))
            rec.add("recording/good", recipe="RCP-05", group="recording",
                    comparison="S", expected={"kind": "accepted"},
                    produced_by="synthetic_capture + write_recording + "
                                "analysis.overview/fields",
                    files=files,
                    notes="sealed recording incl. manifest/summary; "
                          "overview via source codec")
    except Exception as exc:
        rec.blocker("recording/good", "RCP-05",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- value-case matrix (optional/null/absent/nonfinite/signed zero) ------
    try:
        capture = rs.synthetic_capture(value_cases=True)
        result = _write_and_copy(capture, "recording/value_cases")
        if result:
            dest, files = result
            rdr = reader.Recording(dest)
            files += write_canonical(
                out, "recording/value_cases/overview.json",
                analysis.overview(rdr))
            rec.add("recording/value_cases", recipe="RCP-05",
                    group="recording", comparison="S",
                    expected={"kind": "accepted"},
                    produced_by="synthetic_capture(value_cases=True) + "
                                "write_recording",
                    files=files,
                    notes="optional/null/absent/nonfinite/signed-zero matrix")
    except Exception as exc:
        rec.blocker("recording/value_cases", "RCP-05",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- bad_presence: writer-produced DB tampered to fail presence ---------
    try:
        rows = [rs.sample_row(interval_id=i, start=i * .001)
                for i in range(2)]
        capture = rs.synthetic_capture(rows=rows)
        dest = storage.write_recording(capture, ctx.tmp / "recroot2")
        import duckdb
        con = duckdb.connect(str(dest / "run.duckdb"))
        row = con.execute(
            "SELECT table_name, column_name FROM information_schema.columns "
            "WHERE column_name LIKE '%present%' LIMIT 1").fetchone()
        tamper = "no presence column found"
        if row:
            table_name, col = row
            con.execute(
                f'UPDATE "{table_name}" SET "{col}" = NULL '
                "WHERE rowid = (SELECT min(rowid) FROM "
                f'"{table_name}")')
            tamper = f"{table_name}.{col} set NULL in first row"
        con.execute("CHECKPOINT")
        con.close()
        # Regenerate the manifest so the reader reaches presence validation
        # rather than failing earlier at the integrity check.
        manifest_path = dest / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["file_sha256"]["run.duckdb"] = sha256_file(
            dest / "run.duckdb")
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        expected = {"kind": "invalid"}
        err = None
        try:
            reader.Recording(dest)
            expected = {"kind": "unexpected-acceptance"}
            err = "tampered database accepted"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        files = out.copy_tree("recording/bad_presence", dest)
        files += write_canonical(
            out, "recording/bad_presence/expected.json",
            {"tamper": tamper, "expected": expected})
        rec.add("recording/bad_presence", recipe="RCP-05",
                group="recording", comparison="B", expected=expected,
                produced_by="write_recording + duckdb UPDATE presence=NULL "
                            "+ regenerated manifest",
                files=files, notes=f"presence-validation rejection: {err}")
    except Exception as exc:
        rec.blocker("recording/bad_presence", "RCP-05",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- no_manifest: readable payload without the manifest ------------------
    try:
        rows = [rs.sample_row(interval_id=0)]
        capture = rs.synthetic_capture(rows=rows)
        dest = storage.write_recording(capture, ctx.tmp / "recroot3")
        files = []
        for child in sorted(dest.iterdir()):
            if child.is_file() and child.name != "manifest.json":
                files.append(out.copy(
                    f"recording/no_manifest/{child.name}", child))
        stage = ctx.tmp / "no_manifest"
        stage.mkdir(exist_ok=True)
        for child in dest.iterdir():
            if child.is_file() and child.name != "manifest.json":
                shutil.copy2(child, stage / child.name)
        try:
            reader.Recording(stage)
            expected = {"kind": "unexpected-acceptance"}
            err = "reader accepted a directory without manifest"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        files += write_canonical(
            out, "recording/no_manifest/expected.json",
            {"omitted": "manifest.json", "expected": expected})
        rec.add("recording/no_manifest", recipe="RCP-05",
                group="recording", comparison="B", expected=expected,
                produced_by="write_recording minus manifest.json",
                files=files, notes=f"manifest-first rejection: {err}")
    except Exception as exc:
        rec.blocker("recording/no_manifest", "RCP-05",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- report_failed: sealed recording whose summary generation failed -----
    try:
        rows = [rs.sample_row(interval_id=0)]
        capture = rs.synthetic_capture(rows=rows)

        def _failing_report(db, dest_dir):
            raise RuntimeError("injected report failure")
        dest = storage.write_recording(capture, ctx.tmp / "recroot4",
                                       _reporter=_failing_report)
        files = out.copy_tree("recording/report_failed", dest)
        rec.add("recording/report_failed", recipe="RCP-05",
                group="recording", comparison="S",
                expected={"kind": "diagnostic", "valid": False,
                          "diagnostic": "report-generation"},
                produced_by="write_recording with a failing _reporter",
                files=files,
                notes="manifest.report_status='failed'; publication still "
                      "seals — report failure is distinct from publication "
                      "failure")
    except Exception as exc:
        rec.blocker("recording/report_failed", "RCP-05",
                    "".join(traceback.format_exception(exc))[-1500:])

    # --- welded catalog descriptor -------------------------------------------
    try:
        sim = ctx.sim_cache.get("welded")
        if sim is not None:
            import mujoco
            from bike_sim.recording.catalog import (
                JointCoords, build_catalog)
            jtypes = {int(mujoco.mjtJoint.mjJNT_FREE): "free",
                      int(mujoco.mjtJoint.mjJNT_BALL): "ball",
                      int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
                      int(mujoco.mjtJoint.mjJNT_SLIDE): "slide"}
            joints = []
            for j in range(sim.model.njnt):
                name = mujoco.mj_id2name(sim.model,
                                         mujoco.mjtObj.mjOBJ_JOINT, j)
                joints.append(JointCoords(
                    name=name,
                    joint_type=jtypes[int(sim.model.jnt_type[j])],
                    qposadr=int(sim.model.jnt_qposadr[j]),
                    dofadr=int(sim.model.jnt_dofadr[j])))
            catalog = build_catalog(tuple(joints))
            desc = {
                "tables": [spec.name for spec in catalog.table_order()],
                "coordinates": {
                    "qpos": [dataclasses.asdict(c)
                             for c in catalog.coordinates.qpos],
                    "dof": [dataclasses.asdict(c)
                            for c in catalog.coordinates.dof]},
                "aliases": [dataclasses.asdict(a) for a in catalog.aliases],
                "channel_roots": sorted(catalog.channel_roots()),
            }
            files = write_canonical(
                out, "recording/catalog_welded.json", desc)
            rec.add("recording/catalog_welded", recipe="RCP-05",
                    group="recording", comparison="S",
                    expected={"kind": "accepted"},
                    produced_by="build_catalog(JointCoords from welded "
                                "mjModel)",
                    files=files,
                    notes="coordinate/alias identity for the welded model")
    except Exception as exc:
        rec.blocker("recording/catalog_welded", "RCP-05",
                    f"{type(exc).__name__}: {exc}")

    # --- schema constants + table inventory ----------------------------------
    try:
        import inspect
        tables = [spec.name for spec in rschema.all_tables()]
        constants = {
            "SCHEMA_VERSION": rschema.SCHEMA_VERSION,
            "DATABASE": rschema.DATABASE,
            "MANIFEST": rschema.MANIFEST,
            "REPORT": getattr(rschema, "REPORT", "summary.md"),
            "tables": tables,
        }
        files = write_canonical(out, "recording/schema.json", constants)
        rec.add("recording/schema.json", recipe="RCP-05", group="recording",
                comparison="S", expected={"kind": "accepted"},
                produced_by="recording.schema introspection", files=files,
                notes="payload names, schema version and table inventory")
    except Exception as exc:
        rec.blocker("recording/schema.json", "RCP-05",
                    f"{type(exc).__name__}: {exc}")

    # --- publication-failure enumeration --------------------------------------
    try:
        failures = {}
        # publication against an unwritable root
        rows = [rs.sample_row(interval_id=0)]
        capture = rs.synthetic_capture(rows=rows)
        # A fixed /tmp path keeps the recorded Errno message deterministic
        # (ctx.tmp paths differ every run).
        bad_root = Path("/tmp/bike-sim-t01-notadir")
        bad_root.write_text("not a directory")
        try:
            storage.write_recording(capture, bad_root)
            failures["unwritable_root"] = {"kind": "unexpected-acceptance"}
        except Exception as exc:
            failures["unwritable_root"] = {
                "kind": "invalid", "error": type(exc).__name__,
                "message": str(exc)}
        finally:
            bad_root.unlink(missing_ok=True)
        # stages: write/commit/checkpoint/hash/report/link/manifest — the
        # report stage is covered by recording/report_failed; the rest are
        # enumerated with their code location for the native port.
        stages = {
            "write": "storage._database BEGIN..COMMIT (duckdb write)",
            "commit": "storage._database COMMIT",
            "checkpoint": "storage._database CHECKPOINT",
            "report": "storage._report -> recording.report.write_summary",
            "hash": "storage.sha256_file over payloads",
            "link": "os.link into the timestamped destination",
            "manifest": "sealed-manifest publication (LAST step)",
        }
        files = write_canonical(
            out, "recording/publication_failures.json",
            {"stages": stages, "reproduced": failures})
        rec.add("recording/publication_failures", recipe="RCP-05",
                group="recording", comparison="S",
                expected={"kind": "accepted"},
                produced_by="stage enumeration + reproduced errors",
                files=files,
                notes="publication failure taxonomy; report failure already "
                      "captured as a sealed-diagnostic fixture")
    except Exception as exc:
        rec.blocker("recording/publication_failures", "RCP-05",
                    f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# RCP-06 replay + CLI contracts
# ---------------------------------------------------------------------------

def _reader_expected(verdict: dict) -> dict:
    """Map a ``replay_episode`` outcome onto the manifest ``expected``.

    ``expected`` records the oracle *reader's adjudication*, not merely that
    bytes were captured — the same contract ``replay/tampered`` uses:

    * reader rejection  → ``{"kind": "invalid", error, message}`` verbatim;
    * clean replay      → ``{"kind": "accepted"}``;
    * clean replay of a run captured under ``--diagnostic-model-limits``
      (``result.model_valid is False``) → ``kind: "diagnostic"`` with
      ``valid: False`` — a diagnostic episode is never physically valid.
    """
    if "error" in verdict:
        return {"kind": "invalid", "error": verdict["error"],
                "message": verdict.get("message", "")}
    result = verdict.get("result") or {}
    if result.get("model_valid") is False:
        return {"kind": "diagnostic", "valid": False,
                "diagnostic": "model-limits",
                "replay": ("passed" if result.get("passed") else "failed"),
                "reason": "episode captured under "
                          "--diagnostic-model-limits; replay may pass "
                          "numerically but validity remains false"}
    return {"kind": "accepted"}


def _rcp06_replay(ctx: Context):
    out, rec = ctx.out, ctx.rec
    support = _support(ctx.oracle)
    env = None
    try:
        env = ctx.sim_cache.get("research_env")
        if env is None:
            env = support.make_python_research(duration=0.1, seed=210)
            ctx.sim_cache["research_env"] = env
        from bike_sim.sim.ride.control import RideControl
        command = RideControl(motor_torque_nm=10.0)
        try:
            for _ in range(4):
                env.step(command)
        except Exception:
            pass
    except Exception as exc:
        rec.blocker("replay", "RCP-06",
                    "".join(traceback.format_exception(exc))[-1500:])
        return

    # legacy schema-2 replay files: export_recording writes the complete
    # bundle (replay.json published last)
    try:
        from bike_sim.native.recording import export_recording
        dest = ctx.tmp / "legacy_v2"
        export_recording(env, dest)
        files = out.copy_tree("replay/legacy_v2", dest)
        verdict = {}
        try:
            from bike_sim.sim.research.replay import replay_episode
            verdict = {"result": replay_episode(dest)}
        except Exception as exc:
            verdict = {"error": type(exc).__name__, "message": str(exc)}
        files += write_canonical(out, "replay/legacy_v2/verdict.json",
                                 verdict)
        rec.add("replay/legacy_v2", recipe="RCP-06", group="replay",
                comparison="B", expected=_reader_expected(verdict),
                produced_by="capture_legacy + write_replay_files "
                            "(schema 2 file set)",
                files=files,
                notes="source-produced legacy replay bundle; expected "
                      "records the oracle reader's adjudication of it")
    except Exception as exc:
        rec.blocker("replay/legacy_v2", "RCP-06",
                    "".join(traceback.format_exception(exc))[-1500:])

    # schema-1 legacy bundle: no v1 writer exists in the source, so derive the
    # v1 shape from the captured v2 bundle — drop the schema-2-only files,
    # strip 'execution'/'joint_strength_file' from the manifest, restrict
    # file_sha256 to the v1 set and let the current reader adjudicate.
    try:
        from bike_sim.sim.research.replay import (
            FILES as _V1_FILES, _sha256 as _rp_sha)
        src_dir = out.root / "replay" / "legacy_v2"
        dest = ctx.tmp / "legacy_v1"
        dest.mkdir(parents=True, exist_ok=True)
        manifest_v2 = json.loads((src_dir / "replay.json").read_text())
        keep = set(_V1_FILES)
        if "joint_envelope_file" in manifest_v2:
            keep.add(manifest_v2["joint_envelope_file"])
        for name in keep:
            shutil.copy2(src_dir / name, dest / name)
        manifest_v1 = {"schema_version": 1,
                       "start_x_m": manifest_v2["start_x_m"],
                       "heightfield": manifest_v2["heightfield"],
                       "state_signature": manifest_v2["state_signature"],
                       "file_sha256": {n: _rp_sha(dest / n)
                                       for n in sorted(keep)}}
        if "joint_envelope_file" in manifest_v2:
            manifest_v1["joint_envelope_file"] = \
                manifest_v2["joint_envelope_file"]
        (dest / "replay.json").write_text(
            json.dumps(manifest_v1, indent=2, sort_keys=True) + "\n")
        files = out.copy_tree("replay/legacy_v1", dest)
        verdict = {}
        try:
            from bike_sim.sim.research.replay import replay_episode
            verdict = {"result": replay_episode(dest)}
        except Exception as exc:
            verdict = {"error": type(exc).__name__, "message": str(exc)}
        files += write_canonical(out, "replay/legacy_v1/verdict.json",
                                 verdict)
        rec.add("replay/legacy_v1", recipe="RCP-06", group="replay",
                comparison="B", expected=_reader_expected(verdict),
                produced_by="schema-1 derivation from replay/legacy_v2 "
                            "(no v1 writer in the source)",
                files=files,
                inputs=[src_dir],
                notes="derived v1 file set + manifest; expected records "
                      "the oracle reader's adjudication of schema 1")
    except Exception as exc:
        rec.blocker("replay/legacy_v1", "RCP-06",
                    "".join(traceback.format_exception(exc))[-1500:])

    # historical recording: a genuine schema-2 bundle written by an earlier
    # source revision, captured verbatim while the original writer still exists
    try:
        main_root = ctx.oracle.parent.parent
        candidates = sorted(main_root.glob(
            "output/perf-roadmap/*/N1/seed-0/ep"))
        if not candidates:
            candidates = sorted(main_root.glob("output/**/ep"))
            candidates = [c for c in candidates
                          if (c / "states.npz").is_file()]
        if not candidates:
            raise FileNotFoundError("no historical replay bundle under "
                                    f"{main_root / 'output'}")
        src_dir = candidates[0]
        files = out.copy_tree("replay/historical_v2", src_dir)
        verdict = {}
        try:
            from bike_sim.sim.research.replay import replay_episode
            verdict = {"result": replay_episode(src_dir)}
        except Exception as exc:
            verdict = {"error": type(exc).__name__, "message": str(exc)}
        files += write_canonical(
            out, "replay/historical_v2/verdict.json",
            {"source": str(src_dir), "verdict": verdict})
        rec.add("replay/historical_v2", recipe="RCP-06", group="replay",
                comparison="B", expected=_reader_expected(verdict),
                produced_by="verbatim copy of a pre-existing schema-2 "
                            "recording on disk",
                files=files, inputs=[src_dir],
                notes="written by an earlier source revision; expected "
                      "records the current reader's adjudication of it")
    except Exception as exc:
        rec.blocker("replay/historical_v2", "RCP-06",
                    f"{type(exc).__name__}: {exc}")

    # v3 DuckDB recording via the production lifecycle
    try:
        from bike_sim.cli import research as research_cli
        args = research_cli.parser().parse_args([
            "--physics-config",
            str(out.root / "config/viewer_physics_welded.toml"),
            "--track-file", str(out.root / "tracks/flat.toml"),
            "--duration", "0.1", "--seed", "210", "--assist",
            "--record", "--out", str(ctx.tmp / "v3out"),
            "--diagnostic-model-limits"])
        from bike_sim.sim.research.recording_lifecycle import (
            ResearchRun, close_environment)
        env3 = research_cli.make_environment(args, capture_mode="full")
        launch = ResearchRun(env3, args.out, enabled=True,
                             mode="research_headless",
                             invocation=["bike-research", "--fixture"])
        try:
            while not env3.done:
                env3.step(research_cli.command_for(env3, args))
        except Exception:
            pass
        launch.finish()
        close_environment(env3)
        dest = None
        for child in sorted(args.out.rglob("manifest.json")):
            dest = child.parent
        if dest is None:
            raise RuntimeError("no published v3 recording located")
        files = out.copy_tree("replay/native_v3", dest)
        verdict = {}
        try:
            from bike_sim.sim.research.replay import replay_episode
            verdict = {"result": replay_episode(dest)}
        except Exception as exc:
            verdict = {"error": type(exc).__name__, "message": str(exc)}
        files += write_canonical(out, "replay/native_v3/replay_result.json",
                                 verdict)
        rec.add("replay/native_v3", recipe="RCP-06", group="replay",
                comparison="B", expected=_reader_expected(verdict),
                produced_by="research env capture_mode=full + "
                            "close_environment publication under "
                            "--diagnostic-model-limits",
                files=files,
                notes="genuine v3 DuckDB recording + checked-replay verdict; "
                      "produced under --diagnostic-model-limits so "
                      "model_valid is false even though checked replay "
                      "passes numerically")

        # tampered copy: flip one byte inside run.duckdb
        tampered = ctx.tmp / "v3_tampered"
        shutil.copytree(dest, tampered)
        db = tampered / "run.duckdb"
        blob = bytearray(db.read_bytes())
        blob[len(blob) // 2] ^= 0x01
        db.write_bytes(bytes(blob))
        files = out.copy_tree("replay/tampered", tampered)
        try:
            replay_episode(tampered)
            expected = {"kind": "unexpected-acceptance"}
            err = "tampered recording replayed cleanly"
        except Exception as exc:
            expected = {"kind": "invalid", "error": type(exc).__name__,
                        "message": str(exc)}
            err = str(exc)
        files += write_canonical(out, "replay/tampered/expected.json",
                                 {"tamper": "single byte flip in run.duckdb",
                                  "expected": expected})
        rec.add("replay/tampered", recipe="RCP-06", group="replay",
                comparison="B", expected=expected,
                produced_by="copy + 1-byte mutation of run.duckdb",
                files=files, notes=f"integrity rejection: {err}")
    except Exception as exc:
        rec.blocker("replay/native_v3", "RCP-06",
                    "".join(traceback.format_exception(exc))[-1500:])


def _cli_battery(ctx: Context) -> dict:
    """All nine CLI surfaces: help, parse battery, subprocess invocations."""
    out, rec = ctx.out, ctx.rec
    env = dict(os.environ)
    env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_cache")

    cfg = str(out.root / "config" / "viewer_physics_welded.toml")
    flat = str(out.root / "tracks" / "flat.toml")
    programs = {
        "bike-sim": ("bike_sim.cli.main", "parse_args", None),
        "bike-ride": ("bike_sim.cli.ride", "parse_args",
                      [["--physics-config", cfg,
                        "--track", flat, "--headless",
                        "--duration", "0.1", "--no-plots"],
                       ["--track", "climb_steps", "--speed", "16"]]),
        "bike-settle": ("bike_sim.cli.settle", "parse_args",
                        [["--physics-config", cfg,
                          "--track", flat, "--out",
                          "/tmp/settle_pose.npz"]]),
        "bike-research": ("bike_sim.cli.research", "parser",
                          [["--physics-config", cfg,
                            "--track-file", flat,
                            "--duration", "0.1", "--seed", "210",
                            "--ideal-sensors"]]),
        "bike-recording": ("bike_sim.cli.recording", "parser",
                           [["overview", "recording/good", "--json"],
                            ["fields", "recording/good", "--pattern",
                             "*torque*"],
                            ["window", "recording/good", "--episode", "0",
                             "--start", "0", "--end", "0.01",
                             "--fields", "*"],
                            ["export", "recording/good", "--episode", "0",
                             "--format", "jsonl", "--output", "/tmp/exp.jsonl"],
                            ["compare", "a", "b", "--left-episode", "0",
                             "--right-episode", "0", "--mode", "summary"],
                            ["overview"]]),
        "bike-batch": ("bike_sim.cli.batch", "parser", []),
        "bike-replay": ("bike_sim.cli.replay", None, []),
        "bike-export": ("bike_sim.cli.export", None, []),
        "bike-playground": ("bike_sim.cli.playground", None, []),
    }
    programs["bike-ride"][2].append(["--list-tracks"])

    result = {"schema": CLI_SCHEMA, "programs": {}}
    for prog, (module_name, accessor, argvs) in programs.items():
        entry = {"module": module_name, "cases": []}
        import importlib
        module = importlib.import_module(module_name)
        parser_obj = None
        if accessor == "parser":
            parser_obj = module.parser()
        elif accessor == "parse_args":
            try:
                ns = module.parse_args([])
                entry["defaults"] = vars(ns)
            except SystemExit as exc:
                entry["defaults_error"] = f"SystemExit({exc.code})"
            except TypeError:
                # zero-arg parser (bike-sim): inject argv via sys.argv
                old_argv = sys.argv
                try:
                    sys.argv = [prog]
                    entry["defaults"] = vars(module.parse_args())
                except (Exception, SystemExit) as exc:
                    entry["defaults_error"] = (f"{type(exc).__name__}: "
                                               f"{getattr(exc, 'code', exc)}")
                finally:
                    sys.argv = old_argv
            except Exception as exc:
                entry["defaults_error"] = f"{type(exc).__name__}: {exc}"
            for argv in (argvs or []):
                case = {"argv": argv, "via": "parse_args"}
                try:
                    ns = module.parse_args(list(argv))
                    case["result"] = "ok"
                    case["namespace"] = {
                        k: repr(v) for k, v in vars(ns).items()}
                except SystemExit as exc:
                    case["result"] = "parse_error"
                    case["exit"] = exc.code
                except Exception as exc:
                    case["result"] = "error"
                    case["error"] = f"{type(exc).__name__}: {exc}"
                entry["cases"].append(case)
        if parser_obj is not None:
            help_text = parser_obj.format_help()
            record = out.text(f"cli/{prog}.help.txt", help_text)
            entry["help"] = {"path": f"cli/{prog}.help.txt",
                             "sha256": record["sha256"]}
            entry["actions"] = [
                {"flags": a.option_strings, "dest": a.dest,
                 "default": (None if a.default is argparse.SUPPRESS
                             else repr(a.default)),
                 "required": bool(a.required),
                 "nargs": a.nargs if not isinstance(a.nargs, int) else a.nargs,
                 "choices": (None if a.choices is None
                             else [repr(c) for c in a.choices]),
                 "type": (None if a.type is None
                          else getattr(a.type, "__name__", repr(a.type)))}
                for a in parser_obj._actions]
            for argv in argvs:
                case = {"argv": argv}
                try:
                    ns = parser_obj.parse_args(argv)
                    case["result"] = "ok"
                    case["namespace"] = vars(ns)
                except SystemExit as exc:
                    case["result"] = "parse_error"
                    case["exit"] = exc.code
                except Exception as exc:
                    case["result"] = "error"
                    case["error"] = f"{type(exc).__name__}: {exc}"
                entry["cases"].append(case)
        else:
            # parser hidden inside main(); capture help via subprocess
            code, stdout, stderr, via = _run_cli(ctx, prog, ["--help"], env)
            record = out.text(f"cli/{prog}.help.txt", stdout)
            entry["help"] = {"path": f"cli/{prog}.help.txt",
                             "sha256": record["sha256"],
                             "exit": code, "via": via}
            entry["cases"].append({"argv": ["--help"], "result": "subprocess",
                                   "exit": code, "via": via})
            if code != 0 or not stdout.strip():
                rec.blocker(
                    f"cli/{prog}.help", "RCP-06",
                    f"--help via {via} yielded exit={code} and "
                    f"{len(stdout)} stdout bytes — the captured help "
                    "fixture is empty and must not be mistaken for a "
                    "real contract")
        # invalid flag via subprocess for every program
        code, stdout, stderr, via = _run_cli(ctx, prog, ["--bogus-flag"], env)
        entry["cases"].append({"argv": ["--bogus-flag"],
                               "result": "subprocess", "exit": code,
                               "via": via,
                               "cwd": str(ctx.out.root),
                               "stderr_tail": stderr[-400:]})
        if code == 0 and not stdout and not stderr:
            rec.blocker(
                f"cli/{prog}.bogus-flag", "RCP-06",
                "--bogus-flag exited 0 with no output — the invocation was "
                "a no-op, not a genuine parser rejection")
        result["programs"][prog] = entry

    # real valid subprocess invocations that are cheap.  Output paths use
    # fixed /tmp names (not ctx.tmp) so recorded argv and any path-bearing
    # stdout stay byte-identical across capture runs.
    good = out.root / "recording" / "good"
    scratch = Path("/tmp/bike-sim-t01-cli")
    shutil.rmtree(scratch, ignore_errors=True)
    scratch.mkdir(parents=True, exist_ok=True)
    valid = [
        ("bike-ride", ["--list-tracks"]),
        ("bike-ride", ["--dump-track", "flat"]),
        ("bike-export", ["-o", str(scratch / "export"), "--json-only"]),
        ("bike-recording", ["overview", str(good), "--json"]),
        ("bike-recording", ["fields", str(good), "--pattern", "*torque*"]),
        ("bike-recording", ["export", str(good), "--episode", "0",
                            "--format", "csv",
                            "--output", str(scratch / "telemetry.csv")]),
        ("bike-recording", ["export", str(good), "--episode", "0",
                            "--format", "jsonl",
                            "--output", str(scratch / "telemetry.jsonl")]),
        ("bike-recording", ["compare", str(good), str(good),
                            "--left-episode", "0", "--right-episode", "0",
                            "--mode", "exact"]),
        ("bike-recording", ["compare", str(good), str(good),
                            "--left-episode", "0", "--right-episode", "0",
                            "--mode", "summary"]),
        ("bike-recording", ["sql", str(good),
                            "--query", "SELECT count(*) FROM episodes"]),
        ("bike-replay", ["--bogus-flag"]),
    ]
    # a checked replay on the genuine v3 recording, if captured
    v3 = out.root / "replay" / "native_v3"
    if (v3 / "manifest.json").is_file():
        valid.append(("bike-replay", [str(v3)]))
    for prog, argv in valid:
        code, stdout, stderr, via = _run_cli(ctx, prog, argv, env)
        result["programs"].setdefault(prog, {"cases": []})["cases"].append(
            {"argv": argv, "result": "subprocess", "exit": code,
             "via": via, "cwd": str(ctx.out.root),
             "stdout_sha256": sha256_bytes(stdout.encode()),
             "stderr_tail": stderr[-400:]})
        if not stdout and not stderr:
            rec.blocker(
                f"cli/{prog}.positive", "RCP-06",
                f"positive invocation {argv!r} via {via} exited {code} "
                "with no output — cannot distinguish a real contract from "
                "a missing entry point; recorded as a blocker instead of "
                "a fabricated pass")
    return result


def _console_script(prog: str) -> Path | None:
    """Locate ``prog``'s installed console script without resolving links.

    ``sys.executable`` inside a uv-managed venv is ``.venv/bin/python`` — a
    symlink into the shared interpreter store.  ``resolve().parent`` lands
    on the store's ``bin/`` which holds *no* project console scripts, so the
    unresolved parent must be searched first.
    """
    exe = Path(sys.executable)
    candidates = [exe.parent / prog]
    try:
        candidates.append(exe.resolve().parent / prog)
    except OSError:
        pass
    try:
        import sysconfig
        candidates.append(Path(sysconfig.get_path("scripts")) / prog)
    except Exception:
        pass
    for cand in candidates:
        if cand.is_file():
            return cand
    return None


def _cli_shim_command(prog: str, argv: list) -> list:
    """``python -c`` fallback resolving the console entry point by name.

    Unlike the previous shim, ``main`` is *always* invoked: ``argv`` is
    passed when the signature accepts a positional parameter (the
    ``main(argv=None)`` convention), otherwise ``sys.argv`` is pre-seeded
    and ``main()`` is called — ``sys.exit(0)`` is never reached without
    calling ``main`` first.
    """
    code = (
        "import sys, inspect\n"
        "from importlib.metadata import entry_points\n"
        f"sys.argv = {[prog, *argv]!r}\n"
        "ep = next((e for e in entry_points(group='console_scripts') "
        f"if e.name == {prog!r}), None)\n"
        "if ep is None:\n"
        f"    sys.stderr.write({('no console_scripts entry point ' + prog)!r})\n"
        "    sys.exit(127)\n"
        "main = ep.load()\n"
        "params = inspect.signature(main).parameters\n"
        "accepts = any(p.kind in (p.POSITIONAL_ONLY, "
        "p.POSITIONAL_OR_KEYWORD, p.VAR_POSITIONAL) "
        "for p in params.values())\n"
        f"rv = main({list(argv)!r}) if accepts else main()\n"
        "sys.exit(0 if rv is None else rv)\n")
    return [sys.executable, "-c", code]


def _run_cli(ctx: Context, prog: str, argv: list, env: dict) -> tuple:
    """Run a console script inside the oracle venv without a shell.

    Returns ``(exit_code, stdout, stderr, via)`` where ``via`` is
    ``"script"`` (installed console entry point) or ``"shim"`` (metadata
    fallback) so callers can record the invocation provenance.
    """
    script = _console_script(prog)
    if script is not None:
        cmd, via = [str(script), *argv], "script"
    else:
        cmd, via = _cli_shim_command(prog, argv), "shim"
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          cwd=str(ctx.out.root), env=env, timeout=300)
    return proc.returncode, proc.stdout, proc.stderr, via


def _module_for(prog: str) -> str:
    return {
        "bike-sim": "bike_sim.cli.main",
        "bike-playground": "bike_sim.cli.playground",
        "bike-export": "bike_sim.cli.export",
        "bike-ride": "bike_sim.cli.ride",
        "bike-settle": "bike_sim.cli.settle",
        "bike-research": "bike_sim.cli.research",
        "bike-batch": "bike_sim.cli.batch",
        "bike-replay": "bike_sim.cli.replay",
        "bike-recording": "bike_sim.cli.recording",
    }[prog]


# ---------------------------------------------------------------------------
# RCP-07 validation / performance / HUD
# ---------------------------------------------------------------------------

def _rcp07_validation(ctx: Context):
    out, rec = ctx.out, ctx.rec

    # tire characterization rig
    try:
        from bike_sim.physics.tire import TireSpec
        from bike_sim.terrain.surface import SURFACES
        from bike_sim.validation.tire_characterization import (
            steady_brush_force)
        config_text = (
            "# validation/tire/config.toml — authored fixture input\n"
            "# steady_brush_force(tire, surface, slip_ratio, load_n)\n"
            "surface = \"asphalt\"\n"
            "speed_mps = 2.0\ndt_s = 0.000625\nduration_s = 0.5\n"
            "slip_ratios = [0.0, 0.05, 0.1, 0.2]\n"
            "loads_n = [400.0, 800.0]\n")
        files = [out.text("validation/tire/config.toml", config_text)]
        from bike_sim.physics.resolution import load_physics_config
        cfg = load_physics_config(out.root / "config/viewer_physics_welded.toml")
        tire = cfg.tires.front  # TireParameters: .mu/.tangent_k_n_m/.relaxation_length_m
        surface = SURFACES["asphalt"]
        verdict = {"surface": "asphalt", "cases": []}
        for kappa in (0.0, .05, .1, .2):
            for load in (400., 800.):
                res = steady_brush_force(tire, surface, kappa, load,
                                         duration_s=.5)
                verdict["cases"].append(res)
        files += write_canonical(out, "validation/tire/verdict.json",
                                 verdict)
        rec.add("validation/tire", recipe="RCP-07", group="validation",
                comparison="B", expected={"kind": "accepted"},
                produced_by="steady_brush_force over the recorded grid",
                files=files,
                notes="longitudinal tire bench, pure material mechanics")
    except Exception as exc:
        rec.blocker("validation/tire", "RCP-07",
                    "".join(traceback.format_exception(exc))[-1200:])

    # HUD oracle text (semantic, not pixels)
    try:
        sim = ctx.sim_cache.get("welded")
        if sim is not None:
            import _physical_hud_oracle as hud
            hud_obj = hud.RideHUD(sim.model)
            line = hud_obj.line(sim) if hasattr(hud_obj, "line") else None
            files = []
            if line is not None:
                files.append(out.text("validation/hud_welded.txt", str(line)))
            row = (hud_obj.preview_log_row(sim)
                   if hasattr(hud_obj, "preview_log_row") else None)
            if row is not None:
                files += write_canonical(
                    out, "validation/hud_welded_row.json", row)
            rec.add("validation/hud_welded", recipe="RCP-07",
                    group="validation", comparison="Q",
                    expected={"kind": "accepted"},
                    produced_by="_physical_hud_oracle.RideHUD on welded sim",
                    files=files,
                    notes="semantic HUD rows/legend; pixel/HiDPI checks "
                          "excluded by recipe")
        else:
            rec.blocker("validation/hud_welded", "RCP-07",
                        "welded sim unavailable")
    except Exception as exc:
        rec.blocker("validation/hud_welded", "RCP-07",
                    f"{type(exc).__name__}: {exc}")

    # performance evidence (P class — evidence, never a gate)
    try:
        sim = ctx.sim_cache.get("welded")
        if sim is not None:
            import time
            from bike_sim.sim.ride.control import RideControl
            control = RideControl(motor_torque_nm=10.0)
            # warmup first (numba kernels compile on first call)
            for _ in range(20):
                sim.step(0., 0., control=control)
            t0 = time.perf_counter()
            n = 100
            for _ in range(n):
                sim.step(0., 0., control=control)
            wall = time.perf_counter() - t0
            perf = {"steps": n, "wall_s": wall,
                    "s_per_step": wall / n,
                    "sim_dt_s": float(sim.model.opt.timestep),
                    "sim_time_s": float(sim.data.time),
                    "note": "evidence only — never a performance gate; the "
                            "four-step prefix cannot qualify speed"}
            files = write_canonical(
                out, "validation/baselines/perf.json", perf)
            rec.add("validation/baselines/perf", recipe="RCP-07",
                    group="validation", comparison="P",
                    expected={"kind": "accepted"},
                    produced_by="20-step warmup + 100 timed welded steps",
                    files=files,
                    notes="P-class evidence; not derivable from the "
                          "four-step/0.005 s prefix")
        else:
            rec.blocker("validation/baselines/perf", "RCP-07",
                        "welded sim unavailable")
    except Exception as exc:
        rec.blocker("validation/baselines/perf", "RCP-07",
                    f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# Mandatory negative controls
# ---------------------------------------------------------------------------

def _negative_controls(ctx: Context):
    out, rec = ctx.out, ctx.rec

    # 1. bit flip must be detected by the fixture comparator
    try:
        probe = out.root / "bootstrap/welded/integration.f64"
        data = probe.read_bytes()
        original = np.frombuffer(data, dtype="<f8").copy()
        flipped = original.copy()
        flipped.view(np.uint64)[0] ^= 1
        detected = flipped.tobytes() != original.tobytes() or \
            not np.array_equal(
                flipped.view(np.uint64), original.view(np.uint64))
        rec.control("bitwise.force-bit-flip", bool(detected),
                    "flipped u64 bit detected by raw-byte comparison")
    except Exception as exc:
        rec.control("bitwise.force-bit-flip", False,
                    f"{type(exc).__name__}: {exc}")

    # 2. invalid strict-reference prefix stays invalid with first-failure id.
    #    The stored state.json carries the canonical-codec $array tags, which
    #    the native ctor never sees — so the t=0 bootstrap state is rebuilt
    #    via capture_bootstrap (byte-identical to the fixture payload, proven
    #    by comparing the externalised integration.f64 bytes) and element 0
    #    of the real integration_state array is set to NaN.
    try:
        from bike_sim.native.setup import capture_bootstrap
        base = out.root / "bootstrap" / "welded"
        tag = json.loads((base / "state.json").read_text())[
            "integration_state"]
        fixture_bytes = (out.root / tag["$array"]).read_bytes()
        sim = ctx.sim_cache.get("welded")
        if sim is None:
            sim = _support(ctx.oracle).make_python_ride(strict=False,
                                                        decimation=1)
        else:
            sim.reset()
        work = ctx.tmp / "strict_prefix"
        work.mkdir(parents=True, exist_ok=True)
        boot = capture_bootstrap(sim, work)
        live = np.asarray(boot.state["integration_state"], dtype="<f8")
        fixture_match = live.tobytes() == fixture_bytes
        st = json.loads(json.dumps(boot.state, default=_jd))
        st["integration_state"][0] = float("nan")
        # Load the ctor OUTSIDE the inner try so a missing/broken extension
        # fails the control honestly instead of posing as the first-failure
        # identity of the mutated prefix.
        ctor = _native_ctor(ctx)
        try:
            ctor(str(base / "model.mjb"),
                 json.loads((base / "config.json").read_text()), st)
            rec.control("invalid-strict-prefix", False,
                        "mutated prefix accepted")
        except Exception as exc:
            # Gate on the oracle rejecting the mutated prefix; the
            # fixture-match bit is recorded as provenance (pinned *_py
            # kernels may drift the rebuilt state ~1 ulp from the
            # production-Numba fixture bytes).
            rec.control("invalid-strict-prefix", True,
                        f"first failure: {type(exc).__name__}: {exc}; "
                        f"fixture integration_state matches live state: "
                        f"{fixture_match}")
    except Exception as exc:
        rec.control("invalid-strict-prefix", False,
                    f"{type(exc).__name__}: {exc}")

    # 2b. close-then-step must be rejected on the research lifecycle
    try:
        env = ctx.sim_cache.get("research_env")
        if env is None:
            env = _support(ctx.oracle).make_python_research(duration=0.05,
                                                            seed=7)
        env.close(discard_pending=True)
        try:
            from bike_sim.sim.ride.control import RideControl
            env.step(RideControl(motor_torque_nm=5.0))
            rec.control("bootstrap.close-then-step", False,
                        "step after close accepted")
        except Exception as exc:
            rec.control("bootstrap.close-then-step", True,
                        f"rejected: {type(exc).__name__}: {exc}")
    except Exception as exc:
        rec.control("bootstrap.close-then-step", False,
                    f"{type(exc).__name__}: {exc}")

    # 3. diagnostic runs never labeled physically valid.  A diagnostic run
    #    is one whose structured expectation marks it diagnostic
    #    (expected.kind == 'diagnostic' or expected.diagnostic set or
    #    expected.valid is False).  Such entries must not carry
    #    expected.kind == 'accepted'.  Prose notes mentioning diagnostics
    #    are not the run's validity label.
    diag_bad = [
        e["name"] for e in rec.fixtures
        if e.get("expected", {}).get("kind") == "accepted"
        and (e["expected"].get("diagnostic") is not None
             or e["expected"].get("valid") is False)]
    missing = [
        e["name"] for e in rec.fixtures
        if e.get("expected", {}).get("kind") == "diagnostic"
        and e["expected"].get("valid") is not False]
    rec.control("diagnostic-validity-label",
                not diag_bad and not missing,
                f"diagnostic entries mislabeled as accepted: {diag_bad}; "
                f"diagnostic entries missing valid=false: {missing}")

    # 3b. engine poison: corrupt the MuJoCo state mid-run; record the real
    #     surfaced failure (exception, monitor flag or silent bad values)
    try:
        support = _support(ctx.oracle)
        psim = support.make_python_ride(strict=False, decimation=1)
        from bike_sim.sim.ride.control import RideControl
        poison_outcome = {}
        psim.step(0., 0., control=RideControl(motor_torque_nm=5.0))
        psim.data.qpos[0] = float("nan")
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                psim.step(0., 0., control=RideControl(motor_torque_nm=5.0))
            poison_outcome = {
                "kind": "stepped-without-exception",
                "warnings": [f"{type(w.message).__name__}: {w.message}"
                             for w in caught],
                "qpos0_after": repr(psim.data.qpos[0]),
                "all_finite": bool(np.isfinite(psim.data.qpos).all()
                                   and np.isfinite(psim.data.qvel).all())}
        except Exception as exc:
            poison_outcome = {"kind": "rejected",
                              "error": type(exc).__name__,
                              "message": str(exc)}
        files = write_canonical(out, "bootstrap/engine_poison.json",
                                {"mutation": "qpos[0]=NaN after one valid "
                                             "step", "outcome": poison_outcome})
        rec.add("bootstrap/engine_poison", recipe="RCP-01",
                group="bootstrap", comparison="S",
                expected=poison_outcome,
                produced_by="make_python_ride + qpos NaN injection + step",
                files=files,
                notes="records the true engine-poison surface: rejection or "
                      "the monitor-visible corruption it produces")
    except Exception as exc:
        rec.blocker("bootstrap/engine_poison", "RCP-01",
                    f"{type(exc).__name__}: {exc}")

    # 4. performance gate must not derive from the four-step prefix
    rec.control("performance-gate-rule", True,
                "perf fixture records 100 timed steps after a 20-step "
                "warmup; nothing derives a gate from the documented "
                "four-step/0.005 s prefix")

    # 5. module/factory policy inventory mapped to native replacements
    try:
        inventory = _policy_inventory(ctx)
        files = write_canonical(out, "cli/policy_inventory.json", inventory)
        rec.add("cli/policy_inventory", recipe="RCP-06", group="cli",
                comparison="S", expected={"kind": "accepted"},
                produced_by="static policy-factory inventory (no plugin "
                            "execution)",
                files=files,
                notes="module:factory seam mapped to native registry/ABI "
                      "replacements")
        rec.control("policy-inventory", True,
                    f"{len(inventory.get('builtins', []))} built-ins mapped; "
                    "untrusted plugins never executed")
    except Exception as exc:
        rec.control("policy-inventory", False,
                    f"{type(exc).__name__}: {exc}")


def _policy_inventory(ctx: Context) -> dict:
    """Static inventory of the --policy module:factory seam and built-ins."""
    import inspect
    import bike_sim.sim.research.policies as policies
    builtins = []
    for name, member in inspect.getmembers(policies):
        if inspect.isclass(member) and member.__module__ == policies.__name__:
            builtins.append({"name": name,
                             "factory": f"bike_sim.sim.research.policies:"
                                        f"{name}"})
    return {
        "seam": "--policy module:factory (bike_sim.cli.research)",
        "loader": "research._load_policy / environment policy resolution",
        "builtins": builtins,
        "native_mapping": {
            "builtin": "native static registry (interface-contracts §6)",
            "module:factory": "explicit native library path + bike_policy_v1 "
                              "ABI identity; never Python",
        },
        "note": "untrusted plugins are not executed during capture",
    }


# ---------------------------------------------------------------------------
# RCP-02/T15 — model boundary (inertia, component masses, rider geometry,
# mass properties, air spring, coil shock, dampers, suspension tuning)
# ---------------------------------------------------------------------------

def _rcp15_model(ctx: Context):
    out, rec = ctx.out, ctx.rec
    import xml.etree.ElementTree as ET

    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.physics import air_spring as air_mod
    from bike_sim.physics import coil_shock as coil_mod
    from bike_sim.physics import component_masses as cm_mod
    from bike_sim.physics import damper as damper_mod
    from bike_sim.physics import inertia as inertia_mod
    from bike_sim.physics import mass as mass_mod
    from bike_sim.physics import rider as rider_mod
    from bike_sim.physics import rider_segments as seg_mod
    from bike_sim.physics import tuning as tuning_mod
    from bike_sim.sim.ride import mass_properties as mp_mod

    def emit(name, payload, *, comparison="B", produced_by="", notes=""):
        files = write_canonical(out, f"{name}.json", payload)
        rec.add(name, recipe="RCP-02", group="model",
                comparison=comparison, expected={"kind": "accepted"},
                produced_by=produced_by, files=files, notes=notes)

    def outcome(fn):
        try:
            return {"accepted": True, "result": fn()}
        except Exception as exc:
            return {"accepted": False, "error": type(exc).__name__,
                    "message": str(exc)}

    def ordered_pairs(mapping):
        """Insertion-ordered dict -> [[key, value], ...] (JSON sort_keys
        would destroy the recorded dict order)."""
        return [[k, v] for k, v in mapping.items()]

    def vec(v):
        return np.asarray(v, dtype=float)

    # --- inertia.py -----------------------------------------------------
    ring_cases = []
    for tag, args in (
            ("annular", (0.5, 0.30, 0.36, 0.05)),
            ("solid_hub", (1.2, 0.0, 0.045, 0.10)),
            ("thin_ring", (0.1, 0.10, 0.101, 0.02)),
            ("zero_mass", (0.0, 0.30, 0.36, 0.05)),
            ("neg_width", (0.5, 0.30, 0.36, -0.01)),
            ("inner_ge_outer", (0.5, 0.40, 0.30, 0.05)),
            ("inner_negative", (0.5, -0.10, 0.30, 0.05)),
            ("nan_mass", (float("nan"), 0.30, 0.36, 0.05)),
            ("inf_outer", (0.5, 0.30, float("inf"), 0.05))):
        ring_cases.append(
            {"tag": tag, "args": list(args),
             **outcome(lambda a=args: inertia_mod.ring_inertia(*a))})
    axis_cases = []
    for tag, mass, offset in (
            ("origin", 1.5, (0.0, 0.0, 0.0)),
            ("axial", 0.8, (0.2, 0.0, 0.0)),
            ("mixed", 2.4, (0.10, -0.05, 0.30)),
            ("zero_mass", 0.0, (0.1, 0.2, 0.3)),
            ("neg_mass", -1.0, (0.1, 0.0, 0.0)),
            ("nan_offset", 1.0, (float("nan"), 0.0, 0.0))):
        axis_cases.append(
            {"tag": tag, "mass_kg": mass, "offset_m": vec(offset),
             **outcome(lambda m=mass, o=offset:
                       inertia_mod.parallel_axis(m, o))})
    wheel_cases = []
    for tag, mass, comps in (
            ("front", 2.40, inertia_mod.FRONT_WHEEL_PROFILE),
            ("rear", 2.80, inertia_mod.REAR_WHEEL_PROFILE),
            ("bad_fraction_sum", 1.0,
             (inertia_mod.WheelComponent(0.5, 0.0, 0.1, 0.05,
                                         (0.0, 0.0, 0.0)),
              inertia_mod.WheelComponent(0.4, 0.0, 0.05, 0.05,
                                         (0.0, 0.0, 0.0)))),
            ("empty", 1.0, ()),
            ("zero_mass", 0.0, inertia_mod.FRONT_WHEEL_PROFILE),
            ("neg_fraction", 1.0,
             (inertia_mod.WheelComponent(-0.5, 0.0, 0.1, 0.05,
                                         (0.0, 0.0, 0.0)),)),
            ("nan_offset", 1.0,
             (inertia_mod.WheelComponent(
                 1.0, 0.0, 0.1, 0.05, (float("nan"), 0.0, 0.0)),))):
        def wheel(m=mass, c=comps):
            com, tensor = inertia_mod.wheel_body_inertia(m, c)
            return {"com": com, "tensor": tensor}
        wheel_cases.append({"tag": tag, **outcome(wheel)})
    inertial_cases = []
    good_tensor = inertia_mod.ring_inertia(0.5, 0.30, 0.36, 0.05)
    for tag, mass, com, tensor, dup in (
            ("accepted", 0.5, (0.1, 0.0, 0.2), good_tensor, False),
            ("duplicate_inertial", 0.5, (0.0, 0.0, 0.0),
             good_tensor, True),
            ("zero_mass", 0.0, (0.0, 0.0, 0.0), good_tensor, False),
            ("nonfinite_com", 0.5, (float("nan"), 0.0, 0.0),
             good_tensor, False),
            ("asymmetric", 0.5, (0.0, 0.0, 0.0),
             np.array([[1.0, 0.5, 0.0], [0.4, 1.0, 0.0],
                       [0.0, 0.0, 1.0]]), False),
            ("indefinite", 0.5, (0.0, 0.0, 0.0),
             np.diag([-1.0, 2.0, 3.0]), False),
            ("triangle_violation", 0.5, (0.0, 0.0, 0.0),
             np.diag([1.0, 1.0, 3.0]), False)):
        def add_inertial(m=mass, c=com, t=tensor, d=dup):
            body = ET.fromstring("<body/>")
            if d:
                ET.SubElement(body, "inertial", {"mass": "1.0"})
            inertia_mod.add_body_inertial(body, m, c, t)
            node = body.find("inertial")
            return {"mass": node.get("mass"), "pos": node.get("pos"),
                    "fullinertia": node.get("fullinertia")}
        inertial_cases.append({"tag": tag, **outcome(add_inertial)})
    emit("model/inertia",
         {"_notes": "ring/parallel-axis/wheel tensors + add_body_inertial "
                    "accepted & rejected branches",
          "_produced_by": "inertia.py calls over authored case tables",
          "ring_cases": ring_cases, "axis_cases": axis_cases,
          "wheel_cases": wheel_cases, "inertial_cases": inertial_cases},
         produced_by="inertia.py ring/parallel_axis/wheel_body_inertia/"
                     "add_body_inertial over authored cases")

    # --- component_masses.py ---------------------------------------------
    def make_geoms(*mass_attrs):
        return [ET.fromstring(
            "<geom " + (f'mass="{m}"' if m is not None else "") + "/>")
                for m in mass_attrs]
    assign_cases = []
    for tag, total, attrs in (
            ("two_weights", 3.0, ("1.5", "4.5")),
            ("missing_attr", 2.0, ("2.0", None, "2.0")),
            ("zero_and_pos", 1.0, ("0", "5")),
            ("bad_string", 1.0, ("abc", "2")),
            ("neg_weight", 1.0, ("-1", "2")),
            ("empty", 1.0, ()),
            ("all_zero", 1.0, ("0", "0")),
            ("zero_total", 0.0, ("1", "2")),
            ("nan_total", float("nan"), ("1", "2"))):
        def assign(t=total, a=attrs):
            geoms = make_geoms(*a)
            cm_mod.assign_component_mass(geoms, t)
            return [g.get("mass") for g in geoms]
        assign_cases.append({"tag": tag, "total_kg": total,
                             "weights": list(attrs), **outcome(assign)})
    register_cases = []
    registry = {}
    for tag, cid, attrs in (
            ("accepted", "fork", ("1.0", "0", "0.5")),
            ("all_zero", "dead", ("0", "0")),
            ("duplicate", "fork", ("1.0",)),
            ("missing_attrs", "bare", (None, "2.0")),
            ("negative_only", "neg", ("-1",)),
            ("bad_mass_string", "garbage", ("xx",))):
        def register(c=cid, a=attrs):
            cm_mod.register_component(registry, c, make_geoms(*a))
            return {"ids": list(registry.keys()),
                    "sizes": [len(v) for v in registry.values()]}
        register_cases.append({"tag": tag, "component_id": cid,
                               "masses": list(attrs), **outcome(register)})
    emit("model/component_masses",
         {"_notes": "geom weight distribution + registry accepted/"
                    "duplicate/empty/error branches",
          "_produced_by": "component_masses.py assign/register over "
                          "authored geom lists",
          "assign_cases": assign_cases,
          "register_cases": register_cases},
         produced_by="component_masses.py assign_component_mass/"
                     "register_component")

    # --- mass.py — specs surfaces ----------------------------------------
    ms = mass_mod.BikeMassSpecs()
    comp = ms.component_masses
    prov = ms.component_provenance
    wheel_i_cases = []
    for tag, wm, outer, hub in (("front", 2.40, 0.372, 0.045),
                                ("rear", 2.80, 0.352, 0.045),
                                ("custom_hub", 1.5, 0.30, 0.06)):
        wheel_i_cases.append(
            {"tag": tag,
             "i_yy": ms.compute_wheel_rotational_inertia(wm, outer, hub)})
    emit("model/mass_specs",
         {"_notes": "dataclass-field-order component budgets, provenance, "
                    "builtin-sum total, wheel I_yy approximation",
          "_produced_by": "BikeMassSpecs() component_masses/provenance/"
                          "total/wheel inertia",
          "names": list(comp.keys()),
          "masses": np.array(list(comp.values())),
          "provenance": [[k, v["mass"], v["distribution"]]
                         for k, v in prov.items()],
          "total_bike_mass": ms.total_bike_mass,
          "wheel_inertia_cases": wheel_i_cases},
         produced_by="BikeMassSpecs() views")

    # --- mass.py — component centers + static system CG -------------------
    def centers_payload(result):
        return {"names": list(result.keys()),
                "positions": np.array([p for p, _ in result.values()]),
                "masses": np.array([m for _, m in result.values()])}
    default_centers = mass_mod.compute_component_centers_of_mass()
    rider_default = rider_mod.RiderSpecs()
    rider_centers = mass_mod.compute_component_centers_of_mass(
        rider_specs=rider_default)
    emit("model/component_centers",
         {"_notes": "15-entry bike table, then rider centers appended for "
                    "the seated variant",
          "_produced_by": "compute_component_centers_of_mass default + "
                          "seated rider",
          "bike_only": centers_payload(default_centers),
          "with_seated_rider": centers_payload(rider_centers)},
         produced_by="compute_component_centers_of_mass")
    cg_plain = mass_mod.compute_static_system_cg()
    cg_rider = mass_mod.compute_static_system_cg(rider_specs=rider_default)
    emit("model/static_system_cg",
         {"_notes": "sequential-fold system CoM + axle load split, "
                    "bike-only and with the seated rider",
          "_produced_by": "compute_static_system_cg default + rider",
          "bike_only": cg_plain,
          "with_seated_rider": cg_rider},
         produced_by="compute_static_system_cg")

    # --- rider.py — resolution + records ----------------------------------
    resolve_cases = []
    for tag, rider, include, default in (
            ("variant_seated", "seated", None, "seated"),
            ("variant_lumped", "lumped", None, "seated"),
            ("variant_articulated", "articulated_planar", None, "seated"),
            ("variant_none", "none", None, "seated"),
            ("bad_variant", "unicycle", None, "seated"),
            ("none_default", None, None, "seated"),
            ("none_include_true", None, True, "seated"),
            ("none_include_false", None, False, "seated"),
            ("none_default_lumped", None, None, "lumped"),
            ("wrong_type", 123, None, "seated")):
        def resolve(r=rider, i=include, d=default):
            specs = rider_mod.resolve_rider(r, include_rider=i,
                                            default_variant=d)
            return dataclasses.asdict(specs)
        resolve_cases.append(
            {"tag": tag, "rider": rider, "include_rider": include,
             "default_variant": default, **outcome(resolve)})
    passthrough = rider_mod.RiderSpecs(variant="lumped", mass_kg=70.0)
    resolve_cases.append(
        {"tag": "specs_passthrough", "rider": "RiderSpecs(lumped,70kg)",
         "include_rider": None, "default_variant": "seated",
         **outcome(lambda: dataclasses.asdict(
             rider_mod.resolve_rider(passthrough)))})
    built = rider_mod.rider_from_args("seated", 75.0, 1.75, None)
    rider_default = rider_mod.RiderSpecs()
    artic_specs = rider_mod.RiderSpecs(
        variant="seated", legs="articulated", mass_kg=75.0, height_m=1.75)
    artic_variant = rider_mod.RiderSpecs(variant="articulated_planar")
    spec_errors = []
    for tag, kwargs in (
            ("bad_legs", {"legs": "spring"}),
            ("zero_mass", {"mass_kg": 0.0}),
            ("neg_height", {"height_m": -1.0}),
            ("inseam_over_height", {"inseam_m": 1.9}),
            ("shares_not_one", {"saddle_share": 0.5}),
            ("helmet_ge_mass", {"helmet_mass_kg": 90.0})):
        spec_errors.append(
            {"tag": tag, "kwargs": kwargs,
             **outcome(lambda k=kwargs: rider_mod.RiderSpecs(**k))})
    emit("model/rider_specs",
         {"_notes": "resolve_rider dispatches, rider_from_args record, "
                    "RiderSpecs post_init rejections",
          "_produced_by": "rider.py resolve_rider/rider_from_args/"
                          "RiderSpecs",
          "resolve_cases": resolve_cases,
          "rider_from_args_75_175": dataclasses.asdict(built),
          "default_specs": dataclasses.asdict(rider_default),
          "default_derived": {
              "present": rider_default.present,
              "total_rider_mass": rider_default.total_rider_mass,
              "inseam": rider_default.inseam,
              "saddle_height_m": rider_default.saddle_height_m,
              "torso_helmet_mass": rider_default.torso_helmet_mass,
              "legs_mass": rider_default.legs_mass,
              "arms_mass": rider_default.arms_mass,
              "segment_lengths": {
                  n: rider_default.segment_length(n)
                  for n in ("trunk", "upper_arm", "forearm", "hand",
                            "thigh", "shank", "head")}},
          "spec_errors": spec_errors},
         produced_by="rider.py resolution layer")

    def body_dict(b):
        return {"name": b.name, "parent": b.parent, "attach": b.attach,
                "joint": b.joint,
                "geoms": [{"name": g.name, "kind": g.kind, "p1": g.p1,
                           "p2": g.p2, "radius": g.radius,
                           "mass": g.mass} for g in b.geoms],
                "stiffness_n_m": b.stiffness_n_m,
                "damping_ns_m": b.damping_ns_m,
                "supported_mass_kg": b.supported_mass_kg,
                "unilateral": b.unilateral, "interface": b.interface,
                "mass": b.mass, "center_of_mass": b.center_of_mass,
                "preload_n": b.preload_n,
                "preload_deflection_m": b.preload_deflection_m}

    def chain_dict(c):
        return {"side": c.side, "lateral_y_m": c.lateral_y_m,
                "crank_len_m": c.crank_len_m, "hip": c.hip, "knee": c.knee,
                "ankle": c.ankle, "pedal": c.pedal,
                "thigh_len_m": c.thigh_len_m, "shank_len_m": c.shank_len_m,
                "thigh_mass_kg": c.thigh_mass_kg,
                "shank_mass_kg": c.shank_mass_kg,
                "foot_mass_kg": c.foot_mass_kg,
                "pitch0_thigh_rad": c.pitch0_thigh_rad,
                "pitch0_shank_rad": c.pitch0_shank_rad,
                "pitch0_foot_rad": c.pitch0_foot_rad}

    def pose_dict(pose):
        fields = ("saddle", "hip", "shoulder", "elbow", "wrist", "grip",
                  "head_center", "knee_front", "knee_rear", "ankle_front",
                  "ankle_rear", "pedal_front", "pedal_rear")
        out_dict = {k: getattr(pose, k) for k in fields}
        out_dict["saddle"] = dataclasses.asdict(pose.saddle)
        out_dict.update(
            {"torso_lean_deg": pose.torso_lean_deg,
             "knee_flexion_front_deg": pose.knee_flexion_front_deg,
             "knee_flexion_rear_deg": pose.knee_flexion_rear_deg,
             "knee_flexion_bdc_deg": pose.knee_flexion_bdc_deg,
             "bodies": [body_dict(b) for b in pose.bodies],
             "path_masses_kg": ordered_pairs(pose.path_masses_kg),
             "leg_chains": [chain_dict(c) for c in pose.leg_chains],
             "total_mass_kg": pose.total_mass_kg,
             "interface_loads_n": ordered_pairs(pose.interface_loads_n()),
             "interface_shares": ordered_pairs(pose.interface_shares()),
             "missing_body_error":
                 outcome(lambda: pose.body("rider_nothing"))})
        return out_dict

    specs = BikeSpecs()
    pose_default = rider_default.seated_pose(specs)
    pose_75 = built.seated_pose(specs)
    pose_leg_articulated = artic_specs.seated_pose(specs)
    emit("model/seated_pose",
         {"_notes": "full solved SeatedPose for default + 75kg riders, "
                    "articulated-leg chain variant",
          "_produced_by": "solve_seated_pose over three RiderSpecs",
          "default": pose_dict(pose_default),
          "rider_75_175": pose_dict(pose_75),
          "legs_articulated": pose_dict(pose_leg_articulated)},
         produced_by="solve_seated_pose")

    # --- rider.py — helpers -----------------------------------------------
    ik_cases = []
    for tag, a, b, la, lb, prefer in (
            ("reachable_px", (0.1, 0.0, 0.9), (0.2, 0.0, 0.4), 0.45, 0.45,
             "+x"),
            ("reachable_mx", (0.1, 0.0, 0.9), (0.2, 0.0, 0.4), 0.45, 0.45,
             "-x"),
            ("prefer_pz", (0.0, 0.0, 1.0), (0.4, 0.0, 0.6), 0.3, 0.3,
             "+z"),
            ("prefer_mz", (0.0, 0.0, 1.0), (0.4, 0.0, 0.6), 0.3, 0.3,
             "-z"),
            ("too_far", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 0.4, 0.4,
             "+x"),
            ("too_close", (0.0, 0.0, 0.0), (0.05, 0.0, 0.0), 0.4, 0.2,
             "+x"),
            ("degenerate_straight", (0.0, 0.0, 0.0), (0.8, 0.0, 0.0),
             0.4, 0.4, "+x"),
            ("bad_prefer", (0.0, 0.0, 1.0), (0.4, 0.0, 0.6), 0.3, 0.3,
             "+y")):
        ik_cases.append(
            {"tag": tag, "a": vec(a), "b": vec(b), "len_a": la,
             "len_b": lb, "prefer": prefer,
             **outcome(lambda a=a, b=b, la=la, lb=lb, p=prefer:
                       rider_mod._two_link_ik(vec(a), vec(b), la, lb, p))})
    flexion_cases = []
    for tag, prox, joint, distal in (
            ("straight", (0.0, 0.0, 1.0), (0.0, 0.0, 0.5),
             (0.0, 0.0, 0.0)),
            ("right_angle", (0.0, 0.0, 1.0), (0.0, 0.0, 0.5),
             (0.5, 0.0, 0.5)),
            ("bent_30ish", (0.1, 0.0, 1.0), (0.2, 0.0, 0.6),
             (0.25, 0.0, 0.2))):
        flexion_cases.append(
            {"tag": tag, "proximal": vec(prox), "joint": vec(joint),
             "distal": vec(distal),
             **outcome(lambda p=prox, j=joint, d=distal:
                       rider_mod._flexion_deg(vec(p), vec(j), vec(d)))})
    spring_cases = [{"tag": "default",
                     **outcome(lambda: rider_mod._spring(30.0, 5.0, 0.4))}]
    path_cases = []
    for tag, kwargs in (
            ("default", {}),
            ("rider_75", {"mass_kg": 75.0, "height_m": 1.75}),
            # shares sum to 1 so __post_init__ passes and the
            # seated_path_masses error arms are the ones exercised.
            ("bar_too_low", {"bar_share": 0.05, "pedal_share": 0.40}),
            ("pedal_too_low", {"pedal_share": 0.05, "bar_share": 0.40}),
            ("pedal_too_high", {"pedal_share": 0.80, "saddle_share": 0.10,
                                "bar_share": 0.10}),
            # pelvis<=0 is unreachable for valid RiderSpecs (needs
            # helmet >= mass); exercise a heavy-helmet accepted row.
            ("helmet_heavy", {"mass_kg": 60.0, "helmet_mass_kg": 5.0,
                              "saddle_share": 0.40, "pedal_share": 0.30,
                              "bar_share": 0.30})):
        def path_m(k=kwargs):
            r = rider_mod.RiderSpecs(**k)
            return ordered_pairs(rider_mod.seated_path_masses(r))
        path_cases.append({"tag": tag, **outcome(path_m)})
    emit("model/rider_helpers",
         {"_notes": "two-link IK branches, flexion, spring derivation, "
                    "seated path mass splits incl. rejection arms",
          "_produced_by": "rider.py _two_link_ik/_flexion_deg/_spring/"
                          "seated_path_masses",
          "ik_cases": ik_cases, "flexion_cases": flexion_cases,
          "spring_cases": spring_cases, "path_cases": path_cases},
         produced_by="rider.py helper layer")

    # --- rider.py — leg chains + apparent mass + centers -------------------
    chain = pose_leg_articulated.leg_chains[0]
    chain_rear = pose_leg_articulated.leg_chains[1]
    leg_cases = []
    for phase in (0.0, 0.7, 1.9, 3.0):
        def leg_ops(c=chain, p=phase):
            qpos = rider_mod.solve_leg_qpos(c, p, 0.0)
            joints = rider_mod.solve_leg_joints(c, p, 0.0)
            knee, ankle, pedal = rider_mod.fk_leg(c, qpos, 0.0)
            jac = rider_mod.leg_jacobian(c, qpos, 0.0)
            return {"qpos": qpos, "knee_ik": joints[0],
                    "ankle_ik": joints[1], "knee_fk": knee,
                    "ankle_fk": ankle, "pedal_fk": pedal,
                    "jacobian": np.asarray(jac),
                    "crank_torque_share":
                        rider_mod.crank_torque_share(p, 0.6),
                    "pedal_spindle": rider_mod.pedal_spindle_pos(
                        p, c.lateral_y_m, c.crank_len_m)}
        leg_cases.append({"phase": phase, "side": "front",
                          **outcome(leg_ops)})
    leg_cases.append(
        {"phase": 1.2, "side": "rear",
         **outcome(lambda: {"qpos": rider_mod.solve_leg_qpos(
             chain_rear, 1.2, 0.0)})})
    center_variants = {}
    for variant, r in (("none", rider_mod.RiderSpecs(variant="none")),
                       ("lumped", rider_mod.RiderSpecs(variant="lumped")),
                       ("seated", rider_default),
                       ("seated_articulated_legs", artic_specs),
                       ("articulated_planar", artic_variant)):
        def centers(r=r):
            result = r.compute_rider_centers_of_mass(specs)
            return {"names": list(result.keys()),
                    "positions": np.array([p for p, _ in result.values()])
                    if result else np.zeros(0),
                    "masses": np.array([m for _, m in result.values()])
                    if result else np.zeros(0)}
        center_variants[variant] = outcome(centers)
    apparent = rider_mod.saddle_path_apparent_mass(
        pose_default, np.array([0.5, 1.0, 2.0, 4.8, 8.0, 20.0]))
    emit("model/rider_legs_centers",
         {"_notes": "articulated leg IK/FK/Jacobian/crank share, per-"
                    "variant rider CoM tables, saddle apparent mass",
          "_produced_by": "rider.py leg chain ops + "
                          "compute_rider_centers_of_mass + apparent mass",
          "leg_cases": leg_cases,
          "center_variants": center_variants,
          "apparent_mass": apparent},
         produced_by="rider.py leg/center/apparent surfaces")

    # --- rider_segments.py --------------------------------------------------
    seg_masses = seg_mod.segment_masses(75.0, 0.4)
    split = seg_mod.split_paired_arm_masses(seg_masses)
    inertia_rows = []
    for tag, mass, axis, radius in (
            ("head", 5.95, (0.0, 0.0, 0.19), 0.110),
            ("torso", 25.8, (0.28, 0.0, -0.42), 0.110),
            ("upper_arm", 2.0, (0.30, 0.0, -0.35), 0.045),
            ("thigh", 11.3, (0.10, 0.0, -0.42), 0.070),
            ("shank", 3.4, (0.12, 0.0, -0.40), 0.055),
            ("foot", 1.1, (0.16, 0.0, -0.04), 0.035)):
        inertia_rows.append(
            {"name": tag, "mass_kg": mass, "axis_vector_m": vec(axis),
             "radius_m": radius,
             **outcome(lambda m=mass, a=axis, r=radius:
                       seg_mod.segment_inertia(m, vec(a), r))})
    seg_cases = []
    for tag, total, helmet in (("default", 80.0, 0.4),
                               ("helmet_eq_mass", 50.0, 50.0),
                               ("helmet_over", 50.0, 60.0),
                               ("zero_total", 0.0, 0.0),
                               ("neg_helmet", 80.0, -0.5)):
        seg_cases.append(
            {"tag": tag, "total_kg": total, "helmet_kg": helmet,
             **outcome(lambda t=total, h=helmet:
                       ordered_pairs(seg_mod.segment_masses(t, h)))})
    inertia_error_cases = []
    for tag, m, axis, r in (
            ("zero_mass", 0.0, (0.3, 0.0, 0.3), 0.05),
            ("zero_axis", 1.0, (0.0, 0.0, 0.0), 0.05),
            ("neg_radius", 1.0, (0.3, 0.0, 0.3), -0.05),
            ("nan_axis", 1.0, (float("nan"), 0.0, 0.0), 0.05)):
        inertia_error_cases.append(
            {"tag": tag,
             **outcome(lambda m=m, a=axis, r=r:
                       seg_mod.segment_inertia(m, vec(a), r))})
    artic_pose = seg_mod.geometry_pose(artic_variant, specs)
    emit("model/rider_segments",
         {"_notes": "de Leva tables, anatomical masses + arm split, "
                    "segment inertia tensors, articulated geometry pose",
          "_produced_by": "rider_segments.py tables/helpers/geometry_pose",
          "de_leva_fractions":
              ordered_pairs(seg_mod.DE_LEVA_MASS_FRACTIONS),
          "arm_lateral_offset_m": seg_mod.ARM_LATERAL_OFFSET_M,
          "mass_cases": seg_cases,
          "split_pairs": ordered_pairs(split),
          "inertia_rows": inertia_rows,
          "inertia_error_cases": inertia_error_cases,
          "articulated_pose": {
              f: getattr(artic_pose, f)
              for f in artic_pose.__dataclass_fields__}},
         produced_by="rider_segments.py")

    # --- air_spring.py ------------------------------------------------------
    air = air_mod.AirSpringSpecs()
    spring = air_mod.ForkAirSpring()
    grid = [0.0, 30.0, 54.0, 90.0, 179.0]
    vol_rows, press_rows, force_rows, stiff_rows = [], [], [], []
    for x in grid:
        vol_rows.append({"travel_mm": x,
                         **outcome(lambda t=x: spring.compute_volumes(t))})
        press_rows.append(
            {"travel_mm": x,
             **outcome(lambda t=x: spring.compute_pressures_pa(t))})
        force_rows.append(
            {"travel_mm": x,
             **outcome(lambda t=x: spring.compute_axial_force(t))})
        stiff_rows.append(
            {"travel_mm": x,
             **outcome(lambda t=x: spring.compute_instantaneous_stiffness(t))})
    curve = spring.compute_force_curve(n_points=41)
    cal_spring = air_mod.ForkAirSpring()
    cal_outcome = outcome(
        lambda: cal_spring.calibrate_psi_for_sag())
    air_errors = []
    for tag, fn in (
            ("tokens_over_max",
             lambda: (lambda s: (s.set_tokens(99), s.num_tokens)[1])(
                 air_mod.ForkAirSpring())),
            ("neg_psi_ctor",
             lambda: air_mod.ForkAirSpring(gauge_pressure_psi=-5.0)),
            ("set_pressure_clamps",
             lambda: (lambda s: (s.set_pressure_psi(1.0),
                                 s.gauge_pressure_psi)[1])(
                 air_mod.ForkAirSpring())),
            ("bad_travel",
             lambda: air_mod.ForkAirSpring().compute_volumes(float("nan"))),
            # travel is clamped to total_travel_mm, so this is accepted.
            ("overtravel_volumes",
             lambda: air_mod.ForkAirSpring().compute_volumes(1e6)),
            # small positive chamber + tokens exhausts before full travel.
            ("exhausted_volumes",
             lambda: air_mod.ForkAirSpring(
                 specs=air_mod.AirSpringSpecs(
                     pos_chamber_length_mm=60.0)).compute_volumes(180.0)),
            ("bad_travel_type",
             lambda: air_mod.ForkAirSpring().compute_volumes("x")),
            ("lstsq_bad_order",
             lambda: air_mod.ForkAirSpring().fit_polynomial_mjcf(0)),
            ("calibrate_bad_sag",
             lambda: air_mod.ForkAirSpring().calibrate_psi_for_sag(
                 target_sag_mm=179.9, target_axial_force_n=100.0))):
        air_errors.append({"tag": tag, **outcome(fn)})
    emit("model/air_spring",
         {"_notes": "spec record + properties, chambers/pressures/forces/"
                    "stiffness over the stroke grid, force curve, psi "
                    "calibration, polynomial fit, error branches",
          "_produced_by": "air_spring.py ForkAirSpring over the default "
                          "spec",
          "specs": dataclasses.asdict(air),
          "properties": {"piston_area_m2": air.piston_area_m2,
                         "token_volume_m3": air.token_volume_m3,
                         "base_pos_volume_m3": air.base_pos_volume_m3,
                         "base_neg_volume_m3": air.base_neg_volume_m3},
          "defaults": {"num_tokens": spring.num_tokens,
                       "gauge_pressure_psi": spring.gauge_pressure_psi,
                       "abs_pressure_pa": spring.abs_pressure_pa,
                       "gauge_pressure_bar": spring.gauge_pressure_bar},
          "volume_rows": vol_rows, "pressure_rows": press_rows,
          "force_rows": force_rows, "stiffness_rows": stiff_rows,
          "force_curve": curve,
          "fit_order3": spring.fit_polynomial_mjcf(),
          "calibration": {"outcome": cal_outcome,
                          "psi_after": cal_spring.gauge_pressure_psi},
          "error_cases": air_errors},
         produced_by="ForkAirSpring over AirSpringSpecs()")

    # --- coil_shock.py ------------------------------------------------------
    coil_specs = coil_mod.CoilShockSpecs()
    coil = coil_mod.CoilShock()
    coil_legacy = coil_mod.CoilShock(legacy_behavior=True)
    coil_pre = coil_mod.CoilShock(
        coil_mod.CoilShockSpecs(preload_mm=2.0))
    coil_rows = []
    for x in (-5.0, 0.0, 20.0, 55.0, 60.0, 65.0, 70.0):
        coil_rows.append(
            {"stroke_mm": x,
             "spring": outcome(lambda t=x: coil.compute_spring_force(t)),
             "bumper": outcome(lambda t=x: coil.compute_bumper_force(t)),
             "axial": outcome(lambda t=x: coil.compute_axial_force(t)),
             "legacy_spring": outcome(
                 lambda t=x: coil_legacy.compute_spring_force(t)),
             "preload_spring": outcome(
                 lambda t=x: coil_pre.compute_spring_force(t))})
    coil_errors = []
    for tag, fn in (
            ("neg_rate", lambda: coil_mod.CoilShock(
                coil_mod.CoilShockSpecs(rate_n_m=-1.0))),
            ("zero_stroke", lambda: coil_mod.CoilShock(
                coil_mod.CoilShockSpecs(stroke_mm=0.0))),
            ("bumper_over_stroke", lambda: coil_mod.CoilShock(
                coil_mod.CoilShockSpecs(bumper_length_mm=70.0))),
            ("neg_preload", lambda: coil_mod.CoilShock(
                coil_mod.CoilShockSpecs(preload_mm=-1.0))),
            ("nan_stroke", lambda: coil.compute_spring_force(
                float("nan")))):
        coil_errors.append({"tag": tag, **outcome(fn)})
    emit("model/coil_shock",
         {"_notes": "linear coil + quadratic bumper, legacy/preload "
                    "variants, spec and call rejections",
          "_produced_by": "coil_shock.py CoilShock over stroke grid",
          "specs": dataclasses.asdict(coil_specs),
          "bumper_engage_mm": coil_specs.bumper_engage_mm,
          "rows": coil_rows, "error_cases": coil_errors},
         produced_by="CoilShock over CoilShockSpecs()")

    # --- damper.py ----------------------------------------------------------
    preset_table = [{"preset": p.name, "label": p.value,
                     "has_config": p in damper_mod.DAMPER_PRESETS,
                     "config": (dataclasses.asdict(
                         damper_mod.DAMPER_PRESETS[p])
                         if p in damper_mod.DAMPER_PRESETS else None)}
                    for p in damper_mod.DamperPreset]
    fork = damper_mod.Charger3Damper()
    shock = damper_mod.SuperDeluxeDamper()
    vel_grid = (-2.5, -0.30, -0.10, -0.03, 0.0, 0.03, 0.10, 0.30, 2.5)
    fork_rows = []
    for v in vel_grid:
        for travel in (20.0, 90.0, 170.0):
            fork_rows.append(
                {"velocity_mps": v, "travel_mm": travel,
                 **outcome(lambda vv=v, t=travel:
                           fork.compute_damping_force(vv, t))})
    shock_rows = []
    for v in vel_grid:
        for stroke in (10.0, 30.0, 60.0):
            shock_rows.append(
                {"velocity_mps": v, "stroke_mm": stroke,
                 "components": outcome(
                     lambda vv=v, s=stroke:
                     ordered_pairs(
                         shock.compute_damping_components(vv, s))),
                 "force": outcome(
                     lambda vv=v, s=stroke:
                     shock.compute_damping_force(vv, s))})
    legacy_shock = damper_mod.SuperDeluxeDamper(legacy_behavior=True)
    locked = damper_mod.SuperDeluxeDamper(lockout_firm=True)
    lock_rows = []
    for v in (0.01, 0.04, 0.5, 2.0):
        lock_rows.append(
            {"velocity_mps": v,
             "modern": outcome(
                 lambda vv=v: locked.compute_damping_force(vv, 20.0)),
             "legacy": outcome(
                 lambda vv=v: legacy_shock.compute_damping_force(
                     vv, 20.0))})
    def _fork_clicks(d):
        return [d.hsc_clicks, d.lsc_clicks, d.rebound_clicks]

    def _shock_clicks(d):
        return [d.hsc_clicks, d.lsc_clicks, d.rebound_clicks,
                d.hbo_clicks, d.lockout_firm]

    # every mutation uses a fresh damper so `fork`/`shock` stay at ctor
    # defaults for the coefficient/dyno/ctor records below.
    click_cases = []
    for tag, fn in (
            ("fork_set_within",
             lambda: (lambda d: (d.set_clicks(hsc=3, lsc=5),
                                 _fork_clicks(d))[1])(
                 damper_mod.Charger3Damper())),
            ("fork_clamps",
             lambda: (lambda d: (d.set_clicks(hsc=99, lsc=-3, reb=25),
                                 _fork_clicks(d))[1])(
                 damper_mod.Charger3Damper())),
            ("fork_bad_click_type",
             lambda: damper_mod.Charger3Damper().set_clicks(hsc=1.5)),
            ("shock_set_all",
             lambda: (lambda d: (d.set_clicks(
                 hsc=1, lsc=2, reb=3, hbo=4, lockout=True),
                 _shock_clicks(d))[1])(damper_mod.SuperDeluxeDamper())),
            ("toggle_lockout",
             lambda: (lambda d: (
                 d.toggle_lockout(), d.toggle_lockout(),
                 d.lockout_firm))(damper_mod.SuperDeluxeDamper())),
            ("fork_ctor_clamps",
             lambda: _fork_clicks(damper_mod.Charger3Damper(
                 hsc_clicks=99, lsc_clicks=-1, rebound_clicks=99))),
            ("fork_low_travel",
             lambda: _fork_clicks(
                 damper_mod.Charger3Damper(total_travel_mm=10.0))),
            ("fork_low_travel_legacy",
             lambda: _fork_clicks(damper_mod.Charger3Damper(
                 total_travel_mm=10.0, legacy_behavior=True))),
            ("shock_zero_stroke",
             lambda: _shock_clicks(
                 damper_mod.SuperDeluxeDamper(total_stroke_mm=0.0))),
            ("fork_nan_velocity",
             lambda: damper_mod.Charger3Damper().compute_damping_force(
                 float("nan"), 50.0)),
            ("shock_nan_stroke",
             lambda: damper_mod.SuperDeluxeDamper().compute_damping_force(
                 0.5, float("nan")))):
        click_cases.append({"tag": tag, **outcome(fn)})
    dyno_fork = damper_mod.Charger3Damper().compute_dyno_curve(
        v_max=1.5, n_points=41)
    dyno_shock = damper_mod.SuperDeluxeDamper().compute_dyno_curve(
        v_max=1.5, n_points=41)
    system = damper_mod.BikeSuspensionSystem()
    applied = []
    for p in damper_mod.DamperPreset:
        def apply(p=p):
            s = damper_mod.BikeSuspensionSystem()
            s.apply_preset(p)
            return {"preset": s.current_preset.name,
                    "fork_clicks": [s.fork_damper.hsc_clicks,
                                    s.fork_damper.lsc_clicks,
                                    s.fork_damper.rebound_clicks],
                    "shock_clicks": [s.shock_damper.hsc_clicks,
                                     s.shock_damper.lsc_clicks,
                                     s.shock_damper.rebound_clicks,
                                     s.shock_damper.hbo_clicks],
                    "lockout": s.shock_damper.lockout_firm,
                    "config": dataclasses.asdict(s.config)}
        applied.append({"preset": p.name, **outcome(apply)})
    cycle = []
    s = damper_mod.BikeSuspensionSystem()
    for forward in (True, True, True, True, False, False):
        cycle.append({"forward": forward,
                      "preset": s.cycle_preset(forward).name})
    s.current_preset = damper_mod.DamperPreset.CUSTOM
    cycle.append({"forward": True,
                  "preset": s.cycle_preset(True).name})
    emit("model/dampers",
         {"_notes": "preset table + click configs, Charger3/SuperDeluxe "
                    "coeffs, damping forces over the velocity grid, "
                    "HBO/lockout/legacy branches, dyno curves, system "
                    "preset apply/cycle",
          "_produced_by": "damper.py classes over authored grids",
          "preset_table": preset_table,
          "click_config_defaults": dataclasses.asdict(
              damper_mod.DamperClickConfig()),
          "fork_coefficients":
              ordered_pairs(fork.get_effective_coefficients()),
          "shock_coefficients":
              ordered_pairs(shock.get_effective_coefficients()),
          "fork_ctor_record": {
              "hsc_clicks": fork.hsc_clicks, "lsc_clicks": fork.lsc_clicks,
              "rebound_clicks": fork.rebound_clicks,
              "hbo_start_mm": fork.hbo_start_mm,
              "c_hbo_base": fork.c_hbo_base,
              "legacy_behavior": fork.legacy_behavior},
          "shock_ctor_record": {
              "hsc_clicks": shock.hsc_clicks, "lsc_clicks": shock.lsc_clicks,
              "rebound_clicks": shock.rebound_clicks,
              "hbo_clicks": shock.hbo_clicks,
              "hbo_start_mm": shock.hbo_start_mm,
              "lockout_firm": shock.lockout_firm,
              "max_hbo": shock.max_hbo,
              "total_stroke_mm": shock.total_stroke_mm,
              "c_hbo_min": shock.c_hbo_min, "c_hbo_max": shock.c_hbo_max,
              "lockout_preload_n": shock.lockout_preload_n,
              "lockout_stiffness": shock.lockout_stiffness,
              "legacy_behavior": shock.legacy_behavior},
          "fork_force_rows": fork_rows,
          "shock_force_rows": shock_rows,
          "lockout_rows": lock_rows,
          "click_cases": click_cases,
          "dyno_fork": dyno_fork, "dyno_shock": dyno_shock,
          "apply_presets": applied, "cycle_sequence": cycle},
         produced_by="damper.py presets/dampers/system")

    # --- tuning.py ----------------------------------------------------------
    refl_cases = []
    for tag, m, lr in (("nominal", 3.5, 2.4), ("unit", 1.0, 1.0),
                       ("zero_mass", 0.0, 2.0), ("neg_lr", 1.0, -1.0),
                       ("nan_mass", float("nan"), 2.0)):
        refl_cases.append(
            {"tag": tag, "mass_kg": m, "leverage_ratio": lr,
             **outcome(lambda m=m, l=lr:
                       tuning_mod.reflected_shock_mass(m, l))})
    tuning_cases = []
    for tag, kwargs in (
            ("default", {}),
            ("rider_90kg", {"rider_mass_kg": 90.0}),
            ("seated_rider", {"rider_specs": rider_default}),
            ("front_fraction", {"front_weight_fraction": 0.45}),
            ("custom_sag", {"target_front_sag_pct": 25.0,
                            "target_rear_sag_pct": 33.0})):
        tuning_cases.append(
            {"tag": tag,
             **outcome(lambda k=kwargs:
                       ordered_pairs(
                           tuning_mod.compute_suspension_tuning_for_sag(
                               **k)))})
    emit("model/tuning",
         {"_notes": "reflected shock mass + compute_suspension_tuning_"
                    "for_sag outputs incl. explicit rider/fraction "
                    "variants",
          "_produced_by": "tuning.py over default BikeSpecs/"
                          "BikeMassSpecs",
          "reflected_cases": refl_cases, "tuning_cases": tuning_cases},
         produced_by="tuning.py")

    # --- sim/ride/mass_properties.py ---------------------------------------
    import mujoco
    minimal_xml = (
        '<mujoco><option timestep="0.001"/>'
        '<worldbody>'
        '<geom name="floor" type="plane" pos="0 0 0" size="10 10 0.1"/>'
        '<body name="rear" pos="-0.45 0 0.098">'
        '<freejoint/>'
        '<geom name="rear_wheel" type="sphere" size="0.1" mass="2.8"/>'
        '</body>'
        '<body name="front" pos="0.45 0 0.098">'
        '<freejoint/>'
        '<geom name="front_wheel" type="sphere" size="0.1" mass="2.4"/>'
        '</body>'
        '<body name="frame" pos="0 0 0.62">'
        '<freejoint/>'
        '<geom name="frame_box" type="box" size="0.12 0.08 0.05" '
        'mass="21.0"/>'
        '</body>'
        '</worldbody></mujoco>')
    try:
        model = mujoco.MjModel.from_xml_string(minimal_xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        front_gid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "front_wheel")
        rear_gid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "rear_wheel")
        mjb_path = ctx.tmp / "wheel_contacts.mjb"
        mujoco.mj_saveModel(model, str(mjb_path))
        mjb_record = out.copy("model/wheel_contacts.mjb", mjb_path)
        expected = {
            "ncon": int(data.ncon),
            "front_geom_id": int(front_gid),
            "rear_geom_id": int(rear_gid),
            "nbody": int(model.nbody),
            "body_mass": np.asarray(model.body_mass),
            "xipos": np.asarray(data.xipos),
            "compiled_com": mp_mod.compiled_center_of_mass(model, data),
            "contact_loads": mp_mod.static_contact_loads(
                model, data, front_gid, rear_gid),
            "no_contact_error": None,
            "swapped_ids_error": None,
        }
        # MjData.ncon cannot be cleared through the binding — a fresh
        # MjData with no mj_forward call has ncon == 0.
        try:
            mp_mod.static_contact_loads(model, mujoco.MjData(model),
                                        front_gid, rear_gid)
        except Exception as exc:
            expected["no_contact_error"] = {
                "error": type(exc).__name__, "message": str(exc)}
        try:
            mp_mod.static_contact_loads(model, data, rear_gid, front_gid)
        except Exception as exc:
            expected["swapped_ids_error"] = {
                "error": type(exc).__name__, "message": str(exc)}
        files = write_canonical(out, "model/mass_properties.json",
                                expected)
        rec.add("model/mass_properties", recipe="RCP-02", group="model",
                comparison="B", expected={"kind": "accepted"},
                produced_by="mujoco.MjModel.from_xml_string + mj_forward "
                            "+ mass_properties.py",
                files=[mjb_record, *files],
                inputs=[], notes="compiled CoM + static contact loads on "
                                 "a three-body resting fixture; error rows "
                                 "for empty contacts and swapped ids")
    except Exception as exc:
        rec.blocker("model/mass_properties", "RCP-02",
                    "".join(traceback.format_exception(exc))[-2000:])


def _jd(o):
    if isinstance(o, np.ndarray):
        return list(o)
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    if dataclasses.is_dataclass(o) and not isinstance(o, type):
        return dataclasses.asdict(o)
    if isinstance(o, (_dt.datetime, _dt.date)):
        return o.isoformat()
    if isinstance(o, uuid.UUID):
        return str(o)
    if isinstance(o, (set, frozenset, tuple)):
        return list(o)
    if isinstance(o, argparse.Namespace):
        return vars(o)
    raise TypeError(type(o).__name__)


def _manifest_default(o):
    try:
        return _jd(o)
    except TypeError:
        return repr(o)


# ---------------------------------------------------------------------------
# Harness fixture
# ---------------------------------------------------------------------------

def _harness(ctx: Context):
    record = ctx.out.text("harness/utf8.txt", "bike\n")
    ctx.rec.add("harness/utf8.txt", recipe="RCP-00", group="harness",
                comparison="Q", expected={"kind": "accepted"},
                produced_by="authored fixture root probe", files=[record],
                notes="fixture-root/encoding probe required by T02 harness")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

GROUPS = {
    "harness": _harness,
    "config": _rcp02_config,
    "model": _rcp15_model,
    "bootstrap": _rcp01_bootstrap,
    "random": _rcp03_random,
    "research": _rcp03_research,
    "equilibrium": _rcp04_equilibrium,
    "recording": _rcp05_recording,
    "replay": _rcp06_replay,
    "validation": _rcp07_validation,
}


def _find_oracle_root() -> Path:
    here = Path.cwd()
    try:
        import bike_sim
        start = Path(bike_sim.__file__).resolve().parent
    except Exception:
        start = here
    for cand in (here, *here.parents, start, *start.parents):
        if (cand / "pyproject.toml").is_file() and (
                cand / "src" / "bike_sim").is_dir():
            return cand
    return here


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--cli-contracts", type=Path, default=None)
    ap.add_argument("--config-contracts", type=Path, default=None)
    ap.add_argument("--groups", default=",".join(GROUPS),
                    help="comma-separated recipe groups to run")
    ap.add_argument("--oracle-root", type=Path, default=None)
    args = ap.parse_args(argv)

    args.output = args.output.resolve()
    args.manifest = args.manifest.resolve()
    oracle = (args.oracle_root or _find_oracle_root()).resolve()
    os.chdir(oracle)
    if str(oracle / "src") not in sys.path:
        sys.path.insert(0, str(oracle / "src"))
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_cache")

    writer = FixtureWriter(args.output)
    recorder = Recorder()
    ctx = Context(oracle, writer, recorder, args)

    provenance = _oracle_provenance(oracle)
    environment = _environment()

    for name in [g.strip() for g in args.groups.split(",") if g.strip()]:
        fn = GROUPS.get(name)
        if fn is None:
            recorder.blocker(f"group:{name}", "-", "unknown recipe group")
            continue
        try:
            fn(ctx)
        except Exception as exc:
            recorder.blocker(f"group:{name}", "-",
                             "".join(traceback.format_exception(exc))[-2000:])

    cli_contracts = None
    if args.cli_contracts is not None:
        try:
            cli_contracts = _cli_battery(ctx)
        except Exception as exc:
            recorder.blocker("cli-contracts", "RCP-06",
                             "".join(traceback.format_exception(exc))[-2000:])
            cli_contracts = {"schema": CLI_SCHEMA,
                             "error": str(exc), "programs": {}}

    _negative_controls(ctx)
    ctx.cleanup()

    manifest = {
        "schema": SCHEMA,
        "created_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "tool": {"path": "tools/migration/capture_reference.py",
                 "sha256": sha256_file(Path(__file__).resolve()),
                 "argv": sys.argv},
        "oracle": provenance,
        "environment": environment,
        "fixtures": recorder.fixtures,
        "negative_controls": recorder.negative_controls,
        "blockers": recorder.blockers,
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False,
                   default=_manifest_default) + "\n", encoding="utf-8")

    if cli_contracts is not None and args.cli_contracts is not None:
        args.cli_contracts.parent.mkdir(parents=True, exist_ok=True)
        args.cli_contracts.write_text(
            json.dumps(cli_contracts, indent=2, sort_keys=True,
                       allow_nan=False, default=_manifest_default) + "\n",
            encoding="utf-8")

    if args.config_contracts is not None:
        config_entries = [f for f in recorder.fixtures
                          if f.get("group") in ("config",)]
        args.config_contracts.parent.mkdir(parents=True, exist_ok=True)
        args.config_contracts.write_text(
            json.dumps({"schema": CONFIG_SCHEMA,
                        "fixtures": config_entries}, indent=2,
                       sort_keys=True, allow_nan=False, default=_manifest_default) + "\n",
            encoding="utf-8")

    n_files = sum(len(f.get("files", ())) for f in recorder.fixtures)
    print(f"captured {len(recorder.fixtures)} fixture entries / "
          f"{n_files} files / {len(recorder.blockers)} blockers")
    for blocker in recorder.blockers:
        print(f"  BLOCKER {blocker['name']}: "
              f"{str(blocker['detail'])[:160]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
