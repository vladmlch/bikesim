"""Probe the selected Python interpreter for native-build dependencies.

The native build must compile and link against the MuJoCo/nanobind that belong
to the *selected* interpreter (``Python_EXECUTABLE`` / the nanobind linkage
interpreter) — never whatever python happens to be on ``PATH``. This module is
therefore split in two:

- the importable API (``select_mujoco_library``, ``validate_dependencies``,
  ``probe_dependencies``) is pure stdlib and runs in any interpreter;
- ``--child-probe`` runs *inside* the selected interpreter, imports
  mujoco/nanobind there, and prints the schema-1 document on stdout only.

Consumers: ``native/cmake/BikeDependencies.cmake`` (configure-time discovery)
and ``tools/dev.py doctor`` (``dependency_probe`` check, which ``json.loads``es
stdout verbatim — keep stdout strictly JSON).

Dependency document, ``schema_version=1``::

    {
      "schema_version": 1,
      "platform":  {"system": "Linux"|"Darwin", "machine": str,
                    "libc": str|null},
      "python":    {"executable": abs, "version": str, "soabi": str,
                    "include_dir": abs},
      "mujoco":    {"version": "3.12.0", "include_dir": abs,
                    "library_path": abs},
      "nanobind":  {"cmake_dir": abs, "include_dir": abs},
      "numerics":  {backend, numpy_version, numpy_package, site_packages,
                    blas_integer_bits, lapack_integer_bits, symbols,
                    numpy_imports, library_paths, threading,
                    provider_config, interp}
    }

Every filesystem path is absolute and exists; ``mujoco.library_path`` resolves
to the one exact platform engine image (see ``select_mujoco_library``).

The ``numerics`` record is the numerical-provider qualification produced by
``discover_numpy_provider`` inside the child probe: which BLAS/LAPACK image
NumPy actually uses, which exact exported symbols satisfy ddot/dgemv/dgemm/
dgelsd, and the integer width of each interface. A wrong width is memory
corruption, so qualification separates metadata validation
(``validate_numerics``, ``require_same_library``, ``checked_symbol``) from
unsafe symbol invocation: foreign names are regex- and export-table-checked
before they are ever declared in generated C or looked up through ctypes, and
the executable ABI check runs in a disposable subprocess whose death is a
hard provider failure. The ``interp`` record is the wheel's measured
``np.interp`` contraction contract (``contracted``/``separate``) — the
port lowers its interpolation mirrors to the same form. The
``multiply_broadcast`` record is the wheel's measured NaN-precedence
contract for ``np.multiply(scalar, array)`` (``scalar``/``element``) —
the broadcast kernel's fmul operand order differs per wheel build, so
a dual-NaN lane resolves to different operands on the qualified
wheels.
Qualified profiles today:

- ``ACCELERATE`` (Darwin): the system framework loaded by NumPy — dyld-cache
  images carry ``identity_kind="apple-system"`` (null sha256, real
  macOS/provider/SDK build id), legacy 32-bit CBLAS plus 64-bit NEWLAPACK
  DGELSD;
- ``NUMPY_OPENBLAS`` (Linux): the single ILP64 scipy-openblas shared object
  recorded in the NumPy wheel's installed distribution records — file
  identity with a real SHA-256, ``scipy_cblas_*64_``/``scipy_*_64_``
  spellings, BLAS and LAPACK both 64-bit.

CLI:

    python tools/native_dependencies.py --python PATH [--output PATH]
    python tools/native_dependencies.py --python PATH \
        --write-numerics-record native/numerics.lock.json
    python tools/native_dependencies.py --check-numerics-record \
        native/numerics.lock.json --probe-document PATH

Without ``--output`` the document is written to stdout; diagnostics and the
named dependency errors go to stderr with a nonzero exit.

``--write-numerics-record`` measures the same document and stores only the
facts that are identical on every machine of the platform under
``records[<platform key>]`` of the committed ``native/numerics.lock.json``
(see "Committed numerics record" below). Machine- and OS-build-specific values
stay in the live document for configure-time consumers.

``--check-numerics-record`` is the configure-time verdict: it projects an
already-measured probe document (``--probe-document``, the one the legacy
configure captured) and requires it to equal the committed entry for this
platform. It runs no probe. On success it prints the platform key on stdout;
on a missing or stale entry it exits 1 with the regeneration command.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import sysconfig
import tempfile
from pathlib import Path

REQUIRED_MUJOCO_VERSION = "3.12.0"
REQUIRED_NUMPY_VERSION = "2.5.2"
SCHEMA_VERSION = 1
_PROBE_TIMEOUT = 300

# Exact, platform-specific candidate filenames only — never a glob and never
# an unrestricted system library search. The first entry is the canonical
# versioned image; the remaining entries are accepted *alias* names that must
# resolve (realpath) to a file still named exactly like the canonical image,
# so an alias can never smuggle in an unversioned or wrong-version library.
_MUJOCO_LIBRARY_CANDIDATES = {
    "Darwin": ("libmujoco.3.12.0.dylib", "libmujoco.dylib"),
    "Linux": ("libmujoco.so.3.12.0", "libmujoco.so"),
}

_SELF = Path(__file__).resolve()


def select_mujoco_library(directory: Path, system: str, version: str) -> Path:
    """Return the one MuJoCo ``version`` engine image inside ``directory``.

    ``directory`` is the installed package directory of the selected
    interpreter's ``mujoco`` package. Candidates are the exact
    platform-specific filenames in ``_MUJOCO_LIBRARY_CANDIDATES``; every hit is
    realpath-resolved, de-duplicated, and must still be named like the
    canonical versioned image. Anything other than exactly one resolved image
    — none, conflicting images, or an unversioned real file — is a named
    ``ValueError``.
    """
    if version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found {version}"
        )
    if system not in _MUJOCO_LIBRARY_CANDIDATES:
        raise ValueError(f"unsupported native platform: {system}")
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(
            f"MuJoCo {version} package directory is missing: {directory}"
        )
    canonical, *aliases = _MUJOCO_LIBRARY_CANDIDATES[system]
    resolved_hits: list[Path] = []
    for name in (canonical, *aliases):
        candidate = directory / name
        if candidate.is_file():
            resolved_hits.append(candidate.resolve())
    misnamed = [path for path in resolved_hits if path.name != canonical]
    if misnamed:
        raise ValueError(
            f"{directory} contains a library that cannot prove the pinned "
            f"MuJoCo {version} engine image: "
            f"{sorted(str(path) for path in misnamed)}"
        )
    found = set(resolved_hits)
    if len(found) != 1:
        raise ValueError(
            f"expected exactly one MuJoCo {version} library in {directory}; "
            f"found {sorted(str(path) for path in found)}"
        )
    return found.pop()


# --------------------------------------------------------------------------
# Numerics symbol/library hygiene (pure stdlib; runs in any interpreter)
# --------------------------------------------------------------------------

# Exported numerical symbols are C/Fortran identifiers that may also carry
# Accelerate's ``$NEWLAPACK$ILP64`` decorations — letters, digits, underscore,
# dollar only. Anything else is unsafe to emit into generated C or hand to a
# dynamic-symbol lookup.
SYMBOL = re.compile(r"[A-Za-z_][A-Za-z0-9_$]*")

_NUMERICS_BACKENDS = ("ACCELERATE", "NUMPY_OPENBLAS")
_NUMERICS_OPS = (
    "ddot", "dgemv", "dgemm", "dgelsd", "dgesv", "dsyevd",
    "ddot_ilp64", "zdotu_sub",
)
_NUMERICS_BLAS_OPS = ("ddot", "dgemv", "dgemm")
_NUMERICS_BLAS_ILP64_OPS = ("ddot_ilp64", "zdotu_sub")
_NUMERICS_LAPACK_OPS = ("dgelsd", "dgesv", "dsyevd")
# The qualified integer widths per backend: Darwin keeps Accelerate's legacy
# 32-bit CBLAS ops with 64-bit NEWLAPACK LAPACK. The separate ILP64 BLAS class
# is always 64-bit, including Darwin; blas_integer_bits describes legacy
# _NUMERICS_BLAS_OPS only. The Linux wheel provider is ILP64 throughout.
# No other profile is qualified — an LP64 or mixed record fails closed.
_NUMERICS_WIDTHS = {"ACCELERATE": (32, 64), "NUMPY_OPENBLAS": (64, 64)}
_NUMERICS_ROLES = ("blas", "lapack", "blas_lapack")
_NUMERICS_IDENTITY_KINDS = ("file", "apple-system")
_ACCELERATE_IMAGE_PREFIX = "/System/Library/Frameworks/Accelerate.framework/"
_SHA256_HEX = re.compile(r"[0-9a-f]{64}")


def checked_symbol(value: str) -> str:
    """Return ``value`` iff it is a safe exported-symbol identifier.

    The gate between untrusted metadata and unsafe invocation: only strings
    passing this check may be placed in a generated asm label or resolved
    through ctypes. ``$`` stays allowed because Accelerate's documented
    ``dgelsd$NEWLAPACK$ILP64`` spelling requires it; the character can never
    escape a double-quoted C string or a subprocess argument array.
    """
    if not isinstance(value, str) or SYMBOL.fullmatch(value) is None:
        raise ValueError(
            f"invalid exported numerical symbol identifier: {value!r}"
        )
    return value


def require_same_library(numpy_path: Path, native_path: Path) -> None:
    """The image the native build links must be the one NumPy uses.

    Resolution is by realpath of existing files — a symlink alias for the
    same inode is one library, a different file with the same basename is a
    substitution and a named failure.
    """
    try:
        resolved_numpy = Path(numpy_path).resolve(strict=True)
        resolved_native = Path(native_path).resolve(strict=True)
    except OSError as exc:
        raise ValueError(
            "cannot prove the native numerical library is the NumPy "
            f"provider: {exc}"
        ) from exc
    if resolved_numpy != resolved_native:
        raise ValueError(
            "native numerical library differs from the NumPy provider: "
            f"{resolved_native} != {resolved_numpy}"
        )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_numerics_op_symbol(backend: str, op: str, name: str) -> None:
    """Pin both operation and integer ABI before accepting a provider name."""
    stem = op.removesuffix("_ilp64")
    if backend == "NUMPY_OPENBLAS":
        if op in (*_NUMERICS_BLAS_OPS, *_NUMERICS_BLAS_ILP64_OPS):
            expected = f"scipy_cblas_{stem}64_"
            interface = "scipy-openblas ILP64 CBLAS spelling"
        else:
            expected = f"scipy_{op}_64_"
            interface = "scipy-openblas ILP64 Fortran spelling"
    elif op in _NUMERICS_BLAS_OPS:
        expected = f"cblas_{op}"
        interface = "qualified legacy 32-bit CBLAS interface"
    elif op in _NUMERICS_BLAS_ILP64_OPS:
        expected = f"cblas_{stem}$NEWLAPACK$ILP64"
        interface = "Accelerate NEWLAPACK ILP64 CBLAS interface"
    else:
        expected = f"{op}$NEWLAPACK$ILP64"
        interface = "Accelerate NEWLAPACK ILP64 interface"
    if name != expected:
        raise ValueError(
            f"numerics symbol {name!r} for '{op}' is not the {interface}: "
            f"expected {expected!r}; it cannot back the qualified integer ABI"
        )


def validate_numerics(value: dict) -> dict:
    """Validate a ``numerics`` record; return it unchanged.

    Pure metadata validation — no symbol is invoked and nothing is executed
    here. Fail-closed on: unknown backend, an unqualified integer width
    (legacy BLAS not matching the backend's pinned width, LAPACK not 64-bit;
    _NUMERICS_BLAS_ILP64_OPS are 64-bit on every backend),
    missing/duplicate/unsafe symbol names, library paths outside the
    recorded NumPy distribution, fabricated digests, apple-system entries
    carrying a file hash, and threading records without a positive count.
    """
    if not isinstance(value, dict):
        raise ValueError(
            f"numerics record must be an object, found {value!r}"
        )
    backend = _require_string(value, "backend", "numerics.backend")
    if backend not in _NUMERICS_BACKENDS:
        raise ValueError(
            f"unsupported numerics backend {backend!r}; expected one of "
            f"{_NUMERICS_BACKENDS}"
        )

    # The integer-ABI check runs first: an unqualified width makes every
    # other field moot and is the failure this whole record exists to stop.
    blas_bits = value.get("blas_integer_bits")
    lapack_bits = value.get("lapack_integer_bits")
    need_blas, need_lapack = _NUMERICS_WIDTHS[backend]
    if (
        not isinstance(blas_bits, int)
        or isinstance(blas_bits, bool)
        or not isinstance(lapack_bits, int)
        or isinstance(lapack_bits, bool)
        or (blas_bits, lapack_bits) != (need_blas, need_lapack)
    ):
        raise ValueError(
            f"unqualified BLAS/LAPACK integer ABI for backend {backend}: "
            f"blas_integer_bits={blas_bits!r} "
            f"lapack_integer_bits={lapack_bits!r}; the qualified profile is "
            f"{need_blas}-bit BLAS with {need_lapack}-bit LAPACK"
        )

    _require_string(value, "numpy_version", "numerics.numpy_version")
    numpy_package = _require_path(
        value, "numpy_package", "numerics.numpy_package", directory=True
    )
    site_packages = _require_path(
        value, "site_packages", "numerics.site_packages", directory=True
    )
    site_resolved = site_packages.resolve()
    if numpy_package.resolve().parent != site_resolved:
        raise ValueError(
            "numerics.numpy_package is not inside the recorded "
            f"site_packages tree: {numpy_package} vs {site_packages} — the "
            "record must describe one installed distribution"
        )

    symbols = _require_object(value, "symbols")
    symbol_keys = set(symbols)
    if symbol_keys != set(_NUMERICS_OPS):
        missing = sorted(set(_NUMERICS_OPS) - symbol_keys)
        extra = sorted(symbol_keys - set(_NUMERICS_OPS))
        raise ValueError(
            f"numerics.symbols must map exactly the {len(_NUMERICS_OPS)} qualified "
            f"operations {_NUMERICS_OPS}; missing={missing} extra={extra}"
        )
    accepted = {
        op: checked_symbol(symbols[op]) for op in _NUMERICS_OPS
    }
    for op, name in accepted.items():
        _validate_numerics_op_symbol(backend, op, name)

    imports = _require_object(value, "numpy_imports")
    if set(imports) != set(_NUMERICS_OPS):
        raise ValueError(
            f"numerics.numpy_imports must map exactly the {len(_NUMERICS_OPS)} qualified "
            f"operations {_NUMERICS_OPS}"
        )
    for op in _NUMERICS_OPS:
        listed = imports[op]
        if not isinstance(listed, list):
            raise ValueError(
                f"numerics.numpy_imports.{op} must be a list, "
                f"found {listed!r}"
            )
        for name in listed:
            checked_symbol(name)
        if op not in _NUMERICS_BLAS_OPS and accepted[op] not in listed:
            raise ValueError(
                f"numerics symbol {accepted[op]!r} for '{op}' is not one "
                "of NumPy's own imports for that operation"
            )

    library_paths = value.get("library_paths")
    if not isinstance(library_paths, list) or not library_paths:
        raise ValueError(
            f"numerics.library_paths must be a nonempty list, "
            f"found {library_paths!r}"
        )
    seen_paths: set[Path] = set()
    covered_roles: set[str] = set()
    for index, entry in enumerate(library_paths):
        label = f"numerics.library_paths[{index}]"
        if not isinstance(entry, dict):
            raise ValueError(f"{label} must be an object, found {entry!r}")
        raw_path = _require_string(entry, "path", f"{label}.path")
        path = Path(raw_path)
        if not path.is_absolute():
            raise ValueError(f"{label}.path is not absolute: {raw_path}")
        role = _require_string(entry, "role", f"{label}.role")
        if role not in _NUMERICS_ROLES:
            raise ValueError(
                f"{label}.role must be one of {_NUMERICS_ROLES}, "
                f"found {role!r}"
            )
        identity = _require_string(
            entry, "identity_kind", f"{label}.identity_kind"
        )
        if identity not in _NUMERICS_IDENTITY_KINDS:
            raise ValueError(
                f"{label}.identity_kind must be one of "
                f"{_NUMERICS_IDENTITY_KINDS}, found {identity!r}"
            )
        sha = entry.get("sha256")
        build_id = entry.get("build_id")
        if identity == "file":
            if backend != "NUMPY_OPENBLAS":
                raise ValueError(
                    f"{label} uses a file identity under backend "
                    f"{backend} — only the wheel-backed OpenBLAS profile "
                    "has a hashable provider file"
                )
            if not path.is_file():
                raise ValueError(
                    f"{label}.path does not name an existing provider "
                    f"file: {raw_path}"
                )
            if not isinstance(sha, str) or _SHA256_HEX.fullmatch(sha) is None:
                raise ValueError(
                    f"{label}.sha256 must be a lowercase 64-hex digest, "
                    f"found {sha!r}"
                )
            actual = _sha256_file(path)
            if actual != sha:
                raise ValueError(
                    f"{label}.sha256 does not match the provider file: "
                    f"recorded {sha}, content hashes to {actual}"
                )
            if build_id is not None:
                raise ValueError(
                    f"{label}.build_id must be null for a file identity, "
                    f"found {build_id!r}"
                )
            resolved = path.resolve()
            if not resolved.is_relative_to(site_resolved):
                raise ValueError(
                    f"{label}.path is outside the installed NumPy "
                    f"distribution records: {resolved} is not under "
                    f"{site_resolved}"
                )
        else:  # apple-system
            if backend != "ACCELERATE":
                raise ValueError(
                    f"{label} uses an apple-system identity under backend "
                    f"{backend} — that identity is only legal for the "
                    "Darwin system framework"
                )
            if sha is not None:
                raise ValueError(
                    f"{label}.sha256 must be null for an apple-system "
                    "identity — a dyld-cache image is not a hashable "
                    f"provider file (got {sha!r})"
                )
            if not isinstance(build_id, str) or not build_id:
                raise ValueError(
                    f"{label}.build_id must be a nonempty string for an "
                    f"apple-system identity, found {build_id!r}"
                )
            if not raw_path.startswith(_ACCELERATE_IMAGE_PREFIX):
                raise ValueError(
                    f"{label}.path is not a system Accelerate.framework "
                    f"image: {raw_path}"
                )
            resolved = path  # dyld-cache path — no file to resolve
        if resolved in seen_paths:
            raise ValueError(
                f"duplicate numerical provider entry: {resolved} is "
                "listed twice"
            )
        seen_paths.add(resolved)
        covered_roles.update(role.split("_"))
    if not {"blas", "lapack"} <= covered_roles:
        raise ValueError(
            "numerics.library_paths does not cover both blas and lapack "
            f"roles: covered {sorted(covered_roles)}"
        )

    threading = _require_object(value, "threading")
    _require_string(
        threading, "implementation", "numerics.threading.implementation"
    )
    num_threads = threading.get("num_threads")
    if (
        not isinstance(num_threads, int)
        or isinstance(num_threads, bool)
        or num_threads < 1
    ):
        raise ValueError(
            "numerics.threading.num_threads must be a positive integer, "
            f"found {num_threads!r}"
        )

    interp = _require_object(value, "interp")
    lowering = _require_string(
        interp, "lowering", "numerics.interp.lowering"
    )
    if lowering not in ("contracted", "separate"):
        raise ValueError(
            f"numerics.interp.lowering must be 'contracted' or "
            f"'separate', found {lowering!r} — the probe classifies the "
            "wheel's np.interp multiply/add contraction and nothing else "
            "is qualified"
        )
    queries = interp.get("queries")
    distinguishing = interp.get("distinguishing")
    for label, counter in (("queries", queries),
                           ("distinguishing", distinguishing)):
        if (
            not isinstance(counter, int)
            or isinstance(counter, bool)
            or counter < 1
        ):
            raise ValueError(
                f"numerics.interp.{label} must be a positive integer, "
                f"found {counter!r}"
            )
    if distinguishing > queries or distinguishing < 8:
        raise ValueError(
            f"numerics.interp evidence is inconsistent: "
            f"distinguishing={distinguishing} queries={queries} — the "
            "lowering claim needs at least 8 distinguishing votes and "
            "cannot outnumber the queries cast"
        )

    multiply_broadcast = _require_object(value, "multiply_broadcast")
    nan_winner = _require_string(
        multiply_broadcast,
        "nan_winner",
        "numerics.multiply_broadcast.nan_winner",
    )
    if nan_winner not in ("scalar", "element"):
        raise ValueError(
            "numerics.multiply_broadcast.nan_winner must be 'scalar' "
            f"or 'element', found {nan_winner!r} — the probe classifies "
            "which operand's NaN wins a dual-NaN lane of "
            "multiply(scalar, array) and nothing else is qualified"
        )
    lanes = multiply_broadcast.get("lanes")
    mb_distinguishing = multiply_broadcast.get("distinguishing")
    for label, counter in (("lanes", lanes),
                           ("distinguishing", mb_distinguishing)):
        if (
            not isinstance(counter, int)
            or isinstance(counter, bool)
            or counter < 1
        ):
            raise ValueError(
                f"numerics.multiply_broadcast.{label} must be a "
                f"positive integer, found {counter!r}"
            )
    if mb_distinguishing > lanes or mb_distinguishing < 8:
        raise ValueError(
            "numerics.multiply_broadcast evidence is inconsistent: "
            f"distinguishing={mb_distinguishing} lanes={lanes} — the "
            "NaN-precedence claim needs at least 8 distinguishing "
            "lanes and cannot outnumber the lanes cast"
        )

    config = _require_object(value, "provider_config")
    if backend == "NUMPY_OPENBLAS":
        openblas = config.get("openblas")
        if not isinstance(openblas, dict):
            raise ValueError(
                "numerics.provider_config.openblas must be an object, "
                f"found {openblas!r}"
            )
        openblas_config = openblas.get("config")
        if (
            not isinstance(openblas_config, str)
            or "USE64BITINT" not in openblas_config
        ):
            raise ValueError(
                "numerics.provider_config.openblas.config must carry the "
                "USE64BITINT build flag — the provider-side evidence for "
                f"the 64-bit integer ABI, found {openblas_config!r}"
            )
        _require_string(openblas, "core", "numerics.provider_config.openblas.core")
        _require_string(
            openblas, "version", "numerics.provider_config.openblas.version"
        )
    else:
        for key in ("macos_build", "sdk_build", "accelerate_uuid"):
            _require_string(config, key, f"numerics.provider_config.{key}")
    return value


# --------------------------------------------------------------------------
# Document validation (pure stdlib; runs in any interpreter)
# --------------------------------------------------------------------------


def _require_object(document: dict, key: str) -> dict:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ValueError(
            f"dependency probe field '{key}' must be an object, "
            f"found {value!r}"
        )
    return value


def _require_string(record: dict, key: str, label: str) -> str:
    value = record.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(
            f"dependency probe field '{label}' must be a nonempty string, "
            f"found {value!r}"
        )
    return value


def _require_path(
    record: dict,
    key: str,
    label: str,
    *,
    directory: bool = False,
    executable: bool = False,
) -> Path:
    value = _require_string(record, key, label)
    path = Path(value)
    if not path.is_absolute():
        raise ValueError(
            f"dependency probe field '{label}' is not an absolute path: {value}"
        )
    if directory:
        if not path.is_dir():
            raise ValueError(
                f"dependency probe field '{label}' does not name an existing "
                f"directory: {value}"
            )
    elif not path.is_file():
        raise ValueError(
            f"dependency probe field '{label}' does not name an existing "
            f"file: {value}"
        )
    if executable and not os.access(path, os.X_OK):
        raise ValueError(
            f"dependency probe field '{label}' is not executable: {value}"
        )
    return path


def validate_dependencies(value: dict) -> dict:
    """Validate a schema-1 dependency document; return it unchanged.

    Fail-closed: wrong types, non-absolute or nonexistent paths, a
    non-executable interpreter, a foreign platform library name, or a MuJoCo
    version other than the 3.12.0 pin are all named ``ValueError``s. The
    ``numerics`` record is T06 scope and only type-checked here.
    """
    if not isinstance(value, dict):
        raise ValueError(
            f"dependency probe document must be a JSON object, found {value!r}"
        )
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported dependency probe schema_version "
            f"{value.get('schema_version')!r}; expected {SCHEMA_VERSION}"
        )

    platform_record = _require_object(value, "platform")
    system = _require_string(platform_record, "system", "platform.system")
    _require_string(platform_record, "machine", "platform.machine")
    libc = platform_record.get("libc")
    if libc is not None and not isinstance(libc, str):
        raise ValueError(
            f"dependency probe field 'platform.libc' must be a string or "
            f"null, found {libc!r}"
        )
    if system not in _MUJOCO_LIBRARY_CANDIDATES:
        raise ValueError(f"unsupported native platform: {system}")

    python_record = _require_object(value, "python")
    _require_path(
        python_record, "executable", "python.executable", executable=True
    )
    _require_string(python_record, "version", "python.version")
    _require_string(python_record, "soabi", "python.soabi")
    _require_path(
        python_record, "include_dir", "python.include_dir", directory=True
    )

    mujoco_record = _require_object(value, "mujoco")
    mujoco_version = _require_string(mujoco_record, "version", "mujoco.version")
    if mujoco_version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found "
            f"{mujoco_version}"
        )
    _require_path(
        mujoco_record, "include_dir", "mujoco.include_dir", directory=True
    )
    library = _require_path(
        mujoco_record, "library_path", "mujoco.library_path"
    )
    canonical = _MUJOCO_LIBRARY_CANDIDATES[system][0]
    if library.name != canonical:
        raise ValueError(
            f"dependency probe field 'mujoco.library_path' must resolve to "
            f"the {system} {REQUIRED_MUJOCO_VERSION} image {canonical}, found "
            f"{library}"
        )

    nanobind_record = _require_object(value, "nanobind")
    _require_path(
        nanobind_record, "cmake_dir", "nanobind.cmake_dir", directory=True
    )
    _require_path(
        nanobind_record, "include_dir", "nanobind.include_dir", directory=True
    )

    numerics = value.get("numerics")
    if numerics is not None:
        # A populated record must pass the full provider qualification —
        # it cannot be partially right. ``null`` stays acceptable here so
        # pre-T06 documents keep validating; ``doctor --numerics`` and the
        # CMake side are the consumers that require it non-null.
        validate_numerics(numerics)
    return value


# --------------------------------------------------------------------------
# Numerical-provider discovery — runs inside the SELECTED interpreter only
# --------------------------------------------------------------------------
#
# Layering contract: everything below this line executes inside the probe
# child, which is a disposable interpreter process. It imports NumPy (and
# SciPy once, deliberately, for the interposition control), enumerates the
# loaded numerical providers, inspects their export tables with platform
# tools, and only then compiles/links/runs a C probe against the qualified
# signatures in a *second* disposable subprocess. Metadata validation
# (validate_numerics) never calls foreign code; foreign invocation only ever
# sees identifiers that passed checked_symbol + provider export membership.

_NUMPY_INSTALL_INSTRUCTION = (
    "the native build requires the locked NumPy "
    f"{REQUIRED_NUMPY_VERSION} wheel — run `uv sync --locked --group "
    "native` (tools/dev.py bootstrap); a system/distribution NumPy is not "
    "qualified"
)

# Op-stem + recognised suffix forms: scipy-openblas spells ILP64 exports
# `scipy_<op>_64_` (Fortran) and `scipy_cblas_<op>64_` (CBLAS); LP64 drops the
# 64 markers; Accelerate spells its new interface `...$NEWLAPACK[$ILP64]` and
# legacy Fortran adds a trailing underscore. Matching is anchored at the end
# so kernel names like ``ddot_k_NEOVERSEN1`` or ``dgemmt`` never qualify.
_OP_SYMBOL_TAIL = re.compile(
    r"(ddot|dgemv|dgemm|dgelsd|dgesv|dsyevd|zdotu_sub)"
    r"(?:\$NEWLAPACK\$ILP64|\$NEWLAPACK|_64_|64_|_)?$"
)


def _run_tool(argv: list[str], *, timeout: int = 60) -> str:
    """Run an inspection tool; nonzero exits are named failures, and the
    tool itself must exist — PATH resolution is explicit, never implicit."""
    executable = shutil.which(argv[0])
    if executable is None:
        raise ValueError(
            f"required inspection tool {argv[0]!r} is not installed"
        )
    try:
        completed = subprocess.run(
            [executable, *argv[1:]],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(f"{argv[0]!r} probe failed: {exc}") from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "(no stderr)"
        raise ValueError(
            f"{argv[0]} {' '.join(argv[1:])} failed "
            f"(exit {completed.returncode}): {detail}"
        )
    return completed.stdout


def _elf_defined_symbols(path: Path) -> set[str]:
    """Exported dynamic symbols of an ELF image via ``nm -D``."""
    output = _run_tool(["nm", "-D", "--defined-only", str(path)])
    names: set[str] = set()
    for line in output.splitlines():
        parts = line.split()
        # `addr T name` or bare `T name` for zero-address/special entries
        if len(parts) >= 2 and parts[-2] in "TtWwDdBbRrVvIi":
            names.add(parts[-1])
    return names


def _elf_undefined_symbols(path: Path) -> set[str]:
    """Undefined dynamic imports of an ELF image via ``nm -D``."""
    output = _run_tool(["nm", "-D", "--undefined-only", str(path)])
    names: set[str] = set()
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in ("U", "u", "w"):
            names.add(parts[-1])
        elif len(parts) == 1:
            names.add(parts[0])
    return names


def _macho_defined_symbols(image_path: str) -> set[str]:
    """Exported symbols of a Mach-O image — works for dyld-cache paths that
    have no on-disk file by reading through ``dyld_info``. Returned names
    are stripped of the Mach-O leading underscore so records carry the
    ABI-level spelling."""
    output = _run_tool(["dyld_info", "-exports", image_path])
    names: set[str] = set()
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("0x"):
            name = parts[1]
            names.add(name[1:] if name.startswith("_") else name)
    return names


def _macho_undefined_symbols(path: Path) -> set[str]:
    """Undefined imports of a Mach-O extension via ``nm -u``; leading
    underscores stripped to the ABI-level spelling."""
    output = _run_tool(["nm", "-u", str(path)])
    names: set[str] = set()
    for line in output.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        if name.startswith("_"):
            name = name[1:]
        if SYMBOL.fullmatch(name) is not None:
            names.add(name)
    return names


def _dyld_loaded_images() -> dict[str, int]:
    """Map loaded-image install names to their mach_header addresses.

    Read-only process introspection through libSystem's public ``_dyld_*``
    API — the only way to name the dyld-cache images NumPy actually bound,
    since they do not exist as files on disk.
    """
    import ctypes

    lib = ctypes.CDLL(None)
    lib._dyld_image_count.restype = ctypes.c_uint32
    lib._dyld_get_image_name.restype = ctypes.c_char_p
    lib._dyld_get_image_name.argtypes = [ctypes.c_uint32]
    lib._dyld_get_image_header.restype = ctypes.c_void_p
    lib._dyld_get_image_header.argtypes = [ctypes.c_uint32]
    images: dict[str, int] = {}
    for index in range(lib._dyld_image_count()):
        raw = lib._dyld_get_image_name(index)
        if raw:
            header = lib._dyld_get_image_header(index)
            images[os.fsdecode(raw)] = int(header) if header else 0
    return images


def _macho_uuid_at(header_address: int) -> str | None:
    """LC_UUID of a loaded Mach-O image, parsed from its load commands."""
    import ctypes

    if not header_address:
        return None
    magic = ctypes.c_uint32.from_address(header_address).value
    if magic != 0xFEEDFACF:  # MH_MAGIC_64 — in-process images are thin
        return None
    ncmds = ctypes.c_uint32.from_address(header_address + 16).value
    offset = header_address + 32  # sizeof(mach_header_64)
    for _ in range(ncmds):
        command = ctypes.c_uint32.from_address(offset).value
        size = ctypes.c_uint32.from_address(offset + 4).value
        if size < 8:
            return None  # corrupt command table — never keep walking
        if command == 0x1B:  # LC_UUID
            raw = ctypes.string_at(offset + 8, 16).hex()
            return (
                f"{raw[0:8]}-{raw[8:12]}-{raw[12:16]}-"
                f"{raw[16:20]}-{raw[20:32]}"
            )
        offset += size
    return None


def _numpy_extension_imports(
    package: Path, system: str
) -> dict[str, list[str]]:
    """Which BLAS/LAPACK entry points NumPy's own extension images import.

    Evidence-driven symbol derivation: every ``*.so`` under the installed
    NumPy package is scanned for the qualified BLAS/LAPACK imports —
    the accepted names are what NumPy actually calls, not what a table of
    constants guesses.
    """
    undefined: set[str] = set()
    extensions = sorted(package.rglob("*.so"))
    if not extensions:
        raise ValueError(
            f"the installed NumPy package {package} has no extension "
            "modules to inspect — an incomplete or stub install cannot "
            "qualify a numerical provider"
        )
    for extension in extensions:
        if system == "Linux":
            undefined |= _elf_undefined_symbols(extension)
        else:
            undefined |= _macho_undefined_symbols(extension)
    per_op: dict[str, set[str]] = {op: set() for op in _NUMERICS_OPS}
    for name in undefined:
        match = _OP_SYMBOL_TAIL.search(name)
        if match is not None:
            op = match.group(1)
            per_op[op].add(name)
            # NumPy's ILP64 ddot import qualifies the raw-fold face too.
            # Do not let an LP64 import stand in for its 64-bit ABI.
            if op == "ddot" and name in (
                "cblas_ddot$NEWLAPACK$ILP64", "scipy_cblas_ddot64_"
            ):
                per_op["ddot_ilp64"].add(name)
    return {op: sorted(names) for op, names in per_op.items()}


def _jsonable(value):
    """Deep-convert numpy/tool output to JSON-safe data for the record."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        asdict = getattr(value, "_asdict", None)
        if callable(asdict):  # namedtuple (uname_result & friends)
            return _jsonable(asdict())
        return [_jsonable(item) for item in value]
    return str(value)


def _numpy_runtime_info(np):
    """``np.show_runtime`` prints its table to stdout — capture it so the
    child's stdout stays JSON-only while still recording the data."""
    import contextlib
    import io

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        result = np.show_runtime()
    return _jsonable(result if result is not None else buffer.getvalue())


def _numpy_build_dependencies(np) -> dict:
    """The blas/lapack entries of NumPy's own build-time config."""
    try:
        config = np.__config__.show(mode="dicts")
    except Exception as exc:
        raise ValueError(
            f"numpy.__config__.show(mode='dicts') failed: {exc}"
        ) from exc
    if not isinstance(config, dict):
        raise ValueError(
            "numpy.__config__ did not return a dict; cannot qualify the "
            "NumPy build"
        )
    dependencies = config.get("Build Dependencies")
    if not isinstance(dependencies, dict):
        raise ValueError(
            "NumPy build configuration has no 'Build Dependencies' "
            f"section: {sorted(config)}"
        )
    return dependencies


def _numpy_distribution():
    """The installed NumPy distribution: package files, RECORD hashes,
    wheel tags — everything needed to bind a provider path to the wheel."""
    import importlib.metadata

    try:
        distribution = importlib.metadata.distribution("numpy")
    except importlib.metadata.PackageNotFoundError as exc:
        raise ValueError(
            "NumPy is not an installed distribution — "
            + _NUMPY_INSTALL_INSTRUCTION
        ) from exc
    files = distribution.files
    if not files:
        raise ValueError(
            "the installed NumPy distribution has no file records — an "
            "editable or non-wheel install cannot qualify a provider; "
            + _NUMPY_INSTALL_INSTRUCTION
        )
    members: dict[str, Path] = {}
    for entry in files:
        located = Path(distribution.locate_file(str(entry))).resolve()
        members[str(entry)] = located
    site_packages = Path(distribution.locate_file("")).resolve()
    wheel_text = distribution.read_text("WHEEL") or ""
    tags = [
        line.split(":", 1)[1].strip()
        for line in wheel_text.splitlines()
        if line.startswith("Tag:")
    ]
    record_hashes: dict[str, tuple[str, int | None]] = {}
    record_entry = next(
        (name for name in members if name.endswith(".dist-info/RECORD")),
        None,
    )
    if record_entry is not None:
        record_file = members[record_entry]
        for raw in record_file.read_text().splitlines():
            if ",sha256=" not in raw:
                continue
            path_field, hash_field, size_field = raw.rsplit(",", 2)
            record_hashes[path_field] = (
                hash_field.removeprefix("sha256="),
                int(size_field) if size_field.isdigit() else None,
            )
    return distribution, members, record_hashes, site_packages, tags


def _numerics_abi_probe_source(backend: str, symbols: dict) -> str:
    """The generated C probe: qualified integer widths and exact exported
    symbols land inside ``__asm__`` labels — nothing is retyped from
    memory, and every symbol passed ``checked_symbol`` first."""
    if backend == "ACCELERATE":
        blas_int = "int32_t"
        label_prefix = "_"  # Mach-O symbol prefix
    elif backend == "NUMPY_OPENBLAS":
        blas_int = "int64_t"
        label_prefix = ""   # ELF symbols carry no leading underscore
    else:
        raise ValueError(
            f"cannot generate an ABI probe for unqualified backend "
            f"{backend!r}"
        )
    qualified = {
        op: label_prefix + checked_symbol(symbols[op])
        for op in _NUMERICS_OPS
    }
    blas_bits = 4 if backend == "ACCELERATE" else 8
    return f"""/* bike numerics ABI probe — generated by tools/native_dependencies.py.
 * Compile, link and run may only happen in a disposable subprocess: a
 * declared-but-wrong integer width is memory corruption, never a stub.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef {blas_int} blas_int;
typedef int64_t blas64_int;
typedef int64_t lapack_int;
_Static_assert(sizeof(blas_int) == {blas_bits},
               "BLAS integer width does not match the qualified ABI");
_Static_assert(sizeof(lapack_int) == 8,
               "LAPACK integer width does not match the qualified ABI");
_Static_assert(sizeof(blas64_int) == 8,
               "ILP64 dot integer width must remain 64-bit on every backend");

extern double probe_ddot(blas_int n, const double *x, blas_int incx,
                         const double *y, blas_int incy)
    __asm__("{qualified['ddot']}");
extern double probe_ddot_ilp64(blas64_int n, const double *x, blas64_int incx,
                               const double *y, blas64_int incy)
    __asm__("{qualified['ddot_ilp64']}");
extern void probe_zdotu_sub(blas64_int n, const void *x, blas64_int incx,
                             const void *y, blas64_int incy, void *out)
    __asm__("{qualified['zdotu_sub']}");
extern void probe_dgemv(blas_int order, blas_int transa, blas_int m,
                        blas_int n, double alpha, const double *a,
                        blas_int lda, const double *x, blas_int incx,
                        double beta, double *y, blas_int incy)
    __asm__("{qualified['dgemv']}");
extern void probe_dgemm(blas_int order, blas_int transa, blas_int transb,
                        blas_int m, blas_int n, blas_int k, double alpha,
                        const double *a, blas_int lda, const double *b,
                        blas_int ldb, double beta, double *c, blas_int ldc)
    __asm__("{qualified['dgemm']}");
extern void probe_dgelsd(const lapack_int *m, const lapack_int *n,
                         const lapack_int *nrhs, double *a,
                         const lapack_int *lda, double *b,
                         const lapack_int *ldb, double *s,
                         const double *rcond, lapack_int *rank,
                         double *work, const lapack_int *lwork,
                         lapack_int *iwork, lapack_int *info)
    __asm__("{qualified['dgelsd']}");
extern void probe_dgesv(const lapack_int *n, const lapack_int *nrhs,
                        double *a, const lapack_int *lda, lapack_int *ipiv,
                        double *b, const lapack_int *ldb, lapack_int *info)
    __asm__("{qualified['dgesv']}");
extern void probe_dsyevd(const char *jobz, const char *uplo,
                         const lapack_int *n, double *a, const lapack_int *lda,
                         double *w, double *work, const lapack_int *lwork,
                         lapack_int *iwork, const lapack_int *liwork,
                         lapack_int *info)
    __asm__("{qualified['dsyevd']}");

static int failures = 0;
static void check(int ok, const char *name) {{
    if (!ok) {{
        ++failures;
        fprintf(stderr, "BIKE-ABI-FAIL %s\\n", name);
    }}
}}

int main(void) {{
    /* dot: [1 2 3] . [4 5 6] == 32 */
    double x3[3] = {{1., 2., 3.}};
    double y3[3] = {{4., 5., 6.}};
    check(probe_ddot(3, x3, 1, y3, 1) == 32., "ddot");
    check(probe_ddot_ilp64(3, x3, 1, y3, 1) == 32., "ddot_ilp64");
    /* Non-conjugating complex dot: -18 + 92i, exact in binary64. */
    double zx[4] = {{1., 2., 5., 6.}}, zy[4] = {{3., 4., 7., 8.}};
    double zout[2] = {{0., 0.}};
    probe_zdotu_sub(2, zx, 1, zy, 1, zout);
    check(zout[0] == -18. && zout[1] == 92., "zdotu_sub");

    /* rectangular dgemv, row-major 3x4 NoTrans: row sums {{10, 26, 42}} */
    double a34[12] = {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}};
    double x4[4] = {{1, 1, 1, 1}};
    double yv[3] = {{0, 0, 0}};
    probe_dgemv(101, 111, 3, 4, 1.0, a34, 4, x4, 1, 0.0, yv, 1);
    check(yv[0] == 10. && yv[1] == 26. && yv[2] == 42., "dgemv");

    /* rectangular dgemm, row-major (2x3)*(3x2) -> [[4,5],[10,11]] */
    double a23[6] = {{1, 2, 3, 4, 5, 6}};
    double b32[6] = {{1, 0, 0, 1, 1, 1}};
    double c22[4] = {{0, 0, 0, 0}};
    probe_dgemm(101, 111, 111, 2, 2, 3, 1.0, a23, 3, b32, 2, 0.0, c22, 2);
    check(c22[0] == 4. && c22[1] == 5. && c22[2] == 10. && c22[3] == 11.,
          "dgemm");

    /* dgelsd: workspace query, then a rank-1-deficient 3x2 solve.
       A = cols (1,2,3) and (2,4,6); b = 3*col1 -> min-norm x = (0.6, 1.2). */
    lapack_int m = 3, n = 2, nrhs = 1, lda = 3, ldb = 3;
    lapack_int info = 0, rank = 0;
    double a[6] = {{1, 2, 3, 2, 4, 6}};
    double b[3] = {{3, 6, 9}};
    double s[2];
    double rcond = -1.;
    lapack_int lwork = -1;
    double work_size = 0.;
    lapack_int iwork_size = 0;
    probe_dgelsd(&m, &n, &nrhs, a, &lda, b, &ldb, s, &rcond, &rank,
                 &work_size, &lwork, &iwork_size, &info);
    check(info == 0, "dgelsd.workspace_query");
    check(work_size > 0. && iwork_size > 0, "dgelsd.workspace_sizes");
    if (info != 0 || !isfinite(work_size) || work_size < 1. ||
        work_size > 10000000. || iwork_size < 1 || iwork_size > 10000000) {{
        return 1;
    }}
    lwork = (lapack_int) work_size;
    double *work = malloc(sizeof(double) * (size_t) lwork);
    lapack_int *iwork = malloc(sizeof(lapack_int) * (size_t) iwork_size);
    check(work != NULL && iwork != NULL, "dgelsd.workspace_alloc");
    if (work == NULL || iwork == NULL) {{
        free(work); free(iwork);
        return 1;
    }}
    a[0] = 1; a[1] = 2; a[2] = 3; a[3] = 2; a[4] = 4; a[5] = 6;
    b[0] = 3; b[1] = 6; b[2] = 9;
    info = 0; rank = 0;
    probe_dgelsd(&m, &n, &nrhs, a, &lda, b, &ldb, s, &rcond, &rank,
                 work, &lwork, iwork, &info);
    check(info == 0, "dgelsd.solve");
    check(rank == 1, "dgelsd.rank");
    check(fabs(b[0] - 0.6) < 1e-9 && fabs(b[1] - 1.2) < 1e-9,
          "dgelsd.solution");
    free(work);
    free(iwork);

    /* DGESV: [[2,1],[1,3]] x = [4,7], exact solution [1,2]. */
    lapack_int ns = 2, rhs = 1, ld = 2, solve_info = -1;
    lapack_int pivots[2] = {{0, 0}};
    double solve_a[4] = {{2., 1., 1., 3.}}, solve_b[2] = {{4., 7.}};
    probe_dgesv(&ns, &rhs, solve_a, &ld, pivots, solve_b, &ld, &solve_info);
    check(solve_info == 0 && solve_b[0] == 1. && solve_b[1] == 2., "dgesv");

    /* DSYEVD: symmetric [[2,1],[1,2]], eigenvalues 1 and 3. */
    double eigen_a[4] = {{2., 1., 1., 2.}}, eigenvalues[2] = {{0., 0.}};
    double eigen_query = 0.;
    lapack_int eigen_lwork = -1, eigen_liwork = -1, eigen_iquery = 0;
    lapack_int eigen_info = -1;
    probe_dsyevd("V", "L", &ns, eigen_a, &ld, eigenvalues, &eigen_query,
                  &eigen_lwork, &eigen_iquery, &eigen_liwork, &eigen_info);
    check(eigen_info == 0, "dsyevd.workspace_query");
    if (eigen_info != 0 || !isfinite(eigen_query) || eigen_query < 1. ||
        eigen_query > 10000000. || eigen_iquery < 1 || eigen_iquery > 10000000) {{
        return 1;
    }}
    eigen_lwork = (lapack_int) eigen_query;
    eigen_liwork = eigen_iquery;
    double *eigen_work = malloc(sizeof(double) * (size_t) eigen_lwork);
    lapack_int *eigen_iwork = malloc(sizeof(lapack_int) * (size_t) eigen_liwork);
    check(eigen_work != NULL && eigen_iwork != NULL, "dsyevd.workspace_alloc");
    if (eigen_work == NULL || eigen_iwork == NULL) {{
        free(eigen_work); free(eigen_iwork);
        return 1;
    }}
    probe_dsyevd("V", "L", &ns, eigen_a, &ld, eigenvalues, eigen_work,
                  &eigen_lwork, eigen_iwork, &eigen_liwork, &eigen_info);
    check(eigen_info == 0 && fabs(eigenvalues[0] - 1.) < 1e-14 &&
          fabs(eigenvalues[1] - 3.) < 1e-14, "dsyevd");
    free(eigen_work);
    free(eigen_iwork);
    printf("BIKE-ABI-OK ddot=32 dgemv=[10 26 42] dgemm=[4 5;10 11] "
           "ddot_ilp64=32 zdotu_sub=[-18 92] dgesv=[1 2] dsyevd=[1 3] "
           "dgelsd_rank=%lld x=[%g %g]\\n",
           (long long) rank, b[0], b[1]);
    return failures ? 1 : 0;
}}
"""


def _verify_abi_compile_link_run(
    backend: str, symbols: dict, provider_path: Path | None
) -> dict:
    """Compile-only type check, then compile+link+run in a disposable
    subprocess. Any failure — compiler diagnostics, unresolved symbols, a
    crash, a wrong result — is a hard provider failure; there is no
    'probably right' state for an integer ABI."""
    compiler_name = os.environ.get("CC") or "cc"
    compiler = shutil.which(compiler_name)
    if compiler is None:
        raise ValueError(
            f"cannot verify the numerical ABI: C compiler "
            f"{compiler_name!r} is not installed"
        )
    source = _numerics_abi_probe_source(backend, symbols)
    with tempfile.TemporaryDirectory(prefix="bike-numerics-") as workdir:
        work = Path(workdir)
        source_file = work / "abi_probe.c"
        binary = work / "abi_probe"
        source_file.write_text(source, encoding="utf-8")

        def _stage(
            argv: list[str], stage: str, env: dict | None = None
        ) -> subprocess.CompletedProcess:
            try:
                completed = subprocess.run(
                    argv, capture_output=True, text=True, timeout=120,
                    check=False, cwd=workdir, env=env,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise ValueError(
                    f"numerics ABI {stage} could not run: {exc}"
                ) from exc
            if completed.returncode != 0:
                raise ValueError(
                    f"numerics ABI {stage} failed (exit "
                    f"{completed.returncode}): "
                    f"{(completed.stderr or completed.stdout).strip()}"
                )
            return completed

        _stage(
            [
                compiler, "-fsyntax-only", "-std=c11",
                "-Wall", "-Wextra", "-Werror", str(source_file),
            ],
            "compile-only type check",
        )
        if backend == "ACCELERATE":
            link_argv = [
                compiler, "-std=c11", "-O2", str(source_file),
                "-o", str(binary), "-framework", "Accelerate",
            ]
            run_env = None
        else:
            if provider_path is None:
                raise ValueError(
                    "the NUMPY_OPENBLAS ABI check requires the provider "
                    "library path"
                )
            link_argv = [
                compiler, "-std=c11", "-O2", str(source_file),
                str(provider_path),
                "-Wl,-rpath," + str(provider_path.parent),
                "-o", str(binary),
            ]
            run_env = dict(os.environ)
            run_env["LD_LIBRARY_PATH"] = (
                str(provider_path.parent)
                + os.pathsep
                + run_env.get("LD_LIBRARY_PATH", "")
            )
        _stage(link_argv, "link check")
        completed = _stage([str(binary)], "run check", env=run_env)
        if "BIKE-ABI-OK" not in completed.stdout:
            raise ValueError(
                "numerics ABI run check did not report success: "
                f"{completed.stdout.strip()}"
            )
    version = _run_tool([compiler, "--version"]).splitlines()[0]
    return {
        "compiler": version,
        "compile_only": "pass",
        "link_run": "pass",
        "output": completed.stdout.strip(),
    }


def _exercise_numpy(np):
    """Deterministic dot/lstsq results — bitwise-comparable across the
    SciPy-interposition boundary."""
    a = np.arange(72, dtype=np.float64).reshape(8, 9) * 0.03125
    b = np.arange(45, dtype=np.float64).reshape(9, 5) * -0.0625
    product = a @ b
    solution, residuals, rank, singular = np.linalg.lstsq(
        a, np.linspace(1.0, 8.0, 8), rcond=None
    )
    return (
        product.tobytes(),
        solution.tobytes(),
        residuals.tobytes(),
        rank,
        singular.tobytes(),
    )


def _measure_np_interp_lowering(np) -> dict:
    """Classify how this wheel lowers np.interp's interior expression.

    ``compiled_base.c`` computes ``slope*(x - dx[j]) + dy[j]`` once per
    wheel build; whether the multiply and the add round separately or
    contract into a single fmadd is a property of the wheel's compiler
    flags, not of this machine — the only trustworthy value is a
    measurement of the installed ``np.interp`` itself. Deterministic
    inputs only: every query that distinguishes the two candidate
    lowerings must vote unanimously, and too few distinguishing queries
    is a probe failure, never a guess.
    """
    import math

    # Strictly increasing knots; queries strictly interior so the C
    # binary_search always lands on the slope branch — never the knot,
    # clamp, or NaN-query shortcuts. Bitwise comparison classifies every
    # query: +0.0/-0.0 counts as distinguishing.
    knots = np.asarray([0.25, 0.375, 0.5, 0.625, 0.75])
    values = np.asarray(
        [0.37500000000000006, -0.12500000000000025, 0.6875000000000001,
         -0.4374999999999998, 0.24999999999999983]
    )
    rng = np.random.default_rng(0x1EE7C0DE)
    queries = rng.uniform(0.2501, 0.7499, 4096)
    expected = np.interp(queries, knots, values)

    fused = separate = distinguishing = 0
    for x, want in zip(queries.tolist(), expected.tolist()):
        j = int(np.searchsorted(knots, x, side="right")) - 1
        if not 0 <= j < knots.size - 1 or knots[j] == x:
            continue
        slope = (values[j + 1] - values[j]) / (knots[j + 1] - knots[j])
        f = math.fma(slope, x - knots[j], values[j])
        s = slope * (x - knots[j]) + values[j]
        f_bits = np.float64(f).view(np.uint64).item()
        s_bits = np.float64(s).view(np.uint64).item()
        if f_bits == s_bits or want != want:
            continue
        distinguishing += 1
        want_bits = np.float64(want).view(np.uint64).item()
        if want_bits == f_bits:
            fused += 1
        elif want_bits == s_bits:
            separate += 1
        else:
            raise ValueError(
                "np.interp produced a result matching neither the "
                f"contracted nor the separate lowering: x={x!r} "
                f"want={want!r} fma={f!r} separate={s!r} — this wheel's "
                "arr_interp is not either candidate expression"
            )
    if distinguishing < 8:
        raise ValueError(
            f"np.interp lowering probe produced only {distinguishing} "
            "distinguishing queries — cannot classify the wheel's "
            "contraction contract from this little evidence"
        )
    if fused == distinguishing:
        lowering = "contracted"
    elif separate == distinguishing:
        lowering = "separate"
    else:
        raise ValueError(
            f"np.interp lowering is inconsistent: {fused} queries "
            f"contracted vs {separate} separate over {distinguishing} "
            "distinguishing cases — the wheel mixes both forms"
        )
    return {
        "lowering": lowering,
        "queries": len(queries),
        "distinguishing": distinguishing,
    }


def _measure_np_multiply_broadcast_nan(np) -> dict:
    """Classify which operand wins a dual-NaN lane of multiply(scalar, array).

    ``a * b`` on AArch64 propagates operand 0's NaN (quieted) when both
    operands carry NaN payloads, and which source operand the ufunc's
    broadcast kernel loads into fmul operand 0 is a property of the
    wheel's compiled inner loop, not of this machine: the AppleClang
    wheel keeps the scalar first while the manylinux wheel's broadcast
    loop puts the contiguous array there. The only trustworthy verdict
    is a measurement of the installed ``np.multiply`` — the scalar's
    NaN wins (``scalar``) or the element's does (``element``); anything
    else is a probe failure, never a guess.
    """
    pos = float("nan")   # +qNaN, 0x7ff8000000000000
    neg = -float("nan")  # -qNaN, 0xfff8000000000000
    lane_count = 64
    arr = np.full(lane_count, neg)

    # Both spellings of the oracle expression must agree: the ufunc
    # call and the Python `float * ndarray` sugar (which dispatches
    # through ndarray.__rmul__ to the same multiply).
    direct = np.multiply(pos, arr).view(np.uint64)
    sugar = (pos * arr).view(np.uint64)
    if not (direct == sugar).all():
        raise ValueError(
            "np.multiply and `float * ndarray` disagree on the same "
            "dual-NaN broadcast — the wheel's kernels are inconsistent"
        )
    scalar_bits = np.float64(pos).view(np.uint64).item()
    element_bits = np.float64(neg).view(np.uint64).item()
    scalar_votes = int((direct == scalar_bits).sum())
    element_votes = int((direct == element_bits).sum())
    if scalar_votes + element_votes != lane_count:
        raise ValueError(
            "multiply broadcast produced lanes matching neither "
            f"operand's NaN: {scalar_votes} scalar, {element_votes} "
            f"element out of {lane_count} — the kernel is not either "
            "candidate operand order"
        )
    if scalar_votes == lane_count:
        winner = "scalar"
    elif element_votes == lane_count:
        winner = "element"
    else:
        raise ValueError(
            f"multiply broadcast NaN precedence is inconsistent: "
            f"{scalar_votes} lanes scalar-first vs {element_votes} "
            "element-first — the kernel mixes both orders"
        )
    return {
        "nan_winner": winner,
        "lanes": lane_count,
        "distinguishing": lane_count,
    }


def _blas_pool(threadpoolctl) -> list[dict]:
    return [
        entry
        for entry in threadpoolctl.threadpool_info()
        if isinstance(entry, dict) and entry.get("user_api") == "blas"
    ]


def _discover_numpy_provider_linux(np, threadpoolctl, package: Path,
                                   site_packages: Path,
                                   build_dependencies: dict) -> dict:
    """The Linux profile: the single ILP64 scipy-openblas shared object the
    NumPy wheel ships — proven by wheel RECORD membership, a matching file
    digest, the USE64BITINT build flag, and the exact ``scipy_*64_``
    export/imports sets."""
    blas_dep = build_dependencies.get("blas")
    lapack_dep = build_dependencies.get("lapack")
    blas_name = blas_dep.get("name") if isinstance(blas_dep, dict) else None
    lapack_name = (
        lapack_dep.get("name") if isinstance(lapack_dep, dict) else None
    )
    if blas_name != "scipy-openblas" or lapack_name != "scipy-openblas":
        raise ValueError(
            f"NumPy {np.__version__} was built against "
            f"blas={blas_name!r}/lapack={lapack_name!r}, not the bundled "
            "scipy-openblas profile — " + _NUMPY_INSTALL_INSTRUCTION
        )
    for label in ("blas", "lapack"):
        entry = build_dependencies[label]
        config_text = entry.get("openblas configuration")
        if not isinstance(config_text, str):
            raise ValueError(
                f"NumPy build dependencies.{label} carries no OpenBLAS "
                "configuration string — cannot prove the integer ABI"
            )
        if "USE64BITINT" not in config_text:
            raise ValueError(
                f"the NumPy wheel's {label} OpenBLAS was not built with "
                "USE64BITINT — an LP64 provider cannot qualify the 64-bit "
                "integer ABI profile; " + _NUMPY_INSTALL_INSTRUCTION
            )

    pool = _blas_pool(threadpoolctl)
    if not pool:
        raise ValueError(
            "no BLAS provider is loaded after exercising NumPy — cannot "
            "qualify what the process did not load"
        )
    distribution, members, record_hashes, dist_site, wheel_tags = (
        _numpy_distribution()
    )
    owned = [
        entry
        for entry in pool
        if isinstance(entry.get("filepath"), str)
        and Path(entry["filepath"]).resolve() in set(members.values())
    ]
    foreign = [entry for entry in pool if entry not in owned]
    if not owned:
        raise ValueError(
            "the loaded BLAS provider "
            f"{sorted(e.get('filepath') for e in pool)} is not part of the "
            "installed NumPy wheel's records — " + _NUMPY_INSTALL_INSTRUCTION
        )
    if len(owned) > 1:
        raise ValueError(
            "duplicate NumPy BLAS providers are loaded: "
            f"{sorted(e['filepath'] for e in owned)}"
        )
    entry = owned[0]
    provider_path = Path(entry["filepath"]).resolve()
    if entry.get("internal_api") != "openblas":
        raise ValueError(
            f"the NumPy BLAS provider reports internal_api "
            f"{entry.get('internal_api')!r}, not openblas — cannot "
            "qualify an unknown provider ABI"
        )
    record_relative = next(
        rel for rel, located in members.items() if located == provider_path
    )
    require_same_library(provider_path, members[record_relative])
    digest = _sha256_file(provider_path)
    declared = record_hashes.get(record_relative)
    if declared is not None:
        actual_b64 = base64.urlsafe_b64encode(
            bytes.fromhex(digest)
        ).decode().rstrip("=")
        if declared[0] != actual_b64:
            raise ValueError(
                f"provider {provider_path} content does not match the "
                "NumPy wheel RECORD hash — the installed file is not the "
                "shipped wheel image"
            )
        if declared[1] is not None and declared[1] != provider_path.stat().st_size:
            raise ValueError(
                f"provider {provider_path} size does not match the NumPy "
                "wheel RECORD size"
            )

    exports = _elf_defined_symbols(provider_path)
    imports = _numpy_extension_imports(package, "Linux")
    symbols: dict[str, str] = {}
    for op in _NUMERICS_OPS:
        imported = imports.get(op) or []
        candidates = sorted(name for name in imported if name in exports)
        if not candidates:
            raise ValueError(
                f"no provider export satisfies NumPy's '{op}' imports "
                f"{imported} — the library cannot serve the qualified "
                "operation"
            )
        preferred = (
            f"scipy_cblas_{op.removesuffix('_ilp64')}64_"
            if op in (*_NUMERICS_BLAS_OPS, *_NUMERICS_BLAS_ILP64_OPS)
            else f"scipy_{op}_64_"
        )
        if preferred in candidates:
            accepted = preferred
        elif len(candidates) == 1 and candidates[0].endswith("64_"):
            accepted = candidates[0]
        else:
            raise ValueError(
                f"ambiguous '{op}' symbol set {candidates}: no unique "
                "scipy-openblas ILP64 spelling — refusing to guess an "
                "integer ABI"
            )
        if not accepted.endswith("64_"):
            raise ValueError(
                f"NumPy imports '{accepted}' for '{op}' — an LP64 "
                "spelling cannot back the 64-bit integer ABI profile"
            )
        symbols[op] = checked_symbol(accepted)
        _validate_numerics_op_symbol("NUMPY_OPENBLAS", op, symbols[op])

    # Foreign-code boundary: the query symbols are regex-validated AND
    # verified members of the provider's export table before ctypes sees
    # them — never an assumed spelling.
    import ctypes

    provider = ctypes.CDLL(str(provider_path))

    def _query(name: str, restype):
        symbol = checked_symbol(name)
        if symbol not in exports:
            raise ValueError(
                f"provider {provider_path} does not export required "
                f"configuration symbol {symbol!r}"
            )
        function = getattr(provider, symbol)
        function.restype = restype
        function.argtypes = []
        return function()

    openblas_config = _query(
        "scipy_openblas_get_config64_", ctypes.c_char_p
    )
    openblas_core = _query(
        "scipy_openblas_get_corename64_", ctypes.c_char_p
    )
    openblas_threads = _query(
        "scipy_openblas_get_num_threads64_", ctypes.c_int
    )
    if openblas_config is None:
        raise ValueError(
            "openblas_get_config returned NULL — the provider cannot "
            "describe its own build"
        )
    config_text = openblas_config.decode("utf-8", "replace")
    if "USE64BITINT" not in config_text:
        raise ValueError(
            f"the loaded provider reports {config_text!r} — without "
            "USE64BITINT the 64-bit integer ABI claim has no support"
        )
    num_threads = entry.get("num_threads")
    if not isinstance(num_threads, int) or num_threads < 1:
        raise ValueError(
            f"threadpoolctl reports invalid num_threads {num_threads!r}"
        )
    if openblas_threads != num_threads:
        raise ValueError(
            f"OpenBLAS reports {openblas_threads} threads but "
            f"threadpoolctl observed {num_threads} — inconsistent "
            "provider state"
        )

    abi_check = _verify_abi_compile_link_run(
        "NUMPY_OPENBLAS", symbols, provider_path
    )

    record = {
        "backend": "NUMPY_OPENBLAS",
        "numpy_version": str(np.__version__),
        "numpy_package": str(package),
        "site_packages": str(site_packages),
        "blas_integer_bits": 64,
        "lapack_integer_bits": 64,
        "symbols": symbols,
        "numpy_imports": {op: list(imports.get(op, [])) for op in _NUMERICS_OPS},
        "library_paths": [
            {
                "path": str(provider_path),
                "role": "blas_lapack",
                "identity_kind": "file",
                "sha256": digest,
                "build_id": None,
            }
        ],
        "threading": {
            "implementation": str(entry.get("threading_layer") or "openblas"),
            "num_threads": num_threads,
        },
        "provider_config": {
            "openblas": {
                "version": str(entry.get("version") or ""),
                "core": openblas_core.decode("utf-8", "replace")
                if isinstance(openblas_core, bytes)
                else str(openblas_core),
                "config": config_text,
                "record_path": record_relative,
                "num_threads_reported": int(openblas_threads),
            },
            "wheel": {"tags": wheel_tags},
            "foreign_providers_loaded": _jsonable(foreign),
            "numpy_show_config": _jsonable(np.__config__.CONFIG)
            if isinstance(getattr(np.__config__, "CONFIG", None), dict)
            else {},
            "numpy_show_runtime": _numpy_runtime_info(np),
            "abi_check": abi_check,
        },
    }
    return record


def _measure_accelerate_threads(np) -> dict:
    """Accelerate has no threadpool query API — count the threads it
    actually spawns for a fixed dgemm workload. The number is measured
    evidence (workload + method recorded), never a guessed constant."""
    import threading
    import time

    def _thread_count() -> int:
        output = _run_tool(
            ["ps", "-M", "-o", "comm=", "-p", str(os.getpid())]
        )
        return sum(1 for line in output.splitlines() if line.strip())

    baseline = _thread_count()
    workload = np.full((3000, 3000), 1.25, dtype=np.float64)
    _ = workload @ workload  # warm-up: spawn the pool before measuring
    stop = threading.Event()

    def _work() -> None:
        while not stop.is_set():
            _ = workload @ workload

    worker = threading.Thread(target=_work, daemon=True)
    worker.start()
    peak = baseline
    try:
        for _ in range(40):
            time.sleep(0.05)
            peak = max(peak, _thread_count())
    finally:
        stop.set()
        worker.join(timeout=30)
    spawned = peak - baseline
    if spawned < 0:
        raise ValueError(
            "thread measurement is inconsistent: "
            f"baseline {baseline} > peak {peak}"
        )
    return {
        "implementation": "accelerate",
        "num_threads": max(1, spawned + 1),
        "measured": {
            "baseline_threads": baseline,
            "peak_threads": peak,
            "workload": "float64 dgemm 3000x3000, 40 x 50ms samples",
            "veclib_maximum_threads": os.environ.get(
                "VECLIB_MAXIMUM_THREADS"
            ),
        },
    }


def _accelerate_build_id(macos_build: str, uuid: str, sdk_build: str) -> str:
    """The build_id formula, spelled once in Python in
    src/bike_sim/native/provider_build_id.py. That module is loaded by path:
    this checker is stdlib-only and must not import the bike_sim package,
    whose __init__ pulls in numpy and mujoco."""
    import importlib.util

    module_path = (_SELF.parents[1] / "src" / "bike_sim" / "native"
                   / "provider_build_id.py")
    spec = importlib.util.spec_from_file_location(
        "_bike_provider_build_id", module_path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot load the build_id formula from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.accelerate_build_id(macos_build, uuid, sdk_build)


def _discover_numpy_provider_darwin(np, threadpoolctl, package: Path,
                                    site_packages: Path,
                                    build_dependencies: dict) -> dict:
    """The Darwin profile: the system Accelerate framework NumPy links —
    dyld-cache images qualified by the loaded-provider list, real export
    tables, the macOS/SDK build identity, and a live ABI run."""
    blas_dep = build_dependencies.get("blas")
    lapack_dep = build_dependencies.get("lapack")
    blas_name = blas_dep.get("name") if isinstance(blas_dep, dict) else None
    lapack_name = (
        lapack_dep.get("name") if isinstance(lapack_dep, dict) else None
    )
    if blas_name != "accelerate" or lapack_name != "accelerate":
        raise ValueError(
            f"NumPy {np.__version__} was built against "
            f"blas={blas_name!r}/lapack={lapack_name!r}, not the system "
            "Accelerate framework — " + _NUMPY_INSTALL_INSTRUCTION
        )

    images = _dyld_loaded_images()
    blas_image = _ACCELERATE_IMAGE_PREFIX + (
        "Versions/A/Frameworks/vecLib.framework/Versions/A/libBLAS.dylib"
    )
    lapack_image = _ACCELERATE_IMAGE_PREFIX + (
        "Versions/A/Frameworks/vecLib.framework/Versions/A/libLAPACK.dylib"
    )
    umbrella = _ACCELERATE_IMAGE_PREFIX + "Versions/A/Accelerate"
    loaded = [name for name in images if name.startswith(_ACCELERATE_IMAGE_PREFIX)]
    for required in (blas_image, lapack_image, umbrella):
        if required not in images:
            raise ValueError(
                f"the Accelerate provider image {required} is not loaded "
                f"after exercising NumPy — loaded images: {loaded}"
            )
        if Path(required).is_file():
            # Modern macOS keeps these in the dyld cache only; a real file
            # would be noteworthy but the identity rule stays apple-system.
            pass

    blas_exports = _macho_defined_symbols(blas_image)
    lapack_exports = _macho_defined_symbols(lapack_image)
    symbols = {
        "ddot": "cblas_ddot",
        "dgemv": "cblas_dgemv",
        "dgemm": "cblas_dgemm",
        "dgelsd": "dgelsd$NEWLAPACK$ILP64",
        "dgesv": "dgesv$NEWLAPACK$ILP64",
        "dsyevd": "dsyevd$NEWLAPACK$ILP64",
        "ddot_ilp64": "cblas_ddot$NEWLAPACK$ILP64",
        "zdotu_sub": "cblas_zdotu_sub$NEWLAPACK$ILP64",
    }
    for op in (*_NUMERICS_BLAS_OPS, *_NUMERICS_BLAS_ILP64_OPS):
        if symbols[op] not in blas_exports:
            raise ValueError(
                f"Accelerate's libBLAS does not export the qualified "
                f"CBLAS symbol {symbols[op]!r} — exports check "
                f"against {blas_image}"
            )
    for op in _NUMERICS_LAPACK_OPS:
        if symbols[op] not in lapack_exports:
            raise ValueError(
                "Accelerate's libLAPACK does not export the qualified ILP64 "
                f"symbol {symbols[op]!r} — exports check against "
                f"{lapack_image}"
            )

    imports = _numpy_extension_imports(package, "Darwin")
    for op in _NUMERICS_OPS:
        available = lapack_exports if op in _NUMERICS_LAPACK_OPS else blas_exports
        missing = [name for name in imports.get(op, []) if name not in available]
        if missing:
            raise ValueError(
                f"NumPy imports {missing} for '{op}' which the loaded "
                "Accelerate does not export — the wheel and the system "
                "provider disagree"
            )
        if op not in _NUMERICS_BLAS_OPS and symbols[op] not in imports[op]:
            raise ValueError(
                f"qualified Accelerate symbol {symbols[op]!r} for '{op}' "
                "is not one of NumPy's own imports — refusing an unmeasured binding"
            )

    macos_build = _run_tool(["sw_vers", "-buildVersion"]).strip()
    macos_version = _run_tool(["sw_vers", "-productVersion"]).strip()
    sdk_version = _run_tool(
        ["xcrun", "--sdk", "macosx", "--show-sdk-version"]
    ).strip()
    sdk_build = _run_tool(
        ["xcrun", "--sdk", "macosx", "--show-sdk-build-version"]
    ).strip()
    sdk_path = _run_tool(
        ["xcrun", "--sdk", "macosx", "--show-sdk-path"]
    ).strip()
    uuid = _macho_uuid_at(images[umbrella])
    if not uuid:
        raise ValueError(
            f"could not read LC_UUID of the loaded Accelerate image "
            f"{umbrella} — the apple-system identity has no provider "
            "evidence"
        )
    build_id = _accelerate_build_id(macos_build, uuid, sdk_build)

    threading = _measure_accelerate_threads(np)
    abi_check = _verify_abi_compile_link_run("ACCELERATE", symbols, None)

    record = {
        "backend": "ACCELERATE",
        "numpy_version": str(np.__version__),
        "numpy_package": str(package),
        "site_packages": str(site_packages),
        "blas_integer_bits": 32,
        "lapack_integer_bits": 64,
        "symbols": symbols,
        "numpy_imports": {
            op: list(imports.get(op, [])) for op in _NUMERICS_OPS
        },
        "library_paths": [
            {
                "path": blas_image,
                "role": "blas",
                "identity_kind": "apple-system",
                "sha256": None,
                "build_id": build_id,
            },
            {
                "path": lapack_image,
                "role": "lapack",
                "identity_kind": "apple-system",
                "sha256": None,
                "build_id": build_id,
            },
        ],
        "threading": {
            "implementation": threading["implementation"],
            "num_threads": threading["num_threads"],
        },
        "provider_config": {
            "macos_version": macos_version,
            "macos_build": macos_build,
            "sdk_version": sdk_version,
            "sdk_build": sdk_build,
            "sdk_path": sdk_path,
            "accelerate_uuid": uuid,
            "accelerate_images": sorted(loaded),
            "thread_measure": threading["measured"],
            "wheel": {"tags": _numpy_distribution()[4]},
            "numpy_show_config": _jsonable(np.__config__.CONFIG)
            if isinstance(getattr(np.__config__, "CONFIG", None), dict)
            else {},
            "numpy_show_runtime": _numpy_runtime_info(np),
            "abi_check": abi_check,
        },
    }
    return record


def _scipy_interposition_check(
    np, system: str, baseline: tuple, provider_paths: list[str]
) -> dict:
    """Import SciPy AFTER the NumPy evidence was captured, exercise its
    BLAS, then re-verify the NumPy provider identity and recompute the
    baseline — bitwise. A changed provider or changed result is a hard
    failure, not a record field."""
    import importlib.util

    if importlib.util.find_spec("scipy") is None:
        return {
            "checked": False,
            "reason": "scipy is not installed in the selected interpreter",
        }
    try:
        import scipy.linalg
    except ImportError as exc:
        raise ValueError(
            f"scipy is installed but cannot import scipy.linalg: {exc}"
        ) from exc
    a = np.arange(72, dtype=np.float64).reshape(8, 9) * 0.03125
    scipy.linalg.lstsq(a, np.linspace(1.0, 8.0, 8), cond=None)
    after = _exercise_numpy(np)
    if after != baseline:
        raise ValueError(
            "NumPy results changed after SciPy loaded its own BLAS — "
            "provider interposition is live; refusing to qualify this "
            "environment"
        )
    check: dict = {
        "checked": True,
        "scipy_version": str(getattr(scipy, "__version__", "unknown")),
        "stable": True,
    }
    if system == "Linux":
        import threadpoolctl

        remaining = {
            entry.get("filepath")
            for entry in _blas_pool(threadpoolctl)
        }
        missing = [p for p in provider_paths if p not in remaining]
        if missing:
            raise ValueError(
                f"the NumPy BLAS provider {missing} disappeared from the "
                "loaded pool after SciPy imported — interposition is live"
            )
        check["providers_after_scipy"] = sorted(
            str(p) for p in remaining if p
        )
    else:
        still = [
            name
            for name in _dyld_loaded_images()
            if name.startswith(_ACCELERATE_IMAGE_PREFIX)
        ]
        missing = [p for p in provider_paths if p not in still]
        if missing:
            raise ValueError(
                f"Accelerate images {missing} are no longer loaded after "
                "SciPy imported — provider identity is unstable"
            )
        check["providers_after_scipy"] = sorted(still)
    return check


def discover_numpy_provider() -> dict:
    """Qualify the BLAS/LAPACK provider of *this* interpreter's NumPy.

    Runs only inside the selected-interpreter child probe. Sequence:
    version pin -> numpy-only exercise -> provider identification by
    threadpoolctl/dyld introspection bound to wheel records -> export-table
    symbol qualification -> a compile/link/run C probe in a disposable
    subprocess -> the SciPy interposition control. Every failure mode is a
    named ``ValueError``; the returned record is re-validated by
    ``validate_numerics`` before it is handed back.
    """
    system = platform.system()
    if system not in _MUJOCO_LIBRARY_CANDIDATES:
        raise ValueError(f"unsupported native platform: {system}")
    try:
        import numpy
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import numpy: "
            f"{exc} — " + _NUMPY_INSTALL_INSTRUCTION
        ) from exc
    if str(numpy.__version__) != REQUIRED_NUMPY_VERSION:
        raise ValueError(
            f"native requires the locked NumPy {REQUIRED_NUMPY_VERSION}, "
            f"found {numpy.__version__} in {sys.executable} — "
            + _NUMPY_INSTALL_INSTRUCTION
        )
    try:
        import threadpoolctl
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import "
            f"threadpoolctl: {exc} — run `uv sync --locked --group native`"
        ) from exc

    baseline = _exercise_numpy(numpy)
    package = Path(numpy.__file__).resolve().parent
    if package.name != "numpy":
        raise ValueError(
            f"unexpected NumPy package location {package} — cannot bind "
            "provider records to an unknown tree"
        )
    site_packages = package.parent
    build_dependencies = _numpy_build_dependencies(numpy)

    if system == "Linux":
        record = _discover_numpy_provider_linux(
            numpy, threadpoolctl, package, site_packages, build_dependencies
        )
        provider_paths = [
            entry["path"] for entry in record["library_paths"]
        ]
    else:
        record = _discover_numpy_provider_darwin(
            numpy, threadpoolctl, package, site_packages, build_dependencies
        )
        provider_paths = [
            entry["path"] for entry in record["library_paths"]
        ]
    record["provider_config"]["interposition"] = _scipy_interposition_check(
        numpy, system, baseline, provider_paths
    )
    # The interpolation contraction contract and the broadcast-multiply
    # NaN precedence are wheel-build properties independent of the BLAS
    # provider — measure the installed np.interp/np.multiply on both
    # backends rather than assuming a platform default.
    record["interp"] = _measure_np_interp_lowering(numpy)
    record["multiply_broadcast"] = _measure_np_multiply_broadcast_nan(
        numpy
    )
    return validate_numerics(record)


# --------------------------------------------------------------------------
# The child probe — runs inside the SELECTED interpreter
# --------------------------------------------------------------------------


def _child_probe() -> dict:
    """Collect the dependency document from *this* interpreter.

    Imports mujoco/nanobind here (the selected interpreter), uses sysconfig
    for SOABI/include data, importlib-provided package roots, and nanobind's
    own ``cmake_dir()``/``include_dir()`` queries. Everything raised becomes a
    named stderr diagnostic; only the JSON document reaches stdout.
    """
    system = platform.system()
    libc_name, libc_version = platform.libc_ver()
    libc = f"{libc_name} {libc_version}".strip() or None

    try:
        import mujoco
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import mujoco: {exc}"
        ) from exc
    mujoco_version = str(mujoco.__version__)
    if mujoco_version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found "
            f"{mujoco_version} in {sys.executable}"
        )
    mujoco_package = Path(mujoco.__file__).resolve().parent
    mujoco_include = mujoco_package / "include"
    if not mujoco_include.is_dir():
        raise ValueError(
            f"mujoco {mujoco_version} package has no include directory: "
            f"{mujoco_include}"
        )
    library = select_mujoco_library(mujoco_package, system, mujoco_version)

    try:
        import nanobind
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import nanobind: "
            f"{exc}"
        ) from exc
    nb_cmake_dir = Path(nanobind.cmake_dir())
    nb_include_dir = Path(nanobind.include_dir())
    for label, path in (
        ("nanobind.cmake_dir", nb_cmake_dir),
        ("nanobind.include_dir", nb_include_dir),
    ):
        if not path.is_dir():
            raise ValueError(f"{label} is not an existing directory: {path}")

    soabi = sysconfig.get_config_var("SOABI")
    python_include = sysconfig.get_path("include")
    if not soabi or not python_include:
        raise ValueError(
            f"selected interpreter {sys.executable} reports incomplete "
            f"sysconfig data: soabi={soabi!r} include={python_include!r}"
        )

    # The numerical provider is qualified last: the MuJoCo/nanobind checks
    # keep their failure precedence, and discover_numpy_provider owns the
    # NumPy-only -> SciPy-loaded interposition window itself.
    numerics = discover_numpy_provider()

    return {
        "schema_version": SCHEMA_VERSION,
        "platform": {
            "system": system,
            "machine": platform.machine(),
            "libc": libc,
        },
        "python": {
            "executable": str(Path(sys.executable)),
            "version": platform.python_version(),
            "soabi": soabi,
            "include_dir": str(Path(python_include).resolve()),
        },
        "mujoco": {
            "version": mujoco_version,
            "include_dir": str(mujoco_include),
            "library_path": str(library),
        },
        "nanobind": {
            "cmake_dir": str(nb_cmake_dir),
            "include_dir": str(nb_include_dir),
        },
        "numerics": numerics,
    }


def _require_interpreter(python: Path) -> Path:
    """The selected interpreter must be an absolute, existing executable.

    The path is *not* resolved: a venv interpreter discovers ``pyvenv.cfg``
    and its site-packages through the exact path it was invoked as, so
    executing the realpath'd base binary would silently drop the environment.
    """
    candidate = Path(python).expanduser()
    if not candidate.is_absolute():
        raise ValueError(
            f"selected interpreter must be an absolute path, found {python!r}"
        )
    if not candidate.is_file():
        raise ValueError(f"selected interpreter does not exist: {candidate}")
    if not os.access(candidate, os.X_OK):
        raise ValueError(f"selected interpreter is not executable: {candidate}")
    return candidate


def probe_dependencies(python: Path) -> dict:
    """Run ``--child-probe`` inside the selected interpreter and validate.

    One interpreter contract: the same ``python`` drives the subprocess and is
    the subject; the reported ``python.executable`` must resolve to the same
    interpreter that was invoked. A nonzero child exit or a non-JSON stdout is
    a named failure — the child's stderr is reproduced verbatim.
    """
    interpreter = _require_interpreter(Path(python))
    try:
        completed = subprocess.run(
            [str(interpreter), str(_SELF), "--child-probe"],
            capture_output=True,
            text=True,
            timeout=_PROBE_TIMEOUT,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(
            f"dependency probe could not run in {interpreter}: {exc}"
        ) from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "(no stderr)"
        raise ValueError(
            f"dependency probe failed in {interpreter} "
            f"(exit {completed.returncode}): {detail}"
        )
    try:
        document = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"dependency probe in {interpreter} did not emit schema JSON on "
            f"stdout: {exc}"
        ) from exc
    document = validate_dependencies(document)
    reported = Path(document["python"]["executable"]).resolve()
    if reported != interpreter.resolve():
        raise ValueError(
            f"dependency probe interpreter mismatch: invoked "
            f"{interpreter.resolve()}, child reported {reported}"
        )
    return document


# --------------------------------------------------------------------------
# Committed numerics record (native/numerics.lock.json)
# --------------------------------------------------------------------------
#
# Measuring the numerics needs a Python interpreter with the pinned NumPy,
# which the Python-free root configure never has. The committed record holds
# only the facts that are identical on every machine of a platform with the
# pinned NumPy: the provider's integer ABI and symbol spellings, its identity
# (system framework images on Darwin; wheel-relative files with their SHA-256
# on Linux), and the wheel's np.interp / np.multiply lowering. Everything tied
# to a machine or an OS build (install location, Python identity, macOS and
# SDK build ids, the ABI-check transcript, evidence counts) is a configure-time
# measurement and is never committed. A record measured on one Mac is
# therefore not stale on another Mac with the same NumPy wheel.

NUMERICS_LOCK_SCHEMA_VERSION = 1
NUMERICS_LOCK_PURPOSE = (
    "Committed per-platform numerics facts for the native build, measured "
    "with the pinned NumPy by tools/native_dependencies.py. Each records[] "
    "entry holds only facts identical on every machine of that platform, "
    "plus the NumPy version as the staleness identity. Regenerate with "
    "--write-numerics-record; do not hand-edit."
)
# The facts BikeNumerics.cmake consumes, plus the NumPy version. Nothing else
# the probe measures is committed.
_RECORD_NUMERICS_KEYS = frozenset({
    "backend", "blas_integer_bits", "lapack_integer_bits", "numpy_version",
    "symbols", "library_paths", "interp", "multiply_broadcast",
})
_RECORD_LIBRARY_KEYS = frozenset({"path", "role", "identity_kind", "sha256"})
# The libc family decides which NumPy wheel and provider a platform uses, so
# it is a fact. Its version is not: a glibc point release does not change the
# qualified provider, and committing it would make every libc update stale.
_LIBC_FAMILIES = ("glibc", "musl")


def libc_family(libc: str | None) -> str | None:
    """The record's libc value for a probe ``platform.libc``.

    ``"glibc 2.39"`` projects to ``"glibc"``; ``None`` (Darwin) stays ``None``.
    An unknown family fails closed, because the record cannot say which
    provider that platform would use.
    """
    if libc is None:
        return None
    words = libc.split()
    family = words[0] if words else ""
    if family not in _LIBC_FAMILIES:
        raise ValueError(
            f"unsupported libc {libc!r}; the record names one of "
            f"{_LIBC_FAMILIES} or null"
        )
    return family


def record_platform_key(document: dict) -> str:
    """The records[] key for a document's platform, e.g. ``darwin-arm64``.

    Uses the probe's own ``platform.machine()`` spelling, so ``linux-aarch64``
    and ``darwin-arm64`` are distinct records by construction.
    """
    platform_section = _require_object(document, "platform")
    system = _require_string(platform_section, "system", "platform.system")
    machine = _require_string(platform_section, "machine", "platform.machine")
    return f"{system.lower()}-{machine}"


def _record_library_path(backend: str, raw_path: str, site_packages: str) -> str:
    """The provider path as the committed record stores it.

    Accelerate keeps its absolute system-framework image: that path is the
    same on every Mac. A wheel-shipped provider is stored relative to the
    site-packages root (``numpy.libs/...``), because the install location is
    a configure-time fact and differs between checkouts.
    """
    if backend == "ACCELERATE":
        return raw_path
    provider = Path(raw_path).resolve()
    site = Path(site_packages).resolve()
    if not provider.is_relative_to(site):
        raise ValueError(
            f"numerics provider {provider} is outside the NumPy "
            f"site-packages tree {site}"
        )
    return provider.relative_to(site).as_posix()


def project_numerics_record(document: dict) -> dict:
    """Project a validated schema-1 document onto one record entry.

    ``site_packages`` is read here only to relativize a wheel provider path;
    it is not stored. The result is compared field-for-field in staleness
    checks, so a field that varies between machines must not appear in it.
    """
    numerics = _require_object(document, "numerics")
    backend = _require_string(numerics, "backend", "numerics.backend")
    site_packages = _require_string(
        numerics, "site_packages", "numerics.site_packages")
    symbols = _require_object(numerics, "symbols")
    interp = _require_object(numerics, "interp")
    broadcast = _require_object(numerics, "multiply_broadcast")
    platform_section = _require_object(document, "platform")
    return {
        "platform": {
            "system": platform_section["system"],
            "machine": platform_section["machine"],
            "libc": libc_family(platform_section["libc"]),
        },
        "numerics": {
            "backend": backend,
            "blas_integer_bits": numerics["blas_integer_bits"],
            "lapack_integer_bits": numerics["lapack_integer_bits"],
            "numpy_version": numerics["numpy_version"],
            "symbols": {op: symbols[op] for op in _NUMERICS_OPS},
            "library_paths": [
                {
                    "path": _record_library_path(
                        backend, lib["path"], site_packages),
                    "role": lib["role"],
                    "identity_kind": lib["identity_kind"],
                    "sha256": lib["sha256"],
                }
                for lib in numerics["library_paths"]
            ],
            "interp": {"lowering": interp["lowering"]},
            "multiply_broadcast": {"nan_winner": broadcast["nan_winner"]},
        },
    }


def validate_numerics_record_entry(key: str, entry: dict) -> dict:
    """Validate one records[] entry; return it unchanged.

    Shape and spelling only, with no filesystem access: the committed record
    is read on machines without the measured NumPy. The probe's own
    validate_numerics ran on the live document before projection.
    """
    label = f"numerics record '{key}'"
    if not isinstance(entry, dict):
        raise ValueError(f"{label} must be an object, found {entry!r}")
    if set(entry) != {"platform", "numerics"}:
        raise ValueError(
            f"{label} must carry exactly platform and numerics; "
            f"found {sorted(entry)}"
        )
    platform_section = _require_object(entry, "platform")
    if set(platform_section) != {"system", "machine", "libc"}:
        raise ValueError(
            f"{label} platform must carry exactly system, machine and libc")
    system = _require_string(
        platform_section, "system", f"{label} platform.system")
    machine = _require_string(
        platform_section, "machine", f"{label} platform.machine")
    if f"{system.lower()}-{machine}" != key:
        raise ValueError(
            f"{label} describes {system}/{machine}, which does not match "
            "its records key"
        )
    libc = platform_section["libc"]
    if libc is not None and libc not in _LIBC_FAMILIES:
        raise ValueError(
            f"{label} platform.libc must be the libc family "
            f"{_LIBC_FAMILIES} or null, found {libc!r}"
        )

    numerics = _require_object(entry, "numerics")
    if set(numerics) != _RECORD_NUMERICS_KEYS:
        raise ValueError(
            f"{label} numerics must carry exactly the committed facts; "
            f"missing={sorted(_RECORD_NUMERICS_KEYS - set(numerics))} "
            f"extra={sorted(set(numerics) - _RECORD_NUMERICS_KEYS)}"
        )
    backend = numerics["backend"]
    if backend not in _NUMERICS_BACKENDS:
        raise ValueError(f"{label} has unsupported backend {backend!r}")
    need_blas, need_lapack = _NUMERICS_WIDTHS[backend]
    widths = (numerics["blas_integer_bits"], numerics["lapack_integer_bits"])
    if widths != (need_blas, need_lapack):
        raise ValueError(
            f"{label} is not the qualified {backend} integer ABI "
            f"({need_blas}/{need_lapack} bits), found {widths}"
        )
    _require_string(
        numerics, "numpy_version", f"{label} numerics.numpy_version")
    symbols = _require_object(numerics, "symbols")
    if set(symbols) != set(_NUMERICS_OPS):
        raise ValueError(
            f"{label} numerics.symbols must map exactly {_NUMERICS_OPS}")
    for op in _NUMERICS_OPS:
        _validate_numerics_op_symbol(backend, op, checked_symbol(symbols[op]))
    _validate_record_library_paths(label, backend, numerics["library_paths"])
    interp = _require_object(numerics, "interp")
    if set(interp) != {"lowering"} or \
            interp["lowering"] not in ("contracted", "separate"):
        raise ValueError(
            f"{label} numerics.interp must be exactly "
            "{lowering: contracted|separate}")
    broadcast = _require_object(numerics, "multiply_broadcast")
    if set(broadcast) != {"nan_winner"} or \
            broadcast["nan_winner"] not in ("scalar", "element"):
        raise ValueError(
            f"{label} numerics.multiply_broadcast must be exactly "
            "{nan_winner: scalar|element}")
    return entry


def _validate_record_library_paths(label: str, backend: str,
                                   library_paths) -> None:
    """Provider entries: the path form depends on the backend.

    ACCELERATE entries are the absolute system-framework images with the
    apple-system identity (no digest). NUMPY_OPENBLAS entries are files the
    NumPy wheel ships, stored relative to site-packages with their SHA-256.
    """
    if not isinstance(library_paths, list) or not library_paths:
        raise ValueError(
            f"{label} numerics.library_paths must be a nonempty list")
    roles: set[str] = set()
    for index, lib in enumerate(library_paths):
        where = f"{label} numerics.library_paths[{index}]"
        if not isinstance(lib, dict) or set(lib) != _RECORD_LIBRARY_KEYS:
            raise ValueError(
                f"{where} must carry exactly {sorted(_RECORD_LIBRARY_KEYS)}")
        path = _require_string(lib, "path", f"{where}.path")
        if lib["role"] not in _NUMERICS_ROLES:
            raise ValueError(f"{where}.role {lib['role']!r} is unknown")
        roles.update(lib["role"].split("_"))
        digest = lib["sha256"]
        if backend == "ACCELERATE":
            if lib["identity_kind"] != "apple-system" or digest is not None:
                raise ValueError(
                    f"{where} must be an apple-system entry with a null "
                    "sha256")
            if not path.startswith(_ACCELERATE_IMAGE_PREFIX):
                raise ValueError(
                    f"{where}.path must be an absolute system Accelerate "
                    f"image, found {path!r}")
        else:
            if lib["identity_kind"] != "file" or not (
                    isinstance(digest, str)
                    and _SHA256_HEX.fullmatch(digest) is not None):
                raise ValueError(
                    f"{where} must be a file entry with a lowercase SHA-256")
            if path.startswith("/") or ".." in path.split("/") or not path:
                raise ValueError(
                    f"{where}.path must be relative to site-packages, found "
                    f"{path!r}")
    if not {"blas", "lapack"} <= roles:
        raise ValueError(
            f"{label} numerics.library_paths does not cover blas and lapack")


def validate_numerics_lock(value: dict) -> dict:
    """Validate the whole committed numerics lock; return it unchanged."""
    if not isinstance(value, dict):
        raise ValueError(f"numerics lock must be an object, found {value!r}")
    if value.get("schema_version") != NUMERICS_LOCK_SCHEMA_VERSION:
        found = value.get("schema_version")
        raise ValueError(
            "numerics lock schema_version must be "
            f"{NUMERICS_LOCK_SCHEMA_VERSION}, found {found!r}"
        )
    records = _require_object(value, "records")
    for key, entry in records.items():
        validate_numerics_record_entry(key, entry)
    return value


def load_numerics_lock(path: Path) -> dict:
    """Read and validate the committed numerics lock at ``path``."""
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"numerics lock {path} is unreadable: {exc}") from exc
    return validate_numerics_lock(value)


def _record_differences(committed, live, where: str) -> list[str]:
    """Paths at which the committed entry and the live projection differ."""
    if isinstance(committed, dict) and isinstance(live, dict):
        found: list[str] = []
        for key in sorted(set(committed) | set(live)):
            child = f"{where}.{key}"
            if key not in live:
                found.append(f"{child} is committed but not measured live")
            elif key not in committed:
                found.append(f"{child} is measured live but not committed")
            else:
                found += _record_differences(committed[key], live[key], child)
        return found
    if isinstance(committed, list) and isinstance(live, list):
        if len(committed) != len(live):
            return [f"{where}: committed {len(committed)} entries, "
                    f"live {len(live)}"]
        found = []
        for index, (old, new) in enumerate(zip(committed, live)):
            found += _record_differences(old, new, f"{where}[{index}]")
        return found
    if committed != live:
        return [f"{where}: committed {committed!r}, live {live!r}"]
    return []


def check_numerics_record(path: Path, document: dict) -> str:
    """Require the live measurement to equal the committed entry; return its key.

    ``document`` is a probe document already measured in this checkout. Its
    projection is compared with ``records[<platform key>]`` in full: the
    committed facts and the NumPy version, plus the platform identity that the
    key is derived from. Machine- and OS-build-specific values are not
    compared, because they are not committed (see
    ``_RECORD_NUMERICS_KEYS``). A missing entry or any differing fact is a
    ``ValueError`` that names the regeneration command.
    """
    document = validate_dependencies(document)
    key = record_platform_key(document)
    hint = (
        "regenerate it with: tools/native_dependencies.py --python "
        f"{document['python']['executable']} --write-numerics-record {path}"
    )
    try:
        lock = load_numerics_lock(path)
    except ValueError as exc:
        # A missing or corrupt record is the same remedy as a stale one.
        raise ValueError(f"{exc}; {hint}") from exc
    committed = lock["records"].get(key)
    if committed is None:
        raise ValueError(
            f"numerics record '{key}' has no committed entry in {path}; {hint}"
        )
    differences = _record_differences(
        committed, project_numerics_record(document), "entry")
    if differences:
        shown = "; ".join(differences[:6])
        more = "" if len(differences) <= 6 else (
            f" (and {len(differences) - 6} more)")
        raise ValueError(
            f"numerics record '{key}' in {path} is stale against the live "
            f"measurement: {shown}{more}; {hint}"
        )
    return key


def write_numerics_record(path: Path, document: dict) -> str:
    """Store the document's projection under its platform key; return the key.

    Other platforms' entries already in ``path`` are kept, so each platform
    lane updates only its own record. The file is serialized with sorted
    keys and a trailing newline: the same measurement always produces the
    same bytes.
    """
    key = record_platform_key(document)
    entry = project_numerics_record(document)
    records = {}
    if path.exists():
        records = dict(load_numerics_lock(path)["records"])
    records[key] = entry
    lock = {
        "schema_version": NUMERICS_LOCK_SCHEMA_VERSION,
        "purpose": NUMERICS_LOCK_PURPOSE,
        "records": records,
    }
    validate_numerics_lock(lock)
    text = json.dumps(lock, indent=2, sort_keys=True) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name + ".tmp")
    staging.write_text(text, encoding="utf-8")
    os.replace(staging, path)
    return key


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="native_dependencies.py",
        description=(
            "Probe the selected Python interpreter for the native build's "
            "mujoco/nanobind dependencies; schema-1 JSON only on stdout."
        ),
    )
    parser.add_argument(
        "--python",
        metavar="PATH",
        help="absolute path of the selected interpreter to probe",
    )
    parser.add_argument(
        "--output",
        metavar="PATH",
        help="write the document to this file instead of stdout",
    )
    parser.add_argument(
        "--write-numerics-record",
        metavar="PATH",
        help=(
            "store this platform's numerics projection in the committed "
            "numerics lock (native/numerics.lock.json), keeping other "
            "platforms' records"
        ),
    )
    parser.add_argument(
        "--check-numerics-record",
        metavar="PATH",
        help=(
            "require the --probe-document measurement to match this platform's "
            "entry in the committed numerics lock; prints the platform key"
        ),
    )
    parser.add_argument(
        "--probe-document",
        metavar="PATH",
        help="the measured probe document to check (with --check-numerics-record)",
    )
    parser.add_argument(
        "--child-probe",
        action="store_true",
        help=argparse.SUPPRESS,  # internal: run inside the selected interpreter
    )
    args = parser.parse_args(argv)

    if args.child_probe:
        try:
            document = _child_probe()
        except Exception as exc:  # named diagnostic, never a traceback contract
            print(
                f"dependency probe failed in {sys.executable}: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 1
        sys.stdout.write(json.dumps(document, sort_keys=True) + "\n")
        return 0

    if args.check_numerics_record or args.probe_document:
        if not (args.check_numerics_record and args.probe_document):
            parser.error(
                "--check-numerics-record and --probe-document must be given "
                "together")
        if args.python or args.output or args.write_numerics_record:
            parser.error(
                "--check-numerics-record reads a measured document; it does "
                "not combine with --python, --output or "
                "--write-numerics-record")
        try:
            document = json.loads(
                Path(args.probe_document).read_text(encoding="utf-8"))
            key = check_numerics_record(
                Path(args.check_numerics_record), document)
        except (ValueError, OSError) as exc:
            print(f"numerics record check failed: {exc}", file=sys.stderr)
            return 1
        print(key)
        return 0

    if not args.python:
        parser.error("--python PATH is required")
    if args.output and args.write_numerics_record:
        parser.error(
            "--output and --write-numerics-record are mutually exclusive")
    try:
        document = probe_dependencies(Path(args.python))
        if args.write_numerics_record:
            key = write_numerics_record(
                Path(args.write_numerics_record), document)
    except (ValueError, OSError) as exc:
        print(f"dependency probe failed: {exc}", file=sys.stderr)
        return 1
    if args.write_numerics_record:
        print(
            f"numerics record '{key}' written to {args.write_numerics_record}",
            file=sys.stderr,
        )
        return 0
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
