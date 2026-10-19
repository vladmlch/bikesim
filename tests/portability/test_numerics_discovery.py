"""Numerical-provider qualification contract for tools.native_dependencies.

The ``numerics`` record of the schema-1 dependency document is the only
evidence the native build is allowed to bind BLAS/LAPACK with: a wrong
integer width or a foreign provider is memory corruption, not a rounding
error. These tests exercise the pure metadata contract (``validate_numerics``,
``require_same_library``, ``checked_symbol``) with package-shaped fixtures —
no third-party imports — plus the fail-closed probe/CMake consumption paths.
The real-provider subprocess tests at the bottom only run when the selected
interpreter actually has the pinned scientific environment.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from interpreter_shim import squashed

from tools.native_dependencies import (
    REQUIRED_NUMPY_VERSION,
    check_numerics_record,
    checked_symbol,
    libc_family,
    load_numerics_lock,
    probe_dependencies,
    project_numerics_record,
    record_platform_key,
    require_same_library,
    validate_dependencies,
    validate_numerics,
    validate_numerics_record_entry,
    write_numerics_record,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PROBE_SCRIPT = REPO_ROOT / "tools" / "native_dependencies.py"
NUMERICS_CMAKE = REPO_ROOT / "native" / "cmake" / "BikeNumerics.cmake"
NUMERICS_LOCK = REPO_ROOT / "native" / "numerics.lock.json"

OPENBLAS_SYMBOLS = {
    "ddot": "scipy_cblas_ddot64_",
    "dgemv": "scipy_cblas_dgemv64_",
    "dgemm": "scipy_cblas_dgemm64_",
    "dgelsd": "scipy_dgelsd_64_",
}
ACCELERATE_SYMBOLS = {
    "ddot": "cblas_ddot",
    "dgemv": "cblas_dgemv",
    "dgemm": "cblas_dgemm",
    "dgelsd": "dgelsd$NEWLAPACK$ILP64",
}
ACCELERATE_BUILD_ID = "macos-26A434+accelerate-a697e7b98ff4356ca50764b38af1ccaa+sdk-26A425"
ACCELERATE_BLAS_IMAGE = (
    "/System/Library/Frameworks/Accelerate.framework/Versions/A/Frameworks/"
    "vecLib.framework/Versions/A/libBLAS.dylib"
)
ACCELERATE_LAPACK_IMAGE = (
    "/System/Library/Frameworks/Accelerate.framework/Versions/A/Frameworks/"
    "vecLib.framework/Versions/A/libLAPACK.dylib"
)


# --------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------


def _site_tree(tmp_path: Path) -> tuple[Path, Path]:
    site = tmp_path / "site-packages"
    package = site / "numpy"
    package.mkdir(parents=True, exist_ok=True)
    return site, package


def _openblas_record(tmp_path: Path) -> dict:
    """A complete, valid NUMPY_OPENBLAS ILP64 record over real fixture files."""
    site, package = _site_tree(tmp_path)
    provider = site / "numpy.libs" / "libscipy_openblas64_-fixture0.so"
    provider.parent.mkdir(parents=True, exist_ok=True)
    provider.write_bytes(b"fake ilp64 openblas image")
    digest = hashlib.sha256(provider.read_bytes()).hexdigest()
    return {
        "backend": "NUMPY_OPENBLAS",
        "numpy_version": "2.5.2",
        "numpy_package": str(package),
        "site_packages": str(site),
        "blas_integer_bits": 64,
        "lapack_integer_bits": 64,
        "symbols": dict(OPENBLAS_SYMBOLS),
        "numpy_imports": {op: [name] for op, name in OPENBLAS_SYMBOLS.items()},
        "library_paths": [
            {
                "path": str(provider),
                "role": "blas_lapack",
                "identity_kind": "file",
                "sha256": digest,
                "build_id": None,
            }
        ],
        "threading": {"implementation": "pthreads", "num_threads": 8},
        "interp": {
            "lowering": "separate",
            "queries": 4096,
            "distinguishing": 1470,
        },
        "multiply_broadcast": {
            "nan_winner": "element",
            "lanes": 64,
            "distinguishing": 64,
        },
        "provider_config": {
            "openblas": {
                "version": "0.3.34.0.0",
                "core": "neoversen1",
                "config": "OpenBLAS 0.3.34.0.0  USE64BITINT DYNAMIC_ARCH "
                          "NO_AFFINITY neoversen1 MAX_THREADS=64",
            },
            "interposition": {"checked": True, "stable": True},
        },
    }


def _accelerate_record(tmp_path: Path) -> dict:
    """A complete, valid ACCELERATE record: the Darwin 32/64 integer split."""
    site, package = _site_tree(tmp_path)
    return {
        "backend": "ACCELERATE",
        "numpy_version": "2.5.2",
        "numpy_package": str(package),
        "site_packages": str(site),
        "blas_integer_bits": 32,
        "lapack_integer_bits": 64,
        "symbols": dict(ACCELERATE_SYMBOLS),
        "numpy_imports": {
            "ddot": ["cblas_ddot$NEWLAPACK$ILP64"],
            "dgemv": ["cblas_dgemv$NEWLAPACK$ILP64"],
            "dgemm": ["cblas_dgemm$NEWLAPACK$ILP64"],
            "dgelsd": ["dgelsd$NEWLAPACK$ILP64"],
        },
        "library_paths": [
            {
                "path": ACCELERATE_BLAS_IMAGE,
                "role": "blas",
                "identity_kind": "apple-system",
                "sha256": None,
                "build_id": ACCELERATE_BUILD_ID,
            },
            {
                "path": ACCELERATE_LAPACK_IMAGE,
                "role": "lapack",
                "identity_kind": "apple-system",
                "sha256": None,
                "build_id": ACCELERATE_BUILD_ID,
            },
        ],
        "threading": {"implementation": "accelerate", "num_threads": 4},
        "interp": {
            "lowering": "contracted",
            "queries": 4096,
            "distinguishing": 1511,
        },
        "multiply_broadcast": {
            "nan_winner": "scalar",
            "lanes": 64,
            "distinguishing": 64,
        },
        "provider_config": {
            "macos_build": "26A434",
            "sdk_build": "26A425",
            "accelerate_uuid": "a697e7b98ff4356ca50764b38af1ccaa",
            "interposition": {"checked": True, "stable": True},
        },
    }


# --------------------------------------------------------------------------
# Brief Step 1 verbatim regressions
# --------------------------------------------------------------------------


def test_linux_rejects_unqualified_integer_width():
    with pytest.raises(ValueError, match="integer ABI"):
        validate_numerics({"backend": "NUMPY_OPENBLAS",
                           "blas_integer_bits": None,
                           "lapack_integer_bits": 64})


def test_mixed_numpy_and_native_libraries_are_not_accepted(tmp_path):
    from tools.native_dependencies import require_same_library
    a, b = tmp_path / "numpy-blas.so", tmp_path / "system-blas.so"
    a.write_bytes(b"one")
    b.write_bytes(b"two")
    with pytest.raises(ValueError, match="NumPy provider"):
        require_same_library(a, b)


# --------------------------------------------------------------------------
# checked_symbol — identifier hygiene before any symbol ever reaches a
# generated declaration or a ctypes lookup
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name",
    [
        "cblas_ddot",
        "scipy_cblas_ddot64_",
        "scipy_dgelsd_64_",
        "dgelsd$NEWLAPACK$ILP64",
        "_underscore_ok",
        "X9_$_mixed",
    ],
)
def test_checked_symbol_accepts_valid_identifiers(name):
    assert checked_symbol(name) == name


@pytest.mark.parametrize(
    "name",
    [
        "",                    # empty
        "9starts_with_digit",  # leading digit
        "has space",           # whitespace
        "ddot;rm -rf",         # shell injection attempt
        "ddot; touch /tmp/x",  # command injection attempt
        "line\nbreak",         # newline injection
        'quote"ddot',          # quote injection into generated C
        "back\\slash",         # escapes
        "ddot-name",           # hyphen is not an identifier char
        "ddot(x)",             # call expression, not an identifier
        "$(rm -rf /)",         # shell substitution
    ],
)
def test_checked_symbol_rejects_unsafe_identifiers(name):
    with pytest.raises(ValueError, match="symbol"):
        checked_symbol(name)


def test_checked_symbol_rejects_non_strings():
    for bad in (None, 42, b"cblas_ddot", ["cblas_ddot"]):
        with pytest.raises(ValueError, match="symbol"):
            checked_symbol(bad)  # type: ignore[arg-type]


# --------------------------------------------------------------------------
# require_same_library — resolved-identity, not string equality
# --------------------------------------------------------------------------


def test_require_same_library_accepts_the_identical_file(tmp_path):
    library = tmp_path / "libblas.so"
    library.write_bytes(b"image")
    require_same_library(library, library)


def test_require_same_library_accepts_a_symlink_alias(tmp_path):
    """A symlink to the provider is the same library, not a substitution."""
    real = tmp_path / "real" / "libopenblas.so"
    real.parent.mkdir()
    real.write_bytes(b"real image")
    alias = tmp_path / "libopenblas.so"
    alias.symlink_to(real)
    require_same_library(real, alias)
    require_same_library(alias, real)


def test_require_same_library_rejects_equal_names_different_files(tmp_path):
    one = tmp_path / "a" / "libopenblas.so"
    two = tmp_path / "b" / "libopenblas.so"
    one.parent.mkdir()
    two.parent.mkdir()
    one.write_bytes(b"one")
    two.write_bytes(b"two")
    with pytest.raises(ValueError, match="NumPy provider"):
        require_same_library(one, two)


def test_require_same_library_fails_closed_on_missing_files(tmp_path):
    with pytest.raises(ValueError, match="NumPy provider|resolve|exist"):
        require_same_library(tmp_path / "ghost-a.so", tmp_path / "ghost-b.so")


# --------------------------------------------------------------------------
# validate_numerics — record shape and profile semantics
# --------------------------------------------------------------------------


def test_valid_ilp64_openblas_record_is_accepted(tmp_path):
    record = _openblas_record(tmp_path)
    assert validate_numerics(record) is record


def test_valid_darwin_split_record_is_accepted(tmp_path):
    """Accelerate's qualified profile is the 32-bit legacy CBLAS plus the
    64-bit NEWLAPACK DGELSD — the split must validate."""
    record = _accelerate_record(tmp_path)
    assert validate_numerics(record) is record


@pytest.mark.parametrize(
    "blas_bits,lapack_bits",
    [(None, 64), (64, None), (32, 64), (64, 32), (32, 32), (None, None)],
)
def test_openblas_requires_ilp64_on_both_interfaces(tmp_path, blas_bits, lapack_bits):
    record = _openblas_record(tmp_path)
    record["blas_integer_bits"] = blas_bits
    record["lapack_integer_bits"] = lapack_bits
    with pytest.raises(ValueError, match="integer ABI"):
        validate_numerics(record)


@pytest.mark.parametrize(
    "blas_bits,lapack_bits",
    [(64, 64), (32, 32), (None, 64), (64, 32)],
)
def test_accelerate_requires_the_32_64_split(tmp_path, blas_bits, lapack_bits):
    record = _accelerate_record(tmp_path)
    record["blas_integer_bits"] = blas_bits
    record["lapack_integer_bits"] = lapack_bits
    with pytest.raises(ValueError, match="integer ABI"):
        validate_numerics(record)


def test_integer_widths_must_be_ints_not_bools(tmp_path):
    record = _openblas_record(tmp_path)
    record["blas_integer_bits"] = True
    with pytest.raises(ValueError, match="integer ABI"):
        validate_numerics(record)


def test_unknown_backend_is_rejected(tmp_path):
    record = _openblas_record(tmp_path)
    record["backend"] = "SYSTEM_BLAS"
    with pytest.raises(ValueError, match="backend"):
        validate_numerics(record)


def test_record_must_be_an_object():
    with pytest.raises(ValueError, match="object"):
        validate_numerics("NUMPY_OPENBLAS")


def test_missing_record_fields_are_named(tmp_path):
    for key in ("numpy_version", "library_paths", "symbols", "threading",
                "interp", "multiply_broadcast"):
        record = _openblas_record(tmp_path / key)
        record.pop(key)
        with pytest.raises(ValueError, match=key.split("_")[0]):
            validate_numerics(record)


def test_interp_lowering_spellings(tmp_path):
    """The record carries exactly two qualified lowerings — the probe
    measured the wheel's np.interp contraction and only 'contracted' or
    'separate' is a meaningful verdict."""
    for lowering in ("contracted", "separate"):
        record = _openblas_record(tmp_path / lowering)
        record["interp"]["lowering"] = lowering
        assert validate_numerics(record) is record
    for lowering in ("fused", "fma", "", None, 1):
        record = _openblas_record(tmp_path / str(lowering))
        record["interp"]["lowering"] = lowering
        with pytest.raises(ValueError, match="interp"):
            validate_numerics(record)


@pytest.mark.parametrize(
    "queries,distinguishing",
    [(4096, 0), (4096, 7), (0, 8), (None, 8), (8, None), (8, True),
     (64, 4096)],
)
def test_interp_evidence_counts_are_checked(tmp_path, queries,
                                            distinguishing):
    record = _openblas_record(tmp_path)
    record["interp"]["queries"] = queries
    record["interp"]["distinguishing"] = distinguishing
    with pytest.raises(ValueError, match="interp"):
        validate_numerics(record)


def test_multiply_broadcast_nan_winner_spellings(tmp_path):
    """The broadcast-multiply record carries exactly two qualified
    NaN winners — the probe measured which operand the wheel's kernel
    puts in fmul operand 0 and only 'scalar' or 'element' is a
    meaningful verdict."""
    for winner in ("scalar", "element"):
        record = _openblas_record(tmp_path / winner)
        record["multiply_broadcast"]["nan_winner"] = winner
        assert validate_numerics(record) is record
    for winner in ("first", "operand0", "", None, 1):
        record = _openblas_record(tmp_path / str(winner))
        record["multiply_broadcast"]["nan_winner"] = winner
        with pytest.raises(ValueError, match="multiply_broadcast"):
            validate_numerics(record)


@pytest.mark.parametrize(
    "lanes,distinguishing",
    [(64, 0), (64, 7), (0, 8), (None, 8), (8, None), (8, True),
     (8, 64)],
)
def test_multiply_broadcast_evidence_counts_are_checked(
        tmp_path, lanes, distinguishing):
    record = _openblas_record(tmp_path)
    record["multiply_broadcast"]["lanes"] = lanes
    record["multiply_broadcast"]["distinguishing"] = distinguishing
    with pytest.raises(ValueError, match="multiply_broadcast"):
        validate_numerics(record)


def test_duplicate_providers_are_rejected(tmp_path):
    """Two entries resolving to the same image is a duplicate provider, not
    a coverage list."""
    record = _openblas_record(tmp_path)
    first = record["library_paths"][0]
    alias = dict(first)
    alias["role"] = "lapack"
    alias["path"] = str(Path(first["path"]).parent / "." / Path(first["path"]).name)
    record["library_paths"] = [first, alias]
    with pytest.raises(ValueError, match="[Dd]uplicate"):
        validate_numerics(record)


def test_symlinked_duplicate_providers_are_rejected(tmp_path):
    """Resolution must defeat a second entry pointing at the same image via
    an alias — the path spelling is not the identity."""
    record = _openblas_record(tmp_path)
    first = record["library_paths"][0]
    alias_path = Path(first["path"]).with_name("libother_openblas.so")
    alias_path.symlink_to(Path(first["path"]).resolve())
    record["library_paths"].append(
        {"path": str(alias_path), "role": "lapack", "identity_kind": "file",
         "sha256": first["sha256"], "build_id": None}
    )
    with pytest.raises(ValueError, match="[Dd]uplicate"):
        validate_numerics(record)


def test_missing_required_symbol_is_rejected(tmp_path):
    record = _openblas_record(tmp_path)
    record["symbols"].pop("dgelsd")
    with pytest.raises(ValueError, match="dgelsd"):
        validate_numerics(record)


def test_unexpected_symbol_keys_are_rejected(tmp_path):
    record = _openblas_record(tmp_path)
    record["symbols"]["dgemm_"] = "scipy_dgemm_64_"
    with pytest.raises(ValueError, match="symbol"):
        validate_numerics(record)


def test_symbol_names_must_be_checked_identifiers(tmp_path):
    record = _openblas_record(tmp_path)
    record["symbols"]["ddot"] = "ddot; rm -rf /"
    with pytest.raises(ValueError, match="symbol"):
        validate_numerics(record)


def test_openblas_symbols_must_carry_the_ilp64_suffix(tmp_path):
    """An unsuffixed export name cannot back a 64-bit claim — the naming
    contract is part of the qualified profile."""
    record = _openblas_record(tmp_path)
    record["symbols"]["ddot"] = "scipy_cblas_ddot"  # scipy's LP64 spelling
    with pytest.raises(ValueError, match="integer ABI|ILP64|symbol"):
        validate_numerics(record)


def test_openblas_lapack_symbol_must_be_the_fortran_ilp64_name(tmp_path):
    record = _openblas_record(tmp_path)
    record["symbols"]["dgelsd"] = "scipy_dgelsd_"  # LP64 spelling
    with pytest.raises(ValueError, match="integer ABI|ILP64|symbol"):
        validate_numerics(record)


def test_accelerate_blas_symbols_must_be_the_legacy_interface(tmp_path):
    """Recording the NEWLAPACK names as the qualified BLAS ABI while claiming
    32-bit integers is self-contradictory — reject it."""
    record = _accelerate_record(tmp_path)
    record["symbols"]["ddot"] = "cblas_ddot$NEWLAPACK$ILP64"
    with pytest.raises(ValueError, match="integer ABI|symbol"):
        validate_numerics(record)


def test_accelerate_lapack_symbol_must_be_newlapack_ilp64(tmp_path):
    record = _accelerate_record(tmp_path)
    record["symbols"]["dgelsd"] = "dgelsd_"  # legacy LP64 Fortran symbol
    with pytest.raises(ValueError, match="integer ABI|symbol|ILP64"):
        validate_numerics(record)


# --------------------------------------------------------------------------
# Library-path identity rules
# --------------------------------------------------------------------------


def test_file_identity_requires_the_real_content_hash(tmp_path):
    record = _openblas_record(tmp_path)
    entry = record["library_paths"][0]
    entry["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="sha256|hash"):
        validate_numerics(record)


def test_file_identity_rejects_missing_or_misshapen_digests(tmp_path):
    record = _openblas_record(tmp_path)
    entry = record["library_paths"][0]
    entry["sha256"] = None
    with pytest.raises(ValueError, match="sha256|null"):
        validate_numerics(record)
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["sha256"] = "ABCDEF" * 4  # uppercase + short
    with pytest.raises(ValueError, match="sha256"):
        validate_numerics(record)


def test_file_identity_must_not_carry_a_build_id(tmp_path):
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["build_id"] = "pretend-build"
    with pytest.raises(ValueError, match="build_id"):
        validate_numerics(record)


def test_file_provider_must_exist_and_be_inside_site_packages(tmp_path):
    record = _openblas_record(tmp_path)
    entry = record["library_paths"][0]
    foreign = tmp_path / "elsewhere" / "libsystem_blas.so"
    foreign.parent.mkdir()
    foreign.write_bytes(b"system blas")
    entry["path"] = str(foreign)
    entry["sha256"] = hashlib.sha256(foreign.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match="distribution|site-packages|outside"):
        validate_numerics(record)


def test_file_provider_must_be_a_file(tmp_path):
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["path"] = record["site_packages"]
    with pytest.raises(ValueError, match="file"):
        validate_numerics(record)


def test_apple_system_identity_never_hashes_a_file(tmp_path):
    record = _accelerate_record(tmp_path)
    record["library_paths"][0]["sha256"] = "a" * 64
    with pytest.raises(ValueError, match="sha256|null|apple-system"):
        validate_numerics(record)


def test_apple_system_identity_requires_a_real_build_id(tmp_path):
    record = _accelerate_record(tmp_path)
    record["library_paths"][0]["build_id"] = None
    with pytest.raises(ValueError, match="build_id"):
        validate_numerics(record)
    record = _accelerate_record(tmp_path)
    record["library_paths"][0]["build_id"] = ""
    with pytest.raises(ValueError, match="build_id"):
        validate_numerics(record)


def test_apple_system_paths_must_be_system_framework_images(tmp_path):
    record = _accelerate_record(tmp_path)
    record["library_paths"][0]["path"] = "/opt/local/lib/libblas.dylib"
    with pytest.raises(ValueError, match="Accelerate|framework"):
        validate_numerics(record)


def test_openblas_record_rejects_apple_system_entries(tmp_path):
    """A dyld-cache identity under the Linux wheel backend is a substitution."""
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["identity_kind"] = "apple-system"
    record["library_paths"][0]["sha256"] = None
    record["library_paths"][0]["build_id"] = "someone"
    with pytest.raises(ValueError, match="identity|apple-system|file"):
        validate_numerics(record)


def test_accelerate_record_rejects_file_entries(tmp_path):
    record = _accelerate_record(tmp_path)
    entry = record["library_paths"][0]
    entry["identity_kind"] = "file"
    entry["sha256"] = "f" * 64
    entry["build_id"] = None
    with pytest.raises(ValueError, match="identity|apple-system|file"):
        validate_numerics(record)


def test_library_roles_must_cover_blas_and_lapack(tmp_path):
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["role"] = "blas"  # lapack uncovered
    with pytest.raises(ValueError, match="lapack|role"):
        validate_numerics(record)


def test_unknown_role_or_identity_kind_is_rejected(tmp_path):
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["role"] = "math"
    with pytest.raises(ValueError, match="role"):
        validate_numerics(record)
    record = _openblas_record(tmp_path)
    record["library_paths"][0]["identity_kind"] = "dyld-cache"
    with pytest.raises(ValueError, match="identity"):
        validate_numerics(record)


# --------------------------------------------------------------------------
# threading / provider_config / package-record coherence
# --------------------------------------------------------------------------


@pytest.mark.parametrize("threads", [0, -1, "16", 16.5, None, True])
def test_thread_count_must_be_a_positive_int(tmp_path, threads):
    record = _openblas_record(tmp_path)
    record["threading"]["num_threads"] = threads
    with pytest.raises(ValueError, match="num_threads|thread"):
        validate_numerics(record)


def test_threading_implementation_must_be_named(tmp_path):
    record = _openblas_record(tmp_path)
    record["threading"]["implementation"] = ""
    with pytest.raises(ValueError, match="implementation|thread"):
        validate_numerics(record)


def test_numpy_package_must_live_in_site_packages(tmp_path):
    record = _openblas_record(tmp_path)
    other = tmp_path / "other-tree" / "numpy"
    other.mkdir(parents=True)
    record["numpy_package"] = str(other)
    with pytest.raises(ValueError, match="site_packages|distribution"):
        validate_numerics(record)


def test_openblas_config_must_prove_the_ilp64_build(tmp_path):
    """The USE64BITINT build flag is the provider-side evidence for the
    integer-width claim — a record without it is a guess."""
    record = _openblas_record(tmp_path)
    record["provider_config"]["openblas"]["config"] = "OpenBLAS 0.3.34 NO_AFFINITY"
    with pytest.raises(ValueError, match="USE64BITINT|integer ABI"):
        validate_numerics(record)


# --------------------------------------------------------------------------
# Integration: validate_dependencies must not wave a bad numerics record
# through — the failure has to surface at the document boundary.
# --------------------------------------------------------------------------


def _fixture_document(tmp_path: Path) -> dict:
    site, _ = _site_tree(tmp_path)
    mujoco_inc = site / "mujoco" / "include" / "mujoco"
    mujoco_inc.mkdir(parents=True, exist_ok=True)
    (mujoco_inc / "mujoco.h").write_text("/* fixture */\n")
    library = site / "mujoco" / "libmujoco.so.3.12.0"
    library.write_bytes(b"fixture image")
    python_include = tmp_path / "python-include"
    python_include.mkdir(exist_ok=True)
    nb_cmake = tmp_path / "nanobind" / "cmake"
    nb_include = tmp_path / "nanobind" / "include"
    nb_cmake.mkdir(parents=True, exist_ok=True)
    nb_include.mkdir(parents=True, exist_ok=True)
    return {
        "schema_version": 1,
        "platform": {"system": "Linux", "machine": "aarch64", "libc": "glibc 2.39"},
        "python": {
            "executable": sys.executable,
            "version": "3.13.16",
            "soabi": "cpython-313-aarch64-linux-gnu",
            "include_dir": str(python_include),
        },
        "mujoco": {
            "version": "3.12.0",
            "include_dir": str(mujoco_inc.parent),
            "library_path": str(library),
        },
        "nanobind": {
            "cmake_dir": str(nb_cmake),
            "include_dir": str(nb_include),
        },
        "numerics": None,
    }


def test_validate_dependencies_accepts_null_or_valid_numerics(tmp_path):
    document = _fixture_document(tmp_path)
    assert validate_dependencies(document) is document
    document["numerics"] = _openblas_record(tmp_path)
    assert validate_dependencies(document) is document


def test_validate_dependencies_rejects_an_invalid_numerics_record(tmp_path):
    document = _fixture_document(tmp_path)
    document["numerics"] = _openblas_record(tmp_path)
    document["numerics"]["blas_integer_bits"] = None
    with pytest.raises(ValueError, match="integer ABI"):
        validate_dependencies(document)


# --------------------------------------------------------------------------
# Generated ABI-check source — the compile contract is data-driven, the
# symbols land exclusively inside checked asm labels.
# --------------------------------------------------------------------------


def test_abi_probe_source_uses_the_qualified_widths_and_symbols():
    from tools.native_dependencies import _numerics_abi_probe_source
    source = _numerics_abi_probe_source("NUMPY_OPENBLAS", dict(OPENBLAS_SYMBOLS))
    assert "int64_t" in source
    assert '__asm__("scipy_cblas_ddot64_")' in source
    assert '__asm__("scipy_cblas_dgemv64_")' in source
    assert '__asm__("scipy_cblas_dgemm64_")' in source
    assert '__asm__("scipy_dgelsd_64_")' in source
    assert "sizeof(blas_int) == 8" in source


def test_abi_probe_source_darwin_uses_32_bit_blas_and_asm_labels():
    from tools.native_dependencies import _numerics_abi_probe_source
    source = _numerics_abi_probe_source("ACCELERATE", dict(ACCELERATE_SYMBOLS))
    assert "sizeof(blas_int) == 4" in source
    assert "sizeof(lapack_int) == 8" in source
    assert '__asm__("_cblas_ddot")' in source
    assert '__asm__("_dgelsd$NEWLAPACK$ILP64")' in source


def test_abi_probe_source_rejects_unknown_backend_and_unsafe_names():
    from tools.native_dependencies import _numerics_abi_probe_source
    with pytest.raises(ValueError, match="backend|symbol"):
        _numerics_abi_probe_source("SYSTEM_BLAS", dict(OPENBLAS_SYMBOLS))
    bad = dict(OPENBLAS_SYMBOLS)
    bad["ddot"] = 'ddot"); system("rm -rf'
    with pytest.raises(ValueError, match="symbol"):
        _numerics_abi_probe_source("NUMPY_OPENBLAS", bad)


# --------------------------------------------------------------------------
# Real probes — the actual provider on this host, only when the pinned
# scientific environment is present.
# --------------------------------------------------------------------------


def _real_environment_available() -> bool:
    return all(
        importlib.util.find_spec(name) is not None
        for name in ("numpy", "threadpoolctl", "mujoco", "nanobind")
    )


def _numerics_identity(numerics: dict) -> dict:
    """The stable provider identity: backend, widths, symbol map and the
    per-library identity entries — independent of measured thread counts
    and absolute tooling paths."""
    return {
        "backend": numerics["backend"],
        "numpy_version": numerics["numpy_version"],
        "blas_integer_bits": numerics["blas_integer_bits"],
        "lapack_integer_bits": numerics["lapack_integer_bits"],
        "symbols": numerics["symbols"],
        "library_identities": [
            {
                "role": entry["role"],
                "identity_kind": entry["identity_kind"],
                "sha256": entry["sha256"],
                "build_id": entry["build_id"],
            }
            for entry in numerics["library_paths"]
        ],
    }


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="real provider probe needs numpy/threadpoolctl/mujoco/nanobind",
)
def test_probe_dependencies_reports_the_qualified_provider():
    document = probe_dependencies(Path(sys.executable))
    numerics = document["numerics"]
    expected_backend = {
        "Darwin": "ACCELERATE", "Linux": "NUMPY_OPENBLAS"
    }[document["platform"]["system"]]
    assert numerics["backend"] == expected_backend
    assert numerics["numpy_version"] == REQUIRED_NUMPY_VERSION
    if expected_backend == "ACCELERATE":
        assert numerics["blas_integer_bits"] == 32
        assert numerics["lapack_integer_bits"] == 64
    else:
        assert numerics["blas_integer_bits"] == 64
        assert numerics["lapack_integer_bits"] == 64
    assert set(numerics["symbols"]) == {"ddot", "dgemv", "dgemm", "dgelsd"}
    assert all(numerics["symbols"].values())
    assert numerics["threading"]["num_threads"] >= 1
    assert numerics["library_paths"]
    for entry in numerics["library_paths"]:
        if entry["identity_kind"] == "file":
            provider = Path(entry["path"])
            assert provider.is_file()
            assert hashlib.sha256(provider.read_bytes()).hexdigest() == \
                entry["sha256"]
            assert entry["build_id"] is None
        else:
            assert entry["sha256"] is None
            assert entry["build_id"]


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="real provider probe needs the pinned scientific environment",
)
def test_repeated_probes_have_stable_provider_identity():
    first = _numerics_identity(probe_dependencies(Path(sys.executable))["numerics"])
    second = _numerics_identity(probe_dependencies(Path(sys.executable))["numerics"])
    assert first == second


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="real provider probe needs the pinned scientific environment",
)
def test_cli_stdout_carries_the_numerics_record():
    result = subprocess.run(
        [sys.executable, str(PROBE_SCRIPT), "--python", sys.executable],
        capture_output=True, text=True, timeout=600,
    )
    assert result.returncode == 0, result.stderr
    document = json.loads(result.stdout)
    numerics = document["numerics"]
    assert numerics["backend"] in {"ACCELERATE", "NUMPY_OPENBLAS"}
    assert numerics["blas_integer_bits"] in (32, 64)
    assert numerics["lapack_integer_bits"] == 64


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="scipy interposition control needs the pinned environment",
)
@pytest.mark.parametrize("scipy_first", [False, True])
def test_numpy_provider_identity_is_stable_across_scipy_import_order(scipy_first):
    """scipy bundles a *different* (LP64) OpenBLAS; whichever order the
    imports happen in, NumPy's provider identity must not move."""
    if importlib.util.find_spec("scipy") is None:
        pytest.skip("scipy is not installed")
    snippet = (
        "import sys, json\n"
        + ("import scipy.linalg\n" if scipy_first else "")
        + "sys.path.insert(0, {root!r})\n"
        "from tools.native_dependencies import discover_numpy_provider\n"
        "record = discover_numpy_provider()\n"
        "print(json.dumps(record['library_paths'][0]['path']))\n"
        "print(json.dumps(record['symbols']))\n"
    ).format(root=str(REPO_ROOT))
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    discovered = subprocess.run(
        [sys.executable, "-c", snippet],
        capture_output=True, text=True, timeout=600, env=env,
    )
    assert discovered.returncode == 0, discovered.stderr
    fresh = subprocess.run(
        [sys.executable, str(PROBE_SCRIPT), "--python", sys.executable],
        capture_output=True, text=True, timeout=600, env=env,
    )
    assert fresh.returncode == 0, fresh.stderr
    reference = json.loads(fresh.stdout)["numerics"]
    assert json.loads(discovered.stdout.splitlines()[0]) == \
        reference["library_paths"][0]["path"]
    assert json.loads(discovered.stdout.splitlines()[1]) == \
        reference["symbols"]


# --------------------------------------------------------------------------
# BikeNumerics.cmake consumption — the module is exercised through a scratch
# project because it must stay includable before T10 wires it into the real
# CMakeLists.
# --------------------------------------------------------------------------


def _write_record(tmp_path: Path, document: dict) -> Path:
    """Commit the document's projection to a fixture record (same format as
    native/numerics.lock.json). Call it before any tampering of ``document``:
    the writer validates what it projects."""
    record = tmp_path / "numerics.lock.json"
    write_numerics_record(record, document)
    return record


def _configure_scratch(
    tmp_path: Path, document: dict, backend: str = "AUTO",
    record: Path | None = None, python: str | None = None,
) -> subprocess.CompletedProcess:
    """Configure BikeNumerics.cmake against ``document`` and ``record``.

    The check runs ``python`` (Python_EXECUTABLE; default this test's
    interpreter), so the scratch project names it; the record defaults to one
    projected from ``document`` so the live measurement and the record agree.
    """
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    scratch = tmp_path / "scratch"
    scratch.mkdir()
    if record is None:
        record = _write_record(tmp_path, document)
    interpreter = python if python is not None else sys.executable
    doc = dict(document)
    doc["numerics"] = doc.get("numerics")
    json_path = tmp_path / "dependencies.json"
    json_path.write_text(json.dumps(doc))
    (scratch / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.27)\n"
        "project(numerics_check NONE)\n"
        f'set(BIKE_TARGET_SYSTEM "{document["platform"]["system"]}")\n'
        f'set(Python_EXECUTABLE "{interpreter}")\n'
        'set(BIKE_NUMERICS_SOURCE LIVE)\n'
        f'file(READ "{json_path}" _native_dependencies_json)\n'
        f'include("{NUMERICS_CMAKE}")\n'
        'message(STATUS "RESOLVED=${BIKE_NUMERICS_BACKEND_RESOLVED}")\n'
        'message(STATUS "DDOT=${_native_numerics_symbol_ddot}")\n'
        'message(STATUS "DGELSD=${_native_numerics_symbol_dgelsd}")\n'
    )
    return subprocess.run(
        [cmake, "-S", str(scratch), "-B", str(tmp_path / "scratch-build"),
         f"-DBIKE_NUMERICS_BACKEND={backend}",
         f"-DBIKE_NUMERICS_RECORD={record}"],
        capture_output=True, text=True, timeout=300,
    )


def _document_with_numerics(tmp_path: Path, system: str) -> dict:
    document = _fixture_document(tmp_path)
    document["platform"]["system"] = system
    if system == "Darwin":
        document["mujoco"]["library_path"] = str(
            document["mujoco"]["library_path"]
        ).replace("libmujoco.so.3.12.0", "libmujoco.3.12.0.dylib")
        dylib = Path(document["mujoco"]["library_path"])
        dylib.write_bytes(b"fixture image")
        document["numerics"] = _accelerate_record(tmp_path)
    else:
        document["numerics"] = _openblas_record(tmp_path)
    return document


def test_bike_numerics_resolves_auto_backend_from_the_probe(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    result = _configure_scratch(tmp_path, document)
    assert result.returncode == 0, result.stderr
    expected = "ACCELERATE" if system == "Darwin" else "NUMPY_OPENBLAS"
    assert f"RESOLVED={expected}" in result.stdout


def test_bike_numerics_resolves_a_wheel_relative_provider_on_linux(tmp_path):
    """The record stores the Linux provider relative to site-packages and
    BikeNumerics resolves it against the live install. The scratch project is
    record-only (no CXX), so the Linux branch runs on a macOS host too."""
    document = _document_with_numerics(tmp_path, "Linux")
    record = _write_record(tmp_path, document)
    stored = json.loads(record.read_text(encoding="utf-8"))
    provider = next(iter(stored["records"].values()))["numerics"][
        "library_paths"][0]["path"]
    assert provider == "numpy.libs/libscipy_openblas64_-fixture0.so"
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode == 0, result.stderr
    assert "RESOLVED=NUMPY_OPENBLAS" in result.stdout


def test_bike_numerics_rejects_a_backend_mismatch(tmp_path):
    """Requesting NUMPY_OPENBLAS on a Darwin host (or vice versa) is a
    provider mismatch — it must fail before any physics target exists."""
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    wrong = "NUMPY_OPENBLAS" if system == "Darwin" else "ACCELERATE"
    result = _configure_scratch(tmp_path, document, backend=wrong)
    assert result.returncode != 0
    assert "numerics" in result.stderr.lower() or "backend" in result.stderr.lower()


def _stub_checker(tmp_path: Path, key: str) -> str:
    """An interpreter stand-in whose verdict is ``key`` (exit 0, no reading of
    its arguments). It lets a test reach the CMake guards with data the real
    checker would reject first: the guards read the committed record, so a
    tampered record is what they must catch."""
    stub = tmp_path / "stub-checker"
    stub.write_text(f"#!/bin/sh\necho {key}\n", encoding="utf-8")
    stub.chmod(0o755)
    return str(stub)


def test_bike_numerics_rejects_an_unqualified_live_width(tmp_path):
    """The live probe's width is checked by the Python checker first, which
    names the unqualified integer ABI before any CMake guard runs."""
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    record = _write_record(tmp_path, document)
    document["numerics"]["blas_integer_bits"] = None
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode != 0
    assert "unqualified BLAS/LAPACK integer ABI" in squashed(result.stderr)


def test_bike_numerics_rejects_a_tampered_live_symbol(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    record = _write_record(tmp_path, document)
    document["numerics"]["symbols"]["ddot"] = 'ddot";execute_process(COMMAND rm -rf'
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode != 0
    assert "invalid exported numerical symbol identifier" in squashed(result.stderr)


def test_cmake_guard_rejects_an_unqualified_width_in_the_record(tmp_path):
    """The check passes (stub verdict) but the committed record carries a
    32-bit OpenBLAS width: the CMake width guard is what must reject it."""
    document = _document_with_numerics(tmp_path, "Linux")
    record = _write_record(tmp_path, document)
    lock = json.loads(record.read_text(encoding="utf-8"))
    entry = next(iter(lock["records"].values()))
    entry["numerics"]["blas_integer_bits"] = 32
    record.write_text(json.dumps(lock), encoding="utf-8")
    stub = _stub_checker(tmp_path, "linux-aarch64")
    result = _configure_scratch(tmp_path, document, record=record, python=stub)
    assert result.returncode != 0, result.stdout
    assert ("unqualified numerics integer ABI: blas_integer_bits='32' does not "
            "match the qualified 64-bit profile for NUMPY_OPENBLAS"
            in squashed(result.stderr))


def test_cmake_guard_rejects_a_tampered_symbol_in_the_record(tmp_path):
    document = _document_with_numerics(tmp_path, "Linux")
    record = _write_record(tmp_path, document)
    lock = json.loads(record.read_text(encoding="utf-8"))
    entry = next(iter(lock["records"].values()))
    entry["numerics"]["symbols"]["ddot"] = 'ddot";execute_process(COMMAND rm -rf'
    record.write_text(json.dumps(lock), encoding="utf-8")
    stub = _stub_checker(tmp_path, "linux-aarch64")
    result = _configure_scratch(tmp_path, document, record=record, python=stub)
    assert result.returncode != 0, result.stdout
    assert ("numerics symbol for 'ddot' is not a safe identifier"
            in squashed(result.stderr))


def test_bike_numerics_rejects_a_missing_numerics_record(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    record = _write_record(tmp_path, document)
    document["numerics"] = None
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode != 0
    assert "has no qualified numerics record" in squashed(result.stderr)


def test_bike_numerics_reports_a_non_spawnable_interpreter(tmp_path):
    """A command that cannot start leaves stderr empty; the failure must still
    name the interpreter, the command and the result, not an empty message."""
    document = _document_with_numerics(tmp_path, platform.system())
    missing = str(tmp_path / "no-such-python")
    result = _configure_scratch(tmp_path, document, python=missing)
    assert result.returncode != 0
    stderr = squashed(result.stderr)
    assert "numerics record check did not pass" in stderr
    assert f"{missing} " in stderr and "--check-numerics-record" in stderr
    assert "result '" in stderr


def test_bike_numerics_fails_configure_on_a_tampered_committed_fact(tmp_path):
    """One fact flipped in the committed record: configure must fail, name
    the stale entry and the Task 16 regeneration command."""
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    record = _write_record(tmp_path, document)
    lock = json.loads(record.read_text(encoding="utf-8"))
    entry = next(iter(lock["records"].values()))
    entry["numerics"]["interp"]["lowering"] = (
        "separate" if entry["numerics"]["interp"]["lowering"] == "contracted"
        else "contracted")
    record.write_text(json.dumps(lock), encoding="utf-8")
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode != 0, result.stdout
    stderr = squashed(result.stderr)
    assert "is stale against the live measurement" in stderr
    assert "interp.lowering" in stderr
    assert "--write-numerics-record" in stderr


def test_bike_numerics_fails_configure_without_a_platform_entry(tmp_path):
    """A record with no entry for this platform fails configure with the
    same regeneration hint."""
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    record = _write_record(tmp_path, document)
    lock = json.loads(record.read_text(encoding="utf-8"))
    lock["records"] = {}
    record.write_text(json.dumps(lock), encoding="utf-8")
    result = _configure_scratch(tmp_path, document, record=record)
    assert result.returncode != 0, result.stdout
    stderr = squashed(result.stderr)
    assert "has no committed entry" in stderr
    assert "--write-numerics-record" in stderr


# --------------------------------------------------------------------------
# RECORD_ONLY (the Python-free root graph): facts from the record alone, with
# no probe, no interpreter and no live value.
# --------------------------------------------------------------------------


def _record_only_document(tmp_path: Path) -> dict:
    """A Darwin document whose machine is this host's, so the record key that
    RECORD_ONLY derives from CMAKE_HOST_SYSTEM_PROCESSOR names its entry."""
    document = _document_with_numerics(tmp_path, "Darwin")
    document["platform"]["machine"] = platform.machine()
    # The shared fixture is Linux-shaped; the Darwin probe reports no libc.
    document["platform"]["libc"] = None
    return document


def _configure_record_only(
    tmp_path: Path, document: dict, record: Path | None = None,
    select_mode: bool = True,
) -> subprocess.CompletedProcess:
    """Configure BikeNumerics.cmake the way the root graph does: no
    Python_EXECUTABLE and no dependency document. A logging python shim leads
    PATH, so any interpreter spawn is recorded in tmp_path/python-calls.log and
    exits 97, which fails configure if CMake ever runs it by name."""
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    scratch = tmp_path / "record-only"
    scratch.mkdir()
    if record is None:
        record = _write_record(tmp_path, document)
    shims = tmp_path / "shims"
    shims.mkdir()
    shim_log = tmp_path / "python-calls.log"
    for name in ("python", "python3"):
        stub = shims / name
        stub.write_text(
            f'#!/bin/sh\necho "{name} $*" >> "{shim_log}"\nexit 97\n',
            encoding="utf-8")
        stub.chmod(0o755)
    mode = "set(BIKE_NUMERICS_SOURCE RECORD_ONLY)\n" if select_mode else ""
    (scratch / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.27)\n"
        "project(numerics_record_only NONE)\n"
        f'set(BIKE_TARGET_SYSTEM "{document["platform"]["system"]}")\n'
        f"{mode}"
        f'include("{NUMERICS_CMAKE}")\n'
        'message(STATUS "RESOLVED=${BIKE_NUMERICS_BACKEND_RESOLVED}")\n'
        "get_directory_property(_defs COMPILE_DEFINITIONS)\n"
        'message(STATUS "DEFS=${_defs}")\n'
    )
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(shims), env.get("PATH", "")])
    return subprocess.run(
        [cmake, "-S", str(scratch), "-B", str(tmp_path / "record-only-build"),
         f"-DBIKE_NUMERICS_RECORD={record}"],
        capture_output=True, text=True, timeout=300, env=env,
    )


def _edit_record(record: Path, edit) -> None:
    """Apply ``edit`` to the single records[] entry of a fixture record."""
    lock = json.loads(record.read_text(encoding="utf-8"))
    edit(next(iter(lock["records"].values())))
    record.write_text(json.dumps(lock), encoding="utf-8")


def test_record_only_binds_the_committed_facts_without_python(tmp_path):
    """The root configure reads the facts from the record, projects the two
    BIKE_NUMPY_* definitions from them, and never starts an interpreter."""
    document = _record_only_document(tmp_path)
    record = _write_record(tmp_path, document)
    result = _configure_record_only(tmp_path, document, record=record)
    assert result.returncode == 0, result.stderr
    assert "RESOLVED=ACCELERATE" in result.stdout
    assert "bike_native_numerics not created: CXX is not enabled" in result.stdout
    entry = next(iter(json.loads(record.read_text(encoding="utf-8"))["records"].values()))
    interp = 1 if entry["numerics"]["interp"]["lowering"] == "contracted" else 0
    nan_element = 1 if entry["numerics"]["multiply_broadcast"]["nan_winner"] == "element" else 0
    assert f"BIKE_NUMPY_INTERP_CONTRACTED={interp}" in result.stdout
    assert f"BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN={nan_element}" in result.stdout
    assert not (tmp_path / "python-calls.log").exists(), (
        "record-only configure started a python interpreter")


def test_record_only_requires_an_explicit_mode(tmp_path):
    """No mode is inferred: a root that forgets BIKE_NUMERICS_SOURCE fails
    instead of falling back to a live verdict or to record-only."""
    document = _record_only_document(tmp_path)
    result = _configure_record_only(tmp_path, document, select_mode=False)
    assert result.returncode != 0
    assert "requires BIKE_NUMERICS_SOURCE=LIVE" in squashed(result.stderr)


def test_record_only_fails_closed_outside_darwin(tmp_path):
    """A Linux key needs the libc family and the NumPy install location, which
    configure cannot read without the probe, so record-only refuses it."""
    document = _document_with_numerics(tmp_path, "Linux")
    result = _configure_record_only(tmp_path, document)
    assert result.returncode != 0
    assert "qualified for Darwin only" in squashed(result.stderr)


def test_record_only_names_a_missing_platform_entry(tmp_path):
    document = _record_only_document(tmp_path)
    record = _write_record(tmp_path, document)
    lock = json.loads(record.read_text(encoding="utf-8"))
    lock["records"] = {}
    record.write_text(json.dumps(lock), encoding="utf-8")
    result = _configure_record_only(tmp_path, document, record=record)
    assert result.returncode != 0
    stderr = squashed(result.stderr)
    assert "committed numerics record has no entry" in stderr
    assert "--write-numerics-record" in stderr


def test_record_only_rejects_an_entry_that_describes_another_host(tmp_path):
    """The key is derived from the host, so an entry whose own platform fields
    name another machine can only come from a hand-edited record."""
    document = _record_only_document(tmp_path)
    record = _write_record(tmp_path, document)
    _edit_record(
        record,
        lambda entry: entry["platform"].update(
            machine=f"not-{platform.machine()}"))
    result = _configure_record_only(tmp_path, document, record=record)
    assert result.returncode != 0
    assert "does not describe this host" in squashed(result.stderr)


def test_record_only_keeps_the_structural_guards(tmp_path):
    """Record-only trusts no value without checks: the same CMake guards as
    LIVE mode still reject an unqualified integer width."""
    document = _record_only_document(tmp_path)
    record = _write_record(tmp_path, document)

    def widen_blas(entry):
        entry["numerics"]["blas_integer_bits"] = 64

    _edit_record(record, widen_blas)
    result = _configure_record_only(tmp_path, document, record=record)
    assert result.returncode != 0
    assert "unqualified numerics integer ABI" in squashed(result.stderr)


def test_record_only_does_not_compare_with_a_measurement(tmp_path):
    """Record-only takes the committed facts as they are. A flipped
    interp.lowering configures here because nothing is measured in this mode;
    the legacy graph's live check is the guard that rejects it."""
    document = _record_only_document(tmp_path)
    record = _write_record(tmp_path, document)

    def flip_lowering(entry):
        entry["numerics"]["interp"]["lowering"] = "separate"

    _edit_record(record, flip_lowering)
    result = _configure_record_only(tmp_path, document, record=record)
    assert result.returncode == 0, result.stderr
    assert "BIKE_NUMPY_INTERP_CONTRACTED=0" in result.stdout


# --------------------------------------------------------------------------
# Committed numerics record (native/numerics.lock.json): projection, writer
# and reader. The fixtures mirror the probe's full output, host-only noise
# included, so the projection is exercised against what a real run emits.
# --------------------------------------------------------------------------

HOST_SENTINEL = "HOST-SENTINEL-NOT-A-FACT"


def _measured_darwin_document(tmp_path: Path) -> dict:
    """A probe-shaped Darwin document carrying every host and OS-build field
    the live probe emits, so the projection is exercised against them."""
    document = _fixture_document(tmp_path)
    document["platform"] = {"system": "Darwin", "machine": "arm64", "libc": None}
    document["python"]["version"] = "3.14.3"
    document["python"]["soabi"] = "cpython-314-darwin"
    numerics = _accelerate_record(tmp_path)
    numerics["provider_config"] = {
        "macos_version": "27.0.1",
        "macos_build": "26A434",
        "sdk_version": "27.0",
        "sdk_build": "26A425",
        "sdk_path": "/Applications/Xcode.app/Contents/Developer/SDKs/MacOSX27.0.sdk",
        "accelerate_uuid": "a697e7b98ff4356ca50764b38af1ccaa",
        "accelerate_images": [ACCELERATE_BLAS_IMAGE, ACCELERATE_LAPACK_IMAGE],
        "thread_measure": {"baseline_threads": 2, "peak_threads": 6},
        "wheel": {"tags": ["cp314-cp314-macosx_14_0_arm64"]},
        "numpy_show_config": {"Build Dependencies": {}},
        "numpy_show_runtime": f"uname(node='{HOST_SENTINEL}')",
        "abi_check": {
            "compile_only": "pass",
            "compiler": "Apple clang version 21.0.0",
            "link_run": "pass",
            "output": "BIKE-ABI-OK ddot=32",
        },
        "interposition": {"checked": True, "stable": True, "scipy_version": "1.17.0"},
    }
    numerics["threading"] = {"implementation": "accelerate", "num_threads": 5}
    numerics["interp"]["queries"] = 4096
    numerics["numpy_imports"] = {"ddot": ["cblas_ddot$NEWLAPACK$ILP64"]}
    document["numerics"] = numerics
    return document


def _measured_linux_document(tmp_path: Path) -> dict:
    document = _fixture_document(tmp_path)
    document["numerics"] = _openblas_record(tmp_path)
    document["numerics"]["provider_config"] = {
        "openblas": {
            "version": "0.3.30",
            "core": "Neoverse-V2",
            "config": "USE64BITINT DYNAMIC_ARCH",
            "record_path": "numpy-2.5.2.dist-info/RECORD",
            "num_threads_reported": 8,
        },
        "wheel": {"tags": ["cp313-cp313-manylinux_2_27_aarch64"]},
        "foreign_providers_loaded": [HOST_SENTINEL],
        "numpy_show_config": {},
        "numpy_show_runtime": f"uname(node='{HOST_SENTINEL}')",
        "abi_check": {"compile_only": "pass", "link_run": "pass", "output": "ok"},
        "interposition": {"checked": True, "stable": True},
    }
    document["numerics"]["threading"] = {"implementation": "openblas", "num_threads": 8}
    return document


def test_record_projection_keeps_only_the_committed_facts(tmp_path):
    entry = project_numerics_record(_measured_darwin_document(tmp_path))
    assert validate_numerics_record_entry("darwin-arm64", entry) is entry
    assert set(entry) == {"platform", "numerics"}
    assert set(entry["platform"]) == {"system", "machine", "libc"}
    assert set(entry["numerics"]) == {
        "backend", "blas_integer_bits", "lapack_integer_bits", "numpy_version",
        "symbols", "library_paths", "interp", "multiply_broadcast",
    }
    assert entry["numerics"]["library_paths"][0] == {
        "path": ACCELERATE_BLAS_IMAGE,
        "role": "blas",
        "identity_kind": "apple-system",
        "sha256": None,
    }
    assert entry["numerics"]["interp"] == {"lowering": "contracted"}
    assert entry["numerics"]["multiply_broadcast"] == {"nan_winner": "scalar"}
    text = json.dumps(entry)
    for leaked in (HOST_SENTINEL, "26A434", "26A425", "27.0.1", "Apple clang",
                   "a697e7b9", "BIKE-ABI-OK", "thread_measure", "Xcode",
                   "distinguishing", "queries", "numpy_imports", "build_id",
                   "site_packages", "accelerate_images", "interposition"):
        assert leaked not in text, leaked
    assert "python" not in entry


def test_record_writer_is_byte_identical_for_the_same_measurement(tmp_path):
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, _measured_darwin_document(tmp_path))
    first = lock.read_bytes()
    write_numerics_record(lock, _measured_darwin_document(tmp_path))
    assert lock.read_bytes() == first
    assert first.endswith(b"\n")


def test_record_writer_keeps_the_other_platforms_entry(tmp_path):
    lock = tmp_path / "numerics.lock.json"
    darwin_key = write_numerics_record(lock, _measured_darwin_document(tmp_path))
    linux_key = write_numerics_record(lock, _measured_linux_document(tmp_path))
    assert (darwin_key, linux_key) == ("darwin-arm64", "linux-aarch64")
    records = load_numerics_lock(lock)["records"]
    assert set(records) == {"darwin-arm64", "linux-aarch64"}
    assert records["linux-aarch64"]["numerics"]["backend"] == "NUMPY_OPENBLAS"


def test_linux_record_stores_wheel_relative_provider_paths(tmp_path):
    document = _measured_linux_document(tmp_path)
    entry = project_numerics_record(document)
    assert validate_numerics_record_entry("linux-aarch64", entry) is entry
    provider = entry["numerics"]["library_paths"][0]
    assert provider["path"] == "numpy.libs/libscipy_openblas64_-fixture0.so"
    assert provider["identity_kind"] == "file"
    live_digest = document["numerics"]["library_paths"][0]["sha256"]
    assert provider["sha256"] == live_digest
    assert str(tmp_path) not in json.dumps(entry)


def test_linux_record_is_independent_of_the_install_location(tmp_path):
    first = project_numerics_record(
        _measured_linux_document(tmp_path / "checkout-a"))
    second = project_numerics_record(
        _measured_linux_document(tmp_path / "checkout-b"))
    assert first == second


def test_numerics_lock_rejects_host_state_smuggled_into_a_record(tmp_path):
    entry = project_numerics_record(_measured_darwin_document(tmp_path))
    entry["numerics"]["threading"] = {"implementation": "accelerate", "num_threads": 5}
    with pytest.raises(ValueError, match="extra="):
        validate_numerics_record_entry("darwin-arm64", entry)


def test_numerics_lock_rejects_a_python_or_site_packages_section(tmp_path):
    entry = project_numerics_record(_measured_darwin_document(tmp_path))
    smuggled_python = dict(entry, python={"version": "3.14.3", "soabi": "x"})
    with pytest.raises(ValueError, match="exactly platform and numerics"):
        validate_numerics_record_entry("darwin-arm64", smuggled_python)
    entry["numerics"]["site_packages"] = "/Users/someone/.venv/site-packages"
    with pytest.raises(ValueError, match="site_packages"):
        validate_numerics_record_entry("darwin-arm64", entry)


def test_numerics_lock_key_must_name_the_recorded_platform(tmp_path):
    entry = project_numerics_record(_measured_darwin_document(tmp_path))
    with pytest.raises(ValueError, match="does not match"):
        validate_numerics_record_entry("linux-aarch64", entry)


def test_numerics_lock_path_form_follows_the_backend(tmp_path):
    """Accelerate needs an absolute system image; a wheel provider needs a
    site-packages-relative path. Each form is rejected under the other."""
    linux = project_numerics_record(_measured_linux_document(tmp_path))
    linux["numerics"]["library_paths"][0]["path"] = (
        "/opt/bike-venv-pfc/lib/python3.13/site-packages/numpy.libs/libx.so")
    with pytest.raises(ValueError, match="relative to site-packages"):
        validate_numerics_record_entry("linux-aarch64", linux)
    darwin = project_numerics_record(_measured_darwin_document(tmp_path))
    darwin["numerics"]["library_paths"][0]["path"] = "vecLib.dylib"
    with pytest.raises(ValueError, match="absolute system Accelerate"):
        validate_numerics_record_entry("darwin-arm64", darwin)


def test_numerics_lock_rejects_an_unknown_schema_version(tmp_path):
    lock = tmp_path / "numerics.lock.json"
    lock.write_text(json.dumps({"schema_version": 2, "records": {}}))
    with pytest.raises(ValueError, match="schema_version"):
        load_numerics_lock(lock)


def test_numerics_lock_rejects_an_unqualified_integer_width_on_write(tmp_path):
    document = _measured_darwin_document(tmp_path)
    document["numerics"]["lapack_integer_bits"] = 32
    lock = tmp_path / "numerics.lock.json"
    with pytest.raises(ValueError, match="integer ABI"):
        write_numerics_record(lock, document)
    assert not lock.exists()


def test_staleness_ignores_os_build_compiler_location_and_python(tmp_path):
    """Differences that are machine or OS-build specific must not count as
    stale: the committed facts are the same on every such machine."""
    base = project_numerics_record(_measured_darwin_document(tmp_path))
    other = copy.deepcopy(_measured_darwin_document(tmp_path))
    config = other["numerics"]["provider_config"]
    config["macos_build"] = "26A999"
    config["sdk_build"] = "26B001"
    config["macos_version"] = "27.1"
    config["abi_check"]["compiler"] = "Apple clang version 99.0.0"
    config["accelerate_uuid"] = "00000000-0000-0000-0000-000000000000"
    other["numerics"]["site_packages"] = "/elsewhere/lib/python3.13/site-packages"
    other["numerics"]["interp"]["queries"] = 8192
    other["python"] = {"version": "3.13.12", "soabi": "cpython-313-darwin"}
    assert project_numerics_record(other) == base


def test_staleness_detects_a_changed_consumed_fact(tmp_path):
    base_document = _measured_darwin_document(tmp_path)
    base = project_numerics_record(base_document)
    changes = {
        "interp lowering": lambda d: d["numerics"]["interp"].update(
            lowering="separate"),
        "multiply nan winner": lambda d: d["numerics"]["multiply_broadcast"]
        .update(nan_winner="element"),
        "NumPy version": lambda d: d["numerics"].update(numpy_version="2.5.3"),
        "symbol spelling": lambda d: d["numerics"]["symbols"].update(
            ddot="cblas_ddot_other"),
        "provider identity": lambda d: d["numerics"]["library_paths"][0]
        .update(build_id="macos-other+accelerate+sdk-other"),
    }
    for label, mutate in changes.items():
        changed = copy.deepcopy(base_document)
        mutate(changed)
        if label == "provider identity":
            # build_id is not committed, so this change must NOT be stale.
            assert project_numerics_record(changed) == base, label
        else:
            assert project_numerics_record(changed) != base, label


def test_committed_numerics_lock_is_a_valid_darwin_record():
    lock = load_numerics_lock(NUMERICS_LOCK)
    entry = lock["records"]["darwin-arm64"]
    assert set(entry) == {"platform", "numerics"}
    assert entry["numerics"]["backend"] == "ACCELERATE"
    assert entry["numerics"]["numpy_version"] == REQUIRED_NUMPY_VERSION
    assert entry["numerics"]["interp"] == {"lowering": "contracted"}
    assert entry["numerics"]["multiply_broadcast"] == {"nan_winner": "scalar"}


def test_cli_rejects_output_combined_with_numerics_record(tmp_path):
    result = subprocess.run(
        [sys.executable, str(PROBE_SCRIPT), "--python", sys.executable,
         "--output", str(tmp_path / "doc.json"),
         "--write-numerics-record", str(tmp_path / "lock.json")],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 2
    assert "mutually exclusive" in result.stderr
    assert not (tmp_path / "lock.json").exists()


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="real provider probe needs the pinned scientific environment",
)
def test_cli_numerics_record_is_byte_identical_across_runs(tmp_path):
    outputs = []
    for name in ("first.json", "second.json"):
        target = tmp_path / name
        result = subprocess.run(
            [sys.executable, str(PROBE_SCRIPT), "--python", sys.executable,
             "--write-numerics-record", str(target)],
            capture_output=True, text=True, timeout=600,
        )
        assert result.returncode == 0, result.stderr
        outputs.append(target.read_bytes())
    assert outputs[0] == outputs[1]


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="real provider probe needs the pinned scientific environment",
)
def test_committed_numerics_record_matches_the_live_measurement():
    """Staleness against this host: the committed facts and NumPy version
    must equal a fresh measurement. Machine-specific values are not compared
    because they are not committed."""
    document = probe_dependencies(Path(sys.executable))
    key = record_platform_key(document)
    committed = load_numerics_lock(NUMERICS_LOCK)["records"].get(key)
    if committed is None:
        pytest.skip(f"no committed numerics record for '{key}' yet")
    assert project_numerics_record(document) == committed


# --------------------------------------------------------------------------
# Configure-time verdict: check_numerics_record and --check-numerics-record.
# The verdict projects an already-measured document and compares it with the
# committed entry. No probe runs here, so fixture documents are enough.
# --------------------------------------------------------------------------


def test_numerics_check_accepts_the_matching_measurement(tmp_path):
    document = _document_with_numerics(tmp_path, "Darwin")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    assert check_numerics_record(lock, document) == "darwin-aarch64"


def test_numerics_check_ignores_machine_specific_values(tmp_path):
    """The install location and build id are configure-time values, not
    committed facts: changing them must not make the record stale."""
    document = _document_with_numerics(tmp_path, "Darwin")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    moved = copy.deepcopy(document)
    for entry in moved["numerics"]["library_paths"]:
        entry["build_id"] = "macos-99Z999+accelerate-other+sdk-99Z999"
    assert check_numerics_record(lock, moved) == "darwin-aarch64"


def test_numerics_check_names_a_flipped_fact_with_the_regeneration_command(
        tmp_path):
    document = _document_with_numerics(tmp_path, "Darwin")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    stored = json.loads(lock.read_text(encoding="utf-8"))
    stored["records"]["darwin-aarch64"]["numerics"]["interp"]["lowering"] = \
        "separate"
    lock.write_text(json.dumps(stored), encoding="utf-8")
    with pytest.raises(ValueError) as caught:
        check_numerics_record(lock, document)
    message = str(caught.value)
    assert "entry.numerics.interp.lowering" in message
    assert "is stale against the live measurement" in message
    assert (f"tools/native_dependencies.py --python {sys.executable} "
            f"--write-numerics-record {lock}") in message


def test_numerics_check_names_a_missing_platform_entry(tmp_path):
    """The lock holds only the Linux entry; this Darwin host has no record
    and must be told to regenerate, not silently pass."""
    linux_document = _document_with_numerics(tmp_path / "linux", "Linux")
    darwin_document = _document_with_numerics(tmp_path / "darwin", "Darwin")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, linux_document)
    with pytest.raises(ValueError, match="has no committed entry") as caught:
        check_numerics_record(lock, darwin_document)
    assert "--write-numerics-record" in str(caught.value)


def test_cli_numerics_check_prints_the_key_and_fails_with_the_hint(tmp_path):
    document = _document_with_numerics(tmp_path, "Darwin")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    probe = tmp_path / "probe.json"
    probe.write_text(json.dumps(document), encoding="utf-8")
    command = [sys.executable, str(PROBE_SCRIPT), "--check-numerics-record",
               str(lock), "--probe-document", str(probe)]
    matching = subprocess.run(command, capture_output=True, text=True,
                              timeout=60)
    assert matching.returncode == 0, matching.stderr
    assert matching.stdout.strip() == "darwin-aarch64"

    stored = json.loads(lock.read_text(encoding="utf-8"))
    stored["records"]["darwin-aarch64"]["numerics"]["multiply_broadcast"][
        "nan_winner"] = "element"
    lock.write_text(json.dumps(stored), encoding="utf-8")
    stale = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert stale.returncode == 1
    assert stale.stdout == ""
    assert "numerics record check failed" in stale.stderr
    assert "--write-numerics-record" in stale.stderr


def test_cli_numerics_check_does_not_combine_with_a_probe_run(tmp_path):
    result = subprocess.run(
        [sys.executable, str(PROBE_SCRIPT),
         "--check-numerics-record", str(tmp_path / "lock.json"),
         "--probe-document", str(tmp_path / "probe.json"),
         "--python", sys.executable],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 2
    assert "does not combine" in result.stderr


# --------------------------------------------------------------------------
# Record identity: libc family, relocation, missing record, and the real
# legacy graph (the committed record, a tampered copy, and the identity header).
# --------------------------------------------------------------------------


def test_libc_projection_keeps_the_family_not_the_version(tmp_path):
    """The family decides the provider, so it is a fact; the version is not
    committed, so a point release cannot make the record stale."""
    entry = project_numerics_record(_document_with_numerics(tmp_path, "Linux"))
    assert entry["platform"]["libc"] == "glibc"
    assert libc_family(None) is None


def test_unknown_libc_family_fails_closed():
    with pytest.raises(ValueError, match="unsupported libc"):
        libc_family("uclibc 1.0")


def test_libc_version_change_with_identical_facts_is_accepted(tmp_path):
    document = _document_with_numerics(tmp_path, "Linux")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    patched = copy.deepcopy(document)
    patched["platform"]["libc"] = "glibc 2.40"
    assert check_numerics_record(lock, patched) == "linux-aarch64"


def test_libc_family_change_is_rejected(tmp_path):
    document = _document_with_numerics(tmp_path, "Linux")
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(lock, document)
    patched = copy.deepcopy(document)
    patched["platform"]["libc"] = "musl 1.2.5"
    with pytest.raises(ValueError, match="entry.platform.libc") as caught:
        check_numerics_record(lock, patched)
    assert "committed 'glibc', live 'musl'" in str(caught.value)


def test_record_rejects_a_libc_version_in_the_committed_entry(tmp_path):
    entry = project_numerics_record(_document_with_numerics(tmp_path, "Linux"))
    entry["platform"]["libc"] = "glibc 2.39"
    with pytest.raises(ValueError, match="libc family"):
        validate_numerics_record_entry("linux-aarch64", entry)


def test_linux_record_is_accepted_after_relocating_site_packages(tmp_path):
    """The record holds the provider relative to site-packages, so the same
    wheel installed elsewhere, with the same bytes, passes the check."""
    lock = tmp_path / "numerics.lock.json"
    write_numerics_record(
        lock, _document_with_numerics(tmp_path / "checkout-a", "Linux"))
    moved = _document_with_numerics(tmp_path / "checkout-b", "Linux")
    assert check_numerics_record(lock, moved) == "linux-aarch64"


def test_check_names_the_regeneration_command_for_a_missing_record(tmp_path):
    document = _document_with_numerics(tmp_path, "Linux")
    missing = tmp_path / "absent.lock.json"
    with pytest.raises(ValueError) as caught:
        check_numerics_record(missing, document)
    message = str(caught.value)
    assert "unreadable" in message
    assert (f"tools/native_dependencies.py --python {sys.executable} "
            f"--write-numerics-record {missing}") in message


def _configure_legacy(tmp_path: Path, record: Path) -> subprocess.CompletedProcess:
    """Configure the real legacy graph (cmake -S native) into a fresh build dir.

    The probe and find_package(Python) must use this test's interpreter, so
    its bin directory leads PATH.
    """
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join(
        [str(Path(sys.executable).parent), env.get("PATH", "")])
    return subprocess.run(
        [cmake, "-S", str(REPO_ROOT / "native"),
         "-B", str(tmp_path / "legacy-build"),
         f"-DBIKE_NUMERICS_RECORD={record}"],
        capture_output=True, text=True, timeout=900, env=env,
    )


def _committed_key_or_skip() -> str:
    key = f"{platform.system().lower()}-{platform.machine()}"
    if key not in load_numerics_lock(NUMERICS_LOCK)["records"]:
        pytest.skip(f"no committed numerics record for '{key}' yet")
    return key


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="legacy graph configure needs the pinned scientific environment",
)
def test_legacy_graph_configure_fails_on_a_tampered_committed_record(tmp_path):
    key = _committed_key_or_skip()
    lock = json.loads(NUMERICS_LOCK.read_text(encoding="utf-8"))
    nan_winner = lock["records"][key]["numerics"]["multiply_broadcast"]
    nan_winner["nan_winner"] = (
        "element" if nan_winner["nan_winner"] == "scalar" else "scalar")
    tampered = tmp_path / "numerics.lock.json"
    tampered.write_text(json.dumps(lock), encoding="utf-8")
    result = _configure_legacy(tmp_path, tampered)
    assert result.returncode != 0, result.stdout
    stderr = squashed(result.stderr)
    assert "is stale against the live measurement" in stderr
    assert "entry.numerics.multiply_broadcast.nan_winner" in stderr
    assert "--write-numerics-record" in stderr


@pytest.mark.skipif(
    not _real_environment_available(),
    reason="legacy graph configure needs the pinned scientific environment",
)
def test_legacy_identity_header_takes_its_facts_from_the_record(tmp_path):
    """The runtime identity's role, identity_kind and sha256 come from the
    committed record; its path is the bound real path; build_id is live."""
    key = _committed_key_or_skip()
    result = _configure_legacy(tmp_path, NUMERICS_LOCK)
    assert result.returncode == 0, result.stderr
    headers = list((tmp_path / "legacy-build").rglob(
        "research_build_identity.hpp"))
    assert len(headers) == 1
    text = headers[0].read_text(encoding="utf-8")
    match = re.search(
        r'numerics_provider_policy_json\[\] = R"identity\((.*?)\)identity"',
        text)
    assert match, "identity header has no provider policy"
    policy = json.loads(match.group(1))
    committed = load_numerics_lock(NUMERICS_LOCK)["records"][key]["numerics"]
    assert policy["backend"] == committed["backend"]
    assert len(policy["libraries"]) == len(committed["library_paths"])
    for bound, recorded in zip(policy["libraries"], committed["library_paths"]):
        assert bound["role"] == recorded["role"]
        assert bound["identity_kind"] == recorded["identity_kind"]
        assert bound["sha256"] == recorded["sha256"]
        assert Path(bound["path"]).is_absolute()
        if recorded["identity_kind"] == "apple-system":
            assert bound["path"] == recorded["path"]
            assert bound["build_id"]
