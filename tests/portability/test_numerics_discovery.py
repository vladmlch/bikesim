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

import hashlib
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tools.native_dependencies import (
    REQUIRED_NUMPY_VERSION,
    checked_symbol,
    probe_dependencies,
    require_same_library,
    validate_dependencies,
    validate_numerics,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PROBE_SCRIPT = REPO_ROOT / "tools" / "native_dependencies.py"
NUMERICS_CMAKE = REPO_ROOT / "native" / "cmake" / "BikeNumerics.cmake"

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
    for key in ("numpy_version", "library_paths", "symbols", "threading"):
        record = _openblas_record(tmp_path / key)
        record.pop(key)
        with pytest.raises(ValueError, match=key.split("_")[0]):
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


def _configure_scratch(
    tmp_path: Path, document: dict, backend: str = "AUTO"
) -> subprocess.CompletedProcess:
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    scratch = tmp_path / "scratch"
    scratch.mkdir()
    doc = dict(document)
    doc["numerics"] = doc.get("numerics")
    json_path = tmp_path / "dependencies.json"
    json_path.write_text(json.dumps(doc))
    (scratch / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.24)\n"
        "project(numerics_check NONE)\n"
        f'set(BIKE_TARGET_SYSTEM "{document["platform"]["system"]}")\n'
        f'file(READ "{json_path}" _native_dependencies_json)\n'
        f'include("{NUMERICS_CMAKE}")\n'
        'message(STATUS "RESOLVED=${BIKE_NUMERICS_BACKEND_RESOLVED}")\n'
        'message(STATUS "DDOT=${_native_numerics_symbol_ddot}")\n'
        'message(STATUS "DGELSD=${_native_numerics_symbol_dgelsd}")\n'
    )
    return subprocess.run(
        [cmake, "-S", str(scratch), "-B", str(tmp_path / "scratch-build"),
         f"-DBIKE_NUMERICS_BACKEND={backend}"],
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


def test_bike_numerics_rejects_a_backend_mismatch(tmp_path):
    """Requesting NUMPY_OPENBLAS on a Darwin host (or vice versa) is a
    provider mismatch — it must fail before any physics target exists."""
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    wrong = "NUMPY_OPENBLAS" if system == "Darwin" else "ACCELERATE"
    result = _configure_scratch(tmp_path, document, backend=wrong)
    assert result.returncode != 0
    assert "numerics" in result.stderr.lower() or "backend" in result.stderr.lower()


def test_bike_numerics_rejects_an_unqualified_width(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    document["numerics"]["blas_integer_bits"] = None
    result = _configure_scratch(tmp_path, document)
    assert result.returncode != 0


def test_bike_numerics_rejects_a_tampered_symbol(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    document["numerics"]["symbols"]["ddot"] = 'ddot";execute_process(COMMAND rm -rf'
    result = _configure_scratch(tmp_path, document)
    assert result.returncode != 0


def test_bike_numerics_rejects_a_missing_numerics_record(tmp_path):
    system = platform.system()
    document = _document_with_numerics(tmp_path, system)
    document["numerics"] = None
    result = _configure_scratch(tmp_path, document)
    assert result.returncode != 0
