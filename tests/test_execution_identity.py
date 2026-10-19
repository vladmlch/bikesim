"""Data-only contract for versioned numerical execution identity.

These tests exercise only ``bike_sim.native.execution_identity``: pure
schema validation and recursive field-path diagnostics. No MuJoCo, NumPy,
or native binary is imported by the module under test, and no recorded
path is ever opened.
"""
from copy import deepcopy
import sys

import pytest

from bike_sim.native.execution_identity import (
    execution_identity_differences, require_execution_compatible,
    validate_execution_identity,
)


def legacy_python():
    return {"backend": "python", "runtime_schema": 1,
            "python_source_sha256": "a" * 64, "native_source_sha256": None,
            "extension_sha256": None, "mujoco_version": "3.12.0",
            "mujoco_library_sha256": None, "build_context": None}


def legacy_native():
    value = legacy_python()
    value.update(backend="native", native_source_sha256="b" * 64,
                 extension_sha256="c" * 64, mujoco_library_sha256="d" * 64,
                 build_context=dict(build_type="Release", compiler_id="AppleClang",
                                    compiler_version="21.0.0",
                                    numerical_flags="-O3 -ffp-contract=off"))
    return value


def platform_darwin():
    return {"system": "Darwin", "machine": "aarch64",
            "libc": {"name": "libSystem", "version": "26A434"},
            "os_build": "26A434", "python_version": "3.14.3",
            "python_soabi": "cpython-314-darwin"}


def platform_linux():
    return {"system": "Linux", "machine": "aarch64",
            "libc": {"name": "glibc", "version": "2.39"},
            "os_build": "7.0.14-linuxkit", "python_version": "3.13.16",
            "python_soabi": "cpython-313-aarch64-linux-gnu"}


def numerics_accelerate():
    return {"backend": "ACCELERATE", "numpy_version": "2.5.2",
            "blas_integer_bits": 32, "lapack_integer_bits": 64,
            "cpu_profile": "Apple M4 Max",
            "threading": {"implementation": "accelerate", "num_threads": 4},
            "libraries": [{"role": "blas_lapack", "identity_kind": "apple-system",
                           "sha256": None,
                           "build_id": "macos-26A434+accelerate-"
                           "a697e7b9-8ff4-356c-a507-64b38af1ccaa+sdk-26A425"}]}


def numerics_openblas():
    return {"backend": "NUMPY_OPENBLAS", "numpy_version": "2.5.2",
            "blas_integer_bits": 64, "lapack_integer_bits": 64,
            "cpu_profile": "neoversen1",
            "threading": {"implementation": "pthreads", "num_threads": 16},
            "libraries": [{"role": "blas_lapack", "identity_kind": "file",
                           "sha256": "5" * 64, "build_id": None}]}


def v2_python():
    value = legacy_python()
    value["identity_schema"] = 2
    value["build_context"] = dict(platform=platform_darwin(),
                                  numerics=numerics_accelerate())
    return value


def v2_native():
    value = legacy_native()
    value["identity_schema"] = 2
    value["build_context"].update(platform=platform_darwin(),
                                  numerics=numerics_accelerate())
    return value


def v2_native_linux():
    value = deepcopy(v2_native())
    context = value["build_context"]
    context["platform"] = platform_linux()
    context["numerics"] = numerics_openblas()
    context["compiler_id"] = "Clang"
    return value


# ---------------------------------------------------------------------------
# Schema acceptance: legacy and v2
# ---------------------------------------------------------------------------


def test_legacy_read_stays_valid():
    value = legacy_python()
    assert validate_execution_identity(value) == value
    assert "identity_schema" not in value


def test_legacy_native_read_stays_valid():
    assert validate_execution_identity(legacy_native()) == legacy_native()


@pytest.mark.parametrize("make", [v2_python, v2_native, v2_native_linux])
def test_v2_records_validate(make):
    value = make()
    assert validate_execution_identity(value) is value
    assert value["identity_schema"] == 2
    assert value["runtime_schema"] == 1


def test_validation_does_not_mutate_the_record():
    for make in (legacy_python, legacy_native, v2_python, v2_native):
        value = make()
        snapshot = deepcopy(value)
        validate_execution_identity(value)
        assert value == snapshot


# ---------------------------------------------------------------------------
# Identity-schema dispatch is exact — nothing is guessed
# ---------------------------------------------------------------------------


def test_unknown_identity_schema_is_not_guessed():
    value = legacy_python()
    value["identity_schema"] = 999
    with pytest.raises(ValueError, match="identity_schema"):
        validate_execution_identity(value)


@pytest.mark.parametrize("schema", [True, False, 1, 3, "2", 2.0, None])
def test_noncanonical_identity_schema_fails(schema):
    value = v2_python()
    value["identity_schema"] = schema
    with pytest.raises(ValueError, match="identity_schema"):
        validate_execution_identity(value)


@pytest.mark.parametrize("make", [v2_python, v2_native])
def test_v2_rejects_missing_and_extra_top_level_keys(make):
    value = make()
    for key in list(value):
        bad = deepcopy(value)
        del bad[key]
        with pytest.raises(ValueError):
            validate_execution_identity(bad)
    bad = deepcopy(value)
    bad["extra"] = "untracked"
    with pytest.raises(ValueError):
        validate_execution_identity(bad)


def test_legacy_rejects_the_v2_field_set():
    value = legacy_python()
    value["extra"] = 1
    with pytest.raises(ValueError):
        validate_execution_identity(value)


@pytest.mark.parametrize("key", ["build_type", "compiler_id",
                                 "compiler_version", "numerical_flags",
                                 "platform", "numerics"])
def test_native_context_requires_exactly_six_keys(key):
    value = v2_native()
    del value["build_context"][key]
    with pytest.raises(ValueError, match="build_context"):
        validate_execution_identity(value)
    value = v2_native()
    value["build_context"]["injected"] = "untracked"
    with pytest.raises(ValueError, match="build_context"):
        validate_execution_identity(value)


def test_python_v2_context_is_only_platform_and_numerics():
    value = v2_python()
    value["build_context"]["build_type"] = "Release"
    with pytest.raises(ValueError, match="build_context"):
        validate_execution_identity(value)


# ---------------------------------------------------------------------------
# Field-level rules — the same strictness the probe applies to its own record
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("backend,field,bad", [
    ("python", "runtime_schema", True),
    ("python", "runtime_schema", 2),
    ("native", "python_source_sha256", "not-a-digest"),
    ("python", "extension_sha256", "0" * 64),
    ("python", "native_source_sha256", "0" * 64),
    ("python", "mujoco_library_sha256", "0" * 64),
    ("native", "extension_sha256", None),
    ("native", "mujoco_version", ""),
])
def test_field_type_and_null_rules(backend, field, bad):
    maker = v2_python if backend == "python" else v2_native
    value = maker()
    value[field] = bad
    with pytest.raises(ValueError):
        validate_execution_identity(value)


@pytest.mark.parametrize("path,bad", [
    (("platform", "system"), "FreeBSD"),
    (("platform", "machine"), ""),
    (("platform", "machine"), "ARM 64"),
    (("platform", "libc"), {"name": "glibc"}),
    (("platform", "libc"), {"name": "glibc", "version": "2.39", "x": 1}),
    (("platform", "libc", "name"), ""),
    (("platform", "os_build"), "   "),
    (("platform", "python_version"), ""),
    (("platform", "python_soabi"), None),
    (("platform", "extra"), "untracked"),
    (("numerics", "extra"), "untracked"),
    (("numerics", "backend"), "MKL"),
    (("numerics", "numpy_version"), ""),
    (("numerics", "blas_integer_bits"), True),
    (("numerics", "blas_integer_bits"), 64),
    (("numerics", "lapack_integer_bits"), 32),
    (("numerics", "cpu_profile"), ""),
    (("numerics", "threading"), {"implementation": "accelerate"}),
    (("numerics", "threading", "num_threads"), 0),
    (("numerics", "threading", "num_threads"), True),
    (("numerics", "libraries"), []),
    (("numerics", "libraries", 0, "role"), "blas"),
    (("numerics", "libraries", 0, "identity_kind"), "file"),
    (("numerics", "libraries", 0, "sha256"), "f" * 64),
    (("numerics", "libraries", 0, "build_id"), None),
    (("numerics", "libraries", 0, "build_id"), ""),
    (("numerics", "libraries", 0, "extra"), "untracked"),
])
def test_nested_field_rules_fail_closed(path, bad):
    value = v2_native()  # ACCELERATE record: file identity is a violation
    node = value["build_context"]
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = bad
    with pytest.raises(ValueError):
        validate_execution_identity(value)


@pytest.mark.parametrize("path,bad", [
    (("numerics", "blas_integer_bits"), 32),
    (("numerics", "libraries", 0, "identity_kind"), "apple-system"),
    (("numerics", "libraries", 0, "sha256"), None),
    (("numerics", "libraries", 0, "sha256"), "FF" * 32),
    (("numerics", "libraries", 0, "build_id"), "fabricated"),
])
def test_linux_file_identity_rules(path, bad):
    value = v2_native_linux()
    node = value["build_context"]
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = bad
    with pytest.raises(ValueError):
        validate_execution_identity(value)


# ---------------------------------------------------------------------------
# Diagnostics name the exact dotted field
# ---------------------------------------------------------------------------


def test_mismatch_names_the_field():
    left = legacy_python()
    right = deepcopy(left)
    right["mujoco_version"] = "3.12.1"
    differences = execution_identity_differences(left, right)
    assert any("mujoco_version" in item for item in differences)


@pytest.mark.parametrize("make,path,bad", [
    # architecture, libc, interpreter identity
    (v2_native, ("build_context", "platform", "machine"), "x86_64"),
    (v2_native, ("build_context", "platform", "libc", "name"), "musl"),
    (v2_native, ("build_context", "platform", "libc", "version"), "26B001"),
    (v2_native, ("build_context", "platform", "python_version"), "3.14.4"),
    (v2_native, ("build_context", "platform", "python_soabi"),
     "cpython-315-darwin"),
    (v2_native, ("build_context", "platform", "os_build"), "26B001"),
    # NumPy/provider identity
    (v2_native, ("build_context", "numerics", "numpy_version"), "2.5.3"),
    (v2_native, ("build_context", "numerics", "cpu_profile"), "Apple M3"),
    (v2_native, ("build_context", "numerics", "threading", "num_threads"), 8),
    (v2_native, ("build_context", "numerics", "threading", "implementation"),
     "openmp"),
    (v2_native, ("build_context", "numerics", "libraries", 0, "build_id"),
     "macos-26B001+accelerate-00000000-0000-4000-8000-000000000000+sdk-26A425"),
    (v2_native_linux, ("build_context", "numerics", "libraries", 0, "sha256"),
     "9" * 64),
    # Compiler numerical flags and the native source digest
    (v2_native, ("build_context", "numerical_flags"),
     "-O3 -ffp-contract=off -ffast-math"),
    (v2_native, ("build_context", "compiler_version"), "21.0.1"),
    (v2_native, ("native_source_sha256",), "9" * 64),
    (v2_native, ("mujoco_library_sha256",), "9" * 64),
    (v2_native, ("mujoco_version",), "3.12.1"),
])
def test_single_field_mismatch_reports_the_dotted_key(make, path, bad):
    recorded = make()
    current = deepcopy(recorded)
    node = current
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = bad
    differences = execution_identity_differences(recorded, current)
    dotted = "execution"
    for part in path:
        dotted += f"[{part}]" if isinstance(part, int) else f".{part}"
    assert any(dotted in item for item in differences), (
        f"expected {dotted} in {differences}")
    assert len(differences) == 1


@pytest.mark.parametrize("make,path", [
    (v2_native, ("numerics", "blas_integer_bits")),
    (v2_native, ("numerics", "lapack_integer_bits")),
    (v2_native_linux, ("numerics", "blas_integer_bits")),
])
def test_integer_abi_change_is_a_named_failure(make, path):
    # Integer widths are pinned per qualified backend — any mutation makes the
    # record unqualified, so the diagnostic is the validation error itself.
    value = make()
    node = value["build_context"]
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = 128  # never a qualified width for any backend
    with pytest.raises(ValueError, match=path[-1]):
        validate_execution_identity(value)


def test_os_change_is_a_named_failure():
    # The qualified pairings are Darwin+ACCELERATE and Linux+NUMPY_OPENBLAS;
    # a bare OS mutation leaves an inconsistent record that validation rejects.
    value = v2_native()
    value["build_context"]["platform"]["system"] = "Linux"
    with pytest.raises(ValueError, match="platform.system"):
        validate_execution_identity(value)
    value = v2_native_linux()
    value["build_context"]["platform"]["system"] = "Darwin"
    with pytest.raises(ValueError, match="platform.system"):
        validate_execution_identity(value)


def test_os_and_backend_pairing_change_reports_the_fields():
    # A consistent foreign-platform record stays readable and diffs cleanly.
    recorded = v2_native()
    current = v2_native_linux()
    differences = execution_identity_differences(recorded, current)
    fields = {item.split(":")[0] for item in differences}
    assert "execution.build_context.platform.system" in fields
    assert "execution.build_context.numerics.backend" in fields


def test_provider_hash_names_the_indexed_library_path():
    recorded = v2_native_linux()
    current = deepcopy(recorded)
    current["build_context"]["numerics"]["libraries"][0]["sha256"] = "e" * 64
    differences = execution_identity_differences(recorded, current)
    assert any("execution.build_context.numerics.libraries[0].sha256"
               in item for item in differences)


def test_library_list_length_difference_is_indexed():
    recorded = v2_native_linux()
    current = deepcopy(recorded)
    current["build_context"]["numerics"]["libraries"].append(
        dict(recorded["build_context"]["numerics"]["libraries"][0],
             sha256="7" * 64))
    differences = execution_identity_differences(recorded, current)
    assert any("libraries[1]" in item for item in differences)


def test_legacy_vs_v2_reports_the_missing_identity_fields():
    recorded = legacy_python()
    current = v2_python()
    differences = execution_identity_differences(recorded, current)
    assert any("identity_schema" in item for item in differences)
    assert any("build_context" in item for item in differences)


def test_differences_validate_both_sides():
    with pytest.raises(ValueError):
        execution_identity_differences({"backend": "python"}, v2_python())
    with pytest.raises(ValueError):
        execution_identity_differences(v2_python(), {"backend": "python"})


# ---------------------------------------------------------------------------
# require_execution_compatible
# ---------------------------------------------------------------------------


def test_require_execution_compatible_accepts_identity():
    require_execution_compatible(v2_native(), deepcopy(v2_native()))
    require_execution_compatible(legacy_python(), deepcopy(legacy_python()))


def test_require_execution_compatible_raises_with_named_fields():
    recorded = v2_native()
    current = deepcopy(recorded)
    current["build_context"]["numerics"]["libraries"][0]["build_id"] = "changed"
    current["mujoco_version"] = "3.12.1"
    with pytest.raises(ValueError, match="execution identity mismatch") as held:
        require_execution_compatible(recorded, current)
    message = str(held.value)
    assert "execution.build_context.numerics.libraries[0].build_id" in message
    assert "execution.mujoco_version" in message


def test_require_execution_compatible_rejects_legacy_recorded():
    with pytest.raises(ValueError, match="execution identity mismatch"):
        require_execution_compatible(legacy_python(), v2_python())


# ---------------------------------------------------------------------------
# The read path performs no imports — it must stay data-only
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("make", [legacy_python, legacy_native, v2_python,
                                  v2_native, v2_native_linux])
def test_validation_imports_nothing(make):
    # The package itself is already imported by collection; the contract is
    # that validation/diff perform no new imports of the numerical stack.
    before = set(sys.modules)
    validate_execution_identity(make())
    execution_identity_differences(make(), deepcopy(make()))
    require_execution_compatible(make(), deepcopy(make()))
    new = set(sys.modules) - before
    heavy = {module for module in new
             if module.split(".")[0] in ("mujoco", "numpy", "scipy",
                                        "bike_native", "threadpoolctl")}
    assert not heavy, f"validation imported {sorted(heavy)}"
