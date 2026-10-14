# 03 - Numerical compatibility and execution identity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Make Linux match its pinned local oracle without weakening macOS or replay verification.

**Architecture:** Keep mathematical algorithms and call order shared. Isolate genuinely provider- or CPU-specific numerical lowerings, qualify them with counterexamples, and version environment identity separately from physics and recording schemas.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.24, and strict ISO C++23 without compiler language extensions.
- Keep MuJoCo exactly 3.12.0 for native execution until its private error ABI is deliberately migrated in a separate change.
- Preserve the macOS Accelerate backend, SDK override/fallback behavior, mjpython launcher, and current application commands.
- Preserve same-environment native/Python bitwise tests, exception safety, source/artifact identity, and fail-closed verification.
- Do not promise bitwise-identical results between different operating systems, CPUs, BLAS implementations, or dependency builds.
- Do not change physics, fixed integration periods, controller behavior, recording opt-in/final-only semantics, or replay tolerances for this port.
- Use uv for project Python environments. Never silently regenerate the production lock or upgrade dependencies during a build.
- No implicit native compilation on application import; no silent native-to-Python fallback; no required GUI for headless execution.
- Keep quick/native/full profile meanings; missing required analysis tools remain errors, not successful skips.
- Treat the supplied Repomix export as a source snapshot, not a complete Git checkout or a runnable distribution.
- Native builds are the initial delivery target. Cross-compilation, universal binaries, Windows, and musl are outside this change.

---

## File map and prerequisites

T11 follows T06-T10 and must complete before Linux numerical support is declared. T12 can develop its data-only validators in parallel, but its measured provider identity depends on T06/T11. This is a correctness package, not permission to replace exact comparisons with tolerances.

New files separate diagnostics (`tools/numeric_portability_probe.py`), provider-specific lowering (`native/src/platform/numpy_math_linux.*` only when demonstrated necessary), and data-only identity validation (`src/bike_sim/native/execution_identity.py`). Existing scientific call sites stay in their current files. A new diagnostic file is not a new production numerical dependency.

### T11: Qualify every platform-sensitive numerical lowering against the local oracle

**Files:** Modify `native/src/numeric_numpy.hpp`, `native/src/numeric_norm.hpp`, `native/src/numeric_sincos.hpp`, `native/src/numeric_sum.hpp`, `native/src/binding.cpp`, and only the demonstrated call sites in `native/src/pneumatic/carcass.cpp` or other failing kernels. Create `tools/numeric_portability_probe.py`, `tests/reference/test_native_numeric_portability.py`, and `docs/agents/native-numerical-profiles.md`. Create `native/src/platform/numpy_math_linux.hpp` and `.cpp` only for lowerings that need a Linux implementation; include them in T10 source ownership and policy. Preserve `tests/reference/_bits.py` and the meaning of `tests/reference/conftest.py::_pin_oracle_kernels`.

**Interfaces:** Consume T06 verified provider metadata and T07/T08 adapters. Retain `_debug_pneumatic_math(xs, ys) -> dict[tanh: ndarray, hypot: ndarray]`. Add debug-only `_debug_numeric_contract(name: str, xs: array, ys: array) -> ndarray` with names `python_hypot`, `minimum`, `maximum`, `sum`, `sin`, `cos`; each invokes the same existing production helper/call sequence used by its documented Python counterpart, not an alternative diagnostic implementation. Produce probe JSON schema 1 with `environment`, `operation`, `input_uint64`, `expected_uint64`, `actual_uint64`, `first_mismatch`, and `classification`.

**Reason and boundaries:** The source has an ARM/NEON-oriented tanh lowering, local libm assumptions, a CPython 3.14.3 norm port, and exact reduction ordering. Matching the BLAS DSO does not settle these. A finite input yielding one differing bit is a real failure at an existing exact boundary. A platform-specific implementation is acceptable; changing the oracle or arithmetic semantics is not.

- [ ] **Step 1 - Add the regression and a valid control.**

Start with the existing debug surface, so the first probe can run before any new binding exists:

```python
import numpy as np
import pytest

@pytest.mark.parametrize("length", [1, 2, 3, 7, 8, 9, 31, 32, 33, 65])
def test_tanh_and_hypot_match_local_array_ufunc(native_module, length):
    pool = np.array([-0.0, 0.0, -0.125, 0.125, -1.0, 1.0,
                     -7.25, 7.25, -19.0, 19.0, 1e-300, -1e-300],
                    dtype=np.float64)
    xs = np.resize(pool, length).copy()
    ys = np.resize(pool[::-1], length).copy()
    got = native_module._debug_pneumatic_math(xs, ys)
    with np.errstate(all="ignore"):
        expected = {"tanh": np.tanh(xs), "hypot": np.hypot(xs, ys)}
    for name in expected:
        assert got[name].dtype == expected[name].dtype
        np.testing.assert_array_equal(got[name].view(np.uint64),
                                      expected[name].view(np.uint64))
```

Add fixed uint64 fixtures for signed zeros, smallest subnormals, adjacent values around tanh saturation/rounding boundaries, infinities, and quiet/signaling NaNs. Never generate expected results from the native implementation. Preserve existing per-operation NaN-payload expectations; where NumPy itself has no stable cross-dispatch payload promise, retain local observed behavior and avoid claiming a stronger cross-platform contract. Add C/F-contiguous, strided, reversed, aligned and offset-array probes wherever the actual production API accepts them.

For the new `python_hypot` hook compare each pair with `math.hypot(float(x), float(y))`; do not use `np.hypot` as its oracle. For the existing pneumatic hook use `np.hypot`, not `math.hypot`. For min/max compare to the exact NumPy call shape from the production Python function. Reduction tests cover zero elements, one element, cancellation, powers of two plus small increments, and lengths around every unroll boundary.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_numeric_portability.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Implement the diagnostic before changing numerical code. Its comparison core must preserve bit patterns:

```python
import numpy as np

def compare_arrays(expected, actual):
    expected = np.ascontiguousarray(expected, dtype=np.float64)
    actual = np.ascontiguousarray(actual, dtype=np.float64)
    if actual.shape != expected.shape:
        return {"classification": "shape", "expected_shape": list(expected.shape),
                "actual_shape": list(actual.shape)}
    eb = expected.view(np.uint64).ravel()
    ab = actual.view(np.uint64).ravel()
    bad = np.flatnonzero(eb != ab)
    return {"classification": "equal" if not bad.size else "bits",
            "expected_uint64": [int(v) for v in eb],
            "actual_uint64": [int(v) for v in ab],
            "first_mismatch": None if not bad.size else int(bad[0])}
```

The command is `uv run --frozen --group native python tools/numeric_portability_probe.py --build ABSOLUTE_BUILD --output PATH`; it loads only that artifact using the application loader, records interpreter/provider/CPU configuration, and exits nonzero on a mismatch. It saves inputs as uint64 values so NaNs and signed zeros survive JSON. Keep the corpus deterministic; use a fixed RNG seed and a checked-in adversarial corpus, not wall-clock seeds.

For each failing operation, follow these concrete steps and append the evidence to native-numerical-profiles.md:

1. Reproduce the failing production Python call with the smallest input and exact dtype/shape/strides. Record whether it is a Python scalar/libm call, NumPy scalar loop, NumPy vector ufunc, reduction, GEMV/GEMM dispatch, or DGELSD call. Inspect the pinned NumPy/CPython source, not a different release. E03-E06 in the audit supply authoritative entry points.
2. Trace the existing C++ helper from the failing test to its production callers. For `np_tanh` start in numeric_numpy.hpp and the pneumatic carcass path; for Python norm start in numeric_norm.hpp; for sin/cos start in numeric_sincos.hpp; for sum start in numeric_sum.hpp. Do not rewrite unrelated kernels after one scalar failure.
3. Establish the local NumPy implementation actually selected on each qualified CPU. NumPy runtime SIMD reporting plus pinned source dispatch tables identify candidates; controlled separate-process feature-disable experiments may diagnose selection, but disabling SIMD globally is not a shipping fix. Test scalar tails and vector lanes separately.
4. If libc/libm already matches a call shape, retain a narrow call to that operation and make its FP contraction/rounding behavior explicit. If a vector polynomial/table differs, copy the exact algorithm and constants from the compatible upstream version into a small licensed implementation, retain upstream attribution/license notices, and preserve argument reduction, instruction-order rounding, explicit FMA versus separate multiply/add, lane/tail handling, signed-zero and NaN branches. Do not approximate it with a nearby standard-library function.
5. Dispatch by a validated numerical profile/capability, not `__linux__` alone when two x86 CPUs choose different algorithms. Define the selected profile in generated build metadata, check it against runtime CPU/provider information before constructing the native runtime, and reject an unqualified mismatch. Support the required baseline machines first, then qualify additional dispatch classes with the same corpus. Never silently fall back to the Python backend.
6. Preserve the macOS implementation under its existing qualified profile. Compare before/after outputs on the actual macOS baseline. Do not make Linux's polynomial the new universal implementation.
7. Re-run the scalar/array tests, each affected kernel oracle, DGELSD tests, and complete native reference suite. Fix the earliest divergence. A changed cumulative rollout can be the symptom of an earlier scalar error; save first-divergence index and owning kernel rather than loosening end-state tolerances.

Add each demonstrated mismatch as a permanent input fixture. The Python 3.13 and baseline-interpreter lanes are independent: copying CPython 3.14 math.hypot into C++ does not prove it matches 3.13. Keep the matching helper per qualified profile if the corpus/source comparison demonstrates a difference. Production Python/Numba smoke tests run separately without `_pin_oracle_kernels`; do not change that fixture to make the two lanes appear identical.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
BUILD="$(pwd)/native/build/linux-clang-release"
BIKE_NATIVE_BUILD_PATH="$BUILD" NATIVE_TEST_BUILD_PATH="$BUILD" uv run --frozen --group native python tools/numeric_portability_probe.py --build "$BUILD" --output output/portability/numerics.json
NATIVE_TEST_BUILD_PATH="$BUILD" uv run --frozen --group native python -m pytest tests/reference/test_native_numeric_portability.py tests/reference/test_native_attachment_wrench.py -q
NATIVE_TEST_BUILD_PATH="$BUILD" bash tools/run_tests.sh native
```

Run the equivalent selected build on macOS and both Linux compilers, Python 3.13 and the recovered baseline interpreter, with the required BLAS thread configuration. Repeat fresh processes with NumPy imported before/after SciPy and MuJoCo to expose provider contamination. Record test count, seed, first mismatch (or zero mismatches), library fingerprints, CPU dispatch, and effective FP flags. Missing provider qualification is a blocker, not a skipped test. Commit only the verified lowering changes and their fixtures; an intermediate diagnostic-only commit may remain red but must be explicitly marked non-release-ready.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/numeric_portability_probe.py tests/reference/test_native_numeric_portability.py docs/agents/native-numerical-profiles.md native/src native/CMakeLists.txt
git commit -m "feat: qualify platform numerical lowerings without weakening oracles"
```

**Review gate:** Linux matches its local pinned reference at all existing exact boundaries; macOS retains its original backend/results. No broad skips, tolerances, CPU-feature suppression, changed production oracle, or implicit Python fallback conceals a mismatch.

### T12: Version numerical provenance and give replay precise incompatibility errors

**Files:** Create `src/bike_sim/native/execution_identity.py` and `tests/test_execution_identity.py`. Modify `src/bike_sim/native/artifact.py`, `native/src/runtime/provenance.cpp`, `native/CMakeLists.txt:942-977` (the existing inline `file(GENERATE)` that produces research_build_identity.hpp), `src/bike_sim/sim/research/replay.py`, `src/bike_sim/sim/research/replay_session.py`, and `src/bike_sim/native/research.py`. Extend existing recording/replay identity tests without changing recording table schemas.

**Interfaces:** Produce `validate_execution_identity(value: object) -> dict`, `execution_identity_differences(recorded: dict, current: dict) -> list[str]`, and `require_execution_compatible(recorded: dict, current: dict) -> None`. Keep public `artifact.validate_execution` as a delegating compatibility entry point. New execution top-level fields equal legacy EXECUTION_FIELDS plus integer `identity_schema=2`; `runtime_schema` stays 1. Native build_context retains its four strings and adds `platform` and `numerics`. New Python identities use build_context containing only `platform` and `numerics`; legacy Python build_context remains null.

**Reason and boundaries:** Current exact dictionary comparison neither explains numerical environment differences nor knows a BLAS provider changed. Adding unversioned fields would make old recordings unreadable. Build-tool context schema and persisted execution schema have different purposes and must not be conflated.

- [ ] **Step 1 - Add the regression and a valid control.**

Test the data contract without importing MuJoCo, NumPy, or a native binary:

```python
from copy import deepcopy
import pytest
from bike_sim.native.execution_identity import (
    execution_identity_differences, validate_execution_identity,
)

def legacy_python():
    return {"backend": "python", "runtime_schema": 1,
            "python_source_sha256": "a" * 64, "native_source_sha256": None,
            "extension_sha256": None, "mujoco_version": "3.12.0",
            "mujoco_library_sha256": None, "build_context": None}

def test_legacy_read_stays_valid():
    value = legacy_python()
    assert validate_execution_identity(value) == value
    assert "identity_schema" not in value

def test_unknown_identity_schema_is_not_guessed():
    value = legacy_python()
    value["identity_schema"] = 999
    with pytest.raises(ValueError, match="identity_schema"):
        validate_execution_identity(value)

def test_mismatch_names_the_field():
    left = legacy_python()
    right = deepcopy(left)
    right["mujoco_version"] = "3.12.1"
    differences = execution_identity_differences(left, right)
    assert any("mujoco_version" in item for item in differences)
```

Add complete version-2 Python/native fixtures using the schema below. Change one field at a time: OS, architecture, libc, Python version, NumPy version, provider library hash, integer ABI, threading count, CPU profile, compiler numerical flags and native source digest. Require the exact dotted key in the diagnostic. Require unknown/extra keys and booleans masquerading as integer schema versions to fail. Legacy read validation must never import packages or follow recorded paths.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/test_execution_identity.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Define the new JSON contract exactly, using finite scalar values and nonempty strings. No arbitrary metadata dictionaries enter equality comparison:

```text
platform = {
  system: "Darwin" | "Linux",
  machine: nonempty normalized architecture,
  libc: {name: nonempty string, version: nonempty string},
  os_build: nonempty string,
  python_version: nonempty string,
  python_soabi: nonempty string
}
numerics = {
  backend: "ACCELERATE" | "NUMPY_OPENBLAS",
  numpy_version: nonempty string,
  blas_integer_bits: 32 | 64,
  lapack_integer_bits: 64,
  cpu_profile: nonempty qualified profile identifier,
  threading: {implementation: nonempty string, num_threads: positive integer},
  libraries: [{role: "blas_lapack", identity_kind: "file" | "apple-system",
               sha256: lowercase 64-hex string | null,
               build_id: nonempty string | null}]
}
```

For Linux file-backed providers, identity_kind=file requires sha256 and null build_id. Hash the resolved actually loaded provider file and verify it corresponds to the configured target; do not hash only a symlink, SONAME string, SDK stub, or generic OpenBLAS config label. For Apple's system framework, an inaccessible dyld-cache image must not produce a fabricated file digest: identity_kind=apple-system requires null sha256 and a stable build_id formed from the actual macOS build identifier, actual Accelerate provider identity and selected SDK version. Collect these values at the runtime boundary; preserve a diagnostic path separately, outside persisted equality. Darwin libc is `{name: "libSystem", version: actual OS-build identifier}`; Linux libc comes from the actual environment. Use `sysconfig.get_config_var("SOABI")` for the interpreter ABI. Tooling-only hostnames, timestamps, roots, absolute paths and PIDs are excluded.

If the selected provider's effective thread count or CPU profile cannot be established, fail native doctor/startup with a specific unqualified-provider error. Do not mutate global thread settings during an active simulation. A deterministic startup threading policy, if necessary, is configured before imports and recorded; it is not secretly introduced by validation. Production and replay identity must describe the policy actually in use.

Split validation into legacy and v2 branches. Legacy has the exact existing field set and all current null/hash/type rules. V2 has the exact new field set. A Python v2 record keeps native_source_sha256, extension_sha256 and mujoco_library_sha256 null, but its platform/numerics context is no longer null. A native v2 record keeps all existing native hash requirements and its six-key build_context. Do not mutate a caller's record while validating.

Use a recursive, stable-order diff after validating both records:

```python
def _diff(left, right, prefix="execution"):
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            path = f"{prefix}.{key}"
            if key not in left or key not in right:
                result.append(f"{path}: field is present in only one identity")
            else:
                result.extend(_diff(left[key], right[key], path))
        return result
    if left != right:
        return [f"{prefix}: recorded={left!r}, current={right!r}"]
    return []
```

`execution_identity_differences` validates inputs and invokes this helper. `require_execution_compatible` raises ValueError with a stable `execution identity mismatch` prefix and the returned dotted fields. Handle library-list differences with indexed paths as an extension of the shown recursion, so a provider hash points to `execution.build_context.numerics.libraries[0].sha256` rather than dumping the entire context.

New writers emit v2. Reading old recordings remains allowed. A v2 runtime must not claim the unrecorded environment of a legacy file is equivalent: strict checked replay reports the missing identity_schema/context as an incompatibility before stepping. The old exact identity validator remains usable for older software/environments; data inspection and non-simulating playback do not require current execution equivalence. Document this distinction rather than silently projecting away new safety fields.

Extend the existing inline `file(GENERATE)` in native/CMakeLists.txt, preserving its per-configuration/target evaluation; no research_build_identity.hpp.in template exists in the snapshot. Capture compile-time source/provider-policy values in the generated research_build_identity.hpp, runtime-loaded MuJoCo identity as today, and runtime-selected NumPy/provider identity in Python at construction. Cross-check compile-time and runtime provider selections before native execution. Keep the existing source-stamp check and startup-versus-save identity check. `dladdr` use must retain the current safe data-address pattern; do not introduce non-ISO function-to-object casts merely to discover a numerical image. Use dependency-loader/provider inspection facilities outside hot loops for that image.

Replace raw execution dictionary comparisons at replay.py and replay_session.py with require_execution_compatible; keep the comparison before any physics advance. Recording validation delegates to the shared data-only schema validator. Update exact-field tests at every current validate_execution caller. Do not bump runtime_schema or the recording root version just to add identity metadata.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/test_execution_identity.py -q
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_replay_recording_v3.py tests/reference/test_native_replay_session.py tests/reference/test_native_research_recording.py -q
```

Use fixtures to prove read-only inspection of legacy and foreign-platform recordings succeeds without importing bike_native. Use a stepper spy to prove every identity mismatch occurs before step count changes. Same-environment v2 recording/checked replay must pass. Mutating a provider file, changing source after startup, importing an alternate extension, or replacing MuJoCo must preserve existing fail-closed behavior. Repeat on macOS and Linux; do not compare only mocked dictionaries.

- [ ] **Step 5 - Review and commit.**

```sh
git add src/bike_sim/native src/bike_sim/sim/research/replay.py src/bike_sim/sim/research/replay_session.py native/src/runtime/provenance.cpp native/cmake native/CMakeLists.txt tests/test_execution_identity.py tests/reference
git commit -m "feat: version numerical execution identity and replay diagnostics"
```

**Review gate:** Existing recordings remain readable; checked replay never silently accepts unknown numerical identity. New Python and native records describe the actual environment. Runtime/recording schema numbers and physics semantics are unchanged.
