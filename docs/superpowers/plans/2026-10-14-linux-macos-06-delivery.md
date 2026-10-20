# 06 - Source development, offline delivery, and documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Provide truthful Linux and macOS installation and development paths, including offline native builds.

**Architecture:** Retain the Python package and make native compilation an explicit source-tree operation. Package platform-matched dependency wheels plus source and manifests; build locally rather than copying a macOS artifact or claiming a pure Python wheel contains the extension.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.27, and strict ISO C++23 without compiler language extensions.
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

T23 follows T03/T10/T12. T24-T25 follow the locked dependency profile from T02 and all native build/toolchain discovery contracts. T26 documents the actually passing commands after the relevant tasks, not speculative instructions.

New tools/offline_bundle.py owns a versioned manifest and assembly. Existing shell entry points stay thin, compatible wrappers. Runtime artifact selection remains in src/bike_sim/native/artifact.py; package installation never starts a compiler during import.

### T23: Make source-first installation and native loader diagnostics explicit

**Files:** Modify `src/bike_sim/native/artifact.py`, `tools/dev.py`, `tests/reference/native_loader.py`, and existing native loader tests. Create `tests/portability/test_install_modes.py` and `docs/development/install-modes.md`. Keep `pyproject.toml` setuptools packaging and existing console scripts unless an actual entry-point defect is demonstrated.

**Interfaces:** Keep BIKE_NATIVE_BUILD_PATH and NATIVE_TEST_BUILD_PATH absolute-path contracts. Preserve load_native_extension(build_dir: Path | None = None) and no compilation on import. Source-first editable installs are the supported native delivery mode. A standalone py3-none-any wheel remains Python-only unless the caller explicitly provides a matching source/build installation under the documented native route.

**Reason and boundaries:** Extension suffix handling is already portable. The real risks are checkout-relative defaults after wheel installation, stale ABI/build selection, confusing linker errors, and project tools accidentally selecting a different uv environment.

- [ ] **Step 1 - Add the regression and a valid control.**

Add subprocess tests for: editable source install with selected build; Python-only wheel --help/reference path with no native binary; native request from that wheel without a configured source/build; missing binary; two matching suffix artifacts; relative selected path; wrong pre-imported module; stale compiled source stamp; wrong MuJoCo; and removed numerical provider file.

The no-implicit-build invariant can be tested without native compilation:

```python
import pytest
from bike_sim.native import artifact

def test_missing_artifact_never_launches_a_build(tmp_path, monkeypatch):
    import subprocess
    def forbidden(*args, **kwargs):
        raise AssertionError("build process launched by application import")
    monkeypatch.setattr(subprocess, "run", forbidden)
    with pytest.raises(RuntimeError, match="build"):
        artifact.load_native_extension(tmp_path.resolve())
```

Run each artifact-selection test in a fresh process where import caching could hide a conflict. A stale sys.modules entry is not fixed by silently unloading/reimporting a different extension.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_install_modes.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Keep importlib.machinery.EXTENSION_SUFFIXES; do not replace it with fixed `.so` or `.dylib` checks. Preserve exactly-one selected artifact, required dependency versions, imported path equality and compiled/native source identity. Improve exceptions with selected build, actual interpreter/SOABI, operating system/architecture and explicit configure/build remediation. Wrap ImportError/OSError only to add context while preserving the original cause and dynamic-loader diagnostic.

For a normal editable source environment, native source root remains the checkout containing native/CMakeLists.txt and pyproject.toml. For a wheel-only environment, do not guess another checkout by searching cwd or parent directories. State that the documented native route requires the bundled/real source tree and its own editable uv environment. The first implementation need not turn an arbitrary installed pure Python wheel into a native distribution.

Make tools/dev.py resolve its project root from its own source path, pass that root explicitly as cwd, and confirm the selected interpreter matches the one CMake recorded. For an installer-specified custom environment, consistently set UV_PROJECT_ENVIRONMENT to that absolute venv in every configure/build/test wrapper, or use the source project's default .venv. Do not activate one venv and let `uv run` silently create another. `doctor --build` checks Python executable/SOABI, source root/hash, selected artifact, MuJoCo private ABI, numerical profile and linked dependencies without compiling.

Keep editable installation via uv sync --frozen --group native. Application import only reads/validates and imports; bootstrap/configure/build are user-invoked steps. A copied/relocated build cache is invalid because its absolute dependencies belong to its original environment; diagnose and rebuild from source rather than adding an arbitrary LD_LIBRARY_PATH. No permissive native-to-Python fallback or implicit toolchain download.

Document three explicit modes: reference Python package, source/editable native development, and native-capable offline source bundle. State capabilities and prerequisites for each, particularly ordinary --record requiring native. Keep the Python-only wheel honest and leave a relocatable multi-platform native wheel migration outside this port.

At the existing importlib.import_module call in load_native_extension, retain all preceding dependency/selection checks and the following imported-path check; add contextual loader diagnostics without retrying or invoking a build:

```python
import platform
import sysconfig

try:
    module = importlib.import_module(EXTENSION_MODULE)
except (ImportError, OSError) as error:
    raise RuntimeError(
        f"native load failed: selected={build}, artifact={expected}, "
        f"python={sys.executable}, soabi={sysconfig.get_config_var('SOABI')}, "
        f"platform={platform.system()}/{platform.machine()}; "
        f"run tools/dev.py doctor --build with the selected absolute path, "
        f"then explicitly configure/build in the same source environment; "
        f"loader error: {error}"
    ) from error
```

Place the two added imports with the module imports. The names build, expected and EXTENSION_MODULE are the current function's validated variables. Preserve the original exception as the cause so missing SONAME/symbol diagnostics remain inspectable.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv sync --frozen --group native
uv run --frozen --group native --no-sync python tools/dev.py configure --build native/build/release --config Release
uv run --frozen --group native --no-sync python tools/dev.py build --build native/build/release
uv run --frozen --group native --no-sync python tools/dev.py doctor --build native/build/release --numerics
uv run --frozen --group native python -m pytest tests/portability/test_install_modes.py -q
```

Use a fresh temporary environment to install the existing pure Python wheel and exercise its documented Python-only mode. Separately test a moved source tree rebuilt in place. Test a copied build/venv and require a clear stale-environment rejection instead of accepting it accidentally. Repeat loader/import-order controls on both operating systems.

- [ ] **Step 5 - Review and commit.**

```sh
git add src/bike_sim/native/artifact.py tools/dev.py tests/reference/native_loader.py tests/portability/test_install_modes.py docs/development/install-modes.md
git commit -m "feat: document and validate explicit source-native install modes"
```

**Review gate:** Native selection stays strict and non-building; source/editable use works on both hosts; wheel-only limitations and stale ABI/library errors are accurate and actionable.

### T24: Build manifest-driven offline source bundles for each qualified target

**Files:** Create `tools/offline_bundle.py` and `tests/portability/test_offline_bundle.py`. Modify `tools/make_offline_bundle.sh`, `tools/check_offline_sources.py`, `packaging/OFFLINE_README_full.md`, and `packaging/OFFLINE_README_slim.md`. Preserve the existing full/slim bundle concepts and original Linux cp313 bundle naming when using the legacy defaults.

**Interfaces:** Produce `validate_bundle_manifest(value: object) -> dict`, `include_source_path(path: PurePosixPath) -> bool`, and CLI `python tools/offline_bundle.py --target host|linux-x86_64|macos-arm64 --python MAJOR.MINOR --mode full|slim --native --output PATH`. An explicitly requested native target must match the build host for this iteration. Existing no-argument shell invocation retains its documented legacy Linux full+slim behavior; new native-capable bundles require the explicit --native path and are labeled accordingly.

**Reason and boundaries:** The current bundler downloads Python wheels but neither compiles bike_native nor excludes native/build. A source bundle must not accidentally carry a Mac binary/cache to Linux or claim native capability from a py3-none-any wheel.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from pathlib import PurePosixPath
from tools.offline_bundle import include_source_path

def test_native_build_and_virtualenv_are_never_source_payloads():
    for value in ("native/build/release/bike_native.so",
                  "native/build/asan/CMakeCache.txt", ".venv/bin/python",
                  ".git/config", "output/run/run.duckdb"):
        assert not include_source_path(PurePosixPath(value))
    for value in ("native/CMakeLists.txt", "native/src/cblas_abi.hpp",
                  "native/cmake/BikeNumerics.cmake", "src/bike_sim/__init__.py",
                  "examples/research/viewer_physics_welded.toml", "uv.lock"):
        assert include_source_path(PurePosixPath(value))
```

Add negative manifest tests: wrong OS/architecture/Python minor, native=true with missing native source, missing lock, omitted nanobind/development closure, duplicate wheel names, mismatched hash, unsafe archive path and a pure Python bundle pretending to contain an extension. Add a fixture with a Mac dylib in native/build and prove it is absent from a Linux source payload.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_offline_bundle.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Use one explicit manifest contract:

```text
schema_version: 1
project: {name, version, git_commit, dirty, python_source_sha256, native_source_sha256}
target: {system, machine, python_minor, implementation, accepted_wheel_tags, libc_floor}
mode: full | slim
capabilities: {python: true, native_source_build: bool, development: bool, gui: bool}
requirements: {lock_sha256, requirements_sha256, groups: [string]}
wheels: [{filename, distribution, version, sha256, tags: [string]}]
source_files: [{path, sha256}]
prerequisites: {python, compiler_feature_profile, cmake_minimum, uv_version,
                system_libraries: [string]}
slim_external: [{distribution, version, qualification_profile}]
```

Use an actual host-derived wheel-tag set and dependency marker evaluation for native-capable bundles. Cross-platform wheel downloading may remain for the explicitly Python-only legacy route, but must not be presented as a tested native bundle. Reject native target/host mismatch. Keep exact target/compiler/system prerequisites in the generated manifest and documentation.

Assemble source from the real checkout's tracked files plus explicitly approved required assets, preserving current source content and recording dirty state. Exclude .git, all virtualenvs, native/build, .work, output, dist artifacts, caches and generated binaries. Detect arbitrary build trees by their known build roots/manifest, not by dropping every directory named tests or every legitimate fixture. Resolve every source path beneath the project root; reject escaping symlinks/traversal. Do not assume the Repomix export supplies executable bits or binary assets.

Export the frozen full native/development closure, including dev and native groups and build-system wheels needed for offline editable installation. The native bundle must include nanobind, exact MuJoCo, scientific packages and the tools required by its declared capability. Include the qualified uv executable/wheel or clearly require that exact tool to be preinstalled; in the chosen implementation bundle it as a platform-matched tool wheel and verify its hash/license. CPython, system compiler and OS graphics libraries remain explicit pre-provisioned prerequisites, not hidden network downloads during installation. Include setuptools/wheel and any actual PEP 517 build backend requirements used by uv build/sync.

A concrete online assembly sequence is:

```sh
uv export --frozen --group native --group dev --no-emit-project -o .work/offline-build/requirements.txt
uv run --frozen --group native --with pip python -m pip download --only-binary=:all: -r .work/offline-build/requirements.txt --dest .work/offline-build/wheelhouse
uv build --out-dir .work/offline-build/dist
```

The assembler creates the work directories first, captures all commands/statuses, adds the project wheel only for the documented Python mode/compatibility path, adds the pinned build/tool wheels, and rejects any unresolved source-only dependency. The selected real lock, not this plan, supplies exact transitive versions and hashes. Verify all downloaded filenames/tags against the target and manifest, not just the top-level NumPy/MuJoCo files.

Native-capable source bundles use source/editable installation and a local native build at the target. Do not place a host-built extension in the source payload. Extend check_offline_sources.py to compare native/src, native/cmake, CMakeLists and required assets against source hashes as well as its current Python wheel/source comparison. Explain that matching a Python wheel alone does not certify native source completeness.

Full includes the complete closure. Slim omits only explicitly enumerated external packages/closure and declares exact required versions/profile; do not infer safe omissions from a static list of names without checking dependency edges. A slim bundle remains native-capable only after target validation in T25. Archive names encode target/Python/mode/native-source capability; retain compatibility aliases only when their manifest truthfully describes the old mode.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python tools/offline_bundle.py --target host --python 3.13 --mode full --native --output output/offline-bundles
uv run --frozen --group native python tools/offline_bundle.py --target host --python 3.13 --mode slim --native --output output/offline-bundles
uv run --frozen --group native python -m pytest tests/portability/test_offline_bundle.py -q
```

Run on Linux and macOS target hosts with their qualified Python/native profiles. List every archive member, check hashes/tags and prove there are no build caches, virtualenv binaries, unrelated credentials or generated recordings. A manifest-complete archive is not yet installation-qualified; T25 is its release gate.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/offline_bundle.py tools/make_offline_bundle.sh tools/check_offline_sources.py tests/portability/test_offline_bundle.py packaging/OFFLINE_README_full.md packaging/OFFLINE_README_slim.md
git commit -m "feat: assemble platform-manifested native source bundles"
```

**Review gate:** Every bundle states its true capabilities and platform/ABI; native source is complete, dependency closure is frozen, and host artifacts cannot leak into another platform package.

### T25: Install full and slim bundles offline with local native compilation

**Files:** Modify `packaging/install_full.sh`, `packaging/install_slim.sh`, `tools/install_offline.py`, `tools/install_offline.sh`, and `tools/dev.py` offline bootstrap path. Create `tests/portability/test_offline_install.py` and `tests/portability/offline_install_probe.py`. Keep legacy manifest-less Python-only bundle installation behavior isolated from the new schema-1 native bundle path.

**Interfaces:** Consume T24 manifest and wheel hashes; keep existing optional install_dir arguments. A native install creates/uses the source project environment, syncs frozen native+dev groups offline, configures/builds locally and runs doctor. Full/slim share manifest validation and error reporting. Unknown target, hash mismatch, missing compiler/wheel or unqualified external scientific stack fails before claiming success.

**Reason and boundaries:** Existing full/slim installers create Python environments only, and the alternate slim route checks broad system packages rather than the actual native numerical profile. Offline must mean no network or warm-cache dependency, not merely passing --no-index to one pip command.

- [ ] **Step 1 - Add the regression and a valid control.**

Use a minimal fixture bundle and assert preflight failures occur before environment mutation: wrong Python minor, wrong OS/arch, corrupted wheel, missing native source, unavailable compiler and mismatched lock. For slim, test SciPy 1.17.1 against required 1.17.0 and require rejection; the current major/minor-only comparison is insufficient.

```python
import pytest
from tools.offline_bundle import validate_bundle_manifest

def test_wrong_manifest_schema_is_not_legacy_native():
    with pytest.raises(ValueError, match="schema"):
        validate_bundle_manifest({"schema_version": 2,
                                  "capabilities": {"native_source_build": True}})
```

Add end-to-end probes that install into a path with spaces, use a custom venv directory, remove the builder's cache, relocate the extracted source before building and run a selected native import/CTest/recording smoke. The probe must assert the final imported extension lives in the target's newly built directory.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_offline_install.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Validate the manifest, all payload hashes, target tags, Python interpreter and compiler prerequisites before installation. Refuse root/system Python mutation. Create a venv only at the explicit/default target path; never delete an existing arbitrary directory to recover from failure. Use the bundled local uv tool or a verified preinstalled equivalent to manage the source project environment. For custom environments set UV_PROJECT_ENVIRONMENT consistently.

For the new native-capable path, sync inside the bundled source tree with offline/network-disabled settings and the bundled wheelhouse:

```sh
export UV_OFFLINE=1
export UV_PROJECT_ENVIRONMENT="$VENV_DIR"
uv sync --offline --no-index --frozen --group native --group dev --find-links "$BUNDLE_DIR/wheelhouse"
uv run --offline --frozen --group native python tools/dev.py configure --build native/build/release --config Release
uv run --offline --frozen --group native python tools/dev.py build --build native/build/release
uv run --offline --frozen --group native python tools/dev.py doctor --build native/build/release --numerics
```

The wrapper resolves BUNDLE_DIR, VENV_DIR and uv from validated local paths, changes into `$BUNDLE_DIR/project`, and does not rely on an inherited active environment. PEP 517/editable build isolation must also resolve entirely from the local wheelhouse; include its build requirements in T24. Missing local build requirements fail rather than falling back to network. After success, print/source a generated activation wrapper that exports the exact environment and selected build path; normal shell activation alone must not let uv select a different project environment.

For slim, verify every declared external package version and its actual import location before creating the system-site-packages environment. Validate NumPy/MuJoCo/provider ABI and qualified numerical profile after environment creation, because import precedence can change. Use exact required SciPy/MuJoCo versions, not >= or major/minor-only checks. External packages that fail the profile are a clear slim rejection with full-bundle remediation. Do not overwrite the system scientific stack. If uv would shadow an external package, either intentionally include/install its pinned wheel and reclassify the bundle or fail; never claim it reused the validated system provider while loading a different one.

Unify tools/install_offline.py and both shell installers through manifest preflight while retaining their documented legacy Python-only path. Remove references to absent build_offline_wheel.sh/refresh_offline_wheels.py and point to the actual assembler. Preserve existing wheel hash checks and source comparison. GUI capability still depends on preinstalled OS graphics libraries and a display; headless native installation must not need them to create a window.

Run the end-to-end qualification in a fresh dedicated target environment with empty uv/pip caches and network disabled by the test harness. Linux may use a container/VM with network disabled; macOS requires a dedicated VM/runner or equivalent network-blocked harness. A skipped namespace/network isolation check is not a successful offline certification. Record the isolation mechanism, attempted network accesses, cache directories and installer logs. A native build failure leaves diagnostics and never prints a success banner.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_offline_install.py -q
uv run --frozen --group native python tests/portability/offline_install_probe.py --bundle output/offline-bundles --mode full
uv run --frozen --group native python tests/portability/offline_install_probe.py --bundle output/offline-bundles --mode slim
```

The probe selects exactly one manifest matching its current host/Python/mode and fails on zero/ambiguous matches. Execute it in the network-blocked fresh harness, not merely the online authoring environment. Require source/editable import, local native build, doctor, CTest, a valid short native fixture and opt-in final recording. Re-run after relocation/rebuild; do not accept a copied old native cache. Full and slim each need Linux and macOS evidence or an explicit unqualified status, not a shared success label.

- [ ] **Step 5 - Review and commit.**

```sh
git add packaging/install_full.sh packaging/install_slim.sh tools/install_offline.py tools/install_offline.sh tools/dev.py tests/portability/test_offline_install.py tests/portability/offline_install_probe.py
git commit -m "feat: qualify full and slim offline native source installation"
```

**Review gate:** Fresh network-blocked installation compiles and runs the target native artifact without hidden caches/downloads. Slim validates the exact imported scientific provider. Legacy Python-only bundles remain honestly labeled.

### T26: Document supported workflows and remove machine-specific development references

**Files:** Modify `README.md`, `AGENTS.md`, `docs/TESTING.md`, `docs/RIDE.md`, `docs/RECORDING.md`, `docs/agents/native-engineering.md`, `native/.clangd`, and packaging offline READMEs. Create `docs/development/linux.md`, `docs/development/macos.md`, `docs/development/troubleshooting.md`, and `tests/portability/test_development_docs.py`. Preserve historical audit statements with their dates.

**Interfaces:** Document tools/dev.py, selected build environment variables, tool overrides, supported platform matrix, optional versus required capabilities, native/Python install modes, offline prerequisites and replay identity behavior. Replace hard-coded developer-specific /Users reference-source locations with NUMPY_SOURCE_ROOT and MUJOCO_SOURCE_ROOT or a documented `.work/reference-sources/` convention; never require those sources for ordinary runtime import.

**Reason and boundaries:** The repository has detailed native safety conventions worth retaining, but some references assume one developer filesystem and historical macOS-only status. Documentation must distinguish newly measured support from historical evidence and unsupported configurations.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a text-level test that active development instructions do not require the original personal /Users path, Linux instructions contain the native group and no mandatory xcrun, and macOS instructions still mention mjpython/SDK overrides. Do not strip personal paths from historical logs merely to make a text search pass; scope the assertion to active instruction sections.

```python
from pathlib import Path

def test_active_linux_guide_has_native_workflow():
    text = Path("docs/development/linux.md").read_text()
    assert "tools/dev.py" in text
    assert "--group native" in text
    assert "BIKE_NATIVE_BUILD_PATH" in text
    assert "CMAKE_OSX_SYSROOT" in text  # documented as invalid on Linux
```

Use a clean-environment rehearsal, not only a text test, to validate every documented bootstrap/configure/build/test command. Check local Markdown links and all referenced file paths.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_development_docs.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Write two platform guides with the same workflow headings: prerequisites, frozen environment, configure/build Debug and Release, doctor, Python/native/headless/GUI run, tests/analysis, sanitizer/coverage, editor setup, offline installation and failure diagnosis. Record the actual passing compiler/stdlib/SDK/uv versions from qualification rather than asserting that the OS default compiler is sufficient.

A Linux bootstrap may use the distribution's build-essential/CMake/Ninja/pkg-config/GLFW/OpenGL packages as initial candidates, but the C++23 and sanitizer capability probes decide qualification. CPython 3.13 is an explicit interpreter requirement, not the unqualified meaning of `python3` on Ubuntu 24.04. Provision it with the chosen uv-managed or already installed interpreter before offline use. Document a separately qualified LLVM toolchain for required RTSan where the default distribution toolchain lacks it; do not silently skip that lane.

macOS keeps Xcode/SDK selection, explicit CMAKE_OSX_SYSROOT behavior, Accelerate and mjpython. Preserve working Homebrew Intel/Apple Silicon discovery where appropriate without embedding one fixed prefix. Document that Linux doesn't run these Apple steps and rejects Apple-only options.

Use executable commands already defined by this plan:

```sh
uv sync --frozen --group native
uv run --frozen --group native --no-sync python tools/dev.py configure --build native/build/release --config Release
uv run --frozen --group native --no-sync python tools/dev.py build --build native/build/release
uv run --frozen --group native --no-sync python tools/dev.py doctor --build native/build/release --numerics
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" bash tools/run_tests.sh native
```

Explain quick/native/full selection, nonzero native test counts, required external analysis tools, and separate build directories for compiler/configuration/sanitizer/coverage. Editor configuration must point at the selected compile_commands.json and qualified compiler; do not make one user's absolute build path shared config.

Troubleshooting is symptom -> evidence command -> bounded remediation: Apple SDK error on Linux; missing MuJoCo3.12 private symbol; NumPy provider/ABI mismatch; missing native artifact; wrong Python SOABI; stale source stamp; unresolved DSO; missing display; unsupported sanitizer; mismatched coverage tools; strict invalid-controller stop; cross-environment checked replay rejection; slim package mismatch. Never recommend arbitrary library symlinks, broad LD_LIBRARY_PATH, disabled tests, changed physics tolerance or deleting lockfiles.

Keep dated macOS performance/failure history in docs/RIDE.md and add a current platform support section linking the new evidence. Recover referenced safety documents per T01; do not erase links simply because Repomix omitted their targets. Record external reference-source version/hash and license when fetching it for numerical port work; do not assume the original developer home directory exists on an agent host.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_development_docs.py -q
git diff --check
```

Have a fresh Linux checkout and a fresh macOS checkout follow the active guide without private paths or pre-existing build caches. Record the commands and actual status. Test all documented local links. Ensure active support claims link to measured release evidence; preserve historical results explicitly as historical.

- [ ] **Step 5 - Review and commit.**

```sh
git add README.md AGENTS.md docs/TESTING.md docs/RIDE.md docs/RECORDING.md docs/agents/native-engineering.md docs/development native/.clangd packaging/OFFLINE_README_full.md packaging/OFFLINE_README_slim.md tests/portability/test_development_docs.py
git commit -m "docs: add verified Linux and macOS development workflows"
```

**Review gate:** A new developer can follow either platform guide without private filesystem assumptions. macOS remains first-class, native prerequisites are explicit, and unsupported/unverified capabilities are not advertised as passing.
