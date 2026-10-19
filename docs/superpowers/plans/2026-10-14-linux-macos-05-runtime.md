# 05 - Runtime, viewers, workers, recordings, and replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Exercise real Linux application workflows while preserving existing macOS behavior.

**Architecture:** Keep platform decisions at launch/resource boundaries. Reuse the shared runtime and recording lifecycle; verify headless, GUI, worker, inspection and checked-replay paths independently instead of treating one successful import as application support.

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

T18-T20 tooling-only tests may begin after T03. Their native integration gates require T10-T12 and T13-T16 as applicable. T21-T22 require the finalized execution-identity schema from T12.

Existing ownership stays intact: sim/playground.py owns the Darwin launcher; cli/batch.py chooses worker counts; sim/ride/async_close.py owns asynchronous publication; recording/ and research/ lifecycle code own persistence. Do not create a new Linux simulator or duplicate the recording engine.

### T18: Prove headless operation does not require a display or initialize rendering

**Files:** Create `tests/portability/test_headless_imports.py` and `tests/reference/test_native_linux_headless.py`. Modify only offending eager imports/calls in `src/bike_sim/cli/ride.py`, `src/bike_sim/cli/research.py`, `src/bike_sim/sim/ride/physical_driver.py`, `src/bike_sim/sim/research/configuration.py`, and viewer modules if the new tests demonstrate a rendering side effect.

**Interfaces:** Keep all existing CLI arguments, backend defaults and environment constructors. Headless construction/stepping/recording must not call glfw.init, glfw.create_window, mujoco.viewer.launch_passive or mujoco.Renderer. Importing MuJoCo itself is permitted; requiring a window/context is not. No new default MUJOCO_GL value is introduced.

**Reason and boundaries:** Linux servers and CI often have no display. A GLFW import is not the same as context initialization, and offscreen EGL/OSMesa setup is unnecessary for non-rendering physics. Headless support must include actual steps, not only --help.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a subprocess import/help test with a stripped display environment:

```python
import os
import subprocess
import sys
import pytest

@pytest.mark.parametrize("module", ["bike_sim.cli.ride", "bike_sim.cli.research",
                                    "bike_sim.cli.batch", "bike_sim.cli.replay",
                                    "bike_sim.cli.recording"])
def test_cli_help_without_display(module):
    env = dict(os.environ)
    for name in ("DISPLAY", "WAYLAND_DISPLAY", "MUJOCO_GL"):
        env.pop(name, None)
    result = subprocess.run([sys.executable, "-m", module, "--help"],
                            env=env, capture_output=True, text=True,
                            timeout=30, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
```

Add a native test using the existing environment_factory fixture and a PhysicalRideDriver. Monkeypatch glfw.init, glfw.create_window and mujoco.Renderer to raise AssertionError("rendering used in headless mode"); advance the fixture's documented four-step duration case, inspect a snapshot and close in finally. Test Python and native backends separately. Do not assert that glfw never appears in sys.modules, since MuJoCo can import its bindings without opening a window.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_headless_imports.py tests/reference/test_native_linux_headless.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Move any demonstrated rendering initialization inside existing interactive launch functions. Keep model/data construction and native runtime ownership in their current non-visual modules. Do not set MUJOCO_GL=egl globally, create a hidden window to placate a test, or import mujoco.viewer from the ordinary headless entry point.

Use the existing driver contract as the integration core:

```python
from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
from bike_sim.sim.ride.control import RideControl

def exercise_driver(environment_factory, backend):
    driver = PhysicalRideDriver(environment_factory().sim, backend=backend,
                                duration_s=.005, decimation=2)
    try:
        result = driver.advance(100, RideControl(0.))
        assert result.step == 4
        assert result.outcome == "duration_reached"
        assert driver.snapshot().integration_state.size > 0
    finally:
        driver.close()
```

This is a known fixture-level valid control, not a claim that the ordinary strict CLI scenario always reaches its requested duration. Add actual CLI subprocess cases for the documented welded config and track, no plots, selected native build and a short duration. Parse the real exit code/outcome/first failure and compare to that scenario's baseline; never accept every nonzero exit merely because it is headless. T21 covers --record publication, while this task verifies no recording output is created without opt-in.

Interactive-only arguments retain their current errors in headless mode, including --time-scale even when explicitly set to 1. Resource cleanup must execute after model construction failure, runtime error, KeyboardInterrupt and normal completion without initializing a viewer during cleanup.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python -m pytest tests/portability/test_headless_imports.py tests/reference/test_native_linux_headless.py -q
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --headless --duration 0.04 --no-plots
```

Run the command in a process with DISPLAY/WAYLAND_DISPLAY/MUJOCO_GL absent; retain its actual outcome and exit status, including a known strict validity stop. Run the same test on macOS without launching mjpython. An Xvfb-enabled test is not proof of this no-display gate.

- [ ] **Step 5 - Review and commit.**

```sh
git add tests/portability/test_headless_imports.py tests/reference/test_native_linux_headless.py src/bike_sim/cli src/bike_sim/sim
git commit -m "test: qualify display-free Python and native runtime paths"
```

**Review gate:** Both backends perform actual valid fixture steps without rendering initialization. Strict invalid scenarios remain invalid and are reported accurately; no offscreen backend is made mandatory.

### T19: Preserve mjpython and qualify real Linux desktop viewers

**Files:** Create `tests/portability/test_viewer_launch_platform.py` and `docs/development/viewer-platform-acceptance.md`. Modify `src/bike_sim/sim/playground.py::ensure_macos_mjpython`, `src/bike_sim/sim/ride/viewer.py`, `src/bike_sim/sim/ride/playback_viewer.py`, `src/bike_sim/sim/research/viewer.py`, `src/bike_sim/sim/research/replay_viewer.py`, and CLI viewer branches only where a failing launch/cleanup test requires a change.

**Interfaces:** Keep ensure_macos_mjpython() -> None and all existing call sites. Darwin preserves one-time re-exec, MUJOCO_MJPYTHON and DYLD_FALLBACK_LIBRARY_PATH handling. Linux returns without mjpython lookup/re-exec. Viewers retain independent render model/data, owning snapshots, key controls and requested/achieved time-scale separation.

**Reason and boundaries:** The existing launcher is already platform-guarded and is essential on macOS. Linux window qualification is a real desktop test, not a reason to remove the launcher or change physics pacing.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from bike_sim.sim import playground

def test_linux_never_reexecutes_mjpython(monkeypatch):
    monkeypatch.setattr(playground.sys, "platform", "linux")
    def forbidden(*args, **kwargs):
        raise AssertionError("mjpython exec on Linux")
    monkeypatch.setattr(playground.os, "execv", forbidden)
    playground.ensure_macos_mjpython()
```

Add Darwin tests with sys._mjpython_checked reset via monkeypatch, mocked shutil.which and os.execv. Require the exact original argv order, one MUJOCO_MJPYTHON marker, preservation/prepending of an existing DYLD_FALLBACK_LIBRARY_PATH when the base interpreter dylib exists, and no recursive relaunch after the marker. Test a missing mjpython command using the current error path; any improved message must explain the required launcher without silently switching renderers.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_viewer_launch_platform.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Retain the existing Darwin implementation unless tests identify a portability regression. Limit Linux changes to initialization/error reporting and platform-appropriate launcher logic. A viewer availability check must attempt the relevant library/context setup and capture the actual error; DISPLAY presence alone is not proof that the server or OpenGL context works.

For Linux X11 document the installed GLFW/OpenGL/Mesa or proprietary driver stack and display transport actually tested. For Wayland qualify the existing GLFW viewer via XWayland as a separate row; do not claim native Wayland support from that result. Display library installation belongs in environment documentation, not runtime shell commands or automatic apt/brew execution.

Use the current physical/research/replay render replica and snapshot ownership. Applying a visual frame must not call mj_step/mj_forward, acquire new sensor samples, change mutable model accounting or alter the integration timestep. Preserve main-thread viewer ownership, orderly close, repeated open/close, and native error propagation without aborting the host.

Fill viewer-platform-acceptance.md with one record per real host/session: OS/build, desktop/session transport, GPU/driver, interpreter/launcher, selected native artifact, exact command, controls exercised, requested scale, measured achieved RTF, outcome, recording location when opted in, and logs/screenshots. Do not mark visual quality passed based only on a window handle.

Exercise physical viewer, research viewer, checked replay --viewer, pure playback --view and the playground. For physical/research: keyboard camera/mode controls, scale 1 and 8, reset, terminal frame hold, close after terminal outcome and Ctrl-C. With recording on, prove reset-separated episodes remain in RAM until close, as in T21. With recording off, no persistence. A headless session is not a replacement for this gate.

Preserve this opening guard verbatim in ensure_macos_mjpython; all existing Darwin re-exec code remains below it. The function must return on Linux before setting the one-time sentinel or inspecting launcher paths:

```python
if sys.platform != "darwin":
    return
if getattr(sys, "_mjpython_checked", False):
    return
sys._mjpython_checked = True
```

This is the opening body of the existing function, not a new top-level module block. Retain its remaining Darwin code and verify the no-re-exec regression plus actual macOS launch.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 8
uv run --frozen --group native python -m pytest tests/portability/test_viewer_launch_platform.py -q
```

Perform separate real macOS and Linux desktop sessions. macOS must still show the required mjpython launch behavior. Use the actual finalized run directory from T21 for replay commands in T22. Do not infer 8x achieved throughput from requesting 8x, or alter dt to reach a frame-rate target.

- [ ] **Step 5 - Review and commit.**

```sh
git add tests/portability/test_viewer_launch_platform.py docs/development/viewer-platform-acceptance.md src/bike_sim/sim/playground.py src/bike_sim/sim/ride/viewer.py src/bike_sim/sim/ride/playback_viewer.py src/bike_sim/sim/research/viewer.py src/bike_sim/sim/research/replay_viewer.py src/bike_sim/cli
git commit -m "test: preserve macOS launcher and qualify Linux viewers"
```

**Review gate:** The existing macOS launch and visualization behavior remains functional; Linux X11 and separately tested XWayland paths have real evidence. Pacing changes never modify simulation time integration.

### T20: Make worker selection portable and qualify asynchronous process startup

**Files:** Modify `src/bike_sim/cli/batch.py`, `src/bike_sim/sim/ride/async_close.py`, and affected CLI module guards only if needed. Create `tests/portability/test_worker_platform.py`; extend `tests/test_async_close.py` and existing batch tests. Add `tests/portability/worker_entry_probe.py` as a guarded subprocess helper, not a production entry point.

**Interfaces:** Keep _performance_cores() -> int. Add private `_worker_start_method(system: str) -> str` returning `spawn` for Linux and the retained baseline method for Darwin until its separately qualified migration. AsyncCloseRunner accepts keyword-only `start_method: str | None = None` for explicit tests; no public CLI default changes other than the Linux path. Preserve queue protocol, sequence numbering, publication-error behavior and close semantics.

**Reason and boundaries:** Apple sysctl should not run on Linux. The async worker currently explicitly forks after scientific imports; inheriting thread/BLAS locks is a Linux safety risk even though the child uses only NumPy. Batch already uses spawn and should retain it.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from bike_sim.cli import batch

def test_linux_cpu_count_does_not_call_apple_sysctl(monkeypatch):
    monkeypatch.setattr(batch.sys, "platform", "linux")
    monkeypatch.setattr(batch.os, "process_cpu_count", lambda: 3)
    monkeypatch.setattr(batch.os, "cpu_count", lambda: 64)
    def forbidden(*args, **kwargs):
        raise AssertionError("Apple sysctl queried on Linux")
    monkeypatch.setattr(batch.subprocess, "run", forbidden)
    assert batch._performance_cores() == 3
```

Add Darwin successful sysctl, malformed output, timeout, unavailable command, process_cpu_count=None and cpu_count=None tests; the final fallback is 1. Parametrize the existing async synthetic-context/RawStep roundtrip and publication-error tests with the Linux spawn method. Run a guarded subprocess that performs NumPy BLAS work in a background thread before starting a worker; enforce a finite timeout and require orderly completion, not a hang or resource-tracker warning.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_worker_platform.py tests/test_async_close.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Guard the existing query and preserve affinity-aware fallback:

```python
def _performance_cores():
    if sys.platform == "darwin":
        try:
            result = subprocess.run(["sysctl", "-n", "hw.perflevel0.physicalcpu"],
                                    capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return max(1, int(result.stdout.strip()))
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
    return os.process_cpu_count() or os.cpu_count() or 1
```

Update batch --jobs help to describe Apple performance cores and process-available CPUs elsewhere. Retain explicit spawn for batch, serial mode, per-job output isolation and Ctrl-C result flushing. Do not overwrite a user's explicit --jobs or set BLAS thread counts after workers have started. Include selected worker count/thread policy in diagnostic output to identify oversubscription.

For AsyncCloseRunner, keep `_close_worker_main` at module scope and create Queue/Process from one selected context. The Linux default is spawn, selected by `_worker_start_method("Linux")`; an explicitly supplied method is validated against multiprocessing.get_all_start_methods(). Preserve the existing Darwin method for this additive port and remove the blanket comment claiming NumPy-only fork is universally safe. A Darwin method migration needs the same real launcher tests before changing its default.

Prove module safety with actual `uv run bike-research`, `python -m bike_sim.cli.research`, `uv run bike-ride`, and the macOS mjpython route, rather than only pytest. Both supplied CLI modules already have __main__ guards: preserve them. Inspect the actual generated console script in the selected environment. A wrapper that re-executes application main during spawn must be fixed through a guarded launcher/bootstrap path and tested before the Linux async feature is released; it must not trigger a silent fallback to fork. No replacement process protocol may be introduced without retaining the current explicit serialization-failure and sequence-order tests.

Add tests for synchronous-versus-async equivalence only where no policy consumes delayed publication; retain documented one-period observation latency elsewhere. Exercise context reset, multiple queued periods, malformed payload, worker-side exception, serialization failure before enqueue, out-of-order result, worker death, close with pending work, double close, repeated startup/shutdown and Ctrl-C. Keep native model/data objects out of worker payloads. Require bounded joins/cleanup and no orphan processes. Preserve the current recovery/republication behavior exactly and prove an error is not counted twice.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_worker_platform.py tests/test_async_close.py -q
uv run --frozen --group native python tests/portability/worker_entry_probe.py
```

The helper must contain `if __name__ == "__main__":` and exit nonzero on timeout, queue protocol mismatch or leaked child. Run under Linux with a warmed threaded BLAS and under the recovered macOS environment. Compare a fixed small batch in --serial and two-worker mode by job results, not completion order. Native async integration follows the existing native capability limits; do not claim arbitrary policies remain bitwise identical under delayed observations.

- [ ] **Step 5 - Review and commit.**

```sh
git add src/bike_sim/cli/batch.py src/bike_sim/sim/ride/async_close.py tests/portability/test_worker_platform.py tests/portability/worker_entry_probe.py tests/test_async_close.py
git commit -m "fix: qualify platform-safe batch and async worker startup"
```

**Review gate:** Linux never executes the Apple CPU query or silently falls back to unsafe fork. Actual entry points start/stop workers correctly. Existing macOS behavior and asynchronous publication semantics remain covered.

### T21: Qualify final-only recording and error lifecycle on both platforms

**Files:** Extend `tests/reference/test_native_recording_lifecycle.py`, `tests/reference/test_native_recording_retention.py`, `tests/reference/test_native_encode_columns.py`, `tests/reference/test_native_research_recording.py`, `tests/reference/test_native_ride_frontend.py`, and `tests/reference/test_native_research_frontend.py`. Create `tests/reference/test_native_platform_recording.py`. Modify `src/bike_sim/recording/`, `src/bike_sim/sim/research/recording_lifecycle.py`, `src/bike_sim/recording/capture.py` only where a reproduced OS-specific failure requires it; physical-ride recording uses this capture path and its existing driver/viewer glue, not a sim/ride/recording_lifecycle.py module.

**Interfaces:** Keep --record as the sole opt-in, --out as location only, native requirement for ordinary new recordings, exclusive final directory, run.duckdb/manifest.json/summary.md publication and existing typed source ownership. New execution identity is T12 v2. No schema-table, physics or recording-retention policy change is authorized.

**Reason and boundaries:** A port is incomplete if stepping works but recording loses data during close, reset or interruption. Existing lifecycle tests are unusually strict and must remain the reference, including known invalid-controller semantics.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a platform integration test using the real native recording fixture path from the existing lifecycle suite, not a hand-built fake database. Require this sequence:

```python
from pathlib import Path

def assert_published_run(run_directory):
    root = Path(run_directory)
    assert (root / "run.duckdb").is_file()
    assert (root / "manifest.json").is_file()
    assert (root / "summary.md").is_file()
```

Before launch close, assert the exclusive final directory and its public files do not exist. Reset twice, prove sealed sources remain owned/decodable, then finalize once and assert all episode IDs appear. Repeat with recording disabled and --out supplied: no run files. Reuse the existing monkeypatched lifecycle controls for Ctrl-C during seal/finalize, double Ctrl-C, simulation error, publication failure and retry; add real filesystem integration in a path containing spaces and non-ASCII characters on Linux/macOS.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_platform_recording.py tests/reference/test_native_recording_lifecycle.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Keep the current in-memory interval capture and finalization ordering. Fix only demonstrated platform problems: exact-case asset paths, descriptor lifetime, read-only handles that prevent cleanup, temporary publication placement and file/path handling. Place temporary publication on the same filesystem as the final target when the existing atomic/exclusive contract needs it. Never replace exclusive creation with an overwrite or broad delete.

Test every retained scientific state, not only the summary file: interval/sample row counts, episode boundaries, validity flags, first-failure evidence, source assets, selected backend and execution identity. Check table schema/order/dtypes through the existing recording reader and SQL, not platform-specific raw DuckDB file-byte equality. Sealed run assets and numerical arrays may have exact field-level contracts; retain those comparisons.

Include both a known valid short fixture and the documented strict scenario that can stop at energy.constraint_work near t=0.005 s. The latter must retain its actual invalid outcome and prefix, not be relabeled success or lengthened by a tolerance change. Diagnostic-model-limits can support diagnostic runs only when explicitly requested and must keep validity false. Performance evidence must report actual realized simulation duration.

Exercise explicit Python-backend --record rejection before environment/window setup, no native fallback, model construction failure before capture, policy failure after capture, interruption, disk-full/permission-denied publication and retry without duplicated episodes. Preserve exception notes and primary-versus-secondary failure ordering. Include a finalization crash subprocess to prove no apparently complete run is published with missing data. Do not add per-step fsync or incremental disk persistence as a portability shortcut.

This task is qualification-first: when existing production code already satisfies the contract, implement the added test matrix without rewriting the lifecycle. Keep these original suites as independent acceptance controls while introducing platform-specific regressions:

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_recording_lifecycle.py tests/reference/test_native_recording_retention.py tests/reference/test_native_encode_columns.py tests/reference/test_native_research_recording.py tests/reference/test_native_ride_frontend.py tests/reference/test_native_research_frontend.py -q
```

For a demonstrated OS failure, modify only its existing capture/finalization boundary, rerun the same command, and include the counterexample in test_native_platform_recording.py. A passing preserved implementation is not grounds to invent a replacement subsystem.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_platform_recording.py tests/reference/test_native_recording_lifecycle.py tests/reference/test_native_recording_retention.py tests/reference/test_native_encode_columns.py tests/reference/test_native_research_recording.py tests/reference/test_native_ride_frontend.py tests/reference/test_native_research_frontend.py -q
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-research --backend native --physics-config examples/research/viewer_physics_welded.toml --track-file examples/research/rough_uphill_savage.toml --duration 0.04 --record --out output/portability/recording
```

Capture the actual output run directory printed by the CLI; do not assume it equals the --out parent. Run the same lifecycle suite on both platforms and in ASan where supported. Validate GUI reset/close publication through T19. Preserve failed-run artifacts and exit codes in acceptance evidence.

- [ ] **Step 5 - Review and commit.**

```sh
git add tests/reference/test_native_platform_recording.py tests/reference/test_native_recording_*.py tests/reference/test_native_research_recording.py tests/reference/test_native_ride_frontend.py tests/reference/test_native_research_frontend.py src/bike_sim/recording src/bike_sim/sim/research/recording_lifecycle.py
git commit -m "test: qualify cross-platform native recording lifecycle"
```

**Review gate:** No automatic recording, lost intervals, premature publication, overwritten run, or altered validity semantics. Real native recording finalizes and can be read on both platforms.

### T22: Separate portable recording inspection and playback from checked physical replay

**Files:** Create `tests/test_recording_platform_compatibility.py` and `tests/reference/test_native_platform_replay.py`; extend `tests/reference/test_native_replay_recording_v3.py` and `tests/reference/test_native_replay_session.py`. Modify `src/bike_sim/cli/replay.py`, `src/bike_sim/cli/recording.py`, `src/bike_sim/recording/replay_input.py`, `src/bike_sim/sim/research/playback.py`, `src/bike_sim/sim/research/replay.py`, and `src/bike_sim/sim/research/replay_session.py` only to preserve this explicit distinction. Store fixture manifests in `tests/fixtures/platform-recordings/` with actual generated data handled under the existing fixture policy.

**Interfaces:** Preserve bike-recording overview/fields/events/window/sql/export/compare. Preserve bike-replay RECORDING [--episode ID], --viewer for checked replay, --view for visual playback, and --view --dry-run for headless non-simulating playback. Use T12 require_execution_compatible before checked stepping. Analysis/playback never reports that physical replay was verified.

**Reason and boundaries:** Cross-platform numerical identity can legitimately differ. Users should still inspect supported portable recording data and visualize recorded states without importing a foreign native binary or silently running different physics.

- [ ] **Step 1 - Add the regression and a valid control.**

Create a small real run on each qualified OS and retain a manifest of generator commit, schema, packages, artifact identity and file hashes. Add tests that overview and SQL return the same semantic counts after moving the run directory. In a fresh process, patch native artifact loading and physical stepping to raise; read-only inspection and non-simulating dry-run must still operate on supported recorded data.

Add this compatibility assertion around the existing checked-replay construction fixture:

```python
import pytest
from bike_sim.native.execution_identity import require_execution_compatible

def test_foreign_identity_cannot_be_checked_replayed(recorded_identity, current_identity):
    recorded_identity["build_context"]["platform"]["system"] = "Darwin"
    current_identity["build_context"]["platform"]["system"] = "Linux"
    with pytest.raises(ValueError, match="execution.build_context.platform.system"):
        require_execution_compatible(recorded_identity, current_identity)
```

Define recorded_identity/current_identity fixtures from the complete v2 native fixture created in T12, deep-copy them for each test, and separately verify a real ReplaySession performs zero steps on mismatch. Test legacy identities, unknown schemas, corrupted assets, missing episodes, multiple episodes without explicit selection, moved directories and unsupported nonportable assets.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/test_recording_platform_compatibility.py tests/reference/test_native_platform_replay.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Keep data-only validators independent of current native artifact/dependency loading. Resolve recorded assets relative to the validated run container, not the recording creator's absolute checkout. Never execute a path, policy factory, shared library or arbitrary source code named by an untrusted recording during inspection. Retain existing schema/hash/path validation and reject path traversal.

Do not assume a MuJoCo binary model dump or every historical asset format is cross-platform merely because DuckDB opens. Inventory the actual stored asset types, use the existing authored/XML/config reconstruction route where already supported, and produce a specific unsupported-asset diagnostic otherwise. New source-first portable recordings should include the data/assets already required by the canonical reader; no new physics reconstruction shortcut is allowed.

For checked replay, compare full versioned execution identity before any step and retain recorded time/control sequence and existing tolerance policy. Same-environment v2 replay is required. A Linux recording on macOS or vice versa may be rejected with a field-level environment mismatch; this is a correct safety outcome, not a reason to delete provenance. Pure --view playback and --view --dry-run must use recorded frames, clearly labeled unverified playback. --viewer remains checked and cannot bypass identity validation.

Run normal read/analysis commands against both fixtures from both operating systems. Compare schema/row counts/selected values semantically; do not require two independently generated DuckDB files to have equal raw bytes. Keep existing exact numerical comparisons where defined, and use explicit compare --mode exact versus --mode summary rather than changing defaults. Verify no output run is mutated during inspection, including source file hashes before/after.

Replace the existing raw dictionary equality at the construction check in sim/research/replay.py with the T12 compatibility function, retaining its current schema/backend branch and its position before physical stepping:

```python
from bike_sim.native.artifact import execution_provenance
from bike_sim.native.execution_identity import require_execution_compatible

require_execution_compatible(manifest["execution"], execution_provenance(backend))
```

At the later ReplaySession identity recheck, use the same function with self._manifest["execution"] and execution_provenance(self.backend). Do not call either function in the independent data-inspection/non-simulating playback branch. Preserve legacy schema-1 logic where no execution record exists rather than indexing a nonexistent field.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

In a shell set RUN to the actual finalized directory printed by T21, then execute these exact command forms:

```sh
: "${RUN:?Set RUN to the finalized recording directory printed by T21}"
uv run --frozen --group native bike-recording overview "$RUN" --json
uv run --frozen --group native bike-replay "$RUN" --view --dry-run
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-replay "$RUN"
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native bike-replay "$RUN" --viewer --time-scale 1
uv run --frozen --group native python -m pytest tests/test_recording_platform_compatibility.py tests/reference/test_native_platform_replay.py tests/reference/test_native_replay_recording_v3.py tests/reference/test_native_replay_session.py -q
```

For a multi-episode run add the real --episode ID to checked replay, as the existing interface requires. Repeat inspection/dry-run on the other OS without copying the original extension. Require correct mismatch diagnostics for checked replay there. Interactive replay/playback is a real desktop gate from T19, not a headless CI assertion.

- [ ] **Step 5 - Review and commit.**

```sh
git add tests/test_recording_platform_compatibility.py tests/reference/test_native_platform_replay.py tests/reference/test_native_replay_recording_v3.py tests/reference/test_native_replay_session.py tests/fixtures/platform-recordings src/bike_sim/cli/replay.py src/bike_sim/cli/recording.py src/bike_sim/recording/replay_input.py src/bike_sim/sim/research
git commit -m "test: distinguish portable playback from verified native replay"
```

**Review gate:** Portable data can be inspected without a native binary; supported recorded-state playback works across hosts. Checked replay remains strict and explains incompatibility before stepping. No command mislabels playback as verification.
