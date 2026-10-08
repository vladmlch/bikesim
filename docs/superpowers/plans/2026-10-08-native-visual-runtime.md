# Native Visual Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the agreed welded bicycle physics in C++ with the existing Python viewer, faster playback, native research execution and checked visual replay.

**Architecture:** Python resolves configuration, builds terrain/model and solves equilibrium. An owned C++ runtime advances the complete physical/research loop; the Python frontend consumes snapshots and invokes external policy plugins only at policy boundaries. Python remains the default backend and numerical oracle.

**Tech Stack:** C++23, CMake, nanobind, MuJoCo 3.12.0, Python >=3.13 through uv, NumPy, existing MuJoCo passive viewer, pytest and the repository native verification launcher.

**Spec:** [Native visual runtime design](../specs/2026-10-08-native-visual-runtime-design.md). Read it before selecting a phase; it is the authority for capabilities, ownership, timing, recording and acceptance.

## Global Constraints

- Use C++23, nanobind, the existing CMake/toolchain conventions, and the pinned MuJoCo 3.12.0 native dependency group.
- Invoke Python and Python tools through `uv`.
- Keep the Python backend and 1x presentation as defaults.
- Preserve physical timestep, numerical operation order, controller periods, physical checks and existing tolerances.
- The supplied welded profile uses `timestep_s=0.00125`; all comparative measurements use that effective value.
- Keep external `module:factory` motor policies in Python, called once per external policy boundary.
- Use one owner thread per runtime. Release the GIL only around code that touches no Python objects.
- Performance evidence comes from an explicit Release artifact executing the complete supported runtime.
- Preserve user-owned untracked plans, skills and output. This design does not resume unrelated unfinished safety-plan tasks.

Read [native engineering rules](../../agents/native-engineering.md), [testing instructions](../../TESTING.md) and [ADR 0001](../../adr/0001-native-port-mujoco-core.md) before native/bridge work.

## Start here

The intended checkout is:

```text
/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2
```

Planning inspected HEAD `7a12394ed22e0657dbb02b29f23f76d5f8308dd5`. Confirm the checkout and current source before execution. The user requested these documents only; the planning session did not run builds, simulations or tests.

The implementation is one dependent runtime project, delivered through three phase documents. Load the active phase and its named source references; later phases become relevant when their dependency gate is met.

| Phase | Open when | Deliverable | Dependencies |
|---|---|---|---|
| [A: native physical runtime](2026-10-08-native-visual-runtime-core.md) | Starting work or changing core physics/setup/accounting | Reproducible supported C++ physical rollout with owned state and full samples | None |
| [B: native research](2026-10-08-native-visual-runtime-research.md) | Phase A accounted rollout passes | Native experiment scheduler, sensors, recording and Python policy seam | A0-A4 |
| [C: viewer, replay and acceptance](2026-10-08-native-visual-runtime-frontends.md) | Integrating graphical/CLI consumers or final evidence | Ride/research windows, checked visual replay, Release acceptance | A and B |

Use substantial component tasks. At most one native implementer should own the shared runtime/state/binding files at a time. An independent worker may implement pure playback tests after the interface contracts are stable. Run physical simulations and benchmarks sequentially.

## Current source facts that constrain execution

| Current behavior | Consequence |
|---|---|
| `native/src/stepper.cpp` calls bare MuJoCo step/forward; components are separately callable | A complete ride scheduler is new work |
| `native/src/writers/brake.hpp` implements legacy actuator braking | Port physical static-friction braking from `static_braking.py` |
| Native config/projection lives in `tools/native_config.py` and `tools/native_schema.py` | Move reusable code into the installed package; keep tools as forwarding compatibility shims |
| Native selected-artifact loading lives in `tests/reference/native_loader.py` | Extract a shared loader without weakening test selector/provenance checks |
| Spindle rider branch bypasses the non-spindle SciPy allocator | Port the current spindle algorithm; no optimizer substitution |
| Strict physical failures publish a completed accounting period before rejection | Preserve period publication and original first-failure timestamp |
| Existing replay checks Python source/runtime and uses only Python environments | Add backend provenance and common incremental replay execution |
| Existing realtime measurement overrides dt to .0005 by default | Make profile dt the default and pass .00125 explicitly for acceptance |
| Existing default native build does not guarantee Release | Build/select `native/build/release` explicitly |

These are planning-baseline observations, not evidence that a current binary implements the feature.

## Shared interfaces and file boundaries

The design spec owns semantics. Phase A introduces the following package boundaries:

| New package/module | Responsibility |
|---|---|
| `src/bike_sim/native/artifact.py` | Exact artifact loading and runtime provenance |
| `src/bike_sim/native/config.py`, `schema.py` | Existing writer projection/validation moved out of tools |
| `src/bike_sim/native/setup.py` | Capability validation and full t=0 bootstrap projection |
| `src/bike_sim/native/contracts.py` | Backend-neutral immutable results and view values |
| `src/bike_sim/native/runtime.py` | Native ride adapter and binding lifecycle |
| `src/bike_sim/native/research.py` | Native implementation of the research environment boundary |
| `native/src/runtime/` | Owning physical runtime, bootstrap, binding, accounting and research scheduler |
| `native/src/rider/` | Native spindle controller and intent primitives |

Phase C introduces `src/bike_sim/sim/playback.py`, `sim/ride/physical_view.py` and `sim/research/replay_viewer.py`. Existing viewer, CLI and replay modules delegate to these small boundaries. Keep legacy ride code on its existing path.

### A0: execution baseline and common test support

**Files:**

- Create `tests/reference/_native_runtime_support.py`.
- Create `tests/reference/conftest.py` with the build-selector fixture below (merge with it if a conftest exists when implementation starts).
- Create `docs/superpowers/audits/2026-10-08-native-visual-runtime.md`.
- Read current `AGENTS.md`, native engineering rules and `tools/run_tests.sh`.

**Interfaces:** produces reusable `make_python_ride()`, `advance_python()`, `assert_tree_close()` and evidence records used by phase tests.

- [ ] **Record branch, HEAD and user-owned changes.**

```bash
git rev-parse --show-toplevel
git branch --show-current
git rev-parse HEAD
git status --short
```

Completion: the audit names the exact checkout/HEAD, preserves existing changes, and records that this work implements the new visual-runtime plan rather than continuing an unrelated ledger.

- [ ] **Build the selected Release artifact and establish the required baseline once.**

```bash
uv run --frozen --group native cmake -S native -B native/build/release -DCMAKE_BUILD_TYPE=Release
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

Record test IDs/errors and selected extension, not just counts. A baseline failure permits source investigation and isolated implementation, but final full acceptance remains red until the gate passes or is explicitly reported unresolved. A missing mandatory tool is an environment blocker to that check, not a skipped success.

- [ ] **Add the common fixture and comparison helpers.**

```python
from collections.abc import Mapping
from pathlib import Path
import math

from bike_sim.cli import ride
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain

PHYSICS = Path("examples/research/viewer_physics_welded.toml")
TRACK = Path("examples/research/rough_uphill_savage.toml")

def make_python_ride(*, strict=False, decimation=1):
    args = ride.parse_args([
        "--physics-config", str(PHYSICS), "--track", str(TRACK),
        "--headless", "--duration", "1", "--no-plots",
    ])
    sim = ride.build_physical_simulation_from_args(args)
    sim.physical.set_strict(strict)
    sim.physical.set_record_decimation(decimation)
    return sim

def advance_python(sim, steps, control=None, *, front=0.0, rear=0.0):
    command = RideControl() if control is None else control
    rows = []
    cursor = -1
    for _ in range(steps):
        sim.step(front, rear, control=command)
        for sample in sim.physical.completed_samples:
            if sample.interval_id > cursor:
                rows.append(sample.as_dict())
                cursor = sample.interval_id
    sim.physical.flush()
    for sample in sim.physical.completed_samples:
        if sample.interval_id > cursor:
            rows.append(sample.as_dict())
            cursor = sample.interval_id
    return rows

def assert_tree_close(actual, expected, *, atol=1e-9, rtol=1e-9, path="root"):
    actual, expected = plain(actual), plain(expected)
    if isinstance(expected, Mapping):
        assert isinstance(actual, Mapping) and actual.keys() == expected.keys(), path
        for key in expected:
            assert_tree_close(actual[key], expected[key], atol=atol, rtol=rtol,
                              path=f"{path}.{key}")
    elif isinstance(expected, (list, tuple)):
        assert isinstance(actual, (list, tuple)) and len(actual) == len(expected), path
        for index, value in enumerate(expected):
            assert_tree_close(actual[index], value, atol=atol, rtol=rtol,
                              path=f"{path}[{index}]")
    elif isinstance(expected, bool) or expected is None or isinstance(expected, str):
        assert type(actual) is type(expected) and actual == expected, path
    elif isinstance(expected, (int, float)):
        assert not isinstance(actual, bool) and isinstance(actual, (int, float)), path
        assert math.isfinite(actual) and math.isfinite(expected), path
        assert math.isclose(actual, expected, abs_tol=atol, rel_tol=rtol), path
    else:
        raise AssertionError(f"unhandled comparison type at {path}: {type(expected)}")
```

This helper executes fresh scalar Python intervals and reads independently captured output; it does not reconstruct native results from the same force matrix. Integer/categorical schedule fields also receive explicit equality assertions in consuming tests. Chunking tests compare native snapshots bitwise with `_bits.assert_bitwise_equal`.

Add the following autouse fixture to the reference-test conftest. It aligns application adapters with the selected test artifact without importing an extension or changing production defaults. An explicit application selector remains authoritative so mismatches remain detectable.

```python
import os
import pytest
from native_loader import selected_build

@pytest.fixture(autouse=True)
def select_application_artifact_for_tests(monkeypatch):
    if "BIKE_NATIVE_BUILD_PATH" not in os.environ:
        monkeypatch.setenv("BIKE_NATIVE_BUILD_PATH", str(selected_build(os.environ)))
```

Completion: these helpers are exercised by the first consuming regression in Phase A. Mark full-plant tests `slow`. Application-facing native tests must select the same regular/sanitized artifact as the test launcher.

- [ ] **Create the evidence ledger.**

Use columns: task ID, source revision, red test/error, green command/result, selected artifact, inspection result, remaining limitations. Record baseline and timings separately from feature acceptance. Suggested checkpoint: `test: establish native visual runtime baseline fixtures`.

## Execution and verification rhythm

For each numbered task in a phase:

1. Add its behavioral regression and run the named failing test. Confirm the intended missing behavior caused the failure.
2. Implement the named interfaces/algorithm with its reference sources.
3. Run the targeted green tests and IDEA inspections on changed code.
4. Record the completion evidence; checkpoint only task-owned files with the specified commit message if commits are authorized in the execution environment.
5. Move to the next task only after that task's deliverable passes.

Use direct selected-artifact pytest during a task after building the modified extension:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_runtime_bootstrap.py -q
```

Replace the test path with the exact path named by the current task. At phase boundaries run the gate specified there. The final task runs `full`, which already includes native preflight at the planning baseline. Recheck the launcher if it changes.

## Requirement coverage and exit gate

| Requirement | Owning tasks |
|---|---|
| Explicit native backend, support matrix and correct selected artifact | A1, C2 |
| Full physical control, braking, rider and accounted state | A2-A4 |
| Reproducible bootstrap/reset and owner-only state | A1, A3, A4 |
| Native research with Python policy seam | B1-B3 |
| Current telemetry/records and compatible provenance | A4, B3 |
| 1x/2x/4x/8x viewer, snapshots and macOS startup | C1-C2 |
| Same-backend checked headless/visual replay | C3 |
| RTF and required correctness evidence | C4 |

The implementation is complete only after C4's checklist is satisfied and the final audit distinguishes completed requirements from any unresolved gate. Do not describe a frozen-force benchmark, a partial native controller, or a window-only smoke test as this feature.

## Handoff

Execute phases in order. Recommended delivery uses one substantial implementer per active component and a fresh reviewer at each phase gate; inline execution with the same gates is equally valid. The user has requested the plan, so implementation begins only upon a separate implementation instruction.
