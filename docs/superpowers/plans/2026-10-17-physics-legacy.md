# Legacy Physics and Suspension Stands Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement T31–T33 as native, owned legacy ride, measurement and headless playground APIs.

**Architecture:** Preserve the existing typed configuration, model builder, Stepper, force kernels and legacy equilibrium solver. Assemble the nonphysical force ordering in a separate `bike::legacy::RideSimulation`; keep legacy pneumatic material equations distinct from the physical pneumatic writer. Publish owned snapshots and measurement values so later GUI and recorder adapters need no live mutable engine pointers.

**Tech Stack:** C++23, existing MuJoCo boundary, standard library, existing numerical kernels and configuration readers.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`; `migration/interface-contracts.md`; T31–T33 in `docs/superpowers/plans/2026-10-14-python-free-cpp-04-remaining-physics.md`; `migration/cards/T31.md`, `T32.md`, `T33.md`.

## Global Constraints

- User instruction: do not compile or execute the application, tests, build scripts or analysis tools. Only inspect and edit source; leave all execution gates explicitly unverified.
- Keep `native/src/` as the native implementation home.
- Preserve existing no-fast-math/disabled FP-contraction flags and operation/reduction order.
- One owner controls each mutable runtime/model/data pair.
- Python source remains available until T63; no interpreter fallback is added.
- GUI/window ownership belongs to T48 and is absent from the headless controller dependency graph.
- The referenced `docs/superpowers/specs/2026-10-06-native-safety.md` is absent from this snapshot. Apply the available native-engineering rules, existing owner interfaces and migration specification; record the missing source.

---

### Task 1: Native nonphysical ride and tyres (T31)

**Files:** Create `native/src/legacy/{ride.hpp,ride.cpp,tyre.hpp,tyre.cpp,force_accumulator.cpp,recorder.cpp}` and `native/tests/migration/test_t31.cpp`.

**Interfaces:** Consume `model::BuildRequest`, `initialization::SetupRequest`, `Stepper`, `nativecfg::WriterConfig`, existing suspension/brake/cruise/resistance/rider writers. Produce complete owning `RideSimulation`, `make_ride(path)`, explicit step/control/reset/snapshot methods and read-only telemetry.

- [ ] Read the T31 source declarations and their actual consuming order in `ride_sim.py`.
- [ ] Preserve sphere, legacy pneumatic and supported material variants with separate legacy tyre state; resolve road samples before the first engine forward.
- [ ] Assemble suspension, tyre, cruise, rider, resistance, leg and braking forces in the source order, then perform one engine step. Capture force contributions as owned vectors.
- [ ] Restore all startup state on reset, including controller/material clocks, and retain snapshots independently of owner lifetime.
- [ ] Add focused C++ regressions covering lifecycle, unsupported physical handoff, zero-speed resistance and tyre pressure/tier branches; do not run them.

### Task 2: Legacy rider, controls and reductions (T32)

**Files:** Create `native/src/legacy/{controls.hpp,controls.cpp,rider.hpp,rider.cpp,legs.hpp,legs.cpp,metrics.hpp,metrics.cpp}`, `native/src/numerics/{filter.hpp,filter.cpp}` and `native/tests/migration/test_t32.cpp`.

**Interfaces:** `legacy_ride_filter(span<const double>,double)` returns an owned filtered series; leg/energy/event functions return typed values; the legacy ride owner calls the controller/leg APIs directly.

- [ ] Read `energy.py`, `leg_loop.py`, `leg_drive.py`, `wheelie.py`, `metrics.py` and related control/termination sources.
- [ ] Port signed-crank leg geometry and force split, rider control and mechanical accounting with the exact source axis/sign conventions.
- [ ] Implement fourth-order Butterworth 100 Hz design, odd padding, steady-state initialization and forward/backward filtering; preserve short-series and Nyquist source branches.
- [ ] Implement ramp-window travel/load/acceleration/energy/event reductions without modifying simulation state.
- [ ] Add meaningful C++ regressions for constant/impulse/short filters, leg closure, event boundaries and energy; leave their execution unchecked.

### Task 3: Headless suspension playground (T33)

**Files:** Create `native/src/legacy/{playground.hpp,playground.cpp,suspension_controller.cpp,static_equilibrium.cpp,telemetry.cpp}` and `native/tests/migration/test_t33.cpp`.

**Interfaces:** Complete `Playground` owner, `make_playground()`, `step(Playground&)`, `reset(Playground&)`, model selection/damper adjustment, immutable telemetry.

- [ ] Read `controllers.py`, `playground.py`, `telemetry.py`, stand model generation and damper settings.
- [ ] Build standard/stand/playground modes through the existing MJCF builder and engine image owner.
- [ ] Apply compression/rebound suspension and static-equilibrium controls, preserve model-switch and reset behavior, and publish labels/units needed by the later HUD.
- [ ] Add lifecycle, model switching and damper/reset regressions without loading any GUI dependencies.

### Task 4: Static handoff

**Files:** Create `migration/physics-legacy-static-review.md`. The integration owner updates common CMake, API and migration ledger files.

- [ ] Statically inspect includes, ownership order, named topology, required field domains and cross-task calls.
- [ ] Document exact source-to-native coverage and the remaining unexecuted parity, sanitizer, performance and integration gates.
- [ ] Hand the integrator the complete source list and required CMake/API edges.

## Deferred execution commands

These commands are supplied for a later authorized run; they must not execute in this task:

```bash
cmake --build build/native-release --target migration_T31 migration_T32 migration_T33
ctest --test-dir build/native-release -R '^migration_T3[123]$' --no-tests=error --output-on-failure
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
```

Completion of implementation is distinct from completion of these gates. Task cards and source-removal rows must not be marked verified based on static inspection alone.
