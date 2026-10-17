# Native Startup Implementation Plan

> **For agentic workers:** Execute the existing T16–T18 and T20–T25 source cards task by task, with independent static review. The user's instruction for this delivery forbids compilation, configuration, tests and running the application; all execution gates remain explicitly unrun.

**Goal:** Construct a native ride from ordinary physics and track inputs without a Python-produced model or bootstrap prerequisite.

**Architecture:** Keep the existing native physical writers and runtime authoritative. An initialization owner assembles a generated model, resolved writer configuration and complete initial state, then seals the bootstrap consumed by `runtime::NativeRideRuntime`.

**Tech Stack:** C++23, CMake >=3.24, existing MuJoCo 3.12.0 ownership/error boundary, existing configuration, numeric, RNG and IO libraries.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`; detailed implementation steps and source inventories remain in `2026-10-14-python-free-cpp-02-configuration-models.md`, `2026-10-14-python-free-cpp-03-terrain-initialization.md`, `migration/interface-contracts.md` and `migration/cards/T*.md`.

## Global Constraints

- Preserve the existing single native physical stepping path and numerical operation order.
- No Python fallback, embedded interpreter, or interpreter subprocess in native startup.
- Preserve physical and legacy model exports, their geometry and named model entities.
- Keep source Python files until T63; do not mark runtime acceptance without executed evidence.
- Do not configure, compile, run tests, execute the application, or regenerate reference fixtures in this delivery.
- Shared build files, API integration and migration evidence have one integration owner.

## 1. Model generation — T16 and T17

**Files:** `native/src/model/builder.hpp`, `builder.cpp`, XML/topology/component builders, `export.hpp`, `export.cpp`, and `native/tests/migration/test_t16.cpp`, `test_t17.cpp`.

**Interfaces:** `bike::model::BuildRequest` owns resolved inputs; `generate_mjcf(const BuildRequest&)` returns XML; `compile_mjcf(std::string_view, const std::filesystem::path&)` returns a unique MuJoCo model owner. Terrain values come from the terrain owner's public header.

- [x] Port each T16/T17 source card and the mandatory mode/visual/name/error branches.
- [x] Add native regressions for generation, compilation diagnostics and model export contracts; leave execution unrun.
- [x] Review generated topology against the existing Python builders and native consumers.

## 2. Terrain — T20 and T21

**Files:** `native/src/terrain/` public records and implementations; `native/tests/migration/test_t20.cpp`, `test_t21.cpp`.

**Interfaces:** Owned `TrackSpec` and heightfield records; native track load/save/generator/profile/surface routines. Reuse T19 RNG instead of changing seeded behavior.

- [x] Port every terrain primitive, profile, surface, generator, preset and TOML round trip in the source cards.
- [x] Integrate heightfield generation with model construction and writer terrain projection.
- [x] Add deterministic, domain and round-trip regressions without executing them.

## 3. Model projection — T18

**Files:** `native/src/model/mapping.hpp`, `mapping.cpp`, `native/src/runtime/project_config.hpp`, `project_config.cpp`, `native/tests/migration/test_t18.cpp`.

**Interfaces:** `required_id(const mjModel&, mjtObj, std::string_view)` rejects missing named entities. Runtime/writer projection consumes typed physics and resolved model/terrain/rider data, preserving the source's configuration schema and field presence.

- [x] Transcribe the native configuration projection and physical force mapping.
- [x] Route projection through existing typed writer/config decoders and validators.
- [x] Review all name, units, enum, optional-field and initial presentation contracts against consumers.

## 4. Equilibrium and state — T22, T23 and T24

**Files:** `native/src/initialization/` settling, solver, refinement, sag and state/cache owners; `native/tests/migration/test_t22.cpp`, `test_t23.cpp`, `test_t24.cpp`.

**Interfaces:** Initialization owns the model/Stepper and mutable pre-ride state. Settling uses existing physical writers; state readers validate provenance, dimensions and values before mutation. A completed initialization seals only once.

- [x] Port settling and measured failure diagnostics, bounded refinement and sag fitting.
- [x] Port typed native state/cache persistence and retained historical readers.
- [x] Add regressions for invalid state, rejected refinement, nonconvergence and state restoration; leave execution unrun.

## 5. Native assembly and handoff — T25

**Files:** `native/src/app/` preparation/factory, initialization construction/sealing, build registrations and `native/tests/migration/test_t25.cpp`.

**Interfaces:** `bike::app::prepare(const std::filesystem::path&)` returns a prepared bootstrap plus metadata; `make_ride(PreparedSimulation)` creates the existing native runtime after preparation succeeds.

- [x] Resolve ordinary inputs and explicit request paths, assemble all preceding producers, and seal complete initial state.
- [x] Expose a native entry point for fresh startup and useful diagnostics.
- [x] Register every new translation unit and targeted migration test in the supported build graphs.
- [x] Independently review ownership, initialization/reset state, error paths and all cross-task interfaces.
- [x] Record source-supported changes separately from unrun acceptance gates and package an exact patch plus changed-file and full-source archives.

## Deferred execution evidence

The existing focused `migration_T16`–`migration_T18` and `migration_T20`–`migration_T25` build/CTest gates, native/full profiles and provider-specific oracle comparisons remain required after this delivery. Their commands are retained in the owner plans. This execution plan does not claim passing tests, numerical equivalence or real-time qualification.

## Delivery status

All implementation actions above have source changes and authored regression sources in this delivery. Static hygiene and packaging evidence are in `migration/native-startup-static-audit.json` and `migration/native-startup-implementation.json`. No configure, build, test, simulation or project execution has occurred. These implementation checkmarks do not close the original task cards or numerical/release acceptance gates.
