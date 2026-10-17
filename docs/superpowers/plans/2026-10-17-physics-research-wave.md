# Physics and research T26–T37 Implementation Plan

> **For agentic workers:** Execute source-sized units with independent static review. The user explicitly prohibits compilation and execution; all runtime, oracle-capture, sanitizer and performance gates remain unrun.

**Goal:** Implement the remaining native physics and research interfaces assigned to T26–T37 while preserving the existing physical stepping path.

**Architecture:** Reuse the existing drivetrain, contact, tyre, sensor and runtime kernels. Add typed native allocation, legacy orchestration, policy and evidence owners, and connect them to native configuration/startup. Keep configuration capability decisions separate from schema validation.

**Tech Stack:** C++23, existing CMake graphs, MuJoCo, the repository's owned IO and numerical providers.

**Spec:** `2026-10-14-python-free-cpp-04-remaining-physics.md`, `2026-10-14-python-free-cpp-05-research-policies.md`, `migration/interface-contracts.md`, and `migration/cards/T26.md` through `T37.md`.

## Global Constraints

- No compilation, configuration, test runs, project execution, or oracle regeneration in this work session.
- Preserve Python sources as references until T63 and preserve existing strict arithmetic and transaction boundaries.
- No interpreter fallback or second physical integration engine.
- No unsupported mode may become accepted solely by removing a validator.
- Runtime evidence, numeric parity, safety and performance remain separate from static implementation evidence.
- Only the integration owner edits shared CMake, runtime dispatch, projection and migration ledgers.
- The referenced `docs/superpowers/specs/2026-10-06-native-safety.md` is absent from this snapshot; the available native engineering rules and current source contracts remain applicable.

## Work units and ownership

### T26–T27: Kernel inventory and drivetrain integration

**Files:** `native/src/drivetrain/chain.*`, `native/src/drivetrain/transmission.*`, `native/src/drivetrain/capabilities.*`, `native/src/runtime/project_config.cpp`, `native/tests/migration/kernel_parity.cpp`, `kernel_properties.cpp`, `test_t27.cpp`, `migration/kernel-reuse-map.json`.

**Interfaces:** Existing `drivetrain::DrivetrainWriter` and `Transmission` remain authoritative. Add pure source helpers for chain extension/Jacobian and transmission linearization/reaction, plus `drivetrain::implemented_capabilities()`.

- [ ] Inspect the source cards, native implementations and their consumers before editing each seam.
- [ ] Remove elastic/clutch/rotor startup rejections only after matching generated topology, writer behavior and reset state.
- [ ] Add regression sources for valid topologies, rejected combinations, geometry/force/energy invariants and detached failure state.
- [ ] Record explicit source/native symbol and test links; never infer parity from name matches or report an uncaptured oracle as present.

### T28–T30: Solvers, coupled rider and attachments

**Files:** `native/src/numerics/`, `native/src/rider/`, `native/src/runtime/capabilities.*`, `native/tests/migration/test_t28.cpp` through `test_t30.cpp`.

**Interfaces:** Owned `bike::rider::AllocationProblem`, allocation/constraint response and mode-search records; existing `::rider::SpindleController` stays authoritative for spindle mode. Existing contact writer owns attachment state and forces.

- [ ] Implement typed bounded allocation and conservative feasibility decisions, including explicit numerical provider identity.
- [ ] Port coupled response, activation and contact mode enumeration with residual-based acceptance.
- [ ] Dispatch flat/weld/spring support and controller combinations through coherent model/writer topology.
- [ ] Add static-readable regression sources for rejection, reset, overload and failure atomicity.

### T31–T33: Legacy ride and stand owners

**Files:** `native/src/legacy/`, `native/src/playground/`, `native/src/numerics/filter.*`, `native/tests/migration/test_t31.cpp` through `test_t33.cpp`.

**Interfaces:** Noncopyable legacy simulation/playground owners compose model generation and existing force writers; physical rides retain their current engine.

- [ ] Implement legacy tire, rider, pedaling, cruise, measurement, filtering and termination behavior.
- [ ] Integrate stand/playground controllers and state reset without importing graphical frontends.
- [ ] Add analytical and source-based regression cases and expose owned telemetry to callers.

### T34–T37: Research construction, policies and lifecycle

**Files:** `native/src/research/`, `native/include/bike/policy_v1.h`, `examples/policies/traction_control.cpp`, `docs/NATIVE_POLICIES.md`, `native/tests/migration/test_t34.cpp` through `test_t37.cpp`.

**Interfaces:** `ResearchRequest`, owned runtime construction, sensor-only `Policy`, validated v1 C ABI, `PolicySession` and sealed-episode summaries.

- [ ] Construct native research startup, programs and noise tapes from resolved user configuration.
- [ ] Implement built-ins and native plugin loading with exact optional control presence and explicit provenance.
- [ ] Preserve accepted command ownership across yields, stops, resets and failures; policies run only at external boundaries.
- [ ] Preserve sticky invalid status, separate physical/control extents and summary ownership after runtime close.

### Integration and handoff

**Files:** Shared CMake registration, `migration/physics-research-implementation.json`, task-specific static reviews, patch and changed-files archive.

- [ ] Register every new translation unit and real regression executable in both build graphs.
- [ ] Review declarations, consumers, includes, ownership, topology and numerical claims statically.
- [ ] Keep runtime gates pending, with the exact future commands documented.
- [ ] Produce a patch relative to the unmodified extracted snapshot and an archive containing every changed/new project file.

No task's original acceptance checkbox is checked by this plan. Acceptance still requires its source comparisons and platform gates after the user permits execution.
