# Native Research and Policy Ownership Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement T34–T37 over the existing native physical/research kernels, including source-free construction, policy ownership, lifecycle and sealed evidence.

**Architecture:** Resolve declarative research requests into a `SetupRequest`, finish equilibrium once, and apply the source research startup force probe before sealing. Construct the existing `NativeResearchRuntime` with a native NumPy-compatible noise tape and its already-delivered startup row. Keep motor policies outside the physical loop, and freeze metrics and metadata with owned episode records.

**Tech Stack:** C++23, existing MuJoCo 3.12.0 ownership/error boundaries, native NumPy-compatible RNG, C-compatible versioned plugin ABI, existing TOML/JSON and SHA-256 facilities.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`, `migration/interface-contracts.md`, `migration/cards/T34.md` through `T37.md`, and `docs/superpowers/plans/2026-10-14-python-free-cpp-05-research-policies.md`.

## Global Constraints

- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- The user's explicit prohibition overrides execution steps: do not compile, configure, run simulations, tests, or other project executables. All runtime acceptance remains pending; inspect sources and diffs only.
- `docs/agents/native-engineering.md` references `2026-10-06-native-safety.md`, which is absent from this uploaded snapshot. Preserve the available native rules and record this input limitation.

## T34: Native construction and programs

**Files:** `native/src/research/environment.hpp`, `environment.cpp`, `configuration.cpp`, `program_loader.cpp`, `rider_random.cpp`; research-only additions in `initialization/setup_owner.hpp/.cpp` and `initialization/research_startup.cpp`; `native/tests/migration/test_t34.cpp`.

**Consumes:** `initialization::make_setup(SetupRequest)`, `settle(SetupOwner&, EquilibriumOptions)`, `SetupOwner::seal_bootstrap()`, `random::SeedSequence/NumpyRng`, `runtime::ResearchPrograms/SensorPipeline/NativeResearchRuntime`.

**Produces:** `research::ResearchRequest`, `read_research_request(path)`, `prepare_environment(ResearchRequest)`, `make_environment(path)`, native program loaders and seeded rider conversion.

- [x] Parse experiment and sensor defaults with exact domains, nonnegative decimal seeds and integer period checks; reject unknown request/program keys. Async telemetry rejects before setup pending its T42 owner.
- [x] Reuse `ResearchPrograms` interpolation and native random transforms; never substitute standard-library distributions.
- [x] Apply the safe zero-torque startup force probe, refresh sensors, consume startup tape row once, and transfer the resulting owned state.
- [x] Add source-derived domain, interpolation, sensor startup and repeat-seed assertions. Runtime execution is pending by user instruction.

```cpp
auto prepared = bike::research::prepare_environment(request);
auto environment = runtime::make_research(std::move(prepared.bootstrap));
```

## T35: Native policies and plugin ABI

**Files:** `native/src/research/policy.hpp/.cpp`, `builtin_policies.cpp`, `plugin_policy.cpp`; `native/include/bike/policy_v1.h`; `examples/policies/traction_control.cpp`; `docs/NATIVE_POLICIES.md`; `native/tests/migration/test_t35.cpp`.

**Produces:** `Policy`, `load_policy(specification)`, explicit configuration/provenance, source builtin spellings and complete C ABI validation.

- [x] Preserve the distinction between historical callable builtins (absent demand means zero) and factory classes (absent demand remains automatic assistance).
- [x] Port traction-control integration, invalid-observation behavior, standstill release and base-policy composition in source operation order.
- [x] Validate ABI version/sizes/callbacks/flags, bounded errors, booleans, masks and control domains. Own plugin instance before module unload, catch callback exceptions at the host, and hash binary/config bytes.
- [x] Add policy trace/domain/ABI tests without executing them.

```cpp
auto policy = bike::research::load_policy("builtin:zero");
policy->reset("1234");
const auto command = policy->act(observation, demand);
```

## T36: Session and final-only lifecycle

**Files:** `native/src/research/policy_session.hpp/.cpp`, `lifecycle.hpp/.cpp`, `exit_codes.cpp`; `native/tests/migration/test_t36.cpp`.

**Produces:** owned `PolicySession`, queued operator commands, reset/advance/drain APIs, exact `episode_exit_code`, final-only lifecycle receipts and structured secondary failures.

- [x] Retain a computed policy command across an allocation failure before native acceptance; retry acceptance without another policy call.
- [x] Advance only the accepted runtime window across pacing yields; acknowledge once after materializing the return value. Queue pause/brakes/reset/stop at the external boundary.
- [x] Stage seeded reset input before resetting the runtime and preserve sealed evidence around reset/close.
- [x] Implement completed-step pacing, explicit finalization retry and original-exception preservation; publication is supplied by the recording owner rather than implemented as a second recorder.
- [x] Add counting-policy, queued-operator, partial-window and exit-classification source tests. Execution remains pending.

```cpp
session.begin_advance();
auto transition = session.advance_pending(0.008, target_step);
session.request_stop(); // drains only an accepted window
```

## T37: Owned evidence and metadata

**Files:** `native/src/research/evidence.hpp`, `metrics.cpp`, `quality.cpp`, `evidence.cpp`, `metadata.cpp`; owned status additions in `runtime/research.hpp/.cpp`; `native/tests/migration/test_t37.cpp`.

**Produces:** `summarize(const OwnedResearchEpisode&)` and complete research/physical outcome and extent metadata after runtime destruction.

- [x] Freeze live evaluation status separately from optional histories; off mode computes required checks without retaining trajectories.
- [x] Reuse native `research_quality`, `channel_violations`, `ModelStatus`, wheelie metrics and reference-monitor first failure.
- [x] Keep physical/accounted/completed-control extents, requested/achieved timing, invalidity and replay eligibility separate.
- [x] Preserve null/absent values and explicit source/config/program/plugin identities in metadata, including the new allocation solver's pending numeric qualification.
- [x] Review source call chains, interface matching, error/cleanup ownership and diff whitespace; record unexecuted gates honestly in `migration/physics-research-policies-static-review.md`.

These checkmarks record authored source work only. All compilation, runtime,
oracle, performance and removal gates remain pending; see the static review.

```cpp
const auto owned = session.seal_episode();
session.close();
const auto summary = bike::research::summarize(owned);
```
