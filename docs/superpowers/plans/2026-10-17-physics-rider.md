# Physics and research: rider solvers and supports Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the native T28 allocation/numerics, T29 constraint response and legacy rider controller, and T30 support dispatch while preserving the existing spindle path.

**Architecture:** The new `bike::rider` matrices own row-major data and callbacks own their captures. Constraint extraction consumes the model/data belonging to the existing controller context. The contact dispatcher reuses `writers::RiderContactWriter`, which already implements flat pads, weld/pin reactions, spring release and attachment energy.

**Tech Stack:** C++23, existing MuJoCo owner and numerical providers; no new interpreter dependency.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`, `migration/interface-contracts.md`, T28–T30 in `docs/superpowers/plans/2026-10-14-python-free-cpp-04-remaining-physics.md`.

## Global Constraints

- User instruction: do not compile or run project code, tests, benchmark commands, configuration generators or interpreter imports. The commands below are deferred qualification instructions.
- Preserve no-fast-math/disabled FP-contraction flags and existing spindle statement order.
- Preserve box projection, exact fast path at residual <=1e-7, equalities/linear inequalities/nonlinear-extra ordering, ftol=1e-12, maxiter=500 and up to four warm restarts.
- An unknown/numerically failed linear solve cannot certify infeasibility.
- New numerical implementations declare their identity; source-supported implementation is not a bitwise or runtime qualification result.
- Preserve Python sources. Do not mark migration gates passed without executed evidence.
- The snapshot has no Git history and omits `docs/superpowers/specs/2026-10-06-native-safety.md`; use the present native-engineering rules and report that evidence gap.

---

## File map and integration

| Files | Responsibility |
|---|---|
| `native/src/rider/allocation.hpp`, `allocation.cpp`, `allocation_json.cpp` | Shared records, domains, source residual/box projection, inverse dynamics and grip constraints; cold JSON is isolated from the numerical target. |
| `native/src/numerics/dense.hpp`, `slsqp.hpp`, `slsqp.cpp`, `linear_feasibility.hpp`, `linear_feasibility.cpp`, `nnls.hpp`, `nnls.cpp`, `brent.hpp`, `brent.cpp` | Owned numerical kernels with explicit provider identity and failure status. |
| `native/src/rider/constraint_dynamics.hpp`, `constraint_dynamics.cpp`, `mode_search.cpp` | Soft-constraint affine modes, unilateral/friction domains, deterministic complete/incomplete search. |
| `native/src/rider/activation_allocation.hpp`, `activation_allocation.cpp`, `response_allocation.hpp`, `response_allocation.cpp` | Delivered activation equality, physical support/task/power bounds, branch warm starts and measured certification. |
| `native/src/rider/legacy_controller.hpp`, `legacy_controller.cpp` | Non-spindle rider control through the existing runtime controller owner. |
| `native/src/rider/support_dispatch.hpp`, `support_dispatch.cpp`, `flat_contacts.cpp`, `weld_settle.cpp`, `grip_springs.cpp` | Support force targets and topology-checked reuse of the existing contact writer. |
| `native/src/runtime/capabilities.hpp`, `capabilities.cpp` | Explicit configuration/topology acceptance without weakening physical validators. |
| `native/tests/migration/test_t28.cpp`, `test_t29.cpp`, `test_t30.cpp` | Analytic and failure regressions written but not executed. |
| `migration/physics-rider-static-review.md` | Source mapping, integration seams, actual static evidence and deferred runtime gates. |

The integration owner registers source/test targets, updates shared config decoding and runtime assembly, and owns migration ledgers. Existing `::rider::SpindleController` remains the runtime interface; additions must leave its current spindle mode's arithmetic unchanged.

### Task 1: T28 allocation and numerical boundaries

- [x] Add dimension/domain checks and owned matrices using the interface contract. Check a bounded target before invoking nonlinear callbacks outside their declared box.
- [x] Implement source-order residual and constraint conversion, projected nearest-point iteration with four restarts, and explicit failure status. Record the native algorithm identity instead of claiming SciPy bitwise identity.
- [x] Implement a linear relaxation that prunes only with a verified infeasibility certificate; retain unknown on rank/budget/numeric failures.
- [x] Add NNLS with nonnegative active set and bracketing root solve with source tolerance defaults.
- [x] Add deterministic analytic regressions:

```cpp
// The warm start must never override the exact projection optimum.
AllocationProblem p{{2., -3.}, {}, {}, {}, {}, {0., 0.}, {1., 1.}, {}, {{.2, .3}}};
const auto a = allocate(p);
bt::require(a.feasible && a.solution == std::vector<double>({1., 0.}), "box projection");
```

- [ ] Future qualification: `cmake --build build/native-release --target migration_T28` followed by `ctest --test-dir build/native-release -R '^migration_T28$' --no-tests=error --output-on-failure`; not executed in this task.

### Task 2: T29 current-state response and mode search

- [x] Extract mass, known forces, actuator columns and dense/sparse Jacobian without reading solved reaction forces or previous acceleration for the coupled response.
- [x] Reproduce bounded dual active-set pivots and each mode's primal/dual domain. Validate fixed rows before indexing.
- [x] Search deduplicated modes in deterministic order; prefer the failed iterate's actual region; completeness means an empty pending queue.
- [x] Implement activation delivery/Jacobian and projection, grip branch ranking, physical support guards, crank task coordinate and power epigraph.
- [x] Integrate non-spindle compute/setup and all extra warm-start state with the existing controller owner. Diagnostic fallback remains invalid; no residual tolerance is relaxed.
- [x] Add regressions for truncated versus complete searches, uncertain LP, activation power saturation and infeasible diagnostic output.
- [ ] Future qualification: the T29 CTest plus source-specific rider dynamics/allocation/coupled-task and `project/tests/reference/test_constraint_mode_search.py`; not executed here.

### Task 3: T30 support topology and physical dispatch

- [x] Reuse existing pad/weld/grip force writers through explicit mode dispatch; check declared equality/site/body topology before constructing a writer.
- [x] Add gravity and pedaling support-target solvers with finite/domain checks and source active-foot/friction logic.
- [x] Bind capabilities to real controller/writer paths, retain source-invalid combinations, and publish diagnostics for physically invalid solved supports.
- [x] Add regression source for topology selection, flat support geometry, spring release/energy and constraint reactions.
- [ ] Future qualification: the T30 CTest and source topology/attachment tests, then serialized native/full gates; not executed here.

### Task 4: static handoff

- [x] Read changed declarations and their consumers, inspect whitespace and source mappings, and record only observed results.
- [x] Deliver exact source lists/signatures to the root integrator. Record remaining qualification requirements in `migration/physics-rider-static-review.md`.

## Static review adjustments

The dense least-squares adapter reuses the existing `rider::LeastSquaresWorkspace` and DGELSD supplier instead of adding another SVD. The planar observability gate uses its singular values with the source's absolute `1e-12` threshold, distinct from relative `lstsq` rcond. Density vector reductions use `numeric::numpy_pairwise_sum`, and fitted prediction uses the source's Fortran-contiguous BLAS matvec. All checkboxes above describe source authored/reviewed, not executed acceptance.
