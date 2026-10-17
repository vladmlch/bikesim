# T26–T37 implementation and static review

Date: 2026-10-17.

## Follow-up review of commits 9c940c96 and 9d9e49c0

**Changes required:** five open source-supported findings are detailed in
[the commit review](native-waves-commit-review.md): native-reference
bootstrap failure (P1), zero-torque ideal speed control (P1), missing
native-reference resistance (P2), incomplete policy ABI numeric validation
(P2), and pedal recovery ignoring its configuration switch (P2).
The review checks the integrated Git tree and does not claim runtime
reproduction. No tests, build, simulation or policy execution was performed.
The implementation and acceptance statements below must be read with these
open findings; task and deletion gates remain open.

## Result and evidence boundary

The Physics and research group has implementation changes, shared build
registration and focused regression sources in the restored Repomix tree.
The inventory contains 12 tasks, 104 Python source modules and 1,044
declarations. T26 accounts for 58 modules and 508 declarations.

This is **uncompiled and unexecuted source delivery**. The user prohibited
compilation and execution. No configure/build/compiler/linker, project test,
simulator, solver, native policy, project Python module or oracle generator
was run. Text searches, source inspection, JSON updates, hash comparisons and
archive preparation are the available evidence. They do not prove that the
C++ compiles or that numerical/runtime contracts pass.

All task cards remain open. Existing removal status, T61/T62 acceptance and
T63 deletion gates are retained. Expected oracle values were not regenerated
from the implementation.

## Integration decisions

The implementation reuses Stepper, native model/config/terrain generation,
writers, state codecs and checked numerical helpers. Physical, legacy and
research owners retain their respective lifecycle responsibilities.

The physical capability gate describes the implemented drive, rider,
attachment and tyre routes while keeping qualification pending.
Native-reference wheel contacts enter the common typed snapshot schema.
Their full force/couple survive measurement; current support and bridged
load remain distinct. Material tyres admit an explicit radial table.

The spindle facade delegates non-spindle articulated allocation to the new
legacy controller. Support availability, recovery, latent activation,
delivered effort, support targets and per-joint terms cross the runtime/state
boundaries. Equilibrium finalization retains actual latent activation when
the legacy allocator ran.

Allocation has the explicit identity
`bike-dense-sqp-bfgs-v1;ftol=1e-12;maxiter=500;restarts=4`. It is not qualified
SciPy SLSQP equivalence. Capability reporting and conditional setup/research
metadata expose this identity. Returned-point residuals determine
feasibility; linear pruning distinguishes proof from unknown.

Legacy covers force accumulation, controls, rider/legs, sphere/pneumatic
contact, recording, filtering, metrics and headless stands. Detailed source
mapping is in [the legacy review](physics-legacy-static-review.md).

Research captures the first observation at its source lifecycle boundary,
keeps sensor/command clocks separate, reuses seeded streams and owns reset,
seal and recording data. Built-ins and the C plugin ABI share validated
inputs and controlled errors. Episode evidence retains configuration,
source/provider/asset identities and logical recording IDs.

## Findings corrected during static integration

- Removed a legacy include of the binding-only Python configuration header.
- Passed the actual drive mode to command validation and avoided articulated
  pose decoding for models without an articulated rider.
- Connected support availability, recovery and legacy diagnostic state to
  runtime publication, including source-specific omitted keys.
- Changed density/flat-load reductions to existing NumPy-compatible
  arithmetic and reused the checked least-squares provider. Added validation
  to the public allocation-residual boundary.
- Qualified original global `::rider` references after adding `bike::rider`
  exposed a namespace lookup collision in existing initialization/model code.
- Retained distinct file-request and typed-API research defaults.
- Found that `async_telemetry` had no native worker implementation. Its source
  owner is T42; true now fails explicitly before setup instead of being
  recorded in metadata while having no effect.
- Added a zero-step legacy preparation outcome without entering positive-step
  execution.
- Guarded MuJoCo contact/Jacobian queries and handlebar load with engine calls,
  model-owner checks and finite aggregate validation before committing state.
- Restored caller ownership after policy initialization failure so the
  zero-step environment can still be sealed, and added explicit model-status
  reset while preserving calibrated thresholds.

These are source-inspection findings and corrections, not executed
regression results.

Independent source reviews are recorded in
[rider and allocation](physics-rider-static-review.md),
[legacy and shared contact integration](physics-legacy-static-review.md), and
[research and policy ownership](physics-research-policies-static-review.md).

## Tests and fixtures

`native/tests/migration/CMakeLists.txt` declares native executables for
T26–T37 plus native-reference contact tests. T35 declares two shared-module
fixtures for real loader/callback failures. Existing strict compiler settings
remain in force.

Authored cases cover arithmetic/material controls, transmission geometry,
allocation/constraint residuals, support/controller state, legacy
lifecycle/filter/metrics, stand controls, clocks/seeds, ABI/lifecycle and
owned evidence. Their presence does not assert complete coverage of all
1,044 declarations and their branches.

The T26 map records every inventoried declaration, source/native locations,
focus targets, existing source test modules and unqualified lexical
branches. It distinguishes direct symbol mapping, integration into an owner
and downstream application boundaries. Every row is ineligible for deletion.

The snapshot lacks some binary64 oracle files, including legacy-flat
bootstrap and filter inputs/expected outputs. Tests retain those original
paths. The missing bytes are not silently skipped or replaced. Same-provider
bitwise comparison, solver qualification, long physical cases,
alias/ownership/error checks, sanitizers and performance remain future gates.

The captured research bootstrap also refers to absent `research.arrays` and
`state.arrays` binary64 files and an absent `model.mjb`. The file audit lists
these indirect references as well as literal paths in the authored tests.

## Remaining acceptance

1. Recover missing immutable oracle bytes and the referenced
   `docs/superpowers/specs/2026-10-06-native-safety.md`.
2. Configure/compile/link with the intended pinned toolchain and resolve
   strict diagnostics; this has not been attempted.
3. Execute T26–T37 and source-specific comparisons. Requalify affected
   upstream runtime/state/startup contracts, including current T11.
4. Acquire missing branch/domain evidence and qualify numerical identities.
   Keep failed/unknown outcomes as failed or unknown.
5. Complete physical, lifecycle, sanitizer and performance acceptance before
   changing package/capability/removal gates.

T38–T51 application work and T52–T60 platform/delivery work are not closed.
No commit or evidence from the user's original macOS worktree/HEAD is
available in this restored snapshot. The patch is based on the supplied
archive.
