# Native engineering rules

Read this document before changing `native/`, native test loading, native config projection, physics mirrored in native
code, analysis wrappers, benchmarks, or sanitizer/coverage tooling. It applies to implementation and review.
The [safety specification](../superpowers/specs/2026-10-06-native-safety.md) defines the required behavior;
the [master plan](../superpowers/plans/2026-10-06-native-safety-master.md) tracks implementation. A planned check is not
evidence that the check already exists. (Both links reference documents recorded as `missing_inputs` by the T01
inventory — the specs tree is not preserved in this checkout; this file remains the authoritative in-repo rules text.)

These rules apply on every platform lane. Qualified development hosts, toolchain details and per-platform diagnostics
live in [development/linux.md](../development/linux.md), [development/macos.md](../development/macos.md) and
[development/troubleshooting.md](../development/troubleshooting.md); a platform difference never weakens a rule below.

## Establish the contract

1. Record the current commit and working-tree changes. Read the affected implementation, its Python counterpart, and the
   consuming call sites. Complete this step when each public input and mutable output has an identified owner and
   domain.
2. Treat review assertions as hypotheses. For each accepted assertion, retain at least two independent checks:
   implementation plus a runtime reproduction, library behavior, caller invariants, or a compiler control. Repeating the
   same source reading is one check. Label reproduced, source-supported, internal-precondition, and unresolved claims
   distinctly.
3. Select a regression that fails for the claimed reason and a valid-input control that exercises the same path. Isolate
   fatal-engine and undefined-behavior probes in subprocesses. Complete diagnosis when the failure and control outcomes
   agree with the proposed causal explanation.

## Implement the shared behavior

Use the specification's strict input domains in Python and C++ together until the Python replacement is qualified.
Preserve valid-input binary64 operation order (parity with the Python oracle per
[ADR-0003](../adr/0003-native-authoritative-python-removal.md) until a file's two-platform goldens and C++ port land;
afterwards determinism and goldens), and existing deliberate saturation. Keep exceptional sentinels local to named fields. Field errors identify the public
API and full field path and use the shared exception class.

Validate and stage before committing state. Include model coefficients, policies, telemetry, clocks, pending actuation,
and energy in the transaction boundary. Python-result boxing happens before commit when its failure could otherwise
leave an operation half completed. Engine failures use the documented poison contract. Complete an exception-safety
change when invalid-input and allocation-failure retries match a single successful operation from the original state.

Encode ownership with owned contexts, RAII, spans, and explicit ndarray return policy. Read-only struct access uses
read-only pointee spans. Arbitrary `mjData*` pairing is a caller precondition unless a context carries model identity;
dimension equality alone is insufficient. Classes using mutable scratch are confined to one owning context; concurrent
use requires separate instances or explicit synchronization outside realtime code.

## Verify the intended artifact

Use the test entry point and profiles in [AGENTS.md](../../AGENTS.md). Until `full` contains native preflight, a native
physics/realtime change requires **both** `native` and `full`. Determine whether that integration exists from the
launcher code, not from a prior assistant message.

For sanitizers, RTSan, or coverage, follow [TESTING.md](../TESTING.md) and verify the imported extension path and tool
runtime in the test process. A green test of a regular extension is not a sanitizer result. A sweep must report selected
translation units, tool exit health, and diagnostic outcome. Resolve missing mandatory tools; experimental findings
remain report-only while execution health is visible.

Run the focused regression first, then required profiles once the task's implementation stabilizes. Re-run broader
profiles after subsequent relevant changes. Record commands, selected build, compiler/library versions, counts,
exclusions, and logs. Complete verification only when required gates pass for the final code and there are no
unexplained skips, zero-unit sweeps, or artifact mismatches.

## Review and hand off

Review correctness, ownership, failure atomicity, numerical parity, tool health, and performance separately. A count of
C++23 features or warning flags is not proof of safety. Apply `noexcept` only to proved nonthrowing interfaces;
allocating exception construction stays outside RTSan regions. Measure allocation and performance assertions on the
supported paths.

For each task, report what changed, why, the evidence, and remaining scope limits. Update the plan checkboxes only after
its completion gate. Record a suppression with checker, origin, reason, reproducer, and removal condition. Complete a
handoff when every task requirement is implemented, verified, or explicitly deferred with a reason and dependency; never
label a deferred requirement complete.
