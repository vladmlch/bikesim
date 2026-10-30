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

## Naming and diagnostic consistency

One concept has one name; one name has one meaning. A name states the operation's real semantics, not its surface
shape: `find_*` may report absence, `require_*` throws on absence, `read_*` produces bytes, `parse_*` decodes syntax,
`compute_*` produces a fresh value, `set_*` assigns, `update_*` re-derives from new inputs, `apply_*` performs an
effect. Renaming a member is a different change from renaming a serialized key; both need evidence, only the first is
internal. Predicates use `is_*`/`has_*`/`can_*`; configuration flags keep `use_*`/`*_enabled`; prefer positive
polarity. Collections take plural names, lookup fields name their key (`*_by_id`), and identity is explicit:
`*_id` identity, `*_index` position, `*_name` display text, `*_key` lookup/serialized key, `*_path` filesystem path.
Units live in names at API and diagnostic boundaries (`*_s`, `*_ms`, `*_ns`, `*_m`, `*_rad`, `*_nm`, `*_bytes`,
`*_count`); simulation time, monotonic elapsed duration and absolute timestamps are different concepts and are not
unified. Vendor spellings (`qpos`, `mjData`), STL/ABI pairs (`create`/`destroy`, `erase`/`remove`) and numerics
locals (`i`, `j`, `x`) are contract exemptions, not debt.

External spellings are contracts, not rename candidates: wire and JSON keys, TOML/schema identifiers, the policy C
ABI, exported binding names, CLI flags and golden fixture values keep their spelling even when the internal concept
is renamed. An internal rename must state which ambiguity it resolves and which external spelling it deliberately
preserves.

Diagnostics have three disjoint channels. Product output on stdout is the CLI contract. Operational diagnostics go
to stderr with one semantic owner per event and the form `LEVEL <component>.<operation>.<outcome>: message
key=value` — message a fixed phrase, values typed fields, never user input or exception text embedded in the event
name. Recorded evidence keeps its product schema and timestamps; it is not routed through operational diagnostics.
Realtime paths keep flag/code/deferred-message transport: no formatting, allocation, locking or stream I/O there. A
throwing layer adds context; the handling layer writes exactly one terminal event. A cleanup failure is a separate
event that never replaces the primary cause and never prints a false completion. Expected invalid input may end in
`ERROR` without a stack trace. Human diagnostics never embed raw argv, environment, credentials, plugin payloads or
whole documents: use an allowlist of typed fields and escape dynamic text as data. Disabled levels build no message
or fields.
