# ADR 0004: Native spatial simulation and independent correctness gates

Status: proposed
Proposal date: 2026-10-23
Verification: pending V00 and V01

## Context

The spatial plan adds terrain queries, multidimensional tyre state, free-root
topology, steering, sensor observations and causal checkpoint owners to the
existing C++ simulation. These are new physical and interface contracts. Their
correctness cannot be established by declaring equality with an existing planar
trajectory or by enabling a configuration flag.

The supplied source tree contains [ADR-0002](0002-drop-python-oracle-parity.md)
and [ADR-0003](0003-native-authoritative-python-removal.md); those identifiers
are occupied. There is no accepted spatial ADR in that tree. This document
proposes identifier 0004. V01 must confirm availability in the actual Git
checkout before accepting or renaming it. Source authoring is not approval.

The contract authority is
[C0–C10](../superpowers/plans/2026-10-22-spatial-3d-native-contracts.md), with
[W00 declarations and W01–W05](../superpowers/plans/2026-10-22-spatial-3d-native-00-foundation.md)
and the machine-readable [registry](../../migration/spatial3d/contracts.json).

## Proposed decision

1. **The new spatial production path is native C++.** Model and terrain
   construction, kernels, runtime, sensors, state, replay and frontend consumers
   use the bindings-OFF root graph. New spatial work does not introduce Python
   twins, Python bootstrap or a second stepper.
2. **Preserve the existing planar contract.** Missing `spatial`, and explicit
   `spatial = "planar"`, retain the existing canonical configuration bytes,
   schema key sets, numerical operation order and replay identity. Existing
   planar live-oracle and frozen-golden obligations continue under ADR-0003's
   per-file transition rules. This proposal neither removes those gates nor
   changes the T63 source-removal requirement.
3. **Qualify new physics with independent evidence.** Each spatial subsystem
   has native unit/contract tests, physical invariants, independently sourced
   references and convergence checks. Examples include terrain analytical
   surfaces and ray roots; force/potential and virtual-work consistency; contact
   passivity and friction budgets; complete-state continuation; and the
   independently transcribed Meijaard benchmark. A finer tyre tier alone is
   not an independent reference. Planar bridge comparisons apply only where
   the physical setups and query laws are equivalent.
4. **Separate numerical identities.** In-build determinism and any claimed
   thread-count invariance are exact comparisons within one execution artifact.
   macOS arm64/Accelerate and Linux aarch64/OpenBLAS retain separate reference
   trees and explicit provider/compiler/engine/input identities. Cross-platform
   bitwise equality is not implied. Preserve `-ffp-contract=off`.
5. **Version each boundary independently.** Spatial config1, runtime2,
   writer3, bootstrap2, BKT3/v1, tyre-state1, checkpoint1, observation2,
   policy ABI2, catalog2, container2 and replay4 are distinct domains. Select
   the decoder by its integer version, then validate exact keys and identities
   before live mutation. Historical planar readers and writers retain their
   existing dispatch. The initial spatial runtime/writer/bootstrap tuple is
   `(2, 3, 2)`; mixed tuples are rejected.
6. **One owner per causal state.** The bootstrap owns engine, tyre, brake,
   rider, steering and outcome state. The semantic checkpoint adds research,
   sensor and policy continuation without competing copies. Restore stages
   and validates every section before commit; reset and continuation remain
   distinct operations. A missing resumable plugin state is an explicit
   capability limit, not an assumed policy continuation.
7. **Authoring and execution are separate phases.** W00–W55 write source,
   declarations, tests and data-only fixtures. They do not configure, compile,
   execute, capture simulation goldens or measure performance. Source handoff
   uses `source_written`/`handed_to_V`; only the assigned V task can establish
   verified status with commands, artifact identities, check counts, exit codes
   and retained evidence on the required platforms.
8. **Missing evidence fails closed.** Required null identities, missing
   fixtures, unsupported combinations and zero-check tests fail. Do not use
   `xfail`, skips, self-generated expected values or placeholder successful
   runs to make the suite green. W05 accepts valid configuration for inspection
   while reporting spatial execution unimplemented and qualification pending.
9. **Resolve contract defects explicitly.** Record deviations from W00 in
   [the deviation log](../../migration/spatial3d/contract-deviations.md). C10
   requires old/new contracts, the concrete reason, affected tasks and a
   migration/readback strategy, followed by coordinated producer/consumer
   updates. Relaxing tolerances, steering range or tyre quality merely to pass
   a gate is not authorized by this proposal.

## Consequences and acceptance

The current W00–W05 delivery establishes declarations, registries, tests and
configuration handling; it does not make a 3D ride executable or qualify its
physics. Declaration-only interfaces are implemented by the later W owners.
The [V plan](../superpowers/plans/2026-10-22-spatial-3d-native-09-verification.md)
owns compilation, execution, repairs and measurements. In particular, V00
resolves the open header deviations before compile closure, and V01 validates
the registries, harness, references, configuration and captured planar traces.

Acceptance of this ADR requires an explicit recorded decision. No approval,
build success, measured result or qualification is asserted here.
