# ADR 0003: Native core is authoritative after per-file golden capture

Status: accepted
Decision date: 2026-10-20
Recorded in this source snapshot: 2026-10-21

Authority: D12 and Task 1.04 of
[the C++ + Linux completion plan](../superpowers/plans/2026-10-20-cpp-linux-completion.md).
This decision supersedes older statements that Python remains permanently
or that all live-oracle comparisons can be dropped immediately.

## Context

The completion plan removes every first-party Python file at T63. Its
decision record describes ADR-0002 from the branch `feat/drop-oracle-parity`,
which retained Python as a readable specification while dropping live
Python/native parity. The transition below preserves the evidence used to
review the current native behavior before the oracle disappears.

On 2026-10-21, [ADR-0002](0002-drop-python-oracle-parity.md) and the r2
research payload were merged into this source snapshot from the supplied
`drop-oracle-parity.xml.zip`, as required by Task 1.03. ADR-0002's status
records this ADR's precedence over its immediate parity-removal and
permanent-Python clauses. The Repomix archives do not provide Git history:
this source merge does not establish a Git merge commit or claim that
Task 1.03's L1 build and checker gates passed.

The actual checkout subsequently merged `feat/drop-oracle-parity` at
`64c0a50043fd87a695e61a402572a63ee44fcfc4`, restoring its nine compressed
research datasets and preserving this ADR's precedence. The existing
shortened geomorphology-report path is retained for Linux portability.
This completes the source/history integration; Task 1.03's L1 gates
remain separate from that integration.

## Decision

1. **Transition per test file.** A pytest file comparing native results to
   the live Python oracle stays a bitwise gate until its porting task
   captures the consumed values on both macOS arm64 and Linux aarch64
   under `tests/fixtures/golden/<platform>/` and lands the C++ test port.
   From that commit the native test against those goldens becomes the gate.
2. **After T63 the native core is the only implementation.** Its gates are
   in-build determinism, native unit and contract tests, cheap physical
   invariants, frozen oracle values, and reviewed native self-goldens
   produced by `bike-recording golden freeze`.
3. **Golden changes require review.** Each change states why behavior
   changed, includes before/after `bike-recording golden compare` output,
   and modifies no unrelated golden.
4. **Per-platform goldens.** macOS/Accelerate and Linux/scipy-openblas keep
   separate golden trees. Cross-platform bitwise identity is not claimed.
5. **Python is deleted at T63, not archived in the active tree.** Git
   history and the `reference/python-oracle` tag preserve the oracle.

## Consequences

- The oracle pinning fixture and its pure-Python arithmetic twins stay
  until T63. A signature change updates the twin and fixture together.
- Valid-input binary64 operation order and `-ffp-contract=off` stay.
- A physics change after capture updates the affected native self-goldens
  in the same reviewed change.
- No test is retired before its per-file two-platform capture and C++
  replacement. This ADR does not itself close qualification or deletion.

## Note 2026-11-04: darwin-arm64 goldens re-recorded natively after the joint-damping removal

Joint damping was removed from the model in every form (rider and bike
joints). The stored `model.mjb` fixtures were patched to `dof_damping = 0`
(the builder output with damping removed matches the patched models), and
every darwin-arm64 golden whose values depend on the model was re-recorded
by the native program in record mode (`BIKE_GOLDEN_RECORD=1`). Entries
whose actual values still matched within their recorded tolerance were
kept, so unrelated goldens are unchanged. These goldens now prove "native
today equals native at re-recording", no longer "native equals the removed
Python oracle". The episode golden `episode/pinned_savage_40` was re-frozen
with `bike-recording golden freeze`; the before comparison (old golden
against the new program) is kept in `docs/superpowers/audits/2026-11-04-no-joint-damping-episode-{before,after}.txt`. The
linux-aarch64 goldens are not re-recorded yet and are stale until the
container run.

## Note 2026-11-05: legacy mode, the stand model modes and the seated rider variant removed

The legacy physics mode (`physics_mode = "legacy"`), the stand model
modes, the legacy damper curve and the `seated` rider variant with its
`rider_forces` writer were removed; their code paths, CLI flags and
consumed goldens are deleted. Physical mode is the only ride/research
mode and a legacy `physics_mode` document now fails validation loudly
(`native/src/configuration/schema.cpp`). Goldens that only the removed
paths consumed were deleted with them; fixtures retained for other
controls are unchanged.
