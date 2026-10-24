# Spatial 3D qualification — W54 evidence skeleton

Authored: 2026-10-24. **Status: pending V10. Execution: not_run.**

This document and [qualification.json](../../../migration/spatial3d/qualification.json)
declare the evidence required for the final spatial candidate. They contain no
build, simulation, sanitizer, replay, installation, GUI or performance results.
The machine-readable ledger has `qualification_schema: 1`, `status: "pending"`
and `identity: null`. Source authoring does not qualify a profile.

The acceptance authority is the
[W/V master plan](../plans/2026-10-22-spatial-3d-native-master.md),
[V10](../plans/2026-10-22-spatial-3d-native-09-verification.md#v10--final-qualification--s54--m6)
and [C8–C10](../plans/2026-10-22-spatial-3d-native-contracts.md).

## Candidate identity

| Field | Current value | V10 evidence required |
|---|---|---|
| Source revision | `null` | Final Git SHA; every dirty payload resolved or separately identified |
| Dependency hashes | `null` | Locked inputs and consumed dependency artifact hashes |
| Compiler and flags | `null` | Compiler/stdlib/flags identity for each platform artifact |
| Numerical provider | `null` | Provider identity for each platform; retain `-ffp-contract=off` |
| Configuration | `null` | Canonical configuration digest and physical asset/calibration identities |
| Platform artifacts | `null` | Separate macOS arm64 and Linux aarch64 executable identities |
| Profile selection | `null` | Explicit root, tyre tier, dt, seed, assets, sensor/recording modes and thread policy |

No source SHA is inferred from the Repomix archive, and no platform result is
copied from a different execution identity. Cross-platform bitwise equality is
not an acceptance claim; within-artifact determinism remains exact.

## Milestones

| ID | Required evidence | Owner | Status |
|---|---|---|---|
| M0 | Base identity, W00 headers, contracts/ADR, reference and harness controls | V01 | pending |
| M1 | Analytical terrain/raycast, tyre law, topology, state roundtrip | V02–V04 | pending |
| M2 | Native prepared/fresh constrained ride and numeric planar bridge | V05 | pending |
| M3 | Free-root cold start without hold, steering/grip, dt convergence, Meijaard applicability | V06 | pending |
| M4 | IMU V2, isolated policy, complete recording/checkpoint, checked replay, independent platform goldens | V07 | pending |
| M5 | Native CLI/viewer/batch/reports, historical reader and relocated installed product | V08 | pending |
| M6 | Final candidate, both platforms, physics/replay/realtime evidence and caveat ledger | V10 | pending |

Every milestone starts with `evidence: []`. V10 may set `passed` only after
retaining the command, selected build, executable/input identities, exit code,
executed check count and relevant output. A failed or blocked check is recorded
with its actual state. Missing fixtures, missing displays, unsupported hosts,
zero-check runs and unexecuted tests never become passing skips.

## Upstream gates

| Ledger ID | Plan gate | Required producer scope | Status |
|---|---|---|---|
| G0 | G0-source | Actual PFC base, dirty changes resolved, locks and fixtures available | pending |
| G1 | G1-core | Bindings-OFF root build, native owned runtime/bootstrap/Value/numerics and targeted native tests | pending |
| G2 | G2-startup | Native model/config/terrain/startup, cache import and construction without Python | pending |
| G3 | G3-research | Owned clocks, environment lifecycle, noise and policy boundary | pending |
| G4 | G4-recording | Catalog/source/reader/assets/goldens and checked replay | pending |
| G5 | G5-gui | Native CLI/viewer, rendering, input and pacing | pending |
| G6 | G6-qualification | Platform/tool provenance, install/package/offline and no-Python audit | pending |

The abbreviated IDs map exactly to the seven named master-plan gates. All seven
are mandatory for overall `passed`; the optional CPU-pool and GPU decisions are
recorded separately and cannot replace one of these gates.

## Machine-readable schema and validator

`spatial_tests::validate_qualification` validates an owned JSON value. Root
members are exactly `qualification_schema`, `status`, `identity`, `milestones`
and `upstream_gates`. Schema 1 is an integer lexeme. Status is one of `pending`,
`passed`, `failed`, `blocked`.

Both collections are arrays with exactly seven unique IDs: M0–M6 and G0–G6.
Each entry has exactly `id`, `status`, `evidence`. Unknown IDs, unknown statuses,
duplicate keys/IDs, missing entries and unexpected members are rejected with
the offending field path. An evidence array contains objects with exactly
`path` and `sha256`: a nonempty repository-relative POSIX path without traversal
and a 64-character lowercase SHA-256. Repeated paths within one entry are
rejected. Pending entries may have empty evidence arrays.

A populated identity has exactly these fields:

| Field | Structural domain |
|---|---|
| `source_sha` | 40- or 64-character lowercase Git object ID |
| `dependency_hashes` | Nonempty object mapping distinct dependency names to lowercase SHA-256 digests |
| `compiler` | Nonempty compiler identity text |
| `provider` | Nonempty numerical-provider identity text |
| `config_sha256` | Lowercase SHA-256 of the candidate configuration |

Compiler/provider text must identify the candidate's platform artifacts; the
hashed evidence retains complete per-platform records, physical asset hashes,
profiles and flags. The structural validator does not infer these records from
the text. A passed milestone or gate requires a populated candidate identity
and nonempty evidence. Overall `passed` requires all seven milestones and all
seven gates passed. Setting only the top-level status cannot bypass any gate.

**Structural validity is not execution evidence.** This Value-only validator
does not read evidence files, recompute their hashes, establish same-candidate
identity, inspect commands or verify the two platform results. V10 must perform
those checks before recording `passed`. In particular, the synthetic complete
accepted document in the validator test is never written to this ledger.

The source-authored `spatial.qualification_registry` test covers the pending
file, a complete synthetic accepted control, the plan's literal false pass,
missing/null/partial identity, bad schema types, duplicate and missing IDs,
pending gates under a passed root, invalid status, empty evidence, unsafe paths
and malformed hashes. It has not been compiled or run. The later command is:

```sh
ctest --test-dir "$BUILD" --output-on-failure -R '^spatial\.qualification_registry$'
```

## Final-profile evidence to acquire

| Required lane | Evidence | Status |
|---|---|---|
| Native/full regression | Final candidate on both platforms; zero unexplained skips or missing fixtures | pending V10 |
| Physical validity | Ordered terrain → tyre → free-root → startup → benchmark → convergence evidence | pending V10 |
| Product chain | Config → terrain → startup → ride/research → schema2 recording → inspect → recipe4 checked replay → report → batch | pending V10 |
| Planar compatibility | Historical schema1/ABI1 and planar controls retained | pending V10 |
| Determinism | Complete same-build state; N-invariance and sanitizers if CPU pool is selected | pending V09/V10 |
| Headless performance | Valid runs, at least one warmup and three measured ≥30 s runs; median RTF ≥2.0 | pending V09/V10 |
| GUI performance | Real display, final model/terrain at 1×; achieved average RTF ≥1.0, debt/lateness/frame rate | pending V10 |
| Recording | Recording on/off measurements, final flush included in full-run time | pending V10 |
| macOS arm64 | Separate executable/provider/reference identity and evidence | pending V10 |
| Linux aarch64 | Separate executable/provider/reference identity and evidence | pending V10 |

The [performance ledger](2026-10-22-spatial-3d-performance.md) owns measurements
and CPU/engine-pool decisions. Invalid diagnostic prefixes and `--benchmark`
runs cannot close these acceptance criteria.

## Caveat ledger

| Item | Status | Required disposition |
|---|---|---|
| `tyre_calibration` | pending V10 | Document validated tyre domain and calibration evidence |
| Unsupported physical ranges | pending V10 | Enumerate limits explicitly; do not widen tolerances to obtain a pass |
| Known contract deviations | pending V00/C10 | Resolve or retain each applicable limitation with affected consumers |
| Tyre CPU pool | pending V09 | `selected` only with measured full-run benefit, exact invariance and sanitizer evidence; otherwise remove unused code |
| MuJoCo engine pool | pending V09 | Require ≥10% complete-run gain and exact state equality; otherwise `not_selected` |
| GPU feasibility | pending V11, optional | Separate memo; no dependency on completion of V10 |

No entry here records an accepted limitation on behalf of V10. Its actual
evidence and disposition remain required before final qualification.
