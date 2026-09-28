# SDD ledger — plan: docs/superpowers/plans/2026-09-28-physics-correctness.md

## Target and baseline

- Target checkout: `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2`, branch `pedals`, starting HEAD `03bede8`.
- Specification base `70623815b98788018bcdbd8eef347d778f9bb3f3` is an ancestor of HEAD; the plan is applied to current source, not assumed to match every file at the old revision.
- Tracked source starts clean. Preserve user-owned untracked inputs: `.claude/skills/`, all supplied plan/spec documents, and the unrelated uphill-climb plan/spec.
- Ruling: stay in the supplied `pedals` checkout. `.git` and common git metadata are read-only under this sandbox; creating a linked worktree/branch cannot be done here and would move work away from the requested branch.
- Package README says 28 tasks / 140 checkbox steps; `plan-checks.json` reports no structural issues, while also recording that repository tests and MuJoCo simulations have not run.
- Baseline environment: `uv sync --locked` succeeded with `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928`; Python 3.12.13 and MuJoCo 3.12.0.
- Baseline command `uv run --locked pytest -q`: 595 passed, 4 setup errors in `tests/test_render_comparison.py`, 210.21s. All four fail at `mujoco.Renderer` with `mujoco.cgl.CGLError: invalid CoreGraphics connection`.
- The lead plan's baseline gate was presented to the user; user directed to proceed with the four renderer errors recorded as baseline.

## Cross-plan contract scan

| Plans | Shared file/interface | Scan result |
|---|---|---|
| 01 / 02 | `RideSimulation(physics_config, mass_specs)`, MJCF builder | 02 adds `mass_specs` to the API from 01; keyword-only extension can preserve the 01 config contract. |
| 01 / 03 | `ForceAccumulator`, physical step order, `RideSimulation` | 03 contact forces must be another named contribution and run after the kinematic forward pass; no second contact-state owner. |
| 01 / 04 | `SimulationPhysicsConfig.drive_mode`, `RideSimulation` | 04 consumes the guarded drive-mode API from 01; release crank effort only after D4. |
| 01 / 05 | `articulated_effort`, named generalized forces, rider builders | 05 consumes the physical mode and force ownership; keep articulated mode guarded until E4. |
| 01 / 06 | resolved config, CLI/session, energy/telemetry | 06 finalizes external configuration and release metadata after all producers exist. |
| 02 / 03 | compiled masses and compliant contact | Contact dynamics use compiled body properties; no duplicate mass/inertia assignment. |
| 02 / 04 | wheel inertia, crank/cassette mass | D1 must use the same component mass budget and explicit inertial contract from B1/B2. |
| 02 / 05 | rider segment masses and total mass budget | E1/E2 add rider bodies without retaining rider mass in the frame. |
| 02 / 06 | compiled CoM and benchmark assertions | F2 validates mass/CoM outputs produced by B1/B2. |
| 03 / 04 | contact patch, slip, force accumulator, resistance | D2/D4 consume physical contact/control signals; C6 owns external rolling/drag forces. |
| 03 / 05 | tire and rider contact force application | Both feed the same physical step/energy ledger; rider reactions must not be mixed into tire backend forces. |
| 03 / 06 | contact snapshots, telemetry, calibration | F1/F3 consume C1/C5/C6 data with explicit units/backend/provenance. |
| 04 / 05 | crank effort, chain, rider joint actuation | E4 consumes drivetrain interfaces and must not duplicate crank work. |
| 04 / 06 | battery, motor, brake and power telemetry | F1/F4 report D4-D6 delivered work and chosen drive mode. |
| 05 / 06 | rider mass, contacts, joint work, angular momentum | F1/F2 include rider bodies and internal contact reactions before articulated release. |

Every producer/consumer pair above has an explicit contract in the lead plan; implementation proceeds in its declared phase order 01→06, with local task reorderings recorded in each phase ledger when needed.

## Carried active workstream criteria

- Workstream `#1153` also requires: set the chosen crank/pedal phase before static equilibrium; refresh MuJoCo kinematics before pneumatic tyre force reads; include drive-mode parameters in the result-directory identifier.
- D1 must apply crank phase and pose before equilibrium, including articulated pedal/weld initialization.
- C5 must refresh kinematics before pneumatic tyre forces in the existing legacy step and equilibrium relaxation as well as the new physical path.
- F4 must resolve drive settings before constructing `run_dir`, and make different drive configurations produce different result identifiers (a saved hash alone is insufficient if directories still collide).
- F4 must preserve physical component-mass provenance (`synthetic` by stable component ID until measured inputs exist) in resolved configuration metadata.
- F1/F4 must report physical airtime from raw working-road contact snapshots and include threshold sensitivity; legacy bridged `front_contact`/`rear_contact` recorder fields are not physical airtime evidence.
- [ ] Workstream #1153 criteria verified: pending D1/C5/F4.

## Phase status

- [x] 01 core suspension: complete after Gate A review (A1-A6 complete; physical CLI `--sag` remains a required F4 release gate)
- [x] 02 mass and inertia: complete after Gate B review (B1/B2 complete at `1fcd925`)
- [ ] 03 tire contact: in progress (C1/C2 complete at `ec648a3`; C3 now starting)
- [ ] 04 drivetrain assist: not started
- [ ] 05 articulated rider: not started
- [ ] 06 validation and release: in progress (F1 foundation complete after A2; remaining F1 channels integrate with components)
- [ ] Broad whole-branch review and finishing workflow: not started
