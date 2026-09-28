# SDD ledger — plan: docs/superpowers/plans/2026-09-28-02-mass-inertia.md

## Target and preflight

- Phase depends on completion/review of Phase 01 (A1-A6); source changes have not started.
- Specification is reachable and authoritative. Preserve current geometric hardpoints, axle positions, and linkage lengths when changing physical masses.
- Plan A1 leaves `BikeMassSpecs` mutable and exposes its physical fields; current fields total 15 mass budgets plus two wheel distribution fractions. B1's “remaining overrides” must cover all actual mass fields, not those fractions as though they were masses.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| B1 / B2 | `steering_fork.py`, `drivetrain.py`, front/rear wheel mass | B1 registers component geometry masses; B2 makes wheel geoms visual-only and transfers the same wheel budget to one explicit body inertial. Ensure exactly one physical mass owner after B2. |
| B1 / A1 | `RideSimulation`, MJCF builder | B1 adds keyword-only `mass_specs` alongside A1's `physics_config`; both values must reach one model build without changing legacy defaults. |
| B1 / E1-E2 | component mass registry / compiled CoM | Later rider bodies must not be registered as frame mass or duplicated in `BikeMassSpecs`. |
| B1 / D1 | crank, cassette and wheel mass | B1 establishes the budget; D1 may split bodies only while preserving total component mass and using B2's inertia convention. |
| B1 / F2 | `compiled_center_of_mass` and benchmark | F2 validates the compiled output; do not substitute the analytic unloaded CoM. |

## Task self-consistency scan

| Task | Result |
|---|---|
| B1 | Compiled total test matches `BikeMassSpecs.total_bike_mass` with `rider="none"`; implementation must include every mass-bearing geom exactly once, exclude zero-mass debug geoms, and leave wheel-specific inertial ownership to B2. `model.body_mass` includes world body mass zero, so sum is valid. |
| B2 | Ring tensor example and fixed-axis alpha test agree with the stated +Y convention. For each compiled body compare the full tensor after `body_iquat` rotation, not sorted principal moments. Keep the 75/25 profile labelled synthetic. |

## Task ledger

- B1: fix round 1/5 (2 addressed, 0 open; commits `d809059..9aa8cb3`); scoped re-review accepted both the tiny-positive budget and provenance fixes.
- [x] B1: complete (commits `d896d57..9aa8cb3`, review clean; one fix round). Initial focused tests 61 passed; fix focused tests 78 passed; full suite at initial commit 829 passed plus four known CoreGraphics setup errors.
- Task B1: minor (deferred): compare compiled axle/hardpoint world positions across default and mass-overridden models; existing tests exercise geometry but do not compare compiled builder output.
- Task B1: minor (deferred): `BikeMassSpecs` docstring still claims compiled MJCF mass 24.35 kg while the new physical contract is 24.40 kg.
- Ruling: the current `BikeMassSpecs` component budgets have no supplied measurement dataset; expose the `synthetic` provenance by stable component ID for the physical budget so F4 can preserve it in resolved metadata. The spec requires provenance and forbids treating synthetic parameters as measured. Cost if wrong: physical outputs could be mistaken for calibrated mass data.
- Ruling: B1's physical wheel geom mass assignment is temporary until B2 converts each wheel to a single explicit inertial owner; preserve exactly one owner after B2. Cost if wrong: wheel mass could be duplicated or dropped at the phase boundary.
- Baseline note: the four reported `test_render_comparison.py` CoreGraphics setup errors match the already accepted 595-pass baseline and are not assertion failures; full-suite success must remain reported as 829 passed plus four environment setup errors, not green.
- [ ] B2: in progress; BASE `9aa8cb3`; brief `.superpowers/sdd/2026-09-28-02-mass-inertia/task-B2-brief.md`.
- Ruling: B2 may extend to `mujoco/builder.py`, `mujoco/rear_linkage.py`, and the B1 mass-contract test to plumb one shared wheel profile and let the component registry account for each explicit wheel inertial as its sole mass owner. The B1 registry validates geom groups, while B2 removes wheel geom masses; without this narrow extension, the builder would reject or omit wheel mass. Cost if wrong: mass can be dropped/duplicated or the physical build can fail at assembly.
- Verified cross-task item: `RideSimulation.physics_revision` is already `legacy-v1` vs `physical-v1`, and `tests/test_physics_config.py` asserts they differ; B2 does not alter this A1 contract.
- Baseline review note: the four image-comparison setup errors remain the same sandbox `invalid CoreGraphics connection` seen at baseline; image rendering is not validated, but this is not a B2 code finding.
- B2: fix round 1/5 (1 addressed, 0 open; commits `af53548..1fcd925`); scoped re-review confirmed compiled mass, CoM, and body-frame tensor checks against the selected profile.
- [x] B2: complete (commits `9aa8cb3..1fcd925`, review clean with one fix round). Focused tests 89 passed; full suite at initial commit 866 passed plus four known CoreGraphics setup errors; fix-round focused tests 89 passed.
- [x] Gate B review: complete. B1 tests account for all 15 budgets and compiled total, CoM translation and telemetry; B2 tests compare each compiled wheel mass, `body_ipos`, and full body-frame tensor to the same synthetic profile, with stand acceleration/energy and decorative-geometry independence. `legacy-v1`/`physical-v1` separation remains covered by A1. Per-component budget provenance is machine-readable `synthetic`; the stale 24.35 kg docstring was corrected in B2. One B1 minor remains deferred: compare compiled axle/hardpoint world positions between default and mass-overridden models.
