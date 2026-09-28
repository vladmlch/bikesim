# SDD ledger — plan: docs/superpowers/plans/2026-09-28-01-core-suspension.md

## Target and preflight

- Starting BASE: `03bede8`; source tracked state is clean. Do not stage/commit the user-owned untracked plans, specs, or `.claude/skills/`.
- The specification is reachable and authoritative. Phase gate A covers SYS/SUS requirements; all units are SI internally and legacy behavior stays separately reproducible.
- Ruling: execute in the supplied `pedals` checkout. Git metadata is read-only in this sandbox, so no linked worktree can be created without changing the requested target branch/workspace.
- The bundled `task-brief` helper only recognizes numeric headings (`Task 1`), while plans use IDs such as `Task A1`. Preserve its implementation and generate equivalent per-task brief files by extracting each complete lettered section.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| A1 / A2 | `RideSimulation`, `ride_sim.py` step | A2 consumes A1's resolved config and extends its physical step with force accumulation; implement A1 before A2. |
| A1 / A3 | `RideSimulation`, `ride_sim.py` construction | Separate config and suspension-factory concerns; A3 preserves the keyword-only config contract. |
| A2 / A5 | `sim/ride/forces.py`, `RideSimulation`, named force contributions | A5 must expose separate stop/bumper force vectors and register them as distinct accumulator entries; a single total suspension vector would lose SYS-04 attribution. |
| A5 / MJCF builder | physical `shock_stroke` range and limit validation | Current `rear_linkage.py` compiles the joint range exactly `[0, working_stroke]`; both top-out below zero and upper emergency stop beyond travel are unreachable. Extend only the physical range and let the force-path validator accept the configured overtravel. At zero speed the bumper-to-stop handoff must preserve static force and stored energy; damping on contact must remain passive, without stacking two progressive springs. |
| A3 / A4 | `physics/damper.py`, `legacy_behavior` | A3's factory passes a flag introduced by A4. Ruling: A4 precedes completion of A3. |
| A3 / A5 | coil configuration, `legacy_behavior` | A3's factory passes a flag introduced by A5. Ruling: A5 precedes completion of A3; A5 remains after A2. |
| A3 / A6 | active suspension factory and static equilibrium | A6's real evaluator consumes A3's factory; A3 precedes A6. |
| A6 / current CLI | `_fit_sag` and `compute_suspension_tuning_for_sag` | A6 produces the real equilibrium fitter; F4 owns mode-aware CLI wiring so physical `--sag` uses it while legacy behavior remains explicit. |

## Task self-consistency scan

| Task | Result |
|---|---|
| A1 | Config defaults and legacy routing agree with SYS-01. Add a regression assertion for the specified temporary `NotImplementedError` effort-mode guard. |
| A2 | Additive copied contributions and explicit external input agree with SYS-03. Preserve old legacy `apply` behavior and verify no force carries across consecutive steps. |
| A3 | Factory fields agree with BikeSpecs. The text says nine settings, but its mapping and current public specs enumerate eight damper controls; test all eight and do not invent a ninth API. |
| A4 | Shared HBO and continuous Firm equations agree with SUS-01; validate the configured HBO interval so the travel denominator cannot be zero. |
| A5 | Compression spring, bumper, top-out and emergency solver limit are separate under SUS-04; existing joint range and force component ownership require additional integration files beyond the plan's list. Physical geometry must agree with any injected CoilShock stroke; do not rebuild/drop an injected subclass. |
| A6 | Kinetic-energy reflected mass and bounded log-space sag fit agree with SUS-03/05; retain the existing equilibrium solver and do not write target travel into qpos. |

## Recorded rulings

- Ruling: run A1, A2, A4, A5, A3, A6. — A3 consumes compatibility flags produced by A4/A5, and A5 consumes A2's accumulator. — Cost if wrong: localized task-order rework; the public phase sequence and interfaces remain unchanged.
- Ruling: parameterize the eight declared damper controls in A3. — The actual `BikeSpecs`/`DamperClickConfig` mapping names eight controls; `preset_name` is not a damper click/physical override. — Cost if wrong: one missing test if a ninth supported field is later identified; no speculative config is added now.
- Ruling: keep per-task briefs and reports in this plan's ignored SDD workspace; use explicit paths in every dispatch.
- Ruling: in physical mode, disable/clear the legacy stabilizer before force collection, then let `ForceAccumulator` own `qfrc_applied`; never zero the pitch DOF after collection. — A1's `disable(data)` clears that DOF, but chain/rider writers may legitimately contribute there. — Cost if wrong: physical internal moments on the bicycle root are silently lost.
- Ruling: clear prior-step `qfrc_applied` and `xfrc_applied` before `mj_forward`, then add only this step's explicit `external_qfrc` and computed writer contributions. — MuJoCo's forward/contact solve consumes retained applied wrenches; pneumatic tyre `xfrc_applied` otherwise leaks the previous wheel force into the new snapshot. — Cost if wrong: stale load changes current contact/constraint forces; if the explicit input is not preserved, current external work is lost.
- Ruling: A5 may extend its file scope to `mujoco/rear_linkage.py` and the narrow builder plumbing needed to set a physical-only emergency travel range; keep the existing range in legacy. It must also register coil, bumper, top-out, and solver-limit vectors as distinct accumulator channels. — Otherwise the upper stop is unreachable and SYS-04 cannot tell stop storage/work apart. — Cost if wrong: hard solver limit masks or duplicates the intended physical stop.
- Ruling: route `legacy_behavior=True` through every legacy simulation factory while keeping the standalone damper corrected by default; route false for physical mode. — A4's new default had leaked into the unconfigured legacy ride and changed `legacy-v1`. — Cost if wrong: the promised legacy numerical baseline is no longer reproducible.
- Ruling: keep A6's implementation on its listed tuning/fitter files and route the physical `--sag` option in F4, which owns CLI mode/config resolution; legacy `--sag` retains its historical analytical path until that mode-aware boundary exists. — The current CLI has no physics mode, while F4 introduces it and owns the CLI contract. — Cost if wrong: if F4 does not wire physical `--sag` to A6, the user-facing physical fit remains analytic; carry this as a release gate.
- Ruling: resolve the conflicting A6 CLI addenda in favor of the earlier A6 addendum and the 06/F4 mode-aware CLI boundary. A6 delivers the equilibrium fitter; F4 routes physical `--sag` to it, keeps legacy analytical, and verifies <=0.5 mm achieved travel error. The later addendum's demand to wire CLI inside A6 would create a partial route before F4 introduces the selector and resolved configuration, and B1 mass-spec plumbing is not available yet. — SUS-05 requires the final physical CLI behavior but does not assign it to A6; F4 explicitly owns the CLI files and runs after mass configuration is introduced. — Cost if wrong: physical `--sag` remains analytic until F4; preserve it as a mandatory release gate.
- Ruling: A5 extends scope to `model_config.py`, `mujoco/builder.py`, `mujoco/rear_linkage.py`, and `ride_sim.py` as required to make physical stops reachable and separately accumulated; legacy joint ranges and writer remain unchanged. The physical synthetic reference is 7000 N at 10 mm (derived from the existing 7000 N/10 mm bumper scale), with 500 N*s/m stop damping and 10 mm permitted overtravel at each end; `k_stop=F_ref/delta_ref=700000 N/m`, provenance `synthetic`. Preserve static force and stored energy at the zero-rate bumper-to-upper-stop boundary; the separate damping contribution may engage on overtravel if passive, with no overlapping progressive spring. — The plan supplies the force/deformation derivation but no production reference values or range plumbing. — Cost if wrong: stop stiffness/damping changes extreme impact and travel results; these remain uncalibrated V1 values.
- Ruling: reject a conflicting explicit CoilShock working stroke before compilation and construct defaults from BikeSpecs; preserve a supplied custom CoilShock instance/subclass rather than reconstructing a base object. — MJCF range is built from BikeSpecs while stop thresholds come from CoilShockSpecs; rebuilding discards caller-defined behavior. — Cost if wrong: stop engages at the wrong stroke or caller overrides silently disappear.
- Ruling: capture native `shock_stroke` joint-limit force separately from explicit `shock_upper_stop`, after MuJoCo solves the step. — The hard range constraint is a distinct emergency force not present in `qfrc_applied`. — Cost if wrong: a hard-limit impact is invisible to SYS-04 diagnostics and energy accounting.
- Task A1: minor (deferred): separate legacy and physical numerical reference/golden results are a release-validation obligation for F2/F4, not produced by A1; carry this to the final review gate.
- Task A1: minor (deferred): `SimulationPhysicsConfig` defaults physical `timestep_s` to 0.0005, which conflicts with detailed pneumatic's required 0.00025 unless callers set it explicitly; F4 must choose or report a compatible resolved value.
- Task A2: minor (deferred): `SuspensionForceApplier.apply` docstring says `model` is unused, but its delegate `compute_qfrc` reads `model.nv`; carry this wording correction to final review triage.
- Baseline gate: `uv sync --locked` succeeded after redirecting the uv cache to `/private/tmp`; Python 3.12.13, MuJoCo 3.12.0. `uv run --locked pytest -q` produced 595 passed and 4 setup errors (210.21s), all in `tests/test_render_comparison.py` at MuJoCo CGL initialization: `invalid CoreGraphics connection`.
- State: user directed to proceed with the four baseline renderer errors recorded; no source changes before A1 dispatch.

## Task ledger

- [x] A1: complete; commits `03bede8..b69c674`, fix round 1/5 resolved and scoped re-review accepted; focused tests 48 passed; full suite before final kinematics change 604 passed plus same four CGL baseline errors
- [x] A2: complete; commits `b69c674..a60df5e`, review clean; focused tests 49 passed; full suite before final aliasing fix had 614 passed and same four baseline CGL errors
- [x] A4: complete; commits `e711c5a..fff698c`, fix round 1/5 resolved and scoped re-review accepted; focused integration 183 passed; full suite before fix 697 passed plus four CGL errors
- [x] A5: complete; commits `fff698c..634a834`, fix round 1/5 resolved and scoped re-review accepted; focused 107 passed plus 28 direct rerun; full suite before fix 718 passed plus four CGL errors
- [x] A3: complete; commits `634a834..8111d84`, fix round 1/5 resolved and scoped re-review accepted; focused related tests 137 passed plus fix-round 52 passed; full suite before final legacy/finite fixes 752 passed plus four CGL errors
- [x] A6: complete; commits `8111d84..d896d57`, review approved under recorded scope ruling. Focused tests 57 passed; full suite 781 passed plus the same four baseline CoreGraphics setup errors. Physical `--sag` CLI routing and <=0.5 mm achieved-sag check remain mandatory in F4.
- [x] Gate A review: complete. Physical mode rejects pitch assist and disables/clears the legacy stabilizer before accumulation; legacy constructs its controller and coil with `legacy_behavior=True`; physical writers are isolated and copied into named accumulator components before one total assignment; corrected HBO behavior is exercised in Open/Firm while legacy forces are pinned to the prior model; A3 config tests cover the eight mapped public controls; A6 evaluator reads equilibrium output and is not a per-step force writer. Evidence: `model_config.py`, `ride_sim.py`, `force_accumulator.py`, `damper.py`, `suspension_config.py`, `sag_fit.py`, `tests/test_damper_physics.py`, and the respective reviewed A1-A6 test reports.
