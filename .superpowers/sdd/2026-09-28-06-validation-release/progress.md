# SDD ledger — plan: docs/superpowers/plans/2026-09-28-06-validation-release.md

## Target and preflight

- Phase follows the simulation components, except F1's ledger foundation is explicitly needed after A2 and must grow as later writers are added.
- No independent measured calibration data is present in the supplied package. F3 can validate data provenance and compute holdout metrics, but the model remains `synthetic`/`parameterized_unvalidated` and cannot be called V2 calibrated.
- Current CLI already has `--speed` with `default=None` and separate legacy `--drive-mode` values (`motor`, `pedal`, `pedelec`). Add the physical `--physics`/`--drive` surface and keep legacy parsing/defaults intact; do not overload old meanings. F4 must route physical `--sag` to A6's equilibrium fitter while leaving legacy `--sag` compatible.
- Active workstream `#1153` requires drive parameters in the result-directory identifier; resolve all drive settings before building `run_dir`, and prove two differing drive configs do not collide.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| F1 / A2 | force accumulator, physical step, `RideSimulation` | Establish pre-step immutable force/qpos/qvel sample immediately after A2; later tasks register new components through the same accumulator. |
| F1 / C5/C6 | tire, Crr, aero powers and stored energy | Add each named component once, distinguishing internal storage, external signed work, and dissipation. |
| F1 / D2-D6 | chain/freehub/motor/battery/brake work | Mechanical ledger and electrical battery ledger remain separate; freehub release and motor work need explicit channels. |
| F1 / E3-E4 | rider contact reactions and joint work | Include internal paired forces and actual joint actuator work; no external root stabilization. |
| F1 / F4 | sample schema and legacy adapter | Physical rows use schema v2; legacy readers remain schema v1 through an explicit adapter. |
| F2 / component phases | every physics backend and drive/rider mode | Suite has explicit required cases and records exceptions as failures, never silent skips. |
| F3 / F4 | calibration state and config hash | F4 reports only a real holdout-backed calibrated status; absent data remains unvalidated. |
| F2 / F4 | validation output, release CLI | F4 exposes/labels only evidence actually produced by F2; a runner exit code is not a completed test report. |

## Task self-consistency scan

| Task | Result |
|---|---|
| F1 | `ForceSample` copies force and state arrays. Add finite/time/shape validation and retain enough interval timing to integrate work at every substep even when output is decimated. `system_momentum` must use the compiled inertial frame and all physical bodies. |
| F2 | Isolated radial rig has the correct sign convention (`delta=-qpos`, force along +Z) and compares static deflection independently. Native reference is diagnostic, not a compliant-model pass. |
| F3 | Dataset split is experiment-level and synthetic data is blocked by default. Enforce strict units/source/date/uncertainty/conditions in addition to the sample snippet's basic shape checks. |
| F4 | Resolve schema defaults → TOML → explicitly supplied CLI values; reject `--speed` in coast/effort, synchronize all initial absolute wheel/rider speeds, and hash canonical resolved config plus actual terrain. Keep legacy args and schema intact. |

## Recorded rulings

- Ruling: start F1's core sample/energy/momentum APIs after A2, then extend their component coverage in C5/C6, D2-D6 and E3-E4. — The lead plan says F1 begins after A2, while the phase table places all of F1 last. — Cost if wrong: later force writers lack an exercised observation contract and would need broad late integration changes.
- Ruling: `schema_version=2` is used for physical output; v1 stays behind the legacy adapter. — DATA-01/02 require versioned output and no silent relabeling. — Cost if wrong: old readers would misinterpret physical channels.
- Ruling: do not claim V2 without an independent real holdout report. — The package supplies plan checks but no measured samples. — Cost if wrong: fabricated calibration provenance.
- Task F1: minor (deferred): F4 must persist `schema_version` and serialize per-component generalized-force vectors as well as power/work to satisfy SYS-04; F1 currently holds vectors in memory only.
- Task F1: minor (deferred): preserve one recorder callback per physical step regardless of decimation; current CLI satisfies this, and F4 must retain the contract for all new run loops.

## Task ledger

- [x] F1: complete; commits `a60df5e..e711c5a`, review clean; focused tests 34 passed; full suite 622 passed plus four known CoreGraphics errors
- [ ] F2: pending component phases
- [ ] F3: pending; no real dataset supplied
- [ ] F4: pending F1-F3 and release gates
- [ ] Coverage matrix and broad review: pending
