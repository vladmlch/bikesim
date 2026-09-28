# SDD ledger — plan: docs/superpowers/plans/2026-09-28-03-tire-contact.md

## Target and preflight

- Phase depends on A1-A2 and B2; it follows completed/reviewed phases 01 and 02.
- Spec TIRE-01..06 is binding; this phase introduces the `native_reference` and `compliant_2d` backends explicitly, keeping the current legacy `sphere`/`pneumatic` selection separate.
- All values stay SI internally. Preserve synthetic provenance; no parameterized pass is described as experimental calibration.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| C1 / C2 | `sim/ride/contacts.py`, `controller_grounded` | C1 owns raw physical contacts; C2 owns only a timestamp-based controller filter. Keep load and filter state separate. |
| C1 / C5 | `ContactPatch`, point velocity, `TireForceApplier` | C5 consumes the immutable snapshot and absolute wheel-point velocity from C1. |
| C2 / C5 | grounded state and backend | C5 computes raw physical contact independently; C2's bool may gate only controllers, never tire force. |
| C3 / C4 | `physics/tire.py` | C4 extends the same pure-law module after C3; preserve one force/energy convention and test both laws. |
| C3 / C5 | radial `normal_contact` and tire config | C5 consumes C3's SI radial law and must not infer material stiffness from native solver parameters. |
| C4 / C5 | brush state, contact frame, force application | C5 owns persistent per-wheel brush state, frame transport and release loss; C4 remains a pure return-map. |
| C1 / C6 | raw Fn, world force, wheel angular speed | C6 uses normal load and signed absolute wheel angular speed, not a vertical projection or relative hinge speed. |
| C5 / C6 | physical step, accumulator, external resistance | Both register separate named contributions in the A2 accumulator; Crr and drag remain distinct from internal hinge/bearing losses. |

## Task self-consistency scan

| Task | Result |
|---|---|
| C1 | Frozen arrays and point velocity are specified. Spec TIRE-01 additionally requires interval ID, backend, moment about wheel axis, effective radius, and loaded-contact state; add these explicit snapshot fields/properties before marking C1 done. |
| C2 | Idempotence on an equal timestamp and time-based hold agree with the spec. Reset on simulation reset; catch-plane remains outside the filter. |
| C3 | The unilateral radial force and spring energy are consistent. Keep calibration data/provenance and native solver parameters separate; expose/consume an explicit `TireSpec` rather than hidden defaults. |
| C4 | Return-map equation and discrete work identity agree with TIRE-03. Confirm force sign, loss on Fn/contact disappearance, and nonnegative energy in load-drop and reversal cases. |
| C5 | Profile geometry and `mj_applyFT` are the sole compliant force path; maintain zero native wheel-terrain contacts in compliant mode and no secondary wheel-hinge torque. Resolve/specify the significance threshold for `multi_support` from the spec before labeling V2. |
| C6 | The provided `rolling_moment` always returns a negative scalar using `abs(omega_abs)`; for reverse wheel rotation this can inject energy. Ruling: use signed angular velocity in the odd/passive law and add a reverse-spin assertion. Drag must remain in X-Z; reject nonzero lateral velocity or otherwise it would violate the planar model. |

## Recorded rulings

- Ruling: extend C1 snapshot to meet every TIRE-01 field, with additive defaults/adapters where legacy callers construct a snapshot. — The spec lists telemetry fields absent from the plan's minimal dataclass. — Cost if wrong: telemetry contract needs another migration in F1/F4.
- Ruling: use signed `omega_world_about_axis` (or equivalent) in rolling torque and verify `tau*omega<=0` for both directions. — The plan's name `omega_abs` and formula lose the rotation sign, conflicting with passivity. — Cost if wrong: reverse rolling resistance could drive the wheel.
- Ruling: keep drag force planar and add a direct contract test. — The project explicitly excludes lateral dynamics. — Cost if wrong: a nonzero Y force could enter unmodelled DOFs.
- Active workstream `#1153`: C5 must also fix freshness in existing pneumatic paths, adding `mj_forward` before tyre force application in ride stepping and equilibrium relaxation; retain regression tests for current legacy pneumatic behavior.

## Task ledger

- C1: fix round 1/5 (2 addressed, 0 open; commits `d54b060..37f292d`); scoped re-review accepted exact native force and catch-plane classification.
- Ruling: preserve the full copied native `world_force_n` per patch independently from projected X-Z normal/tangent decomposition, and classify working-road vs emergency catch-plane contacts. Keep existing legacy bridged loads unchanged; expose road-only loaded contact for C2/physical drive gating. — TIRE-01 requires exact world resultant and the catch-plane rule forbids it from re-enabling drive as road; the planar model still requires separate X-Z force components. — Cost if wrong: altered contact wrench diagnostics or physical drive reactivation after an off-road fall.
- Task C1: minor (deferred): the native and pneumatic snapshot adapters have a small duplicated construction pattern; not a behavior defect.
- Ruling: keep legacy load/support/contact properties, HUD/recorder, and pneumatic behavior as compatibility outputs; add separate `front_controller_grounded` / `rear_controller_grounded` time-filtered booleans from raw working-road contact and route only physical native-reference controller gates through them. C2 may extend to the narrow `RideSimulation`/`CruiseController` call sites; physical force, Crr, raw load/support, airtime metrics, and legacy consumers remain unfiltered. `TerrainContactQuery.reset()` resets the filter state. — TIRE-01 requires a bool-only controller filter and raw physical metrics, while legacy bridge outputs remain a compatibility contract. — Cost if wrong: either the filter remains unused in physical control, legacy CSV/gating drifts, or filtered state contaminates forces/airtime.
- Ruling: derive raw controller-grounded input from working-road normal load above the existing `CONTACT_LOAD_THRESHOLD_N=1.0 N`; geometric-only rows and catch-plane loads are not grounded. The timestamp filter can bridge the bool but never synthesize load. — This preserves the existing solver-noise/margin threshold. — Cost if wrong: insignificant contact rows can spuriously engage native-reference control.
- Ruling: preserve legacy recorder/summary contact columns during C2, but physical airtime remains unresolved until F1/F4 records raw `road_loaded_contact` and threshold sensitivity in physical channels. Bridged legacy fields are not accepted physical airtime evidence. — The lead plan maps TIRE-01 to C1-C2/C5/F1 and DATA-02 to F1/F4; C2 owns controller filtering, F1/F4 own physical telemetry/schema. — Cost if wrong: controller debounce is misreported as measured physical airtime.
- Task C2: minor (deferred): update `front_load_n` documentation to clarify it is a legacy bridge/gate; physical native cruise uses `front/rear_controller_grounded`.
- [x] C2: complete (commit `ec648a3`, review approved; one minor deferred). Focused tests 12 passed; full suite 897 passed plus four known CoreGraphics renderer setup errors.
- [ ] C3: in progress; BASE `ec648a3`; brief `.superpowers/sdd/2026-09-28-03-tire-contact/task-C3-brief.md`.
- [x] C1: complete (commits `1fcd925..37f292d`, review clean with one fix round). Initial focused tests 57 passed; full suite 882 passed plus four known CoreGraphics setup errors; fix-round focused tests 60 passed.
- [ ] C2: in progress; BASE `37f292d`; brief `.superpowers/sdd/2026-09-28-03-tire-contact/task-C2-brief.md`; consume `WheelContactSnapshot.road_loaded_contact` for physical controller-grounded input and leave load/force unfiltered.
- [ ] C2: pending C1
- [ ] C3: pending after C1-C2
- [ ] C4: pending C3
- [ ] C5: pending C1-C4
- [ ] C6: pending C1/C5
- [ ] Gate C review: pending
