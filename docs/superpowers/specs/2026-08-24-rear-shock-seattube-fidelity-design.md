# Design Specification: Photo-Derived Frame, Rear Shock & Seat Tube Fidelity

**Date**: 2026-08-24
**Status**: Awaiting user review
**Scope**: Full re-derivation of suspension hardpoints and front-triangle visuals from a calibrated reference photograph of the Bulls Sonic EVO, while holding the published geometry table invariant.

---

## 1. Overview & Problem Statement

Rendered screenshots of the current MuJoCo model show the rear-shock / seat-tube region reading nothing like the real bike:

1. **Split seat tube cage.** `geom_seattube_upper` stops at Z ≈ 288 mm and two struts (`geom_seattube_strut_l/r_top/bot`) bow out to Y = ±48 mm, wrap the linkage and converge at the BB. On screen this reads as a cage tangled with the rocker and yoke. It exists to dodge a geometric conflict, not because the real frame has it.
2. **Floating shock mount.** The trunnion at P7 is carried by `geom_shock_tab`, a thin metal capsule dropped from the top tube (`export_mujoco.py:848-860`). The real frame bolts the trunnion straight into the tube-junction casting; there is no visible bracket.
3. **Wrong damper proportions.** `trunnion_overhang = 28 mm` leaves the black body protruding past its own mounting bolt like a stub, and `body_can_len = 133 mm` makes the air can too short relative to the exposed shaft.
4. **Detached piggyback.** The reservoir is offset Y = +38 mm laterally (`export_mujoco.py:1892-1893`), so it renders as a separate box beside the shock rather than a reservoir riding on it.
5. **Debug livery always on.** Yellow pivot spheres, a red rocker (`mat_linkage`), a purple yoke (`mat_yoke`) and orange sites are unconditional. There is no toggle anywhere in `export_mujoco.py` or `run_playground.py`.

Underlying all of it, measurement (§3) shows the entire linkage sits **17–24 mm too high** relative to the reference bike.

---

## 2. Invariants — Non-Negotiable

These are fixed by the user and must survive the refit unchanged. Every one of them gets an explicit assertion (§7).

| Parameter | Value |
|---|---|
| Wheel configuration | Mullet — 29" front (r = 372 mm) / 27.5" rear (r = 352 mm) |
| Rear shock | 205 × 65 mm trunnion |
| Suspension travel | 180 mm front / 180 mm rear |
| Wheelbase | 1281 mm |
| Reach | 480 mm |
| Stack | 646 mm |
| Head angle | 64° |
| Effective seat angle | 77° |
| BB drop | 22.5 mm |
| Chainstay | 447.5 mm |

---

## 3. Reference Measurement — Method & Results

The reference photograph highlights every pivot bolt in red. Those clusters are the measurement, not an eyeball estimate.

**Calibration.** Red clusters are segmented with `R > 110, R − G > 55, R − B > 55`, connected components of ≥ 25 px. The BB cluster at pixel (587.4, 733.3) is the origin; the two axle clusters (298.1, 728.2) and (1089.2, 719.6) are 1281 mm apart, giving **1.6193 mm/px**. Image +x maps to bike +X (forward), image +y to bike −Z.

**Calibration self-check.** Under that mapping the front axle lands at Z = **+22.2 mm** against a specified BB drop of **22.5 mm** — a 0.3 mm agreement from a value never used to fit the calibration. This is the check that must re-run if the photo is ever replaced.

**Extracted pivots** (mm, BB origin):

| Cluster (px area) | Photo (X, Z) | Identification |
|---|---|---|
| 145 | (−45.0, +45.9) | main pivot P0 *(candidate — see §4)* |
| 133 | (−400.4, −8.3) | Horst pivot P2 *(candidate — see §4)* |
| 87 | (−71.3, +206.4) | P3 seatstay–rocker |
| 128 | (−39.4, +191.8) | P4 rocker–yoke |
| 122 | (+41.9, +185.0) | P5 rocker–frame |
| 51 | (+29.6, +261.6) | P6 shock lower eyelet |
| 95 | (+171.4, +401.7) | P7 trunnion (rebound knob) |
| 87 | (−468.5, +8.3) | rear axle |
| 45 | (+812.5, +22.2) | front axle |

**Current model vs photo:**

| Point | Photo | Model | ΔX | ΔZ |
|---|---|---|---|---|
| P3 | (−71.3, 206.4) | (−72.2, 222.9) | −0.9 | **+16.5** |
| P4 | (−39.4, 191.8) | (−49.8, 211.0) | −10.4 | **+19.2** |
| P5 | (+41.9, 185.0) | (+43.3, 206.1) | −1.4 | **+21.1** |
| P6 | (+29.6, 261.6) | (+26.0, 283.9) | +3.6 | **+22.3** |
| P7 | (+171.4, 401.7) | (+173.6, 426.1) | −2.2 | **+24.4** |

X agrees to within 3.6 mm everywhere except P4; Z is systematically high by 17–24 mm. Control-point noise is ±6 mm, so the Z offset is signal.

**Independent corroboration.** Measured shock eye-to-eye |P7 − P6| = **199.3 mm** against the specified **205 mm**. The red clusters are genuinely the suspension pivots.

**Feasibility probe.** Feeding the photo points straight into `HorstLinkageSolver`:

```
max_stroke @ 180 mm travel = 68.322 mm      (target 65.0)
leverage ratio 3.153 -> 2.31, progressivity 26.73%
max_link_error = 2.8e-13                     (four-bar closes exactly)
```

The photo geometry is a valid, well-conditioned four-bar and misses the stroke target by only 5%. The refit is a correction, not a redesign.

**Limits of the source.** The white front triangle photographed against a white background cannot be segmented reliably — gloss highlights break the mask. Dark members in the linkage region (shock, rocker, yoke, stays) are all black and mutually occluding. Therefore only pivot points, the seatpost axis, the shock axis and the wheels are *measured*; front-triangle tube shapes are *authored* against anchors and must be labelled as such.

**Measured non-pivot geometry:**

- Seatpost axis: 72–73° actual, through (−83, 430) and (−146, 624).
- Seat tube: straight from (−83, 430) down to (−30, 284), ~70°, terminating at Z ≈ 258 where the silhouette width steps from 29 mm to 68 mm (X −35…+33) as the casting takes over.
- Down tube / battery box: X 150…266 at Z = 255; X 208…305 at Z = 323.
- Shock along its axis (unit vector P6→P7 = (0.712, 0.703)): exposed shaft ≈ 66 mm, air can ≈ 145 mm, trunnion overhang ≈ 12 mm. Sum 211 mm reconciles with e2e 205 + overhang 12.

---

## 4. Reference Extraction (Implementation)

`tools/photo_reference.py`, committed so the numbers are auditable:

- auto-detects the red clusters with the thresholds above;
- calibrates px→mm from the BB and axle anchors;
- asserts the BB-drop self-check (22.5 ± 1 mm) and fails loudly otherwise;
- emits `docs/reference/bulls_reference_points.json`.

**Ambiguity resolution.** Two identifications are uncertain: the (−45.0, +45.9) cluster is either the main pivot P0 or a motor mount bolt, and (−400.4, −8.3) is either the Horst pivot P2 or a derailleur hanger bolt. These are **not** resolved by eye. Candidate assignments are enumerated and scored by whether they produce a closed four-bar with plausible travel; the probe in §3 already shows one consistent assignment at `link_error = 2.8e-13`.

---

## 5. Constrained Refit

- **Free:** P0, P2, P3, P4, P5, P6, P7, P12.
- **Hard:** the entire §2 table.
- **Objective:** minimise summed squared distance from each fitted pivot to its photo target.
- **Binding equality:** `solve_trajectory(max_travel=180).max_stroke == 65.000 ± 0.01`. Currently 68.322. The optimiser distributes the correction across links rather than hand-tuning a single length.
- **Chainstay conflict:** the photo implies 468.5 mm, the spec mandates 447.5 mm. **Spec wins.** The rear group (P2, P12, P1) translates forward 21.0 mm as a rigid body, preserving the photographed linkage shape while placing the axle where the spec requires.

Outputs: new seed hardpoints in `bike_geometry.py` / `HorstLinkageSolver` defaults, regenerated `coordinates.json`, new leverage and progressivity figures.

---

## 6. Visual Rebuild

**Seat tube.** The measured silhouette explains the real frame: the seat tube simply does not reach the BB. It runs straight from the collar and dies into the casting at Z ≈ 258.

- delete `geom_seattube_strut_l_top`, `geom_seattube_strut_l_bot`, `geom_seattube_strut_r_top`, `geom_seattube_strut_r_bot`;
- `geom_seattube` becomes continuous from P10 down to Z ≈ 258;
- add `geom_frame_yoke`, the motor / shock-tunnel casting below Z ≈ 258 and forward of the rocker;
- the rocker keeps straddling it at Y = ±45 mm — `geom_roc_arm_*` is already correct and is not touched.

**Trunnion.** Delete `geom_shock_tab`. Add `geom_frame_junction`, a casting at the top-tube / seat-tube / down-tube meeting point that contains P7, so the trunnion bolts into frame material.

**Damper.** `body_can_len` and `trunnion_overhang` stop being literals and derive from spec:

- `body_can_len = e2e − stroke − 2 mm` (= 138 mm). The rule guarantees the can never passes the lower eyelet at full bottom-out, where e2e collapses to 140 mm.
- `trunnion_overhang = 14 mm` (photo ≈ 12 mm).
- Piggyback offset moves into the frame plane: ≈ 34 mm perpendicular to the shock axis, ≈ 8 mm lateral, replacing the current Y = +38 mm.

**Down tube.** Replace the r = 28 mm capsule with a rounded box matching the measured battery-box envelope.

**Debug livery.** Move pivot markers and `mat_pivot_yellow` / `mat_linkage` / `mat_yoke` / debug sites into a group behind a flag. Default off for hero renders (frame in reference colours), on for kinematic debugging. Without this, visual comparison against the photo is meaningless.

---

## 7. Verification

- `tests/test_kinematics.py`: updated LR / progressivity expectations; `max_stroke @ 180 == 65.00 ± 0.01`; **explicit assertions on every §2 invariant** so a future refit cannot silently break the published geometry.
- New fidelity test: each fitted pivot within **8 mm** of its photo target — chosen above the ±6 mm control-point noise floor established in §3, so the test fails on real drift rather than on measurement scatter.
- New clearance test: the air can does not intersect the lower eyelet at full bottom-out (the `e2e − stroke − 2` rule).
- `tests/test_playground.py`: geom counts change (−5 deleted, plus new castings).
- Render script: stand mode from a camera matching the reference viewpoint, emitting a side-by-side PNG, so similarity is judged on one frame rather than from memory.
- Air-spring re-tune to restore 30 % rear sag on the new LR curve, with a sag assertion.
- Regenerate `coordinates.json` and the README tables.

Python is executed via `uv` per `AGENTS.md`.

---

## 8. Risks & Open Items

1. **Front-triangle shapes are authored, not measured** (§3). They must be marked as styling in code comments so later work does not mistake them for reference data.
2. **P0 / P2 identification** is resolved by four-bar consistency, not by direct evidence. If no assignment closes cleanly, this returns to the user as a question.
3. **Chainstay divergence of 21 mm** between photo and spec is resolved in favour of spec, so the rear axle will not sit exactly where the photo shows it. This is a deliberate, user-directed trade.
4. **Leverage curve changes** (3.173 → 2.489 becomes roughly 3.153 → 2.31), which shifts damper behaviour. Sag is re-tuned; ride feel will differ from the current build.

---

## 9. Out of Scope

Drivetrain, brakes, cockpit and wheel cosmetics; rider model; track/obstacle generation; any change to the physics integration scheme.
