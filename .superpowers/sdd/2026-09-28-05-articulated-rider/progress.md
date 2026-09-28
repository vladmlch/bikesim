# SDD ledger — plan: docs/superpowers/plans/2026-09-28-05-articulated-rider.md

## Target and preflight

- Phase follows B1-B2, C1/C3-C4, D1-D4; implementation has not started.
- Current source already supports `RiderSpecs(variant="seated", legs="articulated")` and hip/knee/ankle leg chains welded to moving pedals. It remains the old seated rider anchored to the bicycle. New `articulated_planar` must be a separate independent-root variant and must not repurpose or silently alter that legacy path.
- The plan explicitly forbids invoking the legacy static CG/load-path builder for the new variant before compilation; use geometry-only pose, compiled CoM, and E3 supports.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| E1 / E2 | `RiderSpecs`, segment names/masses | E2 consumes fixed anatomical budgets from E1; support shares remain pose/control inputs only. |
| E2 / D1 | pedal bodies/sites and initial pose | E2 requires D1's physical pedal sites; reuse existing pedal topology where possible and adapt new root pose after crank phase is resolved. |
| E2 / E3 | body names and worldbody root | E3 resolves the independent pelvis/limbs built by E2; do not attach the pelvis under the frame. |
| E3 / C1/C3/C4 | point kinematics, unilateral normal force, brush friction | E3 uses shared contact conventions and point velocities; a rider contact cannot use hidden ties or a tire-only backend. |
| E3 / A2 | force accumulator and per-subsystem components | Internal reactions are accumulated as one named rider contribution and remain equal/opposite at a shared point. |
| E3 / E4 | support states and foot/pedal work | E4 may act through a foot only while its unilateral contact is active; release clears tangential spring state with recorded loss. |
| E4 / D4 | human effort, crank, mid-drive | Articulated joint work is the human source; no second `human_crank` actuator, while motor work remains distinct. |
| E2/E3/E4 / F1-F2 | compiled masses, internal work, total angular momentum | F1/F2 must include all rider bodies and force pairs before the articulated mode is released. |

## Task self-consistency scan

| Task | Result |
|---|---|
| E1 | Fractions sum anatomical segments once, including helmet once; preserve legacy `seated_path_masses` for old variant and verify invariance to support-share changes. |
| E2 | Topology test agrees with RIDER-02. Physical mass/inertia are separate from render geoms; create one worldbody root and do not solve equilibrium before E3. |
| E3 | Same-point reactions conserve net world force/moment. `compute_qfrc` may retain state for compliant support; reset/release must report stored-energy loss. |
| E4 | Stance waveform has the declared mean over both legs; cap the combined feed-forward and PD torque, and compute delivered work from actual joint actuator forces/speeds. |

## Recorded rulings

- Ruling: preserve current `seated` plus articulated-legs path as legacy; introduce `articulated_planar` independently. — Current branch already has a different articulated implementation. — Cost if wrong: break existing welds, rider mass accounting, or golden baselines.
- Ruling: no wheel/crank generalized input from rider root stabilization; all pose control uses internal joint actuators. — SYS-01/RIDER-04 forbid external pitch or root force. — Cost if wrong: create angular momentum and invalid energy results.

## Task ledger

- [ ] E1: pending Phase 04 gate
- [ ] E2: pending E1 and D1
- [ ] E3: pending E2/C1/C3-C4/A2
- [ ] E4: pending D1-D4/E3
- [ ] Gate E review: pending

