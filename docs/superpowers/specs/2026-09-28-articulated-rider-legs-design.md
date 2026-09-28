# Design Specification: Articulated Rider Legs with Physical Pedal Force

**Date**: 2026-09-28
**Status**: Awaiting user review
**Scope**: Replace the rigid slide-mounted leg clusters with articulated, actuated legs (hip/knee/ankle) welded to real pedal bodies, so the rider's legs visibly pedal and physically drive the cranks — including producing real propulsion torque uphill.

---

## 1. Overview & Problem Statement

The current "seated" rider legs contain **no leg kinematics at all**. Each leg is a single rigid body — thigh, shank and foot capsules frozen in the horizontal-crank pose — riding a vertical slide joint (`rider_leg_front_z`, `rider_leg_rear_z`) backed by a one-sided spring (`k ≈ 13 028 N/m`, `c ≈ 332 N·s/m`, ~5 Hz). Per step, `_follow_cranks` (`src/bike_sim/sim/ride_sim.py:353-367`) reads `qpos[crank_spin]` and shifts the spring support point by `±crank_length·sin(phase)` in antiphase (`sim/ride/rider_forces.py:50-76,122-141`).

Consequences, all visible:

- The knee never bends — there is no knee (or hip, or ankle) joint.
- The foot tracks pedal *height* only, through compliant spring lag/overshoot. Fore-aft it is up to ±165 mm off the pedal spindle at 6/12 o'clock; laterally it is drawn at y = 0 while pedals sit at y = ±115 mm.
- The thigh's top end slides ±165 mm through the pelvis body.
- Causality is inverted: the `crank_drive` torque actuator drives the cranks (`actuators.py:71-80`); legs bob passively.

Goal (agreed with user): legs that **physically drive the cranks** — pedaling force matters dynamically, including climbing — *and* look like real pedaling. Purely cosmetic fixes were explicitly rejected (Q1 → option C; Q4: "I want pedaling force taken into account while riding, including uphill").

Two existing facts constrain the design:

1. **The leg-spring load path is a tested feature.** The slide bodies carry ~26 kg (~33% of an 80 kg rider) through the ~5 Hz spring into the frame. `tools/validate_pedals.py` check 3 *asserts* amplified shock-stroke response at 2× cadence — the "bob" is deliberate suspension-research signal.
2. **Cranks only rotate in `--drive-mode pedal|pedelec`.** In the default `motor` mode (and playground/test-stand) the crankset is welded static geometry (`mujoco/drivetrain.py:33-129`, gated at `builder.py:89-90`).

---

## 2. Settled Decisions (interview outcome)

| # | Decision | Choice |
|---|---|---|
| Q1 | Depth of realism | **Physical** — legs genuinely drive the crank |
| Q2 | Worst symptom | Everything (all artifacts above) |
| Q3 | Scope | **Legs only** — torso/arms/pelvis slides stay as-is |
| Q4 | Perf budget | ~6 extra DOF + 2 welds is well inside the ~5× real-time margin; pedaling force must be real |
| Q5 | Parameterization | **Geometry-driven** — IK computed from model dimensions, no hand-tuned poses |
| Q6 | Drive modes | Pedal modes get real legs; `motor` unchanged + opt-in `--visual-pedalling` flag |
| Q7 | Foot–pedal attachment | **Weld** (`equality`/`weld`) = clipless pedals; push *and* pull-up, no slip |
| Q8 | Force production | **Hybrid** — pedal force → joint torques via leg Jacobian, + low-gain impedance toward per-step IK pose |
| Q9 | Leg DOF | **Sagittal only** — hip/knee/ankle hinges, axis Y, one plane per leg at pedal offset y = ±0.115 m |
| Q10 | Old load path | **Migrate mass into articulated legs**; delete slide bodies; retune bob via joint compliance (fallback: keep hidden slide bodies, or add pelvis compliance) |
| Q11 | Pedals | **Real pedal bodies** on passive Y-hinges at crank-arm ends (level platforms, light damping); foot welds to platform |
| Q12 | Rigid variant | **Keep behind flag** — `legs=articulated` (default in pedal modes) vs `rigid` (today's behavior, comparison baseline) |
| Q13 | Application scope | `bike-ride` pedal|pedelec only; playground/test-stand untouched |
| Q14 | Verification | Full bar — extend `validate_pedals.py`, unit tests, rendered clip |

---

## 3. Model Changes

### 3.1 Articulated leg variant (`mujoco/rider.py`)

New build path alongside `build_seated_rider`, selected by a `legs` parameter (`articulated`|`rigid`; `articulated` default when `crank_joint=True`, `rigid` otherwise):

Per leg (front/rear):

| Body | Parent | Joint | Axis | Notes |
|---|---|---|---|---|
| `rider_thigh_{f,r}` | `rider_pelvis` | hinge | Y | hip flexion; pivot at hip point from pose solver |
| `rider_shank_{f,r}` | `rider_thigh_{f,r}` | hinge | Y | knee flexion |
| `rider_foot_{f,r}` | `rider_shank_{f,r}` | hinge | Y | ankle; foot capsule |

- Segment masses total ≈13.2 kg/leg (as today), distributed realistically: thigh ≈ 10%, shank ≈ 4.5%, foot ≈ 1.4% of rider mass.
- The `rider_leg_{front,rear}` slide bodies and their `_z` joints are **not emitted** in the articulated variant (they remain in `rigid`). Pelvis/torso/arms slide joints are unchanged.
- Leg geoms remain non-colliding (`contype=0, conaffinity=0`) — same as today.
- Hip pivot rides on `rider_pelvis`, which keeps its vertical slide spring — so hips bob with the pelvis, and the leg chain absorbs relative hip↔pedal motion over bumps, as a real rider does.

### 3.2 Pedal bodies (`mujoco/drivetrain.py`)

When `crank_joint=True`:

- Each crank arm gets a `pedal_{f,r}` body at the spindle end on a **passive hinge** (axis Y, light damping — a real pedal bearing). The existing pedal-box geom moves onto this body.
- Foot platform stays level; orientation is slaved to the welded foot + ankle (see §4).
- Crank length stays 165 mm (`geometry/specs.py:58`); arms at y = ∓0.075, pedals at y = ∓0.115 (`cockpit.py:53-56`) — leg planes are built at the pedal offset, not y=0.

### 3.3 Foot weld

`<equality><weld>` (or `connect` — decided in implementation; weld preferred to also fix foot orientation) between each `rider_foot_*` body and its `pedal_*` body, active in pedal modes. This is the clipless-pedal model: bidirectional force transmission (drive *and* pull-up), zero slip.

### 3.4 Actuation (`mujoco/actuators.py`)

- Per leg: torque actuators on hip, knee, ankle hinges — 6 total. Gear/limits sized from the human range needed to produce the 60 N·m crank ceiling through the pedal lever (165 mm crank, ~0.5 m leg lever): rough sizing to be validated in implementation.
- `crank_drive` (motor on `crank_spin`, ±180 N·m): **removed in `pedal` mode** — legs are the sole engine; **retained in `pedelec`** as the mid-drive assist channel; **retained in `motor`** (see §6).

---

## 4. Control & Force Path (`sim/ride/`)

### 4.1 Demand pipeline — unchanged upstream

`PedalDrivetrain.compute` (`sim/ride/drivetrain.py:186-264`) keeps doing what it does: cruise controller wheel-torque demand → mean crank torque (`demand × gear_ratio`, clamped by 60 N·m torque / 300 W ceilings) → `ripple_shape(phase)` → first-order-lagged assist (pedelec). The change is only the **sink**: in pedal mode the rider-torque component no longer writes to `ctrl[crank_drive]`; it is distributed to the legs. Assist torque (pedelec) still writes to `crank_drive`, as a real mid-drive would.

### 4.2 Pedal force → joint torques

Per leg, per step:

1. **IK reference**: compute hip/knee/ankle angles for the current pedal position on the crank circle, reusing `physics/rider.py::_two_link_ik` (:422-450) and the `solve_seated_pose` geometry (:526-717 — saddle height LeMond 0.883, hip +60 mm over saddle top, ankle +115 mm over pedal spindle `ANKLE_ABOVE_PEDAL_M`). Solved in each leg's own sagittal plane at pedal y-offset. Hip anchor follows live `rider_pelvis` position.
2. **Force component**: rider demand splits per leg by crank phase (downstroke leg produces the positive stroke; the existing `ripple_shape` already encodes the phase profile). Desired pedal force → joint torques via leg Jacobian transpose: `τ_leg = Jᵀ · F_pedal`. *As shipped:* `Jᵀ` is applied directly — a 3-joint leg mapping a 2-D sagittal force is injective in this direction (only torque → force would be underdetermined), so no pseudoinverse is needed; the full joint-space impedance of item 3 covers the drift the null-space term was meant to absorb.
3. **Impedance component**: low-gain position servo (`kp`, `kd` on each hinge) toward the IK pose — holds pedaling shape, damps null-space drift, absorbs bump-induced relative motion, and returns legs to a clean pose after disturbances.
4. `qfrc` written via actuators (`ctrl`), not `qfrc_applied` — the leg force physically drives `crank_spin`, and wheel resistance pushes back through `chain_drive`. Uphill = demand rises = pedal force rises = wheel torque rises.

### 4.3 Freewheel & coast

Unchanged concept: on zero/negative demand `chain_drive` disengages (`_disengage`, :354-365) and the crank is held at its latched phase by the existing PD hold (:367-381). Legs' impedance relaxes to a rest pose; feet stay welded and ride the held cranks — a coasting rider.

### 4.4 Reset

`PedalDrivetrain.reset` teleports `qpos[crank_spin]` to `--crank-phase` (ride_sim.py:259-262). On reset the leg joint `qpos` must be initialized to the IK solution for that phase — fixes the existing transient where spring supports mismatched pedal position until the first `_follow_cranks`.

---

## 5. Physics Consequences

- **Load path**: leg mass (~26 kg) now loads the pedals/crank bearings and the pelvis interface — the physically real path — instead of a scripted spring to the frame.
- **The cadence bob**: emerges from leg joint impedance acting at 2× cadence rather than a dedicated spring. `validate_pedals.py` check 3 is **recalibrated** against the articulated variant (the rigid baseline remains for comparison). Expected retuning work; if impedance tuning fights the test, fallbacks in order: (a) keep hidden mass-bearing slide bodies, (b) add compliance at the pelvis interface.
- **Motor mode freebie**: with feet welded, spinning `crank_drive` drags the passive legs through the stroke — impedance shapes the motion. This is the "coasting passenger" and powers the `--visual-pedalling` flag.
- **DOF count**: in pedalled modes, +8 joints added (6 leg hinges + 2 pedal hinges), −2 leg slide joints removed → `nq` goes 18 → 24.

---

## 6. Modes & CLI

| Mode | Legs | Crank | Force source |
|---|---|---|---|
| `bike-ride --drive-mode pedal` | articulated (physical) | `crank_spin` | legs only |
| `bike-ride --drive-mode pedelec` | articulated (physical) | `crank_spin` | legs + `crank_drive` assist |
| `bike-ride --drive-mode motor` (default) | rigid (unchanged physics) | welded | `crank_drive` n/a — wheel motor |
| `motor` + `--visual-pedalling` | articulated, impedance-only | `crank_spin` dragged by `chain_drive` off the driven wheel (`crank_drive` emitted but idles at ctrl 0) | crank drags legs (coasting passenger) |

Note: `--visual-pedalling` forces `crank_joint=True` and `crank_drive` emission at build time even though physics stays in motor mode (the wheel actuator remains the propulsion source; `crank_drive` stays at ctrl 0 and the `chain_drive` equality is what spins the cranks for the leg kinematics).
| playground / test-stand / export | unchanged (lumped/rigid) | welded | — |

- New flags: `--legs articulated|rigid` (default articulated in pedal modes, rigid elsewhere); `--visual-pedalling` (motor mode only).
- `bike-export` keeps writing the welded-crankset models; optionally gains an articulated snapshot file (decide in implementation).

---

## 7. Verification & Acceptance

Extend `tools/validate_pedals.py`:

1. **Propulsion**: pedalled cruise still reaches commanded speed; measured wheel torque consistent with leg-produced crank torque through `chain_drive` (check 2 generalizes: force now flows legs→crank→wheel).
2. **Cadence bob**: 2×-cadence shock-stroke amplitude persists with articulated legs — recalibrated expectation vs the rigid baseline.
3. **Freewheel**: open/re-engage leaves no position residual (check 4 unchanged).
4. **No-slip invariant**: weld holds — foot↔pedal residual stays ~0 through the whole stroke and over bumps.

New unit tests:

- IK vs pedal circle: solved foot point lands on the pedal spindle for all crank phases, in each leg's plane.
- Joint ROM: hip/knee/ankle stay within human ranges over a full pedal cycle (knee flexion within the existing 20–60° BDC band convention, `physics/rider.py:586-592`).
- Uphill: on a grade, sustained demand produces proportionate wheel torque (the Q4 requirement).

Render: one short offscreen clip of a pedal cycle for visual eyeballing (`scripts/mujoco_render_views.py` or equivalent) — the original complaint was visual, so a human looks at it.

---

## 8. Risks & Mitigations

| Risk | Mitigation |
|---|---|
| Closed chain (pelvis→leg→weld→pedal→crank→frame) destabilizes the solver | Impedance term keeps joints near IK; torque ceilings; rigid variant retained as fallback; weld over connect preferred (fully fixes foot frame) |
| Leg torque control produces unphysical poses via Jacobian null-space | Pseudoinverse min-norm + null-space impedance toward IK pose |
| Bob-test recalibration fights joint tuning | Fallbacks (§5): hidden mass bodies or pelvis compliance |
| Pedal hinge + ankle + weld over-constrains foot orientation | Pedal hinge is passive; ankle impedance is soft — solver resolves the shared DOF; verify no equality-solver warnings at all phases |
| `--crank-phase` init mismatch | IK-initialize leg qpos at reset (§4.4) |
| Perf regression | +8 DOF is small vs the 238 µs tyre budget; measure `validate_pedals` runtime before/after |

---

## 9. Explicitly Out of Scope

- Standing/out-of-saddle pedaling, bike rocking under sprint.
- Frontal-plane leg DOF (knee sway, hip rock).
- True flat-pedal contact physics (slip, push-only) — weld models clipless.
- Torso/arms/head changes beyond what exists (Q3 = legs only).
- Playground/test-stand/export riders.
