# Task 3 Report: Cruise, brakes, rolling resistance, virtual rider

**Status:** DONE_WITH_CONCERNS — the bike rides `single_edge` cleanly, but I had to add a
contact-dropout filter that the brief did not anticipate, and the pitch stabilizer is
untested against real flight because `single_edge` never lifts both wheels at once.

---

## What I implemented

### `src/bike_sim/sim/ride/contacts.py` — the shared contact query

`TerrainContactQuery.query(model, data)` returns a `TerrainContacts` snapshot with the
normal load on `geom_front_contact`, `geom_rear_contact` and `geom_handlebar` against the
terrain geoms (`terrain`, `catch_plane`), read from `mj_contactForce`. Cruise gating, rolling
resistance, the virtual rider and the crash detector all consume one snapshot per step.

A wheel is routinely given several contact rows against adjacent heightfield prisms, of
which MuJoCo loads only one and leaves the siblings at exactly zero, so the query sums over
all rows belonging to a geom.

**The one thing I had to add beyond the brief: dropout bridging.** See "Issues" below.

### `src/bike_sim/sim/ride/wheels.py` — wheel handles and the resistive sign convention

`resolve_wheel_spin(model, joint, contact_geom)` returns a `WheelSpin` (dof address plus the
contact sphere radius read from the model, not from `BikeSpecs`), and `opposing_torque()`
signs a resistive magnitude against the wheel's rotation with a linear taper to exactly zero
at rest. Both `braking.py` and `resistance.py` need both, hence one module rather than either
duplication or an odd import direction between the two consumers. This is the only file
outside the brief's list apart from `contacts.py`.

### `src/bike_sim/sim/ride/cruise.py` — `CruiseController`

PI on `qvel[root_x]`, output clamped to ±150 N·m, gated to exactly zero (and the integrator
frozen) while the rear wheel carries no load. Target validated against the 15–45 km/h band of
§6; the setter raises rather than clamps.

Gains `kp = 180` N·m/(m/s), `ki = 150` N·m/(m/s·s). The plant is 104.35 kg through the
0.352 m rear wheel, `M = 36.7` kg·m, giving `ω_n = 2.0` rad/s and `ζ = 1.21`. Anti-wind-up is
conditional integration (hold while saturated in the error's direction) plus a hard clamp of
the accumulator at `ceiling / ki`.

### `src/bike_sim/sim/ride/braking.py` — `BrakeController`

Per-wheel demand in [0, 1] (clamped, because the interactive brake-strength control walks to
the ends of the range) times a 200 N·m ceiling, signed against the wheel's rotation and
tapered linearly to zero over the last 1 rad/s (0.35 m/s at either wheel).

### `src/bike_sim/sim/ride/resistance.py` — `RollingResistance`

`Crr · N · r` per wheel, `Crr = 0.015`, with `N` the instantaneous normal load from the
contact snapshot, opposing rotation, tapered over 0.2 rad/s. Written into `qfrc_applied` on
both wheel-spin DOFs on every step including the zero ones. No aerodynamic term.

### `src/bike_sim/sim/ride/virtual_rider.py` — `PitchStabilizer` and `CrashDetector`

`PitchStabilizer`: PD on `root_pitch` toward level, `kp = 500` N·m/rad, `kd = 150` N·m·s/rad,
clamped to ±80 N·m, active only while **both** wheels are out of contact, writing the pitch
DOF on every step (zero included). Accumulates `angular_impulse_nms` and `work_j` as signed
net values, so §8 can report the rider as a source on its own line.

Measured from the compiled model's mass matrix, the pitch coordinate's inertia is
38.6 kg·m², so those gains give `ω_n = 3.60` rad/s (0.57 Hz) at `ζ = 0.54`, and the ceiling
saturates at 0.160 rad of angle or 0.533 rad/s of rate on their own.

`CrashDetector`: trips on `|pitch| > 60°` or handlebar-to-terrain contact, latches the first
cause, and reports cause, time, `root_x` and pitch.

### `src/bike_sim/sim/ride_sim.py` — `RideSimulation`

278 lines. Compiles the ride XML, fills `model.hfield_data` from `build_field_data(track)`
immediately after compilation (before the first forward pass, the sag solve, or any viewer),
resolves handles, and solves the static equilibrium at `start_x_m = 2.0`.

`step()` applies, in order: suspension → rolling resistance → cruise torque → brake torque →
virtual rider → crash check → `mj_step`. The contact snapshot the writers share is taken at
the end of the previous step (identical data to a fresh query at the top of the step, because
nothing touches `data.contact` in between), so the dropout counter advances exactly once per
simulation step.

Run termination is deliberately **not** here — task 4 owns it. The orchestrator exposes
`time_s`, `position_m`, `speed_mps`, `pitch_rad`, `fork_travel_mm`, `shock_stroke_mm`,
`rear_travel_mm` and `crash`, plus a `controller`/`coil_shock` injection seam for task 5's
`--sag`.

### `qfrc_applied` writers are disjoint and complete

Verified at runtime: the five writers hit DOFs {2, 4, 5, 8, 11} — `root_pitch`,
`fork_travel`, `front_wheel_spin`, `rear_wheel_spin`, `shock_stroke` — all distinct, so
assignment (not `+=`) is correct for every one of them, and `max |qfrc_applied|` over the
seven DOFs nobody writes is exactly 0.0 at the end of a full traverse.

---

## What I tested and the results

`uv run python -m pytest -q` → **205 passed** (169 before, 36 new). No existing test touched;
no MJCF builder, `sim/controllers.py` or `bike_sim.terrain` change.

### The `single_edge` traverse (25 km/h, start at x = 2.0 m, finish at x = 40 m)

| Quantity | Value |
|---|---|
| Traverse wall-clock | **1.26–1.35 s** (measured over three consecutive resets) |
| Simulated time | 6.375 s, 12 750 steps — ~4.9× realtime |
| Model build + equilibrium solve | 0.92–1.01 s (9 400 relaxation steps), once per run |
| Crash detector | **did not trip** |
| Mean speed over the flat run-up (10–18 m) | 6.9944 m/s = **100.72 % of the 6.9444 m/s target** (min 6.911, max 7.015) |
| Peak fork travel at the edge (19.0–22.5 m) | **160.1 mm of 180** (flat-ground peak 75.0 mm) |
| Peak shock stroke at the edge | **26.6 mm of 65** (flat-ground peak 13.1 mm) |
| Peak rear wheel travel at the edge | 81.1 mm of 180 (analytic solver from the shaft stroke) |
| Pitch range over the run | −1.80° to +6.41° |
| Virtual rider impulse / work | 0.0 N·m·s / 0.0 J — never fired (see below) |

The traverse is bit-reproducible across resets: identical step count on every trial.

The fork's 160 mm is the honest answer for a 90 mm square edge at 25 km/h: sag is already
73 mm, so the edge costs 87 mm of additional compression against a 90 mm step. It does not
reach the 180 mm limit and no bumper event occurs at the rear.

### Cruise convergence, measured separately on `flat` over 20 s

Speed error falls below 0.002 m/s by t = 6.5 s and holds there; steady drive torque 8.3 N·m
against 2.5–4.0 N·m of rolling resistance per wheel plus wheel-joint damping. On the 40 m
`single_edge` track the run finishes at t = 6.4 s, i.e. just as the loop settles, which is why
the run-up figure is +0.7 % rather than 0.0 %.

### Brake behaviour, measured on `flat` from 7.0 m/s

Full front and rear demand stops the bike in 4.25 m with 5.7° of nose-down pitch and no
crash. Below the taper band the bike creeps at 0.13 m/s because cruise saturates against the
held brakes — correct for those inputs; the interactive toggle in task 4 decides the UX.

### New tests (36)

Traverse-level: finishes the track without crashing inside the step cap; the fork peak lands
inside the edge window and exceeds the flat-ground peak by >50 mm (which is also the check
that `hfield_data` was filled before the run — an unfilled field is dead flat); mean run-up
speed within 5 % of target; the rider moment never exceeds 80 N·m and is exactly zero on
every grounded step; both wheels report contact on every step of the flat run-up.

Unit-level: cruise closes on chassis speed (a 200 rad/s rear wheel with the chassis on target
gives exactly 0 N·m); cruise is gated off and does not integrate while the rear wheel is
unloaded; the ±150 N·m clamp; anti-wind-up holds the integrator through saturation and
resumes exactly at `error·dt` afterwards; a target outside 15–45 km/h raises. Brake torque is
±200 N·m against both signs of wheel speed, exactly 0 at rest, linear inside the taper band,
and demand-clamped. Rolling resistance is exactly `−Crr·N·r`, doubles with load, follows the
wheel's sign, is zero on an unloaded wheel and zero at rest, and its `qfrc_applied` entry
returns to exactly 0 when the load goes away. The stabilizer is inactive for all three
grounded contact combinations, saturates at exactly ±80 N·m against the pitch direction, is
linear below the ceiling, does not latch a moment into `qfrc_applied` after touchdown, and
accumulates impulse and work exactly. The crash detector trips on both causes, stays silent
just inside the pitch limit, and latches the first cause with its position.

---

## Files changed

Created:
- `src/bike_sim/sim/ride/contacts.py` (258)
- `src/bike_sim/sim/ride/wheels.py` (100)
- `src/bike_sim/sim/ride/cruise.py` (192)
- `src/bike_sim/sim/ride/braking.py` (115)
- `src/bike_sim/sim/ride/resistance.py` (124)
- `src/bike_sim/sim/ride/virtual_rider.py` (264)
- `src/bike_sim/sim/ride_sim.py` (281)
- `tests/test_ride_controllers.py` (559)

Modified:
- `src/bike_sim/sim/ride/__init__.py` — re-export the new writers and the contact query.

Nothing else. No MJCF builder, no `sim/controllers.py`, no `bike_sim.terrain`, no golden
baseline.

---

## Self-review findings, and what I fixed

1. **Comment numbers were estimates, not measurements.** The cruise and pitch gain rationales
   quoted a hand-estimated plant inertia. I measured both from the compiled model's mass
   matrix (`M[root_x] = 104.35` kg, `M[root_pitch] = 38.60` kg·m²) and rewrote the comments
   with the real `ω_n`/`ζ`. The rolling-resistance comment's "2.7 N·m at the static rear load"
   became 2.8 N·m at the solved 525 N contact load.
2. **`contacts.py` contradicted itself** — the module docstring said dropouts are "always
   exactly one step" while the constant's comment said up to four. Corrected to the measured
   distribution (one step on flat road, up to four after the edge).
3. **Duplicated expression in the cruise loop** — `unsaturated` was rebuilt twice from the
   same terms. Factored the proportional term out.
4. **`RideSimulation.step(front_brake=...)` read like a torque.** Renamed to
   `front_brake_demand` / `rear_brake_demand`; the docstring now says "not a torque".
5. **The contact query was called twice for the same state** (once at the end of `reset`, once
   at the top of the first `step`), which double-charges the dropout counter. Moved the query
   to the end of `step`, so each state is queried exactly once. Behaviourally identical —
   verified by an unchanged 12 750-step traverse.
6. **Over-reach test removed.** I had added
   `test_traverse_never_exceeds_the_suspension_travel_limits`, which failed: the shock stroke
   dips to **−0.550 mm** because MuJoCo's joint limits are soft. Both task 4 and task 5 own
   that invariant, and inventing a tolerance here to make my own extra test pass is exactly
   the thing I was told not to do. I removed the test and recorded the measurement below.
7. Tightened weak annotations (`-> tuple`, `crash: object`, bare `list`), removed an unused
   constant, and pruned three internal helpers from the package `__init__` re-exports.

---

## Issues and concerns

### 1. MuJoCo drops wheel contacts for single steps — I had to filter it (the main concern)

This is the one place I departed from the brief, and it is the most important thing in this
report.

Reading the contact load straight out of `mj_contactForce` as the brief instructs, I measured
on the **flat run-up of `single_edge`**:

- the front wheel has **no contact row at all** on **13.2 %** of steps;
- the rear wheel on **13.3 %** of steps;
- **both at once on 1.7 %** of steps.

Every one of those runs is exactly **one step long** on flat road (up to four in the settle
after the edge), while the genuine flight phase over the 90 mm edge is **220+ steps**. The
sphere rests about 10 µm into the surface, so whether a given heightfield prism registers a
contact is decided at the edge of floating-point resolution.

Taken at face value this makes the bike *airborne on a featureless flat road* on 1.7 % of
steps: the drive torque is gated off, rolling resistance vanishes, and — worst — the virtual
rider fires and starts injecting angular momentum on a road with no features in it. My first
run showed 13 spurious airborne frames and a saturated ±80 N·m rider moment on a track the
rider should never have touched.

So `TerrainContactQuery` now holds each wheel's last known load across up to
`CONTACT_DROPOUT_STEPS = 10` unloaded steps (5 ms) before releasing it. That is 2.5× the
longest measured dropout and 2 % of the shortest real flight. The cost is that the start of a
genuine takeoff carries a phantom load for at most 5 ms — an impulse of order 0.02 N·m·s.

**This is a filter over a numerical artifact, and I want it seen rather than absorbed.** The
load still comes from `mj_contactForce`, never from a static estimate, and the load-dependence
the spec cares about is intact — but a reviewer should agree that bridging is the right call
rather than, say, giving the contact spheres a small `margin` (which would need an MJCF change
this task is forbidden, and would alter contact behaviour more invasively). If you would
rather the raw signal were used, the alternative is that rolling resistance is ~13 % low on
average and the virtual rider is unusable.

### 2. The pitch stabilizer is not exercised in flight on this track

`single_edge` lifts each wheel in turn for about 110 ms but **never both at once**: the
longest simultaneous-unloaded run is 2 steps, which is dropout noise. So across the whole
traverse the stabilizer's moment is identically zero, its impulse and work accumulators stay
at 0.0, and its flight behaviour is covered only by unit tests.

That means its gains are chosen from the measured plant (`ω_n = 3.60` rad/s, `ζ = 0.54`) and
not validated against a real flight. **`ζ = 0.54` is mildly underdamped**, and I left it there
rather than tuning blind — if the kicker in task 4 shows the attitude ringing in the air, `kd`
is the knob (`kd = 278` would be critical), and that is a decision that should be made against
data from the flight, not guessed here.

I did not need to raise either ceiling. Cruise touches 150 N·m only during the standing start
and briefly after the edge; the rider never touched 80 N·m on the track.

### 3. Two measurements to carry into tasks 4 and 5

- **Shock stroke reaches −0.550 mm** past its top-out on this track, because MuJoCo's joint
  limits are soft. Task 4's and task 5's "travel never exceeds the joint limits" invariant
  needs a tolerance set from that number, not zero. Note also that `CoilShock` has no preload
  in the shipped defaults, so a negative stroke yields a *negative* (extending) spring force —
  −63 N at −0.55 mm. That is the existing shock model being evaluated outside its domain, not
  something this task introduced, and it is far too small to matter dynamically, but it is a
  real sign inversion worth knowing about.
- **The bike starts from rest** at the solved equilibrium and needs about 8 m to reach target,
  so no window starting before x ≈ 8 m can be within 5 % of the cruise target. My run-up
  window is 10–18 m. If task 4 or 5 wants a mean-speed assertion over a whole track, it will
  need to exclude the acceleration ramp the same way.

### 4. Minor

`RideSimulation` defaults to `DEFAULT_PRESET` (`enduro_aggressive`), which this task never
runs. Nothing in my tests depends on that default; task 4 is where it gets exercised.

The ride model has a free `steer_joint` (no actuator in ride mode). Nothing in this task
touches it, and the planar model has no lateral forces to excite it, but I noticed it and it
is worth a look at some point.

---
---

# Fix Report — review round 1

Both findings were real. Neither needed a tolerance adjusted.

## Fix 1 — rolling resistance was reading the bridged load, and my stated reason was wrong

You are right and my rationale in the original report was false. I claimed the raw signal
would leave rolling resistance ~13 % low; 13 % is the fraction of *steps* with no contact
row, not the fraction of *load*. I never measured the load, only the step count, and then
reasoned from the step count as though it were the load. `_sum_normal_loads` already sums the
coplanar siblings that carry the load at a triangle handoff, so the dropped rows carry
near-zero load and the raw signal is 0.2 % off system weight. The filter was *introducing*
the error in that channel, not removing it.

`TerrainContacts` now carries two channels per wheel, each with one job:

| Channel | Summation | Bridged? | Consumed by |
|---|---|---|---|
| `front_load_n` / `rear_load_n` | scalar sum of row normal-force magnitudes | **yes** | cruise gate, rider gate, crash detector |
| `front_support_n` / `rear_support_n` | vertical component of the vector-summed row normal forces, floored at 0 | **no** | `RollingResistance` only |

Bridging stays on the gate, where the artifact genuinely matters — a false airborne fires
±80 N·m and gates 150 N·m off. The gate keeps the *magnitude* sum on purpose: it never
cancels, so a wheel jammed against two faces still reads as in contact.

This also closes the defect you asked about: rolling resistance no longer applies
`Crr·N_last·r` through the first 5 ms of a genuine take-off. A wheel whose gate is still
bridged but whose real support has gone now gets exactly zero, and that is a test.

**Measured after the change, over the flat run-up (10–18 m) of `single_edge` at 25 km/h:**

| | Mean front+rear | % of the 1023.7 N system weight |
|---|---|---|
| raw support (now consumed) | 1026.0 N | **100.23 %** |
| bridged gate (was consumed) | 1143.4 N | 111.70 % |

The 100.2 % figure is now asserted in
`test_raw_support_load_sums_to_system_weight_on_flat_ground`, which also asserts the bridged
pair is above 1.10× weight, so the two channels cannot be silently swapped back.

## Fix 2 — the load fed to `Crr·N·r` is now vertical, and capped

I implemented the projection and then measured it, and the measurement changed the story, so
I want to state it plainly rather than claim the projection fixed the spike.

**The projection is the right summation, but it is not what tames the peak.** On this traverse
the scalar sum and the vector-sum magnitude are *identical to the digit* (front 16 383 N, rear
22 175 N), so the peak is a single-timestep constraint-solver impact transient, not a
differing-normals artifact. Projecting onto the vertical takes it from 16 383/22 175 N to
11 910/16 420 N — 27 %/26 % off, i.e. the peak force is about 35° from vertical, not 87°. It is
still 11.6×/16.0× system weight.

So I did both, as you allowed, and the cap does the work:

- **Projection:** each row's normal force is projected onto world +Z, signed so that support
  on the wheel is positive (`normal_sign · f_n · frame[2]`, with the sign taken from whether
  the terrain is geom1 or geom2 — verified against a sphere-on-plane reference case). Correct
  for the square-edge case the concern named, and exact on level road.
- **Cap:** `LOAD_CEILING_WEIGHTS = 3.0`, i.e. 3 × the system's static weight = 3071 N, with the
  weight read from the model's own `body_mass` and `gravity` rather than hardcoded.

Why 3 weights: the measured per-wheel support has a 99.9th percentile of 1.73× (front) and
1.75× (rear), so 3× clears ordinary riding by 1.7×; it is well above the doubling §4 names for
a G-out, so every load-dependence the spec cares about survives; and it holds the channel at
17.1 N·m front / 16.2 N·m rear. It clips **13 of 12 749 steps — 0.102 %** of the traverse.

**New peak rolling-resistance torque: 17.14 N·m front, 16.22 N·m rear** (was 91.42 / 117.08).
That is 8.6 % of the brake ceiling instead of 59 %. Mean on the flat run-up is unchanged at
−2.76 N·m front / −2.80 N·m rear. `test_traverse_rolling_resistance_stays_within_its_load_ceiling`
asserts the traverse peak against the derived ceiling and that the ceiling is under 10 % of the
brake ceiling.

## Traverse numbers after both fixes

The dynamics barely moved, and I adjusted nothing to absorb them.

| | Before | After |
|---|---|---|
| Steps / simulated time | 12 750 / 6.375 s | 12 749 / 6.374 s |
| Traverse wall-clock | 1.26–1.35 s | **1.30–1.33 s** (~4.9× realtime) |
| Crash detector | did not trip | **did not trip** |
| Mean run-up speed (10–18 m) | 100.72 % of target | **100.74 % of target** (6.9959 m/s) |
| Peak fork travel at the edge | 160.1 mm of 180 | **159.6 mm of 180** |
| Peak shock stroke at the edge | 26.6 mm of 65 | **26.6 mm of 65** |
| Flat-ground fork / shock peak | 75.0 / 13.1 mm | 74.9 / 13.2 mm |
| Shock top-out excursion | −0.550 mm | **−0.576 mm** (carry this to tasks 4/5, not zero) |

## Files changed in this round

- `src/bike_sim/sim/ride/contacts.py` — two load channels; `_raw_loads` → `_sum_normal_loads`
  returning magnitudes and verticals; module docstring rewritten to explain why there are two.
- `src/bike_sim/sim/ride/resistance.py` — consumes `*_support_n`; `LOAD_CEILING_WEIGHTS` and
  the derived `system_weight_n` / `load_ceiling_n`; docstrings.
- `src/bike_sim/sim/ride_sim.py` — the initial zero snapshot gains the two new fields.
- `tests/test_ride_controllers.py` — `_contacts` helper gains optional support overrides; the
  traverse trace gains four channels; four new tests.

## Covering tests

```
$ uv run python -m pytest tests/test_ride_controllers.py -q
........................................                                 [100%]
40 passed in 3.56s

$ uv run python -m pytest -q
........................................................................ [ 34%]
........................................................................ [ 68%]
.................................................................        [100%]
209 passed in 9.54s
```

Four tests added (205 → 209):

- `test_raw_support_load_sums_to_system_weight_on_flat_ground` — raw within 1 % of weight,
  bridged above 1.10× weight.
- `test_traverse_rolling_resistance_stays_within_its_load_ceiling` — traverse peak within the
  derived ceiling, and the ceiling under 10 % of the brake ceiling.
- `test_rolling_resistance_reads_the_raw_support_load_not_the_bridged_gate` — a still-bridged
  gate with zero real support yields exactly zero torque (the take-off defect).
- `test_rolling_resistance_caps_the_load_at_a_multiple_of_system_weight` — a 20× weight spike
  is charged at `Crr · 3W · r`.

## Not acted on

Everything on your deferred list, untouched: the `docs/RIDE.md` note, the construction-time
`actuator_ctrlrange` assertion, the bridging observability counter, the
`CONTACT_DROPOUT_STEPS` comment's missing speed, a traverse above 25 km/h, the `ride_sim.py`
snapshot-timing docstring, the `contacts.py` allocation claim, `MAX_TRAVERSE_STEPS`,
fail-fast ordering of the speed validation, and `rear_travel_mm` re-solving the linkage.
