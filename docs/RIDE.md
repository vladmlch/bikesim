# Ride Mode

Rolling whole-bike simulation over a longitudinal road profile with bumps, potholes,
rock gardens, a drop and a kicker. Complements the three existing modes
(`standard`, `stand`, `playground`), all of which bolt the frame to the world.

> **Status.** This document currently contains the *Physics Contract* only — the formal
> statement of what is simulated, how, and with what admitted inaccuracies. It is written
> before the implementation deliberately: every numbered subsection below is a commitment
> the code is expected to honour, and several of the numbers in it were measured rather
> than assumed. Usage, CLI and telemetry reference sections are added as the mode lands.

---

## Table of Contents

- [Physics Contract](#physics-contract)
  - [0. Model class and scope](#0-model-class-and-scope)
  - [1. Generalized coordinates and states](#1-generalized-coordinates-and-states)
  - [2. Road profile](#2-road-profile)
  - [3. Wheel–ground contact](#3-wheelground-contact)
  - [4. Longitudinal friction and slip](#4-longitudinal-friction-and-slip)
  - [5. Suspension force path](#5-suspension-force-path)
  - [6. Propulsion and braking torque path](#6-propulsion-and-braking-torque-path)
  - [7. Virtual rider](#7-virtual-rider)
  - [8. Energy and momentum accounting](#8-energy-and-momentum-accounting)
  - [9. Static equilibrium and sag](#9-static-equilibrium-and-sag)
  - [10. Numerical settings](#10-numerical-settings)
  - [11. Authored assumptions](#11-authored-assumptions)
  - [12. Known non-physicalities](#12-known-non-physicalities)

---

# Physics Contract

## 0. Model class and scope

Ride mode is a **planar (sagittal-plane) multibody simulation**. The bike translates
longitudinally and vertically and pitches; it has no lateral position, no roll and no yaw.

This is a deliberate choice, not a simplification pending removal. A 6-DOF free-floating
bicycle is laterally unstable and would require a balance and steering controller — a
different project with a different validation burden. The planar model isolates exactly
what this repository is about: how the Horst-link kinematics and the fork/shock hardware
respond to vertical terrain input.

**Directly implied by the planar restriction, and therefore out of scope by construction:**

- lateral terrain relief, camber, banking, off-camber traction;
- roll dynamics, countersteer, cornering loads;
- any difference between the front and rear wheel in the lateral direction;
- rider lateral weight shift.

None of these are unobservable "missing features"; they do not appear in the equations of
motion of a planar system at all. The road is therefore a **one-dimensional profile**
`z = h(x)`, not a terrain `z = h(x, y)` — see §2.

---

## 1. Generalized coordinates and states

MuJoCo is a generalized-coordinate engine. Everything below lives in `qpos`/`qvel`; the
loop closures are **soft constraint forces** in `efc_*`, not algebraic states, and `act`
is empty because every actuator used here is stateless (`position`, `motor`).

| Coordinate | Joint type | Range | Status |
|---|---|---|---|
| `root_x` | slide, world X | — | free |
| `root_z` | slide, world Z | — | free |
| `root_pitch` | hinge, world Y | — | free |
| `steer_joint` | hinge, steer axis | ±0.01° | **locked** (`stiffness=50000`) |
| `fork_travel` | slide, steer axis | 0 … 0.180 m | free, loaded via `qfrc_applied` |
| `main_pivot` | hinge Y | — | rear linkage |
| `horst_pivot` | hinge Y | — | rear linkage |
| `rocker_frame_pivot` | hinge Y | — | rear linkage |
| `yoke_pivot` | hinge Y | — | rear linkage |
| `shock_stroke` | slide, shock axis | 0 … 0.065 m | rear linkage, loaded via `qfrc_applied` |
| `front_wheel_spin` | hinge Y | — | free, **dynamic** |
| `rear_wheel_spin` | hinge Y | — | free, **dynamic** |

`nq = nv = 12`, `neq = 2`.

**Rear linkage mobility.** Five coordinates (`main_pivot`, `horst_pivot`,
`rocker_frame_pivot`, `yoke_pivot`, `shock_stroke`) are reduced by two `<connect>` loop
closures (seatstay↔rocker at P3, shock↔frame at P7). Each `<connect>` is a 3-dimensional
ball constraint, of which two dimensions are effective in a planar linkage and one is
redundant, absorbed by the solver's regularization. Net rear suspension mobility is
therefore exactly **1 DOF**, as the analytical 4-bar solver assumes.

**Total mobility: 7** — three chassis, one fork, one rear suspension, two wheel spins,
plus the locked steer coordinate.

**Wheel rotations are genuine dynamic degrees of freedom**, not auxiliary states. They
carry rotational inertia, they are driven by actuator torque and by contact friction, and
they can spin up freely when the wheel leaves the ground. Any control law that assumes
wheel speed tracks ground speed is invalid here — see §6.

**Steer lock rationale.** A planar root cannot absorb steer rotation: turning the steerer
moves the front contact patch out of the XZ plane, which the model has no coordinate for.
The joint is retained (rather than removed) so that the body tree, the sensor suite and
the existing builder code stay shared with the other three modes.

---

## 2. Road profile

A **longitudinal heightfield**: `z = h(x)`, extruded across Y.

| Parameter | Value |
|---|---|
| `ncol` (along X) | 24001 |
| `nrow` (across Y) | 2 |
| `size` = (rx, ry, elev, base) | 60 m, 0.5 m, 3.4 m, 0.5 m |
| Longitudinal resolution | exactly 5 mm |
| Road datum | `z_local = 2.8 m` |
| Field envelope | −2.8 m … +0.6 m about the start datum |
| `enduro_aggressive` extent, measured | −2.510 m … +0.090 m |

`ncol` is 24001 rather than 24000 because a heightfield's samples sit at cell corners:
the spacing is `2·rx / (ncol − 1)`, and an even count would give 5.0002 mm.

**The geometry is fixed in the MJCF and is identical for every track preset.** Presets
differ only in the numbers written into `model.hfield_data` from Python, before the
viewer is created. Consequences: `mode="ride"` produces exactly one XML and one golden
baseline; the track is a pure Python object; `generate_mujoco_xml` gains no domain
parameter. Every preset must fit within 120 m and share the 5 mm grid.

`hfield_data` is `float32`, giving ~0.4 µm of vertical resolution over the 3.4 m span.
This is why the profile is supplied programmatically rather than through a PNG asset:
an 8-bit heightmap would quantize to 13 mm steps, coarser than several features on the
track and more than twice the grid spacing.

**No vertical faces exist in a heightfield.** A "square edge" is a one-cell ramp: 90 mm
over 5 mm is 87°. This is accepted — the contact patch of a 29″ tyre smooths a 5 mm
discontinuity more than the grid does, and a true right angle would produce a delta
impulse, i.e. a numerical artefact rather than a more honest result.

**`ry = 0.5 m`, not 2 m.** The bike cannot move laterally, so the strip only has to be
wider than the tyre. A wide strip would make each cell 5 mm × 4 m — sliver triangles with
an 800:1 aspect ratio, the worst case for MuJoCo's convex collision routine.

**No plane geom sits at road level.** In MuJoCo a `plane` geom is an **infinite** half-space
for collision purposes; its `size` affects rendering only. This was verified directly: a
sphere dropped at *x* = 5 m rests on a plane declared `size="0.1 0.1 0.1"`. A road-level
plane intended as an "approach section" would therefore floor the entire world and bridge
every pothole, the G-out and the drop. The 120 m field covers run-up, track and runout in
one piece; the only plane in the scene is a catch plane backing up the X-based run
termination, and its height is derived from the *field floor* rather than from any preset —
`geom_z − 1 m`, which for the model's −349.5 mm ground level puts it at −4.15 m.

**Net elevation.** The track descends 2.420 m end to end: 600 mm at the step-down and
1.820 m down the kicker's landing slope. The road between features is level — the descent
is carried by two discrete drops, not by a tilted road — so there is no gradient in the
sense of §2, but the track does finish 2.4 m lower than it starts and this is deliberate.
Together with the +90 mm high point it sets the 2.600 m span the field envelope contains.

### Default preset `enduro_aggressive` (115 m)

| X, m | Feature |
|---|---|
| 0–12 | run-up |
| 12–20 | braking bumps: 35 mm amplitude, 900 mm wavelength, 9 waves |
| 22 | square edge 90 × 250 mm |
| 26–34 | rock garden #1: ±60 mm, 250 mm correlation length, fixed seed |
| 38 | pothole 600 × 180 mm |
| 42–47 | G-out: 350 mm dip over 5 m |
| 52 | drop 600 mm |
| 57–65.9 | kicker 350 mm / 1.4 m (14.0° launch), 2.5 m gap, 5.0 m landing at 20° |
| 69–76 | roots: 7 bumps 80 × 250 mm, irregular spacing |
| 80 | double edge 70 mm and 90 mm, 800 mm apart |
| 84–95 | rock garden #2: ±90 mm, 200 mm correlation length |
| 95–115 | runout |

The kicker's landing runs 5.0 m although the flight at the default speed needs only 3.0 m.
Measured against the assembled profile, a 3.0 m landing works only between **20.9 and
26.9 km/h**, so a 20 / 25 / 30 km/h speed sweep would case at the bottom end and overshoot
at the top, and the section would not be comparable across the sweep it exists to serve. At 5.0 m the working band is
**20.9–30.6 km/h** and the touchdown at 25 km/h is unchanged, because at that speed the
bike lands 4.54 m past the lip, well inside the shorter slope. Steepening the landing
instead was measured and rejected: it *increases* touchdown severity, because the bike
falls further before meeting the slope. The root section then moves to 69 m so that it sits
3.1 m clear of the landing runout rather than 0.1 m.

The pothole width is chosen against the wheel radius rather than picked freely: a 372 mm
front wheel crossing a 600 mm gap drops 152 mm — into a 180 mm hole it **does not reach the
bottom**. The 352 mm rear wheel drops 168 mm, also clear. The event being measured is
"the wheel fell into the hole", not "the wheel rolled across the floor of the hole".

Rock gardens carry a fixed seed and are part of the preset: the randomness is reproducible
and belongs to the obstacle, not to a global noise switch.

---

## 3. Wheel–ground contact

| Property | Value |
|---|---|
| Contact geom | **sphere**, wheel radius, mass 0 |
| Visual/inertial geom | existing tyre cylinder, `contype=0 conaffinity=0` |
| `condim` | 3 (sliding friction; no rolling or torsional friction) |
| `solref` | `-130000 -800` |
| Sliding friction | 1.2, set explicitly on **both** the sphere and the heightfield |

**Why a sphere.** In the sagittal plane a sphere and a coaxial cylinder of equal radius
contact `h(x)` at the same point, so the two are geometrically indistinguishable here.
The sphere has no flat end faces to catch on a heightfield prism edge, which is the
dominant source of jitter in cylinder–heightfield collision. Keeping the cylinder as the
mass carrier preserves the existing rotational inertia exactly, so `test_mass_distribution`
is untouched.

**Why compliant contact.** `solref="-130000 -800"` prescribes contact stiffness and damping
directly: ≈130 N/mm, representative of a 2.5″ enduro tyre at ~25 psi, giving 4–5 mm of
static deflection at the rear. The tyre carcass is physically the first suspension element
and absorbs a meaningful fraction of a square-edge hit; a rigid contact would convert that
into a numerical spike and contaminate the shaft-velocity histogram that the damper tuning
depends on.

**Friction must be set on both sides.** With equal geom priorities MuJoCo combines contact
friction as the element-wise maximum of the two geoms. Setting only one side silently
inherits the other's value.

**Contacts elsewhere on the bike are left enabled**, as in the existing modes. Pedal strike
is a real consequence of 180 mm of travel and a −22.5 mm BB drop, and disabling frame and
rider contact would render a crash as the model sinking through the ground while the crash
detector (§7) flags it anyway.

---

## 4. Longitudinal friction and slip

Contact is **regularized Coulomb friction**. There is no brush model, no Pacejka curve, no
slip-ratio characteristic and no distinction between static and kinetic friction. Slip
appears when the demanded tangential force exceeds `μN`, and the transition is smoothed by
MuJoCo's constraint regularization rather than by a tyre model.

At the static rear axle load of 546 N and μ = 1.2, the traction limit is 655 N, i.e.
**230.6 N·m** at the 0.352 m rear wheel. The drive torque ceiling is 150 N·m (426 N), so
steady-state cruise is comfortably inside the friction cone; traction can still break when
the rear wheel unloads over a crest or lands.

Rolling resistance is **not** left to the contact model. It is applied explicitly as a
resistive torque proportional to the instantaneous normal load with `Crr = 0.015`. Joint
damping on the wheel spin axes (0.05 and 0.01 N·m·s/rad) is a constant and does not scale
with load, so it cannot represent a loss that doubles in a G-out.

**Aerodynamic drag is not modelled.** At 25 km/h this omits roughly 13 N, about 90 W —
so the reported wheel power is systematically low by around a quarter and coast-down is
optimistic. Comparisons *between* suspension settings are unaffected because the error is
identical across runs; absolute wattage is not trustworthy.

---

## 5. Suspension force path

Both suspension elements are driven by **generalized forces written into `qfrc_applied`**
on their own slide coordinates:

- `fork_travel` ← air spring (`ForkAirSpring`, polytropic, positive/negative chambers,
  volume tokens) + Charger 3 damping, as a function of `qpos[fork_travel]` and
  `qvel[fork_travel]`;
- `shock_stroke` ← coil spring + preload + bottom-out bumper + Super Deluxe RC2T damping
  with HBO, as a function of `qpos[shock_stroke]` and `qvel[shock_stroke]`.

A generalized force on a slide coordinate is exactly the internal equal-and-opposite axial
pair between the two bodies the joint connects. No reaction leaks to the world.

### The leverage ratio is applied by the engine, not by us

By virtual work `δW = F·δs`, so the axial shock force *is* the generalized force on
`shock_stroke`. MuJoCo then propagates it through the constraint Jacobian of the linkage,
which applies the instantaneous leverage ratio exactly, at the current configuration,
every step.

**Multiplying the spring force by an analytically computed leverage ratio would apply the
ratio twice.** This is called out explicitly because it is a natural-looking mistake in a
repository that already owns a high-precision leverage-ratio solver.

The analytical solver's role in ride mode is threefold, and none of it is in the force path:

1. sag autofit — choosing a spring rate for a target sag requires the ratio (§9);
2. telemetry — converting shaft stroke into wheel travel for display and CSV;
3. cross-validation — see below.

### Two representations of one linkage

The linkage exists twice: as the closed-form analytical solver
(`kinematics/solver.py`) and as MuJoCo's soft `<connect>` loop closures
(`solref="0.0005 1"`). Soft constraints deflect under load, so the two can drift apart.

Measured on the test stand under servo load:

| Wheel travel | MuJoCo | Analytic | Δ |
|---|---|---|---|
| 0 mm | −0.032 mm | 0 mm | −0.032 mm |
| 90 mm | 89.961 mm | 90 mm | −0.039 mm |
| 175 mm | 174.964 mm | 175 mm | −0.036 mm |

Shaft stroke agrees to 0.014 mm; peak `<connect>` violation is 0.0003 mm.

These are quasi-static loads. Ride mode applies impact loads orders of magnitude larger,
so this comparison is re-run under riding conditions and becomes an invariant test with a
tolerance set from measurement. Note that the existing `test_playground.py` leverage-ratio
assertion does **not** cover this: it compares telemetry against the solver, and both sides
of that comparison are analytical.

---

## 6. Propulsion and braking torque path

There is **no drivetrain model**: no chain, no cassette ratios, no motor speed–torque
curve, no cadence. Torque is applied directly to the wheel spin joints through `motor`
actuators with `gear="1"`.

**Cruise control** is a PI regulator on **chassis longitudinal velocity** (`qvel[root_x]`),
not on wheel angular velocity. Regulating wheel speed would command torque against a wheel
that is airborne or slipping, spinning it up without bound and producing a violent
re-engagement on landing. Torque is additionally gated off when the rear wheel has no
contact.

| Quantity | Value |
|---|---|
| Default target speed | 25 km/h (6.94 m/s) |
| Adjustable range | 15–45 km/h |
| Drive torque ceiling | 150 N·m at the rear wheel |
| Brake torque ceiling | 200 N·m per wheel |

The default speed is set by the kicker, not chosen for roundness. Measured against the
assembled profile, the 14° ramp at 25 km/h puts the bike down 4.54 m past the lip, on the
20° landing slope, at a flight-path angle of 36°. The whole jump works between 20.9 and
30.6 km/h; below that the bike cases into the gap, above it lands on the flat past the
slope. 25 km/h sits mid-band with 4.1 km/h of margin below and 5.6 km/h above.

**Brakes are sign-aware**: the applied torque opposes the current wheel angular velocity
and is zeroed near zero speed. A one-sided `ctrlrange` motor applied as a constant torque
would otherwise drive the wheel backwards at a standstill.

In the interactive viewer, braking is a **toggle** rather than a hold: MuJoCo's passive
viewer delivers key press events, not key state, so a held-lever input cannot be
represented faithfully.

---

## 7. Virtual rider

The rider is lumped rigidly into the `frame` body (80.00 kg of the 104.40 kg system).
Pitch management in flight is modelled as a **PD regulator applying a bounded moment to
`root_pitch`**, capped at ±80 N·m, active only while both wheels are out of contact.

Without it the aggressive preset is not measurable: after the 600 mm drop and the kicker
the bike noses over and everything downstream of 52 m is garbage. A real rider does manage
pitch in the air, and ±80 N·m is the order of magnitude an 80 kg rider can generate by
rotating their torso.

**This moment violates conservation of angular momentum.** See §12 — it is the single most
significant non-physicality in the model, and it is instrumented rather than hidden.

**Crash detection** runs independently of the rider: pitch beyond 60° or handlebar–ground
contact marks the run as failed, with the position and cause recorded. A stabilized run is
not assumed to be a successful one.

---

## 8. Energy and momentum accounting

Tracked per step and reported per run:

| Sources | Sinks |
|---|---|
| gravity (net −600 mm over the track) | fork and shock damping |
| drive torque | contact dissipation |
| **virtual rider moment** | rolling resistance (`Crr = 0.015`) |
| | brake torque |
| | joint damping |

The virtual rider is accounted **as a source on its own line**, never folded into "the
physics". Its injected angular impulse and work are separate telemetry channels, so a
reader can judge how much of a given run it paid for. If that share turns out to be
material, the honest fix is to make the rider a separate body on a hip joint so the
reaction becomes internal — deferred deliberately, with the measurement as the trigger.

Aerodynamic drag is absent from the sink column by decision (§4).

---

## 9. Static equilibrium and sag

A run starts from a **numerically solved static equilibrium**, written into `qpos`, rather
than from a settling drop. This removes the first half-second of transient from the
telemetry and makes headless runs reproducible from the first frame.

Rider is on by default. Measured static state:

| Quantity | Value |
|---|---|
| System mass | 104.40 kg (24.40 bike + 80.00 rider) |
| CoG from BB | (0.1504, 0.4750) m |
| Static front load | 478.2 N (46.7 %) |
| Static rear load | 546.0 N (53.3 %) |
| Ground plane | −349.50 mm from BB |
| Leverage ratio | 3.3495 (top) → 2.4118 (bottom) |

### Sag at the shipped defaults

The repository's spring defaults (`fork_initial_psi = 85.2`, `shock_stiffness = 114600`)
are derived at a **35 % front / 65 % rear** static split — the seated convention bike
manufacturers publish. At that split they are exact:

| Split | Front load | Rear load | Front sag | Rear sag |
|---|---|---|---|---|
| Documented 35 / 65 | 358.5 N | 665.7 N | 54.0 mm = **30.0 %** | 54.0 mm = **30.0 %** |
| Model's own CoG, 46.7 / 53.3 | 478.2 N | 546.0 N | 75.6 mm = **42.0 %** | 45.5 mm = **25.3 %** |

The model's own centre of mass does not produce a 35/65 split — it puts 46.7 % on the
front for the 80 kg standing attack pose in `RiderSpecs`. Springs calibrated for 35/65 and
loaded at 46.7/53.3 therefore sag in opposite directions at the two ends: the fork is
under-sprung and settles at 42 %, the shock is over-sprung and settles at 25.3 %.

**This is pre-existing and already documented** as Known Limitation #1 in the README, where
it is correctly classified as a ride-feel decision rather than a bug. Ride mode does not
introduce it — but ride mode is the first mode in which sag has dynamic consequences, so it
cannot be left at the defaults for a meaningful run.

### The `--sag` option

`physics/tuning.py::compute_suspension_tuning_for_sag` already performs exactly the
re-derivation the README calls for, and defaults `front_weight_fraction` to the model's own
CoG when it is not overridden. `--sag` therefore fits **both ends together** against the
model's own centre of mass and prints the result:

| Target 30 / 30 against the model's CoG | Value | Shipped default |
|---|---|---|
| Fork pressure | 118.5 psi | 85.2 psi |
| Shock rate | 93 984 N/m = 537 lb/in | 114 600 N/m = 654 lb/in |

Both fitted values are physically ordinary: 118.5 psi is mid-range for a 38 mm-stanchion
enduro fork, and 537 lb/in is a stock catalogue coil size. Bottom-out reserve is unchanged
at 6.11 g front and 4.64 g rear.

The shipped defaults are **not** silently changed: they are documented in the README and
referenced by existing tests. Fitting is opt-in per run, so a corrected run costs one flag
and a permanent change stays an explicit decision by the maintainer.

Note that this repository's sag convention is **30 % at both ends**. 30 % is a lot for a
fork by common enduro practice (15–20 % is more usual); the front target is therefore
settable separately, because ride mode is precisely where that choice becomes visible.

---

## 10. Numerical settings

| Setting | Value | Reason |
|---|---|---|
| `timestep` | 0.0005 | see below |
| `integrator` | `implicitfast` | matches the other modes |
| Joint limits | softened `solreflimit` | final barrier, not the bottom-out mechanism |

The suspension forces are integrated **explicitly** regardless of the integrator: MuJoCo
can treat implicitly only the terms it knows about, and the air spring, the coil and the
dampers are external forces it does not. The only defence against their oscillation is a
shorter step, hence 0.0005 rather than the 0.001 used by the stand modes.

Cost is not a concern. Measured with a wheel-sized sphere rolling on a heightfield of this
grid size at `timestep=0.0005`: **22.5× real time**. The interactive viewer runs at real
time; the headless reference runs at 0.0005 regardless.

MuJoCo's `accelerometer` sensor reports **proper acceleration**, verified directly: 0 in
free fall, 9.81 m/s² at rest. Bar and saddle accelerations therefore come from real
`<accelerometer>` sensors on dedicated sites — gated to ride mode so the three existing
golden baselines are untouched — rather than from finite-differencing `site_xpos`, which
would give noisy coordinate acceleration instead of what the rider feels.

---

## 11. Authored assumptions

Following this repository's convention of separating measurement from authorship: the
suspension hardpoints are photo-derived and refit; **everything in the list below is
authored** and carries no claim of correspondence to a specific real bike, tyre or trail.

| Assumption | Value |
|---|---|
| Track layout and obstacle dimensions | §2 |
| Rolling resistance coefficient | 0.015 |
| Tyre vertical stiffness / damping | 130 N/mm, 800 N·s/m |
| Sliding friction | 1.2 |
| Virtual rider moment ceiling | ±80 N·m |
| Coil preload and bumper characteristic | to be fixed with the implementation |
| Crash thresholds | 60° pitch, handlebar contact |
| Aerodynamic drag | omitted |
| Default cruise speed | 25 km/h |

---

## 12. Known non-physicalities

**1 — The virtual rider injects angular momentum.** A pure moment on `root_pitch` has no
reaction body, because the rider is rigid mass inside `frame`. A real rider pitches the
bike by counter-rotating their own body: an internal torque with a genuine reaction, and
total angular momentum in flight is conserved. Ours is not. This is why the ±80 N·m cap
is a damage limit rather than a realism parameter, and why §8 logs the moment's impulse and
work separately. The physical alternative — rider as a separate body on a driven hip joint —
was deferred because it removes the rider from the lumped `frame` mass and thereby moves
`physics/mass.py`, the CoG computation, `test_mass_distribution` and all three golden
baselines, for an effect confined to two or three seconds of flight in a sixteen-second run.

**2 — The spring defaults are calibrated for a weight split the model does not produce**
(§9). At the shipped settings the fork sags 42 % and the shock 25.3 %, in opposite
directions, from a single root cause: the springs were derived at 35/65 and the model's own
CoG loads them at 46.7/53.3. Both ends return to exactly 30 % at the documented split, so
the tuning chain is internally consistent — it is the convention that disagrees with the
centre of mass, exactly as README Known Limitation #1 states. Ride mode does not change the
defaults; it fits both ends behind `--sag` and reports what it used.

**3 — Two independent representations of one linkage** (§5), agreeing to 0.039 mm
quasi-statically, unverified under impact until the invariant test measures it.

**4 — No lateral dynamics** (§0), by construction.

**5 — No tyre slip model** (§4): regularized Coulomb friction only.

**6 — No drivetrain** (§6): torque is applied at the wheel.

**7 — No aerodynamic drag** (§4): wheel power reads ~90 W low at 25 km/h.
