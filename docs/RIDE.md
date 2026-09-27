# Ride Mode

Rolling whole-bike simulation over a longitudinal road profile with bumps, potholes,
rock gardens, a drop and a kicker. Complements the three existing modes
(`standard`, `stand`, `playground`), all of which bolt the frame to the world.

> **Status.** Part one of this document is the *Physics Contract* — the formal statement of
> what is simulated, how, and with what admitted inaccuracies. It was written before the
> implementation deliberately: every numbered subsection is a commitment the code honours,
> and several of the numbers in it were measured rather than assumed. Where the
> implementation later proved a contract figure wrong the section says so and gives both
> numbers (§9). Part two, *Usage*, documents the `bike-ride` command, track files, the
> telemetry channels and the summary metrics.
>
> **Pneumatic tyre (added 2026-09-26).** §3.1 and §4.1 specify a second, opt-in tyre model
> the same way: written ahead of its implementation
> ([plan](superpowers/plans/2026-09-26-pneumatic-tyre.md)). Figures marked *pending* are
> filled in by measurement as the plan's tasks land. Until the plan's task 13, `sphere` (§3.0,
> §4.0) is the default and everything else in this document describes it.

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
  - [7. The rider](#7-the-rider)
  - [8. Energy and momentum accounting](#8-energy-and-momentum-accounting)
  - [9. Static equilibrium and sag](#9-static-equilibrium-and-sag)
  - [10. Numerical settings](#10-numerical-settings)
  - [11. Authored assumptions](#11-authored-assumptions)
  - [12. Known non-physicalities](#12-known-non-physicalities)
- [Usage](#usage)
  - [Running a ride](#running-a-ride)
  - [Tracks: presets and track files](#tracks-presets-and-track-files)
  - [Rough-road generator](#rough-road-generator)
  - [Heightfield sizing](#heightfield-sizing)
  - [Viewer key map](#viewer-key-map)
  - [Telemetry channels](#telemetry-channels)
  - [Summary metrics](#summary-metrics)
  - [Reading the plots](#reading-the-plots)

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
| `rider_pelvis_z` | slide, frame Z | — | **seated rider only**, loaded via `qfrc_applied` |
| `rider_torso_z` | slide, frame Z (on pelvis) | — | seated rider only, `qfrc_applied` |
| `rider_arms_z` | slide, frame Z | — | seated rider only, `qfrc_applied` |
| `rider_leg_front_z` | slide, frame Z | — | seated rider only, `qfrc_applied` |
| `rider_leg_rear_z` | slide, frame Z | — | seated rider only, `qfrc_applied` |

`nq = nv = 12` with the bike alone or the lumped rider; **`nq = nv = 17`** with the seated
rider (§7), one vertical slide per lumped rider mass. `neq = 2` in every case.

**Rear linkage mobility.** Five coordinates (`main_pivot`, `horst_pivot`,
`rocker_frame_pivot`, `yoke_pivot`, `shock_stroke`) are reduced by two `<connect>` loop
closures (seatstay↔rocker at P3, shock↔frame at P7). Each `<connect>` is a 3-dimensional
ball constraint, of which two dimensions are effective in a planar linkage and one is
redundant, absorbed by the solver's regularization. Net rear suspension mobility is
therefore exactly **1 DOF**, as the analytical 4-bar solver assumes.

**Total mobility: 7** — three chassis, one fork, one rear suspension, two wheel spins,
plus the locked steer coordinate — **12 with the seated rider**, whose five slides are each
a genuine degree of freedom carried by a preloaded spring-damper.

**Wheel rotations are genuine dynamic degrees of freedom**, not auxiliary states. They
carry rotational inertia, they are driven by actuator torque and by the road — contact
friction with `sphere`, the tyre model's tangential force with `pneumatic` — and they can
spin up freely when the wheel leaves the ground. Any control law that assumes wheel speed
tracks ground speed is invalid here — see §6.

**The `pneumatic` tyre carries states outside MuJoCo** (§3.1, §4.1): each element's previous
deflection, elastic force and Maxwell force, and the transient slip `κ'` of each contact
patch (`fast`) or the bristle deflections of the tread (`detailed`). They are not generalized coordinates — they add no
mass and no constraint — but they are state: they are reset with the run, and at every
velocity reset of the equilibrium solve (§9), so a run is reproducible from `qpos` plus a
cleared tyre.

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

**The default geometry is fixed in the MJCF and is identical for every shipped track
preset.** Presets differ only in the numbers written into `model.hfield_data` from Python,
before the viewer is created. Consequences: the default `mode="ride"` model is exactly one
XML and one golden baseline, and the track is a pure Python object. Every shipped preset
fits within 120 m with 5 m of runout to spare and shares the 5 mm grid.

*Amendment (2026-09-26).* A track longer than 115 m gets a longer field derived from its
length — same 5 mm grid, same vertical envelope, more columns
(`HeightFieldSpec.for_track`, see [Heightfield sizing](#heightfield-sizing)).
`generate_mujoco_xml` therefore did gain one optional parameter, `field`; left at its
default it produces the golden XML byte for byte.

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

### Road-scale obstacles (added 2026-09-26)

The catalogue also carries defects at road rather than trail scale, used by the
`road_*` presets and by track files: `Bump` (raised cosine), `TrapezoidBump` (speed
bump: ramps and a plateau), `SlopedPothole` (chamfered edges, flat floor), `BowlPothole`
(cosine dip) and `RoadRoughness` (millimetre-scale correlated texture, built like
`RockGarden`, kept out of the plot markers). `Pothole` with its vertical walls is the
"sharp" pothole.

A rigid wheel of radius *r* cannot reach the floor of a hole shorter than 2*r* (0.744 m
front, 0.704 m rear). Its drop is bounded by geometry, `r − √(r² − (w/2)²)`: a 0.3 m
hole drops the front wheel ~32 mm and a 0.5 m hole ~100 mm whatever depth is declared.
`terrain/wheelpath.py` computes the rolling-wheel envelope over any obstacle, and the
summary, the preview and the generator report this **effective drop** next to the
declared depth.

---

## 3. Wheel–ground contact

Two tyre models, chosen per run (`TyreConfig.model`, `--tyre-model`):

| Model | What carries the wheel on the road | Status |
|---|---|---|
| `sphere` | MuJoCo contact of a sphere against the heightfield — §3.0, §4.0 | **default** |
| `pneumatic` | a Python tyre model against the road profile, applied through `xfrc_applied` — §3.1, §4.1 | opt-in until validated (plan D17) |

Everything outside §3 and §4 is shared unless a paragraph names a model.

### 3.0 `sphere` (default)

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

*Correction (2026-09-26).* Measured 29″ MTB tyres are about half as stiff and a fifth as
damped as these values: ≈ 62 N/mm static at 1.72 bar and 418 N, 100–190 N·s/m, 7–10 mm of
static deflection at this bike's loads (§11, sources there). The values above stay as the
`sphere` model's definition, because changing them would change every result recorded
against it; the physically parameterised alternative is §3.1.

**Friction must be set on both sides.** With equal geom priorities MuJoCo combines contact
friction as the element-wise maximum of the two geoms. Setting only one side silently
inherits the other's value.

**Frame contacts are left enabled**, as in the existing modes, so a crash renders as a bike
on the ground rather than one sinking through it while the crash detector (§7.4) flags it
anyway. **The rider and the crankset do not collide** (`contype="0"`): the rider's capsules
are mass and visuals, and a rider on the ground is a crash the detector has already called
from the frame's attitude and the handlebar; the horizontal cranks and pedals sit 350 mm
above the road and would only meet it in that same crash. (Earlier revisions of this
section said rider contact was enabled; the builder never did that.) This paragraph holds
for both tyre models.

### 3.1 `pneumatic`

**Why not MuJoCo contact.** A pressure-dependent, non-linear load–deflection law, a rim
behind the carcass and a slip-dependent tangential force are not expressible in `solref`,
`solimp` and a Coulomb cone. The road, on the other hand, is known in closed form (§2): the
tyre does not need collision detection to find it. So in `pneumatic` the wheels do not
collide at all, and a force model evaluates each wheel against `h(x)` once per step. The
sphere–heightfield dropouts of §12 item 8 do not exist in this model.

**What changes in the compiled model — only this.** The two contact spheres get
`contype="0" conaffinity="0"`. They stay in the model, because `resolve_wheel_spin` reads
the wheel radius from them. The tyre cylinders already do not collide (§3.0). The
heightfield and the catch plane are unchanged: they render the road and carry the frame and
handlebar collisions of the paragraph above. `detailed` (below) sets `model.opt.timestep`
after compilation; the XML keeps 0.0005.

**Road seen by the tyre.** The same profile the heightfield is rasterised from —
`build_profile(track, field.track_x())`, metres, 5 mm — linearly interpolated. Physics and
picture therefore cannot disagree. The road is rigid; its surface properties come from §4.1.

**Geometry.**

| Quantity | Front (Magic Mary 29×2.4) | Rear (Hans Dampf 27.5×2.4) |
|---|---|---|
| Outer radius `R` | 0.372 m (`BikeSpecs`) | 0.352 m |
| Rim radius `R_rim` (builder's rim geom) | 0.320 m | 0.300 m |
| Tyre height above the rim | 52 mm | 52 mm |
| Casing width `W_c` | 60 mm [est] | 60 mm [est] |
| Compressed casing and tread at rim strike `t_c` | 6 mm [est] | 6 mm [est] |
| Rim-strike deflection `δ_rim = R − R_rim − t_c` | **46 mm** | **46 mm** |

`δ_rim` is 0.84 of a 55 mm section, inside the 0.80–0.85 band derived from drop tests
(§11). Tubeless, no insert (plan D5).

**Radial elements.** Each wheel carries `N` rays from the hub centre, uniformly spaced over
**±75°** about world −Z. The rays are fixed in the world frame: they neither spin with the
wheel nor pitch with the bike. The tyre is axisymmetric, so any non-spinning frame gives the
same forces, and the world frame keeps the coverage pointed at the road whatever the pitch.
±75° sees an edge up to `R(1 − cos 75°)` ≈ 0.74 R = 275 / 261 mm above the wheel's lowest
point. The far edge of a 150 mm pothole meets the tyre about 55° off vertical, and its
patch extends several degrees beyond that, which is why ±60° is not enough. Road met
outside the coverage is logged as a **coverage event**; a shipped preset that raises one is
a bug in this section.

For ray `i` with unit direction `u_i`, `r_i` is the distance from the hub centre to the
first **visible** road point along the ray. Road hidden behind a nearer part of the profile
along the same ray, such as the back face of a steep descent, is discarded, not clipped. The
element deflection is `δ_i = max(0, R − r_i)`.

**Element force.** Per ray, over its share of arc `Δs = R·Δθ`, directed along `−u_i`, the
element's **elastic** force is

`f_e,i = [ c_A · p · w(δ_i) + k_c · δ_i ] · Δs`

- `p` is gauge pressure. It is read every step, so it can change mid-run (Usage, key map).
- `w(δ) = 2·√(δ·(2ρ − δ))` is the chord of the casing cross-section, radius `ρ = W_c/2`, at
  depth `δ`. It saturates at `2ρ` for `δ ≥ ρ`.
- `c_A ≤ 1` is an effective-area factor. It stands for the knob voids and the bulge that
  make the real footprint smaller than the geometric one.
- `k_c` is the carcass stiffness per unit arc length.

The total element force adds rate stiffening and hysteresis (below) and is clamped at zero
from below: a tyre pushes on the road but never pulls it.

**Contact length.** A circle pressed into a plane has a chord of `2·√(2Rδ)`. At the
measured deflections that chord is about 15 % longer than the measured footprint, because
a real crown flattens and part of the deflection happens outside the footprint. The patch
length the brush uses (§4.1) and the telemetry reports is therefore the geometric span of
the loaded rays scaled by a fitted **contact-length factor** `c_L ≤ 1`. The element forces
are not scaled.

`c_A`, `k_c` and `c_L` are fitted, per wheel, to three targets:

1. Static stiffness on flat road follows `k(p) ≈ 22 + 24·p[bar]` N/mm within ±15 % over
   1.0–2.0 bar.
2. Contact length at 418 N is 133 mm at 1.38 bar and 122 mm at 1.72 bar, within ±15 %.
3. The pressure term carries 70–100 % of the load at nominal pressure.

Fitted values (1024-ray flat-road fit; the 256-ray `detailed` tier also passes the targets):

| Wheel | `c_A` | `k_c`, N/mm² | `c_L` | `k_r` | `η` |
|---|---:|---:|---:|---:|---:|
| Front, Magic Mary | 0.4435 | 0.1194 | 0.8589 | 0.2484 | 0.0662 |
| Rear, Hans Dampf | 0.4576 | 0.1225 | 0.8801 | 0.2498 | 0.0690 |

The law is mildly progressive by construction, because the loaded length and width both
grow with deflection.

**Material rate.** The rays are fixed in space and the tyre rotates through them, so an
element's rate of deflection is taken **along the material**, not at the ray:

`Dδ_i/Dt = (δ_i − δ_i,prev)/dt − ω · ∂δ/∂θ |_i`

Here θ is measured from −Z towards the front, material at the bottom of a forward-rolling
wheel moves towards −θ, and `∂δ/∂θ` is an upwind difference along the ray grid. The second
term is not a refinement. In steady rolling on flat road every ray's deflection is
constant, so without it the leading half of the patch would never be seen compressing and
rolling resistance could not arise.

States that belong to the material rather than to a ray — the Maxwell force below, and the
tread's bristles in §4.1 — are **advected** with it: shifted by `−ω·dt` along the ray grid
semi-Lagrangian each step, before the step's update.

**Rate stiffening.** Rubber is stiffer when loaded fast. The drop tests measure dynamic
stiffness at 1.16–1.35 × static: 1.38 × on first impact, 1.19 × in the settled 6.5 Hz
oscillation [T2]. Neither the elastic law nor hysteresis produces that: hysteresis adds
only `η` ≈ 7 % while compressing. Each element therefore carries a **Maxwell branch** in
parallel with its elastic force, which makes it a standard linear solid:

`f_m,i ← e^(−dt/τ) · f̃_m,i + k_r · (f_e,i − f̃_e,i)`

The tildes are last step's values advected with the material, so the branch is driven by
the material increment of the element's own elastic force, and scales with its local
tangent stiffness.

| Parameter | Value | Role |
|---|---|---|
| `k_r` | 0.2484 front / 0.2498 rear | high-frequency stiffness `(1 + k_r)` × static; drop-sled ratios are 1.256 front / 1.255 rear |
| `τ` | 0.2 s [est] | fully stiff at wheel-hop frequencies and over the ~20 ms an element spends in a rolling footprint; relaxed within a second in a static load |

Sag therefore still follows the static law, and the equilibrium solve (§9), which clears
the tyre's states every 20 ms, converges onto it. Added damping at 6.5 Hz is about 1 % of
critical, and it counts towards the damping band below.

**Hysteresis.** `f_h,i = η · f_e,i · tanh((Dδ_i/Dt) / δ̇_ε)`, with `δ̇_ε = 0.01 m/s`. It is
**rate-independent**: the loss per load cycle does not depend on frequency, as for rubber,
and the `tanh` only regularises the sign change through zero. This choice follows from the
drum data. A rolling tyre's leading half is compressing and its trailing half is
recovering, so hysteresis shifts the centre of pressure forward and produces a
rolling-resistance moment that is nearly speed-independent, as drum tests show. Viscous
element damping would make that moment grow linearly with speed.

`η` is calibrated in this order:

1. **Vertical damping ratio** of the free oscillation of a 44 kg drop sled on the tyre must
   land in 2–5.5 % (loss factor 0.05–0.09), rate stiffening included. This is the quantity
   that reaches the rider, which is what ride mode exists to study (§0), so it has priority.
2. **Rolling resistance** on flat asphalt at 20 km/h, 490.5 N and 1.5 bar must match the drum
   targets within ±15 %: Crr 0.0103 rear, ≈ 0.011 front.

If an `η` inside the damping band leaves Crr more than 15 % short, the shortfall is closed
by a **tread-loss term** on each patch, `F_roll = (Crr_target − Crr_η) · N_p`, opposing
rolling and tapered through zero wheel speed like `opposing_torque`. This is the model's one
phenomenological term. On the 20 km/h, 490.5 N reference roll, the carcass alone gives Crr
0.00514 front and 0.00549 rear, more than 15 % below the drum targets. The fitted
`tread_loss_crr` values are 0.00586 front and 0.00481 rear at 1.5 bar; they scale with
`p⁻⁰·³`. With that correction, the simulated Crr is 0.01100 front and 0.01030 rear at the
reference pressure. Across 1.3–1.8 bar the fitted exponents are −0.335 and −0.340.

The current hysteresis fits are `η` = 0.0662 front and 0.0690 rear. A pure-NumPy 44 kg sled
dropped 10 mm onto the tyre gives mean log-decrement damping ratios of ≈3.75 % on both tyres
over the first three rebound cycles. A small viscous element term is permitted only if the
damping ratio falls below 2 % with `η` at the top of its band.

**Rim.** For `δ_i > δ_rim` the element adds `k_rim·(δ_i − δ_rim)·Δs`, carrying the same
hysteresis. `k_rim = 2.8×10⁷ N/m²` makes a 50 mm rim patch add ≈ 1 400 N/mm. This is stiff
compared with the tyre and keeps the 0.5 ms explicit step stable for the compiled front-wheel
mass (below). A **rim-strike event** opens when any element first exceeds `δ_rim` and closes
when none does. It records `x`, speed, peak wheel load, peak rim force and the peak elastic
energy stored in the rim term. The tyre stays inflated: there is no puncture and no pressure
loss (plan D11).
For scale, Schwalbe's edge-drop test damages a Super Trail casing at 61 J and a Super
Gravity casing at 95 J (§11). The event energy can be read against those figures, but no
threshold is enforced.

**Patches.** Each contiguous run of loaded rays is a patch `p`. Flat road gives one patch, a
square edge two (the flat and the face), and the wheel is airborne when there are none. A
patch has:

- normal load `N_p`: the magnitude of the vector sum of its element forces;
- direction `n_p`, and tangent `t_p ⟂ n_p` in the sagittal plane, pointing forward;
- centroid `c_p`: the force-weighted mean of the element road points;
- length `2a_p`: the distance between its first and last road points;
- mean deflection `δ_p`.

**Application.** The tangential force `F_t,p` comes from §4.1. Each patch force
`F_p = N_p·n_p + F_t,p·t_p` acts at `c_p`. Per wheel, the tyre applier **assigns**

`xfrc_applied[wheel] = [ Σ_p F_p ,  Σ_p (c_p − x_i) × F_p ]`

where `x_i` is the wheel body's centre of mass (`data.xipos`). It does this on every step,
zero included, because MuJoCo never clears `xfrc_applied`. The applier is the array's only
writer. The moment about the axle is what drives and brakes the wheel against the road: no
separate wheel torque is written. The drive and brake actuators act on the spin joints
exactly as in §6, and their reaction reaches the road through `F_t`. Velocities used by the
tyre come from `mj_objectVelocity` on the wheel body in world axes, so ω is the wheel's
**absolute** spin. That differs from `qvel[*_wheel_spin]`, which is relative to a fork or
swingarm that itself pitches.

**Loads for gating.** In `pneumatic`, `TerrainContacts` takes its wheel channels from the
tyre:

- `*_load_n` is `Σ_p N_p`, unbridged, because nothing drops out;
- `*_support_n` is the vertical component of `Σ_p N_p·n_p`;
- the handlebar channel still comes from MuJoCo contact.

The contact snapshot is built from the tyre evaluated at the top of the step, before the
other writers run, so every writer sees one snapshot as before (§6).

**Stability budget.** The tyre force is integrated explicitly, like the suspension (§10),
so `ω·dt ≤ 0.4` must hold for the stiffest element set a wheel can see. The tyre alone gives
`√(63 000 / 2.6) · 0.0005 ≈ 0.08`. At 2 bar, the fitted high-frequency carcass stiffness is
at most 87.5 N/mm. Together with the 50 mm rim patch (1 400 N/mm), this gives
`ω·dt = 0.394` for the compiled 2.40 kg front wheel and `0.365` for the 2.80 kg rear wheel.
The `detailed` tier halves those values with its 0.25 ms step. A test asserts the bound from
the compiled wheel masses.

**Tiers.** The carcass and surface laws are shared. `detailed` also gives each bristle a
0.5 ms dashpot; this regularizes a self-excited wheel-slip mode measured near 33 Hz when the
per-bristle spring was coupled to wheel rotation. The dashpot is a numerical regularizer for
the resolved brush, not a measured tyre property. The full-traverse convergence test covers
the resulting ride metrics.

| Tier | Rays over ±75° | Spacing at the tread | Tread model (§4.1) | Timestep |
|---|---|---|---|---|
| `fast` | 64 | ≈ 15 mm | lumped (steady-state) brush | 0.0005 s |
| `detailed` | 256 | ≈ 4 mm | discretised brush | 0.00025 s |

`fast` must keep the interactive viewer at ≥ 1× real time. `detailed` may run 2–3× slower
and exists as `fast`'s reference: a convergence test holds `fast`'s summary metrics to
`detailed`'s. The final ray counts are *pending (plan task 10)*.

**Tyre mass** stays on the wheel body. There is no separate belt or ring body, so the
masses and inertias of §3.0 and `test_mass_distribution` are untouched.

---

## 4. Longitudinal friction and slip

### 4.0 `sphere` (default)

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
damping on the wheel spin axes (0.01 N·m·s/rad on both wheels; earlier revisions said
"0.05 and 0.01", the builder has always emitted 0.01) is a constant and does not scale
with load, so it cannot represent a loss that doubles in a G-out. It stays in both models
as bearing loss.

### 4.1 `pneumatic`: brush slip

**Surface.** One `SurfaceSpec` per run (`terrain/surface.py`). The tyre asks
`SurfaceMap.at(x)` for it and never holds a surface itself, so per-zone surfaces in a track
file can be added later without touching the tyre (plan D9). The default comes from the
track: the `road_*` presets and track files with a `[generator]` block are `asphalt`,
everything else is `hardpack`. `--surface` overrides the default. All values are authored
(§11):

| Surface | μ peak | μ sliding | C_κ/F_z | Stribeck speed |
|---|---|---|---|---|
| `asphalt` | 1.05 | 0.75 | 15 | 4.5 m/s |
| `hardpack` | 0.80 | 0.60 | 12 | 4.5 m/s |
| `loose` | 0.55 | 0.45 | 7 | 4.5 m/s |
| `wet` | 0.50 | 0.40 | 10 | 4.5 m/s |

`V_str = 4.5 m/s` is an authored numerical choice: with the `lumped` curve it keeps peak
force within 10 % of each surface's `μ_peak` from 15 to 45 km/h.

Friction falls from static to sliding with the sliding speed:
`μ(V_s) = μ_slide + (μ_peak − μ_slide) · exp(−|V_s| / V_str)`.

The lumped curve's peak-slip estimates on the uniform reference patch (`N = 418 N`,
`a = 61 mm`) are recorded below as **braking / drive** pairs. They are estimates from the
authored curve, not acceptance limits; the test asserts peak force, not `κ_peak`.

| Surface | 15 km/h | 30 km/h | 45 km/h |
|---|---|---|---|
| asphalt | −0.153 / 0.206 | −0.144 / 0.186 | −0.138 / 0.174 |
| hardpack | −0.149 / 0.199 | −0.141 / 0.182 | −0.135 / 0.172 |
| loose | −0.173 / 0.248 | −0.166 / 0.229 | −0.161 / 0.218 |
| wet | −0.119 / 0.151 | −0.114 / 0.141 | −0.110 / 0.135 |

**Slip, per patch.**

- `V_x = v_hub · t_p` is the hub's speed along the patch tangent.
- `R_e = R − δ_p/3` is the effective rolling radius of a loaded pneumatic tyre.
- `V_s = V_x − ω · R_e` is the tread's sliding speed over the road, with ω the absolute spin
  (§3.1).
- `κ = −V_s / |V_x|` is the longitudinal slip: positive when driving, −1 for a locked wheel.

**Transient slip.** `σ · dκ'/dt + |V_x| · κ' = −V_s`, with relaxation length σ = 60 mm
[est]. It is integrated with its exact exponential solution, which is stable for any
timestep and finite at `V_x = 0`. At rest with no torque, `κ'` holds its value like a
deflected tread, so a parked bike does not creep. Under a torque at rest, the force builds
over a distance rolled, not instantaneously. That is the property that makes a slip model
well-posed near zero speed, where `κ` itself is 0/0.

**Tread stiffness.** `c_px = (C_κ/F_z)_surface · N_p / (2·a_p²)`. The initial slope of the
brush curve then equals the surface's normalised slip stiffness at every load, and the
contact half-length `a_p` comes from §3.1. This is where pressure reaches traction: a softer
tyre has a longer patch, and the curve changes with it.

**Tangential force, by tier.**

- *`fast`: lumped brush.* The steady-state brush with a parabolic pressure distribution,
  evaluated at `κ'`: `σ_x = κ'/(1 + κ')`, `θ = 2·c_px·a_p² / (3·μ·N_p)`,
  `F_t = 3μN_p·θσ_x·(1 − |θσ_x| + (θσ_x)²/3)` for `|θσ_x| < 1`, and `μN_p·sign(σ_x)`
  beyond. `σ_x → −∞` as `κ' → −1`, which is the locked wheel, fully sliding. The patch is
  **fully sliding** when `|θσ_x| ≥ 1`.
- *`detailed`: discretised brush.* One bristle per ray inside the patch. Each bristle's load
  share is that element's own carcass force, so on an edge the pressure distribution is the
  real, lopsided one rather than a parabola. Bristle deflections are advected at the measured
  tread speed by a semi-Lagrangian shift, which stays stable at a Courant number above 1:
  45 km/h at 4 mm and 0.25 ms is about 0.8, and the scheme does not rely on staying below 1.
  The bristles grow with the filtered sliding speed. The trial force is
  `c_px·Δx·(q_j + τ_b·(−V'_s))`, clipped to `±μ·f_j`; here `q_j` is bristle deflection,
  `τ_b = 0.5 ms` is the dashpot time constant, and `V'_s` is the filtered sliding speed.
  The patch is fully sliding when every bristle reaches its local Coulomb cap. Transporting
  bristles through the patch supplies about `a_p` of relaxation, so the input filter uses
  `max(0, σ − a_p)`.

The 60 mm relaxation length is the lower end of the authored 60–120 mm estimate. The detailed
dashpot removes the 33 Hz mode while the uniform-patch steady force remains within 4 % of the
lumped brush; the full-ride convergence test checks both tiers on `flat` and `single_edge`.

**Consequences the rest of the contract relies on.**

- **Wheelspin** is the rear patch fully sliding with drive torque applied. **Lock-up** is a
  patch fully sliding under brake torque. Both are logged (Usage, summary metrics).
- **Rolling resistance** in `pneumatic` is the hysteresis moment of §3.1, plus the tread-loss
  term only if §3.1 found it necessary. `RollingResistance` (`Crr·N·r`) is not applied.
- The traction limit is no longer a fixed `μN`. On `asphalt` at the static rear load of
  610.9 N (seated rider, §9) it peaks near `1.05 · 610.9` ≈ 641 N, or ≈ 224 N·m at the rear
  wheel's `R_e`, and falls to `μ_slide` once the tyre slides. The 150 N·m drive ceiling sits
  inside it, and `hardpack` (≈ 170 N·m) still clears it. On `loose` (≈ 117 N·m) and `wet`
  (≈ 107 N·m) full drive spins the rear wheel on flat ground. That is intended physics, and
  it is logged, not suppressed (plan D12).

**Both models.** **Aerodynamic drag is not modelled.** At 25 km/h this omits roughly 13 N, about 90 W —
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
contact. That gate reads a **debounced** contact signal: MuJoCo's sphere–heightfield
collision drops the contact row of a demonstrably loaded wheel on a speed-dependent share
of steps (measured on flat ground: 2.0 / 14.7 / 27.2 / 43.2 % at 15 / 25 / 35 / 45 km/h),
so `sim/ride/contacts.py` holds each wheel's last measured load for up to 10 steps (5 ms)
before declaring it airborne. See §12 item 8. With `pneumatic` the gate reads the tyre's own
load (§3.1), which has no dropouts, so nothing is debounced.

The integrator's anti-wind-up is unchanged for `sphere`: the accumulator is clamped, and
integration pauses while the output is saturated in the direction of the error or the rear
wheel is airborne. With `pneumatic` it also pauses while the rear patch is **fully sliding**
in the drive direction (§4.1). Error accumulated against a spinning tyre would otherwise
land as a torque spike when grip returns. There is no traction control and no ABS: a
spinning or locked wheel is physics to be observed, not suppressed (plan D12).

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

The fade is linear below `BRAKE_TAPER_RADPS = 1.0` rad/s, and it has a consequence under
`pneumatic`. A wheel that the brake would lock settles with ω inside the taper band — at
most 0.35 m/s of rim speed, against the 6.9 m/s of a 25 km/h run — rather than at exactly
zero. The tyre sees `κ` of about −0.95 and a fully sliding patch, which is a lock-up in
every sense §4.1 measures. Lock-up is therefore reported from the patch state, never from
`ω = 0`. The brake taper stays: it is what keeps the torque from chattering in sign at rest.

In the interactive viewer, braking is a **toggle** rather than a hold: MuJoCo's passive
viewer delivers key press events, not key state, so a held-lever input cannot be
represented faithfully.

---

## 7. The rider

Ride mode has three rider variants, chosen with `bike-ride --rider {none,lumped,seated}`
(`seated` is the default). One source of truth describes all of them:
`physics/rider.py::RiderSpecs` — mass, stature, inseam, the load split, the interface
dynamics — and every number in it is tagged *literature*, *derived* or *authored*.

### 7.1 `none` and `lumped`

`none` is the 24.35 kg bike alone. `lumped` is the original rider: 80.00 kg in three
capsules (torso + helmet 55, legs 18, arms 7) in a standing attack pose, **rigidly part of
the `frame` body**. It is kept as the regression reference and as the rider for the
flight-heavy presets (§7.4).

### 7.2 `seated` — a biodynamic rider on the saddle

The seated rider is the model for rough roads. A seated human is not a rigid mass: the
body is a low-pass filter with a resonance near 5 Hz, and how the road reaches the rider
depends on that as much as on the suspension. The model is the lumped-parameter class the
whole-body-vibration literature uses (ISO 5982; Fairley & Griffin 1989; Wei & Griffin 1998;
Kumar & Saran 2019; for bicycles Wang & Hull 1997), reduced to the sagittal plane:

| Body (MJCF) | Mass, 80 kg rider | Carried by | Spring k / damper c | Sided |
|---|---|---|---|---|
| `rider_pelvis` | 14.11 kg | saddle → pelvis | 101 000 N/m / 2 762 N·s/m | one-sided |
| `rider_torso` | 29.89 kg | pelvis → torso ("spine") | 57 800 N/m / 1 052 N·s/m | two-sided |
| `rider_arms` | 9.60 kg | handlebar → arms | 6 064 N/m / 193 N·s/m | two-sided |
| `rider_leg_front` | 13.20 kg | front pedal → leg | 13 028 N/m / 332 N·s/m | one-sided |
| `rider_leg_rear` | 13.20 kg | rear pedal → leg | 13 028 N/m / 332 N·s/m | one-sided |

Each body sits on a **vertical slide joint along the frame's Z** and is carried by a
spring-damper written into `qfrc_applied` by `sim/ride/rider_forces.py`, the same path the
fork and shock use (§5). The springs are preloaded to their static loads at zero travel, so
the equilibrium pose is the drawn pose. The saddle and the flat pedals are **one-sided**: they
push but cannot pull, so the rider can leave the saddle and come back under gravity, and the
`saddle_gap_m` channel records it. The spine and the gripped bar are two-sided.

**Reactions are internal.** A generalized force on a slide joint acts between the joint's
child and its parent, so every newton lifting the pelvis presses the frame down through the
saddle. Unlike the virtual rider moment (§7.4, §12) nothing here lacks a reaction body.

**Mass allocation (de Leva 1996, males).** Segment masses are de Leva's fractions of body
mass — head 6.94 %, trunk 43.46 % (upper 15.96 / middle 16.33 / lower 11.17), upper arm
2.71, forearm 1.62, hand 0.61, thigh 14.16, shank 4.33, foot 1.37 % per limb — with a 0.4 kg
helmet on the head. They are assigned to the three load paths so that the **static split is
saddle 55 % / pedals 33 % / bar 12 %** of rider weight: the arms path is the arms plus the
shoulder girdle it takes to reach 12 %; each leg is shank + foot plus the thigh it takes to
reach 16.5 %; the saddle path is the rest, stacked as torso (upper + middle trunk, head,
helmet) on pelvis (lower trunk, remaining thigh). The split itself is *extrapolated from
literature*: Carahalios (2015) measured 44 / 41 / 15 % (saddle / bottom bracket / stem) at
2 W/kg on the hoods, shifting 5.2 pp from the saddle and 3.3 pp from the bars to the pedals
per W/kg, which at 0 W/kg — coasting — is about 54 / 34 / 12 %; Wilson et al. (2007) put
49–52 % on the saddle at 125 W. Those are road postures; an upright MTB posture moves bar
load to the saddle. The solved equilibrium reproduces the target to 0.1 pp
(`tests/test_ride_equilibrium.py`).

**Springs.** The pelvis-to-saddle contact is Kumar & Saran (2019) Table 2, K8 / C8, the
path beneath the pelvis of a subject seated upright on a hard seat — *literature*. The torso
spring is *derived*: its uncoupled resonance is 7.0 Hz so that, coupled with the saddle
contact, the two-mass saddle path's **apparent mass peaks at 4.8 Hz at 1.6 × the static
mass**, which is where shaker measurements of seated humans put it (Fairley & Griffin 1989:
4–6 Hz, ~1.5 ×; Kumar & Saran 2019: 5 Hz). The damping ratio of every derived spring is
0.40; fitted seated-body models give 0.3–0.45 (Wei & Griffin 1998: k₁ = 42.9 kN/m,
c₁ = 721 N·s/m on 31 kg → ζ 0.31; Muksian & Nash via Turner 2024: 50 kN/m, 1 kN·s/m on
66 kg → 0.28; Kumar & Saran's fits are heavier). The legs (5 Hz) and arms (4 Hz) are
*authored*: the measured values are Wang & Hull (1997) Table 2, which this repository could
not read; replace them when it can. `physics/rider.py::saddle_path_apparent_mass` computes
the coupled response and `tests/test_ride_equilibrium.py` asserts the peak against the
literature band, so a change to any of these numbers is caught.

### 7.3 Pose and saddle height

The pose is solved, not drawn (`physics/rider.py::solve_seated_pose`):

- **Saddle height** follows the rider: inseam = 0.47 × stature (ANSUR II, approximate;
  `--rider-inseam` overrides) and saddle height = 0.883 × inseam (LeMond), measured BB centre
  to saddle top. For the default 1.80 m rider that is 0.747 m, **+36 mm over the
  photograph's saddle**, which the `none` and `lumped` variants keep. The seatpost follows
  along its measured axis and the analytic mass table reads the same geometry
  (`geometry/cockpit.py`). Exposed post outside 50–250 mm is refused with a message.
- **Hips** sit 60 mm above the saddle top; **feet** stand on the pedals of 165 mm horizontal
  cranks (3 and 9 o'clock, coasting), ankle 115 mm above the spindle for a toe-down foot.
  Knees follow by two-link inverse kinematics; the knee at bottom dead centre is checked to
  lie within 20–60° of flexion (30° for the default rider — where fitters put it) and the
  model refuses to build otherwise, naming the saddle height, inseam and crank length.
- **Torso lean** is whatever lets an arm with a 15° elbow reach the handlebar centre:
  42° from vertical for the default rider on this frame.
- Segment lengths are de Leva's, scaled by stature; capsule radii are drawing choices. Every
  rider geom is non-colliding: rider–ground contact is a crash the detector already calls.

The bike keeps its own accelerometers at the bar and saddle; the seated rider adds two on
the torso and pelvis, so the body's transmissibility is measured, not assumed.

### 7.4 Pitch management in flight (all variants)

Pitch management in flight is modelled as a **PD regulator applying a bounded moment to
`root_pitch`**, capped at ±80 N·m, active only while both wheels are out of contact — as
judged by the same debounced contact signal the drive gate uses (§6), so a one-step
collision dropout on flat road cannot fire it.

Without it the aggressive preset is not measurable: after the 600 mm drop and the kicker
the bike noses over and everything downstream of 52 m is garbage. A real rider does manage
pitch in the air, and ±80 N·m is the order of magnitude an 80 kg rider can generate by
rotating their torso.

**This moment violates conservation of angular momentum.** See §12 — it is the single most
significant non-physicality in the model, and it is instrumented rather than hidden. It is
unchanged for the seated rider, whose slides are vertical only: a seated rider is a
rough-road model, and a rider does not sit through a 600 mm drop. `bike-ride` therefore
**warns** when the seated rider is sent over a track with drops or kickers and points at
`--rider lumped`, which is what `tests/test_ride_track.py` rides the aggressive preset with.

**Crash detection** runs independently of the rider: pitch beyond 60° or handlebar–ground
contact marks the run as failed, with the position and cause recorded. A stabilized run is
not assumed to be a successful one.

---

## 8. Energy and momentum accounting

Tracked per step and reported per run:

| Sources | Sinks |
|---|---|
| gravity (net −600 mm over the track) | fork and shock damping |
| drive torque | contact dissipation (`sphere`) |
| **virtual rider moment** (flight, §7.4) | rolling resistance (`sphere`: `Crr = 0.015`) |
| | **tyre hysteresis** (`pneumatic`: carcass and rim elements, plus the tread-loss term if §3.1 needs it) |
| | **tyre sliding** (`pneumatic`: `F_t · V_s` per patch) |
| | brake torque |
| | joint damping |
| | **seated rider damping** (saddle, spine, arms, legs) |

With `pneumatic`, the two tyre sinks replace contact dissipation and `Crr·N·r`. They are
recorded per wheel as `*_tyre_loss_w`, and summed over rim-strike events as each event's
energy (§3.1).

The virtual rider moment is accounted **as a source on its own line**, never folded into
"the physics". Its injected angular impulse and work are separate telemetry channels, so a
reader can judge how much of a given run it paid for. The seated rider's spring-dampers are
different in kind: they are internal forces with reactions on the frame, so they move no
momentum; their dampers are a genuine sink — the energy a real rider's tissue absorbs from
a rough road — and their loads are recorded per interface (`saddle_load_n`,
`pedal_load_*_n`, `bar_hand_load_n`). Making the flight moment internal too would take a
rider that stands and moves fore-aft; that remains deferred (§12, item 1).

Aerodynamic drag is absent from the sink column by decision (§4).

---

## 9. Static equilibrium and sag

A run starts from a **numerically solved static equilibrium**, written into `qpos`, rather
than from a settling drop. This removes the first half-second of transient from the
telemetry and makes headless runs reproducible from the first frame.

**With `pneumatic`** the tyre applier joins the relaxation beside the suspension and rider
forces, and its transient state (§1) is cleared at every velocity reset. The relaxation
cycle is specified in time, 20 ms, which is 40 steps at 0.5 ms and 80 at `detailed`'s
0.25 ms. The axle loads below do not depend on the tyre model, because they are set by the
centre of mass. With the default seated rider and `fast` tier on the flat preset, pneumatic
equilibrium gives 59.4 mm (33.0 %) front sag and 46.2 mm (25.7 %) rear sag, nearly unchanged
from `sphere` (59.7 / 46.1 mm). The tyre's mean loaded-ray deflection is 5.84 mm front and
7.36 mm rear; fitted patch lengths are 118.4 and 140.1 mm. The solved root height is
−57.91 mm, about 8.99 mm below the `sphere` result (−48.92 mm), while pitch is 0.00403 rad.
The tyre outputs support 425.0 N front and 597.9 N rear, 1022.9 N total against 1023.7 N
system weight (0.08 % residual). The sag tables below remain the `sphere` reference.

The rider is on by default (`seated`; §7). Analytic static state, both rider variants:

| Quantity | `lumped` | `seated` (default) |
|---|---|---|
| System mass | 104.40 kg (24.40 bike + 80.00 rider) | 104.40 kg |
| CoG from BB | (0.1504, 0.4750) m | (0.0693, 0.6841) m |
| Static front load | 478.2 N (46.7 %) | 413.3 N (40.4 %) |
| Static rear load | 546.0 N (53.3 %) | 610.9 N (59.6 %) |
| Ground plane | −349.50 mm from BB | same |
| Leverage ratio | 3.3495 (top) → 2.4118 (bottom) | same |

The seated rider's centre of mass is 81 mm further back and 209 mm higher than the lumped
rider's: seated, the weight goes to the saddle behind the BB rather than to the pedals and
bar in front of it. The rider's own interfaces carry, at equilibrium, saddle 431 N / pedals
259 N / bar 94 N — 55.0 / 33.0 / 12.0 % of rider weight, the designed split (§7.2).

### Sag at the shipped defaults

The repository's spring defaults (`fork_initial_psi = 85.2`, `shock_stiffness = 114600`)
are derived at a **35 % front / 65 % rear** static split — the seated convention bike
manufacturers publish. At that split, to first order, they are exact: 54.0 mm = 30.0 % at
both ends.

The model's own centre of mass does not produce a 35/65 split — it puts 46.7 % on the
front for the 80 kg standing attack pose in `RiderSpecs`. What the shipped springs settle
to under the model's own load depends on how the question is asked:

| Method (`lumped` rider) | Front sag | Rear sag |
|---|---|---|
| First-order analytic: whole axle load through the spring, evaluated at the undeflected CoG | 75.6 mm = 42.0 % | 45.5 mm = 25.3 % |
| **Solved equilibrium — what the simulation produces** | **73.1 mm = 40.6 %** | **40.9 mm = 22.7 %** |
| **Solved equilibrium, `seated` rider (the default run start)** | **59.7 mm = 33.2 %** | **46.1 mm = 25.6 %** |

With the seated rider the same springs sag less at the fork and more at the shock, because
40.4 % rather than 46.7 % of the weight is on the front axle (table above).

The first row is what this document originally stated; the second is measured from
`solve_static_equilibrium` and is the state every run starts in. They differ because the
analytic figure omits two real effects. The **unsprung mass** — 3.90 kg front (fork lowers
and wheel), 5.10 kg rear (chainstay, seatstay and wheel) — rests on the contact patch
*below* the spring and is never carried by it. And the bike **pitches about 1.1° nose-down
at sag**, redistributing the axle loads. Feeding the solved contact loads minus the
unsprung weight back into `ForkAirSpring` and the leverage-ratio solver reproduces the
solved sag to under 1 mm at both ends, so the two representations agree; it is the
first-order shortcut that is off.

`physics/tuning.py` makes the same first-order simplification in its spring calibration
(`f_shock_sag = f_rear_vert · lr_at_sag` uses the full axle load) while correctly
subtracting unsprung mass for damping, so the tuning chain is consistent with itself and
both of its spring figures are first-order. It is deliberately left as is: recording the
discrepancy is this document's job, re-deriving the shipped rates is not.

The direction of the disagreement is unchanged: the fork is under-sprung and the shock
over-sprung for the model's own load. **This is pre-existing and already documented** as
Known Limitation #1 in the README, where it is correctly classified as a ride-feel
decision rather than a bug. Ride mode does not introduce it — but ride mode is the first
mode in which sag has dynamic consequences.

### The `--sag` option

`physics/tuning.py::compute_suspension_tuning_for_sag` already performs exactly the
re-derivation the README calls for, and defaults `front_weight_fraction` to the model's own
CoG when it is not overridden. `--sag` therefore fits **both ends together** against the
model's own centre of mass and prints the result:

| Target 30 / 30 against the model's CoG | `lumped` | `seated` (default) | Shipped default |
|---|---|---|---|
| Fork pressure | 118.5 psi | 100.5 psi | 85.2 psi |
| Shock rate | 93 984 N/m = 537 lb/in | 105 148 N/m = 600 lb/in | 114 600 N/m = 654 lb/in |

The fit uses the chosen rider's own centre of mass — `RiderSpecs.compute_rider_centers_of_mass`
reads the solved seated pose, so `--sag` and the compiled model agree on where the rider is.
Both fitted lumped values are physically ordinary: 118.5 psi is mid-range for a
38 mm-stanchion enduro fork, and 537 lb/in is a stock catalogue coil size. Bottom-out
reserve is unchanged at 6.11 g front and 4.64 g rear.

Because the fit is first-order (see above), the **solved** equilibrium at `--sag 30` is
not exactly 30/30: measured 49.6 mm = 27.6 % front and 49.6 mm = 27.5 % rear. The fit
removes the 18-point front/rear imbalance of the shipped tune; the remaining 2.5-point
offset is the unsprung-mass and pitch effect the first-order calculation does not see.

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
| `timestep`, `pneumatic` / `detailed` | 0.00025, set after compilation | bristle transport and the reference role of the tier (§3.1) |
| `integrator` | `implicitfast` | matches the other modes |
| Joint limits | softened `solreflimit` | final barrier, not the bottom-out mechanism |

The suspension forces are integrated **explicitly** regardless of the integrator: MuJoCo
can treat implicitly only the terms it knows about, and the air spring, the coil and the
dampers are external forces it does not. The only defence against their oscillation is a
shorter step, hence 0.0005 rather than the 0.001 used by the stand modes.

Cost is not a concern. Measured with a wheel-sized sphere rolling on a heightfield of this
grid size at `timestep=0.0005`: **22.5× real time**. The interactive viewer runs at real
time; the headless reference runs at 0.0005 regardless.

That figure is for a bare sphere. The full ride loop — six Python writers and a contact
query per step, around `mj_step` — is what the `pneumatic` tyre adds to, and it is the
budget `fast` must fit (§3.1). Measured with `tools/bench_ride.py` on the development
machine (Apple M4 Max, MuJoCo 3.12.0, Python 3.12.13, seated rider, every preset ridden
end to end):

| | Range over the seven presets |
|---|---|
| Real-time factor | **4.48–5.61×** (`flat` slowest, `road_broken` fastest) |
| Cost per step | 89–112 µs |
| `mj_step` | 67–88 µs |
| Contact query | 9–10 µs |
| All six writers together | ≈ 11 µs |

So `mj_step` is three quarters of the step and the Python around it is cheap. The viewer
spends part of each real second on `viewer.sync` at 120 Hz, the HUD and a 1 ms frame
sleep. Reserving 30 % of each timestep for that leaves **238 µs per step** for the tyre,
both wheels together, set by the slowest track. `pneumatic` also removes the wheels'
collision rows from `mj_step`, which the budget does not count on.

Task 10 measured all three configurations on all seven presets with
`uv run python -m tools.bench_ride`:

| Tyre config | Real-time factor | Total step cost | Tyre writer per step | Result |
|---|---:|---:|---:|---|
| `sphere` | 4.48–5.61× | 89–112 µs | — | baseline |
| `pneumatic/fast` | 1.97–2.21× | 227–254 µs | 181.5–210.5 µs | within the 238 µs budget; 28 µs headroom at the worst track |
| `pneumatic/detailed` | 0.82–0.93× | 269–304 µs | 222.9–260.0 µs | 1.1–1.2× slower than real time, inside the 2–3× allowance |

The slowest `fast` tyre writer is `road_smooth` at 210.5 µs; its full ride step is 253.6 µs.
`detailed` is slowest on `road_smooth` at 0.82× real time. No ray-count reduction was needed.

The tyre force, like the suspension forces, is external to MuJoCo and therefore integrated
explicitly. Its stability bound is part of §3.1.

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
| Rolling resistance coefficient (`sphere`) | 0.015 |
| Tyre vertical stiffness / damping (`sphere`) | 130 N/mm, 800 N·s/m |
| Sliding friction (`sphere`) | 1.2 |
| Virtual rider moment ceiling | ±80 N·m |
| Coil preload and bumper characteristic | to be fixed with the implementation |
| Crash thresholds | 60° pitch, handlebar contact |
| Aerodynamic drag | omitted |
| Default cruise speed | 25 km/h |
| Seated rider: leg and arm path resonances | 5 Hz, 4 Hz (§7.2; Wang & Hull 1997 Table 2 would replace them) |
| Seated rider: hip above saddle, ankle above pedal, elbow flexion | 60 mm, 115 mm, 15° (§7.3) |
| Seated rider: inseam / stature | 0.47 (ANSUR II, rounded; `--rider-inseam` overrides) |
| Seated rider: helmet | 0.4 kg inside the rider mass |
| Cranks | 165 mm, horizontal, rigid; spindle 0.10 + arms 2 × 0.20 + pedals 2 × 0.175 kg |

The seated rider's *literature* values — de Leva's segment fractions, Kumar & Saran's saddle
contact, the 4–6 Hz / ~1.5 × apparent-mass peak, the 55 / 33 / 12 % split extrapolated from
Carahalios — are listed with their sources in §7.2 and in `physics/rider.py`.

### 11.1 The `sphere` tyre against the literature

The reason §3.1 exists:

| Quantity | `sphere` | Measured, 29×2.3–2.4 MTB tyres |
|---|---|---|
| Vertical stiffness | 130 N/mm, linear, pressure-independent | ≈ 62 N/mm static at 1.72 bar / 418 N; 74–86 N/mm dynamic [T1][T2] |
| Vertical damping | 800 N·s/m, viscous | ≈ 100–190 N·s/m, ζ ≈ 2–5.5 % [T2] |
| Static deflection at this bike's loads | 3–5 mm | ≈ 7–10 mm [derived from T1] |
| Rolling resistance | Crr 0.015 | 0.0103 (Hans Dampf, Addix Soft) / ≈ 0.011 (Magic Mary, Soft) at 1.5 bar [T4] |
| Friction | μ 1.2 on every surface | asphalt peak ≈ 1.0–1.1, hardpack ≈ 0.7–0.85 [est, T5][T6] |

### 11.2 `pneumatic` parameters

**Literature-derived.** These are targets the model is fitted to or tested against, with
their stated tolerances:

| Parameter | Value | Source |
|---|---|---|
| Static stiffness law | `k(p) ≈ 22 + 24·p[bar]` N/mm, ±15 % | derived from [T1] (29×2.3 on a 25 mm rim) |
| Pressure term's share of load | 70–100 % | [T1]: p·A of the bald tyre ≈ 107 % of the load |
| Contact length at 418 N | 133 mm at 1.38 bar, 122 mm at 1.72 bar, ±15 % | [T1] Fig. 16 |
| Dynamic / static stiffness | 1.16–1.35 | [T2] |
| Vertical damping ratio | 2–5.5 % (loss factor 0.05–0.09) | [T2] |
| Rim-strike deflection | 0.80–0.85 × section height | [T1]: 48.7 mm at 1.03 bar with no rim strike |
| Section height, Hans Dampf 29×2.35 | 55 mm | [T3] |
| Crr, Hans Dampf SG Addix Soft | 0.0103 at 1.5 bar, 490.5 N, 20 km/h | [T4] |
| Crr vs pressure | ∝ p^−0.3 | derived from [T3] |
| Casing damage energy, reference only | Super Trail 61 J, Super Gravity 95 J | [T4b] |

**Authored.** These are estimates or choices, to be replaced by measurement where the plan
says so:

| Parameter | Value | Basis |
|---|---|---|
| Crr, Magic Mary Addix Soft | ≈ 0.011 | Ultra Soft measured at 0.0176; Soft ≈ Ultra Soft / 1.6 [T4] |
| Casing width `W_c` | 60 mm | tyre width class |
| Compressed casing and tread `t_c` | 6 mm | casing 1.9 mm + centre knobs 3.8–4.0 mm [T3] |
| Hysteresis regularisation `δ̇_ε` | 0.01 m/s | numerical |
| Rim stiffness `k_rim` | 2.8×10⁷ N/m² | stability cap with the compiled 2.40 kg front wheel (§3.1) |
| Tread-loss Crr correction at 1.5 bar | 0.00586 front / 0.00481 rear | residual after carcass hysteresis, fit to T4 drum targets (§3.1) |
| Relaxation length σ | 60 mm (60–120) | lower end of the estimate; lateral 160 mm derived from [T1], road tyres 79–141 mm |
| Detailed bristle dashpot τ_b | 0.5 ms | numerical regularizer; suppresses the measured 33 Hz wheel-slip mode (§4.1, Task 10) |
| Normalised slip stiffness C_κ/F_z | 15 / 12 / 7 / 10 by surface | MTB cornering stiffness as proxy [T1]; car and trekking data [T5] |
| Surface μ peak / sliding | §4.1 table | car Burckhardt curves scaled to MTB data [T6] |
| Stribeck speed | 4.5 m/s | numerical; preserves the 15–45 km/h peak-μ target on all surfaces |
| Ray coverage and counts | ±75°; 64 / 256 | §3.1 coverage argument; tuned in plan task 10 |
| Default pressures | 1.5 bar front / 1.7 bar rear | plan D5 |

The user's own measurements (plan Appendix A: loaded deflection at two pressures per wheel)
will replace the stiffness fit's literature anchor with this bike's tyres. When that
happens, the rows affected move from "literature-derived" to "measured".

Sources:

- **[T1]** Dressel & Sadauckas, *Applied Sciences* 10(9):3156, 2020.
- **[T2]** Sadauckas et al., *Vehicle System Dynamics*, 2024.
- **[T3]** bicyclerollingresistance.com: Hans Dampf TrailStar 2017 review and test protocol.
- **[T4]** ENDURO Mountainbike Magazine: Schwalbe tyre lab test, 2025.
- **[T4b]** ENDURO Mountainbike Magazine: tyre insert test.
- **[T5]** O. Maier, dissertation, KIT.
- **[T6]** Burckhardt tyre–road friction parameters.

Full URLs are in the plan's Appendix B.

---

## 12. Known non-physicalities

**1 — The virtual rider moment injects angular momentum.** A pure moment on `root_pitch`
has no reaction body. A real rider pitches the bike by counter-rotating their own body: an
internal torque with a genuine reaction, and total angular momentum in flight is conserved.
Ours is not. This is why the ±80 N·m cap is a damage limit rather than a realism parameter,
and why §8 logs the moment's impulse and work separately. The seated rider (§7.2) did not
close this: its bodies move only along the frame's vertical, so they can carry no pitch
torque, and a seated rider is not the rider who flies a 600 mm drop anyway — `bike-ride`
warns when the two are combined. The physical alternative remains a standing rider whose
mass can shift fore-aft on a driven joint; it is deferred, with the flight share of the
energy audit as the trigger, as before.

**1b — The seated rider is vertical-only.** Every seated rider mass slides along the frame's
Z. Under braking and pitch the rider does not shift fore-aft relative to the bike, and when
the bike pitches, "vertical" tilts with it. The whole-body-vibration literature this model
follows is vertical too, and the fore-aft parameters it would need are scarcely published;
the restriction is documented rather than filled with invented numbers. Two sided-ness is
also a simplification: hands are taken as gripping (two-sided), feet as resting on flat
pedals (one-sided).

**2 — The spring defaults are calibrated for a weight split the model does not produce**
(§9). At the shipped settings the fork sags 40.6 % and the shock 22.7 % (solved; 42.0 /
25.3 % to first order), in opposite directions, from a single root cause: the springs were derived at 35/65 and the model's own
CoG loads them at 46.7/53.3. Both ends return to exactly 30 % at the documented split, so
the tuning chain is internally consistent — it is the convention that disagrees with the
centre of mass, exactly as README Known Limitation #1 states. Ride mode does not change the
defaults; it fits both ends behind `--sag` and reports what it used.

**3 — Two independent representations of one linkage** (§5), agreeing to 0.039 mm
quasi-statically, unverified under impact until the invariant test measures it.

**4 — No lateral dynamics** (§0), by construction. That includes the seated rider's
lateral weight shift.

**5 — No tyre slip model** (`sphere` only, §4.0): regularized Coulomb friction only.
`pneumatic` has one (§4.1).

**6 — No drivetrain** (§6): torque is applied at the wheel.

**7 — No aerodynamic drag** (§4): wheel power reads ~90 W low at 25 km/h.

**8 — Contact gates are debounced** (`sphere` only; §6, §7). MuJoCo's sphere–heightfield collision drops
the contact row of a loaded wheel on a speed-dependent share of steps — 2.0 / 14.7 / 27.2 /
43.2 % at 15 / 25 / 35 / 45 km/h on flat ground — and `sim/ride/contacts.py` holds the
last measured load for up to 10 steps (5 ms) so a false airborne neither gates the drive
torque off nor fires the virtual rider. Two facts make the fixed window sound: the
artefact's *duration* is speed-invariant (worst run 1 both-wheel step, 3 single-wheel
steps at 45 km/h) even though its rate is not; and a genuine both-wheels flight ending
inside 5 ms would need a take-off vertical velocity of 0.025 m/s, below anything the
track produces. Rolling resistance deliberately consumes the **raw** load rather than the
debounced one, because there the artefact costs nothing (an unloaded step carries no
`Crr·N·r`) and the hold would bias a §8 sink. The telemetry's `*_contact` flags are the
debounced signal; `*_load_n` are the bridged magnitudes.

Items 9–14 are what the `pneumatic` tyre (§3.1, §4.1) still does not do.

**9 — No lateral tyre forces.** No cornering or camber force and no turn slip, which is
implied by §0. Only the longitudinal half of the brush exists.

**10 — Rigid road.** The surface presets change the friction curve and nothing else. `loose`
does not sink, rut or shed material, and no surface deforms under the tyre.

**11 — Independent radial elements.** The carcass is a set of radial springs with no shear
between neighbouring elements and no belt inertia. Standing waves and carcass modes above
the wheel-hop band are absent. The tyre's mass rides on the wheel body (§3.1).

**12 — No tread-block, temperature, wear or sealant effects.** Rubber compound enters only
through the rolling-resistance calibration and the surface μ.

**13 — Finite coverage.** The rays see ±75° about the vertical. Road beyond that, an edge
higher than about 0.74 R above the wheel's lowest point, is invisible to the tyre and raises
a coverage event.

**14 — Rim strikes do no damage.** A strike is a stiff element and a logged event. The tyre
stays inflated and the rim stays true (plan D11).

---

# Usage

## Running a ride

Everything runs under `uv`. The console script is `bike-ride`
(`src/bike_sim/cli/ride.py`); the default track is the generated `road_worn`.

```bash
uv run bike-ride                                  # interactive viewer, road_worn, 25 km/h
uv run bike-ride --track enduro_aggressive        # the trail preset in the viewer
uv run bike-ride --headless                       # telemetry + summary + plots, no window
uv run bike-ride --headless --track road_broken --speed 30 --seed 3
uv run bike-ride --headless --track my_road.toml --sag 30
uv run bike-ride --headless --rider lumped        # the original rigid standing rider
uv run bike-ride --headless --rider-mass 92 --rider-height 1.88
uv run bike-ride --headless --tyre-model pneumatic --tyre-tier fast --tyre-pressure 1.5/1.7
uv run bike-ride --track my_road.toml --preview   # draw the road, do not simulate
uv run bike-ride --dump-track road_worn > my_road.toml
uv run bike-ride --list-tracks
```

| Flag | Meaning |
|---|---|
| `--track NAME\|PATH` | Preset name or path to a `.toml` track file. Default `road_worn`. |
| `--seed N` | Generator seed override; applies to `road_*` presets and files with a `[generator]` block. |
| `--length M` | Track length override, same applicability. |
| `--speed KMH` | Cruise target, 15–45 km/h. Default 25. |
| `--headless` | Run without a viewer; write artifacts (below). |
| `--sag PCT` | Fit fork pressure and coil rate to this static sag at both ends (§9) instead of the shipped tune; headless only. |
| `--out DIR` | Artifact root. Default `output/ride`. |
| `--decimate N` | Keep every N-th step in `telemetry.csv`. Default 1. |
| `--rider {none,lumped,seated}` | Rider model (§7). Default `seated`. |
| `--rider-mass KG` | Rider mass incl. helmet and kit. Default 80. |
| `--rider-height M` | Stature; scales segments and, seated, sets the saddle via the inseam. Default 1.80. |
| `--rider-inseam M` | Inseam override. Default 0.47 × height. |
| `--no-rider` | Alias for `--rider none`. |
| `--tyre-model {sphere,pneumatic}` | Wheel contact model. Default `sphere` (§3). |
| `--tyre-tier {fast,detailed}` | Pneumatic brush resolution. Default `fast` (§3.1). |
| `--tyre-pressure FRONT/REAR` | Front/rear pressures in bar, each 0.8–3.0. Default `1.5/1.7`. |
| `--surface NAME` | Surface preset (`asphalt`, `hardpack`, `loose`, `wet`); default from the track (§4.1). |
| `--no-plots` | Headless: skip the PNG figures. |
| `--preview` | Render the road profile with effective pothole drops to `<out>/preview_<track>_s<seed>.png` and exit. |
| `--dump-track NAME` | Print a preset as a track file and exit. |
| `--list-tracks` | List presets and exit. |

To compare the sphere and pneumatic models on every preset, run
`uv run python -m tools.compare_tyre_models`. Add `--detailed` to include the detailed brush
tier. The script writes `output/tyre_compare/summary.json` and `table.md`.

A headless sphere run writes to `output/ride/<track>_<speed>_s<seed>/` (the `_s<seed>` suffix
only when a generator seed applies). Pneumatic runs append the model, tier, both pressures and
surface to the directory name, so a pressure sweep does not overwrite the sphere run:
`<track>_<speed>_s<seed>_pneumatic-fast_1.50-1.70bar_asphalt/`.

| File | Content |
|---|---|
| `telemetry.csv` | One row per recorded step, [channels below](#telemetry-channels). |
| `summary.json` | The [summary metrics](#summary-metrics); also printed as a table. |
| `travel.png` | Fork travel and shock stroke / rear wheel travel against X with obstacle markers. |
| `shaft_velocity.png` | Shaft-velocity histogram per end. |
| `acceleration.png` | Low-passed bar and saddle vertical acceleration, raw peak annotated; the seated rider's torso overlaid on the saddle panel. |
| `tyres.png` | Pneumatic only: per-wheel load, deflection/rim strikes, slip and patch length. |
| `profile.png` | The road profile and effective pothole drops. |

The run starts from the solved static equilibrium at *x* = 2 m and ends at the track's
end, at a crash (pitch > 60° or handlebar contact), at the step cap or at the wall-clock
cap (`sim/ride/termination.py`). The outcome is printed and stored in the summary.

## Tracks: presets and track files

Built-in presets (`terrain/presets.py`): `enduro_aggressive` (115 m, the §2 trail),
`flat`, `single_edge`, `washboard_only`, and the three generated roads `road_smooth`,
`road_worn`, `road_broken` (100 m each, seed 0). `DEFAULT_PRESET` remains
`enduro_aggressive` for the library; the CLI's default is `road_worn`.

A **track file** is TOML (`terrain/trackfile.py`). Heights, depths and amplitudes take
`*_mm` keys; positions and lengths are in metres. Any class's own field names (metres)
are also accepted, which is what a dump uses so it round-trips exactly.

```toml
name = "my_road"
length_m = 150
description = "optional"

[[obstacles]]             # hand-placed; obstacles may not overlap
type = "pothole"          # edge = sharp | sloped | bowl  (sloped takes edge_m, default 0.10)
start_m = 20.0
depth_mm = 80
length_m = 0.5
edge = "sharp"

[[obstacles]]
type = "bump"             # shape = cosine | trapezoid  (trapezoid takes ramp_m, plateau_m)
start_m = 35.0
height_mm = 50
length_m = 0.4
shape = "cosine"

[[obstacles]]             # the trail catalogue is available too, with its own field names
type = "square_edge"
start_m = 60.0
height_mm = 90
length_m = 0.25

[generator]               # optional: procedural fill around the hand-placed obstacles
seed = 0
runup_m = 10.0            # reserved flat road at the start (the bike needs ~8 m to reach speed)
runout_m = 5.0
min_gap_m = 1.0
potholes_per_100m = 4
pothole_depth_mm = [40, 120]          # [low, high] = uniform per obstacle; a scalar = fixed
pothole_length_m = [0.3, 0.8]
pothole_edge = { sharp = 0.6, sloped = 0.3, bowl = 0.1 }   # weights; or a single name
pothole_edge_m = 0.10
bumps_per_100m = 6
bump_height_mm = [30, 80]
bump_length_m = [0.3, 0.6]
bump_shape = { cosine = 0.7, trapezoid = 0.3 }
bump_ramp_fraction = 0.3
roughness_mm = 3                      # 0 disables the background texture
roughness_correlation_m = 0.2
```

Type names: `pothole`, `bump`, `square_edge`, `washboard`, `g_out`, `drop`, `kicker`,
`roots`, `rock_garden`, `roughness`. `length_m` maps to each type's own length field
(`hole_length_m`, `bump_length_m`, `ledge_length_m`, `dip_length_m`,
`section_length_m`); types with several lengths (`kicker`, `trapezoid`, `drop`) take
their own fields. Errors name the offending entry and the allowed keys.

`--dump-track` writes the **materialised** layout — every generated obstacle listed,
no `[generator]` block — so a dumped file pins exactly what was simulated and can be
edited by hand. Roughness segments in a file **yield** to anything placed on top of
them: add a pothole inside one and the loader splits the segment around it (the right
remainder gets a derived seed; remainders under 0.5 m are dropped). Two real obstacles
that overlap are still an error.

## Rough-road generator

`terrain/road.py::generate_road` turns a `RoadGeneratorSpec` into obstacles. Counts are
deterministic — rate × usable length, rounded — so two seeds of one level differ in
layout and individual sizes, not in severity. Sizes and shapes are drawn first in a fixed
order, then placed largest-first into the free stretches between `runup_m` and
`length − runout_m`, keeping `min_gap_m` clear of each other and of hand-placed
obstacles. Sizes are rounded to the millimetre and positions to the centimetre so a
dump reads like a file a person wrote. When `roughness_mm > 0`, every free stretch at
least four correlation lengths long gets a `RoadRoughness` segment with its own seed.
A density that does not fit raises a clear error rather than silently dropping defects.

The three levels:

| Level | Potholes /100 m | Depth mm | Length m | Bumps /100 m | Height mm | Length m | Shapes | Texture |
|---|---|---|---|---|---|---|---|---|
| `road_smooth` | 1 | 30–60 | 0.3–0.5 | 2 | 20–40 | 0.3–0.5 | sharp / cosine | 2 mm |
| `road_worn` | 4 | 40–120 | 0.3–0.8 | 6 | 30–80 | 0.3–0.6 | sharp / cosine | 3 mm |
| `road_broken` | 8 | 60–150 | 0.4–1.0 | 10 | 50–100 | 0.3–0.7 | mixed | 5 mm, 0.15 m |

## Heightfield sizing

`HeightFieldSpec.for_track(track)` picks the field. A track of up to 115 m uses the
shipped 120 m field unchanged, so `baseline_bike_ride.xml` still pins it. A longer track
gets the same 5 mm grid and vertical envelope stretched to `length + 5 m`, rounded up to
10 m; memory is trivial (two rows) and sphere–heightfield collision is local, so headless
cost grows linearly with length only. The 5 m margin matters: the front wheel runs
ahead of the chassis root, and beyond the field's far edge there is only the catch
plane. Tracks over 500 m trigger a note about viewer triangle count.

## Viewer key map

Interactive mode (`bike-ride` without `--headless`) prints this on start and on `?`:

| Keys | Action |
|---|---|
| `W` / `S` | Cruise target ± 1 km/h (15–45) |
| `Space` | Brake toggle; `,` / `.` brake strength ∓ 10 % |
| `R` | Restart from the solved equilibrium |
| `[` / `]` | Fork air tokens −/+; `-` / `=` fork pressure ∓ 2 psi |
| `N` / `M` | Pneumatic front tyre pressure −/+ 0.05 bar |
| `;` / `'` | Pneumatic rear tyre pressure −/+ 0.05 bar |
| `H` `J` / `K` `L` / `Y` `U` | Fork HSC / LSC / rebound clicks |
| `7` `8` / `9` `0` / `5` `6` / `3` `4` | Shock HSC / LSC / rebound / HBO clicks |
| `X` | Shock lockout toggle; `P` cycle factory damper presets |
| `C` / `1` / `2` | Camera cycle / 2D side / 3D isometric |
| `G` | Pivot markers; `T` telemetry line |
| `Esc` | Quit |

These pressure keys are clamped to 0.8–3.0 bar and take effect on the next step, without a
restart, because the tyre reads pressure every step (§3.1). With `sphere`, they print a
one-line notice.

The track and the rider are chosen on the command line and cannot be changed from the
viewer: a change of track means new `hfield_data` and a new equilibrium, and a change of
rider means a different set of bodies and coordinates (§1) — both are a restart of the
command. The test stand's `B` rider toggle is therefore deliberately unbound here; compare
riders with two headless runs on one seed. `--sag` is not applied in the viewer (use `-`/`=`
and `P` there).

## Telemetry channels

`telemetry.csv` (`sim/ride/recorder.py`, `%.10g` floats) has one header row and one row
per recorded step, the first row being the start state:

| Channel | Unit | Meaning |
|---|---|---|
| `time_s` | s | Simulation time. |
| `x_m` | m | Chassis root position along the track. |
| `speed_mps` | m/s | Chassis longitudinal velocity (what the cruise regulates). |
| `fork_travel_mm`, `fork_shaft_mps` | mm, m/s | Fork compression and shaft velocity (+ compressing). |
| `shock_stroke_mm`, `shock_shaft_mps` | mm, m/s | Shock stroke and shaft velocity. |
| `rear_wheel_mm` | mm | Rear wheel travel for the stroke, from the tabulated leverage curve. |
| `fork_spring_n`, `fork_damper_n` | N | Fork air-spring and damper force. |
| `shock_spring_n`, `shock_bumper_n`, `shock_damper_n` | N | Coil, bottom-out bumper and damper force. |
| `pitch_rad` | rad | Chassis pitch, positive nose-down. |
| `front_contact`, `rear_contact` | 0/1 | Debounced contact flags (§12 item 8). |
| `front_load_n`, `rear_load_n` | N | Bridged normal-load magnitudes. |
| `drive_torque_nm`, `wheel_power_w` | N·m, W | Cruise torque and torque × rear wheel ω. |
| `bar_acc_vert_mps2`, `bar_acc_long_mps2` | m/s² | Handlebar **proper** acceleration in world Z / X: +9.81 at rest, 0 in free fall. Raw. |
| `saddle_acc_vert_mps2`, `saddle_acc_long_mps2` | m/s² | Same at the seatpost top. |
| `rider_moment_nm`, `rider_impulse_nms`, `rider_work_j` | N·m, N·m·s, J | Virtual rider moment, accumulated angular impulse and work (§8). |
| `saddle_load_n` | N | Force the saddle exerts on the seated rider's pelvis; 0 when they have left it. |
| `saddle_gap_m` | m | Pelvis-to-saddle separation; 0 while seated. |
| `bar_hand_load_n` | N | Force the bar exerts on the arms; negative when the rider pulls up. |
| `pedal_load_front_n`, `pedal_load_rear_n` | N | Force each pedal exerts on its leg; 0 when the foot unweights. |
| `rider_torso_acc_vert_mps2`, `rider_torso_acc_long_mps2` | m/s² | Seated rider's torso proper acceleration, world Z / X. |
| `rider_pelvis_acc_vert_mps2`, `rider_pelvis_acc_long_mps2` | m/s² | Same for the pelvis. |

The seated rider's nine channels (§7.2) read zero for `none` and `lumped`. The pneumatic
channels also remain present under `sphere` and read zero, so every CSV has 54 columns.

| Channel | Unit | Meaning |
|---|---|---|
| `{front,rear}_tyre_fz_n` | N | World-vertical tyre support force. |
| `{front,rear}_tyre_fx_n` | N | Resultant longitudinal force, including radial hysteresis and brush force. |
| `{front,rear}_tyre_deflection_mm` | mm | Load-weighted mean radial-element deflection. |
| `{front,rear}_patch_length_mm` | mm | Sum of the scaled contact lengths of the wheel's patches. |
| `{front,rear}_slip_ratio` | — | Longitudinal slip κ; positive is drive, −1 is a locked wheel. |
| `{front,rear}_tyre_full_sliding` | 0/1 | At least one patch is fully sliding. |
| `{front,rear}_rim_strike` | 0/1 | A rim-strike event is active or closes on this step. |
| `{front,rear}_tyre_pressure_bar` | bar | Live pressure read by the tyre model. |
| `{front,rear}_tyre_loss_w` | W | Carcass hysteresis and sliding power loss. |

## Summary metrics

`summary.json` (`sim/ride/metrics.py`) is computed over the **window** `x ≥ start + 8 m`,
excluding the acceleration ramp. Per end (`fork`, `shock`): max travel in mm and %, 95th
percentile, mean, bottom-out count (fork: within 5 mm of full travel; shock: bumper
engaged, > 55 mm), top-out count (within 1 mm of zero), max compression and rebound shaft
velocity. Counts are events — contiguous runs — not samples. Per contact point (`bar`,
`saddle`): RMS and peak of vertical acceleration with gravity removed after a 4th-order
zero-phase Butterworth low-pass at 100 Hz, **and** the raw peak beside it, labelled.
Also: outcome and crash reason, steps, sim and wall time, mean speed, airborne events
and time, a per-pothole table of declared depth versus effective front and rear wheel
drop, `rider_variant`, and `extras` (seed, rider mass, sag fit, start equilibrium).

With the seated rider a `rider` block is added: the same acceleration statistics for the
rider's `torso` and `pelvis`; the mean saddle / pedal / bar load shares over the window,
against the 55 / 33 / 12 % static split; the saddle load range; and saddle lift-offs as
events, total time and maximum gap. On `road_worn` at 25 km/h the default rider's torso
sees an RMS of 3.0 m/s² against 4.6 at the saddle beneath it — the body's own compliance is
a third of the isolation — and leaves the saddle once, for 27 ms and 0.6 mm, at the 105 mm
pothole.

With `pneumatic`, the summary adds a `tyres` block per wheel: peak normal load, maximum
deflection, rim-strike count and event details (x, speed, peak load/force, stored energy),
wheelspin and lock time, mean loss power and its Crr equivalent. Wheelspin means a fully
sliding patch with positive κ; lock means a fully sliding patch with negative κ. `extras`
carries tyre model, tier, front/rear pressures and surface.

Why filter, and why keep the raw peak: the rigid contact sphere meeting a sharp
heightfield edge produces solver transients of 12–16 system weights lasting 2–4 steps
(1–2 ms) — a real tyre spreads that over 10–20 ms. Unfiltered, "peak acceleration" is a
property of the solver, not of the suspension. The 100 Hz low-pass keeps every band the
suspension works in (fork 2–4 Hz, wheel hop 10–15 Hz) and spreads the transient's impulse
over ~5 ms, which is the honest, tyre-like number; the raw peak is reported next to it so
nothing is hidden. Compare *filtered* values between runs.

The gap between raw and filtered peaks above is expected to shrink under `pneumatic`. A
tyre that spreads an edge hit over its footprint is the physical version of what the 100 Hz
filter approximates; plan task 11 reports the gap for both models.

## Reading the plots

**`travel.png`** — the limit lines are full travel; a trace touching them is a bottom-out
event in the summary. Markers name the obstacle that caused each excursion.

**`shaft_velocity.png`** — the damper is a force-versus-velocity device, so this is the
view to tune clicks against. Compression is to the right, rebound to the left, density on a
log scale. The bulk near zero is low-speed circuit territory (LSC / rebound clicks); the
tails past ~0.5 m/s are the high-speed shim stack (HSC). A rebound tail much shorter than
the compression tail means the rebound circuit is doing its job; a compression tail that
reaches 4–5 m/s on a road preset is the square-edged potholes.

**`acceleration.png`** — filtered traces; the red annotation is the raw peak and where it
happened. **`profile.png`** — the road, and for each pothole three bars: declared depth,
front-wheel drop, rear-wheel drop. When the coloured bars are shorter than the grey one,
the hole is shorter than the wheel can fall into and its declared depth is moot.
