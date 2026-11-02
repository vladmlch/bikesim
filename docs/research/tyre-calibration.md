# Pneumatic tyre — model contract, calibration and sources

Restored 2026-10-14 from the last `docs/RIDE.md` revision that carried the Physics
Contract (commit `2e93585`, 2026-10-01: §3.1, §4.1, §11.1–11.2, §12 items 9–14 and the
pneumatic plan's Appendix B). The code still cites those section numbers
(`physics/tyre.py`, `sim/ride/tyre/*.py`, `terrain/surface.py`, `physical_config.py`,
`native/src/pneumatic/carcass.hpp`, `examples/research/viewer_physics_pneumatic.toml`);
**the anchors below are those sections**, so `RIDE.md §3.1` reads as
[§3.1](#31-pneumatic-wheelground-contact) here, `§4.1` as
[§4.1](#41-pneumatic-brush-slip), `§11.2` as [§11.2](#112-pneumatic-parameters) and
`§12 item 10` as [§12](#12-known-non-physicalities-of-the-tyre).

Status of the text: the physics and the numbers were re-checked against the tree on
2026-10-14 (`physics/tyre.py`, `sim/ride/tyre/{geometry,carcass,brush,model}.py`,
`terrain/surface.py`). Where the current code differs from the 2026-10-01 text, the
difference is stated in [§0](#0-what-changed-since-the-text-was-written), and the
sections below are left as the original authors wrote them except for those corrections.
Audit that motivated the restoration:
`docs/superpowers/audits/2026-10-14-tire-research-vs-implementation.md`.

---

## 0. What changed since the text was written

| Then (2026-10-01) | Now | Where |
|---|---|---|
| The applier **assigned `xfrc_applied`** per wheel | The physical runtime's `PneumaticTireForceApplier` writes **`qfrc_applied`** through `mj_jac` evaluated at each patch centroid; the `xfrc` applier survives only for the legacy stack | `sim/ride/pneumatic_tire_forces.py` (module docstring), `sim/ride/tyre/applier.py` |
| `detailed` set `model.opt.timestep` after compilation | The physical runtime owns the integration step and passes `dt` in; the applier never mutates `opt.timestep` | `pneumatic_tire_forces.py` docstring |
| One `SurfaceSpec` per run | `SurfaceMap`: a base surface plus validated half-open material zones along `x` (plan D9 delivered); `tires.pneumatic.surface` still overrides | `terrain/surface.py`, `physical_config.py:41-66` |
| Four surfaces | Five: `scree` added by the alpine track work (`mu_peak 0.45, mu_slide 0.38, C_κ/F_z 4.5`, "loose rock debris over bedrock") — still a **rigid** road, see §12 item 10 | `terrain/surface.py:110-112`, `terrain/alpine.py` |
| Python only | Bitwise C++ port (`native/src/pneumatic/*`, `native/src/writers/pneumatic_tire.cpp`); the Python module is the oracle, `tests/reference/test_native_pneumatic_tire.py` the arbiter | `native/src/writers/pneumatic_tire.hpp` header comment |
| Final ray counts "pending (plan task 10)" | Settled: `fast` 64 rays / 0.5 ms, `detailed` 256 rays / 0.25 ms | `physics/tyre.py:183-187` |
| Configuration via `TyreConfig(model="pneumatic")` | Physical mode uses `tires.backend = "pneumatic"` + `PneumaticTireConfig(tier, front, rear, surface)`; `TyreConfig` remains for the legacy stack | `physical_config.py:41`, `physics/tyre.py:192` |

Nothing in the force laws, the fitted constants or the literature targets changed.

---

## 3.1 `pneumatic` wheel–ground contact

**Why not MuJoCo contact.** A pressure-dependent, non-linear load–deflection law, a rim
behind the carcass and a slip-dependent tangential force are not expressible in `solref`,
`solimp` and a Coulomb cone. The road, on the other hand, is known in closed form: the
tyre does not need collision detection to find it. So in `pneumatic` the wheels do not
collide at all, and a force model evaluates each wheel against `h(x)` once per step. The
sphere–heightfield dropouts of the legacy `sphere` tyre do not exist in this model.

**What changes in the compiled model — only this.** The two contact spheres get
`contype="0" conaffinity="0"`. They stay in the model, because `resolve_wheel_spin` reads
the wheel radius from them. The tyre cylinders already do not collide. The heightfield and
the catch plane are unchanged: they render the road and carry the frame and handlebar
collisions.

**Road seen by the tyre.** The same profile the heightfield is rasterised from —
`build_profile(track, field.track_x())`, metres, 5 mm — linearly interpolated. Physics and
picture therefore cannot disagree. The road is rigid; its surface properties come from
§4.1.

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
(§11.2). Tubeless, no insert (plan D5).

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

- `p` is gauge pressure. It is read every step, so it can change mid-run (viewer keys
  `N`/`M`).
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

Fitted values (1024-ray flat-road fit; the 256-ray `detailed` tier also passes the targets;
`physics/tyre.py:133-160`):

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

Sag therefore still follows the static law, and the equilibrium solve converges onto it.
Added damping at 6.5 Hz is about 1 % of critical, and it counts towards the damping band
below.

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
   that reaches the rider, so it has priority.
2. **Rolling resistance** on flat asphalt at 20 km/h, 490.5 N and 1.5 bar must match the drum
   targets within ±15 %: Crr 0.0103 rear, ≈ 0.011 front.

If an `η` inside the damping band leaves Crr more than 15 % short, the shortfall is closed
by a **tread-loss term** on each patch, `F_roll = (Crr_target − Crr_η) · N_p`, opposing
rolling and tapered through zero wheel speed. This is the model's one phenomenological
term. On the 20 km/h, 490.5 N reference roll, the carcass alone gives Crr 0.00514 front and
0.00549 rear, more than 15 % below the drum targets. The fitted `tread_loss_crr` values are
0.00586 front and 0.00481 rear at 1.5 bar; they scale with `p⁻⁰·³`. With that correction,
the simulated Crr is 0.01100 front and 0.01030 rear at the reference pressure. Across
1.3–1.8 bar the fitted exponents are −0.335 and −0.340.

The current hysteresis fits are `η` = 0.0662 front and 0.0690 rear. A pure-NumPy 44 kg sled
dropped 10 mm onto the tyre gives mean log-decrement damping ratios of ≈3.75 % on both tyres
over the first three rebound cycles. A small viscous element term is permitted only if the
damping ratio falls below 2 % with `η` at the top of its band.

**Rim.** For `δ_i > δ_rim` the element adds `k_rim·(δ_i − δ_rim)·Δs`, carrying the same
hysteresis. `k_rim = 2.8×10⁷ N/m²` (`rim_stiffness_n_mm2 = 28.0`) makes a 50 mm rim patch
add ≈ 1 400 N/mm. This is stiff compared with the tyre and keeps the 0.5 ms explicit step
stable for the compiled front-wheel mass (below). A **rim-strike event** opens when any
element first exceeds `δ_rim` and closes when none does. It records `x`, speed, peak wheel
load, peak rim force and the peak elastic energy stored in the rim term. The tyre stays
inflated: there is no puncture and no pressure loss (plan D11). For scale, Schwalbe's
edge-drop test damages a Super Trail casing at 61 J and a Super Gravity casing at 95 J
[T4b]. The event energy can be read against those figures, but no threshold is enforced.

**Patches.** Each contiguous run of loaded rays is a patch `p`. Flat road gives one patch, a
square edge two (the flat and the face), and the wheel is airborne when there are none. A
patch has:

- normal load `N_p`: the magnitude of the vector sum of its element forces;
- direction `n_p`, and tangent `t_p ⟂ n_p` in the sagittal plane, pointing forward;
- centroid `c_p`: the force-weighted mean of the element road points;
- length `2a_p`: the distance between its first and last road points;
- mean deflection `δ_p`.

**Application** (physical runtime; see §0 for the legacy `xfrc` form). The tangential force
`F_t,p` comes from §4.1. Each patch force `F_p = N_p·n_p + F_t,p·t_p` acts at `c_p` and is
mapped into `qfrc_applied` through the point Jacobian at `c_p`. The moment about the axle
is what drives and brakes the wheel against the road: no separate wheel torque is written.
Velocities used by the tyre come from `mj_objectVelocity` on the wheel body in world axes,
so ω is the wheel's **absolute** spin. That differs from `qvel[*_wheel_spin]`, which is
relative to a fork or swingarm that itself pitches.

**Loads for gating.** The contact snapshot takes its wheel channels from the tyre:
`*_load_n` is `Σ_p N_p`, unbridged, because nothing drops out; `*_support_n` is the
vertical component of `Σ_p N_p·n_p`. The snapshot is built from the tyre evaluated at the
top of the step, before the other writers run, so every writer sees one snapshot.

**Stability budget.** The tyre force is integrated explicitly, like the suspension, so
`ω·dt ≤ 0.4` must hold for the stiffest element set a wheel can see. The tyre alone gives
`√(63 000 / 2.6) · 0.0005 ≈ 0.08`. At 2 bar, the fitted high-frequency carcass stiffness is
at most 87.5 N/mm. Together with the 50 mm rim patch (1 400 N/mm), this gives
`ω·dt = 0.394` for the compiled 2.40 kg front wheel and `0.365` for the 2.80 kg rear wheel.
The `detailed` tier halves those values with its 0.25 ms step. A test asserts the bound from
the compiled wheel masses.

**Tiers.** The carcass and surface laws are shared. `detailed` also gives each bristle a
0.5 ms dashpot (`BRISTLE_DAMPING_S = 0.0005`); this regularizes a self-excited wheel-slip
mode measured near 33 Hz when the per-bristle spring was coupled to wheel rotation. The
dashpot is a numerical regularizer for the resolved brush, not a measured tyre property. The
full-traverse convergence test covers the resulting ride metrics.

| Tier | Rays over ±75° | Spacing at the tread | Tread model (§4.1) | Timestep |
|---|---|---|---|---|
| `fast` | 64 | ≈ 15 mm | lumped (steady-state) brush | 0.0005 s |
| `detailed` | 256 | ≈ 4 mm | discretised brush | 0.00025 s |

`fast` must keep the interactive viewer at ≥ 1× real time. `detailed` may run 2–3× slower
and exists as `fast`'s reference: a convergence test holds `fast`'s summary metrics to
`detailed`'s.

**Tyre mass** stays on the wheel body. There is no separate belt or ring body.

---

## 4.1 `pneumatic`: brush slip

**Surface.** The tyre asks the `SurfaceMap` for the surface at the contact position and
never holds a surface itself. The default comes from the track: the `road_*` presets and
track files with a `[generator]` block are `asphalt`, everything else is `hardpack`.
`tires.pneumatic.surface` (physical mode) or `--surface` (legacy) overrides. All values are
authored (§11.2; `terrain/surface.py:98-112`):

| Surface | μ peak | μ sliding | C_κ/F_z | Stribeck speed | Description |
|---|---|---|---|---|---|
| `asphalt` | 1.05 | 0.75 | 15 | 4.5 m/s | dry asphalt |
| `hardpack` | 0.80 | 0.60 | 12 | 4.5 m/s | dry hard-packed dirt |
| `loose` | 0.55 | 0.45 | 7 | 4.5 m/s | loose gravel over hardpack |
| `wet` | 0.50 | 0.40 | 10 | 4.5 m/s | wet dirt |
| `scree` | 0.45 | 0.38 | 4.5 | 4.5 m/s | alpine scree: loose rock debris over bedrock (added 2026-10-11) |

`V_str = 4.5 m/s` is an authored numerical choice: with the `lumped` curve it keeps peak
force within 10 % of each surface's `μ_peak` from 15 to 45 km/h.

Friction falls from static to sliding with the sliding speed:
`μ(V_s) = μ_slide + (μ_peak − μ_slide) · exp(−|V_s| / V_str)`.

The lumped curve's peak-slip estimates on the uniform reference patch (`N = 418 N`,
`a = 61 mm`) are recorded below as **braking / drive** pairs. They are estimates from the
authored curve, not acceptance limits; the test asserts peak force, not `κ_peak`. (`scree`
was added after this table was computed and has no row.)

| Surface | 15 km/h | 30 km/h | 45 km/h |
|---|---|---|---|
| asphalt | −0.153 / 0.206 | −0.144 / 0.186 | −0.138 / 0.174 |
| hardpack | −0.149 / 0.199 | −0.141 / 0.182 | −0.135 / 0.172 |
| loose | −0.173 / 0.248 | −0.166 / 0.229 | −0.161 / 0.218 |
| wet | −0.119 / 0.151 | −0.114 / 0.141 | −0.110 / 0.135 |

**Slip, per patch.**

- `V_x = v_hub · t_p` is the hub's speed along the patch tangent.
- `R_e = R − δ_p/3` is the effective rolling radius of a loaded pneumatic tyre
  (`model.py:312-313`).
- `V_s = V_x − ω · R_e` is the tread's sliding speed over the road, with ω the absolute spin
  (§3.1).
- `κ = −V_s / |V_x|` is the longitudinal slip: positive when driving, −1 for a locked wheel.

**Transient slip.** `σ · dκ'/dt + |V_x| · κ' = −V_s`, with relaxation length σ = 60 mm
[est]. It is integrated with its exact exponential solution (`brush.py::relax`), which is
stable for any timestep and finite at `V_x = 0`. At rest with no torque, `κ'` holds its
value like a deflected tread, so a parked bike does not creep. Under a torque at rest, the
force builds over a distance rolled, not instantaneously. That is the property that makes a
slip model well-posed near zero speed, where `κ` itself is 0/0.

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
  patch fully sliding under brake torque. Both are logged.
- **Rolling resistance** in `pneumatic` is the hysteresis moment of §3.1, plus the tread-loss
  term. `RollingResistance` (`Crr·N·r`) is not applied.
- The traction limit is no longer a fixed `μN`. On `asphalt` at a static rear load of
  610.9 N it peaks near `1.05 · 610.9` ≈ 641 N, or ≈ 224 N·m at the rear wheel's `R_e`, and
  falls to `μ_slide` once the tyre slides. On `loose` (≈ 117 N·m) and `wet` (≈ 107 N·m)
  full drive spins the rear wheel on flat ground. That is intended physics, and it is
  logged, not suppressed (plan D12).

**Aerodynamic drag is not modelled.** At 25 km/h this omits roughly 13 N, about 90 W.

---

## 11.1 The `sphere` tyre against the literature

The reason §3.1 exists:

| Quantity | `sphere` | Measured, 29×2.3–2.4 MTB tyres |
|---|---|---|
| Vertical stiffness | 130 N/mm, linear, pressure-independent | ≈ 62 N/mm static at 1.72 bar / 418 N; 74–86 N/mm dynamic [T1][T2] |
| Vertical damping | 800 N·s/m, viscous | ≈ 100–190 N·s/m, ζ ≈ 2–5.5 % [T2] |
| Static deflection at this bike's loads | 3–5 mm | ≈ 7–10 mm [derived from T1] |
| Rolling resistance | Crr 0.015 | 0.0103 (Hans Dampf, Addix Soft) / ≈ 0.011 (Magic Mary, Soft) at 1.5 bar [T4] |
| Friction | μ 1.2 on every surface | asphalt peak ≈ 1.0–1.1, hardpack ≈ 0.7–0.85 [est, T5][T6] |

## 11.2 `pneumatic` parameters

**Literature-derived.** These are targets the model is fitted to or tested against, with
their stated tolerances (constants: `physics/tyre.py:245-273`):

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
| Detailed bristle dashpot τ_b | 0.5 ms | numerical regularizer; suppresses the measured 33 Hz wheel-slip mode (§4.1) |
| Normalised slip stiffness C_κ/F_z | 15 / 12 / 7 / 10 / 4.5 by surface | MTB cornering stiffness as proxy [T1]; car and trekking data [T5]; `scree` authored 2026-10-11 by the alpine track design |
| Surface μ peak / sliding | §4.1 table | car Burckhardt curves scaled to MTB data [T6] |
| Stribeck speed | 4.5 m/s | numerical; preserves the 15–45 km/h peak-μ target on all surfaces |
| Ray coverage and counts | ±75°; 64 / 256 | §3.1 coverage argument; tuned in plan task 10 |
| Default pressures | 1.5 bar front / 1.7 bar rear | plan D5 |

The user's own measurements (plan Appendix A: loaded deflection at two pressures per wheel,
at 1.3 and 1.8 bar, expected `δ` 7–10 mm) would replace the stiffness fit's literature
anchor with this bike's tyres. When that happens, the rows affected move from
"literature-derived" to "measured". As of 2026-10-14 no such measurement exists in the
tree.

**What is not covered by any source above.** Lateral (cornering, camber) behaviour — the
planar model has none (§12 item 9); the spatial-3D design pins it separately
(`docs/superpowers/specs/2026-10-14-spatial-3d-design.md` §10.4). Deformable ground — every
surface is a friction curve on a rigid road (§12 item 10).

### Sources

- **[T1]** Dressel & Sadauckas, "Characterization and Modelling of Various Sized Mountain
  Bike Tires and the Effects of Tire Tread Knobs and Inflation Pressure", *Applied Sciences*
  10(9):3156, 2020 — Figs 8, 16, 17. https://www.mdpi.com/2076-3417/10/9/3156
- **[T2]** Sadauckas et al., *Vehicle System Dynamics*, 2024 — Tables 2–3, Fig. 9.
  https://www.tandfonline.com/doi/full/10.1080/00423114.2024.2398001
- **[T3]** bicyclerollingresistance.com — Schwalbe Hans Dampf TrailStar 2017 review (section
  measurements), load test, test protocol.
  https://www.bicyclerollingresistance.com/mtb-reviews/schwalbe-hans-dampf-trailstar-2017 ·
  https://www.bicyclerollingresistance.com/specials/crr-load-test ·
  https://www.bicyclerollingresistance.com/the-test
- **[T4]** ENDURO Mountainbike Magazine, Schwalbe tyre lab test 2025 (1.5 bar, 50 kg,
  20 km/h). https://enduro-mtb.com/en/schwalbe-mountain-bike-tires-review/
- **[T4b]** ENDURO Mountainbike Magazine, tyre insert test (Schwalbe edge-drop data).
  https://enduro-mtb.com/en/best-tire-insert/
- **[T5]** O. Maier, dissertation, KIT (bicycle tyre longitudinal behaviour).
  https://www.hs-pforzheim.de/fileadmin/user_upload/uploads_redakteur_technik/10_Forschung/Diss_O_Maier_978-3-7315-0778-9.pdf
- **[T6]** Burckhardt tyre–road friction parameters (car), as cited in
  https://www.researchgate.net/figure/Burckhardt-tyre-model-parameters-for-different-friction-coefficients_tbl1_270684635
- TU Delft bicycle tyre vertical testing (background). http://bicycle.tudelft.nl/tiretestingvertical/

Not used by the planar model, recorded for the 3D lateral set (spec §10.4): Dell'Orto et
al., *Vehicle System Dynamics* 2024, DOI 10.1080/00423114.2024.2338143 — Magic Formula
(B, C, D, E for Fy, Mz) for five bicycle tyres on the VeTyT rig, open data Zenodo 7866646 /
7920415, code github.com/mechmotum/Magic-Formula-89. Urban/cargo tyres only, 300–500 kPa,
343–526 N, camber ±5°, 9.3 km/h; lateral-force saturation not reached. Not an MTB source;
usable as a measured cross-check of `C_α/F_z` and of relaxation-length order of magnitude
within its stated ranges, not for extrapolation to 1.5 bar knobby tyres.

---

## 12. Known non-physicalities of the tyre

(Items 9–14 of the original §12; items 1–8 concerned the rider, suspension and the legacy
`sphere` tyre.)

**9 — No lateral tyre forces.** No cornering or camber force and no turn slip, which is
implied by the planar model class. Only the longitudinal half of the brush exists.

**10 — Rigid road.** The surface presets change the friction curve and nothing else. `loose`
and `scree` do not sink, rut or shed material, and no surface deforms under the tyre.

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
