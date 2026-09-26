# Pneumatic Tyre Model Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` to implement this plan task-by-task.

**Goal:** Give ride mode a second, selectable tyre model, `pneumatic`, that replaces the
contact sphere's linear spring and Coulomb friction with a physically parameterised tyre:
inflation-pressure-dependent, non-linear vertical stiffness, envelopment of sharp edges,
rim strike, rate-independent carcass hysteresis (from which rolling resistance emerges),
and brush-model longitudinal slip with a relaxation length. The existing `sphere` model
stays the default, unchanged, until the new model has been validated and the user has
signed off on an A/B comparison.

**Architecture:** In `pneumatic` mode MuJoCo's wheel–road contact is switched off. A
Python force model evaluates each wheel against the known 1-D road profile `h(x)` once per
step — a ring of radial elements for the carcass, a brush for the tread — and writes the
resulting force and moment into `data.xfrc_applied` on the wheel body, in the same
writer phase as the suspension and rider forces. Everything that used to read wheel load
from `mj_contactForce` (drive gating, virtual rider, crash detector's wheel channels,
rolling resistance) reads it from the tyre model instead. The model has two fidelity
tiers, `fast` and `detailed`, which share formulas and differ in resolution.

**Tech Stack:** Python 3.12+, MuJoCo 3.12, NumPy, SciPy, Matplotlib, pytest, `uv`. No new
dependencies.

**Spec:** [`docs/RIDE.md`](../../RIDE.md), the Physics Contract. Task 1 amends it *before*
any code is written; from then on the amended contract is the binding authority and
conflicts inside this plan resolve against it.

**Predecessors:** [`2026-08-25-ride-mode.md`](2026-08-25-ride-mode.md) (ride mode),
[`2026-09-26-rough-road.md`](2026-09-26-rough-road.md) (tracks, telemetry, CLI). Both landed;
HEAD is `7062381` (seated rider).

---

## Decisions (settled 2026-09-26, design interview with the user)

| # | Decision |
|---|---|
| D1 | Scope is **vertical carcass behaviour** (non-linear pressure-dependent stiffness, envelopment, rim strike, hysteresis) **and longitudinal slip**. No lateral / cornering dynamics — the model stays planar (RIDE.md §0). No visual tyre work. |
| D2 | Two tyre models selected by configuration: `sphere` (today's, **default**) and `pneumatic`. The default flips only per D17. |
| D3 | Two fidelity tiers of `pneumatic`, selected by configuration: `fast` must keep the interactive viewer at ≥ 1× real time; `detailed` may run 2–3× slower. |
| D4 | Tyre forces come from an **own force model** applied through `xfrc_applied`; wheel–road MuJoCo contact is disabled in `pneumatic`. The contact-dropout debounce is not used by `pneumatic`. |
| D5 | Modelled tyres: **Schwalbe Magic Mary 29×2.4 front, Hans Dampf 27.5×2.4 rear**, tubeless, no insert. Default pressures **1.5 bar front / 1.7 bar rear**. |
| D6 | **Pressure is an input**: vertical load = pressure × contact area + carcass term; front and rear independent; set on the command line and adjustable live from the viewer keyboard, like the fork's psi. |
| D7 | Validation: **literature curves for shape, the user's own measurements for calibration** (two points per wheel, Appendix A). Every acceptance criterion is a test. |
| D8 | Longitudinal slip: **brush model** (static/sliding μ, tread stiffness), contact length taken from the vertical model, with a **relaxation length** so the model is well-posed at low speed and at rest. |
| D9 | Surfaces: **one surface per run**, chosen from presets (asphalt, hardpack, loose, wet). The data structure must admit per-zone surfaces in the track file later without a redesign. |
| D10 | Vertical model: **ring of radial elements** (Davis-type radial-spring model). Forces are summed vectorially, so a square edge produces a rearward force component. Tyre mass stays on the wheel body; no separate belt/ring body. |
| D11 | Rim strike: a **stiff rim-contact element** engages when an element's deflection reaches the section height; every strike is logged (load, speed, position, energy). The tyre stays inflated — no puncture simulation. |
| D12 | Controllers are **not** given ABS or traction control. Wheelspin and lock-up happen and are logged. The cruise integrator gets one extra anti-wind-up condition for a traction-saturated rear tyre. |
| D13 | Telemetry: per-wheel tyre channels, HUD line, live pressure keys, a tyre figure, and a **comparison script** running every preset on both models. |
| D14 | Tiers differ in resolution, not in physics: `fast` 64 elements, lumped (steady-state) brush, dt 0.5 ms; `detailed` 256 elements, discretised brush, dt 0.25 ms. A convergence test holds `fast` to `detailed`. Numbers are starting values, tuned by measurement (Task 10). *(Agreed as ≈ 50 / ≈ 250 over ±60°; the contract widened coverage to ±75° — see RIDE.md §3.1 — which at the same spacing is 64 / 256.)* |
| D15 | Rolling resistance **emerges from carcass hysteresis**, calibrated against drum tests. `RollingResistance` (`Crr·N·r`) is not applied in `pneumatic`. Hysteresis is rate-independent, not viscous. |
| D16 | Default surface is a **property of the track**: `road_*` presets → asphalt, all others → hardpack; the command line may override. |
| D17 | `pneumatic` becomes the default only when (a) the full suite is green including calibration and convergence tests, **and** (b) the user has reviewed the comparison report and said yes. The existing ride golden baselines are regenerated in that same commit, and not before. |

### Defaults chosen during planning (confirmed by the user)

| # | Decision |
|---|---|
| D18 | Elements sample the **same profile array** the heightfield is rasterised from (`build_profile(track, field.track_x())`, metres, 5 mm), linearly interpolated, so physics and picture cannot disagree. |
| D19 | The heightfield geom and the catch plane stay. Frame and handlebar collisions with the road stay enabled (crash rendering and the handlebar crash trigger depend on them). Only the two wheel contact spheres become non-colliding (`contype=0 conaffinity=0`); they remain in the model because `resolve_wheel_spin` reads the wheel radius from them. |
| D20 | Tyre parameters live in a new `TyreSpecs`, in the style of `AirSpringSpecs`. New flags: `--tyre-model {sphere,pneumatic}`, `--tyre-tier {fast,detailed}`, `--tyre-pressure FRONT/REAR` (bar), `--surface NAME`. |
| D21 | In `pneumatic`, drive gating, the virtual rider and the crash detector's wheel inputs take wheel load from the tyre model (no bridging). The handlebar channel still comes from MuJoCo contact. |
| D22 | Wheel masses and inertias are unchanged; `test_mass_distribution` is untouched. |
| D23 | Starting parameters are literature values (Key facts, below); the user's measurements refine them in Task 12. |
| D24 | RIDE.md is corrected where this work touches it, including the wheel-spin damping mismatch (§4 says "0.05 and 0.01"; code and XML have 0.01 on both). |

---

## Global Constraints

- `uv` for every Python invocation (`uv run python -m pytest`, `uv run python …`).
- **`sphere` is untouched.** Every change to a shared module is gated on
  `tyre_model == "pneumatic"`. With `tyre_model="sphere"` (the default) the emitted XML
  is byte-identical to today's — `tests/golden/baseline_bike_ride.xml` and
  `baseline_bike_ride_lumped.xml` do not change until D17 — and a `sphere` run is
  bit-identical step for step. If a golden test fails before Task 13, the gating is wrong:
  fix the gating, never regenerate the baseline.
- Every existing test keeps passing unchanged. New `pneumatic` behaviour gets new tests.
- No source file over 500 lines. `sim/ride/tyre/` is a package precisely so that no module
  has to be long.
- **Layering:** `bike_sim.terrain` imports no MuJoCo and no other `bike_sim` package
  (surfaces live there as pure data). `bike_sim.physics.tyre` is a pure calculator in the
  package's **mm / bar** convention and never touches `mjModel`/`mjData`. The per-step
  kernels in `sim/ride/tyre/` work in **metres**, because they consume the terrain profile
  in metres; every argument carries its unit suffix. Only `sim/ride/tyre/applier.py`
  writes to `data`.
- **`xfrc_applied` has exactly one writer**, the tyre applier, and it **assigns** both
  wheel bodies' rows on every step, zero included — MuJoCo never clears it. (No module
  writes `xfrc_applied` today; `grep` confirms.)
- Tolerances in tests are **measured, then recorded in the docstring** with their source —
  never invented. Literature targets carry the tolerance stated in Key facts.
- Seeded generators only. Docstring style as in the repository (module docstring with a
  title line; Google-style `Args:`/`Returns:`/`Raises:`).
- One commit per task, message style as in `git log` (`feat(sim): …`, `docs: …`).

---

## Key facts

### Current `sphere` model vs. the literature

The comparison below is the reason this work exists; it goes into RIDE.md §11 verbatim.

| Quantity | `sphere` today (RIDE.md §3, §4, §11) | Literature, 29×2.3–2.4 MTB tyre |
|---|---|---|
| Vertical stiffness | 130 N/mm, linear, pressure-independent | **≈ 62 N/mm static** at 1.72 bar / 418 N; 74–86 N/mm dynamic [1][2] |
| Vertical damping | 800 N·s/m, viscous | **≈ 100–190 N·s/m**, ζ ≈ 2–5.5 % [2] |
| Static deflection | 3–5 mm (§3) | **≈ 7–10 mm** at our wheel loads and pressures [derived from 1] |
| Rolling resistance | Crr 0.015, constant | **0.010–0.011** at 1.5 bar (Addix Soft), 0.0176 for Ultra Soft [5] |
| Friction | μ 1.2, Coulomb, every surface | asphalt peak ≈ 1.0–1.1, hardpack ≈ 0.7–0.85 [est, 7][8] |

### Model geometry (from the code)

| Quantity | Value | Where |
|---|---|---|
| Tyre outer radius, front / rear | 0.372 / 0.352 m | `geometry/specs.py:21-22`, contact spheres |
| Rim geom radius, front / rear | 0.320 / 0.300 m | `mujoco/steering_fork.py::_build_front_wheel`, `mujoco/drivetrain.py::build_rear_wheel` |
| Tyre height above rim, both | **52 mm** | difference of the two rows above |
| Static wheel loads (system) | **seated rider (default): 413.3 N front, 610.9 N rear**; lumped: 478.2 / 546.0 N | RIDE.md §9 |
| Ride timestep | 0.0005 s, `implicitfast` | `mujoco/builder.py`, RIDE.md §10 |
| Wheel masses | 2.40 kg front, 2.80 kg rear | `physics/mass.py:52-57` |
| Per-step writer order | suspension → rider → rolling resistance → cruise → brakes → virtual rider → crash check → `mj_step` → contact query | `sim/ride_sim.py:192-223` |

### Starting parameters (literature; [est] = estimate, flagged "authored" in RIDE.md §11)

| Parameter | Starting value | Basis |
|---|---|---|
| Static vertical stiffness law | `k(p) ≈ 22 + 24·p[bar]` N/mm, **±15 %** → 58 N/mm @ 1.5 bar, 63 N/mm @ 1.7 bar | derived from [1] (29×2.3, 25 mm rim) |
| Load carried by the pressure term | 70–100 % at nominal pressure → carcass term starts at **15 %** of stiffness | [1] (p·A of the bald tyre ≈ 107 % of load) |
| Contact length at 418 N | ≈ 133 mm @ 1.38 bar, ≈ 122 mm @ 1.72 bar, **±15 %** | [1] Fig. 16 |
| Dynamic / static stiffness | 1.16–1.35 | [2] |
| Loss factor η | 0.05–0.09 (ζ 2–5.5 %) | [2]; rate-independent hysteresis appropriate |
| Section height | Hans Dampf 29×2.35: 55 mm; Magic Mary 2.4: 57–60 mm [est] | [4] |
| Rim-strike deflection | 0.80–0.85 × H ≈ **45–50 mm**; model: 52 mm − ~6 mm compressed casing and tread ≈ **46 mm** | [1] (48.7 mm without rim strike at 1.03 bar), [est] |
| Casing damage energy (reference only) | Super Trail 61 J, Super Gravity 95 J (19 kg edge drop, 1.5 bar) | [6] |
| Crr, Hans Dampf SG Addix Soft 29×2.35 | **0.0103** @ 1.5 bar, 490.5 N, 20 km/h, steel drum | [5] |
| Crr, Magic Mary Addix Soft | **≈ 0.011** [est] (Ultra Soft measured 0.0176; Soft ≈ Ultra Soft / 1.6) | [5] |
| Crr vs pressure | Crr ∝ p^−0.3 | derived from [3] |
| Longitudinal slip stiffness | C_κ/Fz ≈ 15 (10–20) hard surfaces, 5–10 loose [est] | [7], MTB cornering stiffness as proxy [1] |
| Brush tread stiffness | c_px ≈ 8×10⁵ N/m² [est] | from C_κ and contact half-length |
| Longitudinal relaxation length | σ ≈ **90 mm** (60–120) [est] | lateral 160 mm derived from [1]; road 79–141 mm |

### Surface presets (all [est]; car-tyre curves scaled to MTB data where it exists)

| Preset | μ peak (κ at peak) | μ sliding | C_κ/Fz |
|---|---|---|---|
| `asphalt` | 1.05 (0.10–0.17) | 0.75 | 15 |
| `hardpack` | 0.80 (0.15–0.25) | 0.60 | 12 |
| `loose` | 0.55 (0.2–0.4, flat after peak) | 0.45 | 7 |
| `wet` | 0.50 | 0.40 | 10 |

---

## Physics

The physics is specified in the amended contract, not here: **RIDE.md §3.1** (radial
elements, element force, hysteresis and its calibration order, rim, patches, application
through `xfrc_applied`, gating loads, stability budget, tiers) and **§4.1** (surfaces,
slip, transient slip, tread stiffness, lumped and discretised brush). Task 1 wrote them.
Points settled while writing the contract, which the tasks below rely on:

- Rays are **world-fixed** (non-spinning, non-pitching), centred on the hub, over **±75°**.
  ±60° truncates the patch at the far edge of a 150 mm pothole (contact ≈ 55° off vertical).
- ω for slip is the wheel's **absolute** spin from `mj_objectVelocity`, not
  `qvel[*_wheel_spin]`, which is relative to a pitching fork or swingarm.
- `w(δ) = 2·√(δ(2ρ − δ))` with `ρ = W_c/2`, scaled by a fitted effective-area factor `c_A`.
- **η calibration order:** the vertical damping ratio (2–5.5 %) first; then Crr. If Crr
  ends up more than 15 % short, a tread-loss term `(Crr_target − Crr_η)·N_p` closes the gap,
  and the contract records it.
- `c_px = (C_κ/F_z)_surface · N_p / (2a_p²)`, so the slip stiffness scales with load.
- The discretised brush filters its slip input with `σ − a_p`, because bristle transport
  already contributes ≈ `a_p` of lag. Both tiers then carry the same total relaxation.
- `k_rim = 3.0×10⁷ N/m²`, which is ≈ 1 500 N/mm over a 50 mm rim patch, giving
  `ω·dt ≈ 0.38`.
- *Added while starting Task 3 (RIDE.md §3.1 amended):* the element's rate of deflection is
  the **material rate** `Dδ/Dt = ∂δ/∂t|ray − ω·∂δ/∂θ`. Without the transport term, steady
  rolling on flat road has no hysteresis loss at all. Material states are advected along
  the ray grid.
- *Added:* a **Maxwell branch** per element (standard linear solid, `k_r` fitted, τ = 0.2 s)
  reproduces the measured dynamic/static stiffness 1.16–1.35. Hysteresis alone gives only
  ≈ 1.07.
- *Added:* a **contact-length factor** `c_L`, because the circle–plane chord overstates the
  measured footprint by ≈ 15 %.

---

## Task Breakdown

### Task 0: Baselines — performance and today's behaviour

Nothing changes in `src/`. This task produces the numbers the budget and the comparison are
judged against.

**Files:**
- Create: `tools/bench_ride.py`

**Steps:**

- [x] `bench_ride.py`: headless runs of every preset with `sphere`, reporting steps/s, the
  real-time factor, and the share of each writer (time `applier`, `rider_forces`, contact
  query, `mj_step` separately with `time.perf_counter`). Seeded, `--decimate` recording off.
- [x] Run it on the development Mac; record the results in this plan under "Measured
  baseline" (a new section at the bottom) and in RIDE.md §10, which today quotes only the
  22.5× of a bare sphere on a heightfield.
- [x] Derive the per-step **tyre budget** for `fast`: the time per step that keeps the
  interactive viewer at ≥ 1× real time with a 30 % margin. Write it next to the baseline.

**Verification:** the script runs from `uv run python tools/bench_ride.py`; numbers recorded.

---

### Task 1: Physics Contract amendment

Written first, reviewed with the user, then binding.

**Files:**
- Modify: `docs/RIDE.md`

**Steps:**

- [x] §3: keep the current table under a new heading "### 3.0 `sphere` (default)". Add
  "### 3.1 `pneumatic`" with the radial-element model, pressure term, carcass term,
  hysteresis, rim strike, stability budget and the D18/D19 facts (profile source, which geoms
  still collide). Section numbers of existing sections do **not** change — code docstrings
  cite them.
- [x] §4: add "### 4.1 `pneumatic`: brush slip" — patches, slip velocity, relaxation, the two
  tiers, μ(V_s), surfaces (D9, D16). Fix the wheel-spin damping sentence (D24).
- [x] §6: brakes keep `BRAKE_TAPER_RADPS = 1.0`, which fades the brake torque out below
  1 rad/s. Document the consequence: in `pneumatic` a "locked" wheel shows as slip → −1
  with ω held inside the taper band, not ω = 0. That is the intended behaviour, not a bug.
- [x] §8: add **tyre hysteresis** to the sink column (it replaces "contact dissipation" and
  "rolling resistance" for `pneumatic`).
- [x] §10: the `detailed` timestep (0.25 ms) and the measured baseline from Task 0.
- [x] §11: the "current vs literature" table from Key facts; every [est] parameter listed as
  authored, with its source.
- [x] §12: items 5 and 8 become "`sphere` only"; add new items for what `pneumatic` still
  does not do: no lateral forces, no camber, no tread-block or rubber-temperature effects,
  no carcass dynamics above the element model, the same rigid road (no deformable soil).
- [x] Usage: new flags, keys, telemetry channels and summary fields (filled in by Task 9,
  placeholders here).

**Verification:** the user has read §3.1 and §4.1. No code yet.

*Status 2026-09-26: drafted in `docs/RIDE.md` — status note, §1, §3.0/§3.1, §4.0/§4.1, §6,
§8, §9, §10, §11.1/§11.2, §12 items 9–14, and pending notes in Usage. Awaiting the user's
read.*

---

### Task 2: Surfaces

**Files:**
- Create: `src/bike_sim/terrain/surface.py`
- Modify: `src/bike_sim/terrain/profile.py` (`TrackSpec`)
- Modify: `src/bike_sim/terrain/presets.py`, `src/bike_sim/terrain/road.py`
- Modify: `src/bike_sim/terrain/trackfile.py`
- Modify: `src/bike_sim/terrain/__init__.py`
- Create: `tests/test_surface.py`

**Steps:**

- [ ] `surface.py`: frozen `SurfaceSpec(name, mu_peak, mu_slide, slip_stiffness_per_load,
  stribeck_speed_mps)`; `SURFACES` with the four presets from Key facts; `get_surface(name)`.
  A `SurfaceMap` with `at(x_m) -> SurfaceSpec` that today holds one surface: this is the
  seam for per-zone surfaces (D9), so the tyre model only ever asks the map.
- [ ] `TrackSpec` gains `surface: str = "hardpack"`. The `road_*` generators set
  `"asphalt"`; `enduro_aggressive`, `flat`, `single_edge`, `washboard_only` keep the default
  (D16). `TrackSpec.validate` rejects unknown names.
- [ ] `trackfile.py`: optional top-level `surface = "…"` key, read next to `description`
  (`trackfile.py:342-347`) and validated against `SURFACES`; written by `track_to_dict`
  (`:413-418`) and the emitter's top-level loop (`:455`). A file without the key follows the
  preset rule (D16): a file with a `[generator]` block describes a road and gets `asphalt`,
  a hand-placed-only file gets `hardpack`.
- [ ] Tests: presets, lookup errors, the per-track defaults, TOML round-trip, and that
  `build_field_data` output is unchanged for every preset (surfaces must not touch geometry).

**Verification:** `uv run python -m pytest -q`; all golden baselines untouched.

---

### Task 3: `TyreSpecs` and the literature laws

**Files:**
- Create: `src/bike_sim/physics/tyre.py`
- Modify: `src/bike_sim/physics/__init__.py`
- Create: `tests/test_tyre_specs.py`

**Steps:**

- [ ] `TyreSpecs` (mm, bar): `name`, `outer_radius_mm`, `rim_radius_mm`,
  `section_height_mm`, `casing_width_mm` (60), `compressed_casing_mm` (6), `pressure_bar`,
  `area_factor` (`c_A`, fitted), `carcass_stiffness` (`k_c`, fitted), `loss_factor`
  (starting 0.07), `relaxation_length_mm` (90), `rim_stiffness_n_m2` (3.0×10⁷),
  `hysteresis_rate_eps_mps` (0.01), `tread_loss_crr` (0 unless RIDE.md §3.1's fallback is
  needed), `contact_length_factor` (`c_L`, fitted), `rate_stiffening` (`k_r`, fitted),
  `rate_relaxation_s` (τ, 0.2). Derived: `rim_strike_deflection_mm = outer − rim −
  compressed_casing` (46 mm).
  Tread stiffness is not a tyre field: it follows from the surface's `C_κ/F_z` (RIDE.md §4.1).
- [ ] `FRONT_TYRE` (Magic Mary 29×2.4) and `REAR_TYRE` (Hans Dampf 27.5×2.4) from D5, radii
  taken from `BikeSpecs` and the builder's rim radii so they cannot drift (assert equality in
  a test).
- [ ] Target functions used only by tests and calibration: `static_stiffness_target_n_mm(p)`,
  `contact_length_target_mm(load_n, p)`, `crr_target(tyre, p)` with the p^−0.3 law.
- [ ] `TyreConfig(model="sphere"|"pneumatic", tier="fast"|"detailed", front, rear,
  surface_override=None)` — the single object that `RideSimulation` and the CLI pass around.

**Verification:** pure-calculator tests, no MuJoCo import in `physics/tyre.py`.

---

### Task 4: Radial-element geometry kernel

**Files:**
- Create: `src/bike_sim/sim/ride/tyre/__init__.py`
- Create: `src/bike_sim/sim/ride/tyre/geometry.py`
- Create: `tests/test_tyre_geometry.py`

**Steps:**

- [ ] `RayRing(n_rays, half_angle_rad=5π/12)`: precomputed **world-frame** unit directions
  over ±75° about −Z (RIDE.md §3.1); no per-step rotation.
- [ ] `ProfileSampler(x_m, z_m)`: wraps the profile array (D18); `window(x_lo, x_hi)` returns
  a view, no copy.
- [ ] `intersect(ring, centre_xz, R, sampler) -> (r, visible)`: transform the profile samples
  inside `[x_c − R, x_c + R]` to polar coordinates about the wheel centre, keep the visible
  (nearest-radius) branch, and interpolate each ray's surface radius. Vectorised, no per-ray
  Python loop, no allocation beyond preallocated buffers.
- [ ] `patches(delta) -> list of (start, stop)` contiguous loaded runs.
- [ ] **Oracle test:** a brute-force per-ray bisection on `h(x)`, compared on flat road,
  the `single_edge` step, a sharp pothole, a bowl pothole and a washboard, at several wheel
  heights: deflections agree to 0.1 mm. On a square edge there are exactly two patches; on
  flat road one; airborne none. A 150 mm pothole's far edge is fully inside the coverage;
  road beyond ±75° raises a coverage event, and no shipped preset raises one.
- [ ] Cheap airborne cull: if the wheel's lowest point is above the window maximum, return
  no contact without intersecting.

**Verification:** tests; a micro-benchmark in the test docstring records µs per call for
64 and 256 rays.

---

### Task 5: Carcass force, hysteresis, rim strike

**Files:**
- Create: `src/bike_sim/sim/ride/tyre/carcass.py`
- Create: `tests/test_tyre_carcass.py`

**Steps:**

- [x] Element forces per RIDE.md §3.1: elastic `c_A·p·w(δ) + k_c·δ` with
  `w(δ) = 2·√(δ(2ρ − δ))`, Maxwell branch, hysteresis `η·f_e·tanh((Dδ/Dt)/δ̇_ε)`, rim
  term, clamp at ≥ 0. `Dδ/Dt` is the **material rate**: the same-ray difference minus
  `ω·∂δ/∂θ` (upwind). Material states (`f̃_e`, `f_m`) are advected by `−ω·dt`,
  semi-Lagrangian, before the update.
- [x] **Calibration fits** (a small `scipy.optimize` routine in the test helpers, results
  frozen as `TyreSpecs` defaults): `c_A`, `k_c`, `c_L` and `k_r` so that
  - static stiffness follows `22 + 24·p` N/mm within ±15 % over 1.0–2.0 bar;
  - contact length at 418 N is 133 mm @ 1.38 bar and 122 mm @ 1.72 bar within ±15 %;
  - dynamic apparent stiffness in a 44 kg drop-sled simulation (1-DOF, pure NumPy) is
    1.16–1.35 × static (this fits `k_r`);
  - steady rolling on flat road shows a nonzero, forward-shifted centre of pressure (a guard
    against losing the transport term).
- [x] Hysteresis: in the same 44 kg sled, dropped 10 mm onto the tyre, the mean log-decrement
  damping ratio over the first three rebound cycles lands in 2–5.5 %. **This sets η** (RIDE.md
  §3.1 calibration order: damping first). Task 7 then checks Crr with this η and applies the
  tread-loss fallback only if Crr is > 15 % short.
- [x] Rim strike: onset deflection equals `rim_strike_deflection_mm` (≈ 46 mm) and lies in
  0.80–0.85 × section height; the event record carries peak load, speed, x and absorbed
  energy.
- [x] Stability test: `ω·dt ≤ 0.4` using the compiled wheel masses and the 50 mm rim cap.

**Verification:** tests; no MuJoCo import in `carcass.py`.

---

### Task 6: Brush slip with relaxation

**Files:**
- Create: `src/bike_sim/sim/ride/tyre/brush.py`
- Create: `tests/test_tyre_brush.py`

**Steps:**

- [x] `relax(kappa_prime, v_x, v_s, sigma_m, dt)`: exact exponential update of
  `σ·dκ'/dt + |V_x|·κ' = −V_s`, stable for any `dt`, finite at `V_x = 0`.
- [x] `lumped_brush(kappa, N, a, surface, tread_stiffness)`: steady-state brush with
  `μ(V_s)`; returns `F_t` and whether the patch is fully sliding.
- [x] `DiscretisedBrush(n_elements)`: bristle deflections on the ray grid, semi-Lagrangian
  advection, stick/slide per element against `μ·q_i` where `q_i` is that element's share of
  `N`.
- [x] Tests:
  - steady-state curves of the two implementations agree within 3 % of `μN` over
    κ ∈ [−1, 1] on a uniform patch;
  - initial slope equals the surface's `C_κ/F_z` within 5 %; the peak μ is within 10 % of
    `μ_peak` at the sliding speeds of 15–45 km/h; κ at peak is reported (the Key facts
    κ bands are car-derived estimates, recorded, not asserted);
  - step response: κ' reaches 63 % after rolling `σ`;
  - a wheel held at rest under load with zero torque produces zero tangential force and no
    drift over 10 s;
  - the discretised brush is stable at 45 km/h, 4 mm, 0.25 ms (Courant ≈ 0.8) **and** in a
    forced run at Courant 2, since semi-Lagrangian advection must not rely on < 1;
  - with the `σ − a_p` input filter, the discretised brush's step response matches the
    lumped one's total relaxation within 10 %.

**Verification:** tests; pure NumPy.

**Calibration note:** the first Stribeck value (1.0 m/s) put the force peak 7–22 % below
`μ_peak` over the accepted 15–45 km/h sweep. `V_str` is now 4.5 m/s on every surface; the
peak-force test passes within 10 %, while `μ_slide` still sets the high-slip limit.

---

### Task 7: Tyre model and force applier

**Files:**
- Create: `src/bike_sim/sim/ride/tyre/model.py`
- Create: `src/bike_sim/sim/ride/tyre/applier.py`
- Create: `tests/test_tyre_applier.py`

**Steps:**

- [ ] `PneumaticTyre` (one per wheel): holds `TyreSpecs`, its `RayRing`, previous `δ`,
  transient slip / bristle state, the last `WheelOutputs` (Fz, Fx, per-patch data,
  deflection, patch length, slip, rim-strike flag, dissipated power). `reset()` clears all
  state.
- [ ] `TyreForceApplier(model, config, profile, surface_map)`: resolves wheel bodies
  `front_wheel`/`rear_wheel` and their spin DOFs; per step reads the hub centre, the hub's
  linear velocity and the wheel's **absolute** angular velocity (`mj_objectVelocity`, world
  axes — not `qvel` of the spin joint); evaluates both tyres; **assigns** `xfrc_applied` for
  both wheel bodies; exposes the outputs.
- [ ] `detailed` tier sets `model.opt.timestep = 0.00025` after compilation (the XML
  keeps 0.0005).
- [ ] **Rolling-resistance check** (η already set by Task 5): a wheel rolled on flat
  `asphalt` at 20 km/h, 490.5 N, 1.5 bar gives Crr = 0.0103 ± 15 % (Hans Dampf) and
  ≈ 0.011 ± 15 % (Magic Mary); raising pressure lowers Crr with an exponent in −0.2…−0.4.
  If Crr is > 15 % short, set `tread_loss_crr` to the shortfall, record the value in
  RIDE.md §3.1 and tell the user — it is the one phenomenological term the contract allows.
- [ ] Tests: `xfrc_applied` rows are exactly the tyre's (no residue after `reset`); vertical
  sum on the solved equilibrium equals system weight within 0.5 %; moment about the axle
  equals `F_x·R_e`.

**Verification:** tests.

---

### Task 8: MJCF gating and ride orchestration

**Files:**
- Modify: `src/bike_sim/mujoco/builder.py`, `steering_fork.py`, `drivetrain.py`, `exporter.py`
- Modify: `src/bike_sim/sim/ride_sim.py`
- Modify: `src/bike_sim/sim/equilibrium.py`
- Modify: `src/bike_sim/sim/ride/contacts.py`
- Modify: `src/bike_sim/sim/ride/cruise.py`
- Create: `tests/golden/baseline_bike_ride_pneumatic.xml`
- Modify: `tests/test_golden_baselines.py`
- Create: `tests/test_ride_pneumatic.py`

**Steps:**

- [ ] `generate_mujoco_xml(..., tyre_model="sphere")`: for `"pneumatic"` only, the two
  contact spheres get `contype="0" conaffinity="0"`. Nothing else in the XML changes. Add a
  **new** golden baseline for the pneumatic XML; the existing ones are not touched.
- [ ] `RideSimulation(..., tyre: TyreConfig | None = None)`. With `pneumatic`:
  - the tyre applier runs **first** in `step()`, and its outputs build this step's
    `TerrainContacts` (wheel loads and vertical support from the tyre, handlebar from the
    MuJoCo query — D21). The other writers then see the same snapshot, as today;
  - `RollingResistance.apply` is not called (D15);
  - `reset()` resets the tyre state.
  With `sphere` the code path is exactly today's.
- [ ] `TerrainContactQuery`: accept an optional wheel-load provider; when present, skip the
  sphere rows and the bridging for the wheels, keep the handlebar row.
- [ ] `solve_static_equilibrium`: take an optional tyre applier in `apply_forces`, reset its
  transient state at every velocity reset, and take `relax_steps_per_cycle` so the 20 ms
  cycle is preserved at 0.25 ms (defaults unchanged for `sphere`). Record the solved
  `pneumatic` sag and chassis height in RIDE.md §9 (it currently says *pending*).
- [ ] `CruiseController`: when given a traction signal (rear tyre fully sliding in the
  direction of the error), suspend integration as it already does for saturation and for an
  airborne rear wheel (`cruise.py:153-169`). `sphere` passes no signal, so its behaviour is
  unchanged (D12).
- [ ] `tests/test_ride_pneumatic.py`: model compiles; spheres do not collide; equilibrium
  converges on every preset; a `flat` traverse completes and holds speed; `single_edge`
  completes, shows two patches at the edge and a rearward force component; airborne flags
  over the kicker match the geometry; `sphere` runs remain bit-identical to a stored trace
  from before this task.

**Verification:** full suite; the pre-existing golden baselines untouched.

---

### Task 9: Telemetry, HUD, keys, CLI, plots

**Files:**
- Modify: `src/bike_sim/sim/ride/recorder.py`, `metrics.py`, `hud.py`, `input.py`, `session.py`
- Modify: `src/bike_sim/viz/ride_plots.py`
- Modify: `src/bike_sim/cli/ride.py`
- Modify: `tests/test_ride_telemetry.py`, `tests/test_ride_cli.py`, `tests/test_ride_plots.py`

**Steps:**

- [ ] Recorder: append per-wheel channels `{front,rear}_tyre_fz_n`, `_tyre_fx_n`,
  `_tyre_deflection_mm`, `_patch_length_mm`, `_slip_ratio`, `_rim_strike`,
  `_tyre_pressure_bar`, `_tyre_loss_w`. They read zero under `sphere`, so every CSV has the
  same columns whichever model rode (the seated-rider convention, `recorder.py:18-22`).
- [ ] Metrics: `TyreStats` per wheel — peak Fz, max deflection, rim strikes (count, and per
  strike x, speed, peak load, energy), time in wheelspin, time locked, mean dissipated power
  and its Crr equivalent. Summary table and `summary.json` gain them; `extras` carries the
  model, tier, pressures and surface.
- [ ] HUD: under `pneumatic`, a tyre line — pressures, slip front/rear, a rim-strike flash.
- [ ] Keys (only under `pneumatic`; under `sphere` they print a one-line notice):
  `N` / `M` front pressure −/+ 0.05 bar, `;` / `'` rear −/+ 0.05 bar (GLFW 78/110, 77/109,
  59, 39; all currently unbound in `input.py:66-118`). Clamped to 0.8–3.0 bar. Pressure is
  read every step, so no recompilation is needed. Verify the passive viewer does not consume
  these keys; if it does, pick other unbound keys and update the help text.
- [ ] Help text in `hud.py:156-196` gains a "TYRES" block.
- [ ] CLI (`cli/ride.py:64-101`): `--tyre-model`, `--tyre-tier`, `--tyre-pressure F/R`,
  `--surface`; the run header prints model, tier, pressures and surface.
- [ ] Plots: `plot_tyres` — deflection and Fz vs x per wheel with the rim-strike line and
  events, slip ratio vs x, patch length; written only for `pneumatic`.

**Verification:** tests; a headless `road_worn` run on each model writes CSV, summary and
figures.

---

### Task 10: Performance to budget, and tier convergence

**Files:**
- Modify: `tools/bench_ride.py`
- Modify: `src/bike_sim/sim/ride/tyre/*` (optimisation only)
- Create: `tests/test_tyre_convergence.py`

**Steps:**

- [ ] Benchmark `sphere`, `pneumatic/fast`, `pneumatic/detailed` on every preset.
- [ ] If `fast` misses the Task 0 budget, in this order: airborne cull, preallocated buffers,
  fewer rays (never below the count at which the contact length error exceeds 10 %),
  window slicing without copies. Record the final `N` and µs per step.
- [ ] `detailed` must be within 2–3× of real time headless; record it.
- [ ] Convergence test on `flat` and `single_edge`: bar/saddle RMS, fork/shock travel use,
  traverse time and rim-strike count of `fast` within tolerances measured against
  `detailed` and recorded in the docstring. Mark it slow if it exceeds 60 s.

**Verification:** numbers recorded in RIDE.md §10 and in "Measured baseline" below.

---

### Task 11: Model comparison report

**Files:**
- Create: `tools/compare_tyre_models.py`
- Create: `tests/test_compare_tyre_models.py` (smoke test on `flat`, short length)

**Steps:**

- [ ] Runs every preset on `sphere` and `pneumatic/fast` (and `detailed` with `--detailed`),
  same seed and speed, writing `output/tyre_compare/summary.json` and `table.md`: bar and
  saddle acceleration RMS and peaks (100 Hz low-pass, per D12 of the rough-road plan), fork
  and shock travel used, traverse time, time in wheelspin and lock, rim strikes, mean tyre
  loss, outcome.
- [ ] One-paragraph reading of the differences at the top of `table.md` (what got softer,
  where the rim was struck), generated from the numbers, no free text.

**Verification:** the smoke test; the full report attached to the review for D17.

---

### Task 12: Calibration from the user's measurements

Blocked on the user's numbers (Appendix A). Until then the literature defaults stand.

**Steps:**

- [ ] Fit `carcass_share` and the width law per wheel to the two measured points; keep the
  literature tests passing, or record which one moved and why.
- [ ] Update `TyreSpecs` defaults, RIDE.md §11 (measured, no longer authored) and the
  comparison report.

---

### Task 13: Switch the default (gated on D17)

Only after the user has read the Task 11 report and said yes.

**Steps:**

- [ ] `TyreConfig` default → `pneumatic/fast`; CLI default likewise.
- [ ] Regenerate `baseline_bike_ride.xml` and `baseline_bike_ride_lumped.xml` in this
  commit; `sphere`-specific tests (dropout bridging, `Crr·N` cap) pin `tyre_model="sphere"`
  explicitly.
- [ ] RIDE.md, README: the default changes; `sphere` documented as the legacy model.

---

## Risks

| Risk | Mitigation |
|---|---|
| Python cost per step breaks the `fast` budget | Task 0 sets the budget first; Task 10's ordered fallbacks; the airborne cull removes all cost in flight. |
| Ray–profile occlusion on steep descents | Visible-branch rule in Task 4, checked against the bisection oracle. |
| Stiff rim contact destabilises explicit integration | Rim stiffness cap and the `ω·dt` test (Task 5). |
| η cannot meet both Crr and the vertical damping ratio | Contract fixes the order: damping band first (sets η), then Crr; a shortfall > 15 % is closed by the tread-loss term, a damping ratio below 2 % by a small viscous term. Both fallbacks are written into RIDE.md §3.1 in advance and reported to the user if used. |
| Discretised brush unstable at high speed | Semi-Lagrangian advection; a forced Courant-2 test. |
| Wheel spin taken relative to a pitching carrier | Slip uses absolute ω from `mj_objectVelocity` (Task 7), asserted on a pitching test case. |
| Patch truncated at the edge of the ray coverage | ±75° coverage; coverage events; no shipped preset may raise one (Task 4). |
| Step-count constants assume 0.5 ms (equilibrium relax cycle, dropout window) | Equilibrium takes a time-based cycle; the dropout window is unused by `pneumatic`. |
| Brake taper prevents ω = 0 lock | Documented behaviour (Task 1, §6); lock is reported by slip, not by ω. |
| Literature tyres are not exactly ours (29×2.3 knobby, 25 mm rim) | ±15 % tolerances; Task 12 calibrates on the real bike. Slip and surface values are estimates and are labelled authored. |

---

## Appendix A — Measurements for the user (Task 12)

Per wheel, on a flat hard floor, tyre warm-ish, same rim/tyre as ridden:

1. Set pressure with a digital gauge to **1.3 bar**. Lift the wheel off the ground (bike on
   a stand or a helper lifts that end) and measure the height from floor to the **centre of
   the axle** (or to the rim's lower edge) — call it `H0`.
2. Put the wheel down, rider seated in normal position, both wheels on floor; if possible
   one bathroom scale under the measured wheel (a spacer of equal height under the other).
   Read the scale load `F` and the loaded height `H1`. Deflection `δ = H0 − H1`.
3. Repeat 1–2 at **1.8 bar**.
4. Send: wheel, pressures, `F`, `H0`, `H1`, and rim internal width if known.

Four numbers per wheel give `k(p)` at two pressures, which fixes the pressure and carcass
terms. A millimetre ruler is enough: expected `δ` is 7–10 mm.

---

## Appendix B — Sources

1. Dressel & Sadauckas, "Characterization and Modelling of Various Sized Mountain Bike
   Tires and the Effects of Tire Tread Knobs and Inflation Pressure", *Applied Sciences*
   10(9):3156, 2020 — Figs 8, 16, 17. https://www.mdpi.com/2076-3417/10/9/3156
2. Sadauckas et al., *Vehicle System Dynamics*, 2024 — Tables 2–3, Fig. 9.
   https://www.tandfonline.com/doi/full/10.1080/00423114.2024.2398001
3. bicyclerollingresistance.com — Schwalbe Hans Dampf TrailStar 2017 and PaceStar
   comparison; load test; test protocol.
   https://www.bicyclerollingresistance.com/mtb-reviews/schwalbe-hans-dampf-trailstar-2017 ·
   https://www.bicyclerollingresistance.com/specials/crr-load-test ·
   https://www.bicyclerollingresistance.com/the-test
4. bicyclerollingresistance.com — Hans Dampf 29×2.35 section measurements (same review as 3).
5. ENDURO Mountainbike Magazine, Schwalbe tyre lab test 2025 (1.5 bar, 50 kg, 20 km/h).
   https://enduro-mtb.com/en/schwalbe-mountain-bike-tires-review/
6. ENDURO Mountainbike Magazine, tyre insert test (Schwalbe edge-drop data).
   https://enduro-mtb.com/en/best-tire-insert/
7. O. Maier, dissertation, KIT (bicycle tyre longitudinal behaviour).
   https://www.hs-pforzheim.de/fileadmin/user_upload/uploads_redakteur_technik/10_Forschung/Diss_O_Maier_978-3-7315-0778-9.pdf
8. Burckhardt tyre-road friction parameters (car), as cited in
   https://www.researchgate.net/figure/Burckhardt-tyre-model-parameters-for-different-friction-coefficients_tbl1_270684635
9. TU Delft bicycle tyre vertical testing. http://bicycle.tudelft.nl/tiretestingvertical/

---

## Measured baseline

### Task 0 — `sphere`, 2026-09-26

`uv run python -m tools.bench_ride --json output/bench/bench_sphere_seated.json` on an Apple
M4 Max (MuJoCo 3.12.0, Python 3.12.13), seated rider, every preset from equilibrium to the
end of the track. Test suite before this plan: **425 passed in 51 s**.

| track | steps | RTF | µs/step | suspension | rider | rolling res. | cruise | brakes | virt. rider | crash | contact query | mj_step | overhead |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| enduro_aggressive | 34338 | 5.30× | 94.4 | 4.6 | 2.3 | 1.1 | 0.8 | 0.9 | 0.8 | 0.3 | 8.8 | 72.4 | 2.3 |
| flat | 33474 | 4.53× | 110.4 | 4.7 | 2.3 | 1.2 | 1.0 | 0.9 | 0.8 | 0.3 | 9.8 | 87.1 | 2.5 |
| road_broken | 30012 | 5.64× | 88.6 | 4.6 | 2.3 | 1.1 | 1.0 | 0.9 | 0.8 | 0.3 | 9.4 | 66.1 | 2.3 |
| road_smooth | 30018 | 5.40× | 92.6 | 4.6 | 2.4 | 1.1 | 1.0 | 0.9 | 0.8 | 0.3 | 9.8 | 69.5 | 2.3 |
| road_worn | 30021 | 5.47× | 91.3 | 4.6 | 2.4 | 1.1 | 1.0 | 0.9 | 0.8 | 0.3 | 9.5 | 68.5 | 2.3 |
| single_edge | 12757 | 4.63× | 108.0 | 4.6 | 2.3 | 1.1 | 0.9 | 0.9 | 0.8 | 0.3 | 9.3 | 85.6 | 2.4 |
| washboard_only | 18489 | 4.90× | 102.0 | 4.6 | 2.3 | 1.1 | 1.0 | 0.9 | 0.8 | 0.3 | 9.4 | 79.4 | 2.3 |

**Tyre budget for `fast`: 240 µs per step for both wheels** — 500 µs × (1 − 30 %) minus the
slowest track's 110 µs. That is about 120 µs per wheel. Removing the wheel collision rows
from `mj_step` in `pneumatic` should add headroom that the budget does not count on.
