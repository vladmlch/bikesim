# Task 2 Report: Coil shock, suspension forces, static equilibrium

**Status: DONE_WITH_CONCERNS** — everything in the brief is implemented and the suite is
green, but the solved rear sag is **22.7 %**, not the **25.3 % ± 1.5 pp** the brief names.
The deviation is fully diagnosed, reproduced by two independent methods, and reconciled
with the repository's own analytic tools below. The brief's rear-sag tolerance is not
reachable by a correct implementation; the test asserts the solved value with the
reconciliation documented in the test module rather than a widened tolerance around the
table figure.

---

## What I implemented

### `src/bike_sim/physics/coil_shock.py` (new)

Pure calculator, no MuJoCo, no `data`, in the style of `physics/air_spring.py`.

- `CoilShockSpecs`: `rate_n_m=114600.0`, `preload_mm=0.0`, `stroke_mm=65.0`,
  `bumper_length_mm=10.0`, `bumper_peak_n=7000.0`, plus a `bumper_engage_mm` property
  (55.0 mm at the defaults) so the engagement point has one definition.
- `CoilShock.compute_spring_force(stroke_mm)` = `rate_n_m * (stroke + preload) / 1000`.
  Linear, deliberately: a steel coil has a constant rate and all progression comes from
  the linkage, which MuJoCo applies through the constraint Jacobian.
- `CoilShock.compute_bumper_force(stroke_mm)` =
  `bumper_peak_n * (max(0, stroke - engage) / bumper_length) ** 2`.
- `CoilShock.compute_axial_force(stroke_mm)` = spring + bumper.

### `src/bike_sim/sim/ride/` (new package: `__init__.py`, `forces.py`)

`SuspensionForceApplier` — the only writer to `data` in the suspension path.

- Constructor takes `(model, controller, coil_shock)` and caches `qposadr`/`dofadr` for
  `fork_travel` and `shock_stroke` once, through `_resolve_compression_joint`, which
  raises `ValueError` unless the joint exists, is limited, and has a range starting at 0
  (the sign-convention check the brief asks for).
- `apply(model, data)` reads `qpos`/`qvel` at those addresses, converts to mm and mm/s,
  and writes `data.qfrc_applied[dof] = -f_total` for both ends.
  - Fork: `SuspensionController.compute_fork_force` (air spring + Charger 3 damping).
  - Shock: `CoilShock` coil + bumper + `suspension_system.shock_damper` (Super Deluxe with
    HBO). `SuspensionController.compute_shock_force` is **not** used, so the old linear
    `stroke * shock_stiffness` term is not applied on top of the coil.
  - **No leverage ratio anywhere.** A dedicated test asserts the shock generalized force
    equals `CoilShock.compute_axial_force(stroke)` exactly, which would fail by the ~3.0×
    leverage ratio if the trap were fallen into.
- Exposes the last computed components (`fork_spring_n`, `fork_damper_n`, `fork_total_n`,
  `shock_spring_n`, `shock_bumper_n`, `shock_damper_n`, `shock_total_n`) and the joint
  addresses, so telemetry and the equilibrium solve read them instead of recomputing.
- `model` is unused in `apply` (addresses are cached); it is kept so every ride-mode force
  writer in tasks 3–4 shares one `(model, data)` signature. This is stated in the docstring.

### `src/bike_sim/sim/equilibrium.py` (new)

`solve_static_equilibrium(model, data, applier, solver, start_x_m=2.0, max_steps=40000,
tol=0.05)`.

- Resets `data`, places the chassis root at `start_x_m` (default 2.0 m: the heightfield's
  near edge is at world x = 0 and the rear contact patch sits ~0.45 m behind the root, so
  x = 0 would drop the rear wheel off the field) and 5 mm above the road, so the solve
  starts with no penetration.
- Damped relaxation: `RELAX_STEPS_PER_CYCLE = 40` steps (20 ms, short against the ~3 Hz
  suspension modes) with the suspension forces applied, then `qvel[:] = 0`, re-apply,
  `mj_forward`, and test `max|qacc| <= tol`. Because the velocities are zero at the
  measurement, the dampers contribute nothing and the residual is a pure static imbalance.
- Returns `fork_travel_mm`, `shock_stroke_mm`, `rear_travel_mm` (via
  `solver.solve_state_from_shock_stroke`), `root_z_m`, `pitch_rad`, `steps`,
  `residual_qacc`. Raises `RuntimeError` with residual, root height and both shaft
  positions if the cap is hit — never returns an unconverged state.
- `solver` is a required argument rather than a defaulted one: reporting rear *wheel*
  travel needs the analytic linkage, and defaulting it to `BikeSpecs()` would silently
  mismatch a model built from other specs.

Also added the new symbols to `physics/__init__.py` and `sim/__init__.py`, which re-export
every module in their packages.

---

## Results

Solved on the `flat` preset with `include_rider=True` and the shipped defaults
(`fork_initial_psi=85.2`, 2 tokens, `shock_stiffness=114600`, coil preload 0):

| Quantity | Solved | docs/RIDE.md §9 |
|---|---|---|
| Fork travel | 73.14 mm | 75.6 mm |
| **Front sag** | **40.64 %** | **42.0 %** |
| Shock stroke | 12.82 mm | 14.35 mm |
| Rear wheel travel | 40.89 mm | 45.5 mm |
| **Rear sag** | **22.71 %** | **25.3 %** |
| Root height | −49.90 mm | — |
| Pitch | +1.134° (nose-down) | — |
| Steps to converge | 9400 (1.24 s wall) | — |
| Residual `max|qacc|` | 0.0495 | — |

Front sag is inside the brief's ±2 pp band. Rear sag is 2.6 pp below the table, outside
the ±1.5 pp band.

### Why the rear reads 22.7 % and not 25.3 %

§9's table is an analytic prediction; the multibody equilibrium is not. Two effects the
table does not model, both measured at the solved state:

1. **Unsprung mass bypasses the spring.** The table routes the whole axle load through the
   spring. In the model the mass below each spring (fork lowers + front wheel; chainstay,
   seatstay + rear wheel) is carried straight by the contact patch: ~34 N at the front and
   ~42 N at the rear.
2. **Load transfer at sag.** The table evaluates at the undeflected CoG. Sagging 73 mm
   front / 41 mm rear pitches the bike 1.13° nose-down, moving ~20 N forward. Measured
   contact loads at equilibrium are **497.9 N front (48.6 %) / 525.0 N rear (51.3 %)**,
   against the table's 478.2 / 546.0 N. Their sum is 1023.7 N = the system weight exactly,
   so the equilibrium is genuinely supported by the road.

The remaining **sprung** loads are 464.5 N front and 483.0 N rear. Fed back into the
repository's own analytic calculators:

- `ForkAirSpring` at 85.2 psi / 2 tokens balances 464.5 N × sin(64°) at **73.2 mm** —
  the solved fork travel;
- the coil at 114 600 N/m through the analytic leverage ratio balances 483.0 N at
  **40.9 mm** of wheel travel — the solved rear travel.

So the force path agrees with the analytic model to the digit. Only the table's load
assumption differs. This is a *reporting* gap in §9, on top of the pre-existing 35/65
calibration mismatch (README Known Limitation #1) that already puts the figures at
42.0/25.3 instead of 30/30 — a different and unrelated cause.

### Independent confirmation

The relaxation is cross-validated by a free dynamic settle: same placement, real damper
forces, no velocity zeroing, 3 s of simulation. It lands at fork 73.26 mm and stroke
12.83 mm, i.e. the same fixed point to within 0.12 mm / 0.01 mm (held to 12 s of
simulation during investigation). That is a test in the suite, not just a one-off check.

---

## What I tested

`uv run python -m pytest -q` → **169 passed** (151 before this task, +7 coil shock,
+11 ride equilibrium), 8.6 s wall.

`tests/test_coil_shock.py` (7 tests): spring linearity against the declared rate and
proportionality; preload offsetting the whole curve without changing the rate; bumper
exactly `0.0` below engagement; bumper quadratic and slope-continuous at engagement (the
total curve's slope is the coil rate on both sides to 1e-4 relative); bumper exactly
`bumper_peak_n` at full stroke and a quarter of it half way in; total force strictly
increasing over 0–65 mm; defaults equal to the shipped `BikeSpecs` spring settings.

`tests/test_ride_equilibrium.py` (11 tests): convergence within the cap with both joints
in range; `RuntimeError` when the step cap is hit; front sag (solved value ±1 pp, and §9's
42.0 % ±2 pp); rear sag (solved value ±1 pp, and strictly below §9's 25.3 %, with the
reconciliation in the module header); `max|qvel| == 0` after the solve and the bike staying
put over 0.1 s of stepping; two solves returning bit-identical dicts; the free-settle
cross-check; the sign convention and no-leverage-ratio assertion at both ends; damper sign
opposing shaft motion for both directions; and the constructor refusing a joint whose range
does not start at zero.

No existing test was modified; no MJCF builder was touched; the four golden baselines are
untouched.

---

## Files changed

New:
- `src/bike_sim/physics/coil_shock.py`
- `src/bike_sim/sim/ride/__init__.py`
- `src/bike_sim/sim/ride/forces.py`
- `src/bike_sim/sim/equilibrium.py`
- `tests/test_coil_shock.py`
- `tests/test_ride_equilibrium.py`

Modified (re-exports only):
- `src/bike_sim/physics/__init__.py`
- `src/bike_sim/sim/__init__.py`

---

## Self-review findings (fixed before reporting)

- Dropped `fork_travel_mm`/`shock_stroke_mm`/`fork_velocity_mps`/`shock_velocity_mps` from
  the applier's cached state: the brief asks for component *forces*, and existing telemetry
  already reads shaft state from `data`. YAGNI.
- Replaced a nonsense index expression (`fork_qposadr - fork_qposadr` as a stand-in for
  root_x) in the settle test with a proper `_qposadr` helper.
- The `bumper_is_continuous` test originally asserted a one-sided slope of ~0 with a
  tolerance the quadratic could not meet; reframed as "the *total* curve's slope is the
  coil rate on both sides", which is the claim that actually matters and discriminates a
  linear bumper by three orders of magnitude.
- `test_force_path_resists_compression` originally ran a full equilibrium solve and a
  second full model compile for what is a unit-level check; both removed (module runtime
  12 s → 4.9 s) by setting `qpos` directly and using a 6-line synthetic MJCF for the
  bad-range case.
- Made every test that needs a particular `data` state produce it itself, so no test
  depends on execution order via the module-scoped fixture.
- Checked style against the repo: ASCII `--` rather than em dashes in source (matching
  `terrain/obstacles.py`), Google-style `Args:`/`Returns:`/`Raises:`, title-line module
  docstrings, `Optional[...] = None` specs constructor. Largest new source file: 132 lines.

## Concerns

1. **The rear sag figure (main concern).** §9's table and this brief expect 25.3 % ± 1.5 pp;
   the model settles at 22.71 %. I did not widen the tolerance around 25.3 %: the test pins
   the solved value with the reconciliation above written into the test module, and asserts
   it stays strictly below the table. Someone should decide whether §9 gains a "solved
   equilibrium" column (recommended — the front figure has the same 1.3 pp offset and only
   passes the ±2 pp band by luck) or whether the sag convention should be restated in terms
   of sprung load. This is a documentation decision, not a code change, so I left both
   `docs/RIDE.md` and the README alone.
2. **`start_x_m` default of 2.0 m** is my choice, not the brief's; the heightfield's near
   edge at world x = 0 makes some positive offset mandatory. Task 3 places the bike at the
   track start after the solve, so it can override it.
3. **`tol=0.05` on `max|qacc|`** (mixed m/s² and rad/s²) reaches a residual floor around
   0.002 from contact-solver noise; 0.05 is met in 9400 steps and tightening to 0.002 costs
   80 000 steps for 0.1 mm of extra fork travel. The tolerance was chosen from that measured
   trade-off, not to make a test pass.
