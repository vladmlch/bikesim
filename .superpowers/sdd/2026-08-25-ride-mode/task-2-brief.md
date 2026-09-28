### Task 2: Coil shock, suspension forces, static equilibrium

Give the model real suspension forces and a defined starting state. Nothing in this task drives the bike forward.

**Files:**
- Create: `src/bike_sim/physics/coil_shock.py`
- Create: `src/bike_sim/sim/ride/__init__.py`
- Create: `src/bike_sim/sim/ride/forces.py`
- Create: `src/bike_sim/sim/equilibrium.py`
- Create: `tests/test_coil_shock.py`
- Create: `tests/test_ride_equilibrium.py`

**Steps:**

- [ ] `coil_shock.py`: a pure calculator in the style of `physics/air_spring.py` — no MuJoCo, no `data`.
  - `CoilShockSpecs` dataclass: `rate_n_m: float = 114600.0`, `preload_mm: float = 0.0`, `stroke_mm: float = 65.0`, `bumper_length_mm: float = 10.0`, `bumper_peak_n: float = 7000.0`.
  - `CoilShock.compute_spring_force(stroke_mm)` → `rate_n_m * (stroke_mm + preload_mm) / 1000.0`. A coil is linear; this is correct, not a simplification. All progression comes from the linkage, which MuJoCo applies through the constraint Jacobian.
  - `CoilShock.compute_bumper_force(stroke_mm)` → let `engage_mm = specs.stroke_mm - specs.bumper_length_mm` (55.0 mm at the defaults) and `excess_mm = max(0, stroke_mm - engage_mm)`; the force is `bumper_peak_n * (excess_mm / bumper_length_mm) ** 2`. Quadratic, so both force and slope are continuous at engagement, reaching `bumper_peak_n` exactly at full stroke.
  - `CoilShock.compute_axial_force(stroke_mm)` → spring + bumper.
  - Default `preload_mm = 0.0` deliberately: the shipped defaults must keep reproducing the documented behaviour.
- [ ] `forces.py`: `SuspensionForceApplier`. It **calls** the existing `SuspensionController` for the fork and `CoilShock` for the shock, and is the only thing that writes to `data`.
  - Constructor takes the model plus the calculators; it resolves and caches the `qposadr`/`dofadr` of `fork_travel` and `shock_stroke` once.
  - `apply(model, data)` reads `qpos`/`qvel` at those addresses, converts to mm and mm/s, computes fork force (air spring + Charger 3 damping) and shock force (coil + bumper + Super Deluxe damping with HBO), and writes `data.qfrc_applied[dof] = -f_total` for each.
  - **Sign convention:** both joints have `range` starting at 0 with increasing value meaning compression, so a resisting force is a negative generalized force. Assert this holds by checking the joint ranges at construction.
  - Do not multiply by any leverage ratio. The generalized force on `shock_stroke` *is* the axial shock force; MuJoCo applies the leverage ratio through the constraint Jacobian. Multiplying by an analytic ratio would double-count it.
  - Expose the last computed component forces (spring, damper, bumper, per end) as attributes so telemetry can read them without recomputing.
- [ ] `equilibrium.py`: `solve_static_equilibrium(model, data, applier, max_steps=..., tol=...)`.
  - Place the bike above the road, then relax: step the model with the suspension forces applied, zeroing `data.qvel` each iteration (damped relaxation), until `max(abs(data.qacc))` falls below the tolerance or the cap is reached.
  - On success zero the velocities, run `mj_forward`, and return a dict with the converged fork travel (mm), shock stroke (mm), rear wheel travel (mm), root height, pitch, and the iteration count.
  - Raise a clear error if it does not converge; never silently return an unconverged state.
- [ ] `tests/test_coil_shock.py`: linearity of the spring term; preload offsets the whole curve; the bumper is exactly zero below engagement, continuous at engagement, and equals `bumper_peak_n` at full stroke; total force is monotonically increasing in stroke.
- [ ] `tests/test_ride_equilibrium.py`: on the `flat` preset the solve converges; the resulting rear sag is 25.3 % ± 1.5 pp and the front sag 42.0 % ± 2 pp, matching the spec's §9 table for the shipped defaults; the bike is at rest afterwards (`max|qvel|` ≈ 0); running the solve twice gives the same answer.

**Verification:** `uv run python -m pytest -q`.

---

