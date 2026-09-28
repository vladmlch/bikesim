### Task 3: Cruise, brakes, rolling resistance, virtual rider

Make the bike ride. The target of this task is `single_edge`, not the full track.

**Files:**
- Create: `src/bike_sim/sim/ride/cruise.py`
- Create: `src/bike_sim/sim/ride/braking.py`
- Create: `src/bike_sim/sim/ride/resistance.py`
- Create: `src/bike_sim/sim/ride/virtual_rider.py`
- Create: `src/bike_sim/sim/ride_sim.py`
- Create: `tests/test_ride_controllers.py`

**Steps:**

- [ ] `cruise.py`: `CruiseController` — PI on **chassis longitudinal velocity** (`qvel[root_x]`), never on wheel angular velocity. A wheel-speed loop commands torque into an airborne or slipping wheel, spins it up without bound, and lands violently. Output is a rear-wheel torque clamped to ±150 N·m. Integrator must be clamped against wind-up. Torque is gated to zero when the rear wheel has no ground contact.
- [ ] `braking.py`: `BrakeController` — takes a demand in [0, 1] per wheel and a torque ceiling of 200 N·m, and emits a torque **opposing the current wheel angular velocity**, tapering to zero as wheel speed approaches zero so a standing bike is not driven backwards.
- [ ] `resistance.py`: rolling resistance as a torque on each wheel proportional to that wheel's instantaneous normal contact load, `Crr = 0.015`, opposing rotation. Read the normal load from the contact forces (`mj_contactForce`), not from a static estimate — the load-dependence is the whole point, since a G-out doubles it. No aerodynamic term.
- [ ] `virtual_rider.py`:
  - `PitchStabilizer` — PD on `root_pitch` toward level, output clamped to ±80 N·m, **active only while both wheels are out of contact**. It writes to `qfrc_applied` on the pitch DOF. This moment has no reaction body and therefore injects angular momentum; the class must accumulate the angular impulse and the work it does so telemetry can report them as an explicit source.
  - `CrashDetector` — trips when `|pitch| > 60°` or when a handlebar geom contacts the terrain. Reports the trip position and cause.
- [ ] `ride_sim.py`: `RideSimulation` — the orchestrator.
  - Build the ride XML, compile it, fill `model.hfield_data` from `build_field_data(track)` **before** any viewer exists, resolve handles, solve the static equilibrium, and place the bike at the track start.
  - `step()` applies, in order: suspension forces, rolling resistance, cruise torque, brake torque, virtual rider; then `mj_step`.
  - Keep the class under 500 lines; it coordinates, it does not compute.
- [ ] Contact queries: a shared helper that reports, per wheel, whether its contact sphere touches the terrain and with what normal force. Cruise gating, rolling resistance and the virtual rider all need it — write it once.
- [ ] `tests/test_ride_controllers.py`: on `single_edge` at 25 km/h the bike reaches the edge, crosses it, and finishes the track without the crash detector tripping; mean speed over the flat run-up is within 5 % of target; brake torque opposes rotation for both signs of wheel speed and is zero at rest; the pitch stabilizer output never exceeds 80 N·m and is exactly zero whenever either wheel is in contact; rolling-resistance torque scales with normal load.

**Verification:** `uv run python -m pytest -q`. Report the wall-clock of the `single_edge` traverse.

---

