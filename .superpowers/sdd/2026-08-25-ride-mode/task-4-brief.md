### Task 4: Full track, run termination, interactive viewer

**Files:**
- Create: `src/bike_sim/sim/ride/input.py`
- Modify: `src/bike_sim/sim/ride_sim.py`
- Modify: `src/bike_sim/sim/camera.py`
- Modify: `src/bike_sim/sim/hud.py` (or add a ride HUD alongside it — do not disturb the playground HUD)
- Create: `tests/test_ride_track.py`

**Steps:**

- [ ] **Carried from Task 3 — the pitch stabilizer is unexercised until now.** On `single_edge` the bike never has both wheels airborne, so the virtual rider never fired and its gains have never run in anger. They were derived from the measured plant: ω_n 3.60 rad/s, ζ 0.54, mildly underdamped. The kicker is the first thing that tests them. If the bike rings in the air after the lip, `kd` is the knob — 278 is critical damping. Report what the rider actually did over the kicker: peak moment against the ±80 N·m ceiling, the angular impulse it injected, and whether the pitch settled before touchdown.
- [ ] **Carried from Task 3 — pin the pitch sign convention.** `ride_sim.py` asserts in a comment that positive `pitch_rad` is nose-down, and nothing pins it. Task 3's stabilizer and crash detector are both sign-symmetric so it never bit there, but this task's HUD and the next task's telemetry both report pitch as a signed quantity. Add a test that puts the bike in a known attitude and asserts the sign, and make the HUD label say which way is which.
- [ ] **Carried from Task 3 — the contact filter meets its first speed sweep here.** MuJoCo's sphere–heightfield collision drops the contact row for a loaded wheel on a speed-dependent fraction of steps (measured on flat ground: 2.0 / 14.7 / 27.2 / 43.2 % at 15 / 25 / 35 / 45 km/h). Task 3's `TerrainContactQuery` masks it by holding the last load for up to 10 steps, a **fixed** window against a speed-dependent artifact, characterized only at 25 km/h. Before trusting a run above 25 km/h, measure the longest consecutive flicker at the top of the range and confirm it stays well inside the window — and that the window stays well below the shortest genuine airborne phase on the track. If either margin fails, report it rather than widening the window.
- [ ] Run termination: a run ends when the bike passes the end of the track, when the crash detector trips, or when a wall-clock/step-count safety cap is hit. The termination reason is part of the result, never implicit.
- [ ] `input.py`: the ride key map. **Every damper and air-spring key keeps the same binding as the playground** (`H`/`J` fork HSC, `K`/`L` fork LSC, `Y`/`U` fork rebound, `3`–`0` shock HSC/LSC/rebound/HBO, `[`/`]` tokens, `-`/`=` pressure), as do `C` camera, `1`/`2` views, `T` telemetry, `R` reset, `B` rider, `G` markers, `P` preset, `/` help. New: `W`/`S` adjust the cruise target by ±1 km/h, `Space` toggles braking, `,`/`.` adjust brake strength. Braking is a toggle because MuJoCo's passive viewer delivers key presses, not key state.
- [ ] Camera: in ride mode track X exactly (no smoothing — at 6.94 m/s the existing `alpha=0.08` lags about 1.4 m) and keep the smoothing on Z so the view does not jolt on every bump. Do not change the behaviour the playground gets.
- [ ] Ride HUD line: track X and the nearest obstacle, target and actual speed, fork and shock travel in mm and % of travel, shaft velocities, brake state, per-wheel contact, pitch, wheel power.
- [ ] `tests/test_ride_track.py`: a headless `enduro_aggressive` run at 25 km/h completes with termination reason "end of track" and no crash; the bike's trajectory over the kicker leaves the ground after the lip and touches down on the landing slope (between 60.9 m and 65.9 m), which is the spec's stated 20.9–30.6 km/h working band exercised at its midpoint; suspension travel never exceeds the joint limits.

**Verification:** `uv run python -m pytest -q`.

---

