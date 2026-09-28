### Task 1: MJCF `ride` mode

Add the fourth mode to the model builders. Everything in this task is gated on `mode == "ride"`; nothing may alter the XML the other three modes emit.

**Files:**
- Create: `src/bike_sim/mujoco/terrain.py`
- Modify: `src/bike_sim/mujoco/builder.py`
- Modify: `src/bike_sim/mujoco/environment.py`
- Modify: `src/bike_sim/mujoco/frame.py`
- Modify: `src/bike_sim/mujoco/steering_fork.py`
- Modify: `src/bike_sim/mujoco/drivetrain.py`
- Modify: `src/bike_sim/mujoco/rear_linkage.py`
- Modify: `src/bike_sim/mujoco/actuators.py`
- Modify: `src/bike_sim/mujoco/sensors.py`
- Modify: `src/bike_sim/mujoco/exporter.py`
- Create: `tests/golden/baseline_bike_ride.xml`
- Modify: `tests/test_golden_baselines.py`
- Create: `tests/test_ride_model.py`

**Steps:**

- [ ] `builder.py`: accept `mode="ride"`. Use `timestep = "0.0005"` for ride (`0.001` for stand/playground, `0.002` otherwise). Pass `mode` to every sub-builder that needs it. Call the new terrain builder for ride only.
- [ ] `terrain.py`: `build_terrain(root, worldbody, ground_z_m, spec=FIELD)`. Emit into `<asset>` a `<hfield name="road" nrow=... ncol=... size=...>` using `HeightFieldSpec.nrow/ncol/size_attr` from `bike_sim.terrain.heightfield.FIELD`. Emit a worldbody `<geom name="terrain" type="hfield" hfield="road">` at `pos = (spec.geom_x_m(), 0, spec.geom_z_m(ground_z_m))` with `condim="3"`, `friction="1.2 0.005 0.0001"`, `solref="-130000 -800"`, and a material. **No elevation data goes into the XML** — the asset is declared with zeroed data and filled from Python later.
- [ ] `environment.py`: for `mode == "ride"`, emit the two lights, **no** `floor` plane at road level (a MuJoCo `plane` is an infinite half-space for collision and would bridge every pothole), **no** stand fixtures. Emit a single `catch_plane` at `pos z = FIELD.catch_plane_z_m(ground_z_m)` with a material and default contact parameters.
- [ ] `frame.py`: for `mode == "ride"`, add the three planar root joints to the `frame` body, in the order `root_x`, `root_z`, `root_pitch`, before any geometry. Also call `build_rider(frame, include_rider=include_rider)` for ride (rider is on by default in this mode).
- [ ] `steering_fork.py`: add `"ride"` to the mode tuple that locks `steer_joint`. Add a `site_handlebar` site at `stem_top` for ride only. On the front wheel, for ride only: set the tyre cylinder to `contype="0" conaffinity="0"` and add `geom_front_contact`, a sphere of radius 0.372 at the wheel origin with `mass="0"`, `condim="3"`, `friction="1.2 0.005 0.0001"`, `solref="-130000 -800"`, `contype="1"`, `conaffinity="1"`, and `rgba` alpha 0 so it does not render over the tyre.
- [ ] `drivetrain.py`: the same treatment for the rear wheel in `build_rear_wheel` — tyre cylinder non-colliding in ride, `geom_rear_contact` sphere of radius 0.352.
- [ ] `rear_linkage.py`: for ride only, add `solreflimit="0.01 1"` to the `fork_travel` and `shock_stroke` joints. (The fork joint lives in `steering_fork.py`; apply it there.) These limits are the final barrier, not the bottom-out mechanism. Confirm the `stand_clamp` weld stays gated on `mode == "playground"` so ride never gets it.
- [ ] `actuators.py`: for `mode == "ride"`, emit exactly three motors — `rear_drive` on `rear_wheel_spin` with `gear="1" ctrlrange="-150 150"`, `front_brake` on `front_wheel_spin` with `gear="1" ctrlrange="-200 200"`, `rear_brake` on `rear_wheel_spin` with `gear="1" ctrlrange="-200 200"`. Brake ranges are two-sided because the controller computes the sign; the actuator does not.
- [ ] `sensors.py`: accept `mode`. For ride only, append `<accelerometer name="sensor_bar_accel" site="site_handlebar"/>` and `<accelerometer name="sensor_saddle_accel" site="site_seatpost_top"/>`. MuJoCo accelerometers report proper acceleration (0 in free fall, 9.81 at rest), which is what the rider feels — do not finite-difference site positions instead.
- [ ] `exporter.py`: `export_playground_models` additionally writes `bike_ride.xml` from `mode="ride"` with `include_rider=True`, and returns it under the key `"ride"`.
- [ ] `tests/test_golden_baselines.py`: add a byte-for-byte baseline assertion for the ride XML, mirroring the three existing ones. Generate `tests/golden/baseline_bike_ride.xml` from the implementation once it is correct.
- [ ] `tests/test_ride_model.py`: compile `mode="ride"` in MuJoCo and assert — `nq == 12` and `nv == 12`; `neq == 2` and no equality constraint named `stand_clamp`; joints `root_x`, `root_z`, `root_pitch` exist with the right types; the wheel contact geoms are spheres of the right radii with `contype == 1` while the tyre cylinders have `contype == 0`; the hfield asset dimensions equal `FIELD.nrow`/`FIELD.ncol`; `model.opt.timestep == 0.0005`; the catch plane sits below `FIELD.geom_z_m(ground_z_m)`; both ride accelerometers resolve. Also assert the other three modes still compile.

**Verification:** `uv run python -m pytest -q` — all previously passing tests plus the new ones. The three pre-existing golden baselines must be untouched; if any fails, the gating is wrong.

---

