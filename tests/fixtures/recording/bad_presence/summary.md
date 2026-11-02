# Run recording

Run UUID: `00000000-0000-0000-0000-000000000001`

Started (UTC): `2026-10-12 02:00:00+02:00`. Backend: `native`. Mode: `ride`.

## Invocation and provenance

Evidence: `runs.metadata_json`; per-episode setup identity is in `episodes.summary_json`.

```json
{
  "provenance": {
    "author": "tests/reference/_recording_support.py",
    "kind": "synthetic",
    "physical_source": "none (hand-authored trees)",
    "purpose": "deterministic capture and catalog regression tests"
  }
}
```

## Interpretation and limits

This is saved numerical evidence, not a claim that the model is calibrated. Recording retains the launch in RAM and publishes at finalization; crashes, SIGKILL and out-of-memory termination have no durability guarantee. NULL, absence and nonfinite values are separate states. No cause is inferred from temporal coincidence.

A forwarded endpoint measurement is not an instantaneous peak at every interval. The extrema below exclude forwarded energy rows; cumulative fields are reported as final values and are not summed. Missing/nonfinite coverage is reported explicitly.

## Episode 0

Native generation: `0`. Physical prefix: `{"first_interval":0,"first_time_s":0.0,"last_interval":1,"last_time_s":0.002,"rows":2}`.

Completed control transitions: `0`. Checked research replay eligible: `False` (contract `None`).

### Outcome, validity and first failure

Evidence: `episodes.summary_json`, selected by `episode_id`.

```json
{
  "execution_changed_during_run": null,
  "failure": null,
  "first_failure": null,
  "model_status": null,
  "outcome": null,
  "source_changed_during_run": null,
  "tail_failure": null
}
```

### Important events

Source events keep their recorded classification. `derived_transition` events describe retained channel changes under a named rule; they are not controller explanations.

Clock semantics: `received_wall_s` is the monotonic receipt timestamp (a wall clock, not simulation time); `interval_time_s` is the simulation start time of the interval the input was processed on, and is null when no interval carries that `processed_step`.

| Event | Kind | Received wall (s) | Sim time (s) | Processed step | Applied step | Recorded detail |
| --- | --- | --- | --- | --- | --- | --- |
| - | No source events recorded | - | - | - | - | - |

### Scalar coverage and extrema

| Field | Unit / phase | Finite | Missing | Nonfinite | Minimum / final | Maximum |
| --- | --- | --- | --- | --- | --- | --- |
| `channel_control.crank_target_rate_rad_s` | rad/s / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_control.human_torque_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_control.motor_limit_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_control.motor_torque_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.assist_gain` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.assist_sensor_nm` | N·m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.battery_energy_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.cadence_rpm` | rpm / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_dissipation_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_energy_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_extension_m` | m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_extension_rate_mps` | m/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.chain_tension_n` | N / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_clutch_dissipation_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_clutch_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_phase_rad` | rad / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_rad_s` | rad/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_target_phase_rad` | rad / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_target_rate_rad_s` | rad/s / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.drive_shaft_rad_s` | rad/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.electrical_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.freehub_deflection_rad` | rad / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.freehub_dissipation_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.freehub_energy_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.freehub_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.freehub_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.front_brake_demand` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.front_brake_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.front_brake_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.gear_ratio` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_active_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_command_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_joint_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_sensor_nm` | N·m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_setpoint_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.human_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_freewheel_dissipation_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_freewheel_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_limit_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_limited_request_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_request_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_setpoint_nm` | N·m / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_shaft_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.rear_brake_demand` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.rear_brake_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.rear_brake_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.required_cadence_rpm` | rpm / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.rollback_brake_demand` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.shift_constraint_work_cumulative_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_drive.shift_interval_constraint_work_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.shift_parameter_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_drive.shift_time_s` | s / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.shift_torque_factor` | 1 / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_boundary_m` | m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_constraint_defect_m` | m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_gap_m` | m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_interval_work_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_phi_m` | m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_reaction_error_n` | N / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_tension_n` | N / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_energy.active_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.constraint_absolute_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.constraint_signed_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.constraint_work_ratio` | 1 / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.electrical_residual_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.electrical_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.energy_scale_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.external_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.loss_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.loss_step_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_energy.mechanical_energy_j` | J / interval_end | 0 | 0 | 0 | unavailable / null | unavailable / null |
| `channel_energy.motor_positive_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.motor_signed_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.muscle_positive_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.muscle_signed_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.residual_j` | J / interval_end | 0 | 0 | 0 | unavailable / null | unavailable / null |
| `channel_energy.solver_constraint_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.source_positive_work_j` | J / episode | 0 | 2 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_energy.source_time_s` | s / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_mass.gravitational_energy_j` | J / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_mass.kinetic_energy_j` | J / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_mass.mass_kg` | kg / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_active_positive_power_limit_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_lean_limit_rad` | rad / control_period | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_passive_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_passive_work_step_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_positive_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.rider_positive_work_step_j` | J / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_rider.weld_crank_torque_nm` | N·m / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_sensors.encoder_crank_rad_s` | rad/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_sensors.encoder_front_wheel_rad_s` | rad/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_sensors.encoder_rear_wheel_rad_s` | rad/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_sensors.human_torque_nm` | N·m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_sensors.motor_torque_nm` | N·m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.fork_travel_m` | m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.fork_velocity_mps` | m/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.linkage_closure_max_m` | m / interval_end | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.shock_solver_limit_power_w` | W / interval | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.shock_stroke_m` | m / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `channel_suspension.shock_velocity_mps` | m/s / interval_start | 0 | 2 | 0 | unavailable / null | unavailable / null |
| `physics.dt_s` | s / interval | 2 | 0 | 0 | 0.001 | 0.001 |
| `physics.interval_end_s` | s / interval_end | 2 | 0 | 0 | 0.001 | 0.002 |
| `physics.time_s` | s / interval_start | 2 | 0 | 0 | 0.0 | 0.001 |

Reproduce each extrema row as `SELECT min(FIELD), max(FIELD) FROM TABLE WHERE episode_id=0 AND isfinite(FIELD)`; for `channel_energy` fields marked `forwarded` in `field_catalog`, also require `measured IS TRUE`. For phase `episode`, select the last `interval_id` instead of aggregating cumulative values.

### Recorded component power integrals

Rule `left_rectangle_recorded_power.v1`: sum of recorded `power_w * dt_s`. This is a derived finite-subset integral when any power is missing/nonfinite; it does not replace the engine's recorded cumulative accounting.

| Component | Intervals | Finite powers | Derived work (J) |
| --- | --- | --- | --- |
| - | 0 | 0 | unavailable |

Evidence query: `forces JOIN physics USING(episode_id,interval_id)`, `episode_id=0`, grouped by `component`.

### Episode metrics and source identity

```json
{
  "episode_metrics": null,
  "execution": null,
  "model_source_sha256": null,
  "research": null,
  "resolved_config": null
}
```

## Field and coordinate coverage

The complete finite field inventory is `field_catalog`; model-address identities are `coordinate_catalog`. `_present` columns describe existence independently of nullable values. Parent `_valid` flags distinguish an explicitly null parent from an empty or absent parent. Structured children carry their own optionality flags.

Diagnostics not represented by this catalog were not recorded and cannot be reconstructed as source facts. Observation delivery and acquisition timestamps are separate; transitions reference observations rather than duplicating them.
