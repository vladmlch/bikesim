# Phase 3 drive reference ports — source review

Review date: 2026-10-21. No compilation, CMake configure, tests, applications,
compiler probes or containers were executed in this work. The entries below
describe implemented source, not passed gates. ADR-0003 authority has not
changed: the Python files remain until the required platform evidence exists.

## Task 3.19

The six files have one native function for each of their 60 Python test
functions. Parameter tables, analytic constants, iteration counts and source
tolerances are carried into the corresponding functions. The five scalar/
rider suites use direct contract or analytic assertions; the momentum suite
calls the native validation rig and checks the original measured-bound fields.

| Source stem | Functions | Native target | Port detail |
| --- | ---: | --- | --- |
| `motor_profile` | 8 | `reference.motor_profile` | Registry/provenance, all nine gain cases, domain rejection, detached ownership and permission gates. |
| `assist_controller` | 22 | `reference.assist_controller` | Lag, sensed effort, taper/cutoff, torque curve and shaft speed, profile overrides, copy isolation, CLI/TOML precedence and removed config keys. |
| `cadence_shifter` | 9 | `reference.cadence_shifter` | Gear direction/landing, smallest cog, cooldown, rollback, wheelspin and EMA. |
| `pedal_waveform` | 10 | `reference.pedal_waveform` | Full-cycle mean/extrema, all phase/friction/tilt cases, net lever torque, recovery preload and zero-load bounds. |
| `pedaling_policy` | 9 | `reference.pedaling_policy` | Removed reposition fields, 3,000 rocking steps, EMA/hysteresis, wheel-implied cadence, stopped-crank resumption. |
| `system_momentum_drive` | 2 | `reference.system_momentum_drive` | Active motor without rider work, passive zero work, impulse/angular/power/root-force/contact bounds; `slow`. |

These tests use `bt::require`, not `assert`, so Release keeps every check.
`drive_reference_support.hpp` obtains resolved assist data through the actual
typed config → runtime writer projection → owned decoder. In particular, the
profile-owned limit tests do not contain a second implementation of the
profile resolution rules.

### Deliberate boundary translations

- `motor_profile::test_profile_is_frozen_and_copies_the_gain_mapping` checks the
  published `const` registry and real detached controller ownership. Python's
  `FrozenInstanceError`, `TypeError`, mapping mutation and shared-object
  identity have no native object-model counterpart. They are not described as
  executed native exceptions.
- `motor_profile::test_pedelec_cap_is_a_permission_not_a_throttle` observes
  permission through `AssistController::step`, where native code owns the
  gate. The uncapped permission delivers the original 3.4 × 10 Nm request;
  native code exposes no standalone infinity-valued cap result. Gate inputs
  use the native rpm boundary, including the original 5/6 rpm exact gate.
- `motor_profile::test_pedelec_cap_rejects_invalid_inputs` ports all five
  numeric/domain cases. The sixth Python parameter, `braking=1`, is a
  Python-specific strict-bool marshalling check and is a documented drop.
- Motor-profile wrong-shape gain inputs are checked at the actual owned wire
  decoder; typed C++ records cannot represent missing members or a scalar in
  place of the two-element eMTB gain field.
- `pedaling_policy::test_reposition_knobs_and_mode_are_gone` checks the closed
  config and mode domains. Python keyword-call introspection has no native
  keyword-argument equivalent. `test_ride_control_has_no_reposition_field`
  additionally checks the emitted control document and rejects the removed
  field at the native decoder.
- `test_rider_program_rejects_the_removed_reposition_command` preserves
  rejection at `programs_from_documents`. The existing native diagnostic is
  `rider keyframe: unknown field 'crank_reposition'`; the source Python
  constructor uses `unknown or malformed rider keyframe`. Exact message
  equality is not claimed or silently introduced by this port.

All other Task 3.19 assertions are direct translations of the source
function's analytic/contract checks. No runtime-generated expected numbers,
dummy assertions or empty test bodies were substituted.

### Exact Task 3.19 function mapping

For each table, the source is `tests/reference/test_<stem>.py`, the
successor source is `native/tests/reference/test_<stem>.cpp`, and the CTest
is `reference.<stem>`. Each row describes source implementation only; none
is a passed CTest. A boundary translation names an assertion's replacement,
not an executed proof that C++ type checking rejects a Python call.

#### `motor_profile` → `reference.motor_profile`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_bosch_profile_values_are_the_recalled_unverified_set` | All recalled torque/power/lag/gate/cutoff values and unverified provenance. |
| `test_mode_gains` | All nine mode/effort/gain rows, including negative and zero effort. |
| `test_unknown_mode_is_rejected` | Unknown mode parsing and owned controller construction rejection. |
| `test_profile_is_frozen_and_copies_the_gain_mapping` | Detached values and controller ownership; Python assignment exceptions/identity translated as described above. |
| `test_profile_rejects_invalid_data` | Nine typed invalid profiles and three malformed gain maps through the actual wire decoder. |
| `test_pedelec_cap_is_a_permission_not_a_throttle` | All permission cases observed through native delivered torque, including exact gate and explicit caps. |
| `test_pedelec_cap_rejects_invalid_inputs` | Five numeric/domain cases; only the `braking=1` marshalling parameter is dropped. |
| `test_gain_rejects_nonfinite_human_torque` | NaN effort rejection through the actual assist boundary. |

#### `assist_controller` → `reference.assist_controller`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_profile_owns_limits_and_lag` | Resolved profile owns torque, power, lag, gate and taper. |
| `test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag` | Original 40 ms analytic response, 500 ms continuation and gain. |
| `test_emtb_gain_rises_with_rider_torque` | Original support interpolation and settling bound. |
| `test_stationary_or_backward_crank_gets_nothing_even_with_pedal_pressure` | Every non-forward cadence case with positive sensed pressure. |
| `test_torque_below_engage_threshold_is_not_pedalling` | Below-threshold effort denies assist. |
| `test_release_of_rider_torque_hard_zeroes_and_forgets` | Settled assist clears immediately and forgets its filter state. |
| `test_positive_unloading_keeps_the_lag` | Positive reduced effort preserves lag instead of clearing it. |
| `test_speed_taper_and_cutoff` | Original below-taper, half-taper and hard-cutoff values. |
| `test_torque_curve_and_power_ceiling_bind_on_shaft_rpm` | Torque curve at 60/150 rpm, distinct shaft speed and power ceiling. |
| `test_external_request_is_a_ceiling_not_a_throttle` | Above/below/zero external cap with original settling bounds. |
| `test_braking_resets_everything` | Immediate brake reset of torque/gain/permission diagnostics. |
| `test_without_profile_the_plain_gain_is_used` | Synthetic scalar gain and original three-second settling interval. |
| `test_rejects_unknown_profile_mode_and_removed_knobs` | Named profile/mode errors and removed config fields through production parsing. |
| `test_assist_config_accepts_profile_and_mode` | Typed config accepts retained fields and rejects removed ones. |
| `test_slow_forward_pedalling_is_assisted_without_a_stall_timer` | Two-second slow-forward continuation without obsolete timer logic. |
| `test_crank_gate_resets_the_filter_even_with_an_external_cap` | All original cadence gate parameters and filter reset. |
| `test_cutoff_is_instantaneous_after_assist_has_settled` | Hard cutoff following genuine settled controller state. |
| `test_profile_overrides_explicit_torque_power_tau_speed_and_gate` | Actual shared projection overrides six conflicting raw limits. |
| `test_controller_rejects_invalid_parameters` | Three nonfinite numeric cases and array-valued mode at the wire boundary. |
| `test_assist_config_exposes_profile_torque_limit` | Profile and synthetic effective maximums. |
| `test_profile_controller_can_be_copied_for_read_only_force_probes` | Detached filter state and preserved gain data; Python shared-profile identity becomes owned-value continuation. |
| `test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile` | Real CLI parse/resolve plus temporary TOML, explicit override and absent-override paths. |

#### `cadence_shifter` → `reference.cadence_shifter`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_low_crank_cadence_downshifts_to_the_next_bigger_cog` | 28 → 33 teeth and down direction. |
| `test_a_wheel_implied_spike_alone_never_upshifts` | All 50 wheel-only spike intervals; original gear retained. |
| `test_high_crank_cadence_upshifts_when_the_landing_stays_in_band` | 28 → 24 teeth and up direction. |
| `test_upshifts_stop_at_the_smallest_cog` | 51 → 45, 12 → 10 and no upshift below 10. |
| `test_cooldown_and_not_pedalling_block_shifts` | Cooldown, not-pedalling, braking and rollback gates. |
| `test_wheelspin_blocks_an_upshift_but_not_a_downshift` | Same slip condition blocks upshift and allows downshift. |
| `test_ema_filters_a_single_stroke_spike` | Original initialization and five spike intervals. |
| `test_upshift_that_lands_below_the_band_is_rejected` | Rejection and unchanged 28-tooth gear. |
| `test_downshift_that_lands_above_the_band_is_rejected` | Rejection and unchanged 28-tooth gear. |

#### `pedal_waveform` → `reference.pedal_waveform`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_waveform_mean_and_ripple` | All 3,600 phase samples, mean and extrema with original bounds. |
| `test_waveform_rejects_invalid_ripple` | Original invalid ripple domain. |
| `test_force_requests_honour_the_ripple_argument` | Force requests vary with the explicit ripple parameter. |
| `test_horizontal_power_stroke_requests_the_finite_crank_torque` | All phase/friction combinations; independently computed lever torque. |
| `test_articulated_config_exposes_the_ripple` | Public articulated config keeps ripple and validates it. |
| `test_return_foot_preload_is_a_nonnegative_force` | Nonnegative recovery preload under original geometry. |
| `test_recovery_preload_preserves_the_requested_net_crank_waveform` | Every phase/tilt case, recovery normal force and total torque. |
| `test_recovering_foot_can_be_excluded_from_preload` | Excluded recovery force and retained front drive torque. |
| `test_infeasible_preload_geometry_never_requests_negative_drive` | Inverted geometry and mandatory recovery preload. |
| `test_preload_inputs_are_validated_and_zero_load_stays_bounded` | Original input failures and finite, zero-tangential-load bounds. |

#### `pedaling_policy` → `reference.pedaling_policy`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_reposition_knobs_and_mode_are_gone` | Removed schema/mode rejection; obsolete Python keyword assertion is language-only. |
| `test_a_rocking_loaded_crank_simply_keeps_pedalling` | All 3,000 original rocking intervals remain pedalling. |
| `test_ride_control_has_no_reposition_field` | Emitted wire omits removed field and decoder rejects insertion. |
| `test_rider_program_rejects_the_removed_reposition_command` | Actual program-document rejection; existing native diagnostic spelling is retained. |
| `test_coast_decision_filters_a_single_stroke_spike_with_ema` | Original 100/50/1,500/1,500 interval sequence, coast reason and resume. |
| `test_zero_tau_keeps_the_raw_hysteresis` | Original raw 90/121 rpm threshold crossing. |
| `test_coast_tau_must_be_nonnegative` | Both native policy and typed public config reject negative tau. |
| `test_wheel_implied_cadence_does_not_stop_legs_below_their_limit` | Leg cadence owns coast; requested cadence/effort retained. |
| `test_stopped_cranks_resume_while_the_open_freehub_wheel_keeps_turning` | Original 1,500 high-cadence and 350 stopped-crank intervals. |

#### `system_momentum_drive` → `reference.system_momentum_drive`

| Source function | Implemented contract / disposition |
| --- | --- |
| `test_airborne_motor_stand_delivers_work_without_root_actuation` | Native momentum rig at 0.00125 s; active motor/no rider, positive motor work, zero rider work and every original rig bound. |
| `test_passive_airborne_stand_has_no_motor_or_joint_work` | Passive rig at 0.00125 s; all three motor/joint work metrics exactly zero. |

## Task 3.18 — scalar policy suite

`native/tests/reference/test_native_drive_policies.cpp` implements all 31
source functions. `policy_reference_support.hpp` only constructs fixtures,
serializes actual typed results and consumes capture records; policy arithmetic
still runs in the existing production policy classes.

The five-policy owner formerly lived only in `policy_binding.cpp`.
`drivetrain/policy_set.hpp` now owns the same config and policy instances,
reset sequence, detached snapshot restore, and config/state relational gates.
The binding retains its original strict scalar parse and validation order,
then calls the shared relational gate before publication. No torque, cadence,
energy or freehub arithmetic changed. The existing writer config and drive
state translation units expose their strict owned codecs for this owner;
there is no test-only implementation of restoration or validation.

### Capture contract

The source helper used set-derived key order, which made an unlabeled golden
stream dependent on Python hash iteration. Its assertions now traverse dict
keys and the invalid-restore key set in sorted order. All comparison semantics
and parameter values remain intact. Every float has its field-path label;
string, bool, integer and null leaves are captured as compact JSON encoded as
UTF-8 `uint8` bytes. Shift return values additionally carry a scalar `bool`
capture. Explicit parameter IDs pin filenames independently of pytest's
formatting of compound objects.

Dict key arrays (`#keys`) and sequence lengths (`#length`) are captured before
their children, including empty containers. Consequently an extra empty
field cannot disappear from a flattened leaf stream. Scalars use shape
`[]`, vectors use `[n]`; the C++ consumers call the explicit shape overload,
so a scalar and a one-element vector do not compare as the same value.

Other suites importing this helper gain the same deterministic traversal.
Capture must therefore rerun complete affected files; old unlabeled data must
not be mixed with the new stream. The new C++ reader consumes every fixture,
return-value and snapshot capture in source order and ends each node with
`done()`. Missing, renamed, extra or reordered entries fail closed.

### Per-function disposition before qualification

`golden` below includes source functions whose fixture compares projected
config with its Python oracle. A contract assertion does not remove that
fixture comparison from the capture stream.

| Function | Disposition / comparison |
| --- | --- |
| `test_native_policy_api_exists` | Structural native owner check; Python `hasattr` detail is binding-specific. |
| `test_crank_torque` | Golden, binary64 bits, all 84 input combinations. |
| `test_battery_draw_matches_python` | Golden, four draws and complete snapshots. |
| `test_pedaling_sequences` | Golden, eight fixed cases plus 200 seeded cases (`934`). |
| `test_shifting` | Golden, both slip modes × both EMA choices, ten fixed + 250 seeded cases (`11`). |
| `test_assist` | Golden, five profile/mode cases, 36 ceilings and eight fixed + 250 seeded steps (`75`). |
| `test_energy_helpers` | Golden, both copper coefficients and enable values, every torque/speed/budget combination. |
| `test_freehub` | Golden, six complete updates and snapshots. |
| `test_replay_reset_and_ownership` | Golden plus direct ownership/invariant checks. |
| `test_invalid_restore_is_atomic` | Golden fixture/snapshots plus all malformed and missing member rejections. |
| `test_invalid_restore_values` | Golden fixture/snapshots plus all eight domain/type cases. |
| `test_config_validation` | Golden fixture plus all required-key and nonfinite-real rejections. |
| `test_projection_resolves_runtime` | Direct structural/analytic config and lifetime contract. |
| `test_projection_normalizes_numeric_leaves` | Owned-scalar tree contract; NumPy object-type normalization is Python-specific. |
| `test_complete_reset_and_atomic_multi_policy_restore` | Golden fixture/snapshots plus late-invalid-member atomicity. |
| `test_restored_finite_snapshots_preserve_bits` | Golden plus direct signed-zero and source-isolation checks. |
| `test_disabled_shifter_initial_teeth_outside_cassette` | Golden complete snapshot. |
| `test_invalid_config_values` | Golden fixture plus all 23 malformed configurations. |
| `test_missing_config_sections` | Golden fixture plus all seven missing sections/scalars. |
| `test_random_curve_interpolation` | Golden, seven endpoints + 1,000 seeded samples (`541`). |
| `test_battery_nonzero_idle_remainder` | Golden fixture plus direct idle-budget/no-debit invariant. |
| `test_config_and_python_object_deletion` | Golden result/state after source config lifetime ends. |
| `test_pedaling_parameter_branches` | Golden, all three parameter sets × six cases. |
| `test_optional_config_sections` | Golden fixture plus omitted profile/curve contract. |
| `test_extra_and_missing_state_sections` | Golden fixture/snapshots plus all three structural mutations. |
| `test_invalid_calls_preserve_state` | Golden fixture/snapshots plus all eleven invalid calls. |
| `test_invalid_human_torque` | Direct domain checks, four cases. |
| `test_shifter_count_boundary_rejects_successful_increment_atomically` | Golden fixture/snapshot plus actual overflow and legal no-shift interval. |
| `test_unrepresentable_count_restore_is_atomic` | Golden fixture/snapshot plus strict 32-bit decoding. |
| `test_assist_computed_crank_overflow_matches_python_state` | Golden, all eight profile/sign/shaft cases, overflow atomicity and continuation. |
| `test_assist_braking_resets_before_computed_crank_overflow` | Golden result and complete reset snapshot. |

No capture payload exists as evidence from this session. Every `golden` row
still needs the complete Python file green and freshly captured on both
`darwin-arm64` and `linux-aarch64`, followed by the corresponding CTest and
required level gates. In particular, a declared CTest or C++ source is not
`verified`, `replacement_landed`, or authority to delete its Python file.

## Task 3.18 — owning drivetrain, clone and topology

All four Task 3.18 successor sources are registered through
`native/tests/reference/Phase3Ports.cmake`. The native drivetrain and full
topology suites carry `slow`; both policy and drivetrain oracle suites
carry `golden`. The support headers only construct fixtures, call production
interfaces, serialize owned results and compare captures.

`runtime::project_drive_config(const SimulationPhysicsConfig&)` is a small
shared public projection seam. It owns and resolves its input, invokes the
existing drive projection and writer wire validation/decoder, and returns
the drive section. It does not change the call order or arithmetic in the
existing full `project_writer_config`. This lets the small drive fixtures
test production profile resolution without building an unrelated rider
model. `decode_policy_config`, `encode_policy_snapshot` and
`decode_policy_snapshot` similarly expose the existing owned codecs.

### `native_drivetrain` → `reference.native_drivetrain`: all 28 functions

Source: `tests/reference/test_native_drivetrain.py`.
Successor: `native/tests/reference/test_native_drivetrain.cpp`.
Every `golden` row still requires both real platform captures and native
execution. `Contract` rows also remain unexecuted; the table grants no
deletion authority.

| Source function | Exact preserved source scope / disposition |
| --- | --- |
| `test_chain_core` | Golden: all 12 seeds, two `normal(0,1)` vector draws, original rear offset/radii/reference draw, geometry tuple and full gradient. |
| `test_force_staging` | Golden: all three transmission families × four drive modes, components, controls, diagnostics and stored energy. |
| `test_solved_intervals` | Golden: seven valid family/topology pairs × dense/sparse × two effort modes, ten genuine engine steps each, pre/post diagnostics, settled forces, qpos/qvel and complete state. |
| `test_atomic_state_inputs_and_probes` | Golden plus direct invariants: read-only probe, preserved policies, restore, rejected NaN battery state and infinite inputs without control mutation. |
| `test_projection` | Actual public projection, initial rear teeth/mode contract and assist-section oracle. |
| `test_no_drive` | Contract: drive access without config fails with named diagnostic; bare engine still accepts its valid zero-input step. |
| `test_transmission_core` | Golden: both ideal families × reset/prepare/ratio, original engine state, boundary/ratio/range/coefficients and every geometric prepared field. |
| `test_random_multiturn_staging` | Golden: eight seeds × three families × 12 original sequential state changes and probes; qpos/qvel/time, pending clear, components, diagnostics, energy and state. |
| `test_shifting_prepared_state_and_clock` | Golden: both ideal families × contact/airborne/positive-slip/negative-slip, four shift preparations, explicit pending/clock transitions, duplicate-call rejection, battery stability and reset. |
| `test_energy_ceiling_and_actual_force` | Golden: all three families × empty/tiny/limited/disabled battery cases, ten genuine intervals with half-request motor controls; actual actuator force, energy and settled state. |
| `test_probe_does_not_touch_ratchets_or_live_state` | Golden: all families × active/inactive/braking × phases 0/40/-40, components/probe diagnostics/control and complete live-state preservation. |
| `test_invalid_config` | Contract: all six original malformed drive config cases via the production owned decoder. |
| `test_invalid_calls_are_atomic` | Golden plus contract: dt-zero, dt-NaN, speed-infinite, sensed-NaN, rider-integer and motor-limit-negative; complete state/control unchanged. The three binding-only call parameters listed below are dropped individually. |
| `test_invalid_restoration_is_atomic` | Golden plus contract: all 11 original malformed snapshots, including nonfinite values, shapes, strict bool/integer/count domains and full-state atomicity. |
| `test_set_inputs_cross_aliases_and_ownership` | Golden: original force/control cross-alias staging, all three malformed input cases, control/force rollback and component ownership after owner destruction. |
| `test_injected_valid_state_and_replay` | Golden: all three families, every injected policy/hub/transmission field, caller mutation isolation and eight genuine replay intervals with pre/post settlement state. |
| `test_pending_and_duplicate_settlement_guards` | Golden plus contract: both ideal families, original time/reset ordering, saved actual controls, unconsumed-interval failure, valid settlement, duplicate-settlement rejection and unchanged battery. |
| `test_passive_topology_without_actuators` | Golden: all families × coast/speed-control, actuator-free source XML, default source drive values, real zero-control force step and settlement/state. |
| `test_diagnostic_state_validation` | Golden plus contract: all six malformed diagnostic keys/types, complete snapshot unchanged. |
| `test_prepared_state_requires_boundary` | Golden plus contract: geometric prepared state with absent boundary is rejected atomically. |
| `test_real_solved_request_violation` | Contract: real engine step with 1,000 Nm motor control, then reserved-effort violation with the original diagnostic. No fabricated settled result. |
| `test_invalid_elastic_topology_rejects` | Contract: both illegal elastic clutch/rotor combinations at typed public config and native wire boundaries. |
| `test_optional_project_and_initial_gearing` | Golden: actual projected initial config, source rear gear versus mutated runtime gear, owned native optional absence. Python `SimpleNamespace`/attribute-deletion mechanics are dropped individually below. |
| `test_set_inputs_both_cross_buffer_aliases` | Golden: original two-joint/two-actuator model and simultaneous control↔force alias swap. |
| `test_diagnostic_ratio_and_integer_ranges` | Golden plus contract: ratio-zero, too-few-front-teeth and negative-shift-count diagnostics rejected atomically. |
| `test_genuine_wheel_and_rotor_overrun` | Golden: three families, original wheel/rotor speeds and eight genuine intervals, unrelated active joint-limit row, settlement/state and exact zero freehub torque. |
| `test_geometric_prepared_buffers_reuse_storage` | Golden in all four operation parameters plus storage contract: original prepare/restore/reset/real-shift operations; generation tokens/capacities remain stable and cover nv, with original repeated intervals and shift state. |
| `test_geometric_prepared_sentinel_owns_reused_storage` | Golden plus contract: absent prepared sentinel, early settlement rejection, restoration, caller Jacobian mutation isolation, reset and unchanged allocated storage. |

The per-parameter boundary dispositions in this suite are deliberately
narrow:

- `test_invalid_calls_are_atomic[braking-integer]`, `[active-string]` and
  `[advance-integer]` exercise strict Python argument conversion on three
  bool-valued call parameters. The native `TickInputs` call has typed bools
  and no corresponding dynamic adapter. Those three parameters are drops;
  they are not represented by passing stubs. The `rider-integer` parameter
  is retained through the actual public rider-control wire decoder, which
  does have a dynamic bool field.
- `test_optional_project_and_initial_gearing` retains the native config,
  ownership and initial-versus-runtime gear contracts. Python wrapper
  `SimpleNamespace`, absent attribute and `delattr` checks have no native
  object representation; no successful C++ rejection is claimed for them.
- Prepared-storage observations retain the production transmission's
  existing generation/capacity contract. The source Python test already
  exposes generations rather than addresses; the native test compares the
  same four observations, with a fixed typed layout replacing dict keys.

The original topology XML, unrelated carrier joint limit, dense/sparse
mode, reset/forward/prepare/step/settle order and RNG draw order are retained.
Scalar, vector and complete-tree oracle sites are consumed in source order.
Transmission range/coefficients/prepared arrays remain whole ndarray
captures; config lists and snapshot angles retain list structure. Every
node calls `done()`, so missing or extra records fail closed.

### `drivetrain_policy_clone` → `reference.drivetrain_policy_clone`

| Source function | Preserved contract / boundary disposition |
| --- | --- |
| `test_clone_matches_deepcopy_and_shares_no_mutable_state` | Actual native policy owner copies detached mutable state, cassette/torque-curve storage and configuration; mutations and continuation remain independent after source owner/config destruction. Original shifting/pedaling/assist parameters are covered. The source's hub parameter is an explicit skip for this ideal-drive fixture and is not counted as a native pass. Python `vars`/`deepcopy` and object-identity mechanics are translated to owned-value checks. |

### `motor_freewheel_topology` → `reference.motor_freewheel_topology`

| Source function | Preserved contract / boundary disposition |
| --- | --- |
| `test_rotor_torque_drives_the_crank_through_the_engaged_freewheel` | Original two-inertia XML, 200 intervals at 10 Nm, coupled velocity, analytic acceleration and transmitted torque. |
| `test_crank_overruns_a_coasting_rotor_without_backdriving_it` | Original acceleration, +5 rad/s crank overrun and 200 coast intervals; transmitted torque <1e-6 and velocity **absolute-only** tolerance 1e-9. |
| `test_rotor_catches_up_and_reengages_without_lash` | Original 200/50/400/50/20 interval sequence, ratchet boundary **absolute-only** tolerance 1e-6, reengaged torque and immediate reload. |
| `test_default_topology_is_rigid_crank_with_motor_on_the_crank` | Actual setup/settle/bootstrap model, absent auxiliary joints/tendons/transmissions, crank actuator and 0/85 range despite raw max torque 1. |
| `test_rotor_inertia_adds_a_freewheeled_rotor_body` | Actual model inertia/mass/freewheel/actuator, positive synchronized initial speed, distinct rotor/crank rates, independent original curve-segment expectation, shaft power and genuine settlement bounds. Private Python `driver_dof` introspection is replaced by named actuator/topology and observed transmission contracts. |
| `test_legacy_crank_clutch_still_builds_for_regression_ab` | Actual setup contains the legacy tendon, separate shaft actuator and clutch snapshot. |
| `test_rotor_and_legacy_clutch_are_exclusive` | All three original mutually exclusive/incompatible/negative-inertia constructions. |
| `test_rotor_inertia_requires_a_finite_nonnegative_scalar` | NaN, infinity and bool through the real dynamic public config constructor. |
| `test_rotor_requires_an_effort_drive_mode` | Actual public config rejects rotor inertia in coast mode. |

## Final source-review boundaries

Read affected Python functions, native policy/drive interfaces, owned
codecs, projection call sites and golden reader contracts. Source review
corrected a missing helper-function closing brace, used the exact
`normal(0,1)` generator operation, and retained absolute-only tolerances
where pytest explicitly supplied only `abs`. Snapshot containers and large
fixtures are owned; copies used for expected invariants outlive their
borrowed spans. Release assertions use `bt::require`.

This remains one static evidence mode. No compile, linker, frontend sweep,
runtime reproduction, platform capture, golden comparison, full momentum
rig, sanitizer or allocation measurement ran. Possible compilation,
numerical lowering, initialization and runtime differences must be resolved
by those prohibited-for-this-session gates before these source ports are
qualified. No fixture bytes or observed pass counts were invented.
