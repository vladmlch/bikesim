# Legacy reference inventory — 2026-11-04

Audit for the plan `docs/superpowers/plans/2026-11-04-remove-legacy-mode.md`
(Tasks 2–13 delete `bike::legacy`, the bike-playground stand, and the seated
rider variant). Task 1 output: every user of each listed symbol outside the
code being deleted, with the task that handles it.

**Tool used:** `git grep -n -w <symbol> -- native tools docs examples` (plus
`tests packaging` and a few literal/extra patterns noted per table). The
mcp-steroid PSI backend was not reachable in this session, so this is the
plan's documented grep fallback. Caveat: grep cannot prove absence — a use
hidden behind a macro, an alias declared far from `bike::legacy`, a
generated file, or a non-word match could be missed. `legacy::`-qualified
uses were cross-checked with `git grep -n 'legacy::'` and every `#include`
of a `legacy/` header outside `native/src/legacy/` was enumerated; the
only includers are the files listed below. Base commit `d83e8d8e`.

**Legend:** deleted = the site dies with the file/function; switched = the
site is edited to the replacement symbol; kept = stays after the plan.

## Verification of the plan's expected result (brief Step 2)

- `RunLimits::for_track` in physical code: **verified** —
  `cli/ride.cpp:130` (inside `physical_run`, confirmed by reading the
  function) and `runtime/realtime_driver.cpp:49,131`.
- `pothole_reports` at `viz/ride.cpp:131` inside `track_profile` (kept;
  used by `bike-ride --preview` at `cli/ride.cpp:342`): **verified**.
- "Nothing else from `bike::legacy`" in physical code: **verified** —
  every other `bike::legacy`/`legacy::` use in kept files sits inside a
  function the plan deletes (`legacy_run`, `legacy_headless`, `run_legacy`,
  `legacy_frame`, `tyre_plot`, `acceleration_plot`) or is a declaration
  deleted with it (`viewer/session.hpp:9` forward declarations).
- "Tests use `front_tyre`, `rear_tyre`, `tier_spec`, `TYRE_TIERS`,
  `TyreConfig`, `build_road_profile`, `TyreForceApplier`": **verified**
  (the reference test files below; `tests/legacy/` dies entirely).

## Stop-condition result

No physical-mode user of a listed symbol exists that the plan does not
handle. Two plan-coverage gaps were found (not stop-condition hits — see
"Notes for later tasks"): `rider_geometry.cpp:1234` and the
`"seated"` choices in `argument_contracts.inc:271,916`.

## Per-symbol inventory

`T<n>` = plan task. "legacy tree" = `native/src/legacy/` +
`native/tests/legacy/` (all deleted by T8).

### `bike::legacy` (namespace)

| file:line | status | task |
|---|---|---|
| `native/src/legacy/*` (namespace blocks in all 19 files) | deleted | T8 |
| `native/src/cli/ride.cpp:11,12` includes `../legacy/metrics.hpp`, `../legacy/ride.hpp`; `:38,67,76,238–330` legacy path (`legacy_options`, `legacy_headless`, `legacy_run`, `legacy_directory_name`, `drivetrain_value`, `ride_track_seed`) | deleted | T5 |
| `native/src/cli/ride.cpp:130` `legacy::RunLimits::for_track` in `physical_run` | switched | T2 |
| `native/src/runtime/realtime_driver.cpp:9` include `../legacy/controls.hpp`; `:49,131` `bike::legacy::RunLimits` | switched | T2 |
| `native/src/app/native_ride_main.cpp:20` include; `:154,160,171` (`RideOptions`, `make_ride`, `RunOutcome`) inside `run_legacy` | deleted | T6 |
| `native/src/cli/playground.cpp:2,7` (`legacy::make_playground`) | deleted | T8 |
| `native/src/viewer/ride.cpp:7,8` includes; `:185–215` (`legacy_frame`, `run_legacy`, `run_playground`) | deleted | T7 (`:185–212`), T8 (`:214–215`) |
| `native/src/viewer/session.hpp:9` fwd decls `RideSimulation`/`Playground`; `:58,60` decls | deleted | T7 / T8 |
| `native/src/viz/ride.cpp:2,3` includes; `:86,94,109` inside `acceleration_plot`/`tyre_plot` | deleted | T7 |
| `native/src/viz/ride.cpp:131` `legacy::pothole_reports` inside kept `track_profile` | switched | T3 |
| `native/tests/legacy/*` (`test_metrics.cpp:28,49`, `test_playground.cpp:18,23`, `test_ride.cpp:26,47,67–81`) | deleted | T8 |
| `native/tests/reference/pneumatic_tire_fixture.hpp:59,60,63,80`; `pneumatic_reference_support.hpp:42` include | switched | T4 |
| `native/tests/reference/test_native_pneumatic_carcass.cpp:49,50,72,126,147,282,359,484,544,593,623`; `test_native_pneumatic_model.cpp:332,333,362,381,410,450,480,520,552,609,660,684,716,804,821,883,907,934,960` | switched | T4 |
| `native/tests/reference/test_pneumatic_tire_forces.cpp:30,137`; `test_pneumatic_tire_oracle.cpp:36,38,87`; `test_tyre_road_profile.cpp:12,51,84,85` | switched/deleted per T4 steps 3–4 | T4 |
| `native/cmake/NativePhysicsResearch.cmake:58–78` (`bike_legacy`, `bike_playground`); `RecordingApplication.cmake:88,120,171,180` | deleted | T8 (`:120` in T7) |
| `native/tests/reference/Phase3Ports.cmake:75,77,84,95,114` `bike_legacy` LIBRARIES | deleted | T4 |
| `native/tests/CMakeLists.txt:68` `add_subdirectory(legacy)` | deleted | T8 |

### `RunLimits`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/controls.hpp:131–139`; `controls.cpp:318–330` def | deleted (moved to `bike::runtime::RunLimits`) | T2 produces, T8 deletes |
| `native/src/cli/ride.cpp:130` in `physical_run` | switched | T2 |
| `native/src/runtime/realtime_driver.cpp:49,131` | switched | T2 |
| `native/src/legacy/ride.cpp:435–436`; `ride.hpp:92` | deleted | T8 |
| `native/tests/legacy/test_metrics.cpp:61` | deleted | T8 |

### `termination_reason`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/controls.hpp:139`; `controls.cpp:333`; `legacy/ride.cpp:439` | deleted | T8 |
| `native/src/cli/ride.cpp:298` `legacy::termination_reason` in `legacy_headless` | deleted | T5 |
| `native/tests/legacy/test_metrics.cpp:61,62` | deleted | T8 |
| `native/src/cli/ride.cpp:104`; `native/src/viz/ride.cpp:45` | **kept** — string literal naming a physical wire channel / marker key, unrelated to the legacy function | — |

### `pothole_reports`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/metrics.hpp:89`; `metrics.cpp:125,162` | deleted (moved to `bike::terrain::pothole_reports`) | T3 produces, T8 deletes |
| `native/src/viz/ride.cpp:131` in kept `track_profile` | switched | T3 |

### `PotholeReport`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/metrics.hpp:27,71`; `metrics.cpp:125,126,181` | deleted (moved to `bike::terrain::PotholeReport`) | T3 produces, T8 deletes |
| outside legacy tree | no named users (`viz/ride.cpp:131` binds via `auto&`) | — |

### `front_tyre` / `rear_tyre`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:32,33,48`; `tyre.cpp:23,27` (return `nativecfg::PneumaticTyreSpec` via `using TyreSpecs =`) | deleted (moved to `bt::tyres::front_tyre`/`rear_tyre`) | T4 produces, T8 deletes |
| `native/src/viz/ride.cpp:109` inside `tyre_plot` | deleted | T7 |
| `native/tests/legacy/test_ride.cpp:37` (`front_tyre` only) | deleted | T8 |
| `native/tests/reference/pneumatic_tire_fixture.hpp:59,60,80`; `test_tyre_road_profile.cpp:84`; `test_native_pneumatic_carcass.cpp` (9 sites); `test_native_pneumatic_model.cpp` (16 sites) | switched | T4 |

### `tier_spec` (legacy function)

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:45,52`; `tyre.cpp:43,51,122` | deleted (moved to `bt::tyres::tier_spec`) | T4 produces, T8 deletes |
| `native/tests/legacy/test_ride.cpp:31,37` | deleted | T8 |
| `native/tests/reference/pneumatic_tire_fixture.hpp:63`; `test_pneumatic_tire_forces.cpp:137`; `test_tyre_road_profile.cpp:85` | switched | T4 |

### `TierSpec`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:39,45,52`; `tyre.cpp:43` — the legacy struct | deleted (re-created as `bt::tyres::TierSpec`) | T4 produces, T8 deletes |
| `native/src/configuration/physical_config.hpp:125,134,138`; `schema.cpp:870,871`; `validate.cpp:399,970,971` | **kept — different symbol, same name**: `configuration::TierSpec` + `kTireTiers` + `find_tire_tier`, the physical tire-tier config | — |

### `TYRE_TIERS`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:18` | deleted (replaced by `bt::tyres::kTiers`) | T4 produces, T8 deletes |
| `native/tests/reference/test_pneumatic_tire_oracle.cpp:87` | switched | T4 |

### `TyreConfig`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:46–56`; `tyre.cpp:48,56,110,115,139,204`; `legacy/ride.hpp:50` | deleted | T8 |
| `native/tests/legacy/test_ride.cpp:27` | deleted | T8 |
| `native/tests/reference/test_pneumatic_tire_forces.cpp:30` (static_assert) | deleted | T4 |
| `native/tests/reference/test_pneumatic_tire_oracle.cpp:36` | deleted | T4 |
| `docs/research/tyre-calibration.md:33` | **kept** — historical research doc, out of scope | — |

### `build_road_profile`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:59`; `tyre.cpp:62`; `legacy/ride.cpp:182` | deleted (moved to `bt::tyres::build_road_profile`) | T4 produces, T8 deletes |
| `native/tests/reference/test_tyre_road_profile.cpp:51` | switched | T4 |

### `first_crossing_reference`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/tyre.hpp:61`; `tyre.cpp:70`; `native/tests/legacy/test_ride.cpp:33,35` | deleted | T8 |
| outside legacy tree | no users | — |

### `kGravityMps2`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/metrics.hpp:8` (the legacy constant); `metrics.cpp:84,163` | deleted | T8 |
| `native/src/viz/ride.cpp:86,94` inside `acceleration_plot` | deleted | T7 |
| `native/src/configuration/rider_spec.hpp:36`; `model/rider_geometry.cpp:302,369` | **kept — different symbol, same name**: `configuration::kGravityMps2`, used by physical rider preload math | — |

### `kRampExclusionM`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/metrics.hpp:8`; `metrics.cpp:150` | deleted | T8 |
| `native/src/cli/ride.cpp:312` inside `legacy_headless` | deleted | T5 |

### `legacy_lowpass`

| file:line | status | task |
|---|---|---|
| `native/src/numerics/filter.hpp:7`; `filter.cpp:115` def; `filter.cpp:143` call inside `legacy_ride_filter` | deleted | T7 |
| `native/src/viz/ride.cpp:87,94` inside `acceleration_plot` | deleted | T7 |
| `native/src/legacy/metrics.cpp:85`; `native/tests/legacy/test_metrics.cpp:20,22,23,25` | deleted | T8 |
| T7 condition "no other user" | **satisfied** — every caller is deleted code | — |

### `legacy_ride_filter`

| file:line | status | task |
|---|---|---|
| `native/src/numerics/filter.hpp:9`; `filter.cpp:141` | deleted | T7 |
| `native/tests/legacy/test_metrics.cpp:74,75` (only callers) | deleted | T8 |

### `travel_plot` / `shaft_velocity_plot` / `acceleration_plot` / `tyre_plot` / `plot_ride`

| file:line | status | task |
|---|---|---|
| `native/src/viz/ride.cpp:53,62,83,99,190` defs; `viz/plot.hpp:77,78,79,80,86` decls | deleted | T7 |
| `native/src/viz/ride.cpp:191` internal calls inside `plot_ride` | deleted | T7 |
| `native/src/cli/ride.cpp:313` (`plot_ride`), `:315` (`tyre_plot`) inside `legacy_headless` | deleted | T5 |

### `track_profile`

| file:line | status | task |
|---|---|---|
| `native/src/viz/ride.cpp:119` def; `viz/plot.hpp:81` decl | **kept** | — |
| `native/src/cli/ride.cpp:314` inside `legacy_headless` | deleted | T5 |
| `native/src/cli/ride.cpp:342` `bike-ride --preview` call site | **kept** | — |
| `native/src/viz/ride.cpp:131` `legacy::pothole_reports` (its only legacy use) | switched | T3 |

### `run_legacy`

| file:line | status | task |
|---|---|---|
| `native/src/viewer/ride.cpp:198` def; `viewer/session.hpp:58` decl | deleted | T7 |
| `native/src/app/native_ride_main.cpp:153` def; `:240` call | deleted | T6 |
| `native/src/cli/ride.cpp:330` call inside `legacy_run` | deleted | T5 |

### `run_playground`

| file:line | status | task |
|---|---|---|
| `native/src/viewer/ride.cpp:214` def; `viewer/session.hpp:60` decl; `native/src/cli/playground.cpp:11` call | deleted | T8 |

### `legacy_frame`

| file:line | status | task |
|---|---|---|
| `native/src/viewer/ride.cpp:185` def; `:202` call inside `run_legacy` | deleted | T7 |

### `viewer::Mode::legacy`

| file:line | status | task |
|---|---|---|
| `native/src/viewer/input.hpp:9` enum member; `input.cpp:24,25,67`; `viewer/ride.cpp:199` | deleted | T7 |
| `native/src/cli/ride.cpp:328` inside `legacy_run` | deleted | T5 |
| `native/tests/viewer/test_hud.cpp:63` | deleted | T7 |

### `viewer::Mode::playground`

| file:line | status | task |
|---|---|---|
| `native/src/viewer/input.hpp:9` enum member; `input.cpp:22`; `viewer/ride.cpp:215`; `native/src/cli/playground.cpp:9`; `native/tests/viewer/test_hud.cpp:64` | deleted | T8 |

### `playground_command`

| file:line | status | task |
|---|---|---|
| `native/src/cli/application.hpp:24` decl; `application.cpp:20` dispatch; `cli/playground.cpp:6` def; `cli/simulation.cpp:28` dispatch | deleted | T8 |

### `make_playground`

| file:line | status | task |
|---|---|---|
| `native/src/legacy/playground.hpp:107,109`; `playground.cpp:92` | deleted | T8 |
| `native/src/cli/playground.cpp:7` call; `native/tests/legacy/test_playground.cpp:23` | deleted | T8 |

### `StaticEquilibriumContext` / `StaticEquilibriumOptions`

| file:line | status | task |
|---|---|---|
| `native/src/initialization/legacy_equilibrium.hpp:26,13` defs, `:46,47` decls; `legacy_equilibrium.cpp:25,26` | deleted | T8 (`NativeStartup.cmake:83` entry too) |
| `native/src/legacy/ride.cpp:215,231`; `legacy/playground.hpp:115`; `legacy/static_equilibrium.cpp:8` (a `bike::legacy` forwarding shim) | deleted | T8 |
| outside legacy tree | no users (`solve_static_equilibrium` called only from `legacy/ride.cpp:235` and the shim) | — |

### `BuildRequest::tyre_model` (field `tyre_model`)

| file:line | status | task |
|---|---|---|
| `native/src/model/builder.hpp:31` field; readers `builder.cpp:18,21,22`, `drivetrain.cpp:61`, `linkage.cpp:21`, `export.cpp:183`; `export.hpp:15` comment | deleted (T9 condition "only legacy users" holds — all readers are the legacy pneumatic MJCF branches) | T9 |
| `native/src/legacy/ride.cpp:173` setter | deleted | T8 |
| `native/tests/model/test_mjcf.cpp:250,256`; `test_export.cpp:68,78` | edited | T8a / T9 (`:68` is not in a listed range — see notes) |
| CLI plumbing (`cli/argument_contracts.inc:405`; `cli/configuration.cpp:142,157`; `cli/ride.cpp:249,290` legacy `TyreConfig.model`) | deleted | T5 |

### `RiderForcesWriter`

| file:line | status | task |
|---|---|---|
| `native/src/writers/rider_forces.hpp:23,52`; `rider_forces.cpp:53,79,96,117,125,142,152,157` | deleted | T11 |
| `native/src/stepper.hpp:31,344,501`; `stepper.cpp:14,156,353` | deleted | T11 |
| `native/src/legacy/rider.cpp:19`; `legacy/rider.hpp:8,26,27` | deleted | T8 |
| `native/tests/test_contracts.cpp:21,2709,3908,3914`; `test_native_rider_forces.cpp` | edited/deleted | T11 |
| `docs/TESTING.md:1148` (`RiderForcesWriter::try_compute_into` row) | **kept doc — flag for T12** (not among T12's named rows) | T12 |

### `RiderForcesConfig`

| file:line | status | task |
|---|---|---|
| `native/src/config_types.hpp:360` def; `config_validation.hpp:219` validate; `writer_config.hpp:57,864–867` field + decoder; `writers/rider_forces.cpp:54`, `rider_forces.hpp:52` | deleted | T11 |
| `native/src/legacy/rider.cpp:15` | deleted | T8 |
| `native/tests/test_contracts.cpp:442,449,2640` | edited | T11 |

### `rider_forces` (config section / `Stepper::rider_forces()` / member)

| file:line | status | task |
|---|---|---|
| `native/src/configuration/schema.cpp:505,593–596`; `config_types.hpp:344–365`; `config_validation.hpp:213–221`; `writer_config.hpp:47,57,324,476–485,864–880,1183–1184` | deleted | T11 |
| `native/src/runtime/step.cpp:31,1304–1307,2339–2341,2482–2485`; `stepper.cpp:14,154–156,353–356`; `stepper.hpp:344,369,501` | deleted | T11 |
| `native/src/runtime/project_config.cpp:436` (emits section for seated rider only) | deleted | T11 |
| `native/src/initialization/legacy_equilibrium.hpp:33`; `legacy_equilibrium.cpp:51`; `legacy/ride.cpp:113,138,219,249`; `legacy/rider.hpp:8` | deleted | T8 |
| tests per T11 list (`test_native_rider_forces.cpp`, `test_writer_config.cpp:79,91,183`, `test_native_input_contracts.cpp:139,182,219,570`, `test_native_numeric_completion.cpp:143,181,534,764–766`, `test_contracts.cpp:21,395,442–449,2640`) | edited/deleted | T11 |
| `tests/fixtures/golden/*/native_rider_forces/` (incl. `rider_forces_qfrc_bitwise.json`) | deleted | T11 |
| `tests/fixtures/golden/*/native_suspension/legacy_env/wire.json` | deleted | T10 |
| `docs/research/simulation-audit-2026-10-07.md`; `docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md`, `2026-11-04-no-joint-damping.md` | **kept** — historical docs | — |

### `has_rider_forces`

| file:line | status | task |
|---|---|---|
| `native/src/stepper.hpp:355`; `runtime/step.cpp:1304,2339,2482`; `native/tests/config/test_writer_config.cpp:183` | deleted | T11 |

### `rider_forces_qfrc`

| file:line | status | task |
|---|---|---|
| `native/src/stepper.hpp:371`; `stepper.cpp:748,752` | deleted | T11 |
| `native/tests/reference/test_native_rider_forces.cpp:226,350,352,385,412,415`; `tests/fixtures/golden/*/native_rider_forces/rider_forces_qfrc_bitwise.json` | deleted | T11 |

### `build_seated_rider`

| file:line | status | task |
|---|---|---|
| `native/src/model/legacy_topology.cpp:19` def; `:131` call | deleted | T11 (`legacy_topology.cpp` itself stays — it also builds the lumped rider; `:132` lumped condition edited by T8a) |

### `saddle_top_z_for_rider`

| file:line | status | task |
|---|---|---|
| `native/src/model/mass.cpp:512–518` def; `mass.hpp:197–200` decl | deleted — T11's condition holds: sole caller `mass.cpp:550` passes the `std::optional` straight to `geometry::saddle_geometry`, which accepts `nullopt` | T11 |
| `native/src/model/mass.cpp:550` call site | edited | T11 |
| `native/tests/model/test_inertia.cpp:642–656` | edited | T11 |

### `legacy_behavior`

| file:line | status | task |
|---|---|---|
| `native/src/runtime/project_config.cpp:125,126,136,143,169` (`legacy_behavior = true` switch + wire writes) | edited — writes become constant `false` | T10 |
| `native/src/writers/suspension.cpp:202` `sd_legacy_damping_force`; `:230,231,262,263,272` branches; `:411,421` HBO guards; `physical_` member `:381,389,429,534,540,550,618` | deleted | T10 |
| `native/src/writer_config.hpp:531–534`; `config_validation.hpp:92`; `configuration/schema.cpp:1108` | edited — accept `physical` only, reject `legacy_behavior == true` | T10 |
| Wire field kept always-false: `config_types.hpp:144,152,161`; `schema.cpp:523–536,672,1114–1119`; `config_validation.hpp:60,73`; `validate.cpp:1188–1204,1422–1464` (HBO validation + ctor defaults) | **kept** — T10 keeps the field; reads stay correct under always-false | T10 (implicit) |
| `native/src/model/suspension_setup.hpp:173,260,303,375`; `suspension_setup.cpp:140–161,549–567,733–735,845–847,913,946,1009–1018` | **kept — flag**: shared physical/legacy file with `legacy_behavior` fields and `if (legacy_behavior)` branches; not in T10's Files list; stays consistent because the flag is always false (branches become dead code) | — (see notes) |
| `native/tests/reference/test_native_numeric_completion.cpp:427,431,876`; `test_native_numeric_domains.cpp:102`; `test_native_input_contracts.cpp` (9 hits); `test_writer_config.cpp` (9 hits); `test_contracts.cpp` (3 hits) — the `true` cases | deleted per T10 Step-1 sweep | T10 |
| `native/tests/model/test_inertia.cpp:1759–1785` (asserts wire field == damper field, both false) | **kept** — still valid | — |
| `tests/fixtures/**/config.json`, `tests/fixtures/golden/*/…/wire.json`, `tests/fixtures/model/dampers.json` with `"legacy_behavior": false` | **kept** — bytes unchanged (field stays, always false) | — |
| `tests/fixtures/golden/*/native_suspension/legacy_env/` (only fixtures with `true` + `physics_mode: legacy`) | deleted | T10 |
| `tools/fp32_probe/artifacts/common/captured_bootstrap.json` (`"legacy_behavior": false`) | **kept** — captured artifact | — |
| `docs/TESTING.md` (suspension-legacy rows) | edited | T12 |

### `sd_legacy_damping_force`

| file:line | status | task |
|---|---|---|
| `native/src/writers/suspension.cpp:202` def; `:231,263` calls | deleted | T10 |
| sibling `SuperDeluxeDamper::compute_legacy_damping_force` (`model/suspension_setup.hpp:346`; `suspension_setup.cpp:914,947,953`) | **kept — flag**: same curve, different symbol, not in the plan's list; dead once `legacy_behavior` is always false | — (see notes) |

## Kept — out-of-scope "legacy" meanings (verified present; must not be deleted)

| symbol/site | evidence |
|---|---|
| `replay/legacy.cpp` + `tests/fixtures/replay/legacy_v1`, `legacy_v2`, `historical_v2` | file exists; fixture dirs exist |
| `load_legacy_input` | `native/src/replay/input.cpp`, `input.hpp` |
| `RetentionMode::legacy` | `runtime/accounting.{hpp,cpp}`, `runtime/config.hpp:67`, `bootstrap.cpp:289`, fixtures `"record_retention": "legacy"` |
| `ResearchCaptureMode::legacy` / `capture_mode='legacy'` | `runtime/research.{hpp,cpp}`, `research/environment.*`, `research/policy_session.cpp`, `replay/export.cpp:206`, fixtures |
| `configuration/legacy_battery.hpp` (`configuration::legacy::*`) | header exists; users `initialization/initial_state.cpp:63`, `replay/rebuild.cpp:79–85` |
| `legacy_equilibrium_fingerprint`, `legacy_source_root` | `initialization/equilibrium_cache.cpp`, `setup_owner.*`, `state_file.hpp`, `app/native_ride_main.cpp`, `replay/export.cpp` |
| `rider/legacy_allocation.cpp`, `rider/legacy_controller.cpp` | files exist (closed-form plan Task 11 owns them) |
| `model/legacy_topology.cpp` | file exists; builds the lumped rider too — only `build_seated_rider` + its call die (T11); `:132` edited by T8a |
| `SlipMode::legacy_signed` | `drivetrain/shifting.hpp:62,123`; `test_contracts.cpp:2246` |
| `app::make_ride`, `configuration::TyreSpecs`/`DrivetrainSpecs`/`with_pressure`/`TierSpec`/`kGravityMps2`, `nativecfg::SuspensionController`, `pneumatic::PRESSURE_*` | same-named physical symbols, not `bike::legacy` members |
| `"seated"` posture name | `validation/acceptance.cpp:237` — a posture label, not the rider variant |
| `"termination_reason"`/`"outcome"` wire keys | `cli/ride.cpp:104`, `viz/ride.cpp:45` — physical channel names |

## Notes for later tasks (plan-coverage observations, not stop-condition hits)

1. **`rider_geometry.cpp:1234` will break under T11.** `geometry_pose()` does
   `neutral.variant = "seated"; … configuration::detail::post_init(neutral);`
   (line 1239). `detail::post_init` is the validating post_init — T11 removes
   `"seated"` from the accepted set at `validate.cpp:864`, so the call starts
   throwing and `geometry_pose` (used by the articulated rider path) breaks
   unless T11 also edits the `neutral.variant`/post_init lines. T11's Files
   list names `rider_geometry.cpp:901–907` only.
2. **`argument_contracts.inc:271,916`** keep `"seated"` in `--rider` choices;
   no task lists them. Parsing stays loud (post_init rejects), but help text
   is stale — fold into T11 or T12.
3. **`model/suspension_setup.{hpp,cpp}`** is shared physical/legacy code with
   `legacy_behavior` fields, `compute_legacy_damping_force`, and
   `if (legacy_behavior)` branches at `:913,:946`. Not in T10's Files list;
   consistent under the always-false rule, but the dead branches remain —
   T10 or a follow-up decides whether to strip them.
4. **`arguments.cpp:187`** `command_names()` still lists `bike-playground`;
   T8's packaging grep finds it, but it is not in T8's Files list.
5. **`native/tests/cli/CMakeLists.txt:24`** and
   **`native/tests/reference/Phase3Ports.cmake:695`** `add_dependencies`
   name `bike-playground` — build breaks unless swept (T8's grep finds them).
6. **`test_export.cpp:68`** `request.tyre_model="pneumatic"` sits between
   T8a's listed lines (`:34–40,76–77,84`) and T9's `:78` — unlisted site.
7. **`docs/TESTING.md:1148`** names `RiderForcesWriter::try_compute_into`;
   T12's row list does not name it explicitly.
