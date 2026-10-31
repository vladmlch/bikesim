# Source ports from the completion plan. Labels classify execution; they do
# not waive the two-platform qualification required by the migration ledger.
bike_add_reference_test(realtime_measurement ENGINE
  SOURCES test_realtime_measurement.cpp LIBRARIES bike_realtime_measurement
  LABELS t53 runtime)
bike_add_reference_test(measure_realtime_report ENGINE
  SOURCES test_measure_realtime_report.cpp LIBRARIES bike_realtime_measurement
  LABELS t53 runtime)
bike_add_reference_test(performance_evidence ENGINE
  SOURCES test_performance_evidence.cpp LIBRARIES bike_realtime_measurement
  LABELS t53 runtime)
bike_add_reference_test(measure_realtime ENGINE
  SOURCES test_measure_realtime.cpp LIBRARIES bike_realtime_measurement
  LABELS t53 runtime slow)
# 3.10/F3 realtime gate: the >=1x RTF contract re-aimed at the produced
# Release bike-measure-realtime artifact (the Python-oracle subject is
# T63-dead). Not a bike_add_reference_test: each CTest node supplies the
# measured artifact plus its track/physics inputs as argv, one node per
# track, mirroring the two parametrized Python node IDs.
add_executable(reference.realtime_gate test_realtime_gate.cpp)
set_target_properties(reference.realtime_gate PROPERTIES CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_link_libraries(reference.realtime_gate PRIVATE
  bike_golden_test bike_platform)
target_include_directories(reference.realtime_gate PRIVATE
  "${CMAKE_SOURCE_DIR}/native/src")
bike_apply_native_policy(reference.realtime_gate)
add_dependencies(reference.realtime_gate bike-measure-realtime)
foreach(_realtime_gate_track IN ITEMS rough_uphill_savage rough_uphill_extreme)
  add_test(NAME "reference.realtime_gate.${_realtime_gate_track}"
    COMMAND reference.realtime_gate
      "$<TARGET_FILE:bike-measure-realtime>"
      "${CMAKE_SOURCE_DIR}/examples/research/${_realtime_gate_track}.toml"
      "${CMAKE_SOURCE_DIR}/examples/research/viewer_physics_welded.toml")
  set_tests_properties("reference.realtime_gate.${_realtime_gate_track}" PROPERTIES
    LABELS "reference;t53;runtime;slow" TIMEOUT 900
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:reference.realtime_gate>")
endforeach()
bike_add_reference_test(playback_clock ENGINE
  SOURCES test_playback_clock.cpp LIBRARIES bike_app bike_viewer
  LABELS t52 runtime)
bike_add_reference_test(control_clock ENGINE
  SOURCES test_control_clock.cpp LIBRARIES bike_app
  LABELS t52 runtime)
bike_add_reference_test(native_static_brake ENGINE
  SOURCES test_native_static_brake.cpp LIBRARIES bike_app LABELS t52 runtime golden)
# 3.20 T52 batch — brakes, cruise, suspension, clocks. Golden consumers
# replay the frozen pinned_savage_40_f2 episode (post-F2 recapture) and the
# native_suspension/legacy_env + control_clock/env_request bundles under
# tests/fixtures/golden/<platform>/.
bike_add_reference_test(native_cruise ENGINE
  SOURCES test_native_cruise.cpp LIBRARIES bike_app bike_random
  LABELS t52 runtime golden)
bike_add_reference_test(native_brake_resistance ENGINE
  SOURCES test_native_brake_resistance.cpp LIBRARIES bike_app bike_golden
  LABELS t52 runtime golden slow)
bike_add_reference_test(native_suspension ENGINE
  SOURCES test_native_suspension.cpp LIBRARIES bike_app bike_golden
  LABELS t52 runtime golden slow)
bike_add_reference_test(native_stepper ENGINE
  SOURCES test_native_stepper.cpp LIBRARIES bike_app LABELS t52 runtime golden)
# control_clock gains the env_request replay — needs the golden capture
# seam (bike_golden) and the research environment builder.
target_link_libraries(reference.control_clock PRIVATE bike_golden)
bike_add_reference_test(native_numeric_domains ENGINE
  SOURCES test_native_numeric_domains.cpp LIBRARIES bike_app LABELS t52 numerics)
bike_add_reference_test(native_numeric_completion ENGINE
  SOURCES test_native_numeric_completion.cpp LIBRARIES bike_app
  LABELS t52 numerics golden)
bike_add_reference_test(native_kernel_contracts ENGINE
  SOURCES test_native_kernel_contracts.cpp LIBRARIES bike_app LABELS t52 numerics)
bike_add_reference_test(pneumatic_tire_config ENGINE
  SOURCES test_pneumatic_tire_config.cpp LIBRARIES bike_app LABELS t52 tyre)
bike_add_reference_test(pneumatic_tire_forces ENGINE
  SOURCES test_pneumatic_tire_forces.cpp LIBRARIES bike_app bike_legacy LABELS t52 tyre)
bike_add_reference_test(pneumatic_tire_oracle ENGINE
  SOURCES test_pneumatic_tire_oracle.cpp LIBRARIES bike_app bike_legacy LABELS t52 tyre golden)
# 3.12a pneumatic kernel oracle ports (T52 batch) — replay the frozen
# two-platform goldens for geometry, carcass and brush.
bike_add_reference_test(native_pneumatic_geometry ENGINE
  SOURCES test_native_pneumatic_geometry.cpp LIBRARIES bike_app
  LABELS t52 tyre golden)
bike_add_reference_test(native_pneumatic_carcass ENGINE
  SOURCES test_native_pneumatic_carcass.cpp LIBRARIES bike_app bike_legacy
  LABELS t52 tyre golden)
bike_add_reference_test(native_pneumatic_brush ENGINE
  SOURCES test_native_pneumatic_brush.cpp LIBRARIES bike_app bike_random
  LABELS t52 tyre golden)
# 3.12b pneumatic remainder — the direct model API, the wire/projection
# layer and the writer end-to-end episodes, all on frozen two-platform
# goldens (fixture bundles under tests/fixtures/golden/<platform>/
# pneumatic_tire/ carry the episode/model/wire artifacts the writer port
# replays).
bike_add_reference_test(native_pneumatic_model ENGINE
  SOURCES test_native_pneumatic_model.cpp LIBRARIES bike_app bike_legacy
  bike_random LABELS t52 tyre golden)
bike_add_reference_test(native_pneumatic_wire ENGINE
  SOURCES test_native_pneumatic_wire.cpp LIBRARIES bike_app bike_random
  LABELS t52 tyre golden)
bike_add_reference_test(native_pneumatic_tire ENGINE
  SOURCES test_native_pneumatic_tire.cpp LIBRARIES bike_app bike_random
  bike_golden LABELS t52 tyre golden slow)
# 3.13 T52 terrain/tyre batch — the frozen-file surface asserts, the
# placement-refactor hash pins, the wheel-stand rigs and the portable
# distributed reference; the tyre road-profile + scalar-formula checks
# replay recorded oracle entries.
bike_add_reference_test(savage_track_surface ENGINE
  SOURCES test_savage_track_surface.cpp LIBRARIES bike_app
  LABELS t52 terrain)
bike_add_reference_test(terrain_placement_regression ENGINE
  SOURCES test_terrain_placement_regression.cpp LIBRARIES bike_app
  LABELS t52 terrain)
bike_add_reference_test(tyre_road_profile ENGINE
  SOURCES test_tyre_road_profile.cpp LIBRARIES bike_app bike_legacy
  LABELS t52 tyre golden)
bike_add_reference_test(native_distributed_stands ENGINE
  SOURCES test_native_distributed_stands.cpp LIBRARIES bike_app
  bike_validation LABELS t52 tyre slow)
bike_add_reference_test(distributed_tire_reference ENGINE
  SOURCES test_distributed_tire_reference.cpp LIBRARIES bike_app
  bike_validation LABELS t52 tyre golden slow)
bike_add_reference_test(alpine_track ENGINE
  SOURCES test_alpine_track.cpp LIBRARIES bike_app bike_cli
  LABELS t52 terrain slow)
bike_add_reference_test(native_tire ENGINE
  SOURCES test_native_tire.cpp LIBRARIES bike_app bike_random
  bike_golden LABELS t52 tyre golden slow)
bike_add_reference_test(native_distributed_tire ENGINE
  SOURCES test_native_distributed_tire.cpp LIBRARIES bike_app
  bike_random bike_golden LABELS t52 tyre golden slow)
bike_add_reference_test(native_clock_contracts ENGINE
  SOURCES test_native_clock_contracts.cpp LIBRARIES bike_app bike_random LABELS t52 runtime golden)
foreach(_drive_port IN ITEMS motor_profile cadence_shifter pedal_waveform pedaling_policy)
  bike_add_reference_test(${_drive_port} ENGINE
    SOURCES "test_${_drive_port}.cpp" LIBRARIES bike_app LABELS t52 drive)
endforeach()
bike_add_reference_test(assist_controller ENGINE
  SOURCES test_assist_controller.cpp LIBRARIES bike_app bike_cli LABELS t52 drive)
# Momentum is registered once the validation library is declared in the root
# graph; a missing implementation must be a configure error, never a skip.
bike_add_reference_test(system_momentum_drive ENGINE
  SOURCES test_system_momentum_drive.cpp LIBRARIES bike_validation LABELS t52 drive slow)
bike_add_reference_test(native_drive_policies ENGINE
  SOURCES test_native_drive_policies.cpp LIBRARIES bike_app bike_random
  LABELS t52 drive golden)
bike_add_reference_test(native_drivetrain ENGINE
  SOURCES test_native_drivetrain.cpp LIBRARIES bike_app bike_random
  LABELS t52 drive golden slow)
bike_add_reference_test(native_atomicity ENGINE
  SOURCES test_native_atomicity.cpp LIBRARIES bike_app bike_random
  LABELS t52 contracts)
bike_add_reference_test(native_input_contracts ENGINE
  SOURCES test_native_input_contracts.cpp LIBRARIES bike_app
  LABELS t52 contracts)
bike_add_reference_test(drivetrain_policy_clone ENGINE
  SOURCES test_drivetrain_policy_clone.cpp LIBRARIES bike_app LABELS t52 drive)
bike_add_reference_test(motor_freewheel_topology ENGINE
  SOURCES test_motor_freewheel_topology.cpp LIBRARIES bike_app LABELS t52 drive slow)
bike_add_reference_test(golden_episode_regressions ENGINE
  SOURCES test_golden_episode_regressions.cpp LIBRARIES bike_golden
  LABELS t52 episode golden slow)
# 3.22b recording column and period-buffer references.
bike_add_reference_test(recorder_columns ENGINE
  SOURCES test_recorder_columns.cpp LIBRARIES bike_app LABELS t52 recording slow)
bike_add_reference_test(hud_columns ENGINE
  SOURCES test_hud_columns.cpp LIBRARIES bike_app bike_viewer LABELS t52 recording)
bike_add_reference_test(period_buffer ENGINE
  SOURCES test_period_buffer.cpp LIBRARIES bike_app LABELS t52 recording golden slow)
# 3.22a encode columns — the columnar encoder + physical source against the
# frozen column_bits identity; the staged bootstrap replays the recorded
# ride on both qualified platforms.
bike_add_reference_test(native_encode_columns ENGINE
  SOURCES test_native_encode_columns.cpp LIBRARIES bike_recording
  LABELS t52 recording golden slow)
bike_add_reference_test(frozen_force_arrays ENGINE
  SOURCES test_frozen_force_arrays.cpp LIBRARIES bike_app LABELS t52 recording)
bike_add_reference_test(ride_plots_html_markers ENGINE
  SOURCES test_ride_plots_html_markers.cpp LIBRARIES bike_viz LABELS t52 recording)

# 3.14a rider contacts oracle ports (T52 batch) — the tiny-plant writer
# core, settle/sample face and contact math replay the frozen
# two-platform goldens; the schema/projection rows are pure ports.
bike_add_reference_test(native_rider_contacts ENGINE
  SOURCES test_native_rider_contacts.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
bike_add_reference_test(native_rider_contact_math ENGINE
  SOURCES test_native_rider_contact_math.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
foreach(_loader_case IN ITEMS
    test_native_contract_require_control_fails_in_selected_build
    test_native_contract_executable_lists_registered_cases
    test_contract_executable_source_is_in_cmake_manifest)
  add_test(NAME "reference.native_loader.${_loader_case}"
    COMMAND "${CMAKE_COMMAND}"
      "-DCASE=${_loader_case}"
      "-DCONTRACT_EXECUTABLE=$<TARGET_FILE:native_contract_tests>"
      "-DSOURCE_ROOT=${CMAKE_SOURCE_DIR}"
      "-DSOURCE_MANIFEST=${CMAKE_BINARY_DIR}/native_sources.json"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test_native_loader_contracts.cmake")
  set_tests_properties("reference.native_loader.${_loader_case}" PROPERTIES
    LABELS "reference;t53;checks" TIMEOUT 45)
endforeach()
# 3.29a T53 checks batch — engine error recovery/poison/callback-policy
# contracts from tests/reference/test_native_engine_errors.py (21 ported
# records; the three Python-only vendor-interop records stay `drop` in the
# ledger). The allocation probes spawn the selected build's
# native_contract_tests through platform::run, and the configure probe runs
# a scratch cmake configure against a wrong-version dependency root — both
# reach the test through a generated header (never a quoted compile
# definition), like the bike_native_bench_path.hpp pattern above.
bike_add_reference_test(native_engine_errors ENGINE
  SOURCES test_native_engine_errors.cpp LIBRARIES bike_app
  LABELS t53 checks)
add_dependencies(reference.native_engine_errors native_contract_tests)
file(GENERATE OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/support/bike_engine_errors_paths.hpp"
  CONTENT "#pragma once\n// Generated by native/tests/reference/Phase3Ports.cmake; do not edit.\n#define BIKE_CONTRACT_TESTS_PATH \"$<TARGET_FILE:native_contract_tests>\"\n#define BIKE_ENGINE_ERRORS_CMAKE_COMMAND \"${CMAKE_COMMAND}\"\n#define BIKE_ENGINE_ERRORS_REPO_ROOT \"${CMAKE_SOURCE_DIR}\"\n#define BIKE_ENGINE_ERRORS_DEPENDENCY_LOCK \"${BIKE_DEPENDENCY_LOCK}\"\n#define BIKE_ENGINE_ERRORS_DEPENDENCY_ROOT \"${BIKE_DEPENDENCY_ROOT}\"\n")
target_include_directories(reference.native_engine_errors PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/support")
# 3.16 rider allocation and dynamics.
bike_add_reference_test(native_rider_allocation ENGINE
  SOURCES test_native_rider_allocation.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(native_coupled_rider_task ENGINE
  SOURCES test_native_coupled_rider_task.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(native_rider_dynamics ENGINE
  SOURCES test_native_rider_dynamics.cpp LIBRARIES bike_app LABELS t52 rider)
bike_add_reference_test(native_rider_information_boundary ENGINE
  SOURCES test_native_rider_information_boundary.cpp LIBRARIES bike_app
  LABELS t52 rider)
bike_add_reference_test(project_constraint_mode_search ENGINE
  SOURCES project_test_constraint_mode_search.cpp LIBRARIES bike_app
  LABELS t52 rider)
# 3.14b contact-safety, attachment budget/wrench and rider-forces ports.
bike_add_reference_test(contact_safety ENGINE
  SOURCES test_contact_safety.cpp LIBRARIES bike_app
  LABELS t52 terrain golden)
bike_add_reference_test(attachment_budget ENGINE
  SOURCES test_attachment_budget.cpp LIBRARIES bike_app
  LABELS t52 rider)
bike_add_reference_test(attachment_wrench ENGINE
  SOURCES test_attachment_wrench.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
bike_add_reference_test(native_attachment_wrench ENGINE
  SOURCES test_native_attachment_wrench.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
bike_add_reference_test(native_rider_forces ENGINE
  SOURCES test_native_rider_forces.cpp LIBRARIES bike_app bike_golden
  LABELS t52 rider golden slow)
# 3.15 rider-control ports (T52 batch) — crank_split's pure virtual-work /
# preload / power-budget contracts need no engine.
bike_add_reference_test(crank_split
  SOURCES test_crank_split.cpp LIBRARIES bike_rider_math
  LABELS t52 rider)
bike_add_reference_test(leg_loop ENGINE
  SOURCES test_leg_loop.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(coupled_rider_task ENGINE
  SOURCES test_coupled_rider_task.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
# The welded-plant settle plus the enabled planned-crank episode runs
# ~16–36 min (task-C315 report, merge-back measurement) — outruns the
# ctest 1500 s default, so the registered node needs an explicit budget.
set_tests_properties(reference.coupled_rider_task PROPERTIES TIMEOUT 5400)
bike_add_reference_test(closed_form_rider ENGINE
  SOURCES test_closed_form_rider.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(joint_strength ENGINE
  SOURCES test_joint_strength.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(planar_arms ENGINE
  SOURCES test_planar_arms.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
# The remaining 3.15 files replay the NativeTestAdapter `rider_intent` and
# `spindle_controller` sessions as recorded JSON argument documents.
bike_add_reference_test(native_rider_intent ENGINE
  SOURCES test_native_rider_intent.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
# F5 (lane/e): native_spindle_controller replays the full A2.3
# spindle-controller family — all 23 source functions including the
# rear-leg-chain effort regression — against two-platform goldens over
# the shared pinned plant fixture. The six reject-only nodes run as
# native contract checks with no captured artifacts (task-F5 report).
bike_add_reference_test(native_spindle_controller ENGINE
  SOURCES test_native_spindle_controller.cpp LIBRARIES bike_app
  LABELS t52 rider golden)
# 3.17 seated topology and welds — the pinned-plant equality/joint
# topology, the welded-profile and articulated-config projection
# surfaces, the seated-climb/pedaling/posture programs and the held-weld
# integration episodes.
bike_add_reference_test(seated_climb_surge ENGINE
  SOURCES test_seated_climb_surge.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider)
bike_add_reference_test(seated_pedaling_cycle ENGINE
  SOURCES test_seated_pedaling_cycle.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider slow)
bike_add_reference_test(seated_posture_program ENGINE
  SOURCES test_seated_posture_program.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider slow)
bike_add_reference_test(seated_topology ENGINE
  SOURCES test_seated_topology.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider slow)
bike_add_reference_test(pinned_topology ENGINE
  SOURCES test_pinned_topology.cpp LIBRARIES bike_app
  LABELS t52 rider slow)
bike_add_reference_test(rider_welds ENGINE
  SOURCES ../integration/test_rider_welds.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider slow)
bike_add_reference_test(welded_profile_config ENGINE
  SOURCES test_welded_profile_config.cpp LIBRARIES bike_app
  LABELS t52 rider)
bike_add_reference_test(articulated_config ENGINE
  SOURCES test_articulated_config.cpp LIBRARIES bike_app
  LABELS t52 rider)
# 3.24a native_bench CLI/state contract port: the unit under test is the
# real native_bench executable from the same build, executed through the
# bike::platform::run argv boundary. The selected binary reaches the test as
# a generated-header macro (same pattern as bike_golden_paths.hpp) — never a
# quoted compile definition (the check-context writer embeds properties
# verbatim into native_check_context.json).
bike_add_reference_test(native_benchmark ENGINE
  SOURCES test_native_benchmark.cpp LIBRARIES bike_app
  LABELS t52 episode golden)
add_dependencies(reference.native_benchmark native_bench)
file(GENERATE OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/support/bike_native_bench_path.hpp"
  CONTENT "#pragma once\n// Generated by native/tests/reference/Phase3Ports.cmake; do not edit.\n#define BIKE_NATIVE_BENCH_PATH \"$<TARGET_FILE:native_bench>\"\n")
target_include_directories(reference.native_benchmark PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/support")

# 3.24b ride-backend CLI batch — the produced bike-ride/bike-research/
# bike-replay executables of the same build are the units under test for
# the route, replay-validation and missing-input records (the parse/resolve
# records drive cli::parse_arguments + cli::resolve_ride in-process, the
# seam the executables reach through cli::dispatch). The selected binaries
# arrive as generated-header macros — the bike_native_bench_path.hpp
# pattern, never a quoted compile definition.
bike_add_reference_test(ride_backend_cli ENGINE
  SOURCES test_ride_backend_cli.cpp LIBRARIES bike_cli
  LABELS t52 episode)
add_dependencies(reference.ride_backend_cli bike-ride bike-research bike-replay)
file(GENERATE OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/support/bike_ride_cli_paths.hpp"
  CONTENT "#pragma once\n// Generated by native/tests/reference/Phase3Ports.cmake; do not edit.\n#define BIKE_RIDE_PATH \"$<TARGET_FILE:bike-ride>\"\n#define BIKE_RESEARCH_PATH \"$<TARGET_FILE:bike-research>\"\n#define BIKE_REPLAY_PATH \"$<TARGET_FILE:bike-replay>\"\n")
target_include_directories(reference.ride_backend_cli PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/support")

# 3.23 T52 physics batch — monitors, energy and acceptance: the
# ReferenceMonitor strict/non-strict boundaries, BalanceMonitor
# grace/dwell/recovery, the sensor-noise draw order and pipeline replay
# against the frozen pre-refactor trace, the energy ledger's
# step-work/period-close contracts, monitor strictness over neutral raws
# plus the delayed research fixture, per-interval violation capture, the
# rider lean limit and the welded pedelec acceptance episodes.
bike_add_reference_test(reference_monitor ENGINE
  SOURCES test_reference_monitor.cpp LIBRARIES bike_app LABELS t52 physics)
bike_add_reference_test(balance_monitor ENGINE
  SOURCES test_balance_monitor.cpp LIBRARIES bike_app bike_cli
  LABELS t52 physics)
bike_add_reference_test(monitor_strictness ENGINE
  SOURCES test_monitor_strictness.cpp LIBRARIES bike_app LABELS t52 physics)
bike_add_reference_test(sensor_noise ENGINE
  SOURCES test_sensor_noise.cpp LIBRARIES bike_app bike_random
  LABELS t52 physics golden)
bike_add_reference_test(acceptance_interval_capture ENGINE
  SOURCES test_acceptance_interval_capture.cpp LIBRARIES bike_app
  LABELS t52 physics)
bike_add_reference_test(lean_limit ENGINE
  SOURCES test_lean_limit.cpp LIBRARIES bike_app LABELS t52 physics)
bike_add_reference_test(energy_ledger ENGINE
  SOURCES test_energy_ledger.cpp LIBRARIES bike_app LABELS t52 physics)
bike_add_reference_test(realistic_pedelec_acceptance ENGINE
  SOURCES test_realistic_pedelec_acceptance.cpp LIBRARIES bike_app
  bike_validation LABELS t52 physics slow)

# 3.25 T53 runtime batch — NativeRideRuntime construction/bootstrap,
# the bounded-advance command contract, sample acknowledgement and
# ownership, and the owned physical step. Ports exercise the welded
# bootstrap capture (tests/fixtures/migration/bootstrap/welded); the
# six+four live-oracle parity reds of the sources stay `planned` and are
# not reproduced (see task-C325-report.md).
bike_add_reference_test(native_runtime_bootstrap ENGINE
  SOURCES test_native_runtime_bootstrap.cpp LIBRARIES bike_app
  LABELS t52 runtime)
bike_add_reference_test(native_runtime_step ENGINE
  SOURCES test_native_runtime_step.cpp LIBRARIES bike_app
  LABELS t52 runtime)
bike_add_reference_test(native_runtime_commands ENGINE
  SOURCES test_native_runtime_commands.cpp LIBRARIES bike_app
  LABELS t52 runtime)
bike_add_reference_test(native_runtime_ownership ENGINE
  SOURCES test_native_runtime_ownership.cpp LIBRARIES bike_app
  LABELS t52 runtime)

# 3.26 T53 runtime batch — the resolved-configuration capability matrix,
# step/work accounting and interval history, rollout schedule/retention,
# and the C4 performance-acceptance contract. Native-vs-native contract
# ports: the four live-oracle parity reds of test_native_runtime_rollout.py
# stay `planned` (same weld-heavy parity family as 3.25; see
# task-C326-report.md) and the two "backend without resolved section"
# config cases are `drop`ped as python-only (the members are non-optional
# natively). The capability boundary is intentionally broader than the
# oracle's setup.py port-qualification gate; the ported contract asserts
# the native verdicts.
bike_add_reference_test(native_runtime_config ENGINE
  SOURCES test_native_runtime_config.cpp LIBRARIES bike_app
  LABELS t53 runtime)
bike_add_reference_test(native_runtime_accounting ENGINE
  SOURCES test_native_runtime_accounting.cpp LIBRARIES bike_app
  LABELS t53 runtime)
bike_add_reference_test(native_runtime_rollout ENGINE
  SOURCES test_native_runtime_rollout.cpp LIBRARIES bike_app
  LABELS t53 runtime)
bike_add_reference_test(native_runtime_performance_contract ENGINE
  SOURCES test_native_runtime_performance_contract.cpp
  LIBRARIES bike_app bike_realtime_measurement
  LABELS t53 runtime)

# 3.27 T53 state batch — BIKEST02 bundle reader/resolver contracts, the
# Stepper set_state/set_inputs aliasing and drive-state restore atomicity
# surfaces, deterministic seeded property fuzzing of the restore parsers,
# and the explicit equilibrium-state-file precedence/fingerprint/residual
# contracts including the post-fix canonical-cache poisoning guarantee.
bike_add_reference_test(native_state_bundle ENGINE
  SOURCES test_native_state_bundle.cpp LIBRARIES bike_app
  LABELS t53 state)
# resolve_artifacts lives inside the native_bench executable — the pointer
# resolution records keep the argv boundary through the generated macro.
add_dependencies(reference.native_state_bundle native_bench)
target_include_directories(reference.native_state_bundle PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/support")
bike_add_reference_test(native_state_restore ENGINE
  SOURCES test_native_state_restore.cpp
  LIBRARIES bike_app bike_random bike_golden
  LABELS t53 state)
bike_add_reference_test(native_state_fuzz ENGINE
  SOURCES test_native_state_fuzz.cpp LIBRARIES bike_app bike_random
  LABELS t53 state)
bike_add_reference_test(equilibrium_state_file ENGINE
  SOURCES test_equilibrium_state_file.cpp LIBRARIES bike_app
  LABELS t53 state)

# 3.30 T53 recording batch — the run-recording lifecycle (R03/R11/R14/R16),
# full/legacy/off retention ownership, RunCapture/journal contracts and the
# staged-publication fault-injection records of
# tests/reference/test_native_recording_lifecycle.py,
# test_native_recording_retention.py, test_recording_capture.py and
# test_recording_publication.py. Publication fault injection goes through
# FinalizeOptions::before_stage (the oracle's publisher monkeypatch).
# 67 records ported; 7 Python-only records stay `drop` in the ledger.
bike_add_reference_test(native_recording_lifecycle ENGINE
  SOURCES test_native_recording_lifecycle.cpp
  LIBRARIES bike_recording bike_cli
  LABELS t53 recording)
bike_add_reference_test(native_recording_retention ENGINE
  SOURCES test_native_recording_retention.cpp
  LIBRARIES bike_recording bike_cli
  LABELS t53 recording slow)
bike_add_reference_test(recording_capture ENGINE
  SOURCES test_recording_capture.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)
bike_add_reference_test(recording_publication ENGINE
  SOURCES test_recording_publication.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)

# 3.31 T53 recording batch — field/coordinate catalog, the ordinary
# bike-ride recording parser contract, report regeneration, storage and
# optional-value masks, the analyst Value surface and the bike-recording
# console of tests/reference/test_recording_{catalog,cli,report,storage,
# values,analysis,analyst_cli}.py over the 3.30 synthetic-capture seam.
# 66 records: 65 ported; the units/phases compare-schema record stays
# `drop` (no native seam authors divergent field metadata — units derive
# from joint_type inside build_catalog).
bike_add_reference_test(recording_catalog ENGINE
  SOURCES test_recording_catalog.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)
bike_add_reference_test(recording_cli ENGINE
  SOURCES test_recording_cli.cpp
  LIBRARIES bike_recording bike_cli
  LABELS t53 recording)
bike_add_reference_test(recording_report ENGINE
  SOURCES test_recording_report.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)
bike_add_reference_test(recording_storage ENGINE
  SOURCES test_recording_storage.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)
bike_add_reference_test(recording_values ENGINE
  SOURCES test_recording_values.cpp
  LIBRARIES bike_recording
  LABELS t53 recording)
bike_add_reference_test(recording_analysis ENGINE
  SOURCES test_recording_analysis.cpp
  LIBRARIES bike_recording bike_recording_cli
  LABELS t53 recording)
bike_add_reference_test(recording_analyst_cli ENGINE
  SOURCES test_recording_analyst_cli.cpp
  LIBRARIES bike_recording bike_recording_cli
  LABELS t53 recording)

# T53 replay/view migration ports (Batch 3.32) — replay/session/manifest/
# playback/view contracts from the nine Python sources.
bike_add_reference_test(replay_manifest_v2 ENGINE
  SOURCES test_replay_manifest_v2.cpp LIBRARIES bike_replay
  LABELS t53 replay)
bike_add_reference_test(replay_recording_v3 ENGINE
  SOURCES test_replay_recording_v3.cpp LIBRARIES bike_replay bike_recording
  LABELS t53 replay)
bike_add_reference_test(replay_session ENGINE
  SOURCES test_replay_session.cpp LIBRARIES bike_replay bike_recording
  LABELS t53 replay)
bike_add_reference_test(native_replay_recording_v3 ENGINE
  SOURCES test_native_replay_recording_v3.cpp LIBRARIES bike_replay bike_recording
  LABELS t53 replay)
bike_add_reference_test(native_replay_session ENGINE
  SOURCES test_native_replay_session.cpp LIBRARIES bike_replay bike_recording
  LABELS t53 replay)
bike_add_reference_test(pedelec_replay_contract ENGINE
  SOURCES test_pedelec_replay_contract.cpp LIBRARIES bike_replay bike_recording bike_validation bike_cli
  LABELS t53 replay)
add_dependencies(reference.pedelec_replay_contract bike-validate)
bike_add_reference_test(native_view_contracts ENGINE
  SOURCES test_native_view_contracts.cpp LIBRARIES bike_app
  LABELS t53 replay)
bike_add_reference_test(replay_playback ENGINE
  SOURCES ../integration/test_replay_playback.cpp LIBRARIES bike_replay bike_recording bike_app
  LABELS t53 replay)
add_dependencies(reference.replay_playback bike-replay)
bike_add_reference_test(native_viewer_snapshot ENGINE
  SOURCES test_native_viewer_snapshot.cpp LIBRARIES bike_replay bike_recording bike_app
  LABELS t53 replay)

# 3.33a T53 research batch — the NativeResearchRuntime external-boundary,
# capture-cursor, failure/reset, PolicySession lifecycle, program
# interpolation, legacy-export publication and sensor-delivery contracts
# from the eight test_native_research_*.py sources. The live-parity legs
# (welded-physics step/recorder/record comparisons plus one capture/
# recording oracle node) stay `planned` on the documented baseline drift;
# the program/sensor oracle legs replay per-platform goldens captured at
# the _golden sites added in this batch (task-C333a-report.md).
bike_add_reference_test(native_research_capture ENGINE
  SOURCES test_native_research_capture.cpp LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_environment ENGINE
  SOURCES test_native_research_environment.cpp LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_failures ENGINE
  SOURCES test_native_research_failures.cpp LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_frontend ENGINE
  SOURCES test_native_research_frontend.cpp research_reference_support.cpp
  LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_policy ENGINE
  SOURCES test_native_research_policy.cpp research_reference_support.cpp
  LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_programs ENGINE
  SOURCES test_native_research_programs.cpp LIBRARIES bike_app
  LABELS t53 research golden)
bike_add_reference_test(native_research_recording ENGINE
  SOURCES test_native_research_recording.cpp research_reference_support.cpp
  LIBRARIES bike_app bike_replay bike_recording
  LABELS t53 research)
bike_add_reference_test(native_research_sensors ENGINE
  SOURCES test_native_research_sensors.cpp LIBRARIES bike_app bike_random
  LABELS t53 research golden)

# 3.33b T53 research batch — capture-mode parity, sealed-evidence ownership,
# the verify consume cursor and the two-episode/failure/interrupt
# finalization contracts (test_research_capture.py); the policy-session
# partial-tick pacing contract (test_research_frontend.py); CLI opt-in,
# zero-step policy failure and the deferred-SIGINT lifecycle probes
# (test_research_recording_lifecycle.py — real signal delivery runs in
# isolated child processes); and the bike-research-sweep fan-out
# (test_research_sweep.py, driven against the bike-research-sweep artifact).
bike_add_reference_test(native_research_capture_contracts ENGINE
  SOURCES test_research_capture.cpp research_reference_support.cpp
  LIBRARIES bike_app bike_cli
  LABELS t53 research)
bike_add_reference_test(native_research_frontend_contracts ENGINE
  SOURCES test_research_frontend.cpp research_reference_support.cpp
  LIBRARIES bike_app
  LABELS t53 research)
bike_add_reference_test(native_research_recording_lifecycle ENGINE
  SOURCES test_research_recording_lifecycle.cpp research_reference_support.cpp
  LIBRARIES bike_app bike_cli bike_platform
  LABELS t53 research)
bike_add_reference_test(native_research_sweep ENGINE
  SOURCES test_research_sweep.cpp
  LIBRARIES bike_batch bike_cli bike_platform
  LABELS t53 research)
add_dependencies(reference.native_research_sweep bike-research-sweep)

# 3.34 frontends/realtime/async/batch batch — tests/test_batch.py port:
# spec expansion, reserved/validated flags, CPU policy and the real
# serial/pool episode equivalence through the bike-batch CLI artifact.
bike_add_reference_test(batch ENGINE
  SOURCES ../integration/test_batch.cpp LIBRARIES bike_app bike_cli
  LABELS t53 frontend slow)
add_dependencies(reference.batch bike-batch bike-research)
bike_add_reference_test(async_close ENGINE
  SOURCES ../integration/test_async_close.cpp LIBRARIES bike_app
  LABELS t53 frontend slow)
# 3.34 frontend/runtime batch — the seven remaining Python reference files.
# frontend_windows ports the fake-window key/pacer/preview/viewer-gate
# contracts; the two GUI/import-only nodes stay `drop` in the ledger.
bike_add_reference_test(frontend_windows ENGINE
  SOURCES test_frontend_windows.cpp LIBRARIES bike_app bike_viewer
  LABELS t53 frontend)
# physical_view ports RenderOwner apply/ownership/reject-before-mutate plus
# the live frame's HUD schema.
bike_add_reference_test(physical_view ENGINE
  SOURCES test_physical_view.cpp LIBRARIES bike_app bike_viewer
  LABELS t53 frontend)
# pinned_measurement ports EqualityReactions force/torque, the closure-row
# mask and the RiderContactWriter settle/sample face on the pinned plant.
bike_add_reference_test(pinned_measurement ENGINE
  SOURCES test_pinned_measurement.cpp LIBRARIES bike_app
  LABELS t53 frontend slow)
# realtime_deferred_checks ports the bulk road_samples parity, the deferred
# unexplained-wrench first-failure attribution and the eager/deferred
# identical-sample contract.
bike_add_reference_test(realtime_deferred_checks ENGINE
  SOURCES test_realtime_deferred_checks.cpp LIBRARIES bike_app
  LABELS t53 frontend)
# native_ride_frontend ports the real measurement driver
# (advance/reason/export/summary) plus per-generation seal dedupe/conflict
# through RunCapture and the physical_assets name set.
bike_add_reference_test(native_ride_frontend ENGINE
  SOURCES test_native_ride_frontend.cpp LIBRARIES bike_app bike_cli bike_recording bike_realtime_measurement
  LABELS t53 frontend slow)
# runtime_single_path ports the accounted HUD real-time factor /
# first-failure marker and the configured-dt measurement driver; the
# adapter-only records stay `drop`.
bike_add_reference_test(runtime_single_path ENGINE
  SOURCES test_runtime_single_path.cpp LIBRARIES bike_app bike_viewer bike_cli bike_realtime_measurement
  LABELS t53 frontend slow)
# pneumatic_physical_runtime ports the end-to-end pneumatic ride record on
# the repository pneumatic profile; the two codec records stay `contract`
# (reference.pneumatic_tire_forces + reference.equilibrium_state_file).
bike_add_reference_test(pneumatic_physical_runtime ENGINE
  SOURCES test_pneumatic_physical_runtime.cpp LIBRARIES bike_app bike_cli
  LABELS t53 frontend slow)

# 3.35 platform batch — tests/test_execution_identity.py port: the data-only
# identity-schema validation rules (legacy and identity_schema=2 records)
# plus the native-executable execution gate's named-field verdicts through
# replay::check_execution.
bike_add_reference_test(execution_identity ENGINE
  SOURCES test_execution_identity.cpp LIBRARIES bike_replay
  LABELS t53 platform)
# test_native_platform_replay.py port: same-host checked replay, foreign
# identity rejection before plant construction, recorded-frame playback of
# the refused file, and the never-consulted gate for inspection paths.
bike_add_reference_test(native_platform_replay ENGINE
  SOURCES test_native_platform_replay.cpp LIBRARIES bike_replay bike_recording
  LABELS t53 platform replay)
add_dependencies(reference.native_platform_replay bike-replay)
# tests/test_recording_platform_compatibility.py port: portable inspection
# and recorded-frame playback (subprocess CLI + in-process loaders) split
# from the checked-replay identity gate.
bike_add_reference_test(recording_platform_compat ENGINE
  SOURCES test_recording_platform_compatibility.cpp
  LIBRARIES bike_replay bike_recording
  LABELS t53 platform replay slow)
add_dependencies(reference.recording_platform_compat bike-recording bike-replay)
# tests/test_async_recovery.py port: detach/replay recovery ownership at
# the PeriodAccounting boundary (the _close_queues records are dropped —
# Python multiprocessing transport mechanics).
bike_add_reference_test(async_recovery ENGINE
  SOURCES test_async_recovery.cpp LIBRARIES bike_app
  LABELS t53 platform runtime)
# tests/reference/test_native_platform_recording.py port: exclusive-name
# collision, crash staging, EACCES+retry, unicode output path, the deep
# real-run payload, strict invalid prefix and the research publication
# contract (viewer-pump records are covered by the lifecycle twins).
bike_add_reference_test(native_platform_recording ENGINE
  SOURCES test_native_platform_recording.cpp
  LIBRARIES bike_app bike_recording bike_cli
  LABELS t53 platform recording slow)
# tests/reference/test_native_linux_headless.py port: the real bike-ride
# executable under a scrubbed display environment keeps its stepping and
# outcome contracts (the python-param/frontend/bootstrap records are
# Python-surface drops). Headless is platform-neutral natively — no GL
# or display dependency exists to make the behavior Linux-only.
bike_add_reference_test(native_linux_headless ENGINE
  SOURCES test_native_linux_headless.cpp
  LIBRARIES bike_platform bike_io bike_cli
  LABELS t53 platform slow)
# The ported subprocess probe drives the real bike-ride executable.
add_dependencies(reference.native_linux_headless bike-ride)
# tests/reference/test_native_platform_linkage.py port: every published
# bin/ artifact executes under a scrubbed link environment and every
# engine consumer's link table resolves the pinned MuJoCo image plus the
# configured numerics provider without rescue (otool -L on Darwin,
# readelf -d + ldd on Linux). The bike_native-import record is a
# Python-extension drop.
bike_add_reference_test(native_platform_linkage ENGINE
  SOURCES test_native_platform_linkage.cpp
  LIBRARIES bike_platform bike_cli
  LABELS t53 platform)
# The link-table probe enumerates the published bin/ artifacts.
add_dependencies(reference.native_platform_linkage
  bike-sim bike-playground bike-export bike-ride bike-settle bike-research
  bike-replay bike-batch bike-recording bike-research-sweep bike-validate
  bike-measure-realtime bike-test-child)
# tests/reference/test_native_numeric_portability.py port: every
# platform-sensitive numerical lowering replayed bitwise from captured
# goldens through the SAME production helpers the debug bindings call
# (np_tanh/hypot, python_vector_norm, min/max, multiply_broadcast,
# numpy_sum, sin/cos, np_interp, spindle-linalg mirrors).
bike_add_reference_test(native_numeric_portability ENGINE
  SOURCES test_native_numeric_portability.cpp
  LIBRARIES bike_app
  LABELS t53 platform numerics)
# tests/reference/test_native_platform_blas.py port: blas::ddot/dgemv/
# dgemm replayed bitwise from captured stored-byte operands through the
# same layout→entry-point rules the debug surface applies (the
# import-order record is a Python-subprocess drop).
bike_add_reference_test(native_platform_blas
  SOURCES test_native_platform_blas.cpp
  LIBRARIES bike_native_numerics
  LABELS t53 platform numerics)
# 3.36-B portability/tooling batch — worker platform contracts from
# tests/portability/test_worker_platform.py: performance-core selection on
# Darwin (perflevel sysctl) and Linux (process affinity) plus the hardware
# fallback. Multiprocessing start-method and interpreter-worker probes
# stay `drop` — native workers are std::jthread/std::thread.
bike_add_reference_test(worker_platform
  SOURCES test_worker_platform.cpp LIBRARIES bike_batch
  LABELS t53 portability)
