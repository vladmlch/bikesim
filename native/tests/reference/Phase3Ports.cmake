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
bike_add_reference_test(playback_clock ENGINE
  SOURCES test_playback_clock.cpp LIBRARIES bike_app bike_viewer
  LABELS t52 runtime)
bike_add_reference_test(control_clock ENGINE
  SOURCES test_control_clock.cpp LIBRARIES bike_app
  LABELS t52 runtime)
bike_add_reference_test(native_static_brake ENGINE
  SOURCES test_native_static_brake.cpp LIBRARIES bike_app LABELS t52 runtime golden)
bike_add_reference_test(native_numeric_domains ENGINE
  SOURCES test_native_numeric_domains.cpp LIBRARIES bike_app LABELS t52 numerics)
bike_add_reference_test(native_kernel_contracts ENGINE
  SOURCES test_native_kernel_contracts.cpp LIBRARIES bike_app LABELS t52 numerics)
bike_add_reference_test(pneumatic_tire_config ENGINE
  SOURCES test_pneumatic_tire_config.cpp LIBRARIES bike_app LABELS t52 tyre)
bike_add_reference_test(pneumatic_tire_forces ENGINE
  SOURCES test_pneumatic_tire_forces.cpp LIBRARIES bike_app bike_legacy LABELS t52 tyre)
bike_add_reference_test(pneumatic_tire_oracle ENGINE
  SOURCES test_pneumatic_tire_oracle.cpp LIBRARIES bike_app bike_legacy LABELS t52 tyre golden)
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
bike_add_reference_test(frozen_force_arrays ENGINE
  SOURCES test_frozen_force_arrays.cpp LIBRARIES bike_app LABELS t52 recording)
bike_add_reference_test(ride_plots_html_markers ENGINE
  SOURCES test_ride_plots_html_markers.cpp LIBRARIES bike_viz LABELS t52 recording)

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
