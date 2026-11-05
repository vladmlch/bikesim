# cli domain contract tests. Registration never invokes the executables.

# analyst CLI regression: needs the produced bike-recording executable at
# test time.
add_executable(cli_recording test_recording.cpp)
bike_startup_target_policy(cli_recording "${_test_repo_root}/native")
target_link_libraries(cli_recording PRIVATE bike_test_fixtures
  bike_recording bike_recording_cli)
add_dependencies(cli_recording bike-recording)
  bike_rewrite_mujoco_load_name(cli_recording
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME cli_recording COMMAND cli_recording
    "$<TARGET_FILE:bike-recording>")
  set_tests_properties(cli_recording PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:cli_recording>")
endif()

# command-dispatch / physical-capture regression: exercises the produced
# bike-* executables themselves.
add_executable(cli_capture test_capture.cpp)
bike_startup_target_policy(cli_capture "${_test_repo_root}/native")
target_link_libraries(cli_capture PRIVATE bike_test_fixtures bike_cli)
add_dependencies(cli_capture bike-sim bike-playground bike-export
  bike-ride bike-settle bike-research bike-recording bike-replay bike-batch
  bike-native-ride)
  bike_rewrite_mujoco_load_name(cli_capture
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME cli_capture COMMAND cli_capture
    "$<TARGET_FILE_DIR:bike-research>" "$<TARGET_FILE:bike-native-ride>")
  set_tests_properties(cli_capture PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:cli_capture>")
endif()
