# recording domain contract tests. Declaring these executables does not
# run a test, simulate a ride or generate a recording during
# configuration.

# catalog contract: reader-side only — no MuJoCo link rewrite.
add_executable(recording_catalog test_catalog.cpp)
bike_startup_target_policy(recording_catalog "${_test_repo_root}/native")
target_link_libraries(recording_catalog PRIVATE bike_test_fixtures
  bike_recording_reader)
if(BUILD_TESTING)
  add_test(NAME recording_catalog COMMAND recording_catalog)
  set_tests_properties(recording_catalog PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:recording_catalog>")
endif()

foreach(_recording_test IN ITEMS recording_encoding recording_capture
    recording_database recording_publish recording_reader
    recording_reports)
  string(REGEX REPLACE "^recording_" "" _recording_src "${_recording_test}")
  add_executable(${_recording_test} "test_${_recording_src}.cpp")
  bike_startup_target_policy(${_recording_test} "${_test_repo_root}/native")
  target_link_libraries(${_recording_test} PRIVATE bike_test_fixtures bike_recording)
  if(_recording_test STREQUAL "recording_reports")
    target_link_libraries(${_recording_test} PRIVATE bike_replay)
  endif()
    bike_rewrite_mujoco_load_name(${_recording_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_recording_test} COMMAND ${_recording_test})
    set_tests_properties(${_recording_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_recording_test}>")
  endif()
endforeach()
