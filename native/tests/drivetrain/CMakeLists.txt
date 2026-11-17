# drivetrain domain contract tests. The same app/library graph is
# exercised; registration never invokes the executable.
foreach(_drivetrain_test IN ITEMS drivetrain_transmission drivetrain_spatial_drivetrain)
  string(REGEX REPLACE "^drivetrain_" "" _drivetrain_src "${_drivetrain_test}")
  add_executable(${_drivetrain_test} "test_${_drivetrain_src}.cpp")
  bike_startup_target_policy(${_drivetrain_test} "${_test_repo_root}/native")
  target_link_libraries(${_drivetrain_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_drivetrain_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_drivetrain_test} COMMAND ${_drivetrain_test})
    set_tests_properties(${_drivetrain_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_drivetrain_test}>")
  endif()
endforeach()
