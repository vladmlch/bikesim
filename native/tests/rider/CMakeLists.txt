# rider domain contract tests. The same app/library graph is exercised
# by both executables; registration never invokes them.
foreach(_rider_test IN ITEMS rider_constraints rider_contacts)
  string(REGEX REPLACE "^rider_" "" _rider_src "${_rider_test}")
  add_executable(${_rider_test} "test_${_rider_src}.cpp")
  bike_startup_target_policy(${_rider_test} "${_test_repo_root}/native")
  target_link_libraries(${_rider_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_rider_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_rider_test} COMMAND ${_rider_test})
    set_tests_properties(${_rider_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_rider_test}>")
  endif()
endforeach()
