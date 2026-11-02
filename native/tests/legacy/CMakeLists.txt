# legacy domain contract tests. The same app/library graph is exercised
# by all three executables; registration never invokes them.
foreach(_legacy_test IN ITEMS legacy_ride legacy_metrics legacy_playground)
  string(REGEX REPLACE "^legacy_" "" _legacy_src "${_legacy_test}")
  add_executable(${_legacy_test} "test_${_legacy_src}.cpp")
  bike_startup_target_policy(${_legacy_test} "${_test_repo_root}/native")
  target_link_libraries(${_legacy_test} PRIVATE bike_test_fixtures bike_app)
  if(_legacy_test STREQUAL "legacy_playground")
    target_link_libraries(${_legacy_test} PRIVATE bike_playground)
  else()
    target_link_libraries(${_legacy_test} PRIVATE bike_legacy)
  endif()
    bike_rewrite_mujoco_load_name(${_legacy_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_legacy_test} COMMAND ${_legacy_test})
    set_tests_properties(${_legacy_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_legacy_test}>")
  endif()
endforeach()
