# initialization domain contract tests. The same app/library graph is
# exercised by all four executables; registration never invokes them.
foreach(_init_test IN ITEMS initialization_setup initialization_sag
    initialization_transfer initialization_pipeline)
  string(REGEX REPLACE "^initialization_" "" _init_src "${_init_test}")
  add_executable(${_init_test} "test_${_init_src}.cpp")
  bike_startup_target_policy(${_init_test} "${_test_repo_root}/native")
  target_link_libraries(${_init_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_init_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_init_test} COMMAND ${_init_test})
    set_tests_properties(${_init_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_init_test}>")
  endif()
endforeach()
