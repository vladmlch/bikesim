# initialization domain contract tests. The same app/library graph is
# exercised by all four executables; registration never invokes them.
foreach(_init_test IN ITEMS initialization_setup initialization_sag
    initialization_transfer initialization_pipeline
    initialization_initial_state_record)
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
add_executable(initialization_spatial_equilibrium test_spatial_equilibrium.cpp)
bike_startup_target_policy(initialization_spatial_equilibrium "${_test_repo_root}/native")
target_link_libraries(initialization_spatial_equilibrium PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(initialization_spatial_equilibrium
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME initialization_spatial_equilibrium COMMAND initialization_spatial_equilibrium)
  set_tests_properties(initialization_spatial_equilibrium PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:initialization_spatial_equilibrium>")
endif()

add_executable(initialization_spatial_finalize test_spatial_finalize.cpp)
bike_startup_target_policy(initialization_spatial_finalize "${_test_repo_root}/native")
target_link_libraries(initialization_spatial_finalize PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(initialization_spatial_finalize
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME initialization_spatial_finalize COMMAND initialization_spatial_finalize)
  set_tests_properties(initialization_spatial_finalize PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:initialization_spatial_finalize>")
endif()

# Only the two document-regenerating targets link the record seam.
target_link_libraries(initialization_pipeline PRIVATE
  bike_test_initial_state_record)
target_link_libraries(initialization_initial_state_record PRIVATE
  bike_test_initial_state_record)
