foreach(_spatial_test IN ITEMS spatial_frames spatial_config)
  string(REGEX REPLACE "^spatial_" "" _spatial_src "${_spatial_test}")
  add_executable(${_spatial_test} "test_${_spatial_src}.cpp")
  bike_startup_target_policy(${_spatial_test} "${_test_repo_root}/native")
  target_link_libraries(${_spatial_test} PRIVATE
    bike_test_fixtures bike_spatial_core bike_configuration)
  if(BUILD_TESTING)
    add_test(NAME ${_spatial_test} COMMAND ${_spatial_test})
    set_tests_properties(${_spatial_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_spatial_test}>")
  endif()
endforeach()

add_executable(spatial_whipple_benchmark test_whipple_benchmark.cpp)
bike_startup_target_policy(spatial_whipple_benchmark "${_test_repo_root}/native")
target_link_libraries(spatial_whipple_benchmark PRIVATE
  bike_test_fixtures bike_spatial_whipple)
  bike_rewrite_mujoco_load_name(spatial_whipple_benchmark
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME spatial_whipple_benchmark COMMAND spatial_whipple_benchmark)
  set_tests_properties(spatial_whipple_benchmark PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:spatial_whipple_benchmark>")
endif()
