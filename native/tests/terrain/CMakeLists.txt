# terrain domain contract tests. The same app/library graph is exercised
# by both executables; registration never invokes them.
foreach(_terrain_test IN ITEMS terrain_generators terrain_pothole_report terrain_presets)
  string(REGEX REPLACE "^terrain_" "" _terrain_src "${_terrain_test}")
  add_executable(${_terrain_test} "test_${_terrain_src}.cpp")
  bike_startup_target_policy(${_terrain_test} "${_test_repo_root}/native")
  target_link_libraries(${_terrain_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_terrain_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_terrain_test} COMMAND ${_terrain_test})
    set_tests_properties(${_terrain_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_terrain_test}>")
  endif()
endforeach()
