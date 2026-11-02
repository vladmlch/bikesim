# viewer domain contract tests. No window is opened by the test bodies;
# registration never invokes the executables.
foreach(_viewer_test IN ITEMS viewer_render viewer_hud)
  string(REGEX REPLACE "^viewer_" "" _viewer_src "${_viewer_test}")
  add_executable(${_viewer_test} "test_${_viewer_src}.cpp")
  bike_startup_target_policy(${_viewer_test} "${_test_repo_root}/native")
  target_link_libraries(${_viewer_test} PRIVATE bike_test_fixtures bike_viewer)
    bike_rewrite_mujoco_load_name(${_viewer_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_viewer_test} COMMAND ${_viewer_test})
    set_tests_properties(${_viewer_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_viewer_test}>")
  endif()
endforeach()
