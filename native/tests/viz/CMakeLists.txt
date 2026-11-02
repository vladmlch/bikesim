# viz domain contract tests. Registration never invokes the executable.
add_executable(viz_plots test_plots.cpp)
bike_startup_target_policy(viz_plots "${_test_repo_root}/native")
target_link_libraries(viz_plots PRIVATE bike_test_fixtures bike_viz)
  bike_rewrite_mujoco_load_name(viz_plots
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME viz_plots COMMAND viz_plots)
  set_tests_properties(viz_plots PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:viz_plots>")
endif()
