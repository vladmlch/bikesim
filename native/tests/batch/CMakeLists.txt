# batch domain contract tests. Registration never invokes the
# executable.
add_executable(batch_plan test_plan.cpp)
bike_startup_target_policy(batch_plan "${_test_repo_root}/native")
# bike_batch's ordered TOML parse pulls bike_terrain's TrackSpec methods,
# which are implemented against biketyre objects in the shared core set.
target_link_libraries(batch_plan PRIVATE bike_test_fixtures
  bike_batch bike_startup_runtime)
  bike_rewrite_mujoco_load_name(batch_plan
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_dependencies(batch_plan bike-test-child)
  add_test(NAME batch_plan COMMAND batch_plan
    "$<TARGET_FILE:bike-test-child>")
  set_tests_properties(batch_plan PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:batch_plan>")
endif()
