# writers domain contract tests.
add_executable(writers_compliant_3d_tire test_compliant_3d_tire.cpp)
bike_startup_target_policy(writers_compliant_3d_tire "${_test_repo_root}/native")
target_link_libraries(writers_compliant_3d_tire PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(writers_compliant_3d_tire
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME writers_compliant_3d_tire COMMAND writers_compliant_3d_tire)
  set_tests_properties(writers_compliant_3d_tire PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:writers_compliant_3d_tire>")
endif()

add_executable(writers_distributed_3d_tire test_distributed_3d_tire.cpp)
bike_startup_target_policy(writers_distributed_3d_tire "${_test_repo_root}/native")
target_link_libraries(writers_distributed_3d_tire PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(writers_distributed_3d_tire
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME writers_distributed_3d_tire COMMAND writers_distributed_3d_tire)
  set_tests_properties(writers_distributed_3d_tire PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:writers_distributed_3d_tire>")
endif()
