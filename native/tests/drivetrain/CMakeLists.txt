# drivetrain domain contract tests. The same app/library graph is
# exercised; registration never invokes the executable.
add_executable(drivetrain_transmission test_transmission.cpp)
bike_startup_target_policy(drivetrain_transmission "${_test_repo_root}/native")
target_link_libraries(drivetrain_transmission PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(drivetrain_transmission
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME drivetrain_transmission COMMAND drivetrain_transmission)
  set_tests_properties(drivetrain_transmission PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:drivetrain_transmission>")
endif()
