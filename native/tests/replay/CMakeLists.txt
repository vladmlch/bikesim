# replay domain contract tests. Registration never invokes the
# executable.
add_executable(replay_session test_session.cpp)
bike_startup_target_policy(replay_session "${_test_repo_root}/native")
target_link_libraries(replay_session PRIVATE bike_test_fixtures
  bike_recording bike_replay)
  bike_rewrite_mujoco_load_name(replay_session
    "${BIKE_DEP_mujoco_LIBRARY}")

add_executable(replay_trajectory test_trajectory.cpp)
bike_startup_target_policy(replay_trajectory "${_test_repo_root}/native")
target_link_libraries(replay_trajectory PRIVATE bike_test_fixtures
  bike_recording bike_replay)
add_dependencies(replay_trajectory bike-replay)
  bike_rewrite_mujoco_load_name(replay_trajectory
    "${BIKE_DEP_mujoco_LIBRARY}")

add_executable(replay_golden_overlay test_golden_overlay.cpp)
bike_startup_target_policy(replay_golden_overlay "${_test_repo_root}/native")
target_link_libraries(replay_golden_overlay PRIVATE bike_test_fixtures
  bike_replay)
  bike_rewrite_mujoco_load_name(replay_golden_overlay
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME replay_session COMMAND replay_session)
  set_tests_properties(replay_session PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:replay_session>")
  add_test(NAME replay_trajectory COMMAND replay_trajectory
    "$<TARGET_FILE:bike-replay>")
  set_tests_properties(replay_trajectory PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:replay_trajectory>")
  add_test(NAME replay_golden_overlay COMMAND replay_golden_overlay)
  set_tests_properties(replay_golden_overlay PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:replay_golden_overlay>")
endif()
