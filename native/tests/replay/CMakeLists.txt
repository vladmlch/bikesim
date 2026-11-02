# replay domain contract tests. Registration never invokes the
# executable.
add_executable(replay_session test_session.cpp)
bike_startup_target_policy(replay_session "${_test_repo_root}/native")
target_link_libraries(replay_session PRIVATE bike_test_fixtures
  bike_recording bike_replay)
  bike_rewrite_mujoco_load_name(replay_session
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME replay_session COMMAND replay_session)
  set_tests_properties(replay_session PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:replay_session>")
endif()
