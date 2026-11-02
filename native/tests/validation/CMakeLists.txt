# validation domain contract tests (root-only validation stands and
# evidence contracts).
if(BUILD_TESTING)
  add_executable(validation_acceptance test_acceptance.cpp)
  bike_startup_target_policy(validation_acceptance "${_test_repo_root}/native")
  target_link_libraries(validation_acceptance PRIVATE bike_test_fixtures bike_validation)
  bike_rewrite_mujoco_load_name(validation_acceptance "${BIKE_DEP_mujoco_LIBRARY}")
  add_dependencies(validation_acceptance bike-validate)
  add_test(NAME validation_acceptance COMMAND validation_acceptance)
  set_tests_properties(validation_acceptance PROPERTIES LABELS "validation;slow"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:validation_acceptance>")
endif()
