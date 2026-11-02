# numerics domain contract tests.

# kernel-parity regression: exact representable controls for source
# arithmetic plus the dyadic kernel-property tables (one executable — the
# properties TU has no entry point of its own).
add_executable(numerics_kernel_parity test_kernel_parity.cpp test_kernel_properties.cpp)
bike_startup_target_policy(numerics_kernel_parity "${_test_repo_root}/native")
target_link_libraries(numerics_kernel_parity PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(numerics_kernel_parity
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME numerics_kernel_parity COMMAND numerics_kernel_parity)
  set_tests_properties(numerics_kernel_parity PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:numerics_kernel_parity>")
endif()

add_executable(numerics_dense test_dense.cpp)
bike_startup_target_policy(numerics_dense "${_test_repo_root}/native")
target_link_libraries(numerics_dense PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(numerics_dense
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME numerics_dense COMMAND numerics_dense)
  set_tests_properties(numerics_dense PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:numerics_dense>")
endif()
