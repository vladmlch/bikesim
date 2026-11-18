# Unit & integration tests for native/src/golden/step_closure_bundle.

set(_golden_repo_root "${CMAKE_SOURCE_DIR}")
if(_golden_repo_root MATCHES "[\"\\\\]")
  message(FATAL_ERROR
    "golden test repository root cannot contain quotes or backslashes: ${_golden_repo_root}")
endif()

set(_golden_test_config_header
  "${CMAKE_CURRENT_BINARY_DIR}/bike_golden_test_config.h")
file(CONFIGURE OUTPUT "${_golden_test_config_header}"
  CONTENT "#pragma once\n#define BIKE_REPO_ROOT \"${_golden_repo_root}\"\n")

add_executable(test_step_closure_bundle
  "${CMAKE_CURRENT_SOURCE_DIR}/test_step_closure_bundle.cpp")
bike_startup_target_policy(test_step_closure_bundle "${CMAKE_SOURCE_DIR}/native")
target_compile_options(test_step_closure_bundle PRIVATE
  -include "${_golden_test_config_header}")
target_include_directories(test_step_closure_bundle PRIVATE
  "${CMAKE_SOURCE_DIR}/native/tests")
target_link_libraries(test_step_closure_bundle PRIVATE
  bike_test_fixtures bike_gpu_export_cli)
bike_rewrite_mujoco_load_name(test_step_closure_bundle "${BIKE_DEP_mujoco_LIBRARY}")
add_dependencies(test_step_closure_bundle bike-gpu-export)

add_test(NAME test_step_closure_bundle
  COMMAND test_step_closure_bundle "$<TARGET_FILE:bike-gpu-export>")
set_tests_properties(test_step_closure_bundle PROPERTIES
  LABELS "golden;unit"
  TIMEOUT 120)
