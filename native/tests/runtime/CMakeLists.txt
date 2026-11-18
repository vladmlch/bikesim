# runtime domain contract tests.

# owned-runtime boundary regression: read_bootstrap/compose_bootstrap
# (bootstrap_io.cpp + bootstrap.cpp) and NativeRideRuntime (runtime.cpp)
# — the shared core object set (bike_core)
# plus the pinned engine interceptor. No Python/nanobind include enters
# this target: compiling test_bootstrap.cpp against
# runtime/runtime.hpp is the include-closure contract itself.
add_executable(runtime_bootstrap test_bootstrap.cpp)
set_target_properties(runtime_bootstrap PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(runtime_bootstrap PRIVATE bike_test_fixtures)
bike_link_numerics(runtime_bootstrap)
  bike_apply_native_policy(runtime_bootstrap)
  target_include_directories(runtime_bootstrap SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(runtime_bootstrap PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(runtime_bootstrap
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME runtime_bootstrap COMMAND runtime_bootstrap)
  set_tests_properties(runtime_bootstrap PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_bootstrap>")
endif()

# encoder/provenance separation regression: parse_encode_plan /
# parse_derived_event_plan over staged bike::io::Value documents, the
# encode_table_native/applied_command_rows_native/
# derived_event_rows_native projectors over authored PhysicalSampleData
# and runtime_identity() — the shared core object set plus the pinned
# engine interceptor (mujoco resolves for runtime_identity) and
# bike_rider_math for the core's state_codec link refs. No Python/nanobind
# include enters this target: compiling test_encoding.cpp
# against runtime/encode.hpp + runtime/provenance.hpp is the
# include-closure contract itself.
add_executable(runtime_encoding test_encoding.cpp)
set_target_properties(runtime_encoding PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(runtime_encoding PRIVATE bike_test_fixtures)
bike_link_numerics(runtime_encoding)
  bike_apply_native_policy(runtime_encoding)
  target_include_directories(runtime_encoding SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(runtime_encoding PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(runtime_encoding
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME runtime_encoding COMMAND runtime_encoding)
  set_tests_properties(runtime_encoding PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_encoding>")
endif()

# RunLimits (runtime/run_limits.hpp) — header-only step/wall-clock caps of
# the physical ride, moved from the removed legacy controls.
add_executable(runtime_run_limits test_run_limits.cpp)
set_target_properties(runtime_run_limits PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(runtime_run_limits PRIVATE bike_test_fixtures)
bike_apply_native_policy(runtime_run_limits)
target_include_directories(runtime_run_limits SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
if(BUILD_TESTING)
  add_test(NAME runtime_run_limits COMMAND runtime_run_limits)
  set_tests_properties(runtime_run_limits PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_run_limits>")
endif()

add_executable(runtime_spatial_resistance_brake test_spatial_resistance_brake.cpp)
bike_startup_target_policy(runtime_spatial_resistance_brake "${_test_repo_root}/native")
target_link_libraries(runtime_spatial_resistance_brake PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(runtime_spatial_resistance_brake
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME runtime_spatial_resistance_brake COMMAND runtime_spatial_resistance_brake)
  set_tests_properties(runtime_spatial_resistance_brake PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_spatial_resistance_brake>")
endif()

add_executable(runtime_spatial_step_energy test_spatial_step_energy.cpp)
bike_startup_target_policy(runtime_spatial_step_energy "${_test_repo_root}/native")
target_link_libraries(runtime_spatial_step_energy PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(runtime_spatial_step_energy
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME runtime_spatial_step_energy COMMAND runtime_spatial_step_energy)
  set_tests_properties(runtime_spatial_step_energy PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_spatial_step_energy>")
endif()

add_executable(runtime_spatial_ride test_spatial_ride.cpp)
bike_startup_target_policy(runtime_spatial_ride "${_test_repo_root}/native")
target_link_libraries(runtime_spatial_ride PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(runtime_spatial_ride
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME runtime_spatial_ride COMMAND runtime_spatial_ride)
  set_tests_properties(runtime_spatial_ride PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_spatial_ride>")
endif()

add_executable(runtime_spatial_launch test_spatial_launch.cpp)
bike_startup_target_policy(runtime_spatial_launch "${_test_repo_root}/native")
target_link_libraries(runtime_spatial_launch PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(runtime_spatial_launch
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME runtime_spatial_launch COMMAND runtime_spatial_launch)
  set_tests_properties(runtime_spatial_launch PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_spatial_launch>")
endif()
