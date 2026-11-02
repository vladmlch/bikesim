# config domain contract tests.

# writer-config boundary regression: the typed WriterConfig decode
# (writer_config.cpp) plus the Python-free Stepper construction path —
# the shared core object set (bike_core) and the pinned engine
# interceptor. No Python/nanobind include enters this target: compiling
# test_writer_config.cpp against stepper.hpp + writer_config.hpp is the
# include-closure contract itself.
# The allocation-override object lands via $<TARGET_OBJECTS:...> — see the
# bike_test_allocation_faults comment in ../CMakeLists.txt for why the TU
# compiles once.
add_executable(config_writer test_writer_config.cpp
  $<TARGET_OBJECTS:bike_test_allocation_faults>)
set_target_properties(config_writer PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(config_writer PRIVATE bike_test_fixtures)
bike_link_numerics(config_writer)
  bike_apply_native_policy(config_writer)
  target_include_directories(config_writer SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(config_writer PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(config_writer
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME config_writer COMMAND config_writer)
  set_tests_properties(config_writer PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:config_writer>")
endif()

add_executable(config_schema test_schema.cpp)
set_target_properties(config_schema PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(config_schema PRIVATE bike_test_fixtures
  bike_configuration)
target_include_directories(config_schema SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
  bike_apply_native_policy(config_schema)

if(BUILD_TESTING)
  add_test(NAME config_schema COMMAND config_schema)
  set_tests_properties(config_schema PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:config_schema>")
endif()

# configuration-loading regression: TOML/JSON decode into the typed
# records, _merge precedence + resolve_physics, referenced-asset resolution
# against the declaring file, and the candidate TOML writer — the same
# bike_configuration objects in both build graphs.
add_executable(config_resolution test_resolution.cpp)
set_target_properties(config_resolution PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(config_resolution PRIVATE bike_test_fixtures
  bike_configuration bike_platform)
target_include_directories(config_resolution SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
  bike_apply_native_policy(config_resolution)

if(BUILD_TESTING)
  add_test(NAME config_resolution COMMAND config_resolution)
  set_tests_properties(config_resolution PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:config_resolution>")
endif()
