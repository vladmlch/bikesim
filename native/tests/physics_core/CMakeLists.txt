# Unit tests for native/src/physics_core/ foundations (host + Apple Metal parity).
set(_physics_core_src_dir "${CMAKE_SOURCE_DIR}/native/src/physics_core")
if(_physics_core_src_dir MATCHES "[\"\\\\]")
  message(FATAL_ERROR
    "physics_core source dir cannot contain quotes or backslashes: ${_physics_core_src_dir}")
endif()
set(_physics_core_test_config_header
  "${CMAKE_CURRENT_BINARY_DIR}/bike_physics_core_test_config.h")
file(CONFIGURE OUTPUT "${_physics_core_test_config_header}"
  CONTENT "#pragma once\n#define BIKE_PHYSICS_CORE_SRC_DIR \"${_physics_core_src_dir}\"\n")

set(_physics_core_math_sources
  "${CMAKE_CURRENT_SOURCE_DIR}/test_physics_core_math.cpp")

if(APPLE)
  # Objective-C++ Metal parity helper: compiled in its own OBJECT library with
  # EXPORT_COMPILE_COMMANDS OFF so check_frontends/check_tidy/check_cppcheck
  # stay scoped to pure C++ translation units (matching bike_gpu_metal_runner_obj).
  add_library(bike_physics_core_metal_helper_obj OBJECT
    "${CMAKE_CURRENT_SOURCE_DIR}/test_physics_core_metal_helper.mm")
  bike_apply_native_policy(bike_physics_core_metal_helper_obj)
  set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/test_physics_core_metal_helper.mm"
    PROPERTIES
      LANGUAGE CXX
      COMPILE_OPTIONS "-x;objective-c++;-fobjc-arc;-std=c++20")
  set_target_properties(bike_physics_core_metal_helper_obj PROPERTIES
    EXPORT_COMPILE_COMMANDS OFF)
  target_include_directories(bike_physics_core_metal_helper_obj SYSTEM PRIVATE
    "${CMAKE_SOURCE_DIR}/native/src")
  list(APPEND _physics_core_math_sources
    $<TARGET_OBJECTS:bike_physics_core_metal_helper_obj>)
endif()

add_executable(test_physics_core_math ${_physics_core_math_sources})
bike_apply_native_policy(test_physics_core_math)
target_compile_options(test_physics_core_math PRIVATE
  -include "${_physics_core_test_config_header}")
target_include_directories(test_physics_core_math PRIVATE
  "${CMAKE_SOURCE_DIR}/native/src")
target_include_directories(test_physics_core_math SYSTEM PRIVATE
  "${BIKE_DEP_mujoco_ROOT}/include")
target_link_libraries(test_physics_core_math PRIVATE
  bike_core bike_engine)
bike_rewrite_mujoco_load_name(test_physics_core_math "${BIKE_DEP_mujoco_LIBRARY}")
if(APPLE)
  target_link_libraries(test_physics_core_math PRIVATE
    "-framework Metal" "-framework Foundation")
endif()

add_test(NAME test_physics_core_math COMMAND test_physics_core_math)
set_tests_properties(test_physics_core_math PROPERTIES LABELS "gpu;unit")

add_executable(test_terrain_scanner
  "${CMAKE_CURRENT_SOURCE_DIR}/test_terrain_scanner.cpp")
bike_apply_native_policy(test_terrain_scanner)
target_include_directories(test_terrain_scanner PRIVATE
  "${CMAKE_SOURCE_DIR}/native/src")
target_include_directories(test_terrain_scanner SYSTEM PRIVATE
  "${BIKE_DEP_mujoco_ROOT}/include")
target_link_libraries(test_terrain_scanner PRIVATE
  bike_test_fixtures bike_terrain bike_core bike_engine)
bike_rewrite_mujoco_load_name(test_terrain_scanner "${BIKE_DEP_mujoco_LIBRARY}")
add_test(NAME test_terrain_scanner COMMAND test_terrain_scanner)
set_tests_properties(test_terrain_scanner PROPERTIES LABELS "gpu;unit")
