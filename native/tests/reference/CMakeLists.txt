# Reference ports of pytest contracts. Missing oracle data always fails;
# captured rows become authoritative only after both platform runs pass.
if(NOT BUILD_TESTING)
  return()
endif()
string(TOLOWER "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}" BIKE_GOLDEN_PLATFORM)
set(BIKE_GOLDEN_ROOT "${CMAKE_SOURCE_DIR}/tests/fixtures/golden")
if(BIKE_GOLDEN_ROOT MATCHES "[\"\\\\]")
  message(FATAL_ERROR "golden root cannot contain quotes or backslashes")
endif()
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/support/bike_golden_paths.hpp"
  CONTENT "#pragma once\n#define BIKE_GOLDEN_ROOT \"${BIKE_GOLDEN_ROOT}\"\n#define BIKE_GOLDEN_PLATFORM \"${BIKE_GOLDEN_PLATFORM}\"\n")
add_library(bike_golden_test STATIC ../support/golden.cpp)
target_link_libraries(bike_golden_test PUBLIC bike_migration_test bike_io)
target_include_directories(bike_golden_test PRIVATE
  "${CMAKE_SOURCE_DIR}/native/src" "${CMAKE_CURRENT_BINARY_DIR}/support")
bike_apply_native_policy(bike_golden_test)

function(bike_add_reference_test name)
  cmake_parse_arguments(REF "ENGINE" "" "SOURCES;LIBRARIES;LABELS" ${ARGN})
  if(REF_UNPARSED_ARGUMENTS OR NOT REF_SOURCES)
    message(FATAL_ERROR "bike_add_reference_test(${name}): invalid arguments or no sources")
  endif()
  set(_target "reference.${name}")
  add_executable(${_target} ${REF_SOURCES})
  target_link_libraries(${_target} PRIVATE bike_golden_test ${REF_LIBRARIES})
  target_include_directories(${_target} PRIVATE "${CMAKE_SOURCE_DIR}/native/src")
  if(REF_ENGINE)
    bike_startup_target_policy(${_target} "${CMAKE_SOURCE_DIR}/native")
    bike_rewrite_mujoco_load_name(${_target} "${BIKE_DEP_mujoco_LIBRARY}")
  else()
    bike_apply_native_policy(${_target})
  endif()
  add_test(NAME ${_target} COMMAND ${_target})
  set_tests_properties(${_target} PROPERTIES LABELS "reference;${REF_LABELS}"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_target}>")
endfunction()

bike_add_reference_test(golden_support SOURCES test_golden_support.cpp LIBRARIES bike_numpy_io)
bike_add_reference_test(golden_episode_codec ENGINE SOURCES test_golden_episode_codec.cpp
  LIBRARIES bike_golden bike_recording_cli LABELS golden)
bike_add_reference_test(golden_episode_tool ENGINE SOURCES test_golden_episode_tool.cpp
  LIBRARIES bike_golden LABELS golden slow)

include("${CMAKE_CURRENT_SOURCE_DIR}/Phase3Ports.cmake")
