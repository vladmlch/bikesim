# Native-only spatial contract tests. Fixtures are data, never configure-time
# generated simulation output. Pending identities remain test failures in V01.
set(BIKE_SPATIAL_SOURCE_ROOT "${CMAKE_SOURCE_DIR}")
set(BIKE_SPATIAL_FIXTURE_ROOT "${CMAKE_SOURCE_DIR}/tests/fixtures/spatial3d")
foreach(_spatial_path IN ITEMS "${BIKE_SPATIAL_SOURCE_ROOT}" "${BIKE_SPATIAL_FIXTURE_ROOT}")
  string(FIND "${_spatial_path}" "\"" _spatial_quote)
  string(FIND "${_spatial_path}" "\\" _spatial_backslash)
  string(FIND "${_spatial_path}" "\n" _spatial_newline)
  string(FIND "${_spatial_path}" "\r" _spatial_carriage_return)
  if(NOT _spatial_quote EQUAL -1 OR NOT _spatial_backslash EQUAL -1
      OR NOT _spatial_newline EQUAL -1 OR NOT _spatial_carriage_return EQUAL -1)
    message(FATAL_ERROR "Spatial source/fixture path cannot be embedded as a C++ string")
  endif()
endforeach()
configure_file(fixture_paths.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/fixture_paths.hpp" @ONLY)

add_library(bike_spatial_support STATIC support/spatial_fixture.cpp)
bike_apply_native_policy(bike_spatial_support)
target_include_directories(bike_spatial_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}/native/tests" "${CMAKE_SOURCE_DIR}/native/src")
target_link_libraries(bike_spatial_support PUBLIC bike_io)

function(bike_add_spatial_test slug)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES;LABELS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "bike_add_spatial_test(${slug}): unrecognized arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_SOURCES)
    message(FATAL_ERROR "bike_add_spatial_test(${slug}) requires nonempty SOURCES")
  endif()
  add_executable(bike_spatial_${slug} ${ARG_SOURCES})
  target_link_libraries(bike_spatial_${slug} PRIVATE
    bike_spatial_support bike_migration_test bike_io ${ARG_LIBRARIES})
  target_include_directories(bike_spatial_${slug} SYSTEM PRIVATE
    "${BIKE_DEP_mujoco_ROOT}/include" "${BIKE_DEP_eigen_ROOT}")
  bike_apply_native_policy(bike_spatial_${slug})
  add_test(NAME spatial.${slug} COMMAND bike_spatial_${slug})
  set_tests_properties(spatial.${slug} PROPERTIES LABELS "spatial;${ARG_LABELS}")
endfunction()

bike_add_spatial_test(harness SOURCES test_harness.cpp LIBRARIES bike_platform LABELS foundation unit)
bike_add_spatial_test(baseline SOURCES test_baseline.cpp LABELS foundation audit)
bike_add_spatial_test(references SOURCES test_references.cpp LABELS foundation unit)
bike_add_spatial_test(contracts_registry SOURCES test_contracts_registry.cpp LABELS foundation unit)
bike_add_spatial_test(config SOURCES test_config.cpp
  LIBRARIES bike_configuration bike_initialization LABELS foundation unit)
# project_config.cpp belongs to bike_initialization, whose startup-runtime
# interface supplies the existing core objects and engine exactly once.
bike_rewrite_mujoco_load_name(bike_spatial_config "${BIKE_DEP_mujoco_LIBRARY}")
