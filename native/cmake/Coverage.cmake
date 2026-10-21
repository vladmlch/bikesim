# Completion plan 3.43. No instrumentation flags or tool probing here:
# BikeCompilerPolicy owns both, with qualified LLVM paths in the cache.
include_guard(GLOBAL)
if(NOT NATIVE_COVERAGE)
  return()
endif()
if(NOT BUILD_TESTING OR NOT BIKE_LLVM_COV OR NOT BIKE_LLVM_PROFDATA)
  message(FATAL_ERROR "coverage requires BUILD_TESTING and the qualified LLVM tools")
endif()

function(_bike_coverage_targets directory out_targets)
  get_property(_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  get_property(_children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(_child IN LISTS _children)
    _bike_coverage_targets("${_child}" _nested)
    list(APPEND _targets ${_nested})
  endforeach()
  set(${out_targets} "${_targets}" PARENT_SCOPE)
endfunction()
_bike_coverage_targets("${CMAKE_SOURCE_DIR}" _coverage_targets)
set(_coverage_binaries)
set(_coverage_dependencies)
foreach(_target IN LISTS _coverage_targets)
  get_target_property(_type "${_target}" TYPE)
  if(_type MATCHES "^(EXECUTABLE|SHARED_LIBRARY|MODULE_LIBRARY)$")
    list(APPEND _coverage_dependencies "${_target}")
    list(APPEND _coverage_binaries "$<TARGET_FILE:${_target}>")
  endif()
endforeach()
if(NOT _coverage_binaries)
  message(FATAL_ERROR "coverage found no instrumented executable/shared-library targets")
endif()
string(JOIN "\n" _coverage_binary_lines ${_coverage_binaries})
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/coverage-binaries-$<CONFIG>.txt"
  CONTENT "${_coverage_binary_lines}\n")
add_custom_target(coverage
  COMMAND "${CMAKE_COMMAND}"
    "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
    "-DCONFIG=$<CONFIG>"
    "-DCTEST=${CMAKE_CTEST_COMMAND}"
    "-DLLVM_COV=${BIKE_LLVM_COV}"
    "-DLLVM_PROFDATA=${BIKE_LLVM_PROFDATA}"
    -P "${CMAKE_SOURCE_DIR}/native/cmake/RunCoverage.cmake"
  DEPENDS ${_coverage_dependencies}
  USES_TERMINAL VERBATIM)
add_test(NAME coverage_smoke COMMAND "${CMAKE_COMMAND}"
  "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
  -P "${CMAKE_SOURCE_DIR}/native/tests/coverage/coverage_smoke.cmake")
set_tests_properties(coverage_smoke PROPERTIES LABELS "coverage" RUN_SERIAL TRUE)
