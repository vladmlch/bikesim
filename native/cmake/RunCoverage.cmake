cmake_minimum_required(VERSION 3.27)
foreach(_required BUILD_DIR CONFIG CTEST LLVM_COV LLVM_PROFDATA)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "RunCoverage requires ${_required}")
  endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
set(_profiles "${_build}/profiles")
if(IS_SYMLINK "${_profiles}")
  message(FATAL_ERROR "Refusing a symlinked coverage profile directory")
endif()
file(REMOVE_RECURSE "${_profiles}")
file(MAKE_DIRECTORY "${_profiles}")
file(REMOVE "${_build}/coverage.profdata" "${_build}/coverage.lcov" "${_build}/coverage-summary.txt")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "LLVM_PROFILE_FILE=${_profiles}/%p-%m.profraw"
    "${CTEST}" --test-dir "${_build}" -C "${CONFIG}" -j8
    --no-tests=error --output-on-failure --output-junit coverage-junit.xml
    -LE "coverage|package|audit|slow" -E "^install_smoke$"
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "coverage CTest run failed (${_rc}); no successful report is published")
endif()
file(GLOB _raw "${_profiles}/*.profraw")
if(NOT _raw)
  message(FATAL_ERROR "The test run produced no raw coverage profiles")
endif()
execute_process(COMMAND "${LLVM_PROFDATA}" merge -sparse ${_raw}
    -o "${_build}/coverage.profdata"
  RESULT_VARIABLE _rc ERROR_VARIABLE _error)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Coverage merge failed (${_rc}): ${_error}")
endif()
file(STRINGS "${_build}/coverage-binaries-${CONFIG}.txt" _binaries)
if(NOT _binaries)
  message(FATAL_ERROR "Coverage binary manifest is empty")
endif()
list(POP_FRONT _binaries _primary)
set(_objects)
foreach(_binary IN LISTS _binaries)
  if(NOT EXISTS "${_binary}")
    message(FATAL_ERROR "Instrumented binary is missing: ${_binary}")
  endif()
  list(APPEND _objects -object "${_binary}")
endforeach()
execute_process(COMMAND "${LLVM_COV}" export "${_primary}" ${_objects}
    "-instr-profile=${_build}/coverage.profdata" -format=lcov
  OUTPUT_FILE "${_build}/coverage.lcov.partial" ERROR_VARIABLE _error RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Coverage export failed (${_rc}): ${_error}")
endif()
execute_process(COMMAND "${LLVM_COV}" report "${_primary}" ${_objects}
    "-instr-profile=${_build}/coverage.profdata"
  OUTPUT_FILE "${_build}/coverage-summary.txt.partial" ERROR_VARIABLE _error RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Coverage summary failed (${_rc}): ${_error}")
endif()
file(RENAME "${_build}/coverage.lcov.partial" "${_build}/coverage.lcov")
file(RENAME "${_build}/coverage-summary.txt.partial" "${_build}/coverage-summary.txt")
execute_process(COMMAND "${CTEST}" --test-dir "${_build}" -C "${CONFIG}"
    -R "^coverage_smoke$" --no-tests=error --output-on-failure
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "coverage_smoke failed (${_rc})")
endif()
