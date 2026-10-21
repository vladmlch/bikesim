# Install-only Darwin relocation. This file never touches the locked .deps
# image or any build-tree executable. The caller includes it after copying
# the artifact and after CMake has applied its executable INSTALL_RPATH.
foreach(_required BIKE_BINARY BIKE_OTOOL BIKE_INSTALL_NAME_TOOL BIKE_CODESIGN
    BIKE_OLD_NAME BIKE_NEW_NAME)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "Darwin install relocation is missing ${_required}")
  endif()
endforeach()
if(NOT EXISTS "${BIKE_BINARY}")
  message(FATAL_ERROR "Installed artifact is missing: ${BIKE_BINARY}")
endif()

if(BIKE_RELOCATE_LIBRARY)
  execute_process(COMMAND "${BIKE_OTOOL}" -D "${BIKE_BINARY}"
    RESULT_VARIABLE _read_rc OUTPUT_VARIABLE _identity ERROR_VARIABLE _error)
  if(NOT _read_rc EQUAL 0)
    message(FATAL_ERROR "Cannot read installed MuJoCo ID: ${_error}")
  endif()
  string(REGEX MATCH "\n[^\n]+\n?$" _identity "${_identity}")
  string(STRIP "${_identity}" _identity)
  if(NOT _identity STREQUAL BIKE_NEW_NAME)
    execute_process(COMMAND "${BIKE_INSTALL_NAME_TOOL}" -id "${BIKE_NEW_NAME}" "${BIKE_BINARY}"
      RESULT_VARIABLE _rewrite_rc ERROR_VARIABLE _error)
    if(NOT _rewrite_rc EQUAL 0)
      message(FATAL_ERROR "Cannot relocate installed MuJoCo ID: ${_error}")
    endif()
    execute_process(COMMAND "${BIKE_OTOOL}" -D "${BIKE_BINARY}"
      RESULT_VARIABLE _read_rc OUTPUT_VARIABLE _identity ERROR_VARIABLE _error)
    string(REGEX MATCH "\n[^\n]+\n?$" _identity "${_identity}")
    string(STRIP "${_identity}" _identity)
    if(NOT _read_rc EQUAL 0 OR NOT _identity STREQUAL BIKE_NEW_NAME)
      message(FATAL_ERROR "Installed MuJoCo ID did not become ${BIKE_NEW_NAME}: ${_error}")
    endif()
  endif()
else()
  execute_process(COMMAND "${BIKE_OTOOL}" -L "${BIKE_BINARY}"
    RESULT_VARIABLE _read_rc OUTPUT_VARIABLE _loads ERROR_VARIABLE _error)
  if(NOT _read_rc EQUAL 0)
    message(FATAL_ERROR "Cannot inspect installed executable ${BIKE_BINARY}: ${_error}")
  endif()
  string(FIND "${_loads}" "${BIKE_OLD_NAME}" _old_at)
  string(FIND "${_loads}" "${BIKE_NEW_NAME}" _new_at)
  if(NOT _old_at EQUAL -1 OR NOT _new_at EQUAL -1)
    include("${CMAKE_CURRENT_LIST_DIR}/BikeDarwinRewriteInstallName.cmake")
  elseif(_loads MATCHES "libmujoco")
    message(FATAL_ERROR
      "Installed executable has an unexpected MuJoCo load name: ${BIKE_BINARY}\n${_loads}")
  endif()
  # Some installed commands do not link MuJoCo. They still need their
  # relative rpath for DuckDB/OpenBLAS, but have no engine name to rewrite.
endif()

# install_name_tool and CMake's own rpath edit can invalidate an existing
# ad-hoc signature. Repair only the installed artifact if verification says
# it needs signing; no launch failure is hidden or converted to success.
execute_process(COMMAND "${BIKE_CODESIGN}" --verify --strict "${BIKE_BINARY}"
  RESULT_VARIABLE _signature_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT _signature_rc EQUAL 0)
  execute_process(COMMAND "${BIKE_CODESIGN}" --force --sign - "${BIKE_BINARY}"
    RESULT_VARIABLE _sign_rc OUTPUT_VARIABLE _sign_output ERROR_VARIABLE _sign_error)
  if(NOT _sign_rc EQUAL 0)
    message(FATAL_ERROR "Cannot sign relocated artifact ${BIKE_BINARY}: ${_sign_output}${_sign_error}")
  endif()
  execute_process(COMMAND "${BIKE_CODESIGN}" --verify --strict "${BIKE_BINARY}"
    RESULT_VARIABLE _signature_rc OUTPUT_QUIET ERROR_VARIABLE _sign_error)
  if(NOT _signature_rc EQUAL 0)
    message(FATAL_ERROR "Relocated artifact signature is invalid: ${BIKE_BINARY}: ${_sign_error}")
  endif()
endif()
