# Idempotent install-name rewrite for one linked binary (Darwin only —
# BikeLinkage.cmake never invokes this elsewhere).
#
# Required -D arguments: BIKE_OTOOL, BIKE_INSTALL_NAME_TOOL,
# BIKE_OLD_NAME (the install name recorded by the selected engine image),
# BIKE_NEW_NAME (its absolute path), BIKE_BINARY (the built artifact).
#
# Semantics: inspect `otool -L`; rewrite only when BIKE_OLD_NAME is present.
#   OLD present      -> install_name_tool -change OLD NEW, then re-inspect:
#                       the new name must be recorded (verifies the
#                       -headerpad_max_install_names reservation held).
#   OLD absent, NEW present -> already rewritten: a successful no-op.
#   Neither present  -> the binary does not reference the engine image via
#                       either name — the link contract changed silently;
#                       fail closed.

foreach(required BIKE_OTOOL BIKE_INSTALL_NAME_TOOL BIKE_OLD_NAME BIKE_NEW_NAME BIKE_BINARY)
  if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
    message(FATAL_ERROR "BikeDarwinRewriteInstallName: missing -D${required}")
  endif()
endforeach()

execute_process(
  COMMAND "${BIKE_OTOOL}" -L "${BIKE_BINARY}"
  RESULT_VARIABLE _otool_result
  OUTPUT_VARIABLE _otool_output
  ERROR_VARIABLE _otool_error)
if(NOT _otool_result EQUAL 0)
  message(FATAL_ERROR "otool -L failed on ${BIKE_BINARY}: ${_otool_error}")
endif()

string(FIND "${_otool_output}" "${BIKE_OLD_NAME}" _old_position)
if(_old_position EQUAL -1)
  string(FIND "${_otool_output}" "${BIKE_NEW_NAME}" _new_position)
  if(_new_position EQUAL -1)
    message(FATAL_ERROR
      "${BIKE_BINARY} references neither the configured MuJoCo install "
      "name '${BIKE_OLD_NAME}' nor the selected image '${BIKE_NEW_NAME}'")
  endif()
  return()
endif()

execute_process(
  COMMAND "${BIKE_INSTALL_NAME_TOOL}" -change
    "${BIKE_OLD_NAME}" "${BIKE_NEW_NAME}" "${BIKE_BINARY}"
  RESULT_VARIABLE _tool_result
  OUTPUT_VARIABLE _tool_output
  ERROR_VARIABLE _tool_error)
if(NOT _tool_result EQUAL 0)
  message(FATAL_ERROR
    "install_name_tool -change failed on ${BIKE_BINARY}: "
    "${_tool_error}${_tool_output}")
endif()

execute_process(
  COMMAND "${BIKE_OTOOL}" -L "${BIKE_BINARY}"
  RESULT_VARIABLE _verify_result
  OUTPUT_VARIABLE _verify_output
  ERROR_VARIABLE _verify_error)
if(NOT _verify_result EQUAL 0)
  message(FATAL_ERROR
    "otool -L verification failed on ${BIKE_BINARY}: ${_verify_error}")
endif()
string(FIND "${_verify_output}" "${BIKE_NEW_NAME}" _verified_position)
if(_verified_position EQUAL -1)
  message(FATAL_ERROR
    "install_name_tool ran on ${BIKE_BINARY} but '${BIKE_NEW_NAME}' is "
    "not recorded — the header padding reservation did not hold")
endif()
