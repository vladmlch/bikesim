# JSON emit helpers shared by the native_check_context.json writer
# (BikeCompilerPolicy.cmake, BikeCheckContext.cmake) and the numerics provider
# identity (BikeProviderIdentity.cmake). The Python-free root graph includes
# this module without the legacy compiler policy.
#
#   _native_json_quote(<out> <value>)   a JSON string literal of <value>
#   _native_json_array(<out> <values>...)  a JSON array of string literals
#
# This module executes no code at include() time.
include_guard(GLOBAL)

function(_native_json_quote output value)
  string(REPLACE "\\" "\\\\" _escaped "${value}")
  string(REPLACE "\"" "\\\"" _escaped "${_escaped}")
  string(REPLACE "\n" "\\n" _escaped "${_escaped}")
  string(REPLACE "\r" "\\r" _escaped "${_escaped}")
  set(${output} "\"${_escaped}\"" PARENT_SCOPE)
endfunction()

function(_native_json_array output)
  set(_json "[")
  set(_separator "")
  foreach(_value IN LISTS ARGN)
    _native_json_quote(_quoted "${_value}")
    string(APPEND _json "${_separator}${_quoted}")
    set(_separator ",")
  endforeach()
  string(APPEND _json "]")
  set(${output} "${_json}" PARENT_SCOPE)
endfunction()
