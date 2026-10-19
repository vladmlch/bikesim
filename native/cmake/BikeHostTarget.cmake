# Host dispatch shared by BikePlatform.cmake (legacy graph) and the root
# CMakeLists.txt (Python-free graph).
#
# The native build qualifies one host profile per platform, and its numerics
# records describe the host's own provider. A cross-compiling configure would
# bind that host's record facts to a different target system, so it is refused
# here, before any target can be built.
#
#   bike_host_target_system(<out_var>)  refuses CMAKE_SYSTEM_NAME different from
#                                       the host, then sets <out_var> in the
#                                       caller's scope to the host system name.

include_guard(GLOBAL)

function(bike_host_target_system out_var)
  if(DEFINED CMAKE_SYSTEM_NAME AND
     NOT CMAKE_SYSTEM_NAME STREQUAL CMAKE_HOST_SYSTEM_NAME)
    message(FATAL_ERROR "Cross-compilation is not supported by this native-build workflow")
  endif()
  set(${out_var} "${CMAKE_HOST_SYSTEM_NAME}" PARENT_SCOPE)
endfunction()
