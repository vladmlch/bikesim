# Portable linkage helpers for the native target graph.
#
# bike_link_engine(target [PUBLIC])
#   Links Bike::MuJoCo plus the platform dynamic-loader library
#   (${CMAKE_DL_LIBS} — "dl" on Linux, empty on Darwin) for every binary
#   whose sources call dladdr or the MuJoCo API (extension/contract paths).
#   The optional PUBLIC scope is for bike_native_engine: consumers link its
#   objects, so the engine image and -ldl are part of its interface. On
#   Linux the engine image's own directory is appended to BUILD_RPATH of
#   linked runtime artifacts so the .so resolves at exec time; environment-
#   rescue variables (LD_LIBRARY_PATH) are never required by the built
#   artifacts.
#
# bike_link_numerics(target)
#   Links Bike::Numerics (the typed CBLAS/LAPACK adapter, which carries the
#   qualified provider through PUBLIC usage requirements) and, on Linux,
#   appends the resolved provider directory to BUILD_RPATH so the wheel's
#   scipy-openblas .so resolves at exec time without environment rescue.
#   Absolute environment paths in BUILD_RPATH are a developer-build truth,
#   not a relocation claim: a relocated checkout re-runs the dependency
#   probe and rebuilds against that environment.
#
# bike_fix_mujoco_install_name(target)
#   Darwin only. The wheel ships the dylib flat under mujoco/, so the
#   install name recorded inside it (@rpath/mujoco.framework/...) has no
#   framework anchor to resolve against. The linked binary records that
#   unresolvable name; the post-build step inspects `otool -L` output and
#   rewrites it to the selected absolute path — but only when the recorded
#   name is actually present, so the action is deterministic and
#   idempotent across rebuilds. install_name_tool and otool are resolved
#   once here via find_program; -headerpad_max_install_names (applied by
#   the compiler policy on Darwin) reserves the header room the longer
#   name needs. On Linux the function is a deliberate no-op: Apple-only
#   tooling never runs.
#
# Prerequisites for the includer:
#   BIKE_TARGET_SYSTEM        set by BikePlatform.cmake
#   Bike::MuJoCo / MJ_DIR /
#   _native_mujoco_library    set by BikeDependencies.cmake
#   Bike::Numerics /
#   _numerics_provider        set by BikeNumerics.cmake (Linux file
#                             provider; empty under Accelerate)

if(NOT DEFINED BIKE_TARGET_SYSTEM)
  message(FATAL_ERROR
    "BikeLinkage.cmake requires BIKE_TARGET_SYSTEM (see BikePlatform.cmake)")
endif()
if(NOT DEFINED _native_mujoco_library OR NOT EXISTS "${_native_mujoco_library}")
  message(FATAL_ERROR
    "BikeLinkage.cmake requires _native_mujoco_library from "
    "BikeDependencies.cmake")
endif()

# BUILD_RPATH entries are only meaningful on artifacts with a dynamic
# loader pass — executables and shared/module libraries. Appending one to a
# static or object library would record a runtime claim that can never be
# exercised.
function(_bike_target_is_runtime_artifact target out_var)
  get_target_property(_bike_target_type "${target}" TYPE)
  if(_bike_target_type MATCHES "^(EXECUTABLE|SHARED_LIBRARY|MODULE_LIBRARY)$")
    set(${out_var} TRUE PARENT_SCOPE)
  else()
    set(${out_var} FALSE PARENT_SCOPE)
  endif()
endfunction()

function(bike_link_engine target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "bike_link_engine: '${target}' is not a CMake target")
  endif()
  cmake_parse_arguments(_bike_link "PUBLIC" "" "" ${ARGN})
  if(_bike_link_PUBLIC)
    set(_bike_link_scope PUBLIC)
  else()
    set(_bike_link_scope PRIVATE)
  endif()
  target_link_libraries(${target} ${_bike_link_scope}
    Bike::MuJoCo ${CMAKE_DL_LIBS})
  _bike_target_is_runtime_artifact(${target} _bike_is_runtime)
  if(BIKE_TARGET_SYSTEM STREQUAL "Linux" AND _bike_is_runtime)
    # $ORIGIN stays legal in any future relative RPATH entries; the current
    # entries are absolute selected-environment paths it does not alter.
    set_property(TARGET ${target} PROPERTY BUILD_RPATH_USE_ORIGIN TRUE)
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${MJ_DIR}")
  endif()
endfunction()

function(bike_link_numerics target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR
      "bike_link_numerics: '${target}' is not a CMake target")
  endif()
  target_link_libraries(${target} PRIVATE Bike::Numerics)
  _bike_target_is_runtime_artifact(${target} _bike_is_runtime)
  if(BIKE_TARGET_SYSTEM STREQUAL "Linux" AND _bike_is_runtime)
    set_property(TARGET ${target} PROPERTY BUILD_RPATH_USE_ORIGIN TRUE)
  endif()
  if(BIKE_TARGET_SYSTEM STREQUAL "Linux" AND _bike_is_runtime
     AND DEFINED _numerics_provider AND EXISTS "${_numerics_provider}")
    get_filename_component(_numerics_provider_dir "${_numerics_provider}"
      DIRECTORY)
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH
      "${_numerics_provider_dir}")
  endif()
endfunction()

if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  find_program(BIKE_INSTALL_NAME_TOOL NAMES install_name_tool REQUIRED)
  find_program(BIKE_OTOOL NAMES otool REQUIRED)
  # The install name the selected engine image records inside itself —
  # measured from the image, never guessed: this is the exact string the
  # linker copies into every binary that links the library.
  execute_process(
    COMMAND "${BIKE_OTOOL}" -D "${_native_mujoco_library}"
    RESULT_VARIABLE _bike_install_name_result
    OUTPUT_VARIABLE _bike_install_name_output
    ERROR_VARIABLE _bike_install_name_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT _bike_install_name_result EQUAL 0)
    message(FATAL_ERROR
      "otool -D failed on ${_native_mujoco_library}: "
      "${_bike_install_name_error}")
  endif()
  # otool -D output is "<image path>:\n<install name>".
  string(REGEX MATCH "\n[^\n]+$" _bike_mujoco_install_name
    "${_bike_install_name_output}")
  string(STRIP "${_bike_mujoco_install_name}" _bike_mujoco_install_name)
  if(_bike_mujoco_install_name STREQUAL "")
    message(FATAL_ERROR
      "otool -D reported no install name for ${_native_mujoco_library}: "
      "'${_bike_install_name_output}'")
  endif()
endif()

function(bike_fix_mujoco_install_name target)
  if(NOT BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    return()
  endif()
  if(NOT TARGET "${target}")
    message(FATAL_ERROR
      "bike_fix_mujoco_install_name: '${target}' is not a CMake target")
  endif()
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND "${CMAKE_COMMAND}"
      -DBIKE_OTOOL=${BIKE_OTOOL}
      -DBIKE_INSTALL_NAME_TOOL=${BIKE_INSTALL_NAME_TOOL}
      -DBIKE_OLD_NAME=${_bike_mujoco_install_name}
      -DBIKE_NEW_NAME=${_native_mujoco_library}
      "-DBIKE_BINARY=$<TARGET_FILE:${target}>"
      -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/BikeDarwinRewriteInstallName.cmake"
    VERBATIM)
endfunction()
