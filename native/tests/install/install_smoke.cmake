cmake_minimum_required(VERSION 3.27)

# Full-product install contract. No application simulation is involved:
# inspect every copied runtime image and exercise every installed CLI's help.
foreach(_required BUILD_DIR SCRATCH_DIR DEPENDENCY_ROOT BIKE_TARGET_SYSTEM)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "install_smoke requires -D${_required}")
  endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
cmake_path(ABSOLUTE_PATH SCRATCH_DIR NORMALIZE OUTPUT_VARIABLE _scratch)
set(_expected_scratch "${_build}/install_smoke")
if(NOT _scratch STREQUAL _expected_scratch OR IS_SYMLINK "${_scratch}")
  message(FATAL_ERROR "install_smoke scratch must be the build's own install_smoke directory")
endif()
file(REMOVE_RECURSE "${_scratch}")
file(MAKE_DIRECTORY "${_scratch}")

set(_config_args)
if(DEFINED INSTALL_CONFIG AND NOT INSTALL_CONFIG STREQUAL "")
  list(APPEND _config_args --config "${INSTALL_CONFIG}")
endif()
# A focused T03 build installs only what was built. This separate smoke must
# first produce the entire graph so a missing application cannot be skipped.
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${_build}" ${_config_args}
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _output ERROR_VARIABLE _error)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "install_smoke build failed (${_rc}):\n${_output}${_error}")
endif()
set(_original_prefix "${_scratch}/prefix")
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${_build}"
    --prefix "${_original_prefix}" ${_config_args}
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _output ERROR_VARIABLE _error)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "install_smoke install failed (${_rc}):\n${_output}${_error}")
endif()

# Move the finished tree before looking at its images or executing anything.
# An absolute install-prefix load path cannot accidentally satisfy the smoke.
set(_prefix "${_scratch}/relocated")
file(RENAME "${_original_prefix}" "${_prefix}" RESULT _rename_rc)
if(NOT _rename_rc STREQUAL "0")
  message(FATAL_ERROR "Cannot relocate install tree: ${_rename_rc}")
endif()
foreach(_example research/viewer_physics_welded.toml research/rough_uphill_savage.toml)
  if(NOT EXISTS "${_prefix}/share/bike_sim/examples/${_example}")
    message(FATAL_ERROR "Missing installed example ${_example}")
  endif()
endforeach()
foreach(_metadata dependency_provenance.json dependencies.lock.json)
  if(NOT EXISTS "${_prefix}/share/bike_sim/${_metadata}")
    message(FATAL_ERROR "Missing installed metadata ${_metadata}")
  endif()
endforeach()

if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  find_program(_inspect NAMES otool REQUIRED)
  set(_inspect_option -l)
  set(_want_rpath "@loader_path/../lib")
elseif(BIKE_TARGET_SYSTEM STREQUAL "Linux")
  find_program(_inspect NAMES readelf REQUIRED)
  set(_inspect_option -d)
  set(_want_rpath "$ORIGIN/../lib")
else()
  message(FATAL_ERROR "Unsupported install-smoke platform ${BIKE_TARGET_SYSTEM}")
endif()

function(_inspect_installed _path _executable)
  execute_process(COMMAND "${_inspect}" "${_inspect_option}" "${_path}"
    RESULT_VARIABLE _read_rc OUTPUT_VARIABLE _loads ERROR_VARIABLE _read_error)
  if(NOT _read_rc EQUAL 0)
    message(FATAL_ERROR "Cannot inspect ${_path}: ${_read_error}")
  endif()
  foreach(_forbidden "${DEPENDENCY_ROOT}" "${_original_prefix}" "/.deps/" "/site-packages/" "/.venv/")
    string(FIND "${_loads}" "${_forbidden}" _bad_at)
    if(NOT _bad_at EQUAL -1)
      message(FATAL_ERROR "${_path} retains a load path into ${_forbidden}:\n${_loads}")
    endif()
  endforeach()
  if(_loads MATCHES "libpython")
    message(FATAL_ERROR "${_path} has an unexpected Python library dependency")
  endif()
  if(_executable)
    string(FIND "${_loads}" "${_want_rpath}" _rpath_at)
    if(_rpath_at EQUAL -1)
      message(FATAL_ERROR "${_path} lacks install rpath ${_want_rpath}")
    endif()
  endif()
endfunction()

file(GLOB _runtime_libraries "${_prefix}/lib/*")
if(NOT _runtime_libraries)
  message(FATAL_ERROR "The install tree contains no runtime libraries")
endif()
foreach(_library IN LISTS _runtime_libraries)
  _inspect_installed("${_library}" FALSE)
endforeach()

set(_entry_points bike-sim bike-playground bike-export bike-ride bike-settle
  bike-research bike-replay bike-batch bike-recording bike-research-sweep bike-native-ride)
foreach(_name IN LISTS _entry_points)
  set(_binary "${_prefix}/bin/${_name}")
  if(NOT EXISTS "${_binary}")
    message(FATAL_ERROR "Missing installed entry point ${_name}")
  endif()
  _inspect_installed("${_binary}" TRUE)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env
      --unset=DYLD_LIBRARY_PATH --unset=DYLD_FALLBACK_LIBRARY_PATH
      --unset=DYLD_INSERT_LIBRARIES --unset=LD_LIBRARY_PATH --unset=LD_PRELOAD
      "${_binary}" --help
    WORKING_DIRECTORY "${_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _help ERROR_VARIABLE _error
    TIMEOUT 30)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "${_name} --help failed (${_rc}):\n${_help}${_error}")
  endif()
endforeach()
message(STATUS "Relocated install: all 11 entry points passed help with inspected load paths")
