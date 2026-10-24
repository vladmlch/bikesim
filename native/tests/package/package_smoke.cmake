cmake_minimum_required(VERSION 3.27)
foreach(_name BUILD_DIR BINARY_PACKAGE SOURCE_PACKAGE AUDIT_BINARY AUDIT_POLICY CONFIG)
  if(NOT DEFINED ${_name} OR "${${_name}}" STREQUAL "")
    message(FATAL_ERROR "package_smoke requires ${_name}")
  endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
set(_scratch "${_build}/package_smoke")
if(IS_SYMLINK "${_scratch}")
  message(FATAL_ERROR "Refusing a symlinked package-smoke directory")
endif()
file(REMOVE_RECURSE "${_scratch}")
file(MAKE_DIRECTORY "${_scratch}/packages" "${_scratch}/binary" "${_scratch}/source")
find_program(_cpack NAMES cpack REQUIRED)
foreach(_kind binary source)
  if(_kind STREQUAL "binary")
    set(_configuration CPackConfig.cmake)
    set(_basename "${BINARY_PACKAGE}")
  else()
    set(_configuration CPackSourceConfig.cmake)
    set(_basename "${SOURCE_PACKAGE}")
  endif()
  execute_process(COMMAND "${_cpack}" --config "${_build}/${_configuration}"
      -G TGZ -C "${CONFIG}" -B "${_scratch}/packages"
    WORKING_DIRECTORY "${_build}" RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "${_kind} CPack failed (${_rc}): ${_out}${_err}")
  endif()
  set(_archive "${_scratch}/packages/${_basename}.tar.gz")
  if(NOT EXISTS "${_archive}")
    message(FATAL_ERROR "Missing expected ${_kind} package ${_archive}")
  endif()
  execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xf "${_archive}"
    WORKING_DIRECTORY "${_scratch}/${_kind}" RESULT_VARIABLE _rc ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "Cannot extract ${_kind} package: ${_err}")
  endif()
endforeach()
set(_prefix "${_scratch}/binary/${BINARY_PACKAGE}")
set(_source "${_scratch}/source/${SOURCE_PACKAGE}")
foreach(_required bike-sim bike-playground bike-export bike-ride bike-settle bike-research
    bike-replay bike-batch bike-recording bike-research-sweep bike-native-ride)
  if(NOT EXISTS "${_prefix}/bin/${_required}")
    message(FATAL_ERROR "Package omits installed entry point ${_required}")
  endif()
endforeach()
set(_clean_env "${CMAKE_COMMAND}" -E env --unset=DYLD_LIBRARY_PATH
  --unset=DYLD_FALLBACK_LIBRARY_PATH --unset=DYLD_INSERT_LIBRARIES
  --unset=LD_LIBRARY_PATH --unset=LD_PRELOAD)
file(GLOB _programs "${_prefix}/bin/bike-*")
foreach(_program IN LISTS _programs)
  execute_process(COMMAND ${_clean_env} "${_program}" --help
    WORKING_DIRECTORY "${_prefix}" RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 30)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "Packaged ${_program} --help failed (${_rc}): ${_out}${_err}")
  endif()
endforeach()
# --check-policy diagnostic keeps the run non-strict so the packaged binary
# still completes the requested duration instead of aborting at step 4 on the
# platform's documented energy.constraint_work baseline violation
# (docs/RIDE.md, docs/development/macos.md — identical on the Python oracle).
# ride.cpp then exits 2: the documented diagnostic code for a completed run
# whose model_status latches invalid. Qualify that code — and only that
# code — by requiring the summary to prove a full duration_reached run whose
# first failure is the documented baseline; every other outcome stays fatal.
execute_process(COMMAND ${_clean_env} "${_prefix}/bin/bike-ride"
    --backend native --physics-config share/bike_sim/examples/research/viewer_physics_welded.toml
    --track share/bike_sim/examples/research/rough_uphill_savage.toml
    --headless --duration 1 --no-plots --check-policy diagnostic
  WORKING_DIRECTORY "${_prefix}" RESULT_VARIABLE _rc
  OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 120)
file(WRITE "${_scratch}/headless-ride.log" "exit=${_rc}\n${_out}${_err}")
if(_rc EQUAL 2)
  string(JSON _ride_reason ERROR_VARIABLE _json_error GET "${_out}" outcome reason)
  string(JSON _ride_failure ERROR_VARIABLE _json_failure GET "${_out}" first_failure)
  if(_json_error OR _json_failure OR NOT _ride_reason STREQUAL "duration_reached"
     OR NOT _ride_failure MATCHES "energy.constraint_work")
    message(FATAL_ERROR "Packaged headless ride exited 2 without the documented baseline shape: ${_out}${_err}")
  endif()
  message(STATUS "Packaged headless ride retained the documented energy.constraint_work baseline after a full duration_reached run")
elseif(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Packaged headless ride failed (${_rc}): ${_out}${_err}")
endif()
execute_process(COMMAND "${AUDIT_BINARY}" --mode package --root "${_prefix}" --policy "${AUDIT_POLICY}"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 120)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Packaged Python audit failed (${_rc}): ${_out}${_err}")
endif()
file(STRINGS "${_build}/package-archive-names.txt" _expected)
file(GLOB_RECURSE _actual LIST_DIRECTORIES FALSE RELATIVE "${_source}/.deps" "${_source}/.deps/*")
list(SORT _expected)
list(SORT _actual)
if(NOT _actual STREQUAL _expected)
  message(FATAL_ERROR "Source .deps must contain exactly the selected archives; expected ${_expected}; got ${_actual}")
endif()
file(STRINGS "${_build}/package-archive-sha256.txt" _hash_lines)
foreach(_line IN LISTS _hash_lines)
  if(NOT _line MATCHES "^([0-9a-f]+)  (_archives/[^/]+)$")
    message(FATAL_ERROR "Malformed package archive digest manifest")
  endif()
  set(_expected_digest "${CMAKE_MATCH_1}")
  set(_relative "${CMAKE_MATCH_2}")
  file(SHA256 "${_source}/.deps/${_relative}" _digest)
  if(NOT _digest STREQUAL _expected_digest)
    message(FATAL_ERROR "Source archive digest changed after configure: ${_relative}")
  endif()
endforeach()
foreach(_forbidden build .venv .git output .worktrees native/build .scratch .work)
  if(EXISTS "${_source}/${_forbidden}")
    message(FATAL_ERROR "Source package contains generated/private tree ${_forbidden}")
  endif()
endforeach()
if(NOT EXISTS "${_source}/tools/linux/extract_deps.sh")
  message(FATAL_ERROR "Source package omits the offline dependency extractor")
endif()
set(SPATIAL_INSTALL_PREFIX "${_prefix}")
include("${CMAKE_CURRENT_LIST_DIR}/../spatial/installed_product.cmake")
message(STATUS "Binary and source package contracts passed; offline rebuild is a separate required lane")
