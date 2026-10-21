# native/cmake/Checks.cmake — T56 produced check orchestration.
#
# Included once at the end of the root CMakeLists (standalone/Python-free
# build). Provides:
#   * bike-native-checks / bike-python-audit executables (bin/)
#   * native_sources.json + native_check_context.json at the build root,
#     describing the standalone target graph — the same contract the Python
#     oracle emitted for the legacy build, with python_executable:null
#   * check_<kind> custom targets driving the produced binary
#   * the migration_T56 ctest script test (fixture negative controls)
#
# This file resolves no interpreter and invokes none. check_* targets are
# skipped when a name already exists (a bindings-ON configure still owns the
# legacy targets under native/).

if(NOT BIKE_ROOT_BUILD)
  message(FATAL_ERROR
    "native/cmake/Checks.cmake is the standalone-build check wiring; "
    "the legacy graph owns its own emission under native/")
endif()

set(_checks_native_dir "${CMAKE_CURRENT_SOURCE_DIR}/native")
set(_checks_tools_dir "${_checks_native_dir}/src/tools")

# --- produced check executables ------------------------------------------
# These translation units are first-party: they enter native_sources.json
# and compile_commands.json like every other target, so the produced
# sweeps analyze the check orchestrator itself under the same contract.
add_executable(bike-native-checks
  "${_checks_tools_dir}/checks_main.cpp"
  "${_checks_tools_dir}/check_context.cpp"
  "${_checks_tools_dir}/check_contracts.cpp"
  "${_checks_tools_dir}/check_frontends.cpp"
  "${_checks_tools_dir}/check_reports.cpp"
  "${_checks_tools_dir}/checks_internal.hpp")
set_target_properties(bike-native-checks PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
bike_apply_native_policy(bike-native-checks)
target_include_directories(bike-native-checks SYSTEM PRIVATE
  "${_checks_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")
# bike_io reaches the link line transitively through bike_platform's private
# edge; listing it again only draws a duplicate-library warning from ld.
target_link_libraries(bike-native-checks PRIVATE bike_platform)

add_executable(bike-python-audit
  "${_checks_tools_dir}/python_audit_main.cpp")
set_target_properties(bike-python-audit PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
bike_apply_native_policy(bike-python-audit)
target_include_directories(bike-python-audit SYSTEM PRIVATE
  "${_checks_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")
target_link_libraries(bike-python-audit PRIVATE bike_platform)

# --- shared schema-2 context and source manifest ---------------------------
# All first-party targets now exist, including the two produced check tools.
# The shared emitter owns both files for both graphs; NATIVE_DIR keeps the
# root graph's first-party include/suppression paths rooted under native/.
include("${_checks_native_dir}/cmake/BikeCheckContext.cmake")
bike_write_check_context(NATIVE_DIR "${_checks_native_dir}")

# --- produced sweep targets ------------------------------------------------
# Each target is one produced kind. The legacy target names under native/
# take precedence when they exist (bindings-on configure) so this block
# never redefines an owned target.
foreach(_check_kind IN ITEMS frontends tidy analyzer cppcheck odr headers
    diagnostic-controls context scripts)
  string(REPLACE "-" "_" _check_target "check_${_check_kind}")
  if(NOT TARGET ${_check_target})
    add_custom_target(${_check_target}
      COMMAND "$<TARGET_FILE:bike-native-checks>" --kind "${_check_kind}"
              --build "${CMAKE_BINARY_DIR}"
      DEPENDS bike-native-checks
      COMMENT "produced native check sweep: ${_check_kind}"
      VERBATIM)
  endif()
endforeach()
if(NOT TARGET check_contracts)
  add_custom_target(check_contracts
    COMMAND "$<TARGET_FILE:bike-native-checks>" --kind contracts
            --build "${CMAKE_BINARY_DIR}"
            --repo-root "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS bike-native-checks
    COMMENT "produced native structural contract checks"
    VERBATIM)
endif()

# --- migration_T56: fixture contract for the produced orchestrator --------
if(BUILD_TESTING)
  add_test(NAME migration_T56
    COMMAND "${CMAKE_COMMAND}"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "AUDIT_BINARY=$<TARGET_FILE:bike-python-audit>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/migration_T56"
      -P "${_checks_native_dir}/tests/migration/migration_T56_check.cmake")
  if(NOT TARGET migration_T56)
    add_custom_target(migration_T56
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^migration_T56$" --no-tests=error --output-on-failure
      DEPENDS bike-native-checks bike-python-audit
      USES_TERMINAL)
  endif()
endif()
