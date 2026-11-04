# native/cmake/Checks.cmake — produced check orchestration.
#
# Included once at the end of the root CMakeLists (standalone build).
# Provides:
#   * the bike-native-checks executable (bin/)
#   * native_sources.json + native_check_context.json at the build root,
#     describing the standalone target graph
#   * check_<kind> custom targets driving the produced binary
#   * the tooling_* ctest script tests (fixture negative controls)
#
# This file resolves no interpreter and invokes none. check_* targets are
# skipped when a name already exists (a bindings-ON configure still owns the
# legacy targets under native/).


set(_checks_native_dir "${CMAKE_CURRENT_SOURCE_DIR}/native")
set(_checks_tools_dir "${_checks_native_dir}/src/tools")

# --- produced check executable ---------------------------------------------
# These translation units are first-party: they enter native_sources.json
# and compile_commands.json like every other target, so the produced
# sweeps analyze the check orchestrator itself under the same contract.
add_executable(bike-native-checks
  "${_checks_tools_dir}/checks_main.cpp"
  "${_checks_tools_dir}/check_context.cpp"
  "${_checks_tools_dir}/check_contracts.cpp"
  "${_checks_tools_dir}/check_frontends.cpp"
  "${_checks_tools_dir}/check_pool.cpp"
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

# --- shared schema-2 context and source manifest ---------------------------
# All first-party targets now exist, including the produced check tool.
# The shared emitter owns both files; NATIVE_DIR keeps the root graph's
# first-party include/suppression paths rooted under native/.
include("${_checks_native_dir}/cmake/BikeCheckContext.cmake")
bike_write_check_context(NATIVE_DIR "${_checks_native_dir}")

# --- produced sweep targets ------------------------------------------------
# Each target is one produced kind. The legacy target names under native/
# take precedence when they exist (bindings-on configure) so this block
# never redefines an owned target.
foreach(_check_kind IN ITEMS frontends tidy cppcheck odr headers
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

# --- tooling_platform: portability/tooling contract -------------------------
# cmake -P script cases: scratch configures against the real Bike*.cmake
# modules (platform dispatch, compiler policy, provider identity, root
# configure guards, declared pin policy). Runs no produced binaries.
if(BUILD_TESTING)
  add_test(NAME tooling_platform
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_platform"
      -D "DEPS_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/.deps"
      -P "${_checks_native_dir}/tests/tooling/tooling_platform.cmake")
  # The case list includes scratch configures and real compiler builds;
  # give the whole script a generous budget.
  set_tests_properties(tooling_platform PROPERTIES
    LABELS "tooling;portability;slow" TIMEOUT 3600)
  if(NOT TARGET tooling_platform)
    add_custom_target(tooling_platform
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^tooling_platform$" --no-tests=error --output-on-failure
      USES_TERMINAL)
  endif()
endif()

# --- tooling_numerics: BikeNumerics.cmake record/provider/ABI ---------------
# Contract fixture: scratch configures of the real module plus committed-lock
# file invariants; needs no built target.
if(BUILD_TESTING)
  add_test(NAME tooling_numerics
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_numerics"
      -P "${_checks_native_dir}/tests/tooling/tooling_numerics.cmake")
  set_tests_properties(tooling_numerics PROPERTIES
    LABELS "tooling;numerics;slow")
  if(NOT TARGET tooling_numerics)
    add_custom_target(tooling_numerics
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^tooling_numerics$" --no-tests=error --output-on-failure
      USES_TERMINAL)
  endif()
endif()

# --- tooling_toolchain: portability tooling contracts -----------------------
# cmake -P script cases: drives the produced bike-native-checks --inspect-tool
# surface, real negative-compile fixtures and the emitted
# native_check_context.json for the toolchain/sanitizer-runtime contracts.
if(BUILD_TESTING)
  add_test(NAME tooling_toolchain
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_toolchain"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -P "${_checks_native_dir}/tests/tooling/tooling_toolchain.cmake")
  set_tests_properties(tooling_toolchain PROPERTIES
    LABELS "tooling;portability" TIMEOUT 1200)
  if(NOT TARGET tooling_toolchain)
    add_custom_target(tooling_toolchain
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^tooling_toolchain$" --no-tests=error --output-on-failure
      DEPENDS bike-native-checks
      USES_TERMINAL)
  endif()
endif()

# --- tooling_install: install/dev tooling contracts -------------------------
# cmake -P script cases: cmake --preset supported-set rejection, the
# relocated-build-cache configure guard and the produced context
# validator's fail-closed sanitizer-runtime rule.
if(BUILD_TESTING)
  add_test(NAME tooling_install
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_install"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -P "${_checks_native_dir}/tests/tooling/tooling_install.cmake")
  set_tests_properties(tooling_install PROPERTIES
    LABELS "tooling;portability" TIMEOUT 600)
  if(NOT TARGET tooling_install)
    add_custom_target(tooling_install
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^tooling_install$" --no-tests=error --output-on-failure
      DEPENDS bike-native-checks
      USES_TERMINAL)
  endif()
endif()

# --- tooling_gate: launcher contract on stub executables ---------------------
# Fast (no compiler, no real sweep): stays in the ordinary CTest set so the
# gate launchers are covered by every quick run.
if(BUILD_TESTING)
  add_test(NAME tooling_gate
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_gate"
      -P "${_checks_native_dir}/tests/tooling/tooling_gate.cmake")
  set_tests_properties(tooling_gate PROPERTIES LABELS "tooling" TIMEOUT 120)
endif()

if(BUILD_TESTING)
  add_test(NAME tooling_sweep_engine
    COMMAND "${CMAKE_COMMAND}"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_sweep_engine"
      -P "${_checks_native_dir}/tests/tooling/tooling_sweep_engine.cmake")
  set_tests_properties(tooling_sweep_engine PROPERTIES LABELS "tooling" TIMEOUT 300)
endif()

# --- tooling_checks: fixture contract for the produced orchestrator ---------
if(BUILD_TESTING)
  add_test(NAME tooling_checks
    COMMAND "${CMAKE_COMMAND}"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "DEPS_ROOT=${BIKE_DEPENDENCY_ROOT}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_checks"
      -P "${_checks_native_dir}/tests/tooling/tooling_checks.cmake")
  # The case list includes real-tree sweeps (headers/diagnostic-controls/
  # scripts over the selected build) plus scratch configures and instrumented
  # control builds; give the whole script a generous budget.
  set_tests_properties(tooling_checks PROPERTIES
    LABELS "tooling;slow" TIMEOUT 3600)
  if(NOT TARGET tooling_checks)
    add_custom_target(tooling_checks
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^tooling_checks$" --no-tests=error --output-on-failure
      DEPENDS bike-native-checks
      USES_TERMINAL)
  endif()
endif()
