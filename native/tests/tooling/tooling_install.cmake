# migration_T336r_portability — install/dev tooling contracts (batch 3.36r).
#
# Ports the re-assertable cases of
#   tests/portability/test_install_modes.py   and
#   tests/portability/test_dev_commands.py.
#
# The dev.py driver (uv/venv bootstrap, doctor report, subprocess/env
# plumbing) and the Python extension loader (bike_sim.native.artifact,
# tests/reference/native_loader) are internal to the removed tools and are
# dropped against their named native successors (see the ledger evidence);
# the records below are the ones whose *observable contract* survives
# natively:
#
#   * the supported configuration set is the declared CMakePresets.json
#     configurePresets list — any other name is rejected by CMake before a
#     single configure step runs, with the supported set named in the
#     diagnostic (test_unsupported_config_is_rejected).
#   * a copied/relocated build tree is rejected at configure time — CMake's
#     own cache-directory guard replaces dev.py's _check_build_cache
#     (test_copied_build_cache_is_rejected_before_build_process).
#   * a sanitizer lane recorded enabled without its resolved runtime is
#     rejected by the produced context validator — the same fail-closed
#     rule dev._doctor_probe_environment enforced on the selected runtime
#     (test_doctor_rejects_missing_selected_sanitizer_before_probe).
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR, CHECKS_BINARY, BUILD_DIR.
# BUILD_DIR must point at a configured build that emits
# native_check_context.json (schema 2).
#
# One case per pytest function; a case records a failure line when the
# outcome differs, exactly like the Python assertions it replaces.

foreach(_required REPO_ROOT SCRATCH_DIR CHECKS_BINARY BUILD_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "migration_T336r_portability requires -D${_required}")
  endif()
endforeach()

# IN_LIST is an if() operator only under the NEW CMP0057 behavior; script
# mode defaults policies to the host cmake_minimum_required (unset -> old).
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")

set(_case_count 0)
# Accumulators live on GLOBAL properties, not scope variables: _fail/_skip
# are called from inside _expect_* helper functions, and a PARENT_SCOPE set
# would only reach the helper's own frame — silently dropping the failure.
set_property(GLOBAL PROPERTY _t336r_failures "")
set_property(GLOBAL PROPERTY _t336r_skips "")

function(_fail msg)
  get_property(_f GLOBAL PROPERTY _t336r_failures)
  set_property(GLOBAL PROPERTY _t336r_failures "${_f}\n  ${msg}")
endfunction()

function(_skip msg)
  get_property(_s GLOBAL PROPERTY _t336r_skips)
  set_property(GLOBAL PROPERTY _t336r_skips "${_s}\n  skip: ${msg}")
  message(STATUS "skip: ${msg}")
endfunction()

# _run(name COMMAND <argv...>) — executes one command, returns rc plus
# combined stdout+stderr (matching subprocess.run capture_output).
function(_run name)
  cmake_parse_arguments(R "" "" "COMMAND;ENV" ${ARGN})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${R_ENV} ${R_COMMAND}
    RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
  set(${name}_rc "${_rc}" PARENT_SCOPE)
  set(${name}_out "${_out}\n${_err}" PARENT_SCOPE)
endfunction()

function(_expect_needle case haystack needle)
  string(FIND "${haystack}" "${needle}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

function(_expect_absent case haystack needle)
  string(FIND "${haystack}" "${needle}" _pos)
  if(NOT _pos EQUAL -1)
    _fail("${case}: unexpected '${needle}' present")
  endif()
endfunction()

function(_expect_equal case actual expected)
  if(NOT "${actual}" STREQUAL "${expected}")
    _fail("${case}: '${actual}' != '${expected}'")
  endif()
endfunction()

# _expect_squashed_t336r(case haystack needle) — whitespace-collapsed match:
# CMake wraps long FATAL_ERROR/diagnostic text at ~80 columns, so message
# needles must compare on words, not lines (same rule as migration_T53).
function(_expect_squashed_t336r case haystack needle)
  string(REGEX REPLACE "[ \t\r\n]+" " " _h "${haystack}")
  string(REGEX REPLACE "[ \t\r\n]+" " " _n "${needle}")
  string(FIND "${_h}" "${_n}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

# _expect_squashed_match(case haystack regex) — whitespace-collapsed MATCHES
# for diagnostics whose wording differs across supported CMake versions
# (3.28 says 'No such preset'; newer say 'No such configure preset').
function(_expect_squashed_match case haystack regex)
  string(REGEX REPLACE "[ \t\r\n]+" " " _h "${haystack}")
  if(NOT "${_h}" MATCHES "${regex}")
    _fail("${case}: output does not match '${regex}'")
  endif()
endfunction()

# ==========================================================================
# test_dev_commands.py — supported-configuration gate
# ==========================================================================

# test_unsupported_config_is_rejected: dev.py's --config gate accepted only
# the declared set. The supported build configuration in the Python-free
# workflow is the configurePresets list in CMakePresets.json; CMake itself
# rejects any other name, before a single configure step runs, and names
# the supported set in the diagnostic. The loop covers a real CMake build
# type spelling, an all-caps spelling and a plainly absent name — the same
# shape the Python test's rejection table had.
set(_c "unsupported_preset_is_rejected")
math(EXPR _case_count "${_case_count} + 1")
foreach(_bad IN ITEMS RelWithDebInfo DEBUG bike-no-such-config)
  _run(${_c}_${_bad} COMMAND "${CMAKE_COMMAND}" -E chdir "${REPO_ROOT}"
    "${CMAKE_COMMAND}" --preset "${_bad}")
  if(${_c}_${_bad}_rc EQUAL 0)
    _fail("${_c}: preset '${_bad}' was accepted")
  endif()
  _expect_squashed_match(${_c} "${${_c}_${_bad}_out}"
    "No such.*preset")
endforeach()
# The diagnostic must name the supported set — the qualified presets this
# workflow documents (docs/installation.md).
_run(${_c}_diag COMMAND "${CMAKE_COMMAND}" -E chdir "${REPO_ROOT}"
  "${CMAKE_COMMAND}" --preset bike-no-such-config)
_expect_needle(${_c} "${${_c}_diag_out}" "\"release\"")
_expect_needle(${_c} "${${_c}_diag_out}" "\"debug\"")

# ==========================================================================
# test_install_modes.py — relocated build cache + sanitizer-runtime gate
# ==========================================================================

# test_copied_build_cache_is_rejected_before_build_process: dev.py's
# _check_build_cache compared the recorded CMAKE_CACHEFILE_DIR against the
# selected build dir and refused a copied tree before spawning a build.
# CMake carries the same guard natively: a configure that finds a foreign
# CMakeCache.txt fails before generating — the identical
# rejected-before-any-build ordering.
set(_c "copied_build_cache_rejected_at_configure")
math(EXPR _case_count "${_case_count} + 1")
file(MAKE_DIRECTORY "${SCRATCH_DIR}/cache-src")
file(WRITE "${SCRATCH_DIR}/cache-src/CMakeLists.txt"
  "cmake_minimum_required(VERSION 3.27)\nproject(relocated_cache NONE)\n")
_run(${_c}_seed COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cache-src"
  -B "${SCRATCH_DIR}/cache-a")
if(NOT ${_c}_seed_rc EQUAL 0)
  _fail("${_c}: seed configure failed: ${${_c}_seed_out}")
else()
  _run(${_c}_copy COMMAND "${CMAKE_COMMAND}" -E copy_directory
    "${SCRATCH_DIR}/cache-a" "${SCRATCH_DIR}/cache-b")
  if(NOT ${_c}_copy_rc EQUAL 0)
    _fail("${_c}: could not stage a copied build tree: ${${_c}_copy_out}")
  else()
    _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cache-src"
      -B "${SCRATCH_DIR}/cache-b")
    if(${_c}_rc EQUAL 0)
      _fail("${_c}: configure accepted a relocated build cache")
    endif()
    _expect_squashed_t336r(${_c} "${${_c}_out}" "is different than the directory")
    _expect_squashed_t336r(${_c} "${${_c}_out}" "where CMakeCache.txt was created")
    _expect_needle(${_c} "${${_c}_out}" "cache-a")
  endif()
endif()

# test_doctor_rejects_missing_selected_sanitizer_before_probe: the preload
# resolver refused a selected sanitizer runtime that does not exist before
# composing a probe environment. The produced context validator carries
# the same fail-closed rule: a sanitizer lane may never be enabled without
# its recorded, resolved runtime file. Drive the real emitted context
# (mutated on a copy, the migration_T56 convention) through
# bike-native-checks --kind context.
set(_c "sanitizer_enabled_without_runtime_is_rejected")
math(EXPR _case_count "${_case_count} + 1")
set(_ctx "${BUILD_DIR}/native_check_context.json")
if(EXISTS "${_ctx}")
  file(READ "${_ctx}" _context_json)
else()
  set(_context_json "")
  _fail("native_check_context: '${_ctx}' missing")
endif()
if(NOT _context_json STREQUAL "")
  string(JSON _mutated SET "${_context_json}"
    capabilities asan requested "true")
  string(JSON _mutated SET "${_mutated}"
    capabilities asan supported "true")
  string(JSON _mutated SET "${_mutated}"
    capabilities asan enabled "true")
  string(JSON _mutated SET "${_mutated}"
    capabilities asan runtime "null")
  file(MAKE_DIRECTORY "${SCRATCH_DIR}/ctx-enabled-no-runtime")
  file(WRITE "${SCRATCH_DIR}/ctx-enabled-no-runtime/native_check_context.json"
    "${_mutated}")
  _run(${_c} COMMAND "${CHECKS_BINARY}" --kind context
    --build "${SCRATCH_DIR}/ctx-enabled-no-runtime" --validate-only)
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: an enabled sanitizer lane with no runtime was accepted")
  endif()
  _expect_squashed_t336r(${_c} "${${_c}_out}"
    "enabled without a recorded sanitizer runtime")
endif()

# ==========================================================================
# Report
# ==========================================================================
get_property(_skipped GLOBAL PROPERTY _t336r_skips)
get_property(_failures GLOBAL PROPERTY _t336r_failures)
if(NOT _skipped STREQUAL "")
  message(STATUS "migration_T336r_portability skips:${_skipped}")
endif()
if(NOT _failures STREQUAL "")
  message(FATAL_ERROR
    "migration_T336r_portability failed (${_case_count} cases):${_failures}")
endif()
message(STATUS "migration_T336r_portability passed (${_case_count} cases)")
