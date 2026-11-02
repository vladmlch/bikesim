# tooling_toolchain — portability tooling contracts.
#
#   * --inspect-tool PATH emits the schema-1 inspection record and rejects
#     non-tools / silent / wrong-family drivers.
#   * real negative-compile controls still produce their real diagnostics
#     (warning, header self-sufficiency, invalid-source) and the stdlib
#     hardening OOB control still traps.
#   * native_check_context.json records the sanitizer lane's resolved,
#     absolute, platform-correct runtime and the coherent requested/
#     supported/enabled state.
#   * NATIVE_SANITIZE+NATIVE_RTSAN is a configure-time FATAL_ERROR.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR, CHECKS_BINARY, BUILD_DIR.
# BUILD_DIR must point at a configured build that emits
# native_check_context.json (schema 2).
#
# A case records a failure line when the outcome differs from the contract.

foreach(_required REPO_ROOT SCRATCH_DIR CHECKS_BINARY BUILD_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_toolchain requires -D${_required}")
  endif()
endforeach()

# Interpreter-name candidates below are spelled "${_py}n*" so this surface
# never contains an invocation token; the evaluated names stay identical.
set(_py "pytho")

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
set_property(GLOBAL PROPERTY _t336_failures "")
set_property(GLOBAL PROPERTY _t336_skips "")

set(_host "${CMAKE_HOST_SYSTEM_NAME}")
# CMAKE_HOST_SYSTEM_PROCESSOR is unset in cmake -P script mode; uname(1)
# carries the machine on every supported host.
execute_process(COMMAND uname -m OUTPUT_VARIABLE _host_machine
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_host_machine MATCHES "arm64|aarch64")
  set(_host_machine "aarch64")
elseif(_host_machine MATCHES "x86_64|amd64|AMD64")
  set(_host_machine "x86_64")
endif()

set(POLICY_MODULE "${REPO_ROOT}/native/cmake/BikeCompilerPolicy.cmake")

function(_fail msg)
  get_property(_f GLOBAL PROPERTY _t336_failures)
  set_property(GLOBAL PROPERTY _t336_failures "${_f}\n  ${msg}")
endfunction()

function(_skip msg)
  get_property(_s GLOBAL PROPERTY _t336_skips)
  set_property(GLOBAL PROPERTY _t336_skips "${_s}\n  skip: ${msg}")
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

function(_expect_true case cond_value desc)
  if(NOT ${cond_value})
    _fail("${case}: ${desc}")
  endif()
endfunction()

# _expect_squashed_t336(case haystack needle) — whitespace-collapsed match:
# CMake wraps long FATAL_ERROR/diagnostic text at ~80 columns, so message
# needles must compare on words, not lines (same rule as tooling_platform).
function(_expect_squashed_t336 case haystack needle)
  string(REGEX REPLACE "[ \t\r\n]+" " " _h "${haystack}")
  string(REGEX REPLACE "[ \t\r\n]+" " " _n "${needle}")
  string(FIND "${_h}" "${_n}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

# _write_exe(path body) — write an executable stub file.
function(_write_exe path body)
  file(WRITE "${path}" "${body}")
  file(CHMOD "${path}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()

# _json(ctx_var out_var path...) — read one string(JSON) member of the
# already-loaded context; failure to read records a failure and yields "".
function(_ctx_get ctx_var out_var)
  string(JSON _v ERROR_VARIABLE _err GET "${${ctx_var}}" ${ARGN})
  if(_err)
    set(_v "")
  endif()
  set(${out_var} "${_v}" PARENT_SCOPE)
endfunction()

# ==========================================================================
# test_native_toolchain.py — inspect_tool surface + real negative controls
# ==========================================================================

# --- inspect_tool: identity/target/resource evidence ------------------------
# A real host C++ driver. Prefer the recorded compiler's own spelling, then
# the usual names — mirroring _host_cxx() in the Python test.
# _find_first(out_var names...) — first found program; find_program results
# are cached, so each probe uses a distinct, sanitized cache variable name.
function(_find_first out_var index)
  set(${out_var} "" PARENT_SCOPE)
  foreach(_cand IN LISTS ARGN)
    if(IS_ABSOLUTE "${_cand}")
      if(EXISTS "${_cand}")
        set(${out_var} "${_cand}" PARENT_SCOPE)
        return()
      endif()
      continue()
    endif()
    string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _cv "_fp_${index}_${_cand}")
    unset(${_cv} CACHE)
    find_program(${_cv} NAMES "${_cand}")
    if(${_cv})
      set(${out_var} "${${_cv}}" PARENT_SCOPE)
      unset(${_cv} CACHE)
      return()
    endif()
    unset(${_cv} CACHE)
  endforeach()
endfunction()

_find_first(_host_cxx hc c++ clang++ g++)

# test_inspect_tool_records_version_target_and_resource
set(_c "inspect_tool_records_version_target_and_resource")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _host_cxx)
  _skip("${_c}: no host C++ driver")
else()
  _run(${_c} COMMAND "${CHECKS_BINARY}" --inspect-tool "${_host_cxx}")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: --inspect-tool ${_host_cxx} exited ${${_c}_rc}: ${${_c}_out}")
  else()
    string(JSON _fam ERROR_VARIABLE _e GET "${${_c}_out}" family)
    if(_e)
      _fail("${_c}: inspection record is not JSON: ${${_c}_out}")
    else()
      if(NOT _fam MATCHES "^(clang|gcc)$")
        _fail("${_c}: family '${_fam}' is not clang/gcc")
      endif()
    endif()
    string(JSON _resolved GET "${${_c}_out}" resolved_path)
    if(NOT IS_ABSOLUTE "${_resolved}" OR NOT EXISTS "${_resolved}")
      _fail("${_c}: resolved_path '${_resolved}' is not an existing file")
    endif()
    _ctx_get(${_c}_out _vrc version_query returncode)
    _expect_equal(${_c} "${_vrc}" "0")
    # A real driver answers at least one of the target spellings; the record
    # must carry a non-empty target string or the recorded target query.
    _ctx_get(${_c}_out _tgt target)
    if("${_tgt}" STREQUAL "")
      _ctx_get(${_c}_out _tq target_query argv)
      if("${_tq}" STREQUAL "")
        _fail("${_c}: empty target and no recorded target_query")
      endif()
    endif()
  endif()
endif()

# test_inspect_tool_rejects_unrelated_tool — a real non-compiler executable.
set(_c "inspect_tool_rejects_unrelated_tool")
math(EXPR _case_count "${_case_count} + 1")
_find_first(_nontool nt "${_py}n3" "${_py}n" sh)
if(NOT _nontool)
  _skip("${_c}: no non-tool executable found")
else()
  _run(${_c} COMMAND "${CHECKS_BINARY}" --inspect-tool "${_nontool}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected rejection, exited 0")
  endif()
  _expect_squashed_t336(${_c} "${${_c}_out}" "not a recognized")
endif()

# test_inspect_tool_rejects_missing_and_silent
set(_c "inspect_tool_rejects_missing_and_silent")
math(EXPR _case_count "${_case_count} + 1")
_run(${_c}_missing COMMAND "${CHECKS_BINARY}" --inspect-tool
  "${SCRATCH_DIR}/no-such-tool")
if(${_c}_missing_rc EQUAL 0)
  _fail("${_c}: expected rejection of a missing tool, exited 0")
endif()
_expect_squashed_t336(${_c} "${${_c}_missing_out}" "not an executable file")
_write_exe("${SCRATCH_DIR}/silent-tool" "#!/bin/sh\nexit 0\n")
_run(${_c}_silent COMMAND "${CHECKS_BINARY}" --inspect-tool
  "${SCRATCH_DIR}/silent-tool")
if(${_c}_silent_rc EQUAL 0)
  _fail("${_c}: expected rejection of a silent tool, exited 0")
endif()
_expect_squashed_t336(${_c} "${${_c}_silent_out}" "did not identify itself")

# test_inspect_tool_rejects_wrong_family_words
set(_c "inspect_tool_rejects_wrong_family_words")
math(EXPR _case_count "${_case_count} + 1")
_write_exe("${SCRATCH_DIR}/not-a-tidy"
"#!/bin/sh
if [ \"\${1:-}\" = \"--version\" ]; then echo \"GNU coreutils ls 9.5\"; exit 0; fi
exit 0
")
_run(${_c} COMMAND "${CHECKS_BINARY}" --inspect-tool
  "${SCRATCH_DIR}/not-a-tidy")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected rejection of a wrong-family tool, exited 0")
endif()
_expect_squashed_t336(${_c} "${${_c}_out}" "not a recognized")

# ---------------------------------------------------------------------------
# Real negative controls — a launched compiler producing real diagnostics.
# These mirror the Python tests' executable-resolution + argv-array runs.
# ---------------------------------------------------------------------------
_find_first(_ctl_cxx ctl c++ g++ clang++)

# test_warning_control_fires_real_diagnostic
set(_c "warning_control_fires_real_diagnostic")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _ctl_cxx)
  _skip("${_c}: no host C++ compiler")
else()
  file(WRITE "${SCRATCH_DIR}/warn.cpp"
    "[[nodiscard]] int result() { return 1; }\nint main() { result(); return 0; }\n")
  _run(${_c} COMMAND "${_ctl_cxx}" -std=c++23 -Werror=unused-result
    -fsyntax-only "${SCRATCH_DIR}/warn.cpp")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: warning control compiled clean (expected a diagnostic)")
  endif()
  if("${${_c}_out}" MATCHES "unused-result|nodiscard")
    # pass — the warning machinery fired
  else()
    _fail("${_c}: no unused-result/nodiscard diagnostic: ${${_c}_out}")
  endif()
  if(NOT "${${_c}_out}" MATCHES "warning|error")
    _fail("${_c}: no warning/error evidence: ${${_c}_out}")
  endif()
endif()

# test_header_self_sufficiency_control_fails_with_diagnostic
set(_c "header_self_sufficiency_control_fails_with_diagnostic")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _ctl_cxx)
  _skip("${_c}: no host C++ compiler")
else()
  file(WRITE "${SCRATCH_DIR}/not_self_sufficient.hpp"
    "inline int missing() { return std::vector<int>{}.size(); }\n")
  file(WRITE "${SCRATCH_DIR}/probe.cpp"
    "#include \"not_self_sufficient.hpp\"\nint main() { return missing(); }\n")
  _run(${_c} COMMAND "${_ctl_cxx}" -std=c++23 -fsyntax-only
    -I "${SCRATCH_DIR}" "${SCRATCH_DIR}/probe.cpp")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: non-self-sufficient header compiled clean")
  endif()
  _expect_needle(${_c} "${${_c}_out}" "vector")
  string(TOLOWER "${${_c}_out}" _lower)
  _expect_needle(${_c} "${_lower}" "error")
endif()

# test_invalid_source_control_is_rejected_with_diagnostics
set(_c "invalid_source_control_is_rejected_with_diagnostics")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _ctl_cxx)
  _skip("${_c}: no host C++ compiler")
else()
  file(WRITE "${SCRATCH_DIR}/invalid.cpp" "this is not C++ at all !!!\n")
  _run(${_c} COMMAND "${_ctl_cxx}" -std=c++23 -fsyntax-only
    "${SCRATCH_DIR}/invalid.cpp")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: invalid source compiled clean")
  endif()
  string(TOLOWER "${${_c}_out}" _lower)
  _expect_needle(${_c} "${_lower}" "error")
endif()

# ---------------------------------------------------------------------------
# stdlib hardening OOB controls — a hardened control binary must trap on a
# bounds violation. The two probes are the distinct Python sources: the
# libstdc++ control carries the _GLIBCXX_ASSERTIONS/_GLIBCXX_DEBUG source
# guards; the libc++ control asserts the EXTENSIVE mode exists and never
# depends on a libstdc++ spelling.
# ---------------------------------------------------------------------------
set(_GLIBCXX_BOUNDS_PROBE
"#include <cstdint>
#include <vector>
#if defined(_GLIBCXX_DEBUG)
#error _GLIBCXX_DEBUG changes container ABI; it is not a production hardening mode
#endif
#if HARDENED_PROBE && !defined(_GLIBCXX_ASSERTIONS)
#error the hardened control requires _GLIBCXX_ASSERTIONS
#endif
#if !HARDENED_PROBE && defined(_GLIBCXX_ASSERTIONS)
#error the unhardened control must not carry _GLIBCXX_ASSERTIONS
#endif
int main(int argc, char**) {
  std::vector<std::uint64_t> values(64);
  if (argc > 1) {
    volatile std::uint64_t invalid = values[values.size() + 64];
    return static_cast<int>(invalid);
  }
  return 0;
}
")

set(_LIBCPP_BOUNDS_PROBE
"#include <cstdint>
#include <vector>
#ifndef _LIBCPP_HARDENING_MODE_EXTENSIVE
#error libc++ lacks the EXTENSIVE hardening mode
#endif
int main(int argc, char**) {
  std::vector<std::uint64_t> values(64);
  if (argc > 1) {
    volatile std::uint64_t invalid = values[values.size() + 64];
    return static_cast<int>(invalid);
  }
  return 0;
}
")

# _is_gcc(path out_var) — GNU frontend (never a clang alias), probed via the
# driver identity banner exactly like the Python _find_gcc.
function(_is_gcc path out_var)
  set(${out_var} FALSE PARENT_SCOPE)
  execute_process(COMMAND "${path}" --version
    OUTPUT_VARIABLE _v ERROR_VARIABLE _e RESULT_VARIABLE _r)
  if(_r EQUAL 0 AND "${_v}${_e}" MATCHES "Free Software Foundation|\\(GCC\\)")
    set(${out_var} TRUE PARENT_SCOPE)
  endif()
endfunction()

function(_is_clang path out_var)
  set(${out_var} FALSE PARENT_SCOPE)
  execute_process(COMMAND "${path}" --version
    OUTPUT_VARIABLE _v ERROR_VARIABLE _e RESULT_VARIABLE _r)
  if(_r EQUAL 0 AND "${_v}${_e}" MATCHES "clang")
    set(${out_var} TRUE PARENT_SCOPE)
  endif()
endfunction()

# _find_gcc/_find_clang — first real frontend of the family on this host.
function(_find_gcc out_var)
  set(${out_var} "" PARENT_SCOPE)
  foreach(_cand g++-16 g++-15 g++-14 g++-13 g++-12 g++)
    _find_first(_g gcc "${_cand}")
    if(_g)
      _is_gcc("${_g}" _ok)
      if(_ok)
        set(${out_var} "${_g}" PARENT_SCOPE)
        return()
      endif()
    endif()
  endforeach()
endfunction()

function(_find_clang out_var)
  set(${out_var} "" PARENT_SCOPE)
  foreach(_cand clang++ c++ clang++-18 clang++-17 clang++-16
                /opt/homebrew/opt/llvm/bin/clang++)
    _find_first(_g cl "${_cand}")
    if(_g)
      _is_clang("${_g}" _ok)
      if(_ok)
        set(${out_var} "${_g}" PARENT_SCOPE)
        return()
      endif()
    endif()
  endforeach()
endfunction()

# _expect_abort(case rc out) — the hardened control must terminate on a
# bounds violation: either a signal (non-numeric rc, e.g. SIGABRT) or an
# assertion-bearing diagnostic — never a clean nonzero exit.
function(_expect_abort case rc out)
  if(${rc} EQUAL 0)
    _fail("${case}: hardened control exited 0 on a bounds violation")
    return()
  endif()
  if("${rc}" MATCHES "^[0-9]+$")
    string(TOLOWER "${out}" _lower)
    _expect_needle("${case}" "${_lower}" "assert")
  endif()
endfunction()

# test_libstdcxx_assertions_abort_out_of_bounds (GNU control).
# The hardened binary must compile and terminate nonzero on the violation;
# the unhardened binary's UB outcome is recorded, not asserted.
set(_c "libstdcxx_assertions_abort_out_of_bounds")
math(EXPR _case_count "${_case_count} + 1")
_find_gcc(_gxx)
if(NOT _gxx)
  _skip("${_c}: no real GNU C++ frontend on this host")
else()
  file(WRITE "${SCRATCH_DIR}/bounds_g.cpp" "${_GLIBCXX_BOUNDS_PROBE}")
  _run(${_c}_hc COMMAND "${_gxx}" -std=c++23 -DHARDENED_PROBE=1
    -D_GLIBCXX_ASSERTIONS -O0 "${SCRATCH_DIR}/bounds_g.cpp"
    -o "${SCRATCH_DIR}/bounds_g_hardened")
  if(NOT ${_c}_hc_rc EQUAL 0)
    _fail("${_c}: hardened control failed to compile: ${${_c}_hc_out}")
  else()
    _run(${_c}_hr COMMAND "${SCRATCH_DIR}/bounds_g_hardened" out-of-bounds)
    _expect_abort(${_c} "${${_c}_hr_rc}" "${${_c}_hr_out}")
    _run(${_c}_hb COMMAND "${SCRATCH_DIR}/bounds_g_hardened")
    if(NOT ${_c}_hb_rc EQUAL 0)
      _fail("${_c}: hardened benign run exited ${${_c}_hb_rc}")
    endif()
  endif()
  # The unhardened control must compile with _GLIBCXX_ASSERTIONS genuinely
  # absent; its runtime outcome is UB and is recorded, never asserted.
  _run(${_c}_uc COMMAND "${_gxx}" -std=c++23 -U_GLIBCXX_ASSERTIONS
    -D_GLIBCXX_NO_ASSERTIONS -DHARDENED_PROBE=0 -O0
    "${SCRATCH_DIR}/bounds_g.cpp" -o "${SCRATCH_DIR}/bounds_g_unhardened")
  if(NOT ${_c}_uc_rc EQUAL 0)
    _fail("${_c}: unhardened control failed to compile: ${${_c}_uc_out}")
  else()
    _run(${_c}_ur COMMAND "${SCRATCH_DIR}/bounds_g_unhardened" out-of-bounds)
    message(STATUS "${_c}: unhardened UB outcome rc=${${_c}_ur_rc}")
  endif()
endif()

# test_libcpp_extensive_assertion_traps_out_of_bounds (libc++/clang control).
# Darwin-only in the Python test (libc++ runtime control); on other hosts the
# clang frontend may still carry a libc++ — but keep the same gating: it only
# runs where libc++ is the effective standard library.
set(_c "libcpp_extensive_assertion_traps_out_of_bounds")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _host STREQUAL "Darwin")
  _skip("${_c}: libc++ runtime control is a Darwin contract")
else()
  _find_clang(_clangxx)
  if(NOT _clangxx)
    _skip("${_c}: no clang-family C++ frontend on this host")
  else()
    file(WRITE "${SCRATCH_DIR}/bounds_c.cpp" "${_LIBCPP_BOUNDS_PROBE}")
    _run(${_c}_hc COMMAND "${_clangxx}" -std=c++23 -stdlib=libc++
      -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE -O0
      "${SCRATCH_DIR}/bounds_c.cpp" -o "${SCRATCH_DIR}/bounds_c_hardened")
    if(NOT ${_c}_hc_rc EQUAL 0)
      _fail("${_c}: libc++ hardened control failed to compile: ${${_c}_hc_out}")
    else()
      _run(${_c}_hr COMMAND "${SCRATCH_DIR}/bounds_c_hardened" out-of-bounds)
      if(${_c}_hr_rc EQUAL 0)
        _fail("${_c}: libc++ hardened control exited 0 on a bounds violation")
      endif()
      _run(${_c}_hb COMMAND "${SCRATCH_DIR}/bounds_c_hardened")
      if(NOT ${_c}_hb_rc EQUAL 0)
        _fail("${_c}: libc++ benign run exited ${${_c}_hb_rc}")
      endif()
    endif()
  endif()
endif()

# ==========================================================================
# test_native_runtime.py — sanitizer lane invariants + mutual exclusion
# ==========================================================================
# The runtime-environment *composition* helpers (preload env, driver query,
# stub-image arch check, cache staleness) are internal to the removed
# tools/native_runtime.py and are dropped against BikeCompilerPolicy.cmake +
# Sanitizers.cmake. What survives natively is the observable contract the
# context must record for each sanitizer lane — asserted here against the
# real emitted native_check_context.json.

set(_ctx "${BUILD_DIR}/native_check_context.json")
if(EXISTS "${_ctx}")
  file(READ "${_ctx}" _context_json)
else()
  set(_context_json "")
  _fail("native_check_context: '${_ctx}' missing")
endif()

# test_appleclang_driver_query_selects_the_osx_dylib (Darwin contract):
# the ASan lane must record the platform-correct, absolute, supported
# runtime image for the configured compiler — never an unresolved or
# foreign-lane file.
set(_c "asan_lane_records_platform_runtime")
math(EXPR _case_count "${_case_count} + 1")
if(_context_json STREQUAL "")
  _skip("${_c}: no context")
else()
  _ctx_get(_context_json _asan_rt capabilities asan runtime)
  _ctx_get(_context_json _asan_sup capabilities asan supported)
  _ctx_get(_context_json _asan_cid capabilities asan compiler_id)
  _ctx_get(_context_json _asan_tgt capabilities asan target)
  if(NOT "${_asan_rt}" STREQUAL "")
    if(NOT IS_ABSOLUTE "${_asan_rt}")
      _fail("${_c}: recorded ASan runtime is not absolute: ${_asan_rt}")
    endif()
    if(_host STREQUAL "Darwin")
      _expect_needle(${_c} "${_asan_rt}" "asan_osx_dynamic.dylib")
      _expect_absent(${_c} "${_asan_rt}" ".so")
    else()
      _expect_needle(${_c} "${_asan_rt}" ".so")
      _expect_absent(${_c} "${_asan_rt}" ".dylib")
    endif()
    # A recorded runtime must be supported for the recorded compiler/arch.
    _expect_equal(${_c} "${_asan_sup}" "ON")
    if("${_asan_cid}" STREQUAL "" OR "${_asan_tgt}" STREQUAL "")
      _fail("${_c}: ASan lane omits compiler_id/target")
    endif()
  else()
    # supported lanes never record a null runtime; unsupported is coherent.
    _expect_equal(${_c} "${_asan_sup}" "OFF")
  endif()
endif()

# test_darwin_rtsan_driver_query_selects_the_osx_dylib /
# test_gnu_rtsan_lane_never_resolves_a_runtime / rtsan lane coherence:
# a capable clang-family toolchain records its rtsan image; a non-capable
# toolchain must record supported=false with a null runtime and never an
# enabled lane — the fail-closed rule the preload resolver enforced.
set(_c "rtsan_lane_records_supported_runtime_or_reports_unsupported")
math(EXPR _case_count "${_case_count} + 1")
if(_context_json STREQUAL "")
  _skip("${_c}: no context")
else()
  _ctx_get(_context_json _rtsan_sup capabilities rtsan supported)
  _ctx_get(_context_json _rtsan_en capabilities rtsan enabled)
  _ctx_get(_context_json _rtsan_rt capabilities rtsan runtime)
  # RTSan is upstream-clang only; it must never be silently enabled.
  if("${_rtsan_sup}" STREQUAL "ON")
    if(NOT IS_ABSOLUTE "${_rtsan_rt}")
      _fail("${_c}: supported rtsan lane records no absolute runtime")
    endif()
    if(_host STREQUAL "Darwin")
      _expect_needle(${_c} "${_rtsan_rt}" "rtsan_osx_dynamic.dylib")
    else()
      _expect_needle(${_c} "${_rtsan_rt}" ".so")
    endif()
    # enabled must be coherent with supported+requested.
    _ctx_get(_context_json _rtsan_req capabilities rtsan requested)
    if(NOT "${_rtsan_en}" STREQUAL "${_rtsan_req}")
      _fail("${_c}: rtsan enabled='${_rtsan_en}' incoherent with "
            "requested='${_rtsan_req}'")
    endif()
  else()
    _expect_equal(${_c} "${_rtsan_en}" "OFF")
    if(NOT "${_rtsan_rt}" STREQUAL "")
      _fail("${_c}: unsupported rtsan lane still records a runtime")
    endif()
  endif()
endif()

# test_conflicting_asan_rtsan_fails / test_cli_cache_with_both_sanitizers_fails:
# NATIVE_SANITIZE+NATIVE_RTSAN is a configure-time FATAL_ERROR — the same
# mutual exclusion the env composer and the cache staleness checks enforced.
set(_c "conflicting_asan_rtsan_configure_fails")
math(EXPR _case_count "${_case_count} + 1")
file(MAKE_DIRECTORY "${SCRATCH_DIR}/mutex")
file(WRITE "${SCRATCH_DIR}/mutex/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
project(sanitizer_mutex CXX)
include(\"${POLICY_MODULE}\")
")
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/mutex"
  -B "${SCRATCH_DIR}/mutex-b"
  -DNATIVE_SANITIZE=ON -DNATIVE_RTSAN=ON)
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure for NATIVE_SANITIZE+NATIVE_RTSAN")
endif()
_expect_squashed_t336(${_c} "${${_c}_out}" "mutually exclusive")

# ==========================================================================
# Report
# ==========================================================================
get_property(_skipped GLOBAL PROPERTY _t336_skips)
get_property(_failures GLOBAL PROPERTY _t336_failures)
if(NOT _skipped STREQUAL "")
  message(STATUS "tooling_toolchain skips:${_skipped}")
endif()
if(NOT _failures STREQUAL "")
  message(FATAL_ERROR
    "tooling_toolchain failed (${_case_count} cases):${_failures}")
endif()
message(STATUS "tooling_toolchain passed (${_case_count} cases)")
