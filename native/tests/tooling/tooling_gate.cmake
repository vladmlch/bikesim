# tooling_gate — fast contract for the gate launchers (tools/run_tests.sh,
# tools/run_native_checks.sh) driven against stub cmake/ctest/checks
# executables. Runs no compiler and no real sweep, so it stays in the
# ordinary (non-slow) CTest set.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_gate requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")

# gate_run(<name> EXPECT_EXIT <rc|FAIL> [EXPECT_OUTPUT <needle>...]
#          [ENV <k=v>...] COMMAND <argv...>)
function(gate_run name)
  cmake_parse_arguments(G "" "EXPECT_EXIT" "EXPECT_OUTPUT;ENV;COMMAND" ${ARGN})
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env ${G_ENV} ${G_COMMAND}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  set(_all "${_out}\n${_err}")
  set(${name}_output "${_all}" PARENT_SCOPE)
  if(G_EXPECT_EXIT STREQUAL "FAIL")
    if(_rc EQUAL 0)
      set(_failures "${_failures}\n  ${name}: expected failure, exited 0" PARENT_SCOPE)
    endif()
  elseif(NOT _rc EQUAL ${G_EXPECT_EXIT})
    set(_failures "${_failures}\n  ${name}: expected exit ${G_EXPECT_EXIT}, got ${_rc}:\n${_all}" PARENT_SCOPE)
  endif()
  foreach(_needle IN LISTS G_EXPECT_OUTPUT)
    string(FIND "${_all}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      set(_failures "${_failures}\n  ${name}: output missing '${_needle}':\n${_all}" PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

# expect_file_lines(<name> <file> <expected;lines>) — exact line list.
function(expect_file_lines name file)
  set(_got "")
  if(EXISTS "${file}")
    file(STRINGS "${file}" _got)
  endif()
  if(NOT "${_got}" STREQUAL "${ARGN}")
    set(_failures "${_failures}\n  ${name}: ${file} is '${_got}', expected '${ARGN}'" PARENT_SCOPE)
  endif()
endfunction()

# A stub bike-native-checks: appends its --kind to calls.log and exits with
# the code named by STUB_FAIL_<kind> (dashes become underscores), else 0.
set(_stub_dir "${SCRATCH_DIR}/stubs")
file(MAKE_DIRECTORY "${_stub_dir}")
# Shell ${...} inside these CMake strings is escaped as \${...} so CMake does
# not expand it at write time (same convention as b29_mock_tool).
file(WRITE "${_stub_dir}/bike-native-checks"
  "#!/usr/bin/env bash\n"
  "kind=''\n"
  "while (($#)); do [[ $1 == --kind ]] && kind=$2; shift; done\n"
  "printf 'checks %s jobs=%s\\n' \"$kind\" \"\${BIKE_CHECK_JOBS:-unset}\" >> \"\${STUB_LOG}\"\n"
  "var=\"STUB_FAIL_\${kind//-/_}\"\n"
  "exit \"\${!var:-0}\"\n")
file(CHMOD "${_stub_dir}/bike-native-checks"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

set(_build "${SCRATCH_DIR}/build")
file(MAKE_DIRECTORY "${_build}")
set(_log "${SCRATCH_DIR}/calls.log")
set(_order scripts cppcheck context diagnostic-controls contracts
  frontends headers odr tidy analyzer)
set(_expected "")
foreach(_k IN LISTS _order)
  list(APPEND _expected "checks ${_k} jobs=unset")
endforeach()
set(_checks_env "BIKE_NATIVE_CHECKS=${_stub_dir}/bike-native-checks"
  "STUB_LOG=${_log}" --unset=BIKE_CHECK_JOBS --unset=BIKE_GATE_TIMINGS_FILE)

# --- run_native_checks.sh: cheapest kinds first ---------------------------
file(REMOVE "${_log}")
gate_run(checks_order EXPECT_EXIT 0 ENV ${_checks_env}
  EXPECT_OUTPUT "all produced sweeps passed"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" "${_build}")
expect_file_lines(checks_order "${_log}" ${_expected})

# --- --from skips every earlier kind and marks the run partial -----------
file(REMOVE "${_log}")
gate_run(checks_from EXPECT_EXIT 0 ENV ${_checks_env}
  EXPECT_OUTPUT "PARTIAL run"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from odr "${_build}")
expect_file_lines(checks_from "${_log}"
  "checks odr jobs=unset" "checks tidy jobs=unset" "checks analyzer jobs=unset")

gate_run(checks_from_unknown EXPECT_EXIT FAIL ENV ${_checks_env}
  EXPECT_OUTPUT "unknown --from kind: bogus"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from bogus "${_build}")

# --- a failing kind stops the run with its exit code ----------------------
file(REMOVE "${_log}")
gate_run(checks_stop EXPECT_EXIT 3 ENV ${_checks_env} "STUB_FAIL_cppcheck=3"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" "${_build}")
expect_file_lines(checks_stop "${_log}"
  "checks scripts jobs=unset" "checks cppcheck jobs=unset")

# --- per-kind timings ------------------------------------------------------
set(_timings "${SCRATCH_DIR}/timings.tsv")
file(REMOVE "${_timings}")
gate_run(checks_timings EXPECT_EXIT 0
  ENV ${_checks_env} "BIKE_GATE_TIMINGS_FILE=${_timings}"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from tidy "${_build}")
file(STRINGS "${_timings}" _timing_lines)
list(LENGTH _timing_lines _timing_count)
if(NOT _timing_count EQUAL 2)
  set(_failures "${_failures}\n  checks_timings: expected 2 lines, got '${_timing_lines}'")
endif()
foreach(_line IN LISTS _timing_lines)
  if(NOT _line MATCHES "^(tidy|analyzer)\t[0-9]+\t0$")
    set(_failures "${_failures}\n  checks_timings: malformed line '${_line}'")
  endif()
endforeach()

# --- run_tests.sh against a scratch checkout with stub cmake/ctest --------
# The launcher resolves REPO_ROOT from its own location, so copy the three
# launcher files into a scratch tree and put stub cmake/ctest first on PATH.
set(_repo "${SCRATCH_DIR}/repo")
file(MAKE_DIRECTORY "${_repo}/tools" "${_repo}/build/native-release")
file(COPY "${REPO_ROOT}/tools/run_tests.sh" "${REPO_ROOT}/tools/run_native_checks.sh"
          "${REPO_ROOT}/tools/native_common.sh" DESTINATION "${_repo}/tools")
foreach(_tool cmake ctest)
  file(WRITE "${_stub_dir}/${_tool}"
    "#!/usr/bin/env bash\n"
    "printf '${_tool} %s\\n' \"$*\" >> \"\${STUB_LOG}\"\n"
    "exit 0\n")
  file(CHMOD "${_stub_dir}/${_tool}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endforeach()
set(_state "${SCRATCH_DIR}/state")
set(_gate_env ${_checks_env} "PATH=${_stub_dir}:$ENV{PATH}"
  "BIKE_GATE_STATE_DIR=${_state}" "BIKE_CPU_COUNT=8")

# native: configure, build, ordinary CTest, then every kind in order.
file(REMOVE "${_log}")
file(REMOVE_RECURSE "${_state}")
gate_run(gate_native EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "stage" "total"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
file(STRINGS "${_log}" _calls)
list(GET _calls 0 _c0)
list(GET _calls 1 _c1)
list(GET _calls 2 _c2)
list(GET _calls 3 _c3)
if(NOT _c0 MATCHES "^cmake --preset release$"
   OR NOT _c1 MATCHES "^cmake --build --preset release --parallel 8$"
   OR NOT _c2 MATCHES "^ctest --test-dir .* -LE slow\\|package "
   OR NOT _c3 MATCHES "^checks scripts ")
  set(_failures "${_failures}\n  gate_native: unexpected call order '${_calls}'")
endif()
file(STRINGS "${_state}/timings.tsv" _journal)
list(LENGTH _journal _journal_count)
if(NOT _journal_count EQUAL 1 OR NOT _journal MATCHES "\tnative\trelease\t-\t8\tpassed\t")
  set(_failures "${_failures}\n  gate_native: journal '${_journal}'")
endif()

# --from tidy: configure and build still run, ordinary CTest is skipped.
file(REMOVE "${_log}")
gate_run(gate_from EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "PARTIAL"
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from tidy)
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "ctest " _ctest_pos)
string(FIND "${_calls}" "checks cppcheck" _cpp_pos)
string(FIND "${_calls}" "checks tidy" _tidy_pos)
if(NOT _ctest_pos EQUAL -1 OR NOT _cpp_pos EQUAL -1 OR _tidy_pos EQUAL -1)
  set(_failures "${_failures}\n  gate_from: unexpected calls '${_calls}'")
endif()
file(STRINGS "${_state}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tnative\trelease\ttidy\t8\tpassed\t")
  set(_failures "${_failures}\n  gate_from: journal line '${_last}'")
endif()

# --from is meaningless for quick and must name a known stage.
gate_run(gate_from_quick EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" quick release --from tidy)
gate_run(gate_from_bogus EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from bogus)
gate_run(gate_from_slow_native EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from slow)

# A failure is journaled as failed and keeps the exit code.
gate_run(gate_failed EXPECT_EXIT 5 ENV ${_gate_env} "STUB_FAIL_odr=5"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
file(STRINGS "${_state}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tfailed\t")
  set(_failures "${_failures}\n  gate_failed: journal line '${_last}'")
endif()

# (run_tests.sh cases are appended by later tasks above this line.)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_gate contract failed:${_failures}")
endif()
message(STATUS "tooling_gate contract passed")
