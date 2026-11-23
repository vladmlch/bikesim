# tooling_gate — fast contract for the gate launchers (tools/run_tests.sh,
# tools/run_native_checks.sh, tools/check_change.sh) driven against stub
# cmake/ctest/checks executables. Runs no compiler and no real sweep, so it stays in the
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

# A stub bike-native-checks: appends its --kind to calls.log — plus the
# contents of the file named by --only-sources when that flag is present —
# and exits with the code named by STUB_FAIL_<kind> (dashes become
# underscores), else 0.
set(_stub_dir "${SCRATCH_DIR}/stubs")
file(MAKE_DIRECTORY "${_stub_dir}")
# Shell ${...} inside these CMake strings is escaped as \${...} so CMake does
# not expand it at write time (same convention as b29_mock_tool).
file(WRITE "${_stub_dir}/bike-native-checks"
  "#!/usr/bin/env bash\n"
  "kind=''\n"
  "only=''\n"
  "while (($#)); do\n"
  "  [[ $1 == --kind ]] && kind=$2\n"
  "  [[ $1 == --only-sources ]] && only=$2\n"
  "  shift\n"
  "done\n"
  "printf 'checks %s jobs=%s\\n' \"$kind\" \"\${BIKE_CHECK_JOBS:-unset}\" >> \"\${STUB_LOG}\"\n"
  "[[ -n $only ]] && cat \"$only\" >> \"\${STUB_LOG}\"\n"
  "var=\"STUB_FAIL_\${kind//-/_}\"\n"
  "exit \"\${!var:-0}\"\n")
file(CHMOD "${_stub_dir}/bike-native-checks"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

set(_build "${SCRATCH_DIR}/build")
file(MAKE_DIRECTORY "${_build}")
set(_log "${SCRATCH_DIR}/calls.log")
set(_order scripts cppcheck context diagnostic-controls contracts
  frontends headers odr tidy)
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
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from headers "${_build}")
expect_file_lines(checks_from "${_log}"
  "checks headers jobs=unset" "checks odr jobs=unset" "checks tidy jobs=unset")

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
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from odr "${_build}")
file(STRINGS "${_timings}" _timing_lines)
list(LENGTH _timing_lines _timing_count)
if(NOT _timing_count EQUAL 2)
  set(_failures "${_failures}\n  checks_timings: expected 2 lines, got '${_timing_lines}'")
endif()
foreach(_line IN LISTS _timing_lines)
  if(NOT _line MATCHES "^(odr|tidy)\t[0-9]+\t0$")
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
file(COPY "${REPO_ROOT}/tools/lib" DESTINATION "${_repo}/tools")
# STUB_FAIL_cmake / STUB_FAIL_ctest make the stub exit non-zero (default 0).
foreach(_tool cmake ctest)
  file(WRITE "${_stub_dir}/${_tool}"
    "#!/usr/bin/env bash\n"
    "printf '${_tool} %s\\n' \"$*\" >> \"\${STUB_LOG}\"\n"
    "exit \"\${STUB_FAIL_${_tool}:-0}\"\n")
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

# --- another live gate halves the share; a stale entry is ignored ---------
execute_process(COMMAND bash -c "sleep 60 >/dev/null 2>&1 & echo $!"
  OUTPUT_VARIABLE _live_pid OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND bash -c "sleep 0 & p=$!; wait $p; echo $p"
  OUTPUT_VARIABLE _dead_pid OUTPUT_STRIP_TRAILING_WHITESPACE)
file(MAKE_DIRECTORY "${_state}/active")
file(WRITE "${_state}/active/${_live_pid}" "${_live_pid}\t0\t/elsewhere\tnative\n")
file(WRITE "${_state}/active/${_dead_pid}" "${_dead_pid}\t0\t/stale\tnative\n")
file(REMOVE "${_log}")
gate_run(gate_share EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "1 other native gate(s) running" "/elsewhere"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
execute_process(COMMAND kill "${_live_pid}")
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "--parallel 4" _par)
string(FIND "${_calls}" "checks tidy jobs=4" _tidy_jobs)
if(_par EQUAL -1 OR _tidy_jobs EQUAL -1)
  set(_failures "${_failures}\n  gate_share: expected 4 of 8 cores, calls '${_calls}'")
endif()
if(EXISTS "${_state}/active/${_dead_pid}")
  set(_failures "${_failures}\n  gate_share: stale registry entry was not removed")
endif()
file(GLOB _left "${_state}/active/*")
list(LENGTH _left _left_count)
if(NOT _left_count EQUAL 1)
  set(_failures "${_failures}\n  gate_share: own registry entry left behind: '${_left}'")
endif()
# An explicit BIKE_CHECK_JOBS wins over the computed share.
file(REMOVE "${_log}")
gate_run(gate_jobs_override EXPECT_EXIT 0 ENV ${_gate_env} "BIKE_CHECK_JOBS=3"
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from tidy)
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "checks tidy jobs=3" _override)
if(_override EQUAL -1)
  set(_failures "${_failures}\n  gate_jobs_override: calls '${_calls}'")
endif()

# --- check_change.sh: level-1 gate against a scratch git history -----------
# The scratch _repo gets a real history (empty base, then a C++ source plus
# a docs file) so the script's merge-base selection sees committed changes.
file(COPY "${REPO_ROOT}/tools/check_change.sh" DESTINATION "${_repo}/tools")
file(MAKE_DIRECTORY "${_repo}/native/src" "${_repo}/docs")
execute_process(COMMAND git init -b main
  WORKING_DIRECTORY "${_repo}"
  RESULT_VARIABLE _git_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT _git_rc EQUAL 0)
  set(_failures "${_failures}\n  check_change: git init failed")
endif()
execute_process(COMMAND git -c user.name=t -c user.email=t@t
  commit --allow-empty -m base
  WORKING_DIRECTORY "${_repo}" OUTPUT_QUIET ERROR_QUIET)
file(WRITE "${_repo}/native/src/x.cpp" "int x() { return 0; }\n")
file(WRITE "${_repo}/docs/a.md" "# a\n")
execute_process(COMMAND git add native/src/x.cpp docs/a.md
  WORKING_DIRECTORY "${_repo}" OUTPUT_QUIET ERROR_QUIET)
execute_process(COMMAND git -c user.name=t -c user.email=t@t
  commit -m change
  WORKING_DIRECTORY "${_repo}" OUTPUT_QUIET ERROR_QUIET)

# A C++ change runs the incremental build, every ordinary test and the three
# per-file kinds on the changed list only; docs/a.md never reaches the list.
file(REMOVE "${_log}")
gate_run(change_cpp EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "1 changed C++ file(s)"
  COMMAND bash "${_repo}/tools/check_change.sh" HEAD~1)
expect_file_lines(change_cpp "${_log}"
  "cmake --build --preset release"
  "ctest --preset release"
  "checks cppcheck jobs=unset"
  "${_repo}/native/src/x.cpp"
  "checks frontends jobs=unset"
  "${_repo}/native/src/x.cpp"
  "checks tidy jobs=unset"
  "${_repo}/native/src/x.cpp")

# A docs-only change selects no C++ file; the analysis stage is skipped.
file(APPEND "${_repo}/docs/a.md" "more\n")
file(REMOVE "${_log}")
gate_run(change_docs_only EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "no C++ source or header changed"
  COMMAND bash "${_repo}/tools/check_change.sh")
expect_file_lines(change_docs_only "${_log}"
  "cmake --build --preset release"
  "ctest --preset release")

# --- check_change.sh: base rule, --uncommitted, tree id, journal, AGENTS.md ---
# tree_id_run(<var> <dir>) — stdout of tools/lib/tree_id.sh <dir> into <var>
# (caller scope); a non-zero exit is a contract failure.
function(tree_id_run var dir)
  execute_process(COMMAND bash "${_repo}/tools/lib/tree_id.sh" "${dir}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT _rc EQUAL 0)
    set(_failures "${_failures}\n  tree_id(${dir}): exit ${_rc}: ${_err}" PARENT_SCOPE)
  endif()
  set(${var} "${_out}" PARENT_SCOPE)
endfunction()

# scratch_git(<name> <git args...>) — git in the scratch repo; failure is a
# contract failure.
function(scratch_git name)
  execute_process(COMMAND git -c user.name=t -c user.email=t@t ${ARGN}
    WORKING_DIRECTORY "${_repo}" RESULT_VARIABLE _rc
    OUTPUT_QUIET ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    set(_failures "${_failures}\n  ${name}: git ${ARGN} failed: ${_err}" PARENT_SCOPE)
  endif()
endfunction()

# The tree id is a 64-digit sha256, stable on an unchanged tree, the same
# from the sourced function, and unmoved by a bare touch of a tracked file.
tree_id_run(_id_a "${_repo}")
string(LENGTH "${_id_a}" _id_len)
if(NOT _id_len EQUAL 64 OR NOT _id_a MATCHES "^[0-9a-f]+$")
  set(_failures "${_failures}\n  tree_id_shape: '${_id_a}' is not a sha256 hex digest")
endif()
tree_id_run(_id_b "${_repo}")
if(NOT _id_b STREQUAL _id_a)
  set(_failures "${_failures}\n  tree_id_stable: '${_id_a}' then '${_id_b}' on an unchanged tree")
endif()
execute_process(COMMAND bash -c [=[source "$1"; bike_tree_id "$2"]=] bash
  "${_repo}/tools/lib/tree_id.sh" "${_repo}"
  RESULT_VARIABLE _src_rc OUTPUT_VARIABLE _sourced OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _src_rc EQUAL 0 OR NOT _sourced STREQUAL _id_a)
  set(_failures "${_failures}\n  tree_id_sourced: '${_sourced}' (exit ${_src_rc}), script gave '${_id_a}'")
endif()
# Touching a tracked file (same bytes, newer mtime) must neither change the
# id nor rewrite .git/index. The sleep makes the mtime newer than the index.
file(SHA256 "${_repo}/.git/index" _index_before)
execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 2)
file(WRITE "${_repo}/native/src/x.cpp" "int x() { return 0; }\n")
tree_id_run(_id_c "${_repo}")
file(SHA256 "${_repo}/.git/index" _index_after)
if(NOT _index_before STREQUAL _index_after)
  set(_failures "${_failures}\n  tree_id_index: tree_id rewrote .git/index")
endif()
if(NOT _id_c STREQUAL _id_a)
  set(_failures "${_failures}\n  tree_id_touch: touching a tracked file changed the id")
endif()
# An edit to a tracked file and a new untracked file each change the id.
file(APPEND "${_repo}/docs/a.md" "edit\n")
tree_id_run(_id_d "${_repo}")
if(_id_d STREQUAL _id_c)
  set(_failures "${_failures}\n  tree_id_edit: an edit to docs/a.md left the id unchanged")
endif()
file(WRITE "${_repo}/docs/new.md" "# new\n")
tree_id_run(_id_e "${_repo}")
if(_id_e STREQUAL _id_d)
  set(_failures "${_failures}\n  tree_id_untracked: a new untracked file left the id unchanged")
endif()
# The id does not depend on the caller's directory: run from the real checkout
# root, which has no docs/new.md, it must still see the scratch repo's files.
execute_process(COMMAND bash "${_repo}/tools/lib/tree_id.sh" "${_repo}"
  WORKING_DIRECTORY "${REPO_ROOT}"
  RESULT_VARIABLE _cwd_rc OUTPUT_VARIABLE _id_cwd OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _cwd_rc EQUAL 0 OR NOT _id_cwd STREQUAL _id_e)
  set(_failures "${_failures}\n  tree_id_cwd: from the checkout root '${_id_cwd}' (exit ${_cwd_rc}), expected '${_id_e}'")
endif()
# A plain directory nested in the checkout's build/ tree has no repository of
# its own: it must report nogit, not borrow the outer checkout's HEAD.
set(_plain "${SCRATCH_DIR}/plain")
file(MAKE_DIRECTORY "${_plain}")
file(WRITE "${_plain}/f.txt" "x\n")
execute_process(COMMAND bash "${_repo}/tools/lib/tree_id.sh" "${_plain}"
  RESULT_VARIABLE _plain_rc OUTPUT_VARIABLE _plain_id OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _plain_rc EQUAL 0 OR NOT _plain_id STREQUAL "nogit")
  set(_failures "${_failures}\n  tree_id_nogit: '${_plain_id}' (exit ${_plain_rc}) for a dir without its own .git")
endif()

# On main itself an explicit HEAD or main base is accepted, as before.
gate_run(base_head_on_main EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "passed in"
  COMMAND bash "${_repo}/tools/check_change.sh" HEAD)
gate_run(base_main_on_main EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "passed in"
  COMMAND bash "${_repo}/tools/check_change.sh" main)

# Branch with its own commits beyond main: an explicit HEAD base is refused
# before any build runs; an explicit main base is still accepted.
scratch_git(feature_branch checkout -q -b feature)
file(WRITE "${_repo}/native/src/y.cpp" "int y() { return 1; }\n")
file(WRITE "${_repo}/docs/b.md" "# b\n")
scratch_git(feature_commit add docs/a.md docs/b.md native/src/y.cpp)
scratch_git(feature_commit commit -q -m feature)
file(REMOVE "${_log}")
gate_run(base_head_refused EXPECT_EXIT 2 ENV ${_gate_env}
  EXPECT_OUTPUT "--uncommitted" "default: main"
  COMMAND bash "${_repo}/tools/check_change.sh" HEAD)
if(EXISTS "${_log}")
  set(_failures "${_failures}\n  base_head_refused: a refused base still ran the build")
endif()
gate_run(base_main_on_branch EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "1 changed C++ file(s) (changes since main)"
  COMMAND bash "${_repo}/tools/check_change.sh" main)

# --uncommitted sees only the uncommitted x.cpp, not the committed y.cpp.
file(WRITE "${_repo}/native/src/x.cpp" "int x() { return 1; }\n")
file(REMOVE "${_log}")
gate_run(uncommitted_only EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "1 changed C++ file(s) (uncommitted changes)"
  COMMAND bash "${_repo}/tools/check_change.sh" --uncommitted)
expect_file_lines(uncommitted_only "${_log}"
  "cmake --build --preset release"
  "ctest --preset release"
  "checks cppcheck jobs=unset"
  "${_repo}/native/src/x.cpp"
  "checks frontends jobs=unset"
  "${_repo}/native/src/x.cpp"
  "checks tidy jobs=unset"
  "${_repo}/native/src/x.cpp")
gate_run(uncommitted_with_base EXPECT_EXIT 2 ENV ${_gate_env}
  EXPECT_OUTPUT "takes no base"
  COMMAND bash "${_repo}/tools/check_change.sh" --uncommitted main)

# A passing run journals one change row: 11 columns, jobs from BIKE_CPU_COUNT,
# and tree= equal to the id of the tree it checked (unchanged by the run).
file(REMOVE "${_state}/timings.tsv")
gate_run(journal_change_pass EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "passed in"
  COMMAND bash "${_repo}/tools/check_change.sh" main)
file(STRINGS "${_state}/timings.tsv" _journal)
list(LENGTH _journal _journal_count)
list(GET _journal -1 _last)
tree_id_run(_id_now "${_repo}")
if(NOT _journal_count EQUAL 1
   OR NOT _last MATCHES "^[^\t]+\t[^\t]+\t[0-9a-f]+\tchange\trelease\t-\t8\tpassed\t[0-9]+\tselected=[0-9]+\ttree=[0-9a-f]+$"
   OR NOT _last MATCHES "\ttree=${_id_now}$")
  set(_failures "${_failures}\n  journal_change_pass: journal '${_journal}', tree '${_id_now}'")
endif()

# A failing run is journaled too, with status failed and the exit kept.
gate_run(journal_change_fail EXPECT_EXIT 3 ENV ${_gate_env} "STUB_FAIL_ctest=3"
  COMMAND bash "${_repo}/tools/check_change.sh" main)
file(STRINGS "${_state}/timings.tsv" _journal)
list(LENGTH _journal _journal_count)
list(GET _journal -1 _last)
if(NOT _journal_count EQUAL 2 OR NOT _last MATCHES "\tchange\trelease\t-\t8\tfailed\t[0-9]+\tselected=[0-9]+\ttree=${_id_now}$")
  set(_failures "${_failures}\n  journal_change_fail: journal '${_journal}'")
endif()

# AGENTS.md above 18000 bytes gets a warning naming the size and agy's 24000
# cap; at exactly 18000 bytes there is none.
string(REPEAT "a" 18000 _agents_quiet)
file(WRITE "${_repo}/AGENTS.md" "${_agents_quiet}")
gate_run(agents_size_quiet EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "passed in"
  COMMAND bash "${_repo}/tools/check_change.sh" main)
string(FIND "${agents_size_quiet_output}" "agy truncates" _pos)
if(NOT _pos EQUAL -1)
  set(_failures "${_failures}\n  agents_size_quiet: warned at 18000 bytes")
endif()
string(REPEAT "a" 18001 _agents_over)
file(WRITE "${_repo}/AGENTS.md" "${_agents_over}")
gate_run(agents_size_warning EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "AGENTS.md is 18001 bytes" "agy truncates rule files at 24000 bytes"
  COMMAND bash "${_repo}/tools/check_change.sh" main)
file(REMOVE "${_repo}/AGENTS.md")

# --- run_tests.sh gate preconditions: native and full need a passed level 1 ---
# A scratch repo with its own history, so the tree id is real (the nogit path
# is covered by gate_native above, which has no .git). Each case journals into
# its own state dir; seeded rows name the real tree ids from tree_id_run.
set(_pre "${SCRATCH_DIR}/precondition")
set(_pre_states "${SCRATCH_DIR}/precondition-state")
file(REMOVE_RECURSE "${_pre}" "${_pre_states}")
file(MAKE_DIRECTORY "${_pre}/tools" "${_pre}/build/native-release" "${_pre}/docs")
file(COPY "${REPO_ROOT}/tools/run_tests.sh" "${REPO_ROOT}/tools/run_native_checks.sh"
          "${REPO_ROOT}/tools/native_common.sh" DESTINATION "${_pre}/tools")
file(COPY "${REPO_ROOT}/tools/lib" DESTINATION "${_pre}/tools")
file(WRITE "${_pre}/docs/a.md" "# a\n")
execute_process(COMMAND bash -c "git init -q -b main && git add -A && git -c user.name=t -c user.email=t@t commit -q -m base"
  WORKING_DIRECTORY "${_pre}" RESULT_VARIABLE _pre_git_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT _pre_git_rc EQUAL 0)
  set(_failures "${_failures}\n  precondition: git setup failed")
endif()
tree_id_run(_pre_id_old "${_pre}")
file(APPEND "${_pre}/docs/a.md" "edit\n")
tree_id_run(_pre_id "${_pre}")

# pre_seed(<state-dir> <profile> <status> <tree-id>) — one journal row in the
# 11-column shape the launchers write.
function(pre_seed dir profile status tree)
  file(MAKE_DIRECTORY "${dir}")
  file(APPEND "${dir}/timings.tsv"
    "2026-11-23T00:00:00Z\t${_pre}\tabc1234\t${profile}\trelease\t-\t8\t${status}\t1\tseeded=1\ttree=${tree}\n")
endfunction()

# journal_count(<var> <state-dir>) — rows in timings.tsv (0 when absent).
function(journal_count var dir)
  set(_n 0)
  if(EXISTS "${dir}/timings.tsv")
    file(STRINGS "${dir}/timings.tsv" _rows)
    list(LENGTH _rows _n)
  endif()
  set(${var} "${_n}" PARENT_SCOPE)
endfunction()

# refused_cleanly(<name> <state-dir> <rows-before>) — a refused run called no
# stub, journaled no new row and left no active-gate entry.
function(refused_cleanly name dir before)
  journal_count(_n "${dir}")
  if(EXISTS "${_log}")
    set(_failures "${_failures}\n  ${name}: refused run still called a stub" PARENT_SCOPE)
  endif()
  if(NOT _n EQUAL before)
    set(_failures "${_failures}\n  ${name}: refused run left ${_n} journal rows, expected ${before}" PARENT_SCOPE)
  endif()
  file(GLOB _active "${dir}/active/*")
  list(LENGTH _active _active_count)
  if(NOT _active_count EQUAL 0)
    set(_failures "${_failures}\n  ${name}: refused run left an active-gate entry: ${_active}" PARENT_SCOPE)
  endif()
endfunction()

# Inherited BIKE_GATE_OVERRIDE would bypass the checks under test: unset it.
set(_pre_env ${_checks_env} "PATH=${_stub_dir}:$ENV{PATH}" "BIKE_CPU_COUNT=8"
  --unset=BIKE_GATE_OVERRIDE)

# A native run with no journal at all is refused before anything starts.
set(_s "${_pre_states}/none")
file(REMOVE "${_log}")
gate_run(pre_native_no_change EXPECT_EXIT 2 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "check_change.sh" "BIKE_GATE_OVERRIDE"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
refused_cleanly(pre_native_no_change "${_s}" 0)

# A passed change row for this tree lets native proceed and be journaled.
set(_s "${_pre_states}/change")
pre_seed("${_s}" change passed "${_pre_id}")
file(REMOVE "${_log}")
gate_run(pre_native_with_change EXPECT_EXIT 0 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "stage" "total"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
file(STRINGS "${_s}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tnative\trelease\t-\t8\tpassed\t[^\t]*\t[^\t]*\ttree=${_pre_id}$")
  set(_failures "${_failures}\n  pre_native_with_change: journal line '${_last}'")
endif()

# A failed change row does not satisfy the precondition.
set(_s "${_pre_states}/failed")
pre_seed("${_s}" change failed "${_pre_id}")
file(REMOVE "${_log}")
gate_run(pre_native_failed_change EXPECT_EXIT 2 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "check_change.sh"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
refused_cleanly(pre_native_failed_change "${_s}" 1)

# A change row for another tree (the tree before the edit) does not satisfy it.
set(_s "${_pre_states}/other-tree")
pre_seed("${_s}" change passed "${_pre_id_old}")
file(REMOVE "${_log}")
gate_run(pre_native_other_tree EXPECT_EXIT 2 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "check_change.sh"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
refused_cleanly(pre_native_other_tree "${_s}" 1)

# native is refused once a passed full run covers the same tree.
set(_s "${_pre_states}/full-covers")
pre_seed("${_s}" change passed "${_pre_id}")
pre_seed("${_s}" full passed "${_pre_id}")
file(REMOVE "${_log}")
gate_run(pre_native_after_full EXPECT_EXIT 2 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "already covers"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
refused_cleanly(pre_native_after_full "${_s}" 2)

# full is not refused by a passed full row: it is the run that produces one.
set(_s "${_pre_states}/full-covers")
file(REMOVE "${_log}")
gate_run(pre_full_after_full EXPECT_EXIT 0 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "stage"
  COMMAND bash "${_pre}/tools/run_tests.sh" full)
file(STRINGS "${_s}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tfull\trelease\t-\t8\tpassed\t[^\t]*\t[^\t]*\ttree=${_pre_id}$")
  set(_failures "${_failures}\n  pre_full_after_full: journal line '${_last}'")
endif()

# full still needs a change row: a full row alone does not stand in for it.
set(_s "${_pre_states}/full-only")
pre_seed("${_s}" full passed "${_pre_id}")
file(REMOVE "${_log}")
gate_run(pre_full_without_change EXPECT_EXIT 2 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "check_change.sh"
  COMMAND bash "${_pre}/tools/run_tests.sh" full)
refused_cleanly(pre_full_without_change "${_s}" 1)

# BIKE_GATE_OVERRIDE proceeds without any row; the reason is journaled as a
# 12th column, with its tab turned into a space.
set(_s "${_pre_states}/override")
file(REMOVE "${_log}")
gate_run(pre_override EXPECT_EXIT 0 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  "BIKE_GATE_OVERRIDE=emergency\tfix"
  EXPECT_OUTPUT "BIKE_GATE_OVERRIDE set" "stage"
  COMMAND bash "${_pre}/tools/run_tests.sh" native)
file(STRINGS "${_s}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tnative\trelease\t-\t8\tpassed\t[^\t]*\t[^\t]*\ttree=${_pre_id}\toverride=emergency fix$")
  set(_failures "${_failures}\n  pre_override: journal line '${_last}'")
endif()

# quick is not gated: it runs with no journal and still writes the tree column.
set(_s "${_pre_states}/quick")
file(REMOVE "${_log}")
gate_run(pre_quick_ungated EXPECT_EXIT 0 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  COMMAND bash "${_pre}/tools/run_tests.sh" quick)
file(STRINGS "${_s}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tquick\trelease\t-\t8\tpassed\t[^\t]*\t[^\t]*\ttree=${_pre_id}$")
  set(_failures "${_failures}\n  pre_quick_ungated: journal line '${_last}'")
endif()

# --from is not gated either: an intermediate re-check runs with no change row.
set(_s "${_pre_states}/from")
file(REMOVE "${_log}")
gate_run(pre_from_ungated EXPECT_EXIT 0 ENV ${_pre_env} "BIKE_GATE_STATE_DIR=${_s}"
  EXPECT_OUTPUT "PARTIAL"
  COMMAND bash "${_pre}/tools/run_tests.sh" native release --from tidy)

# (run_tests.sh cases are appended by later tasks above this line.)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_gate contract failed:${_failures}")
endif()
message(STATUS "tooling_gate contract passed")
