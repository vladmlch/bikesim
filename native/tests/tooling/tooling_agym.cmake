# tooling_agym — contract for tools/agy/agym and its quota parser (lib_agym.sh).
# Covers: the --parse-log report of a 429 reset line (local reset time and the
# resume line), the conversation id taken over the latency line's trajectory_id,
# the latest of several 429 lines and its "reset after" wording, silence without
# a 429, the raw-duration fallback when the
# time does not parse, --help, and a run under a fake agy on PATH: the argv
# (skip-permissions, per-session --log-file, --add-dir for each reachable target,
# the note for a missing one), the exit status passthrough, and the 429 scan of
# the log the fake agy writes. Nothing contacts agy. The scripts run under
# /bin/bash when it exists (bash 3.2 on macOS), so the 3.2 contract is tested.
# Stays in the ordinary (non-slow) CTest set.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_agym requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")

if(EXISTS "/bin/bash")
  set(_bash "/bin/bash")
else()
  set(_bash "bash")
endif()
set(_agym "${REPO_ROOT}/tools/agy/agym")

# Fixtures use the glog line shapes of ~/.gemini/antigravity-cli/log, copied from
# a real log (cli-20261117_215701.log). The agym- prefixed names carry the year,
# as the wrapper's own log files do. The latency line carries both the
# conversation id (cascade_id) and the trajectory_id, which must not be picked.
set(_quota_name "agym-20261117T231838Z-quota.log")
file(WRITE "${SCRATCH_DIR}/${_quota_name}" [==[I1117 23:18:37.053611   31090 conversation_manager.go:884] Forwarding user message to conversation a83dad8b-7b39-49a0-9316-29a1b87b49c8 (items=1, media=0)
I1117 23:18:37.064177   31090 latency_breakdown.go:168] SEND_USER_CASCADE_MESSAGE_LATENCY latency breakdown: map[active_user_state_latency_ms:0 blocking:false cascade_id:a83dad8b-7b39-49a0-9316-29a1b87b49c8 cascade_send_latency_ms:2 hydrate_slash_commands_latency_ms:0 load_trajectory_latency_ms:0 media_count:0 media_summaries_latency_ms:0 message_origin:AGENT_MESSAGE_ORIGIN_UNSPECIFIED resolve_agent_config_latency_ms:2 save_media_latency_ms:0 saved_media_count:0 status:OK total_latency_ms:5 trajectory_id:d7db33a0-1827-40ff-8ff9-efb01b21c24f]
E1117 23:18:38.409415   29699 errorreport.go:307] agent executor error: generating and executing: RESOURCE_EXHAUSTED (code 429): Individual quota reached. Please upgrade your subscription to increase your limits. Resets in 1h45m13s.
E1117 23:18:38.411286   29699 errorreport.go:307] generating and executing: RESOURCE_EXHAUSTED (code 429): Individual quota reached. Please upgrade your subscription to increase your limits. Resets in 1h45m13s.
]==])

# Two 429s. The first ("Resets in", under conversation a83dad8b) is stale: the
# user sent a new message to conversation 4d1b0e41 later, and the latest 429 is a
# monitor line in the "reset after" wording (as in cli-20261122_194826.log). The
# report must use the latest line and its conversation.
set(_multi_name "agym-20261117T090000Z-multi.log")
file(WRITE "${SCRATCH_DIR}/${_multi_name}" [==[I1117 08:59:58.000001   31090 conversation_manager.go:884] Forwarding user message to conversation a83dad8b-7b39-49a0-9316-29a1b87b49c8 (items=1, media=0)
E1117 08:59:59.409415   29699 errorreport.go:307] agent executor error: generating and executing: RESOURCE_EXHAUSTED (code 429): Individual quota reached. Please upgrade your subscription to increase your limits. Resets in 1h45m13s.
I1117 09:30:00.000001   31090 conversation_manager.go:884] Forwarding user message to conversation 4d1b0e41-63b7-4fd7-852a-4276583e6ad6 (items=2, media=0)
I1117 09:30:00.009000   31090 latency_breakdown.go:168] SEND_USER_CASCADE_MESSAGE_LATENCY latency breakdown: map[active_user_state_latency_ms:0 blocking:false cascade_id:4d1b0e41-63b7-4fd7-852a-4276583e6ad6 cascade_send_latency_ms:0 hydrate_slash_commands_latency_ms:0 load_trajectory_latency_ms:0 media_count:0 media_summaries_latency_ms:0 message_origin:AGENT_MESSAGE_ORIGIN_UNSPECIFIED resolve_agent_config_latency_ms:0 save_media_latency_ms:0 saved_media_count:0 status:OK total_latency_ms:1 trajectory_id:9b1f0c52-3e7a-4d1e-8c55-2a6f0d9e7b13]
I1117 09:31:01.000002   31090 runner.go:957] Monitor sir_flash_lite FINISHED: triggered=false, reason="Failed: LLM call failed: modelAPI.Generate failed: RESOURCE_EXHAUSTED (code 429): You have exhausted your capacity on this model. Your quota will reset after 19m9s.", duration=118.370709ms
]==])

# Clean log: a 500 with "Resets in" is not a quota reset and must stay silent.
set(_clean_name "agym-20261117T231838Z-clean.log")
file(WRITE "${SCRATCH_DIR}/${_clean_name}" [==[I1117 23:18:37.053611   31090 conversation_manager.go:884] Forwarding user message to conversation a83dad8b-7b39-49a0-9316-29a1b87b49c8 (items=1, media=0)
E1117 23:18:38.409415   29699 errorreport.go:307] generating and executing: code 500 Internal: upstream hiccup. Resets in 5m.
I1117 23:18:40.000000   29699 done
]==])

# Unparseable time (month 13): the raw duration is printed instead.
set(_badtime_name "agym-20261117T231838Z-badtime.log")
file(WRITE "${SCRATCH_DIR}/${_badtime_name}" [==[E1317 25:61:00.000000   1 x.go:1] RESOURCE_EXHAUSTED (code 429): quota. Resets in 2h3m4s.
]==])

# --- --parse-log: 429 report ---------------------------------------------------
execute_process(COMMAND "${_bash}" "${_agym}" --parse-log "${SCRATCH_DIR}/${_quota_name}"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0 OR NOT "${_out}" STREQUAL "")
  string(APPEND _failures "\n  parse_quota: rc=${_rc}, stdout '${_out}' (expected rc 0, empty stdout)")
endif()
# 23:18:38 + 1h45m13s is 01:03:51 on the next local day.
string(FIND "${_err}" "quota resets at 2026-11-18 01:03:51 local time" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  parse_quota: reset time missing from stderr: '${_err}'")
endif()
string(FIND "${_err}" "Resets in 1h45m13s" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  parse_quota: raw duration missing from stderr: '${_err}'")
endif()
# The resume line must be exactly the command, alone on its line.
set(_resume_line "\nagym --conversation=a83dad8b-7b39-49a0-9316-29a1b87b49c8\n")
string(FIND "\n${_err}\n" "${_resume_line}" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  parse_quota: exact resume line missing from stderr: '${_err}'")
endif()
# The latency line's trajectory_id names no brain directory and must not be resumed.
string(FIND "${_err}" "d7db33a0" _pos)
if(NOT _pos EQUAL -1)
  string(APPEND _failures "\n  parse_quota: trajectory id reached the report: '${_err}'")
endif()

# --- --parse-log: the latest 429 wins, with its wording and conversation ---------
execute_process(COMMAND "${_bash}" "${_agym}" --parse-log "${SCRATCH_DIR}/${_multi_name}"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
# 09:31:01 + 19m9s is 09:50:10 the same day; the first 429 (1h45m13s) is stale.
string(FIND "${_err}" "quota resets at 2026-11-17 09:50:10 local time" _pos)
if(NOT _rc EQUAL 0 OR NOT "${_out}" STREQUAL "" OR _pos EQUAL -1)
  string(APPEND _failures "\n  parse_multi: expected the latest reset at 09:50:10, rc=${_rc}, stdout '${_out}', stderr '${_err}'")
endif()
string(FIND "${_err}" "reset after 19m9s" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  parse_multi: 'reset after' wording missing from stderr: '${_err}'")
endif()
string(FIND "${_err}" "1h45m13s" _pos)
if(NOT _pos EQUAL -1)
  string(APPEND _failures "\n  parse_multi: the stale first 429 was reported: '${_err}'")
endif()
string(FIND "\n${_err}\n" "\nagym --conversation=4d1b0e41-63b7-4fd7-852a-4276583e6ad6\n" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  parse_multi: resume line does not name the conversation of the latest 429: '${_err}'")
endif()

# --- --parse-log: silence without a quota reset --------------------------------
foreach(_case clean badtime)
  if(_case STREQUAL "clean")
    set(_file "${SCRATCH_DIR}/${_clean_name}")
  else()
    set(_file "${SCRATCH_DIR}/${_badtime_name}")
  endif()
  execute_process(COMMAND "${_bash}" "${_agym}" --parse-log "${_file}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  if(_case STREQUAL "clean")
    if(NOT _rc EQUAL 0 OR NOT "${_err}" STREQUAL "")
      string(APPEND _failures "\n  parse_clean: expected rc 0 and silence, got rc=${_rc}, stderr '${_err}'")
    endif()
  else()
    # Raw fallback: no local time, the duration is still reported.
    string(FIND "${_err}" "quota resets in 2h3m4s" _pos)
    string(FIND "${_err}" "local time" _lt)
    if(_pos EQUAL -1 OR NOT _lt EQUAL -1 OR NOT _rc EQUAL 0)
      string(APPEND _failures "\n  parse_badtime: expected raw duration and rc 0, got rc=${_rc}, stderr '${_err}'")
    endif()
  endif()
endforeach()

execute_process(COMMAND "${_bash}" "${_agym}" --parse-log "${SCRATCH_DIR}/does-not-exist.log"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0 OR NOT "${_err}" STREQUAL "")
  string(APPEND _failures "\n  parse_missing_file: expected rc 0 and silence, got rc=${_rc}, stderr '${_err}'")
endif()

# --- --help ---------------------------------------------------------------------
execute_process(COMMAND "${_bash}" "${_agym}" --help
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(FIND "${_out}" "usage: agym" _pos)
if(NOT _rc EQUAL 0 OR _pos EQUAL -1)
  string(APPEND _failures "\n  help: expected rc 0 and a usage line, got rc=${_rc}, stdout '${_out}'")
endif()

# --- a run under a fake agy ----------------------------------------------------
# The fake agy records its argv one argument per line in $FAKE_ARGV, copies
# $FAKE_LOG_SRC over the --log-file path when set, and exits $FAKE_RC.
set(_bin "${SCRATCH_DIR}/bin")
file(MAKE_DIRECTORY "${_bin}")
file(WRITE "${_bin}/agy" [==[#!/bin/sh
: > "$FAKE_ARGV"
prev=""
for arg in "$@"; do
  printf '%s\n' "$arg" >> "$FAKE_ARGV"
  if [ "$prev" = "--log-file" ] && [ -n "${FAKE_LOG_SRC:-}" ]; then
    cp "$FAKE_LOG_SRC" "$arg"
  fi
  prev="$arg"
done
exit "${FAKE_RC:-0}"
]==])
file(CHMOD "${_bin}/agy"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# A repository copy with agym at tools/agy/ and .scratch / .deps symlinks to the
# given targets. A target that does not exist models an unmounted SSD.
function(make_repo name scratch_target deps_target)
  set(_repo "${SCRATCH_DIR}/${name}")
  file(COPY "${REPO_ROOT}/tools/agy/agym" "${REPO_ROOT}/tools/agy/lib_agym.sh"
    DESTINATION "${_repo}/tools/agy")
  file(CREATE_LINK "${scratch_target}" "${_repo}/.scratch" SYMBOLIC)
  file(CREATE_LINK "${deps_target}" "${_repo}/.deps" SYMBOLIC)
endfunction()

# run_agym(<repo> <fake log source> <fake rc> <argv-out> <err-out> <rc-out> <args...>)
function(run_agym repo fake_log fake_rc argv_out err_out rc_out)
  set(_logs "${SCRATCH_DIR}/logs-${repo}")
  set(_argv "${SCRATCH_DIR}/argv-${repo}.txt")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
      "PATH=${_bin}:$ENV{PATH}"
      "AGYM_LOG_DIR=${_logs}"
      "FAKE_ARGV=${_argv}"
      "FAKE_LOG_SRC=${fake_log}"
      "FAKE_RC=${fake_rc}"
      "${_bash}" "${SCRATCH_DIR}/${repo}/tools/agy/agym" ${ARGN}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  # A run that never reached the fake agy leaves no argv file; the empty argv then fails the checks.
  set(_argv_text "")
  if(EXISTS "${_argv}")
    file(READ "${_argv}" _argv_text)
  endif()
  set(${argv_out} "${_argv_text}" PARENT_SCOPE)
  set(${err_out} "${_err}" PARENT_SCOPE)
  set(${rc_out} "${_rc}" PARENT_SCOPE)
endfunction()

# Missing SSD: .scratch dangles, .deps resolves.
file(MAKE_DIRECTORY "${SCRATCH_DIR}/real-deps")
file(REAL_PATH "${SCRATCH_DIR}/real-deps" _deps_phys)
make_repo(repo-missing-ssd "${SCRATCH_DIR}/ssd-not-mounted/work" "${SCRATCH_DIR}/real-deps")

run_agym(repo-missing-ssd "${SCRATCH_DIR}/${_quota_name}" 7 _argv _err _rc --conversation=test-id hello)
string(STRIP "${_argv}" _argv)
string(REPLACE "\n" ";" _args "${_argv}")
list(LENGTH _args _count)
if(NOT _rc EQUAL 7)
  string(APPEND _failures "\n  run_missing_ssd: agym exit ${_rc}, expected agy's 7")
endif()
# Expected argv: skip-permissions, log, add-dir deps, conversation, prompt.
if(NOT _count EQUAL 7)
  string(APPEND _failures "\n  run_missing_ssd: argv has ${_count} items, expected 7: '${_argv}'")
else()
  list(GET _args 0 _a0)
  list(GET _args 1 _a1)
  list(GET _args 2 _a2)
  list(GET _args 3 _a3)
  list(GET _args 4 _a4)
  list(GET _args 5 _a5)
  list(GET _args 6 _a6)
  if(NOT _a0 STREQUAL "--dangerously-skip-permissions" OR NOT _a1 STREQUAL "--log-file")
    string(APPEND _failures "\n  run_missing_ssd: leading flags wrong: '${_a0}' '${_a1}'")
  endif()
  string(FIND "${_a2}" "${SCRATCH_DIR}/logs-repo-missing-ssd/agym-" _pos)
  if(NOT _pos EQUAL 0 OR NOT _a2 MATCHES "agym-[0-9]+T[0-9]+Z-[0-9]+\\.log$")
    string(APPEND _failures "\n  run_missing_ssd: log path '${_a2}' is not the per-session agym log")
  endif()
  if(NOT _a3 STREQUAL "--add-dir" OR NOT _a4 STREQUAL "${_deps_phys}")
    string(APPEND _failures "\n  run_missing_ssd: expected --add-dir for .deps at '${_deps_phys}', got '${_a3}' '${_a4}'")
  endif()
  if(NOT _a5 STREQUAL "--conversation=test-id" OR NOT _a6 STREQUAL "hello")
    string(APPEND _failures "\n  run_missing_ssd: agy arguments not passed through: '${_a5}' '${_a6}'")
  endif()
endif()
string(FIND "${_err}" "agym: skipping --add-dir for .scratch" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  run_missing_ssd: no skip note for .scratch on stderr: '${_err}'")
endif()
# The 429 that the fake agy wrote to --log-file is reported after the exit.
string(FIND "${_err}" "quota resets at 2026-11-18 01:03:51 local time" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  run_missing_ssd: post-exit 429 report missing: '${_err}'")
endif()
string(FIND "\n${_err}\n" "\nagym --conversation=a83dad8b-7b39-49a0-9316-29a1b87b49c8\n" _pos)
if(_pos EQUAL -1)
  string(APPEND _failures "\n  run_missing_ssd: resume line missing after the 429: '${_err}'")
endif()

# Both targets reachable and a clean run: two --add-dir pairs, exit 0, no notes.
file(MAKE_DIRECTORY "${SCRATCH_DIR}/real-scratch")
file(REAL_PATH "${SCRATCH_DIR}/real-scratch" _scratch_phys)
make_repo(repo-both "${SCRATCH_DIR}/real-scratch" "${SCRATCH_DIR}/real-deps")

run_agym(repo-both "${SCRATCH_DIR}/${_clean_name}" 0 _argv _err _rc)
string(STRIP "${_argv}" _argv)
string(REPLACE "\n" ";" _args "${_argv}")
list(LENGTH _args _count)
if(NOT _rc EQUAL 0 OR NOT "${_err}" STREQUAL "")
  string(APPEND _failures "\n  run_both: expected rc 0 and silence, got rc=${_rc}, stderr '${_err}'")
endif()
if(NOT _count EQUAL 7)
  string(APPEND _failures "\n  run_both: argv has ${_count} items, expected 7: '${_argv}'")
else()
  list(GET _args 3 _a3)
  list(GET _args 4 _a4)
  list(GET _args 5 _a5)
  list(GET _args 6 _a6)
  if(NOT _a3 STREQUAL "--add-dir" OR NOT _a4 STREQUAL "${_scratch_phys}")
    string(APPEND _failures "\n  run_both: expected --add-dir for .scratch at '${_scratch_phys}', got '${_a3}' '${_a4}'")
  endif()
  if(NOT _a5 STREQUAL "--add-dir" OR NOT _a6 STREQUAL "${_deps_phys}")
    string(APPEND _failures "\n  run_both: expected --add-dir for .deps at '${_deps_phys}', got '${_a5}' '${_a6}'")
  endif()
endif()

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_agym contract failed:${_failures}")
endif()
message(STATUS "tooling_agym contract passed")
