# tooling_reports — contract for tools/agy/check-reports.sh (shard report completeness).
# Covers: a complete set passes with and without --expect; a report with no end
# marker (truncated) fails; a marker whose shard id differs from the file name fails;
# blank lines after the marker pass; an --expect mismatch fails; usage errors exit 2.
# The script runs under /bin/bash when it exists (bash 3.2 on macOS), so the 3.2
# contract is tested. Nothing is executed beyond the script itself.
# Stays in the ordinary (non-slow) CTest set.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_reports requires -D${_required}")
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
set(_check "${REPO_ROOT}/tools/agy/check-reports.sh")

# run_check(<dir-name> <rc-out> <out-out> <err-out> <args...>) runs the script;
# each "@DIR" in <args...> becomes SCRATCH_DIR/<dir-name>.
function(run_check dir_name rc_out out_out err_out)
  set(_args "")
  foreach(_arg ${ARGN})
    if(_arg STREQUAL "@DIR")
      list(APPEND _args "${SCRATCH_DIR}/${dir_name}")
    else()
      list(APPEND _args "${_arg}")
    endif()
  endforeach()
  execute_process(COMMAND "${_bash}" "${_check}" ${_args}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  string(STRIP "${_out}" _out)
  string(STRIP "${_err}" _err)
  set(${rc_out} "${_rc}" PARENT_SCOPE)
  set(${out_out} "${_out}" PARENT_SCOPE)
  set(${err_out} "${_err}" PARENT_SCOPE)
endfunction()

# Complete set: three reports, each ending with its own marker.
file(WRITE "${SCRATCH_DIR}/complete/alpha.md" "# alpha\nfinding\n<!-- END REPORT shard=alpha items=2 -->\n")
file(WRITE "${SCRATCH_DIR}/complete/beta.md" "# beta\n<!-- END REPORT shard=beta items=0 -->\n")
file(WRITE "${SCRATCH_DIR}/complete/D14.md" "# D14\nText\n<!-- END REPORT shard=D14 items=13 -->\n")

# Complete set with the matching count passes; so does the default run without --expect.
run_check(complete _rc _out _err @DIR --expect 3)
if(NOT _rc EQUAL 0 OR NOT "${_out}${_err}" STREQUAL "")
  string(APPEND _failures "\n  complete_expect: expected rc 0 and silence, got rc=${_rc}, out '${_out}', err '${_err}'")
endif()
run_check(complete _rc _out _err @DIR)
if(NOT _rc EQUAL 0 OR NOT "${_out}${_err}" STREQUAL "")
  string(APPEND _failures "\n  complete_default: expected rc 0 and silence, got rc=${_rc}, out '${_out}', err '${_err}'")
endif()

# Truncated: a report cut mid-word, no marker, fails and names only that file.
file(WRITE "${SCRATCH_DIR}/truncated/ok.md" "# ok\n<!-- END REPORT shard=ok items=1 -->\n")
file(WRITE "${SCRATCH_DIR}/truncated/cut.md" "# cut\nThe agent stopped mid-wo")
run_check(truncated _rc _out _err @DIR)
if(NOT _rc EQUAL 1 OR NOT _out STREQUAL "cut.md: no end marker")
  string(APPEND _failures "\n  truncated: expected rc 1 and 'cut.md: no end marker', got rc=${_rc}, out '${_out}'")
endif()

# Shard mismatch: the marker names a shard other than the file's basename.
file(WRITE "${SCRATCH_DIR}/mismatch/x.md" "# x\n<!-- END REPORT shard=y items=4 -->\n")
run_check(mismatch _rc _out _err @DIR)
if(NOT _rc EQUAL 1 OR NOT _out STREQUAL "x.md: marker shard=y does not match file")
  string(APPEND _failures "\n  mismatch: expected rc 1 and the mismatch line, got rc=${_rc}, out '${_out}'")
endif()

# Trailing blank lines (empty and whitespace-only) after the marker still pass.
file(WRITE "${SCRATCH_DIR}/blanks/z.md" "# z\n<!-- END REPORT shard=z items=7 -->\n\n   \n\n")
run_check(blanks _rc _out _err @DIR)
if(NOT _rc EQUAL 0 OR NOT "${_out}${_err}" STREQUAL "")
  string(APPEND _failures "\n  blanks: expected rc 0 and silence, got rc=${_rc}, out '${_out}', err '${_err}'")
endif()

# --expect mismatch: four expected, three present.
run_check(complete _rc _out _err @DIR --expect 4)
if(NOT _rc EQUAL 1 OR NOT _out STREQUAL "expected 4 reports, found 3")
  string(APPEND _failures "\n  expect_mismatch: expected rc 1 and the count line, got rc=${_rc}, out '${_out}'")
endif()

# Usage errors: no directory, a non-numeric --expect, and a missing directory all exit 2.
run_check(complete _rc _out _err)
if(NOT _rc EQUAL 2)
  string(APPEND _failures "\n  usage_no_dir: expected rc 2, got rc=${_rc}")
endif()
run_check(complete _rc _out _err @DIR --expect three)
if(NOT _rc EQUAL 2)
  string(APPEND _failures "\n  usage_bad_expect: expected rc 2, got rc=${_rc}")
endif()
run_check(no-such-dir _rc _out _err @DIR)
if(NOT _rc EQUAL 2)
  string(APPEND _failures "\n  usage_missing_dir: expected rc 2, got rc=${_rc}")
endif()

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_reports contract failed:${_failures}")
endif()
message(STATUS "tooling_reports contract passed")
