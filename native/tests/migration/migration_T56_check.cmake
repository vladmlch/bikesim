# migration_T56 — fixture contract for the produced check orchestrator.
#
# Exercises bike-native-checks / bike-python-audit without touching the real
# sweep semantics: argparse parity, fail-closed context/manifest/database
# validation, the scripts sweep against planted fixtures, and the audit
# modes against planted interpreter edges. Full sweeps run through the
# check_* targets / tools/run_native_checks.sh, not inside this ctest.
#
# Required -D inputs: CHECKS_BINARY, AUDIT_BINARY, BUILD_DIR, REPO_ROOT,
# SCRATCH_DIR.

foreach(_required CHECKS_BINARY AUDIT_BINARY BUILD_DIR REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "migration_T56 requires -D${_required}")
  endif()
endforeach()

file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")

set(_failures "")

# run_case(<name> EXPECT_EXIT <rc> EXPECT_STDERR <needle> COMMAND <argv...>)
# Executes one control and records a failure line when the exit status or
# the required stderr needle differs. EXPECT_EXIT 0 means success required;
# FAIL means any nonzero; numeric means exact.
function(run_case name)
  cmake_parse_arguments(CASE "" "EXPECT_EXIT;EXPECT_STDERR;EXPECT_STDOUT"
    "COMMAND" ${ARGN})
  execute_process(
    COMMAND ${CASE_COMMAND}
    RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
  set(_combined "${_out}\n${_err}")
  if(CASE_EXPECT_EXIT STREQUAL "FAIL")
    if(_rc EQUAL 0)
      set(_failures "${_failures}\n  ${name}: expected failure, exited 0" PARENT_SCOPE)
    endif()
  elseif(NOT _rc EQUAL ${CASE_EXPECT_EXIT})
    set(_failures
      "${_failures}\n  ${name}: expected exit ${CASE_EXPECT_EXIT}, got ${_rc}: ${_combined}"
      PARENT_SCOPE)
  endif()
  if(DEFINED CASE_EXPECT_STDERR AND NOT CASE_EXPECT_STDERR STREQUAL "")
    string(FIND "${_combined}" "${CASE_EXPECT_STDERR}" _pos)
    if(_pos EQUAL -1)
      set(_failures
        "${_failures}\n  ${name}: stderr missing '${CASE_EXPECT_STDERR}': ${_combined}"
        PARENT_SCOPE)
    endif()
  endif()
endfunction()

# ---------------------------------------------------------------------------
# CLI / argparse parity
run_case(t56_no_args EXPECT_EXIT 2
  EXPECT_STDERR "the following arguments are required"
  COMMAND "${CHECKS_BINARY}")
run_case(t56_bad_kind EXPECT_EXIT 2
  EXPECT_STDERR "invalid choice"
  COMMAND "${CHECKS_BINARY}" --kind bogus --build "${BUILD_DIR}")
run_case(t56_missing_build EXPECT_EXIT FAIL
  EXPECT_STDERR "build directory does not exist"
  COMMAND "${CHECKS_BINARY}" --kind context
        --build "${SCRATCH_DIR}/missing-build")
run_case(t56_help EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --help)
run_case(t56_check_child EXPECT_EXIT 0
  EXPECT_STDOUT "wrapped-child-ok"
  COMMAND "${CHECKS_BINARY}" --check-child /bin/echo wrapped-child-ok)

# ---------------------------------------------------------------------------
# Context validation fixtures — mutate a copy of the real emitted context.
set(_ctx "${BUILD_DIR}/native_check_context.json")
set(_sources "${BUILD_DIR}/native_sources.json")
set(_ccdb "${BUILD_DIR}/compile_commands.json")
foreach(_required_file "${_ctx}" "${_sources}" "${_ccdb}")
  if(NOT EXISTS "${_required_file}")
    message(FATAL_ERROR
      "migration_T56 requires the produced build to emit ${_required_file}")
  endif()
endforeach()
file(READ "${_ctx}" _context_json)

# malformed context
set(_fixture "${SCRATCH_DIR}/ctx-malformed")
file(MAKE_DIRECTORY "${_fixture}")
file(WRITE "${_fixture}/native_check_context.json" "{")
run_case(t56_ctx_malformed EXPECT_EXIT FAIL
  EXPECT_STDERR "malformed native check context"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# stale compiler path
set(_fixture "${SCRATCH_DIR}/ctx-stale-compiler")
file(MAKE_DIRECTORY "${_fixture}")
string(REGEX REPLACE "(\"compiler\":\\{\"path\":\")[^\"]*"
  "\\1/nonexistent/c++" _mutated "${_context_json}")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
run_case(t56_ctx_stale_compiler EXPECT_EXIT FAIL
  EXPECT_STDERR "missing configured C++ compiler"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# missing configured SDK
set(_fixture "${SCRATCH_DIR}/ctx-stale-sdk")
file(MAKE_DIRECTORY "${_fixture}")
string(REGEX REPLACE "(\"sdk\":\")[^\"]*"
  "\\1/nonexistent/MacOSX.sdk" _mutated "${_context_json}")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
run_case(t56_ctx_stale_sdk EXPECT_EXIT FAIL
  EXPECT_STDERR "missing configured SDK"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# missing required file
set(_fixture "${SCRATCH_DIR}/ctx-missing-required")
file(MAKE_DIRECTORY "${_fixture}")
string(REGEX REPLACE "(\"required_files\":\\[)"
  "\\1\"/nonexistent/required-file\"," _mutated "${_context_json}")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
run_case(t56_ctx_missing_required EXPECT_EXIT FAIL
  EXPECT_STDERR "missing context required_files entry"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# empty compilation database
set(_fixture "${SCRATCH_DIR}/db-empty")
file(MAKE_DIRECTORY "${_fixture}")
file(WRITE "${_fixture}/native_check_context.json" "${_context_json}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(WRITE "${_fixture}/compile_commands.json" "[]")
run_case(t56_db_empty EXPECT_EXIT FAIL
  EXPECT_STDERR "compilation database has an empty translation-unit selection"
  COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture}")

# manifest references a missing source (fail closed before any tool runs)
set(_fixture "${SCRATCH_DIR}/manifest-stale")
file(MAKE_DIRECTORY "${_fixture}")
file(WRITE "${_fixture}/native_check_context.json" "${_context_json}")
file(READ "${_sources}" _manifest_json)
string(REGEX REPLACE "(\"sources\":\\[)"
  "\\1{\"path\":\"/nonexistent/source.cpp\",\"target\":\"stale\",\"context\":\"stale:Release\"},"
  _manifest_mutated "${_manifest_json}")
file(WRITE "${_fixture}/native_sources.json" "${_manifest_mutated}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_manifest_stale EXPECT_EXIT FAIL
  EXPECT_STDERR "manifest source is outside first-party roots"
  COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture}")

# tool override naming a missing executable resolves to a fail-closed error
set(_fixture "${SCRATCH_DIR}/tool-missing")
file(MAKE_DIRECTORY "${_fixture}")
string(REGEX REPLACE "(\"tool_overrides\":\\{)[^}]*\\}"
  "\\1\"GXX\":\"/nonexistent/g++-16\"}" _mutated "${_context_json}")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_tool_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "required tool is missing"
  COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture}")

# ---------------------------------------------------------------------------
# scripts sweep fixtures (real build context, planted tools directories)
set(_tools_bad "${SCRATCH_DIR}/tools-bad")
file(MAKE_DIRECTORY "${_tools_bad}")
file(WRITE "${_tools_bad}/bad.sh"
  "#!/bin/bash\nfor f in $UNQUOTED; do echo $f; done\n")
run_case(t56_scripts_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "native scripts"
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_bad}")

set(_tools_clean "${SCRATCH_DIR}/tools-clean")
file(MAKE_DIRECTORY "${_tools_clean}")
file(WRITE "${_tools_clean}/clean.sh"
  "#!/usr/bin/env bash\nset -euo pipefail\nprintf '%s\\n' \"fixture-ok\"\n")
run_case(t56_scripts_clean EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_clean}")
if(NOT EXISTS "${BUILD_DIR}/native_check_summaries/scripts.json")
  set(_failures "${_failures}\n  t56_scripts_clean: scripts.json summary not published")
endif()

set(_tools_empty "${SCRATCH_DIR}/tools-empty")
file(MAKE_DIRECTORY "${_tools_empty}")
run_case(t56_scripts_empty EXPECT_EXIT FAIL
  EXPECT_STDERR "native scripts selection: 0 scripts"
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_empty}")

# Produced sweep on the real tools tree (positive produced-orchestration
# control; exercises every tools/*.sh through shellcheck).
run_case(t56_scripts_real EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}")

# ---------------------------------------------------------------------------
# bike-python-audit fixtures
set(_pkg_bad "${SCRATCH_DIR}/pkg-bad")
file(MAKE_DIRECTORY "${_pkg_bad}/lib" "${_pkg_bad}/dist" "${_pkg_bad}/bin")
file(WRITE "${_pkg_bad}/app.py" "print('planted')\n")
file(WRITE "${_pkg_bad}/lib/module.pyc" "planted\n")
file(WRITE "${_pkg_bad}/dist/pkg-1.0.whl" "planted\n")
file(WRITE "${_pkg_bad}/bin/tool" "#!/usr/bin/env python3\nprint('x')\n")
run_case(t56_audit_pkg_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "python package audit"
  COMMAND "${AUDIT_BINARY}" --root "${_pkg_bad}" --mode package
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

set(_pkg_clean "${SCRATCH_DIR}/pkg-clean")
file(MAKE_DIRECTORY "${_pkg_clean}")
file(WRITE "${_pkg_clean}/README.txt" "no interpreter artifacts\n")
run_case(t56_audit_pkg_clean EXPECT_EXIT 0
  COMMAND "${AUDIT_BINARY}" --root "${_pkg_clean}" --mode package
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

set(_doc_bad "${SCRATCH_DIR}/doc-bad")
file(MAKE_DIRECTORY "${_doc_bad}")
file(WRITE "${_doc_bad}/README.md"
  "# fixture\n\n\`\`\`bash\npython tool.py --flag\n\`\`\`\n")
run_case(t56_audit_doc_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "python documentation audit"
  COMMAND "${AUDIT_BINARY}" --root "${_doc_bad}" --mode documentation
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

set(_doc_clean "${SCRATCH_DIR}/doc-clean")
file(MAKE_DIRECTORY "${_doc_clean}")
file(WRITE "${_doc_clean}/README.md"
  "# fixture\n\n\`\`\`bash\n./bike-native-checks --kind context\n\`\`\`\n")
run_case(t56_audit_doc_clean EXPECT_EXIT 0
  COMMAND "${AUDIT_BINARY}" --root "${_doc_clean}" --mode documentation
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

set(_src_bad "${SCRATCH_DIR}/src-bad")
file(MAKE_DIRECTORY "${_src_bad}/tools")
file(WRITE "${_src_bad}/tools/thing.sh"
  "#!/usr/bin/env bash\nuv run python thing.py\n")
run_case(t56_audit_src_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "interpreter invocation"
  COMMAND "${AUDIT_BINARY}" --root "${_src_bad}" --mode source
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "migration_T56 fixture contract failed:${_failures}")
endif()
message(STATUS "migration_T56 fixture contract passed")
