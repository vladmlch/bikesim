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
  if(DEFINED CASE_EXPECT_STDOUT AND NOT CASE_EXPECT_STDOUT STREQUAL "")
    string(FIND "${_out}" "${CASE_EXPECT_STDOUT}" _pos)
    if(_pos EQUAL -1)
      set(_failures
        "${_failures}\n  ${name}: stdout missing '${CASE_EXPECT_STDOUT}': ${_combined}"
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
run_case(t56_validate_only_kind EXPECT_EXIT 2
  EXPECT_STDERR "require --kind context"
  COMMAND "${CHECKS_BINARY}" --kind headers --build "${BUILD_DIR}" --validate-only)
run_case(t56_list_controls_kind EXPECT_EXIT 2
  EXPECT_STDERR "require --kind context"
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}" --list-controls)
run_case(t56_context_modes_exclusive EXPECT_EXIT 2
  EXPECT_STDERR "mutually exclusive"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}" --validate-only --list-controls)
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
string(JSON _mutated SET "${_context_json}" compiler path "\"/nonexistent/c++\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
run_case(t56_ctx_stale_compiler EXPECT_EXIT FAIL
  EXPECT_STDERR "missing configured C++ compiler"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# A missing configured SDK is a Darwin-only contract. Linux instead
# rejects any Apple SDK, including an otherwise existing directory.
set(_fixture "${SCRATCH_DIR}/ctx-stale-sdk")
file(MAKE_DIRECTORY "${_fixture}")
string(JSON _mutated SET "${_context_json}" sdk "\"/nonexistent/MacOSX.sdk\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
  set(_sdk_error "missing configured SDK")
else()
  set(_sdk_error "Linux native check context must not record an Apple SDK")
endif()
run_case(t56_ctx_stale_sdk EXPECT_EXIT FAIL
  EXPECT_STDERR "${_sdk_error}"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# missing required file
set(_fixture "${SCRATCH_DIR}/ctx-missing-required")
file(MAKE_DIRECTORY "${_fixture}")
string(JSON _required_count LENGTH "${_context_json}" required_files)
string(JSON _mutated SET "${_context_json}" required_files ${_required_count}
  "\"/nonexistent/required-file\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
run_case(t56_ctx_missing_required EXPECT_EXIT FAIL
  EXPECT_STDERR "missing context required_files entry"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture}")

# Schema-2 fixtures keep real dependency/compiler paths, but carry only the
# context file: --validate-only must never require a TU manifest or compile.
cmake_host_system_information(RESULT _t56_machine QUERY OS_PLATFORM)
set(_t56_system "${CMAKE_HOST_SYSTEM_NAME}")
function(t56_schema2 out_var)
  set(_json "${_context_json}")
  string(JSON _json SET "${_json}" schema_version "2")
  string(JSON _json SET "${_json}" platform
    "{\"system\":\"${_t56_system}\",\"host_system\":\"${_t56_system}\",\"machine\":\"${_t56_machine}\",\"pointer_bits\":64}")
  if(_t56_system STREQUAL "Darwin")
    string(JSON _sdk GET "${_json}" sdk)
    string(JSON _json SET "${_json}" sysroot "\"${_sdk}\"")
    string(JSON _json SET "${_json}" stdlib
      "{\"name\":\"libc++\",\"version\":\"1\",\"hardening\":{\"mode\":\"EXTENSIVE\",\"effective\":true}}")
  else()
    string(JSON _json SET "${_json}" sysroot "null")
    string(JSON _json SET "${_json}" sdk "null")
    string(JSON _json SET "${_json}" sdk_requested "null")
    string(JSON _json SET "${_json}" sdk_source "\"not-applicable\"")
    string(JSON _json SET "${_json}" stdlib
      "{\"name\":\"libstdc++\",\"version\":\"13\",\"hardening\":{\"mode\":\"ASSERTIONS\",\"effective\":true}}")
  endif()
  string(JSON _json SET "${_json}" numerics "{\"backend\":\"TEST\"}")
  string(JSON _json SET "${_json}" policy
    "{\"required_flags\":[],\"accepted_warnings\":[],\"rejected_warnings\":[]}")
  set(_sanitizer "{\"requested\":false,\"supported\":false,\"enabled\":false,\"runtime\":null,\"compiler_id\":\"Clang\",\"target\":null,\"evidence\":{\"probe\":\"fixture\"}}")
  string(JSON _json SET "${_json}" capabilities
    "{\"asan\":${_sanitizer},\"rtsan\":${_sanitizer},\"coverage\":{\"requested\":false,\"supported\":false,\"enabled\":false,\"runtime\":null,\"evidence\":{\"probe\":\"fixture\"}},\"frontend_tools\":{}}")
  set(${out_var} "${_json}" PARENT_SCOPE)
endfunction()

function(t56_ctx_case name json)
  set(_fixture "${SCRATCH_DIR}/${name}")
  file(MAKE_DIRECTORY "${_fixture}")
  file(WRITE "${_fixture}/native_check_context.json" "${json}")
  set(_fixture_dir "${_fixture}" PARENT_SCOPE)
endfunction()

t56_schema2(_s2)
t56_ctx_case(ctx-schema2-host "${_s2}")
run_case(t56_ctx_schema2_host EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
if(_t56_system STREQUAL "Darwin")
  string(JSON _s1 SET "${_s2}" schema_version "1")
  t56_ctx_case(ctx-schema1-darwin "${_s1}")
  run_case(t56_ctx_schema1_darwin EXPECT_EXIT 0
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endif()

string(JSON _bad SET "${_s2}" schema_version "3")
t56_ctx_case(ctx-schema3 "${_bad}")
run_case(t56_ctx_schema3 EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has an unsupported schema"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" platform host_system "\"Plan9\"")
t56_ctx_case(ctx-schema2-host-mismatch "${_bad}")
run_case(t56_ctx_schema2_host_mismatch EXPECT_EXIT FAIL
  EXPECT_STDERR "host_system='Plan9' on a '${_t56_system}' host"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" platform machine "\"not-the-host-machine\"")
t56_ctx_case(ctx-schema2-machine "${_bad}")
run_case(t56_ctx_schema2_machine EXPECT_EXIT FAIL
  EXPECT_STDERR "native check platform machine 'not-the-host-machine' does not match host machine"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" platform pointer_bits "32")
t56_ctx_case(ctx-schema2-pointer-width "${_bad}")
run_case(t56_ctx_schema2_pointer_width EXPECT_EXIT FAIL
  EXPECT_STDERR "native check platform pointer width does not match the host"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad REMOVE "${_s2}" platform)
t56_ctx_case(ctx-schema2-no-platform "${_bad}")
run_case(t56_ctx_schema2_no_platform EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has no platform record"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

# Match the Python validator's actual failure ordering: a missing path and
# a non-path value have different diagnostics.
string(JSON _bad SET "${_s2}" sysroot "\"/nonexistent/sysroot\"")
t56_ctx_case(ctx-schema2-missing-sysroot "${_bad}")
run_case(t56_ctx_schema2_missing_sysroot EXPECT_EXIT FAIL
  EXPECT_STDERR "missing configured sysroot: /nonexistent/sysroot"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" sysroot "false")
t56_ctx_case(ctx-schema2-bad-sysroot "${_bad}")
run_case(t56_ctx_schema2_bad_sysroot EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context sysroot must be null or a directory path"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" stdlib name "\"msvcrt\"")
t56_ctx_case(ctx-schema2-bad-stdlib "${_bad}")
run_case(t56_ctx_schema2_bad_stdlib EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has an unsupported stdlib: 'msvcrt'"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" stdlib hardening effective "false")
t56_ctx_case(ctx-schema2-inactive-hardening "${_bad}")
run_case(t56_ctx_schema2_inactive_hardening EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context stdlib hardening must be effective"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad REMOVE "${_s2}" numerics backend)
t56_ctx_case(ctx-schema2-no-numerics "${_bad}")
run_case(t56_ctx_schema2_no_numerics EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context numerics has no backend"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" policy rejected_warnings "[{\"flag\":\"-Wtest\"}]")
t56_ctx_case(ctx-schema2-policy "${_bad}")
run_case(t56_ctx_schema2_policy EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context policy rejected_warnings must be a list of flag/reason records"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

foreach(_capability asan rtsan coverage)
  string(JSON _bad SET "${_s2}" capabilities ${_capability} enabled "true")
  t56_ctx_case(ctx-schema2-${_capability}-unsupported "${_bad}")
  run_case(t56_ctx_schema2_${_capability}_unsupported EXPECT_EXIT FAIL
    EXPECT_STDERR "native check context capability ${_capability} is enabled without support evidence"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad REMOVE "${_s2}" capabilities ${_capability})
  t56_ctx_case(ctx-schema2-no-${_capability} "${_bad}")
  run_case(t56_ctx_schema2_no_${_capability} EXPECT_EXIT FAIL
    EXPECT_STDERR "native check context has no ${_capability} capability record"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad REMOVE "${_s2}" capabilities ${_capability} runtime)
  t56_ctx_case(ctx-schema2-${_capability}-no-runtime "${_bad}")
  run_case(t56_ctx_schema2_${_capability}_no_runtime EXPECT_EXIT FAIL
    EXPECT_STDERR "native check context capability ${_capability} has no runtime record"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endforeach()
string(JSON _bad SET "${_s2}" capabilities asan evidence "{}")
t56_ctx_case(ctx-schema2-empty-evidence "${_bad}")
run_case(t56_ctx_schema2_empty_evidence EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context capability asan evidence must map names to strings"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" capabilities frontend_tools tidy
  "{\"configured\":\"clang-tidy\",\"located\":false}")
t56_ctx_case(ctx-schema2-bad-tool "${_bad}")
run_case(t56_ctx_schema2_bad_tool EXPECT_EXIT FAIL
  EXPECT_STDERR "native check frontend tool 'tidy' located record must be a path or null"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

if(_t56_system STREQUAL "Darwin")
  string(JSON _bad SET "${_s2}" platform system "\"Linux\"")
  t56_ctx_case(ctx-schema2-foreign-system "${_bad}")
  run_case(t56_ctx_schema2_foreign_system EXPECT_EXIT FAIL
    EXPECT_STDERR "native check platform records system='Linux'"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
else()
  string(JSON _bad SET "${_s2}" sdk_requested "\"/tmp\"")
  t56_ctx_case(ctx-schema2-linux-sdk-requested "${_bad}")
  run_case(t56_ctx_schema2_linux_sdk_requested EXPECT_EXIT FAIL
    EXPECT_STDERR "Linux native check context must not record a requested SDK"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad SET "${_s2}" sdk_source "\"default\"")
  t56_ctx_case(ctx-schema2-linux-sdk-source "${_bad}")
  run_case(t56_ctx_schema2_linux_sdk_source EXPECT_EXIT FAIL
    EXPECT_STDERR "Linux native check context must record sdk_source 'not-applicable'"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endif()

# Listing uses the runner's ordered table, but runs no configure/probe.
run_case(t56_context_controls_platform EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}" --list-controls)
execute_process(COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}" --list-controls
  OUTPUT_VARIABLE _controls RESULT_VARIABLE _rc)
if(_t56_system STREQUAL "Darwin")
  set(_want "explicit_sdk;missing_sdk;default_sdk;default_sdk_fallback")
else()
  set(_want "no_sdk;bad_sysroot;rejected_osx_sysroot;no_apple_tool;sysroot_preservation")
endif()
string(STRIP "${_controls}" _controls)
string(REPLACE "\n" ";" _control_list "${_controls}")
if(NOT _control_list STREQUAL _want)
  set(_failures "${_failures}\n  t56_controls: expected '${_want}', got '${_control_list}'")
endif()

t56_ctx_case(ctx-tool-override "${_context_json}")
string(JSON _ovr SET "${_context_json}" tool_overrides CLANG_TIDY "\"/nonexistent/clang-tidy\"")
file(WRITE "${_fixture_dir}/native_check_context.json" "${_ovr}")
file(COPY "${_sources}" "${_ccdb}" DESTINATION "${_fixture_dir}")
run_case(t56_tool_override_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "invalid tidy override: /nonexistent/clang-tidy"
  COMMAND "${CMAKE_COMMAND}" -E env --unset=CLANG_TIDY
        "${CHECKS_BINARY}" --kind tidy --build "${_fixture_dir}")

if(_t56_system STREQUAL "Linux")
  t56_ctx_case(ctx-linux-sysroot-arg "${_s2}")
  file(COPY "${_sources}" DESTINATION "${_fixture_dir}")
  file(READ "${_ccdb}" _db)
  string(JSON _entry_count LENGTH "${_db}")
  math(EXPR _last_entry "${_entry_count} - 1")
  foreach(_index RANGE 0 ${_last_entry})
    string(JSON _entry GET "${_db}" ${_index})
    string(JSON _args_type ERROR_VARIABLE _args_error TYPE "${_entry}" arguments)
    if(_args_error STREQUAL "NOTFOUND")
      string(JSON _args GET "${_entry}" arguments)
      string(JSON _argc LENGTH "${_args}")
      string(JSON _args SET "${_args}" ${_argc} "\"-isysroot\"")
      math(EXPR _argc "${_argc} + 1")
      string(JSON _args SET "${_args}" ${_argc} "\"/tmp\"")
      string(JSON _entry SET "${_entry}" arguments "${_args}")
    endif()
    string(JSON _command_type ERROR_VARIABLE _command_error TYPE "${_entry}" command)
    if(_command_error STREQUAL "NOTFOUND")
      string(JSON _command GET "${_entry}" command)
      set(_command "${_command} -isysroot /tmp")
      string(REPLACE "\\" "\\\\" _command "${_command}")
      string(REPLACE "\"" "\\\"" _command "${_command}")
      string(JSON _entry SET "${_entry}" command "\"${_command}\"")
    endif()
    string(JSON _db SET "${_db}" ${_index} "${_entry}")
  endforeach()
  file(WRITE "${_fixture_dir}/compile_commands.json" "${_db}")
  run_case(t56_ctx_linux_sysroot_arg EXPECT_EXIT FAIL
    EXPECT_STDERR "but the build context configures none"
    COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture_dir}")
endif()

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
string(JSON _source_count LENGTH "${_manifest_json}" sources)
string(JSON _manifest_mutated SET "${_manifest_json}" sources ${_source_count}
  "{\"path\":\"/nonexistent/source.cpp\",\"target\":\"stale\",\"context\":\"stale:Release\"}")
file(WRITE "${_fixture}/native_sources.json" "${_manifest_mutated}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_manifest_stale EXPECT_EXIT FAIL
  EXPECT_STDERR "manifest source is outside first-party roots"
  COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture}")

# tool override naming a missing executable resolves to a fail-closed error
set(_fixture "${SCRATCH_DIR}/tool-missing")
file(MAKE_DIRECTORY "${_fixture}")
string(JSON _mutated SET "${_context_json}" tool_overrides GXX "\"/nonexistent/g++-16\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_tool_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "invalid frontends override: /nonexistent/g++-16"
  COMMAND "${CMAKE_COMMAND}" -E env --unset=GXX
        "${CHECKS_BINARY}" --kind frontends --build "${_fixture}")

# ---------------------------------------------------------------------------
# Mandatory negative controls: crashed analyzer, malformed diagnostics,
# failed diagnostic control, stale/wrong contract artifacts and a direct
# no-interpreter reach check.

# crashed analyzer — a planted tool that dies on every invocation surfaces
# as tool-health failure, never a silent pass
set(_crash_tool "${SCRATCH_DIR}/crash-analyzer.sh")
file(WRITE "${_crash_tool}" "#!/bin/sh\nexit 139\n")
file(CHMOD "${_crash_tool}"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
run_case(t56_analyzer_crash EXPECT_EXIT FAIL
  EXPECT_STDERR "tool health: failed"
  COMMAND "${CMAKE_COMMAND}" -E env "ANALYZER_CLANG=${_crash_tool}"
        "${CHECKS_BINARY}" --kind analyzer --build "${BUILD_DIR}")

# malformed diagnostics — the analysis suppression manifest is the
# diagnostics-control input; a malformed file must fail closed
set(_fixture "${SCRATCH_DIR}/diag-malformed")
file(MAKE_DIRECTORY "${_fixture}")
file(WRITE "${_fixture}/suppressions.json" "{")
string(JSON _mutated SET "${_context_json}" analysis_suppressions_manifest
  "\"${_fixture}/suppressions.json\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_diagnostics_malformed EXPECT_EXIT FAIL
  EXPECT_STDERR "malformed analysis suppression manifest"
  COMMAND "${CHECKS_BINARY}" --kind analyzer --build "${_fixture}")

# failed diagnostic control — a silent-exit analyzer passes version health
# but emits no cplusplus.Move marker, so the alpha control must fail
set(_silent_tool "${SCRATCH_DIR}/silent-analyzer.sh")
file(WRITE "${_silent_tool}" "#!/bin/sh\nexit 0\n")
file(CHMOD "${_silent_tool}"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
run_case(t56_diag_control_failed EXPECT_EXIT FAIL
  EXPECT_STDERR "first_party_use_after_move"
  COMMAND "${CMAKE_COMMAND}" -E env "ANALYZER_CLANG=${_silent_tool}"
        "${CHECKS_BINARY}" --kind diagnostic-controls --build "${BUILD_DIR}")

# contracts on the produced build — positive contract control
run_case(t56_contracts_clean EXPECT_EXIT 0
  EXPECT_STDERR "native contract checks passed"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${BUILD_DIR}"
        --repo-root "${REPO_ROOT}")
run_case(t56_contracts_default_root EXPECT_EXIT 0
  EXPECT_STDERR "native contract checks passed"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${BUILD_DIR}")

# stale extension — a bindings-era context (python_executable non-null) on a
# build with no produced extension must fail closed rather than pass
set(_fixture "${SCRATCH_DIR}/contracts-stale-extension")
file(MAKE_DIRECTORY "${_fixture}")
string(JSON _mutated SET "${_context_json}" python_executable "\"${CMAKE_COMMAND}\"")
file(WRITE "${_fixture}/native_check_context.json" "${_mutated}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_contracts_stale_extension EXPECT_EXIT FAIL
  EXPECT_STDERR "expected exactly one bike_native extension"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${_fixture}"
        --repo-root "${REPO_ROOT}")

# wrong artifact — a planted bogus extension must not satisfy the produced
# artifact contract: loader provenance verification fails closed
set(_fixture "${SCRATCH_DIR}/contracts-wrong-artifact")
file(MAKE_DIRECTORY "${_fixture}")
file(WRITE "${_fixture}/bike_native.planted.so" "not a real extension\n")
file(WRITE "${_fixture}/native_check_context.json" "${_context_json}")
file(COPY "${_sources}" DESTINATION "${_fixture}")
file(COPY "${_ccdb}" DESTINATION "${_fixture}")
run_case(t56_contracts_wrong_artifact EXPECT_EXIT FAIL
  EXPECT_STDERR "native contract checks failed"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${_fixture}"
        --repo-root "${REPO_ROOT}")

# Provider assembler bindings are legal only in the platform backends.
# This source root is deliberately independent of the compiled manifest;
# the structural check must discover a newly added, uncompiled source too.
set(_provider_root "${SCRATCH_DIR}/provider-root")
file(MAKE_DIRECTORY "${_provider_root}/native/src/geometry")
file(WRITE "${_provider_root}/native/src/geometry/direct.cpp"
  "extern \"C\" double leaked_dot() __asm__(\"_cblas_ddot$NEWLAPACK$ILP64\");\n")
run_case(t56_provider_boundary_accelerate EXPECT_EXIT FAIL
  EXPECT_STDERR "provider symbol bound outside native/src/platform"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${BUILD_DIR}"
        --repo-root "${_provider_root}")
file(WRITE "${_provider_root}/native/src/geometry/direct.cpp"
  "extern \"C\" double leaked_dot() __asm__(\"scipy_cblas_ddot64_\");\n")
run_case(t56_provider_boundary_openblas EXPECT_EXIT FAIL
  EXPECT_STDERR "provider symbol bound outside native/src/platform"
  COMMAND "${CHECKS_BINARY}" --kind contracts --build "${BUILD_DIR}"
        --repo-root "${_provider_root}")

# no-interpreter reach — the contracts sweep spawns no children on the
# Python-free graph, so it must complete with no usable PATH and a wheel
# planted on PYTHONPATH: no interpreter dependency, no silent wheel reach
set(_wheel_dir "${SCRATCH_DIR}/wheel-bait")
file(MAKE_DIRECTORY "${_wheel_dir}" "${SCRATCH_DIR}/empty-bin")
file(WRITE "${_wheel_dir}/planted-1.0-py3-none-any.whl" "planted\n")
run_case(t56_no_interpreter_reach EXPECT_EXIT 0
  EXPECT_STDERR "native contract checks passed"
  COMMAND "${CMAKE_COMMAND}" -E env
        "PATH=${SCRATCH_DIR}/empty-bin"
        "PYTHONPATH=${_wheel_dir}"
        "PYTHONHOME=${SCRATCH_DIR}"
        "${CHECKS_BINARY}" --kind contracts --build "${BUILD_DIR}"
        --repo-root "${REPO_ROOT}")

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

# Depth 3 stays outside the sweep, while depth 2 is required for tools/linux.
file(MAKE_DIRECTORY "${_tools_clean}/linux/deeper")
file(WRITE "${_tools_clean}/linux/deeper/bad.sh"
  "#!/bin/bash\nfor f in $UNQUOTED; do echo $f; done\n")
run_case(t56_scripts_depth3_excluded EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_clean}")
file(WRITE "${_tools_clean}/linux/bad.sh"
  "#!/bin/bash\nfor f in $UNQUOTED; do echo $f; done\n")
run_case(t56_scripts_depth2_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "linux/bad.sh"
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_clean}")

set(_tools_empty "${SCRATCH_DIR}/tools-empty")
file(MAKE_DIRECTORY "${_tools_empty}")
run_case(t56_scripts_empty EXPECT_EXIT FAIL
  EXPECT_STDERR "native scripts selection: 0 scripts"
  COMMAND "${CHECKS_BINARY}" --kind scripts --build "${BUILD_DIR}"
        --tools-dir "${_tools_empty}")

# Produced sweep on the real tools tree (positive produced-orchestration
# control; exercises tools/*.sh and tools/*/*.sh through shellcheck).
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
