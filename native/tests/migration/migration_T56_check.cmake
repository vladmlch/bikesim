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

# expect_json(<name> <file> <expected> <member-path...>)
# Reads a JSON document and requires the member/value at the path to equal
# <expected> (string compare of the decoded value, as string(JSON GET)
# renders it).
function(expect_json name file expected)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _val ERROR_VARIABLE _err GET "${_doc}" ${ARGN})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  if(NOT "${_val}" STREQUAL "${expected}")
    set(_failures
      "${_failures}\n  ${name}: json '${ARGN}' is '${_val}', expected '${expected}'"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_item_names(<name> <file> <path-to-array> <expected;names>)
# Collects the "name" string member of every object in a JSON array and
# requires the sorted name set to equal <expected;names>.
function(expect_json_item_names name file)
  cmake_parse_arguments(J "" "" "PATH;EXPECT" ${ARGN})
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _count ERROR_VARIABLE _err LENGTH "${_doc}" ${J_PATH})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${J_PATH}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  set(_names "")
  if(_count GREATER 0)
    math(EXPR _last "${_count} - 1")
    foreach(_i RANGE 0 ${_last})
      string(JSON _member ERROR_VARIABLE _err2 GET "${_doc}" ${J_PATH} ${_i} name)
      if(_err2)
        set(_failures
          "${_failures}\n  ${name}: ${J_PATH}[${_i}] has no 'name': ${_err2}"
          PARENT_SCOPE)
        return()
      endif()
      list(APPEND _names "${_member}")
    endforeach()
  endif()
  list(SORT _names)
  list(JOIN _names ";" _actual)
  list(JOIN J_EXPECT ";" _wanted)
  if(NOT _actual STREQUAL _wanted)
    set(_failures
      "${_failures}\n  ${name}: names '${_actual}', expected '${_wanted}'"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_contains(<name> <file> <path-to-array-of-strings> <needle>)
# Requires at least one string element of the array to contain <needle>.
function(expect_json_contains name file needle)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _count ERROR_VARIABLE _err LENGTH "${_doc}" ${ARGN})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  set(_found FALSE)
  if(_count GREATER 0)
    math(EXPR _last "${_count} - 1")
    foreach(_i RANGE 0 ${_last})
      string(JSON _item GET "${_doc}" ${ARGN} ${_i})
      string(FIND "${_item}" "${needle}" _pos)
      if(NOT _pos EQUAL -1)
        set(_found TRUE)
      endif()
    endforeach()
  endif()
  if(NOT _found)
    set(_failures
      "${_failures}\n  ${name}: no element of '${ARGN}' contains '${needle}'"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_length(<name> <file> <expected-count> <member-path...>)
function(expect_json_length name file expected)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _len ERROR_VARIABLE _err LENGTH "${_doc}" ${ARGN})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  if(NOT _len EQUAL ${expected})
    set(_failures
      "${_failures}\n  ${name}: json '${ARGN}' length ${_len}, expected ${expected}"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_member_set(<name> <file> MEMBER <m> PATH <path...> EXPECT <v...>)
# Collects one string member of every object in a JSON array and requires
# the sorted value set to equal EXPECT.
function(expect_json_member_set name file)
  cmake_parse_arguments(J "" "MEMBER" "PATH;EXPECT" ${ARGN})
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _count ERROR_VARIABLE _err LENGTH "${_doc}" ${J_PATH})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${J_PATH}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  set(_values "")
  if(_count GREATER 0)
    math(EXPR _last "${_count} - 1")
    foreach(_i RANGE 0 ${_last})
      string(JSON _member ERROR_VARIABLE _err2
        GET "${_doc}" ${J_PATH} ${_i} ${J_MEMBER})
      if(_err2)
        set(_failures
          "${_failures}\n  ${name}: ${J_PATH}[${_i}].${J_MEMBER}: ${_err2}"
          PARENT_SCOPE)
        return()
      endif()
      list(APPEND _values "${_member}")
    endforeach()
  endif()
  list(SORT _values)
  list(JOIN _values ";" _actual)
  list(JOIN J_EXPECT ";" _wanted)
  if(NOT _actual STREQUAL _wanted)
    set(_failures
      "${_failures}\n  ${name}: values '${_actual}', expected '${_wanted}'"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_equal(<name> <file> <member-path-a...> = <member-path-b...>)
# Requires two decoded values inside one document to compare equal.
function(expect_json_equal name file)
  set(_sep -1)
  foreach(_i RANGE ${ARGC})
    if(_i LESS ${ARGC} AND "${ARGV${_i}}" STREQUAL "=")
      set(_sep ${_i})
    endif()
  endforeach()
  # '=' must sit after at least one path-a member (index >= 3) and leave at
  # least one path-b member, i.e. at ARGC-1 or earlier.
  math(EXPR _last_arg "${ARGC} - 1")
  if(_sep LESS 3 OR _sep EQUAL ${_last_arg})
    message(FATAL_ERROR "expect_json_equal needs '<path-a> = <path-b>'")
  endif()
  set(_a "")
  set(_b "")
  # ARGV0/ARGV1 are the name/file parameters — member paths start at index 2.
  set(_i 2)
  while(_i LESS _sep)
    list(APPEND _a "${ARGV${_i}}")
    math(EXPR _i "${_i} + 1")
  endwhile()
  math(EXPR _i "${_sep} + 1")
  while(_i LESS ${ARGC})
    list(APPEND _b "${ARGV${_i}}")
    math(EXPR _i "${_i} + 1")
  endwhile()
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _va ERROR_VARIABLE _erra GET "${_doc}" ${_a})
  string(JSON _vb ERROR_VARIABLE _errb GET "${_doc}" ${_b})
  if(_erra OR _errb)
    set(_failures
      "${_failures}\n  ${name}: json path failed ('${_erra}'/'${_errb}')"
      PARENT_SCOPE)
    return()
  endif()
  if(NOT "${_va}" STREQUAL "${_vb}")
    set(_failures
      "${_failures}\n  ${name}: '${_a}'=${_va} != '${_b}'=${_vb}"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_nonzero(<name> <file> <member-path...>)
# Requires the decoded numeric member to be nonzero.
function(expect_json_nonzero name file)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _val ERROR_VARIABLE _err GET "${_doc}" ${ARGN})
  if(_err)
    set(_failures
      "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_err}"
      PARENT_SCOPE)
    return()
  endif()
  if(NOT _val MATCHES "^-?[0-9]+(\\.[0-9]+)?$" OR _val EQUAL 0)
    set(_failures
      "${_failures}\n  ${name}: json '${ARGN}' is '${_val}', expected nonzero"
      PARENT_SCOPE)
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
string(JSON _root_schema GET "${_context_json}" schema_version)
if(NOT _root_schema STREQUAL "2")
  set(_failures
    "${_failures}\n  t56_root_context_schema: root graph emits schema ${_root_schema}, expected 2")
endif()
foreach(_python_field python python_executable)
  string(JSON _python_type ERROR_VARIABLE _python_error TYPE
    "${_context_json}" ${_python_field})
  if(_python_error OR NOT _python_type STREQUAL "NULL")
    set(_failures
      "${_failures}\n  t56_root_python: ${_python_field} must be explicit null")
  endif()
endforeach()
string(JSON _native_root GET "${_context_json}" source_roots 0)
if(NOT _native_root STREQUAL "${REPO_ROOT}/native")
  set(_failures
    "${_failures}\n  t56_root_native_dir: first source root is '${_native_root}'")
endif()
string(JSON _provider_count LENGTH "${_context_json}" numerics library_paths)
if(_provider_count LESS 1)
  set(_failures "${_failures}\n  t56_root_providers: no numerical provider recorded")
else()
  math(EXPR _provider_last "${_provider_count} - 1")
  foreach(_index RANGE ${_provider_last})
    string(JSON _provider GET "${_context_json}" numerics library_paths ${_index} path)
    if(NOT IS_ABSOLUTE "${_provider}")
      set(_failures
        "${_failures}\n  t56_root_provider_path: '${_provider}' is not resolved")
    endif()
  endforeach()
endif()

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
  # The real root graph now emits schema 2; mutate truthful copies rather
  # than replacing its platform, policy or provider record with fixtures.
  set(${out_var} "${_context_json}" PARENT_SCOPE)
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

# Root metadata is an all-or-nothing path-validated contract, not a marker
# that lets a malformed binding context discard its interpreter.
foreach(_field dependency_root dependency_lock configure_source_dir cmake)
  string(JSON _bad SET "${_s2}" ${_field} "null")
  t56_ctx_case(ctx-root-null-${_field} "${_bad}")
  run_case(t56_ctx_root_null_${_field} EXPECT_EXIT FAIL
    EXPECT_STDERR "root context ${_field} is missing or is not a path"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad SET "${_s2}" ${_field} "\"/nonexistent/root-context/${_field}\"")
  t56_ctx_case(ctx-root-missing-${_field} "${_bad}")
  run_case(t56_ctx_root_missing_${_field} EXPECT_EXIT FAIL
    EXPECT_STDERR "missing root context ${_field}"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endforeach()
string(JSON _bad REMOVE "${_s2}" dependency_root)
t56_ctx_case(ctx-root-without-dependency-root "${_bad}")
run_case(t56_ctx_root_without_dependency_root EXPECT_EXIT FAIL
  EXPECT_STDERR "native check root metadata requires dependency_root"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" dependency_root "\"${REPO_ROOT}/CMakeLists.txt\"")
t56_ctx_case(ctx-root-directory-is-file "${_bad}")
run_case(t56_ctx_root_directory_is_file EXPECT_EXIT FAIL
  EXPECT_STDERR "root context dependency_root is not a directory"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_s2}" dependency_lock "\"${REPO_ROOT}\"")
t56_ctx_case(ctx-root-lock-is-directory "${_bad}")
run_case(t56_ctx_root_lock_is_directory EXPECT_EXIT FAIL
  EXPECT_STDERR "root context dependency_lock is not a file"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
foreach(_field python python_executable)
  string(JSON _bad REMOVE "${_s2}" ${_field})
  t56_ctx_case(ctx-root-missing-${_field} "${_bad}")
  run_case(t56_ctx_root_missing_${_field} EXPECT_EXIT FAIL
    EXPECT_STDERR "Python-free root context must record ${_field}: null"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad SET "${_s2}" ${_field} "\"${CMAKE_COMMAND}\"")
  t56_ctx_case(ctx-root-nonnull-${_field} "${_bad}")
  run_case(t56_ctx_root_nonnull_${_field} EXPECT_EXIT FAIL
    EXPECT_STDERR "Python-free root context must record ${_field}: null"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endforeach()

# CMake is only an existing executable-path fixture here. --validate-only
# does not launch it or claim that it is an actual Python interpreter.
set(_binding_context "${_s2}")
foreach(_field dependency_root dependency_lock configure_source_dir cmake)
  string(JSON _binding_context REMOVE "${_binding_context}" ${_field})
endforeach()
string(JSON _binding_context SET "${_binding_context}" python_executable
  "\"${CMAKE_COMMAND}\"")
string(JSON _binding_context SET "${_binding_context}" python
  "{\"executable\":\"${CMAKE_COMMAND}\"}")
t56_ctx_case(ctx-binding-metadata "${_binding_context}")
run_case(t56_ctx_binding_metadata EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_binding_context}" python_executable "null")
t56_ctx_case(ctx-binding-null-interpreter "${_bad}")
run_case(t56_ctx_binding_null_interpreter EXPECT_EXIT FAIL
  EXPECT_STDERR "configured Python executable is missing or is not a path"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad REMOVE "${_binding_context}" python_executable)
t56_ctx_case(ctx-binding-missing-interpreter "${_bad}")
run_case(t56_ctx_binding_missing_interpreter EXPECT_EXIT FAIL
  EXPECT_STDERR "configured Python executable is missing or is not a path"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
string(JSON _bad SET "${_binding_context}" python "null")
t56_ctx_case(ctx-binding-null-identity "${_bad}")
run_case(t56_ctx_binding_null_identity EXPECT_EXIT FAIL
  EXPECT_STDERR "binding native check context python must be an object"
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
  string(JSON _bad SET "${_s2}" capabilities ${_capability} supported "false")
  string(JSON _bad SET "${_bad}" capabilities ${_capability} enabled "true")
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
file(WRITE "${_fixture}/native_check_context.json" "${_binding_context}")
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
# shellcheck plain output names the path only in non-diagnostic "In <path>
# line N:" context lines and reports severity as "SC2168 (error):", neither
# of which the checker's `warning:|error:` diagnostic pattern collects —
# with shellcheck 0.11 the fail-closed stderr therefore carries only the
# returncode line. Assert that stable line rather than a format-dependent
# path or check id (the earlier 'linux/bad.sh' needle was a documented
# shellcheck-0.11.0 drift failure).
file(WRITE "${_tools_clean}/linux/bad.sh"
  "#!/bin/bash\nlocal unquoted=$UNQUOTED\n")
run_case(t56_scripts_depth2_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "shellcheck: rc=1"
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
# Backticks are literal in CMake strings; escaping them (\`) is tolerated
# differently by CMake 3.x (kept) and 4.x (stripped), which changed the
# fence content per platform — write them unescaped for identical bytes.
file(WRITE "${_doc_bad}/README.md"
  "# fixture\n\n```bash\npython tool.py --flag\n```\n")
run_case(t56_audit_doc_bad EXPECT_EXIT FAIL
  EXPECT_STDERR "python documentation audit"
  COMMAND "${AUDIT_BINARY}" --root "${_doc_bad}" --mode documentation
        --policy "${REPO_ROOT}/migration/no-python-policy.json")

set(_doc_clean "${SCRATCH_DIR}/doc-clean")
file(MAKE_DIRECTORY "${_doc_clean}")
file(WRITE "${_doc_clean}/README.md"
  "# fixture\n\n```bash\n./bike-native-checks --kind context\n```\n")
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

# ===========================================================================
# Batch 3.29b — check_tools / contract_checks ports (T53)
#
# Fixture contract cases for the produced checker surfaces, one per ported
# reference row: manifest/compile-database selection, sweep bookkeeping,
# suppression filtering, produced-configure probes, and the structural
# contract checks on a scratch repository.
# ===========================================================================

# Facts mirrored from the produced context so fixture entries validate the
# same way the emitted compile database does.
string(JSON _b29_compiler GET "${_context_json}" compiler path)
set(_b29_argv "${_b29_compiler}" "-std=c++23")
string(JSON _b29_sysroot_type ERROR_VARIABLE _b29_sysroot_err
  TYPE "${_context_json}" sysroot)
set(_b29_sysroot "")
if(NOT _b29_sysroot_err AND _b29_sysroot_type STREQUAL "STRING")
  string(JSON _b29_sysroot GET "${_context_json}" sysroot)
  list(APPEND _b29_argv "-isysroot" "${_b29_sysroot}")
endif()
set(_b29_def "")
set(_b29_def_arg "")
string(JSON _b29_hardening ERROR_VARIABLE _b29_herr
  GET "${_context_json}" libcpp_hardening supported)
if(NOT _b29_herr AND _b29_hardening)
  string(JSON _b29_hmode GET "${_context_json}" libcpp_hardening mode)
  set(_b29_def "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${_b29_hmode}")
  list(APPEND _b29_argv "-D${_b29_def}")
  set(_b29_def_arg "-D${_b29_def}")
endif()
list(JOIN _b29_argv "\",\"" _b29_argv_json)
set(_b29_argv_json "\"${_b29_argv_json}\"")
# Scratch configures need a >=3.13 project interpreter: find_package(Python)
# probes the interpreter's dependency surface, which only the project venv
# satisfies (homebrew/system pythons cannot import mujoco). The lane
# container provisions it at /opt/bike-venv; fall back to a found python3.
if(EXISTS "${REPO_ROOT}/.venv/bin/python")
  set(_b29_python "${REPO_ROOT}/.venv/bin/python")
elseif(EXISTS "/opt/bike-venv/bin/python")
  set(_b29_python "/opt/bike-venv/bin/python")
else()
  find_program(_b29_python NAMES python3.13 python3 python REQUIRED)
endif()
# Same argv minus the compiler for entries driven through a symlinked driver.
list(SUBLIST _b29_argv 1 -1 _b29_drv_args)
list(JOIN _b29_drv_args "\",\"" _b29_drv_args_json)
set(_b29_drv_args_json "\"${_b29_drv_args_json}\"")

# One fixture target context: strict ISO C++23 with the configured
# hardening definition, everything else empty. On toolchains without
# libc++ hardening support (libstdc++) _b29_def is empty — the required
# definitions list must then be [] rather than [""], else every fixture
# entry "omits target <t> definition ''".
set(_b29_defs_json "[]")
if(NOT _b29_def STREQUAL "")
  set(_b29_defs_json "[\"${_b29_def}\"]")
endif()
# One single-argument string: a multi-argument set would store a ;-list and
# corrupt the surrounding JSON wherever this value interpolates.
set(_b29_tc
  "{\"configuration\":\"fixture\",\"cxx_standard\":23,\"cxx_extensions\":false,\"include_directories\":[],\"compile_definitions\":${_b29_defs_json},\"compile_options\":[]}")

# b29_fixture_context(<builddir> <sourceroot> <target_contexts_json> <suppressions>)
# Writes a copy of the produced context with source_roots, target_contexts
# and the suppression manifest redirected at the fixture.
function(b29_fixture_context builddir sourceroot target_contexts_json suppressions)
  set(_c "${_context_json}")
  string(JSON _c SET "${_c}" source_roots "[\"${sourceroot}\"]")
  string(JSON _c SET "${_c}" target_contexts "${target_contexts_json}")
  string(JSON _c SET "${_c}" analysis_suppressions_manifest
    "\"${suppressions}\"")
  file(WRITE "${builddir}/native_check_context.json" "${_c}")
endfunction()

# b29_ccdb_entry(<out_var> <directory> <source> <output> [extra args...])
# Emits one compile_commands entry in "arguments" form.
function(b29_ccdb_entry out_var dir source output)
  set(_extra "")
  foreach(_arg IN LISTS ARGN)
    set(_extra "${_extra},\"${_arg}\"")
  endforeach()
  # Single-argument set: multiple arguments would store a ;-list and corrupt
  # the JSON wherever this value interpolates.
  set(${out_var}
    "{\"directory\":\"${dir}\",\"file\":\"${source}\",\"output\":\"${output}\",\"arguments\":[${_b29_argv_json}${_extra},\"-c\",\"${source}\",\"-o\",\"${output}\"]}"
    PARENT_SCOPE)
endfunction()

# b29_ccdb_args(<entry-json> <out-var>) — extracts a compile entry's argv as
# a CMake list from either the "arguments" array or the "command" string;
# emitted databases use whichever form the generator produced.
function(b29_ccdb_args entry out_var)
  string(JSON _t ERROR_VARIABLE _e TYPE "${entry}" arguments)
  set(_out "")
  if(NOT _e AND _t STREQUAL "ARRAY")
    string(JSON _n LENGTH "${entry}" arguments)
    if(_n GREATER 0)
      math(EXPR _l "${_n} - 1")
      foreach(_i RANGE 0 ${_l})
        string(JSON _a GET "${entry}" arguments ${_i})
        list(APPEND _out "${_a}")
      endforeach()
    endif()
  else()
    string(JSON _cmd GET "${entry}" command)
    separate_arguments(_out UNIX_COMMAND "${_cmd}")
  endif()
  set(${out_var} "${_out}" PARENT_SCOPE)
endfunction()

# b29_mock_tool(<path> VERSION <v> STABLE_RC <rc> STABLE_TEXT <t>
#               ALPHA_RC <rc> ALPHA_TEXT <t>)
# The stable/alpha split mirrors the reference write_fake_tool: --version
# prints the configured string; -analyzer-checker=alpha arguments take the
# alpha branch; every other invocation takes the stable branch.
function(b29_mock_tool path)
  cmake_parse_arguments(T "" "VERSION;STABLE_RC;STABLE_TEXT;ALPHA_RC;ALPHA_TEXT"
    "" ${ARGN})
  if(NOT DEFINED T_VERSION)
    set(T_VERSION "mock analysis tool 1")
  endif()
  if(NOT DEFINED T_STABLE_RC)
    set(T_STABLE_RC 0)
  endif()
  if(NOT DEFINED T_STABLE_TEXT)
    set(T_STABLE_TEXT "")
  endif()
  if(NOT DEFINED T_ALPHA_RC)
    set(T_ALPHA_RC 0)
  endif()
  if(NOT DEFINED T_ALPHA_TEXT)
    set(T_ALPHA_TEXT "")
  endif()
  file(WRITE "${path}"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--version\" ]; then printf '%s\\n' \"${T_VERSION}\"; exit 0; fi\n"
    "case \"\$*\" in\n"
    "  *-analyzer-checker=alpha*) printf '%s' \"${T_ALPHA_TEXT}\" >&2; exit ${T_ALPHA_RC} ;;\n"
    "  *) printf '%s' \"${T_STABLE_TEXT}\" >&2; exit ${T_STABLE_RC} ;;\n"
    "esac\n")
  file(CHMOD "${path}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endfunction()

# Unset every environment override the sweeps honor so resolution/order and
# leak-through behave deterministically in run_case commands.
set(_b29_clean_env
  --unset=CLANG_TIDY --unset=ANALYZER_CLANG --unset=CPPCHECK --unset=GXX
  --unset=NATIVE_TEST_BUILD_PATH --unset=NATIVE_TEST_PROVENANCE_PATH
  --unset=NATIVE_TEST_SANITIZER_RUNTIME)

# ---------------------------------------------------------------------------
# load_entries selection contracts (fixture manifest + compile database)
set(_fx "${SCRATCH_DIR}/b29-entries")
file(MAKE_DIRECTORY "${_fx}/native/src" "${_fx}/obj")
file(WRITE "${_fx}/native/src/one.cpp" "int first_party() { return 0; }\n")
file(WRITE "${_fx}/native/src/two.cpp" "int second_party() { return 0; }\n")
file(WRITE "${_fx}/native/src/shared.cpp" "int shared_party() { return 0; }\n")
file(WRITE "${_fx}/suppressions.json" "{\"schema_version\":1,\"suppressions\":[]}\n")
file(MAKE_DIRECTORY "${_fx}/build")
b29_fixture_context("${_fx}/build" "${_fx}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx}/suppressions.json")
b29_mock_tool("${_fx}/mock-tidy")
file(WRITE "${_fx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/one.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"}]}\n")
b29_ccdb_entry(_one "${_fx}" "${_fx}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o")
file(WRITE "${_fx}/build/compile_commands.json" "[${_one}]\n")

# both compilation-database forms are accepted
run_case(b29_entries_arguments EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
expect_json(b29_entries_arguments "${_fx}/build/native_check_summaries/tidy.json"
  1 translation_unit_count)
expect_json(b29_entries_arguments "${_fx}/build/native_check_summaries/tidy.json"
  "${_fx}/native/src/one.cpp" selected_sources 0)

set(_fx_cmd "${SCRATCH_DIR}/b29-entries-command")
file(MAKE_DIRECTORY "${_fx_cmd}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_cmd}/build")
list(JOIN _b29_argv " " _b29_cmdline)
file(WRITE "${_fx_cmd}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"command\":\"${_b29_cmdline} -c ${_fx}/native/src/one.cpp"
  " -o CMakeFiles/bike_native.dir/one.cpp.o\"}]\n")
run_case(b29_entries_command EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_cmd}/build")
expect_json(b29_entries_command "${_fx_cmd}/build/native_check_summaries/tidy.json"
  1 translation_unit_count)

# a relative file member resolves against the entry directory
set(_fx_rel "${SCRATCH_DIR}/b29-entries-relative")
file(MAKE_DIRECTORY "${_fx_rel}/native/src" "${_fx_rel}/build")
file(WRITE "${_fx_rel}/native/src/one.cpp" "int first_party() { return 0; }\n")
file(WRITE "${_fx_rel}/suppressions.json" "{\"schema_version\":1,\"suppressions\":[]}\n")
b29_fixture_context("${_fx_rel}/build" "${_fx_rel}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx_rel}/suppressions.json")
file(WRITE "${_fx_rel}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx_rel}/native\"],"
  "\"sources\":[{\"path\":\"${_fx_rel}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_rel}/build/compile_commands.json"
  "[{\"directory\":\"${_fx_rel}\",\"file\":\"native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":[${_b29_argv_json},\"-c\",\"native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_entries_relative EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_rel}/build")
expect_json(b29_entries_relative
  "${_fx_rel}/build/native_check_summaries/tidy.json"
  "${_fx_rel}/native/src/one.cpp" selected_sources 0)

# identical duplicate translation units fail closed
file(WRITE "${_fx}/build/compile_commands.json" "[${_one},${_one}]\n")
run_case(b29_dup_tu EXPECT_EXIT FAIL
  EXPECT_STDERR "duplicate identical translation unit"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
# Restore the single-entry database — later fixtures copy ${_fx}/build.
file(WRITE "${_fx}/build/compile_commands.json" "[${_one}]\n")

# one shared source under two distinct target contexts selects twice
set(_fx_shared "${SCRATCH_DIR}/b29-shared")
file(MAKE_DIRECTORY "${_fx_shared}/build")
b29_fixture_context("${_fx_shared}/build" "${_fx}/native"
  "{\"bike_native\":${_b29_tc},\"native_contract_tests\":${_b29_tc}}"
  "${_fx}/suppressions.json")
file(WRITE "${_fx_shared}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/shared.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"},"
  "{\"path\":\"${_fx}/native/src/shared.cpp\",\"target\":\"native_contract_tests\","
  "\"context\":\"native_contract_tests:fixture\"}]}\n")
b29_ccdb_entry(_shared_a "${_fx}" "${_fx}/native/src/shared.cpp"
  "CMakeFiles/bike_native.dir/shared.cpp.o" "-DVARIANT=1")
b29_ccdb_entry(_shared_b "${_fx}" "${_fx}/native/src/shared.cpp"
  "CMakeFiles/native_contract_tests.dir/shared.cpp.o" "-DVARIANT=2")
file(WRITE "${_fx_shared}/build/compile_commands.json"
  "[${_shared_a},${_shared_b}]\n")
run_case(b29_shared_source EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_shared}/build")
expect_json(b29_shared_source
  "${_fx_shared}/build/native_check_summaries/tidy.json"
  1 unique_source_count)
expect_json(b29_shared_source
  "${_fx_shared}/build/native_check_summaries/tidy.json"
  2 translation_unit_count)

# joined include flags (-I/-isystem/-iquote/-idirafter, no space) satisfy a
# declared target include directory
foreach(_flag I isystem iquote idirafter)
  file(MAKE_DIRECTORY "${_fx}/target-include")
  set(_fx_join "${SCRATCH_DIR}/b29-joined-${_flag}")
  file(MAKE_DIRECTORY "${_fx_join}/build")
  set(_c "${_context_json}")
  string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
  string(JSON _c SET "${_c}" target_contexts
    "{\"bike_native\":{\"configuration\":\"fixture\",\"cxx_standard\":23,\"cxx_extensions\":false,\"include_directories\":[\"${_fx}/target-include\"],\"compile_definitions\":${_b29_defs_json},\"compile_options\":[]}}")
  string(JSON _c SET "${_c}" analysis_suppressions_manifest
    "\"${_fx}/suppressions.json\"")
  file(WRITE "${_fx_join}/build/native_check_context.json" "${_c}")
  file(WRITE "${_fx_join}/build/native_sources.json"
    "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],"
    "\"sources\":[{\"path\":\"${_fx}/native/src/one.cpp\","
    "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
  b29_ccdb_entry(_joined "${_fx}" "${_fx}/native/src/one.cpp"
    "CMakeFiles/bike_native.dir/one.cpp.o" "-${_flag}${_fx}/target-include")
  file(WRITE "${_fx_join}/build/compile_commands.json" "[${_joined}]\n")
  run_case(b29_joined_${_flag} EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "CLANG_TIDY=${_fx}/mock-tidy"
          "${CHECKS_BINARY}" --kind tidy --build "${_fx_join}/build")
endforeach()

# an empty compile database fails closed when sources are expected
set(_fx_empty "${SCRATCH_DIR}/b29-db-empty")
file(MAKE_DIRECTORY "${_fx_empty}/build")
file(COPY "${_fx_shared}/build/native_check_context.json"
          "${_fx_shared}/build/native_sources.json"
     DESTINATION "${_fx_empty}/build")
file(WRITE "${_fx_empty}/build/compile_commands.json" "[]\n")
run_case(b29_empty_db EXPECT_EXIT FAIL
  EXPECT_STDERR "empty translation-unit selection"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_empty}/build")

# a manifest-referenced source file that is absent fails before any tool runs
set(_fx_gone "${SCRATCH_DIR}/b29-missing-source")
file(MAKE_DIRECTORY "${_fx_gone}/build")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
file(WRITE "${_fx_gone}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_gone}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/one.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"},"
  "{\"path\":\"${_fx}/native/src/absent.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"}]}\n")
b29_ccdb_entry(_gone "${_fx}" "${_fx}/native/src/absent.cpp"
  "CMakeFiles/bike_native.dir/absent.cpp.o")
file(WRITE "${_fx_gone}/build/compile_commands.json" "[${_one},${_gone}]\n")
run_case(b29_missing_source EXPECT_EXIT FAIL
  EXPECT_STDERR "missing source file"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_gone}/build")

# a first-party database entry the manifest never declared fails selection
set(_fx_extra "${SCRATCH_DIR}/b29-unexpected")
file(MAKE_DIRECTORY "${_fx_extra}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_extra}/build")
b29_ccdb_entry(_extra "${_fx}" "${_fx}/native/src/two.cpp"
  "CMakeFiles/bike_native.dir/two.cpp.o")
file(WRITE "${_fx_extra}/build/compile_commands.json" "[${_one},${_extra}]\n")
run_case(b29_unexpected_tu EXPECT_EXIT FAIL
  EXPECT_STDERR "unexpected first-party translation unit"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_extra}/build")

# a target-carrying manifest with an unattributable output fails closed
set(_fx_notarget "${SCRATCH_DIR}/b29-no-target")
file(MAKE_DIRECTORY "${_fx_notarget}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_notarget}/build")
file(WRITE "${_fx_notarget}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"one.o\","
  "\"arguments\":[${_b29_argv_json},\"-c\",\"${_fx}/native/src/one.cpp\","
  "\"-o\",\"one.o\"]}]\n")
run_case(b29_no_target EXPECT_EXIT FAIL
  EXPECT_STDERR "target context is unavailable"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_notarget}/build")

# a -I directory that does not exist fails closed
set(_fx_noinc "${SCRATCH_DIR}/b29-missing-include")
file(MAKE_DIRECTORY "${_fx_noinc}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_noinc}/build")
b29_ccdb_entry(_noinc "${_fx}" "${_fx}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o" "-I" "/nonexistent/missing-include")
file(WRITE "${_fx_noinc}/build/compile_commands.json" "[${_noinc}]\n")
run_case(b29_missing_include EXPECT_EXIT FAIL
  EXPECT_STDERR "missing dependency path"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_noinc}/build")

# the entry's sysroot must match the configured sysroot exactly
set(_fx_sysroot "${SCRATCH_DIR}/b29-sysroot-mismatch")
file(MAKE_DIRECTORY "${_fx_sysroot}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_sysroot}/build")
b29_ccdb_entry(_sysbad "${_fx}" "${_fx}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o" "-isysroot" "/nonexistent/other-sdk")
file(WRITE "${_fx_sysroot}/build/compile_commands.json" "[${_sysbad}]\n")
if(_b29_sysroot)
  set(_b29_sysroot_needle "sysroot differs from configured")
else()
  set(_b29_sysroot_needle "but the build context configures none")
endif()
run_case(b29_sysroot_mismatch EXPECT_EXIT FAIL
  EXPECT_STDERR "${_b29_sysroot_needle}"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sysroot}/build")

# the entry's compiler must match the configured compiler
set(_fx_wrongcc "${SCRATCH_DIR}/b29-compiler-mismatch")
file(MAKE_DIRECTORY "${_fx_wrongcc}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
     DESTINATION "${_fx_wrongcc}/build")
file(WRITE "${_fx_wrongcc}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":[\"/bin/sh\",\"-std=c++23\",\"-c\","
  "\"${_fx}/native/src/one.cpp\",\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_compiler_mismatch EXPECT_EXIT FAIL
  EXPECT_STDERR "compiler differs from configured C++ compiler"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_wrongcc}/build")

# context include_paths entries must be directories
set(_fx_badinc "${SCRATCH_DIR}/b29-include-paths")
file(MAKE_DIRECTORY "${_fx_badinc}")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" include_paths "[\"${_fx}/native/src/one.cpp\"]")
file(WRITE "${_fx_badinc}/native_check_context.json" "${_c}")
run_case(b29_include_paths EXPECT_EXIT FAIL
  EXPECT_STDERR "context include_paths entry is not a directory"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fx_badinc}"
        --validate-only)

# context library_paths directories must be .framework bundles
set(_fx_badlib "${SCRATCH_DIR}/b29-library-paths")
file(MAKE_DIRECTORY "${_fx_badlib}/plain-dir")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" library_paths "[\"${_fx_badlib}/plain-dir\"]")
file(WRITE "${_fx_badlib}/native_check_context.json" "${_c}")
run_case(b29_library_paths EXPECT_EXIT FAIL
  EXPECT_STDERR "context library_paths entry is neither a library file nor a framework directory"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fx_badlib}"
        --validate-only)

# a recorded compiler path keeps its symlink spelling verbatim
set(_fx_link "${SCRATCH_DIR}/b29-compiler-symlink")
file(MAKE_DIRECTORY "${_fx_link}/build")
file(CREATE_LINK "${_b29_compiler}" "${_fx_link}/cxx-driver" SYMBOLIC)
set(_c "${_context_json}")
string(JSON _c SET "${_c}" compiler path "\"${_fx_link}/cxx-driver\"")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
file(WRITE "${_fx_link}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_link}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/one.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_link}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":[\"${_fx_link}/cxx-driver\",${_b29_drv_args_json},"
  "\"-c\",\"${_fx}/native/src/one.cpp\",\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_compiler_symlink EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_link}/build")
expect_json(b29_compiler_symlink
  "${_fx_link}/build/native_check_summaries/tidy.json"
  "${_fx_link}/cxx-driver" compiler_context path)

# a missing build directory fails before translation-unit selection
run_case(b29_tidy_missing_build EXPECT_EXIT FAIL
  EXPECT_STDERR "build directory does not exist"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind tidy --build "${SCRATCH_DIR}/not-configured")

# the shell wrapper routes a missing build to the fail-closed controller.
# BIKE_NATIVE_CHECKS pins the produced binary so the case does not depend
# on the repo-root fallback existing inside the lane (the container keeps
# build trees under /build, not /src/build).
run_case(b29_tidy_wrapper_missing_build EXPECT_EXIT FAIL
  EXPECT_STDERR "build directory does not exist"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "BIKE_NATIVE_CHECKS=${CHECKS_BINARY}"
        bash "${REPO_ROOT}/tools/check_native_tidy.sh"
        "${SCRATCH_DIR}/not-configured" "${REPO_ROOT}/native/src")

# ---------------------------------------------------------------------------
# sweep bookkeeping: tool health and finding classification fail closed.
# One shared single-TU fixture; only the mock tool's stable behavior varies.
set(_fx_sweep "${SCRATCH_DIR}/b29-sweep")
file(MAKE_DIRECTORY "${_fx_sweep}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
          "${_fx}/build/compile_commands.json"
     DESTINATION "${_fx_sweep}/build")
set(_sweep_summary "${_fx_sweep}/build/native_check_summaries/tidy.json")

b29_mock_tool("${_fx_sweep}/rc127-tool" STABLE_RC 127 STABLE_TEXT "tool not found")
run_case(b29_stable_rc127 EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_sweep}/rc127-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sweep}/build")
expect_json(b29_stable_rc127 "${_sweep_summary}" "failed" tool_health)
expect_json(b29_stable_rc127 "${_sweep_summary}" "none" finding_status)

b29_mock_tool("${_fx_sweep}/rc2-tool" STABLE_RC 2
  STABLE_TEXT "error: analysis failed")
run_case(b29_stable_rc2 EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_sweep}/rc2-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sweep}/build")
expect_json(b29_stable_rc2 "${_sweep_summary}" "failed" tool_health)
expect_json(b29_stable_rc2 "${_sweep_summary}" "blocking" finding_status)

b29_mock_tool("${_fx_sweep}/rc139-tool" STABLE_RC 139
  STABLE_TEXT "compiler crash")
run_case(b29_stable_rc139 EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_sweep}/rc139-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sweep}/build")
expect_json(b29_stable_rc139 "${_sweep_summary}" "failed" tool_health)
expect_json(b29_stable_rc139 "${_sweep_summary}" "none" finding_status)

b29_mock_tool("${_fx_sweep}/warn-tool" STABLE_RC 0
  STABLE_TEXT "warning: unsafe operation")
run_case(b29_stable_warn EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_sweep}/warn-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sweep}/build")
expect_json(b29_stable_warn "${_sweep_summary}" "ok" tool_health)
expect_json(b29_stable_warn "${_sweep_summary}" "blocking" finding_status)

# alpha findings are report-only; an alpha crash still fails tool health
set(_fx_alpha "${SCRATCH_DIR}/b29-alpha")
file(MAKE_DIRECTORY "${_fx_alpha}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
          "${_fx}/build/compile_commands.json"
     DESTINATION "${_fx_alpha}/build")
set(_alpha_summary "${_fx_alpha}/build/native_check_summaries/analyzer.json")

b29_mock_tool("${_fx_alpha}/report-tool"
  ALPHA_TEXT "warning: experimental finding")
run_case(b29_alpha_report EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "ANALYZER_CLANG=${_fx_alpha}/report-tool"
        "${CHECKS_BINARY}" --kind analyzer --build "${_fx_alpha}/build")
expect_json(b29_alpha_report "${_alpha_summary}" "ok" tool_health)
expect_json(b29_alpha_report "${_alpha_summary}" "report-only" finding_status)
expect_json(b29_alpha_report "${_alpha_summary}" 1 translation_unit_count)

b29_mock_tool("${_fx_alpha}/crash-tool" ALPHA_RC 139
  ALPHA_TEXT "segmentation fault")
run_case(b29_alpha_crash EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "ANALYZER_CLANG=${_fx_alpha}/crash-tool"
        "${CHECKS_BINARY}" --kind analyzer --build "${_fx_alpha}/build")
expect_json(b29_alpha_crash "${_alpha_summary}" "failed" tool_health)

# positive control: one selected translation unit, clean records
b29_mock_tool("${_fx_sweep}/clean-tool")
run_case(b29_positive EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_sweep}/clean-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sweep}/build")
expect_json(b29_positive "${_sweep_summary}" "ok" tool_health)
expect_json(b29_positive "${_sweep_summary}" "none" finding_status)
expect_json(b29_positive "${_sweep_summary}" 1 unique_source_count)
expect_json(b29_positive "${_sweep_summary}" 1 translation_unit_count)
expect_json(b29_positive "${_sweep_summary}"
  "${_fx}/native/src/one.cpp" selected_sources 0)
expect_json(b29_positive "${_sweep_summary}" 0 records 0 returncode)

# a configured tool default runs without any environment override
set(_fx_default "${SCRATCH_DIR}/b29-tool-default")
file(MAKE_DIRECTORY "${_fx_default}/build")
b29_mock_tool("${_fx_default}/configured-tidy")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
# A driver-named symlink keeps compiler-sibling candidates out of the
# resolution path on every compiler family, so the context default wins.
file(CREATE_LINK "${_b29_compiler}" "${_fx_default}/cxx-driver" SYMBOLIC)
string(JSON _c SET "${_c}" compiler path "\"${_fx_default}/cxx-driver\"")
string(JSON _c SET "${_c}" tool_defaults tidy "\"${_fx_default}/configured-tidy\"")
file(WRITE "${_fx_default}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_default}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/one.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_default}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":[\"${_fx_default}/cxx-driver\",${_b29_drv_args_json},"
  "\"-c\",\"${_fx}/native/src/one.cpp\",\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_tidy_default EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_default}/build")
expect_json(b29_tidy_default
  "${_fx_default}/build/native_check_summaries/tidy.json"
  "${_fx_default}/configured-tidy" tool)

# the gxx context default runs frontends and odr without GXX in the env
set(_fx_gxx "${SCRATCH_DIR}/b29-gxx-default")
file(MAKE_DIRECTORY "${_fx_gxx}/build")
b29_mock_tool("${_fx_gxx}/configured-gxx")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
file(CREATE_LINK "${_b29_compiler}" "${_fx_gxx}/cxx-driver" SYMBOLIC)
string(JSON _c SET "${_c}" compiler path "\"${_fx_gxx}/cxx-driver\"")
string(JSON _c SET "${_c}" tool_defaults gxx "\"${_fx_gxx}/configured-gxx\"")
file(WRITE "${_fx_gxx}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_gxx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":["
  "{\"path\":\"${_fx}/native/src/one.cpp\",\"target\":\"bike_native\","
  "\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_gxx}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":[\"${_fx_gxx}/cxx-driver\",${_b29_drv_args_json},"
  "\"-c\",\"${_fx}/native/src/one.cpp\",\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_gxx_frontends EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind frontends --build "${_fx_gxx}/build")
expect_json(b29_gxx_frontends
  "${_fx_gxx}/build/native_check_summaries/frontends.json"
  "${_fx_gxx}/configured-gxx" tool)
run_case(b29_gxx_odr EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind odr --build "${_fx_gxx}/build")
expect_json(b29_gxx_odr
  "${_fx_gxx}/build/native_check_summaries/odr.json"
  "${_fx_gxx}/configured-gxx" tool)

# ---------------------------------------------------------------------------
# HeaderFilterRegex is checkout-portable: a relocated fixture TU under a
# differently named root keeps header diagnostics, and main-file
# diagnostics are never filtered out. Uses the produced build's real
# clang-tidy (context tool resolution) on a relocated compile entry.
string(JSON _b29_tidy_default GET "${_context_json}" tool_defaults tidy)
string(JSON _b29_tidy_override ERROR_VARIABLE _b29_tov
  GET "${_context_json}" tool_overrides CLANG_TIDY)
if(NOT _b29_tov AND NOT _b29_tidy_override STREQUAL "")
  set(_b29_tidy_tool "${_b29_tidy_override}")
else()
  set(_b29_tidy_tool "${_b29_tidy_default}")
endif()
# Reuse the checkout's configured filter verbatim — the fixture .clang-tidy
# only narrows Checks to the control checker.
file(STRINGS "${REPO_ROOT}/native/.clang-tidy" _b29_hfilter_lines
  REGEX "^HeaderFilterRegex")
if(NOT _b29_hfilter_lines)
  set(_failures
    "${_failures}\n  b29_headerfilter: no HeaderFilterRegex in native/.clang-tidy")
endif()
list(GET _b29_hfilter_lines 0 _b29_hfilter_regex)
# The fixture compile entry reuses the produced argv (configured compiler,
# strict ISO C++23, resolved sysroot and the hardening definition) plus the
# fixture include dir so the real tidy resolves the planted header. The
# oracle relocates a produced entry; the native port builds the equivalent
# argv directly — every argument the sweep validates is already present and
# no produced-argv positional quirk (separate -isysroot/-I operands, output
# paths) can corrupt the fixture entry.
list(JOIN _b29_argv "\",\"" _b29_entry_argv_json)
set(_b29_entry_argv_json "\"${_b29_entry_argv_json}\"")

function(b29_header_fixture root diagnostic_in_header)
  file(MAKE_DIRECTORY "${root}/native/src" "${root}/build")
  set(_header "namespace control { inline double divide(int a, int b) { return a / b; } }\n")
  set(_source "#include \"control.hpp\"\nint main() { return static_cast<int>(control::divide(1, 2)); }\n")
  if(NOT diagnostic_in_header)
    set(_header "namespace control { inline int value() { return 0; } }\n")
    set(_source "#include \"control.hpp\"\ndouble divide(int a, int b) { return a / b; }\nint main() { return static_cast<int>(divide(1, 2) + control::value()); }\n")
  endif()
  file(WRITE "${root}/native/src/control.hpp" "${_header}")
  file(WRITE "${root}/native/src/control.cpp" "${_source}")
  file(WRITE "${root}/native/.clang-tidy"
    "Checks: 'bugprone-integer-division'\n${_b29_hfilter_regex}\n")
  set(_c "${_context_json}")
  string(JSON _c SET "${_c}" source_roots "[\"${root}/native\"]")
  string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
  file(WRITE "${root}/build/native_check_context.json" "${_c}")
  file(WRITE "${root}/build/native_sources.json"
    "{\"schema_version\":1,\"first_party_roots\":[\"${root}/native\"],"
    "\"sources\":[{\"path\":\"${root}/native/src/control.cpp\","
    "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
  file(WRITE "${root}/build/compile_commands.json"
    "[{\"directory\":\"${root}\","
    "\"file\":\"${root}/native/src/control.cpp\","
    "\"output\":\"CMakeFiles/bike_native.dir/control.cpp.o\","
    "\"arguments\":[${_b29_entry_argv_json},"
    "\"-I\",\"${root}/native/src\",\"-c\",\"${root}/native/src/control.cpp\","
    "\"-o\",\"CMakeFiles/bike_native.dir/control.cpp.o\"]}]\n")
endfunction()

foreach(_root_name original-root renamed-root)
  b29_header_fixture("${SCRATCH_DIR}/b29-hfilter-${_root_name}" TRUE)
  run_case(b29_hfilter_${_root_name} EXPECT_EXIT FAIL
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "CLANG_TIDY=${_b29_tidy_tool}"
          "${CHECKS_BINARY}" --kind tidy
          --build "${SCRATCH_DIR}/b29-hfilter-${_root_name}/build")
  expect_json_contains(b29_hfilter_${_root_name}
    "${SCRATCH_DIR}/b29-hfilter-${_root_name}/build/native_check_summaries/tidy.json"
    "bugprone-integer-division" records 0 unsuppressed_diagnostics)
endforeach()
b29_header_fixture("${SCRATCH_DIR}/b29-hfilter-main" FALSE)
run_case(b29_hfilter_main EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_b29_tidy_tool}"
        "${CHECKS_BINARY}" --kind tidy
        --build "${SCRATCH_DIR}/b29-hfilter-main/build")
expect_json_contains(b29_hfilter_main
  "${SCRATCH_DIR}/b29-hfilter-main/build/native_check_summaries/tidy.json"
  "bugprone-integer-division" records 0 unsuppressed_diagnostics)

# ---------------------------------------------------------------------------
# Produced-configure contracts: scratch cmake configures against the real
# native/ graph exercise the same probes the reference file configured.

# An unsupported hardening name fails configuration outright.
set(_cfg_bad "${SCRATCH_DIR}/b29-cfg-hardening-invalid")
run_case(b29_hardening_invalid EXPECT_EXIT FAIL
  EXPECT_STDERR "BIKE_LIBCPP_HARDENING"
  COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_bad}"
        "-DCMAKE_BUILD_TYPE=Release"
        "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
        "-DPython_EXECUTABLE=${_b29_python}"
        "-DBIKE_LIBCPP_HARDENING=INVALID")

# Every first-party compile command uses strict ISO C++23.
set(_cfg_iso "${SCRATCH_DIR}/b29-cfg-iso23")
set(_iso_options "-DCMAKE_BUILD_TYPE=Release"
  "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
  "-DPython_EXECUTABLE=${_b29_python}")
if(_t56_system STREQUAL "Darwin")
  list(APPEND _iso_options "-DCMAKE_OSX_SYSROOT=${_b29_sysroot}")
endif()
run_case(b29_iso23 EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_iso}"
        ${_iso_options})
if(EXISTS "${_cfg_iso}/compile_commands.json")
  file(READ "${_cfg_iso}/compile_commands.json" _iso_db)
  string(JSON _iso_count LENGTH "${_iso_db}")
  math(EXPR _iso_last "${_iso_count} - 1")
  set(_iso_native 0)
  set(_iso_bad "")
  foreach(_i RANGE 0 ${_iso_last})
    string(JSON _e GET "${_iso_db}" ${_i})
    string(JSON _f GET "${_e}" file)
    if(NOT _f MATCHES "^${REPO_ROOT}/native/")
      continue()
    endif()
    math(EXPR _iso_native "${_iso_native} + 1")
    b29_ccdb_args("${_e}" _iso_args)
    set(_iso_ok FALSE)
    set(_iso_gnu FALSE)
    list(FIND _iso_args "-std=c++23" _iso_idx)
    if(NOT _iso_idx EQUAL -1)
      set(_iso_ok TRUE)
    endif()
    list(FIND _iso_args "-std=gnu++23" _gnu_idx)
    if(NOT _gnu_idx EQUAL -1)
      set(_iso_gnu TRUE)
    endif()
    if(NOT _iso_ok OR _iso_gnu)
      set(_iso_bad "${_iso_bad}\n    ${_f}")
    endif()
  endforeach()
  if(_iso_native EQUAL 0)
    set(_failures "${_failures}\n  b29_iso23: no first-party entries selected")
  elseif(NOT _iso_bad STREQUAL "")
    set(_failures "${_failures}\n  b29_iso23: non-ISO entries:${_iso_bad}")
  endif()
else()
  set(_failures "${_failures}\n  b29_iso23: no compile_commands.json emitted")
endif()

if(_t56_system STREQUAL "Darwin")
  # An explicit versioned SDK alias is retained as requested and
  # canonicalized into every compile command.
  file(GLOB _sdk_siblings
    LIST_DIRECTORIES true "${_b29_sysroot}/../MacOSX*.sdk")
  set(_b29_alias "")
  foreach(_candidate IN LISTS _sdk_siblings)
    if(NOT _candidate STREQUAL _b29_sysroot)
      file(REAL_PATH "${_candidate}" _real)
      file(REAL_PATH "${_b29_sysroot}" _sdk_real)
      if(_real STREQUAL _sdk_real)
        set(_b29_alias "${_candidate}")
      endif()
    endif()
  endforeach()
  if(_b29_alias STREQUAL "")
    set(_failures
      "${_failures}\n  b29_sdk_alias: no versioned MacOSX*.sdk sibling resolves to ${_b29_sysroot}")
  else()
    set(_cfg_alias "${SCRATCH_DIR}/b29-cfg-sdk-alias")
    run_case(b29_sdk_alias EXPECT_EXIT 0
      COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_alias}"
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_OSX_SYSROOT=${_b29_alias}"
            "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
            "-DPython_EXECUTABLE=${_b29_python}")
    expect_json(b29_sdk_alias "${_cfg_alias}/native_check_context.json"
      "${_b29_alias}" sdk_requested)
    file(REAL_PATH "${_b29_alias}" _alias_real)
    set(_alias_sysroot_bad FALSE)
    if(EXISTS "${_cfg_alias}/compile_commands.json")
      file(READ "${_cfg_alias}/compile_commands.json" _alias_db)
      string(JSON _alias_count LENGTH "${_alias_db}")
      math(EXPR _alias_last "${_alias_count} - 1")
      set(_alias_seen FALSE)
      foreach(_i RANGE 0 ${_alias_last})
        string(JSON _e GET "${_alias_db}" ${_i})
        string(JSON _f GET "${_e}" file)
        if(NOT _f MATCHES "^${REPO_ROOT}/native/")
          continue()
        endif()
        b29_ccdb_args("${_e}" _alias_args)
        list(FIND _alias_args "-isysroot" _sr_idx)
        if(NOT _sr_idx EQUAL -1)
          math(EXPR _k "${_sr_idx} + 1")
          list(GET _alias_args ${_k} _sr)
          file(REAL_PATH "${_sr}" _sr_real)
          set(_alias_seen TRUE)
          if(NOT _sr_real STREQUAL _alias_real)
            set(_alias_sysroot_bad TRUE)
          endif()
        endif()
      endforeach()
      if(NOT _alias_seen)
        set(_failures "${_failures}\n  b29_sdk_alias: no -isysroot selected")
      elseif(_alias_sysroot_bad)
        set(_failures
          "${_failures}\n  b29_sdk_alias: entry sysroot differs from alias")
      endif()
    else()
      set(_failures "${_failures}\n  b29_sdk_alias: no compile_commands.json")
    endif()
  endif()

  # A missing explicit SDK fails without silently falling back.
  run_case(b29_sdk_missing EXPECT_EXIT FAIL
    EXPECT_STDERR "${SCRATCH_DIR}/missing-explicit-sdk"
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native"
          -B "${SCRATCH_DIR}/b29-cfg-sdk-missing"
          "-DCMAKE_BUILD_TYPE=Release"
          "-DCMAKE_OSX_SYSROOT=${SCRATCH_DIR}/missing-explicit-sdk"
          "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
          "-DPython_EXECUTABLE=${_b29_python}")

  # A default configure resolves the SDK from xcrun.
  execute_process(COMMAND xcrun --sdk macosx --show-sdk-path
    OUTPUT_VARIABLE _xcrun_sdk RESULT_VARIABLE _xcrun_rc
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(_cfg_sdk "${SCRATCH_DIR}/b29-cfg-sdk-default")
  run_case(b29_sdk_default EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_sdk}"
          "-DCMAKE_BUILD_TYPE=Release"
          "-DPython_EXECUTABLE=${_b29_python}")
  if(_xcrun_rc EQUAL 0)
    file(REAL_PATH "${_xcrun_sdk}" _xcrun_real)
    expect_json(b29_sdk_default "${_cfg_sdk}/native_check_context.json"
      "default" sdk_source)
    if(EXISTS "${_cfg_sdk}/native_check_context.json")
      file(READ "${_cfg_sdk}/native_check_context.json" _sdk_ctx)
      string(JSON _sdk_path GET "${_sdk_ctx}" sdk)
      file(REAL_PATH "${_sdk_path}" _sdk_path_real)
      if(NOT _sdk_path_real STREQUAL _xcrun_real)
        set(_failures
          "${_failures}\n  b29_sdk_default: sdk '${_sdk_path_real}' != xcrun '${_xcrun_real}'")
      endif()
    endif()
  else()
    set(_failures "${_failures}\n  b29_sdk_default: xcrun probe failed")
  endif()

  # An empty build type defaults to EXTENSIVE libc++ hardening.
  set(_cfg_hel "${SCRATCH_DIR}/b29-cfg-hardening-default")
  run_case(b29_hardening_default EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_hel}"
          "-DCMAKE_BUILD_TYPE="
          "-DCMAKE_OSX_SYSROOT=${_b29_sysroot}"
          "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
          "-DPython_EXECUTABLE=${_b29_python}")
  expect_json(b29_hardening_default
    "${_cfg_hel}/native_check_context.json" "EXTENSIVE"
    libcpp_hardening mode)
  expect_json(b29_hardening_default
    "${_cfg_hel}/native_check_context.json" "ON"
    libcpp_hardening supported)

  # FAST is available as an explicit comparison and lands in every entry.
  set(_cfg_fast "${SCRATCH_DIR}/b29-cfg-hardening-fast")
  run_case(b29_hardening_fast EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}/native" -B "${_cfg_fast}"
          "-DCMAKE_BUILD_TYPE=Release"
          "-DCMAKE_OSX_SYSROOT=${_b29_sysroot}"
          "-DCMAKE_CXX_COMPILER=${_b29_compiler}"
          "-DPython_EXECUTABLE=${_b29_python}"
          "-DBIKE_LIBCPP_HARDENING=FAST")
  expect_json(b29_hardening_fast
    "${_cfg_fast}/native_check_context.json" "FAST" libcpp_hardening mode)
  if(EXISTS "${_cfg_fast}/compile_commands.json")
    file(READ "${_cfg_fast}/compile_commands.json" _fast_db)
    string(JSON _fast_count LENGTH "${_fast_db}")
    math(EXPR _fast_last "${_fast_count} - 1")
    set(_fast_native 0)
    set(_fast_bad "")
    foreach(_i RANGE 0 ${_fast_last})
      string(JSON _e GET "${_fast_db}" ${_i})
      string(JSON _f GET "${_e}" file)
      if(NOT _f MATCHES "^${REPO_ROOT}/native/")
        continue()
      endif()
      math(EXPR _fast_native "${_fast_native} + 1")
      b29_ccdb_args("${_e}" _fast_args)
      set(_has_fast FALSE)
      list(FIND _fast_args
        "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST" _fast_idx)
      if(NOT _fast_idx EQUAL -1)
        set(_has_fast TRUE)
      endif()
      if(NOT _has_fast)
        set(_fast_bad "${_fast_bad}\n    ${_f}")
      endif()
    endforeach()
    if(_fast_native EQUAL 0)
      set(_failures "${_failures}\n  b29_hardening_fast: no first-party entries")
    elseif(NOT _fast_bad STREQUAL "")
      set(_failures
        "${_failures}\n  b29_hardening_fast: entries missing FAST define:${_fast_bad}")
    endif()
  else()
    set(_failures "${_failures}\n  b29_hardening_fast: no compile_commands.json")
  endif()
endif()

# ---------------------------------------------------------------------------
# Suppression manifest completeness (oracle: every real entry carries all
# seven fields; the C++ loader enforces the same contract fail-closed).
file(READ "${REPO_ROOT}/native/analysis_suppressions.json" _sup_doc)
string(JSON _sup_schema ERROR_VARIABLE _sup_err GET "${_sup_doc}" schema_version)
if(_sup_err OR NOT _sup_schema STREQUAL "1")
  set(_failures
    "${_failures}\n  b29_suppression_manifest: schema_version is '${_sup_schema}'")
endif()
string(JSON _sup_count ERROR_VARIABLE _sup_err2 LENGTH "${_sup_doc}" suppressions)
if(_sup_err2)
  set(_failures
    "${_failures}\n  b29_suppression_manifest: suppressions is not a list")
elseif(_sup_count GREATER 0)
  # An empty list is valid (zero suppressions); guard the RANGE loop —
  # 'foreach(RANGE 0 -1)' would otherwise iterate indices 0 and -1.
  math(EXPR _sup_last "${_sup_count} - 1")
  foreach(_i RANGE 0 ${_sup_last})
    foreach(_field checker origin message tool_version reason reproducer
            remove_when)
      string(JSON _v ERROR_VARIABLE _e
        GET "${_sup_doc}" suppressions ${_i} ${_field})
      if(_e)
        set(_failures
          "${_failures}\n  b29_suppression_manifest: entry ${_i} lacks '${_field}'")
      endif()
    endforeach()
  endforeach()
endif()

# Suppression matching: only the diagnostic whose checker, origin regex,
# message substring and tool-version substring all match is filtered out —
# the fixture sweeps a mock tidy emitting the oracle's four-line matrix.
set(_fx_supp "${SCRATCH_DIR}/b29-suppression")
file(MAKE_DIRECTORY "${_fx_supp}/build" "${_fx_supp}/native/src")
file(WRITE "${_fx_supp}/native/src/one.cpp" "int first_party() { return 0; }\n")
file(WRITE "${_fx_supp}/suppressions.json"
  "{\"schema_version\":1,\"suppressions\":[{"
  "\"checker\":\"cplusplus.Move\","
  "\"origin\":\"/nanobind/include/nanobind/stl/detail/nb_list\\\\.h$\","
  "\"message\":\"documented array-caster lifetime false positive\","
  "\"tool_version\":\"23.1.2\","
  "\"reason\":\"reproduced false positive in a third-party nanobind header\","
  "\"reproducer\":\"test_suppression_matching\","
  "\"remove_when\":\"the installed analyzer no longer reports this diagnostic\""
  "}]}\n")
b29_fixture_context("${_fx_supp}/build" "${_fx_supp}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx_supp}/suppressions.json")
file(WRITE "${_fx_supp}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx_supp}/native\"],"
  "\"sources\":[{\"path\":\"${_fx_supp}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
b29_ccdb_entry(_supp_e "${_fx_supp}" "${_fx_supp}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o")
file(WRITE "${_fx_supp}/build/compile_commands.json" "[${_supp_e}]\n")
set(_supp_match "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: warning: documented array-caster lifetime false positive [cplusplus.Move]")
set(_supp_fp "${_fx_supp}/native/src/one.cpp:67:4: warning: documented array-caster lifetime false positive [cplusplus.Move]")
set(_supp_om "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: warning: another move diagnostic [cplusplus.Move]")
set(_supp_oc "/venv/site-packages/nanobind/include/nanobind/stl/detail/nb_list.h:67:4: warning: documented array-caster lifetime false positive [bugprone-use-after-move]")
b29_mock_tool("${_fx_supp}/mock-tidy"
  VERSION "Homebrew LLVM version 23.1.2"
  STABLE_TEXT "${_supp_match}\n${_supp_fp}\n${_supp_om}\n${_supp_oc}")
set(_supp_sum "${_fx_supp}/build/native_check_summaries/tidy.json")
run_case(b29_suppression_match EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_supp}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_supp}/build")
expect_json(b29_suppression_match "${_supp_sum}" 1
  suppression_counts cplusplus.Move)
expect_json_length(b29_suppression_match "${_supp_sum}" 1
  records 0 suppressed_diagnostics)
expect_json(b29_suppression_match "${_supp_sum}" "${_supp_match}"
  records 0 suppressed_diagnostics 0 diagnostic)
expect_json_length(b29_suppression_match "${_supp_sum}" 3
  records 0 unsuppressed_diagnostics)
expect_json(b29_suppression_match "${_supp_sum}" "${_supp_fp}"
  records 0 unsuppressed_diagnostics 0)
expect_json(b29_suppression_match "${_supp_sum}" "${_supp_om}"
  records 0 unsuppressed_diagnostics 1)
expect_json(b29_suppression_match "${_supp_sum}" "${_supp_oc}"
  records 0 unsuppressed_diagnostics 2)

# A tool whose version no longer carries the pinned string suppresses
# nothing (the oracle's version-mismatch half).
b29_mock_tool("${_fx_supp}/mock-tidy"
  VERSION "Homebrew LLVM version 24.0.0"
  STABLE_TEXT "${_supp_match}")
run_case(b29_suppression_version EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_supp}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_supp}/build")
expect_json_length(b29_suppression_version "${_supp_sum}" 1
  records 0 unsuppressed_diagnostics)
expect_json_length(b29_suppression_version "${_supp_sum}" 0
  records 0 suppressed_diagnostics)

# The clang-analyzer move suppression stays scoped to the nanobind origin:
# a first-party diagnostic with the same checker/message keeps blocking.
set(_fx_move "${SCRATCH_DIR}/b29-move-suppression")
file(MAKE_DIRECTORY "${_fx_move}/build" "${_fx_move}/native/src")
file(WRITE "${_fx_move}/native/src/one.cpp" "int first_party() { return 0; }\n")
file(WRITE "${_fx_move}/suppressions.json"
  "{\"schema_version\":1,\"suppressions\":[{"
  "\"checker\":\"clang-analyzer-cplusplus.Move\","
  "\"origin\":\"(?:^|/)nanobind/include/nanobind/stl/detail/nb_array\\\\.h$\","
  "\"message\":\"Method called on moved-from object 'value' of type 'std::array'\","
  "\"tool_version\":\"1\","
  "\"reason\":\"specific third-party false-positive control\","
  "\"reproducer\":\"test_clang_tidy_move_suppression_is_limited_to_nanobind_origin\","
  "\"remove_when\":\"the installed checker no longer emits this diagnostic\""
  "}]}\n")
b29_fixture_context("${_fx_move}/build" "${_fx_move}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx_move}/suppressions.json")
file(WRITE "${_fx_move}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx_move}/native\"],"
  "\"sources\":[{\"path\":\"${_fx_move}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
b29_ccdb_entry(_move_e "${_fx_move}" "${_fx_move}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o")
file(WRITE "${_fx_move}/build/compile_commands.json" "[${_move_e}]\n")
set(_move_3p "/venv/lib/python3.13/site-packages/nanobind/include/nanobind/stl/detail/nb_array.h:34:17: warning: Method called on moved-from object 'value' of type 'std::array' [clang-analyzer-cplusplus.Move]")
set(_move_fp "${_fx_move}/native/src/one.cpp:4:5: warning: Method called on moved-from object 'value' of type 'std::array' [clang-analyzer-cplusplus.Move]")
b29_mock_tool("${_fx_move}/mock-tidy"
  STABLE_TEXT "${_move_3p}\n${_move_fp}")
set(_move_sum "${_fx_move}/build/native_check_summaries/tidy.json")
run_case(b29_move_suppression EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx_move}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_move}/build")
expect_json(b29_move_suppression "${_move_sum}" 1
  suppression_counts clang-analyzer-cplusplus.Move)
expect_json_length(b29_move_suppression "${_move_sum}" 2
  records 0 diagnostics)
expect_json(b29_move_suppression "${_move_sum}" "${_move_3p}"
  records 0 diagnostics 0)
expect_json(b29_move_suppression "${_move_sum}" "${_move_fp}"
  records 0 diagnostics 1)
expect_json_length(b29_move_suppression "${_move_sum}" 1
  records 0 unsuppressed_diagnostics)
expect_json(b29_move_suppression "${_move_sum}" "${_move_fp}"
  records 0 unsuppressed_diagnostics 0)

# ---------------------------------------------------------------------------
# Produced-build sweeps: the real headers matrix, the diagnostic controls
# and the context control records on the configured build.

# test_standalone_header_sweep_compiles_first_party_headers — the produced
# reachability-scoped sweep is this row's native successor.
run_case(b29_headers EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind headers --build "${BUILD_DIR}")
set(_hdr_sum "${BUILD_DIR}/native_check_summaries/headers.json")
expect_json(b29_headers "${_hdr_sum}" "ok" tool_health)
expect_json(b29_headers "${_hdr_sum}" "none" finding_status)
expect_json_nonzero(b29_headers "${_hdr_sum}" header_count)

# test_diagnostic_controls_cover_warning_families_and_move
run_case(b29_diag_controls EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind diagnostic-controls --build "${BUILD_DIR}")
set(_dc_sum "${BUILD_DIR}/native_check_summaries/diagnostic-controls.json")
expect_json(b29_diag_controls "${_dc_sum}" "ok" tool_health)
expect_json_item_names(b29_diag_controls "${_dc_sum}"
  PATH controls
  EXPECT bitwise_instead_of_logical discarded_nodiscard extra_semicolon
         first_party_use_after_move libcpp_probe_rejects_missing_mode
         signed_bounds)
# The move control is pushed fifth; guard the index before drilling in.
expect_json(b29_diag_controls "${_dc_sum}" "first_party_use_after_move"
  controls 4 name)
expect_json_contains(b29_diag_controls "${_dc_sum}" "cplusplus.Move"
  controls 4 matched_diagnostics)
expect_json_length(b29_diag_controls "${_dc_sum}" 0
  controls 4 suppressed_diagnostics)

# test_context_control_records_sdk_hardening_and_timing_evidence — the
# produced context sweep records the same sdk_controls/hardening_evidence
# surface; the oracle's common_controls half is covered by the selection
# fixtures above (stale manifest, duplicate TU, unexpected TU, target
# context and sysroot/compiler drift cases).
run_case(b29_context_control EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}")
set(_ctx_sum "${BUILD_DIR}/native_check_summaries/context.json")
expect_json(b29_context_control "${_ctx_sum}" "ok" tool_health)
if(_t56_system STREQUAL "Darwin")
  expect_json(b29_context_control "${_ctx_sum}" "passed"
    sdk_controls explicit_sdk)
  expect_json(b29_context_control "${_ctx_sum}" "rejected"
    sdk_controls missing_sdk)
  expect_json(b29_context_control "${_ctx_sum}" "passed"
    sdk_controls default_sdk)
  expect_json_member_set(b29_context_control "${_ctx_sum}"
    MEMBER build_type PATH hardening_evidence EXPECT Debug Release default)
  expect_json_length(b29_context_control "${_ctx_sum}" 3 hardening_evidence)
  foreach(_i RANGE 0 2)
    expect_json_equal(b29_context_control "${_ctx_sum}"
      hardening_evidence ${_i} sizeof EXTENSIVE
      = hardening_evidence ${_i} sizeof FAST)
    # timings_ns.<mode> is the per-mode timing array — the oracle's truthy
    # check maps to a non-empty array here.
    file(READ "${_ctx_sum}" _ctx_doc)
    foreach(_mode EXTENSIVE FAST)
      string(JSON _tcount ERROR_VARIABLE _terr
        LENGTH "${_ctx_doc}" hardening_evidence ${_i} timings_ns ${_mode})
      if(_terr OR _tcount LESS 1)
        set(_failures
          "${_failures}\n  b29_context_control: hardening_evidence[${_i}].timings_ns.${_mode} empty")
      endif()
    endforeach()
    expect_json(b29_context_control "${_ctx_sum}" "ON"
      hardening_evidence ${_i} assertion_active EXTENSIVE)
    expect_json(b29_context_control "${_ctx_sum}" "SIGTRAP"
      hardening_evidence ${_i} assertion_evidence EXTENSIVE signal)
    expect_json(b29_context_control "${_ctx_sum}"
      "libc++ std::vector element access"
      hardening_evidence ${_i} assertion_evidence EXTENSIVE category)
  endforeach()
else()
  foreach(_pair "no_sdk passed" "bad_sysroot rejected"
          "rejected_osx_sysroot rejected" "no_apple_tool passed"
          "sysroot_preservation passed" "stdlib_hardening passed")
    string(REPLACE " " ";" _kv "${_pair}")
    list(GET _kv 0 _ctl)
    list(GET _kv 1 _want)
    expect_json(b29_context_control "${_ctx_sum}" "${_want}"
      sdk_controls ${_ctl})
  endforeach()
  file(READ "${_ctx_sum}" _ctx_doc)
  string(JSON _he_count ERROR_VARIABLE _he_err LENGTH "${_ctx_doc}"
    hardening_evidence)
  if(_he_err OR _he_count LESS 1)
    set(_failures
      "${_failures}\n  b29_context_control: missing libstdc++ hardening evidence")
  else()
    math(EXPR _he_last "${_he_count} - 1")
    foreach(_i RANGE 0 ${_he_last})
      expect_json(b29_context_control "${_ctx_sum}" "libstdc++"
        hardening_evidence ${_i} stdlib)
      expect_json(b29_context_control "${_ctx_sum}" "ASSERTIONS"
        hardening_evidence ${_i} mode)
      expect_json(b29_context_control "${_ctx_sum}" "ON"
        hardening_evidence ${_i} assertion_active ASSERTIONS)
    endforeach()
  endif()
endif()

# ---------------------------------------------------------------------------
# Structural contract checks on scratch repositories
# (test_native_contract_checks.py::_write_contract_fixture port).

# b29_contract_fixture(<name>) writes the oracle's scratch repository +
# build pair: a bindings-style schema-1 context, source manifest, compile
# database, CTest registration, a fake native_contract_tests executable,
# one planted extension and a matching loader-provenance record. The
# optional NODEFF flag writes the post-mutation shape of the hardening
# test (no _LIBCPP hardening definition anywhere). Exports
# <name>_REPO/<name>_BUILD/<name>_EXT/<name>_DEF (canonical paths).
function(b29_contract_fixture name)
  cmake_parse_arguments(F "NODEFF" "" "" ${ARGN})
  set(_base "${SCRATCH_DIR}/${name}")
  set(_repo "${_base}/repo")
  set(_build "${_base}/build")
  set(_sdk "${_base}/sdk")
  file(MAKE_DIRECTORY "${_repo}/native/src" "${_repo}/native/tests"
    "${_repo}/tests/reference" "${_build}" "${_sdk}")
  file(WRITE "${_repo}/native/src/one.cpp"
    "int native_source() { return 1; }\n")
  file(WRITE "${_repo}/native/tests/test_contracts.cpp"
    "int contract_source() { return 2; }\n")
  file(WRITE "${_repo}/native/analysis_suppressions.json"
    "{\"schema_version\":1,\"suppressions\":[]}\n")
  file(WRITE "${_repo}/native/.clang-tidy"
    "Checks: 'bugprone-*,clang-analyzer-cplusplus.Move'\n"
    "HeaderFilterRegex: '(^|/)(native/(src|tests)|tools/proto_native_bench)/'\n")
  file(WRITE "${_repo}/tests/reference/test_native_probe.py"
    "from native_loader import load_native\nbike_native = load_native()\n")
  file(REAL_PATH "${_repo}" _repo_real)
  file(REAL_PATH "${_build}" _build_real)
  file(REAL_PATH "${_sdk}" _sdk_real)
  if(F_NODEFF)
    set(_hard "")
    set(_defs "[]")
    set(_def_arg "")
  else()
    set(_hard "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE")
    set(_defs "[\"${_hard}\"]")
    set(_def_arg "\"-D${_hard}\",")
  endif()
  set(_cc "${CMAKE_COMMAND}")
  set(_tc_json "")
  foreach(_t bike_native native_contract_tests native_distance_tests
          native_station_tests native_bench native_platform_blas_tests
          native_platform_lapack_tests)
    if(_t MATCHES "platform")
      set(_edges "\"Bike::Numerics\"")
    elseif(_t STREQUAL "native_bench")
      set(_edges "\"Bike::MuJoCo\"")
    else()
      set(_edges "\"Bike::MuJoCo\",\"Bike::Numerics\"")
    endif()
    string(APPEND _tc_json
      "\"${_t}\":{\"configuration\":\"fixture\",\"cxx_standard\":23,"
      "\"cxx_extensions\":false,\"include_directories\":[],"
      "\"compile_definitions\":${_defs},\"compile_options\":[],"
      "\"link_directories\":[],\"link_libraries\":[${_edges}]},")
  endforeach()
  string(REGEX REPLACE ",$" "" _tc_json "${_tc_json}")
  file(WRITE "${_build}/native_check_context.json"
    "{\"schema_version\":1,"
    "\"compiler\":{\"path\":\"${_cc}\",\"id\":\"fixture\",\"version\":\"1\","
    "\"launcher\":[]},"
    "\"sdk\":\"${_sdk_real}\",\"sdk_requested\":\"${_sdk_real}\","
    "\"sdk_source\":\"explicit\","
    "\"libcpp_hardening\":{\"supported\":true,\"mode\":\"EXTENSIVE\","
    "\"requested_mode\":\"EXTENSIVE\"},"
    "\"analysis_suppressions_manifest\":"
    "\"${_repo_real}/native/analysis_suppressions.json\","
    "\"include_paths\":[],\"library_paths\":[],\"required_files\":[],"
    "\"source_roots\":[\"${_repo_real}/native\"],"
    "\"python_executable\":\"${_cc}\","
    "\"tool_defaults\":{},\"tool_overrides\":{},"
    "\"target_contexts\":{${_tc_json}}}\n")
  file(WRITE "${_build}/native_sources.json"
    "{\"schema_version\":1,\"first_party_roots\":[\"${_repo_real}/native\"],"
    "\"sources\":["
    "{\"path\":\"${_repo_real}/native/src/one.cpp\","
    "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"},"
    "{\"path\":\"${_repo_real}/native/tests/test_contracts.cpp\","
    "\"target\":\"native_contract_tests\","
    "\"context\":\"native_contract_tests:fixture\"}]}\n")
  # Single-argument sets: multiple arguments would store ;-lists and corrupt
  # the compile_commands array on interpolation.
  set(_e1 "{\"directory\":\"${_base}\",\"file\":\"${_repo_real}/native/src/one.cpp\",\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\",\"arguments\":[\"${_cc}\",\"-std=c++23\",\"-isysroot\",\"${_sdk_real}\",${_def_arg}\"-c\",\"${_repo_real}/native/src/one.cpp\",\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}")
  set(_e2 "{\"directory\":\"${_base}\",\"file\":\"${_repo_real}/native/tests/test_contracts.cpp\",\"output\":\"CMakeFiles/native_contract_tests.dir/test_contracts.cpp.o\",\"arguments\":[\"${_cc}\",\"-std=c++23\",\"-isysroot\",\"${_sdk_real}\",${_def_arg}\"-c\",\"${_repo_real}/native/tests/test_contracts.cpp\",\"-o\",\"CMakeFiles/native_contract_tests.dir/test_contracts.cpp.o\"]}")
  file(WRITE "${_build}/compile_commands.json" "[${_e1},${_e2}]\n")
  file(WRITE "${_build}/CMakeCache.txt" "NATIVE_COVERAGE:BOOL=OFF\n")
  file(WRITE "${_build}/CTestTestfile.cmake"
    "add_test([=[native_contract_tests]=] \"native_contract_tests\")\n"
    "add_test([=[native_distance_tests]=] \"native_distance_tests\")\n"
    "add_test([=[native_station_tests]=] \"native_station_tests\")\n"
    "add_test([=[native_platform_blas_tests]=] \"native_platform_blas_tests\")\n"
    "add_test([=[native_platform_lapack_tests]=] "
    "\"native_platform_lapack_tests\")\n"
    "add_test([=[native_bench_bare]=] \"native_bench\" \"bare\" \"20\" \"5\")\n"
    "add_test([=[native_bench_glue]=] \"native_bench\" \"glue\" \"10\" \"4\")\n"
    "add_test([=[native_bench_heavy]=] \"native_bench\" \"heavy\" \"6\" \"3\")\n")
  file(WRITE "${_build}/native_contract_tests"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--list\" ]; then "
    "printf 'human_crank_torque\\npedaling_policy_valid_transition\\n'; fi\n"
    "exit 0\n")
  file(CHMOD "${_build}/native_contract_tests"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
  file(WRITE "${_build}/bike_native.fake.so"
    "fixture extension; checker must not import it\n")
  file(WRITE "${_build}/native_test_provenance.json"
    "{\"schema_version\":1,\"selected_build\":\"${_build_real}\","
    "\"imported_extension\":\"${_build_real}/bike_native.fake.so\","
    "\"native_test_count\":1}\n")
  set(${name}_REPO "${_repo_real}" PARENT_SCOPE)
  set(${name}_BUILD "${_build_real}" PARENT_SCOPE)
  set(${name}_SDK "${_sdk_real}" PARENT_SCOPE)
  set(${name}_EXT "${_build_real}/bike_native.fake.so" PARENT_SCOPE)
  set(${name}_DEF "${_hard}" PARENT_SCOPE)
endfunction()

# b29_contract_run(<case> EXPECT_EXIT <rc> [EXPECT_STDERR <needle>] NAME <fx>)
# Runs the contract checker on one fixture with the oracle's environment
# override pair set.
function(b29_contract_run case)
  cmake_parse_arguments(R "" "EXPECT_EXIT;EXPECT_STDERR;NAME" "" ${ARGN})
  set(fx "${R_NAME}")
  if(NOT DEFINED R_EXPECT_EXIT)
    set(R_EXPECT_EXIT 0)
  endif()
  if(DEFINED R_EXPECT_STDERR)
    set(_stderr EXPECT_STDERR "${R_EXPECT_STDERR}")
  else()
    set(_stderr "")
  endif()
  run_case(${case} EXPECT_EXIT ${R_EXPECT_EXIT} ${_stderr}
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "NATIVE_TEST_BUILD_PATH=${${fx}_BUILD}"
          "NATIVE_TEST_PROVENANCE_PATH=${${fx}_BUILD}/native_test_provenance.json"
          "${CHECKS_BINARY}" --kind contracts
          --build "${${fx}_BUILD}" --repo-root "${${fx}_REPO}")
  # run_case's failure aggregation lands in this frame — re-export it.
  set(_failures "${_failures}" PARENT_SCOPE)
endfunction()

# Positive control — the clean fixture must pass with the future rules
# still pending, and the loader provenance must name the selected build
# and artifact exactly (the coverage-provenance row's native half).
b29_contract_fixture(b29fx-base)
b29_contract_run(b29_contract_base EXPECT_EXIT 0
  EXPECT_STDERR "native contract checks passed" NAME b29fx-base)
set(_base_sum "${b29fx-base_BUILD}/native_check_summaries/contracts.json")
expect_json(b29_contract_base "${_base_sum}" "passed" status)
expect_json(b29_contract_base "${_base_sum}" "pending"
  checks engine_call_wrappers status)
expect_json(b29_contract_base "${_base_sum}" "pending"
  checks benchmark_manifest_membership status)
expect_json(b29_contract_base "${_base_sum}" "passed"
  checks loader_provenance status)
expect_json(b29_contract_base "${_base_sum}" "${b29fx-base_BUILD}"
  checks loader_provenance provenance selected_build)
expect_json(b29_contract_base "${_base_sum}" "${b29fx-base_EXT}"
  checks loader_provenance provenance imported_extension)
expect_json(b29_contract_base "${_base_sum}" 2
  checks manifest_selection source_count)
expect_json(b29_contract_base "${_base_sum}" 2
  checks manifest_selection translation_unit_context_count)
expect_json(b29_contract_base "${_base_sum}" "passed"
  checks contract_source_manifest_membership status)
expect_json(b29_contract_base "${_base_sum}" "passed"
  checks ctest_contract_cases status)
expect_json_length(b29_contract_base "${_base_sum}" 2
  checks ctest_contract_cases cases)
expect_json(b29_contract_base "${_base_sum}" "human_crank_torque"
  checks ctest_contract_cases cases 0)
expect_json(b29_contract_base "${_base_sum}" "pedaling_policy_valid_transition"
  checks ctest_contract_cases cases 1)

# Zero or two planted extensions fail closed on the bindings context
# (the oracle's extension-count params).
b29_contract_fixture(b29fx-extzero)
file(REMOVE "${b29fx-extzero_BUILD}/bike_native.fake.so")
b29_contract_run(b29_ext_zero EXPECT_EXIT FAIL
  EXPECT_STDERR "expected exactly one bike_native extension" NAME b29fx-extzero)

b29_contract_fixture(b29fx-exttwo)
file(WRITE "${b29fx-exttwo_BUILD}/bike_native.second.so"
  "second planted extension\n")
b29_contract_run(b29_ext_two EXPECT_EXIT FAIL
  EXPECT_STDERR "expected exactly one bike_native extension" NAME b29fx-exttwo)

# A provenance naming a different imported artifact fails closed.
b29_contract_fixture(b29fx-imported)
file(MAKE_DIRECTORY "${SCRATCH_DIR}/b29fx-imported/wrong-build")
file(WRITE "${SCRATCH_DIR}/b29fx-imported/wrong-build/bike_native.wrong.so"
  "wrong extension; never import\n")
file(REAL_PATH "${SCRATCH_DIR}/b29fx-imported/wrong-build/bike_native.wrong.so"
  _wrong_ext)
file(WRITE "${b29fx-imported_BUILD}/native_test_provenance.json"
  "{\"schema_version\":1,\"selected_build\":\"${b29fx-imported_BUILD}\","
  "\"imported_extension\":\"${_wrong_ext}\",\"native_test_count\":1}\n")
b29_contract_run(b29_prov_imported EXPECT_EXIT FAIL
  EXPECT_STDERR "imported extension does not match selected artifact"
  NAME b29fx-imported)

# A provenance whose native_test_count is not a nonnegative integer fails
# closed (the oracle's missing-count case; the C++ record treats the field
# as optional-but-validated, so the fixture plants an invalid value).
b29_contract_fixture(b29fx-count)
file(WRITE "${b29fx-count_BUILD}/native_test_provenance.json"
  "{\"schema_version\":1,\"selected_build\":\"${b29fx-count_BUILD}\","
  "\"imported_extension\":\"${b29fx-count_EXT}\","
  "\"native_test_count\":\"unknown\"}\n")
b29_contract_run(b29_prov_count EXPECT_EXIT FAIL
  EXPECT_STDERR "invalid native_test_count" NAME b29fx-count)

# A CTest registration file without the contract case fails closed.
b29_contract_fixture(b29fx-ctest)
file(WRITE "${b29fx-ctest_BUILD}/CTestTestfile.cmake"
  "# no contract case registered\n")
b29_contract_run(b29_ctest_registration EXPECT_EXIT FAIL
  EXPECT_STDERR "native_contract_tests is not registered with CTest"
  NAME b29fx-ctest)

# An executable that lists no cases fails closed.
b29_contract_fixture(b29fx-registry)
file(WRITE "${b29fx-registry_BUILD}/native_contract_tests"
  "#!/bin/sh\nexit 0\n")
file(CHMOD "${b29fx-registry_BUILD}/native_contract_tests"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
b29_contract_run(b29_empty_registry EXPECT_EXIT FAIL
  EXPECT_STDERR "lists no cases" NAME b29fx-registry)

# b29_contract_engine_files(<repo>) plants the three engine boundary files.
function(b29_contract_engine_files repo)
  file(WRITE "${repo}/native/src/engine_call.cpp"
    "void engine_operation() {}\n")
  file(WRITE "${repo}/native/src/engine_abi_312.hpp" "#pragma once\n")
  file(WRITE "${repo}/native/src/engine_call.hpp"
    "bool invoke();\nvoid* load_model();\nvoid* make_data();\n"
    "void forward();\nvoid step();\nvoid reset_data();\n"
    "void set_const();\nbool try_forward();\nbool try_step();\n")
endfunction()

# b29_contract_engine_manifest(<fx>) mirrors _install_engine_fixture:
# manifest entry, target context and compile-database row for the planted
# engine source under bike_native_engine.
function(b29_contract_engine_manifest fx)
  set(_repo "${${fx}_REPO}")
  set(_build "${${fx}_BUILD}")
  set(_engine "${_repo}/native/src/engine_call.cpp")
  set(_def "${${fx}_DEF}")
  file(READ "${_build}/native_sources.json" _doc)
  string(JSON _count LENGTH "${_doc}" sources)
  string(JSON _doc SET "${_doc}" sources ${_count}
    "{\"path\":\"${_engine}\",\"target\":\"bike_native_engine\",\"context\":\"bike_native_engine:fixture\"}")
  file(WRITE "${_build}/native_sources.json" "${_doc}")
  file(READ "${_build}/native_check_context.json" _ctx)
  string(JSON _bike_tc GET "${_ctx}" target_contexts bike_native)
  string(JSON _ctx SET "${_ctx}" target_contexts bike_native_engine
    "${_bike_tc}")
  file(WRITE "${_build}/native_check_context.json" "${_ctx}")
  file(READ "${_build}/compile_commands.json" _db)
  string(JSON _count LENGTH "${_db}")
  if(_def STREQUAL "")
    set(_def_arg "")
  else()
    set(_def_arg "\"-D${_def}\",")
  endif()
  string(JSON _db SET "${_db}" ${_count}
    "{\"directory\":\"${_build}\",\"file\":\"${_engine}\",\"output\":\"CMakeFiles/bike_native_engine.dir/engine_call.cpp.o\",\"arguments\":[\"${CMAKE_COMMAND}\",\"-std=c++23\",\"-isysroot\",\"${${fx}_SDK}\",${_def_arg}\"-c\",\"${_engine}\",\"-o\",\"CMakeFiles/bike_native_engine.dir/engine_call.cpp.o\"]}")
  file(WRITE "${_build}/compile_commands.json" "${_db}")
endfunction()

# Installed engine boundary: manifested wrappers pass; an unguarded
# fatal-capable call in another TU fails closed (both oracle params).
foreach(_call_item "forward mj_forward(nullptr, nullptr)"
        "loadmodel mj_loadModelBuffer(nullptr, 0)")
  string(REPLACE " " ";" _kv "${_call_item}")
  list(GET _kv 0 _call_name)
  list(GET _kv 1 _call_text)
  set(_fx_eng "b29fx-engine-${_call_name}")
  b29_contract_fixture(${_fx_eng})
  b29_contract_engine_files("${${_fx_eng}_REPO}")
  b29_contract_engine_manifest(${_fx_eng})
  b29_contract_run(b29_engine_${_call_name} EXPECT_EXIT 0 NAME ${_fx_eng})
  expect_json(b29_engine_${_call_name}
    "${${_fx_eng}_BUILD}/native_check_summaries/contracts.json" "passed"
    checks engine_call_wrappers status)
  file(WRITE "${${_fx_eng}_REPO}/native/src/one.cpp"
    "void unguarded() { ${_call_text}; }\n")
  b29_contract_run(b29_engine_unguarded_${_call_name} EXPECT_EXIT FAIL
    EXPECT_STDERR "unguarded fatal-capable MuJoCo calls" NAME ${_fx_eng})
endforeach()

# Engine boundary files without the manifest entry fail closed.
b29_contract_fixture(b29fx-engine-bare)
b29_contract_engine_files("${b29fx-engine-bare_REPO}")
b29_contract_run(b29_engine_unmanifested EXPECT_EXIT FAIL
  EXPECT_STDERR "engine_call.cpp is not uniquely manifested"
  NAME b29fx-engine-bare)

# The hardening-definition-free shape fails closed.
b29_contract_fixture(b29fx-nodef NODEFF)
b29_contract_run(b29_hardening_def EXPECT_EXIT FAIL
  EXPECT_STDERR "missing libc++ hardening definition" NAME b29fx-nodef)

# A tidy policy that disables the move checker fails closed.
b29_contract_fixture(b29fx-move)
file(WRITE "${b29fx-move_REPO}/native/.clang-tidy"
  "Checks: 'bugprone-*,-clang-analyzer-cplusplus.Move'\n"
  "HeaderFilterRegex: '(^|/)(native/(src|tests)|tools/proto_native_bench)/'\n")
b29_contract_run(b29_move_disabled EXPECT_EXIT FAIL
  EXPECT_STDERR "must enable first-party clang-analyzer-cplusplus.Move"
  NAME b29fx-move)

# A native test bypassing the selected-artifact loader fails closed.
b29_contract_fixture(b29fx-loader)
file(WRITE "${b29fx-loader_REPO}/tests/reference/test_native_probe.py"
  "import bike_native\n")
b29_contract_run(b29_loader_bypass EXPECT_EXIT FAIL
  EXPECT_STDERR "must use the selected-artifact loader" NAME b29fx-loader)

# One source manifested under a second target context is accepted and
# counted once per source.
b29_contract_fixture(b29fx-multictx)
file(READ "${b29fx-multictx_BUILD}/native_sources.json" _doc)
string(JSON _count LENGTH "${_doc}" sources)
string(JSON _doc SET "${_doc}" sources ${_count}
  "{\"path\":\"${b29fx-multictx_REPO}/native/src/one.cpp\",\"target\":\"native_analysis_probe\",\"context\":\"native_analysis_probe:fixture\"}")
file(WRITE "${b29fx-multictx_BUILD}/native_sources.json" "${_doc}")
file(READ "${b29fx-multictx_BUILD}/native_check_context.json" _ctx)
string(JSON _bike_tc GET "${_ctx}" target_contexts bike_native)
string(JSON _ctx SET "${_ctx}" target_contexts native_analysis_probe
  "${_bike_tc}")
file(WRITE "${b29fx-multictx_BUILD}/native_check_context.json" "${_ctx}")
file(READ "${b29fx-multictx_BUILD}/compile_commands.json" _db)
string(JSON _count LENGTH "${_db}")
string(JSON _db SET "${_db}" ${_count}
  "{\"directory\":\"${b29fx-multictx_BUILD}\",\"file\":\"${b29fx-multictx_REPO}/native/src/one.cpp\",\"output\":\"CMakeFiles/native_analysis_probe.dir/one.cpp.o\",\"arguments\":[\"${CMAKE_COMMAND}\",\"-std=c++23\",\"-isysroot\",\"${b29fx-multictx_SDK}\",\"-D${b29fx-multictx_DEF}\",\"-c\",\"${b29fx-multictx_REPO}/native/src/one.cpp\",\"-o\",\"CMakeFiles/native_analysis_probe.dir/one.cpp.o\"]}")
file(WRITE "${b29fx-multictx_BUILD}/compile_commands.json" "${_db}")
b29_contract_run(b29_multi_context EXPECT_EXIT 0 NAME b29fx-multictx)
set(_mc_sum "${b29fx-multictx_BUILD}/native_check_summaries/contracts.json")
expect_json(b29_multi_context "${_mc_sum}" 2
  checks manifest_selection source_count)
expect_json(b29_multi_context "${_mc_sum}" 3
  checks manifest_selection translation_unit_context_count)

# A manifested native_bench source under tools/proto_native_bench flips
# the pending rule to passed.
b29_contract_fixture(b29fx-bench)
set(_bench_root "${b29fx-bench_REPO}/tools/proto_native_bench")
file(MAKE_DIRECTORY "${_bench_root}")
file(WRITE "${_bench_root}/bench.cpp" "int benchmark_source() { return 3; }\n")
file(READ "${b29fx-bench_BUILD}/native_sources.json" _doc)
string(JSON _r_count LENGTH "${_doc}" first_party_roots)
string(JSON _doc SET "${_doc}" first_party_roots ${_r_count}
  "\"${_bench_root}\"")
string(JSON _count LENGTH "${_doc}" sources)
string(JSON _doc SET "${_doc}" sources ${_count}
  "{\"path\":\"${_bench_root}/bench.cpp\",\"target\":\"native_bench\",\"context\":\"native_bench:fixture\"}")
file(WRITE "${b29fx-bench_BUILD}/native_sources.json" "${_doc}")
file(READ "${b29fx-bench_BUILD}/native_check_context.json" _ctx)
string(JSON _r_count LENGTH "${_ctx}" source_roots)
string(JSON _ctx SET "${_ctx}" source_roots ${_r_count} "\"${_bench_root}\"")
string(JSON _bike_tc GET "${_ctx}" target_contexts bike_native)
string(JSON _ctx SET "${_ctx}" target_contexts native_bench "${_bike_tc}")
file(WRITE "${b29fx-bench_BUILD}/native_check_context.json" "${_ctx}")
file(READ "${b29fx-bench_BUILD}/compile_commands.json" _db)
string(JSON _count LENGTH "${_db}")
string(JSON _db SET "${_db}" ${_count}
  "{\"directory\":\"${b29fx-bench_BUILD}\",\"file\":\"${_bench_root}/bench.cpp\",\"output\":\"CMakeFiles/native_bench.dir/bench.cpp.o\",\"arguments\":[\"${CMAKE_COMMAND}\",\"-std=c++23\",\"-isysroot\",\"${b29fx-bench_SDK}\",\"-D${b29fx-bench_DEF}\",\"-c\",\"${_bench_root}/bench.cpp\",\"-o\",\"CMakeFiles/native_bench.dir/bench.cpp.o\"]}")
file(WRITE "${b29fx-bench_BUILD}/compile_commands.json" "${_db}")
b29_contract_run(b29_bench EXPECT_EXIT 0 NAME b29fx-bench)
expect_json(b29_bench
  "${b29fx-bench_BUILD}/native_check_summaries/contracts.json" "passed"
  checks benchmark_manifest_membership status)

# The repository's own folded .clang-tidy parses as a valid policy.
b29_contract_fixture(b29fx-folded)
file(COPY "${REPO_ROOT}/native/.clang-tidy"
  DESTINATION "${b29fx-folded_REPO}/native")
b29_contract_run(b29_folded_tidy EXPECT_EXIT 0 NAME b29fx-folded)

# ---------------------------------------------------------------------------
# Coverage orchestration rows.

# test_coverage_requires_profiles_from_this_run — the RunCoverage script
# wipes the profile directory before the test run, so a planted stale
# profraw cannot satisfy the run's profile glob; the script fails closed.
set(_cov_build "${SCRATCH_DIR}/b29-coverage-build")
file(MAKE_DIRECTORY "${_cov_build}/profiles")
file(WRITE "${_cov_build}/profiles/stale.profraw"
  "planted profile from a previous run\n")
file(WRITE "${_cov_build}/CTestTestfile.cmake"
  "add_test(trivial \"${CMAKE_COMMAND}\" -E true)\n")
find_program(_b29_ctest_exe NAMES ctest REQUIRED)
run_case(b29_run_coverage_empty EXPECT_EXIT FAIL
  EXPECT_STDERR "no raw coverage profiles"
  COMMAND "${CMAKE_COMMAND}"
        "-DBUILD_DIR=${_cov_build}" "-DCONFIG=Debug"
        "-DCTEST=${_b29_ctest_exe}" "-DLLVM_COV=unused"
        "-DLLVM_PROFDATA=unused"
        -P "${REPO_ROOT}/native/cmake/RunCoverage.cmake")

# test_real_one_file_llvm_coverage_report — a real instrumented compile,
# profile merge and llvm-cov report/export through the qualified
# toolchain; skipped where no toolchain qualifies (the oracle skips too).
string(JSON _cov_cap ERROR_VARIABLE _cov_cap_err
  GET "${_context_json}" capabilities coverage supported)
if(_cov_cap_err OR NOT _cov_cap)
  message(STATUS
    "b29_llvm_report: coverage unsupported on this toolchain — skipped")
else()
  get_filename_component(_cc_dir "${_b29_compiler}" DIRECTORY)
  find_program(_b29_llvm_profdata NAMES llvm-profdata HINTS "${_cc_dir}")
  find_program(_b29_llvm_cov NAMES llvm-cov HINTS "${_cc_dir}")
  if(_t56_system STREQUAL "Darwin")
    if(NOT _b29_llvm_profdata)
      execute_process(COMMAND xcrun --find llvm-profdata
        OUTPUT_VARIABLE _b29_llvm_profdata RESULT_VARIABLE _xrc
        OUTPUT_STRIP_TRAILING_WHITESPACE)
      if(NOT _xrc EQUAL 0)
        set(_b29_llvm_profdata "")
      endif()
    endif()
    if(NOT _b29_llvm_cov)
      execute_process(COMMAND xcrun --find llvm-cov
        OUTPUT_VARIABLE _b29_llvm_cov RESULT_VARIABLE _xrc2
        OUTPUT_STRIP_TRAILING_WHITESPACE)
      if(NOT _xrc2 EQUAL 0)
        set(_b29_llvm_cov "")
      endif()
    endif()
  endif()
  if(NOT _b29_llvm_profdata OR NOT _b29_llvm_cov)
    message(STATUS
      "b29_llvm_report: no qualified LLVM coverage toolchain — skipped")
  else()
    set(_cov "${SCRATCH_DIR}/b29-llvm-report")
    file(MAKE_DIRECTORY "${_cov}/profiles")
    file(WRITE "${_cov}/coverage_probe.cpp"
      "extern \"C\" int coverage_probe(int value) {\n"
      "  if (value > 0) return value + 1;\n"
      "  return -value;\n"
      "}\n")
    file(WRITE "${_cov}/coverage_driver.cpp"
      "#include <dlfcn.h>\n"
      "using CoverageFunction = int (*)(int);\n"
      "int main(int argc, char **argv) {\n"
      "  if (argc != 2) return 2;\n"
      "  void *handle = dlopen(argv[1], RTLD_NOW);\n"
      "  if (!handle) return 3;\n"
      "  auto function = reinterpret_cast<CoverageFunction>("
      "dlsym(handle, \"coverage_probe\"));\n"
      "  if (!function) return 4;\n"
      "  const int result = function(3);\n"
      "  dlclose(handle);\n"
      "  return result == 4 ? 0 : 5;\n"
      "}\n")
    set(_sdk_flags "")
    if(NOT _b29_sysroot STREQUAL "")
      list(APPEND _sdk_flags "-isysroot" "${_b29_sysroot}")
    endif()
    set(_dl_lib "")
    if(NOT _t56_system STREQUAL "Darwin")
      set(_dl_lib "-ldl")
    endif()
    execute_process(
      COMMAND "${_b29_compiler}" -std=c++23 ${_sdk_flags} -shared -fPIC
        -fprofile-instr-generate -fcoverage-mapping
        "${_cov}/coverage_probe.cpp" -o "${_cov}/bike_native.probe.so"
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e)
    if(NOT _rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_llvm_report: instrumented compile failed: ${_e}")
    else()
      execute_process(
        COMMAND "${_b29_compiler}" -std=c++23 ${_sdk_flags}
          "${_cov}/coverage_driver.cpp" -o "${_cov}/coverage_driver" ${_dl_lib}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e)
      if(NOT _rc EQUAL 0)
        set(_failures
          "${_failures}\n  b29_llvm_report: driver compile failed: ${_e}")
      endif()
    endif()
    if(EXISTS "${_cov}/coverage_driver")
      execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env
          "LLVM_PROFILE_FILE=${_cov}/profiles/%p.profraw"
          "${_cov}/coverage_driver" "${_cov}/bike_native.probe.so"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e)
      if(NOT _rc EQUAL 0)
        set(_failures
          "${_failures}\n  b29_llvm_report: driver run rc=${_rc} ${_e}")
      endif()
      file(GLOB _raw "${_cov}/profiles/*.profraw")
      if(NOT _raw)
        set(_failures
          "${_failures}\n  b29_llvm_report: no profraw produced")
      else()
        execute_process(
          COMMAND "${_b29_llvm_profdata}" merge -sparse ${_raw}
            -o "${_cov}/coverage.profdata"
          RESULT_VARIABLE _rc ERROR_VARIABLE _e)
        if(NOT _rc EQUAL 0)
          set(_failures
            "${_failures}\n  b29_llvm_report: profdata merge failed: ${_e}")
        else()
          execute_process(
            COMMAND "${_b29_llvm_cov}" report "${_cov}/bike_native.probe.so"
              "-instr-profile=${_cov}/coverage.profdata"
            RESULT_VARIABLE _rc OUTPUT_FILE "${_cov}/report.txt"
            ERROR_VARIABLE _e)
          if(NOT _rc EQUAL 0)
            set(_failures
              "${_failures}\n  b29_llvm_report: llvm-cov report failed: ${_e}")
          else()
            file(READ "${_cov}/report.txt" _report)
            if(NOT _report MATCHES "coverage_probe\\.cpp")
              set(_failures
                "${_failures}\n  b29_llvm_report: report lacks coverage_probe.cpp")
            endif()
            if(NOT _report MATCHES "TOTAL")
              set(_failures
                "${_failures}\n  b29_llvm_report: report lacks TOTAL")
            endif()
          endif()
          execute_process(
            COMMAND "${_b29_llvm_cov}" export "${_cov}/bike_native.probe.so"
              "-instr-profile=${_cov}/coverage.profdata"
            RESULT_VARIABLE _rc OUTPUT_FILE "${_cov}/export.json"
            ERROR_VARIABLE _e)
          if(NOT _rc EQUAL 0)
            set(_failures
              "${_failures}\n  b29_llvm_report: llvm-cov export failed: ${_e}")
          else()
            file(READ "${_cov}/export.json" _export)
            string(JSON _fc ERROR_VARIABLE _fe
              LENGTH "${_export}" data 0 files)
            if(_fe)
              set(_failures
                "${_failures}\n  b29_llvm_report: export has no files: ${_fe}")
            else()
              math(EXPR _flast "${_fc} - 1")
              set(_found FALSE)
              foreach(_i RANGE 0 ${_flast})
                string(JSON _fn GET "${_export}" data 0 files ${_i} filename)
                if(_fn MATCHES "coverage_probe\\.cpp$")
                  set(_found TRUE)
                  string(JSON _lc GET "${_export}" data 0 files ${_i}
                    summary lines covered)
                  string(JSON _lt GET "${_export}" data 0 files ${_i}
                    summary lines count)
                  string(JSON _bc GET "${_export}" data 0 files ${_i}
                    summary branches covered)
                  string(JSON _bt GET "${_export}" data 0 files ${_i}
                    summary branches count)
                  if(NOT (_lc GREATER 0 AND _lc LESS _lt AND
                          _bc GREATER 0 AND _bc LESS _bt))
                    set(_failures
                      "${_failures}\n  b29_llvm_report: coverage ${_lc}/${_lt} ${_bc}/${_bt} out of bounds")
                  endif()
                endif()
              endforeach()
              if(NOT _found)
                set(_failures
                  "${_failures}\n  b29_llvm_report: export lacks coverage_probe.cpp")
              endif()
            endif()
          endif()
        endif()
      endif()
    endif()
  endif()
endif()

# ===========================================================================
# Batch 3.29b2 — check_tools / contract_checks remainder (T53)
#
# Remaining ported rows: tool resolution/override contracts, schema-2
# platform-context validation (incl. the Linux no-SDK lane), capability
# record validation, RTSan/coverage configure-time qualification, the RTSan
# control pair, support-module parity and the coverage-provenance rows.
#
# Driver-capability probing below mirrors the reference helpers: candidate
# order (configured compiler, CXX, PATH spellings, known toolchains), the
# --version family classify, the -fsanitize=realtime accept probe and the
# CoverageTools resolution order (sibling -> PATH -> xcrun). Where no driver
# qualifies the cases report an explicit skip — never a silent pass.
# ===========================================================================

# expect_json_type(<name> <file> <TYPE> <member-path...>) — the JSON member
# at the path must exist with the given string(JSON TYPE) kind.
function(expect_json_type name file expected)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _t ERROR_VARIABLE _e TYPE "${_doc}" ${ARGN})
  if(_e)
    set(_failures "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_e}"
      PARENT_SCOPE)
  elseif(NOT _t STREQUAL expected)
    set(_failures
      "${_failures}\n  ${name}: json '${ARGN}' type is '${_t}', expected '${expected}'"
      PARENT_SCOPE)
  endif()
endfunction()

# expect_json_substring(<name> <file> <needle> <member-path...>) — the
# string member at the path must contain <needle>.
function(expect_json_substring name file needle)
  if(NOT EXISTS "${file}")
    set(_failures "${_failures}\n  ${name}: JSON document not found: ${file}"
      PARENT_SCOPE)
    return()
  endif()
  file(READ "${file}" _doc)
  string(JSON _val ERROR_VARIABLE _e GET "${_doc}" ${ARGN})
  if(_e)
    set(_failures "${_failures}\n  ${name}: json path '${ARGN}' failed: ${_e}"
      PARENT_SCOPE)
    return()
  endif()
  string(FIND "${_val}" "${needle}" _pos)
  if(_pos EQUAL -1)
    set(_failures
      "${_failures}\n  ${name}: json '${ARGN}' is '${_val}', missing '${needle}'"
      PARENT_SCOPE)
  endif()
endfunction()

# b29_tool_family(<path> <out-var>) — mirrors inspect_tool's banner classify
# (order: clang-tidy/cppcheck first so a plain "LLVM version" banner is not
# misread): returns clang, clang-tidy, cppcheck, gcc or "".
function(b29_tool_family path out_var)
  execute_process(COMMAND "${path}" --version
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 30)
  set(_banner "${_o}\n${_e}")
  string(STRIP "${_banner}" _banner)
  set(_family "")
  if(_rc EQUAL 0 AND NOT _banner STREQUAL "")
    if(_banner MATCHES "[Cc][Ll][Aa][Nn][Gg]-[Tt][Ii][Dd][Yy]|LLVM version")
      set(_family "clang-tidy")
    elseif(_banner MATCHES "[Cc][Pp][Pp][Cc][Hh][Ee][Cc][Kk]")
      set(_family "cppcheck")
    elseif(_banner MATCHES "[Cc][Ll][Aa][Nn][Gg]")
      set(_family "clang")
    elseif(_banner MATCHES "\\(GCC\\)|Free Software Foundation")
      set(_family "gcc")
    endif()
  endif()
  set(${out_var} "${_family}" PARENT_SCOPE)
endfunction()

# b29_driver_major(<path> <out-var>) — first dotted-version integer in the
# driver's --version banner (the oracle's _compiler_major_version).
function(b29_driver_major path out_var)
  execute_process(COMMAND "${path}" --version
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 30)
  set(_banner "${_o}\n${_e}")
  set(_major "")
  if(_rc EQUAL 0 AND _banner MATCHES "([0-9]+)\\.[0-9]+")
    set(_major "${CMAKE_MATCH_1}")
  endif()
  set(${out_var} "${_major}" PARENT_SCOPE)
endfunction()

# b29_driver_accepts_rtsan(<driver> <dir> <out-var>) — compile+link probe of
# the real lane flag, mirroring _driver_accepts_rtsan.
function(b29_driver_accepts_rtsan driver dir out_var)
  file(MAKE_DIRECTORY "${dir}")
  file(WRITE "${dir}/rtsan_accept_probe.cpp" "int main() { return 0; }\n")
  execute_process(COMMAND "${driver}" -std=c++23 -fsanitize=realtime
    "${dir}/rtsan_accept_probe.cpp" -o "${dir}/rtsan_accept_probe"
    RESULT_VARIABLE _rc OUTPUT_QUIET ERROR_QUIET TIMEOUT 120)
  if(_rc EQUAL 0)
    set(${out_var} TRUE PARENT_SCOPE)
  else()
    set(${out_var} FALSE PARENT_SCOPE)
  endif()
endfunction()

# b29_driver_accepts_coverage(<driver> <dir> <out-var>) — compile+link probe
# of the real coverage flags, mirroring the produced lane probe; a driver
# without the profile runtime library cannot qualify the lane.
function(b29_driver_accepts_coverage driver dir out_var)
  file(MAKE_DIRECTORY "${dir}")
  file(WRITE "${dir}/cov_accept_probe.cpp" "int main() { return 0; }\n")
  execute_process(COMMAND "${driver}" -std=c++23
    -fprofile-instr-generate -fcoverage-mapping
    "${dir}/cov_accept_probe.cpp" -o "${dir}/cov_accept_probe"
    RESULT_VARIABLE _rc OUTPUT_QUIET ERROR_QUIET TIMEOUT 120)
  if(_rc EQUAL 0)
    set(${out_var} TRUE PARENT_SCOPE)
  else()
    set(${out_var} FALSE PARENT_SCOPE)
  endif()
endfunction()

# Real C++ drivers to probe, most relevant first — mirrors
# _compiler_candidates: produced compiler, CXX, PATH spellings, known
# toolchain locations, then versioned distro spellings, deduped by real path.
set(_b29_candidate_names "${_b29_compiler}" "$ENV{CXX}" "c++" "clang++" "g++"
  "/usr/bin/clang++" "/opt/homebrew/opt/llvm/bin/clang++"
  "/usr/local/opt/llvm/bin/clang++")
foreach(_dir /usr/bin /usr/local/bin /opt/homebrew/bin)
  if(IS_DIRECTORY "${_dir}")
    file(GLOB _clangs LIST_DIRECTORIES false "${_dir}/clang++-*")
    list(SORT _clangs ORDER DESCENDING)
    list(APPEND _b29_candidate_names ${_clangs})
    file(GLOB _gccs LIST_DIRECTORIES false "${_dir}/g++-*")
    list(SORT _gccs ORDER DESCENDING)
    list(APPEND _b29_candidate_names ${_gccs})
  endif()
endforeach()
set(_b29_candidates "")
foreach(_name IN LISTS _b29_candidate_names)
  if(_name STREQUAL "")
    continue()
  endif()
  set(_located "")
  if(IS_ABSOLUTE "${_name}")
    if(EXISTS "${_name}" AND NOT IS_DIRECTORY "${_name}")
      set(_located "${_name}")
    endif()
  else()
    find_program(_located NAMES "${_name}" NO_CACHE)
  endif()
  if(NOT _located)
    continue()
  endif()
  # Dedup on the invoked path, not REAL_PATH: clang resolves symlinks to
  # the same binary under both 'clang' and 'clang++' spellings, and the
  # driver is argv[0]-sensitive — 'clang' does not link the C++ stdlib.
  list(FIND _b29_candidates "${_located}" _seen)
  if(_seen EQUAL -1)
    list(APPEND _b29_candidates "${_located}")
  endif()
endforeach()

# b29_cov_describe(<candidate> <major> <out-var>) — mirror of
# _bike_coverage_describe: resolved tool with an LLVM banner whose major
# matches the selected compiler's.
function(b29_cov_describe candidate major out_var)
  set(_located "")
  if(IS_ABSOLUTE "${candidate}")
    if(EXISTS "${candidate}" AND NOT IS_DIRECTORY "${candidate}")
      set(_located "${candidate}")
    endif()
  else()
    find_program(_located NAMES "${candidate}" NO_CACHE)
  endif()
  set(_found "")
  if(_located)
    execute_process(COMMAND "${_located}" --version
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 30)
    set(_banner "${_o}\n${_e}")
    if(_rc EQUAL 0 AND _banner MATCHES "[Ll][Ll][Vv][Mm]|clang"
        AND NOT major STREQUAL "")
      string(REGEX MATCH "([0-9]+)\\.[0-9]+" _v "${_banner}")
      if(NOT _v STREQUAL "" AND CMAKE_MATCH_1 STREQUAL "${major}")
        set(_found "${_located}")
      endif()
    endif()
  endif()
  set(${out_var} "${_found}" PARENT_SCOPE)
endfunction()

# b29_cov_tool_resolve(<tool> <compiler> <major> <out-var>) — mirror of
# _bike_resolve_coverage_tool's discovery tiers (no env override — the
# oracle's capability probe resolves with empty overrides).
function(b29_cov_tool_resolve tool compiler major out_var)
  get_filename_component(_dir "${compiler}" DIRECTORY)
  get_filename_component(_cname "${compiler}" NAME)
  set(_names "")
  if(_cname MATCHES "^((.*-)?)(clang\\+\\+|clang)(-([0-9]+(\\.[0-9]+)*))?$")
    set(_prefix "${CMAKE_MATCH_1}")
    set(_suffix "${CMAKE_MATCH_5}")
    if(NOT _suffix STREQUAL "")
      list(APPEND _names "${tool}-${_suffix}" "${_prefix}${tool}-${_suffix}")
    endif()
    list(APPEND _names "${tool}" "${_prefix}${tool}")
  endif()
  list(REMOVE_DUPLICATES _names)
  set(_found "")
  foreach(_n IN LISTS _names)
    b29_cov_describe("${_dir}/${_n}" "${major}" _found)
    if(_found)
      break()
    endif()
  endforeach()
  if(NOT _found AND NOT major STREQUAL "")
    b29_cov_describe("${tool}-${major}" "${major}" _found)
  endif()
  if(NOT _found)
    b29_cov_describe("${tool}" "${major}" _found)
  endif()
  if(NOT _found AND _t56_system STREQUAL "Darwin")
    find_program(_xcrun NAMES xcrun NO_CACHE)
    if(_xcrun)
      execute_process(COMMAND "${_xcrun}" --find "${tool}"
        RESULT_VARIABLE _xrc OUTPUT_VARIABLE _located ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE TIMEOUT 30)
      if(_xrc EQUAL 0)
        b29_cov_describe("${_located}" "${major}" _found)
      endif()
    endif()
  endif()
  set(${out_var} "${_found}" PARENT_SCOPE)
endfunction()

# b29_flag_reject_driver(<path> <real> <flag> <label>) — the oracle's
# flag-rejecting compiler fixture: forwards everything except one flag.
function(b29_flag_reject_driver path real flag label)
  file(WRITE "${path}"
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  case \"$arg\" in\n"
    "    *${flag}*)\n"
    "      echo \"${label}-c++: unsupported argument ${flag}\" >&2\n"
    "      exit 1 ;;\n"
    "  esac\n"
    "done\n"
    "exec \"${real}\" \"$@\"\n")
  file(CHMOD "${path}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endfunction()

# b29_shadow_rtsan_driver(<path> <real>) — forwards everything except rtsan
# -print-file-name queries, echoed back unresolved (oracle's
# _write_runtime_shadowing_driver).
function(b29_shadow_rtsan_driver path real)
  file(WRITE "${path}"
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  case \"$arg\" in\n"
    "    -print-file-name=*rtsan*)\n"
    "      echo \"\${arg#-print-file-name=}\"\n"
    "      exit 0 ;;\n"
    "  esac\n"
    "done\n"
    "exec \"${real}\" \"$@\"\n")
  file(CHMOD "${path}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endfunction()

# b29_broken_profdata(<path> <major>) — resolved-but-incompatible
# llvm-profdata stand-in (oracle's _write_broken_profdata): matching LLVM
# major so it resolves, then every merge fails rc=7 naming $0.
function(b29_broken_profdata path major)
  file(WRITE "${path}"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--version\" ]; then\n"
    "  printf 'broken LLVM version ${major}.0.0\\n'\n"
    "  exit 0\n"
    "fi\n"
    "if [ \"\${1:-}\" = \"merge\" ]; then\n"
    "  printf '%s: incompatible profile format\\n' \"$0\" >&2\n"
    "  exit 7\n"
    "fi\n"
    "exit 5\n")
  file(CHMOD "${path}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endfunction()

# b29_fake_llvm_tools(<dir> <major>) — the oracle's _write_fake_llvm_tools:
# banners report the compiler's LLVM major so the pair resolves; merge
# writes a nonempty profile; export emits a valid llvm.coverage export.
function(b29_fake_llvm_tools dir major)
  file(MAKE_DIRECTORY "${dir}")
  file(WRITE "${dir}/fake-llvm-profdata"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--version\" ]; then\n"
    "  printf 'fake LLVM version ${major}.0.0\\n'\n"
    "  exit 0\n"
    "fi\n"
    "if [ \"\${1:-}\" = \"merge\" ]; then\n"
    "  shift\n"
    "  output=''\n"
    "  while [ \"$#\" -gt 0 ]; do\n"
    "    if [ \"$1\" = \"-o\" ]; then shift; output=\"$1\"; fi\n"
    "    shift\n"
    "  done\n"
    "  [ -n \"$output\" ] || exit 4\n"
    "  printf 'merged profile\\n' > \"$output\"\n"
    "  exit 0\n"
    "fi\n"
    "exit 5\n")
  file(CHMOD "${dir}/fake-llvm-profdata"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
  file(WRITE "${dir}/fake-llvm-cov"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--version\" ]; then\n"
    "  printf 'fake LLVM version ${major}.0.0\\n'\n"
    "  exit 0\n"
    "fi\n"
    "if [ \"\${1:-}\" = \"report\" ]; then\n"
    "  printf 'Filename Regions Missed Lines Cover\\n'\n"
    "  printf '%s 2 0 100%%\\n' \"$2\"\n"
    "  printf 'TOTAL 2 0 100%%\\n'\n"
    "  exit 0\n"
    "fi\n"
    "if [ \"\${1:-}\" = \"export\" ]; then\n"
    "  printf '%s\\n' '{\"type\":\"llvm.coverage.json.export\",\"version\":\"${major}.0.0\",\"data\":[{\"files\":[{\"filename\":\"control.cpp\",\"summary\":{\"lines\":{\"count\":4,\"covered\":3}}}]}]}'\n"
    "  exit 0\n"
    "fi\n"
    "exit 5\n")
  file(CHMOD "${dir}/fake-llvm-cov"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endfunction()

# Scratch-configure the real native graph (the produced configure under
# test): every case runs the same project the oracle configured.
function(b29_scratch_configure name builddir)
  cmake_parse_arguments(C "" "EXPECT_EXIT" "ARGS;ENV;NEEDLES" ${ARGN})
  set(_options "-DCMAKE_BUILD_TYPE=Release"
    "-DPython_EXECUTABLE=${_b29_python}")
  if(_t56_system STREQUAL "Darwin" AND NOT _b29_sysroot STREQUAL "")
    list(APPEND _options "-DCMAKE_OSX_SYSROOT=${_b29_sysroot}")
  endif()
  list(APPEND _options ${C_ARGS})
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env ${C_ENV} "${CMAKE_COMMAND}"
      -S "${REPO_ROOT}/native" -B "${builddir}" ${_options}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 1200)
  set(_combined "${_o}\n${_e}")
  if(C_EXPECT_EXIT STREQUAL "FAIL")
    if(_rc EQUAL 0)
      set(_failures "${_failures}\n  ${name}: configure unexpectedly passed"
        PARENT_SCOPE)
    endif()
  elseif(NOT _rc EQUAL ${C_EXPECT_EXIT})
    set(_failures
      "${_failures}\n  ${name}: expected configure exit ${C_EXPECT_EXIT}, got ${_rc}: ${_combined}"
      PARENT_SCOPE)
  endif()
  foreach(_needle IN LISTS C_NEEDLES)
    string(FIND "${_combined}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      set(_failures
        "${_failures}\n  ${name}: configure output missing '${_needle}': ${_combined}"
        PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

# ---------------------------------------------------------------------------
# Tool selection contracts (fixture manifest + compile database).

# A deliberately nonexistent required override fails at resolution — before
# an empty sweep could be labeled passing (the oracle's five-kind matrix).
foreach(_pair "tidy CLANG_TIDY" "analyzer ANALYZER_CLANG" "cppcheck CPPCHECK"
        "frontends GXX" "odr GXX")
  string(REPLACE " " ";" _kv "${_pair}")
  list(GET _kv 0 _kind)
  list(GET _kv 1 _env)
  set(_missing "${SCRATCH_DIR}/missing-${_kind}-override")
  run_case(b29_override_${_kind} EXPECT_EXIT FAIL
    EXPECT_STDERR "invalid ${_kind} override"
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "${_env}=${_missing}"
          "${CHECKS_BINARY}" --kind ${_kind} --build "${_fx}/build")
  expect_json(b29_override_${_kind}
    "${_fx}/build/native_check_summaries/${_kind}.json" "failed" status)
  expect_json_contains(b29_override_${_kind}
    "${_fx}/build/native_check_summaries/${_kind}.json" "override" errors)
  expect_json(b29_override_${_kind}
    "${_fx}/build/native_check_summaries/${_kind}.json" 0
    translation_unit_count)
endforeach()

# A real executable from the wrong toolchain family is a selection error,
# not a passing sweep: the configured clang++ is invoked as clang-tidy and
# its process record reports the health failure.
run_case(b29_wrong_family EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_b29_compiler}"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
set(_wf_sum "${_fx}/build/native_check_summaries/tidy.json")
expect_json(b29_wrong_family "${_wf_sum}" "failed" tool_health)
expect_json(b29_wrong_family "${_wf_sum}" "failed" status)
expect_json_nonzero(b29_wrong_family "${_wf_sum}" records 0 returncode)

# The configured compiler's own toolchain sibling wins over the recorded
# default — tidy resolves the clang-tidy-18 sitting next to clang++-18, not
# the unrelated default spelling.
set(_fx_sib "${SCRATCH_DIR}/b29-sibling-tidy")
file(MAKE_DIRECTORY "${_fx_sib}/build" "${_fx_sib}/llvm-18/bin")
b29_mock_tool("${_fx_sib}/llvm-18/bin/clang++-18")
b29_mock_tool("${_fx_sib}/llvm-18/bin/clang-tidy-18")
b29_mock_tool("${_fx_sib}/unrelated-clang-tidy")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
string(JSON _c SET "${_c}" compiler path
  "\"${_fx_sib}/llvm-18/bin/clang++-18\"")
string(JSON _c SET "${_c}" compiler id "\"Clang\"")
string(JSON _c SET "${_c}" tool_defaults tidy
  "\"${_fx_sib}/unrelated-clang-tidy\"")
file(WRITE "${_fx_sib}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_sib}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],"
  "\"sources\":[{\"path\":\"${_fx}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_sib}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\",\"arguments\":["
  "\"${_fx_sib}/llvm-18/bin/clang++-18\",${_b29_drv_args_json},"
  "\"-c\",\"${_fx}/native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_tidy_sibling EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind tidy --build "${_fx_sib}/build")
expect_json(b29_tidy_sibling
  "${_fx_sib}/build/native_check_summaries/tidy.json"
  "${_fx_sib}/llvm-18/bin/clang-tidy-18" tool)

# The GNU secondary frontend is the configured compiler's own spelling —
# the sweep resolves g++-13 (the compiler itself), not the unrelated
# recorded gxx default.
set(_fx_gxx "${SCRATCH_DIR}/b29-sibling-gxx")
file(MAKE_DIRECTORY "${_fx_gxx}/build" "${_fx_gxx}/gcc-13/bin")
b29_mock_tool("${_fx_gxx}/gcc-13/bin/g++-13")
b29_mock_tool("${_fx_gxx}/unrelated-gxx")
set(_c "${_context_json}")
string(JSON _c SET "${_c}" source_roots "[\"${_fx}/native\"]")
string(JSON _c SET "${_c}" target_contexts "{\"bike_native\":${_b29_tc}}")
string(JSON _c SET "${_c}" analysis_suppressions_manifest
  "\"${_fx}/suppressions.json\"")
string(JSON _c SET "${_c}" compiler path "\"${_fx_gxx}/gcc-13/bin/g++-13\"")
string(JSON _c SET "${_c}" compiler id "\"GNU\"")
string(JSON _c SET "${_c}" tool_defaults gxx "\"${_fx_gxx}/unrelated-gxx\"")
file(WRITE "${_fx_gxx}/build/native_check_context.json" "${_c}")
file(WRITE "${_fx_gxx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],"
  "\"sources\":[{\"path\":\"${_fx}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
file(WRITE "${_fx_gxx}/build/compile_commands.json"
  "[{\"directory\":\"${_fx}\",\"file\":\"${_fx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\",\"arguments\":["
  "\"${_fx_gxx}/gcc-13/bin/g++-13\",${_b29_drv_args_json},"
  "\"-c\",\"${_fx}/native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_gxx_sibling EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind frontends --build "${_fx_gxx}/build")
expect_json(b29_gxx_sibling
  "${_fx_gxx}/build/native_check_summaries/frontends.json"
  "${_fx_gxx}/gcc-13/bin/g++-13" tool)

# ---------------------------------------------------------------------------
# Schema-2 platform context — the produced context on the real host.
# _ctx is reused as JSON scratch by the contract-fixture mutations above; a
# dedicated variable keeps the path spelling stable here.
set(_b29_ctx "${BUILD_DIR}/native_check_context.json")

# test_schema2_context_loads_on_matching_host: the emitted context records
# this host's system, the platform-correct stdlib, and the SDK/sysroot pair
# (equal on Darwin, absent on Linux).
expect_json(b29_schema2_host "${_b29_ctx}" "${_t56_system}" platform system)
if(_t56_system STREQUAL "Darwin")
  expect_json(b29_schema2_host "${_b29_ctx}" "libc++" stdlib name)
  expect_json_type(b29_schema2_host "${_b29_ctx}" STRING sysroot)
  expect_json_equal(b29_schema2_host "${_b29_ctx}" sdk = sysroot)
else()
  # The oracle asserts membership, not a fixed name: the produced Linux
  # lane is libstdc++ today, but either supported stdlib is honest.
  file(READ "${_b29_ctx}" _host_doc)
  string(JSON _host_stdlib ERROR_VARIABLE _host_stdlib_err
    GET "${_host_doc}" stdlib name)
  if(_host_stdlib_err OR
      (NOT _host_stdlib STREQUAL "libc++" AND
       NOT _host_stdlib STREQUAL "libstdc++"))
    set(_failures
      "${_failures}\n  b29_schema2_host: stdlib.name '${_host_stdlib}' is not libc++/libstdc++")
  endif()
  expect_json_type(b29_schema2_host "${_b29_ctx}" NULL sdk)
  expect_json_type(b29_schema2_host "${_b29_ctx}" NULL sysroot)
endif()

# test_schema1_context_remains_supported: schema 1 keeps the Darwin-shaped
# SDK contract. On Darwin the produced context already carries it; on Linux
# a fixture SDK directory carries the same records the legacy schema
# requires — the produced validator accepts either.
set(_s1 "${_s2}")
string(JSON _s1 SET "${_s1}" schema_version "1")
if(NOT _t56_system STREQUAL "Darwin")
  file(MAKE_DIRECTORY "${SCRATCH_DIR}/schema1-sdk")
  file(REAL_PATH "${SCRATCH_DIR}/schema1-sdk" _s1_sdk)
  string(JSON _s1 SET "${_s1}" sdk "\"${_s1_sdk}\"")
  string(JSON _s1 SET "${_s1}" sdk_requested "\"${_s1_sdk}\"")
  string(JSON _s1 SET "${_s1}" sdk_source "\"explicit\"")
endif()
t56_ctx_case(ctx-b29-schema1 "${_s1}")
run_case(b29_schema1_supported EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_linux_context_loads_and_records_no_sdk — Linux-only: the
# produced Linux context records the no-SDK shape and libstdc++.
if(NOT _t56_system STREQUAL "Darwin")
  expect_json_type(b29_schema2_linux_loads "${_b29_ctx}" NULL sdk)
  expect_json_type(b29_schema2_linux_loads "${_b29_ctx}" NULL sdk_requested)
  expect_json(b29_schema2_linux_loads "${_b29_ctx}" "not-applicable" sdk_source)
  expect_json_type(b29_schema2_linux_loads "${_b29_ctx}" NULL sysroot)
  expect_json(b29_schema2_linux_loads "${_b29_ctx}" "libstdc++" stdlib name)
  expect_json(b29_schema2_linux_loads "${_b29_ctx}" "ASSERTIONS"
    stdlib hardening mode)
endif()

# test_schema2_platform_host_mismatch_fails: a context claiming the other
# system is rejected against the real host (both directions — the Darwin
# half already exists as t56_ctx_schema2_foreign_system).
if(_t56_system STREQUAL "Darwin")
  set(_foreign "Linux")
else()
  set(_foreign "Darwin")
endif()
string(JSON _bad SET "${_s2}" platform system "\"${_foreign}\"")
string(JSON _bad SET "${_bad}" platform host_system "\"${_foreign}\"")
t56_ctx_case(ctx-b29-foreign-system "${_bad}")
run_case(b29_schema2_foreign_system EXPECT_EXIT FAIL
  EXPECT_STDERR "on a '${_t56_system}' host"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_darwin_sysroot_drift_fails — Darwin-only: a sysroot that
# exists but is not the selected SDK fails the pairing.
if(_t56_system STREQUAL "Darwin")
  file(MAKE_DIRECTORY "${SCRATCH_DIR}/drifted-sdk")
  file(REAL_PATH "${SCRATCH_DIR}/drifted-sdk" _drifted)
  string(JSON _bad SET "${_s2}" sysroot "\"${_drifted}\"")
  t56_ctx_case(ctx-b29-sysroot-drift "${_bad}")
  run_case(b29_schema2_sysroot_drift EXPECT_EXIT FAIL
    EXPECT_STDERR "does not equal the selected SDK"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
          --validate-only)
endif()

# test_schema2_stdlib_mode_mismatch_fails — Linux-only: a hardening mode the
# produced stdlib does not carry is rejected (ASSERTIONS is libstdc++-only,
# so the invalid spelling is selected from the produced record's name).
if(NOT _t56_system STREQUAL "Darwin")
  file(READ "${_b29_ctx}" _sm_doc)
  string(JSON _sm_stdlib GET "${_sm_doc}" stdlib name)
  if(_sm_stdlib STREQUAL "libstdc++")
    set(_sm_mode "FAST")
  else()
    set(_sm_mode "ASSERTIONS")
  endif()
  string(JSON _bad SET "${_s2}" stdlib hardening mode "\"${_sm_mode}\"")
  t56_ctx_case(ctx-b29-stdlib-mode "${_bad}")
  run_case(b29_schema2_stdlib_mode EXPECT_EXIT FAIL
    EXPECT_STDERR "stdlib hardening mode '${_sm_mode}' is invalid for ${_sm_stdlib}"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
          --validate-only)
endif()

# test_schema2_linux_context_must_not_record_sdk — Linux-only: even an
# existing directory is a recorded Apple SDK the Linux context rejects.
if(NOT _t56_system STREQUAL "Darwin")
  string(JSON _bad SET "${_s2}" sdk "\"${SCRATCH_DIR}\"")
  t56_ctx_case(ctx-b29-linux-sdk "${_bad}")
  run_case(b29_schema2_linux_sdk EXPECT_EXIT FAIL
    EXPECT_STDERR "Linux native check context must not record an Apple SDK"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
          --validate-only)
endif()

# test_schema2_missing_numerics_fails — the whole numerics record, not just
# its backend (the backend half is t56_ctx_schema2_no_numerics).
string(JSON _bad REMOVE "${_s2}" numerics)
t56_ctx_case(ctx-b29-no-numerics "${_bad}")
run_case(b29_schema2_no_numerics EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has no numerics record"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_linux_invalid_library_path_fails — both oracle params: an
# absent path and a plain (non-.framework) directory are invalid
# library_paths entries.
string(JSON _bad SET "${_s2}" library_paths
  "[\"${SCRATCH_DIR}/missing-libprovider.so\"]")
t56_ctx_case(ctx-b29-library-missing "${_bad}")
run_case(b29_library_path_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "missing context library_paths entry"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)
file(MAKE_DIRECTORY "${SCRATCH_DIR}/libprovider-dir")
file(REAL_PATH "${SCRATCH_DIR}/libprovider-dir" _lib_dir)
string(JSON _bad SET "${_s2}" library_paths "[\"${_lib_dir}\"]")
t56_ctx_case(ctx-b29-library-dir "${_bad}")
run_case(b29_library_path_dir EXPECT_EXIT FAIL
  EXPECT_STDERR "context library_paths entry is neither a library file nor a framework directory"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# ---------------------------------------------------------------------------
# Schema-2 capability records — sanitizer runtime, identity, target and
# request consistency. The asan record carries the oracle's cases verbatim.

# test_schema2_enabled_sanitizer_runtime_must_be_recorded — an enabled lane
# whose runtime is null could never be preloaded.
string(JSON _bad SET "${_s2}" capabilities asan supported "true")
string(JSON _bad SET "${_bad}" capabilities asan requested "true")
string(JSON _bad SET "${_bad}" capabilities asan enabled "true")
string(JSON _bad SET "${_bad}" capabilities asan runtime "null")
t56_ctx_case(ctx-b29-asan-runtime-null "${_bad}")
run_case(b29_schema2_asan_runtime_null EXPECT_EXIT FAIL
  EXPECT_STDERR "is enabled without a recorded sanitizer runtime"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_sanitizer_runtime_must_exist — an enabled lane's runtime
# record must resolve to a real file.
string(JSON _bad SET "${_s2}" capabilities asan supported "true")
string(JSON _bad SET "${_bad}" capabilities asan requested "true")
string(JSON _bad SET "${_bad}" capabilities asan enabled "true")
string(JSON _bad SET "${_bad}" capabilities asan runtime
  "\"${SCRATCH_DIR}/absent/libasan.so\"")
t56_ctx_case(ctx-b29-asan-runtime-absent "${_bad}")
run_case(b29_schema2_asan_runtime_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "missing capability asan runtime"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_sanitizer_runtime_must_be_absolute — a relative spelling
# resolves nowhere the runner could preload.
string(JSON _bad SET "${_s2}" capabilities asan supported "true")
string(JSON _bad SET "${_bad}" capabilities asan requested "true")
string(JSON _bad SET "${_bad}" capabilities asan enabled "true")
string(JSON _bad SET "${_bad}" capabilities asan runtime "\"libasan.so\"")
t56_ctx_case(ctx-b29-asan-runtime-relative "${_bad}")
run_case(b29_schema2_asan_runtime_relative EXPECT_EXIT FAIL
  EXPECT_STDERR "missing capability asan runtime"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_sanitizer_runtime_must_be_a_file — a directory is not a
# runtime image.
file(MAKE_DIRECTORY "${SCRATCH_DIR}/libasan-dir.so")
file(REAL_PATH "${SCRATCH_DIR}/libasan-dir.so" _asan_dir)
string(JSON _bad SET "${_s2}" capabilities asan supported "true")
string(JSON _bad SET "${_bad}" capabilities asan requested "true")
string(JSON _bad SET "${_bad}" capabilities asan enabled "true")
string(JSON _bad SET "${_bad}" capabilities asan runtime "\"${_asan_dir}\"")
t56_ctx_case(ctx-b29-asan-runtime-dir "${_bad}")
run_case(b29_schema2_asan_runtime_dir EXPECT_EXIT FAIL
  EXPECT_STDERR "capability asan runtime is not a file"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_enabled_sanitizer_records_a_real_runtime — the green path:
# an enabled lane with an existing runtime file loads.
file(WRITE "${SCRATCH_DIR}/libasan-fixture.so" "runtime image\n")
file(REAL_PATH "${SCRATCH_DIR}/libasan-fixture.so" _asan_rt)
string(JSON _good SET "${_s2}" capabilities asan supported "true")
string(JSON _good SET "${_good}" capabilities asan requested "true")
string(JSON _good SET "${_good}" capabilities asan enabled "true")
string(JSON _good SET "${_good}" capabilities asan runtime "\"${_asan_rt}\"")
t56_ctx_case(ctx-b29-asan-enabled "${_good}")
run_case(b29_schema2_asan_enabled EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)
expect_json(b29_schema2_asan_enabled
  "${_fixture_dir}/native_check_context.json" "ON"
  capabilities asan enabled)
expect_json(b29_schema2_asan_enabled
  "${_fixture_dir}/native_check_context.json" "${_asan_rt}"
  capabilities asan runtime)

# test_schema2_enabled_sanitizer_requires_requested — enabled without a
# requested lane is inconsistent.
string(JSON _bad SET "${_s2}" capabilities asan supported "true")
string(JSON _bad SET "${_bad}" capabilities asan requested "false")
string(JSON _bad SET "${_bad}" capabilities asan enabled "true")
string(JSON _bad SET "${_bad}" capabilities asan runtime "\"${_asan_rt}\"")
t56_ctx_case(ctx-b29-asan-not-requested "${_bad}")
run_case(b29_schema2_asan_not_requested EXPECT_EXIT FAIL
  EXPECT_STDERR "is enabled without a requested lane"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_sanitizer_compiler_identity_required — the recorded compiler
# identity selects the runtime; a record without it is meaningless.
string(JSON _bad REMOVE "${_s2}" capabilities asan compiler_id)
t56_ctx_case(ctx-b29-asan-no-compiler-id "${_bad}")
run_case(b29_schema2_asan_no_compiler_id EXPECT_EXIT FAIL
  EXPECT_STDERR "has no compiler identity record"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# test_schema2_sanitizer_target_may_be_null_or_triple — only a triple
# string or null is a valid target record (the null half is exercised by
# every passing context).
string(JSON _bad SET "${_s2}" capabilities asan target "42")
t56_ctx_case(ctx-b29-asan-bad-target "${_bad}")
run_case(b29_schema2_asan_bad_target EXPECT_EXIT FAIL
  EXPECT_STDERR "target must be a triple string or null"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}"
        --validate-only)

# ---------------------------------------------------------------------------
# Schema-2 Linux compile-database controls — the same fail-closed checks
# under the real no-SDK context. The produced loader runs the identical
# selection path on both platforms, so the platform-invariant diagnostics
# run unconditionally; the sysroot/SDK-shaped cases stay Linux-gated like
# their oracle records.

set(_lx "${SCRATCH_DIR}/b29-linux-db")
file(MAKE_DIRECTORY "${_lx}/native/src" "${_lx}/build")
file(WRITE "${_lx}/native/src/one.cpp" "int first_party() { return 0; }\n")
file(WRITE "${_lx}/native/src/two.cpp" "int second_party() { return 0; }\n")
file(WRITE "${_lx}/suppressions.json"
  "{\"schema_version\":1,\"suppressions\":[]}\n")
b29_fixture_context("${_lx}/build" "${_lx}/native"
  "{\"bike_native\":${_b29_tc}}" "${_lx}/suppressions.json")
file(WRITE "${_lx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_lx}/native\"],"
  "\"sources\":[{\"path\":\"${_lx}/native/src/one.cpp\","
  "\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}]}\n")
b29_ccdb_entry(_lx_one "${_lx}" "${_lx}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o")
file(WRITE "${_lx}/build/compile_commands.json" "[${_lx_one}]\n")

# b29_linux_case(<case> <file-name> ...) — clone the linux fixture's
# context/manifest into a per-case build dir before mutating the database.
function(b29_linux_case case dirname)
  set(_dir "${SCRATCH_DIR}/${dirname}")
  file(MAKE_DIRECTORY "${_dir}")
  file(COPY "${_lx}/build/native_check_context.json"
            "${_lx}/build/native_sources.json"
       DESTINATION "${_dir}")
  set(${case} "${_dir}" PARENT_SCOPE)
endfunction()

# test_schema2_linux_stale_compiler_fails — argv[0] is not the configured
# compiler.
b29_linux_case(_lx_stale "b29-linux-stale")
file(WRITE "${_lx_stale}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\",\"arguments\":["
  "\"/bin/sh\",${_b29_drv_args_json},\"-c\",\"${_lx}/native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_linux_stale_compiler EXPECT_EXIT FAIL
  EXPECT_STDERR "compiler differs from configured C++ compiler"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_stale}")

# test_schema2_linux_duplicate_translation_unit_fails — the produced check
# is already dual-platform via b29_dup_tu; this record documents the
# Linux-shaped selection exercising the same rejection.
b29_linux_case(_lx_dup "b29-linux-dup")
file(WRITE "${_lx_dup}/compile_commands.json" "[${_lx_one},${_lx_one}]\n")
run_case(b29_linux_dup_tu EXPECT_EXIT FAIL
  EXPECT_STDERR "duplicate identical translation unit"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_dup}")

# test_schema2_linux_missing_translation_unit_fails — a manifested source
# absent from the database fails selection by name.
b29_linux_case(_lx_miss "b29-linux-missing")
file(WRITE "${_lx_miss}/compile_commands.json" "[${_lx_one}]\n")
file(READ "${_lx_miss}/native_sources.json" _doc)
string(JSON _doc SET "${_doc}" sources 1
  "{\"path\":\"${_lx}/native/src/two.cpp\",\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}")
file(WRITE "${_lx_miss}/native_sources.json" "${_doc}")
run_case(b29_linux_missing_tu EXPECT_EXIT FAIL
  EXPECT_STDERR "missing expected translation units: two.cpp"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_miss}")

# test_schema2_linux_wrong_target_fails — the -o attribution names a target
# the source manifest never selected.
b29_linux_case(_lx_target "b29-linux-target")
file(WRITE "${_lx_target}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/other_target.dir/one.cpp.o\",\"arguments\":["
  "${_b29_argv_json},\"-c\",\"${_lx}/native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/other_target.dir/one.cpp.o\"]}]\n")
run_case(b29_linux_wrong_target EXPECT_EXIT FAIL
  EXPECT_STDERR "unexpected target context"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_target}")

# test_schema2_linux_wrong_selected_flags_fail — both oracle params: the
# extensions-mode standard spelling and a missing required compile option.
b29_linux_case(_lx_gnu "b29-linux-gnu-std")
file(WRITE "${_lx_gnu}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\",\"arguments\":["
  "\"${_b29_compiler}\",\"-std=gnu++23\",${_b29_drv_args_json},"
  "\"-c\",\"${_lx}/native/src/one.cpp\","
  "\"-o\",\"CMakeFiles/bike_native.dir/one.cpp.o\"]}]\n")
run_case(b29_linux_gnu_std EXPECT_EXIT FAIL
  EXPECT_STDERR "is not strict ISO C++23"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_gnu}")

b29_linux_case(_lx_opt "b29-linux-omitted-opt")
file(READ "${_lx_opt}/native_check_context.json" _doc)
string(JSON _doc SET "${_doc}" target_contexts bike_native compile_options
  "[\"-ffp-contract=off\"]")
file(WRITE "${_lx_opt}/native_check_context.json" "${_doc}")
file(WRITE "${_lx_opt}/compile_commands.json" "[${_lx_one}]\n")
run_case(b29_linux_omitted_option EXPECT_EXIT FAIL
  EXPECT_STDERR "omits target bike_native compile option"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_opt}")

# test_schema2_linux_relative_dependency_path_fails — a relative -I
# resolves against the entry directory and must exist.
b29_linux_case(_lx_rel "b29-linux-rel-include")
b29_ccdb_entry(_lx_rel_e "${_lx}" "${_lx}/native/src/one.cpp"
  "CMakeFiles/bike_native.dir/one.cpp.o" "-I" "relative-include")
file(WRITE "${_lx_rel}/compile_commands.json" "[${_lx_rel_e}]\n")
run_case(b29_linux_rel_include EXPECT_EXIT FAIL
  EXPECT_STDERR "missing dependency path in compilation database entry 0"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_rel}")

# test_schema2_linux_malformed_command_fails — the three oracle params:
# non-array arguments, an unparseable command string, and neither member.
b29_linux_case(_lx_ba "b29-linux-bad-args")
file(WRITE "${_lx_ba}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"arguments\":\"not-a-list\"}]\n")
run_case(b29_linux_bad_args EXPECT_EXIT FAIL
  EXPECT_STDERR "malformed arguments"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_ba}")

b29_linux_case(_lx_bc "b29-linux-bad-cmd")
file(WRITE "${_lx_bc}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\","
  "\"command\":\"${_b29_compiler} 'unterminated\"}]\n")
run_case(b29_linux_bad_cmd EXPECT_EXIT FAIL
  EXPECT_STDERR "malformed command"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_bc}")

b29_linux_case(_lx_nc "b29-linux-no-cmd")
file(WRITE "${_lx_nc}/compile_commands.json"
  "[{\"directory\":\"${_lx}\",\"file\":\"${_lx}/native/src/one.cpp\","
  "\"output\":\"CMakeFiles/bike_native.dir/one.cpp.o\"}]\n")
run_case(b29_linux_no_cmd EXPECT_EXIT FAIL
  EXPECT_STDERR "needs command or arguments"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_nc}")

# test_schema2_linux_absent_compile_database_fails — a build without the
# database cannot select anything.
b29_linux_case(_lx_nodb "b29-linux-no-db")
run_case(b29_linux_no_db EXPECT_EXIT FAIL
  EXPECT_STDERR "missing compilation database"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/mock-tidy"
        "${CHECKS_BINARY}" --kind tidy --build "${_lx_nodb}")

# test_schema2_linux_entries_without_sysroot_validate — Linux-only: the
# produced no-SDK context selects entries carrying no -isysroot.
if(NOT _t56_system STREQUAL "Darwin")
  run_case(b29_linux_entries_no_sysroot EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "CLANG_TIDY=${_fx}/mock-tidy"
          "${CHECKS_BINARY}" --kind tidy --build "${_lx}/build")
  expect_json(b29_linux_entries_no_sysroot
    "${_lx}/build/native_check_summaries/tidy.json" 1
    translation_unit_count)
endif()

# test_schema2_linux_sysroot_argument_is_rejected — already covered on
# Linux by t56_ctx_linux_sysroot_arg (the produced -isysroot injection over
# the real compilation database above, run through --kind frontends).

# test_schema2_explicit_sysroot_directory_is_accepted — Linux-only: an
# existing sysroot recorded in the context and selected in the entry
# validates.
if(NOT _t56_system STREQUAL "Darwin")
  b29_linux_case(_lx_es "b29-linux-explicit-sysroot")
  file(MAKE_DIRECTORY "${_lx_es}/sysroot")
  file(REAL_PATH "${_lx_es}/sysroot" _lx_es_sysroot)
  file(READ "${_lx_es}/native_check_context.json" _doc)
  string(JSON _doc SET "${_doc}" sysroot "\"${_lx_es_sysroot}\"")
  file(WRITE "${_lx_es}/native_check_context.json" "${_doc}")
  b29_ccdb_entry(_lx_es_e "${_lx}" "${_lx}/native/src/one.cpp"
    "CMakeFiles/bike_native.dir/one.cpp.o"
    "-isysroot" "${_lx_es_sysroot}")
  file(WRITE "${_lx_es}/compile_commands.json" "[${_lx_es_e}]\n")
  run_case(b29_linux_explicit_sysroot EXPECT_EXIT 0
    COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
          "CLANG_TIDY=${_fx}/mock-tidy"
          "${CHECKS_BINARY}" --kind tidy --build "${_lx_es}")
  expect_json(b29_linux_explicit_sysroot
    "${_lx_es}/native_check_summaries/tidy.json" 1
    translation_unit_count)
endif()

# ---------------------------------------------------------------------------
# Support-module parity — the oracle's AST/regex comparison over the same
# two files, re-expressed in CMake so the C++ loader-rule exemption list
# cannot drift from the Python reference while both exist.
file(READ "${REPO_ROOT}/tools/check_native_contracts.py" _py_contracts)
string(REGEX MATCH "_SUPPORT_TEST_MODULES[ \t\n]*=[ \t\n]*\\{([^}]*)\\}"
  _sup_py_m "${_py_contracts}")
if(_sup_py_m STREQUAL "")
  set(_failures
    "${_failures}\n  b29_support_modules: no _SUPPORT_TEST_MODULES in tools/check_native_contracts.py")
else()
  string(REGEX MATCHALL "'[^']+'" _sup_py "${CMAKE_MATCH_1}")
  set(_sup_py_clean "")
  foreach(_n IN LISTS _sup_py)
    string(REPLACE "'" "" _n "${_n}")
    list(APPEND _sup_py_clean "${_n}")
  endforeach()
endif()
file(READ "${REPO_ROOT}/native/src/tools/check_contracts.cpp" _cc_contracts)
string(REGEX MATCH
  "std::array<std::string_view,[ \t]*([0-9]+)>[ \t\n]*kSupportTestModules[ \t\n]*\\{([^}]*)\\};"
  _sup_cc_m "${_cc_contracts}")
if(_sup_cc_m STREQUAL "")
  set(_failures
    "${_failures}\n  b29_support_modules: no kSupportTestModules in check_contracts.cpp")
else()
  set(_sup_cc_declared "${CMAKE_MATCH_1}")
  string(REGEX MATCHALL "\"[^\"]+\"" _sup_cc "${CMAKE_MATCH_2}")
  set(_sup_cc_clean "")
  foreach(_n IN LISTS _sup_cc)
    string(REPLACE "\"" "" _n "${_n}")
    list(APPEND _sup_cc_clean "${_n}")
  endforeach()
  list(LENGTH _sup_cc_clean _sup_cc_count)
  if(NOT _sup_cc_count EQUAL _sup_cc_declared)
    set(_failures
      "${_failures}\n  b29_support_modules: kSupportTestModules declares ${_sup_cc_declared} entries and lists ${_sup_cc_count}")
  endif()
endif()
if(DEFINED _sup_py_clean AND DEFINED _sup_cc_clean)
  list(SORT _sup_py_clean)
  list(SORT _sup_cc_clean)
  list(JOIN _sup_py_clean ";" _sup_py_joined)
  list(JOIN _sup_cc_clean ";" _sup_cc_joined)
  if(NOT _sup_cc_joined STREQUAL _sup_py_joined)
    set(_failures
      "${_failures}\n  b29_support_modules: C++ '${_sup_cc_joined}' != Python '${_sup_py_joined}'")
  endif()
endif()

# ---------------------------------------------------------------------------
# Coverage-provenance rows (test_native_contract_checks.py) — the produced
# provenance surface is the contracts checker's loader-provenance record
# and RunCoverage's fail-closed run.

# test_coverage_rejects_malformed_provenance — an unparseable provenance
# file fails at load, never mid-report.
b29_contract_fixture(b29fx-malformed)
file(WRITE "${b29fx-malformed_BUILD}/native_test_provenance.json"
  "{not json")
b29_contract_run(b29_prov_malformed EXPECT_EXIT FAIL
  EXPECT_STDERR "malformed native loader provenance" NAME b29fx-malformed)

# test_coverage_rejects_a_provenance_for_a_different_build — the recorded
# selected build must equal the requested build.
b29_contract_fixture(b29fx-wrong-build)
file(MAKE_DIRECTORY "${SCRATCH_DIR}/b29fx-wrong-build/other-build")
file(REAL_PATH "${SCRATCH_DIR}/b29fx-wrong-build/other-build" _other_build)
file(WRITE "${b29fx-wrong-build_BUILD}/native_test_provenance.json"
  "{\"schema_version\":1,\"selected_build\":\"${_other_build}\","
  "\"imported_extension\":\"${b29fx-wrong-build_EXT}\","
  "\"native_test_count\":1}\n")
b29_contract_run(b29_prov_wrong_build EXPECT_EXIT FAIL
  EXPECT_STDERR "selected build does not match" NAME b29fx-wrong-build)

# test_coverage_requires_zero_excluded_native_tests — the produced lane
# runs CTest with --no-tests=error under the exclusion labels; a run whose
# whole selection is excluded fails closed rather than publishing an empty
# report.
set(_cov_x "${SCRATCH_DIR}/b29-coverage-excluded")
file(MAKE_DIRECTORY "${_cov_x}")
file(WRITE "${_cov_x}/CTestTestfile.cmake"
  "add_test(excluded_one \"${CMAKE_COMMAND}\" -E true)\n"
  "set_tests_properties(excluded_one PROPERTIES LABELS \"coverage\")\n"
  "add_test(install_smoke \"${CMAKE_COMMAND}\" -E true)\n")
run_case(b29_run_coverage_no_tests EXPECT_EXIT FAIL
  EXPECT_STDERR "coverage CTest run failed"
  COMMAND "${CMAKE_COMMAND}"
        "-DBUILD_DIR=${_cov_x}" "-DCONFIG=Debug"
        "-DCTEST=${_b29_ctest_exe}" "-DLLVM_COV=unused"
        "-DLLVM_PROFDATA=unused"
        -P "${REPO_ROOT}/native/cmake/RunCoverage.cmake")

# ---------------------------------------------------------------------------
# RTSan qualification — real driver probes, then scratch configures against
# the produced policy. All gating is explicit: an absent toolchain prints a
# STATUS skip, never a fabricated pass.

# Driver probes. Only the family split matters for reachability: a GNU
# driver on Darwin never reaches the RTSan/coverage gates (earlier
# toolchain/stdlib checks fail it — the produced fail-closed layering is
# equivalent, so the oracle's Darwin GNU cases resolve to the same
# rejections only on the graph where GNU is qualified).
set(_b29_clang_driver "")
set(_b29_rtsan_capable "")
set(_b29_rtsan_lacking_clang "")
set(_b29_rtsan_lacking_other "")
set(_b29_probe_i 0)
foreach(_cand IN LISTS _b29_candidates)
  b29_tool_family("${_cand}" _fam)
  if(_fam STREQUAL "clang" AND _b29_clang_driver STREQUAL "")
    set(_b29_clang_driver "${_cand}")
  endif()
  b29_driver_accepts_rtsan("${_cand}"
    "${SCRATCH_DIR}/b29-probe-${_b29_probe_i}" _ok)
  if(_ok)
    if(_b29_rtsan_capable STREQUAL "")
      set(_b29_rtsan_capable "${_cand}")
    endif()
  elseif(_fam STREQUAL "clang" AND _b29_rtsan_lacking_clang STREQUAL "")
    set(_b29_rtsan_lacking_clang "${_cand}")
  elseif(_b29_rtsan_lacking_other STREQUAL "")
    set(_b29_rtsan_lacking_other "${_cand}")
  endif()
  math(EXPR _b29_probe_i "${_b29_probe_i} + 1")
endforeach()
# A lacking driver usable on this graph: clang-family everywhere; on Linux
# any family reaches the rtsan gate because GNU is a qualified toolchain.
set(_b29_rtsan_lacking "")
if(NOT _b29_rtsan_lacking_clang STREQUAL "")
  set(_b29_rtsan_lacking "${_b29_rtsan_lacking_clang}")
elseif(NOT _t56_system STREQUAL "Darwin")
  set(_b29_rtsan_lacking "${_b29_rtsan_lacking_other}")
endif()

# test_requested_rtsan_fails_on_a_flag_rejecting_compiler_fixture — C++23
# works but -fsanitize=realtime is rejected; a requested lane must fail
# naming the flag and the selected compiler.
b29_flag_reject_driver("${SCRATCH_DIR}/no-rtsan-c++" "${_b29_compiler}"
  "-fsanitize=realtime" "no-rtsan")
b29_scratch_configure(b29_rtsan_flag_reject
  "${SCRATCH_DIR}/b29-cfg-rtsan-flag-reject"
  EXPECT_EXIT FAIL
  ARGS "-DCMAKE_CXX_COMPILER=${SCRATCH_DIR}/no-rtsan-c++"
       "-DNATIVE_RTSAN=ON"
  NEEDLES "-fsanitize=realtime" "${SCRATCH_DIR}/no-rtsan-c++")

if(_b29_rtsan_lacking STREQUAL "")
  message(STATUS
    "b29_rtsan_lacking/b29_rtsan_unrequested: every candidate accepts -fsanitize=realtime — skipped")
else()
  # test_requested_rtsan_fails_on_a_known_lacking_compiler — the measured
  # negative on a real driver.
  b29_scratch_configure(b29_rtsan_lacking
    "${SCRATCH_DIR}/b29-cfg-rtsan-lacking"
    EXPECT_EXIT FAIL
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_rtsan_lacking}" "-DNATIVE_RTSAN=ON"
    NEEDLES "-fsanitize=realtime" "${_b29_rtsan_lacking}")

  # test_unrequested_rtsan_capability_is_explicitly_unsupported — an
  # unrequested lane records {requested:false, supported:false,
  # enabled:false, runtime:null} with probe evidence — never a silent skip.
  b29_scratch_configure(b29_rtsan_unrequested
    "${SCRATCH_DIR}/b29-cfg-rtsan-unrequested"
    EXPECT_EXIT 0
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_rtsan_lacking}")
  set(_ru_ctx "${SCRATCH_DIR}/b29-cfg-rtsan-unrequested/native_check_context.json")
  expect_json(b29_rtsan_unrequested "${_ru_ctx}" "OFF"
    capabilities rtsan requested)
  expect_json(b29_rtsan_unrequested "${_ru_ctx}" "OFF"
    capabilities rtsan supported)
  expect_json(b29_rtsan_unrequested "${_ru_ctx}" "OFF"
    capabilities rtsan enabled)
  expect_json_type(b29_rtsan_unrequested "${_ru_ctx}" NULL
    capabilities rtsan runtime)
  expect_json_equal(b29_rtsan_unrequested "${_ru_ctx}"
    capabilities rtsan compiler_id = compiler id)
  expect_json_substring(b29_rtsan_unrequested "${_ru_ctx}" "probe"
    capabilities rtsan evidence control)
endif()

if(_b29_rtsan_capable STREQUAL "")
  message(STATUS
    "b29_rtsan_no_runtime/b29_rtsan_qualified/b29_rtsan_control: no RTSan-capable compiler on this host — skipped")
else()
  # test_requested_rtsan_fails_when_the_runtime_is_unresolvable — the flag
  # probes pass but the runtime query echoes back unresolved: a setup
  # failure distinct from flag rejection.
  b29_shadow_rtsan_driver("${SCRATCH_DIR}/no-rtsan-runtime-c++"
    "${_b29_rtsan_capable}")
  b29_scratch_configure(b29_rtsan_no_runtime
    "${SCRATCH_DIR}/b29-cfg-rtsan-no-runtime"
    EXPECT_EXIT FAIL
    ARGS "-DCMAKE_CXX_COMPILER=${SCRATCH_DIR}/no-rtsan-runtime-c++"
         "-DNATIVE_RTSAN=ON"
    NEEDLES "rtsan" "${SCRATCH_DIR}/no-rtsan-runtime-c++")

  # test_qualified_rtsan_configure_enables_the_lane — the requested lane
  # records enabled with a resolved runtime and run/violation evidence,
  # and the built control behaves as contracted.
  b29_scratch_configure(b29_rtsan_qualified
    "${SCRATCH_DIR}/b29-cfg-rtsan-qualified"
    EXPECT_EXIT 0
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_rtsan_capable}" "-DNATIVE_RTSAN=ON")
  set(_rq_ctx "${SCRATCH_DIR}/b29-cfg-rtsan-qualified/native_check_context.json")
  expect_json(b29_rtsan_qualified "${_rq_ctx}" "ON"
    capabilities rtsan requested)
  expect_json(b29_rtsan_qualified "${_rq_ctx}" "ON"
    capabilities rtsan supported)
  expect_json(b29_rtsan_qualified "${_rq_ctx}" "ON"
    capabilities rtsan enabled)
  expect_json_equal(b29_rtsan_qualified "${_rq_ctx}"
    capabilities rtsan compiler_id = compiler id)
  expect_json_substring(b29_rtsan_qualified "${_rq_ctx}" "probe"
    capabilities rtsan evidence control)
  expect_json_substring(b29_rtsan_qualified "${_rq_ctx}" "control"
    capabilities rtsan evidence run_control)
  expect_json_substring(b29_rtsan_qualified "${_rq_ctx}" "control"
    capabilities rtsan evidence annotation_control)
  expect_json_substring(b29_rtsan_qualified "${_rq_ctx}" "control"
    capabilities rtsan evidence macro_control)
  expect_json_substring(b29_rtsan_qualified "${_rq_ctx}"
    "RealtimeSanitizer" capabilities rtsan evidence violation_control)
  if(EXISTS "${_rq_ctx}")
    file(READ "${_rq_ctx}" _rq_doc)
    string(JSON _rq_rt ERROR_VARIABLE _rq_rt_err
      GET "${_rq_doc}" capabilities rtsan runtime)
    if(_rq_rt_err OR NOT EXISTS "${_rq_rt}" OR IS_DIRECTORY "${_rq_rt}")
      set(_failures
        "${_failures}\n  b29_rtsan_qualified: runtime '${_rq_rt}' is not a file")
    elseif(NOT _rq_rt MATCHES "rtsan")
      set(_failures
        "${_failures}\n  b29_rtsan_qualified: runtime '${_rq_rt}' names no rtsan image")
    endif()
  endif()
  execute_process(COMMAND "${CMAKE_COMMAND}" --build
    "${SCRATCH_DIR}/b29-cfg-rtsan-qualified" --target native_rtsan_control
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 600)
  if(NOT _rc EQUAL 0)
    set(_failures
      "${_failures}\n  b29_rtsan_qualified: control build failed (${_rc}): ${_o}${_e}")
  else()
    set(_rq_bin "${SCRATCH_DIR}/b29-cfg-rtsan-qualified/native_rtsan_control")
    execute_process(COMMAND "${_rq_bin}"
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 60)
    if(NOT _rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_rtsan_qualified: clean control rc=${_rc} ${_e}")
    endif()
    execute_process(COMMAND "${_rq_bin}" forbidden-allocation
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 60)
    set(_v "${_o}\n${_e}")
    if(_rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_rtsan_qualified: violation control exited 0")
    else()
      foreach(_needle "RealtimeSanitizer" "malloc")
        string(FIND "${_v}" "${_needle}" _pos)
        if(_pos EQUAL -1)
          set(_failures
            "${_failures}\n  b29_rtsan_qualified: violation output missing '${_needle}': ${_v}")
        endif()
      endforeach()
    endif()
  endif()

  # test_rtsan_control_source_reports_the_sanitizer_diagnostic — the same
  # source compiled by hand: instrumented violation reports, the
  # uninstrumented twin exits clean (the annotation is load-bearing).
  set(_rc_dir "${SCRATCH_DIR}/b29-rtsan-control")
  file(MAKE_DIRECTORY "${_rc_dir}")
  execute_process(COMMAND "${_b29_rtsan_capable}" -std=c++23 -g -O0
    -fsanitize=realtime "${REPO_ROOT}/native/tests/rtsan_control.cpp"
    -o "${_rc_dir}/rtsan_control"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 300)
  if(NOT _rc EQUAL 0)
    set(_failures
      "${_failures}\n  b29_rtsan_control: instrumented compile failed: ${_e}")
  else()
    execute_process(COMMAND "${_rc_dir}/rtsan_control"
      RESULT_VARIABLE _rc ERROR_VARIABLE _e TIMEOUT 60)
    if(NOT _rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_rtsan_control: clean mode rc=${_rc} ${_e}")
    endif()
    execute_process(COMMAND "${_rc_dir}/rtsan_control" forbidden-allocation
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 60)
    set(_v "${_o}\n${_e}")
    if(_rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_rtsan_control: instrumented violation exited 0")
    else()
      foreach(_needle "RealtimeSanitizer" "malloc")
        string(FIND "${_v}" "${_needle}" _pos)
        if(_pos EQUAL -1)
          set(_failures
            "${_failures}\n  b29_rtsan_control: violation output missing '${_needle}': ${_v}")
        endif()
      endforeach()
    endif()
  endif()
  execute_process(COMMAND "${_b29_rtsan_capable}" -std=c++23 -g -O0
    "${REPO_ROOT}/native/tests/rtsan_control.cpp"
    -o "${_rc_dir}/rtsan_control_plain"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 300)
  if(NOT _rc EQUAL 0)
    set(_failures
      "${_failures}\n  b29_rtsan_control: plain compile failed: ${_e}")
  else()
    execute_process(COMMAND "${_rc_dir}/rtsan_control_plain"
      forbidden-allocation
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _e TIMEOUT 60)
    set(_v "${_o}\n${_e}")
    if(NOT _rc EQUAL 0)
      set(_failures
        "${_failures}\n  b29_rtsan_control: uninstrumented violation rc=${_rc} ${_v}")
    elseif(_v MATCHES "RealtimeSanitizer")
      set(_failures
        "${_failures}\n  b29_rtsan_control: uninstrumented run reported a sanitizer diagnostic: ${_v}")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# Coverage qualification — configure-time policy gates over real drivers.

# test_requested_coverage_fails_on_a_flag_rejecting_compiler_fixture —
# -fprofile-instr-generate is rejected; a requested lane must fail naming
# the flag and the selected compiler. The fake wraps a clang-family driver:
# only the clang spellings carry that flag (a GNU probe uses
# -fprofile-arcs/-ftest-coverage, which the oracle's fixture does not
# reject — its own candidate order already prefers the clang family).
if(_b29_clang_driver STREQUAL "")
  message(STATUS "b29_cov_flag_reject: no clang-family compiler on this host — skipped")
else()
  b29_flag_reject_driver("${SCRATCH_DIR}/no-coverage-c++" "${_b29_clang_driver}"
    "-fprofile-instr-generate" "no-coverage")
  b29_scratch_configure(b29_cov_flag_reject
    "${SCRATCH_DIR}/b29-cfg-cov-flag-reject"
    EXPECT_EXIT FAIL
    ARGS "-DCMAKE_CXX_COMPILER=${SCRATCH_DIR}/no-coverage-c++"
         "-DNATIVE_COVERAGE=ON"
    NEEDLES "-fprofile-instr-generate" "${SCRATCH_DIR}/no-coverage-c++")
endif()

# test_requested_coverage_fails_for_a_non_clang_compiler — a requested
# coverage lane against a GNU driver must fail the family gate. On Darwin a
# GNU driver never reaches that gate (the platform/stdlib layers reject it
# earlier — the fail-closed layering is equivalent, the oracle's toy
# fixture simply bypassed them), so the produced check is exercised on the
# real graph where GNU is the qualified toolchain.
set(_b29_gnu_driver "")
foreach(_cand IN LISTS _b29_candidates)
  b29_tool_family("${_cand}" _fam)
  if(_fam STREQUAL "gcc")
    set(_b29_gnu_driver "${_cand}")
    break()
  endif()
endforeach()
if(_t56_system STREQUAL "Darwin")
  message(STATUS
    "b29_cov_non_clang: GNU driver on Darwin is rejected before the coverage-family gate — skipped")
elseif(_b29_gnu_driver STREQUAL "")
  message(STATUS
    "b29_cov_non_clang: no GNU-family compiler on this host — skipped")
else()
  b29_scratch_configure(b29_cov_non_clang
    "${SCRATCH_DIR}/b29-cfg-cov-non-clang"
    EXPECT_EXIT FAIL
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_gnu_driver}" "-DNATIVE_COVERAGE=ON"
    NEEDLES "LLVM clang-family")
endif()

# Coverage-capable driver detection — mirror of the oracle's
# _coverage_capable_driver: first clang-family candidate whose
# llvm-profdata/llvm-cov pair resolves under the produced resolution order.
set(_b29_cov_driver "")
set(_b29_cov_major "")
set(_b29_cov_probe_i 0)
foreach(_cand IN LISTS _b29_candidates)
  b29_tool_family("${_cand}" _fam)
  if(NOT _fam STREQUAL "clang")
    continue()
  endif()
  b29_driver_major("${_cand}" _major)
  if(_major STREQUAL "")
    continue()
  endif()
  # The driver must actually compile+link an instrumented TU — resolved
  # tools alone do not qualify the lane when the profile runtime is absent.
  b29_driver_accepts_coverage("${_cand}"
    "${SCRATCH_DIR}/b29-cov-probe-${_b29_cov_probe_i}" _cov_ok)
  math(EXPR _b29_cov_probe_i "${_b29_cov_probe_i} + 1")
  if(NOT _cov_ok)
    continue()
  endif()
  b29_cov_tool_resolve("llvm-profdata" "${_cand}" "${_major}" _p)
  if(NOT _p)
    continue()
  endif()
  b29_cov_tool_resolve("llvm-cov" "${_cand}" "${_major}" _c)
  if(NOT _c)
    continue()
  endif()
  set(_b29_cov_driver "${_cand}")
  set(_b29_cov_major "${_major}")
  break()
endforeach()

if(_b29_cov_driver STREQUAL "")
  message(STATUS
    "b29_cov_bad_override/b29_cov_merge_fail/b29_cov_qualified/b29_cov_tool_evidence: no clang toolchain with qualified coverage tools on this host — skipped")
else()
  # test_requested_coverage_fails_on_an_unresolvable_tool_override — an
  # explicit LLVM_PROFDATA override that does not resolve is a hard
  # configure failure, never a silent fallthrough.
  b29_scratch_configure(b29_cov_bad_override
    "${SCRATCH_DIR}/b29-cfg-cov-bad-override"
    EXPECT_EXIT FAIL
    ENV ${_b29_clean_env} "LLVM_PROFDATA=${SCRATCH_DIR}/missing-llvm-profdata"
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_cov_driver}" "-DNATIVE_COVERAGE=ON"
    NEEDLES "LLVM_PROFDATA" "${SCRATCH_DIR}/missing-llvm-profdata")

  # test_coverage_failing_merge_tool_reports_command_and_log — a resolved
  # profdata that cannot merge is a configure-time qualification failure
  # naming the command, its status and the retained diagnostics directory
  # (the produced lane qualifies the merge at configure: the same
  # rejection, earlier).
  b29_broken_profdata("${SCRATCH_DIR}/broken-llvm-profdata" "${_b29_cov_major}")
  b29_scratch_configure(b29_cov_merge_fail
    "${SCRATCH_DIR}/b29-cfg-cov-merge-fail"
    EXPECT_EXIT FAIL
    ENV ${_b29_clean_env} --unset=LLVM_COV
        "LLVM_PROFDATA=${SCRATCH_DIR}/broken-llvm-profdata"
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_cov_driver}" "-DNATIVE_COVERAGE=ON"
    NEEDLES "control-merge" "(7)" "${SCRATCH_DIR}/broken-llvm-profdata"
            "diagnostics")
  set(_mf_probe
    "${SCRATCH_DIR}/b29-cfg-cov-merge-fail/native_check_work/compiler-policy/coverage-format")
  if(EXISTS "${_mf_probe}/control-merge.stderr.log")
    file(READ "${_mf_probe}/control-merge.stderr.log" _mf_log)
    string(FIND "${_mf_log}" "incompatible profile format" _pos)
    if(_pos EQUAL -1)
      set(_failures
        "${_failures}\n  b29_cov_merge_fail: retained merge log lacks the tool diagnostic: ${_mf_log}")
    endif()
  else()
    set(_failures
      "${_failures}\n  b29_cov_merge_fail: no retained control-merge.stderr.log")
  endif()
  if(NOT EXISTS "${_mf_probe}/control-merge.command.json")
    set(_failures
      "${_failures}\n  b29_cov_merge_fail: no retained control-merge.command.json")
  endif()

  # test_qualified_coverage_configure_records_tool_evidence +
  # test_coverage_records_tool_resolution_evidence — one qualified
  # configure with the oracle's env-resolved fake pair: the context records
  # enabled plus control/tools/format-probe evidence, and the emitted
  # coverage_tool_resolution.json names the resolved paths, versions and
  # the passed probe (the produced lane's tool-resolution provenance).
  b29_fake_llvm_tools("${SCRATCH_DIR}/fake-llvm" "${_b29_cov_major}")
  b29_scratch_configure(b29_cov_qualified
    "${SCRATCH_DIR}/b29-cfg-cov-qualified"
    EXPECT_EXIT 0
    ENV ${_b29_clean_env}
        "LLVM_PROFDATA=${SCRATCH_DIR}/fake-llvm/fake-llvm-profdata"
        "LLVM_COV=${SCRATCH_DIR}/fake-llvm/fake-llvm-cov"
    ARGS "-DCMAKE_CXX_COMPILER=${_b29_cov_driver}" "-DNATIVE_COVERAGE=ON")
  set(_cq_ctx "${SCRATCH_DIR}/b29-cfg-cov-qualified/native_check_context.json")
  expect_json(b29_cov_qualified "${_cq_ctx}" "ON"
    capabilities coverage requested)
  expect_json(b29_cov_qualified "${_cq_ctx}" "ON"
    capabilities coverage supported)
  expect_json(b29_cov_qualified "${_cq_ctx}" "ON"
    capabilities coverage enabled)
  expect_json_substring(b29_cov_qualified "${_cq_ctx}" "probe"
    capabilities coverage evidence control)
  expect_json_substring(b29_cov_qualified "${_cq_ctx}" "llvm-profdata"
    capabilities coverage evidence tools)
  expect_json_substring(b29_cov_qualified "${_cq_ctx}" "llvm-cov"
    capabilities coverage evidence tools)
  expect_json_substring(b29_cov_qualified "${_cq_ctx}" "export"
    capabilities coverage evidence format_probe)
  set(_cq_tr
    "${SCRATCH_DIR}/b29-cfg-cov-qualified/native_check_work/compiler-policy/coverage_tool_resolution.json")
  expect_json(b29_cov_tool_evidence "${_cq_tr}"
    "${SCRATCH_DIR}/fake-llvm/fake-llvm-profdata" llvm_profdata path)
  expect_json(b29_cov_tool_evidence "${_cq_tr}"
    "${SCRATCH_DIR}/fake-llvm/fake-llvm-cov" llvm_cov path)
  expect_json_substring(b29_cov_tool_evidence "${_cq_tr}" "LLVM"
    llvm_profdata version)
  expect_json_substring(b29_cov_tool_evidence "${_cq_tr}" "LLVM"
    llvm_cov version)
  expect_json(b29_cov_tool_evidence "${_cq_tr}" "ON"
    format_probe format_probe_passed)
endif()

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "migration_T56 fixture contract failed:${_failures}")
endif()
message(STATUS "migration_T56 fixture contract passed")
