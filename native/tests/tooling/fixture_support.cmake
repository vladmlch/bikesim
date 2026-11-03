# fixture_support.cmake — shared helpers for the tooling_* cmake -P tests:
# JSON expectation functions and the b29 fixture-build helpers. Requires
# BUILD_DIR (a configured standalone build) and SCRATCH_DIR; callers own
# _failures and the final FATAL_ERROR report.
include_guard(GLOBAL)
set(_fs_ctx "${BUILD_DIR}/native_check_context.json")
if(NOT EXISTS "${_fs_ctx}")
  message(FATAL_ERROR "fixture_support requires the produced build to emit ${_fs_ctx}")
endif()
file(READ "${_fs_ctx}" _context_json)

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
