# tooling_sweep_engine — fast fixture contract for bike-native-checks'
# per-file engine: bounded parallelism, deterministic record order,
# BIKE_CHECK_JOBS validation. Mock tools only; no real analysis.
#
# Required -D inputs: CHECKS_BINARY, BUILD_DIR, SCRATCH_DIR.
foreach(_required CHECKS_BINARY BUILD_DIR SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_sweep_engine requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")
include("${CMAKE_CURRENT_LIST_DIR}/fixture_support.cmake")

# An eight-TU fixture build under one target.
set(_fx "${SCRATCH_DIR}/engine")
file(MAKE_DIRECTORY "${_fx}/native/src" "${_fx}/build")
file(WRITE "${_fx}/suppressions.json" "{\"schema_version\":1,\"suppressions\":[]}\n")
b29_fixture_context("${_fx}/build" "${_fx}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx}/suppressions.json")
set(_manifest_sources "")
set(_ccdb "")
foreach(_i RANGE 1 8)
  file(WRITE "${_fx}/native/src/tu${_i}.cpp" "int tu${_i}() { return ${_i}; }\n")
  if(NOT _manifest_sources STREQUAL "")
    string(APPEND _manifest_sources ",")
    string(APPEND _ccdb ",")
  endif()
  string(APPEND _manifest_sources
    "{\"path\":\"${_fx}/native/src/tu${_i}.cpp\",\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}")
  b29_ccdb_entry(_entry "${_fx}" "${_fx}/native/src/tu${_i}.cpp"
    "CMakeFiles/bike_native.dir/tu${_i}.cpp.o")
  string(APPEND _ccdb "${_entry}")
endforeach()
file(WRITE "${_fx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":[${_manifest_sources}]}\n")
file(WRITE "${_fx}/build/compile_commands.json" "[${_ccdb}]\n")
set(_summary "${_fx}/build/native_check_summaries/tidy.json")

# A mock tool that takes one second per TU.
file(WRITE "${_fx}/slow-tool"
  "#!/bin/sh\n"
  "if [ \"\${1:-}\" = \"--version\" ]; then echo 'mock analysis tool 1'; exit 0; fi\n"
  "sleep 1\nexit 0\n")
file(CHMOD "${_fx}/slow-tool" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

# Eight TUs on eight workers finish in well under the serial 8 s.
string(TIMESTAMP _t0 "%s")
run_case(engine_parallel EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=8"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
string(TIMESTAMP _t1 "%s")
math(EXPR _elapsed "${_t1} - ${_t0}")
if(_elapsed GREATER 5)
  set(_failures "${_failures}\n  engine_parallel: 8 x 1 s TUs took ${_elapsed} s on 8 workers")
endif()
expect_json(engine_parallel "${_summary}" 8 jobs)
expect_json_length(engine_parallel "${_summary}" 8 records)
# Records stay in manifest (source path) order regardless of finish order.
foreach(_i RANGE 0 7)
  math(EXPR _n "${_i} + 1")
  expect_json(engine_parallel "${_summary}" "${_fx}/native/src/tu${_n}.cpp"
    records ${_i} source)
endforeach()

# One worker is the old serial behavior.
run_case(engine_serial EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=1"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
expect_json(engine_serial "${_summary}" 1 jobs)

# A malformed job count fails closed instead of silently serializing.
run_case(engine_jobs_invalid EXPECT_EXIT FAIL
  EXPECT_STDERR "BIKE_CHECK_JOBS must be a positive integer"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=zero"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")

# A blocking finding in one TU still fails the whole sweep in parallel.
file(WRITE "${_fx}/warn-one-tool"
  "#!/bin/sh\n"
  "if [ \"\${1:-}\" = \"--version\" ]; then echo 'mock analysis tool 1'; exit 0; fi\n"
  "case \"$*\" in *tu5.cpp*) echo 'tu5.cpp:1:1: warning: planted finding' >&2 ;; esac\n"
  "exit 0\n")
file(CHMOD "${_fx}/warn-one-tool" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
run_case(engine_finding EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=8"
        "CLANG_TIDY=${_fx}/warn-one-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
expect_json(engine_finding "${_summary}" "blocking" finding_status)

# ---------------------------------------------------------------------------
# Header-sweep context dedup: two targets whose compile contexts differ only
# in the source file compile the same header probe once; a differing
# definition keeps two real compiles.
#
# The headers kind runs each entry's compiler argv directly (validated
# against context compiler.path), so the fixture compiler is a mock script
# wired as compiler.path and argv[0]. It appends one line to
# probe-calls.log per header_probe.cpp invocation, letting the case count
# real compiles instead of trusting the summary alone.

# b29_header_dedup_fixture(<root> [VARIANT_B]) — one shared.hpp under
# <root>/native/src included by tu_a.cpp (bike_a) and tu_b.cpp (bike_b)
# whose compile arguments are otherwise identical. VARIANT_B adds
# -DVARIANT=1 to bike_b's compile arguments and target context.
function(b29_header_dedup_fixture root)
  cmake_parse_arguments(F "VARIANT_B" "" "" ${ARGN})
  file(MAKE_DIRECTORY "${root}/native/src" "${root}/build")
  file(WRITE "${root}/native/src/shared.hpp"
    "namespace shared { inline int value() { return 1; } }\n")
  file(WRITE "${root}/native/src/tu_a.cpp"
    "#include \"shared.hpp\"\nint tu_a() { return shared::value(); }\n")
  file(WRITE "${root}/native/src/tu_b.cpp"
    "#include \"shared.hpp\"\nint tu_b() { return shared::value(); }\n")
  file(WRITE "${root}/suppressions.json"
    "{\"schema_version\":1,\"suppressions\":[]}\n")
  file(WRITE "${root}/mock-cxx"
    "#!/bin/sh\n"
    "if [ \"\${1:-}\" = \"--version\" ]; then echo 'mock cxx 1'; exit 0; fi\n"
    "case \"\$*\" in *header_probe.cpp*) echo \"\$*\" >> \"${root}/probe-calls.log\" ;; esac\n"
    "exit 0\n")
  file(CHMOD "${root}/mock-cxx"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
  set(_tc_b "${_b29_tc}")
  set(_variant_arg "")
  if(F_VARIANT_B)
    set(_defs_v "[\"VARIANT=1\"]")
    if(NOT _b29_def STREQUAL "")
      set(_defs_v "[\"${_b29_def}\",\"VARIANT=1\"]")
    endif()
    # Single-argument set: a ;-list would corrupt the surrounding JSON.
    set(_tc_b
      "{\"configuration\":\"fixture\",\"cxx_standard\":23,\"cxx_extensions\":false,\"include_directories\":[],\"compile_definitions\":${_defs_v},\"compile_options\":[]}")
    set(_variant_arg "\"-DVARIANT=1\",")
  endif()
  b29_fixture_context("${root}/build" "${root}/native"
    "{\"bike_a\":${_b29_tc},\"bike_b\":${_tc_b}}"
    "${root}/suppressions.json")
  # b29_fixture_context keeps the produced compiler; repoint it at the mock
  # so probe compiles hit the log instead of the real toolchain.
  file(READ "${root}/build/native_check_context.json" _hc)
  string(JSON _hc SET "${_hc}" compiler path "\"${root}/mock-cxx\"")
  file(WRITE "${root}/build/native_check_context.json" "${_hc}")
  file(WRITE "${root}/build/native_sources.json"
    "{\"schema_version\":1,\"first_party_roots\":[\"${root}/native\"],"
    "\"sources\":["
    "{\"path\":\"${root}/native/src/tu_a.cpp\",\"target\":\"bike_a\",\"context\":\"bike_a:fixture\"},"
    "{\"path\":\"${root}/native/src/tu_b.cpp\",\"target\":\"bike_b\",\"context\":\"bike_b:fixture\"}]}\n")
  file(WRITE "${root}/build/compile_commands.json"
    "[{\"directory\":\"${root}\",\"file\":\"${root}/native/src/tu_a.cpp\","
    "\"output\":\"CMakeFiles/bike_a.dir/tu_a.cpp.o\","
    "\"arguments\":[\"${root}/mock-cxx\",${_b29_drv_args_json},"
    "\"-c\",\"${root}/native/src/tu_a.cpp\",\"-o\",\"CMakeFiles/bike_a.dir/tu_a.cpp.o\"]},"
    "{\"directory\":\"${root}\",\"file\":\"${root}/native/src/tu_b.cpp\","
    "\"output\":\"CMakeFiles/bike_b.dir/tu_b.cpp.o\","
    "\"arguments\":[\"${root}/mock-cxx\",${_b29_drv_args_json},${_variant_arg}"
    "\"-c\",\"${root}/native/src/tu_b.cpp\",\"-o\",\"CMakeFiles/bike_b.dir/tu_b.cpp.o\"]}]\n")
endfunction()

# Identical contexts: one real probe compile covers both targets; bike_b is
# reported covered_by the first target in context (target-name) order.
set(_hfx "${SCRATCH_DIR}/headers-dedup")
b29_header_dedup_fixture("${_hfx}")
set(_hsummary "${_hfx}/build/native_check_summaries/headers.json")
run_case(headers_dedup EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind headers --build "${_hfx}/build")
expect_json(headers_dedup "${_hsummary}" 1 header_count)
expect_json(headers_dedup "${_hsummary}" 2 header_context_count)
expect_json(headers_dedup "${_hsummary}" 1 header_translation_unit_count)
expect_json_length(headers_dedup "${_hsummary}" 1 header_duplicate_contexts)
expect_json(headers_dedup "${_hsummary}" "${_hfx}/native/src/shared.hpp"
  header_duplicate_contexts 0 source)
expect_json(headers_dedup "${_hsummary}" "bike_b"
  header_duplicate_contexts 0 target)
expect_json(headers_dedup "${_hsummary}" "bike_a"
  header_duplicate_contexts 0 covered_by)
expect_json_length(headers_dedup "${_hsummary}" 1 records)
expect_json(headers_dedup "${_hsummary}" "bike_a" records 0 target)
file(STRINGS "${_hfx}/probe-calls.log" _probe_calls)
list(LENGTH _probe_calls _probe_count)
if(NOT _probe_count EQUAL 1)
  set(_failures "${_failures}\n  headers_dedup: ${_probe_count} probe compiles, expected 1")
endif()

# A different -D makes the two contexts distinct compiles again.
set(_hvfx "${SCRATCH_DIR}/headers-distinct")
b29_header_dedup_fixture("${_hvfx}" VARIANT_B)
set(_hvsummary "${_hvfx}/build/native_check_summaries/headers.json")
run_case(headers_distinct EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "${CHECKS_BINARY}" --kind headers --build "${_hvfx}/build")
expect_json(headers_distinct "${_hvsummary}" 2 header_context_count)
expect_json(headers_distinct "${_hvsummary}" 2 header_translation_unit_count)
expect_json_length(headers_distinct "${_hvsummary}" 0
  header_duplicate_contexts)
expect_json_length(headers_distinct "${_hvsummary}" 2 records)
file(STRINGS "${_hvfx}/probe-calls.log" _probe_calls)
list(LENGTH _probe_calls _probe_count)
if(NOT _probe_count EQUAL 2)
  set(_failures "${_failures}\n  headers_distinct: ${_probe_count} probe compiles, expected 2")
endif()

# The context's compiler.launcher names what compile_commands.json actually
# prefixes to the compiler. CMake never writes CMAKE_CXX_COMPILER_LAUNCHER
# there, so it is always empty; the build-time wrapper (ccache) is recorded
# separately as compiler.build_launcher, for provenance only.
expect_json_length(context_launcher "${BUILD_DIR}/native_check_context.json" 0
  compiler launcher)
string(JSON _build_launcher_type ERROR_VARIABLE _bl_err
  TYPE "${_context_json}" compiler build_launcher)
if(_bl_err OR NOT _build_launcher_type STREQUAL "ARRAY")
  set(_failures "${_failures}\n  context_launcher: compiler.build_launcher must be an array")
endif()

# (later tasks append cases above this line)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_sweep_engine contract failed:${_failures}")
endif()
message(STATUS "tooling_sweep_engine contract passed")
