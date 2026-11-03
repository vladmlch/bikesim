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

# (later tasks append cases above this line)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_sweep_engine contract failed:${_failures}")
endif()
message(STATUS "tooling_sweep_engine contract passed")
