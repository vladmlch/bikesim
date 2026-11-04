# Test Durations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **This session's user forbids subagents** ("не запускай агентов, делай все сам"): execute inline with superpowers:executing-plans.

**Goal:** Every CTest test states, next to its registration, how long it takes (a number the tooling reads plus a comment saying when and where it was measured); the `slow` label, `TIMEOUT` and `COST` are derived from that number; every run warns when a test's real time leaves its declared range.

**Architecture:** One CMake helper, `bike_test_seconds(<test> <seconds> [SERIAL_STAGE])` in `native/cmake/TestDurations.cmake`, is the only place that sets `slow`, `TIMEOUT` and `COST`. One script, `native/cmake/TestDurationCheck.cmake`, reads `ctest --show-only=json-v1`. In `MODE=audit` (a CTest test in the ordinary set) it rejects any test without a declared time or with hand-written values. In `MODE=report` (called by `run_tests.sh`, `check_change.sh` and `mise run test`) it compares JUnit times with the declarations and prints warnings, never failures. The numbers come from one measurement run inside this plan (Task 4).

**Tech Stack:** CMake ≥ 3.27 (script mode, `string(JSON)`), CTest (`--show-only=json-v1`, `--output-junit`, `COST`, `TIMEOUT`, `LABELS`), bash (`tools/*.sh`, checked by shellcheck through `--kind scripts`), mise.

**Spec:** No separate design document. The decisions below come from the user (2026-11-04) and from the industry research in this session.

## Decisions (user, 2026-11-04)

- D1. Slow threshold: 30 s ("1 ok").
- D2. A test that runs longer than declared gives a warning, not a failure ("2 предупреждение").
- D3. Executing this plan runs the tests, measures them and marks them ("план, во время исполнения которого как раз будут прогнаны и промаркированы тесты"). Each test gets its time recorded ("для каждого из них можно будет записать его время") as a comment right above it ("комментарием прямо над ним").
- D4. The form is a number plus a comment ("Число + комментарий"). The number is read by the tooling, so a stale value produces a warning instead of going stale silently. The comment says when and in which stage it was measured.
- D5. `TIMEOUT` is derived from the number with a large margin ("Да, с большим запасом"): `max(120, 5 × seconds)`.

## Industry basis (research 2026-11-04)

- Bazel declares a class per test (`size`/`timeout`) and, with `--test_verbose_timeout_warnings`, warns when the real time does not fit the class in either direction (https://bazel.build/reference/test-encyclopedia). The report mode mirrors this, with both directions.
- LLVM lit (`.lit_test_times.txt`) and CTest (`CTestCostData.txt`) keep measured times in the build directory and start the slowest tests first. Here `COST` = declared seconds does the same deterministically: CTest documents that tests "will be run in descending order of cost" (https://cmake.org/cmake/help/latest/prop_test/COST.html).
- No notable project keeps per-test times in source comments; the risk is silent staleness. D4 removes that risk: the number is checked on every run.

## Global Constraints

- The derived values live in one place (`TestDurations.cmake`). No registration writes `slow`, `TIMEOUT` or `COST` by hand after this plan; the audit enforces it.
- `slow` keeps its current meaning for every consumer (`CMakePresets.json` test filter `slow|audit|package`, `run_tests.sh`, `docs/TESTING.md`): "not in the ordinary set; runs in the serial `slow` stage". It is set when `seconds ≥ 30` or `SERIAL_STAGE` is given.
- `SERIAL_STAGE` is only for tests that assert on wall-clock speed. Today that is `reference.realtime_gate.*`: it requires `median_real_time_factor >= 1` (`native/tests/reference/test_realtime_gate.cpp`), so it must not share the machine or run under the ordinary leg's `MallocScribble`. `reference.measure_realtime` drives a fake clock and `reference.runtime_single_path` only checks `wall_clock_s > 0`; neither needs it.
- Warnings never change an exit code. A script error (missing file, malformed JSON) does fail: that is a tooling bug, not a timing.
- Report only on the `release` preset: sanitizer and coverage builds are slower by design, and the declared numbers are release numbers.
- No physics, golden or C++ source changes. `git status tests/fixtures` stays clean.
- No subagents. Commit only when the user asks. Linux runs only on explicit request: Linux-only tests do not exist today; if one appears, its declaration is added when it is first measured there.
- Before tooling changes, read `docs/agents/native-engineering.md` (AGENTS.md rule 1).
- Order: execute after `2026-11-04-remove-legacy-mode.md` has merged. That plan deletes tests and edits `Phase3Ports.cmake`; marking first would mark tests about to be deleted.

## Starting data (last full gate, 2026-11-04, 233 tests)

From `/tmp/njd_gate_full3.log` (the no-joint-damping gate; it is not kept, and Task 4 measures afresh):

- 47 tests carry `slow`, but only 5 run 30 s or longer: `tooling_checks` 324 s, `tooling_platform` 140 s, `validation_acceptance` 108 s, `platform_build` 60 s, `reference.native_rider_allocation` 39 s. `reference.realtime_gate.*` (9.8 s, 8.9 s) stays in the slow stage through `SERIAL_STAGE`. The other 40 run 0.3–23.6 s, about 125 s summed, and move into the ordinary set.
- Expected effect: the serial slow stage drops from 815 s to about 690 s. The ordinary leg (185 tests, 309 s summed, 45 s wall at `-j8`) gains about 125 s summed, roughly +15–20 s wall before the `COST` ordering. Level 1 grows from about 34 s to about 50 s. In return, those 40 tests run in every stage.
- Hand-written test `TIMEOUT`s that go away: `tooling_platform` 3600 (`native/cmake/Checks.cmake:87`), 1200 (`:128`), `tooling_install` 600 (`:151`), `tooling_gate` 120 (`:170`), `tooling_sweep_engine` 300 (`:180`), `tooling_checks` 3600 (`:197`), `platform_build` 1200 (`native/cmake/Provenance.cmake:406`), `reference.realtime_gate.*` 900 (`native/tests/reference/Phase3Ports.cmake:36`), `reference.native_loader.*` 45 (`Phase3Ports.cmake:201`), `package_smoke` 600 (`CMakeLists.txt`), `sanitizer_negative_control`/`rtsan_negative_control` 30 (`native/cmake/Sanitizers.cmake:23,31`). `execute_process(... TIMEOUT ...)` calls inside scripts are not test properties and stay.
- `RUN_SERIAL` (`install_smoke`, `package_smoke`, `coverage_smoke`) is untouched: it means "nothing else in parallel within its leg", which is a different fact from the stage.

## Verification budget

Measured costs (`<git-common-dir>/bike-gate/timings.tsv` and the last gate, 2026-11-04): ordinary ctest 45 s, `check_change.sh` 34 s on a warm tree, slow stage 815 s serial, package 45 s, `run_tests.sh full` 1506 s.

- Inside a task: `mise run test '<pattern>'` with the tests the task names (seconds each).
- Task 4, the measurement run the user asked for (D3): the three CTest legs exactly as the gate runs them, about 16 min.
- Task 5: the moved tests three times in the ordinary leg's conditions, about 2 min.
- Level 1 (`mise run check <base>`) three times: after Task 3 (stage A: tooling), after Task 5 (stage B: declarations), after Task 6 (stage C: docs). About 1 min each.
- At the end: one `bash tools/run_tests.sh full` (Task 7), about 25 min. Full rather than native, because the change moves tests between the ordinary and slow stages and adds the report to the slow and package stages, which only `full` runs.
- Expected total test time: about 47 minutes.

## File map

Created:
- `native/cmake/TestDurations.cmake`: constants, `bike_test_timeout`, `bike_test_seconds`.
- `native/cmake/TestDurationCheck.cmake`: `cmake -P` script, `MODE=audit|report`.
- `native/tests/tooling/tooling_test_durations.cmake`: contract test for both files, on scratch inputs.

Modified:
- `CMakeLists.txt`: include `TestDurations.cmake` after `include(CTest)` (line 86); declarations for the tests registered there.
- `native/cmake/Checks.cmake`: register `tooling_test_durations` and `test_duration_audit`; declarations.
- `native/tests/reference/CMakeLists.txt`: `bike_add_reference_test` gains a required `SECONDS <n>` and optional `SERIAL_STAGE`.
- Every file that registers a test: `native/tests/reference/Phase3Ports.cmake` and the reference `CMakeLists.txt`, `native/tests/*/CMakeLists.txt`, `native/cmake/{Checks,Provenance,Sanitizers,Coverage,NativeValidation,NativeStartup,NativePhysicsResearch,RealtimeMeasurement,RecordingApplication,BikeCheckContext,BikeHostTarget,BikeLinkage,BikeProviderIdentity,BikeDependencyLock,BikeAppleSDK,BikeCompilerPolicy,CoverageTools}.cmake` where they call `add_test`. Task 5 Step 1 lists the exact sites from CTest's own backtraces.
- `tools/run_tests.sh`, `tools/check_change.sh`, `mise.toml`: JUnit for every CTest leg, then the report.
- `native/tests/tooling/tooling_gate.cmake`: stub `ctest` writes the JUnit file; expectations for the report calls.
- `AGENTS.md`, `docs/TESTING.md`: how to declare and re-measure.

---

### Task 0: Branch and budget

**Files:** none.

- [ ] **Step 1: Branch in the built worktree**

```bash
cd .worktrees/no-joint-damping        # build/native-release is already built here
git status --short                    # must be empty
git switch -c test-durations main     # main itself stays checked out in the main checkout
```

- [ ] **Step 2: Write the budget down**

Read `$(git rev-parse --path-format=absolute --git-common-dir)/bike-gate/timings.tsv`, take the newest `full` and `native` rows and copy the budget from "Verification budget" into the task log, updating any number the new rows change. No baseline run: the legacy-removal plan ends with a full gate on this code (AGENTS.md "Reuse evidence").

- [ ] **Step 3: Read the engineering rules**

Read `docs/agents/native-engineering.md`.

---

### Task 1: The helper `bike_test_seconds`

**Files:**
- Create: `native/cmake/TestDurations.cmake`
- Create: `native/tests/tooling/tooling_test_durations.cmake`
- Modify: `CMakeLists.txt:86` (include), `native/cmake/Checks.cmake` (register the contract test)

**Interfaces:**
- Produces: `bike_test_seconds(<test> <seconds> [SERIAL_STAGE])`, `bike_test_timeout(<seconds> <out-var>)`, variables `BIKE_SLOW_TEST_SECONDS` (30), `BIKE_TEST_TIMEOUT_FACTOR` (5), `BIKE_TEST_TIMEOUT_FLOOR` (120). Test properties `BIKE_EXPECTED_SECONDS` (integer) and `BIKE_SERIAL_STAGE` (TRUE when given). `ctest --show-only=json-v1` lists both properties (verified 2026-11-04 with CMake 4.4.3).

- [ ] **Step 1: Write the failing contract test (helper part)**

Create `native/tests/tooling/tooling_test_durations.cmake`:

```cmake
# tooling_test_durations — contract for native/cmake/TestDurations.cmake and
# native/cmake/TestDurationCheck.cmake on scratch projects and scratch
# JSON/JUnit inputs. Configures only `project(... NONE)` scratch trees, so it
# stays in the ordinary CTest set.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR, CTEST.
foreach(_required REPO_ROOT SCRATCH_DIR CTEST)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_test_durations requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")
set(_check "${REPO_ROOT}/native/cmake/TestDurationCheck.cmake")

# run_case(<name> EXPECT_EXIT <rc|FAIL> [EXPECT_OUTPUT <needle>...]
#          [REJECT_OUTPUT <needle>...] COMMAND <argv...>)
function(run_case name)
  cmake_parse_arguments(C "" "EXPECT_EXIT" "EXPECT_OUTPUT;REJECT_OUTPUT;COMMAND" ${ARGN})
  execute_process(COMMAND ${C_COMMAND}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  set(_all "${_out}\n${_err}")
  if(C_EXPECT_EXIT STREQUAL "FAIL")
    if(_rc EQUAL 0)
      set(_failures "${_failures}\n  ${name}: expected failure, exited 0:\n${_all}" PARENT_SCOPE)
    endif()
  elseif(NOT _rc EQUAL ${C_EXPECT_EXIT})
    set(_failures "${_failures}\n  ${name}: expected exit ${C_EXPECT_EXIT}, got ${_rc}:\n${_all}" PARENT_SCOPE)
  endif()
  foreach(_needle IN LISTS C_EXPECT_OUTPUT)
    string(FIND "${_all}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      set(_failures "${_failures}\n  ${name}: output missing '${_needle}':\n${_all}" PARENT_SCOPE)
    endif()
  endforeach()
  foreach(_needle IN LISTS C_REJECT_OUTPUT)
    string(FIND "${_all}" "${_needle}" _pos)
    if(NOT _pos EQUAL -1)
      set(_failures "${_failures}\n  ${name}: output must not contain '${_needle}':\n${_all}" PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

# scratch_project(<name> <body>) — a NONE-language project that includes
# the helper; <body> registers tests with `cmake -E true`.
function(scratch_project name body)
  file(MAKE_DIRECTORY "${SCRATCH_DIR}/${name}")
  file(WRITE "${SCRATCH_DIR}/${name}/CMakeLists.txt"
    "cmake_minimum_required(VERSION 3.27)\n"
    "project(${name} NONE)\n"
    "enable_testing()\n"
    "include(\"${REPO_ROOT}/native/cmake/TestDurations.cmake\")\n"
    "${body}\n")
endfunction()

# --- the helper derives COST, TIMEOUT and the slow label ------------------
scratch_project(derive [=[
add_test(NAME fast COMMAND ${CMAKE_COMMAND} -E true)
set_tests_properties(fast PROPERTIES LABELS "unit")
bike_test_seconds(fast 2)
add_test(NAME long COMMAND ${CMAKE_COMMAND} -E true)
bike_test_seconds(long 40)
add_test(NAME timed COMMAND ${CMAKE_COMMAND} -E true)
bike_test_seconds(timed 9 SERIAL_STAGE)
]=])
run_case(derive_configure EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/derive" -B "${SCRATCH_DIR}/derive/b")
execute_process(COMMAND "${CTEST}" --test-dir "${SCRATCH_DIR}/derive/b" --show-only=json-v1
  OUTPUT_VARIABLE _json RESULT_VARIABLE _rc)
string(JSON _tests GET "${_json}" tests)
# want(<index> <property> <expected>) — exact property value in json-v1.
function(want index property expected)
  string(JSON _t GET "${_tests}" ${index})
  string(JSON _name GET "${_t}" name)
  string(JSON _n LENGTH "${_t}" properties)
  math(EXPR _last "${_n} - 1")
  set(_got "<absent>")
  foreach(_k RANGE ${_last})
    string(JSON _p GET "${_t}" properties ${_k} name)
    if(_p STREQUAL property)
      string(JSON _got GET "${_t}" properties ${_k} value)
    endif()
  endforeach()
  if(NOT "${_got}" STREQUAL "${expected}")
    set(_failures "${_failures}\n  derive: ${_name} ${property} is '${_got}', expected '${expected}'" PARENT_SCOPE)
  endif()
endfunction()
want(0 BIKE_EXPECTED_SECONDS 2)
want(0 COST 2.0)
want(0 TIMEOUT 120.0)                 # floor: 5 × 2 = 10 < 120
want(0 LABELS [=[[ "unit" ]]=])
want(1 TIMEOUT 200.0)                 # 5 × 40
want(1 LABELS [=[[ "slow" ]]=])       # 40 ≥ 30
want(2 BIKE_SERIAL_STAGE TRUE)
want(2 LABELS [=[[ "slow" ]]=])       # SERIAL_STAGE, though 9 < 30

# --- hand-written values are rejected at configure time -------------------
scratch_project(hand_timeout [=[
add_test(NAME t COMMAND ${CMAKE_COMMAND} -E true)
set_tests_properties(t PROPERTIES TIMEOUT 600)
bike_test_seconds(t 5)
]=])
run_case(hand_timeout EXPECT_EXIT FAIL
  EXPECT_OUTPUT "remove the hand-written TIMEOUT"
  COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/hand_timeout" -B "${SCRATCH_DIR}/hand_timeout/b")
scratch_project(hand_slow [=[
add_test(NAME t COMMAND ${CMAKE_COMMAND} -E true)
set_tests_properties(t PROPERTIES LABELS "x;slow")
bike_test_seconds(t 5)
]=])
run_case(hand_slow EXPECT_EXIT FAIL
  EXPECT_OUTPUT "remove the hand-written slow label"
  COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/hand_slow" -B "${SCRATCH_DIR}/hand_slow/b")
scratch_project(bad_seconds [=[
add_test(NAME t COMMAND ${CMAKE_COMMAND} -E true)
bike_test_seconds(t 2.5)
]=])
run_case(bad_seconds EXPECT_EXIT FAIL
  EXPECT_OUTPUT "seconds must be a positive integer"
  COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/bad_seconds" -B "${SCRATCH_DIR}/bad_seconds/b")
scratch_project(no_test [=[
bike_test_seconds(missing 5)
]=])
run_case(no_test EXPECT_EXIT FAIL
  EXPECT_OUTPUT "no such test in this directory"
  COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/no_test" -B "${SCRATCH_DIR}/no_test/b")

# (TestDurationCheck.cmake cases are appended by Task 2 above this line.)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_test_durations contract failed:${_failures}")
endif()
message(STATUS "tooling_test_durations contract passed")
```

Note on `want(... LABELS ...)`: `string(JSON GET)` returns an array value as its JSON text, `[ "unit" ]` (verified with CMake 4.4.3).

Register it in `native/cmake/Checks.cmake`, right after the `tooling_gate` block (`if(BUILD_TESTING) add_test(NAME tooling_gate ...)`, around line 170):

```cmake
# --- tooling_test_durations: the duration helper and its check script -------
# Scratch NONE-language configures only; stays in the ordinary set.
if(BUILD_TESTING)
  add_test(NAME tooling_test_durations
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_test_durations"
      -D "CTEST=${CMAKE_CTEST_COMMAND}"
      -P "${_checks_native_dir}/tests/tooling/tooling_test_durations.cmake")
  set_tests_properties(tooling_test_durations PROPERTIES LABELS "tooling")
endif()
```

(The declaration for this test is added in Task 5 with all the others.)

- [ ] **Step 2: Run it to verify it fails**

Run: `mise run test '^tooling_test_durations$'`
Expected: FAIL. The scratch configures fail with `include could not find requested file: .../native/cmake/TestDurations.cmake`, so `derive_configure` reports an unexpected exit.

- [ ] **Step 3: Write the helper**

Create `native/cmake/TestDurations.cmake`:

```cmake
# TestDurations.cmake — every CTest test declares how long it takes.
#
# bike_test_seconds(<test> <seconds> [SERIAL_STAGE])
#   Call in the directory that registered <test>, after its last
#   set_tests_properties (a later LABELS/TIMEOUT would overwrite what this
#   derives; the audit in TestDurationCheck.cmake catches that).
#   <seconds> is the measured time rounded up to a whole second; the
#   comment above the registration says when and in which stage it was
#   measured. Derived here and nowhere else:
#     COST    = <seconds>   (ctest -j starts the longest tests first)
#     TIMEOUT = max(BIKE_TEST_TIMEOUT_FLOOR, BIKE_TEST_TIMEOUT_FACTOR × <seconds>)
#     LABELS += slow        when <seconds> ≥ BIKE_SLOW_TEST_SECONDS or SERIAL_STAGE
#   SERIAL_STAGE: the test asserts on wall-clock speed, so it runs in the
#   serial slow stage whatever its time — it must not share the machine or
#   run under the ordinary leg's allocator debugging (tools/run_tests.sh).
#
# Included by the root CMakeLists.txt and, in script mode, by
# TestDurationCheck.cmake, which shares the constants and the timeout rule.
include_guard(GLOBAL)

# User decisions 2026-11-04: slow from 30 s; a large timeout margin.
set(BIKE_SLOW_TEST_SECONDS 30)
set(BIKE_TEST_TIMEOUT_FACTOR 5)
set(BIKE_TEST_TIMEOUT_FLOOR 120)

# bike_test_timeout(<seconds> <out-var>) — the derived TIMEOUT.
function(bike_test_timeout seconds out_var)
  math(EXPR _timeout "${seconds} * ${BIKE_TEST_TIMEOUT_FACTOR}")
  if(_timeout LESS BIKE_TEST_TIMEOUT_FLOOR)
    set(_timeout ${BIKE_TEST_TIMEOUT_FLOOR})
  endif()
  set(${out_var} ${_timeout} PARENT_SCOPE)
endfunction()

function(bike_test_seconds test seconds)
  cmake_parse_arguments(PARSE_ARGV 2 _dur "SERIAL_STAGE" "" "")
  if(_dur_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "bike_test_seconds(${test}): unexpected arguments '${_dur_UNPARSED_ARGUMENTS}'")
  endif()
  if(NOT seconds MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR
      "bike_test_seconds(${test}): seconds must be a positive integer, got '${seconds}'")
  endif()
  if(NOT TEST "${test}")
    message(FATAL_ERROR
      "bike_test_seconds(${test}): no such test in this directory; call it after add_test")
  endif()
  get_test_property("${test}" TIMEOUT _by_hand)
  if(_by_hand)
    message(FATAL_ERROR
      "bike_test_seconds(${test}): remove the hand-written TIMEOUT (${_by_hand}); "
      "it is derived from the seconds")
  endif()
  get_test_property("${test}" LABELS _labels)
  if("slow" IN_LIST _labels)
    message(FATAL_ERROR
      "bike_test_seconds(${test}): remove the hand-written slow label; "
      "it is derived from the seconds")
  endif()
  bike_test_timeout(${seconds} _timeout)
  set_tests_properties("${test}" PROPERTIES
    BIKE_EXPECTED_SECONDS ${seconds} COST ${seconds} TIMEOUT ${_timeout})
  if(_dur_SERIAL_STAGE)
    set_tests_properties("${test}" PROPERTIES BIKE_SERIAL_STAGE TRUE)
  endif()
  if(seconds GREATER_EQUAL BIKE_SLOW_TEST_SECONDS OR _dur_SERIAL_STAGE)
    set_property(TEST "${test}" APPEND PROPERTY LABELS slow)
  endif()
endfunction()
```

In `CMakeLists.txt`, right after line 86 `include(CTest)`:

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/TestDurations.cmake")
```

- [ ] **Step 4: Run it to verify it passes**

Run: `mise run test '^tooling_test_durations$'`
Expected: PASS, `tooling_test_durations contract passed`.

- [ ] **Step 5: Commit**

```bash
git add native/cmake/TestDurations.cmake native/tests/tooling/tooling_test_durations.cmake \
  native/cmake/Checks.cmake CMakeLists.txt
git commit -m "build(tests): derive slow, TIMEOUT and COST from a declared test time"
```

---

### Task 2: The audit and the report

**Files:**
- Create: `native/cmake/TestDurationCheck.cmake`
- Modify: `native/tests/tooling/tooling_test_durations.cmake` (append cases)

**Interfaces:**
- Consumes: `TestDurations.cmake` (constants, `bike_test_timeout`).
- Produces: `cmake -DMODE=audit|report [-DTESTS_JSON=<file> | -DCTEST=<ctest> -DBUILD_DIR=<dir>] [-DJUNIT=<file>[;<file>…]] -P native/cmake/TestDurationCheck.cmake`.
  - Audit: exit 0 with `test durations audit: <n> tests declare their time`, or a fatal error listing `  <test>: <problem>` lines.
  - Report: exit 0, one `warning: test <name> took <t> s, declared <s> s…` line per warning, then `test durations: <n> tests checked, <w> warning(s)`.
  - Warning rule: slower when `measured > 2 × declared + 5 s`; faster (slow tests only, not `SERIAL_STAGE`) when `measured < declared / 3`. The 5 s keeps a sub-second test's noise from warning; the two directions follow Bazel's `--test_verbose_timeout_warnings`.

- [ ] **Step 1: Append the failing cases**

Insert above the `# (TestDurationCheck.cmake cases are appended by Task 2 ...)` line of `native/tests/tooling/tooling_test_durations.cmake`, then delete that marker line:

```cmake
# --- audit over scratch json-v1 documents ---------------------------------
# json_test(<out-var> <name> <properties-json>) — one json-v1 test entry.
function(json_test out name properties)
  set(${out} "{ \"name\" : \"${name}\", \"properties\" : [ ${properties} ] }" PARENT_SCOPE)
endfunction()
json_test(_fast fast [=[
  { "name" : "COST", "value" : 2.0 }, { "name" : "LABELS", "value" : [ "unit" ] },
  { "name" : "TIMEOUT", "value" : 120.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "2" }]=])
json_test(_long long [=[
  { "name" : "COST", "value" : 40.0 }, { "name" : "LABELS", "value" : [ "slow" ] },
  { "name" : "TIMEOUT", "value" : 200.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "40" }]=])
json_test(_timed timed [=[
  { "name" : "COST", "value" : 9.0 }, { "name" : "LABELS", "value" : [ "slow" ] },
  { "name" : "TIMEOUT", "value" : 120.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "9" },
  { "name" : "BIKE_SERIAL_STAGE", "value" : "TRUE" }]=])
json_test(_bare bare [=[{ "name" : "LABELS", "value" : [ "unit" ] }]=])
json_test(_hand_timeout hand_timeout [=[
  { "name" : "COST", "value" : 40.0 }, { "name" : "LABELS", "value" : [ "slow" ] },
  { "name" : "TIMEOUT", "value" : 3600.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "40" }]=])
json_test(_lost_slow lost_slow [=[
  { "name" : "COST", "value" : 40.0 }, { "name" : "LABELS", "value" : [ "unit" ] },
  { "name" : "TIMEOUT", "value" : 200.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "40" }]=])
json_test(_stray_slow stray_slow [=[
  { "name" : "COST", "value" : 2.0 }, { "name" : "LABELS", "value" : [ "slow" ] },
  { "name" : "TIMEOUT", "value" : 120.0 }, { "name" : "BIKE_EXPECTED_SECONDS", "value" : "2" }]=])
# tests_json(<file> <entry>...) — a json-v1 document with these tests.
function(tests_json file)
  list(JOIN ARGN ", " _entries)
  file(WRITE "${file}" "{ \"kind\" : \"ctestInfo\", \"tests\" : [ ${_entries} ] }\n")
endfunction()

tests_json("${SCRATCH_DIR}/good.json" "${_fast}" "${_long}" "${_timed}")
run_case(audit_good EXPECT_EXIT 0
  EXPECT_OUTPUT "test durations audit: 3 tests declare their time"
  COMMAND "${CMAKE_COMMAND}" -DMODE=audit "-DTESTS_JSON=${SCRATCH_DIR}/good.json" -P "${_check}")
tests_json("${SCRATCH_DIR}/bad.json"
  "${_fast}" "${_bare}" "${_hand_timeout}" "${_lost_slow}" "${_stray_slow}")
run_case(audit_bad EXPECT_EXIT FAIL
  EXPECT_OUTPUT "bare: no declared time"
                "hand_timeout: TIMEOUT 3600.0 is not the derived 200"
                "lost_slow: slow label FALSE, its declared time requires TRUE"
                "stray_slow: slow label TRUE, its declared time requires FALSE"
  REJECT_OUTPUT "  fast:"
  COMMAND "${CMAKE_COMMAND}" -DMODE=audit "-DTESTS_JSON=${SCRATCH_DIR}/bad.json" -P "${_check}")
tests_json("${SCRATCH_DIR}/empty.json")
run_case(audit_empty EXPECT_EXIT FAIL
  EXPECT_OUTPUT "the test list is empty"
  COMMAND "${CMAKE_COMMAND}" -DMODE=audit "-DTESTS_JSON=${SCRATCH_DIR}/empty.json" -P "${_check}")
run_case(audit_no_input EXPECT_EXIT FAIL
  EXPECT_OUTPUT "needs -DTESTS_JSON or -DCTEST and -DBUILD_DIR"
  COMMAND "${CMAKE_COMMAND}" -DMODE=audit -P "${_check}")
run_case(bad_mode EXPECT_EXIT FAIL
  EXPECT_OUTPUT "MODE must be audit or report"
  COMMAND "${CMAKE_COMMAND}" -DMODE=bogus "-DTESTS_JSON=${SCRATCH_DIR}/good.json" -P "${_check}")

# --- report over a scratch JUnit file --------------------------------------
# fast declared 2 s took 30 s (> 2×2+5): slower warning.
# long declared 40 s took 5 s (< 40/3): faster warning.
# timed (SERIAL_STAGE) declared 9 s took 0.5 s: no warning.
# a case CTest did not run and an undeclared name are ignored; a
# sub-microsecond time in e-notation parses.
file(WRITE "${SCRATCH_DIR}/junit.xml" [=[<?xml version="1.0" encoding="UTF-8"?>
<testsuite name="scratch" tests="5">
	<testcase name="fast" classname="fast" time="30.25" status="run">
	</testcase>
	<testcase name="long" classname="long" time="5" status="run">
	</testcase>
	<testcase name="timed" classname="timed" time="0.5" status="run">
	</testcase>
	<testcase name="long" classname="long" time="999" status="notrun">
	</testcase>
	<testcase name="unknown" classname="unknown" time="1.2e-05" status="run">
	</testcase>
</testsuite>
]=])
run_case(report EXPECT_EXIT 0
  EXPECT_OUTPUT "warning: test fast took 30.25 s, declared 2 s"
                "warning: test long took 5 s, declared 40 s (slow)"
                "test durations: 3 tests checked, 2 warning(s)"
  REJECT_OUTPUT "test timed took" "took 999"
  COMMAND "${CMAKE_COMMAND}" -DMODE=report "-DTESTS_JSON=${SCRATCH_DIR}/good.json"
    "-DJUNIT=${SCRATCH_DIR}/junit.xml" -P "${_check}")
run_case(report_missing_junit EXPECT_EXIT FAIL
  EXPECT_OUTPUT "JUnit file not found"
  COMMAND "${CMAKE_COMMAND}" -DMODE=report "-DTESTS_JSON=${SCRATCH_DIR}/good.json"
    "-DJUNIT=${SCRATCH_DIR}/absent.xml" -P "${_check}")

# --- end to end: the helper's output passes the audit through real ctest ---
run_case(audit_derived EXPECT_EXIT 0
  EXPECT_OUTPUT "test durations audit: 3 tests declare their time"
  COMMAND "${CMAKE_COMMAND}" -DMODE=audit "-DCTEST=${CTEST}"
    "-DBUILD_DIR=${SCRATCH_DIR}/derive/b" -P "${_check}")
```

- [ ] **Step 2: Run it to verify it fails**

Run: `mise run test '^tooling_test_durations$'`
Expected: FAIL, with every new case reporting an unexpected exit or missing output, because `TestDurationCheck.cmake` does not exist.

- [ ] **Step 3: Write the script**

Create `native/cmake/TestDurationCheck.cmake`:

```cmake
# TestDurationCheck.cmake — cmake -P script over declared test times
# (TestDurations.cmake).
#   -DMODE=audit   every test declares its time, and COST, TIMEOUT and the
#                  slow label equal what the declaration derives. Any
#                  mismatch is an error; registered as the CTest test
#                  test_duration_audit.
#   -DMODE=report  compare measured times from JUnit files (ctest
#                  --output-junit) with the declarations and print one
#                  warning per test outside its range. A warning never
#                  fails (user decision 2026-11-04); a missing or
#                  malformed input does.
# Test list: -DTESTS_JSON=<file> (ctest --show-only=json-v1 output), or
# -DCTEST=<ctest> -DBUILD_DIR=<dir> to ask ctest. Report: -DJUNIT=<f>[;<f>…].
include("${CMAKE_CURRENT_LIST_DIR}/TestDurations.cmake")

if(NOT MODE STREQUAL "audit" AND NOT MODE STREQUAL "report")
  message(FATAL_ERROR "TestDurationCheck: MODE must be audit or report, got '${MODE}'")
endif()
if(DEFINED TESTS_JSON)
  file(READ "${TESTS_JSON}" _json)
elseif(DEFINED CTEST AND DEFINED BUILD_DIR)
  execute_process(COMMAND "${CTEST}" --test-dir "${BUILD_DIR}" --show-only=json-v1
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _json ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "TestDurationCheck: ctest --show-only=json-v1 failed (${_rc}): ${_err}")
  endif()
else()
  message(FATAL_ERROR "TestDurationCheck: needs -DTESTS_JSON or -DCTEST and -DBUILD_DIR")
endif()
string(JSON _tests GET "${_json}" tests)
string(JSON _test_count LENGTH "${_tests}")

# _test_fields(<test-json>) — sets _name, _seconds ("" when undeclared),
# _cost, _timeout, _slow (TRUE/FALSE), _serial (TRUE/FALSE) in the caller.
function(_test_fields test_json)
  string(JSON _name GET "${test_json}" name)
  set(_seconds "")
  set(_cost "")
  set(_timeout "")
  set(_slow FALSE)
  set(_serial FALSE)
  string(JSON _n ERROR_VARIABLE _no_properties LENGTH "${test_json}" properties)
  if(NOT _no_properties AND _n GREATER 0)
    math(EXPR _last "${_n} - 1")
    foreach(_k RANGE ${_last})
      string(JSON _property GET "${test_json}" properties ${_k} name)
      string(JSON _value GET "${test_json}" properties ${_k} value)
      if(_property STREQUAL "BIKE_EXPECTED_SECONDS")
        set(_seconds "${_value}")
      elseif(_property STREQUAL "COST")
        set(_cost "${_value}")
      elseif(_property STREQUAL "TIMEOUT")
        set(_timeout "${_value}")
      elseif(_property STREQUAL "BIKE_SERIAL_STAGE" AND _value)
        set(_serial TRUE)
      elseif(_property STREQUAL "LABELS")
        string(JSON _label_count LENGTH "${test_json}" properties ${_k} value)
        if(_label_count GREATER 0)
          math(EXPR _label_last "${_label_count} - 1")
          foreach(_m RANGE ${_label_last})
            string(JSON _label GET "${test_json}" properties ${_k} value ${_m})
            if(_label STREQUAL "slow")
              set(_slow TRUE)
            endif()
          endforeach()
        endif()
      endif()
    endforeach()
  endif()
  foreach(_field _name _seconds _cost _timeout _slow _serial)
    set(${_field} "${${_field}}" PARENT_SCOPE)
  endforeach()
endfunction()

# _milliseconds(<seconds-text> <out-var>) — JUnit time to integer ms
# (math(EXPR) is integer-only). CTest prints tiny times in e-notation.
function(_milliseconds text out_var)
  if(text MATCHES "[eE]-")
    set(${out_var} 0 PARENT_SCOPE)
    return()
  endif()
  if(NOT text MATCHES "^([0-9]+)(\\.([0-9]*))?$")
    message(FATAL_ERROR "TestDurationCheck: malformed JUnit time '${text}'")
  endif()
  set(_whole "${CMAKE_MATCH_1}")
  string(SUBSTRING "${CMAKE_MATCH_3}000" 0 3 _fraction)
  # The leading 1 keeps a fraction such as 045 decimal.
  math(EXPR _ms "${_whole} * 1000 + 1${_fraction} - 1000")
  set(${out_var} ${_ms} PARENT_SCOPE)
endfunction()

# A build always registers tests; an empty list means a wrong BUILD_DIR.
# (foreach(RANGE 0 -1) would also iterate 0 and -1, so this guard is needed.)
if(_test_count EQUAL 0)
  message(FATAL_ERROR "TestDurationCheck: the test list is empty")
endif()
math(EXPR _last_test "${_test_count} - 1")

if(MODE STREQUAL "audit")
  set(_errors "")
  set(_declared 0)
  foreach(_i RANGE ${_last_test})
    string(JSON _test GET "${_tests}" ${_i})
    _test_fields("${_test}")
    if(_seconds STREQUAL "")
      string(APPEND _errors "\n  ${_name}: no declared time; add bike_test_seconds(${_name} <seconds>)"
        " (or SECONDS <seconds> in bike_add_reference_test) with a measured comment above it")
      continue()
    endif()
    if(NOT _seconds MATCHES "^[1-9][0-9]*$")
      string(APPEND _errors "\n  ${_name}: declared time '${_seconds}' is not a positive integer")
      continue()
    endif()
    math(EXPR _declared "${_declared} + 1")
    bike_test_timeout(${_seconds} _want_timeout)
    if(NOT _timeout EQUAL _want_timeout)
      string(APPEND _errors "\n  ${_name}: TIMEOUT ${_timeout} is not the derived ${_want_timeout};"
        " a hand-written TIMEOUT after bike_test_seconds overrides it")
    endif()
    if(NOT _cost EQUAL _seconds)
      string(APPEND _errors "\n  ${_name}: COST ${_cost} is not the declared ${_seconds}")
    endif()
    set(_want_slow FALSE)
    if(_seconds GREATER_EQUAL BIKE_SLOW_TEST_SECONDS OR _serial)
      set(_want_slow TRUE)
    endif()
    if(NOT _slow STREQUAL _want_slow)
      string(APPEND _errors "\n  ${_name}: slow label ${_slow}, its declared time requires ${_want_slow};"
        " a LABELS set after bike_test_seconds drops or adds it")
    endif()
  endforeach()
  if(NOT _errors STREQUAL "")
    message(FATAL_ERROR "test durations audit failed:${_errors}")
  endif()
  message(STATUS "test durations audit: ${_declared} tests declare their time")
  return()
endif()

# MODE=report
if(NOT DEFINED JUNIT)
  message(FATAL_ERROR "TestDurationCheck: report needs -DJUNIT=<file>")
endif()
foreach(_i RANGE ${_last_test})
  string(JSON _test GET "${_tests}" ${_i})
  _test_fields("${_test}")
  if(NOT _seconds STREQUAL "")
    set("_declared_${_name}" "${_seconds}")
    set("_serial_${_name}" "${_serial}")
  endif()
endforeach()
set(_checked 0)
set(_warnings 0)
foreach(_file IN LISTS JUNIT)
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "TestDurationCheck: JUnit file not found: ${_file}")
  endif()
  file(STRINGS "${_file}" _cases REGEX "<testcase ")
  foreach(_case IN LISTS _cases)
    if(NOT _case MATCHES "<testcase name=\"([^\"]+)\"[^>]* time=\"([^\"]+)\"[^>]* status=\"([a-z]+)\"")
      message(FATAL_ERROR "TestDurationCheck: unexpected JUnit line in ${_file}: ${_case}")
    endif()
    set(_case_name "${CMAKE_MATCH_1}")
    set(_case_time "${CMAKE_MATCH_2}")
    set(_case_status "${CMAKE_MATCH_3}")
    if(NOT _case_status STREQUAL "run" OR NOT DEFINED "_declared_${_case_name}")
      continue()
    endif()
    math(EXPR _checked "${_checked} + 1")
    set(_seconds "${_declared_${_case_name}}")
    _milliseconds("${_case_time}" _ms)
    math(EXPR _slower_ms "(2 * ${_seconds} + 5) * 1000")
    math(EXPR _faster_ms "${_seconds} * 1000 / 3")
    if(_ms GREATER _slower_ms)
      message("warning: test ${_case_name} took ${_case_time} s, declared ${_seconds} s:"
        " re-measure it and update its declared seconds and the comment above it")
      math(EXPR _warnings "${_warnings} + 1")
    elseif(_seconds GREATER_EQUAL BIKE_SLOW_TEST_SECONDS
           AND NOT _serial_${_case_name} AND _ms LESS _faster_ms)
      message("warning: test ${_case_name} took ${_case_time} s, declared ${_seconds} s (slow):"
        " re-measure it; it may belong in the ordinary set")
      math(EXPR _warnings "${_warnings} + 1")
    endif()
  endforeach()
endforeach()
message("test durations: ${_checked} tests checked, ${_warnings} warning(s)")
```

Verified 2026-11-04 (CMake 4.4.3): `string(JSON GET)` returns an array as `[ "unit" ]` and a JSON `true` as `ON`. `foreach(RANGE 0 -1)` iterates 0 and −1, which is why the empty-list guard is an error instead of a loop bound.

- [ ] **Step 4: Run it to verify it passes**

Run: `mise run test '^tooling_test_durations$'`
Expected: PASS. If a needle differs only in number formatting (for example `TIMEOUT 3600.0` versus `3600`), fix the needle to what CTest's json-v1 really prints, since that is what users will see. Do not change the script to match the needle.

- [ ] **Step 5: Commit**

```bash
git add native/cmake/TestDurationCheck.cmake native/tests/tooling/tooling_test_durations.cmake
git commit -m "build(tests): audit declared test times and report drift"
```

---

### Task 3: Every CTest leg writes JUnit and reports

**Files:**
- Modify: `tools/run_tests.sh` (`run_native_gate`, the `full` loop), `tools/check_change.sh`, `mise.toml` (`[tasks.test]`)
- Modify: `native/tests/tooling/tooling_gate.cmake` (stub ctest, expectations)

**Interfaces:**
- Consumes: `TestDurationCheck.cmake MODE=report` (Task 2).
- Produces: JUnit files `build/native-release/Testing/junit.xml` (ordinary leg, existing), `junit-slow.xml`, `junit-package.xml` (gate), `junit-change.xml` (level 1), `junit-task.xml` (`mise run test`).

- [ ] **Step 1: Update the gate contract first**

In `native/tests/tooling/tooling_gate.cmake`, replace the stub loop `foreach(_tool cmake ctest) ... endforeach()` with two stubs. The stub ctest creates the file named by `--output-junit`, so the launchers see a JUnit file as they would after a real run:

```cmake
file(WRITE "${_stub_dir}/cmake"
  "#!/usr/bin/env bash\n"
  "printf 'cmake %s\\n' \"$*\" >> \"\${STUB_LOG}\"\n"
  "exit 0\n")
file(WRITE "${_stub_dir}/ctest"
  "#!/usr/bin/env bash\n"
  "printf 'ctest %s\\n' \"$*\" >> \"\${STUB_LOG}\"\n"
  "while (($#)); do\n"
  "  if [[ $1 == --output-junit ]]; then mkdir -p \"\$(dirname \"$2\")\"; : > \"$2\"; fi\n"
  "  shift\n"
  "done\n"
  "exit 0\n")
foreach(_tool cmake ctest)
  file(CHMOD "${_stub_dir}/${_tool}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endforeach()
```

In the `gate_native` case, the report now sits between the ordinary CTest and the first sweep:

```cmake
list(GET _calls 4 _c4)
if(NOT _c0 MATCHES "^cmake --preset release$"
   OR NOT _c1 MATCHES "^cmake --build --preset release --parallel 8$"
   OR NOT _c2 MATCHES "^ctest --test-dir .* -LE slow\\|package "
   OR NOT _c3 MATCHES "^cmake -DMODE=report .*-DJUNIT=.*/Testing/junit\\.xml -P .*/TestDurationCheck\\.cmake$"
   OR NOT _c4 MATCHES "^checks scripts ")
```

(Add the `list(GET _calls 4 _c4)` next to the existing `list(GET _calls 3 _c3)`.)

Above the `# (run_tests.sh cases are appended by later tasks above this line.)` line, add:

```cmake
# full: the slow and package legs write JUnit and are reported too.
file(REMOVE "${_log}")
gate_run(gate_full EXPECT_EXIT 0 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" full)
file(READ "${_log}" _full_calls)
foreach(_label slow package)
  if(NOT _full_calls MATCHES
      "ctest --test-dir [^\n]* -L ${_label} --no-tests=error --output-on-failure --output-junit [^\n]*/Testing/junit-${_label}\\.xml\ncmake -DMODE=report [^\n]*-DJUNIT=[^\n]*/Testing/junit-${_label}\\.xml ")
    set(_failures "${_failures}\n  gate_full: no JUnit+report for ${_label}:\n${_full_calls}")
  endif()
endforeach()

# A non-release preset runs the same legs without the report: the declared
# times are release numbers.
file(REMOVE "${_log}")
gate_run(gate_asan_no_report EXPECT_EXIT 0 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" native asan)
file(READ "${_log}" _asan_calls)
string(FIND "${_asan_calls}" "-DMODE=report" _asan_report)
if(NOT _asan_report EQUAL -1)
  set(_failures "${_failures}\n  gate_asan_no_report: report ran on asan:\n${_asan_calls}")
endif()
```

In the `change_cpp` and `change_docs_only` expectations, replace the line `"ctest --preset release"` with these two:

```cmake
  "ctest --preset release --output-junit ${_repo}/build/native-release/Testing/junit-change.xml"
  "cmake -DMODE=report -DCTEST=ctest -DBUILD_DIR=${_repo}/build/native-release -DJUNIT=${_repo}/build/native-release/Testing/junit-change.xml -P ${_repo}/native/cmake/TestDurationCheck.cmake"
```

- [ ] **Step 2: Run it to verify it fails**

Run: `mise run test '^tooling_gate$'`
Expected: FAIL on `gate_native` (no report call), `gate_full`, `change_cpp` and `change_docs_only`. `gate_asan_no_report` passes already.

- [ ] **Step 3: Change the launchers**

`tools/run_tests.sh`: add this function above `run_native_gate()`:

```bash
# report_durations <junit> — print a warning for each test whose measured
# time left the range of its declared seconds
# (native/cmake/TestDurationCheck.cmake). Warnings never fail the gate.
# Release only: sanitizer and coverage builds are slower by design, and the
# declared seconds are release numbers.
report_durations() {
  [[ "$preset" == release && -f "$1" ]] || return 0
  cmake -DMODE=report -DCTEST=ctest -DBUILD_DIR="$build_dir" -DJUNIT="$1" \
    -P "$REPO_ROOT/native/cmake/TestDurationCheck.cmake"
}
```

In `run_native_gate`, right after the `stage ctest run_ordinary_leg ctest ...` command and inside the same `if wanted ctest` block:

```bash
    report_durations "$build_dir/Testing/junit.xml"
```

In the `full)` branch, the loop becomes:

```bash
    for label in slow package; do
      if wanted "$label"; then
        stage "$label" ctest --test-dir "$build_dir" -L "$label" --no-tests=error \
          --output-on-failure --output-junit "$build_dir/Testing/junit-$label.xml"
        report_durations "$build_dir/Testing/junit-$label.xml"
      fi
    done
```

`tools/check_change.sh`: replace `ctest --preset release` with:

```bash
junit="$build_dir/Testing/junit-change.xml"
ctest --preset release --output-junit "$junit"
# Warnings only: a test outside its declared time is reported, not failed.
cmake -DMODE=report -DCTEST=ctest -DBUILD_DIR="$build_dir" -DJUNIT="$junit" \
  -P "$REPO_ROOT/native/cmake/TestDurationCheck.cmake"
```

Fix the stale closing message of `check_change.sh`, which still says the native gate runs before every merge. AGENTS.md level 2 now makes that conditional:

```bash
printf 'check_change: passed in %ss (level 1; before a merge, AGENTS.md level 2 says whether the native gate is needed)\n' \
  "$((SECONDS - started))"
```

Also update the header comment line `# Default base: main. Not a substitute for ...` to: `# Default base: main. Before a merge, AGENTS.md level 2 decides whether tools/run_tests.sh native is needed.`

`mise.toml`, `[tasks.test]` `run`:

```toml
run = '''
cmake --build build/native-release
ctest --test-dir build/native-release -R "${usage_pattern?}" --no-tests=error --output-on-failure --output-junit "$PWD/build/native-release/Testing/junit-task.xml"
cmake -DMODE=report -DCTEST=ctest -DBUILD_DIR="$PWD/build/native-release" -DJUNIT="$PWD/build/native-release/Testing/junit-task.xml" -P native/cmake/TestDurationCheck.cmake
'''
```

- [ ] **Step 4: Run the contracts and the scripts sweep**

Run: `mise run test '^tooling_gate$|^tooling_test_durations$'`
Expected: PASS.

Run: `build/native-release/bin/bike-native-checks --kind scripts --build build/native-release`
Expected: pass (shellcheck on the changed `tools/*.sh`).

Check that a failing ctest still stops the mise task before the report. A wrong report exit status must not hide a red test:

Run: `mise run test '^no_such_test_zzz$'; echo "exit=$?"`
Expected: `No tests were found!!!`, `exit=` non-zero, and no `test durations:` line. If the report line appears, mise is not running the script with errexit. Then prefix the `run` script with `set -e` and repeat.

- [ ] **Step 5: Stage A level 1 and commit**

Until Task 5 no real test is declared, and the report skips undeclared tests (`NOT DEFINED _declared_<name>`). It therefore prints `0 tests checked` and no warnings.

Run: `mise run check main`
Expected: `check_change: passed`, with a `test durations: 0 tests checked, 0 warning(s)` line.

```bash
git add tools/run_tests.sh tools/check_change.sh mise.toml native/tests/tooling/tooling_gate.cmake
git commit -m "build(tests): write JUnit for every ctest leg and report test durations"
```

---

### Task 4: The measurement run

**Files:** none committed. Output goes to `.work/durations/` (`.work/` is in `.gitignore`).

This is the run D3 asks for. It mirrors the gate's legs exactly, so each number is measured in the conditions where its test runs today: the ordinary leg at `-j8` under `MallocScribble`, and the slow and package legs serially.

- [ ] **Step 1: Run the three legs**

```bash
mkdir -p .work/durations
cmake --build build/native-release
( export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
  ctest --test-dir build/native-release -LE 'slow|package' -j8 --no-tests=error \
    --output-on-failure --output-junit "$PWD/.work/durations/ordinary.xml" )
ctest --test-dir build/native-release -L slow --no-tests=error --output-on-failure \
  --output-junit "$PWD/.work/durations/slow.xml"
ctest --test-dir build/native-release -L package --no-tests=error --output-on-failure \
  --output-junit "$PWD/.work/durations/package.xml"
date +%F > .work/durations/date.txt
```

Expected: every test passes; about 1 + 13.6 + 0.75 min. A failure here is a real failure on this code. Fix it or stop and report it; do not record a time for a red test.

- [ ] **Step 2: One table: name, seconds, stage, where it is registered**

```bash
for leg in ordinary slow package; do
  grep -o '<testcase name="[^"]*" classname="[^"]*" time="[^"]*" status="run"' \
    ".work/durations/$leg.xml" |
    sed -E "s/.*name=\"([^\"]*)\".*time=\"([^\"]*)\".*/\1\t\2\t$leg/"
done | sort > .work/durations/measured.tsv
wc -l .work/durations/measured.tsv      # must equal: ctest --test-dir build/native-release -N | tail -1
```

Then write `.work/durations/where.cmake` and run it. It prints, for every test, its registration call chain from CTest's own backtrace graph (`file:line < file:line`; the outermost frame is the line to edit):

```cmake
execute_process(COMMAND ctest --test-dir "${BUILD}" --show-only=json-v1 OUTPUT_VARIABLE j)
string(JSON files GET "${j}" backtraceGraph files)
string(JSON nodes GET "${j}" backtraceGraph nodes)
string(JSON tests GET "${j}" tests)
string(JSON n LENGTH "${tests}")
math(EXPR last "${n} - 1")
foreach(i RANGE ${last})
  string(JSON name GET "${tests}" ${i} name)
  string(JSON node GET "${tests}" ${i} backtrace)
  set(chain "")
  while(NOT node STREQUAL "")
    string(JSON f GET "${nodes}" ${node} file)
    string(JSON file GET "${files}" ${f})
    string(JSON line ERROR_VARIABLE noline GET "${nodes}" ${node} line)
    list(APPEND chain "${file}:${line}")
    string(JSON node ERROR_VARIABLE top GET "${nodes}" ${node} parent)
    if(top)
      set(node "")
    endif()
  endwhile()
  list(JOIN chain " < " chain)
  message("${name}\t${chain}")
endforeach()
```

```bash
cmake -DBUILD="$PWD/build/native-release" -P .work/durations/where.cmake 2>&1 |
  sed "s|$PWD/||g" | sort > .work/durations/where.tsv
join -t $'\t' .work/durations/measured.tsv .work/durations/where.tsv > .work/durations/table.tsv
sort -t $'\t' -k2 -g -r .work/durations/table.tsv | head -25
```

Expected: one row per test (`name  seconds  leg  chain`). Compare the top of the list with "Starting data"; a test that changed by more than 2× since then is worth one line in the task log.

---

### Task 5: Declare every test

**Files:** every registration site in `.work/durations/where.tsv`, `native/tests/reference/CMakeLists.txt` (`bike_add_reference_test`), `native/cmake/Checks.cmake` (`test_duration_audit`).

**Interfaces:**
- Consumes: `bike_test_seconds` (Task 1), `.work/durations/table.tsv` (Task 4).
- Produces: `bike_add_reference_test(<name> [ENGINE] [SERIAL_STAGE] SECONDS <n> SOURCES … [LIBRARIES …] [LABELS …])`, where `SECONDS` is required. CTest test `test_duration_audit` (ordinary set).

Rules for every test:
- Declared seconds = the measured time rounded up to a whole second, at least 1.
- Comment on the line right above the registration call: `# measured <t> s, <leg> stage, <date>`, with `<t>` as printed in the table (two decimals are enough), `<leg>` one of `ordinary`/`slow`/`package`, and `<date>` from `.work/durations/date.txt`. Example: `# measured 39.19 s, slow stage, 2026-11-05`.
- A test registered in a loop (`foreach`) gets one declaration inside the loop: the maximum over the loop's tests, with the comment `# measured up to <t> s (<slowest case>), <leg> stage, <date>`.
- Tests registered only in other presets (`sanitizer_negative_control` and `rtsan_negative_control` in `Sanitizers.cmake`, `coverage_smoke` in `Coverage.cmake`) are not in the release table. Declare them `1` with the comment `# not measured: registered only in the <asan|rtsan|coverage> preset; 1 s until it is`. Their real time is far below the 120 s floor, so the derived TIMEOUT cannot fail them.
- Remove every hand-written `slow` from `LABELS` and every `TIMEOUT` from `set_tests_properties` of a test (list in "Starting data"). The helper refuses to run while they remain, so nothing can be missed silently.
- `SERIAL_STAGE` only on `reference.realtime_gate.*` (Global Constraints). Before finishing, confirm nothing else asserts on wall-clock speed:
  `grep -lE 'real_time_factor|median_real_time_factor' native/tests/reference/test_*.cpp`. Every file listed must belong to `realtime_gate`, or to a test whose assertion is only on a fake or injected clock, as with `test_measure_realtime.cpp`. Read each hit before deciding.

- [ ] **Step 1: The reference helper takes `SECONDS`**

In `native/tests/reference/CMakeLists.txt`:

```cmake
function(bike_add_reference_test name)
  cmake_parse_arguments(REF "ENGINE;SERIAL_STAGE" "SECONDS" "SOURCES;LIBRARIES;LABELS" ${ARGN})
  if(REF_UNPARSED_ARGUMENTS OR NOT REF_SOURCES OR NOT REF_SECONDS)
    message(FATAL_ERROR
      "bike_add_reference_test(${name}): invalid arguments, no sources or no SECONDS")
  endif()
  set(_target "reference.${name}")
  add_executable(${_target} ${REF_SOURCES})
  target_link_libraries(${_target} PRIVATE bike_golden_test ${REF_LIBRARIES})
  target_include_directories(${_target} PRIVATE "${CMAKE_SOURCE_DIR}/native/src")
  if(REF_ENGINE)
    bike_startup_target_policy(${_target} "${CMAKE_SOURCE_DIR}/native")
    bike_rewrite_mujoco_load_name(${_target} "${BIKE_DEP_mujoco_LIBRARY}")
  else()
    bike_apply_native_policy(${_target})
  endif()
  add_test(NAME ${_target} COMMAND ${_target})
  set_tests_properties(${_target} PROPERTIES LABELS "reference;${REF_LABELS}"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_target}>")
  set(_serial_stage "")
  if(REF_SERIAL_STAGE)
    set(_serial_stage SERIAL_STAGE)
  endif()
  bike_test_seconds(${_target} ${REF_SECONDS} ${_serial_stage})
endfunction()
```

From here until Step 3 is done, configure fails on the first call without `SECONDS`. That is the to-do list.

- [ ] **Step 2: Declare file by file**

Go through `.work/durations/where.tsv` grouped by the outermost file. For each `bike_add_reference_test` call, add `SECONDS <n>`, drop `slow` from its `LABELS`, and put the comment above it:

```cmake
# measured 39.19 s, slow stage, 2026-11-05
bike_add_reference_test(native_rider_allocation ENGINE SECONDS 40
  SOURCES test_native_rider_allocation.cpp LIBRARIES bike_rider_math
  LABELS t52 rider)
```

(The `SOURCES`/`LIBRARIES`/`LABELS` above illustrate the shape; keep each call's own lists.)

For each plain `add_test`, call the helper after the test's last `set_tests_properties`, with the comment above the `add_test`:

```cmake
# measured 323.72 s, slow stage, 2026-11-05
add_test(NAME tooling_checks
  COMMAND "${CMAKE_COMMAND}" ...)
set_tests_properties(tooling_checks PROPERTIES LABELS "tooling")
bike_test_seconds(tooling_checks 324)
```

The realtime gate loop (`native/tests/reference/Phase3Ports.cmake`):

```cmake
foreach(_realtime_gate_track IN ITEMS rough_uphill_savage rough_uphill_extreme)
  # measured up to 9.78 s (rough_uphill_savage), slow stage, 2026-11-05
  add_test(NAME "reference.realtime_gate.${_realtime_gate_track}"
    COMMAND reference.realtime_gate
      "$<TARGET_FILE:bike-measure-realtime>"
      "${CMAKE_SOURCE_DIR}/examples/research/${_realtime_gate_track}.toml"
      "${CMAKE_SOURCE_DIR}/examples/research/viewer_physics_welded.toml")
  set_tests_properties("reference.realtime_gate.${_realtime_gate_track}" PROPERTIES
    LABELS "reference;t53;runtime"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:reference.realtime_gate>")
  # Asserts median real-time factor ≥ 1: it must run alone, outside the
  # ordinary leg's allocator debugging.
  bike_test_seconds("reference.realtime_gate.${_realtime_gate_track}" 10 SERIAL_STAGE)
endforeach()
```

Note: the test's own `process.timeout_s = 900.` inside `test_realtime_gate.cpp` is the child process deadline, not the CTest property. The derived CTest TIMEOUT for 10 s is 120 s, and a full measurement takes about 10 s, so 120 s is still a 12× margin. If the user wants the old 900 s, it needs `bike_test_seconds` to accept a floor; ask before adding one.

After each file: `cmake --preset release` (≈11 s) and fix whatever the helper rejects.

- [ ] **Step 3: Declare the contract tests and register the audit**

`tooling_test_durations` and `tooling_gate` get their measured times like any other test. Then add the real-tree audit to `native/cmake/Checks.cmake`, after `tooling_test_durations`:

```cmake
# --- test_duration_audit: every test of this build declares its time --------
# Reads `ctest --show-only=json-v1` of this build dir (no test runs), so it
# stays in the ordinary set and catches an undeclared test in every stage.
if(BUILD_TESTING)
  # not measured yet: added after the measurement run; ~1 s (one json query)
  add_test(NAME test_duration_audit
    COMMAND "${CMAKE_COMMAND}" -DMODE=audit
      "-DCTEST=${CMAKE_CTEST_COMMAND}" "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
      -P "${_checks_native_dir}/cmake/TestDurationCheck.cmake")
  set_tests_properties(test_duration_audit PROPERTIES LABELS "tooling")
  bike_test_seconds(test_duration_audit 1)
endif()
```

Update its comment with the measured time after Step 5's run.

- [ ] **Step 4: Audit and labels**

```bash
cmake --preset release
mise run test '^test_duration_audit$'
ctest --test-dir build/native-release -N -L slow | tail -1
```

Expected: the audit passes with `<N> tests declare their time`, where N is the line count of `measured.tsv` plus 1 for the audit itself. The `-L slow` count is the number of table rows ≥ 30 s plus 2 (realtime_gate). From "Starting data" that is 7 if nothing moved across 30 s.

- [ ] **Step 5: The moved tests in their new conditions**

The tests that lost `slow` now run in the ordinary leg: `-j8`, under `MallocScribble`, next to other tests. Run them there three times to catch both a failure and a time far from what was measured serially:

```bash
moved=$(awk -F'\t' '$3 == "slow" && $2 < 30 {print $1}' .work/durations/table.tsv |
  grep -v '^reference\.realtime_gate\.' | sed 's/\./\\./g' | paste -sd'|' -)
cmake --build build/native-release
( export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
  ctest --test-dir build/native-release -R "^($moved)\$" -j8 --repeat until-fail:3 \
    --no-tests=error --output-on-failure --output-junit "$PWD/.work/durations/moved.xml" )
cmake -DMODE=report -DCTEST=ctest -DBUILD_DIR="$PWD/build/native-release" \
  -DJUNIT="$PWD/.work/durations/moved.xml" -P native/cmake/TestDurationCheck.cmake
```

Expected: all pass three times. A test that fails only here (allocator debugging or parallel neighbours) is a real bug. Diagnose it (systematic-debugging) and do not put `slow` back to hide it. A warning means the serial number does not hold in the ordinary leg: take that test's time from `moved.xml`, update its seconds and its comment (`ordinary stage`), and re-run the report.

- [ ] **Step 6: Stage B level 1 and commit**

Run: `mise run check main`
Expected: passed; `test durations: <n> tests checked, 0 warning(s)`. A warning here is handled as in Step 5.

```bash
git add -u native CMakeLists.txt
git commit -m "test: declare the measured time of every test"
```

---

### Task 6: Documentation

**Files:**
- Modify: `AGENTS.md` ("Test budget"), `docs/TESTING.md`

- [ ] **Step 1: AGENTS.md**

In "### Test budget", after the "Inside a task" bullet, add:

```markdown
- **Test times are declared.** Every test registration carries its
  measured time: `bike_test_seconds(<test> <seconds>)` after its last
  `set_tests_properties`, or `SECONDS <seconds>` in
  `bike_add_reference_test`, with `# measured <t> s, <stage> stage, <date>`
  above it. `slow` (≥30 s), `TIMEOUT` and `COST` are derived from it;
  never write them by hand (`test_duration_audit` fails). A new test is
  measured once with `mise run test '<its name>'` before it is declared.
  `warning: test … took …` after a run means the declaration is stale:
  re-measure that test and update the number and comment; it is not a
  failure.
```

Keep the existing wording of the other bullets.

- [ ] **Step 2: docs/TESTING.md**

After the section that lists the profiles (the lines around `bash tools/run_tests.sh full   # native plus the explicit slow and package label gates`), add:

```markdown
### Test durations

Each test declares its measured time where it is registered
(`native/cmake/TestDurations.cmake`):

- `slow` is set for 30 s or more, or for `SERIAL_STAGE` tests (they assert
  on wall-clock speed and must run alone, outside the ordinary leg's
  allocator debugging). The `slow` stage runs serially.
- `TIMEOUT` is `max(120, 5 × seconds)`. `COST` is the seconds, so
  `ctest -j` starts the longest tests first.
- `test_duration_audit` (ordinary set) fails when a test has no declared
  time or a hand-written `slow`/`TIMEOUT`/`COST`.
- Every CTest leg of `run_tests.sh`, `check_change.sh` and `mise run test`
  writes JUnit under `build/native-release/Testing/` and then prints a
  warning for each test that ran longer than `2 × seconds + 5 s`, or a slow
  test that ran under a third of its seconds. Warnings never fail a run.
  Release preset only.

To re-measure: run the legs as Task 4 of
`docs/superpowers/plans/2026-11-04-test-durations.md` does (or take the
times from the JUnit files of the last full gate on the same code), round
up, and update the number and the comment above the registration.
```

- [ ] **Step 3: Context sweep, stage C level 1, commit**

Run: `build/native-release/bin/bike-native-checks --kind context --build build/native-release`
Expected: pass.

Run: `mise run check main`
Expected: passed.

```bash
git add AGENTS.md docs/TESTING.md
git commit -m "docs(testing): declare, derive and re-measure test durations"
```

---

### Task 7: Final check and the measured gate cost

**Files:**
- Modify: `AGENTS.md` (level 3 measurement), `.work/durations/` (not committed)

- [ ] **Step 1: One full gate**

Run: `bash tools/run_tests.sh full 2>&1 | tee .work/durations/final-gate.log`
Expected: exit 0. `test durations: … 0 warning(s)` after the ordinary, slow and package legs. The stage table shows `slow` about 690 s instead of 815 s, and `ctest` about 60 s instead of 45 s.

If a warning appears: update that test's seconds and comment from `build/native-release/Testing/junit*.xml`. If its slow label flips (it crossed 30 s), run that one test in its new leg with `mise run test '^<name>$'`. No new full gate is needed for a number-only change: the audit (`mise run test '^test_duration_audit$'`) proves the derived values, and the test itself already passed in this gate.

- [ ] **Step 2: Record the new gate cost**

In `AGENTS.md` level 3, replace `measured 1506 s on 2026-11-04, of which slow = 815 s` with the total and slow seconds from this gate's stage table and today's date. Update the level-1 "(≤5 min)" only if `check_change` measured over 5 min (it will not).

- [ ] **Step 3: Commit, then ask about the merge**

```bash
git add AGENTS.md
git commit -m "docs(agents): record the gate cost after the test-duration split"
```

Report to the user: what ran (Task 4 legs, Task 5 repeats, three level-1 runs, the full gate), the slow and ordinary stage times before and after, every test that changed stage, and any warning that was resolved. Merge into `main` only when the user says so. This branch touches CMake files and `tools/*.sh`, so AGENTS.md level 2 applies, and this full gate covers it.

---

## Self-review notes

- D1 → `BIKE_SLOW_TEST_SECONDS 30`, audited. D2 → report mode, exit 0, tested (`report`). D3 → Task 4 (measure) + Task 5 (mark). D4 → number argument + comment rules in Task 5; staleness caught by the report. D5 → `bike_test_timeout`, tested (`derive`, `audit_bad`).
- Every new name used later is defined earlier: `bike_test_seconds`/`bike_test_timeout` (Task 1), `TestDurationCheck.cmake` and its `MODE`/`TESTS_JSON`/`CTEST`/`BUILD_DIR`/`JUNIT` inputs (Task 2), the JUnit file names (Task 3), `.work/durations/{measured,where,table}.tsv` (Task 4).
- Open risk, stated rather than assumed: the derived 120 s TIMEOUT for `reference.realtime_gate.*` replaces 900 s (Task 5 Step 2 note). The ordinary leg grows by the 40 moved tests; Task 5 Step 5 measures it before the full gate does.
