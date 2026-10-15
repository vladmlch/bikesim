# Migration contract-test harness and per-task regression executables (T02).
# One executable and one CTest name per task: migration_TNN <- test_tNN.cpp.
# Registered inside the legacy native build until T03's standalone root
# build; targets inherit this directory scope's warning/hardening/sanitizer
# context through the _native_contract_* variables captured by
# native/CMakeLists.txt.

# The fixture root is computed from this directory up to the repository root
# (native/tests/migration/../../..) — never from the process working
# directory or a nested PROJECT_SOURCE_DIR (which is native/ inside this
# legacy build).
get_filename_component(_migration_repo_root
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(_migration_fixture_root "${_migration_repo_root}/tests/fixtures/migration")
if(NOT IS_DIRECTORY "${_migration_fixture_root}")
  message(FATAL_ERROR
    "migration fixture tree is missing: ${_migration_fixture_root}")
endif()
# The root is embedded into the harness TU as a compile definition; quotes
# or backslashes would break the -D flag's quoting contract.
if(_migration_fixture_root MATCHES "[\"'\\\\]")
  message(FATAL_ERROR
    "fixture root must not contain quotes or backslashes: ${_migration_fixture_root}")
endif()

# T05 oracle-manifest path: the harness TU also owns BIKE_ORACLE_MANIFEST —
# same compile-definition contract as BIKE_FIXTURE_ROOT, verified present at
# configure time.
set(_migration_oracle_manifest "${_migration_repo_root}/migration/oracle-manifest.json")
if(NOT EXISTS "${_migration_oracle_manifest}")
  message(FATAL_ERROR
    "oracle manifest is missing: ${_migration_oracle_manifest}")
endif()
if(_migration_oracle_manifest MATCHES "[\"'\\\\]")
  message(FATAL_ERROR
    "oracle manifest path must not contain quotes or backslashes: ${_migration_oracle_manifest}")
endif()

# Standalone harness: stdlib-only plus the owned bike::io boundary (strict
# JSON + SHA-256 for the T05 manifest layer) — no MuJoCo/Python/nanobind
# includes. The support TU owns BIKE_FIXTURE_ROOT and BIKE_ORACLE_MANIFEST
# privately so test targets and the standalone-header sweep never see them.
add_library(bike_migration_test STATIC
  ../support/migration_test.cpp
  ../support/migration_test.hpp)
set_target_properties(bike_migration_test PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_migration_test PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_migration_test PRIVATE ${_native_contract_compile_definitions})
endif()
target_compile_definitions(bike_migration_test PRIVATE
  "BIKE_FIXTURE_ROOT=\"${_migration_fixture_root}\""
  "BIKE_ORACLE_MANIFEST=\"${_migration_oracle_manifest}\"")
get_filename_component(_migration_test_include_root
  "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(bike_migration_test PUBLIC
  "${_migration_test_include_root}")
target_include_directories(bike_migration_test SYSTEM PRIVATE
  "${_migration_repo_root}/native/src")
target_link_libraries(bike_migration_test PUBLIC bike_io)
# In the standalone root build the _native_contract_* context belongs to the
# legacy binding scope and is unset here — apply the shared policy directly.
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(bike_migration_test)
endif()

# allocation_faults.cpp is the throwing-allocation override TU: it defines
# replacement operator new/delete plus the counting seam, so it must land
# in test binaries directly, never in a shipped library. Two consumers
# share it — native_contract_tests (legacy graph) and migration_T07 —
# and the ODR sweep compiles every compile-database entry once before a
# combined relocatable link, so per-target duplicate compiles would
# collide. One OBJECT library = one compile entry; each executable gets
# the same object through $<TARGET_OBJECTS:...> exactly like a listed
# source.
add_library(bike_test_allocation_faults OBJECT ../allocation_faults.cpp)
set_target_properties(bike_test_allocation_faults PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_test_allocation_faults PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_test_allocation_faults PRIVATE ${_native_contract_compile_definitions})
endif()
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(bike_test_allocation_faults)
endif()

add_executable(migration_T02 test_t02.cpp)
set_target_properties(migration_T02 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T02 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T02 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T02 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T02 PRIVATE bike_migration_test)
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T02)
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T02 COMMAND migration_T02)
  # Fixtures must resolve from the compiled-in root regardless of cwd: the
  # test runs from the binary's own output directory, unrelated to the
  # fixture tree, and test_t02.cpp also chdirs to a second unrelated
  # directory mid-run.
  set_tests_properties(migration_T02 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T02>")
endif()

# T04 engine-boundary regression: links the shared pinned-interceptor
# library — bike_engine in the standalone root build, bike_native_engine in
# the legacy binding build — so both graphs exercise the identical
# engine_call.cpp TU. The src/MuJoCo include edges mirror the consuming
# targets' SYSTEM treatment in each graph.
add_executable(migration_T04 test_t04.cpp)
set_target_properties(migration_T04 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T04 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T04 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T04 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T04 PRIVATE bike_migration_test)
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T04)
  target_include_directories(migration_T04 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(migration_T04 PRIVATE bike_engine)
  bike_rewrite_mujoco_load_name(migration_T04
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
else()
  target_include_directories(migration_T04 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${MJ_DIR}/include")
  target_link_libraries(migration_T04 PRIVATE bike_native_engine)
  bike_rewrite_mujoco_load_name(migration_T04
    "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T04 COMMAND migration_T04)
  set_tests_properties(migration_T04 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T04>")
endif()

# T05 io-boundary regression: the strict lossless JSON/Value boundary,
# recording-tag codec, shared SHA-256 and manifest/provenance validation.
# Links bike_migration_test (which itself links bike_io) — identical
# implementation objects in both build graphs.
add_executable(migration_T05 test_t05.cpp)
set_target_properties(migration_T05 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T05 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T05 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T05 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T05 PRIVATE bike_migration_test)
target_include_directories(migration_T05 SYSTEM PRIVATE
  "${_migration_repo_root}/native/src")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T05)
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T05 COMMAND migration_T05)
  set_tests_properties(migration_T05 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T05>")
endif()

# T06 platform-primitives regression: bounded filesystem, subprocess,
# deferred-SIGINT shield and build provenance. Links bike_migration_test
# plus the owned bike_platform boundary — identical implementation objects
# in both build graphs.
add_executable(migration_T06 test_t06.cpp)
set_target_properties(migration_T06 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T06 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T06 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T06 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T06 PRIVATE bike_migration_test bike_platform)
target_include_directories(migration_T06 SYSTEM PRIVATE
  "${_migration_repo_root}/native/src")
# The packaging-axis expectation mirrors the bike_platform identity constants
# in this graph: the standalone root graph records its own BIKE_BUILD_*
# options; the standalone legacy graph always builds the transition
# extension (bindings=true, GUI off unless configured).
set(_t06_expect_bindings 1)
set(_t06_expect_gui 0)
if(BIKE_ROOT_BUILD)
  set(_t06_expect_bindings 0)
  if(BIKE_BUILD_PYTHON_BINDINGS)
    set(_t06_expect_bindings 1)
  endif()
endif()
if(BIKE_BUILD_GUI)
  set(_t06_expect_gui 1)
endif()
target_compile_definitions(migration_T06 PRIVATE
  "BIKE_T06_EXPECT_BINDINGS=${_t06_expect_bindings}"
  "BIKE_T06_EXPECT_GUI=${_t06_expect_gui}")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T06)
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T06 COMMAND migration_T06)
  set_tests_properties(migration_T06 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T06>")
endif()

# T07 writer-config boundary regression: the typed WriterConfig decode
# (writer_config.cpp) plus the Python-free Stepper construction path —
# the shared core object set (bike_core / bike_native_contract_core) and
# the pinned engine interceptor, the identical TUs the transition
# extension links. No Python/nanobind include enters this target in
# either graph: compiling test_t07.cpp against stepper.hpp +
# writer_config.hpp is the include-closure contract itself.
# The allocation-override object lands via $<TARGET_OBJECTS:...> — see the
# bike_test_allocation_faults comment above for why the TU compiles once.
add_executable(migration_T07 test_t07.cpp
  $<TARGET_OBJECTS:bike_test_allocation_faults>)
set_target_properties(migration_T07 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T07 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T07 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T07 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T07 PRIVATE bike_migration_test
  "-framework Accelerate")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T07)
  target_include_directories(migration_T07 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(migration_T07 PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(migration_T07
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
else()
  target_include_directories(migration_T07 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${MJ_DIR}/include")
  target_link_libraries(migration_T07 PRIVATE
    bike_native_contract_core bike_native_engine bike_rider_math)
  bike_rewrite_mujoco_load_name(migration_T07
    "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T07 COMMAND migration_T07)
  set_tests_properties(migration_T07 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T07>")
endif()

# T08 state-wire codec regression: the owned bike::io::Value codecs for
# the rider and drivetrain state documents (drivetrain/state_codec.cpp,
# rider/state_codec.cpp) — the shared core object set (bike_core /
# bike_native_contract_core) plus the pinned engine interceptor for the
# welded_pinned Stepper that supplies the contacts topology gate, plus
# bike_rider_math: the OBJECT core now carries rider/state_codec.o whose
# spindle:: validator refs need the single-compile library at final link. No
# Python/nanobind include enters this target in either graph: compiling
# test_t08.cpp against the codec headers is the include-closure contract
# itself.
add_executable(migration_T08 test_t08.cpp)
set_target_properties(migration_T08 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T08 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T08 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T08 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T08 PRIVATE bike_migration_test
  "-framework Accelerate")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T08)
  target_include_directories(migration_T08 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(migration_T08 PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(migration_T08
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
else()
  target_include_directories(migration_T08 SYSTEM PRIVATE
    "${_migration_repo_root}/native/src" "${MJ_DIR}/include")
  target_link_libraries(migration_T08 PRIVATE
    bike_native_contract_core bike_native_engine bike_rider_math)
  bike_rewrite_mujoco_load_name(migration_T08
    "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T08 COMMAND migration_T08)
  set_tests_properties(migration_T08 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T08>")
endif()

add_executable(migration_T12 test_t12.cpp)
set_target_properties(migration_T12 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T12 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T12 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T12 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T12 PRIVATE bike_migration_test
  bike_configuration)
target_include_directories(migration_T12 SYSTEM PRIVATE
  "${_migration_repo_root}/native/src")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T12)
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T12 COMMAND migration_T12)
  set_tests_properties(migration_T12 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T12>")
endif()

# T13 configuration-loading regression: TOML/JSON decode into the typed
# records, _merge precedence + resolve_physics, referenced-asset resolution
# against the declaring file, and the candidate TOML writer — the same
# bike_configuration objects in both build graphs.
add_executable(migration_T13 test_t13.cpp)
set_target_properties(migration_T13 PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(migration_T13 PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(migration_T13 PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_link_options)
  target_link_options(migration_T13 PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(migration_T13 PRIVATE bike_migration_test
  bike_configuration bike_platform)
target_include_directories(migration_T13 SYSTEM PRIVATE
  "${_migration_repo_root}/native/src")
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(migration_T13)
endif()

if(BUILD_TESTING)
  add_test(NAME migration_T13 COMMAND migration_T13)
  set_tests_properties(migration_T13 PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:migration_T13>")
endif()

# T03 build-contract regression: a CTest script test running the produced
# configure/build/ctest command contract plus a real custom build target —
# not a fake empty C++ executable. The dependency module anchors its
# BIKE_DEPENDENCY_ROOT/LOCK defaults at the repository root, so both this
# build and the legacy binding build hand the script the same .deps tree.
bike_register_migration_t03(
  "${_migration_repo_root}" "${BIKE_DEPENDENCY_ROOT}" "${BIKE_DEPENDENCY_LOCK}")
