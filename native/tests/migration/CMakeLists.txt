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

# Standalone harness: stdlib-only, no MuJoCo/Python/nanobind includes. The
# support TU owns BIKE_FIXTURE_ROOT privately so test targets and the
# standalone-header sweep never see it.
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
  "BIKE_FIXTURE_ROOT=\"${_migration_fixture_root}\"")
get_filename_component(_migration_test_include_root
  "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(bike_migration_test PUBLIC
  "${_migration_test_include_root}")
# In the standalone root build the _native_contract_* context belongs to the
# legacy binding scope and is unset here — apply the shared policy directly.
if(BIKE_ROOT_BUILD)
  bike_target_strict_policy(bike_migration_test)
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

# T03 build-contract regression: a CTest script test running the produced
# configure/build/ctest command contract plus a real custom build target —
# not a fake empty C++ executable. The dependency module anchors its
# BIKE_DEPENDENCY_ROOT/LOCK defaults at the repository root, so both this
# build and the legacy binding build hand the script the same .deps tree.
bike_register_migration_t03(
  "${_migration_repo_root}" "${BIKE_DEPENDENCY_ROOT}" "${BIKE_DEPENDENCY_LOCK}")
