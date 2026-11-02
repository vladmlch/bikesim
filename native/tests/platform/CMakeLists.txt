# platform domain contract tests.

# platform-primitives regression: bounded filesystem, subprocess,
# deferred-SIGINT shield and build provenance. Links bike_test_fixtures
# plus the owned bike_platform boundary — identical implementation objects
# in both build graphs.
add_executable(platform_primitives test_primitives.cpp)
set_target_properties(platform_primitives PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(platform_primitives PRIVATE bike_test_fixtures bike_platform)
target_include_directories(platform_primitives SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
# The packaging-axis expectation mirrors the bike_platform identity
# constants: bindings are permanently absent; the GUI bit reflects
# BIKE_BUILD_GUI.
set(_t06_expect_bindings 0)
set(_t06_expect_gui 0)
if(BIKE_BUILD_GUI)
  set(_t06_expect_gui 1)
endif()
target_compile_definitions(platform_primitives PRIVATE
  "BIKE_T06_EXPECT_BINDINGS=${_t06_expect_bindings}"
  "BIKE_T06_EXPECT_GUI=${_t06_expect_gui}")
  bike_apply_native_policy(platform_primitives)

if(BUILD_TESTING)
  add_test(NAME platform_primitives COMMAND platform_primitives)
  set_tests_properties(platform_primitives PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:platform_primitives>")
endif()

# build-contract regression: a CTest script test running the produced
# configure/build/ctest command contract plus a real custom build target —
# not a fake empty C++ executable. The dependency module anchors its
# BIKE_DEPENDENCY_ROOT/LOCK defaults at the repository root, so the build
# hands the script the same .deps tree it consumed at configure time.
bike_register_build_contract(
  "${_test_repo_root}" "${BIKE_DEPENDENCY_ROOT}" "${BIKE_DEPENDENCY_LOCK}")
