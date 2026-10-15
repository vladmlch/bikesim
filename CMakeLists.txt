cmake_minimum_required(VERSION 3.24)

# Standalone repository-root build (migration task T03).
#
# This is the FIRST build entry point that never finds Python, wheel, or
# venv directories. Every native dependency is an explicitly acquired tree
# under .deps/, verified against native/dependencies.lock.json at configure
# time; configure, build, and install run no dependency-code-generation
# Python and make no network calls.
#
#   cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release \
#     -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF \
#     -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
#   cmake --build build/native-release
#   ctest --test-dir build/native-release --output-on-failure
#
# The legacy binding build stays where it was (cmake -S native -B ...): this
# file only takes over when the repository ROOT is the -S directory. Setting
# BIKE_BUILD_PYTHON_BINDINGS=ON reuses the existing native/ project as a
# subdirectory so the optional extension is compiled from the same core
# objects the standalone tests exercise; that transition mode still needs
# the native dev group's Python+nanobind by design.

# Modules only declare functions/macros — safe to include pre-project().
# SDK resolution MUST precede project(): CMake runs its compiler link check
# inside project(), so an incompatible ambient SDK must be replaced (or an
# incompatible explicit SDK rejected with the module's precise diagnostic)
# before that check runs — identical ordering to the legacy native build.
include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/CompilerPolicy.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/Dependencies.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/Provenance.cmake")

if(APPLE)
  bike_resolve_macos_sdk()
endif()

project(bike_sim CXX)

option(BIKE_BUILD_PYTHON_BINDINGS "Build the nanobind extension (requires the Python native dev group)" OFF)
option(BIKE_BUILD_GUI "Build GLFW/Cairo GUI targets (not wired yet; OFF = no-op)" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

get_filename_component(BIKE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(BIKE_ROOT_BUILD ON)

# Acquisition + verification gate: missing/tampered/wrong-version .deps is a
# configure-time failure before any first-party target can build.
bike_dependencies_resolve()

if(APPLE)
  bike_verify_sdk_compat()
endif()

# Same libc++ hardening contract as the legacy binding build: EXTENSIVE by
# default, FAST as an explicit comparison option — probed, not assumed.
set(BIKE_LIBCPP_HARDENING "EXTENSIVE" CACHE STRING "libc++ hardening mode")
set_property(CACHE BIKE_LIBCPP_HARDENING PROPERTY STRINGS EXTENSIVE FAST)
if(NOT BIKE_LIBCPP_HARDENING STREQUAL "EXTENSIVE" AND NOT BIKE_LIBCPP_HARDENING STREQUAL "FAST")
  message(FATAL_ERROR "BIKE_LIBCPP_HARDENING must be EXTENSIVE or FAST, got '${BIKE_LIBCPP_HARDENING}'")
endif()
bike_detect_libcpp_hardening()

# Same instrumentation knobs the legacy build exposes; the shared policy
# consumes them per target.
option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
option(NATIVE_RTSAN "RealtimeSanitizer build (brew clang only)" OFF)
option(NATIVE_COVERAGE "llvm-cov instrumented build" OFF)

include(CTest)

set(_bike_native_dir "${CMAKE_CURRENT_SOURCE_DIR}/native")
set(_bike_native_tests_dir "${CMAKE_CURRENT_SOURCE_DIR}/native/tests")

# ---------------------------------------------------------------------------
# First-party targets in the standalone graph. Only already Python-free
# translation units are compiled here — the binding TUs and the headers they
# require are deliberately out of scope until later migration tasks.

# bike_core: the Python-free core object library — the identical TU set the
# binding build shares as bike_native_contract_core, so the optional
# extension and the standalone tests exercise the same core objects.
add_library(bike_core OBJECT
  "${_bike_native_dir}/src/drivetrain/pedaling.cpp"
  "${_bike_native_dir}/src/drivetrain/chain.cpp"
  "${_bike_native_dir}/src/drivetrain/transmission.cpp"
  "${_bike_native_dir}/src/drivetrain/shifting.cpp"
  "${_bike_native_dir}/src/drivetrain/motor.cpp"
  "${_bike_native_dir}/src/drivetrain/freehub.cpp"
  "${_bike_native_dir}/src/writers/drivetrain.cpp"
  "${_bike_native_dir}/src/writers/cruise.cpp"
  "${_bike_native_dir}/src/tyre/profile.cpp"
  "${_bike_native_dir}/src/tyre/distance.cpp"
  "${_bike_native_dir}/src/tyre/station.cpp"
  "${_bike_native_dir}/src/pneumatic/geometry.cpp"
  "${_bike_native_dir}/src/pneumatic/carcass.cpp"
  "${_bike_native_dir}/src/pneumatic/brush.cpp"
  "${_bike_native_dir}/src/pneumatic/model.cpp"
  "${_bike_native_dir}/src/rider/contact_math.cpp"
  "${_bike_native_dir}/src/rider/support_geometry.cpp"
  "${_bike_native_dir}/src/rider/least_squares.cpp"
  "${_bike_native_dir}/src/runtime/samples.cpp"
  "${_bike_native_dir}/src/runtime/wire.cpp"
  "${_bike_native_dir}/src/runtime/control.cpp"
  "${_bike_native_dir}/src/writers/suspension.cpp"
  "${_bike_native_dir}/src/writers/brake.cpp"
  "${_bike_native_dir}/src/writers/resistance.cpp"
  "${_bike_native_dir}/src/writers/rider_forces.cpp"
  "${_bike_native_dir}/src/writers/tire_common.cpp"
  "${_bike_native_dir}/src/writers/tire.cpp"
  "${_bike_native_dir}/src/writers/distributed_tire.cpp"
  "${_bike_native_dir}/src/writers/pneumatic_tire.cpp")
set_target_properties(bike_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_core)
target_include_directories(bike_core SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include"
  "${BIKE_DEP_rapidjson_ROOT}/include" "${BIKE_DEP_tomlplusplus_ROOT}/include"
  "${BIKE_DEP_eigen_ROOT}" "${BIKE_DEP_duckdb_ROOT}/include")

# bike_engine: the pinned error interceptor shared by contract tests and the
# extension (identical source/context to the legacy bike_native_engine).
add_library(bike_engine STATIC "${_bike_native_dir}/src/engine_call.cpp")
set_target_properties(bike_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_engine)
target_include_directories(bike_engine SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
target_link_libraries(bike_engine PUBLIC bike::mujoco)

if(BUILD_TESTING)
  # tyre/distance.cpp SignedProfile + tyre/station.cpp scalar station law:
  # the produced-command contract's focused executable pair.
  add_executable(native_distance_tests "${_bike_native_tests_dir}/test_distance.cpp")
  bike_target_strict_policy(native_distance_tests)
  target_include_directories(native_distance_tests SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(native_distance_tests PRIVATE bike_core bike_engine)
  target_link_libraries(native_distance_tests PRIVATE "-framework Accelerate")
  bike_rewrite_mujoco_load_name(native_distance_tests
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")

  add_executable(native_station_tests "${_bike_native_tests_dir}/test_station.cpp")
  bike_target_strict_policy(native_station_tests)
  target_include_directories(native_station_tests SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(native_station_tests PRIVATE bike_core bike_engine)
  target_link_libraries(native_station_tests PRIVATE "-framework Accelerate")
  bike_rewrite_mujoco_load_name(native_station_tests
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")

  add_test(NAME native_distance_tests COMMAND native_distance_tests)
  add_test(NAME native_station_tests COMMAND native_station_tests)
endif()

# The migration contract-test harness + task regressions (migration_T02 and
# the script-registered migration_T03; later tasks add their own). Added
# after the native executables exist: the migration_T03 build target depends
# on them.
add_subdirectory(native/tests/migration)

# ---------------------------------------------------------------------------
# Optional transition extension: the existing native/ project as a subdir.
# BIKE_ROOT_BUILD suppresses its duplicate test/graph targets and redirects
# its MuJoCo edge to the locked .deps copy so no .venv path enters the
# produced binary's load commands (the byte-identical copy was verified at
# configure time above). The binding TUs only compile in this mode — the
# Python discovery stays inside the transition, exactly as the standalone
# legacy configure does.
if(BIKE_BUILD_PYTHON_BINDINGS)
  add_subdirectory(native)
endif()

# ---------------------------------------------------------------------------
# Install + provenance: the install tree is self-contained — binaries +
# locked dependency dylibs + the provenance manifest.
if(BUILD_TESTING)
  install(TARGETS native_distance_tests native_station_tests
    RUNTIME DESTINATION bin)
endif()
install(FILES "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib"
  DESTINATION lib)
bike_write_dependency_provenance("${CMAKE_BINARY_DIR}/dependency_provenance.json")
install(FILES "${CMAKE_BINARY_DIR}/dependency_provenance.json"
  DESTINATION share/bike_sim)
install(FILES "${BIKE_DEPENDENCY_LOCK}" DESTINATION share/bike_sim
  RENAME dependencies.lock.json)
