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
# T07 moved three more TUs into the shared pure set: stepper.cpp (the
# typed WriterConfig construction path), writers/rider_contacts.cpp and
# writer_config.cpp (the owned bike::io::Value decode). They carry no
# nanobind/Python edge — the binding-side callback recognizer stays in
# binding.cpp behind stepper_detail::foreign_time_callback_probe().
# T08 adds the owned Value state codecs (drivetrain/state_codec.cpp,
# rider/state_codec.cpp); the torque-curve/joint-envelope validators the
# rider codec calls stay in the single-compile bike_rider_math STATIC and
# reach core consumers through the link edge below (one TU per graph).
# T09 adds the owned ride runtime: runtime.cpp (NativeRideRuntime),
# bootstrap.cpp/bootstrap_io.cpp (the typed config/state decoders plus the
# capture-directory loader), and the previously module-hosted pure TUs the
# runtime links — the physical step, accounting, presentation, samples
# buffers, static brake and status faces, plus rider/intent.cpp and
# rider/spindle_controller.cpp (the typed control/controller cores the
# step consumes). The nanobind twins — *_wire.cpp, *_binding.cpp,
# runtime_binding.cpp — stay in the optional extension.
# T10 adds the owned research core: runtime/{sensors,programs,
# research_metrics,recording,research,research_io}.cpp — the deterministic
# sensor pipeline, research programs/truth metrics, the legacy recorder
# and NativeResearchRuntime plus its capture-directory bootstrap face. The
# adapter keeps only research_binding.cpp's nanobind parsing and boxing.
add_library(bike_core OBJECT
  "${_bike_native_dir}/src/stepper.cpp"
  "${_bike_native_dir}/src/writer_config.cpp"
  "${_bike_native_dir}/src/drivetrain/pedaling.cpp"
  "${_bike_native_dir}/src/drivetrain/chain.cpp"
  "${_bike_native_dir}/src/drivetrain/transmission.cpp"
  "${_bike_native_dir}/src/drivetrain/shifting.cpp"
  "${_bike_native_dir}/src/drivetrain/motor.cpp"
  "${_bike_native_dir}/src/drivetrain/freehub.cpp"
  "${_bike_native_dir}/src/drivetrain/state_codec.cpp"
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
  "${_bike_native_dir}/src/rider/attachment_wrench.cpp"
  "${_bike_native_dir}/src/rider/equality_reactions.cpp"
  "${_bike_native_dir}/src/rider/state_codec.cpp"
  "${_bike_native_dir}/src/rider/intent.cpp"
  "${_bike_native_dir}/src/rider/spindle_controller.cpp"
  "${_bike_native_dir}/src/runtime/samples.cpp"
  "${_bike_native_dir}/src/runtime/wire.cpp"
  "${_bike_native_dir}/src/runtime/control.cpp"
  "${_bike_native_dir}/src/runtime/accounting.cpp"
  "${_bike_native_dir}/src/runtime/period_buffer.cpp"
  "${_bike_native_dir}/src/runtime/presentation.cpp"
  "${_bike_native_dir}/src/runtime/status.cpp"
  "${_bike_native_dir}/src/runtime/static_brake.cpp"
  "${_bike_native_dir}/src/runtime/step.cpp"
  "${_bike_native_dir}/src/runtime/bootstrap.cpp"
  "${_bike_native_dir}/src/runtime/bootstrap_io.cpp"
  "${_bike_native_dir}/src/runtime/runtime.cpp"
  "${_bike_native_dir}/src/runtime/sensors.cpp"
  "${_bike_native_dir}/src/runtime/programs.cpp"
  "${_bike_native_dir}/src/runtime/research_metrics.cpp"
  "${_bike_native_dir}/src/runtime/recording.cpp"
  "${_bike_native_dir}/src/runtime/research.cpp"
  "${_bike_native_dir}/src/runtime/research_io.cpp"
  "${_bike_native_dir}/src/writers/suspension.cpp"
  "${_bike_native_dir}/src/writers/brake.cpp"
  "${_bike_native_dir}/src/writers/resistance.cpp"
  "${_bike_native_dir}/src/writers/rider_forces.cpp"
  "${_bike_native_dir}/src/writers/tire_common.cpp"
  "${_bike_native_dir}/src/writers/tire.cpp"
  "${_bike_native_dir}/src/writers/distributed_tire.cpp"
  "${_bike_native_dir}/src/writers/pneumatic_tire.cpp"
  "${_bike_native_dir}/src/writers/rider_contacts.cpp")
set_target_properties(bike_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_core)
target_include_directories(bike_core SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include"
  "${BIKE_DEP_rapidjson_ROOT}/include" "${BIKE_DEP_tomlplusplus_ROOT}/include"
  "${BIKE_DEP_eigen_ROOT}" "${BIKE_DEP_duckdb_ROOT}/include")

# bike_io: the T05 cold-path owned boundary — strict lossless JSON
# parse/serialize plus the recording-tag codec on the pinned RapidJSON
# headers, and the shared FIPS 180-4 SHA-256. bike_engine links it for the
# engine-image digest so engine_call.cpp carries no private copy; the
# migration harness and later config/recording consumers share the same
# objects.
add_library(bike_io STATIC
  "${_bike_native_dir}/src/io/json.cpp"
  "${_bike_native_dir}/src/io/hash.cpp"
  "${_bike_native_dir}/src/io/value.hpp"
  "${_bike_native_dir}/src/io/json.hpp"
  "${_bike_native_dir}/src/io/hash.hpp")
set_target_properties(bike_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_io)
target_include_directories(bike_io SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")

# bike_platform: the T06 bounded OS boundary — RAII file handles, exclusive
# no-follow creation, directory sync, argv subprocesses through posix_spawn
# with a dedicated child process group plus a concurrent bounded drain and
# kill/reap on deadline, the deferred-SIGINT shield, and the
# build/image-identity + clock provenance block. It carries no physics
# execution callbacks; its only first-party edge is bike_io's shared SHA-256.
add_library(bike_platform STATIC
  "${_bike_native_dir}/src/platform/files.cpp"
  "${_bike_native_dir}/src/platform/process.cpp"
  "${_bike_native_dir}/src/platform/signals.cpp"
  "${_bike_native_dir}/src/platform/provenance.cpp"
  "${_bike_native_dir}/src/platform/files.hpp"
  "${_bike_native_dir}/src/platform/process.hpp"
  "${_bike_native_dir}/src/platform/signals.hpp"
  "${_bike_native_dir}/src/platform/provenance.hpp")
set_target_properties(bike_platform PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_platform)
target_include_directories(bike_platform SYSTEM PRIVATE
  "${_bike_native_dir}/src")
target_link_libraries(bike_platform PRIVATE bike_io)

# Track-B build identity for bike_platform — identical convention to
# research_build_identity.hpp: the configured compiler/flags/SDK, the checked
# dependency lock and the first-party source manifest are hashed and compiled
# in at build time, never read from an editable JSON at runtime. Two hashes
# are file digests an auditor can reproduce byte-for-byte:
# migration/source-manifest.json (the T00 checked-in manifest) and the lock.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/migration/source-manifest.json")
  message(FATAL_ERROR
    "platform build identity requires migration/source-manifest.json")
endif()
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/migration/source-manifest.json"
  _platform_source_manifest_sha256)
file(SHA256 "${BIKE_DEPENDENCY_LOCK}" _platform_dependency_lock_sha256)
if(CMAKE_CONFIGURATION_TYPES)
  set(_platform_identity_configuration "$<CONFIG>")
else()
  set(_platform_identity_configuration "${CMAKE_BUILD_TYPE}")
endif()
set(_platform_effective_flags "${CMAKE_CXX_FLAGS}")
set(_platform_identity_configurations ${CMAKE_CONFIGURATION_TYPES}
  "${CMAKE_BUILD_TYPE}" Debug Release RelWithDebInfo MinSizeRel)
list(TRANSFORM _platform_identity_configurations TOUPPER)
list(REMOVE_DUPLICATES _platform_identity_configurations)
list(FILTER _platform_identity_configurations EXCLUDE REGEX "^$")
foreach(_configuration IN LISTS _platform_identity_configurations)
  string(TOUPPER "${_configuration}" _configuration_upper)
  string(APPEND _platform_effective_flags
    " $<$<CONFIG:${_configuration}>:${CMAKE_CXX_FLAGS_${_configuration_upper}}>")
endforeach()
string(APPEND _platform_effective_flags
  " $<JOIN:$<TARGET_GENEX_EVAL:bike_platform,$<TARGET_PROPERTY:bike_platform,COMPILE_OPTIONS>>, >")
# COMPILE_DEFINITIONS are part of the effective flag surface too — the
# hardening/fortify defines bike_target_strict_policy applies change the
# generated code, so two builds differing only in BIKE_LIBCPP_HARDENING must
# not record an identical identity. Map each stored definition back to its
# -D spelling; generator-expression entries still resolve per configuration
# inside file(GENERATE)'s target context.
get_target_property(_platform_compile_definitions bike_platform
  COMPILE_DEFINITIONS)
if(_platform_compile_definitions)
  foreach(_platform_definition IN LISTS _platform_compile_definitions)
    string(APPEND _platform_effective_flags " -D${_platform_definition}")
  endforeach()
endif()
set(_platform_sdk "")
if(APPLE)
  set(_platform_sdk "${CMAKE_OSX_SYSROOT}")
endif()
set(_platform_bindings_bool "false")
if(BIKE_BUILD_PYTHON_BINDINGS)
  set(_platform_bindings_bool "true")
endif()
set(_platform_gui_bool "false")
if(BIKE_BUILD_GUI)
  set(_platform_gui_bool "true")
endif()
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/platform_identity/$<CONFIG>/platform_build_identity.hpp"
  CONTENT "// Generated by CMake for this configuration; do not edit.\n#pragma once\nnamespace bike::platform::identity {\ninline constexpr char compiler_id[] = \"${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\";\ninline constexpr char numerical_flags[] = R\"identity(${_platform_effective_flags})identity\";\ninline constexpr char sdk[] = R\"identity(${_platform_sdk})identity\";\ninline constexpr char platform[] = \"${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}\";\ninline constexpr char build_type[] = \"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\";\ninline constexpr char dependency_lock_sha256[] = \"${_platform_dependency_lock_sha256}\";\ninline constexpr char source_manifest_sha256[] = \"${_platform_source_manifest_sha256}\";\ninline constexpr bool python_bindings = ${_platform_bindings_bool};\ninline constexpr bool gui = ${_platform_gui_bool};\n}\n"
  TARGET bike_platform)
target_include_directories(bike_platform PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/platform_identity/${_platform_identity_configuration}")

# bike_rider_math: the pure spindle rider-math TU — the rider_envelope.py /
# joint_strength.py helpers (anatomical_angle, joint_q_range, soft_edge_*,
# directional_capacity, envelope/curve validators) that configuration's
# rider_spec.hpp re-exports, plus the IK/leg-loop/crank-split set the
# optional extension's spindle controller compiles against. One STATIC
# library keeps the TU in a single compile-database entry per build graph —
# the ODR sweep relocatable-links every selected TU into one image, so a
# second compile of the same file would collide. bike_configuration resolves
# the re-exported envelope helpers through this edge; the optional
# transition extension links the same objects instead of listing the source
# on the module.
add_library(bike_rider_math STATIC
  "${_bike_native_dir}/src/rider/spindle_math.cpp")
set_target_properties(bike_rider_math PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_rider_math)
target_include_directories(bike_rider_math SYSTEM PRIVATE
  "${_bike_native_dir}/src")

# bike_configuration: the T12 configuration boundary — the ported Python
# dataclass records with declared defaults, every __post_init__/domain
# validator, the dataclass-introspection schema document and the
# native/schema.py resolved-wire validator + plain() coercion. The JSON
# profile loaders read bounded regular files through the T06 bike_platform
# edge; the schema/wire surfaces only touch bike_io's Value tree.
add_library(bike_configuration STATIC
  "${_bike_native_dir}/src/configuration/schema.cpp"
  "${_bike_native_dir}/src/configuration/validate.cpp"
  "${_bike_native_dir}/src/configuration/load.cpp"
  "${_bike_native_dir}/src/configuration/paths.cpp"
  "${_bike_native_dir}/src/configuration/profile_writer.cpp"
  "${_bike_native_dir}/src/configuration/fields.hpp"
  "${_bike_native_dir}/src/configuration/load.hpp"
  "${_bike_native_dir}/src/configuration/model_config.hpp"
  "${_bike_native_dir}/src/configuration/physical_config.hpp"
  "${_bike_native_dir}/src/configuration/rider_spec.hpp"
  "${_bike_native_dir}/src/configuration/suspension_spec.hpp"
  "${_bike_native_dir}/src/configuration/wire_decode.hpp")
set_target_properties(bike_configuration PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_configuration)
target_include_directories(bike_configuration SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")
target_link_libraries(bike_configuration PRIVATE bike_io bike_platform
  bike_rider_math bike::tomlplusplus)

# bike_geometry: the T14 geometry boundary — the specs.py records with
# declared defaults, hardpoints.py's headtube/axle/ground/trail/
# frame-angle calculators, validation.py's domain validator and
# cockpit.py's metre-domain saddle/seatpost/grip/pedal placement. One
# STATIC library keeps each TU in a single compile-database entry per
# build graph (the ODR sweep relocatable-links every selected TU into
# one image). The Accelerate edge is PUBLIC: specs.hpp's
# detail::np_dot/np_norm declare the same Accelerate ILP64 CBLAS entry
# points the oracle's numpy resolves, so every consumer's final link
# needs the framework.
add_library(bike_geometry STATIC
  "${_bike_native_dir}/src/geometry/specs.hpp"
  "${_bike_native_dir}/src/geometry/hardpoints.cpp"
  "${_bike_native_dir}/src/geometry/cockpit.cpp"
  "${_bike_native_dir}/src/geometry/validation.cpp")
set_target_properties(bike_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_geometry)
target_include_directories(bike_geometry SYSTEM PRIVATE
  "${_bike_native_dir}/src")
target_link_libraries(bike_geometry PUBLIC "-framework Accelerate")

# bike_kinematics: the T14 Horst-link boundary — linkage_math.py's
# brentq_pure/circle_circle_intersection_2d primitives, solver.py's
# HorstLinkageSolver (state/trajectory/inverse solves) and curves.py's
# instant centers. curves.cpp's 2x2 np.linalg.solve reaches the same
# Accelerate ILP64 dgesv the oracle's linalg imports; the geometry
# edge supplies the hardpoint/specs declarations and the transitive
# framework edge.
add_library(bike_kinematics STATIC
  "${_bike_native_dir}/src/kinematics/linkage_math.hpp"
  "${_bike_native_dir}/src/kinematics/linkage_math.cpp"
  "${_bike_native_dir}/src/kinematics/solver.hpp"
  "${_bike_native_dir}/src/kinematics/solver.cpp"
  "${_bike_native_dir}/src/kinematics/curves.cpp")
set_target_properties(bike_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_kinematics)
target_include_directories(bike_kinematics SYSTEM PRIVATE
  "${_bike_native_dir}/src")
target_link_libraries(bike_kinematics PUBLIC bike_geometry)

# bike_random: the T19 seeded-random boundary — SeedSequence's entropy
# coercion/INIT_A-pool mixing/spawn bookkeeping, the PCG64 XSL-RR core with
# the buffered next_uint32 lane, the distributions.c Ziggurat normal /
# uniform / Lemire-integer transforms, _generator.pyx's choice() and
# _common.pyx's kahan_sum, plus the sensor_noise.py and rider_random.py
# seams riding them. One STATIC library keeps each TU in a single
# compile-database entry per build graph (the ODR sweep relocatable-links
# every selected TU into one image). The boundary carries no third-party
# or Python dependency — numpy_compat.hpp reuses only first-party record
# headers (configuration/rider_spec.hpp, rider/rider_posture.hpp).
add_library(bike_random STATIC
  "${_bike_native_dir}/src/random/numpy_compat.hpp"
  "${_bike_native_dir}/src/random/numpy_compat.cpp"
  "${_bike_native_dir}/src/random/seed_sequence.cpp"
  "${_bike_native_dir}/src/random/distributions.cpp"
  "${_bike_native_dir}/src/random/noise_tape.cpp")
set_target_properties(bike_random PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_random)
target_include_directories(bike_random SYSTEM PRIVATE
  "${_bike_native_dir}/src")

# bike_engine: the pinned error interceptor shared by contract tests and the
# extension (identical source/context to the legacy bike_native_engine).
add_library(bike_engine STATIC "${_bike_native_dir}/src/engine_call.cpp")
set_target_properties(bike_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_target_strict_policy(bike_engine)
target_include_directories(bike_engine SYSTEM PRIVATE
  "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
target_link_libraries(bike_engine PUBLIC bike::mujoco bike_io)

# native_bench: the proto_native_bench prototype TU. The produced manifest
# declares tools/proto_native_bench a first-party root exactly like the
# legacy graph, so this target must exist here or the headers sweep has no
# compile context for state_format.hpp (the oracle fails closed on that
# hole). When the transition bindings are enabled, native/ owns the target.
if(NOT BIKE_BUILD_PYTHON_BINDINGS)
  add_executable(native_bench
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/proto_native_bench/bench.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/proto_native_bench/state_format.hpp")
  bike_target_strict_policy(native_bench)
  target_include_directories(native_bench SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(native_bench PRIVATE bike_engine)
  bike_rewrite_mujoco_load_name(native_bench
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
endif()

if(BUILD_TESTING)
  # tyre/distance.cpp SignedProfile + tyre/station.cpp scalar station law:
  # the produced-command contract's focused executable pair.
  add_executable(native_distance_tests "${_bike_native_tests_dir}/test_distance.cpp")
  bike_target_strict_policy(native_distance_tests)
  target_include_directories(native_distance_tests SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(native_distance_tests PRIVATE bike_core bike_engine
    bike_rider_math)
  target_link_libraries(native_distance_tests PRIVATE "-framework Accelerate")
  bike_rewrite_mujoco_load_name(native_distance_tests
    "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")

  add_executable(native_station_tests "${_bike_native_tests_dir}/test_station.cpp")
  bike_target_strict_policy(native_station_tests)
  target_include_directories(native_station_tests SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(native_station_tests PRIVATE bike_core bike_engine
    bike_rider_math)
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

# ---------------------------------------------------------------------------
# Produced check orchestration (T56): bike-native-checks/bike-python-audit
# executables, the emitted native_check_context.json/native_sources.json for
# the standalone graph, the check_<kind> sweep targets and the migration_T56
# fixture contract. Included last so every first-party target — root scope,
# native/tests/migration, and the transition extension when bindings are on —
# is already defined before the manifest enumerates it.
include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/Checks.cmake")
