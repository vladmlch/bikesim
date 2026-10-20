cmake_minimum_required(VERSION 3.27)
# Legacy transition build (cmake -S native -B <dir>): the nanobind extension,
# its Python-free contract core, the sweep harnesses and the migration tests.
#
# Platform layer first (cmake/Bike*.cmake, the base of this graph): host/SDK
# dispatch before project(), compiler/stdlib policy, the schema-1 dependency
# probe (MuJoCo, nanobind, Python), the qualified numerics provider, portable
# linkage and the schema-2 check context. HEAD's functional targets (io,
# configuration, geometry, kinematics, random, model, platform, startup, app,
# research and recording) are carried on that base. The repository-root
# standalone graph keeps its own wiring (CMakeLists.txt, BIKE_ROOT_BUILD).

if(BIKE_ROOT_BUILD)
  # The root graph with BIKE_BUILD_PYTHON_BINDINGS=ON would add this directory
  # after the root already resolved .deps. That mode is not migrated to the
  # platform layer yet; fail closed instead of mixing two dependency policies.
  message(FATAL_ERROR
    "BIKE_BUILD_PYTHON_BINDINGS=ON from the repository root is not wired to "
    "the cmake/Bike*.cmake platform layer yet; configure the legacy extension "
    "graph with cmake -S native instead")
endif()

# Host/SDK dispatch must precede project(): CMake runs its compiler link check
# there, so an SDK a compiler defaults to is resolved first (Apple only).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikePlatform.cmake")

project(bike_native CXX)
# A manually supplied CMAKE_SYSTEM_NAME (even one equal to the host name)
# makes CMake flag this build as cross-compiling; reject it now that the
# toolchain detection has run.
if(CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "Cross-compilation is not supported by this native-build workflow")
endif()
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler/architecture/standard-library policy — resolved once here and
# applied DIRECTLY to every first-party target via bike_apply_native_policy().
# The module declares NATIVE_SANITIZE/NATIVE_RTSAN/NATIVE_COVERAGE and
# BIKE_LIBCPP_HARDENING, rejects the ASan+RTSan combination at configure
# time, probes every flag it emits, and writes bike_policy_report.json.
# Include before BikeNumerics.cmake — its bike_native_numerics target fails
# configuration if the policy is not loaded first.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeCompilerPolicy.cmake")

# The compiler identification check above is not enough on Apple hosts:
# compile a source that includes an SDK header with the selected compiler and
# sysroot. The probe is Apple-only; Linux has no Availability.h.
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  bike_verify_apple_sdk_compatibility()
endif()

# Resolve the interpreter first so tools/native_dependencies.py is probed by
# the SAME python we compile against (uv run exposes .venv via VIRTUAL_ENV).
find_package(Python 3.13 REQUIRED COMPONENTS Interpreter Development.Module)
# test_contracts.cpp spawns std::thread — portable C++ threads go through
# the platform threads interface, not an assumed-integrated libc.
find_package(Threads REQUIRED)

# Dependency discovery: BikeDependencies.cmake runs the probe inside the
# selected interpreter, validates the schema-1 document it returns, then
# creates the imported Bike::MuJoCo target plus MJ_DIR/_native_mujoco_library
# and the nanobind/python include roots consumed below.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeDependencies.cmake")

# Numerics selection: BikeNumerics.cmake consumes the same schema-1 document,
# generates numerics_symbols.hpp, and builds the selected CBLAS/LAPACK
# backend TUs into bike_native_numerics (alias Bike::Numerics). The blas::
# call face has no definitions without it. This graph measures live, so it
# selects LIVE; the Python-free root graph selects RECORD_ONLY.
set(BIKE_NUMERICS_SOURCE LIVE)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeNumerics.cmake")

# Portable linkage contract: engine/dynamic-loader edges, numerics-provider
# linkage, per-platform runtime search paths (BUILD_RPATH on Linux — never
# environment rescue), and the Darwin MuJoCo install-name rewrite. Helpers
# live in cmake/BikeLinkage.cmake; every binary below calls the ones its
# dependency usage declares.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeLinkage.cmake")

# Header-only and SDK roots that the platform probe does not cover (RapidJSON,
# tomlplusplus, DuckDB, the lock) come from the repository's locked .deps tree.
# Same defaults the standalone root graph uses, anchored at this source tree.
# A missing tree is a configure-time acquisition failure, never a fallback.
get_filename_component(BIKE_MODULE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
if(DEFINED BIKE_DEPENDENCY_ROOT)
  get_filename_component(BIKE_DEPENDENCY_ROOT "${BIKE_DEPENDENCY_ROOT}" ABSOLUTE)
else()
  set(BIKE_DEPENDENCY_ROOT "${BIKE_MODULE_REPO_ROOT}/.deps" CACHE PATH
    "Explicitly acquired native dependency root (.deps)")
endif()
if(DEFINED BIKE_DEPENDENCY_LOCK)
  get_filename_component(BIKE_DEPENDENCY_LOCK "${BIKE_DEPENDENCY_LOCK}" ABSOLUTE)
else()
  set(BIKE_DEPENDENCY_LOCK "${BIKE_MODULE_REPO_ROOT}/native/dependencies.lock.json" CACHE FILEPATH
    "Checked dependency lock for the native build")
endif()

# Shared first-party target policy: C++23 without extensions, PIC for the
# extension link, the compiler policy applied directly, and the include surface
# the standalone-header sweep compiles every first-party header under. The
# include mirror widens the probe context only — it is not a dependency edge.
function(bike_apply_native_graph_policy target)
  set_target_properties(${target} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
  bike_apply_native_policy(${target})
  if(_native_contract_include_directories)
    target_include_directories(${target} SYSTEM PRIVATE
      ${_native_contract_include_directories})
  endif()
  target_include_directories(${target} SYSTEM PRIVATE
    "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
endfunction()

# Dependency-root include surface shared by the whole graph. The header sweep
# (native/tools checks --kind headers) compiles each first-party header under
# every target context of its root, so all first-party targets share it.
file(READ "${BIKE_DEPENDENCY_LOCK}" _native_dependency_lock_json)
string(JSON _native_duckdb_include GET "${_native_dependency_lock_json}"
  dependencies duckdb extracted_root)
set(_native_duckdb_include "${BIKE_DEPENDENCY_ROOT}/${_native_duckdb_include}/include")
if(NOT IS_DIRECTORY "${_native_duckdb_include}")
  message(FATAL_ERROR
    "pinned DuckDB include tree is missing: ${_native_duckdb_include} — acquire the locked .deps before configure")
endif()
include_directories(SYSTEM
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests"
  "${MJ_DIR}/include"
  "${_native_python_include}"
  "${_native_nb_include}"
  "${_native_robin_include}"
  "${_native_duckdb_include}")

# NB_SUPPRESS_WARNINGS: nanobind/Python headers land as SYSTEM includes —
# third-party TU internals stay outside our warning contract.
# PROTECT_STACK: nanobind otherwise appends -fno-stack-protector in Release
# configs, which would silently nullify -fstack-protector-strong.
# The Python-free core TUs live in bike_native_contract_core below; the
# extension keeps only the adapter TUs (nanobind parsing, boxing, the
# foreign-callback recognizer, the held-GIL materializer).
nanobind_add_module(bike_native NB_SUPPRESS_WARNINGS PROTECT_STACK
  src/binding.cpp
  src/drivetrain/policy_binding.cpp
  src/drivetrain/drive_binding.cpp
  src/rider/attachment_binding.cpp
  src/rider/contact_binding.cpp
  src/rider/intent_wire.cpp
  src/rider/rider_contact_binding.cpp
  src/rider/spindle_wire.cpp
  src/runtime/research_binding.cpp
  src/runtime/encode_binding.cpp
  src/runtime/samples_binding.cpp
  src/runtime/runtime_binding.cpp
  src/runtime/test_adapter.cpp)

set_target_properties(bike_native PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

# Compile-time provider identity of the numerics record, and the research
# build identity compiled into runtime/provenance.cpp (see below).
if(CMAKE_CONFIGURATION_TYPES)
  set(_research_identity_configuration "$<CONFIG>")
else()
  set(_research_identity_configuration "${CMAKE_BUILD_TYPE}")
endif()
target_include_directories(bike_native PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/research_identity/${_research_identity_configuration}")
# The full compiler/architecture/standard-library policy — warning list,
# required numerical/safety flags (-ffp-contract=off keeps bitwise parity with
# the Python oracle), Release -O2, libc++ hardening / _FORTIFY_SOURCE, the
# -headerpad_max_install_names link option, and the sanitizer/RTSan/coverage
# instrumentation — is applied DIRECTLY to each first-party target.
bike_apply_native_policy(bike_native)
# BIKE_MUJOCO_COMPAT_INCLUDE (BikeDependencies.cmake) precedes the real
# mujoco include dir only when the toolchain cannot parse mjsan.h under
# the sanitizer lane — it carries a generated attribute-compatible
# shadow header, never a forked copy.
target_include_directories(bike_native SYSTEM PRIVATE
  ${BIKE_MUJOCO_COMPAT_INCLUDE} "${MJ_DIR}/include")

# The captured context is the extension's effective compile/link state
# (policy plus nanobind's own options) and its shared include surface
# (research identity, MuJoCo, Python). Every target below mirrors it, so it is
# captured before the first of them is created.
get_target_property(_native_contract_compile_options bike_native COMPILE_OPTIONS)
get_target_property(_native_contract_compile_definitions bike_native COMPILE_DEFINITIONS)
get_target_property(_native_contract_include_directories bike_native INCLUDE_DIRECTORIES)
get_target_property(_native_contract_link_options bike_native LINK_OPTIONS)
if(TARGET bike_native_numerics)
  # bike_native_numerics received bike_apply_native_policy inside
  # BikeNumerics.cmake; only the shared include roots still mirror here.
  if(_native_contract_include_directories)
    target_include_directories(bike_native_numerics SYSTEM PRIVATE
      ${_native_contract_include_directories})
  endif()
  target_include_directories(bike_native_numerics SYSTEM PRIVATE
    "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
endif()

# Python-free core TU set shared by the extension and the direct contract
# tests: each TU is compiled exactly once (the ODR sweep relocatable-links
# every selected TU into a single image) and linked through the one
# target_link_libraries edge below. It is PIC for module linkage and carries
# the same policy context as the extension (-ffp-contract=off included).
add_library(bike_native_contract_core OBJECT src/stepper.cpp
  src/writer_config.cpp src/drivetrain/pedaling.cpp
  src/drivetrain/chain.cpp src/drivetrain/transmission.cpp
  src/drivetrain/elastic_chain.cpp src/drivetrain/motor_clutch.cpp
  src/drivetrain/shifting.cpp src/drivetrain/motor.cpp
  src/drivetrain/freehub.cpp src/drivetrain/state_codec.cpp
  src/writers/drivetrain.cpp
  src/writers/cruise.cpp src/tyre/profile.cpp
  src/tyre/distance.cpp src/tyre/station.cpp
  src/pneumatic/geometry.cpp src/pneumatic/carcass.cpp
  src/pneumatic/brush.cpp src/pneumatic/model.cpp
  src/rider/contact_math.cpp src/rider/support_geometry.cpp
  src/rider/least_squares.cpp src/rider/attachment_wrench.cpp
  src/rider/equality_reactions.cpp src/rider/state_codec.cpp
  src/runtime/samples.cpp
  src/runtime/wire.cpp
  src/runtime/control.cpp
  src/rider/intent.cpp src/rider/spindle_controller.cpp
  src/runtime/accounting.cpp src/runtime/period_buffer.cpp
  src/runtime/presentation.cpp src/runtime/status.cpp
  src/runtime/static_brake.cpp src/runtime/step.cpp
  src/runtime/bootstrap.cpp src/runtime/bootstrap_io.cpp
  src/runtime/runtime.cpp
  src/runtime/sensors.cpp src/runtime/programs.cpp
  src/runtime/research_metrics.cpp src/runtime/recording.cpp
  src/runtime/research.cpp src/runtime/research_io.cpp
  src/runtime/encode.cpp src/runtime/provenance.cpp
  src/runtime/provider_images.cpp
  src/writers/suspension.cpp src/writers/brake.cpp
  src/writers/resistance.cpp src/writers/rider_forces.cpp
  src/writers/tire_common.cpp src/writers/tire.cpp
  src/writers/distributed_tire.cpp src/writers/pneumatic_tire.cpp
  src/writers/rider_contacts.cpp)
bike_apply_native_graph_policy(bike_native_contract_core)


# T05 owned cold-path boundary: strict lossless JSON parse/serialize plus the
# recording-tag codec on the pinned RapidJSON headers, and the shared FIPS
# 180-4 SHA-256 that bike_native_engine uses for its engine-image digest.
set(_native_rapidjson_include
  "${BIKE_DEPENDENCY_ROOT}/rapidjson-24b5e7a/include")
if(NOT IS_DIRECTORY "${_native_rapidjson_include}")
  message(FATAL_ERROR
    "pinned RapidJSON include tree is missing: ${_native_rapidjson_include} — acquire the locked .deps before configure")
endif()
add_library(bike_io STATIC src/io/json.cpp src/io/hash.cpp
  src/io/value.hpp src/io/json.hpp src/io/hash.hpp)
bike_apply_native_graph_policy(bike_io)
target_include_directories(bike_io SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src" "${_native_rapidjson_include}")

# T06 bounded OS boundary: RAII file handles, exclusive no-follow creation,
# argv subprocesses, the deferred-SIGINT shield and the build/image identity
# block. Its only first-party edge is bike_io's shared SHA-256.
add_library(bike_platform STATIC
  src/platform/files.cpp src/platform/process.cpp
  src/platform/signals.cpp src/platform/provenance.cpp
  src/platform/files.hpp src/platform/process.hpp
  src/platform/signals.hpp src/platform/provenance.hpp)
bike_apply_native_graph_policy(bike_platform)
target_include_directories(bike_platform SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(bike_platform PRIVATE bike_io)

# Track-B build identity for bike_platform, same convention as the standalone
# root graph and research_build_identity.hpp: compiler/flags/SDK, the checked
# dependency lock and the first-party source manifest are hashed at configure
# and compiled in. This graph IS the transition extension, so the packaging
# axis records bindings.
if(NOT EXISTS "${BIKE_MODULE_REPO_ROOT}/migration/source-manifest.json")
  message(FATAL_ERROR
    "platform build identity requires migration/source-manifest.json")
endif()
file(SHA256 "${BIKE_MODULE_REPO_ROOT}/migration/source-manifest.json"
  _native_platform_source_manifest_sha256)
file(SHA256 "${BIKE_DEPENDENCY_LOCK}" _native_platform_lock_sha256)
if(CMAKE_CONFIGURATION_TYPES)
  set(_native_platform_identity_config "$<CONFIG>")
else()
  set(_native_platform_identity_config "${CMAKE_BUILD_TYPE}")
endif()
set(_native_platform_flags "${CMAKE_CXX_FLAGS}")
set(_native_platform_configs ${CMAKE_CONFIGURATION_TYPES}
  "${CMAKE_BUILD_TYPE}" Debug Release RelWithDebInfo MinSizeRel)
list(TRANSFORM _native_platform_configs TOUPPER)
list(REMOVE_DUPLICATES _native_platform_configs)
list(FILTER _native_platform_configs EXCLUDE REGEX "^$")
foreach(_configuration IN LISTS _native_platform_configs)
  string(TOUPPER "${_configuration}" _configuration_upper)
  string(APPEND _native_platform_flags
    " $<$<CONFIG:${_configuration}>:${CMAKE_CXX_FLAGS_${_configuration_upper}}>")
endforeach()
string(APPEND _native_platform_flags
  " $<JOIN:$<TARGET_GENEX_EVAL:bike_platform,$<TARGET_PROPERTY:bike_platform,COMPILE_OPTIONS>>, >")
# COMPILE_DEFINITIONS are part of the effective flag surface too — the
# contract definitions (_LIBCPP_HARDENING_MODE/_FORTIFY_SOURCE) change the
# generated code, so two builds differing only in BIKE_LIBCPP_HARDENING must
# not record an identical identity. Generator-expression entries still resolve
# per configuration inside file(GENERATE)'s target context.
get_target_property(_native_platform_definitions bike_platform
  COMPILE_DEFINITIONS)
if(_native_platform_definitions)
  foreach(_native_platform_definition IN LISTS _native_platform_definitions)
    string(APPEND _native_platform_flags " -D${_native_platform_definition}")
  endforeach()
endif()
set(_native_platform_gui_bool "false")
if(BIKE_BUILD_GUI)
  set(_native_platform_gui_bool "true")
endif()
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/platform_identity/$<CONFIG>/platform_build_identity.hpp"
  CONTENT "// Generated by CMake for this configuration; do not edit.\n#pragma once\nnamespace bike::platform::identity {\ninline constexpr char compiler_id[] = \"${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\";\ninline constexpr char numerical_flags[] = R\"identity(${_native_platform_flags})identity\";\ninline constexpr char sdk[] = R\"identity(${_native_sdk_path})identity\";\ninline constexpr char platform[] = \"${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}\";\ninline constexpr char build_type[] = \"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\";\ninline constexpr char dependency_lock_sha256[] = \"${_native_platform_lock_sha256}\";\ninline constexpr char source_manifest_sha256[] = \"${_native_platform_source_manifest_sha256}\";\ninline constexpr bool python_bindings = true;\ninline constexpr bool gui = ${_native_platform_gui_bool};\n}\n"
  TARGET bike_platform)
target_include_directories(bike_platform PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/platform_identity/${_native_platform_identity_config}")

# T12 configuration boundary: the ported configuration records, validators and
# the schema/wire surfaces. The JSON profile loaders read bounded regular files
# through bike_platform; the schema/wire surfaces only touch bike_io.
set(_native_tomlplusplus_include
  "${BIKE_DEPENDENCY_ROOT}/tomlplusplus-3.4.0/include")
if(NOT IS_DIRECTORY "${_native_tomlplusplus_include}")
  message(FATAL_ERROR
    "pinned tomlplusplus include tree is missing: ${_native_tomlplusplus_include} — acquire the locked .deps before configure")
endif()
add_library(bike_configuration STATIC
  src/configuration/schema.cpp src/configuration/validate.cpp
  src/configuration/load.cpp src/configuration/paths.cpp
  src/configuration/profile_writer.cpp
  src/configuration/fields.hpp src/configuration/load.hpp
  src/configuration/model_config.hpp
  src/configuration/physical_config.hpp src/configuration/rider_spec.hpp
  src/configuration/suspension_spec.hpp src/configuration/wire_decode.hpp)
bike_apply_native_graph_policy(bike_configuration)
target_include_directories(bike_configuration SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src" "${_native_rapidjson_include}"
  "${_native_tomlplusplus_include}")
target_link_libraries(bike_configuration PRIVATE bike_io bike_platform
  bike_rider_math)

# bike_rider_math: the pure spindle rider-math TU — one library per graph,
# so the ODR sweep sees a single compile-database entry for it. The extension
# and bike_configuration both resolve its symbols through this link edge.
add_library(bike_rider_math STATIC src/rider/spindle_math.cpp)
bike_apply_native_graph_policy(bike_rider_math)
# spindle_math.cpp's np_norm reaches the blas:: face (ddot), so the adapter
# is part of this library's interface, like bike_native_engine's.
target_link_libraries(bike_rider_math PUBLIC Bike::Numerics)

# T14 geometry boundary: the identical geometry/*.cpp TUs as the root graph.
# These TUs still call the raw Accelerate ILP64 CBLAS/LAPACK entry points
# (specs.hpp detail::np_dot/np_norm); the port's provider target carries the
# Apple framework on Darwin. Linux has no such provider and these TUs are not
# qualified there yet (see the report's raw-Accelerate note).
add_library(bike_geometry STATIC
  src/geometry/specs.hpp src/geometry/hardpoints.cpp
  src/geometry/cockpit.cpp src/geometry/validation.cpp)
bike_apply_native_graph_policy(bike_geometry)
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  target_link_libraries(bike_geometry PUBLIC Bike::NumericsProvider)
endif()

# T14 Horst-link boundary: curves.cpp's 2x2 np.linalg.solve reaches Accelerate's
# ILP64 dgesv; the bike_geometry edge supplies the transitive provider.
add_library(bike_kinematics STATIC
  src/kinematics/linkage_math.hpp src/kinematics/linkage_math.cpp
  src/kinematics/solver.hpp src/kinematics/solver.cpp
  src/kinematics/curves.cpp)
bike_apply_native_graph_policy(bike_kinematics)
target_link_libraries(bike_kinematics PUBLIC bike_geometry)

# T19 seeded-random boundary: no third-party or Python dependency.
add_library(bike_random STATIC
  src/random/numpy_compat.hpp src/random/numpy_compat.cpp
  src/random/seed_sequence.cpp src/random/distributions.cpp
  src/random/noise_tape.cpp)
bike_apply_native_graph_policy(bike_random)

# T15 model boundary: mass.cpp's compiled-model reads inspect mjModel/mjData
# fields (the MuJoCo include edge is compile-only), and the Accelerate provider
# arrives transitively through bike_kinematics -> bike_geometry.
add_library(bike_model STATIC
  src/model/checks.hpp src/model/mass.hpp src/model/mass.cpp
  src/model/rider_geometry.hpp src/model/rider_geometry.cpp
  src/model/suspension_setup.hpp src/model/suspension_setup.cpp)
bike_apply_native_graph_policy(bike_model)
target_link_libraries(bike_model PUBLIC bike_kinematics bike_configuration)

# The pinned error interceptor is compiled once and shared by the extension,
# direct C++ contracts, and the native benchmark target. Its engine-image
# digest uses the shared bike::io::Sha256 — no private copy.
add_library(bike_native_engine STATIC src/engine_call.cpp)
bike_apply_native_graph_policy(bike_native_engine)
# The engine image and the platform dynamic-loader library are part of the
# engine library's interface — every consumer links its objects. engine_call.cpp
# reports the numerics provider through blas::provider_name, so the typed
# adapter is part of the same interface: every consumer of this library gets it.
bike_link_engine(bike_native_engine PUBLIC)
target_link_libraries(bike_native_engine PUBLIC bike_io Bike::Numerics)

# The module links the shared core objects, the engine image, the pure
# spindle math and the typed numerics adapter used by the blas:: call face in
# resistance/tire/etc. The Darwin install-name rewrite makes `import
# bike_native` work before mujoco is loaded.
target_link_libraries(bike_native PRIVATE
  bike_native_contract_core bike_native_engine bike_rider_math)
bike_link_engine(bike_native)
bike_link_numerics(bike_native)
bike_fix_mujoco_install_name(bike_native)

# allocation_faults.cpp overrides the test binary's throwing allocation
# operators — it must never land in a shipped target, so it reaches the
# executable as an injected object, not through the shared contract core.
# The single OBJECT compile lives in tests/migration (shared with
# migration_T07); TARGET_OBJECTS resolves at generate time, so the later
# add_subdirectory definition is fine.
add_executable(native_contract_tests tests/test_contracts.cpp
  $<TARGET_OBJECTS:bike_test_allocation_faults>)
bike_apply_native_graph_policy(native_contract_tests)
target_link_libraries(native_contract_tests PRIVATE
  bike_native_contract_core bike_native_engine bike_rider_math Threads::Threads)
# chain.cpp's GeometryWorkspace resolves CBLAS through the typed adapter —
# Bike::Numerics carries the qualified provider transitively (Accelerate
# here, the scipy-openblas wheel .so on Linux).
bike_link_engine(native_contract_tests)
bike_link_numerics(native_contract_tests)
bike_fix_mujoco_install_name(native_contract_tests)

# tyre/distance.cpp (SignedProfile) contract cases — same build context as
# native_contract_tests, without the allocation-fault override TU.
add_executable(native_distance_tests tests/test_distance.cpp)
bike_apply_native_graph_policy(native_distance_tests)
target_link_libraries(native_distance_tests PRIVATE
  bike_native_contract_core bike_native_engine bike_rider_math)
bike_link_engine(native_distance_tests)
bike_link_numerics(native_distance_tests)
bike_fix_mujoco_install_name(native_distance_tests)

# tyre/station.cpp (scalar station law) contract cases — same build context
# as native_distance_tests.
add_executable(native_station_tests tests/test_station.cpp)
bike_apply_native_graph_policy(native_station_tests)
target_link_libraries(native_station_tests PRIVATE
  bike_native_contract_core bike_native_engine bike_rider_math)
bike_link_engine(native_station_tests)
bike_link_numerics(native_station_tests)
bike_fix_mujoco_install_name(native_station_tests)

# tests/test_platform_blas.cpp — the standalone CBLAS adapter harness. It
# calls only blas:: wrappers and std headers, so Bike::Numerics alone
# supplies the backend definitions plus the qualified provider link.
add_executable(native_platform_blas_tests tests/test_platform_blas.cpp)
bike_apply_native_graph_policy(native_platform_blas_tests)
bike_link_numerics(native_platform_blas_tests)

# tests/test_platform_lapack.cpp — the standalone LAPACK adapter harness.
# It calls only numeric_backend::dgelsd and std headers, so
# Bike::Numerics alone supplies the backend definition plus the
# qualified provider link.
add_executable(native_platform_lapack_tests tests/test_platform_lapack.cpp)
bike_apply_native_graph_policy(native_platform_lapack_tests)
bike_link_numerics(native_platform_lapack_tests)

# Prototype native benchmark (native-safety runtime plan task R2). The TU
# lives outside native/, so its directory joins the first-party source roots
# below and the target inherits the extension's warning/hardening/sanitizer
# context — the manifest loop then places bench.cpp into every sweep.
# bench.cpp reaches engine_call.hpp/model_access.hpp by relative include and
# links bike_native_engine like native_contract_tests does.
get_filename_component(_native_bench_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../tools/proto_native_bench" ABSOLUTE)
if(NOT IS_DIRECTORY "${_native_bench_dir}")
  message(FATAL_ERROR "proto_native_bench directory is missing: ${_native_bench_dir}")
endif()
set(_native_first_party_roots "${CMAKE_CURRENT_SOURCE_DIR}" "${_native_bench_dir}")

add_executable(native_bench
  "${_native_bench_dir}/bench.cpp" "${_native_bench_dir}/state_format.hpp")
bike_apply_native_graph_policy(native_bench)
target_link_libraries(native_bench PRIVATE bike_native_engine)
bike_link_engine(native_bench)
bike_fix_mujoco_install_name(native_bench)

# native/tests/sanitizer_control.cpp — the T15 runtime control: compiled
# with the selected lane's real instrumentation through the same policy
# application as every other first-party target. CTest runs only its
# clean mode; the use-after-free mode is exercised by
# tests/portability/test_native_runtime.py in a subprocess, where the
# fatal ASan report is the expected outcome (a loader error or unrelated
# crash is not accepted as a sanitizer result).
add_executable(native_sanitizer_control tests/sanitizer_control.cpp)
set_target_properties(native_sanitizer_control PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_sanitizer_control)

# native/tests/rtsan_control.cpp — the T16 realtime-sanitizer control pair:
# the default mode is a valid nonblocking arithmetic kernel (clean under an
# instrumented build and under every ordinary build), and the
# `forbidden-allocation` mode is the intentional violation the configure-time
# qualification (BikeCompilerPolicy.cmake) and the subprocess controls in
# tests/reference/test_native_check_tools.py require to die with the real
# RealtimeSanitizer diagnostic.
add_executable(native_rtsan_control tests/rtsan_control.cpp)
set_target_properties(native_rtsan_control PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_rtsan_control)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME native_contract_tests COMMAND native_contract_tests)
  add_test(NAME native_distance_tests COMMAND native_distance_tests)
  add_test(NAME native_station_tests COMMAND native_station_tests)
  add_test(NAME native_platform_blas_tests COMMAND native_platform_blas_tests)
  add_test(NAME native_platform_lapack_tests COMMAND native_platform_lapack_tests)
  add_test(NAME native_sanitizer_control COMMAND native_sanitizer_control)
  # Only the clean mode runs under CTest — the forbidden-allocation mode
  # must die with the sanitizer report, so it is exercised by the
  # configure-time qualification and the pytest subprocess controls.
  add_test(NAME native_rtsan_control COMMAND native_rtsan_control)
  add_test(NAME native_bench_bare
    COMMAND native_bench bare 20 5
            --artifacts "${_native_bench_dir}/artifacts")
  add_test(NAME native_bench_glue
    COMMAND native_bench glue 10 4
            --artifacts "${_native_bench_dir}/artifacts")
  add_test(NAME native_bench_heavy
    COMMAND native_bench heavy 6 3
            --artifacts "${_native_bench_dir}/artifacts")
endif()

# Startup graph (T16–T25) and the physics/research/recording applications.
# Their targets reuse the captured _native_contract_* context above and the
# portable linkage helpers; they are declared here, never compiled at configure.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Provenance.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/NativeStartup.cmake")
bike_add_native_startup("${CMAKE_CURRENT_SOURCE_DIR}")

# Migration contract-test harness and per-task regressions (T02): one
# executable and one CTest name per task under native/tests/migration. The
# subdirectory inherits this scope's _native_contract_* context and the
# BUILD_TESTING gate from include(CTest) above.
add_subdirectory(tests/migration)

add_custom_target(check_native_contracts
  COMMAND uv run --frozen --group native python "${CMAKE_SOURCE_DIR}/../tools/check_native_contracts.py"
          --build "${CMAKE_BINARY_DIR}"
  COMMENT "native structural contract checks"
  VERBATIM)

# Multi-compiler sweep: same TU set through GCC's frontend, warnings-as-errors.
add_custom_target(check_frontends
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_native_frontends.sh
          "${CMAKE_BINARY_DIR}"
          "${CMAKE_SOURCE_DIR}/src" "${MJ_DIR}/include"
  COMMENT "gcc -fsyntax-only sweep (second frontend)")

# clang-tidy sweep: LLVM static-analysis lint over the same TU set, configured
# by native/.clang-tidy and driven by compile_commands.json.
add_custom_target(check_tidy
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_native_tidy.sh
          "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}/src"
  COMMENT "clang-tidy sweep (LLVM linter)")

# Clang Static Analyzer sweep: path-sensitive per-TU analysis — deeper than
# the clang-analyzer-* subset inside tidy. Needs the brew clang.
add_custom_target(check_analyzer
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_analyzer.sh
          "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}/src"
  COMMENT "clang static analyzer sweep (path-sensitive)")

# cppcheck sweep: a third analyzer engine with different coverage; suppressions
# for this tree are documented in the script header.
add_custom_target(check_cppcheck
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_cppcheck.sh
          "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}/src"
          "${Python_EXECUTABLE}"
  COMMENT "cppcheck sweep (third analyzer engine)")

# ODR sweep: -fsyntax-only sees no cross-TU type conflicts — -flto -Wodr at
# link time does.
add_custom_target(check_odr
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_odr.sh
          "${CMAKE_BINARY_DIR}"
          "${CMAKE_SOURCE_DIR}/src" "${MJ_DIR}/include"
          "${Python_EXECUTABLE}"
  COMMENT "gcc -flto -Wodr sweep (cross-TU type conflicts)")

# shellcheck over tools/*.sh — same everything-is-an-error contract.
add_custom_target(check_scripts
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_scripts.sh
          "${CMAKE_SOURCE_DIR}/../tools"
  COMMENT "shellcheck sweep (tools/*.sh)")

# Track B identity is compiled into the extension. It is not read from an
# editable build-context JSON at runtime. Source-list and source-content changes
# force CMake regeneration before the corresponding binary can be delivered.
file(GLOB_RECURSE _research_identity_inputs CONFIGURE_DEPENDS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.in")
list(APPEND _research_identity_inputs "CMakeLists.txt")
list(REMOVE_DUPLICATES _research_identity_inputs)
list(SORT _research_identity_inputs)
set(_research_identity_manifest "")
foreach(_relative IN LISTS _research_identity_inputs)
  set(_absolute "${CMAKE_CURRENT_SOURCE_DIR}/${_relative}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_absolute}")
  file(SHA256 "${_absolute}" _digest)
  string(APPEND _research_identity_manifest "${_relative}=${_digest}\n")
endforeach()
string(SHA256 _research_identity_sha256 "${_research_identity_manifest}")
set(_research_effective_flags "${CMAKE_CXX_FLAGS}")
set(_research_identity_configurations ${CMAKE_CONFIGURATION_TYPES} "${CMAKE_BUILD_TYPE}"
  Debug Release RelWithDebInfo MinSizeRel)
list(TRANSFORM _research_identity_configurations TOUPPER)
list(REMOVE_DUPLICATES _research_identity_configurations)
list(FILTER _research_identity_configurations EXCLUDE REGEX "^$")
foreach(_configuration IN LISTS _research_identity_configurations)
  string(TOUPPER "${_configuration}" _configuration_upper)
  string(APPEND _research_effective_flags
    " $<$<CONFIG:${_configuration}>:${CMAKE_CXX_FLAGS_${_configuration_upper}}>")
endforeach()
string(APPEND _research_effective_flags
  " $<JOIN:$<TARGET_GENEX_EVAL:bike_native,$<TARGET_PROPERTY:bike_native,COMPILE_OPTIONS>>, >")
_native_json_quote(_research_effective_flags_json "${_research_effective_flags}")
# The identity header serializes the compiler identity itself — quote it at
# directory scope here; the check-context writer's own quoted variables only
# exist inside bike_write_check_context() below and cannot be referenced.
_native_json_quote(_identity_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
_native_json_quote(_identity_compiler_version_json "${CMAKE_CXX_COMPILER_VERSION}")

# Compile-time numerical provider identity — the qualified selection baked into
# the binary. The extension's runtime_identity() returns the policy declared
# here together with the provider images actually loaded (provider_images.cpp
# inspection, read in provenance.cpp); Python-side provenance cross-checks the
# two before any native execution, so a substituted or drifted provider is a
# named failure rather than silent bitwise drift. The declarations come from
# BikeProviderIdentity.cmake, which the Python-free root graph also uses. This
# graph's build_id is machine-specific and comes from the live probe document;
# the root graph derives it from its configure-time probe instead.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeProviderIdentity.cmake")
set(_identity_build_ids "")
list(LENGTH _native_numerics_provider_paths _identity_library_count)
if(_identity_library_count GREATER 0)
  math(EXPR _identity_library_last "${_identity_library_count} - 1")
  foreach(_index RANGE ${_identity_library_last})
    string(JSON _identity_build_type TYPE "${_native_dependencies_json}"
      numerics library_paths ${_index} build_id)
    set(_identity_build_id "<none>")
    if(_identity_build_type STREQUAL "STRING")
      string(JSON _identity_build_id GET "${_native_dependencies_json}"
        numerics library_paths ${_index} build_id)
    endif()
    list(APPEND _identity_build_ids "${_identity_build_id}")
  endforeach()
endif()
bike_provider_identity_declarations(_identity_declarations
  RECORD_VAR _numerics_record_json
  RECORD_KEY "${_native_numerics_record_key}"
  BACKEND "${BIKE_NUMERICS_BACKEND_RESOLVED}"
  BLAS_BITS "${_native_numerics_blas_bits}"
  LAPACK_BITS "${_native_numerics_lapack_bits}"
  PROVIDER_PATHS_VAR _native_numerics_provider_paths
  BUILD_IDS_VAR _identity_build_ids)

file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/research_identity/$<CONFIG>/research_build_identity.hpp"
  CONTENT "// Generated by CMake for this configuration; do not edit.\n#pragma once\nnamespace runtime::build_identity {\ninline constexpr const char native_source_sha256[] = \"${_research_identity_sha256}\";\ninline constexpr const char context_json[] = R\"identity({\"build_type\":\"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\",\"compiler_id\":${_identity_compiler_id_json},\"compiler_version\":${_identity_compiler_version_json},\"numerical_flags\":${_research_effective_flags_json}})identity\";\n${_identity_declarations}}\n"
  TARGET bike_native)

# Emit the source and environment contract from the configured build graph.
# BikeCheckContext.cmake writes the schema-1 native_sources.json manifest
# the sweeps consume and the schema-2 native_check_context.json build
# context — per-configuration, generator-expression-expanded, emitted here
# because every first-party target must already exist when the target
# scan runs. (_native_json_quote/_native_json_array come from
# BikeCompilerPolicy.cmake; _native_mujoco_library from
# BikeDependencies.cmake.)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeCheckContext.cmake")
bike_write_check_context()
