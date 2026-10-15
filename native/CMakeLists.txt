cmake_minimum_required(VERSION 3.24)
# Shared toolchain policy with the standalone root build (T03): SDK probing
# (behind APPLE, explicit-SDK failures retained), libc++ hardening detection
# and the strict-flag/sanitizer policy applied to every first-party target.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerPolicy.cmake")
# The mach-o load-name rewrite helper lives in the dependency module; the
# legacy build still resolves MuJoCo through the pinned wheel (MJ_DIR below)
# and only reuses the helper — it never consumes .deps.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Provenance.cmake")

# CMake performs its compiler link check during project(), so resolve a
# default SDK before that call — behind APPLE, never replacing an SDK or
# compiler supplied by the caller. When this file is add_subdirectory'd into
# the standalone root build (BIKE_ROOT_BUILD), the parent already resolved
# and verified the SDK; this build inherits the resolved path.
if(APPLE AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  bike_resolve_macos_sdk()
elseif(APPLE)
  set(_native_sdk_source "inherited")
  set(_native_sdk_requested "${CMAKE_OSX_SYSROOT}")
  if(IS_DIRECTORY "${CMAKE_OSX_SYSROOT}")
    get_filename_component(_native_sdk_path "${CMAKE_OSX_SYSROOT}" REALPATH)
  else()
    set(_native_sdk_path "${CMAKE_OSX_SYSROOT}")
  endif()
else()
  set(_native_sdk_source "not_applicable")
  set(_native_sdk_requested "")
  set(_native_sdk_path "")
endif()

project(bike_native CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BIKE_LIBCPP_HARDENING "EXTENSIVE" CACHE STRING "libc++ hardening mode")
set_property(CACHE BIKE_LIBCPP_HARDENING PROPERTY STRINGS EXTENSIVE FAST)
if(NOT BIKE_LIBCPP_HARDENING STREQUAL "EXTENSIVE" AND NOT BIKE_LIBCPP_HARDENING STREQUAL "FAST")
  message(FATAL_ERROR "BIKE_LIBCPP_HARDENING must be EXTENSIVE or FAST, got '${BIKE_LIBCPP_HARDENING}'")
endif()

if(APPLE)
  # The compiler identification check is not enough: compile a source that
  # includes an SDK header with the selected compiler and sysroot.
  bike_verify_sdk_compat()
endif()

# Instrumentation options shared with the standalone root build; the shared
# policy function consumes them per target (docs in CompilerPolicy.cmake).
# RTSan requires the BREW clang — Apple clang 21 lacks the sanitizer:
#   cmake -S native -B native/build/rtsan \
#     -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -DNATIVE_RTSAN=ON
# and run native tests with the rtsan dylib in DYLD_INSERT_LIBRARIES.
# NATIVE_COVERAGE pairs with tools/native_coverage.sh.
option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
option(NATIVE_RTSAN "RealtimeSanitizer build (brew clang only)" OFF)
option(NATIVE_COVERAGE "llvm-cov instrumented build" OFF)

# Resolve the interpreter first so mujoco/nanobind paths below are queried from
# the SAME python we compile against (uv run exposes .venv via VIRTUAL_ENV).
find_package(Python 3.13 REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(COMMAND "${Python_EXECUTABLE}" -c "import mujoco; print(mujoco.__version__)"
  RESULT_VARIABLE _native_mujoco_version_result
  OUTPUT_VARIABLE _native_mujoco_version
  ERROR_VARIABLE _native_mujoco_version_error
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _native_mujoco_version_result EQUAL 0 OR NOT _native_mujoco_version STREQUAL "3.12.0")
  message(FATAL_ERROR
    "bike_native requires the pinned MuJoCo 3.12.0 error ABI; configured Python '${Python_EXECUTABLE}' found '${_native_mujoco_version}': ${_native_mujoco_version_error}")
endif()

execute_process(COMMAND "${Python_EXECUTABLE}" -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MJ_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# The stock 3.12.0 Python binding uses these private TLS functions. Check both
# declarations and linked symbols against the installed wheel before building
# any target that could otherwise fall back to MuJoCo's process-exiting handler.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/src" "${MJ_DIR}/include")
set(CMAKE_REQUIRED_LIBRARIES "${MJ_DIR}/libmujoco.3.12.0.dylib")
unset(BIKE_MUJOCO_ERROR_ABI_312 CACHE)
check_cxx_source_compiles([=[
#include "engine_abi_312.hpp"
int main() {
  mjfLogHandler previous = _mjPRIVATE_setTlsLogHandler(nullptr);
  (void) _mjPRIVATE_setTlsLogHandler(previous);
  void (*volatile raw_data)(mjData**, const mjModel*) = &mj_makeRawData;
  return _mjPRIVATE_getGlobalLogHandler() == nullptr || raw_data == nullptr;
}
]=] BIKE_MUJOCO_ERROR_ABI_312)
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_LIBRARIES)
if(NOT BIKE_MUJOCO_ERROR_ABI_312)
  message(FATAL_ERROR "MuJoCo 3.12.0 private TLS error-handler ABI is unavailable for ${Python_EXECUTABLE}")
endif()

# Resolve standalone third-party header roots before adding the direct-contract
# targets; their header sweep compiles every first-party native/src header.
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --include_dir
  RESULT_VARIABLE _native_nb_include_result
  OUTPUT_VARIABLE _native_nb_include
  ERROR_VARIABLE _native_nb_include_error
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _native_nb_include_result EQUAL 0 OR NOT IS_DIRECTORY "${_native_nb_include}")
  message(FATAL_ERROR "Cannot resolve nanobind include directory: ${_native_nb_include_error}")
endif()
get_filename_component(_native_nb_package "${_native_nb_include}/.." REALPATH)
get_filename_component(_native_robin_include "${_native_nb_package}/ext/robin_map/include" REALPATH)
get_filename_component(_native_python_include "${Python_INCLUDE_DIRS}" REALPATH)

bike_detect_libcpp_hardening()
set(_native_libcpp_hardening_supported "${BIKE_LIBCPP_HARDENING_SUPPORTED}")
set(_native_libcpp_hardening_mode "${BIKE_LIBCPP_HARDENING_EFFECTIVE}")

# NB_SUPPRESS_WARNINGS: nanobind/Python headers land as SYSTEM includes —
# third-party TU internals stay outside our warning contract.
# PROTECT_STACK: nanobind otherwise appends -fno-stack-protector in Release
# configs, which would silently nullify -fstack-protector-strong below.
# stepper.cpp and writers/rider_contacts.cpp moved into the shared
# Python-free core TU set (T07): the extension picks them up through the
# ${_native_core_lib} link edge like the writer bodies. Only adapter-side
# translation units — nanobind parsing, boxing, the foreign-callback
# recognizer — remain listed on the module.
nanobind_add_module(bike_native NB_SUPPRESS_WARNINGS PROTECT_STACK
  src/binding.cpp
  src/drivetrain/policy_binding.cpp
  src/drivetrain/drive_binding.cpp
  src/rider/attachment_binding.cpp
  src/rider/contact_binding.cpp
  src/rider/intent.cpp
  src/rider/intent_wire.cpp
  src/rider/rider_contact_binding.cpp
  src/rider/spindle_controller.cpp
  src/rider/spindle_wire.cpp
  src/runtime/sensors.cpp
  src/runtime/programs.cpp
  src/runtime/research_metrics.cpp
  src/runtime/research.cpp
  src/runtime/research_binding.cpp
  src/runtime/recording.cpp
  src/runtime/provenance.cpp
  src/runtime/accounting.cpp
  src/runtime/period_buffer.cpp
  src/runtime/samples_binding.cpp
  src/runtime/encode.cpp
  src/runtime/status.cpp
  src/runtime/runtime_binding.cpp
  src/runtime/static_brake.cpp
  src/runtime/step.cpp
  src/runtime/presentation.cpp
  src/runtime/test_adapter.cpp)

set_target_properties(bike_native PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

# Strict flags are PRIVATE to our TUs: nanobind's own static-lib sources are
# not expected to survive this warning set under -Werror.
# -ffp-contract=off: clang's default (on) fuses mul+add into FMA on arm64 —
# one rounding instead of two, breaking bitwise parity with the Python
# oracle, whose interpreter rounds every operation separately.
if(CMAKE_CONFIGURATION_TYPES)
  set(_research_identity_configuration "$<CONFIG>")
else()
  set(_research_identity_configuration "${CMAKE_BUILD_TYPE}")
endif()
target_include_directories(bike_native PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/research_identity/${_research_identity_configuration}")
# Strict flags are PRIVATE to our TUs: nanobind's own static-lib sources are
# not expected to survive this warning set under -Werror. The shared policy
# (warnings-as-errors, stack protection, hardening define, -O2 under Release,
# disabled FP contraction, optional sanitizer/RTSan/coverage) is identical
# to the standalone root build's per-target policy.
bike_target_strict_policy(bike_native)
target_include_directories(bike_native SYSTEM PRIVATE "${MJ_DIR}/include")
# -headerpad_max_install_names is attached by bike_rewrite_mujoco_load_name
# below — every rewritten binary reserves the padding by construction.

# Share Python-free core code with direct native contract tests: the
# drivetrain/policy TUs plus the owning writer cores (suspension, brake,
# resistance, rider_forces, tire) whose warm-core entries the R1 contract
# cases exercise, plus the pneumatic geometry kernel the same single-TU
# convention keeps linkable for later writer work. The object is compiled
# once with the extension's complete
# warning, hardening, sanitizer, and coverage context, and remains PIC for
# module linkage. The contract binary links these objects directly, so each
# TU keeps exactly one translation unit across all targets (the ODR sweep
# links every selected TU into a single image). bike_native picks them up
# through the existing target_link_libraries edge below — same
# COMPILE_OPTIONS, including -ffp-contract=off, as when they were listed on
# the module itself.
#
# In the standalone root build (BIKE_ROOT_BUILD) the parent already compiled
# the same TU set as bike_core/bike_engine and owns the direct test
# executables: the extension links those identical objects, so the optional
# extension and the standalone tests exercise the same core objects for
# differential tests.
get_target_property(_native_contract_compile_options bike_native COMPILE_OPTIONS)
get_target_property(_native_contract_compile_definitions bike_native COMPILE_DEFINITIONS)
get_target_property(_native_contract_include_directories bike_native INCLUDE_DIRECTORIES)
get_target_property(_native_contract_link_options bike_native LINK_OPTIONS)

if(BIKE_ROOT_BUILD)
  set(_native_core_lib bike_core)
  set(_native_engine_lib bike_engine)
else()
add_library(bike_native_contract_core OBJECT src/stepper.cpp
  src/writer_config.cpp src/drivetrain/pedaling.cpp
  src/drivetrain/chain.cpp src/drivetrain/transmission.cpp
  src/drivetrain/shifting.cpp src/drivetrain/motor.cpp
  src/drivetrain/freehub.cpp src/drivetrain/state_codec.cpp
  src/writers/drivetrain.cpp
  src/writers/cruise.cpp src/tyre/profile.cpp
  src/tyre/distance.cpp src/tyre/station.cpp
  src/pneumatic/geometry.cpp src/pneumatic/carcass.cpp
  src/pneumatic/brush.cpp src/pneumatic/model.cpp
  src/rider/contact_math.cpp src/rider/support_geometry.cpp
  src/rider/least_squares.cpp src/rider/attachment_wrench.cpp
  src/rider/equality_reactions.cpp src/rider/spindle_math.cpp
  src/rider/state_codec.cpp src/runtime/samples.cpp
  src/runtime/wire.cpp
  src/runtime/control.cpp
  src/writers/suspension.cpp src/writers/brake.cpp
  src/writers/resistance.cpp src/writers/rider_forces.cpp
  src/writers/tire_common.cpp src/writers/tire.cpp
  src/writers/distributed_tire.cpp src/writers/pneumatic_tire.cpp
  src/writers/rider_contacts.cpp)
set_target_properties(bike_native_contract_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_native_contract_core PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_native_contract_core PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(bike_native_contract_core SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(bike_native_contract_core SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
set(_native_core_lib bike_native_contract_core)

# T05 owned cold-path boundary — the identical io/*.cpp TUs the standalone
# root build compiles as bike_io, here in the legacy warning/hardening
# context. This graph does not resolve .deps imported targets, so the pinned
# RapidJSON include is anchored at BIKE_DEPENDENCY_ROOT directly; a missing
# tree is a configure-time acquisition failure, never a fallback.
set(_native_rapidjson_include
  "${BIKE_DEPENDENCY_ROOT}/rapidjson-24b5e7a/include")
if(NOT IS_DIRECTORY "${_native_rapidjson_include}")
  message(FATAL_ERROR
    "pinned RapidJSON include tree is missing: ${_native_rapidjson_include} — acquire the locked .deps before configure")
endif()
add_library(bike_io STATIC src/io/json.cpp src/io/hash.cpp
  src/io/value.hpp src/io/json.hpp src/io/hash.hpp)
set_target_properties(bike_io PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_io PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_io PRIVATE ${_native_contract_compile_definitions})
endif()
target_include_directories(bike_io SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src" "${_native_rapidjson_include}")
# The standalone-header sweep compiles every first-party header under
# native/src in every target context that owns a native/src translation
# unit, so bike_io's compile context must resolve the shared include
# surface (MuJoCo/nanobind/Python/robin) even though the io translation
# units include none of it — include paths widen the probe context, they
# are not a dependency of the shipped objects.
if(_native_contract_include_directories)
  target_include_directories(bike_io SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(bike_io SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")

# T06 bounded OS boundary — the identical platform/*.cpp TUs the standalone
# root build compiles as bike_platform, here in the legacy warning/hardening
# context. Its only first-party edge is bike_io's shared SHA-256; it carries
# no physics execution callbacks.
add_library(bike_platform STATIC
  src/platform/files.cpp src/platform/process.cpp
  src/platform/signals.cpp src/platform/provenance.cpp
  src/platform/files.hpp src/platform/process.hpp
  src/platform/signals.hpp src/platform/provenance.hpp)
set_target_properties(bike_platform PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_platform PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_platform PRIVATE ${_native_contract_compile_definitions})
endif()
target_include_directories(bike_platform SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(_native_contract_include_directories)
  target_include_directories(bike_platform SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
# Same sweep widening as bike_io: every first-party header compiles in each
# owning target context.
target_include_directories(bike_platform SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(bike_platform PRIVATE bike_io)

# Track-B build identity, same convention as the standalone root graph and
# research_build_identity.hpp: compiler/flags/SDK + the checked dependency
# lock + the T00 source manifest hashed at configure and compiled in. This
# graph IS the transition extension, so the packaging axis records bindings.
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
# not record an identical identity. Map each stored definition back to its
# -D spelling; generator-expression entries still resolve per configuration
# inside file(GENERATE)'s target context.
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

# The pinned error interceptor is compiled once and shared by the extension,
# direct C++ contracts, and the later native benchmark target. Its
# engine-image digest uses the shared bike::io::Sha256 — no private copy.
add_library(bike_native_engine STATIC src/engine_call.cpp)
set_target_properties(bike_native_engine PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(bike_native_engine PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(bike_native_engine PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(bike_native_engine SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(bike_native_engine SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(bike_native_engine PUBLIC "${MJ_DIR}/libmujoco.3.12.0.dylib" bike_io)
set(_native_engine_lib bike_native_engine)

# allocation_faults.cpp overrides the test binary's throwing allocation
# operators — it must never land in a shipped target, so it reaches the
# executable as an injected object, not through the shared contract core.
# The single OBJECT compile lives in tests/migration (shared with
# migration_T07 in both graphs); TARGET_OBJECTS resolves at generate
# time, so the later add_subdirectory definition is fine.
add_executable(native_contract_tests tests/test_contracts.cpp
  $<TARGET_OBJECTS:bike_test_allocation_faults>)
set_target_properties(native_contract_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_contract_tests PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_contract_tests PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_contract_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_contract_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
if(_native_contract_link_options)
  target_link_options(native_contract_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_contract_tests PRIVATE bike_native_contract_core bike_native_engine)
# chain.cpp's GeometryWorkspace resolves CBLAS through Apple Accelerate, the
# same BLAS provider bike_native links.
target_link_libraries(native_contract_tests PRIVATE "-framework Accelerate")
bike_rewrite_mujoco_load_name(native_contract_tests "${MJ_DIR}/libmujoco.3.12.0.dylib")

# tyre/distance.cpp (SignedProfile) contract cases — same build context as
# native_contract_tests, without the allocation-fault override TU.
add_executable(native_distance_tests tests/test_distance.cpp)
set_target_properties(native_distance_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_distance_tests PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_distance_tests PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_distance_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_distance_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
if(_native_contract_link_options)
  target_link_options(native_distance_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_distance_tests PRIVATE bike_native_contract_core bike_native_engine)
# tyre/distance.cpp resolves CBLAS through Apple Accelerate, the same BLAS
# provider bike_native links.
target_link_libraries(native_distance_tests PRIVATE "-framework Accelerate")
bike_rewrite_mujoco_load_name(native_distance_tests "${MJ_DIR}/libmujoco.3.12.0.dylib")

# tyre/station.cpp (scalar station law) contract cases — same build context
# as native_distance_tests.
add_executable(native_station_tests tests/test_station.cpp)
set_target_properties(native_station_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_station_tests PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_station_tests PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_station_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_station_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
if(_native_contract_link_options)
  target_link_options(native_station_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_station_tests PRIVATE bike_native_contract_core bike_native_engine)
target_link_libraries(native_station_tests PRIVATE "-framework Accelerate")
bike_rewrite_mujoco_load_name(native_station_tests "${MJ_DIR}/libmujoco.3.12.0.dylib")

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

add_executable(native_bench
  "${_native_bench_dir}/bench.cpp" "${_native_bench_dir}/state_format.hpp")
set_target_properties(native_bench PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_bench PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_bench PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_bench SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
if(_native_contract_link_options)
  target_link_options(native_bench PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_bench PRIVATE bike_native_engine)
bike_rewrite_mujoco_load_name(native_bench "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()
target_link_libraries(bike_native PRIVATE ${_native_core_lib} ${_native_engine_lib})

get_filename_component(_native_bench_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../tools/proto_native_bench" ABSOLUTE)
if(NOT IS_DIRECTORY "${_native_bench_dir}")
  message(FATAL_ERROR "proto_native_bench directory is missing: ${_native_bench_dir}")
endif()
set(_native_first_party_roots "${CMAKE_CURRENT_SOURCE_DIR}" "${_native_bench_dir}")

include(CTest)
if(BUILD_TESTING AND NOT BIKE_ROOT_BUILD)
  add_test(NAME native_contract_tests COMMAND native_contract_tests)
  add_test(NAME native_distance_tests COMMAND native_distance_tests)
  add_test(NAME native_station_tests COMMAND native_station_tests)
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

# Migration contract-test harness and per-task regressions (T02): one
# executable and one CTest name per task under native/tests/migration. The
# subdirectory inherits this scope's _native_contract_* context and the
# BUILD_TESTING gate from include(CTest) above. In the standalone root build
# the parent already registered the harness (same subdir, same names), so it
# is only added in the standalone legacy configure.
if(NOT BIKE_ROOT_BUILD)
  add_subdirectory(tests/migration)
endif()

add_custom_target(check_native_contracts
  COMMAND uv run --frozen --group native python "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_native_contracts.py"
          --build "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "native structural contract checks"
  VERBATIM)

if(BIKE_ROOT_BUILD)
  # In the standalone root build the extension resolves the locked .deps
  # library (byte-identical to the wheel copy verified above) so no venv
  # path enters the produced binary's load commands.
  target_link_libraries(bike_native PRIVATE bike::mujoco)
else()
  target_link_libraries(bike_native PRIVATE "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()

# The resistance writer calls the legacy CBLAS symbols numpy's `@`/`np.dot`
# resolve to on this platform (numpy's BLAS is Apple Accelerate — see
# writers/resistance.cpp). The symbols stay exported; only the SDK headers
# deprecate them, so the TU declares the prototypes itself.
target_link_libraries(bike_native PRIVATE "-framework Accelerate")

# The dylib's install name (@rpath/mujoco.framework/...) is unresolvable on
# its own — the wheel ships the dylib flat under mujoco/, with no framework
# directory to anchor the @rpath. Rewrite the recorded name to the absolute
# path so `import bike_native` works even before mujoco is loaded.
# (-headerpad_max_install_names above reserves room for the longer name.)
if(BIKE_ROOT_BUILD)
  bike_rewrite_mujoco_load_name(bike_native "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
else()
  bike_rewrite_mujoco_load_name(bike_native "${MJ_DIR}/libmujoco.3.12.0.dylib")
endif()

# Multi-compiler sweep: same TU set through GCC's frontend, warnings-as-errors.
add_custom_target(check_frontends
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_native_frontends.sh
          "${CMAKE_CURRENT_BINARY_DIR}"
          "${CMAKE_CURRENT_SOURCE_DIR}/src" "${MJ_DIR}/include"
  COMMENT "gcc -fsyntax-only sweep (second frontend)")

# clang-tidy sweep: LLVM static-analysis lint over the same TU set, configured
# by native/.clang-tidy and driven by compile_commands.json.
add_custom_target(check_tidy
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_native_tidy.sh
          "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src"
  COMMENT "clang-tidy sweep (LLVM linter)")

# Clang Static Analyzer sweep: path-sensitive per-TU analysis — deeper than
# the clang-analyzer-* subset inside tidy. Needs the brew clang.
add_custom_target(check_analyzer
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_analyzer.sh
          "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src"
  COMMENT "clang static analyzer sweep (path-sensitive)")

# cppcheck sweep: a third analyzer engine with different coverage; suppressions
# for this tree are documented in the script header.
add_custom_target(check_cppcheck
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_cppcheck.sh
          "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src"
          "${Python_EXECUTABLE}"
  COMMENT "cppcheck sweep (third analyzer engine)")

# ODR sweep: -fsyntax-only sees no cross-TU type conflicts — -flto -Wodr at
# link time does.
add_custom_target(check_odr
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_odr.sh
          "${CMAKE_CURRENT_BINARY_DIR}"
          "${CMAKE_CURRENT_SOURCE_DIR}/src" "${MJ_DIR}/include"
          "${Python_EXECUTABLE}"
  COMMENT "gcc -flto -Wodr sweep (cross-TU type conflicts)")

# shellcheck over tools/*.sh — same everything-is-an-error contract.
add_custom_target(check_scripts
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_scripts.sh
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools"
  COMMENT "shellcheck sweep (tools/*.sh)")

# Emit the source and environment contract from the configured build graph.
# Wrappers consume these files instead of independently globbing src/.
function(_native_json_quote output value)
  string(REPLACE "\\" "\\\\" _escaped "${value}")
  string(REPLACE "\"" "\\\"" _escaped "${_escaped}")
  string(REPLACE "\n" "\\n" _escaped "${_escaped}")
  string(REPLACE "\r" "\\r" _escaped "${_escaped}")
  set(${output} "\"${_escaped}\"" PARENT_SCOPE)
endfunction()

function(_native_json_array output)
  set(_json "[")
  set(_separator "")
  foreach(_value IN LISTS ARGN)
    _native_json_quote(_quoted "${_value}")
    string(APPEND _json "${_separator}${_quoted}")
    set(_separator ",")
  endforeach()
  string(APPEND _json "]")
  set(${output} "${_json}" PARENT_SCOPE)
endfunction()

get_filename_component(_native_mujoco_library "${MJ_DIR}/libmujoco.3.12.0.dylib" REALPATH)

set(_native_required_includes
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${_native_python_include}"
  "${_native_nb_include}"
  "${_native_robin_include}"
  "${MJ_DIR}/include")
set(_native_required_files
  "${CMAKE_CURRENT_SOURCE_DIR}/analysis_suppressions.json"
  "${_native_python_include}/Python.h"
  "${_native_nb_include}/nanobind/nanobind.h"
  "${_native_robin_include}/tsl/robin_map.h"
  "${MJ_DIR}/include/mujoco/mujoco.h"
  "${_native_mujoco_library}")
foreach(_required_path IN LISTS _native_required_includes _native_required_files)
  if(NOT EXISTS "${_required_path}")
    message(FATAL_ERROR "Required native check dependency is missing: ${_required_path}")
  endif()
endforeach()

if(CMAKE_BUILD_TYPE)
  set(_native_configuration "${CMAKE_BUILD_TYPE}")
else()
  set(_native_configuration "default")
endif()

set(_native_source_entries "[")
set(_native_source_separator "")
set(_native_target_contexts "{")
set(_native_target_separator "")
get_property(_native_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
# BUILDSYSTEM_TARGETS is scoped to this directory — targets created through
# add_subdirectory() (native/tests/migration) must be collected via
# SUBDIRECTORIES or their first-party translation units never enter the
# manifest while compile_commands.json still compiles them.
get_property(_native_subdirectories DIRECTORY PROPERTY SUBDIRECTORIES)
foreach(_native_subdirectory IN LISTS _native_subdirectories)
  get_property(_native_subdirectory_targets DIRECTORY "${_native_subdirectory}"
    PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND _native_targets ${_native_subdirectory_targets})
endforeach()
list(REMOVE_DUPLICATES _native_targets)
foreach(_target IN LISTS _native_targets)
  get_target_property(_target_type "${_target}" TYPE)
  if(NOT _target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
    continue()
  endif()

  get_target_property(_target_sources "${_target}" SOURCES)
  if(NOT _target_sources OR _target_sources MATCHES "-NOTFOUND$")
    continue()
  endif()
  # SOURCES entries are relative to the directory where the target was
  # created — subdir targets (native/tests/migration) need their own
  # SOURCE_DIR, not this scope's CMAKE_CURRENT_SOURCE_DIR.
  get_target_property(_target_source_dir "${_target}" SOURCE_DIR)
  foreach(_source IN LISTS _target_sources)
    if(_source MATCHES "^\\$<")
      continue()
    endif()
    get_filename_component(_source_absolute "${_source}" ABSOLUTE BASE_DIR "${_target_source_dir}")
    set(_source_is_first_party FALSE)
    foreach(_first_party_root IN LISTS _native_first_party_roots)
      if(NOT _source_is_first_party)
        cmake_path(IS_PREFIX _first_party_root "${_source_absolute}" NORMALIZE _source_is_first_party)
      endif()
    endforeach()
    if(NOT _source_is_first_party OR NOT _source_absolute MATCHES "\\.(cpp|cc|cxx|C)$")
      continue()
    endif()
    get_filename_component(_source_path "${_source_absolute}" REALPATH)
    if(NOT EXISTS "${_source_path}")
      message(FATAL_ERROR "CMake target ${_target} has a missing first-party source: ${_source_path}")
    endif()
    _native_json_quote(_source_json "${_source_path}")
    _native_json_quote(_target_json "${_target}")
    set(_source_context "${_target}:${_native_configuration}")
    _native_json_quote(_source_context_json "${_source_context}")
    string(APPEND _native_source_entries
      "${_native_source_separator}{\"path\":${_source_json},\"target\":${_target_json},\"context\":${_source_context_json}}")
    set(_native_source_separator ",")
  endforeach()

  get_target_property(_target_includes "${_target}" INCLUDE_DIRECTORIES)
  get_target_property(_target_definitions "${_target}" COMPILE_DEFINITIONS)
  get_target_property(_target_options "${_target}" COMPILE_OPTIONS)
  get_target_property(_target_link_dirs "${_target}" LINK_DIRECTORIES)
  get_target_property(_target_link_libs "${_target}" LINK_LIBRARIES)
  get_target_property(_target_standard "${_target}" CXX_STANDARD)
  get_target_property(_target_extensions "${_target}" CXX_EXTENSIONS)
  foreach(_target_property _target_includes _target_definitions _target_options _target_link_dirs _target_link_libs)
    if(NOT ${_target_property} OR "${${_target_property}}" MATCHES "-NOTFOUND$")
      set(${_target_property} "")
    endif()
  endforeach()
  if(NOT _target_standard OR _target_standard MATCHES "-NOTFOUND$")
    set(_target_standard "${CMAKE_CXX_STANDARD}")
  endif()
  if(NOT DEFINED _target_extensions OR _target_extensions MATCHES "-NOTFOUND$")
    set(_target_extensions "${CMAKE_CXX_EXTENSIONS}")
  endif()
  if(_target_extensions)
    set(_target_extensions_json "true")
  else()
    set(_target_extensions_json "false")
  endif()
  set(_resolved_target_includes "")
  foreach(_include IN LISTS _target_includes)
    if(_include MATCHES "^\\$<")
      list(APPEND _resolved_target_includes "${_include}")
    else()
      get_filename_component(_include_absolute "${_include}" ABSOLUTE BASE_DIR "${_target_source_dir}")
      list(APPEND _resolved_target_includes "${_include_absolute}")
    endif()
  endforeach()
  _native_json_array(_target_includes_json ${_resolved_target_includes})
  _native_json_array(_target_definitions_json ${_target_definitions})
  _native_json_array(_target_options_json ${_target_options})
  _native_json_array(_target_link_dirs_json ${_target_link_dirs})
  _native_json_array(_target_link_libs_json ${_target_link_libs})
  _native_json_quote(_target_name_json "${_target}")
  string(APPEND _native_target_contexts
    "${_native_target_separator}${_target_name_json}:{\"configuration\":\"${_native_configuration}\",\"cxx_standard\":${_target_standard},\"cxx_extensions\":${_target_extensions_json},\"include_directories\":${_target_includes_json},\"compile_definitions\":${_target_definitions_json},\"compile_options\":${_target_options_json},\"link_directories\":${_target_link_dirs_json},\"link_libraries\":${_target_link_libs_json}}")
  set(_native_target_separator ",")
endforeach()
string(APPEND _native_source_entries "]")
string(APPEND _native_target_contexts "}")
if(_native_source_separator STREQUAL "")
  message(FATAL_ERROR "No first-party C++ translation units were found in configured CMake targets")
endif()

# First-party roots: native/ first (source_roots[0]'s parent is the
# repository in tools/native_checks.py), then the proto_native_bench tree so
# bench.cpp's target context lands in the same manifest and sweep contract.
_native_json_array(_native_source_roots_json ${_native_first_party_roots})
_native_json_quote(_native_compiler_path_json "${CMAKE_CXX_COMPILER}")
_native_json_quote(_native_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
_native_json_quote(_native_compiler_version_json "${CMAKE_CXX_COMPILER_VERSION}")
_native_json_array(_native_compiler_launcher_json ${CMAKE_CXX_COMPILER_LAUNCHER})
_native_json_quote(_native_sdk_json "${_native_sdk_path}")
_native_json_quote(_native_sdk_requested_json "${_native_sdk_requested}")
_native_json_quote(_native_sdk_source_json "${_native_sdk_source}")
_native_json_quote(_native_suppressions_manifest_json "${CMAKE_CURRENT_SOURCE_DIR}/analysis_suppressions.json")
_native_json_quote(_native_python_json "${Python_EXECUTABLE}")
_native_json_array(_native_include_paths_json ${_native_required_includes})
_native_json_array(_native_library_paths_json "${_native_mujoco_library}" "${_native_sdk_path}/System/Library/Frameworks/Accelerate.framework")
_native_json_array(_native_required_files_json ${_native_required_files})
if(_native_libcpp_hardening_supported)
  set(_native_libcpp_hardening_supported_json "true")
else()
  set(_native_libcpp_hardening_supported_json "false")
endif()
_native_json_quote(_native_libcpp_hardening_mode_json "${_native_libcpp_hardening_mode}")
_native_json_quote(_native_libcpp_hardening_requested_json "${BIKE_LIBCPP_HARDENING}")
set(_native_libcpp_hardening_json
  "{\"supported\":${_native_libcpp_hardening_supported_json},\"mode\":${_native_libcpp_hardening_mode_json},\"requested_mode\":${_native_libcpp_hardening_requested_json}}")

set(_native_tool_defaults "{\"tidy\":\"/opt/homebrew/opt/llvm/bin/clang-tidy\",\"analyzer\":\"/opt/homebrew/opt/llvm/bin/clang++\",\"cppcheck\":\"cppcheck\",\"gxx\":\"g++-16\"}")
set(_native_tool_overrides "{")
set(_native_tool_separator "")
foreach(_tool_override CLANG_TIDY ANALYZER_CLANG CPPCHECK GXX)
  if(DEFINED ENV{${_tool_override}} AND NOT "$ENV{${_tool_override}}" STREQUAL "")
    _native_json_quote(_tool_name_json "${_tool_override}")
    _native_json_quote(_tool_value_json "$ENV{${_tool_override}}")
    string(APPEND _native_tool_overrides "${_native_tool_separator}${_tool_name_json}:${_tool_value_json}")
    set(_native_tool_separator ",")
  endif()
endforeach()
string(APPEND _native_tool_overrides "}")

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":${_native_source_roots_json},\"sources\":${_native_source_entries}}\n")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/native_check_context.json"
  "{\"schema_version\":1,\"compiler\":{\"path\":${_native_compiler_path_json},\"id\":${_native_compiler_id_json},\"version\":${_native_compiler_version_json},\"launcher\":${_native_compiler_launcher_json}},\"sdk\":${_native_sdk_json},\"sdk_requested\":${_native_sdk_requested_json},\"sdk_source\":${_native_sdk_source_json},\"libcpp_hardening\":${_native_libcpp_hardening_json},\"analysis_suppressions_manifest\":${_native_suppressions_manifest_json},\"include_paths\":${_native_include_paths_json},\"library_paths\":${_native_library_paths_json},\"required_files\":${_native_required_files_json},\"source_roots\":${_native_source_roots_json},\"python_executable\":${_native_python_json},\"tool_defaults\":${_native_tool_defaults},\"tool_overrides\":${_native_tool_overrides},\"target_contexts\":${_native_target_contexts}}\n")

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
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/research_identity/$<CONFIG>/research_build_identity.hpp"
  CONTENT "// Generated by CMake for this configuration; do not edit.\n#pragma once\nnamespace runtime::build_identity {\ninline constexpr const char native_source_sha256[] = \"${_research_identity_sha256}\";\ninline constexpr const char context_json[] = R\"identity({\"build_type\":\"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\",\"compiler_id\":${_native_compiler_id_json},\"compiler_version\":${_native_compiler_version_json},\"numerical_flags\":${_research_effective_flags_json}})identity\";\n}\n"
  TARGET bike_native)
