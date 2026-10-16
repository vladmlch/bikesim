cmake_minimum_required(VERSION 3.27)
# CMake performs its compiler link check during project(), so platform
# dispatch — Apple SDK resolution on Darwin, the null SDK contract on Linux —
# runs before that call. Never replace an SDK or compiler supplied by the
# caller; see cmake/BikePlatform.cmake and cmake/BikeAppleSDK.cmake.
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
# time, probes the real compiler/standard library for every flag it emits
# (required flags fail configuration), resolves the compiler-matched
# sanitizer runtime for enabled lanes and runs a clean instrumented
# control under the preload variable tools/native_runtime.py injects, and
# writes bike_policy_report.json for tests/portability/test_compiler_policy.py.
# Include before BikeNumerics.cmake — bike_native_numerics' required
# bike_apply_native_policy call fails configuration if this ordering
# regresses (a CXX-enabled project without the policy module is an error,
# not a skipped flag list).
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
# call face has no definitions without it.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeNumerics.cmake")

# Portable linkage contract: engine/dynamic-loader edges, numerics-provider
# linkage, per-platform runtime search paths (BUILD_RPATH on Linux — never
# environment rescue), and the Darwin MuJoCo install-name rewrite. Helpers
# live in cmake/BikeLinkage.cmake; every binary below calls the ones its
# dependency usage declares.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeLinkage.cmake")

# Standard-library detection, hardening-mode selection and every flag probe
# moved into cmake/BikeCompilerPolicy.cmake (included before the dependency
# modules above): _native_uses_libcpp / _native_libcpp_hardening_supported /
# _native_libcpp_hardening_mode are exported by that module.
# NB_SUPPRESS_WARNINGS: nanobind/Python headers land as SYSTEM includes —
# third-party TU internals stay outside our warning contract.
# PROTECT_STACK: nanobind otherwise appends -fno-stack-protector in Release
# configs, which would silently nullify -fstack-protector-strong below.
nanobind_add_module(bike_native NB_SUPPRESS_WARNINGS PROTECT_STACK
  src/binding.cpp src/stepper.cpp
  src/drivetrain/policy_binding.cpp
  src/drivetrain/drive_binding.cpp
  src/rider/attachment_binding.cpp
  src/rider/attachment_wrench.cpp
  src/rider/contact_binding.cpp
  src/rider/equality_reactions.cpp
  src/rider/intent.cpp
  src/rider/intent_wire.cpp
  src/rider/rider_contact_binding.cpp
  src/rider/spindle_controller.cpp
  src/rider/spindle_math.cpp
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
  src/runtime/test_adapter.cpp
  src/writers/rider_contacts.cpp)

set_target_properties(bike_native PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

# The full compiler/architecture/standard-library policy — warning list,
# required numerical/safety flags (-ffp-contract=off keeps bitwise parity
# with the Python oracle), Release -O2, libc++ hardening / _FORTIFY_SOURCE,
# and the NATIVE_SANITIZE/NATIVE_RTSAN/NATIVE_COVERAGE instrumentation —
# is selected by cmake/BikeCompilerPolicy.cmake and applied DIRECTLY here
# and to every other first-party target below via bike_apply_native_policy.
if(CMAKE_CONFIGURATION_TYPES)
  set(_research_identity_configuration "$<CONFIG>")
else()
  set(_research_identity_configuration "${CMAKE_BUILD_TYPE}")
endif()
target_include_directories(bike_native PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/research_identity/${_research_identity_configuration}")
bike_apply_native_policy(bike_native)
# BIKE_MUJOCO_COMPAT_INCLUDE (BikeDependencies.cmake) precedes the real
# mujoco include dir only when the toolchain cannot parse mjsan.h under
# the sanitizer lane — it carries a generated attribute-compatible
# shadow header, never a forked copy.
target_include_directories(bike_native SYSTEM PRIVATE
  ${BIKE_MUJOCO_COMPAT_INCLUDE} "${MJ_DIR}/include")

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
add_library(bike_native_contract_core OBJECT src/drivetrain/pedaling.cpp
  src/drivetrain/chain.cpp src/drivetrain/transmission.cpp
  src/drivetrain/shifting.cpp src/drivetrain/motor.cpp
  src/drivetrain/freehub.cpp src/writers/drivetrain.cpp
  src/writers/cruise.cpp src/tyre/profile.cpp
  src/tyre/distance.cpp src/tyre/station.cpp
  src/pneumatic/geometry.cpp src/pneumatic/carcass.cpp
  src/pneumatic/brush.cpp src/pneumatic/model.cpp
  src/rider/contact_math.cpp src/rider/support_geometry.cpp
  src/rider/least_squares.cpp src/runtime/samples.cpp
  src/runtime/wire.cpp
  src/runtime/control.cpp
  src/writers/suspension.cpp src/writers/brake.cpp
  src/writers/resistance.cpp src/writers/rider_forces.cpp
  src/writers/tire_common.cpp src/writers/tire.cpp
  src/writers/distributed_tire.cpp src/writers/pneumatic_tire.cpp)
set_target_properties(bike_native_contract_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
# Every first-party target below receives the compiler policy DIRECTLY via
# bike_apply_native_policy (BikeCompilerPolicy.cmake) — no transitive-only
# propagation. What remains target-specific is the shared include context:
# the extension's private roots plus the third-party SYSTEM roots, mirrored
# so the standalone-header sweep sees identical -isystem state per target.
get_target_property(_native_contract_include_directories bike_native INCLUDE_DIRECTORIES)
if(TARGET bike_native_numerics)
  # bike_native_numerics already received bike_apply_native_policy inside
  # BikeNumerics.cmake; only the shared include roots still mirror here.
  if(_native_contract_include_directories)
    target_include_directories(bike_native_numerics SYSTEM PRIVATE
      ${_native_contract_include_directories})
  endif()
  target_include_directories(bike_native_numerics SYSTEM PRIVATE
    "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
endif()
bike_apply_native_policy(bike_native_contract_core)
if(_native_contract_include_directories)
  target_include_directories(bike_native_contract_core SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(bike_native_contract_core SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(bike_native PRIVATE bike_native_contract_core)

# The pinned error interceptor is compiled once and shared by the extension,
# direct C++ contracts, and the later native benchmark target.
add_library(bike_native_engine STATIC src/engine_call.cpp)
set_target_properties(bike_native_engine PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(bike_native_engine)
if(_native_contract_include_directories)
  target_include_directories(bike_native_engine SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(bike_native_engine SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
# The engine image and the platform dynamic-loader library are part of the
# engine library's interface — every consumer links its objects.
bike_link_engine(bike_native_engine PUBLIC)
target_link_libraries(bike_native PRIVATE bike_native_engine)
# The module itself links the engine image (dladdr provenance + the lazy
# mjf/mjb entry points) and the typed numerics adapter used by the blas::
# call face in resistance/tire/etc.; the Darwin install-name rewrite makes
# `import bike_native` work before mujoco is loaded.
bike_link_engine(bike_native)
bike_link_numerics(bike_native)
bike_fix_mujoco_install_name(bike_native)

# allocation_faults.cpp overrides the test binary's throwing allocation
# operators — it must never land in a shipped target, so it joins the
# executable's own sources, not the shared contract core.
add_executable(native_contract_tests tests/test_contracts.cpp
  tests/allocation_faults.cpp)
set_target_properties(native_contract_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_contract_tests)
if(_native_contract_include_directories)
  target_include_directories(native_contract_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_contract_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(native_contract_tests PRIVATE
  bike_native_contract_core bike_native_engine Threads::Threads)
# chain.cpp's GeometryWorkspace resolves CBLAS through the typed adapter —
# Bike::Numerics carries the qualified provider transitively (Accelerate
# here, the scipy-openblas wheel .so on Linux). The engine edge supplies
# MuJoCo + the dynamic-loader library and, on Linux, the runtime search
# path; the Darwin rewrite fixes the wheel's unanchored install name.
bike_link_engine(native_contract_tests)
bike_link_numerics(native_contract_tests)
bike_fix_mujoco_install_name(native_contract_tests)

# tyre/distance.cpp (SignedProfile) contract cases — same build context as
# native_contract_tests, without the allocation-fault override TU.
add_executable(native_distance_tests tests/test_distance.cpp)
set_target_properties(native_distance_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_distance_tests)
if(_native_contract_include_directories)
  target_include_directories(native_distance_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_distance_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(native_distance_tests PRIVATE bike_native_contract_core bike_native_engine)
# tyre/distance.cpp resolves CBLAS through the typed adapter — Bike::Numerics
# carries the qualified provider transitively.
bike_link_engine(native_distance_tests)
bike_link_numerics(native_distance_tests)
bike_fix_mujoco_install_name(native_distance_tests)

# tyre/station.cpp (scalar station law) contract cases — same build context
# as native_distance_tests.
add_executable(native_station_tests tests/test_station.cpp)
set_target_properties(native_station_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_station_tests)
if(_native_contract_include_directories)
  target_include_directories(native_station_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
target_include_directories(native_station_tests SYSTEM PRIVATE
  "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
target_link_libraries(native_station_tests PRIVATE bike_native_contract_core bike_native_engine)
bike_link_engine(native_station_tests)
bike_link_numerics(native_station_tests)
bike_fix_mujoco_install_name(native_station_tests)

# tests/test_platform_blas.cpp — the standalone CBLAS adapter harness. It
# calls only blas:: wrappers and std headers, so Bike::Numerics alone
# supplies the backend definitions plus the qualified provider link.
add_executable(native_platform_blas_tests tests/test_platform_blas.cpp)
set_target_properties(native_platform_blas_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_platform_blas_tests)
if(_native_contract_include_directories)
  target_include_directories(native_platform_blas_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
bike_link_numerics(native_platform_blas_tests)

# tests/test_platform_lapack.cpp — the standalone LAPACK adapter harness.
# It calls only numeric_backend::dgelsd and std headers, so
# Bike::Numerics alone supplies the backend definition plus the
# qualified provider link.
add_executable(native_platform_lapack_tests tests/test_platform_lapack.cpp)
set_target_properties(native_platform_lapack_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_platform_lapack_tests)
if(_native_contract_include_directories)
  target_include_directories(native_platform_lapack_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
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
set_target_properties(native_bench PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
bike_apply_native_policy(native_bench)
if(_native_contract_include_directories)
  target_include_directories(native_bench SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
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
# RealtimeSanitizer diagnostic. Built through the same policy application
# as every first-party target: an NATIVE_RTSAN tree compiles it with the
# lane's instrumentation, a normal tree compiles it with BIKE_NONBLOCKING
# expanded to nothing — which is itself the load-bearing annotation check.
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

# Compile-time numerical provider policy — the qualified selection baked into
# the binary. The extension's runtime_identity() returns this record together
# with the provider images actually loaded (dl_iterate_phdr/_dyld inspection
# in provenance.cpp); Python-side provenance cross-checks the two before any
# native execution, so a substituted or drifted provider is a named failure
# rather than silent bitwise drift. The values below are read back from the
# same schema-1 dependency document BikeNumerics.cmake already qualified.
set(_identity_required_images "")
string(JSON _identity_library_count ERROR_VARIABLE _identity_json_error
  LENGTH "${_native_dependencies_json}" numerics library_paths)
if(_identity_json_error OR NOT _identity_library_count MATCHES "^[0-9]+$"
   OR _identity_library_count LESS 1)
  message(FATAL_ERROR
    "numerics provider identity is missing from the dependency document")
endif()
math(EXPR _identity_library_last "${_identity_library_count} - 1")
set(_identity_libraries_json "")
foreach(_index RANGE ${_identity_library_last})
  foreach(_field path role identity_kind)
    string(JSON _identity_member ERROR_VARIABLE _identity_json_error
      GET "${_native_dependencies_json}" numerics library_paths ${_index} ${_field})
    if(_identity_json_error OR "${_identity_member}" MATCHES "-NOTFOUND$")
      message(FATAL_ERROR
        "numerics provider ${_index} is missing '${_field}'")
    endif()
    _native_json_quote(_identity_${_field}_json "${_identity_member}")
    if(_field STREQUAL "path")
      list(APPEND _identity_required_images "${_identity_member}")
    endif()
  endforeach()
  foreach(_field sha256 build_id)
    string(JSON _identity_member_type TYPE "${_native_dependencies_json}"
      numerics library_paths ${_index} ${_field})
    if(_identity_member_type STREQUAL "STRING")
      string(JSON _identity_member GET "${_native_dependencies_json}"
        numerics library_paths ${_index} ${_field})
      _native_json_quote(_identity_${_field}_json "${_identity_member}")
    else()
      set(_identity_${_field}_json "null")
    endif()
  endforeach()
  if(_index GREATER 0)
    string(APPEND _identity_libraries_json ",")
  endif()
  string(APPEND _identity_libraries_json
    "{\"path\":${_identity_path_json},\"role\":${_identity_role_json},\"identity_kind\":${_identity_identity_kind_json},\"sha256\":${_identity_sha256_json},\"build_id\":${_identity_build_id_json}}")
endforeach()
if(BIKE_NUMERICS_BACKEND_RESOLVED STREQUAL "ACCELERATE")
  # The umbrella image carries the LC_UUID used for apple-system identity.
  list(APPEND _identity_required_images
    "/System/Library/Frameworks/Accelerate.framework/Versions/A/Accelerate")
endif()
if(NOT _identity_required_images)
  message(FATAL_ERROR "no required provider images were recorded")
endif()
# Newline-separated C literal: provider image names are probe-qualified loader
# paths, which never contain newlines, quotes or backslashes.
string(JOIN "\\n" _identity_images_literal ${_identity_required_images})
_native_json_quote(_identity_backend_json "${BIKE_NUMERICS_BACKEND_RESOLVED}")
set(_identity_policy_json
  "{\"backend\":${_identity_backend_json},\"blas_integer_bits\":${_native_numerics_blas_bits},\"lapack_integer_bits\":${_native_numerics_lapack_bits},\"libraries\":[${_identity_libraries_json}]}")

file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/research_identity/$<CONFIG>/research_build_identity.hpp"
  CONTENT "// Generated by CMake for this configuration; do not edit.\n#pragma once\nnamespace runtime::build_identity {\ninline constexpr const char native_source_sha256[] = \"${_research_identity_sha256}\";\ninline constexpr const char context_json[] = R\"identity({\"build_type\":\"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\",\"compiler_id\":${_identity_compiler_id_json},\"compiler_version\":${_identity_compiler_version_json},\"numerical_flags\":${_research_effective_flags_json}})identity\";\ninline constexpr const char numerics_provider_policy_json[] = R\"identity(${_identity_policy_json})identity\";\ninline constexpr const char required_provider_images[] = \"${_identity_images_literal}\";\n}\n"
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
