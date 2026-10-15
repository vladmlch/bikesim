cmake_minimum_required(VERSION 3.24)
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

set(BIKE_LIBCPP_HARDENING "EXTENSIVE" CACHE STRING "libc++ hardening mode")
set_property(CACHE BIKE_LIBCPP_HARDENING PROPERTY STRINGS EXTENSIVE FAST)
if(NOT BIKE_LIBCPP_HARDENING STREQUAL "EXTENSIVE" AND NOT BIKE_LIBCPP_HARDENING STREQUAL "FAST")
  message(FATAL_ERROR "BIKE_LIBCPP_HARDENING must be EXTENSIVE or FAST, got '${BIKE_LIBCPP_HARDENING}'")
endif()

# The compiler identification check above is not enough on Apple hosts:
# compile a source that includes an SDK header with the selected compiler and
# sysroot. The probe is Apple-only; Linux has no Availability.h.
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  bike_verify_apple_sdk_compatibility()
endif()

# Resolve the interpreter first so tools/native_dependencies.py is probed by
# the SAME python we compile against (uv run exposes .venv via VIRTUAL_ENV).
find_package(Python 3.13 REQUIRED COMPONENTS Interpreter Development.Module)

# Dependency discovery: BikeDependencies.cmake runs the probe inside the
# selected interpreter, validates the schema-1 document it returns, then
# creates the imported Bike::MuJoCo target plus MJ_DIR/_native_mujoco_library
# and the nanobind/python include roots consumed below.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeDependencies.cmake")

# PROVISIONAL T07 wiring ahead of T10: BikeNumerics.cmake consumes the same
# schema-1 document, generates numerics_symbols.hpp, and builds the one
# selected CBLAS backend TU into bike_native_numerics (alias Bike::Numerics).
# The blas:: call face has no definitions without it — the link below is the
# minimum that keeps this tree buildable; T10 owns the final target-graph
# integration (policy application, LAPACK adapter, remaining consumers).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BikeNumerics.cmake")

set(_native_hardening_probe_dir "${CMAKE_BINARY_DIR}/native_check_work/libcpp-hardening")
file(MAKE_DIRECTORY "${_native_hardening_probe_dir}")
set(_native_libcpp_probe_source "${_native_hardening_probe_dir}/detect_libcpp.cpp")
file(WRITE "${_native_libcpp_probe_source}"
  "#include <__config>\n"
  "#ifndef _LIBCPP_VERSION\n#error The configured standard library is not libc++\n#endif\n"
  "int main() { return 0; }\n")
unset(_native_uses_libcpp CACHE)
try_compile(_native_uses_libcpp
  "${_native_hardening_probe_dir}/detect-build"
  SOURCES "${_native_libcpp_probe_source}"
  CMAKE_FLAGS
    "-DCMAKE_OSX_SYSROOT:PATH=${CMAKE_OSX_SYSROOT}"
    "-DCMAKE_CXX_STANDARD:STRING=23"
    "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
  OUTPUT_VARIABLE _native_libcpp_detection_output)

set(_native_libcpp_hardening_supported FALSE)
if(_native_uses_libcpp)
  set(_native_hardening_mode_probe_source "${_native_hardening_probe_dir}/check_hardening_mode.cpp")
  file(WRITE "${_native_hardening_mode_probe_source}"
    "#include <__config>\n"
    "#ifndef _LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}\n"
    "#error libc++ omitted the selected hardening mode macro\n#endif\n"
    "#ifndef _LIBCPP_HARDENING_MODE\n#error libc++ omitted the hardening mode selection\n#endif\n"
    "#if _LIBCPP_HARDENING_MODE != _LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}\n"
    "#error libc++ did not accept the selected hardening mode\n#endif\n"
    "int main() { return 0; }\n")
  unset(_native_libcpp_mode_probe_result)
  unset(_native_libcpp_mode_probe_result CACHE)
  try_compile(_native_libcpp_mode_probe_result
    "${_native_hardening_probe_dir}/mode-build"
    SOURCES "${_native_hardening_mode_probe_source}"
    CMAKE_FLAGS
      "-DCMAKE_OSX_SYSROOT:PATH=${CMAKE_OSX_SYSROOT}"
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS
      "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}"
    OUTPUT_VARIABLE _native_libcpp_hardening_output)
  if(NOT _native_libcpp_mode_probe_result)
    message(FATAL_ERROR
      "Configured libc++ does not support hardening mode ${BIKE_LIBCPP_HARDENING}:\n${_native_libcpp_hardening_output}")
  endif()
  set(_native_libcpp_hardening_supported TRUE)
  set(_native_libcpp_hardening_mode "${BIKE_LIBCPP_HARDENING}")
else()
  message(STATUS "libc++ hardening is not applied: configured standard library is not libc++")
  set(_native_libcpp_hardening_mode "NOT_APPLICABLE")
endif()

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
target_compile_options(bike_native PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
  -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wnull-dereference -Wunsafe-buffer-usage
  # Declarations / ODR hygiene: free functions without a header decl get
  # flagged so internal linkage is a deliberate choice, not an accident.
  -Wmissing-declarations -Wmissing-variable-declarations -Wmissing-noreturn
  -Wredundant-decls -Wredeclared-class-member -Wstatic-in-inline
  # Casts beyond -Wcast-qual (no -Wcast-align=strict on Apple clang).
  -Wcast-align -Wcast-function-type-strict -Wreinterpret-base-class
  # Dead or conditionally-uninitialized code paths.
  -Wunreachable-code-break -Wunreachable-code-return
  -Wunreachable-code-aggressive -Wconditional-uninitialized
  -Wloop-analysis -Wrange-loop-analysis -Winfinite-recursion
  # Inheritance hygiene.
  -Winconsistent-missing-override -Winconsistent-missing-destructor-override
  -Wsuggest-override -Wsuggest-destructor-override
  -Wweak-vtables -Wweak-template-vtables
  -Wdelete-non-abstract-non-virtual-dtor -Wabstract-vbase-init
  -Wctad-maybe-unsupported
  # enum/switch discipline.
  -Wswitch-enum -Wcovered-switch-default -Wassign-enum
  -Wbitfield-enum-conversion -Wsigned-enum-bitfield -Wduplicate-enum
  # Reserved identifiers/macros/UDLs.
  -Wreserved-identifier -Wreserved-macro-identifier
  -Wreserved-user-defined-literal
  # Ambiguity and latent UB.
  -Wunevaluated-expression -Wnull-pointer-arithmetic
  -Warray-bounds-pointer-arithmetic -Watomic-implicit-seq-cst
  -Wxor-used-as-pow -Wdangling
  -Widiomatic-parentheses -Wredundant-parens -Wlogical-not-parentheses
  -Wshift-op-parentheses
  -Wunguarded-availability -Wunguarded-availability-new
  -Wunicode-homoglyph -Wunicode-whitespace -Wunicode-zero-width
  # Memory APIs and object layout.
  -Wmemset-transposed-args -Wsuspicious-bzero -Wsuspicious-memaccess
  -Wnontrivial-memaccess -Wdynamic-class-memaccess
  -Wsizeof-array-div -Wsizeof-pointer-div -Wsizeof-pointer-memaccess
  -Wover-aligned -Wpacked -Walloca -Walloca-with-align-alignof
  -Wvla -Wzero-length-array
  # Strings and format strings.
  -Wstring-compare -Wstring-concatenation -Wstring-plus-int
  -Wformat-pedantic -Wformat-type-confusion -Wfortify-source
  # General hygiene / portability.
  -Wheader-hygiene -Wcomma -Wkeyword-macro -Wlanguage-extension-token
  -Wextra-tokens -Wextra-semi-stmt -Wignored-qualifiers -Wextra-qualification
  -Wnonportable-include-path -Wnonportable-system-include-path
  -Wunneeded-member-function -Wunneeded-internal-declaration
  -Wunnamed-type-template-args -Wclass-varargs
  -Wpredefined-identifier-outside-function -Wduplicate-method-arg
  -Wmultichar -Wfour-char-constants -Wdate-time -Wnewline-eof
  -Wused-but-marked-unused -Winit-self -Wsign-promo -Wctor-dtor-privacy
  -Wpass-failed -Wstack-protector -Wlarge-by-value-copy
  -Wframe-larger-than=8192
  # Deliberately NOT enabled (verified on Apple clang 21):
  #   noise tier: -Wfloat-equal -Wpadded -Wexit-time-destructors
  #     -Wglobal-constructors -Wzero-as-null-pointer-constant -Wdocumentation
  #   unsupported here: -Wcast-align=strict -Wenum-constexpr-conversion
  #     -Wfixed-enum-extension -fsanitize=safe-stack/memory/dataflow
  #   dead on target: -fstack-clash-protection (accepted, unused),
  #     -fsanitize=thread (compiles, runtime aborts on arm64-darwin)
  -ffp-contract=off
  -fstack-protector-strong -ftrivial-auto-var-init=zero -fvisibility=hidden
  -fvisibility-inlines-hidden -fstrict-flex-arrays=3
  -mbranch-protection=standard)
# The previous Release compile line ended in -Os (CMAKE's -O3, then
# nanobind's -Os, last wins); -O2 measures ~7% faster than that stack on
# this runtime's branchy code (verified arm64-apple, ~0.181 vs ~0.194
# ms/step on the same session). Appending -O2 here overrides inherited -O
# flags since the last one wins; this does not stomp a caller's -O choice
# for other build types, and the option is recorded into
# research_identity flags.
target_compile_options(bike_native PRIVATE $<$<CONFIG:Release>:-O2>)
target_include_directories(bike_native SYSTEM PRIVATE "${MJ_DIR}/include")
# libc++ hardening modes do not change container layout. Keep the selected mode
# consistent across this target's TUs; EXTENSIVE is the default in every build
# type, while FAST is an explicit comparison option.
if(_native_libcpp_hardening_supported)
  target_compile_definitions(bike_native PRIVATE
    "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}")
endif()
target_link_options(bike_native PRIVATE -headerpad_max_install_names)

option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
if(NOT NATIVE_SANITIZE)
  # The ASan driver predefines _FORTIFY_SOURCE (and intercepts the same
  # functions anyway) — redefining it would trip -Wmacro-redefined.
  target_compile_definitions(bike_native PRIVATE _FORTIFY_SOURCE=3)
endif()
if(NATIVE_SANITIZE)
  # local-bounds: checked access through bounds of stack/heap objects.
  # float-cast-overflow: NOT part of the undefined group — out-of-range
  # float→int conversions at the FFI boundary (tooth counts etc.).
  # no-recover: UBSan findings abort instead of logging-and-continuing, so a
  # violation under pytest can never pass silently.
  # Available-but-rejected extras (verified arm64-apple): implicit-conversion
  # and unsigned-integer-overflow are too noisy for numerics; thread/msan/
  # cfi/safe-stack are unsupported or dead on this target.
  # The 8192-byte -Wframe-larger-than budget guards production stack use;
  # ASan wraps every alloca in >=32-byte redzones, roughly tripling the
  # measured frame (DistributedTireWriter::accumulate_validated reports
  # 9152 bytes instrumented), so the instrumented budget is raised 3x —
  # appended last, clang's repeated-flag last-wins semantics verified on
  # Apple clang 21.
  target_compile_options(bike_native PRIVATE
    -fsanitize=address,undefined,local-bounds,float-cast-overflow
    -fno-sanitize-recover=all -fno-omit-frame-pointer
    -Wframe-larger-than=24576)
  target_link_options(bike_native PRIVATE
    -fsanitize=address,undefined,local-bounds,float-cast-overflow -shared-libasan)
endif()

# RTSan (-fsanitize=realtime): flags allocation/locking inside
# [[clang::nonblocking]] functions (BIKE_NONBLOCKING in rtsan.hpp). Requires
# the BREW clang — Apple clang 21 lacks the sanitizer — so configure a
# dedicated build dir:
#   cmake -S native -B native/build/rtsan \
#     -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -DNATIVE_RTSAN=ON
# and run native tests with the rtsan dylib in DYLD_INSERT_LIBRARIES, same
# recipe as the asan build. Verified arm64-apple: mj_step/mj_forward do not
# allocate, so the marked hot path is clean today.
option(NATIVE_RTSAN "RealtimeSanitizer build (brew clang only)" OFF)
if(NATIVE_RTSAN)
  target_compile_options(bike_native PRIVATE -fsanitize=realtime)
  target_link_options(bike_native PRIVATE -fsanitize=realtime)
endif()

# NATIVE_COVERAGE: llvm-cov source coverage over the native TUs. Build into a
# dedicated dir (-DNATIVE_COVERAGE=ON), then tools/native_coverage.sh runs
# the native tests, merges profraw and prints the uncovered-line report —
# shows which ported code the oracle suite never reaches.
option(NATIVE_COVERAGE "llvm-cov instrumented build" OFF)
if(NATIVE_COVERAGE)
  target_compile_options(bike_native PRIVATE
    -fprofile-instr-generate -fcoverage-mapping)
  target_link_options(bike_native PRIVATE -fprofile-instr-generate)
endif()

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
get_target_property(_native_contract_compile_options bike_native COMPILE_OPTIONS)
get_target_property(_native_contract_compile_definitions bike_native COMPILE_DEFINITIONS)
get_target_property(_native_contract_include_directories bike_native INCLUDE_DIRECTORIES)
if(TARGET bike_native_numerics)
  # The language/hardening contract and the standalone-header sweep inspect
  # every first-party TU under its own target's context — the adapter must
  # carry the same contract options/definitions and third-party include
  # roots as the contract targets, even though its own TU only needs
  # mujoco.h plus the generated symbol header.
  if(_native_contract_compile_options)
    target_compile_options(bike_native_numerics PRIVATE ${_native_contract_compile_options})
  endif()
  if(_native_contract_compile_definitions)
    target_compile_definitions(bike_native_numerics PRIVATE ${_native_contract_compile_definitions})
  endif()
  if(_native_contract_include_directories)
    target_include_directories(bike_native_numerics SYSTEM PRIVATE
      ${_native_contract_include_directories})
  endif()
  target_include_directories(bike_native_numerics SYSTEM PRIVATE
    "${_native_python_include}" "${_native_nb_include}" "${_native_robin_include}")
endif()
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
target_link_libraries(bike_native PRIVATE bike_native_contract_core)

# The pinned error interceptor is compiled once and shared by the extension,
# direct C++ contracts, and the later native benchmark target.
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
target_link_libraries(bike_native_engine PUBLIC Bike::MuJoCo)
target_link_libraries(bike_native PRIVATE bike_native_engine)

# allocation_faults.cpp overrides the test binary's throwing allocation
# operators — it must never land in a shipped target, so it joins the
# executable's own sources, not the shared contract core.
add_executable(native_contract_tests tests/test_contracts.cpp
  tests/allocation_faults.cpp)
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
get_target_property(_native_contract_link_options bike_native LINK_OPTIONS)
if(_native_contract_link_options)
  target_link_options(native_contract_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_contract_tests PRIVATE bike_native_contract_core bike_native_engine)
# chain.cpp's GeometryWorkspace resolves CBLAS through the typed adapter —
# Bike::Numerics carries the qualified provider transitively (Accelerate
# here, the scipy-openblas wheel .so on Linux).
target_link_libraries(native_contract_tests PRIVATE Bike::Numerics)
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  add_custom_command(TARGET native_contract_tests POST_BUILD
    COMMAND install_name_tool -change
      "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
      "${MJ_DIR}/libmujoco.3.12.0.dylib"
      "$<TARGET_FILE:native_contract_tests>"
    VERBATIM)
endif()

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
# tyre/distance.cpp resolves CBLAS through the typed adapter — Bike::Numerics
# carries the qualified provider transitively.
target_link_libraries(native_distance_tests PRIVATE Bike::Numerics)
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  add_custom_command(TARGET native_distance_tests POST_BUILD
    COMMAND install_name_tool -change
      "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
      "${MJ_DIR}/libmujoco.3.12.0.dylib"
      "$<TARGET_FILE:native_distance_tests>"
    VERBATIM)
endif()

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
target_link_libraries(native_station_tests PRIVATE Bike::Numerics)
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  add_custom_command(TARGET native_station_tests POST_BUILD
    COMMAND install_name_tool -change
      "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
      "${MJ_DIR}/libmujoco.3.12.0.dylib"
      "$<TARGET_FILE:native_station_tests>"
    VERBATIM)
endif()

# tests/test_platform_blas.cpp — the standalone CBLAS adapter harness. It
# calls only blas:: wrappers and std headers, so Bike::Numerics alone
# supplies the backend definitions plus the qualified provider link.
add_executable(native_platform_blas_tests tests/test_platform_blas.cpp)
set_target_properties(native_platform_blas_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_platform_blas_tests PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_platform_blas_tests PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_platform_blas_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
if(_native_contract_link_options)
  target_link_options(native_platform_blas_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_platform_blas_tests PRIVATE Bike::Numerics)

# tests/test_platform_lapack.cpp — the standalone LAPACK adapter harness.
# It calls only numeric_backend::dgelsd and std headers, so
# Bike::Numerics alone supplies the backend definition plus the
# qualified provider link.
add_executable(native_platform_lapack_tests tests/test_platform_lapack.cpp)
set_target_properties(native_platform_lapack_tests PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
if(_native_contract_compile_options)
  target_compile_options(native_platform_lapack_tests PRIVATE ${_native_contract_compile_options})
endif()
if(_native_contract_compile_definitions)
  target_compile_definitions(native_platform_lapack_tests PRIVATE ${_native_contract_compile_definitions})
endif()
if(_native_contract_include_directories)
  target_include_directories(native_platform_lapack_tests SYSTEM PRIVATE
    ${_native_contract_include_directories})
endif()
if(_native_contract_link_options)
  target_link_options(native_platform_lapack_tests PRIVATE ${_native_contract_link_options})
endif()
target_link_libraries(native_platform_lapack_tests PRIVATE Bike::Numerics)

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
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  add_custom_command(TARGET native_bench POST_BUILD
    COMMAND install_name_tool -change
      "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
      "${MJ_DIR}/libmujoco.3.12.0.dylib"
      "$<TARGET_FILE:native_bench>"
    VERBATIM)
endif()

include(CTest)
if(BUILD_TESTING)
  add_test(NAME native_contract_tests COMMAND native_contract_tests)
  add_test(NAME native_distance_tests COMMAND native_distance_tests)
  add_test(NAME native_station_tests COMMAND native_station_tests)
  add_test(NAME native_platform_blas_tests COMMAND native_platform_blas_tests)
  add_test(NAME native_platform_lapack_tests COMMAND native_platform_lapack_tests)
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

target_link_libraries(bike_native PRIVATE Bike::MuJoCo)

# The blas:: call face (resistance/tire/etc.) resolves through the typed
# adapter: Bike::Numerics provides the one selected backend TU and carries
# the qualified provider transitively — Accelerate on Darwin, the
# scipy-openblas wheel .so on Linux.
target_link_libraries(bike_native PRIVATE Bike::Numerics)

# The dylib's install name (@rpath/mujoco.framework/...) is unresolvable on
# its own — the wheel ships the dylib flat under mujoco/, with no framework
# directory to anchor the @rpath. Rewrite the recorded name to the absolute
# path so `import bike_native` works even before mujoco is loaded.
# (-headerpad_max_install_names above reserves room for the longer name.)
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  add_custom_command(TARGET bike_native POST_BUILD
    COMMAND install_name_tool -change
      "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
      "${MJ_DIR}/libmujoco.3.12.0.dylib"
      "$<TARGET_FILE:bike_native>"
    VERBATIM)
endif()

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

# _native_mujoco_library is set by BikeDependencies.cmake (realpath of the
# selected engine image reported by the probe).

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
foreach(_target IN LISTS _native_targets)
  get_target_property(_target_type "${_target}" TYPE)
  if(NOT _target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
    continue()
  endif()

  get_target_property(_target_sources "${_target}" SOURCES)
  if(NOT _target_sources OR _target_sources MATCHES "-NOTFOUND$")
    continue()
  endif()
  foreach(_source IN LISTS _target_sources)
    if(_source MATCHES "^\\$<")
      continue()
    endif()
    get_filename_component(_source_absolute "${_source}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
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
      get_filename_component(_include_absolute "${_include}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
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

file(WRITE "${CMAKE_BINARY_DIR}/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":${_native_source_roots_json},\"sources\":${_native_source_entries}}\n")
file(WRITE "${CMAKE_BINARY_DIR}/native_check_context.json"
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
