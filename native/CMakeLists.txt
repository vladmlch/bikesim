cmake_minimum_required(VERSION 3.24)
# CMake performs its compiler link check during project(), so resolve a default
# SDK before that call. Never replace an SDK or compiler supplied by the caller.
if(NOT DEFINED CMAKE_OSX_SYSROOT OR CMAKE_OSX_SYSROOT STREQUAL "")
  execute_process(
    COMMAND xcrun --sdk macosx --show-sdk-path
    RESULT_VARIABLE _native_default_sdk_result
    OUTPUT_VARIABLE _native_default_sdk
    ERROR_VARIABLE _native_default_sdk_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(_native_sdk_source "default")
  set(_native_sdk_requested "")
  set(_native_sdk_candidates "")
  if(_native_default_sdk_result EQUAL 0 AND IS_DIRECTORY "${_native_default_sdk}")
    list(APPEND _native_sdk_candidates "${_native_default_sdk}")
  endif()
  execute_process(
    COMMAND xcode-select -p
    RESULT_VARIABLE _native_developer_dir_result
    OUTPUT_VARIABLE _native_developer_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_native_developer_dir_result EQUAL 0)
    file(GLOB _native_xcode_sdks LIST_DIRECTORIES TRUE
      "${_native_developer_dir}/Platforms/MacOSX.platform/Developer/SDKs/MacOSX*.sdk")
    list(SORT _native_xcode_sdks COMPARE NATURAL ORDER DESCENDING)
    list(APPEND _native_sdk_candidates ${_native_xcode_sdks})
  endif()
  file(GLOB _native_command_line_sdks LIST_DIRECTORIES TRUE
    "/Library/Developer/CommandLineTools/SDKs/MacOSX*.sdk")
  list(APPEND _native_sdk_candidates ${_native_command_line_sdks})
  if(NOT _native_default_sdk_result EQUAL 0)
    message(STATUS "xcrun's default macOS SDK was unavailable: ${_native_default_sdk_error}")
  endif()
else()
  set(_native_sdk_source "explicit")
  set(_native_sdk_requested "${CMAKE_OSX_SYSROOT}")
  if(IS_DIRECTORY "${CMAKE_OSX_SYSROOT}")
    get_filename_component(_native_explicit_sdk "${CMAKE_OSX_SYSROOT}" REALPATH)
  elseif(CMAKE_OSX_SYSROOT MATCHES "^[A-Za-z0-9_.-]+$")
    execute_process(
      COMMAND xcrun --sdk "${CMAKE_OSX_SYSROOT}" --show-sdk-path
      RESULT_VARIABLE _native_explicit_sdk_result
      OUTPUT_VARIABLE _native_explicit_sdk
      ERROR_VARIABLE _native_explicit_sdk_error
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _native_explicit_sdk_result EQUAL 0 OR NOT IS_DIRECTORY "${_native_explicit_sdk}")
      message(FATAL_ERROR
        "Explicit CMAKE_OSX_SYSROOT '${CMAKE_OSX_SYSROOT}' is unavailable: ${_native_explicit_sdk_error}")
    endif()
    get_filename_component(_native_explicit_sdk "${_native_explicit_sdk}" REALPATH)
  else()
    message(FATAL_ERROR "Explicit CMAKE_OSX_SYSROOT does not exist: ${CMAKE_OSX_SYSROOT}")
  endif()
  set(_native_sdk_candidates "${_native_explicit_sdk}")
endif()

if(CMAKE_CXX_COMPILER)
  set(_native_sdk_probe_compiler "${CMAKE_CXX_COMPILER}")
else()
  find_program(_native_sdk_probe_compiler NAMES c++ clang++ REQUIRED)
endif()
separate_arguments(_native_sdk_probe_compiler_arg1 NATIVE_COMMAND "${CMAKE_CXX_COMPILER_ARG1}")
separate_arguments(_native_sdk_probe_flags NATIVE_COMMAND "${CMAKE_CXX_FLAGS}")
set(_native_sdk_probe_source "${CMAKE_BINARY_DIR}/native_check_work/sdk-compat.cpp")
get_filename_component(_native_sdk_probe_source_dir "${_native_sdk_probe_source}" DIRECTORY)
file(MAKE_DIRECTORY "${_native_sdk_probe_source_dir}")
file(WRITE "${_native_sdk_probe_source}"
  "#include <Accelerate/Accelerate.h>\n"
  "int main() { double x = 1.0; double y = 2.0; cblas_daxpy(1, 1.0, &x, 1, &y, 1); return y == 3.0 ? 0 : 1; }\n")
set(_native_sdk_probe_arch_args "")
foreach(_architecture IN LISTS CMAKE_OSX_ARCHITECTURES)
  list(APPEND _native_sdk_probe_arch_args -arch "${_architecture}")
endforeach()
set(_native_sdk_probe_launcher ${CMAKE_CXX_COMPILER_LAUNCHER})
set(_native_sdk_probe_failures "")
set(_native_seen_sdk_paths "")
set(_native_sdk_probe_index 0)
foreach(_sdk_candidate IN LISTS _native_sdk_candidates)
  get_filename_component(_sdk_candidate_real "${_sdk_candidate}" REALPATH)
  if(NOT IS_DIRECTORY "${_sdk_candidate_real}")
    continue()
  endif()
  list(FIND _native_seen_sdk_paths "${_sdk_candidate_real}" _native_seen_index)
  if(NOT _native_seen_index EQUAL -1)
    continue()
  endif()
  list(APPEND _native_seen_sdk_paths "${_sdk_candidate_real}")
  math(EXPR _native_sdk_probe_index "${_native_sdk_probe_index} + 1")
  set(_native_sdk_probe_binary "${CMAKE_BINARY_DIR}/native_check_work/sdk-compat-probe-${_native_sdk_probe_index}")
  execute_process(
    COMMAND ${_native_sdk_probe_launcher} "${_native_sdk_probe_compiler}"
      ${_native_sdk_probe_compiler_arg1} ${_native_sdk_probe_flags}
      -std=c++23 ${_native_sdk_probe_arch_args}
      -isysroot "${_sdk_candidate_real}" "${_native_sdk_probe_source}"
      -framework Accelerate -o "${_native_sdk_probe_binary}"
    RESULT_VARIABLE _native_sdk_probe_result
    OUTPUT_VARIABLE _native_sdk_probe_output
    ERROR_VARIABLE _native_sdk_probe_error)
  if(_native_sdk_probe_result EQUAL 0)
    set(_native_sdk_path "${_sdk_candidate_real}")
    if(_native_sdk_source STREQUAL "default")
      set(CMAKE_OSX_SYSROOT "${_native_sdk_path}" CACHE PATH "macOS SDK for bike_native" FORCE)
      set(_native_sdk_requested "${_native_sdk_path}")
    else()
      # Resolve SDK names for compile commands while preserving the caller's
      # cache value and the exact request in the generated context.
      set(CMAKE_OSX_SYSROOT "${_native_sdk_path}")
    endif()
    break()
  endif()
  string(APPEND _native_sdk_probe_failures
    "\nSDK ${_sdk_candidate_real}: ${_native_sdk_probe_error}${_native_sdk_probe_output}")
endforeach()
if(NOT _native_sdk_path)
  if(_native_sdk_source STREQUAL "explicit")
    message(FATAL_ERROR
      "Explicit CMAKE_OSX_SYSROOT '${_native_sdk_requested}' is incompatible with compiler '${_native_sdk_probe_compiler}':${_native_sdk_probe_failures}")
  endif()
  message(FATAL_ERROR
    "No installed macOS SDK is compatible with compiler '${_native_sdk_probe_compiler}':${_native_sdk_probe_failures}")
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

# The compiler identification check above is not enough: compile a source that
# includes an SDK header with the selected compiler and sysroot.
set(_native_sdk_probe_dir "${CMAKE_BINARY_DIR}/native_check_work/sdk-compat")
file(MAKE_DIRECTORY "${_native_sdk_probe_dir}")
set(_native_sdk_probe_source "${_native_sdk_probe_dir}/sdk_compat.cpp")
file(WRITE "${_native_sdk_probe_source}"
  "#include <Availability.h>\n"
  "#ifndef __MAC_OS_X_VERSION_MIN_REQUIRED\n#error No compatible macOS SDK selected\n#endif\n"
  "int main() { return __MAC_OS_X_VERSION_MIN_REQUIRED > 0 ? 0 : 1; }\n")
unset(_native_sdk_compatible CACHE)
try_compile(_native_sdk_compatible
  "${_native_sdk_probe_dir}/build"
  SOURCES "${_native_sdk_probe_source}"
  CMAKE_FLAGS
    "-DCMAKE_OSX_SYSROOT:PATH=${CMAKE_OSX_SYSROOT}"
    "-DCMAKE_CXX_STANDARD:STRING=23"
    "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
  OUTPUT_VARIABLE _native_sdk_probe_output)
if(NOT _native_sdk_compatible)
  message(FATAL_ERROR
    "Configured C++ compiler '${CMAKE_CXX_COMPILER}' is incompatible with SDK '${CMAKE_OSX_SYSROOT}':\n${_native_sdk_probe_output}")
endif()

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
  src/binding.cpp src/stepper.cpp src/writers/suspension.cpp
  src/writers/brake.cpp src/writers/resistance.cpp
  src/writers/rider_forces.cpp src/writers/tire.cpp src/tyre/profile.cpp
  src/drivetrain/shifting.cpp
  src/drivetrain/motor.cpp src/drivetrain/freehub.cpp src/drivetrain/policy_binding.cpp
  src/drivetrain/drive_binding.cpp
  src/writers/drivetrain.cpp
  src/rider/contact_math.cpp src/rider/support_geometry.cpp src/rider/contact_binding.cpp)

set_target_properties(bike_native PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

# Strict flags are PRIVATE to our TUs: nanobind's own static-lib sources are
# not expected to survive this warning set under -Werror.
# -ffp-contract=off: clang's default (on) fuses mul+add into FMA on arm64 —
# one rounding instead of two, breaking bitwise parity with the Python
# oracle, whose interpreter rounds every operation separately.
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
  target_compile_options(bike_native PRIVATE
    -fsanitize=address,undefined,local-bounds,float-cast-overflow
    -fno-sanitize-recover=all -fno-omit-frame-pointer)
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

# Share Python-free drivetrain code with direct native contract tests. The
# object is compiled once with the extension's complete warning, hardening,
# sanitizer, and coverage context, and remains PIC for module linkage.
# The contract binary links these objects directly, so each drivetrain TU
# keeps exactly one translation unit across all targets (the ODR sweep links
# every selected TU into a single image). bike_native picks them up through
# the existing target_link_libraries edge below — same COMPILE_OPTIONS,
# including -ffp-contract=off, as when they were listed on the module itself.
add_library(bike_native_contract_core OBJECT src/drivetrain/pedaling.cpp
  src/drivetrain/chain.cpp src/drivetrain/transmission.cpp
  src/writers/cruise.cpp)
set_target_properties(bike_native_contract_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
get_target_property(_native_contract_compile_options bike_native COMPILE_OPTIONS)
get_target_property(_native_contract_compile_definitions bike_native COMPILE_DEFINITIONS)
get_target_property(_native_contract_include_directories bike_native INCLUDE_DIRECTORIES)
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
target_link_libraries(bike_native_engine PUBLIC "${MJ_DIR}/libmujoco.3.12.0.dylib")
target_link_libraries(bike_native PRIVATE bike_native_engine)

add_executable(native_contract_tests tests/test_contracts.cpp)
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
# chain.cpp's GeometryWorkspace resolves CBLAS through Apple Accelerate, the
# same BLAS provider bike_native links.
target_link_libraries(native_contract_tests PRIVATE "-framework Accelerate")
add_custom_command(TARGET native_contract_tests POST_BUILD
  COMMAND install_name_tool -change
    "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
    "${MJ_DIR}/libmujoco.3.12.0.dylib"
    "$<TARGET_FILE:native_contract_tests>"
  VERBATIM)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME native_contract_tests COMMAND native_contract_tests)
endif()

add_custom_target(check_native_contracts
  COMMAND uv run --frozen --group native python "${CMAKE_SOURCE_DIR}/../tools/check_native_contracts.py"
          --build "${CMAKE_BINARY_DIR}"
  COMMENT "native structural contract checks"
  VERBATIM)

target_link_libraries(bike_native PRIVATE "${MJ_DIR}/libmujoco.3.12.0.dylib")

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
add_custom_command(TARGET bike_native POST_BUILD
  COMMAND install_name_tool -change
    "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
    "${MJ_DIR}/libmujoco.3.12.0.dylib"
    "$<TARGET_FILE:bike_native>"
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
    cmake_path(IS_PREFIX CMAKE_CURRENT_SOURCE_DIR "${_source_absolute}" NORMALIZE _source_is_first_party)
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

_native_json_array(_native_source_roots_json "${CMAKE_CURRENT_SOURCE_DIR}")
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
