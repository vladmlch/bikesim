# CompilerPolicy.cmake — shared toolchain policy for every first-party
# target in the bike-sim C++ migration (T03).
#
# Contents:
#   * bike_resolve_macos_sdk()        — pre-project() macOS SDK resolution
#                                       (call only behind APPLE, before
#                                       project()). Retains the explicit-SDK
#                                       failure checks of the legacy native
#                                       build; never picks a different SDK
#                                       or compiler silently.
#   * bike_verify_sdk_compat()        — post-project() compile probe of the
#                                       selected compiler + sysroot pair.
#   * bike_detect_libcpp_hardening()  — post-project() libc++ detection and
#                                       hardening-mode acceptance probe.
#   * BIKE_STRICT_WARNING_FLAGS       — the single strict-warning/numerics
#                                       list shared by the legacy binding
#                                       build and the standalone root build.
#   * bike_target_strict_policy()     — applies strict flags, stack
#                                       protection, hardening, no-fast-math
#                                       and disabled FP contraction to one
#                                       first-party target, plus the
#                                       optional sanitizer/RTSan/coverage
#                                       instrumentation. Third-party
#                                       warning policy stays isolated
#                                       (SYSTEM includes, per-target use).
#
# This module executes no code at include() time; callers invoke the macros/
# functions explicitly so pre-project() order stays under the CMakeLists's
# control.

# ---------------------------------------------------------------------------
# Strict first-party warning and numerics flags.
#
# -ffp-contract=off: clang's default (on) fuses mul+add into FMA on arm64 —
# one rounding instead of two, breaking bitwise parity with the Python
# oracle, whose interpreter rounds every operation separately.
#
# Deliberately NOT enabled (verified on Apple clang 21):
#   noise tier: -Wfloat-equal -Wpadded -Wexit-time-destructors
#     -Wglobal-constructors -Wzero-as-null-pointer-constant -Wdocumentation
#   unsupported here: -Wcast-align=strict -Wenum-constexpr-conversion
#     -Wfixed-enum-extension -fsanitize=safe-stack/memory/dataflow
#   dead on target: -fstack-clash-protection (accepted, unused),
#     -fsanitize=thread (compiles, runtime aborts on arm64-darwin)
set(BIKE_STRICT_WARNING_FLAGS
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
  -ffp-contract=off
  -fstack-protector-strong -ftrivial-auto-var-init=zero -fvisibility=hidden
  -fvisibility-inlines-hidden -fstrict-flex-arrays=3
  -mbranch-protection=standard)

# ---------------------------------------------------------------------------
# Pre-project() macOS SDK resolution. CMake performs its compiler link check
# during project(), so a default SDK must be resolved before that call.
# Never replace an SDK or compiler supplied by the caller.
#
# Must be invoked as a macro (variables land in the caller's directory scope)
# behind `if(APPLE)` and before project(). Produces:
#   _native_sdk_path       — resolved REALPATH of the selected SDK
#   _native_sdk_source     — "default" | "explicit"
#   _native_sdk_requested  — the SDK the caller asked for (or resolved path)
macro(bike_resolve_macos_sdk)
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
  set(_native_sdk_probe_source "${CMAKE_CURRENT_BINARY_DIR}/native_check_work/sdk-compat.cpp")
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
    set(_native_sdk_probe_binary "${CMAKE_CURRENT_BINARY_DIR}/native_check_work/sdk-compat-probe-${_native_sdk_probe_index}")
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
endmacro()

# ---------------------------------------------------------------------------
# Post-project() compiler/SDK compatibility probe. The compiler identification
# check is not enough: compile a source that includes an SDK header with the
# selected compiler and sysroot. Call behind `if(APPLE)` after project().
function(bike_verify_sdk_compat)
  set(_native_sdk_probe_dir "${CMAKE_CURRENT_BINARY_DIR}/native_check_work/sdk-compat")
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
endfunction()

# ---------------------------------------------------------------------------
# libc++ detection + hardening-mode acceptance probe. Call after project().
# Reads BIKE_LIBCPP_HARDENING (already validated by the caller) and produces,
# in the caller's scope:
#   BIKE_LIBCPP_HARDENING_SUPPORTED — TRUE when the standard library is libc++
#                                     and accepted the selected mode
#   BIKE_LIBCPP_HARDENING_EFFECTIVE — the applied mode name, or NOT_APPLICABLE
# On non-APPLE hosts the probe still runs (libc++ exists elsewhere) but the
# sysroot flag is omitted.
function(bike_detect_libcpp_hardening)
  set(_native_hardening_probe_dir "${CMAKE_CURRENT_BINARY_DIR}/native_check_work/libcpp-hardening")
  file(MAKE_DIRECTORY "${_native_hardening_probe_dir}")
  set(_native_libcpp_probe_source "${_native_hardening_probe_dir}/detect_libcpp.cpp")
  file(WRITE "${_native_libcpp_probe_source}"
    "#include <__config>\n"
    "#ifndef _LIBCPP_VERSION\n#error The configured standard library is not libc++\n#endif\n"
    "int main() { return 0; }\n")
  set(_native_hardening_cmake_flags
    "-DCMAKE_CXX_STANDARD:STRING=23"
    "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF")
  if(APPLE)
    list(APPEND _native_hardening_cmake_flags
      "-DCMAKE_OSX_SYSROOT:PATH=${CMAKE_OSX_SYSROOT}")
  endif()
  unset(_native_uses_libcpp CACHE)
  try_compile(_native_uses_libcpp
    "${_native_hardening_probe_dir}/detect-build"
    SOURCES "${_native_libcpp_probe_source}"
    CMAKE_FLAGS ${_native_hardening_cmake_flags}
    OUTPUT_VARIABLE _native_libcpp_detection_output)

  set(BIKE_LIBCPP_HARDENING_SUPPORTED FALSE PARENT_SCOPE)
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
      CMAKE_FLAGS ${_native_hardening_cmake_flags}
      COMPILE_DEFINITIONS
        "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}"
      OUTPUT_VARIABLE _native_libcpp_hardening_output)
    if(NOT _native_libcpp_mode_probe_result)
      message(FATAL_ERROR
        "Configured libc++ does not support hardening mode ${BIKE_LIBCPP_HARDENING}:\n${_native_libcpp_hardening_output}")
    endif()
    set(BIKE_LIBCPP_HARDENING_SUPPORTED TRUE PARENT_SCOPE)
    set(BIKE_LIBCPP_HARDENING_EFFECTIVE "${BIKE_LIBCPP_HARDENING}" PARENT_SCOPE)
  else()
    message(STATUS "libc++ hardening is not applied: configured standard library is not libc++")
    set(BIKE_LIBCPP_HARDENING_EFFECTIVE "NOT_APPLICABLE" PARENT_SCOPE)
  endif()
endfunction()

# ---------------------------------------------------------------------------
# Apply the shared first-party target policy to one target:
#   * BIKE_STRICT_WARNING_FLAGS (warnings-as-errors, stack protection,
#     trivial-var-init, visibility, strict-flex-arrays, branch protection,
#     disabled FP contraction — the no-fast-math contract)
#   * -O2 under Release: the previous Release compile line ended in -Os
#     (CMAKE's -O3, then nanobind's -Os, last wins); -O2 measures ~7% faster
#     than that stack on this runtime's branchy code (verified arm64-apple,
#     ~0.181 vs ~0.194 ms/step on the same session). Appending -O2 here
#     overrides inherited -O flags since the last one wins; this does not
#     stomp a caller's -O choice for other build types.
#   * libc++ hardening define when the probe accepted the requested mode.
#     Hardening modes do not change container layout; the selected mode stays
#     consistent across the target's TUs.
#   * _FORTIFY_SOURCE=3 — skipped under NATIVE_SANITIZE because the ASan
#     driver predefines _FORTIFY_SOURCE (and intercepts the same functions
#     anyway); redefining it would trip -Wmacro-redefined.
#   * NATIVE_SANITIZE (ASan+UBSan):
#       local-bounds: checked access through bounds of stack/heap objects.
#       float-cast-overflow: NOT part of the undefined group — out-of-range
#       float→int conversions at the FFI boundary (tooth counts etc.).
#       no-recover: UBSan findings abort instead of logging-and-continuing.
#       Available-but-rejected extras (verified arm64-apple): implicit-
#       conversion and unsigned-integer-overflow are too noisy for numerics;
#       thread/msan/cfi/safe-stack are unsupported or dead on this target.
#       The 8192-byte -Wframe-larger-than budget guards production stack
#       use; ASan wraps every alloca in >=32-byte redzones, roughly tripling
#       the measured frame, so the instrumented budget is raised 3x —
#       appended last (clang repeated-flag last-wins semantics verified on
#       Apple clang 21).
#   * NATIVE_RTSAN (-fsanitize=realtime): flags allocation/locking inside
#     [[clang::nonblocking]] functions (BIKE_NONBLOCKING in rtsan.hpp);
#     requires the brew clang — Apple clang 21 lacks the sanitizer.
#   * NATIVE_COVERAGE: llvm-cov instrumentation (-fprofile-instr-generate
#     -fcoverage-mapping) for tools/native_coverage.sh.
# Third-party warning policy stays isolated: this function never relaxes the
# flag set and callers keep third-party headers on SYSTEM include paths.
function(bike_target_strict_policy target)
  target_compile_options(${target} PRIVATE ${BIKE_STRICT_WARNING_FLAGS})
  target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:-O2>)
  if(BIKE_LIBCPP_HARDENING_SUPPORTED)
    target_compile_definitions(${target} PRIVATE
      "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}")
  endif()
  if(NOT NATIVE_SANITIZE)
    target_compile_definitions(${target} PRIVATE _FORTIFY_SOURCE=3)
  endif()
  if(NATIVE_SANITIZE)
    target_compile_options(${target} PRIVATE
      -fsanitize=address,undefined,local-bounds,float-cast-overflow
      -fno-sanitize-recover=all -fno-omit-frame-pointer
      -Wframe-larger-than=24576)
    target_link_options(${target} PRIVATE
      -fsanitize=address,undefined,local-bounds,float-cast-overflow -shared-libasan)
  endif()
  if(NATIVE_RTSAN)
    target_compile_options(${target} PRIVATE -fsanitize=realtime)
    target_link_options(${target} PRIVATE -fsanitize=realtime)
  endif()
  if(NATIVE_COVERAGE)
    target_compile_options(${target} PRIVATE
      -fprofile-instr-generate -fcoverage-mapping)
    target_link_options(${target} PRIVATE -fprofile-instr-generate)
  endif()
endfunction()
