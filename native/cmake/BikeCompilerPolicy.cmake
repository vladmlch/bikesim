# Native compiler/build policy — platform-aware flag selection shared by
# every first-party target.
#
# Preconditions: include after project() (a CXX compiler must be selected).
# In the real build BikePlatform.cmake has already fixed
# BIKE_TARGET_SYSTEM; a standalone fixture project gets the host name.
#
# Exports consumed by native/CMakeLists.txt (and the portability fixture):
#   bike_require_cxx_flag(flag var)     required flag probe — FATAL on
#                                       an unsupported required flag
#   bike_apply_native_policy(target)    applies the DIRECT (not
#                                       transitive-only) compile options,
#                                       compile definitions and link
#                                       options of the selected policy to
#                                       one first-party target
#   BIKE_POLICY_COMPILE_OPTIONS / BIKE_POLICY_COMPILE_DEFINITIONS /
#   BIKE_POLICY_LINK_OPTIONS            the resolved lists (recorded into
#                                       bike_policy_report.json and the
#                                       compiler_policy field of
#                                       native_check_context.json)
#   _native_uses_libcpp / _native_libcpp_hardening_supported /
#   _native_libcpp_hardening_mode       kept names for the schema-1
#                                       context emission
#   _native_json_quote / _native_json_array   JSON emit helpers shared by
#                                       the context writer below
#
# Build options owned here (declared once so targets created while a
# sibling cmake module is being included — e.g. bike_native_numerics —
# already see them):
#   NATIVE_SANITIZE / NATIVE_RTSAN / NATIVE_TSAN / NATIVE_COVERAGE
#   BIKE_LIBCPP_HARDENING=EXTENSIVE|FAST
#
# Test-only knobs (production builds leave them empty): the overrides
# drive the *dispatch* decisions of the policy only — compiler identity,
# the standard library and every flag probe still run against the real
# toolchain, so a simulated "Linux x86_64" never fabricates support.
#   BIKE_POLICY_OVERRIDE_SYSTEM / BIKE_POLICY_OVERRIDE_MACHINE
#   BIKE_POLICY_TEST_EXTRA_WARNINGS    extra optional warning candidates
#   BIKE_POLICY_TEST_EXTRA_REQUIRED    extra required flags (the fail
#                                      closed path is covered by passing
#                                      an unsupportable flag here)

if(NOT CMAKE_CXX_COMPILER_LOADED)
  message(FATAL_ERROR
    "BikeCompilerPolicy.cmake requires an enabled CXX compiler; "
    "include it after project()")
endif()
if(NOT DEFINED BIKE_TARGET_SYSTEM OR BIKE_TARGET_SYSTEM STREQUAL "")
  set(BIKE_TARGET_SYSTEM "${CMAKE_HOST_SYSTEM_NAME}")
endif()

option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
option(NATIVE_RTSAN "RealtimeSanitizer build (brew clang only)" OFF)
option(NATIVE_TSAN "ThreadSanitizer build (Linux only)" OFF)
option(NATIVE_COVERAGE "llvm-cov instrumented build" OFF)

# ASan and RTSan are mutually exclusive lanes: their runtimes compete for
# the process preload slot, and a realtime-allocation finding is meaningless
# inside an ASan-instrumented image. The runner-side environment helper
# (tools/native_runtime.py) refuses the same combination; rejecting it at
# configure time keeps a mixed-request tree from building at all.
if(NATIVE_TSAN AND BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  message(FATAL_ERROR "TSan runtime unsupported on arm64-Darwin")
endif()
if(NATIVE_TSAN AND NOT BIKE_TARGET_SYSTEM STREQUAL "Linux")
  message(FATAL_ERROR "NATIVE_TSAN requires a Linux target")
endif()
if(NATIVE_TSAN AND (NATIVE_SANITIZE OR NATIVE_RTSAN OR NATIVE_COVERAGE))
  message(FATAL_ERROR "NATIVE_TSAN requires a separate build tree from ASan, RTSan and coverage")
endif()
if(NATIVE_SANITIZE AND NATIVE_RTSAN)
  message(FATAL_ERROR
    "NATIVE_SANITIZE and NATIVE_RTSAN are mutually exclusive; configure "
    "separate build trees")
endif()

set(BIKE_LIBCPP_HARDENING "EXTENSIVE" CACHE STRING "libc++ hardening mode")
set_property(CACHE BIKE_LIBCPP_HARDENING PROPERTY STRINGS EXTENSIVE FAST)
if(NOT BIKE_LIBCPP_HARDENING STREQUAL "EXTENSIVE"
   AND NOT BIKE_LIBCPP_HARDENING STREQUAL "FAST")
  message(FATAL_ERROR
    "BIKE_LIBCPP_HARDENING must be EXTENSIVE or FAST, "
    "got '${BIKE_LIBCPP_HARDENING}'")
endif()

set(BIKE_POLICY_OVERRIDE_SYSTEM "" CACHE STRING
  "test-only: evaluate the policy for this target system instead of the host")
set(BIKE_POLICY_OVERRIDE_MACHINE "" CACHE STRING
  "test-only: evaluate the policy for this target machine instead of the host")
set(BIKE_POLICY_TEST_EXTRA_WARNINGS "" CACHE STRING
  "test-only: extra optional warning candidates to probe")
set(BIKE_POLICY_TEST_EXTRA_REQUIRED "" CACHE STRING
  "test-only: extra required flags to enforce fail-closed")

include(CheckCXXCompilerFlag)
include(CheckCXXSourceCompiles)
include(CheckLinkerFlag)
include(CMakePushCheckState)

# JSON emit helpers (_native_json_quote, _native_json_array) are shared with
# the identity module, so they live in their own include-guarded file.
include("${CMAKE_CURRENT_LIST_DIR}/BikeJson.cmake")

# --------------------------------------------------------------------------
# Dispatch identity — real compiler/stdlib/arch probes; only the OS/arch
# *predicates* honor the test overrides, so support is never fabricated.
# --------------------------------------------------------------------------

if(BIKE_POLICY_OVERRIDE_SYSTEM)
  set(BIKE_POLICY_SYSTEM "${BIKE_POLICY_OVERRIDE_SYSTEM}")
else()
  set(BIKE_POLICY_SYSTEM "${BIKE_TARGET_SYSTEM}")
endif()
if(BIKE_POLICY_OVERRIDE_MACHINE)
  set(_bike_policy_machine_raw "${BIKE_POLICY_OVERRIDE_MACHINE}")
else()
  set(_bike_policy_machine_raw "${CMAKE_SYSTEM_PROCESSOR}")
endif()
string(TOLOWER "${_bike_policy_machine_raw}" BIKE_POLICY_MACHINE)
if(BIKE_POLICY_MACHINE STREQUAL "arm64")
  set(BIKE_POLICY_MACHINE "aarch64")
elseif(BIKE_POLICY_MACHINE MATCHES "^(amd64|x64)$")
  set(BIKE_POLICY_MACHINE "x86_64")
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(BIKE_COMPILER_FAMILY "appleclang")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(BIKE_COMPILER_FAMILY "clang")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(BIKE_COMPILER_FAMILY "gcc")
else()
  message(FATAL_ERROR
    "unsupported native C++ compiler family: ${CMAKE_CXX_COMPILER_ID}")
endif()
# Both Clang dialects share the warning-name space; GCC gets its own
# positive-spelling list because it silently tolerates unknown -Wno-* (and
# errors on unknown positive -W, so plain probing stays fail-closed).
if(BIKE_COMPILER_FAMILY STREQUAL "gcc")
  set(_bike_clang_family FALSE)
else()
  set(_bike_clang_family TRUE)
endif()

set(_bike_policy_probe_dir "${CMAKE_BINARY_DIR}/native_check_work/compiler-policy")
file(MAKE_DIRECTORY "${_bike_policy_probe_dir}")
set(_bike_policy_sysroot_flags "")
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  set(_bike_policy_sysroot_flags
    "-DCMAKE_OSX_SYSROOT:PATH=${CMAKE_OSX_SYSROOT}")
endif()

function(_bike_policy_flag_var flag out_var)
  string(MAKE_C_IDENTIFIER "BIKE_HAS_${flag}" _name)
  set(${out_var} "${_name}" PARENT_SCOPE)
endfunction()

# Required policy flag: unsupported => configuration fails, never skipped.
function(bike_require_cxx_flag flag result_var)
  check_cxx_compiler_flag("${flag}" ${result_var})
  if(NOT ${result_var})
    message(FATAL_ERROR
      "Required native compiler flag is unsupported: ${flag}")
  endif()
endfunction()

# Optional warning candidate: Clang warning names are probed under
# -Werror=unknown-warning-option (otherwise an unknown warning is only a
# warning and the probe would pass vacuously); GCC errors on unknown
# positive -W spellings natively.
function(_bike_probe_warning flag out_var)
  _bike_policy_flag_var("${flag}" _var)
  cmake_push_check_state(RESET)
  if(_bike_clang_family)
    set(CMAKE_REQUIRED_FLAGS
      "${CMAKE_REQUIRED_FLAGS} -Werror=unknown-warning-option")
  endif()
  check_cxx_compiler_flag("${flag}" ${_var})
  cmake_pop_check_state()
  set(${out_var} "${${_var}}" PARENT_SCOPE)
endfunction()

set(BIKE_POLICY_OMITTED_WARNINGS "")
set(BIKE_POLICY_OMITTED_REASONS "")
function(_bike_record_omission flag reason)
  list(APPEND BIKE_POLICY_OMITTED_WARNINGS "${flag}")
  list(APPEND BIKE_POLICY_OMITTED_REASONS "${reason}")
  set(BIKE_POLICY_OMITTED_WARNINGS "${BIKE_POLICY_OMITTED_WARNINGS}" PARENT_SCOPE)
  set(BIKE_POLICY_OMITTED_REASONS "${BIKE_POLICY_OMITTED_REASONS}" PARENT_SCOPE)
endfunction()

# --------------------------------------------------------------------------
# C++23 standard-library feature check — the writers and the benchmark use
# std::format, std::span and std::bit_cast; CXX_STANDARD alone does not
# prove the library provides them.
# --------------------------------------------------------------------------

cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_cxx_source_compiles([=[
#include <bit>
#include <cstdint>
#include <format>
#include <span>
#include <string>
int main() {
  const double data[] = {1.5, 2.5};
  const std::span<const double> view(data);
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(view[0]);
  const std::string text = std::format("{} {:x}", view.size(), bits);
  return text.empty() ? 1 : 0;
}
]=] BIKE_CXX23_LIBRARY_USABLE)
cmake_pop_check_state()
if(NOT BIKE_CXX23_LIBRARY_USABLE)
  message(FATAL_ERROR
    "The selected C++23 standard library cannot compile the required "
    "std::format/std::span/std::bit_cast subset")
endif()

# --------------------------------------------------------------------------
# Standard-library identity and hardening.
#   libc++:      _LIBCPP_VERSION via a normal header; EXTENSIVE/FAST mode
#                probed by compiling with the selected mode define.
#   libstdc++:   __GLIBCXX__ via a normal header; the checked mode is
#                _GLIBCXX_ASSERTIONS — the non-ABI-changing assertions
#                level. _GLIBCXX_DEBUG changes container layout and is
#                never emitted into the production module ABI.
# Anything else: fail closed.
# --------------------------------------------------------------------------

cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_cxx_source_compiles([=[
#include <version>
#if !defined(_LIBCPP_VERSION)
#error The configured standard library is not libc++
#endif
int main() { return 0; }
]=] BIKE_STDLIB_IS_LIBCPP)
check_cxx_source_compiles([=[
#include <vector>
#if !defined(__GLIBCXX__)
#error The configured standard library is not libstdc++
#endif
int main() { return 0; }
]=] BIKE_STDLIB_IS_LIBSTDCXX)
# glibc drives the _FORTIFY_SOURCE decision: fortified shims exist only
# when compiling with optimization, so on glibc the define is applied to
# the optimized configurations only (recorded below).
check_cxx_source_compiles([=[
#include <cstdio>
#if !defined(__GLIBC__)
#error The configured C library is not glibc
#endif
int main() { return 0; }
]=] BIKE_TARGET_GLIBC)
cmake_pop_check_state()

set(BIKE_STDLIB_IDENTITY "")
if(BIKE_STDLIB_IS_LIBCPP)
  set(BIKE_STDLIB_IDENTITY "libc++")
  set(_native_uses_libcpp TRUE)
  set(_native_libcpp_mode_probe_source
    "${_bike_policy_probe_dir}/check_hardening_mode.cpp")
  file(WRITE "${_native_libcpp_mode_probe_source}"
    "#include <vector>\n"
    "#ifndef _LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}\n"
    "#error libc++ omitted the selected hardening mode macro\n#endif\n"
    "#ifndef _LIBCPP_HARDENING_MODE\n#error libc++ omitted the hardening mode selection\n#endif\n"
    "#if _LIBCPP_HARDENING_MODE != _LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}\n"
    "#error libc++ did not accept the selected hardening mode\n#endif\n"
    "int main() { return 0; }\n")
  unset(_native_libcpp_mode_probe_result)
  unset(_native_libcpp_mode_probe_result CACHE)
  try_compile(_native_libcpp_mode_probe_result
    "${_bike_policy_probe_dir}/mode-build"
    SOURCES "${_native_libcpp_mode_probe_source}"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags}
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
  set(_bike_stdlib_hardening_detail
    "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}")
elseif(BIKE_STDLIB_IS_LIBSTDCXX)
  set(BIKE_STDLIB_IDENTITY "libstdc++")
  set(_native_uses_libcpp FALSE)
  set(_native_libcpp_hardening_supported FALSE)
  set(_native_libcpp_hardening_mode "NOT_APPLICABLE")
  set(_bike_stdlib_hardening_detail "_GLIBCXX_ASSERTIONS")
else()
  message(FATAL_ERROR
    "The selected C++ standard library is neither libc++ nor libstdc++; "
    "no hardening policy exists for it")
endif()

# Standard-library version — the identity probe above establishes WHICH
# library is linked; the schema-2 context also records its version. Measured
# by compiling and running a probe under the same sysroot contract: libc++
# reports _LIBCPP_VERSION, libstdc++ reports its release/date macros. A
# failed probe while an identity was already proven is a contradiction —
# fail closed rather than emit a fabricated version.
file(WRITE "${_bike_policy_probe_dir}/stdlib_version.cpp"
  "#include <version>\n#include <cstdio>\n"
  "int main() {\n"
  "#if defined(_LIBCPP_VERSION)\n"
  "  std::printf(\"%d\", static_cast<int>(_LIBCPP_VERSION));\n"
  "#elif defined(_GLIBCXX_RELEASE)\n"
  "  std::printf(\"%d.%ld\", static_cast<int>(_GLIBCXX_RELEASE),\n"
  "             static_cast<long>(__GLIBCXX__));\n"
  "#elif defined(__GLIBCXX__)\n"
  "  std::printf(\"%ld\", static_cast<long>(__GLIBCXX__));\n"
  "#else\n"
  "  return 1;\n"
  "#endif\n"
  "  return 0;\n"
  "}\n")
try_run(_bike_stdlib_version_run _bike_stdlib_version_compile
  "${_bike_policy_probe_dir}/stdlib-version"
  SOURCES "${_bike_policy_probe_dir}/stdlib_version.cpp"
  CMAKE_FLAGS
    ${_bike_policy_sysroot_flags}
    "-DCMAKE_CXX_STANDARD:STRING=23"
    "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
  RUN_OUTPUT_VARIABLE _bike_stdlib_version_output
  COMPILE_OUTPUT_VARIABLE _bike_stdlib_version_compile_log)
if(NOT _bike_stdlib_version_compile
   OR NOT _bike_stdlib_version_run EQUAL 0
   OR _bike_stdlib_version_output STREQUAL "")
  message(FATAL_ERROR
    "the ${BIKE_STDLIB_IDENTITY} identity probe succeeded but the version "
    "probe failed — the toolchain is inconsistent:\n"
    "${_bike_stdlib_version_compile_log}")
endif()
set(BIKE_STDLIB_VERSION "${_bike_stdlib_version_output}")

# --------------------------------------------------------------------------
# Warning candidates — the Clang list preserves the previous exact set and
# ordering; GCC gets the equivalent positive spellings (the same hygiene
# set the g++ frontends sweep enforces). Required flags stay inside the
# canonical order and are probed fail-closed.
# --------------------------------------------------------------------------

# Deliberately NOT enabled (verified on Apple clang 21):
#   noise tier: -Wfloat-equal -Wpadded -Wexit-time-destructors
#     -Wglobal-constructors -Wzero-as-null-pointer-constant -Wdocumentation
#   unsupported here: -Wcast-align=strict -Wenum-constexpr-conversion
#     -Wfixed-enum-extension -fsanitize=safe-stack/memory/dataflow
#   dead on target: -fstack-clash-protection (accepted, unused),
#     -fsanitize=thread (compiles, runtime aborts on arm64-darwin)
set(_bike_clang_warning_candidates
  -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
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
  -Wframe-larger-than=8192)

# GCC equivalents: every warning has a GCC-native positive spelling; the
# Clang-only names are never offered to GCC. This list mirrors the
# frontends sweep's _GCC_WARNING_FLAGS (tools/native_checks.py) minus
# -fanalyzer (analysis pass, not a build flag) plus the shared base set.
set(_bike_gcc_warning_candidates
  # -Wno-psabi: aarch64 GCC emits informational ABI-change notes for
  # std::pair<...> parameter passing on every binding TU; the ODR sweep
  # already silences the same notes (tools/native_checks.py's odr argv).
  -Wno-psabi
  -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
  -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wnull-dereference
  -Wlogical-op -Wduplicated-cond -Wduplicated-branches -Wrestrict
  -Wuseless-cast -Warith-conversion
  -Wstringop-overflow=4 -Wformat-signedness -Wformat-overflow=2
  -Wformat-truncation=2 -Wextra-semi -Wredundant-tags
  -Wcast-function-type -Wcast-align=strict
  -Wcatch-value=3 -Wconditionally-supported -Wdeprecated-copy-dtor
  -Wvolatile -Winit-self -Wsign-promo -Wctor-dtor-privacy
  -Wplacement-new=2 -Wmismatched-new-delete -Wsized-deallocation
  -Winterference-size -Wsubobject-linkage -Wsuggest-override
  -Wstrict-null-sentinel
  -Wmissing-declarations -Wredundant-decls
  -Wswitch-enum -Wmultichar -Wdate-time
  -Walloca -Wvla -Warray-bounds=2 -Waggressive-loop-optimizations
  # GCC frame accounting runs ~1.4x clang's on the same binding-heavy TUs
  # (Ubuntu 13.3 aarch64 measures 11.5 KB where clang stays under 8 KB), so
  # the GCC budget is calibrated to 16384 — still a hard wall against
  # pathological frames, without clang-calibrated false positives. The clang
  # lane keeps the 8192-byte contract; the sanitizer lane's 24576 bump stacks
  # on top of whichever budget the family selected.
  # NOTE: tools/native_checks.py's _GCC_WARNING_FLAGS still names =8192 for
  # the GCC frontends sweep — that -fsyntax-only pass never measures codegen
  # frames, so the two budgets do not collide today; reconciling the sweep's
  # spelling with this calibrated budget is deferred to T14/T10, which wire
  # the sweep onto the Linux lane.
  -Wstack-usage=16384 -Wframe-larger-than=16384 -Wstack-protector)

# Required flags — safety/numerics contract, fail closed when unsupported.
# -ffp-contract=off: clang's default (on) fuses mul+add into FMA on arm64 —
# one rounding instead of two, breaking bitwise parity with the Python
# oracle, whose interpreter rounds every operation separately.
# -fstack-protector-strong / -ftrivial-auto-var-init=zero /
# -fstrict-flex-arrays=3 / -fvisibility*: the required safety policy; a
# target that cannot provide them fails configuration rather than
# silently degrading.
set(_bike_required_flags
  -Werror
  -ffp-contract=off
  -fstack-protector-strong
  -ftrivial-auto-var-init=zero
  -fvisibility=hidden
  -fvisibility-inlines-hidden
  -fstrict-flex-arrays=3)
if(BIKE_POLICY_TEST_EXTRA_REQUIRED)
  list(APPEND _bike_required_flags ${BIKE_POLICY_TEST_EXTRA_REQUIRED})
endif()

if(_bike_clang_family)
  set(_bike_warning_candidates ${_bike_clang_warning_candidates})
else()
  set(_bike_warning_candidates ${_bike_gcc_warning_candidates})
endif()
if(BIKE_POLICY_TEST_EXTRA_WARNINGS)
  list(APPEND _bike_warning_candidates ${BIKE_POLICY_TEST_EXTRA_WARNINGS})
endif()

# Batch-probe all warning candidates in one compile — a trivial TU cannot
# trip a real warning, so a batch rejection means an unsupported name.
# Per-flag probing then names exactly which candidates were omitted.
cmake_push_check_state(RESET)
if(_bike_clang_family)
  set(CMAKE_REQUIRED_FLAGS "-Werror=unknown-warning-option")
endif()
list(JOIN _bike_warning_candidates " " _bike_warnings_joined)
string(APPEND CMAKE_REQUIRED_FLAGS " ${_bike_warnings_joined}")
check_cxx_source_compiles("int main() { return 0; }"
  BIKE_WARNING_BATCH_SUPPORTED)
cmake_pop_check_state()

set(_bike_applied_warnings "")
if(BIKE_WARNING_BATCH_SUPPORTED)
  set(_bike_applied_warnings ${_bike_warning_candidates})
else()
  foreach(_candidate IN LISTS _bike_warning_candidates)
    _bike_probe_warning("${_candidate}" _candidate_supported)
    if(_candidate_supported)
      list(APPEND _bike_applied_warnings "${_candidate}")
    else()
      _bike_record_omission("${_candidate}"
        "compiler rejected the warning name during probing")
    endif()
  endforeach()
endif()

# -Wunsafe-buffer-usage needs more than name support: LLVM <19 flags
# provably-bounded constant-index member-array access (the
# mjData::timer[mjTIMER_*] idiom in engine_call.cpp). Probe the exact
# pattern under -Werror; omit the warning with a recorded reason when the
# analyzer still fires on it.
if(_bike_clang_family
   AND "-Wunsafe-buffer-usage" IN_LIST _bike_applied_warnings)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS
    "-Werror -Wunsafe-buffer-usage -Werror=unknown-warning-option")
  check_cxx_source_compiles([=[
struct BikeProbeBuffer { int slots[4]; };
int bike_probe_read(const BikeProbeBuffer *buffer) { return buffer->slots[2]; }
int main() { BikeProbeBuffer buffer{}; return bike_probe_read(&buffer); }
]=] BIKE_UNSAFE_BUFFER_USAGE_USABLE)
  cmake_pop_check_state()
  if(NOT BIKE_UNSAFE_BUFFER_USAGE_USABLE)
    list(REMOVE_ITEM _bike_applied_warnings "-Wunsafe-buffer-usage")
    _bike_record_omission("-Wunsafe-buffer-usage"
      "supported but rejects provably-bounded constant-index member-array access on this clang; enabling it would fail first-party TUs (LLVM <19 analysis)")
  endif()
endif()

# -Wuseless-cast needs more than name support on GCC too: the flag is
# type-identity sensitive, and first-party TUs cast std::int64_t
# differences to std::vector<>::difference_type — an identity conversion
# on LP64 Linux (int64_t == long == difference_type) but a real one on
# Darwin LP64 (int64_t == long long != long). Probe the exact idiom under
# -Werror; omit with a recorded reason where the analyzer rejects it.
if(BIKE_COMPILER_FAMILY STREQUAL "gcc"
   AND "-Wuseless-cast" IN_LIST _bike_applied_warnings)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS "-Werror -Wuseless-cast")
  check_cxx_source_compiles([=[
#include <cstdint>
#include <vector>
std::vector<int>::difference_type bike_probe_delta(std::int64_t cursor,
                                                   std::int64_t base) {
  return static_cast<std::vector<int>::difference_type>(cursor - base);
}
int main() { return static_cast<int>(bike_probe_delta(3, 1)); }
]=] BIKE_USELESS_CAST_USABLE)
  cmake_pop_check_state()
  if(NOT BIKE_USELESS_CAST_USABLE)
    list(REMOVE_ITEM _bike_applied_warnings "-Wuseless-cast")
    _bike_record_omission("-Wuseless-cast"
      "supported but rejects the type-identity-dependent casts first-party TUs rely on for LP64 targets (int64_t == vector::difference_type here); the frontends sweep still enforces it on toolchains where the types differ")
  endif()
endif()

# --------------------------------------------------------------------------
# Architecture/OS-gated flags — each is a dispatch predicate AND a real
# compile/link probe, so a test override can never fabricate support.
# --------------------------------------------------------------------------

set(_bike_arch_flags "")
set(_bike_branch_protection_applied FALSE)
set(_bike_branch_protection_reason "")
if(BIKE_POLICY_MACHINE STREQUAL "aarch64")
  _bike_policy_flag_var("-mbranch-protection=standard" _bike_bp_var)
  file(WRITE "${_bike_policy_probe_dir}/branch_protection.cpp"
    "int main() { return 0; }\n")
  try_compile(${_bike_bp_var}
    "${_bike_policy_probe_dir}/branch-protection"
    SOURCES "${_bike_policy_probe_dir}/branch_protection.cpp"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags}
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS "-mbranch-protection=standard"
    LINK_OPTIONS "-mbranch-protection=standard"
    OUTPUT_VARIABLE _bike_bp_output)
  if(${_bike_bp_var})
    list(APPEND _bike_arch_flags "-mbranch-protection=standard")
    set(_bike_branch_protection_applied TRUE)
    set(_bike_branch_protection_reason
      "aarch64 target; compile/link probe succeeded")
  else()
    set(_bike_branch_protection_reason
      "aarch64 target but the compile/link probe failed")
    _bike_record_omission("-mbranch-protection=standard"
      "aarch64 target but the compile/link probe failed")
  endif()
else()
  set(_bike_branch_protection_reason
    "target machine '${BIKE_POLICY_MACHINE}' is not AArch64")
endif()

# --------------------------------------------------------------------------
# Instrumentation — the same sanitizer/coverage context that used to
# propagate via the mirrored compile-options list now lives here, so every
# policy target is compiled identically (T15-T17 own the final semantics).
# --------------------------------------------------------------------------

set(_bike_instrument_options "")
set(_bike_instrument_link_options "")
if(NATIVE_SANITIZE)
  # local-bounds: checked access through bounds of stack/heap objects.
  # float-cast-overflow: NOT part of the undefined group — out-of-range
  # float→int conversions at the FFI boundary (tooth counts etc.).
  # no-recover: UBSan findings abort instead of logging-and-continuing, so
  # a violation under pytest can never pass silently.
  # Available-but-rejected extras (verified arm64-apple):
  # implicit-conversion and unsigned-integer-overflow are too noisy for
  # numerics; thread/msan/cfi/safe-stack are unsupported or dead on this
  # target.
  # The 8192-byte -Wframe-larger-than budget guards production stack use;
  # ASan wraps every alloca in >=32-byte redzones, roughly tripling the
  # measured frame (DistributedTireWriter::accumulate_validated reports
  # 9152 bytes instrumented), so the instrumented budget is raised 3x —
  # appended last, clang's repeated-flag last-wins semantics verified on
  # Apple clang 21.
  if(_bike_clang_family)
    list(APPEND _bike_instrument_options
      -fsanitize=address,undefined,local-bounds,float-cast-overflow
      -fno-sanitize-recover=all -fno-omit-frame-pointer
      -Wframe-larger-than=24576)
    list(APPEND _bike_instrument_link_options
      -fsanitize=address,undefined,local-bounds,float-cast-overflow
      -shared-libasan)
  else()
    # GCC has no local-bounds sanitizer and no -shared-libasan spelling;
    # keep the GCC-compatible core and record the omissions. The frame
    # budget scales off GCC's calibrated 16384 baseline by the same 3x
    # ASan redzone factor clang's 8192 uses — g++-13/aarch64 measures
    # binding.cpp's module-init frame at ~29.6 KB instrumented, so both
    # frame spellings (-Wstack-usage and -Wframe-larger-than carry the
    # family budget) get the last-wins override, not just one.
    list(APPEND _bike_instrument_options
      -fsanitize=address,undefined
      -fno-sanitize-recover=all -fno-omit-frame-pointer
      -Wstack-usage=49152 -Wframe-larger-than=49152)
    list(APPEND _bike_instrument_link_options -fsanitize=address,undefined)
    _bike_record_omission("-fsanitize=local-bounds,float-cast-overflow"
      "no GCC equivalent; the ASan lane keeps the address,undefined core under GCC (T15-T17 finalize)")
    _bike_record_omission("-shared-libasan"
      "no GCC spelling on this target; the default shared libasan is linked")
  endif()
endif()
if(NATIVE_TSAN)
  list(APPEND _bike_instrument_options -fsanitize=thread -fno-omit-frame-pointer)
  list(APPEND _bike_instrument_link_options -fsanitize=thread)
endif()
if(NATIVE_RTSAN)
  if(_bike_clang_family)
    list(APPEND _bike_instrument_options -fsanitize=realtime)
    list(APPEND _bike_instrument_link_options -fsanitize=realtime)
  else()
    message(FATAL_ERROR
      "NATIVE_RTSAN requires a Clang-family compiler that accepts "
      "-fsanitize=realtime (the brew LLVM clang); the selected compiler "
      "${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_ID} "
      "${CMAKE_CXX_COMPILER_VERSION}) provides no realtime sanitizer lane")
  endif()
endif()
if(NATIVE_COVERAGE)
  if(_bike_clang_family)
    # T17: llvm-cov's source-based instrumentation format is clang-only —
    # there is no GNU gcov reporting lane, so a non-clang compiler in a
    # requested coverage build is a configure error, not a flag fallback.
    list(APPEND _bike_instrument_options
      -fprofile-instr-generate -fcoverage-mapping)
    list(APPEND _bike_instrument_link_options -fprofile-instr-generate)
  else()
    message(FATAL_ERROR
      "NATIVE_COVERAGE requires an LLVM clang-family compiler (llvm-cov "
      "reads clang's source-based instrumentation format; this project "
      "reports no gcov data); the selected compiler ${CMAKE_CXX_COMPILER} "
      "(${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}) cannot "
      "produce it")
  endif()
endif()

# --------------------------------------------------------------------------
# Capability evidence — compile+link probes for every instrumentation lane,
# recorded into the schema-2 context's `capabilities` record. Each probe runs
# the real flag set against the selected toolchain; the record describes
# measured truth only — a requested-but-unsupported capability still fails
# the build through the flag paths above, never through this record.
# --------------------------------------------------------------------------

set(_bike_capability_probe_dir "${_bike_policy_probe_dir}/capabilities")
file(WRITE "${_bike_capability_probe_dir}/probe.cpp"
  "int main() { return 0; }\n")
# The module's own location derives the native tree — a fixture project
# including this file may place CMAKE_CURRENT_SOURCE_DIR elsewhere.
get_filename_component(BIKE_NATIVE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
# The RTSan lane's capability probe is NOT the trivial TU: the flag, the
# __has_feature query and the BIKE_NONBLOCKING annotation must compile and
# link together, matching the production contract exactly. The macro sits
# in the trailing declarator position — since LLVM 20 [[clang::nonblocking]]
# is a function *type* attribute and the leading decl-specifier position is
# a hard error, while trailing applies on every supported clang.
file(WRITE "${_bike_capability_probe_dir}/rtsan_probe.cpp"
  "#include \"rtsan.hpp\"\n"
  "#if !defined(__has_feature)\n"
  "#error realtime sanitizer feature query is unavailable\n"
  "#elif !__has_feature(realtime_sanitizer)\n"
  "#error realtime sanitizer instrumentation is unavailable\n"
  "#endif\n"
  "void control() BIKE_NONBLOCKING {}\n"
  "int main() { control(); return 0; }\n")
# _bike_probe_capability(<result-var> <label>
#                        COMPILE_FLAGS <flags...> [LINK_FLAGS <flags...>]
#                        [SOURCE <file>] [INCLUDE_DIRS <dirs...>])
# Compile AND link the probe TU with the flag set the real lane uses; the
# link half proves the runtime image exists, not just the driver spelling.
function(_bike_probe_capability result_var label)
  cmake_parse_arguments(_cap "" "SOURCE"
    "COMPILE_FLAGS;LINK_FLAGS;INCLUDE_DIRS" ${ARGN})
  if(NOT DEFINED _cap_LINK_FLAGS OR _cap_LINK_FLAGS STREQUAL "")
    set(_cap_LINK_FLAGS ${_cap_COMPILE_FLAGS})
  endif()
  if(NOT DEFINED _cap_SOURCE OR _cap_SOURCE STREQUAL "")
    set(_cap_SOURCE "${_bike_capability_probe_dir}/probe.cpp")
  endif()
  set(_cap_include_flags "")
  foreach(_cap_include_dir IN LISTS _cap_INCLUDE_DIRS)
    list(APPEND _cap_include_flags "-I${_cap_include_dir}")
  endforeach()
  try_compile(${result_var}
    "${_bike_capability_probe_dir}/build-${label}"
    SOURCES "${_cap_SOURCE}"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags}
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS ${_cap_COMPILE_FLAGS} ${_cap_include_flags}
    LINK_OPTIONS ${_cap_LINK_FLAGS}
    OUTPUT_VARIABLE _bike_cap_probe_output)
  set(${result_var} "${${result_var}}" PARENT_SCOPE)
  list(JOIN _cap_COMPILE_FLAGS " " _cap_compile_text)
  list(JOIN _cap_LINK_FLAGS " " _cap_link_text)
  if(${result_var})
    set(${result_var}_EVIDENCE
      "compile+link probe passed: compile='${_cap_compile_text}' link='${_cap_link_text}'"
      PARENT_SCOPE)
  else()
    set(${result_var}_EVIDENCE
      "compile+link probe failed: compile='${_cap_compile_text}' link='${_cap_link_text}'"
      PARENT_SCOPE)
  endif()
endfunction()

if(_bike_clang_family)
  # The probe must represent the lane actually enabled by NATIVE_SANITIZE:
  # the full -fsanitize spelling set and clang's shared runtime link, so a
  # toolchain missing the shared ASan runtime reports supported=false.
  set(_bike_cap_compile_asan
    -fsanitize=address,undefined,local-bounds,float-cast-overflow
    -fno-sanitize-recover=all -fno-omit-frame-pointer)
  set(_bike_cap_link_asan ${_bike_cap_compile_asan} -shared-libasan)
  set(_bike_cap_compile_coverage -fprofile-instr-generate -fcoverage-mapping)
else()
  # The GCC lane's ASan core omits the clang-only -fsanitize spellings.
  set(_bike_cap_compile_asan
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  set(_bike_cap_link_asan ${_bike_cap_compile_asan})
  set(_bike_cap_compile_coverage -fprofile-arcs -ftest-coverage)
endif()
# -fsanitize=realtime has no GCC spelling — the probe itself measures that.
# Its source is rtsan_probe.cpp above: flag, feature query and annotation
# together, so a compiler that accepts the spelling without the realtime
# instrumentation contract still reports supported=false.
set(_bike_cap_compile_rtsan -fsanitize=realtime)
set(_bike_cap_link_rtsan ${_bike_cap_compile_rtsan})

_bike_probe_capability(BIKE_CAP_ASAN_SUPPORTED asan
  COMPILE_FLAGS ${_bike_cap_compile_asan} LINK_FLAGS ${_bike_cap_link_asan})
_bike_probe_capability(BIKE_CAP_RTSAN_SUPPORTED rtsan
  COMPILE_FLAGS ${_bike_cap_compile_rtsan} LINK_FLAGS ${_bike_cap_link_rtsan}
  SOURCE "${_bike_capability_probe_dir}/rtsan_probe.cpp"
  INCLUDE_DIRS "${BIKE_NATIVE_ROOT}/src")
_bike_probe_capability(BIKE_CAP_COVERAGE_SUPPORTED coverage
  COMPILE_FLAGS ${_bike_cap_compile_coverage})

# TSan is never represented as supported on the known-broken Darwin lane.
# Probe instrumentation plus linking, not flag spelling alone. The enabled
# lane below also executes a clean control using the same linker-selected
# runtime as native executables (no foreign preload).
set(_bike_cap_compile_tsan -fsanitize=thread -fno-omit-frame-pointer -pthread)
set(_bike_cap_link_tsan -fsanitize=thread -pthread)
if(BIKE_TARGET_SYSTEM STREQUAL "Linux")
  file(WRITE "${_bike_capability_probe_dir}/tsan_probe.cpp"
    "#if defined(__SANITIZE_THREAD__)\n"
    "#elif defined(__has_feature)\n"
    "#if !__has_feature(thread_sanitizer)\n#error TSan instrumentation missing\n#endif\n"
    "#else\n#error TSan instrumentation missing\n#endif\n"
    "#include <thread>\n"
    "int main() { int value=0; std::thread worker([&]{value=1;}); worker.join(); return value==1?0:1; }\n")
  _bike_probe_capability(BIKE_CAP_TSAN_SUPPORTED tsan
    COMPILE_FLAGS ${_bike_cap_compile_tsan} LINK_FLAGS ${_bike_cap_link_tsan}
    SOURCE "${_bike_capability_probe_dir}/tsan_probe.cpp")
else()
  set(BIKE_CAP_TSAN_SUPPORTED FALSE)
  set(BIKE_CAP_TSAN_SUPPORTED_EVIDENCE "unsupported target; TSan is Linux-only")
endif()


# Coverage toolchain qualification (T17) — a requested coverage lane is
# qualified only when BOTH gates pass under this toolchain:
#   1. the flags probe above (compile+link with -fprofile-instr-generate
#      -fcoverage-mapping), and
#   2. CoverageTools.cmake's Python-free resolution against the
#      selected compiler: explicit LLVM_PROFDATA/LLVM_COV override, then a
#      compiler sibling, then a PATH candidate, then the Darwin-only xcrun
#      fallback — followed by the real instrumented format probe (compile,
#      run under a fresh LLVM_PROFILE_FILE, merge, export). A resolved pair
#      that cannot merge/export the compiler's own control profile is a
#      configure-time rejection, not an empty report at finalize time.
# The capability record stays honest when the lane is unrequested:
# `supported` describes the flag probe alone, and the tool evidence records
# that qualification was not run.
if(NATIVE_COVERAGE AND NOT BIKE_CAP_COVERAGE_SUPPORTED)
  message(FATAL_ERROR
    "NATIVE_COVERAGE was requested but the selected toolchain "
    "${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_ID} "
    "${CMAKE_CXX_COMPILER_VERSION}) failed the coverage instrumentation "
    "probe for -fprofile-instr-generate -fcoverage-mapping: "
    "${BIKE_CAP_COVERAGE_SUPPORTED_EVIDENCE}")
endif()

set(BIKE_CAP_COVERAGE_TOOLS "coverage tools not qualified (lane unrequested)")
set(BIKE_CAP_COVERAGE_FORMAT_PROBE
  "format probe not run (lane unrequested)")
if(NATIVE_COVERAGE)
  include("${CMAKE_CURRENT_LIST_DIR}/CoverageTools.cmake")
  bike_qualify_coverage_tools()
endif()

# --------------------------------------------------------------------------
# Sanitizer runtime resolution (T15). The recorded compiler *identity*
# selects the runtime name — libasan.so for GNU, the target-suffixed
# libclang_rt.<san>-<machine>.so for upstream Clang on Linux, the
# *_osx_dynamic.dylib spellings on Darwin — and the configured driver
# resolves it through -print-file-name. The driver basename is never
# consulted and an unresolved echo is a failed resolution. An enabled lane
# additionally proves the runtime by compiling and running a clean control
# under the same preload variable tools/native_runtime.py injects.
# --------------------------------------------------------------------------

# The driver's own target triple — recorded into every sanitizer capability
# record so the context carries toolchain evidence, not host guesses.
execute_process(
  COMMAND "${CMAKE_CXX_COMPILER}" -dumpmachine
  RESULT_VARIABLE _bike_target_rc
  OUTPUT_VARIABLE _bike_target_out
  ERROR_QUIET
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_bike_target_rc EQUAL 0 AND NOT _bike_target_out STREQUAL "")
  set(BIKE_SANITIZER_TARGET "${_bike_target_out}")
else()
  set(BIKE_SANITIZER_TARGET "")
endif()

# RTSan probe-cache invalidation (T16): check_cxx_source_compiles caches its
# result in this build's CMakeCache, and the cache must never qualify a lane
# measured under a different toolchain. The signature binds every cached
# RTSan probe result to the selected compiler's path, identity, version and
# target; a change drops the cached results so they re-probe.
set(_bike_rtsan_signature
  "${CMAKE_CXX_COMPILER}|${CMAKE_CXX_COMPILER_ID}|${CMAKE_CXX_COMPILER_VERSION}|${BIKE_TARGET_SYSTEM}|${BIKE_POLICY_MACHINE}|${BIKE_SANITIZER_TARGET}")
if(DEFINED BIKE_RTSAN_TOOLCHAIN_SIGNATURE
   AND NOT BIKE_RTSAN_TOOLCHAIN_SIGNATURE STREQUAL _bike_rtsan_signature)
  # try_compile results are uncached and re-probe every configure; the
  # CheckCXXSourceCompiles result below is the one cached value that must
  # be dropped on a toolchain change.
  unset(BIKE_RTSAN_INSTRUMENTATION_LINKS CACHE)
endif()
set(BIKE_RTSAN_TOOLCHAIN_SIGNATURE "${_bike_rtsan_signature}"
  CACHE INTERNAL
  "toolchain signature the cached RTSan probes were computed under")

function(_bike_sanitizer_runtime_name out_var sanitizer)
  if(_bike_clang_family)
    if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
      set(${out_var} "libclang_rt.${sanitizer}_osx_dynamic.dylib" PARENT_SCOPE)
    elseif(BIKE_POLICY_MACHINE STREQUAL "aarch64"
           OR BIKE_POLICY_MACHINE STREQUAL "x86_64")
      set(${out_var}
        "libclang_rt.${sanitizer}-${BIKE_POLICY_MACHINE}.so" PARENT_SCOPE)
    else()
      set(${out_var} "" PARENT_SCOPE)
    endif()
  elseif(sanitizer STREQUAL "asan")
    if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
      set(${out_var} "libasan.dylib" PARENT_SCOPE)
    else()
      set(${out_var} "libasan.so" PARENT_SCOPE)
    endif()
  else()
    # GCC has no RTSan runtime — the flag probe already rejected the lane.
    set(${out_var} "" PARENT_SCOPE)
  endif()
endfunction()

# Resolve a runtime filename through the SELECTED driver's -print-file-name.
# An unresolved echo (the driver prints the bare name back), a non-absolute
# or nonexistent answer is a failed resolution — never a fabricated path.
function(_bike_driver_file_name out_var query_var name)
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" "-print-file-name=${name}"
    RESULT_VARIABLE _drv_rc
    OUTPUT_VARIABLE _drv_out
    ERROR_VARIABLE _drv_err
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "^[^\r\n]*" _drv_first "${_drv_out}")
  set(${query_var}
    "${CMAKE_CXX_COMPILER} -print-file-name=${name} -> rc=${_drv_rc} '${_drv_first}'"
    PARENT_SCOPE)
  if(_drv_rc EQUAL 0 AND NOT _drv_first STREQUAL ""
     AND NOT _drv_first STREQUAL "${name}"
     AND IS_ABSOLUTE "${_drv_first}" AND EXISTS "${_drv_first}"
     AND NOT IS_DIRECTORY "${_drv_first}")
    get_filename_component(_drv_resolved "${_drv_first}" ABSOLUTE)
    set(${out_var} "${_drv_resolved}" PARENT_SCOPE)
  else()
    set(${out_var} "" PARENT_SCOPE)
  endif()
endfunction()

# _bike_run_sanitizer_control(<sanitizer> <runtime> <rc-var> <detail-var>)
# Compile the clean control with the lane's real instrumentation and run it
# with its linked runtime for the root graph, or the binding runner
# preload contract. Any nonzero exit — a loader failure included — means the selected runtime cannot carry an
# instrumented process.
function(_bike_run_sanitizer_control sanitizer runtime rc_var detail_var)
  set(_control_dir "${_bike_capability_probe_dir}/run-${sanitizer}")
  file(MAKE_DIRECTORY "${_control_dir}")
  file(WRITE "${_control_dir}/control.cpp"
    "#include <cstdlib>\n"
    "int main() {\n"
    "  void *block = std::malloc(16);\n"
    "  std::free(block);\n"
    "  return block == nullptr ? 1 : 0;\n"
    "}\n")
  set(_control_rpath_flags)
  if(BIKE_ROOT_BUILD)
    get_filename_component(_runtime_directory "${runtime}" DIRECTORY)
    list(APPEND _control_rpath_flags "-DCMAKE_BUILD_RPATH:STRING=${_runtime_directory}")
  endif()
  try_compile(_control_compiled
    "${_control_dir}/build"
    SOURCES "${_control_dir}/control.cpp"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags} ${_control_rpath_flags}
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS ${_bike_cap_compile_${sanitizer}}
    LINK_OPTIONS ${_bike_cap_link_${sanitizer}}
    COPY_FILE "${_control_dir}/control_exe"
    OUTPUT_VARIABLE _control_build_log)
  if(NOT _control_compiled OR NOT EXISTS "${_control_dir}/control_exe")
    set(${rc_var} -1 PARENT_SCOPE)
    set(${detail_var}
      "control build failed: ${_control_build_log}" PARENT_SCOPE)
    return()
  endif()
  if(BIKE_ROOT_BUILD)
    set(_preload_arguments --unset=LD_PRELOAD --unset=DYLD_INSERT_LIBRARIES)
    set(_runtime_launch_description "linked runtime (no preload)")
  else()
    if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
      set(_preload_var "DYLD_INSERT_LIBRARIES")
      set(_existing_preload "$ENV{DYLD_INSERT_LIBRARIES}")
    else()
      set(_preload_var "LD_PRELOAD")
      set(_existing_preload "$ENV{LD_PRELOAD}")
    endif()
    if(_existing_preload STREQUAL "")
      set(_preload_value "${runtime}")
    else()
      set(_preload_value "${runtime}:${_existing_preload}")
    endif()
    set(_preload_arguments "${_preload_var}=${_preload_value}")
    set(_runtime_launch_description "${_preload_var}=<selected>")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
      ${_preload_arguments}
      "${_control_dir}/control_exe"
    RESULT_VARIABLE _control_rc
    OUTPUT_VARIABLE _control_out
    ERROR_VARIABLE _control_err
    TIMEOUT 60)
  string(STRIP "${_control_out}" _control_out)
  string(STRIP "${_control_err}" _control_err)
  set(${rc_var} "${_control_rc}" PARENT_SCOPE)
  set(${detail_var}
    "control ran with ${_runtime_launch_description}: rc=${_control_rc} "
    "stdout='${_control_out}' stderr='${_control_err}'" PARENT_SCOPE)
endfunction()

# _bike_run_rtsan_violation_control(<runtime> <result-var> <detail-var>)
# The RTSan lane's negative gate: build the checked-in violation control
# (native/tests/rtsan_control.cpp) with the lane's real instrumentation and
# run its forbidden-allocation mode with the graph's runtime launch contract.
# TRUE is returned only when the process dies with the real
# RealtimeSanitizer report — a clean exit proves the annotation never
# instrumented the call, and a loader failure or unrelated signal without
# the report is not sanitizer evidence.
function(_bike_run_rtsan_violation_control runtime result_var detail_var)
  set(_violation_dir "${_bike_capability_probe_dir}/violation-rtsan")
  file(MAKE_DIRECTORY "${_violation_dir}")
  set(_control_rpath_flags)
  if(BIKE_ROOT_BUILD)
    get_filename_component(_runtime_directory "${runtime}" DIRECTORY)
    list(APPEND _control_rpath_flags "-DCMAKE_BUILD_RPATH:STRING=${_runtime_directory}")
  endif()
  try_compile(_violation_compiled
    "${_violation_dir}/build"
    SOURCES "${BIKE_NATIVE_ROOT}/tests/rtsan_control.cpp"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags} ${_control_rpath_flags}
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS -fsanitize=realtime
    LINK_OPTIONS -fsanitize=realtime
    COPY_FILE "${_violation_dir}/violation_exe"
    OUTPUT_VARIABLE _violation_build_log)
  if(NOT _violation_compiled
     OR NOT EXISTS "${_violation_dir}/violation_exe")
    set(${result_var} FALSE PARENT_SCOPE)
    set(${detail_var}
      "violation control build failed under -fsanitize=realtime: "
      "${_violation_build_log}" PARENT_SCOPE)
    return()
  endif()
  if(BIKE_ROOT_BUILD)
    set(_preload_arguments --unset=LD_PRELOAD --unset=DYLD_INSERT_LIBRARIES)
    set(_runtime_launch_description "linked runtime (no preload)")
  else()
    if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
      set(_preload_var "DYLD_INSERT_LIBRARIES")
      set(_existing_preload "$ENV{DYLD_INSERT_LIBRARIES}")
    else()
      set(_preload_var "LD_PRELOAD")
      set(_existing_preload "$ENV{LD_PRELOAD}")
    endif()
    if(_existing_preload STREQUAL "")
      set(_preload_value "${runtime}")
    else()
      set(_preload_value "${runtime}:${_existing_preload}")
    endif()
    set(_preload_arguments "${_preload_var}=${_preload_value}")
    set(_runtime_launch_description "${_preload_var}=<selected>")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
      ${_preload_arguments}
      "${_violation_dir}/violation_exe" forbidden-allocation
    RESULT_VARIABLE _violation_rc
    OUTPUT_VARIABLE _violation_out
    ERROR_VARIABLE _violation_err
    TIMEOUT 60)
  string(STRIP "${_violation_out}" _violation_out)
  string(STRIP "${_violation_err}" _violation_err)
  set(_violation_combined "${_violation_out}\n${_violation_err}")
  if(_violation_rc EQUAL 0)
    set(${result_var} FALSE PARENT_SCOPE)
    set(${detail_var}
      "violation control exited 0 — BIKE_NONBLOCKING produced no realtime "
      "interception for the forbidden malloc/free pair" PARENT_SCOPE)
  elseif(NOT _violation_combined MATCHES "RealtimeSanitizer"
         OR NOT _violation_combined MATCHES "malloc")
    set(${result_var} FALSE PARENT_SCOPE)
    set(${detail_var}
      "violation control died without a RealtimeSanitizer report "
      "(rc=${_violation_rc}): '${_violation_combined}'" PARENT_SCOPE)
  else()
    set(${result_var} TRUE PARENT_SCOPE)
    set(${detail_var}
      "violation control reported the RealtimeSanitizer diagnostic "
      "(rc=${_violation_rc}, ${_runtime_launch_description}): "
      "'${_violation_combined}'" PARENT_SCOPE)
  endif()
endfunction()

_bike_sanitizer_runtime_name(BIKE_ASAN_RUNTIME_NAME asan)
_bike_sanitizer_runtime_name(BIKE_RTSAN_RUNTIME_NAME rtsan)

set(BIKE_CAP_ASAN_RUNTIME "")
set(BIKE_CAP_ASAN_RUNTIME_QUERY "asan runtime not queried")
set(BIKE_CAP_ASAN_RUN_CONTROL "asan run control not executed")
set(BIKE_CAP_ASAN_ANNOTATION_CONTROL
  "annotation gate not applicable to the asan lane")
set(BIKE_CAP_ASAN_MACRO_CONTROL "macro gate not applicable to the asan lane")
set(BIKE_CAP_ASAN_VIOLATION_CONTROL
  "asan violation evidence lives in sanitizer_negative_control (root) and the pytest subprocess control (binding)")
set(BIKE_CAP_RTSAN_RUNTIME "")
set(BIKE_CAP_RTSAN_RUNTIME_QUERY "rtsan runtime not queried")
set(BIKE_CAP_RTSAN_RUN_CONTROL "rtsan run control not executed")
set(BIKE_CAP_RTSAN_ANNOTATION_CONTROL "rtsan annotation control not executed")
set(BIKE_CAP_RTSAN_MACRO_CONTROL "rtsan macro control not executed")
set(BIKE_CAP_RTSAN_VIOLATION_CONTROL "rtsan violation control not executed")
set(BIKE_CAP_COVERAGE_RUNTIME "")
set(BIKE_CAP_COVERAGE_RUNTIME_QUERY "coverage has no preload runtime")
set(BIKE_CAP_COVERAGE_RUN_CONTROL "coverage has no preload runtime")
set(BIKE_CAP_COVERAGE_ANNOTATION_CONTROL
  "annotation gate not applicable to the coverage lane")
set(BIKE_CAP_COVERAGE_MACRO_CONTROL
  "macro gate not applicable to the coverage lane")
set(BIKE_CAP_COVERAGE_VIOLATION_CONTROL
  "violation gate not applicable to the coverage lane")
set(BIKE_CAP_ASAN_TOOLS "coverage-tool evidence is not applicable to the asan lane")
set(BIKE_CAP_ASAN_FORMAT_PROBE
  "coverage format probe is not applicable to the asan lane")
set(BIKE_CAP_RTSAN_TOOLS "coverage-tool evidence is not applicable to the rtsan lane")
set(BIKE_CAP_RTSAN_FORMAT_PROBE
  "coverage format probe is not applicable to the rtsan lane")

if(BIKE_CAP_ASAN_SUPPORTED AND NOT BIKE_ASAN_RUNTIME_NAME STREQUAL "")
  _bike_driver_file_name(BIKE_CAP_ASAN_RUNTIME BIKE_CAP_ASAN_RUNTIME_QUERY
    "${BIKE_ASAN_RUNTIME_NAME}")
endif()
if(BIKE_CAP_RTSAN_SUPPORTED AND NOT BIKE_RTSAN_RUNTIME_NAME STREQUAL "")
  _bike_driver_file_name(BIKE_CAP_RTSAN_RUNTIME BIKE_CAP_RTSAN_RUNTIME_QUERY
    "${BIKE_RTSAN_RUNTIME_NAME}")
endif()

set(BIKE_CAP_TSAN_RUNTIME "")
set(BIKE_CAP_TSAN_RUNTIME_QUERY "TSan runtime not queried on this target")
set(BIKE_CAP_TSAN_RUN_CONTROL "TSan clean control not executed")
set(BIKE_CAP_TSAN_ANNOTATION_CONTROL "TSan instruments all accesses; no function annotation")
set(BIKE_CAP_TSAN_MACRO_CONTROL "instrumented feature macro is checked by the compile control")
set(BIKE_CAP_TSAN_VIOLATION_CONTROL "pending V09: bike_spatial_worker_pool --tsan-race-control")
set(BIKE_CAP_TSAN_TOOLS "coverage-tool evidence is not applicable to TSan")
set(BIKE_CAP_TSAN_FORMAT_PROBE "coverage format is not applicable to TSan")
if(BIKE_CAP_TSAN_SUPPORTED)
  if(_bike_clang_family)
    # Clang links compiler-rt's archive by default; this is the actual linked
    # runtime identity, not a preload request for a similarly named DSO.
    set(_bike_tsan_runtime_name "libclang_rt.tsan-${BIKE_POLICY_MACHINE}.a")
  else()
    set(_bike_tsan_runtime_name "libtsan.so")
  endif()
  _bike_driver_file_name(BIKE_CAP_TSAN_RUNTIME BIKE_CAP_TSAN_RUNTIME_QUERY "${_bike_tsan_runtime_name}")
endif()
if(NATIVE_TSAN)
  if(NOT BIKE_CAP_TSAN_SUPPORTED OR NOT BIKE_CAP_TSAN_RUNTIME)
    message(FATAL_ERROR "NATIVE_TSAN requires instrumented compile/link and a compiler-resolved runtime: ${BIKE_CAP_TSAN_SUPPORTED_EVIDENCE}; ${BIKE_CAP_TSAN_RUNTIME_QUERY}")
  endif()
  try_run(_bike_tsan_run_rc _bike_tsan_compile
    "${_bike_capability_probe_dir}/run-tsan"
    SOURCES "${_bike_capability_probe_dir}/tsan_probe.cpp"
    CMAKE_FLAGS "-DCMAKE_CXX_STANDARD:STRING=23" "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS ${_bike_cap_compile_tsan}
    LINK_OPTIONS ${_bike_cap_link_tsan}
    RUN_OUTPUT_VARIABLE _bike_tsan_run_output
    COMPILE_OUTPUT_VARIABLE _bike_tsan_compile_output)
  if(NOT _bike_tsan_compile OR NOT _bike_tsan_run_rc EQUAL 0)
    message(FATAL_ERROR "TSan clean instrumented control failed on this Linux host: ${_bike_tsan_compile_output}; ${_bike_tsan_run_output}")
  endif()
  set(BIKE_CAP_TSAN_RUN_CONTROL "clean instrumented thread/join control exited 0 with the compiler-linked runtime")
endif()

# A requested-but-unsupported sanitizer fails at configure time — a build
# that adds the flags anyway would break at the first instrumented TU.
if(NATIVE_SANITIZE AND NOT BIKE_CAP_ASAN_SUPPORTED)
  message(FATAL_ERROR
    "NATIVE_SANITIZE was requested but the selected toolchain "
    "${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_ID} "
    "${CMAKE_CXX_COMPILER_VERSION}) failed the ASan lane probe: "
    "${BIKE_CAP_ASAN_SUPPORTED_EVIDENCE}")
endif()
if(NATIVE_RTSAN AND NOT BIKE_CAP_RTSAN_SUPPORTED)
  message(FATAL_ERROR
    "NATIVE_RTSAN was requested but the selected toolchain "
    "${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_ID} "
    "${CMAKE_CXX_COMPILER_VERSION}) failed the RTSan lane probe for "
    "-fsanitize=realtime plus the BIKE_NONBLOCKING annotation: "
    "${BIKE_CAP_RTSAN_SUPPORTED_EVIDENCE}")
endif()

# Enabled lanes must resolve a compiler-matched runtime AND prove it runs a
# clean instrumented control under the runner's preload contract.
if(NATIVE_SANITIZE)
  if(NOT BIKE_CAP_ASAN_RUNTIME)
    message(FATAL_ERROR
      "NATIVE_SANITIZE requires a resolvable ASan runtime; the selected "
      "compiler ${CMAKE_CXX_COMPILER} could not resolve "
      "${BIKE_ASAN_RUNTIME_NAME}: ${BIKE_CAP_ASAN_RUNTIME_QUERY}")
  endif()
  _bike_run_sanitizer_control(asan "${BIKE_CAP_ASAN_RUNTIME}"
    _bike_asan_run_rc _bike_asan_run_detail)
  set(BIKE_CAP_ASAN_RUN_CONTROL "${_bike_asan_run_detail}")
  if(NOT _bike_asan_run_rc EQUAL 0)
    message(FATAL_ERROR
      "the ASan runtime control failed to run cleanly under the selected "
      "runtime ${BIKE_CAP_ASAN_RUNTIME}: ${_bike_asan_run_detail}")
  endif()
endif()
if(NATIVE_RTSAN)
  # T16 qualification gates, in order: flag+annotation compile/link, the
  # load-bearing annotation proof, compiler-matched runtime resolution,
  # the clean run control, and the forbidden-allocation violation control.
  # Every gate fails the configure — an RTSan lane is qualified only when
  # all of them passed under THIS toolchain.

  # Requested-mode compile control: the brief's CMakePushCheckState/
  # CheckCXXSourceCompiles pattern — CMAKE_REQUIRED_FLAGS carries the real
  # lane flag, CMAKE_REQUIRED_INCLUDES makes "rtsan.hpp" resolve, and the
  # source requires __has_feature(realtime_sanitizer) AND a BIKE_NONBLOCKING
  # function to compile+link. BIKE_NONBLOCKING is in the trailing
  # declarator position — since LLVM 20 [[clang::nonblocking]] is a
  # function *type* attribute and the leading position is a hard error.
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS "-std=c++23 -fsanitize=realtime")
  set(CMAKE_REQUIRED_INCLUDES "${BIKE_NATIVE_ROOT}/src")
  check_cxx_source_compiles([=[
#include "rtsan.hpp"
#if !defined(__has_feature)
#error realtime sanitizer feature query is unavailable
#elif !__has_feature(realtime_sanitizer)
#error realtime sanitizer instrumentation is unavailable
#endif
void control() BIKE_NONBLOCKING {}
int main() { control(); return 0; }
]=] BIKE_RTSAN_INSTRUMENTATION_LINKS)
  cmake_pop_check_state()
  set(BIKE_CAP_RTSAN_ANNOTATION_CONTROL
    "flag+annotation compile/link control: -fsanitize=realtime with "
    "rtsan.hpp, __has_feature(realtime_sanitizer) and a BIKE_NONBLOCKING "
    "function")
  if(NOT BIKE_RTSAN_INSTRUMENTATION_LINKS)
    message(FATAL_ERROR
      "NATIVE_RTSAN requested but the RTSan compiler/annotation/link "
      "control failed under ${CMAKE_CXX_COMPILER} "
      "(${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}): "
      "-fsanitize=realtime plus the BIKE_NONBLOCKING annotation did not "
      "compile and link (see CMakeError.log)")
  endif()

  # The macro must be load-bearing, not an empty expansion: a malloc/free
  # pair inside a BIKE_NONBLOCKING function must FAIL to compile under the
  # function-effects checker, and the diagnostic must name the attribute.
  # A clean compile means the macro expanded to nothing; an unrelated
  # failure (unknown warning option, etc.) proves nothing either way.
  file(WRITE "${_bike_policy_probe_dir}/rtsan_macro_active.cpp"
    "#include <cstdlib>\n"
    "#include \"rtsan.hpp\"\n"
    "void forbidden_allocation() BIKE_NONBLOCKING {\n"
    "  void* volatile pointer = std::malloc(32);\n"
    "  std::free(pointer);\n"
    "}\n"
    "int main() { forbidden_allocation(); return 0; }\n")
  try_compile(BIKE_RTSAN_ANNOTATION_INERT
    "${_bike_policy_probe_dir}/rtsan-macro-active"
    SOURCES "${_bike_policy_probe_dir}/rtsan_macro_active.cpp"
    CMAKE_FLAGS
      ${_bike_policy_sysroot_flags}
      "-DCMAKE_CXX_STANDARD:STRING=23"
      "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    COMPILE_DEFINITIONS
      -fsanitize=realtime -Werror -Wfunction-effects
      "-I${BIKE_NATIVE_ROOT}/src"
    LINK_OPTIONS -fsanitize=realtime
    OUTPUT_VARIABLE _bike_rtsan_macro_output)
  set(BIKE_CAP_RTSAN_MACRO_CONTROL
    "macro-active control: forbidden malloc/free inside BIKE_NONBLOCKING "
    "must fail under -fsanitize=realtime -Werror -Wfunction-effects "
    "(compiled=${BIKE_RTSAN_ANNOTATION_INERT})")
  if(BIKE_RTSAN_ANNOTATION_INERT)
    message(FATAL_ERROR
      "NATIVE_RTSAN requested but BIKE_NONBLOCKING did not activate the "
      "function-effects checker on ${CMAKE_CXX_COMPILER} "
      "(${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}) — the "
      "macro expanded to nothing under -fsanitize=realtime")
  endif()
  if(NOT _bike_rtsan_macro_output MATCHES "nonblocking")
    message(FATAL_ERROR
      "NATIVE_RTSAN requested but the annotation control failed for an "
      "unrelated reason under ${CMAKE_CXX_COMPILER} "
      "(${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}) — "
      "expected the -Wfunction-effects diagnostic naming 'nonblocking':\n"
      "${_bike_rtsan_macro_output}")
  endif()

  if(NOT BIKE_CAP_RTSAN_RUNTIME)
    message(FATAL_ERROR
      "NATIVE_RTSAN requires a resolvable RTSan runtime; the selected "
      "compiler ${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_ID} "
      "${CMAKE_CXX_COMPILER_VERSION}) could not resolve "
      "${BIKE_RTSAN_RUNTIME_NAME}: ${BIKE_CAP_RTSAN_RUNTIME_QUERY}")
  endif()
  _bike_run_sanitizer_control(rtsan "${BIKE_CAP_RTSAN_RUNTIME}"
    _bike_rtsan_run_rc _bike_rtsan_run_detail)
  set(BIKE_CAP_RTSAN_RUN_CONTROL "${_bike_rtsan_run_detail}")
  if(NOT _bike_rtsan_run_rc EQUAL 0)
    message(FATAL_ERROR
      "the RTSan runtime control failed to run cleanly under the selected "
      "runtime ${BIKE_CAP_RTSAN_RUNTIME} for compiler "
      "${CMAKE_CXX_COMPILER}: ${_bike_rtsan_run_detail}")
  endif()
  _bike_run_rtsan_violation_control("${BIKE_CAP_RTSAN_RUNTIME}"
    _bike_rtsan_violation_ok _bike_rtsan_violation_detail)
  set(BIKE_CAP_RTSAN_VIOLATION_CONTROL "${_bike_rtsan_violation_detail}")
  if(NOT _bike_rtsan_violation_ok)
    message(FATAL_ERROR
      "NATIVE_RTSAN requested but the violation control did not report a "
      "RealtimeSanitizer diagnostic under ${CMAKE_CXX_COMPILER} "
      "(${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}): "
      "${_bike_rtsan_violation_detail}")
  endif()
endif()

# --------------------------------------------------------------------------
# Assemble the effective policy.
# --------------------------------------------------------------------------

set(BIKE_POLICY_COMPILE_OPTIONS
  ${_bike_applied_warnings}
  ${_bike_required_flags}
  ${_bike_arch_flags}
  # The previous Release compile line ended in -Os (CMAKE's -O3, then
  # nanobind's -Os, last wins); -O2 measures ~7% faster than that stack on
  # this runtime's branchy code (verified arm64-apple, ~0.181 vs ~0.194
  # ms/step on the same session). Appending -O2 here overrides inherited
  # -O flags since the last one wins; this does not stomp a caller's -O
  # choice for other build types, and the option is recorded into
  # research_identity flags. Keeping it last inside this list preserves
  # the ordering after nanobind's own target_compile_options.
  $<$<CONFIG:Release>:-O2>
  ${_bike_instrument_options})

# Prove each required flag fail-closed (after assembling, so the test
# injection point shares the same enforcement path as the real set).
foreach(_required IN LISTS _bike_required_flags)
  _bike_policy_flag_var("${_required}" _required_var)
  bike_require_cxx_flag("${_required}" ${_required_var})
endforeach()

set(BIKE_POLICY_COMPILE_DEFINITIONS "")
if(_native_libcpp_hardening_supported)
  list(APPEND BIKE_POLICY_COMPILE_DEFINITIONS
    "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_${BIKE_LIBCPP_HARDENING}")
elseif(BIKE_STDLIB_IS_LIBSTDCXX)
  list(APPEND BIKE_POLICY_COMPILE_DEFINITIONS "_GLIBCXX_ASSERTIONS")
endif()

# _FORTIFY_SOURCE=3: the ASan driver predefines it (and intercepts the same
# functions anyway) — redefining would trip -Wmacro-redefined. On glibc the
# fortify shims only exist under optimization, so the define is applied to
# the optimized configurations only (Debug/default record the omission
# explicitly). Non-glibc libcs tolerate it in every config — macOS keeps
# the previous unconditional behavior.
set(_bike_fortify_mode "disabled")
set(_bike_fortify_detail "")
if(NATIVE_SANITIZE)
  set(_bike_fortify_mode "disabled")
  set(_bike_fortify_detail
    "ASan driver predefines _FORTIFY_SOURCE; redefining it would trip -Wmacro-redefined")
elseif(BIKE_TARGET_GLIBC)
  set(_bike_fortify_mode "optimized-configs")
  set(_bike_fortify_detail
    "glibc fortify is inert without __OPTIMIZE__; applied to Release/RelWithDebInfo/MinSizeRel only")
  list(APPEND BIKE_POLICY_COMPILE_DEFINITIONS
    "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:_FORTIFY_SOURCE=3>")
else()
  set(_bike_fortify_mode "all-configs")
  set(_bike_fortify_detail "non-glibc libc; applied in every build type")
  list(APPEND BIKE_POLICY_COMPILE_DEFINITIONS "_FORTIFY_SOURCE=3")
endif()

set(BIKE_POLICY_LINK_OPTIONS "")
if(BIKE_POLICY_SYSTEM STREQUAL "Darwin")
  # Reserves room for the install_name_tool rewrite of the MuJoCo dylib
  # install name (the wheel ships the dylib flat, without a framework
  # anchor for @rpath). Apple linker only.
  check_linker_flag(CXX "-headerpad_max_install_names"
    BIKE_LINKER_HEADERPAD_MAX_INSTALL_NAMES)
  if(BIKE_LINKER_HEADERPAD_MAX_INSTALL_NAMES)
    list(APPEND BIKE_POLICY_LINK_OPTIONS "-headerpad_max_install_names")
  else()
    message(FATAL_ERROR
      "Darwin build requires the linker flag -headerpad_max_install_names")
  endif()
endif()
list(APPEND BIKE_POLICY_LINK_OPTIONS ${_bike_instrument_link_options})

# Fast-math/reassociation flags are contract violations: they change
# float results and break bitwise parity with the Python oracle. Reject
# them in the final policy list rather than merely never emitting them.
foreach(_option IN LISTS BIKE_POLICY_COMPILE_OPTIONS)
  if(_option MATCHES "^-f(fast-math|finite-math-only|associative-math|reciprocal-math|unsafe-math-optimizations|cx-limited-range|rounding-math|signaling-nans)"
     OR _option MATCHES "^-ffp-contract=(fast|on)"
     OR _option MATCHES "^-mrecip"
     OR _option STREQUAL "-Ofast")
    message(FATAL_ERROR
      "native policy produced a forbidden fast-math/reassociation flag: ${_option}")
  endif()
endforeach()

# --------------------------------------------------------------------------
# bike_apply_native_policy — DIRECT (not transitive-only) application.
# --------------------------------------------------------------------------

function(bike_apply_native_policy target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR
      "bike_apply_native_policy: '${target}' is not a CMake target")
  endif()
  # Strict flags stay PRIVATE to our TUs: nanobind's own static-lib sources
  # are not expected to survive this warning set under -Werror.
  target_compile_options(${target} PRIVATE ${BIKE_POLICY_COMPILE_OPTIONS})
  if(BIKE_POLICY_COMPILE_DEFINITIONS)
    target_compile_definitions(${target} PRIVATE
      ${BIKE_POLICY_COMPILE_DEFINITIONS})
  endif()
  if(BIKE_POLICY_LINK_OPTIONS)
    target_link_options(${target} PRIVATE ${BIKE_POLICY_LINK_OPTIONS})
  endif()
  # Root executables discover the shared sanitizer they link without an
  # interpreter preload. This is a build-tree loader path, never an
  # instrumentation change or an installed-package runtime dependency.
  if(BIKE_ROOT_BUILD)
    foreach(_runtime IN ITEMS "${BIKE_CAP_ASAN_RUNTIME}" "${BIKE_CAP_RTSAN_RUNTIME}")
      if((NATIVE_SANITIZE OR NATIVE_RTSAN) AND NOT _runtime STREQUAL "")
        get_filename_component(_runtime_directory "${_runtime}" DIRECTORY)
        set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${_runtime_directory}")
      endif()
    endforeach()
  endif()
endfunction()

# --------------------------------------------------------------------------
# Name bridges for the merged single-policy graph (merge ruling D6):
# bike_apply_native_policy is the single compiler-policy entry point for
# BOTH build graphs. The standalone root graph's HEAD-era spellings are
# kept here as thin aliases so the module swap in W2.3 is wiring-only and
# emitted schema-1 names stay identical.
# --------------------------------------------------------------------------

# The root graph's policy entry point — also the name BIKE_NUMERICS_APPLY_POLICY
# may carry. A function alias, never a second flag list.
function(bike_target_strict_policy target)
  bike_apply_native_policy("${target}")
endfunction()

# The root graph calls bike_detect_libcpp_hardening() after project() to
# populate these names in the caller's scope; this module probes at include()
# time, so the bridge re-exports the already-measured results with zero
# re-probing — the same contract Checks.cmake's emitted "libcpp_hardening"
# record reads.
function(bike_detect_libcpp_hardening)
  set(BIKE_LIBCPP_HARDENING_SUPPORTED ${_native_libcpp_hardening_supported}
    PARENT_SCOPE)
  set(BIKE_LIBCPP_HARDENING_EFFECTIVE "${_native_libcpp_hardening_mode}"
    PARENT_SCOPE)
endfunction()

# Directory-scope aliases as well: HEAD's bike_detect_libcpp_hardening()
# exported these names for consumers in the same directory scope; keeping
# them bound here means a caller that never invokes the detection function
# still reads the resolved values.
set(BIKE_LIBCPP_HARDENING_SUPPORTED ${_native_libcpp_hardening_supported})
set(BIKE_LIBCPP_HARDENING_EFFECTIVE "${_native_libcpp_hardening_mode}")

# --------------------------------------------------------------------------
# The recorded policy — consumed by the portability fixture and embedded
# verbatim as compiler_policy inside native_check_context.json.
# --------------------------------------------------------------------------

set(_bike_policy_omitted_json "[")
set(_bike_omit_separator "")
list(LENGTH BIKE_POLICY_OMITTED_WARNINGS _bike_omit_count)
if(_bike_omit_count GREATER 0)
  math(EXPR _bike_omit_last "${_bike_omit_count} - 1")
  foreach(_i RANGE ${_bike_omit_last})
    list(GET BIKE_POLICY_OMITTED_WARNINGS ${_i} _omit_flag)
    list(GET BIKE_POLICY_OMITTED_REASONS ${_i} _omit_reason)
    _native_json_quote(_omit_flag_json "${_omit_flag}")
    _native_json_quote(_omit_reason_json "${_omit_reason}")
    string(APPEND _bike_policy_omitted_json
      "${_bike_omit_separator}{\"flag\":${_omit_flag_json},\"reason\":${_omit_reason_json}}")
    set(_bike_omit_separator ",")
  endforeach()
endif()
string(APPEND _bike_policy_omitted_json "]")

if(BIKE_POLICY_OVERRIDE_SYSTEM OR BIKE_POLICY_OVERRIDE_MACHINE)
  set(_bike_policy_override_json
    "{\"system\":\"${BIKE_POLICY_OVERRIDE_SYSTEM}\",\"machine\":\"${BIKE_POLICY_OVERRIDE_MACHINE}\"}")
else()
  set(_bike_policy_override_json "null")
endif()
if(_native_libcpp_hardening_supported)
  set(_bike_libcpp_supported_json "true")
else()
  set(_bike_libcpp_supported_json "false")
endif()
if(BIKE_TARGET_GLIBC)
  set(_bike_glibc_json "true")
else()
  set(_bike_glibc_json "false")
endif()
if(_bike_branch_protection_applied)
  set(_bike_bp_applied_json "true")
else()
  set(_bike_bp_applied_json "false")
endif()
if(BIKE_CXX23_LIBRARY_USABLE)
  set(_bike_cxx23_json "true")
else()
  set(_bike_cxx23_json "false")
endif()

set(BIKE_POLICY_REQUIRED_FLAGS ${_bike_required_flags})
_native_json_array(_bike_required_json ${_bike_required_flags})
_native_json_array(_bike_warnings_json ${_bike_applied_warnings})
_native_json_array(_bike_options_json ${BIKE_POLICY_COMPILE_OPTIONS})
_native_json_array(_bike_definitions_json ${BIKE_POLICY_COMPILE_DEFINITIONS})
_native_json_array(_bike_link_json ${BIKE_POLICY_LINK_OPTIONS})
_native_json_quote(_bike_system_json "${BIKE_POLICY_SYSTEM}")
_native_json_quote(_bike_machine_json "${BIKE_POLICY_MACHINE}")
_native_json_quote(_bike_machine_raw_json "${_bike_policy_machine_raw}")
_native_json_quote(_bike_family_json "${BIKE_COMPILER_FAMILY}")
_native_json_quote(_bike_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
_native_json_quote(_bike_stdlib_json "${BIKE_STDLIB_IDENTITY}")
_native_json_quote(_bike_stdlib_version_json "${BIKE_STDLIB_VERSION}")
_native_json_quote(_bike_stdlib_detail_json "${_bike_stdlib_hardening_detail}")
_native_json_quote(_bike_fortify_mode_json "${_bike_fortify_mode}")
_native_json_quote(_bike_fortify_detail_json "${_bike_fortify_detail}")
_native_json_quote(_bike_bp_reason_json "${_bike_branch_protection_reason}")

set(BIKE_POLICY_REPORT_JSON
  "{\"schema_version\":1,\"system\":${_bike_system_json},\"machine\":${_bike_machine_json},\"machine_raw\":${_bike_machine_raw_json},\"override\":${_bike_policy_override_json},\"compiler_family\":${_bike_family_json},\"compiler_id\":${_bike_compiler_id_json},\"cxx_standard\":23,\"cxx_extensions\":false,\"cxx23_library\":{\"supported\":${_bike_cxx23_json},\"checked\":[\"std::format\",\"std::span\",\"std::bit_cast\"]},\"stdlib\":{\"identity\":${_bike_stdlib_json},\"version\":${_bike_stdlib_version_json},\"hardening\":${_bike_stdlib_detail_json},\"libcpp_hardening_supported\":${_bike_libcpp_supported_json}},\"glibc\":${_bike_glibc_json},\"fortify_source\":{\"level\":3,\"mode\":${_bike_fortify_mode_json},\"detail\":${_bike_fortify_detail_json}},\"branch_protection\":{\"applied\":${_bike_bp_applied_json},\"flag\":\"-mbranch-protection=standard\",\"reason\":${_bike_bp_reason_json}},\"required_flags\":${_bike_required_json},\"applied_warnings\":${_bike_warnings_json},\"omitted_warnings\":${_bike_policy_omitted_json},\"compile_options\":${_bike_options_json},\"compile_definitions\":${_bike_definitions_json},\"link_options\":${_bike_link_json}}")
file(WRITE "${CMAKE_BINARY_DIR}/bike_policy_report.json"
  "${BIKE_POLICY_REPORT_JSON}\n")

list(LENGTH _bike_applied_warnings _bike_applied_warnings_count)
list(LENGTH BIKE_POLICY_OMITTED_WARNINGS _bike_omitted_warnings_count)
message(STATUS
  "native compiler policy: ${BIKE_COMPILER_FAMILY}/${CMAKE_CXX_COMPILER_ID} "
  "on ${BIKE_POLICY_SYSTEM}/${BIKE_POLICY_MACHINE}, stdlib ${BIKE_STDLIB_IDENTITY}, "
  "warnings applied=${_bike_applied_warnings_count} "
  "omitted=${_bike_omitted_warnings_count}")
