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
#   NATIVE_SANITIZE / NATIVE_RTSAN / NATIVE_COVERAGE
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
option(NATIVE_COVERAGE "llvm-cov instrumented build" OFF)

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

# --------------------------------------------------------------------------
# JSON emit helpers (shared with the native_check_context.json writer)
# --------------------------------------------------------------------------

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
    # keep the GCC-compatible core and record the omissions.
    list(APPEND _bike_instrument_options
      -fsanitize=address,undefined
      -fno-sanitize-recover=all -fno-omit-frame-pointer
      -Wframe-larger-than=24576)
    list(APPEND _bike_instrument_link_options -fsanitize=address,undefined)
    _bike_record_omission("-fsanitize=local-bounds,float-cast-overflow"
      "no GCC equivalent; the ASan lane keeps the address,undefined core under GCC (T15-T17 finalize)")
    _bike_record_omission("-shared-libasan"
      "no GCC spelling on this target; the default shared libasan is linked")
  endif()
endif()
if(NATIVE_RTSAN)
  if(_bike_clang_family)
    list(APPEND _bike_instrument_options -fsanitize=realtime)
    list(APPEND _bike_instrument_link_options -fsanitize=realtime)
  else()
    message(FATAL_ERROR
      "NATIVE_RTSAN requires a Clang-family compiler (the brew LLVM clang); "
      "${CMAKE_CXX_COMPILER_ID} has no -fsanitize=realtime")
  endif()
endif()
if(NATIVE_COVERAGE)
  if(_bike_clang_family)
    list(APPEND _bike_instrument_options
      -fprofile-instr-generate -fcoverage-mapping)
    list(APPEND _bike_instrument_link_options -fprofile-instr-generate)
  else()
    # llvm-cov's source-profile spelling is clang-only; GCC emits the gcov
    # data format instead. T15-T17 own the coverage toolchain selection.
    list(APPEND _bike_instrument_options -fprofile-arcs -ftest-coverage)
    list(APPEND _bike_instrument_link_options -fprofile-arcs)
    _bike_record_omission("-fprofile-instr-generate -fcoverage-mapping"
      "llvm-cov instrumentation is clang-only; GCC gets the gcov-format equivalent")
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
endfunction()

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
_native_json_quote(_bike_stdlib_detail_json "${_bike_stdlib_hardening_detail}")
_native_json_quote(_bike_fortify_mode_json "${_bike_fortify_mode}")
_native_json_quote(_bike_fortify_detail_json "${_bike_fortify_detail}")
_native_json_quote(_bike_bp_reason_json "${_bike_branch_protection_reason}")

set(BIKE_POLICY_REPORT_JSON
  "{\"schema_version\":1,\"system\":${_bike_system_json},\"machine\":${_bike_machine_json},\"machine_raw\":${_bike_machine_raw_json},\"override\":${_bike_policy_override_json},\"compiler_family\":${_bike_family_json},\"compiler_id\":${_bike_compiler_id_json},\"cxx_standard\":23,\"cxx_extensions\":false,\"cxx23_library\":{\"supported\":${_bike_cxx23_json},\"checked\":[\"std::format\",\"std::span\",\"std::bit_cast\"]},\"stdlib\":{\"identity\":${_bike_stdlib_json},\"hardening\":${_bike_stdlib_detail_json},\"libcpp_hardening_supported\":${_bike_libcpp_supported_json}},\"glibc\":${_bike_glibc_json},\"fortify_source\":{\"level\":3,\"mode\":${_bike_fortify_mode_json},\"detail\":${_bike_fortify_detail_json}},\"branch_protection\":{\"applied\":${_bike_bp_applied_json},\"flag\":\"-mbranch-protection=standard\",\"reason\":${_bike_bp_reason_json}},\"required_flags\":${_bike_required_json},\"applied_warnings\":${_bike_warnings_json},\"omitted_warnings\":${_bike_policy_omitted_json},\"compile_options\":${_bike_options_json},\"compile_definitions\":${_bike_definitions_json},\"link_options\":${_bike_link_json}}")
file(WRITE "${CMAKE_BINARY_DIR}/bike_policy_report.json"
  "${BIKE_POLICY_REPORT_JSON}\n")

list(LENGTH _bike_applied_warnings _bike_applied_warnings_count)
list(LENGTH BIKE_POLICY_OMITTED_WARNINGS _bike_omitted_warnings_count)
message(STATUS
  "native compiler policy: ${BIKE_COMPILER_FAMILY}/${CMAKE_CXX_COMPILER_ID} "
  "on ${BIKE_POLICY_SYSTEM}/${BIKE_POLICY_MACHINE}, stdlib ${BIKE_STDLIB_IDENTITY}, "
  "warnings applied=${_bike_applied_warnings_count} "
  "omitted=${_bike_omitted_warnings_count}")
