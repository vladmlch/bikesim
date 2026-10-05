cmake_minimum_required(VERSION 3.24)
# Pin the SDK the toolchain understands — the CLT ld cannot parse arm64e.x1
# in the 27.0 SDK's .tbd (hit during the microbench build). Must precede
# project(): the compiler check links against the default CLT SDK otherwise.
set(CMAKE_OSX_SYSROOT "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk")
project(bike_native CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Resolve the interpreter first so mujoco/nanobind paths below are queried from
# the SAME python we compile against (uv run exposes .venv via VIRTUAL_ENV).
find_package(Python 3.13 REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(COMMAND "${Python_EXECUTABLE}" -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MJ_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# NB_SUPPRESS_WARNINGS: nanobind/Python headers land as SYSTEM includes —
# third-party TU internals stay outside our warning contract.
# PROTECT_STACK: nanobind otherwise appends -fno-stack-protector in Release
# configs, which would silently nullify -fstack-protector-strong below.
nanobind_add_module(bike_native NB_SUPPRESS_WARNINGS PROTECT_STACK
  src/binding.cpp src/stepper.cpp src/writers/suspension.cpp
  src/writers/brake.cpp src/writers/cruise.cpp src/writers/resistance.cpp
  src/writers/rider_forces.cpp src/writers/tire.cpp src/tyre/profile.cpp
  src/drivetrain/pedaling.cpp src/drivetrain/shifting.cpp
  src/drivetrain/motor.cpp src/drivetrain/freehub.cpp src/drivetrain/policy_binding.cpp
  src/drivetrain/chain.cpp src/drivetrain/transmission.cpp src/drivetrain/drive_binding.cpp
  src/writers/drivetrain.cpp
  src/rider/contact_math.cpp src/rider/support_geometry.cpp src/rider/contact_binding.cpp)

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
# libc++ hardening: EXTENSIVE under Debug (still ABI-stable), FAST elsewhere.
# DEBUG mode is deliberately avoided — it changes libc++ layout, which would
# ODR-clash with mujoco/nanobind built without hardening.
target_compile_definitions(bike_native PRIVATE
  "_LIBCPP_HARDENING_MODE=$<IF:$<CONFIG:Debug>,_LIBCPP_HARDENING_MODE_EXTENSIVE,_LIBCPP_HARDENING_MODE_FAST>")
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
          "${CMAKE_SOURCE_DIR}/src" "${MJ_DIR}/include"
          "${Python_EXECUTABLE}"
  COMMENT "gcc -flto -Wodr sweep (cross-TU type conflicts)")

# shellcheck over tools/*.sh — same everything-is-an-error contract.
add_custom_target(check_scripts
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_scripts.sh
          "${CMAKE_SOURCE_DIR}/../tools"
  COMMENT "shellcheck sweep (tools/*.sh)")
