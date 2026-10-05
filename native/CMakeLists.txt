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
  src/drivetrain/motor.cpp src/drivetrain/freehub.cpp src/drivetrain/policy_binding.cpp)

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
  -ffp-contract=off
  -fstack-protector-strong -ftrivial-auto-var-init=zero -fvisibility=hidden
  -mbranch-protection=standard)
target_include_directories(bike_native SYSTEM PRIVATE "${MJ_DIR}/include")
target_compile_definitions(bike_native PRIVATE
  _LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST)
target_link_options(bike_native PRIVATE -headerpad_max_install_names)

option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
if(NATIVE_SANITIZE)
  target_compile_options(bike_native PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(bike_native PRIVATE
    -fsanitize=address,undefined -shared-libasan)
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
