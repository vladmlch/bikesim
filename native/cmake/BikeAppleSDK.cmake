# Apple SDK discovery for the native build. Included from BikePlatform.cmake
# before project() — CMake performs its compiler link check during project(),
# so resolve a default SDK before that call. Never replace an SDK or compiler
# supplied by the caller.
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

# The compiler identification check above is not enough: compile a source that
# includes an SDK header with the selected compiler and sysroot. Apple-only;
# CMakeLists.txt invokes this once, after project(), on Darwin.
function(bike_verify_apple_sdk_compatibility)
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
endfunction()

# bike_verify_sdk_compat — the standalone root graph's spelling of the same
# post-project() probe. Kept as a thin alias so the shared-modules swap needs
# no call-site rename; there is never a second probe.
function(bike_verify_sdk_compat)
  bike_verify_apple_sdk_compatibility()
endfunction()
