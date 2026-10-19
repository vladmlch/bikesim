# Native-build host dispatch. This file is included before project(); CMake
# runs its compiler link check inside project(), so any SDK a compiler
# defaults to must be resolved here — that is the Apple-only concern handled
# by BikeAppleSDK.cmake. Linux needs no SDK and must never invoke Apple
# tooling (xcrun/xcode-select); never set CMAKE_OSX_SYSROOT on Linux and never
# substitute a fake SDK path for it.
if(DEFINED CMAKE_SYSTEM_NAME AND
   NOT CMAKE_SYSTEM_NAME STREQUAL CMAKE_HOST_SYSTEM_NAME)
  message(FATAL_ERROR "Cross-compilation is not supported by this native-build workflow")
endif()
set(BIKE_TARGET_SYSTEM "${CMAKE_HOST_SYSTEM_NAME}")
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  include("${CMAKE_CURRENT_LIST_DIR}/BikeAppleSDK.cmake")
elseif(BIKE_TARGET_SYSTEM STREQUAL "Linux")
  if(DEFINED CMAKE_OSX_SYSROOT AND NOT CMAKE_OSX_SYSROOT STREQUAL "")
    message(FATAL_ERROR "CMAKE_OSX_SYSROOT is a macOS option; use a Linux-native toolchain")
  endif()
  # Null SDK contract consumed by the context JSON emitted post-project().
  set(_native_sdk_path "")
  set(_native_sdk_requested "")
  set(_native_sdk_source "not-applicable")
else()
  message(FATAL_ERROR "Unsupported native-build host: ${BIKE_TARGET_SYSTEM}")
endif()
