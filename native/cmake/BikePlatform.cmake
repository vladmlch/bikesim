# Native-build host dispatch. This file is included before project(); CMake
# runs its compiler link check inside project(), so any SDK a compiler
# defaults to must be resolved here — that is the Apple-only concern handled
# by BikeAppleSDK.cmake. Linux needs no SDK and must never invoke Apple
# tooling (xcrun/xcode-select); never set CMAKE_OSX_SYSROOT on Linux and never
# substitute a fake SDK path for it.
include("${CMAKE_CURRENT_LIST_DIR}/BikeHostTarget.cmake")
bike_host_target_system(BIKE_TARGET_SYSTEM)
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
