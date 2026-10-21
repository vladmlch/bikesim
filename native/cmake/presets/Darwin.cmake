# Ordinary presets use the selected Xcode toolchain. AppleClang lacks RTSan;
# the explicit RTSan preset uses the same Homebrew LLVM path as native/.
if(BIKE_PRESET_RTSAN AND (NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER STREQUAL ""))
  set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++" CACHE FILEPATH "RTSan compiler")
endif()
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES BIKE_PRESET_RTSAN)
