# Preset compiler selection is evaluated before project(), including its
# compiler probes. A user-supplied compiler remains an explicit override.
if(NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER STREQUAL "")
  if(BIKE_PRESET_GNU)
    set(CMAKE_CXX_COMPILER "/usr/bin/g++" CACHE FILEPATH "Linux Debug compiler")
  elseif(BIKE_PRESET_RTSAN)
    # The capability probe, not the executable's name, decides support.
    find_program(_bike_preset_rtsan NAMES clang++-23 clang++-22 clang++-21
      clang++-20 clang++-19 clang++-18 NO_CACHE REQUIRED)
    set(CMAKE_CXX_COMPILER "${_bike_preset_rtsan}" CACHE FILEPATH "RTSan compiler")
  else()
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++-18" CACHE FILEPATH "Qualified Linux compiler")
  endif()
endif()
# Propagate lane selectors into nested try_compile projects without changing
# the already-selected compiler in those projects.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES BIKE_PRESET_GNU BIKE_PRESET_RTSAN)
