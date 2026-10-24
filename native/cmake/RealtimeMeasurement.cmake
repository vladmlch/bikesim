include_guard(GLOBAL)
function(bike_add_realtime_measurement native_dir)
  if(TARGET bike-measure-realtime)
    return()
  endif()
  add_library(bike_realtime_measurement STATIC
    "${native_dir}/src/runtime/realtime_measurement.cpp"
    "${native_dir}/src/runtime/realtime_report.cpp"
    "${native_dir}/src/runtime/realtime_options.cpp"
    "${native_dir}/src/runtime/realtime_driver.cpp"
    "${native_dir}/src/runtime/realtime_provenance.cpp"
    "${native_dir}/src/runtime/realtime_command.cpp")
  bike_startup_target_policy(bike_realtime_measurement "${native_dir}")
  target_link_libraries(bike_realtime_measurement PUBLIC bike_cli bike_spatial bike_pneumatic3d)
  foreach(_lane IN ITEMS address realtime coverage)
    set(_measurement_${_lane} false)
  endforeach()
  if(NATIVE_SANITIZE)
    set(_measurement_address true)
  endif()
  if(NATIVE_RTSAN)
    set(_measurement_realtime true)
  endif()
  if(NATIVE_COVERAGE)
    set(_measurement_coverage true)
  endif()
  # Paths stay out of quoted COMPILE_DEFINITIONS: those are serialized into
  # the shared native check context. One private generated header owns them.
  get_filename_component(_measurement_source "${native_dir}/.." ABSOLUTE)
  foreach(_path IN ITEMS "${_measurement_source}" "${CMAKE_BINARY_DIR}")
    if(_path MATCHES "[\"\\\\\n\r]")
      message(FATAL_ERROR "Measurement build paths cannot contain quotes, backslashes or newlines")
    endif()
  endforeach()
  file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/measurement/bike_measurement_paths.hpp"
    CONTENT "// Generated; no runtime qualification is implied.\n#pragma once\nnamespace runtime::measurement::build {\ninline constexpr char source_root[] = \"${_measurement_source}\";\ninline constexpr char context_directory[] = \"${CMAKE_BINARY_DIR}\";\ninline constexpr bool address_sanitizer = ${_measurement_address};\ninline constexpr bool realtime_sanitizer = ${_measurement_realtime};\ninline constexpr bool coverage = ${_measurement_coverage};\n}\n")
  target_include_directories(bike_realtime_measurement PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/measurement")
  add_executable(bike-measure-realtime "${native_dir}/src/tools/measure_realtime_main.cpp")
  bike_startup_target_policy(bike-measure-realtime "${native_dir}")
  target_link_libraries(bike-measure-realtime PRIVATE bike_realtime_measurement)
  set_target_properties(bike-measure-realtime PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  bike_rewrite_mujoco_load_name(bike-measure-realtime "${BIKE_DEP_mujoco_LIBRARY}")
  install(TARGETS bike-measure-realtime RUNTIME DESTINATION bin OPTIONAL)
  bike_install_relocate_mujoco(bike-measure-realtime)
endfunction()
