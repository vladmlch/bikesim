# T38–T51: declarations only. No generation, simulation, download or test
# runs at configure time. The analyst has no runtime/model/viewer link edge.
include_guard(GLOBAL)

function(bike_add_recording_application native_dir core_target mujoco_library)
  if(TARGET bike_recording_reader)
    return()
  endif()
  find_package(Threads REQUIRED)
  bike_dependency_lock_read(_recording_lock)
  target_sources(${core_target} PRIVATE "${native_dir}/src/runtime/async_close.cpp")
  target_link_libraries(${core_target} PUBLIC Threads::Threads)

  if(NOT TARGET bike::duckdb)
    # Transition builds use the same pinned C SDK. Never borrow _duckdb.so
    # from a Python environment or resolve an unpinned system library.
    bike_dependency_lock_select("${_recording_lock}" duckdb _duck_entry _duck_required)
    if(NOT _duck_required)
      message(FATAL_ERROR "Recording requires a DuckDB SDK for the selected platform")
    endif()
    bike_dependency_lock_verify("${_recording_lock}" duckdb BIKE_DEP_duckdb_ROOT)
    _bike_dep_locked_library("${_recording_lock}" duckdb _duck_library_member)
    set(BIKE_DEP_duckdb_LIBRARY "${BIKE_DEP_duckdb_ROOT}/${_duck_library_member}")
    add_library(bike::duckdb SHARED IMPORTED GLOBAL)
    set_target_properties(bike::duckdb PROPERTIES
      IMPORTED_LOCATION "${BIKE_DEP_duckdb_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_duckdb_ROOT}/include")
  endif()

  add_library(bike_recording_reader STATIC
    "${native_dir}/src/recording/catalog.cpp"
    "${native_dir}/src/recording/coordinates.cpp"
    "${native_dir}/src/recording/duckdb.cpp"
    "${native_dir}/src/recording/schema.cpp"
    "${native_dir}/src/recording/encoding.cpp"
    "${native_dir}/src/recording/database_validate.cpp"
    "${native_dir}/src/recording/reader.cpp"
    "${native_dir}/src/recording/analysis.cpp"
    "${native_dir}/src/recording/query.cpp"
    "${native_dir}/src/recording/report.cpp"
    "${native_dir}/src/recording/export.cpp")
  bike_startup_target_policy(bike_recording_reader "${native_dir}")
  # bike_numpy_io supplies the bounded no-follow byte reader used for manifest,
  # asset and ledger payloads. It links bike_io/bike_platform only — no engine,
  # runtime, model, policy or display edge enters the analyst chain.
  target_link_libraries(bike_recording_reader PUBLIC
    bike_io bike_platform bike_numpy_io bike::duckdb)

  add_library(bike_recording STATIC
    "${native_dir}/src/recording/catalog_model.cpp"
    "${native_dir}/src/recording/assets.cpp"
    "${native_dir}/src/recording/capture.cpp"
    "${native_dir}/src/recording/events.cpp"
    "${native_dir}/src/recording/plan.cpp"
    "${native_dir}/src/recording/physical_source.cpp"
    "${native_dir}/src/recording/research_source.cpp"
    "${native_dir}/src/recording/duckdb_chunks.cpp"
    "${native_dir}/src/recording/database.cpp"
    "${native_dir}/src/recording/publish.cpp"
    "${native_dir}/src/recording/finalize.cpp"
    "${native_dir}/src/recording/interrupt.cpp"
    "${native_dir}/src/recording/research_publication.cpp")
  bike_startup_target_policy(bike_recording "${native_dir}")
  target_link_libraries(bike_recording PUBLIC bike_recording_reader bike_app)

  add_library(bike_replay STATIC
    "${native_dir}/src/replay/spatial_recipe.cpp"
    "${native_dir}/src/replay/input.cpp"
    "${native_dir}/src/replay/legacy.cpp"
    "${native_dir}/src/replay/rebuild.cpp"
    "${native_dir}/src/replay/session.cpp"
    "${native_dir}/src/replay/playback.cpp"
    "${native_dir}/src/replay/export.cpp")
  bike_startup_target_policy(bike_replay "${native_dir}")
  target_link_libraries(bike_replay PUBLIC bike_recording_reader bike_app bike_numpy_io)

  add_library(bike_viewer STATIC
    "${native_dir}/src/viewer/input.cpp"
    "${native_dir}/src/viewer/pacer.cpp"
    "${native_dir}/src/viewer/camera.cpp"
    "${native_dir}/src/viewer/render_owner.cpp"
    "${native_dir}/src/viewer/snapshot.cpp"
    "${native_dir}/src/viewer/glfw_context.cpp"
    "${native_dir}/src/viewer/hud.cpp"
    "${native_dir}/src/viewer/ride.cpp"
    "${native_dir}/src/viewer/research.cpp"
    "${native_dir}/src/viewer/replay.cpp")
  bike_startup_target_policy(bike_viewer "${native_dir}")
  target_link_libraries(bike_viewer PUBLIC bike_app bike_playground Threads::Threads)
  option(BIKE_BUILD_GUI "Enable the native GLFW viewer" OFF)
  if(BIKE_BUILD_GUI)
    bike_dependency_lock_verify("${_recording_lock}" glfw BIKE_DEP_glfw_ROOT)
    if(NOT EXISTS "${BIKE_DEP_glfw_ROOT}/CMakeLists.txt" OR
       NOT EXISTS "${BIKE_DEP_glfw_ROOT}/src/CMakeLists.txt")
      message(FATAL_ERROR "GUI requires the complete acquired GLFW 3.4 source tree: ${BIKE_DEP_glfw_ROOT}")
    endif()
    enable_language(C)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    if(BIKE_TARGET_SYSTEM STREQUAL "Linux")
      # X11 (including XWayland) is the qualified desktop transport.
      set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
      set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
    endif()
    add_subdirectory("${BIKE_DEP_glfw_ROOT}" "${CMAKE_BINARY_DIR}/third_party/glfw" EXCLUDE_FROM_ALL)
    target_compile_definitions(bike_viewer PRIVATE BIKE_BUILD_GUI=1)
    target_link_libraries(bike_viewer PRIVATE glfw)
  endif()

  add_library(bike_viz STATIC
    "${native_dir}/src/viz/spatial.cpp"
    "${native_dir}/src/viz/plot.cpp"
    "${native_dir}/src/viz/svg.cpp"
    "${native_dir}/src/viz/png.cpp"
    "${native_dir}/src/viz/html.cpp"
    "${native_dir}/src/viz/geometry.cpp"
    "${native_dir}/src/viz/ride.cpp"
    "${native_dir}/src/viz/tables.cpp")
  bike_startup_target_policy(bike_viz "${native_dir}")
  target_link_libraries(bike_viz PUBLIC bike_legacy bike_terrain bike_io bike_platform)
  # The source root reaches html.cpp as a generated header, never as a quoted
  # compile definition: the check-context writer embeds target properties
  # verbatim, and a quote inside one would make native_check_context.json invalid.
  file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/viz/bike_viz_paths.hpp"
    CONTENT "// Generated by native/cmake/RecordingApplication.cmake; do not edit.\n#pragma once\n#define BIKE_SOURCE_ROOT \"${native_dir}/..\"\n")
  target_include_directories(bike_viz PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/viz")
  # A native raster PNG backend is always available. A separately acquired,
  # checksum-pinned Cairo SDK optionally supplies its Unicode text renderer.
  set(BIKE_CAIRO_LIBRARY "" CACHE FILEPATH "Explicit Cairo 1.18.4 library (optional)")
  set(BIKE_CAIRO_INCLUDE_DIR "" CACHE PATH "Cairo SDK include directory")
  set(BIKE_CAIRO_LIBRARY_SHA256 "" CACHE STRING "Required SHA-256 for the optional Cairo library")
  if(BIKE_CAIRO_LIBRARY)
    if(NOT EXISTS "${BIKE_CAIRO_INCLUDE_DIR}/cairo.h" OR NOT EXISTS "${BIKE_CAIRO_LIBRARY}" OR NOT BIKE_CAIRO_LIBRARY_SHA256)
      message(FATAL_ERROR "Explicit Cairo requires headers, library and BIKE_CAIRO_LIBRARY_SHA256")
    endif()
    file(SHA256 "${BIKE_CAIRO_LIBRARY}" _cairo_actual)
    if(NOT _cairo_actual STREQUAL BIKE_CAIRO_LIBRARY_SHA256)
      message(FATAL_ERROR "Explicit Cairo library checksum differs")
    endif()
    string(JSON _cairo_header_expected GET "${_recording_lock}" dependencies cairo required_files "src/cairo.h")
    file(SHA256 "${BIKE_CAIRO_INCLUDE_DIR}/cairo.h" _cairo_header_actual)
    if(NOT _cairo_header_actual STREQUAL _cairo_header_expected)
      message(FATAL_ERROR "Explicit Cairo header differs from the locked 1.18.4 source")
    endif()
    target_include_directories(bike_viz SYSTEM PRIVATE "${BIKE_CAIRO_INCLUDE_DIR}")
    target_compile_definitions(bike_viz PRIVATE BIKE_HAS_CAIRO=1)
    target_link_libraries(bike_viz PRIVATE "${BIKE_CAIRO_LIBRARY}")
  endif()

  add_library(bike_cli_arguments STATIC "${native_dir}/src/cli/arguments.cpp")
  bike_startup_target_policy(bike_cli_arguments "${native_dir}")
  target_link_libraries(bike_cli_arguments PUBLIC bike_io bike_configuration)
  add_library(bike_cli_configuration STATIC "${native_dir}/src/cli/configuration.cpp")
  bike_startup_target_policy(bike_cli_configuration "${native_dir}")
  target_link_libraries(bike_cli_configuration PUBLIC bike_cli_arguments bike_app bike_configuration)
  install(TARGETS bike_cli_configuration ARCHIVE DESTINATION lib OPTIONAL)
  add_library(bike_batch STATIC
    "${native_dir}/src/batch/plan.cpp"
    "${native_dir}/src/batch/spec.cpp"
    "${native_dir}/src/batch/process_pool.cpp"
    "${native_dir}/src/batch/summary.cpp"
    "${native_dir}/src/batch/sweep.cpp")
  bike_startup_target_policy(bike_batch "${native_dir}")
  target_link_libraries(bike_batch PUBLIC bike_cli_configuration bike_terrain bike_platform Threads::Threads)

  add_library(bike_cli STATIC
    "${native_dir}/src/cli/application.cpp"
    "${native_dir}/src/cli/physical_capture.cpp"
    "${native_dir}/src/cli/ride.cpp"
    "${native_dir}/src/cli/research.cpp"
    "${native_dir}/src/cli/settle.cpp"
    "${native_dir}/src/cli/export.cpp"
    "${native_dir}/src/cli/simulation.cpp"
    "${native_dir}/src/cli/playground.cpp"
    "${native_dir}/src/cli/replay.cpp"
    "${native_dir}/src/cli/batch.cpp")
  bike_startup_target_policy(bike_cli "${native_dir}")
  target_link_libraries(bike_cli PUBLIC bike_cli_configuration bike_cli_arguments bike_recording bike_replay bike_viewer bike_viz bike_batch bike_spatial bike_pneumatic3d)
  # One compile-database entry for each main; do not compile the same TU eight
  # times and break the produced source/ODR checks.
  add_library(bike_cli_main OBJECT "${native_dir}/src/cli/main.cpp")
  bike_startup_target_policy(bike_cli_main "${native_dir}")
  foreach(_command IN ITEMS bike-sim bike-playground bike-export bike-ride bike-settle bike-research bike-replay bike-batch)
    add_executable(${_command} "$<TARGET_OBJECTS:bike_cli_main>")
    bike_startup_target_policy(${_command} "${native_dir}")
    target_link_libraries(${_command} PRIVATE bike_cli)
    if(BIKE_ROOT_BUILD)
      bike_rewrite_mujoco_load_name(${_command} "${mujoco_library}")
    else()
      bike_fix_mujoco_install_name(${_command})
    endif()
    set_target_properties(${_command} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
    # OPTIONAL: focused builds (e.g. the T03 bindings-OFF provenance build)
    # compile only a subset of targets; installing the produced subset must
    # not fail on executables that were never built.
    install(TARGETS ${_command} RUNTIME DESTINATION bin OPTIONAL)
    if(BIKE_ROOT_BUILD)
      bike_install_relocate_mujoco(${_command})
    endif()
  endforeach()
  # The analyst library remains engine-free. Golden simulation belongs to a
  # separate library used only by the explicit recording golden subcommand.
  add_library(bike_golden STATIC
    "${native_dir}/src/golden/spatial_registry.cpp"
    "${native_dir}/src/golden/episode.cpp"
    "${native_dir}/src/golden/capture.cpp")
  bike_startup_target_policy(bike_golden "${native_dir}")
  target_link_libraries(bike_golden PUBLIC bike_app bike_numpy_io bike_platform)
  add_library(bike_recording_cli STATIC
    "${native_dir}/src/cli/recording.cpp"
    "${native_dir}/src/cli/recording_golden.cpp")
  bike_startup_target_policy(bike_recording_cli "${native_dir}")
  target_link_libraries(bike_recording_cli PUBLIC bike_recording_reader bike_golden bike_viz)
  target_link_libraries(bike_cli PRIVATE bike_recording_cli)
  add_executable(bike-recording "${native_dir}/src/cli/recording_main.cpp")
  bike_startup_target_policy(bike-recording "${native_dir}")
  target_link_libraries(bike-recording PRIVATE bike_recording_cli)
  if(BIKE_ROOT_BUILD)
    bike_rewrite_mujoco_load_name(bike-recording "${mujoco_library}")
  else()
    bike_fix_mujoco_install_name(bike-recording)
  endif()
  set_target_properties(bike-recording PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  install(TARGETS bike-recording RUNTIME DESTINATION bin OPTIONAL)
  if(BIKE_ROOT_BUILD)
    bike_install_relocate_mujoco(bike-recording)
  endif()
  add_executable(bike-research-sweep "${native_dir}/src/tools/research_sweep_main.cpp")
  bike_startup_target_policy(bike-research-sweep "${native_dir}")
  target_link_libraries(bike-research-sweep PRIVATE bike_batch)
  set_target_properties(bike-research-sweep PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  if(BIKE_ROOT_BUILD)
    bike_rewrite_mujoco_load_name(bike-research-sweep "${mujoco_library}")
  else()
    bike_fix_mujoco_install_name(bike-research-sweep)
  endif()
  install(TARGETS bike-research-sweep RUNTIME DESTINATION bin OPTIONAL)
  if(BIKE_ROOT_BUILD)
    bike_install_relocate_mujoco(bike-research-sweep)
  endif()
  if(BUILD_TESTING)
    add_executable(bike-test-child "${native_dir}/src/tools/test_child_main.cpp")
    bike_startup_target_policy(bike-test-child "${native_dir}")
    target_link_libraries(bike-test-child PRIVATE bike_platform Threads::Threads)
    set_target_properties(bike-test-child PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  endif()
  install(FILES "${BIKE_DEP_duckdb_LIBRARY}" DESTINATION lib)
  install(FILES "${native_dir}/resources/recording/catalog-v1.json" DESTINATION share/bike_sim/recording)
  install(FILES "${native_dir}/resources/recording/catalog-v2.json" DESTINATION share/bike_sim/recording)
  install(FILES "${native_dir}/resources/reports/summary.md.in" DESTINATION share/bike_sim/reports)
endfunction()
