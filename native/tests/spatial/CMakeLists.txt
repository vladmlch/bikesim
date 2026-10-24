# Native-only spatial contract tests. Fixtures are data, never configure-time
# generated simulation output. Pending identities remain test failures in V01.
set(BIKE_SPATIAL_SOURCE_ROOT "${CMAKE_SOURCE_DIR}")
set(BIKE_SPATIAL_FIXTURE_ROOT "${CMAKE_SOURCE_DIR}/tests/fixtures/spatial3d")
foreach(_spatial_path IN ITEMS "${BIKE_SPATIAL_SOURCE_ROOT}" "${BIKE_SPATIAL_FIXTURE_ROOT}")
  string(FIND "${_spatial_path}" "\"" _spatial_quote)
  string(FIND "${_spatial_path}" "\\" _spatial_backslash)
  string(FIND "${_spatial_path}" "\n" _spatial_newline)
  string(FIND "${_spatial_path}" "\r" _spatial_carriage_return)
  if(NOT _spatial_quote EQUAL -1 OR NOT _spatial_backslash EQUAL -1
      OR NOT _spatial_newline EQUAL -1 OR NOT _spatial_carriage_return EQUAL -1)
    message(FATAL_ERROR "Spatial source/fixture path cannot be embedded as a C++ string")
  endif()
endforeach()
configure_file(fixture_paths.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/fixture_paths.hpp" @ONLY)

add_library(bike_spatial_support STATIC
  support/spatial_fixture.cpp support/terrain_fixture.cpp support/startup_fixture.cpp
  support/tyre_fixture.cpp)
bike_apply_native_policy(bike_spatial_support)
target_include_directories(bike_spatial_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}/native/tests" "${CMAKE_SOURCE_DIR}/native/src")
target_include_directories(bike_spatial_support SYSTEM PRIVATE "${BIKE_DEP_mujoco_ROOT}/include")
target_link_libraries(bike_spatial_support PUBLIC
  bike_io bike_terrain bike_spatial bike_model bike_engine bike_pneumatic3d)

function(bike_add_spatial_test slug)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES;LABELS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "bike_add_spatial_test(${slug}): unrecognized arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_SOURCES)
    message(FATAL_ERROR "bike_add_spatial_test(${slug}) requires nonempty SOURCES")
  endif()
  add_executable(bike_spatial_${slug} ${ARG_SOURCES})
  target_link_libraries(bike_spatial_${slug} PRIVATE
    bike_spatial_support bike_migration_test bike_io ${ARG_LIBRARIES})
  target_include_directories(bike_spatial_${slug} SYSTEM PRIVATE
    "${BIKE_DEP_mujoco_ROOT}/include" "${BIKE_DEP_eigen_ROOT}")
  bike_apply_native_policy(bike_spatial_${slug})
  add_test(NAME spatial.${slug} COMMAND bike_spatial_${slug})
  set_tests_properties(spatial.${slug} PROPERTIES LABELS "spatial;${ARG_LABELS}")
endfunction()

bike_add_spatial_test(harness SOURCES test_harness.cpp LIBRARIES bike_platform LABELS foundation unit)
bike_add_spatial_test(baseline SOURCES test_baseline.cpp LABELS foundation audit)
bike_add_spatial_test(references SOURCES test_references.cpp LABELS foundation unit)
bike_add_spatial_test(contracts_registry SOURCES test_contracts_registry.cpp LABELS foundation unit)
bike_add_spatial_test(config SOURCES test_config.cpp
  LIBRARIES bike_configuration bike_initialization LABELS foundation unit)
# project_config.cpp belongs to bike_initialization, whose startup-runtime
# interface supplies the existing core objects and engine exactly once.
bike_rewrite_mujoco_load_name(bike_spatial_config "${BIKE_DEP_mujoco_LIBRARY}")

# W06-W12: asset/query/generation and startup/render separation. The extrusion
# comparator calls the actual legacy pneumatic ray implementation in the core.
bike_add_spatial_test(terrain_codec SOURCES terrain_codec.cpp LIBRARIES bike_terrain LABELS terrain unit)
bike_add_spatial_test(terrain_sampling SOURCES terrain_sampling.cpp LIBRARIES bike_terrain LABELS terrain unit)
bike_add_spatial_test(terrain_raycast SOURCES terrain_raycast.cpp LIBRARIES bike_terrain LABELS terrain unit)
bike_add_spatial_test(terrain_generator SOURCES terrain_generator.cpp LIBRARIES bike_terrain LABELS terrain unit)
bike_add_spatial_test(terrain_extrusion SOURCES terrain_extrusion.cpp
  LIBRARIES bike_terrain bike_startup_runtime LABELS terrain regression)
bike_add_spatial_test(terrain_startup SOURCES terrain_startup.cpp
  LIBRARIES bike_terrain bike_model bike_initialization LABELS terrain startup)
bike_add_spatial_test(terrain_acceptance SOURCES terrain_acceptance.cpp LIBRARIES bike_terrain LABELS terrain acceptance)
bike_add_spatial_test(terrain_budget_300m SOURCES terrain_budget_300m.cpp LIBRARIES bike_terrain LABELS terrain slow)

# W13-W19: use the repository's existing startup-runtime object interface.
bike_add_spatial_test(topology SOURCES test_topology.cpp LIBRARIES bike_spatial LABELS topology unit)
bike_add_spatial_test(frames SOURCES test_frames.cpp LIBRARIES bike_spatial bike_model LABELS topology unit)
bike_add_spatial_test(model SOURCES test_model.cpp LIBRARIES bike_model LABELS topology model)
bike_add_spatial_test(force_mapping SOURCES test_force_mapping.cpp
  LIBRARIES bike_spatial bike_startup_runtime LABELS topology force)
bike_add_spatial_test(drive SOURCES test_drive.cpp
  LIBRARIES bike_spatial bike_golden_test bike_initialization LABELS topology drive)
bike_add_spatial_test(rider SOURCES test_rider.cpp
  LIBRARIES bike_rider_spatial bike_initialization LABELS topology rider)
bike_add_spatial_test(grip SOURCES test_grip.cpp
  LIBRARIES bike_rider_spatial bike_initialization LABELS topology rider slow)
bike_add_spatial_test(runtime_state SOURCES test_runtime_state.cpp
  LIBRARIES bike_spatial bike_initialization LABELS topology lifecycle)
bike_add_spatial_test(planar_bridge SOURCES test_planar_bridge.cpp
  LIBRARIES bike_spatial bike_pneumatic3d bike_initialization bike_validation bike_platform LABELS bridge regression)
bike_add_spatial_test(planar_bridge_long SOURCES test_planar_bridge_long.cpp
  LIBRARIES bike_spatial bike_pneumatic3d bike_initialization bike_validation bike_platform LABELS bridge slow)
bike_add_spatial_test(topology_negatives SOURCES test_topology_negatives.cpp
  LIBRARIES bike_spatial bike_initialization bike_validation LABELS topology negative)

# W20-W36 source-authored gates. They are registered for local V04/V06;
# registration does not assert a passing qualification or execute a test.
bike_add_spatial_test(tyre_geometry SOURCES tyre_geometry.cpp LIBRARIES bike_pneumatic3d LABELS tyre unit)
bike_add_spatial_test(tyre_carcass SOURCES tyre_carcass.cpp LIBRARIES bike_pneumatic3d LABELS tyre unit)
bike_add_spatial_test(tyre_patches SOURCES tyre_patches.cpp LIBRARIES bike_pneumatic3d LABELS tyre unit)
bike_add_spatial_test(tyre_brush_fast SOURCES tyre_brush_fast.cpp LIBRARIES bike_pneumatic3d LABELS tyre unit)
bike_add_spatial_test(tyre_brush_detailed SOURCES tyre_brush_detailed.cpp LIBRARIES bike_pneumatic3d LABELS tyre unit)
bike_add_spatial_test(tyre_writer SOURCES tyre_writer.cpp $<TARGET_OBJECTS:bike_test_allocation_faults>
  LIBRARIES bike_pneumatic3d bike_startup_runtime LABELS tyre force)
bike_add_spatial_test(brake_static_3d SOURCES brake_static_3d.cpp LIBRARIES bike_startup_runtime LABELS brake lifecycle)
bike_add_spatial_test(tyre_state SOURCES tyre_state.cpp LIBRARIES bike_pneumatic3d LABELS tyre state)
bike_add_spatial_test(tyre_qualification SOURCES tyre_qualification.cpp LIBRARIES bike_pneumatic3d bike_configuration LABELS tyre qualification slow)
bike_add_spatial_test(equilibrium SOURCES equilibrium.cpp LIBRARIES bike_initialization LABELS startup slow)
bike_add_spatial_test(bootstrap_hold SOURCES bootstrap_hold.cpp LIBRARIES bike_initialization bike_validation LABELS startup lifecycle)
bike_add_spatial_test(startup_paths SOURCES startup_paths.cpp LIBRARIES bike_initialization bike_validation LABELS startup lifecycle slow)
bike_add_spatial_test(steer_controller SOURCES steer_controller.cpp LIBRARIES bike_rider_spatial bike_initialization LABELS balance unit)
bike_add_spatial_test(outcomes SOURCES outcomes.cpp LIBRARIES bike_spatial bike_initialization LABELS balance outcome)
add_library(bike_spatial_benchmark_support STATIC
  support/meijaard_reference.cpp support/meijaard_constraints.cpp support/benchmark_linalg.cpp)
bike_apply_native_policy(bike_spatial_benchmark_support)
target_include_directories(bike_spatial_benchmark_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/native/tests" "${CMAKE_SOURCE_DIR}/native/src")
target_include_directories(bike_spatial_benchmark_support SYSTEM PRIVATE
  "${BIKE_DEP_mujoco_ROOT}/include" "${BIKE_DEP_eigen_ROOT}")
target_link_libraries(bike_spatial_benchmark_support PUBLIC bike_model bike_engine Bike::Numerics)
bike_add_spatial_test(meijaard SOURCES meijaard.cpp
  LIBRARIES bike_spatial_benchmark_support LABELS balance benchmark slow)
bike_add_spatial_test(cornering SOURCES cornering.cpp
  LIBRARIES bike_spatial_benchmark_support bike_initialization bike_validation LABELS balance ride slow)
bike_add_spatial_test(mirror SOURCES mirror.cpp
  LIBRARIES bike_initialization bike_validation LABELS balance ride slow)
bike_add_spatial_test(timestep SOURCES timestep.cpp
  LIBRARIES bike_initialization bike_validation LABELS balance ride slow)
bike_add_spatial_test(target_rides SOURCES target_rides.cpp
  LIBRARIES bike_initialization bike_validation LABELS balance ride slow)

foreach(_spatial_test IN ITEMS terrain_codec terrain_sampling terrain_raycast terrain_generator
    terrain_extrusion terrain_startup terrain_acceptance terrain_budget_300m topology frames model
    force_mapping drive rider grip runtime_state planar_bridge planar_bridge_long topology_negatives
    tyre_geometry tyre_carcass tyre_patches tyre_brush_fast tyre_brush_detailed tyre_writer
    brake_static_3d tyre_state tyre_qualification equilibrium bootstrap_hold startup_paths steer_controller outcomes meijaard
    cornering mirror timestep target_rides)
  bike_rewrite_mujoco_load_name(bike_spatial_${_spatial_test} "${BIKE_DEP_mujoco_LIBRARY}")
endforeach()
