# T16–T18/T20–T25: one source registration shared by the standalone and
# transition graphs. This module only declares targets; it runs no generator,
# interpreter, solver, or application during configure.
include_guard(GLOBAL)

function(bike_startup_target_policy target native_dir)
  set_target_properties(${target} PROPERTIES
    POSITION_INDEPENDENT_CODE ON CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
  if(BIKE_ROOT_BUILD)
    bike_target_strict_policy(${target})
    target_include_directories(${target} SYSTEM PRIVATE
      "${native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include"
      "${BIKE_DEP_eigen_ROOT}" "${BIKE_DEP_rapidjson_ROOT}/include"
      "${BIKE_DEP_tomlplusplus_ROOT}/include" "${BIKE_DEP_duckdb_ROOT}/include")
  else()
    if(_native_contract_compile_options)
      target_compile_options(${target} PRIVATE ${_native_contract_compile_options})
    endif()
    if(_native_contract_compile_definitions)
      target_compile_definitions(${target} PRIVATE ${_native_contract_compile_definitions})
    endif()
    if(_native_contract_link_options)
      target_link_options(${target} PRIVATE ${_native_contract_link_options})
    endif()
    target_include_directories(${target} SYSTEM PRIVATE "${native_dir}/src" "${MJ_DIR}/include")
    if(_native_tomlplusplus_include)
      target_include_directories(${target} SYSTEM PRIVATE "${_native_tomlplusplus_include}")
    endif()
    if(_native_contract_include_directories)
      target_include_directories(${target} SYSTEM PRIVATE ${_native_contract_include_directories})
    endif()
  endif()
endfunction()

function(bike_add_native_startup native_dir)
  if(TARGET bike_app)
    return()
  endif()
  if(BIKE_ROOT_BUILD)
    set(_startup_core bike_core)
    set(_startup_engine bike_engine)
    set(_startup_mujoco "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
  else()
    set(_startup_core bike_native_contract_core)
    set(_startup_engine bike_native_engine)
    set(_startup_mujoco "${MJ_DIR}/libmujoco.3.12.0.dylib")
  endif()

  # Object-library objects do not propagate through an ordinary transitive
  # link edge. Carry the already compiled objects explicitly, exactly once,
  # so every app consumer gets the existing runtime/writers without a fork.
  add_library(bike_startup_runtime INTERFACE)
  target_link_libraries(bike_startup_runtime INTERFACE
    ${_startup_core} "$<TARGET_OBJECTS:${_startup_core}>"
    ${_startup_engine} bike_rider_math "-framework Accelerate")

  add_library(bike_terrain STATIC
    "${native_dir}/src/terrain/profile.cpp"
    "${native_dir}/src/terrain/heightfield.cpp"
    "${native_dir}/src/terrain/surface.cpp"
    "${native_dir}/src/terrain/obstacles.cpp"
    "${native_dir}/src/terrain/contact_safety.cpp"
    "${native_dir}/src/terrain/wheelpath.cpp"
    "${native_dir}/src/terrain/placement.cpp"
    "${native_dir}/src/terrain/road.cpp"
    "${native_dir}/src/terrain/generator.cpp"
    "${native_dir}/src/terrain/alpine.cpp"
    "${native_dir}/src/terrain/presets.cpp"
    "${native_dir}/src/terrain/trackfile.cpp")
  bike_startup_target_policy(bike_terrain "${native_dir}")
  target_link_libraries(bike_terrain PUBLIC bike_configuration bike_geometry bike_random)

  target_sources(bike_model PRIVATE
    "${native_dir}/src/model/builder.cpp"
    "${native_dir}/src/model/xml_writer.cpp"
    "${native_dir}/src/model/physical_topology.cpp"
    "${native_dir}/src/model/articulated_rider.cpp"
    "${native_dir}/src/model/drivetrain.cpp"
    "${native_dir}/src/model/linkage.cpp"
    "${native_dir}/src/model/terrain.cpp"
    "${native_dir}/src/model/legacy_topology.cpp"
    "${native_dir}/src/model/export.cpp"
    "${native_dir}/src/model/assets.cpp"
    "${native_dir}/src/model/environment.cpp"
    "${native_dir}/src/model/sensors.cpp"
    "${native_dir}/src/model/mapping.cpp")
  target_link_libraries(bike_model PUBLIC bike_terrain ${_startup_engine} bike_io)

  # Filesystem-backed arrays depend on platform; keep them above the small
  # JSON/hash archive so platform -> io never becomes a circular edge.
  add_library(bike_numpy_io STATIC
    "${native_dir}/src/io/npy.cpp" "${native_dir}/src/io/npz.cpp")
  bike_startup_target_policy(bike_numpy_io "${native_dir}")
  target_link_libraries(bike_numpy_io PUBLIC bike_io bike_platform)

  add_library(bike_initialization STATIC
    "${native_dir}/src/numerics/least_squares.cpp"
    "${native_dir}/src/runtime/project_config.cpp"
    "${native_dir}/src/initialization/request.cpp"
    "${native_dir}/src/initialization/setup_owner.cpp"
    "${native_dir}/src/initialization/capture.cpp"
    "${native_dir}/src/initialization/initial_pose.cpp"
    "${native_dir}/src/initialization/equilibrium.cpp"
    "${native_dir}/src/initialization/legacy_equilibrium.cpp"
    "${native_dir}/src/initialization/settle_contacts.cpp"
    "${native_dir}/src/initialization/refine.cpp"
    "${native_dir}/src/initialization/finalize.cpp"
    "${native_dir}/src/initialization/sag_fit.cpp"
    "${native_dir}/src/initialization/sag_evaluator.cpp"
    "${native_dir}/src/initialization/state_file.cpp"
    "${native_dir}/src/initialization/artifact_identity.cpp"
    "${native_dir}/src/initialization/equilibrium_cache.cpp"
    "${native_dir}/src/initialization/initial_state.cpp")
  bike_startup_target_policy(bike_initialization "${native_dir}")
  target_link_libraries(bike_initialization PUBLIC
    bike_model bike_terrain bike_configuration bike_platform bike_io bike_numpy_io bike_startup_runtime)

  add_library(bike_app STATIC
    "${native_dir}/src/app/prepare.cpp" "${native_dir}/src/app/physical_session.cpp")
  bike_startup_target_policy(bike_app "${native_dir}")
  target_link_libraries(bike_app PUBLIC bike_initialization)

  add_executable(bike-native-ride "${native_dir}/src/app/native_ride_main.cpp")
  bike_startup_target_policy(bike-native-ride "${native_dir}")
  target_link_libraries(bike-native-ride PRIVATE bike_app)
  bike_rewrite_mujoco_load_name(bike-native-ride "${_startup_mujoco}")
  install(TARGETS bike-native-ride RUNTIME DESTINATION bin)
  install(FILES "${native_dir}/src/numerics/SCIPY-LICENSE.txt"
    DESTINATION share/licenses/bike-native-ride)
  # Keep shipped TOML and relative rider assets together after relocation.
  # Missing optional legacy NPZ inputs remain explicit acquisition failures;
  # --fresh deliberately requests a new native equilibrium instead.
  install(DIRECTORY "${native_dir}/../examples/"
    DESTINATION share/bike_sim/examples
    FILES_MATCHING PATTERN "*.toml" PATTERN "*.json" PATTERN "*.npz")
endfunction()
