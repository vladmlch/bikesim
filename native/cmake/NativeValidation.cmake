include_guard(GLOBAL)
function(bike_add_native_validation native_dir)
  add_library(bike_validation STATIC
    "${native_dir}/src/validation/acceptance.cpp"
    "${native_dir}/src/validation/evidence.cpp"
    "${native_dir}/src/validation/tire.cpp"
    "${native_dir}/src/validation/rigs.cpp"
    "${native_dir}/src/validation/drivetrain.cpp"
    "${native_dir}/src/validation/rider.cpp"
    "${native_dir}/src/validation/runtime_access.cpp"
    "${native_dir}/src/validation/scenarios.cpp"
    "${native_dir}/src/validation/experiments.cpp"
    "${native_dir}/src/validation/benchmark.cpp"
    "${native_dir}/src/validation/registry.cpp")
  bike_startup_target_policy(bike_validation "${native_dir}")
  target_include_directories(bike_validation PUBLIC "${native_dir}/src")
  # bike_cli brings the existing setup, research, recording and engine graph;
  # the validation stands use those same physical implementations.
  target_link_libraries(bike_validation PUBLIC bike_cli)
  add_executable(bike-validate "${native_dir}/src/tools/validate_main.cpp")
  bike_startup_target_policy(bike-validate "${native_dir}")
  target_link_libraries(bike-validate PRIVATE bike_validation)
  set_target_properties(bike-validate PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  bike_rewrite_mujoco_load_name(bike-validate "${BIKE_DEP_mujoco_LIBRARY}")
  install(TARGETS bike-validate RUNTIME DESTINATION bin OPTIONAL)
  bike_install_relocate_mujoco(bike-validate)
endfunction()
