# T26–T37 source registration. This file declares build relationships only;
# it never executes a compiler, solver, simulator, policy or Python process.
include_guard(GLOBAL)

function(bike_add_physics_research native_dir core_target)
  get_target_property(_registered ${core_target} BIKE_PHYSICS_RESEARCH_REGISTERED)
  if(_registered)
    return()
  endif()
  set_target_properties(${core_target} PROPERTIES BIKE_PHYSICS_RESEARCH_REGISTERED TRUE)
  target_sources(${core_target} PRIVATE
    "${native_dir}/src/contact/tabulated_radial.cpp"
    "${native_dir}/src/runtime/native_contacts.cpp"
    "${native_dir}/src/numerics/slsqp.cpp"
    "${native_dir}/src/numerics/linear_feasibility.cpp"
    "${native_dir}/src/numerics/nnls.cpp"
    "${native_dir}/src/numerics/brent.cpp"
    "${native_dir}/src/rider/allocation.cpp"
    "${native_dir}/src/rider/constraint_dynamics.cpp"
    "${native_dir}/src/rider/mode_search.cpp"
    "${native_dir}/src/rider/activation_allocation.cpp"
    "${native_dir}/src/rider/response_allocation.cpp"
    "${native_dir}/src/rider/response_codec.cpp"
    "${native_dir}/src/rider/legacy_allocation.cpp"
    "${native_dir}/src/rider/legacy_controller.cpp"
    "${native_dir}/src/rider/support_dispatch.cpp"
    "${native_dir}/src/rider/flat_contacts.cpp"
    "${native_dir}/src/rider/weld_settle.cpp"
    "${native_dir}/src/rider/grip_springs.cpp")
  # The owned controller decoder reconstructs ArticulatedConfig without
  # importing its historical Python dataclass.
  target_link_libraries(${core_target} PUBLIC bike_configuration)

  target_sources(bike_initialization PRIVATE
    "${native_dir}/src/runtime/capabilities.cpp"
    "${native_dir}/src/initialization/research_startup.cpp")
  target_sources(bike_app PRIVATE
    "${native_dir}/src/research/allocation_json.cpp"
    "${native_dir}/src/numerics/density_fit.cpp"
    "${native_dir}/src/research/environment.cpp"
    "${native_dir}/src/research/configuration.cpp"
    "${native_dir}/src/research/program_loader.cpp"
    "${native_dir}/src/research/rider_random.cpp"
    "${native_dir}/src/research/policy.cpp"
    "${native_dir}/src/research/builtin_policies.cpp"
    "${native_dir}/src/research/plugin_policy.cpp"
    "${native_dir}/src/research/policy_session.cpp"
    "${native_dir}/src/research/rider_behavior.cpp"
    "${native_dir}/src/research/lifecycle.cpp"
    "${native_dir}/src/research/exit_codes.cpp"
    "${native_dir}/src/research/metrics.cpp"
    "${native_dir}/src/research/quality.cpp"
    "${native_dir}/src/research/evidence.cpp"
    "${native_dir}/src/research/metadata.cpp"
    "${native_dir}/src/research/preview_csv.cpp")
  target_link_libraries(bike_app PUBLIC bike_random ${CMAKE_DL_LIBS})

  option(BIKE_BUILD_POLICY_EXAMPLES "Build the native policy ABI example" OFF)
  if(BIKE_BUILD_POLICY_EXAMPLES)
    add_library(bike_traction_policy MODULE "${native_dir}/../examples/policies/traction_control.cpp")
    bike_startup_target_policy(bike_traction_policy "${native_dir}")
    target_include_directories(bike_traction_policy PRIVATE "${native_dir}/include")
    set_target_properties(bike_traction_policy PROPERTIES PREFIX "")
    install(TARGETS bike_traction_policy LIBRARY DESTINATION lib/bike_sim/policies)
  endif()
  install(FILES "${native_dir}/include/bike/policy_v1.h" DESTINATION include/bike)
endfunction()
