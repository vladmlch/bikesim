# model domain contract tests.

# model-boundary regression: inertia tensors and the synthetic wheel
# mass profile, MJCF component-mass distribution + registry,
# BikeMassSpecs budgets, component/system CG and static loads,
# rider pose/segment mass solvers and the seated apparent-mass path,
# air-spring/coil/damper calculators and the sag tuning rollup — linked
# against the bike_model library (one compile per TU per graph) plus the
# shared core object set for the LeastSquaresWorkspace and the pinned
# engine interceptor for the compiled-model reads (a real mjModel is
# built through mj_loadXML in the test). No Python/nanobind include
# enters this target: compiling test_inertia.cpp against
# model/*.hpp is the include-closure contract itself.
add_executable(model_inertia test_inertia.cpp)
set_target_properties(model_inertia PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(model_inertia PRIVATE bike_test_fixtures bike_model)
bike_link_numerics(model_inertia)
  bike_apply_native_policy(model_inertia)
  target_include_directories(model_inertia SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(model_inertia PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(model_inertia
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME model_inertia COMMAND model_inertia)
  set_tests_properties(model_inertia PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:model_inertia>")
endif()

# Native startup: the same app/library graph is exercised by all three
# model-boundary executables. Registration never invokes the executable.
foreach(_model_test IN ITEMS model_mjcf model_export model_compat)
  string(REGEX REPLACE "^model_" "" _model_src "${_model_test}")
  add_executable(${_model_test} "test_${_model_src}.cpp")
  bike_startup_target_policy(${_model_test} "${_test_repo_root}/native")
  target_link_libraries(${_model_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_model_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    add_test(NAME ${_model_test} COMMAND ${_model_test})
    set_tests_properties(${_model_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_model_test}>")
  endif()
endforeach()
