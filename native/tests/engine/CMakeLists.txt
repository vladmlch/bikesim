# engine domain contract tests.

# engine-boundary regression: links the shared pinned-interceptor
# library bike_engine so this graph exercises the engine_call.cpp TU. The
# src/MuJoCo include edges mirror the consuming targets' SYSTEM treatment.
add_executable(engine_boundaries test_boundaries.cpp)
set_target_properties(engine_boundaries PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(engine_boundaries PRIVATE bike_test_fixtures)
  bike_apply_native_policy(engine_boundaries)
  target_include_directories(engine_boundaries SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(engine_boundaries PRIVATE bike_engine)
  bike_rewrite_mujoco_load_name(engine_boundaries
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME engine_boundaries COMMAND engine_boundaries)
  set_tests_properties(engine_boundaries PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:engine_boundaries>")
endif()

add_executable(engine_contacts test_contacts.cpp)
bike_startup_target_policy(engine_contacts "${_test_repo_root}/native")
target_link_libraries(engine_contacts PRIVATE bike_test_fixtures bike_app)
  bike_rewrite_mujoco_load_name(engine_contacts
    "${BIKE_DEP_mujoco_LIBRARY}")
if(BUILD_TESTING)
  add_test(NAME engine_contacts COMMAND engine_contacts)
  set_tests_properties(engine_contacts PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:engine_contacts>")
endif()
