# io domain contract tests.

# io-boundary regression: the strict lossless JSON/Value boundary,
# recording-tag codec and shared SHA-256 validation.
# Links bike_test_fixtures (which itself links bike_io) — identical
# implementation objects in both build graphs.
add_executable(io_json test_json.cpp)
set_target_properties(io_json PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(io_json PRIVATE bike_test_fixtures)
target_include_directories(io_json SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
  bike_apply_native_policy(io_json)

if(BUILD_TESTING)
  add_test(NAME io_json COMMAND io_json)
  set_tests_properties(io_json PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:io_json>")
endif()

# state-wire codec regression: the owned bike::io::Value codecs for
# the rider and drivetrain state documents (drivetrain/state_codec.cpp,
# rider/state_codec.cpp) — the shared core object set (bike_core)
# plus the pinned engine interceptor for the
# welded_pinned Stepper that supplies the contacts topology gate, plus
# bike_rider_math: the OBJECT core now carries rider/state_codec.o whose
# spindle:: validator refs need the single-compile library at final link. No
# Python/nanobind include enters this target: compiling test_state_codecs.cpp
# against the codec headers is the include-closure contract itself.
add_executable(io_state_codecs test_state_codecs.cpp)
set_target_properties(io_state_codecs PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(io_state_codecs PRIVATE bike_test_fixtures)
bike_link_numerics(io_state_codecs)
  bike_apply_native_policy(io_state_codecs)
  target_include_directories(io_state_codecs SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(io_state_codecs PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(io_state_codecs
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME io_state_codecs COMMAND io_state_codecs)
  set_tests_properties(io_state_codecs PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:io_state_codecs>")
endif()
