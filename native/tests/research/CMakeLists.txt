# research domain contract tests.

# owned-research boundary regression: read_research_bootstrap /
# decode_* (research_io.cpp) and NativeResearchRuntime (research.cpp) —
# the shared core object set (bike_core)
# plus the pinned engine interceptor and bike_rider_math: the core's
# rider/state_codec.o spindle:: validator refs need the single-compile
# library at final link. No Python/nanobind include enters this target:
# compiling test_bootstrap.cpp against runtime/research.hpp
# + runtime/research_io.hpp is the include-closure contract itself.
add_executable(research_bootstrap test_bootstrap.cpp)
set_target_properties(research_bootstrap PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(research_bootstrap PRIVATE bike_test_fixtures)
bike_link_numerics(research_bootstrap)
  bike_apply_native_policy(research_bootstrap)
  target_include_directories(research_bootstrap SYSTEM PRIVATE
    "${_test_repo_root}/native/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(research_bootstrap PRIVATE bike_core bike_engine
    bike_rider_math)
  bike_rewrite_mujoco_load_name(research_bootstrap
    "${BIKE_DEP_mujoco_LIBRARY}")

if(BUILD_TESTING)
  add_test(NAME research_bootstrap COMMAND research_bootstrap)
  set_tests_properties(research_bootstrap PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:research_bootstrap>")
endif()

# The remaining research contracts exercise the same app/library graph;
# registration never invokes the executables.
foreach(_research_test IN ITEMS research_environment research_policy
    research_lifecycle research_status)
  string(REGEX REPLACE "^research_" "" _research_src "${_research_test}")
  add_executable(${_research_test} "test_${_research_src}.cpp")
  bike_startup_target_policy(${_research_test} "${_test_repo_root}/native")
  target_link_libraries(${_research_test} PRIVATE bike_test_fixtures bike_app)
    bike_rewrite_mujoco_load_name(${_research_test}
      "${BIKE_DEP_mujoco_LIBRARY}")
  if(BUILD_TESTING)
    if(_research_test STREQUAL "research_policy")
      add_test(NAME ${_research_test} COMMAND ${_research_test}
        "$<TARGET_FILE:research_policy_fixture>"
        "$<TARGET_FILE:research_policy_missing_symbol>")
    else()
      add_test(NAME ${_research_test} COMMAND ${_research_test})
    endif()
    set_tests_properties(${_research_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_research_test}>")
  endif()
endforeach()

# Shared-module failure controls for research_policy. Registration/build
# graph changes do not execute or load these deliberately invalid
# callbacks.
foreach(_policy_fixture IN ITEMS research_policy_fixture research_policy_missing_symbol)
  add_library(${_policy_fixture} MODULE policy_fixture.cpp)
  bike_startup_target_policy(${_policy_fixture} "${_test_repo_root}/native")
  target_include_directories(${_policy_fixture} PRIVATE "${_test_repo_root}/native/include")
  set_target_properties(${_policy_fixture} PROPERTIES PREFIX "")
endforeach()
target_compile_definitions(research_policy_missing_symbol PRIVATE BIKE_POLICY_FIXTURE_MISSING_SYMBOL=1)
add_dependencies(research_policy research_policy_fixture research_policy_missing_symbol)
