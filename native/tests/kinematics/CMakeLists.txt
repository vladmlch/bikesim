# kinematics domain contract tests.

# geometry + Horst-link kinematics regression: the owned
# geometry/specs.hpp records, hardpoint/trail/frame-angle and cockpit
# placement calculators, the geometry validator, the linkage_math
# brentq/circle primitives, and the HorstLinkageSolver state/trajectory/
# inverse solves plus curves' instant centers — linked against the
# bike_geometry/bike_kinematics libraries (one compile per TU per
# graph), plus the shared fixture harness for the bitwise fixture
# comparisons.
add_executable(kinematics_horst_link test_horst_link.cpp)
set_target_properties(kinematics_horst_link PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(kinematics_horst_link PRIVATE bike_test_fixtures
  bike_kinematics) # bike_geometry comes transitively (PUBLIC edge).
target_include_directories(kinematics_horst_link SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
  bike_apply_native_policy(kinematics_horst_link)

if(BUILD_TESTING)
  add_test(NAME kinematics_horst_link COMMAND kinematics_horst_link)
  set_tests_properties(kinematics_horst_link PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:kinematics_horst_link>")
endif()
