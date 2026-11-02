# random domain contract tests.

# seeded-random regression: SeedSequence entropy coercion/pool mixing/
# spawn bookkeeping, the PCG64 raw/uniform/Ziggurat-normal streams incl.
# the buffered next_uint32 lane, Lemire bounded integers and choice(),
# sensor noise NoiseTape/SensorNoiseSource/build_noise_tape and
# rider_random.hpp's RiderRandomSpec/sample_rider — linked against the
# bike_random library (one compile per TU per graph) plus the shared
# fixture harness for the bitwise fixtures.
add_executable(random_seeded test_seeded.cpp)
set_target_properties(random_seeded PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(random_seeded PRIVATE bike_test_fixtures
  bike_random)
target_include_directories(random_seeded SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
  bike_apply_native_policy(random_seeded)

if(BUILD_TESTING)
  add_test(NAME random_seeded COMMAND random_seeded)
  set_tests_properties(random_seeded PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:random_seeded>")
endif()
