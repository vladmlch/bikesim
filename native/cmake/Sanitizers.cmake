# Completion plan 3.42. Instrumentation belongs to BikeCompilerPolicy;
# controls link that same runtime and require no Python/preload launcher.
include_guard(GLOBAL)
if(NOT BUILD_TESTING)
  return()
endif()
foreach(_control sanitizer rtsan)
  add_executable(bike_${_control}_control
    "${CMAKE_SOURCE_DIR}/native/tests/${_control}_control.cpp")
  bike_apply_native_policy(bike_${_control}_control)
  add_test(NAME ${_control}_clean_control COMMAND bike_${_control}_control)
  set_tests_properties(${_control}_clean_control PROPERTIES LABELS "sanitizer;control")
endforeach()

if(NATIVE_SANITIZE)
  add_test(NAME sanitizer_negative_control
    COMMAND bike_sanitizer_control use-after-free)
  # CTest's output-match rule replaces the exit-status rule. WILL_FAIL here
  # would invert the result and reward absence of the required report.
  set_tests_properties(sanitizer_negative_control PROPERTIES
    PASS_REGULAR_EXPRESSION "ERROR: AddressSanitizer: heap-use-after-free"
    ENVIRONMENT "ASAN_OPTIONS=halt_on_error=1:abort_on_error=0:exitcode=86"
    LABELS "sanitizer;control" TIMEOUT 30)
endif()
if(NATIVE_RTSAN)
  add_test(NAME rtsan_negative_control
    COMMAND bike_rtsan_control forbidden-allocation)
  set_tests_properties(rtsan_negative_control PROPERTIES
    PASS_REGULAR_EXPRESSION "ERROR: RealtimeSanitizer"
    ENVIRONMENT "RTSAN_OPTIONS=halt_on_error=1:abort_on_error=0:exitcode=86"
    LABELS "sanitizer;control" TIMEOUT 30)
endif()
