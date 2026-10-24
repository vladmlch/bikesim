cmake_minimum_required(VERSION 3.27)
if(NOT DEFINED ENV{DISPLAY} AND NOT APPLE)
  message(FATAL_ERROR "viewer smoke requires a real display")
endif()
foreach(_required BIKE_RIDE EXAMPLE PHYSICS_CONFIG)
  if(NOT DEFINED ${_required} OR NOT EXISTS "${${_required}}")
    message(FATAL_ERROR "viewer smoke requires existing ${_required}")
  endif()
endforeach()
# No source tree, output or test fixture is modified. The ordinary lifecycle
# closes the main-thread viewer on terminal completion. Inputs C/1/2 and R
# remain a separately recorded manual V08 interaction check.
execute_process(COMMAND "${BIKE_RIDE}" --spatial 3d --terrain3d "${EXAMPLE}"
    --physics-config "${PHYSICS_CONFIG}" --duration 2 --no-plots --close-on-finish
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 180)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "spatial viewer failed (${_rc}): ${_out}${_err}")
endif()
