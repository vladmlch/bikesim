cmake_minimum_required(VERSION 3.27)
# native/tests/package/no_python.cmake — T62 installed-runtime audit gate
# (completion-plan Task 4.04, plan-09 T62). Rebuilds the binary package,
# extracts it into scratch, and runs bike-python-audit --mode package over
# the real install tree. package_smoke runs the same audit inline as one
# step of a longer smoke; this test isolates the audit contract so the
# `-L audit` lane reports interpreter-freedom on its own. Exit 0 means the
# shipped tree carries no Python artifact, interpreter shebang, venv/site
# residue or Python-linked binary.
foreach(_name BUILD_DIR BINARY_PACKAGE AUDIT_BINARY AUDIT_POLICY CONFIG)
  if(NOT DEFINED ${_name} OR "${${_name}}" STREQUAL "")
    message(FATAL_ERROR "no_python requires ${_name}")
  endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
set(_scratch "${_build}/no_python")
if(IS_SYMLINK "${_scratch}")
  message(FATAL_ERROR "Refusing a symlinked no-python audit directory")
endif()
file(REMOVE_RECURSE "${_scratch}")
file(MAKE_DIRECTORY "${_scratch}/packages" "${_scratch}/binary")
find_program(_cpack NAMES cpack REQUIRED)
execute_process(COMMAND "${_cpack}" --config "${_build}/CPackConfig.cmake"
    -G TGZ -C "${CONFIG}" -B "${_scratch}/packages"
  WORKING_DIRECTORY "${_build}" RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "binary CPack failed (${_rc}): ${_out}${_err}")
endif()
set(_archive "${_scratch}/packages/${BINARY_PACKAGE}.tar.gz")
if(NOT EXISTS "${_archive}")
  message(FATAL_ERROR "Missing expected binary package ${_archive}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xf "${_archive}"
  WORKING_DIRECTORY "${_scratch}/binary" RESULT_VARIABLE _rc ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Cannot extract binary package: ${_err}")
endif()
set(_prefix "${_scratch}/binary/${BINARY_PACKAGE}")
execute_process(COMMAND "${AUDIT_BINARY}" --mode package --root "${_prefix}" --policy "${AUDIT_POLICY}"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 120)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Installed-tree Python audit failed (${_rc}): ${_out}${_err}")
endif()
message(STATUS "Installed tree is Python-free under ${AUDIT_POLICY} package rules")
