cmake_minimum_required(VERSION 3.27)
# native/tests/package/audit_negative_controls.cmake — T62 negative controls
# for bike-python-audit (completion-plan Task 4.04, plan-09 T62). Rebuilds
# the binary package, unpacks one working copy, then injects each forbidden
# artifact class one at a time: every injection must make --mode package
# exit non-zero and the report must name the injected edge. Removing the
# artifact must restore a clean audit — the alternation proves the audit is
# not vacuously failing and reports the exact path/edge it found.
#
# The Python-linked-binary control needs no CPython: it compiles a stub
# shared library whose install name/SONAME is the platform's Python runtime
# path and links a two-line executable against it (Task 4.04 file block).
foreach(_name BUILD_DIR BINARY_PACKAGE AUDIT_BINARY AUDIT_POLICY CONFIG CMAKE_CXX_COMPILER)
  if(NOT DEFINED ${_name} OR "${${_name}}" STREQUAL "")
    message(FATAL_ERROR "audit_negative_controls requires ${_name}")
  endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
set(_scratch "${_build}/audit_negative_controls")
if(IS_SYMLINK "${_scratch}")
  message(FATAL_ERROR "Refusing a symlinked audit-control directory")
endif()
file(REMOVE_RECURSE "${_scratch}")
file(MAKE_DIRECTORY "${_scratch}/packages" "${_scratch}/pristine")
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
  WORKING_DIRECTORY "${_scratch}/pristine" RESULT_VARIABLE _rc ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Cannot extract binary package: ${_err}")
endif()
set(_case "${_scratch}/case")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory
    "${_scratch}/pristine/${BINARY_PACKAGE}" "${_case}"
  RESULT_VARIABLE _rc ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "Cannot stage the working package copy: ${_err}")
endif()

# _audit_run <out-rc> <out-text>: one package audit over the working copy.
function(_audit_run _rc_var _out_var)
  execute_process(COMMAND "${AUDIT_BINARY}" --mode package --root "${_case}"
      --policy "${AUDIT_POLICY}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 180)
  set(${_rc_var} "${_rc}" PARENT_SCOPE)
  set(${_out_var} "${_out}${_err}" PARENT_SCOPE)
endfunction()

# _expect_detected <label> <needle>: the last audit must have failed AND its
# report must carry the injected path/edge (literal substring, not regex).
function(_expect_detected _label _needle)
  if(_rc EQUAL 0)
    message(FATAL_ERROR
      "negative control '${_label}': package audit passed despite the injected artifact")
  endif()
  string(FIND "${_text}" "${_needle}" _pos)
  if(_pos EQUAL -1)
    message(FATAL_ERROR
      "negative control '${_label}': audit failed without reporting '${_needle}': ${_text}")
  endif()
  message(STATUS "negative control '${_label}': rejected with edge '${_needle}'")
endfunction()

# Baseline: the untouched package copy must audit clean, or every later
# rejection could come from residue instead of the injected artifact.
_audit_run(_rc _text)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "baseline package copy is not clean before injection: ${_text}")
endif()
message(STATUS "baseline package copy audits clean")

# --- artifact class: Python source/stub/bytecode/wheel/notebook files -----
foreach(_pair IN ITEMS
    "py_source|share/bike_sim/injected_cycle.py|print('planted')|injected_cycle.py"
    "pyi_stub|share/bike_sim/injected_stubs.pyi|def planted() -> None: ...|injected_stubs.pyi"
    "pyc_bytecode|share/bike_sim/injected_bytecode.pyc|planted-bytecode|injected_bytecode.pyc"
    "wheel|share/wheels/bike_sim-1.0-py3-none-any.whl|planted-wheel|bike_sim-1.0-py3-none-any.whl"
    "notebook|share/notebooks/injected.ipynb|{}|injected.ipynb")
  string(REPLACE "|" ";" _fields "${_pair}")
  list(GET _fields 0 _label)
  list(GET _fields 1 _relative)
  list(GET _fields 2 _content)
  list(GET _fields 3 _needle)
  get_filename_component(_parent "${_case}/${_relative}" DIRECTORY)
  file(MAKE_DIRECTORY "${_parent}")
  file(WRITE "${_case}/${_relative}" "${_content}\n")
  _audit_run(_rc _text)
  _expect_detected("${_label}" "${_needle}")
  file(REMOVE "${_case}/${_relative}")
  # Directories created only for the injection must not linger as residue.
  while(NOT _parent STREQUAL "${_case}")
    file(GLOB _leftover "${_parent}/*")
    if(NOT _leftover STREQUAL "")
      break()
    endif()
    file(REMOVE_RECURSE "${_parent}")
    get_filename_component(_parent "${_parent}" DIRECTORY)
  endwhile()
endforeach()

# --- artifact class: interpreter shebang executables -----------------------
# An extensionless file only trips the audit through its first line.
foreach(_pair IN ITEMS
    "shebang_env|bin/bike-tool-env|#!/usr/bin/env python3"
    "shebang_absolute|bin/bike-tool-abs|#!/usr/bin/python3.13")
  string(REPLACE "|" ";" _fields "${_pair}")
  list(GET _fields 0 _label)
  list(GET _fields 1 _relative)
  list(GET _fields 2 _shebang)
  file(WRITE "${_case}/${_relative}" "${_shebang}\nprint('planted')\n")
  _audit_run(_rc _text)
  _expect_detected("${_label}" "interpreter shebang '${_shebang}'")
  file(REMOVE "${_case}/${_relative}")
endforeach()

# --- artifact class: installed-package/venv directories --------------------
foreach(_pair IN ITEMS
    "site_packages|lib/site-packages|site-packages"
    "venv_marker|.venv|.venv"
    "dist_info|lib/bike_sim-1.0.dist-info|.dist-info"
    "python_version_dir|lib/python3.13|python3.13")
  string(REPLACE "|" ";" _fields "${_pair}")
  list(GET _fields 0 _label)
  list(GET _fields 1 _relative)
  list(GET _fields 2 _needle)
  file(MAKE_DIRECTORY "${_case}/${_relative}")
  _audit_run(_rc _text)
  _expect_detected("${_label}" "${_needle}")
  file(REMOVE_RECURSE "${_case}/${_relative}")
endforeach()

# --- artifact class: Python-linked binary ----------------------------------
# Stub runtime whose dynamic identity is the platform's Python library; a
# real function call keeps the link edge load-bearing on both platforms.
set(_stub_dir "${_scratch}/stub-build")
file(MAKE_DIRECTORY "${_stub_dir}")
file(WRITE "${_stub_dir}/python_runtime_stub.cpp"
  "extern \"C\" int bike_audit_stub_python(void) { return 0; }\n")
file(WRITE "${_stub_dir}/linked_tool.cpp"
  "extern \"C\" int bike_audit_stub_python(void);\nint main() { return bike_audit_stub_python(); }\n")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
  set(_stub_lib "${_stub_dir}/libpython3.13.dylib")
  set(_link_needle "Python.framework")
  execute_process(COMMAND "${CMAKE_CXX_COMPILER}" -dynamiclib
      -install_name /Library/Frameworks/Python.framework/Versions/3.13/Python
      -o "${_stub_lib}" "${_stub_dir}/python_runtime_stub.cpp"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
else()
  set(_stub_lib "${_stub_dir}/libpython3.13.so.1.0")
  set(_link_needle "libpython")
  execute_process(COMMAND "${CMAKE_CXX_COMPILER}" -fPIC -shared
      "-Wl,-soname,libpython3.13.so.1.0"
      -o "${_stub_lib}" "${_stub_dir}/python_runtime_stub.cpp"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
endif()
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "stub Python runtime failed to build (${_rc}): ${_out}${_err}")
endif()
execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "${_stub_dir}/linked_tool.cpp"
    "${_stub_lib}" -o "${_stub_dir}/bike-audit-linked"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "linked control executable failed to build (${_rc}): ${_out}${_err}")
endif()
get_filename_component(_stub_name "${_stub_lib}" NAME)
file(MAKE_DIRECTORY "${_case}/bin" "${_case}/lib")
file(COPY_FILE "${_stub_dir}/bike-audit-linked" "${_case}/bin/bike-audit-linked")
file(COPY_FILE "${_stub_lib}" "${_case}/lib/${_stub_name}")
_audit_run(_rc _text)
_expect_detected("linked_libpython" "${_link_needle}")
file(REMOVE "${_case}/bin/bike-audit-linked" "${_case}/lib/${_stub_name}")

# --- benign controls: names/prose that mention Python must not trip --------
# A C++ header named pyfloat.hpp and historical prose in a data file are
# required non-failures (plan-09 mandatory case). The .md body is never a
# shebang and no file name matches a forbidden glob.
file(WRITE "${_case}/share/pyfloat.hpp" "#pragma once\n")
file(MAKE_DIRECTORY "${_case}/share/notes")
file(WRITE "${_case}/share/notes/python_history.md"
  "This tree historically invoked python during development.\n")
_audit_run(_rc _text)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR
    "benign-name control: audit flagged historical names/prose: ${_text}")
endif()
message(STATUS "benign-name control: pyfloat.hpp and python prose accepted")
file(REMOVE "${_case}/share/pyfloat.hpp" "${_case}/share/notes/python_history.md")
file(REMOVE_RECURSE "${_case}/share/notes")

# Restoration: every injection removed must return the copy to a clean
# audit, proving each rejection came from its injected artifact alone.
_audit_run(_rc _text)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "package copy did not return to clean after removals: ${_text}")
endif()
message(STATUS "all negative controls tripped the package audit; restored copy is clean")
