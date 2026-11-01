cmake_minimum_required(VERSION 3.27)
# migration_T63 — the Python-removal transaction's own contract.
#
# Asserts, against the tracked tree (never the build tree):
#   1. The produced source audit exits 0 with its clean summary.
#   2. Every tracked .py/.pyi path sits inside the approved exempt set
#      (.agents/**, tools/research/**).
#   3. Every removal-ledger row is status deleted|exempt with
#      verification_ids; deleted paths are absent on disk and untracked,
#      exempt paths exist on disk and carry exempt_reason.
#   4. The binding/package build surfaces are gone: no binding-only TU or
#      graph file is tracked, no FindPython discovery or bindings option
#      declaration remains on a live (non-comment) cmake line.
#   5. migration/final-closure.json exists, names T63, records the same
#      deleted/exempt split the ledger holds, and lists the documented
#      qualification exceptions.
#   6. The tracked tree is clean (git status --porcelain is empty) — the
#      removal commits are the state being qualified.
#
# Inputs (cmake -D):
#   AUDIT_BINARY  the built bike-python-audit executable
#   POLICY        migration/no-python-policy.json
#   REPO_ROOT     repository root (tracked tree under audit)
#   LEDGER        migration/python-removal.json
#   CLOSURE       migration/final-closure.json

foreach(_required AUDIT_BINARY POLICY REPO_ROOT LEDGER CLOSURE)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "migration_T63 requires ${_required}")
  endif()
endforeach()

function(_t63_fail _message)
  message(FATAL_ERROR "migration_T63: ${_message}")
endfunction()

# --- 1. produced source audit ------------------------------------------------
if(NOT EXISTS "${AUDIT_BINARY}")
  _t63_fail("audit binary missing: ${AUDIT_BINARY}")
endif()
if(NOT EXISTS "${POLICY}")
  _t63_fail("audit policy missing: ${POLICY}")
endif()
execute_process(
  COMMAND "${AUDIT_BINARY}" --mode source --root "${REPO_ROOT}"
          --policy "${POLICY}"
  RESULT_VARIABLE _audit_rc
  OUTPUT_VARIABLE _audit_out ERROR_VARIABLE _audit_err)
if(NOT _audit_rc EQUAL 0)
  _t63_fail("source audit exited ${_audit_rc}:\n${_audit_out}${_audit_err}")
endif()
if(NOT _audit_out MATCHES "source audit clean")
  _t63_fail("source audit did not print the clean summary: ${_audit_out}")
endif()

# --- 2. tracked interpreter sources confined to the exempt set ----------------
execute_process(
  COMMAND git -C "${REPO_ROOT}" ls-files
  RESULT_VARIABLE _git_rc OUTPUT_VARIABLE _tracked_raw)
if(NOT _git_rc EQUAL 0)
  _t63_fail("git ls-files failed (${_git_rc})")
endif()
string(REPLACE "\n" ";" _tracked "${_tracked_raw}")
set(_tracked_surfaces "")
foreach(_file IN LISTS _tracked)
  if(_file MATCHES "\\.pyi?$"
     AND NOT _file MATCHES "^\\.agents/"
     AND NOT _file MATCHES "^tools/research/")
    list(APPEND _tracked_surfaces "${_file}")
  endif()
endforeach()
if(_tracked_surfaces)
  _t63_fail("tracked interpreter sources outside the exempt set: "
            "${_tracked_surfaces}")
endif()

# --- 3. removal-ledger rows ---------------------------------------------------
if(NOT EXISTS "${LEDGER}")
  _t63_fail("removal ledger missing: ${LEDGER}")
endif()
file(READ "${LEDGER}" _ledger_json)
string(JSON _row_count ERROR_VARIABLE _json_error LENGTH "${_ledger_json}")
if(_json_error OR NOT _row_count MATCHES "^[0-9]+$")
  _t63_fail("removal ledger is not a JSON array: ${_json_error}")
endif()
math(EXPR _row_last "${_row_count} - 1")
set(_deleted_count 0)
set(_exempt_count 0)
foreach(_i RANGE ${_row_last})
  string(JSON _path GET "${_ledger_json}" ${_i} path)
  string(JSON _status GET "${_ledger_json}" ${_i} status)
  string(JSON _vid_count ERROR_VARIABLE _vid_error
    LENGTH "${_ledger_json}" ${_i} verification_ids)
  if(_vid_error OR _vid_count LESS 1)
    _t63_fail("ledger row '${_path}' carries no verification_ids")
  endif()
  if(_status STREQUAL "deleted")
    math(EXPR _deleted_count "${_deleted_count} + 1")
    if(EXISTS "${REPO_ROOT}/${_path}")
      _t63_fail("deleted ledger path still present: ${_path}")
    endif()
    if(_path IN_LIST _tracked)
      _t63_fail("deleted ledger path still tracked: ${_path}")
    endif()
  elseif(_status STREQUAL "exempt")
    math(EXPR _exempt_count "${_exempt_count} + 1")
    if(NOT _path MATCHES "^\\.agents/"
       AND NOT _path MATCHES "^tools/research/")
      _t63_fail("exempt ledger row outside the approved set: ${_path}")
    endif()
    if(NOT EXISTS "${REPO_ROOT}/${_path}")
      _t63_fail("exempt ledger path missing on disk: ${_path}")
    endif()
    string(JSON _reason ERROR_VARIABLE _reason_error
      GET "${_ledger_json}" ${_i} exempt_reason)
    if(_reason_error OR _reason STREQUAL "")
      _t63_fail("exempt ledger row has no exempt_reason: ${_path}")
    endif()
  else()
    _t63_fail("ledger row '${_path}' has status '${_status}' — expected "
              "deleted or exempt")
  endif()
endforeach()

# --- 4. binding/package build surfaces ---------------------------------------
set(_absent_surfaces
  "native/CMakeLists.txt"
  "native/CMakePresets.json"
  "native/cmake/BikeDependencies.cmake"
  "native/src/binding.cpp"
  "native/src/config.hpp"
  "native/src/binding_arrays.hpp"
  "native/src/binding_readers.hpp"
  "native/src/binding_ownership.hpp"
  "native/src/runtime/test_adapter.cpp"
  "native/src/runtime/test_adapter.hpp"
  "pyproject.toml"
  "uv.lock"
  ".python-version")
foreach(_surface IN LISTS _absent_surfaces)
  if(_surface IN_LIST _tracked OR EXISTS "${REPO_ROOT}/${_surface}")
    _t63_fail("binding/package surface still present: ${_surface}")
  endif()
endforeach()
# The binding-only surface lived under native/src/ (drive/policy/contact/
# intent/spindle wire adapters + *_binding translation units); wire-codec
# coverage elsewhere (e.g. reference/test_native_pneumatic_wire.cpp) is
# legitimate and stays.
foreach(_file IN LISTS _tracked)
  if(_file MATCHES "^native/src/.*(_binding|_wire)\\.(cpp|hpp)$")
    _t63_fail("binding-only translation unit still tracked: ${_file}")
  endif()
endforeach()
# No live interpreter discovery or bindings option declaration may remain
# on a non-comment cmake line.
foreach(_file IN LISTS _tracked)
  if(_file MATCHES "(CMakeLists\\.txt|\\.cmake)$"
     AND NOT _file MATCHES "^docs/")
    file(STRINGS "${REPO_ROOT}/${_file}" _lines)
    foreach(_line IN LISTS _lines)
      if(_line MATCHES "^[ \t]*#")
        continue()
      endif()
      if(_line MATCHES "find_package[ \t]*\\([ \t]*Python"
         OR _line MATCHES "option[ \t]*\\([ \t]*BIKE_BUILD_PYTHON_BINDINGS")
        _t63_fail("live interpreter discovery remains in ${_file}: ${_line}")
      endif()
    endforeach()
  endif()
endforeach()

# --- 5. final-closure evidence ------------------------------------------------
if(NOT EXISTS "${CLOSURE}")
  _t63_fail("final-closure document missing: ${CLOSURE}")
endif()
file(READ "${CLOSURE}" _closure_json)
string(JSON _closure_task ERROR_VARIABLE _closure_error
  GET "${_closure_json}" task)
if(_closure_error OR NOT _closure_task STREQUAL "T63")
  _t63_fail("final-closure task field is '${_closure_task}' "
            "(${_closure_error})")
endif()
string(JSON _closure_exceptions ERROR_VARIABLE _closure_error
  LENGTH "${_closure_json}" qualification_exceptions)
if(_closure_error OR NOT _closure_exceptions GREATER_EQUAL 7)
  _t63_fail("final-closure records ${_closure_exceptions} qualification "
            "exceptions; the documented set has 7")
endif()
string(JSON _closure_deleted ERROR_VARIABLE _closure_error
  GET "${_closure_json}" ledger deleted)
string(JSON _closure_exempt ERROR_VARIABLE _closure_error2
  GET "${_closure_json}" ledger exempt)
if(_closure_error OR _closure_error2
   OR NOT _closure_deleted EQUAL _deleted_count
   OR NOT _closure_exempt EQUAL _exempt_count)
  _t63_fail("final-closure ledger split ${_closure_deleted}/${_closure_exempt} "
            "does not match the ledger ${_deleted_count}/${_exempt_count}")
endif()
string(JSON _closure_gate ERROR_VARIABLE _closure_error
  GET "${_closure_json}" verification native_gate)
if(_closure_error OR _closure_gate STREQUAL "")
  _t63_fail("final-closure verification.native_gate is empty")
endif()

# --- 6. clean tracked tree -----------------------------------------------------
execute_process(
  COMMAND git -C "${REPO_ROOT}" status --porcelain
  RESULT_VARIABLE _git_rc OUTPUT_VARIABLE _dirty_raw)
if(NOT _git_rc EQUAL 0)
  _t63_fail("git status --porcelain failed (${_git_rc})")
endif()
string(STRIP "${_dirty_raw}" _dirty)
if(NOT _dirty STREQUAL "")
  _t63_fail("tracked tree is not clean — the removal commits must be the "
            "qualified state:\n${_dirty}")
endif()

message(STATUS
  "migration_T63 satisfied: ${_deleted_count} deleted + "
  "${_exempt_count} exempt ledger rows, source audit clean, closure "
  "recorded, tree clean")
