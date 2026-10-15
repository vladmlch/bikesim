# Qualified BLAS/LAPACK provider binding for the native build.
#
# This module consumes ONLY the numerics record of the schema-1 dependency
# document produced by tools/native_dependencies.py — the probe already ran
# the provider inside the selected interpreter, inspected its export table,
# and compile/linked/ran a C control against the qualified signatures. There
# is deliberately no find_package(BLAS) here: a generic BLAS search could
# return a system library whose integer ABI differs from the NumPy provider
# the record describes, and a wrong BLAS integer width is memory corruption,
# not a link error.
#
# Selection:
#   BIKE_NUMERICS_BACKEND=AUTO          (default) platform profile:
#                                       ACCELERATE on Darwin, NUMPY_OPENBLAS
#                                       on Linux
#   BIKE_NUMERICS_BACKEND=ACCELERATE    system Accelerate (Darwin only)
#   BIKE_NUMERICS_BACKEND=NUMPY_OPENBLAS  the wheel's scipy-openblas ILP64
#                                       shared object (Linux only)
#
# Prerequisites for the includer:
#   BIKE_TARGET_SYSTEM            set by BikePlatform.cmake
#   _native_dependencies_json     the probe document, normally populated by
#                                 BikeDependencies.cmake; it may also be
#                                 file(READ) from a captured document
#
# Exports consumed by the rest of the native build:
#   Bike::Numerics                  ALIAS of bike_native_numerics — the
#                                   typed CBLAS adapter static library
#                                   (link this; the provider arrives via
#                                   its PUBLIC usage requirements)
#   Bike::NumericsProvider          IMPORTED target — INTERFACE on Darwin
#                                   (-framework Accelerate) or SHARED with
#                                   IMPORTED_LOCATION on Linux
#   bike_native_numerics            STATIC target holding exactly one
#                                   backend TU (src/platform/blas_*.cpp);
#                                   created only when CXX is enabled —
#                                   record-only consumers configure fine
#   BIKE_NUMERICS_BACKEND_RESOLVED  ACCELERATE or NUMPY_OPENBLAS
#   _native_numerics_blas_bits      qualified BLAS integer width
#   _native_numerics_lapack_bits    qualified LAPACK integer width
#   _native_numerics_symbol_ddot    \
#   _native_numerics_symbol_dgemv    > exact provider-export spellings
#   _native_numerics_symbol_dgemm    |
#   _native_numerics_symbol_dgelsd  /
#   _native_numerics_provider       provider path (Linux file or Darwin
#                                   framework umbrella description)
#   _native_numerics_symbols_header generated numerics_symbols.hpp path

if(NOT DEFINED BIKE_TARGET_SYSTEM)
  message(FATAL_ERROR
    "BikeNumerics.cmake requires BIKE_TARGET_SYSTEM (see BikePlatform.cmake)")
endif()
if(NOT DEFINED _native_dependencies_json
   OR "${_native_dependencies_json}" STREQUAL "")
  message(FATAL_ERROR
    "BikeNumerics.cmake requires the schema-1 dependency document in "
    "_native_dependencies_json (see BikeDependencies.cmake)")
endif()

set(BIKE_NUMERICS_BACKEND "AUTO" CACHE STRING
  "Native numerical backend: AUTO|ACCELERATE|NUMPY_OPENBLAS")
set_property(CACHE BIKE_NUMERICS_BACKEND PROPERTY STRINGS
  AUTO ACCELERATE NUMPY_OPENBLAS)

# --------------------------------------------------------------------------
# JSON field access — typed member reads only, fail closed on anything
# missing/mistyped/-NOTFOUND. Document text is never evaluated.
# --------------------------------------------------------------------------
macro(_numerics_field _out_var)
  string(JSON ${_out_var} ERROR_VARIABLE _numerics_json_error
    GET "${_native_dependencies_json}" ${ARGN})
  if(_numerics_json_error OR "${${_out_var}}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "numerics record is missing required field '${ARGN}': "
      "${_numerics_json_error} — run tools/native_dependencies.py to "
      "produce a qualified dependency document")
  endif()
endmacro()

# The numerics member must be a populated record, not JSON null.
string(JSON _numerics_type ERROR_VARIABLE _numerics_type_error
  TYPE "${_native_dependencies_json}" numerics)
if(_numerics_type_error OR NOT _numerics_type STREQUAL "OBJECT")
  message(FATAL_ERROR
    "dependency document has no qualified numerics record "
    "(numerics=${_numerics_type}); re-run tools/native_dependencies.py "
    "--python <selected interpreter> against the pinned environment")
endif()

_numerics_field(_numerics_backend numerics backend)
_numerics_field(_numerics_blas_bits numerics blas_integer_bits)
_numerics_field(_numerics_lapack_bits numerics lapack_integer_bits)
_numerics_field(_numerics_numpy_version numerics numpy_version)
_numerics_field(_numerics_site_packages numerics site_packages)

# --------------------------------------------------------------------------
# Backend resolution — AUTO binds the qualified profile for this platform;
# an explicit request must match what the probe actually qualified. A
# requested-but-different backend is a provider mismatch, never a silent
# substitution.
# --------------------------------------------------------------------------
if(BIKE_NUMERICS_BACKEND STREQUAL "AUTO")
  if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    set(_numerics_backend_expected "ACCELERATE")
  elseif(BIKE_TARGET_SYSTEM STREQUAL "Linux")
    set(_numerics_backend_expected "NUMPY_OPENBLAS")
  else()
    message(FATAL_ERROR
      "BIKE_NUMERICS_BACKEND=AUTO has no qualified profile for "
      "'${BIKE_TARGET_SYSTEM}'")
  endif()
elseif(BIKE_NUMERICS_BACKEND STREQUAL "ACCELERATE"
    OR BIKE_NUMERICS_BACKEND STREQUAL "NUMPY_OPENBLAS")
  set(_numerics_backend_expected "${BIKE_NUMERICS_BACKEND}")
else()
  message(FATAL_ERROR
    "unknown BIKE_NUMERICS_BACKEND='${BIKE_NUMERICS_BACKEND}'; expected "
    "AUTO|ACCELERATE|NUMPY_OPENBLAS")
endif()

if(NOT _numerics_backend STREQUAL _numerics_backend_expected)
  message(FATAL_ERROR
    "numerics backend mismatch: requested '${_numerics_backend_expected}' "
    "but the dependency document qualified '${_numerics_backend}' — the "
    "record and the request describe different providers")
endif()
set(BIKE_NUMERICS_BACKEND_RESOLVED "${_numerics_backend}")

# --------------------------------------------------------------------------
# Integer-width contract — the qualified profile per backend, checked from
# the record (the probe derived these from provider build configuration).
# --------------------------------------------------------------------------
if(_numerics_backend STREQUAL "ACCELERATE")
  set(_numerics_expected_blas_bits 32)
  set(_numerics_expected_lapack_bits 64)
else()
  set(_numerics_expected_blas_bits 64)
  set(_numerics_expected_lapack_bits 64)
endif()
foreach(_pair "blas:${_numerics_blas_bits}:${_numerics_expected_blas_bits}"
              "lapack:${_numerics_lapack_bits}:${_numerics_expected_lapack_bits}")
  string(REPLACE ":" ";" _fields "${_pair}")
  list(GET _fields 0 _which)
  list(GET _fields 1 _actual)
  list(GET _fields 2 _expected)
  if(NOT _actual MATCHES "^[0-9]+$" OR NOT _actual EQUAL _expected)
    message(FATAL_ERROR
      "unqualified numerics integer ABI: ${_which}_integer_bits='${_actual}' "
      "does not match the qualified ${_expected}-bit profile for "
      "${_numerics_backend}")
  endif()
endforeach()
set(_native_numerics_blas_bits "${_numerics_blas_bits}")
set(_native_numerics_lapack_bits "${_numerics_lapack_bits}")

# --------------------------------------------------------------------------
# Symbol table — exact spellings qualified by the probe. Identifiers must
# match the C-identifier-plus-$ grammar (the same rule checked_symbol
# enforces in Python) before any name can land in a generated declaration.
# Backend spellings pin the integer ABI: NEWLAPACK marks Accelerate's ILP64
# interface; scipy-openblas ILP64 carries the 64_ suffixes.
# --------------------------------------------------------------------------
set(_numerics_symbol_re "^[A-Za-z_][A-Za-z0-9_$]*$")
foreach(_op ddot dgemv dgemm dgelsd)
  _numerics_field(_native_numerics_symbol_${_op} numerics symbols ${_op})
  set(_sym "${_native_numerics_symbol_${_op}}")
  if(NOT _sym MATCHES "${_numerics_symbol_re}")
    message(FATAL_ERROR
      "numerics symbol for '${_op}' is not a safe identifier: '${_sym}'")
  endif()
  if(_op STREQUAL "dgelsd")
    if(_numerics_backend STREQUAL "ACCELERATE")
      if(NOT _sym MATCHES "NEWLAPACK\\$ILP64$")
        message(FATAL_ERROR
          "numerics symbol '${_sym}' for 'dgelsd' is not the qualified "
          "Accelerate NEWLAPACK ILP64 interface")
      endif()
    elseif(NOT _sym MATCHES "^scipy_.*_64_$" OR _sym MATCHES "cblas")
      message(FATAL_ERROR
        "numerics symbol '${_sym}' for 'dgelsd' is not the qualified "
        "scipy-openblas ILP64 Fortran spelling")
    endif()
  else()
    if(_numerics_backend STREQUAL "ACCELERATE")
      if(NOT _sym MATCHES "^cblas_${_op}$")
        message(FATAL_ERROR
          "numerics symbol '${_sym}' for '${_op}' is not the qualified "
          "legacy 32-bit CBLAS interface")
      endif()
    elseif(NOT _sym STREQUAL "scipy_cblas_${_op}64_")
      message(FATAL_ERROR
        "numerics symbol '${_sym}' for '${_op}' is not the qualified "
        "scipy-openblas ILP64 CBLAS spelling 'scipy_cblas_${_op}64_'")
    endif()
  endif()
endforeach()

# --------------------------------------------------------------------------
# Provider identity — roles must cover blas+lapack; the Linux file provider
# must exist inside the recorded site-packages and carry its SHA-256;
# Accelerate entries carry the apple-system identity with a real build id
# (dyld-cache images have no file to hash).
# --------------------------------------------------------------------------
string(JSON _numerics_library_count ERROR_VARIABLE _numerics_json_error
  LENGTH "${_native_dependencies_json}" numerics library_paths)
if(_numerics_json_error OR NOT _numerics_library_count MATCHES "^[0-9]+$")
  message(FATAL_ERROR
    "numerics record is missing 'library_paths': ${_numerics_json_error}")
endif()
if(_numerics_library_count LESS 1)
  message(FATAL_ERROR "numerics.library_paths is empty")
endif()
math(EXPR _numerics_library_last "${_numerics_library_count} - 1")
set(_numerics_roles "")
set(_numerics_provider "")
foreach(_i RANGE ${_numerics_library_last})
  _numerics_field(_lib_path numerics library_paths ${_i} path)
  _numerics_field(_lib_role numerics library_paths ${_i} role)
  _numerics_field(_lib_kind numerics library_paths ${_i} identity_kind)
  list(APPEND _numerics_roles "${_lib_role}")
  if(NOT IS_ABSOLUTE "${_lib_path}")
    message(FATAL_ERROR
      "numerics provider path is not absolute: '${_lib_path}'")
  endif()
  if(_numerics_backend STREQUAL "NUMPY_OPENBLAS")
    if(NOT _lib_kind STREQUAL "file")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has identity '${_lib_kind}', "
        "not 'file' — the wheel provider must be a hashable file")
    endif()
    _numerics_field(_lib_sha numerics library_paths ${_i} sha256)
    # CMake's MATCHES regex has no counted repetition — check the
    # lowercase-hex shape and the 64-char length separately.
    string(LENGTH "${_lib_sha}" _lib_sha_len)
    if(NOT _lib_sha MATCHES "^[0-9a-f]+$" OR NOT _lib_sha_len EQUAL 64)
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has no SHA-256 digest — the "
        "wheel file identity is unqualified")
    endif()
    if(NOT EXISTS "${_lib_path}")
      message(FATAL_ERROR
        "numerics provider does not exist: '${_lib_path}' — re-run the "
        "dependency probe against the selected interpreter")
    endif()
    # The provider must live inside the recorded NumPy install, so a
    # system libblas.so can never be substituted by path.
    file(REAL_PATH "${_lib_path}" _lib_real)
    file(REAL_PATH "${_numerics_site_packages}" _site_real)
    file(RELATIVE_PATH _lib_rel "${_site_real}" "${_lib_real}")
    if(_lib_rel MATCHES "^\\.\\.")
      message(FATAL_ERROR
        "numerics provider '${_lib_real}' is outside the installed NumPy "
        "distribution '${_site_real}'")
    endif()
    set(_numerics_provider "${_lib_real}")
  else()
    if(NOT _lib_kind STREQUAL "apple-system")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has identity '${_lib_kind}', "
        "not 'apple-system' — Darwin binds only the dyld-cache Accelerate")
    endif()
    _numerics_field(_lib_build numerics library_paths ${_i} build_id)
    if(_lib_build STREQUAL "")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has an empty build_id — the "
        "apple-system identity has no provider evidence")
    endif()
  endif()
endforeach()
foreach(_role blas lapack)
  list(FIND _numerics_roles "${_role}" _role_index)
  list(FIND _numerics_roles "blas_lapack" _combined_index)
  if(_role_index EQUAL -1 AND _combined_index EQUAL -1)
    message(FATAL_ERROR
      "numerics.library_paths has no provider for role '${_role}'")
  endif()
endforeach()

# --------------------------------------------------------------------------
# The imported provider target — normal imported-target semantics only: a
# framework link interface on Darwin, a SHARED import at the qualified
# path on Linux. Runtime kernels never link it directly: the adapter
# carries it through PUBLIC usage requirements. Nothing calls dlsym or
# mutates provider threading here.
# --------------------------------------------------------------------------
if(_numerics_backend STREQUAL "ACCELERATE")
  set(_numerics_provider "Accelerate.framework (apple-system)")
  if(NOT TARGET Bike::NumericsProvider)
    add_library(Bike::NumericsProvider INTERFACE IMPORTED GLOBAL)
    set_target_properties(Bike::NumericsProvider PROPERTIES
      INTERFACE_LINK_LIBRARIES "-framework Accelerate")
  endif()
else()
  if(NOT TARGET Bike::NumericsProvider)
    add_library(Bike::NumericsProvider SHARED IMPORTED GLOBAL)
    set_target_properties(Bike::NumericsProvider PROPERTIES
      IMPORTED_LOCATION "${_numerics_provider}")
  endif()
endif()

# --------------------------------------------------------------------------
# Generated symbol header — the record's qualified spellings land in
# numerics_symbols.hpp as asm-label string literals via configure_file.
# The label prefix is the object format's convention: Mach-O prepends
# "_" on Darwin; ELF symbols carry no leading underscore — reusing the
# Mach-O spelling on ELF produces an unresolved symbol, never a subtle
# redefinition.
# --------------------------------------------------------------------------
if(_numerics_backend STREQUAL "ACCELERATE")
  set(_numerics_asm_prefix "_")
else()
  set(_numerics_asm_prefix "")
endif()
foreach(_op ddot dgemv dgemm dgelsd)
  set(_numerics_asm_${_op}
      "${_numerics_asm_prefix}${_native_numerics_symbol_${_op}}")
endforeach()
set(_native_numerics_symbols_dir
    "${CMAKE_CURRENT_BINARY_DIR}/bike_numerics")
configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/../src/platform/numerics_symbols.hpp.in"
  "${_native_numerics_symbols_dir}/numerics_symbols.hpp"
  @ONLY)
set(_native_numerics_symbols_header
    "${_native_numerics_symbols_dir}/numerics_symbols.hpp")

# --------------------------------------------------------------------------
# bike_native_numerics — the typed adapter: exactly one backend TU per
# platform (verified against the generated record width at compile time),
# strict ISO C++23, PIC so it can join the extension module, and the
# provider through PUBLIC usage requirements so Python-free harnesses
# receive the link too. bike_apply_native_policy (BikeCompilerPolicy,
# T09) is applied when the includer has already defined it. The mujoco
# include dir comes from the same schema-1 document — the backend TU
# reaches cblas_abi.hpp's mjtNum assertion through it.
# --------------------------------------------------------------------------
if(NOT DEFINED _native_numerics_target_guard)
  set(_native_numerics_target_guard TRUE)
  if(CMAKE_CXX_COMPILER_LOADED)
    if(_numerics_backend STREQUAL "ACCELERATE")
      set(_numerics_backend_source
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/blas_accelerate.cpp")
    else()
      set(_numerics_backend_source
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/blas_numpy_openblas.cpp")
    endif()
    _numerics_field(_numerics_mujoco_include mujoco include_dir)
    add_library(bike_native_numerics STATIC "${_numerics_backend_source}")
    set_target_properties(bike_native_numerics PROPERTIES
      CXX_STANDARD 23
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
      POSITION_INDEPENDENT_CODE ON)
    target_include_directories(bike_native_numerics PRIVATE
      "${_native_numerics_symbols_dir}"
      "${_numerics_mujoco_include}")
    if(COMMAND bike_apply_native_policy)
      bike_apply_native_policy(bike_native_numerics)
    endif()
    target_link_libraries(bike_native_numerics
      PUBLIC Bike::NumericsProvider)
    if(NOT TARGET Bike::Numerics)
      add_library(Bike::Numerics ALIAS bike_native_numerics)
    endif()
  else()
    message(STATUS
      "bike_native_numerics not created: CXX is not enabled in this "
      "project (record-only consumption)")
  endif()
endif()

message(STATUS
  "numerics backend ${BIKE_NUMERICS_BACKEND_RESOLVED}: "
  "BLAS ${_native_numerics_blas_bits}-bit / "
  "LAPACK ${_native_numerics_lapack_bits}-bit, "
  "provider ${_numerics_provider}, NumPy ${_numerics_numpy_version}")
