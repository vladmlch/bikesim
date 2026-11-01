# Qualified BLAS/LAPACK provider binding for the native build.
#
# One source, the committed numerics record
# (native/numerics.lock.json, records[<platform key>]): integer widths,
# symbol spellings, provider identity, np.interp / np.multiply lowering —
# facts identical on every machine of the platform with the pinned NumPy.
# T63 removed the LIVE lane and its interpreter-probe document: no probe,
# no interpreter and no machine-specific value is consulted here. The
# platform key is derived from the host, and the entry it names must
# describe that same host.
#
#   BIKE_NUMERICS_SOURCE=RECORD_ONLY  the only mode since T63
#       (BIKE_NUMERICS_SOURCE=LIVE belonged to the removed transition graph
#       and now fails configure). Darwin binds the apple-system Accelerate
#       framework. Linux binds the locked wheel provider under
#       BIKE_DEP_numpy_openblas_ROOT; BikeCompilerPolicy's glibc probe
#       supplies the libc family without an interpreter. ABI facts require
#       the committed record produced by the original oracle probe — the
#       record is maintained as a committed artifact.
#
# There is deliberately no find_package(BLAS) here: a generic BLAS search
# could return a system library whose integer ABI differs from the NumPy
# provider the record describes, and a wrong BLAS integer width is memory
# corruption, not a link error.
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
#   BIKE_NUMERICS_SOURCE          RECORD_ONLY; the removed LIVE value fails
#                                 configure
#   BIKE_TARGET_SYSTEM            set by BikePlatform.cmake
#   BIKE_NUMERICS_RECORD          the committed record; defaults to
#                                 native/numerics.lock.json, overridable so
#                                 harnesses can supply a fixture record
#   BIKE_NUMERICS_MUJOCO_INCLUDE  the MuJoCo include root for the backend
#                                 TUs (cblas_abi.hpp includes mujoco.h)
#   BIKE_DEP_numpy_openblas_ROOT / BIKE_DEP_numpy_openblas_LIBRARY
#                                 Linux only: verified by the dependency
#                                 lock resolver, never a site-packages path.
#   BIKE_NUMERICS_APPLY_POLICY    optional: the policy function applied to
#                                 bike_native_numerics. Default
#                                 bike_apply_native_policy (BikeCompilerPolicy).
#
# Exports consumed by the rest of the native build:
#   Bike::Numerics                  ALIAS of bike_native_numerics — the
#                                   typed CBLAS/LAPACK adapter static
#                                   library (link this; the provider
#                                   arrives via its PUBLIC usage
#                                   requirements).
#   Bike::NumericsProvider          IMPORTED target — INTERFACE on
#                                   Darwin (-framework Accelerate) or SHARED
#                                   with IMPORTED_LOCATION on Linux.
##   bike_native_numerics            STATIC target holding exactly
#                                   the two resolved backend TUs
#                                   (src/platform/blas_*.cpp +
#                                   lapack_*.cpp); created only when CXX
#                                   is enabled — record-only consumers
#                                   configure fine.
#   BIKE_NUMERICS_BACKEND_RESOLVED  ACCELERATE or NUMPY_OPENBLAS
#   _native_numerics_blas_bits      qualified BLAS integer width
#   _native_numerics_lapack_bits    qualified LAPACK integer width
#   _native_numerics_symbol_ddot        \
#   _native_numerics_symbol_dgemv        |
#   _native_numerics_symbol_dgemm        |
#   _native_numerics_symbol_dgelsd       > exact provider-export spellings
#   _native_numerics_symbol_dgesv        |
#   _native_numerics_symbol_dsyevd       |
#   _native_numerics_symbol_ddot_ilp64   |
#   _native_numerics_symbol_zdotu_sub   /
#   _native_numerics_provider       provider path (Linux file or Darwin
#                                   framework umbrella description)
#   _native_numerics_symbols_header generated numerics_symbols.hpp path
#   _native_numerics_record_key     the records[] key the check verified
#                                   (e.g. darwin-arm64)
#   _native_numerics_provider_paths the bound provider image per record entry,
#                                   in record order: real path on Linux,
#                                   system image on Darwin
#   _native_numerics_interp_contracted  1 when the wheel contracts
#                                   np.interp's slope*(x-dx)+dy into one
#                                   fmadd, 0 when it rounds separately —
#                                   projected as the directory-scope
#                                   BIKE_NUMPY_INTERP_CONTRACTED compile
#                                   definition

if(NOT DEFINED BIKE_TARGET_SYSTEM)
  message(FATAL_ERROR
    "BikeNumerics.cmake requires BIKE_TARGET_SYSTEM (see BikePlatform.cmake)")
endif()
# The mode is never inferred. RECORD_ONLY is the only lane since T63: the
# LIVE probe mode belonged to the removed transition graph and a stale
# request must fail configure, not silently degrade.
if(NOT DEFINED BIKE_NUMERICS_SOURCE
   OR NOT BIKE_NUMERICS_SOURCE STREQUAL "RECORD_ONLY")
  message(FATAL_ERROR
    "BikeNumerics.cmake requires BIKE_NUMERICS_SOURCE=RECORD_ONLY; the "
    "LIVE probe lane was removed at T63 and there is no default")
endif()

set(BIKE_NUMERICS_BACKEND "AUTO" CACHE STRING
  "Native numerical backend: AUTO|ACCELERATE|NUMPY_OPENBLAS")
set_property(CACHE BIKE_NUMERICS_BACKEND PROPERTY STRINGS
  AUTO ACCELERATE NUMPY_OPENBLAS)
set(BIKE_NUMERICS_RECORD "${CMAKE_CURRENT_LIST_DIR}/../numerics.lock.json"
  CACHE FILEPATH "Committed numerics record (records[<platform key>] holds the facts)")
get_filename_component(_numerics_record_file "${BIKE_NUMERICS_RECORD}" ABSOLUTE)

# --------------------------------------------------------------------------
# JSON field access — typed member reads only, fail closed on anything
# missing/mistyped/-NOTFOUND. Document text is never evaluated. The only
# source is the committed record's entry for this platform.
# --------------------------------------------------------------------------
macro(_numerics_record_field _out_var)
  string(JSON ${_out_var} ERROR_VARIABLE _numerics_json_error
    GET "${_numerics_record_json}" records "${_native_numerics_record_key}" ${ARGN})
  if(_numerics_json_error OR "${${_out_var}}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "committed numerics record '${_native_numerics_record_key}' is missing "
      "required field '${ARGN}': ${_numerics_json_error} — the record in "
      "${_numerics_record_file} must be regenerated for this platform")
  endif()
endmacro()

# The key is the host's platform identity (system.lower() + machine). The
# entry's own platform fields are checked against the host below, so a
# hand-edited key cannot select another entry.
if(NOT EXISTS "${_numerics_record_file}")
  message(FATAL_ERROR
    "committed numerics record is missing: ${_numerics_record_file}")
endif()
string(TOLOWER "${BIKE_TARGET_SYSTEM}" _numerics_host_system)
set(_native_numerics_record_key
  "${_numerics_host_system}-${CMAKE_HOST_SYSTEM_PROCESSOR}")

file(READ "${_numerics_record_file}" _numerics_record_json)
string(JSON _numerics_record_type ERROR_VARIABLE _numerics_record_error
  TYPE "${_numerics_record_json}" records "${_native_numerics_record_key}" numerics)
if(_numerics_record_error OR NOT _numerics_record_type STREQUAL "OBJECT")
  message(FATAL_ERROR
    "committed numerics record has no entry '${_native_numerics_record_key}' "
    "(${_numerics_record_error}) — the record in ${_numerics_record_file} "
    "must be regenerated for this platform")
endif()

_numerics_record_field(_numerics_backend numerics backend)
_numerics_record_field(_numerics_blas_bits numerics blas_integer_bits)
_numerics_record_field(_numerics_lapack_bits numerics lapack_integer_bits)
_numerics_record_field(_numerics_numpy_version numerics numpy_version)
# The entry must describe this host. The key was derived from these fields,
# so only a hand-edited record can differ; a Darwin entry must carry a null
# libc, because the record stores a libc family only on Linux.
_numerics_record_field(_numerics_record_system platform system)
_numerics_record_field(_numerics_record_machine platform machine)
string(JSON _numerics_record_libc_type ERROR_VARIABLE _numerics_json_error
  TYPE "${_numerics_record_json}" records "${_native_numerics_record_key}" platform libc)
if(NOT "${_numerics_record_system}" STREQUAL "${BIKE_TARGET_SYSTEM}"
   OR NOT "${_numerics_record_machine}" STREQUAL "${CMAKE_HOST_SYSTEM_PROCESSOR}")
  message(FATAL_ERROR
    "numerics record entry '${_native_numerics_record_key}' does not describe "
    "this host (${BIKE_TARGET_SYSTEM}/${CMAKE_HOST_SYSTEM_PROCESSOR}): it names "
    "${_numerics_record_system}/${_numerics_record_machine} with libc "
    "${_numerics_record_libc_type} in ${_numerics_record_file}")
endif()
if(BIKE_TARGET_SYSTEM STREQUAL "Linux")
  _numerics_record_field(_numerics_record_libc platform libc)
  if(NOT _numerics_record_libc_type STREQUAL "STRING"
     OR NOT _numerics_record_libc STREQUAL "glibc" OR NOT BIKE_TARGET_GLIBC)
    message(FATAL_ERROR
      "numerics record libc '${_numerics_record_libc}' does not match the host "
      "(glibc probe: ${BIKE_TARGET_GLIBC}); musl is not qualified")
  endif()
elseif(NOT _numerics_record_libc_type STREQUAL "NULL")
  message(FATAL_ERROR "Darwin numerics record platform.libc must be null")
endif()
message(STATUS
  "numerics record '${_native_numerics_record_key}' describes this host "
  "(record-only facts from the committed record)")

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
if(_numerics_backend STREQUAL "NUMPY_OPENBLAS")
  if(NOT BIKE_TARGET_SYSTEM STREQUAL "Linux"
     OR NOT IS_DIRECTORY "${BIKE_DEP_numpy_openblas_ROOT}"
     OR NOT EXISTS "${BIKE_DEP_numpy_openblas_LIBRARY}")
    message(FATAL_ERROR
      "record-only NUMPY_OPENBLAS requires the lock-verified Linux numpy_openblas dependency")
  endif()
endif()

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
foreach(_op ddot dgemv dgemm dgelsd dgesv dsyevd ddot_ilp64 zdotu_sub)
  _numerics_record_field(_native_numerics_symbol_${_op} numerics symbols ${_op})
  set(_sym "${_native_numerics_symbol_${_op}}")
  if(NOT _sym MATCHES "${_numerics_symbol_re}")
    message(FATAL_ERROR
      "numerics symbol for '${_op}' is not a safe identifier: '${_sym}'")
  endif()
  if(_op MATCHES "^(dgelsd|dgesv|dsyevd)$")
    if(_numerics_backend STREQUAL "ACCELERATE")
      if(NOT _sym MATCHES "^${_op}\\$NEWLAPACK\\$ILP64$")
        message(FATAL_ERROR
          "numerics symbol '${_sym}' for '${_op}' is not the qualified "
          "Accelerate NEWLAPACK ILP64 interface")
      endif()
    elseif(NOT _sym STREQUAL "scipy_${_op}_64_")
      message(FATAL_ERROR
        "numerics symbol '${_sym}' for '${_op}' is not the qualified "
        "scipy-openblas ILP64 Fortran spelling")
    endif()
  elseif(_op MATCHES "^(ddot_ilp64|zdotu_sub)$")
    string(REGEX REPLACE "_ilp64$" "" _stem "${_op}")
    if(_numerics_backend STREQUAL "ACCELERATE")
      if(NOT _sym MATCHES "^cblas_${_stem}\\$NEWLAPACK\\$ILP64$")
        message(FATAL_ERROR
          "numerics symbol '${_sym}' for '${_op}' is not the qualified "
          "Accelerate NEWLAPACK ILP64 CBLAS interface")
      endif()
    elseif(NOT _sym STREQUAL "scipy_cblas_${_stem}64_")
      message(FATAL_ERROR
        "numerics symbol '${_sym}' for '${_op}' is not the qualified "
        "scipy-openblas ILP64 CBLAS spelling 'scipy_cblas_${_stem}64_'")
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
# np.interp lowering — compiled_base.c's interior expression
# slope*(x - dx[j]) + dy[j] rounds the multiply and the add separately or
# contracts them into one fmadd depending on how the *wheel* was
# compiled, not on this compiler. The dependency probe measured the
# installed np.interp (numerics.interp.lowering: "contracted" or
# "separate"); the port must lower its interpolation mirrors the same
# way. The define is projected at directory scope so every first-party
# target — the extension, the shared contract core, the harnesses — sees
# numeric::np_interp_eval with identical semantics (the helper is an
# inline in numeric_numpy.hpp; an inconsistent define would be an ODR
# violation and a silent bitwise fork).
# --------------------------------------------------------------------------
_numerics_record_field(_numerics_interp_lowering numerics interp lowering)
if(_numerics_interp_lowering STREQUAL "contracted")
  set(_native_numerics_interp_contracted 1)
elseif(_numerics_interp_lowering STREQUAL "separate")
  set(_native_numerics_interp_contracted 0)
else()
  message(FATAL_ERROR
    "numerics.interp.lowering must be 'contracted' or 'separate', found "
    "'${_numerics_interp_lowering}' — the committed record must be "
    "regenerated for this platform")
endif()
add_compile_definitions(
  BIKE_NUMPY_INTERP_CONTRACTED=${_native_numerics_interp_contracted})
message(STATUS
  "np.interp lowering: ${_numerics_interp_lowering} "
  "(BIKE_NUMPY_INTERP_CONTRACTED=${_native_numerics_interp_contracted})")

# --------------------------------------------------------------------------
# multiply(scalar, array) NaN precedence — an sNaN quiets and wins from
# either operand position on AArch64, but a same-class dual-NaN lane
# resolves to fmul operand 0, and which source operand the ufunc
# broadcast kernel loads there is a *wheel* property: the AppleClang
# wheel keeps the scalar in operand 0 (its NaN wins the tie), the
# manylinux wheel's broadcast loop puts the contiguous array there (the
# element's NaN wins). The dependency probe measured which
# (numerics.multiply_broadcast.nan_winner: "scalar" or "element"); the
# port selects the same operand-0 winner through
# numeric::np_multiply_broadcast. Directory scope like the interp
# define — an inconsistent inline-helper define is an ODR violation.
# --------------------------------------------------------------------------
_numerics_record_field(_numerics_mul_nan_winner
  numerics multiply_broadcast nan_winner)
if(_numerics_mul_nan_winner STREQUAL "element")
  set(_native_numerics_mul_element_nan 1)
elseif(_numerics_mul_nan_winner STREQUAL "scalar")
  set(_native_numerics_mul_element_nan 0)
else()
  message(FATAL_ERROR
    "numerics.multiply_broadcast.nan_winner must be 'scalar' or "
    "'element', found '${_numerics_mul_nan_winner}' — the committed record "
    "must be regenerated for this platform")
endif()
add_compile_definitions(
  BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=${_native_numerics_mul_element_nan})
message(STATUS
  "multiply broadcast NaN winner: ${_numerics_mul_nan_winner} "
  "(BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN="
  "${_native_numerics_mul_element_nan})")

# --------------------------------------------------------------------------
# Generator.uniform/normal tail transform — distributions.c evaluates
# low + range*u and loc + scale*z as a multiply then an add per draw;
# whether they contract into one fmadd is a wheel-build property
# (AppleClang wheel: contracted; manylinux wheel: separate). The
# dependency probe measured which (numerics.distributions.lowering); the
# port selects the same spelling through
# BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED. Directory scope like the interp
# define — an inconsistent define is an ODR violation.
# --------------------------------------------------------------------------
_numerics_record_field(_numerics_distributions_lowering
  numerics distributions lowering)
if(_numerics_distributions_lowering STREQUAL "contracted")
  set(_native_numerics_distributions_contracted 1)
elseif(_numerics_distributions_lowering STREQUAL "separate")
  set(_native_numerics_distributions_contracted 0)
else()
  message(FATAL_ERROR
    "numerics.distributions.lowering must be 'contracted' or 'separate', "
    "found '${_numerics_distributions_lowering}' — the committed record "
    "must be regenerated for this platform")
endif()
add_compile_definitions(
  BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED=${_native_numerics_distributions_contracted})
message(STATUS
  "Generator distributions lowering: ${_numerics_distributions_lowering} "
  "(BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED="
  "${_native_numerics_distributions_contracted})")

# --------------------------------------------------------------------------
# scipy.ndimage symmetric correlate — NI_Correlate1D walks each
# coefficient pair outermost-first as out += (left + right) * weight;
# whether that multiply/accumulate contracts into one fmadd is a
# wheel-build property (AppleClang wheel: contracted; manylinux wheel:
# separate). The dependency probe measured which
# (numerics.scipy_correlate.lowering); the port selects the same spelling
# through BIKE_SCIPY_CORRELATE_CONTRACTED. Directory scope like the
# interp define — an inconsistent define is an ODR violation.
# --------------------------------------------------------------------------
_numerics_record_field(_numerics_scipy_correlate_lowering
  numerics scipy_correlate lowering)
if(_numerics_scipy_correlate_lowering STREQUAL "contracted")
  set(_native_numerics_scipy_correlate_contracted 1)
elseif(_numerics_scipy_correlate_lowering STREQUAL "separate")
  set(_native_numerics_scipy_correlate_contracted 0)
else()
  message(FATAL_ERROR
    "numerics.scipy_correlate.lowering must be 'contracted' or "
    "'separate', found '${_numerics_scipy_correlate_lowering}' — the "
    "committed record must be regenerated for this platform")
endif()
add_compile_definitions(
  BIKE_SCIPY_CORRELATE_CONTRACTED=${_native_numerics_scipy_correlate_contracted})
message(STATUS
  "scipy correlate lowering: ${_numerics_scipy_correlate_lowering} "
  "(BIKE_SCIPY_CORRELATE_CONTRACTED="
  "${_native_numerics_scipy_correlate_contracted})")

# --------------------------------------------------------------------------
# Provider identity — roles must cover blas+lapack; the Linux file provider
# must exist inside the recorded site-packages and carry its SHA-256;
# Accelerate entries carry the apple-system identity with a real build id
# (dyld-cache images have no file to hash).
# The record stores Linux providers relative to site-packages;
# RECORD_ONLY maps numpy.libs/<file> to the lock's lib/<file>, then checks
# containment and the recorded SHA-256. Darwin paths are absolute system
# images and need no resolution. No build_id is read: the provider is
# bound by identity kind, and a machine-specific value has no committed
# source for this graph.
# --------------------------------------------------------------------------
string(JSON _numerics_library_count ERROR_VARIABLE _numerics_json_error
  LENGTH "${_numerics_record_json}"
  records "${_native_numerics_record_key}" numerics library_paths)
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
# The bound provider image per record entry, in record order: the real path
# on Linux (what the runtime's loaded-image inspection reports), the system
# image on Darwin. CMakeLists.txt's identity block reads this list.
set(_native_numerics_provider_paths "")
foreach(_i RANGE ${_numerics_library_last})
  _numerics_record_field(_lib_path numerics library_paths ${_i} path)
  _numerics_record_field(_lib_role numerics library_paths ${_i} role)
  _numerics_record_field(_lib_kind numerics library_paths ${_i} identity_kind)
  list(APPEND _numerics_roles "${_lib_role}")
  if(_numerics_backend STREQUAL "NUMPY_OPENBLAS")
    if(NOT _lib_path MATCHES "^numpy\\.libs/[^/]+$")
      message(FATAL_ERROR "record-only NumPy provider path must be numpy.libs/<file>: '${_lib_path}'")
    endif()
    get_filename_component(_provider_name "${_lib_path}" NAME)
    set(_numerics_provider_root "${BIKE_DEP_numpy_openblas_ROOT}")
    set(_lib_path "${BIKE_DEP_numpy_openblas_ROOT}/lib/${_provider_name}")
    if(NOT _lib_path STREQUAL "${BIKE_DEP_numpy_openblas_LIBRARY}")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' differs from the locked scipy-openblas image '${BIKE_DEP_numpy_openblas_LIBRARY}'")
    endif()
  endif()
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
    _numerics_record_field(_lib_sha numerics library_paths ${_i} sha256)
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
        "numerics provider does not exist: '${_lib_path}' — acquire the pinned "
        "dependency for this graph before configuring")
    endif()
    file(SHA256 "${_lib_path}" _lib_actual_sha)
    if(NOT _lib_actual_sha STREQUAL _lib_sha)
      message(FATAL_ERROR
        "numerics provider SHA-256 differs for '${_lib_path}': record ${_lib_sha}, file ${_lib_actual_sha}")
    endif()
    # A system libblas.so can never be substituted by path in either mode.
    file(REAL_PATH "${_lib_path}" _lib_real)
    file(REAL_PATH "${_numerics_provider_root}" _site_real)
    file(RELATIVE_PATH _lib_rel "${_site_real}" "${_lib_real}")
    if(_lib_rel MATCHES "^\\.\\.")
      message(FATAL_ERROR
        "numerics provider '${_lib_real}' is outside the selected NumPy "
        "provider root '${_site_real}'")
    endif()
    set(_numerics_provider "${_lib_real}")
    list(APPEND _native_numerics_provider_paths "${_lib_real}")
  else()
    if(NOT _lib_kind STREQUAL "apple-system")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has identity '${_lib_kind}', "
        "not 'apple-system' — Darwin binds only the dyld-cache Accelerate")
    endif()
    list(APPEND _native_numerics_provider_paths "${_lib_path}")
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
# mutates provider threading here. Both modes use these same imported-target
# branches; Linux RECORD_ONLY has already resolved and verified its .deps image.
# --------------------------------------------------------------------------
if(_numerics_backend STREQUAL "ACCELERATE")
  set(_numerics_provider "Accelerate.framework (apple-system)")
endif()
if(NOT TARGET Bike::NumericsProvider)
  if(_numerics_backend STREQUAL "ACCELERATE")
    add_library(Bike::NumericsProvider INTERFACE IMPORTED GLOBAL)
    set_target_properties(Bike::NumericsProvider PROPERTIES
      INTERFACE_LINK_LIBRARIES "-framework Accelerate")
  else()
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
foreach(_op ddot dgemv dgemm dgelsd dgesv dsyevd ddot_ilp64 zdotu_sub)
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
# platform per interface (verified against the generated record width at
# compile time), strict ISO C++23, PIC, and the provider through PUBLIC
# usage requirements so Python-free harnesses receive the link too. The
# policy applied is BIKE_NUMERICS_APPLY_POLICY (default
# bike_apply_native_policy, BikeCompilerPolicy, T09). The mujoco include
# dir is supplied by the includer as BIKE_NUMERICS_MUJOCO_INCLUDE — the
# blas backend TU reaches cblas_abi.hpp's mjtNum assertion through it.
# --------------------------------------------------------------------------
if(NOT DEFINED _native_numerics_target_guard)
  set(_native_numerics_target_guard TRUE)
  if(CMAKE_CXX_COMPILER_LOADED)
    if(_numerics_backend STREQUAL "ACCELERATE")
      set(_numerics_backend_sources
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/blas_accelerate.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/lapack_accelerate.cpp")
    else()
      set(_numerics_backend_sources
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/blas_numpy_openblas.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../src/platform/lapack_numpy_openblas.cpp")
    endif()
    if(NOT DEFINED BIKE_NUMERICS_MUJOCO_INCLUDE
       OR "${BIKE_NUMERICS_MUJOCO_INCLUDE}" STREQUAL "")
      message(FATAL_ERROR
        "BikeNumerics.cmake requires BIKE_NUMERICS_MUJOCO_INCLUDE (the "
        "MuJoCo include root of the locked dependency tree) to build "
        "bike_native_numerics")
    endif()
    set(_numerics_mujoco_include "${BIKE_NUMERICS_MUJOCO_INCLUDE}")
    add_library(bike_native_numerics STATIC "${_numerics_backend_sources}")
    set_target_properties(bike_native_numerics PROPERTIES
      CXX_STANDARD 23
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
      POSITION_INDEPENDENT_CODE ON)
    target_include_directories(bike_native_numerics PRIVATE
      "${_native_numerics_symbols_dir}")
    # cblas_abi.hpp's public scalar-width contract includes mujoco.h. Inline
    # geometry callers now consume that face, so its include root is a usage
    # requirement; the private provider-symbol header remains backend-only.
    target_include_directories(bike_native_numerics SYSTEM PUBLIC
      "${_numerics_mujoco_include}")
    # Fail closed: a CXX-enabled project that reaches this point is a real
    # target-build context, so the compiler policy must already be loaded.
    # Silently skipping the policy would emit a first-party TU without the
    # contract flags. The default is the port's policy; an includer that has
    # not loaded it must name the policy it did load.
    if(NOT DEFINED BIKE_NUMERICS_APPLY_POLICY)
      set(BIKE_NUMERICS_APPLY_POLICY bike_apply_native_policy)
    endif()
    if(NOT COMMAND ${BIKE_NUMERICS_APPLY_POLICY})
      message(FATAL_ERROR
        "${BIKE_NUMERICS_APPLY_POLICY} is undefined while CXX is enabled: "
        "include the policy module before BikeNumerics.cmake under the project, "
        "or name a loaded policy in BIKE_NUMERICS_APPLY_POLICY")
    endif()
    cmake_language(CALL ${BIKE_NUMERICS_APPLY_POLICY} bike_native_numerics)
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
