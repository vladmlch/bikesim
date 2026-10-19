# Qualified BLAS/LAPACK provider binding for the native build.
#
# Two sources, with a split by what each may contain:
#
#   facts     the committed numerics record (native/numerics.lock.json,
#             records[<platform key>]) — integer widths, symbol spellings,
#             provider identity, np.interp / np.multiply lowering. Identical on
#             every machine of the platform with the pinned NumPy.
#   live      the schema-1 probe document captured by BikeDependencies.cmake —
#             machine-specific values only: the NumPy install location
#             (site_packages), the Darwin build_id, and the MuJoCo include.
#
# Every fact is read from the record in both modes. The modes differ in what
# else they require, and the includer selects one explicitly:
#
#   BIKE_NUMERICS_SOURCE=LIVE         the legacy binding graph (cmake -S native).
#       The live measurement is projected and compared with the committed entry
#       by tools/native_dependencies.py --check-numerics-record. A missing or
#       stale entry fails configure and names the regeneration command. Python
#       is required for that verdict. Machine-specific values come from the
#       probe document, and this mode creates the provider target and
#       bike_native_numerics.
#   BIKE_NUMERICS_SOURCE=RECORD_ONLY  the Python-free root graph (CMakeLists.txt).
#       No probe, no interpreter and no machine-specific value. The platform key
#       is derived from the host, and the entry it names must describe that same
#       host. The record's structure and facts are checked exactly as in LIVE
#       mode, but they are not compared with a measurement: the LIVE check run by
#       the binding graph is what keeps the committed record current. It exports
#       the compile definitions the root needs and binds the Darwin Accelerate
#       profile exactly as LIVE does: Bike::NumericsProvider and
#       bike_native_numerics are created. Accelerate is an apple-system framework,
#       so its location needs no probe. The includer supplies the two inputs the
#       live probe would otherwise provide (see Prerequisites). Only Darwin is
#       qualified here: a Linux provider lives inside the NumPy install, and
#       neither its location nor the libc family is known without a probe.
#
# The probe already ran the provider inside the selected interpreter, inspected
# its export table, and compile/linked/ran a C control against the qualified
# signatures. There is deliberately no find_package(BLAS) here: a generic BLAS
# search could return a system library whose integer ABI differs from the NumPy
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
#   BIKE_NUMERICS_SOURCE          LIVE or RECORD_ONLY; there is no default, so a
#                                 forgotten selection fails configure
#   BIKE_TARGET_SYSTEM            set by BikePlatform.cmake (LIVE), or by the root
#                                 from CMAKE_HOST_SYSTEM_NAME (RECORD_ONLY)
#   _native_dependencies_json     LIVE only: the live probe document, normally
#                                 populated by BikeDependencies.cmake; it may
#                                 also be file(READ) from a captured document
#   Python_EXECUTABLE             LIVE only: the selected interpreter that runs
#                                 the record check (stdlib only, no probe)
#   BIKE_NUMERICS_RECORD          the committed record; defaults to
#                                 native/numerics.lock.json, overridable so
#                                 harnesses can supply a fixture record
#   BIKE_NUMERICS_MUJOCO_INCLUDE  RECORD_ONLY only: the MuJoCo include root for
#                                 the backend TUs (cblas_abi.hpp includes
#                                 mujoco.h). LIVE takes it from the probe.
#   BIKE_NUMERICS_APPLY_POLICY    optional: the policy function applied to
#                                 bike_native_numerics. Default
#                                 bike_apply_native_policy (BikeCompilerPolicy);
#                                 the root graph sets bike_target_strict_policy
#                                 until it moves onto BikeCompilerPolicy.
#
# Exports consumed by the rest of the native build:
#   Bike::Numerics                  ALIAS of bike_native_numerics — the
#                                   typed CBLAS/LAPACK adapter static
#                                   library (link this; the provider
#                                   arrives via its PUBLIC usage
#                                   requirements). LIVE and RECORD_ONLY.
#   Bike::NumericsProvider          IMPORTED target — INTERFACE on
#                                   Darwin (-framework Accelerate) or SHARED
#                                   with IMPORTED_LOCATION on Linux (LIVE only).
#                                   LIVE and RECORD_ONLY (Darwin).
#   bike_native_numerics            STATIC target holding exactly
#                                   the two resolved backend TUs
#                                   (src/platform/blas_*.cpp +
#                                   lapack_*.cpp); created only when CXX
#                                   is enabled — record-only consumers
#                                   configure fine. LIVE and RECORD_ONLY.
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
# The mode is never inferred, neither from the host nor from whether Python is
# found: an includer that does not select one fails configure.
if(NOT DEFINED BIKE_NUMERICS_SOURCE
   OR NOT BIKE_NUMERICS_SOURCE MATCHES "^(LIVE|RECORD_ONLY)$")
  message(FATAL_ERROR
    "BikeNumerics.cmake requires BIKE_NUMERICS_SOURCE=LIVE (legacy graph, "
    "live probe) or RECORD_ONLY (Python-free root graph); there is no default")
endif()
if(BIKE_NUMERICS_SOURCE STREQUAL "LIVE")
  set(_numerics_is_live TRUE)
  if(NOT DEFINED _native_dependencies_json
     OR "${_native_dependencies_json}" STREQUAL "")
    message(FATAL_ERROR
      "BikeNumerics.cmake requires the schema-1 dependency document in "
      "_native_dependencies_json (see BikeDependencies.cmake)")
  endif()
else()
  set(_numerics_is_live FALSE)
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
# missing/mistyped/-NOTFOUND. Document text is never evaluated. Two sources:
# _numerics_record_field reads the committed record's entry for this
# platform (facts); _numerics_live_field reads the live probe document
# (machine-specific values only).
# --------------------------------------------------------------------------
macro(_numerics_record_field _out_var)
  string(JSON ${_out_var} ERROR_VARIABLE _numerics_json_error
    GET "${_numerics_record_json}" records "${_native_numerics_record_key}" ${ARGN})
  if(_numerics_json_error OR "${${_out_var}}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "committed numerics record '${_native_numerics_record_key}' is missing "
      "required field '${ARGN}': ${_numerics_json_error} — regenerate it "
      "with tools/native_dependencies.py --python <interpreter> "
      "--write-numerics-record ${_numerics_record_file}")
  endif()
endmacro()

macro(_numerics_live_field _out_var)
  string(JSON ${_out_var} ERROR_VARIABLE _numerics_json_error
    GET "${_native_dependencies_json}" ${ARGN})
  if(_numerics_json_error OR "${${_out_var}}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "dependency document is missing required field '${ARGN}': "
      "${_numerics_json_error} — run tools/native_dependencies.py to "
      "produce a qualified dependency document")
  endif()
endmacro()

if(_numerics_is_live)
  # The numerics member must be a populated record, not JSON null.
  string(JSON _numerics_type ERROR_VARIABLE _numerics_type_error
    TYPE "${_native_dependencies_json}" numerics)
  if(_numerics_type_error OR NOT _numerics_type STREQUAL "OBJECT")
    message(FATAL_ERROR
      "dependency document has no qualified numerics record "
      "(numerics=${_numerics_type}); re-run tools/native_dependencies.py "
      "--python <selected interpreter> against the pinned environment")
  endif()

  # ------------------------------------------------------------------------
  # Live-versus-committed check. The probe document is written to the build
  # tree so the stdlib-only checker can read it; the checker projects it,
  # requires its platform entry to equal the committed one, and prints that
  # entry's key. Its exit status is the verdict: a missing or stale entry is a
  # configure error that carries the regeneration command.
  # ------------------------------------------------------------------------
  set(_numerics_probe_document "${CMAKE_CURRENT_BINARY_DIR}/bike_numerics/probe_document.json")
  file(WRITE "${_numerics_probe_document}" "${_native_dependencies_json}\n")
  set(_numerics_check_tool "${CMAKE_CURRENT_LIST_DIR}/../../tools/native_dependencies.py")
  set(_numerics_check_command
    "${Python_EXECUTABLE} ${_numerics_check_tool} --check-numerics-record ${_numerics_record_file} --probe-document ${_numerics_probe_document}")
  execute_process(
    COMMAND "${Python_EXECUTABLE}" "${_numerics_check_tool}"
            --check-numerics-record "${_numerics_record_file}"
            --probe-document "${_numerics_probe_document}"
    RESULT_VARIABLE _numerics_check_result
    OUTPUT_VARIABLE _native_numerics_record_key
    ERROR_VARIABLE _numerics_check_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  # A verdict failure prints the checker's own message on stderr. A command that
  # cannot start (missing or non-executable interpreter) leaves stderr empty and
  # reports the reason in the result, so both fields and the command line are
  # named here.
  if(NOT _numerics_check_result EQUAL 0)
    message(FATAL_ERROR
      "numerics record check did not pass (result '${_numerics_check_result}') "
      "running: ${_numerics_check_command}\n${_numerics_check_error}")
  endif()
  if(NOT _native_numerics_record_key MATCHES "^[a-z]+-[A-Za-z0-9_]+$")
    message(FATAL_ERROR
      "numerics record check returned an invalid platform key "
      "'${_native_numerics_record_key}'")
  endif()
else()
  # The root graph has no probe, so the key is the host's platform identity,
  # spelled the way tools/native_dependencies.py record_platform_key spells it
  # (system.lower() + machine). The entry's own platform fields are checked
  # against the host below, so a hand-edited key cannot select another entry.
  # Only Darwin is qualified: a Linux key also needs the libc family, which
  # configure cannot name without a probe.
  if(NOT BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    message(FATAL_ERROR
      "record-only numerics (the Python-free root graph) is qualified for "
      "Darwin only; ${BIKE_TARGET_SYSTEM} needs the libc family and the NumPy "
      "install location, which configure cannot read without the live probe")
  endif()
  if(NOT EXISTS "${_numerics_record_file}")
    message(FATAL_ERROR
      "committed numerics record is missing: ${_numerics_record_file}")
  endif()
  string(TOLOWER "${BIKE_TARGET_SYSTEM}" _numerics_host_system)
  set(_native_numerics_record_key
    "${_numerics_host_system}-${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

file(READ "${_numerics_record_file}" _numerics_record_json)
string(JSON _numerics_record_type ERROR_VARIABLE _numerics_record_error
  TYPE "${_numerics_record_json}" records "${_native_numerics_record_key}" numerics)
if(_numerics_record_error OR NOT _numerics_record_type STREQUAL "OBJECT")
  message(FATAL_ERROR
    "committed numerics record has no entry '${_native_numerics_record_key}' "
    "(${_numerics_record_error}) — regenerate it with "
    "tools/native_dependencies.py --python <interpreter> "
    "--write-numerics-record ${_numerics_record_file}")
endif()

_numerics_record_field(_numerics_backend numerics backend)
_numerics_record_field(_numerics_blas_bits numerics blas_integer_bits)
_numerics_record_field(_numerics_lapack_bits numerics lapack_integer_bits)
_numerics_record_field(_numerics_numpy_version numerics numpy_version)
if(_numerics_is_live)
  _numerics_live_field(_numerics_site_packages numerics site_packages)
  message(STATUS
    "numerics record '${_native_numerics_record_key}' matches the live measurement")
else()
  # Record-only: the entry must describe this host. The key was derived from
  # these fields, so only a hand-edited record can differ; a Darwin entry must
  # carry a null libc, because the record stores a libc family only on Linux.
  _numerics_record_field(_numerics_record_system platform system)
  _numerics_record_field(_numerics_record_machine platform machine)
  string(JSON _numerics_record_libc_type ERROR_VARIABLE _numerics_json_error
    TYPE "${_numerics_record_json}" records "${_native_numerics_record_key}" platform libc)
  if(NOT "${_numerics_record_system}" STREQUAL "${BIKE_TARGET_SYSTEM}"
     OR NOT "${_numerics_record_machine}" STREQUAL "${CMAKE_HOST_SYSTEM_PROCESSOR}"
     OR NOT _numerics_record_libc_type STREQUAL "NULL")
    message(FATAL_ERROR
      "numerics record entry '${_native_numerics_record_key}' does not describe "
      "this host (${BIKE_TARGET_SYSTEM}/${CMAKE_HOST_SYSTEM_PROCESSOR}): it names "
      "${_numerics_record_system}/${_numerics_record_machine} with libc "
      "${_numerics_record_libc_type} in ${_numerics_record_file}")
  endif()
  message(STATUS
    "numerics record '${_native_numerics_record_key}' describes this host "
    "(record-only: facts are not compared with a live measurement)")
endif()

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
# Record-only binds only the Darwin Accelerate profile. A NUMPY_OPENBLAS entry
# needs the live NumPy install location to resolve its provider, and this mode
# has none.
if(NOT _numerics_is_live AND NOT _numerics_backend STREQUAL "ACCELERATE")
  message(FATAL_ERROR
    "record-only numerics binds only the qualified ACCELERATE profile; the "
    "record names '${_numerics_backend}', which needs the live probe")
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
foreach(_op ddot dgemv dgemm dgelsd)
  _numerics_record_field(_native_numerics_symbol_${_op} numerics symbols ${_op})
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
    "'${_numerics_interp_lowering}' — re-run tools/native_dependencies.py "
    "to re-measure the wheel's np.interp lowering")
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
    "'element', found '${_numerics_mul_nan_winner}' — re-run "
    "tools/native_dependencies.py to re-measure the wheel's broadcast "
    "multiply NaN precedence")
endif()
add_compile_definitions(
  BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=${_native_numerics_mul_element_nan})
message(STATUS
  "multiply broadcast NaN winner: ${_numerics_mul_nan_winner} "
  "(BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN="
  "${_native_numerics_mul_element_nan})")

# --------------------------------------------------------------------------
# Provider identity — roles must cover blas+lapack; the Linux file provider
# must exist inside the recorded site-packages and carry its SHA-256;
# Accelerate entries carry the apple-system identity with a real build id
# (dyld-cache images have no file to hash).
# The record stores Linux providers relative to site-packages, so they are
# resolved against the live install location here. Darwin paths are absolute
# system images and need no resolution. build_id is machine-specific and is
# read from the live document at the same index; the check above guarantees
# the two documents list the providers in the same order. RECORD_ONLY reads no
# build_id: it binds the provider by identity kind, and a machine-specific
# value has no committed source for the root graph.
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
    set(_lib_path "${_numerics_site_packages}/${_lib_path}")
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
    list(APPEND _native_numerics_provider_paths "${_lib_real}")
  else()
    if(NOT _lib_kind STREQUAL "apple-system")
      message(FATAL_ERROR
        "numerics provider '${_lib_path}' has identity '${_lib_kind}', "
        "not 'apple-system' — Darwin binds only the dyld-cache Accelerate")
    endif()
    if(_numerics_is_live)
      _numerics_live_field(_lib_build numerics library_paths ${_i} build_id)
      if(_lib_build STREQUAL "")
        message(FATAL_ERROR
          "numerics provider '${_lib_path}' has an empty build_id — the "
          "apple-system identity has no provider evidence")
      endif()
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
# mutates provider threading here. RECORD_ONLY creates the same target for the
# Darwin Accelerate profile: the framework is an apple-system image, so no probe
# is needed to name it. Linux is refused above in RECORD_ONLY, so the SHARED
# branch below is LIVE-only in practice.
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
# platform per interface (verified against the generated record width at
# compile time), strict ISO C++23, PIC so it can join the extension
# module, and the provider through PUBLIC usage requirements so
# Python-free harnesses receive the link too. The policy applied is
# BIKE_NUMERICS_APPLY_POLICY (default bike_apply_native_policy, BikeCompilerPolicy,
# T09). The mujoco include dir is measured by the live probe, or in RECORD_ONLY
# supplied by the includer as BIKE_NUMERICS_MUJOCO_INCLUDE — the blas backend TU
# reaches cblas_abi.hpp's mjtNum assertion through it.
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
    if(_numerics_is_live)
      _numerics_live_field(_numerics_mujoco_include mujoco include_dir)
    else()
      if(NOT DEFINED BIKE_NUMERICS_MUJOCO_INCLUDE
         OR "${BIKE_NUMERICS_MUJOCO_INCLUDE}" STREQUAL "")
        message(FATAL_ERROR
          "BikeNumerics.cmake RECORD_ONLY requires BIKE_NUMERICS_MUJOCO_INCLUDE "
          "(the MuJoCo include root of the locked dependency tree) to build "
          "bike_native_numerics")
      endif()
      set(_numerics_mujoco_include "${BIKE_NUMERICS_MUJOCO_INCLUDE}")
    endif()
    add_library(bike_native_numerics STATIC "${_numerics_backend_sources}")
    set_target_properties(bike_native_numerics PROPERTIES
      CXX_STANDARD 23
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
      POSITION_INDEPENDENT_CODE ON)
    # BIKE_MUJOCO_COMPAT_INCLUDE precedes the mujoco include dir only when
    # the toolchain needs the generated mjsan.h shadow under the ASan lane
    # (BikeDependencies.cmake); it is empty otherwise.
    target_include_directories(bike_native_numerics PRIVATE
      "${_native_numerics_symbols_dir}"
      ${BIKE_MUJOCO_COMPAT_INCLUDE}
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
