# migration_T53_numerics — BikeNumerics.cmake record/provider/ABI contracts.
#
# Batch 3.36b ports tests/portability/test_numerics_discovery.py. The file's
# subject is the numerics discovery contract: the committed record
# (native/numerics.lock.json), the integer-width profile, the provider-export
# spellings, the provider identity/path/sha256 rules, the libc family check,
# and the LIVE vs RECORD_ONLY consumption modes of BikeNumerics.cmake — all
# checked through scratch configures that include the real module, plus file
# invariants asserted directly against the committed lock.
#
# A dedicated fixture (not part of migration_T53_portability): the subject is
# exclusively BikeNumerics.cmake/numerics-record contracts, while
# migration_T53_portability covers platform/policy/root-configure tooling.
# The tranche-1 fixture also lives on lane/c, not this lane, so there is
# nothing here to extend.
#
# Python-side functions whose semantics stay interpreter-only (the
# validate_numerics live-evidence checks — threading, numpy_imports,
# provider_config, threading counts — plus require_same_library, the
# record writer/checker, the probe orchestration and the CLI plumbing)
# are dropped: they die with the interpreter at T63, and their committed
# contracts are exactly what the cases below exercise.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.

foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "migration_T53_numerics requires -D${_required}")
  endif()
endforeach()

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")

set(_case_count 0)
# Accumulators live on GLOBAL properties, not scope variables: _fail/_skip
# are called from inside _expect_* helper functions, and a PARENT_SCOPE set
# would only reach the helper's own frame — silently dropping the failure.
set_property(GLOBAL PROPERTY _t53n_failures "")
set_property(GLOBAL PROPERTY _t53n_skips "")

set(_host "${CMAKE_HOST_SYSTEM_NAME}")
# The record key is <system.lower()>-<CMAKE_HOST_SYSTEM_PROCESSOR> inside the
# scratch configure; in cmake -P that variable is unset, so uname(1) supplies
# the identical spelling (arm64 on Darwin, aarch64/x86_64 on Linux).
execute_process(COMMAND uname -m OUTPUT_VARIABLE _uname_m OUTPUT_STRIP_TRAILING_WHITESPACE)

set(NUMERICS_MODULE "${REPO_ROOT}/native/cmake/BikeNumerics.cmake")
set(IDENTITY_MODULE "${REPO_ROOT}/native/cmake/BikeProviderIdentity.cmake")
set(NUMERICS_LOCK "${REPO_ROOT}/native/numerics.lock.json")

set(ACCEL_FRAMEWORK "/System/Library/Frameworks/Accelerate.framework/Versions/A")
set(ACCEL_BLAS_IMAGE "${ACCEL_FRAMEWORK}/Frameworks/vecLib.framework/Versions/A/libBLAS.dylib")
set(ACCEL_LAPACK_IMAGE "${ACCEL_FRAMEWORK}/Frameworks/vecLib.framework/Versions/A/libLAPACK.dylib")

function(_fail msg)
  get_property(_f GLOBAL PROPERTY _t53n_failures)
  set_property(GLOBAL PROPERTY _t53n_failures "${_f}\n  ${msg}")
endfunction()

function(_skip msg)
  get_property(_s GLOBAL PROPERTY _t53n_skips)
  set_property(GLOBAL PROPERTY _t53n_skips "${_s}\n  skip: ${msg}")
  message(STATUS "skip: ${msg}")
endfunction()

# _run(name COMMAND <argv...> [ENV <cmake -E env args>]) -> ${name}_rc, ${name}_out
function(_run name)
  cmake_parse_arguments(R "" "" "COMMAND;ENV" ${ARGN})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${R_ENV} ${R_COMMAND}
    RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
  set(${name}_rc "${_rc}" PARENT_SCOPE)
  set(${name}_out "${_out}\n${_err}" PARENT_SCOPE)
endfunction()

function(_expect_needle case haystack needle)
  string(FIND "${haystack}" "${needle}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

# Whitespace-collapsed match — the port of the Python tests' squashed():
# CMake wraps long FATAL_ERROR text at 80 columns, so needles match on words.
function(_expect_squashed case haystack needle)
  string(REGEX REPLACE "[ \t\r\n]+" " " _h "${haystack}")
  string(REGEX REPLACE "[ \t\r\n]+" " " _n "${needle}")
  string(FIND "${_h}" "${_n}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

function(_expect_absent case haystack needle)
  string(FIND "${haystack}" "${needle}" _pos)
  if(NOT _pos EQUAL -1)
    _fail("${case}: unexpected '${needle}' present")
  endif()
endfunction()

function(_expect_equal case actual expected)
  if(NOT "${actual}" STREQUAL "${expected}")
    _fail("${case}: '${actual}' != '${expected}'")
  endif()
endfunction()

function(_write_exe path body)
  file(WRITE "${path}" "${body}")
  file(CHMOD "${path}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()

# ==========================================================================
# Committed-record fixtures — the schema-1 numerics lock shape
# ==========================================================================

# _write_lock(file key system machine libc_json numerics_json)
# libc_json is raw JSON: "glibc" as "\"glibc\"", Darwin as null.
function(_write_lock file key system machine libc_json numerics_json)
  file(WRITE "${file}"
"{\"schema_version\":1,\"records\":{\"${key}\":{\"platform\":{\"system\":\"${system}\",\"machine\":\"${machine}\",\"libc\":${libc_json}},\"numerics\":${numerics_json}}}}\n")
endfunction()

# _lock_set(file key value_json member...) — typed JSON replacement of
# records[key].<member...>; value_json is a raw JSON token.
function(_lock_set file key value_json)
  file(READ "${file}" _j)
  string(JSON _new SET "${_j}" records "${key}" ${ARGN} "${value_json}")
  file(WRITE "${file}" "${_new}")
endfunction()

# _lock_remove(file key member...) — delete records[key].<member...>.
function(_lock_remove file key)
  file(READ "${file}" _j)
  string(JSON _new REMOVE "${_j}" records "${key}" ${ARGN})
  file(WRITE "${file}" "${_new}")
endfunction()

# _json_escape(raw outvar) — encode a raw string as a JSON string token, so
# hostile spellings land inside the record exactly as the Python vectors do.
function(_json_escape raw outvar)
  string(REPLACE "\\" "\\\\" _e "${raw}")
  string(REPLACE "\"" "\\\"" _e "${_e}")
  string(REPLACE "
" "\\n" _e "${_e}")
  string(REPLACE "	" "\\t" _e "${_e}")
  set(${outvar} "\"${_e}\"" PARENT_SCOPE)
endfunction()

# A valid NUMPY_OPENBLAS committed entry: 64/64 ILP64, scipy_*_64_ exports,
# one wheel-relative file provider, all lowerings separate, element NaN.
function(_openblas_numerics outvar provider_rel sha)
  set(${outvar}
"{\"backend\":\"NUMPY_OPENBLAS\",\"blas_integer_bits\":64,\"lapack_integer_bits\":64,\"numpy_version\":\"2.5.2\",\"symbols\":{\"ddot\":\"scipy_cblas_ddot64_\",\"dgemv\":\"scipy_cblas_dgemv64_\",\"dgemm\":\"scipy_cblas_dgemm64_\",\"dgelsd\":\"scipy_dgelsd_64_\",\"dgesv\":\"scipy_dgesv_64_\",\"dsyevd\":\"scipy_dsyevd_64_\",\"ddot_ilp64\":\"scipy_cblas_ddot64_\",\"zdotu_sub\":\"scipy_cblas_zdotu_sub64_\"},\"library_paths\":[{\"path\":\"${provider_rel}\",\"role\":\"blas_lapack\",\"identity_kind\":\"file\",\"sha256\":\"${sha}\"}],\"interp\":{\"lowering\":\"separate\"},\"multiply_broadcast\":{\"nan_winner\":\"element\"},\"distributions\":{\"lowering\":\"separate\"},\"scipy_correlate\":{\"lowering\":\"separate\"}}"
    PARENT_SCOPE)
endfunction()

# A valid ACCELERATE committed entry: the 32/64 split, legacy CBLAS plus
# NEWLAPACK ILP64 exports, two apple-system dyld images, contracted/scalar.
function(_accelerate_numerics outvar)
  set(${outvar}
"{\"backend\":\"ACCELERATE\",\"blas_integer_bits\":32,\"lapack_integer_bits\":64,\"numpy_version\":\"2.5.2\",\"symbols\":{\"ddot\":\"cblas_ddot\",\"dgemv\":\"cblas_dgemv\",\"dgemm\":\"cblas_dgemm\",\"dgelsd\":\"dgelsd\$NEWLAPACK\$ILP64\",\"dgesv\":\"dgesv\$NEWLAPACK\$ILP64\",\"dsyevd\":\"dsyevd\$NEWLAPACK\$ILP64\",\"ddot_ilp64\":\"cblas_ddot\$NEWLAPACK\$ILP64\",\"zdotu_sub\":\"cblas_zdotu_sub\$NEWLAPACK\$ILP64\"},\"library_paths\":[{\"path\":\"${ACCEL_BLAS_IMAGE}\",\"role\":\"blas\",\"identity_kind\":\"apple-system\",\"sha256\":null},{\"path\":\"${ACCEL_LAPACK_IMAGE}\",\"role\":\"lapack\",\"identity_kind\":\"apple-system\",\"sha256\":null}],\"interp\":{\"lowering\":\"contracted\"},\"multiply_broadcast\":{\"nan_winner\":\"scalar\"},\"distributions\":{\"lowering\":\"contracted\"},\"scipy_correlate\":{\"lowering\":\"contracted\"}}"
    PARENT_SCOPE)
endfunction()

# ==========================================================================
# Scratch-configure harnesses — the real BikeNumerics.cmake under both modes
# ==========================================================================

# _ro_source(dir system mode extra) — RECORD_ONLY scratch project.
# `mode` is "set(BIKE_NUMERICS_SOURCE RECORD_ONLY)\n" or "" (mode-forgotten
# case); `extra` carries the Linux inputs (BIKE_TARGET_GLIBC /
# BIKE_DEP_numpy_openblas_{ROOT,LIBRARY}).
function(_ro_source dir system mode extra)
  file(MAKE_DIRECTORY "${dir}")
  file(WRITE "${dir}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
project(numerics_ro NONE)
set(BIKE_TARGET_SYSTEM \"${system}\")
${mode}${extra}include(\"${NUMERICS_MODULE}\")
message(STATUS \"RESOLVED=\${BIKE_NUMERICS_BACKEND_RESOLVED}\")
message(STATUS \"KEY=\${_native_numerics_record_key}\")
get_directory_property(_defs COMPILE_DEFINITIONS)
message(STATUS \"DEFS=\${_defs}\")
get_target_property(_provider_type Bike::NumericsProvider TYPE)
message(STATUS \"PROVIDER_TYPE=\${_provider_type}\")
if(_provider_type STREQUAL \"SHARED_LIBRARY\")
  get_target_property(_provider_path Bike::NumericsProvider IMPORTED_LOCATION)
  message(STATUS \"PROVIDER_PATH=\${_provider_path}\")
endif()
")
endfunction()

# _live_source(dir system checker doc extra) — LIVE scratch project; checker
# is the Python_EXECUTABLE stand-in (a stub printing the record key).
function(_live_source dir system checker doc extra)
  file(MAKE_DIRECTORY "${dir}")
  file(WRITE "${dir}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
project(numerics_live NONE)
set(BIKE_TARGET_SYSTEM \"${system}\")
set(BIKE_NUMERICS_SOURCE LIVE)
set(Python_EXECUTABLE \"${checker}\")
file(READ \"${doc}\" _native_dependencies_json)
${extra}include(\"${NUMERICS_MODULE}\")
message(STATUS \"RESOLVED=\${BIKE_NUMERICS_BACKEND_RESOLVED}\")
message(STATUS \"KEY=\${_native_numerics_record_key}\")
message(STATUS \"DDOT=\${_native_numerics_symbol_ddot}\")
message(STATUS \"DGELSD=\${_native_numerics_symbol_dgelsd}\")
get_target_property(_provider_type Bike::NumericsProvider TYPE)
message(STATUS \"PROVIDER_TYPE=\${_provider_type}\")
if(_provider_type STREQUAL \"SHARED_LIBRARY\")
  get_target_property(_provider_path Bike::NumericsProvider IMPORTED_LOCATION)
  message(STATUS \"PROVIDER_PATH=\${_provider_path}\")
endif()
")
endfunction()

# Logging stubs for every interpreter name — the record-only boundary must
# never spawn one. _shim_env(dir log outvar) writes them and returns the
# leading PATH entry list for _run ENV.
function(_write_shims dir log)
  file(MAKE_DIRECTORY "${dir}")
  foreach(_n python python3 python3.13 python3.14 python3.15 uv)
    _write_exe("${dir}/${_n}"
      "#!/bin/sh\necho \"${_n} \$*\" >> \"${log}\"\nexit 97\n")
  endforeach()
endfunction()

# _ro_case(name) — RECORD_ONLY configure.
#   SYSTEM <s>            BIKE_TARGET_SYSTEM (default: record key's system)
#   KEY <k>               record key (default <system-lower>-<uname_m>)
#   RECORD_FILE <f>       committed record (required)
#   MODE <m>              RECORD_ONLY or FORGOT
#   GLIBC <ON|OFF>        BIKE_TARGET_GLIBC
#   DEPS_ROOT <d>         BIKE_DEP_numpy_openblas_ROOT
#   DEPS_LIB <f>          BIKE_DEP_numpy_openblas_LIBRARY
#   EXTRA <text>          extra CMake lines
#   BACKEND <b>           -DBIKE_NUMERICS_BACKEND override
# Returns ${name}_rc ${name}_out; scratch at ${SCRATCH_DIR}/ro-<name>.
function(_ro_case name)
  cmake_parse_arguments(R "" "SYSTEM;KEY;RECORD_FILE;MODE;GLIBC;DEPS_ROOT;DEPS_LIB;EXTRA;BACKEND" "" ${ARGN})
  if(NOT DEFINED R_SYSTEM)
    set(R_SYSTEM "${_host}")
  endif()
  if(NOT DEFINED R_KEY)
    string(TOLOWER "${R_SYSTEM}" _ls)
    set(R_KEY "${_ls}-${_uname_m}")
  endif()
  if(NOT DEFINED R_MODE OR R_MODE STREQUAL "RECORD_ONLY")
    set(_mode "set(BIKE_NUMERICS_SOURCE RECORD_ONLY)\n")
  else()
    set(_mode "")
  endif()
  set(_linux "")
  if(DEFINED R_GLIBC)
    set(_linux "${_linux}set(BIKE_TARGET_GLIBC ${R_GLIBC})\n")
  endif()
  if(DEFINED R_DEPS_ROOT)
    set(_linux "${_linux}set(BIKE_DEP_numpy_openblas_ROOT \"${R_DEPS_ROOT}\")\n")
  endif()
  if(DEFINED R_DEPS_LIB)
    set(_linux "${_linux}set(BIKE_DEP_numpy_openblas_LIBRARY \"${R_DEPS_LIB}\")\n")
  endif()
  set(_src "${SCRATCH_DIR}/ro-${name}/src")
  _ro_source("${_src}" "${R_SYSTEM}" "${_mode}" "${_linux}${R_EXTRA}")
  set(_shimlog "${SCRATCH_DIR}/ro-${name}/python-calls.log")
  _write_shims("${SCRATCH_DIR}/ro-${name}/shims" "${_shimlog}")
  set(_args "")
  if(DEFINED R_BACKEND)
    set(_args "-DBIKE_NUMERICS_BACKEND=${R_BACKEND}")
  endif()
  _run(${name}
    ENV "PATH=${SCRATCH_DIR}/ro-${name}/shims:/usr/bin:/bin"
    COMMAND "${CMAKE_COMMAND}" -S "${_src}" -B "${SCRATCH_DIR}/ro-${name}/b"
            "-DBIKE_NUMERICS_RECORD=${R_RECORD_FILE}" ${_args})
  set(${name}_rc "${${name}_rc}" PARENT_SCOPE)
  set(${name}_out "${${name}_out}" PARENT_SCOPE)
  set(${name}_shimlog "${_shimlog}" PARENT_SCOPE)
  set(${name}_build "${SCRATCH_DIR}/ro-${name}/b" PARENT_SCOPE)
endfunction()

# _live_case(name) — LIVE configure with a stub checker.
#   SYSTEM <s>, KEY <k> (the key the stub echoes as its verdict; must equal
#   the record key the lock was written with — darwin/linux-${_uname_m}),
#   RECORD_FILE, DOC <f>
#   CHECKER <path> (default: generated stub echoing KEY, exit 0)
#   BACKEND <b>, EXTRA <text>
function(_live_case name)
  cmake_parse_arguments(R "" "SYSTEM;KEY;RECORD_FILE;DOC;CHECKER;BACKEND;EXTRA" "" ${ARGN})
  if(NOT DEFINED R_CHECKER)
    set(R_CHECKER "${SCRATCH_DIR}/live-${name}/stub-checker")
    _write_exe("${R_CHECKER}" "#!/bin/sh\necho ${R_KEY}\n")
  endif()
  _live_source("${SCRATCH_DIR}/live-${name}/src" "${R_SYSTEM}" "${R_CHECKER}" "${R_DOC}" "${R_EXTRA}")
  set(_args "")
  if(DEFINED R_BACKEND)
    set(_args "-DBIKE_NUMERICS_BACKEND=${R_BACKEND}")
  endif()
  _run(${name}
    COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/live-${name}/src"
            -B "${SCRATCH_DIR}/live-${name}/b"
            "-DBIKE_NUMERICS_RECORD=${R_RECORD_FILE}" ${_args})
  set(${name}_rc "${${name}_rc}" PARENT_SCOPE)
  set(${name}_out "${${name}_out}" PARENT_SCOPE)
endfunction()

# _deps_provider(tag name) — a fake acquired-tree provider under
# ${SCRATCH_DIR}/<tag>-deps/lib/<name>; sets ${tag}_root/_lib/_sha.
function(_deps_provider tag name)
  set(_root "${SCRATCH_DIR}/${tag}-deps")
  file(MAKE_DIRECTORY "${_root}/lib")
  file(WRITE "${_root}/lib/${name}" "fake ilp64 openblas provider image for ${tag}\n")
  file(SHA256 "${_root}/lib/${name}" _sha)
  set(${tag}_root "${_root}" PARENT_SCOPE)
  set(${tag}_lib "${_root}/lib/${name}" PARENT_SCOPE)
  set(${tag}_sha "${_sha}" PARENT_SCOPE)
endfunction()

# _linux_record(tag root lib sha) — a committed lock keyed linux-<uname_m>
# naming numpy.libs/<basename(lib)> with the given digest.
function(_linux_record tag root lib sha)
  get_filename_component(_libname "${lib}" NAME)
  _openblas_numerics(_n "numpy.libs/${_libname}" "${sha}")
  _write_lock("${SCRATCH_DIR}/${tag}-lock.json" "linux-${_uname_m}"
    "Linux" "${_uname_m}" "\"glibc\"" "${_n}")
endfunction()

# _darwin_record(tag) — a committed lock keyed darwin-<uname_m> with the
# Accelerate committed entry.
function(_darwin_record tag)
  _accelerate_numerics(_n)
  _write_lock("${SCRATCH_DIR}/${tag}-lock.json" "darwin-${_uname_m}"
    "Darwin" "${_uname_m}" "null" "${_n}")
endfunction()

# ==========================================================================
# Section 1 — committed-lock file invariants (the real numerics.lock.json)
# ==========================================================================
file(READ "${NUMERICS_LOCK}" _lock)

# test_numerics_lock_rejects_an_unknown_schema_version — the committed lock is
# schema_version 1 (the reader-side rejection is writer/checker code; the
# native invariant is that the committed file stays on the qualified schema).
set(_c "committed_lock_schema_is_v1")
math(EXPR _case_count "${_case_count} + 1")
string(JSON _sv GET "${_lock}" schema_version)
_expect_equal(${_c} "${_sv}" "1")

# test_committed_numerics_lock_is_a_valid_darwin_record +
# test_record_projection_keeps_only_the_committed_facts: the darwin entry
# carries exactly the committed facts, in the committed shape.
set(_c "committed_lock_darwin_record_facts")
math(EXPR _case_count "${_case_count} + 1")
string(JSON _nk LENGTH "${_lock}" records darwin-arm64)
_expect_equal(${_c} "${_nk}" "2")
string(JSON _pk LENGTH "${_lock}" records darwin-arm64 platform)
_expect_equal(${_c} "${_pk}" "3")
string(JSON _v GET "${_lock}" records darwin-arm64 platform system)
_expect_equal(${_c} "${_v}" "Darwin")
string(JSON _v GET "${_lock}" records darwin-arm64 platform machine)
_expect_equal(${_c} "${_v}" "arm64")
string(JSON _t TYPE "${_lock}" records darwin-arm64 platform libc)
_expect_equal(${_c} "${_t}" "NULL")
string(JSON _nk LENGTH "${_lock}" records darwin-arm64 numerics)
_expect_equal(${_c} "${_nk}" "10")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics backend)
_expect_equal(${_c} "${_v}" "ACCELERATE")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics blas_integer_bits)
_expect_equal(${_c} "${_v}" "32")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics lapack_integer_bits)
_expect_equal(${_c} "${_v}" "64")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics numpy_version)
_expect_equal(${_c} "${_v}" "2.5.2")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics interp lowering)
_expect_equal(${_c} "${_v}" "contracted")
string(JSON _ik LENGTH "${_lock}" records darwin-arm64 numerics interp)
_expect_equal(${_c} "${_ik}" "1")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics multiply_broadcast nan_winner)
_expect_equal(${_c} "${_v}" "scalar")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics distributions lowering)
_expect_equal(${_c} "${_v}" "contracted")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics scipy_correlate lowering)
_expect_equal(${_c} "${_v}" "contracted")
string(JSON _ns LENGTH "${_lock}" records darwin-arm64 numerics symbols)
_expect_equal(${_c} "${_ns}" "8")
# test_accelerate_legacy_dot_keeps_its_separate_32_bit_contract: the raw dot
# keeps the legacy 32-bit interface while the ILP64 op spells NEWLAPACK.
string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ddot)
_expect_equal(${_c} "${_v}" "cblas_ddot")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ddot_ilp64)
_expect_equal(${_c} "${_v}" "cblas_ddot\$NEWLAPACK\$ILP64")
foreach(_p dgemv dgemm)
  string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ${_p})
  _expect_equal(${_c} "${_v}" "cblas_${_p}")
endforeach()
foreach(_p dgelsd dgesv dsyevd)
  string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ${_p})
  _expect_equal(${_c} "${_v}" "${_p}\$NEWLAPACK\$ILP64")
endforeach()
string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols zdotu_sub)
_expect_equal(${_c} "${_v}" "cblas_zdotu_sub\$NEWLAPACK\$ILP64")
# apple-system identity: absolute Accelerate.framework images, sha256 null,
# no build_id member, blas+lapack roles.
string(JSON _nl LENGTH "${_lock}" records darwin-arm64 numerics library_paths)
_expect_equal(${_c} "${_nl}" "2")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics library_paths 0 role)
_expect_equal(${_c} "${_v}" "blas")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics library_paths 1 role)
_expect_equal(${_c} "${_v}" "lapack")
foreach(_i 0 1)
  string(JSON _ek LENGTH "${_lock}" records darwin-arm64 numerics library_paths ${_i})
  _expect_equal(${_c} "${_ek}" "4")
  string(JSON _v GET "${_lock}" records darwin-arm64 numerics library_paths ${_i} identity_kind)
  _expect_equal(${_c} "${_v}" "apple-system")
  string(JSON _t TYPE "${_lock}" records darwin-arm64 numerics library_paths ${_i} sha256)
  _expect_equal(${_c} "${_t}" "NULL")
  # build_id is a live value, never committed: the member must be absent.
  string(JSON _bt ERROR_VARIABLE _be GET "${_lock}"
    records darwin-arm64 numerics library_paths ${_i} build_id)
  if(NOT _be)
    _fail("${_c}: darwin library_paths[${_i}] carries a committed build_id")
  endif()
  string(JSON _v GET "${_lock}" records darwin-arm64 numerics library_paths ${_i} path)
  _expect_needle(${_c} "${_v}" "/System/Library/Frameworks/Accelerate.framework/")
endforeach()

# The linux entry — same committed shape, wheel-relative file provider, the
# libc family (never the version), separate lowerings.
set(_c "committed_lock_linux_record_facts")
math(EXPR _case_count "${_case_count} + 1")
string(JSON _nk LENGTH "${_lock}" records linux-aarch64)
_expect_equal(${_c} "${_nk}" "2")
string(JSON _v GET "${_lock}" records linux-aarch64 platform system)
_expect_equal(${_c} "${_v}" "Linux")
string(JSON _v GET "${_lock}" records linux-aarch64 platform machine)
_expect_equal(${_c} "${_v}" "aarch64")
string(JSON _v GET "${_lock}" records linux-aarch64 platform libc)
_expect_equal(${_c} "${_v}" "glibc")
string(JSON _nk LENGTH "${_lock}" records linux-aarch64 numerics)
_expect_equal(${_c} "${_nk}" "10")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics backend)
_expect_equal(${_c} "${_v}" "NUMPY_OPENBLAS")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics blas_integer_bits)
_expect_equal(${_c} "${_v}" "64")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics lapack_integer_bits)
_expect_equal(${_c} "${_v}" "64")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics numpy_version)
_expect_equal(${_c} "${_v}" "2.5.2")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics interp lowering)
_expect_equal(${_c} "${_v}" "separate")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics multiply_broadcast nan_winner)
_expect_equal(${_c} "${_v}" "element")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics distributions lowering)
_expect_equal(${_c} "${_v}" "separate")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics scipy_correlate lowering)
_expect_equal(${_c} "${_v}" "separate")
string(JSON _ns LENGTH "${_lock}" records linux-aarch64 numerics symbols)
_expect_equal(${_c} "${_ns}" "8")
foreach(_p ddot ddot_ilp64)
  string(JSON _v GET "${_lock}" records linux-aarch64 numerics symbols ${_p})
  _expect_equal(${_c} "${_v}" "scipy_cblas_ddot64_")
endforeach()
foreach(_p dgemv dgemm)
  string(JSON _v GET "${_lock}" records linux-aarch64 numerics symbols ${_p})
  _expect_equal(${_c} "${_v}" "scipy_cblas_${_p}64_")
endforeach()
foreach(_p dgelsd dgesv dsyevd)
  string(JSON _v GET "${_lock}" records linux-aarch64 numerics symbols ${_p})
  _expect_equal(${_c} "${_v}" "scipy_${_p}_64_")
endforeach()
string(JSON _v GET "${_lock}" records linux-aarch64 numerics symbols zdotu_sub)
_expect_equal(${_c} "${_v}" "scipy_cblas_zdotu_sub64_")
# test_linux_record_stores_wheel_relative_provider_paths: a wheel-relative
# numpy.libs/<file> path, file identity, 64-hex sha, no build_id member.
string(JSON _nl LENGTH "${_lock}" records linux-aarch64 numerics library_paths)
_expect_equal(${_c} "${_nl}" "1")
string(JSON _ek LENGTH "${_lock}" records linux-aarch64 numerics library_paths 0)
_expect_equal(${_c} "${_ek}" "4")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics library_paths 0 path)
if(NOT "${_v}" MATCHES "^numpy\\.libs/[^/]+$")
  _fail("${_c}: linux provider path is not wheel-relative: '${_v}'")
endif()
string(JSON _v GET "${_lock}" records linux-aarch64 numerics library_paths 0 role)
_expect_equal(${_c} "${_v}" "blas_lapack")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics library_paths 0 identity_kind)
_expect_equal(${_c} "${_v}" "file")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics library_paths 0 sha256)
if(NOT "${_v}" MATCHES "^[0-9a-f]+$")
  _fail("${_c}: linux provider sha256 is not lowercase hex: '${_v}'")
endif()
string(LENGTH "${_v}" _len)
_expect_equal(${_c} "${_len}" "64")
string(JSON _bt ERROR_VARIABLE _be GET "${_lock}"
  records linux-aarch64 numerics library_paths 0 build_id)
if(NOT _be)
  _fail("${_c}: linux library_paths[0] carries a committed build_id")
endif()

# test_legacy_identity_header_takes_its_facts_from_the_record — the identity
# declarations reproduce the committed record's role/identity_kind/sha256
# (path is the bound real path; build_id is the live value, stubbed here).
set(_c "identity_declarations_take_facts_from_committed_record")
math(EXPR _case_count "${_case_count} + 1")
set(_synthetic_id "synthetic-build-id-336b")
foreach(_key darwin-arm64 linux-aarch64)
  string(JSON _backend GET "${_lock}" records ${_key} numerics backend)
  string(JSON _bb GET "${_lock}" records ${_key} numerics blas_integer_bits)
  string(JSON _lb GET "${_lock}" records ${_key} numerics lapack_integer_bits)
  if(_backend STREQUAL "ACCELERATE")
    string(JSON _p0 GET "${_lock}" records ${_key} numerics library_paths 0 path)
    string(JSON _p1 GET "${_lock}" records ${_key} numerics library_paths 1 path)
    set(_paths "\"${_p0}\" \"${_p1}\"")
    set(_ids "\"${_synthetic_id}\" \"${_synthetic_id}\"")
  else()
    set(_bound "${SCRATCH_DIR}/bound/libscipy_openblas64_-fixture.so")
    set(_paths "\"${_bound}\"")
    set(_ids "\"<none>\"")
  endif()
  set(_hf "${SCRATCH_DIR}/identity-${_key}.cmake")
  set(_of "${SCRATCH_DIR}/identity-${_key}.txt")
  file(WRITE "${_hf}"
"include(\"${IDENTITY_MODULE}\")
file(READ \"${NUMERICS_LOCK}\" _record_json)
set(_paths ${_paths})
set(_ids ${_ids})
bike_provider_identity_declarations(_decl
  RECORD_VAR _record_json RECORD_KEY \"${_key}\"
  BACKEND \"${_backend}\" BLAS_BITS ${_bb} LAPACK_BITS ${_lb}
  PROVIDER_PATHS_VAR _paths BUILD_IDS_VAR _ids)
file(WRITE \"${_of}\" \"\${_decl}\")
")
  _run(id-${_key} COMMAND "${CMAKE_COMMAND}" -P "${_hf}")
  if(NOT id-${_key}_rc EQUAL 0)
    _fail("${_c}[${_key}]: declarations harness failed: ${id-${_key}_out}")
  else()
    file(READ "${_of}" _decl)
    string(REGEX MATCH "numerics_provider_policy_json\\[\\] = R\"identity\\(([^)]*)\\)identity\";" _m "${_decl}")
    if(NOT _m)
      _fail("${_c}[${_key}]: policy line not generated")
    else()
      set(_policy "${CMAKE_MATCH_1}")
      string(JSON _v GET "${_policy}" backend)
      _expect_equal("${_c}[${_key}]" "${_v}" "${_backend}")
      string(JSON _v GET "${_policy}" blas_integer_bits)
      _expect_equal("${_c}[${_key}]" "${_v}" "${_bb}")
      string(JSON _v GET "${_policy}" lapack_integer_bits)
      _expect_equal("${_c}[${_key}]" "${_v}" "${_lb}")
      string(JSON _nlib LENGTH "${_lock}" records ${_key} numerics library_paths)
      string(JSON _npl LENGTH "${_policy}" libraries)
      _expect_equal("${_c}[${_key}]" "${_npl}" "${_nlib}")
      math(EXPR _last "${_nlib} - 1")
      foreach(_i RANGE ${_last})
        string(JSON _role GET "${_lock}" records ${_key} numerics library_paths ${_i} role)
        string(JSON _kind GET "${_lock}" records ${_key} numerics library_paths ${_i} identity_kind)
        string(JSON _v GET "${_policy}" libraries ${_i} role)
        _expect_equal("${_c}[${_key}]" "${_v}" "${_role}")
        string(JSON _v GET "${_policy}" libraries ${_i} identity_kind)
        _expect_equal("${_c}[${_key}]" "${_v}" "${_kind}")
        string(JSON _v GET "${_policy}" libraries ${_i} path)
        if(_kind STREQUAL "apple-system")
          string(JSON _rp GET "${_lock}" records ${_key} numerics library_paths ${_i} path)
          _expect_equal("${_c}[${_key}]" "${_v}" "${_rp}")
          string(JSON _v GET "${_policy}" libraries ${_i} build_id)
          _expect_equal("${_c}[${_key}]" "${_v}" "${_synthetic_id}")
          string(JSON _t TYPE "${_policy}" libraries ${_i} sha256)
          _expect_equal("${_c}[${_key}]" "${_t}" "NULL")
        else()
          _expect_equal("${_c}[${_key}]" "${_v}" "${_bound}")
          if(NOT IS_ABSOLUTE "${_v}")
            _fail("${_c}[${_key}]: bound provider path is not absolute")
          endif()
          string(JSON _t TYPE "${_policy}" libraries ${_i} build_id)
          _expect_equal("${_c}[${_key}]" "${_t}" "NULL")
          string(JSON _v GET "${_lock}" records ${_key} numerics library_paths ${_i} sha256)
          string(JSON _sv GET "${_policy}" libraries ${_i} sha256)
          _expect_equal("${_c}[${_key}]" "${_sv}" "${_v}")
        endif()
      endforeach()
    endif()
  endif()
endforeach()

# ==========================================================================
# Section 2 — RECORD_ONLY scratch configures (the Python-free root graph)
# ==========================================================================

# test_record_only_requires_an_explicit_mode
set(_c "record_only_requires_an_explicit_mode")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-mode)
_ro_case(${_c} SYSTEM Darwin MODE FORGOT RECORD_FILE "${SCRATCH_DIR}/ro-mode-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "requires BIKE_NUMERICS_SOURCE=LIVE")

# test_record_only_binds_the_committed_facts_without_python —
# Darwin positive: facts from the record alone, the four defines projected,
# no interpreter spawned, generated header carries the Mach-O spellings.
set(_c "record_only_darwin_binds_committed_facts")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-darwin)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-darwin-lock.json")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "RESOLVED=ACCELERATE")
  _expect_needle(${_c} "${${_c}_out}" "KEY=darwin-${_uname_m}")
  _expect_needle(${_c} "${${_c}_out}" "bike_native_numerics not created: CXX is not enabled")
  _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_INTERP_CONTRACTED=1")
  _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=0")
  _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED=1")
  _expect_needle(${_c} "${${_c}_out}" "BIKE_SCIPY_CORRELATE_CONTRACTED=1")
  _expect_needle(${_c} "${${_c}_out}" "PROVIDER_TYPE=INTERFACE_LIBRARY")
  if(EXISTS "${${_c}_shimlog}")
    file(READ "${${_c}_shimlog}" _calls)
    _fail("${_c}: interpreter spawned during record-only configure: ${_calls}")
  endif()
  set(_hdr "${${_c}_build}/bike_numerics/numerics_symbols.hpp")
  if(NOT EXISTS "${_hdr}")
    _fail("${_c}: generated header missing")
  else()
    file(READ "${_hdr}" _h)
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_BACKEND \"ACCELERATE\"")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_BLAS_BITS 32")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_LAPACK_BITS 64")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_SYM_DDOT \"_cblas_ddot\"")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_SYM_DGELSD \"_dgelsd\$NEWLAPACK\$ILP64\"")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_SYM_DDOT_ILP64 \"_cblas_ddot\$NEWLAPACK\$ILP64\"")
    _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_SYM_ZDOTU_SUB \"_cblas_zdotu_sub\$NEWLAPACK\$ILP64\"")
  endif()
endif()

# test_record_only_linux_binds_acquired_provider_without_python (2 vectors:
# plain root, symlinked root) + test_valid_ilp64_openblas_record_is_accepted.
foreach(_symlinked IN ITEMS OFF ON)
  set(_c "record_only_linux_binds_acquired_provider[${_symlinked}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider(ro-lin-${_symlinked} "libfixture-openblas.so")
  set(_root "${ro-lin-${_symlinked}_root}")
  set(_lib "${ro-lin-${_symlinked}_lib}")
  set(_sha "${ro-lin-${_symlinked}_sha}")
  if(_symlinked)
    set(_real "${SCRATCH_DIR}/ro-lin-real-${_symlinked}")
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${_root}" "${_real}")
    file(REMOVE_RECURSE "${_root}")
    file(CREATE_LINK "${_real}" "${_root}" SYMBOLIC)
  endif()
  _linux_record(ro-lin-${_symlinked} "${_root}" "${_lib}" "${_sha}")
  _ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-lin-${_symlinked}-lock.json"
           GLIBC ON DEPS_ROOT "${_root}" DEPS_LIB "${_lib}")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  else()
    _expect_needle(${_c} "${${_c}_out}" "RESOLVED=NUMPY_OPENBLAS")
    _expect_needle(${_c} "${${_c}_out}" "KEY=linux-${_uname_m}")
    _expect_needle(${_c} "${${_c}_out}" "PROVIDER_TYPE=SHARED_LIBRARY")
    get_filename_component(_lib_real "${_lib}" REALPATH)
    _expect_needle(${_c} "${${_c}_out}" "PROVIDER_PATH=${_lib_real}")
    _expect_needle(${_c} "${${_c}_out}" "bike_native_numerics not created: CXX is not enabled")
    _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_INTERP_CONTRACTED=0")
    _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=1")
    _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED=0")
    _expect_needle(${_c} "${${_c}_out}" "BIKE_SCIPY_CORRELATE_CONTRACTED=0")
    if(EXISTS "${${_c}_shimlog}")
      file(READ "${${_c}_shimlog}" _calls)
      _fail("${_c}: interpreter spawned: ${_calls}")
    endif()
    set(_hdr "${${_c}_build}/bike_numerics/numerics_symbols.hpp")
    if(NOT EXISTS "${_hdr}")
      _fail("${_c}: generated header missing")
    else()
      file(READ "${_hdr}" _h)
      _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_BACKEND \"NUMPY_OPENBLAS\"")
      _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_BLAS_BITS 64")
      _expect_needle(${_c} "${_h}" "#define BIKE_NUMERICS_LAPACK_BITS 64")
      foreach(_op_sym "DDOT|scipy_cblas_ddot64_" "DGEMV|scipy_cblas_dgemv64_"
                      "DGEMM|scipy_cblas_dgemm64_" "DGELSD|scipy_dgelsd_64_"
                      "DGESV|scipy_dgesv_64_" "DSYEVD|scipy_dsyevd_64_"
                      "DDOT_ILP64|scipy_cblas_ddot64_" "ZDOTU_SUB|scipy_cblas_zdotu_sub64_")
        string(REGEX MATCH "([^|]+)\\|(.*)" _mm "${_op_sym}")
        _expect_needle(${_c} "${_h}"
          "#define BIKE_NUMERICS_SYM_${CMAKE_MATCH_1} \"${CMAKE_MATCH_2}\"")
      endforeach()
    endif()
  endif()
endforeach()

# test_record_only_linux_requires_the_acquired_provider (2 vectors)
foreach(_missing IN ITEMS root library)
  set(_c "record_only_linux_requires_acquired_provider[${_missing}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider(ro-miss-${_missing} "libfixture-openblas.so")
  set(_root "${ro-miss-${_missing}_root}")
  set(_lib "${ro-miss-${_missing}_lib}")
  set(_sha "${ro-miss-${_missing}_sha}")
  _linux_record(ro-miss-${_missing} "${_root}" "${_lib}" "${_sha}")
  if(_missing STREQUAL "root")
    file(REMOVE_RECURSE "${_root}")
  else()
    file(REMOVE "${_lib}")
  endif()
  _ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-miss-${_missing}-lock.json"
           GLIBC ON DEPS_ROOT "${_root}" DEPS_LIB "${_lib}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}"
    "record-only NUMPY_OPENBLAS requires the lock-verified Linux numpy_openblas dependency")
endforeach()

# test_record_only_linux_requires_matching_glibc +
# test_libc_family_change_is_rejected + test_unknown_libc_family_fails_closed
# + test_record_rejects_a_libc_version_in_the_committed_entry:
# a committed libc that is not exactly the family 'glibc' (or a glibc probe
# that says the host is not glibc) fails closed.
foreach(_pair "glibc|OFF" "musl|ON" "NULL|ON" "glibc 2.39|ON")
  string(REGEX MATCH "([^|]+)\\|(.*)" _mm "${_pair}")
  set(_libc "${CMAKE_MATCH_1}")
  set(_glibc "${CMAKE_MATCH_2}")
  set(_c "record_only_linux_requires_matching_glibc[${_libc}-${_glibc}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider(ro-libc-${_libc}-${_glibc} "libfixture-openblas.so")
  _linux_record(ro-libc-${_libc}-${_glibc}
    "${ro-libc-${_libc}-${_glibc}_root}" "${ro-libc-${_libc}-${_glibc}_lib}"
    "${ro-libc-${_libc}-${_glibc}_sha}")
  if(_libc STREQUAL "NULL")
    _lock_set("${SCRATCH_DIR}/ro-libc-${_libc}-${_glibc}-lock.json"
      "linux-${_uname_m}" "null" platform libc)
  elseif(NOT _libc STREQUAL "glibc")
    _json_escape("${_libc}" _libc_json)
    _lock_set("${SCRATCH_DIR}/ro-libc-${_libc}-${_glibc}-lock.json"
      "linux-${_uname_m}" "${_libc_json}" platform libc)
  endif()
  _ro_case(${_c} SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-libc-${_libc}-${_glibc}-lock.json"
           GLIBC ${_glibc}
           DEPS_ROOT "${ro-libc-${_libc}-${_glibc}_root}"
           DEPS_LIB "${ro-libc-${_libc}-${_glibc}_lib}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "numerics record libc")
  _expect_squashed(${_c} "${${_c}_out}" "does not match the host (glibc probe:")
endforeach()

# test_record_only_linux_rejects_changed_provider_bytes +
# test_file_identity_requires_the_real_content_hash — the qualified file
# identity is the real content hash, not the recorded string.
foreach(_variant changed forged)
  set(_c "record_only_linux_rejects_provider_sha_mismatch[${_variant}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider(ro-sha-${_variant} "libfixture-openblas.so")
  if(_variant STREQUAL "forged")
    set(_sha "0000000000000000000000000000000000000000000000000000000000000000")
  else()
    set(_sha "${ro-sha-${_variant}_sha}")
  endif()
  _linux_record(ro-sha-${_variant}
    "${ro-sha-${_variant}_root}" "${ro-sha-${_variant}_lib}" "${_sha}")
  if(_variant STREQUAL "changed")
    file(WRITE "${ro-sha-${_variant}_lib}" "tampered provider bytes\n")
  endif()
  _ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-sha-${_variant}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-sha-${_variant}_root}"
           DEPS_LIB "${ro-sha-${_variant}_lib}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "numerics provider SHA-256 differs")
endforeach()

# test_file_identity_rejects_missing_or_misshapen_digests (2 vectors)
set(_c "record_only_linux_rejects_misshapen_provider_digests")
math(EXPR _case_count "${_case_count} + 1")
foreach(_i 0 1)
  _deps_provider(ro-msh-${_i} "libfixture-openblas.so")
  _linux_record(ro-msh-${_i}
    "${ro-msh-${_i}_root}" "${ro-msh-${_i}_lib}" "${ro-msh-${_i}_sha}")
  if(_i EQUAL 0)
    _lock_set("${SCRATCH_DIR}/ro-msh-${_i}-lock.json" "linux-${_uname_m}"
      "null" numerics library_paths 0 sha256)
  else()
    _lock_set("${SCRATCH_DIR}/ro-msh-${_i}-lock.json" "linux-${_uname_m}"
      "\"ABCDEFABCDEFABCDEFABCDEF\"" numerics library_paths 0 sha256)
  endif()
  _ro_case(msh-${_i} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-msh-${_i}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-msh-${_i}_root}" DEPS_LIB "${ro-msh-${_i}_lib}")
  if(msh-${_i}_rc EQUAL 0)
    _fail("${_c}[${_i}]: expected configure failure, exited 0")
  endif()
  _expect_squashed("${_c}[${_i}]" "${msh-${_i}_out}" "has no SHA-256 digest")
endforeach()

# test_record_only_linux_rejects_an_escaping_provider_symlink — identical
# bytes outside the acquired root still fail containment (sha passes first).
set(_c "record_only_linux_rejects_escaping_provider_symlink")
math(EXPR _case_count "${_case_count} + 1")
_deps_provider(ro-sym "libfixture-openblas.so")
set(_outside "${SCRATCH_DIR}/ro-sym-outside.so")
execute_process(COMMAND ${CMAKE_COMMAND} -E copy "${ro-sym_lib}" "${_outside}")
file(REMOVE "${ro-sym_lib}")
file(CREATE_LINK "${_outside}" "${ro-sym_lib}" SYMBOLIC)
_linux_record(ro-sym "${ro-sym_root}" "${ro-sym_lib}" "${ro-sym_sha}")
_ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-sym-lock.json"
         GLIBC ON DEPS_ROOT "${ro-sym_root}" DEPS_LIB "${ro-sym_lib}")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "outside the selected NumPy provider root")

# test_file_provider_must_be_a_file — a directory is not a provider image:
# EXISTS passes but the content identity does not (sha of the empty read).
set(_c "record_only_linux_provider_must_be_a_file")
math(EXPR _case_count "${_case_count} + 1")
_deps_provider(ro-dir "libfixture-openblas.so")
file(REMOVE "${ro-dir_lib}")
file(MAKE_DIRECTORY "${ro-dir_lib}")
_linux_record(ro-dir "${ro-dir_root}" "${ro-dir_lib}" "${ro-dir_sha}")
_ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-dir-lock.json"
         GLIBC ON DEPS_ROOT "${ro-dir_root}" DEPS_LIB "${ro-dir_lib}")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: a directory provider configured successfully")
endif()

# test_record_only_names_a_missing_platform_entry — empty records object.
set(_c "record_only_names_a_missing_platform_entry")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-noentry)
file(WRITE "${SCRATCH_DIR}/ro-noentry-lock.json" "{\"schema_version\":1,\"records\":{}}\n")
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-noentry-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "committed numerics record has no entry")
_expect_squashed(${_c} "${${_c}_out}" "--write-numerics-record")

# test_record_only_rejects_an_entry_that_describes_another_host +
# test_numerics_lock_key_must_name_the_recorded_platform.
set(_c "record_only_rejects_an_entry_that_describes_another_host")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-other)
_json_escape("not-${_uname_m}" _mach_json)
_lock_set("${SCRATCH_DIR}/ro-other-lock.json" "darwin-${_uname_m}"
  "${_mach_json}" platform machine)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-other-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "does not describe this host")

# test_record_only_keeps_the_structural_guards — the same width guard runs
# in record-only mode (Darwin blas width widened to 64).
set(_c "record_only_keeps_the_structural_guards")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-guard)
_lock_set("${SCRATCH_DIR}/ro-guard-lock.json" "darwin-${_uname_m}"
  "64" numerics blas_integer_bits)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-guard-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "unqualified numerics integer ABI")

# test_record_only_does_not_compare_with_a_measurement — a flipped committed
# fact still configures (the LIVE check is the comparison guard, not this).
set(_c "record_only_does_not_compare_with_a_measurement")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-nocmp)
_lock_set("${SCRATCH_DIR}/ro-nocmp-lock.json" "darwin-${_uname_m}"
  "\"separate\"" numerics interp lowering)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-nocmp-lock.json")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "BIKE_NUMPY_INTERP_CONTRACTED=0")
endif()

# test_unknown_backend_is_rejected — a backend the profile does not know.
set(_c "record_only_rejects_an_unknown_backend")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-unk)
_lock_set("${SCRATCH_DIR}/ro-unk-lock.json" "darwin-${_uname_m}"
  "\"SYSTEM_BLAS\"" numerics backend)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-unk-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics backend mismatch")
_expect_needle(${_c} "${${_c}_out}" "SYSTEM_BLAS")

# test_record_must_be_an_object — a non-object entry fails the shape check.
set(_c "record_only_rejects_a_non_object_entry")
math(EXPR _case_count "${_case_count} + 1")
file(WRITE "${SCRATCH_DIR}/ro-nonobj-lock.json"
  "{\"schema_version\":1,\"records\":{\"darwin-${_uname_m}\":\"NUMPY_OPENBLAS\"}}\n")
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-nonobj-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "committed numerics record has no entry")

# test_check_names_the_regeneration_command_for_a_missing_record — the
# record-only twin: a missing lock file fails closed naming the record.
set(_c "record_only_missing_lock_file")
math(EXPR _case_count "${_case_count} + 1")
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-absent.lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "committed numerics record is missing")

# Integer-width vectors — the qualified profile per backend is enforced
# against the committed record in both modes. NUMPY_OPENBLAS is 64/64;
# ACCELERATE is the 32/64 split; a non-integer or bool member fails too.
# (test_openblas_requires_ilp64_on_both_interfaces,
#  test_integer_widths_must_be_ints_not_bools,
#  test_linux_rejects_unqualified_integer_width,
#  test_numerics_lock_rejects_an_unqualified_integer_width_on_write)
foreach(_pair "null|64" "64|null" "32|64" "64|32" "32|32" "null|null" "true|64")
  string(REGEX MATCH "([^|]+)\\|(.*)" _mm "${_pair}")
  set(_blas "${CMAKE_MATCH_1}")
  set(_lapack "${CMAKE_MATCH_2}")
  set(_c "record_only_openblas_width[${_blas}-${_lapack}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider(ro-w-${_blas}-${_lapack} "libfixture-openblas.so")
  _linux_record(ro-w-${_blas}-${_lapack}
    "${ro-w-${_blas}-${_lapack}_root}" "${ro-w-${_blas}-${_lapack}_lib}"
    "${ro-w-${_blas}-${_lapack}_sha}")
  _lock_set("${SCRATCH_DIR}/ro-w-${_blas}-${_lapack}-lock.json"
    "linux-${_uname_m}" "${_blas}" numerics blas_integer_bits)
  _lock_set("${SCRATCH_DIR}/ro-w-${_blas}-${_lapack}-lock.json"
    "linux-${_uname_m}" "${_lapack}" numerics lapack_integer_bits)
  _ro_case(${_c} SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-w-${_blas}-${_lapack}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-w-${_blas}-${_lapack}_root}"
           DEPS_LIB "${ro-w-${_blas}-${_lapack}_lib}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "unqualified numerics integer ABI")
endforeach()

# test_accelerate_requires_the_32_64_split (4 vectors)
foreach(_pair "64|64" "32|32" "null|64" "64|32")
  string(REGEX MATCH "([^|]+)\\|(.*)" _mm "${_pair}")
  set(_blas "${CMAKE_MATCH_1}")
  set(_lapack "${CMAKE_MATCH_2}")
  set(_c "record_only_accelerate_width[${_blas}-${_lapack}]")
  math(EXPR _case_count "${_case_count} + 1")
  _darwin_record(ro-aw-${_blas}-${_lapack})
  _lock_set("${SCRATCH_DIR}/ro-aw-${_blas}-${_lapack}-lock.json"
    "darwin-${_uname_m}" "${_blas}" numerics blas_integer_bits)
  _lock_set("${SCRATCH_DIR}/ro-aw-${_blas}-${_lapack}-lock.json"
    "darwin-${_uname_m}" "${_lapack}" numerics lapack_integer_bits)
  _ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-aw-${_blas}-${_lapack}-lock.json")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "unqualified numerics integer ABI")
endforeach()

# Identifier grammar — checked_symbol's native twin is the record symbol
# guard: [A-Za-z_][A-Za-z0-9_$]* before any name reaches a declaration.
# test_checked_symbol_accepts_valid_identifiers: grammar-passing names reach
# the spelling stage (or configure when the name IS the qualified spelling).
foreach(_name "cblas_ddot" "scipy_cblas_ddot64_" "scipy_dgelsd_64_"
              "dgelsd\$NEWLAPACK\$ILP64" "_underscore_ok" "X9_\$_mixed")
  set(_c "record_only_symbol_identifier_accepts[${_name}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider("ro-sa-${_name}" "libfixture-openblas.so")
  _linux_record("ro-sa-${_name}"
    "${ro-sa-${_name}_root}" "${ro-sa-${_name}_lib}" "${ro-sa-${_name}_sha}")
  _json_escape("${_name}" _sym_json)
  _lock_set("${SCRATCH_DIR}/ro-sa-${_name}-lock.json" "linux-${_uname_m}"
    "${_sym_json}" numerics symbols ddot)
  _ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-sa-${_name}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-sa-${_name}_root}" DEPS_LIB "${ro-sa-${_name}_lib}")
  _expect_absent(${_c} "${${_c}_out}" "not a safe identifier")
endforeach()

# test_checked_symbol_rejects_unsafe_identifiers +
# test_symbol_names_must_be_checked_identifiers — every injection-shaped
# name fails the grammar check verbatim.
foreach(_name "" "9starts_with_digit" "has space" "ddot;rm -rf"
              "ddot; touch /tmp/x" "line
break" "quote\"ddot" "back\\slash" "ddot-name" "ddot(x)" "\$(rm -rf /)")
  set(_c "record_only_symbol_identifier_rejects")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider("ro-sr-${_case_count}" "libfixture-openblas.so")
  _linux_record("ro-sr-${_case_count}"
    "${ro-sr-${_case_count}_root}" "${ro-sr-${_case_count}_lib}"
    "${ro-sr-${_case_count}_sha}")
  _json_escape("${_name}" _sym_json)
  _lock_set("${SCRATCH_DIR}/ro-sr-${_case_count}-lock.json" "linux-${_uname_m}"
    "${_sym_json}" numerics symbols ddot)
  _ro_case("sr-${_case_count}" SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-sr-${_case_count}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-sr-${_case_count}_root}"
           DEPS_LIB "${ro-sr-${_case_count}_lib}")
  if(sr-${_case_count}_rc EQUAL 0)
    _fail("${_c}['${_name}']: unsafe identifier configured")
  endif()
  _expect_squashed("${_c}['${_name}']" "${sr-${_case_count}_out}"
    "numerics symbol for 'ddot' is not a safe identifier")
endforeach()

# test_checked_symbol_rejects_non_strings — typed non-string members are
# rejected. null/number/array render as non-identifiers; JSON true renders
# as 'ON', which is a valid identifier, so it is rejected at the spelling
# stage instead — the contract is rejection either way.
foreach(_tv "null" "42" "true" "[\"cblas_ddot\"]")
  set(_c "record_only_symbol_identifier_rejects_non_string[${_tv}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider("ro-sns-${_case_count}" "libfixture-openblas.so")
  _linux_record("ro-sns-${_case_count}"
    "${ro-sns-${_case_count}_root}" "${ro-sns-${_case_count}_lib}"
    "${ro-sns-${_case_count}_sha}")
  _lock_set("${SCRATCH_DIR}/ro-sns-${_case_count}-lock.json" "linux-${_uname_m}"
    "${_tv}" numerics symbols ddot)
  _ro_case("sns-${_case_count}" SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-sns-${_case_count}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-sns-${_case_count}_root}"
           DEPS_LIB "${ro-sns-${_case_count}_lib}")
  if(sns-${_case_count}_rc EQUAL 0)
    _fail("${_c}: non-string symbol configured")
  endif()
  if(_tv STREQUAL "true")
    _expect_squashed(${_c} "${sns-${_case_count}_out}"
      "is not the qualified scipy-openblas ILP64 CBLAS spelling")
  else()
    _expect_squashed(${_c} "${sns-${_case_count}_out}" "not a safe identifier")
  endif()
endforeach()

# Backend symbol spellings pin the integer ABI — every operation carries the
# qualified spelling; a same-width name of another operation is not a
# substitute. (test_openblas_symbols_must_carry_the_ilp64_suffix,
#  test_openblas_lapack_symbol_must_be_the_fortran_ilp64_name,
#  test_accelerate_blas_symbols_must_be_the_legacy_interface,
#  test_accelerate_lapack_symbol_must_be_newlapack_ilp64,
#  test_accelerate_raw_dot_cannot_use_the_legacy_ddot_abi,
#  test_matching_integer_width_cannot_substitute_a_different_operation)
foreach(_vec
    "openblas|ddot|scipy_cblas_ddot|scipy-openblas ILP64 CBLAS spelling"
    "openblas|dgelsd|scipy_dgelsd_|scipy-openblas ILP64 Fortran spelling"
    "openblas|dgesv|scipy_dsyevd_64_|scipy-openblas ILP64 Fortran spelling"
    "accelerate|ddot_ilp64|cblas_ddot|Accelerate NEWLAPACK ILP64 CBLAS interface"
    "accelerate|ddot|cblas_ddot\$NEWLAPACK\$ILP64|legacy 32-bit CBLAS interface"
    "accelerate|dgelsd|dgelsd_|Accelerate NEWLAPACK ILP64 interface"
    "accelerate|dgesv|dsyevd\$NEWLAPACK\$ILP64|Accelerate NEWLAPACK ILP64 interface")
  string(REGEX MATCH "([^|]+)\\|([^|]+)\\|([^|]+)\\|(.*)" _mm "${_vec}")
  set(_be "${CMAKE_MATCH_1}")
  set(_op "${CMAKE_MATCH_2}")
  set(_sym "${CMAKE_MATCH_3}")
  set(_want "${CMAKE_MATCH_4}")
  set(_c "record_only_symbol_spelling[${_be}-${_op}-${_sym}]")
  math(EXPR _case_count "${_case_count} + 1")
  if(_be STREQUAL "openblas")
    _deps_provider("ro-sp-${_case_count}" "libfixture-openblas.so")
    _linux_record("ro-sp-${_case_count}"
      "${ro-sp-${_case_count}_root}" "${ro-sp-${_case_count}_lib}"
      "${ro-sp-${_case_count}_sha}")
    set(_key "linux-${_uname_m}")
    set(_sys "Linux")
  else()
    _darwin_record("ro-sp-${_case_count}")
    set(_key "darwin-${_uname_m}")
    set(_sys "Darwin")
  endif()
  _json_escape("${_sym}" _sym_json)
  _lock_set("${SCRATCH_DIR}/ro-sp-${_case_count}-lock.json" "${_key}"
    "${_sym_json}" numerics symbols ${_op})
  if(_be STREQUAL "openblas")
    _ro_case("sp-${_case_count}" SYSTEM Linux
             RECORD_FILE "${SCRATCH_DIR}/ro-sp-${_case_count}-lock.json"
             GLIBC ON DEPS_ROOT "${ro-sp-${_case_count}_root}"
             DEPS_LIB "${ro-sp-${_case_count}_lib}")
  else()
    _ro_case("sp-${_case_count}" SYSTEM Darwin
             RECORD_FILE "${SCRATCH_DIR}/ro-sp-${_case_count}-lock.json")
  endif()
  if(sp-${_case_count}_rc EQUAL 0)
    _fail("${_c}: wrong spelling '${_sym}' configured for '${_op}'")
  endif()
  _expect_squashed("${_c}" "${sp-${_case_count}_out}" "is not the qualified ${_want}")
endforeach()

# Lowering spellings — every committed lowering fact carries exactly the
# qualified verdicts; anything else fails closed. interp/multiply_broadcast
# are the Python vectors; distributions/scipy_correlate are the same
# contract on the newer committed facts (same guard, same needle).
foreach(_vec
    "interp|lowering|contracted|OK"
    "interp|lowering|separate|OK"
    "interp|lowering|fused|numerics.interp.lowering must be 'contracted' or 'separate'"
    "interp|lowering|fma|numerics.interp.lowering must be 'contracted' or 'separate'"
    "interp|lowering||numerics.interp.lowering must be 'contracted' or 'separate'"
    "interp|lowering|null|numerics.interp.lowering must be 'contracted' or 'separate'"
    "interp|lowering|1|numerics.interp.lowering must be 'contracted' or 'separate'"
    "multiply_broadcast|nan_winner|scalar|OK"
    "multiply_broadcast|nan_winner|element|OK"
    "multiply_broadcast|nan_winner|first|numerics.multiply_broadcast.nan_winner must be 'scalar' or 'element'"
    "multiply_broadcast|nan_winner|operand0|numerics.multiply_broadcast.nan_winner must be 'scalar' or 'element'"
    "multiply_broadcast|nan_winner||numerics.multiply_broadcast.nan_winner must be 'scalar' or 'element'"
    "multiply_broadcast|nan_winner|null|numerics.multiply_broadcast.nan_winner must be 'scalar' or 'element'"
    "multiply_broadcast|nan_winner|1|numerics.multiply_broadcast.nan_winner must be 'scalar' or 'element'"
    "distributions|lowering|fused|numerics.distributions.lowering must be 'contracted' or 'separate'"
    "distributions|lowering|null|numerics.distributions.lowering must be 'contracted' or 'separate'"
    "scipy_correlate|lowering|fused|numerics.scipy_correlate.lowering must be 'contracted' or 'separate'"
    "scipy_correlate|lowering|null|numerics.scipy_correlate.lowering must be 'contracted' or 'separate'")
  string(REGEX MATCH "([^|]+)\\|([^|]+)\\|(.*)\\|(.*)" _mm "${_vec}")
  set(_fact "${CMAKE_MATCH_1}")
  set(_member "${CMAKE_MATCH_2}")
  set(_val "${CMAKE_MATCH_3}")
  set(_want "${CMAKE_MATCH_4}")
  set(_c "record_only_lowering[${_fact}-${_val}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider("ro-lo-${_case_count}" "libfixture-openblas.so")
  _linux_record("ro-lo-${_case_count}"
    "${ro-lo-${_case_count}_root}" "${ro-lo-${_case_count}_lib}"
    "${ro-lo-${_case_count}_sha}")
  if(_val STREQUAL "null" OR _val STREQUAL "1")
    set(_vjson "${_val}")
  else()
    _json_escape("${_val}" _vjson)
  endif()
  _lock_set("${SCRATCH_DIR}/ro-lo-${_case_count}-lock.json" "linux-${_uname_m}"
    "${_vjson}" numerics ${_fact} ${_member})
  _ro_case("lo-${_case_count}" SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-lo-${_case_count}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-lo-${_case_count}_root}"
           DEPS_LIB "${ro-lo-${_case_count}_lib}")
  if(_want STREQUAL "OK")
    if(NOT lo-${_case_count}_rc EQUAL 0)
      _fail("${_c}: qualified lowering '${_val}' rejected: ${lo-${_case_count}_out}")
    endif()
  else()
    if(lo-${_case_count}_rc EQUAL 0)
      _fail("${_c}: unqualified lowering '${_val}' configured")
    endif()
    _expect_squashed("${_c}" "${lo-${_case_count}_out}" "${_want}")
  endif()
endforeach()

# Missing committed fields are named — test_missing_record_fields_are_named
# (the five committed keys; `threading` is a live-only field with no
# committed counterpart) + test_missing_required_symbol_is_rejected (dgelsd).
foreach(_vec
    "numpy_version|numpy_version"
    "library_paths|missing 'library_paths'"
    "symbols|missing required field 'numerics;symbols;ddot'"
    "interp|missing required field 'numerics;interp;lowering'"
    "multiply_broadcast|missing required field 'numerics;multiply_broadcast;nan_winner'"
    "symbols dgelsd|missing required field 'numerics;symbols;dgelsd'")
  string(REGEX MATCH "([^|]+)\\|(.*)" _mm "${_vec}")
  set(_member "${CMAKE_MATCH_1}")
  set(_want "${CMAKE_MATCH_2}")
  set(_c "record_only_missing_record_field[${_member}]")
  math(EXPR _case_count "${_case_count} + 1")
  _deps_provider("ro-mf-${_case_count}" "libfixture-openblas.so")
  _linux_record("ro-mf-${_case_count}"
    "${ro-mf-${_case_count}_root}" "${ro-mf-${_case_count}_lib}"
    "${ro-mf-${_case_count}_sha}")
  separate_arguments(_mp NATIVE_COMMAND "${_member}")
  _lock_remove("${SCRATCH_DIR}/ro-mf-${_case_count}-lock.json" "linux-${_uname_m}"
    numerics ${_mp})
  _ro_case("mf-${_case_count}" SYSTEM Linux
           RECORD_FILE "${SCRATCH_DIR}/ro-mf-${_case_count}-lock.json"
           GLIBC ON DEPS_ROOT "${ro-mf-${_case_count}_root}"
           DEPS_LIB "${ro-mf-${_case_count}_lib}")
  if(mf-${_case_count}_rc EQUAL 0)
    _fail("${_c}: missing '${_member}' configured")
  endif()
  if(_want STREQUAL "numpy_version" OR _want STREQUAL "missing 'library_paths'")
    _expect_squashed("${_c}" "${mf-${_case_count}_out}" "${_want}")
  else()
    _expect_squashed("${_c}" "${mf-${_case_count}_out}" "missing required field")
    _expect_squashed("${_c}" "${mf-${_case_count}_out}" "${_member}")
  endif()
endforeach()

# Provider identity + role coverage —
# test_openblas_record_rejects_apple_system_entries,
# test_accelerate_record_rejects_file_entries,
# test_library_roles_must_cover_blas_and_lapack,
# test_unknown_role_or_identity_kind_is_rejected.
foreach(_vec
    "openblas|role|blas|has no provider for role 'lapack'"
    "openblas|role|math|has no provider for role 'blas'"
    "openblas|identity_kind|apple-system|has identity 'apple-system', not 'file'"
    "openblas|identity_kind|dyld-cache|has identity 'dyld-cache', not 'file'"
    "accelerate|identity_kind|file|has identity 'file', not 'apple-system'")
  string(REGEX MATCH "([^|]+)\\|([^|]+)\\|([^|]+)\\|(.*)" _mm "${_vec}")
  set(_be "${CMAKE_MATCH_1}")
  set(_member "${CMAKE_MATCH_2}")
  set(_val "${CMAKE_MATCH_3}")
  set(_want "${CMAKE_MATCH_4}")
  set(_c "record_only_provider_identity[${_be}-${_member}-${_val}]")
  math(EXPR _case_count "${_case_count} + 1")
  if(_be STREQUAL "openblas")
    _deps_provider("ro-pi-${_case_count}" "libfixture-openblas.so")
    _linux_record("ro-pi-${_case_count}"
      "${ro-pi-${_case_count}_root}" "${ro-pi-${_case_count}_lib}"
      "${ro-pi-${_case_count}_sha}")
    set(_key "linux-${_uname_m}")
  else()
    _darwin_record("ro-pi-${_case_count}")
    set(_key "darwin-${_uname_m}")
  endif()
  _json_escape("${_val}" _vjson)
  _lock_set("${SCRATCH_DIR}/ro-pi-${_case_count}-lock.json" "${_key}"
    "${_vjson}" numerics library_paths 0 ${_member})
  if(_be STREQUAL "openblas")
    _ro_case("pi-${_case_count}" SYSTEM Linux
             RECORD_FILE "${SCRATCH_DIR}/ro-pi-${_case_count}-lock.json"
             GLIBC ON DEPS_ROOT "${ro-pi-${_case_count}_root}"
             DEPS_LIB "${ro-pi-${_case_count}_lib}")
  else()
    _ro_case("pi-${_case_count}" SYSTEM Darwin
             RECORD_FILE "${SCRATCH_DIR}/ro-pi-${_case_count}-lock.json")
  endif()
  if(pi-${_case_count}_rc EQUAL 0)
    _fail("${_c}: '${_member}=${_val}' configured")
  endif()
  _expect_squashed("${_c}" "${pi-${_case_count}_out}" "${_want}")
endforeach()

# test_numerics_lock_path_form_follows_the_backend +
# test_file_provider_must_exist_and_be_inside_site_packages — the wheel
# provider path stays relative; the Accelerate image stays absolute.
set(_c "record_only_openblas_rejects_absolute_provider_path")
math(EXPR _case_count "${_case_count} + 1")
_deps_provider(ro-pf-abs "libfixture-openblas.so")
_linux_record(ro-pf-abs "${ro-pf-abs_root}" "${ro-pf-abs_lib}" "${ro-pf-abs_sha}")
_json_escape("/opt/bike-venv/lib/python3.13/site-packages/numpy.libs/libx.so" _pjson)
_lock_set("${SCRATCH_DIR}/ro-pf-abs-lock.json" "linux-${_uname_m}"
  "${_pjson}" numerics library_paths 0 path)
_ro_case(${_c} SYSTEM Linux RECORD_FILE "${SCRATCH_DIR}/ro-pf-abs-lock.json"
         GLIBC ON DEPS_ROOT "${ro-pf-abs_root}" DEPS_LIB "${ro-pf-abs_lib}")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: absolute wheel path configured")
endif()
_expect_squashed(${_c} "${${_c}_out}" "record-only NumPy provider path must be numpy.libs/<file>")

set(_c "record_only_accelerate_rejects_relative_provider_path")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(ro-pf-rel)
_lock_set("${SCRATCH_DIR}/ro-pf-rel-lock.json" "darwin-${_uname_m}"
  "\"vecLib.dylib\"" numerics library_paths 0 path)
_ro_case(${_c} SYSTEM Darwin RECORD_FILE "${SCRATCH_DIR}/ro-pf-rel-lock.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: relative system path configured")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics provider path is not absolute")

# ==========================================================================
# Section 3 — LIVE scratch configures with a stub checker
# ==========================================================================

# test_bike_numerics_resolves_auto_backend_from_the_probe (both platforms'
# AUTO profile) — the stub verdict names the record key the module reads.
set(_c "live_resolves_auto_backend[Darwin]")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(live-auto-d)
file(WRITE "${SCRATCH_DIR}/live-auto-d-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-auto-d-site\",\"library_paths\":[{\"build_id\":\"macos-26A434+accelerate-a697e7b98ff4356ca50764b38af1ccaa+sdk-26A425\"},{\"build_id\":\"macos-26A434+accelerate-a697e7b98ff4356ca50764b38af1ccaa+sdk-26A425\"}]}}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}"
           RECORD_FILE "${SCRATCH_DIR}/live-auto-d-lock.json"
           DOC "${SCRATCH_DIR}/live-auto-d-doc.json")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "RESOLVED=ACCELERATE")
endif()

set(_c "live_resolves_auto_backend[Linux]")
math(EXPR _case_count "${_case_count} + 1")
set(_site "${SCRATCH_DIR}/live-auto-l-site")
file(MAKE_DIRECTORY "${_site}/numpy.libs")
file(WRITE "${_site}/numpy.libs/libfixture-openblas.so" "live openblas provider\n")
file(SHA256 "${_site}/numpy.libs/libfixture-openblas.so" _lsha)
_openblas_numerics(_ln "numpy.libs/libfixture-openblas.so" "${_lsha}")
_write_lock("${SCRATCH_DIR}/live-auto-l-lock.json" "linux-aarch64"
  "Linux" "aarch64" "\"glibc\"" "${_ln}")
file(WRITE "${SCRATCH_DIR}/live-auto-l-doc.json"
"{\"numerics\":{\"site_packages\":\"${_site}\"}}\n")
_live_case(${_c} SYSTEM Linux KEY "linux-aarch64"
           RECORD_FILE "${SCRATCH_DIR}/live-auto-l-lock.json"
           DOC "${SCRATCH_DIR}/live-auto-l-doc.json")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "RESOLVED=NUMPY_OPENBLAS")
endif()

# test_bike_numerics_resolves_a_wheel_relative_provider_on_linux +
# test_linux_record_is_accepted_after_relocating_site_packages — the record
# holds the provider relative to site-packages; the same wheel installed at
# a fresh location resolves and verifies.
set(_c "live_resolves_wheel_relative_provider_on_linux")
math(EXPR _case_count "${_case_count} + 1")
set(_site "${SCRATCH_DIR}/live-rel-site")
file(MAKE_DIRECTORY "${_site}/numpy.libs")
file(WRITE "${_site}/numpy.libs/libscipy_openblas64_-fixture0.so" "relocated wheel provider\n")
file(SHA256 "${_site}/numpy.libs/libscipy_openblas64_-fixture0.so" _lsha)
_openblas_numerics(_ln "numpy.libs/libscipy_openblas64_-fixture0.so" "${_lsha}")
_write_lock("${SCRATCH_DIR}/live-rel-lock.json" "linux-aarch64"
  "Linux" "aarch64" "\"glibc\"" "${_ln}")
file(WRITE "${SCRATCH_DIR}/live-rel-doc.json"
"{\"numerics\":{\"site_packages\":\"${_site}\"}}\n")
_live_case(${_c} SYSTEM Linux KEY "linux-aarch64"
           RECORD_FILE "${SCRATCH_DIR}/live-rel-lock.json"
           DOC "${SCRATCH_DIR}/live-rel-doc.json")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "RESOLVED=NUMPY_OPENBLAS")
  get_filename_component(_site_real "${_site}" REALPATH)
  _expect_needle(${_c} "${${_c}_out}"
    "PROVIDER_PATH=${_site_real}/numpy.libs/libscipy_openblas64_-fixture0.so")
endif()

# test_file_provider_must_exist_and_be_inside_site_packages (LIVE half) — a
# record path escaping site-packages resolves outside the provider root.
set(_c "live_rejects_provider_outside_site_packages")
math(EXPR _case_count "${_case_count} + 1")
set(_site "${SCRATCH_DIR}/live-esc/site")
file(MAKE_DIRECTORY "${_site}/numpy.libs")
set(_outside "${SCRATCH_DIR}/live-esc/outside.so")
file(WRITE "${_outside}" "outside provider bytes\n")
file(SHA256 "${_outside}" _osha)
_openblas_numerics(_ln "../outside.so" "${_osha}")
_write_lock("${SCRATCH_DIR}/live-esc-lock.json" "linux-aarch64"
  "Linux" "aarch64" "\"glibc\"" "${_ln}")
file(WRITE "${SCRATCH_DIR}/live-esc-doc.json"
"{\"numerics\":{\"site_packages\":\"${_site}\"}}\n")
_live_case(${_c} SYSTEM Linux KEY "linux-aarch64"
           RECORD_FILE "${SCRATCH_DIR}/live-esc-lock.json"
           DOC "${SCRATCH_DIR}/live-esc-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: provider outside site-packages configured")
endif()
_expect_squashed(${_c} "${${_c}_out}" "outside the selected NumPy provider root")

# test_bike_numerics_rejects_a_backend_mismatch — an explicit request that
# does not match the qualified record is a provider mismatch.
set(_c "live_rejects_backend_mismatch")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(live-mism)
file(WRITE "${SCRATCH_DIR}/live-mism-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-mism-site\",\"library_paths\":[{\"build_id\":\"id\"},{\"build_id\":\"id\"}]}}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}" BACKEND NUMPY_OPENBLAS
           RECORD_FILE "${SCRATCH_DIR}/live-mism-lock.json"
           DOC "${SCRATCH_DIR}/live-mism-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics backend mismatch")

# test_bike_numerics_rejects_a_missing_numerics_record — a null numerics
# document is not a qualified record.
set(_c "live_rejects_missing_numerics_record")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(live-null)
file(WRITE "${SCRATCH_DIR}/live-null-doc.json" "{\"numerics\":null}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}"
           RECORD_FILE "${SCRATCH_DIR}/live-null-lock.json"
           DOC "${SCRATCH_DIR}/live-null-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "has no qualified numerics record")

# test_bike_numerics_reports_a_non_spawnable_interpreter — a command that
# cannot start still names the interpreter, the command and the result.
set(_c "live_reports_non_spawnable_interpreter")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(live-nospawn)
set(_missing "${SCRATCH_DIR}/live-nospawn/no-such-python")
file(WRITE "${SCRATCH_DIR}/live-nospawn-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-nospawn-site\",\"library_paths\":[{\"build_id\":\"id\"},{\"build_id\":\"id\"}]}}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}" CHECKER "${_missing}"
           RECORD_FILE "${SCRATCH_DIR}/live-nospawn-lock.json"
           DOC "${SCRATCH_DIR}/live-nospawn-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics record check did not pass")
_expect_needle(${_c} "${${_c}_out}" "${_missing}")
_expect_needle(${_c} "${${_c}_out}" "--check-numerics-record")
_expect_needle(${_c} "${${_c}_out}" "result '")

# test_bike_numerics_fails_configure_on_a_tampered_committed_fact +
# test_legacy_graph_configure_fails_on_a_tampered_committed_record — a
# failing checker verdict is a configure error carrying its message: the
# stale entry, the flipped fact and the regeneration command all propagate.
set(_c "live_stale_verdict_reaches_configure")
math(EXPR _case_count "${_case_count} + 1")
_darwin_record(live-stale)
set(_stub "${SCRATCH_DIR}/live-stale/stub-checker")
_write_exe("${_stub}"
"#!/bin/sh
echo \"numerics record check failed: entry.numerics.interp.lowering is stale against the live measurement (committed 'contracted', live 'separate') — regenerate it with tools/native_dependencies.py --python <interpreter> --write-numerics-record record\" >&2
exit 1
")
file(WRITE "${SCRATCH_DIR}/live-stale-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-stale-site\",\"library_paths\":[{\"build_id\":\"id\"},{\"build_id\":\"id\"}]}}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}" CHECKER "${_stub}"
           RECORD_FILE "${SCRATCH_DIR}/live-stale-lock.json"
           DOC "${SCRATCH_DIR}/live-stale-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics record check did not pass")
_expect_squashed(${_c} "${${_c}_out}" "is stale against the live measurement")
_expect_squashed(${_c} "${${_c}_out}" "interp.lowering")
_expect_squashed(${_c} "${${_c}_out}" "--write-numerics-record")

# test_bike_numerics_fails_configure_without_a_platform_entry — a stub
# verdict whose key names no record entry fails with the regeneration hint.
set(_c "live_missing_platform_entry_names_regeneration")
math(EXPR _case_count "${_case_count} + 1")
file(WRITE "${SCRATCH_DIR}/live-noentry-lock.json" "{\"schema_version\":1,\"records\":{}}\n")
file(WRITE "${SCRATCH_DIR}/live-noentry-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-noentry-site\",\"library_paths\":[{\"build_id\":\"id\"},{\"build_id\":\"id\"}]}}\n")
_live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}"
           RECORD_FILE "${SCRATCH_DIR}/live-noentry-lock.json"
           DOC "${SCRATCH_DIR}/live-noentry-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "committed numerics record has no entry")
_expect_squashed(${_c} "${${_c}_out}" "--write-numerics-record")

# test_cmake_guard_rejects_an_unqualified_width_in_the_record — verbatim:
# the stub passes, the committed record carries a 32-bit OpenBLAS width,
# and the CMake width guard is what rejects it.
set(_c "live_cmake_guard_rejects_unqualified_record_width")
math(EXPR _case_count "${_case_count} + 1")
set(_site "${SCRATCH_DIR}/live-wg-site")
file(MAKE_DIRECTORY "${_site}/numpy.libs")
file(WRITE "${_site}/numpy.libs/libfixture-openblas.so" "width guard provider\n")
file(SHA256 "${_site}/numpy.libs/libfixture-openblas.so" _lsha)
_openblas_numerics(_ln "numpy.libs/libfixture-openblas.so" "${_lsha}")
_write_lock("${SCRATCH_DIR}/live-wg-lock.json" "linux-aarch64"
  "Linux" "aarch64" "\"glibc\"" "${_ln}")
_lock_set("${SCRATCH_DIR}/live-wg-lock.json" "linux-aarch64"
  "32" numerics blas_integer_bits)
file(WRITE "${SCRATCH_DIR}/live-wg-doc.json"
"{\"numerics\":{\"site_packages\":\"${_site}\"}}\n")
_live_case(${_c} SYSTEM Linux KEY "linux-aarch64"
           RECORD_FILE "${SCRATCH_DIR}/live-wg-lock.json"
           DOC "${SCRATCH_DIR}/live-wg-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}"
  "unqualified numerics integer ABI: blas_integer_bits='32' does not match the qualified 64-bit profile for NUMPY_OPENBLAS")

# test_cmake_guard_rejects_a_tampered_symbol_in_the_record — verbatim:
# a record symbol outside the identifier grammar is rejected before it can
# reach a generated declaration.
set(_c "live_cmake_guard_rejects_tampered_record_symbol")
math(EXPR _case_count "${_case_count} + 1")
_openblas_numerics(_ln "numpy.libs/libfixture-openblas.so" "${_lsha}")
_write_lock("${SCRATCH_DIR}/live-ts-lock.json" "linux-aarch64"
  "Linux" "aarch64" "\"glibc\"" "${_ln}")
_json_escape("ddot\");execute_process(COMMAND rm -rf" _sym_json)
_lock_set("${SCRATCH_DIR}/live-ts-lock.json" "linux-aarch64"
  "${_sym_json}" numerics symbols ddot)
file(WRITE "${SCRATCH_DIR}/live-ts-doc.json"
"{\"numerics\":{\"site_packages\":\"${_site}\"}}\n")
_live_case(${_c} SYSTEM Linux KEY "linux-aarch64"
           RECORD_FILE "${SCRATCH_DIR}/live-ts-lock.json"
           DOC "${SCRATCH_DIR}/live-ts-doc.json")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "numerics symbol for 'ddot' is not a safe identifier")

# test_apple_system_identity_requires_a_real_build_id — the live document's
# per-image build_id must exist and be non-empty (2 vectors).
foreach(_i 0 1)
  set(_c "live_apple_system_requires_build_id[${_i}]")
  math(EXPR _case_count "${_case_count} + 1")
  _darwin_record("live-bid-${_i}")
  if(_i EQUAL 0)
    file(WRITE "${SCRATCH_DIR}/live-bid-${_i}-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-bid-site\",\"library_paths\":[{},{}]}}\n")
    set(_want "is missing required field")
  else()
    file(WRITE "${SCRATCH_DIR}/live-bid-${_i}-doc.json"
"{\"numerics\":{\"site_packages\":\"${SCRATCH_DIR}/live-bid-site\",\"library_paths\":[{\"build_id\":\"\"},{\"build_id\":\"x\"}]}}\n")
    set(_want "has an empty build_id")
  endif()
  _live_case(${_c} SYSTEM Darwin KEY "darwin-${_uname_m}"
             RECORD_FILE "${SCRATCH_DIR}/live-bid-${_i}-lock.json"
             DOC "${SCRATCH_DIR}/live-bid-${_i}-doc.json")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "${_want}")
endforeach()

# ==========================================================================
# Report
# ==========================================================================
get_property(_skipped GLOBAL PROPERTY _t53n_skips)
get_property(_failures GLOBAL PROPERTY _t53n_failures)
if(NOT _skipped STREQUAL "")
  message(STATUS "migration_T53_numerics skips:${_skipped}")
endif()
if(NOT _failures STREQUAL "")
  message(FATAL_ERROR
    "migration_T53_numerics failed (${_case_count} cases):${_failures}")
endif()
message(STATUS "migration_T53_numerics passed (${_case_count} cases)")
