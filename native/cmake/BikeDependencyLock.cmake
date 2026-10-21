# BikeDependencyLock.cmake — checked .deps/ dependency resolution shared by
# both native build graphs (the Python-free standalone root build and the
# legacy extension build). The shared schema-2 reader selects artifact facts
# by the same platform key as the numerics record.
#
# .deps/ is populated ONLY by explicit acquisition before configure. This
# module performs no network calls and executes no interpreter: it reads
# native/dependencies.lock.json, verifies every recorded SHA-256 (acquired
# archive plus consumed files), checks each dependency's recorded version
# against its extracted BIKE_VERSION marker, and creates the imported targets
# the first-party build consumes:
#
#   bike::mujoco        SHARED  — pinned 3.12.0 image + headers; the private
#                        TLS error-handler ABI is compile/link-probed at
#                        configure time, before any target is built.
#   bike::duckdb        SHARED  — pinned 1.5.6 C API image + headers.
#   bike::tomlplusplus  INTERFACE — header-only TOML (v3.4.0).
#   bike::rapidjson     INTERFACE — header-only JSON (pinned commit).
#   bike::eigen         INTERFACE — header-only linear algebra (3.4.0).
#   bike::glfw_headers  INTERFACE — headers for the later GLFW build (3.4).
#   bike::cairo_headers INTERFACE — pinned Cairo headers pending the binary
#                        SDK required for the qualified workflow (T50/T59).
#
# A missing/tampered/wrong-version dependency is a configure-time acquisition
# failure — FATAL before a single first-party object can build.
#
# This lane is deliberately separate from BikeDependencies.cmake: that module
# runs the schema-1 interpreter probe, which the Python-free root graph must
# never execute. This module runs no code at include() time beyond variable
# defaults, so callers invoke bike_dependencies_resolve() explicitly, keeping
# pre-project() include order under the CMakeLists's control.
#
# Exports consumed by the rest of the build:
#   BIKE_MODULE_REPO_ROOT / BIKE_DEPENDENCY_ROOT / BIKE_DEPENDENCY_LOCK —
#     the single home for the .deps/lock defaults (previously duplicated
#     inline in native/CMakeLists.txt).
#   BIKE_FIRST_PARTY_INCLUDE_DIR — engine_abi_312.hpp root for the TLS probe.
#   BIKE_DEP_<name>_ROOT — verified extracted root per dependency.
#   BIKE_DEP_mujoco_LIBRARY / BIKE_DEP_duckdb_LIBRARY — the sha256-verified
#     shared-library path for the two binary SDKs. The relative path comes
#     from the lock's required_files record (Darwin pins
#     lib/libmujoco.3.12.0.dylib; a Linux entry pins lib/libmujoco.so.3.12.0)
#     — never a platform spelling assumed by this module.
#   BIKE_RESOLVED_DEPENDENCIES / BIKE_DEP_PROVENANCE_FRAGMENTS — the
#     dependency names and per-dep provenance JSON fragments consumed by
#     bike_write_dependency_provenance (Provenance.cmake).

include_guard(GLOBAL)

# Defaults anchor at the repository root computed from this module's own
# location (native/cmake/../..), so both build graphs resolve the same
# .deps/ and lock file.
get_filename_component(BIKE_MODULE_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

if(DEFINED BIKE_DEPENDENCY_ROOT)
  get_filename_component(BIKE_DEPENDENCY_ROOT "${BIKE_DEPENDENCY_ROOT}" ABSOLUTE)
else()
  set(BIKE_DEPENDENCY_ROOT "${BIKE_MODULE_REPO_ROOT}/.deps" CACHE PATH
    "Explicitly acquired native dependency root (.deps)")
endif()
if(DEFINED BIKE_DEPENDENCY_LOCK)
  get_filename_component(BIKE_DEPENDENCY_LOCK "${BIKE_DEPENDENCY_LOCK}" ABSOLUTE)
else()
  set(BIKE_DEPENDENCY_LOCK "${BIKE_MODULE_REPO_ROOT}/native/dependencies.lock.json" CACHE FILEPATH
    "Checked dependency lock for the standalone build")
endif()

# First-party include dir holding engine_abi_312.hpp; the caller may override.
if(NOT DEFINED BIKE_FIRST_PARTY_INCLUDE_DIR)
  set(BIKE_FIRST_PARTY_INCLUDE_DIR "${BIKE_MODULE_REPO_ROOT}/native/src")
endif()

function(_bike_dep_fatal dep message_text)
  message(FATAL_ERROR
    "dependency acquisition failure for '${dep}': ${message_text}\n"
    "  Acquire the pinned artifact into BIKE_DEPENDENCY_ROOT before configure; "
    "configure/build/install never fetch or substitute dependencies.")
endfunction()

function(_bike_sha256 out path)
  file(SHA256 "${path}" _digest)
  set(${out} "${_digest}" PARENT_SCOPE)
endfunction()

function(bike_dependency_lock_read out_json)
  if(NOT EXISTS "${BIKE_DEPENDENCY_LOCK}")
    _bike_dep_fatal("(lock)" "dependency lock file is missing: ${BIKE_DEPENDENCY_LOCK}")
  endif()
  file(READ "${BIKE_DEPENDENCY_LOCK}" _lock_json)
  string(JSON _schema ERROR_VARIABLE _schema_error GET "${_lock_json}" schema_version)
  string(JSON _schema_type ERROR_VARIABLE _schema_type_error TYPE "${_lock_json}" schema_version)
  if(_schema_error OR _schema_type_error OR NOT _schema_type STREQUAL "NUMBER" OR NOT _schema STREQUAL "2")
    _bike_dep_fatal("(lock)"
      "unsupported lock schema_version '${_schema}' in ${BIKE_DEPENDENCY_LOCK}")
  endif()
  set(${out_json} "${_lock_json}" PARENT_SCOPE)
endfunction()

# Resolve one platform at a time. Common metadata stays at dependency level;
# the selected object's acquisition/required_files replace those fields. A
# literal null means not required, while a missing/invalid key fails closed.
# No archive or tree belonging only to another platform is inspected.
function(bike_dependency_lock_select lock_json dep out_entry out_required)
  string(JSON _schema ERROR_VARIABLE _error GET "${lock_json}" schema_version)
  string(JSON _schema_type ERROR_VARIABLE _type_error TYPE "${lock_json}" schema_version)
  if(_error OR _type_error OR NOT _schema_type STREQUAL "NUMBER" OR NOT _schema STREQUAL "2")
    _bike_dep_fatal("(lock)" "unsupported lock schema_version '${_schema}'")
  endif()
  string(JSON _entry_type ERROR_VARIABLE _error TYPE "${lock_json}" dependencies "${dep}")
  if(_error OR NOT _entry_type STREQUAL "OBJECT")
    _bike_dep_fatal("${dep}" "lock dependency must be an object")
  endif()
  string(JSON _entry GET "${lock_json}" dependencies "${dep}")
  string(JSON _platforms_type ERROR_VARIABLE _no_platforms TYPE "${_entry}" platforms)
  if(NOT _no_platforms)
    if(NOT _platforms_type STREQUAL "OBJECT")
      _bike_dep_fatal("${dep}" "lock platforms must be an object")
    endif()
    if(NOT BIKE_TARGET_SYSTEM OR NOT CMAKE_HOST_SYSTEM_PROCESSOR)
      _bike_dep_fatal("${dep}" "platform selection requires BikePlatform and host processor detection")
    endif()
    string(TOLOWER "${BIKE_TARGET_SYSTEM}" _system)
    set(_key "${_system}-${CMAKE_HOST_SYSTEM_PROCESSOR}")
    string(JSON _selected_type ERROR_VARIABLE _missing TYPE "${_entry}" platforms "${_key}")
    if(_missing)
      message(FATAL_ERROR "dependency '${dep}' has no lock entry for platform '${_key}'")
    endif()
    if(_selected_type STREQUAL "NULL")
      set(${out_entry} "" PARENT_SCOPE)
      set(${out_required} FALSE PARENT_SCOPE)
      return()
    endif()
    if(NOT _selected_type STREQUAL "OBJECT")
      _bike_dep_fatal("${dep}" "lock entry for platform '${_key}' must be an object or null")
    endif()
    string(JSON _selected GET "${_entry}" platforms "${_key}")
    string(JSON _entry REMOVE "${_entry}" platforms)
    # Only artifact facts vary by platform. Identity and layout remain common.
    foreach(_field acquisition required_files)
      string(JSON _field_type ERROR_VARIABLE _field_error TYPE "${_selected}" "${_field}")
      if(_field_error OR NOT _field_type STREQUAL "OBJECT")
        _bike_dep_fatal("${dep}" "platform '${_key}' must record ${_field} as an object")
      endif()
      string(JSON _value GET "${_selected}" "${_field}")
      string(JSON _entry SET "${_entry}" "${_field}" "${_value}")
    endforeach()
  endif()
  set(${out_entry} "${_entry}" PARENT_SCOPE)
  set(${out_required} TRUE PARENT_SCOPE)
endfunction()

# Shared-library names come from the selected, hash-verified artifact facts.
# Returns all images for install (OpenBLAS includes its wheel's libgfortran).
function(_bike_dep_entry_libraries entry out_members)
  string(JSON _count LENGTH "${entry}" required_files)
  set(_members "")
  if(_count GREATER 0)
    math(EXPR _last "${_count} - 1")
    foreach(_index RANGE 0 ${_last})
      string(JSON _member MEMBER "${entry}" required_files ${_index})
      if(_member MATCHES "^lib/lib[^/]+\\.(dylib|so)(\\.[0-9]+)*$")
        list(APPEND _members "${_member}")
      endif()
    endforeach()
  endif()
  set(${out_members} "${_members}" PARENT_SCOPE)
endfunction()

# Verify one selected dependency without importing targets or probing an ABI.
# Used by the binding graph for the same DuckDB/GLFW artifact contract.
function(bike_dependency_lock_verify lock_json dep out_root)
  bike_dependency_lock_select("${lock_json}" "${dep}" _dep_entry _required)
  if(NOT _required)
    set(${out_root} "" PARENT_SCOPE)
    return()
  endif()
  set(_dep "${dep}")
  string(JSON _dep_version GET "${_dep_entry}" version)
  string(JSON _dep_license GET "${_dep_entry}" license)
  string(JSON _dep_root_rel GET "${_dep_entry}" extracted_root)
  string(JSON _dep_archive GET "${_dep_entry}" acquisition archive)
  string(JSON _dep_archive_sha GET "${_dep_entry}" acquisition archive_sha256)

  set(_dep_dir "${BIKE_DEPENDENCY_ROOT}/${_dep_root_rel}")
  if(NOT IS_DIRECTORY "${_dep_dir}")
    _bike_dep_fatal("${_dep}"
      "extracted dependency root is missing: ${_dep_dir}")
  endif()

  # Version identity: the acquired tree declares the same version the lock
  # requires. An invalid version fails here, before any application build.
  # The marker is BIKE_VERSION, not VERSION: on case-insensitive filesystems
  # a VERSION file at an include-root (Eigen) would shadow libc++'s
  # <version> header.
  set(_dep_version_file "${_dep_dir}/BIKE_VERSION")
  if(NOT EXISTS "${_dep_version_file}")
    _bike_dep_fatal("${_dep}"
      "BIKE_VERSION marker is missing: ${_dep_version_file}")
  endif()
  file(READ "${_dep_version_file}" _dep_actual_version)
  string(STRIP "${_dep_actual_version}" _dep_actual_version)
  if(NOT _dep_actual_version STREQUAL "${_dep_version}")
    _bike_dep_fatal("${_dep}"
      "lock requires version '${_dep_version}' but the dependency root provides '${_dep_actual_version}' (${_dep_version_file})")
  endif()

  # License provenance: every recorded license file must exist.
  string(JSON _dep_license_count LENGTH "${_dep_entry}" license_files)
  if(NOT _dep_license_count GREATER 0)
    _bike_dep_fatal("${_dep}" "lock records no license_files")
  endif()
  math(EXPR _dep_license_last "${_dep_license_count} - 1")
  foreach(_lic_index RANGE 0 ${_dep_license_last})
    string(JSON _dep_license_file GET "${_dep_entry}" license_files ${_lic_index})
    if(NOT EXISTS "${BIKE_DEPENDENCY_ROOT}/${_dep_license_file}")
      _bike_dep_fatal("${_dep}"
        "recorded license file is missing: ${BIKE_DEPENDENCY_ROOT}/${_dep_license_file}")
    endif()
  endforeach()

  # Acquired-archive integrity.
  set(_dep_archive_path "${BIKE_DEPENDENCY_ROOT}/${_dep_archive}")
  if(NOT EXISTS "${_dep_archive_path}")
    _bike_dep_fatal("${_dep}"
      "recorded archive is missing: ${_dep_archive_path}")
  endif()
  _bike_sha256(_dep_archive_actual "${_dep_archive_path}")
  if(NOT _dep_archive_actual STREQUAL "${_dep_archive_sha}")
    _bike_dep_fatal("${_dep}"
      "archive sha256 mismatch for ${_dep_archive}: lock requires ${_dep_archive_sha}, acquired ${_dep_archive_actual}")
  endif()

  # Consumed-file integrity.
  string(JSON _dep_file_count LENGTH "${_dep_entry}" required_files)
  if(NOT _dep_file_count GREATER 0)
    _bike_dep_fatal("${_dep}" "lock records no required_files")
  endif()
  math(EXPR _dep_file_last "${_dep_file_count} - 1")
  foreach(_file_index RANGE 0 ${_dep_file_last})
    string(JSON _dep_file MEMBER "${_dep_entry}" required_files ${_file_index})
    string(JSON _dep_file_sha GET "${_dep_entry}" required_files "${_dep_file}")
    set(_dep_file_path "${_dep_dir}/${_dep_file}")
    if(NOT EXISTS "${_dep_file_path}")
      _bike_dep_fatal("${_dep}"
        "required file is missing: ${_dep_file_path}")
    endif()
    _bike_sha256(_dep_file_actual "${_dep_file_path}")
    if(NOT _dep_file_actual STREQUAL "${_dep_file_sha}")
      _bike_dep_fatal("${_dep}"
        "sha256 mismatch for ${_dep_file}: lock requires ${_dep_file_sha}, acquired ${_dep_file_actual}")
    endif()
  endforeach()
  set(${out_root} "${_dep_dir}" PARENT_SCOPE)
endfunction()

# _bike_dep_locked_library(<lock-json> <dep> <out-var>) — the sha256-verified
# shared library under the dependency's extracted root, selected from the
# lock's required_files member names (^lib/lib*.dylib, ^lib/lib*.so[.ver]).
# The lock is the source of truth for the image name, so no .dylib/.so
# spelling is assumed: whichever artifact this platform's lock entry pinned
# — and passed the sha256 check inside bike_dependencies_resolve() — is the
# library imported here. Zero or several library members in the record is a
# lock-schema violation, reported through the acquisition-failure channel.
function(_bike_dep_locked_library lock_json dep out_var)
  bike_dependency_lock_select("${lock_json}" "${dep}" _entry _required)
  if(NOT _required)
    _bike_dep_fatal("${dep}" "dependency is not required on the selected platform")
  endif()
  string(JSON _lib_file_count LENGTH "${_entry}" required_files)
  if(NOT _lib_file_count GREATER 0)
    _bike_dep_fatal("${dep}"
      "lock records no required_files for the dependency")
  endif()
  math(EXPR _lib_file_last "${_lib_file_count} - 1")
  set(_lib_found "")
  foreach(_lib_index RANGE 0 ${_lib_file_last})
    string(JSON _lib_member MEMBER "${_entry}" required_files ${_lib_index})
    if(_lib_member MATCHES "^lib/lib[^/]+\\.(dylib|so)(\\.[0-9]+)*$")
      if(NOT _lib_found STREQUAL "")
        _bike_dep_fatal("${dep}"
          "lock records more than one shared library under lib/: '${_lib_found}' and '${_lib_member}'")
      endif()
      set(_lib_found "${_lib_member}")
    endif()
  endforeach()
  if(_lib_found STREQUAL "")
    _bike_dep_fatal("${dep}"
      "lock records no shared library under lib/ for this platform's artifact set")
  endif()
  set(${out_var} "${_lib_found}" PARENT_SCOPE)
endfunction()

function(bike_dependencies_resolve)
  if(NOT IS_DIRECTORY "${BIKE_DEPENDENCY_ROOT}")
    _bike_dep_fatal("(root)"
      "BIKE_DEPENDENCY_ROOT is missing: ${BIKE_DEPENDENCY_ROOT}")
  endif()
  if(NOT EXISTS "${BIKE_DEPENDENCY_LOCK}")
    _bike_dep_fatal("(lock)"
      "dependency lock file is missing: ${BIKE_DEPENDENCY_LOCK}")
  endif()

  bike_dependency_lock_read(_dep_lock_json)

  string(JSON _dep_count LENGTH "${_dep_lock_json}" dependencies)
  if(_dep_count EQUAL 0)
    _bike_dep_fatal("(lock)" "lock file declares zero dependencies")
  endif()
  math(EXPR _dep_last "${_dep_count} - 1")

  set(_dep_names "")
  set(_dep_provenance_fragments "")
  foreach(_dep_index RANGE 0 ${_dep_last})
    string(JSON _dep MEMBER "${_dep_lock_json}" dependencies ${_dep_index})
    bike_dependency_lock_select("${_dep_lock_json}" "${_dep}" _dep_entry _dep_required)
    if(NOT _dep_required)
      continue()
    endif()
    bike_dependency_lock_verify("${_dep_lock_json}" "${_dep}" _dep_dir)
    string(JSON _dep_version GET "${_dep_entry}" version)
    string(JSON _dep_license GET "${_dep_entry}" license)
    string(JSON _dep_archive GET "${_dep_entry}" acquisition archive)
    string(JSON _dep_archive_sha GET "${_dep_entry}" acquisition archive_sha256)

    list(APPEND _dep_names "${_dep}")
    # Both scopes: the loop below consumes ${BIKE_DEP_<name>_ROOT} here while
    # first-party targets read it in the calling directory scope.
    set(BIKE_DEP_${_dep}_ROOT "${_dep_dir}")
    set(BIKE_DEP_${_dep}_ROOT "${_dep_dir}" PARENT_SCOPE)
    _bike_dep_entry_libraries("${_dep_entry}" _dep_library_members)
    set(_dep_libraries "")
    set(_dep_numpy_provider "")
    foreach(_member IN LISTS _dep_library_members)
      list(APPEND _dep_libraries "${_dep_dir}/${_member}")
      if(_dep STREQUAL "numpy_openblas" AND _member MATCHES "^lib/libscipy_openblas64_[^/]+\\.so$")
        if(NOT _dep_numpy_provider STREQUAL "")
          _bike_dep_fatal("${_dep}" "lock records multiple scipy-openblas providers")
        endif()
        set(_dep_numpy_provider "${_dep_dir}/${_member}")
      endif()
    endforeach()
    if(_dep STREQUAL "numpy_openblas")
      if(_dep_numpy_provider STREQUAL "")
        _bike_dep_fatal("${_dep}" "lock records no scipy-openblas provider")
      endif()
      set(BIKE_DEP_numpy_openblas_LIBRARY "${_dep_numpy_provider}" PARENT_SCOPE)
    endif()
    set(BIKE_DEP_${_dep}_LIBRARIES "${_dep_libraries}" PARENT_SCOPE)
    list(APPEND _dep_provenance_fragments
      "\"${_dep}\":{\"version\":\"${_dep_version}\",\"license\":\"${_dep_license}\",\"root\":\"${_dep_dir}\",\"archive\":\"${_dep_archive}\",\"archive_sha256\":\"${_dep_archive_sha}\"}")
    message(STATUS "dependency ${_dep} ${_dep_version} -> ${_dep_dir} (sha256 verified, ${_dep_license})")
  endforeach()

  set(BIKE_RESOLVED_DEPENDENCIES "${_dep_names}" PARENT_SCOPE)
  set(BIKE_DEP_PROVENANCE_FRAGMENTS "${_dep_provenance_fragments}" PARENT_SCOPE)

  # The shared-library member of each binary SDK comes from the lock record
  # just verified — Darwin's entry pins lib/libmujoco.3.12.0.dylib and
  # lib/libduckdb.dylib; Linux entries pin their .so spellings — so the
  # imported targets below carry no per-platform filename assumption.
  foreach(_binary_dep mujoco duckdb)
    if(NOT _binary_dep IN_LIST _dep_names)
      continue()
    endif()
    _bike_dep_locked_library("${_dep_lock_json}" "${_binary_dep}" _library_rel)
    set(_library "${BIKE_DEP_${_binary_dep}_ROOT}/${_library_rel}")
    set(BIKE_DEP_${_binary_dep}_LIBRARY "${_library}")
    set(BIKE_DEP_${_binary_dep}_LIBRARY "${_library}" PARENT_SCOPE)
    add_library(bike::${_binary_dep} SHARED IMPORTED GLOBAL)
    set_target_properties(bike::${_binary_dep} PROPERTIES
      IMPORTED_LOCATION "${_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_${_binary_dep}_ROOT}/include")
  endforeach()

  foreach(_header_dep tomlplusplus rapidjson eigen glfw cairo)
    if(NOT _header_dep IN_LIST _dep_names)
      continue()
    endif()
    set(_target "bike::${_header_dep}")
    set(_include "${BIKE_DEP_${_header_dep}_ROOT}/include")
    if(_header_dep STREQUAL "eigen")
      set(_include "${BIKE_DEP_eigen_ROOT}")
    elseif(_header_dep STREQUAL "glfw")
      set(_target bike::glfw_headers)
    elseif(_header_dep STREQUAL "cairo")
      set(_target bike::cairo_headers)
      set(_include "${BIKE_DEP_cairo_ROOT}/src")
    endif()
    add_library(${_target} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${_target} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${_include}")
  endforeach()

  if(NOT TARGET bike::mujoco)
    return()
  endif()

  # Pinned MuJoCo identity: 3.12.0 headers (mjVERSION_HEADER 312) and the
  # private TLS error-handler ABI must exist before any application target is
  # allowed to build. An invalid version or missing ABI fails here. Both
  # probes compile AND link: taking the function's address proves the
  # acquired image exports the symbol the headers declare.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_INCLUDES "${BIKE_DEP_mujoco_ROOT}/include")
  set(CMAKE_REQUIRED_LIBRARIES "${BIKE_DEP_mujoco_LIBRARY}")
  unset(BIKE_MUJOCO_VERSION_312 CACHE)
  check_cxx_source_compiles([=[
#include <mujoco/mujoco.h>
// mjVERSION_HEADER encodes major*1000000 + minor*1000 + patch: 3.12.0 = 3012000.
#if mjVERSION_HEADER != 3012000
#error MuJoCo headers are not the pinned 3.12.0 series
#endif
int main() {
  int (*volatile version_fn)(void) = &mj_version;
  return version_fn == nullptr;
}
]=] BIKE_MUJOCO_VERSION_312)
  if(NOT BIKE_MUJOCO_VERSION_312)
    _bike_dep_fatal("mujoco"
      "the dependency root does not provide pinned MuJoCo 3.12.0 headers/library (mjVERSION_HEADER 3012000): ${BIKE_DEP_mujoco_ROOT}")
  endif()

  # The stock 3.12.0 binary uses these private TLS functions. Check both
  # declarations and linked symbols against the acquired image before building
  # any target that could otherwise fall back to MuJoCo's process-exiting
  # handler.
  set(CMAKE_REQUIRED_INCLUDES "${BIKE_FIRST_PARTY_INCLUDE_DIR}" "${BIKE_DEP_mujoco_ROOT}/include")
  set(CMAKE_REQUIRED_LIBRARIES "${BIKE_DEP_mujoco_LIBRARY}")
  unset(BIKE_MUJOCO_ERROR_ABI_312 CACHE)
  check_cxx_source_compiles([=[
#include "engine_abi_312.hpp"
int main() {
  mjfLogHandler previous = _mjPRIVATE_setTlsLogHandler(nullptr);
  (void) _mjPRIVATE_setTlsLogHandler(previous);
  void (*volatile raw_data)(mjData**, const mjModel*) = &mj_makeRawData;
  return _mjPRIVATE_getGlobalLogHandler() == nullptr || raw_data == nullptr;
}
]=] BIKE_MUJOCO_ERROR_ABI_312)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT BIKE_MUJOCO_ERROR_ABI_312)
    _bike_dep_fatal("mujoco"
      "MuJoCo 3.12.0 private TLS error-handler ABI is unavailable in ${BIKE_DEP_mujoco_LIBRARY}")
  endif()
endfunction()
