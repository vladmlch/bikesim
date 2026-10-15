# Dependencies.cmake — checked dependency resolution for the standalone
# Python-free native build (T03).
#
# .deps/ is populated ONLY by explicit acquisition before configure. This
# module performs no network calls and executes no interpreter: it reads
# native/dependencies.lock.json, verifies every recorded SHA-256 (acquired
# archive plus consumed files), checks each dependency's recorded version
# against its extracted VERSION marker, and creates the imported targets the
# first-party build consumes:
#
#   bike::mujoco        SHARED  — pinned 3.12.0 dylib + headers; the private
#                        TLS error-handler ABI is compile/link-probed at
#                        configure time, before any target is built.
#   bike::duckdb        SHARED  — pinned 1.5.6 C API dylib + headers.
#   bike::tomlplusplus  INTERFACE — header-only TOML (v3.4.0).
#   bike::rapidjson     INTERFACE — header-only JSON (pinned commit).
#   bike::eigen         INTERFACE — header-only linear algebra (3.4.0).
#   bike::glfw_headers  INTERFACE — headers for the later GLFW build (3.4).
#   bike::cairo_headers INTERFACE — pinned Cairo headers pending the binary
#                        SDK required for the qualified workflow (T50/T59).
#
# A missing/tampered/wrong-version dependency is a configure-time acquisition
# failure — FATAL before a single first-party object can build.

# Defaults anchor at the repository root computed from this module's own
# location (native/cmake/../..), so both the standalone root build and the
# legacy binding build resolve the same .deps/ and lock file.
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

function(bike_dependencies_resolve)
  if(NOT IS_DIRECTORY "${BIKE_DEPENDENCY_ROOT}")
    _bike_dep_fatal("(root)"
      "BIKE_DEPENDENCY_ROOT is missing: ${BIKE_DEPENDENCY_ROOT}")
  endif()
  if(NOT EXISTS "${BIKE_DEPENDENCY_LOCK}")
    _bike_dep_fatal("(lock)"
      "dependency lock file is missing: ${BIKE_DEPENDENCY_LOCK}")
  endif()

  file(READ "${BIKE_DEPENDENCY_LOCK}" _dep_lock_json)
  string(JSON _dep_lock_schema GET "${_dep_lock_json}" schema_version)
  if(NOT _dep_lock_schema EQUAL 1)
    _bike_dep_fatal("(lock)"
      "unsupported lock schema_version '${_dep_lock_schema}' in ${BIKE_DEPENDENCY_LOCK}")
  endif()

  string(JSON _dep_count LENGTH "${_dep_lock_json}" dependencies)
  if(_dep_count EQUAL 0)
    _bike_dep_fatal("(lock)" "lock file declares zero dependencies")
  endif()
  math(EXPR _dep_last "${_dep_count} - 1")

  set(_dep_names "")
  set(_dep_provenance_fragments "")
  foreach(_dep_index RANGE 0 ${_dep_last})
    string(JSON _dep MEMBER "${_dep_lock_json}" dependencies ${_dep_index})
    string(JSON _dep_version GET "${_dep_lock_json}" dependencies ${_dep} version)
    string(JSON _dep_license GET "${_dep_lock_json}" dependencies ${_dep} license)
    string(JSON _dep_root_rel GET "${_dep_lock_json}" dependencies ${_dep} extracted_root)
    string(JSON _dep_archive GET "${_dep_lock_json}" dependencies ${_dep} acquisition archive)
    string(JSON _dep_archive_sha GET "${_dep_lock_json}" dependencies ${_dep} acquisition archive_sha256)

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
    string(JSON _dep_license_count LENGTH "${_dep_lock_json}" dependencies ${_dep} license_files)
    math(EXPR _dep_license_last "${_dep_license_count} - 1")
    foreach(_lic_index RANGE 0 ${_dep_license_last})
      string(JSON _dep_license_file GET "${_dep_lock_json}" dependencies ${_dep} license_files ${_lic_index})
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
    string(JSON _dep_file_count LENGTH "${_dep_lock_json}" dependencies ${_dep} required_files)
    math(EXPR _dep_file_last "${_dep_file_count} - 1")
    foreach(_file_index RANGE 0 ${_dep_file_last})
      string(JSON _dep_file MEMBER "${_dep_lock_json}" dependencies ${_dep} required_files ${_file_index})
      string(JSON _dep_file_sha GET "${_dep_lock_json}" dependencies ${_dep} required_files "${_dep_file}")
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

    list(APPEND _dep_names "${_dep}")
    # Both scopes: the loop below consumes ${BIKE_DEP_<name>_ROOT} here while
    # first-party targets read it in the calling directory scope.
    set(BIKE_DEP_${_dep}_ROOT "${_dep_dir}")
    set(BIKE_DEP_${_dep}_ROOT "${_dep_dir}" PARENT_SCOPE)
    list(APPEND _dep_provenance_fragments
      "\"${_dep}\":{\"version\":\"${_dep_version}\",\"license\":\"${_dep_license}\",\"root\":\"${_dep_dir}\",\"archive\":\"${_dep_archive}\",\"archive_sha256\":\"${_dep_archive_sha}\"}")
    message(STATUS "dependency ${_dep} ${_dep_version} -> ${_dep_dir} (sha256 verified, ${_dep_license})")
  endforeach()

  set(BIKE_RESOLVED_DEPENDENCIES "${_dep_names}" PARENT_SCOPE)
  set(BIKE_DEP_PROVENANCE_FRAGMENTS "${_dep_provenance_fragments}" PARENT_SCOPE)

  # Imported targets ---------------------------------------------------------
  add_library(bike::mujoco SHARED IMPORTED GLOBAL)
  set_target_properties(bike::mujoco PROPERTIES
    IMPORTED_LOCATION "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib"
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_mujoco_ROOT}/include")

  add_library(bike::duckdb SHARED IMPORTED GLOBAL)
  set_target_properties(bike::duckdb PROPERTIES
    IMPORTED_LOCATION "${BIKE_DEP_duckdb_ROOT}/lib/libduckdb.dylib"
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_duckdb_ROOT}/include")

  add_library(bike::tomlplusplus INTERFACE IMPORTED GLOBAL)
  set_target_properties(bike::tomlplusplus PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_tomlplusplus_ROOT}/include")

  add_library(bike::rapidjson INTERFACE IMPORTED GLOBAL)
  set_target_properties(bike::rapidjson PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_rapidjson_ROOT}/include")

  add_library(bike::eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(bike::eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_eigen_ROOT}")

  add_library(bike::glfw_headers INTERFACE IMPORTED GLOBAL)
  set_target_properties(bike::glfw_headers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_glfw_ROOT}/include")

  add_library(bike::cairo_headers INTERFACE IMPORTED GLOBAL)
  set_target_properties(bike::cairo_headers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BIKE_DEP_cairo_ROOT}/src")

  # Pinned MuJoCo identity: 3.12.0 headers (mjVERSION_HEADER 312) and the
  # private TLS error-handler ABI must exist before any application target is
  # allowed to build. An invalid version or missing ABI fails here. Both
  # probes compile AND link: taking the function's address proves the
  # acquired dylib exports the symbol the headers declare.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_INCLUDES "${BIKE_DEP_mujoco_ROOT}/include")
  set(CMAKE_REQUIRED_LIBRARIES "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
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
  # declarations and linked symbols against the acquired dylib before building
  # any target that could otherwise fall back to MuJoCo's process-exiting
  # handler.
  set(CMAKE_REQUIRED_INCLUDES "${BIKE_FIRST_PARTY_INCLUDE_DIR}" "${BIKE_DEP_mujoco_ROOT}/include")
  set(CMAKE_REQUIRED_LIBRARIES "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
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
      "MuJoCo 3.12.0 private TLS error-handler ABI is unavailable in ${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib")
  endif()
endfunction()

# The dylib's recorded install name (@rpath/mujoco.framework/...) is
# unresolvable on its own — the wheel ships the dylib flat with no framework
# directory to anchor the @rpath. Rewrite the recorded name to the acquired
# absolute path; -headerpad_max_install_names reserves room for the longer
# name. dylib_path is the lock-verified library actually linked.
function(bike_rewrite_mujoco_load_name target dylib_path)
  if(APPLE)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND install_name_tool -change
        "@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
        "${dylib_path}"
        "$<TARGET_FILE:${target}>"
      VERBATIM)
  endif()
endfunction()
