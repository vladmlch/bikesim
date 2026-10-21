# Completion plan 3.44: native install tree and platform-specific offline source.
# Acquisition is always explicit; configuration only verifies/copies bytes.
include_guard(GLOBAL)
if(NOT BIKE_ROOT_BUILD)
  message(FATAL_ERROR "CPack distributions belong to the Python-free root graph")
endif()
if(PROJECT_VERSION STREQUAL "")
  message(FATAL_ERROR "The root project must declare VERSION (initial migration version: 0.0.0)")
endif()
string(TOLOWER "${BIKE_TARGET_SYSTEM}" _package_system)
set(_package_platform "${_package_system}-${CMAKE_HOST_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_NAME "bike-sim")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_FILE_NAME "bike-sim-${PROJECT_VERSION}-${_package_platform}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "bike-sim-${PROJECT_VERSION}-source")
set(CPACK_GENERATOR TGZ)
set(CPACK_SOURCE_GENERATOR TGZ)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_SOURCE_TOPLEVEL_TAG "${_package_platform}-Source")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/packages")
set(CPACK_VERBATIM_VARIABLES YES)

set(BIKE_PACKAGE_SOURCE_DIR "${CMAKE_SOURCE_DIR}")
set(BIKE_PACKAGE_BUILD_DIR "${CMAKE_BINARY_DIR}")
set(BIKE_PACKAGE_VERSION "${PROJECT_VERSION}")
set(BIKE_PACKAGE_SOURCE_DEPS "${CMAKE_BINARY_DIR}/source-deps")
if(IS_SYMLINK "${BIKE_PACKAGE_SOURCE_DEPS}")
  message(FATAL_ERROR "Source-package dependency staging must not be a symlink")
endif()
file(REMOVE_RECURSE "${BIKE_PACKAGE_SOURCE_DEPS}")
file(MAKE_DIRECTORY "${BIKE_PACKAGE_SOURCE_DEPS}/_archives")
bike_dependency_lock_read(_package_lock)
set(_archive_manifest)
set(_selected_archives)
foreach(_dependency IN LISTS BIKE_RESOLVED_DEPENDENCIES)
  bike_dependency_lock_select("${_package_lock}" "${_dependency}" _entry _required)
  if(NOT _required)
    continue()
  endif()
  string(JSON _archive GET "${_entry}" acquisition archive)
  string(JSON _digest GET "${_entry}" acquisition archive_sha256)
  if(NOT _archive MATCHES "^_archives/[^/]+$" OR _archive MATCHES "[;\r\n]")
    message(FATAL_ERROR "Unsafe or unsupported source-package archive name: ${_archive}")
  endif()
  file(SHA256 "${BIKE_DEPENDENCY_ROOT}/${_archive}" _actual)
  if(NOT _actual STREQUAL _digest)
    message(FATAL_ERROR "Source-package archive hash mismatch: ${_archive}")
  endif()
  file(COPY_FILE "${BIKE_DEPENDENCY_ROOT}/${_archive}"
    "${BIKE_PACKAGE_SOURCE_DEPS}/${_archive}" ONLY_IF_DIFFERENT)
  list(APPEND _selected_archives "${_archive}")
  string(APPEND _archive_manifest "${_digest}  ${_archive}\n")
endforeach()
if(NOT _selected_archives)
  message(FATAL_ERROR "Source package has no selected dependency archives")
endif()
list(SORT _selected_archives)
string(JOIN "\n" _archive_names ${_selected_archives})
file(WRITE "${CMAKE_BINARY_DIR}/package-archive-names.txt" "${_archive_names}\n")
file(WRITE "${CMAKE_BINARY_DIR}/package-archive-sha256.txt" "${_archive_manifest}")

# A direct source-root plus source-deps overlay is unsafe here: the usual
# /build exclusion also matches source-deps inside the build, or omitting
# that exclusion packages the build itself. Stage a filtered source snapshot
# at CPack time, then add exactly the selected archives. The source-tree is
# package scratch, not the user's checkout, and is never used by a build.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/package-source-tree")
set(CPACK_SOURCE_INSTALLED_DIRECTORIES "${CMAKE_BINARY_DIR}/package-source-tree;/")
set(CPACK_SOURCE_IGNORE_FILES "")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/CPackProject.cmake.in"
  "${CMAKE_BINARY_DIR}/CPackProject.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_BINARY_DIR}/CPackProject.cmake")
# include(CPack) reuses CPACK_PACKAGE_FILE_NAME for the SOURCE package name
# while configuring CPackSourceConfig.cmake; capture the binary tarball name
# first so package_smoke extracts the right artifact.
set(_package_binary_file_name "${CPACK_PACKAGE_FILE_NAME}")
include(CPack)

if(BUILD_TESTING)
  add_test(NAME package_smoke COMMAND "${CMAKE_COMMAND}"
    "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
    "-DBINARY_PACKAGE=${_package_binary_file_name}"
    "-DSOURCE_PACKAGE=${CPACK_SOURCE_PACKAGE_FILE_NAME}"
    "-DAUDIT_BINARY=$<TARGET_FILE:bike-python-audit>"
    "-DAUDIT_POLICY=${CMAKE_SOURCE_DIR}/migration/no-python-policy.json"
    "-DCONFIG=$<CONFIG>"
    -P "${CMAKE_SOURCE_DIR}/native/tests/package/package_smoke.cmake")
  set_tests_properties(package_smoke PROPERTIES LABELS "package" RUN_SERIAL TRUE TIMEOUT 600)
endif()
