# native/cmake/Checks.cmake — T56 produced check orchestration.
#
# Included once at the end of the root CMakeLists (standalone/Python-free
# build). Provides:
#   * bike-native-checks / bike-python-audit executables (bin/)
#   * native_sources.json + native_check_context.json at the build root,
#     describing the standalone target graph — the same contract the Python
#     oracle emitted for the legacy build, with python_executable:null
#   * check_<kind> custom targets driving the produced binary
#   * the migration_T56 ctest script test (fixture negative controls)
#
# This file resolves no interpreter and invokes none. check_* targets are
# skipped when a name already exists (a bindings-ON configure still owns the
# legacy targets under native/).

if(NOT BIKE_ROOT_BUILD)
  message(FATAL_ERROR
    "native/cmake/Checks.cmake is the standalone-build check wiring; "
    "the legacy graph owns its own emission under native/")
endif()

set(_checks_native_dir "${CMAKE_CURRENT_SOURCE_DIR}/native")
set(_checks_tools_dir "${_checks_native_dir}/src/tools")

# --- produced check executables ------------------------------------------
# These translation units are first-party: they enter native_sources.json
# and compile_commands.json like every other target, so the produced
# sweeps analyze the check orchestrator itself under the same contract.
add_executable(bike-native-checks
  "${_checks_tools_dir}/checks_main.cpp"
  "${_checks_tools_dir}/check_context.cpp"
  "${_checks_tools_dir}/check_contracts.cpp"
  "${_checks_tools_dir}/check_frontends.cpp"
  "${_checks_tools_dir}/check_reports.cpp"
  "${_checks_tools_dir}/checks_internal.hpp")
set_target_properties(bike-native-checks PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
bike_target_strict_policy(bike-native-checks)
target_include_directories(bike-native-checks SYSTEM PRIVATE
  "${_checks_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")
# bike_io reaches the link line transitively through bike_platform's private
# edge; listing it again only draws a duplicate-library warning from ld.
target_link_libraries(bike-native-checks PRIVATE bike_platform)

add_executable(bike-python-audit
  "${_checks_tools_dir}/python_audit_main.cpp")
set_target_properties(bike-python-audit PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
bike_target_strict_policy(bike-python-audit)
target_include_directories(bike-python-audit SYSTEM PRIVATE
  "${_checks_native_dir}/src" "${BIKE_DEP_rapidjson_ROOT}/include")
target_link_libraries(bike-python-audit PRIVATE bike_platform)

# --- emitted contract -----------------------------------------------------
# Identical writer to the legacy emission in native/CMakeLists.txt: local
# helpers keep this file self-contained.
function(_checks_json_quote output value)
  string(REPLACE "\\" "\\\\" _escaped "${value}")
  string(REPLACE "\"" "\\\"" _escaped "${_escaped}")
  string(REPLACE "\n" "\\n" _escaped "${_escaped}")
  string(REPLACE "\r" "\\r" _escaped "${_escaped}")
  set(${output} "\"${_escaped}\"" PARENT_SCOPE)
endfunction()

function(_checks_json_array output)
  set(_json "[")
  set(_separator "")
  foreach(_value IN LISTS ARGN)
    _checks_json_quote(_quoted "${_value}")
    string(APPEND _json "${_separator}${_quoted}")
    set(_separator ",")
  endforeach()
  string(APPEND _json "]")
  set(${output} "${_json}" PARENT_SCOPE)
endfunction()

# First-party roots: native/ first (source_roots[0]'s parent resolves to the
# repository in the orchestrator), then the proto_native_bench tree so its
# headers stay inside the header sweep's contract.
set(_checks_first_party_roots
  "${_checks_native_dir}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/proto_native_bench")
get_filename_component(_checks_mujoco_library
  "${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib" REALPATH)

# Standalone include surface: the first-party root plus the locked .deps
# include roots the standalone targets compile against.
set(_checks_required_includes
  "${_checks_native_dir}/src"
  "${BIKE_DEP_mujoco_ROOT}/include"
  "${BIKE_DEP_rapidjson_ROOT}/include"
  "${BIKE_DEP_tomlplusplus_ROOT}/include"
  "${BIKE_DEP_eigen_ROOT}"
  "${BIKE_DEP_duckdb_ROOT}/include")
set(_checks_required_files
  "${_checks_native_dir}/analysis_suppressions.json"
  "${BIKE_DEP_mujoco_ROOT}/include/mujoco/mujoco.h"
  "${_checks_mujoco_library}"
  "${BIKE_DEP_rapidjson_ROOT}/include/rapidjson/document.h")
foreach(_required_path IN LISTS _checks_required_includes _checks_required_files)
  if(NOT EXISTS "${_required_path}")
    message(FATAL_ERROR
      "Required native check dependency is missing: ${_required_path}")
  endif()
endforeach()

if(CMAKE_BUILD_TYPE)
  set(_checks_configuration "${CMAKE_BUILD_TYPE}")
else()
  set(_checks_configuration "default")
endif()

set(_checks_source_entries "[")
set(_checks_source_separator "")
set(_checks_target_contexts "{")
set(_checks_target_separator "")
get_property(_checks_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
# BUILDSYSTEM_TARGETS is scoped to this directory — targets created through
# add_subdirectory() (native/tests/migration, and native/ itself when the
# transition bindings are enabled) must be collected via SUBDIRECTORIES or
# their first-party translation units never enter the manifest while
# compile_commands.json still compiles them.
get_property(_checks_subdirectories DIRECTORY PROPERTY SUBDIRECTORIES)
foreach(_checks_subdirectory IN LISTS _checks_subdirectories)
  get_property(_checks_subdirectory_targets DIRECTORY "${_checks_subdirectory}"
    PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND _checks_targets ${_checks_subdirectory_targets})
endforeach()
list(REMOVE_DUPLICATES _checks_targets)
foreach(_target IN LISTS _checks_targets)
  get_target_property(_target_type "${_target}" TYPE)
  if(NOT _target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
    continue()
  endif()

  get_target_property(_target_sources "${_target}" SOURCES)
  if(NOT _target_sources OR _target_sources MATCHES "-NOTFOUND$")
    continue()
  endif()
  # SOURCES entries are relative to the directory where the target was
  # created — subdir targets need their own SOURCE_DIR.
  get_target_property(_target_source_dir "${_target}" SOURCE_DIR)
  foreach(_source IN LISTS _target_sources)
    if(_source MATCHES "^\\$<")
      continue()
    endif()
    get_filename_component(_source_absolute "${_source}" ABSOLUTE
      BASE_DIR "${_target_source_dir}")
    set(_source_is_first_party FALSE)
    foreach(_first_party_root IN LISTS _checks_first_party_roots)
      if(NOT _source_is_first_party)
        cmake_path(IS_PREFIX _first_party_root "${_source_absolute}" NORMALIZE
          _source_is_first_party)
      endif()
    endforeach()
    if(NOT _source_is_first_party OR
       NOT _source_absolute MATCHES "\\.(cpp|cc|cxx|C)$")
      continue()
    endif()
    get_filename_component(_source_path "${_source_absolute}" REALPATH)
    if(NOT EXISTS "${_source_path}")
      message(FATAL_ERROR
        "CMake target ${_target} has a missing first-party source: ${_source_path}")
    endif()
    _checks_json_quote(_source_json "${_source_path}")
    _checks_json_quote(_target_json "${_target}")
    set(_source_context "${_target}:${_checks_configuration}")
    _checks_json_quote(_source_context_json "${_source_context}")
    string(APPEND _checks_source_entries
      "${_checks_source_separator}{\"path\":${_source_json},\"target\":${_target_json},\"context\":${_source_context_json}}")
    set(_checks_source_separator ",")
  endforeach()

  get_target_property(_target_includes "${_target}" INCLUDE_DIRECTORIES)
  get_target_property(_target_definitions "${_target}" COMPILE_DEFINITIONS)
  get_target_property(_target_options "${_target}" COMPILE_OPTIONS)
  get_target_property(_target_link_dirs "${_target}" LINK_DIRECTORIES)
  get_target_property(_target_link_libs "${_target}" LINK_LIBRARIES)
  get_target_property(_target_standard "${_target}" CXX_STANDARD)
  get_target_property(_target_extensions "${_target}" CXX_EXTENSIONS)
  foreach(_target_property _target_includes _target_definitions
      _target_options _target_link_dirs _target_link_libs)
    if(NOT ${_target_property} OR "${${_target_property}}" MATCHES "-NOTFOUND$")
      set(${_target_property} "")
    endif()
  endforeach()
  if(NOT _target_standard OR _target_standard MATCHES "-NOTFOUND$")
    set(_target_standard "${CMAKE_CXX_STANDARD}")
  endif()
  if(NOT DEFINED _target_extensions OR _target_extensions MATCHES "-NOTFOUND$")
    set(_target_extensions "${CMAKE_CXX_EXTENSIONS}")
  endif()
  if(_target_extensions)
    set(_target_extensions_json "true")
  else()
    set(_target_extensions_json "false")
  endif()
  set(_resolved_target_includes "")
  foreach(_include IN LISTS _target_includes)
    if(_include MATCHES "^\\$<")
      list(APPEND _resolved_target_includes "${_include}")
    else()
      get_filename_component(_include_absolute "${_include}" ABSOLUTE
        BASE_DIR "${_target_source_dir}")
      list(APPEND _resolved_target_includes "${_include_absolute}")
    endif()
  endforeach()
  _checks_json_array(_target_includes_json ${_resolved_target_includes})
  _checks_json_array(_target_definitions_json ${_target_definitions})
  _checks_json_array(_target_options_json ${_target_options})
  _checks_json_array(_target_link_dirs_json ${_target_link_dirs})
  _checks_json_array(_target_link_libs_json ${_target_link_libs})
  _checks_json_quote(_target_name_json "${_target}")
  string(APPEND _checks_target_contexts
    "${_checks_target_separator}${_target_name_json}:{\"configuration\":\"${_checks_configuration}\",\"cxx_standard\":${_target_standard},\"cxx_extensions\":${_target_extensions_json},\"include_directories\":${_target_includes_json},\"compile_definitions\":${_target_definitions_json},\"compile_options\":${_target_options_json},\"link_directories\":${_target_link_dirs_json},\"link_libraries\":${_target_link_libs_json}}")
  set(_checks_target_separator ",")
endforeach()
string(APPEND _checks_source_entries "]")
string(APPEND _checks_target_contexts "}")
if(_checks_source_separator STREQUAL "")
  message(FATAL_ERROR
    "No first-party C++ translation units were found in configured CMake targets")
endif()

_checks_json_array(_checks_source_roots_json ${_checks_first_party_roots})
_checks_json_quote(_checks_compiler_path_json "${CMAKE_CXX_COMPILER}")
_checks_json_quote(_checks_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
_checks_json_quote(_checks_compiler_version_json "${CMAKE_CXX_COMPILER_VERSION}")
_checks_json_array(_checks_compiler_launcher_json ${CMAKE_CXX_COMPILER_LAUNCHER})
# The root build resolved the SDK through the shared macro before project();
# its _native_sdk_* results land in this directory scope (macro contract).
if(APPLE)
  set(_checks_sdk "${_native_sdk_path}")
  set(_checks_sdk_requested "${_native_sdk_requested}")
  set(_checks_sdk_source "${_native_sdk_source}")
else()
  set(_checks_sdk "")
  set(_checks_sdk_requested "")
  set(_checks_sdk_source "not_applicable")
endif()
_checks_json_quote(_checks_sdk_json "${_checks_sdk}")
_checks_json_quote(_checks_sdk_requested_json "${_checks_sdk_requested}")
_checks_json_quote(_checks_sdk_source_json "${_checks_sdk_source}")
_checks_json_quote(_checks_suppressions_manifest_json
  "${_checks_native_dir}/analysis_suppressions.json")
_checks_json_array(_checks_include_paths_json ${_checks_required_includes})
_checks_json_array(_checks_library_paths_json
  "${_checks_mujoco_library}"
  "${_checks_sdk}/System/Library/Frameworks/Accelerate.framework")
_checks_json_array(_checks_required_files_json ${_checks_required_files})
if(BIKE_LIBCPP_HARDENING_SUPPORTED)
  set(_checks_libcpp_hardening_supported_json "true")
else()
  set(_checks_libcpp_hardening_supported_json "false")
endif()
_checks_json_quote(_checks_libcpp_hardening_mode_json
  "${BIKE_LIBCPP_HARDENING_EFFECTIVE}")
_checks_json_quote(_checks_libcpp_hardening_requested_json
  "${BIKE_LIBCPP_HARDENING}")
set(_checks_libcpp_hardening_json
  "{\"supported\":${_checks_libcpp_hardening_supported_json},\"mode\":${_checks_libcpp_hardening_mode_json},\"requested_mode\":${_checks_libcpp_hardening_requested_json}}")

set(_checks_tool_defaults
  "{\"tidy\":\"/opt/homebrew/opt/llvm/bin/clang-tidy\",\"analyzer\":\"/opt/homebrew/opt/llvm/bin/clang++\",\"cppcheck\":\"cppcheck\",\"gxx\":\"g++-16\"}")
set(_checks_tool_overrides "{")
set(_checks_tool_separator "")
foreach(_tool_override CLANG_TIDY ANALYZER_CLANG CPPCHECK GXX)
  if(DEFINED ENV{${_tool_override}} AND NOT "$ENV{${_tool_override}}" STREQUAL "")
    _checks_json_quote(_tool_name_json "${_tool_override}")
    _checks_json_quote(_tool_value_json "$ENV{${_tool_override}}")
    string(APPEND _checks_tool_overrides
      "${_checks_tool_separator}${_tool_name_json}:${_tool_value_json}")
    set(_checks_tool_separator ",")
  endif()
endforeach()
string(APPEND _checks_tool_overrides "}")

# Standalone-build extras the produced orchestrator consumes: the cmake
# binary for the diagnostic/context reconfigure probes and the locked
# dependency metadata for fail-closed context validation. The Python oracle
# fields it emitted for transition validation stay in the schema with
# python_executable:null — the produced build resolves no interpreter.
_checks_json_quote(_checks_cmake_json "${CMAKE_COMMAND}")
_checks_json_quote(_checks_dependency_root_json "${BIKE_DEPENDENCY_ROOT}")
_checks_json_quote(_checks_dependency_lock_json "${BIKE_DEPENDENCY_LOCK}")
_checks_json_quote(_checks_configure_source_json "${CMAKE_CURRENT_SOURCE_DIR}")

file(WRITE "${CMAKE_BINARY_DIR}/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":${_checks_source_roots_json},\"sources\":${_checks_source_entries}}\n")
file(WRITE "${CMAKE_BINARY_DIR}/native_check_context.json"
  "{\"schema_version\":1,\"compiler\":{\"path\":${_checks_compiler_path_json},\"id\":${_checks_compiler_id_json},\"version\":${_checks_compiler_version_json},\"launcher\":${_checks_compiler_launcher_json}},\"sdk\":${_checks_sdk_json},\"sdk_requested\":${_checks_sdk_requested_json},\"sdk_source\":${_checks_sdk_source_json},\"libcpp_hardening\":${_checks_libcpp_hardening_json},\"analysis_suppressions_manifest\":${_checks_suppressions_manifest_json},\"include_paths\":${_checks_include_paths_json},\"library_paths\":${_checks_library_paths_json},\"required_files\":${_checks_required_files_json},\"source_roots\":${_checks_source_roots_json},\"python_executable\":null,\"cmake\":${_checks_cmake_json},\"dependency_root\":${_checks_dependency_root_json},\"dependency_lock\":${_checks_dependency_lock_json},\"configure_source_dir\":${_checks_configure_source_json},\"tool_defaults\":${_checks_tool_defaults},\"tool_overrides\":${_checks_tool_overrides},\"target_contexts\":${_checks_target_contexts}}\n")

# --- produced sweep targets ------------------------------------------------
# Each target is one produced kind. The legacy target names under native/
# take precedence when they exist (bindings-on configure) so this block
# never redefines an owned target.
foreach(_check_kind IN ITEMS frontends tidy analyzer cppcheck odr headers
    diagnostic-controls context scripts)
  string(REPLACE "-" "_" _check_target "check_${_check_kind}")
  if(NOT TARGET ${_check_target})
    add_custom_target(${_check_target}
      COMMAND "$<TARGET_FILE:bike-native-checks>" --kind "${_check_kind}"
              --build "${CMAKE_BINARY_DIR}"
      DEPENDS bike-native-checks
      COMMENT "produced native check sweep: ${_check_kind}"
      VERBATIM)
  endif()
endforeach()
if(NOT TARGET check_contracts)
  add_custom_target(check_contracts
    COMMAND "$<TARGET_FILE:bike-native-checks>" --kind contracts
            --build "${CMAKE_BINARY_DIR}"
            --repo-root "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS bike-native-checks
    COMMENT "produced native structural contract checks"
    VERBATIM)
endif()

# --- migration_T56: fixture contract for the produced orchestrator --------
if(BUILD_TESTING)
  add_test(NAME migration_T56
    COMMAND "${CMAKE_COMMAND}"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "AUDIT_BINARY=$<TARGET_FILE:bike-python-audit>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/migration_T56"
      -P "${_checks_native_dir}/tests/migration/migration_T56_check.cmake")
  if(NOT TARGET migration_T56)
    add_custom_target(migration_T56
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              -R "^migration_T56$" --no-tests=error --output-on-failure
      DEPENDS bike-native-checks bike-python-audit
      USES_TERMINAL)
  endif()
endif()
