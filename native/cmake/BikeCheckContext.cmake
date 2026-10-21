# Schema-2 native build-context emission.
#
# bike_write_check_context() must be called AFTER every first-party target
# exists — it scans the directory's BUILDSYSTEM_TARGETS, serializes each
# target's effective compile context (generator expressions expanded per
# configuration through file(GENERATE)) into native_check_context.json, and
# writes the schema-1 native_sources.json manifest the sweep readers
# consume. Interface aliases are never buildsystem targets, so the scan can
# never double-count a TU through Bike::Numerics-style aliases.
#
# Exported context record (schema_version 2):
#   platform      {system, host_system, machine, pointer_bits}
#   sysroot       the selected sysroot: the Apple SDK on Darwin; an explicit
#                 absolute CMAKE_SYSROOT on Linux, else null
#   sdk/sdk_requested/sdk_source
#                 Darwin Apple SDK record; null + "not-applicable" on Linux
#   stdlib        {name, version, hardening:{mode, effective}}
#   numerics      binding graph: the validated live probe record, verbatim;
#                 root graph: committed facts with resolved provider paths
#   policy        {required_flags, accepted_warnings, rejected_warnings[]}
#   capabilities  {asan, rtsan, coverage, frontend_tools} measured records
#   python        the selected dependency probe's executable/version/SOABI
#                 and include directory, or null in a Python-free root build
#   cmake, dependency_root, dependency_lock, configure_source_dir
#                 existing paths for root-build reconfigure controls only
#   source_directory, build_directory
#                 original absolute CMake roots, to diagnose copied caches
#   plus the retained schema-1 fields: compiler, compiler_policy,
#   libcpp_hardening, analysis_suppressions_manifest, include_paths,
#   library_paths, required_files, source_roots, python_executable,
#   tool_defaults, tool_overrides, target_contexts
#
# The embedded compiler_policy record is per-configuration truth here:
# generator expressions inside it are expanded by file(GENERATE) for the
# configuration being generated, while the standalone
# bike_policy_report.json keeps the unexpanded genex record.

if(NOT DEFINED BIKE_TARGET_SYSTEM)
  message(FATAL_ERROR
    "BikeCheckContext.cmake requires BIKE_TARGET_SYSTEM (see BikePlatform.cmake)")
endif()

# _ctx_emit_list(<out> <target> <property>) — a generator-expression
# fragment evaluating to a JSON string array of the target's effective
# <property> for the configuration being generated. Elements whose genex
# expands empty are dropped by the property-level JOIN; an empty property
# emits []. Values are emitted unquoted-by-CMake (inside a fixed "..."
# wrapper): target property values on this build never contain quotes or
# backslashes, and a pathological one would surface as invalid JSON rather
# than a silently wrong context.
set(_ctx_json_list_glue "\",\"")
function(_ctx_emit_list out_var target prop)
  set(${out_var}
    "$<IF:$<BOOL:$<TARGET_PROPERTY:${target},${prop}>>,\"$<JOIN:$<TARGET_PROPERTY:${target},${prop}>,${_ctx_json_list_glue}>\",>"
    PARENT_SCOPE)
endfunction()

function(bike_write_check_context)
  cmake_parse_arguments(CTX "" "NATIVE_DIR" "" ${ARGN})
  if(CTX_UNPARSED_ARGUMENTS OR CTX_KEYWORDS_MISSING_VALUES)
    message(FATAL_ERROR
      "bike_write_check_context accepts only NATIVE_DIR <native source directory>")
  endif()
  if(DEFINED CTX_NATIVE_DIR)
    get_filename_component(_ctx_native_dir "${CTX_NATIVE_DIR}" ABSOLUTE)
  else()
    # The binding graph calls from native/; retain its existing path spelling.
    set(_ctx_native_dir "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  if(NOT IS_DIRECTORY "${_ctx_native_dir}/src")
    message(FATAL_ERROR
      "bike_write_check_context: invalid native source directory '${_ctx_native_dir}'")
  endif()

  set(_ctx_required_vars BIKE_POLICY_REPORT_JSON BIKE_POLICY_REQUIRED_FLAGS
    BIKE_STDLIB_IDENTITY BIKE_STDLIB_VERSION)
  if(BIKE_ROOT_BUILD)
    list(APPEND _ctx_required_vars
      BIKE_DEPENDENCY_ROOT BIKE_DEPENDENCY_LOCK BIKE_DEP_mujoco_ROOT
      BIKE_DEP_mujoco_LIBRARY BIKE_DEP_rapidjson_ROOT BIKE_DEP_tomlplusplus_ROOT
      BIKE_DEP_eigen_ROOT BIKE_DEP_duckdb_ROOT _numerics_record_json
      _native_numerics_record_key _native_numerics_provider_paths)
  else()
    list(APPEND _ctx_required_vars
      _native_first_party_roots _native_mujoco_library MJ_DIR
      _native_dependencies_json _native_python_include _native_nb_include
      _native_robin_include)
  endif()
  foreach(_required_var IN LISTS _ctx_required_vars)
    if(NOT DEFINED ${_required_var})
      message(FATAL_ERROR
        "bike_write_check_context: '${_required_var}' is undefined — the "
        "dependency/numerics/policy modules must run before context emission")
    endif()
  endforeach()

  if(BIKE_ROOT_BUILD)
    set(_native_first_party_roots "${_ctx_native_dir}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/proto_native_bench")
    set(MJ_DIR "${BIKE_DEP_mujoco_ROOT}")
    get_filename_component(_native_mujoco_library
      "${BIKE_DEP_mujoco_LIBRARY}" REALPATH)

    # BikeNumerics has already validated the selected record and the images it
    # binds. Preserve those facts and substitute only the resolved paths, in
    # record order. An extracted NumPy provider is not a Python installation:
    # do not manufacture site_packages, imports, ABI transcripts or evidence.
    string(JSON _ctx_root_numerics ERROR_VARIABLE _ctx_record_error
      GET "${_numerics_record_json}" records "${_native_numerics_record_key}" numerics)
    if(_ctx_record_error)
      message(FATAL_ERROR
        "Cannot project the selected numerics record: ${_ctx_record_error}")
    endif()
    string(JSON _ctx_provider_count LENGTH "${_ctx_root_numerics}" library_paths)
    list(LENGTH _native_numerics_provider_paths _ctx_resolved_provider_count)
    if(_ctx_provider_count LESS 1 OR
       NOT _ctx_provider_count EQUAL _ctx_resolved_provider_count)
      message(FATAL_ERROR
        "Numerics context provider count differs from the bound record")
    endif()
    math(EXPR _ctx_provider_last "${_ctx_provider_count} - 1")
    foreach(_ctx_index RANGE ${_ctx_provider_last})
      list(GET _native_numerics_provider_paths ${_ctx_index} _ctx_provider_path)
      if(NOT IS_ABSOLUTE "${_ctx_provider_path}")
        message(FATAL_ERROR
          "Numerics context provider is not absolute: '${_ctx_provider_path}'")
      endif()
      _native_json_quote(_ctx_provider_path_json "${_ctx_provider_path}")
      string(JSON _ctx_root_numerics SET "${_ctx_root_numerics}"
        library_paths ${_ctx_index} path "${_ctx_provider_path_json}")
    endforeach()
    set(_native_dependencies_json
      "{\"numerics\":${_ctx_root_numerics},\"python\":null}")
  endif()

  # ---------------------------------------------------------------
  # Platform + sysroot + SDK records.
  # ---------------------------------------------------------------
  _native_json_quote(_ctx_system_json "${BIKE_TARGET_SYSTEM}")
  _native_json_quote(_ctx_host_json "${CMAKE_HOST_SYSTEM_NAME}")
  _native_json_quote(_ctx_machine_json "${CMAKE_SYSTEM_PROCESSOR}")
  math(EXPR _ctx_pointer_bits "${CMAKE_SIZEOF_VOID_P} * 8")
  set(_ctx_platform_json
    "{\"system\":${_ctx_system_json},\"host_system\":${_ctx_host_json},\"machine\":${_ctx_machine_json},\"pointer_bits\":${_ctx_pointer_bits}}")

  if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    if(NOT IS_DIRECTORY "${_native_sdk_path}")
      message(FATAL_ERROR
        "Darwin context emission requires the resolved Apple SDK directory; "
        "got '${_native_sdk_path}'")
    endif()
    _native_json_quote(_ctx_sdk_json "${_native_sdk_path}")
    _native_json_quote(_ctx_sdk_requested_json "${_native_sdk_requested}")
    _native_json_quote(_ctx_sdk_source_json "${_native_sdk_source}")
    # On Darwin the selected sysroot IS the SDK — identical string, the
    # reader validates the consistency.
    _native_json_quote(_ctx_sysroot_json "${_native_sdk_path}")
  else()
    # Linux: truthful no-Apple-SDK record. Never fabricate an SDK.
    set(_ctx_sdk_json "null")
    set(_ctx_sdk_requested_json "null")
    _native_json_quote(_ctx_sdk_source_json "not-applicable")
    if(DEFINED CMAKE_SYSROOT AND NOT CMAKE_SYSROOT STREQUAL "")
      if(NOT IS_ABSOLUTE "${CMAKE_SYSROOT}")
        message(FATAL_ERROR
          "CMAKE_SYSROOT must be an absolute selected native sysroot, got "
          "'${CMAKE_SYSROOT}'")
      endif()
      if(NOT IS_DIRECTORY "${CMAKE_SYSROOT}")
        message(FATAL_ERROR
          "CMAKE_SYSROOT does not name a directory: '${CMAKE_SYSROOT}'")
      endif()
      _native_json_quote(_ctx_sysroot_json "${CMAKE_SYSROOT}")
    else()
      set(_ctx_sysroot_json "null")
    endif()
  endif()

  # ---------------------------------------------------------------
  # Standard-library record.
  # ---------------------------------------------------------------
  if(BIKE_STDLIB_IDENTITY STREQUAL "libc++")
    set(_ctx_stdlib_mode "${BIKE_LIBCPP_HARDENING}")
  elseif(BIKE_STDLIB_IDENTITY STREQUAL "libstdc++")
    set(_ctx_stdlib_mode "ASSERTIONS")
  else()
    message(FATAL_ERROR
      "context emission supports libc++/libstdc++ stdlib identities; got "
      "'${BIKE_STDLIB_IDENTITY}'")
  endif()
  _native_json_quote(_ctx_stdlib_name_json "${BIKE_STDLIB_IDENTITY}")
  _native_json_quote(_ctx_stdlib_version_json "${BIKE_STDLIB_VERSION}")
  _native_json_quote(_ctx_stdlib_mode_json "${_ctx_stdlib_mode}")
  set(_ctx_stdlib_json
    "{\"name\":${_ctx_stdlib_name_json},\"version\":${_ctx_stdlib_version_json},\"hardening\":{\"mode\":${_ctx_stdlib_mode_json},\"effective\":true}}")

  # ---------------------------------------------------------------
  # Numerics — the binding document remains verbatim; the root projection
  # above contains committed facts plus the absolute paths BikeNumerics bound.
  # Both check-context readers validate an object with a nonempty backend;
  # detailed ABI/provider validation belongs to the dependency/numerics layer.
  # No machine-specific probe field is invented or relaxed for either graph.
  # ---------------------------------------------------------------
  string(JSON _ctx_numerics_json ERROR_VARIABLE _ctx_numerics_error
    GET "${_native_dependencies_json}" numerics)
  if(_ctx_numerics_error OR "${_ctx_numerics_json}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "context emission cannot read the numerics record from the "
      "dependency document: ${_ctx_numerics_error}")
  endif()

  # ---------------------------------------------------------------
  # Policy record — required flags, accepted warnings, rejected warnings
  # with reasons (all already probed by BikeCompilerPolicy).
  # ---------------------------------------------------------------
  _native_json_array(_ctx_policy_required_json ${BIKE_POLICY_REQUIRED_FLAGS})
  _native_json_array(_ctx_policy_warnings_json ${_bike_applied_warnings})
  set(_ctx_policy_json
    "{\"required_flags\":${_ctx_policy_required_json},\"accepted_warnings\":${_ctx_policy_warnings_json},\"rejected_warnings\":${_bike_policy_omitted_json}}")

  # ---------------------------------------------------------------
  # Tool defaults — platform-aware. The Darwin defaults keep the brew LLVM
  # paths used today; Linux defaults resolve through PATH.
  # ---------------------------------------------------------------
  if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    set(_ctx_tool_default_tidy "/opt/homebrew/opt/llvm/bin/clang-tidy")
    set(_ctx_tool_default_analyzer "/opt/homebrew/opt/llvm/bin/clang++")
    set(_ctx_tool_default_cppcheck "cppcheck")
    set(_ctx_tool_default_gxx "g++-16")
  else()
    set(_ctx_tool_default_tidy "clang-tidy")
    set(_ctx_tool_default_analyzer "clang++")
    set(_ctx_tool_default_cppcheck "cppcheck")
    set(_ctx_tool_default_gxx "g++")
  endif()
  set(_native_tool_defaults
    "{\"tidy\":\"${_ctx_tool_default_tidy}\",\"analyzer\":\"${_ctx_tool_default_analyzer}\",\"cppcheck\":\"${_ctx_tool_default_cppcheck}\",\"gxx\":\"${_ctx_tool_default_gxx}\"}")

  # ---------------------------------------------------------------
  # Frontend-tool resolution evidence: the configured spelling (env
  # override or platform default) and what it resolves to at configure
  # time. A missing tool is recorded as null — the sweeps fail closed on
  # their own when they actually need it.
  # ---------------------------------------------------------------
  set(_ctx_frontend_tools_json "{")
  set(_ctx_tool_sep "")
  foreach(_tool_pair "tidy:CLANG_TIDY" "analyzer:ANALYZER_CLANG"
          "cppcheck:CPPCHECK" "gxx:GXX")
    string(REPLACE ":" ";" _tool_fields "${_tool_pair}")
    list(GET _tool_fields 0 _tool_key)
    list(GET _tool_fields 1 _tool_env)
    if(DEFINED ENV{${_tool_env}} AND NOT "$ENV{${_tool_env}}" STREQUAL "")
      set(_tool_configured "$ENV{${_tool_env}}")
    else()
      set(_tool_configured "${_ctx_tool_default_${_tool_key}}")
    endif()
    set(_tool_located_json "null")
    if(_tool_configured MATCHES "/")
      if(EXISTS "${_tool_configured}")
        _native_json_quote(_tool_located_json "${_tool_configured}")
      endif()
    else()
      find_program(_ctx_tool_found_${_tool_key} NAMES "${_tool_configured}")
      if(_ctx_tool_found_${_tool_key})
        _native_json_quote(_tool_located_json "${_ctx_tool_found_${_tool_key}}")
      endif()
    endif()
    _native_json_quote(_tool_configured_json "${_tool_configured}")
    _native_json_quote(_tool_key_json "${_tool_key}")
    string(APPEND _ctx_frontend_tools_json
      "${_ctx_tool_sep}${_tool_key_json}:{\"configured\":${_tool_configured_json},\"located\":${_tool_located_json}}")
    set(_ctx_tool_sep ",")
  endforeach()
  string(APPEND _ctx_frontend_tools_json "}")

  # ---------------------------------------------------------------
  # Capabilities — measured compile+link probes, driver-resolved sanitizer
  # runtime paths and the enabled-lane run control from BikeCompilerPolicy,
  # plus the frontend-tool resolution evidence above. Each sanitizer record
  # is {requested, supported, enabled, runtime, compiler_id, target,
  # evidence{control, runtime_query, run_control}} — the shape T16's RTSan
  # qualification consumes.
  # ---------------------------------------------------------------
  _native_json_quote(_ctx_cap_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
  if(BIKE_SANITIZER_TARGET)
    _native_json_quote(_ctx_cap_target_json "${BIKE_SANITIZER_TARGET}")
  else()
    set(_ctx_cap_target_json "null")
  endif()
  set(_ctx_capabilities_json "{")
  foreach(_cap asan rtsan coverage)
    string(TOUPPER "${_cap}" _cap_upper)
    set(_cap_requested "false")
    if((_cap STREQUAL "asan" AND NATIVE_SANITIZE)
       OR (_cap STREQUAL "rtsan" AND NATIVE_RTSAN)
       OR (_cap STREQUAL "coverage" AND NATIVE_COVERAGE))
      set(_cap_requested "true")
    endif()
    if(BIKE_CAP_${_cap_upper}_SUPPORTED)
      set(_cap_supported "true")
    else()
      set(_cap_supported "false")
    endif()
    if(${_cap_requested} AND ${_cap_supported})
      set(_cap_enabled "true")
    else()
      set(_cap_enabled "false")
    endif()
    if(BIKE_CAP_${_cap_upper}_RUNTIME)
      _native_json_quote(_cap_runtime_json "${BIKE_CAP_${_cap_upper}_RUNTIME}")
    else()
      set(_cap_runtime_json "null")
    endif()
    _native_json_quote(_cap_control_json
      "${BIKE_CAP_${_cap_upper}_SUPPORTED_EVIDENCE}")
    _native_json_quote(_cap_runtime_query_json
      "${BIKE_CAP_${_cap_upper}_RUNTIME_QUERY}")
    _native_json_quote(_cap_run_control_json
      "${BIKE_CAP_${_cap_upper}_RUN_CONTROL}")
    # T16 RTSan qualification evidence: the flag+annotation compile control,
    # the load-bearing macro check, and the violation run control. Other
    # lanes record "not applicable" strings so the evidence shape is uniform.
    _native_json_quote(_cap_annotation_json
      "${BIKE_CAP_${_cap_upper}_ANNOTATION_CONTROL}")
    _native_json_quote(_cap_macro_json
      "${BIKE_CAP_${_cap_upper}_MACRO_CONTROL}")
    _native_json_quote(_cap_violation_json
      "${BIKE_CAP_${_cap_upper}_VIOLATION_CONTROL}")
    # T17 coverage qualification evidence: the resolved llvm-profdata/
    # llvm-cov pair and the compile/run/merge/export format-probe outcome.
    # Sanitizer lanes record "not applicable" strings so the evidence shape
    # is uniform across capabilities.
    _native_json_quote(_cap_tools_json
      "${BIKE_CAP_${_cap_upper}_TOOLS}")
    _native_json_quote(_cap_format_probe_json
      "${BIKE_CAP_${_cap_upper}_FORMAT_PROBE}")
    if(_cap STREQUAL "asan")
      set(_cap_sep "")
    else()
      set(_cap_sep ",")
    endif()
    string(APPEND _ctx_capabilities_json
      "${_cap_sep}\"${_cap}\":{\"requested\":${_cap_requested},\"supported\":${_cap_supported},\"enabled\":${_cap_enabled},\"runtime\":${_cap_runtime_json},\"compiler_id\":${_ctx_cap_compiler_id_json},\"target\":${_ctx_cap_target_json},\"evidence\":{\"control\":${_cap_control_json},\"runtime_query\":${_cap_runtime_query_json},\"run_control\":${_cap_run_control_json},\"annotation_control\":${_cap_annotation_json},\"macro_control\":${_cap_macro_json},\"violation_control\":${_cap_violation_json},\"tools\":${_cap_tools_json},\"format_probe\":${_cap_format_probe_json}}}")
  endforeach()
  string(APPEND _ctx_capabilities_json
    ",\"frontend_tools\":${_ctx_frontend_tools_json}}")

  # ---------------------------------------------------------------
  # Dependency path lists (existence-validated at emission).
  # ---------------------------------------------------------------
  if(BIKE_ROOT_BUILD)
    set(_native_required_includes
      "${_ctx_native_dir}/src"
      "${MJ_DIR}/include"
      "${BIKE_DEP_rapidjson_ROOT}/include"
      "${BIKE_DEP_tomlplusplus_ROOT}/include"
      "${BIKE_DEP_eigen_ROOT}"
      "${BIKE_DEP_duckdb_ROOT}/include")
    set(_native_required_files
      "${_ctx_native_dir}/analysis_suppressions.json"
      "${MJ_DIR}/include/mujoco/mujoco.h"
      "${_native_mujoco_library}"
      "${BIKE_DEP_rapidjson_ROOT}/include/rapidjson/document.h")
  else()
    set(_native_required_includes
      "${_ctx_native_dir}/src"
      "${_native_python_include}"
      "${_native_nb_include}"
      "${_native_robin_include}"
      "${MJ_DIR}/include")
    set(_native_required_files
      "${_ctx_native_dir}/analysis_suppressions.json"
      "${_native_python_include}/Python.h"
      "${_native_nb_include}/nanobind/nanobind.h"
      "${_native_robin_include}/tsl/robin_map.h"
      "${MJ_DIR}/include/mujoco/mujoco.h"
      "${_native_mujoco_library}")
  endif()
  foreach(_required_path IN LISTS _native_required_includes _native_required_files)
    if(NOT EXISTS "${_required_path}")
      message(FATAL_ERROR
        "Required native check dependency is missing: ${_required_path}")
    endif()
  endforeach()

  # library_paths records the real provider paths only: the Accelerate
  # framework directory is a Darwin artifact (there is no SDK path on
  # Linux), and the Linux numerics provider is the scipy-openblas wheel
  # .so selected by BikeNumerics.cmake — not an Apple framework.
  set(_native_library_path_entries "${_native_mujoco_library}")
  if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    list(APPEND _native_library_path_entries
      "${_native_sdk_path}/System/Library/Frameworks/Accelerate.framework")
  elseif(DEFINED _numerics_provider AND EXISTS "${_numerics_provider}")
    list(APPEND _native_library_path_entries "${_numerics_provider}")
  endif()
  foreach(_library_path IN LISTS _native_library_path_entries)
    if(NOT EXISTS "${_library_path}")
      message(FATAL_ERROR
        "Required native library path is missing: ${_library_path}")
    endif()
  endforeach()

  # ---------------------------------------------------------------
  # Target scan — manifest entries (schema-1) and per-config expanded
  # target contexts (schema-2). ALIAS/INTERFACE/IMPORTED targets are never
  # buildsystem targets, so no TU can be double-owned here.
  # ---------------------------------------------------------------
  if(CMAKE_BUILD_TYPE)
    set(_native_configuration "${CMAKE_BUILD_TYPE}")
  else()
    set(_native_configuration "default")
  endif()

  set(_native_source_entries "[")
  set(_native_source_separator "")
  set(_native_target_contexts "{")
  set(_native_target_separator "")
  # BUILDSYSTEM_TARGETS is scoped to this directory — targets created through
  # add_subdirectory() (native/tests/migration) must be collected via
  # SUBDIRECTORIES or their first-party translation units never enter the
  # manifest while compile_commands.json still compiles them.
  get_property(_native_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
  get_property(_native_subdirectories DIRECTORY PROPERTY SUBDIRECTORIES)
  foreach(_native_subdirectory IN LISTS _native_subdirectories)
    get_property(_native_subdirectory_targets DIRECTORY "${_native_subdirectory}"
      PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND _native_targets ${_native_subdirectory_targets})
  endforeach()
  list(REMOVE_DUPLICATES _native_targets)
  # Target property values are embedded verbatim (generator expressions expand
  # later), so a quote or backslash in one would corrupt the JSON. Fail at
  # configure time, naming the target and property, instead.
  foreach(_native_target IN LISTS _native_targets)
    foreach(_native_prop IN ITEMS INCLUDE_DIRECTORIES COMPILE_DEFINITIONS
                                  COMPILE_OPTIONS LINK_DIRECTORIES LINK_LIBRARIES)
      get_target_property(_native_prop_value "${_native_target}" ${_native_prop})
      if(_native_prop_value AND _native_prop_value MATCHES "[\"\\\\]")
        message(FATAL_ERROR
          "CMake target ${_native_target} property ${_native_prop} contains a quote or backslash, "
          "which the check-context JSON cannot carry: ${_native_prop_value}")
      endif()
    endforeach()
  endforeach()
  foreach(_target IN LISTS _native_targets)
    get_target_property(_target_type "${_target}" TYPE)
    if(NOT _target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
      continue()
    endif()

    get_target_property(_target_sources "${_target}" SOURCES)
    if(NOT _target_sources OR _target_sources MATCHES "-NOTFOUND$")
      continue()
    endif()
    # SOURCES entries are relative to the directory where the target was
    # created — subdirectory targets need their own SOURCE_DIR.
    get_target_property(_target_source_dir "${_target}" SOURCE_DIR)
    foreach(_source IN LISTS _target_sources)
      if(_source MATCHES "^\\$<")
        continue()
      endif()
      get_filename_component(_source_absolute "${_source}" ABSOLUTE
        BASE_DIR "${_target_source_dir}")
      set(_source_is_first_party FALSE)
      foreach(_first_party_root IN LISTS _native_first_party_roots)
        if(NOT _source_is_first_party)
          cmake_path(IS_PREFIX _first_party_root "${_source_absolute}"
            NORMALIZE _source_is_first_party)
        endif()
      endforeach()
      if(NOT _source_is_first_party
         OR NOT _source_absolute MATCHES "\\.(cpp|cc|cxx|C)$")
        continue()
      endif()
      get_filename_component(_source_path "${_source_absolute}" REALPATH)
      if(NOT EXISTS "${_source_path}")
        message(FATAL_ERROR
          "CMake target ${_target} has a missing first-party source: ${_source_path}")
      endif()
      _native_json_quote(_source_json "${_source_path}")
      _native_json_quote(_target_json "${_target}")
      set(_source_context "${_target}:${_native_configuration}")
      _native_json_quote(_source_context_json "${_source_context}")
      string(APPEND _native_source_entries
        "${_native_source_separator}{\"path\":${_source_json},\"target\":${_target_json},\"context\":${_source_context_json}}")
      set(_native_source_separator ",")
    endforeach()

    get_target_property(_target_standard "${_target}" CXX_STANDARD)
    get_target_property(_target_extensions "${_target}" CXX_EXTENSIONS)
    if(NOT _target_standard OR _target_standard MATCHES "-NOTFOUND$")
      set(_target_standard "${CMAKE_CXX_STANDARD}")
    endif()
    # CXX_EXTENSIONS is initialized from CMAKE_CXX_EXTENSIONS at target
    # creation; when neither is set the compiler default is extensions-ON,
    # so an unset property records true, not a fabricated OFF.
    if(NOT DEFINED _target_extensions
       OR _target_extensions MATCHES "-NOTFOUND$")
      if(DEFINED CMAKE_CXX_EXTENSIONS)
        set(_target_extensions "${CMAKE_CXX_EXTENSIONS}")
      else()
        set(_target_extensions TRUE)
      endif()
    endif()
    if(_target_extensions)
      set(_target_extensions_json "true")
    else()
      set(_target_extensions_json "false")
    endif()
    _ctx_emit_list(_target_includes_json "${_target}" INCLUDE_DIRECTORIES)
    _ctx_emit_list(_target_definitions_json "${_target}" COMPILE_DEFINITIONS)
    _ctx_emit_list(_target_options_json "${_target}" COMPILE_OPTIONS)
    _ctx_emit_list(_target_link_dirs_json "${_target}" LINK_DIRECTORIES)
    _ctx_emit_list(_target_link_libs_json "${_target}" LINK_LIBRARIES)
    _native_json_quote(_target_name_json "${_target}")
    # configuration expands per generated configuration; "default" covers
    # the no-CMAKE_BUILD_TYPE single-config build, matching the manifest's
    # context naming.
    string(APPEND _native_target_contexts
      "${_native_target_separator}${_target_name_json}:{\"configuration\":\"$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,default>\",\"cxx_standard\":${_target_standard},\"cxx_extensions\":${_target_extensions_json},\"include_directories\":[${_target_includes_json}],\"compile_definitions\":[${_target_definitions_json}],\"compile_options\":[${_target_options_json}],\"link_directories\":[${_target_link_dirs_json}],\"link_libraries\":[${_target_link_libs_json}]}")
    set(_native_target_separator ",")
  endforeach()
  string(APPEND _native_source_entries "]")
  string(APPEND _native_target_contexts "}")
  if(_native_source_separator STREQUAL "")
    message(FATAL_ERROR
      "No first-party C++ translation units were found in configured CMake targets")
  endif()

  # ---------------------------------------------------------------
  # Remaining scalar fields.
  # ---------------------------------------------------------------
  _native_json_array(_native_source_roots_json ${_native_first_party_roots})
  _native_json_quote(_native_compiler_path_json "${CMAKE_CXX_COMPILER}")
  _native_json_quote(_native_compiler_id_json "${CMAKE_CXX_COMPILER_ID}")
  _native_json_quote(_native_compiler_version_json "${CMAKE_CXX_COMPILER_VERSION}")
  _native_json_array(_native_compiler_launcher_json ${CMAKE_CXX_COMPILER_LAUNCHER})
  _native_json_quote(_native_suppressions_manifest_json
    "${_ctx_native_dir}/analysis_suppressions.json")
  if(BIKE_ROOT_BUILD)
    set(_native_python_json "null")
  else()
    _native_json_quote(_native_python_json "${Python_EXECUTABLE}")
  endif()
  # Retain python_executable for all existing readers. The richer record is
  # additive within schema 2; doctor can request reconfiguration when an old
  # context lacks it, without making old contexts invalid for sweep readers.
  string(JSON _ctx_python_json ERROR_VARIABLE _ctx_python_error
    GET "${_native_dependencies_json}" python)
  if(_ctx_python_error)
    message(FATAL_ERROR
      "Cannot record the configured Python identity: ${_ctx_python_error}")
  endif()
  _native_json_quote(_ctx_source_directory_json "${CMAKE_CURRENT_SOURCE_DIR}")
  _native_json_quote(_ctx_build_directory_json "${CMAKE_BINARY_DIR}")
  _native_json_array(_native_include_paths_json ${_native_required_includes})
  _native_json_array(_native_library_paths_json ${_native_library_path_entries})
  _native_json_array(_native_required_files_json ${_native_required_files})
  if(_native_libcpp_hardening_supported)
    set(_native_libcpp_hardening_supported_json "true")
  else()
    set(_native_libcpp_hardening_supported_json "false")
  endif()
  _native_json_quote(_native_libcpp_hardening_mode_json
    "${_native_libcpp_hardening_mode}")
  _native_json_quote(_native_libcpp_hardening_requested_json
    "${BIKE_LIBCPP_HARDENING}")
  set(_native_libcpp_hardening_json
    "{\"supported\":${_native_libcpp_hardening_supported_json},\"mode\":${_native_libcpp_hardening_mode_json},\"requested_mode\":${_native_libcpp_hardening_requested_json}}")

  set(_native_tool_overrides "{")
  set(_native_tool_separator "")
  foreach(_tool_override CLANG_TIDY ANALYZER_CLANG CPPCHECK GXX)
    if(DEFINED ENV{${_tool_override}} AND NOT "$ENV{${_tool_override}}" STREQUAL "")
      _native_json_quote(_tool_name_json "${_tool_override}")
      _native_json_quote(_tool_value_json "$ENV{${_tool_override}}")
      string(APPEND _native_tool_overrides
        "${_native_tool_separator}${_tool_name_json}:${_tool_value_json}")
      set(_native_tool_separator ",")
    endif()
  endforeach()
  string(APPEND _native_tool_overrides "}")

  set(_ctx_root_fields "")
  if(BIKE_ROOT_BUILD)
    foreach(_ctx_directory "${BIKE_DEPENDENCY_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}")
      if(NOT IS_DIRECTORY "${_ctx_directory}")
        message(FATAL_ERROR "Root check-context directory is missing: ${_ctx_directory}")
      endif()
    endforeach()
    foreach(_ctx_file "${CMAKE_COMMAND}" "${BIKE_DEPENDENCY_LOCK}")
      if(NOT EXISTS "${_ctx_file}" OR IS_DIRECTORY "${_ctx_file}")
        message(FATAL_ERROR "Root check-context file is missing: ${_ctx_file}")
      endif()
    endforeach()
    _native_json_quote(_ctx_cmake_json "${CMAKE_COMMAND}")
    _native_json_quote(_ctx_dependency_root_json "${BIKE_DEPENDENCY_ROOT}")
    _native_json_quote(_ctx_dependency_lock_json "${BIKE_DEPENDENCY_LOCK}")
    _native_json_quote(_ctx_configure_source_json "${CMAKE_CURRENT_SOURCE_DIR}")
    set(_ctx_root_fields
      ",\"cmake\":${_ctx_cmake_json},\"dependency_root\":${_ctx_dependency_root_json},\"dependency_lock\":${_ctx_dependency_lock_json},\"configure_source_dir\":${_ctx_configure_source_json}")
  endif()

  # ---------------------------------------------------------------
  # Write the manifest (schema-1, configure-time constants) and the
  # schema-2 context (generate-time per-config expansion).
  # ---------------------------------------------------------------
  file(WRITE "${CMAKE_BINARY_DIR}/native_sources.json"
    "{\"schema_version\":1,\"first_party_roots\":${_native_source_roots_json},\"sources\":${_native_source_entries}}\n")

  set(_ctx_content
    "{\"schema_version\":2,\"platform\":${_ctx_platform_json},\"sysroot\":${_ctx_sysroot_json},\"stdlib\":${_ctx_stdlib_json},\"numerics\":${_ctx_numerics_json},\"policy\":${_ctx_policy_json},\"capabilities\":${_ctx_capabilities_json},\"compiler\":{\"path\":${_native_compiler_path_json},\"id\":${_native_compiler_id_json},\"version\":${_native_compiler_version_json},\"launcher\":${_native_compiler_launcher_json}},\"compiler_policy\":${BIKE_POLICY_REPORT_JSON},\"sdk\":${_ctx_sdk_json},\"sdk_requested\":${_ctx_sdk_requested_json},\"sdk_source\":${_ctx_sdk_source_json},\"libcpp_hardening\":${_native_libcpp_hardening_json},\"analysis_suppressions_manifest\":${_native_suppressions_manifest_json},\"include_paths\":${_native_include_paths_json},\"library_paths\":${_native_library_paths_json},\"required_files\":${_native_required_files_json},\"source_roots\":${_native_source_roots_json},\"python_executable\":${_native_python_json},\"python\":${_ctx_python_json},\"source_directory\":${_ctx_source_directory_json},\"build_directory\":${_ctx_build_directory_json}${_ctx_root_fields},\"tool_defaults\":${_native_tool_defaults},\"tool_overrides\":${_native_tool_overrides},\"target_contexts\":${_native_target_contexts}}\n")

  if(CMAKE_CONFIGURATION_TYPES)
    # Multi-config generators cannot share one compile_commands.json; emit
    # one truthful context per configuration — the reader selects the file
    # matching the commands it actually inspects.
    file(GENERATE OUTPUT
      "${CMAKE_BINARY_DIR}/native_check_context/$<CONFIG>.json"
      CONTENT "${_ctx_content}")
  else()
    file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/native_check_context.json"
      CONTENT "${_ctx_content}")
  endif()
endfunction()
