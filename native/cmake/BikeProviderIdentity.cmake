# Numerics provider identity — the numerics_provider_policy_json and
# required_provider_images declarations of research_build_identity.hpp.
#
# The runtime reports this identity and checks it against the provider images
# actually loaded (native/src/runtime/provenance.cpp). Both graphs generate the
# declarations through bike_provider_identity_declarations(), so the text and
# its field order cannot drift between them. They differ only in where the
# machine-specific build_id comes from: the legacy graph reads the live probe
# document, which Python measured inside the selected interpreter.
#
# Preconditions: BikeNumerics.cmake has bound the providers (its
# _native_numerics_provider_paths, _native_numerics_record_key and the
# BIKE_NUMERICS_BACKEND_RESOLVED / _native_numerics_*_bits variables). This
# module includes BikeJson.cmake for the JSON emit helpers.
#
# List inputs are passed by variable name, never by value: an unquoted list
# expansion drops empty elements. A build_id list therefore carries the marker
# <none> for an identity without one, never an empty string.
#
#   bike_provider_identity_required_images(<out> BACKEND <backend>
#       PROVIDER_PATHS_VAR <var>)
#       the loader names the build requires: the bound provider paths, plus the
#       Accelerate umbrella image on the ACCELERATE backend.
#
#   bike_provider_identity_declarations(<out> RECORD_VAR <var> RECORD_KEY <key>
#       BACKEND <backend> BLAS_BITS <n> LAPACK_BITS <n>
#       PROVIDER_PATHS_VAR <var> BUILD_IDS_VAR <var>)
#       the two C++ declarations, one per line, ready for a file(GENERATE)
#       CONTENT. BUILD_IDS_VAR holds one build_id per bound provider in record
#       order; the element <none> means the identity has none (JSON null). An
#       empty element cannot be used: CMake drops a lone empty list element, so
#       the count check would fail for a one-provider file identity.
#
# The Python-free root graph additionally probes the loaded images (Darwin):
#   bike_provider_probe_loaded_images(<out> REQUIRED_VAR <var>)
#       try_run of provider_identity_probe.cpp, compiled with the runtime's own
#       discovery TU and linked against Accelerate.
#   bike_provider_probe_image_uuid(<out> IMAGE <path> PROBE <text>)
#   bike_darwin_accelerate_build_id(<out> UUID <uuid>)
#       the build_id from sw_vers and xcrun plus the probed LC_UUID, composed by
#       bike_accelerate_build_id(<out> MACOS_BUILD <m> SDK_BUILD <s> UUID <u>).
#
# Functions that locate native sources read the module directory captured here:
# inside a function CMAKE_CURRENT_LIST_DIR names the file that called it.
set(_bike_provider_identity_dir "${CMAKE_CURRENT_LIST_DIR}")
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/BikeJson.cmake")

# The Accelerate umbrella carries the LC_UUID used for the apple-system identity.
set(BIKE_ACCELERATE_UMBRELLA_IMAGE
  "/System/Library/Frameworks/Accelerate.framework/Versions/A/Accelerate")

function(bike_provider_identity_required_images out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _required "" "BACKEND;PROVIDER_PATHS_VAR" "")
  set(_images "${${_required_PROVIDER_PATHS_VAR}}")
  if(_required_BACKEND STREQUAL "ACCELERATE")
    list(APPEND _images "${BIKE_ACCELERATE_UMBRELLA_IMAGE}")
  endif()
  if(NOT _images)
    message(FATAL_ERROR "no required provider images were recorded")
  endif()
  set(${out_var} "${_images}" PARENT_SCOPE)
endfunction()

function(bike_provider_identity_declarations out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _decl ""
    "RECORD_VAR;RECORD_KEY;BACKEND;BLAS_BITS;LAPACK_BITS;PROVIDER_PATHS_VAR;BUILD_IDS_VAR" "")
  set(_record_json "${${_decl_RECORD_VAR}}")
  set(_paths "${${_decl_PROVIDER_PATHS_VAR}}")
  set(_build_ids "${${_decl_BUILD_IDS_VAR}}")
  list(LENGTH _paths _count)
  if(_count LESS 1)
    message(FATAL_ERROR
      "numerics provider identity is missing: BikeNumerics bound no provider")
  endif()
  list(LENGTH _build_ids _build_count)
  if(NOT _build_count EQUAL _count)
    message(FATAL_ERROR
      "numerics provider identity has ${_build_count} build ids for ${_count} bound providers")
  endif()
  string(JSON _record_count ERROR_VARIABLE _json_error
    LENGTH "${_record_json}"
    records "${_decl_RECORD_KEY}" numerics library_paths)
  if(_json_error OR NOT _record_count EQUAL _count)
    message(FATAL_ERROR
      "numerics record ${_decl_RECORD_KEY} lists "
      "${_record_count} providers, BikeNumerics bound ${_count}")
  endif()

  math(EXPR _last "${_count} - 1")
  set(_libraries_json "")
  foreach(_index RANGE ${_last})
    list(GET _paths ${_index} _path)
    _native_json_quote(_path_json "${_path}")
    foreach(_field role identity_kind)
      string(JSON _member ERROR_VARIABLE _json_error
        GET "${_record_json}" records "${_decl_RECORD_KEY}"
        numerics library_paths ${_index} ${_field})
      if(_json_error OR "${_member}" MATCHES "-NOTFOUND$")
        message(FATAL_ERROR
          "committed numerics record provider ${_index} is missing '${_field}'")
      endif()
      _native_json_quote(_${_field}_json "${_member}")
    endforeach()
    string(JSON _sha_type TYPE "${_record_json}"
      records "${_decl_RECORD_KEY}" numerics library_paths ${_index} sha256)
    if(_sha_type STREQUAL "STRING")
      string(JSON _sha GET "${_record_json}"
        records "${_decl_RECORD_KEY}" numerics library_paths ${_index} sha256)
      _native_json_quote(_sha256_json "${_sha}")
    else()
      set(_sha256_json "null")
    endif()
    list(GET _build_ids ${_index} _build_id)
    if(_build_id STREQUAL "<none>")
      set(_build_id_json "null")
    else()
      _native_json_quote(_build_id_json "${_build_id}")
    endif()
    if(_index GREATER 0)
      string(APPEND _libraries_json ",")
    endif()
    string(APPEND _libraries_json
      "{\"path\":${_path_json},\"role\":${_role_json},\"identity_kind\":${_identity_kind_json},\"sha256\":${_sha256_json},\"build_id\":${_build_id_json}}")
  endforeach()

  bike_provider_identity_required_images(_required
    BACKEND "${_decl_BACKEND}" PROVIDER_PATHS_VAR _paths)
  # Newline-separated C literal: provider image names are probe-qualified
  # loader paths, which never contain newlines, quotes or backslashes.
  string(JOIN "\\n" _images_literal ${_required})
  _native_json_quote(_backend_json "${_decl_BACKEND}")
  set(_policy_json
    "{\"backend\":${_backend_json},\"blas_integer_bits\":${_decl_BLAS_BITS},\"lapack_integer_bits\":${_decl_LAPACK_BITS},\"libraries\":[${_libraries_json}]}")
  set(_declarations
    "inline constexpr const char numerics_provider_policy_json[] = R\"identity(${_policy_json})identity\";\ninline constexpr const char required_provider_images[] = \"${_images_literal}\";\n")
  set(${out_var} "${_declarations}" PARENT_SCOPE)
endfunction()

# --------------------------------------------------------------------------
# Python-free build_id for the root graph (Darwin, Accelerate). The legacy
# graph reads build_id from the live probe document, which Python measured.
# The root graph has no such document, so it probes the loaded images itself.
# --------------------------------------------------------------------------

# bike_provider_probe_loaded_images(<out> REQUIRED_VAR <var>)
#   Compiles provider_identity_probe.cpp together with the runtime's discovery
#   TU (runtime/provider_images.cpp) and links the selected backend, then runs
#   the probe with the required loader names. <out> receives the probe's text:
#   one "<path>\t<LC_UUID or ->" line per loaded provider image. A required
#   image that is not loaded fails configure with the discovery's message.
#   Darwin only: the root's record-only numerics bind only Accelerate.
function(bike_provider_probe_loaded_images out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _probe "" "REQUIRED_VAR" "")
  if(NOT BIKE_TARGET_SYSTEM STREQUAL "Darwin")
    message(FATAL_ERROR
      "the provider identity probe is qualified for Darwin only; ${BIKE_TARGET_SYSTEM} "
      "has no Python-free provider inspection in this graph")
  endif()
  if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Cross-compilation is not supported by this native-build workflow")
  endif()
  get_filename_component(_native_dir "${_bike_provider_identity_dir}/.." ABSOLUTE)
  set(_probe_dir "${CMAKE_BINARY_DIR}/bike_provider_identity")
  set(_probe_file "${_probe_dir}/loaded_images.tsv")
  file(REMOVE "${_probe_file}")
  set(_probe_required "${${_probe_REQUIRED_VAR}}")

  # try_run caches its results. A cached verdict would describe the machine as
  # it was at an earlier configure, so every configure runs the probe again.
  unset(_provider_probe_run CACHE)
  unset(_provider_probe_run)
  unset(_provider_probe_compiled CACHE)
  unset(_provider_probe_compiled)
  try_run(_provider_probe_run _provider_probe_compiled "${_probe_dir}"
    SOURCES "${_bike_provider_identity_dir}/provider_identity_probe.cpp"
            "${_native_dir}/src/runtime/provider_images.cpp"
    CMAKE_FLAGS "-DINCLUDE_DIRECTORIES=${_native_dir}/src"
    LINK_LIBRARIES "-framework Accelerate"
    CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    COMPILE_OUTPUT_VARIABLE _provider_probe_compile_log
    RUN_OUTPUT_VARIABLE _provider_probe_run_log
    ARGS "${_probe_file}" ${_probe_required})
  if(NOT _provider_probe_compiled)
    message(FATAL_ERROR
      "the provider identity probe does not compile:\n${_provider_probe_compile_log}")
  endif()
  if(NOT "${_provider_probe_run}" STREQUAL "0")
    message(FATAL_ERROR
      "the provider identity probe failed (result '${_provider_probe_run}'):\n"
      "${_provider_probe_run_log}")
  endif()
  if(NOT EXISTS "${_probe_file}")
    message(FATAL_ERROR "the provider identity probe wrote no identity file")
  endif()
  file(READ "${_probe_file}" _probe_text)
  set(${out_var} "${_probe_text}" PARENT_SCOPE)
endfunction()

# bike_provider_probe_image_uuid(<out> IMAGE <path> PROBE <text>)
#   The LC_UUID the probe reported for <path>. Fails when the probe did not
#   report the image, reported no uuid for it, or reported a uuid that is not
#   the 8-4-4-4-12 lowercase hexadecimal form the probe prints.
function(bike_provider_probe_image_uuid out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _uuid "" "IMAGE;PROBE" "")
  string(REPLACE "\n" ";" _lines "${_uuid_PROBE}")
  foreach(_line IN LISTS _lines)
    if(_line STREQUAL "")
      continue()
    endif()
    string(REPLACE "\t" ";" _fields "${_line}")
    list(LENGTH _fields _field_count)
    if(NOT _field_count EQUAL 2)
      message(FATAL_ERROR "malformed provider identity probe line: '${_line}'")
    endif()
    list(GET _fields 0 _path)
    if(NOT _path STREQUAL _uuid_IMAGE)
      continue()
    endif()
    list(GET _fields 1 _value)
    if(_value STREQUAL "-")
      message(FATAL_ERROR
        "the loaded provider image ${_uuid_IMAGE} has no LC_UUID; the apple-system identity has no evidence")
    endif()
    _bike_is_uuid_text(_is_uuid "${_value}")
    if(NOT _is_uuid)
      message(FATAL_ERROR
        "the loaded provider image ${_uuid_IMAGE} reported a malformed LC_UUID '${_value}'")
    endif()
    set(${out_var} "${_value}" PARENT_SCOPE)
    return()
  endforeach()
  message(FATAL_ERROR
    "the provider identity probe did not report the required image ${_uuid_IMAGE}")
endfunction()

# _bike_is_uuid_text(<out> <text>): TRUE when <text> is 8-4-4-4-12 lowercase hex.
# CMake's regular expressions have no counted repetition, so the group lengths
# are checked separately.
function(_bike_is_uuid_text out_var text)
  set(_ok FALSE)
  if(text MATCHES "^[0-9a-f-]+$")
    string(REPLACE "-" ";" _groups "${text}")
    list(LENGTH _groups _group_count)
    if(_group_count EQUAL 5)
      set(_ok TRUE)
      set(_expected 8 4 4 4 12)
      foreach(_index RANGE 4)
        list(GET _groups ${_index} _group)
        list(GET _expected ${_index} _length)
        string(LENGTH "${_group}" _actual)
        if(NOT _actual EQUAL _length)
          set(_ok FALSE)
        endif()
      endforeach()
    endif()
  endif()
  set(${out_var} "${_ok}" PARENT_SCOPE)
endfunction()

# _bike_identity_query(<out> <description> <command>...): the stdout of a host
# tool the Darwin build_id needs. A missing or empty answer fails configure.
function(_bike_identity_query out_var description)
  execute_process(COMMAND ${ARGN}
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _error
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REPLACE ";" " " _command_text "${ARGN}")
  if(NOT _result EQUAL 0 OR _output STREQUAL "")
    message(FATAL_ERROR
      "cannot read the ${description} for the Accelerate build_id: "
      "'${_command_text}' failed (${_result}) ${_error}")
  endif()
  set(${out_var} "${_output}" PARENT_SCOPE)
endfunction()

# bike_accelerate_build_id(<out> MACOS_BUILD <m> SDK_BUILD <s> UUID <u>)
#   The apple-system build_id, macos-<m>+accelerate-<u>+sdk-<s>. This is the
#   CMake spelling of the formula; src/bike_sim/native/provider_build_id.py is
#   the Python spelling, and both are pinned to
#   tests/fixtures/portability/build_id_vectors.json. Each component must be a
#   nonempty token of [A-Za-z0-9.-]. A malformed component sets <out> to the
#   empty string, so the caller names the failure and the vectors can pin it.
function(bike_accelerate_build_id out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _build "" "MACOS_BUILD;SDK_BUILD;UUID" "")
  set(${out_var} "" PARENT_SCOPE)
  foreach(_component IN ITEMS MACOS_BUILD SDK_BUILD UUID)
    if(NOT "${_build_${_component}}" MATCHES "^[A-Za-z0-9.-]+$")
      return()
    endif()
  endforeach()
  set(${out_var}
    "macos-${_build_MACOS_BUILD}+accelerate-${_build_UUID}+sdk-${_build_SDK_BUILD}"
    PARENT_SCOPE)
endfunction()

# bike_darwin_accelerate_build_id(<out> UUID <uuid>)
#   The root graph's build_id: the host's macOS build and SDK build (sw_vers,
#   xcrun) with the probed umbrella LC_UUID, composed by bike_accelerate_build_id.
#   A malformed component fails configure.
function(bike_darwin_accelerate_build_id out_var)
  cmake_parse_arguments(PARSE_ARGV 1 _host "" "UUID" "")
  _bike_identity_query(_macos_build "macOS build"
    sw_vers -buildVersion)
  _bike_identity_query(_sdk_build "macOS SDK build"
    xcrun --sdk macosx --show-sdk-build-version)
  bike_accelerate_build_id(_build
    MACOS_BUILD "${_macos_build}" SDK_BUILD "${_sdk_build}" UUID "${_host_UUID}")
  if(_build STREQUAL "")
    message(FATAL_ERROR
      "the Accelerate build_id components are malformed: macOS build "
      "'${_macos_build}', SDK build '${_sdk_build}', LC_UUID '${_host_UUID}'")
  endif()
  set(${out_var} "${_build}" PARENT_SCOPE)
endfunction()
