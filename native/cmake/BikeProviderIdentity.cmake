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
# expansion drops empty elements, and build_id is empty for file identities.
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
#       order; an empty element means the identity has none (JSON null).

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
    if(_build_id STREQUAL "")
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
