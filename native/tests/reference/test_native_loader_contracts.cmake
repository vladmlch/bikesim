# The three native contracts in tests/reference/test_native_loader.py survive
# removal of the Python extension loader. This script runs only through CTest.
cmake_minimum_required(VERSION 3.27)
foreach(_required CASE CONTRACT_EXECUTABLE SOURCE_ROOT SOURCE_MANIFEST)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "native loader contracts require -D${_required}")
  endif()
endforeach()

if(CASE STREQUAL "test_native_contract_require_control_fails_in_selected_build")
  if(NOT EXISTS "${CONTRACT_EXECUTABLE}" OR IS_DIRECTORY "${CONTRACT_EXECUTABLE}")
    message(FATAL_ERROR "native contract executable has not been built: ${CONTRACT_EXECUTABLE}")
  endif()
  execute_process(COMMAND "${CONTRACT_EXECUTABLE}" --require-failure
    WORKING_DIRECTORY "${SOURCE_ROOT}" TIMEOUT 30
    RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  if("${_result}" STREQUAL "0" OR NOT _err MATCHES "deliberate require failure")
    message(FATAL_ERROR "require control did not fail as specified (${_result}): ${_out}\n${_err}")
  endif()
elseif(CASE STREQUAL "test_native_contract_executable_lists_registered_cases")
  execute_process(COMMAND "${CONTRACT_EXECUTABLE}" --list
    WORKING_DIRECTORY "${SOURCE_ROOT}" TIMEOUT 30
    RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  if(NOT "${_result}" STREQUAL "0")
    message(FATAL_ERROR "contract --list failed (${_result}): ${_err}")
  endif()
  # Literal expected order from the original assertion, not executable output
  # used as its own expected value. CMake lists preserve the exact 46 names.
  set(_expected
    human_crank_torque pedaling_policy_valid_transition pedaling_ctor_domain
    shifter_ctor_domain assist_ctor_domain battery_ctor_domain freehub_ctor_domain
    writer_config_domains transmission_lifecycle_invariants
    typed_ctor_matches_validator_domain engine_fatal_status_and_reuse
    nested_engine_frames thread_local_engine_frames previous_tls_handler_restored
    owned_staging_owner_failure_frees_storage transmission_staged_ratio_atomicity
    interval_clock_contracts cruise_live_timestep model_access_contracts
    geometry_topology_and_ids geometry_nonfinite_transform profile_contract_boundaries
    support_geometry_parent_contracts settlement_sparse_last_atomicity
    drivetrain_allocation_enumeration cblas_and_norm closed_mode_enums
    force_component_layout warm_core_zero_allocation rtsan_invalid_status_control
    warm_distributed_zero_allocation distributed_status_control distributed_probe_isolation
    distributed_committed_snapshot_borrow warm_pneumatic_zero_allocation
    pneumatic_long_surface_zero_allocation pneumatic_status_control pneumatic_probe_isolation
    pneumatic_state_wire pneumatic_state_rejection_atomic pneumatic_stored_energy_and_snapshots
    warm_allocation_matrix rider_contacts_config_domain owned_episode_batch_ranges
    owned_episode_rejects_malformed_samples owned_episode_seal_batch_allocation_faults)
  string(JOIN "\n" _expected_text ${_expected})
  string(REPLACE "\r\n" "\n" _out "${_out}")
  if(NOT "${_out}" STREQUAL "${_expected_text}\n" AND
     NOT "${_out}" STREQUAL "${_expected_text}")
    message(FATAL_ERROR "registered contract cases or order differ:\n${_out}")
  endif()
elseif(CASE STREQUAL "test_contract_executable_source_is_in_cmake_manifest")
  if(NOT EXISTS "${SOURCE_MANIFEST}")
    message(FATAL_ERROR "native source manifest missing: ${SOURCE_MANIFEST}")
  endif()
  file(READ "${SOURCE_MANIFEST}" _manifest)
  string(JSON _count ERROR_VARIABLE _json_error LENGTH "${_manifest}" sources)
  if(NOT _json_error STREQUAL "NOTFOUND")
    message(FATAL_ERROR "invalid source manifest: ${_json_error}")
  endif()
  file(REAL_PATH "${SOURCE_ROOT}/native/tests/test_contracts.cpp" _expected_source)
  set(_matches 0)
  if(_count GREATER 0)
    math(EXPR _last "${_count} - 1")
    foreach(_index RANGE 0 ${_last})
      string(JSON _path GET "${_manifest}" sources ${_index} path)
      file(REAL_PATH "${_path}" _canonical BASE_DIRECTORY "${SOURCE_ROOT}")
      if(_canonical STREQUAL _expected_source)
        math(EXPR _matches "${_matches} + 1")
        string(JSON _target GET "${_manifest}" sources ${_index} target)
        if(NOT _target STREQUAL "native_contract_tests")
          message(FATAL_ERROR "contract source assigned to ${_target}")
        endif()
      endif()
    endforeach()
  endif()
  if(NOT _matches EQUAL 1)
    message(FATAL_ERROR "expected exactly one native contract source, got ${_matches}")
  endif()
else()
  message(FATAL_ERROR "unknown native loader contract: ${CASE}")
endif()
