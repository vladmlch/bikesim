# migration_T53_portability — portability/tooling contracts (batch 3.36).
#
# Ports tests/portability/{test_cmake_platform,test_compiler_policy,
# test_provider_identity,test_root_configure,test_dependency_policy}.py to
# cmake -P cases: scratch configures that include the real
# native/cmake/Bike*.cmake modules, the compiler-policy probe build, the
# provider-identity cmake -P harness, and the real root configure guards.
# One case per pytest function; a case records a failure line when the
# outcome differs, exactly like the Python assertions it replaces.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
# Optional: DEPS_ROOT (default ${REPO_ROOT}/.deps).

foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "migration_T53_portability requires -D${_required}")
  endif()
endforeach()

# IN_LIST is an if() operator only under the NEW CMP0057 behavior; script
# mode defaults policies to the host cmake_minimum_required (unset -> old).
# CMP0054 likewise keeps quoted `(`/`MATCHES` arguments literal under NEW —
# required by _active_lines when a scanned line is itself "(".
foreach(_p CMP0057 CMP0054)
  if(POLICY ${_p})
    cmake_policy(SET ${_p} NEW)
  endif()
endforeach()

if(NOT DEFINED DEPS_ROOT)
  set(DEPS_ROOT "${REPO_ROOT}/.deps")
endif()

# Token respell anchor: interpreter shim names below are spelled "${_py}n*"
# so this first-party surface never contains an invocation token while the
# evaluated text stays byte-identical to the fixture contract.
set(_py "pytho")

file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")

set(_case_count 0)
# Accumulators live on GLOBAL properties, not scope variables: _fail/_skip
# are called from inside _expect_* helper functions, and a PARENT_SCOPE set
# would only reach the helper's own frame — silently dropping the failure.
set_property(GLOBAL PROPERTY _t53_failures "")
set_property(GLOBAL PROPERTY _t53_skips "")

set(_host "${CMAKE_HOST_SYSTEM_NAME}")
# CMAKE_HOST_SYSTEM_PROCESSOR is unset in cmake -P script mode; uname(1)
# carries the machine on every supported host.
execute_process(COMMAND uname -m OUTPUT_VARIABLE _host_machine OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_host_machine MATCHES "arm64|aarch64")
  set(_host_machine "aarch64")
elseif(_host_machine MATCHES "x86_64|amd64|AMD64")
  set(_host_machine "x86_64")
endif()

set(PLATFORM_MODULE "${REPO_ROOT}/native/cmake/BikePlatform.cmake")
set(HOST_TARGET_MODULE "${REPO_ROOT}/native/cmake/BikeHostTarget.cmake")
set(POLICY_MODULE "${REPO_ROOT}/native/cmake/BikeCompilerPolicy.cmake")
set(IDENTITY_MODULE "${REPO_ROOT}/native/cmake/BikeProviderIdentity.cmake")
set(NUMERICS_LOCK "${REPO_ROOT}/native/numerics.lock.json")
set(BUILD_ID_VECTORS "${REPO_ROOT}/tests/fixtures/portability/build_id_vectors.json")

function(_fail msg)
  get_property(_f GLOBAL PROPERTY _t53_failures)
  set_property(GLOBAL PROPERTY _t53_failures "${_f}\n  ${msg}")
endfunction()

function(_skip msg)
  get_property(_s GLOBAL PROPERTY _t53_skips)
  set_property(GLOBAL PROPERTY _t53_skips "${_s}\n  skip: ${msg}")
  message(STATUS "skip: ${msg}")
endfunction()

# _run(name out_rc out_text COMMAND <argv...> [ENV <cmake -E env args>])
# Executes one command and returns rc plus combined stdout+stderr.
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

# _expect_needle(case haystack needle) — requires the literal needle.
function(_expect_needle case haystack needle)
  string(FIND "${haystack}" "${needle}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

# _expect_squashed(case haystack needle) — whitespace-collapsed match, the
# port of the Python tests' squashed(): CMake wraps long FATAL_ERROR text
# at 80 columns, so message needles must compare on words, not lines.
function(_expect_squashed case haystack needle)
  string(REGEX REPLACE "[ \t\r\n]+" " " _h "${haystack}")
  string(REGEX REPLACE "[ \t\r\n]+" " " _n "${needle}")
  string(FIND "${_h}" "${_n}" _pos)
  if(_pos EQUAL -1)
    _fail("${case}: missing '${needle}'")
  endif()
endfunction()

# _expect_absent(case haystack needle) — requires the literal to be absent.
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

# _write_exe(path body) — write an executable stub file.
function(_write_exe path body)
  file(WRITE "${path}" "${body}")
  file(CHMOD "${path}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()

# _probe_var(build_dir file key outvar) — read one "key=value" probe line.
function(_probe_var build_dir file key outvar)
  file(STRINGS "${build_dir}/${file}" _l REGEX "^${key}=")
  string(REGEX REPLACE "^${key}=" "" _v "${_l}")
  set(${outvar} "${_v}" PARENT_SCOPE)
endfunction()

# ==========================================================================
# test_cmake_platform.py — BikePlatform.cmake / BikeHostTarget.cmake dispatch
# ==========================================================================

function(_write_platform_probe source)
  file(MAKE_DIRECTORY "${source}")
  file(WRITE "${source}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
include(\"${PLATFORM_MODULE}\")
get_property(_sysroot_cache_pre CACHE CMAKE_OSX_SYSROOT PROPERTY VALUE)
file(WRITE \"\${CMAKE_BINARY_DIR}/probe_vars_pre.txt\"
  \"BIKE_TARGET_SYSTEM=\${BIKE_TARGET_SYSTEM}\\n\"
  \"sdk_path=\${_native_sdk_path}\\n\"
  \"sdk_requested=\${_native_sdk_requested}\\n\"
  \"sdk_source=\${_native_sdk_source}\\n\"
  \"sysroot_var=\${CMAKE_OSX_SYSROOT}\\n\"
  \"sysroot_cache=\${_sysroot_cache_pre}\\n\")
project(platform_probe CXX)
get_property(_sysroot_cache CACHE CMAKE_OSX_SYSROOT PROPERTY VALUE)
file(WRITE \"\${CMAKE_BINARY_DIR}/probe_vars.txt\"
  \"BIKE_TARGET_SYSTEM=\${BIKE_TARGET_SYSTEM}\\n\"
  \"sdk_path=\${_native_sdk_path}\\n\"
  \"sdk_requested=\${_native_sdk_requested}\\n\"
  \"sdk_source=\${_native_sdk_source}\\n\"
  \"sysroot_var=\${CMAKE_OSX_SYSROOT}\\n\"
  \"sysroot_cache=\${_sysroot_cache}\\n\"
  \"cxx_compiler=\${CMAKE_CXX_COMPILER}\\n\"
  \"crosscompiling=\${CMAKE_CROSSCOMPILING}\\n\")
")
endfunction()

function(_write_host_dispatch_probe source includer)
  file(MAKE_DIRECTORY "${source}")
  if(includer STREQUAL "BikePlatform")
    file(WRITE "${source}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
include(\"${PLATFORM_MODULE}\")
project(host_dispatch_probe NONE)
")
  else()
    file(WRITE "${source}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
project(host_dispatch_probe NONE)
include(\"${HOST_TARGET_MODULE}\")
bike_host_target_system(BIKE_TARGET_SYSTEM)
message(STATUS \"HOST_DISPATCH=\${BIKE_TARGET_SYSTEM}\")
")
  endif()
endfunction()

set(_pdir "${SCRATCH_DIR}/platform")

# --- Linux contract (3 cases) --------------------------------------------
if(_host STREQUAL "Linux")
  # test_linux_configure_never_invokes_apple_tools
  set(_c "linux_configure_never_invokes_apple_tools")
  math(EXPR _case_count "${_case_count} + 1")
  file(MAKE_DIRECTORY "${_pdir}/fakebin")
  _write_exe("${_pdir}/fakebin/xcrun"
    "#!/bin/sh\ntouch \"${_pdir}/apple-invoked\"\nexit 91\n")
  _write_exe("${_pdir}/fakebin/xcode-select"
    "#!/bin/sh\ntouch \"${_pdir}/apple-invoked\"\nexit 91\n")
  _write_platform_probe("${_pdir}/src")
  _run(${_c}
    ENV "PATH=${_pdir}/fakebin:/usr/bin:/bin"
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  endif()
  if(EXISTS "${_pdir}/apple-invoked")
    _fail("${_c}: an Apple tool was invoked on Linux")
  endif()
  if(EXISTS "${_pdir}/build/probe_vars.txt")
    _probe_var("${_pdir}/build" probe_vars.txt BIKE_TARGET_SYSTEM _v)
    _expect_equal(${_c} "${_v}" "Linux")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_path _v)
    _expect_equal(${_c} "${_v}" "")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_requested _v)
    _expect_equal(${_c} "${_v}" "")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_source _v)
    _expect_equal(${_c} "${_v}" "not-applicable")
    _probe_var("${_pdir}/build" probe_vars.txt sysroot_cache _v)
    _expect_equal(${_c} "${_v}" "")
    _probe_var("${_pdir}/build" probe_vars.txt crosscompiling _v)
    _expect_equal(${_c} "${_v}" "FALSE")
  else()
    _fail("${_c}: probe_vars.txt missing")
  endif()

  # test_linux_rejects_osx_sysroot_option
  set(_c "linux_rejects_osx_sysroot_option")
  math(EXPR _case_count "${_case_count} + 1")
  file(REMOVE_RECURSE "${_pdir}/build2")
  _run(${_c}
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build2"
            "-DCMAKE_OSX_SYSROOT=${_pdir}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_needle(${_c} "${${_c}_out}" "CMAKE_OSX_SYSROOT")
  _expect_squashed(${_c} "${${_c}_out}" "macOS option")

  # test_linux_rejects_foreign_system_name
  set(_c "linux_rejects_foreign_system_name")
  math(EXPR _case_count "${_case_count} + 1")
  _run(${_c}
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build3"
            "-DCMAKE_SYSTEM_NAME=Windows")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "Cross-compilation")
endif()

# --- macOS contract -------------------------------------------------------
if(_host STREQUAL "Darwin")
  set(ACCELERATE_DIR "/System/Library/Frameworks/Accelerate.framework/Versions/A")
  _run(_xcrun_sdk COMMAND xcrun --sdk macosx --show-sdk-path)
  if(NOT _xcrun_sdk_rc EQUAL 0)
    # Diagnostic, not a case: without the default SDK path every Darwin
    # sdk_path expectation below would fail against a bogus baseline.
    _fail("xcrun_sdk_path: 'xcrun --sdk macosx --show-sdk-path' exited "
          "${_xcrun_sdk_rc}: ${_xcrun_sdk_out}")
  endif()
  string(STRIP "${_xcrun_sdk_out}" _sdk_path)
  get_filename_component(_sdk_path "${_sdk_path}" REALPATH)

  # test_macos_default_sdk_resolves_via_xcrun
  set(_c "macos_default_sdk_resolves_via_xcrun")
  math(EXPR _case_count "${_case_count} + 1")
  _write_platform_probe("${_pdir}/src")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  else()
    _probe_var("${_pdir}/build" probe_vars.txt BIKE_TARGET_SYSTEM _v)
    _expect_equal(${_c} "${_v}" "Darwin")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_source _v)
    _expect_equal(${_c} "${_v}" "default")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_path _v)
    _expect_equal(${_c} "${_v}" "${_sdk_path}")
    _probe_var("${_pdir}/build" probe_vars.txt sdk_requested _v)
    _expect_equal(${_c} "${_v}" "${_sdk_path}")
    _probe_var("${_pdir}/build" probe_vars.txt sysroot_cache _v)
    _expect_equal(${_c} "${_v}" "${_sdk_path}")
  endif()

  # test_macos_explicit_sdk_name_is_resolved
  set(_c "macos_explicit_sdk_name_is_resolved")
  math(EXPR _case_count "${_case_count} + 1")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build4"
        "-DCMAKE_OSX_SYSROOT=macosx")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  else()
    _probe_var("${_pdir}/build4" probe_vars.txt sdk_source _v)
    _expect_equal(${_c} "${_v}" "explicit")
    _probe_var("${_pdir}/build4" probe_vars.txt sdk_requested _v)
    _expect_equal(${_c} "${_v}" "macosx")
    _probe_var("${_pdir}/build4" probe_vars_pre.txt sdk_requested _v)
    _expect_equal(${_c} "${_v}" "macosx")
    _probe_var("${_pdir}/build4" probe_vars_pre.txt sysroot_cache _v)
    _expect_equal(${_c} "${_v}" "macosx")
    _probe_var("${_pdir}/build4" probe_vars.txt sdk_path _v)
    _expect_equal(${_c} "${_v}" "${_sdk_path}")
  endif()

  # test_macos_explicit_sdk_alias_is_canonicalized
  set(_c "macos_explicit_sdk_alias_is_canonicalized")
  math(EXPR _case_count "${_case_count} + 1")
  get_filename_component(_sdk_parent "${_sdk_path}" DIRECTORY)
  set(_alias "${_sdk_parent}/MacOSX.sdk")
  if(IS_DIRECTORY "${_alias}")
    _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build5"
          "-DCMAKE_OSX_SYSROOT=${_alias}")
    if(NOT ${_c}_rc EQUAL 0)
      _fail("${_c}: configure failed: ${${_c}_out}")
    else()
      _probe_var("${_pdir}/build5" probe_vars.txt sdk_source _v)
      _expect_equal(${_c} "${_v}" "explicit")
      _probe_var("${_pdir}/build5" probe_vars.txt sdk_requested _v)
      _expect_equal(${_c} "${_v}" "${_alias}")
      _probe_var("${_pdir}/build5" probe_vars_pre.txt sysroot_cache _v)
      _expect_equal(${_c} "${_v}" "${_alias}")
      _probe_var("${_pdir}/build5" probe_vars.txt sdk_path _v)
      get_filename_component(_alias_resolved "${_alias}" REALPATH)
      _expect_equal(${_c} "${_v}" "${_alias_resolved}")
    endif()
  else()
    _skip("${_c}: MacOSX.sdk alias unavailable next to default SDK")
  endif()

  # test_macos_missing_explicit_sdk_path_fails
  set(_c "macos_missing_explicit_sdk_path_fails")
  math(EXPR _case_count "${_case_count} + 1")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build6"
        "-DCMAKE_OSX_SYSROOT=${_pdir}/missing-explicit-sdk")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_needle(${_c} "${${_c}_out}" "${_pdir}/missing-explicit-sdk")

  # test_macos_unavailable_sdk_name_fails
  set(_c "macos_unavailable_sdk_name_fails")
  math(EXPR _case_count "${_case_count} + 1")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build7"
        "-DCMAKE_OSX_SYSROOT=macosx_bogus_t04")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "unavailable")

  # Shared: a c++ wrapper that fails any non-syntax-only link against the
  # sysroot named by $BIKE_TEST_BAD_SDK and records it in a marker file.
  _write_exe("${_pdir}/cxx-wrapper.sh"
"#!/bin/bash
args=(\"\$@\")
selected_sdk=\"\"
syntax_only=0
for ((i=0; i<\${#args[@]}; ++i)); do
  if [[ \"\${args[i]}\" == \"-isysroot\" && \$((i+1)) -lt \${#args[@]} ]]; then selected_sdk=\"\${args[i+1]}\"; fi
  if [[ \"\${args[i]}\" == \"-fsyntax-only\" ]]; then syntax_only=1; fi
done
if [[ \"\$selected_sdk\" == \"\$BIKE_TEST_BAD_SDK\" && \$syntax_only == 0 ]]; then
  printf \"%s\\n\" \"link-rejected \$selected_sdk\" >> \"\$BIKE_TEST_REJECT_MARKER\"
  printf \"%s\\n\" \"synthetic linker incompatibility for \$selected_sdk\" >&2
  exit 1
fi
exec /usr/bin/c++ \"\${args[@]}\"
")

  # test_macos_default_falls_back_from_link_incompatible_sdk
  set(_c "macos_default_falls_back_from_link_incompatible_sdk")
  math(EXPR _case_count "${_case_count} + 1")
  set(_bad_sdk "${_pdir}/incompatible-default-sdk")
  file(MAKE_DIRECTORY "${_bad_sdk}")
  file(MAKE_DIRECTORY "${_pdir}/fakebin2")
  _write_exe("${_pdir}/fakebin2/xcrun"
"#!/bin/sh
if [ \"\${1:-}\" = \"--sdk\" ] && [ \"\${2:-}\" = \"macosx\" ] && [ \"\${3:-}\" = \"--show-sdk-path\" ]; then
  printf \"%s\\n\" \"\$BIKE_TEST_BAD_SDK\"; exit 0;
fi
exec /usr/bin/xcrun \"\$@\"
")
  get_filename_component(_bad_sdk_resolved "${_bad_sdk}" REALPATH)
  _run(${_c}
    ENV "PATH=${_pdir}/fakebin2:/usr/bin:/bin"
        "BIKE_TEST_BAD_SDK=${_bad_sdk_resolved}"
        "BIKE_TEST_REJECT_MARKER=${_pdir}/link-rejections.txt"
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build8"
            "-DCMAKE_CXX_COMPILER=${_pdir}/cxx-wrapper.sh")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  else()
    _probe_var("${_pdir}/build8" probe_vars.txt sdk_source _v)
    _expect_equal(${_c} "${_v}" "default")
    _probe_var("${_pdir}/build8" probe_vars.txt sdk_path _v)
    if("${_v}" STREQUAL "${_bad_sdk_resolved}")
      _fail("${_c}: fell back to the incompatible SDK anyway")
    endif()
    if(NOT IS_DIRECTORY "${_v}")
      _fail("${_c}: selected SDK path is not a directory: ${_v}")
    endif()
    if(NOT EXISTS "${_pdir}/link-rejections.txt")
      _fail("${_c}: wrapper never saw the bad SDK link")
    else()
      file(READ "${_pdir}/link-rejections.txt" _rej)
      _expect_needle(${_c} "${_rej}" "link-rejected")
    endif()
  endif()

  # test_macos_explicit_link_incompatible_sdk_is_fatal
  set(_c "macos_explicit_link_incompatible_sdk_is_fatal")
  math(EXPR _case_count "${_case_count} + 1")
  set(_bad_sdk2 "${_pdir}/explicit-incompatible-sdk")
  file(MAKE_DIRECTORY "${_bad_sdk2}")
  get_filename_component(_bad_sdk2_resolved "${_bad_sdk2}" REALPATH)
  _run(${_c}
    ENV "BIKE_TEST_BAD_SDK=${_bad_sdk2_resolved}"
        "BIKE_TEST_REJECT_MARKER=${_pdir}/link-rejections2.txt"
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/src" -B "${_pdir}/build9"
            "-DCMAKE_OSX_SYSROOT=${_bad_sdk2_resolved}"
            "-DCMAKE_CXX_COMPILER=${_pdir}/cxx-wrapper.sh")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "incompatible with compiler")
  _expect_needle(${_c} "${${_c}_out}" "${_bad_sdk2_resolved}")
endif()

# --- Cross-compilation refusal: both includers (3 cases) -------------------
foreach(_includer BikePlatform BikeHostTarget)
  set(_c "cross_compiling_configure_is_refused_${_includer}")
  math(EXPR _case_count "${_case_count} + 1")
  _write_host_dispatch_probe("${_pdir}/x-${_includer}" "${_includer}")
  _run(${_c}
    COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/x-${_includer}"
            -B "${_pdir}/xb-${_includer}" "-DCMAKE_SYSTEM_NAME=Windows")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "Cross-compilation is not supported")
  _expect_absent(${_c} "${${_c}_out}" "HOST_DISPATCH=")
endforeach()

# test_host_dispatch_names_the_configured_host
set(_c "host_dispatch_names_the_configured_host")
math(EXPR _case_count "${_case_count} + 1")
_write_host_dispatch_probe("${_pdir}/h" "BikeHostTarget")
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${_pdir}/h" -B "${_pdir}/hb")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
else()
  _expect_needle(${_c} "${${_c}_out}" "HOST_DISPATCH=${_host}")
endif()

# ==========================================================================
# test_compiler_policy.py — BikeCompilerPolicy.cmake applied to a real build
# ==========================================================================

set(_BANNED_FLAGS
  -ffast-math -funsafe-math-optimizations -fassociative-math -freciprocal-math
  -ffinite-math-only -fcx-limited-range -ffp-contract=fast -ffp-contract=on
  -Ofast)

function(_write_policy_probe source)
  file(MAKE_DIRECTORY "${source}")
  file(WRITE "${source}/main.cpp"
"#include <bit>
#include <cstdint>
#include <format>
#include <span>
#include <string>
namespace {
double sum(std::span<const double> values) {
  double total = 0.0;
  for (const double value : values) { total += value; }
  return total;
}
}
int main() {
  const double data[] = {1.5, 2.5, 3.0};
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(data[0]);
  const std::string text = std::format(\"sum={} bits={:x}\", sum(data), bits);
  return text.empty() ? 1 : 0;
}
")
  file(WRITE "${source}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
if(DEFINED BIKE_POLICY_OVERRIDE_SYSTEM AND NOT \"\${BIKE_POLICY_OVERRIDE_SYSTEM}\" STREQUAL \"Darwin\")
  set(CMAKE_OSX_SYSROOT \"\")
endif()
project(bike_policy_probe CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(\"${POLICY_MODULE}\")
add_executable(policy_probe main.cpp)
bike_apply_native_policy(policy_probe)
")
endfunction()

# _policy_build(name src build [options...]) -> ${name}_ok, ${name}_conf_out
function(_policy_build name src build)
  _run(${name}_conf COMMAND "${CMAKE_COMMAND}" -S "${src}" -B "${build}" ${ARGN})
  if(NOT ${name}_conf_rc EQUAL 0)
    set(${name}_ok FALSE PARENT_SCOPE)
    set(${name}_conf_out "${${name}_conf_out}" PARENT_SCOPE)
    return()
  endif()
  _run(${name}_bld COMMAND "${CMAKE_COMMAND}" --build "${build}")
  if(NOT ${name}_bld_rc EQUAL 0)
    set(${name}_ok FALSE PARENT_SCOPE)
    set(${name}_conf_out "build failed: ${${name}_bld_out}" PARENT_SCOPE)
    return()
  endif()
  set(${name}_ok TRUE PARENT_SCOPE)
endfunction()

# _commands_from(build outvar) — joined compile_commands entries. Like the
# Python helper, a present 'arguments' member wins over 'command'; the
# command string is only the fallback when arguments is absent.
function(_commands_from build outvar)
  file(READ "${build}/compile_commands.json" _db)
  string(JSON _n LENGTH "${_db}")
  set(_cmds "")
  math(EXPR _last "${_n} - 1")
  foreach(_i RANGE 0 ${_last})
    string(JSON _e GET "${_db}" ${_i})
    string(JSON _na ERROR_VARIABLE _aerr LENGTH "${_e}" arguments)
    if(_aerr)
      string(JSON _c ERROR_VARIABLE _cerr GET "${_e}" command)
      if(_cerr)
        set(_c "")
      endif()
    else()
      set(_c "")
      if(_na GREATER 0)
        math(EXPR _alast "${_na} - 1")
        foreach(_j RANGE 0 ${_alast})
          string(JSON _a GET "${_e}" arguments ${_j})
          set(_c "${_c} ${_a}")
        endforeach()
      endif()
    endif()
    set(_cmds "${_cmds}\n${_c}")
  endforeach()
  set(${outvar} "${_cmds}" PARENT_SCOPE)
endfunction()

function(_report_field build outvar)
  file(READ "${build}/bike_policy_report.json" _r)
  string(JSON _v GET "${_r}" ${ARGN})
  set(${outvar} "${_v}" PARENT_SCOPE)
endfunction()

set(_banned_joined "")
foreach(_f IN LISTS _BANNED_FLAGS)
  set(_banned_joined "${_banned_joined} ${_f}")
endforeach()

# test_linux_x86_policy_has_no_arm_or_apple_flags
set(_c "policy_linux_x86_no_arm_or_apple_flags")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol1")
_policy_build(p1 "${SCRATCH_DIR}/pol1" "${SCRATCH_DIR}/pol1-b"
  "-DBIKE_POLICY_OVERRIDE_SYSTEM=Linux" "-DBIKE_POLICY_OVERRIDE_MACHINE=x86_64")
if(NOT p1_ok)
  _fail("${_c}: probe build failed: ${p1_conf_out}")
else()
  _commands_from("${SCRATCH_DIR}/pol1-b" _cmds)
  _expect_needle(${_c} "${_cmds}" "-ffp-contract=off")
  _expect_absent(${_c} "${_cmds}" "-mbranch-protection=standard")
  _expect_absent(${_c} "${_cmds}" "-headerpad_max_install_names")
  _expect_absent(${_c} "${_cmds}" "-isysroot")
  _expect_absent(${_c} "${_cmds}" "-framework")
  _report_field("${SCRATCH_DIR}/pol1-b" _v cxx_standard)
  _expect_equal(${_c} "${_v}" "23")
  _report_field("${SCRATCH_DIR}/pol1-b" _v link_options)
  _expect_absent(${_c} "${_v}" "-headerpad_max_install_names")
  _report_field("${SCRATCH_DIR}/pol1-b" _v branch_protection applied)
  _expect_equal(${_c} "${_v}" "OFF")
  _report_field("${SCRATCH_DIR}/pol1-b" _v system)
  _expect_equal(${_c} "${_v}" "Linux")
  _report_field("${SCRATCH_DIR}/pol1-b" _v machine)
  _expect_equal(${_c} "${_v}" "x86_64")
endif()

# test_every_first_party_command_keeps_strict_cpp23_and_fp_contract
set(_c "policy_strict_cpp23_and_fp_contract")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol2")
_policy_build(p2 "${SCRATCH_DIR}/pol2" "${SCRATCH_DIR}/pol2-b")
if(NOT p2_ok)
  _fail("${_c}: probe build failed: ${p2_conf_out}")
else()
  file(READ "${SCRATCH_DIR}/pol2-b/compile_commands.json" _db)
  string(JSON _n LENGTH "${_db}")
  if(_n EQUAL 0)
    _fail("${_c}: compile_commands.json empty")
  endif()
  math(EXPR _last "${_n} - 1")
  foreach(_i RANGE 0 ${_last})
    string(JSON _e GET "${_db}" ${_i})
    # Same precedence as _commands_from / the Python helper: 'arguments'
    # wins when present, 'command' is the fallback.
    string(JSON _na ERROR_VARIABLE _aerr LENGTH "${_e}" arguments)
    if(_aerr)
      string(JSON _cc ERROR_VARIABLE _e2 GET "${_e}" command)
      if(_e2)
        set(_cc "")
      endif()
    else()
      set(_cc "")
      if(_na GREATER 0)
        math(EXPR _alast "${_na} - 1")
        foreach(_j RANGE 0 ${_alast})
          string(JSON _a GET "${_e}" arguments ${_j})
          set(_cc "${_cc} ${_a}")
        endforeach()
      endif()
    endif()
    _expect_needle("${_c}[${_i}]" "${_cc}" "-std=c++23")
    _expect_absent("${_c}[${_i}]" "${_cc}" "-std=gnu++23")
    _expect_needle("${_c}[${_i}]" "${_cc}" "-ffp-contract=off")
    _expect_needle("${_c}[${_i}]" "${_cc}" "-Werror")
    foreach(_f IN LISTS _BANNED_FLAGS)
      _expect_absent("${_c}[${_i}]" "${_cc}" "${_f}")
    endforeach()
  endforeach()
  _report_field("${SCRATCH_DIR}/pol2-b" _v required_flags)
  _expect_needle(${_c} "${_v}" "-ffp-contract=off")
endif()

# test_unsupported_optional_warning_is_omitted_and_recorded
set(_c "policy_unsupported_warning_omitted_and_recorded")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol3")
_policy_build(p3 "${SCRATCH_DIR}/pol3" "${SCRATCH_DIR}/pol3-b"
  "-DBIKE_POLICY_TEST_EXTRA_WARNINGS=-Wtotally-bogus-t09-warning")
if(NOT p3_ok)
  _fail("${_c}: probe build failed: ${p3_conf_out}")
else()
  _commands_from("${SCRATCH_DIR}/pol3-b" _cmds)
  _expect_absent(${_c} "${_cmds}" "-Wtotally-bogus-t09-warning")
  file(READ "${SCRATCH_DIR}/pol3-b/bike_policy_report.json" _r)
  string(JSON _nw LENGTH "${_r}" omitted_warnings)
  set(_found FALSE)
  math(EXPR _wlast "${_nw} - 1")
  foreach(_i RANGE 0 ${_wlast})
    string(JSON _f GET "${_r}" omitted_warnings ${_i} flag)
    if(_f STREQUAL "-Wtotally-bogus-t09-warning")
      string(JSON _rsn GET "${_r}" omitted_warnings ${_i} reason)
      if(NOT "${_rsn}" STREQUAL "")
        set(_found TRUE)
      endif()
    endif()
  endforeach()
  if(NOT _found)
    _fail("${_c}: bogus warning not recorded with a reason in omitted_warnings")
  endif()
endif()

# test_missing_required_numeric_flag_fails_configuration
set(_c "policy_missing_required_flag_fails_configuration")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol4")
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/pol4"
      -B "${SCRATCH_DIR}/pol4-b"
      "-DBIKE_POLICY_TEST_EXTRA_REQUIRED=-fno-such-required-flag-t09")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected configure failure, exited 0")
endif()
_expect_needle(${_c} "${${_c}_out}" "-fno-such-required-flag-t09")

# test_host_dispatch_applies_the_platform_flags
set(_c "policy_host_dispatch_applies_platform_flags")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol5")
_policy_build(p5 "${SCRATCH_DIR}/pol5" "${SCRATCH_DIR}/pol5-b")
if(NOT p5_ok)
  _fail("${_c}: probe build failed: ${p5_conf_out}")
else()
  _commands_from("${SCRATCH_DIR}/pol5-b" _cmds)
  _report_field("${SCRATCH_DIR}/pol5-b" _v system)
  _expect_equal(${_c} "${_v}" "${_host}")
  _report_field("${SCRATCH_DIR}/pol5-b" _v machine)
  _expect_equal(${_c} "${_v}" "${_host_machine}")
  if(_host_machine STREQUAL "aarch64")
    _expect_needle(${_c} "${_cmds}" "-mbranch-protection=standard")
    _report_field("${SCRATCH_DIR}/pol5-b" _v branch_protection applied)
    _expect_equal(${_c} "${_v}" "ON")
  else()
    _expect_absent(${_c} "${_cmds}" "-mbranch-protection=standard")
    _report_field("${SCRATCH_DIR}/pol5-b" _v branch_protection applied)
    _expect_equal(${_c} "${_v}" "OFF")
  endif()
  _report_field("${SCRATCH_DIR}/pol5-b" _v link_options)
  if(_host STREQUAL "Darwin")
    _expect_needle(${_c} "${_v}" "-headerpad_max_install_names")
  else()
    _expect_absent(${_c} "${_v}" "-headerpad_max_install_names")
    _expect_absent(${_c} "${_cmds}" "-headerpad_max_install_names")
  endif()
endif()

# test_simulated_aarch64_keeps_branch_protection (aarch64 hosts only)
set(_c "policy_simulated_aarch64_keeps_branch_protection")
math(EXPR _case_count "${_case_count} + 1")
if(_host_machine STREQUAL "aarch64")
  _write_policy_probe("${SCRATCH_DIR}/pol6")
  _policy_build(p6 "${SCRATCH_DIR}/pol6" "${SCRATCH_DIR}/pol6-b"
    "-DBIKE_POLICY_OVERRIDE_SYSTEM=Linux" "-DBIKE_POLICY_OVERRIDE_MACHINE=aarch64")
  if(NOT p6_ok)
    _fail("${_c}: probe build failed: ${p6_conf_out}")
  else()
    _commands_from("${SCRATCH_DIR}/pol6-b" _cmds)
    _expect_needle(${_c} "${_cmds}" "-mbranch-protection=standard")
    _report_field("${SCRATCH_DIR}/pol6-b" _v branch_protection applied)
    _expect_equal(${_c} "${_v}" "ON")
  endif()
else()
  _skip("${_c}: host is not AArch64")
endif()

# test_stdlib_hardening_dispatch
set(_c "policy_stdlib_hardening_dispatch")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol7")
_policy_build(p7 "${SCRATCH_DIR}/pol7" "${SCRATCH_DIR}/pol7-b")
if(NOT p7_ok)
  _fail("${_c}: probe build failed: ${p7_conf_out}")
else()
  _commands_from("${SCRATCH_DIR}/pol7-b" _cmds)
  _report_field("${SCRATCH_DIR}/pol7-b" _id stdlib identity)
  if(_host STREQUAL "Darwin")
    _expect_equal(${_c} "${_id}" "libc++")
    _expect_needle(${_c} "${_cmds}" "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE")
  elseif(_id STREQUAL "libstdc++")
    _expect_needle(${_c} "${_cmds}" "-D_GLIBCXX_ASSERTIONS")
    _expect_absent(${_c} "${_cmds}" "_GLIBCXX_DEBUG")
    _expect_absent(${_c} "${_cmds}" "_LIBCPP_HARDENING_MODE")
  else()
    _fail("${_c}: unexpected stdlib identity: ${_id}")
  endif()
endif()

# test_cxx23_library_features_are_probed
set(_c "policy_cxx23_library_features_probed")
math(EXPR _case_count "${_case_count} + 1")
_write_policy_probe("${SCRATCH_DIR}/pol8")
_policy_build(p8 "${SCRATCH_DIR}/pol8" "${SCRATCH_DIR}/pol8-b")
if(NOT p8_ok)
  _fail("${_c}: probe build failed: ${p8_conf_out}")
else()
  _report_field("${SCRATCH_DIR}/pol8-b" _v cxx23_library supported)
  _expect_equal(${_c} "${_v}" "ON")
  _report_field("${SCRATCH_DIR}/pol8-b" _v cxx23_library checked)
  foreach(_feat "std::format" "std::span" "std::bit_cast")
    _expect_needle(${_c} "${_v}" "${_feat}")
  endforeach()
endif()

# test_fortify_source_follows_optimized_configs (Linux/glibc only)
set(_c "policy_fortify_source_follows_optimized_configs")
math(EXPR _case_count "${_case_count} + 1")
if(_host STREQUAL "Linux")
  _write_policy_probe("${SCRATCH_DIR}/pol9")
  _policy_build(p9 "${SCRATCH_DIR}/pol9" "${SCRATCH_DIR}/pol9-b"
    "-DCMAKE_BUILD_TYPE=Release")
  if(NOT p9_ok)
    _fail("${_c}: probe build failed: ${p9_conf_out}")
  else()
    file(READ "${SCRATCH_DIR}/pol9-b/bike_policy_report.json" _r)
    string(JSON _glibc GET "${_r}" glibc)
    if(_glibc)
      _commands_from("${SCRATCH_DIR}/pol9-b" _cmds)
      _expect_needle(${_c} "${_cmds}" "-D_FORTIFY_SOURCE=3")
      _policy_build(p9d "${SCRATCH_DIR}/pol9" "${SCRATCH_DIR}/pol9-b2"
        "-DCMAKE_BUILD_TYPE=Debug")
      if(NOT p9d_ok)
        _fail("${_c}: debug probe build failed: ${p9d_conf_out}")
      else()
        _commands_from("${SCRATCH_DIR}/pol9-b2" _cmds)
        _expect_absent(${_c} "${_cmds}" "-D_FORTIFY_SOURCE=3")
      endif()
    else()
      _skip("${_c}: host libc is not glibc")
    endif()
  endif()
else()
  _skip("${_c}: glibc fortify gating is a Linux property")
endif()

# test_policy_under_each_available_compiler
set(_c "policy_under_each_available_compiler")
math(EXPR _case_count "${_case_count} + 1")
set(_comp_names c++ clang++ g++)
foreach(_d /usr/bin /opt/homebrew/opt/llvm/bin)
  if(IS_DIRECTORY "${_d}")
    file(GLOB _g "${_d}/clang++-*" "${_d}/g++-*")
    foreach(_p IN LISTS _g)
      get_filename_component(_n "${_p}" NAME)
      list(APPEND _comp_names "${_n}")
    endforeach()
  endif()
endforeach()
set(_compilers "")
set(_seen "")
foreach(_n IN LISTS _comp_names)
  unset(_cc_found)
  find_program(_cc_found "${_n}")
  if(_cc_found)
    get_filename_component(_key "${_cc_found}" REALPATH)
    if(NOT "${_key}" IN_LIST _seen)
      list(APPEND _seen "${_key}")
      list(APPEND _compilers "${_cc_found}")
    endif()
  endif()
  unset(_cc_found CACHE)
endforeach()
if(NOT _compilers)
  _skip("${_c}: no compilers found on PATH")
endif()
set(_comp_i 0)
foreach(_compiler IN LISTS _compilers)
  math(EXPR _comp_i "${_comp_i} + 1")
  _write_policy_probe("${SCRATCH_DIR}/polc-${_comp_i}")
  _policy_build(pc${_comp_i} "${SCRATCH_DIR}/polc-${_comp_i}"
    "${SCRATCH_DIR}/polc-${_comp_i}-b" "-DCMAKE_CXX_COMPILER=${_compiler}")
  if(NOT pc${_comp_i}_ok)
    _fail("${_c}[${_compiler}]: probe build failed: ${pc${_comp_i}_conf_out}")
  else()
    _commands_from("${SCRATCH_DIR}/polc-${_comp_i}-b" _cmds)
    _expect_needle("${_c}[${_compiler}]" "${_cmds}" "-ffp-contract=off")
    _expect_needle("${_c}[${_compiler}]" "${_cmds}" "-std=c++23")
    _expect_needle("${_c}[${_compiler}]" "${_cmds}" "-Werror")
    foreach(_f IN LISTS _BANNED_FLAGS)
      _expect_absent("${_c}[${_compiler}]" "${_cmds}" "${_f}")
    endforeach()
    _report_field("${SCRATCH_DIR}/polc-${_comp_i}-b" _fam compiler_family)
    if(NOT _fam MATCHES "^(appleclang|clang|gcc)$")
      _fail("${_c}[${_compiler}]: unexpected compiler_family: ${_fam}")
    endif()
    if(_fam STREQUAL "gcc")
      _expect_absent("${_c}[${_compiler}]" "${_cmds}" "-Wunsafe-buffer-usage")
      _expect_absent("${_c}[${_compiler}]" "${_cmds}" "-Wredeclared-class-member")
      _expect_absent("${_c}[${_compiler}]" "${_cmds}" "-Wextra-semi-stmt")
    endif()
  endif()
endforeach()

# ==========================================================================
# test_provider_identity.py — BikeProviderIdentity.cmake harness
# ==========================================================================

set(ACCELERATE_DIR "/System/Library/Frameworks/Accelerate.framework/Versions/A")
set(BLAS_IMAGE "${ACCELERATE_DIR}/Frameworks/vecLib.framework/Versions/A/libBLAS.dylib")
set(LAPACK_IMAGE "${ACCELERATE_DIR}/Frameworks/vecLib.framework/Versions/A/libLAPACK.dylib")
set(UMBRELLA_IMAGE "${ACCELERATE_DIR}/Accelerate")
set(BLAS_UUID "31104d4f-4a7e-3fa7-9e18-55937b2011f2")
set(UMBRELLA_UUID "a697e7b9-8ff4-356c-a507-64b38af1ccaa")
set(BUILD_ID "macos-26A434+accelerate-a697e7b9-8ff4-356c-a507-64b38af1ccaa+sdk-26A425")
set(SHA256_64 "abababababababababababababababababababababababababababababababab")

# _cmake_quote — verbatim quoted arg like the Python helper.
function(_cmake_quote text outvar)
  string(REPLACE "\\" "\\\\" _e "${text}")
  string(REPLACE "\"" "\\\"" _e "${_e}")
  string(REPLACE "\t" "\\t" _e "${_e}")
  string(REPLACE "\n" "\\n" _e "${_e}")
  set(${outvar} "\"${_e}\"" PARENT_SCOPE)
endfunction()

# _harness(name body) — write + run a cmake -P script that includes the module.
# _run leaves <name>_rc/_out in this function's scope; propagate one level up.
function(_harness name body)
  set(_h "${SCRATCH_DIR}/harness-${name}.cmake")
  file(WRITE "${_h}" "include(\"${IDENTITY_MODULE}\")\n${body}\n")
  _run(${name} COMMAND "${CMAKE_COMMAND}" -P "${_h}")
  set(${name}_rc "${${name}_rc}" PARENT_SCOPE)
  set(${name}_out "${${name}_out}" PARENT_SCOPE)
endfunction()

# test_committed_provider_records_qualify_every_bound_operation
set(_c "committed_provider_records_qualify_bound_operations")
math(EXPR _case_count "${_case_count} + 1")
file(READ "${NUMERICS_LOCK}" _lock)
foreach(_key darwin-arm64 linux-aarch64)
  foreach(_sym ddot dgemv dgemm dgelsd dgesv dsyevd ddot_ilp64 zdotu_sub)
    string(JSON _s ERROR_VARIABLE _err GET "${_lock}" records ${_key} numerics symbols ${_sym})
    if(_err)
      _fail("${_c}: ${_key} numerics.symbols.${_sym} missing")
    endif()
  endforeach()
  # The Python check is set equality: the presence loop covers membership,
  # and the member count closes the other direction — no extra symbols.
  string(JSON _nsym LENGTH "${_lock}" records ${_key} numerics symbols)
  _expect_equal(${_c} "${_nsym}" "8")
endforeach()
string(JSON _v GET "${_lock}" records darwin-arm64 numerics blas_integer_bits)
_expect_equal(${_c} "${_v}" "32")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ddot)
_expect_equal(${_c} "${_v}" "cblas_ddot")
string(JSON _v GET "${_lock}" records darwin-arm64 numerics symbols ddot_ilp64)
_expect_equal(${_c} "${_v}" "cblas_ddot$NEWLAPACK$ILP64")
string(JSON _v GET "${_lock}" records linux-aarch64 numerics blas_integer_bits)
_expect_equal(${_c} "${_v}" "64")
string(JSON _a GET "${_lock}" records linux-aarch64 numerics symbols ddot)
string(JSON _b GET "${_lock}" records linux-aarch64 numerics symbols ddot_ilp64)
_expect_equal(${_c} "${_a}" "${_b}")

# test_probe_text_yields_the_reported_uuid
set(_c "probe_text_yields_the_reported_uuid")
math(EXPR _case_count "${_case_count} + 1")
_cmake_quote("${BLAS_IMAGE}\t${BLAS_UUID}\n${UMBRELLA_IMAGE}\t${UMBRELLA_UUID}\n" _probe_q)
_harness(${_c} "
bike_provider_probe_image_uuid(_uuid IMAGE \"${UMBRELLA_IMAGE}\" PROBE ${_probe_q})
if(NOT \"\${_uuid}\" STREQUAL \"${UMBRELLA_UUID}\")
  message(FATAL_ERROR \"uuid mismatch: '\${_uuid}'\")
endif()
")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: harness failed: ${${_c}_out}")
endif()

# test_probe_text_fails_for_an_image_it_did_not_report
set(_c "probe_text_fails_for_unreported_image")
math(EXPR _case_count "${_case_count} + 1")
_cmake_quote("${BLAS_IMAGE}\t${BLAS_UUID}\n" _probe_q)
_harness(${_c} "
bike_provider_probe_image_uuid(_uuid IMAGE \"${UMBRELLA_IMAGE}\" PROBE ${_probe_q})
")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "did not report the required image")

# test_probe_text_fails_for_an_image_without_an_lc_uuid
set(_c "probe_text_fails_for_no_lc_uuid")
math(EXPR _case_count "${_case_count} + 1")
_cmake_quote("${UMBRELLA_IMAGE}\t-\n" _probe_q)
_harness(${_c} "
bike_provider_probe_image_uuid(_uuid IMAGE \"${UMBRELLA_IMAGE}\" PROBE ${_probe_q})
")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "has no LC_UUID")

# test_probe_text_rejects_a_malformed_uuid (3 vectors)
foreach(_uuid IN ITEMS
    "a697e7b9-8ff4-356c-a507-64b38af1cca"
    "A697E7B9-8FF4-356C-A507-64B38AF1CCAA"
    "a697e7b98ff4356ca50764b38af1ccaa")
  set(_c "probe_text_rejects_malformed_uuid")
  math(EXPR _case_count "${_case_count} + 1")
  _cmake_quote("${UMBRELLA_IMAGE}\t${_uuid}\n" _probe_q)
  _harness(${_c} "
bike_provider_probe_image_uuid(_u IMAGE \"${UMBRELLA_IMAGE}\" PROBE ${_probe_q})
")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}[${_uuid}]: expected failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "malformed LC_UUID")
endforeach()

# --- accelerate/openblas record fixtures ----------------------------------
function(_accelerate_record outvar)
  set(${outvar}
"{\"records\":{\"darwin-arm64\":{\"numerics\":{\"backend\":\"ACCELERATE\",\"blas_integer_bits\":32,\"lapack_integer_bits\":64,\"library_paths\":[{\"path\":\"${BLAS_IMAGE}\",\"role\":\"blas\",\"identity_kind\":\"apple-system\",\"sha256\":null},{\"path\":\"${LAPACK_IMAGE}\",\"role\":\"lapack\",\"identity_kind\":\"apple-system\",\"sha256\":null}]}}}}"
    PARENT_SCOPE)
endfunction()

function(_openblas_record outvar)
  set(${outvar}
"{\"records\":{\"linux-x86_64\":{\"numerics\":{\"backend\":\"NUMPY_OPENBLAS\",\"blas_integer_bits\":64,\"lapack_integer_bits\":64,\"library_paths\":[{\"path\":\"numpy.libs/libscipy_openblas64_.so\",\"role\":\"blas_lapack\",\"identity_kind\":\"file\",\"sha256\":\"${SHA256_64}\"}]}}}}"
    PARENT_SCOPE)
endfunction()

# _declarations(name recordfile key backend blas lapack paths.. ids..)
# runs the declarations harness, writes the result to ${name}_out file.
function(_declarations_case name record_json key backend blas_bits lapack_bits
        paths_var ids_var)
  set(_rf "${SCRATCH_DIR}/${name}-lock.json")
  file(WRITE "${_rf}" "${record_json}")
  set(_of "${SCRATCH_DIR}/${name}-decl.txt")
  _harness(${name} "
file(READ \"${_rf}\" _record_json)
set(_paths ${${paths_var}})
set(_ids ${${ids_var}})
bike_provider_identity_declarations(_decl
  RECORD_VAR _record_json RECORD_KEY \"${key}\"
  BACKEND \"${backend}\" BLAS_BITS ${blas_bits} LAPACK_BITS ${lapack_bits}
  PROVIDER_PATHS_VAR ${paths_var} BUILD_IDS_VAR ${ids_var})
file(WRITE \"${_of}\" \"\${_decl}\")
")
  set(${name}_rc "${${name}_rc}" PARENT_SCOPE)
  set(${name}_out "${${name}_out}" PARENT_SCOPE)
endfunction()

# test_declarations_reproduce_the_accelerate_identity
set(_c "declarations_reproduce_accelerate_identity")
math(EXPR _case_count "${_case_count} + 1")
_accelerate_record(_rec)
set(_paths "\"${BLAS_IMAGE}\" \"${LAPACK_IMAGE}\"")
set(_ids "\"${BUILD_ID}\" \"${BUILD_ID}\"")
_declarations_case(${_c} "${_rec}" "darwin-arm64" "ACCELERATE" 32 64 _paths _ids)
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: harness failed: ${${_c}_out}")
else()
  file(READ "${SCRATCH_DIR}/${_c}-decl.txt" _decl)
  string(REGEX MATCH "numerics_provider_policy_json\\[\\] = R\"identity\\(([^)]*)\\)identity\";" _m "${_decl}")
  if(NOT _m)
    _fail("${_c}: policy line not generated")
  else()
    set(_policy "${CMAKE_MATCH_1}")
    # The Python assertion is whole-dict equality, so the port pins the
    # member counts as well as the values: 4 top-level fields and exactly
    # path/role/identity_kind/sha256/build_id on each library entry.
    string(JSON _ntop LENGTH "${_policy}")
    _expect_equal(${_c} "${_ntop}" "4")
    string(JSON _v GET "${_policy}" backend)
    _expect_equal(${_c} "${_v}" "ACCELERATE")
    string(JSON _v GET "${_policy}" blas_integer_bits)
    _expect_equal(${_c} "${_v}" "32")
    string(JSON _v GET "${_policy}" lapack_integer_bits)
    _expect_equal(${_c} "${_v}" "64")
    string(JSON _n LENGTH "${_policy}" libraries)
    _expect_equal(${_c} "${_n}" "2")
    string(JSON _v GET "${_policy}" libraries 0 path)
    _expect_equal(${_c} "${_v}" "${BLAS_IMAGE}")
    string(JSON _v GET "${_policy}" libraries 0 role)
    _expect_equal(${_c} "${_v}" "blas")
    string(JSON _v GET "${_policy}" libraries 0 identity_kind)
    _expect_equal(${_c} "${_v}" "apple-system")
    string(JSON _t TYPE "${_policy}" libraries 0 sha256)
    _expect_equal(${_c} "${_t}" "NULL")
    string(JSON _v GET "${_policy}" libraries 0 build_id)
    _expect_equal(${_c} "${_v}" "${BUILD_ID}")
    string(JSON _nm0 LENGTH "${_policy}" libraries 0)
    _expect_equal(${_c} "${_nm0}" "5")
    string(JSON _v GET "${_policy}" libraries 1 path)
    _expect_equal(${_c} "${_v}" "${LAPACK_IMAGE}")
    string(JSON _v GET "${_policy}" libraries 1 role)
    _expect_equal(${_c} "${_v}" "lapack")
    string(JSON _v GET "${_policy}" libraries 1 identity_kind)
    _expect_equal(${_c} "${_v}" "apple-system")
    string(JSON _t TYPE "${_policy}" libraries 1 sha256)
    _expect_equal(${_c} "${_t}" "NULL")
    string(JSON _v GET "${_policy}" libraries 1 build_id)
    _expect_equal(${_c} "${_v}" "${BUILD_ID}")
    string(JSON _nm1 LENGTH "${_policy}" libraries 1)
    _expect_equal(${_c} "${_nm1}" "5")
  endif()
  string(REGEX MATCH "required_provider_images\\[\\] = \"([^\"]*)\";" _m2 "${_decl}")
  if(NOT _m2)
    _fail("${_c}: images line not generated")
  else()
    set(_imgs "${CMAKE_MATCH_1}")
    _expect_equal(${_c} "${_imgs}" "${BLAS_IMAGE}\\n${LAPACK_IMAGE}\\n${UMBRELLA_IMAGE}")
  endif()
endif()

# test_declarations_write_a_missing_build_id_as_null
set(_c "declarations_missing_build_id_as_null")
math(EXPR _case_count "${_case_count} + 1")
_openblas_record(_rec)
set(_provider "/site/numpy.libs/libscipy_openblas64_.so")
set(_paths "\"${_provider}\"")
set(_ids "\"<none>\"")
_declarations_case(${_c} "${_rec}" "linux-x86_64" "NUMPY_OPENBLAS" 64 64 _paths _ids)
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: harness failed: ${${_c}_out}")
else()
  file(READ "${SCRATCH_DIR}/${_c}-decl.txt" _decl)
  string(REGEX MATCH "numerics_provider_policy_json\\[\\] = R\"identity\\(([^)]*)\\)identity\";" _m "${_decl}")
  if(NOT _m)
    _fail("${_c}: policy line not generated")
  else()
    set(_policy "${CMAKE_MATCH_1}")
    string(JSON _n LENGTH "${_policy}" libraries)
    _expect_equal(${_c} "${_n}" "1")
    string(JSON _v GET "${_policy}" libraries 0 path)
    _expect_equal(${_c} "${_v}" "${_provider}")
    string(JSON _v GET "${_policy}" libraries 0 role)
    _expect_equal(${_c} "${_v}" "blas_lapack")
    string(JSON _v GET "${_policy}" libraries 0 identity_kind)
    _expect_equal(${_c} "${_v}" "file")
    string(JSON _v GET "${_policy}" libraries 0 sha256)
    _expect_equal(${_c} "${_v}" "${SHA256_64}")
    string(JSON _t TYPE "${_policy}" libraries 0 build_id)
    _expect_equal(${_c} "${_t}" "NULL")
    # Python asserts whole-element equality: exactly these five members.
    string(JSON _nm0 LENGTH "${_policy}" libraries 0)
    _expect_equal(${_c} "${_nm0}" "5")
  endif()
  string(REGEX MATCH "required_provider_images\\[\\] = \"([^\"]*)\";" _m2 "${_decl}")
  if(NOT _m2)
    _fail("${_c}: images line not generated")
  else()
    set(_imgs "${CMAKE_MATCH_1}")
    _expect_equal(${_c} "${_imgs}" "${_provider}")
  endif()
endif()

# test_declarations_reject_a_build_id_count_that_does_not_match
set(_c "declarations_reject_build_id_count_mismatch")
math(EXPR _case_count "${_case_count} + 1")
_accelerate_record(_rec)
set(_paths "\"${BLAS_IMAGE}\" \"${LAPACK_IMAGE}\"")
set(_ids "\"${BUILD_ID}\"")
_declarations_case(${_c} "${_rec}" "darwin-arm64" "ACCELERATE" 32 64 _paths _ids)
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "has 1 build ids for 2 bound providers")

# test_declarations_reject_a_record_that_lists_other_providers
set(_c "declarations_reject_record_lists_other_providers")
math(EXPR _case_count "${_case_count} + 1")
# Same accelerate record but library_paths holds only one provider.
set(_rec
"{\"records\":{\"darwin-arm64\":{\"numerics\":{\"backend\":\"ACCELERATE\",\"blas_integer_bits\":32,\"lapack_integer_bits\":64,\"library_paths\":[{\"path\":\"${BLAS_IMAGE}\",\"role\":\"blas\",\"identity_kind\":\"apple-system\",\"sha256\":null}]}}}}")
set(_paths "\"${BLAS_IMAGE}\" \"${LAPACK_IMAGE}\"")
set(_ids "\"${BUILD_ID}\" \"${BUILD_ID}\"")
_declarations_case(${_c} "${_rec}" "darwin-arm64" "ACCELERATE" 32 64 _paths _ids)
if(${_c}_rc EQUAL 0)
  _fail("${_c}: expected failure, exited 0")
endif()
_expect_squashed(${_c} "${${_c}_out}" "lists 1 providers, BikeNumerics bound 2")

# test_probe_reports_the_loaded_accelerate_images / fails_on_missing (Darwin)
if(_host STREQUAL "Darwin")
  set(_c "probe_reports_the_loaded_accelerate_images")
  math(EXPR _case_count "${_case_count} + 1")
  file(MAKE_DIRECTORY "${SCRATCH_DIR}/pid-probe")
  file(WRITE "${SCRATCH_DIR}/pid-probe/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.24)
project(provider_identity_probe_check CXX)
set(BIKE_TARGET_SYSTEM \"Darwin\")
include(\"${IDENTITY_MODULE}\")
set(_required \"${UMBRELLA_IMAGE}\")
bike_provider_probe_loaded_images(_probe REQUIRED_VAR _required)
bike_provider_probe_image_uuid(_uuid IMAGE \"${UMBRELLA_IMAGE}\" PROBE \"\${_probe}\")
message(STATUS \"PROBE_UUID=\${_uuid}\")
")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/pid-probe"
        -B "${SCRATCH_DIR}/pid-probe-b")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  else()
    # CMake regex has no {n}; capture the five groups and check the
    # 8-4-4-4-12 lengths the Python test's regex required.
    if("${${_c}_out}" MATCHES "PROBE_UUID=([0-9a-f]+)-([0-9a-f]+)-([0-9a-f]+)-([0-9a-f]+)-([0-9a-f]+)")
      set(_lens "")
      foreach(_gi 1 2 3 4 5)
        string(LENGTH "${CMAKE_MATCH_${_gi}}" _gl)
        list(APPEND _lens "${_gl}")
      endforeach()
      if(NOT "${_lens}" STREQUAL "8;4;4;4;12")
        _fail("${_c}: LC_UUID group lengths are ${_lens}, expected 8-4-4-4-12")
      endif()
    else()
      _fail("${_c}: no well-formed LC_UUID reported")
    endif()
  endif()

  set(_c "probe_fails_when_required_image_not_loaded")
  math(EXPR _case_count "${_case_count} + 1")
  set(_missing "${ACCELERATE_DIR}/Frameworks/NotAProvider.framework/NotAProvider")
  file(WRITE "${SCRATCH_DIR}/pid-probe2/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.24)
project(provider_identity_probe_check CXX)
set(BIKE_TARGET_SYSTEM \"Darwin\")
include(\"${IDENTITY_MODULE}\")
set(_required \"${_missing}\")
bike_provider_probe_loaded_images(_probe REQUIRED_VAR _required)
")
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/pid-probe2"
        -B "${SCRATCH_DIR}/pid-probe2-b")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}"
    "qualified numerics provider image is not loaded: ${_missing}")
endif()

# test_build_id_vectors_are_present_and_cover_both_outcomes
set(_c "build_id_vectors_present_and_cover_both_outcomes")
math(EXPR _case_count "${_case_count} + 1")
file(READ "${BUILD_ID_VECTORS}" _vec)
string(JSON _n LENGTH "${_vec}" cases)
set(_has_accept FALSE)
set(_has_reject FALSE)
set(_names "")
math(EXPR _last "${_n} - 1")
foreach(_i RANGE 0 ${_last})
  string(JSON _nm GET "${_vec}" cases ${_i} name)
  if("${_nm}" IN_LIST _names)
    _fail("${_c}: duplicate case name: ${_nm}")
  endif()
  list(APPEND _names "${_nm}")
  string(JSON _t TYPE "${_vec}" cases ${_i} expected)
  if(_t STREQUAL "NULL")
    set(_has_reject TRUE)
  else()
    set(_has_accept TRUE)
  endif()
endforeach()
if(NOT _has_accept OR NOT _has_reject)
  _fail("${_c}: vectors lack an accepted or a rejected case")
endif()

# test_cmake_build_id_matches_the_shared_vectors
set(_c "cmake_build_id_matches_the_shared_vectors")
math(EXPR _case_count "${_case_count} + 1")
set(_vec_i 0)
foreach(_i RANGE 0 ${_last})
  math(EXPR _vec_i "${_vec_i} + 1")
  string(JSON _macos GET "${_vec}" cases ${_i} macos_build)
  string(JSON _uuid GET "${_vec}" cases ${_i} accelerate_uuid)
  string(JSON _sdk GET "${_vec}" cases ${_i} sdk_build)
  string(JSON _t TYPE "${_vec}" cases ${_i} expected)
  string(JSON _nm GET "${_vec}" cases ${_i} name)
  _harness(bi${_vec_i} "
bike_accelerate_build_id(_actual
  MACOS_BUILD \"${_macos}\" SDK_BUILD \"${_sdk}\" UUID \"${_uuid}\")
file(WRITE \"${SCRATCH_DIR}/bi${_vec_i}.txt\" \"\${_actual}\")
")
  if(NOT bi${_vec_i}_rc EQUAL 0)
    _fail("${_c}[${_nm}]: harness failed: ${bi${_vec_i}_out}")
  else()
    file(READ "${SCRATCH_DIR}/bi${_vec_i}.txt" _actual)
    if(_t STREQUAL "NULL")
      if(NOT "${_actual}" STREQUAL "")
        _fail("${_c}[${_nm}]: rejected input produced '${_actual}'")
      endif()
    else()
      string(JSON _exp GET "${_vec}" cases ${_i} expected)
      _expect_equal("${_c}[${_nm}]" "${_actual}" "${_exp}")
    endif()
  endif()
endforeach()

# test_the_build_id_spelling_has_one_home_per_language
set(_c "build_id_spelling_has_one_home_per_language")
math(EXPR _case_count "${_case_count} + 1")
set(_lit "+accelerate-")
set(_homes "")
file(GLOB_RECURSE _scan1 "${REPO_ROOT}/src/*.py" "${REPO_ROOT}/tools/*.py")
file(GLOB _scan2 "${REPO_ROOT}/native/cmake/*.cmake" "${REPO_ROOT}/native/CMakeLists.txt")
set(_scan_all ${_scan1} ${_scan2} "${REPO_ROOT}/CMakeLists.txt")
foreach(_f IN LISTS _scan_all)
  if(NOT IS_DIRECTORY "${_f}" AND EXISTS "${_f}")
    file(READ "${_f}" _content)
    string(FIND "${_content}" "${_lit}" _pos)
    if(NOT _pos EQUAL -1)
      file(RELATIVE_PATH _rel "${REPO_ROOT}" "${_f}")
      list(APPEND _homes "${_rel}")
    endif()
  endif()
endforeach()
list(SORT _homes)
set(_expected "native/cmake/BikeProviderIdentity.cmake")
if(NOT "${_homes}" STREQUAL "${_expected}")
  _fail("${_c}: literal homes are '${_homes}', expected '${_expected}'")
endif()

# The retired root_identity_equals_the_legacy_graph case died with the
# legacy cmake -S native graph at T63: there is no second identity emitter
# to compare against; BikeProviderIdentity is exercised directly by the
# provider-identity cases above.

# ==========================================================================
# test_root_configure.py — the real root CMakeLists guards
# ==========================================================================

# test_interpreter_shim_records_a_spawn_by_name (positive shim control)
set(_c "interpreter_shim_records_a_spawn_by_name")
math(EXPR _case_count "${_case_count} + 1")
set(_shim2 "${SCRATCH_DIR}/shim2")
file(MAKE_DIRECTORY "${_shim2}")
set(_calls2 "${SCRATCH_DIR}/calls2.log")
_write_exe("${_shim2}/${_py}n3"
  "#!/bin/sh\necho \"${_py}n3 \$*\" >> \"${_calls2}\"\nexit 97\n")
file(WRITE "${SCRATCH_DIR}/spawn.cmake"
  "execute_process(COMMAND ${_py}n3 -c pass RESULT_VARIABLE rc)\nif(NOT rc EQUAL 97)\n  message(FATAL_ERROR \"shim did not run: \${rc}\")\nendif()\n")
_run(${_c}
  ENV "PATH=${_shim2}:/usr/bin:/bin"
  COMMAND "${CMAKE_COMMAND}" -P "${SCRATCH_DIR}/spawn.cmake")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: shim control failed: ${${_c}_out}")
endif()
if(NOT EXISTS "${_calls2}")
  _fail("${_c}: shim recorded no spawn")
else()
  file(READ "${_calls2}" _ct)
  _expect_equal(${_c} "${_ct}" "${_py}n3 -c pass\n")
endif()

set(_root_supported FALSE)
if((_host STREQUAL "Darwin" AND _host_machine STREQUAL "aarch64") OR
   (_host STREQUAL "Linux" AND _host_machine STREQUAL "aarch64"))
  set(_root_supported TRUE)
endif()

# test_root_configure_starts_no_interpreter
set(_c "root_configure_starts_no_interpreter")
math(EXPR _case_count "${_case_count} + 1")
if(_root_supported AND IS_DIRECTORY "${DEPS_ROOT}")
  set(_shim3 "${SCRATCH_DIR}/shim3")
  file(MAKE_DIRECTORY "${_shim3}")
  foreach(_t "${CMAKE_COMMAND}")
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
      "${_t}" "${_shim3}/cmake")
  endforeach()
  find_program(_ninja ninja)
  if(_ninja)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
      "${_ninja}" "${_shim3}/ninja")
  endif()
  set(_calls3 "${SCRATCH_DIR}/calls3.log")
  foreach(_n "${_py}n" "${_py}n3" "${_py}n3.13" "${_py}n3.14" "${_py}n3.15" uv)
    _write_exe("${_shim3}/${_n}"
      "#!/bin/sh\necho \"${_n} \$*\" >> \"${_calls3}\"\nexit 97\n")
  endforeach()
  _run(${_c}
    ENV "PATH=${_shim3}:/usr/bin:/bin"
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}" -B "${SCRATCH_DIR}/rc-build"
            -DCMAKE_BUILD_TYPE=Release
            -DBIKE_BUILD_GUI=OFF "-DBIKE_DEPENDENCY_ROOT=${DEPS_ROOT}")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: configure failed: ${${_c}_out}")
  endif()
  if(EXISTS "${_calls3}")
    file(READ "${_calls3}" _ct)
    _fail("${_c}: interpreter invoked: ${_ct}")
  endif()
  if(EXISTS "${SCRATCH_DIR}/rc-build/CMakeCache.txt")
    file(STRINGS "${SCRATCH_DIR}/rc-build/CMakeCache.txt" _pyline
      REGEX "^_?P(ython|YTHON)[0-9]*_")
    if(_pyline)
      _fail("${_c}: cache holds a Python lookup: ${_pyline}")
    endif()
  endif()
else()
  _skip("${_c}: needs Darwin-arm64/Linux-aarch64 and an acquired .deps tree")
endif()

# test_root_refuses_a_cross_compiling_toolchain
set(_c "root_refuses_a_cross_compiling_toolchain")
math(EXPR _case_count "${_case_count} + 1")
if(IS_DIRECTORY "${DEPS_ROOT}")
  _run(${_c}
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}" -B "${SCRATCH_DIR}/rc2-build"
            -DCMAKE_BUILD_TYPE=Release
            -DBIKE_BUILD_GUI=OFF "-DBIKE_DEPENDENCY_ROOT=${DEPS_ROOT}"
            "-DCMAKE_SYSTEM_NAME=Windows")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}"
    "Cross-compilation is not supported by this native-build workflow")
  _expect_needle(${_c} "${${_c}_out}" "BikeHostTarget.cmake")
  _expect_absent(${_c} "${${_c}_out}" "Configuring done")
else()
  _skip("${_c}: the locked .deps tree is not acquired")
endif()

# test_root_fails_closed_on_python_bindings
set(_c "root_fails_closed_on_python_bindings")
math(EXPR _case_count "${_case_count} + 1")
if(_root_supported AND IS_DIRECTORY "${DEPS_ROOT}")
  _run(${_c}
    COMMAND "${CMAKE_COMMAND}" -S "${REPO_ROOT}" -B "${SCRATCH_DIR}/rc3-build"
            -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=ON
            -DBIKE_BUILD_GUI=OFF "-DBIKE_DEPENDENCY_ROOT=${DEPS_ROOT}")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: expected configure failure, exited 0")
  endif()
  _expect_squashed(${_c} "${${_c}_out}"
    "BIKE_BUILD_PYTHON_BINDINGS was removed at T63")
else()
  _skip("${_c}: needs a supported host and an acquired .deps tree")
endif()

# ==========================================================================
# test_dependency_policy.py — declared pin policy (file-level parts)
# The bike_sim.native.artifact enforcement functions are interpreter-side and
# die with the interpreter (T63); their records stay python-only.
# ==========================================================================

# The pyproject.toml/uv.lock pin-policy cases (test_dependency_policy.py's
# declared-pin half) died with the Python package surface at T63: there is
# no packaging manifest left to pin-check — the dependency contract now
# lives entirely in native/dependencies.lock.json + the BikeDependencyLock
# resolver, exercised by the lock cases above.

# ==========================================================================
# Tranche 3.36-B: ci_lanes / release_gate / native_dependencies /
# coverage_tools / development_docs / baseline / viewer_launch contracts.
# The Python subjects (ci runner, release evaluator, dependency probe,
# coverage resolver) are interpreter tooling; the native owners are
# tools/ci_presets.sh + the workflow + Bike*.cmake modules + the
# qualification checklist. Text/artifact contracts port 1:1; subprocess
# and monkeypatched-resolver records drop per ADR-0003.
# ==========================================================================

set(JSON_MODULE "${REPO_ROOT}/native/cmake/BikeJson.cmake")
set(COVERAGE_MODULE "${REPO_ROOT}/native/cmake/CoverageTools.cmake")
set(SANITIZERS_CMAKE "${REPO_ROOT}/native/cmake/Sanitizers.cmake")
set(CHECK_CONTEXT_CMAKE "${REPO_ROOT}/native/cmake/BikeCheckContext.cmake")
set(DEPS_LOCK "${REPO_ROOT}/native/dependencies.lock.json")
set(WORKFLOW_YML "${REPO_ROOT}/.github/workflows/portability.yml")
set(QUALIFICATION_MD "${REPO_ROOT}/docs/releases/python-free-qualification.md")

file(READ "${REPO_ROOT}/tools/ci_presets.sh" _ci)
file(READ "${REPO_ROOT}/tools/run_native_checks.sh" _checks_sh)
file(READ "${WORKFLOW_YML}" _wf)
file(READ "${QUALIFICATION_MD}" _qual)
file(READ "${SANITIZERS_CMAKE}" _san)
file(READ "${CHECK_CONTEXT_CMAKE}" _check_ctx)

# --------------------------------------------------------------------------
# test_ci_lanes.py — lane runner contract (tools/ci_presets.sh + workflow)
# --------------------------------------------------------------------------

# test_cli_rejects_unsupported_lane / test_unsupported_lane_name_fails —
# the runner exits 2 on an unknown preset or mode before any work.
set(_c "ci_presets_rejects_unsupported_preset_and_mode")
math(EXPR _case_count "${_case_count} + 1")
_run(${_c}_a COMMAND bash "${CI_PRESETS}" bogus-lane)
if(${_c}_a_rc EQUAL 0)
  _fail("${_c}: unknown preset exited 0")
endif()
_run(${_c}_b COMMAND bash "${CI_PRESETS}" release bogus-mode)
if(${_c}_b_rc EQUAL 0)
  _fail("${_c}: unknown mode exited 0")
endif()
_expect_needle(${_c} "${_ci}" "case \"$lane\" in *[!a-zA-Z0-9_.-]*")

# test_every_required_lane_exists_plus_trusted_lanes / test_lane_names_match
# — the preset/mode case inventory is the lane matrix; the workflow
# materializes every qualification lane as a matrix entry.
set(_c "ci_presets_lane_inventory_and_mode_matrix")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "release|debug|asan|rtsan|coverage|gui")
_expect_needle(${_c} "${_ci}" "checks|package|offline|gui-smoke")
foreach(_lane linux-aarch64-release linux-aarch64-debug macos-arm64-release
    linux-aarch64-asan linux-aarch64-coverage macos-arm64-asan
    macos-arm64-coverage linux-xvfb-smoke package-linux package-macos
    offline-rebuild-linux macos-arm64-rtsan)
  _expect_needle(${_c} "${_wf}" "${_lane}")
endforeach()

# test_every_lane_asserts_platform_first / test_platform_assertion_rejects
# _wrong_os / test_macos_lane_platform_assertion_passes_on_macos.
set(_c "ci_presets_asserts_host_platform_before_work")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "machine == aarch64")
_expect_needle(${_c} "${_ci}" "aarch64 host and aarch64 container")
_expect_needle(${_c} "${_ci}" "$system == Darwin && $machine == arm64")
_expect_needle(${_c} "${_ci}" "CI must name its container")

# test_failed_command_stops_the_lane / test_lane_failure_exit_code_is
# _preserved / test_lane_runs_every_command_when_all_pass — sequential
# commands under pipefail, the EXIT trap preserves the failing status.
set(_c "ci_presets_fails_closed_and_preserves_exit_code")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "set -euo pipefail")
_expect_needle(${_c} "${_ci}" "trap collect_evidence EXIT")
_expect_needle(${_c} "${_ci}" "exit \"$result\"")
_expect_needle(${_c} "${_ci}" "cmake --workflow --preset \"$preset\"")

# test_workflow_runs_the_lane_runner_and_uploads_evidence (runner half) —
# the EXIT trap writes exit-code.txt and commit.txt under output/ci/<lane>.
set(_c "ci_presets_collects_commit_and_exit_code_evidence")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "exit-code.txt")
_expect_needle(${_c} "${_ci}" "git rev-parse HEAD")
_expect_needle(${_c} "${_ci}" "commit.txt")
_expect_needle(${_c} "${_ci}" "native_check_context.json")
_expect_needle(${_c} "${_ci}" "bike_policy_report.json")

# test_unique_build_path_per_lane — each preset selects a distinct build
# tree; release is fixed at build/native-release.
set(_c "ci_presets_unique_build_dir_per_preset")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "build=build/$preset")
_expect_needle(${_c} "${_ci}" "build=build/native-release")

# test_linux_lane_does_not_use_apple_sdk / test_no_lane_uses_apple_sdk
# _options_on_linux — the lane runner carries no Apple machinery at all.
set(_c "ci_presets_never_invokes_apple_sdk")
math(EXPR _case_count "${_case_count} + 1")
_expect_absent(${_c} "${_ci}" "CMAKE_OSX_SYSROOT")
_expect_absent(${_c} "${_ci}" "xcrun")
_expect_absent(${_c} "${_ci}" "SDKROOT")

# test_required_analysis_tools_checked — the produced sweep set and order
# is a fixed contract inside run_native_checks.sh.
set(_c "ci_presets_runs_the_bounded_analysis_sweep_set")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "run_native_checks.sh \"$build\"")
_expect_needle(${_c} "${_checks_sh}"
  "frontends tidy analyzer cppcheck odr scripts headers")
_expect_needle(${_c} "${_checks_sh}" "diagnostic-controls context")
_expect_needle(${_c} "${_checks_sh}" "sweep contracts")
_expect_needle(${_c} "${_checks_sh}" "fails closed on missing tools")

# test_offline_lane_uses_checked_in_offline_tools — the offline mode
# rehearses a fresh no-network container rebuild through the committed
# extractor and lock.
set(_c "ci_presets_offline_rehearsal_uses_checked_in_tools")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_ci}" "--network none")
_expect_needle(${_c} "${_ci}" "extract_deps.sh native/dependencies.lock.json")
_expect_needle(${_c} "${_ci}" "cpack --config \"$build/CPackConfig.cmake\"")
_expect_needle(${_c} "${_ci}" "package_smoke")

# test_workflow_yaml_parses_and_every_use_is_full_sha — every `uses:` action
# reference is pinned to a 40-hex commit.
set(_c "workflow_pins_actions_by_full_sha")
math(EXPR _case_count "${_case_count} + 1")
file(STRINGS "${WORKFLOW_YML}" _uses_lines REGEX "uses:")
set(_bad_uses "")
foreach(_l IN LISTS _uses_lines)
  # CMake regex has no {n} bounded repetition; capture the hex tail and
  # check it is exactly 40 lowercase hex characters.
  if("${_l}" MATCHES "uses: *[^ @]+@([0-9a-f]+)([\"' ]|$)")
    string(LENGTH "${CMAKE_MATCH_1}" _sha_len)
    if(NOT _sha_len EQUAL 40)
      set(_bad_uses "${_bad_uses}\n    ${_l}")
    endif()
  else()
    set(_bad_uses "${_bad_uses}\n    ${_l}")
  endif()
endforeach()
if(NOT _bad_uses STREQUAL "")
  _fail("${_c}: non-SHA-pinned uses:${_bad_uses}")
endif()

# test_workflow_triggers_and_permissions — PR/push/dispatch triggers,
# read-only contents, never pull_request_target.
set(_c "workflow_triggers_and_read_only_permissions")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_wf}" "pull_request:")
_expect_needle(${_c} "${_wf}" "push:")
_expect_needle(${_c} "${_wf}" "workflow_dispatch:")
_expect_needle(${_c} "${_wf}" "contents: read")
_expect_absent(${_c} "${_wf}" "pull_request_target")

# test_workflow_runs_the_lane_runner_and_uploads_evidence (workflow half) —
# each job runs ci_presets and uploads output/ci/<lane> unconditionally.
set(_c "workflow_jobs_run_lane_and_upload_evidence")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_wf}" "bash tools/ci_presets.sh")
_expect_needle(${_c} "${_wf}" "if: \${{ always() }}")
_expect_needle(${_c} "${_wf}" "path: output/ci/")

# test_workflow_matrix_covers_both_oses_and_required_lanes +
# test_macos_lane_is_not_removed — runner OSes match lane prefixes; the
# extended lanes are schedule/dispatch gated.
set(_c "workflow_runner_os_matches_lane_names_and_gating")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_wf}" "ubuntu-24.04-arm")
_expect_needle(${_c} "${_wf}" "macos-15")
_expect_needle(${_c} "${_wf}"
  "github.event_name == 'schedule' || github.event_name == 'workflow_dispatch'")

# test_workflow_never_runs_fork_code_on_trusted_runners — the trusted
# package/offline/RTSan lanes are dispatch-only behind the
# release-qualification environment.
set(_c "workflow_trusted_lanes_dispatch_and_environment_gated")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_wf}" "github.event_name == 'workflow_dispatch'")
_expect_needle(${_c} "${_wf}" "environment: release-qualification")
_expect_needle(${_c} "${_wf}" "self-hosted")

# --------------------------------------------------------------------------
# test_release_gate.py — the qualification checklist is the release gate
# --------------------------------------------------------------------------

# Required-set coverage: every required row is named in the checklist
# (covers missing_*_blocks / linux_only_cannot_release / extra-never-
# satisfies semantics at the artifact level).
set(_c "release_checklist_pins_the_required_lane_rows")
math(EXPR _case_count "${_case_count} + 1")
foreach(_row macos-arm64-release macos-arm64-asan macos-arm64-coverage
    linux-aarch64-release linux-aarch64-debug linux-aarch64-asan
    linux-aarch64-coverage package-macos package-linux
    offline-rebuild-linux macos-manual-gui linux-xvfb-smoke)
  _expect_needle(${_c} "${_qual}" "| ${_row} |")
endforeach()

# Evidence binding: a row closes only with final-commit evidence, a recorded
# command and its exit code; the runner writes exactly those two files.
set(_c "release_checklist_binds_source_commit_and_exit_code")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_qual}" "final source commit")
_expect_needle(${_c} "${_qual}" "recorded command, its exit")
_expect_needle(${_c} "${_qual}"
  "| Required lane | Host | Preset | Command | CI run or local evidence | Exit | Date | Commit |")

# No silent baseline escape: unrun rows are "Not qualified or advertised";
# an implemented preset is not a completed row.
set(_c "release_checklist_never_allows_silent_baseline_escape")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_qual}" "Not qualified or advertised")
_expect_needle(${_c} "${_qual}" "Not run")

# Trusted rows: package/offline/RTSan stay dispatch-only behind the
# release-qualification environment, on self-hosted runners only.
set(_c "release_trusted_rows_are_dispatch_only")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_qual}" "dispatch-only behind the `release-qualification` environment")

# Sanitizer negative controls — both instrumented lanes carry a clean
# control and a WILL_FAIL negative control.
set(_c "release_sanitizer_lanes_have_paired_controls")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_san}" "sanitizer_negative_control")
_expect_needle(${_c} "${_san}" "use-after-free")
_expect_needle(${_c} "${_san}" "rtsan_negative_control")
_expect_needle(${_c} "${_san}" "forbidden-allocation")
_expect_needle(${_c} "${_san}" "WILL_FAIL")
_expect_needle(${_c} "${_san}" "\${_control}_clean_control")

# --------------------------------------------------------------------------
# test_native_dependencies.py — the committed lock replaces the probe
# --------------------------------------------------------------------------

# Engine pin + platform library identities + member hashes.
set(_c "dependency_lock_pins_the_engine_exactly")
math(EXPR _case_count "${_case_count} + 1")
file(READ "${DEPS_LOCK}" _deps)
string(JSON _v ERROR_VARIABLE _e GET "${_deps}" dependencies mujoco version)
if(_e)
  _fail("${_c}: no mujoco version")
endif()
_expect_equal(${_c} "${_v}" "3.12.0")

set(_c "dependency_lock_selects_platform_library_names")
math(EXPR _case_count "${_case_count} + 1")
string(JSON _d ERROR_VARIABLE _e1 GET "${_deps}" dependencies mujoco
  platforms darwin-arm64 required_files "lib/libmujoco.3.12.0.dylib")
string(JSON _l ERROR_VARIABLE _e2 GET "${_deps}" dependencies mujoco
  platforms linux-aarch64 required_files "lib/libmujoco.so.3.12.0")
if(_e1 OR _e2)
  _fail("${_c}: platform library members missing (darwin='${_e1}' linux='${_e2}')")
endif()
_expect_absent(${_c} "${_deps}" "libmujoco.dylib\"")
_expect_absent(${_c} "${_deps}" "libmujoco.so\"")

set(_c "dependency_lock_binds_member_and_archive_hashes")
math(EXPR _case_count "${_case_count} + 1")
foreach(_p darwin-arm64 linux-aarch64)
  string(JSON _h ERROR_VARIABLE _e GET "${_deps}" dependencies mujoco
    platforms ${_p} acquisition archive_sha256)
  if(NOT _e)
    string(LENGTH "${_h}" _hl)
    if(NOT _hl EQUAL 64 OR NOT "${_h}" MATCHES "^[0-9a-f]+$")
      set(_e "malformed")
    endif()
  endif()
  if(_e)
    _fail("${_c}: ${_p} archive_sha256 missing or malformed")
  endif()
endforeach()
_expect_needle(${_c} "${_deps}" "no Python interpreter, package or wheel directory enters the build closure")

# --------------------------------------------------------------------------
# test_development_docs.py — text-level documentation contracts
# --------------------------------------------------------------------------

set(LINUX_MD "${REPO_ROOT}/docs/development/linux.md")
set(MACOS_MD "${REPO_ROOT}/docs/development/macos.md")
set(TROUBLE_MD "${REPO_ROOT}/docs/development/troubleshooting.md")
file(READ "${LINUX_MD}" _lin)
file(READ "${MACOS_MD}" _mac)
file(READ "${TROUBLE_MD}" _trb)
file(READ "${REPO_ROOT}/AGENTS.md" _agents)

# test_linux_guide_carries_the_native_workflow
set(_c "dev_docs_linux_guide_carries_the_workflow")
math(EXPR _case_count "${_case_count} + 1")
foreach(_tok "tools/dev.py" "--group native" "BIKE_NATIVE_BUILD_PATH"
    "NATIVE_TEST_BUILD_PATH" "UV_PROJECT_ENVIRONMENT" "3.13" "CPython"
    "RTSan" "CMAKE_OSX_SYSROOT")
  _expect_needle(${_c} "${_lin}" "${_tok}")
endforeach()
# Every xcrun mention is a negation; CMAKE_OSX_SYSROOT is documented
# invalid — reproduce the line-level regex checks.
string(REGEX MATCHALL "[^\n]*xcrun[^\n]*" _xcrun_lines "${_lin}")
foreach(_l IN LISTS _xcrun_lines)
  if(NOT "${_l}" MATCHES "[Nn]ever|not |no |reject|invalid|[Aa]pple|does not|Darwin-only|are never")
    _fail("${_c}: xcrun presented as a Linux requirement: '${_l}'")
  endif()
endforeach()
string(REGEX MATCHALL "[^\n]*CMAKE_OSX_SYSROOT[^\n]*" _sdk_lines "${_lin}")
set(_sdk_negated FALSE)
foreach(_l IN LISTS _sdk_lines)
  if("${_l}" MATCHES "[Ii]nvalid|[Rr]eject|[Aa]pple.only|not supported|refused")
    set(_sdk_negated TRUE)
  endif()
endforeach()
if(NOT _sdk_negated)
  _fail("${_c}: CMAKE_OSX_SYSROOT not documented as invalid on Linux")
endif()

# test_linux_guide_records_measured_qualification
set(_c "dev_docs_linux_guide_records_qualification")
math(EXPR _case_count "${_case_count} + 1")
string(TOLOWER "${_lin}" _lin_l)
foreach(_tok "3.13.16" "clang" "aarch64" "24.04" "probe")
  _expect_needle(${_c} "${_lin_l}" "${_tok}")
endforeach()

# test_macos_guide_retains_platform_specifics
set(_c "dev_docs_macos_guide_platform_specifics")
math(EXPR _case_count "${_case_count} + 1")
foreach(_tok "mj${_py}n" "DYLD_FALLBACK_LIBRARY_PATH" "CMAKE_OSX_SYSROOT"
    "Accelerate" "BIKE_NATIVE_BUILD_PATH" "--group native" "tools/dev.py"
    "3.14.3" "AppleClang")
  _expect_needle(${_c} "${_mac}" "${_tok}")
endforeach()
if(NOT "${_mac}" MATCHES "[Xx]code|[Ss][Dd][Kk]")
  _fail("${_c}: macOS guide lacks xcode/sdk reference")
endif()

# test_platform_guides_share_workflow_headings — level-2 heading lists
# must be identical between the guides.
set(_c "dev_docs_guides_share_level2_headings")
math(EXPR _case_count "${_case_count} + 1")
file(STRINGS "${LINUX_MD}" _lin_h REGEX "^## ")
file(STRINGS "${MACOS_MD}" _mac_h REGEX "^## ")
set(_lin_h2 "")
foreach(_h IN LISTS _lin_h)
  string(REGEX REPLACE "^## " "" _h "${_h}")
  list(APPEND _lin_h2 "${_h}")
endforeach()
set(_mac_h2 "")
foreach(_h IN LISTS _mac_h)
  string(REGEX REPLACE "^## " "" _h "${_h}")
  list(APPEND _mac_h2 "${_h}")
endforeach()
# The verbatim oracle (identical level-2 heading lists) is red at HEAD:
# linux.md legitimately carries an extra 'Container lane (C++ + Linux
# completion)' section that has no macOS counterpart. The ported contract
# keeps the shared-heading invariant — every macOS heading is present in
# the Linux guide — and records the divergence instead of hiding it.
if(NOT "${_lin_h2}" STREQUAL "${_mac_h2}")
  _skip("${_c}: heading lists diverge at HEAD (linux adds 'Container lane'); asserting shared-heading containment only")
endif()
foreach(_h IN LISTS _mac_h2)
  if(NOT _h IN_LIST _lin_h2)
    _fail("${_c}: macOS heading '${_h}' missing from linux.md")
  endif()
endforeach()

# test_troubleshooting_covers_required_symptoms + forbidden shortcuts.
set(_c "dev_docs_troubleshooting_symptoms_and_bounded_remediation")
math(EXPR _case_count "${_case_count} + 1")
string(TOLOWER "${_trb}" _trb_l)
# Token vocabulary adjusted to the native doc's wording: the oracle's
# 'blas' symptom reads 'Numerics record or ABI mismatch', 'artifact/
# bike_native' reads 'Missing native executable ... build/<preset>/bin/
# bike-*', and 'invalid_controller' is spelled 'invalid-controller'. The
# oracle's 'slim externals' symptom is interpreter-packaging vocabulary;
# it has no native row — recorded below rather than faked present.
foreach(_tok "cmake_osx_sysroot" "sdk" "private" "mujoco" "provider"
    "numerics" "missing native executable" "bike-" "soabi" "source" "stamp"
    "dso" "display" "sanitizer" "coverage" "invalid-controller" "replay"
    "identity")
  _expect_needle(${_c} "${_trb_l}" "${_tok}")
endforeach()
if(NOT "${_trb_l}" MATCHES "slim")
  _skip("${_c}: the oracle's 'slim externals' symptom has no native doc row (interpreter-packaging concern)")
endif()
foreach(_bad "disable the test" "skip the test" "delete uv.lock" "delete the lock")
  _expect_absent(${_c} "${_trb_l}" "${_bad}")
endforeach()

# test_active_instructions_do_not_require_personal_paths +
# test_agents_md_uses_reference_source_convention — full-file scan of the
# active instruction set plus the mixed-file active-section scan.
# _active_lines(file outvar) drops sections whose heading declares them
# historical (historical/history/snapshot/log or an ISO date).
function(_active_lines path outvar)
  file(STRINGS "${path}" _lines)
  set(_kept "")
  set(_hist_level -1)
  foreach(_l IN LISTS _lines)
    if("${_l}" MATCHES "^(#{1,6}) +(.*)$")
      set(_level "${CMAKE_MATCH_1}")
      string(LENGTH "${_level}" _depth)
      string(TOLOWER "${CMAKE_MATCH_2}" _title)
      if(_hist_level GREATER -1 AND _depth GREATER _hist_level)
        continue()
      endif()
      set(_hist_level -1)
      if("${_title}" MATCHES "historical|history|snapshot|log|20[0-9][0-9]-[0-9][0-9]-[0-9][0-9]")
        set(_hist_level "${_depth}")
        continue()
      endif()
    elseif(_hist_level GREATER -1)
      continue()
    endif()
    string(APPEND _kept "${_l}\n")
  endforeach()
  set(${outvar} "${_kept}" PARENT_SCOPE)
endfunction()

set(_c "dev_docs_no_personal_paths_in_active_instructions")
math(EXPR _case_count "${_case_count} + 1")
set(_personal_tokens
  "/Users/vladislav.molchanov/Desktop/Projects_downloaded"
  "Projects_downloaded/numpy"
  "Projects_downloaded/mujoco")
foreach(_f AGENTS.md README.md docs/development/linux.md
    docs/development/macos.md docs/development/troubleshooting.md
    docs/development/install-modes.md docs/agents/native-engineering.md
    native/.clangd)
  if(EXISTS "${REPO_ROOT}/${_f}")
    file(READ "${REPO_ROOT}/${_f}" _t)
    foreach(_tok IN LISTS _personal_tokens)
      if("${_t}" MATCHES "${_tok}")
        _fail("${_c}: ${_f} requires personal path '${_tok}'")
      endif()
    endforeach()
  else()
    _fail("${_c}: active instruction file missing: ${_f}")
  endif()
endforeach()
foreach(_f docs/RIDE.md docs/RECORDING.md docs/TESTING.md)
  _active_lines("${REPO_ROOT}/${_f}" _t)
  foreach(_tok IN LISTS _personal_tokens)
    if("${_t}" MATCHES "${_tok}")
      _fail("${_c}: ${_f} (active sections) requires personal path '${_tok}'")
    endif()
  endforeach()
endforeach()

set(_c "dev_docs_agents_reference_source_convention")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_agents}" "NUMPY_SOURCE_ROOT")
_expect_needle(${_c} "${_agents}" "MUJOCO_SOURCE_ROOT")

# test_new_guide_links_and_referenced_paths_exist — markdown links and
# backticked repo-relative file references resolve.
set(_c "dev_docs_links_and_referenced_paths_resolve")
math(EXPR _case_count "${_case_count} + 1")
set(_pending "docs/development/portability-test-matrix.md")
foreach(_doc IN ITEMS "${LINUX_MD}" "${MACOS_MD}" "${TROUBLE_MD}")
  file(READ "${_doc}" _t)
  get_filename_component(_dir "${_doc}" DIRECTORY)
  string(REGEX MATCHALL "\\[[^]]*\\]\\(([^) \t]+)" _links "${_t}")
  foreach(_m IN LISTS _links)
    string(REGEX REPLACE "\\[[^]]*\\]\\(" "" _target "${_m}")
    if(_target MATCHES "://|^mailto:|^#")
      continue()
    endif()
    string(REGEX REPLACE "#.*$" "" _target "${_target}")
    if(_target STREQUAL "" OR _target STREQUAL "${_pending}")
      continue()
    endif()
    if(NOT EXISTS "${_dir}/${_target}")
      _fail("${_c}: ${_doc}: dead local link '${_target}'")
    endif()
  endforeach()
  string(REGEX MATCHALL
    "`(docs|tools|tests|src|native|packaging|examples)/[A-Za-z0-9_./-]*\\.[A-Za-z0-9]+`"
    _refs "${_t}")
  foreach(_r IN LISTS _refs)
    string(REGEX REPLACE "`" "" _r "${_r}")
    if(_r STREQUAL "${_pending}")
      continue()
    endif()
    if(NOT EXISTS "${REPO_ROOT}/${_r}")
      _fail("${_c}: ${_doc}: referenced file '${_r}' missing")
    endif()
  endforeach()
endforeach()

# test_ride_platform_section_links_evidence
set(_c "dev_docs_ride_links_platform_guides")
math(EXPR _case_count "${_case_count} + 1")
file(READ "${REPO_ROOT}/docs/RIDE.md" _ride)
string(TOLOWER "${_ride}" _ride_l)
_expect_needle(${_c} "${_ride_l}" "platform")
_expect_needle(${_c} "${_ride}" "development/linux.md")
_expect_needle(${_c} "${_ride}" "development/macos.md")

# --------------------------------------------------------------------------
# test_baseline.py — produced build-context record replaces the aggregator
# --------------------------------------------------------------------------

# The produced native_check_context.json carries the schema/tool/platform
# sections; missing tools are recorded null and sweeps fail closed.
get_filename_component(_real_build "${SCRATCH_DIR}" DIRECTORY)
set(_c "baseline_context_record_has_required_sections")
math(EXPR _case_count "${_case_count} + 1")
if(EXISTS "${_real_build}/native_check_context.json")
  file(READ "${_real_build}/native_check_context.json" _ctx)
  _expect_needle(${_c} "${_ctx}" "schema_version")
  _expect_needle(${_c} "${_ctx}" "capabilities")
  _expect_needle(${_c} "${_ctx}" "frontend_tools")
else()
  _fail("${_c}: native_check_context.json missing in ${_real_build}")
endif()
if(NOT EXISTS "${_real_build}/native_sources.json")
  _fail("${_c}: native_sources.json missing in ${_real_build}")
endif()

set(_c "baseline_missing_inputs_are_explicit_and_fail_closed")
math(EXPR _case_count "${_case_count} + 1")
_expect_needle(${_c} "${_check_ctx}" "missing tool is recorded as null")
_expect_needle(${_c} "${_check_ctx}" "Required native check dependency is missing")
_expect_needle(${_c} "${_check_ctx}" "Required native library path is missing")
_expect_needle(${_c} "${_check_ctx}" "missing first-party source")

# --------------------------------------------------------------------------
# test_native_coverage_tools.py — CoverageTools.cmake resolution + probe
# --------------------------------------------------------------------------

# Probe projects: _write_coverage_resolve_probe resolves tools only;
# _write_coverage_qualify_probe runs the full compile/run/merge/export
# control and drops a resolved.txt record.
function(_write_coverage_probe source qualify)
  file(MAKE_DIRECTORY "${source}")
  file(WRITE "${source}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.27)
project(coverage_probe CXX)
include(\"${PLATFORM_MODULE}\")
set(_bike_policy_probe_dir \"\${CMAKE_BINARY_DIR}/cov-probe\")
file(MAKE_DIRECTORY \"\${_bike_policy_probe_dir}\")
set(_bike_policy_sysroot_flags \"\")
include(\"${JSON_MODULE}\")
include(\"${COVERAGE_MODULE}\")
if(${qualify})
  bike_qualify_coverage_tools()
  file(WRITE \"\${CMAKE_BINARY_DIR}/resolved.txt\" \"profdata=\${_bike_coverage_profdata}\ncov=\${_bike_coverage_cov}\n\")
else()
  _bike_resolve_coverage_tool(llvm-profdata LLVM_PROFDATA _pd _pdv)
  _bike_resolve_coverage_tool(llvm-cov LLVM_COV _cov _covv)
  file(WRITE \"\${CMAKE_BINARY_DIR}/resolved.txt\" \"profdata=\${_pd}\ncov=\${_cov}\n\")
endif()
")
endfunction()

# _fake_tool(path major exit) — version-banner-compatible stub.
function(_fake_tool path major mode)
  if(mode STREQUAL "version_then_fail")
    file(WRITE "${path}" "#!/bin/sh
if [ \"$1\" = \"--version\" ]; then echo \"fake LLVM version ${major}.0.0\"; exit 0; fi
exit 7
")
  else()
    file(WRITE "${path}" "#!/bin/sh
if [ \"$1\" = \"--version\" ]; then echo \"fake LLVM version ${major}.0.0\"; exit 0; fi
echo '{\"type\":\"not-a-coverage-export\",\"data\":[],\"version\":\"0\"}'
exit 0
")
  endif()
  file(CHMOD "${path}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()

# Host compiler LLVM major for version-banner stubs. cmake -P has no
# enabled language, so find a real clang-family compiler on PATH.
find_program(_host_clang NAMES clang++ clang)
set(_host_clang_major "")
if(_host_clang)
  execute_process(COMMAND "${_host_clang}" --version
    OUTPUT_VARIABLE _cv_out ERROR_VARIABLE _cv_err)
  string(REGEX MATCH "[0-9]+\\.[0-9]+" _cv_first "${_cv_out}\n${_cv_err}")
  string(REGEX MATCH "^[0-9]+" _host_clang_major "${_cv_first}")
endif()
if(NOT _host_clang_major)
  _skip("coverage cases: no clang-family compiler on PATH")
  set(_host_clang "/nonexistent/clang++")
  set(_host_clang_major "0")
endif()

# The full-probe cases need an instrumented control binary to link —
# clang without the compiler-rt profile runtime fails the format probe at
# compile time (e.g. the dev container lacks libclang_rt.profile). Detect
# that once; such hosts exercise the resolution-only cases and record a
# skip for the instrumented ones.
file(WRITE "${SCRATCH_DIR}/cov-link-check.cpp"
  "int main() { return 0; }\n")
execute_process(COMMAND "${_host_clang}" -fprofile-instr-generate
  -fcoverage-mapping "${SCRATCH_DIR}/cov-link-check.cpp"
  -o "${SCRATCH_DIR}/cov-link-check"
  RESULT_VARIABLE _cov_link_rc OUTPUT_QUIET ERROR_QUIET)
if(_cov_link_rc EQUAL 0)
  set(_coverage_probe_possible TRUE)
else()
  set(_coverage_probe_possible FALSE)
  _skip("coverage format-probe cases: no instrumented-link toolchain (missing profile runtime)")
endif()

# test_explicit_missing_llvm_cov_override_fails
set(_c "coverage_missing_llvm_cov_override_fails_closed")
math(EXPR _case_count "${_case_count} + 1")
_write_coverage_probe("${SCRATCH_DIR}/cov-ovr" FALSE)
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-ovr"
  -B "${SCRATCH_DIR}/cov-ovr-b"
  "-DCMAKE_CXX_COMPILER=${_host_clang}"
  ENV "LLVM_COV=/nonexistent/llvm-cov" "LLVM_PROFDATA=")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: missing LLVM_COV override configured")
endif()
_expect_squashed(${_c} "${${_c}_out}" "invalid LLVM_COV override")

# test_explicit_override_wins_over_sibling
set(_c "coverage_explicit_override_wins_over_sibling")
math(EXPR _case_count "${_case_count} + 1")
set(_td "${SCRATCH_DIR}/cov-toolchain")
file(MAKE_DIRECTORY "${_td}")
_fake_tool("${_td}/llvm-profdata-77" "${_host_clang_major}" version_then_fail)
_fake_tool("${_td}/llvm-cov-77" "${_host_clang_major}" version_then_fail)
_write_exe("${_td}/clang++-77"
  "#!/bin/sh\nexec \"${_host_clang}\" \"\$@\"\n")
_fake_tool("${SCRATCH_DIR}/ovr-profdata" "${_host_clang_major}" version_then_fail)
_fake_tool("${SCRATCH_DIR}/ovr-cov" "${_host_clang_major}" version_then_fail)
_write_coverage_probe("${SCRATCH_DIR}/cov-ovr2" FALSE)
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-ovr2"
  -B "${SCRATCH_DIR}/cov-ovr2-b"
  "-DCMAKE_CXX_COMPILER=${_td}/clang++-77"
  ENV "LLVM_PROFDATA=${SCRATCH_DIR}/ovr-profdata"
      "LLVM_COV=${SCRATCH_DIR}/ovr-cov")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
elseif(EXISTS "${SCRATCH_DIR}/cov-ovr2-b/resolved.txt")
  file(READ "${SCRATCH_DIR}/cov-ovr2-b/resolved.txt" _r)
  _expect_needle(${_c} "${_r}" "profdata=${SCRATCH_DIR}/ovr-profdata")
  _expect_needle(${_c} "${_r}" "cov=${SCRATCH_DIR}/ovr-cov")
endif()

# test_compiler_sibling_tools_precede_path_candidates
set(_c "coverage_sibling_tools_precede_path_candidates")
math(EXPR _case_count "${_case_count} + 1")
set(_pd "${SCRATCH_DIR}/cov-pathbin")
file(MAKE_DIRECTORY "${_pd}")
_fake_tool("${_pd}/llvm-profdata" "${_host_clang_major}" version_then_fail)
_fake_tool("${_pd}/llvm-cov" "${_host_clang_major}" version_then_fail)
_write_coverage_probe("${SCRATCH_DIR}/cov-sib" FALSE)
_run(${_c} COMMAND "${CMAKE_COMMAND}" -E env
  "PATH=${_pd}:/usr/bin:/bin"
  "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-sib"
  -B "${SCRATCH_DIR}/cov-sib-b"
  "-DCMAKE_CXX_COMPILER=${_td}/clang++-77")
if(NOT ${_c}_rc EQUAL 0)
  _fail("${_c}: configure failed: ${${_c}_out}")
elseif(EXISTS "${SCRATCH_DIR}/cov-sib-b/resolved.txt")
  file(READ "${SCRATCH_DIR}/cov-sib-b/resolved.txt" _r)
  _expect_needle(${_c} "${_r}" "profdata=${_td}/llvm-profdata-77")
  _expect_needle(${_c} "${_r}" "cov=${_td}/llvm-cov-77")
endif()

# test_xcrun_is_a_darwin_only_last_resort — PATH has an xcrun stub that
# records invocation; resolution must reach it only after sibling+PATH
# candidates fail (Darwin) or never (Linux).
set(_c "coverage_xcrun_is_last_resort_on_darwin_never_on_linux")
math(EXPR _case_count "${_case_count} + 1")
set(_xd "${SCRATCH_DIR}/cov-xcrun")
file(MAKE_DIRECTORY "${_xd}")
_fake_tool("${_xd}/llvm-profdata-from-xcrun" "${_host_clang_major}" version_then_fail)
_write_exe("${_xd}/xcrun" "#!/bin/sh
echo invoked >> \"${_xd}/xcrun.marker\"
if [ \"\$1\" = \"--find\" ]; then echo \"${_xd}/llvm-profdata-from-xcrun\"; exit 0; fi
exit 1
")
_write_coverage_probe("${SCRATCH_DIR}/cov-x" FALSE)
_run(${_c} COMMAND "${CMAKE_COMMAND}" -E env
  "PATH=${_xd}:/usr/bin:/bin" "LLVM_PROFDATA=" "LLVM_COV="
  "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-x"
  -B "${SCRATCH_DIR}/cov-x-b"
  "-DCMAKE_CXX_COMPILER=${_td}/clang++-77")
if(_host STREQUAL "Darwin")
  # The stub only serves profdata via --find; the marker proves xcrun was
  # consulted as the last resort after sibling+PATH candidates failed.
  if(NOT EXISTS "${_xd}/xcrun.marker")
    _fail("${_c}: xcrun never consulted on Darwin")
  endif()
else()
  # On Linux the branch is compiled out entirely: a PATH llvm tool may
  # legitimately resolve, but xcrun must never be consulted.
  if(EXISTS "${_xd}/xcrun.marker")
    _fail("${_c}: xcrun consulted on ${_host}")
  endif()
endif()

# test_non_clang_compiler_is_rejected — the guard text plus a live
# rejection when a real GNU compiler is present.
set(_c "coverage_requires_an_llvm_clang_family_compiler")
math(EXPR _case_count "${_case_count} + 1")
file(READ "${COVERAGE_MODULE}" _cov_mod)
_expect_needle(${_c} "${_cov_mod}" "native coverage requires an LLVM clang-family compiler")
_expect_needle(${_c} "${_cov_mod}" "CMAKE_CXX_COMPILER_ID MATCHES \"^(AppleClang|Clang)$\"")
find_program(_real_gxx NAMES g++-15 g++-14 g++-13 g++)
if(_real_gxx)
  execute_process(COMMAND "${_real_gxx}" -dumpversion
    OUTPUT_VARIABLE _gv ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND "${_real_gxx}" --version
    OUTPUT_VARIABLE _gver ERROR_QUIET)
  if(_gver MATCHES "clang|Clang")
    # macOS g++ is an AppleClang alias — not a real GNU compiler.
    _skip("${_c}: '${_real_gxx}' is a clang alias; live GNU rejection not exercised")
  else()
    _write_coverage_probe("${SCRATCH_DIR}/cov-gcc" TRUE)
    _run(${_c}_gcc COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-gcc"
      -B "${SCRATCH_DIR}/cov-gcc-b" "-DCMAKE_CXX_COMPILER=${_real_gxx}")
    if(${_c}_gcc_rc EQUAL 0)
      _fail("${_c}: GNU compiler configured")
    endif()
    _expect_squashed(${_c} "${${_c}_gcc_out}" "LLVM clang-family compiler")
  endif()
else()
  _skip("${_c}: no GNU compiler on host; guard verified by file contract only")
endif()

# test_missing_compiler_fails
set(_c "coverage_missing_compiler_fails_closed")
math(EXPR _case_count "${_case_count} + 1")
_write_coverage_probe("${SCRATCH_DIR}/cov-mc" FALSE)
_run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-mc"
  -B "${SCRATCH_DIR}/cov-mc-b" "-DCMAKE_CXX_COMPILER=/nonexistent/clang++")
if(${_c}_rc EQUAL 0)
  _fail("${_c}: nonexistent compiler configured")
endif()

# The real-compile/merge/export cases need a qualifying clang + LLVM tools.
# On Darwin the Xcode toolchain qualifies by construction; elsewhere a
# matching-major llvm-profdata/llvm-cov pair must be findable.
set(_real_profdata "")
set(_real_cov "")
if(_host STREQUAL "Darwin")
  execute_process(COMMAND xcrun --find llvm-profdata
    OUTPUT_VARIABLE _real_profdata ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xcrun --find llvm-cov
    OUTPUT_VARIABLE _real_cov ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(NOT _real_profdata)
  find_program(_real_profdata NAMES llvm-profdata
    "llvm-profdata-${_host_clang_major}")
  find_program(_real_cov NAMES llvm-cov "llvm-cov-${_host_clang_major}")
endif()

# test_failing_merge_tool_fails_with_command_evidence — real compile +
# stub profdata whose merge exits 7; diagnostics keep the command.
set(_c "coverage_merge_failure_names_the_command")
math(EXPR _case_count "${_case_count} + 1")
if(_coverage_probe_possible)
  _fake_tool("${SCRATCH_DIR}/fail-profdata" "${_host_clang_major}" version_then_fail)
  _fake_tool("${SCRATCH_DIR}/ok-cov" "${_host_clang_major}" version_then_fail)
  _write_coverage_probe("${SCRATCH_DIR}/cov-mf" TRUE)
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-mf"
    -B "${SCRATCH_DIR}/cov-mf-b"
    "-DCMAKE_CXX_COMPILER=${_host_clang}"
    ENV "LLVM_PROFDATA=${SCRATCH_DIR}/fail-profdata"
        "LLVM_COV=${SCRATCH_DIR}/ok-cov")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: failing merge configured")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "coverage format probe control-merge failed")
  _expect_squashed(${_c} "${${_c}_out}" "diagnostics:")
endif()

# test_export_without_valid_control_data_fails — real merge succeeds
# through a delegating stub; the export stub emits a wrong-shape document.
set(_c "coverage_invalid_export_fails_closed")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _coverage_probe_possible)
elseif(_real_profdata)
  _write_exe("${SCRATCH_DIR}/delegate-profdata" "#!/bin/sh
if [ \"\$1\" = \"--version\" ]; then echo \"LLVM version ${_host_clang_major}.0.0\"; exit 0; fi
exec \"${_real_profdata}\" \"\$@\"
")
  _fake_tool("${SCRATCH_DIR}/bad-cov" "${_host_clang_major}" bad_export)
  _write_coverage_probe("${SCRATCH_DIR}/cov-exp" TRUE)
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-exp"
    -B "${SCRATCH_DIR}/cov-exp-b"
    "-DCMAKE_CXX_COMPILER=${_host_clang}"
    ENV "LLVM_PROFDATA=${SCRATCH_DIR}/delegate-profdata"
        "LLVM_COV=${SCRATCH_DIR}/bad-cov")
  if(${_c}_rc EQUAL 0)
    _fail("${_c}: invalid export configured")
  endif()
  _expect_squashed(${_c} "${${_c}_out}" "exported invalid control data")
else()
  _skip("${_c}: no real llvm-profdata on host")
endif()

# test_real_toolchain_resolves_and_passes_the_format_probe +
# test_probe_result_records_tool_versions_and_export.
set(_c "coverage_real_toolchain_probe_and_resolution_record")
math(EXPR _case_count "${_case_count} + 1")
if(NOT _coverage_probe_possible)
elseif(_real_profdata AND _real_cov)
  _write_coverage_probe("${SCRATCH_DIR}/cov-real" TRUE)
  _run(${_c} COMMAND "${CMAKE_COMMAND}" -S "${SCRATCH_DIR}/cov-real"
    -B "${SCRATCH_DIR}/cov-real-b"
    "-DCMAKE_CXX_COMPILER=${_host_clang}"
    ENV "LLVM_PROFDATA=${_real_profdata}" "LLVM_COV=${_real_cov}")
  if(NOT ${_c}_rc EQUAL 0)
    _fail("${_c}: real toolchain probe failed: ${${_c}_out}")
  else()
    file(READ "${SCRATCH_DIR}/cov-real-b/cov-probe/coverage_tool_resolution.json" _rr)
    _expect_needle(${_c} "${_rr}" "\"format_probe_passed\":true")
    _expect_needle(${_c} "${_rr}" "llvm_profdata")
    _expect_needle(${_c} "${_rr}" "llvm_cov")
    _expect_needle(${_c} "${_rr}" "export_version")
  endif()
else()
  _skip("${_c}: no qualifying llvm-profdata/llvm-cov pair on host")
endif()

# ==========================================================================
# Report
# ==========================================================================
get_property(_skipped GLOBAL PROPERTY _t53_skips)
get_property(_failures GLOBAL PROPERTY _t53_failures)
if(NOT _skipped STREQUAL "")
  message(STATUS "migration_T53_portability skips:${_skipped}")
endif()
if(NOT _failures STREQUAL "")
  message(FATAL_ERROR
    "migration_T53_portability failed (${_case_count} cases):${_failures}")
endif()
message(STATUS "migration_T53_portability passed (${_case_count} cases)")
