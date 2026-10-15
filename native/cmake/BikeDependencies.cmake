# Selected-interpreter dependency discovery for the native build.
#
# find_package(Python) must already have run: the SAME Python_EXECUTABLE runs
# tools/native_dependencies.py, which probes inside its own environment —
# mujoco/nanobind locations come from the interpreter the extension links
# against, never whatever python happens to be on PATH. The returned schema-1
# document is consumed exclusively through string(JSON) member access —
# returned text is never evaluated — and every reported path is re-validated
# here before use.
#
# Exports consumed by the rest of CMakeLists.txt (historical names kept):
#   Bike::MuJoCo            GLOBAL IMPORTED SHARED target (location + includes)
#   MJ_DIR                  mujoco package directory (include/ and the engine
#                           image live directly inside it)
#   _native_mujoco_library  realpath of the selected engine image
#   _native_nb_include      nanobind include directory
#   _native_robin_include   nanobind's bundled tsl::robin_map include dir
#   _native_python_include  selected interpreter's sysconfig include dir

if(NOT DEFINED Python_EXECUTABLE OR NOT EXISTS "${Python_EXECUTABLE}")
  message(FATAL_ERROR
    "BikeDependencies.cmake requires find_package(Python) to have selected "
    "Python_EXECUTABLE first; got '${Python_EXECUTABLE}'")
endif()

set(_native_dependencies_probe
  "${CMAKE_CURRENT_SOURCE_DIR}/../tools/native_dependencies.py")
if(NOT EXISTS "${_native_dependencies_probe}")
  message(FATAL_ERROR
    "dependency probe is missing: ${_native_dependencies_probe}")
endif()

# The probe subprocess IS the selected interpreter; a nonzero exit carries the
# named dependency error on stderr, which is reproduced verbatim.
execute_process(
  COMMAND "${Python_EXECUTABLE}" "${_native_dependencies_probe}"
          --python "${Python_EXECUTABLE}"
  RESULT_VARIABLE _native_dependencies_result
  OUTPUT_VARIABLE _native_dependencies_json
  ERROR_VARIABLE _native_dependencies_error
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _native_dependencies_result EQUAL 0)
  message(FATAL_ERROR
    "dependency probe failed for interpreter '${Python_EXECUTABLE}':\n"
    "${_native_dependencies_error}")
endif()
if(NOT _native_dependencies_error STREQUAL "")
  message(STATUS "dependency probe stderr: ${_native_dependencies_error}")
endif()

# Typed member access into the probe document. A parse error, a missing
# member (-NOTFOUND), or an empty value are all fatal — fail closed before any
# reported path reaches a target.
macro(_native_dependency_field _out_var)
  string(JSON ${_out_var} ERROR_VARIABLE _native_json_error
    GET "${_native_dependencies_json}" ${ARGN})
  if(_native_json_error OR "${${_out_var}}" STREQUAL ""
     OR "${${_out_var}}" MATCHES "-NOTFOUND$")
    message(FATAL_ERROR
      "dependency probe document is missing required field "
      "'${ARGN}': ${_native_json_error}")
  endif()
endmacro()

_native_dependency_field(_native_dep_schema_version schema_version)
_native_dependency_field(_native_dep_platform_system platform system)
_native_dependency_field(_native_dep_python_executable python executable)
_native_dependency_field(_native_dep_python_version python version)
_native_dependency_field(_native_dep_python_soabi python soabi)
_native_dependency_field(_native_dep_python_include python include_dir)
_native_dependency_field(_native_dep_mujoco_version mujoco version)
_native_dependency_field(_native_dep_mujoco_include mujoco include_dir)
_native_dependency_field(_native_dep_mujoco_library mujoco library_path)
_native_dependency_field(_native_dep_nb_cmake_dir nanobind cmake_dir)
_native_dependency_field(_native_dep_nb_include nanobind include_dir)

if(NOT _native_dep_schema_version EQUAL 1)
  message(FATAL_ERROR
    "unsupported dependency probe schema_version "
    "'${_native_dep_schema_version}'")
endif()
if(NOT _native_dep_platform_system STREQUAL "${BIKE_TARGET_SYSTEM}")
  message(FATAL_ERROR
    "dependency probe platform '${_native_dep_platform_system}' does not "
    "match build platform '${BIKE_TARGET_SYSTEM}'")
endif()
if(NOT _native_dep_mujoco_version STREQUAL "3.12.0")
  message(FATAL_ERROR
    "bike_native requires the pinned MuJoCo 3.12.0 error ABI; the selected "
    "interpreter provides '${_native_dep_mujoco_version}'")
endif()

# Every consumed path must be absolute and exist on this host.
foreach(_native_dep_var
    _native_dep_python_executable _native_dep_python_include
    _native_dep_mujoco_include _native_dep_mujoco_library
    _native_dep_nb_cmake_dir _native_dep_nb_include)
  if(NOT IS_ABSOLUTE "${${_native_dep_var}}")
    message(FATAL_ERROR
      "dependency probe path is not absolute: ${${_native_dep_var}}")
  endif()
  if(NOT EXISTS "${${_native_dep_var}}")
    message(FATAL_ERROR
      "dependency probe path does not exist: ${${_native_dep_var}}")
  endif()
endforeach()
foreach(_native_dep_dir_var
    _native_dep_python_include _native_dep_mujoco_include
    _native_dep_nb_cmake_dir _native_dep_nb_include)
  if(NOT IS_DIRECTORY "${${_native_dep_dir_var}}")
    message(FATAL_ERROR
      "dependency probe path is not a directory: ${${_native_dep_dir_var}}")
  endif()
endforeach()
if(NOT EXISTS "${_native_dep_mujoco_include}/mujoco/mujoco.h")
  message(FATAL_ERROR
    "MuJoCo public headers are missing under ${_native_dep_mujoco_include}")
endif()
if(NOT EXISTS "${_native_dep_python_include}/Python.h")
  message(FATAL_ERROR
    "Python.h is missing under the selected interpreter's include dir "
    "${_native_dep_python_include}")
endif()

# One-interpreter contract: the interpreter that answered the probe must be
# the interpreter find_package selected (compared by resolved identity so a
# venv/bin alias and its resolved name stay equivalent).
get_filename_component(_native_dep_invoked_python "${Python_EXECUTABLE}" REALPATH)
get_filename_component(_native_dep_reported_python "${_native_dep_python_executable}" REALPATH)
if(NOT _native_dep_invoked_python STREQUAL _native_dep_reported_python)
  message(FATAL_ERROR
    "dependency probe ran in a different interpreter than the selected one: "
    "selected='${Python_EXECUTABLE}', reported='${_native_dep_python_executable}'")
endif()

# The reported engine image must carry the exact platform filename — the probe
# already enforces this; a contradictory document is a hard error, not a fix.
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  set(_native_dep_mujoco_expected "libmujoco.3.12.0.dylib")
else()
  set(_native_dep_mujoco_expected "libmujoco.so.3.12.0")
endif()
get_filename_component(_native_dep_mujoco_name "${_native_dep_mujoco_library}" NAME)
if(NOT _native_dep_mujoco_name STREQUAL "${_native_dep_mujoco_expected}")
  message(FATAL_ERROR
    "dependency probe selected an unexpected MuJoCo image "
    "'${_native_dep_mujoco_library}'; expected '${_native_dep_mujoco_expected}'")
endif()

get_filename_component(_native_mujoco_library "${_native_dep_mujoco_library}" REALPATH)
string(REGEX REPLACE "/+$" "" _native_dep_mujoco_include "${_native_dep_mujoco_include}")
get_filename_component(MJ_DIR "${_native_dep_mujoco_include}" DIRECTORY)

# MuJoCo 3.12's mjsan.h declares its ASan stack-frame helpers with the
# attribute between the declarator and the body —
#   static inline void mj_markStack(mjData* d) __attribute__((always_inline)) {
# — a spelling GCC's C++ frontend rejects ("attributes are not allowed on
# a function-definition") whenever __SANITIZE_ADDRESS__ is live, so the
# GNU ASan lane cannot parse any TU that reaches <mujoco/mujoco.h>. Probe
# the real header under the sanitizer flag; when the frontend rejects it,
# emit a generated shadow header — identical text except the attribute
# moves ahead of the return type — into
# ${CMAKE_BINARY_DIR}/mujoco_compat/mujoco/ and search that directory
# first. The transform matches the pinned 3.12.0 text exactly, fails the
# configure on drift, and a re-probe proves the shadow compiles.
# BIKE_MUJOCO_COMPAT_INCLUDE is empty whenever the real header parses.
set(BIKE_MUJOCO_COMPAT_INCLUDE "")
if(NATIVE_SANITIZE)
  set(_mjsan_probe_dir "${CMAKE_BINARY_DIR}/native_check_work/mujoco-headers")
  file(MAKE_DIRECTORY "${_mjsan_probe_dir}")
  # mjsan.h is a fragment header: it needs the mjData declaration that
  # mujoco.h establishes via mjdata.h, so the probe includes the public
  # umbrella header — the same path every first-party TU reaches.
  file(WRITE "${_mjsan_probe_dir}/probe.cpp"
    "#include <mujoco/mujoco.h>\nint main() { return 0; }\n")
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -std=c++23 -fsanitize=address
      "-I${_native_dep_mujoco_include}" -fsyntax-only -x c++
      "${_mjsan_probe_dir}/probe.cpp"
    RESULT_VARIABLE _mjsan_probe_rc
    OUTPUT_QUIET ERROR_VARIABLE _mjsan_probe_err)
  if(NOT _mjsan_probe_rc EQUAL 0)
    file(READ "${_native_dep_mujoco_include}/mujoco/mjsan.h" _mjsan_text)
    foreach(_mjsan_fn mj_markStack mj_freeStack)
      set(_mjsan_broken
        "static inline void ${_mjsan_fn}(mjData* d) __attribute__((always_inline)) {")
      set(_mjsan_fixed
        "static inline __attribute__((always_inline)) void ${_mjsan_fn}(mjData* d) {")
      string(FIND "${_mjsan_text}" "${_mjsan_broken}" _mjsan_pos)
      if(_mjsan_pos EQUAL -1)
        message(FATAL_ERROR
          "${CMAKE_CXX_COMPILER_ID} rejects mujoco's mjsan.h under ASan "
          "and the header text drifted from the known GCC-incompatible "
          "spelling for ${_mjsan_fn} — inspect and fix the compat "
          "transform:\n${_mjsan_probe_err}")
      endif()
      string(REPLACE "${_mjsan_broken}" "${_mjsan_fixed}"
        _mjsan_text "${_mjsan_text}")
    endforeach()
    set(_native_mujoco_compat_dir "${CMAKE_BINARY_DIR}/mujoco_compat")
    file(MAKE_DIRECTORY "${_native_mujoco_compat_dir}/mujoco")
    file(WRITE "${_native_mujoco_compat_dir}/mujoco/mjsan.h"
      "${_mjsan_text}")
    execute_process(
      COMMAND "${CMAKE_CXX_COMPILER}" -std=c++23 -fsanitize=address
        "-I${_native_mujoco_compat_dir}" "-I${_native_dep_mujoco_include}"
        -fsyntax-only -x c++ "${_mjsan_probe_dir}/probe.cpp"
      RESULT_VARIABLE _mjsan_shadow_rc
      OUTPUT_QUIET ERROR_VARIABLE _mjsan_shadow_err)
    if(NOT _mjsan_shadow_rc EQUAL 0)
      message(FATAL_ERROR
        "the generated mjsan.h compat header still fails the ASan probe "
        "under ${CMAKE_CXX_COMPILER_ID}:\n${_mjsan_shadow_err}")
    endif()
    set(BIKE_MUJOCO_COMPAT_INCLUDE "${_native_mujoco_compat_dir}")
    message(STATUS
      "mujoco mjsan.h is rejected by ${CMAKE_CXX_COMPILER_ID} under ASan; "
      "a generated attribute-compatible shadow in "
      "${_native_mujoco_compat_dir} precedes the real include dir")
  endif()
endif()

set(_native_mujoco_interface_includes "${_native_dep_mujoco_include}")
if(BIKE_MUJOCO_COMPAT_INCLUDE)
  list(PREPEND _native_mujoco_interface_includes "${BIKE_MUJOCO_COMPAT_INCLUDE}")
endif()
add_library(Bike::MuJoCo SHARED IMPORTED GLOBAL)
set_target_properties(Bike::MuJoCo PROPERTIES
  IMPORTED_LOCATION "${_native_mujoco_library}"
  INTERFACE_INCLUDE_DIRECTORIES "${_native_mujoco_interface_includes}")

list(APPEND CMAKE_PREFIX_PATH "${_native_dep_nb_cmake_dir}")
find_package(nanobind CONFIG REQUIRED)

# Standalone third-party header roots for the contract/ODR sweeps, resolved
# from the same probe document (never re-queried from another interpreter).
set(_native_nb_include "${_native_dep_nb_include}")
get_filename_component(_native_nb_package "${_native_nb_include}/.." REALPATH)
get_filename_component(_native_robin_include
  "${_native_nb_package}/ext/robin_map/include" REALPATH)
get_filename_component(_native_python_include
  "${_native_dep_python_include}" REALPATH)

# The stock 3.12.0 Python binding uses these private TLS functions. Check both
# declarations and linked symbols against the selected engine image before
# building any target that could otherwise fall back to MuJoCo's
# process-exiting handler — never replace this TLS contract with
# mju_user_error or a process-global callback. The push/pop keeps the
# REQUIRED_* settings from contaminating other checks; the static_assert and
# the mj_version() link reference pin the public header/runtime version.
include(CheckCXXSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src" "${_native_dep_mujoco_include}")
set(CMAKE_REQUIRED_LIBRARIES Bike::MuJoCo)
unset(BIKE_MUJOCO_ERROR_ABI_312 CACHE)
check_cxx_source_compiles([=[
#include "engine_abi_312.hpp"
static_assert(mjVERSION_HEADER == 3012000,
              "selected MuJoCo headers do not declare version 3.12.0");
int main() {
  mjfLogHandler previous = _mjPRIVATE_setTlsLogHandler(nullptr);
  (void) _mjPRIVATE_setTlsLogHandler(previous);
  void (*volatile raw_data)(mjData**, const mjModel*) = &mj_makeRawData;
  return _mjPRIVATE_getGlobalLogHandler() == nullptr || raw_data == nullptr
      || mj_version() != mjVERSION_HEADER;
}
]=] BIKE_MUJOCO_ERROR_ABI_312)
cmake_pop_check_state()
if(NOT BIKE_MUJOCO_ERROR_ABI_312)
  message(FATAL_ERROR
    "MuJoCo 3.12.0 private TLS error-handler ABI is unavailable for "
    "${Python_EXECUTABLE}")
endif()
