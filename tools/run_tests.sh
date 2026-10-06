#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: bash tools/run_tests.sh [quick|native|full] [pytest arguments...]

  quick   Run Python tests without the slow marker or direct native tests.
  native  Configure/build the selected extension, run every native sweep and
          CTest, then native/golden tests against that extension.
  full    Run the same native preflight, then all tests including slow episodes.
  -h, --help  Show this help.

All profiles use -q --durations=10 and forward additional pytest arguments.
NATIVE_TEST_BUILD_PATH selects an absolute build directory. The legacy
NATIVE_TEST_BUILD_DIR values '', asan, rtsan, and coverage remain supported.
EOF
}

profile="${1:-quick}"
if (($# > 0)); then
  shift
fi

pytest_args=(-q --durations=10)
# Python dev mode: DeprecationWarning/ResourceWarning surface + extra
# runtime checks — free on every profile.
export PYTHONDEVMODE=1
case "$profile" in
  quick)
    pytest_args+=(-m 'not slow')
    # malloc debugging on the fast profile only — `full` measures realtime.
    export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
    # Direct native tests require the selected, built extension. Keep the
    # tool-controller unit tests in quick; the native profile owns the rest.
    for native_file in "$REPO_ROOT"/tests/reference/test_native_*.py; do
      [[ -e "$native_file" ]] || continue
      if [[ "${native_file##*/}" != test_native_check_tools.py ]]; then
        pytest_args+=(--ignore "$native_file")
      fi
    done
    ;;
  native)
    # The full native verification chain is shared with `full` below.
    export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
    native_files=(tests/reference/test_native_*.py tests/reference/test_golden_episode*.py)
    pytest_args+=("${native_files[@]}")
    ;;
  full)
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    printf 'Unknown test profile: %s\n' "$profile" >&2
    usage >&2
    exit 2
    ;;
esac

native_runtime_env=()

cache_value() {
  local key="$1"
  local cache_file="$2"
  sed -n "s/^${key}:BOOL=//p" "$cache_file" | tail -n 1
}

resolve_native_runtime() {
  local build_dir="$1"
  local cache_file="$build_dir/CMakeCache.txt"
  local asan_enabled rtsan_enabled compiler runtime_name preload_name system_name
  native_runtime_env=()

  asan_enabled="$(cache_value NATIVE_SANITIZE "$cache_file")"
  rtsan_enabled="$(cache_value NATIVE_RTSAN "$cache_file")"
  if [[ "$asan_enabled" != ON && "$rtsan_enabled" != ON ]]; then
    return 0
  fi
  if [[ "$asan_enabled" == ON && "$rtsan_enabled" == ON ]]; then
    printf 'selected build enables ASan and RTSan together; runtime injection is ambiguous: %s\n' "$build_dir" >&2
    return 1
  fi

  compiler="$(sed -n 's/^CMAKE_CXX_COMPILER:FILEPATH=//p; s/^CMAKE_CXX_COMPILER:STRING=//p' "$cache_file" | head -n 1)"
  if [[ -z "$compiler" || ! -x "$compiler" ]]; then
    printf 'cannot resolve selected build compiler from %s\n' "$cache_file" >&2
    return 1
  fi

  system_name="$(uname -s)"
  if [[ "$system_name" == Darwin ]]; then
    preload_name=DYLD_INSERT_LIBRARIES
    if [[ "$asan_enabled" == ON ]]; then
      runtime_name=libclang_rt.asan_osx_dynamic.dylib
    else
      runtime_name=libclang_rt.rtsan_osx_dynamic.dylib
    fi
  elif [[ "$system_name" == Linux ]]; then
    preload_name=LD_PRELOAD
    if [[ "$asan_enabled" == ON && "$compiler" != *clang* ]]; then
      runtime_name=libasan.so
    elif [[ "$asan_enabled" == ON ]]; then
      runtime_name="libclang_rt.asan-$(uname -m).so"
    else
      runtime_name="libclang_rt.rtsan-$(uname -m).so"
    fi
  else
    printf 'selected sanitizer runtime is unsupported on %s\n' "$system_name" >&2
    return 1
  fi

  local runtime_path selected_runtime_path
  runtime_path="$("$compiler" "-print-file-name=$runtime_name")"
  if [[ ! -f "$runtime_path" ]]; then
    printf 'selected compiler %s did not resolve sanitizer runtime %s (got %s)\n' \
      "$compiler" "$runtime_name" "$runtime_path" >&2
    return 1
  fi

  selected_runtime_path="$runtime_path"
  local existing_preload="${!preload_name:-}"
  if [[ -n "$existing_preload" ]]; then
    runtime_path="$runtime_path:$existing_preload"
  fi
  native_runtime_env+=("$preload_name=$runtime_path")
  native_runtime_env+=("NATIVE_TEST_SANITIZER_RUNTIME=$selected_runtime_path")
  if [[ "$asan_enabled" == ON ]]; then
    local asan_options="${ASAN_OPTIONS:-}"
    if [[ -n "$asan_options" ]]; then
      asan_options="$asan_options:"
    fi
    native_runtime_env+=("ASAN_OPTIONS=${asan_options}detect_leaks=0:detect_stack_use_after_return=1")
    native_runtime_env+=(PYTHONMALLOC=malloc)
  fi
}

run_with_native_runtime() {
  if ((${#native_runtime_env[@]})); then
    uv run env "${native_runtime_env[@]}" "$@"
  else
    uv run "$@"
  fi
}

preflight_native() {
  local build_dir="$1"
  local target

  printf 'selected native build: %s\n' "$build_dir"
  uv run cmake -S "$REPO_ROOT/native" -B "$build_dir"
  uv run cmake --build "$build_dir"
  for target in frontends tidy analyzer cppcheck odr scripts; do
    uv run cmake --build "$build_dir" --target "check_$target"
  done
  for kind in headers diagnostic-controls context; do
    uv run python "$REPO_ROOT/tools/native_checks.py" --kind "$kind" --build "$build_dir"
  done

  resolve_native_runtime "$build_dir"
  run_with_native_runtime ctest --test-dir "$build_dir" --output-on-failure
  local extension_path
  extension_path="$(run_with_native_runtime python -c 'from native_loader import load_native; print(load_native().__file__)')"
  printf 'selected native extension: %s\n' "$extension_path"
}

if [[ "$profile" == native || "$profile" == full ]]; then
  export PYTHONPATH="$REPO_ROOT/tests/reference${PYTHONPATH:+:$PYTHONPATH}"
  selected_build="$(uv run python -c 'import os, sys; sys.path.insert(0, "tests/reference"); from native_loader import selected_build; print(selected_build(os.environ))')"
  export NATIVE_TEST_BUILD_PATH="$selected_build"
  preflight_native "$selected_build"
  pytest_args+=(-p native_test_reporter)
fi

if [[ "$profile" == native || "$profile" == full ]]; then
  run_with_native_runtime python -m pytest "${pytest_args[@]}" "$@"
else
  exec uv run python -m pytest "${pytest_args[@]}" "$@"
fi
