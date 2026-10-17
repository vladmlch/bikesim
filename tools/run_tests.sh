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

native_uv() {
  uv run --frozen --group native "$@"
}

profile="${1:-quick}"
if (($# > 0)); then
  shift
fi

pytest_args=(-q --durations=10)
# output/ is gitignored scratch space (probe fixtures, extracted bundles,
# vendored trees). It can contain whole copied test suites that collide
# with the real tests by module name; run artifacts are never suite
# members, so every profile ignores that directory during collection.
pytest_args+=(--ignore "$REPO_ROOT/output")
# Python dev mode: DeprecationWarning/ResourceWarning surface + extra
# runtime checks — free on every profile.
export PYTHONDEVMODE=1
case "$profile" in
  quick)
    pytest_args+=(-m 'not slow')
    # malloc debugging on the fast profile only — `full` measures realtime.
    # These are Darwin allocator variables; they must never be exported on
    # Linux, where invented allocator settings would perturb numerical and
    # performance baselines instead of debugging them.
    if [[ "$(uname -s)" == Darwin ]]; then
      export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
    fi
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
    # Darwin allocator debugging only — never exported on Linux.
    if [[ "$(uname -s)" == Darwin ]]; then
      export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
    fi
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

# Every consumer that must run under the selected build's sanitizer runtime
# goes through tools/native_runtime.py: it loads the build's validated
# schema-2 context, cross-checks it against the CMakeCache selections,
# resolves the compiler-matched runtime by the recorded toolchain identity
# (never the driver basename), prepends it to the platform preload variable
# and runs the argv array — propagating the child's exit status and failing
# closed on any inconsistent state. Uninstrumented builds pass the caller
# environment through unchanged.
run_with_native_runtime() {
  native_uv python "$REPO_ROOT/tools/native_runtime.py" run \
    --build "${NATIVE_TEST_BUILD_PATH:?NATIVE_TEST_BUILD_PATH must select the build}" -- "$@"
}

preflight_native() {
  local build_dir="$1"
  local target

  if [[ -z "${NATIVE_TEST_PROVENANCE_PATH:-}" ]]; then
    export NATIVE_TEST_PROVENANCE_PATH="$build_dir/native_test_provenance.json"
  fi
  printf 'selected native build: %s\n' "$build_dir"
  native_uv python -c 'import mujoco, nanobind, sys; sys.exit(0 if mujoco.__version__ == "3.12.0" else f"native build requires MuJoCo 3.12.0, found {mujoco.__version__}")'
  native_uv cmake -S "$REPO_ROOT/native" -B "$build_dir"
  native_uv cmake --build "$build_dir"
  for target in frontends tidy analyzer cppcheck odr scripts; do
    native_uv cmake --build "$build_dir" --target "check_$target"
  done
  for kind in headers diagnostic-controls context; do
    native_uv python "$REPO_ROOT/tools/native_checks.py" --kind "$kind" --build "$build_dir"
  done

  # Fail fast when the selected build's sanitizer runtime cannot be
  # resolved — the helper validates the recorded context and rejects a
  # stale or unresolvable lane before any instrumented consumer launches.
  run_with_native_runtime python -c 'import sys'
  run_with_native_runtime ctest --test-dir "$build_dir" --output-on-failure
  local extension_path
  extension_path="$(run_with_native_runtime python -c 'from native_loader import load_native; print(load_native().__file__)')"
  printf 'selected native extension: %s\n' "$extension_path"
  run_with_native_runtime cmake --build "$build_dir" --target check_native_contracts
}

if [[ "$profile" == native || "$profile" == full ]]; then
  export PYTHONPATH="$REPO_ROOT/tests/reference${PYTHONPATH:+:$PYTHONPATH}"
  selected_build="$(native_uv python -c 'import os, sys; sys.path.insert(0, "tests/reference"); from native_loader import selected_build; print(selected_build(os.environ))')"
  export NATIVE_TEST_BUILD_PATH="$selected_build"
  preflight_native "$selected_build"
  pytest_args+=(-p native_test_reporter)
fi

if [[ "$profile" == native || "$profile" == full ]]; then
  run_with_native_runtime python -m pytest "${pytest_args[@]}" "$@"
else
  exec uv run python -m pytest "${pytest_args[@]}" "$@"
fi
