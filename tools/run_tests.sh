#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: bash tools/run_tests.sh [quick|native|full] [pytest arguments...]

  quick   Run tests without the slow marker (default).
  native  Build bike_native, run the compiler/static-analysis sweeps
          (GCC frontend + ODR, clang-tidy, CSA, cppcheck, shellcheck),
          then native/golden tests.
  full    Run all tests, including slow physics and realtime episodes.
  -h, --help  Show this help.

All profiles use -q --durations=10 and forward additional pytest arguments.
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
    ;;
  native)
    # The full native verification chain: build, then every linter sweep,
    # then tests — so "native" cannot silently pass on a stale .so.
    cmake --build "$REPO_ROOT/native/build"
    cmake --build "$REPO_ROOT/native/build" --target check_frontends
    cmake --build "$REPO_ROOT/native/build" --target check_tidy
    cmake --build "$REPO_ROOT/native/build" --target check_analyzer
    cmake --build "$REPO_ROOT/native/build" --target check_cppcheck
    cmake --build "$REPO_ROOT/native/build" --target check_odr
    cmake --build "$REPO_ROOT/native/build" --target check_scripts
    export PYTHONPATH="$REPO_ROOT/native/build${PYTHONPATH:+:$PYTHONPATH}"
    export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
    uv run python -c 'import bike_native; print(bike_native.__file__)'
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

if [[ "${NATIVE_TEST_BUILD_DIR:-}" == asan ]]; then
  # PYTHONMALLOC=malloc routes CPython allocations through ASan-visible
  # malloc — pymalloc would mask them from the interceptor. detect_leaks=0
  # matches the documented recipe; stack-use-after-return is the extra.
  export PYTHONMALLOC=malloc
  export ASAN_OPTIONS="${ASAN_OPTIONS:+$ASAN_OPTIONS:}detect_leaks=0:detect_stack_use_after_return=1"
fi

exec uv run python -m pytest "${pytest_args[@]}" "$@"
