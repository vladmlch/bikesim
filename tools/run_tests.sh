#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: bash tools/run_tests.sh [quick|native|full] [pytest arguments...]

  quick   Run tests without the slow marker (default).
  native  Require bike_native, then run native and golden episode tests.
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
case "$profile" in
  quick)
    pytest_args+=(-m 'not slow')
    ;;
  native)
    export PYTHONPATH="$REPO_ROOT/native/build${PYTHONPATH:+:$PYTHONPATH}"
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

exec uv run python -m pytest "${pytest_args[@]}" "$@"
