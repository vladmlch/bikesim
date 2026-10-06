#!/usr/bin/env bash
# Build and measure one native test run without mixing its LLVM profiles with
# earlier runs. Logs and profile artifacts stay under the run directory.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
BUILD_DIR="${1:-$REPO_ROOT/native/build/coverage}"
if (($# > 1)); then
  printf 'usage: tools/native_coverage.sh [build_dir]\n' >&2
  exit 2
fi
mkdir -p "$BUILD_DIR"
BUILD_DIR="$(cd "$BUILD_DIR" && pwd -P)"
RUNS_DIR="$BUILD_DIR/native_coverage_runs"
mkdir -p "$RUNS_DIR"
RUN_DIR="$(mktemp -d "$RUNS_DIR/run.XXXXXX")"
PROFILE_DIR="$RUN_DIR/profiles"
LOG_DIR="$RUN_DIR/logs"
mkdir -p "$PROFILE_DIR" "$LOG_DIR"

run_logged() {
  local name="$1"
  shift
  local log_path="$LOG_DIR/$name.log"
  local status=0
  "$@" >"$log_path" 2>&1 || status=$?
  cat "$log_path"
  if ((status != 0)); then
    printf '%s failed with exit %s; log retained at %s\n' "$name" "$status" "$log_path" >&2
    return "$status"
  fi
}

resolve_tool() {
  local variable_name="$1"
  local tool_name="$2"
  local value="${!variable_name:-}"
  if [[ -z "$value" ]]; then
    if command -v xcrun >/dev/null 2>&1; then
      local resolve_log="$LOG_DIR/xcrun-$tool_name.log"
      if ! value="$(xcrun --find "$tool_name" 2>"$resolve_log")"; then
        cat "$resolve_log" >&2
        printf 'xcrun could not resolve %s; log retained at %s\n' "$tool_name" "$resolve_log" >&2
        return 1
      fi
    else
      value="$(command -v "$tool_name" || true)"
    fi
  fi
  if [[ -z "$value" || ! -x "$value" ]]; then
    printf '%s must resolve to an executable file (set %s)\n' "$tool_name" "$variable_name" >&2
    return 1
  fi
  printf '%s\n' "$value"
}

LLVM_PROFDATA="$(resolve_tool LLVM_PROFDATA llvm-profdata)"
LLVM_COV="$(resolve_tool LLVM_COV llvm-cov)"

cd "$REPO_ROOT"
run_logged configure uv run --frozen --group native cmake -S "$REPO_ROOT/native" -B "$BUILD_DIR" -DNATIVE_COVERAGE=ON

unset NATIVE_TEST_BUILD_DIR
export NATIVE_TEST_BUILD_PATH="$BUILD_DIR"
export NATIVE_TEST_PROVENANCE_PATH="$RUN_DIR/test_native_provenance.json"
export LLVM_PROFILE_FILE="$PROFILE_DIR/%p.profraw"
run_logged native-tests bash "$REPO_ROOT/tools/run_tests.sh" native
run_logged coverage-report uv run --frozen --group native python "$REPO_ROOT/tools/native_coverage.py" \
  --build "$BUILD_DIR" \
  --run-dir "$RUN_DIR" \
  --llvm-profdata "$LLVM_PROFDATA" \
  --llvm-cov "$LLVM_COV"
