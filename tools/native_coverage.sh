#!/usr/bin/env bash
# Build and measure one native test run without mixing its LLVM profiles with
# earlier runs. Logs and profile artifacts stay under the run directory.
#
# Toolchain selection is delegated to tools/native_toolchain.py's
# coverage-tools command — explicit LLVM_PROFDATA/LLVM_COV environment
# overrides win, then the configured compiler's siblings, qualified PATH
# candidates, and (Darwin only) the xcrun fallback; the selected pair must
# pass the real compile/run/merge/export format probe. The same qualification
# runs inside the NATIVE_COVERAGE=ON configure below and again inside
# tools/native_coverage.py — one resolution contract, three call sites.
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

cd "$REPO_ROOT"
run_logged configure uv run --frozen --group native cmake -S "$REPO_ROOT/native" -B "$BUILD_DIR" -DNATIVE_COVERAGE=ON

# Resolve + qualify the coverage toolchain against the compiler the build
# actually selected (recorded in native_check_context.json). The resolution
# record is retained as run evidence; the pinned paths are forwarded to the
# finalizer so the report is produced by exactly these executables.
RESOLUTION_JSON="$RUN_DIR/coverage_tool_resolution.json"
run_logged coverage-tools uv run --frozen --group native python \
  "$REPO_ROOT/tools/native_toolchain.py" coverage-tools \
  --context "$BUILD_DIR" --json "$RESOLUTION_JSON"

LLVM_PROFDATA="$(uv run --frozen --group native python -c \
  'import json, sys; print(json.load(open(sys.argv[1]))["llvm_profdata"]["path"])' \
  "$RESOLUTION_JSON")"
LLVM_COV="$(uv run --frozen --group native python -c \
  'import json, sys; print(json.load(open(sys.argv[1]))["llvm_cov"]["path"])' \
  "$RESOLUTION_JSON")"
export LLVM_PROFDATA LLVM_COV

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
