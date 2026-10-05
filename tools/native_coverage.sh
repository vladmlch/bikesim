#!/usr/bin/env bash
# llvm-cov source coverage over the native TUs: builds the instrumented
# extension (configure the dir with -DNATIVE_COVERAGE=ON), runs the native
# oracle tests against it, merges profraw, prints the summary report.
#
#   cmake -S native -B native/build/coverage -DNATIVE_COVERAGE=ON
#   tools/native_coverage.sh [build_dir]
#
# The build dir name must be 'coverage' — NATIVE_TEST_BUILD_DIR resolves
# native/build/<name> and asserts the imported .so lives there.
# Xcode's llvm-cov/llvm-profdata deliberately: they match the producing
# Apple clang's profile format; do not substitute the brew llvm pair.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-$REPO_ROOT/native/build/coverage}"
PROF_DIR="$(mktemp -d -t native_cov)"; trap 'rm -rf "$PROF_DIR"' EXIT

cmake --build "$BUILD_DIR" -j"$(sysctl -n hw.ncpu)"
cd "$REPO_ROOT"
# %p: a profraw per process, so concurrent pytest workers cannot collide.
NATIVE_TEST_BUILD_DIR=coverage \
LLVM_PROFILE_FILE="$PROF_DIR/%p.profraw" \
  uv run python -m pytest tests/reference/test_native_*.py \
    tests/reference/test_golden_episode*.py -q --durations=5

xcrun llvm-profdata merge -sparse "$PROF_DIR"/*.profraw \
  -o "$BUILD_DIR/coverage.profdata"
SO=$(echo "$BUILD_DIR"/bike_native.*.so)
xcrun llvm-cov report "$SO" -instr-profile="$BUILD_DIR/coverage.profdata"
echo
echo "per-line detail: xcrun llvm-cov show $SO -instr-profile=$BUILD_DIR/coverage.profdata"
