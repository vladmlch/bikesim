#!/usr/bin/env bash
# tools/run_tests.sh <profile> [preset] — the qualification gate on the
# produced (interpreter-free) native graph only.
#
# Profiles (<build> is the preset's binaryDir: build/<preset>; release keeps
# build/native-release, Task 3.41):
#   quick  = cmake --workflow --preset <preset>
#            (the preset's test step excludes the slow|audit|package labels)
#   native = cmake --preset <preset> && cmake --build --preset <preset>,
#            then tools/run_native_checks.sh <build> (every produced sweep
#            plus the contracts kind), then
#            ctest --test-dir <build> -LE 'slow|audit|package'
#   full   = native, then the explicit label gates
#            ctest -L slow / -L audit / -L package, each --no-tests=error so
#            an absent registration fails instead of passing silently.
#
# `native asan` runs the same stages on build/asan: sanitizer runtimes come
# from the preset's own link flags now, not a preload. The [preset] argument
# replaces the old NATIVE_TEST_BUILD_PATH / NATIVE_TEST_BUILD_DIR=asan|rtsan|
# coverage build selection.
#
# Successor map for the retired interpreter-driven gate stages:
#   extension configure/build   -> cmake --preset / --build --preset on the
#                                  root graph (there is no bindings extension
#                                  to select or import);
#   check_<name> sweep targets  -> tools/run_native_checks.sh <build>, which
#                                  drives the produced bike-native-checks
#                                  binary in fixed contract order;
#   native_runtime preload      -> preset sanitizer link flags (Task 3.42);
#   native/golden test legs     -> the `reference` CTest label set;
#   NATIVE_TEST_PROVENANCE_PATH / the native_test_reporter plugin
#                               -> CTest's <build>/Testing/ tree plus
#                                  <build>/Testing/junit.xml; the Task 3.45
#                                  CI lanes upload both.
#
# NATIVE_TEST_BUILD_PATH is still exported to the selected build directory:
# consumers honoring the selection convention see a consistent environment
# (the contracts kind rejects a mismatched value).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: bash tools/run_tests.sh [quick|native|full] [preset]

  quick   Configure, build and test the selected preset in one workflow run;
          its test step excludes the slow, audit and package label gates.
  native  Configure and build the selected preset, run every produced native
          sweep plus the structural contracts check, then CTest without the
          slow, audit and package labels.
  full    The native profile, then the explicit slow, audit and package
          label gates (--no-tests=error, so an absent registration fails).
  -h, --help  Show this help.

preset selects a configure preset from CMakePresets.json (default: release).
Its binaryDir is the build directory: build/<preset>; release keeps
build/native-release. `native asan` runs the same stages on build/asan —
sanitizer runtimes come from the preset's own link flags.
EOF
}

profile="${1:-quick}"
preset="${2:-release}"
case "$profile" in
  quick|native|full) ;;
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
case "$preset" in
  release|debug|gui|asan|rtsan|coverage) ;;
  *)
    printf 'Unknown preset: %s\n' "$preset" >&2
    usage >&2
    exit 2
    ;;
esac
if (($# > 2)); then
  printf 'Unexpected extra arguments: the gate takes <profile> [preset] only.\n' >&2
  usage >&2
  exit 2
fi

build_dir="$REPO_ROOT/build/$preset"
if [[ "$preset" == release ]]; then
  build_dir="$REPO_ROOT/build/native-release"
fi

# Darwin allocator debugging wraps only the ordinary `ctest -LE
# 'slow|audit|package'` invocation — wherever a profile runs it — in a
# subshell. Never exported on Linux, and never around `ctest -L slow`,
# which contains realtime measurements (Task 3.10).
run_ordinary_leg() {
  if [[ "$(uname -s)" == Darwin ]]; then
    (
      export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
      "$@"
    )
  else
    "$@"
  fi
}

run_native_gate() {
  cmake --preset "$preset"
  cmake --build --preset "$preset"
  printf 'selected native build: %s\n' "$build_dir"
  export NATIVE_TEST_BUILD_PATH="$build_dir"
  bash "$REPO_ROOT/tools/run_native_checks.sh" "$build_dir"
  run_ordinary_leg ctest --test-dir "$build_dir" -LE 'slow|audit|package' \
    -j8 --no-tests=error --output-on-failure \
    --output-junit "$build_dir/Testing/junit.xml"
}

case "$profile" in
  quick)
    # The workflow's test step is the ordinary `ctest -LE 'slow|audit|package'`
    # leg, so the same Darwin allocator debugging wraps the whole invocation;
    # the configure/build steps ignore the variables.
    run_ordinary_leg cmake --workflow --preset "$preset"
    ;;
  native)
    run_native_gate
    ;;
  full)
    run_native_gate
    for label in slow audit package; do
      ctest --test-dir "$build_dir" -L "$label" --no-tests=error \
        --output-on-failure
    done
    ;;
esac
