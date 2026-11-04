#!/usr/bin/env bash
# tools/run_tests.sh <profile> [preset] — the qualification gate on the
# produced (interpreter-free) native graph only.
#
# Profiles (<build> is the preset's binaryDir: build/<preset>; release keeps
# build/native-release, Task 3.41):
#   quick  = cmake --workflow --preset <preset>
#            (the preset's test step excludes the slow|package labels)
#   native = cmake --preset <preset> && cmake --build --preset <preset>,
#            then the ordinary tests
#            ctest --test-dir <build> -LE 'slow|package' (they run first —
#            the cheapest failure signal), then
#            tools/run_native_checks.sh <build> (every produced sweep
#            plus the contracts kind)
#   full   = native, then the explicit label gates
#            ctest -L slow / -L package, each --no-tests=error so
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
#
# Stages after configure+build, in order: `ctest`, then the sweep kinds of
# tools/run_native_checks.sh, then — full only — `slow` and `package`.
# `--from STAGE` (native/full only) is an intermediate re-check after a
# fix: configure and build still run, every earlier stage is skipped and
# the run is marked PARTIAL — never a final verification.
#
# Every run ends with a per-stage seconds/exit table and appends one line
# to $BIKE_GATE_STATE_DIR/timings.tsv (default: <git common dir>/bike-gate,
# or <repo>/build/bike-gate outside git). BIKE_CPU_COUNT overrides the
# detected CPU count used for --parallel and recorded as the journal's
# jobs field.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: bash tools/run_tests.sh [quick|native|full] [preset] [--from STAGE]

  quick   Configure, build and test the selected preset in one workflow run;
          its test step excludes the slow and package label gates.
  native  Configure and build the selected preset, run CTest without the
          slow and package labels first, then every produced native sweep
          plus the structural contracts check.
  full    The native profile, then the explicit slow and package
          label gates (--no-tests=error, so an absent registration fails).
  -h, --help  Show this help.
  --from STAGE  native/full only: an intermediate re-check after a fix.
                Configure and build always run; earlier stages are skipped
                and the run is marked PARTIAL. Stages: ctest scripts cppcheck
                context diagnostic-controls contracts frontends headers odr
                tidy, plus slow package for full.

preset selects a configure preset from CMakePresets.json (default: release).
Its binaryDir is the build directory: build/<preset>; release keeps
build/native-release. `native asan` runs the same stages on build/asan —
sanitizer runtimes come from the preset's own link flags.
EOF
}

profile="${1:-quick}"
preset="release"
from=""
if (($# > 0)); then shift; fi
if (($# > 0)) && [[ "$1" != --* ]]; then
  preset="$1"
  shift
fi
while (($# > 0)); do
  case "$1" in
    --from)
      (($# >= 2)) || { printf -- '--from needs a stage\n' >&2; usage >&2; exit 2; }
      from="$2"
      shift 2
      ;;
    *)
      printf 'Unexpected argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done
case "$profile" in
  quick|native|full) ;;
  -h|--help) usage; exit 0 ;;
  *) printf 'Unknown test profile: %s\n' "$profile" >&2; usage >&2; exit 2 ;;
esac
case "$preset" in
  release|debug|gui|asan|rtsan|coverage) ;;
  *) printf 'Unknown preset: %s\n' "$preset" >&2; usage >&2; exit 2 ;;
esac

# Stages after configure+build, in execution order. The sweep kinds are
# owned by tools/run_native_checks.sh; keep this list in the same order.
sweep_kinds=(scripts cppcheck context diagnostic-controls contracts
  frontends headers odr tidy)
stages=(ctest "${sweep_kinds[@]}")
[[ "$profile" == full ]] && stages+=(slow package)
if [[ -n "$from" ]]; then
  if [[ "$profile" == quick ]]; then
    printf -- '--from applies to native and full only\n' >&2
    exit 2
  fi
  known=0
  for stage_name in "${stages[@]}"; do
    [[ "$stage_name" == "$from" ]] && known=1
  done
  if ((!known)); then
    printf 'Unknown --from stage for %s: %s (choose from: %s)\n' \
      "$profile" "$from" "${stages[*]}" >&2
    exit 2
  fi
fi
# stage_index <name> -> position in stages (or 999 when absent)
stage_index() {
  local i
  for i in "${!stages[@]}"; do
    [[ "${stages[$i]}" == "$1" ]] && { printf '%s\n' "$i"; return; }
  done
  printf '999\n'
}
from_index=0
[[ -n "$from" ]] && from_index="$(stage_index "$from")"
wanted() { (( $(stage_index "$1") >= from_index )); }

build_dir="$REPO_ROOT/build/$preset"
if [[ "$preset" == release ]]; then
  build_dir="$REPO_ROOT/build/native-release"
fi

# Shared gate state (journal; active-gate registry in later tasks): the git
# common dir is shared by every worktree of this repository.
if [[ -z "${BIKE_GATE_STATE_DIR:-}" ]]; then
  if common_dir="$(git -C "$REPO_ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)"; then
    BIKE_GATE_STATE_DIR="$common_dir/bike-gate"
  else
    BIKE_GATE_STATE_DIR="$REPO_ROOT/build/bike-gate"
  fi
fi
mkdir -p "$BIKE_GATE_STATE_DIR"
cpu_count="${BIKE_CPU_COUNT:-$(getconf _NPROCESSORS_ONLN)}"

# Active-gate registry: one file per running native/full gate, named by its
# pid. No queue — each gate takes an equal share of the cores, so parallel
# gates slow each other down evenly instead of oversubscribing the machine.
# Entries of dead processes are removed; live ones are reported.
# The share is computed once, when the gate starts: a neighbour finishing
# early leaves the rest of this run on its share. A deliberate
# simplification.
active_dir="$BIKE_GATE_STATE_DIR/active"
mkdir -p "$active_dir"
others=0
for entry in "$active_dir"/*; do
  [[ -e "$entry" ]] || continue
  pid="${entry##*/}"
  if [[ "$pid" == "$$" ]]; then continue; fi
  if kill -0 "$pid" 2>/dev/null; then
    others=$((others + 1))
    IFS=$'\t' read -r _ started where kind_of < "$entry" || true
    printf 'note: another %s gate is running: pid %s in %s (since %s)\n' \
      "${kind_of:-native}" "$pid" "${where:-?}" \
      "$(date -r "${started:-0}" '+%H:%M' 2>/dev/null || printf '?')" >&2
  else
    rm -f "$entry"
  fi
done
if [[ "$profile" != quick ]]; then
  printf '%s\t%s\t%s\t%s\n' "$$" "$(date +%s)" "$REPO_ROOT" "$profile" \
    > "$active_dir/$$"
fi
if ((others > 0)); then
  printf 'note: %s other native gate(s) running; using %s of %s cores\n' \
    "$others" "$((cpu_count / (others + 1) > 0 ? cpu_count / (others + 1) : 1))" \
    "$cpu_count" >&2
fi
jobs=$((cpu_count / (others + 1)))
((jobs >= 1)) || jobs=1
if [[ -n "${BIKE_CHECK_JOBS:-}" ]]; then
  jobs="$BIKE_CHECK_JOBS"
fi
export BIKE_CHECK_JOBS="$jobs"

timings_file="$(mktemp "${TMPDIR:-/tmp}/bike-gate-timings.XXXXXX")"
export BIKE_GATE_TIMINGS_FILE="$timings_file"
gate_started=$SECONDS

# stage <name> <command...> — run one stage and record its duration.
stage() {
  local name="$1" started=$SECONDS rc=0
  shift
  "$@" || rc=$?
  printf '%s\t%s\t%s\n' "$name" "$((SECONDS - started))" "$rc" >> "$timings_file"
  return "$rc"
}

finish() {
  local rc=$?
  rm -f "$active_dir/$$"
  local total=$((SECONDS - gate_started)) status=passed summary=""
  ((rc == 0)) || status=failed
  printf '\n%-22s %8s %6s\n' stage seconds exit
  while IFS=$'\t' read -r name seconds stage_rc; do
    printf '%-22s %8s %6s\n' "$name" "$seconds" "$stage_rc"
    summary+="${summary:+,}${name}=${seconds}"
  done < "$timings_file"
  printf '%-22s %8s %6s\n' total "$total" "$rc"
  if [[ -n "$from" ]]; then
    printf 'PARTIAL run (--from %s): not a final verification\n' "$from"
  fi
  local head
  head="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || printf 'nogit')"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$REPO_ROOT" "$head" "$profile" \
    "$preset" "${from:--}" "$jobs" "$status" "$total" "$summary" \
    >> "$BIKE_GATE_STATE_DIR/timings.tsv"
  rm -f "$timings_file"
  exit "$rc"
}
trap finish EXIT

# Darwin allocator debugging wraps only the ordinary `ctest -LE
# 'slow|package'` invocation — wherever a profile runs it — in a
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
  stage configure cmake --preset "$preset"
  stage build cmake --build --preset "$preset" --parallel "$jobs"
  printf 'selected native build: %s\n' "$build_dir"
  export NATIVE_TEST_BUILD_PATH="$build_dir"
  # The ordinary tests run first: ~1 min, and they catch what would
  # otherwise surface only after every sweep (2026-11-03: a 2h21m gate
  # failed on two tests that fail in under 5 s).
  if wanted ctest; then
    local ctest_jobs=$((jobs < 8 ? jobs : 8))
    stage ctest run_ordinary_leg ctest --test-dir "$build_dir" -LE 'slow|package' \
      -j"$ctest_jobs" --no-tests=error --output-on-failure \
      --output-junit "$build_dir/Testing/junit.xml"
  fi
  local first_kind=""
  for kind in "${sweep_kinds[@]}"; do
    if wanted "$kind"; then first_kind="$kind"; break; fi
  done
  if [[ -n "$first_kind" ]]; then
    local from_args=()
    [[ "$first_kind" != "${sweep_kinds[0]}" ]] && from_args=(--from "$first_kind")
    bash "$REPO_ROOT/tools/run_native_checks.sh" "${from_args[@]}" "$build_dir"
  fi
}

case "$profile" in
  quick)
    # The workflow's test step is the ordinary `ctest -LE 'slow|package'`
    # leg, so the same Darwin allocator debugging wraps the whole invocation;
    # the configure/build steps ignore the variables.
    stage workflow run_ordinary_leg cmake --workflow --preset "$preset"
    ;;
  native)
    run_native_gate
    ;;
  full)
    run_native_gate
    for label in slow package; do
      if wanted "$label"; then
        stage "$label" ctest --test-dir "$build_dir" -L "$label" --no-tests=error \
          --output-on-failure
      fi
    done
    ;;
esac
