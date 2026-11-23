#!/usr/bin/env bash
# tools/check_change.sh [--uncommitted] [base] — level 1: verify one change in minutes.
#   1. incremental build of the release preset;
#   2. every ordinary test (ctest --preset release: no slow|audit|package;
#      ~1 min with -j8 — cheaper to run all than to guess which);
#   3. cppcheck, frontends and clang-tidy on the changed translation units
#      plus every unit that includes a changed first-party header.
# Changed = committed since merge-base(HEAD, base) + uncommitted + untracked.
# --uncommitted narrows that to uncommitted + untracked files (no base).
# Default base: main. Pass a base only on a long-lived branch, and never
# HEAD: an explicit HEAD base is refused while this branch has commits not in
# main, because it would select none of them.
# Every run, failures included, appends one row to
# $BIKE_GATE_STATE_DIR/timings.tsv: the run_tests.sh columns plus tree=<id>
# (tools/lib/tree_id.sh, taken when the run starts).
# Warns on stderr when AGENTS.md exceeds 18000 bytes: agy truncates rule
# files at 24000 bytes, silently.
# Not a substitute for `tools/run_tests.sh native` before a merge (level 2).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
# shellcheck source=/dev/null
source "$REPO_ROOT/tools/lib/gate_state.sh"
# shellcheck source=/dev/null
source "$REPO_ROOT/tools/lib/tree_id.sh"

usage() {
  printf 'usage: check_change.sh [--uncommitted] [base]\n' >&2
  exit 2
}

uncommitted=0
base=main
explicit=0
for arg in "$@"; do
  case "$arg" in
    --uncommitted) uncommitted=1 ;;
    -h|--help)
      printf 'usage: check_change.sh [--uncommitted] [base]\n'
      exit 0
      ;;
    -*) usage ;;
    *)
      ((explicit == 0)) || usage
      base="$arg"
      explicit=1
      ;;
  esac
done
if ((uncommitted && explicit)); then
  printf 'check_change: --uncommitted takes no base\n' >&2
  exit 2
fi

gate_started=$SECONDS
BIKE_GATE_STATE_DIR="$(bike_gate_state_dir "$REPO_ROOT")"
mkdir -p "$BIKE_GATE_STATE_DIR"
build_dir="$REPO_ROOT/build/native-release"

agents_limit=18000
if [[ -f AGENTS.md ]]; then
  agents_bytes="$(wc -c < AGENTS.md | tr -d ' ')"
  if ((agents_bytes > agents_limit)); then
    printf 'check_change: warning: AGENTS.md is %s bytes (over %s); agy truncates rule files at 24000 bytes silently, so trim it\n' \
      "$agents_bytes" "$agents_limit" >&2
  fi
fi

# An explicit base naming the HEAD commit selects only uncommitted changes.
# That is right on main itself, and wrong on a branch with commits of its own.
if ((explicit)); then
  head_sha="$(git rev-parse --verify HEAD)"
  base_sha="$(git rev-parse --verify --quiet "$base^{commit}" || true)"
  if [[ -n "$base_sha" && "$base_sha" == "$head_sha" ]]; then
    main_base="$(git merge-base HEAD main)"
    if [[ "$main_base" != "$head_sha" ]]; then
      printf 'check_change: refusing base %s: it is the HEAD commit, which skips this branch'\''s commits not in main\n' "$base" >&2
      printf 'check_change: omit the base (default: main, merge-base), or pass --uncommitted for uncommitted and untracked files only\n' >&2
      exit 2
    fi
  fi
fi

tree="$(bike_tree_id "$REPO_ROOT")"
head_short="$(git rev-parse --short HEAD 2>/dev/null || printf 'nogit')"
cpu_count="${BIKE_CPU_COUNT:-$(getconf _NPROCESSORS_ONLN)}"
jobs="${BIKE_CHECK_JOBS:-$cpu_count}"
list="$(mktemp "${TMPDIR:-/tmp}/bike-change.XXXXXX")"
selected="-"

# finish — EXIT trap: drop the file list and journal this run, pass or fail.
finish() {
  local rc=$?
  [[ -n "$list" ]] && rm -f "$list"
  local status=passed total=$((SECONDS - gate_started))
  ((rc == 0)) || status=failed
  printf '%s\t%s\t%s\tchange\trelease\t-\t%s\t%s\t%s\tselected=%s\ttree=%s\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$REPO_ROOT" "$head_short" "$jobs" \
    "$status" "$total" "$selected" "$tree" \
    >> "$BIKE_GATE_STATE_DIR/timings.tsv"
  exit "$rc"
}
trap finish EXIT

if ((uncommitted)); then
  diff_base=HEAD
  scope="uncommitted changes"
else
  diff_base="$(git merge-base HEAD "$base")"
  scope="changes since $base"
fi
{
  git diff --name-only "$diff_base"
  git ls-files --others --exclude-standard
} | sort -u | while IFS= read -r path; do
  case "$path" in
    native/*.cpp|native/*.hpp|native/*.h|native/*.cc)
      [[ -f "$path" ]] && printf '%s/%s\n' "$REPO_ROOT" "$path"
      ;;
  esac
done > "$list"

selected="$(wc -l < "$list" | tr -d ' ')"
printf 'check_change: %s changed C++ file(s) (%s)\n' "$selected" "$scope"
if ((selected > 100)); then
  # On a long-lived branch merge-base(HEAD, main) can be far back; level 1
  # is meant for the change at hand.
  printf 'check_change: note: many files selected; pass a closer base (e.g. the commit before your task)\n'
fi

started=$SECONDS
cmake --build --preset release
ctest --preset release
if [[ ! -s "$list" ]]; then
  printf 'check_change: no C++ source or header changed (%s); analysis skipped\n' "$scope"
else
  checks="$build_dir/bin/bike-native-checks"
  [[ -n "${BIKE_NATIVE_CHECKS:-}" ]] && checks="$BIKE_NATIVE_CHECKS"
  for kind in cppcheck frontends tidy; do
    "$checks" --kind "$kind" --build "$build_dir" --only-sources "$list"
  done
fi
printf 'check_change: passed in %ss (level 1; run tools/run_tests.sh native once before merging)\n' \
  "$((SECONDS - started))"
