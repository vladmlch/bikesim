#!/usr/bin/env bash
# tools/check_change.sh [base] — level 1: verify one change in minutes.
#   1. incremental build of the release preset;
#   2. every ordinary test (ctest --preset release: no slow|audit|package;
#      ~1 min with -j8 — cheaper to run all than to guess which);
#   3. cppcheck, frontends and clang-tidy on the changed translation units
#      plus every unit that includes a changed first-party header.
# Changed = committed since merge-base(HEAD, base) + uncommitted + untracked.
# Default base: main. Not a substitute for `tools/run_tests.sh native`
# before a merge (level 2).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
base="${1:-main}"
build_dir="$REPO_ROOT/build/native-release"

merge_base="$(git merge-base HEAD "$base")"
list="$(mktemp "${TMPDIR:-/tmp}/bike-change.XXXXXX")"
trap 'rm -f "$list"' EXIT
{
  git diff --name-only "$merge_base"
  git ls-files --others --exclude-standard
} | sort -u | while IFS= read -r path; do
  case "$path" in
    native/*.cpp|native/*.hpp|native/*.h|native/*.cc)
      [[ -f "$path" ]] && printf '%s/%s\n' "$REPO_ROOT" "$path"
      ;;
  esac
done > "$list"

selected="$(wc -l < "$list" | tr -d ' ')"
printf 'check_change: %s changed C++ file(s) since %s\n' "$selected" "$base"
if ((selected > 100)); then
  # On a long-lived branch merge-base(HEAD, main) can be far back; level 1
  # is meant for the change at hand.
  printf 'check_change: note: many files selected; pass a closer base (e.g. the commit before your task)\n'
fi

started=$SECONDS
cmake --build --preset release
ctest --preset release
if [[ ! -s "$list" ]]; then
  printf 'check_change: no C++ source or header changed since %s; analysis skipped\n' "$base"
else
  checks="$build_dir/bin/bike-native-checks"
  [[ -n "${BIKE_NATIVE_CHECKS:-}" ]] && checks="$BIKE_NATIVE_CHECKS"
  for kind in cppcheck frontends tidy; do
    "$checks" --kind "$kind" --build "$build_dir" --only-sources "$list"
  done
fi
printf 'check_change: passed in %ss (level 1; run tools/run_tests.sh native once before merging)\n' \
  "$((SECONDS - started))"
