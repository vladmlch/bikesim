#!/usr/bin/env bash
# tools/agy/check-reports.sh — completeness check for the reports of fanned-out runs.
#
#   bash tools/agy/check-reports.sh <dir> [--expect N]
#
# Every *.md directly in <dir> must end with an end marker: its last non-empty
# line is exactly
#   <!-- END REPORT shard=<id> items=<n> -->
# with <id> matching [A-Za-z0-9_.-]+ and equal to the file's basename without
# .md, and <n> an integer. Blank lines after the marker are allowed. With
# --expect N the directory must hold exactly N reports.
#
# Prints one line per failing shard and exits 1; exits 0 silently when every
# report passes. Exits 2 on a usage error. Nothing is written or executed. Without
# --expect an empty directory passes, so pass --expect in a fan-out.
#
# Why the marker is the last line: a report cut off mid-word has no marker, and
# the cut is otherwise invisible until someone reads the file to the end.
# Bash 3.2 compatible: macOS /bin/bash is the reference.
set -euo pipefail

usage() {
  printf 'usage: check-reports.sh <dir> [--expect N]\n' >&2
  exit 2
}

dir=""
expect=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --expect)
      [[ $# -ge 2 && $2 =~ ^[0-9]+$ ]] || usage
      expect=$((10#$2))
      shift 2
      ;;
    -*) usage ;;
    *)
      [[ -z $dir ]] || usage
      dir=$1
      shift
      ;;
  esac
done
[[ -n $dir ]] || usage
[[ -d $dir ]] || { printf 'check-reports: not a directory: %s\n' "$dir" >&2; exit 2; }

marker_re='^<!-- END REPORT shard=([A-Za-z0-9_.-]+) items=[0-9]+ -->$'
failed=0
found=0
for report in "$dir"/*.md; do
  [[ -f $report ]] || continue
  found=$((found + 1))
  name=${report##*/}
  shard=${name%.md}
  # awk NF skips lines that hold only whitespace.
  if ! last=$(awk 'NF { line = $0 } END { print line }' "$report"); then
    printf '%s: unreadable\n' "$name"
    failed=1
    continue
  fi
  if [[ ! $last =~ $marker_re ]]; then
    printf '%s: no end marker\n' "$name"
    failed=1
  elif [[ ${BASH_REMATCH[1]} != "$shard" ]]; then
    printf '%s: marker shard=%s does not match file\n' "$name" "${BASH_REMATCH[1]}"
    failed=1
  fi
done

if [[ -n $expect && $found -ne $expect ]]; then
  printf 'expected %s reports, found %s\n' "$expect" "$found"
  failed=1
fi
exit "$failed"
