#!/usr/bin/env bash
# Shell-script lint sweep over tools/*.sh and tools/*/*.sh — same everything-is-an-error
# contract as the C++ sweeps. SHELLCHECK overrides the binary.
# (The word "shellcheck" must not start a comment line — it parses as a
# directive and trips SC1073.)
set -euo pipefail
[[ $# -eq 1 ]] || { printf 'usage: check_scripts.sh TOOLS_DIR\n' >&2; exit 2; }
TOOLS_DIR="$(cd -- "$1" && pwd -P)"
SHELLCHECK="${SHELLCHECK:-shellcheck}"
# Complete selection before invoking the tool: traversal and sorting failures
# must not lint a partial set. An empty set is never a successful sweep.
selection="$(mktemp "${TMPDIR:-/tmp}/bike-check-scripts.XXXXXXXX")"
trap 'rm -f -- "$selection"' EXIT
find -L "$TOOLS_DIR" -maxdepth 2 -type f -name '*.sh' -print0 > "$selection"
LC_ALL=C sort -z -o "$selection" "$selection"
[[ -s "$selection" ]] || { printf 'no shell scripts selected: %s\n' "$TOOLS_DIR" >&2; exit 1; }
xargs -0 "$SHELLCHECK" < "$selection"
echo "shellcheck clean: $TOOLS_DIR"
