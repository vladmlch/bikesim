#!/usr/bin/env bash
# Shell-script lint sweep over tools/*.sh — same everything-is-an-error
# contract as the C++ sweeps. SHELLCHECK overrides the binary.
# (The word "shellcheck" must not start a comment line — it parses as a
# directive and trips SC1073.)
set -euo pipefail
TOOLS_DIR="$1"
SHELLCHECK="${SHELLCHECK:-shellcheck}"
find "$TOOLS_DIR" -maxdepth 1 -name '*.sh' -print0 | sort -z | \
  xargs -0 "$SHELLCHECK"
echo "shellcheck clean: $TOOLS_DIR"
