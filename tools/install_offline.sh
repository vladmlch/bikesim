#!/usr/bin/env bash
# Thin entry point for tools/install_offline.py.
#
# The installer itself performs interpreter selection (manifest target,
# --python, then PATH) and never relies on an inherited activated
# environment, so this wrapper only needs an interpreter capable of running
# the stdlib-only preflight dispatcher. PYTHON overrides the dispatch
# interpreter; the bundle's own target interpreter is selected inside.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PYTHON:-python3}" "$ROOT/tools/install_offline.py" "$@"
