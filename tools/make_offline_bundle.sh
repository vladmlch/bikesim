#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# No project synchronization, build hook, resolver, or Python download. All
# archive bytes must already exist in the explicitly supplied input directories.
exec uv run --offline --no-project --no-python-downloads \
  python "$SCRIPT_DIR/offline_bundle.py" create "$@"
