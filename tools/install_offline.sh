#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PYTHON:-python3}"
if command -v uv >/dev/null 2>&1; then
    exec uv run --offline --no-project --python "$PY" python "$ROOT/tools/install_offline.py" "$@"
fi
exec "$PY" "$ROOT/tools/install_offline.py" "$@"
