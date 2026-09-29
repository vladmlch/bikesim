#!/usr/bin/env bash
# Run from any directory. Needs the existing setuptools>=61 environment, not a network.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${1:-$ROOT/../wheelhouse}"
mkdir -p "$DEST"
cd "$ROOT"
# setuptools >=70 contains bdist_wheel, so a separate wheel/build package is not needed.
uv run --offline --no-project --python "${PYTHON:-python3}" python - "$DEST" <<'PY'
import sys
from setuptools.build_meta import build_wheel
print(build_wheel(sys.argv[1]))
PY
