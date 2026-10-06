#!/usr/bin/env bash
# Keep the CMake target entry point; the controller owns selection and health.
set -euo pipefail
if (($# < 3)); then
  echo "usage: $0 BUILD_DIR SOURCE_DIR MUJOCO_INCLUDE_DIR" >&2
  exit 2
fi
BUILD_DIR="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec uv run --frozen --group native python "$SCRIPT_DIR/native_checks.py" --kind frontends --build "$BUILD_DIR"
