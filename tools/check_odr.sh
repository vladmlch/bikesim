#!/usr/bin/env bash
# Keep the CMake target entry point; the controller owns selection and health.
# T56: launches the produced orchestrator — no interpreter in the check path.
set -euo pipefail
if (($# < 4)); then
  echo "usage: $0 BUILD_DIR SOURCE_DIR MUJOCO_INCLUDE_DIR PYTHON_EXECUTABLE" >&2
  exit 2
fi
BUILD_DIR="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/native_common.sh"
exec "$(bike_checks_binary "$BUILD_DIR")" --kind odr --build "$BUILD_DIR"
