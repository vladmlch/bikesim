#!/usr/bin/env bash
# Offline installer for bike-sim (Linux x86_64, CPython 3.12).
# Usage: ./install.sh [install_dir]
#   install_dir — where to create the venv (default: ./.venv)
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="${1:-$BUNDLE_DIR/.venv}"

PY=python3
if ! $PY -c 'import sys; assert sys.version_info[:2] == (3, 12)' 2>/dev/null; then
    if python3.12 -c 'pass' 2>/dev/null; then PY=python3.12; else
        echo "ERROR: Python 3.12 is required (found: $($PY --version 2>&1))." >&2
        exit 1
    fi
fi

echo "Creating virtual environment in $VENV_DIR ..."
$PY -m venv "$VENV_DIR" || {
    echo "ERROR: venv creation failed. On Debian/Ubuntu install the 'python3.12-venv' package." >&2
    exit 1
}

PIP="$VENV_DIR/bin/pip"
if [ ! -x "$PIP" ]; then
    echo "ERROR: venv has no pip (ensurepip missing). Install 'python3.12-venv'." >&2
    exit 1
fi

echo "Installing dependencies from local wheelhouse (no network) ..."
"$PIP" install --no-index --find-links "$BUNDLE_DIR/wheelhouse" \
    -r "$BUNDLE_DIR/requirements.txt"
"$PIP" install --no-index --no-deps \
    "$BUNDLE_DIR"/wheelhouse/bike_sim-0.2.0-py3-none-any.whl

echo
echo "Done. Activate with:  source $VENV_DIR/bin/activate"
echo "Entry points: bike-sim, bike-playground, bike-export, bike-ride"
echo "Project sources are in: $BUNDLE_DIR/project"
