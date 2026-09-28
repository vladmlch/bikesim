#!/usr/bin/env bash
# Offline installer for bike-sim (Linux x86_64, CPython 3.12) — SLIM variant.
# Reuses system-installed: numpy>=1.24, scipy>=1.18, matplotlib>=3.7, pytest>=7.
# Usage: ./install.sh [install_dir]
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

echo "Checking system packages ..."
$PY - <<'PYEOF'
import importlib.metadata as md, re, sys
reqs = {"numpy": (1, 24, 0), "scipy": (1, 18, 0),
        "matplotlib": (3, 7, 0), "pytest": (7, 0, 0)}
bad = []
for name, minv in reqs.items():
    try:
        v = tuple(int(x) for x in re.findall(r"\d+", md.version(name))[:3])
    except md.PackageNotFoundError:
        v = ()
    if v < minv:
        bad.append(f"{name}>={'.'.join(map(str, minv))} (found: {v or 'not installed'})")
if bad:
    sys.exit("ERROR: unsuitable system packages:\n  " + "\n  ".join(bad) +
             "\nInstall them first, or use the full (non-slim) bundle.")
PYEOF

echo "Creating virtual environment (with system site-packages) in $VENV_DIR ..."
$PY -m venv --system-site-packages "$VENV_DIR" || {
    echo "ERROR: venv creation failed. On Debian/Ubuntu install the 'python3.12-venv' package." >&2
    exit 1
}

PIP="$VENV_DIR/bin/pip"
if [ ! -x "$PIP" ]; then
    echo "ERROR: venv has no pip (ensurepip missing). Install 'python3.12-venv'." >&2
    exit 1
fi

echo "Installing missing dependencies from local wheelhouse (no network) ..."
"$PIP" install --no-index --find-links "$BUNDLE_DIR/wheelhouse" \
    -r "$BUNDLE_DIR/requirements.txt"
# deps enabled: pip verifies system numpy/scipy/matplotlib/pytest satisfy constraints
"$PIP" install --no-index --find-links "$BUNDLE_DIR/wheelhouse" \
    "$BUNDLE_DIR"/wheelhouse/bike_sim-0.2.0-py3-none-any.whl

echo
echo "Done. Activate with:  source $VENV_DIR/bin/activate"
echo "Entry points: bike-sim, bike-playground, bike-export, bike-ride"
echo "Project sources are in: $BUNDLE_DIR/project"
