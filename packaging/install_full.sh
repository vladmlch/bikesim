#!/usr/bin/env bash
# Full offline installer for a bike-sim schema-1 source bundle, or the
# documented legacy manifest-less Python-only bundle.
#
# Usage: bash project/packaging/install_full.sh [install_dir]
#   install_dir — where the venv is created (default: <bundle>/.venv)
#
# Everything resolves from validated local paths: the manifest, wheel hashes,
# interpreter, uv and wheelhouse are checked by tools/install_offline.py
# before any environment is created. A full bundle carries its entire
# dependency closure; no system scientific stack is required or reused.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
bundle_dir="$(cd -- "$project_dir/.." && pwd)"

if [[ ! -f "$bundle_dir/manifest.json" ]] && \
   { [[ ! -f "$bundle_dir/requirements.txt" ]] || [[ ! -d "$bundle_dir/wheelhouse" ]]; }; then
    echo "ERROR: $script_dir is not inside an extracted bike-sim offline bundle." >&2
    echo "Expected $bundle_dir to contain manifest.json (schema 1), or" >&2
    echo "requirements.txt + wheelhouse/ (legacy manifest-less bundle)." >&2
    exit 1
fi

exec "${PYTHON:-python3}" "$project_dir/tools/install_offline.py" --mode full "$@"
