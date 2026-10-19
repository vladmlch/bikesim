#!/usr/bin/env bash
# Slim offline installer for a bike-sim schema-1 source bundle, or the
# documented legacy manifest-less Python-only bundle.
#
# Usage: bash project/packaging/install_slim.sh [install_dir]
#   install_dir — where the venv is created (default: <bundle>/.venv)
#
# Slim bundles omit the packages declared in manifest.slim_external; those
# providers must be preinstalled on the selected interpreter at the exact
# versions the manifest requires (checked by tools/install_offline.py before
# and after environment creation). If the external stack does not qualify,
# the install fails and the full bundle is the remediation — the bundled
# wheels are never used to silently shadow the validated providers.
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

exec "${PYTHON:-python3}" "$project_dir/tools/install_offline.py" --mode slim "$@"
