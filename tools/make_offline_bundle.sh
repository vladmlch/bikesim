#!/usr/bin/env bash
# Reconstructed documented compatibility wrapper: the previous untracked
# implementation was not supplied in Repomix. Native capability is opt-in.
set -euo pipefail

bundle_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$bundle_project_root"

if [[ $# -eq 0 ]]; then
    # Legacy no-argument contract: Linux x64 CPython 3.13 full and slim.
    # A current project environment and selected tool lock/wheels must already
    # exist. --no-sync avoids changing it just by entering this wrapper.
    for bundle_mode in full slim; do
        uv run --frozen --group native --no-sync python tools/offline_bundle.py \
            --target linux-x86_64 --python 3.13 --mode "$bundle_mode" \
            --legacy-names --output output/offline-bundles
    done
else
    exec uv run --frozen --group native --no-sync python tools/offline_bundle.py "$@"
fi
