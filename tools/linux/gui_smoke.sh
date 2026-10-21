#!/usr/bin/env bash
# tools/linux/gui_smoke.sh BUILD_DIR — bounded native GLFW/X11 viewer smoke.
# Uses Xvfb/software GL; this is NOT viewer-platform acceptance and must not
# close any row in docs/development/viewer-platform-acceptance.md.
# Preserve every nonzero diagnostic/loader/timeout exit; no unmeasured physics
# baseline is treated as success.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    printf 'usage: %s BUILD_DIR\n' "$0" >&2
    exit 2
fi
build_dir="$(cd -- "$1" && pwd -P)"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd -- "$script_dir/../.." && pwd -P)"
if [[ ! -x "$build_dir/bin/bike-ride" ]]; then
    printf 'missing executable: %s/bin/bike-ride\n' "$build_dir" >&2
    exit 2
fi
for required in xvfb-run timeout; do
    if ! command -v "$required" >/dev/null 2>&1; then
        printf 'missing required Linux smoke tool: %s\n' "$required" >&2
        exit 2
    fi
done
cd -- "$repo_root"
export LIBGL_ALWAYS_SOFTWARE=1
# Cover setup, rendering and terminal cleanup with a wall-clock bound too.
# SIGTERM requests normal cleanup; SIGKILL bounds a stuck cleanup path.
exec timeout --kill-after=10s 120s \
    xvfb-run -a -s '-screen 0 1280x720x24' \
    "$build_dir/bin/bike-ride" --backend native \
    --physics-config "$repo_root/examples/research/viewer_physics_welded.toml" \
    --track "$repo_root/examples/research/rough_uphill_savage.toml" \
    --duration 1 --close-on-finish --no-plots
