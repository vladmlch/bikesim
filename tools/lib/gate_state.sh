#!/usr/bin/env bash
# tools/lib/gate_state.sh — where the gate journal lives. Sourced, never
# executed.
#
#   bike_gate_state_dir <repo_root>
#     prints $BIKE_GATE_STATE_DIR when set and non-empty; else
#     <git common dir>/bike-gate (shared by every worktree); else
#     <repo_root>/build/bike-gate outside git. Creates nothing.

bike_gate_state_dir() {
    local repo_root="$1" common_dir
    if [[ -n "${BIKE_GATE_STATE_DIR:-}" ]]; then
        printf '%s\n' "$BIKE_GATE_STATE_DIR"
    elif common_dir="$(git -C "$repo_root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)"; then
        printf '%s/bike-gate\n' "$common_dir"
    else
        printf '%s/build/bike-gate\n' "$repo_root"
    fi
}
