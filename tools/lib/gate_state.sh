#!/usr/bin/env bash
# tools/lib/gate_state.sh — where the gate journal lives. Sourced, never
# executed.
#
#   bike_gate_state_dir <repo_root>
#     prints $BIKE_GATE_STATE_DIR when set and non-empty; else
#     <git common dir>/bike-gate (shared by every worktree) when <repo_root>
#     is the top of its own repository; else <repo_root>/build/bike-gate.
#     A scratch directory nested in a checkout has no repository of its own,
#     so it must not borrow the outer checkout's .git. Creates nothing.

bike_gate_state_dir() {
    local repo_root="$1" phys top common_dir
    if [[ -n "${BIKE_GATE_STATE_DIR:-}" ]]; then
        printf '%s\n' "$BIKE_GATE_STATE_DIR"
        return 0
    fi
    # Same boundary test as tree_id.sh: the physical root must be the
    # repository's own top-level directory.
    phys="$(cd -- "$repo_root" 2>/dev/null && pwd -P)" || phys=""
    top="$(git -C "$repo_root" rev-parse --show-toplevel 2>/dev/null)" || top=""
    if [[ -n "$phys" && "$top" == "$phys" ]] \
        && common_dir="$(git -C "$repo_root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)"; then
        printf '%s/bike-gate\n' "$common_dir"
    else
        printf '%s/build/bike-gate\n' "$repo_root"
    fi
}
