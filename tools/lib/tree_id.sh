#!/usr/bin/env bash
# tools/lib/tree_id.sh — content id of a git working tree, computed without
# writing git objects or touching .git/index.
#
#   bike_tree_id <repo_root>     sourced (no shell options are set here)
#   tools/lib/tree_id.sh <root>  run directly; prints the same id
#
# The id is sha256 over: HEAD's commit id, the binary diff of tracked files
# against HEAD, and the sorted untracked non-ignored files with size and
# mtime. Untracked contents are not hashed (output/ holds ~950 MB), so a
# rewrite that keeps size and mtime is invisible; accepted.
#
# `git diff-index` is used instead of `git diff`: `git diff` rewrites
# .git/index when stat data is stale, which this id must never do.
#
# Prints `nogit` when <repo_root> is not the top of its own repository (a
# scratch repo nested under a checkout must not borrow the outer HEAD) or
# has no commit yet.

# _bike_stat_files <repo_root> — stdin: NUL-separated paths relative to
# repo_root; stdout: "size mtime path". Runs inside repo_root (a subshell, so
# the caller's directory is untouched): the paths are relative to it, and a
# caller elsewhere would stat other files and silently drop these.
_bike_stat_files() {
    if [[ "$(uname -s)" == Darwin ]]; then
        (cd -- "$1" && xargs -0 stat -f '%z %m %N' 2>/dev/null)
    else
        (cd -- "$1" && xargs -0 stat -c '%s %Y %n' 2>/dev/null)
    fi
}

# _bike_sha256 — stdin to hex digest; sha256sum where it exists, else shasum.
_bike_sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum | cut -d' ' -f1
    else
        shasum -a 256 | cut -d' ' -f1
    fi
}

bike_tree_id() {
    local root="$1" phys top head tracked list
    phys="$(cd -- "$root" && pwd -P)" || return 1
    top="$(git -C "$phys" rev-parse --show-toplevel 2>/dev/null)" || top=""
    head="$(git -C "$phys" rev-parse --verify -q HEAD 2>/dev/null)" || head=""
    if [[ -z "$head" || "$top" != "$phys" ]]; then
        printf 'nogit\n'
        return 0
    fi
    tracked="$(git -C "$phys" diff-index -p --binary --no-color --no-ext-diff HEAD --)" || return 1
    list="$(mktemp "${TMPDIR:-/tmp}/bike-tree-id.XXXXXX")" || return 1
    git -C "$phys" ls-files --others --exclude-standard -z | LC_ALL=C sort -z > "$list"
    {
        printf 'head %s\n' "$head"
        printf '%s\n' "$tracked"
        printf 'untracked\n'
        if [[ -s "$list" ]]; then
            _bike_stat_files "$phys" < "$list"
        fi
    } | _bike_sha256
    rm -f "$list"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    if (($# != 1)); then
        printf 'usage: tree_id.sh REPO_ROOT\n' >&2
        exit 2
    fi
    bike_tree_id "$1"
fi
