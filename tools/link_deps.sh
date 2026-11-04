#!/usr/bin/env bash
# link_deps.sh — local development: give this checkout its pinned native
# dependencies through one shared copy, so every checkout and worktree
# reuses a single acquisition instead of holding its own 300+ MB .deps.
#
# The copy lives at $BIKE_DEPS_CACHE/<platform>-<lock hash> (default
# ~/.cache/bike-deps); a changed lock gets a fresh copy. <checkout>/.deps
# becomes a symlink to it, so CMake keeps reading <checkout>/.deps and
# still verifies every recorded hash at configure. Offline bundles and the
# Linux container keep calling tools/linux/acquire_deps.sh directly.
#
# A real .deps directory already in the checkout is adopted as the shared
# copy when it equals a fresh acquisition: its files keep their timestamps,
# so an existing build does not recompile. Otherwise it is kept aside as
# .deps.replaced-<time>.
#
# Usage: tools/link_deps.sh [darwin-arm64|linux-aarch64]
set -euo pipefail

die() { printf 'link_deps: %s\n' "$*" >&2; exit 1; }

[[ $# -le 1 ]] || die 'usage: link_deps.sh [darwin-arm64|linux-aarch64]'
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
lock=$repo/native/dependencies.lock.json
[[ -f $lock ]] || die "missing lock: $lock"

if [[ $# -eq 1 ]]; then
    platform=$1
else
    case "$(uname -s)-$(uname -m)" in
        Darwin-arm64) platform=darwin-arm64 ;;
        Linux-aarch64) platform=linux-aarch64 ;;
        *) die "no default platform for $(uname -s)-$(uname -m); pass one" ;;
    esac
fi

if command -v sha256sum >/dev/null; then
    lock_sha=$(sha256sum -- "$lock")
else
    lock_sha=$(shasum -a 256 -- "$lock")
fi
cache_root=${BIKE_DEPS_CACHE:-$HOME/.cache/bike-deps}
target=$cache_root/$platform-${lock_sha:0:16}
link=$repo/.deps

if [[ ! -f $target/.complete ]]; then
    mkdir -p -- "$cache_root"
    # A copy without the marker is an interrupted earlier run.
    rm -rf -- "$target"
    stage=$(mktemp -d "$cache_root/.stage.XXXXXXXX")
    trap 'rm -rf -- "$stage"' EXIT
    # Archives already on disk are verified by hash, not downloaded again.
    if [[ -d $link/_archives && ! -L $link ]]; then
        mkdir -p -- "$stage/_archives"
        cp -p -- "$link"/_archives/* "$stage/_archives/"
    fi
    bash "$repo/tools/linux/acquire_deps.sh" "$lock" "$stage" "$platform"
    if [[ -d $link && ! -L $link ]] && diff -rq -- "$stage" "$link" >/dev/null; then
        mv -- "$link" "$target"
        printf 'link_deps: adopted the existing .deps as %s\n' "$target"
    else
        mv -- "$stage" "$target"
        printf 'link_deps: acquired %s\n' "$target"
    fi
    printf '%s\n' "${lock_sha:0:64}" > "$target/.complete"
fi

if [[ -L $link ]]; then
    if [[ $(readlink -- "$link") == "$target" ]]; then
        printf 'link_deps: .deps already links to %s\n' "$target"
        exit 0
    fi
    rm -- "$link"
elif [[ -e $link ]]; then
    backup=$link.replaced-$(date +%Y%m%d%H%M%S)
    mv -- "$link" "$backup"
    printf 'link_deps: kept the previous .deps as %s\n' "$backup" >&2
fi
ln -s -- "$target" "$link"
printf 'link_deps: .deps -> %s\n' "$target"
