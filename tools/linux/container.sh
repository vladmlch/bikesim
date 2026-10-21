#!/usr/bin/env bash
# Linux aarch64 lane for one checkout. No host venv or dependency/build tree
# crosses into the container. Sources live in /src, build trees in /build.
#
# image                         build the pinned lane image
# up                            create/start this checkout's container
# sync                          copy tracked and unignored source files
# venv                          provision /opt/bike-venv from uv.lock
# configure NAME native|root    configure the selected graph; extra CMake args follow
# build NAME [TARGET...]        build with Ninja; keep /build/NAME.log
# failures NAME                 count failed commands in the last build log
# ctest NAME [ARG...]            run CTest in /build/NAME
# exec CMD...                   run a command from /src
# pull PATH...                  copy selected files from /src to this checkout
# gate PROFILE                  run the shared quick|native|full test profile
# down                          explicitly remove this checkout's container
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." >/dev/null 2>&1 && pwd -P)"
image="${BIKE_LINUX_IMAGE:-bike-dev-linux:24.04}"
name="${BIKE_LINUX_CONTAINER:-bike-dev-$(basename -- "$repo_root")}"

die() {
    printf 'container.sh: %s\n' "$*" >&2
    exit 2
}

require_container_owner() {
    local owner
    owner="$(docker container inspect --format '{{index .Config.Labels "bike.checkout"}}' "$name")"
    [[ "$owner" == "$repo_root" ]] ||
        die "container $name belongs to another checkout; set BIKE_LINUX_CONTAINER"
}

in_container() {
    require_container_owner
    docker exec -i -w /src "$name" "$@"
}

build_name() {
    [[ "$1" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] ||
        die "invalid build name: $1"
}

relative_path() {
    case "$1" in
        ''|/*|.|..|../*|*/../*|*/..|*/./*|./*|*/.|*//*|*/)
            die "expected a checkout-relative file path: $1" ;;
    esac
}

require_checkout_parents() {
    local parent="$1"
    while [[ "$parent" == */* ]]; do
        parent="${parent%/*}"
        [[ ! -L "$repo_root/$parent" ]] || die "symlink parent: $parent"
    done
}

source_path() {
    # Explicit exclusions also cover accidentally tracked generated files.
    # --no-recursion below prevents a tracked directory/gitlink from packing
    # its ignored contents. gitignored files otherwise stay excluded by Git.
    case "$1" in
        .git|.git/*|.venv|.venv/*|*/.venv|*/.venv/*|\
        .deps|.deps/*|*/.deps|*/.deps/*|\
        build|build*/*|*/build|*/build*/*|\
        __pycache__/*|*/__pycache__/*)
            return 1 ;;
    esac
    return 0
}

[[ $# -ge 1 ]] ||
    die "usage: container.sh <image|up|sync|venv|configure|build|failures|ctest|exec|pull|gate|down> ..."
[[ "$name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] ||
    die "invalid container name; set BIKE_LINUX_CONTAINER"
cmd="$1"
shift

case "$cmd" in
    image)
        [[ $# -eq 0 ]] || die "usage: image"
        # Supply exactly the acquisition recipe and its pinned lock. A full
        # checkout context would upload host dependency/build/venv trees to
        # Docker before COPY selection could exclude them.
        image_stage="$(mktemp -d "${TMPDIR:-/tmp}/bike-linux-image.XXXXXXXX")"
        trap 'rm -rf -- "$image_stage"' EXIT
        cp -- "$repo_root/tools/linux/Dockerfile" "$image_stage/Dockerfile"
        cp -- "$repo_root/tools/linux/acquire_deps.sh" "$image_stage/acquire_deps.sh"
        cp -- "$repo_root/native/dependencies.lock.json" "$image_stage/dependencies.lock.json"
        docker build --platform linux/arm64 -t "$image" \
            -f "$image_stage/Dockerfile" "$image_stage"
        ;;
    up)
        [[ $# -eq 0 ]] || die "usage: up"
        if ! docker container inspect "$name" >/dev/null 2>&1; then
            docker run -d --platform linux/arm64 --name "$name" \
                --label "bike.checkout=$repo_root" "$image" >/dev/null
        fi
        require_container_owner
        docker start "$name" >/dev/null
        # Provision this container from its own acquired Linux artifacts.
        # A source sync must never copy a Darwin .deps tree over this one.
        # shellcheck disable=SC2016  # Paths and stage variables belong to the container.
        in_container bash -c '
            set -euo pipefail
            mkdir -p /src /build
            if [[ -L /src/.deps || ( -e /src/.deps && ! -d /src/.deps ) ]]; then
                printf "container.sh: /src/.deps must be a real directory\n" >&2
                exit 2
            fi
            if [[ ! -d /src/.deps ]]; then
                if [[ ! -d /opt/bike-deps ]]; then
                    printf "container.sh: image lacks acquired Linux dependencies; rebuild the image and explicitly recreate this checkout container\n" >&2
                    exit 2
                fi
                deps_stage="$(mktemp -d /src/.bike-deps-stage.XXXXXXXX)"
                trap '\''rm -rf -- "$deps_stage"'\'' EXIT
                cp -a /opt/bike-deps/. "$deps_stage/"
                mv -- "$deps_stage" /src/.deps
            fi
        '
        ;;
    sync)
        [[ $# -eq 0 ]] || die "usage: sync"
        # Stage the archive completely before touching the container. A Git
        # or tar failure must not erase the previous source tree.
        stage="$(mktemp -d "${TMPDIR:-/tmp}/bike-linux-sync.XXXXXXXX")"
        trap 'rm -rf -- "$stage"' EXIT
        git -C "$repo_root" ls-files -z --cached --others --exclude-standard > "$stage/candidates"
        while IFS= read -r -d '' rel; do
            source_path "$rel" || continue
            relative_path "$rel"
            # A cached child may outlive a directory replaced by a symlink.
            # tar must not follow that parent into a host-only environment.
            require_checkout_parents "$rel"
            [[ -e "$repo_root/$rel" || -L "$repo_root/$rel" ]] ||
                die "tracked path is missing; stage its deletion before sync: $rel"
            # Submodules need an explicit source acquisition; never silently
            # descend through a gitlink and copy its host build environment.
            [[ ! -d "$repo_root/$rel" || -L "$repo_root/$rel" ]] ||
                die "source path is a directory/gitlink, not an enumerated file: $rel"
            printf '%s\0' "$rel" >> "$stage/files"
        done < "$stage/candidates"
        [[ -s "$stage/files" ]] || die "no source files selected"
        # COPYFILE_DISABLE suppresses Apple metadata without passing a
        # BSD-only option when GNU tar is selected on a macOS host.
        tar_args=(--null --no-xattrs --no-recursion)
        (cd -- "$repo_root" &&
            COPYFILE_DISABLE=1 tar "${tar_args[@]}" -T "$stage/files" -cf "$stage/source.tar")
        "$0" up
        docker cp "$stage/source.tar" "$name:/tmp/bike-source-sync.tar"
        # shellcheck disable=SC2016  # Variables belong to the container's bash.
        in_container bash -c '
            set -euo pipefail
            staged="$(mktemp -d /tmp/bike-source-sync.XXXXXXXX)"
            trap '\''rm -rf -- "$staged"; rm -f /tmp/bike-source-sync.tar'\'' EXIT
            tar --warning=no-unknown-keyword -xf /tmp/bike-source-sync.tar -C "$staged"
            mkdir -p /src
            # Protect Linux dependencies and preset build trees. rsync keeps
            # unchanged mtimes and removes files deleted from the host tree.
            rsync -a --delete --exclude=/.deps/ --exclude=/build/ "$staged/" /src/
        '
        ;;
    venv)
        [[ $# -eq 0 ]] || die "usage: venv"
        in_container uv sync --frozen --group native
        ;;
    configure)
        [[ $# -ge 2 ]] || die "usage: configure NAME native|root [cmake args...]"
        build="$1"
        build_name "$build"
        graph="$2"
        shift 2
        case "$graph" in
            native)
                src=/src/native
                extra=(-DPython_EXECUTABLE=/opt/bike-venv/bin/python3)
                ;;
            root)
                src=/src
                extra=(-DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_DEPENDENCY_ROOT=/src/.deps)
                ;;
            *) die "unknown graph: $graph (expected native or root)" ;;
        esac
        in_container cmake -S "$src" -B "/build/$build" -G Ninja "${extra[@]}" "$@"
        ;;
    build)
        [[ $# -ge 1 ]] || die "usage: build NAME [ninja targets...]"
        build="$1"
        build_name "$build"
        shift
        # shellcheck disable=SC2016  # Positionals are expanded by the container's bash.
        in_container bash -c 'set -euo pipefail; ninja -C "/build/$1" -k 0 "${@:2}" 2>&1 | tee "/build/$1.log"' build "$build" "$@"
        ;;
    failures)
        [[ $# -eq 1 ]] || die "usage: failures NAME"
        build_name "$1"
        # Publish a count only after awk read the log successfully. Its END
        # block may otherwise print zero even when opening the log failed.
        # shellcheck disable=SC2016  # $1 is the container command's positional.
        in_container bash -c 'set -euo pipefail; count="$(awk '\''/^FAILED:/ {n++} END {print n+0}'\'' "/build/$1.log")"; printf "%s\n" "$count"' failures "$1"
        ;;
    ctest)
        [[ $# -ge 1 ]] || die "usage: ctest NAME [ctest args...]"
        build="$1"
        build_name "$build"
        shift
        in_container ctest --test-dir "/build/$build" --no-tests=error --output-on-failure "$@"
        ;;
    exec)
        [[ $# -ge 1 ]] || die "usage: exec CMD..."
        in_container "$@"
        ;;
    pull)
        [[ $# -ge 1 ]] || die "usage: pull PATH..."
        for rel in "$@"; do
            relative_path "$rel"
            source_path "$rel" || die "not a source-file destination: $rel"
            # Refuse host symlink parents: docker cp must stay in this checkout.
            require_checkout_parents "$rel"
            [[ ! -L "$repo_root/$rel" ]] || die "symlink destination: $rel"
            [[ ! -e "$repo_root/$rel" || -f "$repo_root/$rel" ]] ||
                die "destination is not a regular file: $rel"
            in_container test -f "/src/$rel"
            in_container test ! -L "/src/$rel"
            mkdir -p -- "$repo_root/$(dirname -- "$rel")"
            docker cp "$name:/src/$rel" "$repo_root/$rel"
        done
        ;;
    gate)
        [[ $# -eq 1 ]] || die "usage: gate quick|native|full"
        case "$1" in
            quick|native|full) ;;
            *) die "usage: gate quick|native|full" ;;
        esac
        # CXX only selects a compiler for a fresh tree. Refuse a stale GCC
        # cache instead of deleting it or silently running the wrong lane.
        # The selected binding tree is likewise owned by the Clang lane.
        # shellcheck disable=SC2016  # Positionals and cache paths belong to the container shell.
        in_container bash -c '
            set -euo pipefail
            export CXX=/usr/bin/clang++-18
            export NATIVE_TEST_BUILD_PATH=/build/native-clang
            export UV_PROJECT_ENVIRONMENT=/opt/bike-venv
            if [[ "$1" != quick ]]; then
                for cache in /src/build/native-release/CMakeCache.txt /build/native-clang/CMakeCache.txt; do
                    [[ -f "$cache" ]] || continue
                    compiler="$(sed -n "s/^CMAKE_CXX_COMPILER:[^=]*=//p" "$cache")"
                    # clang and clang++ can resolve to the same executable;
                    # argv[0] still selects different driver behavior.
                    if [[ "$compiler" != "$CXX" ]]; then
                        printf "gate: %s was not configured with %s; move that build directory aside and configure a fresh Clang tree before retrying\n" "$cache" "$CXX" >&2
                        exit 2
                    fi
                done
            fi
            exec bash tools/run_tests.sh "$1"
        ' gate "$1"
        ;;
    down)
        [[ $# -eq 0 ]] || die "usage: down"
        require_container_owner
        docker rm -f "$name" >/dev/null
        ;;
    *)
        die "unknown command: $cmd"
        ;;
esac
