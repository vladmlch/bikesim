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
# pull PATH...                  merge selected files/directories from /src
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

# Linux names may alias protected host names on a case-insensitive filesystem.
# Conservatively exclude ASCII case variants on every host, for pulls only.
pull_source_path() {
    local folded
    folded="$(printf '%s/' "$1" | LC_ALL=C tr '[:upper:]' '[:lower:]')"
    source_path "${folded%/}"
}

# Compare filesystem identities along both ancestor chains. Reservation paths
# exist on the checkout volume, so this also detects case/normalization aliases
# for destinations that do not exist yet. Existing host paths catch hardlinks.
pull_paths_overlap() {
    local ancestor="$1" other="$2" direction
    for ((direction=0; direction<2; direction++)); do
        while :; do
            [[ ! "$ancestor" -ef "$other" ]] || return 0
            [[ "$ancestor" != / ]] || break
            ancestor="${ancestor%/*}"
            [[ -n "$ancestor" ]] || ancestor=/
        done
        ancestor="$2" other="$1"
    done
    return 1
}

# Validate every copied entry, including dotfiles, without following symlinks.
# Also used inside the container, with exclusion helpers supplied as functions.
pull_tree() {
    local root="$1" rel="$2" entry suffix
    [[ ! -L "$root" && ( -f "$root" || -d "$root" ) ]] || {
        printf 'container.sh: missing, symlink or special pull source: %s\n' "$rel" >&2
        return 2
    }
    find "$root" -print0 > "$3"
    while IFS= read -r -d '' entry; do
        suffix="${entry#"$root"}"
        pull_source_path "$rel$suffix" || {
            printf 'container.sh: excluded pull entry: %s\n' "$rel$suffix" >&2
            return 2
        }
        # A trailing slash also applies build*/__pycache__ exclusions to
        # empty directories, which have no child for find to expose.
        if [[ -d "$entry" ]] && ! pull_source_path "$rel$suffix/"; then
            printf 'container.sh: excluded pull directory: %s\n' "$rel$suffix" >&2
            return 2
        fi
        [[ ! -L "$entry" && ( -f "$entry" || -d "$entry" ) ]] || {
            printf 'container.sh: symlink or special pull entry: %s\n' "$rel$suffix" >&2
            return 2
        }
    done < "$3"
}

pull_cleanup() {
    local status=$? index destination backup rollback_failed=0
    trap - EXIT
    if [[ "$pull_committed" == 0 ]]; then
        for ((index=pull_started-1; index>=0; index--)); do
            destination="$repo_root/${pull_paths[index]}"
            backup="$pull_stage/$index/original/${pull_paths[index]##*/}"
            if [[ -e "$backup" ]]; then
                rm -rf -- "$destination" && mv -- "$backup" "$(dirname -- "$destination")/" || rollback_failed=1
            elif [[ "${pull_installed[index]}" == 1 ]]; then
                rm -rf -- "$destination" || rollback_failed=1
            fi
        done
    fi
    if [[ "$rollback_failed" == 1 ]]; then
        printf 'container.sh: rollback failed; backups retained in %s\n' "$pull_stage" >&2
        exit 2
    fi
    rm -rf -- "$pull_stage"
    exit "$status"
}

pull_paths_from_container() {
    local rel previous index=0 entry suffix destination ready parent
    pull_paths=() pull_existed=() pull_installed=() pull_started=0 pull_committed=0
    pull_stage="$(mktemp -d "$repo_root/.bike-linux-pull.XXXXXXXX")"
    trap pull_cleanup EXIT
    for rel in "$@"; do
        relative_path "$rel"
        pull_source_path "$rel" || die "not a source destination: $rel"
        mkdir -p -- "$pull_stage/aliases/$rel"
        # Overlapping operands would invalidate the prepared merge/rollback.
        for previous in ${pull_paths[@]+"${pull_paths[@]}"}; do
            [[ "$rel" != "$previous" ]] || continue 2
            if pull_paths_overlap "$pull_stage/aliases/$rel" "$pull_stage/aliases/$previous" ||
                pull_paths_overlap "$repo_root/$rel" "$repo_root/$previous"; then
                die "overlapping pull paths: $previous and $rel"
            fi
        done
        require_checkout_parents "$rel"
        destination="$repo_root/$rel"
        [[ ! -L "$destination" ]] || die "symlink destination: $rel"
        mkdir -- "$pull_stage/$index"
        # Reject source symlink parents before Docker can resolve them. The
        # selected tree is checked both before transfer and after staging.
        # shellcheck disable=SC2016  # This script runs in the container.
        in_container bash -c "$(declare -f source_path pull_source_path pull_tree)"'
            set -euo pipefail
            parent="/src/$1"
            while :; do
                [[ ! -L "$parent" ]] || { printf "container.sh: source symlink parent: %s\n" "$parent" >&2; exit 2; }
                [[ "$parent" != /src ]] || break
                parent="${parent%/*}"
            done
            list="$(mktemp /tmp/bike-pull-entries.XXXXXXXX)"
            trap '\''rm -f -- "$list"'\'' EXIT
            pull_tree "/src/$1" "$1" "$list"
        ' pull "$rel"
        docker cp "$name:/src/$rel" "$pull_stage/$index/incoming"
        pull_tree "$pull_stage/$index/incoming" "$rel" "$pull_stage/$index/entries"
        # Existing counterparts must have compatible types and real parents.
        # Unrelated destination entries remain in the merged tree unchanged.
        while IFS= read -r -d '' entry; do
            suffix="${entry#"$pull_stage/$index/incoming"}"
            parent="$destination$suffix"
            [[ ! -L "$parent" ]] || die "symlink destination: $rel$suffix"
            if [[ -e "$parent" ]]; then
                [[ ( -d "$entry" && -d "$parent" ) || ( -f "$entry" && -f "$parent" ) ]] ||
                    die "incompatible destination type: $rel$suffix"
            fi
        done < "$pull_stage/$index/entries"
        mkdir -- "$pull_stage/$index/ready" "$pull_stage/$index/original"
        ready="$pull_stage/$index/ready/${rel##*/}"
        if [[ -e "$destination" ]]; then
            pull_existed+=(1)
            cp -pPR -- "$destination" "$ready"
            if [[ -d "$ready" ]]; then
                cp -pPR -- "$pull_stage/$index/incoming/." "$ready/"
            else
                cp -pP -- "$pull_stage/$index/incoming" "$ready"
            fi
        else
            pull_existed+=(0)
            mv -- "$pull_stage/$index/incoming" "$ready"
        fi
        pull_paths+=("$rel")
        pull_installed+=(0)
        index=$((index + 1))
    done
    # All transfers and merges succeeded. Publish by rename, retaining each
    # original until every operand is installed; EXIT rolls back on failure.
    for ((index=0; index<${#pull_paths[@]}; index++)); do
        destination="$repo_root/${pull_paths[index]}"
        mkdir -p -- "$(dirname -- "$destination")"
        require_checkout_parents "${pull_paths[index]}"
        [[ ! -L "$destination" ]] || die "symlink destination: ${pull_paths[index]}"
        if [[ "${pull_existed[index]}" == 1 ]]; then
            [[ -e "$destination" ]] || die "pull destination disappeared: ${pull_paths[index]}"
        else
            [[ ! -e "$destination" ]] || die "pull destination appeared: ${pull_paths[index]}"
        fi
        pull_started=$((index + 1))
        if [[ "${pull_existed[index]}" == 1 ]]; then
            mv -- "$destination" "$pull_stage/$index/original/"
        fi
        # Match the source basename and target the parent directory. Unlike
        # `mv ready destination`, this cannot nest ready inside a destination
        # directory that unexpectedly appears after the existence check.
        mv -- "$pull_stage/$index/ready/${pull_paths[index]##*/}" "$(dirname -- "$destination")/"
        pull_installed[index]=1
    done
    pull_committed=1
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
        cp -- "$repo_root/tools/linux/extract_deps.sh" "$image_stage/extract_deps.sh"
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
        pull_paths_from_container "$@"
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
