#!/usr/bin/env bash
# Exercise the actual driver with an isolated checkout and Docker CLI fixture.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
scratch="$(mktemp -d "${TMPDIR:-/tmp}/bike-pull-test.XXXXXXXX")"
scratch="$(cd -- "$scratch" && pwd -P)"
trap 'rm -rf -- "$scratch"' EXIT
export PULL_TEST_CHECKOUT="$scratch/checkout" PULL_TEST_SOURCE="$scratch/source"
mkdir -p "$PULL_TEST_CHECKOUT/tools/linux" "$PULL_TEST_SOURCE" "$scratch/bin"
cp "$repo_root/tools/linux/container.sh" "$PULL_TEST_CHECKOUT/tools/linux/"
cat > "$scratch/bin/docker" <<'DOCKER'
#!/usr/bin/env bash
set -euo pipefail
case "$1" in
    container)
        [[ "$2" == inspect && "$3" == --format ]] || exit 90
        printf '%s\n' "${PULL_TEST_OWNER:-$PULL_TEST_CHECKOUT}"
        ;;
    exec)
        [[ "$2" == -i && "$3" == -w && "$4" == /src ]] || exit 91
        shift 5
        args=()
        for arg in "$@"; do args+=("${arg//\/src/$PULL_TEST_SOURCE}"); done
        "${args[@]}"
        ;;
    cp)
        [[ "$2" == *:/src/* ]] || exit 92
        src="$PULL_TEST_SOURCE/${2#*:/src/}"
        if [[ "${PULL_TEST_CP_FAIL:-0}" == 1 ]]; then
            mkdir -p "$3"
            printf 'partial\n' > "$3/partial"
            exit 41
        fi
        cp -pPR "$src" "$3"
        if [[ "${PULL_TEST_BAD_STAGE:-0}" == 1 ]]; then
            ln -s "$PULL_TEST_SOURCE/outside" "$3/injected-link"
        fi
        ;;
    *) exit 93 ;;
esac
DOCKER
chmod +x "$scratch/bin/docker"
cat > "$scratch/bin/mv" <<'MOVE'
#!/usr/bin/env bash
set -euo pipefail
if [[ "${PULL_TEST_PUBLISH_FAIL:-0}" == 1 && "$2" == */1/ready ]]; then
    exit 42
fi
exec /bin/mv "$@"
MOVE
chmod +x "$scratch/bin/mv"
export PATH="$scratch/bin:$PATH" BIKE_LINUX_CONTAINER=pull-test
passed=0
pull() { /bin/bash "$PULL_TEST_CHECKOUT/tools/linux/container.sh" pull "$@"; }
pass() { passed=$((passed + 1)); printf 'ok %d - %s\n' "$passed" "$1"; }
reject() {
    local status=0
    pull "$@" > "$scratch/rejection.log" 2>&1 || status=$?
    if [[ "$status" == 0 ]]; then
        printf 'FAIL: accepted unsafe/failed pull: %s\n' "$*" >&2
        exit 1
    fi
    if [[ -n "${PULL_TEST_EXPECT_STATUS:-}" && "$status" != "$PULL_TEST_EXPECT_STATUS" ]]; then
        cat "$scratch/rejection.log" >&2
        printf 'FAIL: expected exit %s, got %s\n' "$PULL_TEST_EXPECT_STATUS" "$status" >&2
        exit 1
    fi
}
# Removing directory support must fail this first case (the original defect).
mkdir -p "$PULL_TEST_SOURCE/tests/fixtures/golden/linux-aarch64/nested dir"
printf 'golden\n' > "$PULL_TEST_SOURCE/tests/fixtures/golden/linux-aarch64/nested dir/value.npy"
printf 'hidden\n' > "$PULL_TEST_SOURCE/tests/fixtures/golden/linux-aarch64/.metadata"
if ! pull tests/fixtures/golden/linux-aarch64 > "$scratch/initial.log" 2>&1; then
    cat "$scratch/initial.log" >&2
    printf 'FAIL: legitimate golden directory rejected\n' >&2
    exit 1
fi
[[ "$(cat "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/nested dir/value.npy")" == golden ]]
[[ "$(cat "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/.metadata")" == hidden ]]
[[ ! -e "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/linux-aarch64" ]]
pass 'initial directory, spaces, hidden entries, exact basename'
printf 'keep\n' > "$PULL_TEST_CHECKOUT/tests/fixtures/golden/sibling"
printf 'keep\n' > "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/local"
printf 'updated\n' > "$PULL_TEST_SOURCE/tests/fixtures/golden/linux-aarch64/nested dir/value.npy"
pull tests/fixtures/golden/linux-aarch64
[[ "$(cat "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/nested dir/value.npy")" == updated ]]
[[ "$(cat "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/local")" == keep ]]
[[ "$(cat "$PULL_TEST_CHECKOUT/tests/fixtures/golden/sibling")" == keep ]]
[[ ! -e "$PULL_TEST_CHECKOUT/tests/fixtures/golden/linux-aarch64/linux-aarch64" ]]
pass 'repeat directory merges, overwrites matching files, preserves siblings'
printf 'executable\n' > "$PULL_TEST_SOURCE/file with spaces"
chmod 751 "$PULL_TEST_SOURCE/file with spaces"
pull 'file with spaces'
[[ "$(cat "$PULL_TEST_CHECKOUT/file with spaces")" == executable && -x "$PULL_TEST_CHECKOUT/file with spaces" ]]
printf 'replacement\n' > "$PULL_TEST_SOURCE/file with spaces"
pull 'file with spaces'
[[ "$(cat "$PULL_TEST_CHECKOUT/file with spaces")" == replacement && -x "$PULL_TEST_CHECKOUT/file with spaces" ]]
pass 'ordinary file initial/repeat pulls retain executable mode'
pull 'file with spaces' 'file with spaces'
[[ "$(cat "$PULL_TEST_CHECKOUT/file with spaces")" == replacement ]]
pass 'duplicate ordinary-file operands remain compatible'
mkdir -p "$PULL_TEST_SOURCE/empty"
pull empty
[[ -d "$PULL_TEST_CHECKOUT/empty" ]]
pass 'empty directory'
for invalid in '' /absolute ../outside a/../outside ./file a//file a/./file a/; do
    reject "$invalid"
    pass "invalid relative path [$invalid]"
done
for excluded in .venv .deps build native/build native/build-debug/cache nested/.venv/file; do
    reject "$excluded"
    pass "excluded path [$excluded]"
done
reject missing
pass 'missing source'
PULL_TEST_OWNER=another-checkout reject 'file with spaces'
pass 'container ownership retained'
mkdir -p "$PULL_TEST_SOURCE/unsafe"
printf 'safe\n' > "$PULL_TEST_CHECKOUT/guard"
printf 'new\n' > "$PULL_TEST_SOURCE/guard"
ln -s ../guard "$PULL_TEST_SOURCE/unsafe/link"
reject unsafe
[[ ! -e "$PULL_TEST_CHECKOUT/unsafe" && "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe ]]
pass 'source nested symlink'
ln -s unsafe "$PULL_TEST_SOURCE/link-parent"
reject link-parent/link
pass 'source symlink parent'
ln -s guard "$PULL_TEST_SOURCE/link-file"
reject link-file
pass 'source root symlink'
rm "$PULL_TEST_SOURCE/unsafe/link"
mkfifo "$PULL_TEST_SOURCE/unsafe/fifo"
reject unsafe
[[ ! -e "$PULL_TEST_CHECKOUT/unsafe" ]]
pass 'source special entry'
rm "$PULL_TEST_SOURCE/unsafe/fifo"
mkdir "$PULL_TEST_SOURCE/unsafe/.venv"
printf 'secret\n' > "$PULL_TEST_SOURCE/unsafe/.venv/file"
reject unsafe
[[ ! -e "$PULL_TEST_CHECKOUT/unsafe" ]]
pass 'excluded subtree'
ln -s "$scratch" "$PULL_TEST_CHECKOUT/link-parent"
mkdir -p "$PULL_TEST_SOURCE/link-parent"
printf 'new\n' > "$PULL_TEST_SOURCE/link-parent/guard"
reject link-parent/guard
[[ ! -e "$scratch/guard" ]]
pass 'destination symlink parent'
ln -s guard "$PULL_TEST_CHECKOUT/link-file"
rm "$PULL_TEST_SOURCE/link-file"
printf 'new\n' > "$PULL_TEST_SOURCE/link-file"
reject link-file
[[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe ]]
pass 'destination root symlink'
mkdir -p "$PULL_TEST_SOURCE/destination/nested" "$PULL_TEST_CHECKOUT/destination" "$scratch/outside"
printf 'outside\n' > "$scratch/outside/value"
printf 'new\n' > "$PULL_TEST_SOURCE/destination/nested/value"
ln -s "$scratch/outside" "$PULL_TEST_CHECKOUT/destination/nested"
reject destination
[[ "$(cat "$scratch/outside/value")" == outside ]]
pass 'nested destination symlink cannot redirect writes'
PULL_TEST_CP_FAIL=1 PULL_TEST_EXPECT_STATUS=41 reject guard
[[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe ]]
pass 'partial Docker copy failure preserves existing file'
mkdir -p "$PULL_TEST_SOURCE/valid" "$PULL_TEST_CHECKOUT/valid"
printf 'original\n' > "$PULL_TEST_CHECKOUT/valid/value"
printf 'new\n' > "$PULL_TEST_SOURCE/valid/value"
PULL_TEST_BAD_STAGE=1 reject valid
[[ "$(cat "$PULL_TEST_CHECKOUT/valid/value")" == original ]]
pass 'staged tree revalidation preserves existing directory'
reject guard missing
[[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe ]]
pass 'all operands validate before publication'
printf 'conflict\n' > "$PULL_TEST_CHECKOUT/valid/new-directory"
mkdir "$PULL_TEST_SOURCE/valid/new-directory"
reject valid
[[ "$(cat "$PULL_TEST_CHECKOUT/valid/value")" == original ]]
pass 'nested file/directory conflict preserves destination'
mkdir -p "$PULL_TEST_SOURCE/generated/build-debug"
reject generated
[[ ! -e "$PULL_TEST_CHECKOUT/generated" ]]
pass 'empty build subtree excluded'
mkdir -p "$PULL_TEST_SOURCE/counterpart" "$PULL_TEST_CHECKOUT/counterpart"
printf 'new\n' > "$PULL_TEST_SOURCE/counterpart/fifo"
mkfifo "$PULL_TEST_CHECKOUT/counterpart/fifo"
reject counterpart
[[ -p "$PULL_TEST_CHECKOUT/counterpart/fifo" ]]
pass 'special destination counterpart preserved'
printf 'old-two\n' > "$PULL_TEST_CHECKOUT/two"
printf 'new-two\n' > "$PULL_TEST_SOURCE/two"
PULL_TEST_PUBLISH_FAIL=1 PULL_TEST_EXPECT_STATUS=42 reject guard two
[[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe && "$(cat "$PULL_TEST_CHECKOUT/two")" == old-two ]]
pass 'publication failure rolls back both existing operands'
rm "$PULL_TEST_CHECKOUT/two"
PULL_TEST_PUBLISH_FAIL=1 PULL_TEST_EXPECT_STATUS=42 reject empty two
[[ -d "$PULL_TEST_CHECKOUT/empty" && ! -e "$PULL_TEST_CHECKOUT/two" ]]
pass 'publication failure restores directory and absent operand'
printf 'PASS: %d pull regression cases\n' "$passed"
