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
        if [[ "${PULL_TEST_EARLY_DEST:-0}" == 1 && "$src" == */late-trigger ]]; then
            mkdir -p "$PULL_TEST_CHECKOUT/late-before"
            printf 'early arrival\n' > "$PULL_TEST_CHECKOUT/late-before/sentinel"
        fi
        ;;
    *) exit 93 ;;
esac
DOCKER
chmod +x "$scratch/bin/docker"
cat > "$scratch/bin/mv" <<'MOVE'
#!/usr/bin/env bash
set -euo pipefail
if [[ "${PULL_TEST_PUBLISH_FAIL:-0}" == 1 && ( "$2" == */1/ready || "$2" == */1/ready/* ) ]]; then
    exit 42
fi
# Invalidate the absent-destination assumption after the driver's last check.
# A same-basename move into the parent must fail, never nest the staged entry.
if [[ "${PULL_TEST_LATE_DEST:-0}" == 1 && ( "$2" == */1/ready || "$2" == */1/ready/* ) ]]; then
    mkdir -p "$PULL_TEST_CHECKOUT/late"
    printf 'late arrival\n' > "$PULL_TEST_CHECKOUT/late/sentinel"
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

# Select an added group to independently record each pre-fix RED. The default
# retains all original cases and runs every new group.
focus="${PULL_TEST_FOCUS:-all}"
if [[ "$focus" == all || "$focus" == excluded ]]; then
    for pair in '.venv .VENV' '.deps .DEPS' 'build BUILD' 'build-debug BUILD-DEBUG' '.git .GIT' '__pycache__ __PYCACHE__'; do
        lower="${pair% *}" upper="${pair#* }"
        mkdir -p "$PULL_TEST_CHECKOUT/$lower" "$PULL_TEST_SOURCE/$upper"
        printf 'protected\n' > "$PULL_TEST_CHECKOUT/$lower/sentinel"
        printf 'replacement\n' > "$PULL_TEST_SOURCE/$upper/sentinel"
        reject "$upper"
        [[ "$(cat "$PULL_TEST_CHECKOUT/$lower/sentinel")" == protected ]]
        pass "direct excluded case alias [$upper] preserves protected content"
        mkdir -p "$PULL_TEST_CHECKOUT/protected-parent/$lower" "$PULL_TEST_SOURCE/protected-parent/$upper"
        printf 'protected\n' > "$PULL_TEST_CHECKOUT/protected-parent/$lower/sentinel"
        printf 'replacement\n' > "$PULL_TEST_SOURCE/protected-parent/$upper/sentinel"
        # .git is excluded at the checkout root by the existing policy.
        if [[ "$lower" != .git ]]; then
            reject protected-parent
            [[ "$(cat "$PULL_TEST_CHECKOUT/protected-parent/$lower/sentinel")" == protected ]]
            pass "nested excluded case alias [$upper] preserves protected content"
        fi
        rm -rf "$PULL_TEST_SOURCE/protected-parent" "$PULL_TEST_CHECKOUT/protected-parent"
    done
fi
if [[ "$focus" == all || "$focus" == operands ]]; then
    mkdir -p "$scratch/identity/lower"
    if [[ "$scratch/identity/lower" -ef "$scratch/identity/LOWER" ]]; then
        printf 'INFO: host case aliases share filesystem identity; alias cases execute\n'
        mkdir -p "$PULL_TEST_SOURCE/newdir"
        printf 'new\n' > "$PULL_TEST_SOURCE/newdir/value"
        reject newdir NEWDIR
        [[ ! -e "$PULL_TEST_CHECKOUT/newdir" && ! -e "$PULL_TEST_CHECKOUT/NEWDIR" ]]
        pass 'absent filesystem-equivalent operands reject before publication'
        mkdir -p "$PULL_TEST_CHECKOUT/newdir"
        printf 'original\n' > "$PULL_TEST_CHECKOUT/newdir/value"
        reject newdir NEWDIR
        [[ "$(cat "$PULL_TEST_CHECKOUT/newdir/value")" == original && ! -e "$PULL_TEST_CHECKOUT/newdir/ready" ]]
        pass 'existing filesystem-equivalent operands preserve original'
        mkdir -p "$PULL_TEST_SOURCE/newdir/child"
        printf 'child\n' > "$PULL_TEST_SOURCE/newdir/child/value"
        reject newdir NEWDIR/child
        reject NEWDIR/child newdir
        [[ "$(cat "$PULL_TEST_CHECKOUT/newdir/value")" == original && ! -e "$PULL_TEST_CHECKOUT/newdir/child" ]]
        pass 'ancestor/descendant aliases reject in both operand orders'
        mkdir -p "$PULL_TEST_SOURCE/absent-parent/child"
        printf 'child\n' > "$PULL_TEST_SOURCE/absent-parent/child/value"
        reject absent-parent ABSENT-PARENT/child
        reject ABSENT-PARENT/child absent-parent
        [[ ! -e "$PULL_TEST_CHECKOUT/absent-parent" ]]
        pass 'absent ancestor/descendant aliases reject in both orders'
        PULL_TEST_PUBLISH_FAIL=1 reject newdir NEWDIR two
        [[ "$(cat "$PULL_TEST_CHECKOUT/newdir/value")" == original && ! -e "$PULL_TEST_CHECKOUT/two" ]]
        pass 'alias operands reject even with publication failure armed'
    else
        printf 'SKIP: five case-alias operand cases require case-insensitive host filesystem\n'
    fi
    # Hardlinks exercise actual inode identity independently of name folding.
    printf 'original\n' > "$PULL_TEST_CHECKOUT/hard-one"
    ln "$PULL_TEST_CHECKOUT/hard-one" "$PULL_TEST_CHECKOUT/hard-two"
    printf 'new\n' > "$PULL_TEST_SOURCE/hard-one"
    printf 'new\n' > "$PULL_TEST_SOURCE/hard-two"
    reject hard-one hard-two
    [[ "$(cat "$PULL_TEST_CHECKOUT/hard-one")" == original && "$PULL_TEST_CHECKOUT/hard-one" -ef "$PULL_TEST_CHECKOUT/hard-two" ]]
    pass 'filesystem-equivalent hardlink operands preserve both originals'
    composed=$'caf\303\251'
    decomposed=$'cafe\314\201'
    mkdir -p "$scratch/identity/$composed"
    if [[ "$scratch/identity/$composed" -ef "$scratch/identity/$decomposed" ]]; then
        printf 'INFO: host Unicode normalization aliases share identity; alias case executes\n'
        mkdir -p "$PULL_TEST_SOURCE/$composed"
        printf 'new\n' > "$PULL_TEST_SOURCE/$composed/value"
        reject "$composed" "$decomposed"
        [[ ! -e "$PULL_TEST_CHECKOUT/$composed" ]]
        pass 'absent Unicode normalization aliases reject before publication'
    else
        printf 'SKIP: Unicode normalization alias case requires normalizing host filesystem\n'
    fi
fi
if [[ "$focus" == all || "$focus" == publication ]]; then
    mkdir -p "$PULL_TEST_SOURCE/brand-new"
    printf 'new\n' > "$PULL_TEST_SOURCE/brand-new/value"
    PULL_TEST_PUBLISH_FAIL=1 PULL_TEST_EXPECT_STATUS=42 reject brand-new two
    [[ ! -e "$PULL_TEST_CHECKOUT/brand-new" && ! -e "$PULL_TEST_CHECKOUT/two" ]]
    pass 'publication failure removes successfully installed absent operand'
    mkdir -p "$PULL_TEST_SOURCE/late-before"
    printf 'new\n' > "$PULL_TEST_SOURCE/late-before/value"
    printf 'trigger\n' > "$PULL_TEST_SOURCE/late-trigger"
    PULL_TEST_EARLY_DEST=1 PULL_TEST_EXPECT_STATUS=2 reject guard late-before late-trigger
    [[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe && "$(cat "$PULL_TEST_CHECKOUT/late-before/sentinel")" == 'early arrival' ]]
    [[ ! -e "$PULL_TEST_CHECKOUT/late-before/value" && ! -e "$PULL_TEST_CHECKOUT/late-trigger" ]]
    pass 'publication recheck preserves unexpected destination and rolls back prior operand'
    mkdir -p "$PULL_TEST_SOURCE/late"
    printf 'new\n' > "$PULL_TEST_SOURCE/late/value"
    PULL_TEST_LATE_DEST=1 reject guard late
    [[ "$(cat "$PULL_TEST_CHECKOUT/guard")" == safe ]]
    [[ "$(cat "$PULL_TEST_CHECKOUT/late/sentinel")" == 'late arrival' ]]
    [[ ! -e "$PULL_TEST_CHECKOUT/late/ready" && ! -e "$PULL_TEST_CHECKOUT/late/late" && ! -e "$PULL_TEST_CHECKOUT/late/value" ]]
    pass 'late destination cannot nest staged directory; prior operand rolls back'
fi
# A normal non-ASCII path remains usable; alias checks must not ban such names.
mkdir -p "$PULL_TEST_SOURCE/données with spaces"
printf 'unicode\n' > "$PULL_TEST_SOURCE/données with spaces/value"
pull 'données with spaces'
[[ "$(cat "$PULL_TEST_CHECKOUT/données with spaces/value")" == unicode ]]
pass 'non-ASCII directory with spaces remains supported'
[[ -z "$(find "$PULL_TEST_CHECKOUT" -maxdepth 1 -name '.bike-linux-pull.*' -print)" ]]
printf 'PASS: %d pull regression cases\n' "$passed"
