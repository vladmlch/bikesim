#!/usr/bin/env bash
# CMake/CTest/CPack CI lane; no interpreter or Python-environment setup.
# Usage: ci_presets.sh PRESET [checks|package|offline|gui-smoke]
set -euo pipefail
[[ $# -ge 1 && $# -le 2 ]] || { printf 'usage: %s PRESET [checks|package|offline|gui-smoke]\n' "$0" >&2; exit 2; }
preset=$1
mode=${2:-checks}
case "$preset" in release|debug|asan|rtsan|coverage|gui) ;; *) exit 2 ;; esac
case "$mode" in checks|package|offline|gui-smoke) ;; *) exit 2 ;; esac
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cd -- "$repo_root"
build=build/$preset
[[ $preset != release ]] || build=build/native-release
system=$(uname -s)
machine=$(uname -m)
lane=${BIKE_CI_LANE:-$system-$machine-$preset-$mode}
case "$lane" in *[!a-zA-Z0-9_.-]*|'') printf 'unsafe lane name\n' >&2; exit 2 ;; esac
evidence=$repo_root/output/ci/$lane
mkdir -p -- "$evidence"
collect_evidence() {
    local result=$?
    trap - EXIT
    if [[ $system == Linux && -n ${BIKE_LINUX_CONTAINER:-} ]]; then
        # build trees are intentionally excluded from container.sh pull;
        # copy evidence from this run's selected build, without object files.
        for item in Testing native_check_summaries native_check_logs CMakeCache.txt \
            compile_commands.json native_check_context.json bike_policy_report.json \
            packages package_smoke coverage.lcov coverage-summary.txt coverage-junit.xml ctest-junit.xml; do
            docker cp "$BIKE_LINUX_CONTAINER:/src/$build/$item" "$evidence/" >/dev/null 2>&1 || true
        done
    elif [[ -d $build ]]; then
        for item in Testing native_check_summaries native_check_logs CMakeCache.txt \
            compile_commands.json native_check_context.json bike_policy_report.json \
            packages package_smoke coverage.lcov coverage-summary.txt coverage-junit.xml ctest-junit.xml; do
            [[ ! -e $build/$item ]] || cp -R "$build/$item" "$evidence/" || true
        done
    fi
    printf '%s\n' "$result" > "$evidence/exit-code.txt"
    git rev-parse HEAD > "$evidence/commit.txt" || printf 'unavailable\n' > "$evidence/commit.txt"
    exit "$result"
}
trap collect_evidence EXIT
if [[ $system == Linux ]]; then
    [[ $machine == aarch64 ]] || { printf 'Linux qualification requires an aarch64 host and aarch64 container\n' >&2; exit 2; }
    [[ -n ${BIKE_LINUX_CONTAINER:-} ]] || { printf 'CI must name its container\n' >&2; exit 2; }
    lane_cmd=(bash tools/linux/container.sh exec)
    # CI supplies a run-unique container name; no existing container is removed.
    bash tools/linux/container.sh image
    bash tools/linux/container.sh sync
else
    [[ $system == Darwin && $machine == arm64 ]] || { printf 'unsupported qualification host\n' >&2; exit 2; }
    lane_cmd=()
    bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps darwin-arm64
fi
"${lane_cmd[@]}" cmake --workflow --preset "$preset"
"${lane_cmd[@]}" bash tools/run_native_checks.sh "$build"
case "$mode" in
    checks) ;;
    package|offline)
        "${lane_cmd[@]}" ctest --test-dir "$build" -R '^package_smoke$' --no-tests=error --output-on-failure
        "${lane_cmd[@]}" cpack --config "$build/CPackConfig.cmake" -G TGZ
        "${lane_cmd[@]}" cpack --config "$build/CPackSourceConfig.cmake" -G TGZ
        if [[ $mode == offline ]]; then
            [[ $system == Linux ]] || { printf 'offline rehearsal is the Linux lane\n' >&2; exit 2; }
            mkdir -p -- "$evidence/offline-input"
            docker cp "$BIKE_LINUX_CONTAINER:/src/$build/packages/." "$evidence/offline-input/"
            # A fresh disposable container, no network, and read-only inputs.
            # The extractor verifies the lock and consumes only local archives.
            docker run --rm --network none --platform linux/arm64 \
                -v "$evidence/offline-input:/packages:ro" \
                "${BIKE_LINUX_IMAGE:-bike-dev-linux:24.04}" bash -euo pipefail -c '
                    mkdir -p /tmp/rebuild
                    cd /tmp/rebuild
                    shopt -s nullglob
                    archives=(/packages/bike-sim-*-source.tar.gz)
                    [[ ${#archives[@]} -eq 1 ]]
                    tar xf "${archives[0]}"
                    roots=(bike-sim-*-source)
                    [[ ${#roots[@]} -eq 1 && -d ${roots[0]} ]]
                    cd "${roots[0]}"
                    bash tools/linux/extract_deps.sh native/dependencies.lock.json .deps linux-aarch64
                    cmake --workflow --preset release
                '
        fi
        ;;
    gui-smoke)
        [[ $system == Linux && $preset == gui ]] || { printf 'gui-smoke needs Linux gui preset\n' >&2; exit 2; }
        "${lane_cmd[@]}" bash tools/linux/gui_smoke.sh "$build"
        ;;
esac
