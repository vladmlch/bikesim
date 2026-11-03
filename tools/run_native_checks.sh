#!/usr/bin/env bash
# tools/run_native_checks.sh [--from KIND] <build_dir> — run the produced
# native check sweeps through bike-native-checks, cheapest first, so a
# failure surfaces before the expensive kinds run (measured 2026-11-03:
# cppcheck ~1 min, frontends ~9, headers ~15, odr ~19, tidy ~33,
# analyzer the longest):
#   scripts cppcheck context diagnostic-controls contracts frontends
#   headers odr tidy analyzer
#
# --from KIND skips every kind before KIND. A --from run is an
# intermediate re-check after a fix and says so (PARTIAL); the final
# verification always runs every kind.
#
# BIKE_GATE_TIMINGS_FILE, when set, receives one
# "<kind>\t<seconds>\t<exit code>" line per executed kind.
#
# Each produced kind fails closed on missing tools, malformed contexts,
# stale manifests, or blocking findings. Tool overrides are picked up from
# the emitted context (configure-time CLANG_TIDY / ANALYZER_CLANG /
# CPPCHECK / GXX environment).
set -euo pipefail

# shellcheck source=/dev/null
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)/native_common.sh"

kinds=(scripts cppcheck context diagnostic-controls contracts frontends
    headers odr tidy analyzer)

from=""
if [[ "${1:-}" == "--from" ]]; then
    [[ $# -ge 2 ]] || bike_die "--from needs a kind"
    from="$2"
    shift 2
    known=0
    for kind in "${kinds[@]}"; do
        [[ "${kind}" == "${from}" ]] && known=1
    done
    ((known)) || bike_die "unknown --from kind: ${from} (choose from: ${kinds[*]})"
fi

build_dir="${1:-}"
if [[ -z "${build_dir}" ]]; then
    bike_die "usage: tools/run_native_checks.sh [--from KIND] <build_dir>"
fi
if [[ ! -d "${build_dir}" ]]; then
    bike_die "build directory does not exist: ${build_dir}"
fi
build_dir="$(cd -- "${build_dir}" >/dev/null 2>&1 && pwd -P)"
repo_root="$(bike_repo_root)"
checks="$(bike_checks_binary "${build_dir}")"

skipping=0
[[ -n "${from}" ]] && skipping=1
for kind in "${kinds[@]}"; do
    if ((skipping)); then
        if [[ "${kind}" != "${from}" ]]; then
            bike_log "sweep ${kind} skipped (--from ${from})"
            continue
        fi
        skipping=0
    fi
    bike_log "sweep ${kind}"
    args=(--kind "${kind}" --build "${build_dir}")
    [[ "${kind}" == contracts ]] && args+=(--repo-root "${repo_root}")
    started=${SECONDS}
    rc=0
    "${checks}" "${args[@]}" || rc=$?
    elapsed=$((SECONDS - started))
    bike_log "sweep ${kind} finished in ${elapsed}s (exit ${rc})"
    if [[ -n "${BIKE_GATE_TIMINGS_FILE:-}" ]]; then
        printf '%s\t%s\t%s\n' "${kind}" "${elapsed}" "${rc}" \
            >> "${BIKE_GATE_TIMINGS_FILE}"
    fi
    ((rc == 0)) || exit "${rc}"
done
if [[ -n "${from}" ]]; then
    bike_log "PARTIAL run: kinds before ${from} were skipped; not a final verification"
fi
bike_log "all produced sweeps passed: ${build_dir}"
