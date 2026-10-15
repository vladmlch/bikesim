#!/usr/bin/env bash
# tools/run_native_checks.sh <build_dir> — run the produced native check
# sweeps in the fixed contract order through bike-native-checks.
#
# This is the Python-free replacement for the python-driven sweep block in
# tools/run_tests.sh: each produced kind fails closed on missing tools,
# malformed contexts, stale manifests, or analyzer findings — identical to
# the oracle, minus the interpreter. The sweep set and order are the same:
#   frontends tidy analyzer cppcheck odr scripts headers
#   diagnostic-controls context
# followed by the structural contracts sweep.
#
# Tool overrides are picked up from the emitted context (configure-time
# CLANG_TIDY / ANALYZER_CLANG / CPPCHECK / GXX environment), matching the
# oracle's contract.
set -euo pipefail

# shellcheck source=/dev/null
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)/native_common.sh"

build_dir="${1:-}"
if [[ -z "${build_dir}" ]]; then
    bike_die "usage: tools/run_native_checks.sh <build_dir>"
fi
if [[ ! -d "${build_dir}" ]]; then
    bike_die "build directory does not exist: ${build_dir}"
fi
build_dir="$(cd -- "${build_dir}" >/dev/null 2>&1 && pwd -P)"
repo_root="$(bike_repo_root)"
checks="$(bike_checks_binary "${build_dir}")"

for kind in frontends tidy analyzer cppcheck odr scripts headers \
    diagnostic-controls context; do
    bike_log "sweep ${kind}"
    "${checks}" --kind "${kind}" --build "${build_dir}"
done
bike_log "sweep contracts"
"${checks}" --kind contracts --build "${build_dir}" --repo-root "${repo_root}"
bike_log "all produced sweeps passed: ${build_dir}"
