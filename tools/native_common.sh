#!/usr/bin/env bash
# tools/native_common.sh — shared helpers for the produced (Python-free)
# native check launchers. Sourced, never executed: callers set their own
# -euo pipefail policy before sourcing.
#
# Contract:
#   bike_repo_root          — absolute repository root (parent of tools/)
#   bike_checks_binary DIR  — produced bike-native-checks (see search order)
#   bike_audit_binary DIR   — produced bike-python-audit (see search order)
#   bike_die MESSAGE        — fail loudly with a stable prefix
#   bike_log MESSAGE        — progress line on stderr
#
# Binary search order (checks, then audit mirrors it):
#   1. $BIKE_NATIVE_CHECKS / $BIKE_PYTHON_AUDIT when set — an explicit tool
#      path, same convention as the CLANG_TIDY/SHELLCHECK overrides.
#   2. DIR/bin/, DIR/ — the analyzed build may carry its own produced tools.
#   3. <repo>/build/native-release/bin/ — the produced standalone build;
#      lets the transition graph's check_* targets run the native
#      orchestrator against the legacy build dir without an interpreter.

bike_repo_root() {
    local source_dir
    source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
    cd -- "${source_dir}/.." >/dev/null 2>&1 && pwd -P
}

bike_die() {
    printf 'native launcher: %s\n' "$1" >&2
    exit 1
}

bike_log() {
    printf 'native launcher: %s\n' "$1" >&2
}

# A produced binary is required, never optional: a missing executable means
# the build is stale or the wrong build dir was selected — fail closed.
bike_checks_binary() {
    local build_dir="$1"
    local candidate=""
    if [[ -n "${BIKE_NATIVE_CHECKS:-}" ]]; then
        candidate="${BIKE_NATIVE_CHECKS}"
        if [[ ! -x "${candidate}" ]]; then
            bike_die "BIKE_NATIVE_CHECKS is not executable: ${candidate}"
        fi
    else
        for candidate in \
            "${build_dir}/bin/bike-native-checks" \
            "${build_dir}/bike-native-checks" \
            "$(bike_repo_root)/build/native-release/bin/bike-native-checks"; do
            if [[ -x "${candidate}" ]]; then
                break
            fi
            candidate=""
        done
        if [[ -z "${candidate}" ]]; then
            bike_die "bike-native-checks not found under ${build_dir} or the produced standalone build (configure and build build/native-release first)"
        fi
    fi
    printf '%s\n' "${candidate}"
}

bike_audit_binary() {
    local build_dir="$1"
    local candidate=""
    if [[ -n "${BIKE_PYTHON_AUDIT:-}" ]]; then
        candidate="${BIKE_PYTHON_AUDIT}"
        if [[ ! -x "${candidate}" ]]; then
            bike_die "BIKE_PYTHON_AUDIT is not executable: ${candidate}"
        fi
    else
        for candidate in \
            "${build_dir}/bin/bike-python-audit" \
            "${build_dir}/bike-python-audit" \
            "$(bike_repo_root)/build/native-release/bin/bike-python-audit"; do
            if [[ -x "${candidate}" ]]; then
                break
            fi
            candidate=""
        done
        if [[ -z "${candidate}" ]]; then
            bike_die "bike-python-audit not found under ${build_dir} or the produced standalone build (configure and build build/native-release first)"
        fi
    fi
    printf '%s\n' "${candidate}"
}
