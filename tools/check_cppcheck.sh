#!/usr/bin/env bash
# cppcheck sweep: a third analyzer engine (after clang warnings/CSA and
# gcc -fanalyzer) over the same TU set, driven by compile_commands.json.
# CPPCHECK overrides the binary.
#
# Categories: warning+performance+portability only — style is deliberately
# excluded (same rule as readability-* in .clang-tidy). --inconclusive stays
# on: the one recurring false positive is suppressed, so real inconclusive
# findings still fail the sweep.
#
# Suppressions (verified against the current tree):
#   missingIncludeSystem   — nanobind/mujoco/Python headers are outside the
#                            warning contract (same rule as NB_SUPPRESS_WARNINGS)
#   uninitMemberVarNoCtor  — aggregate PODs in config headers; every instance
#                            is designated-initialized at the use site, and
#                            -ftrivial-auto-var-init=zero +
#                            -Wconditional-uninitialized cover the residue
#   passedByValue          — small numeric value types (Vec3 = 24B) are by
#                            value deliberately; -Wlarge-by-value-copy owns
#                            the real threshold
#   returnByReference      — state() snapshots are owning copies on purpose
#   normalCheckLevelMaxBranches — analysis-limit notice, not a finding
set -euo pipefail
BUILD_DIR="$1"; SRC_DIR="$2"; PY="${3:-python}"
CPPCHECK="${CPPCHECK:-cppcheck}"
# nanobind.h drags in src/nb_internals.h, whose vendored tsl::robin_map lives
# outside the include dir the compile line already carries. Without this path
# cppcheck's preprocessor dies on nb_internals.h's version #error.
NB_INC="$("$PY" -m nanobind --include_dir 2>/dev/null || echo /nonexistent)"
ROBIN_INC="$(dirname "$NB_INC")/ext/robin_map/include"
"$CPPCHECK" --project="$BUILD_DIR/compile_commands.json" \
  -I "$ROBIN_INC" \
  --file-filter="*/native/src/*" \
  --enable=warning,performance,portability --inconclusive --std=c++23 \
  --error-exitcode=1 --inline-suppr -j8 \
  --suppress=missingIncludeSystem \
  --suppress=uninitMemberVarNoCtor \
  --suppress=passedByValue \
  --suppress=returnByReference \
  --suppress=normalCheckLevelMaxBranches \
  --quiet
echo "cppcheck clean: $SRC_DIR"
