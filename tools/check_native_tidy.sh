#!/usr/bin/env bash
# clang-tidy sweep: every core TU through LLVM's linter, compile_commands-driven.
# CLANG_TIDY overrides the binary (brew llvm is keg-only, hence the default).
set -euo pipefail
BUILD_DIR="$1"; SRC_DIR="$2"
TIDY="${CLANG_TIDY:-/opt/homebrew/opt/llvm/bin/clang-tidy}"
OUT="$(mktemp -t native_tidy)"
trap 'rm -f "$OUT"' EXIT
find "$SRC_DIR" -name '*.cpp' | sort | xargs -P 8 -I{} \
  "$TIDY" -p "$BUILD_DIR" --extra-arg=-Wno-error --quiet {} >>"$OUT" 2>&1 || true
if grep -q 'warning:' "$OUT"; then
  cat "$OUT"
  echo "clang-tidy findings in $SRC_DIR — see above" >&2
  exit 1
fi
echo "tidy clean: $SRC_DIR"
