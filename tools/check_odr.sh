#!/usr/bin/env bash
# ODR sweep: -fsyntax-only cannot see One Definition Rule violations across
# TUs — they only surface at LTO link. Every core TU is compiled with -flto,
# then merged with `g++ -flto -Wodr -r`, which runs the LTO front-end pass
# that emits -Wodr diagnostics. Any warning fails the sweep.
# GXX overrides the compiler.
set -euo pipefail
SRC_DIR="$1"; MJ_INC="$2"; PY="${3:-python}"
GXX="${GXX:-g++-16}"
SDK=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk
PY_INC="$("$PY" -c "import sysconfig;print(sysconfig.get_paths()['include'])" 2>/dev/null || echo /nonexistent)"
NB_INC="$("$PY" -m nanobind --include_dir 2>/dev/null || echo /nonexistent)"
OBJ="$(mktemp -d -t native_odr)"
trap 'rm -rf "$OBJ"' EXIT
i=0
while IFS= read -r f; do
  i=$((i+1))
  # No warning set here — diagnostics are the syntax sweep's contract; this
  # phase only feeds the LTO merge that emits -Wodr.
  "$GXX" -std=c++23 -O2 -flto=auto -ffat-lto-objects -Wno-psabi \
    -isysroot "$SDK" \
    -isystem "$MJ_INC" -isystem "$NB_INC" -isystem "$PY_INC" \
    -c "$f" -o "$OBJ/$i.o"
done < <(find "$SRC_DIR" -name '*.cpp' | sort)
# -flto=auto silences lto-wrapper's serial-compilation notice; the grep only
# counts file-anchored diagnostics (file:line:col: warning:), not driver notes
# like psABI calling-convention notes.
OUT="$("$GXX" -flto=auto -Wodr -Wno-psabi -r "$OBJ"/*.o -o "$OBJ/combined.o" 2>&1)" || {
  printf '%s\n' "$OUT"; exit 1; }
if printf '%s' "$OUT" | grep -qE ':[0-9]+:[0-9]+: (warning|error):'; then
  printf '%s\n' "$OUT"
  echo "odr findings in $SRC_DIR — see above" >&2
  exit 1
fi
echo "odr clean: $SRC_DIR"
