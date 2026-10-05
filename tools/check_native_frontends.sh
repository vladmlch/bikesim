#!/usr/bin/env bash
# Second-frontend warning sweep: every core TU through GCC, syntax-only.
set -euo pipefail
SRC_DIR="$1"; MJ_INC="$2"
SDK=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk
PY_INC="$(python -c "import sysconfig;print(sysconfig.get_paths()['include'])" 2>/dev/null || echo /nonexistent)"
NB_INC="$(python -m nanobind --include_dir 2>/dev/null || echo /nonexistent)"
for f in "$SRC_DIR"/*.cpp; do
  g++-16 -std=c++23 -fsyntax-only -isysroot "$SDK" \
    -isystem "$MJ_INC" -isystem "$NB_INC" -isystem "$PY_INC" \
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion \
    -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef \
    -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast \
    -Woverloaded-virtual -Wnull-dereference \
    -Wlogical-op -Wduplicated-cond -Wuseless-cast -Wstringop-overflow=4 \
    "$f"
done
echo "gcc sweep clean: $SRC_DIR"
