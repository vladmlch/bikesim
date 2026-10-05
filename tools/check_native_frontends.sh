#!/usr/bin/env bash
# Second-frontend warning sweep: every core TU through GCC, syntax-only.
set -euo pipefail
SRC_DIR="$1"; MJ_INC="$2"
SDK=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk
PY_INC="$(uv run python -c "import sysconfig;print(sysconfig.get_paths()['include'])" 2>/dev/null || echo /nonexistent)"
NB_INC="$(uv run python -m nanobind --include_dir 2>/dev/null || echo /nonexistent)"
while IFS= read -r f; do
  g++-16 -std=c++23 -fsyntax-only -isysroot "$SDK" \
    -isystem "$MJ_INC" -isystem "$NB_INC" -isystem "$PY_INC" \
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion \
    -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef \
    -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast \
    -Woverloaded-virtual -Wnull-dereference -ffp-contract=off \
    -Wlogical-op -Wduplicated-cond -Wuseless-cast -Wstringop-overflow=4 \
    -fanalyzer \
    -Warith-conversion -Wduplicated-branches -Wrestrict \
    -Wformat-signedness -Wformat-overflow=2 -Wformat-truncation=2 \
    -Wextra-semi -Wredundant-tags -Wcast-function-type -Wcast-align=strict \
    -Wcatch-value=3 -Wconditionally-supported -Wdeprecated-copy-dtor \
    -Wvolatile -Winit-self -Wsign-promo -Wctor-dtor-privacy \
    -Wplacement-new=2 -Wmismatched-new-delete -Wsized-deallocation \
    -Winterference-size -Wsubobject-linkage -Wsuggest-override \
    -Wstrict-null-sentinel -Walloca -Wvla -Warray-bounds=2 \
    -Waggressive-loop-optimizations -Wstack-usage=8192 \
    -Wframe-larger-than=8192 -Wmissing-declarations -Wswitch-enum \
    "$f"
done < <(find "$SRC_DIR" -name '*.cpp' | sort)
echo "gcc sweep clean: $SRC_DIR"
