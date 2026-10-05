#!/usr/bin/env bash
# Clang Static Analyzer sweep: full path-sensitive per-TU analysis — deeper
# than the clang-analyzer-* checks clang-tidy runs (same engine, but tidy
# maps only the stable subset and shares its TU budget with lint checks).
# ANALYZER_CLANG overrides the binary; Apple clang is NOT supported here —
# it lacks several checkers (keg-only brew llvm is the default).
#
# Two passes over every TU's real compile flags (compile_commands.json):
#   stable+optin  — findings are errors (the warning contract applies)
#   alpha         — report-only: upstream marks alpha checkers unstable, and
#                   they require aggressive-binary-operation-simplification,
#                   which changes the exploration strategy for the whole
#                   engine. Graduated alpha findings should be fixed anyway.
set -euo pipefail
BUILD_DIR="$1"; SRC_DIR="$2"
CLANG="${ANALYZER_CLANG:-/opt/homebrew/opt/llvm/bin/clang++}"

python3 - "$BUILD_DIR" "$SRC_DIR" "$CLANG" <<'PY'
import json, os, shlex, subprocess, sys

build_dir, src_dir, clang = sys.argv[1], os.path.realpath(sys.argv[2]), sys.argv[3]
cc = json.load(open(os.path.join(build_dir, 'compile_commands.json')))
entries = [e for e in cc if e['file'].startswith(src_dir + os.sep)]

def base_cmd(e):
    args = shlex.split(e['command'])
    args[0] = clang
    out, i = [], 0
    while i < len(args):
        a = args[i]
        if a == '-o':
            i += 2; continue
        if a == '-c':
            i += 1; continue
        out.append(a); i += 1
    return out + ['--analyze', '-Xanalyzer', '-analyzer-output=text']

def run(e, extra):
    r = subprocess.run(base_cmd(e) + extra, capture_output=True, text=True,
                       cwd=e['directory'])
    return r.returncode, r.stdout + r.stderr

# optin adds the stable opt-in checkers (.clang-tidy parity);
# cplusplus.Move is a documented false positive inside nanobind's
# std::array caster — NOLINT cannot reach a third-party header.
stable = ['-Xanalyzer', '-analyzer-checker=optin',
          '-Xanalyzer', '-analyzer-disable-checker=cplusplus.Move']
alpha = ['-Xanalyzer', '-analyzer-checker=alpha',
         '-Xanalyzer', '-analyzer-config',
         '-Xanalyzer', 'aggressive-binary-operation-simplification=true',
         '-Xanalyzer', '-analyzer-disable-checker=cplusplus.Move',
         '-Wno-error']

fails = []
for e in entries:
    rc, text = run(e, stable)
    if rc or 'warning:' in text or 'error:' in text:
        fails.append((e['file'], text))
        continue
    rc, text = run(e, alpha)
    interesting = [ln for ln in text.splitlines()
                   if 'warning:' in ln or 'error:' in ln]
    if interesting:
        print(f'== alpha findings (report-only): {e["file"]}',
              file=sys.stderr)
        print('\n'.join(interesting), file=sys.stderr)

if fails:
    for f, t in fails:
        print(f'== analyzer failure: {f}', file=sys.stderr)
        print(t, file=sys.stderr)
    sys.exit(1)
print(f'csa clean: {src_dir}')
PY
