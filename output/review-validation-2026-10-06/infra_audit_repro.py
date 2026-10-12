from pathlib import Path
import json, os, subprocess, struct, shutil

REPO = Path('/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2')
ROOT = Path('/private/tmp/infra_audit_repro')
ROOT.mkdir(exist_ok=True)
results = []

def run(label, args, cwd=None, env=None, timeout=30):
    merged = os.environ.copy()
    merged['UV_CACHE_DIR'] = '/private/tmp/infra-audit-uv-cache'
    if env:
        merged.update(env)
    r = subprocess.run(args, cwd=cwd or REPO, env=merged, text=True, capture_output=True, timeout=timeout)
    output = r.stdout + r.stderr
    results.append({'label': label, 'args': args, 'rc': r.returncode, 'output': output})
    print(f'{label}: rc={r.returncode}\n{output[:1800]}', flush=True)
    (ROOT / 'results.json').write_text(json.dumps(results, indent=2))
    return r

src = ROOT / 'native' / 'src'
src.mkdir(parents=True, exist_ok=True)
(src / 'test.cpp').write_text('int main() { return 0; }\n')
build = ROOT / 'build'
build.mkdir(exist_ok=True)
for label, content in [('error', '#!/bin/sh\necho "error: mocked parse error" >&2\nexit 1\n'), ('crash', '#!/bin/sh\nkill -SEGV $$\n'), ('warning', '#!/bin/sh\necho "warning: mocked finding" >&2\nexit 0\n')]:
    mock = ROOT / f'tidy-{label}'
    mock.write_text(content)
    mock.chmod(0o755)
    run(f'tidy-{label}', ['bash', str(REPO / 'tools/check_native_tidy.sh'), str(build), str(src)], env={'CLANG_TIDY': str(mock)})
run('tidy-missing', ['bash', str(REPO / 'tools/check_native_tidy.sh'), str(build), str(src)], env={'CLANG_TIDY': str(ROOT / 'missing')})
(build / 'compile_commands.json').write_text('[]')
run('analyzer-empty', ['bash', str(REPO / 'tools/check_analyzer.sh'), str(build), str(src)], env={'ANALYZER_CLANG': str(ROOT / 'missing')})
(build / 'compile_commands.json').write_text(json.dumps([{'directory': str(build), 'file': '../native/src/test.cpp', 'command': 'c++ -c ../native/src/test.cpp -o test.o'}]))
run('analyzer-relative-file', ['bash', str(REPO / 'tools/check_analyzer.sh'), str(build), str(src)], env={'ANALYZER_CLANG': str(ROOT / 'missing')})
(build / 'compile_commands.json').write_text(json.dumps([{'directory': str(build), 'file': str(src / 'test.cpp'), 'command': 'c++ -c ' + str(src / 'test.cpp') + ' -o test.o'}]))
mock = ROOT / 'analyzer-alpha-crash'
mock.write_text('#!/bin/sh\ncase "$*" in *analyzer-checker=alpha*) echo "LLVM ERROR: mocked analyzer crash" >&2; exit 1;; esac\nexit 0\n')
mock.chmod(0o755)
run('analyzer-alpha-crash', ['bash', str(REPO / 'tools/check_analyzer.sh'), str(build), str(src)], env={'ANALYZER_CLANG': str(mock)})

for checkout in ['cpp-port-p2', 'other-checkout']:
    base = ROOT / checkout / 'native'
    (base / 'src').mkdir(parents=True, exist_ok=True)
    shutil.copy(REPO / 'native/.clang-tidy', base / '.clang-tidy')
    (base / 'src/test.hpp').write_text('#pragma once\ninline double divide(int a,int b) {return a/b;}\n')
    (base / 'src/test.cpp').write_text('#include "test.hpp"\ndouble caller(int a,int b) {return divide(a,b);}\n')
    run('header-filter-' + checkout, ['/opt/homebrew/opt/llvm/bin/clang-tidy', '--checks=-*,bugprone-integer-division', '--quiet', str(base / 'src/test.cpp'), '--', '-std=c++23'])

import mujoco
mj = Path(mujoco.__file__).parent
model = mujoco.MjModel.from_binary_path(str(REPO / 'tools/proto_native_bench/artifacts/model.mjb'))
hdr = struct.unpack('6i', (REPO / 'tools/proto_native_bench/artifacts/state.bin').read_bytes()[:24])
print(f'original dims: nq={model.nq} nv={model.nv} na={model.na} nu={model.nu} nbody={model.nbody}; hdr={hdr}', flush=True)
results.append({'label':'original-dimensions','model':[model.nq,model.nv,model.na,model.nu,model.nbody], 'hdr':hdr})
bench = ROOT / 'bench-asan'
run('compile-bench-asan', ['clang++', '-isysroot', '/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk', '-O1', '-g', '-std=c++17', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', str(REPO / 'tools/proto_native_bench/bench.cpp'), '-I' + str(mj / 'include'), str(mj / 'libmujoco.3.12.0.dylib'), '-Wl,-rpath,' + str(mj), '-Wl,-headerpad_max_install_names', '-o', str(bench)], timeout=60)
run('fix-bench-install-name', ['install_name_tool', '-change', '@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib', str(mj / 'libmujoco.3.12.0.dylib'), str(bench)])
art = ROOT / 'tools/proto_native_bench/artifacts'
art.mkdir(parents=True, exist_ok=True)
shutil.copy(REPO / 'tools/proto_native_bench/artifacts/model.mjb', art / 'model.mjb')
shutil.copy(REPO / 'tools/proto_native_bench/artifacts/state.bin', art / 'state.bin')
run('bench-rewind-zero', [str(bench), 'bare', '1', '0'], ROOT)
run('bench-rewind-invalid', [str(bench), 'bare', '1', 'abc'], ROOT)
run('bench-heavy-current-model', [str(bench), 'heavy', '1'], ROOT)
state = (art / 'state.bin').read_bytes()
negative = bytearray(state)
struct.pack_into('i', negative, 4, -1)
(art / 'state.bin').write_bytes(negative)
run('bench-negative-nq', [str(bench), 'bare', '1'], ROOT)
shutil.copy(REPO / 'tools/proto_native_bench/artifacts/state.bin', art / 'state.bin')
def save_state(m):
    d=mujoco.MjData(m)
    p=struct.pack('6i',0xBEA0,m.nq,m.nv,m.na,m.nu,m.nbody)+struct.pack('d',d.time)
    for value in [d.qpos,d.qvel,d.act,d.ctrl,d.qfrc_applied,d.xfrc_applied]: p+=value.tobytes()
    (art/'state.bin').write_bytes(p)

for nv in [33,65]:
    xml='<mujoco><worldbody>' + ''.join(f'<body pos="{i} 0 0"><joint name="j{i}" type="slide"/><geom type="sphere" size=".1"/></body>' for i in range(nv)) + '</worldbody><actuator><motor joint="j0"/></actuator></mujoco>'
    m=mujoco.MjModel.from_xml_string(xml)
    mujoco.mj_saveModel(m,str(art/'model.mjb'))
    save_state(m)
    run(f'bench-glue-nv{nv}', [str(bench),'glue','1'],ROOT)
    if nv==33:
        run('bench-heavy-nv33',[str(bench),'heavy','1'],ROOT)

model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
mujoco.mj_saveModel(model,str(art/'model.mjb'))
save_state(model)
run('bench-glue-no-actuator',[str(bench),'glue','1'],ROOT)
shutil.copy(REPO / 'tools/proto_native_bench/artifacts/state.bin', art / 'state.bin')
run('bench-state-model-mismatch',[str(bench),'bare','1'],ROOT)
(ROOT/'results.json').write_text(json.dumps(results,indent=2))
