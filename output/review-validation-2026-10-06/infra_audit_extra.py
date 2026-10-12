from pathlib import Path
import json, os, subprocess, shutil
REPO=Path('/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2')
ROOT=Path('/private/tmp/infra_audit_extra');ROOT.mkdir(exist_ok=True)
results=[]
def run(label,args,cwd=None,env=None):
    e=os.environ.copy();e['UV_CACHE_DIR']='/private/tmp/infra-audit-uv-cache';e.update(env or {})
    r=subprocess.run(args,cwd=cwd or REPO,env=e,text=True,capture_output=True,timeout=30)
    out=r.stdout+r.stderr
    results.append(dict(label=label,args=args,rc=r.returncode,output=out))
    print(f'{label}: rc={r.returncode}\n{out[:1800]}',flush=True)
    (ROOT/'results.json').write_text(json.dumps(results,indent=2))

# Missing system header hides a feature-selected buggy branch. Control includes real header.
src=ROOT/'native/src';src.mkdir(parents=True,exist_ok=True)
inc=ROOT/'include';inc.mkdir(exist_ok=True)
(inc/'required.hpp').write_text('#define FEATURE 1\n')
(src/'feature.cpp').write_text('#include <required.hpp>\n#ifdef FEATURE\nint f(){int *p=nullptr;return *p;}\n#endif\n')
build=ROOT/'build';build.mkdir(exist_ok=True)
for include in [False,True]:
    args='c++ -std=c++23 ' + ('-I'+str(inc)+' ' if include else '') + '-c '+str(src/'feature.cpp')+' -o feature.o'
    (build/'compile_commands.json').write_text(json.dumps([dict(directory=str(build),file=str(src/'feature.cpp'),command=args)]))
    run('cppcheck-header-present' if include else 'cppcheck-header-missing',['bash',str(REPO/'tools/check_cppcheck.sh'),str(build),str(src),str(REPO/'.venv/bin/python')])

# Missing find source dirs are silently accepted by the frontend loop.
run('frontend-source-missing',['bash',str(REPO/'tools/check_native_frontends.sh'),str(ROOT/'missing-source'),'/nonexistent'])

# Real clang support, umbrella-warning comparison.
sample=ROOT/'warnings.cpp';sample.write_text('bool a(); bool b();\nbool c() { return a() & b(); }\nstruct S { int n;; };\n')
sdk='/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk'
flags=['clang++','-isysroot',sdk,'-std=c++23','-fsyntax-only','-Wall','-Wextra','-Wpedantic','-Wextra-semi-stmt',str(sample)]
run('warning-current-umbrella',flags)
run('warning-extra-flags',flags+['-Wbitwise-instead-of-logical','-Wextra-semi'])

# Hardening debug macro: compare ordinary container/iterator layouts, no ABI toggles.
layout=ROOT/'layout.cpp';layout.write_text('#include <vector>\n#include <string>\n#include <span>\n#include <iostream>\nint main(){std::cout << sizeof(std::vector<int>) << " " << sizeof(std::vector<int>::iterator) << " " << sizeof(std::string) << " " << sizeof(std::span<int>) << "\\n";}\n')
for mode in ['FAST','EXTENSIVE','DEBUG']:
    exe=ROOT/f'layout-{mode}'
    run('hardening-compile-'+mode,['clang++','-isysroot',sdk,'-std=c++23','-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_'+mode,str(layout),'-o',str(exe)])
    run('hardening-layout-'+mode,[str(exe)])

# Mock coverage script dependencies and preserve command/env routing.
project=ROOT/'coverage-project';(project/'tools').mkdir(parents=True,exist_ok=True)
shutil.copy(REPO/'tools/native_coverage.sh',project/'tools/native_coverage.sh')
mockbin=ROOT/'mockbin';mockbin.mkdir(exist_ok=True)
scripts={
'cmake':'#!/bin/sh\necho "cmake args=$*"\n',
'sysctl':'#!/bin/sh\necho 1\n',
'uv':'#!/bin/sh\necho "test selector=$NATIVE_TEST_BUILD_DIR cwd=$PWD"\ntouch "${LLVM_PROFILE_FILE%/*}/mock.profraw"\n',
'xcrun':'#!/bin/sh\necho "xcrun argc=$# args=$*"\n'}
for name,content in scripts.items():
    p=mockbin/name;p.write_text(content);p.chmod(0o755)
custom=ROOT/'custom/coverage';custom.mkdir(parents=True,exist_ok=True)
(custom/'bike_native.cpython-313-darwin.so').touch();(custom/'bike_native.cpython-314-darwin.so').touch()
run('coverage-custom-dir-multiple-so',['bash',str(project/'tools/native_coverage.sh'),str(custom)],env={'PATH':str(mockbin)+':'+os.environ['PATH']})

# Fresh standalone rider-forces collection selects regular extension despite coverage selector.
probe=ROOT/'rider_import.py';probe.write_text('import sys, runpy\nfrom pathlib import Path\nrepo=Path('+repr(str(REPO))+')\nsys.path.insert(0,str(repo/"tests/reference"))\nrunpy.run_path(str(repo/"tests/reference/test_native_rider_forces.py"))\nimport bike_native\nprint(bike_native.__file__)\n')
run('rider-forces-coverage-ignored',['uv','run','python',str(probe)],env={'NATIVE_TEST_BUILD_DIR':'coverage'})
run('committed-bench-rewind-zero',[str(REPO/'tools/proto_native_bench/bench'),'bare','1','0'])
run('committed-bench-zero-steps',[str(REPO/'tools/proto_native_bench/bench'),'bare','0'])

# Oversized positive dimensions demonstrate unchecked model compatibility beyond one arena.
import struct,mujoco
art=Path('/private/tmp/infra_audit_repro/tools/proto_native_bench/artifacts')
small=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
mujoco.mj_saveModel(small,str(art/'model.mjb'))
n=100000
(art/'state.bin').write_bytes(struct.pack('6i',0xBEA0,n,small.nv,small.na,small.nu,small.nbody)+struct.pack('d',0.)+bytes(8*(n+2*small.nv+small.na+small.nu+6*small.nbody)))
run('bench-mismatched-large-nq',['/private/tmp/infra_audit_repro/bench-asan','bare','1'],cwd=art.parents[2])
