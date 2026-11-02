# GPU Code Gate Compliance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put `native/src/gpu/*` and `native/tests/gpu/*` under the project gate (`bash tools/run_tests.sh native`, then `full`). That means every compiler-policy diagnostic and every sweep finding gets fixed, and the 8 GPU CTest nodes build and pass in `build/native-release`.

**Architecture:** The 8 GPU tests are already registered in the root graph (`CMakeLists.txt`, function `bike_add_gpu_unit_test`, labels `gpu;unit`), but under `bike_apply_native_policy` they produce 573 `-Werror` diagnostics. clang-tidy adds roughly another 100–400 findings per file on top of that.
- **Host-only code** (tests, `model_assets.hpp`, `cpu_batch_runner.cpp`, `metal_batch_runner.mm`) is fixed for real. Tests move to the project's existing test style (`native/tests/test_station.cpp`).
- **Shared FP32 kernel headers** compile as host C++, CUDA and Metal MSL. MSL has no `std::span`/`std::array`, and its address-space qualifiers need raw pointers. Only the checks that are inherent to that constraint get a narrow, registered suppression. Everything else in those headers is fixed for real.

**Tech Stack:** C++23 (host), MSL (runtime-compiled from the same headers), CUDA (host-syntax checked), CMake/Ninja, clang (AppleClang for the build, `/opt/homebrew/opt/llvm/bin/clang-tidy` for the tidy sweep), cppcheck, MuJoCo 3.12.0, uv for Python.

**Spec:** No separate spec. The requirements are AGENTS.md ("Verification"), `docs/agents/native-engineering.md`, and the Context section below. Executors must read both documents first.

## Context (facts established before this plan)

- The worktree is `R=/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/fp32-physics/.worktrees/colab-gpu`, with uncommitted work on top of `0eef55d0`. **Do not commit.** The user decides about commits.
- Earlier, the GPU tests were only in the legacy graph (`native/CMakeLists.txt`), which the gate never builds. The root wiring was added in this session.
- Diagnostic inventory: K2's run produced `/tmp/gpu_diagnostics_unique.txt`. It is regenerated reproducibly by Task 1.
  - By flag: `-Wunsafe-buffer-usage` 344, `-Wsign-conversion` 121, `-Wdouble-promotion` 44, `-Wunsafe-buffer-usage-in-libc-call` 34, `-Wunused-variable` 18, `-Wshorten-64-to-32` 9, `-Wunused-but-set-variable` 2, `-Wformat-nonliteral` 1.
  - By file: test_gpu_batch_step.cpp 121, test_gpu_rider.cpp 83, test_gpu_tire.cpp 75, test_gpu_model_geometry.cpp 67, test_gpu_suspension.cpp 55, test_gpu_model_assets.cpp 32, test_gpu_drivetrain_resistance.cpp 23, tire_kernel.hpp 22, batch_step.hpp 20, rider_kernel.hpp 19, model_assets.hpp 19, common.hpp 14, drivetrain_resistance_kernel.hpp 12, suspension_kernel.hpp 6, cuda_kernels.cu 5.
  - The two runner sources (`cpu_batch_runner.cpp`, `metal_batch_runner.mm`) and `test_gpu_batch_runners.cpp` have **not been counted yet**, because target `bike_test_gpu_batch_runners` was never built under policy.
- clang-tidy sample (`native/.clang-tidy`, HeaderFilterRegex covers `native/src|tests`):
  - test_gpu_suspension.cpp: unsafe-buffer 49, pro-bounds-array-to-pointer-decay 24, pro-bounds-pointer-arithmetic 20, unused-variable 18, avoid-c-arrays 13, easily-swappable-parameters 7, use-designated-initializers 6, double-promotion 6, misc-const-correctness 4, pro-type-union-access 3.
  - test_gpu_model_assets.cpp: use-std-print 63, pro-type-vararg 63, cert-err33-c 61, concurrency-mt-unsafe 60, avoid-do-while 59, pointer-arithmetic 30, unsafe-buffer 25/24, use-designated-initializers 23.
- Sweep order in the gate (`tools/run_native_checks.sh`): frontends, tidy, analyzer, cppcheck, odr, scripts, headers, diagnostic-controls, context, then contracts. `native_sources.json` only scans `.cpp/.cc/.cxx/.C`, so `.mm` and `.cu` are compiled under policy but not tidy-swept. Headers are swept through the TUs that reach them.
- Suppression precedent:
  - Scoped `#pragma clang diagnostic push / ignored / pop` under `#if defined(__clang__)`: `native/src/platform/argv.hpp`, `native/src/engine_abi_312.hpp`, `native/src/tools/check_reports.cpp:482`.
  - Scoped `NOLINT(<check>) <reason>`: `check_reports.cpp`.
  - Register: the table `| Checker | Narrow origin and reason | Reproducer | Remove when |` in `docs/TESTING.md` (the copy at ~line 1059).
  - Rule (native-engineering.md): "Record a suppression with checker, origin, reason, reproducer, and removal condition."

## Global Constraints

- Never weaken `bike_apply_native_policy`, `native/.clang-tidy`, or sweep configuration, and never add `-Wno-*` to GPU targets.
- Suppressions are allowed only in the shared kernel headers (`common.hpp`, `suspension_kernel.hpp`, `drivetrain_resistance_kernel.hpp`, `rider_kernel.hpp`, `tire_kernel.hpp`, `batch_step.hpp`, `cuda_kernels.cu`), and only for this closed list:
  - `-Wunsafe-buffer-usage`
  - `cppcoreguidelines-pro-bounds-pointer-arithmetic`
  - `cppcoreguidelines-pro-bounds-array-to-pointer-decay`
  - `cppcoreguidelines-avoid-c-arrays`
  - `modernize-avoid-c-arrays`
  - `bugprone-easily-swappable-parameters`

  Anything else needs an orchestrator decision before it is suppressed.
- Kernel headers stay MSL- and CUDA-compatible:
  - Use `BIKE_HD`, `BIKE_THREAD`, `BIKE_DEVICE` and `BIKE_CONSTEXPR`, and the `gpu_*` math helpers.
  - No `std::` and no new includes inside code reachable from MSL. Host-only blocks must be guarded by `#if !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)`.
  - After each kernel-header task, the Metal shader must still compile at runtime: `bash .work/gpu-compliance/build_unit.sh batch_runners <outdir>`, test section `test_metal_shader_and_buffer_reuse` passes.
- Numerics must not change. Fixes are casts, types and restructuring only.
  - For valid inputs the FP32 operation order is preserved (`-ffp-contract=off` is the project policy).
  - Every kernel-header task proves this: the GPU unit tests stay green, and a host-vs-Metal trajectory check (Task 20) shows the same `max|Δqpos|`.
- Every task:
  - Runs its own files' inventory before and after (`compile_tu.sh`, `tidy_tu.sh`, Task 1), and the after count must be 0.
  - Edits only the files listed in that task.
  - Uses no state-changing git commands.
  - Never runs a ninja or CMake build in `build/native-release`. Tasks 1, 20, 21 and 22 are the exceptions; there the orchestrator owns the build dir.
- Python always runs through `uv`.
- Do not commit. Each task ends with a checkpoint, not a commit.

---

## File Map

| File | Responsibility | Task |
|---|---|---|
| `.work/gpu-compliance/compile_tu.sh` (new, gitignored) | Re-runs one TU's exact policy compile command with `-fsyntax-only -ferror-limit=0` and filters by file | 1 |
| `.work/gpu-compliance/tidy_tu.sh` (new, gitignored) | Runs clang-tidy with the project config on one TU and filters by file | 1 |
| `.work/gpu-compliance/build_unit.sh` (new, gitignored) | Ad-hoc behaviour build and run of one GPU test (-Wall -Wextra -Werror) | 1 |
| `.work/gpu-compliance/baseline/*.txt` | Per-file diagnostic and tidy inventory | 1 |
| `native/src/gpu/common.hpp` | Suppression macros, union removal, fixes | 2 |
| `native/src/gpu/suspension_kernel.hpp` | Kernel fixes and registered suppression | 3 |
| `native/src/gpu/drivetrain_resistance_kernel.hpp` | same | 4 |
| `native/src/gpu/rider_kernel.hpp` | same | 5 |
| `native/src/gpu/tire_kernel.hpp` | same | 6 |
| `native/src/gpu/batch_step.hpp` | same | 7 |
| `native/src/gpu/cuda_kernels.cu` | same | 8 |
| `native/src/gpu/model_assets.hpp` | Host-only, real fixes (no suppression) | 9 |
| `native/tests/gpu/test_gpu_{suspension,drivetrain_resistance,rider,tire,batch_step,model_geometry,model_assets}.cpp` | Port to the project test style, real fixes | 10–16 |
| `native/src/gpu/cpu_batch_runner.cpp` | Host, real fixes | 17 |
| `native/src/gpu/metal_batch_runner.mm` | Host ObjC++, real fixes | 18 |
| `native/tests/gpu/test_gpu_batch_runners.cpp` | Port to the test style, plus physical assertions for the captured start | 19 |
| `docs/TESTING.md` | Suppression register rows | 2 |

---

### Task 1: Inventory tooling and baseline

**Files:**
- Create: `.work/gpu-compliance/compile_tu.sh`, `.work/gpu-compliance/tidy_tu.sh`, `.work/gpu-compliance/build_unit.sh`, `.work/gpu-compliance/baseline/`

**Interfaces:**
- Produces:
  - `compile_tu.sh <repo-relative-TU> [repo-relative-file-filter]` prints policy diagnostics `path:line:col: error|warning: msg [flag]` and ends with the line `COUNT <n>`. The exit code is 0 even when diagnostics are found.
  - `tidy_tu.sh <TU> [filter]` uses the same output format.
  - `build_unit.sh <name|batch_runners|cuda_syntax> <outdir>` builds and runs one test.

- [ ] **Step 1: Configure (orchestrator only).** Run `cd $R && cmake --preset release`. Expected: configure OK, and `ctest --test-dir build/native-release -N -L gpu` lists 8 tests.

- [ ] **Step 2: Write `compile_tu.sh`.**

```bash
#!/usr/bin/env bash
# compile_tu.sh <repo-relative TU> [repo-relative file filter]
# Re-runs the exact policy compile command of one TU from compile_commands.json
# with -fsyntax-only -ferror-limit=0 (no object, no ninja, safe to run in parallel).
set -euo pipefail
R="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
tu="$R/$1"; filter="${2:-}"
meta=()  # [0]=directory, [1]=command; read loop instead of mapfile (bash 3.2 safe)
while IFS= read -r line; do meta+=("$line"); done < <(uv run python - "$R/build/native-release/compile_commands.json" "$tu" <<'EOF'
import json, shlex, sys
db, tu = sys.argv[1], sys.argv[2]
for e in json.load(open(db)):
    if e["file"] == tu:
        args = e.get("arguments") or shlex.split(e["command"])
        out, skip = [], False
        for a in args:
            if skip: skip = False; continue
            if a == "-o": skip = True; continue
            out.append(a)
        print(e["directory"]); print(shlex.join(out + ["-fsyntax-only", "-ferror-limit=0"]))
        break
else:
    sys.exit(f"no compile command for {tu}")
EOF
)
cd "${meta[0]}"
log="$(eval "${meta[1]}" 2>&1 || true)"
lines="$(printf '%s\n' "$log" | grep -E '^[^ ].*:[0-9]+:[0-9]+: (error|warning): ' | sed "s|^$R/||" | sort -u || true)"
if [[ -n "$filter" ]]; then lines="$(printf '%s\n' "$lines" | grep -F "$filter:" || true)"; fi
[[ -n "$lines" ]] && printf '%s\n' "$lines"
echo "COUNT $( [[ -n "$lines" ]] && printf '%s\n' "$lines" | wc -l | tr -d ' ' || echo 0)"
```

- [ ] **Step 3: Write `tidy_tu.sh`.**

```bash
#!/usr/bin/env bash
# tidy_tu.sh <repo-relative TU> [repo-relative file filter]
set -euo pipefail
R="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
tu="$R/$1"; filter="${2:-}"
log="$(/opt/homebrew/opt/llvm/bin/clang-tidy -p "$R/build/native-release" \
        --config-file="$R/native/.clang-tidy" "$tu" 2>/dev/null || true)"
lines="$(printf '%s\n' "$log" | grep -E '^/.*:[0-9]+:[0-9]+: (error|warning): .*\[[a-z].*\]$' | sed "s|^$R/||" | sort -u || true)"
if [[ -n "$filter" ]]; then lines="$(printf '%s\n' "$lines" | grep -F "$filter:" || true)"; fi
[[ -n "$lines" ]] && printf '%s\n' "$lines"
echo "COUNT $( [[ -n "$lines" ]] && printf '%s\n' "$lines" | wc -l | tr -d ' ' || echo 0)"
```

- [ ] **Step 4: Copy the behaviour build script.** `cp /tmp/bike_orch/build_unit.sh .work/gpu-compliance/build_unit.sh` (the content is fixed: ad-hoc `c++ -Wall -Wextra -Werror` build plus run, with the modes `<name>`, `batch_runners` and `cuda_syntax`). Then `chmod +x .work/gpu-compliance/*.sh`.

- [ ] **Step 5: Baseline.** For each TU in `native/tests/gpu/*.cpp` plus `native/src/gpu/cpu_batch_runner.cpp` and `native/src/gpu/metal_batch_runner.mm`, run `compile_tu.sh` and save the output to `.work/gpu-compliance/baseline/compile/<basename>.txt`. For each `.cpp` TU, run `tidy_tu.sh` and save the output to `baseline/tidy/<basename>.txt`. Then derive per-file lists, one file per source or header, by splitting on the path prefix and de-duplicating. Header lists come from all TUs. Example: `cat baseline/compile/*.txt | grep -F 'native/src/gpu/common.hpp:' | sort -u > baseline/files/common.hpp.compile.txt`.

  Expected: totals close to the Context numbers. The runner files and `test_gpu_batch_runners.cpp` get their first counts here.

- [ ] **Step 6: Checkpoint.** Write the per-file counts table to `.work/gpu-compliance/baseline/SUMMARY.md`. No commit.

---

### Task 2: `common.hpp`: suppression macros, union removal, fixes

**Files:**
- Modify: `native/src/gpu/common.hpp`
- Modify: `native/src/gpu/metal_batch_runner.mm`. This is a mechanical rename only: `frame_bid` becomes `frame_body`, `front_wheel_bid` becomes `front_wheel_body`, `rear_wheel_bid` becomes `rear_wheel_body`.
- Modify: `docs/TESTING.md` (register rows)
- Check: `tools/colab_gpu/build_cuda_writers.py`. Read-only: confirm ctypes field names/order still match. Its `frame_bid` is a Python property and stays.

**Interfaces:**
- Produces, used by Tasks 3–8:

```cpp
// common.hpp, after the BIKE_* address-space macros
#if defined(__clang__) && !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
#define BIKE_GPU_SHARED_CODE_BEGIN \
    _Pragma("clang diagnostic push") \
    _Pragma("clang diagnostic ignored \"-Wunsafe-buffer-usage\"")
#define BIKE_GPU_SHARED_CODE_END _Pragma("clang diagnostic pop")
#else
#define BIKE_GPU_SHARED_CODE_BEGIN
#define BIKE_GPU_SHARED_CODE_END
#endif
```

  - The per-file clang-tidy region (comments; NOLINT cannot live in a macro):

```cpp
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN
...namespace bike::gpu { ... }...
BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
```

  - `GpuModelIndices` has no unions: plain `int frame_body = 1; ... int front_wheel_body = 7; ... int rear_wheel_body = 10;`. The layout and `static_assert(sizeof(GpuModelIndices) == 228)` are unchanged.

- [ ] **Step 1: Baseline.**
  - `bash .work/gpu-compliance/compile_tu.sh native/tests/gpu/test_gpu_batch_step.cpp native/src/gpu/common.hpp`. Expected: `COUNT 14`, or the Task 1 number.
  - `bash .work/gpu-compliance/tidy_tu.sh native/tests/gpu/test_gpu_batch_step.cpp native/src/gpu/common.hpp`. Record the count.

- [ ] **Step 2: Grep the alias users.** Run `grep -rn "_bid\b" native/src/gpu native/tests/gpu tools/colab_gpu`. Expected users are `metal_batch_runner.mm` (C++ field access) and `build_cuda_writers.py` (a Python property, which stays).

- [ ] **Step 3: Implement.**
  - Add the macro pair above.
  - Remove the three unions and rename the `.mm` uses.
  - Wrap the whole `namespace bike::gpu {...}` body in the NOLINT region and the `BIKE_GPU_SHARED_CODE_*` pair. They go inside the include guard, after the macro definitions; the macros must be defined before use.
  - Fix every remaining non-listed diagnostic for real:
    - `-Wsign-conversion`: `static_cast<std::size_t>` is not available in MSL, so make index types consistently `int`, or cast at the boundary on the host side.
    - `-Wdouble-promotion`: use `f`-suffixed literals.
    - `misc-const-correctness`: add `const`.
    - `modernize-use-designated-initializers`: use designated initializers `GpuVec2{.x = a, .y = b}`. MSL accepts C++14 aggregate init only, so if the MSL compile fails with designated initializers, keep positional init and add `modernize-use-designated-initializers` to the region **only after asking the orchestrator**.

- [ ] **Step 4: Add register rows.** Add these rows to the `docs/TESTING.md` table at ~line 1059:

```markdown
| `-Wunsafe-buffer-usage`, `cppcoreguidelines-pro-bounds-pointer-arithmetic`, `cppcoreguidelines-pro-bounds-array-to-pointer-decay` | `native/src/gpu/{common,suspension_kernel,drivetrain_resistance_kernel,rider_kernel,tire_kernel,batch_step}.hpp`, `cuda_kernels.cu`: the same FP32 kernel source compiles as host C++, CUDA and Metal MSL; device buffers are address-space-qualified raw pointers (`BIKE_THREAD`/`BIKE_DEVICE`) and MSL has no `std::span`. Host-only code (`model_assets.hpp`, runners, tests) carries no suppression. | `bike_test_gpu_*` CTest nodes; Metal runtime shader compile in `bike_test_gpu_batch_runners` | The kernels are split into a host span-based front and a device-only body, or MSL gains a span-equivalent the headers can use. |
| `cppcoreguidelines-avoid-c-arrays`, `modernize-avoid-c-arrays` | Same files: POD structs (`GpuModelIndices`, `GpuRiderParams`, …) are bit-copied into Metal/CUDA buffers and mirrored by Python ctypes (`tools/colab_gpu/build_cuda_writers.py`); `std::array` is unavailable in MSL. | `test_gpu_batch_step` sizeof checks; `static_assert(sizeof(GpuModelIndices) == 228)` | Same as above. |
| `bugprone-easily-swappable-parameters` | Same files: positional float signatures are the shared CUDA/MSL kernel ABI; call sites are covered by per-kernel unit tests with distinct values per argument. | `bike_test_gpu_{suspension,tire,drivetrain_resistance,rider,batch_step}` | Kernel entry points take parameter structs. |
```

- [ ] **Step 5: Verify.**
  - `compile_tu.sh … common.hpp` reports `COUNT 0`.
  - `tidy_tu.sh … common.hpp` reports `COUNT 0`.
  - `bash .work/gpu-compliance/build_unit.sh model_geometry /tmp/gc/T2`, `… tire …`, `… batch_step …`, `… cuda_syntax …` and `… batch_runners …` all end with PASS. For batch_runners: the Metal shader section passes; a later failure at the `root_x > 2.65` assertion is known until Task 19.

- [ ] **Step 6: Checkpoint.** Report a diff summary, the before and after counts, and the test outputs.

---

### Tasks 3–8: one shared kernel file each

The tasks run in parallel after Task 2. Each task has the same shape. Only the file and the covering TU differ:

| Task | File | Covering TU for `compile_tu.sh` / `tidy_tu.sh` | Behaviour tests (`build_unit.sh`) |
|---|---|---|---|
| 3 | `native/src/gpu/suspension_kernel.hpp` | `native/tests/gpu/test_gpu_suspension.cpp` | suspension, batch_step, cuda_syntax, batch_runners |
| 4 | `native/src/gpu/drivetrain_resistance_kernel.hpp` | `native/tests/gpu/test_gpu_drivetrain_resistance.cpp` | drivetrain_resistance, batch_step, cuda_syntax, batch_runners |
| 5 | `native/src/gpu/rider_kernel.hpp` | `native/tests/gpu/test_gpu_rider.cpp` | rider, batch_step, cuda_syntax, batch_runners |
| 6 | `native/src/gpu/tire_kernel.hpp` | `native/tests/gpu/test_gpu_tire.cpp` | tire, model_geometry, batch_step, cuda_syntax, batch_runners |
| 7 | `native/src/gpu/batch_step.hpp` | `native/tests/gpu/test_gpu_batch_step.cpp` | batch_step, cuda_syntax, batch_runners |
| 8 | `native/src/gpu/cuda_kernels.cu` | `native/tests/gpu/test_gpu_batch_step.cpp` (it `#include`s the .cu) | batch_step, cuda_syntax |

**Files:** Modify only the file in the row.

**Interfaces:**
- Consumes `BIKE_GPU_SHARED_CODE_BEGIN/END` and the exact NOLINTBEGIN/NOLINTEND check list from Task 2.
- Produces no API change. Function signatures, struct layouts and numerics are unchanged.

- [ ] **Step 1: Baseline.** `bash .work/gpu-compliance/compile_tu.sh <TU> <file>` and `bash .work/gpu-compliance/tidy_tu.sh <TU> <file>`. Record both counts.
- [ ] **Step 2: Add the region.** Wrap the file's `namespace bike::gpu {…}` body (for the `.cu`, the kernel and host-wrapper bodies) in the Task 2 NOLINT region and the `BIKE_GPU_SHARED_CODE_BEGIN/END` pair, copying the check list verbatim from Task 2. Re-run both scripts. Only non-listed diagnostics may remain.
- [ ] **Step 3: Fix the rest for real.**
  - `-Wsign-conversion` / `-Wshorten-64-to-32`: consistent `int` indices; explicit `static_cast` on host-only paths.
  - `-Wdouble-promotion`: `f` literals, `gpu_*` float helpers.
  - `-Wunused-*`: remove the variable.
  - `misc-const-correctness`: add `const`.
  - `modernize-use-designated-initializers`: see the Task 2 MSL note.
  - `cppcoreguidelines-pro-type-member-init`: initialise the member.
  - Any other check: ask the orchestrator before suppressing.
- [ ] **Step 4: Verify.** Both scripts must report `COUNT 0` for the file. All behaviour tests in the row end with PASS; the batch_runners exception is the known `root_x > 2.65` failure until Task 19, but the Metal shader section must pass.
- [ ] **Step 5: Checkpoint.** Report a diff summary, the counts and the outputs.

---

### Task 9: `model_assets.hpp` (host-only, no suppression)

**Files:** Modify `native/src/gpu/model_assets.hpp`.

**Interfaces:** The public API in namespace `bike::gpu::assets` is unchanged: `AssetStatus`, `asset_status_name`, `asset_dir`, `load_hfield_bin`, `Heightfield1D`, `extract_heightfield_1d`, `load_captured_initial_state`. The statuses returned for the same inputs are unchanged.

- [ ] **Step 1: Baseline.** `compile_tu.sh native/tests/gpu/test_gpu_model_assets.cpp native/src/gpu/model_assets.hpp` (expected ~19) and `tidy_tu.sh` on the same arguments.
- [ ] **Step 2: Fix.**
  - Raw `mjModel` arrays: wrap each access range once in a `std::span` built from the model counts. Example: `const std::span<const float> data(m->hfield_data, static_cast<std::size_t>(m->nhfielddata));`. A span built from a raw pointer is `-Wunsafe-buffer-usage-in-container`. Put that single construction in one helper, `model_span(ptr, count)`, with a scoped `#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"`, exactly like `native/src/platform/argv.hpp`, and add a register row naming the helper.
  - `strlen`/`strtod` libc calls: replace with `std::string_view` scanning and `std::from_chars` for `double`. Keep `strtod` only if `from_chars` for double is unavailable in the toolchain; in that case report to the orchestrator.
  - Byte reads: `std::ifstream::read` into `std::vector<double>`, through `std::as_writable_bytes(std::span(vec))`.
- [ ] **Step 3: Verify.**
  - Both scripts report `COUNT 0` for the file.
  - `bash .work/gpu-compliance/build_unit.sh model_assets /tmp/gc/T9` passes.
  - Bitwise check: run `uv run --with numpy==2.5.2 --with mujoco==3.12.0 python tools/colab_gpu/test_model_assets.py` (14/14). Spot-check that the C++ captured values are still bitwise equal to Python's (the test already does this).
- [ ] **Step 4: Checkpoint.**

---

### Tasks 10–16: one GPU test file each, ported to the project test style

| Task | File |
|---|---|
| 10 | `native/tests/gpu/test_gpu_suspension.cpp` |
| 11 | `native/tests/gpu/test_gpu_drivetrain_resistance.cpp` |
| 12 | `native/tests/gpu/test_gpu_rider.cpp` |
| 13 | `native/tests/gpu/test_gpu_tire.cpp` |
| 14 | `native/tests/gpu/test_gpu_batch_step.cpp` |
| 15 | `native/tests/gpu/test_gpu_model_geometry.cpp` |
| 16 | `native/tests/gpu/test_gpu_model_assets.cpp` |

These run in parallel. They may start before Tasks 3–8 finish, because each task filters diagnostics to its own file.

**Files:** Modify only the file in the row.

**Interfaces:** CTest name, exit code (0 pass, 1 fail, 2 usage) and the set of checked behaviours are unchanged. **Every existing assertion is kept, with the same expected values and tolerances.** Only the harness and types change.

- [ ] **Step 1: Baseline.** `compile_tu.sh <file> <file>` and `tidy_tu.sh <file> <file>`. Also record the PASS output of `build_unit.sh <name> /tmp/gc/T<n>`, together with the number of checks if the file prints one.
- [ ] **Step 2: Replace the harness.** Remove the `REQUIRE`/`check_*` macros, `fprintf`, `do { } while (0)` and `std::exit`. Use this skeleton, copied from `native/tests/test_station.cpp`:

```cpp
#include "../../src/platform/argv.hpp"
#include <array>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
    using TestFunction = void (*)();
    struct TestCase { std::string_view name; TestFunction run; };

    void require(bool condition, std::string_view message) {
        if (!condition) throw std::runtime_error(std::string(message));
    }
    // numeric helper used instead of printf-style messages
    void require_near(double actual, double expected, double tol, std::string_view what) {
        if (!(std::abs(actual - expected) <= tol))
            throw std::runtime_error(std::string(what) + ": actual=" + std::to_string(actual) +
                                     " expected=" + std::to_string(expected) +
                                     " tol=" + std::to_string(tol));
    }

    // ... existing test functions, unchanged in logic, now calling require/require_near ...

    constexpr std::array<TestCase, N> cases{{
        {.name = "<existing_function_name>", .run = <existing_function_name>},
        // one entry per existing test function, same order as the old main()
    }};

    int run_case(const TestCase &test_case) {
        try { test_case.run(); std::cout << "PASS " << test_case.name << '\n'; return 0; }
        catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n'; return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
    const std::span<char *> arguments = bike::platform::main_arguments(argc, argv);
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--list") {
        for (const TestCase &c : cases) std::cout << c.name << '\n';
        return 0;
    }
    if (arguments.size() > 2) { std::cerr << "usage: <test> [case-name|--list]\n"; return 2; }
    const std::string_view requested = arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all"; int status = 0;
    for (const TestCase &c : cases)
        if (requested == "all" || requested == c.name) { found = true; if (run_case(c) != 0) status = 1; }
    if (!found) { std::cerr << "unknown case: " << requested << '\n'; return 2; }
    return status;
}
```

  `test_gpu_model_geometry.cpp` keeps its optional `argv[1]` model path. Accept a path argument when it is not a case name: if `argv[1]` ends with `.xml`, treat it as the model path. Document this in the usage line.
- [ ] **Step 3: Replace C arrays and raw-pointer test buffers.** Use `std::array<float, kMaxNq>` and similar containers. Pass `.data()` to kernel APIs, since a container's `.data()` is not flagged. Do pointer arithmetic over rows only through `std::span::subspan`. Fix the sign, double-promotion and unused-variable diagnostics for real. Replace union access with the renamed fields from Task 2.
- [ ] **Step 4: Verify.**
  - `compile_tu.sh <file> <file>` and `tidy_tu.sh <file> <file>` both report `COUNT 0`.
  - `build_unit.sh <name> /tmp/gc/T<n>` passes with one `PASS <case>` line per case and exit 0.
  - The number of executed assertions is at least the baseline. If you count assertions, they are the `require` calls reached.
  - Mutation spot check: temporarily flip one expected value in a scratch copy under `/tmp/gc/T<n>/` (never in the repo) and show that the test fails with the case name.
- [ ] **Step 5: Checkpoint.**

---

### Task 17: `cpu_batch_runner.cpp` (host)

**Files:** Modify `native/src/gpu/cpu_batch_runner.cpp`.

**Interfaces:** The C ABI is unchanged: signatures of `bike_cpu_run_batch_ex`, `bike_cpu_run_batch`, `bike_cpu_run_batch_sweep`, and return codes -1, -2, -5, -6. Outputs for the same inputs stay bitwise identical.

- [ ] **Step 1: Baseline.**
  - `compile_tu.sh native/src/gpu/cpu_batch_runner.cpp native/src/gpu/cpu_batch_runner.cpp` and `tidy_tu.sh` on the same arguments.
  - Bit-identity reference: build the current runner with `build_unit.sh batch_runners /tmp/gc/T17-before`. With a harness under `/tmp/gc/T17/`, call `bike_cpu_run_batch_sweep` (nworld 4, 400 steps, null init and null heights, real model) and dump fork, shock, root_x and root_z bytes.
- [ ] **Step 2: Fix.**
  - At the C ABI boundary, wrap each incoming pointer once into a `std::span` sized by `nworld` and the model counts. Construct each span inside one helper, using the same pattern as Task 9: a scoped `-Wunsafe-buffer-usage-in-container` pragma plus a register row naming the helper.
  - Use spans everywhere else. Fix the sign and shorten diagnostics with explicit casts.
  - Replace `fprintf(stderr, …)` with `std::cerr <<`.
- [ ] **Step 3: Verify.**
  - Both scripts report `COUNT 0`.
  - The harness dump after the change is byte-identical to the dump before.
  - The error-path harness still returns -6 (no hfield .bin), -5 (no captured JSON with null init), and 0 (explicit init).
- [ ] **Step 4: Checkpoint.**

---

### Task 18: `metal_batch_runner.mm` (host ObjC++)

**Files:** Modify `native/src/gpu/metal_batch_runner.mm`.

**Interfaces:** The C ABI is unchanged. Return-code tables are documented in the file (launch functions 0/-1/-2/-3/-4; hybrid -1…-8). The MSL header list and the shader source are unchanged.

- [ ] **Step 1: Baseline.** Run `compile_tu.sh native/src/gpu/metal_batch_runner.mm native/src/gpu/metal_batch_runner.mm`; `.mm` is not tidy-swept. Record a reference dump with a harness under `/tmp/gc/T18/`: `bike_metal_run_hybrid_sweep` with nworld 4 and 128, 400 steps, null init, real model, recording root_x and fork/shock bytes plus `max|Metal−CPU|`.
- [ ] **Step 2: Fix.** Use the same recipe as Task 17. `[buf contents]` returns `void*`; convert it once per buffer, through the helper, into a typed `std::span`. Objective-C message sends are not flagged. If `-Wunsafe-buffer-usage` diagnostics appear inside the MSL source string, they are a false positive and string content is not compiled by the host. Report them instead of suppressing.
- [ ] **Step 3: Verify.**
  - `COUNT 0`.
  - Dumps are byte-identical to the baseline, and `max|Metal−CPU|` is unchanged.
  - `build_unit.sh batch_runners /tmp/gc/T18`: the shader, launch and argument-validation sections pass.
- [ ] **Step 4: Checkpoint.**

---

### Task 19: `test_gpu_batch_runners.cpp`: port plus physical assertions

**Files:** Modify `native/tests/gpu/test_gpu_batch_runners.cpp`.

**Interfaces:** It consumes the runner C ABI (Tasks 17 and 18). Launch functions return `int`.

- [ ] **Step 1: Baseline.** `compile_tu.sh` and `tidy_tu.sh` on the file. Run `build_unit.sh batch_runners /tmp/gc/T19`. Known failure: `cpu_root_x_m[...] > 2.65f`. That assertion was derived from an invented 3.5 m/s start; the runners now start from `captured_bootstrap.json` (vx ≈ 0.058 m/s).
- [ ] **Step 2: Measure.** Use a harness under `/tmp/gc/T19/` with nworld 4, 400 steps (0.2 s) and null init. Current numbers: root_x 2.055095 m, fork peak 59.88 mm, shock peak 21.35 mm, Metal−CPU 0 m. Also record x0 = captured qpos[0] = 2.0163204663006566.
- [ ] **Step 3: Replace the physical assertions with ones derived from the captured start.** Every new constant gets a one-line comment with its derivation:
  - `root_x > x0 + 0.01` (the bike moves forward under pedal_demand 0.65 in 0.2 s; measured +0.039 m);
  - `root_x < x0 + 0.5` (an upper bound from max plausible speed × 0.2 s);
  - fork and shock peaks inside the existing physically motivated bands;
  - Metal−CPU parity `< 1e-2 m` (kept as is).

  Port the harness to the Tasks 10–16 skeleton.
- [ ] **Step 4: Verify.** Both scripts report `COUNT 0`. `build_unit.sh batch_runners /tmp/gc/T19` passes completely, with exit 0.
- [ ] **Step 5: Checkpoint.**

---

### Task 20: Integration build, CTest and trajectory parity (orchestrator)

**Files:** none. If a build fails because of a file, it goes back to that file's task.

- [ ] **Step 1: Build.** `cmake --build build/native-release --target bike_test_gpu_suspension bike_test_gpu_tire bike_test_gpu_drivetrain_resistance bike_test_gpu_rider bike_test_gpu_batch_step bike_test_gpu_model_geometry bike_test_gpu_model_assets bike_test_gpu_batch_runners`. Expected: no diagnostics.
- [ ] **Step 2: CTest.** `ctest --test-dir build/native-release -L gpu --output-on-failure`. Expected: 8/8 passed.
- [ ] **Step 3: Trajectory parity.** Run a 5 s host-vs-Metal trajectory, comparing qpos per step (harness under `/tmp/gc/T20/`). Expected: max|Δqpos| is of the same order as before the compliance work (8.6e-6 was measured earlier with the 3.5 m/s start, so re-baseline with the captured start before Task 2 if possible). Record crank cadence: under the new taper (105–120 rpm) it must stay ≤ 120 rpm at steady state.
- [ ] **Step 4: Python sweep smoke test.** Run `uv run … python tools/colab_gpu/metal_hybrid_sweep.py` with the smallest configuration. Expected: rc 0 and all values finite.

---

### Task 21: Sweeps (orchestrator dispatches per-file fixes)

**Files:** none directly. Each finding goes back to the owning file's task.

- [ ] **Step 1: Run each sweep standalone, in gate order.** `build/native-release/bin/bike-native-checks --kind <k> --build build/native-release` for k in `frontends tidy analyzer cppcheck odr scripts headers diagnostic-controls context`, then `--kind contracts --repo-root $R`.
- [ ] **Step 2: Dispatch.** For each finding, open a follow-up for the owning file's task. Use the same recipes and the same suppression rules; anything outside the closed list requires an orchestrator decision plus a register row.
  - `headers`: every first-party header must compile standalone with its own includes. `model_assets.hpp` and the kernel headers may need missing `<cstddef>`/`<cstdint>` includes, guarded for MSL.
  - `odr`: check that the tests' direct `#include` of `station.cpp`/`freehub.cpp`/`spindle_math.cpp` does not create duplicate definitions inside any one link.
- [ ] **Step 3: Re-run.** Repeat until all sweeps pass.

---

### Task 22: Gate (orchestrator)

- [ ] **Step 1:** Run `bash tools/run_tests.sh native`. Expected: exit 0. Compare any failure with the documented baseline list before re-dispatching.
- [ ] **Step 2:** Run `bash tools/run_tests.sh full`. A physics change needs both `native` and `full`, per native-engineering.md. Expected: exit 0.
- [ ] **Step 3: Final report.**
  - Commands, compiler and LLVM versions, counts (diagnostics before and after per file), suppression rows added.
  - Behaviour evidence (Tasks 19 and 20).
  - Remaining scope limits: the ASan preset has no MuJoCo compat header in the root lane (`native/cmake/BikeDependencies.cmake` ~195–216), so `native asan` was not verified. Linux was not verified. The CUDA path is only host-syntax checked.

---

## Self-review notes

- Coverage:
  - The 573 policy diagnostics are covered by Tasks 2–16. The runner and batch_runners test diagnostics are covered by Tasks 17–19.
  - Sweep findings: Task 21. Gate: Task 22.
  - The known physical-assertion failure is covered by Task 19.
- Parallelism:
  - Task 1 runs first.
  - Task 2 runs before Tasks 3–8 and before 18 (the `.mm` rename).
  - Tasks 3–8, 9, 10–16 and 17 can run in parallel; each edits disjoint files.
  - Task 18 runs after Task 2. Task 19 runs after Tasks 17 and 18. Then Tasks 20, 21 and 22.
- Names used across tasks: `BIKE_GPU_SHARED_CODE_BEGIN` / `BIKE_GPU_SHARED_CODE_END`, the NOLINT check list (copied verbatim), the `frame_body`/`front_wheel_body`/`rear_wheel_body` fields, and `.work/gpu-compliance/{compile_tu,tidy_tu,build_unit}.sh`.
