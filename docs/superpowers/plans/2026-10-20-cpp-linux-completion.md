# C++ + Linux Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `impl/python-free-cpp` build, check and test on macOS arm64 **and** Linux aarch64, then finish the Python→C++ migration through deletion of every first-party Python file (T52–T63).

**Architecture:** Four phases separated by short verification checkpoints.
1. **Phase 1:** restore the macOS gate (the checker accepts the merged schema-2 build context) and make the binding graph (`cmake -S native`) compile on Linux.
2. **Phase 2:** move the Python-free root graph (`cmake -S .`) onto the platform-aware `Bike*` CMake layer, with per-platform dependencies, so it builds on Linux.
3. **Phase 3:** port the pytest suites to CTest against frozen golden data captured from the Python oracle. Replace Python tooling with CMake, CTest, CPack and three small C++ tools.
4. **Phase 4:** qualify, then delete Python and the binding graph.

**Tech Stack:** C++23 (Clang 18 / GCC 13 on Linux, AppleClang on macOS), CMake ≥ 3.27 + Ninja, CTest, CPack, MuJoCo 3.12.0, DuckDB 1.5.6, toml++ 3.4.0, Eigen 3.4.0, GLFW 3.4, NumPy 2.5.2 (oracle only, until Phase 4), Docker Desktop `linux/arm64` (Ubuntu 24.04) for the Linux lane.

**Spec (read before any task):**
- Decisions agreed with the user on 2026-10-20 (the interview record is reproduced in "Decision record" below; it is authoritative).
- `docs/superpowers/plans/2026-10-14-python-free-cpp-master.md` and the subsystem plans `…-00` to `…-09`. They give the original per-task contracts. Where this plan deviates, the deviation is stated inline and **this plan wins**.
- `docs/superpowers/plans/2026-10-14-python-free-cpp-08-tests-tools-packaging.md`: the per-file C++ test target paths for T52/T53.
- `docs/agents/native-engineering.md`: mandatory for every task that touches `native/`.
- `migration/migration-verified-state-2026-10-19.md`: the last verified ledger state. Statuses there predate this plan.

---

## Decision record (2026-10-20 interview — binding)

| # | Decision |
|---|---|
| D1 | Scope: everything through T63 (Python deletion). Four phases; few phases, but every task fits one fresh subagent session. |
| D2 | Verification ladder L1 / L2 / L3 (see "Verification ladder"). L3 (full native gate) only at the end of Phase 1 (macOS), at the end of Phase 2 (Linux; **deviation**, see below) and at the end of Phase 4 (both). |
| D3 | Tests may be run by the implementing agents (targeted at L1/L2, full at L3). |
| D4 | Linux = **aarch64 container only**. x86_64 becomes optional/not advertised (D10). |
| D5 | The Linux lane is a committed recipe (`tools/linux/Dockerfile` + `tools/linux/container.sh`). The tree is copied into the container. Never bind-mount the host `.venv`. |
| D6 | Work directly on `impl/python-free-cpp`, one commit per task in the existing style (`fix(cpp): …`, `build(native): …`, `test(cpp): …`, `docs(migration): …`). Push (`git push origin impl/python-free-cpp`) only after a phase's final checkpoint passes. Never merge into `main2`/`main`. |
| D7 | The C++ checker learns schema 2 with the same semantics as `tools/native_checks.py`. Do not revert the producer. |
| D8 | `long double` in `native/src/numerics/linear_feasibility.cpp` becomes `double`. The oracle is `scipy.optimize.linprog`, and on macOS arm64 `long double == double`, so macOS is bit-identical. |
| D9 | Linux OpenBLAS for the Python-free graph: the pinned `libscipy_openblas64_-bb25d554.so` from the NumPy 2.5.2 cp313 manylinux aarch64 wheel, extracted into `.deps`. Never the system OpenBLAS. |
| D10 | Release qualification: Linux aarch64 is the required Linux lane; x86_64 is "not qualified / not advertised". |
| D11 | GUI: Linux gets a build with GLFW/X11 plus an Xvfb smoke run. This is **not** acceptance; Xvfb rows never close `viewer-platform-acceptance.md`. macOS gets a manual checklist run by the user once, in Phase 4. |
| D12 | ADR-0003: bitwise parity with Python stays the gate for a test file until its golden data is captured on both platforms; afterwards the C++ test against the goldens is the gate, and C++ is authoritative. Python is deleted at T63. This amends ADR-0002, whose item "Python stays" is overruled. |
| D13 | Merge `feat/drop-oracle-parity` (ADR-0002 + `main`'s r2 research) into this branch. `research/fp32-physics` stays out. |
| D14 | Each of the 146 + 9 pytest files is triaged into **port** (rewrite as C++ test), **golden** (compare C++ output to frozen Python values) or **drop** (Python-only internals; a written reason is mandatory). |
| D15 | Golden data is captured **per porting task** on macOS and in the Linux container. One completeness check runs before T63. |
| D16 | Every documented baseline failure (24 macOS + 3 Linux) gets its own root-cause task. Porting a known-red test as "expected to fail" is forbidden. If the cause is physics, the task stops and records a written decision with the user. |
| D17 | Phases 1, 2 and 4 run sequentially in this worktree. Phase 3 may use up to 3 parallel worktrees, each with its own build dirs and its own container. L2 runs in this worktree after each merge-back. |
| D18 | Python tools are not ported 1:1. Ported to C++: `golden_episode` + `episode_compare` (as `bike-recording golden …` subcommands) and `measure_realtime` (`bike-measure-realtime`). Deleted: Python-environment/oracle tools. Replaced by CMakePresets / CTest / CPack / shell: `dev.py`, coverage, offline bundles, the `run_tests.sh` internals. Replaced by CI artifacts + a Markdown checklist: `ci_portability.py`, `portability_qualification.py`. **Deviation from plan-08 T55/T57/T59**: no `bike-test-runner`, `bike-install`, `bike-package-verify`, `coverage_main`, `validate`-style tool zoo beyond what is listed here. |
| D19 | Do not delete or prune worktrees or old containers. |
| D20 | Distribution: per-platform relocatable `tar.gz` (`bin/`, `lib/` with `$ORIGIN`/`@loader_path` rpath, `share/`), plus a source archive that includes the `.deps` archives for offline builds. |

**Deviation (D2 refinement, found while planning):** `tools/run_tests.sh native` always configures the **root** graph first (`ensure_native_checks`, `tools/run_tests.sh:33-44`). The root graph cannot configure on Linux until Phase 2. Therefore the Linux L3 moves from "end of Phase 1" to "end of Phase 2" (Task 2.11). Phase 1 closes with L2 on Linux and L3 on macOS.

**Deviation (L2 cadence):** the interview set an L2 every 4–6 tasks. Phase 1 has CHECKPOINT 1-A after nine tasks and 1-B after ten. Tasks 1.01–1.05 are lane and bookkeeping work: each carries its own L1 tests and touches no compiled code except the 1.01 fixes. Tasks 1.13–1.18 are each verified by Linux L1 on the TUs they fix. An extra L2 there would rebuild everything without catching anything L1 misses. Phase 4 gets an L2 after Task 4.04 (before deletion starts).

**Note (D10):** `docs/development/toolchain-matrix.json` has no `machines` key (its keys are `platforms`, `lanes`, `sanitizer_lanes`, …). "First machine is required" from the interview maps to the `platforms`/`lanes` entries Task 3.45 edits: Linux aarch64 is required, and x86_64 is marked not qualified.

**Open decision OD-1 (ask the user at Task 3.02, because Task 4.01 needs the ledger rows resolved; default stated):** the `feat/drop-oracle-parity` merge (Task 1.03) brings 22 research scripts under `tools/research/**/*.py`. These are OSM/DEM statistics and figure digitisation; their outputs back `docs/reference/*`. Default if the user does not answer: delete the scripts in Task 4.05 like every other Python file, keep their committed outputs, and record the deletion with the commit hash in `docs/reference/README-provenance.md` so the scripts stay recoverable from git.

---

## Global Constraints

- Work only in `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp` (branch `impl/python-free-cpp`). Never `cd` to the main checkout. **Exception (D17):** a Phase-3 lane task runs in its own lane worktree (the path the lane preamble names, e.g. `../pfc-lane-a`). Substitute that path wherever this plan or the prompt template names the integration worktree. Each lane uses its own container (`container.sh` derives the name from the checkout basename).
- Python is invoked only via `uv` (`uv run --frozen --group native …` or `uv run --no-project …`) until Task 4.05 removes Python. This also applies inside the container: `tools/linux/container.sh exec uv run --frozen --group native python …`. The image sets `UV_PROJECT_ENVIRONMENT=/opt/bike-venv`, so `uv` uses the lane venv; never call `/opt/bike-venv/bin/python` directly.
- Bitwise parity contract (until D12 hands a file over to goldens): C++ must bitwise-equal the reference Python arithmetic. Preserve valid-input binary64 operation order, `-ffp-contract=off`, and existing deliberate saturation (`docs/agents/native-engineering.md`).
- **macOS arithmetic must not change silently.** Any task touching numerics re-runs the macOS bitwise tests it lists. A changed bit on macOS is a failure unless the task explicitly says the change is intended and approved.
- MuJoCo 3.12.0 private ABI (`_mjPRIVATE_setTlsLogHandler` / `_mjPRIVATE_getGlobalLogHandler` / `mj_makeRawData`) stays pinned. No dependency version changes.
- CPython 3.13 (`.python-version` = `3.13`, `requires-python = ">=3.13,<3.14"`) is the oracle interpreter on both platforms.
- Never run two L3 gates at the same time on this Mac. The container shares the host CPU, so it counts as the same machine.
- Never hide a failed verification. Report command, exit code and the failing node IDs verbatim.
- Do not edit historical plans (`docs/superpowers/plans/2026-10-0*`, `2026-10-1*`), baseline logs under `migration/baseline-*.log`, or the data payload of `migration/oracle-manifest.json`.
- Never hand-edit generated artifacts (`native/numerics.lock.json` records are written by `tools/native_dependencies.py --write-numerics-record` only). Deleting the file so that the tool can regenerate it is not hand-editing. After Task 4.05 deletes the generator, the record is frozen: any later change needs a new task that first restores a generator.
- No network access from CMake configure/build/install. Network fetches happen only in the explicit acquisition steps named in this plan (Dockerfile, `.deps` acquisition), always with SHA-256 verification.
- Commit messages end with the `Co-Authored-By:` trailer that the executing harness specifies for the model that did the work. The commit commands in this plan write it as `Co-Authored-By: <the trailer the executing harness specifies>`; substitute the real line, never commit the placeholder.
- Status vocabulary in ledgers: `tests green` means the named CTest/pytest nodes passed. It never means runtime-qualified or acceptance-closed.

---

## Verification ladder

Every task names the exact L1 commands it needs. The building blocks:

### L1: per task (target ≤ 5 min)

```bash
# macOS, from the worktree root. Build whichever graph(s) the task touched;
# both exist until Task 4.06 deletes the binding graph.
cmake --build native/build -j10                  # binding graph (cmake -S native)
cmake --build build/native-release -j10          # Python-free root graph (cmake -S .)

# Linux aarch64 container (created by Task 1.02).
tools/linux/container.sh sync                    # copy the working tree into /src
tools/linux/container.sh build native-clang      # binding graph, Clang 18, Release
tools/linux/container.sh build native-gcc        # binding graph, GCC 13, Debug
tools/linux/container.sh build root-clang        # root graph, from Task 2.07 on
tools/linux/container.sh failures native-clang   # count of FAILED: TUs in the last log

# Exactly one sweep that covers the change (a failed sweep names its target):
cmake --build native/build --target check_<frontends|tidy|analyzer|cppcheck|odr|scripts>
# (these check_* targets exec bike-native-checks, which only the root graph builds;
#  in the container before Task 2.07 use, e.g.:
#  tools/linux/container.sh exec uv run --frozen --group native python tools/native_checks.py --kind tidy --build /build/native-clang)
build/native-release/bin/bike-native-checks --kind <headers|diagnostic-controls|context|contracts> --build <build-dir>

# Targeted tests:
ctest --test-dir build/native-release -R '^<test-name>$' --output-on-failure
ctest --test-dir native/build -R '^<test-name>$' --output-on-failure
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
  uv run --frozen --group native python -m pytest -q '<node-id>'
```

While the Linux binding graph still has known failures (Tasks 1.10–1.18), the Linux L1 is "the TUs this task fixes compile". Build their object targets by name; `ninja -t targets all | grep <file>.cpp.o` prints the exact names:

```bash
tools/linux/container.sh exec bash -c 'ninja -C /build/native-clang -t targets all | grep "io/json.cpp.o"'
tools/linux/container.sh build native-clang CMakeFiles/bike_io.dir/src/io/json.cpp.o
```

### L2: checkpoint (target ≤ 20 min; marked `CHECKPOINT` in the task list)

```bash
# macOS
cmake --build native/build -j10 && ctest --test-dir native/build -j8 --output-on-failure
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -j8 --output-on-failure
tools/run_native_checks.sh native/build
tools/run_native_checks.sh build/native-release
# Linux
tools/linux/container.sh sync
tools/linux/container.sh build native-clang && tools/linux/container.sh ctest native-clang -j8
tools/linux/container.sh build native-gcc   && tools/linux/container.sh ctest native-gcc -j8
# from Task 2.07 on, additionally:
tools/linux/container.sh build root-clang   && tools/linux/container.sh ctest root-clang -j8
tools/linux/container.sh exec tools/run_native_checks.sh /build/root-clang
```

A checkpoint passes only if every command exits 0. Record the commands, exit codes and test counts in the checkpoint's evidence block in `migration/cpp-linux-completion-evidence.md` (created by Task 1.01).

### L3: full gate (end of Phase 1 macOS, end of Phase 2 Linux, end of Phase 4 both)

```bash
bash tools/run_tests.sh native                   # macOS, 15–30 min
tools/linux/container.sh gate native             # Linux, from Task 2.11
```

Compare the failures against the documented baselines. Line numbers move (Task 1.03's merge and Tasks 1.04/3.05–3.11 edit these files), so always extract the lists by anchor, never by line range:

```bash
awk '/^The 24 failing node IDs/,/^## /' docs/TESTING.md | grep -oE 'tests/[^ `]+'          # macOS, 24 at planning time
awk '/retained failures/,/kept verbatim/' docs/development/linux.md | grep -oE 'tests/[^ `]+'  # Linux, 3 at planning time
```

Phase-3 tasks 3.05–3.11 shrink these lists (and update the count in the anchor sentence when they do). A failure outside the current list is a regression and must be fixed before the phase closes.

---

## Subagent prompt template

Every task is dispatched with this prompt (fill in `{TASK}`):

> You are implementing Task {TASK} of `docs/superpowers/plans/2026-10-20-cpp-linux-completion.md` in the worktree `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`.
>
> Read, in order: the plan's "Decision record", "Global Constraints" and "Verification ladder"; Task {TASK} in full; `AGENTS.md`; and, if the task touches `native/`, `docs/agents/native-engineering.md`. Follow the task's steps exactly, including the failing-test-first steps.
>
> Run only the L1 commands the task lists, plus any the task says to add. Never run `bash tools/run_tests.sh native|full` unless the task is a checkpoint that says so. If a step's expected result does not happen, stop and diagnose the root cause; do not weaken a check, skip a test or widen a tolerance.
>
> Commit exactly as the task's final step says, replacing the `Co-Authored-By: <…>` placeholder with the trailer your harness specifies. Do not push. Report: the files changed, every command with its exit code, test counts, and anything deferred (with the reason).

Reviewers use the task's **Review focus** line plus `docs/agents/native-engineering.md` "Review and hand off".

---

## File structure (what this plan creates or reshapes)

| Path | Responsibility | Created / changed in |
|---|---|---|
| `tools/linux/Dockerfile` | Linux aarch64 lane image: toolchains, GUI dev libs, Xvfb, uv + CPython 3.13.16 (until 4.06), pinned `.deps` acquisition (from 2.05) | 1.02, 2.05, 2.10, 4.06 |
| `tools/linux/container.sh` | Drives the lane: `image/up/sync/venv/configure/build/failures/ctest/exec/pull/gate/down` | 1.02, 2.11, 4.06 |
| `migration/cpp-linux-completion-evidence.md` | Append-only evidence log for checkpoints and L3 runs | 1.01 onward |
| `docs/adr/0003-native-authoritative-python-removal.md` | ADR-0003 (D12) | 1.04 |
| `native/src/tools/check_context.cpp` (+ `check_contracts.cpp`) | Schema-2 context validation, sysroot rules, tool resolution, platform-branched controls | 1.06–1.09 |
| `native/cmake/BikeNumerics.cmake`, `native/src/platform/numerics_symbols.hpp.in`, `tools/native_dependencies.py`, `native/numerics.lock.json` | Numerics ops `dgesv`, `dsyevd` (LAPACK) and the ILP64 BLAS class `ddot_ilp64`, `zdotu_sub`; Linux record; record-only Linux mode | 1.10, 2.06 |
| `native/src/cblas_abi.hpp`, `native/src/lapack_abi.hpp`, `native/src/platform/{blas,lapack}_{accelerate,numpy_openblas}.cpp` | Typed numerics face incl. `ddot_raw` (bound to `BIKE_NUMERICS_SYM_DDOT_ILP64`), `zdotu_sub`, `dgesv`, `dsyevd` | 1.11 |
| `native/src/io/value.hpp` | `Value` complete under libstdc++ | 1.13 |
| `CMakeLists.txt` (root), `native/cmake/Checks.cmake` | Root graph on the `Bike*` layer; schema-2 context | 2.01–2.03, 2.07 |
| `native/dependencies.lock.json` (schema 2) | Per-platform dependency entries | 2.04, 2.05 |
| `tests/reference/_golden.py`, `tests/reference/_bits.py` | Golden capture hook | 3.01 |
| `native/tests/support/golden.{hpp,cpp}` | `bt::oracle::*` readers (bitwise and tolerance), consumed-index check | 3.01 |
| `native/src/io/npy.hpp`, `native/src/io/npz.cpp` | `encode_npz` writer (stored ZIP members, shared CRC32) for episode artifacts | 3.03a |
| `migration/ctest-graph-map.json` | Binding-graph-only CTest nodes → root-graph successor or recorded drop | 3.02, checked in 4.06 |
| `native/tests/reference/CMakeLists.txt` + `test_*.cpp` | Ported test suites (paths from plan-08) | 3.01, 3.12–3.36 |
| `tests/fixtures/golden/<platform>/…` | Frozen oracle values, per platform | 3.12–3.36 |
| `native/src/cli/recording_golden.cpp` | `bike-recording golden freeze/compare` | 3.03 |
| `native/src/tools/measure_realtime_main.cpp` | `bike-measure-realtime` | 3.04 |
| `CMakePresets.json` (root) | Configure/build/test presets replacing `tools/dev.py` | 3.41 |
| `native/cmake/Sanitizers.cmake`, `native/cmake/Coverage.cmake` | Sanitizer/coverage lanes for the root graph | 3.42, 3.43 |
| `packaging/CMakeLists.txt` (CPack) | Binary + source distributions | 3.44 |
| `.github/workflows/portability.yml`, `docs/releases/python-free-qualification.md` | CI on presets; release checklist | 3.45, 4.03 |

---

# Phase 1: Restore the macOS gate and build the binding graph on Linux

### Task 1.01: Commit the in-flight build fixes and open the evidence log

**Files:**
- Modify (already modified in the tree): `native/cmake/BikeLinkage.cmake:184-193`, `native/src/io/value.hpp:32`
- Create: `migration/cpp-linux-completion-evidence.md`

**Interfaces:**
- Consumes: nothing.
- Produces: a clean tree; the evidence log that every checkpoint appends to.

**Review focus:** the ARGC change must keep one-argument callers working (they rely on `_native_mujoco_library`).

- [ ] **Step 1: Inspect the two uncommitted changes**

```bash
git status --short
git diff native/cmake/BikeLinkage.cmake native/src/io/value.hpp
```

Expected: exactly these two files.
- `BikeLinkage.cmake`: `bike_fix_mujoco_install_name` reads its optional second argument through `ARGC`, because `${ARGV1}` would otherwise resolve to the caller's own second argument.
- `value.hpp`: adds `#include <algorithm>`.

- [ ] **Step 2: L1 on macOS (both graphs)**

```bash
cmake --build native/build -j10
cmake --build build/native-release -j10
```

Expected: both exit 0.

- [ ] **Step 3: Create the evidence log**

Create `migration/cpp-linux-completion-evidence.md`:

```markdown
# C++ + Linux completion: evidence log

Append-only. One section per checkpoint or L3 run of
`docs/superpowers/plans/2026-10-20-cpp-linux-completion.md`. Each section
records: date, commit (`git rev-parse HEAD`), host (`uname -a` or container
image digest), every command with its exit code, test counts, and the
failing node IDs verbatim. Do not rewrite earlier sections.

## 2026-10-20 audit snapshot (before Task 1.01)

- macOS: `cmake --build native/build` and `cmake --build build/native-release` exit 0.
- macOS: 8 of 9 sweeps fail with `native check context has an unsupported schema`
  (only `check_scripts` passes): the binding graph emits schema 2, and
  `native/src/tools/check_context.cpp:172-179` accepts only schema 1.
- Linux aarch64 (Ubuntu 24.04 container):
  - `cmake -S native` fails configure: `native/numerics.lock.json` has no `linux-aarch64` record.
  - With a record generated inside the container: Clang 18 fails 239 TUs, GCC 13 fails 170 TUs plus one link error.
- Linux root graph (`cmake -S .`) fails configure: `.deps` holds Darwin artifacts only,
  and `BikeNumerics.cmake:226` refuses record-only numerics off Darwin.
```

- [ ] **Step 4: Commit**

```bash
git add native/cmake/BikeLinkage.cmake native/src/io/value.hpp migration/cpp-linux-completion-evidence.md
git commit -m "build(native): ARGC-guarded install-name image; include <algorithm> in io/value

Opens migration/cpp-linux-completion-evidence.md for the completion plan.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.02: Commit the Linux aarch64 lane (Dockerfile + driver script)

**Files:**
- Create: `tools/linux/Dockerfile`
- Create: `tools/linux/container.sh` (mode 0755)
- Modify: `docs/development/linux.md` (new section "Container lane", after "Prerequisites")
- Modify: `tools/check_scripts.sh:9` and `native/src/tools/check_frontends.cpp` (scripts kind selection) — sweep depth 2
- Test: `native/tests/migration/migration_T56_check.cmake` (depth-2 scripts case)

**Interfaces:**
- Consumes: Docker Desktop with `linux/arm64`.
- Produces: `tools/linux/container.sh <image|up|sync|venv|configure|build|failures|ctest|exec|pull|down>`. Every later task uses these exact subcommands. Container name: `bike-dev-<basename of checkout>`, overridable with `BIKE_LINUX_CONTAINER`. Image tag: `bike-dev-linux:24.04` (`BIKE_LINUX_IMAGE`). Build trees live at `/build/<name>`; sources at `/src`.

**Review focus:**
- `sync` never copies `.venv`, `build*/` or the Darwin `.deps`, and never deletes `/src/.deps` or `/src/build`.
- File mtimes survive (incremental rebuilds).
- `shellcheck` is clean.

- [ ] **Step 1: Pin the base image digest**

```bash
docker pull --platform linux/arm64 ubuntu:24.04
docker inspect --format '{{index .RepoDigests 0}}' ubuntu:24.04
```

Expected: `ubuntu@sha256:<64 hex>`. Use that exact string in the `FROM` line below.

- [ ] **Step 2: Write `tools/linux/Dockerfile`**

```Dockerfile
# tools/linux/Dockerfile — the qualified Linux aarch64 development lane.
#
# Build with: tools/linux/container.sh image
# Toolchain versions are the measured lane (docs/development/linux.md):
# Clang 18.1.3, GCC 13.3.0, CMake 3.28.3, uv 0.13.1, CPython 3.13.16.
# The Python layer (uv + interpreter) exists only for the binding graph
# and the oracle; Task 4.06 of the completion plan removes it.
FROM ubuntu@sha256:REPLACE_WITH_STEP_1_DIGEST

ARG UV_VERSION=0.13.1
ARG PYTHON_VERSION=3.13.16
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git rsync unzip xz-utils file pkg-config \
        clang clang-18 clang-tidy clang-tidy-18 clang-tools-18 \
        llvm-18 lld-18 g++ g++-13 \
        cmake ninja-build cppcheck shellcheck \
        libwayland-dev wayland-protocols libxkbcommon-dev xorg-dev libgl-dev \
        xvfb xauth libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" \
        | env UV_INSTALL_DIR=/usr/local/bin UV_NO_MODIFY_PATH=1 sh \
    && uv python install "${PYTHON_VERSION}"

ENV UV_PROJECT_ENVIRONMENT=/opt/bike-venv UV_LINK_MODE=copy
WORKDIR /src
CMD ["sleep", "infinity"]
```

Replace `REPLACE_WITH_STEP_1_DIGEST` with the digest from Step 1 before building. The final file must contain no placeholder.

- [ ] **Step 3: Write `tools/linux/container.sh`**

```bash
#!/usr/bin/env bash
# tools/linux/container.sh — drive the Linux aarch64 lane for this checkout.
#
#   image                          build the lane image (tools/linux/Dockerfile)
#   up                             start this checkout's container (idempotent)
#   sync                           copy tracked + untracked-unignored files to /src
#   venv                           provision /opt/bike-venv from uv.lock
#   configure NAME GRAPH [args]    GRAPH = native (cmake -S native) | root (cmake -S .)
#   build NAME [ninja targets]     ninja -k 0; log kept in /build/NAME.log
#   failures NAME                  number of FAILED: lines in the last build log
#   ctest NAME [ctest args]        ctest --test-dir /build/NAME --output-on-failure
#   exec CMD...                    run CMD with /src as the working directory
#   pull PATH...                   copy /src/PATH back into this checkout
#   down                           remove the container
#
# Each checkout owns one container (BIKE_LINUX_CONTAINER overrides the
# name), so parallel worktrees never share /src or /build. The host .venv,
# build trees and the Darwin .deps never enter the container: sync sends
# only `git ls-files --cached --others --exclude-standard`. Archive mtimes
# are preserved, so unchanged sources do not rebuild. Deleted files must be
# `git rm`-ed (or restored) before sync, otherwise tar reports them missing.
# sync keeps /src/.deps (Linux artifacts, Task 2.05) and /src/build (root
# preset trees, so in-container preset runs stay incremental); the tar never
# carries build/ because it is gitignored.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." >/dev/null 2>&1 && pwd -P)"
image="${BIKE_LINUX_IMAGE:-bike-dev-linux:24.04}"
name="${BIKE_LINUX_CONTAINER:-bike-dev-$(basename -- "$repo_root")}"

die() {
    printf 'container.sh: %s\n' "$*" >&2
    exit 2
}

in_container() {
    docker exec -i -w /src "$name" "$@"
}

[[ $# -ge 1 ]] || die "usage: container.sh <image|up|sync|venv|configure|build|failures|ctest|exec|pull|down> ..."
cmd="$1"
shift

case "$cmd" in
    image)
        docker build --platform linux/arm64 -t "$image" \
            -f "$repo_root/tools/linux/Dockerfile" "$repo_root/tools/linux"
        ;;
    up)
        if ! docker container inspect "$name" >/dev/null 2>&1; then
            docker run -d --platform linux/arm64 --name "$name" "$image" >/dev/null
        fi
        docker start "$name" >/dev/null
        ;;
    sync)
        "$0" up
        (cd "$repo_root" &&
            git ls-files -z --cached --others --exclude-standard |
            COPYFILE_DISABLE=1 tar --null --no-xattrs --no-mac-metadata -T - -cf -) |
            in_container bash -c 'mkdir -p /src &&
                find /src -mindepth 1 -maxdepth 1 ! -name .deps ! -name build -exec rm -rf {} + &&
                tar --warning=no-unknown-keyword -xf - -C /src'
        ;;
    venv)
        in_container uv sync --frozen --group native
        ;;
    configure)
        [[ $# -ge 2 ]] || die "usage: configure NAME native|root [cmake args...]"
        build="$1"
        graph="$2"
        shift 2
        case "$graph" in
            native) src=/src/native; extra=(-DPython_EXECUTABLE=/opt/bike-venv/bin/python3) ;;
            root) src=/src; extra=(-DBIKE_BUILD_PYTHON_BINDINGS=OFF "-DBIKE_DEPENDENCY_ROOT=/src/.deps") ;;
            *) die "unknown graph: $graph (expected native or root)" ;;
        esac
        in_container cmake -S "$src" -B "/build/$build" -G Ninja "${extra[@]}" "$@"
        ;;
    build)
        [[ $# -ge 1 ]] || die "usage: build NAME [ninja targets...]"
        build="$1"
        shift
        # shellcheck disable=SC2016  # $0/$@ are the bash -c positionals, expanded in the container
        in_container bash -c 'set -o pipefail; ninja -C "/build/$0" -k 0 "$@" 2>&1 | tee "/build/$0.log"' "$build" "$@"
        ;;
    failures)
        [[ $# -eq 1 ]] || die "usage: failures NAME"
        # shellcheck disable=SC2016  # $0 is the bash -c positional, expanded in the container
        in_container bash -c 'grep -c "^FAILED:" "/build/$0.log" || true' "$1"
        ;;
    ctest)
        [[ $# -ge 1 ]] || die "usage: ctest NAME [ctest args...]"
        build="$1"
        shift
        in_container ctest --test-dir "/build/$build" --output-on-failure "$@"
        ;;
    exec)
        [[ $# -ge 1 ]] || die "usage: exec CMD..."
        in_container "$@"
        ;;
    pull)
        [[ $# -ge 1 ]] || die "usage: pull PATH..."
        for rel in "$@"; do
            mkdir -p "$repo_root/$(dirname -- "$rel")"
            docker cp "$name:/src/$rel" "$repo_root/$(dirname -- "$rel")/"
        done
        ;;
    down)
        docker rm -f "$name" >/dev/null
        ;;
    *)
        die "unknown command: $cmd"
        ;;
esac
```

```bash
chmod 0755 tools/linux/container.sh
```

- [ ] **Step 4: Make the scripts sweep cover `tools/linux/` and lint the script**

Both scripts sweeps select `tools/*.sh` at depth 1 only, so neither sees `tools/linux/*.sh` today:
- binding graph: `tools/check_scripts.sh:9` (`find "$TOOLS_DIR" -maxdepth 1 -name '*.sh'`), run by `check_scripts` (`native/CMakeLists.txt:595`);
- root graph: the C++ `scripts` kind, `native/src/tools/check_frontends.cpp` (the `fs::directory_iterator` loop over `tools_dir`, about line 1010), exercised by `migration_T56` cases `t56_scripts_*` (`migration_T56_check.cmake:255-285`).

The Python oracle has no scripts kind, so there is no third selection to keep in parity.

Widen both to depth 2, the same set in the same byte order:
- `tools/check_scripts.sh`: `find "$TOOLS_DIR" -maxdepth 2 -name '*.sh' -print0 | sort -z | xargs -0 "$SHELLCHECK"`; update its header comment to `tools/*.sh and tools/*/*.sh`.
- `check_frontends.cpp`: iterate `tools_dir` and each immediate subdirectory (one extra `fs::directory_iterator` level, not `recursive_directory_iterator`), keep `std::ranges::sort(scripts)`.
- `migration_T56_check.cmake`: add a case that plants `bad.sh` in a `linux/` subdirectory of the planted clean tools dir and expects `t56_scripts_*` style failure (`EXPECT_EXIT FAIL`), proving depth 2 is swept.

```bash
shellcheck tools/linux/container.sh
cmake --build native/build --target check_scripts
cmake --build build/native-release -j10 --target bike-native-checks
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
```

Expected: all exit 0. The `check_scripts` output names `tools/linux/container.sh` among the checked files (`bash -x tools/check_scripts.sh tools 2>&1 | grep linux/` if it is not printed).

- [ ] **Step 5: Build the image and provision the lane**

```bash
tools/linux/container.sh image
tools/linux/container.sh sync
tools/linux/container.sh venv
tools/linux/container.sh exec clang++-18 --version
tools/linux/container.sh exec g++-13 --version
tools/linux/container.sh exec uv run --frozen --group native python -c "import numpy, mujoco; print(numpy.__version__, mujoco.__version__)"
```

Expected: `clang version 18.1.3`, `g++-13 (Ubuntu 13.3.0…)`, `2.5.2 3.12.0`.

- [ ] **Step 6: Record the expected red configure (the Task 1.10 entry condition)**

```bash
tools/linux/container.sh configure native-clang native -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18; echo "exit=$?"
```

Expected: non-zero exit, with `committed numerics record has no entry 'linux-aarch64'` (`native/cmake/BikeNumerics.cmake:245-249`). Any other error is a lane defect; fix the Dockerfile or script before continuing.

- [ ] **Step 7: Document the lane**

In `docs/development/linux.md`, after the "Prerequisites" section, add:

````markdown
## Container lane (this repository's Linux aarch64 qualification lane)

The lane is reproducible from the checkout. Docker Desktop with `linux/arm64` is the only host requirement:

```sh
tools/linux/container.sh image      # once per Dockerfile change
tools/linux/container.sh sync       # before every build: copies the working tree to /src
tools/linux/container.sh venv       # once per uv.lock change (binding graph only)
tools/linux/container.sh configure native-clang native -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
tools/linux/container.sh configure native-gcc   native -DCMAKE_BUILD_TYPE=Debug   -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
tools/linux/container.sh build native-clang
tools/linux/container.sh ctest native-clang -j8
```

`sync` copies tracked and untracked-unignored files only. The host `.venv`, build trees and the Darwin `.deps` never cross. Each checkout gets its own container (`bike-dev-<checkout name>`, overridable with `BIKE_LINUX_CONTAINER`).
````

- [ ] **Step 8: Commit**

```bash
git add tools/linux/Dockerfile tools/linux/container.sh docs/development/linux.md tools/check_scripts.sh \
        native/src/tools/check_frontends.cpp native/tests/migration/migration_T56_check.cmake
git commit -m "build(linux): commit the aarch64 container lane

The previous Linux evidence came from a hand-built image bind-mounted on
the linux-macos-port worktree; the merged branch was never built on Linux.
The lane now rebuilds from the checkout and copies the tree instead of
mounting it, so the host .venv cannot be clobbered. Both scripts sweeps
now reach tools/*/*.sh, so the lane scripts are linted like tools/*.sh.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.03: Merge `feat/drop-oracle-parity` (ADR-0002 + `main` r2 research)

**Files:**
- Merge only. Brings `docs/adr/0002-drop-python-oracle-parity.md`, edits to `AGENTS.md`, `CONTEXT.md`, `docs/TESTING.md`, `docs/agents/native-engineering.md`, `docs/adr/0001-…`, plus `main`'s r2 research docs, data and `tools/research/**`.

**Interfaces:**
- Consumes: nothing.
- Produces: ADR-0002 in the tree, which Task 1.04 amends.

**Review focus:** no conflict markers; no native source changes arrive with the merge.

- [ ] **Step 1: Preview**

```bash
git merge-tree --write-tree --name-only impl/python-free-cpp feat/drop-oracle-parity
git diff --stat impl/python-free-cpp...feat/drop-oracle-parity -- native/ CMakeLists.txt | tail -3
```

Expected:
- the first command prints only a tree hash (no conflicts, verified clean on 2026-10-20);
- the second prints nothing (no native changes).

If either differs, stop and report.

- [ ] **Step 2: Merge**

```bash
git merge --no-ff feat/drop-oracle-parity -m "merge: drop-oracle-parity (ADR-0002) and main r2 research

Co-Authored-By: <the trailer the executing harness specifies>"
```

- [ ] **Step 3: L1**

```bash
cmake --build native/build -j10
cmake --build build/native-release -j10
cmake --build native/build --target check_scripts
```

Expected: exit 0 each.

- [ ] **Step 4: Register the new Python files for later triage**

```bash
git diff --name-only HEAD^1 HEAD -- '*.py' > /tmp/r2-python-files.txt
wc -l /tmp/r2-python-files.txt
```

Expected: 22 paths under `tools/research/`. Append them to the evidence log under a heading `## Task 1.03 — Python files added by the r2 merge (OD-1)`, then commit:

```bash
git add migration/cpp-linux-completion-evidence.md
git commit -m "docs(migration): record the r2 research scripts awaiting OD-1

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.04: ADR-0003 and the agent contract

**Files:**
- Create: `docs/adr/0003-native-authoritative-python-removal.md`
- Modify: `AGENTS.md` (the paragraph the Task 1.03 merge leaves starting `Oracle parity is dropped (ADR-0002):`; the old "Bitwise oracle pinning" paragraph no longer exists after the merge)
- Modify: `docs/agents/native-engineering.md` (the sentence containing `operation-order parity requirement toward the Python reference`, which ADR-0002 rewrote)
- Modify: `docs/adr/0002-drop-python-oracle-parity.md` (status line only)

**Interfaces:**
- Consumes: ADR-0002 (Task 1.03).
- Produces: the rule every Phase-3 task follows: parity until goldens, then goldens.

**Review focus:**
- No contradiction remains between AGENTS.md, native-engineering.md, ADR-0002 and ADR-0003 about when Python parity is required.

- [ ] **Step 1: Read the merged texts**

```bash
sed -n 1,80p docs/adr/0002-drop-python-oracle-parity.md
grep -n -i -A8 'oracle\|parity' AGENTS.md | head -60
grep -n -i -B2 -A6 'parity' docs/agents/native-engineering.md
```

- [ ] **Step 2: Write ADR-0003**

```markdown
# ADR 0003: Native core is authoritative; Python is removed after golden capture

Status: accepted
Date: 2026-10-20
Amends: [ADR-0002](0002-drop-python-oracle-parity.md). It keeps ADR-0002's verification
model and overrules its item 1 sentence "Python physics stays in the repository".

## Context

ADR-0002 dropped the Python↔C++ parity contract and kept Python as a readable
specification. The completion plan (`docs/superpowers/plans/2026-10-20-cpp-linux-completion.md`)
deletes every first-party Python file (T63). Without a transition rule, deleting the oracle
would delete the only evidence that today's C++ behavior is the reviewed behavior.

## Decision

1. **Transition rule, per test file.** While a pytest file still compares C++ against the
   live Python oracle, that comparison stays a gate (bitwise, as today). The file's porting
   task captures the oracle values the comparisons consume into `tests/fixtures/golden/<platform>/`
   on macOS arm64 and Linux aarch64. From the commit that lands the C++ port, the C++ test
   against those goldens is the gate, and the pytest file is no longer one.
2. **After T63 the native core is the only implementation.** Verification is ADR-0002 item 2:
   - in-build determinism;
   - native unit and contract tests;
   - cheap physical invariants;
   - goldens (the frozen oracle values plus native self-goldens recorded with `bike-recording golden freeze`).
3. **Golden updates are reviewed changes.** A golden changes only in a commit that says why the
   behavior changed, shows `bike-recording golden compare` output before/after, and touches no
   unrelated golden.
4. **Per-platform goldens.** macOS (Accelerate) and Linux (scipy-openblas) keep separate golden
   trees. Cross-platform bitwise identity is not claimed.
5. **Python is deleted, not archived in-tree.** Git history is the archive. The oracle tag
   `reference/python-oracle` stays.

## Consequences

- `tests/reference/conftest.py::_pin_oracle_kernels` and the `*_py` twins live until T63, then go.
- `-ffp-contract=off` stays (ADR-0002 item 4).
- A physics change after a file's capture re-records that file's native self-goldens in the same commit.
```

- [ ] **Step 3: Align the agent contract**

In `AGENTS.md`, replace the whole paragraph starting `Oracle parity is dropped (ADR-0002):` (through the line ending `docs/adr/0002-drop-python-oracle-parity.md.`) with:

```markdown
Oracle parity (ADR-0003): a pytest file that compares C++ with the live
Python oracle stays a bitwise gate until its porting task captures goldens
(`tests/fixtures/golden/<platform>/`) on macOS and Linux and lands the C++
port; from then on the C++ test against the goldens is the gate. Inside
`test_native_*.py` files `tests/reference/conftest.py::_pin_oracle_kernels`
rebinds the njit cores to their `*_py` twins; if a kernel signature changes,
its twin and the fixture change with it. All of this is deleted at T63.
```

In `docs/agents/native-engineering.md`, replace the sentence containing `operation-order parity requirement toward the Python reference` so that it reads: `Preserve valid-input binary64 operation order (parity with the Python oracle per ADR-0003 until a file's goldens land; afterwards determinism and goldens).`

In ADR-0002, change the status line to `Статус: принято; п.1 «Python остаётся» отменён ADR-0003` (keep the document's language).

- [ ] **Step 4: Check consistency**

```bash
grep -rn -i 'oracle' AGENTS.md docs/agents/native-engineering.md docs/adr/0002-drop-python-oracle-parity.md docs/adr/0003-native-authoritative-python-removal.md
```

Expected: no statement says parity is dropped "now" or that Python "stays".

- [ ] **Step 5: Commit**

```bash
git add docs/adr/0003-native-authoritative-python-removal.md docs/adr/0002-drop-python-oracle-parity.md AGENTS.md docs/agents/native-engineering.md
git commit -m "docs(adr): ADR-0003 native authoritative after per-file golden capture

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.05: Close the F2/F3/F5 bookkeeping

**Files:**
- Modify: `migration/python-removal.json` (73 rows whose `implementation.status == "implemented_tested_findings_open"`; each also carries `implementation.findings_open: ["F2","F3","F5"]`)
- Modify: `migration/physics-research-implementation.json` (4 `task_records[*]` entries whose `implementation_status == "implemented_tested_findings_open"`, each with `findings_open`)
- Modify: `migration/native-waves-commit-review.md` (append a dated section)
- Modify: `migration/cards/T26.md:1372` (the clause `OPEN FINDINGS F2/F3/F5 (see …) — …` on the F189 implementation note)

**Interfaces:**
- Consumes: commit `89cfcb1` ("close wave-review findings F2/F3/F5 in PhysicalStep"), which added regressions to `native/tests/migration/test_t23.cpp`.
- Produces: honest ledgers. 73 rows move from `implemented_tested_findings_open` to `implemented_tested` **only if** the regressions pass.

**Review focus:** the status flip happens only after the named tests pass on both graphs.

- [ ] **Step 1: Run the F2/F3/F5 regressions and their neighbors**

```bash
cmake --build build/native-release -j10 --target migration_T23 migration_T26 migration_T29 migration_T30 migration_T32
ctest --test-dir build/native-release -R '^migration_T(23|26|29|30|32)$' --output-on-failure
cmake --build native/build -j10
ctest --test-dir native/build -R '^migration_T(23|26|29|30|32)$' --output-on-failure
```

Expected: all pass. If any fails, stop: the findings are not closed. Report the failure and do not edit the ledgers.

- [ ] **Step 2: Flip the ledger rows**

Run (one-off edit, not committed as a tool):

The two ledgers spell the status differently, and neither has a `notes` field:
- `python-removal.json` (top-level array): `row.implementation.status`, `row.implementation.findings_open`, `row.implementation.evidence` (a list of repository paths).
- `physics-research-implementation.json`: `task_records[*].implementation_status` and `task_records[*].findings_open`.

Both files are `json.dumps(…, indent=2, ensure_ascii=False)` + `"\n"` byte-for-byte (checked 2026-10-20), so the rewrite below produces a diff of only the changed fields.

```bash
uv run --no-project python - <<'EOF'
import json, pathlib
REVIEW = "migration/native-waves-commit-review.md"
OPEN = "implemented_tested_findings_open"
for path in ("migration/python-removal.json", "migration/physics-research-implementation.json"):
    p = pathlib.Path(path)
    text = p.read_text()
    data = json.loads(text)
    assert json.dumps(data, indent=2, ensure_ascii=False) + "\n" == text, f"{path}: formatting drifted"
    changed = 0
    def walk(node):
        global changed
        if isinstance(node, dict):
            impl = node.get("implementation")
            if isinstance(impl, dict) and impl.get("status") == OPEN:
                impl["status"] = "implemented_tested"
                impl["findings_open"] = []
                if isinstance(impl.get("evidence"), list) and REVIEW not in impl["evidence"]:
                    impl["evidence"].append(REVIEW)
                changed += 1
            if node.get("implementation_status") == OPEN:
                node["implementation_status"] = "implemented_tested"
                node["findings_open"] = []
                changed += 1
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
    walk(data)
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    print(path, changed)
EOF
git diff --stat migration/
```

Expected:
- `migration/python-removal.json 73`;
- `migration/physics-research-implementation.json 4`.

`git diff` shows only `status`/`implementation_status`, `findings_open` and the appended evidence path; no whole-file reformatting. The dated closure sentence lives in `native-waves-commit-review.md` (Step 3), which the evidence path points to.

- [ ] **Step 3: Append the re-verification section** to `migration/native-waves-commit-review.md`:

```markdown
## Re-verification 2026-10-20

- F2 — FIXED by 89cfcb1: `ideal_speed_control` computes the cruise torque (regression in `native/tests/migration/test_t23.cpp`).
- F3 — FIXED by 89cfcb1: `native_reference` applies resistance from contact snapshots.
- F5 — FIXED by 89cfcb1: `pedal_recovery` receives `seated_climb.enabled`.
- Evidence: `ctest -R '^migration_T(23|26|29|30|32)$'` green on `native/build` and `build/native-release` (commit <HEAD>).
```

Replace `<HEAD>` with `git rev-parse --short HEAD`. In `migration/cards/T26.md:1372`, delete the whole clause from `OPEN FINDINGS F2/F3/F5` to the end of that line (`… pedal_recovery switch ignored.`) and append `F2/F3/F5 closed 2026-10-20 (see migration/native-waves-commit-review.md).` Then `grep -rn 'OPEN FINDINGS' migration/` prints nothing.

- [ ] **Step 4: Commit**

```bash
git add migration/python-removal.json migration/physics-research-implementation.json migration/native-waves-commit-review.md migration/cards/T26.md
git commit -m "docs(migration): close F2/F3/F5 after re-running their regressions

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.06: Checker: load schema-2 contexts with platform validation

**Files:**
- Modify: `native/src/tools/check_context.cpp:164-366` (`load_context`; the schema dispatch is at 172-179, the SDK block at about 204-230; lines 232-366 — `libcpp_hardening`, suppressions, path lists — already accept the real schema-2 context and need no change)
- Modify: `native/src/tools/checks_main.cpp` (`--validate-only`; `usage_line` at :39-46 and the `--help` text at :106-113)
- Modify: `native/src/tools/checks_internal.hpp` (declare the new helpers if `check_contracts.cpp` needs them)
- Test: `native/tests/migration/migration_T56_check.cmake` (new `run_case` entries)

**Interfaces:**
- Consumes: the schema-2 record written by `native/cmake/BikeCheckContext.cmake:505-507`, and the reference semantics in `tools/native_checks.py:171-338` (`validate_platform_context`) and `:340-449` (capabilities, frontend tools).
- Produces:
  - `Value load_context(const fs::path &build)` accepts `schema_version` 1 (validated with Darwin semantics, as `native_checks.py:234-235`) and 2.
  - The returned context always carries normalized `sdk` / `sdk_requested`: strings on Darwin, null on Linux.
  - A new top-level member `"platform_system"` (`"Darwin"` | `"Linux"`), which Tasks 1.07–1.08 read.

**Review focus:**
- Each Python rule has a C++ twin with the **same error text**, so the T56 needles and the later port of `test_native_check_tools.py` (Task 3.29) match.
- No `std::get` is reached on a null value.

- [ ] **Step 1: Write the failing CTest cases**

In `native/tests/migration/migration_T56_check.cmake`, after the `t56_ctx_missing_required` case, add a helper that derives a host-matching schema-2 context from the produced root context (still schema 1 until Task 2.03a), and the cases:

```cmake
# ---------------------------------------------------------------------------
# Schema-2 contexts (BikeCheckContext.cmake) — derived from the produced
# schema-1 context so every path stays real on this host.
cmake_host_system_information(RESULT _t56_machine QUERY OS_PLATFORM)
set(_t56_system "${CMAKE_HOST_SYSTEM_NAME}")

function(t56_schema2 out_var system)
  set(_json "${_context_json}")
  string(JSON _json SET "${_json}" schema_version "2")
  string(JSON _json SET "${_json}" platform
    "{\"system\":\"${system}\",\"host_system\":\"${CMAKE_HOST_SYSTEM_NAME}\",\"machine\":\"${_t56_machine}\",\"pointer_bits\":64}")
  if(system STREQUAL "Darwin")
    string(JSON _sdk GET "${_json}" sdk)
    string(JSON _json SET "${_json}" sysroot "\"${_sdk}\"")
    string(JSON _json SET "${_json}" stdlib
      "{\"name\":\"libc++\",\"version\":\"1\",\"hardening\":{\"mode\":\"EXTENSIVE\",\"effective\":true}}")
  else()
    string(JSON _json SET "${_json}" sysroot "null")
    string(JSON _json SET "${_json}" sdk "null")
    string(JSON _json SET "${_json}" sdk_requested "null")
    string(JSON _json SET "${_json}" sdk_source "\"not-applicable\"")
    string(JSON _json SET "${_json}" stdlib
      "{\"name\":\"libstdc++\",\"version\":\"13\",\"hardening\":{\"mode\":\"ASSERTIONS\",\"effective\":true}}")
  endif()
  string(JSON _json SET "${_json}" numerics "{\"backend\":\"TEST\"}")
  string(JSON _json SET "${_json}" policy
    "{\"required_flags\":[],\"accepted_warnings\":[],\"rejected_warnings\":[]}")
  # tools/native_checks.py:340-449 requires asan/rtsan/coverage records and
  # a frontend_tools object; evidence maps names to strings.
  string(JSON _json SET "${_json}" capabilities
    "{\"asan\":{\"requested\":false,\"supported\":false,\"enabled\":false,\"runtime\":null,\"compiler_id\":\"Clang\",\"target\":null,\"evidence\":{\"probe\":\"fixture\"}},\"rtsan\":{\"requested\":false,\"supported\":false,\"enabled\":false,\"runtime\":null,\"compiler_id\":\"Clang\",\"target\":null,\"evidence\":{\"probe\":\"fixture\"}},\"coverage\":{\"requested\":false,\"supported\":false,\"enabled\":false,\"runtime\":null,\"evidence\":{\"probe\":\"fixture\"}},\"frontend_tools\":{}}")
  set(${out_var} "${_json}" PARENT_SCOPE)
endfunction()

function(t56_ctx_case name json)
  set(_fixture "${SCRATCH_DIR}/${name}")
  file(MAKE_DIRECTORY "${_fixture}")
  file(WRITE "${_fixture}/native_check_context.json" "${json}")
  set(_fixture_dir "${_fixture}" PARENT_SCOPE)
endfunction()

t56_schema2(_s2 "${_t56_system}")
t56_ctx_case(ctx-schema2-host "${_s2}")
run_case(t56_ctx_schema2_host EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" schema_version "3")
t56_ctx_case(ctx-schema3 "${_bad}")
run_case(t56_ctx_schema3 EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has an unsupported schema"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" platform host_system "\"Plan9\"")
t56_ctx_case(ctx-schema2-host-mismatch "${_bad}")
run_case(t56_ctx_schema2_host_mismatch EXPECT_EXIT FAIL
  EXPECT_STDERR "host_system='Plan9' on a '${_t56_system}' host"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad REMOVE "${_s2}" platform)
t56_ctx_case(ctx-schema2-no-platform "${_bad}")
run_case(t56_ctx_schema2_no_platform EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has no platform record"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" sysroot "\"/nonexistent/sysroot\"")
t56_ctx_case(ctx-schema2-bad-sysroot "${_bad}")
run_case(t56_ctx_schema2_bad_sysroot EXPECT_EXIT FAIL
  EXPECT_STDERR "sysroot must be null or a directory path"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" stdlib name "\"msvcrt\"")
t56_ctx_case(ctx-schema2-bad-stdlib "${_bad}")
run_case(t56_ctx_schema2_bad_stdlib EXPECT_EXIT FAIL
  EXPECT_STDERR "stdlib"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad REMOVE "${_s2}" numerics backend)
t56_ctx_case(ctx-schema2-no-numerics "${_bad}")
run_case(t56_ctx_schema2_no_numerics EXPECT_EXIT FAIL
  EXPECT_STDERR "numerics"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad SET "${_s2}" capabilities asan enabled "true")
t56_ctx_case(ctx-schema2-enabled-unsupported "${_bad}")
run_case(t56_ctx_schema2_enabled_unsupported EXPECT_EXIT FAIL
  EXPECT_STDERR "capability asan is enabled without support evidence"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

string(JSON _bad REMOVE "${_s2}" capabilities coverage)
t56_ctx_case(ctx-schema2-no-coverage "${_bad}")
run_case(t56_ctx_schema2_no_coverage EXPECT_EXIT FAIL
  EXPECT_STDERR "native check context has no coverage capability record"
  COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)

if(_t56_system STREQUAL "Darwin")
  # A Linux-shaped record on a Darwin host is a platform mismatch.
  t56_schema2(_s2_linux "Linux")
  t56_ctx_case(ctx-schema2-foreign-system "${_s2_linux}")
  run_case(t56_ctx_schema2_foreign_system EXPECT_EXIT FAIL
    EXPECT_STDERR "native check platform records system='Linux'"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
else()
  string(JSON _bad SET "${_s2}" sdk "\"/tmp\"")
  t56_ctx_case(ctx-schema2-linux-sdk "${_bad}")
  run_case(t56_ctx_schema2_linux_sdk EXPECT_EXIT FAIL
    EXPECT_STDERR "sdk"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
  string(JSON _bad SET "${_s2}" sdk_source "\"default\"")
  t56_ctx_case(ctx-schema2-linux-sdk-source "${_bad}")
  run_case(t56_ctx_schema2_linux_sdk_source EXPECT_EXIT FAIL
    EXPECT_STDERR "not-applicable"
    COMMAND "${CHECKS_BINARY}" --kind context --build "${_fixture_dir}" --validate-only)
endif()
```

`--validate-only` does not exist yet; this task adds it to `--kind context`. With it, the kind loads and validates the context and exits 0, without running the compile controls. The controls need real compilers and are covered by the existing kind runs.

`EXPECT_STDERR` is a plain substring match (`string(FIND)`, `migration_T56_check.cmake:45-46`). The platform and capability needles above are already the verbatim Python texts (`native_checks.py:191-216`, `:340-449`, with Python `repr` quoting rendered as `'…'`). The remaining short needles (`sysroot …`, `stdlib`, `numerics`, `sdk`, `not-applicable`) are the minimum; when Step 3 copies the Python messages verbatim, tighten each to the full Python message text.

- [ ] **Step 2: Run them, expect failure**

```bash
cmake --build build/native-release -j10 --target bike-native-checks
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
```

Expected: FAIL, because `--validate-only` is an unknown option and/or `t56_ctx_schema2_host` is rejected with "unsupported schema".

- [ ] **Step 3: Implement**

In `native/src/tools/check_context.cpp`:

(a) Schema dispatch replaces lines 172-179:

```cpp
    const Value *schema = member(context, "schema_version");
    const auto *schema_number =
        schema != nullptr ? std::get_if<io::Number>(&schema->data) : nullptr;
    const std::string schema_text =
        schema_number != nullptr ? schema_number->lexeme : std::string{};
    if (schema_text != "1" && schema_text != "2")
        throw CheckError("native check context has an unsupported schema");
    // Schema 1 cannot express a no-SDK build: native_checks.py validates it
    // with Darwin semantics (tools/native_checks.py:178-182, 234-235).
    const std::string system =
        schema_text == "2" ? validate_platform_record(context) : "Darwin";
    set_member(context, "platform_system", vstr(system));
```

(b) Add, in the anonymous namespace above `load_context`, a port of `tools/native_checks.py:191-216` (platform), `:218-233` (sysroot), `:278-309` (stdlib), `:311-315` (numerics), `:317-338` (policy) and `:340-449` (capabilities and frontend tools). It must use the **exact** error strings of those Python lines:

```cpp
struct HostIdentity {
    std::string system;
    std::string machine;
    long pointer_bits;
};

HostIdentity host_identity() {
    struct utsname u {};
    if (::uname(&u) != 0)
        throw CheckError("native check context: uname() failed");
    return {u.sysname, u.machine, static_cast<long>(sizeof(void *) * 8)};
}

const std::string &required_text(const Value &parent, std::string_view key,
                                 const std::string &label) {
    const Value *v = member(parent, key);
    const auto *s = v != nullptr ? std::get_if<std::string>(&v->data) : nullptr;
    if (s == nullptr || s->empty())
        throw CheckError(label + " is missing or empty");
    return *s;
}

const Value &required_object(const Value &parent, std::string_view key,
                             const std::string &label) {
    const Value *v = member(parent, key);
    if (v == nullptr || !std::holds_alternative<Object>(v->data))
        throw CheckError(label + " is missing or not an object");
    return *v;
}

bool required_bool(const Value &parent, std::string_view key, const std::string &label) {
    const Value *v = member(parent, key);
    const auto *b = v != nullptr ? std::get_if<bool>(&v->data) : nullptr;
    if (b == nullptr)
        throw CheckError(label + " must be a boolean");
    return *b;
}

// Returns the validated platform system ("Darwin" | "Linux") and normalizes
// sdk/sdk_requested/sysroot in place. Mirrors tools/native_checks.py
// validate_platform_context; error texts are copied from it verbatim.
// Python repr() of a str-or-None, the quoting native_checks.py messages use.
std::string py_repr(const Value *v) {
    if (v == nullptr || io::is_null(*v)) return "None";
    if (const auto *s = std::get_if<std::string>(&v->data)) return "'" + *s + "'";
    return "<non-string>";
}

std::string validate_platform_record(Value &context) {
    const HostIdentity host = host_identity();
    const Value *platform = member(context, "platform");
    if (platform == nullptr || !std::holds_alternative<Object>(platform->data))
        throw CheckError("native check context has no platform record");
    const Value *system_value = member(*platform, "system");
    const auto *system = system_value != nullptr
                             ? std::get_if<std::string>(&system_value->data) : nullptr;
    if (system == nullptr || (*system != "Darwin" && *system != "Linux"))
        throw CheckError("native check context has an unsupported platform system: " +
                         py_repr(system_value));
    const Value *host_system = member(*platform, "host_system");
    const auto *host_system_text = host_system != nullptr
                                       ? std::get_if<std::string>(&host_system->data) : nullptr;
    if (*system != host.system || host_system_text == nullptr || *host_system_text != host.system)
        throw CheckError("native check platform records system=" + py_repr(system_value) +
                         " host_system=" + py_repr(host_system) + " on a '" + host.system +
                         "' host");
    const Value *machine = member(*platform, "machine");
    const auto *machine_text = machine != nullptr ? std::get_if<std::string>(&machine->data) : nullptr;
    if (machine_text == nullptr || *machine_text != host.machine)
        throw CheckError("native check platform machine " + py_repr(machine) +
                         " does not match host machine '" + host.machine + "'");
    // pointer_bits ("native check platform pointer width does not match the host"),
    // sysroot, SDK (Darwin: sdk dir, sdk_requested non-empty,
    // sdk_source in {explicit, default}, sysroot == sdk; Linux: sdk and
    // sdk_requested null or "", sdk_source == "not-applicable", both
    // normalized to null), stdlib, numerics.backend, policy, capabilities
    // and capabilities.frontend_tools follow the same pattern: one Python
    // rule = one C++ check, one verbatim message.
    // ...
    return *system;
}
```

The generic `required_text` / `required_object` / `required_bool` helpers are for rules where `native_checks.py` has **no** specific message; wherever Python raises a specific `CheckError`, the C++ throw carries that text (Python `repr` of a `str` renders as `'…'`, of `None` as `None`).

The `// ...` marks where Step 3 continues the same pattern. Each remaining rule is a numbered requirement below and must exist in the final code:
1. `pointer_bits` must equal `host.pointer_bits`; otherwise `native check platform pointer width does not match the host`.
2. `sysroot` is null, or a non-empty string naming an existing directory. Otherwise: `sysroot must be null or a directory path`.
3. On Darwin, move the existing SDK block (old lines 204-230) into the Darwin branch unchanged, and add: when schema is 2, the canonical `sysroot` must equal the canonical `sdk`.
4. On Linux: `sdk` and `sdk_requested` must each be absent, null or `""`; `sdk_source` must be `not-applicable`. Then `set_member(context, "sdk", vnull())` and the same for `sdk_requested`.
5. `stdlib.name` is in {`libc++`, `libstdc++`}, and Darwin requires `libc++`. `stdlib.version` is non-empty. `stdlib.hardening.mode` is in {`EXTENSIVE`, `FAST`} for libc++ or {`ASSERTIONS`} for libstdc++. `stdlib.hardening.effective` is `true`.
6. `numerics.backend` is a non-empty string.
7. `policy.required_flags` and `policy.accepted_warnings` are string arrays. `policy.rejected_warnings` is an array of `{flag: string, reason: string}`.
8. `capabilities` must be an object (`native check context has no capabilities record`). `asan`, `rtsan` and `coverage` must **each be present** (`native check context has no {capability} capability record`), in this order of checks (`native_checks.py:340-449`):
   - `requested`, `supported` and `enabled` are booleans; `enabled` without `supported` fails first (`… is enabled without support evidence`), then `enabled` without `requested` (`… is enabled without a requested lane`);
   - `evidence` is a non-empty string, or a non-empty object mapping string names to string values; otherwise the matching Python message (`empty evidence record` / `evidence must map names to strings` / `has no evidence record`);
   - the `runtime` key is mandatory (`… has no runtime record`); a non-null runtime is an existing file (`capability {capability} runtime is not a file: …`);
   - for `asan`/`rtsan` only: an enabled lane requires a non-null `runtime`; `compiler_id` is a non-empty string; `target` is null or a non-empty string.
   `capabilities.frontend_tools` is mandatory (`native check context has no frontend_tools capability record`); each `<k>.configured` is a string, and `.located` is null or an existing executable path (symlink preserved).

Schema 1 contexts skip `validate_platform_record` and keep today's SDK block unchanged.

(c) Add `--validate-only` to the `context` kind's argument handling (in `checks_main.cpp` / the context-kind entry point; follow how the existing flags are parsed there). When it is set, the kind returns 0 right after `load_context` succeeds.

Add `#include <sys/utsname.h>` at the top of `check_context.cpp`.

- [ ] **Step 4: Run, expect pass**

```bash
cmake --build build/native-release -j10 --target bike-native-checks
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
build/native-release/bin/bike-native-checks --kind context --build native/build --validate-only; echo "exit=$?"
```

Expected:
- `migration_T56` passes;
- the real schema-2 binding context validates (`exit=0`).

The `else()` (Linux-host) cases first execute in the container at Task 2.07, where the root graph — the only graph that registers `migration_T56` — first configures on Linux.

- [ ] **Step 5: Sweeps that only needed the loader**

```bash
build/native-release/bin/bike-native-checks --kind headers --build native/build
cmake --build native/build --target check_frontends
```

Expected: both pass, or fail on a later, schema-unrelated rule. In that case record the message; Tasks 1.07–1.09 cover the sysroot, tool and controls rules.

- [ ] **Step 6: Commit**

```bash
git add native/src/tools/check_context.cpp native/src/tools/checks_internal.hpp native/src/tools/checks_main.cpp native/tests/migration/migration_T56_check.cmake
git commit -m "fix(cpp): T56 checker loads schema-2 contexts with platform validation

The linux-macos-port merge made the binding graph emit schema 2; the C++
orchestrator accepted only schema 1, so every sweep failed before running.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.07: Checker: sysroot-aware compile-context validation and tool resolution parity

**Files:**
- Modify: `native/src/tools/check_context.cpp`:
  - `validate_compilation_context`, about line 609 (configured sysroot);
  - `gcc_context_args`, about 1180-1245 (`--sysroot` forwarding);
  - `resolve_tool`, about 1055-1095 (candidate chain).
- Test: `native/tests/migration/migration_T56_check.cmake`

**Interfaces:**
- Consumes: Task 1.06's normalized context (`platform_system`, null `sdk` on Linux).
- Produces:
  - `std::optional<fs::path> configured_sysroot(const Value &context)`;
  - tool resolution order: override → configured-compiler sibling → `tool_defaults` → `capabilities.frontend_tools.<key>.located` → Darwin brew candidates (Darwin or schema 1 only).

**Review focus:** these semantics must match `tools/native_checks.py:724-747` (sysroot), `:995-1019` (tools), `:1332-1431` (forwarded pairs) and `tools/native_toolchain.py:162-230` (candidate lists) exactly. The Linux branches are compiled but not executed until Task 2.07 (see Step 2).

- [ ] **Step 1: Write the cases (one Linux red case, one macOS guard)**

Append to `migration_T56_check.cmake`:

```cmake
# A compile entry with -isysroot while the context records no sysroot
# (Linux semantics) must fail closed, never crash on a null sdk.
if(NOT _t56_system STREQUAL "Darwin")
  t56_ctx_case(ctx-linux-sysroot-arg "${_s2}")
  file(COPY "${_sources}" DESTINATION "${_fixture_dir}")
  file(READ "${_ccdb}" _db)
  string(REGEX REPLACE "\"command\": \"" "\"command\": \"-isysroot /tmp " _db "${_db}")
  file(WRITE "${_fixture_dir}/compile_commands.json" "${_db}")
  run_case(t56_ctx_linux_sysroot_arg EXPECT_EXIT FAIL
    EXPECT_STDERR "sysroot"
    COMMAND "${CHECKS_BINARY}" --kind frontends --build "${_fixture_dir}")
endif()

# Tool override must win over defaults (resolve_tool order). tool_overrides
# is keyed by the env-var name (CLANG_TIDY, ANALYZER_CLANG, CPPCHECK, GXX;
# check_context.cpp:1058-1075, BikeCheckContext.cmake:485-495).
t56_ctx_case(ctx-tool-override "${_context_json}")
string(JSON _ovr SET "${_context_json}" tool_overrides CLANG_TIDY "\"/nonexistent/clang-tidy\"")
file(WRITE "${_fixture_dir}/native_check_context.json" "${_ovr}")
file(COPY "${_sources}" "${_ccdb}" DESTINATION "${_fixture_dir}")
run_case(t56_tool_override_missing EXPECT_EXIT FAIL
  EXPECT_STDERR "/nonexistent/clang-tidy"
  COMMAND "${CHECKS_BINARY}" --kind tidy --build "${_fixture_dir}")
```

If the compile-database JSON key is `"arguments"` rather than `"command"` in the produced `compile_commands.json`, inject `"-isysroot","/tmp",` after the compiler element instead. Inspect with `head -c 400 build/native-release/compile_commands.json`.

- [ ] **Step 2: Run on macOS (guard green, Linux case deferred)**

```bash
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
```

Expected on macOS: `t56_tool_override_missing` **passes already** — today's `resolve_tool` honors `tool_overrides.CLANG_TIDY` and its message (`required tool is missing or not executable: <requested>`) contains the path. It is a guard that keeps the override first after the Step 3 rewrite, not a red test. Nothing else in this step is red on macOS.

The Linux case (`t56_ctx_linux_sysroot_arg`) cannot run before Phase 2: `migration_T56` and `bike-native-checks` exist only in the root graph (`native/cmake/Checks.cmake:16-20, 319-328`), and the root graph first configures on Linux at Task 2.07. It first executes in the container there; Task 2.07's acceptance includes `ctest -R '^migration_T56$'` in `/build/root-clang`.

- [ ] **Step 3: Implement**

```cpp
// tools/native_checks.py:728-747 — `context.get("sysroot") or context.get("sdk")`.
// A null configured sysroot admits no -isysroot/--sysroot argument at all.
std::optional<fs::path> configured_sysroot(const Value &context) {
    for (const std::string_view key : {std::string_view{"sysroot"}, std::string_view{"sdk"}}) {
        const Value *v = member(context, key);
        if (v == nullptr) continue;
        if (const auto *s = std::get_if<std::string>(&v->data); s != nullptr && !s->empty())
            return canonical_path(*s);
    }
    return std::nullopt;
}
```

- At the old `canonical_path(std::get<std::string>(member(context, "sdk")->data))` site (about line 609), use `configured_sysroot(context)`.
- When it is `nullopt`, a compile entry that carries any of `-isysroot X`, `--sysroot X`, `--sysroot=X` fails with the Python message from `native_checks.py:724-747`. An empty sysroot selection is then **valid**; delete the "empty list is a mismatch" rule for that case.
- In `gcc_context_args`, forward the pairs `-isysroot`, `--sysroot`, the joined `--sysroot=` and `-arch`, as `_FORWARD_PAIRS` (`native_checks.py:1332-1344`). Check each forwarded sysroot against `configured_sysroot` (`_check_forwarded_sysroot`, `:1367-1379`).
- Rewrite `resolve_tool` to the five-step chain in Interfaces, keeping `tool_overrides` keyed by env-var name. Port `sibling_candidates` and `darwin_tool_candidates` from `tools/native_toolchain.py:162` and `:201` (not `native_checks.py`) with identical candidate lists and order.

- [ ] **Step 4: Run, expect pass; then the sweeps on both macOS graphs**

```bash
cmake --build build/native-release -j10 --target bike-native-checks
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
cmake --build native/build --target check_frontends check_tidy
```

Expected: all exit 0.

- [ ] **Step 5: Commit**

```bash
git add native/src/tools/check_context.cpp native/tests/migration/migration_T56_check.cmake
git commit -m "fix(cpp): T56 sysroot-aware compile contexts and tool resolution parity

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.08: Checker: platform-branched contract controls

**Files:**
- Modify: `native/src/tools/check_contracts.cpp`:
  - the context-control runner, about 511-830;
  - Darwin SDK controls at about 627-817;
  - the hardening compile at about 945.
- Modify: `native/src/tools/checks_main.cpp` (`--list-controls`; the parser rejects unknown arguments at :145-147; update `usage_line` :39-46 and the `--help` text :106-113)
- Test: `native/tests/migration/migration_T56_check.cmake`

**Interfaces:**
- Consumes: `platform_system` from Task 1.06.
- Produces: `--kind context` runs exactly:
  - the Darwin SDK controls `explicit_sdk`, `missing_sdk`, `default_sdk`, `default_sdk_fallback` (+ `libcpp_hardening`) on Darwin — the C++ already emits exactly these names;
  - the Linux controls `no_sdk`, `bad_sysroot`, `rejected_osx_sysroot`, `no_apple_tool`, `sysroot_preservation` (+ the `stdlib-hardening-*` records from `_run_stdlib_hardening`, Task 1.09) on Linux.
  
  Control names, argv and expectations are ported from `tools/native_checks.py:2178-2310` (`_run_darwin_controls`) and `:2468-2592` (`_run_linux_controls`).

**Review focus:**
- No Darwin control runs on Linux.
- No `std::get<std::string>` touches a null `sdk`.
- `-isysroot` is passed only when a sysroot exists.

- [ ] **Step 1: Failing case (on macOS asserts the Darwin set; the Linux set first executes in the container at Task 2.07, because `migration_T56` and `bike-native-checks` exist only in the root graph, which cannot configure on Linux before then)**

```cmake
run_case(t56_context_controls_platform EXPECT_EXIT 0
  COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}" --list-controls)
```

Add `--list-controls`, which prints one control name per line and exits 0. Then:

```cmake
execute_process(COMMAND "${CHECKS_BINARY}" --kind context --build "${BUILD_DIR}" --list-controls
  OUTPUT_VARIABLE _controls RESULT_VARIABLE _rc)
if(_t56_system STREQUAL "Darwin")
  set(_want "explicit_sdk;missing_sdk;default_sdk;default_sdk_fallback")
  set(_forbid "no_sdk;no_apple_tool;bad_sysroot;rejected_osx_sysroot;sysroot_preservation")
else()
  set(_want "no_sdk;bad_sysroot;rejected_osx_sysroot;no_apple_tool;sysroot_preservation")
  set(_forbid "explicit_sdk;missing_sdk;default_sdk;default_sdk_fallback")
endif()
string(STRIP "${_controls}" _controls)
string(REPLACE "\n" ";" _control_list "${_controls}")
foreach(_c IN LISTS _want)
  list(FIND _control_list "${_c}" _p)
  if(_p EQUAL -1)
    set(_failures "${_failures}\n  t56_controls: missing ${_c}")
  endif()
endforeach()
foreach(_c IN LISTS _forbid)
  list(FIND _control_list "${_c}" _p)
  if(NOT _p EQUAL -1)
    set(_failures "${_failures}\n  t56_controls: forbidden ${_c}")
  endif()
endforeach()
```

The names above are the exact `sdk_controls[...]` keys of `native_checks.py` (verified 2026-10-20). The match is per line (`list(FIND)`), because `default_sdk` is a prefix of `default_sdk_fallback`.

- [ ] **Step 2: Run, expect failure** (`--list-controls` unknown).

```bash
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
```

- [ ] **Step 3: Implement**
  - Wrap the Darwin controls in `if (platform_system == "Darwin")`.
  - Port the five Linux controls as one function `run_linux_controls(const Value &context, …)` mirroring `_run_linux_controls`: same child-configure argv, same expected exit/needles, same `passed`/`rejected`/`failed` values per key.
  - Gate the hardening compile's `-isysroot` on `configured_sysroot(context)` (Task 1.07; declare it in `checks_internal.hpp`).
  - Implement `--list-controls` from the same table the runner iterates, so the list cannot drift from what runs. Accept it only with `--kind context` (as `--repo-root` is accepted only for `contracts`); add it to `usage_line` and `--help` so `t56_help` still passes.

- [ ] **Step 4: Run, expect pass**

```bash
cmake --build build/native-release -j10 --target bike-native-checks
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
build/native-release/bin/bike-native-checks --kind context --build native/build
```

Expected: exit 0.

- [ ] **Step 5: Commit**

```bash
git add native/src/tools/check_contracts.cpp native/src/tools/checks_internal.hpp native/src/tools/check_context.cpp native/src/tools/checks_main.cpp native/tests/migration/migration_T56_check.cmake
git commit -m "fix(cpp): T56 runs Darwin or Linux context controls by platform

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.09: Checker: child-configure argv and stdlib probe parity

**Files:**
- Modify: `native/src/tools/check_contracts.cpp`:
  - child configure argv, about 585-598;
  - cmake resolution, about 519-540;
  - libstdc++ hardening, about 1113-1119.
- Test: direct L1 check on the binding context (Steps 1/4; no CTest twin, see below) + existing `migration_T56` stays green

**Interfaces:**
- Consumes: `python_executable` (string or null) and the root-only `cmake` field (string or absent).
- Produces: the child configure is
  - `<cmake> -S <src> -B <tmp> -DCMAKE_CXX_COMPILER=<path> [-DPython_EXECUTABLE=<python_executable> when non-null] …`;
  - where `<cmake>` = the context `cmake` field if present, else `PATH` lookup of `cmake`. Never `uv run cmake`.
  
  A libstdc++ context runs the stdlib-hardening probe (`native_checks.py` `_stdlib_probe_spec`, about 1687; `_run_stdlib_hardening`, about 2593) instead of reporting `not-applicable`.

**Review focus:** no `uv` or interpreter dependency remains on the path from `--kind context` to the child configure, except the explicit `-DPython_EXECUTABLE` for binding contexts.

`uv` is reached only in `run_context_checks` (`--kind context`, `check_contracts.cpp:482`; resolved at `:532-539`, used in the argv at `:577-583`), and only for a context without `dependency_root` — that is, the binding context (`native/build/native_check_context.json`: `dependency_root: null`, `cmake: null`). The root context records both, so a `migration_T56` case on `BUILD_DIR=build/native-release` cannot go red (the existing `t56_no_interpreter_reach`, `migration_T56_check.cmake:240-247`, already runs `--kind contracts` with an empty `PATH` and passes). A T56 fixture cannot express the binding case either: it would have to strip `cmake`/`dependency_root`/`configure_source_dir` and then configure `<repo>/native`, which needs the venv. The red→green check is therefore a direct L1 command on the binding context.

- [ ] **Step 1: Failing check (direct, on the binding context)**

`uv` and `cmake` share `/opt/homebrew/bin` on this host, so the PATH gets a scratch directory holding only a `cmake` link:

```bash
mkdir -p /tmp/t109-bin && ln -sf "$(command -v cmake)" /tmp/t109-bin/cmake
env PATH=/tmp/t109-bin:/usr/bin:/bin \
  build/native-release/bin/bike-native-checks --kind context --build native/build; echo "exit=$?"
```

- [ ] **Step 2: Expect failure**: non-zero exit, stderr `required tool is unavailable: uv`.
- [ ] **Step 3: Implement** the argv rule above (the Python argv is `native_checks.py:2007-2018`: `[cmake, -S, native_root, -B, case_build, -DPython_EXECUTABLE=<context python_executable>]`, no `uv`) and the libstdc++ probe, porting the probe source text and the expected-diagnostic needles verbatim from `native_checks.py` (`_stdlib_probe_spec` about 1687; `_run_stdlib_hardening` :2593, records `stdlib-hardening-compile` / `-assertion` / `-benign`). Delete the `uv` resolution at `:532-539`.
- [ ] **Step 4: Run, expect pass**, then the context and contracts sweeps on both graphs:

```bash
cmake --build build/native-release -j10 --target bike-native-checks
env PATH=/tmp/t109-bin:/usr/bin:/bin \
  build/native-release/bin/bike-native-checks --kind context --build native/build; echo "exit=$?"
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
build/native-release/bin/bike-native-checks --kind contracts --build native/build --repo-root "$PWD"
build/native-release/bin/bike-native-checks --kind contracts --build build/native-release --repo-root "$PWD"
```

Expected: `exit=0`, then all exit 0. Record the Step 1/Step 4 outputs in the evidence log (this check has no CTest twin). The libstdc++ probe is compiled on macOS but first executes in the container at Task 2.07 (root graph on Linux).

- [ ] **Step 5: Commit**

```bash
git add native/src/tools/check_contracts.cpp migration/cpp-linux-completion-evidence.md
git commit -m "fix(cpp): T56 child configure without uv; libstdc++ hardening probe

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### CHECKPOINT 1-A (L2, macOS only): every sweep green on both graphs

- [ ] Run:

```bash
cmake --build native/build -j10 && ctest --test-dir native/build -j8 --output-on-failure
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -j8 --output-on-failure
tools/run_native_checks.sh native/build
tools/run_native_checks.sh build/native-release
```

Expected: all exit 0. Append the evidence section `## Checkpoint 1-A` to `migration/cpp-linux-completion-evidence.md` and commit it as `docs(migration): checkpoint 1-A evidence`.

---

### Task 1.10: Numerics probe and record: add `dgesv`, `dsyevd`, `ddot_ilp64`, `zdotu_sub`; commit the Linux record

**Files:**
- Modify: `tools/native_dependencies.py`:
  - `_NUMERICS_OPS` / `_NUMERICS_BLAS_OPS` at lines 177-178, plus a new `_NUMERICS_BLAS_ILP64_OPS`;
  - `_NUMERICS_WIDTHS` (183) and its use at 261;
  - the record validators at 290-350, including the two "exactly the four qualified" messages (297, 347): reword them to the op count;
  - `_OP_SYMBOL_TAIL` (734-737) and the import classifier at about 897;
  - `_numerics_abi_probe_source` (1005-1050): add a second typedef `blas64_int = int64_t` and the four new prototypes;
  - the Darwin hard-coded `symbols = {...}` (1706-1711) and its export checks (1712-1735).
- Modify: `native/cmake/BikeNumerics.cmake`:
  - both `foreach(_op ddot dgemv dgemm dgelsd)` lists (358, 593);
  - the per-op spelling rules (359-389).
- Modify: `native/src/platform/numerics_symbols.hpp.in`
- Modify (generated, via the tool only): `native/numerics.lock.json`
- Test:
  - extend `tests/portability/test_native_dependencies.py`;
  - update `tests/portability/test_numerics_discovery.py`. Its record and `numpy_imports` fixtures spell the four-op set, including the 13 `dgelsd` fixtures.
  - update `tests/portability/test_provider_identity.py` (it reads the lock).

**Op classes (the change this task makes):**

| Class | Ops | Darwin spelling | Linux spelling | Integer width |
|---|---|---|---|---|
| `_NUMERICS_BLAS_OPS` (unchanged) | `ddot dgemv dgemm` | `cblas_<op>` | `scipy_cblas_<op>64_` | Darwin 32, Linux 64 |
| `_NUMERICS_BLAS_ILP64_OPS` (new) | `ddot_ilp64 zdotu_sub` | `cblas_<stem>$NEWLAPACK$ILP64` | `scipy_cblas_<stem>64_` | 64 on both |
| LAPACK (implicit: the rest) | `dgelsd dgesv dsyevd` | `<op>$NEWLAPACK$ILP64` | `scipy_<op>_64_` | 64 on both |

- `<stem>` is the op name with a trailing `_ilp64` removed, so `ddot_ilp64` → `cblas_ddot$NEWLAPACK$ILP64` on Darwin and `scipy_cblas_ddot64_` on Linux.
- On Linux, `ddot_ilp64` and `ddot` resolve to the same export. That is fine: the face distinguishes them by contract (whether the `0. +` seed is applied), not by symbol.

**Why `ddot_ilp64` is a separate op.**
- NumPy 2.5.2 on macOS imports `_cblas_ddot$NEWLAPACK$ILP64`, and `geometry/specs.hpp:253-260` binds that symbol by hand.
- The committed Darwin record instead pins `ddot: cblas_ddot` at `blas_integer_bits: 32`. `blas::ddot` uses that legacy symbol, and the existing Python parity was measured against it.
- The probe never checked that a recorded symbol is NumPy's own import (1727-1735 only check that NumPy's imports are exported). So the hand-bound ILP64 ddot had no record at all.
- Keeping `ddot` as it is and adding `ddot_ilp64` qualifies both bindings without changing either one's arithmetic.
- The `ACCELERATE` width rule `(32, 64)` and `blas_integer_bits: 32` stay. Document them in the validator's docstring as "legacy CBLAS ops only; `_NUMERICS_BLAS_ILP64_OPS` are 64-bit on every backend".

**Interfaces:**
- Consumes these NumPy 2.5.2 imports (measured with `nm -u` on the macOS venv):
  - `_multiarray_umath`: `cblas_ddot$NEWLAPACK$ILP64` and `cblas_zdotu_sub$NEWLAPACK$ILP64` (`DOUBLE_dot`, `CDOUBLE_dot`);
  - `_umath_linalg`: `dgesv$NEWLAPACK$ILP64` (solve) and `dsyevd$NEWLAPACK$ILP64` (eigh).
- Produces:
  - `native/numerics.lock.json` records for `darwin-arm64` **and** `linux-aarch64`, each with `symbols.{ddot,dgemv,dgemm,dgelsd,dgesv,dsyevd,ddot_ilp64,zdotu_sub}`;
  - generated macros `BIKE_NUMERICS_SYM_DGESV`, `BIKE_NUMERICS_SYM_DSYEVD`, `BIKE_NUMERICS_SYM_DDOT_ILP64`, `BIKE_NUMERICS_SYM_ZDOTU_SUB`.
- New validator rule: every op in `_NUMERICS_BLAS_ILP64_OPS` and every LAPACK op must be one of NumPy's own imports for that op. This closes the gap described above. Legacy `_NUMERICS_BLAS_OPS` keep their current rule, because NumPy does not import `cblas_ddot` on Darwin.

**Review focus:** the measured Darwin spellings for the new ops must equal the spellings hard-coded today. If they differ, that is a parity finding: stop and report.
- `_dgesv$NEWLAPACK$ILP64` in `kinematics/curves.cpp:28` and `numerics/filter.cpp:19`;
- `_dsyevd$NEWLAPACK$ILP64` in `model/mass.cpp:230`;
- `_cblas_zdotu_sub$NEWLAPACK$ILP64` in `numerics/filter.cpp:25`;
- `_cblas_ddot$NEWLAPACK$ILP64` in `geometry/specs.hpp:258-260`.

- [ ] **Step 1: Failing tests**

In `tests/portability/test_native_dependencies.py`, add:

```python
def test_numerics_ops_include_solve_eigh_and_ilp64_dots():
    """The probe must qualify every provider entry point the C++ port calls,
    not only the original four (BikeNumerics face, completion plan 1.10).
    The ILP64 dots are a separate class: on Darwin they are NEWLAPACK ILP64
    exports, while the legacy BLAS ops stay 32-bit cblas_*."""
    import native_dependencies as nd
    assert set(nd._NUMERICS_OPS) == {
        "ddot", "dgemv", "dgemm", "dgelsd", "dgesv", "dsyevd",
        "ddot_ilp64", "zdotu_sub"}
    assert set(nd._NUMERICS_BLAS_ILP64_OPS) == {"ddot_ilp64", "zdotu_sub"}
    assert not set(nd._NUMERICS_BLAS_ILP64_OPS) & set(nd._NUMERICS_BLAS_OPS)
    assert "dgesv" not in nd._NUMERICS_BLAS_OPS


def test_op_symbol_tail_does_not_confuse_dgesvd_with_dgesv():
    """_umath_linalg also imports dgesvd; the end-anchored classifier must
    not file it under dgesv."""
    import native_dependencies as nd
    assert nd._OP_SYMBOL_TAIL.search("dgesvd$NEWLAPACK$ILP64") is None
    assert nd._OP_SYMBOL_TAIL.search("dgesv$NEWLAPACK$ILP64").group(1) == "dgesv"
    assert nd._OP_SYMBOL_TAIL.search(
        "cblas_zdotu_sub$NEWLAPACK$ILP64").group(1) == "zdotu_sub"
```

Before adding them, read the file's top imports and follow its import convention for `native_dependencies`.

```bash
uv run --frozen --group native python -m pytest -q tests/portability/test_native_dependencies.py -k 'ilp64_dots or dgesvd'
```

Expected: both tests FAIL with an `AttributeError` or an assertion error.

- [ ] **Step 2: Extend the probe**
  - Add the four ops and the `_NUMERICS_BLAS_ILP64_OPS` class from the table.
  - `_OP_SYMBOL_TAIL` gains the stems `dgesv|dsyevd|zdotu_sub` and stays end-anchored. NumPy's `ddot` imports feed both `ddot` and `ddot_ilp64`; the classifier picks the ILP64 spelling for `ddot_ilp64`.
  - Spelling validators, in both backends' branches at 300-350, follow the table.
    - The existing `ACCELERATE` BLAS rule (`not name.startswith("cblas_") or "NEWLAPACK" in name → raise`) applies to `_NUMERICS_BLAS_OPS` only.
    - The ILP64 class must end in `$NEWLAPACK$ILP64` on Darwin and in `64_` on Linux.
  - `_numerics_abi_probe_source` declares the ILP64 class with `blas64_int` (`int64_t`), never with `blas_int`. An ILP64 export declared with 32-bit ints is the stack corruption the probe exists to prevent.
  - The Darwin branch (1706-1711) adds the four hard-coded spellings. Its export check uses `blas_exports` for the BLAS classes and `lapack_exports` for the three LAPACK ops; today only `dgelsd` goes to LAPACK.
  - Extend the probe's link/run control so it links all eight symbols against the selected provider.

- [ ] **Step 3: Extend CMake**
  - In `BikeNumerics.cmake`, both `foreach(_op …)` lists become `ddot dgemv dgemm dgelsd dgesv dsyevd ddot_ilp64 zdotu_sub`.
  - The rule block (359-389) gains:
    - an `elseif(_op MATCHES "^(dgesv|dsyevd)$")` branch with the `dgelsd` grammar;
    - an `elseif(_op MATCHES "^(ddot_ilp64|zdotu_sub)$")` branch. It strips `_ilp64` into `_stem` and requires `^cblas_${_stem}\\$NEWLAPACK\\$ILP64$` on Accelerate, or `STREQUAL "scipy_cblas_${_stem}64_"` on scipy-openblas.
  - The legacy `else()` branch keeps `^cblas_${_op}$`.
  - In `numerics_symbols.hpp.in`, add:

```c
#define BIKE_NUMERICS_SYM_DGESV "@_numerics_asm_dgesv@"
#define BIKE_NUMERICS_SYM_DSYEVD "@_numerics_asm_dsyevd@"
#define BIKE_NUMERICS_SYM_DDOT_ILP64 "@_numerics_asm_ddot_ilp64@"
#define BIKE_NUMERICS_SYM_ZDOTU_SUB "@_numerics_asm_zdotu_sub@"
```

- [ ] **Step 4: Regenerate the Darwin record (macOS) and write the Linux record (container)**

`write_numerics_record` (2449-2474) loads the existing file through `load_numerics_lock`. That validator requires `set(symbols) == set(_NUMERICS_OPS)`, so the committed four-symbol record is rejected. Delete the file before the first write; regeneration by the tool is not hand-editing.

```bash
rm native/numerics.lock.json
uv run --frozen --group native python tools/native_dependencies.py --python .venv/bin/python3 --write-numerics-record native/numerics.lock.json
tools/linux/container.sh sync
tools/linux/container.sh exec uv run --frozen --group native python tools/native_dependencies.py --python /opt/bike-venv/bin/python3 --write-numerics-record native/numerics.lock.json
tools/linux/container.sh pull native/numerics.lock.json
git diff native/numerics.lock.json
```

The container write keeps the Darwin record: the tool keeps other platforms' records (2452-2462). Sync after the macOS write and pull only after the container write. In the container, `uv run` resolves to `/opt/bike-venv`, because `UV_PROJECT_ENVIRONMENT` is set in the image ENV (Task 1.02). The `--python` argument still names the interpreter whose NumPy is measured.

Expected diff:
- `darwin-arm64.numerics.symbols` gains `dgesv`, `dsyevd`, `ddot_ilp64` and `zdotu_sub` with the Review-focus spellings (without the leading `_`). `ddot`, `dgemv`, `dgemm` and `dgelsd` keep their bytes.
- A new `linux-aarch64` record has:
  - `backend: NUMPY_OPENBLAS` and `platform.libc: "glibc"`;
  - a `libscipy_openblas64_-bb25d554.so` provider with sha256 `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`;
  - the eight `scipy_*` symbols.

- [ ] **Step 5: Run tests + configure both platforms**

```bash
uv run --frozen --group native python -m pytest -q tests/portability/test_native_dependencies.py tests/portability/test_numerics_discovery.py tests/portability/test_provider_identity.py
cmake -S native -B native/build && cmake --build native/build -j10
tools/linux/container.sh sync
tools/linux/container.sh configure native-clang native -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
tools/linux/container.sh configure native-gcc native -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
```

Expected:
- pytest passes;
- the macOS build passes;
- both Linux configures succeed and report that the `linux-aarch64` record matches the live measurement.

- [ ] **Step 6: Commit**

```bash
git add tools/native_dependencies.py native/cmake/BikeNumerics.cmake native/src/platform/numerics_symbols.hpp.in native/numerics.lock.json tests/portability/test_native_dependencies.py tests/portability/test_numerics_discovery.py tests/portability/test_provider_identity.py
git commit -m "build(native): qualify dgesv/dsyevd/ILP64 dots; add linux-aarch64 numerics

Five call sites bind Accelerate symbols directly; the provider face must
cover them before they can move behind it (Linux link failure). The ILP64
dots get their own op class: NumPy imports the NEWLAPACK ILP64 ddot on
macOS while the legacy cblas_ddot binding stays as measured.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.11: Numerics face and backends for the new operations

**Files:**
- Modify: `native/src/cblas_abi.hpp`:
  - declare `ddot_raw` and `zdotu_sub`;
  - rewrite the 22-30 comment ("macOS links Accelerate's legacy 32-bit cblas_*"): it must name the two ILP64 exceptions.
- Modify: `native/src/lapack_abi.hpp` (declare `dgesv`, `dsyevd`)
- Modify: `native/src/platform/blas_accelerate.cpp`:
  - add `#include "numerics_symbols.hpp"`;
  - rewrite the header comment at 7-13 ("exact export spellings are also pinned by the dependency probe (numerics.symbols: cblas_ddot/cblas_dgemv/cblas_dgemm)") to list the ILP64 ops as well.
- Modify: `native/src/platform/blas_numpy_openblas.cpp`
- Modify: `native/src/platform/lapack_accelerate.cpp`, `native/src/platform/lapack_numpy_openblas.cpp`
- Test:
  - `native/tests/test_platform_blas.cpp` and `native/tests/test_platform_lapack.cpp`;
  - CTest registers them as `native_platform_blas_tests` and `native_platform_lapack_tests`, in the binding graph only (`native/CMakeLists.txt:454-464`, `519-520`). The root graph has no platform tests.

**Interfaces:**
- Consumes: the `BIKE_NUMERICS_SYM_*` macros from Task 1.10, including `BIKE_NUMERICS_SYM_DDOT_ILP64`.
- Produces (exact signatures, used by Task 1.12):

```cpp
namespace blas {
    // np.dot WITHOUT numpy's `0. +` seed: the bare provider fold. Only
    // geometry/specs.hpp's np_dot/np_norm use it; its -0.0 results are part
    // of the captured kinematics fixtures. Every other dot uses blas::ddot.
    [[nodiscard]] double ddot_raw(int n, const double *x, int incx,
                                  const double *y, int incy) noexcept;
    // np.convolve's complex inner loop: CDOUBLE_dot -> cblas_zdotu_sub.
    // x, y, out point at interleaved (re, im) binary64 pairs.
    void zdotu_sub(int n, const void *x, int incx, const void *y, int incy,
                   void *out) noexcept;
}
namespace numeric_backend {
    void dgesv(const integer *n, const integer *nrhs, double *a,
               const integer *lda, integer *ipiv, double *b,
               const integer *ldb, integer *info) noexcept;
    void dsyevd(const char *jobz, const char *uplo, const integer *n,
                double *a, const integer *lda, double *w, double *work,
                const integer *lwork, integer *iwork, const integer *liwork,
                integer *info) noexcept;
}
```

**Review focus:**
- On Darwin, `ddot_raw` must bind `BIKE_NUMERICS_SYM_DDOT_ILP64`, which is `_cblas_ddot$NEWLAPACK$ILP64`. That is the symbol `geometry/specs.hpp:258-260` uses today, and it is **not** the legacy `cblas_ddot` behind `blas::ddot`. Swapping them is an unapproved arithmetic change.
- No hand-written asm label may remain in `blas_accelerate.cpp` for the new ops. Every new binding goes through a probe-qualified macro.

- [ ] **Step 0: Measure the bare ILP64 fold's signed-zero behaviour on both platforms**

The sign of a bare fold of `(-0.0)·(+1.0)` at `n=1` depends on how the provider seeds its accumulator. Nothing in the repo measures it. Measure it before any test asserts it.

```bash
cat > /tmp/ddot_sign.py <<'PY'
import ctypes, ctypes.util, struct, sys
x = (ctypes.c_double * 1)(-0.0); y = (ctypes.c_double * 1)(1.0)
if sys.platform == "darwin":
    lib = ctypes.CDLL("/System/Library/Frameworks/Accelerate.framework/Accelerate")
    fn = getattr(lib, "cblas_ddot$NEWLAPACK$ILP64")
else:
    import numpy, pathlib
    so = next(pathlib.Path(numpy.__file__).parent.parent.glob("numpy.libs/libscipy_openblas64_*.so"))
    fn = ctypes.CDLL(str(so)).scipy_cblas_ddot64_
fn.restype = ctypes.c_double
fn.argtypes = [ctypes.c_int64, ctypes.c_void_p, ctypes.c_int64, ctypes.c_void_p, ctypes.c_int64]
r = fn(1, x, 1, y, 1)
print(f"{struct.unpack('<Q', struct.pack('<d', r))[0]:016x}")
PY
uv run --frozen --group native python /tmp/ddot_sign.py
tools/linux/container.sh exec bash -c 'cat > /tmp/ddot_sign.py' < /tmp/ddot_sign.py
tools/linux/container.sh exec uv run --frozen --group native python /tmp/ddot_sign.py
```

Record both hex results in the evidence log: `8000000000000000` means `-0.0` and `0000000000000000` means `+0.0`. Step 1 asserts exactly what you measured, per platform. Do not assume the sign.

- [ ] **Step 1: Failing tests**

Add to `native/tests/test_platform_blas.cpp`:

```cpp
// ddot_raw is the bare provider fold (no numpy `0. +` seed); blas::ddot is
// seeded and therefore returns +0.0 for (-0.0)·(+1.0) on every provider.
// kDdotRawNegZero is the per-platform value measured in Task 1.11 Step 0
// (evidence log), not an assumption about the provider's accumulator.
void ddot_raw_matches_measured_fold_sign() {
#if defined(__APPLE__)
    constexpr bool kDdotRawNegZero = /* measured on darwin-arm64 */;
#else
    constexpr bool kDdotRawNegZero = /* measured on linux-aarch64 */;
#endif
    const double x[1] = {-0.0};
    const double y[1] = {1.0};
    require(std::signbit(blas::ddot_raw(1, x, 1, y, 1)) == kDdotRawNegZero,
            "ddot_raw fold sign matches the measured provider");
    require(!std::signbit(blas::ddot(1, x, 1, y, 1)), "ddot seeds +0.0");
}

// (1+2i)(3+4i) + (5+6i)(7+8i) = (-5+10i) + (-13+82i) = -18+92i, exact in binary64.
void zdotu_sub_exact_small_case() {
    const double x[4] = {1., 2., 5., 6.};
    const double y[4] = {3., 4., 7., 8.};
    double out[2] = {0., 0.};
    blas::zdotu_sub(2, x, 1, y, 1, out);
    require(out[0] == -18. && out[1] == 92., "zdotu_sub exact");
}
```

Fill in both `/* measured … */` placeholders with `true` or `false` from Step 0. If both platforms measure `+0.0`, the sign case cannot tell `ddot_raw` from `ddot`. Then drop the sign case and keep only the `ddot` seed assertion; the bitwise baseline in Task 1.12 Step 1 still guards `ddot_raw`. The file's `require(bool, std::string_view)` helper and its `main` registration pattern match these snippets.

Add to `native/tests/test_platform_lapack.cpp`:

```cpp
// 2x2 solve with an exact answer: [[2,1],[1,3]] x = [3,5] -> x = [0.8, 1.4].
// dgesv works column-major in place.
void dgesv_solves_small_system() {
    numeric_backend::integer n = 2, nrhs = 1, lda = 2, ldb = 2, info = -1;
    numeric_backend::integer ipiv[2] = {0, 0};
    double a[4] = {2., 1., 1., 3.};
    double b[2] = {3., 5.};
    numeric_backend::dgesv(&n, &nrhs, a, &lda, ipiv, b, &ldb, &info);
    require(info == 0, "dgesv info");
    require(std::abs(b[0] - 0.8) < 1e-15 && std::abs(b[1] - 1.4) < 1e-15, "dgesv solution");
}

// Symmetric eigenvalues of [[2,1],[1,2]] are 1 and 3 (ascending).
void dsyevd_eigenvalues() {
    numeric_backend::integer n = 2, lda = 2, info = -1, lwork = -1, liwork = -1;
    double a[4] = {2., 1., 1., 2.};
    double w[2] = {0., 0.};
    double wq = 0.;
    numeric_backend::integer iwq = 0;
    numeric_backend::dsyevd("V", "L", &n, a, &lda, w, &wq, &lwork, &iwq, &liwork, &info);
    require(info == 0, "dsyevd workspace query");
    lwork = static_cast<numeric_backend::integer>(wq);
    liwork = iwq;
    std::vector<double> work(static_cast<std::size_t>(lwork));
    std::vector<numeric_backend::integer> iwork(static_cast<std::size_t>(liwork));
    numeric_backend::dsyevd("V", "L", &n, a, &lda, w, work.data(), &lwork, iwork.data(), &liwork, &info);
    require(info == 0 && std::abs(w[0] - 1.) < 1e-15 && std::abs(w[1] - 3.) < 1e-15, "dsyevd eigenvalues");
}
```

Register the new functions in each file's `main` the way the existing cases are registered.

```bash
cmake --build native/build -j10; echo "exit=$?"
```

Expected: compile FAIL, because the face functions are undeclared.

- [ ] **Step 2: Implement the face** (declarations exactly as in Interfaces, with the comments).

- [ ] **Step 3: Implement the backends.** Darwin, in `blas_accelerate.cpp`:

```cpp
#include "numerics_symbols.hpp"

// The ILP64 provider fold geometry/specs.hpp has always called (NumPy's own
// DOUBLE_dot import on macOS). It differs from the legacy 32-bit cblas_ddot
// behind blas::ddot by symbol and integer width; keeping it preserves the
// captured kinematics fixtures bitwise. The probe qualifies the spelling
// (numerics.symbols.ddot_ilp64).
extern "C" double bike_accelerate_ddot_ilp64(std::int64_t n, const double *x, std::int64_t incx,
                                              const double *y, std::int64_t incy)
    __asm__(BIKE_NUMERICS_SYM_DDOT_ILP64);
extern "C" void bike_accelerate_zdotu_sub(std::int64_t n, const void *x, std::int64_t incx,
                                           const void *y, std::int64_t incy, void *out)
    __asm__(BIKE_NUMERICS_SYM_ZDOTU_SUB);

namespace blas {
    double ddot_raw(int n, const double *x, int incx, const double *y, int incy) noexcept {
        return bike_accelerate_ddot_ilp64(n, x, incx, y, incy);
    }
    void zdotu_sub(int n, const void *x, int incx, const void *y, int incy, void *out) noexcept {
        bike_accelerate_zdotu_sub(n, x, incx, y, incy, out);
    }
}
```

The Linux file `blas_numpy_openblas.cpp` follows the existing pattern there, with `std::int64_t` widths:
- `ddot_raw` binds `BIKE_NUMERICS_SYM_DDOT_ILP64` **without** the `0. +` seed. On Linux that is the same export as `BIKE_NUMERICS_SYM_DDOT`. Declare it under a distinct C name so the two `extern "C"` declarations do not collide.
- `zdotu_sub` binds `BIKE_NUMERICS_SYM_ZDOTU_SUB`.

In `lapack_accelerate.cpp` and `lapack_numpy_openblas.cpp`, `dgesv` and `dsyevd` bind `BIKE_NUMERICS_SYM_DGESV` / `BIKE_NUMERICS_SYM_DSYEVD`, the same way `dgelsd` binds `BIKE_NUMERICS_SYM_DGELSD` in those files. Add the same compile-time signature pins (`static_assert`) those files keep for `dgelsd`.

- [ ] **Step 4: Run tests on macOS and compile the backends on Linux**

```bash
cmake --build native/build -j10
ctest --test-dir native/build -R '^native_platform_(blas|lapack)_tests$' --output-on-failure
tools/linux/container.sh sync
tools/linux/container.sh exec bash -c 'ninja -C /build/native-clang -t targets all | grep -E "platform/(blas|lapack)_numpy_openblas.cpp.o"'
tools/linux/container.sh build native-clang <the two object targets printed above>
```

Expected: both macOS tests pass, and both Linux backend objects compile. The Linux tests themselves run in Task 1.19.

- [ ] **Step 5: Commit**

```bash
git add native/src/cblas_abi.hpp native/src/lapack_abi.hpp native/src/platform/ native/tests/test_platform_blas.cpp native/tests/test_platform_lapack.cpp
git commit -m "feat(cpp): numerics face gains ddot_raw, zdotu_sub, dgesv, dsyevd

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.12: Route the five direct Accelerate call sites through the face

**Files:**
- Modify: `native/src/geometry/specs.hpp:248-272`
- Modify: `native/src/kinematics/curves.cpp:18-30` (+ its call sites in the same file)
- Modify: `native/src/model/mass.cpp:220-232` (+ call sites)
- Modify: `native/src/numerics/filter.cpp:15-26` (+ call sites)
- Modify, if the link fails: `CMakeLists.txt` and `native/CMakeLists.txt` target link lines for the libraries owning these TUs:
  - root: `curves.cpp` at `CMakeLists.txt:492`, `mass.cpp` at `:536`;
  - `filter.cpp` in `native/cmake/NativePhysicsResearch.cmake:59`;
  - binding graph: `native/CMakeLists.txt:370` and `:385`.

  Add `Bike::Numerics` to their link lines.

**Interfaces:**
- Consumes: Task 1.11 face.
- Produces: no `__asm__("_…$NEWLAPACK…")` outside `native/src/platform/`.

**Review focus:** this is a pure re-plumbing. The macOS arithmetic is unchanged: the same symbols, the same argument values.

- [ ] **Step 1: Capture the macOS bitwise baseline for the affected tests**

```bash
cmake --build build/native-release -j10
ctest --test-dir build/native-release -R '^migration_T(14|15|19|26|27|28|29|30|31|32|33|36|37)$' --output-on-failure | tee /tmp/t112-before.txt
grep -rln 'numerics/filter.hpp\|kinematics/curves.hpp\|model/mass.hpp\|geometry/specs.hpp' native/tests | sort > /tmp/t112-tests.txt
```

Expected: all listed pass. At audit time, only `test_t14.cpp`, `test_t15.cpp` and `test_t32.cpp` include the touched headers, and all three are in the regex. If `/tmp/t112-tests.txt` names any other test, add it to the regex and re-run.

- [ ] **Step 2: Write the absence check (red)**

```bash
grep -rn '__asm__("_[a-z_]*\$NEWLAPACK' native/src --include='*.cpp' --include='*.hpp' | grep -v '^native/src/platform/'
```

Expected now: 5 hits. After Step 3: none. Add this check permanently as a contracts rule: in `native/src/tools/check_contracts.cpp`, extend the structural-contract kind so it scans `native/src/**` outside `platform/` for `__asm__("_` with `$NEWLAPACK` or `scipy_` and fails with `provider symbol bound outside native/src/platform`. Add a T56 negative case that drops a fixture file containing such a line into a scratch source root and expects the failure. Follow how the existing contracts rules accept a `--repo-root`.

- [ ] **Step 3: Rewrite the call sites**

`geometry/specs.hpp`. Delete the `extern "C" … cblas_ddot_ilp64 … __asm__` block and include the face:

```cpp
#include "../cblas_abi.hpp"
// ...
    // np.dot(a, b) on contiguous 1-D doubles: the bare provider fold
    // (blas::ddot_raw), exactly the entry point used since T14.
    [[nodiscard]] inline double np_dot(std::span<const double> a,
                                       std::span<const double> b) noexcept {
        return blas::ddot_raw(static_cast<int>(a.size()), a.data(), 1, b.data(), 1);
    }

    // np.linalg.norm(v) on a real 1-D vector: sqrt(x.dot(x)).
    [[nodiscard]] inline double np_norm(std::span<const double> v) noexcept {
        return std::sqrt(blas::ddot_raw(static_cast<int>(v.size()), v.data(), 1, v.data(), 1));
    }
```

`kinematics/curves.cpp`:
- delete `dgesv_ilp64`;
- at each call, use `numeric_backend::dgesv`, with integer variables of type `numeric_backend::integer` (that is `std::int64_t`; today's `long` differs in spelling on Darwin, where `std::int64_t` is `long long`).

`model/mass.cpp`:
- delete `dsyevd_ilp64` and `lapack_int`, and use `numeric_backend::dsyevd` with `numeric_backend::integer`;
- the `static_assert` on `decltype(&dsyevd_ilp64)` at about 233-240 goes with the declaration, because the face backend keeps its own signature pin (Task 1.11 Step 3).

`long` → `std::int64_t` is ABI-identical on both platforms (LP64), so this is a spelling change only.

`numerics/filter.cpp`: delete `filter_dgesv_ilp64` and `filter_zdotu`; use `numeric_backend::dgesv` and `blas::zdotu_sub` (`n` narrowed to `int` with `static_cast<int>`, as every other face call does).

Keep each site's explanatory comment, reworded to name the face function instead of the symbol.

- [ ] **Step 4: Verify macOS bitwise, then Linux compile+link of those TUs**

```bash
cmake --build build/native-release -j10 && cmake --build native/build -j10
ctest --test-dir build/native-release -R '^migration_T(14|15|19|26|27|28|29|30|31|32|33|36|37)$' --output-on-failure | tee /tmp/t112-after.txt
diff <(grep -E 'Passed|Failed' /tmp/t112-before.txt | sed 's/ *[0-9.]* sec//') <(grep -E 'Passed|Failed' /tmp/t112-after.txt | sed 's/ *[0-9.]* sec//')
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run --frozen --group native python -m pytest -q tests/reference/test_native_suspension.py tests/reference/test_native_rider_forces.py
grep -rn '__asm__("_[a-z_]*\$NEWLAPACK' native/src | grep -v '^native/src/platform/' ; echo "hits=$?"
```

Expected:
- the `diff` prints nothing;
- pytest passes (both files compare kinematics/mass outputs bitwise with the oracle);
- `hits=1` (grep found nothing).

```bash
tools/linux/container.sh sync && tools/linux/container.sh build native-clang migration_T14 migration_T15 migration_T32
```

Expected: all three link on Linux. This checks a **predicted** link failure. In the audit, T14 and T15 never reached the link step: they failed at compile on `value.hpp` (clang log ~25054 and ~26104), and the log has no `undefined reference` lines. The direct `$NEWLAPACK` labels would be undefined against OpenBLAS once compilation passes. If Task 1.13 has not landed yet, this step still fails on `value.hpp`. In that case, order this verification after 1.13 and record the order in the evidence log.

- [ ] **Step 5: Commit**

```bash
git add native/src/geometry/specs.hpp native/src/kinematics/curves.cpp native/src/model/mass.cpp native/src/numerics/filter.cpp native/src/tools/check_contracts.cpp native/tests/migration/migration_T56_check.cmake CMakeLists.txt native/CMakeLists.txt native/cmake/NativePhysicsResearch.cmake
git commit -m "fix(cpp): bind dot/solve/eigh/zdotu through the numerics face

Direct Accelerate asm labels in geometry, kinematics, mass and filter
cannot resolve against OpenBLAS on Linux; macOS keeps the same entry
points bit for bit.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.13: `io::Value` complete under libstdc++

**Files:**
- Modify: `native/src/io/value.hpp:62-68` (and the lines immediately after `struct Value`)
- Modify, only if the Step 3 fallback is taken, every `Value::Object` access site:
  - `grep -rn 'Value::Object\|<Object>' native/src native/tests` gives 452 lines at audit time (365 in src, 87 in tests);
  - 146 of them are `get_if`/`holds_alternative`/`get<Object>`;
  - 5 are `std::visit` over `Value::data`;
  - about 72 are `Value{…}`-style constructions.

**Interfaces:**
- Consumes: nothing.
- Produces: `bike::io::Value`, compiling on Clang 18 + libstdc++ 13 and on GCC 13, with an unchanged aggregate shape: `Value{...}` and designated initializers keep compiling.

**Diagnosis at audit time.** Read the chain in `/tmp/audit-logs/linux-native-clang.log` around lines 4374-4450 before you choose a fix. It is **not** "Value incomplete inside the variant":
1. `std::pair<std::string, Value>` is first instantiated at `value.hpp:84:28`, in `find_member`'s `member.first`. That is after `struct Value` has closed.
2. Pair's default-ctor constraint (`stl_pair.h:247`, `explicit(__not_<__and_<__is_implicitly_default_constructible<…>>>)`) asks whether `Value` is default-constructible.
3. That pulls in Value's implicit destructor, then the variant's constexpr `_M_reset`, then `~vector<pair<…>>`, then `std::_Destroy`, then `is_destructible<pair<…>>`.
4. At that point pair is still being defined (log: "definition of 'std::pair<…>' is not complete until the closing '}'").

The cycle is pair → `~Value` → `~vector<pair>` → pair. libstdc++'s constexpr `~vector` and the constexpr variant reset are instantiated eagerly. So any fix that leaves the destructor chain inline and constexpr keeps the cycle. That includes a plain alias template with defaulted members, and a `Boxed` whose `~Boxed() = default` sits inside its class (`unique_ptr`'s destructor and `default_delete` are `_GLIBCXX23_CONSTEXPR` under `-std=c++23`).

**Review focus:**
- macOS JSON/config behaviour stays unchanged. The guards are `migration_T05` (JSON boundary) and `migration_T12`/`T13` (config).
- `std::pair<std::string, Value>` is spelled nowhere outside `value.hpp` (grep it to confirm). So a member-type change behind `Value::Object` needs no site rewrite.

- [ ] **Step 1: Reproduce in one TU with the full chain (red)**

```bash
tools/linux/container.sh sync
tools/linux/container.sh exec bash -c 'ninja -C /build/native-clang -t targets all | grep -E "src/io/json.cpp.o"'
tools/linux/container.sh exec bash -c 'cd /build/native-clang && ninja -t commands <json.cpp object target> | tail -1 > /tmp/json-cmd.sh && sed -i "s|\$| -ftemplate-backtrace-limit=0|" /tmp/json-cmd.sh && bash /tmp/json-cmd.sh 2>&1 | tee /tmp/value-chain.txt | grep -c "error:"'
tools/linux/container.sh exec cat /tmp/value-chain.txt > /tmp/value-chain.txt
```

Expected: at least one `error:` line with `static assertion failed due to requirement 'std::__is_complete_or_unbounded…pair<…Value>'`. Paste the full `note: in instantiation of` chain into the evidence log. If it does not match the Diagnosis above, stop and report: the fix below assumes that chain.

- [ ] **Step 2: Out-of-line special members behind an `ObjectList` member type**

Replace lines 62-68 with the declarations below. Put the definitions **immediately after** `struct Value`, before `find_member`:

```cpp
// libstdc++ 13: instantiating std::pair<std::string, Value> checks Value's
// default-constructibility, which instantiates Value's destructor through the
// constexpr variant reset, which instantiates ~vector<pair> — while pair is
// still being defined. ObjectList's special members are declared here and
// defined after Value with non-constexpr bodies, so the vector<pair>
// destructor is instantiated at end of TU, when pair is complete. libc++
// defers this chain on its own (macOS never saw it).
template <class V>
struct ObjectList : std::vector<std::pair<std::string, V>> {
    using base = std::vector<std::pair<std::string, V>>;
    using base::base;
    ObjectList();
    ObjectList(base members);
    ObjectList(const ObjectList &other);
    ObjectList(ObjectList &&other) noexcept;
    ObjectList &operator=(const ObjectList &other);
    ObjectList &operator=(ObjectList &&other) noexcept;
    ~ObjectList();
};

struct Value {
    using Array = std::vector<Value>;
    using Object = ObjectList<Value>;
    std::variant<std::monostate, bool, Number, std::string, Array, Object,
                 Bytes, Tagged>
        data;
};

// Out-of-line on purpose (see ObjectList): user-provided, non-constexpr bodies.
// NOLINTBEGIN(modernize-use-equals-default)
template <class V> ObjectList<V>::ObjectList() {}
template <class V> ObjectList<V>::ObjectList(base members) : base(std::move(members)) {}
template <class V> ObjectList<V>::ObjectList(const ObjectList &other) : base(other) {}
template <class V> ObjectList<V>::ObjectList(ObjectList &&other) noexcept : base(std::move(other)) {}
template <class V> ObjectList<V> &ObjectList<V>::operator=(const ObjectList &other) { base::operator=(other); return *this; }
template <class V> ObjectList<V> &ObjectList<V>::operator=(ObjectList &&other) noexcept { base::operator=(std::move(other)); return *this; }
template <class V> ObjectList<V>::~ObjectList() {}
// NOLINTEND(modernize-use-equals-default)
```

Before building, run `grep -rn 'constexpr[^;]*\bValue\b\|constinit[^;]*\bValue\b' native/src native/tests`. A non-constexpr destructor removes `Value` from constant expressions. Any hit is a stop-and-report finding: none existed at audit time.

```bash
tools/linux/container.sh sync && tools/linux/container.sh build native-clang <json.cpp object target>
tools/linux/container.sh exec bash -c 'ninja -C /build/native-gcc -t targets all | grep -E "src/io/json.cpp.o"'
tools/linux/container.sh build native-gcc <json.cpp object target from native-gcc>
cmake --build native/build -j10 && cmake --build build/native-release -j10
```

Decision rule:
- If both Linux objects compile **and** both macOS builds pass, keep this and skip Step 3.
- If either Linux compile still reports the incomplete-pair assertion, capture the new chain with Step 1's command, paste it into the evidence log, revert this edit, and do Step 3.
- If macOS fails on another construct, such as an aggregate init or a defaulted comparison that relied on implicit members, fix that construct; do not fall back for it.

- [ ] **Step 3 (fallback only): box the object alternative, with the same out-of-line rule**

```cpp
namespace detail {
// Owns one T on the heap so std::variant holds a complete, copyable type.
// Special members are defined after Value (non-constexpr bodies), for the
// reason documented on Step 2's ObjectList: an inline ~Boxed() = default
// instantiates ~unique_ptr<T> → delete T → ~vector<pair> eagerly under
// C++23 and keeps the libstdc++ cycle. Non-explicit from T, so
// Value{Value::Object{...}} keeps compiling.
template <class T>
class Boxed {
public:
    Boxed();
    Boxed(T v);
    Boxed(const Boxed &o);
    Boxed(Boxed &&o) noexcept;
    Boxed &operator=(const Boxed &o);
    Boxed &operator=(Boxed &&o) noexcept;
    ~Boxed();
    T &get() noexcept { return *p_; }
    const T &get() const noexcept { return *p_; }
private:
    std::unique_ptr<T> p_;
};
} // namespace detail

struct Value {
    using Array = std::vector<Value>;
    using Object = std::vector<std::pair<std::string, Value>>;
    std::variant<std::monostate, bool, Number, std::string, Array,
                 detail::Boxed<Object>, Bytes, Tagged>
        data;
};

// NOLINTBEGIN(modernize-use-equals-default)
template <class T> detail::Boxed<T>::Boxed() : p_(std::make_unique<T>()) {}
template <class T> detail::Boxed<T>::Boxed(T v) : p_(std::make_unique<T>(std::move(v))) {}
template <class T> detail::Boxed<T>::Boxed(const Boxed &o) : p_(std::make_unique<T>(*o.p_)) {}
template <class T> detail::Boxed<T>::Boxed(Boxed &&o) noexcept : p_(std::move(o.p_)) {}
template <class T> detail::Boxed<T> &detail::Boxed<T>::operator=(const Boxed &o) { p_ = std::make_unique<T>(*o.p_); return *this; }
template <class T> detail::Boxed<T> &detail::Boxed<T>::operator=(Boxed &&o) noexcept { p_ = std::move(o.p_); return *this; }
template <class T> detail::Boxed<T>::~Boxed() {}
// NOLINTEND(modernize-use-equals-default)

inline const Value::Object *object_of(const Value &v) noexcept {
    const auto *box = std::get_if<detail::Boxed<Value::Object>>(&v.data);
    return box != nullptr ? &box->get() : nullptr;
}
inline Value::Object *object_of(Value &v) noexcept {
    auto *box = std::get_if<detail::Boxed<Value::Object>>(&v.data);
    return box != nullptr ? &box->get() : nullptr;
}
```

A moved-from `Boxed` holds a null `p_`. Every site reads `get()` only on a live alternative, but state this invariant in the comment and add a `migration_T05` case that copies and moves an object `Value`.

Then rewrite every access mechanically (counts are in Files):

| Old | New |
|---|---|
| `std::get_if<Object>(&x.data)` / `std::get_if<Value::Object>(&x.data)` | `object_of(x)` |
| `std::holds_alternative<Object>(x.data)` | `object_of(x) != nullptr` |
| `std::get<Object>(x.data)` | `*object_of(x)`, after a check that throws the site's existing error type and message |
| `std::visit` arms taking `const Object &` (5 sites) | take `const detail::Boxed<Object> &b` and use `b.get()` |

Include `<memory>`. Re-run the Step 2 commands until all four builds pass.

- [ ] **Step 4: macOS guards + Linux sample TUs**

```bash
cmake --build build/native-release -j10 && cmake --build native/build -j10
ctest --test-dir build/native-release -R '^migration_T(05|12|13|56)$' --output-on-failure
tools/linux/container.sh sync
tools/linux/container.sh build native-clang; tools/linux/container.sh failures native-clang
```

Expected:
- macOS green;
- the Linux failure count drops from 239 to the TUs owned by Tasks 1.14–1.18, about 15 TUs. Record the number in the commit body.

- [ ] **Step 5: Commit**

```bash
git add -A native/src native/tests
git commit -m "fix(cpp): keep io::Value complete under libstdc++

libstdc++ 13 instantiates ~vector<pair<string, Value>> eagerly while
pair is being defined (pair -> ~Value -> constexpr variant reset ->
~vector<pair>); out-of-line non-constexpr special members defer it.

Co-Authored-By: <the trailer the executing harness specifies>"
```

Before `git add -A`, run `git status --short` and confirm that only Task 1.13 files are listed.

---

### Task 1.14: Linux platform branches that never compiled

**Files:**
- Modify: `native/src/platform/process.cpp:115-134` (`add_chdir`), `:194` (lock CTAD)
- Modify: `native/src/platform/provenance.cpp:70-71`

**Interfaces:**
- Consumes: nothing.
- Produces: `platform::add_chdir` and `platform::executable_path` compile on glibc 2.39.

**Review focus:** glibc 2.39 declares only `posix_spawn_file_actions_addchdir_np`. The unsuffixed name arrived in glibc 2.40 (2024-07); the old `__GLIBC_PREREQ(2, 38)` gate was wrong.

- [ ] **Step 1: Red**

```bash
tools/linux/container.sh sync
tools/linux/container.sh exec bash -c 'ninja -C /build/native-clang -t targets all | grep -E "platform/(process|provenance).cpp.o"'
tools/linux/container.sh build native-clang <the two object targets>
```

Expected: errors at `process.cpp:124` and `provenance.cpp:70`.

- [ ] **Step 2: Fix**

```cpp
#elif defined(__linux__) && defined(__GLIBC__) && defined(__GLIBC_PREREQ)
#if __GLIBC_PREREQ(2, 40)
    // POSIX.1-2024 name; glibc exports it from 2.40.
    return ::posix_spawn_file_actions_addchdir(actions, path);
#else
    return ::posix_spawn_file_actions_addchdir_np(actions, path);
#endif
```

```cpp
    return std::filesystem::path{
        std::string_view{buffer.data(), static_cast<std::size_t>(got)}};
```

(add `#include <string_view>`), and at `process.cpp:194`: `std::unique_lock<std::mutex> spawn_guard(spawn_mutex);`.

- [ ] **Step 3: Green on Linux (both compilers), then macOS**

```bash
tools/linux/container.sh sync
tools/linux/container.sh build native-clang <targets>
tools/linux/container.sh build native-gcc <targets for native-gcc>
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -R '^migration_T06$' --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add native/src/platform/process.cpp native/src/platform/provenance.cpp
git commit -m "fix(cpp): compile the Linux spawn-chdir and executable-path branches

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.15: Explicit lock template arguments (Clang `-Wctad-maybe-unsupported`)

**Files (all sites at audit time; re-grep in Step 1):**
- `native/src/runtime/async_close.cpp`:
  - `std::scoped_lock` at 24, 60, 68, 72, 80, 94, 122, 127, 134, 138;
  - `std::unique_lock` at 38, 106 (mutex `std::mutex mutex_`, `async_close.hpp:61`).
- `native/src/legacy/playground.cpp`: `std::scoped_lock` at 93, 94, 99, 106-113, 117, 119-121, 124, 148, 150, 166 (mutex `std::recursive_mutex`, `:46`).
- `native/src/viz/html.cpp`: 17, 85 (`std::mutex`).
- `native/src/batch/process_pool.cpp`: `scoped_lock` 68, 72; `unique_lock` 81 (`std::mutex`).

**Interfaces:** none (mechanical).

**Review focus:** the template argument matches the actual mutex type at each site (`recursive_mutex` in playground).

- [ ] **Step 1: Re-grep**

```bash
grep -rnE 'std::(scoped_lock|unique_lock|lock_guard|shared_lock)( const)? [a-z_]+[({]' native/src native/tests
```

Every hit without `<…>` is in scope.

- [ ] **Step 2: Rewrite** each to `std::scoped_lock<std::mutex> const lock(mutex_);`, `std::unique_lock<std::mutex> lock(mutex_);`, `std::scoped_lock<std::recursive_mutex> …`, preserving `const` and the variable names.

- [ ] **Step 3: Verify**

```bash
tools/linux/container.sh sync
tools/linux/container.sh build native-clang; tools/linux/container.sh failures native-clang
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -R '^migration_T(33|38|39|40|41|42|43|44|45|46|47|48|49|50|51)$' --output-on-failure
```

Expected:
- no `ctad-maybe-unsupported` lines in `/build/native-clang.log` (check with `tools/linux/container.sh exec grep -c ctad /build/native-clang.log`, expect `0`);
- macOS tests green.

- [ ] **Step 4: Commit**

```bash
git add native/src/runtime/async_close.cpp native/src/legacy/playground.cpp native/src/viz/html.cpp native/src/batch/process_pool.cpp
git commit -m "fix(cpp): spell lock template arguments for libstdc++ CTAD warnings

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.16: GCC: constant initialization, redundant declarations, deleter attribute

**Files:**
- Modify: `native/src/configuration/physical_config.hpp:124-136` (`TierSpec`, `kTireTiers`)
- Modify: `native/src/initialization/finalize.hpp:9`
- Modify: `native/src/runtime/step.hpp` (include before the friend at about 154)
- Modify: `native/src/viz/plot.cpp:63`

**Interfaces:**
- Produces: `struct TierSpec { std::string_view name; int n_rays; double half_angle_deg; double timestep_s; bool discretised_brush; };` and `inline constexpr std::array<TierSpec, 2> kTireTiers{…}`.

**Review focus:** every `TierSpec::name` consumer works with `string_view`. At audit time: `configuration/validate.cpp:402` (`+=` into a `std::string`) and `:939` (`==`). Re-grep.

- [ ] **Step 1: Red** (GCC lane)

```bash
tools/linux/container.sh sync
tools/linux/container.sh build native-gcc; tools/linux/container.sh failures native-gcc
tools/linux/container.sh exec grep -c "is not a constant expression\|redundant-decls\|ignored-attributes" /build/native-gcc.log
```

Expected: a non-zero count.

- [ ] **Step 2: Fix**

```cpp
struct TierSpec {
    std::string_view name;
    int n_rays;
    double half_angle_deg;
    double timestep_s;
    bool discretised_brush;
};

// tyre.py TIERS, in dict insertion order ('fast', 'detailed').
inline constexpr std::array<TierSpec, 2> kTireTiers{{
    {.name = "fast", .n_rays = 64, .half_angle_deg = 75.0, .timestep_s = 0.0005, .discretised_brush = false},
    {.name = "detailed", .n_rays = 256, .half_angle_deg = 75.0, .timestep_s = 0.00025, .discretised_brush = true},
}};
```

(`#include <string_view>`). `grep -rn 'kTireTiers\|TierSpec\|find_tire_tier' native/src native/tests` lists all users; fix any that needed `std::string`.

- In `finalize.hpp`, delete the line `[[nodiscard]] ResearchStartup prepare_research_startup(SetupContext&);` (line 9).
- In `runtime/step.hpp`, add `#include "../initialization/research_startup.hpp"` with the other includes, so the friend at about 154 refers to an existing declaration.
- Fix every TU that relied on `finalize.hpp` for that declaration by including `initialization/research_startup.hpp`; the compiler names them.

`viz/plot.cpp:63`:

```cpp
struct FileCloser {
    void operator()(std::FILE *file) const noexcept { std::fclose(file); }
};
// ...
    std::unique_ptr<std::FILE, FileCloser> in(::fdopen(handle.get(), "rb"));
```

Place `FileCloser` in the file's anonymous namespace.

- [ ] **Step 3: Green (GCC + Clang lanes on the touched objects; macOS tests)**

```bash
tools/linux/container.sh sync
tools/linux/container.sh build native-gcc; tools/linux/container.sh failures native-gcc
tools/linux/container.sh exec grep -c "is not a constant expression\|redundant-decls\|ignored-attributes" /build/native-gcc.log
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -R '^migration_T(12|13|16|17|18|20|21|22|23|24|25|47)$' --output-on-failure
```

Expected: the grep count is `0`, and macOS is green.

- [ ] **Step 4: Commit**

```bash
git add native/src/configuration/physical_config.hpp native/src/configuration/validate.cpp native/src/initialization/ native/src/runtime/step.hpp native/src/viz/plot.cpp
git commit -m "fix(cpp): GCC 13 constant init, redundant decls and FILE deleter

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.17: `linear_feasibility` in binary64 (D8)

**Files:**
- Modify: `native/src/numerics/linear_feasibility.cpp` (every `long double`, lines 12-68)
- Test: `native/tests/migration/test_t28.cpp` (add a case)

**Interfaces:**
- Produces: the same `FeasibilityResult linear_feasibility(...)` signature; the interval certificate is computed in `double` with outward `std::nextafter` rounding.

**Review focus:**
- The certificate stays conservative: every `down`/`up` survives.
- macOS arm64 results are bit-identical (`long double == double` there).

- [ ] **Step 1: Failing guard test**

Add to `test_t28.cpp`:

```cpp
// The outward-rounded interval certificate must work in binary64 on every
// platform (D8): a contradiction separated by one ulp is still proven, and a
// system that is feasible exactly on its boundary is never proven empty.
// Slack and primal tolerance are zeroed: the defaults (inequality_slack=1e-7,
// linear_feasibility.hpp:15) would make the one-ulp system feasible by design.
void binary64_certificate_edges() {
    const n::FeasibilityOptions exact{.inequality_slack = 0., .primal_tolerance = 0.};
    const double one_ulp_below = std::nextafter(1.0, 0.0);
    // x >= 1 (as -x <= -1) and x <= 1 - ulp on x in [0, 2].
    const Matrix g{.rows = 2, .cols = 1, .values = {-1., 1.}};
    const std::vector<double> h{-1., one_ulp_below};
    const std::vector<double> lo{0.}, hi{2.};
    const auto proven = n::linear_feasibility(g, h, lo, hi, {}, {}, exact);
    bt::require(proven.status == n::FeasibilityStatus::proven_infeasible || proven.status == n::FeasibilityStatus::unknown,
                "one-ulp contradiction is never reported feasible-certified");
    const std::vector<double> h_exact{-1., 1.};
    bt::require(n::linear_feasibility(g, h_exact, lo, hi, {}, {}, exact).status != n::FeasibilityStatus::proven_infeasible,
                "boundary-feasible system is never proven empty");
}
```

Call it from `main`. `Matrix` is `bike::rider::Matrix`, visible through the file's `using namespace bike::rider;` (`test_t28.cpp:16`). `FeasibilityOptions` is the seventh parameter, after the defaulted equality pair, hence the `{}, {}`.

This case is a guard, not a red test. On macOS it must pass **before** Step 2: `long double` is binary64 there. Run `cmake --build build/native-release -j10 && ctest --test-dir build/native-release -R '^migration_T28$'` once to prove that, and record it in the evidence log. If it fails on macOS before the rewrite, stop: the case or the certificate is wrong, not D8.

```bash
tools/linux/container.sh sync
tools/linux/container.sh exec bash -c 'ninja -C /build/native-clang -t targets all | grep "linear_feasibility.cpp.o"'
tools/linux/container.sh build native-clang <that object>
```

Expected: `-Wdouble-promotion` errors.

- [ ] **Step 2: Rewrite**
  - Replace every `long double` with `double`, `std::numeric_limits<long double>` with `std::numeric_limits<double>`, and remove every `static_cast<long double>(…)`.
  - Lines 65/68 lose their `static_cast<double>`.
  - Keep `down()`/`up()` (`std::nextafter` toward ∓inf) exactly where they are.
  - Update the file comment: `Binary64 interval arithmetic with outward rounding (completion plan D8; macOS arm64 long double was already binary64).`

- [ ] **Step 3: Green**

```bash
cmake --build build/native-release -j10 && ctest --test-dir build/native-release -R '^migration_T(28|29)$' --output-on-failure
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run --frozen --group native python -m pytest -q tests/reference/test_rider_allocation.py
tools/linux/container.sh sync && tools/linux/container.sh build native-clang <that object>
```

Expected:
- CTest green;
- pytest shows exactly the 3 documented baseline failures of `test_rider_allocation.py` and no new ones. Find them in the `docs/TESTING.md` baseline list with `grep -n 'test_rider_allocation' docs/TESTING.md`; do not rely on line numbers, because Task 1.03 shifts the file;
- the Linux object compiles.

- [ ] **Step 4: Commit**

```bash
git add native/src/numerics/linear_feasibility.cpp native/tests/migration/test_t28.cpp
git commit -m "fix(cpp): linear feasibility certificate in binary64 on every platform

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.18: `np_sin`/`np_cos` NaN-sign parity on Linux

**Files:**
- Modify: `native/src/numeric_sincos.hpp:60-85`
- Create: `native/tests/test_platform_sincos.cpp`, registered as `native_platform_sincos_tests` in `native/CMakeLists.txt` next to `native_platform_blas_tests` (454-464, 519-520). The `test_platform_*` tests exist **only in the binding graph**; the root graph registers none.
- Root-graph decision (explicit): do **not** add the test to the root graph in this task. Task 3.02 maps it to the root graph together with the other `test_platform_*` sources, before 4.06 deletes the binding graph. Record that mapping row in Task 3.02's port map.

**Interfaces:**
- Produces: `numeric::np_sin`/`np_cos` reproduce NumPy 2.5.2's result bits for non-finite input on **each** qualified platform.

**Review focus:** the gate is chosen from a measurement on Linux, not by analogy with macOS.

- [ ] **Step 1: Measure NumPy and libm on Linux**

```bash
tools/linux/container.sh exec uv run --frozen --group native python -c "
import numpy as np, struct
def bits(x): return f\"{struct.unpack('<Q', struct.pack('<d', x))[0]:016x}\"
for v in [float('inf'), float('-inf'), float('nan'), -float('nan')]:
    scalar = (bits(float(np.sin(np.float64(v)))), bits(float(np.cos(np.float64(v)))))
    # Contiguous ufunc path: NumPy may dispatch a SIMD loop for arrays.
    vector = (bits(float(np.sin(np.full(64, v))[0])), bits(float(np.cos(np.full(64, v))[0])))
    print(bits(v), 'scalar sin/cos', *scalar, 'array sin/cos', *vector)
"
tools/linux/container.sh exec bash -c 'cat > /tmp/s.cpp <<EOF
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
static unsigned long long b(double x){unsigned long long u;std::memcpy(&u,&x,8);return u;}
int main(){volatile double v[4]={INFINITY,-INFINITY,std::numeric_limits<double>::quiet_NaN(),-std::numeric_limits<double>::quiet_NaN()};
for(double x:v){volatile double y=x;std::printf("%016llx sin %016llx cos %016llx sub %016llx\n",b(x),b(std::sin(y)),b(std::cos(y)),b(y-y));}}
EOF
clang++-18 -O2 -ffp-contract=off /tmp/s.cpp -o /tmp/s && /tmp/s'
```

Record both outputs in the evidence log. The `bits()` helper prints the 64-bit integer value as `%016x`, which is the same spelling as the C++ `%016llx`. Compare the strings directly. Run the NumPy half on macOS as well (`uv run --frozen --group native python -c …`, same script), so the evidence log has both platforms' tables.

- [ ] **Step 2: Decide from the measurement**
  - The scalar and array columns must agree on each platform. If they differ, `np_sin`/`np_cos` cannot reproduce both NumPy paths with one rule. Stop and report the table.
  - If NumPy's result bits equal `y - y` for every non-finite input on Linux and differ from `std::sin`/`std::cos`: extend the gate to `#if (defined(__APPLE__) && defined(__arm64__)) || (defined(__linux__) && defined(__aarch64__))`.
  - If NumPy equals glibc `std::sin`/`std::cos`: leave the gate Apple-only and add a comment citing the measurement.
  - Any other pattern: stop and report the table.

- [ ] **Step 3: Add the regression**, a test asserting `np_sin`/`np_cos` bits for the four inputs equal the NumPy bits measured on **the platform the test runs on**. Hard-code both platforms' expected tables under `#if` with the measurement date and command in a comment. Run it on both platforms:

```bash
cmake --build native/build -j10 && ctest --test-dir native/build -R '^native_platform_sincos_tests$' --output-on-failure
tools/linux/container.sh sync && tools/linux/container.sh build native-clang <sincos test target>
tools/linux/container.sh exec /build/native-clang/<sincos test binary path>
```

On Linux, the binding graph cannot run CTest as a whole until Task 1.19. Execute the test binary directly; `ninja -t targets` gives its path.

- [ ] **Step 4: Commit**

```bash
git add native/src/numeric_sincos.hpp native/tests/test_platform_sincos.cpp native/CMakeLists.txt migration/cpp-linux-completion-evidence.md
git commit -m "fix(cpp): measured NaN-sign rule for np_sin/np_cos on Linux aarch64

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 1.19: Linux binding graph to zero failures (Clang + GCC), CTest green

**Files:** whatever residual TUs the full Linux builds still reject. The Clang log stopped at 20 errors per TU during the audit, so earlier fixes can unmask new errors.

**Interfaces:** none new.

**Review focus:** every residual fix names its root cause in the commit body. No `-Wno-*`, no `#pragma` suppression, no `analysis_suppressions.json` entry without the checker/origin/reason/reproducer/removal-condition record that `native-engineering.md` requires.

- [ ] **Step 1: Full builds**

```bash
tools/linux/container.sh sync
tools/linux/container.sh build native-clang; tools/linux/container.sh failures native-clang
tools/linux/container.sh build native-gcc;   tools/linux/container.sh failures native-gcc
```

- [ ] **Step 2: For each remaining distinct error**, ordered by frequency (`tools/linux/container.sh exec bash -c "grep -E 'error:' /build/native-clang.log | sed -E 's/^[^ ]+ //' | sort | uniq -c | sort -rn | head"`):
  - fix the root cause;
  - rebuild only the failing object (L1);
  - commit one root cause per commit: `fix(cpp): <cause> under <compiler>`.

  Stop and report if a fix would change arithmetic on macOS.

- [ ] **Step 3: CTest on Linux (both lanes)**

```bash
tools/linux/container.sh ctest native-clang -j8
tools/linux/container.sh ctest native-gcc -j8
```

Expected: all registered tests pass. A failing test is diagnosed (systematic-debugging) and fixed at its root cause, or, if it is platform numerics, reported with the measured bit difference before any change.

- [ ] **Step 4: Linux sweeps through the Python checker (transition only)**

The binding graph's `check_frontends/tidy/analyzer/cppcheck/odr` targets (`native/CMakeLists.txt:557-592`) run `tools/check_native_*.sh` / `tools/check_{analyzer,cppcheck,odr}.sh`. Each of those scripts `exec`s `bike_checks_binary` (`tools/native_common.sh:38-50`), which is the produced C++ `bike-native-checks`. That binary is built only by the root graph, and the root graph does not configure on Linux before Task 2.07. In the container those five targets therefore fail with a missing-binary error, and invoking them proves nothing.

Until Phase 2, call the Python checker directly on the Linux binding build. `tools/native_checks.py:3016-3024` accepts exactly these kinds plus `--build`. Only `check_scripts` (plain shellcheck) and `check_native_contracts` (`uv run … tools/check_native_contracts.py`) stay as CMake targets:

```bash
for k in frontends tidy analyzer cppcheck odr headers diagnostic-controls context; do
  tools/linux/container.sh exec uv run --frozen --group native python tools/native_checks.py --kind "$k" --build /build/native-clang || { echo "FAILED kind=$k"; exit 1; }
done
tools/linux/container.sh exec cmake --build /build/native-clang --target check_scripts check_native_contracts
```

Expected: every kind exits 0 and the final CMake build exits 0. There is no permitted deferral. The C++ checker's Linux run is part of Task 2.07's acceptance (`bike-native-checks --kind context --build /build/root-clang`) and of Task 2.11.

- [ ] **Step 5: Commit** the residual fixes (Step 2) and append `## Task 1.19 Linux binding graph` to the evidence log with both lanes' counts.

---

### CHECKPOINT 1-B (L2 both platforms) and L3 macOS: close Phase 1

- [ ] **L2.** Run the L2 block of the Verification ladder (macOS + Linux `native-clang`/`native-gcc`). For the Linux sweeps, the evidence is the Task 1.19 Step 4 block, rerun here: the `tools/native_checks.py --kind …` loop plus `check_scripts check_native_contracts`. Do not use the binding graph's `check_*` CMake targets, which need `bike-native-checks`. Paste each kind's exit code into the evidence log.
- [ ] **L3 macOS:**

```bash
bash tools/run_tests.sh native 2>&1 | tee /tmp/l3-phase1-macos.log; echo "exit=${PIPESTATUS[0]}"
```

Expected: the only failures are the 24 documented baseline node IDs. Extract them by anchor, not by line number, because Task 1.03 shifts `docs/TESTING.md`: `awk '/^The 24 failing node IDs/,/^## /' docs/TESTING.md | grep -oE 'tests/[^ `]+'`. Any other failure blocks the phase. Diagnose and fix it in a separate task-sized commit.
- [ ] **Evidence + push:** append `## Checkpoint 1-B / L3 macOS` (commands, exit codes, counts, failing node IDs verbatim) and commit `docs(migration): phase 1 evidence`. Then:

```bash
git push origin impl/python-free-cpp
```


---

# Phase 2: The Python-free root graph on Linux

Phase 2 changes only the root graph (`cmake -S .`, `build/native-release`) and the lane. The binding graph keeps working unchanged until Task 4.06 deletes it. Every task keeps macOS behavior identical. Each task's L1 includes `ctest --test-dir build/native-release -j8` on macOS (the root's full CTest set; ~1–3 min).

Linux root builds use the build name `root-clang`:

```bash
tools/linux/container.sh configure root-clang root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF
```

### Task 2.01: Root graph resolves dependencies through `BikeDependencyLock.cmake` (Darwin parity)

**Root `CMakeLists.txt` layout (measured 2026-10-21, HEAD 89cfcb1; re-measure with `grep -n` before editing, line numbers drift):**
legacy includes 28-30 (`CompilerPolicy.cmake`, `Dependencies.cmake`, `Provenance.cmake`); `bike_resolve_macos_sdk()` 32-34; `project(bike_sim CXX)` 36; `bike_dependencies_resolve()` 60; `bike_verify_sdk_compat()` 62-64; `BikeHostTarget` include + dispatch 72-73; libc++ hardening cache block + `bike_detect_libcpp_hardening()` 77-82; duplicate `option(NATIVE_SANITIZE|NATIVE_RTSAN|NATIVE_COVERAGE)` 86-88; `BIKE_NUMERICS_APPLY_POLICY` + `BikeNumerics.cmake` 100-101; provider-identity block 272-296; `.dylib` literals 575, 588, 597, 627; `add_subdirectory(native/tests/migration)` 607; bindings branch 611-618; `install()` 622-633; `Checks.cmake` include 642.

**Who defines what the root calls** (after 2.01/2.02): `bike_dependencies_resolve` → `BikeDependencyLock.cmake`; `bike_verify_sdk_compat` → `BikeAppleSDK.cmake:153` (bridge); `bike_target_strict_policy` / `bike_detect_libcpp_hardening` → `BikeCompilerPolicy.cmake:1390/1399` (bridges); `bike_rewrite_mujoco_load_name` → `BikeLinkage.cmake:227`; `bike_host_target_system` → `BikeHostTarget.cmake` (dropped in 2.02, `BikePlatform` replaces it). `BIKE_STRICT_WARNING_FLAGS` and `bike_resolve_macos_sdk` have no successor; nothing outside `CompilerPolicy.cmake` and root lines 33/71 references them, so they disappear with 2.02/2.03.

**Files:**
- Modify: `CMakeLists.txt`:
  - line 29: `include(…/Dependencies.cmake)` → `BikeDependencyLock.cmake`;
  - the hard-coded `libmujoco.3.12.0.dylib` at 575, 588, 597 and 627.
- Modify: `native/cmake/BikeLinkage.cmake:227` (`bike_rewrite_mujoco_load_name`, the root-build variant), only if its signature needs the library path from `BIKE_DEP_mujoco_LIBRARY`.
- Modify (dylib literals outside the root file, same substitution): `native/cmake/Checks.cmake:84` (moved into the emitter in 2.03; until then substitute `${BIKE_DEP_mujoco_LIBRARY}`), `native/cmake/NativeStartup.cmake:43` (root branch; `:47` is the binding-graph `MJ_DIR` branch — leave it), `native/cmake/RecordingApplication.cmake:19,31,229` (handled with the lock reader in 2.04), and the 11 `libmujoco.3.12.0.dylib` literals in `native/tests/migration/CMakeLists.txt` (168, 295, 343, 415, 529, 577, 661, 685, 714, 747, 794). The test-CMake ones do not break a Linux build (`bike_rewrite_mujoco_load_name` returns early off Darwin, `BikeLinkage.cmake:228`), but route them through the variable now so 2.07 finds none.

**Interfaces:**
- Consumes: `bike_dependencies_resolve()` from `BikeDependencyLock.cmake`. It sets `BIKE_DEP_<name>_ROOT`, `BIKE_DEP_mujoco_LIBRARY` and `BIKE_DEP_duckdb_LIBRARY` (`BikeDependencyLock.cmake:240-245`), and the imported targets the legacy module created under the same names. Keep the call where it is (after `project()`, line 60): 2.04's platform key needs the processor variable.
- Produces: a root graph with no literal `.dylib` in any root-graph CMake file except the binding-only `NativeStartup.cmake:47` branch.

**Review focus:** identical imported-target names and identical `dependency_provenance.json`. `diff` it before and after.

- [ ] **Step 1: Baseline**

```bash
cp build/native-release/dependency_provenance.json /tmp/prov-before.json
otool -L build/native-release/bin/bike-sim | grep -i mujoco > /tmp/otool-before.txt
grep -rn --include=CMakeLists.txt --include='*.cmake' 'dylib' CMakeLists.txt native/cmake native/tests
```

Expected: the root hits at 575, 588, 597 and 627 plus the module/test hits listed under Files.

- [ ] **Step 2: Switch the include and the names**

In `CMakeLists.txt`:
- replace the `Dependencies.cmake` include with `include("${CMAKE_CURRENT_SOURCE_DIR}/native/cmake/BikeDependencyLock.cmake")`;
- replace each literal `"${BIKE_DEP_mujoco_ROOT}/lib/libmujoco.3.12.0.dylib"` with `"${BIKE_DEP_mujoco_LIBRARY}"`;
- if the root calls `bike_rewrite_mujoco_load_name(<target> <dylib>)` from the legacy module, call the `BikeLinkage.cmake:227` variant instead and include `BikeLinkage.cmake` after `BikeNumerics.cmake`. The Darwin-only guard inside it must stay.

Apply the same substitution to `Checks.cmake:84`, `NativeStartup.cmake:43` and the 11 test-CMake literals.

- [ ] **Step 3: Verify Darwin parity**

```bash
cmake -S . -B build/native-release
cmake --build build/native-release -j10
diff <(jq -S . /tmp/prov-before.json) <(jq -S . build/native-release/dependency_provenance.json)
ctest --test-dir build/native-release -j8 --output-on-failure
otool -L build/native-release/bin/bike-sim | grep -i mujoco | diff /tmp/otool-before.txt -
grep -rn --include=CMakeLists.txt --include='*.cmake' 'dylib' CMakeLists.txt native/cmake native/tests | grep -v 'NativeStartup.cmake:47\|RecordingApplication.cmake'
```

Expected:
- the provenance diff is empty;
- CTest passes, including `migration_T03`;
- the `otool` diff is empty;
- the last grep prints only comments (if any); the `RecordingApplication.cmake` literals go in 2.04.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt native/cmake/BikeLinkage.cmake native/cmake/Checks.cmake native/cmake/NativeStartup.cmake native/tests/migration/CMakeLists.txt
git commit -m "build(native): root graph resolves .deps through BikeDependencyLock

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.02: Root graph adopts `BikePlatform` + `BikeCompilerPolicy` (Darwin parity, flag diff recorded)

**Files:**
- Modify: `CMakeLists.txt` (line numbers as measured in the 2.01 layout note):
  - lines 28 and 32-34 (the legacy `CompilerPolicy.cmake` include and the pre-`project()` `bike_resolve_macos_sdk()`);
  - lines 62-64 (`bike_verify_sdk_compat()` → `bike_verify_apple_sdk_compatibility()` inside `if(APPLE)`);
  - lines 66-73 (`BikeHostTarget` include, dispatch and its comment);
  - lines 77-82 (libc++ hardening cache block; `BikeCompilerPolicy.cmake:52-73` declares `BIKE_LIBCPP_HARDENING` and runs the detection);
  - lines 86-88 (duplicate `NATIVE_*` options; also declared by `BikeCompilerPolicy.cmake:52-73`);
  - line 100 (`BIKE_NUMERICS_APPLY_POLICY`).

`BikeCompilerPolicy.cmake` requires `CMAKE_CXX_COMPILER_LOADED` (line 44) and `BIKE_TARGET_SYSTEM` (line 48), so it must come after both `BikePlatform.cmake` and `project()`. Platform spelling: `BikeNumerics.cmake:238` keys on `CMAKE_HOST_SYSTEM_PROCESSOR`; every new key in Phase 2 (2.04 lock selection included) uses the same variable. Cross-compilation is refused by `BikePlatform`, so host = target.

**Interfaces:**
- Consumes:
  - `BikePlatform.cmake`: included before `project()`; dispatches the host, sets `BIKE_TARGET_SYSTEM`, owns the Apple SDK resolution.
  - `BikeCompilerPolicy.cmake`: after `project()`; probes flags; provides `bike_apply_native_policy(target)` and the bridges `bike_target_strict_policy` / `bike_detect_libcpp_hardening` (`BikeCompilerPolicy.cmake:1390-1403`).
- Produces: the root graph uses the same platform layer as `native/CMakeLists.txt:25-46`.

**Review focus:** the effective compile flags of every root target. Some change is expected because the newer policy is platform-aware. The commit body must list each added/removed flag with the reason.

- [ ] **Step 1: Baseline flags**

```bash
uv run --no-project python -c "
import json,shlex,collections
db=json.load(open('build/native-release/compile_commands.json'))
flags=collections.Counter()
for e in db:
    args=e.get('arguments') or shlex.split(e['command'])
    for a in args:
        if a.startswith('-') and not a.startswith(('-I','-o','-c','-D','-isystem','-MF','-MT','-MD')): flags[a]+=1
for f,c in sorted(flags.items()): print(c,f)
" > /tmp/flags-before.txt
```

- [ ] **Step 2: Switch**, mirroring the order in `native/CMakeLists.txt:25-52`:
  1. `include(native/cmake/BikePlatform.cmake)` before `project(bike_sim CXX)`. This replaces `CompilerPolicy.cmake` + `bike_resolve_macos_sdk()`.
  2. After `project()`: `include(native/cmake/BikeCompilerPolicy.cmake)`, then `if(APPLE) bike_verify_apple_sdk_compatibility() endif()`.
  3. Delete the now-duplicate `BikeHostTarget` dispatch (root 66-73): `BikePlatform` sets `BIKE_TARGET_SYSTEM`.
  4. Delete the root's local `BIKE_LIBCPP_HARDENING` cache block and `bike_detect_libcpp_hardening()` call (77-82) and the duplicate `NATIVE_*` options (86-88); `BikeCompilerPolicy.cmake:52-73` declares all of them. Confirm with `grep -n 'BIKE_LIBCPP_HARDENING\|option(NATIVE_' native/cmake/BikeCompilerPolicy.cmake` before deleting.
  5. Set `BIKE_NUMERICS_APPLY_POLICY` to `bike_apply_native_policy`.
  6. Replace every `bike_target_strict_policy(<t>)` call in the root with `bike_apply_native_policy(<t>)`. The bridge makes both work, but use one spelling.

- [ ] **Step 3: Verify and diff**

```bash
rm -rf /tmp/root-fresh && cmake -S . -B /tmp/root-fresh -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake -S . -B build/native-release && cmake --build build/native-release -j10
ctest --test-dir build/native-release -j8 --output-on-failure
# regenerate /tmp/flags-after.txt with the Step 1 script, then:
diff /tmp/flags-before.txt /tmp/flags-after.txt
```

Expected:
- the fresh configure succeeds;
- CTest is green;
- the flag diff is either empty or every line is explained in the commit body, e.g. "`-Wfoo` added by the platform-aware policy; already applied to the binding graph since 9bb59df".

A removed hardening flag (`-D_LIBCPP_HARDENING_MODE`, `-ftrivial-auto-var-init`, `-fstack-protector-strong`, `-ffp-contract=off`) is a failure.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt
git commit -m "build(native): root graph adopts BikePlatform and BikeCompilerPolicy

<flag diff explanation from Step 3>

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.03a: Root graph emits a schema-2 check context through the shared emitter

(Split from the original 2.03: the emitter's prerequisites are larger than first measured. 2.03b deletes the legacy layer.)

**Files:**
- Modify: `native/cmake/Checks.cmake`:
  - `:78-100`: the hand-built `_checks_first_party_roots`, `_checks_mujoco_library` (literal dylib + `REALPATH`), `_checks_required_includes` / `_checks_required_files` and their `EXISTS` loop move into the emitter's root branch (keep `tools/proto_native_bench` as a first-party root);
  - `:250-292`: replace the schema-1 `file(WRITE …native_check_context.json)` with a call to the `BikeCheckContext.cmake` emitter. `native_sources.json` keeps its current writer unless the emitter already produces it (check before deleting).
- Modify: `native/cmake/BikeCheckContext.cmake`. `bike_write_check_context` requires 11 variables today (`:56-60`): `_native_first_party_roots _native_mujoco_library MJ_DIR _native_dependencies_json _native_python_include _native_nb_include _native_robin_include BIKE_POLICY_REPORT_JSON BIKE_POLICY_REQUIRED_FLAGS BIKE_STDLIB_IDENTITY BIKE_STDLIB_VERSION`. After 2.02 the root has only the last four. Add:
  - a `NATIVE_DIR <dir>` argument (`cmake_parse_arguments`), defaulting to `${CMAKE_CURRENT_SOURCE_DIR}` for the binding graph. Every native-relative path (`:285` `…/src`, `:291` and `:457` `…/analysis_suppressions.json`) uses it; on the root `CMAKE_CURRENT_SOURCE_DIR` is the repo root;
  - a `BIKE_ROOT_BUILD` branch that:
    - drops the `_native_python_include` / `_native_nb_include` / `_native_robin_include` requirements and the `Python.h`, `nanobind/nanobind.h`, `tsl/robin_map.h` entries of the required-file list (`:292-294`) — those are FATAL `EXISTS` checks on a Python-free graph;
    - takes `_native_mujoco_library=${BIKE_DEP_mujoco_LIBRARY}`, `MJ_DIR=${BIKE_DEP_mujoco_ROOT}`, and the first-party roots `${NATIVE_DIR};${repo}/tools/proto_native_bench`;
    - synthesizes `_native_dependencies_json` as `{"numerics": <N>, "python": null}`. The emitter reads `numerics` verbatim (`:137-138`) and `python` at `:463`. `<N>` is the committed record entry for the build's key (`native/numerics.lock.json` `records[<key>]`) with its relative provider paths replaced by the absolute paths `BikeNumerics.cmake` resolved (`_native_numerics_provider_paths` or the equivalent variable — read `BikeNumerics.cmake` for the exact name). The emitter's own comment (`:130-136`) says readers expect machine-specific fields the committed entry lacks (install location, build ids, ABI transcript, evidence counts): list what `native/src/tools/check_context.cpp` actually validates under `numerics`, fill each field from a CMake-resolved value, and where no Python-free source exists make the C++ loader accept `null` for that field **only when `dependency_root` is present** (root contexts). Record the field list in the commit body;
    - emits the four root-only fields `cmake`, `dependency_root`, `dependency_lock`, `configure_source_dir` (today `Checks.cmake:282-285`) and `python_executable: null`.
- Modify: `native/src/tools/check_context.cpp`. Under schema 2 accept the four root-only fields (strings, existing paths), `python: null` / `python_executable: null` when `dependency_root` is present, and the nullable numerics fields above. Binding-graph contexts (no `dependency_root`) keep today's strictness. The Python oracle (`tools/native_checks.py`) only ever reads binding contexts; do not change it.
- Test: `native/tests/migration/migration_T56_check.cmake`

**Interfaces:**
- Produces: `build/native-release/native_check_context.json` with `schema_version: 2` on both platforms.

**Review focus:**
- Both graphs emit the same schema through one emitter. There is no second hand-written JSON writer.
- On Linux, `sdk_source` is `"not-applicable"` (hyphen); the old root writer wrote `not_applicable`.
- The root branch relaxes nothing for binding contexts.

- [ ] **Step 1: Failing check**

```cmake
string(JSON _root_schema GET "${_context_json}" schema_version)
if(NOT _root_schema STREQUAL "2")
  set(_failures "${_failures}\n  t56_root_context_schema: root graph emits schema ${_root_schema}, expected 2")
endif()
```

Add this after `file(READ "${_ctx}" _context_json)` in `migration_T56_check.cmake`. The Task 1.06 helper `t56_schema2` then becomes a no-op transform for the root context: keep it, since the cases still mutate copies.

```bash
cp native/build/native_check_context.json /tmp/ctx-binding-before.json   # binding-graph baseline for Step 3
ctest --test-dir build/native-release -R '^migration_T56$' --output-on-failure
```

Expected: FAIL with `t56_root_context_schema`.

- [ ] **Step 2: Implement** the emitter `NATIVE_DIR` argument and root branch, the `Checks.cmake` call, and the loader relaxations listed under Files. Order: emitter first (the binding graph must still emit a `jq -S`-identical context, checked in Step 3), then the root call, then the loader.

- [ ] **Step 3: Verify**

```bash
cmake -S native -B native/build && diff <(jq -S . /tmp/ctx-binding-before.json) <(jq -S . native/build/native_check_context.json)
cmake -S . -B build/native-release && cmake --build build/native-release -j10
ctest --test-dir build/native-release -j8 --output-on-failure
tools/run_native_checks.sh build/native-release
tools/run_native_checks.sh native/build
```

Expected: the binding-context diff is empty; everything else exits 0.

- [ ] **Step 4: Commit**

```bash
git add native/cmake/Checks.cmake native/cmake/BikeCheckContext.cmake native/src/tools/check_context.cpp native/tests/migration/migration_T56_check.cmake
git commit -m "build(native): root graph emits the schema-2 context through the shared emitter

<numerics field list: CMake source or null-in-root-context>

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.03b: Delete the legacy CMake layer

**Files:**
- Delete: `native/cmake/CompilerPolicy.cmake`, `native/cmake/Dependencies.cmake`. Measured 2026-10-21: only the root `CMakeLists.txt:28-29` includes them; `BikeCompilerPolicy.cmake` / `BikeDependencies.cmake` are different files and stay.
- Modify: `CMakeLists.txt` (drop the two includes if 2.01/2.02 left them), `native/dependencies.md:7` (cites `Dependencies.cmake`; name `BikeDependencyLock.cmake` instead).

**Review focus:** nothing else references the deleted modules or their functions (`bike_resolve_macos_sdk`, `BIKE_STRICT_WARNING_FLAGS`).

- [ ] **Step 1: Prove they are unreferenced**

```bash
grep -rn --exclude-dir=build --exclude-dir=.deps --exclude-dir=.venv \
  -e '[^e]CompilerPolicy.cmake' -e '[^e]Dependencies.cmake' -e 'bike_resolve_macos_sdk' -e 'BIKE_STRICT_WARNING_FLAGS' \
  CMakeLists.txt native/ tools/ docs/development
```

Expected: hits only inside the two files being deleted and `native/dependencies.md:7`.

- [ ] **Step 2: Delete, fix the doc line, verify**

```bash
git rm native/cmake/CompilerPolicy.cmake native/cmake/Dependencies.cmake
cmake -S . -B build/native-release && cmake --build build/native-release -j10
ctest --test-dir build/native-release -j8 --output-on-failure
tools/run_native_checks.sh build/native-release
```

Expected: all exit 0. The generated build identity's `native_source_sha256` (root `CMakeLists.txt:211-218` hashes every `native/cmake/*.cmake`) changes — expected, and true of every task that edits `native/cmake/`; say so in the commit body.

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt native/dependencies.md
git commit -m "build(native): delete the legacy CompilerPolicy/Dependencies modules

native_source_sha256 changes because native/cmake/*.cmake is hashed.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.04: Per-platform dependency lock (schema 2)

**Files:**
- Modify: `native/dependencies.lock.json` (schema 2)
- Modify: `native/cmake/BikeDependencyLock.cmake` (select per-platform entries; verify only the selected platform's archives)
- Modify: `native/CMakeLists.txt:115-118`. The binding graph reads `dependencies.<name>.extracted_root` directly; make it read through the same selection.
- Modify: `native/cmake/RecordingApplication.cmake:10-32`. It reads the lock itself (`dependencies duckdb extracted_root`, `dependencies duckdb required_files "lib/libduckdb.dylib"`) and hard-codes the dylib name; this path runs in the **binding** graph, where `bike::duckdb` is not pre-created, so after the move `string(JSON GET)` is FATAL. Select `platforms.<key>` the same way and take the library member name from the selected `required_files` (the `lib/libduckdb*` key). Lines 99-110 (glfw, platform-neutral) stay. `:229` (`install(FILES …libduckdb.dylib)`) moves to `${BIKE_DEP_duckdb_LIBRARY}` / the selected member.
- Modify: `native/cmake/Provenance.cmake` (`migration_T03`). Its scratch locks are written as schema 1 (`:253`, `:322`; `:33` is the provenance document, not a lock — leave it) and the stub-sha injection regex (`:315-316`) is keyed on the literal `"lib/libmujoco.3.12.0.dylib"`. Write the scratch locks with `"schema_version": 2`, embedding the `_t03_mj` entry with its `platforms` object, and inject the stub sha into `platforms.<key>.required_files.<locked library member>` (use `string(JSON SET)`, not a regex). Without this every T03 negative case fails with `unsupported lock schema_version` and reports "failed for an unexpected reason".
- Modify: `native/dependencies.md` (schema description and acquisition recipe)
- Test: the `migration_T03` script test (`native/cmake/Provenance.cmake`), which already scratch-configures with `-DBIKE_DEPENDENCY_LOCK=` (`:110`). Put the lock negatives there.

**Lock readers (measured 2026-10-21, `grep -rn 'dependencies.lock\|BIKE_DEPENDENCY_LOCK'`):** `BikeDependencyLock.cmake` (resolve), `Dependencies.cmake` (deleted in 2.03b), `native/CMakeLists.txt:115` (+ `:277` hashes it), `RecordingApplication.cmake:10`, `Provenance.cmake` (T03 scratch locks), root `CMakeLists.txt:355` (hashes it) and `:632` (installs it), `Checks.cmake:285` (path only). No `tools/*.py` or C++ source parses it; `migration/*.json` mention paths descriptively only.

**Interfaces:**
- Produces: lock schema 2. A dependency is either platform-neutral (today's shape) or carries `platforms` keyed by the record key `darwin-arm64` / `linux-aarch64` (the same keys as `native/numerics.lock.json`):

```json
"mujoco": {
  "version": "3.12.0",
  "license": "Apache-2.0",
  "license_files": ["mujoco-3.12.0/LICENSE", "mujoco-3.12.0/LICENSES_THIRD_PARTY.md"],
  "purpose": "…",
  "extracted_root": "mujoco-3.12.0",
  "platforms": {
    "darwin-arm64": {
      "acquisition": { "kind": "pypi-wheel-extract", "url": "…macosx_11_0_arm64.whl", "archive": "_archives/mujoco-3.12.0-cp313-cp313-macosx_11_0_arm64.whl", "archive_sha256": "5122a5affff0f592d088a89226360fe86a000b737cac6370ff5e5b552f20bc81", "pin_source": "…" },
      "required_files": { "include/mujoco/mujoco.h": "c7378e47…", "lib/libmujoco.3.12.0.dylib": "9e772461…" }
    },
    "linux-aarch64": { "acquisition": { … }, "required_files": { … } }
  }
}
```

  `BikeDependencyLock.cmake` computes the key `lower(BIKE_TARGET_SYSTEM)-CMAKE_HOST_SYSTEM_PROCESSOR` (the spelling `BikeNumerics.cmake:238` uses; `arm64` on macOS, `aarch64` on Linux). It selects `platforms.<key>`:
  - key present with an object → that entry is verified and resolved;
  - key present with JSON `null` → the dependency is **not required on this platform** and is skipped (no `BIKE_DEP_<name>_*` variables, no imported target);
  - key absent → `FATAL_ERROR "dependency '<name>' has no lock entry for platform '<key>'"`.
  It never falls back to another platform. A typo in a key therefore fails loudly instead of silently skipping a dependency.

**Review focus:**
- Darwin verification is byte-for-byte the same (same archives, same sha256).
- Platform-neutral entries (`tomlplusplus`, `rapidjson`, `eigen`, `glfw`, `cairo`) stay unchanged.

- [ ] **Step 1: Failing negative (wrong platform key)**

Add to `migration_T03`: a scratch copy of the lock whose `mujoco.platforms` holds only `"plan9-mips"`, configured with `-DBIKE_DEPENDENCY_LOCK=<scratch>` in a scratch build dir. Expect configure failure with `has no lock entry for platform`. Use the existing scratch-configure helper `_t03_configure` (`native/cmake/Provenance.cmake`). The `null` branch needs no scratch case: 2.05 adds `numpy_openblas` with `"darwin-arm64": null`, which every macOS configure exercises.

- [ ] **Step 2: Restructure the lock and every reader**
  - Move today's `mujoco` and `duckdb` entries under `platforms.darwin-arm64`; everything else stays top-level.
  - Bump `schema_version` to `2`.
  - Implement the selection in `BikeDependencyLock.cmake`. Accept schema 1 nowhere: the lock is checked in, so there is no old reader to support.
  - Update every reader listed above: `native/CMakeLists.txt:115-118`, `RecordingApplication.cmake:10-32,229`, and the T03 scratch locks/regex in `Provenance.cmake`.

- [ ] **Step 3: Verify Darwin parity**

```bash
cmake -S . -B build/native-release && cmake --build build/native-release -j10
ctest --test-dir build/native-release -j8 --output-on-failure
cmake -S native -B native/build && cmake --build native/build -j10
ctest --test-dir native/build -j8 --output-on-failure
diff <(jq -S . /tmp/prov-before.json) <(jq -S . build/native-release/dependency_provenance.json)
```

Expected: both CTest runs green (root `migration_T03` included; the binding graph builds `bike-recording` through the updated `RecordingApplication.cmake`), and the provenance diff shows only the lock file's own sha256 field.

- [ ] **Step 4: Document** the schema in `native/dependencies.md`: a "Lock schema 2" section with the shape above and the three-way selection rule (object / `null` / absent).

- [ ] **Step 5: Commit**

```bash
git add native/dependencies.lock.json native/cmake/BikeDependencyLock.cmake native/CMakeLists.txt native/cmake/RecordingApplication.cmake native/dependencies.md native/cmake/Provenance.cmake
git commit -m "build(native): per-platform dependency lock (schema 2)

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.05: Linux `.deps` acquisition in the lane and the `linux-aarch64` lock entries

**Files:**
- Modify: `native/dependencies.lock.json`. Add `platforms.linux-aarch64` for `mujoco` and `duckdb`, and a new platform-only dependency `numpy_openblas` with only `linux-aarch64`.
- Modify: `tools/linux/Dockerfile`. A new acquisition stage populates `/src/.deps` from pinned URLs with `sha256sum -c`; `container.sh sync` preserves `/src/.deps`.
- Modify: `native/dependencies.md` (Linux acquisition table)

**Interfaces:**
- Consumes: from `uv.lock` (verified 2026-10-20):
  - mujoco wheel `mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl`, URL `https://files.pythonhosted.org/packages/56/48/b1e477dc1caec78074ace77d3597f5768a6a1c11a258d7f214473f564b51/mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl`, sha256 `267c3fc36b5a0536073557760e36c464da2ad9a744b047893db73fe273931ad1`;
  - numpy wheel `numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl`, URL `https://files.pythonhosted.org/packages/29/f1/2a64a307d92c5d98f5255a4014eb43bb6103ee477087b61ecae44a3aa9b9/numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl`, sha256 `0aadf13b60048d501e05fa699efaf7734e2494f3498a4c2a5521d822640324f3`. Its `numpy.libs/libscipy_openblas64_-bb25d554.so` has sha256 `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`.
  - DuckDB: the `v1.5.6` GitHub release asset for Linux arm64. Confirm the exact asset name on `https://github.com/duckdb/duckdb/releases/tag/v1.5.6` (most likely `libduckdb-linux-aarch64.zip`; `libduckdb-linux-arm64.zip` is the other spelling DuckDB has used — **unverified**, check the release page) and measure its sha256 on download.
- Produces, in the container, `/src/.deps/{_archives,mujoco-3.12.0,duckdb-1.5.6,numpy-openblas-2.5.2,…}` laid out exactly as the lock's `extracted_root`/`required_files`:
  - `mujoco-3.12.0/include/mujoco/*.h` and `mujoco-3.12.0/lib/libmujoco.so.3.12.0`, taken from the wheel's `mujoco/` directory (the macOS wheel has `mujoco/libmujoco.3.12.0.dylib` and `mujoco/include/mujoco/mujoco.h`; the Linux wheel is expected to be the analogue — Step 1 confirms);
  - `numpy-openblas-2.5.2/lib/libscipy_openblas64_-bb25d554.so`;
  - `BIKE_VERSION` markers and license files per `native/dependencies.md:40-50`. License files are **not** under `mujoco/` in the wheel: the macOS wheel keeps them at `mujoco-3.12.0.dist-info/licenses/{LICENSE,LICENSES_THIRD_PARTY.md}`, and the lock's `license_files` are `mujoco-3.12.0/LICENSE` and `mujoco-3.12.0/LICENSES_THIRD_PARTY.md` — copy them across;
  - plus copies of the platform-neutral archives and trees.

**Review focus:**
- Every archive is checked with `sha256sum -c` before extraction.
- Lock `required_files` hashes are measured after extraction, and the measurement commands are in the commit body.
- The scipy-openblas `.so` is byte-identical to the one the binding graph uses (`5dafca…`).

- [ ] **Step 1: Acquire inside the running container and measure** (exploratory; the Dockerfile codifies it in Step 3):

```bash
tools/linux/container.sh exec bash -c '
set -euo pipefail
mkdir -p /tmp/acq && cd /tmp/acq
curl -fLO https://files.pythonhosted.org/packages/56/48/b1e477dc1caec78074ace77d3597f5768a6a1c11a258d7f214473f564b51/mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
echo "267c3fc36b5a0536073557760e36c464da2ad9a744b047893db73fe273931ad1  mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl" | sha256sum -c
curl -fLO https://files.pythonhosted.org/packages/29/f1/2a64a307d92c5d98f5255a4014eb43bb6103ee477087b61ecae44a3aa9b9/numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
echo "0aadf13b60048d501e05fa699efaf7734e2494f3498a4c2a5521d822640324f3  numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl" | sha256sum -c
DUCK=libduckdb-linux-aarch64.zip   # unverified: replace with the asset name the release page lists
curl -fLO "https://github.com/duckdb/duckdb/releases/download/v1.5.6/$DUCK"
sha256sum "$DUCK"
unzip -l mujoco-*.whl | grep -Ei "licen|libmujoco|mujoco\.h$"
unzip -o -q mujoco-*.whl "mujoco/*" "mujoco-3.12.0.dist-info/licenses/*" -d mj && find mj -name "libmujoco*" -o -name "mujoco.h" -o -path "*licenses/*" | xargs sha256sum
unzip -l numpy-*.whl | grep -Ei "licen"
unzip -o -q numpy-*.whl "numpy.libs/*" -d np && sha256sum np/numpy.libs/libscipy_openblas64_-bb25d554.so
unzip -o -q "$DUCK" -d dd && ls dd && sha256sum dd/duckdb.h dd/libduckdb.so
'
```

Expected:
- `OK` for the two wheel checks;
- the `libscipy_openblas64_` sha equals `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`.

Record every printed sha256. If the DuckDB asset name differs, use the real one and record it. If the mujoco wheel ships its headers somewhere other than `mujoco/include/mujoco/`, record the real layout. The extraction step maps it to `include/mujoco/`.

- [ ] **Step 2: Lock entries**
  - Add `platforms.linux-aarch64` to `mujoco` and `duckdb`, plus:

```json
"numpy_openblas": {
  "version": "2.5.2",
  "license": "BSD-3-Clause",
  "license_files": ["numpy-openblas-2.5.2/LICENSE.txt"],
  "purpose": "The ILP64 scipy-openblas provider bundled in the pinned NumPy wheel; the Linux numerics provider of record (native/numerics.lock.json linux-aarch64), so the Python-free graph links the same image the oracle used.",
  "extracted_root": "numpy-openblas-2.5.2",
  "platforms": {
    "darwin-arm64": null,
    "linux-aarch64": {
      "acquisition": { "kind": "pypi-wheel-extract", "url": "https://files.pythonhosted.org/packages/29/f1/2a64a307d92c5d98f5255a4014eb43bb6103ee477087b61ecae44a3aa9b9/numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl", "archive": "_archives/numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl", "archive_sha256": "0aadf13b60048d501e05fa699efaf7734e2494f3498a4c2a5521d822640324f3", "pin_source": "uv.lock numpy==2.5.2 cp313 manylinux aarch64" },
      "required_files": { "lib/libscipy_openblas64_-bb25d554.so": "5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798" }
    }
  }
}
```

  - Use NumPy's real license file name inside the wheel (`numpy-2.5.2.dist-info/LICENSE.txt` or the `licenses/` directory; Step 1's `unzip -l` prints it).
  - `"darwin-arm64": null` is the explicit "not required here" marker from 2.04's three-way rule: on Darwin Accelerate is the apple-system provider. Do not omit the key — an absent key is a configure error.

- [ ] **Step 3: Codify acquisition in the Dockerfile**

Add an `acquire` stage. It downloads every lock archive for `linux-aarch64` plus the platform-neutral archives (URLs and sha256 copied from the lock), verifies with `sha256sum -c`, and extracts into `/opt/bike-deps` with the exact layout (`BIKE_VERSION` = the lock `version`; license files copied to the lock's `license_files` paths). In the final image, `COPY --from=acquire /opt/bike-deps /opt/bike-deps`.

In `container.sh sync`, after extraction, run `ln -sfn /opt/bike-deps /src/.deps` when `/src/.deps` does not already exist. `/src/.deps` is preserved by the `! -name .deps` filter.

- [ ] **Step 4: Verify**

```bash
cmake -S . -B build/native-release && ctest --test-dir build/native-release -j8 --output-on-failure   # macOS: numpy_openblas null path
tools/linux/container.sh image
tools/linux/container.sh down && tools/linux/container.sh sync && tools/linux/container.sh venv
tools/linux/container.sh exec bash -c 'ls /src/.deps && cat /src/.deps/mujoco-3.12.0/BIKE_VERSION'
tools/linux/container.sh configure root-clang root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF 2>&1 | grep -E 'dependency .* sha256 verified|Error' 
```

Expected:
- macOS CTest green (the `numpy_openblas` `null` entry is skipped);
- every Linux dependency prints `sha256 verified`;
- Linux configure then fails **only** at the `BikeNumerics.cmake` record-only Linux refusal, which is Task 2.06.

`tools/linux/container.sh down` recreates the container from the new image. The `native-clang`/`native-gcc` build trees inside it are lost; reconfigure them as in Task 1.02 Step 7.

- [ ] **Step 5: Commit**

```bash
git add native/dependencies.lock.json native/cmake/BikeDependencyLock.cmake tools/linux/Dockerfile tools/linux/container.sh native/dependencies.md
git commit -m "build(linux): pinned linux-aarch64 .deps (mujoco .so, duckdb, scipy-openblas)

<sha256 measurement commands and results from Step 1>

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.06: Record-only numerics on Linux

**Files:**
- Modify: `native/cmake/BikeNumerics.cmake`:
  - 220-238: the record key and the Darwin-only refusal;
  - 264-276: libc must be null;
  - 314-321: record-only ACCELERATE only;
  - 490-526: provider path joined with site-packages;
  - 555-578: the SHARED import being LIVE-only.
- Modify: `CMakeLists.txt:272-296` (root provider-identity block). Today it unconditionally calls `bike_provider_probe_loaded_images` (FATAL off Darwin, `BikeProviderIdentity.cmake:153-157`), `bike_provider_probe_image_uuid(IMAGE ${BIKE_ACCELERATE_UMBRELLA_IMAGE})` and `bike_darwin_accelerate_build_id` (`sw_vers`/`xcrun`). Branch on `BIKE_NUMERICS_BACKEND_RESOLVED`: keep the three calls for `ACCELERATE`; for `NUMPY_OPENBLAS` skip them and append `<none>` to `_root_identity_build_ids` for each entry of `_native_numerics_provider_paths` (a file identity carries `build_id: null` — `tools/native_dependencies.py:415-418`; `BikeProviderIdentity.cmake:115` maps `<none>` to JSON null). `bike_provider_identity_required_images` and `bike_provider_identity_declarations` stay unconditional; read them for any other Darwin-only branch before relying on that.

**Interfaces:**
- Consumes:
  - `BIKE_TARGET_GLIBC` (set by the glibc probe at `BikeCompilerPolicy.cmake:240-247`);
  - `BIKE_DEP_numpy_openblas_ROOT` (Task 2.05);
  - the `linux-aarch64` numerics record (Task 1.10).
- Produces: `Bike::Numerics` in RECORD_ONLY mode on Linux:
  - an IMPORTED SHARED target at `${BIKE_DEP_numpy_openblas_ROOT}/lib/<record file name>`;
  - its sha256 checked against the record;
  - the same `BIKE_NUMERICS_SYM_*` macros as LIVE mode.

**Review focus:**
- No Python is invoked.
- The record's `platform.libc` must equal the probed host libc family (`glibc` when `BIKE_TARGET_GLIBC`, else fail: musl is not qualified).
- The provider image is the lock-verified file, never a path from site-packages.

- [ ] **Step 1: Red** (this is the Task 2.05 Step 4 failure):

```bash
tools/linux/container.sh sync
tools/linux/container.sh configure root-clang root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF 2>&1 | grep 'qualified for Darwin only'
```

- [ ] **Step 2: Implement**
  1. RECORD_ONLY key: drop the Darwin-only refusal. Key = `lower(BIKE_TARGET_SYSTEM)-CMAKE_HOST_SYSTEM_PROCESSOR` (unchanged spelling).
  2. libc: on Linux, require `platform.libc == "glibc"` and `BIKE_TARGET_GLIBC`, otherwise `FATAL_ERROR "numerics record libc '<x>' does not match the host (glibc probe: <bool>)"`. On Darwin, still require null.
  3. Backend: allow `NUMPY_OPENBLAS` in RECORD_ONLY when `BIKE_DEP_numpy_openblas_ROOT` is defined.
  4. Provider path: in RECORD_ONLY, the record's relative path `numpy.libs/<file>` maps to `${BIKE_DEP_numpy_openblas_ROOT}/lib/<file>`. Keep the existing sha256/containment checks (503-526), with the containment root = `BIKE_DEP_numpy_openblas_ROOT`.
  5. Remove the "LIVE-only in practice" comment and let the SHARED import serve both modes.
  6. Root identity block: the `ACCELERATE` / `NUMPY_OPENBLAS` branch described under Files. The Linux build tree loads the `.so` without extra linkage work: CMake adds an imported SHARED library's directory to the build RPATH.

- [ ] **Step 3: Verify**

```bash
tools/linux/container.sh configure root-clang root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF
tools/linux/container.sh exec grep -E 'numerics backend' /build/root-clang/CMakeFiles/CMakeOutput.log /dev/null || true
cmake -S . -B build/native-release && cmake --build build/native-release -j10 && ctest --test-dir build/native-release -j8 --output-on-failure
```

Expected: the Linux configure succeeds (past the identity block), printing `numerics backend NUMPY_OPENBLAS … provider /src/.deps/numpy-openblas-2.5.2/lib/libscipy_openblas64_-bb25d554.so`; macOS is green and its generated build-identity header is unchanged (`diff` it before/after: `find build/native-release -name '*build_identity*.hpp'`).

- [ ] **Step 4: Commit**

```bash
git add native/cmake/BikeNumerics.cmake CMakeLists.txt
git commit -m "build(native): record-only numerics on Linux from the locked scipy-openblas

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.07: Root graph builds and tests on Linux

**Files:** residual root-only Linux breaks. `Checks.cmake:84` and the `RecordingApplication.cmake` dylib literals are already gone (2.01, 2.03a, 2.04). Known at audit time:
- `native/cmake/Provenance.cmake` (`migration_T03`). Only Case 2 is `CMAKE_HOST_APPLE`-gated (`:174`); Cases 4/5/6 (`:262-330`) also run on Linux and hard-code Darwin spellings:
  - the macOS wheel symlink name `mujoco-3.12.0-cp313-cp313-macosx_11_0_arm64.whl` (`:300-302`) → derive from the selected lock entry's `platforms.<key>.acquisition.archive`;
  - `lib/libmujoco.3.12.0.dylib` as the stub output and sha-injection key → the selected `required_files` library member (after 2.04 the injection is a `string(JSON SET)` on that member);
  - `${BIKE_CXX} -dynamiclib` (`:306`) → `-shared -fPIC` when not Apple;
  - Case 2 on Linux: `readelf -d` / `ldd` instead of `otool -L`, asserting the `.deps` path and no `libpython`.
  Do not remove Darwin assertions.

**Interfaces:**
- Produces: `/build/root-clang` builds all targets; `ctest` passes on Linux.

**Review focus:**
- `migration_T03` on Linux exercises the same contract (Python-free root configure: no interpreter found, no wheel dirs, no network) with Linux spellings. The test is not skipped on Linux.
- This is the **first** execution of the Linux branches added in Tasks 1.06–1.09 (schema-2 platform validation, sysroot/tool resolution, Linux contract controls, libstdc++ probe): `bike-native-checks` and `migration_T56` exist only in the root graph, which could not configure on Linux before this phase. Failures there are fixed here, in the checker, with an L1 each.

- [ ] **Step 1: Build and list failures**

```bash
tools/linux/container.sh sync
tools/linux/container.sh build root-clang; tools/linux/container.sh failures root-clang
```

- [ ] **Step 2: Fix each root cause** (one commit each):
  - Any remaining literal `.dylib`: `grep -rn --include=CMakeLists.txt --include='*.cmake' 'dylib' CMakeLists.txt native/cmake native/tests` (expected: only `NativeStartup.cmake:47`, the binding-graph branch, and comments).
  - The T03 Darwin-isms listed under Files.
  - Linux-branch defects of the 1.06–1.09 checker work that the Step 3 commands expose.

- [ ] **Step 3: CTest + C++ sweeps on Linux**

```bash
tools/linux/container.sh build root-clang && tools/linux/container.sh ctest root-clang -j8
tools/linux/container.sh ctest root-clang -R '^(migration_T03|migration_T56)$' --output-on-failure
tools/linux/container.sh exec /build/root-clang/bin/bike-native-checks --kind context --build /build/root-clang
tools/linux/container.sh exec /build/root-clang/bin/bike-native-checks --kind context --build /build/native-clang
tools/linux/container.sh exec /build/root-clang/bin/bike-native-checks --kind diagnostic-controls --build /build/root-clang
tools/linux/container.sh exec tools/run_native_checks.sh /build/root-clang
tools/linux/container.sh exec tools/run_native_checks.sh /build/native-clang
tools/linux/container.sh exec bash -c 'ldd /build/root-clang/bin/bike-sim | grep -Ei "python|mujoco|duckdb|openblas"'
```

Expected:
- CTest passes, `migration_T03` and `migration_T56` included (not skipped);
- both contexts validate and the Linux diagnostic controls (`no_sdk`, `no_apple_tool`, `bad_sysroot`, `rejected_osx_sysroot`, `sysroot_preservation`, `stdlib_hardening`) pass;
- all sweeps pass on both builds through the C++ checker. Task 1.19 ran the binding-graph sweeps through the Python oracle (`tools/native_checks.py`) because `bike-native-checks` did not exist in the container yet; this is the first C++-checker sweep on Linux;
- `ldd` shows the `/src/.deps` `libmujoco.so.3.12.0`, `libduckdb.so` and `libscipy_openblas64_…so`, and **no** `libpython`.

Check `bike-native-checks --help` for the exact `--kind` spellings before running (`native/src/tools/checks_main.cpp`); adjust the two `--kind` lines if they differ.

- [ ] **Step 4: Commit** (per root cause, trailer as in Global Constraints), then append `## Task 2.07 Linux root graph` to the evidence log, including the Step 3 command outputs for T03/T56 and the context/controls checks.

---

### Task 2.08: Relocatable install rpath + install smoke on both platforms

**Facts (measured 2026-10-21):**
- Installed entry points (root graph): `bike-sim bike-playground bike-export bike-ride bike-settle bike-research bike-replay bike-batch bike-recording bike-research-sweep bike-native-ride` (11). `bike-native-checks`, `bike-python-audit` and `bike-test-child` have no `install()` rule and stay uninstalled.
- Every install rule is `install(TARGETS … OPTIONAL)` on purpose: `migration_T03` Case 2 (`Provenance.cmake:144-146` builds only `native_distance_tests native_station_tests`, then `:216-220` runs `cmake --install` on that focused build). `RecordingApplication.cmake:203-206` documents this. **Keep `OPTIONAL`.**
- All 11 return 0 on `--help`: `cli/application.cpp:14`, `app/native_ride_main.cpp:223`, `cli/recording.cpp:139`, and `bike-research-sweep` through `batch/sweep.cpp:21-23` (`research_sweep_main.cpp` forwards argv there).
- Darwin load names: the build-time rewrite (`BikeDarwinRewriteInstallName.cmake`, called from `BikeLinkage.cmake:223`) records the **absolute** `.deps` path of `libmujoco` in every executable's `LC_LOAD_DYLIB`. CMake's install rewrites rpaths of its own targets but never the load commands of imported libraries, so an installed executable still points into `.deps`. `libmujoco`'s own ID is `@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib`; `libduckdb.dylib`'s ID is already `@rpath/libduckdb.dylib` and needs only the rpath.
- `examples/` is already installed by `NativeStartup.cmake:146-148` (`*.toml`, `*.json`, `*.npz` → `share/bike_sim/examples`); `package_smoke` (3.44) relies on it.

**Files:**
- Modify: `native/cmake/BikeLinkage.cmake`:
  - add `INSTALL_RPATH` `$ORIGIN/../lib` (Linux) and `@loader_path/../lib` (Darwin) to every installed executable;
  - Darwin: a helper `bike_install_relocate_mujoco(<target>)` that adds `install(CODE …)` running `BikeDarwinRewriteInstallName.cmake` on `\${CMAKE_INSTALL_PREFIX}/bin/<target>` with `BIKE_OLD_NAME=${BIKE_DEP_mujoco_LIBRARY}` (the absolute build-time name) and `BIKE_NEW_NAME=@rpath/libmujoco.3.12.0.dylib`. Guard the `install(CODE)` with `if(EXISTS …)` so a focused (`OPTIONAL`) install that skipped the executable stays a no-op. Then `install_name_tool -id @rpath/libmujoco.3.12.0.dylib` on the installed dylib. The build tree keeps the absolute rewrite.
  - If an installed binary is killed on launch after `install_name_tool` (invalid ad-hoc signature: exit 137 / `Killed: 9`), add `codesign --force --sign -` to the same `install(CODE)`; record which was needed in the commit body.
- Modify: `CMakeLists.txt:622-633`, `native/cmake/RecordingApplication.cmake:198-231`, `native/cmake/NativeStartup.cmake:136-148`. Install the locked `libduckdb`, `libmujoco` and (Linux) `libscipy_openblas64_…so` into `lib/` through `${BIKE_DEP_*_LIBRARY}`; call `bike_install_relocate_mujoco` for every installed executable. `OPTIONAL` stays on every `install(TARGETS)`.
- Test: new CTest `install_smoke` in the root graph (`native/tests/install/install_smoke.cmake`, a `cmake -P` script).

**Interfaces:**
- Produces: `cmake --install build/native-release --prefix <dir>` yields a tree whose executables carry no `.deps` load path and find their libraries through `@loader_path/../lib` / `$ORIGIN/../lib`.

**Review focus:**
- The build-tree binaries keep their current load paths. Only installed copies get `@rpath`/`$ORIGIN`.
- Relocatability is proven by load commands, not by an environment scrub alone (with `.deps` still on disk an absolute path would also "work").

- [ ] **Step 1: Failing smoke test**

Create `native/tests/install/install_smoke.cmake`:

```cmake
# install_smoke — build every target, install into a scratch prefix, and prove
# the installed tree is relocatable: no load command points into .deps, every
# executable carries the relative rpath, and each entry point's --help runs
# with a scrubbed loader environment.
# -D BUILD_DIR=… -D SCRATCH_DIR=… -D DEPENDENCY_ROOT=…
set(_expected bike-sim bike-playground bike-export bike-ride bike-settle bike-research
    bike-replay bike-batch bike-recording bike-research-sweep bike-native-ride)
file(REMOVE_RECURSE "${SCRATCH_DIR}")
# A focused build (T03) installs a subset; the smoke must see the full set.
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BUILD_DIR}" RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "build failed: ${_rc}")
endif()
set(_prefix "${SCRATCH_DIR}/prefix")
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --prefix "${_prefix}"
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "install failed: ${_rc}")
endif()
foreach(_toml research/viewer_physics_welded.toml research/rough_uphill_savage.toml)
  if(NOT EXISTS "${_prefix}/share/bike_sim/examples/${_toml}")
    message(FATAL_ERROR "missing installed example ${_toml}")
  endif()
endforeach()
foreach(_name IN LISTS _expected)
  set(_bin "${_prefix}/bin/${_name}")
  if(NOT EXISTS "${_bin}")
    message(FATAL_ERROR "missing installed entry point ${_name}")
  endif()
  if(APPLE)
    execute_process(COMMAND otool -l "${_bin}" OUTPUT_VARIABLE _load)
    set(_want_rpath "@loader_path/../lib")
  else()
    execute_process(COMMAND readelf -d "${_bin}" OUTPUT_VARIABLE _load)
    set(_want_rpath "$ORIGIN/../lib")
  endif()
  string(FIND "${_load}" "${DEPENDENCY_ROOT}" _deps_at)
  if(NOT _deps_at EQUAL -1)
    message(FATAL_ERROR "${_name} still loads from ${DEPENDENCY_ROOT}")
  endif()
  string(FIND "${_load}" "${_want_rpath}" _rpath_at)
  if(_rpath_at EQUAL -1)
    message(FATAL_ERROR "${_name} lacks rpath ${_want_rpath}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=DYLD_LIBRARY_PATH --unset=DYLD_FALLBACK_LIBRARY_PATH --unset=LD_LIBRARY_PATH
            "${_bin}" --help
    RESULT_VARIABLE _rc OUTPUT_QUIET ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "${_name} --help exited ${_rc}: ${_err}")
  endif()
endforeach()
```

Register it in the root `CMakeLists.txt` after the existing `install()` calls:

```cmake
if(BUILD_TESTING)
  add_test(NAME install_smoke COMMAND "${CMAKE_COMMAND}"
    -D "BUILD_DIR=${CMAKE_BINARY_DIR}" -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/install_smoke"
    -D "DEPENDENCY_ROOT=${BIKE_DEPENDENCY_ROOT}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/native/tests/install/install_smoke.cmake")
  set_tests_properties(install_smoke PROPERTIES RUN_SERIAL TRUE)
endif()
```

`RUN_SERIAL`: the smoke runs `cmake --build` on its own build tree, which must not overlap other tests or a parallel build.

```bash
cmake -S . -B build/native-release && cmake --build build/native-release -j10
ctest --test-dir build/native-release -R '^install_smoke$' --output-on-failure
```

Expected now: FAIL with `still loads from …/.deps` (Darwin) or `lacks rpath` (Linux).

- [ ] **Step 2: Implement** the rpath/install rules and the Darwin relocation helper. Do **not** remove `OPTIONAL`.

- [ ] **Step 3: Verify on both platforms**

```bash
ctest --test-dir build/native-release -R '^(install_smoke|migration_T03)$' --output-on-failure
otool -L build/native-release/install_smoke/prefix/bin/bike-sim | grep -E 'mujoco|duckdb'
otool -L build/native-release/bin/bike-sim | grep mujoco      # build tree: still the absolute .deps path
tools/linux/container.sh sync && tools/linux/container.sh build root-clang && tools/linux/container.sh ctest root-clang -R '^(install_smoke|migration_T03)$'
tools/linux/container.sh exec bash -c 'readelf -d /build/root-clang/install_smoke/prefix/bin/bike-sim | grep -E "RUNPATH|RPATH|NEEDED"'
```

Expected: both tests pass on both platforms; the installed `bike-sim` loads `@rpath/libmujoco.3.12.0.dylib` and `@rpath/libduckdb.dylib`; the build-tree binary is unchanged; Linux shows `RUNPATH [$ORIGIN/../lib]`.

- [ ] **Step 4: Commit**

```bash
git add native/cmake/BikeLinkage.cmake CMakeLists.txt native/cmake/RecordingApplication.cmake native/cmake/NativeStartup.cmake native/tests/install/install_smoke.cmake
git commit -m "build(native): relocatable install (rpath, Darwin load names) and an install smoke

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### CHECKPOINT 2-A (L2, both platforms, root graph included)

- [ ] Run the full L2 block (including the `root-clang` lines). Append `## Checkpoint 2-A`. Commit the evidence.

---

### Task 2.09: Complete GLFW tree; GUI builds on macOS and Linux

**Files:**
- Modify: `native/dependencies.lock.json`, the `glfw` entry. `required_files` gains the `CMakeLists.txt` hash; today `.deps/glfw-3.4` lacks the top-level `CMakeLists.txt` that `RecordingApplication.cmake:99-100` requires. The archive itself is complete (`unzip -l .deps/_archives/glfw-3.4.zip` lists `CMakeLists.txt`, `src/CMakeLists.txt`, `deps/wayland/*.xml`). Do not pin the `deps/wayland/*.xml` files: with `GLFW_BUILD_WAYLAND OFF` nothing reads them.
- Modify: `native/dependencies.md` (the glfw extraction instruction: extract the whole archive tree)
- Modify: `native/cmake/RecordingApplication.cmake:95-122`. On Linux set `GLFW_BUILD_WAYLAND OFF`, `GLFW_BUILD_X11 ON` before `add_subdirectory`, since X11 is the qualified desktop transport (`viewer-platform-acceptance.md`; XWayland runs X11 clients).
- Host action (macOS): re-extract `.deps/glfw-3.4` from `.deps/_archives/glfw-3.4.zip`, following `native/dependencies.md`.

**Interfaces:**
- Produces: `-DBIKE_BUILD_GUI=ON` configures and builds `bike-ride` with the viewer on both platforms.

**Review focus:** archive sha256 unchanged (same zip); only extracted-file hashes are added.

- [ ] **Step 1: Red**

```bash
cmake -S . -B build/gui-macos -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=ON -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"; echo "exit=$?"
```

Expected: fails at `RecordingApplication.cmake:99-100` (missing glfw `CMakeLists.txt`).

- [ ] **Step 2: Re-extract and pin**

```bash
mv .deps/glfw-3.4 /tmp/glfw-3.4.old
unzip -q .deps/_archives/glfw-3.4.zip -d .deps/
cp /tmp/glfw-3.4.old/BIKE_VERSION .deps/glfw-3.4/BIKE_VERSION
shasum -a 256 .deps/glfw-3.4/CMakeLists.txt .deps/glfw-3.4/include/GLFW/glfw3.h
```

Add the `CMakeLists.txt` hash to the lock's `glfw.required_files` and update `native/dependencies.md`. The Dockerfile's acquire stage (Task 2.05) must extract the full tree too.

- [ ] **Step 3: Build GUI on both**

```bash
cmake -S . -B build/gui-macos -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=ON -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/gui-macos -j10
tools/linux/container.sh image && tools/linux/container.sh down && tools/linux/container.sh sync
tools/linux/container.sh configure root-gui root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=ON
tools/linux/container.sh build root-gui; tools/linux/container.sh failures root-gui
```

Expected: both build, and the failure count is `0`.

`down` recreates the container, so every `/build/*` tree (`native-clang`, `native-gcc`, `root-clang`) is gone. Reconfigure them as in Task 1.02 Step 7 and Task 2.05 Step 4 before the next task that uses them.

- [ ] **Step 4: Commit**

```bash
git add native/dependencies.lock.json native/dependencies.md native/cmake/RecordingApplication.cmake tools/linux/Dockerfile
git commit -m "build(native): full GLFW tree; GUI builds on macOS and Linux (X11)

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.10: Viewer `--close-on-finish` and the Linux Xvfb smoke (not acceptance, D11)

**Files:**
- Modify: `native/src/cli/argument_contracts.inc`. This is the C++ flag table (`kArgumentContracts`, consumed at `native/src/cli/arguments.cpp:108/181`). Add the option to `"bike-ride".options` in the existing entry shape: `"names": ["--close-on-finish"]`, `"dest": "close_on_finish"`, `"flag": true`, `"default": false`, `"type": "str"`, `"choices": null`, `"required": false`, `"positional": false` (copy an existing `flag: true` entry and change only names/dest).
- Modify: `native/src/cli/configuration.cpp` (validation next to the `--time-scale` rule at `:123-124`: valid only with an interactive physical/research run and `--duration`)
- Modify: `native/src/cli/ride.cpp:195` (set `options.close_on_terminal` where the `ViewerOptions` are filled)
- Leave `migration/cli-contracts.json` untouched. It is the captured Python argparse contract (`migration-cli-contracts/v1`, `programs.bike-ride` = `{cases, defaults, help, module}` — there is no `actions` key and no `native_only` notion), and nothing compares it with `argument_contracts.inc`, so a native-only flag cannot break T47 parity. Note the flag in the `bike-ride` section of the evidence log instead.
- Create: `tools/linux/gui_smoke.sh`
- Test: `native/tests/migration/test_t47.cpp` (CLI parsing case). `migration_T47` is registered in both graphs (`native/tests/migration/CMakeLists.txt:760-812`).

**Interfaces:**
- Consumes: `viewer::ViewerOptions::close_on_terminal` (`native/src/viewer/session.hpp:18`), already honored at `viewer/ride.cpp:157`.
- Produces: `bike-ride --backend native … --duration 1 --close-on-finish` opens the window, runs 1 s of simulated time, then closes and exits 0.

**Review focus:**
- The flag has no effect in headless mode; there it is rejected like `--time-scale` (`configuration.cpp:124`).
- The smoke is documented as **not** acceptance.

- [ ] **Step 1: Failing CLI test** (`test_t47.cpp`, using its existing argument-parsing helper):
  - `--close-on-finish` without `--duration` → `ArgumentError` containing `--close-on-finish requires --duration`;
  - with `--headless` → `ArgumentError` containing `--close-on-finish requires an interactive physical or research run`;
  - with `--duration 1` and no `--headless` → parses, and the produced options have `close_on_terminal == true`.

```bash
cmake --build build/native-release -j10 --target migration_T47 && ctest --test-dir build/native-release -R '^migration_T47$' --output-on-failure
```

Expected: FAIL (unknown option).

- [ ] **Step 2: Implement** the flag: the `argument_contracts.inc` entry, the validation in `configuration.cpp`, and `options.close_on_terminal = args.flag("close_on_finish")` in `ride.cpp`.

- [ ] **Step 3: Write `tools/linux/gui_smoke.sh`**

```bash
#!/usr/bin/env bash
# tools/linux/gui_smoke.sh BUILD_DIR — Xvfb smoke of the native viewer.
# Proves the GLFW/X11/GL stack links and a window opens, renders and closes.
# NOT viewer acceptance: docs/development/viewer-platform-acceptance.md
# excludes Xvfb; this never closes an acceptance row.
set -euo pipefail
build_dir="${1:?usage: gui_smoke.sh BUILD_DIR}"
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a -s "-screen 0 1280x720x24" \
    "$build_dir/bin/bike-ride" --backend native \
    --physics-config examples/research/viewer_physics_welded.toml \
    --track examples/research/rough_uphill_savage.toml \
    --duration 1 --close-on-finish --no-plots
```

```bash
chmod 0755 tools/linux/gui_smoke.sh && shellcheck tools/linux/gui_smoke.sh
```

Before writing the script, confirm that `examples/research/viewer_physics_welded.toml` and `rough_uphill_savage.toml` exist (`ls examples/research/`). These are the files the existing Linux guide uses.

- [ ] **Step 4: Run it**

```bash
tools/linux/container.sh sync && tools/linux/container.sh build root-gui
tools/linux/container.sh exec tools/linux/gui_smoke.sh /build/root-gui; echo "exit=$?"
```

Expected: `exit=0`.

A strict physical run may terminate at the documented `energy.constraint_work` baseline violation (`docs/development/linux.md`, Run section). If it does, the window must still close and the exit code must be the documented diagnostic code. Record that code in the script as the accepted exit and say so in the evidence log.

- [ ] **Step 5: Commit**

```bash
git add native/src/cli/argument_contracts.inc native/src/cli/configuration.cpp native/src/cli/ride.cpp tools/linux/gui_smoke.sh native/tests/migration/test_t47.cpp
git commit -m "feat(cpp): bike-ride --close-on-finish; Linux Xvfb viewer smoke

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 2.11: Linux full gate through the container

**Files:**
- Modify: `tools/linux/container.sh` (new subcommand `gate PROFILE`)
- Modify: `tools/run_tests.sh`, only for real Linux defects the first run exposes (for example the Darwin-only malloc exports are already gated)

**Interfaces:**
- Produces: `tools/linux/container.sh gate native`. It runs `bash tools/run_tests.sh native` inside the container, with `NATIVE_TEST_BUILD_PATH=/build/native-clang`, `UV_PROJECT_ENVIRONMENT=/opt/bike-venv` (image `ENV`, inherited by `docker exec`) and the root graph at `/src/build/native-release` (the path `ensure_native_checks` uses; `bike_checks_binary` falls back to `<repo>/build/native-release/bin`).

**Review focus:**
- The gate on Linux runs the same profile code as on macOS; there is no Linux-specific skip list.
- `/src/build/native-release` uses the same compiler as the `root-clang` lane. `ensure_native_checks` passes no compiler, so without `CXX` it would configure with Ubuntu's default GCC 13.
- `container.sh sync` keeps `/src/build` (Task 1.02 excludes it from the wipe), so repeated gates reuse the root build incrementally instead of rebuilding cold.

- [ ] **Step 1: Add the subcommand**

```bash
    gate)
        [[ $# -eq 1 ]] || die "usage: gate quick|native|full"
        # shellcheck disable=SC2016  # $0 is the bash -c positional, expanded in the container
        in_container bash -c 'export CXX=/usr/bin/clang++-18 NATIVE_TEST_BUILD_PATH=/build/native-clang; bash tools/run_tests.sh "$0"' "$1"
        ;;
```

Insert it before the `down)` case and update the usage comment. `tools/run_tests.sh` selects the build via `native_loader.selected_build` (`tests/reference/native_loader.py:32-37`), which honours `NATIVE_TEST_BUILD_PATH`. Its configure of `native/` targets `$build_dir` = `/build/native-clang`, which already exists with the clang settings. If `/src/build/native-release` was first configured with GCC by an earlier run, delete it once (`container.sh exec rm -rf /src/build/native-release`): CMake ignores a changed `CXX` on an existing cache.

```bash
shellcheck tools/linux/container.sh
```

Expected: no findings.

- [ ] **Step 2: L3 Linux** (no other L3 may run on this Mac meanwhile)

```bash
tools/linux/container.sh sync
tools/linux/container.sh gate native 2>&1 | tee /tmp/l3-phase2-linux.log; echo "exit=${PIPESTATUS[0]}"
```

Expected: the only failures are the 3 Linux baseline node IDs listed under "retained failures" in `docs/development/linux.md` (find them by that anchor — `awk '/retained failures/,/^## /' docs/development/linux.md | grep -oE 'tests/[^ `]+'` — not by line number; Phase 1 edits shift the file). Fix any other failure at its root cause first; each fix is its own commit, with an L1 for that fix.

- [ ] **Step 3: Close Phase 2**
  - Append `## L3 Linux (end of Phase 2)` to the evidence log.
  - Update `docs/development/linux.md`: qualified lane table, `impl/python-free-cpp` commit, root graph build commands, and the measured counts.
  - Commit `docs(migration): phase 2 evidence; Linux lane on the merged branch`.
  - `git push origin impl/python-free-cpp`.

---

# Phase 3: Port the tests and replace the tooling

Phase 3 is the bulk of the work. It consists of:
- 5 enabling tasks (3.01, 3.02, 3.03a, 3.03b, 3.04);
- 7 baseline-failure tasks (3.05–3.11);
- 33 test-port batches (3.12a–3.36; oversized batches are split into lettered sub-tasks that each fit one subagent session);
- 4 validation-rig tasks (3.37–3.40);
- 5 tooling tasks (3.41–3.45).

Checkpoints sit after 3.04, 3.11, 3.17, 3.23, 3.29b, 3.36, 3.40 and 3.45.

**Parallel lanes (D17).** After Task 3.04, batches without a "Depends on" link between them may run in up to three lanes:

```bash
# create lane A (B, C analogous) from the integration worktree
git worktree add ../pfc-lane-a -b lane/a impl/python-free-cpp
cp -cR .deps ../pfc-lane-a/.deps            # APFS clone of the Darwin .deps
cd ../pfc-lane-a
cmake -S native -B native/build && cmake --build native/build -j6
cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/native-release -j6
tools/linux/container.sh sync      # container name derives from the lane directory: bike-dev-pfc-lane-a
```

Then configure that lane's Linux builds as in Task 1.02 Step 7, plus `root-clang` (Phase 2 preamble).

Inside a lane task, every worktree path in the Global Constraints and the subagent prompt template means the lane worktree (`../pfc-lane-a`, …), not the integration worktree; the dispatching prompt substitutes it.

Lanes build with `-j6`, so three lanes fit on the 16-core host. A lane merges back with `git -C <integration worktree> merge --no-ff lane/a` after its batch commits. Run L2 in the integration worktree after each merge. Lanes never run L3.

## Shared procedure for test-port batches (Tasks 3.12–3.36)

Each batch task lists its pytest files. For **each file** the batch performs these steps in order.

**Lane path:** commands below are written for the integration worktree. A lane task (D17) runs them in its own lane worktree (e.g. `../pfc-lane-a`), with that lane's container (`tools/linux/container.sh` names the container after the checkout basename) and its own `build/` tree. Substitute the lane path wherever the Global Constraints or the subagent prompt template name the integration worktree.

1. **Triage** every `test-port-map.json` record of the file into `port`, `golden` or `drop` (D14):
   - **golden:** the test compares a C++ result with a Python oracle value. Bitwise forms: `assert_bitwise_equal`, `np.testing.assert_array_equal`, `np.array_equal`, `.tobytes() ==`, `.view(np.uint64)` comparisons, `==` on oracle-derived numbers. Tolerance forms: `np.testing.assert_allclose`, `pytest.approx` on an oracle-derived value, the tree helpers `conftest.py::_assert_tree`, `_native_runtime_support.py::assert_tree_close`, `test_native_drive_policies.py::assert_tree`. The C++ port compares with the captured oracle value.
   - **port:** the assertions are self-contained: contracts, exceptions, invariants, structure, analytic values (including `pytest.approx` against a literal or closed-form constant). The C++ port re-asserts them directly.
   - **drop:** the test only exercises Python internals (nanobind marshalling, `native_loader`, Numba twins, pytest plumbing). `evidence` must state why no C++ behavior is lost, in one sentence that names the C++ test that covers the behavior if one exists.

   **Capture sites for non-`assert_bitwise_equal` golden comparisons.** Only `assert_bitwise_equal` records automatically (Task 3.01). For every other golden comparison, edit the pytest file and insert an explicit call immediately before the comparison, on the oracle side:
   - bitwise forms: `_golden.record(expected, "<label>")`;
   - tolerance forms: `_golden.record(expected, "<label>", kind="close", rtol=<r>, atol=<a>)` with the tolerances the assertion uses (`pytest.approx` defaults: `rtol=1e-6`, `atol=1e-12`; `assert_allclose` defaults: `rtol=1e-7`, `atol=0`).

   `<label>` is the assertion's `msg`/field name, or `"<var>"` when there is none. Add `import _golden` to the file. The pytest file is edited only for capture; Task 4.05 deletes it.
   
   Write the decision into the record:

   ```json
   "disposition": "golden",
   "comparison_class": "bitwise-oracle",
   "golden_platforms": ["darwin-arm64", "linux-aarch64"],
   "native_test_id": "reference.native_pneumatic_brush::slip_patch_matches_oracle",
   "evidence": ["2026-10-2x: captured darwin-arm64 + linux-aarch64 goldens (commit <sha>); CTest reference.native_pneumatic_brush green on both"]
   ```

   Values for `comparison_class`: `bitwise-oracle`, `tolerance-oracle`, `analytic`, `contract`, `invariant`, `structural`, `python-only`.

   `golden_platforms` is mandatory on `golden` records. It lists the platforms that legitimately produce the node (a Linux-only test such as `test_native_linux_headless.py`, or a skip-guarded node, lists one). A one-platform record carries an `evidence` sentence naming the guard that limits it. Task 4.01 checks the golden tree against this list.

2. **Make sure the file is green on both platforms before capture:**

   ```bash
   PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run --frozen --group native python -m pytest -q <file>
   tools/linux/container.sh sync
   tools/linux/container.sh exec bash -c 'PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=/build/native-clang uv run --frozen --group native python -m pytest -q <file>'
   ```

   In the container `uv` uses `/opt/bike-venv` through `UV_PROJECT_ENVIRONMENT` (Task 1.02 Dockerfile). `container.sh venv` has already synced it against `uv.lock`, so `uv run --frozen` installs nothing. If it starts downloading, the venv is stale: run `tools/linux/container.sh venv` and record it, rather than letting the capture run pull packages.

   A failing node blocks the batch. If it is a documented baseline failure, its Task 3.05–3.11 must land first; the batch's "Depends on" line names it. If it is new, diagnose it as a regression.

3. **Capture goldens** (only for files with `golden` records):

   ```bash
   BIKE_GOLDEN_CAPTURE="$PWD/tests/fixtures/golden" PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
     uv run --frozen --group native python -m pytest -q <file>
   tools/linux/container.sh exec bash -c 'BIKE_GOLDEN_CAPTURE=/src/tests/fixtures/golden PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=/build/native-clang uv run --frozen --group native python -m pytest -q <file>'
   tools/linux/container.sh pull tests/fixtures/golden/linux-aarch64
   git status --short tests/fixtures/golden | wc -l
   du -sh tests/fixtures/golden
   ```

   Capture runs the **whole file** in one pytest process (no `-k`, no `--lf`), so every node of the file is rewritten from index 0 (Task 3.01 truncates a node's files on its first record). Only a node that was renamed or removed since an earlier capture leaves orphaned files. Before committing, list the node keys of the file's sidecars (`jq -r '.[0].node' <platform>/<file-key>/*.json | sort -u`), compare them with the file's collected node IDs, and delete the orphans.

   **Size budget:** if one batch adds more than 20 MiB of goldens, stop and report the largest entries (`du -a tests/fixtures/golden | sort -n | tail`). Do not commit them; the user decides between trimming and Git LFS.

4. **Write the C++ test** at the path plan-08 assigns: `native/tests/reference/<same stem>.cpp`, or the `native_source` of the record.
   - One C++ function per pytest test function; parametrized tests loop over the same parameter table.
   - Each function starts with a comment naming the pytest node it ports.
   - Each `bt::require` message states what the check establishes.
   - Golden comparisons use `bt::oracle::Node` from Task 3.01. Open it with the pytest node ID, consume the entries **in capture order** with `f64` / `f64_close` / `bytes`, passing the same label the Python side recorded, and end the function with `node.done()`. `done()` fails when an entry was not consumed, so the port cannot silently skip a comparison. Count the entries first: `jq length tests/fixtures/golden/darwin-arm64/<file-key>/<test-key>.json` (`bt::oracle::node_path(node)` prints the path).
   - Data-dependent loops in Python helpers (dict-key recursion such as `_assert_tree_bitwise`, per-row loops) define the capture order; the C++ port iterates in the same order (sorted keys where Python iterates a dict in insertion order, take the order from the sidecar labels).

5. **Register** the test in `native/tests/reference/CMakeLists.txt`:

   ```cmake
   bike_add_reference_test(native_pneumatic_brush
     SOURCES test_native_pneumatic_brush.cpp
     LIBRARIES bike_writers bike_configuration)
   ```

   Link exactly the libraries the code under test lives in; find them with `grep -n 'add_library' CMakeLists.txt native/cmake/*.cmake`.

6. **Red → green proof.** Run the new CTest node; it must pass on macOS and Linux:

   ```bash
   cmake --build build/native-release -j10 --target reference.native_pneumatic_brush
   ctest --test-dir build/native-release -R '^reference\.native_pneumatic_brush$' --output-on-failure
   tools/linux/container.sh sync && tools/linux/container.sh build root-clang reference.native_pneumatic_brush
   tools/linux/container.sh ctest root-clang -R '^reference\.native_pneumatic_brush$'
   ```

   Then prove it can fail. Temporarily flip one golden byte (`printf '\x01' | dd of=<one node .bin> bs=1 seek=0 conv=notrunc`), confirm the test fails with a message naming the node, index and label, then `git checkout -- <file>`. Do this once per batch, not per file.

7. **Ledger:** set each record's `disposition` and `evidence`. Set the `python-removal.json` row of the pytest file to `"status": "replacement_landed"` with `replacement_paths` = the C++ test path, and `verification_ids` = the CTest name.
   - The pytest file is **not** deleted (Task 4.05 does that).
   - It stops being a gate per ADR-0003 once the C++ test lands.

8. **Commit** (one commit per batch):

   ```bash
   git add native/tests/reference tests/fixtures/golden migration/test-port-map.json migration/python-removal.json
   git commit -m "test(cpp): T52 port <batch title>

   <files>: <n port / n golden / n drop>; goldens darwin-arm64 + linux-aarch64.

   Co-Authored-By: <the trailer the executing harness specifies>"
   ```

   Use `T53` instead of `T52` in the subject for T53 batches. If the batch edited the pytest file for capture sites (step 1), `git add` it too.

**L1 for a batch:**
- steps 2, 3 and 6 for its files;
- `ctest --test-dir build/native-release -L reference -j8` (every ported test so far, ≤ 5 min; if it grows past that, run only the batch's tests);
- `tools/run_native_checks.sh build/native-release` is **not** L1. It runs at the checkpoint.

---

### Task 3.01: Golden capture hook (Python) + golden readers and reference-test registration (C++)

**Depends on:** 2.02 (`bike_apply_native_policy` exists in the root graph only after `BikeCompilerPolicy.cmake` is included, `BikeCompilerPolicy.cmake:1363`).

**Files:**
- Create: `tests/reference/_golden.py`
- Modify: `tests/reference/_bits.py` (call the hook)
- Create: `native/tests/support/golden.hpp`, `native/tests/support/golden.cpp`
- Create: `native/tests/reference/CMakeLists.txt`
- Modify: `CMakeLists.txt` (root): `add_subdirectory(native/tests/reference)` right after `add_subdirectory(native/tests/migration)` at line 607
- Create: `tests/fixtures/golden/.gitattributes` with `*.bin binary`
- Test: `tests/reference/test_golden_hook.py` (new); `native/tests/reference/test_golden_support.cpp` (new)

**Layout decision (one file pair per node).** A per-call file layout would produce tens of thousands of tiny files (≈336 static `assert_bitwise_equal` sites × loops × 2 files × 2 platforms). Each pytest node therefore owns exactly two files:
- `<root>/<platform>/<file-key>/<test-key>.bin`: the C-order payloads of every recorded value, concatenated in call order;
- `<root>/<platform>/<file-key>/<test-key>.json`: a JSON array, one object per call, `{"index": n, "node": "<pytest node id>", "label": "<msg>", "kind": "bitwise"|"close", "dtype": "<f8", "shape": [..], "offset": <byte offset>, "nbytes": <n>, "rtol": <r|null>, "atol": <a|null>}`.

**Interfaces:**
- Produces (Python):
  - `_golden.record(value, label="", *, kind="bitwise", rtol=None, atol=None) -> None`. A no-op unless `BIKE_GOLDEN_CAPTURE` is set. `kind="close"` requires both tolerances.
  - `<platform>` = `f"{platform.system().lower()}-{platform.machine()}"` (`darwin-arm64`, `linux-aarch64`).
  - The node ID comes from `PYTEST_CURRENT_TEST` with its trailing ` (setup)`, ` (call)` or ` (teardown)` removed by an end-anchored regex. (`PYTEST_CURRENT_TEST` is `"<nodeid> (<phase>)"`; anchoring on the known phase suffix keeps parameter IDs that contain ` (` intact.) The suite does not use `pytest-xdist` (`pyproject.toml`, `tools/run_tests.sh`), so one process owns each node.
  - `node_path(node) -> "<file-key>/<test-key>"`. `<file-key>` is the part before the first `::`, `<test-key>` the rest; in both, every character outside `[A-Za-z0-9._-]` becomes `_`. A key longer than 150 characters is cut to 140 plus `-` plus the first 8 hex digits of `sha256(<unsanitized part>)`.
  - The index counts `record` calls per node, starting at 0. **On index 0 the node's two files are truncated**, so a re-capture never leaves entries from an earlier, longer run.
- Produces (C++), in `namespace bt::oracle` (kept apart from the existing fixture-file readers `bt::golden_text` / `bt::golden_f64(relative, span)` in `migration_test.hpp`):

```cpp
namespace bt::oracle {
struct Entry {
    std::size_t index;
    std::string label;
    std::string kind;                   // "bitwise" | "close"
    std::string dtype;                  // numpy dtype string, e.g. "<f8", "|b1", "<i8"
    std::vector<std::size_t> shape;
    std::vector<std::byte> bytes;       // raw C-order payload
    double rtol = 0., atol = 0.;        // kind == "close" only
};
// "<file-key>/<test-key>", computed exactly like _golden.node_path.
std::string node_path(std::string_view node);
// Reads <BIKE_GOLDEN_ROOT>/<BIKE_GOLDEN_PLATFORM>/<node_path>.{json,bin}.
// A missing node is a failure, never an empty pass.
class Node {
public:
    explicit Node(std::string_view node);
    [[nodiscard]] std::size_t size() const;
    // Each call consumes the next entry; its recorded label must equal `label`.
    // Bitwise binary64 (dtype "<f8", kind "bitwise"); the failure message names
    // node, index, label, first differing element and both bit patterns.
    void f64(std::span<const double> actual, std::string_view label);
    // kind "close": |a - b| <= atol + rtol * |b| per element (np.testing.assert_allclose
    // semantics, NaN equals NaN, tolerances from the sidecar).
    void f64_close(std::span<const double> actual, std::string_view label);
    // Exact byte comparison for every other dtype; dtype and shape must match.
    void bytes(std::span<const std::byte> actual, std::string_view dtype,
               std::span<const std::size_t> shape, std::string_view label);
    // Fails unless every recorded entry was consumed.
    void done() const;
};
}
```

- Produces (CMake): `bike_add_reference_test(<name> SOURCES … LIBRARIES … [LABELS …])`, which creates the executable and the CTest node `reference.<name>` with labels `reference;<labels>`.

**Review focus:**
- The capture path and the C++ lookup compute `<node_path>` identically (tested both ways, including the hashed long form).
- The hook never changes `assert_bitwise_equal`'s verdict.
- A port that consumes fewer entries than Python recorded fails (`done()`), and a port that consumes them in a different order fails on the label.

- [ ] **Step 1: Failing Python test** (`tests/reference/test_golden_hook.py`):

```python
"""The golden hook writes what assert_bitwise_equal compared, per node and
call order, truncates on re-capture, and writes nothing when capture is off
(completion plan 3.01)."""
import json
import numpy as np
from _bits import assert_bitwise_equal


def _node_files(root, request):
    import _golden
    return _golden.node_files(root, request.node.nodeid)


def test_capture_writes_expected_per_call(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    assert_bitwise_equal(np.array([1.0, -0.0]), np.array([1.0, -0.0]), "pair")
    assert_bitwise_equal(np.int64(7), np.int64(7))
    meta_path, bin_path = _node_files(tmp_path, request)
    meta = json.loads(meta_path.read_text())
    assert [m["index"] for m in meta] == [0, 1]
    assert meta[0]["dtype"] == "<f8" and meta[0]["shape"] == [2] and meta[0]["label"] == "pair"
    assert meta[1]["dtype"] == "<i8" and meta[1]["offset"] == 16
    assert bin_path.read_bytes() == np.array([1.0, -0.0]).tobytes() + np.int64(7).tobytes()


def test_close_kind_records_tolerances(tmp_path, monkeypatch, request):
    import _golden
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    _golden.record(np.array([0.5]), "speed", kind="close", rtol=1e-6, atol=1e-12)
    meta = json.loads(_node_files(tmp_path, request)[0].read_text())
    assert meta[0]["kind"] == "close" and meta[0]["rtol"] == 1e-6 and meta[0]["atol"] == 1e-12


def test_index_zero_truncates_an_earlier_capture(tmp_path, monkeypatch, request):
    import _golden
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    meta_path, bin_path = _node_files(tmp_path, request)
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text('[{"index": 0}, {"index": 1}, {"index": 2}]\n')
    bin_path.write_bytes(b"\0" * 64)
    _golden.record(np.array([3.0]))
    assert len(json.loads(meta_path.read_text())) == 1
    assert bin_path.read_bytes() == np.array([3.0]).tobytes()


def test_capture_off_writes_nothing(tmp_path, monkeypatch):
    monkeypatch.delenv("BIKE_GOLDEN_CAPTURE", raising=False)
    assert_bitwise_equal(np.array([2.0]), np.array([2.0]))
    assert not any(tmp_path.iterdir())


def test_node_path_is_filesystem_safe_and_bounded():
    import _golden
    long_node = "tests/reference/test_x.py::test_y[" + "a/b (c) " * 40 + "]"
    file_key, test_key = _golden.node_path(long_node).split("/")
    assert file_key == "tests_reference_test_x.py"
    assert len(test_key) <= 150 and all(c.isalnum() or c in "._-" for c in test_key)


def test_dotted_parameter_keeps_its_full_key(tmp_path):
    import _golden
    meta, payload = _golden.node_files(tmp_path, "tests/a.py::t[1.5]")
    assert meta.name == "t_1.5_.json" and payload.name == "t_1.5_.bin"


def test_phase_suffix_is_stripped_only_at_the_end(monkeypatch):
    import _golden
    monkeypatch.setenv("PYTEST_CURRENT_TEST", "tests/a.py::t[x (call) y] (call)")
    assert _golden._current_node() == "tests/a.py::t[x (call) y]"
```

```bash
PYTHONPATH=tests/reference uv run --frozen --group native python -m pytest -q tests/reference/test_golden_hook.py
```

Expected: FAIL (`ModuleNotFoundError: _golden`).

- [ ] **Step 2: Implement `tests/reference/_golden.py`**

```python
"""Golden capture for ADR-0003: freeze the oracle values a parity test
compares, so the C++ port can compare against them after Python is gone.

Active only when BIKE_GOLDEN_CAPTURE names a directory. Layout, one file
pair per pytest node: <root>/<platform>/<file-key>/<test-key>.bin holds the
payloads in call order, .json the per-call index (dtype, shape, offset,
label, kind, tolerances). native/tests/support/golden.cpp computes the same
<platform> and node path. The first record() of a node truncates both files,
so a re-capture never keeps entries from an earlier run.
"""
from __future__ import annotations

import hashlib
import json
import os
import platform
import re
from pathlib import Path

import numpy as np

_counters: dict[str, int] = {}
_UNSAFE = re.compile(r"[^A-Za-z0-9._-]")
_PHASE = re.compile(r" \((?:setup|call|teardown)\)$")


def platform_key() -> str:
    return f"{platform.system().lower()}-{platform.machine()}"


def _key(part: str) -> str:
    safe = _UNSAFE.sub("_", part)
    if len(safe) > 150:
        safe = safe[:140] + "-" + hashlib.sha256(part.encode()).hexdigest()[:8]
    return safe


def node_path(node: str) -> str:
    file_part, _, test_part = node.partition("::")
    return f"{_key(file_part)}/{_key(test_part)}"


def node_files(root, node: str) -> tuple[Path, Path]:
    # Not Path.with_suffix: a sanitized test key may contain "." ("_1.5_").
    base = Path(root) / platform_key() / node_path(node)
    return base.parent / (base.name + ".json"), base.parent / (base.name + ".bin")


def _current_node() -> str:
    return _PHASE.sub("", os.environ.get("PYTEST_CURRENT_TEST", ""))


def record(value, label: str = "", *, kind: str = "bitwise", rtol=None, atol=None) -> None:
    root = os.environ.get("BIKE_GOLDEN_CAPTURE")
    if not root:
        return
    if kind not in ("bitwise", "close") or (kind == "close") != (rtol is not None and atol is not None):
        raise ValueError(f"golden kind {kind!r} needs rtol/atol exactly when kind='close'")
    node = _current_node()
    if not node:
        raise RuntimeError("golden capture outside a pytest node")
    index = _counters.get(node, 0)
    _counters[node] = index + 1
    array = np.ascontiguousarray(np.asarray(value))
    meta_path, bin_path = node_files(root, node)
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta = [] if index == 0 else json.loads(meta_path.read_text())
    offset = 0 if index == 0 else bin_path.stat().st_size
    with bin_path.open("wb" if index == 0 else "ab") as payload:
        payload.write(array.tobytes())
    meta.append({"index": index, "node": node, "label": str(label), "kind": kind,
                 "dtype": array.dtype.str, "shape": list(array.shape),
                 "offset": offset, "nbytes": array.nbytes, "rtol": rtol, "atol": atol})
    meta_path.write_text(json.dumps(meta, indent=1, sort_keys=True) + "\n")
```

In `tests/reference/_bits.py`, make `assert_bitwise_equal` call `_golden.record(expected, msg)` (the oracle side: the second argument, normalized with `np.asarray`) **before** its assertions. Add the import `import _golden` next to the existing imports.

A passing comparison means the two sides are bitwise equal, so the recorded value does not depend on argument order. A failing comparison records `expected` and then fails the node; capture runs require the file to be green first (shared procedure step 2), so a red node's capture is never committed. Document both facts in the function's docstring.

- [ ] **Step 3: Green (Python)** — re-run Step 1's command. Expected: 7 passed.

- [ ] **Step 4: Failing C++ test** (`native/tests/reference/test_golden_support.cpp`):

```cpp
// Golden reader contract (completion plan 3.01). Fixtures are written by
// this test itself in the layout _golden.py uses, so the C++ and Python
// node-path rules are checked against one shared expectation table.
#include "support/golden.hpp"
#include "support/migration_test.hpp"

namespace {
void node_path_matches_python_rule() {
    // Expected values computed with tests/reference/_golden.py node_path().
    bt::require(bt::oracle::node_path("tests/reference/test_a.py::test_b[x/y z]") ==
                    "tests_reference_test_a.py/test_b_x_y_z_",
                "unsafe characters become underscores; file and test parts split at the first ::");
}
void missing_node_fails() {
    bool threw = false;
    try {
        bt::oracle::Node node("tests/reference/no_such.py::test_none");
    } catch (const std::exception &) {
        threw = true;
    }
    bt::require(threw, "a missing golden node is a failure, never an empty pass");
}
void unconsumed_entry_fails_done() {
    // Uses a two-entry node written by the test into a scratch root
    // (BIKE_GOLDEN_ROOT override for the test binary only); consumes one entry.
}
void label_mismatch_fails() {}
void close_kind_uses_recorded_tolerances() {}
} // namespace

int main() {
    return bt::run([] {
        node_path_matches_python_rule();
        missing_node_fails();
        unconsumed_entry_fails_done();
        label_mismatch_fails();
        close_kind_uses_recorded_tolerances();
    });
}
```

Fill the three stub bodies: each writes a small `.json`/`.bin` pair into a scratch directory under `$<TARGET_FILE_DIR>` and points the reader at it through a test-only constructor overload `Node(std::string_view node, const std::filesystem::path &root)`. `close_kind_uses_recorded_tolerances` checks that a value inside `atol + rtol*|b|` passes, one just outside fails, and NaN vs NaN passes.

The hashed suffix for long keys uses `bike::io::sha256` (`native/src/io/hash.hpp:34`; SHA-256 is the only digest there and the only one the plan needs). Write the long-key expectation with Python and paste it into the test as a further `bt::require` case:

```bash
PYTHONPATH=tests/reference uv run --no-project --with numpy python -c "import _golden; print(_golden.node_path('tests/reference/test_x.py::test_y[' + 'a/b (c) ' * 40 + ']'))"
```

- [ ] **Step 5: Implement** `golden.hpp/.cpp` and `native/tests/reference/CMakeLists.txt`:

```cmake
# Reference ports of the pytest suites (completion plan Phase 3).
string(TOLOWER "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}" BIKE_GOLDEN_PLATFORM)
add_library(bike_golden_test STATIC ../support/golden.cpp)
target_link_libraries(bike_golden_test PUBLIC bike_migration_test bike_io)
target_compile_definitions(bike_golden_test PUBLIC
  "BIKE_GOLDEN_ROOT=\"${CMAKE_SOURCE_DIR}/tests/fixtures/golden\""
  "BIKE_GOLDEN_PLATFORM=\"${BIKE_GOLDEN_PLATFORM}\"")
bike_apply_native_policy(bike_golden_test)

function(bike_add_reference_test name)
  cmake_parse_arguments(REF "" "" "SOURCES;LIBRARIES;LABELS" ${ARGN})
  set(_target "reference.${name}")
  add_executable(${_target} ${REF_SOURCES})
  target_link_libraries(${_target} PRIVATE bike_golden_test ${REF_LIBRARIES})
  bike_apply_native_policy(${_target})
  add_test(NAME ${_target} COMMAND ${_target})
  set_tests_properties(${_target} PROPERTIES LABELS "reference;${REF_LABELS}"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_target}>")
endfunction()

bike_add_reference_test(golden_support SOURCES test_golden_support.cpp)
```

`CMAKE_SOURCE_DIR` is the repository root, because this directory is only added from the root graph. On macOS, `CMAKE_SYSTEM_PROCESSOR` is `arm64`, matching Python's `platform.machine()`; on Linux it is `aarch64`, again matching. Check `add_subdirectory(native/tests/migration)` is still at root line 607 and `bike_migration_test` at `native/tests/migration/CMakeLists.txt:55` before editing.

Implementation notes for `golden.cpp`:
- read the JSON sidecar with `bike::io::parse_json` (`native/src/io/json.hpp:43`);
- read the `.bin` with `bt::text` (binary read, capped) and slice it by `offset`/`nbytes`; reject a sidecar whose offsets are not contiguous or whose last `offset + nbytes` differs from the file size;
- `f64` compares the `std::bit_cast<std::uint64_t>` of each element;
- `f64_close` implements `|a - b| <= atol + rtol * |b|` with `b` the recorded value, and treats NaN/NaN as equal (the `assert_allclose` default `equal_nan=True`).

- [ ] **Step 6: Green (C++, both platforms)**

```bash
cmake -S . -B build/native-release && cmake --build build/native-release -j10 --target reference.golden_support
ctest --test-dir build/native-release -R '^reference\.golden_support$' --output-on-failure
tools/linux/container.sh sync && tools/linux/container.sh build root-clang reference.golden_support && tools/linux/container.sh ctest root-clang -R '^reference\.golden_support$'
```

- [ ] **Step 7: Commit**

```bash
git add tests/reference/_golden.py tests/reference/_bits.py tests/reference/test_golden_hook.py native/tests/support/golden.hpp native/tests/support/golden.cpp native/tests/reference/ tests/fixtures/golden/.gitattributes CMakeLists.txt
git commit -m "test(cpp): golden capture hook and C++ golden readers (ADR-0003)

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 3.02: Complete the test-port map

**Files:**
- Modify: `migration/test-port-map.json`
- Modify: `migration/python-removal.json` (rows for files the map did not know)
- Create: `migration/ctest-graph-map.json` (binding-graph-only CTest nodes → root-graph successor or recorded drop; Step 2b)
- Modify: `CMakeLists.txt` (root): register the portable binding-only C++ tests (Step 2b)

**Interfaces:**
- Produces: one record per test function for:
  - the 9 unmapped files: `tests/reference/test_native_linux_headless.py`, `test_native_numeric_portability.py`, `test_native_platform_blas.py`, `test_native_platform_linkage.py`, `test_native_platform_recording.py`, `test_native_platform_replay.py`, `tests/test_async_recovery.py`, `tests/test_execution_identity.py`, `tests/test_recording_platform_compatibility.py`;
  - the 24 `tests/portability/test_*.py` files (545 test functions at audit time; the directory's helpers `interpreter_shim.py`, `offline_install_probe.py` and `worker_entry_probe.py` are not tests and get ledger rows only, Step 2);
  - `tests/reference/test_golden_hook.py` (triaged `drop` in Task 3.35: it tests the Python capture hook only).
  
  New records carry `"owner_task": "T53"` (platform/portability files) and `"disposition": "planned"`.

**Review focus:**
- The record shape is identical to the existing records (12 keys: `source_path, source_symbol, line, end_line, owner_task, native_test_id, native_source, collected_node_ids, assertions_capture, comparison_class, disposition, evidence`). `golden_platforms` is added per record only when a batch triages it `golden` (shared procedure step 1).
- After this task, every tracked `.py` file has a removal-ledger row (Step 2's `comm` check prints nothing).
- Existing records carry `comparison_class: "source-specific; assigned by T01 before implementation"`; new ones carry `"unassigned"`. Both mean "not triaged yet"; Task 4.01 accepts neither.

- [ ] **Step 1: Generate the records with a one-off AST scan** (not committed as a tool):

```bash
uv run --no-project python - <<'EOF'
import ast, json, pathlib
pm_path = pathlib.Path("migration/test-port-map.json")
pm = json.loads(pm_path.read_text())
known = {(r["source_path"], r["source_symbol"]) for r in pm["records"]}
files = [
    "tests/reference/test_native_linux_headless.py", "tests/reference/test_native_numeric_portability.py",
    "tests/reference/test_native_platform_blas.py", "tests/reference/test_native_platform_linkage.py",
    "tests/reference/test_native_platform_recording.py", "tests/reference/test_native_platform_replay.py",
    "tests/test_async_recovery.py", "tests/test_execution_identity.py",
    "tests/test_recording_platform_compatibility.py", "tests/reference/test_golden_hook.py",
] + sorted(str(p) for p in pathlib.Path("tests/portability").glob("test_*.py"))
added = 0
for f in files:
    tree = ast.parse(pathlib.Path(f).read_text())
    stem = pathlib.Path(f).stem
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name.startswith("test_"):
            if (f, node.name) in known:
                continue
            pm["records"].append({
                "source_path": f, "source_symbol": node.name, "line": node.lineno,
                "end_line": node.end_lineno, "owner_task": "T53",
                "native_test_id": f"ported::{f[:-3].replace('/', '::')}::{node.name}",
                "native_source": f"native/tests/reference/{stem}.cpp",
                "collected_node_ids": [], "assertions_capture": "completion plan 3.02 static scan",
                "comparison_class": "unassigned", "disposition": "planned", "evidence": []})
            added += 1
pm["static_test_functions"] = len(pm["records"])
pm_path.write_text(json.dumps(pm, indent=2, ensure_ascii=False) + "\n")
print("added", added)
EOF
git diff --stat migration/test-port-map.json
```

Expected: `added` ≥ 545 + the platform files' function count. The file uses 2-space indentation (checked at planning time); `git diff --stat` must show only insertions plus the `static_test_functions` line.

- [ ] **Step 2: Removal-ledger rows** for the same files: one row each, following the existing row schema (`path, sha256, line_count, owner_task, action, replacement_paths, verification_ids, deletion_gate, status, snapshot_sha256, snapshot_line_count, tracked, presence`), `status: planned`, `action: port_assertions_and_fixture_orchestration`. Compute `sha256`/`line_count` from the file. Add rows the same way for the 22 `tools/research/**/*.py` from Task 1.03, with `action: remove_or_relocate_pending_OD-1`, and for the Python tools that had no row (Task-1.03 evidence plus the D18 list):
  - `tools/dev.py`, `tools/ci_portability.py`, `tools/portability_qualification.py`, `tools/portability_baseline.py`;
  - `tools/native_toolchain.py`, `tools/native_runtime.py`, `tools/native_dependencies.py`, `tools/numeric_portability_probe.py`;
  - `tools/offline_bundle.py`, `tools/offline_manifest.py`, `tools/prepare_offline_tools.py`;
  - `tools/migration/capture_reference.py`, `tools/migration/_t10_capture.py`.

  Each gets `action` from D18: `delete_with_python`, `replace_with_cmake_ctest_cpack` or `replace_with_ci_checklist`.

  Then close the gap between the ledger and the tree. Every tracked `.py` without a row gets one (51 at planning time, before this task's additions; among them `src/bike_sim/native/{execution_identity,provider_build_id}.py`, the `tests/reference` helpers `conftest.py`, `_bits.py`, `_encode_oracle.py`, `_legacy_sensor_oracle.py`, `_native_runtime_support.py`, `_physical_hud_oracle.py`, `_recording_support.py`, `native_loader.py`, `native_safety_helpers.py`, `native_test_reporter.py`, the three `tests/portability` helpers and Task 3.01's `_golden.py`). Helpers get `action: delete_with_python`.

  ```bash
  comm -13 <(jq -r '.[].path' migration/python-removal.json | sort -u) <(git ls-files '*.py' | sort)
  ```

  Expected after the edit: no output. (`python-removal.json` is a top-level array with 2-space indentation; write it back the same way.)

- [ ] **Step 2b: Binding-graph-only CTest nodes.** Task 4.06 deletes `native/CMakeLists.txt`; every C++ test registered only there must have a root-graph successor first, or a recorded reason to drop it. List them from the two configured trees:

  ```bash
  comm -23 <(ctest --test-dir native/build -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort) \
           <(ctest --test-dir build/native-release -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort)
  ```

  At planning time this printed `native_bench_{bare,glue,heavy}`, `native_contract_tests`, `native_platform_blas_tests`, `native_platform_lapack_tests`, `native_rtsan_control`, `native_sanitizer_control`; Task 1.18 adds `native_platform_sincos_tests`. Re-run it; the live output is authoritative.

  Write `migration/ctest-graph-map.json`, a top-level array, one object per binding-only name:

  ```json
  {"binding_name": "native_platform_blas_tests", "root_name": "native_platform_blas_tests",
   "disposition": "moved", "task": "3.02", "evidence": "registered in the root graph from native/tests/test_platform_blas.cpp"}
  ```

  `disposition` is `moved` (same test, root graph, same name), `replaced` (a root test under another name covers it; `root_name` names it), or `drop` (`root_name: null`, `evidence` says why no behavior is lost). Planned entries:
  - `native_contract_tests`, `native_platform_blas_tests`, `native_platform_lapack_tests`, `native_platform_sincos_tests` → `moved` **in this task**. Mirror their `add_executable` blocks (`native/CMakeLists.txt:419-464`, plus Task 1.18's sincos block) in the root `CMakeLists.txt` next to `native_distance_tests` (root `:599`), with `bike_apply_native_policy` instead of `bike_apply_native_graph_policy`, and `add_test` with the same names. `native_contract_tests` needs `$<TARGET_OBJECTS:bike_test_allocation_faults>` from `native/tests/migration` (already added at root `:607`).
  - `native_bench_{bare,glue,heavy}` → `moved` in this task: the root graph already builds `native_bench` (root `:567`) but registers none of the three; copy the three `add_test` calls (`native/CMakeLists.txt:526-534`). These keep `tools/proto_native_bench/bench.cpp` and `state_format.hpp` alive; Task 4.05 must not delete those two files.
  - `native_sanitizer_control`, `native_rtsan_control` → `replaced`, `task: "3.42"`, `root_name` = the clean-mode controls Task 3.42 registers (`sanitizer_clean_control`, `rtsan_clean_control`). Task 3.42 flips their `evidence` to the landed commit.

  Verify:

  ```bash
  cmake --build build/native-release -j10 --target native_contract_tests native_platform_blas_tests native_platform_lapack_tests native_platform_sincos_tests native_bench
  ctest --test-dir build/native-release -R '^(native_contract_tests|native_platform_(blas|lapack|sincos)_tests|native_bench_(bare|glue|heavy))$' --output-on-failure
  tools/linux/container.sh sync && tools/linux/container.sh build root-clang native_contract_tests native_platform_blas_tests native_platform_lapack_tests native_platform_sincos_tests native_bench && tools/linux/container.sh ctest root-clang -R '^(native_contract_tests|native_platform_(blas|lapack|sincos)_tests|native_bench_(bare|glue|heavy))$'
  ```

  Task 4.06's pre-delete check relies on exactly this file: every name the `comm -23` above prints must appear as a `binding_name` in `migration/ctest-graph-map.json`, and each entry's `root_name` must be non-null and present in the root `ctest -N` list unless its `disposition` is `drop`.

- [ ] **Step 2c: Ask the user OD-1 now.** Present the 22 `tools/research/**/*.py` files from Task 1.03 with their provenance, and ask: delete (default) or relocate outside the repository. Record the answer in the evidence log, and replace every `remove_or_relocate_pending_OD-1` action with `delete_with_python` or `relocate_outside_repo`. Task 4.01 rejects any row still pending, and Task 4.05 executes the recorded answer. If the user is not reachable, stop the task here: the rest of Phase 3 does not depend on it, but Task 3.02 is not done until OD-1 is answered.

- [ ] **Step 3: Verify JSON validity and counts**

```bash
uv run --no-project python -c "import json;d=json.load(open('migration/test-port-map.json'));print(len(d['records']))"
uv run --no-project python -c "import json;d=json.load(open('migration/python-removal.json'));print(len(d))"
```

- [ ] **Step 4: Commit**

```bash
git add migration/test-port-map.json migration/python-removal.json migration/ctest-graph-map.json CMakeLists.txt
git commit -m "docs(migration): map the unmapped platform/portability tests and tools

Root graph now registers the binding-only C++ tests; the rest are mapped
in migration/ctest-graph-map.json for the binding-graph deletion.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 3.03: `bike-recording golden freeze|compare` (replaces `tools/golden_episode.py` + `tools/episode_compare.py`)

This is two sub-tasks, each sized for one session. "Depends on 3.03" elsewhere in the plan means both are done.

**Facts checked at planning time (re-check before editing):**
- No episode artifact is committed today: `test_golden_episode_v2.py:48-52` writes one into `tmp_path`. Sub-task 3.03a freezes and commits one.
- `native/src/io` reads `.npz` (`parse_npz`, `npz.cpp`, CRC32 in its anonymous namespace) and encodes single `.npy` arrays (`encode_npy`, `double_array`, `string_array`, …, `npy.hpp:29-37`). There is no ZIP writer, and Python `save` writes `np.savez` (ZIP, stored members).
- `mj_saveModel(model, nullptr, buffer, size)` is already used (`runtime/project_config.cpp:59`); writing to a file name is the same call with `filename`.
- There is no `bike::app::Ride` and no `runtime::RideControl`. Python `capture_episode(env, steps, control)` takes a research environment. The C++ equivalent is `research::make_environment` → `std::unique_ptr<runtime::NativeResearchRuntime>` (`research/environment.hpp:75-76`, `runtime/research.hpp:111`). The control type is `bike::writers::drivetrain::RideControl` (`writers/drivetrain.hpp:37`).
- `bike-recording` already has a `compare` subcommand (DuckDB recordings, `--atol/--rtol`, `recording.cpp:22,99,128`). `recording_command` validates a flat command set (`:90`) with positional-count rules (`:117`).
- Tests importing the Python tools: `test_episode_compare.py`, `test_golden_episode.py`, `test_golden_episode_v2.py`, `test_native_brake_resistance.py`, `test_native_distributed_tire.py`, `test_native_pneumatic_tire.py`, `test_native_rider_forces.py`, `test_native_state_restore.py`, `test_native_suspension.py`, `test_native_tire.py`.

**Files:**
- Modify: `native/src/io/npy.hpp`, `native/src/io/npz.cpp`: add `std::vector<std::byte> encode_npz(const NpzArchive &)` (3.03a).
- Create: `native/src/golden/episode.hpp`, `native/src/golden/episode.cpp`. These are the C++ port of `golden_episode.py`'s `capture_episode`, `flatten_row`, `save`, `load_episode` and of `episode_compare.py`'s `replay_episode`, `compare_rows`, `compare_episode`.
- Create: `native/src/cli/recording_golden.cpp` (subcommand dispatch, 3.03b)
- Modify: `native/src/cli/recording.cpp` (route `golden`, 3.03b)
- Modify: `native/cmake/RecordingApplication.cmake` (new library `bike_golden` linked into `bike-recording`)
- Create: `tests/fixtures/golden/<platform>/episode/pinned_savage_40/{episode.npz,manifest.json,model.mjb}` for `darwin-arm64` and `linux-aarch64`, plus the committed inputs `tests/fixtures/golden/episode/pinned_savage_40/{physics.toml,request.toml,freeze.py}` (3.03a; `freeze.py` is the one-off capture script, deleted with Python in Task 4.05)
- Test: `native/tests/reference/test_golden_episode_tool.cpp`

**Interfaces:**
- Consumes: the artifact format the Python tool writes: `episode.npz` with the keys of `tools/golden_episode.py:280-292` (`channels`, `channel_names`, `init_<k>`, `controls`, `control_names`, `state_qpos`, `state_qvel`, `state_act`, `state_warmstart`, `state_time`, `ctrl_written`, `qfrc_applied`, `force_names`, `forces`, `tire_state`, `tire_state_names`), plus `manifest.json` (with `first_failure`, `non_numeric`, `absent`, `controls`) and `model.mjb`.
- Produces:

```cpp
namespace bike::io {
// ZIP with STORED members (local headers, central directory, EOCD), the
// layout np.savez writes; CRC32 shared with parse_npz.
[[nodiscard]] std::vector<std::byte> encode_npz(const NpzArchive &archive);
}
namespace bike::golden {
// One flattened sample: leaf path -> value. Rows carry strings, None and the
// _SENTINELS spellings (episode_compare.py:73-110), so a leaf is a variant.
using Leaf = std::variant<std::monostate, bool, std::int64_t, double, std::string>;
using Row = std::vector<std::pair<std::string, Leaf>>;   // insertion order kept
struct EpisodeArtifact { /* every golden_episode.EpisodeArtifact field, listed in the header comment */ };
EpisodeArtifact capture_episode(runtime::NativeResearchRuntime &env, std::size_t steps,
                                const writers::drivetrain::RideControl &control);
void save(const EpisodeArtifact &episode, const std::filesystem::path &out_dir, const mjModel *model);
EpisodeArtifact load_episode(const std::filesystem::path &dir);
// Port of compare_rows: row-count check, union of leaf names, _close per leaf,
// truncation after 50 diffs.
std::vector<std::string> compare_rows(const std::vector<Row> &golden, const std::vector<Row> &candidate,
                                      double rtol = 1e-12, double atol = 1e-12);
}
```

  CLI:
  - `bike-recording golden freeze --request <request.toml> --steps N --out DIR` writes the artifact;
  - `bike-recording golden compare --golden DIR --request <request.toml> [--rtol R --atol A]` replays and prints one line per mismatch; exits 0 on match, 1 on mismatch, 2 on usage error.
  - `recording_command` intercepts `arguments[0] == "golden"` **before** its flat `parse_args` and dispatches `golden_command(arguments.subspan(1))`. The top-level help lists `golden freeze|compare` separately from the existing DuckDB `compare`. The `bike-recording` help string is pinned by `cli-contracts.json`; add `golden` to the top-level help only if `migration_T47` stays green, otherwise document it in `golden --help` alone.

**Review focus:**
- The C++ `save` writes artifacts that Python `load_episode` reads unchanged, and C++ `load_episode` reads the committed Python-written artifacts. Cross-reading is tested both ways while Python exists.
- `compare_rows` reproduces `_close` (`episode_compare.py:73`) exactly: sentinel-string mapping, `np.isclose(float(a), float(b), rtol, atol, equal_nan=True)` with `b` the **candidate** (`|a-b| <= atol + rtol*|b|`), fallback to `==` for non-numeric leaves, and the failure-row semantics of `_same_failure` (`:106`).

#### 3.03a: npz writer, artifact load/save/compare, committed Python artifact

- [ ] **Step 1: Read the Python source end to end**

```bash
sed -n 1,342p tools/golden_episode.py
sed -n 1,125p tools/episode_compare.py
sed -n 1,60p tests/reference/test_golden_episode_v2.py
sed -n 1,40p tests/reference/test_pinned_topology.py
```

List every field of `EpisodeArtifact` and every `np.savez` key in the header comment of `episode.hpp`.

- [ ] **Step 2: Freeze the reference artifact with the Python tool, on both platforms.** Reproduce `test_golden_episode_v2.py::_ep` (pinned physics config, `examples/research/rough_uphill_savage.toml`, `--duration 1 --dt .00125 --diagnostic-model-limits`, 40 steps, `RideControl(human_torque_nm=35.)`). Write the physics TOML that `test_pinned_topology.py::_pinned_config` generates to `tests/fixtures/golden/episode/pinned_savage_40/physics.toml` and commit it, so C++ and Python read the same file. Save this script as `tests/fixtures/golden/episode/pinned_savage_40/freeze.py`:

```python
"""One-off: freeze the pinned-savage 40-step episode with the Python tool
for this platform (completion plan 3.03a). Deleted with Python (4.05)."""
import sys
from pathlib import Path
sys.path[:0] = ['.', 'tests/reference']
from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl
from tools.golden_episode import capture_episode, save
import _golden

fixture = Path('tests/fixtures/golden/episode/pinned_savage_40')
args = research_cli.parser().parse_args([
    '--physics-config', str(fixture / 'physics.toml'),
    '--track-file', 'examples/research/rough_uphill_savage.toml',
    '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits', '--out', '/tmp/g-py-out'])
env = research_cli.make_environment(args)
episode = capture_episode(env, 40, RideControl(human_torque_nm=35.))
save(episode, Path('tests/fixtures/golden') / _golden.platform_key() / 'episode' / 'pinned_savage_40', env.sim.model)
```

```bash
uv run --frozen --group native python tests/fixtures/golden/episode/pinned_savage_40/freeze.py
tools/linux/container.sh sync && tools/linux/container.sh exec uv run --frozen --group native python tests/fixtures/golden/episode/pinned_savage_40/freeze.py
tools/linux/container.sh pull tests/fixtures/golden/linux-aarch64
du -sh tests/fixtures/golden/*/episode
```

Write `request.toml` next to `physics.toml`: the research request that `research::read_research_request` accepts and that describes the same run (copy the shape of `tests/fixtures/migration/requests/welded_flat.toml`). Check the size budget (shared procedure step 3).

- [ ] **Step 3: Failing C++ test** (`test_golden_episode_tool.cpp`, first three checks):

```cpp
// bike-recording golden: artifact and comparison contract (plan 3.03).
// 1. load_episode reads the committed Python-frozen artifact for this
//    platform (tests/fixtures/golden/<platform>/episode/pinned_savage_40):
//    40 rows, state_qpos 40 x nq, forces 40 x len(force_names) x nv;
// 2. save -> load is lossless (bitwise on every numeric array, equal names
//    and manifest), and parse_npz(encode_npz(x)) == x for a mixed-dtype archive;
// 3. compare_rows reports exactly one mismatch when one leaf of one row
//    moves beyond atol + rtol*|candidate|, none for NaN vs NaN, and a
//    row-count line when the lengths differ.
```

Read the artifact through `BIKE_GOLDEN_ROOT`/`BIKE_GOLDEN_PLATFORM` (Task 3.01 definitions). Write scratch output under `$<TARGET_FILE_DIR>`.

```bash
cmake --build build/native-release -j10 --target reference.golden_episode_tool; echo "exit=$?"
```

Expected: build failure (no `bike::golden`, no `encode_npz`).

- [ ] **Step 4: Implement** `encode_npz` (move `crc32` out of the anonymous namespace so both directions share it), `load_episode`, `save`, `flatten_row`, `compare_rows`. Port line by line, keeping the Python line references in comments.

- [ ] **Step 5: Green + cross-read.** Add a fourth check that `save`s the loaded artifact to `$<TARGET_FILE_DIR>/resaved` and prints that path; then confirm Python reads the C++-written copy:

```bash
ctest --test-dir build/native-release -R '^reference\.golden_episode_tool$' --output-on-failure
uv run --frozen --group native python -c "import sys; sys.path.insert(0,'.'); from tools.golden_episode import load_episode; e=load_episode('<printed resaved path>'); print(len(e.rows), e.state_qpos.shape)"
```

Expected: CTest passes; Python prints `40 (40, <nq>)`.

- [ ] **Step 6: Linux + commit**

```bash
tools/linux/container.sh sync && tools/linux/container.sh build root-clang reference.golden_episode_tool && tools/linux/container.sh ctest root-clang -R '^reference\.golden_episode_tool$'
git add native/src/io/npy.hpp native/src/io/npz.cpp native/src/golden native/cmake/RecordingApplication.cmake native/tests/reference tests/fixtures/golden/episode tests/fixtures/golden/darwin-arm64/episode tests/fixtures/golden/linux-aarch64/episode
git commit -m "feat(cpp): T55 golden episode artifacts (npz writer, load/save/compare)

Co-Authored-By: <the trailer the executing harness specifies>"
```

#### 3.03b: capture, replay and the `golden` CLI

- [ ] **Step 1: Failing test.** Add to `test_golden_episode_tool.cpp`: `capture_episode` on `make_environment(<fixture>/request.toml)` for 40 steps with `RideControl{.human_torque_nm = 35.}`; `compare_rows` against the committed artifact's rows must be empty, and the state/force arrays must be bitwise equal (ADR-0003: parity holds until goldens are the authority; this artifact **is** the golden). Expected: build failure (no `capture_episode`).

- [ ] **Step 2: Implement** `capture_episode` (name the sample-flattening source in `runtime/step.hpp` in a comment), `replay_episode`, `compare_episode` and `recording_golden.cpp`. A mismatch in Step 1 is a parity defect: diagnose it (native-engineering.md "Establish the contract"); never loosen the tolerance to pass.

- [ ] **Step 3: CLI checks**

```bash
ctest --test-dir build/native-release -R '^reference\.golden_episode_tool$' --output-on-failure
F=tests/fixtures/golden/episode/pinned_savage_40
build/native-release/bin/bike-recording golden compare --golden tests/fixtures/golden/darwin-arm64/episode/pinned_savage_40 --request $F/request.toml; echo "exit=$?"
build/native-release/bin/bike-recording golden freeze --request $F/request.toml --steps 40 --out /tmp/g-cpp
uv run --frozen --group native python -c "import sys; sys.path.insert(0,'.'); from tools.golden_episode import load_episode; e=load_episode('/tmp/g-cpp'); print(len(e.rows), len(e.channel_names))"
build/native-release/bin/bike-recording golden compare --golden /tmp/g-cpp --request $F/request.toml --rtol 0 --atol 0; echo "exit=$?"
build/native-release/bin/bike-recording golden compare --golden /tmp/g-cpp; echo "exit=$?"
ctest --test-dir build/native-release -R '^migration_T47$' --output-on-failure
```

Expected: first `compare` exit 0; Python prints `40 <n>`; self-compare exit 0; missing `--request` exit 2; `migration_T47` green (CLI contract unchanged for the existing commands).

- [ ] **Step 4: Linux + commit**

```bash
tools/linux/container.sh sync && tools/linux/container.sh build root-clang reference.golden_episode_tool bike-recording && tools/linux/container.sh ctest root-clang -R '^reference\.golden_episode_tool$'
tools/linux/container.sh exec /build/root-clang/bin/bike-recording golden compare --golden /src/tests/fixtures/golden/linux-aarch64/episode/pinned_savage_40 --request /src/tests/fixtures/golden/episode/pinned_savage_40/request.toml
git add native/src/golden native/src/cli/recording_golden.cpp native/src/cli/recording.cpp native/cmake/RecordingApplication.cmake native/tests/reference
git commit -m "feat(cpp): T55 bike-recording golden freeze/compare

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 3.04: `bike-measure-realtime` (replaces `tools/measure_realtime.py` + `src/bike_sim/sim/realtime_measurement.py`)

**Files:**
- Create: `native/src/tools/measure_realtime_main.cpp`
- Create: `native/src/runtime/realtime_measurement.hpp`, `.cpp`. Port of `src/bike_sim/sim/realtime_measurement.py`. Read it fully; the reuse candidates are `native/src/diag.hpp` and `native_bench`.
- Modify: `native/cmake/RecordingApplication.cmake` (target `bike-measure-realtime`, installed)
- Test: `native/tests/reference/test_measure_realtime.cpp`

**Interfaces:**
- Consumes: the Python CLI, `tools/measure_realtime.py:231-247`. Flags:
  - `--backend {python,native}` (native only in C++; `python` is rejected with exit 2);
  - `--physics-config` (default `examples/research/viewer_physics_welded.toml`), `--track` (default `examples/research/rough_uphill_savage.toml`), `--duration` (30), `--dt`, `--warmup` (1), `--runs` (3);
  - `--record-decimation`/`--decimate` (80), `--seed`, `--out` (required, new or empty dir), `--compare`, `--benchmark`.
- The JSON report schema is the one `tests/reference/test_measure_realtime_report.py` pins (including the nested `comparability` and `provenance.sanitizer_options` objects, `:60-114`). Read that test and copy its key paths into the header comment.
- Output files, as the Python tool writes them: `report.json` (`measure_realtime.py:345`) plus one `{phase}-{index:02d}.json` per run (`:325`).
- Provenance fields the C++ tool must derive without Python: `git_sha` = `git rev-parse HEAD` in the repository root (`:187`; run `git` through `platform/process.hpp`, and record `null` with a reason when the binary runs outside a checkout), the dirty flag from `git status --porcelain` over the native sources and the tool's own source (the Python tool scopes it to `src/bike_sim` and itself, `:188-189`), and `machine` = the `platform.platform()` equivalent (`uname` sysname-release-machine). The Python `source_fingerprint` (`bike_sim.validation.environment`) becomes the native source digest the build already records (`native_source_sha256`, root `CMakeLists.txt` provenance).
- Produces: `bike-measure-realtime` with the same flags, the same output files and the same report key paths/semantics.

**Review focus:**
- Warmup is excluded from the timing.
- Report fields separate validity from performance, as the Python report does.
- Timing uses `std::chrono::steady_clock`.

- [ ] **Step 1:** Read `tools/measure_realtime.py`, `src/bike_sim/sim/realtime_measurement.py` and `tests/reference/test_measure_realtime_report.py`. Write the key list.
- [ ] **Step 2: Failing test**. `test_measure_realtime.cpp` checks:
  - a 0.2 s native run with `--runs 1 --warmup 0` writes a report containing every pinned key;
  - `--out` pointing at a non-empty directory exits 2;
  - `--runs 0` exits 2.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Compare** against the Python tool on the same inputs. Keys and non-timing fields must match exactly; timing fields need only be present and finite:

```bash
rm -rf /tmp/rt-cpp /tmp/rt-py
build/native-release/bin/bike-measure-realtime --backend native --duration 1 --runs 1 --warmup 0 --out /tmp/rt-cpp
uv run --frozen --group native python tools/measure_realtime.py --backend native --duration 1 --runs 1 --warmup 0 --out /tmp/rt-py
diff <(cd /tmp/rt-cpp && ls) <(cd /tmp/rt-py && ls)
uv run --no-project python - <<'PY'
import json
def paths(v, p=""):
    if isinstance(v, dict):
        for k, x in v.items():
            yield from paths(x, f"{p}.{k}" if p else k)
    elif isinstance(v, list):
        for i, x in enumerate(v):
            yield from paths(x, f"{p}.{i}")
    else:
        yield p, v
a = dict(paths(json.load(open('/tmp/rt-cpp/report.json'))))
b = dict(paths(json.load(open('/tmp/rt-py/report.json'))))
print("key-path diff:", sorted(set(a) ^ set(b)))
timing = ("seconds", "real_time_factor", "rtf", "wall", "per_step", "_ns", "_us", "timestamp", "git_sha", "machine")
print("value diff:", sorted(k for k in set(a) & set(b) if a[k] != b[k] and not any(t in k for t in timing)))
PY
```

Expected: no file-list difference; `key-path diff: []`; `value diff: []`. Before trusting the `timing` filter, print the excluded paths and confirm each is a timing or host field; extend the tuple only with fields that are, and list them in the evidence log.

- [ ] **Step 5: Linux + commit**

```bash
tools/linux/container.sh sync && tools/linux/container.sh build root-clang bike-measure-realtime reference.measure_realtime && tools/linux/container.sh ctest root-clang -R '^reference\.measure_realtime$'
git add native/src/tools/measure_realtime_main.cpp native/src/runtime/realtime_measurement.hpp native/src/runtime/realtime_measurement.cpp native/cmake/RecordingApplication.cmake native/tests/reference
git commit -m "feat(cpp): T55 bike-measure-realtime

Co-Authored-By: <the trailer the executing harness specifies>"
```

### CHECKPOINT 3-A (L2)

- [ ] Run the full L2 block. Append `## Checkpoint 3-A`, commit the evidence.

---

## Baseline-failure tasks (D16)

Each of Tasks 3.05–3.11 follows the systematic-debugging loop:
1. reproduce the failing nodes (command given);
2. trace to the root cause, writing the hypothesis and two independent checks into the evidence log (native-engineering.md "Establish the contract");
3. fix the cause, in C++, in Python, in fixtures, or in the test if the **test** is wrong (with a written argument);
4. run the nodes green on both platforms;
5. remove the nodes from the baseline list. Locate the lists by text, never by line number (Task 1.03's merge and later edits move them):

   ```bash
   awk '/failing node IDs, identical in the baseline/,/^## /' docs/TESTING.md | grep -oE 'tests/[^ `]+'
   awk '/^retained failures$/,/rather than waived/' docs/development/linux.md | grep -oE 'tests/[^ `]+'
   ```

   When removing nodes, update the count in the macOS sentence (`The 24 failing node IDs, identical in the baseline …`) but keep the phrase `failing node IDs, identical in the baseline` and the following `## ` heading, so the anchor stays valid. Keep `retained failures` on its own line in `linux.md` for the same reason. (Both anchors were checked at planning time in the current tree and in `feat/drop-oracle-parity`, where the macOS list moves from line 794 to 723.)

**Physics stop rule:** if the cause is a physical-model choice rather than a defect (for example a test asserts 25 km/h within 10 s and the plant reaches 24), the task stops. It records the measured values and the candidate decisions, and asks the user. It never re-tunes physics or edits a threshold unilaterally.

A stopped task is not done until the user's decision has been carried out, and that decision must end in one of three states:
- **approved fix**: the model change the user chose, landed and green on both platforms;
- **approved threshold change**: the test edited, with the user's decision and the written physical argument quoted in the commit body and the evidence log;
- **drop**: the node is triaged `drop` in `test-port-map.json` with evidence naming the decision, and no C++ node is created for it.

No node may stay red into Phase 4: Task 4.03's `full` gate must exit 0.

Commit subject: `fix(<area>): <cause> (baseline <file>)`.

### Task 3.05: Rider allocation family

Nodes (macOS baseline):
- `tests/reference/test_rider_allocation.py`:
  - `::test_healthy_step_allocates_a_feasible_command`
  - `::test_allocated_wrenches_close_the_rider_dynamics`
  - `::test_infeasible_intent_keeps_the_physical_bounds`
- `tests/reference/test_coupled_rider_task.py`:
  - `::test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step`
  - `::test_tissue_damping_does_not_eat_the_muscle_budget`
- `tests/reference/test_closed_form_rider.py::test_flat_launch_delivers_crank_torque`
- `tests/reference/test_joint_strength.py::test_stepped_effort_respects_strength_and_the_power_budget`

- [ ] Reproduce:

```bash
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run --frozen --group native python -m pytest -q tests/reference/test_rider_allocation.py tests/reference/test_coupled_rider_task.py tests/reference/test_closed_form_rider.py tests/reference/test_joint_strength.py 2>&1 | tail -40
```

- [ ] Check first whether all seven share one cause, e.g. a common allocation-feasibility path in `src/bike_sim/sim/ride/rider_response_allocation.py` (`linear_region_infeasible`, `scipy.optimize.linprog`). Record the evidence.
- [ ] Fix, verify green on macOS **and** in the container, update `docs/TESTING.md` (step 5 anchors), commit.

### Task 3.06: Seated pedalling and posture

Nodes:
- `tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[20.0]`, `[40.0]` and `[60.0]`
- `tests/reference/test_seated_pedaling_cycle.py::test_free_coast_is_an_intent_not_a_crank_lock`
- `tests/reference/test_seated_posture_program.py::test_pulse_redistributes_load_through_inertia_only`

Steps as in the shared loop. Reproduce with the files above. Check the dead-spot crank-freeze finding before forming a hypothesis (`motor_clutch` opens at vertical, legs stall; coast hysteresis toggles 4/s).

### Task 3.07: Rider welds and planar arms

Nodes:
- `tests/test_rider_welds.py`:
  - `::test_held_rider_weight_is_carried_by_the_connects`
  - `::test_contact_diagnostics_carry_the_solved_weight`
  - `::test_steep_grade_saddle_shear_exceeds_friction`
- `tests/reference/test_planar_arms.py::test_compiled_topology_has_two_arms_and_two_grip_sites`

### Task 3.08: Period buffer

Nodes, both in `tests/reference/test_period_buffer.py`:
- `::test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms`
- `::test_runtime_batch_matches_preserved_scalar_step_and_flush`

### Task 3.09: Realistic pedelec acceptance

Nodes, all in `tests/reference/test_realistic_pedelec_acceptance.py`:
- `::test_flat_reaches_25_kmh_within_10_s`
- `::test_motor_follows_the_rider_through_a_shift_without_an_extra_cut`
- `::test_fifteen_percent_climb_holds_12_kmh`
- `::test_savage_is_ridden_to_the_end`

These are physical acceptance criteria. The physics stop rule applies with full force. Measure and report first.

### Task 3.10: Realtime gate

Nodes:
- `tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_savage.toml]`
- `…[rough_uphill_extreme.toml]`

- [ ] Measure with `bike-measure-realtime` (Task 3.04), after a warmup run (the AGENTS.md Numba caveat applies to the Python backend), on both platforms. Record the achieved RTF.
- [ ] Realtime ≥ 1× is a hard requirement. A miss is a performance defect, to be diagnosed with the perf roadmap (`.scratch/perf-roadmap/` in the main checkout). A test bug is fixed in the test.

### Task 3.11: Linux-only failures

Nodes (Linux baseline):
- `tests/reference/test_native_platform_recording.py::test_publication_permission_denied_then_retry`. Suspect: the container runs as root, so `chmod`-based denial does not deny. If confirmed, run the test as a non-root user in the lane: add `useradd -m bike` + `USER bike` to the Dockerfile for test runs, or a `container.sh exec --user`. Never skip-as-root.
- `tests/reference/test_native_runtime_commands.py::test_engine_failure_keeps_committed_prefix`
- `tests/reference/test_native_runtime_ownership.py::test_fatal_engine_error_preserves_completed_prefix_and_can_reset`. Suspect for both: the MuJoCo fatal-error path (`engine::invoke`, private TLS handler ABI) behaves differently with glibc. Use `migration_T04`'s fatal-engine containment as the reference.

### CHECKPOINT 3-B (L2)

- [ ] Run the full L2 block plus the former-baseline pytest proof. The file list is explicit: the baseline lists shrink as Tasks 3.05–3.11 land, so they cannot name the files any more.

```bash
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run --frozen --group native python -m pytest -q \
  tests/reference/test_closed_form_rider.py tests/reference/test_coupled_rider_task.py \
  tests/reference/test_joint_strength.py tests/reference/test_period_buffer.py \
  tests/reference/test_planar_arms.py tests/reference/test_realistic_pedelec_acceptance.py \
  tests/reference/test_realtime_gate.py tests/reference/test_rider_allocation.py \
  tests/reference/test_seated_pedaling_cycle.py tests/reference/test_seated_posture_program.py \
  tests/test_rider_welds.py
tools/linux/container.sh sync && tools/linux/container.sh exec bash -c 'PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=/build/native-clang uv run --frozen --group native python -m pytest -q tests/reference/test_native_platform_recording.py tests/reference/test_native_runtime_commands.py tests/reference/test_native_runtime_ownership.py'
awk '/failing node IDs, identical in the baseline/,/^## /' docs/TESTING.md | grep -cE 'tests/[^ `]+' || true
```

Expected: 0 failed on both; the remaining macOS baseline count is 0. A node still red here means its task (3.05–3.11) is not done; the stop rule's three end states above are the only exits.

---

## Test-port batches (Tasks 3.12–3.36)

Every batch follows the shared procedure.

**Owner and labels:** T52 = unit/numerical (3.12–3.24), T53 = integration (3.25–3.36). Labels for `bike_add_reference_test`: `t52` or `t53`, plus the batch's area label.

**Sizing:** a batch is one subagent session. Batches above ~60 records were split at planning time (suffixes `a`/`b`/`c`); a range such as "3.12–3.24" includes the suffixed tasks, and "Depends on 3.12" means all of 3.12a–c. Record counts are from `migration/test-port-map.json` (146 files, each in exactly one batch, owners checked). Two single-file batches stay large because a file is not split across sessions: 3.21a (61) and 3.24a (56). If a batch runs out of session, split it by file (or, for those two, by test-function range) at the boundary reached, and record the split in the evidence log.

**Lanes:** 3.12–3.24 split across lanes A/B/C freely, except where "Depends on" says otherwise; 3.25–3.36 likewise. Each lane uses its own worktree path (shared procedure, "Lane path").

| Task | Title (area label) | Files (`R/` = `tests/reference/`, `T/` = `tests/`) — records | Depends on | Notes |
|---|---|---|---|---|
| 3.12a | pneumatic tyre: brush, carcass, geometry (`tyre`) | `R/test_native_pneumatic_brush.py` 27, `R/test_native_pneumatic_carcass.py` 20, `R/test_native_pneumatic_geometry.py` 14 — 61 | 3.01 | Bitwise → mostly golden. `carcass`/`geometry` compare `.tobytes()` directly (`test_native_pneumatic_geometry.py:335-337`, `test_native_pneumatic_carcass.py:748-750`): insert capture sites (shared procedure step 1). |
| 3.12b | pneumatic tyre: model, tire, wire (`tyre`) | `R/test_native_pneumatic_model.py` 19, `R/test_native_pneumatic_tire.py` 13, `R/test_native_pneumatic_wire.py` 17 — 49 | 3.01, 3.03 | `test_native_pneumatic_tire.py` imports `tools.golden_episode`. Its `test_config_projection_carries_the_pneumatic_section` takes 146 s; the C++ port must not re-run the same projection loop more than once. |
| 3.12c | pneumatic tyre config and forces (`tyre`) | `R/test_pneumatic_tire_config.py` 13, `R/test_pneumatic_tire_forces.py` 16 — 29 | 3.01 | |
| 3.13 | tyre and terrain (`terrain`) | `R/test_native_tire.py` 11, `R/test_native_distributed_tire.py` 8, `R/test_native_distributed_stands.py` 2, `R/test_distributed_tire_reference.py` 8, `R/test_tyre_road_profile.py` 2, `R/test_savage_track_surface.py` 1, `R/test_terrain_placement_regression.py` 3, `R/test_alpine_track.py` 30 | 3.01, 3.03 | `test_native_distributed_tire.py` and `test_native_tire.py` import `tools.golden_episode`: port their episode reads with `bike::golden::load_episode` (3.03). `_assert_tree_bitwise` (`test_native_distributed_tire.py:338`) recurses over dict keys: the capture order is the key order (shared procedure step 4). |
| 3.14a | rider contacts and contact math (`rider`) | `R/test_native_rider_contacts.py` 40, `R/test_native_rider_contact_math.py` 15 — 55 | 3.01 | |
| 3.14b | rider forces, attachment wrench, contact safety (`rider`) | `R/test_native_rider_forces.py` 10, `R/test_native_attachment_wrench.py` 39, `R/test_attachment_wrench.py` 8, `R/test_attachment_budget.py` 9, `R/test_contact_safety.py` 10 — 76 | 3.01, 3.03 | `test_native_rider_forces.py` imports `tools.golden_episode`. |
| 3.15 | rider control (`rider`) | `R/test_native_rider_intent.py` 26, `R/test_native_spindle_controller.py` 22, `R/test_crank_split.py` 9, `R/test_leg_loop.py` 7, `R/test_joint_strength.py` 15, `R/test_planar_arms.py` 11, `R/test_closed_form_rider.py` 8, `R/test_coupled_rider_task.py` 5 | 3.05, 3.07 | |
| 3.16 | rider allocation and dynamics (`rider`) | `R/test_rider_allocation.py` 31, `R/test_rider_dynamics.py` 10, `R/test_rider_information_boundary.py` 8, `project/tests/reference/test_constraint_mode_search.py` 6 | 3.05 | The `project/` file's C++ path is `native/tests/reference/project_test_constraint_mode_search.cpp` (plan-08). |
| 3.17 | seated topology and welds (`rider`) | `R/test_seated_climb_surge.py` 4, `R/test_seated_pedaling_cycle.py` 8, `R/test_seated_posture_program.py` 15, `R/test_seated_topology.py` 7, `R/test_pinned_topology.py` 2, `T/test_rider_welds.py` 4, `R/test_welded_profile_config.py` 3, `R/test_articulated_config.py` 7 | 3.06, 3.07 | `T/test_rider_welds.py` → `native/tests/integration/test_rider_welds.cpp` (plan-08), registered with `bike_add_reference_test(rider_welds SOURCES ../integration/test_rider_welds.cpp …)`. |
| 3.18 | drivetrain core (`drive`) | `R/test_native_drivetrain.py` 28, `R/test_native_drive_policies.py` 31, `R/test_drivetrain_policy_clone.py` 1, `R/test_motor_freewheel_topology.py` 9 | 3.01 | |
| 3.19 | drive policies and pedalling (`drive`) | `R/test_motor_profile.py` 8, `R/test_assist_controller.py` 22, `R/test_cadence_shifter.py` 9, `R/test_pedal_waveform.py` 10, `R/test_pedaling_policy.py` 9, `R/test_system_momentum_drive.py` 2 | 3.01 | |
| 3.20 | brakes, cruise, suspension, clocks (`runtime`) | `R/test_native_brake_resistance.py` 5, `R/test_native_static_brake.py` 5, `R/test_native_cruise.py` 24, `R/test_native_suspension.py` 4, `R/test_native_stepper.py` 1, `R/test_native_clock_contracts.py` 16, `R/test_control_clock.py` 4, `R/test_playback_clock.py` 7 | 3.03 | `brake_resistance`, `suspension` and `stepper` read golden episodes. |
| 3.21a | numeric completion (`numerics`) | `R/test_native_numeric_completion.py` 61 | 3.01 | |
| 3.21b | numeric domains, Numba kernels (`numerics`) | `R/test_native_numeric_domains.py` 10, `R/test_numba_kernels.py` 8 — 18 | 3.01 | `test_numba_kernels.py` tests the Numba twins: expect `drop`, with evidence naming the C++ kernel tests that cover the arithmetic. |
| 3.22a | encode columns (`recording`) | `R/test_native_encode_columns.py` 40 | 3.01 | |
| 3.22b | recorder/HUD columns, period buffer (`recording`) | `R/test_recorder_columns.py` 4, `R/test_hud_columns.py` 2, `R/test_period_buffer.py` 10, `R/test_frozen_force_arrays.py` 2, `R/test_ride_plots_html_markers.py` 3 — 21 | 3.08 | |
| 3.23 | monitors, energy, acceptance (`physics`) | `R/test_energy_ledger.py` 14, `R/test_balance_monitor.py` 9, `R/test_monitor_strictness.py` 10, `R/test_reference_monitor.py` 4, `R/test_sensor_noise.py` 8, `R/test_acceptance_interval_capture.py` 2, `R/test_realistic_pedelec_acceptance.py` 4, `R/test_lean_limit.py` 2 | 3.09 | |
| 3.24a | native benchmark (`episode`) | `R/test_native_benchmark.py` 56 | 3.01, 3.02 | May port to `native_bench` assertions (registered in the root graph by Task 3.02 Step 2b). |
| 3.24b | golden episodes, ride CLI (`episode`) | `R/test_golden_episode.py` 2, `R/test_golden_episode_v2.py` 4, `R/test_episode_compare.py` 3, `R/test_divergence_horizon.py` 2, `R/test_ride_backend_cli.py` 11 — 22 | 3.03 | `test_divergence_horizon.py` → `drop` (ADR-0002: no subject without the oracle). The three golden-episode files are covered by `reference.golden_episode_tool` (3.03): triage each record against it before writing new C++. |
| 3.25 | runtime step, commands, ownership, bootstrap (`runtime`) | `R/test_native_runtime_step.py` 10, `R/test_native_runtime_commands.py` 10, `R/test_native_runtime_ownership.py` 10, `R/test_native_runtime_bootstrap.py` 19 | 3.11 | |
| 3.26 | runtime config, accounting, rollout, performance contract (`runtime`) | `R/test_native_runtime_config.py` 17, `R/test_native_runtime_accounting.py` 11, `R/test_native_runtime_rollout.py` 6, `R/test_native_runtime_performance_contract.py` 10 | 3.04 | `performance_contract` uses the realtime measurement (3.04). |
| 3.27 | state bundles (`state`) | `R/test_native_state_bundle.py` 31, `R/test_native_state_fuzz.py` 2, `R/test_native_state_restore.py` 19, `R/test_equilibrium_state_file.py` 9 | 3.03 | `state_restore` reads golden episodes. Fuzz → a seeded C++ property test with the same seed and case count. |
| 3.28 | atomicity and input contracts (`contracts`) | `R/test_native_atomicity.py` 21, `R/test_native_input_contracts.py` 32 | 3.01 | |
| 3.29a | engine errors, loader (`checks`) | `R/test_native_engine_errors.py` 24, `R/test_native_loader.py` 20 — 44 | 1.06–1.09 | `native_loader` → `drop` (Python extension loading). |
| 3.29b | contract checks, check tools (`checks`) | `R/test_native_contract_checks.py` 17, `R/test_native_check_tools.py` 37 — 54 | 1.06–1.09 | `check_tools` / `contract_checks` → `run_case` entries in `migration_T56_check.cmake`; the ~30 schema-2 cases (`test_native_check_tools.py:1869-2414`) port 1:1. |
| 3.30 | native recording lifecycle (`recording`) | `R/test_native_recording_lifecycle.py` 31, `R/test_native_recording_retention.py` 17, `R/test_recording_capture.py` 24, `R/test_recording_publication.py` 2 | 3.11 | |
| 3.31 | recording catalog, CLI, analysis (`recording`) | `R/test_recording_catalog.py` 15, `R/test_recording_cli.py` 12, `R/test_recording_report.py` 1, `R/test_recording_storage.py` 11, `R/test_recording_values.py` 5, `R/test_recording_analysis.py` 16, `R/test_recording_analyst_cli.py` 6 | 3.01 | |
| 3.32 | replay and view (`replay`) | `R/test_native_replay_recording_v3.py` 1, `R/test_native_replay_session.py` 2, `R/test_replay_manifest_v2.py` 9, `R/test_replay_recording_v3.py` 6, `R/test_replay_session.py` 6, `R/test_pedelec_replay_contract.py` 5, `T/test_replay_playback.py` 5, `R/test_native_viewer_snapshot.py` 2, `R/test_native_view_contracts.py` 9 | 3.01 | `T/test_replay_playback.py` → `native/tests/integration/test_replay_playback.cpp` (plan-08). |
| 3.33a | native research (`research`) | `R/test_native_research_capture.py` 2, `_environment.py` 14, `_failures.py` 1, `_frontend.py` 1, `_policy.py` 7, `_programs.py` 3, `_recording.py` 7, `_sensors.py` 6 — 41 | 3.01 | The `_…` entries are `R/test_native_research…`. |
| 3.33b | research capture, frontend, lifecycle, sweep (`research`) | `R/test_research_capture.py` 7, `R/test_research_frontend.py` 1, `R/test_research_recording_lifecycle.py` 7, `R/test_research_sweep.py` 3 — 18 | 3.01 | `research_sweep` ports against `bike-research-sweep`. |
| 3.34 | frontends, realtime, async, batch (`frontend`) | `R/test_frontend_windows.py` 6, `R/test_native_ride_frontend.py` 2, `R/test_physical_view.py` 4, `R/test_pinned_measurement.py` 3, `R/test_pneumatic_physical_runtime.py` 3, `R/test_realtime_deferred_checks.py` 3, `R/test_realtime_gate.py` 1, `R/test_realtime_measurement.py` 7, `R/test_measure_realtime_report.py` 6, `R/test_runtime_single_path.py` 6, `T/test_async_close.py` 4, `T/test_batch.py` 8 | 3.04, 3.10 | `T/test_async_close.py` → `native/tests/integration/test_async_close.cpp`; `T/test_batch.py` → `…/test_batch.cpp` (plan-08). |
| 3.35 | platform files (`platform`) | `R/test_native_linux_headless.py`, `R/test_native_numeric_portability.py`, `R/test_native_platform_blas.py`, `R/test_native_platform_linkage.py`, `R/test_native_platform_recording.py`, `R/test_native_platform_replay.py`, `T/test_async_recovery.py`, `T/test_execution_identity.py`, `T/test_recording_platform_compatibility.py`, `R/test_golden_hook.py` (counts from 3.02) | 3.02, 3.11 | Linkage tests port to `cmake -P` scripts that inspect `otool -L` / `readelf -d` of installed binaries (2.08). `test_native_linux_headless.py` is Linux-only: its golden records list one platform (`golden_platforms`). `test_golden_hook.py` → `drop` (tests the Python capture hook; the C++ readers are covered by `reference.golden_support`). `test_native_numeric_portability.py:157` compares `.view(np.uint64)`: insert capture sites. |
| 3.36 | portability tooling tests (`portability`) | 24 `tests/portability/test_*.py` (counts from 3.02) | 3.02, 3.41–3.45 | Port `test_cmake_platform`, `test_compiler_policy`, `test_dependency_policy`, `test_provider_identity` and `test_root_configure` to `cmake -P` CTest scripts (scratch configures, as `migration_T03` does). Tests of tools that D18 deletes (`dev`, `ci_lanes`, `offline_*`, `qualification_report`, `release_gate`, `baseline`, `native_dependencies`, `numerics_discovery`, `native_runtime`, `native_toolchain`, `native_coverage_tools`, `install_modes`) → `drop`; the evidence names the replacing preset/CTest/CPack test from 3.41–3.45. |

### CHECKPOINTs 3-C, 3-D, 3-E, 3-F (L2)

- [ ] After 3.17, 3.23, 3.29b and 3.36 (counting merged-back lane work, including every suffixed task numbered below the checkpoint), run the full L2 block plus:

```bash
ctest --test-dir build/native-release -L reference -j8 --no-tests=error --output-on-failure
tools/linux/container.sh ctest root-clang -L reference -j8 --no-tests=error
```

Append the evidence and commit.

---

## Validation rigs (T54: 30 Python files in `src/bike_sim/validation/` + 2 in `src/bike_sim/physics/`)

These tasks follow plan-08 §T54. The target files are:
- `native/src/validation/{rigs,tire,rider,drivetrain,acceptance,evidence}.cpp`;
- `native/src/validation/acceptance.hpp`;
- `native/src/tools/validate_main.cpp` (`bike-validate`);
- `native/tests/migration/test_t54.cpp`, registered as `migration_T54`.

Method:
1. Each task ports its Python modules line by line, with line references in comments.
2. It captures goldens for each module's public outputs by running the module's existing entry points under `BIKE_GOLDEN_CAPTURE`. Where a module has no `assert_bitwise_equal` call, add an explicit `_golden.record(result, "<output name>")` (or `kind="close"` with the tolerance the module's own check uses) at the module's pytest/driver call site.
3. It adds `migration_T54` cases.
4. It sets each `python-removal.json` row to `replacement_landed`.

### Task 3.37: Tyre calibration, curve and characterization

- Files: `src/bike_sim/physics/tire_calibration.py`, `src/bike_sim/physics/tire_curve.py`, `src/bike_sim/validation/tire_characterization.py`, `src/bike_sim/validation/datasets.py`.
- C++ target: `native/src/validation/tire.cpp`.
- L1: `ctest -R '^migration_T54$'` on both platforms.

### Task 3.38: Contact and load rigs

- Files: `src/bike_sim/validation/rigs.py`, `contact_acceptance.py`, `contact_manifold_rigs.py`, `contact_resolution.py`, `load_transfer.py`, `grade_feasibility.py`, `drive_suspension_rig.py`, `environment.py`.
- C++ target: `native/src/validation/rigs.cpp`.

### Task 3.39: Plant and rider rigs

- Files: `src/bike_sim/validation/plant_prefix.py`, `plant_release.py`, `plant_torque_rig.py`, `rider_cases.py`, `rider_factor_matrix.py`, `rider_replay.py`, `rider_work.py`, `system_momentum.py`, `ride_cases.py`.
- C++ targets: `native/src/validation/rider.cpp`, `native/src/validation/drivetrain.cpp`.

### Task 3.40: Benchmarks, acceptance, evidence and `bike-validate`

- Files: `src/bike_sim/validation/__init__.py`, `antiwheelie_bench.py`, `antiwheelie_reports.py`, `benchmarks.py`, `climbing_benchmark.py`, `climbing_evidence.py`, `experiment_acceptance.py`, `fidelity_sweep.py`, `reference_release.py`.
- C++ targets: `native/src/validation/acceptance.cpp`, `evidence.cpp`, and `native/src/tools/validate_main.cpp`. The CLI mirrors the Python entry points' arguments; list them from each module's `main`/`argparse`.
- Reports keep physical validity, parity and performance as separate fields (plan-08 T54 mandatory case).

### CHECKPOINT 3-G (L2)

---

## Tooling replacement (D18)

### Task 3.41: Root `CMakePresets.json` (replaces `tools/dev.py` configure/build/test)

**Files:**
- Create: `CMakePresets.json` (repository root)
- Modify: `CMakeLists.txt` (root): `cmake_minimum_required(VERSION 3.27)` (from 3.24), matching the presets' `cmakeMinimumRequired`; host CMake 4.4.3 and container CMake 3.28.3 both satisfy it, and preset schema 6 needs ≥ 3.25
- Modify: `docs/development/macos.md`, `docs/development/linux.md` (commands)

**Interfaces:**
- Produces:
  - configure presets `release` (Release, GUI OFF), `debug`, `gui` (Release, GUI ON), `asan` and `rtsan` (bodies from 3.42), `coverage` (body from 3.43). All use `binaryDir` = `${sourceDir}/build/<preset>`, except `release`, which keeps `${sourceDir}/build/native-release` so existing scripts keep working.
  - every configure preset sets `BIKE_BUILD_PYTHON_BINDINGS=OFF` and `BIKE_DEPENDENCY_ROOT=${sourceDir}/.deps`;
  - the compiler is chosen per host OS with hidden base presets and `condition` blocks (`{"type": "equals", "lhs": "${hostSystemName}", "rhs": "Linux"}` → `CMAKE_CXX_COMPILER=/usr/bin/clang++-18`, matching Phase 2's `root-clang`; `Darwin` → the Xcode toolchain default). The `debug` preset on Linux uses `/usr/bin/g++`, so the GCC lane of the checklist (3.45) has a preset. Without this, the container's default `c++` (GCC 13) would silently build `release`;
  - matching build and test presets (`test` presets set `"output": {"outputOnFailure": true}`, `"execution": {"jobs": 8, "noTestsAction": "error"}`); the `release` test preset also sets `"filter": {"exclude": {"label": "slow"}}`, because a workflow preset cannot take `-LE slow` from the command line (Task 4.02 runs `-L slow` separately);
  - workflow presets `release` (configure → build → test) and `gui`.

- [ ] **Step 1: Failing check**

```bash
cmake --workflow --preset release; echo "exit=$?"
```

Expected: `exit≠0` (no presets file).

- [ ] **Step 2: Write `CMakePresets.json`** (version 6, `cmakeMinimumRequired` 3.27) and bump the root `cmake_minimum_required`. For `rtsan` on macOS, the compiler path is the one `native/CMakePresets.json` uses (`/opt/homebrew/opt/llvm/bin/clang++`). On Linux, use `clang++-19` or newer if present; otherwise the preset fails configure with the RTSan capability probe (`BIKE_CAP_RTSAN_SUPPORTED`), which is the honest result on Clang 18.
- [ ] **Step 3: Verify**

```bash
cmake --workflow --preset release
tools/linux/container.sh sync && tools/linux/container.sh exec cmake --workflow --preset release
tools/linux/container.sh exec grep -E '^CMAKE_CXX_COMPILER:' /src/build/native-release/CMakeCache.txt
```

Expected: exit 0 on both (CTest runs inside); the container cache names `/usr/bin/clang++-18`. The container's `build/native-release` lives under `/src/build`; `container.sh sync` must preserve `/src/build` between runs (Task 1.02), otherwise every preset run is a cold build.
- [ ] **Step 4: Docs.** Replace every `tools/dev.py configure|build|doctor` command block in `docs/development/{macos,linux,troubleshooting,install-modes}.md` with the preset equivalent. Leave `doctor` as prose pointing at the configure-time probes. Commit `build: root CMakePresets replace tools/dev.py`.

### Task 3.42: Sanitizer and RTSan lanes for the root graph (replaces `tools/native_runtime.py` preload logic)

**Facts checked at planning time:** `BikeCompilerPolicy.cmake` (in the root graph since Task 2.02) already owns the instrumentation options `NATIVE_SANITIZE`, `NATIVE_RTSAN`, `NATIVE_COVERAGE` (`:52-54`, mutually exclusive `:61`), their flags (`:596-672`) and the capability probes (`BIKE_CAP_*_SUPPORTED`). This task adds **no** new options and no flags. The controls take a mode argument (`sanitizer_control.cpp:70` `use-after-free`; `rtsan_control.cpp:58-63` `forbidden-allocation`); today CTest runs only their clean mode, and only in the binding graph (`native/CMakeLists.txt:521-525`).

**Files:**
- Create: `native/cmake/Sanitizers.cmake`: registers the controls in the root graph (the executables from `native/tests/sanitizer_control.cpp` / `rtsan_control.cpp`, built as in `native/CMakeLists.txt`).
- Modify: `CMakeLists.txt` (include it); `CMakePresets.json` (`asan` = `NATIVE_SANITIZE=ON`, `rtsan` = `NATIVE_RTSAN=ON` bodies)
- Modify: `migration/ctest-graph-map.json` (Task 3.02 entries for `native_sanitizer_control` / `native_rtsan_control` → landed)
- Test: four CTest nodes:
  - `sanitizer_clean_control`, `rtsan_clean_control`: the clean mode, always registered (successors of the binding graph's `native_sanitizer_control` / `native_rtsan_control`);
  - `sanitizer_negative_control` (only when `NATIVE_SANITIZE`): `COMMAND <control> use-after-free`, `PASS_REGULAR_EXPRESSION "ERROR: AddressSanitizer: heap-use-after-free"`, **no** `WILL_FAIL` (`PASS_REGULAR_EXPRESSION` already replaces the exit-code criterion; adding `WILL_FAIL` would invert it and pass only when the report is absent);
  - `rtsan_negative_control` (only when `NATIVE_RTSAN`): `COMMAND <control> forbidden-allocation`, `PASS_REGULAR_EXPRESSION "ERROR: RealtimeSanitizer"`, no `WILL_FAIL`.

**Interfaces:** native executables link their sanitizer runtime themselves. Nothing is preloaded, because there is no interpreter.

- [ ] Write the control registrations first and prove the negative control can fail: configure the `release` preset, run `ctest -R negative_control -N` (expected: not registered), then the `asan` preset (expected: registered and passing because the report appears). Temporarily point `sanitizer_negative_control` at the clean mode and confirm it fails; revert.
- [ ] Run `cmake --workflow --preset asan` on macOS and Linux; expect the control to detect the fault on both. Do the same for `rtsan` where supported, and record "unsupported on Clang 18" for Linux if the probe says so.
- [ ] Commit `build: root-graph ASan/UBSan/RTSan presets with negative controls`.

### Task 3.43: Coverage target (replaces `tools/native_coverage.py` / `.sh`)

**Facts checked at planning time:** the instrumentation flags for `NATIVE_COVERAGE` already exist (`BikeCompilerPolicy.cmake:653-672`). Its tool qualification still runs Python: `:805-816` requires a `python3`, and `:818-830` runs `tools/native_toolchain.py coverage-tools` to resolve and version-check `llvm-profdata`/`llvm-cov`.

**Files:**
- Create: `native/cmake/Coverage.cmake`: with `NATIVE_COVERAGE=ON` (flags come from the policy, not from this file) it adds a target `coverage`. That target runs `ctest` with `LLVM_PROFILE_FILE=${CMAKE_BINARY_DIR}/profiles/%p-%m.profraw`, then `llvm-profdata merge` and `llvm-cov export -format=lcov` into `build/coverage/coverage.lcov`, plus a text summary.
- Modify: `native/cmake/BikeCompilerPolicy.cmake:805-850`: replace the `python3` requirement and the `native_toolchain.py coverage-tools` call with CMake: `find_program` for `llvm-profdata`/`llvm-cov` beside `CMAKE_CXX_COMPILER` first, then `xcrun --find` on Darwin (port the candidate order of `tools/native_toolchain.py:162,201`, `sibling_candidates`/`darwin_tool_candidates`), `--version` must report the compiler's LLVM major, and the instrumented control compile/run/merge/export probe stays. Keep the produced variables (`BIKE_CAP_COVERAGE_TOOLS`, `BIKE_CAP_COVERAGE_FORMAT_PROBE`, the `llvm_profdata`/`llvm_cov` paths) identical, because the check context records them.
- Modify: `CMakePresets.json` (`coverage` preset = `NATIVE_COVERAGE=ON`); `.github/workflows/portability.yml` (later, in 3.45)
- Test: CTest `coverage_smoke` (coverage builds only). It asserts that `coverage.lcov` exists, is non-empty and names at least one file under `native/src/`.

- [ ] Write the test, run `cmake --workflow --preset coverage` (macOS, then Linux), and confirm the configure log contains no `python` invocation (`grep -i python build/coverage/CMakeFiles/CMakeConfigureLog.yaml` finds no executed command). Run `cmake --build native/build --target check_context` too: the binding graph includes the same policy file until Task 4.06. Commit `build: llvm coverage target replaces native_coverage tooling`.

### Task 3.44: CPack distributions (replaces the `offline_*` tools, D20)

**Files:**
- Create: `packaging/CMakeLists.txt` (included from the root at the end)
- Create: `native/tests/package/package_smoke.cmake`
- Modify: `packaging/OFFLINE_README_full.md`, `packaging/OFFLINE_README_slim.md` → merge into `docs/installation.md` (plan-08 T59 path); delete the two README files and `packaging/install_full.sh`, `packaging/install_slim.sh` in Task 4.05.

**Interfaces:**
- Produces:
  - `cpack --config build/native-release/CPackConfig.cmake -G TGZ` → `bike-sim-<version>-<platform-key>.tar.gz`, containing exactly the `cmake --install` tree;
  - `cpack --config build/native-release/CPackSourceConfig.cmake -G TGZ` → `bike-sim-<version>-source.tar.gz`: the source tree **plus** the dependency archives the lock lists for the current platform, under `.deps/_archives/`, so an offline host can rebuild with `cmake --workflow --preset release`.

  CPack's source generator packs the whole source directory minus `CPACK_SOURCE_IGNORE_FILES`; it cannot filter `.deps/_archives` by lock entry, and `.deps/` holds extracted trees plus archives of both platforms. So at configure time `packaging/CMakeLists.txt` copies exactly the archives the lock selects for `BIKE_TARGET_SYSTEM` (the `platforms.<key>.acquisition.archive` entries, Task 2.04) into `${CMAKE_BINARY_DIR}/source-deps/_archives`, and sets `CPACK_SOURCE_INSTALLED_DIRECTORIES "${CMAKE_SOURCE_DIR};/;${CMAKE_BINARY_DIR}/source-deps;/.deps"`.

  Version = the root `project(VERSION …)`. If the root project has no version, add `VERSION 0.0.0` in this task and document that bumping it is a release decision.

**Depends on:** 2.08. The canned ride inputs reach the prefix through the existing `install(DIRECTORY …/examples/ DESTINATION share/bike_sim/examples …)` in `native/cmake/NativeStartup.cmake:146-148`, which installs `*.toml`, `*.json` and `*.npz`. 2.08's `install_smoke` asserts that they exist, and `package_smoke` step 4 needs them.

- [ ] **Step 1: Failing smoke** (`package_smoke.cmake`):
  1. run `cpack` in `BUILD_DIR`;
  2. untar the binary package into a scratch dir;
  3. run every `bin/bike-*` with `--help` under a scrubbed loader environment (reuse the `install_smoke` loop);
  4. run one canned headless ride: `bin/bike-ride --backend native --physics-config share/bike_sim/examples/research/viewer_physics_welded.toml --track share/bike_sim/examples/research/rough_uphill_savage.toml --headless --duration 1 --no-plots`, and accept the documented diagnostic exit as in Task 2.10;
  5. run `bike-python-audit --mode package --root <unpacked>`, expecting exit 0.

  Register it as CTest `package_smoke` with label `package`.

- [ ] **Step 2: Implement** `packaging/CMakeLists.txt`: `CPACK_GENERATOR TGZ`, `CPACK_SOURCE_GENERATOR TGZ`, the configure-time archive staging above, and `CPACK_SOURCE_IGNORE_FILES` with `/\\.deps/`, `/build`, `/\\.venv/`, `/\\.git/`, `/output/`, `/\\.worktrees/`, `/native/build`, `/\\.scratch/`, `/\\.work/`. Add a smoke assertion that the source tarball lists exactly the staged archive names under `.deps/_archives/` and nothing else under `.deps/` (`tar tzf … | grep '/.deps/'`). `bike-python-audit --root <dir> --mode package` is the exact CLI (`python_audit_main.cpp:592`).
- [ ] **Step 3: Verify** on macOS and Linux (`ctest -R '^package_smoke$'`), plus an offline rebuild rehearsal in a fresh container started with `--network none`:

```bash
docker run --rm --network none --platform linux/arm64 -v /tmp/pkg:/pkg bike-dev-linux:24.04 bash -c 'cd /tmp && tar xf /pkg/bike-sim-*-source.tar.gz && cd bike-sim-*-source && mkdir -p .deps && <extract archives per native/dependencies.md> && cmake --workflow --preset release'
```

Replace `<extract archives per native/dependencies.md>` with the exact extraction commands Task 2.05 codified in the Dockerfile's acquire stage. Factor them into `tools/linux/extract_deps.sh` (shell, no network) in this task, so the Dockerfile and this rehearsal call the same script.

- [ ] **Step 4: Commit** `build: CPack binary and offline source distributions`.

### Task 3.45: CI on presets and the release checklist (replaces `ci_portability.py`, `portability_qualification.py`)

**Files:**
- Modify: `.github/workflows/portability.yml`. Every lane runs `cmake --workflow --preset <preset>` followed by `tools/run_native_checks.sh build/<preset>`, and uploads `build/<preset>/Testing/`, `native_check_summaries/` and the CPack outputs as artifacts. Keep the SHA-pinned actions and the trusted-lane gating already in the file.
- Create: `docs/releases/python-free-qualification.md`. A Markdown checklist: one row per required lane with columns (lane, host, preset, command, CI run link or local evidence section, exit code, date, commit).
  - Required lanes: `macos-arm64-release`, `macos-arm64-asan`, `macos-arm64-coverage`, `linux-aarch64-release` (clang), `linux-aarch64-debug` (gcc), `linux-aarch64-asan`, `linux-aarch64-coverage`, `package-macos`, `package-linux`, `offline-rebuild-linux`, `macos-manual-gui` (user), `linux-xvfb-smoke` (not acceptance).
  - Not qualified / not advertised (D10): x86_64 Linux, Linux desktop GUI acceptance, RTSan where the probe says unsupported.
- Modify: `docs/development/toolchain-matrix.json`. Convert to a static table or delete it if nothing reads it after `portability_qualification.py` goes; `grep -rn toolchain-matrix` decides. Record the outcome in the commit body. The file has no `machines` key (its keys are `platforms`, `lanes`, `interpreters`, …), so the D-record wording "reorder `machines`" maps to: x86_64 Linux is absent from the required-lane list of the checklist above and is marked "not qualified" in whatever form the matrix survives. Note this deviation in the commit body.

- [ ] Verify the workflow syntax with `actionlint` if available (`brew list actionlint` / `which actionlint`). Otherwise do a YAML parse: `uv run --no-project --with pyyaml python -c "import yaml;yaml.safe_load(open('.github/workflows/portability.yml'))"`.
- [ ] Commit `ci: lanes run presets; release qualification is a checklist`.

### CHECKPOINT 3-H (L2): Phase 3 closes

- [ ] Run the full L2 block, plus:

```bash
cmake --workflow --preset release
ctest --test-dir build/native-release -L 'reference|package' -j8 --no-tests=error --output-on-failure
tools/linux/container.sh exec cmake --workflow --preset release
```

- [ ] Append the evidence. Push `impl/python-free-cpp`.

---

# Phase 4: Qualify, then delete Python

### Task 4.01: Golden completeness and ledger reconciliation

**Files:**
- Create: `native/tests/golden/completeness.cmake` (CTest `golden_completeness`, registered from `native/tests/reference/CMakeLists.txt` with label `audit` only, so it does not run in every `native` CTest pass)
- Modify: `migration/test-port-map.json`, `migration/python-removal.json`
- Modify: `migration/task-graph.json`. Statuses: T16–T51 → `verified` where their migration tests and, if any, ported reference tests are green; T52–T60 → `verified`.

**Interfaces:**
- Produces:
  - `golden_completeness` fails if any of these holds:
    1. a `golden` record's node directory is missing under any platform named in that record's `golden_platforms` list (written by shared-procedure step 1), or exists under a platform the list does not name. A record whose `golden_platforms` has one entry (e.g. `test_native_linux_headless.py`, skip-guarded platform nodes) must carry an `evidence` sentence saying why;
    2. a directory's `NNN.json` indices are not contiguous from `000`;
    3. any `test-port-map.json` record has `disposition` outside `{port, golden, drop}`, or `comparison_class` outside the enum defined in shared-procedure step 1. All 1611 records start as `"disposition": "planned"` with `"comparison_class": "source-specific; assigned by T01 before implementation"`, so both fields must have been rewritten by a Phase 3 batch task;
    4. any `drop` record has an empty `evidence`.
  - Every `python-removal.json` row ends in `replacement_landed`, `delete_with_python`, `replace_with_cmake_ctest_cpack` or `replace_with_ci_checklist`. The `remove_or_relocate_pending_OD-1` rows are resolved by OD-1 before this task closes. If the user has not been asked yet, ask in Step 3; with no answer, apply the D-record default (delete, provenance in `docs/reference/README-provenance.md`) and record that.

**Review focus:** the 46 rows that lacked `implementation` records for already-verified tasks (T03, T05, T09, T11–T15, T19, e.g. `src/bike_sim/geometry/*.py`, `src/bike_sim/physics/*.py`) get a `replacement_paths` entry pointing at their C++ module. Find it in `migration/file-work-units.json` or `migration/api-index.md`. None is waved through without one.

- [ ] **Step 1: Write `completeness.cmake`** implementing rules 1–4 with `file(GLOB …)` and `string(JSON …)`. Read `migration/test-port-map.json` (≈1.2 MB) via `file(READ)`. Each `string(JSON GET)` re-parses the document, so 1611 × ≥4 GETs cost minutes. That is acceptable only because the test carries the `audit` label alone and runs at checkpoints, not in every `native` pass. If one run exceeds 5 minutes, move the rules into `bike-native-checks --kind golden-completeness` (C++ JSON is already linked there) and keep the CTest name. Register it:

```cmake
add_test(NAME golden_completeness COMMAND "${CMAKE_COMMAND}"
  -D "REPO_ROOT=${CMAKE_SOURCE_DIR}" -P "${CMAKE_CURRENT_SOURCE_DIR}/../golden/completeness.cmake")
set_tests_properties(golden_completeness PROPERTIES LABELS "audit")
```

- [ ] **Step 2: Run, fix gaps, re-run** until green:

```bash
ctest --test-dir build/native-release -R '^golden_completeness$' --output-on-failure
```

Every gap is fixed in its owning batch's files (capture the missing platform, triage the missing record). Never edit a rule to make it pass.

- [ ] **Step 3: Reconcile `python-removal.json`** as described above. Then:

```bash
uv run --no-project python -c "
import json,collections;r=json.load(open('migration/python-removal.json'))
print(collections.Counter(x['status'] for x in r))"
comm -13 <(jq -r '.[].path' migration/python-removal.json | sort -u) <(git ls-files '*.py' | sort)
```

Expected:
- no `planned` and no `remove_or_relocate_pending_OD-1` (today all 459 rows are `planned`; the ledger is a top-level array);
- the `comm` output is empty: every tracked `.py` has a ledger row. Task 3.02 adds most of the missing rows, but some paths are still absent at review time: `src/bike_sim/native/{execution_identity,provider_build_id}.py`, the `tests/reference` helpers (`conftest.py`, `_bits.py`, `_golden.py`, `_encode_oracle.py`, `_legacy_sensor_oracle.py`, `_native_runtime_support.py`, `_physical_hud_oracle.py`, `_recording_support.py`, `native_loader.py`, `native_safety_helpers.py`, `native_test_reporter.py`) and `tests/portability/{interpreter_shim,offline_install_probe,worker_entry_probe}.py`. Add a `delete_with_python` row for each one still missing.

- [ ] **Step 4: Commit** `docs(migration): golden completeness gate and reconciled removal ledger`.

---

### Task 4.02: `tools/run_tests.sh` on the native graph only

**Files:**
- Modify: `tools/run_tests.sh` (rewrite; keep the profile names `quick`, `native`, `full` and the usage text style)
- Modify: `native/tests/reference/CMakeLists.txt`, `native/tests/migration/CMakeLists.txt`. Add the label `slow` to tests whose measured duration exceeds 30 s (`ctest --test-dir build/native-release -j1 --timeout 600 | sort -k5 -n` lists durations).
- Modify: `CMakePresets.json` (Task 3.41). The `release` test preset gets `"filter": {"exclude": {"label": "slow"}}`, because a workflow preset cannot take `-LE slow` from argv.

**Interfaces:**
- Produces: `tools/run_tests.sh <profile> [preset]`. `preset` defaults to `release`, and `<build>` = the preset's `binaryDir` (`build/<preset>`, Task 3.41). `native asan` therefore runs the same stages on `build/asan`. This replaces `NATIVE_TEST_BUILD_PATH`/`NATIVE_TEST_BUILD_DIR=asan|rtsan|coverage` (old script lines 18-19, 101-112). Sanitizer runtimes now come from the preset's own link flags, not a preload. The profiles:
  - `quick` = `cmake --workflow --preset <preset>` (its test preset already excludes `slow`);
  - `native` = `cmake --preset <preset> && cmake --build --preset <preset>`, then `tools/run_native_checks.sh <build>`, then `ctest --test-dir <build> -LE slow -j8 --output-junit Testing/junit.xml`;
  - `full` = `native` + `ctest --test-dir <build> -L slow --no-tests=error` + `ctest --test-dir <build> -L package --no-tests=error`. `--no-tests=error` stops an unlabelled tree from passing silently: no `slow` or `package` label exists today.

  No `uv`, `python` or `pytest` token remains in the script.
- Provenance: the old `NATIVE_TEST_PROVENANCE_PATH` / `native_test_reporter` output (old script lines 118-119) is replaced by CTest's `<build>/Testing/` tree plus the JUnit file. The CI artifacts of Task 3.45 upload both. Record this mapping in the header comment.

**Review focus:**
- Every previous gate stage has a successor: build, build selection (preset argument), every sweep, the contracts kind, CTest, provenance (JUnit + `Testing/`).
- The pytest stage's successor is the `reference` label set. Record the mapping in the script header comment.

- [ ] **Step 1: Failing audit check**

```bash
grep -nE 'python|pytest|uv run|uv sync' tools/run_tests.sh; echo "hits=$?"
```

Expected: hits (exit 0).

- [ ] **Step 2: Rewrite the script.** Keep `set -euo pipefail` and the `usage()` style. Keep the Darwin-only malloc debugging exports (`MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1`, old lines 63-69 and 81-84; never on Linux). They wrap only the non-`slow` CTest invocation of every profile: run that `ctest -LE slow` in a subshell that exports them. They are never set around `ctest -L slow`, because the `slow` set holds the realtime measurements (Task 3.10) and the old script kept them off `full` for that reason.
- [ ] **Step 3: Verify**

```bash
grep -nE 'python|pytest|uv run|uv sync' tools/run_tests.sh; echo "hits=$?"
shellcheck tools/run_tests.sh
bash tools/run_tests.sh quick
ctest --test-dir build/native-release -N -L slow | tail -1
```

Expected: `hits=1`; shellcheck clean; `quick` exits 0; the `slow` set is non-empty (`Total Tests: N` with N > 0).

- [ ] **Step 4: Update `AGENTS.md` "Verification"**:
  - profiles describe the C++ gate;
  - `check_<name>` reruns become `build/native-release/bin/bike-native-checks --kind <kind> --build build/native-release` (and `cmake --build build/native-release --target check_<name>` where Checks.cmake defines one);
  - remove the Numba paragraph, keeping a one-line note "removed with Python, see ADR-0003".

  Commit `build: run_tests.sh drives the native graph only`.

---

### Task 4.03: Release qualification run (T61, simplified per D18)

**Files:**
- Modify: `docs/releases/python-free-qualification.md` (fill every row)
- Modify: `migration/cpp-linux-completion-evidence.md`

**Interfaces:**
- Consumes: Tasks 3.41–3.45 presets, CI and checklist.
- Produces: a filled checklist. Every required row has an exit code, date, commit and evidence link.

**Review focus:** no row is marked done without a command, an exit code and evidence. "Not qualified" rows say so explicitly.

- [ ] **Step 1: L3 on both platforms with the new gate** (one at a time):

```bash
bash tools/run_tests.sh full 2>&1 | tee /tmp/l3-p4-macos.log; echo "exit=${PIPESTATUS[0]}"
tools/linux/container.sh sync && tools/linux/container.sh exec bash tools/run_tests.sh full 2>&1 | tee /tmp/l3-p4-linux.log; echo "exit=${PIPESTATUS[0]}"
```

Expected: both exit 0. Python-era baselines no longer apply: the ported tests must all be green. No red node reaches this task. Each physics-stop-rule node from Tasks 3.05–3.10 must already have ended in one of three ways:
- an approved fix;
- an approved threshold change, with the test edited and the written argument in the evidence log;
- `drop` with evidence and no C++ node.

A node still waiting for a decision blocks 4.03: stop and ask the user.

- [ ] **Step 2: Lanes:** ASan, coverage, package and offline rehearsal on both platforms, per the checklist rows. Use the presets and commands from Tasks 3.42–3.44.
- [ ] **Step 3: macOS manual GUI (user).** Stop and ask the user to run the macOS rows M-PH, M-RS, M-CR, M-PB, M-PG of `docs/development/viewer-platform-acceptance.md` with the native binaries from `cmake --workflow --preset gui`. Give them the exact argv per row from that document. Record the user's results verbatim. Do not mark the row from agent-side evidence.
- [ ] **Step 4: Commit** `docs(release): python-free qualification evidence`.

---

### Task 4.04: No-Python enforcement (T62)

**Files:**
- Modify: `native/cmake/Checks.cmake`. Register CTest `python_audit_source` (`bike-python-audit --mode source --root ${CMAKE_SOURCE_DIR}`) with `WILL_FAIL TRUE` and label `audit`, and CTest `python_audit_documentation` (`--mode documentation`) the same way. Task 4.06 removes `WILL_FAIL`.
- Create: `native/tests/package/audit_negative_controls.cmake` (plan-09 T62 path). It copies an unpacked package into a scratch dir and injects, one at a time: a `.py` file, a `#!/usr/bin/env python3` script, a `site-packages/` directory, and a Python-linked binary. Each injection must make `--mode package` exit non-zero.
  - The Python-linked binary must not need a real CPython, because 4.06 reruns `-L audit` in a container rebuilt without one. The control builds a stub with `execute_process(${CMAKE_CXX_COMPILER} …)`:
    - Linux: `-shared -Wl,-soname,libpython3.13.so.1.0 -o libpython3.13.so.1.0` from a one-function TU;
    - Darwin: `-dynamiclib -install_name /Library/Frameworks/Python.framework/Versions/3.13/Python`.
  - It then links a two-line executable against the stub and copies both into `bin/`/`lib/`. The audit detects this through `readelf -d` / `otool -L` (`python_audit_main.cpp:468-477`) against `package_mode.mach_o_forbidden_loads` (`Python.framework`, `libpython`, …).

**Interfaces:**
- Produces: the package audit is mandatory in `package_smoke` (Task 3.44) and has negative controls. The source/documentation audits are measured failures until Task 4.06.

**Review focus:**
- Negative controls prove detection, not just a clean pass.
- The source audit's failure list equals the remaining Python paths. Record that list in the evidence.

- [ ] Write the controls, run `ctest -R 'python_audit|audit_negative' --output-on-failure` on both platforms, record the source-mode stderr list, commit `test(cpp): T62 python audit gates and negative controls`.

---

### CHECKPOINT 4-A (L2, both platforms): before deletion

Run after 4.01–4.04 and before any deletion (one platform at a time):

```bash
bash tools/run_tests.sh native 2>&1 | tail -20; echo "exit=${PIPESTATUS[0]}"
tools/linux/container.sh sync && tools/linux/container.sh exec bash tools/run_tests.sh native 2>&1 | tail -20
ctest --test-dir build/native-release -L audit --output-on-failure
```

Expected:
- both `native` runs exit 0;
- under `-L audit`, `golden_completeness` and `audit_negative_controls` pass;
- `python_audit_source`/`python_audit_documentation` "pass" by failing as expected (`WILL_FAIL`).

Record the evidence. A red result stops Phase 4 here: nothing has been deleted yet, so the fix is cheap.

---

### Task 4.05: Delete Python sources, tests, tools and packaging metadata (T63 part 1)

**Before starting:** OD-1 (`tools/research/**/*.py`) must already be resolved. 4.01 cannot close while `remove_or_relocate_pending_OD-1` rows remain, so ask the user no later than 4.01 Step 3, or earlier where the Phase 3 tasks schedule it. If the user never answered, the default was applied there; repeat that in the commit body.

**Files (delete):**
- Python packages and tests:
  - `src/bike_sim/` (entire tree);
  - every `*.py` under `tests/`, with `tests/fixtures/` data **kept**;
  - `conftest.py`, `project/tests/reference/*.py`.
- Python tools:
  - `tools/*.py`, `tools/migration/*.py`, `tools/proto_native_bench/*.py`. Keep `tools/proto_native_bench/{bench.cpp,state_format.hpp,artifacts/}`: the root graph compiles `native_bench` from them (`CMakeLists.txt:561-569`), and `Checks.cmake:82` names the directory as a first-party root. Its `README.md` is rewritten in 4.07;
  - `tools/research/**/*.py` (per OD-1).
- Shell wrappers whose only job was Python:
  - `tools/install_offline.sh`, `tools/make_offline_bundle.sh`, `tools/native_coverage.sh`;
  - `packaging/install_full.sh`, `packaging/install_slim.sh`, `packaging/OFFLINE_README_*.md` (content moved to `docs/installation.md` in 3.44).
- Project metadata: `pyproject.toml`, `uv.lock`, `.python-version`.

**Files (modify):**
- `tools/native_common.sh`, `tools/run_native_checks.sh`, `tools/check_scripts.sh`: remove interpreter invocations and `PYTHON_EXECUTABLE` usage strings. Find them with `grep -nwE 'python3?|uv|uvx|pip3?|pytest|mjpython|PYTHON_EXECUTABLE' tools/*.sh tools/linux/*.sh`, which matches the policy's `invocation_tokens`. Leave the `bike-python-audit` / `BIKE_PYTHON_AUDIT` names alone: they name the C++ audit, not an interpreter. The five binding-graph wrappers `tools/check_{odr,cppcheck,analyzer,native_frontends,native_tidy}.sh` are deleted in 4.06, not edited here.
- `.gitignore`: keep `.venv/`, `__pycache__/` and `*.pyc`, under a comment `# host hygiene: stale interpreter artefacts may remain on developer machines`. Removing them would make an on-disk `.venv/` untracked-but-unignored, and `container.sh sync` (`git ls-files --cached --others --exclude-standard`) would then copy the Darwin venv into the container, the defect D5 forbids. Remove only entries that name deleted project paths (check with `git check-ignore -v`).

**Interfaces:**
- Produces: `git ls-files '*.py' | wc -l` = `0`.

**Review focus:**
- The deletion set equals the `python-removal.json` rows plus the 3.02 additions. Compare the deleted paths with the ledger mechanically (Step 2).
- No fixture data is deleted.

- [ ] **Step 1: Delete**

```bash
git rm -r -q src/bike_sim conftest.py pyproject.toml uv.lock .python-version
git ls-files '*.py' | xargs git rm -q
git rm -q tools/install_offline.sh tools/make_offline_bundle.sh tools/native_coverage.sh packaging/install_full.sh packaging/install_slim.sh packaging/OFFLINE_README_full.md packaging/OFFLINE_README_slim.md
git ls-files '*.py' | wc -l
```

Expected: `0`.

- [ ] **Step 2: Compare with the ledger**

```bash
jq -r '.[].path' migration/python-removal.json | sort -u > /tmp/ledger-paths.txt
git diff --cached --name-only --diff-filter=D | sort > /tmp/deleted.txt
echo "ledger-not-deleted:"; comm -23 /tmp/ledger-paths.txt /tmp/deleted.txt
echo "deleted-not-in-ledger:"; comm -13 /tmp/ledger-paths.txt /tmp/deleted.txt
```

Expected: both `comm` lists are empty. The first shows that every ledger path was deleted. The second shows that nothing outside the ledger was deleted; the ledger was completed in 4.01 Step 3, and the non-`.py` deletions listed above (shell wrappers, packaging, `pyproject.toml`, `uv.lock`, `.python-version`) must be ledger rows too. Use `jq` here because the project's Python environment is gone after Step 1. Any listed path is a miss: delete it, add its row, or explain in the evidence why it remains.

- [ ] **Step 3: Verify**

```bash
cmake --workflow --preset release
tools/run_native_checks.sh build/native-release
tools/linux/container.sh sync && tools/linux/container.sh exec cmake --workflow --preset release
```

Expected: exit 0. The binding graph `native/build` no longer configures, because Python is gone; Task 4.06 deletes it.

From this commit on, `native/numerics.lock.json` is frozen: its only writer, `tools/native_dependencies.py`, is deleted. Any later change to a numerics provider needs a new task that ports the probe; nobody edits the record by hand.

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(cpp): T63 delete Python implementations, tests and tools

Ledger: migration/python-removal.json (all rows replacement_landed or
delete_with_python). Goldens: tests/fixtures/golden (ADR-0003).
OD-1: <answer or default>.

Co-Authored-By: <the trailer the executing harness specifies>"
```

---

### Task 4.06: Delete the binding graph (T63 part 2)

**Files (delete):**
- `native/CMakeLists.txt` (the nanobind extension graph) and `native/CMakePresets.json`
- Binding sources: only the files that `#include` nanobind or `Python.h` (29 today; Step 1 inventory) and are absent from the root graph's `native_sources.json`. A plain `grep -rl nanobind native/src` returns ~64 files, because many root-graph TUs only mention nanobind in comments (`runtime/runtime.cpp`, `stepper.cpp`, `runtime/bootstrap.cpp`, `runtime/control.cpp`, `contact/laws.hpp`, `tools/check_contracts.cpp`, `runtime/step.hpp`, `runtime/config.hpp`, …); those are kept. Note that `#include "config.hpp"` in `native/src/runtime/*.hpp` resolves to `runtime/config.hpp`, not the nanobind `native/src/config.hpp`.
- CMake modules unreachable from the root graph. Compute transitive `include()` reachability starting at the root `CMakeLists.txt` (follow `include(`/`add_subdirectory(` through `native/cmake/*.cmake`, `native/tests/**/CMakeLists.txt`). Do **not** seed it from `native/CMakeLists.txt`, which is being deleted and itself includes `BikeCheckContext.cmake` (:688) and `BikeDependencies.cmake`. Delete every `native/cmake/*.cmake` that is not reached. `BikeDependencies.cmake` (the Python-probe document) is expected among them; verify.
- `BikeNumerics.cmake` LIVE mode and `tools/native_dependencies.py` references in it.
- `tools/check_{odr,cppcheck,analyzer,native_frontends,native_tidy}.sh`. Their only callers are `native/CMakeLists.txt:558-588`; the root graph runs the same kinds inside `bike-native-checks` (`check_frontends.cpp`). Keep `tools/check_scripts.sh` if a root target or `run_native_checks.sh` calls it (`grep -rn check_scripts.sh CMakeLists.txt native/cmake tools/*.sh`), and delete it otherwise.

**Files (modify):**
- `CMakeLists.txt`: remove the `BIKE_BUILD_PYTHON_BINDINGS` option and the `add_subdirectory(native)` refusal branch (lines 611-618).
- `tools/linux/Dockerfile`: remove uv, CPython and `UV_PROJECT_ENVIRONMENT`.
- `tools/linux/container.sh`: remove `venv` and the `native` graph from `configure`; `gate` runs `bash tools/run_tests.sh` without `NATIVE_TEST_BUILD_PATH`.
- `native/cmake/Checks.cmake`: drop `WILL_FAIL` from `python_audit_source` and `python_audit_documentation`.
- `migration/no-python-policy.json`:
  - `source_mode.exempt_globs` becomes `["build*/**", "native/build*/**", ".venv/**", ".work/**", "output/**", ".worktrees/**", "docs/superpowers/**", "migration/**", "native/tests/migration/migration_T56_check.cmake"]`. Narrowing it to historical paths alone would expose untracked host trees: today's run reports hits in 7 stale `native/build*/` trees and in `.work/reference-sources/cpython/**/CMakeLists.txt`, the reference checkouts AGENTS.md sanctions. The T56 script is exempt because its `EXPECT_STDERR "python … audit"` needles and `uv run python thing.py` fixture body (:292, :308, :323) are the audit's own negative fixtures; state that in the policy's `description` string (JSON has no comments). Drop `**/proto_native_bench/**`: after 4.05 only C++ and data remain there, so the exemption must not hide new Python.
  - `documentation_mode.exempt_globs` stays `["docs/superpowers/**", "docs/archive/**", "migration/**", "**/CHANGELOG*"]`.
- Residual interpreter tokens on the root graph's surfaces, found by `bike-python-audit --mode source` today:
  - `native/cmake/Provenance.cmake:83`: write the forbidden-path regex as `/include/[p]ython|/lib/[p]ython|lib[p]ython`, so the source text no longer carries the token and the regex is unchanged;
  - `native/cmake/BikeCompilerPolicy.cmake:810-816`: the coverage lane's `find_program(... python3 python)`. Task 3.43 removes it when `Coverage.cmake` lands; if it is still present here, remove it now;
  - `native/cmake/BikeCompilerPolicy.cmake:1090`: reword to `"asan violation evidence lives in the sanitizer_negative_control CTest"` (Task 3.42);
  - `native/cmake/BikeCheckContext.cmake:463`: drop the `python` field read of the dependency document, now always absent or null on the root graph.

**Interfaces:**
- Produces: one build graph, `cmake -S .`; `bike-python-audit --mode source` exits 0.

**Review focus:**
- No first-party target loses a source it still needs. Diff `native_sources.json` before and after; every removed TU must be binding-only.
- No test loses coverage. Every `add_test` in `native/CMakeLists.txt` (e.g. the `native_bench` artifact tests, `native_platform_*_tests`, sanitizer/RTSan controls) has a root-graph successor or a written drop reason. Compare `ctest -N` names of `native/build` against `build/native-release` before deleting.

- [ ] **Step 1: Inventory**

```bash
R="$PWD/"
grep -rlE '^\s*#\s*include\s*[<"](nanobind|Python\.h)' native/src | sort | tee /tmp/binding-tus.txt
cp build/native-release/native_sources.json /tmp/sources-before.json
jq -r '.sources[].path' /tmp/sources-before.json | sed "s|^$R||" | sort -u > /tmp/root-tus.txt
echo "binding-includes-in-root-graph:"; comm -12 /tmp/binding-tus.txt /tmp/root-tus.txt
ctest --test-dir native/build -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort > /tmp/tests-binding.txt
ctest --test-dir build/native-release -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort > /tmp/tests-root.txt
echo "binding-only-tests:"; comm -23 /tmp/tests-binding.txt /tmp/tests-root.txt
```

Expected:
- the `comm -12` list is **empty**. Any entry is a root-graph TU that includes nanobind: **stop** and report it. It is not deleted, and its include is not silently removed;
- every `binding-only-tests` entry appears as a `binding_name` in `migration/ctest-graph-map.json` (written by Task 3.02 Step 2b). Each entry's `root_name` is non-null and present in `/tmp/tests-root.txt`, unless its `disposition` is `drop` with a reason. Check it mechanically:

```bash
jq -r '.[].binding_name' migration/ctest-graph-map.json | sort -u > /tmp/mapped.txt
echo "unmapped:"; comm -23 <(comm -23 /tmp/tests-binding.txt /tmp/tests-root.txt) /tmp/mapped.txt
echo "missing-successor:"; jq -r '.[] | select(.disposition != "drop") | .root_name // "NULL"' migration/ctest-graph-map.json | sort -u | comm -23 - /tmp/tests-root.txt
```

Both lists must be empty before Step 2. An unmapped name means Task 3.02 missed a test: add the mapping (and the root registration) in this task and record it in the evidence.

`native_sources.json` entries are `{path, target, context}` with absolute paths, hence the prefix strip.

- [ ] **Step 2: Delete and edit** as listed.
- [ ] **Step 3: Verify both platforms**

```bash
tools/linux/container.sh image && tools/linux/container.sh down && tools/linux/container.sh sync
cmake --workflow --preset release
ctest --test-dir build/native-release -L audit --output-on-failure
tools/linux/container.sh exec cmake --workflow --preset release
tools/linux/container.sh exec ctest --test-dir build/native-release -L audit --output-on-failure
diff <(jq -r '.sources[].path' /tmp/sources-before.json | sort) <(jq -r '.sources[].path' build/native-release/native_sources.json | sort)
```

Expected:
- every command exits 0. `python_audit_source` passes because its exempt list covers only untracked host trees, history and the T56 fixtures;
- the `diff` shows no root-graph removals: the binding TUs were never in the root manifest. Any removal must be listed in `/tmp/binding-tus.txt`.

- [ ] **Step 4: Commit** `feat(cpp): T63 delete the nanobind binding graph`.

---

### Task 4.07: Documentation and agent contracts (T60)

**Files:**
- Modify: `README.md` (install = CPack tarball or source build via presets; run = `bike-*` executables)
- Modify: `AGENTS.md`:
  - delete the uv rules 1–2;
  - keep rule 3 (native-engineering);
  - remove the Python reference-sources paragraph, or reword it for upstream MuJoCo only;
  - the Verification section from Task 4.02 stays.
- Modify: `CONTEXT.md`, `docs/TESTING.md` (C++ gate; the baseline section becomes a historical note dated before T63)
- Modify: `docs/development/{linux,macos,troubleshooting,dependency-matrix,install-modes,numerical-profiles,viewer-platform-acceptance}.md`, `docs/reference/*.md` (the r2 merge of Task 1.03 brought `uv run …` fenced commands into `alpine-trail-statistics.md` and `mtb-lateral-digitisation.md`), `tools/proto_native_bench/README.md` (rewrite for `native_bench`; it documents the deleted `bench.py`/`dump_model.py`). The pre-4.05 `--mode documentation` run reports 86 hits:
  - macos 18, linux 18, install-modes 18, viewer-platform-acceptance 15, dependency-matrix 7, numerical-profiles 2;
  - `tools/proto_native_bench/README.md` 6;
  - `README.md` 2.

  Rerun it first and use its list as the work list.
- Out of scope: `.claude/skills/**` and other agent-tooling files. The policy's `document_globs` (`*.md`, `docs/**/*.md`, `native/**/*.md`, `tools/**/*.md`, `.github/**/*.md`) do not cover them, so they may keep Python examples. State this in `docs/native-workflow.md`.
- Create: `docs/native-workflow.md` (plan-08 T60), `docs/python-to-cpp-compatibility.md`, a table mapping each old Python command/flag to its native one, generated from `migration/command-map.json` + `cli-contracts.json`.
- Modify: `migration/command-map.json`, `migration/cli-contracts.json` (`status: "native"`)

**Review focus:**
- Every fenced command block in live docs runs as written. `bike-python-audit --mode documentation` passes.
- No link is broken: `grep -o '\](\([^)]*\))' -r README.md docs/*.md docs/development/*.md | …`. Check each relative target with `test -e`.

- [ ] Edit; run `ctest --test-dir build/native-release -R '^python_audit_documentation$' --output-on-failure`; check links; commit `docs: native workflow after Python removal`.

---

### Task 4.08: Final gate and closure

**Files:**
- Create: `migration/final-closure.json` (plan-09 T63 path): `{ "date", "commit", "macos": {command, exit, counts}, "linux": {…}, "removed_python_files": <n>, "goldens": {"darwin-arm64": <n dirs>, "linux-aarch64": <n dirs>}, "not_qualified": ["linux-x86_64", "linux-desktop-gui", …] }`
- Modify: `migration/cpp-linux-completion-evidence.md`, `migration/task-graph.json` (T61–T63 → `verified`)

- [ ] **Step 1: L3 both platforms** (one at a time):

```bash
bash tools/run_tests.sh full; echo "exit=$?"
tools/linux/container.sh sync && tools/linux/container.sh gate full; echo "exit=$?"
```

Expected: both exit 0.

- [ ] **Step 2: Fresh-checkout build** (proves no untracked file is needed besides `.deps`):

```bash
rm -rf /tmp/fresh && git clone --no-local . /tmp/fresh && cp -cR .deps /tmp/fresh/.deps
(cd /tmp/fresh && cmake --workflow --preset release)
```

- [ ] **Step 3: Write `final-closure.json`**, append the evidence, commit `docs(migration): T63 closure`, then `git push origin impl/python-free-cpp`. Do **not** merge into `main2`/`main` (D6).

---

## Self-review record (planner)

- **Spec coverage:**
  - D1–D20 map to tasks:
    - D5 → 1.02;
    - D7 → 1.06–1.09;
    - D8 → 1.17;
    - D9 → 2.05/2.06;
    - D10 → 3.45/4.03;
    - D11 → 2.10/4.03;
    - D12 → 1.04/3.01;
    - D13 → 1.03;
    - D14/D15 → shared procedure + 4.01;
    - D16 → 3.05–3.11;
    - D17 → Phase-3 preamble;
    - D18 → 3.03/3.04/3.41–3.45/4.02/4.05/4.06;
    - D19 → Global Constraints (nothing deletes worktrees);
    - D20 → 2.08/3.44.
  - T52/T53 → the Phase 3 batch tasks; T54 → 3.37–3.40; T55 → 3.03/3.04 (rest deleted per D18); T56 → 1.06–1.09 + 3.29; T57 → 3.42/3.43/4.02; T58 → Phase 2; T59 → 3.44; T60 → 4.07; T61 → 4.03; T62 → 4.04; T63 → 4.05/4.06/4.08.
  - Audit findings → tasks:
    - schema-2 checker → 1.06–1.09;
    - `numerics.lock` Linux → 1.10;
    - `io::Value` → 1.13;
    - Linux platform branches → 1.14;
    - CTAD → 1.15;
    - GCC → 1.16;
    - direct Accelerate symbols → 1.10–1.12;
    - `long double` → 1.17;
    - sincos → 1.18;
    - root-graph Linux refusal → 2.04–2.07;
    - incomplete GLFW tree → 2.09;
    - stale ledgers → 1.05/4.01;
    - unmapped tests → 3.02.
- **Known judgment calls an executor may hit:**
  - 1.13: alias versus box. A decision rule is given.
  - 1.18: the gate is decided from the measurement.
  - 2.02: the expected flag diff must be explained.
  - 3.05–3.10: the physics stop rule.
  - Golden size budget in the shared procedure step 3.
  - OD-1 before 4.05.
- **Deviation from the interview:** Linux L3 moves from the end of Phase 1 to the end of Phase 2 (Task 2.11), for the reason stated under D2.
- **Review round 1 (2026-10-21):** five independent reviewers checked the plan against the repository, each on one slice: Tasks 1.01–1.09, Tasks 1.10–1.19, Phase 2, Phase 3, and Phase 4 plus cross-plan consistency. Each finding was re-verified read-only before it was applied; findings that did not hold were rejected with a reason. Phase 4 changed in these ways:
  - include-based binding inventory, with a stop rule on root-graph intersection and a test-registration diff;
  - source-audit exempt list sized to the real hit list, with residual CMake tokens named;
  - `.gitignore` host-hygiene entries kept;
  - per-record `golden_platforms`;
  - enum-based triage rule;
  - no red node reaching 4.03;
  - stub-libpython negative control;
  - malloc exports kept off `slow`;
  - `run_tests.sh <profile> [preset]` with JUnit provenance and `--no-tests=error`;
  - two-way ledger comparison;
  - CHECKPOINT 4-A;
  - `numerics.lock.json` frozen after 4.05;
  - trailer taken from the executing harness.

  Rejected: deleting `tools/proto_native_bench/` whole. `bench.cpp`/`state_format.hpp` are root-graph TUs (`CMakeLists.txt:561-569`).
