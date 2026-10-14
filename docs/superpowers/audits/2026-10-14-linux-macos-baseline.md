# Linux/macOS platform baseline — T01

Date: 2026-10-14. Task: T01 of `docs/superpowers/plans/2026-10-14-linux-macos-master.md`.
Spec: `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`.

This ledger freezes the real checkout's platform baseline **as observed on this
machine**. Every claim is tagged `[observed]` (freshly measured on this host, raw
evidence retained) or `[historical]` (documented by earlier audits/docs, not
re-measured here). Nothing historical is presented as newly measured.

Collector: `tools/portability_baseline.py` (schema_version 1) — read-only
subprocesses and metadata lookups only; no package installation, lock rewrite,
or checkout mutation. Machine-readable output:
`output/portability/baseline.json` (untracked evidence, per `output/*` policy).
Gate logs: `output/portability/logs/{quick,native,full}.log` and
`linux-configure.log`.

## Evidence map — observed vs historical

| Observed on this machine | Historical/source-only |
|---|---|
| Git/env identity: worktree branch `linux-macos-port`, HEAD `4667639`→`a1a2ae8`/`ecfff78`, dirty only via untracked entries; `.git` worktree pointer verified by `git rev-parse` (§1) | — |
| Packages/lock: `uv.lock` sha256 `706c2e5c…` preserved; importlib.metadata versions — numpy 2.5.2, mujoco 3.12.0, nanobind 3.1.0, numba 0.68.0, scipy 1.17.0, duckdb 1.5.6, pyarrow 26.0.0 (§2, baseline.json) | source-comment claims of NumPy 2.5.2 / CPython 3.14.3 — consistent with, and now superseded by, observed values |
| Toolchain/SDK: AppleClang 21.0.0, CMake 4.4.3, uv 0.12.9, git 2.50.1, Xcode 26.6; build SDK = Xcode `MacOSX.sdk` (default source); CLT `xcrun --show-sdk-version` fails rc=1 (§2, §4) | — |
| `quick`: exit 1 on fresh tree (12 `test_native_check_tools` build-ordering failures) → exit 0, 930 passed after `native/build` configured (§4) | — |
| `native`: exit 0, 2957 passed; build + 6 sweeps + headers/diag/context + CTest 6/6 + contracts all green (§4) | TESTING.md/RIDE.md "native profile is green" — re-confirmed here |
| `full`: exit 1, 24 failed / 3878 passed; failing node IDs byte-identical to the documented set (§4) | the 24-ID baseline list and representative traces, TESTING.md ~line 717; `1ccdfcb` isolated-archive baseline |
| Linux configure: exit 1 at `CMakeLists.txt:119` macOS-SDK check, verbatim on Ubuntu 24.04.5 **aarch64** container (§3) | source audit's same signature on Linux **x86_64** — different lane, not this measurement |
| Missing inputs: `.python-version`, `.github/`, `docs/superpowers/{specs,plans}/2026-10-06-native-safety*.md` absent in the real checkout (§1) | snapshot absence ≠ real-repo absence — resolved by direct checkout inspection |
| File modes and fixture assets: `tools/*.sh` exec bits; `proto_native_bench/artifacts/**` content hashes (§1, baseline.json) | — |
| macOS host identity: macOS 27.0.1 arm64, CPython 3.14.3 (§2) | RIDE.md 2026-10-10 GUI acceptance and RTF figures (branch `impl/cpp-port-p2`) — not re-run |
| Not run here: GUI acceptance, sanitizer/RTSan, coverage — pending later task gates | prior sanitizer/RTSan results in `2026-10-08-native-visual-runtime.md` |

## 1. Recovered checkout identity — [observed]

- Worktree `.worktrees/linux-macos-port`, branch `linux-macos-port`, base
  commit `4667639c6b2917d1aa1f65ff159d45c90544421d` (main2 HEAD: "add
  equilibrium cache"). `.git` is a worktree pointer file (92 bytes), not a
  snapshot guess — Git identity is real and verified by `git rev-parse`.
- Working tree dirty at collection time only via untracked entries: this task's
  new files (`tools/portability_baseline.py`, `tests/portability/`), the
  2026-10-14 plan/spec/audit docs, and the handoff bundle. Zero modified
  tracked files.
- `uv.lock` present, 175,814 bytes, sha256
  `706c2e5cccf6e582f0adbd18aa5951a942e14a470d32452b04f95d50914807d6`. Preserved
  exactly; no regeneration was performed or needed.
- `pyproject.toml` sha256
  `0c3b0bb0ee1d1d4df3d81850cef1e6897859099e3e1e30f8bfaf49784a34cd0f`.

### missing_inputs (genuinely absent in the real checkout)

| Input | Status | Note |
|---|---|---|
| `uv.lock` | present | tracked, authoritative |
| `.python-version` | absent | controller ruling: record, do not create |
| `.github/` | absent | no workflows exist to preserve |
| `docs/superpowers/specs/2026-10-06-native-safety.md` | absent | linked by `docs/agents/native-engineering.md:5`; do not fabricate |
| `docs/superpowers/plans/2026-10-06-native-safety-master.md` | absent | linked by `docs/agents/native-engineering.md:6`; do not fabricate |

### File modes and binary fixture assets — [observed]

Executable bits survived checkout: `tools/check_*.sh`, `tools/native_coverage.sh`
are `0755`; `tools/run_tests.sh` is `0644` (invoked via `bash`, consistent with
its git index mode `100644`). Fixture assets needed by tests are present and
content-hashed in `baseline.json.fixture_assets`:
`tools/proto_native_bench/artifacts/{model.mjb (5,792,840 B), current.json,
fixture-v2/, fixture-v3/}` each with `manifest.json`, `model.mjb`, `state.bin`,
`state.npz`.

## 2. macOS host environment — [observed]

| Field | Value |
|---|---|
| Hardware/OS | macOS 27.0.1, arm64 (Darwin Kernel 27.0.0, RELEASE_ARM64_T6041) |
| Hostname | DE-UNIT-2433 (diagnostic only) |
| Python | CPython 3.14.3, uv-provided `cpython-3.14.3-macos-aarch64-none`, soabi `cpython-314-darwin` |
| uv | 0.12.9 (Homebrew, aarch64-apple-darwin) |
| CMake | 4.4.3 |
| Compilers (PATH) | Apple clang 21.0.0 (clang-2100.1.1.101) for `cc`, `c++`, `clang` |
| Git | 2.50.1 (Apple Git-155) |
| Xcode | 26.6 (17F113); `xcode-select -p` → `/Applications/Xcode.app/Contents/Developer` |
| SDK probes | `xcrun --show-sdk-path` → `/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk` (CLT); `xcrun --show-sdk-version` **fails rc=1** on the CLT SDK; `xcrun --sdk macosx --show-sdk-path` → `MacOSX26.5.sdk` under Xcode, version 26.5 |
| CLT SDKs installed | MacOSX.sdk, MacOSX26.sdk, MacOSX26.5.sdk, MacOSX27.sdk, MacOSX27.0.sdk |

Locked environment (`importlib.metadata`, uv `--frozen` env): numpy 2.5.2,
mujoco 3.12.0, nanobind 3.1.0, numba 0.68.0, scipy 1.17.0, duckdb 1.5.6,
pyarrow 26.0.0, matplotlib 3.11.1, glfw 2.10.2, pytest 9.1.1, plotly 7.1.0,
rich 15.0.0, pytz 2026.5, hypothesis 6.168.4. `threadpoolctl` not installed
(numpy `show_runtime` warns accordingly; recorded verbatim in baseline.json).

NumPy 2.5.2 provider evidence (`numpy.show_config()` / `show_runtime()` output
preserved verbatim in `baseline.json.numpy_config`): arm64 wheel, SIMD baseline
NEON/NEON_FP16/NEON_VFPV4/ASIMD, dispatched ASIMDHP+ASIMDDP, ASIMDFHM not
found. The build-machine C compiler recorded by NumPy is clang 15.0.0 — the
wheel's provenance, not this host's toolchain.

This matches the source comments' NumPy 2.5.2 / CPython 3.14.3 claims; the
`>=3.13` metadata floor is satisfied by 3.14.3. One interpreter, one machine —
no claim about other Macs.

## 3. Linux verification host — [observed]

Docker container `bike-linux` on this Mac (Docker Desktop linuxkit VM):

| Field | Value |
|---|---|
| Kernel/arch | Linux 7.0.14-linuxkit, **aarch64** (not x86_64 — recorded as the actual lane) |
| Distro | Ubuntu 24.04.5 LTS (Noble Numbat) |
| CMake | 3.28.3 |
| Compilers | clang 18.1.3, gcc 13.3.0 |
| uv | 0.13.1 (aarch64-unknown-linux-gnu) |
| Python | CPython 3.13.16 via uv |
| Worktree mount | `/work` |

Native configure reproduction (verbatim evidence:
`output/portability/logs/linux-configure.log`):

```text
$ cmake -S /work/native -B /tmp/linux-probe
-- xcrun's default macOS SDK was unavailable:
CMake Error at CMakeLists.txt:119 (message):
  No installed macOS SDK is compatible with compiler '/usr/bin/c++':
-- Configuring incomplete, errors occurred!
CONFIGURE_EXIT=1
```

This **is** the current Linux native baseline: configure fails at the
unconditional macOS SDK probe (finding A01), before dependency discovery. It is
the reproduced blocker, not an accepted final failure (fixed in T04+). The
spec's release-blocking Ubuntu 24.04 **x86_64** lane remains unqualified — no
x86_64 host was used.

## 4. Gate results — [observed on this machine]

Commands run on the macOS host against this worktree; Linux configure result
from the container above. One native gate at a time; nothing ran concurrently.

| Profile | Command | Exit | Result | Log |
|---|---|---|---|---|
| quick (fresh tree) | `bash tools/run_tests.sh quick` | 1 | 918 passed, 1 skipped, 65 deselected, **12 failed** in 294.79 s | `output/portability/logs/quick.log` |
| quick (after native build) | `bash tools/run_tests.sh quick` | 0 | **930 passed**, 1 skipped, 65 deselected in 378.20 s | `output/portability/logs/quick-after-build.log` |
| native | `bash tools/run_tests.sh native` | 0 | **2957 passed**, 196 warnings in 692.67 s; build + 6 sweeps + headers/diag/context + CTest 6/6 + contracts all green | `output/portability/logs/native.log` |
| full | `bash tools/run_tests.sh full` | 1 | **24 failed** / 3878 passed / 1 skipped / 288 warnings in 1604.76 s — failure set **byte-identical** to the documented 24-ID baseline | `output/portability/logs/full.log` |
| linux configure | `cmake -S /work/native -B /tmp/linux-probe` | 1 | configure fails at CMakeLists.txt:119 (macOS SDK) | `output/portability/logs/linux-configure.log` |

native selected build: `native/build` → extension
`native/build/bike_native.cpython-314-darwin.so`; context
(`native/build/native_check_context.json`, schema_version 1): compiler
`/usr/bin/c++` AppleClang 21.0.0.21000101, SDK `MacOSX.sdk` under Xcode
(default source, no override), libcpp_hardening EXTENSIVE. Sweep evidence:
frontends g++-16 ok/none, tidy findings none, analyzer report-only (alpha
findings print, never block), cppcheck none, odr g++-16 none, shellcheck
clean; headers 90 headers/268 TUs none; diagnostic-controls 6/0; context
controls all passed (explicit_sdk passed, missing_sdk rejected, default_sdk
passed, default_sdk_fallback passed, libcpp_hardening passed); CTest 6/6;
check_native_contracts all passed. pytest stderr carries repeated LAPACK
`DLASCL … illegal value` diagnostics (36 occurrences in native.log) —
existing observed noise, kept verbatim.

### quick failures (12) — all `tests/reference/test_native_check_tools.py`

Signature: `FileNotFoundError: native/build/native_check_context.json` or
`AssertionError: native * failed: build directory does not exist:
native/build`. Failing node IDs:

```
test_header_filter_is_portable[original-root]
test_header_filter_is_portable[renamed-root]
test_header_filter_keeps_main_file_diagnostics
test_explicit_sdk_alias_is_retained_and_canonicalized
test_missing_explicit_sdk_fails_without_default_fallback
test_default_sdk_is_resolved_from_xcrun
test_empty_build_type_defaults_to_extensive_hardening
test_fast_hardening_is_available_as_an_explicit_comparison
test_first_party_compile_commands_use_iso_cpp23
test_standalone_header_sweep_compiles_first_party_headers
test_diagnostic_controls_cover_warning_families_and_move
test_context_control_records_sdk_hardening_and_timing_evidence
```

Cause, observed: quick keeps the native-check *controller* tests
(`test_native_check_tools.py` is deliberately not ignored), and on this fresh
worktree no configured build exists at the default selected path
`native/build` — `native_loader.selected_build` maps the empty
`NATIVE_TEST_BUILD_DIR` to `native/build`. These were build-ordering
failures, not physics failures: after the `native` profile configured
`native/build`, the identical quick command went fully green (930 passed,
exit 0 — `quick-after-build.log`).

### full failures (24) — match the documented baseline set

The 24 FAILED node IDs in `full.log` were diffed sorted against the 24 IDs
documented in `docs/TESTING.md` (~line 717): **zero differences** — same IDs,
none new, none missing. Fresh failure signatures match the documented
representatives on the same inputs: flat-launch motor threshold observed
`np.float64(0.5875) >= 0.8`; the two coupled-rider tests raise
`tomllib.TOMLDecodeError: Cannot overwrite a value (at line 104, column 36)`;
the weld-equilibrium stop carries `energy.constraint_work` among its
violation tuple. Exact ID list: `full.log` FAILED block; documented list in
TESTING.md.

## 5. Historical / source-only statements — [historical]

Not re-measured in this session; cited for comparison only.

- `docs/RIDE.md` "Historical visual-runtime acceptance: 2026-10-10" (branch
  `impl/cpp-port-p2`): native profile green; `full` fails only on the recorded
  platform-baseline failure set; strict headless runs terminate at step 4 on a
  documented `energy.constraint_work` baseline violation (t=0.005 s,
  `invalid_controller`) — reported medians describe a diagnostic prefix, not
  throughput; real macOS GUI acceptance exercised (`bike-ride` 1x/8x,
  `bike-replay --viewer`), achieved_rtf mean 0.9999 at 1x, ~1.8x at 8x.
- `docs/TESTING.md` (~line 717) lists **24 failing node IDs** as the
  documented baseline set for `full`, established from an isolated archive of
  commit `1ccdfcb`; representative traces recorded there (flat-launch motor
  threshold 0.5875 vs 0.8, TOMLDecodeError duplicate key, ~8.72e-5 J
  scalar/batch delta, missing planar-arm sites, 40000-step weld residual
  0.197922, ~632 N weld loads vs 785 N rider weight).
- `docs/superpowers/audits/2026-10-08-native-visual-runtime.md` A4/B/C gates:
  `full` at `a743548`: 24 failed / 2973 passed in 791.84 s, sorted FAILED IDs
  byte-identical to the documented baseline set.
- Source audit (`2026-10-14-linux-macos-source-audit.md`): the planning-time
  Linux configure control ran on a **different** host (Linux x86_64, Python
  3.13.5). The failure signature is reproduced here on aarch64, but the x86_64
  environment itself is historical to this ledger.
- GUI acceptance, sanitizer/RTSan gates, coverage: not run for this baseline;
  they are later tasks' gates.

## 6. Observed-vs-historical reconciliation

- `full` failure set: freshly measured 24 IDs == documented 24 IDs
  (TESTING.md), zero new, zero missing. The historical statement "`full`
  fails only on the recorded platform-baseline failure set" holds on this
  machine at this commit.
- `native` profile: historically green; observed green here (2957 passed).
- quick: no historical statement claimed fresh-checkout green; observed red
  (12 build-ordering failures) on a tree without `native/build`, green once
  the build exists. Worth noting for anyone interpreting "quick" as
  build-free: it is not, because `test_native_check_tools.py` runs
  `tools/native_checks.py` against the selected build directory.
- `energy.constraint_work`/`invalid_controller` headless-termination claim
  (RIDE.md): consistent with observed strict-check violations inside the
  full run's recorded failures; the standalone GUI/interactive acceptance
  was not re-run (later task gates).
- Linux: historical audit reproduced the configure blocker on x86_64;
  observed here on aarch64 with the same signature — consistent blocker,
  different architecture lane.

## 7. Collection integrity — [observed]

- `collect_baseline` runs only read-only probes (`git rev-parse/status`,
  `--version` commands, `xcrun` queries, `sw_vers`, NumPy show_config/runtime
  via `sys.executable` subprocess) plus in-process `importlib.metadata` and
  file hashing. No install, sync, stash, or checkout mutation occurs.
- `uv.lock` sha256 recorded in `baseline.json`
  (`706c2e5cccf6e582f0adbd18aa5951a942e14a470d32452b04f95d50914807d6`) and
  re-hashed after all baseline runs — identical; `git status` shows uv.lock
  unmodified. `tools/run_tests.sh` invoked via `bash`, environment used
  `uv run --frozen` throughout — no `uv sync`/`uv lock` ran.
- Scope limits: one macOS host, one interpreter, one Linux container
  (aarch64). No claim about other machines, x86_64, or other SDK/Xcode
  versions.
