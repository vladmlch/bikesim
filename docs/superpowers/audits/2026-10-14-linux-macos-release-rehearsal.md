# Linux/macOS release rehearsal — T29

Date: 2026-10-18. Task: T29 of
`docs/superpowers/plans/2026-10-14-linux-macos-07-validation.md`.

This ledger rehearses the real desktop, performance and offline release
workflows. It is a **structure with live status**, not a completed rehearsal:
rows marked `pending` have no evidence and make no claim. Nothing here may be
copied across platforms or promoted from a headless/CI job.

Ledger rules (from the task's reason and boundaries):

- Every row binds host/machine/toolchain, source commit, command, exit code,
  evidence path/hash and status (`pass` | `fail` | `pending`).
- `pass` means observed on the named host with retained evidence. `pending`
  means the gate has not been demonstrated on the required host; a later run
  must fill the row before release. A gate pass is not a steady-state
  performance claim, and an Xvfb/headless job is not desktop acceptance.
- Historical measurements cover only a 0.005 s invalid prefix; they cannot
  establish Linux realtime throughput or 8x viewer performance
  (`steady_state_evidence` in `src/bike_sim/sim/realtime_measurement.py`
  now classifies every measured run `eligible` | `diagnostic` against the
  requested window).
- Performance comparisons are same-hardware before/after only; cross-machine
  numbers remain descriptive.

Current tree: branch `linux-macos-port`, HEAD
`82d8b685b16f777429acd1e538e5831c7f7c2b94`, plus this task's uncommitted code
changes (`realtime_measurement.steady_state_evidence`, per-run classification
in `tools/measure_realtime.py`, provenance line in
`tools/recording_finalize_bench.py`, `tests/portability/test_performance_evidence.py`).
Pending rows must re-pin the commit at run time.

Authoring host: `DE-UNIT-2433`, macOS 27.0.1 (26A434), Darwin 27.0.0 arm64,
AppleClang 21.0.0, CMake 4.4.3, CPython 3.14.3 (uv), uv 0.12.9.

## Evidence matrix

| # | Gate row | Host / machine / toolchain | Source commit | Command | Exit | Evidence path / hash | Status |
|---|----------|----------------------------|---------------|---------|------|----------------------|--------|
| 1 | Real macOS GUI viewer (T19): physical viewer, camera/mode controls, scale 1 and requested 8 with measured achieved RTF, reset, close | Requires real macOS GUI session on qualified baseline (mjpython launch); no headless substitute | to pin at run | `BIKE_NATIVE_BUILD_PATH=$(pwd)/native/build/release uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1` (and `--time-scale 8`) | — | `docs/development/viewer-platform-acceptance.md` record + session logs/screenshots | **pending** |
| 2 | Linux X11 viewer (T19): same control/scale/close protocol on a real X11 session | Requires real Linux desktop with working DISPLAY + GLFW/OpenGL stack; document Mesa/driver actually used | to pin at run | same `bike-ride` commands on the Linux host | — | `docs/development/viewer-platform-acceptance.md` record + session logs/screenshots | **pending** |
| 3 | XWayland case (T19): GLFW viewer exercised under Wayland via XWayland, separately listed — never evidence for native Wayland | Requires Linux Wayland session with XWayland; separate row, not inferred from row 2 | to pin at run | same `bike-ride` commands under the Wayland/XWayland session | — | `docs/development/viewer-platform-acceptance.md` record | **pending** |
| 4 | macOS native steady-state measurement: complete, valid, identity-stable ≥30 s window ×3 + warmup, per-run `steady_state` classification | `DE-UNIT-2433` macOS 27.0.1 arm64, AppleClang 21.0.0, Release build | to pin at run | `BIKE_NATIVE_BUILD_PATH=$(pwd)/native/build/release uv run --frozen --group native python tools/measure_realtime.py --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --warmup 1 --runs 3 --out output/portability/measure-native` | — | `output/portability/measure-native/report.json` (per-run `steady_state` fields; not yet produced — the gate pass below is compile/test evidence, not a steady-state claim) | **pending** |
| 5 | Recording finalize-bench native-vs-oracle: identical retained data and batch size, both modes, with platform/provider provenance | `DE-UNIT-2433` macOS 27.0.1 arm64, AppleClang 21.0.0, Release build | to pin at run | `BIKE_NATIVE_BUILD_PATH=$(pwd)/native/build/release uv run --frozen --group native python tools/recording_finalize_bench.py --steps 5000 --batch-rows 4096` and same with `--oracle` | — | tool stdout incl. `provenance:` line + preserved column/row equality (`tests/reference/test_native_encode_columns.py`) | **pending** |
| 6 | Offline **full** install on fresh network-blocked target — macOS arm64 py3.14 | `DE-UNIT-2433` macOS 27.0.1 arm64; bundle-built venv CPython 3.14.3 | `068399dbb7444f54166fe9ae61a18b684c0cc4e3` (bundle `project.git_commit`; predates HEAD — see note) | `uv run --frozen --group native python tests/portability/offline_install_probe.py --bundle output/offline-bundles --mode full` (+ `--record`) | 0 | `output/probe work/venv dir {0,1}/bikesim-install-report.json` (sha256 `6c2771405bd40112`, `281fc2e3bf2c93dc`); bundle `output/offline-bundles/bike-sim-offline-py314-macos-arm64-full-native-source.tar.gz` sha256 `8ff120c589ce6b20`; harness `tests/portability/test_offline_install.py` | **pass** |
| 7 | Offline **slim** install on fresh network-blocked target — macOS arm64 py3.14, exact external-provider validation | `DE-UNIT-2433` macOS 27.0.1 arm64; system-site venv CPython 3.14.3 | `068399dbb7444f54166fe9ae61a18b684c0cc4e3` (bundle `project.git_commit`; predates HEAD — see note) | `uv run --frozen --group native python tests/portability/offline_install_probe.py --bundle output/offline-bundles --mode slim --record` | 0 | `output/probe slim work/venv dir {0,1}/bikesim-install-report.json` (sha256 `f39f72e2358139f2`, `329951afb7cacc8a`); bundle `output/offline-bundles/bike-sim-offline-py314-macos-arm64-slim-native-source.tar.gz` sha256 `0aa136323965ecc4`; published recording `output/probe slim work/run recording{, 2}/` | **pass** |
| 8 | Offline full + slim installs on fresh network-blocked **Linux** target | Requires fresh Linux host/container with network disabled at the harness level and cold caches | to pin at run | `tests/portability/offline_install_probe.py --bundle <bundle dir> --mode full` / `--mode slim` inside the blocked environment | — | probe JSON reports + installer logs from the blocked target | **pending** |
| 9 | Recording finalize/replay identity evidence bound to the corresponding real run | Same host as the producing run; identity re-verified on the sealed artifact | `068399db…` for the slim-probe run; replay row to pin at run | publication: probe `--record` leg (done, macOS slim); checked replay: `bike-ride`-family replay command on that artifact | — | publication/finalization: `output/probe slim work/run recording/…/20261017T220453…/manifest.json` (`report_status=complete`, sha256 `0a63939bd72e39ec`) and `record check{, 2}/`; **same-run checked-replay identity not yet recorded** | **pending** |
| 10 | Supplementary — macOS native gate (compile/test correctness; context for rows 4–5, not a substitute) | `DE-UNIT-2433` macOS 27.0.1 arm64, AppleClang 21.0.0, CPython 3.14.3 | `82d8b685b16f777429acd1e538e5831c7f7c2b94` (this session, pre-T29-code tree) | `bash tools/run_tests.sh native` | 0 | session terminal output: 3231 passed + CTest 10/10; `run_tests.sh` writes no log file — export a retained log on the next run | **pass** |
| 11 | Supplementary — Linux aarch64 container native gate | `bike-linux-port` Docker container, Ubuntu aarch64, Clang 18 + ASan runtime | container mounts `/work` = this worktree | `docker exec bike-linux-port bash -c 'cd /work && … bash tools/run_tests.sh native'` | in progress at time of writing; previous attempt exit 1 | `output/portability/logs/linux-gate.log` (prior attempt: CTest 10/10, contract sweeps green, one pytest `F` at ~53% of collection; rerun started 2026-10-18 02:44 local) | **pending** |

## Rows that require a live GUI or a specific host

- **Live GUI required:** rows 1 (real macOS GUI viewer), 2 (Linux X11
  viewer), 3 (XWayland). No headless run, Xvfb job, or window-handle check
  can satisfy them; each needs a documented interactive session with the
  recorded achieved RTF at requested 1x and 8x.
- **Real Linux host required:** rows 2, 3, 8 additionally need a physical or
  VM Linux desktop/target — the aarch64 CI container does not provide a
  display or a network-blocked install target by itself.
- **x86_64 host required:** the advertised `ubuntu-24.04` x86_64 lane
  (T28 runner matrix) has **no rows above** — every Linux result so far is
  aarch64. Any x86_64 support claim needs its own rows (gate, viewer where a
  display exists, offline install) recorded from an x86_64 host; aarch64
  results are descriptive for that lane, not transferable.
- **Same-host pairing:** rows 4–5 are the macOS-arm64 performance pair; their
  before/after numbers may be compared only against measurements recorded on
  `DE-UNIT-2433` (or another single named host), never against Linux runs.

## Notes

- Rows 6–7 evidence predates HEAD: the bundles and probe installs were
  produced at `068399d`. That is recorded, not hidden — a release-candidate
  rehearsal must rebuild bundles at the release commit and re-run the probes
  before those rows can bind the final revision.
- Row 9: the slim probe's `--record` leg proved publication/finalization on
  macOS (`report_status=complete` over a duration-completed run). The
  finalize→replay **identity** half of the row still needs a checked-replay
  run bound to that sealed artifact; the row stays pending until both halves
  exist for the same real run.
- Row 10 provides context only: a green gate establishes correctness, never
  the steady-state claim of row 4 or the desktop acceptance of rows 1–3.
- Row 11: the earlier container attempt reached CTest 10/10 and green
  contract/diagnostic sweeps before exiting 1 inside the pytest leg
  (`linux-pytest-*.log` triage belongs to the gate owner); the row records
  the gate, not a release claim.
- New since this ledger opened: `tools/measure_realtime.py` now stamps each
  measured run with `steady_state` (`eligible`/`diagnostic` + reasons) and
  `tools/recording_finalize_bench.py` prints an
  `execution_provenance('native')` line, so rows 4–5 will produce the
  required claim-classification and provenance when executed.
