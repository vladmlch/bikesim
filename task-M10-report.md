# Task M10 — lane/d batch 3.13 merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `501bef6c` — one housekeeping commit
past the stated `46600977`; now `c4cf470f`)
Merged: `lane/d` tip `d456cd59` (base `411c92f3`; 13 commits — 8 ports
incl. `native_tire`/`native_distributed_tire`/
`distributed_tire_reference`/`alpine_track`/`savage_track_surface`/
`terrain_placement_regression`/`tyre_road_profile`/
`native_distributed_stands`, Phase3Ports dedupe repair `9e0cabad`,
numerics fma-lowering fix `008415ad`, terrain/validation test-seam
exports `5df6c45e`, ledgers, evidence): 65 records — 12 golden (all
`golden_platforms` both platforms) / 49 port / 4 drop; ~44.5 MiB
per-platform goldens incl. `episode/` bundles under `native_tire/` +
`native_distributed_tire/` (`probe5` + `stand40`; per-platform
`model.mjb` deliberately not byte-identical — commit-as-is per the
3.12b allowance); new CTest nodes `reference.{savage_track_surface,
terrain_placement_regression,tyre_road_profile,
native_distributed_stands,distributed_tire_reference,alpine_track,
native_tire,native_distributed_tire}` (labels `t52 tyre|terrain golden
slow`).

## Commits

- `d4511d26` — `merge: lane/d batch 3.13 terrain/tyre +
  distributed-tire ports` (`git merge --no-ff lane/d`, two conflicts
  resolved + verified auto-merges)
- `c4cf470f` — `docs(migration): lane/d 3.13 merge-back evidence`
  (appended `## 2026-10-25 — Task 3.13 (lane/d) merge-back`)

Both carry `Co-Authored-By: Devin
<158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

Two real conflicts:

- `migration/cpp-linux-completion-evidence.md` — both sides appended
  at the EOF seam. Chronological resolution per precedent: HEAD's
  `Task 3.14b (lane/c) merge-back` (authored `09:51`/`10:18 +0300`)
  precedes the lane's `Task 3.13 (lane/d)` section (`d456cd59`,
  `22:52:46 +0300`), so the lane's section lands at EOF. Verified
  byte-exact programmatically: merged file == HEAD blob + the lane's
  7,043-byte suffix verbatim; zero HEAD bytes absent; single trailing
  LF at EOF.
- `native/tests/support/golden.hpp` — both sides added
  `Node::peek_label()`: HEAD's 3.14b version is the superset
  (declaration with empty-at-end semantics defined in `golden.cpp`,
  plus `has_next()` + `peek_shape()`); the lane's is an inline
  throwing variant (`entries_.at(cursor_)`). Grep-verified that **no
  lane/d test calls `peek_label()`** (all eight new test files +
  `tyre_reference_support.hpp`), so HEAD's block is kept verbatim —
  merged `golden.hpp` is byte-identical to the HEAD blob and
  `peek_label` keeps exactly one definition. Keeping both would have
  been a redefinition error.

Auto-merged, then verified semantically (programmatic per-record union
check keyed on `(source_path, source_symbol)`):

- `test-port-map.json`: 2,381 record keys identical across
  base/HEAD/lane/merged; **79 changed only by HEAD, 65 only by the
  lane, 0 contested**, 2,237 unchanged — merged field-equal to the
  changing side on every record (bad=0). Lane's 65 rows land as
  49 port / 4 drop / 12 golden; all 12 golden rows carry
  `golden_platforms: [darwin-arm64, linux-aarch64]`. Record spread:
  `test_alpine_track.py` 30, `test_distributed_tire_reference.py` 8,
  `test_native_distributed_stands.py` 2,
  `test_native_distributed_tire.py` 8, `test_native_tire.py` 11,
  `test_savage_track_surface.py` 1, `test_terrain_placement_regression.py`
  3, `test_tyre_road_profile.py` 2.
- `python-removal.json`: 535 rows; 5 HEAD-changed + 8 lane-changed +
  0 contested; the eight batch files flipped `planned →
  replacement_landed` with `verification_ids` retargeted to the new
  `reference.*` nodes.
- `Phase3Ports.cmake`: union — lane's 3.13 eight-registration block
  mid-file after `native_pneumatic_tire`, HEAD's 3.14b five at EOF; 54
  registrations, zero duplicate test names (the intermediate-commit
  quad-append ghost is absent at the tip).
- T25 schema-2 layout preserved — `welded_flat.json` only under
  platform-keyed `initial_state/{darwin-arm64,linux-aarch64}/`; the
  lane's only `manifest.json` adds are the three new per-platform
  episode captures.
- Python sources: the four lane-touched `tests/reference/test_*.py`
  files are lane-only changes — merged blobs byte-identical to the
  lane's.
- All 217 non-shared lane-touched files byte-identical to lane blobs;
  all 190 `tests/fixtures/golden/` files byte-verbatim; trailing LFs
  on every text file (fixture `.json`/`.bin` keep the repo's
  no-trailing-LF convention).

## Merge-back L2

- `cmake --build native/build -j10`: green (auto-reconfigure picked up
  the eight new registrations). `cmake --build build/native-release
  -j10`: green; all eight new executables link (only pre-existing `ld`
  duplicate-library chatter).
- `ctest --test-dir build/native-release -R
  'reference\.(native_tire|native_distributed|distributed_tire_reference|alpine_track|savage_track|terrain_placement|tyre_road)'`:
  **8/8** (0.66–1.20 s).
- `ctest -L reference -j8`: **57/63** — all eight new nodes green
  inside the sweep; six reds = baseline-verbatim: five
  `FAIL fixture is not a regular file` (`native_static_brake`,
  `pneumatic_tire_oracle`, `native_clock_contracts`,
  `native_drive_policies`, `native_drivetrain`) plus
  `golden_episode_tool` keeping its documented post-3.14b signature
  `FAIL native recorder rows reproduce Python oracle` (step-0
  `attachment_samples` divergence — the committed
  `episode/physics.toml` pinning gap; unchanged, verified standalone).
- Linux aarch64 `bike-dev-python-free-cpp` (`/build/root-clang`,
  clang++-18): `container.sh sync` → scoped build of the eight targets
  clean → `ctest -R 'reference\.(…)'` **8/8** — linux-aarch64 goldens
  + `episode/` bundles validated on-platform.
- Parity pytest cold-cache (`NATIVE_TEST_BUILD_PATH=$PWD/native/build`
  + `BIKE_NATIVE_BUILD_PATH` same, fresh
  `BIKE_SIM_EQUILIBRIUM_CACHE_DIR=/tmp/m10-coldcache`, `uv run --frozen
  --group native`, all eight ported sources): **156 passed / 2 failed
  / 7 warnings** in 21.78 s — both reds are exactly the documented
  batch drops' underlying live-oracle baseline failures
  (`test_distributed_probe_and_clock`: `probe_step_inputs →
  apply_forces(advance=False)` staged-fold bitwise divergence;
  `test_distributed_trajectory_matches_python_channel_wise_bitwise`:
  `root.truth.front_load_n` 444.30 vs 428.85). The lane's
  four-golden-file subset figure (55+2) holds within this count.
  Warnings = documented `energy.constraint_work` RuntimeWarning noise.
- `ctest -R 'migration_T' -j8`: **50/53** — **`migration_T25` PASSED
  (5.80 s)** post-merge. Reds baseline-verbatim: `T09`, `T29`, `T56`
  (shellcheck-0.11.0 env drift).

## Concerns

- **Intermediate-commits bisect caveat** (flagged in evidence): the
  lane's per-port commits each appended the cumulative Phase3Ports
  block; commits between `039763a5` and the `9e0cabad` fix carry
  duplicated registrations. A bisect landing mid-lane sees a dirty
  registration file — evaluate the batch at the tip or at the merge.
- `migration_T56` reproduces standalone (shellcheck 0.11.0 stderr
  env drift — seen at M5/M8/M9); environment-dependent, unrelated.
- The replay-cache poisoning vector flagged in M5–M9 remains a live
  infrastructure defect for the parent to surface.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
