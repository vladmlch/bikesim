# T00 Baseline — frozen checkout, environment and reconciliation evidence

Produced by task T00 ("Freeze the actual checkout and a complete removal
ledger"). Everything below was measured, not assumed. No tests were executed
and no goldens were regenerated; acquisition commands only.

## 1. Frozen identities

| Item | Value |
|---|---|
| Migration worktree | `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp` (branch `impl/python-free-cpp`) |
| Worktree HEAD | `3cc805c2a281aac2ac11be30237fa57407617b14` (`feat(cpp): import python-free-cpp migration handoff bundle`, pure-add of 96 files) |
| Pre-migration application commit | `4667639c6b2917d1aa1f65ff159d45c90544421d` (`add equilibrium cache`, 2026-10-14T19:02:22+02:00) |
| Reference tag (created, annotated) | `reference/python-oracle` → `4667639c6b2917d1aa1f65ff159d45c90544421d`; verified free before creation (only prior tag: `v0.2.0`) |
| Reference worktree (created) | `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/reference-oracle`, detached HEAD at the tag, `.worktrees/` is gitignored (`.gitignore:56`). Stays on the Python code until T63 |
| Second (dirty) checkout | `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean`, branch `main2`, HEAD `4667639` — read-only for this task; nothing there was modified |

## 2. Environment (measured)

| Component | Value | How obtained |
|---|---|---|
| OS | macOS 27.0.1 (26A434), Darwin 27.0.0 | `sw_vers`, `uname -a` |
| Architecture | arm64 (`RELEASE_ARM64_T6041`) | `uname -m` |
| Compiler | Apple clang 21.0.0 (clang-2100.1.1.101), target arm64-apple-darwin27.0.0 | `clang --version` |
| macOS SDK | 26.5 at `/Applications/Xcode.app/.../MacOSX26.5.sdk` | `xcrun --sdk macosx --show-sdk-version/-path` |
| git | 2.50.1 (Apple Git-155) | `git --version` |
| cmake | 4.4.3 | `cmake --version` |
| uv | 0.12.9 (Homebrew aarch64) | `uv --version` |
| Python | CPython 3.14.3 (built with Clang 22.1.1), `requires-python = ">=3.13"` | `uv run --frozen python -c 'import sys; print(sys.version)'` (exit 0; created gitignored `.venv`, installed 38 packages, built bike-sim 0.3.0) |
| NumPy | 2.5.2 — BLAS **and** LAPACK provider: Apple Accelerate (`numpy.show_config`) | `uv run --frozen python` |
| SciPy | 1.17.0 (pinned `==1.17.0` in pyproject) | uv.lock + import |
| MuJoCo | 3.12.0; ships `libmujoco.3.12.0.dylib` inside the Python wheel; `native/CMakeLists.txt:162-169` hard-requires exactly 3.12.0 (pinned error ABI) and links `@rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib` | uv.lock, `import mujoco`, CMakeLists |
| DuckDB | 1.5.6 (pinned `==1.5.6`) | uv.lock + import |
| PyArrow | 26.0.0 (pinned `==26.0.0`) | uv.lock + import |
| Numba | 0.68.0 (llvmlite 0.50.0) | uv.lock + import |
| pytest | 9.1.1 | uv.lock + import |
| nanobind | 3.1.0 | uv.lock |
| bike-sim (first-party) | 0.3.0 | uv.lock |
| Native BLAS provider | Apple Accelerate (`-framework Accelerate`); `native/src/cblas_abi.hpp` pins the classic 32-bit-int CBLAS ABI and documents that NumPy resolves the same provider on this platform | native/CMakeLists.txt:68-95, 695-699; cblas_abi.hpp:1-56 |

Environment/seed notes:

- `NATIVE_TEST_BUILD_PATH` selects the absolute native build dir for tests
  (`tools/run_tests.sh:184-185`, `tests/reference/conftest.py:53`).
- `BIKE_NATIVE_BUILD_PATH` is set by `tests/reference/conftest.py:58-61`.
- `MPLCONFIGDIR=/tmp/matplotlib_cache` is set by `src/bike_sim/viz/theme.py:13`.
- `MUJOCO_MJPYTHON=1` and `DYLD_FALLBACK_LIBRARY_PATH` are probed by
  `src/bike_sim/sim/playground.py:325,336`.
- `PYTHONPATH` is prepended with `tests/reference` for `native`/`full`
  profiles (`tools/run_tests.sh:182`).
- No `PYTHONHASHSEED` or fixed global RNG seed is required by the harness;
  deterministic randomness flows through `default_rng`/`SeedSequence` in
  `src/bike_sim/terrain/*` and `src/bike_sim/sim/research/*` (T19/T34 own the
  seeded-contract capture).
- Numba caveat (from AGENTS.md): `@njit(cache=True)` kernels in
  `physics/tire.py`, `physics/joint_strength.py`, `sim/ride/support_geometry.py`
  compile on first call per process; a cold `step_profile`/`measure_realtime`
  overstates per-step cost — warm up first.
- No test profile was run in T00 (acquisition-only task; no code changes);
  `uv run --frozen` created the gitignored `.venv/` inside the worktree.

## 3. Plan commands and exit codes

| Command | Exit |
|---|---|
| `git rev-parse HEAD` → `3cc805c2a281aac2ac11be30237fa57407617b14` | 0 |
| `git status --porcelain=v1` → empty (worktree clean before T00 writes) | 0 |
| `git ls-files '*.py' '*.pyi' '*.ipynb' 'uv.lock' 'native/*' > migration/tracked-inputs.txt` → 616 lines: 456 `*.py`, 0 `*.pyi`, 0 `*.ipynb`, 159 `native/*`, 1 `uv.lock` | 0 |
| `test -s migration/tracked-inputs.txt` | 0 |
| `git -C /Users/.../mujoco_clean status --porcelain=v1` (read-only enumeration of dirty checkout) | 0 |
| `git -C /Users/.../mujoco_clean ls-files --others --exclude-standard` → 41 untracked paths | 0 |
| `git tag -l` / `git rev-parse --verify reference/python-oracle` → name free (only `v0.2.0`; verify exit 128 = absent) | — |
| `git tag -a reference/python-oracle 4667639… -m "…"` | 0 |
| `git worktree add /Users/.../mujoco_clean/.worktrees/reference-oracle reference/python-oracle` → detached HEAD at 4667639 | 0 |
| `git -C /Users/.../mujoco_clean check-ignore -v .worktrees/reference-oracle` → `.gitignore:56:.worktrees/` | 0 |
| `uv run --frozen python -c 'import sys; print(sys.version)'` | 0 |
| `uv run --frozen python -c 'import numpy,scipy,duckdb,pyarrow,mujoco,numba,pytest …'` | 0 |

## 4. Reconciliation: supplied snapshot vs actual checkout

The supplied `migration/source-manifest.json` covered a Repomix **text**
snapshot of the *working tree* of the dirty `main2` checkout (untracked files
included), not a git tree. Provenance is verified end-to-end:

- `repomix-output.xml` (untracked, reference checkout)
  sha256 `0885327e…0ca4424` == supplied `repomix_sha256`. **Match.**
- `repomix-output.xml.zip`
  sha256 `caf5212a…83abdc` == supplied `archive_sha256`. **Match.**

**Normalization discovered (the only systematic difference):** the supplied
`sha256`/`bytes`/`lines` were computed over file content with **all trailing
newlines stripped**. Verified per file: for every hash-mismatched common file,
`sha256(actual_bytes.rstrip(b'\n'))` equals the supplied hash and supplied
`bytes` equals the stripped length exactly (0 exceptions). Supplied `lines`
equals the line count of the stripped content — which differs from the real
physical count only for files with ≥2 trailing newlines (9 files, all listed
below).

| Class | Count | Meaning |
|---|---|---|
| byte-identical | 3 | `.devin/hooks.v1.json`, `CLAUDE.md`, `tools/proto_native_bench/artifacts/current.json` (files with no trailing newline) |
| trailing-newline-normalized | 1034 | content byte-identical modulo trailing newline(s) — **zero semantic source differences** |
| snapshot-excluded-symlink | 36 | tracked `.claude/skills/*` symlinks (git mode 120000); targets recorded in manifest |
| snapshot-excluded-binary | 8 | `tools/proto_native_bench/artifacts/**` `.mjb/.bin/.npz` (7) + `.claude/skills/grilling.zip` |
| snapshot-excluded-other | 3 | `uv.lock`, `.superpowers/sdd/2026-10-03-v2-seated-plant/progress.md`, `.claude/skills/writing-skills/graphviz-conventions.dot` |
| migration-bundle | 96 | added by HEAD commit `3cc805c` (plans, cards, `migration/*`, `agent-prompts/`, source audit) |
| untracked-snapshot-text | 38 | captured from working tree but **never committed** — exist only in the dirty `main2` checkout; all verified byte-identical modulo trailing newline |
| untracked-not-in-snapshot | 1 | `examples/research/poses/savage_pneumatic.npz` (binary; Repomix excludes binaries) |
| **content-differs (semantic)** | **0** | — |

Accounting identity: 1180 tracked at HEAD = 1084 application tree at 4667639
+ 96 bundle additions (bundle is pure-add; `git diff --name-status` shows 96
`A`, no `M`/`D`). Supplied 1075 = 1037 common + 38 untracked-captured.
Tracked-but-not-in-snapshot 143 = 47 repomix-excluded + 96 bundle.

Files whose supplied `lines` differ from the real physical line count (all
have ≥2 trailing newlines; content still byte-identical modulo the strip):
`native/src/rider/spindle_math.hpp`, `src/bike_sim/mujoco/_xml_format.py`,
`src/bike_sim/mujoco/actuators.py`, `src/bike_sim/mujoco/environment.py`,
`src/bike_sim/sim/__init__.py`, `src/bike_sim/viz/comparison_plot.py`,
`src/bike_sim/viz/geometry_plot.py`,
`tests/reference/test_native_spindle_controller.py`,
`tests/reference/test_realtime_measurement.py`.

### Manifest schema going forward (reconciled file)

`migration/source-manifest.json` now records, per file: real `sha256`,
`bytes`, `lines` (physical count: `\n` count + unterminated last line),
`git_mode` (`null` = untracked), `membership`
(`application-tree` | `migration-bundle` | `reference-checkout-untracked`),
`tracked`, `classification`, and `supplied_sha256/bytes/lines` where the file
existed in the snapshot. Entries flagged `modified_by_t00` carry the pre-T00
committed hash; `reconciliation.t00_outputs` lists the five files this task
creates or rewrites (their frozen hashes are the T00 commit's, by definition —
the manifest does not hash itself).

## 5. Removal ledger (`migration/python-removal.json`)

459 rows — exactly the supplied 459 Python paths. Coverage is complete:

- 456 tracked `*.py` at HEAD — every row's `sha256` now equals the **actual
  file content hash** (verified file-by-file after rewrite).
- 3 files are **untracked in the reference checkout** (never committed):
  `tools/check_offline_sources.py` (T59), `tools/diag_run.py` (T55),
  `tools/install_offline.py` (T59). Each row records `tracked: false` and
  `presence: "untracked-in-reference-checkout"`; their `sha256` is the real
  content hash measured in the dirty checkout (content verified identical to
  the snapshot modulo trailing newline).

New fields per row: `snapshot_sha256`, `snapshot_line_count` (supplied values,
`null` if absent), `tracked`, `presence`. `sha256`/`line_count` are now the
real checkout values. All rows keep `owner_task` (51 distinct owners T03–T63),
`action`, `replacement_paths`, `verification_ids`,
`deletion_gate: "T63 after T61/T62 and row-specific evidence"`,
`status: "planned"`. No Python file — tracked, untracked, or
snapshot-captured — lacks exactly one accountable owner. No extensionless
Python scripts were found among tracked files (shebang sweep). No `*.pyi` or
`*.ipynb` exist.

## 6. Test map (`migration/test-port-map.json`)

Records unchanged (1611 static `test_` functions across 146 source files —
all tracked, all hash-verified). Added `checkout_reconciliation` block with
the verified counts and this note: **no test file is missing or moved**.
`project/tests/reference/test_constraint_mode_search.py` is the only
`project/` test file, is owned by T52 with 6 records, and has no same-name
duplicate anywhere in the checkout — it remains an independent entry until
content/collection prove otherwise (per the mandatory case). Collected pytest
node IDs remain a T01 deliverable (`runtime_node_acquisition_task: "T01"`),
not recorded here.

## 7. Dirty-tree record (reference checkout, left untouched)

`git -C /Users/.../mujoco_clean status --porcelain=v1` shows **only untracked
entries** (41 paths, no modified/staged tracked files): the 38
snapshot-captured files (`.scratch/perf-roadmap/*` ×21, `docs/superpowers/`
2026-10-03/04 audits+plans ×8, `packaging/*` ×4, `tools/*.py|sh` ×5),
`examples/research/poses/savage_pneumatic.npz`, and the two Repomix artifacts
themselves. None were modified by T00.

## 8. Artifact recovery status

| Artifact | Status |
|---|---|
| `uv.lock` | **Present**, tracked at 4667639 (and unchanged at HEAD). Used by `uv run --frozen`. |
| Excluded binary fixtures | **Present**: `tools/proto_native_bench/artifacts/{model.mjb, fixture-v2/*, fixture-v3/*}` (7 files) tracked at 4667639; excluded only from the text snapshot. |
| Model assets | **Present**: three tracked `.mjb` under `tools/proto_native_bench/artifacts/`; generated `*.xml` models are gitignored by design. |
| Native-safety spec/master | **Present**: `docs/agents/native-engineering.md` (AGENTS.md rule 3 target) and `docs/TESTING.md` are tracked; `docs/adr/0001-native-port-mujoco-core.md` also tracked. |
| Snapshot archive | **Verified in place**: `repomix-output.xml` + `.zip` hashes match the supplied manifest fields (see §4). Read-only; not tracked. |
| `examples/research/poses/savage_pneumatic.npz` | **Present but untracked** in the reference checkout (7535 bytes, sha256 `8db0e92c…38faa9`), and **not in the snapshot**. Referenced by committed `examples/research/viewer_physics_pneumatic.toml:18` (`equilibrium_state_file = "poses/savage_pneumatic.npz"`). Consumers: tasks porting equilibrium state-file loading and the pneumatic research/viewer example (T12/T24 config+state seams; the T47-range entrypoints). Obtain/commit-or-fixture it before those gates; do not rename it silently. |
| Git history | Fully present in the real checkout (snapshot lacked it; moot). |

Nothing required by T00–T03 is missing. The only acquisition risk is the
untracked pose NPZ above.

## 9. Blockers and risks carried forward

- `savage_pneumatic.npz` (see §8): binary fixture existing only as an
  untracked file; must be captured/committed before any task validating the
  pneumatic example config or equilibrium state-file import runs.
- The three untracked Python tools live only in the reference checkout;
  T55/T59 must treat the *reference checkout* (or the snapshot bytes) as
  their source of truth — they are not in any git tree.
- `tools/migration/capture_reference.py` and `tests/fixtures/migration/` do
  not exist yet — expected; they are T01/T02 outputs, not missing inputs.
- No gate profile was run (acquisition-only task); there is no fresh
  test-pass evidence for the baseline yet. T01 owns measured test collection.

## 10. Proposed T01 assignment

**T01 — Capture reference behavior before deleting any oracle** is ready to
start. Proposed scope/inputs for the T01 worker:

1. Run inside the reference worktree `.worktrees/reference-oracle` (Python
   oracle, pinned at `reference/python-oracle`); create its `.venv` with
   `uv sync --frozen` (uv.lock verified present) and a numba warmup pass.
2. Collect real pytest node IDs (per-file collection, parameterization
   included) and write them into `test-port-map.json` `collected_node_ids`;
   reconcile the 1611 static count vs collected count; keep
   `test_constraint_mode_search.py` independent.
3. Implement `tools/migration/capture_reference.py` writing to
   `tests/fixtures/migration/` + `migration/oracle-manifest.json` with input
   hashes, this baseline's provider/tool identity (§2), comparison class and
   per-payload SHA-256.
4. Classify the dirty-checkout-only artifacts: capture `savage_pneumatic.npz`
   bytes into the fixture manifest, and snapshot the three untracked tools'
   behavior if any test exercises them (else record "untested tool" rows).
5. Record any historically failing reference tests verbatim (per M0) rather
   than relabeling — baseline has no run data today.
6. Predecessor gate satisfied: T00 ledger coverage is complete (459/459 rows
   have owners; zero semantic source drift vs snapshot).
