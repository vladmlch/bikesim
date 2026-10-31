# Task C336r — test-port 3.36 remainder report (install/dev tooling; T53/T56 family)

**Batch**: 3.36r — the last two `tests/portability/` files assigned to the
3.36-remainder tranche: `test_install_modes.py` (28 records),
`test_dev_commands.py` (18 records) — 46 records total, all `planned` at
batch start.

**Lane**: `.worktrees/pfc-lane-a`, branch `lane/a2`, base `3901bdb4`
(integration tip after lane/c tranche-B + lane/f FT-1 merge-backs). The
earlier `lane/a` worktree state (`uv.lock` local diff) was stashed before
branching; stash `stash@{0}` on this worktree ("uv.lock local change -
stash before lane/a2") is unrelated to this batch and remains.

## Status: 9 `port`, 0 `golden`, 37 `drop`, all 46 records classified

| file | records | port | drop |
|---|---:|---:|---:|
| test_install_modes.py | 28 | 8 | 20 |
| test_dev_commands.py | 18 | 1 | 17 |

Both `python-removal.json` rows are `replacement_landed`.

Ported surface:

- **One new `cmake -P` CTest contract script** —
  `native/tests/migration/migration_T336r_portability.cmake` (3 cases) —
  registered as `migration_T336r_portability` in `native/cmake/Checks.cmake`
  (root graph only, same pattern as the T53/T336 scripts;
  `migration;t336r;portability` labels, 600 s timeout, convenience target
  `DEPENDS bike-native-checks`). Cases:
  - `unsupported_preset_is_rejected` — `cmake --preset` on a real build-type
    spelling (`RelWithDebInfo`), an all-caps spelling (`DEBUG`) and an
    absent name is each rejected by name with the supported preset set
    listed in the diagnostic — the supported-configuration gate the
    removed `dev.build_commands --config {Debug,Release}` check enforced,
    now owned by `CMakePresets.json` itself.
  - `copied_build_cache_rejected_at_configure` — a real seed configure is
    staged, the build tree copied, and `cmake -S/-B` on the copied tree
    fails with "is different than the directory … where CMakeCache.txt was
    created" (measured exit 1) — the same rejected-before-any-build
    ordering `dev._check_build_cache` enforced.
  - `sanitizer_enabled_without_runtime_is_rejected` — the real emitted
    `native_check_context.json` mutated to `asan.enabled=true` +
    `runtime=null` is rejected by `bike-native-checks --kind context
    --validate-only` ("enabled without a recorded sanitizer runtime") —
    the fail-closed rule `dev._doctor_probe_environment` applied to a
    selected-but-missing sanitizer runtime, moved to configure+validation
    time.
- **Five records map to pre-existing registered tests** (t2a convention):
  - `test_removed_mapped_numerical_provider_is_not_reused` →
    `migration_T56::t56_ctx_missing_required` (a recorded provider file
    that no longer exists is rejected; present-provider arm is
    `t56_ctx_schema2_host`).
  - `test_install_context_matches_source_build_interpreter_and_dependencies`
    → `migration_T56::t56_ctx_schema2_host` (a truthful schema-2 context
    validates; the interpreter record is null-by-design on the Python-free
    root context).
  - `test_doctor_rejects_stale_install_context` (5 parametrized stale
    fields) → `migration_T56` — the mutation battery covers the field
    family (`t56_ctx_root_*`, `t56_ctx_schema2_{host_mismatch,machine,
    pointer_width,foreign_system}`, `t56_ctx_missing_required`,
    `t56_ctx_binding_*`).
  - `test_old_context_requests_reconfigure_without_changing_its_schema` →
    `migration_T56::t56_ctx_root_missing_python` (missing additive
    `python` record → "must record python: null"; the unknown-schema arm
    is `t56_ctx_schema3`).
  - `test_wheel_help_needs_no_native_binary` → `package_smoke` (every
    packaged `bike-* --help` — including `bike-ride`/`bike-research` —
    runs in a scrubbed environment inside the package prefix).
  - `test_wrong_mujoco_fails_before_import` →
    `migration_T53_portability::dependency_lock_pins_the_engine_exactly`
    (the engine version is pinned to exactly 3.12.0 in
    `native/dependencies.lock.json`; wrong bytes fail
    `BikeDependencyLock.cmake` sha256 checks at configure — the gate moved
    from import time to dependency selection).

No golden fixtures: contract/structural surface only, no numerical output
to capture per platform.

## Dropped surface (37 records, `python-only` + named successor)

- **artifact.py loader internals (12)**: extension selection, sys.modules
  caching, cwd-independent selection, no-lazy-build refusal, suffix
  disambiguation, absolute-path selector gate, pre-imported-module guard,
  source-stamp provenance, loader-cause chaining, provider-identity
  failure context, `tests/reference/native_loader.py` plumbing — all are
  `src/bike_sim/native/artifact.py` / test-harness internals; the
  `bike_native` extension does not exist in the Python-free build.
  Successors named per row: `dependencies.lock.json` +
  `BikeDependencyLock.cmake`/`BikeProviderIdentity.cmake` pins,
  `Provenance.cmake`/`dependency_provenance.json`, `provider_binding_
  boundary` contract, `package_smoke` entry-point enumeration.
- **wheel-layout (2)**: Python-wheel checkout-borrowing and python-backend
  route — no wheel remains; `package_smoke` (self-contained package,
  `--backend native` headless ride) + `bike-python-audit --mode package`
  (`migration/no-python-policy.json`).
- **dev.py bootstrap/driver internals (15)**: uv.lock/uv sync bootstrap,
  offline wheelhouse, env-selector composition, subprocess rc plumbing,
  argv builder/path resolution, `shell=True` AST scan, doctor schema/CLI/
  probes (6), `--help` listing — replaced by `CMakePresets.json` presets,
  `docs/installation.md` explicit acquisition + `extract_deps.sh` +
  `ci_presets.sh <preset> offline`, `ctest`/testPresets, the emitted
  `native_check_context.json` + `bike-native-checks --kind context`,
  `Sanitizers.cmake` link-time instrumentation (no preload composer).
- **venv/interpreter internals (4)**: `_same_interpreter`, venv binding,
  `UV_PROJECT_ENVIRONMENT` handling — no interpreter environment remains.

## Case map (test → native case)

- `test_unsupported_config_is_rejected` →
  `migration_T336r_portability::unsupported_preset_is_rejected`
- `test_copied_build_cache_is_rejected_before_build_process` →
  `migration_T336r_portability::copied_build_cache_rejected_at_configure`
- `test_doctor_rejects_missing_selected_sanitizer_before_probe` →
  `migration_T336r_portability::sanitizer_enabled_without_runtime_is_rejected`
- `test_wrong_mujoco_fails_before_import` →
  `migration_T53_portability::dependency_lock_pins_the_engine_exactly`
- `test_removed_mapped_numerical_provider_is_not_reused` →
  `migration_T56::t56_ctx_missing_required`
- `test_install_context_matches_source_build_interpreter_and_dependencies`
  → `migration_T56::t56_ctx_schema2_host`
- `test_doctor_rejects_stale_install_context` → `migration_T56`
- `test_old_context_requests_reconfigure_without_changing_its_schema` →
  `migration_T56::t56_ctx_root_missing_python`
- `test_wheel_help_needs_no_native_binary` → `package_smoke`

## Verification

Oracle files green on both platforms before triage (shared-procedure
step 2): `pytest -q tests/portability/test_dev_commands.py
tests/portability/test_install_modes.py` → **60 passed** (10.45 s) on
darwin-arm64 and **60 passed** (5.73 s) in the linux-aarch64 container.
(The container's `uv run` installed 39 packages — the lane venv was
stale; tests still green. Recorded per the procedure note.)
Measured `pytest --collect-only` ids recorded on every landed record.

| gate | result |
|---|---|
| `cmake build/native-release` reconfigure (darwin-arm64) | clean, 15.6 s |
| `cmake --build build/native-release -j4` | clean, 100% |
| `ctest -R '^migration_T336r_portability$'` darwin | **Passed 1.13 s → 3.13 s, `passed (3 cases)`** |
| red→green proof | needle flipped to absent → test FAILED as designed; restored → green |
| `ctest -R '^migration_T336_portability$'` darwin (regression) | Passed, 6.92 s |
| `ctest -R '^package_smoke$'` darwin | **Passed, 340.19 s** |
| `ctest -R '^migration_T56$'` darwin | **Timeout 1500.76 s** — the fixture exceeds ctest's default 1500 s timeout on this host (no `TIMEOUT` property is registered). Direct `cmake -P` run of the identical command: **`migration_T56 fixture contract passed`, exit 0, ~43 min wall** — every cited `t56_ctx_*` case green, including the shellcheck-flaky arm this time |
| `ctest -R '^migration_T53_portability$'` darwin | **Passed, 430.61 s** |
| linux-aarch64 `container.sh up/sync` | clean (bike-dev-pfc-lane-a) |
| linux `configure native-release root` + `build bike-native-checks bike-python-audit` | clean |
| linux `ctest -R '^migration_T336r_portability$'` | **Passed 0.17 s, `passed (3 cases)`** — first attempt FAILED on message wording (see below), fixed and re-verified |
| linux `ctest -R '^migration_T56$'` | **Failed 1226.23 s — environmental only; all batch-cited `t56_ctx_*` cases passed.** Failed arms: `t56_contracts_{clean,default_root}`/`t56_no_interpreter_reach` (scoped container build has no `bike-contract-tests` binary), `b29_tidy_wrapper_missing_build`, `b29_iso23` (container Python 3.12.3 < required ≥3.13 for the binding-graph scratch configure), `b29_diag_controls` (clang-tidy diagnostic-control differences). Fixture is byte-identical to HEAD; container-qualification gap, not a batch regression |

### Linux wording fix (found by the scoped run, red→green)

The first Linux run failed `unsupported_preset_is_rejected`: the
container's CMake 3.28.3 reports `No such preset` while the newer Darwin
CMake reports `No such configure preset`. The rejection semantics are
identical (nonzero exit + supported-set listing); the needle now matches
`No such.*preset` whitespace-collapsed — verified on both platforms.

### Ledger

- `migration/test-port-map.json`: all 46 records updated —
  `disposition` 9 `port` / 37 `drop`; `native_test_id` = registered names
  only (`migration_T336r_portability::<case>`, `migration_T56[::<case>]`,
  `migration_T53_portability::<case>`, `package_smoke`); `native_source` =
  carrier file; `collected_node_ids` = measured `pytest --collect-only`
  ids on every record (60 nodes over 46 functions); `assertions_capture` =
  measured node counts; `comparison_class` = `contract` (9) / `python-only`
  (37); dated `2026-10-31 3.36r lane/a2` evidence naming each successor.
  Canonical serialization (`indent=2, ensure_ascii=True` + trailing LF);
  programmatic check: exactly 46 records differ vs HEAD, zero outside
  scope, all header fields byte-identical.
- `migration/python-removal.json`: the two rows → `replacement_landed`
  with real `replacement_paths` (CMakePresets.json, BikeCheckContext.cmake,
  check_context.cpp, the fixture carriers, packaging/CMakeLists.txt,
  dependencies.lock.json, BikeDependencyLock.cmake, Provenance.cmake,
  Sanitizers.cmake, package_smoke.cmake, docs/installation.md) and
  `verification_ids` = the ctest names verified green above. The two
  placeholder `native/tests/reference/test_*.cpp` paths the T01 inventory
  predicted were never created — replaced per the tranche convention.
  Inventory-note/repomix scratch fields retained.

## Unresolved concerns

- `bash tools/run_tests.sh native` (L3) is correctly not run: the plan's
  subagent template runs L3 only at checkpoints, and this batch is an L1
  task. The documented baseline reds (`migration_T09`/`migration_T10`
  frozen-oracle + the 41-entry live-parity pytest family at
  `d20ea74c`/`3901bdb4`) are unchanged — the batch touches no physics or
  compiled native sources. Scoped L1: the new registration plus every
  cited carrier ctest green on darwin-arm64;
  `migration_T336r_portability` green on linux-aarch64; source pytest
  files green on both platforms.
- `migration_T56` cannot complete under `ctest` on this darwin host: the
  fixture (~43 min wall here) exceeds ctest's built-in 1500 s default and
  the registration sets no `TIMEOUT` property — pre-existing
  infrastructure debt for the parent to surface (a `TIMEOUT 3600` on the
  `migration_T56`/`migration_T56_check` registrations, or a split
  fixture). The fixture itself passes end-to-end via the direct
  `cmake -P` command ctest issues.
- linux `migration_T56` fails in this lane's container on environmental
  arms only (`bike-contract-tests` absent from the scoped build, container
  Python 3.12 < required 3.13, clang-tidy diagnostic controls) — every
  `t56_ctx_*` case this batch cites passed. Closing that needs a fuller
  container build + image Python ≥3.13; recorded here as open container
  qualification debt, not as a batch regression (fixture unchanged vs
  HEAD, darwin green end-to-end).
- `tools/dev.py`, `src/bike_sim/native/artifact.py`,
  `tests/reference/native_loader.py` and the two pytest files remain in
  tree — deletion is the T63/4.05 gate; `tools/dev.py`'s own
  `python-removal` row stays `planned` (owned by T57, not this batch).
- Process note: an earlier darwin `migration_T56` ctest run was
  invalidated mid-flight (a duplicate `ctest` invocation wiped the
  fixture's shared `SCRATCH_DIR` — `file(REMOVE_RECURSE)` at fixture
  start). Both instances were killed; the clean rerun then hit the 1500 s
  ctest ceiling, and the verdict was taken from the untimed direct
  `cmake -P` run instead.
