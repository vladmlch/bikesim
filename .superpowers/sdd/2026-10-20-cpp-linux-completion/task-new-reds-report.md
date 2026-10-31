# Task report — NEW pytest red cluster fixes (lane/h)

Date: 2026-10-31. Lane: `.worktrees/pfc-lane-h`, branch `lane/h` at
integration tip `dc1533af`.
Scope: the four NEW pytest defect clusters in
`migration/full-gate-2026-10-31.md` (qualification at `d20ea74c`).

## Cluster 1 — `closure_signed_j` worker-crash cluster

**Root cause confirmed.** F1 welded-plant re-scope `ec9b031e` added
`ctx['closure_signed_j']` / `ctx['closure_absolute_j']` reads in
`_build_rt` (`src/bike_sim/sim/ride/async_close.py:99-100`). The real
producer, `PhysicalRuntime._async_context()` (physical_runtime.py:943-944),
snapshots the two cumulative accumulators; `tests/test_async_close.py::
_synthetic_ctx` never learned the keys, so the worker died with
`KeyError: 'closure_signed_j'` → `AsyncWorkerLost`. Reproduced verbatim
before the fix.

**Fix.** `_synthetic_ctx` now carries `'closure_signed_j': 0.` and
`'closure_absolute_j': 0.` — the faithful fresh-episode values
(`PhysicalRuntime.__init__`/`reset` initialize both accumulators to `0.`,
physical_runtime.py:143/478), identical in kind to every sibling work
counter already in the fixture. Not a stub: the fixture's raws feed
`{'equality:0': .01}` through `split_constraint_powers` → `step_work`, so
the closure channel accumulates real work (5e-6 J/step, 5e-5 J over the
10-raw period) on top of those zeros. `worker_entry_probe.py` imports the
same helper — fixed transitively. `tests/test_async_recovery.py` mocks
`_async_context` outright and needed nothing. The native twin
(`native/tests/integration/test_async_close.cpp` `synthetic_state`) zero-
initializes the same counters; no desync.

**Nodes now green (darwin arm64, CPython 3.13.12):**
- `tests/test_async_close.py::test_worker_roundtrip_publishes_period[None]`, `[spawn]`
- `tests/test_async_close.py::test_worker_reports_publication_error[None]`, `[spawn]`
- `tests/test_async_close.py::test_async_close_articulated_episode_completes` (slow; passes — the gate's "sample 0 diverged" signature did not reproduce at this tip)
- `tests/portability/test_worker_platform.py::test_threaded_parent_worker_probe[False-0]`, `[True-130]`
- `tests/portability/test_worker_platform.py::test_mjpython_worker_probe` (darwin-only)
- Whole file rerun: `tests/test_async_close.py` 19 non-slow nodes + `tests/test_async_recovery.py` + `tests/portability/test_worker_platform.py` non-slow — all green. The 4 deselected slow `test_real_cli_async_entry_points` params exercise the real `_async_context` path, not the fixture.

## Cluster 2 — `test_platform_guides_share_workflow_headings`

**Root cause confirmed.** `9866d5d9` (PHASE 1, 2026-10-21) added
`## Container lane (C++ + Linux completion)` to linux.md; the contract
requires identical level-2 heading lists between the platform guides.

**Decision.** The content is legitimate and Linux-only, so no shared
level-2 title fits it and macos.md gains nothing from an N/A stub. The
section provisions the qualified host/toolchain — prerequisite semantics —
so it was demoted to `### Container lane (C++ + Linux completion)` inside
`## Prerequisites`, and its child `### Python-free root graph: Phase 2
source implementation` to `####`. Level-2 heading lists are now identical
(grep-verified). The test contract is unchanged; document structure now
satisfies it.

Side note: the native port `migration_T53_portability.cmake::
dev_docs_guides_share_level2_headings` already recorded this divergence
and checks containment-with-skip; after this fix the strict-equality
branch holds again and its "red at HEAD" comment is stale but benign
(native sweep scope — left for the native lane).

**Green:** `tests/portability/test_development_docs.py` — all 10 nodes.

## Cluster 3 — `test_troubleshooting_covers_required_symptoms`

**Root cause confirmed.** The test fails at the *first* missing token in
dict order; `blas` was masking three more gaps. Measured missing evidence:
`artifact` + `bike_native` (missing-artifact symptom), `invalid_controller`
(doc spelled it `invalid-controller`), `slim` (slim externals symptom).

**Fix — troubleshooting.md rows, all honest content:**
- "Numerics record or ABI mismatch" → "Numerics record or BLAS/LAPACK
  provider-ABI mismatch", naming the rejected provider/integer-width/
  symbol evidence and the pinned providers (NumPy wheel OpenBLAS / system
  Accelerate), per `numerical-profiles.md`. Matches the native port's own
  mapping of the oracle's 'blas' symptom to this row.
- New "Missing extension artifact" row: `bike_native` +
  `NATIVE_TEST_BUILD_PATH` loader vocabulary.
- "Strict invalid-controller stop" evidence now names the verbatim
  `invalid_controller` raw flag and its `rider_controller.infeasible`
  surfacing.
- New "Slim external unresolved at destination" row for the offline
  bundle's `slim_external` provider contract (native port already treats
  this symptom as interpreter-packaging and skips it — no desync).

**Green:** `test_troubleshooting_covers_required_symptoms` passes; no
forbidden-remediation text introduced.

## Cluster 4 — `test_real_checkout_baseline`

**Root cause confirmed.** `.python-version` (content `3.13`) was committed
by `160ead37`/`cc5e18c8` (2026-10-20); the test still asserted it absent.

**Fix.** The expected baseline now asserts `.python-version` is NOT in
`missing_inputs` and that `inputs['.python-version'].present is True`.
The "genuinely absent inputs are named" contract remains covered by the
two native-safety doc assertions. Linux's earlier `git.verified` failure
is the documented container `.git`-less sync artifact, unchanged.

**Green:** `tests/portability/test_baseline.py` — all 4 nodes.

## Ledger

`migration/test-port-map.json`: 15 records' `line`/`end_line` re-derived
by AST after the fixture/test edits (14 in tests/test_async_close.py, 1 in
tests/portability/test_baseline.py). Canonical serializer: indent=2,
ensure_ascii=True, single trailing LF. No evidence strings altered.

## Environment notes

- Disk was full (121 MiB free) at task start — `uv cache clean` freed it;
  a concurrent pytest tmp-dir race also caused one spurious
  `FileNotFoundError` in `test_async_close_articulated_episode_completes`
  (passed on rerun).
- This worktree's `.venv` lacked `threadpoolctl`; two
  `test_native_dependencies.py` probe nodes red purely from that gap —
  `uv sync --frozen --group native` installed the locked 3.7.0 and both
  pass. No relock.
- Full portability + async sweep: 802 passed, 24 skipped, 6 deselected —
  zero source-attributable reds.

## Commits (lane/h)

1. `fix(pytest): carry F1 closure work keys through the async-close synthetic ctx`
2. `fix(docs): nest the linux container lane under the shared prerequisites heading`
3. `fix(docs): cover blas/artifact/invalid_controller/slim symptoms in troubleshooting`
4. `fix(pytest): record the committed .python-version pin in the checkout baseline`
5. `docs(sdd): task-new-reds report — NEW pytest cluster fixes`

## Concerns / leftovers

- `native/tests/migration/migration_T53_portability.cmake:1874-1880`
  comment ("verbatim oracle ... is red at HEAD") is now stale — the strict
  heading-equality branch holds again. Benign: the ported containment
  check still passes. Flag for the native lane to refresh on its next
  sweep-touching change rather than burn a native profile run here.
- Not run: `bash tools/run_tests.sh` profiles (scope is Python
  helper/doc fixes verified at targeted-node level per task brief);
  `test_real_cli_async_entry_points` (slow, unaffected path).
