# Task 4.07 report — Documentation and agent contracts (T60)

**Lane**: `.worktrees/pfc-lane-f`, branch `lane/f5`, base `a1268978`
(integration tip). Darwin-arm64 only (container.sh guarded; no docker leg).
Build tree `build/native-release` (Ninja, AppleClang, MuJoCo 3.12.0).

Scope per plan `2026-10-20-cpp-linux-completion.md` Task 4.07 and plan-08
T60: rewrite every live documentation surface to the post-T63 native
workflow, keep historical evidence as labeled history, create the two
planned documents, mark the command ledgers `native`, and promote
`python_audit_documentation` from a measured `WILL_FAIL` to a required
positive gate.

## Commits

| Commit | Contents |
|---|---|
| `500c0533` | Documentation rewrite (29 files), policy glob widening, positive documentation gate, `docs/native-workflow.md`, `docs/python-to-cpp-compatibility.md`, `docs/reference/README-provenance.md`, `migration/command-map.json` + `migration/cli-contracts.json` statuses → `native` |

## Verification evidence

| Check | Before | After |
|---|---|---|
| `bike-python-audit --mode documentation` | exit **1**, `146 active invocation(s)` (147-line report, `/tmp/doc-audit-before.txt`) | exit **0**, `python documentation audit clean` |
| `ctest -R 'python_audit'` (`build/native-release`) | — | `python_audit_source` + `python_audit_documentation` passed (2/2) |
| `ctest -L audit` | — | 4/4 passed (`python_audit_source`, `python_audit_documentation`, `no_python`, `audit_negative_controls`), 43.28 s |
| `bash tools/run_tests.sh native` (final gate) | — | exit **0**: configure + build + all produced sweeps (frontends, tidy, alpha report-only, cppcheck, odr, scripts, headers, diagnostic-controls, context, contracts) + CTest — **100% tests passed out of 182** (57.65 s test time; includes `t53` 55 doc-contract cases and `t63` clean-tree check) |

## Documentation surfaces rewritten

- **Policy/gate**: `migration/no-python-policy.json` top-level `*.md`
  globs added per scanned directory (fixes the `docs/**/*.md` gap for
  root-level guides); `native/cmake/Checks.cmake` drops `WILL_FAIL` from
  `python_audit_documentation` and asserts the clean output.
- **New documents**: `docs/native-workflow.md` (canonical
  configure/build/test/gate workflow, platforms, compatibility rules,
  agent-tooling audit exclusion per plan), `docs/python-to-cpp-compatibility.md`
  (interpreter-era → native command/flag table generated from
  `migration/command-map.json` + `cli-contracts.json`),
  `docs/reference/README-provenance.md` (post-ADR-0003 immutable golden
  model and retained-research provenance).
- **Live guides**: `README.md`, `CONTEXT.md`, `AGENTS.md`,
  `docs/TESTING.md` (52 findings), `docs/RIDE.md` (11),
  `docs/RECORDING.md` (15), `docs/installation.md`,
  `docs/native-startup.md`, `docs/releases/python-free-qualification.md`.
- **Development docs**: `linux.md`, `macos.md`, `troubleshooting.md`,
  `dependency-matrix.md`, `install-modes.md`, `numerical-profiles.md`,
  `portability-test-matrix.md`, `viewer-platform-acceptance.md` — live
  commands now name produced `build/<preset>/bin/bike-*` executables,
  `cmake --workflow --preset`, `ctest --test-dir`, `tools/run_tests.sh`
  and `tools/run_native_checks.sh`; recorded/deferred commands moved to
  `~~~` fences; Linux container lane marked deferred; T53 doc-contract
  tokens retained (verified by the `t53` label inside the final gate).
- **Reference/research docs**: `alpine-trail-statistics.md`,
  `mtb-lateral-digitisation.md`, `tools/research/{adjudication,alpine_osm_dem}/README.md`
  — external offline tooling fences made non-active and labeled
  provenance. `tools/proto_native_bench/README.md` rewritten for the
  produced `native_bench` target.
- **Agent/native docs**: `docs/agents/native-engineering.md`,
  `docs/agents/native-numerical-profiles.md`, `native/dependencies.md`,
  `native/cmake/BikeProviderIdentity.cmake` comments — post-T63
  verification/parity vocabulary (captured goldens are the gate).
- **Ledgers**: `migration/command-map.json` 13 entry statuses + top-level
  note → `native`; `migration/cli-contracts.json` top-level
  `status: "native"`. `migration/python-removal.json` untouched — it
  tracks source rows, not documentation paths.

## Preserved history (deliberate)

Dated reports, ADRs, qualification records and evidence under
`.superpowers/`, `migration/` and `docs/` keep their Python-era wording;
where they also carried executable-looking fences the fences are now
`~~~` and the prose is explicitly labeled historical/deferred — nothing
was rewritten to pretend past runs were native.

## Remaining Task 4.08 closure items

Per plan `2026-10-20-cpp-linux-completion.md` §Task 4.08:

1. **Step 1 — L3 both platforms**: `bash tools/run_tests.sh full`
   (Darwin arm64) and `tools/linux/container.sh sync &&
   tools/linux/container.sh gate full` (Linux aarch64 container lane —
   currently guard-deferred; requires the container environment).
2. **Step 2 — fresh-checkout build**: `git clone --no-local` + `.deps`
   copy + `cmake --workflow --preset release` to prove no untracked file
   is required.
3. **Step 3 — closure record**: update `migration/final-closure.json`
   (exists from T63) + `migration/cpp-linux-completion-evidence.md`,
   `migration/task-graph.json` (T61–T63 → `verified`), commit
   `docs(migration): T63 closure`, push `impl/python-free-cpp`. Do **not**
   merge into `main2`/`main` (D6).
