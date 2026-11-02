# Task T63 — final Python→C++ removal transaction

## Scope and result

T63 executes the last removal wave of the Python→C++ migration: every
ledger-owned first-party interpreter source, the binding/package build
surface and all interpreter tooling leave the tree. The product is now a
single native C++ graph configured from the repository root with no
first-party Python runtime, binding or packaging surface.

Platform: Darwin arm64 only. The Linux lane stays deferred (container.sh
carries its PLAN-CHANGE guard); no Docker/container execution was
attempted.

## Commits

| Commit | Contents |
|---|---|
| `acc15a8d` | Preparation: unconditional root graph, binding TU/header deletion (30 files), `native/` transition graph removal (`native/CMakeLists.txt`, `native/CMakePresets.json`, `BikeDependencies.cmake`), `pyproject.toml`/`uv.lock`/`.python-version`/`test.ipynb` removal, offline-bundle/packaging scripts deleted, `BikeNumerics` record-only lane, T53/T336/T56 fixture ports, audit/policy rewrite, dev-guide rewrite, `migration_T63` contract registration |
| `3d84cf04` | Deletion wave: 518 ledger-owned `.py` files + unledgered `tools/migration/_ft7_recapture.py` — 143,889 interpreter-source lines removed |
| (this commit) | Ledger rewrite (`518 deleted` / `29 exempt` with `verification_ids` + `deletion_commit`), `migration/final-closure.json`, this report |

## What was removed

- 519 interpreter-source files total (518 ledger rows + the unledgered
  recapture helper found by the 2026-10-31 reconciliation).
- The nanobind/CPython binding surface: `binding.cpp`, `config.hpp`,
  `binding_{arrays,ownership,readers}.hpp`, all `*_binding.{cpp,hpp}` and
  `*_wire.{cpp,hpp}` adapters, `runtime/test_adapter.*`, the
  `native/CMakeLists.txt` + `native/CMakePresets.json` transition graph
  and `BikeDependencies.cmake`.
- The package/interpreter metadata: `pyproject.toml`, `uv.lock`,
  `.python-version`, `test.ipynb`, `packaging/install_{full,slim}.sh`,
  `tools/{install_offline,make_offline_bundle,native_coverage}.sh`,
  `OFFLINE_README_*`.
- The LIVE numerics probe lane in `BikeNumerics.cmake` (interpreter
  spawn, `tools/native_dependencies.py`); `RECORD_ONLY` is now the only
  source mode, failing closed on any other value.
- Interpreter plumbing inside retained tools (`container.sh` venv/graph
  commands, `ci_presets.sh` Python setup, dev.py bootstrap paths).

## What was kept (deliberate)

- 29 exempt `.py` files: 7 agent-skill helpers under `.agents/` and 22
  offline research/adjudication scripts under `tools/research/` — outside
  the product runtime and every delivery surface; each row carries an
  `exempt_reason`.
- `native/src/writers/pyfloat.hpp`, `state_codec`, `runtime/encode.hpp`,
  `runtime/samples.hpp` and other C++ sources that merely shared a
  nanobind history — verified still consumed by C++ targets.
- `native/tests/reference/test_native_clock_contracts.cpp` — a real
  native test (Stepper/tire writer clocks), not binding infrastructure.
- Immutable fixtures, goldens, `migration/` evidence, oracle manifest —
  retained as data, not runtime dependencies.
- `project/docs/superpowers/audits/2026-10-03-g5-mode-search.md` — a
  sole-copy historical audit document (data).

## Verification evidence (pre-gate, this session)

| Check | Result |
|---|---|
| Full root build | 762 targets green; only duplicate-static-library ld warnings |
| `bike-python-audit --mode source` | exit 0 — all 519 deleted paths absent, 29 exempt present, "source audit clean" |
| `migration_T53_numerics` | pass (96 cases) after the record-only fixture port |
| `migration_T53_portability` | pass after the dev-guide rewrite (doc-link contract green on rewritten linux.md/macos.md) |
| `migration_T02` / `migration_T03` / `migration_T336_portability` | pass |
| `BIKE_BUILD_PYTHON_BINDINGS=ON` configure | fails closed with the T63 removal diagnostic (fixture-verified) |

## Final clean-tree gate

`bash tools/run_tests.sh native` — **exit 0** on the clean-tree release
run: configure + 762-target build + every produced sweep clean
(frontends / tidy / cppcheck / odr / scripts / headers /
diagnostic-controls / context / contracts; analyzer report-only) +
`ctest -LE 'slow|audit|package'` **182/182 pass** including
`migration_T63`. Recorded in `migration/final-closure.json` →
`verification`.

Four gate-hardening commits (`ce35544d`, `573be889`, `f5bbe945`,
`aa30c602`) cleared findings the gate itself exposed — two GCC `-Werror`
diagnostics, a `modernize-raw-string-literal` tidy warning, the contracts
sweep still demanding the deleted `tests/reference` Python tree
(`selected_artifact_assertions` now `not-applicable`, mirroring the
`loader_provenance` convention), a segfault in `native_contract_tests`
from an explicit `std::array<TestCase,46>` that outlived the removed
case (now `std::to_array` — the size cannot go stale), the stale
46-name loader-contracts list, and `replay_manifest_v2` hashing the
retired `native/CMakeLists.txt` into the source manifest.

- `cmake --install` / `cpack` TGZ pair + `package_smoke` + audit negative
  controls — package contents carry no `.py*`/interpreter surface.

## Known documented exceptions

The seven `planned` baseline exceptions enumerated in
`migration/ledger-reconciliation-2026-10-31.md` §7 (tissue-damping share,
pneumatic-tire parity family, pedelec acceptance premises, savage
plateau) are physics/contract re-scope work recorded in
`final-closure.json.qualification_exceptions` — not release blockers and
not T63 deliverables. `python_audit_documentation` remains the
measured-failure WILL_FAIL gate: historical fenced commands in
RIDE/TESTING/RECORDING are marked T63-retired in prose; their fenced
rewrites are tracked separately by design.
