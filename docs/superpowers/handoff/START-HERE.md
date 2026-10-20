# Linux + macOS implementation handoff

## Update: 2026-10-17, first numbered wave

T18, T20 and T23 have source implementations and regression tests with status
`implementation_ready`. The user prohibited compilation and execution, so no
project command, test, import probe or native gate was run for this update.
Read [the wave report](reports/2026-10-17-wave-1.md) and its individual task
records before using the historical prompts below.

The user reports T01-T17 plus the parity fix through `bc67bca` already complete.
That external history was not reproduced here. Only T18/T20/T23 statuses were
updated in the task index; its remaining statuses retain the older planning
snapshot. The following original handoff describes that planning snapshot,
not the current implementation state.

## Original planning handoff: 2026-10-14

This bundle is a **plan**, not an implemented port. It is based on the supplied repomix-output2.xml.zip, restored to 1,072 text files and audited without changing implementation source. The existing Linux CMake failure was reproduced. Native builds/tests and macOS/GUI acceptance were not run during planning.

## Read these first

1. [Design contract](docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md)
2. [Source audit and reproduced blocker](docs/superpowers/audits/2026-10-14-linux-macos-source-audit.md)
3. [Master plan, dependencies, owners and release gates](docs/superpowers/plans/2026-10-14-linux-macos-master.md)
4. [Machine-readable task index](task-index.json)

The seven work-package plans contain **30 implementation tasks**, concrete regression examples, file-level changes, interfaces, commands and review criteria. Task numbers are identifiers; follow dependencies. T10 and T13 are a coordinated build-graph/context integration group.

## Transfer instructions

Copy the `docs/superpowers` files into the real repository without replacing unrelated documents. Keep this task index and evidence directory available to every agent. The original repository or uploaded Repomix archive is still needed: this handoff does not include a second copy of the whole source tree.

Recover uv.lock, .python-version, .github workflows, binary assets and Git state from the real checkout first. These are absent from the export, not necessarily absent from the project. Preserve existing macOS support, user edits and the original scientific environment. Source-first native development is the intended installation route; a pure Python wheel is not a native binary package.

## Copy-ready lead-agent prompt

> Implement the Linux + macOS portability plan in this handoff. Start with T01 from task-index.json. Read the design contract, source audit, master plan, actual AGENTS.md and native-engineering safety documents before editing. Preserve macOS, MuJoCo 3.12.0 private ABI, exact local native/Python oracles, physics/controller timing, explicit artifact selection and opt-in final-only recording. Follow task dependencies and file ownership; coordinate T10/T13. For each task, add/run the failing regression and valid control, implement the bounded change, verify exact tests and negative paths, review owned changes and commit. Never replace failures with broad skips/tolerances, generic system BLAS, Python fallback or fake SDK paths. Record actual host/environment/artifact identity and raw test results. Mark unavailable macOS/GUI/offline gates as pending/blocked, not passed. Stop the release if any required Linux or macOS gate is missing. Continue task-by-task until T30 can aggregate complete evidence for both platforms.

## Copy-ready worker prompt: first build task

> Execute T04 only, after T01-T03 are verified. Read the shared spec, audit and T04 in the native-build work package. Isolate the existing Apple SDK logic without deleting it; dispatch before project() by host platform, reject cross-compilation and invalid Apple SDK options on Linux. Add the fake-xcrun/no-Apple-invocation regression and preserve actual Darwin SDK controls. Do not change dependency versions, numerical kernels, runtime behavior or unrelated CMake policy. Return the diff, exact red/green commands/results, host/toolchain evidence and any unrun macOS gate. Coordinate all native/CMakeLists.txt edits with its owner.

## What is included

The design/spec; master and seven task plans; source audit with primary references; task-index.json; extraction/source-hash manifests; Linux configure log; targeted platform-source evidence; and a planning self-check report. See the combined Markdown artifact for a single-file reading version. The split bundle preserves task boundaries and is preferable for independent implementation agents.
