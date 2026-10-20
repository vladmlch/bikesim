# Planning review - Linux and macOS portability

**Date:** 2026-10-14. **Scope:** validation of the handoff, not verification of an implemented port.

## Review outcome

The package contains 30 uniquely identified tasks in seven work packages, plus the shared design contract, master plan, source audit, machine-readable scheduling index and source evidence. All tasks remain `not_started`. The dependency graph is acyclic; T10/T13 are explicitly coordinated for build-graph/context integration.

The required user outcomes map to named task gates: Linux compilation (T02-T10/T13); Linux development tools (T03/T09/T13-T17/T23/T26-T28); headless and desktop operation (T18-T22/T29); retained macOS behavior (the mandatory platform matrix and T30); and source-native/offline installation (T23-T26). Numerical correctness and versioned recording/replay identity are separate mandatory gates, not optional improvements.

## Checks performed

- Rehashed all 1,072 restored text files against the extraction manifest: none changed. The original ZIP hash still matches the source audit.
- Checked 30 task headings, 30 task-index entries, dependency references, required files/interfaces/review fields and 150 tracked test/implementation/review phases.
- Parsed 45 Python code examples with Python AST, checked 96 shell examples with `bash -n`, parsed 0 JSON examples and 1 YAML examples, and checked Markdown fence pairing. This checks example syntax, not successful imports or runtime behavior; scoped function-body fragments remain explicitly labeled.
- Checked that every split plan carries the same global constraints and that START-HERE links resolve in the delivered bundle. Checked the plan text for unfinished placeholders.
- Reconciled incorrect draft paths with the actual snapshot: existing analyzer/cppcheck/ODR wrappers, the native column encoder test, physical recording capture, and the inline CMake-generated research identity header. New proposed files remain explicitly distinguished from existing source files.
- Corrected dependency-update ordering, non-synchronizing baseline/build commands, runtime provider identity variants and the performance regression to use real measurement-result fields. Refreshed task-index interfaces after those corrections.

The code examples are proposed implementation/test patterns, not applied repository patches. No review claims that these new tests passed against an implemented port. See `evidence/planning-self-check.json` for machine-readable results.

## Actual project evidence and remaining qualifications

The only executed native-project probe during planning is the original Linux configure command. It failed at the unconditional macOS SDK check; its raw log is included. No native compile, full dependency installation, project simulation suite, macOS session or GUI acceptance was run.

The implementation must still recover the real lockfile/Git state/excluded assets, establish the original macOS environment, qualify complete target wheels and toolchains, probe the actual Linux NumPy provider ABI and CPU dispatch, obtain local exact numerical parity, and run all real-host release gates. These are explicit tasks with stop conditions, not presumed facts.

Initial required Linux scope is Ubuntu 24.04 x86_64/glibc with qualified Clang/GCC. Existing macOS support remains mandatory. Additional architectures and deployment floors require their own evidence; no claim of universal Linux or universal cross-platform bitwise replay is made.
