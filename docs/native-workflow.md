# Native workflow

BikeSim is a single native C++ product: one CMake graph configured from
the repository root, no first-party Python runtime, binding or packaging
surface (ADR-0003, removal completed at T63). This document is the
canonical developer workflow. For the interpreter-era to native command
mapping see docs/python-to-cpp-compatibility.md; for dependency provenance
see native/dependencies.md and docs/reference/README-provenance.md.

## Prerequisites and dependency roots

Third-party sources live under `.deps/` at the repository root. Fetch or
refresh them with the lock-driven acquisition script:

```bash
bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps darwin-arm64
```

(the last argument is the platform token; `linux-aarch64` selects the
Linux lane. `tools/linux/extract_deps.sh` consumes already-downloaded
archives offline. See docs/development/{macos,linux}.md and
native/dependencies.md.)

Every `.deps/<name>-<version>` tree is hash-verified against
`native/dependencies.lock.json`; no interpreter, package or wheel
directory enters the build closure. The optional upstream reference
checkouts (`NUMPY_SOURCE_ROOT`/`MUJOCO_SOURCE_ROOT` or
`.work/reference-sources/`) are port-work aids only — never required for
builds or tests (see AGENTS.md).

## Configure, build, run

```bash
cmake --preset release                       # configure build/native-release
cmake --workflow --preset release            # configure + build + test step
build/native-release/bin/bike-ride --help    # produced executables
```

Presets come from `CMakePresets.json` (default `release`,
`build/native-release`); sanitizer and coverage presets are documented in
docs/TESTING.md. Installed layouts come from CPack packages
(docs/installation.md); the produced binaries are the nine entrypoints —
`bike-sim`, `bike-playground`, `bike-export`, `bike-ride`, `bike-settle`,
`bike-research`, `bike-batch`, `bike-replay`, `bike-recording` — plus the
developer tools `bike-measure-realtime`, `bike-validate`,
`bike-research-sweep`, `bike-native-checks` and `bike-python-audit`.

## Test and verification gates

`bash tools/run_tests.sh <profile> [preset]` is the entry point:

| Profile | Contents |
|---|---|
| `quick` | one `cmake --workflow --preset <preset>` run (configure, build, test step without `slow\|audit\|package`) |
| `native` | configure + build, every produced static-analysis sweep and the structural contracts check via `tools/run_native_checks.sh`, then CTest without `slow\|audit\|package` |
| `full` | `native` plus the explicit `slow`, `audit` and `package` label gates (`--no-tests=error`) |

Rerun one sweep standalone with
`build/native-release/bin/bike-native-checks --kind <kind> --build build/native-release`,
or `cmake --build build/native-release --target check_<name>` where
`native/cmake/Checks.cmake` defines the target. Run one CTest node with
`ctest --test-dir build/native-release -R '<pattern>' --output-on-failure`.

The `audit` label gates the no-Python policy end to end:
`python_audit_source`, `python_audit_documentation` (both required
positive gates), `no_python` (installed-tree audit) and
`audit_negative_controls` (per-class injection controls). The
documentation audit scans active fenced command blocks in the
documentation globs of `migration/no-python-policy.json`; recorded or
deferred commands keep `~~~` fences so prose examples are not mistaken
for live instructions.

Gates serialize on one machine: run them sequentially, not in parallel
against the same build tree.

## Supported platforms

- **macOS arm64 (Darwin/aarch64)** — qualified lane; all gates in this
  document are measured here.
- **Linux aarch64** — the required Linux lane; its container gate is
  deferred (see docs/development/linux.md and
  docs/releases/python-free-qualification.md for the recorded status).
- Linux x86_64, macOS x86_64, Windows, musl, cross-compilation and native
  Wayland are outside the qualified lanes; documentation marks deferred
  workflows explicitly instead of claiming unexecuted qualification.

## Compatibility and migration behavior

- **Research policies.** `load_policy` accepts explicit builtin names
  (`builtin:<name>` and the recognized historical factory/class spellings)
  and `plugin:/absolute/path` native modules. Interpreter-era
  `module:factory` strings are rejected with an actionable error — they
  are never imported (docs/NATIVE_POLICIES.md). Native plugins implement
  the `bike_policy_v1` ABI (native/include/bike/policy_v1.h): a
  same-platform shared-module contract, not a file or wire format.
- **Recordings.** Replay enforces the recorded execution-identity rules:
  the recorded source/runtime/native-artifact identity is checked against
  the current build, and changed or missing identities cannot qualify
  replay (docs/RECORDING.md). Captured goldens under
  `tests/fixtures/golden/<platform>/` are immutable references.
- **Config and output.** Simulation, track, profile, experiment and
  policy TOML/JSON inputs keep the documented schemas; removed
  interpreter-only flags and environment variables are listed in
  docs/python-to-cpp-compatibility.md.

## Out of scope

`.claude/skills/**`, `.agents/skills/**` and other agent-tooling files are outside the policy's
`document_globs` (`*.md`, `docs/**/*.md`, `native/**/*.md`,
`tools/**/*.md`, `.github/**/*.md` plus the top-level `*.md` entries in
each directory). They may keep Python examples; the produced audits do
not treat them as product surfaces. Retained offline research tooling
under `tools/research/` is exempt ledger content — provenance, not a
product runtime dependency (docs/reference/README-provenance.md).
