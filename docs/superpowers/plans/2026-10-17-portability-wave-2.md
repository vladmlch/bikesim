# Portability Wave 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. These execution skills are unavailable in this session; implement the existing task/review gates with independent workers and static review. The user's prohibition on compilation and execution overrides every run/qualification step.

**Goal:** Implement the second numbered wave, T19 and T24, against the supplied source snapshot, retaining honest unverified platform and installation gates.

**Architecture:** One runtime owner preserves the macOS launcher and viewer ownership while adding platform regression coverage. A separate delivery owner implements manifest-driven source assembly and exact dependency/payload validation. Existing wave-1 code is the input; no physics or recording policy changes are part of this wave.

**Tech Stack:** Python >=3.13, uv, MuJoCo 3.12.0, NumPy, pytest, setuptools, platform-matched wheels, the existing C++23/CMake >=3.27 native build.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`; T19 in `2026-10-14-linux-macos-05-runtime.md`; T24 in `2026-10-14-linux-macos-06-delivery.md`.

**Completion convention:** Checked items below record source implementation,
static review or delivery file operations only. They do not mark any test,
build, execution or host-qualification gate passed. Both task statuses are
`implementation_ready`, with `verification=not_run_by_user_instruction`.

## Global Constraints

- Add Linux support; preserve macOS support and the existing macOS launcher.
- Keep Python >=3.13, CMake >=3.27, strict ISO C++23, and MuJoCo exactly 3.12.0.
- Preserve native/Python numerical contracts, fixed integration periods, render replicas, owning snapshots, recording opt-in/final-only publication, and strict artifact selection.
- No compilation or implicit native fallback on import. No mandatory renderer or display for headless operations.
- Use the frozen dependency profile; do not regenerate `uv.lock` or invent unqualified package versions.
- Native-source bundles target their actual build host; full/slim capability and omitted external dependencies must be explicit.
- Do not ship virtualenvs, host build products, credentials, generated recordings, escaping symlinks, or untracked files by default.
- This is a Repomix source snapshot without original Git history, executable metadata, or binary assets. The unchanged extraction is the patch baseline.
- User instruction: no project execution, builds, tests, probes, package installs, or GUI sessions. Read/edit/package source only. Test code is authored but not executed.
- T18/T20/T23 are implementation-ready inputs with unrun gates; do not promote them or this wave to verified status.

---

## Task 1: T19 launcher and viewer contracts

**Files:** Create `tests/portability/test_viewer_launch_platform.py` and `docs/development/viewer-platform-acceptance.md`. Change `src/bike_sim/sim/playground.py`, the four ride/research viewer modules and the actual physical viewer owner `src/bike_sim/sim/ride/physical_frontend.py` only for source-supported launcher or cleanup defects. Preserve CLI ownership and existing call sites.

**Interfaces:** Consume and retain `ensure_macos_mjpython() -> None`, `MUJOCO_MJPYTHON`, original `sys.argv`, existing DYLD fallback handling, independent render models/data, and current driver/snapshot interfaces.

- [x] Read launcher, callers, viewer loops and existing launch/cleanup regressions.
- [x] Add Linux and Darwin controls, including missing launcher and marked re-exec:

  ```python
  def test_linux_never_reexecutes_mjpython(monkeypatch):
      import shutil
      monkeypatch.setattr(playground.sys, "platform", "linux")
      def forbidden(*args, **kwargs):
          raise AssertionError("Linux inspected or launched mjpython")
      monkeypatch.setattr(playground.os, "execv", forbidden)
      monkeypatch.setattr(shutil, "which", forbidden)
      playground.ensure_macos_mjpython()
  ```

- [x] Trace cleanup on launch failure, terminal hold, normal close and interrupt. Add focused controls for any changed resource ownership. Preserve main-thread rendering and fixed physics integration.
- [x] Preserve the opening platform/sentinel guard and Darwin re-exec behavior. Any production change must have a concrete source-supported defect and an unrun regression.
- [x] Document separate macOS, Linux X11 and Linux XWayland acceptance rows for physical, research, checked replay, recorded playback and playground, all pending actual host evidence.
- [x] Record exact future commands and source-review findings; do not mark visual quality or achieved RTF passed.

## Task 2: T24 source-bundle assembly

**Files:** Create `tools/offline_bundle.py`, `tools/offline_manifest.py`, `tests/portability/test_offline_bundle.py`, `tools/make_offline_bundle.sh`, `tools/check_offline_sources.py`, `packaging/OFFLINE_README_full.md`, and `packaging/OFFLINE_README_slim.md`. Create `tools/prepare_offline_tools.py` and `tests/portability/test_offline_tools.py` for the separate, explicit tool-wheel lock preparation route.

**Interfaces:** Produce `validate_bundle_manifest(value: object) -> dict`, `include_source_path(path: PurePosixPath) -> bool`, and the planned `--target host|linux-x86_64|macos-arm64 --python MAJOR.MINOR --mode full|slim --native --output PATH` CLI. Keep schema version 1 and the project/target/capabilities/requirements/wheels/source_files/prerequisites/slim_external fields from the parent T24 plan.

**Missing legacy inputs:** The supplied snapshot contains no `packaging/`, legacy offline scripts, `uv.lock`, or required benchmark binary payloads. Reconstruct the documented wrapper/naming/source-check contract as new files, explicitly documenting that unseen legacy implementation parity cannot be verified. Do not fabricate an import, original file contents, original executable modes, or lock/binary assets. T25 installation and real-host bundle qualification remain separate.

- [x] Inspect available dependency declarations and lock references, native build inputs and all documented legacy bundle references; identify the dependency closure contract and required source assets. The original lock is missing, so its actual resolution remains unverified.
- [x] Add valid schema/source controls and concrete exclusions:

  ```python
  def test_source_and_generated_payloads_are_distinct():
      assert include_source_path(PurePosixPath("native/CMakeLists.txt"))
      assert include_source_path(PurePosixPath("tests/fixtures/input.json"))
      assert not include_source_path(PurePosixPath("native/build/release/bike_native.so"))
      assert not include_source_path(PurePosixPath(".venv/bin/python"))
      assert not include_source_path(PurePosixPath(".git/config"))
  ```

- [x] Implement strict schema, target/Python/tag, duplicate, hash, path and capability validation. Distinguish native source capability from an already-built extension and from completed qualification.
- [x] Implement tracked-file assembly with explicit required assets, dirty-state provenance, current file bytes, source hashes, source completeness checks, build-root detection and symlink confinement.
- [x] Implement frozen native/dev dependency export, binary-wheel-only resolution, target marker/tag checks, locked wheel hashes and pinned build/tool closure. Include uv wheel/license evidence; capture command arrays, statuses and errors.
- [x] Add `prepare_tool_lock(project_root, wheelhouse, target) -> dict`: read the explicitly selected local wheels, require uv/pip/setuptools/wheel plus the actual PEP 517 requirements, verify metadata and dependency closure, and freeze exact versions/hashes without running a resolver. The CLI writes a fresh supplemental tool lock and refuses to overwrite an existing file.
- [x] Implement full closure and explicit slim external provider requirements with dependency-edge validation. Keep scientific-stack qualification pending T25.
- [x] Add shell wrapper compatibility for documented legacy Linux cp313 full/slim names and Python-only capability; native assembly requires `--native`.
- [x] Add payload/source checker and full/slim instructions. Author negative tests for missing native/lock/dev sources, incompatible host/tags, wheel tampering, duplicate names, unsafe paths and misleading capability claims.

## Integration, review and deliverables

- [x] Review T19 and T24 independently against their parent specs and consumers; resolve source-supported defects without executing project code.
- [x] Update only T19/T24 implementation status and create task/wave reports. Leave all execution and desktop acceptance gates unrun.
- [x] Create a conventional-checkout patch and an exact-snapshot patch if Repomix final-newline removal requires it; verify them as file transformations only.
- [x] Deliver an archive containing every added/changed file with project-relative paths and a complete source snapshot for reproducibility.

## Deferred execution gates (commands are not run in this session)

```sh
uv run --frozen --group native python -m pytest tests/portability/test_viewer_launch_platform.py tests/portability/test_offline_bundle.py tests/portability/test_offline_tools.py -q
bash tools/run_tests.sh native
uv run --frozen --group native python tools/offline_bundle.py --target host --python 3.13 --mode full --native --output output/offline-bundles
uv run --frozen --group native python tools/offline_bundle.py --target host --python 3.13 --mode slim --native --output output/offline-bundles
```

Real macOS/Linux desktop acceptance is T19's outstanding host gate. Actual full/slim network-blocked installation is T25. The x86_64, sanitizer, baseline and later release gates from the supplied review remain outside this wave.
