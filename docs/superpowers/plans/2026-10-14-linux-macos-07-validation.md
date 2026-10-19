# 07 - Qualification, CI, and release gates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Make Linux support continuously verifiable without lowering the macOS quality bar.

**Architecture:** Use one evidence schema and the existing quick/native/full runner. Require native tests and real platform acceptance, distinguish exact historical baseline failures from regressions, and preserve every raw test/build outcome.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.27, and strict ISO C++23 without compiler language extensions.
- Keep MuJoCo exactly 3.12.0 for native execution until its private error ABI is deliberately migrated in a separate change.
- Preserve the macOS Accelerate backend, SDK override/fallback behavior, mjpython launcher, and current application commands.
- Preserve same-environment native/Python bitwise tests, exception safety, source/artifact identity, and fail-closed verification.
- Do not promise bitwise-identical results between different operating systems, CPUs, BLAS implementations, or dependency builds.
- Do not change physics, fixed integration periods, controller behavior, recording opt-in/final-only semantics, or replay tolerances for this port.
- Use uv for project Python environments. Never silently regenerate the production lock or upgrade dependencies during a build.
- No implicit native compilation on application import; no silent native-to-Python fallback; no required GUI for headless execution.
- Keep quick/native/full profile meanings; missing required analysis tools remain errors, not successful skips.
- Treat the supplied Repomix export as a source snapshot, not a complete Git checkout or a runnable distribution.
- Native builds are the initial delivery target. Cross-compilation, universal binaries, Windows, and musl are outside this change.

---

## File map and prerequisites

T27 integrates the preceding tests; T28 automates qualified commands rather than inventing a second build process. T29 covers measurements and real desktop/offline evidence that hosted unit tests cannot replace. T30 is the release decision, not an extra implementation feature.

New tools/portability_qualification.py owns result comparison/reporting. tools/ci_portability.py is a thin lane runner driven by a checked-in toolchain matrix. Existing workflows from the real checkout must be recovered before new workflow files are added.

### T27: Preserve test profile meaning and compare failures by exact provenance

**Files:** Create `tools/portability_qualification.py`, `tests/portability/test_qualification_report.py`, and `docs/development/portability-test-matrix.md`. Modify `tools/run_tests.sh`, `tests/reference/native_test_reporter.py`, and `tests/reference/native_loader.py` only to add truthful platform evidence or correct selection defects. Keep `tests/reference/_bits.py` and oracle-pinning behavior intact.

**Interfaces:** Produce `classify_result(current: dict, baseline: dict | None) -> dict` with status pass|baseline_unchanged|fail and raw command/exit/failure evidence. CLI `python tools/portability_qualification.py --profile quick|native|full --build PATH --output PATH [--baseline PATH]` executes the existing runner, never a weakened replacement. Baseline comparison requires matching qualified platform/environment and exact test IDs/signatures.

**Reason and boundaries:** The source records historical full-profile macOS failures. Those do not justify new Linux failures or arbitrary xfails. Collection-only, empty native selection, Python fallback or a sanitizer loader crash must never look like a green native suite.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from tools.portability_qualification import classify_result

def test_new_failure_is_never_a_baseline_pass():
    baseline = {"environment_id": "mac-baseline", "profile": "full",
                "failures": [{"nodeid": "tests/test_old.py::test_old",
                              "signature": "known-check"}]}
    current = {"environment_id": "mac-baseline", "profile": "full",
               "returncode": 1, "collected": 20, "executed": 20,
               "failures": baseline["failures"] + [
                   {"nodeid": "tests/test_new.py::test_new", "signature": "new"}]}
    assert classify_result(current, baseline)["status"] == "fail"

def test_zero_tests_is_failure_even_with_zero_exit():
    current = {"environment_id": "linux", "profile": "native",
               "returncode": 0, "collected": 0, "executed": 0,
               "failures": []}
    assert classify_result(current, None)["status"] == "fail"
```

Add mismatched environment/profile, changed failure signature, missing baseline evidence, collection error, deselected-all, missing actual imported extension, unexpected skip, xfail inflation, timeout and signal-exit cases. The approved baseline is captured independently before the port; the classifier must not automatically bless current failures as a new baseline.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_qualification_report.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Retain profile behavior: quick excludes direct native reference files and slow tests while including tooling-only controls; native runs all selected native/golden tests after full native preflight; full runs the same preflight followed by all tests. New tests/portability files that need no extension stay in quick. Direct native tests belong in tests/reference/test_native_*.py and must be included by the existing selector. Keep Python production/Numba tests separate from the native pure-Python oracle fixture.

Guard MallocScribble/MallocPreScribble/MallocGuardEdges exports to Darwin; do not invent Linux allocator settings that perturb numerical/performance baselines. Preserve PYTHONDEVMODE, sanitizer environments, selected build propagation and extra pytest argument forwarding. Do not swallow preflight errors.

The qualification wrapper uses subprocess.run(check=False) with captured stdout/stderr and a pytest JUnit/report artifact, storing raw status even for an approved historical failure. Parse exact failed node IDs and stable first-failure signatures, plus counts for collected, executed, skipped, xfailed and errored. Native requires a verified imported selected artifact and a nonzero executed direct-oracle set, not only tooling tests with native in their filename. Store provider/compiler/CPU/source/lock identity with results.

`classify_result` first rejects tool/collection/import/timeout errors and missing evidence. Zero failures and zero exit with required executed tests is pass. Only the independently approved macOS full baseline may return baseline_unchanged when every failure matches the exact baseline and no new skip/xfail/error appears; retain raw exit=1 and an explicit non-green full-suite statement. Linux required gates are pass, not inherited baseline_unchanged. Removed baseline failures are improvements, but a partial result must not hide unexecuted tests. Do not change pytest decorators to suppress failures as part of this comparison.

Build a requirement-to-test matrix spanning environment, SDK, engine ABI, BLAS/LAPACK/numerics, loader/provenance, compiler/hardening, source membership, analysis, sanitizer/RTSan, coverage, headless, GUI, async/batch, recording, replay and offline install. Each row names exact test/command and required hosts. Existing golden traces belong to their originating environment; retain local exact oracle tests and portable semantic invariants without pretending a Mac binary trace is automatically a Linux bitwise oracle. A fixture provenance mismatch must be reported, not silently regenerated.

Keep failure identity exact rather than comparing only error categories. Add this private helper in tools/portability_qualification.py and use it only after validating required result/environment fields:

```python
def _failure_keys(value: dict) -> set[tuple[str, str]]:
    keys = set()
    for failure in value["failures"]:
        nodeid, signature = failure["nodeid"], failure["signature"]
        if not isinstance(nodeid, str) or not nodeid:
            raise ValueError("failure nodeid must be a nonempty string")
        if not isinstance(signature, str) or not signature:
            raise ValueError("failure signature must be a nonempty string")
        key = (nodeid, signature)
        if key in keys:
            raise ValueError("duplicate failure evidence")
        keys.add(key)
    return keys
```

For baseline comparison, require _failure_keys(current) to be a subset of _failure_keys(baseline), in addition to the matching Darwin/full environment, complete execution and independently approved baseline conditions above. The subset permits fixed historical failures but cannot admit a new failure; an empty failure set still requires a zero exit and all required tests before status=pass.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_qualification_report.py -q
bash tools/run_tests.sh quick
uv run --frozen --group native python tools/portability_qualification.py --profile native --build native/build/release --output output/portability/native-result.json
uv run --frozen --group native python tools/portability_qualification.py --profile full --build native/build/release --output output/portability/full-result.json
```

Run full on both hosts, passing the independently recorded baseline path only on its exact matching macOS environment. Capture and review every raw result. Verify deliberate deselection or missing-tool injection fails the wrapper. No report may say all tests passed when it only classified unchanged historical failures.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/portability_qualification.py tools/run_tests.sh tests/portability/test_qualification_report.py tests/reference/native_test_reporter.py tests/reference/native_loader.py docs/development/portability-test-matrix.md
git commit -m "test: enforce cross-platform qualification and exact baseline comparison"
```

**Review gate:** Every required test is executed against the selected artifact; new failures/skips cannot hide behind historical macOS baselines, empty collections or a Python fallback.

### T28: Add mandatory Linux and macOS CI without replacing existing workflows

**Files:** Create or extend `.github/workflows/portability.yml` after recovering actual workflows in T01. Create `tools/ci_portability.py`, `docs/development/toolchain-matrix.json`, and `tests/portability/test_ci_lanes.py`. Reuse existing artifact upload/cache/provisioning actions when their pinned versions meet the requirements; do not delete existing macOS jobs.

**Interfaces:** Produce `lane_commands(lane: str, root: Path) -> list[list[str]]` and CLI `python3 tools/ci_portability.py --lane NAME`. Lanes are linux-clang-release, linux-gcc-debug, macos-appleclang-release, linux-asan, macos-asan, linux-rtsan, macos-rtsan, linux-coverage, macos-coverage, linux-full, macos-full, and trusted release/offline/manual lanes. The matrix file contains exact qualified tool/interpreter/provider versions and runner requirements measured in T01-T17.

**Reason and boundaries:** A Linux-only matrix would violate the user requirement. A generic ubuntu-latest/macOS-latest green check can also conceal architecture/toolchain changes. CI must retain concrete runner/environment evidence and enforce both operating systems.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from pathlib import Path
from tools.ci_portability import lane_commands

def test_linux_lane_does_not_use_apple_sdk():
    commands = lane_commands("linux-clang-release", Path("/work/project"))
    text = " ".join(" ".join(command) for command in commands)
    assert "--group native" in text
    assert "CMAKE_OSX_SYSROOT" not in text
    assert "run_tests.sh" in text

def test_macos_lane_is_not_removed():
    commands = lane_commands("macos-appleclang-release", Path("/work/project"))
    assert commands
```

Add lane tests for unique build paths, exact Python/native-group selection, frozen lock, declared compiler, required analysis tools and unsupported-lane failure. Test that a failed command stops the lane and remains a failed job. Validate workflow YAML using the repository's existing workflow linter or a pinned actionlint tool provisioned in the CI toolchain matrix.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_ci_lanes.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Use the lane runner to install/provision only the checked-in exact toolchain profile, then invoke tools/dev.py and run_tests.sh with argv arrays. Resolve compiler/tool paths on the host and record versions; do not assume a default `clang++` is the same across runner images. Pin the interpreter minor plus the actual baseline interpreter lane and retain wheel/provider evidence. No build invokes uv lock or unbounded dependency upgrades.

A concrete hosted workflow starting point is below. The checkout commit is the verified v7.0.1 release commit as checked during planning; review its runtime requirements against the actual runner before use. Existing repository action pins may be retained when deliberately qualified. The native lane script supplies setup, compile, doctor, required analysis and test execution; it may not skip unavailable prerequisites.

```yaml
name: Linux and macOS portability
on:
  pull_request:
  push:
  workflow_dispatch:
permissions:
  contents: read
jobs:
  native:
    strategy:
      fail-fast: false
      matrix:
        include:
          - runner: ubuntu-24.04
            lane: linux-clang-release
          - runner: ubuntu-24.04
            lane: linux-gcc-debug
          - runner: macos-15
            lane: macos-appleclang-release
    runs-on: ${{ matrix.runner }}
    steps:
      - uses: actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1
        with:
          persist-credentials: false
      - name: Run qualified lane
        env:
          BIKE_CI_LANE: ${{ matrix.lane }}
        run: python3 tools/ci_portability.py --lane "$BIKE_CI_LANE"
```

Current official runner documentation lists ubuntu-24.04 x86_64 and macos-15 ARM64; still assert actual platform.machine/OS/image identity at job start and compare to the lane. A hosted macos-15 job does not replace the recovered original macOS environment if it differs. Retain that original baseline as a required trusted release gate. Additional Linux ARM64/macOS Intel lanes become advertised support only after their full qualification.

Add required Debug and Release coverage of the core runtime across the matrix, Python 3.13 plus baseline-interpreter lanes, and separate ASan/RTSan/coverage/full jobs. Runtime-intensive lanes can run as protected scheduled/release jobs, but release acceptance cannot omit them. Missing required hardware/tooling is an unmet gate, not continue-on-error. Preserve existing branch-protection checks; changing required status names must be coordinated so the new jobs are actually required.

Add `if: always()` evidence upload using the repository's approved full-SHA-pinned upload action, or resolve and pin an official compatible release during this task. Upload CMake cache, compile commands, dependency/context/source manifests, doctor report, native test provenance, JUnit/raw logs, sanitizer negative-control logs, coverage exports, and exact platform/tool versions. Do not upload virtualenvs, whole user environments, credentials, source archives with secrets or arbitrary recordings. No success report may be produced by a failed upload masquerading as test completion; retain job logs too.

Cache only safe downloads with keys including OS/arch/Python/lock hash. Never share compiled native artifacts between OSes, compiler families, ABI variants or sanitizer modes. Reconfigure from source on each lane; compiled artifacts are evidence, not reusable universal binaries.

Do not run untrusted pull-request code on persistent self-hosted macOS/GPU/offline runners or with secrets. Use hosted isolated PR jobs; reserve self-hosted GUI/baseline jobs for trusted approved revisions and protected manual/release workflows. Do not use pull_request_target to check out and execute fork code. The real interactive GUI/offline tests remain T29 release evidence even if hosted tests cannot exercise them.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_ci_lanes.py -q
python3 tools/ci_portability.py --lane linux-clang-release
```

Run each lane on its declared host; the shown Linux lane is not run on a Mac. Open one actual CI run from the implementation checkout, confirm both OS jobs executed rather than skipped, inspect test counts and artifact identity, and inject a controlled failure on a disposable branch to verify required-check behavior. Recover/retain current macOS workflows before merging the new one.

- [ ] **Step 5 - Review and commit.**

```sh
git add .github/workflows/portability.yml tools/ci_portability.py docs/development/toolchain-matrix.json tests/portability/test_ci_lanes.py
git commit -m "ci: require Linux and macOS native portability gates"
```

**Review gate:** Both OSes have real required checks and uploaded evidence, exact toolchains are recorded, no mandatory lane uses continue-on-error, and trusted desktop runners cannot execute untrusted PR code.

### T29: Rehearse real desktop, performance, and offline release workflows

**Files:** Create `docs/superpowers/audits/2026-10-14-linux-macos-release-rehearsal.md` and `tests/portability/test_performance_evidence.py`. Modify `src/bike_sim/sim/realtime_measurement.py`, `tools/measure_realtime.py` and `tools/recording_finalize_bench.py` only to include new platform/provider provenance or the explicit reporting classification below. Reuse T19 GUI and T25 offline harnesses.

**Interfaces:** Keep existing measure_realtime CLI and benchmark semantics. Produce `steady_state_evidence(run: dict, minimum_simulated_time_s: float) -> dict` in src/bike_sim/sim/realtime_measurement.py; it consumes the existing measure_run result and returns status=eligible|diagnostic plus a stable reasons list. This is an additional claim-classification field, not a replacement for measured_ok, valid_prefix or compare_runs. Reports include requested duration, actual simulated duration, step count, outcome/first failure, startup/warmup/steady-state/finalization timing and full execution identity. Comparisons are same-hardware before/after; cross-machine differences remain descriptive.

**Reason and boundaries:** Historical measurements can cover only a 0.005-second invalid prefix. They must not be used to claim Linux realtime throughput or 8x viewer performance. Packaging and GUI also require real-host evidence beyond compile/test success.

- [ ] **Step 1 - Add the regression and a valid control.**

Test the actual report-classification function using the existing measure_run field names. A short invalid prefix can retain diagnostic throughput, but cannot establish completed-window steady-state performance:

```python
from bike_sim.sim.realtime_measurement import steady_state_evidence

def measured_run(*, duration=30.0, steps=24000, valid=True,
                 reason="duration_reached", identity=True):
    return {"measured_ok": True, "valid_prefix": valid,
            "identity_unchanged": identity, "errors": [],
            "physics_steps": steps, "simulated_time_s": duration,
            "wall_clock_s": 10.0,
            "prefix": {"steps": steps, "requested_steps": 24000,
                       "reason": reason, "model_valid": valid,
                       "numerically_valid": True}}

def test_short_invalid_prefix_is_diagnostic_not_steady_state():
    run = measured_run(duration=.005, steps=4, valid=False,
                       reason="invalid_controller")
    result = steady_state_evidence(run, 30.0)
    assert result["status"] == "diagnostic"
    assert "invalid_prefix" in result["reasons"]
    assert "incomplete_requested_prefix" in result["reasons"]
    assert "insufficient_simulated_window" in result["reasons"]

def test_completed_valid_window_is_eligible():
    assert steady_state_evidence(measured_run(), 30.0) == {
        "status": "eligible", "reasons": []}

def test_changed_identity_is_not_a_valid_performance_claim():
    result = steady_state_evidence(measured_run(identity=False), 30.0)
    assert result["status"] == "diagnostic"
    assert "identity_changed" in result["reasons"]
```

Extend real compare_runs/report tests with differing physics, timestep, machine and numerical environment, and check that changed settings are exposed. Preserve the existing intentional Python/native backend comparison: backend names and binary/compiler identities are evidence, not a reason to forbid all cross-backend comparison. Same-host before/after claims require explicitly matching workloads and an explanation of the changed implementation/provider.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_performance_evidence.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Add the classification beside the existing measure_run/build_report functions; use their actual schema rather than historical flat fields such as sim_seconds:

```python
import math

def steady_state_evidence(run: dict, minimum_simulated_time_s: float) -> dict:
    if (isinstance(minimum_simulated_time_s, bool)
            or not math.isfinite(minimum_simulated_time_s)
            or minimum_simulated_time_s <= 0):
        raise ValueError("minimum_simulated_time_s must be finite and positive")
    prefix = run["prefix"]
    reasons = []
    if run["measured_ok"] is not True or run["errors"]:
        reasons.append("measurement_failed")
    if (run["valid_prefix"] is not True or prefix["model_valid"] is not True
            or prefix["numerically_valid"] is not True):
        reasons.append("invalid_prefix")
    if run["identity_unchanged"] is not True:
        reasons.append("identity_changed")
    if (prefix["requested_steps"] <= 0
            or prefix["steps"] != prefix["requested_steps"]
            or run["physics_steps"] != prefix["steps"]):
        reasons.append("incomplete_requested_prefix")
    if prefix["reason"] != "duration_reached":
        reasons.append("non_duration_terminal")
    actual = run["simulated_time_s"]
    if (not math.isfinite(actual) or actual <= 0
            or (actual < minimum_simulated_time_s
                and not math.isclose(actual, minimum_simulated_time_s,
                                     rel_tol=1e-9, abs_tol=1e-9))):
        reasons.append("insufficient_simulated_window")
    if not math.isfinite(run["wall_clock_s"]) or run["wall_clock_s"] <= 0:
        reasons.append("invalid_wall_clock")
    return {"status": "diagnostic" if reasons else "eligible", "reasons": reasons}
```

Add per-run classification to the CLI report using the explicit requested measurement duration. Update any exact report-schema validators/fixtures together. `eligible` establishes only this measured prefix/window; it does not prove GUI acceptance, physical calibration or a universal realtime guarantee. Existing `--benchmark` remains a clearly labeled diagnostic mode, with its monitor/validity limitations recorded. Do not rewrite historical benchmark behavior or remove diagnostic throughput from invalid runs.

Retain current benchmark scripts and measured boundaries. Include T12 numerical identity and T13 build policy in provenance, separating diagnostics from comparability fields. Preserve startup/JIT/BLAS warmup and final flush/finalization timing instead of hiding them in a favorable average. Run repeated measurements on the same hardware under documented load and thread settings; report distribution/variance and actual sample counts.

Use the existing native_bench bare/glue/heavy CTest targets as bounded microbenchmarks and the existing whole-accounted-path tool for application timing. Do not set a universal Linux wall-time threshold based on a Mac. Retain the existing visual 1x acceptance requirement on its qualified baseline, and measure the actual achieved scale at requested 8x without changing timestep or controller cadence. If a valid long scenario is unavailable under the unchanged physics checks, report no steady-state conclusion and keep that performance claim unqualified; compile/runtime correctness is still tested by valid short fixtures.

Measure recording finalize native encoder versus existing reviewed oracle mode with identical retained data and batch size. Preserve column/row equality tests before comparing performance. Capture memory growth and shutdown behavior for repeated runs, viewer resets and async workers. A faster result after dropping telemetry/recording intervals is a failure, not an improvement.

Run T19 on real macOS and Linux X11 plus the separately listed XWayland case. Run T25 full/slim in fresh network-blocked target environments. Record exact source commit, environment, commands, outcomes, logs and artifact hashes for every row. Do not copy a result from one platform to another or mark desktop acceptance passed from an Xvfb job. Keep publication/finalization and replay identity evidence attached to the corresponding real run.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python tools/measure_realtime.py --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --warmup 1 --runs 3 --out output/portability/measure-native
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python tools/recording_finalize_bench.py --steps 5000 --batch-rows 4096
BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release" uv run --frozen --group native python tools/recording_finalize_bench.py --steps 5000 --batch-rows 4096 --oracle
uv run --frozen --group native python -m pytest tests/portability/test_performance_evidence.py -q
```

Use new/empty output directories as required by the tool. Capture actual exit codes and realized durations. Rehearsal evidence must link to the GUI sessions and offline install logs from their real hosts. Investigate any material same-host slowdown at the exact changed boundary before release; do not hide it by changing benchmark inputs.

- [ ] **Step 5 - Review and commit.**

```sh
git add src/bike_sim/sim/realtime_measurement.py tools/measure_realtime.py tools/recording_finalize_bench.py tests/portability/test_performance_evidence.py docs/superpowers/audits/2026-10-14-linux-macos-release-rehearsal.md
git commit -m "test: record cross-platform release and performance rehearsal"
```

**Review gate:** Release claims are supported by real host/application/offline evidence. Invalid prefixes, changed workloads or foreign-hardware timings are never presented as steady-state performance success.

### T30: Close every requirement with evidence and perform a no-regression release review

**Files:** Create `docs/superpowers/audits/2026-10-14-linux-macos-acceptance.md` and `tests/portability/test_release_gate.py`. Extend `tools/portability_qualification.py` with release aggregation. Update current support documentation only after its corresponding gate passes.

**Interfaces:** Produce `evaluate_release(records: list[dict], requirements: dict) -> dict` and CLI `python tools/portability_qualification.py --release --evidence DIR --output PATH`. Report status pass|blocked, required missing/failed rows and exact evidence links. Every input record binds source commit, platform/profile, command, raw exit, counts, artifact identity and evidence hashes.

**Reason and boundaries:** Thirty completed code tasks do not automatically establish platform support. Final aggregation must reject missing macOS evidence, different source commits, stale artifacts, unqualified providers and unrun GUI/offline gates.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from tools.portability_qualification import evaluate_release

def test_linux_only_results_cannot_release():
    records = [{"gate": "native", "platform": "Linux", "status": "pass",
                "source_commit": "a" * 40, "evidence": "linux-native.json"}]
    requirements = {"source_commit": "a" * 40,
                    "required": [{"gate": "native", "platform": "Linux"},
                                 {"gate": "native", "platform": "Darwin"}]}
    result = evaluate_release(records, requirements)
    assert result["status"] == "blocked"
    assert any(item["platform"] == "Darwin" for item in result["missing"])
```

Add stale commit, missing evidence file/hash, zero tests, baseline_unchanged on Linux, missing numerical qualification, missing GUI, missing offline mode, invalid sanitizer negative control and duplicate/conflicting record tests. A manually typed status=pass without underlying evidence cannot satisfy a required gate.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_release_gate.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Generate the required gate set from the approved platform matrix, not from whichever result files happen to exist. Validate evidence files and hashes, normalize platform names without collapsing architectures, and require all release records to refer to the same reviewed source commit and intended environment profiles. For amended documentation-only commits, record the explicit source/build fingerprint equivalence rather than silently accepting any newer SHA.

The final matrix must include: recovered macOS baseline; Linux Clang/GCC Debug/Release; Python 3.13 and baseline interpreter; all CTest targets and nonzero exact native oracles; engine private-ABI controls; selected-loader/provenance checks; numerical profile qualification; source/ODR/analysis/hardening controls; ASan and qualified RTSan; LLVM coverage; no-display execution; Linux/macOS real viewers; async/batch lifecycle; opt-in final recording; legacy/foreign recording inspection; same-environment checked replay and foreign mismatch; full/slim offline source installation; current documentation rehearsal. Additional architectures are advertised only when their required rows pass.

A known original macOS full baseline may remain explicitly baseline_unchanged under T27, with the approved ledger linked; no new macOS failure or any new Linux baseline is accepted by analogy. Record raw full-suite status and exact remaining historical failures. Other missing/failed required rows produce blocked.

Perform a diff review focused on prohibited shortcuts: removed macOS branches, deleted SDK fallback tests, generic system BLAS substitution, unverified ABI casts, global fast-math, widened comparison tolerances, platform skips/xfails, dropped native sources, removed realtime annotations, relaxed source identity, loader auto-build/fallback, changed dt/controller physics, premature recording publication and falsely native-capable Python wheels. Review third-party numerical code licenses and dependency/action pins.

Rollback is ordinary reviewed commit reversal of the problematic portability change while retaining user work and original macOS operation; never delete user build/source directories or force-reset the repository. An incomplete Linux qualification means support remains unreleased, not that macOS is removed to simplify the matrix. Summarize actual implemented changes, measured support and remaining qualified limitations accurately in the final acceptance document.

Use a required-set comparison rather than accepting every file discovered under the evidence directory. This private helper computes only missing records; evaluate_release still performs the source/hash/platform/count/negative-control validation described above on every present record:

```python
def _missing_gates(records: list[dict], requirements: dict) -> list[dict]:
    dimensions = ("gate", "platform", "machine", "profile", "toolchain", "mode")
    def key(row):
        return tuple(row.get(name) for name in dimensions)
    present = {key(record) for record in records}
    return [dict(required) for required in requirements["required"]
            if key(required) not in present]
```

The approved real matrix must populate every relevant dimension: e.g. machine/toolchain/profile for compile lanes, mode for full/slim installation. The minimal test fixture intentionally demonstrates only the missing-platform case. Reject ambiguous duplicate keys and missing required dimension values during report validation; do not collapse all Linux or all macOS results into one row.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_release_gate.py -q
uv run --frozen --group native python tools/portability_qualification.py --release --evidence output/portability --output output/portability/acceptance.json
git diff --check
git status --short
```

Review the generated acceptance.json and human-readable audit against the requirement map in the master plan. Deliberately remove one macOS evidence record in a throwaway copy and require blocked. Do not mark this task complete until all required records exist and the independent review finds no prohibited shortcut.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/portability_qualification.py tests/portability/test_release_gate.py docs/superpowers/audits/2026-10-14-linux-macos-acceptance.md
git commit -m "docs: close Linux and macOS portability acceptance gates"
```

**Review gate:** The release has evidence for both platforms and every required capability, with historical exceptions explicit and unchanged. No unrun platform, GUI, numerical or offline test is reported as successful.
