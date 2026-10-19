# 01 - Environment and developer entry points Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Establish a reproducible baseline and one explicit cross-platform development workflow.

**Architecture:** Keep dependency resolution separate from building. Add a small uv-backed command driver without replacing the existing test profiles or runtime loader.

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

Read AGENTS.md, docs/agents/native-engineering.md, pyproject.toml, tools/run_tests.sh, src/bike_sim/native/artifact.py, and the source audit. tools/dev.py owns user-facing orchestration; tools/portability_baseline.py owns evidence collection; tests/portability contains Python-only tooling tests. Do not place extension-dependent tests in that directory, because the quick profile must not accidentally import native artifacts.

Execute T01-T03 first. T03 may initially report the existing Linux configure failure honestly; native compilation becomes its passing integration gate after T10. Running the command planner itself is independently testable now.

### T01: Recover the real checkout and freeze a truthful platform baseline

**Files:** Create `tools/portability_baseline.py`, `tests/portability/test_baseline.py`, and `docs/superpowers/audits/2026-10-14-linux-macos-baseline.md`. Recover, do not blindly overwrite, `uv.lock`, `.python-version`, `.github/`, and the linked native-safety documents from the real repository.

**Interfaces:** Produce `collect_baseline(root: Path) -> dict` and CLI `uv run --frozen --group native --no-sync python tools/portability_baseline.py --output PATH`. JSON has `schema_version=1`, `git`, `python`, `platform`, `packages`, `numpy_config`, `tools`, and `missing_inputs`. Collection never installs packages.

**Reason and boundaries:** The packed snapshot is insufficient to choose exact new locks or claim current macOS behavior. Baseline evidence is a deliverable, not an excuse to discard excluded project files. The source comments mention NumPy 2.5.2/CPython 3.14.3 while metadata allows Python 3.13.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a test that works without a native binary:

```python
from tools.portability_baseline import collect_baseline

def test_missing_checkout_inputs_are_explicit(tmp_path):
    (tmp_path / "pyproject.toml").write_text('[project]\nname="probe"\n')
    report = collect_baseline(tmp_path)
    assert report["schema_version"] == 1
    assert "uv.lock" in report["missing_inputs"]
    assert report["git"]["commit"] is None
    assert report["git"]["verified"] is False
```

Also create a temporary Git repository with one committed text file, edit it, and require `dirty=True` without modifying or stashing the change.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native --no-sync python -m pytest tests/portability/test_baseline.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Use non-mutating subprocesses and preserve failed-command evidence:

```python
import subprocess
from pathlib import Path

def capture(argv: list[str], root: Path) -> dict:
    try:
        result = subprocess.run(argv, cwd=root, text=True,
                                capture_output=True, timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"argv": argv, "returncode": None, "error": str(exc)}
    return {"argv": argv, "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}
```

`collect_baseline` uses this helper for `git rev-parse HEAD`, `git status --porcelain`, `uv --version`, `cmake --version`, the configured compiler `--version`, and platform-specific SDK commands only on Darwin. Record importlib.metadata versions for every declared direct dependency plus nanobind, hypothesis and threadpoolctl when installed. Use null plus a named error for unavailable optional collection tools. Capture `numpy.show_config()` and `numpy.show_runtime()` output without editing environment variables. Hash pyproject.toml and uv.lock if present.

Recover the original checkout before applying code changes. Inventory file modes and any excluded model/fixture assets needed by tests. Preserve the existing lock exactly for the first macOS baseline. Run current quick/native/full on their appropriate hosts and retain exact failing node IDs, signatures, stdout/stderr and exit status. The Linux SDK configure failure can be the initial native baseline; it is not an accepted final failure.

Write the baseline ledger with two explicit columns: observed on this machine, and historical/source-only. Record the real macOS version/architecture/SDK/compiler/interpreter rather than claiming all Macs were tested. Do not run concurrent full native gates on one host.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native --no-sync python -m pytest tests/portability/test_baseline.py -q
uv run --frozen --group native --no-sync python tools/portability_baseline.py --output output/portability/baseline.json
git diff --check
```

Verify no package installation or lock rewrite occurred during evidence collection. Review baseline failures against the exact historical docs/RIDE.md statements, not broad exception categories. If the real lock is irretrievable, create a separate candidate environment and label it reconstructed; preserve that distinction through release review.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/portability_baseline.py tests/portability/test_baseline.py docs/superpowers/audits/2026-10-14-linux-macos-baseline.md
git commit -m "test: capture cross-platform portability baseline"
```

**Review gate:** Both the extracted-snapshot limitation and the actual checkout/environment identity are recorded. No historical macOS claim is presented as newly measured.

### T02: Make native dependency resolution consistent and qualify wheels

**Files:** Modify `pyproject.toml`, the recovered `uv.lock`, and `src/bike_sim/native/artifact.py::required_dependencies/check_dependencies`; create `tests/portability/test_dependency_policy.py` and `docs/development/dependency-matrix.md`.

**Interfaces:** Native scientific profile has an exact MuJoCo pin and an exact baseline NumPy pin. Existing `required_dependencies() -> dict[str,str]` continues to return the direct exact native pins. New tooling dependency `threadpoolctl>=3.6` is used only for provider discovery/diagnostics, not physics.

**Reason and boundaries:** Plain core requirements allow MuJoCo >=3.0.0 while native requires 3.12.0. An unqualified NumPy upgrade can invalidate handcrafted numerical kernels. Optional-group synchronization must not silently change the interpreter environment under a built extension.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
import tomllib
from pathlib import Path

def test_native_scientific_versions_are_pinned():
    project = tomllib.loads(Path("pyproject.toml").read_text())
    native = project["dependency-groups"]["native"]
    assert "mujoco==3.12.0" in native
    assert "numpy==2.5.2" in native
    assert "mujoco==3.12.0" in project["project"]["dependencies"]
```

The NumPy literal is the source-supported candidate. T01 must confirm it against the recovered baseline. If the real baseline uses another NumPy version, preserve that version and its source reference, update this test/profile consistently, and document the discrepancy; do not perform an unrelated upgrade merely to match a comment.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native --no-sync python -m pytest tests/portability/test_dependency_policy.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Change the core MuJoCo requirement to `mujoco==3.12.0`, retain that native-group pin, and pin NumPy to the verified baseline in the native group. Preserve SciPy 1.17.0, DuckDB 1.5.6, Arrow 26.0.0 and all other existing constraints unless a demonstrated resolver conflict requires a separately reviewed adjustment. Add threadpoolctl to the native group. Keep nanobind/hypothesis development requirements. Do not raise Python's declared floor.

Use an isolated candidate environment for each Python/platform lane:

```sh
mkdir -p output/portability
uv lock
git diff -- uv.lock
uv lock --check
uv sync --locked --group native --group dev
uv pip check
uv export --locked --group native --group dev --no-emit-project -o output/portability/native-requirements.txt
```

The shown `uv lock` is the one intentional lock update after editing pyproject.toml, not a build-time action. Compare its diff and reject unrelated upgrades. `uv pip check` does not require installing pip as an application dependency. Retain both Linux and macOS resolution branches in the universal lock. All native bootstrap/build/test/coverage scripts use the same frozen native group.

Verify complete compatible wheel availability on native target hosts, including transitive packages, not just MuJoCo. Use the exported, reviewed requirements with an explicit temporary pip packaging helper:

```sh
mkdir -p output/portability/wheelhouse
uv run --frozen --group native --with pip python -m pip download --only-binary=:all: -r output/portability/native-requirements.txt --dest output/portability/wheelhouse
```

Resolve the complete dependency intersection before assigning glibc/macOS floors. Never claim a manylinux tag makes Apple-built bike_native portable. The pip helper is a build/packaging tool, not a new application requirement.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native --no-sync python -m pytest tests/portability/test_dependency_policy.py -q
```


Run the new policy tests and existing artifact dependency tests. On Python 3.13 and the recovered macOS interpreter, resolve, sync and import NumPy/SciPy/MuJoCo/Numba/DuckDB/Arrow/nanobind. Record versions and exact wheel filenames/hashes. Re-run a plain `uv run` followed by the documented native command and ensure the native group either remains correct or is restored explicitly before loading; mismatch must fail clearly, never import a stale artifact.

- [ ] **Step 5 - Review and commit.**

```sh
git add pyproject.toml uv.lock src/bike_sim/native/artifact.py tests/portability/test_dependency_policy.py docs/development/dependency-matrix.md
git commit -m "build: pin the native scientific environment consistently"
```

**Review gate:** No unreviewed scientific upgrade or lower macOS support floor. Both required Python lanes resolve; a package missing from a target blocks that target qualification with an exact resolver report.

### T03: Add a cross-platform explicit development driver

**Files:** Create `tools/dev.py`, `tests/portability/test_dev_commands.py`; modify `README.md` only to reference the new development command entry point.

**Interfaces:** Produce `build_commands(root: Path, build: Path, config: str, compiler: str|None) -> list[list[str]]` and the bootstrap/configure/build/doctor/test CLI described by the spec. Existing run_tests.sh remains the implementation of profile verification.

**Reason and boundaries:** Users need one source-build workflow; hiding compilation in module imports would break existing fail-closed loader behavior. The command driver must preserve argument boundaries for paths with spaces.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from pathlib import Path
from tools.dev import build_commands

def test_configure_uses_one_interpreter_and_preserves_path_tokens(tmp_path):
    root = tmp_path / "source tree"
    build = root / "native/build/release"
    commands = build_commands(root, build, "Release", "/opt/llvm tools/clang++")
    configure = commands[0]
    assert configure[:5] == ["uv", "run", "--frozen", "--group", "native"]
    assert str(build) in configure
    assert "-DCMAKE_CXX_COMPILER=/opt/llvm tools/clang++" in configure
    assert all(isinstance(token, str) for token in configure)
```

Add tests for an unsupported config, relative external build selector, a subprocess failure exit code, and a missing lock. No command should use shell=True.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_dev_commands.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

The command construction core is deliberately small:

```python
from pathlib import Path

PREFIX = ["uv", "run", "--frozen", "--group", "native", "--no-sync"]

def build_commands(root: Path, build: Path, config: str,
                   compiler: str | None) -> list[list[str]]:
    if config not in {"Debug", "Release"}:
        raise ValueError("config must be Debug or Release")
    if not root.is_absolute() or not build.is_absolute():
        raise ValueError("root and build must be absolute paths")
    configure = [*PREFIX, "cmake", "-S", str(root / "native"),
                 "-B", str(build), f"-DCMAKE_BUILD_TYPE={config}"]
    if compiler is not None:
        configure.append(f"-DCMAKE_CXX_COMPILER={compiler}")
    return [configure, [*PREFIX, "cmake", "--build", str(build), "--parallel"]]
```

Resolve relative CLI `--build` values against the checkout before calling this function. `bootstrap` checks the lock and performs an explicit `uv sync --locked --group native --group dev`; offline mode adds `--offline` and uses only the specified local wheelhouse through uv's supported index/find-links controls. It never downloads OS compilers. Run bootstrap before configure/build. `configure` and `build` use `uv run --frozen --no-sync`, never sync or relock implicitly; CMake is run through the selected project interpreter environment. `test` sets both BIKE_NATIVE_BUILD_PATH and NATIVE_TEST_BUILD_PATH to one absolute build path, then invokes `bash tools/run_tests.sh PROFILE`. Keep user-provided sanitizer/tool environment variables intact.

Implement doctor as read-only named checks for interpreter, lock, packages, compilers, dependency probe, selected artifact and numerical profile. The dependency/numerical checks become live when T05/T06 land; before then report an unavailable feature as a failure when requested, not a passing stub. Keep structured stdout separate from human diagnostics on stderr.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_dev_commands.py -q
uv run --frozen --group native python tools/dev.py --help
uv run --frozen --group native --no-sync python tools/dev.py configure --build native/build/release --config Release
```

Before T10, Linux's last command may reproduce the known configure blocker. After T10 it must configure and the paired build command must compile. Verify execution from a checkout path containing spaces and from a different current working directory.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/dev.py tests/portability/test_dev_commands.py README.md
git commit -m "feat: add explicit cross-platform development commands"
```

**Review gate:** No hidden compile or lock mutation, correct subprocess exit propagation, and no duplicated test-profile logic. The documentation distinguishes unit-tested orchestration from later native integration qualification.
