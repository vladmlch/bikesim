#!/usr/bin/env python3
"""Qualified CI lane runner for the Linux/macOS portability matrix (T28).

This module is a *thin lane runner*: it automates only the commands that are
already the qualified entry points — ``tools/dev.py`` (bootstrap, configure,
build, doctor) and ``tools/run_tests.sh`` (quick/native/full profiles),
``tools/native_coverage.sh`` for the coverage lanes, and the checked-in
offline bundle/installer tooling for the trusted offline lanes. It never
invents a second build process, never runs ``uv lock``, and never performs an
unbounded dependency upgrade.

Contract:

    lane_commands(lane: str, root: Path) -> list[list[str]]

returns the lane's argv arrays (token boundaries preserved, no shell) and

    python3 tools/ci_portability.py --lane NAME

executes them in order with ``cwd=root``; the first nonzero command exit
stops the lane and becomes the process exit code — a missing tool, a failed
probe, or a failed test command is a failed gate, never continue-on-error.

Lane model:

* Every lane first asserts the host's ``platform.system()`` /
  ``platform.machine()`` against the lane's recorded expectation, then
  resolves every required tool to a real host path and records its
  ``--version`` banner into ``output/ci/<lane>/toolchain.json``.
* ``--configure`` resolves the lane's declared compiler on the host (explicit
  ``CXX`` wins over the lane's candidate spellings) and passes the resolved
  absolute path to ``tools/dev.py configure --compiler`` — or, for the
  sanitizer lanes, to the documented direct ``cmake -S native -B <build>
  -DNATIVE_*=ON -DCMAKE_CXX_COMPILER=<resolved>`` configure under the same
  ``uv run --frozen --group native`` prefix, mirroring dev.py's
  ``-DPython_EXECUTABLE`` pin so the build binds the project interpreter.
* Each lane builds in its own ``native/build/<lane>`` directory and selects
  it for the test profile through ``NATIVE_TEST_BUILD_PATH`` /
  ``BIKE_NATIVE_BUILD_PATH`` (with the legacy ``NATIVE_TEST_BUILD_DIR``
  selector unset, matching ``tools/native_coverage.sh``).
* Trusted lanes (release gates on the recovered baseline hosts, offline
  bundle/install rehearsals, manual GUI acceptance) assert their qualified
  machine set instead of a hosted runner image and are provisioned
  externally — the lane verifies, never improvises, the host toolchain.

Python compatibility: the ``--assert-platform``/``--check-tools`` /
``--provision`` steps run under the runner's bare ``python3`` before the uv
environment exists, so this file uses only the standard library. The shared
``tools/dev.py`` ``PREFIX`` import pulls in ``tomllib``, so the runner's
``python3`` must be CPython 3.11+ — ubuntu-24.04 ships 3.12 and the macos-15
image provides 3.13, so both hosted lanes satisfy it before bootstrap pins
the project interpreter.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
import tarfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]

try:  # imported as tools.ci_portability by the test suite
    from tools import dev as _dev
except ImportError:  # invoked as `python3 tools/ci_portability.py`
    import dev as _dev  # type: ignore[no-redef]

# The one uv prefix every build-facing command shares. Single-sourced from
# tools/dev.py: frozen lock, native group, no implicit environment mutation.
PREFIX: tuple[str, ...] = tuple(_dev.PREFIX)

SCHEMA_VERSION = 1
LATEST_COVERAGE_RUNS = "native_coverage_runs"

_MACHINE_ALIASES = {"amd64": "x86_64", "arm64": "arm64", "aarch64": "aarch64"}


@dataclass(frozen=True)
class ToolSpec:
    """One required host tool: a logical name plus candidate spellings.

    Candidates are resolved in order: absolute paths are checked directly on
    the filesystem, bare spellings through ``shutil.which``. The first
    executable match is recorded with its ``--version`` evidence.
    """

    name: str
    candidates: tuple[str, ...]


@dataclass(frozen=True)
class LaneSpec:
    name: str
    system: str                       # platform.system() value
    machines: tuple[str, ...]         # accepted platform.machine() values
    kind: str                         # native|coverage|offline
    profile: str | None               # quick|native|full run through run_tests.sh
    config: str | None                # CMAKE_BUILD_TYPE for building lanes
    python_minor: str                 # pinned interpreter minor for bootstrap
    compiler: ToolSpec | None         # declared C++ compiler candidates
    cmake_defines: tuple[str, ...]    # extra -D flags on the qualified configure
    required_tools: tuple[ToolSpec, ...]
    runner: str                       # hosted label or trusted runner class
    trusted: bool                     # True → protected/manual host, never PR code
    provision: str | None             # "apt"|"brew" hosted provisioning, None=pre-provisioned
    offline_target: str | None = None
    notes: str = ""

    @property
    def is_native_build(self) -> bool:
        return self.kind == "native"


def _tool(name: str, *candidates: str) -> ToolSpec:
    return ToolSpec(name, tuple(candidates) if candidates else (name,))


# Required tool evidence for every lane that runs the native profile: the
# six sweeps (frontends/g++, tidy/clang-tidy, analyzer/clang++, cppcheck,
# odr/g++, scripts/shellcheck) plus the build generator set. Resolution
# follows the same spellings tools/native_toolchain.py accepts; the sweep
# controller itself remains the authoritative qualification.
_LINUX_ANALYSIS_TOOLS: tuple[ToolSpec, ...] = (
    _tool("python3"),
    _tool("uv"),
    _tool("bash"),
    _tool("cmake"),
    _tool("ctest"),
    _tool("ninja"),
    _tool("clang-tidy", "clang-tidy", "clang-tidy-21", "clang-tidy-20",
          "clang-tidy-19", "clang-tidy-18"),
    _tool("analyzer-clang++", "clang++", "clang++-21", "clang++-20",
          "clang++-19", "clang++-18"),
    _tool("cppcheck"),
    _tool("g++", "g++", "g++-15", "g++-14", "g++-13"),
    _tool("shellcheck"),
)

# Darwin reaches the upstream-LLVM analysis tools through the Homebrew prefix
# (AppleClang ships no clang-tidy); "g++" is the GNU frontend the
# frontends/odr sweeps require, not the clang alias — the versioned Homebrew
# spellings are tried first for that reason.
_DARWIN_ANALYSIS_TOOLS: tuple[ToolSpec, ...] = (
    _tool("python3"),
    _tool("uv"),
    _tool("bash"),
    _tool("cmake"),
    _tool("ctest"),
    _tool("ninja"),
    _tool("clang-tidy", "/opt/homebrew/opt/llvm/bin/clang-tidy",
          "/usr/local/opt/llvm/bin/clang-tidy", "clang-tidy"),
    _tool("analyzer-clang++", "/opt/homebrew/opt/llvm/bin/clang++",
          "/usr/local/opt/llvm/bin/clang++", "clang++"),
    _tool("cppcheck", "/opt/homebrew/bin/cppcheck", "/usr/local/bin/cppcheck",
          "cppcheck"),
    _tool("g++", "g++-16", "g++-15", "g++-14", "g++-13",
          "/opt/homebrew/bin/g++-16", "/usr/local/bin/g++-16", "g++"),
    _tool("shellcheck", "/opt/homebrew/bin/shellcheck",
          "/usr/local/bin/shellcheck", "shellcheck"),
    _tool("xcrun", "xcrun"),  # Apple SDK/mjpython toolchain evidence
)

_LINUX_COVERAGE_TOOLS: tuple[ToolSpec, ...] = (
    _tool("llvm-profdata", "llvm-profdata", "llvm-profdata-21",
          "llvm-profdata-20", "llvm-profdata-19", "llvm-profdata-18"),
    _tool("llvm-cov", "llvm-cov", "llvm-cov-21", "llvm-cov-20",
          "llvm-cov-19", "llvm-cov-18"),
)
_DARWIN_COVERAGE_TOOLS: tuple[ToolSpec, ...] = (
    _tool("llvm-profdata", "/opt/homebrew/opt/llvm/bin/llvm-profdata",
          "/usr/local/opt/llvm/bin/llvm-profdata", "llvm-profdata"),
    _tool("llvm-cov", "/opt/homebrew/opt/llvm/bin/llvm-cov",
          "/usr/local/opt/llvm/bin/llvm-cov", "llvm-cov"),
)

_OFFLINE_TOOLS: tuple[ToolSpec, ...] = (
    _tool("python3"),
    _tool("uv"),
    _tool("bash"),
    _tool("cmake"),
    _tool("ninja"),
    _tool("c++", "c++", "clang++", "g++"),  # native-source bundle installer probe
    _tool("tar"),
)

_CLANG = ("clang++", "clang++-21", "clang++-20", "clang++-19", "clang++-18")
_GCC = ("g++", "g++-15", "g++-14", "g++-13")
# RTSan has no GCC spelling and AppleClang rejects -fsanitize=realtime; the
# lane needs an upstream LLVM clang — the documented brew prefix on Darwin
# and a provisioned LLVM on Linux. Configure probes the flag and fails
# closed, so an unqualified host is a red gate, not a skip.
_DARWIN_BREW_CLANG = ("/opt/homebrew/opt/llvm/bin/clang++",
                    "/usr/local/opt/llvm/bin/clang++", "clang++")
_LINUX_RTSAN_CLANG = ("clang++-21", "clang++-20", "clang++-19", "clang++")
_APPLECLANG = ("clang++",)  # /usr/bin/clang++ — the AppleClang driver

_APT_PROVISION = (
    "clang", "clang-tidy", "llvm", "g++", "gcc", "cppcheck", "shellcheck",
    "cmake", "ninja-build", "make", "pkg-config",
)
_BREW_PROVISION = ("llvm", "gcc", "cppcheck", "shellcheck", "ninja", "cmake")

# Hosted lanes assert the exact runner architecture (ubuntu-24.04 → x86_64,
# macos-15 → arm64). Trusted lanes accept every qualified machine measured
# for this repo: the Linux baseline was collected on Ubuntu 24.04.5 aarch64
# and the release-blocking lane is Ubuntu 24.04 x86_64; macOS is arm64 only
# (macOS Intel is not qualified).
LANES: dict[str, LaneSpec] = {}


def _lane(**kwargs: Any) -> LaneSpec:
    spec = LaneSpec(**kwargs)
    LANES[spec.name] = spec
    return spec


_lane(
    name="linux-clang-release", system="Linux", machines=("x86_64",),
    kind="native", profile="native", config="Release", python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
)
_lane(
    name="linux-gcc-debug", system="Linux", machines=("x86_64",),
    kind="native", profile="native", config="Debug", python_minor="3.13",
    compiler=ToolSpec("g++", _GCC), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
)
_lane(
    name="macos-appleclang-release", system="Darwin", machines=("arm64",),
    kind="native", profile="native", config="Release", python_minor="3.14",
    compiler=ToolSpec("AppleClang clang++", _APPLECLANG), cmake_defines=(),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="macos-15", trusted=False, provision="brew",
    notes="default CMake SDK resolution; AppleClang + Accelerate baseline",
)
_lane(
    name="linux-asan", system="Linux", machines=("x86_64",),
    kind="native", profile="native", config="Debug", python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG),
    cmake_defines=("-DNATIVE_SANITIZE=ON",),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
)
_lane(
    name="macos-asan", system="Darwin", machines=("arm64",),
    kind="native", profile="native", config="Debug", python_minor="3.14",
    compiler=ToolSpec("AppleClang clang++", _APPLECLANG),
    cmake_defines=("-DNATIVE_SANITIZE=ON",),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="macos-15", trusted=False, provision="brew",
    notes="clang ASan set address,undefined,local-bounds,float-cast-overflow "
          "with -fno-sanitize-recover=all, verified arm64-apple",
)
_lane(
    name="linux-rtsan", system="Linux", machines=("x86_64",),
    kind="native", profile="native", config="Release", python_minor="3.13",
    compiler=ToolSpec("upstream clang++ (RTSan)", _LINUX_RTSAN_CLANG),
    cmake_defines=("-DNATIVE_RTSAN=ON",),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
    notes="requires an upstream LLVM clang accepting -fsanitize=realtime "
          "(LLVM >= 20 on Linux); the capability probe fails closed on a "
          "runner whose distro toolchain lacks it",
)
_lane(
    name="macos-rtsan", system="Darwin", machines=("arm64",),
    kind="native", profile="native", config="Release", python_minor="3.14",
    compiler=ToolSpec("brew LLVM clang++ (RTSan)", _DARWIN_BREW_CLANG),
    cmake_defines=("-DNATIVE_RTSAN=ON",),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="macos-15", trusted=False, provision="brew",
    notes="AppleClang rejects -fsanitize=realtime; the brew LLVM clang is "
          "the documented RTSan toolchain",
)
_lane(
    name="linux-coverage", system="Linux", machines=("x86_64",),
    kind="coverage", profile=None, config=None, python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS + _LINUX_COVERAGE_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
    notes="drives tools/native_coverage.sh end-to-end with CXX set to the "
          "resolved compiler",
)
_lane(
    name="macos-coverage", system="Darwin", machines=("arm64",),
    kind="coverage", profile=None, config=None, python_minor="3.14",
    compiler=ToolSpec("clang++", ("clang++",)), cmake_defines=(),
    required_tools=_DARWIN_ANALYSIS_TOOLS + _DARWIN_COVERAGE_TOOLS,
    runner="macos-15", trusted=False, provision="brew",
    notes="drives tools/native_coverage.sh end-to-end with CXX set to the "
          "resolved compiler",
)
_lane(
    name="linux-full", system="Linux", machines=("x86_64",),
    kind="native", profile="full", config="Release", python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="ubuntu-24.04", trusted=False, provision="apt",
    notes="full profile including slow episodes; Linux required gates are "
          "pass, not inherited baseline_unchanged",
)
_lane(
    name="macos-full", system="Darwin", machines=("arm64",),
    kind="native", profile="full", config="Release", python_minor="3.14",
    compiler=ToolSpec("AppleClang clang++", _APPLECLANG), cmake_defines=(),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="macos-15", trusted=False, provision="brew",
    notes="the documented 24-failure macOS baseline is classified by "
          "tools/portability_qualification.py (T27); the lane reports the "
          "raw profile exit code verbatim",
)
_lane(
    name="linux-release", system="Linux", machines=("x86_64", "aarch64"),
    kind="native", profile="native", config="Release", python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="self-hosted/linux/bike-qualified", trusted=True, provision=None,
    notes="trusted release gate on a qualified Linux host (x86_64 or the "
          "aarch64 baseline container), never on untrusted PR code",
)
_lane(
    name="macos-release", system="Darwin", machines=("arm64",),
    kind="native", profile="native", config="Release", python_minor="3.14",
    compiler=ToolSpec("AppleClang clang++", _APPLECLANG), cmake_defines=(),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="self-hosted/macos/bike-qualified", trusted=True, provision=None,
    notes="trusted release gate on the recovered macOS baseline environment "
          "(macOS 27.0.1 arm64, AppleClang 21.0.0, Xcode 26.6)",
)
_lane(
    name="linux-offline", system="Linux", machines=("x86_64", "aarch64"),
    kind="offline", profile=None, config=None, python_minor="3.13",
    compiler=None, cmake_defines=(), required_tools=_OFFLINE_TOOLS,
    runner="self-hosted/linux/bike-qualified", trusted=True, provision=None,
    offline_target="host",
    notes="assembles host full/slim/native-source bundles with "
          "tools/offline_bundle.py and rehearses the checked-in "
          "packaging/install_full.sh + install_slim.sh installers",
)
_lane(
    name="macos-offline", system="Darwin", machines=("arm64",),
    kind="offline", profile=None, config=None, python_minor="3.14",
    compiler=None, cmake_defines=(), required_tools=_OFFLINE_TOOLS,
    runner="self-hosted/macos/bike-qualified", trusted=True, provision=None,
    offline_target="host",
    notes="assembles host full/slim/native-source bundles with "
          "tools/offline_bundle.py and rehearses the checked-in "
          "packaging/install_full.sh + install_slim.sh installers",
)
_lane(
    name="linux-manual-gui", system="Linux", machines=("x86_64", "aarch64"),
    kind="native", profile="native", config="Release", python_minor="3.13",
    compiler=ToolSpec("clang++", _CLANG), cmake_defines=(),
    required_tools=_LINUX_ANALYSIS_TOOLS,
    runner="self-hosted/linux/bike-qualified", trusted=True, provision=None,
    notes="manual GUI acceptance lane; the interactive viewer/replay checks "
          "are operator evidence (T19/T29), the lane covers the headless "
          "native gate plus the viewer-launch platform controls",
)
_lane(
    name="macos-manual-gui", system="Darwin", machines=("arm64",),
    kind="native", profile="native", config="Release", python_minor="3.14",
    compiler=ToolSpec("AppleClang clang++", _APPLECLANG), cmake_defines=(),
    required_tools=_DARWIN_ANALYSIS_TOOLS,
    runner="self-hosted/macos/bike-qualified", trusted=True, provision=None,
    notes="manual GUI acceptance lane retaining the mjpython/SDK/Accelerate "
          "expectations; interactive evidence remains an operator gate",
)

HOSTED_LANES = tuple(name for name, spec in LANES.items() if not spec.trusted)
TRUSTED_LANES = tuple(name for name, spec in LANES.items() if spec.trusted)


def lane_spec(lane: str) -> LaneSpec:
    try:
        return LANES[lane]
    except KeyError:
        raise ValueError(
            f"unsupported lane {lane!r}; declared lanes: {sorted(LANES)}"
        ) from None


def build_path(root: Path, spec: LaneSpec) -> Path:
    """The lane's unique build directory under the checkout."""
    return root / "native" / "build" / spec.name


def evidence_dir(root: Path, spec: LaneSpec) -> Path:
    return root / "output" / "ci" / spec.name


def _self(root: Path, *args: str) -> list[str]:
    return ["python3", str(root / "tools" / "ci_portability.py"), *args]


def _dev_command(root: Path, *args: str) -> list[str]:
    return [*PREFIX, "python", str(root / "tools" / "dev.py"), *args]


def _test_command(root: Path, build: Path, profile: str) -> list[str]:
    """Select the lane build for run_tests.sh via env(1) assignment argv.

    Both selector names are set to the same absolute directory and the legacy
    NATIVE_TEST_BUILD_DIR is unset — the loader fails closed on contradictory
    selectors, so the lane must not inherit an ambient one.
    """
    return [
        "env", "-u", "NATIVE_TEST_BUILD_DIR",
        f"NATIVE_TEST_BUILD_PATH={build}",
        f"BIKE_NATIVE_BUILD_PATH={build}",
        "bash", str(root / "tools" / "run_tests.sh"), profile,
    ]


def lane_commands(lane: str, root: Path) -> list[list[str]]:
    """The lane's ordered argv arrays; nothing here executes a subprocess.

    ``root`` is the checkout the commands run against; it must be absolute so
    the generated argv stays correct regardless of the caller's cwd.
    """
    spec = lane_spec(lane)
    root = Path(root)
    if not root.is_absolute():
        raise ValueError(f"root must be an absolute path, got {root!r}")

    commands: list[list[str]] = [_self(root, "--assert-platform", spec.name)]
    if spec.provision is not None:
        commands.append(_self(root, "--provision", spec.name))
    commands.append(_self(root, "--check-tools", spec.name))
    commands.append(_dev_command(
        root, "bootstrap", "--python", spec.python_minor))

    if spec.kind == "coverage":
        assert spec.compiler is not None
        build = build_path(root, spec)
        commands.append(_self(
            root, "--run-coverage", spec.name,
            "--compiler", spec.compiler.candidates[0]))
        return commands

    if spec.kind == "offline":
        bundle_out = root / "output" / "offline-bundles" / spec.name
        bundle_tool = str(root / "tools" / "offline_bundle.py")
        for extra in (["--mode", "full"], ["--mode", "slim"],
                      ["--mode", "full", "--native"]):
            commands.append([
                *PREFIX, "python", bundle_tool,
                "--target", spec.offline_target or "host",
                "--python", spec.python_minor,
                *extra,
                "--output", str(bundle_out),
            ])
        commands.append(_self(root, "--offline-install", spec.name))
        return commands

    # kind == "native" (release/debug, sanitizer, full, release-gate, manual)
    assert spec.config is not None and spec.profile is not None and spec.compiler
    build = build_path(root, spec)
    commands.append(_self(
        root, "--configure", spec.name,
        "--compiler", spec.compiler.candidates[0]))
    commands.append(_dev_command(root, "build", "--build", str(build)))
    commands.append(_dev_command(
        root, "doctor", "--build", str(build), "--numerics"))
    commands.append(_test_command(root, build, spec.profile))
    if spec.name.endswith("-manual-gui"):
        commands.append([
            *PREFIX, "python", "-m", "pytest",
            "tests/portability/test_viewer_launch_platform.py", "-q",
        ])
    return commands


# ---------------------------------------------------------------------------
# Lane steps — the only parts that execute on the host.
# ---------------------------------------------------------------------------


def _normalize_machine(value: str) -> str:
    lowered = value.strip().lower()
    return _MACHINE_ALIASES.get(lowered, lowered)


def _platform_record() -> dict[str, str]:
    return {
        "system": platform.system(),
        "machine": platform.machine(),
        "release": platform.release(),
        "version": platform.version(),
        "python": sys.version.split()[0],
        "python_executable": sys.executable,
    }


def _assert_platform(spec: LaneSpec) -> int:
    record = _platform_record()
    actual_system = record["system"]
    actual_machine = _normalize_machine(record["machine"])
    expected = {_normalize_machine(m) for m in spec.machines}
    problems = []
    if actual_system != spec.system:
        problems.append(f"system={actual_system!r} expected {spec.system!r}")
    if actual_machine not in expected:
        problems.append(f"machine={actual_machine!r} expected one of {sorted(expected)!r}")
    print(f"lane {spec.name}: platform {json.dumps(record, sort_keys=True)}")
    if problems:
        print(f"lane {spec.name}: platform mismatch: {'; '.join(problems)}",
              file=sys.stderr)
        return 1
    print(f"lane {spec.name}: platform assertion passed")
    return 0


def _resolve_candidate(candidate: str) -> Path | None:
    if Path(candidate).is_absolute():
        path = Path(candidate)
        return path.absolute() if path.is_file() and os.access(path, os.X_OK) else None
    located = shutil.which(candidate)
    if located is None:
        return None
    path = Path(located)
    return path.absolute() if path.is_file() and os.access(path, os.X_OK) else None


def _version_evidence(executable: Path) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            [str(executable), "--version"], capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=60, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"argv": [str(executable), "--version"], "returncode": None,
                "error": str(error), "first_line": ""}
    output = (completed.stdout or completed.stderr).strip()
    return {
        "argv": [str(executable), "--version"],
        "returncode": completed.returncode,
        "first_line": output.splitlines()[0] if output else "",
    }


def resolve_tool(tool: ToolSpec) -> Path | None:
    """First executable candidate for one required tool, in declared order."""
    for candidate in tool.candidates:
        resolved = _resolve_candidate(candidate)
        if resolved is not None:
            return resolved
    return None


def _check_tools(spec: LaneSpec, root: Path) -> int:
    records: list[dict[str, Any]] = []
    missing: list[str] = []
    for tool in spec.required_tools:
        resolved = resolve_tool(tool)
        record: dict[str, Any] = {
            "name": tool.name,
            "candidates": list(tool.candidates),
            "resolved": str(resolved) if resolved is not None else None,
        }
        if resolved is None:
            missing.append(tool.name)
        else:
            record["version"] = _version_evidence(resolved)
        records.append(record)
    compiler_record: dict[str, Any] | None = None
    if spec.compiler is not None:
        resolved = resolve_tool(spec.compiler)
        compiler_record = {
            "declared": spec.compiler.name,
            "candidates": list(spec.compiler.candidates),
            "resolved": str(resolved) if resolved is not None else None,
        }
        if resolved is None:
            missing.append(f"compiler:{spec.compiler.name}")
        else:
            compiler_record["version"] = _version_evidence(resolved)
    destination = evidence_dir(root, spec)
    destination.mkdir(parents=True, exist_ok=True)
    document = {
        "schema_version": SCHEMA_VERSION,
        "lane": spec.name,
        "platform": _platform_record(),
        "tools": records,
        "compiler": compiler_record,
        "missing": missing,
        "status": "pass" if not missing else "fail",
    }
    report_path = destination / "toolchain.json"
    report_path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    print(f"lane {spec.name}: toolchain evidence written to {report_path}")
    for record in records:
        print(f"  {record['name']}: {record['resolved'] or 'MISSING'}")
    if compiler_record is not None:
        print(f"  compiler {compiler_record['declared']}: "
              f"{compiler_record['resolved'] or 'MISSING'}")
    if missing:
        print(f"lane {spec.name}: required tools missing: {', '.join(missing)}",
              file=sys.stderr)
        return 1
    return 0


def _resolve_compiler(spec: LaneSpec, preferred: str | None = None) -> Path:
    """Resolve the lane compiler on this host.

    An explicit ``CXX`` environment override wins. Otherwise the declared
    spelling (the ``--compiler`` token visible in the lane's argv) is tried
    first and the lane's remaining candidate spellings after it — the
    declared compiler is a requirement, the candidates are its qualified
    spellings. An unresolvable set is a hard error, never a silent default.
    """
    if spec.compiler is None:
        raise ValueError(f"lane {spec.name} declares no compiler")
    override = os.environ.get("CXX")
    if override:
        resolved = _resolve_candidate(override)
        if resolved is None:
            raise ValueError(f"CXX override {override!r} is not executable")
        print(f"lane {spec.name}: compiler from CXX={resolved}")
        return resolved
    spellings = [preferred, *spec.compiler.candidates] if preferred else \
        list(spec.compiler.candidates)
    checked: list[str] = []
    for spelling in spellings:
        if spelling in checked:
            continue
        checked.append(spelling)
        resolved = _resolve_candidate(spelling)
        if resolved is not None:
            print(f"lane {spec.name}: compiler {spec.compiler.name} -> {resolved}")
            return resolved
    raise ValueError(
        f"lane {spec.name}: compiler {spec.compiler.name!r} unresolved; "
        f"checked {checked}"
    )


def _project_python(root: Path) -> Path:
    return root / ".venv" / "bin" / "python"


def _configure(spec: LaneSpec, root: Path, requested: str | None) -> int:
    build = build_path(root, spec)
    try:
        compiler = _resolve_compiler(spec, preferred=requested)
    except ValueError as error:
        print(f"lane {spec.name}: {error}", file=sys.stderr)
        return 2
    if not spec.cmake_defines:
        argv = _dev_command(
            root, "configure", "--build", str(build),
            "--config", spec.config or "Release", "--compiler", str(compiler))
    else:
        # Sanitizer lanes use the documented direct configure (TESTING.md):
        # dev.py's configure has no -D passthrough, so the argv mirrors its
        # build_commands() output exactly — same uv prefix, source/build
        # dirs, build type, compiler and project-interpreter pin.
        argv = [
            *PREFIX, "cmake",
            "-S", str(root / "native"),
            "-B", str(build),
            f"-DCMAKE_BUILD_TYPE={spec.config or 'Release'}",
            f"-DCMAKE_CXX_COMPILER={compiler}",
            f"-DPython_EXECUTABLE={_project_python(root)}",
            *spec.cmake_defines,
        ]
    return _run(argv, cwd=root)


def _run_coverage(spec: LaneSpec, root: Path, requested: str | None) -> int:
    build = build_path(root, spec)
    try:
        compiler = _resolve_compiler(spec, preferred=requested)
    except ValueError as error:
        print(f"lane {spec.name}: {error}", file=sys.stderr)
        return 2
    # The coverage driver configures itself; CXX is the standard explicit
    # compiler declaration CMake honors at first configure.
    env = dict(os.environ)
    env["CXX"] = str(compiler)
    argv = ["bash", str(root / "tools" / "native_coverage.sh"), str(build)]
    return _run(argv, cwd=root, env=env)


def _provision(spec: LaneSpec, root: Path) -> int:
    """Hosted-runner provisioning: only the checked-in toolchain set."""
    if spec.provision is None:
        print(f"lane {spec.name}: trusted lane — host is pre-provisioned, "
              "nothing to install")
        return 0
    if spec.provision == "apt":
        steps = [
            ["sudo", "-n", "apt-get", "update"],
            ["sudo", "-n", "apt-get", "install", "-y", "--no-install-recommends",
             *_APT_PROVISION],
        ]
        env = dict(os.environ)
        env["DEBIAN_FRONTEND"] = "noninteractive"
        for argv in steps:
            status = _run(argv, cwd=root, env=env)
            if status != 0:
                return status
        return 0
    if spec.provision == "brew":
        return _run(["brew", "install", *_BREW_PROVISION], cwd=root)
    print(f"lane {spec.name}: unknown provision mode {spec.provision!r}",
          file=sys.stderr)
    return 2


def _extract_safely(archive: Path, destination: Path) -> Path:
    """Extract a self-produced bundle tarball under destination; confine members."""
    destination.mkdir(parents=True, exist_ok=True)
    base = destination.resolve()
    top: str | None = None
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle.getmembers():
            member_path = (base / member.name).resolve()
            if not str(member_path).startswith(str(base) + os.sep):
                raise ValueError(f"unsafe archive member {member.name!r} in {archive}")
            if member.issym() or member.islnk():
                link_target = (member_path.parent / member.linkname).resolve()
                if not str(link_target).startswith(str(base) + os.sep):
                    raise ValueError(
                        f"unsafe archive link {member.name!r} -> {member.linkname!r}")
        bundle.extractall(base)
        names = {name.split("/", 1)[0] for name in bundle.getnames() if name}
        if len(names) == 1:
            top = next(iter(names))
    if top is None or not (base / top).is_dir():
        raise ValueError(f"archive {archive} has no single bundle root")
    return base / top


def _bundle_mode(bundle_root: Path, archive: Path) -> str:
    manifest_path = bundle_root / "manifest.json"
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            manifest = {}
        mode = manifest.get("mode")
        if mode in ("full", "slim"):
            return mode
    name = archive.name
    if "-slim" in name:
        return "slim"
    return "full"


def _offline_install(spec: LaneSpec, root: Path) -> int:
    """Rehearse the checked-in installers against the lane's fresh bundles."""
    bundle_dir = root / "output" / "offline-bundles" / spec.name
    archives = sorted(bundle_dir.glob("*.tar.gz"))
    if not archives:
        print(f"lane {spec.name}: no bundle archives in {bundle_dir}",
              file=sys.stderr)
        return 1
    work = evidence_dir(root, spec)
    extracted_root = work / "extracted"
    python = _project_python(root)
    env = dict(os.environ)
    env["PYTHON"] = str(python)  # the installer's own interpreter
    for archive in archives:
        stem = archive.name[: -len(".tar.gz")]
        try:
            bundle_root = _extract_safely(archive, extracted_root / stem)
            mode = _bundle_mode(bundle_root, archive)
        except (OSError, tarfile.TarError, ValueError) as error:
            print(f"lane {spec.name}: cannot unpack {archive}: {error}",
                  file=sys.stderr)
            return 1
        installer = bundle_root / "project" / "packaging" / f"install_{mode}.sh"
        if not installer.is_file():
            print(f"lane {spec.name}: bundle {archive.name} lacks {installer.name}",
                  file=sys.stderr)
            return 1
        argv = ["bash", str(installer), str(work / f"install-{stem}"),
                "--python", str(python)]
        status = _run(argv, cwd=root, env=env)
        if status != 0:
            return status
    return 0


def _run(argv: Sequence[str], *, cwd: Path,
         env: Mapping[str, str] | None = None) -> int:
    """The single spawn point: argument array, explicit cwd, status out."""
    command = [str(token) for token in argv]
    print(f"+ {shlex.join(command)}", file=sys.stderr)
    try:
        completed = subprocess.run(command, cwd=cwd, env=env, check=False)
    except OSError as error:
        print(f"ci_portability: cannot execute {command[0]!r}: {error}",
              file=sys.stderr)
        return 127
    return completed.returncode


def _run_lane(spec: LaneSpec, root: Path) -> int:
    commands = lane_commands(spec.name, root)
    print(f"lane {spec.name}: {len(commands)} commands", file=sys.stderr)
    for command in commands:
        status = _run(command, cwd=root)
        if status != 0:
            print(f"lane {spec.name}: command exited {status}; lane failed",
                  file=sys.stderr)
            return status
    print(f"lane {spec.name}: all commands passed", file=sys.stderr)
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ci_portability.py",
        description="Qualified CI lane runner for the Linux/macOS portability "
                    "matrix (tools/dev.py + tools/run_tests.sh only).",
    )
    ops = parser.add_mutually_exclusive_group(required=True)
    ops.add_argument("--lane", metavar="NAME",
                     help="run every command of the named lane")
    ops.add_argument("--list", action="store_true",
                     help="list declared lane names and exit")
    ops.add_argument("--assert-platform", metavar="NAME",
                     help="lane step: assert host system/machine")
    ops.add_argument("--provision", metavar="NAME",
                     help="lane step: install the checked-in hosted toolchain")
    ops.add_argument("--check-tools", metavar="NAME",
                     help="lane step: resolve required tools, record versions")
    ops.add_argument("--configure", metavar="NAME",
                     help="lane step: resolve the compiler and configure")
    ops.add_argument("--run-coverage", metavar="NAME",
                     help="lane step: drive tools/native_coverage.sh")
    ops.add_argument("--offline-install", metavar="NAME",
                     help="lane step: install the lane's offline bundles")
    parser.add_argument("--compiler", metavar="SPELLING",
                        help="declared compiler spelling asserted by "
                             "--configure/--run-coverage")
    return parser


def main(argv: Sequence[str] | None = None, *, root: Path | None = None) -> int:
    root = REPO_ROOT if root is None else Path(root).resolve()
    args = _parser().parse_args(argv)
    if args.list:
        for name in LANES:
            print(name)
        return 0

    name = (args.lane or args.assert_platform or args.provision
            or args.check_tools or args.configure or args.run_coverage
            or args.offline_install)
    try:
        spec = lane_spec(name)
    except ValueError as error:
        print(f"ci_portability: {error}", file=sys.stderr)
        return 2
    try:
        if args.lane:
            return _run_lane(spec, root)
        if args.assert_platform:
            return _assert_platform(spec)
        if args.provision:
            return _provision(spec, root)
        if args.check_tools:
            return _check_tools(spec, root)
        if args.configure:
            return _configure(spec, root, args.compiler)
        if args.run_coverage:
            return _run_coverage(spec, root, args.compiler)
        if args.offline_install:
            return _offline_install(spec, root)
    except (OSError, ValueError) as error:
        print(f"ci_portability: {error}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
