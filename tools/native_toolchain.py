"""Cross-platform native analysis-tool discovery and inspection (T14).

The native check sweeps (``tools/native_checks.py``) resolve every external
analysis tool through the same order on macOS and Linux:

1. an explicit override (``CLANG_TIDY`` / ``ANALYZER_CLANG`` / ``CPPCHECK`` /
   ``GXX`` environment variables, or the context's recorded
   ``tool_overrides``) — an invalid override fails rather than falling
   through to a different tool;
2. a compatible sibling of the configured compiler — ``clang-tidy-<v>`` /
   ``clang++-<v>`` next to an upstream-LLVM ``clang++-<v>``, or ``g++-<v>``
   next to ``g++-<v>`` — so the sweep tool shares the compiler's own
   toolchain view instead of inheriting a fixed prefix or branding;
3. the context's recorded default spelling, resolved through PATH;
4. Darwin-only: the existing Homebrew/Xcode discovery candidates.

Sibling discovery follows the compiler's *family* (upstream LLVM ``Clang``
or ``GNU``), not its directory alone: ``/usr/bin/g++-13`` sits next to
``clang-tidy-18`` on Debian-style systems but they are different toolchains.
AppleClang contributes no siblings — its ``/usr/bin`` toolchain directory
ships no clang-tidy, and its analyzer is a distinct fork from the
upstream-LLVM toolchain the suppression manifest and controls qualify —
so Darwin reaches clang tools through the recorded Homebrew/Xcode
candidates instead.

No shell is ever invoked: resolution is ``shutil.which`` or explicit
filesystem checks, inspection is an argv subprocess, and a resolved
executable keeps its invocation spelling — ``clang++`` may be a symlink
whose basename selects C++ driver behavior.

``inspect_tool`` probes a resolved tool: ``--version`` plus compiler
target/resource queries where supported, recording argv, return codes,
stdout and stderr. Tools that cannot identify themselves, or that identify
as something unrelated to a compiler/analysis toolchain, are rejected.

``resolve_coverage_tools`` resolves the llvm-profdata/llvm-cov pair for a
clang-family compiler in the order the coverage lane requires — an explicit
``LLVM_PROFDATA``/``LLVM_COV`` override, a sibling of the compiler's own
toolchain directory, a PATH candidate, and a Darwin-only ``xcrun --find``
fallback that Linux never invokes — then qualifies the pair by running the
real format probe: the compiler builds a minimal instrumented control, the
control emits a ``.profraw`` under a fresh ``LLVM_PROFILE_FILE``, and the
resolved tools merge and export it. Branding or a shared major version is
not evidence; only a probe that compiles, runs, merges and exports is.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

# Environment-variable names for the four supported explicit overrides,
# keyed by the native_checks sweep kinds that consume them.
TOOL_ENV_OVERRIDES: dict[str, str] = {
    "tidy": "CLANG_TIDY",
    "analyzer": "ANALYZER_CLANG",
    "cppcheck": "CPPCHECK",
    "gxx": "GXX",
    "frontends": "GXX",
    "odr": "GXX",
}

# Sibling lookup: kind -> (required CMAKE_CXX_COMPILER_ID, tool spellings).
# Only the upstream-LLVM "Clang" and "GNU" identities qualify a directory's
# tools as siblings; see the module docstring for the AppleClang rationale.
# The coverage kinds accept every clang-family compiler: the Xcode
# toolchain directory ships llvm-cov/llvm-profdata beside clang++, and an
# upstream clang++-<v> sits beside llvm-cov-<v>/llvm-profdata-<v> on Linux.
_SIBLING_FAMILY: dict[str, tuple[str | None, tuple[str, ...]]] = {
    "tidy": ("Clang", ("clang-tidy",)),
    "analyzer": ("Clang", ("clang++",)),
    "cppcheck": (None, ()),
    "gxx": ("GNU", ("g++",)),
    "frontends": ("GNU", ("g++",)),
    "odr": ("GNU", ("g++",)),
    "llvm-profdata": ("Clang", ("llvm-profdata",)),
    "llvm-cov": ("Clang", ("llvm-cov",)),
}

# The coverage lane's explicit-override environment variables.
COVERAGE_TOOL_OVERRIDES: dict[str, str] = {
    "llvm-profdata": "LLVM_PROFDATA",
    "llvm-cov": "LLVM_COV",
}

# Baseline Darwin discovery candidates appended after the recorded defaults:
# the Homebrew LLVM prefix locations the qualified analysis toolchain is
# installed from on arm64 and x86_64 hosts (the Xcode toolchain itself is
# already reachable through the compiler record's own directory).
_DARWIN_TOOL_CANDIDATES: dict[str, tuple[str, ...]] = {
    "tidy": (
        "/opt/homebrew/opt/llvm/bin/clang-tidy",
        "/usr/local/opt/llvm/bin/clang-tidy",
    ),
    "analyzer": (
        "/opt/homebrew/opt/llvm/bin/clang++",
        "/usr/local/opt/llvm/bin/clang++",
    ),
    "cppcheck": (
        "/opt/homebrew/bin/cppcheck",
        "/usr/local/bin/cppcheck",
    ),
    "gxx": (
        "g++-16",
        "g++-15",
        "g++-14",
        "g++-13",
        "g++",
        "/opt/homebrew/bin/g++-16",
        "/usr/local/bin/g++-16",
    ),
}

# compiler driver spellings: an optional vendor triplet prefix (one or more
# dash-terminated segments), driver stem and an optional trailing -<version>
# ("aarch64-linux-gnu-g++-13", "x86_64-pc-linux-gnu-clang++-18").
_DRIVER_BASENAME = re.compile(
    r"^(?P<prefix>(?:[A-Za-z0-9_.+]+-)+)?"
    r"(?P<stem>clang\+\+|clang|g\+\+|gcc)"
    r"(?:-(?P<version>[0-9]+(?:\.[0-9]+)*))?$"
)

# Version-banner signatures, in match order: clang-tidy identifies as
# "<distro> LLVM version X" which contains no "clang-tidy" token, while a
# plain clang banner always contains "clang". GCC banners carry "(GCC)" or
# the FSF copyright line. Anything else is an unrelated tool.
_FAMILY_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("clang-tidy", re.compile(r"clang-tidy|LLVM version", re.IGNORECASE)),
    ("cppcheck", re.compile(r"cppcheck", re.IGNORECASE)),
    ("clang", re.compile(r"clang", re.IGNORECASE)),
    ("gcc", re.compile(r"\(GCC\)|Free Software Foundation", re.IGNORECASE)),
)


def resolve_tool(name: str, override: str | None, candidates: list[str]) -> Path:
    """Resolve a sweep tool: explicit override wins, then candidates in order.

    ``candidates`` may be absolute paths or PATH spellings. A located tool
    must be an executable file. An explicit override that does not resolve
    is a hard error — never silently fall through to a different tool.
    """
    choices = [override] if override is not None else list(candidates)
    for choice in choices:
        located = shutil.which(choice) if not Path(choice).is_absolute() else choice
        if located and Path(located).is_file() and os.access(located, os.X_OK):
            return Path(located).absolute()
    if override is not None:
        raise ValueError(f"invalid {name} override: {override}")
    raise ValueError(f"required tool {name} was not found; checked {choices!r}")


def sibling_candidates(compiler: Mapping[str, Any], kind: str) -> list[str]:
    """Tool spellings next to the configured compiler, in its own toolchain.

    ``compiler`` is the context compiler record (``path`` plus CMake's
    ``CMAKE_CXX_COMPILER_ID`` as ``id``). Candidates are absolute paths —
    versioned first, then the compiler's own basename — and are returned
    whether or not they exist (``resolve_tool`` performs the executable
    check). Kinds with no compiler-family relationship (``cppcheck``) and
    compilers outside the upstream LLVM/GNU identities produce no siblings.
    """
    family = _SIBLING_FAMILY.get(kind)
    if family is None:
        raise ValueError(f"unknown native sweep kind: {kind}")
    required_id, stems = family
    if not stems or compiler.get("id") != required_id:
        return []
    path = compiler.get("path")
    if not isinstance(path, str) or not path:
        return []
    compiler_path = Path(path)
    match = _DRIVER_BASENAME.match(compiler_path.name)
    if match is None:
        return []
    prefix = match.group("prefix") or ""
    version = match.group("version")
    names: list[str] = []
    for stem in stems:
        spellings: list[str] = []
        if version:
            spellings.append(f"{stem}-{version}")
            spellings.append(f"{prefix}{stem}-{version}")
        spellings.append(stem)
        spellings.append(f"{prefix}{stem}")
        for spelling in spellings:
            if spelling not in names:
                names.append(spelling)
    return [str(compiler_path.parent / name) for name in names]


def darwin_tool_candidates(kind: str) -> list[str]:
    """Darwin-only Homebrew/Xcode discovery candidates for one sweep kind."""
    if kind not in _DARWIN_TOOL_CANDIDATES:
        raise ValueError(f"unknown native sweep kind: {kind}")
    return list(_DARWIN_TOOL_CANDIDATES[kind])


def _query(argv: list[str], timeout: float = 60.0) -> dict[str, Any]:
    """Run one inspection probe as an argv array; return the full record."""
    command = [str(token) for token in argv]
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
            timeout=timeout,
        )
        return {
            "argv": command,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "launch_error": None,
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {
            "argv": command,
            "returncode": None,
            "stdout": "",
            "stderr": "",
            "launch_error": str(error),
        }


def _classify_family(version_text: str) -> str | None:
    for name, pattern in _FAMILY_PATTERNS:
        if pattern.search(version_text):
            return name
    return None


def inspect_tool(path: Path) -> dict:
    """Probe a resolved tool and record identity/capability evidence.

    Raises ValueError when the path is not an executable file, when
    ``--version`` fails or produces no output, or when the banner does not
    identify a recognized compiler/analysis toolchain family. Compiler
    families additionally answer target and resource queries where the
    driver supports them; unsupported queries are recorded, not fatal.
    """
    tool = Path(path)
    if not tool.is_file() or not os.access(tool, os.X_OK):
        raise ValueError(f"cannot inspect tool that is not an executable file: {path}")
    version_query = _query([str(tool), "--version"])
    version_text = (version_query["stdout"] + "\n" + version_query["stderr"]).strip()
    if (
        version_query["launch_error"]
        or version_query["returncode"] != 0
        or not version_text
    ):
        raise ValueError(
            f"tool {tool} did not identify itself (--version failed)"
        )
    family = _classify_family(version_text)
    first_line = version_text.splitlines()[0]
    if family is None:
        raise ValueError(
            f"tool {tool} is not a recognized compiler or analysis tool: {first_line!r}"
        )
    record: dict[str, Any] = {
        "path": str(tool),
        "resolved_path": str(tool.resolve()),
        "family": family,
        "version": first_line,
        "version_query": version_query,
        "target_query": None,
        "target": None,
        "resource_query": None,
        "resource_dir": None,
    }
    if family in {"clang", "gcc"}:
        target_spellings = (
            ("-print-target-triple", "-dumpmachine")
            if family == "clang"
            else ("-dumpmachine", "-print-target-triple")
        )
        resource_spellings = (
            ("-print-resource-dir", "-print-search-dirs")
            if family == "clang"
            else ("-print-search-dirs", "-print-resource-dir")
        )
        for label, spellings in (
            ("target", target_spellings),
            ("resource", resource_spellings),
        ):
            value_field = "resource_dir" if label == "resource" else label
            for spelling in spellings:
                result = _query([str(tool), spelling])
                record[f"{label}_query"] = result
                if result["returncode"] == 0 and result["stdout"].strip():
                    record[value_field] = result["stdout"].strip().splitlines()[0]
                    break
    return record


# ---------------------------------------------------------------------------
# LLVM coverage-tool resolution and format qualification (T17).
# ---------------------------------------------------------------------------


def _host_platform() -> str:
    """The platform this resolution runs for — a seam tests can pin."""
    return platform.system()


def _darwin_xcrun_find(tool_name: str) -> Path | None:
    """Darwin last-resort tool lookup through the selected Xcode toolchain.

    Only ever called on Darwin — Linux must never invoke ``xcrun``. Returns
    the located executable, or None when xcrun cannot find the tool.
    """
    xcrun = shutil.which("xcrun")
    if xcrun is None:
        return None
    completed = subprocess.run(
        [xcrun, "--find", tool_name],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
        timeout=60,
    )
    if completed.returncode != 0:
        return None
    located = Path(completed.stdout.strip())
    if located.is_file() and os.access(located, os.X_OK):
        return located
    return None


def _compiler_major_version(record: Mapping[str, Any]) -> str | None:
    """The first dotted-version integer in the compiler's --version banner."""
    version = record.get("version")
    if not isinstance(version, str):
        return None
    match = re.search(r"(\d+)(?:\.\d+)+", version)
    return match.group(1) if match else None


def _describe_coverage_tool(path: Path, *, source: str) -> dict[str, Any]:
    """Inspect one resolved coverage tool; LLVM-family tools only."""
    record = inspect_tool(path)
    if record["family"] not in {"clang", "clang-tidy"}:
        raise ValueError(
            f"tool {path} is not an LLVM toolchain tool "
            f"(banner: {record['version']!r})"
        )
    return {
        "path": record["path"],
        "resolved_path": record["resolved_path"],
        "version": record["version"],
        "source": source,
    }


def _resolve_coverage_tool(
    tool_name: str,
    compiler: Path,
    compiler_record: Mapping[str, Any],
    override: str | None,
) -> dict[str, Any]:
    """Resolve one llvm-profdata/llvm-cov in the lane's fixed order.

    Explicit override → compiler sibling → PATH candidate → (Darwin only)
    ``xcrun --find``. An unresolvable override is a hard error; a tool that
    resolves but cannot identify as an LLVM toolchain tool is skipped only
    within the discovery tiers, while the final format probe arbitrates
    compatibility for the selected pair.
    """
    env_name = COVERAGE_TOOL_OVERRIDES[tool_name]
    rejected: list[str] = []
    if override is not None and override != "":
        resolved = resolve_tool(env_name, override, [])
        return _describe_coverage_tool(resolved, source="override")

    def _candidate(path: Path, source: str) -> dict[str, Any] | None:
        try:
            return _describe_coverage_tool(path, source=source)
        except ValueError as error:
            rejected.append(f"{path} ({error})")
            return None

    # The compiler record carries only the version banner; sibling selection
    # is keyed by the clang family, which inspect_tool already established.
    sibling_spellings = sibling_candidates(
        {"path": str(compiler), "id": "Clang"}, tool_name
    )
    for spelling in sibling_spellings:
        candidate = Path(spelling)
        if candidate.is_file() and os.access(candidate, os.X_OK):
            described = _candidate(candidate, "sibling")
            if described is not None:
                return described

    major = _compiler_major_version(compiler_record)
    path_spellings = (
        [f"{tool_name}-{major}", tool_name] if major else [tool_name]
    )
    for spelling in path_spellings:
        located = shutil.which(spelling)
        if located and Path(located).is_file() and os.access(located, os.X_OK):
            described = _candidate(Path(located), "path")
            if described is not None:
                return described

    if _host_platform() == "Darwin":
        found = _darwin_xcrun_find(tool_name)
        if found is not None:
            described = _candidate(found, "xcrun")
            if described is not None:
                return described
            raise ValueError(
                f"the xcrun-resolved {tool_name} at {found} is not a usable "
                f"LLVM coverage tool for compiler {compiler}"
            )

    raise ValueError(
        f"could not resolve a usable {tool_name} for compiler {compiler}: "
        f"checked override {env_name}, compiler siblings "
        f"{sibling_spellings}, PATH candidates {path_spellings}"
        + (", and the Darwin xcrun fallback" if _host_platform() == "Darwin" else "")
        + (f"; rejected candidates: {rejected}" if rejected else "")
    )


def _probe_step(
    probe_directory: Path,
    name: str,
    argv: list[str],
    *,
    env: Mapping[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """Run one format-probe step, retaining command/stdout/stderr logs in the
    probe directory. Any launch failure or nonzero exit raises a ValueError
    that names the step, the exact argv, and the retained diagnostics."""
    log_dir = probe_directory / "diagnostics"
    log_dir.mkdir(parents=True, exist_ok=True)
    command = [str(token) for token in argv]
    (log_dir / f"{name}.command.json").write_text(json.dumps(command, indent=2) + "\n")
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
            timeout=120,
            env={**os.environ, **env} if env is not None else None,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        (log_dir / f"{name}.stdout.log").write_text("")
        (log_dir / f"{name}.stderr.log").write_text(f"{error}\n")
        raise ValueError(
            f"coverage format probe could not launch {name}: "
            f"{shlex.join(command)}: {error}; "
            f"probe diagnostics retained in {probe_directory}"
        ) from error
    (log_dir / f"{name}.stdout.log").write_text(completed.stdout)
    (log_dir / f"{name}.stderr.log").write_text(completed.stderr)
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise ValueError(
            f"coverage format probe {name} exited {completed.returncode}: "
            f"{shlex.join(command)}"
            + (f"\nstderr: {detail}" if detail else "")
            + f"\nprobe diagnostics retained in {probe_directory}"
        )
    return completed


def _coverage_format_probe(
    compiler: Path, llvm_profdata: Path, llvm_cov: Path
) -> dict[str, Any]:
    """Prove this compiler/tool pair shares the LLVM coverage format.

    Compile a minimal instrumented control with the selected compiler, run
    it under a fresh ``LLVM_PROFILE_FILE``, then merge and export the
    profile with the resolved tools. Only a complete chain counts as
    evidence; on failure the probe directory is retained so the caller can
    point at the per-step logs.
    """
    probe_directory = Path(tempfile.mkdtemp(prefix="bike-coverage-probe-"))
    try:
        source = probe_directory / "control.cpp"
        source.write_text(
            "int main(int argc, char**) { return argc > 1 ? 1 : 0; }\n"
        )
        binary = probe_directory / "control"
        _probe_step(
            probe_directory,
            "control-compile",
            [
                str(compiler),
                "-std=c++23",
                "-fprofile-instr-generate",
                "-fcoverage-mapping",
                str(source),
                "-o",
                str(binary),
            ],
        )
        profile_pattern = probe_directory / "control-%p.profraw"
        _probe_step(
            probe_directory,
            "control-run",
            [str(binary)],
            env={"LLVM_PROFILE_FILE": str(profile_pattern)},
        )
        raw_profiles = sorted(probe_directory.glob("control-*.profraw"))
        if not raw_profiles:
            raise ValueError(
                "coverage format probe produced no .profraw under "
                f"{profile_pattern}; probe diagnostics retained in "
                f"{probe_directory}"
            )
        merged = probe_directory / "control.profdata"
        _probe_step(
            probe_directory,
            "control-merge",
            [
                str(llvm_profdata),
                "merge",
                "-sparse",
                *(str(path) for path in raw_profiles),
                "-o",
                str(merged),
            ],
        )
        if not merged.is_file() or merged.stat().st_size == 0:
            raise ValueError(
                f"coverage format probe produced no merged profile at {merged}; "
                f"probe diagnostics retained in {probe_directory}"
            )
        export = _probe_step(
            probe_directory,
            "control-export",
            [str(llvm_cov), "export", str(binary), f"-instr-profile={merged}"],
        )
        try:
            document = json.loads(export.stdout)
        except json.JSONDecodeError as error:
            raise ValueError(
                "coverage format probe export did not return valid JSON: "
                f"{error}; probe diagnostics retained in {probe_directory}"
            ) from error
        if (
            not isinstance(document, dict)
            or document.get("type") != "llvm.coverage.json.export"
            or not isinstance(document.get("data"), list)
            or not document["data"]
        ):
            raise ValueError(
                "coverage format probe export did not contain valid control "
                f"data (expected type llvm.coverage.json.export with a "
                f"nonempty data array); probe diagnostics retained in "
                f"{probe_directory}"
            )
    except ValueError:
        raise
    except Exception as error:
        raise ValueError(
            f"coverage format probe failed: {error}; "
            f"probe diagnostics retained in {probe_directory}"
        ) from error
    shutil.rmtree(probe_directory, ignore_errors=True)
    return {
        "format_probe_passed": True,
        "export_version": document.get("version"),
        "steps": [
            "control-compile",
            "control-run",
            "control-merge",
            "control-export",
        ],
    }


def resolve_coverage_tools(compiler: Path, overrides: dict[str, str]) -> dict:
    """Resolve and qualify the llvm-profdata/llvm-cov pair for a compiler.

    Resolution order per tool: explicit ``LLVM_PROFDATA``/``LLVM_COV``
    override (an invalid override is a hard error), a sibling beside the
    compiler, a PATH candidate, and — Darwin only — the ``xcrun --find``
    fallback. The selected pair is then qualified by the real instrumented
    compile/run/merge/export format probe; a resolved-but-incompatible pair
    fails here with the command's diagnostics, never at report time.

    Returns the selected paths, version banners, resolution sources, and
    the probe evidence. Raises ValueError for a non-executable compiler, a
    non-clang-family compiler, an unresolvable tool, or a failed probe.
    """
    compiler_path = Path(compiler).expanduser()
    if not compiler_path.is_file() and shutil.which(str(compiler)) is None:
        raise ValueError(f"coverage compiler is not an executable file: {compiler}")
    if not compiler_path.is_file():
        compiler_path = Path(shutil.which(str(compiler)) or str(compiler))
    unknown = sorted(set(overrides) - set(COVERAGE_TOOL_OVERRIDES.values()))
    if unknown:
        raise ValueError(f"unknown coverage tool overrides: {unknown!r}")

    compiler_record = inspect_tool(compiler_path)
    if compiler_record["family"] != "clang":
        raise ValueError(
            "native coverage requires an LLVM clang-family compiler; "
            f"{compiler_path} identifies as {compiler_record['family']!r}"
        )

    profdata = _resolve_coverage_tool(
        "llvm-profdata", compiler_path, compiler_record, overrides.get("LLVM_PROFDATA")
    )
    cov = _resolve_coverage_tool(
        "llvm-cov", compiler_path, compiler_record, overrides.get("LLVM_COV")
    )
    probe = _coverage_format_probe(
        compiler_path, Path(profdata["path"]), Path(cov["path"])
    )
    return {
        "compiler": {
            "path": compiler_record["path"],
            "resolved_path": compiler_record["resolved_path"],
            "version": compiler_record["version"],
            "family": compiler_record["family"],
            "target": compiler_record.get("target"),
        },
        "llvm_profdata": profdata,
        "llvm_cov": cov,
        "format_probe": probe,
    }


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    coverage = commands.add_parser(
        "coverage-tools",
        help="resolve and qualify the LLVM coverage tools for a compiler",
    )
    coverage.add_argument(
        "--compiler",
        help="compiler to qualify coverage tools for (overrides --context)",
    )
    coverage.add_argument(
        "--context",
        help="build directory whose native_check_context.json records the compiler",
    )
    coverage.add_argument("--llvm-profdata", default=os.environ.get("LLVM_PROFDATA"))
    coverage.add_argument("--llvm-cov", default=os.environ.get("LLVM_COV"))
    coverage.add_argument(
        "--json", help="also write the resolution record to this path"
    )
    arguments = parser.parse_args(argv)
    if arguments.command == "coverage-tools":
        compiler_value = arguments.compiler
        if not compiler_value and arguments.context:
            context_path = Path(arguments.context) / "native_check_context.json"
            try:
                context = json.loads(context_path.read_text())
            except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
                print(
                    f"coverage tool resolution failed: cannot read "
                    f"{context_path}: {error}",
                    file=sys.stderr,
                )
                return 1
            compiler_value = (context.get("compiler") or {}).get("path")
        if not isinstance(compiler_value, str) or not compiler_value:
            print(
                "coverage tool resolution failed: pass --compiler or a "
                "--context build directory carrying native_check_context.json",
                file=sys.stderr,
            )
            return 1
        overrides = {
            key: value
            for key, value in (
                ("LLVM_PROFDATA", arguments.llvm_profdata),
                ("LLVM_COV", arguments.llvm_cov),
            )
            if value
        }
        try:
            record = resolve_coverage_tools(Path(compiler_value), overrides)
        except ValueError as error:
            print(f"coverage tool resolution failed: {error}", file=sys.stderr)
            return 1
        text = json.dumps(record, indent=2, sort_keys=True) + "\n"
        if arguments.json:
            Path(arguments.json).parent.mkdir(parents=True, exist_ok=True)
            Path(arguments.json).write_text(text)
        sys.stdout.write(text)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(_main())
