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
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
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
_SIBLING_FAMILY: dict[str, tuple[str | None, tuple[str, ...]]] = {
    "tidy": ("Clang", ("clang-tidy",)),
    "analyzer": ("Clang", ("clang++",)),
    "cppcheck": (None, ()),
    "gxx": ("GNU", ("g++",)),
    "frontends": ("GNU", ("g++",)),
    "odr": ("GNU", ("g++",)),
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
