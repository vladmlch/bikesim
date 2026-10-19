"""Text-level contract tests for the T26 development documentation.

These tests never import the native extension and never spawn builds; the
quick profile runs them on any host. They pin the facts the platform guides
must carry (qualified toolchains, selector environment variables, the
invalid-on-Linux Apple SDK option), forbid the original personal
reference-source path in *active* instructions, and validate local Markdown
links plus referenced repository paths. Dated/historical log sections keep
their verbatim content — the personal-path check is scoped to active
instruction sections so history is never rewritten to satisfy a text search.
"""
from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

LINUX_GUIDE = REPO_ROOT / "docs" / "development" / "linux.md"
MACOS_GUIDE = REPO_ROOT / "docs" / "development" / "macos.md"
TROUBLESHOOTING = REPO_ROOT / "docs" / "development" / "troubleshooting.md"

# The one-time developer checkout paths that T26 removes from active
# instructions. Historical log sections keep whatever they recorded.
_PERSONAL_PATH_TOKENS = (
    "/Users/vladislav.molchanov/Desktop/Projects_downloaded",
    "Projects_downloaded/numpy",
    "Projects_downloaded/mujoco",
)

# Files that are entirely active instructions: no historical log sections.
_ACTIVE_INSTRUCTION_FILES = (
    "AGENTS.md",
    "README.md",
    "docs/development/linux.md",
    "docs/development/macos.md",
    "docs/development/troubleshooting.md",
    "docs/development/install-modes.md",
    "docs/agents/native-engineering.md",
    "native/.clangd",
)

# Documents that interleave active instructions with dated evidence; the
# personal-path check applies only to their non-historical sections.
_HISTORICAL_MIXED_FILES = (
    "docs/RIDE.md",
    "docs/RECORDING.md",
    "docs/TESTING.md",
)

# Referenced-but-still-authored documents: links point at them on purpose
# while a concurrent task finishes the file. Existence is not asserted yet;
# keeping the list explicit prevents a silent dead link later.
_PENDING_REFERENCES = frozenset(
    {Path("docs/development/portability-test-matrix.md")}
)

_HEADING = re.compile(r"^(#{1,6})\s+(.*)$")
_MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
# Backticked repo-relative file references such as `tools/dev.py` or
# `native/src/engine_call.cpp`. Directory mentions (no filename extension)
# are skipped on purpose: build/output directories are machine-local.
_REFERENCED_FILE = re.compile(
    r"`((?:docs|tools|tests|src|native|packaging|examples)/[A-Za-z0-9_./-]*"
    r"\.[A-Za-z0-9]+)`"
)


def _read(path: Path) -> str:
    assert path.is_file(), f"required document is missing: {path}"
    return path.read_text(encoding="utf-8")


def _active_text(path: Path) -> str:
    """Document text with dated/historical log sections removed.

    A section is historical when its heading declares it (``historical``,
    ``history``, ``log``, ``snapshot`` or an ISO date like ``2026-10-10``).
    Everything else — prerequisites, workflows, command blocks — is active
    instruction text and must never require the personal path.
    """
    text = _read(path)
    kept: list[str] = []
    historical_level: int | None = None
    for line in text.splitlines():
        heading = _HEADING.match(line)
        if heading:
            level = len(heading.group(1))
            title = heading.group(2).lower()
            if historical_level is not None and level > historical_level:
                continue  # still inside the historical section's subtree
            historical_level = None
            if (
                "historical" in title
                or "history" in title
                or "snapshot" in title
                or re.search(r"\b20\d\d-\d\d-\d\d\b", title)
            ):
                historical_level = level
                continue
        if historical_level is not None:
            continue
        kept.append(line)
    return "\n".join(kept)


def _local_markdown_links(text: str) -> list[str]:
    links = []
    for match in _MARKDOWN_LINK.finditer(text):
        target = match.group(1)
        if "://" in target or target.startswith(("mailto:", "#")):
            continue
        links.append(target.split("#", 1)[0])
    return links


def _assert_no_personal_path(text: str, where: str) -> None:
    for token in _PERSONAL_PATH_TOKENS:
        assert token not in text, (
            f"active instructions in {where} still require the personal "
            f"reference-source path {token!r}; use NUMPY_SOURCE_ROOT/"
            "MUJOCO_SOURCE_ROOT or .work/reference-sources/ instead"
        )


def test_linux_guide_carries_the_native_workflow():
    text = _read(LINUX_GUIDE)
    assert "tools/dev.py" in text
    assert "--group native" in text
    assert "BIKE_NATIVE_BUILD_PATH" in text
    assert "NATIVE_TEST_BUILD_PATH" in text
    assert "UV_PROJECT_ENVIRONMENT" in text
    # Apple-only configure option, documented as invalid on Linux.
    assert "CMAKE_OSX_SYSROOT" in text
    osx_lines = [l for l in text.splitlines() if "CMAKE_OSX_SYSROOT" in l]
    assert any(
        re.search(r"invalid|reject|apple.only|not supported|refused", l, re.I)
        for l in osx_lines
    ), "CMAKE_OSX_SYSROOT must be documented as invalid on Linux"
    # No mandatory Apple tooling: every xcrun mention is a negation.
    for line in text.splitlines():
        if "xcrun" in line:
            assert re.search(
                r"never|not|no\b|reject|invalid|apple.only|does not", line, re.I
            ), f"xcrun presented as a Linux requirement: {line!r}"
    # CPython 3.13 is an explicit requirement, not unqualified `python3`.
    assert "3.13" in text and "CPython" in text
    # The RTSan lane is documented, not silently skipped.
    assert "RTSan" in text or "-fsanitize=realtime" in text


def test_linux_guide_records_measured_qualification():
    text = _read(LINUX_GUIDE)
    for evidence in ("3.13.16", "clang", "aarch64", "24.04"):
        assert evidence in text.lower() or evidence in text, (
            f"Linux guide lacks measured qualification evidence {evidence!r}"
        )
    # Honesty: the OS-default compiler is never asserted sufficient on its
    # own — configure-time capability probes decide.
    assert "probe" in text.lower()


def test_macos_guide_retains_platform_specifics():
    text = _read(MACOS_GUIDE)
    for required in (
        "mjpython",
        "DYLD_FALLBACK_LIBRARY_PATH",
        "CMAKE_OSX_SYSROOT",
        "Accelerate",
        "BIKE_NATIVE_BUILD_PATH",
        "--group native",
        "tools/dev.py",
    ):
        assert required in text, f"macOS guide lost required token {required!r}"
    assert re.search(r"xcode|sdk", text, re.I)
    # Measured host: AppleClang and the uv-managed 3.14 interpreter.
    assert "3.14.3" in text and "AppleClang" in text


def test_platform_guides_share_workflow_headings():
    linux_headings = [
        m.group(2).strip().lower()
        for m in map(_HEADING.match, _read(LINUX_GUIDE).splitlines())
        if m and len(m.group(1)) == 2
    ]
    macos_headings = [
        m.group(2).strip().lower()
        for m in map(_HEADING.match, _read(MACOS_GUIDE).splitlines())
        if m and len(m.group(1)) == 2
    ]
    assert linux_headings, "linux.md has no level-2 headings"
    assert linux_headings == macos_headings, (
        f"guides diverge: linux={linux_headings} macos={macos_headings}"
    )


def test_troubleshooting_covers_required_symptoms():
    text = _read(TROUBLESHOOTING).lower()
    required_evidence = {
        "apple sdk on linux": ("cmake_osx_sysroot", "sdk"),
        "mujoco private abi": ("private", "mujoco"),
        "numerical provider/abi": ("provider", "blas"),
        "missing artifact": ("artifact", "bike_native"),
        "interpreter soabi": ("soabi",),
        "stale source stamp": ("source", "stamp"),
        "unresolved shared object": ("dso",) ,
        "display": ("display",),
        "sanitizer toolchain": ("sanitizer",),
        "coverage tools": ("coverage",),
        "strict invalid controller": ("invalid_controller",),
        "replay identity": ("replay", "identity"),
        "slim externals": ("slim",),
    }
    for symptom, tokens in required_evidence.items():
        for token in tokens:
            assert token in text, (
                f"troubleshooting lacks evidence for {symptom}: {token!r}"
            )
    # Bounded remediation only: the forbidden shortcuts are never advised.
    for forbidden in ("disable the test", "skip the test", "delete uv.lock",
                      "delete the lock"):
        assert forbidden not in text


def test_active_instructions_do_not_require_personal_paths():
    for relative in _ACTIVE_INSTRUCTION_FILES:
        _assert_no_personal_path(_read(REPO_ROOT / relative), relative)
    for relative in _HISTORICAL_MIXED_FILES:
        _assert_no_personal_path(
            _active_text(REPO_ROOT / relative),
            f"{relative} (active sections)",
        )


def test_agents_md_uses_reference_source_convention():
    text = _read(REPO_ROOT / "AGENTS.md")
    assert "NUMPY_SOURCE_ROOT" in text
    assert "MUJOCO_SOURCE_ROOT" in text
    _assert_no_personal_path(text, "AGENTS.md")


def test_new_guide_links_and_referenced_paths_exist():
    for document in (LINUX_GUIDE, MACOS_GUIDE, TROUBLESHOOTING):
        text = _read(document)
        for link in _local_markdown_links(text):
            resolved = (document.parent / link).resolve()
            assert resolved.is_file() or link in {
                str(p) for p in _PENDING_REFERENCES
            }, f"{document.name}: dead local link {link!r}"
        for reference in _REFERENCED_FILE.findall(text):
            candidate = Path(reference)
            if candidate in _PENDING_REFERENCES:
                continue
            assert (REPO_ROOT / candidate).is_file(), (
                f"{document.name}: referenced file {reference!r} "
                "does not exist in this checkout"
            )


def test_ride_platform_section_links_evidence():
    text = _read(REPO_ROOT / "docs" / "RIDE.md")
    assert "platform" in text.lower()
    for link in ("development/linux.md", "development/macos.md"):
        assert link in text, f"RIDE.md does not link {link}"
