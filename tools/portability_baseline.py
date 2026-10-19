"""Collect a non-mutating platform/environment baseline for the checkout.

Deliverable for the Linux+macOS portability plan (task T01). The report
separates what is actually observed on this machine from what the packed
source snapshot could not carry: real Git identity, interpreter, locked
package versions, toolchain versions, NumPy provider configuration, file
modes, and binary fixture assets that tests depend on.

Collection NEVER installs packages, edits the lock, or mutates the
checkout. Every probe is a read-only subprocess (or in-process metadata
lookup); failed probes are preserved verbatim as evidence instead of
being hidden.

CLI:
    uv run --frozen --group native --no-sync python \
        tools/portability_baseline.py --output PATH [--root PATH]
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import platform
import re
import subprocess
import sys
import sysconfig
import tomllib
from pathlib import Path

SCHEMA_VERSION = 1

# Inputs the packed snapshot could not carry, or that external documents
# reference. Absence is recorded, never fabricated over.
CHECKOUT_INPUTS = (
    "pyproject.toml",
    "uv.lock",
    ".python-version",
    ".github",
    ".git",
    # Linked from docs/agents/native-engineering.md; genuinely absent in
    # this checkout (controller ruling: record, do not recreate).
    "docs/superpowers/specs/2026-10-06-native-safety.md",
    "docs/superpowers/plans/2026-10-06-native-safety-master.md",
)

# Names worth recording beyond the declared direct dependencies; only
# reported when actually installed.
SUPPLEMENTARY_PACKAGES = ("nanobind", "hypothesis", "threadpoolctl")

# Binary/model assets the test suite loads from the checkout; they do not
# survive a text-only snapshot, so their identity must be observable.
FIXTURE_GLOBS = ("tools/proto_native_bench/artifacts/**/*",)

# Files whose executable bit is part of the delivery contract.
MODE_GLOBS = ("tools/*.sh", "packaging/*.sh", "*.sh")

_DEPENDENCY_NAME = re.compile(r"^\s*([A-Za-z0-9][A-Za-z0-9._-]*)")


def capture(argv: list[str], root: Path) -> dict:
    """Run a read-only probe, preserving stdout/stderr/status verbatim."""
    try:
        result = subprocess.run(
            argv,
            cwd=root,
            text=True,
            capture_output=True,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"argv": argv, "returncode": None, "error": str(exc)}
    return {
        "argv": argv,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _probe_ok(probe: dict) -> bool:
    return probe.get("returncode") == 0


def _git_section(root: Path) -> dict:
    """Real checkout identity; nulls plus evidence when not verifiable."""
    inside = capture(["git", "rev-parse", "--is-inside-work-tree"], root)
    head = capture(["git", "rev-parse", "HEAD"], root)
    branch = capture(["git", "rev-parse", "--abbrev-ref", "HEAD"], root)
    status = capture(["git", "status", "--porcelain"], root)
    verified = (
        _probe_ok(inside)
        and inside["stdout"].strip() == "true"
        and _probe_ok(head)
        and _probe_ok(status)
    )
    entries: list[str] = []
    if verified:
        entries = [line for line in status["stdout"].splitlines() if line.strip()]
    untracked = [line for line in entries if line.startswith("??")]
    return {
        "verified": verified,
        "commit": head["stdout"].strip() if _probe_ok(head) else None,
        "branch": branch["stdout"].strip() if _probe_ok(branch) else None,
        "dirty": bool(entries) if verified else None,
        "modified_entries": len(entries) - len(untracked) if verified else None,
        "untracked_entries": len(untracked) if verified else None,
        "status_porcelain": status["stdout"] if verified else None,
        "commands": {
            "rev_parse_is_inside_work_tree": inside,
            "rev_parse_head": head,
            "rev_parse_abbrev_ref": branch,
            "status_porcelain": status,
        },
    }


def _python_section() -> dict:
    return {
        "executable": sys.executable,
        "version": sys.version,
        "version_info": list(sys.version_info[:3]),
        "implementation": platform.python_implementation(),
        "compiler": platform.python_compiler(),
        "soabi": sysconfig.get_config_var("SOABI"),
    }


def _platform_section() -> dict:
    libc_name, libc_version = platform.libc_ver()
    return {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "platform": platform.platform(),
        "hostname": platform.node(),
        "libc": {"name": libc_name or None, "version": libc_version or None},
    }


def _declared_dependencies(root: Path) -> dict[str, dict]:
    """Declared direct dependencies: [project].dependencies + groups."""
    document_path = root / "pyproject.toml"
    declared: dict[str, dict] = {}
    try:
        document = tomllib.loads(document_path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError) as exc:
        return {"_error": {"name": type(exc).__name__, "message": str(exc)}}
    requirements: list[tuple[str, str]] = [
        ("project.dependencies", entry)
        for entry in document.get("project", {}).get("dependencies", [])
    ]
    for group, entries in sorted(document.get("dependency-groups", {}).items()):
        for entry in entries:
            requirements.append((f"dependency-groups.{group}", entry))
    for origin, requirement in requirements:
        match = _DEPENDENCY_NAME.match(str(requirement))
        if match is None:
            continue
        name = match.group(1)
        record = declared.setdefault(name, {"requirements": []})
        record["requirements"].append({"origin": origin, "spec": str(requirement)})
    return declared


def _packages_section(root: Path) -> dict:
    declared = _declared_dependencies(root)
    error = declared.pop("_error", None)
    for name, record in declared.items():
        try:
            record["version"] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            record["version"] = None
            record["error"] = "PackageNotFoundError"
    additional: dict[str, str] = {}
    for name in SUPPLEMENTARY_PACKAGES:
        try:
            additional[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue  # recorded only when installed
    section = {
        "source": "importlib.metadata of the collecting interpreter",
        "declared": declared,
        "additional": additional,
    }
    if error is not None:
        section["error"] = error
    return section


def _numpy_section(root: Path) -> dict:
    """NumPy identity and provider config via the collecting interpreter.

    Uses subprocesses so show_config/show_runtime output is captured
    verbatim without editing environment variables.
    """
    version_probe = capture(
        [sys.executable, "-c", "import numpy; print(numpy.__version__)"], root
    )
    config_probe = capture(
        [sys.executable, "-c", "import numpy; numpy.show_config()"], root
    )
    runtime_probe = capture(
        [sys.executable, "-c", "import numpy; numpy.show_runtime()"], root
    )
    available = _probe_ok(version_probe)
    section = {
        "available": available,
        "version": version_probe["stdout"].strip() if available else None,
        "error": None if available else "numpy import/version probe failed",
        "show_config": config_probe,
        "show_runtime": runtime_probe,
    }
    return section


def _tools_section(root: Path) -> dict:
    tools: dict[str, dict] = {
        "git": capture(["git", "--version"], root),
        "uv": capture(["uv", "--version"], root),
        "cmake": capture(["cmake", "--version"], root),
        "cc": capture(["cc", "--version"], root),
        "cxx": capture(["c++", "--version"], root),
    }
    if platform.system() == "Darwin":
        # Apple-only probes; never run on other platforms.
        tools["sw_vers"] = capture(["sw_vers"], root)
        tools["xcode_select_path"] = capture(["xcode-select", "-p"], root)
        tools["xcodebuild_version"] = capture(["xcodebuild", "-version"], root)
        tools["xcrun_sdk_path"] = capture(["xcrun", "--show-sdk-path"], root)
        tools["xcrun_sdk_version"] = capture(
            ["xcrun", "--show-sdk-version"], root
        )
        tools["xcrun_macosx_sdk_path"] = capture(
            ["xcrun", "--sdk", "macosx", "--show-sdk-path"], root
        )
        tools["xcrun_macosx_sdk_version"] = capture(
            ["xcrun", "--sdk", "macosx", "--show-sdk-version"], root
        )
        tools["clang"] = capture(["clang", "--version"], root)
    return tools


def _inputs_section(root: Path) -> dict:
    """Presence, mode, and content identity of named checkout inputs."""
    inventory: dict[str, dict] = {}
    for relative in CHECKOUT_INPUTS:
        path = root / relative
        record: dict = {"present": path.exists()}
        if path.is_file():
            record.update(
                kind="file",
                bytes=path.stat().st_size,
                sha256=_sha256(path),
                mode=oct(path.stat().st_mode & 0o777),
            )
        elif path.is_dir():
            record["kind"] = "directory"
        elif path.exists():
            record["kind"] = "other"
        inventory[relative] = record
    return inventory


def _file_modes(root: Path) -> dict[str, str]:
    """Octal permission bits for delivered scripts (executable-bit audit)."""
    modes: dict[str, str] = {}
    for pattern in MODE_GLOBS:
        for path in sorted(root.glob(pattern)):
            if path.is_file():
                modes[path.relative_to(root).as_posix()] = oct(
                    path.stat().st_mode & 0o777
                )
    return modes


def _fixture_assets(root: Path) -> dict[str, dict]:
    """Binary/model assets tests load; content-hashed for provenance."""
    assets: dict[str, dict] = {}
    for pattern in FIXTURE_GLOBS:
        for path in sorted(root.glob(pattern)):
            if not path.is_file():
                continue
            relative = path.relative_to(root).as_posix()
            stat = path.stat()
            assets[relative] = {
                "bytes": stat.st_size,
                "sha256": _sha256(path),
                "mode": oct(stat.st_mode & 0o777),
            }
    return assets


def collect_baseline(root: Path) -> dict:
    """Assemble the schema-1 baseline report for the checkout at ``root``.

    Purely observational: read-only subprocesses, metadata lookups, and
    file hashing. Never installs, locks, stashes, or writes into ``root``.
    """
    root = Path(root).resolve()
    inputs = _inputs_section(root)
    missing = [
        relative
        for relative, record in inputs.items()
        if not record["present"]
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "root": str(root),
        "git": _git_section(root),
        "python": _python_section(),
        "platform": _platform_section(),
        "packages": _packages_section(root),
        "numpy_config": _numpy_section(root),
        "tools": _tools_section(root),
        "inputs": inputs,
        "file_modes": _file_modes(root),
        "fixture_assets": _fixture_assets(root),
        "missing_inputs": missing,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Collect a non-mutating platform baseline (schema 1)."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="checkout root to describe (default: this repository)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="write JSON here (default: stdout)",
    )
    args = parser.parse_args(argv)
    report = collect_baseline(args.root)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        sys.stdout.write(rendered)
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"wrote baseline report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
