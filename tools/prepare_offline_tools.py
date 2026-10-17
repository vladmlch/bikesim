#!/usr/bin/env python3
"""Freeze explicitly selected local build/tool wheels for offline assembly.

This command reads wheel metadata and bytes. It never resolves, downloads,
installs, or executes a wheel. The resulting lock records selected inputs;
it does not certify that they passed the later offline installation gate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tomllib
from zipfile import BadZipFile

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.offline_bundle import (
    inspect_wheel,
    marker_environment,
    select_target,
    validate_dependency_closure,
)
from tools.offline_manifest import TOOL_NAMES, validate_target


def prepare_tool_lock(project_root: Path, wheelhouse: Path, target: dict) -> dict:
    """Validate one selected wheel per distribution and return exact tool pins."""
    validate_target(target)
    project_root = project_root.resolve(strict=True)
    wheelhouse = wheelhouse.resolve(strict=True)
    if not wheelhouse.is_dir():
        raise ValueError(f"tool wheelhouse is not a directory: {wheelhouse}")
    with (project_root / "pyproject.toml").open("rb") as source:
        project = tomllib.load(source)
    build_system = project.get("build-system")
    if not isinstance(build_system, dict):
        raise ValueError("pyproject.toml: missing build-system requirements")
    requirements = build_system.get("requires")
    if not isinstance(requirements, list) or not requirements or any(
        not isinstance(requirement, str) or not requirement for requirement in requirements
    ):
        raise ValueError("pyproject.toml: build-system.requires must be a nonempty list of requirements")

    records = []
    names = set()
    for path in sorted(wheelhouse.glob("*.whl")):
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"tool wheel must be a regular, non-symlink file: {path.name}")
        record = inspect_wheel(path, target)
        row = record["wheel"]
        name = row["distribution"]
        if name in names:
            raise ValueError(f"tool wheelhouse has more than one wheel for {name}")
        names.add(name)
        records.append(record)
    missing = TOOL_NAMES - names
    if missing:
        raise ValueError(f"tool wheelhouse is missing required distributions: {', '.join(sorted(missing))}")

    # Start from the actual tools and PEP 517 roots. Their active dependency
    # edges must explain every additional wheel; an unrelated selected package
    # must not silently become part of the offline tool environment.
    roots = [f"{record['wheel']['distribution']}=={record['wheel']['version']}"
             for record in records if record["wheel"]["distribution"] in TOOL_NAMES]
    validate_dependency_closure(records, roots + requirements, marker_environment(target))
    uv_record = next(record for record in records if record["wheel"]["distribution"] == "uv")
    if not uv_record["licenses"]:
        raise ValueError("uv tool wheel must retain its license files")
    return {
        "schema_version": 1,
        "target": target,
        "uv_version": uv_record["wheel"]["version"],
        "wheels": [record["wheel"] for record in records],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheelhouse", type=Path, required=True,
                        help="directory containing only the explicitly selected tool wheel set")
    parser.add_argument("--target", choices=("host", "linux-x86_64", "macos-arm64"), default="host")
    parser.add_argument("--python", default=f"{sys.version_info.major}.{sys.version_info.minor}",
                        help="target CPython MAJOR.MINOR (default: this interpreter)")
    parser.add_argument("--native", action="store_true", help="require the native target to match this host")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1],
                        help="source tree whose PEP 517 requirements must be satisfied")
    parser.add_argument("--output", type=Path, required=True,
                        help="new supplemental tool lock; existing files are never overwritten")
    args = parser.parse_args(argv)
    try:
        target = select_target(args.target, args.python, args.native)
        lock = prepare_tool_lock(args.project, args.wheelhouse, target)
        content = json.dumps(lock, indent=2, sort_keys=True, allow_nan=False) + "\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation also rejects a dangling output symlink. Never
        # overwrite an authoritative pin set while preparing a new one.
        with args.output.open("x", encoding="utf-8") as destination:
            destination.write(content)
    except (OSError, ValueError, BadZipFile) as error:
        print(f"OFFLINE TOOL LOCK ERROR: {error}", file=sys.stderr)
        return 2
    print(f"Wrote {args.output}: {len(lock['wheels'])} selected tool wheels; installation qualification pending.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
