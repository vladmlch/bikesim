#!/usr/bin/env python3
"""Assemble frozen Python or native-source bundles; never qualify an install.

The online authoring command is explicit. Importing this module only defines
data validation helpers: it never downloads, installs, invokes Git, or builds.
Only the Python-only route builds the project's existing setuptools wheel.
"""
from __future__ import annotations

import argparse
import email.parser
import hashlib
import json
import os
import platform
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import tomllib
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path, PurePosixPath
from typing import Any

from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.tags import compatible_tags, cpython_tags, mac_platforms, parse_tag, sys_tags
from packaging.utils import canonicalize_name, parse_wheel_filename
from packaging.version import Version

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.offline_manifest import (  # noqa: E402
    BUILD_MARKERS, SCHEMA_VERSION, TOOL_NAMES, _NATIVE_SOURCES, confined_file, include_source_path,
    python_minor, safe_relative_path, sha256_file, source_digest, validate_bundle_manifest,
    validate_target, validate_wheel_row,
)


DEFAULT_SLIM_EXTERNAL = ("numpy", "scipy", "numba", "llvmlite", "pyarrow")
_HASH_OPTION = re.compile(r"\s+--hash=sha256:([0-9a-f]{64})(?=\s|$)")
_NATIVE_REQUIRED = _NATIVE_SOURCES


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)


def _host_identity() -> tuple[str, str, str]:
    system = platform.system()
    machine = {"AMD64": "x86_64", "aarch64": "arm64" if system == "Darwin" else "aarch64"}.get(platform.machine(), platform.machine())
    return system, machine, f"{sys.version_info.major}.{sys.version_info.minor}"


def select_target(target_name: str, python_version: str, native: bool) -> dict[str, Any]:
    """Native uses this interpreter's complete actual supported wheel-tag set."""
    version = python_minor(python_version)
    host_system, host_machine, host_python = _host_identity()
    named = {"linux-x86_64": ("Linux", "x86_64"), "macos-arm64": ("Darwin", "arm64")}
    if target_name == "host":
        system, machine = host_system, host_machine
        if python_version != host_python:
            raise ValueError(f"--target host requires running under CPython {python_version}; actual interpreter is {host_python}")
    elif target_name in named:
        system, machine = named[target_name]
    else:
        raise ValueError(f"unsupported target {target_name!r}")
    host_match = (system, machine, python_version) == (host_system, host_machine, host_python)
    if native and not host_match:
        raise ValueError("native target/host mismatch: native source bundles must be assembled on their target OS/architecture/Python")
    if host_match:
        if platform.python_implementation() != "CPython":
            raise ValueError("only standard CPython is qualified for these bundles")
        tags = [str(tag) for tag in sys_tags()]
        if system == "Linux":
            libc, libc_version = platform.libc_ver()
            if libc != "glibc" or not re.fullmatch(r"2\.[0-9]+", libc_version):
                raise ValueError("native/source host must use identifiable glibc, not musl")
            floor: str | None = "glibc>=" + libc_version
        else:
            floor = None
    else:
        # Availability-only compatibility route. Enumerate all lower platform
        # spellings: pip/PyPI wheel tags are not a >= comparison.
        if system == "Linux":
            platforms = [f"manylinux_2_{minor}_x86_64" for minor in range(28, 4, -1)]
            platforms += ["manylinux2014_x86_64", "manylinux2010_x86_64", "manylinux1_x86_64"]
            floor = "glibc>=2.28"
        else:
            platforms = list(mac_platforms(version=(14, 0), arch="arm64"))
            floor = None
        tags = list(dict.fromkeys(str(tag) for tag in (
            *cpython_tags(python_version=version, abis=[f"cp{version[0]}{version[1]}"], platforms=platforms),
            *compatible_tags(python_version=version, interpreter=f"cp{version[0]}{version[1]}", platforms=platforms),
        )))
    return validate_target({"system": system, "machine": machine, "python_minor": python_version,
                            "implementation": "cpython", "accepted_wheel_tags": tags, "libc_floor": floor})


def marker_environment(target: dict[str, Any]) -> dict[str, str]:
    validate_target(target)
    if (target["system"], target["machine"], target["python_minor"]) == _host_identity():
        return default_environment()
    return {"implementation_name": "cpython", "implementation_version": target["python_minor"] + ".0",
            "os_name": "posix", "platform_machine": target["machine"],
            "platform_python_implementation": "CPython", "platform_release": "",
            "platform_system": target["system"], "platform_version": "",
            "python_full_version": target["python_minor"] + ".0", "python_version": target["python_minor"],
            "sys_platform": "linux" if target["system"] == "Linux" else "darwin"}


def _marker_applies(requirement: Requirement, environment: dict[str, str], extras: set[str]) -> bool:
    if requirement.marker is None:
        return True
    expression = str(requirement.marker)
    if not environment["platform_release"] and any(name in expression for name in ("platform_release", "platform_version")):
        raise ValueError(f"cross-target marker requires an actual target host: {requirement}")
    values = [dict(environment, extra=extra) for extra in sorted(extras | {""})]
    result = any(requirement.marker.evaluate(value) for value in values)
    if not environment["platform_release"] and any(name in expression for name in ("python_full_version", "implementation_version")):
        upper = [dict(value, python_full_version=environment["python_version"] + ".999", implementation_version=environment["python_version"] + ".999") for value in values]
        if any(requirement.marker.evaluate(value) for value in upper) != result:
            raise ValueError(f"cross-target marker varies within Python minor; use actual host: {requirement}")
    return result


def _wheel_member_bytes(archive: zipfile.ZipFile, member: str) -> bytes:
    try:
        return archive.read(member)
    except (RuntimeError, NotImplementedError, zipfile.BadZipFile) as error:
        raise ValueError(f"unreadable wheel member {member}: {error}") from error


def inspect_wheel(path: Path, target: dict[str, Any]) -> dict[str, Any]:
    """Check filename, ZIP paths, authoritative metadata, ABI and uv licenses."""
    name, version, _build, tags = parse_wheel_filename(path.name)
    row = {"filename": path.name, "distribution": str(name), "version": str(version),
           "sha256": sha256_file(path), "tags": sorted(str(tag) for tag in tags)}
    validate_wheel_row(row, target)
    with zipfile.ZipFile(path) as archive:
        members: dict[str, zipfile.ZipInfo] = {}
        folded: set[str] = set()
        for item in archive.infolist():
            member = item.filename.rstrip("/")
            safe_relative_path(member, f"wheel {path.name} member")
            if member.casefold() in folded:
                raise ValueError(f"wheel {path.name}: duplicate/case-colliding ZIP member {member}")
            folded.add(member.casefold())
            if stat.S_ISLNK(item.external_attr >> 16):
                raise ValueError(f"wheel {path.name}: symlink member {member}")
            if item.flag_bits & 1:
                raise ValueError(f"wheel {path.name}: encrypted ZIP member {member}")
            if not item.is_dir():
                members[member] = item
        # Only top-level .dist-info directories are wheel metadata; packages may
        # legitimately vendor nested dist-infos (setuptools/_vendor, pip/_vendor).
        metadata_members = [member for member in members
                            if member.endswith(".dist-info/METADATA")
                            and "/" not in member.rsplit("/", 1)[0]]
        wheel_members = [member for member in members
                         if member.endswith(".dist-info/WHEEL")
                         and "/" not in member.rsplit("/", 1)[0]]
        if len(metadata_members) != 1 or len(wheel_members) != 1 or metadata_members[0].rsplit("/", 1)[0] != wheel_members[0].rsplit("/", 1)[0]:
            raise ValueError(f"wheel {path.name}: expected one matching top-level .dist-info METADATA/WHEEL pair")
        dist_info = metadata_members[0].rsplit("/", 1)[0]
        if "/" in dist_info or "-" not in dist_info:
            raise ValueError(f"wheel {path.name}: .dist-info must be a matching top-level directory")
        info_name, info_version = dist_info.removesuffix(".dist-info").rsplit("-", 1)
        if canonicalize_name(info_name, validate=True) != name or Version(info_version) != version:
            raise ValueError(f"wheel {path.name}: .dist-info directory disagrees with filename")
        metadata = email.parser.BytesParser().parsebytes(_wheel_member_bytes(archive, metadata_members[0]))
        wheel_metadata = email.parser.BytesParser().parsebytes(_wheel_member_bytes(archive, wheel_members[0]))
        if any(len(metadata.get_all(field, [])) != 1 for field in ("Name", "Version")):
            raise ValueError(f"wheel {path.name}: METADATA must have one Name and Version")
        wheel_versions = wheel_metadata.get_all("Wheel-Version", [])
        if len(wheel_versions) != 1 or re.fullmatch(r"1\.[0-9]+", wheel_versions[0]) is None:
            raise ValueError(f"wheel {path.name}: missing/unsupported Wheel-Version")
        purelib = wheel_metadata.get_all("Root-Is-Purelib", [])
        if len(purelib) != 1 or purelib[0].lower() not in ("true", "false"):
            raise ValueError(f"wheel {path.name}: invalid Root-Is-Purelib")
        if canonicalize_name(metadata.get("Name", ""), validate=True) != name or Version(metadata.get("Version", "")) != version:
            raise ValueError(f"wheel {path.name}: METADATA name/version disagrees with filename")
        metadata_tags = {str(tag) for value in wheel_metadata.get_all("Tag", []) for tag in parse_tag(value)}
        if metadata_tags != set(row["tags"]):
            raise ValueError(f"wheel {path.name}: WHEEL tags disagree with filename")
        requires_python = metadata.get("Requires-Python", "")
        environment = marker_environment(target)
        if requires_python and not SpecifierSet(requires_python).contains(environment["python_full_version"], prereleases=True):
            raise ValueError(f"wheel {path.name}: Requires-Python excludes target interpreter")
        requirements = metadata.get_all("Requires-Dist", [])
        for requirement in requirements:
            parsed = Requirement(requirement)
            if parsed.url:
                raise ValueError(f"wheel {path.name}: direct/source dependency is not an offline registry closure: {requirement}")
        licenses = []
        for member in sorted(members):
            if member.startswith(dist_info + "/") and ("license" in PurePosixPath(member).name.lower() or "/licenses/" in member.lower()):
                content = _wheel_member_bytes(archive, member)
                if content.strip():
                    licenses.append({"path": member, "sha256": hashlib.sha256(content).hexdigest()})
        if name == "uv" and not licenses:
            raise ValueError(f"wheel {path.name}: uv license payload is missing")
        return {"wheel": row, "requires_dist": requirements, "requires_python": requires_python,
                "provides_extra": sorted(canonicalize_name(extra) for extra in metadata.get_all("Provides-Extra", [])),
                "licenses": licenses}


def validate_dependency_closure(records: list[dict[str, Any]], roots: list[str], environment: dict[str, str]) -> dict[str, list[str]]:
    """Follow active metadata edges, including extras, through all exact pins."""
    by_name: dict[str, dict[str, Any]] = {}
    for record in records:
        name = record["wheel"]["distribution"]
        if name in by_name:
            raise ValueError(f"dependency closure: duplicate distribution {name}")
        by_name[name] = record
    pending = [(Requirement(value), "project/tool roots") for value in roots]
    requested_extras: dict[str, set[str]] = {}
    visited: dict[str, frozenset[str]] = {}
    graph: dict[str, list[str]] = {}
    while pending:
        requirement, owner = pending.pop()
        if not _marker_applies(requirement, environment, set()):
            continue
        if requirement.url:
            raise ValueError(f"dependency closure: direct URL dependency from {owner}: {requirement}")
        name = canonicalize_name(requirement.name)
        if name not in by_name:
            raise ValueError(f"dependency closure: missing {requirement} required by {owner}")
        record = by_name[name]
        if not requirement.specifier.contains(record["wheel"]["version"], prereleases=True):
            raise ValueError(f"dependency closure: {requirement} required by {owner}, bundled {record['wheel']['version']}")
        extras = requested_extras.setdefault(name, set())
        extras.update(canonicalize_name(extra) for extra in requirement.extras)
        if not extras <= set(record["provides_extra"]):
            raise ValueError(f"dependency closure: {name} does not provide extras {sorted(extras - set(record['provides_extra']))}")
        current = frozenset(extras)
        if visited.get(name) == current:
            continue
        visited[name] = current
        edges: set[str] = set(graph.get(name, []))
        for value in record["requires_dist"]:
            dependency = Requirement(value)
            if _marker_applies(dependency, environment, extras):
                edges.add(canonicalize_name(dependency.name))
                # Its marker is evaluated in the parent extra context exactly
                # once. Re-evaluation with extra='' would silently drop edges.
                dependency.marker = None
                pending.append((dependency, name))
        graph[name] = sorted(edges)
    unused = set(by_name) - visited.keys()
    if unused:
        raise ValueError(f"dependency closure: undeclared/unreachable wheels {sorted(unused)}")
    return graph


def parse_frozen_requirements(text: str, environment: dict[str, str]) -> list[dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for line in text.replace("\\\n", " ").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        hashes = sorted(set(_HASH_OPTION.findall(line)))
        requirement = Requirement(_HASH_OPTION.sub("", line).strip())
        if requirement.url or not hashes:
            raise ValueError(f"frozen requirements: exact registry pin with SHA256 required: {line}")
        specifiers = list(requirement.specifier)
        if len(specifiers) != 1 or specifiers[0].operator != "==" or "*" in specifiers[0].version:
            raise ValueError(f"frozen requirements: not an exact pin: {requirement}")
        if not _marker_applies(requirement, environment, set()):
            continue
        name = canonicalize_name(requirement.name)
        if name in rows:
            raise ValueError(f"frozen requirements: duplicate active distribution {name}")
        rows[name] = {"name": name, "version": str(Version(specifiers[0].version)), "hashes": hashes,
                      "requirement": str(requirement)}
    if not rows:
        raise ValueError("frozen requirements: empty dependency closure")
    return [rows[name] for name in sorted(rows)]


def declared_roots(project: dict[str, Any], native: bool) -> list[str]:
    roots = list(project["project"]["dependencies"])
    if native:
        groups = project.get("dependency-groups", {})
        for name in ("native", "dev"):
            entries = groups.get(name)
            if not isinstance(entries, list) or not entries:
                raise ValueError(f"pyproject: missing required {name} dependency group")
            for entry in entries:
                if not isinstance(entry, str):
                    raise ValueError(f"pyproject: unsupported nested dependency group in {name}; expand the reviewed frozen group explicitly")
                roots.append(entry)
    return roots


def load_tool_lock(path: Path, wheelhouse: Path, target: dict[str, Any], project: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if not path.is_file():
        raise ValueError(f"missing exact build/tool lock: {path}; use tools/prepare_offline_tools.py with selected qualified tool wheels (do not regenerate uv.lock)")
    value = read_json(path)
    if not isinstance(value, dict) or set(value) != {"schema_version", "target", "uv_version", "wheels"} or type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("tool lock: unsupported schema")
    locked_target = validate_target(value["target"])
    for field in ("system", "machine", "python_minor", "implementation"):
        if locked_target[field] != target[field]:
            raise ValueError(f"tool lock: target {field} mismatch")
    if not isinstance(value["wheels"], list) or not value["wheels"]:
        raise ValueError("tool lock: missing wheel rows")
    records = []
    names: set[str] = set()
    for item in value["wheels"]:
        row = validate_wheel_row(item, target)
        if row["distribution"] in names:
            raise ValueError(f"tool lock: duplicate distribution {row['distribution']}")
        names.add(row["distribution"])
        wheel = confined_file(wheelhouse, row["filename"])
        record = inspect_wheel(wheel, target)
        if record["wheel"] != row:
            raise ValueError(f"tool lock: wheel hash/metadata mismatch for {row['filename']}")
        records.append(record)
    if not TOOL_NAMES <= names:
        raise ValueError(f"tool lock: missing required tools {sorted(TOOL_NAMES - names)}")
    uv = next(record for record in records if record["wheel"]["distribution"] == "uv")
    if uv["wheel"]["version"] != value["uv_version"]:
        raise ValueError("tool lock: uv version mismatch")
    # Top-level selected tools and their actual backend requirements are the
    # roots. Transitive tools must be reachable, not arbitrary added wheels.
    validate_dependency_closure(records, _tool_roots(records, project), marker_environment(target))
    return value, records


def required_native_assets(root: Path) -> set[str]:
    """Explicitly approved benchmark inputs; never silently regenerate them."""
    base = PurePosixPath("tools/proto_native_bench/artifacts")
    pointer_name = str(base / "current.json")
    pointer_file = confined_file(root, pointer_name, source=True)
    pointer = read_json(pointer_file)
    if not isinstance(pointer, dict) or type(pointer.get("format")) is not int or pointer["format"] != 1:
        raise ValueError(f"{pointer_name}: invalid required native benchmark pointer")
    name = safe_relative_path(pointer.get("bundle"), "benchmark bundle")
    if len(name.parts) != 1:
        raise ValueError("benchmark bundle must be one directory name")
    selected = base / name
    manifest_name = str(selected / "manifest.json")
    fixture = read_json(confined_file(root, manifest_name, source=True))
    checksums = fixture.get("checksums") if isinstance(fixture, dict) else None
    if not isinstance(checksums, dict):
        raise ValueError(f"{manifest_name}: missing benchmark asset checksums")
    # The reference tests also load this older shared model directly. It is
    # independent of current.json and must not be substituted with v3 bytes.
    flat_model = str(base / "model.mjb")
    try:
        confined_file(root, flat_model, source=True)
    except ValueError as error:
        raise ValueError(f"missing required native reference asset {flat_model}; restore the original independent model") from error
    required = {pointer_name, manifest_name, flat_model}
    for filename, field in (("model.mjb", "model_mjb_sha256"), ("state.bin", "state_bin_sha256"), ("state.npz", "state_npz_sha256")):
        relative = str(selected / filename)
        try:
            asset = confined_file(root, relative, source=True)
        except ValueError as error:
            raise ValueError(f"missing required native development asset {relative}; restore the original binary input omitted by the source snapshot") from error
        if sha256_file(asset) != checksums.get(field):
            raise ValueError(f"{relative}: required benchmark asset hash mismatch")
        required.add(relative)
    return required


class CommandLog:
    def __init__(self) -> None:
        self.commands: list[dict[str, Any]] = []

    def run(self, argv: list[str], cwd: Path, *, binary: bool = False, env: dict[str, str] | None = None) -> Any:
        record: dict[str, Any] = {"argv": argv, "cwd": str(cwd), "status": "running", "returncode": None}
        self.commands.append(record)
        try:
            completed = subprocess.run(argv, cwd=cwd, env=env, capture_output=True, text=not binary, check=False)
        except OSError as error:
            record.update(status="failed", error=str(error))
            raise ValueError(f"command unavailable: {argv[0]}: {error}") from error
        record.update(returncode=completed.returncode, status="passed" if completed.returncode == 0 else "failed",
                      stdout=completed.stdout.decode("utf-8", "backslashreplace") if binary else completed.stdout,
                      stderr=completed.stderr.decode("utf-8", "backslashreplace") if binary else completed.stderr)
        if completed.returncode != 0:
            raise ValueError(f"command failed ({completed.returncode}): {argv!r}\n{record['stderr']}")
        return completed.stdout


def collect_source_files(root: Path, log: CommandLog, *, native: bool, assets: list[str]) -> tuple[list[tuple[str, Path, int]], dict[str, Any]]:
    """Use tracked *current* bytes, not git archive's committed version."""
    root = root.resolve(strict=True)
    top = Path(log.run(["git", "rev-parse", "--show-toplevel"], root).strip()).resolve()
    if top != root:
        raise ValueError("source root is not the actual Git checkout root; a Repomix snapshot is not a release checkout")
    commit = log.run(["git", "rev-parse", "HEAD"], root).strip()
    status_text = log.run(["git", "status", "--porcelain=v1", "--untracked-files=normal"], root)
    tracked_status = log.run(["git", "status", "--porcelain=v1", "--untracked-files=no"], root)
    entries = log.run(["git", "ls-files", "--stage", "-z"], root, binary=True)
    tracked: dict[str, str] = {}
    for entry in entries.split(b"\0"):
        if not entry:
            continue
        header, raw_path = entry.split(b"\t", 1)
        mode, _oid, stage = header.decode("ascii").split()
        if stage != "0":
            raise ValueError("source checkout contains unresolved Git conflicts")
        name = raw_path.decode("utf-8")
        safe_relative_path(name, "tracked source")
        if mode == "160000":
            raise ValueError(f"source submodule needs an explicit source inventory: {name}")
        tracked[name] = mode
    approved = {str(safe_relative_path(value, "approved asset")) for value in assets}
    if native:
        approved.update(required_native_assets(root))
    selected: list[tuple[str, Path, int]] = []
    excluded: list[str] = []
    for name in sorted(set(tracked) | approved):
        if not include_source_path(PurePosixPath(name)):
            if name in approved:
                raise ValueError(f"approved asset is excluded by source policy: {name}")
            excluded.append(name)
            continue
        candidate = root / name
        if not candidate.exists() and not candidate.is_symlink():
            if name in approved:
                raise ValueError(f"missing approved asset {name}")
            continue  # Preserve an intentional dirty deletion; required paths are checked below.
        generated = False
        for parent in candidate.parents:
            if parent == root:
                break
            if any((parent / marker).exists() for marker in BUILD_MARKERS):
                generated = True
                break
        if generated:
            if name in approved:
                raise ValueError(f"approved asset is inside a generated build/venv tree: {name}")
            excluded.append(name)
            continue
        source = confined_file(root, name, source=True)
        selected.append((name, source, stat.S_IMODE(source.stat().st_mode) & 0o777))
    paths = {name for name, _source, _mode in selected}
    required = {"pyproject.toml", "uv.lock", "src/bike_sim/__init__.py", "README.md"}
    if native:
        required |= _NATIVE_REQUIRED | approved
        for prefix in ("native/src/", "native/cmake/", "native/tests/"):
            if not any(name.startswith(prefix) for name in paths):
                raise ValueError(f"missing required tracked native source directory {prefix}")
    if required - paths:
        raise ValueError(f"missing tracked/approved required source files {sorted(required - paths)}; restore omitted source/lock inputs or explicitly approve required assets")
    return selected, {"git_commit": commit, "dirty": bool(status_text) or bool(tracked_status) or bool(approved - tracked.keys()),
                      "approved_assets": sorted(approved), "excluded": excluded,
                      "tracked_status_sha256": hashlib.sha256(tracked_status.encode()).hexdigest(),
                      "index_sha256": hashlib.sha256(entries).hexdigest(),
                      "source_modes": {name: mode for name, _source, mode in selected}}


def _select_locked_wheel(row: dict[str, Any], lock: dict[str, Any], target: dict[str, Any]) -> dict[str, Any]:
    packages = [item for item in lock.get("package", []) if canonicalize_name(item.get("name", "")) == row["name"] and str(item.get("version")) == row["version"]]
    if len(packages) != 1:
        raise ValueError(f"uv.lock: missing or ambiguous exact registry package {row['name']}=={row['version']}")
    package = packages[0]
    if "registry" not in package.get("source", {}):
        raise ValueError(f"uv.lock: source-only/local dependency cannot be bundled as a locked wheel: {row['name']}")
    rank = {tag: index for index, tag in enumerate(target["accepted_wheel_tags"])}
    candidates = []
    for item in package.get("wheels", []):
        url = item.get("url", "")
        parts = urllib.parse.urlsplit(url)
        if parts.scheme != "https" or parts.username or parts.password:
            raise ValueError(f"uv.lock: require credential-free HTTPS wheel URL for {row['name']}")
        filename = urllib.parse.unquote(parts.path.rsplit("/", 1)[-1])
        safe_relative_path(filename, "locked wheel filename")
        name, version, _build, tags = parse_wheel_filename(filename)
        if str(name) != row["name"] or str(version) != row["version"]:
            raise ValueError(f"uv.lock: wheel identity mismatch for {filename}")
        digest = item.get("hash", "").removeprefix("sha256:")
        if digest not in row["hashes"]:
            continue
        matches = [rank[str(tag)] for tag in tags if str(tag) in rank]
        if matches:
            candidates.append((min(matches), filename, {"url": url, "filename": filename, "sha256": digest}))
    if not candidates:
        raise ValueError(f"no hash-locked binary wheel matches target for {row['name']}=={row['version']}; source builds/download fallbacks are forbidden")
    return min(candidates, key=lambda value: (value[0], value[1]))[2]


def _download_wheel(item: dict[str, Any], wheelhouse: Path, downloads: list[dict[str, Any]]) -> Path:
    destination = wheelhouse / item["filename"]
    record = dict(item, status="running")
    downloads.append(record)
    try:
        with urllib.request.urlopen(item["url"], timeout=60) as response, destination.open("xb") as output:
            if urllib.parse.urlsplit(response.geturl()).scheme != "https":
                raise ValueError("wheel redirect downgraded HTTPS")
            shutil.copyfileobj(response, output)
        if sha256_file(destination) != item["sha256"]:
            raise ValueError(f"downloaded wheel SHA256 mismatch: {destination.name}")
    except Exception as error:
        record.update(status="failed", error=str(error))
        raise
    record["status"] = "passed"
    return destination


def _tool_roots(records: list[dict[str, Any]], project: dict[str, Any]) -> list[str]:
    versions = {item["wheel"]["distribution"]: item["wheel"]["version"] for item in records}
    return [f"{name}=={versions[name]}" for name in sorted(TOOL_NAMES)] + project["build-system"]["requires"]


def _host_build_tools(records: list[dict[str, Any]], project: dict[str, Any]) -> list[dict[str, Any]]:
    """Cross-downloads do not make target uv/tools executable on the author."""
    selected = [record for record in records if record["wheel"]["distribution"] != "uv"]
    host_tags = {str(tag) for tag in sys_tags()}
    environment = default_environment()
    for record in selected:
        if not host_tags.intersection(record["wheel"]["tags"]):
            raise ValueError(f"Python-only cross assembly requires host-compatible build tools: {record['wheel']['filename']}")
        if record["requires_python"] and not SpecifierSet(record["requires_python"]).contains(environment["python_full_version"], prereleases=True):
            raise ValueError(f"Python-only cross assembly build tool excludes the host Python: {record['wheel']['filename']}")
    versions = {record["wheel"]["distribution"]: record["wheel"]["version"] for record in selected}
    roots = [f"{name}=={versions[name]}" for name in sorted(TOOL_NAMES - {"uv"})] + project["build-system"]["requires"]
    try:
        validate_dependency_closure(selected, roots, environment)
    except ValueError as error:
        raise ValueError(f"Python-only cross assembly host build-tool closure is incomplete: {error}") from error
    return selected


def _json_write(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _archive_name(target: dict[str, Any], mode: str, native: bool, legacy_names: bool) -> str:
    python_tag = "py" + target["python_minor"].replace(".", "")
    if legacy_names:
        if native or (target["system"], target["machine"], target["python_minor"]) != ("Linux", "x86_64", "3.13"):
            raise ValueError("legacy names apply only to explicitly Python-only Linux x64 CPython 3.13 bundles")
        suffix = "" if mode == "full" else "-slim"
        return f"bike-sim-offline-{python_tag}-linux-x64{suffix}"
    system = "linux" if target["system"] == "Linux" else "macos"
    capability = "native-source" if native else "python-only"
    return f"bike-sim-offline-{python_tag}-{system}-{target['machine']}-{mode}-{capability}"


def _system_prerequisites(target: dict[str, Any], native: bool) -> list[str]:
    libraries = ([target["libc_floor"], "Linux desktop GLFW/OpenGL/X11 or XWayland libraries for optional viewers"]
                 if target["system"] == "Linux" else
                 ["macOS >=14.0 (or the higher deployment target in accepted wheel tags)", "macOS window server for optional mjpython viewers"])
    if native:
        libraries += ["preinstalled CMake >=3.27 and Ninja or Make",
                      "preinstalled strict ISO C++23 compiler and standard library; local CMake feature probes must pass",
                      "analysis/sanitizer/coverage tools specified by docs/TESTING.md for the development gate"]
        libraries.append("Apple SDK and Accelerate" if target["system"] == "Darwin" else "matching NumPy ILP64 OpenBLAS runtime provider from the frozen scientific wheel")
    return libraries


def _assert_source_payload_complete(root: Path, rows: list[dict[str, str]], native: bool) -> None:
    names = {row["path"] for row in rows}
    actual: set[str] = set()
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"source payload must contain copied regular files, not symlinks: {path.relative_to(root)}")
        if path.is_file():
            actual.add(path.relative_to(root).as_posix())
        elif not path.is_dir():
            raise ValueError(f"source payload contains nonregular member {path.relative_to(root)}")
    if actual != names:
        raise ValueError(f"source inventory differs from payload: missing={sorted(names - actual)}, undeclared={sorted(actual - names)}")
    for row in rows:
        path = confined_file(root, row["path"], source=True)
        if sha256_file(path) != row["sha256"]:
            raise ValueError(f"source SHA256 mismatch: {row['path']}")
    if native:
        assets = required_native_assets(root)
        if assets - names:
            raise ValueError(f"missing required native asset inventory {sorted(assets - names)}")
        # Concrete, literal source/CMake inputs referenced by the build must be
        # present too. Generated includes with variables are validated by the
        # local CMake build in T25, not fabricated here.
        for build_file in [root / "native/CMakeLists.txt", *(root / "native/cmake").rglob("*.cmake")]:
            text = build_file.read_text(encoding="utf-8")
            for relative in re.findall(r"(?<![\w])(?:src|tests|cmake)/[A-Za-z0-9_./-]+\.(?:cpp|hpp|h|in|cmake)(?![A-Za-z0-9_.])", text):
                name = "native/" + relative
                if name not in names:
                    raise ValueError(f"missing native build input {name}, referenced by {build_file.relative_to(root)}")
            for name in re.findall(r"(?<![\w])tools/[A-Za-z0-9_./-]+\.(?:py|sh|cpp|hpp)(?![A-Za-z0-9_.])", text):
                if name not in names:
                    raise ValueError(f"missing native build tool {name}, referenced by {build_file.relative_to(root)}")


def _compare_project_wheel(path: Path, source_root: Path) -> None:
    """The compatibility wheel must contain exactly the staged Python sources."""
    expected = {item.relative_to(source_root / "src").as_posix(): sha256_file(item)
                for item in (source_root / "src/bike_sim").rglob("*.py") if item.is_file()}
    if not expected or "bike_sim/__init__.py" not in expected:
        raise ValueError("Python wheel/source comparison requires a real src/bike_sim source tree")
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        if len(set(names)) != len(names):
            raise ValueError("Python project wheel contains duplicate members")
        for name in names:
            safe_relative_path(name.rstrip("/"), "Python project wheel member")
        actual = {name: hashlib.sha256(archive.read(name)).hexdigest()
                  for name in archive.namelist() if name.startswith("bike_sim/") and name.endswith(".py")}
        if any(name.endswith((".so", ".dylib", ".dll", ".pyd")) for name in archive.namelist()):
            raise ValueError("Python-only project wheel contains a native binary")
    if expected != actual:
        raise ValueError("Python project wheel does not match bundled source bytes")


def check_bundle(bundle: Path, *, require_host: bool = False) -> dict[str, Any]:
    """Validate extracted assembly data without installing, importing or building."""
    bundle = bundle.resolve(strict=True)
    if any(path.is_symlink() for path in bundle.rglob("*")):
        raise ValueError("bundle must contain regular payloads, not symlinks")
    manifest = validate_bundle_manifest(read_json(confined_file(bundle, "manifest.json")))
    target = manifest["target"]
    native = manifest["capabilities"]["native_source_build"]
    if require_host:
        if (target["system"], target["machine"], target["python_minor"]) != _host_identity():
            raise ValueError("bundle target does not match current OS/architecture/Python")
        actual_tags = {str(tag) for tag in sys_tags()}
        if any(not actual_tags.intersection(row["tags"]) for row in manifest["wheels"]):
            raise ValueError("bundle has wheels incompatible with current host tags")
    source_root = bundle / "project"
    _assert_source_payload_complete(source_root, manifest["source_files"], native)
    requirements_path = confined_file(bundle, "requirements.txt")
    if sha256_file(requirements_path) != manifest["requirements"]["requirements_sha256"]:
        raise ValueError("requirements.txt hash mismatch")
    project = tomllib.loads((source_root / "pyproject.toml").read_text(encoding="utf-8"))
    if (project["project"]["name"], project["project"]["version"]) != (manifest["project"]["name"], manifest["project"]["version"]):
        raise ValueError("manifest project identity differs from pyproject.toml")
    report = read_json(confined_file(bundle, "assembly-report.json"))
    if (not isinstance(report, dict) or type(report.get("schema_version")) is not int
            or report["schema_version"] != 1 or report.get("installation_qualification") != "pending-T25"
            or report.get("status") != "assembled-not-installed" or report.get("native_extension_built") is not False):
        raise ValueError("assembly report must explicitly leave installation qualification pending T25")
    for filename, key in (("offline-tools.json", "tool_lock_sha256"), ("dependency-closure.json", "dependency_closure_sha256")):
        if sha256_file(confined_file(bundle, filename)) != report.get(key):
            raise ValueError(f"assembly report hash mismatch: {filename}")
    closure = read_json(confined_file(bundle, "dependency-closure.json"))
    if (not isinstance(closure, dict) or type(closure.get("schema_version")) is not int
            or closure["schema_version"] != 1 or closure.get("target") != target):
        raise ValueError("dependency closure schema/target mismatch")
    environment = closure.get("marker_environment")
    if not isinstance(environment, dict) or set(environment) != set(default_environment()) or any(not isinstance(value, str) for value in environment.values()):
        raise ValueError("dependency closure marker environment is incomplete")
    for key, expected in (("platform_system", target["system"]), ("platform_machine", target["machine"]),
                          ("python_version", target["python_minor"]), ("implementation_name", "cpython"),
                          ("sys_platform", "linux" if target["system"] == "Linux" else "darwin"),
                          ("os_name", "posix"), ("platform_python_implementation", "CPython")):
        if environment[key] != expected:
            raise ValueError(f"dependency closure marker target mismatch: {key}")
    expected_python = python_minor(target["python_minor"])
    for key in ("python_full_version", "implementation_version"):
        if Version(environment[key]).release[:2] != expected_python:
            raise ValueError(f"dependency closure marker target mismatch: {key}")
    selected = parse_frozen_requirements(requirements_path.read_text(encoding="utf-8"), environment)
    lock = tomllib.loads((source_root / "uv.lock").read_text(encoding="utf-8"))
    expected_app = {row["name"]: _select_locked_wheel(row, lock, target) for row in selected}
    records = closure.get("wheels")
    if not isinstance(records, list):
        raise ValueError("dependency closure is missing wheel metadata")
    by_name: dict[str, dict[str, Any]] = {}
    for record in records:
        if not isinstance(record, dict) or set(record) != {"wheel", "requires_dist", "requires_python", "provides_extra", "licenses"}:
            raise ValueError("invalid wheel metadata in dependency closure")
        if (not isinstance(record["requires_python"], str)
                or any(not isinstance(record[field], list) or any(not isinstance(item, str) for item in record[field])
                       for field in ("requires_dist", "provides_extra"))
                or not isinstance(record["licenses"], list)):
            raise ValueError("invalid wheel metadata field types in dependency closure")
        row = validate_wheel_row(record["wheel"], target)
        if row["distribution"] in by_name:
            raise ValueError("duplicate wheel distribution in dependency closure")
        by_name[row["distribution"]] = record
    tool_lock = read_json(confined_file(bundle, "offline-tools.json"))
    if (not isinstance(tool_lock, dict) or set(tool_lock) != {"schema_version", "target", "uv_version", "wheels"}
            or type(tool_lock["schema_version"]) is not int or tool_lock["schema_version"] != 1):
        raise ValueError("invalid supplemental tool lock schema")
    validate_target(tool_lock["target"])
    for key in ("system", "machine", "python_minor", "implementation"):
        if tool_lock["target"][key] != target[key]:
            raise ValueError("supplemental tool lock target mismatch")
    tool_rows: dict[str, dict[str, Any]] = {}
    for value in tool_lock["wheels"]:
        row = validate_wheel_row(value, target)
        if row["distribution"] in tool_rows:
            raise ValueError("duplicate distribution in supplemental tool lock")
        tool_rows[row["distribution"]] = row
    expected_names = set(expected_app) | set(tool_rows) | (set() if native else {"bike-sim"})
    if set(by_name) != expected_names:
        raise ValueError("dependency closure differs from exact application/tool lock distribution set")
    for name, expected in expected_app.items():
        actual = by_name[name]["wheel"]
        if (actual["filename"], actual["sha256"]) != (expected["filename"], expected["sha256"]):
            raise ValueError(f"wheel does not match frozen uv.lock/export hash: {name}")
    for name, expected in tool_rows.items():
        if by_name[name]["wheel"] != expected:
            raise ValueError(f"wheel differs from supplemental tool lock: {name}")
    if tool_lock["uv_version"] != manifest["prerequisites"]["uv_version"]:
        raise ValueError("supplemental uv version differs from manifest")
    bundled = {row["distribution"]: row for row in manifest["wheels"]}
    external = {row["distribution"]: row for row in manifest["slim_external"]}
    if set(bundled) | set(external) != set(by_name):
        raise ValueError("bundle/external inventory does not close the dependency graph")
    wheelhouse = bundle / "wheelhouse"
    actual_filenames = {path.name for path in wheelhouse.iterdir() if path.is_file()}
    if actual_filenames != {row["filename"] for row in bundled.values()} or any(not path.is_file() or path.is_symlink() for path in wheelhouse.iterdir()):
        raise ValueError("wheelhouse has missing, undeclared or nonregular payload")
    for name, row in bundled.items():
        record = inspect_wheel(confined_file(wheelhouse, row["filename"]), target)
        if record != by_name[name] or record["wheel"] != row:
            raise ValueError(f"wheel hash/metadata mismatch: {row['filename']}")
    for name, row in external.items():
        if name in tool_rows:
            raise ValueError(f"slim bundle cannot omit a build/tool dependency: {name}")
        if row["version"] != by_name[name]["wheel"]["version"]:
            raise ValueError(f"slim external exact version mismatch: {name}")
    roots = declared_roots(project, native) + _tool_roots(records, project)
    if not native:
        roots.append(f"bike-sim=={project['project']['version']}")
    if closure.get("roots") != roots:
        raise ValueError("dependency closure roots differ from actual project/tool requirements")
    graph = validate_dependency_closure(records, roots, environment)
    if closure.get("dependency_edges") != graph:
        raise ValueError("dependency closure edge evidence mismatch")
    source_report = report.get("source")
    modes = source_report.get("source_modes") if isinstance(source_report, dict) else None
    if not isinstance(modes, dict) or set(modes) != {row["path"] for row in manifest["source_files"]}:
        raise ValueError("source executable-mode inventory is incomplete")
    for relative, mode in modes.items():
        if type(mode) is not int or not 0 <= mode <= 0o777 or stat.S_IMODE((source_root / relative).stat().st_mode) != mode:
            raise ValueError(f"source file mode mismatch: {relative}")
    if not native:
        _compare_project_wheel(wheelhouse / bundled["bike-sim"]["filename"], source_root)
    fixed_files = {"manifest.json", "requirements.txt", "offline-tools.json", "dependency-closure.json", "assembly-report.json", "README.md"}
    if {path.name for path in bundle.iterdir()} != fixed_files | {"project", "wheelhouse"}:
        raise ValueError("bundle root contains undeclared or missing files")
    for name in fixed_files:
        confined_file(bundle, name)
    return manifest


def assemble(args: argparse.Namespace) -> Path:
    root = args.project.resolve(strict=True)
    # Fail before network, build, or output creation when an extraction omitted
    # source inputs. Restoring the actual lock is the owner's responsibility.
    if not (root / "uv.lock").is_file():
        raise ValueError("uv.lock is absent: this source snapshot cannot assemble frozen bundles; restore the actual checkout's committed lock, never regenerate it for this operation")
    project = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))
    if project["project"]["name"] != "bike-sim":
        raise ValueError("expected bike-sim project root")
    target = select_target(args.target, args.python, args.native)
    environment = marker_environment(target)
    if not SpecifierSet(project["project"]["requires-python"]).contains(environment["python_full_version"], prereleases=True):
        raise ValueError("project requires-python excludes requested target")
    declared_roots(project, args.native)
    tool_path = args.tool_lock or root / "packaging/offline-tools.json"
    tool_path = tool_path.resolve()
    tool_wheelhouse = (args.tool_wheelhouse or tool_path.parent / "offline-tool-wheels").resolve()
    tool_lock, tool_records = load_tool_lock(tool_path, tool_wheelhouse, target, project)
    host_build_tools = [] if args.native else _host_build_tools(tool_records, project)
    name = _archive_name(target, args.mode, args.native, args.legacy_names)
    output = args.output.resolve()
    final = output / (name + ".tar.gz")
    if final.exists():
        raise ValueError(f"refusing to overwrite existing bundle {final}")
    log = CommandLog()
    sources, source_evidence = collect_source_files(root, log, native=args.native, assets=args.asset)
    source_initial = {name: sha256_file(path) for name, path, _mode in sources}
    uv = shutil.which("uv")
    if uv is None:
        raise ValueError(f"preinstall qualified uv=={tool_lock['uv_version']}; assembler never bootstraps or upgrades its own runner")
    actual_uv = log.run([uv, "--version"], root).strip().split()
    if len(actual_uv) < 2 or actual_uv[0] != "uv" or actual_uv[1] != tool_lock["uv_version"]:
        raise ValueError(f"invoked uv version differs from exact supplemental tool lock {tool_lock['uv_version']}")
    output.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=".offline-build-", dir=output))
    bundle = work / name
    bundle.mkdir()
    staged = bundle / "project"
    staged.mkdir()
    all_wheels = work / "all-wheels"
    all_wheels.mkdir()
    wheelhouse = bundle / "wheelhouse"
    wheelhouse.mkdir()
    report: dict[str, Any] = {"schema_version": 1, "status": "assembling", "installation_qualification": "pending-T25",
                              "commands": log.commands, "downloads": [], "source": source_evidence,
                              "native_extension_built": False, "tool_lock_sha256": sha256_file(tool_path)}
    try:
        rows = []
        for relative, source, mode in sources:
            destination = staged / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            destination.chmod(mode)
            digest = sha256_file(destination)
            if digest != source_initial[relative]:
                raise ValueError(f"source changed while staging: {relative}")
            rows.append({"path": relative, "sha256": digest})
        _assert_source_payload_complete(staged, rows, args.native)
        shutil.copyfile(tool_path, bundle / "offline-tools.json")
        lock_sha = sha256_file(staged / "uv.lock")
        tool_env = dict(os.environ, UV_PYTHON_DOWNLOADS="never", UV_CACHE_DIR=str(work / "uv-cache"),
                        UV_PROJECT_ENVIRONMENT=str(work / "tool-environment"), PIP_CONFIG_FILE=os.devnull)
        # --check proves the current (possibly dirty) pyproject still matches
        # the lock. --frozen alone would silently accept stale dependency data.
        log.run([uv, "lock", "--check", "--offline", "--python", sys.executable], staged, env=tool_env)
        requirements = bundle / "requirements.txt"
        command = [uv, "export", "--frozen", "--no-emit-project", "--output-file", str(requirements)]
        command += ["--group", "native", "--group", "dev"] if args.native else ["--no-dev"]
        log.run(command, staged, env=tool_env)
        if sha256_file(staged / "uv.lock") != lock_sha:
            raise ValueError("frozen export unexpectedly changed uv.lock")
        resolved = parse_frozen_requirements(requirements.read_text(encoding="utf-8"), environment)
        lock = tomllib.loads((staged / "uv.lock").read_text(encoding="utf-8"))
        records: dict[str, dict[str, Any]] = {}
        for requirement in resolved:
            item = _select_locked_wheel(requirement, lock, target)
            wheel = _download_wheel(item, all_wheels, report["downloads"])
            record = inspect_wheel(wheel, target)
            records[record["wheel"]["distribution"]] = record
        for record in tool_records:
            row = record["wheel"]
            if row["distribution"] in records:
                if records[row["distribution"]]["wheel"] != row:
                    raise ValueError(f"tool/application lock conflict for {row['distribution']}; select the same exact locked wheel")
            else:
                shutil.copyfile(tool_wheelhouse / row["filename"], all_wheels / row["filename"])
                records[row["distribution"]] = record
        if not args.native:
            build_source = work / "wheel-project"
            shutil.copytree(staged, build_source)
            dist = work / "project-wheel"
            dist.mkdir()
            command = [uv, "run", "--offline", "--no-project", "--python", sys.executable]
            for record in host_build_tools:
                command += ["--with", str(all_wheels / record["wheel"]["filename"])]
            command += ["python", "-m", "pip", "wheel", "--no-index", "--no-deps", "--no-build-isolation",
                        "--wheel-dir", str(dist), str(build_source)]
            log.run(command, work, env=tool_env)
            built = list(dist.glob("*.whl"))
            if len(built) != 1:
                raise ValueError("Python-only project build must produce exactly one wheel")
            record = inspect_wheel(built[0], target)
            if record["wheel"]["distribution"] != "bike-sim" or record["wheel"]["tags"] != ["py3-none-any"]:
                raise ValueError("compatibility route must produce a pure Python bike-sim wheel")
            _compare_project_wheel(built[0], staged)
            shutil.copyfile(built[0], all_wheels / built[0].name)
            records["bike-sim"] = record
        ordered_records = [records[name] for name in sorted(records)]
        roots = declared_roots(project, args.native) + _tool_roots(ordered_records, project)
        if not args.native:
            roots.append(f"bike-sim=={project['project']['version']}")
        graph = validate_dependency_closure(ordered_records, roots, environment)
        omitted = {canonicalize_name(name, validate=True) for name in (args.slim_external or DEFAULT_SLIM_EXTERNAL)} if args.mode == "slim" else set()
        tool_names = {record["wheel"]["distribution"] for record in tool_records}
        if omitted - records.keys() or omitted.intersection({"bike-sim", "nanobind", "hypothesis", "threadpoolctl"} | tool_names):
            raise ValueError(f"invalid slim omissions {sorted(omitted)}; external packages must be active scientific dependencies, not project/build/development tools")
        if args.mode == "full" and args.slim_external:
            raise ValueError("--slim-external requires --mode slim")
        included = []
        for record in ordered_records:
            row = record["wheel"]
            if row["distribution"] not in omitted:
                shutil.copyfile(all_wheels / row["filename"], wheelhouse / row["filename"])
                included.append(row)
        manifest = {"schema_version": SCHEMA_VERSION,
                    "project": {"name": project["project"]["name"], "version": project["project"]["version"],
                                "git_commit": source_evidence["git_commit"], "dirty": source_evidence["dirty"],
                                "python_source_sha256": source_digest(rows, "src/"), "native_source_sha256": source_digest(rows, "native/")},
                    "target": target, "mode": args.mode,
                    "capabilities": {"python": True, "native_source_build": args.native, "development": args.native,
                                     "gui": {"glfw", "pyopengl", "matplotlib"} <= records.keys()},
                    "requirements": {"lock_sha256": lock_sha, "requirements_sha256": sha256_file(requirements),
                                     "groups": ["native", "dev"] if args.native else []},
                    "wheels": included, "source_files": rows,
                    "prerequisites": {"python": f"CPython {target['python_minor']}.*",
                                      "compiler_feature_profile": "bike-cxx23-strict-local-probes-required" if args.native else None,
                                      "cmake_minimum": "3.27" if args.native else None,
                                      "uv_version": tool_lock["uv_version"], "system_libraries": _system_prerequisites(target, args.native)},
                    "slim_external": [{"distribution": name, "version": records[name]["wheel"]["version"],
                                       "qualification_profile": "native-exact-provider-pending-T25" if args.native else "python-exact-import-pending-T25"}
                                      for name in sorted(omitted)]}
        validate_bundle_manifest(manifest)
        _json_write(bundle / "manifest.json", manifest)
        _json_write(bundle / "dependency-closure.json", {"schema_version": 1, "target": target, "marker_environment": environment,
                                                        "roots": roots, "wheels": ordered_records, "dependency_edges": graph})
        report["dependency_closure_sha256"] = sha256_file(bundle / "dependency-closure.json")
        report["status"] = "assembled-not-installed"
        _json_write(bundle / "assembly-report.json", report)
        readme = staged / "packaging" / f"OFFLINE_README_{args.mode}.md"
        if not readme.is_file():
            raise ValueError(f"missing tracked offline instructions: {readme.name}")
        shutil.copyfile(readme, bundle / "README.md")
        check_bundle(bundle)
        for relative, source, mode in sources:
            if sha256_file(source) != source_initial[relative] or stat.S_IMODE(source.stat().st_mode) != mode:
                raise ValueError(f"source changed during assembly; rerun from stable checkout: {relative}")
        if log.run(["git", "rev-parse", "HEAD"], root).strip() != source_evidence["git_commit"]:
            raise ValueError("Git HEAD changed during assembly")
        final_status = log.run(["git", "status", "--porcelain=v1", "--untracked-files=no"], root)
        final_index = log.run(["git", "ls-files", "--stage", "-z"], root, binary=True)
        if (hashlib.sha256(final_status.encode()).hexdigest() != source_evidence["tracked_status_sha256"]
                or hashlib.sha256(final_index).hexdigest() != source_evidence["index_sha256"]):
            raise ValueError("Git tracked status/index changed during assembly; retry from a stable checkout")
        _json_write(bundle / "assembly-report.json", report)
        # Only this explicitly inventoried directory goes into the archive;
        # online caches, Python wheel build files and logs outside it stay out.
        candidate = work / (name + ".tar.gz")
        with tarfile.open(candidate, "w:gz", format=tarfile.PAX_FORMAT) as archive:
            for path in sorted(bundle.rglob("*")):
                if path.is_file():
                    archive.add(path, arcname=f"{name}/{path.relative_to(bundle).as_posix()}", recursive=False)
        # Hard-link publication is atomic and refuses an existing destination.
        # The temporary build root lives on the same filesystem as --output.
        os.link(candidate, final)
        shutil.rmtree(work)
        return final
    except Exception as error:
        report.update(status="failed", error=str(error))
        _json_write(work / "assembly-failure.json", report)
        raise ValueError(f"offline assembly failed; diagnostics retained at {work / 'assembly-failure.json'}: {error}") from error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--target", choices=("host", "linux-x86_64", "macos-arm64"), default="host")
    parser.add_argument("--python", default=f"{sys.version_info.major}.{sys.version_info.minor}")
    parser.add_argument("--mode", choices=("full", "slim"), default="full")
    parser.add_argument("--native", action="store_true", help="include frozen native+dev source/build capability; requires target host")
    parser.add_argument("--output", type=Path, default=Path("output/offline-bundles"))
    parser.add_argument("--tool-lock", type=Path, help="exact local tool lock; default packaging/offline-tools.json")
    parser.add_argument("--tool-wheelhouse", type=Path, help="validated tool wheels; default offline-tool-wheels beside tool lock")
    parser.add_argument("--asset", action="append", default=[], help="explicitly approved required source asset relative to --project")
    parser.add_argument("--slim-external", action="append", help="explicit exact-provider omission; repeat for each dependency")
    parser.add_argument("--legacy-names", action="store_true", help="reconstructed Linux cp313 Python-only archive names")
    args = parser.parse_args(argv)
    try:
        result = assemble(args)
    except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile, tomllib.TOMLDecodeError) as error:
        print(f"offline bundle: {error}", file=sys.stderr)
        return 1
    print(f"Assembled {result}; installation/native/GUI qualification remains pending T25/T19.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
