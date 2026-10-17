#!/usr/bin/env python3
"""Describe and package supplied sources/wheels without resolving or running them.

Only Python's standard library is needed. A valid manifest is content evidence;
it is deliberately not evidence of installation, dependency closure, a native
build, binary deployment compatibility, or GUI acceptance on its named target.
"""

from __future__ import annotations

import argparse
from configparser import ConfigParser, Error as ConfigError
from email.parser import BytesParser
from hashlib import sha256
from itertools import product
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import stat
import sys
import tempfile
import tomllib
import unicodedata
from zipfile import BadZipFile, ZIP_DEFLATED, ZipFile, ZipInfo


MANIFEST = "bundle-manifest.json"
FORMAT = "bike-sim-offline"
TARGETS = {
    "ubuntu-24.04-x86_64-cp313": {
        "system": "linux", "machine": "x86_64", "python": "3.13",
        "distribution": "ubuntu", "release": "24.04",
    },
    "macos-arm64-cp313": {
        "system": "darwin", "machine": "arm64", "python": "3.13",
        "distribution": "macos", "release": "unspecified",
    },
    "macos-x86_64-cp313": {
        "system": "darwin", "machine": "x86_64", "python": "3.13",
        "distribution": "macos", "release": "unspecified",
    },
    "macos-arm64-cp314": {
        "system": "darwin", "machine": "arm64", "python": "3.14",
        "distribution": "macos", "release": "unspecified",
    },
    "macos-x86_64-cp314": {
        "system": "darwin", "machine": "x86_64", "python": "3.14",
        "distribution": "macos", "release": "unspecified",
    },
}
QUALIFICATION = {
    "installation": "not_run", "native_build": "not_run",
    "gui": "not_run", "binary_compatibility": "not_run",
}
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")
_BINARY = re.compile(r"\.(?:so(?:\..*)?|dylib|dll|pyd|a|lib|o|obj|exe|pyc|pyo|whl)\Z", re.IGNORECASE)
_PIN = re.compile(r"([A-Za-z0-9](?:[A-Za-z0-9_.-]*[A-Za-z0-9])?)==([A-Za-z0-9][A-Za-z0-9_.+!-]*)\Z")
_SKIP_DIRS = {
    ".git", ".hg", ".svn", ".deps", ".venv", "venv", "env", "__pycache__",
    ".pytest_cache", ".mypy_cache", ".ruff_cache", ".cache", ".tox", ".nox",
    "build", "dist", "wheelhouse", "node_modules", "CMakeFiles", ".idea", ".vscode",
    ".agents", ".ai", ".codex", ".junie", ".work", ".worktrees",
}
_ROOT_OUTPUTS = {"output", "results", "upload"}
_ROOT_GENERATED = {"MUJOCO_LOG.TXT", "bike_model.xml", "coordinates.json", MANIFEST}
_SOURCE_DOTFILES = {
    ".github", ".gitignore", ".gitattributes", ".gitmodules", ".python-version",
    ".clangd", ".clang-tidy", ".clang-format", ".editorconfig", ".coveragerc",
    ".pre-commit-config.yaml", ".ruff.toml", ".cmake-format.yaml", ".cmake-format.json",
}
_GENERATED_FILES = {"CMakeCache.txt", "compile_commands.json", "cmake_install.cmake", "build.ninja", "rules.ninja"}


class BundleError(ValueError):
    """A bundle is malformed, stale, unsafe, or contradicts its declaration."""


def _keys(value: object, expected: set[str], where: str) -> dict:
    if not isinstance(value, dict) or set(value) != expected:
        raise BundleError(f"{where}: expected exactly these fields: {', '.join(sorted(expected))}")
    return value


def _text(value: object, where: str) -> str:
    if not isinstance(value, str) or not value or any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise BundleError(f"{where}: expected nonempty single-line text")
    return value


def _strings(value: object, where: str) -> list[str]:
    if not isinstance(value, list):
        raise BundleError(f"{where}: expected a list")
    return [_text(item, f"{where}[{index}]") for index, item in enumerate(value)]


def _relative_path(value: object) -> str:
    name = _text(value, "path")
    path = PurePosixPath(name)
    if (path.is_absolute() or "\\" in name or ":" in name or path.as_posix() != name
            or any(part in ("", ".", "..") for part in name.split("/"))):
        raise BundleError(f"invalid relative path: {name!r}")
    return name


def _digest(path: Path) -> str:
    result = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def _unique_json(pairs: list[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise BundleError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _read_manifest(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_json)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BundleError(f"cannot read {MANIFEST}: {exc}") from exc


def _target(value: object) -> dict:
    result = _keys(value, {"system", "machine", "python", "distribution", "release"}, "target")
    if result not in TARGETS.values():
        raise BundleError("unsupported target; select Ubuntu 24.04 x86_64 CPython 3.13 or a listed macOS CPython target")
    return result


def _skip_part(name: str) -> bool:
    return (name in _SKIP_DIRS or name.startswith((".venv-", "build-", "build_", "cmake-build-", "bike-sim-offline-"))
            or name.endswith(".egg-info"))


def _source_path_allowed(name: str) -> bool:
    parts = PurePosixPath(name).parts
    return (not any(_skip_part(part) for part in parts)
            and parts[0] not in _ROOT_OUTPUTS
            and not any(part.startswith(".") and part not in _SOURCE_DOTFILES
                        for part in parts)
            and parts[-1] not in _GENERATED_FILES
            and not _BINARY.search(parts[-1])
            and not (len(parts) == 1 and (parts[-1] in _ROOT_GENERATED
                                         or parts[-1].startswith("bike_playground"))))


def _walk_error(error: OSError) -> None:
    raise BundleError(f"cannot enumerate source/bundle directory: {error}") from error


def _regular_files(root: Path) -> list[Path]:
    if root.is_symlink() or not root.is_dir():
        raise BundleError(f"bundle/source root must be a regular directory, not a symlink: {root}")
    result = []
    for directory, folders, files in os.walk(root, followlinks=False, onerror=_walk_error):
        for name in folders:
            if (Path(directory) / name).is_symlink():
                raise BundleError(f"symlink directory is forbidden: {Path(directory) / name}")
        for name in files:
            path = Path(directory) / name
            if path.is_symlink() or not stat.S_ISREG(path.stat().st_mode):
                raise BundleError(f"symlink/nonregular file is forbidden: {path}")
            result.append(path)
    return sorted(result, key=lambda path: path.relative_to(root).as_posix())


def _copy_sources(source: Path, destination: Path) -> None:
    if source.is_symlink() or not source.is_dir():
        raise BundleError("source must be a regular directory, not a symlink")
    for directory, folders, files in os.walk(source, followlinks=False, onerror=_walk_error):
        relative_dir = Path(directory).relative_to(source)
        folders[:] = sorted(name for name in folders
                            if _source_path_allowed((relative_dir / name).as_posix()))
        for name in folders:
            if (Path(directory) / name).is_symlink():
                raise BundleError(f"source symlink directory is forbidden: {Path(directory) / name}")
        for name in sorted(files):
            relative = (relative_dir / name).as_posix()
            if not _source_path_allowed(relative):
                continue
            _relative_path(relative)
            original = Path(directory) / name
            if original.is_symlink() or not stat.S_ISREG(original.stat().st_mode):
                raise BundleError(f"source symlink/nonregular file is forbidden: {original}")
            copied = destination / relative
            copied.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(original, copied)
            copied.chmod(0o755 if original.stat().st_mode & 0o111 else 0o644)


def _project(root: Path) -> tuple[dict, dict]:
    try:
        data = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))
        project = data["project"]
        name = _text(project["name"], "project.name")
        version = _text(project["version"], "project.version")
        requires_python = _text(project["requires-python"], "project.requires-python")
        if name != "bike-sim" or not _PIN.fullmatch(f"{name}=={version}"):
            raise BundleError("pyproject.toml must contain a static bike-sim name/version")
        runtime = _strings(project.get("dependencies", []), "project.dependencies")
        build = _strings(data.get("build-system", {}).get("requires", []), "build-system.requires")
        native = _strings(data.get("dependency-groups", {}).get("native", []), "dependency-groups.native")
        return {"name": name, "version": version, "requires_python": requires_python}, {"runtime": runtime, "build": build + native}
    except (OSError, UnicodeError, tomllib.TOMLDecodeError, KeyError, TypeError) as exc:
        raise BundleError(f"invalid project/pyproject.toml: {exc}") from exc


def _python_specifiers(requirement: str) -> set[tuple[str, int, int]]:
    """Support minor-version bounds without guessing a target patch version.

    General PEP 440 resolution belongs to installation. The current source floor
    and bounded >=3.13,<3.15 declarations are decidable for every patch release of
    the selected interpreter; patch-specific/other forms fail explicitly.
    """
    bounds = set()
    for clause in _text(requirement, "requires-python").split(","):
        match = re.fullmatch(r"\s*(>=|<)\s*([0-9]+)\.([0-9]+)(?:\.0)?\s*", clause)
        if match is None:
            raise BundleError("unsupported requires-python specifier: use >=major.minor and/or <major.minor bounds; "
                              "patch-specific/general specifiers require installer target resolution")
        bounds.add((match[1], int(match[2]), int(match[3])))
    return bounds


def _python_bounds(requirement: str, target: dict) -> None:
    version = tuple(map(int, target["python"].split(".")))
    for operator, major, minor in _python_specifiers(requirement):
        bound = (major, minor)
        if (operator == ">=" and version < bound) or (operator == "<" and version >= bound):
            raise BundleError(f"requires-python {requirement!r} excludes target CPython {target['python']}")


def _source_entry_points(root: Path) -> dict[str, dict[str, str]]:
    data = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))["project"]
    declared = data.get("entry-points", {})
    if not isinstance(declared, dict) or {"console_scripts", "gui_scripts"} & set(declared):
        raise BundleError("project.entry-points must use project.scripts/gui-scripts for reserved groups")
    groups = dict(declared)
    for key, group in (("scripts", "console_scripts"), ("gui-scripts", "gui_scripts")):
        if key in data:
            groups[group] = data[key]
    result = {}
    for group, entries in groups.items():
        _text(group, "entry point group")
        if not isinstance(entries, dict):
            raise BundleError(f"entry point group {group} must be a table")
        if entries:
            result[group] = {_text(name, "entry point name"): _text(value, "entry point value")
                             for name, value in entries.items()}
    return result


def _wheel_entry_points(archive: ZipFile, members: dict, metadata_root: str) -> dict[str, dict[str, str]]:
    name = metadata_root + "entry_points.txt"
    if name not in members:
        return {}
    parser = ConfigParser(interpolation=None, delimiters=("=",), strict=True, empty_lines_in_values=False)
    parser.optionxform = str
    try:
        parser.read_string(archive.read(name).decode("utf-8"))
        if parser.defaults():
            raise BundleError("project wheel entry points cannot use inherited defaults")
        return {section: dict(parser.items(section, raw=True)) for section in parser.sections()
                if parser.items(section, raw=True)}
    except (UnicodeError, ConfigError) as exc:
        raise BundleError(f"invalid project wheel entry_points.txt: {exc}") from exc


def _normal_name(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def _members(archive: ZipFile) -> dict[str, object]:
    members = {}
    identities = set()
    for info in archive.infolist():
        name = _relative_path(info.filename[:-1] if info.is_dir() else info.filename)
        identity = unicodedata.normalize("NFC", name).casefold()
        if identity in identities:
            raise BundleError(f"duplicate/colliding wheel path: {name}")
        identities.add(identity)
        mode = info.external_attr >> 16
        if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
            raise BundleError(f"symlink/nonregular wheel member: {name}")
        if not info.is_dir():
            members[name] = info
    return members


def _platform_tag_matches(tag: str, target: dict) -> bool:
    if tag == "any":
        return True
    if target["system"] == "linux":
        return bool(re.fullmatch(r"(?:linux|manylinux(?:1|2010|2014|_\d+_\d+))_x86_64", tag))
    return bool(re.fullmatch(r"macosx_\d+_\d+_(?:" + target["machine"] + r"|universal2)", tag))


def _wheel_tags_match(python_tag: str, abi_tag: str, platform_tag: str, target: dict) -> bool:
    # This is an exclusion check for known mismatches, not packaging.tags.sys_tags
    # and not a claim about the target's libc, macOS deployment floor or CPU ISA.
    version = tuple(map(int, target["python"].split(".")))
    cpython = "cp" + target["python"].replace(".", "")
    for interpreter in python_tag.split("."):
        generic_version = re.fullmatch(r"py(\d)(\d+)", interpreter)
        cpython_version = re.fullmatch(r"cp(\d)(\d+)", interpreter)
        for abi in abi_tag.split("."):
            pure = abi == "none" and (interpreter == "py3" or
                    (generic_version is not None and (3, 0) <= tuple(map(int, generic_version.groups())) <= version))
            exact = interpreter == cpython and abi in {cpython, "none", "abi3"}
            stable = (cpython_version is not None and abi == "abi3"
                      and (3, 2) <= tuple(map(int, cpython_version.groups())) <= version)
            if pure or exact or stable:
                if any(_platform_tag_matches(tag, target) for tag in platform_tag.split(".")):
                    return True
    return False


def _wheel(path: Path, target: dict) -> dict:
    if path.is_symlink() or not path.is_file():
        raise BundleError(f"wheel must be a regular file, not a symlink: {path}")
    _relative_path(path.name)
    parts = path.name[:-4].split("-") if path.name.endswith(".whl") else []
    if len(parts) not in (5, 6):
        raise BundleError(f"invalid wheel filename: {path.name}")
    name, version = parts[:2]
    python_tag, abi_tag, platform_tag = parts[-3:]
    if (not re.fullmatch(r"[A-Za-z0-9_]+", name)
            or not _PIN.fullmatch(f"{name}=={version}")
            or not all(re.fullmatch(r"[A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*", tag)
                       for tag in (python_tag, abi_tag, platform_tag))
            or (len(parts) == 6 and not re.fullmatch(r"[0-9][A-Za-z0-9_]*", parts[2]))):
        raise BundleError(f"invalid wheel filename: {path.name}")
    if not _wheel_tags_match(python_tag, abi_tag, platform_tag, target):
        raise BundleError(f"wheel filename tags do not match target: {path.name}")
    try:
        with ZipFile(path) as archive:
            members = _members(archive)
            metadata_names = [member for member in members
                              if re.fullmatch(r"[^/]+\.dist-info/METADATA", member)]
            if len(metadata_names) != 1:
                raise BundleError(f"wheel needs exactly one METADATA record: {path.name}")
            metadata = BytesParser().parsebytes(archive.read(metadata_names[0]))
            if len(metadata.get_all("Name", [])) != 1 or len(metadata.get_all("Version", [])) != 1:
                raise BundleError(f"wheel has missing/duplicate Name or Version: {path.name}")
            if _normal_name(metadata["Name"]) != _normal_name(name) or metadata["Version"] != version:
                raise BundleError(f"wheel filename and METADATA identity disagree: {path.name}")
            wheel_metadata_name = metadata_names[0].rsplit("/", 1)[0] + "/WHEEL"
            if wheel_metadata_name not in members:
                raise BundleError(f"wheel has no WHEEL tag metadata: {path.name}")
            wheel_metadata = BytesParser().parsebytes(archive.read(wheel_metadata_name))
            declared_tags = set()
            for tag in wheel_metadata.get_all("Tag", []):
                parts = tag.split("-")
                if len(parts) != 3:
                    raise BundleError(f"invalid WHEEL Tag: {path.name}")
                declared_tags.update(product(*(part.split(".") for part in parts)))
            filename_tags = set(product(python_tag.split("."), abi_tag.split("."), platform_tag.split(".")))
            if declared_tags != filename_tags:
                raise BundleError(f"wheel filename and WHEEL tags disagree: {path.name}")
    except (BadZipFile, OSError, RuntimeError) as exc:
        raise BundleError(f"cannot read wheel {path.name}: {exc}") from exc
    return {"path": f"wheelhouse/{path.name}", "distribution": _normal_name(name), "version": version,
            "python_tag": python_tag, "abi_tag": abi_tag, "platform_tag": platform_tag,
            "sha256": _digest(path)}


def _project_wheel(root: Path, wheels: list[dict], identity: dict) -> None:
    selected = [item for item in wheels if item["distribution"] == "bike-sim"]
    if len(selected) != 1:
        raise BundleError("wheelhouse must contain exactly one bike_sim project wheel")
    item = selected[0]
    if not PurePosixPath(item["path"]).name.startswith(f"bike_sim-{identity['version']}-"):
        raise BundleError("project wheel filename must use the canonical bike_sim prefix for the legacy installer")
    if item["version"] != identity["version"]:
        raise BundleError("project wheel version differs from authoritative source")
    if item["abi_tag"] != "none" or item["platform_tag"] != "any":
        raise BundleError("project wheel must be source-only; native artifacts must be rebuilt on target")
    package_root = root / "project" / "src" / "bike_sim"
    local = {path.relative_to(package_root.parent).as_posix(): path for path in _regular_files(package_root)}
    if "bike_sim/__init__.py" not in local:
        raise BundleError("authoritative source package has no bike_sim/__init__.py")
    with ZipFile(root / item["path"]) as archive:
        members = _members(archive)
        metadata_root = f"bike_sim-{identity['version']}.dist-info/"
        unexpected = {name for name in members if not name.startswith(("bike_sim/", metadata_root))}
        if unexpected:
            raise BundleError(f"project wheel payload is outside authoritative source/metadata: {sorted(unexpected)}")
        if any(_BINARY.search(name) for name in members):
            raise BundleError("project wheel contains a native/build artifact; source must remain authoritative")
        packaged = {name for name in members if name.startswith("bike_sim/")}
        if packaged != set(local) or any(archive.read(name) != local[name].read_bytes() for name in packaged):
            raise BundleError("project wheel content differs from authoritative source (code or package resources)")
        if _wheel_entry_points(archive, members, metadata_root) != _source_entry_points(root / "project"):
            raise BundleError("project wheel entry points differ from authoritative pyproject.toml")
        metadata = BytesParser().parsebytes(archive.read(metadata_root + "METADATA"))
        python_requirements = metadata.get_all("Requires-Python", [])
        if len(python_requirements) != 1:
            raise BundleError("project wheel must declare exactly one Requires-Python matching source")
        # Source and wheel must agree on the supported bound form, regardless of
        # whitespace, order or redundant .0 spelling. Target eligibility itself
        # is checked against the actual selected target in create/validate.
        if _python_specifiers(python_requirements[0]) != _python_specifiers(identity["requires_python"]):
            raise BundleError("project wheel Requires-Python differs from authoritative source")


def _requirements(wheels: list[dict]) -> bytes:
    return "".join(f'{item["distribution"]}=={item["version"]} --hash=sha256:{item["sha256"]}\n'
                   for item in sorted(wheels, key=lambda item: item["distribution"])).encode("utf-8")


def _inventory(root: Path) -> list[dict]:
    return [{"path": path.relative_to(root).as_posix(), "size": path.stat().st_size, "sha256": _digest(path)}
            for path in _regular_files(root) if path.relative_to(root).as_posix() != MANIFEST]


def _source_digest(files: list[dict]) -> str:
    payload = "".join(f'{item["path"][8:]}={item["sha256"]}\n'
                      for item in sorted(files, key=lambda item: item["path"]) if item["path"].startswith("project/"))
    return sha256(payload.encode("utf-8")).hexdigest()


def _preinstalled(values: object, profile: str, wheels: list[dict]) -> list[str]:
    values = _strings(values, "requirements.preinstalled")
    if profile == "full" and values:
        raise BundleError("full bundles cannot declare inherited Python packages")
    if profile == "slim" and not values:
        raise BundleError("slim bundles must explicitly declare pinned preinstalled requirements")
    names = set()
    local_names = {item["distribution"] for item in wheels}
    for requirement in values:
        match = _PIN.fullmatch(requirement)
        if match is None:
            raise BundleError("preinstalled requirements must be explicit name==version pins")
        name = _normal_name(match[1])
        if name in names or name in local_names:
            raise BundleError(f"duplicate or both local/preinstalled requirement: {name}")
        names.add(name)
    return values


def validate_bundle(root: Path) -> dict:
    """Validate an extracted bundle, reading bytes only; return its manifest.

    The exact inventory is checked before installation. Environments or reports
    added inside the directory afterwards require an untouched bundle copy for
    this preflight. Qualification is never inferred from this function's return.
    """
    root = Path(root)
    actual_files = _regular_files(root)
    manifest = _keys(_read_manifest(root / MANIFEST), {
        "schema_version", "format", "profile", "target", "producer", "project",
        "requirements", "wheels", "files", "qualification",
    }, "manifest")
    if type(manifest["schema_version"]) is not int or manifest["schema_version"] != 1 or manifest["format"] != FORMAT:
        raise BundleError("unsupported bundle format/schema version")
    if manifest["profile"] not in ("full", "slim"):
        raise BundleError("profile must be full or slim")
    target = _target(manifest["target"])
    producer = _keys(manifest["producer"], {"system", "machine", "python"}, "producer")
    for name, value in producer.items():
        if not re.fullmatch(r"[A-Za-z0-9_.+-]+", _text(value, f"producer.{name}")):
            raise BundleError(f"producer.{name} must be a portable identity, not a filesystem path")
    project = _keys(manifest["project"], {"path", "name", "version", "requires_python", "source_sha256", "authority", "native_artifacts"}, "project")
    if project["path"] != "project" or project["authority"] != "source" or project["native_artifacts"] != "rebuild_on_target":
        raise BundleError("project must declare source authority and rebuilding native artifacts on target")
    requirements = _keys(manifest["requirements"], {"runtime", "build", "preinstalled", "closure", "wheel_dependency_metadata"}, "requirements")
    if requirements["closure"] != "not_verified" or requirements["wheel_dependency_metadata"] != "not_verified":
        raise BundleError("dependency closure qualification is outside the bundle manifest contract")
    _keys(manifest["qualification"], set(QUALIFICATION), "qualification")
    if manifest["qualification"] != QUALIFICATION:
        raise BundleError("bundle generation must not claim execution qualification")
    if not isinstance(manifest["files"], list) or not manifest["files"]:
        raise BundleError("file inventory must be a nonempty list")
    names = set()
    identities = set()
    for entry in manifest["files"]:
        _keys(entry, {"path", "size", "sha256"}, "file entry")
        name = _relative_path(entry["path"])
        canonical = unicodedata.normalize("NFC", name).casefold()
        if canonical in identities:
            raise BundleError(f"duplicate/colliding file path: {name}")
        identities.add(canonical)
        names.add(name)
        if (name != "requirements.txt" and not name.startswith(("project/", "wheelhouse/"))) or name == MANIFEST:
            raise BundleError(f"unsupported bundle file path: {name}")
        if name.startswith("project/") and not _source_path_allowed(name[8:]):
            raise BundleError(f"machine-local/generated project path is forbidden: {name}")
        if name.startswith("wheelhouse/") and (len(PurePosixPath(name).parts) != 2 or not name.endswith(".whl")):
            raise BundleError(f"wheelhouse contains a non-wheel path: {name}")
        if type(entry["size"]) is not int or entry["size"] < 0:
            raise BundleError(f"invalid byte size for {name}")
        if not isinstance(entry["sha256"], str) or not _DIGEST.fullmatch(entry["sha256"]):
            raise BundleError(f"invalid SHA-256 digest for {name}")
    actual_names = {path.relative_to(root).as_posix() for path in actual_files}
    if not {"project/pyproject.toml", "requirements.txt"} <= names:
        raise BundleError("file inventory must include project/pyproject.toml and requirements.txt")
    if actual_names != names | {MANIFEST}:
        raise BundleError(f"file inventory differs: missing={sorted(names - actual_names)}, "
                          f"undeclared={sorted(actual_names - names - {MANIFEST})}")
    for entry in manifest["files"]:
        path = root / entry["path"]
        if path.stat().st_size != entry["size"] or _digest(path) != entry["sha256"]:
            raise BundleError(f"file size/digest mismatch: {entry['path']}")
    identity, declarations = _project(root / "project")
    if any(project[key] != identity[key] for key in ("name", "version", "requires_python")):
        raise BundleError("manifest project identity differs from pyproject.toml")
    _python_bounds(identity["requires_python"], target)
    if project["source_sha256"] != _source_digest(manifest["files"]):
        raise BundleError("project source digest differs from manifest inventory")
    for key in ("runtime", "build"):
        if _strings(requirements[key], f"requirements.{key}") != declarations[key]:
            raise BundleError(f"requirements.{key} differs from authoritative pyproject.toml")
    if not isinstance(manifest["wheels"], list):
        raise BundleError("wheels must be a list")
    wheels = []
    for entry in manifest["wheels"]:
        _keys(entry, {"path", "distribution", "version", "python_tag", "abi_tag", "platform_tag", "sha256"}, "wheel entry")
        if _relative_path(entry["path"]) not in names or not entry["path"].startswith("wheelhouse/"):
            raise BundleError("wheel path is missing from the file inventory")
        actual = _wheel(root / entry["path"], target)
        if actual != entry:
            raise BundleError(f"wheel metadata/digest mismatch: {entry['path']}")
        wheels.append(actual)
    if {entry["path"] for entry in wheels} != {name for name in names if name.startswith("wheelhouse/")}:
        raise BundleError("wheel list differs from the wheelhouse inventory")
    if len({entry["distribution"] for entry in wheels}) != len(wheels):
        raise BundleError("wheelhouse must contain only one wheel per distribution")
    _preinstalled(requirements["preinstalled"], manifest["profile"], wheels)
    _project_wheel(root, wheels, identity)
    if (root / "requirements.txt").read_bytes() != _requirements(wheels):
        raise BundleError("requirements.txt must contain exactly the local wheel pins and hashes")
    return manifest


def create_bundle(source: Path, wheelhouse: Path, output: Path, *, target: dict,
                  profile: str, preinstalled: list[str] | tuple[str, ...] = ()) -> dict:
    """Create a deterministic ZIP from explicit local inputs, never overwriting.

    A fresh sibling staging directory is removed on failure. The output must be
    outside the source tree; this prevents self-inclusion on repeated packaging.
    """
    source, wheelhouse, output = Path(source), Path(wheelhouse), Path(output)
    _target(target)
    if profile not in ("full", "slim"):
        raise BundleError("profile must be full or slim")
    if output.exists() or output.is_symlink():
        raise BundleError("output already exists; choose a new .zip path")
    if output.suffix != ".zip":
        raise BundleError("output must be a .zip archive")
    if output.resolve().is_relative_to(source.resolve()):
        raise BundleError("output must be outside the source tree")
    if wheelhouse.is_symlink() or not wheelhouse.is_dir():
        raise BundleError("wheelhouse must be an explicit local directory, not a symlink")
    input_wheels = sorted(wheelhouse.iterdir())
    if any(path.is_symlink() or not path.is_file() or path.suffix != ".whl" for path in input_wheels):
        raise BundleError("wheelhouse must contain only regular local .whl files")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="bike-sim-offline-", dir=output.parent) as temporary:
        stage = Path(temporary) / "payload"
        (stage / "project").mkdir(parents=True)
        (stage / "wheelhouse").mkdir()
        _copy_sources(source, stage / "project")
        identity, declarations = _project(stage / "project")
        _python_bounds(identity["requires_python"], target)
        wheels = []
        for original in input_wheels:
            copied = stage / "wheelhouse" / original.name
            shutil.copyfile(original, copied)
            wheels.append(_wheel(copied, target))
        _project_wheel(stage, wheels, identity)
        external = _preinstalled(list(preinstalled), profile, wheels)
        (stage / "requirements.txt").write_bytes(_requirements(wheels))
        files = _inventory(stage)
        manifest = {
            "schema_version": 1, "format": FORMAT, "profile": profile,
            "target": dict(target),
            "producer": {"system": sys.platform, "machine": platform.machine(), "python": platform.python_version()},
            "project": {"path": "project", **identity, "authority": "source",
                        "native_artifacts": "rebuild_on_target", "source_sha256": _source_digest(files)},
            "requirements": {**declarations, "preinstalled": external, "closure": "not_verified",
                             "wheel_dependency_metadata": "not_verified"},
            "wheels": wheels, "files": files, "qualification": dict(QUALIFICATION),
        }
        (stage / MANIFEST).write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        validate_bundle(stage)
        archive_path = Path(temporary) / "archive.zip"
        with ZipFile(archive_path, "w", compression=ZIP_DEFLATED, compresslevel=6) as archive:
            for path in _regular_files(stage):
                info = ZipInfo(path.relative_to(stage).as_posix(), date_time=(1980, 1, 1, 0, 0, 0))
                info.create_system = 3
                info.compress_type = ZIP_DEFLATED
                info.external_attr = (stat.S_IFREG | (0o755 if path.stat().st_mode & 0o111 else 0o644)) << 16
                with path.open("rb") as source_stream, archive.open(info, "w") as destination:
                    shutil.copyfileobj(source_stream, destination)
        # Exclusive creation preserves an existing file even if another producer
        # published it after the initial path check. Do not rename over it.
        try:
            os.link(archive_path, output)
        except FileExistsError as exc:
            raise BundleError("output was created by another producer; it was preserved") from exc
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    create = subparsers.add_parser("create", help="package supplied files; no build, resolver or network")
    create.add_argument("--source", required=True, type=Path)
    create.add_argument("--wheelhouse", required=True, type=Path)
    create.add_argument("--output", required=True, type=Path)
    create.add_argument("--target", choices=tuple(TARGETS), required=True)
    create.add_argument("--profile", choices=("full", "slim"), required=True)
    create.add_argument("--preinstalled", action="append", default=[], metavar="NAME==VERSION")
    check = subparsers.add_parser("check", help="validate an extracted bundle without executing it")
    check.add_argument("bundle", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "create":
            manifest = create_bundle(args.source, args.wheelhouse, args.output, target=TARGETS[args.target],
                                     profile=args.profile, preinstalled=args.preinstalled)
        else:
            manifest = validate_bundle(args.bundle)
        print(json.dumps({"content_valid": True, "profile": manifest["profile"], "target": manifest["target"],
                          "file_count": len(manifest["files"]), "qualification": manifest["qualification"]}, sort_keys=True))
        return 0
    except (BundleError, OSError, BadZipFile) as exc:
        print(json.dumps({"content_valid": False, "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
