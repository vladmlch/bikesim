"""Shared, non-building schema and payload rules for offline source bundles.

Schema 1 describes assembly capability, never installation qualification.  The
assembler and T25 installer must both validate the actual payload as well as
this document; a self-consistent JSON document is not proof of its contents.
"""
from __future__ import annotations

import hashlib
import re
from pathlib import Path, PurePosixPath
from typing import Any

from packaging.tags import parse_tag
from packaging.utils import canonicalize_name, parse_wheel_filename
from packaging.version import InvalidVersion, Version


SCHEMA_VERSION = 1
TOOL_NAMES = frozenset({"uv", "pip", "setuptools", "wheel"})
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_PRIVATE_DIRS = frozenset({
    ".git", ".aws", ".ssh", ".gnupg", ".codex", ".claude", ".agents",
    ".devin", ".work", ".scratch", ".tox", ".nox", ".venv", "venv",
    "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache",
    "node_modules", ".cache",
})
_GENERATED_SUFFIXES = frozenset({
    ".so", ".dylib", ".dll", ".a", ".o", ".obj", ".lib", ".exe",
    ".pyc", ".pyo", ".whl", ".egg", ".duckdb", ".wal", ".pyd",
})
BUILD_MARKERS = ("CMakeCache.txt", "CMakeFiles", "pyvenv.cfg", "meson-private", "native_check_context.json")
_BASIC_SOURCES = frozenset({"pyproject.toml", "uv.lock", "src/bike_sim/__init__.py"})
_NATIVE_SOURCES = frozenset({
    "native/CMakeLists.txt", "tools/dev.py", "tools/run_tests.sh",
    "tools/proto_native_bench/bench.cpp", "tools/proto_native_bench/state_format.hpp",
})


def safe_relative_path(value: object, field: str = "path") -> PurePosixPath:
    """Accept one portable, already-normalized relative archive member."""
    if not isinstance(value, str) or not value or any(c in value for c in "\\\x00:\r\n"):
        raise ValueError(f"{field}: unsafe archive path {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in value.split("/")):
        raise ValueError(f"{field}: unsafe archive path {value!r}")
    if str(path) != value:
        raise ValueError(f"{field}: path must be normalized")
    return path


def include_source_path(path: PurePosixPath) -> bool:
    """Lexical exclusions; filesystem build-root/symlink checks are additional.

    Deliberately retain tests, fixtures, and legitimate ``artifacts`` directories:
    the native benchmark's model.mjb/state.bin/state.npz are required inputs.
    """
    try:
        path = safe_relative_path(str(path), "source path")
    except ValueError:
        return False
    if any(part in _PRIVATE_DIRS or part.startswith((".venv-", ".venv_")) for part in path.parts):
        return False
    if path.parts[0] in {"output", "dist", "build", "recordings"}:
        return False
    if path.parts[:2] == ("native", "build"):
        return False
    if any(part.endswith((".egg-info", ".dist-info")) for part in path.parts):
        return False
    if path.name == ".env" or path.name.startswith(".env."):
        return False
    if path.name in {".DS_Store", "CMakeCache.txt", "compile_commands.json", "build.ninja", "pyvenv.cfg", "native_check_context.json"}:
        return False
    return path.suffix.lower() not in _GENERATED_SUFFIXES


def confined_file(root: Path, relative: str, *, source: bool = False) -> Path:
    path = safe_relative_path(relative)
    root = root.resolve(strict=True)
    candidate = root.joinpath(*path.parts)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        raise ValueError(f"{relative}: missing file or escaping symlink") from error
    if not resolved.is_file():
        raise ValueError(f"{relative}: expected a regular file")
    if source:
        if not include_source_path(path):
            raise ValueError(f"{relative}: excluded source path")
        # Check both names: an innocuous tracked symlink must not expose a venv,
        # an ignored build product, or credentials through its resolved target.
        resolved_relative = resolved.relative_to(root)
        if not include_source_path(PurePosixPath(resolved_relative.as_posix())):
            raise ValueError(f"{relative}: symlink resolves to excluded source")
        for selected in (candidate, resolved):
            parent = selected.parent
            while parent != root:
                if any((parent / marker).exists() for marker in BUILD_MARKERS):
                    raise ValueError(f"{relative}: generated build tree or virtualenv at {parent.relative_to(root)}")
                parent = parent.parent
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_digest(rows: list[dict[str, str]], prefix: str) -> str:
    if prefix == "native/":
        # Match native_source_fingerprint/CMake's existing artifact identity;
        # tests, benchmark fixtures and tools are still individually hashed.
        selected = []
        for row in rows:
            path = PurePosixPath(row["path"])
            if len(path.parts) < 2 or path.parts[:1] != ("native",):
                continue
            relative = path.relative_to("native")
            if (str(relative) == "CMakeLists.txt"
                    or (relative.parts[0] == "src" and relative.suffix in {".cpp", ".hpp", ".h", ".in"})
                    or (relative.parts[0] == "cmake" and relative.suffix in {".cmake", ".in"})):
                selected.append((str(relative), row["sha256"]))
        return hashlib.sha256("".join(f"{path}={sha}\n" for path, sha in sorted(selected)).encode()).hexdigest()
    selected = sorted((row["path"], row["sha256"]) for row in rows if row["path"].startswith(prefix))
    return hashlib.sha256("".join(f"{path}\0{sha}\n" for path, sha in selected).encode()).hexdigest()


def _mapping(value: object, field: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != keys:
        actual = set(value) if isinstance(value, dict) else set()
        raise ValueError(f"{field}: expected fields {sorted(keys)}; missing={sorted(keys - actual)}, extra={sorted(actual - keys, key=str)}")
    return value


def _text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value or any(char in value for char in "\x00\r\n"):
        raise ValueError(f"{field}: expected nonempty text")
    return value


def _hash(value: object, field: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{field}: expected lowercase SHA256")
    return value


def _strings(value: object, field: str, *, nonempty: bool = False) -> list[str]:
    if not isinstance(value, list) or (nonempty and not value):
        raise ValueError(f"{field}: expected {'nonempty ' if nonempty else ''}list")
    for entry in value:
        _text(entry, field)
    if len(set(value)) != len(value):
        raise ValueError(f"{field}: duplicate values")
    return value


def _version(value: object, field: str) -> str:
    value = _text(value, field)
    try:
        Version(value)
    except InvalidVersion as error:
        raise ValueError(f"{field}: invalid exact version") from error
    if any(char in value for char in "<>=!~*, "):
        raise ValueError(f"{field}: expected exact version")
    return value


def python_minor(value: object) -> tuple[int, int]:
    if not isinstance(value, str) or re.fullmatch(r"3\.[0-9]+", value) is None:
        raise ValueError("target.python_minor: expected MAJOR.MINOR")
    minor = int(value.split(".")[1])
    if value != f"3.{minor}" or minor < 13:
        raise ValueError("target.python_minor: Python >=3.13 is required")
    return (3, minor)


def validate_target(value: object) -> dict[str, Any]:
    target = _mapping(value, "target", {"system", "machine", "python_minor", "implementation", "accepted_wheel_tags", "libc_floor"})
    machines = {"Linux": {"x86_64", "aarch64"}, "Darwin": {"arm64", "x86_64"}}
    if (not isinstance(target["system"], str) or not isinstance(target["machine"], str)
            or target["system"] not in machines or target["machine"] not in machines[target["system"]]):
        raise ValueError("target: unsupported OS/architecture")
    major, minor = python_minor(target["python_minor"])
    if target["implementation"] != "cpython":
        raise ValueError("target.implementation: only CPython is supported")
    if target["system"] == "Linux":
        floor = target["libc_floor"]
        if not isinstance(floor, str) or re.fullmatch(r"glibc>=2\.[0-9]+", floor) is None:
            raise ValueError("target.libc_floor: Linux requires an explicit glibc floor")
        if Version(floor.removeprefix("glibc>=")) < Version("2.28"):
            raise ValueError("target.libc_floor: qualified scientific wheels require glibc>=2.28")
    elif target["libc_floor"] is not None:
        raise ValueError("target.libc_floor: macOS has no glibc prerequisite")
    for value in _strings(target["accepted_wheel_tags"], "target.accepted_wheel_tags", nonempty=True):
        parsed = parse_tag(value)
        if len(parsed) != 1 or str(next(iter(parsed))) != value:
            raise ValueError(f"target.accepted_wheel_tags: expected expanded canonical tag {value!r}")
        tag = next(iter(parsed))
        cp = re.fullmatch(r"cp([0-9])([0-9]+)", tag.interpreter)
        py = re.fullmatch(r"py([0-9])([0-9]*)", tag.interpreter)
        if cp:
            version = (int(cp[1]), int(cp[2]))
            if version[0] != major or version > (major, minor) or (version != (major, minor) and tag.abi != "abi3"):
                raise ValueError(f"target.accepted_wheel_tags: incompatible Python tag {value}")
            if tag.abi not in {"abi3", "none", f"cp{major}{minor}"}:
                raise ValueError(f"target.accepted_wheel_tags: unsupported Python ABI {value}")
        elif not py or int(py[1]) != major or (py[2] and int(py[2]) > minor) or tag.abi != "none":
            raise ValueError(f"target.accepted_wheel_tags: incompatible interpreter {value}")
        if tag.platform == "any":
            if tag.abi != "none":
                raise ValueError(f"target.accepted_wheel_tags: platform any with binary ABI {value}")
            continue
        if target["system"] == "Linux":
            if re.fullmatch(r"(?:linux|manylinux_[0-9]+_[0-9]+|manylinux1|manylinux2010|manylinux2014)_" + target["machine"], tag.platform) is None:
                raise ValueError(f"target.accepted_wheel_tags: wrong OS/architecture {value}")
            match = re.match(r"manylinux_([0-9]+)_([0-9]+)_", tag.platform)
            if match and Version(f"{match[1]}.{match[2]}") > Version(target["libc_floor"].removeprefix("glibc>=")):
                raise ValueError(f"target.accepted_wheel_tags: tag exceeds declared glibc floor {value}")
        else:
            arches = "(?:arm64|universal2)" if target["machine"] == "arm64" else "(?:x86_64|intel|fat32|fat64|universal2|universal)"
            if re.fullmatch(r"macosx_[0-9]+_[0-9]+_" + arches, tag.platform) is None:
                raise ValueError(f"target.accepted_wheel_tags: wrong OS/architecture {value}")
    return target


def validate_wheel_row(value: object, target: dict[str, Any]) -> dict[str, Any]:
    row = _mapping(value, "wheel", {"filename", "distribution", "version", "sha256", "tags"})
    path = safe_relative_path(row["filename"], "wheel.filename")
    if len(path.parts) != 1:
        raise ValueError("wheel.filename: expected a basename")
    try:
        name, version, _build, tags = parse_wheel_filename(row["filename"])
    except ValueError as error:
        raise ValueError(f"wheel.filename: invalid wheel {row['filename']!r}") from error
    if row["distribution"] != str(name) or row["version"] != str(version):
        raise ValueError(f"wheel {row['filename']}: distribution/version disagrees with filename")
    _hash(row["sha256"], "wheel.sha256")
    actual_tags = {str(tag) for tag in tags}
    if set(_strings(row["tags"], "wheel.tags", nonempty=True)) != actual_tags:
        raise ValueError(f"wheel {row['filename']}: filename/tags mismatch")
    if not actual_tags.intersection(target["accepted_wheel_tags"]):
        raise ValueError(f"wheel {row['filename']}: incompatible target tags")
    return row


def validate_bundle_manifest(value: object) -> dict[str, Any]:
    """Validate schema 1 without importing the application or touching a host."""
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != SCHEMA_VERSION:
        raise ValueError("schema_version: unsupported offline manifest schema")
    manifest = _mapping(value, "manifest", {"schema_version", "project", "target", "mode", "capabilities", "requirements", "wheels", "source_files", "prerequisites", "slim_external"})
    project = _mapping(manifest["project"], "project", {"name", "version", "git_commit", "dirty", "python_source_sha256", "native_source_sha256"})
    if project["name"] != "bike-sim":
        raise ValueError("project.name: expected bike-sim")
    _version(project["version"], "project.version")
    if not isinstance(project["git_commit"], str) or re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", project["git_commit"]) is None:
        raise ValueError("project.git_commit: expected actual Git object ID")
    if type(project["dirty"]) is not bool:
        raise ValueError("project.dirty: expected boolean")
    target = validate_target(manifest["target"])
    if manifest["mode"] not in ("full", "slim"):
        raise ValueError("mode: expected full or slim")
    capabilities = _mapping(manifest["capabilities"], "capabilities", {"python", "native_source_build", "development", "gui"})
    if any(type(item) is not bool for item in capabilities.values()) or capabilities["python"] is not True:
        raise ValueError("capabilities: expected boolean flags and python=true")
    native = capabilities["native_source_build"]
    if native != capabilities["development"]:
        raise ValueError("capabilities: development must match the complete native-source profile")
    requirements = _mapping(manifest["requirements"], "requirements", {"lock_sha256", "requirements_sha256", "groups"})
    _hash(requirements["lock_sha256"], "requirements.lock_sha256")
    _hash(requirements["requirements_sha256"], "requirements.requirements_sha256")
    groups = set(_strings(requirements["groups"], "requirements.groups"))
    if groups != ({"native", "dev"} if native else set()):
        raise ValueError("requirements.groups: native requires frozen native+dev groups; Python-only declares neither")
    if not isinstance(manifest["source_files"], list) or not manifest["source_files"]:
        raise ValueError("source_files: missing source inventory")
    sources: dict[str, str] = {}
    folded: set[str] = set()
    for value in manifest["source_files"]:
        row = _mapping(value, "source_file", {"path", "sha256"})
        path = safe_relative_path(row["path"], "source_file.path")
        if not include_source_path(path):
            raise ValueError(f"source_file.path: generated/private payload {path}")
        if str(path).casefold() in folded:
            raise ValueError(f"source_files: duplicate/case-colliding path {path}")
        folded.add(str(path).casefold())
        sources[str(path)] = _hash(row["sha256"], "source_file.sha256")
    missing = (_BASIC_SOURCES | (_NATIVE_SOURCES if native else frozenset())) - sources.keys()
    if missing:
        raise ValueError(f"source_files: missing required source {sorted(missing)}")
    if native:
        for prefix in ("native/src/", "native/cmake/", "native/tests/"):
            if not any(name.startswith(prefix) for name in sources):
                raise ValueError(f"source_files: missing native source directory {prefix}")
    if sources["uv.lock"] != requirements["lock_sha256"]:
        raise ValueError("source_files: uv.lock hash disagrees with requirements")
    for key, prefix in (("python_source_sha256", "src/"), ("native_source_sha256", "native/")):
        _hash(project[key], f"project.{key}")
        if project[key] != source_digest(manifest["source_files"], prefix):
            raise ValueError(f"project.{key}: source inventory hash mismatch")
    if not isinstance(manifest["wheels"], list) or not manifest["wheels"]:
        raise ValueError("wheels: missing wheel inventory")
    wheels: dict[str, dict[str, Any]] = {}
    filenames: set[str] = set()
    for value in manifest["wheels"]:
        row = validate_wheel_row(value, target)
        if row["filename"].casefold() in filenames or row["distribution"] in wheels:
            raise ValueError(f"wheels: duplicate filename or distribution {row['filename']}")
        filenames.add(row["filename"].casefold())
        wheels[row["distribution"]] = row
    if not isinstance(manifest["slim_external"], list):
        raise ValueError("slim_external: expected list")
    external: dict[str, str] = {}
    for value in manifest["slim_external"]:
        row = _mapping(value, "slim_external entry", {"distribution", "version", "qualification_profile"})
        name = _text(row["distribution"], "slim_external.distribution")
        if name != canonicalize_name(name, validate=True) or name in wheels or name in external or name in TOOL_NAMES:
            raise ValueError(f"slim_external: invalid, duplicate, tool or bundled distribution {name}")
        external[name] = _version(row["version"], "slim_external.version")
        required_profile = "native-exact-provider-pending-T25" if native else "python-exact-import-pending-T25"
        if row["qualification_profile"] != required_profile:
            raise ValueError("slim_external.qualification_profile: assembly cannot claim installation qualification")
    if manifest["mode"] == "full" and external:
        raise ValueError("slim_external: full bundle cannot omit dependencies")
    if manifest["mode"] == "slim" and not external:
        raise ValueError("slim_external: slim mode must enumerate its omissions")
    available = {name: row["version"] for name, row in wheels.items()} | external
    if not TOOL_NAMES <= wheels.keys():
        raise ValueError(f"wheels: missing frozen build/tool closure {sorted(TOOL_NAMES - wheels.keys())}")
    if native:
        expected = {"mujoco": "3.12.0", "numpy": "2.5.2", "scipy": "1.17.0"}
        for name, version in expected.items():
            if available.get(name) != version:
                raise ValueError(f"wheels: native requires {name}=={version}")
        for name in ("nanobind", "hypothesis", "threadpoolctl"):
            if name not in wheels:
                raise ValueError(f"wheels: missing native/development dependency {name}")
        if "bike-sim" in wheels:
            raise ValueError("capabilities: native source bundle must not replace editable source with a Python wheel")
    elif "bike-sim" not in wheels or set(wheels["bike-sim"]["tags"]) != {"py3-none-any"}:
        raise ValueError("capabilities: Python-only compatibility path requires its pure Python project wheel")
    elif wheels["bike-sim"]["version"] != str(Version(project["version"])):
        raise ValueError("project.version: Python wheel and source versions differ")
    if capabilities["gui"] and not {"glfw", "pyopengl", "matplotlib"} <= available.keys():
        raise ValueError("capabilities.gui: missing rendering dependencies")
    prerequisites = _mapping(manifest["prerequisites"], "prerequisites", {"python", "compiler_feature_profile", "cmake_minimum", "uv_version", "system_libraries"})
    if prerequisites["python"] != f"CPython {target['python_minor']}.*":
        raise ValueError("prerequisites.python: must match target Python minor")
    if prerequisites["uv_version"] != wheels["uv"]["version"]:
        raise ValueError("prerequisites.uv_version: bundled uv version mismatch")
    if native:
        if prerequisites["cmake_minimum"] != "3.27" or prerequisites["compiler_feature_profile"] != "bike-cxx23-strict-local-probes-required":
            raise ValueError("prerequisites: native requires CMake>=3.27 and local strict C++23 qualification")
    elif prerequisites["cmake_minimum"] is not None or prerequisites["compiler_feature_profile"] is not None:
        raise ValueError("prerequisites: Python-only mode cannot advertise native compiler capability")
    _strings(prerequisites["system_libraries"], "prerequisites.system_libraries", nonempty=True)
    return manifest
