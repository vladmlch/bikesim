#!/usr/bin/env python3
"""Validate CMake's native TU manifest and run fail-closed analysis sweeps."""

from __future__ import annotations

import argparse
import json
import os
import re
import resource
import shlex
import signal
import shutil
import statistics
import subprocess
import sys
import tempfile
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


class CheckError(ValueError):
    """A malformed build context or incomplete translation-unit selection."""


def _disable_core_dumps() -> None:
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


_PATH_OPTIONS = {
    "-I",
    "-isystem",
    "-iquote",
    "-idirafter",
    "-isysroot",
    "--sysroot",
    "-L",
    "-F",
    "-iframework",
    "-include",
    "-imacros",
}
_DIAGNOSTIC = re.compile(r"\b(?:warning|error):|fatal error:", re.IGNORECASE)
_LOCATED_DIAGNOSTIC = re.compile(
    r"^(?P<origin>.*?):(?P<line>\d+):(?P<column>\d+):\s+"
    r"(?P<severity>warning|error):\s*(?P<message>.*?)(?:\s+\[(?P<checker>[^\]]+)\])?\s*$",
    re.IGNORECASE,
)
_TARGET_FROM_OUTPUT = re.compile(r"(?:^|[/\\])CMakeFiles[/\\]([^/\\]+)\.dir(?:[/\\]|$)")
_SUPPRESSION_FIELDS = {
    "checker",
    "origin",
    "message",
    "tool_version",
    "reason",
    "reproducer",
    "remove_when",
}


def _canonical(path: str | Path, base: Path | None = None) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute() and base is not None:
        candidate = base / candidate
    return candidate.resolve()


def _read_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as error:
        raise CheckError(f"missing {label}: {path}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise CheckError(f"malformed {label}: {path}: {error}") from error


def load_suppressions(path: Path) -> list[dict[str, str]]:
    manifest = _read_json(path, "analysis suppression manifest")
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise CheckError(f"unsupported analysis suppression manifest schema: {path}")
    entries = manifest.get("suppressions")
    if not isinstance(entries, list):
        raise CheckError(f"analysis suppression manifest has no suppression list: {path}")
    normalized: list[dict[str, str]] = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict) or not _SUPPRESSION_FIELDS <= entry.keys():
            raise CheckError(f"incomplete analysis suppression entry {index}: {path}")
        if any(not isinstance(entry[field], str) or not entry[field] for field in _SUPPRESSION_FIELDS):
            raise CheckError(f"analysis suppression entry {index} has empty or nonstring fields: {path}")
        try:
            re.compile(entry["origin"])
        except re.error as error:
            raise CheckError(f"invalid origin pattern in analysis suppression entry {index}: {error}") from error
        normalized.append({field: entry[field] for field in _SUPPRESSION_FIELDS})
    return normalized


def apply_suppressions(
    diagnostics: Sequence[str],
    tool_version: str,
    suppressions: Sequence[Mapping[str, str]],
) -> tuple[list[str], list[dict[str, str]]]:
    """Filter only diagnostics matching the declared checker, origin, message, and version."""
    remaining: list[str] = []
    matched: list[dict[str, str]] = []
    for diagnostic in diagnostics:
        parsed = _LOCATED_DIAGNOSTIC.match(diagnostic.strip())
        if parsed is None:
            remaining.append(diagnostic)
            continue
        checker = parsed.group("checker") or ""
        origin = parsed.group("origin")
        message = parsed.group("message")
        suppression = next(
            (
                entry
                for entry in suppressions
                if entry["checker"] == checker
                and entry["tool_version"] in tool_version
                and entry["message"] in message
                and re.search(entry["origin"], origin)
            ),
            None,
        )
        if suppression is None:
            remaining.append(diagnostic)
        else:
            matched.append(
                {
                    "diagnostic": diagnostic,
                    "checker": checker,
                    "origin": origin,
                    "reason": suppression["reason"],
                }
            )
    return remaining, matched


def _require_existing_path(
    value: Any, label: str, *, executable: bool = False, preserve_symlink: bool = False
) -> Path:
    if not isinstance(value, str) or not value:
        raise CheckError(f"{label} is missing or is not a path")
    expanded = Path(value).expanduser()
    # A C++ driver may be a symlink to the C driver binary. Its invocation
    # basename controls C++ runtime linking, so keep that spelling when used.
    path = expanded.absolute() if preserve_symlink else expanded.resolve()
    if not path.exists():
        raise CheckError(f"missing {label}: {path}")
    if executable and (not path.is_file() or not os.access(path, os.X_OK)):
        raise CheckError(f"{label} is not executable: {path}")
    return path


def load_context(build: Path) -> dict[str, Any]:
    """Load CMake's toolchain/dependency context and fail on stale paths."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        raise CheckError(f"build directory does not exist: {build_dir}")
    context = _read_json(build_dir / "native_check_context.json", "native check context")
    if not isinstance(context, dict) or context.get("schema_version") != 1:
        raise CheckError("native check context has an unsupported schema")

    compiler = context.get("compiler")
    if not isinstance(compiler, dict):
        raise CheckError("native check context has no compiler record")
    compiler_path = _require_existing_path(
        compiler.get("path"), "configured C++ compiler", executable=True,
        preserve_symlink=True,
    )
    compiler["path"] = str(compiler_path)

    python_executable = _require_existing_path(
        context.get("python_executable"), "configured Python executable", executable=True
    )
    context["python_executable"] = str(python_executable)
    sdk_path = _require_existing_path(context.get("sdk"), "configured SDK")
    if not sdk_path.is_dir():
        raise CheckError(f"configured SDK is not a directory: {sdk_path}")
    context["sdk"] = str(sdk_path)
    sdk_requested = context.get("sdk_requested")
    sdk_source = context.get("sdk_source")
    if not isinstance(sdk_requested, str) or not sdk_requested:
        raise CheckError("native check context has no requested SDK value")
    if sdk_source not in {"explicit", "default"}:
        raise CheckError("native check context has an invalid SDK source")
    requested_sdk_path = Path(sdk_requested).expanduser()
    if requested_sdk_path.exists() and requested_sdk_path.resolve() != sdk_path:
        raise CheckError(
            f"requested SDK and selected SDK differ: requested={requested_sdk_path.resolve()}, selected={sdk_path}"
        )

    hardening = context.get("libcpp_hardening")
    if not isinstance(hardening, dict):
        raise CheckError("native check context has no libc++ hardening record")
    if not isinstance(hardening.get("supported"), bool):
        raise CheckError("native check libc++ hardening support must be boolean")
    if hardening.get("mode") not in {"EXTENSIVE", "FAST", "NOT_APPLICABLE"}:
        raise CheckError("native check context has an invalid libc++ hardening mode")
    if hardening.get("requested_mode") not in {"EXTENSIVE", "FAST"}:
        raise CheckError("native check context has an invalid requested libc++ hardening mode")
    if hardening["supported"] != (hardening["mode"] != "NOT_APPLICABLE"):
        raise CheckError("native check libc++ hardening support and selected mode disagree")
    if hardening["supported"] and hardening["mode"] != hardening["requested_mode"]:
        raise CheckError("native check libc++ hardening mode differs from its configured request")
    suppressions_manifest = _require_existing_path(
        context.get("analysis_suppressions_manifest"),
        "analysis suppression manifest",
    )
    if not suppressions_manifest.is_file():
        raise CheckError(f"analysis suppression manifest is not a file: {suppressions_manifest}")
    context["analysis_suppressions_manifest"] = str(suppressions_manifest)

    for field in ("include_paths", "library_paths", "required_files", "source_roots"):
        values = context.get(field)
        if not isinstance(values, list) or any(not isinstance(value, str) for value in values):
            raise CheckError(f"native check context field {field!r} must be a path list")
        require_file = field == "required_files"
        for value in values:
            path = _require_existing_path(value, f"context {field} entry")
            if require_file and not path.is_file():
                raise CheckError(f"required dependency is not a file: {path}")
            if field in {"include_paths", "source_roots"} and not path.is_dir():
                raise CheckError(f"context {field} entry is not a directory: {path}")
            if field == "library_paths" and not path.is_file() and not (
                path.is_dir() and path.suffix == ".framework"
            ):
                raise CheckError(
                    f"context library_paths entry is neither a library file nor a framework directory: {path}"
                )
        context[field] = [str(Path(value).resolve()) for value in values]

    for field in ("tool_defaults", "tool_overrides"):
        values = context.get(field)
        if not isinstance(values, dict) or any(
            not isinstance(key, str) or not isinstance(value, str)
            for key, value in values.items()
        ):
            raise CheckError(f"native check context field {field!r} must be a string map")

    target_contexts = context.get("target_contexts", {})
    if not isinstance(target_contexts, dict):
        raise CheckError("native check context field 'target_contexts' must be an object")
    for target, target_context in target_contexts.items():
        if not isinstance(target, str) or not isinstance(target_context, dict):
            raise CheckError("native check target contexts must map target names to objects")
        for field in ("include_directories", "compile_definitions", "compile_options"):
            values = target_context.get(field, [])
            if not isinstance(values, list) or any(not isinstance(value, str) for value in values):
                raise CheckError(f"target context {target!r} field {field!r} must be a string list")
        if target_context.get("cxx_standard") != 23:
            raise CheckError(f"target context {target!r} must use C++23")
        if target_context.get("cxx_extensions") is not False:
            raise CheckError(f"target context {target!r} must disable C++ extensions")
    context["target_contexts"] = target_contexts
    return context


def _parse_command(entry: Mapping[str, Any], index: int) -> list[str]:
    arguments = entry.get("arguments")
    command = entry.get("command")
    parsed_command: list[str] | None = None
    if isinstance(command, str):
        try:
            parsed_command = shlex.split(command)
        except ValueError as error:
            raise CheckError(f"malformed command in compilation database entry {index}: {error}") from error
    if arguments is not None:
        if not isinstance(arguments, list) or not arguments or any(
            not isinstance(value, str) for value in arguments
        ):
            raise CheckError(f"malformed arguments in compilation database entry {index}")
        parsed_arguments = list(arguments)
        if parsed_command is not None and parsed_command != parsed_arguments:
            raise CheckError(f"conflicting command and arguments in compilation database entry {index}")
        argv = parsed_arguments
    elif parsed_command is not None:
        argv = parsed_command
    else:
        raise CheckError(f"compilation database entry {index} needs command or arguments")
    if not argv or not argv[0]:
        raise CheckError(f"compilation database entry {index} has no compiler")
    return argv


def _resolved_argument_path(value: str, directory: Path) -> Path:
    return _canonical(value, directory)


def _check_path_arguments(argv: Sequence[str], directory: Path, source: Path, index: int) -> None:
    position = 1
    while position < len(argv):
        argument = argv[position]
        value: str | None = None
        path_is_file = False
        if argument in _PATH_OPTIONS:
            if position + 1 >= len(argv):
                raise CheckError(f"missing path after {argument} in compilation database entry {index}")
            value = argv[position + 1]
            path_is_file = argument in {"-include", "-imacros"}
            position += 2
        else:
            for prefix in ("--sysroot=", "-isysroot", "-isystem", "-iquote", "-idirafter", "-iframework", "-I", "-L", "-F"):
                if argument.startswith(prefix) and argument != prefix:
                    value = argument[len(prefix) :]
                    break
            if argument.startswith("-fmodule-map-file="):
                value = argument.split("=", 1)[1]
                path_is_file = True
            elif argument.startswith("-resource-dir="):
                value = argument.split("=", 1)[1]
            position += 1
        if value and value not in {"<built-in>", "<command-line>"}:
            path = _resolved_argument_path(value, directory)
            if path == source and path_is_file:
                continue
            if not path.exists() or (path_is_file and not path.is_file()):
                raise CheckError(
                    f"missing dependency path in compilation database entry {index}: {path}"
                )


def _source_for_entry(entry: Mapping[str, Any], index: int) -> tuple[Path, Path]:
    directory_value = entry.get("directory")
    file_value = entry.get("file")
    if not isinstance(directory_value, str) or not directory_value:
        raise CheckError(f"compilation database entry {index} has no directory")
    if not isinstance(file_value, str) or not file_value:
        raise CheckError(f"compilation database entry {index} has no source file")
    directory = _canonical(directory_value)
    if not directory.is_dir():
        raise CheckError(f"missing compilation directory in entry {index}: {directory}")
    return directory, _canonical(file_value, directory)


def _target_from_entry(entry: Mapping[str, Any], argv: Sequence[str]) -> str | None:
    outputs: list[str] = []
    output = entry.get("output")
    if isinstance(output, str):
        outputs.append(output)
    for index, argument in enumerate(argv[:-1]):
        if argument == "-o":
            outputs.append(argv[index + 1])
    for candidate in outputs:
        match = _TARGET_FROM_OUTPUT.search(candidate)
        if match:
            return match.group(1)
    return None


def _contains_source_argument(argv: Sequence[str], directory: Path, source: Path) -> bool:
    for argument in argv[1:]:
        if argument.startswith("-"):
            continue
        try:
            if _canonical(argument, directory) == source:
                return True
        except (OSError, ValueError):
            continue
    return False


def _resolve_command_executable(value: str, directory: Path) -> Path | None:
    if os.sep in value or (os.altsep and os.altsep in value):
        candidate = _canonical(value, directory)
        return candidate if candidate.is_file() else None
    located = shutil.which(value)
    return Path(located).resolve() if located else None


def _validate_compilation_context(
    argv: Sequence[str], directory: Path, context: Mapping[str, Any], index: int
) -> None:
    compiler = context["compiler"]
    configured_compiler = Path(compiler["path"]).resolve()
    launcher = compiler.get("launcher", [])
    if isinstance(launcher, str):
        launcher = [launcher] if launcher else []
    if not isinstance(launcher, list) or any(not isinstance(item, str) for item in launcher):
        raise CheckError("configured compiler launcher must be a string list")
    compiler_index = 0
    if launcher:
        configured_launcher = _resolve_command_executable(launcher[0], directory)
        selected_launcher = _resolve_command_executable(argv[0], directory)
        if configured_launcher is None or selected_launcher != configured_launcher:
            raise CheckError(
                f"compilation database launcher differs from configured C++ compiler in entry {index}"
            )
        compiler_index = len(launcher)
    selected_compiler = (
        _resolve_command_executable(argv[compiler_index], directory)
        if compiler_index < len(argv)
        else None
    )
    if selected_compiler != configured_compiler:
        raise CheckError(
            f"compilation database compiler differs from configured C++ compiler in entry {index}: "
            f"selected={selected_compiler}, configured={configured_compiler}"
        )

    selected_sysroots: list[Path] = []
    position = 1
    while position < len(argv):
        argument = argv[position]
        if argument in {"-isysroot", "--sysroot"}:
            if position + 1 >= len(argv):
                raise CheckError(f"missing SDK path in compilation database entry {index}")
            selected_sysroots.append(_canonical(argv[position + 1], directory))
            position += 2
        elif argument.startswith("--sysroot="):
            selected_sysroots.append(_canonical(argument.split("=", 1)[1], directory))
            position += 1
        elif argument.startswith("-isysroot") and argument != "-isysroot":
            selected_sysroots.append(_canonical(argument[len("-isysroot") :], directory))
            position += 1
        else:
            position += 1
    configured_sdk = Path(context["sdk"]).resolve()
    if not selected_sysroots or any(path != configured_sdk for path in selected_sysroots):
        raise CheckError(
            f"compilation database sysroot differs from configured SDK in entry {index}: "
            f"selected={[str(path) for path in selected_sysroots]}, configured={configured_sdk}"
        )


def _compile_identity(source: Path, target: str | None, argv: Sequence[str], directory: Path) -> tuple[str, str | None, tuple[str, ...]]:
    normalized: list[str] = []
    position = 0
    while position < len(argv):
        argument = argv[position]
        if argument == "-o":
            position += 2
            continue
        if argument == "-c":
            position += 1
            continue
        if position > 0 and not argument.startswith("-"):
            try:
                if _canonical(argument, directory) == source:
                    position += 1
                    continue
            except (OSError, ValueError):
                pass
        normalized.append(argument)
        position += 1
    return str(source), target, tuple(normalized)


def _is_under(path: Path, roots: Sequence[Path]) -> bool:
    return any(path == root or root in path.parents for root in roots)


def _validate_target_context(
    argv: Sequence[str],
    target: str,
    context: Mapping[str, Any],
    directory: Path,
    index: int,
) -> None:
    target_contexts = context.get("target_contexts", {})
    target_context = target_contexts.get(target)
    if not isinstance(target_context, dict):
        return
    arguments = list(argv)
    if "-std=c++23" not in arguments or "-std=gnu++23" in arguments:
        raise CheckError(
            f"compilation database entry {index} for target {target} is not strict ISO C++23"
        )
    for expected in target_context.get("compile_options", []):
        if isinstance(expected, str) and "$<" not in expected and expected not in arguments:
            raise CheckError(
                f"compilation database entry {index} omits target {target} compile option {expected!r}"
            )
    for expected in target_context.get("compile_definitions", []):
        if not isinstance(expected, str) or "$<" in expected:
            continue
        flag = f"-D{expected}"
        if flag not in arguments and not any(
            arguments[pos] == "-D" and pos + 1 < len(arguments) and arguments[pos + 1] == expected
            for pos in range(len(arguments))
        ):
            raise CheckError(
                f"compilation database entry {index} omits target {target} definition {expected!r}"
            )
    for expected in target_context.get("include_directories", []):
        if not isinstance(expected, str) or "$<" in expected:
            continue
        resolved = str(Path(expected).resolve())
        include_present = resolved in arguments or any(
            arguments[pos] in {"-I", "-isystem", "-iquote", "-idirafter"}
            and pos + 1 < len(arguments)
            and str(_canonical(arguments[pos + 1], directory)) == resolved
            for pos in range(len(arguments))
        )
        if not include_present:
            include_present = any(
                argument.startswith(prefix)
                and argument != prefix
                and str(_canonical(argument[len(prefix) :], directory)) == resolved
                for argument in arguments
                for prefix in ("-isystem", "-iquote", "-idirafter", "-I")
            )
        if not include_present:
            raise CheckError(
                f"compilation database entry {index} omits target {target} include {resolved}"
            )


def load_entries(build: Path, expected: set[Path]) -> list[dict[str, Any]]:
    """Validate and return every first-party compile context from CMake."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        raise CheckError(f"build directory does not exist: {build_dir}")
    context = load_context(build_dir)
    manifest = _read_json(build_dir / "native_sources.json", "native source manifest")
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise CheckError("native source manifest has an unsupported schema")
    roots_value = manifest.get("first_party_roots")
    sources_value = manifest.get("sources")
    if not isinstance(roots_value, list) or not roots_value or any(
        not isinstance(value, str) for value in roots_value
    ):
        raise CheckError("native source manifest has no valid first-party roots")
    if not isinstance(sources_value, list) or not sources_value:
        raise CheckError("native source manifest has an empty expected translation-unit set")
    roots = [_canonical(value) for value in roots_value]
    for root in roots:
        if not root.is_dir():
            raise CheckError(f"missing first-party source root: {root}")

    expected_by_source: dict[Path, list[dict[str, str]]] = defaultdict(list)
    manifest_identities: set[tuple[Path, str, str]] = set()
    for index, item in enumerate(sources_value):
        if not isinstance(item, dict):
            raise CheckError(f"malformed native source manifest entry {index}")
        source_value = item.get("path")
        target = item.get("target")
        compile_context = item.get("context")
        if not isinstance(source_value, str) or not isinstance(target, str) or not isinstance(compile_context, str):
            raise CheckError(f"malformed native source manifest entry {index}")
        source = _canonical(source_value)
        if not _is_under(source, roots):
            raise CheckError(f"manifest source is outside first-party roots: {source}")
        if not source.is_file():
            raise CheckError(f"missing source file in native source manifest: {source}")
        identity = (source, target, compile_context)
        if identity in manifest_identities:
            raise CheckError(f"duplicate source/target context in native source manifest: {source} ({target})")
        manifest_identities.add(identity)
        expected_by_source[source].append({"target": target, "context": compile_context})

    target_contexts = context["target_contexts"]
    expected_targets = {
        context_entry["target"]
        for source_contexts in expected_by_source.values()
        for context_entry in source_contexts
    }
    missing_target_contexts = expected_targets - set(target_contexts)
    if missing_target_contexts:
        raise CheckError(
            "native check context is missing CMake target contexts: "
            + ", ".join(sorted(missing_target_contexts))
        )

    expected_sources = {_canonical(path) for path in expected}
    manifest_sources = set(expected_by_source)
    if not expected_sources:
        raise CheckError("expected first-party translation-unit set is empty")
    if expected_sources != manifest_sources:
        missing = sorted(str(path) for path in expected_sources - manifest_sources)
        unexpected = sorted(str(path) for path in manifest_sources - expected_sources)
        raise CheckError(f"expected source set differs from CMake manifest; missing={missing}; unexpected={unexpected}")

    database = _read_json(build_dir / "compile_commands.json", "compilation database")
    if not isinstance(database, list):
        raise CheckError("compilation database must be a JSON array")
    if not database:
        raise CheckError("compilation database has an empty translation-unit selection")

    actual_by_source: dict[Path, list[dict[str, Any]]] = defaultdict(list)
    identities: set[tuple[str, str | None, tuple[str, ...]]] = set()
    for index, raw_entry in enumerate(database):
        if not isinstance(raw_entry, dict):
            raise CheckError(f"malformed compilation database entry {index}")
        directory, source = _source_for_entry(raw_entry, index)
        argv = _parse_command(raw_entry, index)
        if not _contains_source_argument(argv, directory, source):
            raise CheckError(f"compilation database entry {index} command does not compile {source}")
        if not _is_under(source, roots):
            continue
        if source not in manifest_sources:
            raise CheckError(f"unexpected first-party translation unit in compilation database: {source}")
        if not source.is_file():
            raise CheckError(f"missing source file referenced by compilation database: {source}")
        _validate_compilation_context(argv, directory, context, index)
        _check_path_arguments(argv, directory, source, index)

        target = _target_from_entry(raw_entry, argv)
        if target_contexts and target is None:
            raise CheckError(
                f"target context is unavailable in compilation database entry {index} for {source}"
            )
        if target is not None:
            allowed_targets = {item["target"] for item in expected_by_source[source]}
            if target not in allowed_targets:
                raise CheckError(
                    f"unexpected target context for {source}: {target}; expected {sorted(allowed_targets)}"
                )
            _validate_target_context(argv, target, context, directory, index)

        identity = _compile_identity(source, target, argv, directory)
        if identity in identities:
            raise CheckError(f"duplicate identical translation unit in compilation database: {source}")
        identities.add(identity)
        actual_by_source[source].append(
            {
                "source": source,
                "directory": directory,
                "arguments": argv,
                "target": target,
                "manifest_contexts": expected_by_source[source],
                "raw": raw_entry,
            }
        )

    missing_sources = sorted(manifest_sources - set(actual_by_source))
    if missing_sources:
        raise CheckError(
            "missing expected translation units: " + ", ".join(path.name for path in missing_sources)
        )

    for source, expected_contexts in expected_by_source.items():
        actual = actual_by_source.get(source, [])
        if len(actual) != len(expected_contexts):
            raise CheckError(
                f"translation-unit context count mismatch for {source}: "
                f"expected {len(expected_contexts)}, selected {len(actual)}"
            )
        known_targets = {item["target"] for item in expected_contexts}
        selected_targets = [item["target"] for item in actual if item["target"] is not None]
        if target_contexts and (
            len(selected_targets) != len(actual) or set(selected_targets) != known_targets
        ):
            raise CheckError(
                f"selected target contexts differ for {source}: expected {sorted(known_targets)}, "
                f"selected {sorted(set(selected_targets))}"
            )
        if selected_targets and (len(selected_targets) != len(actual) or set(selected_targets) != known_targets):
            raise CheckError(
                f"selected target contexts differ for {source}: expected {sorted(known_targets)}, "
                f"selected {sorted(set(selected_targets))}"
            )

    entries = [entry for source in sorted(actual_by_source) for entry in actual_by_source[source]]
    if not entries:
        raise CheckError("first-party translation-unit selection is empty")
    return entries


def _resolve_tool(kind: str, context: Mapping[str, Any], environ: Mapping[str, str]) -> tuple[Path, str]:
    configuration = {
        "tidy": ("CLANG_TIDY", "tidy"),
        "analyzer": ("ANALYZER_CLANG", "analyzer"),
        "cppcheck": ("CPPCHECK", "cppcheck"),
        "frontends": ("GXX", "gxx"),
        "odr": ("GXX", "gxx"),
    }
    if kind not in configuration:
        raise CheckError(f"unknown native sweep kind: {kind}")
    override_name, default_key = configuration[kind]
    overrides = context["tool_overrides"]
    defaults = context["tool_defaults"]
    requested = environ.get(override_name) or overrides.get(override_name) or defaults.get(default_key)
    if not requested:
        raise CheckError(f"no executable configured for {kind}; set {override_name}")
    if os.sep in requested or (os.altsep and os.altsep in requested):
        executable = Path(requested).expanduser().resolve()
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise CheckError(f"required tool is missing or not executable: {requested}")
    else:
        located = shutil.which(requested)
        if located is None:
            raise CheckError(f"required tool is unavailable: {requested} (set {override_name})")
        executable = Path(located).resolve()
    return executable, override_name


def _run_process(
    argv: Sequence[str],
    *,
    cwd: Path,
    log_directory: Path,
    name: str,
    env: Mapping[str, str] | None = None,
    disable_core_dumps: bool = False,
    timeout_seconds: int = 1800,
) -> dict[str, Any]:
    log_directory.mkdir(parents=True, exist_ok=True)
    stdout_path = log_directory / f"{name}.stdout.log"
    stderr_path = log_directory / f"{name}.stderr.log"
    started = datetime.now(timezone.utc).isoformat()
    launch_error: str | None = None
    timed_out = False
    try:
        completed = subprocess.run(
            list(argv),
            cwd=cwd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=dict(env) if env is not None else None,
            preexec_fn=_disable_core_dumps if disable_core_dumps else None,
            check=False,
            timeout=timeout_seconds,
        )
        returncode: int | None = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except FileNotFoundError as error:
        returncode = None
        stdout = ""
        stderr = ""
        launch_error = str(error)
    except PermissionError as error:
        returncode = None
        stdout = ""
        stderr = ""
        launch_error = str(error)
    except subprocess.TimeoutExpired as error:
        returncode = None
        timed_out = True
        stdout_value = error.stdout or ""
        stderr_value = error.stderr or ""
        stdout = stdout_value.decode("utf-8", errors="replace") if isinstance(stdout_value, bytes) else stdout_value
        stderr = stderr_value.decode("utf-8", errors="replace") if isinstance(stderr_value, bytes) else stderr_value
    stdout_path.write_text(stdout)
    stderr_path.write_text(stderr)
    combined = f"{stdout}\n{stderr}"
    diagnostic_lines = [line for line in combined.splitlines() if _DIAGNOSTIC.search(line)]
    process_failed = launch_error is not None or timed_out or returncode != 0
    return {
        "argv": list(argv),
        "cwd": str(cwd),
        "started_at": started,
        "returncode": returncode,
        "launch_error": launch_error,
        "timed_out": timed_out,
        "process_health": "failed" if process_failed else "ok",
        "diagnostics": diagnostic_lines,
        "stdout_log": str(stdout_path),
        "stderr_log": str(stderr_path),
        "stdout": stdout,
        "stderr": stderr,
    }


def _version_record(tool: Path, log_directory: Path) -> dict[str, Any]:
    record = _run_process([str(tool), "--version"], cwd=Path.cwd(), log_directory=log_directory, name="tool-version", timeout_seconds=60)
    version_text = (record["stdout"] + record["stderr"]).strip()
    record["version"] = version_text
    return record


def _temporary_database(build: Path, kind: str, run_id: str, index: int, entry: Mapping[str, Any]) -> Path:
    database_dir = build / "native_check_work" / kind / run_id / f"entry-{index:04d}"
    database_dir.mkdir(parents=True, exist_ok=True)
    raw = entry["raw"]
    database_entry = {
        "directory": str(entry["directory"]),
        "file": str(entry["source"]),
        "arguments": list(entry["arguments"]),
    }
    if isinstance(raw, dict) and isinstance(raw.get("output"), str):
        database_entry["output"] = raw["output"]
    (database_dir / "compile_commands.json").write_text(json.dumps([database_entry], indent=2) + "\n")
    return database_dir


def _replace_compile_source(
    entry: Mapping[str, Any], replacement: Path, *, syntax_only: bool
) -> list[str]:
    argv = list(entry["arguments"])
    source = entry["source"]
    directory = entry["directory"]
    output: list[str] = []
    position = 0
    while position < len(argv):
        argument = argv[position]
        if argument in {"-o", "-MF", "-MT", "-MQ"}:
            position += 2
            continue
        if argument in {"-c", "-MD", "-MMD", "-MP"}:
            position += 1
            continue
        if not argument.startswith("-"):
            try:
                if _canonical(argument, directory) == source:
                    output.append(str(replacement))
                    position += 1
                    continue
            except (OSError, ValueError):
                pass
        output.append(argument)
        position += 1
    if str(replacement) not in output:
        raise CheckError(f"could not replace translation unit {source} in compile context")
    if syntax_only:
        output.append("-fsyntax-only")
    return output


def _standalone_header_roots(first_party_roots: Sequence[Path]) -> list[Path]:
    roots: list[Path] = []
    for first_party_root in first_party_roots:
        for relative in ("src", "tests"):
            candidate = first_party_root / relative
            if candidate.is_dir():
                roots.append(candidate.resolve())
        repository = first_party_root.parent
        benchmark_headers = repository / "tools" / "proto_native_bench"
        if benchmark_headers.is_dir():
            roots.append(benchmark_headers.resolve())
    return roots


def run_header_sweep(build: Path) -> int:
    """Compile every first-party header as the sole include in a generated TU."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        print(f"native headers failed: build directory does not exist: {build_dir}", file=sys.stderr)
        return 1
    kind = "headers"
    run_id = _new_run_id()
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": kind,
        "build": str(build_dir),
        "run_id": run_id,
        "tool_health": "failed",
        "finding_status": "none",
        "status": "failed",
        "header_count": 0,
        "header_translation_unit_count": 0,
        "records": [],
        "errors": [],
    }
    try:
        context = load_context(build_dir)
        source_manifest = _read_json(build_dir / "native_sources.json", "native source manifest")
        expected = {_canonical(item["path"]) for item in source_manifest["sources"]}
        entries = load_entries(build_dir, expected)
        roots = [_canonical(value) for value in source_manifest["first_party_roots"]]
        header_roots = _standalone_header_roots(roots)
        headers = sorted(
            {
                path.resolve()
                for root in header_roots
                for suffix in (".h", ".hh", ".hpp", ".hxx")
                for path in root.rglob(f"*{suffix}")
                if path.is_file()
            }
        )
        if not headers:
            raise CheckError("no first-party headers found for standalone compilation")

        entries_by_root: dict[Path, dict[str, dict[str, Any]]] = defaultdict(dict)
        for root in header_roots:
            matching = [entry for entry in entries if _is_under(entry["source"], [root])]
            for entry in matching:
                entries_by_root[root].setdefault(entry["target"], entry)
        for header in headers:
            root = next(root for root in header_roots if _is_under(header, [root]))
            if not entries_by_root.get(root):
                raise CheckError(f"no configured target compile context for first-party header: {header}")

        compiler = Path(context["compiler"]["path"])
        summary["tool"] = str(compiler)
        summary["tool_version"] = _version_record(
            compiler, build_dir / "native_check_logs" / kind / run_id
        )["version"]
        summary["header_count"] = len(headers)
        summary["header_translation_unit_count"] = sum(
            len(entries_by_root[next(root for root in header_roots if _is_under(header, [root]))])
            for header in headers
        )
        summary["headers"] = [str(header) for header in headers]
    except (CheckError, KeyError, TypeError, StopIteration) as error:
        summary["errors"].append(str(error))
        _write_summary(build_dir, kind, summary)
        print(f"native headers failed: {error}", file=sys.stderr)
        return 1

    work_root = build_dir / "native_check_work" / kind / run_id
    work_root.mkdir(parents=True, exist_ok=True)
    log_directory = build_dir / "native_check_logs" / kind / run_id
    records: list[dict[str, Any]] = []
    tool_health_failed = False
    blocking_findings = False
    index = 0
    for header in headers:
        root = next(root for root in header_roots if _is_under(header, [root]))
        for target, entry in sorted(entries_by_root[root].items()):
            with tempfile.TemporaryDirectory(prefix="header-", dir=work_root) as temporary:
                probe = Path(temporary) / "header_probe.cpp"
                probe.write_text(f'#include "{header.as_posix()}"\nint main() {{ return 0; }}\n')
                argv = _replace_compile_source(entry, probe, syntax_only=True)
                name = f"{index:04d}-{header.stem}-{target}"
                record = _run_process(
                    argv,
                    cwd=entry["directory"],
                    log_directory=log_directory,
                    name=name,
                )
            record.update(
                {
                    "pass": "header",
                    "source": str(header),
                    "target": target,
                }
            )
            records.append(record)
            if record["process_health"] != "ok":
                tool_health_failed = True
            if record["diagnostics"]:
                blocking_findings = True
            index += 1

    summary["records"] = records
    summary["tool_health"] = "failed" if tool_health_failed else "ok"
    summary["finding_status"] = "blocking" if blocking_findings else "none"
    summary["status"] = "passed" if not tool_health_failed and not blocking_findings else "failed"
    summary["log_directory"] = str(log_directory)
    _write_summary(build_dir, kind, summary)
    print(
        f"native headers selection: {summary['header_count']} headers, "
        f"{summary['header_translation_unit_count']} header translation units"
    )
    print(f"native headers tool: {compiler}")
    print(f"native headers tool health: {summary['tool_health']}")
    print(f"native headers findings: {summary['finding_status']}")
    print(f"native headers summary: {build_dir / 'native_check_summaries' / 'headers.json'}")
    if summary["status"] != "passed":
        for record in records:
            for diagnostic in record["diagnostics"]:
                print(diagnostic, file=sys.stderr)
            if record["process_health"] != "ok" or record["diagnostics"]:
                print(
                    f"  {record['source']} [{record['target']}]: rc={record['returncode']} "
                    f"stdout={record['stdout_log']} stderr={record['stderr_log']}",
                    file=sys.stderr,
                )
        return 1
    return 0


def _extra_dependency_args(context: Mapping[str, Any]) -> list[str]:
    args: list[str] = []
    for include in context["include_paths"]:
        args.extend(["-I", include])
    return args


_GCC_WARNING_FLAGS = [
    "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-Wconversion", "-Wsign-conversion",
    "-Wdouble-promotion", "-Wshadow", "-Wcast-qual", "-Wformat=2", "-Wundef",
    "-Wimplicit-fallthrough", "-Wnon-virtual-dtor", "-Wold-style-cast",
    "-Woverloaded-virtual", "-Wnull-dereference", "-Wlogical-op", "-Wduplicated-cond",
    "-Wuseless-cast", "-Wstringop-overflow=4", "-fanalyzer", "-Warith-conversion",
    "-Wduplicated-branches", "-Wrestrict", "-Wformat-signedness", "-Wformat-overflow=2",
    "-Wformat-truncation=2", "-Wextra-semi", "-Wredundant-tags", "-Wcast-function-type",
    "-Wcast-align=strict", "-Wcatch-value=3", "-Wconditionally-supported",
    "-Wdeprecated-copy-dtor", "-Wvolatile", "-Winit-self", "-Wsign-promo",
    "-Wctor-dtor-privacy", "-Wplacement-new=2", "-Wmismatched-new-delete",
    "-Wsized-deallocation", "-Winterference-size", "-Wsubobject-linkage", "-Wsuggest-override",
    "-Wstrict-null-sentinel", "-Walloca", "-Wvla", "-Warray-bounds=2",
    "-Waggressive-loop-optimizations", "-Wstack-usage=8192", "-Wframe-larger-than=8192",
    "-Wmissing-declarations", "-Wswitch-enum",
]


def _gcc_context_args(entry: Mapping[str, Any], context: Mapping[str, Any]) -> list[str]:
    argv = entry["arguments"]
    directory = entry["directory"]
    source = entry["source"]
    output: list[str] = []
    position = 1
    accepted_pairs = {"-I", "-isystem", "-iquote", "-idirafter", "-isysroot", "-D", "-U", "-include"}
    accepted_prefixes = ("-I", "-D", "-U", "-isystem", "-isysroot", "-iquote", "-idirafter")
    while position < len(argv):
        argument = argv[position]
        if argument in {"-o", "-MF", "-MT", "-MQ"}:
            position += 2
            continue
        if argument in {"-c", "-MD", "-MMD", "-MP"} or argument.startswith("-W"):
            position += 1
            continue
        if not argument.startswith("-"):
            try:
                if _canonical(argument, directory) == source:
                    position += 1
                    continue
            except (OSError, ValueError):
                pass
        if argument in accepted_pairs and position + 1 < len(argv):
            output.extend([argument, argv[position + 1]])
            position += 2
            continue
        if argument.startswith(accepted_prefixes):
            output.append(argument)
            position += 1
            continue
        if argument in {"-pthread", "-fPIC", "-fpic", "-ffp-contract=off", "-fvisibility=hidden", "-fvisibility-inlines-hidden"}:
            output.append(argument)
        position += 1
    for include in context["include_paths"]:
        if include not in output:
            output.extend(["-isystem", include])
    return output


def _tool_argv(
    kind: str,
    tool: Path,
    entry: Mapping[str, Any],
    context: Mapping[str, Any],
    *,
    build: Path,
    run_id: str,
    index: int,
    pass_name: str,
) -> tuple[list[str], Path]:
    source = entry["source"]
    log_directory = build / "native_check_logs" / kind / run_id
    if kind == "tidy":
        database = _temporary_database(build, kind, run_id, index, entry)
        return (
            [str(tool), "-p", str(database), "--extra-arg=-Wno-error", "--quiet", str(source)],
            log_directory,
        )
    if kind == "analyzer":
        arguments = list(entry["arguments"])
        arguments[0] = str(tool)
        cleaned: list[str] = []
        position = 0
        while position < len(arguments):
            if arguments[position] == "-o":
                position += 2
            elif arguments[position] in {"-c", "-MD", "-MMD", "-MP"}:
                position += 1
            else:
                cleaned.append(arguments[position])
                position += 1
        extra = ["-Xanalyzer", "-analyzer-output=text"]
        if pass_name == "stable":
            extra.extend(
                [
                    "-Xanalyzer",
                    "-analyzer-checker=optin",
                    "-Xanalyzer",
                    "-analyzer-checker=cplusplus.Move",
                ]
            )
        else:
            extra.extend([
                "-Xanalyzer", "-analyzer-checker=alpha",
                "-Xanalyzer", "-analyzer-config",
                "-Xanalyzer", "aggressive-binary-operation-simplification=true",
                "-Wno-error",
            ])
        return cleaned + ["--analyze", *extra], log_directory
    if kind == "cppcheck":
        database = _temporary_database(build, kind, run_id, index, entry)
        argv = [
            str(tool),
            f"--project={database / 'compile_commands.json'}",
            *_extra_dependency_args(context),
            "--enable=warning,performance,portability",
            "--inconclusive",
            "--std=c++23",
            "--error-exitcode=1",
            "--inline-suppr",
            "-j1",
            "--suppress=missingIncludeSystem",
            "--suppress=uninitMemberVarNoCtor",
            "--suppress=passedByValue",
            "--suppress=returnByReference",
            "--suppress=normalCheckLevelMaxBranches",
            "--quiet",
        ]
        return argv, log_directory
    gcc_args = _gcc_context_args(entry, context)
    if kind == "frontends":
        argv = [str(tool), "-std=c++23", "-fsyntax-only", *gcc_args, *_GCC_WARNING_FLAGS, str(source)]
    elif kind == "odr":
        object_path = build / "native_check_work" / kind / run_id / f"{index:04d}.o"
        object_path.parent.mkdir(parents=True, exist_ok=True)
        argv = [
            str(tool), "-std=c++23", "-O2", "-flto=auto", "-ffat-lto-objects", "-Wno-psabi",
            *gcc_args, "-c", str(source), "-o", str(object_path),
        ]
    else:
        raise CheckError(f"unknown native sweep kind: {kind}")
    return argv, log_directory


def _write_summary(build: Path, kind: str, summary: Mapping[str, Any]) -> None:
    directory = build / "native_check_summaries"
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f"{kind}.json"
    destination.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")


def _new_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def run_diagnostic_controls(build: Path) -> int:
    """Prove each required compiler/analyzer diagnostic is active on controls."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        print(f"native diagnostic controls failed: build directory does not exist: {build_dir}", file=sys.stderr)
        return 1
    kind = "diagnostic-controls"
    run_id = _new_run_id()
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": kind,
        "build": str(build_dir),
        "run_id": run_id,
        "tool_health": "failed",
        "finding_status": "none",
        "status": "failed",
        "controls": [],
        "suppression_counts": {},
        "errors": [],
    }
    try:
        context = load_context(build_dir)
        manifest = _read_json(build_dir / "native_sources.json", "native source manifest")
        expected = {_canonical(item["path"]) for item in manifest["sources"]}
        entries = load_entries(build_dir, expected)
        suppressions = load_suppressions(Path(context["analysis_suppressions_manifest"]))
        compiler = Path(context["compiler"]["path"])
        analyzer, _ = _resolve_tool("analyzer", context, os.environ)
    except (CheckError, KeyError, TypeError) as error:
        summary["errors"].append(str(error))
        _write_summary(build_dir, kind, summary)
        print(f"native diagnostic controls failed: {error}", file=sys.stderr)
        return 1

    version_logs = build_dir / "native_check_logs" / kind / run_id
    compiler_version = _version_record(compiler, version_logs)
    analyzer_version = _version_record(analyzer, version_logs)
    summary["compiler"] = str(compiler)
    summary["compiler_version"] = compiler_version["version"]
    summary["analyzer"] = str(analyzer)
    summary["analyzer_version"] = analyzer_version["version"]
    tool_health_failed = (
        compiler_version["process_health"] != "ok"
        or analyzer_version["process_health"] != "ok"
    )

    control_root = build_dir / "native_check_work" / kind / run_id
    control_root.mkdir(parents=True, exist_ok=True)
    log_directory = build_dir / "native_check_logs" / kind / run_id
    entry = entries[0]
    controls: list[dict[str, Any]] = []

    def compile_control(name: str, source_text: str, flags: Sequence[str], markers: Sequence[str]) -> None:
        nonlocal tool_health_failed
        source = control_root / f"{name}.cpp"
        source.write_text(source_text)
        argv = _replace_compile_source(entry, source, syntax_only=True)
        argv.extend(flags)
        record = _run_process(
            argv,
            cwd=entry["directory"],
            log_directory=log_directory,
            name=name,
        )
        matched = [line for line in record["diagnostics"] if any(marker in line for marker in markers)]
        control_status = (
            record["returncode"] == 0
            and record["launch_error"] is None
            and not record["timed_out"]
            and bool(matched)
        )
        if record["launch_error"] is not None or record["timed_out"] or record["returncode"] not in {0}:
            tool_health_failed = True
        controls.append(
            {
                "name": name,
                "status": "passed" if control_status else "failed",
                "expected_markers": list(markers),
                "matched_diagnostics": matched,
                **record,
            }
        )

    compile_control(
        "discarded_nodiscard",
        "[[nodiscard]] int result() { return 1; }\nint main() { result(); return 0; }\n",
        ["-Wunused-result", "-Wno-error=unused-result"],
        ("unused-result",),
    )
    compile_control(
        "bitwise_instead_of_logical",
        "bool left() { return true; } bool right() { return false; }\n"
        "int main() { if (left() & right()) return 1; return 0; }\n",
        ["-Wbitwise-instead-of-logical", "-Wno-error=bitwise-instead-of-logical"],
        ("bitwise-instead-of-logical",),
    )
    compile_control(
        "extra_semicolon",
        "struct Control { int value;; };\nint main() { return 0; }\n",
        ["-Wextra-semi", "-Wno-error=extra-semi"],
        ("extra-semi",),
    )
    compile_control(
        "signed_bounds",
        "#include <vector>\nint main() { std::vector<int> values(2); int index = 0; return values[index]; }\n",
        ["-Wsign-conversion", "-Wno-error=sign-conversion"],
        ("sign-conversion", "signedness"),
    )

    move_source = control_root / "first_party_use_after_move.cpp"
    move_source.write_text(
        "#include <string>\n#include <utility>\n"
        "void check_move() { std::string source{\"value\"}; "
        "std::string destination{std::move(source)}; (void)source.size(); }\n"
    )
    move_argv = _replace_compile_source(entry, move_source, syntax_only=False)
    move_argv[0] = str(analyzer)
    move_argv.extend(
        [
            "--analyze",
            "-Xanalyzer",
            "-analyzer-output=text",
            "-Xanalyzer",
            "-analyzer-checker=cplusplus.Move",
            "-Wno-error",
        ]
    )
    move_record = _run_process(
        move_argv,
        cwd=entry["directory"],
        log_directory=log_directory,
        name="first_party_use_after_move",
    )
    unsuppressed, move_suppressions = apply_suppressions(
        move_record["diagnostics"], analyzer_version["version"], suppressions
    )
    move_matched = [line for line in unsuppressed if "cplusplus.Move" in line]
    for item in move_suppressions:
        checker = item["checker"]
        summary["suppression_counts"][checker] = summary["suppression_counts"].get(checker, 0) + 1
    move_status = (
        move_record["returncode"] == 0
        and move_record["launch_error"] is None
        and not move_record["timed_out"]
        and bool(move_matched)
    )
    if move_record["launch_error"] is not None or move_record["timed_out"] or move_record["returncode"] != 0:
        tool_health_failed = True
    controls.append(
        {
            "name": "first_party_use_after_move",
            "status": "passed" if move_status else "failed",
            "expected_markers": ["cplusplus.Move"],
            "matched_diagnostics": move_matched,
            "suppressed_diagnostics": move_suppressions,
            **move_record,
        }
    )

    fake_libcpp_include = control_root / "fake-libcpp"
    fake_libcpp_include.mkdir(exist_ok=True)
    (fake_libcpp_include / "__config").write_text("#define _LIBCPP_VERSION 1\n")
    guard_source = control_root / "libcpp_hardening_guard.cpp"
    guard_source.write_text(
        "#include <__config>\n"
        "#ifndef _LIBCPP_VERSION\n#error not libc++\n#endif\n"
        "#ifndef _LIBCPP_HARDENING_MODE_EXTENSIVE\n#error missing hardening mode macro\n#endif\n"
        "#ifndef _LIBCPP_HARDENING_MODE\n#error missing hardening selection macro\n#endif\n"
        "int main() { return 0; }\n"
    )
    guard_record = _run_process(
        [str(compiler), "-std=c++23", "-I", str(fake_libcpp_include), "-fsyntax-only", str(guard_source)],
        cwd=entry["directory"],
        log_directory=log_directory,
        name="libcpp_probe_rejects_missing_mode",
    )
    guard_error = "missing hardening mode macro" in guard_record["stderr"]
    guard_status = guard_record["returncode"] != 0 and guard_error and guard_record["launch_error"] is None
    if guard_record["launch_error"] is not None or guard_record["timed_out"] or not guard_status:
        tool_health_failed = True
    controls.append(
        {
            "name": "libcpp_probe_rejects_missing_mode",
            "status": "passed" if guard_status else "failed",
            "expected_markers": ["missing hardening mode macro"],
            "matched_diagnostics": [guard_record["stderr"]] if guard_error else [],
            "expected_nonzero": True,
            **guard_record,
        }
    )

    failures = [control["name"] for control in controls if control["status"] != "passed"]
    summary["controls"] = controls
    summary["tool_health"] = "failed" if tool_health_failed else "ok"
    summary["status"] = "passed" if not failures and not tool_health_failed else "failed"
    summary["log_directory"] = str(log_directory)
    _write_summary(build_dir, kind, summary)
    print(f"native diagnostic controls: {len(controls)} controls, {len(failures)} failed")
    print(f"native diagnostic control tool health: {summary['tool_health']}")
    print(f"native diagnostic controls summary: {build_dir / 'native_check_summaries' / f'{kind}.json'}")
    if failures:
        for control in controls:
            if control["status"] != "passed":
                print(
                    f"  {control['name']}: rc={control.get('returncode')} "
                    f"stdout={control.get('stdout_log')} stderr={control.get('stderr_log')}",
                    file=sys.stderr,
                )
        return 1
    return 0


def run_context_checks(build: Path) -> int:
    """Exercise SDK selection and compare libc++ hardening modes on real binaries."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        print(f"native context checks failed: build directory does not exist: {build_dir}", file=sys.stderr)
        return 1
    kind = "context"
    run_id = _new_run_id()
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": kind,
        "build": str(build_dir),
        "run_id": run_id,
        "tool_health": "failed",
        "finding_status": "none",
        "status": "failed",
        "sdk_controls": {},
        "hardening_evidence": [],
        "records": [],
        "errors": [],
    }
    try:
        context = load_context(build_dir)
        manifest = _read_json(build_dir / "native_sources.json", "native source manifest")
        entries = load_entries(build_dir, {_canonical(item["path"]) for item in manifest["sources"]})
        uv = shutil.which("uv")
        if uv is None:
            raise CheckError("required tool is unavailable: uv")
        compiler = Path(context["compiler"]["path"])
    except (CheckError, KeyError, TypeError) as error:
        summary["errors"].append(str(error))
        _write_summary(build_dir, kind, summary)
        print(f"native context checks failed: {error}", file=sys.stderr)
        return 1

    work_root = build_dir / "native_check_work" / kind / run_id
    work_root.mkdir(parents=True, exist_ok=True)
    log_directory = build_dir / "native_check_logs" / kind / run_id
    repository = Path(context["source_roots"][0]).parent
    native_root = Path(context["source_roots"][0])
    cmake_records: list[dict[str, Any]] = []

    def configure_case(
        name: str,
        *,
        options: Sequence[str] = (),
        environment: Mapping[str, str] | None = None,
        configured_compiler: Path | None = None,
        use_default_compiler: bool = False,
    ) -> tuple[dict[str, Any], Path]:
        case_build = work_root / f"cmake-{name}"
        argv = [
            str(uv),
            "run",
            "cmake",
            "-S",
            str(native_root),
            "-B",
            str(case_build),
        ]
        if not use_default_compiler:
            argv.append(f"-DCMAKE_CXX_COMPILER={configured_compiler or compiler}")
        argv.extend(options)
        record = _run_process(
            argv,
            cwd=repository,
            log_directory=log_directory,
            name=f"cmake-{name}",
            env=environment,
            timeout_seconds=300,
        )
        record["control"] = name
        cmake_records.append(record)
        return record, case_build

    def configured_context(case_build: Path) -> dict[str, Any]:
        child_context = load_context(case_build)
        child_manifest = _read_json(case_build / "native_sources.json", "configured native source manifest")
        child_expected = {_canonical(item["path"]) for item in child_manifest["sources"]}
        selected = load_entries(case_build, child_expected)
        if not selected:
            raise CheckError(f"configured context selected no translation units: {case_build}")
        return child_context

    sdk_controls: dict[str, str] = {}
    sdk_details: dict[str, Any] = {}
    sdk_request = context["sdk_requested"]

    explicit_record, explicit_build = configure_case(
        "sdk-explicit",
        options=(f"-DCMAKE_OSX_SYSROOT={sdk_request}",),
    )
    try:
        explicit_context = configured_context(explicit_build)
        explicit_ok = (
            explicit_record["returncode"] == 0
            and explicit_context["sdk_source"] == "explicit"
            and Path(explicit_context["sdk"]).resolve() == Path(context["sdk"]).resolve()
            and Path(explicit_context["compiler"]["path"]).resolve() == compiler.resolve()
        )
        sdk_controls["explicit_sdk"] = "passed" if explicit_ok else "failed"
        sdk_details["explicit_sdk"] = {
            "requested": sdk_request,
            "selected": explicit_context["sdk"],
            "compiler": explicit_context["compiler"]["path"],
        }
    except (CheckError, KeyError, TypeError) as error:
        sdk_controls["explicit_sdk"] = "failed"
        sdk_details["explicit_sdk"] = {"error": str(error)}

    missing_sdk = work_root / "missing-explicit-sdk"
    missing_record, _ = configure_case(
        "sdk-missing",
        options=(f"-DCMAKE_OSX_SYSROOT={missing_sdk}",),
    )
    missing_output = missing_record["stdout"] + missing_record["stderr"]
    missing_ok = missing_record["returncode"] != 0 and str(missing_sdk) in missing_output
    sdk_controls["missing_sdk"] = "rejected" if missing_ok else "failed"
    sdk_details["missing_sdk"] = {
        "requested": str(missing_sdk),
        "returncode": missing_record["returncode"],
    }

    default_environment = os.environ.copy()
    default_environment.pop("SDKROOT", None)
    default_environment.pop("CMAKE_OSX_SYSROOT", None)
    default_record, default_build = configure_case(
        "sdk-default",
        environment=default_environment,
        use_default_compiler=True,
    )
    try:
        default_context = configured_context(default_build)
        # This case deliberately omits CMAKE_CXX_COMPILER; an RTSan build may
        # use LLVM clang++ while the default control selects AppleClang.
        default_ok = (
            default_record["returncode"] == 0
            and default_context["sdk_source"] == "default"
            and Path(default_context["sdk"]).is_dir()
        )
        sdk_controls["default_sdk"] = "passed" if default_ok else "failed"
        sdk_details["default_sdk"] = {
            "selected": default_context["sdk"],
            "compiler": default_context["compiler"]["path"],
        }
    except (CheckError, KeyError, TypeError) as error:
        sdk_controls["default_sdk"] = "failed"
        sdk_details["default_sdk"] = {"error": str(error)}

    rejected_default_sdk = work_root / "incompatible-default-sdk"
    rejected_default_sdk.mkdir(parents=True, exist_ok=True)
    fake_bin = work_root / "fake-tool-bin"
    fake_bin.mkdir(parents=True, exist_ok=True)
    fake_xcrun = fake_bin / "xcrun"
    fake_xcrun.write_text(
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--sdk" ] && [ "${2:-}" = "macosx" ] && [ "${3:-}" = "--show-sdk-path" ]; then\n'
        '  printf "%s\\n" "$BIKE_TEST_BAD_SDK"; exit 0;\n'
        "fi\n"
        'exec /usr/bin/xcrun "$@"\n'
    )
    fake_xcrun.chmod(0o755)
    reject_marker = work_root / "link-incompatibility-seen.txt"
    compiler_wrapper = work_root / "compiler-link-control.sh"
    compiler_wrapper.write_text(
        "#!/bin/bash\n"
        "args=(\"$@\")\n"
        "selected_sdk=\"\"\n"
        "syntax_only=0\n"
        "for ((i=0; i<${#args[@]}; ++i)); do\n"
        "  if [[ \"${args[i]}\" == \"-isysroot\" && $((i+1)) -lt ${#args[@]} ]]; then selected_sdk=\"${args[i+1]}\"; fi\n"
        "  if [[ \"${args[i]}\" == \"-fsyntax-only\" ]]; then syntax_only=1; fi\n"
        "done\n"
        "if [[ \"$selected_sdk\" == \"$BIKE_TEST_BAD_SDK\" && $syntax_only == 0 ]]; then\n"
        '  printf "%s\\n" "link-rejected $selected_sdk" >> "$BIKE_TEST_REJECT_MARKER"\n'
        '  printf "%s\\n" "synthetic linker incompatibility for $selected_sdk" >&2\n'
        "  exit 1\n"
        "fi\n"
        'exec /usr/bin/c++ "${args[@]}"\n'
    )
    compiler_wrapper.chmod(0o755)
    fallback_environment = default_environment.copy()
    fallback_environment["PATH"] = str(fake_bin) + os.pathsep + fallback_environment.get("PATH", "")
    fallback_environment["BIKE_TEST_BAD_SDK"] = str(rejected_default_sdk)
    fallback_environment["BIKE_TEST_REJECT_MARKER"] = str(reject_marker)
    fallback_record, fallback_build = configure_case(
        "sdk-default-link-fallback",
        environment=fallback_environment,
        configured_compiler=compiler_wrapper,
    )
    try:
        fallback_context = configured_context(fallback_build)
        fallback_ok = (
            fallback_record["returncode"] == 0
            and fallback_context["sdk_source"] == "default"
            and Path(fallback_context["sdk"]).resolve() != rejected_default_sdk.resolve()
            and reject_marker.is_file()
            and "link-rejected" in reject_marker.read_text()
        )
        sdk_controls["default_sdk_fallback"] = "passed" if fallback_ok else "failed"
        sdk_details["default_sdk_fallback"] = {
            "rejected": str(rejected_default_sdk),
            "selected": fallback_context["sdk"],
            "compiler_wrapper": str(compiler_wrapper),
            "link_rejection_observed": reject_marker.is_file(),
        }
    except (CheckError, KeyError, TypeError) as error:
        sdk_controls["default_sdk_fallback"] = "failed"
        sdk_details["default_sdk_fallback"] = {"error": str(error)}

    hardening_evidence: list[dict[str, Any]] = []
    hardening_supported = context["libcpp_hardening"]["supported"]
    if hardening_supported:
        hardening_source = work_root / "hardening_probe.cpp"
        hardening_source.write_text(
            "#include <__config>\n#include <chrono>\n#include <cstdint>\n"
            "#include <cstdio>\n#include <optional>\n#include <string>\n#include <vector>\n"
            "#if !defined(__STRICT_ANSI__) || __cplusplus < 202302L\n"
            "#error Native first-party code must compile as strict C++23\n#endif\n"
            "#if _LIBCPP_HARDENING_MODE == _LIBCPP_HARDENING_MODE_EXTENSIVE\n"
            "constexpr int selected_mode = 1;\n"
            "#elif _LIBCPP_HARDENING_MODE == _LIBCPP_HARDENING_MODE_FAST\n"
            "constexpr int selected_mode = 2;\n"
            "#else\n#error Unsupported hardening probe mode\n#endif\n"
            "[[clang::noinline]] std::uint64_t element(const std::vector<std::uint64_t>& values, std::size_t index) { return values[index]; }\n"
            "int main(int argc, char**) {\n"
            "  std::vector<std::uint64_t> values(1024);\n"
            "  for (std::size_t i = 0; i < values.size(); ++i) values[i] = i * 17u + 3u;\n"
            "  if (argc > 1) { volatile std::uint64_t invalid = values[values.size()]; return static_cast<int>(invalid); }\n"
            "  std::uint64_t checksum = 0;\n"
            "  const auto start = std::chrono::steady_clock::now();\n"
            "  for (std::uint64_t i = 0; i < 3000000; ++i) checksum += element(values, static_cast<std::size_t>(i & 1023u));\n"
            "  const auto end = std::chrono::steady_clock::now();\n"
            "  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();\n"
            "  std::printf(\"%d %zu %zu %zu %lld %llu\\n\", selected_mode, sizeof(values), sizeof(std::string), sizeof(std::optional<int>), static_cast<long long>(elapsed), static_cast<unsigned long long>(checksum));\n"
            "  return 0;\n}\n"
        )
        arch_args: list[str] = []
        configured_entry = entries[0]
        configured_args = configured_entry["arguments"]
        position = 0
        while position < len(configured_args):
            if configured_args[position] == "-arch" and position + 1 < len(configured_args):
                arch_args.extend(["-arch", configured_args[position + 1]])
                position += 2
            else:
                position += 1
        build_type_options = {
            "Debug": ["-O0", "-g"],
            "Release": ["-O3", "-DNDEBUG"],
            "default": [],
        }
        control_logs = build_dir / "native_check_logs" / kind / run_id
        control_work = work_root / "hardening-binaries"
        control_work.mkdir(parents=True, exist_ok=True)
        hardening_failed = False
        for build_type, optimization in build_type_options.items():
            row: dict[str, Any] = {
                "build_type": build_type,
                "sizeof": {},
                "timings_ns": {},
                "median_ns": {},
                "assertion_active": {},
                "optimization_flags": optimization,
            }
            for mode in ("EXTENSIVE", "FAST"):
                binary = control_work / f"{build_type.lower()}-{mode.lower()}"
                compile_argv = [
                    str(compiler),
                    "-std=c++23",
                    *arch_args,
                    "-isysroot",
                    context["sdk"],
                    "-stdlib=libc++",
                    f"-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_{mode}",
                    *optimization,
                    str(hardening_source),
                    "-o",
                    str(binary),
                ]
                compile_record = _run_process(
                    compile_argv,
                    cwd=repository,
                    log_directory=control_logs,
                    name=f"hardening-{build_type.lower()}-{mode.lower()}-compile",
                    timeout_seconds=180,
                )
                summary["records"].append(compile_record)
                if compile_record["process_health"] != "ok":
                    hardening_failed = True
                    continue

                assertion_record = _run_process(
                    [str(binary), "invalid-element-control"],
                    cwd=repository,
                    log_directory=control_logs,
                    name=f"hardening-{build_type.lower()}-{mode.lower()}-assertion",
                    disable_core_dumps=True,
                    timeout_seconds=30,
                )
                assertion_active = (
                    assertion_record["returncode"] == -int(signal.SIGTRAP)
                )
                summary["records"].append(
                    {
                        **assertion_record,
                        "control": f"hardening-{build_type.lower()}-{mode.lower()}-assertion",
                        "assertion_active": assertion_active,
                        "assertion_category": "libc++ std::vector element access",
                    }
                )
                row["assertion_active"][mode] = assertion_active
                signal_number = (
                    -assertion_record["returncode"]
                    if isinstance(assertion_record["returncode"], int)
                    and assertion_record["returncode"] < 0
                    else None
                )
                row.setdefault("assertion_evidence", {})[mode] = {
                    "returncode": assertion_record["returncode"],
                    "signal": signal.Signals(signal_number).name if signal_number is not None else None,
                    "category": "libc++ std::vector element access",
                    "stderr_log": assertion_record["stderr_log"],
                }
                if mode == "EXTENSIVE" and not assertion_active:
                    hardening_failed = True
                for run_index in range(6):
                    timing_record = _run_process(
                        [str(binary)],
                        cwd=repository,
                        log_directory=control_logs,
                        name=f"hardening-{build_type.lower()}-{mode.lower()}-run-{run_index}",
                        timeout_seconds=30,
                    )
                    summary["records"].append(timing_record)
                    if timing_record["process_health"] != "ok":
                        hardening_failed = True
                        continue
                    fields = timing_record["stdout"].strip().split()
                    if len(fields) != 6:
                        hardening_failed = True
                        continue
                    selected_mode, size_vector, size_string, size_optional, elapsed_ns, _checksum = map(int, fields)
                    expected_mode = 1 if mode == "EXTENSIVE" else 2
                    if selected_mode != expected_mode:
                        hardening_failed = True
                    row["sizeof"][mode] = {
                        "vector_uint64": size_vector,
                        "string": size_string,
                        "optional_int": size_optional,
                    }
                    if run_index > 0:
                        row["timings_ns"].setdefault(mode, []).append(elapsed_ns)
                samples = row["timings_ns"].get(mode, [])
                if samples:
                    row["median_ns"][mode] = int(statistics.median(samples))
            if row["sizeof"].get("EXTENSIVE") != row["sizeof"].get("FAST"):
                hardening_failed = True
            hardening_evidence.append(row)
        if hardening_failed:
            sdk_controls["libcpp_hardening"] = "failed"
        else:
            sdk_controls["libcpp_hardening"] = "passed"
    else:
        sdk_controls["libcpp_hardening"] = "not-applicable"
        summary["hardening_limitation"] = (
            "The selected compiler does not use libc++; native libc++ hardening modes cannot be measured."
        )

    summary["sdk_controls"] = sdk_controls
    summary["sdk_control_details"] = sdk_details
    summary["hardening_evidence"] = hardening_evidence
    summary["tool_health"] = "failed" if any(
        record.get("launch_error") or record.get("timed_out")
        for record in cmake_records
    ) else "ok"
    summary["status"] = (
        "passed"
        if all(value in {"passed", "rejected", "not-applicable"} for value in sdk_controls.values())
        and summary["tool_health"] == "ok"
        else "failed"
    )
    summary["records"].extend(cmake_records)
    summary["log_directory"] = str(log_directory)
    _write_summary(build_dir, kind, summary)
    print(f"native context SDK controls: {sdk_controls}")
    print(f"native context tool health: {summary['tool_health']}")
    print(f"native context summary: {build_dir / 'native_check_summaries' / 'context.json'}")
    if summary["status"] != "passed":
        print("native context controls failed; see retained control logs above", file=sys.stderr)
        for name, status in sdk_controls.items():
            if status not in {"passed", "rejected", "not-applicable"}:
                print(f"  {name}: {status}", file=sys.stderr)
        return 1
    return 0


def _report_alpha_findings(record: Mapping[str, Any], source: Path) -> None:
    diagnostic_lines = record.get("unsuppressed_diagnostics", record["diagnostics"])
    lines = [line for line in diagnostic_lines if re.search(r"\bwarning:", line, re.IGNORECASE)]
    if lines:
        print(f"== alpha findings (report-only): {source}", file=sys.stderr)
        print("\n".join(lines), file=sys.stderr)


def run_sweep(kind: str, build: Path) -> int:
    """Run one native sweep and retain its selection, health, findings, and logs."""
    if kind == "headers":
        return run_header_sweep(build)
    if kind == "diagnostic-controls":
        return run_diagnostic_controls(build)
    if kind == "context":
        return run_context_checks(build)
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        print(f"native sweep failed: build directory does not exist: {build_dir}", file=sys.stderr)
        return 1
    run_id = _new_run_id()
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": kind,
        "build": str(build_dir),
        "run_id": run_id,
        "tool_health": "failed",
        "finding_status": "none",
        "status": "failed",
        "unique_source_count": 0,
        "translation_unit_count": 0,
        "selected_sources": [],
        "records": [],
        "errors": [],
    }
    try:
        context = load_context(build_dir)
        expected = {
            _canonical(item["path"])
            for item in _read_json(build_dir / "native_sources.json", "native source manifest")["sources"]
        }
        entries = load_entries(build_dir, expected)
        suppressions = (
            load_suppressions(Path(context["analysis_suppressions_manifest"]))
            if kind in {"analyzer", "tidy"}
            else []
        )
        tool, override_name = _resolve_tool(kind, context, os.environ)
        summary["tool"] = str(tool)
        summary["tool_override"] = override_name
        summary["tool_overrides"] = {
            name: os.environ[name]
            for name in ("CLANG_TIDY", "ANALYZER_CLANG", "CPPCHECK", "GXX")
            if name in os.environ
        }
        summary["compiler_context"] = context["compiler"]
        summary["sdk"] = context["sdk"]
        summary["python_executable"] = context["python_executable"]
        summary["unique_source_count"] = len({entry["source"] for entry in entries})
        summary["translation_unit_count"] = len(entries)
        summary["selected_sources"] = sorted({str(entry["source"]) for entry in entries})
        summary["selected_translation_units"] = [
            {"source": str(entry["source"]), "target": entry["target"]}
            for entry in entries
        ]
        summary["suppression_counts"] = {}
    except (CheckError, KeyError, TypeError) as error:
        summary["errors"].append(str(error))
        _write_summary(build_dir, kind, summary)
        print(f"native sweep failed: {error}", file=sys.stderr)
        return 1

    print(f"native {kind} selection: {summary['unique_source_count']} unique sources, "
          f"{summary['translation_unit_count']} translation units")
    for source in summary["selected_sources"]:
        print(f"  {source}")
    print(f"native {kind} tool: {tool}")

    log_directory = build_dir / "native_check_logs" / kind / run_id
    version_record = _version_record(tool, log_directory)
    summary["tool_version"] = version_record["version"]
    if version_record["process_health"] != "ok":
        summary["records"].append({"pass": "version", **version_record})

    records: list[dict[str, Any]] = []
    blocking_findings = False
    report_only_findings = False
    tool_health_failed = version_record["process_health"] != "ok"
    if kind == "odr":
        object_paths = [
            build_dir / "native_check_work" / kind / run_id / f"{index:04d}.o"
            for index in range(len(entries))
        ]

    for index, entry in enumerate(entries):
        passes = ("stable", "alpha") if kind == "analyzer" else ("stable",)
        stable_failed = False
        for pass_name in passes:
            if kind == "analyzer" and pass_name == "alpha" and stable_failed:
                continue
            argv, _ = _tool_argv(
                kind,
                tool,
                entry,
                context,
                build=build_dir,
                run_id=run_id,
                index=index,
                pass_name=pass_name,
            )
            name = f"{index:04d}-{entry['source'].stem}-{pass_name}"
            record = _run_process(argv, cwd=entry["directory"], log_directory=log_directory, name=name)
            record.update({"pass": pass_name, "source": str(entry["source"]), "target": entry["target"]})
            if kind in {"analyzer", "tidy"}:
                unsuppressed, matched = apply_suppressions(
                    record["diagnostics"], summary["tool_version"], suppressions
                )
                record["unsuppressed_diagnostics"] = unsuppressed
                record["suppressed_diagnostics"] = matched
                for item in matched:
                    checker = item["checker"]
                    summary["suppression_counts"][checker] = (
                        summary["suppression_counts"].get(checker, 0) + 1
                    )
            else:
                unsuppressed = record["diagnostics"]
            records.append(record)
            if record["process_health"] != "ok":
                tool_health_failed = True
            has_warning = any(re.search(r"\bwarning:", line, re.IGNORECASE) for line in unsuppressed)
            has_error = any(re.search(r"\b(?:error|fatal error):", line, re.IGNORECASE) for line in unsuppressed)
            if kind == "analyzer" and pass_name == "alpha":
                if has_warning:
                    report_only_findings = True
                    _report_alpha_findings(record, entry["source"])
                if has_error:
                    # An alpha finding may be noisy, but an error diagnostic is a broken analysis run.
                    tool_health_failed = True
                continue
            if has_warning or has_error:
                blocking_findings = True
                if kind == "analyzer" and pass_name == "stable":
                    stable_failed = True

    if kind == "odr" and not tool_health_failed:
        linker = [str(tool), "-flto=auto", "-Wodr", "-Wno-psabi", "-r", *map(str, object_paths), "-o", str(build_dir / "native_check_work" / kind / run_id / "combined.o")]
        link_record = _run_process(
            linker,
            cwd=build_dir,
            log_directory=log_directory,
            name="odr-link",
        )
        link_record.update({"pass": "link", "source": None, "target": None})
        records.append(link_record)
        if link_record["process_health"] != "ok":
            tool_health_failed = True
        if link_record["diagnostics"]:
            blocking_findings = True

    summary["records"] = records
    summary["tool_health"] = "failed" if tool_health_failed else "ok"
    summary["finding_status"] = (
        "blocking" if blocking_findings else "report-only" if report_only_findings else "none"
    )
    summary["status"] = (
        "passed" if not tool_health_failed and not blocking_findings else "failed"
    )
    summary["log_directory"] = str(log_directory)
    _write_summary(build_dir, kind, summary)

    print(f"native {kind} tool health: {summary['tool_health']}")
    print(f"native {kind} findings: {summary['finding_status']}")
    if summary.get("suppression_counts"):
        counts = ", ".join(
            f"{checker}={count}" for checker, count in sorted(summary["suppression_counts"].items())
        )
        print(f"native {kind} suppressions: {counts}")
    print(f"native {kind} summary: {build_dir / 'native_check_summaries' / f'{kind}.json'}")
    if summary["status"] != "passed":
        for record in records:
            if record.get("process_health") != "ok" or record.get("diagnostics"):
                for diagnostic in record.get("diagnostics", []):
                    print(diagnostic, file=sys.stderr)
                print(
                    f"  {record.get('pass')} {record.get('source')}: rc={record.get('returncode')} "
                    f"stdout={record.get('stdout_log')} stderr={record.get('stderr_log')}",
                    file=sys.stderr,
                )
        return 1
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--kind",
        required=True,
        choices=("tidy", "analyzer", "cppcheck", "frontends", "odr", "headers", "diagnostic-controls", "context"),
    )
    parser.add_argument("--build", required=True, type=Path)
    arguments = parser.parse_args(argv)
    return run_sweep(arguments.kind, arguments.build)


if __name__ == "__main__":
    raise SystemExit(main())
