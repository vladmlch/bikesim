#!/usr/bin/env python3
"""Validate CMake's native TU manifest and run fail-closed analysis sweeps."""

from __future__ import annotations

import argparse
import json
import os
import platform
import re
import resource
import shlex
import signal
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile
from collections import defaultdict
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

try:  # imported as tools.native_checks by the test suite
    from tools import native_toolchain
except ImportError:  # invoked as `python tools/native_checks.py`: tools/ is sys.path[0]
    import native_toolchain


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
# Clang echoes the offending source line under each diagnostic with a
# "   <line> | <code>" gutter. Quoted code can contain text like "error:"
# (a loop variable named error, a label, a string literal), so collected
# diagnostic lines are not all severity-bearing headers.
_SNIPPET_CONTEXT = re.compile(r"^\s*\d*\s*\|")
# A severity token only counts where a diagnostic header can put it: after a
# "file:line[:col]:" location, after a tool-name prefix ("clang++: error:"),
# or at the start of the line ("error: unable to open output file").
_DIAGNOSTIC_HEADER = re.compile(
    r"^(?:.*?:\d+(?::\d+)?:|[^\s:][^:\n]*:)?\s*"
    r"(?P<severity>warning|error|fatal error):",
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


def _diagnostic_severity(line: str) -> str | None:
    """Return "warning" or "error" for a diagnostic header line, else None.

    Snippet/context lines clang echoes beneath a diagnostic are excluded so
    quoted source text cannot masquerade as a severity-bearing finding.
    """
    if _SNIPPET_CONTEXT.match(line):
        return None
    match = _DIAGNOSTIC_HEADER.match(line)
    if match is None:
        return None
    severity = match.group("severity").lower()
    return "error" if severity == "fatal error" else severity


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


def _reconfigure_hint(build: Path | None = None) -> str:
    """The actionable reconfigure command attached to stale-context errors."""
    directory = str(build) if build is not None else "<build>"
    return (
        "reconfigure the build with the selected interpreter: "
        f"cmake -S native -B {directory} "
        "-DPython_EXECUTABLE=<project python>"
    )


def validate_platform_context(context: dict[str, Any]) -> dict[str, Any]:
    """Validate the platform/compiler/SDK/sysroot/stdlib records.

    Returns the validated context — the same mapping, with recorded paths
    canonicalized in place. Schema-1 contexts are Darwin-only records —
    they cannot express a no-SDK build, so every schema-1 file is validated
    with Darwin semantics and any schema-1 context on a Linux host fails the
    SDK existence check.
    """
    schema_version = context.get("schema_version")
    if schema_version not in (1, 2):
        raise CheckError(
            f"native check context has an unsupported schema "
            f"({schema_version!r}); {_reconfigure_hint()}"
        )

    if schema_version == 2:
        platform_record = context.get("platform")
        if not isinstance(platform_record, dict):
            raise CheckError("native check context has no platform record")
        system = platform_record.get("system")
        if system not in {"Darwin", "Linux"}:
            raise CheckError(
                f"native check context has an unsupported platform system: {system!r}"
            )
        host_system = platform.system()
        if system != host_system or platform_record.get("host_system") != host_system:
            raise CheckError(
                f"native check platform records system={system!r} "
                f"host_system={platform_record.get('host_system')!r} on a "
                f"{host_system!r} host"
            )
        machine = platform_record.get("machine")
        if machine != platform.machine():
            raise CheckError(
                f"native check platform machine {machine!r} does not match "
                f"host machine {platform.machine()!r}"
            )
        if platform_record.get("pointer_bits") != struct.calcsize("P") * 8:
            raise CheckError(
                "native check platform pointer width does not match the host"
            )

        sysroot_value = context.get("sysroot")
        if sysroot_value is None:
            context["sysroot"] = None
        elif isinstance(sysroot_value, str) and sysroot_value:
            sysroot_path = _require_existing_path(
                sysroot_value, "configured sysroot"
            )
            if not sysroot_path.is_dir():
                raise CheckError(
                    f"configured sysroot is not a directory: {sysroot_path}"
                )
            context["sysroot"] = str(sysroot_path)
        else:
            raise CheckError(
                "native check context sysroot must be null or a directory path"
            )
    else:
        system = "Darwin"

    if system == "Darwin":
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
        if schema_version == 2:
            # On Darwin the selected sysroot IS the SDK — the writer emits
            # identical strings and any drift between them is a writer defect.
            if context.get("sysroot") != context["sdk"]:
                raise CheckError(
                    f"Darwin sysroot {context.get('sysroot')!r} does not "
                    f"equal the selected SDK {context['sdk']!r}"
                )
    else:
        if context.get("sdk") not in (None, ""):
            raise CheckError(
                "Linux native check context must not record an Apple SDK"
            )
        if context.get("sdk_requested") not in (None, ""):
            raise CheckError(
                "Linux native check context must not record a requested SDK"
            )
        if context.get("sdk_source") != "not-applicable":
            raise CheckError(
                "Linux native check context must record sdk_source "
                "'not-applicable'"
            )
        context["sdk"] = None
        context["sdk_requested"] = None

    if schema_version == 2:
        stdlib = context.get("stdlib")
        if not isinstance(stdlib, dict):
            raise CheckError("native check context has no stdlib record")
        stdlib_name = stdlib.get("name")
        if stdlib_name not in {"libc++", "libstdc++"}:
            raise CheckError(
                f"native check context has an unsupported stdlib: {stdlib_name!r}"
            )
        if system == "Darwin" and stdlib_name != "libc++":
            raise CheckError(
                "the Darwin native build requires the libc++ standard library"
            )
        stdlib_version = stdlib.get("version")
        if not isinstance(stdlib_version, str) or not stdlib_version:
            raise CheckError("native check context stdlib has no version")
        stdlib_hardening = stdlib.get("hardening")
        if not isinstance(stdlib_hardening, dict):
            raise CheckError("native check context stdlib has no hardening record")
        expected_modes = {
            "libc++": {"EXTENSIVE", "FAST"},
            "libstdc++": {"ASSERTIONS"},
        }[stdlib_name]
        if stdlib_hardening.get("mode") not in expected_modes:
            raise CheckError(
                f"native check context stdlib hardening mode "
                f"{stdlib_hardening.get('mode')!r} is invalid for {stdlib_name}"
            )
        if stdlib_hardening.get("effective") is not True:
            raise CheckError(
                "native check context stdlib hardening must be effective"
            )

        numerics = context.get("numerics")
        if not isinstance(numerics, dict):
            raise CheckError("native check context has no numerics record")
        if not isinstance(numerics.get("backend"), str) or not numerics["backend"]:
            raise CheckError("native check context numerics has no backend")

        policy = context.get("policy")
        if not isinstance(policy, dict):
            raise CheckError("native check context has no policy record")
        for field in ("required_flags", "accepted_warnings"):
            values = policy.get(field)
            if not isinstance(values, list) or any(
                not isinstance(value, str) for value in values
            ):
                raise CheckError(
                    f"native check context policy {field!r} must be a string list"
                )
        rejected = policy.get("rejected_warnings")
        if not isinstance(rejected, list) or any(
            not isinstance(item, dict)
            or not isinstance(item.get("flag"), str)
            or not isinstance(item.get("reason"), str)
            for item in rejected
        ):
            raise CheckError(
                "native check context policy rejected_warnings must be a "
                "list of flag/reason records"
            )

        capabilities = context.get("capabilities")
        if not isinstance(capabilities, dict):
            raise CheckError("native check context has no capabilities record")
        # TSan was added after the schema-2 contexts. Validate it whenever
        # present while retaining support for prior captures without the lane.
        sanitizer_capabilities = ("asan", "rtsan", "coverage")
        if "tsan" in capabilities:
            sanitizer_capabilities += ("tsan",)
        for capability in sanitizer_capabilities:
            record = capabilities.get(capability)
            if not isinstance(record, dict):
                raise CheckError(
                    f"native check context has no {capability} capability record"
                )
            for field in ("requested", "supported", "enabled"):
                if not isinstance(record.get(field), bool):
                    raise CheckError(
                        f"native check context capability {capability}.{field} "
                        "must be boolean"
                    )
            if record["enabled"] and not record["supported"]:
                raise CheckError(
                    f"native check context capability {capability} is enabled "
                    "without support evidence"
                )
            if record["enabled"] and not record["requested"]:
                raise CheckError(
                    f"native check context capability {capability} is enabled "
                    "without a requested lane"
                )
            evidence = record.get("evidence")
            if isinstance(evidence, str):
                if not evidence:
                    raise CheckError(
                        f"native check context capability {capability} has an "
                        "empty evidence record"
                    )
            elif isinstance(evidence, dict):
                if not evidence or any(
                    not isinstance(key, str) or not isinstance(value, str)
                    for key, value in evidence.items()
                ):
                    raise CheckError(
                        f"native check context capability {capability} "
                        "evidence must map names to strings"
                    )
            else:
                raise CheckError(
                    f"native check context capability {capability} has no "
                    "evidence record"
                )
            if "runtime" not in record:
                raise CheckError(
                    f"native check context capability {capability} has no "
                    "runtime record"
                )
            runtime_value = record["runtime"]
            if runtime_value is not None:
                runtime_path = _require_existing_path(
                    runtime_value, f"capability {capability} runtime"
                )
                if not runtime_path.is_file():
                    raise CheckError(
                        f"capability {capability} runtime is not a file: "
                        f"{runtime_path}"
                    )
                record["runtime"] = str(runtime_path)
            if capability in {"asan", "rtsan", "tsan"}:
                # A compiler-resolved runtime is mandatory on an enabled
                # sanitizer lane. TSan is linked, never preloaded.
                if record["enabled"] and record["runtime"] is None:
                    raise CheckError(
                        f"native check context capability {capability} is "
                        "enabled without a recorded sanitizer runtime"
                    )
                compiler_id = record.get("compiler_id")
                if not isinstance(compiler_id, str) or not compiler_id:
                    raise CheckError(
                        f"native check context capability {capability} has "
                        "no compiler identity record"
                    )
                target = record.get("target")
                if target is not None and (
                    not isinstance(target, str) or not target
                ):
                    raise CheckError(
                        f"native check context capability {capability} "
                        "target must be a triple string or null"
                    )
        frontend_tools = capabilities.get("frontend_tools")
        if not isinstance(frontend_tools, dict):
            raise CheckError(
                "native check context has no frontend_tools capability record"
            )
        for tool_name, record in frontend_tools.items():
            if not isinstance(record, dict) or not isinstance(
                record.get("configured"), str
            ):
                raise CheckError(
                    f"native check frontend tool {tool_name!r} has no "
                    "configured spelling"
                )
            located = record.get("located")
            if located is not None:
                if not isinstance(located, str) or not located:
                    raise CheckError(
                        f"native check frontend tool {tool_name!r} located "
                        "record must be a path or null"
                    )
                located_path = _require_existing_path(
                    located, f"located frontend tool {tool_name}",
                    executable=True, preserve_symlink=True,
                )
                record["located"] = str(located_path)
    return context


def load_context(build: Path) -> dict[str, Any]:
    """Load CMake's toolchain/dependency context and fail on stale paths."""
    build_dir = _canonical(build)
    if not build_dir.is_dir():
        raise CheckError(f"build directory does not exist: {build_dir}")
    context_path = build_dir / "native_check_context.json"
    if not context_path.is_file():
        # Multi-config generators cannot share one compile_commands.json;
        # the writer emits one context per configuration. Select the file
        # matching the commands this build actually produced.
        candidates = sorted(
            (build_dir / "native_check_context").glob("*.json")
        ) if (build_dir / "native_check_context").is_dir() else []
        if len(candidates) != 1:
            raise CheckError(
                f"native check context selection is ambiguous: {build_dir} "
                f"has no native_check_context.json and "
                f"{len(candidates)} per-configuration contexts"
            )
        context_path = candidates[0]
    context = _read_json(context_path, "native check context")
    if not isinstance(context, dict):
        raise CheckError(
            "native check context has an unsupported schema "
            f"(not an object); {_reconfigure_hint(build_dir)}"
        )

    compiler = context.get("compiler")
    if not isinstance(compiler, dict):
        raise CheckError("native check context has no compiler record")
    compiler_path = _require_existing_path(
        compiler.get("path"), "configured C++ compiler", executable=True,
        preserve_symlink=True,
    )
    compiler["path"] = str(compiler_path)

    python_executable = _require_existing_path(
        context.get("python_executable"), "configured Python executable",
        executable=True,
        # A venv interpreter is a symlink to its base python; resolving it
        # would drop the venv site-packages the dependency probe needs.
        preserve_symlink=True,
    )
    context["python_executable"] = str(python_executable)
    validate_platform_context(context)

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
    # The configured sysroot is the Apple SDK on Darwin; on Linux it is an
    # explicit CMAKE_SYSROOT or null. A null configured sysroot admits no
    # sysroot argument at all — any -isysroot/--sysroot in an entry would be
    # a selection the build context did not record.
    configured_sysroot_value = context.get("sysroot") or context.get("sdk")
    configured_sysroot = (
        Path(configured_sysroot_value).resolve()
        if configured_sysroot_value
        else None
    )
    if configured_sysroot is None:
        if selected_sysroots:
            raise CheckError(
                f"compilation database selects a sysroot in entry {index} but "
                f"the build context configures none: "
                f"selected={[str(path) for path in selected_sysroots]}"
            )
    elif not selected_sysroots or any(
        path != configured_sysroot for path in selected_sysroots
    ):
        raise CheckError(
            f"compilation database sysroot differs from configured sysroot in entry {index}: "
            f"selected={[str(path) for path in selected_sysroots]}, configured={configured_sysroot}"
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
    # A live environment override wins over the configure-time recording;
    # either is an explicit request that must resolve — never a fallthrough.
    override = environ.get(override_name) or overrides.get(override_name)
    # Resolution order: valid explicit override; a sibling of the configured
    # compiler from its own toolchain family; the recorded default spelling
    # (PATH); the configure-time `located` evidence; then, on Darwin only,
    # the existing Homebrew/Xcode discovery candidates.
    candidates = native_toolchain.sibling_candidates(context["compiler"], default_key)
    default = defaults.get(default_key)
    if default:
        candidates.append(default)
    frontend_tools = context.get("capabilities", {}).get("frontend_tools", {})
    located = frontend_tools.get(default_key, {}).get("located")
    if isinstance(located, str) and located:
        candidates.append(located)
    platform_system = (context.get("platform") or {}).get("system")
    if platform_system == "Darwin" or context.get("schema_version") == 1:
        candidates.extend(native_toolchain.darwin_tool_candidates(default_key))
    try:
        tool = native_toolchain.resolve_tool(kind, override, candidates)
    except ValueError as error:
        raise CheckError(str(error)) from error
    return tool, override_name


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

    def _probe(job: tuple[int, Path, str, Mapping[str, Any]]) -> dict[str, Any]:
        index, header, target, entry = job
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
        return record

    jobs: list[tuple[int, Path, str, Mapping[str, Any]]] = []
    index = 0
    for header in headers:
        root = next(root for root in header_roots if _is_under(header, [root]))
        for target, entry in sorted(entries_by_root[root].items()):
            jobs.append((index, header, target, entry))
            index += 1

    # The probes are independent subprocesses; a pool sized to the host keeps
    # the ~10k-TU sweep bounded by compiler throughput rather than serial
    # launch latency. pool.map preserves submission order, so records keep the
    # deterministic header x target enumeration of the serial version.
    workers = min(32, (os.cpu_count() or 4))
    records: list[dict[str, Any]] = []
    tool_health_failed = False
    blocking_findings = False
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for record in pool.map(_probe, jobs):
            records.append(record)
            if record["process_health"] != "ok":
                tool_health_failed = True
            if record["diagnostics"]:
                blocking_findings = True

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


# Mandatory diagnostics for the secondary frontend: these are never probed
# away — a "GCC" that rejects them is not a usable frontend.
_GCC_REQUIRED_FLAGS = ["-Werror", "-Wpedantic"]

# Optional GCC warning names. Each is probed against the *resolved* frontend
# once per run (a trivial TU cannot trip a real warning, so a batch rejection
# means an unsupported spelling — per-flag probing then drops only the
# offending names, never the whole set).
_GCC_OPTIONAL_WARNING_FLAGS = [
    "-Wall", "-Wextra", "-Wconversion", "-Wsign-conversion",
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

_GCC_WARNING_FLAGS = [*_GCC_REQUIRED_FLAGS, *_GCC_OPTIONAL_WARNING_FLAGS]

# Tokens the secondary frontend keeps from the TU's own compile context:
# include roots and definitions (paired and joined spellings), the recorded
# sysroot/SDK, the language version, architecture/ABI, and the FP/PIC/thread
# and hardening flags both frontends share. Warning spellings (-W*),
# clang-only spellings (-stdlib=), sanitizer runtimes, optimization levels
# and output artifacts are rebuilt or dropped — never forwarded verbatim
# into a different toolchain.
_FORWARD_PAIRS = {
    "-I", "-isystem", "-iquote", "-idirafter",
    "-isysroot", "--sysroot",
    "-D", "-U", "-include", "-imacros",
    "-arch",
}
_FORWARD_PREFIXES = (
    "-I", "-D", "-U",
    "-isystem", "-iquote", "-idirafter", "-imacros",
    "-isysroot", "--sysroot=",
)
_SYSROOT_PAIRS = {"-isysroot", "--sysroot"}
_SYSROOT_JOINED = ("-isysroot", "--sysroot=")
_FORWARD_EXACT = {
    "-pthread", "-fPIC", "-fpic", "-fPIE", "-fpie", "-fno-common",
    "-fvisibility=hidden", "-fvisibility-inlines-hidden",
    "-fstack-protector", "-fstack-protector-strong", "-fstack-protector-all",
    "-fno-omit-frame-pointer",
}
_FORWARD_FLAG_PREFIXES = (
    "-std=", "-ffp-contract=", "-fvisibility=",
    "-ftrivial-auto-var-init=", "-fstrict-flex-arrays=",
    "-fcf-protection=", "-fabi-version=",
    "-march=", "-mcpu=", "-mtune=", "-mabi=", "-mfpu=",
    "-mfloat-abi=", "-mbranch-protection=",
)


def _compiler_launcher(context: Mapping[str, Any]) -> list[str]:
    launcher = context.get("compiler", {}).get("launcher") or []
    if isinstance(launcher, str):
        launcher = [launcher] if launcher else []
    return list(launcher)


def _check_forwarded_sysroot(
    value: str, directory: Path, configured_sysroot: Path | None
) -> None:
    """A sysroot may be forwarded to the secondary frontend only when the
    build context recorded exactly that sysroot — anything else is context
    drift (an Apple SDK must never reach a native Linux GCC)."""
    selected = _canonical(value, directory)
    if configured_sysroot is None or selected != configured_sysroot:
        raise CheckError(
            "secondary-frontend compile context selects a sysroot the build "
            f"context did not record: selected={selected}, "
            f"configured={configured_sysroot}"
        )


def _gcc_context_args(entry: Mapping[str, Any], context: Mapping[str, Any]) -> list[str]:
    """Rebuild one TU's semantic compile context for the secondary frontend."""
    argv = entry["arguments"]
    directory = entry["directory"]
    source = entry["source"]
    # Entries were validated to begin with [*launcher, compiler]; both tokens
    # belong to the configured toolchain, not to the semantic context.
    position = 1 + len(_compiler_launcher(context))
    configured_sysroot_value = context.get("sysroot") or context.get("sdk")
    configured_sysroot = (
        Path(configured_sysroot_value).resolve() if configured_sysroot_value else None
    )
    output: list[str] = []
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
        if argument in _FORWARD_PAIRS and position + 1 < len(argv):
            if argument in _SYSROOT_PAIRS:
                _check_forwarded_sysroot(argv[position + 1], directory, configured_sysroot)
            output.extend([argument, argv[position + 1]])
            position += 2
            continue
        joined = next(
            (
                prefix
                for prefix in _FORWARD_PREFIXES
                if argument.startswith(prefix) and argument != prefix
            ),
            None,
        )
        if joined is not None:
            if joined in _SYSROOT_JOINED:
                _check_forwarded_sysroot(
                    argument[len(joined) :], directory, configured_sysroot
                )
            output.append(argument)
            position += 1
            continue
        if argument in _FORWARD_EXACT or argument.startswith(_FORWARD_FLAG_PREFIXES):
            output.append(argument)
        position += 1
    for include in context["include_paths"]:
        if include not in output:
            output.extend(["-isystem", include])
    return output


def _probe_frontend_warning_flags(
    tool: Path,
    *,
    work_directory: Path,
    log_directory: Path,
) -> tuple[list[str], list[dict[str, Any]]]:
    """Probe which optional GCC warning names the resolved frontend supports.

    Mandatory flags (strict C++23, -Werror, -Wpedantic) are proven first and
    are never dropped; optional warning names are batch-probed, then probed
    individually when the batch fails — one unsupported spelling must not
    remove the whole warning set, but a frontend accepting none of them is
    not a usable analysis compiler.
    """
    probe_directory = work_directory / "warning-probe"
    probe_directory.mkdir(parents=True, exist_ok=True)
    probe_source = probe_directory / "probe.cpp"
    probe_source.write_text("int main() { return 0; }\n")
    base = ["-std=c++23", "-fsyntax-only", *_GCC_REQUIRED_FLAGS]
    required_record = _run_process(
        [str(tool), *base, str(probe_source)],
        cwd=probe_directory,
        log_directory=log_directory,
        name="frontend-required-flags",
    )
    if required_record["process_health"] != "ok":
        raise CheckError(
            f"secondary frontend {tool} rejects mandatory flags "
            f"{_GCC_REQUIRED_FLAGS}: {required_record['diagnostics']}"
        )
    batch_record = _run_process(
        [str(tool), *base, *_GCC_OPTIONAL_WARNING_FLAGS, str(probe_source)],
        cwd=probe_directory,
        log_directory=log_directory,
        name="frontend-warning-batch",
    )
    applied: list[str] = []
    unavailable: list[dict[str, Any]] = []
    if batch_record["process_health"] == "ok":
        applied = list(_GCC_OPTIONAL_WARNING_FLAGS)
    else:
        for flag in _GCC_OPTIONAL_WARNING_FLAGS:
            name = "frontend-warning-" + re.sub(r"[^A-Za-z0-9]+", "_", flag).strip("_")
            record = _run_process(
                [str(tool), *base, flag, str(probe_source)],
                cwd=probe_directory,
                log_directory=log_directory,
                name=name,
            )
            if record["process_health"] == "ok":
                applied.append(flag)
            else:
                unavailable.append(
                    {
                        "flag": flag,
                        "returncode": record["returncode"],
                        "diagnostics": record["diagnostics"],
                    }
                )
    if not applied:
        raise CheckError(
            f"secondary frontend {tool} accepts none of the optional warning "
            "policy; it is not a usable analysis frontend"
        )
    return [*_GCC_REQUIRED_FLAGS, *applied], unavailable


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
    frontend_flags: Sequence[str] | None = None,
) -> tuple[list[str], Path]:
    source = entry["source"]
    log_directory = build / "native_check_logs" / kind / run_id
    launcher = _compiler_launcher(context)
    if kind == "tidy":
        database = _temporary_database(build, kind, run_id, index, entry)
        argv = [str(tool), "-p", str(database), "--extra-arg=-Wno-error", "--quiet"]
        # First-party sources outside native/ (e.g. tools/proto_native_bench)
        # do not inherit native/.clang-tidy through parent-directory
        # discovery — without it clang-tidy reports "no checks enabled".
        policy = Path(context["source_roots"][0]) / ".clang-tidy"
        if policy.is_file():
            argv.append(f"--config-file={policy}")
        argv.append(str(source))
        return argv, log_directory
    if kind == "analyzer":
        arguments = list(entry["arguments"])
        # Entries begin with [*launcher, compiler]: preserve the launcher
        # spelling and replace the compiler slot with the analyzer driver.
        arguments[len(launcher)] = str(tool)
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
    warnings = list(frontend_flags) if frontend_flags is not None else list(_GCC_WARNING_FLAGS)
    if kind == "frontends":
        argv = [*launcher, str(tool), "-std=c++23", "-fsyntax-only", *gcc_args, *warnings, str(source)]
    elif kind == "odr":
        object_path = build / "native_check_work" / kind / run_id / f"{index:04d}.o"
        object_path.parent.mkdir(parents=True, exist_ok=True)
        argv = [
            *launcher, str(tool), "-std=c++23", "-O2", "-flto=auto", "-ffat-lto-objects", "-Wno-psabi",
            # Sweep TUs may each define `main` (contract tests, the benchmark);
            # rename it uniquely per TU so the relocatable link of all objects
            # has no duplicate-symbol conflict. -Wodr checks types across TUs
            # regardless of the entry-point symbol name.
            f"-Dmain=native_check_odr_entry_{index:04d}",
            *gcc_args, "-c", str(source), "-o", str(object_path),
        ]
    else:
        raise CheckError(f"unknown native sweep kind: {kind}")
    return argv, log_directory


# The toolchain family each resolved sweep tool must identify as; a resolved
# tool that identifies as something else is a selection error, not a pass.
_EXPECTED_TOOL_FAMILY = {
    "tidy": {"clang-tidy"},
    "analyzer": {"clang"},
    "cppcheck": {"cppcheck"},
    "frontends": {"gcc"},
    "odr": {"gcc"},
}


def _inspect_sweep_tool(kind: str, tool: Path, summary: dict[str, Any]) -> bool:
    """Record identity/target/resource evidence for the resolved tool.

    Returns True when the tool identifies as the wrong family (e.g. a GNU
    compiler handed the tidy sweep is a selection error). Tools that cannot
    identify themselves are recorded — the sweep's own diagnostics remain
    the qualification gate for the chosen tool/context combination.
    """
    try:
        inspection = native_toolchain.inspect_tool(tool)
    except ValueError as error:
        summary["tool_inspection_error"] = str(error)
        return False
    summary["tool_inspection"] = inspection
    summary["tool_resolved"] = inspection["resolved_path"]
    expected = _EXPECTED_TOOL_FAMILY[kind]
    if inspection["family"] not in expected:
        summary["errors"].append(
            f"resolved {kind} tool {tool} identifies as {inspection['family']!r}; "
            f"expected one of {sorted(expected)}"
        )
        return True
    return False


def _write_summary(build: Path, kind: str, summary: Mapping[str, Any]) -> None:
    directory = build / "native_check_summaries"
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f"{kind}.json"
    destination.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")


def _new_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


# The standard-library hardening control probe, shared by the context
# controls and the portability tests. The compile-time guards make the
# control's intent self-evidencing: a hardened build carries exactly the
# stdlib's production hardening define and never _GLIBCXX_DEBUG (which
# substitutes a different container ABI rather than hardening the real one);
# an unhardened build proves no hardening define leaked into it.
# GCC reports __cplusplus 202100L for -std=c++23 until GCC 14; 202100L is
# the floor for a dialect the compiler accepts as strict C++23.
_STDLIB_HARDENING_PROBE = (
    "#include <cstdint>\n#include <vector>\n"
    "#if !defined(__STRICT_ANSI__) || __cplusplus < 202100L\n"
    "#error Native first-party code must compile as strict C++23\n#endif\n"
    "#if defined(_GLIBCXX_DEBUG)\n"
    "#error _GLIBCXX_DEBUG changes container ABI; it is not a production hardening mode\n"
    "#endif\n"
    "#if defined(BIKE_HARDENED) && defined(__GLIBCXX__) && !defined(_GLIBCXX_ASSERTIONS)\n"
    "#error the hardened libstdc++ control requires _GLIBCXX_ASSERTIONS\n"
    "#endif\n"
    "#if !defined(BIKE_HARDENED) && defined(_GLIBCXX_ASSERTIONS)\n"
    "#error _GLIBCXX_ASSERTIONS leaked into the unhardened control\n"
    "#endif\n"
    "int main(int argc, char**) {\n"
    "  std::vector<std::uint64_t> values(64);\n"
    "  if (argc > 1) {\n"
    "    volatile std::uint64_t invalid = values[values.size() + 64];\n"
    "    return static_cast<int>(invalid);\n"
    "  }\n"
    "  return 0;\n"
    "}\n"
)


def _stdlib_probe_spec(context: Mapping[str, Any]) -> dict[str, Any]:
    """The hardening control parameters for the configured standard library.

    libc++ asserts through its selected hardening mode (SIGTRAP on aarch64,
    SIGILL on x86_64 via __builtin_trap's ud2); libstdc++ asserts through
    _GLIBCXX_ASSERTIONS/__glibcxx_assert_fail (SIGABRT). -stdlib= is a clang
    driver spelling — a GNU frontend never receives it.
    """
    stdlib_name = context["stdlib"]["name"]
    compiler_id = context["compiler"].get("id", "")
    if stdlib_name == "libc++":
        return {
            "stdlib": "libc++",
            "mode": context["stdlib"]["hardening"]["mode"],
            "define": (
                "-D_LIBCPP_HARDENING_MODE="
                f"_LIBCPP_HARDENING_MODE_{context['stdlib']['hardening']['mode']}"
            ),
            "stdlib_args": (
                ["-stdlib=libc++"] if compiler_id in {"Clang", "AppleClang"} else []
            ),
            # The explicit "none" spelling unhardens whatever default the
            # toolchain's libc++ headers select.
            "unhardened_args": ["-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_NONE"],
            "expected_signals": {int(signal.SIGTRAP), int(signal.SIGILL)},
            "probe": _STDLIB_HARDENING_PROBE,
        }
    return {
        "stdlib": "libstdc++",
        "mode": context["stdlib"]["hardening"]["mode"],
        "define": "-D_GLIBCXX_ASSERTIONS",
        "stdlib_args": [],
        # libstdc++ may self-enable assertions (driver predefines, or the
        # "unoptimized builds assert" default in newer GCC): the unhardened
        # control must undefine the driver macro AND opt out of the header's
        # own enable — either mechanism alone is not proof of absence.
        "unhardened_args": ["-U_GLIBCXX_ASSERTIONS", "-D_GLIBCXX_NO_ASSERTIONS"],
        "expected_signals": {int(signal.SIGABRT)},
        "probe": _STDLIB_HARDENING_PROBE,
    }


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
    for label, inspected_tool, families in (
        ("compiler", compiler, {"clang", "gcc"}),
        ("analyzer", analyzer, {"clang"}),
    ):
        try:
            summary[f"{label}_inspection"] = native_toolchain.inspect_tool(inspected_tool)
        except ValueError as error:
            summary[f"{label}_inspection_error"] = str(error)
        else:
            if summary[f"{label}_inspection"]["family"] not in families:
                tool_health_failed = True
                summary["errors"].append(
                    f"{label} {inspected_tool} identifies as "
                    f"{summary[f'{label}_inspection']['family']!r}; expected {sorted(families)}"
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
    move_argv[len(_compiler_launcher(context))] = str(analyzer)
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
        expected = {_canonical(item["path"]) for item in manifest["sources"]}
        entries = load_entries(build_dir, expected)
        cmake = shutil.which("cmake")
        if cmake is None:
            raise CheckError("required tool is unavailable: cmake")
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
            str(cmake),
            "-S",
            str(native_root),
            "-B",
            str(case_build),
            # The child configure re-binds the interpreter this context
            # recorded — never a PATH/python-env rescue resolution.
            f"-DPython_EXECUTABLE={context['python_executable']}",
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

    context_platform = (
        context["platform"]["system"]
        if context.get("schema_version") == 2
        else "Darwin"
    )
    sdk_controls: dict[str, str] = {}
    sdk_details: dict[str, Any] = {}
    common_controls: dict[str, str] = {}
    common_details: dict[str, Any] = {}
    hardening_evidence: list[dict[str, Any]] = []

    def _record_explicit_paths(child_context: Mapping[str, Any]) -> None:
        """Common control: a child configure must re-bind the exact compiler
        and interpreter this context recorded — never a PATH/python-env
        rescue. Recorded from the first successfully configured child."""
        if "explicit_paths" in common_controls:
            return
        paths_ok = (
            Path(child_context["compiler"]["path"]).resolve() == compiler.resolve()
            and child_context["python_executable"] == context["python_executable"]
            and (child_context.get("platform") or {}).get("system") == context_platform
        )
        common_controls["explicit_paths"] = "passed" if paths_ok else "failed"
        common_details["explicit_paths"] = {
            "compiler": child_context["compiler"]["path"],
            "python": child_context["python_executable"],
            "platform": (child_context.get("platform") or {}).get("system"),
        }

    def _selection_case(name: str) -> Path:
        """A throwaway copy of this build's real selection files for a
        corruption control — context, manifest and compile database are
        copied verbatim and mutated in exactly one place."""
        case_dir = work_root / f"selection-{name}"
        case_dir.mkdir(parents=True)
        context_file = build_dir / "native_check_context.json"
        if context_file.is_file():
            shutil.copy2(context_file, case_dir / "native_check_context.json")
        else:
            nested = sorted((build_dir / "native_check_context").glob("*.json"))
            destination = case_dir / "native_check_context"
            destination.mkdir()
            shutil.copy2(nested[0], destination / nested[0].name)
        shutil.copy2(build_dir / "native_sources.json", case_dir / "native_sources.json")
        shutil.copy2(build_dir / "compile_commands.json", case_dir / "compile_commands.json")
        return case_dir

    def _case_context_file(case_dir: Path) -> Path:
        direct = case_dir / "native_check_context.json"
        if direct.is_file():
            return direct
        return next((case_dir / "native_check_context").glob("*.json"))

    def _rewrite_json(path: Path, mutate: Callable[[Any], None]) -> None:
        data = json.loads(path.read_text())
        mutate(data)
        path.write_text(json.dumps(data))

    def _expect_rejected(
        name: str,
        mutate: Callable[[Path, Path, Path], None],
        marker: str,
    ) -> None:
        """Corrupt one copied selection artifact; load_entries must refuse
        it with the expected diagnostic — accepting it, or failing on an
        unrelated check, both fail the control."""
        case_dir = _selection_case(name)
        context_path = _case_context_file(case_dir)
        manifest_path = case_dir / "native_sources.json"
        database_path = case_dir / "compile_commands.json"
        mutate(context_path, manifest_path, database_path)
        try:
            load_entries(case_dir, expected)
        except CheckError as error:
            common_controls[name] = "rejected" if marker in str(error) else "failed"
            common_details[name] = {"expected_marker": marker, "error": str(error)}
            return
        common_controls[name] = "failed"
        common_details[name] = {
            "expected_marker": marker,
            "error": "the corrupted selection was accepted",
        }

    def _run_common_controls() -> None:
        """Selection-integrity controls, identical on both platforms: each
        corrupts a copy of this build's real selection artifacts."""
        _expect_rejected(
            "stale_dependency",
            lambda context_path, _m, _d: _rewrite_json(
                context_path,
                lambda data: data["include_paths"].append(
                    str(work_root / "no-longer-present-include")
                ),
            ),
            "include_paths",
        )
        _expect_rejected(
            "wrong_artifact",
            lambda context_path, _m, _d: _rewrite_json(
                context_path,
                lambda data: data["required_files"].append(str(work_root)),
            ),
            "not a file",
        )
        _expect_rejected(
            "source_list_change",
            lambda _c, manifest_path, _d: _rewrite_json(
                manifest_path, lambda data: data["sources"].pop()
            ),
            "differs",
        )
        _expect_rejected(
            "duplicate_tu",
            lambda _c, _m, database_path: _rewrite_json(
                database_path, lambda data: data.append(data[0])
            ),
            "duplicate",
        )
        drift_targets = list(context["target_contexts"])
        if drift_targets:
            _expect_rejected(
                "target_context_drift",
                lambda context_path, _m, _d: _rewrite_json(
                    context_path,
                    lambda data: data["target_contexts"][drift_targets[0]][
                        "compile_options"
                    ].append("-ft14-context-drift"),
                ),
                "omits target",
            )
        else:
            common_controls["target_context_drift"] = "not-applicable"
            common_details["target_context_drift"] = {
                "error": "context records no target contexts to drift"
            }

    def _run_darwin_controls() -> None:
        """Apple SDK selection controls (explicit/default/missing/fallback)."""

        sdk_request = context["sdk_requested"]

        explicit_record, explicit_build = configure_case(
            "sdk-explicit",
            options=(f"-DCMAKE_OSX_SYSROOT={sdk_request}",),
        )
        try:
            explicit_context = configured_context(explicit_build)
            _record_explicit_paths(explicit_context)
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

    def _run_libcpp_hardening_darwin() -> None:
        """Darwin libc++ controls: EXTENSIVE-vs-FAST assertion, object-size
        and timing evidence on real binaries (retained unchanged)."""
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


    def _run_linux_controls() -> None:
        """Linux toolchain controls: no SDK, no Apple tools, sysroot policy."""
        # A child configure re-validates the whole chain: load_context proves
        # the context records no Apple SDK, and load_entries proves no compile
        # entry smuggles an -isysroot the context did not record.
        no_sdk_record, no_sdk_build = configure_case("linux-no-sdk")
        try:
            child = configured_context(no_sdk_build)
            _record_explicit_paths(child)
            no_sdk_ok = (
                no_sdk_record["returncode"] == 0
                and child.get("sdk") is None
                and child.get("sdk_requested") is None
                and child.get("sdk_source") == "not-applicable"
                and child.get("sysroot") is None
                and child.get("platform", {}).get("system") == "Linux"
            )
            sdk_controls["no_sdk"] = "passed" if no_sdk_ok else "failed"
            sdk_details["no_sdk"] = {
                "sdk": child.get("sdk"),
                "sysroot": child.get("sysroot"),
                "compiler": child["compiler"]["path"],
            }
        except (CheckError, KeyError, TypeError) as error:
            sdk_controls["no_sdk"] = "failed"
            sdk_details["no_sdk"] = {"error": str(error)}

        # A nonexistent CMAKE_SYSROOT must fail configuration. Rejection can
        # surface at either layer — BikeCheckContext's explicit IS_DIRECTORY
        # refusal, or the compiler-policy compile check once the missing
        # sysroot has broken header discovery — so the marker is the
        # configure-level failure itself, contrasted against the adjacent
        # linux-no-sdk configure which differs only in this option.
        missing_sysroot = work_root / "missing-sysroot"
        bad_record, _bad_build = configure_case(
            "linux-bad-sysroot",
            options=(f"-DCMAKE_SYSROOT={missing_sysroot}",),
        )
        bad_output = bad_record["stdout"] + bad_record["stderr"]
        bad_ok = bad_record["returncode"] != 0 and (
            "CMake Error" in bad_output or "Configuring incomplete" in bad_output
        )
        sdk_controls["bad_sysroot"] = "rejected" if bad_ok else "failed"
        sdk_details["bad_sysroot"] = {
            "requested": str(missing_sysroot),
            "returncode": bad_record["returncode"],
            "named_in_output": str(missing_sysroot) in bad_output,
        }

        # CMAKE_OSX_SYSROOT is a macOS option; a Linux configure must refuse
        # it outright (BikePlatform.cmake FATAL_ERROR) rather than adapt it.
        rejected_record, _rejected_build = configure_case(
            "linux-reject-osx-sysroot",
            options=("-DCMAKE_OSX_SYSROOT=/System/Library/Frameworks",),
        )
        rejected_output = rejected_record["stdout"] + rejected_record["stderr"]
        rejected_ok = (
            rejected_record["returncode"] != 0 and "CMAKE_OSX_SYSROOT" in rejected_output
        )
        sdk_controls["rejected_osx_sysroot"] = "rejected" if rejected_ok else "failed"
        sdk_details["rejected_osx_sysroot"] = {
            "returncode": rejected_record["returncode"],
            "matched_option": "CMAKE_OSX_SYSROOT" in rejected_output,
        }

        # The Linux configure must never invoke Apple-only tools; stubs record
        # every invocation so a real xcrun/sw_vers call cannot pass silently.
        fake_bin = work_root / "fake-apple-tool-bin"
        fake_bin.mkdir(parents=True, exist_ok=True)
        marker = work_root / "apple-tool-invocations.txt"
        for tool_name in ("xcrun", "sw_vers", "otool", "install_name_tool"):
            stub = fake_bin / tool_name
            stub.write_text(
                "#!/bin/sh\n"
                'printf "%s %s\\n" "$0" "$*" >> "$BIKE_T14_APPLE_TOOL_LOG"\n'
                "exit 1\n"
            )
            stub.chmod(0o755)
        apple_environment = os.environ.copy()
        apple_environment["BIKE_T14_APPLE_TOOL_LOG"] = str(marker)
        apple_environment["PATH"] = (
            str(fake_bin) + os.pathsep + apple_environment.get("PATH", "")
        )
        apple_record, _apple_build = configure_case(
            "linux-no-apple-tools",
            environment=apple_environment,
        )
        no_apple_ok = apple_record["returncode"] == 0 and not marker.exists()
        sdk_controls["no_apple_tool"] = "passed" if no_apple_ok else "failed"
        sdk_details["no_apple_tool"] = {
            "configure_returncode": apple_record["returncode"],
            "invocations": marker.read_text().splitlines() if marker.exists() else [],
        }

        # A native Linux sysroot must be recorded and preserved verbatim into
        # every compile command — never silently dropped, never an Apple SDK.
        preserved_record, preserved_build = configure_case(
            "linux-sysroot-preserved",
            options=("-DCMAKE_SYSROOT=/",),
        )
        try:
            preserved_context = configured_context(preserved_build)
            preserved_entries = load_entries(preserved_build, expected)
            sysroot_args_seen = {
                argument
                for preserved_entry in preserved_entries
                for argument in preserved_entry["arguments"]
                if argument.startswith("--sysroot") or argument.startswith("-isysroot")
            }
            preserved_ok = (
                preserved_record["returncode"] == 0
                and preserved_context.get("sysroot") == "/"
                and sysroot_args_seen == {"--sysroot=/"}
            )
            sdk_controls["sysroot_preservation"] = (
                "passed" if preserved_ok else "failed"
            )
            sdk_details["sysroot_preservation"] = {
                "sysroot": preserved_context.get("sysroot"),
                "sysroot_args": sorted(sysroot_args_seen),
            }
        except (CheckError, KeyError, TypeError) as error:
            sdk_controls["sysroot_preservation"] = "failed"
            sdk_details["sysroot_preservation"] = {"error": str(error)}

    def _run_stdlib_hardening() -> None:
        """The configured standard library's production hardening control.

        Darwin libc++ keeps the existing EXTENSIVE-vs-FAST assertion,
        object-size and timing evidence; every other platform compiles a
        hardened and an unhardened probe under the stdlib's configured mode
        and proves the out-of-bounds assertion terminates nonzero. The
        unhardened run exercises undefined behavior — its outcome is
        recorded as evidence, never asserted as a guarantee.
        """
        if context_platform == "Darwin" and context["stdlib"]["name"] == "libc++":
            _run_libcpp_hardening_darwin()
            return
        spec = _stdlib_probe_spec(context)
        stdlib_name = spec["stdlib"]
        stdlib_mode = spec["mode"]
        probe_source = work_root / "stdlib_hardening_probe.cpp"
        probe_source.write_text(spec["probe"])
        sysroot_args: list[str] = []
        if context.get("sysroot"):
            sysroot_args = (
                ["-isysroot", context["sysroot"]]
                if context_platform == "Darwin"
                else ["--sysroot", context["sysroot"]]
            )
        arch_args: list[str] = []
        configured_args = entries[0]["arguments"]
        position = 0
        while position < len(configured_args):
            if configured_args[position] == "-arch" and position + 1 < len(configured_args):
                arch_args.extend(["-arch", configured_args[position + 1]])
                position += 2
            else:
                position += 1

        hardened_binary = work_root / "stdlib-hardening-probe-hardened"
        compile_record = _run_process(
            [
                str(compiler),
                "-std=c++23",
                *arch_args,
                *sysroot_args,
                *spec["stdlib_args"],
                "-DBIKE_HARDENED=1",
                spec["define"],
                "-O0",
                str(probe_source),
                "-o",
                str(hardened_binary),
            ],
            cwd=repository,
            log_directory=log_directory,
            name="stdlib-hardening-compile",
            timeout_seconds=180,
        )
        summary["records"].append(compile_record)
        outcome = "compile-failed" if compile_record["process_health"] != "ok" else None
        hardening_failed = outcome is not None
        assertion_active = False
        assertion_returncode: int | None = None
        if not hardening_failed:
            assertion_record = _run_process(
                [str(hardened_binary), "invalid-element-control"],
                cwd=repository,
                log_directory=log_directory,
                name="stdlib-hardening-assertion",
                disable_core_dumps=True,
                timeout_seconds=30,
            )
            assertion_returncode = assertion_record["returncode"]
            signal_number = (
                -assertion_returncode
                if isinstance(assertion_returncode, int) and assertion_returncode < 0
                else None
            )
            if assertion_record["launch_error"] is not None or assertion_record["timed_out"]:
                outcome = "tool-failed"
            else:
                assertion_active = signal_number in spec["expected_signals"]
                outcome = "asserted" if assertion_active else "silent"
            summary["records"].append(
                {
                    **assertion_record,
                    "control": "stdlib-hardening-assertion",
                    "assertion_active": assertion_active,
                    "assertion_category": f"{stdlib_name} std::vector element access",
                }
            )
            benign_record = _run_process(
                [str(hardened_binary)],
                cwd=repository,
                log_directory=log_directory,
                name="stdlib-hardening-benign",
                timeout_seconds=30,
            )
            summary["records"].append(benign_record)
            if benign_record["returncode"] != 0:
                hardening_failed = True
            if outcome != "asserted":
                hardening_failed = True

        # The unhardened control proves the probe itself compiles without a
        # hardening define (its own #error guards report a leaked define as a
        # compile failure, distinct from a tool failure); the unhardened run
        # reads out of bounds — undefined behavior whose outcome is recorded
        # as evidence only.
        unhardened_binary = work_root / "stdlib-hardening-probe-unhardened"
        unhardened_compile = _run_process(
            [
                str(compiler),
                "-std=c++23",
                *arch_args,
                *sysroot_args,
                *spec["stdlib_args"],
                *spec["unhardened_args"],
                "-O0",
                str(probe_source),
                "-o",
                str(unhardened_binary),
            ],
            cwd=repository,
            log_directory=log_directory,
            name="stdlib-unhardened-compile",
            timeout_seconds=180,
        )
        summary["records"].append(unhardened_compile)
        unhardened_evidence: dict[str, Any] = {
            "compiled": unhardened_compile["process_health"] == "ok",
            "returncode": None,
            "note": (
                "out-of-bounds access without assertions is undefined "
                "behavior — recorded as evidence, not a guarantee"
            ),
        }
        if unhardened_evidence["compiled"]:
            unhardened_run = _run_process(
                [str(unhardened_binary), "invalid-element-control"],
                cwd=repository,
                log_directory=log_directory,
                name="stdlib-unhardened-run",
                disable_core_dumps=True,
                timeout_seconds=30,
            )
            summary["records"].append(
                {**unhardened_run, "control": "stdlib-unhardened-run"}
            )
            unhardened_evidence["returncode"] = unhardened_run["returncode"]

        hardening_evidence.append(
            {
                "build_type": "control",
                "stdlib": stdlib_name,
                "mode": stdlib_mode,
                "outcome": outcome,
                "assertion_active": {stdlib_mode: assertion_active},
                "expected_signals": sorted(spec["expected_signals"]),
                "assertion_returncode": assertion_returncode,
                "unhardened": unhardened_evidence,
            }
        )
        sdk_controls["stdlib_hardening"] = (
            "failed" if hardening_failed else "passed"
        )

    _run_common_controls()
    if context_platform == "Darwin":
        _run_darwin_controls()
    elif context_platform == "Linux":
        _run_linux_controls()
    else:
        raise CheckError(f"unsupported context platform: {context_platform}")
    _run_stdlib_hardening()

    all_controls = {**common_controls, **sdk_controls}
    summary["common_controls"] = common_controls
    summary["sdk_controls"] = sdk_controls
    summary["sdk_control_details"] = {**common_details, **sdk_details}
    summary["hardening_evidence"] = hardening_evidence
    summary["tool_health"] = "failed" if any(
        record.get("launch_error") or record.get("timed_out")
        for record in cmake_records
    ) else "ok"
    summary["status"] = (
        "passed"
        if all(value in {"passed", "rejected", "not-applicable"} for value in all_controls.values())
        and summary["tool_health"] == "ok"
        else "failed"
    )
    summary["records"].extend(cmake_records)
    summary["log_directory"] = str(log_directory)
    _write_summary(build_dir, kind, summary)
    print(f"native context common controls: {common_controls}")
    print(f"native context SDK controls: {sdk_controls}")
    print(f"native context tool health: {summary['tool_health']}")
    print(f"native context summary: {build_dir / 'native_check_summaries' / 'context.json'}")
    if summary["status"] != "passed":
        print("native context controls failed; see retained control logs above", file=sys.stderr)
        for name, status in all_controls.items():
            if status not in {"passed", "rejected", "not-applicable"}:
                print(f"  {name}: {status}", file=sys.stderr)
        return 1
    return 0


def _report_alpha_findings(record: Mapping[str, Any], source: Path) -> None:
    diagnostic_lines = record.get("unsuppressed_diagnostics", record["diagnostics"])
    lines = [line for line in diagnostic_lines if _diagnostic_severity(line) == "warning"]
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
    # Tool identity/capability evidence: the version banner must name the
    # expected toolchain family for this kind (wrong-family tools are a
    # selection error); uninspectable tools are recorded but remain gated by
    # the sweep's own diagnostics.
    tool_health_failed = _inspect_sweep_tool(kind, tool, summary) or tool_health_failed

    frontend_flags: Sequence[str] = _GCC_WARNING_FLAGS
    if kind in {"frontends", "odr"}:
        try:
            frontend_flags, unavailable_flags = _probe_frontend_warning_flags(
                tool,
                work_directory=build_dir / "native_check_work" / kind / run_id,
                log_directory=log_directory,
            )
        except CheckError as error:
            summary["errors"].append(str(error))
            summary["records"] = records
            summary["tool_health"] = "failed"
            summary["log_directory"] = str(log_directory)
            _write_summary(build_dir, kind, summary)
            print(f"native {kind} failed: {error}", file=sys.stderr)
            return 1
        summary["warning_flags"] = {
            "applied": list(frontend_flags),
            "unavailable": unavailable_flags,
        }
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
                frontend_flags=frontend_flags,
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
            has_warning = any(_diagnostic_severity(line) == "warning" for line in unsuppressed)
            has_error = any(_diagnostic_severity(line) == "error" for line in unsuppressed)
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
