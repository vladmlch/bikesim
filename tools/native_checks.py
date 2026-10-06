#!/usr/bin/env python3
"""Validate CMake's native TU manifest and run fail-closed analysis sweeps."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


class CheckError(ValueError):
    """A malformed build context or incomplete translation-unit selection."""


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
_TARGET_FROM_OUTPUT = re.compile(r"(?:^|[/\\])CMakeFiles[/\\]([^/\\]+)\.dir(?:[/\\]|$)")


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


def _require_existing_path(value: Any, label: str, *, executable: bool = False) -> Path:
    if not isinstance(value, str) or not value:
        raise CheckError(f"{label} is missing or is not a path")
    path = Path(value).expanduser().resolve()
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
        compiler.get("path"), "configured C++ compiler", executable=True
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
            extra.extend(["-Xanalyzer", "-analyzer-checker=optin", "-Xanalyzer", "-analyzer-disable-checker=cplusplus.Move"])
        else:
            extra.extend([
                "-Xanalyzer", "-analyzer-checker=alpha",
                "-Xanalyzer", "-analyzer-config",
                "-Xanalyzer", "aggressive-binary-operation-simplification=true",
                "-Xanalyzer", "-analyzer-disable-checker=cplusplus.Move",
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


def _report_alpha_findings(record: Mapping[str, Any], source: Path) -> None:
    lines = [line for line in record["diagnostics"] if re.search(r"\bwarning:", line, re.IGNORECASE)]
    if lines:
        print(f"== alpha findings (report-only): {source}", file=sys.stderr)
        print("\n".join(lines), file=sys.stderr)


def run_sweep(kind: str, build: Path) -> int:
    """Run one native sweep and retain its selection, health, findings, and logs."""
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
            records.append(record)
            if record["process_health"] != "ok":
                tool_health_failed = True
            has_warning = any(re.search(r"\bwarning:", line, re.IGNORECASE) for line in record["diagnostics"])
            has_error = any(re.search(r"\b(?:error|fatal error):", line, re.IGNORECASE) for line in record["diagnostics"])
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
    print(f"native {kind} summary: {build_dir / 'native_check_summaries' / f'{kind}.json'}")
    if summary["status"] != "passed":
        for record in records:
            if record.get("process_health") != "ok" or record.get("diagnostics"):
                print(
                    f"  {record.get('pass')} {record.get('source')}: rc={record.get('returncode')} "
                    f"stdout={record.get('stdout_log')} stderr={record.get('stderr_log')}",
                    file=sys.stderr,
                )
        return 1
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", required=True, choices=("tidy", "analyzer", "cppcheck", "frontends", "odr"))
    parser.add_argument("--build", required=True, type=Path)
    arguments = parser.parse_args(argv)
    return run_sweep(arguments.kind, arguments.build)


if __name__ == "__main__":
    raise SystemExit(main())
