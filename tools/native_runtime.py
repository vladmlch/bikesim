#!/usr/bin/env python3
"""Select and verify native sanitizer runtimes by toolchain (T15).

The native sanitizer lanes (``-DNATIVE_SANITIZE=ON`` / ``-DNATIVE_RTSAN=ON``)
produce binaries whose instrumented code needs the *compiler's own* runtime
mapped before it runs: upstream Clang's ``-shared-libasan`` objects NEED a
``libclang_rt.asan-<machine>.so`` image that is not on the default loader
path, and AppleClang extensions load
``libclang_rt.asan_osx_dynamic.dylib`` through ``DYLD_INSERT_LIBRARIES``.

The runtime is selected by *recorded* toolchain identity, never by driver
basename:

* ``context["compiler"]["id"]`` (CMAKE_CXX_COMPILER_ID) selects the runtime
  *name* — ``libasan.so`` for GNU, the target-suffixed
  ``libclang_rt.<san>-<machine>.so`` for upstream Clang on Linux, and the
  ``*_osx_dynamic.dylib`` spellings on Darwin;
* ``context["compiler"]["path"]`` answers ``-print-file-name=<name>`` — an
  unresolved echo or a missing file is a failed resolution, not a guess;
* when the build context already records ``capabilities.<san>.runtime``
  (the configure-time-resolved path) that path is authoritative — but it is
  revalidated before use;
* the runtime must be a real ELF/Mach-O image whose machine matches the
  recorded target and whose name belongs to the lane it serves — a Linux
  lane never preloads an Apple ``.dylib``, and an ``rtsan`` record never
  injects a ``libasan`` image.

Environment contract for the enabled lane:

* the platform preload variable (``DYLD_INSERT_LIBRARIES`` on Darwin,
  ``LD_PRELOAD`` on Linux) is *prepended* with the selected runtime — an
  existing value is preserved after it;
* ``NATIVE_TEST_SANITIZER_RUNTIME`` records the selected runtime path so
  in-process checks (``tests/reference/native_loader.py``) can verify the
  image is actually mapped;
* ASan lanes set ``PYTHONMALLOC=malloc`` and append the project options
  ``detect_leaks=0:detect_stack_use_after_return=1`` after any caller
  ``ASAN_OPTIONS`` (last write wins, so the project contract is enforced);
* ASan and RTSan enabled together — or a requested-but-disabled record,
  which means a stale context — fail instead of running uninstrumented.
* Linux TSan uses the compiler-linked runtime qualified during configure;
  it never preloads Clang's archive or a second sanitizer image. Its path
  is recorded in ``NATIVE_TEST_LINKED_SANITIZER_RUNTIME`` and its options
  enforce ``halt_on_error=1:exitcode=66``.

CLI: ``python tools/native_runtime.py run --build PATH -- COMMAND [ARGS]``
loads the validated schema-2 context, checks it against the build's
CMakeCache selection, computes the environment and runs the argv array
with the child's exit status propagated. The helper prints only a
sanitizer/compiler evidence line — never the environment.
"""
from __future__ import annotations

import argparse
import os
import platform
import re
import struct
import subprocess
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

try:  # imported as tools.native_runtime by the test suite
    from tools import native_checks
except ImportError:  # invoked as `python tools/native_runtime.py`
    import native_checks


class NativeRuntimeError(ValueError):
    """Sanitizer runtime selection or verification failure."""


_REQUIRED_ASAN_OPTIONS = "detect_leaks=0:detect_stack_use_after_return=1"

_CLANG_FAMILY_IDS = {"Clang", "AppleClang"}
_GNU_FAMILY_IDS = {"GNU"}
_SANITIZER_LANES = ("asan", "rtsan", "tsan")

# Normalized machine spellings shared with the CMake policy's normalized
# BIKE_POLICY_MACHINE: arm64 and aarch64 are the same target.
_MACHINE_ALIASES = {
    "arm64": "aarch64",
    "aarch64": "aarch64",
    "x86_64": "x86_64",
    "amd64": "x86_64",
    "x64": "x86_64",
}

# ELF e_machine / Mach-O cputype values for the supported targets. Unknown
# values surface as "not a parseable image" / "wrong architecture" failures
# rather than a guessed mapping.
_ELF_MACHINE_NAMES = {
    3: "x86",
    40: "arm",
    62: "x86_64",
    183: "aarch64",
    243: "riscv64",
}
_MACHO_CPU_NAMES = {
    0x00000007: "x86",
    0x0000000C: "arm",
    0x01000007: "x86_64",
    0x0100000C: "aarch64",
}
_MACHO_THIN_LE = (b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe")  # 64/32-bit LE
_MACHO_THIN_BE = (b"\xfe\xed\xfa\xcf", b"\xfe\xed\xfa\xce")  # 64/32-bit BE
_FAT_MAGIC = b"\xca\xfe\xba\xbe"      # 20-byte fat_arch entries
_FAT_MAGIC_64 = b"\xca\xfe\xba\xbf"   # 32-byte fat_arch_64 entries


def _normalize_machine(value: Any) -> str:
    lowered = str(value).lower()
    return _MACHINE_ALIASES.get(lowered, lowered)


def _image_machines(path: Path) -> set[str]:
    """The normalized machine names a binary image targets (may be empty).

    Parses the file's own header — ELF e_machine, thin Mach-O cputype, or
    the cputype list of a fat/universal image — so a recorded or resolved
    runtime can be checked against the build's target architecture without
    trusting its filename.
    """
    try:
        with path.open("rb") as handle:
            head = handle.read(8192)
    except OSError:
        return set()
    if len(head) < 20:
        return set()
    if head[:4] == b"\x7fELF":
        endian = "<" if head[5] == 1 else ">"
        (e_machine,) = struct.unpack(f"{endian}H", head[18:20])
        machine = _ELF_MACHINE_NAMES.get(e_machine)
        return {machine} if machine is not None else set()
    if head[:4] in _MACHO_THIN_LE:
        (cpu,) = struct.unpack("<I", head[4:8])
        machine = _MACHO_CPU_NAMES.get(cpu)
        return {machine} if machine is not None else set()
    if head[:4] in _MACHO_THIN_BE:
        (cpu,) = struct.unpack(">I", head[4:8])
        machine = _MACHO_CPU_NAMES.get(cpu)
        return {machine} if machine is not None else set()
    if head[:4] in (_FAT_MAGIC, _FAT_MAGIC_64):
        entry_size = 32 if head[:4] == _FAT_MAGIC_64 else 20
        (count,) = struct.unpack(">I", head[4:8])
        machines: set[str] = set()
        for index in range(min(count, 64)):
            base = 8 + index * entry_size
            if base + 4 > len(head):
                break
            (cpu,) = struct.unpack(">I", head[base:base + 4])
            machine = _MACHO_CPU_NAMES.get(cpu)
            if machine is not None:
                machines.add(machine)
        return machines
    return set()


def _runtime_name_candidates(
    compiler_id: str, system: str, machine: str, sanitizer: str
) -> list[str]:
    """The runtime filename(s) the recorded compiler identity selects.

    Only the compiler *id* and the recorded target decide — a GNU compiler
    installed as ``/opt/clang-clone/c++`` still asks for ``libasan.so``,
    and an upstream Clang named ``g++`` still asks for its ``libclang_rt``
    runtime.
    """
    if compiler_id in _CLANG_FAMILY_IDS:
        if system == "Darwin":
            return [f"libclang_rt.{sanitizer}_osx_dynamic.dylib"]
        if system == "Linux":
            if machine not in {"aarch64", "x86_64"}:
                raise NativeRuntimeError(
                    f"no {sanitizer} clang runtime naming for target machine "
                    f"{machine!r}"
                )
            return [f"libclang_rt.{sanitizer}-{machine}.so"]
        raise NativeRuntimeError(
            f"unsupported sanitizer platform system: {system!r}"
        )
    if compiler_id in _GNU_FAMILY_IDS:
        if sanitizer == "asan":
            if system == "Darwin":
                # GCC on Darwin links libasan through the system naming.
                return ["libasan.dylib", "libasan.so"]
            return ["libasan.so"]
        # GCC has no RTSan runtime; the flag probe rejects the lane before
        # this point on a coherent toolchain — an enabled record reaching
        # here is inconsistent, so fail rather than fabricate a filename.
        raise NativeRuntimeError(
            f"{compiler_id} provides no {sanitizer} runtime"
        )
    raise NativeRuntimeError(
        f"unsupported compiler id for sanitizer runtime selection: "
        f"{compiler_id!r}"
    )


def _driver_file_name(compiler: Path, name: str) -> tuple[str | None, str]:
    """Ask the configured driver for ``name``; reject the unresolved echo."""
    try:
        completed = subprocess.run(
            [str(compiler), f"-print-file-name={name}"],
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return None, f"{name}: driver query could not run ({error})"
    lines = completed.stdout.splitlines()
    answer = lines[0].strip() if lines else ""
    if completed.returncode != 0:
        return None, f"{name}: driver exited {completed.returncode}"
    if not answer:
        return None, f"{name}: driver printed no answer"
    if answer == name:
        # GCC/Clang print the bare filename back when nothing resolves —
        # treating it as a path would inject a nonexistent runtime.
        return None, f"{name}: unresolved (driver echoed the bare filename)"
    candidate = Path(answer)
    if not candidate.is_absolute() or candidate.is_dir() or not candidate.is_file():
        return None, f"{name}: driver answer {answer!r} is not a file"
    return str(candidate), f"{name}: {answer}"


def _check_runtime_platform(runtime: Path, system: str) -> None:
    name = runtime.name
    if system == "Darwin":
        if not name.endswith(".dylib"):
            raise NativeRuntimeError(
                f"the Darwin sanitizer runtime is not an Apple dylib: {runtime}"
            )
    elif system == "Linux":
        if name.endswith(".dylib"):
            raise NativeRuntimeError(
                "a Linux sanitizer lane must never preload an Apple dylib: "
                f"{runtime}"
            )
        if ".so" not in name:
            raise NativeRuntimeError(
                f"the Linux sanitizer runtime is not a shared object: {runtime}"
            )
    else:
        raise NativeRuntimeError(
            f"unsupported sanitizer platform system: {system!r}"
        )


def _check_runtime_arch(runtime: Path, machine: str) -> None:
    machines = _image_machines(runtime)
    if not machines:
        raise NativeRuntimeError(
            "the selected sanitizer runtime is not a parseable ELF/Mach-O "
            f"image: {runtime}"
        )
    if machine not in machines:
        raise NativeRuntimeError(
            f"the selected sanitizer runtime's architecture "
            f"{sorted(machines)} does not match the recorded target "
            f"{machine}: {runtime}"
        )


def _check_runtime_lane(runtime: Path, sanitizer: str) -> None:
    """The runtime image must name the lane it claims to serve.

    Every runtime spelling this runner selects carries the sanitizer name —
    ``libasan.so``/``libclang_rt.asan-*`` for ASan and
    ``libclang_rt.rtsan-*`` for RTSan — and the two spellings never overlap
    (``rtsan`` does not contain the ``asan`` substring). A record that names
    the other lane's image is inconsistent: the preloaded library could not
    carry the lane's instrumentation.
    """
    if sanitizer == "rtsan" and "rtsan" not in runtime.name:
        raise NativeRuntimeError(
            f"capability rtsan recorded a runtime that is not a realtime "
            f"sanitizer image: {runtime}"
        )
    if sanitizer == "asan" and "asan" not in runtime.name:
        raise NativeRuntimeError(
            f"capability asan recorded a runtime that is not an address "
            f"sanitizer image: {runtime}"
        )


def resolve_sanitizer_runtime(
    context: Mapping[str, Any], sanitizer: str
) -> Path:
    """Resolve the runtime through the recorded compiler driver.

    Used when the context does not record a ``capabilities.<san>.runtime``
    path (a pre-T15 context, or a capability probed before the runtime
    record existed). Every answer is verified: real file, right platform
    object format, right architecture.
    """
    compiler = context.get("compiler")
    platform_record = context.get("platform")
    if not isinstance(compiler, Mapping) or not isinstance(
        platform_record, Mapping
    ):
        raise NativeRuntimeError(
            "context lacks the compiler/platform records needed to resolve "
            "a sanitizer runtime"
        )
    compiler_value = compiler.get("path")
    compiler_id = compiler.get("id")
    if not isinstance(compiler_value, str) or not compiler_value:
        raise NativeRuntimeError("context records no compiler path")
    if not isinstance(compiler_id, str) or not compiler_id:
        raise NativeRuntimeError("context records no compiler id")
    compiler_path = Path(compiler_value)
    if not compiler_path.is_file() or not os.access(compiler_path, os.X_OK):
        raise NativeRuntimeError(
            f"recorded compiler is not executable: {compiler_path}"
        )
    system = platform_record.get("system")
    machine = _normalize_machine(platform_record.get("machine"))
    names = _runtime_name_candidates(compiler_id, system, machine, sanitizer)
    attempts: list[str] = []
    for name in names:
        resolved, detail = _driver_file_name(compiler_path, name)
        attempts.append(detail)
        if resolved is None:
            continue
        candidate = Path(resolved)
        _check_runtime_platform(candidate, system)
        _check_runtime_arch(candidate, machine)
        return candidate
    raise NativeRuntimeError(
        f"{sanitizer} runtime resolution failed for {compiler_id} compiler "
        f"{compiler_path} on {system}/{machine}: " + "; ".join(attempts)
    )


def _select_runtime(
    context: Mapping[str, Any],
    record: Mapping[str, Any],
    sanitizer: str,
    system: str,
    machine: str,
) -> Path:
    recorded = record.get("runtime")
    if recorded is not None:
        if not isinstance(recorded, str) or not recorded:
            raise NativeRuntimeError(
                f"capability {sanitizer}.runtime must be an absolute path "
                "or null"
            )
        runtime = Path(recorded)
        if not runtime.is_absolute():
            raise NativeRuntimeError(
                f"capability {sanitizer}.runtime is not an absolute path: "
                f"{recorded!r}"
            )
        if not runtime.is_file():
            raise NativeRuntimeError(
                f"capability {sanitizer} recorded a missing sanitizer "
                f"runtime: {runtime}"
            )
        _check_runtime_platform(runtime, system)
        _check_runtime_arch(runtime, machine)
        _check_runtime_lane(runtime, sanitizer)
        return runtime
    return resolve_sanitizer_runtime(context, sanitizer)


def runtime_environment(
    context: Mapping[str, Any], environ: Mapping[str, str]
) -> dict[str, str]:
    """The subprocess environment for the build's selected sanitizer lane.

    Starts from a copy of ``environ``, injects the compiler-matched runtime
    into the platform preload variable and sets the sanitizer bookkeeping
    variables. Unrelated variables pass through untouched; an existing
    preload value is preserved after the selected runtime. Raises
    ``NativeRuntimeError`` for every inconsistent or unverifiable state —
    this helper never returns an environment it cannot justify.
    """
    capabilities = context.get("capabilities")
    if capabilities is None:
        # Schema-1 contexts predate the capability record — a build that
        # never recorded sanitizer lanes runs uninstrumented.
        return dict(environ)
    if not isinstance(capabilities, Mapping):
        raise NativeRuntimeError(
            "context capabilities record is not an object"
        )
    lanes: dict[str, Mapping[str, Any]] = {}
    for name in _SANITIZER_LANES:
        record = capabilities.get(name)
        if record is None:
            continue
        if not isinstance(record, Mapping):
            raise NativeRuntimeError(
                f"capability {name} record is not an object"
            )
        requested = record.get("requested") is True
        enabled = record.get("enabled") is True
        if enabled and not requested:
            raise NativeRuntimeError(
                f"capability {name} is enabled without a requested lane — "
                "the recorded context is inconsistent; reconfigure the build"
            )
        if requested and not enabled:
            raise NativeRuntimeError(
                f"capability {name} was requested but is not enabled in the "
                "recorded context — the toolchain probe failed or the "
                "context is stale; reconfigure the build"
            )
        if enabled:
            lanes[name] = record
    if len(lanes) > 1:
        raise NativeRuntimeError(
            "ASan, RTSan and TSan require separate builds; multiple "
            "sanitizer lanes are not a runnable configuration"
        )
    if not lanes:
        return dict(environ)

    env = dict(environ)
    sanitizer, record = next(iter(lanes.items()))
    if record.get("supported") is not True:
        raise NativeRuntimeError(
            f"capability {sanitizer} is enabled without support evidence"
        )
    platform_record = context.get("platform")
    if not isinstance(platform_record, Mapping):
        raise NativeRuntimeError("context has no platform record")
    system = platform_record.get("system")
    machine = _normalize_machine(platform_record.get("machine"))
    if sanitizer == "tsan":
        if system != "Linux":
            raise NativeRuntimeError("TSan runtime unsupported outside Linux")
        recorded = record.get("runtime")
        if not isinstance(recorded, str) or not recorded:
            raise NativeRuntimeError("TSan requires a recorded compiler-linked runtime")
        runtime = Path(recorded)
        if not runtime.is_absolute() or not runtime.is_file():
            raise NativeRuntimeError("TSan recorded runtime is not an existing absolute file")
        compiler = context.get("compiler")
        compiler_id = compiler.get("id") if isinstance(compiler, Mapping) else None
        if compiler_id in _CLANG_FAMILY_IDS:
            # The successful compiler/link control validates the archive's
            # target. A .a file is not a preloadable shared image.
            if runtime.name != f"libclang_rt.tsan-{machine}.a":
                raise NativeRuntimeError("TSan archive does not match the recorded compiler target")
            with runtime.open("rb") as archive:
                if archive.read(8) != b"!<arch>\n":
                    raise NativeRuntimeError("TSan recorded archive has an invalid format")
        elif compiler_id in _GNU_FAMILY_IDS:
            if not re.fullmatch(r"libtsan\.so(?:\.\d+)*", runtime.name):
                raise NativeRuntimeError("TSan runtime is not the GNU linked runtime")
            _check_runtime_platform(runtime, system)
            _check_runtime_arch(runtime, machine)
        else:
            raise NativeRuntimeError("TSan requires a recorded Clang or GNU compiler")
        env["NATIVE_TEST_LINKED_SANITIZER_RUNTIME"] = str(runtime)
        existing_options = env.get("TSAN_OPTIONS", "")
        required_options = "halt_on_error=1:exitcode=66"
        env["TSAN_OPTIONS"] = (
            f"{existing_options}:{required_options}" if existing_options else required_options
        )
        return env
    runtime = _select_runtime(context, record, sanitizer, system, machine)

    preload_name = (
        "DYLD_INSERT_LIBRARIES" if system == "Darwin" else "LD_PRELOAD"
    )
    existing = env.get(preload_name)
    env[preload_name] = (
        str(runtime) if not existing else f"{runtime}{os.pathsep}{existing}"
    )
    env["NATIVE_TEST_SANITIZER_RUNTIME"] = str(runtime)
    if sanitizer == "asan":
        # Required project options go last so they win over a caller's
        # ASAN_OPTIONS without discarding that caller's other settings.
        existing_options = env.get("ASAN_OPTIONS", "")
        env["ASAN_OPTIONS"] = (
            f"{existing_options}:{_REQUIRED_ASAN_OPTIONS}"
            if existing_options
            else _REQUIRED_ASAN_OPTIONS
        )
        # ASan cannot interpose on pymalloc arenas; instrumented lanes pin
        # the system allocator so allocation checks stay real.
        env["PYTHONMALLOC"] = "malloc"
    return env


# ---------------------------------------------------------------------------
# CLI: run --build PATH -- COMMAND [ARGUMENTS]
# ---------------------------------------------------------------------------

_CACHE_SANITIZER = re.compile(r"^NATIVE_(SANITIZE|RTSAN|TSAN):[A-Z]+=(.*)$")


def _build_cache_modes(build: Path) -> dict[str, bool]:
    """The sanitizer selections the build's CMakeCache actually declares."""
    cache = build / "CMakeCache.txt"
    modes: dict[str, bool] = {}
    if not cache.is_file():
        return modes
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        match = _CACHE_SANITIZER.match(line)
        if match is not None:
            key = {"SANITIZE": "asan", "RTSAN": "rtsan", "TSAN": "tsan"}[match.group(1)]
            modes[key] = match.group(2).strip().upper() in {
                "1", "ON", "TRUE", "YES",
            }
    return modes


def _cross_check_build_cache(
    build: Path, context: Mapping[str, Any]
) -> None:
    """Reject a cache/context pair that disagree about enabled sanitizers.

    ``file(GENERATE)`` rewrites the context on every configure, so a
    disagreement means a hand-edited or half-regenerated build — a state
    the runner must refuse rather than instrument by guess.
    """
    modes = _build_cache_modes(build)
    if sum(modes.values()) > 1:
        raise NativeRuntimeError(
            "the selected build enables multiple ASan, RTSan or TSan lanes; "
            "separate build trees are required"
        )
    capabilities = context.get("capabilities")
    if not isinstance(capabilities, Mapping):
        # A context without capability records predates the sanitizer
        # lane records entirely — a build cache that still requests a
        # sanitizer cannot be served by it, so it is stale by definition.
        if any(modes.values()):
            raise NativeRuntimeError(
                "CMakeCache.txt requests a sanitizer but the recorded "
                "context has no capability records — reconfigure the build"
            )
        return
    for sanitizer, cached in modes.items():
        record = capabilities.get(sanitizer)
        enabled = (
            isinstance(record, Mapping) and record.get("enabled") is True
        )
        if cached and not enabled:
            raise NativeRuntimeError(
                f"CMakeCache.txt requests {sanitizer} but the recorded "
                "context did not enable it — reconfigure the build"
            )
        if not cached and enabled:
            raise NativeRuntimeError(
                f"the recorded context enables {sanitizer} but "
                "CMakeCache.txt does not request it — reconfigure the build"
            )


def _sanitizer_evidence(context: Mapping[str, Any], env: Mapping[str, str]) -> str:
    """The one-line evidence record the CLI prints — never the full env."""
    compiler = context.get("compiler")
    compiler_text = ""
    if isinstance(compiler, Mapping):
        compiler_text = (
            f" compiler={compiler.get('id')}:{compiler.get('path')}"
        )
    linked_runtime = env.get("NATIVE_TEST_LINKED_SANITIZER_RUNTIME")
    if linked_runtime is not None:
        return f"sanitizer=tsan runtime={linked_runtime} linkage=compiler-selected{compiler_text}"
    runtime = env.get("NATIVE_TEST_SANITIZER_RUNTIME")
    if runtime is None:
        return f"sanitizer=none{compiler_text}"
    platform_record = context.get("platform") or {}
    system = platform_record.get("system") if isinstance(
        platform_record, Mapping
    ) else None
    preload = (
        "DYLD_INSERT_LIBRARIES" if system == "Darwin" else "LD_PRELOAD"
    )
    capabilities = context.get("capabilities") or {}
    sanitizer = "asan"
    rtsan = capabilities.get("rtsan") if isinstance(
        capabilities, Mapping
    ) else None
    if isinstance(rtsan, Mapping) and rtsan.get("enabled") is True:
        sanitizer = "rtsan"
    return (
        f"sanitizer={sanitizer} runtime={runtime} preload={preload}"
        f"{compiler_text}"
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if "--" in args:
        index = args.index("--")
        front, command = args[:index], args[index + 1:]
    else:
        front, command = args, []
    parser = argparse.ArgumentParser(
        prog="native_runtime.py",
        description="run a command under a build's selected sanitizer runtime",
    )
    subcommands = parser.add_subparsers(dest="action", required=True)
    run_parser = subcommands.add_parser(
        "run",
        help="run COMMAND under the build's sanitizer runtime environment",
    )
    run_parser.add_argument(
        "--build",
        required=True,
        type=Path,
        help="configured native build directory",
    )
    try:
        namespace = parser.parse_args(front)
    except SystemExit as exit_code:
        return exit_code.code if isinstance(exit_code.code, int) else 2

    if namespace.action == "run":
        if not command:
            print(
                "native_runtime: `run` requires a command after `--`",
                file=sys.stderr,
            )
            return 2
        try:
            context = native_checks.load_context(namespace.build)
            _cross_check_build_cache(namespace.build, context)
            env = runtime_environment(context, dict(os.environ))
        except (ValueError, OSError) as error:
            print(f"native_runtime: {error}", file=sys.stderr)
            return 2
        print(
            f"native_runtime: {_sanitizer_evidence(context, env)}",
            file=sys.stderr,
        )
        try:
            completed = subprocess.run(
                list(command), env=env, check=False
            )
        except OSError as error:
            print(
                f"native_runtime: cannot execute {command[0]!r}: {error}",
                file=sys.stderr,
            )
            return 127
        return completed.returncode
    return 2


if __name__ == "__main__":
    sys.exit(main())
