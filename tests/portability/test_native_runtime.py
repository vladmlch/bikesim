"""Compiler-matched sanitizer runtime selection and controls (T15).

``tools/native_runtime.py`` owns the environment every instrumented native
consumer runs under: the runtime file the *recorded* compiler identity
selects (never inferred from the driver's filename), the platform preload
variable (``DYLD_INSERT_LIBRARIES`` on Darwin, ``LD_PRELOAD`` on Linux),
the project's ASan options and ``NATIVE_TEST_SANITIZER_RUNTIME``. This file
covers three layers:

1.  pure ``runtime_environment`` composition against recorded and
    driver-resolved runtimes — including the brief's regression where the
    compiler path basename must not select the runtime name;
2.  the ``run --build PATH -- argv`` CLI: context loading, exit-status
    propagation and non-secret output;
3.  real-toolchain controls: an instrumented control binary compiled from
    ``native/tests/sanitizer_control.cpp`` runs clean under the computed
    environment, and its ``use-after-free`` mode must die with the real
    AddressSanitizer diagnostic — never a loader error. When a configured
    ASan build is selected via ``NATIVE_TEST_BUILD_PATH``, a fresh Python
    subprocess imports the instrumented extension under the same contract.
"""
from __future__ import annotations

import json
import os
import platform
import shlex
import shutil
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from tools import native_checks
from tools import native_runtime


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROL_SOURCE = REPO_ROOT / "native" / "tests" / "sanitizer_control.cpp"
LOADER_DIR = REPO_ROOT / "tests" / "reference"
HOST_SYSTEM = platform.system()


def _normalize_machine(machine: str) -> str:
    lowered = machine.lower()
    if lowered in {"arm64", "aarch64"}:
        return "aarch64"
    if lowered in {"x86_64", "amd64", "x64"}:
        return "x86_64"
    return lowered


# ---------------------------------------------------------------------------
# Binary image stubs: minimal headers the arch verifier can parse.
# ---------------------------------------------------------------------------

_ELF_MACHINES = {"x86_64": 62, "aarch64": 183}
_MACHO_CPUS = {"x86_64": 0x01000007, "aarch64": 0x0100000C}


def _write_elf(path: Path, system: str, machine: str) -> Path:
    """A minimal ELF64 shared-object header — enough for e_machine parsing."""
    assert system != "Darwin", "Darwin runtimes are Mach-O images"
    e_machine = _ELF_MACHINES[_normalize_machine(machine)]
    ident = b"\x7fELF" + bytes([2, 1, 1, 0]) + bytes(8)  # 64-bit, LE, SysV
    body = struct.pack("<HHI", 3, e_machine, 1) + bytes(64 - 16 - 8)
    path.write_bytes(ident + body)
    return path


def _write_macho(path: Path, system: str, machine: str) -> Path:
    """A minimal 64-bit Mach-O dylib header."""
    assert system == "Darwin", "the Mach-O stub is the Darwin runtime format"
    cpu = _MACHO_CPUS[_normalize_machine(machine)]
    # MH_MAGIC_64 little-endian + cputype + cpusubtype + filetype(MH_DYLIB=6).
    path.write_bytes(
        b"\xcf\xfa\xed\xfe" + struct.pack("<III", cpu, 0, 6) + bytes(64)
    )
    return path


def _write_image(path: Path, system: str, machine: str) -> Path:
    if system == "Darwin":
        return _write_macho(path, system, machine)
    return _write_elf(path, system, machine)


# ---------------------------------------------------------------------------
# Fake compiler drivers for the driver-resolution tests.
# ---------------------------------------------------------------------------


def _fake_driver(
    directory: Path, name: str, mapping: dict[str, Path | str]
) -> Path:
    """A driver stand-in answering ``-print-file-name=<key>``.

    Every invocation's first argument is logged to ``<name>.argv.log`` so a
    test can assert *which* runtime name the resolver asked for. Unmapped
    keys echo the bare filename back, matching a real driver's unresolved
    query. The driver's basename deliberately lies about the toolchain —
    selection must come from the recorded compiler id, never from it.
    """
    driver = directory / name
    log = directory / f"{name}.argv.log"
    lines = [
        "#!/bin/sh",
        f'printf "%s\\n" "$1" >> {shlex.quote(str(log))}',
        'case "$1" in',
        "  -print-file-name=*)",
        '    key="${1#-print-file-name=}"',
        '    case "$key" in',
        *(
            f"      {key}) printf '%s\\n' {shlex.quote(str(value))} ;;"
            for key, value in mapping.items()
        ),
        '      *) printf "%s\\n" "$key" ;;',
        "    esac",
        "    ;;",
        "esac",
        "exit 0",
    ]
    driver.write_text("\n".join(lines) + "\n")
    driver.chmod(0o755)
    return driver


# ---------------------------------------------------------------------------
# Context fixtures for the pure runtime_environment contract.
# ---------------------------------------------------------------------------


def _capability(
    *, requested: bool = False, supported: bool = False,
    enabled: bool = False, runtime: Path | str | None = None,
    compiler_id: str = "fixture",
) -> dict[str, Any]:
    return {
        "requested": requested,
        "supported": supported,
        "enabled": enabled,
        "runtime": None if runtime is None else str(runtime),
        "compiler_id": compiler_id,
        "target": None,
        "evidence": {"control": "fixture probe"},
    }


def _context(
    *,
    system: str,
    compiler_id: str = "GNU",
    compiler_path: Path | str | None = None,
    machine: str | None = None,
    asan: dict[str, Any] | None = None,
    rtsan: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A schema-2-shaped context dict for the pure function's input domain."""
    return {
        "schema_version": 2,
        "platform": {
            "system": system,
            "host_system": system,
            "machine": machine or platform.machine(),
            "pointer_bits": 64,
        },
        "compiler": {
            "path": str(compiler_path if compiler_path is not None else sys.executable),
            "id": compiler_id,
            "version": "1",
        },
        "capabilities": {
            "asan": asan or _capability(),
            "rtsan": rtsan or _capability(),
            "coverage": _capability(),
            "frontend_tools": {},
        },
    }


# ---------------------------------------------------------------------------
# runtime_environment composition
# ---------------------------------------------------------------------------


def test_linux_asan_uses_recorded_compiler_not_filename(tmp_path: Path) -> None:
    """The brief's regression: id=GNU at a path whose basename is not a GCC
    spelling selects libasan.so; an existing LD_PRELOAD is preserved."""
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", platform.machine())
    context = _context(
        system="Linux",
        compiler_id="GNU",
        compiler_path="/custom/c++",  # recorded driver path — never opened
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    env = native_runtime.runtime_environment(
        context, {"LD_PRELOAD": "/preexisting/libkeep.so", "OTHER": "keep"}
    )
    assert env["LD_PRELOAD"] == f"{runtime}:/preexisting/libkeep.so"
    assert env["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)
    assert env["PYTHONMALLOC"] == "malloc"
    assert env["ASAN_OPTIONS"].endswith(
        "detect_leaks=0:detect_stack_use_after_return=1"
    )
    assert env["OTHER"] == "keep"
    assert "DYLD_INSERT_LIBRARIES" not in env


def test_darwin_asan_prepends_dyld_insert_libraries(tmp_path: Path) -> None:
    runtime = _write_macho(
        tmp_path / "libclang_rt.asan_osx_dynamic.dylib",
        "Darwin", platform.machine(),
    )
    context = _context(
        system="Darwin",
        compiler_id="AppleClang",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    env = native_runtime.runtime_environment(
        context,
        {"DYLD_INSERT_LIBRARIES": "/preexisting/libkeep.dylib"},
    )
    assert env["DYLD_INSERT_LIBRARIES"] == (
        f"{runtime}:/preexisting/libkeep.dylib"
    )
    assert env["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)
    assert "LD_PRELOAD" not in env


def test_no_sanitizer_leaves_environment_unchanged(tmp_path: Path) -> None:
    context = _context(system="Linux")
    original = {"LD_PRELOAD": "/keep.so", "ASAN_OPTIONS": "foo=1"}
    env = native_runtime.runtime_environment(context, dict(original))
    assert env == original


def test_disabled_records_do_not_inject_a_runtime(tmp_path: Path) -> None:
    """A supported-but-disabled lane must not inject its recorded runtime."""
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", platform.machine())
    context = _context(
        system="Linux",
        asan=_capability(supported=True, runtime=runtime),
    )
    env = native_runtime.runtime_environment(context, {"PATH": "/bin"})
    assert env == {"PATH": "/bin"}


def test_missing_recorded_runtime_fails(tmp_path: Path) -> None:
    context = _context(
        system="Linux",
        asan=_capability(
            requested=True, supported=True, enabled=True,
            runtime=tmp_path / "absent" / "libasan.so",
        ),
        compiler_path=sys.executable,
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="runtime"):
        native_runtime.runtime_environment(context, {})


def test_non_absolute_recorded_runtime_fails(tmp_path: Path) -> None:
    context = _context(
        system="Linux",
        asan=_capability(
            requested=True, supported=True, enabled=True,
            runtime="libasan.so",
        ),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="absolute"):
        native_runtime.runtime_environment(context, {})


def test_conflicting_asan_rtsan_fails(tmp_path: Path) -> None:
    asan_rt = _write_elf(tmp_path / "libasan.so", "Linux", platform.machine())
    rtsan_rt = _write_elf(
        tmp_path / "libclang_rt.rtsan-aarch64.so", "Linux", "aarch64"
    )
    context = _context(
        system="Linux",
        machine="aarch64",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=asan_rt
        ),
        rtsan=_capability(
            requested=True, supported=True, enabled=True, runtime=rtsan_rt
        ),
    )
    with pytest.raises(
        native_runtime.NativeRuntimeError, match="ASan.*RTSan|asan.*rtsan"
    ):
        native_runtime.runtime_environment(context, {})


def test_requested_but_not_enabled_fails(tmp_path: Path) -> None:
    """Requested/unsupported is a stale-context state: fail rather than run
    the test process uninstrumented."""
    context = _context(
        system="Linux",
        asan=_capability(requested=True, supported=False, enabled=False),
    )
    with pytest.raises(
        native_runtime.NativeRuntimeError, match="requested.*not enabled"
    ):
        native_runtime.runtime_environment(context, {})


def test_runtime_path_with_spaces_is_a_single_value(tmp_path: Path) -> None:
    runtime_dir = tmp_path / "runtime dir with spaces"
    runtime_dir.mkdir()
    runtime = _write_elf(
        runtime_dir / "libasan.so", "Linux", platform.machine()
    )
    context = _context(
        system="Linux",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["LD_PRELOAD"] == str(runtime)
    assert env["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)


def test_existing_asan_options_are_preserved(tmp_path: Path) -> None:
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", platform.machine())
    context = _context(
        system="Linux",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    env = native_runtime.runtime_environment(
        context, {"ASAN_OPTIONS": "abort_on_error=1"}
    )
    options = env["ASAN_OPTIONS"].split(":")
    assert "abort_on_error=1" in options
    assert "detect_leaks=0" in options
    assert "detect_stack_use_after_return=1" in options


def test_linux_never_preloads_an_apple_dylib(tmp_path: Path) -> None:
    dylib = _write_macho(
        tmp_path / "libclang_rt.asan_osx_dynamic.dylib",
        "Darwin", platform.machine(),
    )
    context = _context(
        system="Linux",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=dylib
        ),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="dylib"):
        native_runtime.runtime_environment(context, {})


def test_darwin_rejects_a_linux_shared_object(tmp_path: Path) -> None:
    runtime = _write_elf(
        tmp_path / "libclang_rt.asan-arm64.so", "Linux", "aarch64"
    )
    context = _context(
        system="Darwin",
        compiler_id="AppleClang",
        machine="arm64",
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="dylib"):
        native_runtime.runtime_environment(context, {})


def test_wrong_arch_recorded_runtime_fails(tmp_path: Path) -> None:
    machine = _normalize_machine(platform.machine())
    wrong = "x86_64" if machine == "aarch64" else "aarch64"
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", wrong)
    context = _context(
        system="Linux",
        machine=platform.machine(),
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="arch"):
        native_runtime.runtime_environment(context, {})


# ---------------------------------------------------------------------------
# Driver-based runtime resolution (recorded runtime absent).
# ---------------------------------------------------------------------------


def test_gnu_driver_query_selects_libasan_so(tmp_path: Path) -> None:
    """id=GNU asks the driver for libasan.so — even when the driver's
    basename reads like a clang spelling."""
    machine = _normalize_machine(platform.machine())
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", machine)
    driver = _fake_driver(tmp_path, "clang++-18", {"libasan.so": runtime})
    context = _context(
        system="Linux",
        compiler_id="GNU",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["LD_PRELOAD"] == str(runtime)
    assert (tmp_path / "clang++-18.argv.log").read_text().strip() == (
        "-print-file-name=libasan.so"
    )


def test_clang_driver_query_selects_the_compiler_runtime(tmp_path: Path) -> None:
    """id=Clang asks for the target-suffixed clang runtime — even when the
    driver's basename reads like a GCC spelling."""
    machine = _normalize_machine(platform.machine())
    name = f"libclang_rt.asan-{machine}.so"
    runtime = _write_elf(tmp_path / name, "Linux", machine)
    driver = _fake_driver(tmp_path, "g++-13", {name: runtime})
    context = _context(
        system="Linux",
        compiler_id="Clang",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["LD_PRELOAD"] == str(runtime)
    assert (tmp_path / "g++-13.argv.log").read_text().strip() == (
        f"-print-file-name={name}"
    )


def test_appleclang_driver_query_selects_the_osx_dylib(tmp_path: Path) -> None:
    name = "libclang_rt.asan_osx_dynamic.dylib"
    runtime = _write_macho(tmp_path / name, "Darwin", platform.machine())
    driver = _fake_driver(tmp_path, "c++", {name: runtime})
    context = _context(
        system="Darwin",
        compiler_id="AppleClang",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["DYLD_INSERT_LIBRARIES"] == str(runtime)


def test_unresolved_driver_answer_fails(tmp_path: Path) -> None:
    """A driver echoing the bare filename back is not a resolved runtime."""
    driver = _fake_driver(tmp_path, "cc", {})
    context = _context(
        system="Linux",
        compiler_id="GNU",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="unresolved"):
        native_runtime.runtime_environment(context, {})


def test_driver_resolving_to_wrong_arch_fails(tmp_path: Path) -> None:
    machine = _normalize_machine(platform.machine())
    wrong = "x86_64" if machine == "aarch64" else "aarch64"
    runtime = _write_elf(tmp_path / "libasan.so", "Linux", wrong)
    driver = _fake_driver(tmp_path, "cc", {"libasan.so": runtime})
    context = _context(
        system="Linux",
        compiler_id="GNU",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="arch"):
        native_runtime.runtime_environment(context, {})


def test_unknown_compiler_id_fails_closed(tmp_path: Path) -> None:
    driver = _fake_driver(tmp_path, "cc", {"libasan.so": "/never"})
    context = _context(
        system="Linux",
        compiler_id="IntelLLVM",
        compiler_path=driver,
        asan=_capability(requested=True, supported=True, enabled=True),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="compiler"):
        native_runtime.runtime_environment(context, {})


def test_rtsan_enabled_uses_the_rtsan_runtime(tmp_path: Path) -> None:
    machine = _normalize_machine(platform.machine())
    name = f"libclang_rt.rtsan-{machine}.so"
    runtime = _write_elf(tmp_path / name, "Linux", machine)
    driver = _fake_driver(tmp_path, "clang++", {name: runtime})
    context = _context(
        system="Linux",
        compiler_id="Clang",
        compiler_path=driver,
        rtsan=_capability(requested=True, supported=True, enabled=True),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["LD_PRELOAD"] == str(runtime)
    assert env["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)
    # RTSan is not ASan: no ASan-only environment contract.
    assert "ASAN_OPTIONS" not in env
    assert "PYTHONMALLOC" not in env


def test_gnu_rtsan_lane_never_resolves_a_runtime(tmp_path: Path) -> None:
    """GCC has no RTSan runtime: an enabled GCC rtsan record cannot produce
    one, so it must fail — T16 keeps qualification on the clang lane."""
    context = _context(
        system="Linux",
        compiler_id="GNU",
        compiler_path=_fake_driver(tmp_path, "cc", {}),
        rtsan=_capability(requested=True, supported=True, enabled=True),
    )
    with pytest.raises(native_runtime.NativeRuntimeError):
        native_runtime.runtime_environment(context, {})


def test_rtsan_requested_but_not_enabled_is_a_stale_context(
    tmp_path: Path,
) -> None:
    """Same fail-closed rule as the ASan lane: a requested-but-disabled
    rtsan record means the toolchain probe failed or the context is stale —
    never run the test process uninstrumented."""
    context = _context(
        system="Linux",
        rtsan=_capability(requested=True, supported=False, enabled=False),
    )
    with pytest.raises(
        native_runtime.NativeRuntimeError, match="requested.*not enabled"
    ):
        native_runtime.runtime_environment(context, {})


def test_darwin_rtsan_driver_query_selects_the_osx_dylib(
    tmp_path: Path,
) -> None:
    """The brew/upstream Clang Darwin RTSan runtime name resolves through
    the driver's -print-file-name and lands in DYLD_INSERT_LIBRARIES."""
    name = "libclang_rt.rtsan_osx_dynamic.dylib"
    runtime = _write_macho(tmp_path / name, "Darwin", platform.machine())
    driver = _fake_driver(tmp_path, "clang++", {name: runtime})
    context = _context(
        system="Darwin",
        compiler_id="Clang",
        compiler_path=driver,
        rtsan=_capability(requested=True, supported=True, enabled=True),
    )
    env = native_runtime.runtime_environment(context, {})
    assert env["DYLD_INSERT_LIBRARIES"] == str(runtime)
    assert env["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)
    assert (tmp_path / "clang++.argv.log").read_text().strip() == (
        f"-print-file-name={name}"
    )


def test_rtsan_record_named_for_the_wrong_lane_is_rejected(
    tmp_path: Path,
) -> None:
    """An enabled rtsan record whose runtime file names the ASan image must
    fail — the preloaded library has to be the lane's own runtime."""
    machine = _normalize_machine(platform.machine())
    wrong_lane = _write_elf(tmp_path / "libasan.so", "Linux", machine)
    context = _context(
        system="Linux",
        compiler_id="GNU",
        rtsan=_capability(
            requested=True, supported=True, enabled=True,
            runtime=wrong_lane,
        ),
    )
    with pytest.raises(
        native_runtime.NativeRuntimeError, match="rtsan"
    ):
        native_runtime.runtime_environment(context, {})


def test_asan_record_named_for_the_wrong_lane_is_rejected(
    tmp_path: Path,
) -> None:
    """Symmetric guard for the ASan lane."""
    machine = _normalize_machine(platform.machine())
    wrong_lane = _write_elf(
        tmp_path / f"libclang_rt.rtsan-{machine}.so", "Linux", machine
    )
    context = _context(
        system="Linux",
        compiler_id="Clang",
        asan=_capability(
            requested=True, supported=True, enabled=True,
            runtime=wrong_lane,
        ),
    )
    with pytest.raises(native_runtime.NativeRuntimeError, match="asan"):
        native_runtime.runtime_environment(context, {})


# ---------------------------------------------------------------------------
# The run --build CLI against a validated schema-2 context.
# ---------------------------------------------------------------------------


def _cli_context(
    tmp_path: Path,
    *,
    asan_runtime: Path | None = None,
    rtsan_runtime: Path | None = None,
    rtsan_requested: bool = False,
) -> dict[str, Any]:
    """A complete schema-2 context that load_context accepts on this host."""
    system = HOST_SYSTEM
    if system == "Darwin":
        sdk_record: dict[str, Any] = {
            "sdk": str(tmp_path.resolve()),
            "sdk_requested": str(tmp_path.resolve()),
            "sdk_source": "explicit",
            "sysroot": str(tmp_path.resolve()),
        }
        stdlib = {
            "name": "libc++",
            "version": "fixture",
            "hardening": {"mode": "EXTENSIVE", "effective": True},
        }
        hardening = {
            "supported": True, "mode": "EXTENSIVE", "requested_mode": "EXTENSIVE"
        }
    else:
        sdk_record = {
            "sdk": None,
            "sdk_requested": None,
            "sdk_source": "not-applicable",
            "sysroot": None,
        }
        stdlib = {
            "name": "libstdc++",
            "version": "fixture",
            "hardening": {"mode": "ASSERTIONS", "effective": True},
        }
        hardening = {
            "supported": False, "mode": "NOT_APPLICABLE",
            "requested_mode": "EXTENSIVE",
        }
    asan_on = asan_runtime is not None
    rtsan_on = rtsan_runtime is not None
    return {
        "schema_version": 2,
        "platform": {
            "system": system,
            "host_system": system,
            "machine": platform.machine(),
            "pointer_bits": 64,
        },
        **sdk_record,
        "stdlib": stdlib,
        "numerics": {"backend": "fixture"},
        "policy": {
            "required_flags": [],
            "accepted_warnings": [],
            "rejected_warnings": [],
        },
        "capabilities": {
            "asan": _capability(
                requested=asan_on, supported=asan_on, enabled=asan_on,
                runtime=asan_runtime,
            ),
            "rtsan": _capability(
                requested=rtsan_requested or rtsan_on,
                supported=rtsan_on,
                enabled=rtsan_on,
                runtime=rtsan_runtime,
            ),
            "coverage": _capability(),
            "frontend_tools": {},
        },
        "compiler": {
            "path": sys.executable, "id": "fixture", "version": "1",
        },
        "compiler_policy": {},
        "libcpp_hardening": hardening,
        "include_paths": [str(tmp_path.resolve())],
        "library_paths": [],
        "required_files": [],
        "source_roots": [str(tmp_path.resolve())],
        "python_executable": sys.executable,
        "tool_defaults": {},
        "tool_overrides": {},
        "target_contexts": {},
    }


def _write_cli_build(
    tmp_path: Path, *, asan_runtime: Path | None = None,
    rtsan_runtime: Path | None = None,
    cache_sanitize: str | None = None, cache_rtsan: str | None = None,
    compiler_id: str = "fixture",
) -> Path:
    build = tmp_path / "build"
    build.mkdir(exist_ok=True)
    source_root = tmp_path / "src"
    source_root.mkdir(exist_ok=True)
    manifest = source_root / "analysis_suppressions.json"
    manifest.write_text(json.dumps({"schema_version": 1, "suppressions": []}))
    context = _cli_context(
        tmp_path, asan_runtime=asan_runtime, rtsan_runtime=rtsan_runtime
    )
    context["analysis_suppressions_manifest"] = str(manifest.resolve())
    context["compiler"]["id"] = compiler_id
    (build / "native_check_context.json").write_text(json.dumps(context))
    cache_lines = []
    if cache_sanitize is not None:
        cache_lines.append(f"NATIVE_SANITIZE:BOOL={cache_sanitize}")
    if cache_rtsan is not None:
        cache_lines.append(f"NATIVE_RTSAN:BOOL={cache_rtsan}")
    if cache_lines:
        (build / "CMakeCache.txt").write_text("\n".join(cache_lines) + "\n")
    return build


def test_cli_propagates_the_child_exit_status(tmp_path: Path) -> None:
    build = _write_cli_build(tmp_path)
    code = native_runtime.main(
        ["run", "--build", str(build), "--", sys.executable, "-c",
         "import sys; sys.exit(42)"]
    )
    assert code == 42


def test_cli_runs_a_successful_child(tmp_path: Path, capfd) -> None:
    build = _write_cli_build(tmp_path)
    code = native_runtime.main(
        ["run", "--build", str(build), "--", sys.executable, "-c",
         "print('child-ran')"]
    )
    assert code == 0
    assert "child-ran" in capfd.readouterr().out


def test_cli_never_prints_the_environment(
    tmp_path: Path, capfd, monkeypatch: pytest.MonkeyPatch
) -> None:
    build = _write_cli_build(tmp_path)
    monkeypatch.setenv("BIKE_TEST_SECRET", "hunter2")
    code = native_runtime.main(
        ["run", "--build", str(build), "--", sys.executable, "-c",
         "print('ran')"]
    )
    assert code == 0
    captured = capfd.readouterr()
    assert "hunter2" not in captured.out
    assert "hunter2" not in captured.err


def test_cli_missing_command_fails(tmp_path: Path, capsys) -> None:
    build = _write_cli_build(tmp_path)
    assert native_runtime.main(["run", "--build", str(build), "--"]) == 2


def test_cli_missing_context_fails(tmp_path: Path) -> None:
    assert (
        native_runtime.main(
            ["run", "--build", str(tmp_path / "nope"), "--",
             sys.executable, "-c", "pass"]
        )
        == 2
    )


def test_cli_cache_context_drift_fails(tmp_path: Path) -> None:
    """A cache that requests a sanitizer the context did not record is a
    stale-context error, never a silently uninstrumented run."""
    build = _write_cli_build(tmp_path, cache_sanitize="ON")
    assert (
        native_runtime.main(
            ["run", "--build", str(build), "--", sys.executable, "-c", "pass"]
        )
        == 2
    )


def test_cli_cache_with_both_sanitizers_fails(tmp_path: Path) -> None:
    build = _write_cli_build(
        tmp_path, cache_sanitize="ON", cache_rtsan="ON"
    )
    assert (
        native_runtime.main(
            ["run", "--build", str(build), "--", sys.executable, "-c", "pass"]
        )
        == 2
    )


def test_cli_cache_requests_rtsan_but_context_did_not_enable(
    tmp_path: Path,
) -> None:
    """NATIVE_RTSAN=ON in the cache with a context that never enabled the
    lane is a stale/inconsistent pair — the runner fails closed rather than
    running uninstrumented."""
    build = _write_cli_build(tmp_path, cache_rtsan="ON")
    assert (
        native_runtime.main(
            ["run", "--build", str(build), "--", sys.executable, "-c", "pass"]
        )
        == 2
    )


def test_cli_enabled_rtsan_injects_the_real_runtime(
    tmp_path: Path, capfd, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A real RTSan-capable driver feeds a real preload into the child.

    Mirrors test_cli_enabled_asan_injects_the_real_runtime: the child proves
    the preload by checking the runtime is *mapped* — the same check
    native_loader.verify_sanitizer_runtime performs in production — and the
    evidence line must name the rtsan lane, never the ASan-only env.
    """
    driver = _rtsan_capable_host_driver(tmp_path)
    if driver is None:
        pytest.skip("no RTSan-capable host driver (-fsanitize=realtime)")
    compiler_id = _compiler_id(driver)
    runtime = (
        _real_rtsan_runtime(driver, compiler_id)
        if compiler_id is not None
        else None
    )
    if compiler_id is None or runtime is None:
        pytest.skip("capable driver has no resolvable RTSan runtime")
    build = _write_cli_build(
        tmp_path, rtsan_runtime=runtime, cache_rtsan="ON",
        compiler_id=compiler_id,
    )
    monkeypatch.setenv("PYTHONPATH", str(LOADER_DIR))
    code = native_runtime.main(
        ["run", "--build", str(build), "--", sys.executable, "-c",
         "import native_loader; "
         "mapped = native_loader.verify_sanitizer_runtime(); "
         "assert mapped is not None; print('runtime-injected', mapped)"]
    )
    assert code == 0
    captured = capfd.readouterr()
    assert "runtime-injected" in captured.out
    assert str(runtime.resolve()) in captured.out
    assert str(runtime.resolve()) in captured.err  # the evidence line names it
    assert "sanitizer=rtsan" in captured.err


def _host_cxx() -> Path | None:
    """A real host C++ driver: $CXX first, then the usual spellings."""
    for spelling in (
        os.environ.get("CXX"), "c++", "clang++", "g++"
    ):
        if not spelling:
            continue
        located = shutil.which(spelling)
        if located and Path(located).is_file():
            return Path(located)
    return None


def _compiler_id(driver: Path) -> str | None:
    try:
        completed = subprocess.run(
            [str(driver), "--version"],
            capture_output=True, text=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    banner = (completed.stdout + completed.stderr).strip().lower()
    first = banner.splitlines()[0] if banner else ""
    if "apple" in first and "clang" in first:
        return "AppleClang"
    if "clang" in first:
        return "Clang"
    if "free software foundation" in banner or "(gcc)" in banner:
        return "GNU"
    return None


def _real_asan_runtime(driver: Path, compiler_id: str) -> Path | None:
    """Ask a real driver for the runtime this task's naming selects."""
    machine = _normalize_machine(platform.machine())
    if compiler_id in {"Clang", "AppleClang"}:
        name = (
            "libclang_rt.asan_osx_dynamic.dylib"
            if HOST_SYSTEM == "Darwin"
            else f"libclang_rt.asan-{machine}.so"
        )
    elif compiler_id == "GNU" and HOST_SYSTEM == "Linux":
        name = "libasan.so"
    else:
        return None
    try:
        completed = subprocess.run(
            [str(driver), f"-print-file-name={name}"],
            capture_output=True, text=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    answer = completed.stdout.strip().splitlines()
    if completed.returncode != 0 or not answer or answer[0].strip() == name:
        return None
    candidate = Path(answer[0].strip())
    return candidate if candidate.is_file() else None


def _driver_accepts_rtsan(driver: Path, work_dir: Path) -> bool:
    """Probe: does this driver compile+link a TU under -fsanitize=realtime?"""
    work_dir.mkdir(parents=True, exist_ok=True)
    source = work_dir / "rtsan_accept_probe.cpp"
    source.write_text("int main() { return 0; }\n")
    try:
        completed = subprocess.run(
            [
                str(driver), "-std=c++23", "-fsanitize=realtime",
                str(source), "-o", str(work_dir / "rtsan_accept_probe"),
            ],
            text=True, capture_output=True, check=False, timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def _rtsan_capable_host_driver(tmp_path: Path) -> Path | None:
    """The first real host driver whose toolchain qualifies for RTSan.

    RTSan is upstream-Clang-only today: AppleClang and GCC reject
    -fsanitize=realtime, so the usual host spellings get probed AND the
    versioned/Homebrew LLVM installs where the lane actually lives.
    """
    names: list[str] = [
        os.environ.get("CXX", ""), "c++", "clang++", "g++",
        "/opt/homebrew/opt/llvm/bin/clang++",
        "/usr/local/opt/llvm/bin/clang++",
    ]
    for directory in ("/usr/bin", "/usr/local/bin", "/opt/homebrew/bin"):
        root = Path(directory)
        if root.is_dir():
            names.extend(
                str(path)
                for path in sorted(root.glob("clang++-*"), reverse=True)
            )
    seen: set[str] = set()
    for index, name in enumerate(names):
        if not name:
            continue
        located = Path(name) if Path(name).is_file() else (
            Path(shutil.which(name)) if shutil.which(name) else None
        )
        if located is None:
            continue
        key = str(located.resolve())
        if key in seen:
            continue
        seen.add(key)
        if _driver_accepts_rtsan(located, tmp_path / f"probe{index}"):
            return located
    return None


def _real_rtsan_runtime(driver: Path, compiler_id: str) -> Path | None:
    """Ask a real RTSan-capable driver for its realtime runtime image."""
    if compiler_id not in {"Clang", "AppleClang"}:
        return None
    machine = _normalize_machine(platform.machine())
    name = (
        "libclang_rt.rtsan_osx_dynamic.dylib"
        if HOST_SYSTEM == "Darwin"
        else f"libclang_rt.rtsan-{machine}.so"
    )
    try:
        completed = subprocess.run(
            [str(driver), f"-print-file-name={name}"],
            capture_output=True, text=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    answer = completed.stdout.strip().splitlines()
    if completed.returncode != 0 or not answer or answer[0].strip() == name:
        return None
    candidate = Path(answer[0].strip())
    return candidate if candidate.is_file() else None


def test_cli_enabled_asan_injects_the_real_runtime(
    tmp_path: Path, capfd, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A real driver query feeds a real preload into the child process.

    dyld clears DYLD_INSERT_LIBRARIES after applying it, so the child proves
    the preload by checking the runtime is *mapped* — the same check
    native_loader.verify_sanitizer_runtime performs in production.
    """
    driver = _host_cxx()
    if driver is None:
        pytest.skip("no host C++ driver")
    compiler_id = _compiler_id(driver)
    runtime = (
        _real_asan_runtime(driver, compiler_id)
        if compiler_id is not None
        else None
    )
    if compiler_id is None or runtime is None:
        pytest.skip("host driver has no resolvable ASan runtime")
    build = _write_cli_build(
        tmp_path, asan_runtime=runtime, cache_sanitize="ON",
        compiler_id=compiler_id,
    )
    monkeypatch.setenv("PYTHONPATH", str(LOADER_DIR))
    code = native_runtime.main(
        ["run", "--build", str(build), "--", sys.executable, "-c",
         "import native_loader; "
         "mapped = native_loader.verify_sanitizer_runtime(); "
         "assert mapped is not None; print('runtime-injected', mapped)"]
    )
    assert code == 0
    captured = capfd.readouterr()
    assert "runtime-injected" in captured.out
    # The context loader resolves symlinks (GCC's -print-file-name answer
    # libasan.so is a symlink to libasan.so.8.0.0), so both the mapped
    # image and the evidence line name the resolved file.
    assert str(runtime.resolve()) in captured.out
    assert str(runtime.resolve()) in captured.err  # the evidence line names it


# ---------------------------------------------------------------------------
# Real-toolchain controls: compile the checked-in control source with the
# policy's sanitizer lane and run both modes under the computed environment.
# ---------------------------------------------------------------------------


def _instrumented_control_binary(driver: Path, compiler_id: str, directory: Path) -> Path:
    """Compile sanitizer_control.cpp with the toolchain's real ASan lane."""
    if compiler_id in {"Clang", "AppleClang"}:
        sanitize = "-fsanitize=address,undefined,local-bounds,float-cast-overflow"
        compile_flags = [
            sanitize, "-fno-sanitize-recover=all", "-fno-omit-frame-pointer"
        ]
        link_flags = [*compile_flags, "-shared-libasan"]
    elif compiler_id == "GNU":
        compile_flags = [
            "-fsanitize=address,undefined",
            "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
        ]
        link_flags = list(compile_flags)
    else:  # pragma: no cover - guarded by the caller
        raise AssertionError(compiler_id)
    output = directory / "sanitizer_control"
    completed = subprocess.run(
        [
            str(driver), "-std=c++23", "-g", "-O0",
            *compile_flags, str(CONTROL_SOURCE), "-o", str(output),
            *link_flags,
        ],
        capture_output=True, text=True, timeout=300, check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"control compile failed: {completed.stderr.strip()}"
        )
    return output


@pytest.fixture()
def instrumented_control(tmp_path: Path):
    """The real compiler/runtime pair this host can instrument with."""
    driver = _host_cxx()
    if driver is None:
        pytest.skip("no host C++ driver")
    compiler_id = _compiler_id(driver)
    if compiler_id is None:
        pytest.skip(f"unrecognized host driver: {driver}")
    runtime = _real_asan_runtime(driver, compiler_id)
    if runtime is None:
        pytest.skip(f"{driver} cannot resolve an ASan runtime")
    try:
        binary = _instrumented_control_binary(driver, compiler_id, tmp_path)
    except RuntimeError as error:
        pytest.skip(str(error))
    context = _context(
        system=HOST_SYSTEM,
        compiler_id=compiler_id,
        compiler_path=driver,
        asan=_capability(
            requested=True, supported=True, enabled=True, runtime=runtime
        ),
    )
    return binary, context


def test_instrumented_clean_control_runs(instrumented_control) -> None:
    binary, context = instrumented_control
    env = native_runtime.runtime_environment(context, dict(os.environ))
    completed = subprocess.run(
        [str(binary)], env=env, capture_output=True, text=True, timeout=60,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_instrumented_use_after_free_reports_asan(instrumented_control) -> None:
    """The negative control must die with the real ASan diagnostic — a
    loader error or unrelated crash is not a sanitizer result."""
    binary, context = instrumented_control
    env = native_runtime.runtime_environment(context, dict(os.environ))
    completed = subprocess.run(
        [str(binary), "use-after-free"],
        env=env, capture_output=True, text=True, timeout=60, check=False,
    )
    assert completed.returncode != 0
    assert "AddressSanitizer" in completed.stderr, completed.stderr
    assert "heap-use-after-free" in completed.stderr, completed.stderr


# ---------------------------------------------------------------------------
# The selected ASan build: a fresh Python subprocess loads the instrumented
# extension under the computed environment, with a load-bearing proof.
# ---------------------------------------------------------------------------


def _selected_asan_context() -> dict[str, Any] | None:
    build_value = os.environ.get("NATIVE_TEST_BUILD_PATH")
    if not build_value:
        return None
    build = Path(build_value)
    try:
        context = native_checks.load_context(build)
    except native_checks.CheckError:
        return None
    asan = context.get("capabilities", {}).get("asan", {})
    if not asan.get("enabled"):
        return None
    return context


def _linker_resolves_runtime(extension: Path, runtime: Path) -> bool:
    """Does the extension's own linkage find the runtime without preload?

    Returns True when the loader can satisfy the runtime's NEEDED name
    through the image's own search paths (Darwin LC_RPATH entries covering
    the runtime directory, or a Linux NEEDED name on the default search
    path); False when the preload is load-bearing.
    """
    name = runtime.name
    if HOST_SYSTEM == "Darwin":
        completed = subprocess.run(
            ["otool", "-l", str(extension)],
            capture_output=True, text=True, check=False,
        )
        lines = completed.stdout.splitlines()
        needed_names = {
            line.split("name", 1)[1].strip().split(" ")[0]
            for line in lines
            if " name " in line and ".dylib" in line
        }
        runtime_parent = str(runtime.resolve().parent)
        rpaths = {
            line.split("path", 1)[1].strip().rsplit(" ", 1)[0]
            for line in lines
            if line.strip().startswith("path ")
        }
        for needed in needed_names:
            if needed.endswith(name) and needed.startswith("@rpath/"):
                if runtime_parent in rpaths:
                    return True
            elif needed == str(runtime):
                return True
        return False
    completed = subprocess.run(
        ["ldd", str(extension)], capture_output=True, text=True, check=False
    )
    for line in completed.stdout.splitlines():
        if name in line and "=> not found" not in line and "=>" in line:
            resolved = line.split("=>", 1)[1].strip().split(" ", 1)[0]
            return Path(resolved).is_file()
    return False


def test_selected_asan_extension_loads_in_a_fresh_process(tmp_path: Path) -> None:
    context = _selected_asan_context()
    if context is None:
        pytest.skip(
            "no ASan-enabled native build selected "
            "(set NATIVE_TEST_BUILD_PATH)"
        )
    build = Path(os.environ["NATIVE_TEST_BUILD_PATH"])
    artifacts = [
        candidate for suffix in (
            ".so", ".cpython-313-darwin.so", ".cpython-314-darwin.so",
            ".cpython-313-aarch64-linux-gnu.so",
            ".cpython-314-aarch64-linux-gnu.so",
            ".cpython-313-x86_64-linux-gnu.so",
            ".cpython-314-x86_64-linux-gnu.so",
        )
        if (candidate := build / f"bike_native{suffix}").is_file()
    ]
    assert len(artifacts) == 1, f"expected one extension in {build}"
    extension = artifacts[0]

    env = native_runtime.runtime_environment(context, dict(os.environ))
    env["PYTHONPATH"] = str(LOADER_DIR)
    env["NATIVE_TEST_BUILD_PATH"] = str(build)
    child = subprocess.run(
        [
            sys.executable, "-c",
            "import native_loader; "
            "module = native_loader.load_native(); "
            "print('imported', module.__file__)",
        ],
        env=env, capture_output=True, text=True, timeout=120, check=False,
    )
    assert child.returncode == 0, child.stderr
    assert "imported" in child.stdout

    # Load-bearing proof: the same import without the helper's environment
    # must not produce a working instrumented extension. On Darwin the
    # embedded toolchain rpath can find the runtime file, but a dlopen'd
    # ASan is "loaded too late" — its interceptors never install and ASan
    # aborts the process itself. A Linux/GCC NEEDED lookup on the default
    # path could in principle satisfy the image; accept that only when the
    # extension's own linkage demonstrably resolves the runtime.
    runtime = Path(env["NATIVE_TEST_SANITIZER_RUNTIME"])
    bare_env = dict(os.environ)
    bare_env.pop("NATIVE_TEST_SANITIZER_RUNTIME", None)
    for preload in ("DYLD_INSERT_LIBRARIES", "LD_PRELOAD"):
        bare_env.pop(preload, None)
    bare_env["PYTHONPATH"] = str(LOADER_DIR)
    bare_env["NATIVE_TEST_BUILD_PATH"] = str(build)
    bare = subprocess.run(
        [
            sys.executable, "-c",
            "import native_loader; native_loader.load_native()",
        ],
        env=bare_env, capture_output=True, text=True, timeout=120,
        check=False,
    )
    if bare.returncode == 0:
        assert _linker_resolves_runtime(extension, runtime), (
            "the instrumented extension loaded without the helper env and "
            "without a self-resolving linkage — the preload claim is void"
        )
    else:
        loader_error = (
            "cannot open shared object" in bare.stderr
            or "Library not loaded" in bare.stderr
            or "image not found" in bare.stderr
            or "Interceptors" in bare.stderr
            or "loaded too late" in bare.stderr
            # GCC/Linux: the runtime resolves through NEEDED but ASan
            # refuses to start unless it was preloaded first.
            or "does not come first" in bare.stderr
            or "AddressSanitizer" in bare.stderr
        )
        assert loader_error, bare.stderr


def _selected_rtsan_context() -> dict[str, Any] | None:
    build_value = os.environ.get("NATIVE_TEST_BUILD_PATH")
    if not build_value:
        return None
    build = Path(build_value)
    try:
        context = native_checks.load_context(build)
    except native_checks.CheckError:
        return None
    rtsan = context.get("capabilities", {}).get("rtsan", {})
    if not rtsan.get("enabled"):
        return None
    return context


def test_selected_rtsan_extension_loads_in_a_fresh_process() -> None:
    """The RTSan-enabled build imports in a fresh process under the computed
    environment, with the rtsan runtime verifiably mapped."""
    context = _selected_rtsan_context()
    if context is None:
        pytest.skip(
            "no RTSan-enabled native build selected "
            "(set NATIVE_TEST_BUILD_PATH)"
        )
    build = Path(os.environ["NATIVE_TEST_BUILD_PATH"])
    artifacts = [
        candidate for suffix in (
            ".so", ".cpython-313-darwin.so", ".cpython-314-darwin.so",
            ".cpython-313-aarch64-linux-gnu.so",
            ".cpython-314-aarch64-linux-gnu.so",
            ".cpython-313-x86_64-linux-gnu.so",
            ".cpython-314-x86_64-linux-gnu.so",
        )
        if (candidate := build / f"bike_native{suffix}").is_file()
    ]
    assert len(artifacts) == 1, f"expected one extension in {build}"

    env = native_runtime.runtime_environment(context, dict(os.environ))
    env["PYTHONPATH"] = str(LOADER_DIR)
    env["NATIVE_TEST_BUILD_PATH"] = str(build)
    child = subprocess.run(
        [
            sys.executable, "-c",
            "import native_loader; "
            "module = native_loader.load_native(); "
            "print('imported', module.__file__)",
        ],
        env=env, capture_output=True, text=True, timeout=120, check=False,
    )
    assert child.returncode == 0, child.stderr
    assert "imported" in child.stdout
