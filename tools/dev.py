"""Explicit cross-platform development commands (portability task T03).

One source-build workflow for macOS and Linux:

    uv run --frozen --group native --no-sync python tools/dev.py --help

Commands:

    bootstrap  Sync the locked native+dev environment via an explicit
               ``uv sync --locked`` (optionally offline from a wheelhouse).
    configure  Configure the native build through the frozen project
               interpreter environment (Debug/Release, optional compiler).
    build      Compile a previously configured native build directory.
    doctor     Read-only named environment checks; structured JSON on
               stdout, human diagnostics on stderr.
    test       Select one absolute build directory and delegate the
               quick/native/full profile to ``tools/run_tests.sh``.

Boundaries: no compilation on import, no implicit ``uv sync``/relock, no
``shell=True`` — every subprocess is an argument array with the checkout
as ``cwd`` and the subprocess status is propagated as the exit code.
Relative ``--build`` selectors resolve against the checkout, never the
caller cwd, so the same command works from any directory and from
checkouts whose path contains spaces.
"""
from __future__ import annotations

import argparse
import importlib.machinery
import importlib.metadata
import json
import os
import platform
import re
import shlex
import subprocess
import sys
import sysconfig
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# Every build-facing command runs inside the frozen native group without
# touching the environment: no sync, no relock, no resolution.
PREFIX = ["uv", "run", "--frozen", "--group", "native", "--no-sync"]

CONFIGS = ("Debug", "Release")
PROFILES = ("quick", "native", "full")
SCHEMA_VERSION = 1
EXTENSION_MODULE = "bike_native"
DEPENDENCY_PROBE = Path("tools") / "native_dependencies.py"


def build_commands(
    root: Path, build: Path, config: str, compiler: str | None
) -> list[list[str]]:
    """Argv arrays for configure + compile; token boundaries preserved."""
    if config not in {"Debug", "Release"}:
        raise ValueError("config must be Debug or Release")
    if not root.is_absolute() or not build.is_absolute():
        raise ValueError("root and build must be absolute paths")
    configure = [
        *PREFIX, "cmake",
        "-S", str(root / "native"),
        "-B", str(build),
        f"-DCMAKE_BUILD_TYPE={config}",
    ]
    if compiler is not None:
        configure.append(f"-DCMAKE_CXX_COMPILER={compiler}")
    return [configure, _compile_command(build)]


def _compile_command(build: Path) -> list[str]:
    """The compile half of build_commands; also the `build` subcommand."""
    return [*PREFIX, "cmake", "--build", str(build), "--parallel"]


def resolve_build_path(root: Path, value: str) -> Path:
    """Absolute build dir; relative selectors resolve against the checkout."""
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = Path(root) / candidate
    return candidate.resolve()


def _cli_path(value: str) -> Path:
    """Caller-relative CLI paths resolve against the invoking cwd."""
    candidate = Path(value)
    return candidate if candidate.is_absolute() else (Path.cwd() / candidate).resolve()


def _run(argv: list[str], *, cwd: Path, env: dict | None = None) -> int:
    """The single spawn point: argument array, explicit cwd, status out."""
    print(f"+ {shlex.join([str(token) for token in argv])}", file=sys.stderr)
    completed = subprocess.run(
        [str(token) for token in argv], cwd=cwd, env=env, check=False
    )
    return completed.returncode


def _capture(argv: list[str], *, cwd: Path, timeout: int = 30) -> dict:
    """Read-only probe for doctor; failures are evidence, not exceptions."""
    try:
        completed = subprocess.run(
            [str(token) for token in argv],
            cwd=cwd,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"argv": list(argv), "returncode": None, "error": str(exc)}
    return {
        "argv": list(argv),
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def _first_line(text: str) -> str:
    return text.strip().splitlines()[0] if text.strip() else ""


def _check(name: str, status: str, evidence: str, remediation=None) -> dict:
    return {
        "name": name,
        "status": status,
        "evidence": evidence,
        "remediation": remediation,
    }


def _python_floor(root: Path) -> tuple[int, int] | None:
    try:
        document = tomllib.loads(
            (root / "pyproject.toml").read_text(encoding="utf-8")
        )
    except (OSError, tomllib.TOMLDecodeError):
        return None
    spec = str(document.get("project", {}).get("requires-python", ""))
    match = re.search(r">=\s*(\d+)\.(\d+)", spec)
    return (int(match.group(1)), int(match.group(2))) if match else None


def _native_exact_pins(root: Path) -> dict[str, str]:
    """The ``==`` pins of pyproject's native dependency group."""
    document = tomllib.loads(
        (root / "pyproject.toml").read_text(encoding="utf-8")
    )
    pins: dict[str, str] = {}
    for entry in document.get("dependency-groups", {}).get("native", []):
        name, separator, version = str(entry).partition("==")
        if separator:
            pins[name.strip()] = version.strip()
    return pins


def _doctor_interpreter(root: Path) -> dict:
    evidence = (
        f"{sys.executable} "
        f"({platform.python_implementation()} {platform.python_version()}, "
        f"soabi={sysconfig.get_config_var('SOABI')})"
    )
    floor = _python_floor(root)
    if floor is not None and sys.version_info[:2] < floor:
        return _check(
            "interpreter", "fail", evidence,
            f"pyproject requires-python is >={floor[0]}.{floor[1]}; "
            "select a conforming interpreter via bootstrap --python",
        )
    return _check("interpreter", "pass", evidence, None)


def _doctor_lock(root: Path) -> dict:
    lock = root / "uv.lock"
    if not lock.is_file():
        return _check(
            "lock", "fail", f"{lock} is absent",
            "restore the committed uv.lock, then run tools/dev.py bootstrap",
        )
    probe = _capture(["uv", "lock", "--check"], cwd=root)
    if probe.get("returncode") == 0:
        return _check(
            "lock", "pass",
            "uv.lock present; `uv lock --check` reports the lock fresh",
            None,
        )
    detail = _first_line(probe.get("stderr") or "") or probe.get("error") \
        or f"exit {probe.get('returncode')}"
    return _check(
        "lock", "fail", f"uv lock --check: {detail}",
        "relock deliberately with `uv lock` and review the diff; "
        "dev.py never regenerates the lock",
    )


def _doctor_packages(root: Path) -> dict:
    try:
        pins = _native_exact_pins(root)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        return _check(
            "packages", "fail",
            f"cannot read native pins from pyproject.toml: {exc}",
            "restore pyproject.toml, then run tools/dev.py bootstrap",
        )
    mismatches = []
    for name, expected in pins.items():
        try:
            actual = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            actual = None
        if actual != expected:
            mismatches.append(f"{name}: required {expected}, found {actual}")
    if mismatches:
        return _check(
            "packages", "fail", "; ".join(mismatches),
            "run tools/dev.py bootstrap to sync the locked native group",
        )
    evidence = ", ".join(f"{n}=={v}" for n, v in pins.items()) \
        or "no exact native pins declared"
    return _check("packages", "pass", evidence, None)


def _doctor_compilers(root: Path) -> dict:
    probes = {
        "c++": _capture(["c++", "--version"], cwd=root),
        "cc": _capture(["cc", "--version"], cwd=root),
        "cmake": _capture(["cmake", "--version"], cwd=root),
    }
    evidence = "; ".join(
        f"{tool}: {_first_line(p.get('stdout') or '') or p.get('error') or 'unavailable'}"
        for tool, p in probes.items()
    )
    missing = [
        tool for tool, p in probes.items()
        if tool != "cc" and p.get("returncode") != 0
    ]
    if missing:
        return _check(
            "compilers", "fail", evidence,
            f"install the missing build tools ({', '.join(missing)}): "
            "Xcode Command Line Tools on macOS, clang/gcc + cmake on Linux",
        )
    return _check("compilers", "pass", evidence, None)


def _doctor_dependency_probe(root: Path) -> tuple[dict, dict | None]:
    """The T05 probe of the selected interpreter; a named failure until it
    exists, then live JSON evidence. Never a passing stub."""
    script = root / DEPENDENCY_PROBE
    if not script.is_file():
        return _check(
            "dependency_probe", "fail",
            f"unavailable: {DEPENDENCY_PROBE.as_posix()} is not implemented "
            "yet (T05 scope)",
            "defer native-build doctor checks until T05 lands",
        ), None
    probe = _capture(
        [sys.executable, str(script), "--python", sys.executable],
        cwd=root, timeout=180,
    )
    if probe.get("returncode") != 0:
        detail = _first_line(probe.get("stderr") or "") \
            or probe.get("error") or f"exit {probe.get('returncode')}"
        return _check(
            "dependency_probe", "fail", f"probe failed: {detail}",
            "run tools/native_dependencies.py --python <interpreter> "
            "directly for the full diagnostic",
        ), None
    try:
        document = json.loads(probe["stdout"])
    except (KeyError, json.JSONDecodeError) as exc:
        return _check(
            "dependency_probe", "fail",
            f"probe did not emit schema JSON on stdout: {exc}",
            "fix tools/native_dependencies.py to write schema-1 JSON",
        ), None
    if document.get("schema_version") != SCHEMA_VERSION:
        return _check(
            "dependency_probe", "fail",
            f"unsupported dependency schema "
            f"{document.get('schema_version')!r}",
            "expected schema_version=1",
        ), None
    mujoco = document.get("mujoco", {})
    return _check(
        "dependency_probe", "pass",
        f"mujoco {mujoco.get('version')} at {mujoco.get('library_path')}",
        None,
    ), document


def _doctor_selected_artifact(build: Path) -> dict:
    artifacts = [
        candidate
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (candidate := build / f"{EXTENSION_MODULE}{suffix}").is_file()
    ]
    if len(artifacts) == 1:
        return _check(
            "selected_artifact", "pass", str(artifacts[0].resolve()), None,
        )
    if not artifacts:
        return _check(
            "selected_artifact", "fail",
            f"no bike_native extension in {build}",
            "run tools/dev.py configure --build <dir> --config Release "
            "then tools/dev.py build --build <dir>",
        )
    return _check(
        "selected_artifact", "fail",
        f"ambiguous bike_native artifacts in {build}: "
        + ", ".join(str(p) for p in artifacts),
        "keep exactly one extension per selected build directory",
    )


def _doctor_numerical_profile(probe_document: dict | None) -> dict:
    """T06 numerics record from the dependency probe; named failure until
    the probe exists and reports a qualified provider."""
    if probe_document is None:
        return _check(
            "numerical_profile", "fail",
            "unavailable: the numerical profile is produced by the "
            "dependency probe (T05/T06 scope)",
            "defer --numerics until the dependency probe lands",
        )
    numerics = probe_document.get("numerics")
    if not isinstance(numerics, dict) or not numerics.get("backend"):
        return _check(
            "numerical_profile", "fail",
            "dependency probe returned no qualified numerics record",
            "qualify the NumPy BLAS/LAPACK provider per "
            "docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md",
        )
    return _check(
        "numerical_profile", "pass",
        f"backend={numerics.get('backend')} "
        f"blas_bits={numerics.get('blas_integer_bits')} "
        f"lapack_bits={numerics.get('lapack_integer_bits')}",
        None,
    )


def doctor_report(
    root: Path, build: Path | None = None, numerics: bool = False
) -> dict:
    """Read-only doctor report (schema 1). Checks are named; optional
    checks appear only when requested via --build/--numerics."""
    root = Path(root).resolve()
    checks = [
        _doctor_interpreter(root),
        _doctor_lock(root),
        _doctor_packages(root),
        _doctor_compilers(root),
    ]
    probe_document = None
    if build is not None or numerics:
        probe_check, probe_document = _doctor_dependency_probe(root)
        checks.append(probe_check)
    if build is not None:
        checks.append(_doctor_selected_artifact(Path(build)))
    if numerics:
        checks.append(_doctor_numerical_profile(probe_document))
    status = "pass" if all(c["status"] == "pass" for c in checks) else "fail"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "root": str(root),
        "checks": checks,
    }


def _cmd_bootstrap(args, root: Path) -> int:
    lock = root / "uv.lock"
    if not lock.is_file():
        print(
            f"bootstrap requires the committed lock file {lock}; "
            "refusing to resolve without it",
            file=sys.stderr,
        )
        return 2
    argv = ["uv", "sync", "--locked", "--group", "native", "--group", "dev"]
    if args.python is not None:
        argv += ["--python", args.python]
    if args.offline:
        wheelhouse = _cli_path(args.wheelhouse)
        if not wheelhouse.is_dir():
            print(
                f"bootstrap --wheelhouse is not a directory: {wheelhouse}",
                file=sys.stderr,
            )
            return 2
        # uv's supported controls: no index, only the local wheelhouse.
        argv += ["--offline", "--no-index", "--find-links", str(wheelhouse)]
    return _run(argv, cwd=root)


def _cmd_configure(args, root: Path) -> int:
    build = resolve_build_path(root, args.build)
    configure = build_commands(root, build, args.config, args.compiler)[0]
    return _run(configure, cwd=root)


def _cmd_build(args, root: Path) -> int:
    build = resolve_build_path(root, args.build)
    return _run(_compile_command(build), cwd=root)


def _cmd_doctor(args, root: Path) -> int:
    build = resolve_build_path(root, args.build) if args.build else None
    report = doctor_report(root, build=build, numerics=args.numerics)
    sys.stdout.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    for check in report["checks"]:
        if check["status"] == "fail":
            remediation = f" — remediation: {check['remediation']}" \
                if check["remediation"] else ""
            print(
                f"doctor[{check['name']}]: {check['evidence']}{remediation}",
                file=sys.stderr,
            )
    return 0 if report["status"] == "pass" else 1


def _cmd_test(args, root: Path) -> int:
    build = resolve_build_path(root, args.build)
    env = dict(os.environ)
    env["BIKE_NATIVE_BUILD_PATH"] = str(build)
    env["NATIVE_TEST_BUILD_PATH"] = str(build)
    return _run(
        ["bash", str(root / "tools" / "run_tests.sh"), args.profile],
        cwd=root,
        env=env,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="dev.py",
        description="Explicit cross-platform development commands "
                    "(bootstrap/configure/build/doctor/test).",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    bootstrap = sub.add_parser(
        "bootstrap",
        help="explicit `uv sync --locked` of the native+dev groups",
    )
    bootstrap.add_argument(
        "--python", metavar="VERSION",
        help="interpreter selector for the project environment",
    )
    bootstrap.add_argument(
        "--offline", action="store_true",
        help="sync without network; requires --wheelhouse",
    )
    bootstrap.add_argument(
        "--wheelhouse", metavar="PATH",
        help="local wheel directory used as the only package source "
             "(requires --offline)",
    )

    configure = sub.add_parser(
        "configure", help="cmake configure inside the frozen native group",
    )
    configure.add_argument(
        "--build", required=True, metavar="PATH",
        help="build directory (relative paths resolve against the checkout)",
    )
    configure.add_argument(
        "--config", required=True, choices=CONFIGS,
        help="CMAKE_BUILD_TYPE",
    )
    configure.add_argument(
        "--compiler", metavar="PATH",
        help="CMAKE_CXX_COMPILER override (name or path, passed verbatim)",
    )

    build = sub.add_parser(
        "build", help="compile a configured build directory",
    )
    build.add_argument(
        "--build", required=True, metavar="PATH",
        help="build directory (relative paths resolve against the checkout)",
    )

    doctor = sub.add_parser(
        "doctor", help="read-only environment checks (JSON on stdout)",
    )
    doctor.add_argument(
        "--build", metavar="PATH",
        help="also check the selected native artifact and linked "
             "dependencies for this build directory",
    )
    doctor.add_argument(
        "--numerics", action="store_true",
        help="also check the qualified BLAS/LAPACK numerical profile",
    )

    test = sub.add_parser(
        "test", help="delegate a quick/native/full profile to run_tests.sh",
    )
    test.add_argument(
        "--build", required=True, metavar="PATH",
        help="build directory exported as BIKE_NATIVE_BUILD_PATH and "
             "NATIVE_TEST_BUILD_PATH",
    )
    test.add_argument(
        "--profile", required=True, choices=PROFILES,
        help="verification profile implemented by tools/run_tests.sh",
    )
    return parser


def main(argv: list[str] | None = None, *, root: Path | None = None) -> int:
    root = REPO_ROOT if root is None else Path(root).resolve()
    parser = _parser()
    args = parser.parse_args(argv)
    if getattr(args, "offline", False) and args.wheelhouse is None:
        parser.error("bootstrap --offline requires --wheelhouse PATH")
    if getattr(args, "wheelhouse", None) is not None and not args.offline:
        parser.error("bootstrap --wheelhouse only applies with --offline")
    handlers = {
        "bootstrap": _cmd_bootstrap,
        "configure": _cmd_configure,
        "build": _cmd_build,
        "doctor": _cmd_doctor,
        "test": _cmd_test,
    }
    return handlers[args.command](args, root)


if __name__ == "__main__":
    raise SystemExit(main())
