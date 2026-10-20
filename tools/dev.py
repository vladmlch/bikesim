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
import tempfile
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
    root: Path, build: Path, config: str, compiler: str | None,
    *, python_executable: str | None = None,
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
    if python_executable is not None:
        configure.append(f"-DPython_EXECUTABLE={python_executable}")
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


def project_environment(root: Path) -> dict[str, str]:
    """One explicit uv environment for all commands, including child probes.

    Installer-selected environments must be absolute. An inherited active
    venv is not a selector: with no override, the source tree's .venv owns
    the build. Keep offline/tool/sanitizer controls from the caller intact.
    """
    environment = dict(os.environ)
    raw = environment.get("UV_PROJECT_ENVIRONMENT")
    selected = Path(raw) if raw is not None else root / ".venv"
    if not selected.is_absolute():
        raise ValueError(
            "UV_PROJECT_ENVIRONMENT must be an absolute venv path, "
            f"got {raw!r}"
        )
    environment["UV_PROJECT_ENVIRONMENT"] = str(selected.resolve())
    return environment


def _same_interpreter(left: str, right: str) -> bool:
    """Allow executable aliases within one venv, never two venvs of one CPython."""
    first, second = Path(left), Path(right)
    return (
        first.is_absolute() and second.is_absolute()
        and first.parent.resolve() == second.parent.resolve()
        and first.resolve() == second.resolve()
    )


def _require_project_interpreter(root: Path) -> str:
    environment = project_environment(root)
    venv = Path(environment["UV_PROJECT_ENVIRONMENT"])
    python = venv / "bin" / "python"
    if (not python.is_file() or Path(sys.prefix).resolve() != venv
            or not _same_interpreter(sys.executable, str(python))):
        raise ValueError(
            f"selected uv environment is {venv}, but dev.py is running "
            f"with python={sys.executable}, prefix={sys.prefix}; run "
            "tools/dev.py through `uv run --frozen --group native --no-sync "
            "python` from the source tree with the same absolute "
            "UV_PROJECT_ENVIRONMENT (bootstrap first if it is absent)"
        )
    # Do not resolve the executable symlink: its venv directory determines
    # site-packages even when several venvs use the same base interpreter.
    return str(Path(sys.executable).absolute())


def _read_build_context(build: Path) -> dict:
    context_path = build / "native_check_context.json"
    if not context_path.is_file():
        candidates = sorted((build / "native_check_context").glob("*.json"))
        if len(candidates) != 1:
            raise ValueError(
                f"native build context is unavailable or ambiguous in {build}; "
                "explicitly configure this selected build first"
            )
        context_path = candidates[0]
    try:
        context = json.loads(context_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {context_path}: {error}") from error
    if not isinstance(context, dict) or context.get("schema_version") not in (1, 2):
        raise ValueError(f"unsupported native build context in {context_path}")
    return context


def _check_build_cache(root: Path, build: Path, *, optional: bool = False) -> None:
    """Reject copied roots/venvs before allowing CMake to configure or build."""
    cache_file = build / "CMakeCache.txt"
    if not cache_file.is_file():
        if optional:
            return
        raise ValueError(
            f"no CMakeCache.txt in selected build {build}; run "
            "tools/dev.py configure --build <dir> --config Release first"
        )
    cache = {}
    for line in cache_file.read_text(encoding="utf-8").splitlines():
        if line.startswith(("#", "//")) or "=" not in line:
            continue
        name, value = line.split("=", 1)
        cache[name.split(":", 1)[0]] = value
    for field, expected in (("CMAKE_HOME_DIRECTORY", root / "native"),
                            ("CMAKE_CACHEFILE_DIR", build)):
        raw = cache.get(field)
        if not raw or not Path(raw).is_absolute() or Path(raw).resolve() != expected.resolve():
            raise ValueError(
                f"stale or relocated build cache {cache_file}: {field}="
                f"{raw!r}, expected {expected}; configure a fresh build "
                "directory in the target source environment"
            )
    python = cache.get("Python_EXECUTABLE") or cache.get("_Python_EXECUTABLE")
    if not python or not _same_interpreter(python, sys.executable):
        raise ValueError(
            f"configured Python executable {python!r} differs from the "
            f"selected interpreter {sys.executable!r}; configure a fresh "
            f"build directory (selected={build}) in the correct uv environment"
        )


def _validate_install_context(root: Path, build: Path, context: dict) -> None:
    """Read-only install checks; old contexts need an explicit reconfigure."""
    _check_build_cache(root, build)
    python = context.get("python")
    if not isinstance(python, dict) or not python.get("soabi"):
        raise ValueError(
            "native build context has no recorded Python SOABI/install "
            "identity; reconfigure this build to refresh its diagnostic "
            "fields (older schema-2 contexts remain valid for existing readers)"
        )
    for field, expected in (("source_directory", root / "native"),
                            ("build_directory", build)):
        raw = context.get(field)
        if not isinstance(raw, str) or not Path(raw).is_absolute():
            raise ValueError(f"build context lacks absolute {field}; reconfigure")
        if Path(raw).resolve() != expected.resolve():
            raise ValueError(
                f"stale build context {field}={raw!r}, expected {expected}; "
                "rebuild relocated source in a fresh build directory"
            )
    for selected in (context.get("python_executable"), python.get("executable")):
        if not isinstance(selected, str) or not _same_interpreter(selected, sys.executable):
            raise ValueError(
                f"configured Python executable {selected!r} differs from "
                f"the selected interpreter {sys.executable!r}"
            )
    if python["soabi"] != sysconfig.get_config_var("SOABI"):
        raise ValueError(
            f"configured Python SOABI={python['soabi']!r} differs from "
            f"the selected interpreter SOABI={sysconfig.get_config_var('SOABI')!r}"
        )
    if python.get("version") != platform.python_version():
        raise ValueError(
            f"configured Python version={python.get('version')!r} differs "
            f"from selected {platform.python_version()!r}; reconfigure/build"
        )
    recorded_platform = context.get("platform")
    if not isinstance(recorded_platform, dict):
        raise ValueError("build context has no platform record; reconfigure")
    aliases = {"arm64": "aarch64", "amd64": "x86_64"}
    recorded_machine = str(recorded_platform.get("machine", "")).lower()
    actual_machine = platform.machine().lower()
    if (recorded_platform.get("system") != platform.system()
            or aliases.get(recorded_machine, recorded_machine)
            != aliases.get(actual_machine, actual_machine)):
        raise ValueError(
            f"configured platform {recorded_platform!r} differs from "
            f"{platform.system()}/{platform.machine()}; rebuild on this host"
        )
    for field in ("library_paths", "required_files"):
        paths = context.get(field)
        if not isinstance(paths, list) or not paths:
            raise ValueError(f"build context has no {field} dependency record")
        for raw in paths:
            if not isinstance(raw, str) or not Path(raw).is_absolute() or not Path(raw).exists():
                raise ValueError(
                    f"recorded {field} dependency is missing or not absolute: "
                    f"{raw!r}; restore the selected environment and rebuild"
                )


def _run(argv: list[str], *, cwd: Path, env: dict | None = None) -> int:
    """The single spawn point: argument array, explicit cwd, status out."""
    print(f"+ {shlex.join([str(token) for token in argv])}", file=sys.stderr)
    completed = subprocess.run(
        [str(token) for token in argv], cwd=cwd, env=env, check=False
    )
    return completed.returncode


def _capture(argv: list[str], *, cwd: Path, timeout: int = 30,
             env: dict[str, str] | None = None) -> dict:
    """Read-only probe for doctor; failures are evidence, not exceptions."""
    try:
        completed = subprocess.run(
            [str(token) for token in argv],
            cwd=cwd,
            env=project_environment(cwd) if env is None else env,
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
    try:
        _require_project_interpreter(root)
    except ValueError as error:
        return _check("interpreter", "fail", f"{evidence}; {error}",
                      "use the source project's selected editable uv environment")
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


_DOCTOR_DEPENDENCY_CODE = r'''
import ctypes
import json
import platform
from pathlib import Path
import sys
import sysconfig
from bike_sim.native import artifact
from tools.native_dependencies import select_mujoco_library

root = Path(sys.argv[1])
if artifact.native_source_root() != root:
    raise ValueError(f"imported Python source belongs to {artifact.native_source_root()}, expected {root}")
artifact.check_dependencies()
import mujoco
if mujoco.mj_versionString() != '3.12.0':
    raise ValueError(f"loaded MuJoCo must be 3.12.0, got {mujoco.mj_versionString()}")
library = select_mujoco_library(Path(mujoco.__file__).resolve().parent,
                                platform.system(), '3.12.0')
handle = ctypes.CDLL(str(library))
private_symbols = ['_mjPRIVATE_setTlsLogHandler', '_mjPRIVATE_getGlobalLogHandler', 'mj_makeRawData']
for symbol in private_symbols:
    if getattr(handle, symbol, None) is None:
        raise ValueError(f"MuJoCo private ABI symbol {symbol} missing from {library}")
document = {'schema_version': 1,
            'python': {'executable': sys.executable, 'version': platform.python_version(),
                       'soabi': sysconfig.get_config_var('SOABI')},
            'mujoco': {'version': mujoco.mj_versionString(), 'library_path': str(library),
                       'private_symbols': private_symbols}}
if sys.argv[2]:
    selected = Path(sys.argv[2])
    module = artifact.load_native_extension(selected)
    identity = module.runtime_identity()
    native_library = Path(identity['mujoco_library_path']).resolve(strict=True)
    if native_library != library.resolve(strict=True):
        raise ValueError(f"native MuJoCo image {native_library} differs from Python image {library}")
    execution = artifact.execution_provenance('native', artifact=module)
    document['numerics'] = execution['build_context']['numerics']
    document['native'] = {'artifact': str(Path(module.__file__).resolve()),
                          'source_root': str(root),
                          'compiled_source_sha256': identity['native_source_sha256'],
                          'source_sha256': execution['native_source_sha256'],
                          'extension_sha256': execution['extension_sha256']}
else:
    # Runtime provider introspection has no compiler/linker subprocess.
    # The full ABI compile/link qualification belongs to configure.
    document['numerics'], _ = artifact._numerics_evidence()
print(json.dumps(document, sort_keys=True))
'''


def _doctor_probe_environment(root: Path, build: Path | None) -> dict[str, str]:
    """Apply the existing sanitizer policy before importing a selected binary."""
    environment = project_environment(root)
    if build is not None:
        try:
            from tools import native_runtime
        except ModuleNotFoundError:  # direct `python tools/dev.py` invocation
            import native_runtime
        context = _read_build_context(build)
        native_runtime._cross_check_build_cache(build, context)
        environment = native_runtime.runtime_environment(context, environment)
    # dyld removes DYLD_INSERT_LIBRARIES after loading the parent runtime.
    # Preserve the selected-runtime evidence and reinject it for this direct
    # Python child, including --numerics without a selected build.
    selected = environment.get("NATIVE_TEST_SANITIZER_RUNTIME")
    if selected is not None:
        runtime = Path(selected)
        if not runtime.is_absolute() or not runtime.is_file():
            raise ValueError(f"selected sanitizer runtime is not an absolute file: {selected!r}")
        variable = "DYLD_INSERT_LIBRARIES" if sys.platform == "darwin" else "LD_PRELOAD"
        previous = environment.get(variable, "")
        if selected not in previous.split(os.pathsep):
            environment[variable] = selected if not previous else f"{selected}{os.pathsep}{previous}"
    return environment


def _doctor_dependency_probe(root: Path, build: Path | None = None) -> tuple[dict, dict | None]:
    """Measure the live runtime in a fresh interpreter, without compiling.

    T05's full probe performs compile/link/run ABI controls, so doctor must
    not invoke that CLI or discover_numpy_provider(). It uses the already
    compiled identity and runtime provider introspection instead.
    """
    script = root / DEPENDENCY_PROBE
    if not script.is_file():
        return _check(
            "dependency_probe", "fail",
            f"unavailable: {DEPENDENCY_PROBE.as_posix()} source is missing",
            "restore the complete source tree before native doctor checks",
        ), None
    try:
        environment = _doctor_probe_environment(root, build)
    except (OSError, ValueError) as error:
        return _check(
            "dependency_probe", "fail", f"native runtime preflight failed: {error}",
            "restore the selected build's recorded sanitizer runtime and context",
        ), None
    probe = _capture(
        [sys.executable, "-c", _DOCTOR_DEPENDENCY_CODE, str(root),
         str(build) if build is not None else ""],
        cwd=root, timeout=180, env=environment,
    )
    if probe.get("returncode") != 0:
        detail = (probe.get("stderr") or "").strip() \
            or probe.get("error") or f"exit {probe.get('returncode')}"
        return _check(
            "dependency_probe", "fail", f"probe failed: {detail}",
            "restore the pinned environment, then explicitly configure/build "
            "in that source environment; doctor never starts compilation",
        ), None
    try:
        document = json.loads(probe["stdout"])
    except (KeyError, json.JSONDecodeError) as exc:
        return _check(
            "dependency_probe", "fail",
            f"probe did not emit schema JSON on stdout: {exc}",
            "inspect the read-only runtime probe's full diagnostic",
        ), None
    if not isinstance(document, dict) or document.get("schema_version") != SCHEMA_VERSION:
        return _check(
            "dependency_probe", "fail",
            "unsupported runtime dependency probe schema",
            "expected schema_version=1",
        ), None
    mujoco = document.get("mujoco")
    python = document.get("python")
    numerics = document.get("numerics")
    if (not isinstance(mujoco, dict) or mujoco.get("version") != "3.12.0"
            or not isinstance(mujoco.get("library_path"), str)
            or mujoco.get("private_symbols") != ["_mjPRIVATE_setTlsLogHandler",
                                                 "_mjPRIVATE_getGlobalLogHandler", "mj_makeRawData"]
            or not isinstance(python, dict)
            or not isinstance(python.get("executable"), str)
            or not _same_interpreter(python["executable"], sys.executable)
            or python.get("soabi") != sysconfig.get_config_var("SOABI")
            or not isinstance(numerics, dict)
            or numerics.get("backend") not in {"ACCELERATE", "NUMPY_OPENBLAS"}):
        return _check(
            "dependency_probe", "fail", "runtime probe returned incomplete or mismatched evidence",
            "inspect the read-only runtime probe's full diagnostic",
        ), None
    check = _check(
        "dependency_probe", "pass",
        f"mujoco {mujoco.get('version')} at {mujoco.get('library_path')}; "
        "private ABI symbols resolved in the selected interpreter",
        None,
    )
    check["details"] = document
    return check, document


def _doctor_build_context(root: Path, build: Path) -> dict:
    try:
        context = _read_build_context(build)
        _validate_install_context(root, build, context)
    except (OSError, ValueError) as error:
        return _check(
            "build_context", "fail", str(error),
            "configure/build from the selected source environment; use a "
            "fresh build directory after relocation or environment changes",
        )
    return _check(
        "build_context", "pass",
        f"source={context['source_directory']}, build={context['build_directory']}, "
        f"python={context['python']['executable']}, soabi={context['python']['soabi']}; "
        "recorded dependency files exist",
    )


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
    """Live runtime provider evidence; configure owns compile/link controls."""
    if probe_document is None:
        return _check(
            "numerical_profile", "fail",
            "unavailable: the read-only runtime dependency probe did not "
            "establish the numerical profile",
            "resolve the dependency_probe failure and rerun doctor",
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
    native_ready = checks[0]["status"] == "pass"
    if build is not None:
        build = Path(build)
        artifact_check = _doctor_selected_artifact(build)
        context_check = _doctor_build_context(root, build)
        checks.extend((artifact_check, context_check))
        native_ready = native_ready and artifact_check["status"] == context_check["status"] == "pass"
    if build is not None or numerics:
        if native_ready:
            probe_check, probe_document = _doctor_dependency_probe(root, build)
        else:
            probe_check = _check(
                "dependency_probe", "fail",
                "unavailable: selected interpreter/artifact/build context failed preflight; no import attempted",
                "resolve the interpreter/selected_artifact/build_context failures and rerun doctor",
            )
        checks.append(probe_check)
    if build is not None or numerics:
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
    environment = project_environment(root)
    if not args.offline:
        argv = ["uv", "sync", "--locked", "--group", "native", "--group", "dev"]
        if args.python is not None:
            argv += ["--python", args.python]
        return _run(argv, cwd=root, env=environment)
    wheelhouse = _cli_path(args.wheelhouse)
    if not wheelhouse.is_dir():
        print(
            f"bootstrap --wheelhouse is not a directory: {wheelhouse}",
            file=sys.stderr,
        )
        return 2
    # `uv sync --locked` cannot consume a flat wheelhouse: locked wheels pin
    # their recorded registry URLs, which are unreachable offline. The
    # offline-equivalent sync is the frozen requirements export installed by
    # `uv pip` against --find-links with enforced hashes, followed by the
    # editable project whose PEP 517 backend resolves from the wheelhouse.
    venv = Path(environment["UV_PROJECT_ENVIRONMENT"])
    python = venv / "bin" / "python"
    if not (venv / "pyvenv.cfg").is_file():
        argv = ["uv", "venv", "--offline", str(venv)]
        if args.python is not None:
            argv += ["--python", args.python]
        status = _run(argv, cwd=root, env=environment)
        if status != 0:
            return status
    requirements = root.parent / "requirements.txt"
    with tempfile.TemporaryDirectory(prefix="bikesim-bootstrap-") as work:
        if requirements.is_file():
            export = requirements
        else:
            export = Path(work) / "requirements.txt"
            status = _run(
                ["uv", "export", "--frozen", "--offline", "--no-emit-project",
                 "--group", "native", "--group", "dev", "-o", str(export)],
                cwd=root, env=environment)
            if status != 0:
                return status
        common = ["--python", str(python), "--offline", "--no-index",
                  "--find-links", str(wheelhouse)]
        status = _run(["uv", "pip", "sync", "--require-hashes", *common,
                       str(export)], cwd=root, env=environment)
        if status != 0:
            return status
        return _run(
            ["uv", "pip", "install", "--no-deps", *common, "--editable",
             str(root)],
            cwd=root, env=environment)


def _cmd_configure(args, root: Path) -> int:
    build = resolve_build_path(root, args.build)
    python = _require_project_interpreter(root)
    _check_build_cache(root, build, optional=True)
    configure = build_commands(
        root, build, args.config, args.compiler, python_executable=python,
    )[0]
    return _run(configure, cwd=root, env=project_environment(root))


def _cmd_build(args, root: Path) -> int:
    build = resolve_build_path(root, args.build)
    _require_project_interpreter(root)
    _check_build_cache(root, build)
    return _run(_compile_command(build), cwd=root, env=project_environment(root))


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
    _require_project_interpreter(root)
    # native/full preflight configures a missing build itself; existing
    # caches must still belong to the selected source/interpreter. quick
    # retains its Python-only meaning and needs no native cache.
    if args.profile != "quick":
        _check_build_cache(root, build, optional=True)
    env = project_environment(root)
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
    try:
        # Reject an ambiguous relative installer selection before a probe or
        # mutating subprocess can choose a different environment.
        project_environment(root)
        return handlers[args.command](args, root)
    except (OSError, ValueError) as error:
        print(f"dev.py: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
