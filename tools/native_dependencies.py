"""Probe the selected Python interpreter for native-build dependencies.

The native build must compile and link against the MuJoCo/nanobind that belong
to the *selected* interpreter (``Python_EXECUTABLE`` / the nanobind linkage
interpreter) — never whatever python happens to be on ``PATH``. This module is
therefore split in two:

- the importable API (``select_mujoco_library``, ``validate_dependencies``,
  ``probe_dependencies``) is pure stdlib and runs in any interpreter;
- ``--child-probe`` runs *inside* the selected interpreter, imports
  mujoco/nanobind there, and prints the schema-1 document on stdout only.

Consumers: ``native/cmake/BikeDependencies.cmake`` (configure-time discovery)
and ``tools/dev.py doctor`` (``dependency_probe`` check, which ``json.loads``es
stdout verbatim — keep stdout strictly JSON).

Dependency document, ``schema_version=1``::

    {
      "schema_version": 1,
      "platform":  {"system": "Linux"|"Darwin", "machine": str,
                    "libc": str|null},
      "python":    {"executable": abs, "version": str, "soabi": str,
                    "include_dir": abs},
      "mujoco":    {"version": "3.12.0", "include_dir": abs,
                    "library_path": abs},
      "nanobind":  {"cmake_dir": abs, "include_dir": abs},
      "numerics":  null | object   # populated by T06
    }

Every filesystem path is absolute and exists; ``mujoco.library_path`` resolves
to the one exact platform engine image (see ``select_mujoco_library``).

CLI:

    python tools/native_dependencies.py --python PATH [--output PATH]

Without ``--output`` the document is written to stdout; diagnostics and the
named dependency errors go to stderr with a nonzero exit.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import sysconfig
from pathlib import Path

REQUIRED_MUJOCO_VERSION = "3.12.0"
SCHEMA_VERSION = 1
_PROBE_TIMEOUT = 120

# Exact, platform-specific candidate filenames only — never a glob and never
# an unrestricted system library search. The first entry is the canonical
# versioned image; the remaining entries are accepted *alias* names that must
# resolve (realpath) to a file still named exactly like the canonical image,
# so an alias can never smuggle in an unversioned or wrong-version library.
_MUJOCO_LIBRARY_CANDIDATES = {
    "Darwin": ("libmujoco.3.12.0.dylib", "libmujoco.dylib"),
    "Linux": ("libmujoco.so.3.12.0", "libmujoco.so"),
}

_SELF = Path(__file__).resolve()


def select_mujoco_library(directory: Path, system: str, version: str) -> Path:
    """Return the one MuJoCo ``version`` engine image inside ``directory``.

    ``directory`` is the installed package directory of the selected
    interpreter's ``mujoco`` package. Candidates are the exact
    platform-specific filenames in ``_MUJOCO_LIBRARY_CANDIDATES``; every hit is
    realpath-resolved, de-duplicated, and must still be named like the
    canonical versioned image. Anything other than exactly one resolved image
    — none, conflicting images, or an unversioned real file — is a named
    ``ValueError``.
    """
    if version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found {version}"
        )
    if system not in _MUJOCO_LIBRARY_CANDIDATES:
        raise ValueError(f"unsupported native platform: {system}")
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(
            f"MuJoCo {version} package directory is missing: {directory}"
        )
    canonical, *aliases = _MUJOCO_LIBRARY_CANDIDATES[system]
    resolved_hits: list[Path] = []
    for name in (canonical, *aliases):
        candidate = directory / name
        if candidate.is_file():
            resolved_hits.append(candidate.resolve())
    misnamed = [path for path in resolved_hits if path.name != canonical]
    if misnamed:
        raise ValueError(
            f"{directory} contains a library that cannot prove the pinned "
            f"MuJoCo {version} engine image: "
            f"{sorted(str(path) for path in misnamed)}"
        )
    found = set(resolved_hits)
    if len(found) != 1:
        raise ValueError(
            f"expected exactly one MuJoCo {version} library in {directory}; "
            f"found {sorted(str(path) for path in found)}"
        )
    return found.pop()


# --------------------------------------------------------------------------
# Document validation (pure stdlib; runs in any interpreter)
# --------------------------------------------------------------------------


def _require_object(document: dict, key: str) -> dict:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ValueError(
            f"dependency probe field '{key}' must be an object, "
            f"found {value!r}"
        )
    return value


def _require_string(record: dict, key: str, label: str) -> str:
    value = record.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(
            f"dependency probe field '{label}' must be a nonempty string, "
            f"found {value!r}"
        )
    return value


def _require_path(
    record: dict,
    key: str,
    label: str,
    *,
    directory: bool = False,
    executable: bool = False,
) -> Path:
    value = _require_string(record, key, label)
    path = Path(value)
    if not path.is_absolute():
        raise ValueError(
            f"dependency probe field '{label}' is not an absolute path: {value}"
        )
    if directory:
        if not path.is_dir():
            raise ValueError(
                f"dependency probe field '{label}' does not name an existing "
                f"directory: {value}"
            )
    elif not path.is_file():
        raise ValueError(
            f"dependency probe field '{label}' does not name an existing "
            f"file: {value}"
        )
    if executable and not os.access(path, os.X_OK):
        raise ValueError(
            f"dependency probe field '{label}' is not executable: {value}"
        )
    return path


def validate_dependencies(value: dict) -> dict:
    """Validate a schema-1 dependency document; return it unchanged.

    Fail-closed: wrong types, non-absolute or nonexistent paths, a
    non-executable interpreter, a foreign platform library name, or a MuJoCo
    version other than the 3.12.0 pin are all named ``ValueError``s. The
    ``numerics`` record is T06 scope and only type-checked here.
    """
    if not isinstance(value, dict):
        raise ValueError(
            f"dependency probe document must be a JSON object, found {value!r}"
        )
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported dependency probe schema_version "
            f"{value.get('schema_version')!r}; expected {SCHEMA_VERSION}"
        )

    platform_record = _require_object(value, "platform")
    system = _require_string(platform_record, "system", "platform.system")
    _require_string(platform_record, "machine", "platform.machine")
    libc = platform_record.get("libc")
    if libc is not None and not isinstance(libc, str):
        raise ValueError(
            f"dependency probe field 'platform.libc' must be a string or "
            f"null, found {libc!r}"
        )
    if system not in _MUJOCO_LIBRARY_CANDIDATES:
        raise ValueError(f"unsupported native platform: {system}")

    python_record = _require_object(value, "python")
    _require_path(
        python_record, "executable", "python.executable", executable=True
    )
    _require_string(python_record, "version", "python.version")
    _require_string(python_record, "soabi", "python.soabi")
    _require_path(
        python_record, "include_dir", "python.include_dir", directory=True
    )

    mujoco_record = _require_object(value, "mujoco")
    mujoco_version = _require_string(mujoco_record, "version", "mujoco.version")
    if mujoco_version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found "
            f"{mujoco_version}"
        )
    _require_path(
        mujoco_record, "include_dir", "mujoco.include_dir", directory=True
    )
    library = _require_path(
        mujoco_record, "library_path", "mujoco.library_path"
    )
    canonical = _MUJOCO_LIBRARY_CANDIDATES[system][0]
    if library.name != canonical:
        raise ValueError(
            f"dependency probe field 'mujoco.library_path' must resolve to "
            f"the {system} {REQUIRED_MUJOCO_VERSION} image {canonical}, found "
            f"{library}"
        )

    nanobind_record = _require_object(value, "nanobind")
    _require_path(
        nanobind_record, "cmake_dir", "nanobind.cmake_dir", directory=True
    )
    _require_path(
        nanobind_record, "include_dir", "nanobind.include_dir", directory=True
    )

    numerics = value.get("numerics")
    if numerics is not None and not isinstance(numerics, dict):
        raise ValueError(
            f"dependency probe field 'numerics' must be an object or null, "
            f"found {numerics!r}"
        )
    return value


# --------------------------------------------------------------------------
# The child probe — runs inside the SELECTED interpreter
# --------------------------------------------------------------------------


def _child_probe() -> dict:
    """Collect the dependency document from *this* interpreter.

    Imports mujoco/nanobind here (the selected interpreter), uses sysconfig
    for SOABI/include data, importlib-provided package roots, and nanobind's
    own ``cmake_dir()``/``include_dir()`` queries. Everything raised becomes a
    named stderr diagnostic; only the JSON document reaches stdout.
    """
    system = platform.system()
    libc_name, libc_version = platform.libc_ver()
    libc = f"{libc_name} {libc_version}".strip() or None

    try:
        import mujoco
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import mujoco: {exc}"
        ) from exc
    mujoco_version = str(mujoco.__version__)
    if mujoco_version != REQUIRED_MUJOCO_VERSION:
        raise ValueError(
            f"native requires MuJoCo {REQUIRED_MUJOCO_VERSION}, found "
            f"{mujoco_version} in {sys.executable}"
        )
    mujoco_package = Path(mujoco.__file__).resolve().parent
    mujoco_include = mujoco_package / "include"
    if not mujoco_include.is_dir():
        raise ValueError(
            f"mujoco {mujoco_version} package has no include directory: "
            f"{mujoco_include}"
        )
    library = select_mujoco_library(mujoco_package, system, mujoco_version)

    try:
        import nanobind
    except ImportError as exc:
        raise ValueError(
            f"selected interpreter {sys.executable} cannot import nanobind: "
            f"{exc}"
        ) from exc
    nb_cmake_dir = Path(nanobind.cmake_dir())
    nb_include_dir = Path(nanobind.include_dir())
    for label, path in (
        ("nanobind.cmake_dir", nb_cmake_dir),
        ("nanobind.include_dir", nb_include_dir),
    ):
        if not path.is_dir():
            raise ValueError(f"{label} is not an existing directory: {path}")

    soabi = sysconfig.get_config_var("SOABI")
    python_include = sysconfig.get_path("include")
    if not soabi or not python_include:
        raise ValueError(
            f"selected interpreter {sys.executable} reports incomplete "
            f"sysconfig data: soabi={soabi!r} include={python_include!r}"
        )

    return {
        "schema_version": SCHEMA_VERSION,
        "platform": {
            "system": system,
            "machine": platform.machine(),
            "libc": libc,
        },
        "python": {
            "executable": str(Path(sys.executable)),
            "version": platform.python_version(),
            "soabi": soabi,
            "include_dir": str(Path(python_include).resolve()),
        },
        "mujoco": {
            "version": mujoco_version,
            "include_dir": str(mujoco_include),
            "library_path": str(library),
        },
        "nanobind": {
            "cmake_dir": str(nb_cmake_dir),
            "include_dir": str(nb_include_dir),
        },
        "numerics": None,  # populated by T06's provider qualification
    }


def _require_interpreter(python: Path) -> Path:
    """The selected interpreter must be an absolute, existing executable.

    The path is *not* resolved: a venv interpreter discovers ``pyvenv.cfg``
    and its site-packages through the exact path it was invoked as, so
    executing the realpath'd base binary would silently drop the environment.
    """
    candidate = Path(python).expanduser()
    if not candidate.is_absolute():
        raise ValueError(
            f"selected interpreter must be an absolute path, found {python!r}"
        )
    if not candidate.is_file():
        raise ValueError(f"selected interpreter does not exist: {candidate}")
    if not os.access(candidate, os.X_OK):
        raise ValueError(f"selected interpreter is not executable: {candidate}")
    return candidate


def probe_dependencies(python: Path) -> dict:
    """Run ``--child-probe`` inside the selected interpreter and validate.

    One interpreter contract: the same ``python`` drives the subprocess and is
    the subject; the reported ``python.executable`` must resolve to the same
    interpreter that was invoked. A nonzero child exit or a non-JSON stdout is
    a named failure — the child's stderr is reproduced verbatim.
    """
    interpreter = _require_interpreter(Path(python))
    try:
        completed = subprocess.run(
            [str(interpreter), str(_SELF), "--child-probe"],
            capture_output=True,
            text=True,
            timeout=_PROBE_TIMEOUT,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(
            f"dependency probe could not run in {interpreter}: {exc}"
        ) from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "(no stderr)"
        raise ValueError(
            f"dependency probe failed in {interpreter} "
            f"(exit {completed.returncode}): {detail}"
        )
    try:
        document = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"dependency probe in {interpreter} did not emit schema JSON on "
            f"stdout: {exc}"
        ) from exc
    document = validate_dependencies(document)
    reported = Path(document["python"]["executable"]).resolve()
    if reported != interpreter.resolve():
        raise ValueError(
            f"dependency probe interpreter mismatch: invoked "
            f"{interpreter.resolve()}, child reported {reported}"
        )
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="native_dependencies.py",
        description=(
            "Probe the selected Python interpreter for the native build's "
            "mujoco/nanobind dependencies; schema-1 JSON only on stdout."
        ),
    )
    parser.add_argument(
        "--python",
        metavar="PATH",
        help="absolute path of the selected interpreter to probe",
    )
    parser.add_argument(
        "--output",
        metavar="PATH",
        help="write the document to this file instead of stdout",
    )
    parser.add_argument(
        "--child-probe",
        action="store_true",
        help=argparse.SUPPRESS,  # internal: run inside the selected interpreter
    )
    args = parser.parse_args(argv)

    if args.child_probe:
        try:
            document = _child_probe()
        except Exception as exc:  # named diagnostic, never a traceback contract
            print(
                f"dependency probe failed in {sys.executable}: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 1
        sys.stdout.write(json.dumps(document, sort_keys=True) + "\n")
        return 0

    if not args.python:
        parser.error("--python PATH is required")
    try:
        document = probe_dependencies(Path(args.python))
    except ValueError as exc:
        print(f"dependency probe failed: {exc}", file=sys.stderr)
        return 1
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
