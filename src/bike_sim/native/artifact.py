"""Application-side native artifact selection and verified import.

``BIKE_NATIVE_BUILD_PATH`` (absolute directory) selects the artifact when
set; otherwise the checkout's ``native/build/release`` is the default. The
import verifies the loaded module file is the selected artifact and rejects
a pre-imported ``bike_native`` module that came from anywhere else
(design section 7). Pinned runtime dependencies are checked before the
extension is loaded.
"""
from collections.abc import Mapping
import importlib
import importlib.machinery
import importlib.metadata
import os
from pathlib import Path
import sys
import tomllib
from types import ModuleType

SELECTOR_ENV = 'BIKE_NATIVE_BUILD_PATH'
EXTENSION_MODULE = 'bike_native'

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_BUILD = _REPO_ROOT / 'native' / 'build' / 'release'
# Mirror of the == pins in pyproject.toml's 'native' dependency group;
# required_dependencies() prefers the live file and falls back to this in
# environments where the checkout layout is absent.
_FALLBACK_DEPENDENCIES = {'mujoco': '3.12.0'}


def selected_build_dir(environ: Mapping[str, str] | None = None) -> Path:
    """Absolute artifact directory from the application selector or default."""
    env = os.environ if environ is None else environ
    raw = env.get(SELECTOR_ENV)
    if raw is None:
        return _DEFAULT_BUILD
    candidate = Path(raw)
    if not candidate.is_absolute():
        raise ValueError(f'{SELECTOR_ENV} must be an absolute path, got {raw!r}')
    return candidate.resolve()


def extension_artifacts(build_dir: Path) -> list[Path]:
    """Every importable bike_native binary in the selected directory."""
    return [
        candidate
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (candidate := build_dir / f'{EXTENSION_MODULE}{suffix}').is_file()
    ]


def required_dependencies() -> dict[str, str]:
    """The == pins of pyproject's 'native' dependency group."""
    pyproject = _REPO_ROOT / 'pyproject.toml'
    try:
        document = tomllib.loads(pyproject.read_text(encoding='utf-8'))
    except (OSError, tomllib.TOMLDecodeError):
        return dict(_FALLBACK_DEPENDENCIES)
    entries = document.get('dependency-groups', {}).get('native', [])
    pins = {}
    for entry in entries:
        name, separator, version = str(entry).partition('==')
        if separator:
            pins[name.strip()] = version.strip()
    return pins or dict(_FALLBACK_DEPENDENCIES)


def check_dependencies(declared: Mapping[str, str] | None = None, *,
                       installed: Mapping[str, str] | None = None) -> dict[str, str]:
    """Reject a runtime environment whose pinned native deps do not match.

    ``declared`` maps package name to required version (defaults to the
    pinned 'native' dependency group); ``installed`` maps name to the actual
    version (defaults to importlib.metadata). A missing package is a
    mismatch, and every failure names the package.
    """
    required = required_dependencies() if declared is None else dict(declared)
    if installed is None:
        resolved = {}
        for name in required:
            try:
                resolved[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                resolved[name] = None
        actual_map = resolved
    else:
        actual_map = dict(installed)
    for name, expected in required.items():
        actual = actual_map.get(name)
        if actual != expected:
            raise ValueError(
                f'native dependency version mismatch: {name} requires '
                f'{expected!r}, installed {actual!r}')
    return actual_map


def _module_path(module: ModuleType) -> Path | None:
    location = getattr(module, '__file__', None)
    return Path(location).resolve() if isinstance(location, str) else None


def load_native_extension(build_dir: Path | None = None) -> ModuleType:
    """Import ``bike_native`` only from the selected build directory."""
    if build_dir is None:
        build = selected_build_dir()
    else:
        build = Path(build_dir)
        if not build.is_absolute():
            raise ValueError(
                f'build_dir must be an absolute path, got {build_dir!r}')
        build = build.resolve()
    artifacts = extension_artifacts(build)
    if len(artifacts) != 1:
        raise RuntimeError(
            f'expected exactly one {EXTENSION_MODULE} extension in {build}, '
            f'found {len(artifacts)}')
    expected = artifacts[0].resolve()
    check_dependencies()

    imported = sys.modules.get(EXTENSION_MODULE)
    if imported is not None:
        actual = _module_path(imported)
        if actual != expected:
            raise RuntimeError(
                f'pre-imported {EXTENSION_MODULE} module does not match '
                f'selected build: selected={build}, expected={expected}, '
                f'imported={actual}')
        return imported

    sys.path[:] = [
        entry
        for entry in sys.path
        if not entry or Path(entry).resolve() != build
    ]
    sys.path.insert(0, str(build))
    module = importlib.import_module(EXTENSION_MODULE)
    actual = _module_path(module)
    if actual != expected:
        raise RuntimeError(
            f'native artifact mismatch: selected={build}, '
            f'expected={expected}, imported={actual}')
    return module
