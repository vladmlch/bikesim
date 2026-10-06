"""Resolve and import exactly the native extension selected for this test run."""

from __future__ import annotations

from collections.abc import Mapping
import ctypes
import importlib
import importlib.machinery
import os
from pathlib import Path
import sys
from types import ModuleType


_REPO_ROOT = Path(__file__).resolve().parents[2]
_BUILD_ROOT = _REPO_ROOT / 'native' / 'build'
_LEGACY_BUILD_DIRS = {
    '': _BUILD_ROOT,
    'asan': _BUILD_ROOT / 'asan',
    'rtsan': _BUILD_ROOT / 'rtsan',
    'coverage': _BUILD_ROOT / 'coverage',
}


def selected_build(environ: Mapping[str, str]) -> Path:
    """Resolve the absolute build directory selected by the caller."""
    path_value = environ.get('NATIVE_TEST_BUILD_PATH')
    explicit_path: Path | None = None
    if path_value is not None:
        candidate = Path(path_value)
        if not candidate.is_absolute():
            raise ValueError('NATIVE_TEST_BUILD_PATH must be absolute')
        explicit_path = candidate.resolve()

    legacy_value = environ.get('NATIVE_TEST_BUILD_DIR', '')
    try:
        legacy_path = _LEGACY_BUILD_DIRS[legacy_value].resolve()
    except KeyError as error:
        valid = ', '.join(repr(value) for value in _LEGACY_BUILD_DIRS)
        raise ValueError(
            f'NATIVE_TEST_BUILD_DIR must be one of {valid}, got {legacy_value!r}'
        ) from error

    if explicit_path is not None and 'NATIVE_TEST_BUILD_DIR' in environ and explicit_path != legacy_path:
        raise ValueError(
            'conflicting native build selectors: '
            f'NATIVE_TEST_BUILD_PATH={str(explicit_path)!r} and '
            f'NATIVE_TEST_BUILD_DIR={legacy_value!r} ({legacy_path})'
        )
    if explicit_path is not None:
        return explicit_path
    return legacy_path


def _extension_artifacts(build: Path) -> list[Path]:
    return [
        candidate
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (candidate := build / f'bike_native{suffix}').is_file()
    ]


def _module_path(module: ModuleType) -> Path | None:
    location = getattr(module, '__file__', None)
    if not isinstance(location, str):
        return None
    return Path(location).resolve()


def _loaded_images() -> set[Path]:
    if sys.platform == 'darwin':
        dyld = ctypes.CDLL(None)
        image_count = dyld._dyld_image_count
        image_count.argtypes = []
        image_count.restype = ctypes.c_uint32
        image_name = dyld._dyld_get_image_name
        image_name.argtypes = [ctypes.c_uint32]
        image_name.restype = ctypes.c_char_p
        paths: set[Path] = set()
        for index in range(image_count()):
            name = image_name(index)
            if name:
                paths.add(Path(os.fsdecode(name)).resolve())
        return paths

    if sys.platform.startswith('linux'):
        paths = set()
        for line in Path('/proc/self/maps').read_text().splitlines():
            fields = line.split(maxsplit=5)
            if len(fields) == 6 and fields[5].startswith('/'):
                paths.add(Path(fields[5].removesuffix(' (deleted)')).resolve())
        return paths

    raise RuntimeError(f'cannot verify sanitizer runtime on {sys.platform}')


def verify_sanitizer_runtime(environ: Mapping[str, str] | None = None) -> Path | None:
    """Require the selected sanitizer runtime to be mapped in this Python process."""
    environment = os.environ if environ is None else environ
    runtime_value = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime_value is None:
        return None
    runtime = Path(runtime_value)
    if not runtime.is_absolute() or not runtime.is_file():
        raise RuntimeError(f'selected sanitizer runtime is not an absolute file: {runtime}')
    runtime = runtime.resolve()

    # dyld clears DYLD_INSERT_LIBRARIES after applying it, so provenance must
    # come from the images mapped in this process, not the environment value.
    loaded_images = _loaded_images()
    if runtime not in loaded_images:
        same_name = sorted(str(path) for path in loaded_images if path.name == runtime.name)
        raise RuntimeError(
            'selected sanitizer runtime is not loaded in this Python process: '
            f'selected={runtime}, loaded_with_same_name={same_name}'
        )
    return runtime


def _check_imported_module(module: ModuleType, expected: Path, build: Path) -> ModuleType:
    actual = _module_path(module)
    if actual != expected:
        raise RuntimeError(
            'native artifact mismatch: '
            f'selected={build}, expected={expected}, imported={actual}'
        )
    return module


def load_native() -> ModuleType:
    """Import ``bike_native`` only from the selected build and verify provenance."""
    verify_sanitizer_runtime()
    build = selected_build(os.environ)
    artifacts = _extension_artifacts(build)
    if len(artifacts) != 1:
        raise RuntimeError(
            f'expected exactly one bike_native extension in {build}, found {len(artifacts)}'
        )
    expected = artifacts[0].resolve()

    imported = sys.modules.get('bike_native')
    if imported is not None:
        actual = _module_path(imported)
        if actual != expected:
            raise RuntimeError(
                'pre-imported bike_native module does not match selected build: '
                f'selected={build}, expected={expected}, imported={actual}'
            )
        return imported

    build_text = str(build)
    sys.path[:] = [
        entry
        for entry in sys.path
        if not entry or Path(entry).resolve() != build
    ]
    sys.path.insert(0, build_text)
    module = importlib.import_module('bike_native')
    return _check_imported_module(module, expected, build)
