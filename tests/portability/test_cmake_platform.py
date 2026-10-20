"""Pre-project platform dispatch contract for the native build.

``native/CMakeLists.txt`` includes ``native/cmake/BikePlatform.cmake`` before
``project()``: on Darwin it runs the Apple SDK discovery module
(``native/cmake/BikeAppleSDK.cmake``); on Linux it must never invoke Apple
tooling, must reject macOS-only options and foreign toolchain selectors, and
must emit the null SDK contract (``_native_sdk_path``/``_native_sdk_requested``
empty, ``_native_sdk_source`` "not-applicable").

Each test configures a minimal probe project that includes the real module —
the native extension and its dependencies are never needed, so the suite runs
identically on both hosts. macOS SDK selector semantics are additionally
covered end-to-end by the real-configure controls in
``tests/reference/test_native_check_tools.py`` and the ``context`` sweep in
``tools/native_checks.py``.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_MODULE = REPO_ROOT / "native" / "cmake" / "BikePlatform.cmake"
APPLE_SDK_MODULE = REPO_ROOT / "native" / "cmake" / "BikeAppleSDK.cmake"

CMAKE = shutil.which("cmake")

pytestmark = pytest.mark.skipif(CMAKE is None, reason="cmake executable unavailable")

PROBE_TIMEOUT = 300


def _write_probe(source: Path) -> None:
    """Minimal project: include the dispatch module pre-project(), then dump
    the SDK contract variables into probe_vars_pre.txt (the module boundary —
    the caller's requested value stays separate from the resolved path) and
    probe_vars.txt (post-project(), after CMake's own platform init)."""
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.27)\n"
        f'include("{PLATFORM_MODULE}")\n'
        "get_property(_sysroot_cache_pre CACHE CMAKE_OSX_SYSROOT PROPERTY VALUE)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/probe_vars_pre.txt"\n'
        '  "BIKE_TARGET_SYSTEM=${BIKE_TARGET_SYSTEM}\\n"\n'
        '  "sdk_path=${_native_sdk_path}\\n"\n'
        '  "sdk_requested=${_native_sdk_requested}\\n"\n'
        '  "sdk_source=${_native_sdk_source}\\n"\n'
        '  "sysroot_var=${CMAKE_OSX_SYSROOT}\\n"\n'
        '  "sysroot_cache=${_sysroot_cache_pre}\\n")\n'
        "project(platform_probe CXX)\n"
        "get_property(_sysroot_cache CACHE CMAKE_OSX_SYSROOT PROPERTY VALUE)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/probe_vars.txt"\n'
        '  "BIKE_TARGET_SYSTEM=${BIKE_TARGET_SYSTEM}\\n"\n'
        '  "sdk_path=${_native_sdk_path}\\n"\n'
        '  "sdk_requested=${_native_sdk_requested}\\n"\n'
        '  "sdk_source=${_native_sdk_source}\\n"\n'
        '  "sysroot_var=${CMAKE_OSX_SYSROOT}\\n"\n'
        '  "sysroot_cache=${_sysroot_cache}\\n"\n'
        '  "cxx_compiler=${CMAKE_CXX_COMPILER}\\n"\n'
        '  "crosscompiling=${CMAKE_CROSSCOMPILING}\\n")\n'
    )


def _configure(source: Path, build: Path, *options: str,
               env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [CMAKE, "-S", str(source), "-B", str(build), *options],
        env=env,
        capture_output=True,
        text=True,
        timeout=PROBE_TIMEOUT,
    )


def _probe_vars(build: Path, filename: str = "probe_vars.txt") -> dict[str, str]:
    values: dict[str, str] = {}
    for line in (build / filename).read_text().splitlines():
        key, _, value = line.partition("=")
        values[key] = value
    return values


def _write_executable(path: Path, body: str) -> Path:
    path.write_text(body)
    path.chmod(0o755)
    return path


# --------------------------------------------------------------------------
# Linux contract
# --------------------------------------------------------------------------


@pytest.mark.skipif(sys.platform != "linux", reason="Linux negative Apple-tool control")
def test_linux_configure_never_invokes_apple_tools(tmp_path: Path) -> None:
    fake = tmp_path / "bin"
    fake.mkdir()
    marker = tmp_path / "apple-invoked"
    for name in ("xcrun", "xcode-select"):
        _write_executable(
            fake / name,
            f'#!/bin/sh\ntouch "{marker}"\nexit 91\n',
        )
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source,
        tmp_path / "build",
        env={**os.environ, "PATH": str(fake) + os.pathsep + os.environ["PATH"]},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert not marker.exists()
    values = _probe_vars(tmp_path / "build")
    assert values["BIKE_TARGET_SYSTEM"] == "Linux"
    # Null SDK contract: no fake "/" sysroot is ever substituted on Linux.
    assert values["sdk_path"] == ""
    assert values["sdk_requested"] == ""
    assert values["sdk_source"] == "not-applicable"
    assert values["sysroot_cache"] == ""
    assert values["crosscompiling"] == "FALSE"


@pytest.mark.skipif(sys.platform != "linux", reason="Linux Apple-option rejection")
def test_linux_rejects_osx_sysroot_option(tmp_path: Path) -> None:
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", f"-DCMAKE_OSX_SYSROOT={tmp_path}"
    )

    assert result.returncode != 0
    output = result.stdout + result.stderr
    assert "CMAKE_OSX_SYSROOT" in output
    assert "macOS option" in output


@pytest.mark.skipif(sys.platform != "linux", reason="Linux cross-compile rejection")
def test_linux_rejects_foreign_system_name(tmp_path: Path) -> None:
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", "-DCMAKE_SYSTEM_NAME=Windows"
    )

    assert result.returncode != 0
    assert "Cross-compilation" in result.stdout + result.stderr


# --------------------------------------------------------------------------
# macOS contract — exercised through the same BikeAppleSDK.cmake the real
# CMakeLists.txt includes; the end-to-end controls live in
# tests/reference/test_native_check_tools.py and tools/native_checks.py.
# --------------------------------------------------------------------------


def _xcrun_sdk_path() -> Path:
    completed = subprocess.run(
        ["xcrun", "--sdk", "macosx", "--show-sdk-path"],
        capture_output=True,
        text=True,
        check=True,
        timeout=60,
    )
    return Path(completed.stdout.strip())


def _write_rejecting_compiler_wrapper(
    work: Path, rejected_var: str = "BIKE_TEST_BAD_SDK"
) -> tuple[Path, Path]:
    """c++ wrapper that fails any link (not -fsyntax-only) against the sysroot
    named by ``$<rejected_var>`` and records the rejection in a marker file."""
    marker = work / "link-rejections.txt"
    wrapper = _write_executable(
        work / "cxx-wrapper.sh",
        "#!/bin/bash\n"
        'args=("$@")\n'
        'selected_sdk=""\n'
        "syntax_only=0\n"
        "for ((i=0; i<${#args[@]}; ++i)); do\n"
        '  if [[ "${args[i]}" == "-isysroot" && $((i+1)) -lt ${#args[@]} ]]; then selected_sdk="${args[i+1]}"; fi\n'
        '  if [[ "${args[i]}" == "-fsyntax-only" ]]; then syntax_only=1; fi\n'
        "done\n"
        f'if [[ "$selected_sdk" == "${rejected_var}" && $syntax_only == 0 ]]; then\n'
        f'  printf "%s\\n" "link-rejected $selected_sdk" >> "$BIKE_TEST_REJECT_MARKER"\n'
        '  printf "%s\\n" "synthetic linker incompatibility for $selected_sdk" >&2\n'
        "  exit 1\n"
        "fi\n"
        'exec /usr/bin/c++ "${args[@]}"\n',
    )
    return wrapper, marker


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK dispatch control")
def test_macos_default_sdk_resolves_via_xcrun(tmp_path: Path) -> None:
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(source, tmp_path / "build")

    assert result.returncode == 0, result.stdout + result.stderr
    values = _probe_vars(tmp_path / "build")
    assert values["BIKE_TARGET_SYSTEM"] == "Darwin"
    assert values["sdk_source"] == "default"
    resolved = str(_xcrun_sdk_path().resolve())
    assert values["sdk_path"] == resolved
    # Default resolution records the resolved path as the request and writes it
    # back into the cache so project() compiles against it.
    assert values["sdk_requested"] == resolved
    assert values["sysroot_cache"] == resolved


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK dispatch control")
def test_macos_explicit_sdk_name_is_resolved(tmp_path: Path) -> None:
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", "-DCMAKE_OSX_SYSROOT=macosx"
    )

    assert result.returncode == 0, result.stdout + result.stderr
    values = _probe_vars(tmp_path / "build")
    pre = _probe_vars(tmp_path / "build", "probe_vars_pre.txt")
    assert values["sdk_source"] == "explicit"
    # The caller's exact request is retained verbatim — name form in
    # sdk_requested and in the cache entry at the module boundary; the
    # resolved path is recorded separately. (CMake's Darwin-Initialize may
    # rewrite the cache entry inside project(); that is identical with or
    # without this module.)
    assert values["sdk_requested"] == "macosx"
    assert pre["sdk_requested"] == "macosx"
    assert pre["sysroot_cache"] == "macosx"
    assert values["sdk_path"] == str(_xcrun_sdk_path().resolve())


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK dispatch control")
def test_macos_explicit_sdk_alias_is_canonicalized(tmp_path: Path) -> None:
    """The versioned SDK's ``MacOSX.sdk`` sibling alias is kept verbatim as
    the request while the selected path is canonicalized."""
    alias = _xcrun_sdk_path().parent / "MacOSX.sdk"
    if not alias.is_dir():
        pytest.skip("MacOSX.sdk alias is unavailable next to the default SDK")
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", f"-DCMAKE_OSX_SYSROOT={alias}"
    )

    assert result.returncode == 0, result.stdout + result.stderr
    values = _probe_vars(tmp_path / "build")
    pre = _probe_vars(tmp_path / "build", "probe_vars_pre.txt")
    assert values["sdk_source"] == "explicit"
    assert values["sdk_requested"] == str(alias)
    assert pre["sysroot_cache"] == str(alias)
    assert values["sdk_path"] == str(alias.resolve())


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK rejection control")
def test_macos_missing_explicit_sdk_path_fails(tmp_path: Path) -> None:
    missing = tmp_path / "missing-explicit-sdk"
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", f"-DCMAKE_OSX_SYSROOT={missing}"
    )

    assert result.returncode != 0
    assert str(missing) in result.stdout + result.stderr


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK rejection control")
def test_macos_unavailable_sdk_name_fails(tmp_path: Path) -> None:
    source = tmp_path / "probe"
    _write_probe(source)

    result = _configure(
        source, tmp_path / "build", "-DCMAKE_OSX_SYSROOT=macosx_bogus_t04"
    )

    assert result.returncode != 0
    assert "unavailable" in result.stdout + result.stderr


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK fallback control")
def test_macos_default_falls_back_from_link_incompatible_sdk(tmp_path: Path) -> None:
    """A default SDK that compiles but cannot link Accelerate is rejected in
    favour of the next installed candidate (same control as the native_checks
    ``sdk-default-link-fallback`` case, at module scope)."""
    bad_sdk = tmp_path / "incompatible-default-sdk"
    bad_sdk.mkdir()
    wrapper, marker = _write_rejecting_compiler_wrapper(tmp_path)
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    _write_executable(
        fake_bin / "xcrun",
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--sdk" ] && [ "${2:-}" = "macosx" ] && [ "${3:-}" = "--show-sdk-path" ]; then\n'
        '  printf "%s\\n" "$BIKE_TEST_BAD_SDK"; exit 0;\n'
        "fi\n"
        'exec /usr/bin/xcrun "$@"\n',
    )
    source = tmp_path / "probe"
    _write_probe(source)
    env = os.environ.copy()
    env["PATH"] = str(fake_bin) + os.pathsep + env.get("PATH", "")
    env["BIKE_TEST_BAD_SDK"] = str(bad_sdk.resolve())
    env["BIKE_TEST_REJECT_MARKER"] = str(marker)

    result = _configure(
        source, tmp_path / "build", f"-DCMAKE_CXX_COMPILER={wrapper}", env=env
    )

    assert result.returncode == 0, result.stdout + result.stderr
    values = _probe_vars(tmp_path / "build")
    assert values["sdk_source"] == "default"
    assert values["sdk_path"] != str(bad_sdk.resolve())
    assert Path(values["sdk_path"]).is_dir()
    assert marker.is_file()
    assert "link-rejected" in marker.read_text()


@pytest.mark.skipif(sys.platform != "darwin", reason="Apple SDK rejection control")
def test_macos_explicit_link_incompatible_sdk_is_fatal(tmp_path: Path) -> None:
    """An explicit SDK directory exists but cannot link: this stays a hard
    error, never a silent fallback to another SDK."""
    bad_sdk = tmp_path / "explicit-incompatible-sdk"
    bad_sdk.mkdir()
    wrapper, marker = _write_rejecting_compiler_wrapper(tmp_path)
    source = tmp_path / "probe"
    _write_probe(source)
    env = os.environ.copy()
    env["BIKE_TEST_BAD_SDK"] = str(bad_sdk.resolve())
    env["BIKE_TEST_REJECT_MARKER"] = str(marker)

    result = _configure(
        source,
        tmp_path / "build",
        f"-DCMAKE_OSX_SYSROOT={bad_sdk}",
        f"-DCMAKE_CXX_COMPILER={wrapper}",
        env=env,
    )

    assert result.returncode != 0
    output = result.stdout + result.stderr
    assert "incompatible with compiler" in output
    assert str(bad_sdk) in output


# --------------------------------------------------------------------------
# Cross-compilation refusal: one rule, two includers. BikePlatform.cmake (the
# legacy graph) and the root's BikeHostTarget.cmake call call the same
# function, so both must refuse a foreign CMAKE_SYSTEM_NAME on any host.
# --------------------------------------------------------------------------

HOST_TARGET_MODULE = REPO_ROOT / "native" / "cmake" / "BikeHostTarget.cmake"
FOREIGN_SYSTEM = "Windows"


def _write_host_dispatch_probe(source: Path, includer: str) -> None:
    """A scratch project that reaches the host dispatch through ``includer``:
    ``BikePlatform`` (legacy, before project()) or ``BikeHostTarget`` (the root
    shape: after project(), through the shared function)."""
    source.mkdir(parents=True)
    if includer == "BikePlatform":
        # The legacy gate runs before project(), as native/CMakeLists.txt does.
        text = (
            "cmake_minimum_required(VERSION 3.27)\n"
            f'include("{PLATFORM_MODULE}")\n'
            "project(host_dispatch_probe NONE)\n"
        )
    else:
        # The root's shape: the shared function runs after project().
        text = (
            "cmake_minimum_required(VERSION 3.27)\n"
            "project(host_dispatch_probe NONE)\n"
            f'include("{HOST_TARGET_MODULE}")\n'
            "bike_host_target_system(BIKE_TARGET_SYSTEM)\n"
            'message(STATUS "HOST_DISPATCH=${BIKE_TARGET_SYSTEM}")\n'
        )
    (source / "CMakeLists.txt").write_text(text)


@pytest.mark.parametrize("includer", ["BikePlatform", "BikeHostTarget"])
def test_cross_compiling_configure_is_refused(tmp_path: Path, includer: str) -> None:
    """A toolchain that targets another system is refused with the shared
    message, before any dispatch value is set. The BikePlatform case is the
    legacy graph's gate; the BikeHostTarget case is the root's."""
    source = tmp_path / "probe"
    _write_host_dispatch_probe(source, includer)

    result = _configure(
        source, tmp_path / "build", f"-DCMAKE_SYSTEM_NAME={FOREIGN_SYSTEM}"
    )

    assert result.returncode != 0
    output = result.stdout + result.stderr
    assert "Cross-compilation is not supported" in output
    assert "HOST_DISPATCH=" not in output


def test_host_dispatch_names_the_configured_host(tmp_path: Path) -> None:
    """The positive control: without a foreign system the shared function
    passes and reports the host it selected, so the refusal above is the only
    thing that changed."""
    import platform as host_platform

    source = tmp_path / "probe"
    _write_host_dispatch_probe(source, "BikeHostTarget")

    result = _configure(source, tmp_path / "build")

    assert result.returncode == 0, result.stdout + result.stderr
    assert f"HOST_DISPATCH={host_platform.system()}" in result.stdout
