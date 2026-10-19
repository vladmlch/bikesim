"""Platform-aware compiler policy contract for the native build.

``native/cmake/BikeCompilerPolicy.cmake`` owns the flag policy every
first-party native target receives: strict ISO C++23, the required
numerical/safety flags (``-ffp-contract=off`` and friends), the
compiler-family warning lists, and the OS/arch-gated options
(``-mbranch-protection=standard`` on AArch64 only,
``-headerpad_max_install_names`` on Darwin only, no ``-framework`` /
``-isysroot`` policy additions anywhere).

Each test configures a minimal probe project that includes the real
module and compiles a std::format/std::span/std::bit_cast program with
the actual selected compiler, then inspects ``compile_commands.json``
and the module's own ``bike_policy_report.json`` record. A mocked JSON
fixture is deliberately NOT used — the claims here are about real
compiler support. ``system=``/``machine=`` parameters exercise the
module's dispatch overrides (``BIKE_POLICY_OVERRIDE_*``) so the
OS/arch-gated paths are testable on a single host; compiler probing
always runs against the real compiler.
"""
from __future__ import annotations

import json
import platform
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
POLICY_MODULE = REPO_ROOT / "native" / "cmake" / "BikeCompilerPolicy.cmake"

CMAKE = shutil.which("cmake")

pytestmark = pytest.mark.skipif(CMAKE is None, reason="cmake executable unavailable")

CONFIGURE_TIMEOUT = 600
BUILD_TIMEOUT = 600

# Flags that must never appear in a first-party compile command regardless
# of host: fast-math/reassociation family. The bitwise oracle contract
# forbids them, so the policy both never emits them and refuses to keep
# them if injected.
BANNED_NUMERICAL_FLAGS = (
    "-ffast-math",
    "-funsafe-math-optimizations",
    "-fassociative-math",
    "-freciprocal-math",
    "-ffinite-math-only",
    "-fcx-limited-range",
    "-ffp-contract=fast",
    "-ffp-contract=on",
    "-Ofast",
)


def _host_system() -> str:
    if sys.platform == "darwin":
        return "Darwin"
    if sys.platform.startswith("linux"):
        return "Linux"
    return sys.platform


def _normalize_machine(machine: str) -> str:
    lowered = machine.lower()
    if lowered in {"arm64", "aarch64"}:
        return "aarch64"
    if lowered in {"x86_64", "amd64"}:
        return "x86_64"
    return lowered


def _host_machine() -> str:
    return _normalize_machine(platform.machine())


def _write_policy_probe(source: Path) -> None:
    """Minimal project: strict C++23 TU using std::format/std::span/
    std::bit_cast with the real module applied to the only target."""
    source.mkdir(parents=True)
    (source / "main.cpp").write_text(
        "#include <bit>\n"
        "#include <cstdint>\n"
        "#include <format>\n"
        "#include <span>\n"
        "#include <string>\n"
        "namespace {\n"
        "double sum(std::span<const double> values) {\n"
        "  double total = 0.0;\n"
        "  for (const double value : values) { total += value; }\n"
        "  return total;\n"
        "}\n"
        "}\n"
        "int main() {\n"
        "  const double data[] = {1.5, 2.5, 3.0};\n"
        "  const std::uint64_t bits = std::bit_cast<std::uint64_t>(data[0]);\n"
        "  const std::string text = std::format(\"sum={} bits={:x}\", sum(data), bits);\n"
        "  return text.empty() ? 1 : 0;\n"
        "}\n"
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.27)\n"
        # Simulating a non-Apple host must also stop CMake's own Darwin
        # toolchain init from injecting -isysroot, otherwise the simulated
        # dispatch stays invisible in compile_commands.json.
        'if(DEFINED BIKE_POLICY_OVERRIDE_SYSTEM AND NOT "${BIKE_POLICY_OVERRIDE_SYSTEM}" STREQUAL "Darwin")\n'
        '  set(CMAKE_OSX_SYSROOT "")\n'
        "endif()\n"
        "project(bike_policy_probe CXX)\n"
        "set(CMAKE_CXX_STANDARD 23)\n"
        "set(CMAKE_CXX_STANDARD_REQUIRED ON)\n"
        "set(CMAKE_CXX_EXTENSIONS OFF)\n"
        "set(CMAKE_EXPORT_COMPILE_COMMANDS ON)\n"
        f'include("{POLICY_MODULE.as_posix()}")\n'
        "add_executable(policy_probe main.cpp)\n"
        "bike_apply_native_policy(policy_probe)\n"
    )


def _commands_from(build: Path) -> list[str]:
    database = json.loads((build / "compile_commands.json").read_text())
    commands: list[str] = []
    for entry in database:
        argv = entry.get("arguments")
        if argv is None:
            argv = shlex.split(entry["command"])
        commands.append(" ".join(argv))
    return commands


@pytest.fixture
def policy_fixture(tmp_path: Path):
    """Configure+build the policy probe project with the real compiler.

    ``system``/``machine`` select the module's dispatch overrides; the
    compiler, library and probes are always the real ones. Returns
    ``(commands, report)`` — compile_commands entries joined to strings
    plus the parsed bike_policy_report.json record.
    """
    state = {"count": 0}

    def _configure(
        *,
        system: str | None = None,
        machine: str | None = None,
        compiler: str | None = None,
        build_type: str | None = None,
        extra_options: Iterable[str] = (),
        expect_failure: bool = False,
    ):
        state["count"] += 1
        case_dir = tmp_path / f"case-{state['count']}"
        source_dir = case_dir / "src"
        build_dir = case_dir / "build"
        _write_policy_probe(source_dir)

        options: list[str] = []
        if system is not None:
            options.append(f"-DBIKE_POLICY_OVERRIDE_SYSTEM={system}")
        if machine is not None:
            options.append(f"-DBIKE_POLICY_OVERRIDE_MACHINE={machine}")
        if compiler is not None:
            options.append(f"-DCMAKE_CXX_COMPILER={compiler}")
        if build_type is not None:
            options.append(f"-DCMAKE_BUILD_TYPE={build_type}")
        options.extend(extra_options)

        configured = subprocess.run(
            [CMAKE, "-S", str(source_dir), "-B", str(build_dir), *options],
            capture_output=True,
            text=True,
            timeout=CONFIGURE_TIMEOUT,
        )
        if expect_failure:
            return configured
        assert configured.returncode == 0, (
            f"policy probe configure failed: {configured.stdout}\n{configured.stderr}"
        )
        built = subprocess.run(
            [CMAKE, "--build", str(build_dir)],
            capture_output=True,
            text=True,
            timeout=BUILD_TIMEOUT,
        )
        assert built.returncode == 0, (
            f"policy probe build failed: {built.stdout}\n{built.stderr}"
        )
        commands = _commands_from(build_dir)
        report = json.loads((build_dir / "bike_policy_report.json").read_text())
        return commands, report

    return _configure


def _compiler_candidates() -> list[str]:
    names = ["c++", "clang++", "g++"]
    for pattern_dir in (Path("/usr/bin"), Path("/opt/homebrew/opt/llvm/bin")):
        if pattern_dir.is_dir():
            names.extend(sorted(p.name for p in pattern_dir.glob("clang++-*")))
            names.extend(sorted(p.name for p in pattern_dir.glob("g++-*")))
    # Deduplicate by realpath but keep the invocation name: distro clang++ is
    # commonly a symlink to the `clang` binary, and the driver selects C++
    # mode (libstdc++ include/link defaults) from argv[0]. Resolving the
    # command path itself would turn a C++ driver into the C driver.
    resolved: list[str] = []
    seen: set[str] = set()
    for name in names:
        found = shutil.which(name)
        if found is None:
            continue
        key = str(Path(found).resolve())
        if key not in seen:
            seen.add(key)
            resolved.append(found)
    return resolved


AVAILABLE_COMPILERS = _compiler_candidates()


# --------------------------------------------------------------------------
# Brief Step 1 verbatim regression (plus the remaining OS-gated exclusions)
# --------------------------------------------------------------------------


def test_linux_x86_policy_has_no_arm_or_apple_flags(policy_fixture):
    commands, report = policy_fixture(system="Linux", machine="x86_64")
    joined = " ".join(commands)
    assert "-ffp-contract=off" in joined
    assert "-mbranch-protection=standard" not in joined
    assert "-headerpad_max_install_names" not in joined
    assert report["cxx_standard"] == 23
    # The remaining OS-gated exclusions from the brief: the policy must
    # never emit Apple linker/system options for a Linux target. CMake's
    # own -isysroot/-arch machinery is neutralized by the fixture when the
    # simulated system is not Darwin, so these hold on real commands too.
    assert "-isysroot" not in joined
    assert "-framework" not in joined
    assert "-headerpad_max_install_names" not in report["link_options"]
    assert report["branch_protection"]["applied"] is False
    assert report["system"] == "Linux"
    assert report["machine"] == "x86_64"


def test_every_first_party_command_keeps_strict_cpp23_and_fp_contract(policy_fixture):
    """The numerics contract on every TU: strict ISO C++23 and
    -ffp-contract=off, with no fast-math/reassociation anywhere."""
    commands, report = policy_fixture()
    assert commands, "fixture produced no compile commands"
    for command in commands:
        assert "-std=c++23" in command
        assert "-std=gnu++23" not in command
        assert "-ffp-contract=off" in command
        assert "-Werror" in command
        for banned in BANNED_NUMERICAL_FLAGS:
            assert banned not in command
    assert "-ffp-contract=off" in report["required_flags"]


def test_unsupported_optional_warning_is_omitted_and_recorded(policy_fixture):
    """An intentionally unsupported warning name is probed, omitted from
    the applied options, and recorded with a reason — never a build
    failure and never silently dropped."""
    commands, report = policy_fixture(
        extra_options=("-DBIKE_POLICY_TEST_EXTRA_WARNINGS=-Wtotally-bogus-t09-warning",)
    )
    joined = " ".join(commands)
    assert "-Wtotally-bogus-t09-warning" not in joined
    omitted = {item["flag"]: item["reason"] for item in report["omitted_warnings"]}
    assert "-Wtotally-bogus-t09-warning" in omitted
    assert omitted["-Wtotally-bogus-t09-warning"]


def test_missing_required_numeric_flag_fails_configuration(policy_fixture):
    """The required-flag path is fail-closed: an unsupportable flag in the
    required set makes configuration fail and names the flag."""
    configured = policy_fixture(
        expect_failure=True,
        extra_options=("-DBIKE_POLICY_TEST_EXTRA_REQUIRED=-fno-such-required-flag-t09",),
    )
    assert configured.returncode != 0
    output = configured.stdout + configured.stderr
    assert "-fno-such-required-flag-t09" in output


# --------------------------------------------------------------------------
# Real-host dispatch — assertions follow the host's actual system/machine
# --------------------------------------------------------------------------


def test_host_dispatch_applies_the_platform_flags(policy_fixture):
    commands, report = policy_fixture()
    joined = " ".join(commands)
    system = _host_system()
    machine = _host_machine()
    assert report["system"] == system
    assert report["machine"] == machine
    if machine == "aarch64":
        assert "-mbranch-protection=standard" in joined
        assert report["branch_protection"]["applied"] is True
    else:
        assert "-mbranch-protection=standard" not in joined
        assert report["branch_protection"]["applied"] is False
    if system == "Darwin":
        assert "-headerpad_max_install_names" in report["link_options"]
    else:
        assert "-headerpad_max_install_names" not in report["link_options"]
        assert "-headerpad_max_install_names" not in joined


def test_simulated_aarch64_keeps_branch_protection(policy_fixture):
    """On an AArch64 host with a compiler that accepts it, the flag is
    probed on and present regardless of which OS name is simulated."""
    if _host_machine() != "aarch64":
        pytest.skip("host is not AArch64")
    commands, report = policy_fixture(system="Linux", machine="aarch64")
    assert "-mbranch-protection=standard" in " ".join(commands)
    assert report["branch_protection"]["applied"] is True


def test_stdlib_hardening_dispatch(policy_fixture):
    """libc++ carries the EXTENSIVE/FAST mode define; libstdc++ carries
    _GLIBCXX_ASSERTIONS and never _GLIBCXX_DEBUG."""
    commands, report = policy_fixture()
    joined = " ".join(commands)
    identity = report["stdlib"]["identity"]
    if _host_system() == "Darwin":
        assert identity == "libc++"
        assert "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE" in joined
    elif identity == "libstdc++":
        assert "-D_GLIBCXX_ASSERTIONS" in joined
        assert "_GLIBCXX_DEBUG" not in joined
        assert "_LIBCPP_HARDENING_MODE" not in joined
    else:
        raise AssertionError(f"unexpected standard-library identity: {identity}")


def test_cxx23_library_features_are_probed(policy_fixture):
    """The module must prove the library carries the C++23 features the
    writers/benchmark use — std::format, std::span, std::bit_cast."""
    _, report = policy_fixture()
    features = report["cxx23_library"]
    assert features["supported"] is True
    assert {"std::format", "std::span", "std::bit_cast"} <= set(features["checked"])


@pytest.mark.skipif(_host_system() != "Linux", reason="glibc fortify gating is a Linux property")
def test_fortify_source_follows_optimized_configs(policy_fixture):
    """On glibc, _FORTIFY_SOURCE=3 is only meaningful with optimization —
    it is applied to optimized configs and recorded absent for Debug."""
    release_commands, release_report = policy_fixture(build_type="Release")
    if release_report["glibc"] is not True:
        pytest.skip("host libc is not glibc; fortify gating does not apply")
    assert "-D_FORTIFY_SOURCE=3" in " ".join(release_commands)
    debug_commands, _ = policy_fixture(build_type="Debug")
    assert "-D_FORTIFY_SOURCE=3" not in " ".join(debug_commands)


# --------------------------------------------------------------------------
# Same policy object under every available real compiler
# --------------------------------------------------------------------------


@pytest.mark.parametrize("compiler", AVAILABLE_COMPILERS)
def test_policy_under_each_available_compiler(policy_fixture, compiler: str):
    """Every compiler actually installed on the host drives the same
    contract: strict C++23, -ffp-contract=off, -Werror, no banned flags,
    and a recorded compiler family matching the real toolchain."""
    commands, report = policy_fixture(compiler=compiler)
    joined = " ".join(commands)
    assert "-ffp-contract=off" in joined
    assert "-std=c++23" in joined
    assert "-Werror" in joined
    for banned in BANNED_NUMERICAL_FLAGS:
        assert banned not in joined
    assert report["compiler_family"] in {"appleclang", "clang", "gcc"}
    family = report["compiler_family"]
    if family == "gcc":
        # No clang-only warning name is ever passed to GCC.
        assert "-Wunsafe-buffer-usage" not in joined
        assert "-Wredeclared-class-member" not in joined
        assert "-Wextra-semi-stmt" not in joined
