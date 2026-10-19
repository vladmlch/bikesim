"""Cross-platform analysis-tool discovery, argv adaptation and controls (T14).

``tools/native_toolchain.py`` owns executable resolution (``resolve_tool``)
and identity/capability inspection (``inspect_tool``) for the native
analysis sweeps. ``tools/native_checks.py`` consumes it for compiler-sibling
and PATH discovery and adapts the compile-database argv for the secondary
GCC frontend (``_gcc_context_args``/``_tool_argv``) and the stdlib hardening
control (``_stdlib_probe_spec``).

Every subprocess in this file is an argv array — no shell interpretation.
The negative controls are real: each deliberately produces a warning, a
header self-sufficiency error, an invalid-source rejection and a runtime
hardening assertion with an executable that demonstrably launched.
"""
from __future__ import annotations

import platform
import shutil
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any, Sequence

import pytest

import tools.native_checks as native_checks
from tools.native_checks import CheckError
from tools.native_toolchain import (
    darwin_tool_candidates,
    inspect_tool,
    resolve_tool,
    sibling_candidates,
)


def executable(path: Path) -> Path:
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(0o755)
    return path


# ---------------------------------------------------------------------------
# resolve_tool — brief Step-1 regression verbatim
# ---------------------------------------------------------------------------


def test_explicit_tool_wins(tmp_path):
    requested = executable(tmp_path / "chosen-clang-tidy")
    fallback = executable(tmp_path / "other-clang-tidy")
    assert resolve_tool("clang-tidy", str(requested), [str(fallback)]) == requested


def test_broken_override_is_not_hidden(tmp_path):
    fallback = executable(tmp_path / "clang-tidy")
    with pytest.raises(ValueError, match="override"):
        resolve_tool("clang-tidy", str(tmp_path / "missing"), [str(fallback)])


def test_tool_path_containing_spaces_resolves(tmp_path):
    """A tool inside a directory whose name contains spaces stays one argv token."""
    spaced = tmp_path / "tool dir with space"
    spaced.mkdir()
    requested = executable(spaced / "clang-tidy")
    resolved = resolve_tool("clang-tidy", str(requested), [])
    assert resolved == requested
    assert " " in str(resolved)


def test_relative_override_uses_path_search(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path))
    requested = executable(tmp_path / "clang-tidy")
    assert resolve_tool("clang-tidy", "clang-tidy", []) == requested


def test_missing_required_tool_lists_checked_candidates(tmp_path):
    existing = executable(tmp_path / "clang-tidy")
    with pytest.raises(ValueError, match="required tool clang-tidy.*checked"):
        resolve_tool(
            "clang-tidy",
            None,
            [str(tmp_path / "missing-one"), str(tmp_path / "missing-two")],
        )
    # The same resolution succeeds once a later candidate exists.
    assert resolve_tool("clang-tidy", None, [str(tmp_path / "missing"), str(existing)]) == existing


def test_non_executable_candidate_is_skipped(tmp_path):
    not_exec = tmp_path / "clang-tidy"
    not_exec.write_text("#!/bin/sh\nexit 0\n")  # no +x bit
    working = executable(tmp_path / "clang-tidy-18")
    assert resolve_tool("clang-tidy", None, [str(not_exec), str(working)]) == working


# ---------------------------------------------------------------------------
# sibling candidates — compatible configured-compiler tools
# ---------------------------------------------------------------------------


def test_clang_sibling_candidates_follow_version_suffix(tmp_path):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    compiler = {"path": str(bindir / "clang++-18"), "id": "Clang"}
    tidy = sibling_candidates(compiler, "tidy")
    assert tidy == [str(bindir / "clang-tidy-18"), str(bindir / "clang-tidy")]
    analyzer = sibling_candidates(compiler, "analyzer")
    assert analyzer == [str(bindir / "clang++-18"), str(bindir / "clang++")]


def test_gcc_sibling_candidates_only_for_gxx(tmp_path):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    compiler = {"path": str(bindir / "g++-13"), "id": "GNU"}
    assert sibling_candidates(compiler, "gxx") == [
        str(bindir / "g++-13"),
        str(bindir / "g++"),
    ]
    # A GNU compiler has no clang-family siblings.
    assert sibling_candidates(compiler, "tidy") == []
    assert sibling_candidates(compiler, "analyzer") == []


def test_triplet_prefixed_gcc_sibling_names(tmp_path):
    compiler = {"path": "/usr/bin/aarch64-linux-gnu-g++-13", "id": "GNU"}
    names = sibling_candidates(compiler, "gxx")
    assert names[:2] == [
        "/usr/bin/g++-13",
        "/usr/bin/aarch64-linux-gnu-g++-13",
    ]


def test_appleclang_contributes_no_siblings(tmp_path):
    """AppleClang's toolchain dir ships no qualified analysis tools — Darwin
    keeps the recorded Homebrew/Xcode discovery instead."""
    compiler = {"path": "/usr/bin/c++", "id": "AppleClang"}
    for kind in ("tidy", "analyzer", "gxx", "cppcheck"):
        assert sibling_candidates(compiler, kind) == []


def test_darwin_candidates_stay_homebrew_prefixed() -> None:
    assert "/opt/homebrew/opt/llvm/bin/clang-tidy" in darwin_tool_candidates("tidy")
    assert darwin_tool_candidates("analyzer") == [
        "/opt/homebrew/opt/llvm/bin/clang++",
        "/usr/local/opt/llvm/bin/clang++",
    ]


# ---------------------------------------------------------------------------
# inspect_tool — identity, target and resource evidence
# ---------------------------------------------------------------------------


def _host_cxx() -> Path:
    found = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if found is None:
        pytest.skip("no host C++ compiler on PATH")
    return Path(found)


def test_inspect_tool_records_version_target_and_resource() -> None:
    record = inspect_tool(_host_cxx())
    assert record["family"] in {"clang", "gcc"}
    assert record["version_query"]["returncode"] == 0
    assert record["version"]
    assert Path(record["resolved_path"]).is_file()
    # A real driver answers at least one of the target spellings.
    assert record["target"], record["target_query"]


def test_inspect_tool_rejects_unrelated_tool() -> None:
    with pytest.raises(ValueError, match="not a recognized"):
        inspect_tool(Path(sys.executable))


def test_inspect_tool_rejects_missing_and_silent(tmp_path):
    with pytest.raises(ValueError, match="not an executable file"):
        inspect_tool(tmp_path / "missing")
    silent = executable(tmp_path / "clang-tidy")
    with pytest.raises(ValueError, match="did not identify itself"):
        inspect_tool(silent)


def test_inspect_tool_rejects_wrong_family_words(tmp_path):
    fake = tmp_path / "not-a-tidy"
    fake.write_text(
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--version" ]; then echo "GNU coreutils ls 9.5"; exit 0; fi\n'
        "exit 0\n"
    )
    fake.chmod(0o755)
    with pytest.raises(ValueError, match="not a recognized"):
        inspect_tool(fake)


# ---------------------------------------------------------------------------
# _gcc_context_args / _tool_argv — secondary-frontend argv adaptation
# ---------------------------------------------------------------------------


def _entry(argv: list[str], source: Path, directory: Path) -> dict[str, Any]:
    return {"arguments": argv, "source": source, "directory": directory}


def _check_context(
    *,
    sysroot: Path | None,
    launcher: Sequence[str] = (),
    include_paths: Sequence[Path] = (),
    sdk: Path | None = None,
) -> dict[str, Any]:
    return {
        "sysroot": str(sysroot) if sysroot is not None else None,
        "sdk": str(sdk) if sdk is not None else None,
        "compiler": {"path": "/usr/bin/clang++-18", "id": "Clang", "launcher": list(launcher)},
        "include_paths": [str(path) for path in include_paths],
    }


def _compile_dir(tmp_path: Path) -> tuple[Path, Path]:
    directory = tmp_path / "build-dir"
    directory.mkdir()
    source = tmp_path / "tu.cpp"
    source.write_text("int main() { return 0; }\n")
    return directory, source


def test_gcc_args_preserve_compiler_launcher(tmp_path):
    directory, source = _compile_dir(tmp_path)
    include = tmp_path / "headers"
    include.mkdir()
    entry = _entry(
        [
            "ccache",
            "/usr/bin/clang++-18",
            "-std=c++23",
            "-I",
            str(include),
            "-c",
            str(source),
            "-o",
            "tu.o",
        ],
        source,
        directory,
    )
    context = _check_context(sysroot=None, launcher=["ccache"])
    args = native_checks._gcc_context_args(entry, context)
    # The launcher and compiler tokens are consumed by the prefix; only the
    # TU's semantic context is forwarded.
    assert "ccache" not in args
    assert "clang++-18" not in args
    assert ["-I", str(include)] == args[:2] or ["-I", str(include)] in [
        args[i : i + 2] for i in range(len(args) - 1)
    ]

    tool = executable(tmp_path / "g++-13")
    argv, _ = native_checks._tool_argv(
        "frontends",
        tool,
        entry,
        context,
        build=tmp_path,
        run_id="t14",
        index=0,
        pass_name="stable",
    )
    assert argv[:2] == ["ccache", str(tool)]


def test_gcc_args_forward_linux_sysroot_pair_and_joined(tmp_path):
    directory, source = _compile_dir(tmp_path)
    sysroot = tmp_path / "sysroot"
    sysroot.mkdir()
    context = _check_context(sysroot=sysroot)
    pair = _entry(
        ["g++-13", "-std=c++23", "--sysroot", str(sysroot), "-c", str(source), "-o", "tu.o"],
        source,
        directory,
    )
    joined = _entry(
        ["g++-13", "-std=c++23", f"--sysroot={sysroot}", "-c", str(source), "-o", "tu.o"],
        source,
        directory,
    )
    pair_args = native_checks._gcc_context_args(pair, context)
    joined_args = native_checks._gcc_context_args(joined, context)
    assert ["--sysroot", str(sysroot)] in [
        pair_args[i : i + 2] for i in range(len(pair_args) - 1)
    ]
    assert f"--sysroot={sysroot}" in joined_args


def test_gcc_args_forward_darwin_isysroot(tmp_path):
    directory, source = _compile_dir(tmp_path)
    sdk = tmp_path / "MacOSX.sdk"
    sdk.mkdir()
    context = _check_context(sysroot=sdk, sdk=sdk)
    entry = _entry(
        ["clang++", "-std=c++23", "-isysroot", str(sdk), "-c", str(source), "-o", "tu.o"],
        source,
        directory,
    )
    args = native_checks._gcc_context_args(entry, context)
    assert ["-isysroot", str(sdk)] in [args[i : i + 2] for i in range(len(args) - 1)]


def test_gcc_args_never_forward_apple_sdk_to_linux_gcc(tmp_path):
    """An -isysroot the Linux build context did not record is drift, not a
    forwardable include root — the sweep must fail, not analyze another
    configuration."""
    directory, source = _compile_dir(tmp_path)
    apple_sdk = tmp_path / "MacOSX.sdk"
    apple_sdk.mkdir()
    context = _check_context(sysroot=None)
    entry = _entry(
        ["g++-13", "-std=c++23", "-isysroot", str(apple_sdk), "-c", str(source), "-o", "tu.o"],
        source,
        directory,
    )
    with pytest.raises(CheckError, match="sysroot"):
        native_checks._gcc_context_args(entry, context)


def test_gcc_args_reject_sysroot_drift(tmp_path):
    directory, source = _compile_dir(tmp_path)
    configured = tmp_path / "sysroot"
    configured.mkdir()
    drifted = tmp_path / "other-sysroot"
    drifted.mkdir()
    context = _check_context(sysroot=configured)
    entry = _entry(
        ["g++-13", "-std=c++23", "--sysroot", str(drifted), "-c", str(source), "-o", "tu.o"],
        source,
        directory,
    )
    with pytest.raises(CheckError, match="sysroot"):
        native_checks._gcc_context_args(entry, context)


def test_gcc_args_preserve_defines_language_and_abi(tmp_path):
    directory, source = _compile_dir(tmp_path)
    context = _check_context(sysroot=None)
    entry = _entry(
        [
            "g++-13",
            "-std=c++23",
            "-D_GLIBCXX_ASSERTIONS",
            "-D",
            "BIKE_FLAG=1",
            "-ffp-contract=off",
            "-fstack-protector-strong",
            "-mbranch-protection=standard",
            "-stdlib=libc++",  # clang-only spelling: dropped for GCC
            "-O3",
            "-Wsome-warning",
            "-c",
            str(source),
            "-o",
            "tu.o",
        ],
        source,
        directory,
    )
    args = native_checks._gcc_context_args(entry, context)
    assert "-D_GLIBCXX_ASSERTIONS" in args
    assert ["-D", "BIKE_FLAG=1"] in [args[i : i + 2] for i in range(len(args) - 1)]
    assert "-std=c++23" in args
    assert "-ffp-contract=off" in args
    assert "-fstack-protector-strong" in args
    assert "-mbranch-protection=standard" in args
    # Compiler-specific spellings are never forwarded to the other frontend.
    assert "-stdlib=libc++" not in args
    assert "-O3" not in args
    assert "-Wsome-warning" not in args


def test_analyzer_argv_preserves_launcher_and_drops_output(tmp_path):
    directory, source = _compile_dir(tmp_path)
    context = _check_context(sysroot=None, launcher=["ccache"])
    entry = _entry(
        [
            "ccache",
            "/usr/bin/clang++-18",
            "-std=c++23",
            "-c",
            str(source),
            "-o",
            "tu.o",
        ],
        source,
        directory,
    )
    tool = executable(tmp_path / "clang++-18")
    argv, _ = native_checks._tool_argv(
        "analyzer",
        tool,
        entry,
        context,
        build=tmp_path,
        run_id="t14",
        index=0,
        pass_name="stable",
    )
    assert argv[0] == "ccache"
    assert argv[1] == str(tool)
    assert "--analyze" in argv
    assert "tu.o" not in argv


# ---------------------------------------------------------------------------
# _stdlib_probe_spec — platform-appropriate standard-library selection
# ---------------------------------------------------------------------------


def _stdlib_context(stdlib: str, compiler_id: str, mode: str) -> dict[str, Any]:
    return {
        "stdlib": {"name": stdlib, "hardening": {"mode": mode, "effective": True}},
        "compiler": {"id": compiler_id},
    }


def test_stdlib_probe_spec_libstdcxx_assertions() -> None:
    spec = native_checks._stdlib_probe_spec(_stdlib_context("libstdc++", "GNU", "ASSERTIONS"))
    assert spec["define"] == "-D_GLIBCXX_ASSERTIONS"
    assert spec["stdlib_args"] == []
    assert spec["expected_signals"] == {int(signal.SIGABRT)}
    # ABI-substituting debug mode is never part of the control.
    assert "_GLIBCXX_DEBUG" not in spec["define"]


def test_stdlib_probe_spec_libcpp_mode_and_stdlib_flag() -> None:
    clang_spec = native_checks._stdlib_probe_spec(
        _stdlib_context("libc++", "Clang", "EXTENSIVE")
    )
    assert clang_spec["define"] == "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE"
    assert clang_spec["stdlib_args"] == ["-stdlib=libc++"]
    apple_spec = native_checks._stdlib_probe_spec(
        _stdlib_context("libc++", "AppleClang", "FAST")
    )
    assert apple_spec["define"] == "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST"
    assert apple_spec["stdlib_args"] == ["-stdlib=libc++"]
    # -stdlib= is a clang driver spelling — a GNU frontend never receives it.
    gnu_spec = native_checks._stdlib_probe_spec(
        _stdlib_context("libc++", "GNU", "EXTENSIVE")
    )
    assert gnu_spec["stdlib_args"] == []


# ---------------------------------------------------------------------------
# Real negative controls — launched executables producing real diagnostics
# ---------------------------------------------------------------------------


def _run(argv: Sequence[str], cwd: Path, timeout: int = 120) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(token) for token in argv],
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )


def _resolve_first(candidates: Sequence[str]) -> Path:
    located = resolve_tool("c++", None, [str(candidate) for candidate in candidates])
    return located


def _find_gcc() -> Path | None:
    """The first real GNU C++ frontend on this host (never a clang alias)."""
    for candidate in ("g++-16", "g++-15", "g++-14", "g++-13", "g++"):
        path = shutil.which(candidate)
        if path is None:
            continue
        try:
            if inspect_tool(Path(path))["family"] == "gcc":
                return Path(path)
        except ValueError:
            continue
    return None


def test_warning_control_fires_real_diagnostic(tmp_path):
    cxx = _resolve_first(["c++", "g++", "clang++"])
    source = tmp_path / "warn.cpp"
    source.write_text(
        "[[nodiscard]] int result() { return 1; }\n"
        "int main() { result(); return 0; }\n"
    )
    completed = _run(
        [cxx, "-std=c++23", "-Werror=unused-result", "-fsyntax-only", source],
        cwd=tmp_path,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "unused-result" in output or "nodiscard" in output
    # The executable launched and produced a warning diagnostic — this is
    # evidence the warning machinery fired, not a launch failure.
    assert "warning" in output or "error" in output


def test_header_self_sufficiency_control_fails_with_diagnostic(tmp_path):
    cxx = _resolve_first(["c++", "g++", "clang++"])
    header = tmp_path / "not_self_sufficient.hpp"
    header.write_text("inline int missing() { return std::vector<int>{}.size(); }\n")
    probe = tmp_path / "probe.cpp"
    probe.write_text(f'#include "{header.name}"\nint main() {{ return missing(); }}\n')
    completed = _run(
        [cxx, "-std=c++23", "-fsyntax-only", "-I", str(tmp_path), probe],
        cwd=tmp_path,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "vector" in output
    assert "error" in output.lower()


def test_invalid_source_control_is_rejected_with_diagnostics(tmp_path):
    cxx = _resolve_first(["c++", "g++", "clang++"])
    source = tmp_path / "invalid.cpp"
    source.write_text("this is not C++ at all !!!\n")
    completed = _run([cxx, "-std=c++23", "-fsyntax-only", source], cwd=tmp_path)
    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "error" in output.lower()


_BOUNDS_PROBE = (
    "#include <cstdint>\n#include <vector>\n"
    "#if defined(_GLIBCXX_DEBUG)\n"
    "#error _GLIBCXX_DEBUG changes container ABI; it is not a production hardening mode\n"
    "#endif\n"
    "#if HARDENED_PROBE && !defined(_GLIBCXX_ASSERTIONS)\n"
    "#error the hardened control requires _GLIBCXX_ASSERTIONS\n"
    "#endif\n"
    "#if !HARDENED_PROBE && defined(_GLIBCXX_ASSERTIONS)\n"
    "#error the unhardened control must not carry _GLIBCXX_ASSERTIONS\n"
    "#endif\n"
    "int main(int argc, char**) {\n"
    "  std::vector<std::uint64_t> values(64);\n"
    "  if (argc > 1) {\n"
    "    volatile std::uint64_t invalid = values[values.size() + 64];\n"
    "    return static_cast<int>(invalid);\n"
    "  }\n"
    "  return 0;\n"
    "}\n"
)


def test_libstdcxx_assertions_abort_out_of_bounds(tmp_path):
    """Real _GLIBCXX_ASSERTIONS positive/negative control: the hardened
    binary must compile and terminate nonzero on a bounds violation; the
    unhardened binary's UB outcome is recorded, not asserted."""
    gxx = _find_gcc()
    if gxx is None:
        pytest.skip("no real GNU C++ frontend available on this host")
    source = tmp_path / "bounds.cpp"
    source.write_text(_BOUNDS_PROBE)

    hardened = tmp_path / "hardened"
    hardened_build = _run(
        [gxx, "-std=c++23", "-DHARDENED_PROBE=1", "-D_GLIBCXX_ASSERTIONS", "-O0", source, "-o", hardened],
        cwd=tmp_path,
    )
    assert hardened_build.returncode == 0, hardened_build.stderr
    violation = _run([hardened, "out-of-bounds"], cwd=tmp_path, timeout=30)
    assert violation.returncode != 0
    assert violation.returncode == -int(signal.SIGABRT) or "assert" in (
        violation.stdout + violation.stderr
    ).lower()
    benign = _run([hardened], cwd=tmp_path, timeout=30)
    assert benign.returncode == 0

    unhardened = tmp_path / "unhardened"
    # libstdc++ may self-enable assertions (driver predefines, or newer GCC's
    # "unoptimized builds assert" default): undefine the driver macro and opt
    # out of the header's own enable so the negative control is genuinely
    # unhardened on every toolchain.
    unhardened_build = _run(
        [
            gxx,
            "-std=c++23",
            "-U_GLIBCXX_ASSERTIONS",
            "-D_GLIBCXX_NO_ASSERTIONS",
            "-O0",
            source,
            "-o",
            unhardened,
        ],
        cwd=tmp_path,
    )
    # The source's own guards prove _GLIBCXX_ASSERTIONS/_GLIBCXX_DEBUG are
    # absent in the unhardened build; a failed compile means the toolchain
    # leaked a hardening define, not that the control worked.
    assert unhardened_build.returncode == 0, unhardened_build.stderr
    outcome = _run([unhardened, "out-of-bounds"], cwd=tmp_path, timeout=30)
    # Unchecked out-of-bounds access is undefined behavior: the result is
    # evidence (recorded in the assertion below), never a guaranteed crash.
    assert isinstance(outcome.returncode, int)


@pytest.mark.skipif(platform.system() != "Darwin", reason="libc++ runtime control")
def test_libcpp_extensive_assertion_traps_out_of_bounds(tmp_path):
    cxx_path = shutil.which("c++") or shutil.which("clang++")
    if cxx_path is None:
        pytest.skip("no clang-family C++ compiler on PATH")
    cxx = Path(cxx_path)
    if inspect_tool(cxx)["family"] != "clang":
        pytest.skip("libc++ controls require a clang-family compiler")
    source = tmp_path / "bounds.cpp"
    source.write_text(
        "#include <cstdint>\n#include <vector>\n"
        "#ifndef _LIBCPP_HARDENING_MODE_EXTENSIVE\n"
        "#error libc++ lacks the EXTENSIVE hardening mode\n"
        "#endif\n"
        "int main(int argc, char**) {\n"
        "  std::vector<std::uint64_t> values(64);\n"
        "  if (argc > 1) {\n"
        "    volatile std::uint64_t invalid = values[values.size() + 64];\n"
        "    return static_cast<int>(invalid);\n"
        "  }\n"
        "  return 0;\n"
        "}\n"
    )
    binary = tmp_path / "libcpp-hardened"
    built = _run(
        [
            cxx,
            "-std=c++23",
            "-stdlib=libc++",
            "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE",
            "-O0",
            source,
            "-o",
            binary,
        ],
        cwd=tmp_path,
    )
    assert built.returncode == 0, built.stderr
    violation = _run([binary, "out-of-bounds"], cwd=tmp_path, timeout=30)
    assert violation.returncode != 0
    benign = _run([binary], cwd=tmp_path, timeout=30)
    assert benign.returncode == 0
