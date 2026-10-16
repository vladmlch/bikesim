"""LLVM coverage-tool discovery and qualification controls (T17).

``tools.native_toolchain.resolve_coverage_tools`` resolves the
llvm-profdata/llvm-cov pair for the configured clang compiler in a fixed
order — an explicit ``LLVM_PROFDATA``/``LLVM_COV`` override, a compiler
sibling from the same toolchain directory, a PATH candidate — with an
``xcrun --find`` fallback that exists on Darwin only. A resolved pair is
then qualified by the format probe: the selected compiler builds a minimal
instrumented control, runs it under a fresh ``LLVM_PROFILE_FILE``, and the
chosen tools merge and export the profile. Branding or a shared major
version alone proves nothing; the compile/run/merge/export chain is the
authoritative check.

Every subprocess in this file is an argv array — no shell interpretation.
The fake LLVM toolchain used throughout is self-contained: its compiler
"compiles" a control binary that emits a ``.profraw`` beside
``LLVM_PROFILE_FILE``, and its tools answer ``--version``/``merge``/
``export`` so the probe's real semantics (file creation, exit codes, JSON
shape) are exercised without depending on a host LLVM install.
"""

from __future__ import annotations

import json
import os
import platform
import shutil
from pathlib import Path

import pytest

import tools.native_toolchain as native_toolchain
from tools.native_toolchain import resolve_coverage_tools


def _write_executable(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(0o755)
    return path


def _fake_clang(directory: Path, name: str = "clang++-99") -> Path:
    """A clang-family compiler stand-in.

    Identifies as upstream clang via ``--version`` (so ``inspect_tool``
    classifies it as the LLVM family) and "compiles" the probe control by
    writing an executable that emits a ``.profraw`` file next to
    ``LLVM_PROFILE_FILE`` — the same observable contract the real
    ``-fprofile-instr-generate`` instrumented binary satisfies.
    """
    return _write_executable(
        directory / name,
        "#!/bin/sh\n"
        'for arg in "$@"; do\n'
        '  if [ "$arg" = "--version" ]; then\n'
        "    printf 'fake clang version 99.0.0\\n'\n"
        "    exit 0\n"
        "  fi\n"
        "done\n"
        'output=""\n'
        'previous=""\n'
        'for arg in "$@"; do\n'
        '  if [ "$previous" = "-o" ]; then output="$arg"; fi\n'
        '  previous="$arg"\n'
        "done\n"
        'if [ -n "$output" ]; then\n'
        "  printf '%s\\n' '#!/bin/sh'"
        " 'printf raw > \"$(dirname \"$LLVM_PROFILE_FILE\")/control-$$.profraw\"'"
        ' > "$output"\n'
        '  chmod +x "$output"\n'
        "fi\n"
        "exit 0\n",
    )


def _fake_profdata(directory: Path, name: str = "llvm-profdata-99", *, merge_rc: int = 0) -> Path:
    return _write_executable(
        directory / name,
        "#!/bin/sh\n"
        'case "${1:-}" in\n'
        "  --version) printf 'fake LLVM version 99.0.0\\n'; exit 0 ;;\n"
        "  merge)\n"
        f"    rc={merge_rc}\n"
        '    output=""\n'
        '    while [ "$#" -gt 0 ]; do\n'
        '      if [ "$1" = "-o" ]; then shift; output="$1"; fi\n'
        "      shift\n"
        "    done\n"
        '    if [ "$rc" -ne 0 ]; then\n'
        '      printf \'fake llvm-profdata: incompatible profile format\\n\' >&2\n'
        '      exit "$rc"\n'
        "    fi\n"
        '    [ -n "$output" ] || exit 4\n'
        '    printf \'merged profile\\n\' > "$output"\n'
        "    exit 0 ;;\n"
        "esac\n"
        "exit 5\n",
    )


def _fake_cov(
    directory: Path,
    name: str = "llvm-cov-99",
    *,
    export_payload: dict | None = None,
) -> Path:
    payload = export_payload or {
        "type": "llvm.coverage.json.export",
        "version": "99.0.0",
        "data": [{"files": [{"filename": "control.cpp", "summary": {}}]}],
    }
    return _write_executable(
        directory / name,
        "#!/bin/sh\n"
        'case "${1:-}" in\n'
        "  --version) printf 'fake LLVM version 99.0.0\\n'; exit 0 ;;\n"
        "  export) printf '%s\\n' '" + json.dumps(payload) + "'; exit 0 ;;\n"
        "esac\n"
        "exit 5\n",
    )


def _fake_toolchain(root: Path) -> tuple[Path, Path, Path]:
    """One directory holding a fake clang-family compiler and matching tools."""
    compiler = _fake_clang(root)
    profdata = _fake_profdata(root)
    cov = _fake_cov(root)
    return compiler, profdata, cov


def _minimal_path(monkeypatch: pytest.MonkeyPatch, *extra: Path) -> None:
    """PATH holding only the extra directories plus the system dirs the fake
    scripts need (``dirname``, ``chmod``)."""
    parts = [str(directory) for directory in extra] + ["/usr/bin", "/bin"]
    monkeypatch.setenv("PATH", os.pathsep.join(parts))


def test_explicit_missing_llvm_cov_override_fails(tmp_path, monkeypatch) -> None:
    """An explicit LLVM_COV override that does not resolve is a hard error —
    never a silent fallthrough to a discoverable tool."""
    compiler, _profdata, sibling_cov = _fake_toolchain(tmp_path / "toolchain")
    _minimal_path(monkeypatch)

    with pytest.raises(ValueError, match="LLVM_COV"):
        resolve_coverage_tools(compiler, {"LLVM_COV": str(tmp_path / "missing-llvm-cov")})

    # Without the override the sibling would have resolved — the override's
    # failure is never masked by it.
    assert sibling_cov.is_file()


def test_linux_tool_search_never_invokes_xcrun(tmp_path, monkeypatch) -> None:
    """On a Linux host the xcrun fallback must not exist — even when no tool
    resolves and even when a fake xcrun could 'find' one."""
    monkeypatch.setattr(native_toolchain, "_host_platform", lambda: "Linux")
    calls: list[str] = []

    def _recording_xcrun(name: str) -> Path:
        calls.append(name)
        return _fake_profdata(tmp_path / "xcrun-only", name)

    monkeypatch.setattr(native_toolchain, "_darwin_xcrun_find", _recording_xcrun)

    compiler, profdata, cov = _fake_toolchain(tmp_path / "toolchain")
    _minimal_path(monkeypatch)  # nothing on PATH — siblings must resolve
    result = resolve_coverage_tools(compiler, {})
    assert result["llvm_profdata"]["path"] == str(profdata)
    assert result["llvm_cov"]["path"] == str(cov)
    assert result["llvm_profdata"]["source"] == "sibling"
    assert calls == []

    # A compiler with no siblings and an empty PATH fails on Linux without
    # ever asking xcrun.
    bare = _fake_clang(tmp_path / "bare", "clang++-77")
    monkeypatch.setenv("PATH", str(tmp_path / "empty"))
    with pytest.raises(ValueError, match="llvm-profdata"):
        resolve_coverage_tools(bare, {})
    assert calls == []


def test_compiler_sibling_tools_precede_path_candidates(tmp_path, monkeypatch) -> None:
    """The compiler's own toolchain directory beats unrelated PATH tools."""
    compiler, sibling_profdata, sibling_cov = _fake_toolchain(tmp_path / "toolchain")
    other = tmp_path / "other-bin"
    _fake_profdata(other, "llvm-profdata")
    _fake_cov(other, "llvm-cov")
    _minimal_path(monkeypatch, other)

    result = resolve_coverage_tools(compiler, {})

    assert result["llvm_profdata"]["path"] == str(sibling_profdata)
    assert result["llvm_cov"]["path"] == str(sibling_cov)
    assert result["llvm_profdata"]["source"] == "sibling"
    assert result["llvm_cov"]["source"] == "sibling"


def test_explicit_override_wins_over_sibling(tmp_path, monkeypatch) -> None:
    compiler, _sib_profdata, _sib_cov = _fake_toolchain(tmp_path / "toolchain")
    chosen = tmp_path / "chosen"
    profdata = _fake_profdata(chosen, "llvm-profdata")
    cov = _fake_cov(chosen, "llvm-cov")
    _minimal_path(monkeypatch)

    result = resolve_coverage_tools(
        compiler, {"LLVM_PROFDATA": str(profdata), "LLVM_COV": str(cov)}
    )

    assert result["llvm_profdata"]["path"] == str(profdata)
    assert result["llvm_cov"]["path"] == str(cov)
    assert result["llvm_profdata"]["source"] == "override"
    assert result["llvm_cov"]["source"] == "override"


def test_xcrun_is_a_darwin_only_last_resort(tmp_path, monkeypatch) -> None:
    """Simulated Darwin: sibling and PATH resolution fail first, then the
    xcrun fallback supplies the toolchain-matched Xcode tools."""
    monkeypatch.setattr(native_toolchain, "_host_platform", lambda: "Darwin")
    compiler = _fake_clang(tmp_path / "bare")
    _minimal_path(monkeypatch)
    answers = {
        "llvm-profdata": _fake_profdata(tmp_path / "xcrun-tools", "llvm-profdata"),
        "llvm-cov": _fake_cov(tmp_path / "xcrun-tools", "llvm-cov"),
    }
    recorded: list[str] = []
    monkeypatch.setattr(
        native_toolchain, "_darwin_xcrun_find", lambda name: recorded.append(name) or answers[name]
    )

    result = resolve_coverage_tools(compiler, {})

    assert recorded == ["llvm-profdata", "llvm-cov"]
    assert result["llvm_profdata"]["source"] == "xcrun"
    assert result["format_probe"]["format_probe_passed"] is True


def test_non_clang_compiler_is_rejected(tmp_path, monkeypatch) -> None:
    """llvm-cov reads the clang instrumentation format — a GNU compiler has
    no place in the qualified coverage lane."""
    gcc = _write_executable(
        tmp_path / "g++-13",
        "#!/bin/sh\n"
        'if [ "${1:-}" = "--version" ]; then\n'
        "  printf 'g++ (GCC) 13.3.0\\n'\n"
        "  exit 0\n"
        "fi\n"
        "exit 0\n",
    )
    _minimal_path(monkeypatch)

    with pytest.raises(ValueError, match="clang"):
        resolve_coverage_tools(gcc, {})


def test_missing_compiler_fails(tmp_path) -> None:
    with pytest.raises(ValueError, match="compiler"):
        resolve_coverage_tools(tmp_path / "no-such-compiler", {})


def test_unknown_override_keys_fail(tmp_path, monkeypatch) -> None:
    compiler, _profdata, _cov = _fake_toolchain(tmp_path / "toolchain")
    _minimal_path(monkeypatch)
    with pytest.raises(ValueError, match="override"):
        resolve_coverage_tools(compiler, {"LLVM_SOMETHING_ELSE": "/usr/bin/true"})


def test_failing_merge_tool_fails_with_command_evidence(tmp_path, monkeypatch) -> None:
    """A resolved pair whose profdata cannot merge the compiler's profile is
    a qualification failure naming the command — never an empty success."""
    root = tmp_path / "toolchain"
    compiler = _fake_clang(root)
    _fake_profdata(root, merge_rc=7)
    _fake_cov(root)
    _minimal_path(monkeypatch)

    with pytest.raises(ValueError) as captured:
        resolve_coverage_tools(compiler, {})

    message = str(captured.value)
    assert "merge" in message
    assert "llvm-profdata-99" in message
    assert "7" in message


def test_export_without_valid_control_data_fails(tmp_path, monkeypatch) -> None:
    root = tmp_path / "toolchain"
    compiler = _fake_clang(root)
    _fake_profdata(root)
    _fake_cov(root, export_payload={"type": "something-else", "data": []})
    _minimal_path(monkeypatch)

    with pytest.raises(ValueError, match="valid control data"):
        resolve_coverage_tools(compiler, {})


def test_probe_result_records_tool_versions_and_export(tmp_path, monkeypatch) -> None:
    compiler, profdata, cov = _fake_toolchain(tmp_path / "toolchain")
    _minimal_path(monkeypatch)

    result = resolve_coverage_tools(compiler, {})

    assert result["format_probe"]["format_probe_passed"] is True
    assert result["format_probe"]["export_version"] == "99.0.0"
    assert result["llvm_profdata"]["version"]
    assert result["llvm_cov"]["version"]
    assert result["compiler"]["path"] == str(compiler)


# ---------------------------------------------------------------------------
# Real-toolchain control — the genuine compile/run/merge/export chain.
# ---------------------------------------------------------------------------


def _host_clang() -> Path | None:
    """A real clang-family C++ driver on this host, if one exists."""
    names = ["clang++", "c++", "/usr/bin/clang++", "/opt/homebrew/opt/llvm/bin/clang++"]
    for directory in ("/usr/bin", "/usr/local/bin", "/opt/homebrew/bin", "/opt/homebrew/opt/llvm/bin"):
        root = Path(directory)
        if root.is_dir():
            names.extend(str(path) for path in sorted(root.glob("clang++-*"), reverse=True))
    for name in names:
        located = Path(name) if Path(name).is_file() else (
            Path(found) if (found := shutil.which(name)) else None
        )
        if located is None:
            continue
        try:
            if native_toolchain.inspect_tool(located)["family"] == "clang":
                return located
        except ValueError:
            continue
    return None


def test_real_toolchain_resolves_and_passes_the_format_probe() -> None:
    """On a host with a clang compiler and any discoverable llvm-cov/
    llvm-profdata pair, the real instrumented control merges and exports.
    Hosts without a coverage toolchain skip rather than fake a pass."""
    compiler = _host_clang()
    if compiler is None:
        pytest.skip("no clang-family C++ compiler on this host")
    try:
        result = resolve_coverage_tools(compiler, {})
    except ValueError as error:
        pytest.skip(f"no resolvable LLVM coverage toolchain here: {error}")

    assert result["format_probe"]["format_probe_passed"] is True
    assert Path(result["llvm_profdata"]["path"]).is_file()
    assert Path(result["llvm_cov"]["path"]).is_file()
    assert result["llvm_profdata"]["version"]
    assert result["llvm_cov"]["version"]
    # The platform-fallback contract: xcrun may only appear on Darwin.
    sources = {result["llvm_profdata"]["source"], result["llvm_cov"]["source"]}
    if platform.system() != "Darwin":
        assert "xcrun" not in sources
