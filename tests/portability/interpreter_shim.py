"""Shared PATH shim for the Python-free root configure tests.

A configure that must not start an interpreter runs with a PATH whose first
entry holds logging stubs for every interpreter name. A stub appends its name
and arguments to a shared log and exits 97. The log is the evidence: it exists
only if a PATH lookup by name reached a stub. Tests import this module by name,
as tests/reference imports native_loader.
"""
from __future__ import annotations

import os
from pathlib import Path

# Every interpreter name a PATH lookup can resolve, plus uv.
INTERPRETER_NAMES = ("python", "python3", "python3.13", "python3.14", "python3.15", "uv")


def interpreter_shim_env(tmp_path: Path, tools: tuple[str, ...] = ()) -> tuple[dict[str, str], Path]:
    """Return (environment, call log) for a configure with no interpreter reachable.

    ``tools`` are absolute paths symlinked into the shim directory, so a
    configure that needs them (for example cmake and ninja) still finds them.
    """
    bin_dir = tmp_path / "interpreter-shim"
    bin_dir.mkdir()
    calls = tmp_path / "interpreter-calls.log"
    for tool in tools:
        (bin_dir / Path(tool).name).symlink_to(tool)
    for name in INTERPRETER_NAMES:
        stub = bin_dir / name
        stub.write_text(f'#!/bin/sh\necho "{name} $*" >> "{calls}"\nexit 97\n',
                        encoding="utf-8")
        stub.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(bin_dir), "/usr/bin", "/bin"])
    return env, calls


def squashed(text: str) -> str:
    """CMake wraps long messages at 80 columns; match on words, not lines."""
    return " ".join(text.split())
