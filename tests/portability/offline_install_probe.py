#!/usr/bin/env python3
"""End-to-end offline-install probe (T25 step 4 driver).

Selects exactly one schema-1 bundle matching this host/Python/--mode from
``--bundle`` (a directory of extracted bundles and/or ``*.tar.gz`` archives —
a single extracted bundle directory is also accepted).  The selected bundle
is copied or extracted into a working directory whose name contains a space,
then installed through the real ``packaging/install_{full,slim}.sh`` entry
points — once at the first location and once after relocating the extracted
source — each time into a fresh venv path containing spaces.  The installer
itself uses an isolated uv cache, so no warm cache can satisfy the install.

For native bundles the probe additionally:

* asserts ``bike_sim.native.artifact.load_native_extension`` resolves the
  extension inside the freshly built ``native/build/release`` of the
  *current* bundle location (a copied or stale build directory is rejected
  by the loader's own identity checks);
* deletes the relocated bundle's previous build directory before the second
  install so the second artifact is provably new;
* runs ``ctest --test-dir`` on the built directory;
* with ``--record``, performs a short headless native ride that publishes a
  run recording.

This harness proves the local machinery.  It does NOT isolate the network:
true air-gapped qualification needs a network-blocked runner (Linux
container/VM or dedicated macOS runner), which is reported separately.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile


def host_identity() -> dict:
    system = platform.system()
    machine = platform.machine()
    machine = {"AMD64": "x86_64",
               "aarch64": "arm64" if system == "Darwin" else "aarch64"}.get(machine, machine)
    return {
        "system": system,
        "machine": machine,
        "python_minor": f"{sys.version_info.major}.{sys.version_info.minor}",
        "implementation": platform.python_implementation().lower(),
    }


def _manifest_matches(manifest: dict, host: dict, mode: str) -> bool:
    target = manifest.get("target")
    if not isinstance(target, dict):
        return False
    return (
        manifest.get("mode") == mode
        and target.get("system") == host["system"]
        and target.get("machine") == host["machine"]
        and target.get("python_minor") == host["python_minor"]
        and target.get("implementation") == host["implementation"]
    )


def iter_candidates(root: Path):
    """Yield (kind, path, manifest) for extracted dirs and .tar.gz members."""
    dirs = [root] if (root / "manifest.json").is_file() else sorted(
        path.parent for path in root.glob("*/manifest.json"))
    for directory in dirs:
        try:
            manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        yield "dir", directory, manifest
    for archive in sorted(root.glob("*.tar.gz")):
        manifest = None
        try:
            with tarfile.open(archive) as tar:
                for member in tar.getnames():
                    parts = member.split("/")
                    if len(parts) == 2 and parts[1] == "manifest.json":
                        extracted = tar.extractfile(member)
                        manifest = json.loads(extracted.read().decode("utf-8"))
                        break
        except (OSError, tarfile.TarError, json.JSONDecodeError):
            continue
        if manifest is not None:
            yield "tar", archive, manifest


def select_bundle(root: Path, mode: str) -> tuple[str, Path, dict]:
    host = host_identity()
    matching = [candidate for candidate in iter_candidates(root)
                if _manifest_matches(candidate[2], host, mode)]
    if not matching:
        raise ValueError(
            f"no bundle manifest matches host {host['system']}/{host['machine']} "
            f"CPython {host['python_minor']} in mode {mode!r} under {root}")
    if len(matching) > 1:
        names = [str(candidate[1]) for candidate in matching]
        raise ValueError(f"ambiguous bundle selection under {root}: {names}")
    return matching[0]


def _run(argv: list, *, env: dict, cwd: Path | None = None) -> None:
    result = subprocess.run([str(item) for item in argv],
                            cwd=None if cwd is None else str(cwd), env=env)
    if result.returncode != 0:
        raise ValueError(f"command failed (exit {result.returncode}): "
                         + " ".join(str(item) for item in argv))


def _query(venv_python: Path, code: str, *, env: dict) -> str:
    result = subprocess.run([str(venv_python), "-c", code], env=env,
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise ValueError(f"probe query failed: {result.stderr.strip()}")
    return result.stdout.strip()


def _install(bundle_dir: Path, venv: Path, mode: str, env: dict) -> None:
    script = (bundle_dir / "project" / "packaging"
              / f"install_{mode}.sh").resolve()
    if not script.is_file():
        raise ValueError(f"bundle lacks {script.relative_to(bundle_dir)}")
    _run(["bash", script, venv.resolve()], env=env, cwd=bundle_dir)


def _python_evidence(venv: Path, bundle_dir: Path, native: bool,
                     env: dict) -> dict:
    output = _query(venv / "bin" / "python",
                    "import bike_sim, json, sys; "
                    "print(json.dumps({'executable': sys.executable, "
                    "'package': bike_sim.__file__}))",
                    env=env)
    evidence = json.loads(output)
    package = Path(evidence["package"]).resolve()
    if native:
        # Source/editable import: the loaded package must live in the bundle's
        # project source tree, not a copied wheel.
        expected = (bundle_dir / "project" / "src").resolve()
        if expected not in package.parents:
            raise ValueError(
                f"native install must import bike_sim from {expected}; "
                f"loaded {package}")
    else:
        expected = venv.resolve()
        if expected not in package.parents:
            raise ValueError(
                f"wheel install must import bike_sim from inside {expected}; "
                f"loaded {package}")
    return evidence


def _native_evidence(venv: Path, build: Path, env: dict) -> dict:
    env = dict(env, BIKE_NATIVE_BUILD_PATH=str(build))
    output = _query(
        venv / "bin" / "python",
        "from bike_sim.native import artifact\n"
        "module = artifact.load_native_extension()\n"
        "print(module.__file__)",
        env=env)
    expected_prefix = str(build.resolve())
    if not output.startswith(expected_prefix):
        raise ValueError(
            f"imported extension {output} is not inside the selected build "
            f"directory {expected_prefix}")
    return {"extension": output}


def _relocate(bundle_dir: Path, work: Path, index: int) -> Path:
    destination = work / f"relocated bundle {index}"
    shutil.move(str(bundle_dir), str(destination))
    return destination


def _prepare_selected(kind: str, path: Path, work: Path) -> Path:
    destination = work / "extracted bundle 0"
    if kind == "dir":
        shutil.copytree(path, destination, symlinks=False)
    else:
        destination.mkdir(parents=True)
        with tarfile.open(path) as tar:
            tar.extractall(destination, filter="data")
        children = [child for child in destination.iterdir() if child.is_dir()]
        if len(children) != 1:
            raise ValueError(f"archive {path} did not extract to one root")
        destination = children[0]
    return destination


def run_probe(bundle_root: Path, mode: str, work: Path | None, record: bool) -> dict:
    kind, source, manifest = select_bundle(bundle_root, mode)
    native = bool(manifest.get("capabilities", {}).get("native_source_build"))
    root = work or Path(tempfile.mkdtemp(prefix="offline install probe "))
    root.mkdir(parents=True, exist_ok=True)
    root = root.resolve()
    bundle_dir = _prepare_selected(kind, source, root)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("UV_", "PIP_", "VIRTUAL_ENV"))}
    # Fresh, empty package-manager caches for every run: no warm-cache reuse.
    # Probe queries must not write __pycache__ into the bundle payload either.
    env["UV_CACHE_DIR"] = str(root / "uv cache")
    env["PYTHONDONTWRITEBYTECODE"] = "1"

    report: dict = {
        "schema_version": 1, "mode": mode, "native_source_build": native,
        "selected": str(source), "work": str(root), "installs": [],
        "network_isolation": "not enforced by this probe — real air-gapped "
                             "qualification requires a network-blocked harness",
    }
    for index in range(2):
        venv = root / f"venv dir {index}"
        build = bundle_dir / "project" / "native" / "build" / "release"
        if index:
            stale = bundle_dir / "project" / "native" / "build"
            if stale.exists():
                shutil.rmtree(stale)
            if (root / "uv cache").exists():
                shutil.rmtree(root / "uv cache")
        _install(bundle_dir, venv, mode, env)
        entry: dict = {"bundle": str(bundle_dir), "venv": str(venv)}
        install_report = json.loads(
            (venv / "bikesim-install-report.json").read_text(encoding="utf-8"))
        if install_report["mode"] != mode or install_report["network_used"]:
            raise ValueError(
                f"install report contradicts the run: {install_report}")
        if not install_report.get("verify", {}).get("passed"):
            raise ValueError("install report verification did not pass")
        entry["report"] = {key: install_report[key] for key in
                           ("mode", "native_source_build", "uv", "activation",
                            "build", "slim_external")}
        entry["python"] = _python_evidence(venv, bundle_dir, native, env)
        if native:
            entry["native"] = _native_evidence(venv, build, env)
            _run(["ctest", "--test-dir", build, "--output-on-failure"], env=env)
            entry["ctest"] = "passed"
        report["installs"].append(entry)
        if index == 0:
            bundle_dir = _relocate(bundle_dir, root, index)
    if native and record:
        out = root / "run recording"
        project = bundle_dir / "project"
        build = project / "native" / "build" / "release"
        env = dict(env, BIKE_NATIVE_BUILD_PATH=str(build))
        # ride exit codes: 0 clean, 2 recorded model violations (the recording
        # still publishes); anything else is an infrastructure failure.
        result = subprocess.run(
            [str(root / "venv dir 1" / "bin" / "bike-ride"),
             "--physics", "physical", "--backend", "native",
             "--physics-config",
             str(project / "examples/research/viewer_physics_welded.toml"),
             "--track",
             str(project / "examples/research/rough_uphill_savage.toml"),
             "--headless", "--no-plots", "--duration", "5",
             "--check-policy", "diagnostic", "--record", "--out", str(out)],
            env=env, capture_output=True, text=True)
        if result.returncode not in (0, 2):
            raise ValueError(
                f"native recording run failed (exit {result.returncode}): "
                f"{result.stderr.strip()}")
        artifacts = [path for path in out.rglob("run.duckdb")]
        if not artifacts:
            raise ValueError(f"no run recording published under {out}")
        # The recording must cover the requested duration, not a run that
        # terminated at the first step: the published manifest carries the
        # finalization status and per-channel row counts.
        manifests = [path for path in out.rglob("manifest.json")]
        finalized = False
        for manifest_path in manifests:
            data = json.loads(manifest_path.read_text())
            if data.get("report_status") != "complete":
                continue
            summary = manifest_path.with_name("summary.md")
            # A run terminated at the first step still finalizes; require a
            # completion reason that proves the native backend stepped the
            # requested duration.
            if summary.is_file() and any(
                    reason in summary.read_text()
                    for reason in ("duration_reached", "end_of_track")):
                finalized = True
                break
        if not finalized:
            raise ValueError(
                f"no duration-completed run recording under {out}")
        report["recording"] = {
            "out": str(out), "exit_code": result.returncode,
            "run_duckdb": [str(path) for path in artifacts],
            "manifest": [str(path) for path in manifests],
            "tail": result.stderr.strip().splitlines()[-1:]
            or result.stdout.strip().splitlines()[-1:],
        }
    report["status"] = "passed"
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True,
                        help="directory of extracted bundles/*.tar.gz, or one bundle")
    parser.add_argument("--mode", choices=("full", "slim"), required=True)
    parser.add_argument("--work", type=Path, default=None,
                        help="working directory (default: a fresh path containing spaces)")
    parser.add_argument("--record", action="store_true",
                        help="opt-in short native recording after the second install")
    args = parser.parse_args(argv)
    try:
        report = run_probe(args.bundle, args.mode, args.work, args.record)
    except (OSError, ValueError, tarfile.TarError, shutil.Error) as error:
        print(f"offline install probe: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
