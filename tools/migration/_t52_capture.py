#!/usr/bin/env python3
"""T52 one-shot capture: freeze the welded-probe initial state document.

tests/reference/test_period_buffer.py builds its research environment with

    --physics-config examples/research/viewer_physics_welded.toml
    --track-file <tmp>/flat.toml  (the inline "probe" 200 m hardpack track)
    --dt .0005 --diagnostic-model-limits

and then puts every measurement path through the identical settled-cache
reset (env.reset()). The ported C++ tests cannot re-run the Python
equilibrium solve bitwise, so they restore this captured PhysicalInitialState
document through request.setup.initial_state — the same adopt-the-pose path
initial_state_file exercises for requests/welded_flat.toml — and let
prepare_research_startup replay the identical t=0 startup on top.

The document is schema_version 2: qpos/qvel/qacc_warmstart, the road support
row, the material state arrays, the rider active_state and the equilibrium
metadata, keyed by contract_sha256 (which excludes timestep, so the same
document serves every --dt the tests pin).
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))


def main() -> int:
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride.initial_state import PhysicalInitialState

    fixture = REPO / "tests" / "fixtures" / "migration"
    track = Path("/tmp/_t52_probe_track.toml")
    track.write_text(
        'name="probe"\nlength_m=200.0\nsurface="hardpack"\n'
        'grade_profile={knots=[[0.0,0.0],[200.0,0.0]]}\n')
    args = research_cli.parser().parse_args([
        "--physics-config",
        str(fixture / "config" / "viewer_physics_welded.toml"),
        "--track-file", str(track), "--dt", ".0005",
        "--diagnostic-model-limits", "--out", "/tmp/_t52_out"])
    env = research_cli.make_environment(args)
    env.reset()
    document = PhysicalInitialState.capture(env.sim)
    # The contract hash covers compiled-model fields and the settled pose —
    # both platform-sensitive — so each platform owns its document under the
    # golden platform key (darwin-arm64, linux-aarch64).
    import platform
    destination = (fixture / "initial_state" /
                   f"{platform.system().lower()}-{platform.machine()}" /
                   "probe_welded_0005.json")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(document.payload_json + "\n")
    print("wrote", destination, destination.stat().st_size, "bytes")
    print("qpos[3] =", document.payload["qpos"][3])
    try:
        env.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
