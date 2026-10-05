"""PROTOYPE (throwaway): dump the compiled welded-physical model + a mid-ride state.

Builds the sim exactly like tools/measure_realtime.py (extreme track +
viewer_physics_welded, dt=0.5ms), warms up 400 steps so the state is a real
mid-ride configuration, then saves:
  - model.mjb  — compiled MjModel (incl. patched hfield_data)
  - state.npz  — qpos/qvel/act + frozen ctrl/qfrc_applied/xfrc_applied snapshot

Run: uv run python tools/proto_native_bench/dump_model.py
"""
from pathlib import Path

import mujoco
import numpy as np

OUT = Path(__file__).resolve().parent / "artifacts"
TRACK = "examples/research/rough_uphill_extreme.toml"
PHYSICS = "examples/research/viewer_physics_welded.toml"
WARMUP_STEPS = 400


def main() -> None:
    from bike_sim.cli import ride as ride_cli

    args = ride_cli.parse_args([
        "--track", TRACK, "--physics-config", PHYSICS, "--headless",
        "--no-plots", "--duration", "1.", "--decimate", "80",
        "--timestep", ".0005", "--out", str(OUT / "run"),
    ])
    sim = ride_cli.build_physical_simulation_from_args(args)
    sim.physical.set_record_decimation(args.decimate)
    sim.physical.reference_monitor.strict = False

    for _ in range(WARMUP_STEPS):
        sim.step()
        if sim.crash is not None:
            raise RuntimeError("crashed during warmup")

    model = sim.model
    data = getattr(sim, "data", None)
    if data is None:
        # physical path may keep mjData inside the runtime; find it
        for attr in ("physical", "runtime"):
            sub = getattr(sim, attr, None)
            data = getattr(sub, "data", None) if sub is not None else None
            if data is not None:
                print(f"mjData found at sim.{attr}.data")
                break
    if data is None:
        raise RuntimeError(f"no MjData on sim: {[a for a in dir(sim) if 'data' in a.lower()]}")

    OUT.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(OUT / "model.mjb"))
    np.savez(
        OUT / "state.npz",
        qpos=data.qpos.copy(), qvel=data.qvel.copy(), act=data.act.copy(),
        ctrl=data.ctrl.copy(), qfrc_applied=data.qfrc_applied.copy(),
        xfrc_applied=data.xfrc_applied.copy(),
        time=np.array([data.time]),
    )
    print(f"model: nv={model.nv} nq={model.nq} nu={model.nu} nbody={model.nbody} "
          f"neq={model.neq} nsite={model.nsite} nhfield={model.nhfield}")
    print(f"state @ t={data.time:.3f}s: qpos[0..3]={data.qpos[:4]}")
    print(f"dt={model.opt.timestep}  saved to {OUT}")


if __name__ == "__main__":
    main()
