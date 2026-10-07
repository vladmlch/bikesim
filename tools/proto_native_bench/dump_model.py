"""Dump the compiled welded-physical model + a mid-ride state as a
versioned BIKEST02 bundle (native-safety plan task R3).

Builds the sim exactly like tools/measure_realtime.py (extreme track +
viewer_physics_welded, dt=0.5ms), warms up so the snapshot is a real
mid-ride configuration, then calls state_format.write_bundle:

    <out>/<name>/model.mjb     compiled MjModel (incl. patched hfield_data)
    <out>/<name>/state.npz     qpos/qvel/act + frozen ctrl/qfrc_applied/
                              xfrc_applied + qacc_warmstart snapshot
    <out>/<name>/state.bin     BIKEST02 header + binary64 payload
    <out>/<name>/manifest.json dims, timestep, source, checksums, workload
    <out>/current.json         atomically published bundle pointer

The snapshot is captured once and every file is re-read and validated
before current.json is replaced, so consumers either see the previous
complete generation or the new one — never a mix.

Run:
  uv run python tools/proto_native_bench/dump_model.py \
      [--out DIR] [--name NAME] [--warmup-steps N] [--duration S]
      [--timestep S] [--decimate N]
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
from state_format import write_bundle  # noqa: E402

OUT = Path(__file__).resolve().parent / "artifacts"
TRACK = "examples/research/rough_uphill_extreme.toml"
PHYSICS = "examples/research/viewer_physics_welded.toml"
WARMUP_STEPS = 400


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="dump_model.py",
        description="Generate a versioned benchmark state bundle",
    )
    parser.add_argument("--out", type=Path, default=OUT,
                        help="artifacts root (bundle dir + current.json)")
    parser.add_argument("--name", default=None,
                        help="bundle directory name (default: v2-<utc>-*)")
    parser.add_argument("--track", default=TRACK)
    parser.add_argument("--physics-config", default=PHYSICS)
    parser.add_argument("--warmup-steps", type=int, default=WARMUP_STEPS)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--timestep", type=float, default=0.0005)
    parser.add_argument("--decimate", type=int, default=80)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    from bike_sim.cli import ride as ride_cli

    args = _parse_args(argv if argv is not None else sys.argv[1:])

    ride_args = ride_cli.parse_args([
        "--track", args.track, "--physics-config", args.physics_config,
        "--headless", "--no-plots", "--duration", str(args.duration),
        "--decimate", str(args.decimate), "--timestep", str(args.timestep),
        "--out", str(args.out / "run"),
    ])
    sim = ride_cli.build_physical_simulation_from_args(ride_args)
    sim.physical.set_record_decimation(ride_args.decimate)
    sim.physical.reference_monitor.strict = False

    for _ in range(args.warmup_steps):
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
        raise RuntimeError(
            f"no MjData on sim: {[a for a in dir(sim) if 'data' in a.lower()]}"
        )

    bundle = write_bundle(
        args.out,
        model,
        data,
        name=args.name,
        source_track=args.track,
        source_config=args.physics_config,
        warmup_steps=args.warmup_steps,
    )
    print(f"model: nv={model.nv} nq={model.nq} nu={model.nu} "
          f"nbody={model.nbody} neq={model.neq} nsite={model.nsite} "
          f"nhfield={model.nhfield}")
    print(f"state @ t={data.time:.3f}s: qpos[0..3]={data.qpos[:4]}")
    print(f"dt={model.opt.timestep}  bundle: {bundle}")
    print(f"pointer: {args.out / 'current.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
