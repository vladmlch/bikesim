#!/usr/bin/env python3
"""F5 one-shot capture: the pinned spindle plant + wire config fixture.

Writes tests/fixtures/migration/spindle/pinned/{model.mjb,config.json} —
the exact artifacts tests/reference/test_native_spindle_controller.py
_plant() builds (pinned TOML -> articulated config -> generate_mujoco_xml
+ rider_controller_config/pose wire documents), so the C++ port
native/tests/reference/test_native_spindle_controller.cpp constructs the
identical SpindleController over the identical compiled model.

Re-run on either qualified platform when the pinned plant recipe changes;
the .mjb is a byte-stable serialization of the compiled mjModel.
"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "reference"))


def main() -> int:
    from dataclasses import asdict

    import mujoco
    from test_pinned_topology import _pinned_config

    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.native.setup import rider_controller_config
    from bike_sim.physics.resolution import load_physics_config
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    from bike_sim.sim.ride.physical_samples import plain
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController

    with tempfile.TemporaryDirectory() as tmp:
        cfg = load_physics_config(_pinned_config(Path(tmp)))
        specs = BikeSpecs()
        rider = RiderSpecs(variant="articulated_planar")
        pose = geometry_pose(rider, specs)
        model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
            specs, mode="ride", rider=rider, crank_joint=True,
            physics_config=cfg))
        crank_m = specs.crank_length / 1000.
        controller = ArticulatedRiderController(
            model, pose, cfg.articulated, crank_m)
        wire = {
            "rider_controller": rider_controller_config(
                cfg.articulated, controller),
            "pose": plain(asdict(pose)),
            "crank_length_m": crank_m,
        }
        out = ROOT / "tests" / "fixtures" / "migration" / "spindle" / "pinned"
        out.mkdir(parents=True, exist_ok=True)
        mjb = out / "model.mjb"
        mujoco.mj_saveModel(model, str(mjb))
        (out / "config.json").write_text(
            json.dumps(wire, indent=2, sort_keys=True) + "\n")
        print("wrote", mjb, mjb.stat().st_size, "bytes")
        print("wrote", out / "config.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
