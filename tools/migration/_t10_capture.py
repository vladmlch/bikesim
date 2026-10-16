#!/usr/bin/env python3
"""T10 one-shot capture: freeze the research bootstrap documents.

Builds the same Python research environment _rcp03_research captured for
research/delayed_bootstrap (welded physics, flat track, --actuator-delay
0.01, seed 210, --diagnostic-model-limits, --assist) and writes the inputs
create_native_research() would stage for extension.NativeResearchRuntime:

    config.json / state.json / model.mjb   — capture_bootstrap(sim) output
    research.json                          — {"research", "noise",
                                              "dropout_uniform",
                                              "sensor_state"}

All JSON uses the canonical tagged codec (encode_tree/write_canonical from
capture_reference.py) so array payloads externalize to sibling .arrays/
payload files exactly like the ride bootstrap fixtures.
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "migration"))
sys.path.insert(0, str(REPO / "src"))

from capture_reference import FixtureWriter, write_canonical  # noqa: E402


def main() -> int:
    from bike_sim.cli import research as research_cli
    from bike_sim.native.setup import capture_bootstrap, _control_dict
    from bike_sim.sim.research.sensor_noise import build_noise_tape

    root = REPO / "tests" / "fixtures" / "migration"
    out = FixtureWriter(root)

    args = research_cli.parser().parse_args([
        "--physics-config", str(root / "config/viewer_physics_welded.toml"),
        "--track-file", str(root / "tracks/flat.toml"),
        "--duration", "0.1", "--seed", "210",
        "--actuator-delay", "0.01",
        "--diagnostic-model-limits", "--assist"])
    env = research_cli.make_environment(args)
    reference = env

    stem = "research/delayed_bootstrap"
    work = REPO / "output" / "_t10_capture"
    work.mkdir(parents=True, exist_ok=True)

    # --- the create_native_research input staging, verbatim -------------
    tape = build_noise_tape(
        reference.sensor_config, seed=reference.seed,
        count=1 + reference.max_steps // reference.sensor_steps)
    sim = reference.sim
    geometry = dict(
        vertices=sim.physical.vertices,
        front_radius=float(
            sim.model.geom_size[sim.contact_query.front_id, 0]),
        rear_radius=float(
            sim.model.geom_size[sim.contact_query.rear_id, 0]),
        root_x_qpos=sim.root_x_qposadr,
        root_x_dof=sim.root_x_dofadr,
        root_pitch_qpos=sim.root_pitch_qposadr,
        root_pitch_dof=sim.physical.address("root_pitch")[1])
    research = {name: getattr(reference, name) for name in
                ("control_steps", "delay_steps", "max_steps",
                 "sensor_steps", "start_position_m")}
    research.update(
        timestep_s=reference.dt_s,
        track_length_m=sim.track.length_m,
        capture_mode=reference.capture_mode,
        record_decimation=reference.config.record_decimation,
        stop_on_model_violation=reference.config.stop_on_model_violation,
        maximum_energy_residual_ratio=(
            reference.config.maximum_energy_residual_ratio),
        wheelie_persistence_s=reference.config.wheelie_persistence_s,
        sensors=dataclasses.asdict(reference.sensor_config),
        geometry=geometry,
        startup_control=_control_dict(reference._applied),
        rider_program=(None if reference.rider_program is None
                       else reference.rider_program.to_dict()),
        demand_program=(None if reference.demand is None
                        else reference.demand.to_dict()))

    boot = capture_bootstrap(sim, work)

    files = [out.copy(f"{stem}/model.mjb", boot.model_path)]
    files += write_canonical(out, f"{stem}/config.json", boot.config)
    files += write_canonical(out, f"{stem}/state.json", boot.state)
    files += write_canonical(out, f"{stem}/research.json", {
        "research": research,
        "noise": tape.noise,
        "dropout_uniform": tape.dropout_uniform,
        "sensor_state": reference.pipeline.state_dict()})

    for record in files:
        print("wrote", record["path"], record["bytes"], "bytes")
    try:
        env.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
