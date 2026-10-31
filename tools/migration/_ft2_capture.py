#!/usr/bin/env python3
"""FT-2 fixture capture: the welded oracle inputs the native reference ports
replay under tests/fixtures/migration/ft2/<platform-key>/.

Every capture runs with the reference-test kernel pinning (conftest's
_pin_oracle_kernels): the C++ port compares against the deterministic
``*_py`` arithmetic, and the oracle's t=0 bootstrap must come from the same
kernels as the trajectories the goldens freeze.

Produces per platform:

    ride/                RuntimeBootstrap trio — make_python_ride(strict=False,
                         decimation=1): welded physics + rough_uphill_savage,
                         the recipe behind test_native_runtime_{step,commands,
                         rollout}.py and the welded leg of the viewer batch.
    research_flat/       ResearchBootstrap — environment_factory() defaults:
                         welded physics, flat scenario, duration .04, seed 19,
                         decimation 1, actuator delay .005, diagnostic limits,
                         capture_mode legacy. Plus provenance.json (seed,
                         metadata, experiment and sensor_config documents the
                         frozen research.json deliberately omits) and the
                         native-stamped recording assets/ the shared-export
                         port consumes.
    research_savage/     ResearchBootstrap — make_python_research(): welded
                         physics + rough_uphill_savage, duration .1, seed 210,
                         diagnostic limits, --assist, capture_mode legacy.

All JSON uses the canonical tagged codec so array payloads externalize to
sibling .arrays/ payloads exactly like the T09/T10 fixtures.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "migration"))
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tests" / "reference"))

from capture_reference import FixtureWriter, write_canonical  # noqa: E402


def _pin_kernels() -> None:
    """conftest._pin_oracle_kernels, applied process-wide for the capture."""
    from bike_sim.physics import joint_strength, tire
    from bike_sim.sim.ride import (distributed_tire_forces, rider_contacts,
                                    rider_control, support_geometry,
                                    tire_forces)
    for module in (tire, tire_forces, distributed_tire_forces, rider_contacts):
        module._brush_step = tire._brush_step_py
    for module in (tire, tire_forces, rider_contacts, rider_control):
        module._normal_contact = tire._normal_contact_py
    joint_strength._capacity_core = joint_strength._capacity_core_py
    support_geometry._box_pad_contact_core = \
        support_geometry._box_pad_contact_core_py


def _platform_key() -> str:
    import platform
    return f"{platform.system().lower()}-{platform.machine()}"


def _write_bootstrap(out: FixtureWriter, sim, stem: str, work: Path) -> list:
    from bike_sim.native.setup import capture_bootstrap
    import mujoco
    import numpy as np
    boot = capture_bootstrap(sim, work)
    files = [out.copy(f"{stem}/model.mjb", boot.model_path)]
    files += write_canonical(out, f"{stem}/config.json", boot.config)
    files += write_canonical(out, f"{stem}/state.json", boot.state)
    arec, _ = out.array(f"{stem}/integration.f64",
                        np.asarray(boot.state["integration_state"],
                                   dtype="<f8"))
    files.append(arec)
    digest = hashlib.sha256((work / "model.mjb").read_bytes()).hexdigest()
    width = int(mujoco.mj_stateSize(sim.model,
                                    mujoco.mjtState.mjSTATE_INTEGRATION))
    meta = {"model_sha256": digest,
            "model_dims": boot.state["model_dims"],
            "integration_width": width,
            "integration_sha256": hashlib.sha256(
                np.asarray(boot.state["integration_state"],
                           dtype="<f8").tobytes()).hexdigest()}
    files += write_canonical(out, f"{stem}/meta.json", meta)
    return files


def _research_documents(env, work: Path):
    """The create_native_research() staging, verbatim from research.py."""
    from bike_sim.native.setup import _control_dict
    from bike_sim.sim.research.sensor_noise import build_noise_tape
    tape = build_noise_tape(
        env.sensor_config, seed=env.seed,
        count=1 + env.max_steps // env.sensor_steps)
    sim = env.sim
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
    research = {name: getattr(env, name) for name in
                ("control_steps", "delay_steps", "max_steps",
                 "sensor_steps", "start_position_m")}
    research.update(
        timestep_s=env.dt_s,
        track_length_m=sim.track.length_m,
        capture_mode=env.capture_mode,
        record_decimation=env.config.record_decimation,
        stop_on_model_violation=env.config.stop_on_model_violation,
        maximum_energy_residual_ratio=env.config.maximum_energy_residual_ratio,
        wheelie_persistence_s=env.config.wheelie_persistence_s,
        sensors=dataclasses.asdict(env.sensor_config),
        geometry=geometry,
        startup_control=_control_dict(env._applied),
        rider_program=(None if env.rider_program is None
                       else env.rider_program.to_dict()),
        demand_program=(None if env.demand is None
                        else env.demand.to_dict()))
    return {"research": research, "noise": tape.noise,
            "dropout_uniform": tape.dropout_uniform,
            "sensor_state": env.pipeline.state_dict()}


def _write_research(out: FixtureWriter, env, stem: str, work: Path,
                    recording_assets=()) -> list:
    files = _write_bootstrap(out, env.sim, stem, work)
    files += write_canonical(out, f"{stem}/research.json",
                             _research_documents(env, work))
    from bike_sim.sim.research.observations import raw_observation
    provenance = {
        "seed_decimal": str(env.seed),
        "raw_initial": dataclasses.asdict(raw_observation(env.sim)),
        "noise_std": [env.sensor_config.acceleration_std_mps2,
                      env.sensor_config.gyro_std_rad_s,
                      env.sensor_config.encoder_std_rad_s,
                      env.sensor_config.torque_std_nm],
        "metadata": env.metadata,
        "experiment": dataclasses.asdict(env.config),
        "sensor_config": dataclasses.asdict(env.sensor_config),
        "execution": env.execution}
    files += write_canonical(out, f"{stem}/provenance.json", provenance)
    if recording_assets:
        manifest = []
        for item in recording_assets:
            files.append(out.bytes(f"{stem}/assets/{item.name}", item.content))
            manifest.append({"name": item.name, "media_type": item.media_type,
                             "sha256": item.sha256})
        files += write_canonical(out, f"{stem}/assets.json", manifest)
    return files


def _native_recording_assets(reference):
    """The adapter's native-stamped asset set for reference, when the
    selected extension is loadable; falls back to the recorded Python set
    with a native execution block built through execution_provenance."""
    try:
        from bike_sim.native.research import create_native_research
        native = create_native_research(reference)
        try:
            return native._recording_assets
        finally:
            native.close()
    except Exception as error:
        print(f"native-stamped assets unavailable ({error}); "
              "stamping via execution_provenance", file=sys.stderr)
        from bike_sim.native.artifact import execution_provenance
        from bike_sim.recording.assets import (replace_recipe,
                                               research_recipe)
        execution = execution_provenance("native")
        recipe = dict(research_recipe(reference), execution=execution)
        return replace_recipe(reference._recording_assets, recipe)


def main() -> int:
    _pin_kernels()
    import _native_runtime_support as support
    from bike_sim.cli import research as research_cli

    root = REPO / "tests" / "fixtures" / "migration" / "ft2" / _platform_key()
    out = FixtureWriter(root)
    work = REPO / "output" / "_ft2_capture"
    work.mkdir(parents=True, exist_ok=True)
    wrote = []

    # -- ride: make_python_ride(strict=False, decimation=1) ----------------
    sim = support.make_python_ride(strict=False, decimation=1)
    wrote += _write_bootstrap(out, sim, "ride", work / "ride")

    # -- research_flat: environment_factory() defaults ---------------------
    flat_args = research_cli.parser().parse_args([
        "--physics-config", str(REPO / support.PHYSICS),
        "--scenario", "flat", "--duration", "0.04", "--seed", "19",
        "--record-decimation", "1", "--actuator-delay", "0.005",
        "--diagnostic-model-limits"])
    flat_env = research_cli.make_environment(flat_args, capture_mode="legacy")
    assets = _native_recording_assets(flat_env)
    wrote += _write_research(out, flat_env, "research_flat",
                             work / "research_flat", recording_assets=assets)
    flat_env.close()

    # -- research_savage: make_python_research() ----------------------------
    savage_args = research_cli.parser().parse_args([
        "--physics-config", str(REPO / support.PHYSICS),
        "--track-file", str(REPO / support.TRACK),
        "--duration", "0.1", "--seed", "210",
        "--diagnostic-model-limits", "--assist"])
    savage_env = research_cli.make_environment(savage_args)
    wrote += _write_research(out, savage_env, "research_savage",
                             work / "research_savage",
                             recording_assets=savage_env._recording_assets)
    savage_env.close()

    for record in wrote:
        print("wrote", record["path"], record["bytes"], "bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
