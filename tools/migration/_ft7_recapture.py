#!/usr/bin/env python3
"""FT-7 verified recapture — T09/T10 frozen-oracle fixtures.

The committed `bootstrap/welded` and `research/delayed_bootstrap` oracles
were captured at reference/python-oracle (4667639c) before the reviewed
welded-plant physics changes (F1 ec9b031e + fix rounds, FT-3 96dad67c /
a0e35355, b21c63ec PHASE 3). The native port mirrors the *current* oracle —
live-oracle parity was proven BEFORE this recapture:

  * T09: NativeRideRuntime(capture_bootstrap(fresh welded sim)).advance(4,
    RideControl(motor=25, human=10)) == live advance_python x4 — 0/292
    elements divergent at atol=rtol=1e-9; t=0 restore bitwise-exact.
  * T10: NativeResearchRuntime on the fresh _t10 staging == the live env's
    3 env.step(RideControl(motor=10)) transitions + 4 observations —
    0 divergent fields at 1e-9.

This script re-runs the SAME recipes the T01/T10 captures used
(RCP-01 `_bootstrap_payload` + steps_control + reset pair; `_t10_capture`
staging + RCP-03 env/transitions) under the current oracle, writing the
canonical tagged codec through capture_reference.FixtureWriter — identical
file layout to the committed fixtures.

Run from the repository root:

    PYTHONPATH=tests/reference uv run --frozen --group native \
        python tools/migration/_ft7_recapture.py
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "migration"))
sys.path.insert(0, str(REPO / "tests" / "reference"))

import mujoco  # noqa: E402

from capture_reference import (FixtureWriter, _stable_repr,  # noqa: E402
                               write_canonical)

ROOT = REPO / "tests" / "fixtures" / "migration"


def _fresh_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)


def bootstrap_payload(out: FixtureWriter, sim, stem: str,
                      *, extras: bool = True) -> dict:
    """_bootstrap_payload verbatim: capture_bootstrap + canonical docs.

    ``extras`` keeps the ride-fixture's integration.f64/meta.json siblings;
    the research staging (create_native_research inputs) carries neither.
    """
    from bike_sim.native.setup import capture_bootstrap
    work = Path(tempfile.mkdtemp(prefix="ft7_boot_"))
    boot = capture_bootstrap(sim, work)
    files = [out.copy(f"{stem}/model.mjb", boot.model_path)]
    files += write_canonical(out, f"{stem}/config.json", boot.config)
    files += write_canonical(out, f"{stem}/state.json", boot.state)
    meta = None
    if extras:
        arec, _ = out.array(f"{stem}/integration.f64",
                            boot.state["integration_state"])
        files.append(arec)
        digest = hashlib.sha256(boot.model_path.read_bytes()).hexdigest()
        meta = {"model_sha256": digest,
                "model_dims": boot.state["model_dims"],
                "integration_width": len(boot.state["integration_state"])}
        files += write_canonical(out, f"{stem}/meta.json", meta)
    digest = hashlib.sha256(boot.model_path.read_bytes()).hexdigest()
    return {"files": files, "meta": meta, "model_sha256": digest}


def capture_welded(out: FixtureWriter) -> None:
    import _native_runtime_support as support
    from bike_sim.sim.ride.control import RideControl

    stem_dir = ROOT / "bootstrap" / "welded"
    _fresh_dir(stem_dir / "state.arrays")
    _fresh_dir(stem_dir / "reset")

    sim = support.make_python_ride(strict=False, decimation=1)
    payload = bootstrap_payload(out, sim, "bootstrap/welded")
    print(f"welded bootstrap: model sha256 "
          f"{payload['meta']['model_sha256'][:16]}…")

    control = RideControl(motor_torque_nm=25.0, human_torque_nm=10.0)
    rows = support.advance_python(sim, 4, control=control)
    write_canonical(out, "bootstrap/welded/steps_control.json",
                    {"control": dataclasses.asdict(control), "rows": rows})
    width = mujoco.mj_stateSize(sim.model,
                              mujoco.mjtState.mjSTATE_INTEGRATION)
    post = np.empty(width, dtype=np.float64)
    mujoco.mj_getState(sim.model, sim.data, post,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    out.array("bootstrap/welded/steps_control_integration.f64", post)
    print(f"welded steps_control: {len(rows)} rows")

    for idx in (1, 2):
        sim.reset()
        bootstrap_payload(out, sim, f"bootstrap/welded/reset/{idx}")
    print("welded reset/1 reset/2 captured")


def capture_research(out: FixtureWriter) -> None:
    from bike_sim.cli import research as research_cli
    from bike_sim.native.setup import _control_dict
    from bike_sim.sim.research.sensor_noise import build_noise_tape
    from bike_sim.sim.ride.control import RideControl

    stem_dir = ROOT / "research" / "delayed_bootstrap"
    _fresh_dir(stem_dir / "state.arrays")
    _fresh_dir(stem_dir / "research.arrays")

    args = research_cli.parser().parse_args([
        "--physics-config", str(ROOT / "config/viewer_physics_welded.toml"),
        "--track-file", str(ROOT / "tracks/flat.toml"),
        "--duration", "0.1", "--seed", "210",
        "--actuator-delay", "0.01",
        "--diagnostic-model-limits", "--assist"])
    env = research_cli.make_environment(args)
    reference = env

    stem = "research/delayed_bootstrap"
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

    payload = bootstrap_payload(out, sim, stem, extras=False)
    write_canonical(out, f"{stem}/research.json", {
        "research": research,
        "noise": tape.noise,
        "dropout_uniform": tape.dropout_uniform,
        "sensor_state": reference.pipeline.state_dict()})
    print(f"research bootstrap: model sha256 "
          f"{payload['model_sha256'][:16]}…")

    # RCP-03 env metadata + transitions oracle.
    sensor_cfg = (dataclasses.asdict(env.sensor_config)
                  if dataclasses.is_dataclass(env.sensor_config)
                  else _stable_repr(env.sensor_config))
    write_canonical(out, f"{stem}/env.json", {
        "config": dataclasses.asdict(env.config),
        "sensors": sensor_cfg,
        "backend": getattr(env, "backend", None),
        "delay_steps": env.delay_steps,
        "control_steps": env.control_steps,
        "sensor_steps": env.sensor_steps})

    command = RideControl(motor_torque_nm=10.0)
    transitions = []
    for _ in range(3):
        try:
            env.step(command)
        except Exception as exc:  # noqa: BLE001 — capture real oracle outcome
            transitions.append({"error": f"{type(exc).__name__}: {exc}"})
            break
    observations = []
    for obs in list(getattr(env, "observations", ())):
        observations.append(dataclasses.asdict(obs)
                            if dataclasses.is_dataclass(obs) else obs)
    rows = []
    for tr in list(getattr(env, "trace", ())):
        rows.append(dataclasses.asdict(tr)
                    if dataclasses.is_dataclass(tr) else tr)
    write_canonical(out, f"{stem}/transitions.json",
                    {"observations": observations, "transitions": rows})
    print(f"research transitions: {len(rows)} windows, "
          f"{len(observations)} observations")
    try:
        env.close()
    except Exception:
        pass


def update_manifest(out: FixtureWriter) -> None:
    """Refresh the oracle manifest's file records for the recaptured
    entries from the writer's actual records (sha256/bytes/dtype/shape/
    order) and record recapture provenance in produced_by/notes."""
    manifest_path = REPO / "migration" / "oracle-manifest.json"
    manifest = json.loads(manifest_path.read_text())
    entries = {"bootstrap/welded", "bootstrap/welded/steps_control",
               "bootstrap/welded/reset", "research/delayed_bootstrap"}
    provenance = (
        "FT-7 recapture under the live oracle after reviewed welded-plant "
        "physics changes (F1 ec9b031e + fix rounds, FT-3 96dad67c/a0e35355, "
        "b21c63ec); live-oracle parity green before recapture "
        "(T09 0/292 divergent, T10 0 divergent fields, 1e-9). "
        "Original produced_by: ")

    def owner(relpath: str) -> str | None:
        owners = [name for name in entries
                  if relpath == name or relpath.startswith(name + "/")
                  or relpath.startswith(name + ".")]
        return max(owners, key=len) if owners else None

    grouped: dict[str, list] = {name: [] for name in entries}
    for relpath, record in out.files.items():
        name = owner(relpath)
        if name is not None:
            grouped[name].append(record)
    for records in grouped.values():
        records.sort(key=lambda r: r["path"])

    # Recapture inputs live in THIS worktree (the original entries point
    # at the historical oracle checkout). Refresh path+sha256 honestly.
    input_map = {
        "bootstrap/welded": [
            REPO / "examples/research/viewer_physics_welded.toml",
            REPO / "examples/research/rough_uphill_savage.toml"],
        "research/delayed_bootstrap": [
            ROOT / "requests/research_delay.toml"],
    }
    updated = []
    for entry in manifest["fixtures"]:
        name = entry.get("name")
        if name not in entries:
            continue
        if not grouped[name]:
            raise RuntimeError(f"no captured files recorded for {name}")
        entry["files"] = grouped[name]
        if name in input_map:
            entry["inputs"] = [
                {"path": str(p),
                 "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                for p in input_map[name]]
        entry["produced_by"] = provenance + entry.get("produced_by", "")
        entry["notes"] = (entry.get("notes", "") +
                          " | recaptured at lane/d4 FT-7 (907687f1 base)")
        updated.append(name)
    if sorted(updated) != sorted(entries):
        missing = sorted(entries - set(updated))
        raise RuntimeError(f"manifest entries missing: {missing}")
    manifest_path.write_text(json.dumps(manifest, indent=2,
                                        sort_keys=True) + "\n")
    print("manifest records refreshed for", sorted(updated))


def main() -> int:
    out = FixtureWriter(ROOT)
    capture_welded(out)
    capture_research(out)
    update_manifest(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
