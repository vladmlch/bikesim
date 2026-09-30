"""Resolved physical run metadata and headless CLI execution."""
import csv
from dataclasses import asdict,replace
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import re
import subprocess
import time
import numpy as np
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.validation.environment import source_fingerprint, environment_contract


class PhysicalPreviewCsv:
    """Flushes the live physical preview state at a fixed simulated-time interval, as CSV."""

    def __init__(self, path: Path, interval_s: float = 0.01):
        if interval_s <= 0.0:
            raise ValueError(f"preview csv interval must be positive, got {interval_s}")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.interval_s = float(interval_s)
        self._stream = self.path.open("w", newline="", encoding="utf-8")
        self._next_time_s = 0.0
        self._generation = None
        self._writer = None

    def due(self, time_s: float, generation: int) -> bool:
        """Whether `write` would emit a row now -- lets callers skip building it."""
        return self._generation != generation or float(time_s) + 1e-12 >= self._next_time_s

    def write(self, time_s: float, generation: int, row: dict) -> None:
        """Write one state row when the next simulated-time boundary is reached."""
        event = ""
        if self._generation != generation:
            if self._generation is not None:
                event = "reset"
            self._generation = generation
            self._next_time_s = float(time_s)
        if float(time_s) + 1e-12 < self._next_time_s:
            return
        record = {"generation": generation, "event": event, **row}
        if self._writer is None:
            self._writer = csv.DictWriter(
                self._stream, fieldnames=list(record), restval="", extrasaction="ignore")
            self._writer.writeheader()
        self._writer.writerow(record)
        self._stream.flush()
        while self._next_time_s <= float(time_s) + 1e-12:
            self._next_time_s += self.interval_s

    def write_marker(self, marker: str) -> None:
        """Write an immediate non-state event row, such as a run termination reason."""
        record = {
            "generation": "" if self._generation is None else self._generation,
            "event": marker,
        }
        if self._writer is None:
            self._writer = csv.DictWriter(
                self._stream, fieldnames=list(record), restval="", extrasaction="ignore")
            self._writer.writeheader()
        self._writer.writerow(record)
        self._stream.flush()

    def close(self) -> None:
        """Flush and close the preview log."""
        self._stream.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def canonical_json(payload):
    return json.dumps(plain(payload),sort_keys=True,separators=(',',':'),allow_nan=False)


def configuration_metadata(sim,seed=None):
    vertices=sim.physical.vertices
    terrain_hash=hashlib.sha256(np.asarray(vertices,dtype='<f8').tobytes()).hexdigest()
    resolved={'physics':asdict(sim.physics_config),'geometry_and_suspension':asdict(sim.specs),
              'mass_budget':asdict(sim.mass_specs),'rider':asdict(sim.rider),
              'target_speed_kmh':sim.cruise.target_speed_kmh if sim.physics_config.drive_mode=='ideal_speed_control' else None,
              'terrain_sha256':terrain_hash,'seed':seed}
    # Friction zones change the plant even when the heightfield is unchanged.
    from bike_sim.terrain.trackfile import track_to_dict
    resolved['track'] = track_to_dict(sim.track)
    envelope=sim.physics_config.articulated.joint_envelope_path
    resolved['joint_envelope_status']='unspecified' if envelope is None else 'declared_unvalidated'
    if envelope is not None:
        resolved['joint_envelope_sha256']=hashlib.sha256(Path(envelope).read_bytes()).hexdigest()
    # Effective overrides are recorded, not just geometry defaults.
    resolved['active_suspension']={
        'fork_pressure_psi':sim.controller.air_spring.gauge_pressure_psi,
        'fork_tokens':sim.controller.air_spring.num_tokens,
        'coil':asdict(sim.applier.coil_shock.specs),
        'fork_damper':sim.controller.suspension_system.fork_damper.get_effective_coefficients(),
        'shock_damper':sim.controller.suspension_system.shock_damper.get_effective_coefficients(),
    }
    resolved['compiled_contact_solver'] = {
        side: {
            'solref': sim.model.geom_solref[geom].tolist(),
            'solimp': sim.model.geom_solimp[geom].tolist(),
            'friction': sim.model.geom_friction[geom].tolist(),
            'condim': int(sim.model.geom_condim[geom]),
            'native_enabled': bool(sim.model.geom_contype[geom] or sim.model.geom_conaffinity[geom]),
            'units': 'MuJoCo solver parameters; not tire material stiffness or damping',
        }
        for side,geom in (('front',sim.contact_query.front_id),('rear',sim.contact_query.rear_id))
    }
    resolved['compiled_equality_solref'] = sim.model.eq_solref.tolist()
    resolved['compiled_equality_solimp'] = sim.model.eq_solimp.tolist()
    source_root=Path(__file__).resolve().parents[2]
    source_hash=source_fingerprint(source_root)
    try:
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source_root,stderr=subprocess.DEVNULL,text=True).strip()
        dirty=bool(subprocess.check_output(['git','status','--porcelain','--','.'],cwd=source_root,stderr=subprocess.DEVNULL,text=True).strip())
    except (OSError,subprocess.CalledProcessError):
        commit=None
        dirty=None
    config_hash=hashlib.sha256(canonical_json(resolved).encode()).hexdigest()
    return {'schema_version':2,'physics_revision':sim.physics_revision,
        'model_commit':commit,'model_source_dirty':dirty,'model_source_sha256':source_hash,
        'versions':{'python':platform.python_version(),**{n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy')}},
        'configuration_sha256':config_hash,'terrain_sha256':terrain_hash,
        'resolved_config':resolved,'seed':seed,'timestep_s':float(sim.model.opt.timestep),
        'controller_interval_s':float(sim.model.opt.timestep),'tire_backend':sim.physics_config.tires.backend,
        'drive_mode':sim.physics_config.drive_mode,'rider_model':sim.rider.variant,
        'external_speed_controller':sim.physics_config.drive_mode=='ideal_speed_control',
        'calibration_status':'parameterized_unvalidated',
        'environment_contract':environment_contract(),
        'component_provenance':{key:'synthetic' for key in sim.mass_specs.component_masses},
        'limitations':['planar X-Z dynamics','synthetic material and drive parameters',
                       ('experimental distributed tire; mesh/time/station gates required' if sim.physics_config.tires.backend=='distributed_2d_reference'
                        else 'single equivalent tire support; multi-support peaks not calibrated'),
                       'native contact work is a solver diagnostic, not material tire hysteresis']}


def physical_run_dir_name(track_name,metadata):
    name=re.sub(r'[^A-Za-z0-9_.-]+','_',track_name).strip('._')[:80] or 'track'
    return f"{name}_{metadata['drive_mode']}_{metadata['configuration_sha256'][:16]}"


def physical_summary(sim,metadata,reason):
    r=sim.physical
    return plain(dict(metadata,outcome={'reason':reason,'time_s':sim.time_s,'position_m':sim.position_m,
                                       'steps':sim.steps,'crashed':sim.crash is not None},
        equilibrium=sim.equilibrium,energy=r.energy,model_status=r.model_status.as_dict(),battery_energy_j=r.drive.battery.energy_j,
        component_work_j=r.history.work_j,airtime_threshold_s=r.history.airtime_s,
        duration_s=r.history.duration_s))


def build_physical_simulation(track,args,rider):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.sim.ride_sim import RideSimulation
    specs=BikeSpecs()
    cfg=args.resolved_physics
    if args.sag is not None:
        from bike_sim.sim.ride.sag_fit import build_equilibrium_evaluator,fit_sag
        pct=float(args.sag)
        if not math.isfinite(pct) or not 0<pct<100:
            raise ValueError('sag percentage must lie strictly between 0 and 100')
        # No running initial speed or human torque is used inside the static fit.
        evaluator=build_equilibrium_evaluator(specs=specs,track=track,rider=rider,physics_config=cfg)
        values=fit_sag(evaluator,(specs.fork_travel*pct/100,specs.rear_wheel_travel*pct/100),
                       (specs.fork_initial_psi,specs.shock_stiffness),((10.,1000.),(400.,250000.)))
        specs=replace(specs,fork_initial_psi=float(values[0]),shock_stiffness=float(values[1]))
    sim=RideSimulation(track=track,specs=specs,rider=rider,
        target_speed_kmh=args.speed if args.speed is not None else 25.,physics_config=cfg)
    if args.sag is not None:
        sim.equilibrium['requested_sag_pct']=float(args.sag)
        errors=[abs(sim.equilibrium['fork_travel_mm']-specs.fork_travel*args.sag/100),
                abs(sim.equilibrium['rear_travel_mm']-specs.rear_wheel_travel*args.sag/100)]
        if max(errors)>.5:
            raise RuntimeError(f'final compiled sag error exceeds 0.5 mm: {errors}')
    return sim


def run_physical_headless(track,args,seed,rider):
    sim=build_physical_simulation(track,args,rider)
    metadata=configuration_metadata(sim,seed)
    out=Path(args.out)/physical_run_dir_name(track.name,metadata)
    out.mkdir(parents=True,exist_ok=True)
    recorder=PhysicalRecorder(sim,args.decimate)
    if args.duration is not None:
        max_steps=math.ceil(args.duration/float(sim.model.opt.timestep))
    else:
        max_steps=sim.default_limits().max_steps
    deadline=time.monotonic()+sim.default_limits().max_wall_clock_s
    reason='duration_reached' if args.duration is not None else 'step_cap'
    try:
        for _ in range(max_steps):
            if sim.position_m>=track.length_m:
                reason='end_of_track';break
            if sim.crash is not None:
                reason='crash';break
            if time.monotonic()>deadline:
                reason='wall_clock_cap';break
            sim.step();recorder.record(sim)
    except (ValueError,RuntimeError,ArithmeticError) as exc:
        reason='simulation_error'
        metadata['failure']=str(exc)
    recorder.write_csv(out/'telemetry.csv')
    recorder.write_jsonl(out/'intervals.jsonl')
    np.save(out/'terrain_vertices.npy',sim.physical.vertices,allow_pickle=False)
    summary=physical_summary(sim,metadata,reason)
    summary['model_status']=sim.physical.model_status.as_dict()
    (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True,allow_nan=False)+'\n')
    if not args.no_plots and recorder.rows:
        from bike_sim.viz.ride_plots import plot_physical_ride
        plot_physical_ride(recorder.columns(),out)
    print(f"[bike-ride] {reason}: {sim.time_s:.6f} s, {sim.position_m:.3f} m -> {out}")
    if reason == 'simulation_error':
        return 1
    if not sim.physical.model_status.as_dict()['model_valid']:
        return 2
    return 0 if reason in ('duration_reached','end_of_track') else 1
