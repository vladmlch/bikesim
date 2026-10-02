from dataclasses import asdict
from math import isfinite
from pathlib import Path
import argparse
import json
import shutil
import numpy as np
from bike_sim.validation.rider_replay import build_sim
from bike_sim.validation.plant_prefix import ValidPrefix
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_session import configuration_metadata


def checked_steps(duration_s: float, dt_s: float) -> int:
    if not isfinite(duration_s) or not isfinite(dt_s) or duration_s <= 0 or dt_s <= 0:
        raise ValueError('duration and timestep must be positive and finite')
    n = round(duration_s / dt_s)
    if n <= 0 or abs(n * dt_s - duration_s) > 1e-10:
        raise ValueError('duration must contain integer physical steps')
    return n


def run_audit(physics: str, track: str, duration_s: float, out: Path,
              *, continue_invalid: bool = False, decimate: int = 8) -> dict:
    if type(decimate) is not int or decimate < 1:
        raise ValueError('decimate must be a positive integer')
    out = Path(out)
    sim = build_sim(physics, track)
    steps = checked_steps(duration_s, float(sim.model.opt.timestep))
    out.mkdir(parents=True, exist_ok=False)
    metadata = plain(configuration_metadata(sim))
    shutil.copyfile(track, out / 'track.toml')
    np.save(out / 'terrain_vertices.npy', sim.physical.vertices, allow_pickle=False)
    saved_vertices = np.load(out / 'terrain_vertices.npy', allow_pickle=False)
    if saved_vertices.ndim != 2 or saved_vertices.shape[1] != 2:
        raise ValueError('terrain evidence must be an X-Z vertex array')
    if not np.isfinite(saved_vertices).all():
        raise ValueError('terrain evidence contains a nonfinite vertex')
    prefix = ValidPrefix()
    reason = 'duration_reached'
    with (out / 'samples.jsonl').open('w') as stream:
        for i in range(steps):
            sim.step(control=RideControl())
            sample = sim.physical.sample
            eligible = prefix.observe(sample)
            c = sample.channels
            row = {'time_s': sample.time_s, 'end_time_s': sample.end_time_s,
                   'x_m': float(sample.qpos[sim.root_x_qposadr]),
                   'eligible': eligible,
                   'tires': plain(c['tires']), 'drive': plain(c['drive']),
                   'energy': plain(c['energy']), 'model_status': plain(c['model_status'])}
            first_invalid = prefix.first_bad is not None and prefix.first_bad['time_s'] == sample.time_s
            if i % decimate == 0 or first_invalid:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
            if not eligible and not continue_invalid:
                reason = prefix.first_bad['reasons'][0]
                break
            if sim.crash is not None:
                reason = 'crash:' + sim.crash.cause
                break
            if sim.position_m >= sim.track.length_m:
                reason = 'end_of_track'
                break
    prefix.first_bad = plain(prefix.first_bad)
    report = {'metadata': metadata, 'mode': 'audited', 'reason': reason,
              'time_s': sim.time_s, 'x_m': sim.position_m,
              'valid_prefix': plain(asdict(prefix)),
              'model_status': sim.physical.model_status.as_dict()}
    (out / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--physics-config', required=True)
    p.add_argument('--track', required=True)
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--continue-invalid', action='store_true')
    p.add_argument('--decimate', type=int, default=8)
    a = p.parse_args()
    report = run_audit(a.physics_config, a.track, a.duration, a.out,
                       continue_invalid=a.continue_invalid, decimate=a.decimate)
    return 2 if report['valid_prefix']['first_bad'] is not None else 0


if __name__ == '__main__':
    raise SystemExit(main())
