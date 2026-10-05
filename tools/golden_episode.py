"""Freeze a deterministic episode as the correctness oracle artifact.

Captures per-step applied control, raw mjData state, and every recorder
channel row, plus the first monitor failure — the layer-2 contract a native
port must reproduce (ADR 0001 §6).
"""
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.initial_state import PhysicalInitialState
from bike_sim.validation.environment import source_fingerprint


@dataclass
class EpisodeArtifact:
    channel_names: list[str]
    initial: dict          # mjData arrays: qpos, qvel, act, time, ctrl, warmstart
    controls: list[dict]   # per-step asdict(applied_control)
    rows: list[dict]       # per-step sample.as_dict()
    first_failure: object  # (time_s, violations tuple) or None
    manifest: dict


def capture_episode(env, steps: int, control: RideControl) -> EpisodeArtifact:
    sim = env.sim
    runtime = sim.physical
    sim.physical_initial_state = PhysicalInitialState.capture(sim)
    sim.reset()
    d = sim.data
    initial = {k: getattr(d, k).copy() for k in
               ('qpos', 'qvel', 'act', 'ctrl', 'qacc_warmstart')}
    initial['time'] = np.array([d.time])
    controls, rows = [], []
    for _ in range(steps):
        runtime.step(control=control)
        controls.append(asdict(runtime.applied_control))
        rows.extend(s.as_dict() for s in runtime.completed_samples)
    rows.extend(s.as_dict() for s in runtime.flush())
    manifest = {
        'source_fingerprint': source_fingerprint(
            Path(__file__).resolve().parents[1] / 'src/bike_sim'),
        'steps': len(rows),
        'dt_s': float(sim.model.opt.timestep),
    }
    # 'full'-period samples carry extra channels (rider, attachment_samples,
    # component_work_j, ...); the channel contract is the union across rows,
    # and save() records which names each row lacks.
    channel_names = sorted({n for r in rows for n in flatten_row(r)})
    return EpisodeArtifact(channel_names, initial, controls,
                           rows, runtime.reference_monitor.first_failure,
                           manifest)


def _nonfinite(x: float) -> str:
    """JSON/portable sentinel for non-finite leaves: NaN never equals itself
    after a reload, so bitwise row equality needs a stable token instead."""
    return 'nan' if np.isnan(x) else ('inf' if x > 0 else '-inf')


def flatten_row(row: dict) -> dict:
    """Leaf-flatten a sample.as_dict() row: {a:{b:1}} -> {'a.b': 1.0}.

    Scalar leaves become floats; arrays flatten elementwise ('a.0','a.1');
    non-numeric leaves are returned as-is for the manifest side channel.
    Non-finite numeric leaves become 'nan'/'inf'/'-inf' sentinels so a saved
    artifact still roundtrips bitwise.
    """
    out = {}
    def walk(v, p):
        if isinstance(v, dict):
            for k, x in v.items(): walk(x, f'{p}.{k}' if p else k)
        elif isinstance(v, np.ndarray):
            for i, x in enumerate(v.flat):
                x = float(x)
                out[f'{p}.{i}'] = x if np.isfinite(x) else _nonfinite(x)
        else:
            if isinstance(v, np.generic):
                v = v.item()
            if isinstance(v, float) and not np.isfinite(v):
                v = _nonfinite(v)
            out[p] = v
    walk(row, '')
    return out


def save(ep: EpisodeArtifact, out_dir: Path, model):
    out_dir.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(out_dir/'model.mjb'))
    flat_rows = [flatten_row(r) for r in ep.rows]
    names = ep.channel_names
    arr = np.full((len(flat_rows), len(names)), np.nan)
    nonnum_rows, absent_rows = [], []
    for i, fr in enumerate(flat_rows):
        nonnum, absent = {}, []
        for j, n in enumerate(names):
            if n not in fr:
                absent.append(n)        # schema hole; key stays absent on load
                continue
            v = fr[n]
            if isinstance(v, (int, float)):
                arr[i, j] = v
            else:
                nonnum[n] = v           # numeric hole; real value in manifest
        nonnum_rows.append(nonnum)
        absent_rows.append(absent)
    # applied_control holds Nones, bools and nested dicts (posture): keep a
    # numeric matrix in the npz for vector reads (None/non-scalar -> NaN) and
    # the verbatim asdict payloads in the manifest for exact reconstruction —
    # object arrays cannot survive np.load(allow_pickle=False).
    control_names = sorted({k for c in ep.controls for k in c})
    controls = np.full((len(ep.controls), len(control_names)), np.nan)
    for i, c in enumerate(ep.controls):
        for j, n in enumerate(control_names):
            v = c.get(n)
            if isinstance(v, (int, float)) and np.isfinite(v):
                controls[i, j] = float(v)
    np.savez(out_dir/'episode.npz', channels=arr,
             channel_names=np.array(names, dtype=str),
             **{f'init_{k}': v for k, v in ep.initial.items()},
             controls=controls,
             control_names=np.array(control_names, dtype=str))
    (out_dir/'manifest.json').write_text(json.dumps({
        **ep.manifest, 'first_failure': ep.first_failure,
        'non_numeric': nonnum_rows, 'absent': absent_rows,
        'controls': ep.controls}, indent=2, default=str))


def load_episode(d: Path) -> EpisodeArtifact:
    d = Path(d)
    z = np.load(d/'episode.npz', allow_pickle=False)
    man = json.loads((d/'manifest.json').read_text())
    names = [str(n) for n in z['channel_names']]
    nonnum = man.get('non_numeric', [])
    absent = man.get('absent', [])
    rows = []
    for i, row in enumerate(z['channels']):
        skip = set(absent[i]) if i < len(absent) else ()
        flat = {n: float(v) for n, v in zip(names, row) if n not in skip}
        if i < len(nonnum):
            flat.update(nonnum[i])        # restore manifest-side values
        rows.append(flat)
    controls = man.get('controls')      # verbatim payloads, exact roundtrip
    if controls is None:                # fall back to the numeric npz matrix
        cnames = [str(n) for n in z['control_names']]
        controls = [{n: (None if np.isnan(v) else float(v))
                     for n, v in zip(cnames, c)} for c in z['controls']]
    ff = man.get('first_failure')
    if isinstance(ff, (list, tuple)):   # JSON arrays -> (time_s, violations)
        ff = (ff[0], tuple(ff[1]))
    return EpisodeArtifact(names,
        {k[5:]: z[k] for k in z.files if k.startswith('init_')},
        controls, rows, ff, man)
