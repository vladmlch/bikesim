"""Freeze a deterministic episode as the correctness oracle artifact.

Captures per-step applied control, raw mjData state, and every recorder
channel row, plus the first monitor failure — the layer-2 contract a native
port must reproduce (ADR 0001 §6).

v2 additionally captures, per physics step, the mjData state at the entry of
``runtime.apply_forces`` (qpos/qvel/act/warmstart/time), the ``d.ctrl`` buffer
it leaves behind, every named force component in ``acc.add`` insertion order,
and the flattened tire brush states the call consumes — the inputs/outputs
per-call writer-equivalence checks replay.
"""
import json
from dataclasses import asdict, dataclass, field
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
    # v2 (P2): per-physics-step captures. Defaults keep v1 artifacts loadable.
    state_qpos: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    state_qvel: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    state_act: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    state_warmstart: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    state_time: np.ndarray = field(default_factory=lambda: np.empty(0))
    ctrl_written: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    force_names: list = field(default_factory=list)   # acc.add insertion order
    forces: np.ndarray = field(default_factory=lambda: np.empty((0, 0, 0)))
    tire_state: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    tire_state_names: list = field(default_factory=list)


def _tire_state_row(tire) -> dict:
    """Flatten both wheels' _BrushState fields into one leaf-named row."""
    if tire is None:
        return {}
    return flatten_row({side: asdict(state) for side, state in tire.states.items()})


def _tire_snapshot_row(tire) -> dict:
    """JSON-able per-side digest of the contact snapshots for one step."""
    if tire is None:
        return {}
    row = {}
    for side, snap in tire.snapshots.items():
        row[side] = {
            'time_s': float(snap.time_s),
            'interval_id': int(snap.interval_id),
            'backend': snap.backend,
            'geometric_contact': bool(snap.geometric_contact),
            'effective_radius_m': float(snap.effective_radius_m),
            'patches': [{'normal_load_n': float(p.normal_load_n),
                         'tangent_force_n': float(p.tangent_force_n),
                         'slip_mps': float(p.slip_mps),
                         'working_surface': bool(p.working_surface)}
                        for p in snap.patches],
        }
    return row


def capture_episode(env, steps: int, control: RideControl) -> EpisodeArtifact:
    sim = env.sim
    runtime = sim.physical
    sim.physical_initial_state = PhysicalInitialState.capture(sim)
    sim.reset()
    d = sim.data
    initial = {k: getattr(d, k).copy() for k in
               ('qpos', 'qvel', 'act', 'ctrl', 'qacc_warmstart')}
    initial['time'] = np.array([d.time])
    # v2 instrumentation — installed only for the step loop. Init/reset paths
    # issue their own apply_forces calls (advance=False probes in reset/restore,
    # and advance=True settling iterations inside solve_physical_equilibrium);
    # none of them are episode physics steps. Within the loop each advance=True
    # call opens a record: entry state snapshot, acc.add components in call
    # order, exit ctrl snapshot. advance=False calls never create records.
    records: list[dict] = []
    stray_adds: dict[str, np.ndarray] = {}   # acc.add with no open step record
    acc = sim.force_accumulator
    orig_add, orig_apply = acc.add, runtime.apply_forces

    def spied_add(name, qfrc):
        orig_add(name, qfrc)
        open_record = records and records[-1]['open']
        target = records[-1]['components'] if open_record else stray_adds
        target[name] = np.asarray(qfrc, float).copy()

    def spied_apply(**kw):
        if not kw.get('advance', True):
            return orig_apply(**kw)
        record = {'qpos': d.qpos.copy(), 'qvel': d.qvel.copy(),
                  'act': d.act.copy(), 'warmstart': d.qacc_warmstart.copy(),
                  'time': float(d.time), 'components': {}, 'ctrl': None,
                  # The brush states compute_qfrc consumes on THIS call —
                  # captured at entry so tire_state[k] restores the state
                  # that produced forces[k], mirroring state_qpos[k].
                  'tire_state': _tire_state_row(runtime.tire),
                  'open': True}
        records.append(record)
        try:
            return orig_apply(**kw)
        finally:
            record['ctrl'] = d.ctrl.copy()
            record['open'] = False

    acc.add = spied_add
    runtime.apply_forces = spied_apply
    controls, rows, tire_snapshots = [], [], []
    tire = runtime.tire
    try:
        for _ in range(steps):
            runtime.step(control=control)
            controls.append(asdict(runtime.applied_control))
            rows.extend(s.as_dict() for s in runtime.completed_samples)
            tire_snapshots.append(_tire_snapshot_row(tire))
    finally:
        # Removing the instance attributes restores the class-bound methods.
        del acc.add
        del runtime.apply_forces
    rows.extend(s.as_dict() for s in runtime.flush())

    nq, nv, na, nu = sim.model.nq, sim.model.nv, sim.model.na, sim.model.nu
    k = len(records)
    def stack(key, width):
        return (np.array([r[key] for r in records]) if records
                else np.empty((0, width)))
    # Component names are the union in first-appearance order: some writers
    # (e.g. shock_hbo, external) do not run every step, and absent means the
    # writer contributed nothing — a zero column row, not a schema hole.
    force_names = list(dict.fromkeys(
        n for r in records for n in r['components']))
    force_index = {n: i for i, n in enumerate(force_names)}
    forces = np.zeros((k, len(force_names), nv))
    for i, r in enumerate(records):
        for name, qfrc in r['components'].items():
            forces[i, force_index[name]] = qfrc
    tire_state_names = list(dict.fromkeys(
        n for r in records for n in r['tire_state']))
    tire_state = np.full((k, len(tire_state_names)), np.nan)
    for i, r in enumerate(records):
        for j, n in enumerate(tire_state_names):
            v = r['tire_state'].get(n)
            if isinstance(v, (int, float)):
                tire_state[i, j] = float(v)   # None/sentinels stay NaN
    manifest = {
        'source_fingerprint': source_fingerprint(
            Path(__file__).resolve().parents[1] / 'src/bike_sim'),
        'steps': len(rows),
        'physics_steps': k,
        'dt_s': float(sim.model.opt.timestep),
        'tire_snapshots': tire_snapshots,
    }
    if stray_adds:
        # acc.add ran with no open step record — an instrumentation anomaly,
        # surfaced in the manifest rather than silently folded into a step.
        manifest['stray_force_adds'] = sorted(stray_adds)
    # 'full'-period samples carry extra channels (rider, attachment_samples,
    # component_work_j, ...); the channel contract is the union across rows,
    # and save() records which names each row lacks.
    channel_names = sorted({n for r in rows for n in flatten_row(r)})
    return EpisodeArtifact(channel_names, initial, controls,
                           rows, runtime.reference_monitor.first_failure,
                           manifest,
                           state_qpos=stack('qpos', nq),
                           state_qvel=stack('qvel', nv),
                           state_act=stack('act', na),
                           state_warmstart=stack('warmstart', nv),
                           state_time=(np.array([r['time'] for r in records])
                                       if records else np.empty(0)),
                           ctrl_written=stack('ctrl', nu),
                           force_names=force_names, forces=forces,
                           tire_state=tire_state,
                           tire_state_names=tire_state_names)


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
             control_names=np.array(control_names, dtype=str),
             state_qpos=ep.state_qpos, state_qvel=ep.state_qvel,
             state_act=ep.state_act, state_warmstart=ep.state_warmstart,
             state_time=ep.state_time, ctrl_written=ep.ctrl_written,
             force_names=np.array(ep.force_names, dtype=str),
             forces=ep.forces,
             tire_state=ep.tire_state,
             tire_state_names=np.array(ep.tire_state_names, dtype=str))
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
    initial = {k[5:]: z[k] for k in z.files if k.startswith('init_')}
    # v2 keys are absent in v1 artifacts: fall back to correctly-shaped empties
    # sized from the initial-state arrays (all present since v1).
    def dim(key):
        return int(initial[key].shape[-1]) if key in initial else 0
    nq, nv = dim('qpos'), dim('qvel')
    def opt(key, shape):
        return z[key] if key in z.files else np.empty(shape)
    return EpisodeArtifact(names, initial, controls, rows, ff, man,
        state_qpos=opt('state_qpos', (0, nq)),
        state_qvel=opt('state_qvel', (0, nv)),
        state_act=opt('state_act', (0, dim('act'))),
        state_warmstart=opt('state_warmstart', (0, nv)),
        state_time=opt('state_time', (0,)),
        ctrl_written=opt('ctrl_written', (0, dim('ctrl'))),
        force_names=([str(n) for n in z['force_names']]
                     if 'force_names' in z.files else []),
        forces=opt('forces', (0, 0, nv)),
        tire_state=opt('tire_state', (0, 0)),
        tire_state_names=([str(n) for n in z['tire_state_names']]
                          if 'tire_state_names' in z.files else []))
