"""Replay a golden episode and diff it row-by-row — the equivalence harness.

Layer-2 of the correctness bar: same applied controls in, all recorder
channels out, plus identical first monitor failure (ADR 0001 §6).

Constraint: `EpisodeArtifact.initial` (qpos/qvel/act/ctrl/warmstart) is
captured but NOT applied here — replay reaches t=0 through the same
deterministic equilibrium reset as capture, which is bitwise-true
Python↔Python. The NATIVE replay path (P2+) must apply `ep.initial`
explicitly or goldens captured from non-fresh states diverge at step 0.
"""
import inspect

import numpy as np

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl

_CONTROL_FIELDS = frozenset(inspect.signature(RideControl).parameters)
_POSTURE_FIELDS = frozenset(inspect.signature(RiderPosture).parameters)

# Non-finite leaves roundtrip through flatten_row's portable JSON tokens.
_SENTINELS = {'nan': float('nan'), 'inf': float('inf'), '-inf': float('-inf')}
_MISSING = '<missing>'


def _control(payload: dict) -> RideControl:
    """Rebuild the applied control from an asdict payload: loaded records may
    carry keys RideControl does not own, and nested dataclasses (posture)
    arrive as plain dicts."""
    fields = {k: v for k, v in payload.items() if k in _CONTROL_FIELDS}
    posture = fields.get('posture')
    if isinstance(posture, dict):
        posture = {k: v for k, v in posture.items() if k in _POSTURE_FIELDS}
        if isinstance(posture.get('pelvis_offset_m'), (list, tuple)):
            posture['pelvis_offset_m'] = tuple(posture['pelvis_offset_m'])
        fields['posture'] = RiderPosture(**posture)
    return RideControl(**fields)


def replay_episode(env, ep) -> list[dict]:
    """Re-apply every recorded control on `env` and return all recorder rows."""
    runtime = env.sim.physical
    env.sim.reset()
    rows = []
    for ctl in ep.controls:
        runtime.step(control=_control(ctl))
        rows.extend(s.as_dict() for s in runtime.completed_samples)
    rows.extend(s.as_dict() for s in runtime.flush())
    return rows


def _leaves(v, path):
    """Scalar leaves of a row at dotted-indexed paths — 'a.b.0'. Descends
    dicts, lists/tuples and ndarrays alike so nested as_dict() rows and flat
    flatten_row() rows share one key space, and each NaN lands on its own
    leaf instead of hiding inside a list equality."""
    if isinstance(v, dict):
        for k, x in v.items():
            yield from _leaves(x, f'{path}.{k}' if path else k)
    elif isinstance(v, np.ndarray):
        for i, x in enumerate(v.flat):
            yield from _leaves(x, f'{path}.{i}')
    elif isinstance(v, (list, tuple)):
        for i, x in enumerate(v):
            yield from _leaves(x, f'{path}.{i}')
    else:
        if isinstance(v, np.generic):
            v = v.item()
        yield path, v


def _close(a, b, rtol, atol):
    if isinstance(a, str):
        a = _SENTINELS.get(a, a)
    if isinstance(b, str):
        b = _SENTINELS.get(b, b)
    try:
        return bool(np.isclose(float(a), float(b), rtol=rtol, atol=atol,
                               equal_nan=True))
    except (TypeError, ValueError):
        return a == b


def compare_rows(golden, candidate, *, rtol=1e-12, atol=1e-12) -> list[str]:
    diffs = []
    if len(golden) != len(candidate):
        diffs.append(f'row count: golden={len(golden)} candidate={len(candidate)}')
        return diffs
    for i, (g, c) in enumerate(zip(golden, candidate)):
        gleaves, cleaves = dict(_leaves(g, '')), dict(_leaves(c, ''))
        # Union: a channel only the candidate emits is a divergence too.
        names = list(gleaves) + [n for n in cleaves if n not in gleaves]
        for name in names:
            gv = gleaves.get(name, _MISSING)
            cv = cleaves.get(name, _MISSING)
            if not _close(gv, cv, rtol, atol):
                diffs.append(f'step={i} channel={name} '
                             f'golden={gv} candidate={cv}')
        if len(diffs) > 50:  # divergence floods fast past the horizon
            diffs.append('…truncated')
            break
    return diffs


def _same_failure(g, c, rtol, atol):
    if g is None or c is None:
        return g is None and c is None
    return _close(g[0], c[0], rtol, atol) and tuple(g[1]) == tuple(c[1])


def compare_episode(golden_dir, env, *, rtol=1e-12, atol=1e-12) -> list[str]:
    """Reload the saved artifact, replay it on `env`, diff every channel row
    and the first monitor failure. The entry point native tests call."""
    # Deferred: keep the mujoco-heavy capture module out of pure-row compares.
    from tools.golden_episode import load_episode
    ep = load_episode(golden_dir)
    rows = replay_episode(env, ep)
    diffs = compare_rows(ep.rows, rows, rtol=rtol, atol=atol)
    golden_ff = ep.first_failure
    candidate_ff = env.sim.physical.reference_monitor.first_failure
    if not _same_failure(golden_ff, candidate_ff, rtol, atol):
        diffs.append(f'first_failure: golden={golden_ff} '
                     f'candidate={candidate_ff}')
    return diffs
