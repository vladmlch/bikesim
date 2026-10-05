"""Measure the chaos-divergence horizon that bounds deterministic comparison.

Two runs of the same episode, one perturbed by eps in a single qvel dof:
the first step any recorded channel exceeds tol is the layer-2 horizon
(ADR 0001 §6). Below tol both runs are the 'same simulation'.
"""
import numpy as np

from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.initial_state import PhysicalInitialState

_MISSING = object()


def _run(env, steps, perturb_at=None, qvel_idx=0, eps=0.):
    """Replay `steps` fixed-control physics steps on a fresh env.

    The captured t=0 transfer makes the start deterministic without paying
    for a second equilibrium solve; completed_samples publishes one row per
    interval, so row index == step index. With eps=0 this is bitwise the
    same run (proven by test_golden_episode.test_capture_is_deterministic).
    """
    sim, rt = env.sim, env.sim.physical
    sim.physical_initial_state = PhysicalInitialState.capture(sim)
    sim.reset()
    rows = []
    for i in range(steps):
        if i == perturb_at:
            sim.data.qvel[qvel_idx] += eps
        rt.step(control=RideControl(human_torque_nm=35.))
        rows.extend(s.as_dict() for s in rt.completed_samples)
    rows.extend(s.as_dict() for s in rt.flush())
    return rows


def _leaves(v, path):
    """Scalar leaves at dotted/indexed paths ('qvel.0') — same walk as
    tools/episode_compare._leaves, inlined so the __main__ CLI runs as a
    plain script with no tools.* import on sys.path. Rows carry list/tuple
    channels (qpos, qvel, patches) and string/bool leaves, so a dict-only
    or numeric-only flatten would respectively miss the perturbed state
    channels or crash on them."""
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


def _differ(a, b, tol):
    """One leaf pair past the equivalence bound.

    Numeric leaves diverge on |a-b| > tol (equal_nan: NaN==NaN stays
    equivalent, NaN vs a number diverges). Non-numeric leaves — crash
    cause, backend names, violation strings — diverge on inequality,
    and a channel present in only one run is a divergence.
    """
    if a is _MISSING or b is _MISSING:
        return True
    try:
        return not np.isclose(float(a), float(b), rtol=0., atol=tol,
                              equal_nan=True)
    except (TypeError, ValueError):
        return a != b


def first_divergences(base, pert, tol):
    """{channel_name: first_step_over_tol} in first-seen order."""
    out = {}
    for i, (b, p) in enumerate(zip(base, pert)):
        bl, pl = dict(_leaves(b, '')), dict(_leaves(p, ''))
        # Union: a channel only one run emits is a divergence too.
        for name in list(bl) + [n for n in pl if n not in bl]:
            if name in out:
                continue
            if _differ(bl.get(name, _MISSING), pl.get(name, _MISSING), tol):
                out[name] = i
    return out


def measure(env_factory, steps, *, perturb, tol=1e-9, perturb_at=10):
    """Run the episode twice and return the full divergence detail:
    {'horizon': int|None, 'channels': {name: step}, 'row_counts': (b, p)}.
    env_factory must return a FRESH env per call — the two runs never share
    mjData. 'horizon' is None when no channel exceeds tol within `steps`.
    """
    base = _run(env_factory(), steps)
    pert = _run(env_factory(), steps, perturb_at,
                perturb['qvel_idx'], perturb['eps'])
    channels = first_divergences(base, pert, tol)
    if len(base) != len(pert):
        channels.setdefault('<row_count>', min(len(base), len(pert)))
    return {'horizon': min(channels.values()) if channels else None,
            'channels': channels,
            'row_counts': (len(base), len(pert))}


def measure_horizon(env_factory, steps, *, perturb, tol=1e-9,
                    perturb_at=10) -> int:
    """First step any recorded channel exceeds tol — `steps` if never."""
    horizon = measure(env_factory, steps, perturb=perturb, tol=tol,
                      perturb_at=perturb_at)['horizon']
    return steps if horizon is None else horizon


if __name__ == '__main__':
    import argparse, json, re
    from pathlib import Path
    from bike_sim.cli import research as research_cli

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--steps', type=int, default=4000)
    p.add_argument('--qvel-idx', type=int, default=0)
    p.add_argument('--eps', type=float, default=1e-14)
    p.add_argument('--tol', type=float, default=1e-9)
    p.add_argument('--perturb-at', type=int, default=10)
    p.add_argument('--out', default='output/divergence_horizon.json')
    a = p.parse_args()

    # Self-contained pinned-config rewrite (same pattern as
    # test_pinned_topology._pinned_config — tools must not import tests/).
    src = Path('examples/research/viewer_physics_welded.toml').read_text()
    for key, val in [('pedal_attachment', 'spindle'), ('saddle_attachment', 'pin'),
                     ('grip_attachment', 'connect')]:
        src = re.sub(rf'{key} = "\w+"', f'{key} = "{val}"', src)
    for key in ('joint_envelope_path', 'joint_strength_path'):
        src = re.sub(rf'{key} = "([^"]+)"',
                     lambda m: f'{key} = "{(Path("examples/research")/m.group(1)).resolve()}"', src)
    cfg = Path('output/divergence_physics.toml')
    cfg.parent.mkdir(parents=True, exist_ok=True)
    cfg.write_text(src)

    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(cfg),
            '--track-file', 'examples/research/rough_uphill_extreme.toml',
            '--duration', str(a.steps * .0005), '--dt', '.0005',
            # Non-strict monitor: horizon measurement must survive early
            # violations — strict mode dies at the first violated period.
            '--diagnostic-model-limits',
            '--out', 'output/divergence_env'])
        return research_cli.make_environment(args)

    detail = measure(make, a.steps,
                     perturb={'qvel_idx': a.qvel_idx, 'eps': a.eps},
                     tol=a.tol, perturb_at=a.perturb_at)
    h = a.steps if detail['horizon'] is None else detail['horizon']
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(
        {'horizon_step': h, 'steps': a.steps, 'eps': a.eps, 'tol': a.tol,
         'qvel_idx': a.qvel_idx, 'perturb_at': a.perturb_at,
         'row_counts': list(detail['row_counts']),
         'channel_first_divergence': dict(sorted(
             detail['channels'].items(), key=lambda kv: (kv[1], kv[0])))},
        indent=2) + '\n')
    print(json.dumps({'horizon_step': h, 'steps': a.steps}))
