"""Display-only playback of recorded research episodes.

Rebuilds the checked plant through replay.rebuild_environment and exposes the
recorded qpos stream as interpolation keyframes; no physics is re-integrated.
"""
import json
from pathlib import Path

import mujoco
import numpy as np

from bike_sim.sim.research.replay import rebuild_environment

INTERVALS_FILE = 'intervals.jsonl'


class Playback:
    """Checked rebuilt simulation plus its decimated qpos keyframes."""

    def __init__(self, sim, times_s: np.ndarray, qpos: np.ndarray,
                 qvel: np.ndarray, duration_s: float):
        self.sim = sim
        self.times_s = times_s
        self.qpos = qpos
        self.qvel = qvel
        self.duration_s = duration_s
        self._quat_blocks = _quaternion_blocks(sim.model)

    def pose_at(self, t_s: float) -> np.ndarray:
        """Interpolated qpos at absolute sim time; clamped to [0, duration]."""
        t = float(t_s)
        if not np.isfinite(t):
            raise ValueError('pose_at requires a finite time')
        t = min(max(t, 0.), self.duration_s)
        index = int(np.searchsorted(self.times_s, t, side='right'))-1
        if index < 0:
            return self.qpos[0].copy()
        if index >= self.times_s.shape[0]-1:
            return self.qpos[-1].copy()
        weight = (t-self.times_s[index])/(self.times_s[index+1]-self.times_s[index])
        pose = self.qpos[index]+(self.qpos[index+1]-self.qpos[index])*weight
        for block in self._quat_blocks:
            pose[block] /= np.linalg.norm(pose[block])
        return pose


def _quaternion_blocks(model):
    """qpos slices that must stay unit quaternions (free and ball joints)."""
    blocks = []
    for joint in range(model.njnt):
        adr = int(model.jnt_qposadr[joint])
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            blocks.append(slice(adr+3, adr+7))  # free: xyz position, then wxyz
        elif model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_BALL:
            blocks.append(slice(adr, adr+4))
    return tuple(blocks)


def _interval_rows(path):
    file = path/INTERVALS_FILE
    if not file.is_file():
        raise ValueError(f'{file} is missing; re-record the episode so the recorder keeps intervals')
    with file.open(encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    if not rows:
        raise ValueError(f'{file} is empty; re-record the episode so the recorder keeps intervals')
    return rows


def load_playback(directory) -> Playback:
    """Rebuild the recorded plant and load its stored qpos stream as keyframes."""
    path = Path(directory)
    rows = _interval_rows(path)
    env = rebuild_environment(path)
    model = env.sim.model
    n_frames = len(rows)
    if n_frames < 2:
        raise ValueError(f'{path/INTERVALS_FILE}: {n_frames} recorded frame(s); playback needs at least two')
    times_s, ends_s = np.empty(n_frames), np.empty(n_frames)
    qpos = np.empty((n_frames, model.nq))
    qvel = np.empty((n_frames, model.nv))
    for i, row in enumerate(rows):
        label = f'frame {i} (interval_id={row.get("interval_id", "?")})'
        try:
            position = np.asarray(row['qpos'], dtype=float)
            velocity = np.asarray(row['qvel'], dtype=float)
            start_s, end_s = float(row['time_s']), float(row['interval_end_s'])
        except KeyError as exc:
            raise ValueError(f'{path/INTERVALS_FILE}: {label} is missing {exc}') from None
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{path/INTERVALS_FILE}: {label} has non-numeric fields') from exc
        if position.shape != (model.nq,):
            raise ValueError(f'{path/INTERVALS_FILE}: {label} qpos length {position.size} != model.nq {model.nq}')
        if velocity.shape != (model.nv,):
            raise ValueError(f'{path/INTERVALS_FILE}: {label} qvel length {velocity.size} != model.nv {model.nv}')
        if not (np.isfinite(position).all() and np.isfinite(velocity).all()):
            raise ValueError(f'{path/INTERVALS_FILE}: {label} has nonfinite state')
        if not (np.isfinite(start_s) and np.isfinite(end_s)):
            raise ValueError(f'{path/INTERVALS_FILE}: {label} has nonfinite time')
        times_s[i], ends_s[i] = start_s, end_s
        qpos[i], qvel[i] = position, velocity
    inverted = np.nonzero(ends_s <= times_s)[0]
    if inverted.size:
        i = int(inverted[0])
        raise ValueError(f'{path/INTERVALS_FILE}: frame {i} (interval_id={rows[i].get("interval_id", "?")}) '
                         'interval_end_s does not exceed time_s')
    regressed = np.nonzero(np.diff(times_s) <= 0.)[0]
    if regressed.size:
        i = int(regressed[0])+1
        raise ValueError(f'{path/INTERVALS_FILE}: frame {i} (interval_id={rows[i].get("interval_id", "?")}) '
                         'time_s is not strictly increasing')
    return Playback(env.sim, times_s, qpos, qvel, float(ends_s[-1]))
