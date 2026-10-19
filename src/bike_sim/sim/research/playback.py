"""Display-only playback of recorded research episodes.

Recorded ``qpos``/``qvel`` keyframes render through the same authored
XML/config reconstruction route the checked recipe documents — but playback
never performs the checked-replay gates: no runtime-version or source
fingerprint comparison, no ``require_execution_compatible`` call, no backend
artifact load and no physical stepping, not even the starting-equilibrium
solve. A ``Playback`` is recorded data on a compiled visual model; it can
never report that physical replay was verified.
"""
import dataclasses
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np

INTERVALS_FILE = 'intervals.jsonl'


class _DisplayModel:
    """Compiled visual model for recorded frames; owns no dynamics.

    ``RideSimulation`` couples model compilation to the runtime owner (force
    path, equilibrium solve, async worker). Playback needs only the compiled
    ``mjModel``/``mjData`` the recorded keyframes index, so this holder keeps
    the model-facing surface the viewers read and nothing else.
    """

    __slots__ = ('model', 'data', 'track', 'field', 'start_x_m',
                 '_root_x_qposadr', '_frame_body_id')

    def __init__(self, model, data, track, field, start_x_m):
        self.model = model
        self.data = data
        self.track = track
        self.field = field
        self.start_x_m = start_x_m
        self._root_x_qposadr = int(model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root_x')])
        self._frame_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, 'frame')

    @property
    def position_m(self):
        """Chassis position of the frame currently staged in ``data.qpos``."""
        return float(self.data.qpos[self._root_x_qposadr])


def _display_model(path, summary, recipe):
    """Compile the recorded recipe's model without simulating anything.

    This is *not* ``replay._rebuild``: the runtime-identity, execution-
    compatibility, backend-artifact and recorded-state comparisons belong to
    checked replay and are never invoked here. External recipe paths stay
    untrusted — bundled rider assets are staged under ``path`` by the caller.
    """
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.kinematics.solver import HorstLinkageSolver
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.physics.mass import BikeMassSpecs
    from bike_sim.physics.resolution import resolve_physics_config
    from bike_sim.physics.rider import (
        DEFAULT_RIDER_VARIANT, RiderSpecs, resolve_rider)
    from bike_sim.physics.tyre import TyreConfig
    from bike_sim.sim.research.replay import ENVELOPE_FILE, STRENGTH_FILE
    from bike_sim.terrain import assert_track_fits, build_field_data
    from bike_sim.terrain.heightfield import HeightFieldSpec
    from bike_sim.terrain.trackfile import load_track
    resolved = summary.get('resolved_config')
    if not isinstance(resolved, dict) or not isinstance(resolved.get('physics'), dict):
        raise ValueError('recording lacks the resolved configuration required '
                         'to rebuild the display model')
    from copy import deepcopy
    data = deepcopy(resolved)
    articulated = data['physics'].get('articulated')
    if not isinstance(articulated, dict):
        raise ValueError('recording articulated recipe must be an object')
    for key, filename in (('joint_envelope', ENVELOPE_FILE),
                          ('joint_strength', STRENGTH_FILE)):
        declared = articulated.get(key + '_path')
        bundled = recipe.get(key + '_file')
        if declared is not None and bundled is None:
            raise ValueError(f'recording lacks a bundled {key}; '
                             'external recipe paths are not trusted')
        articulated[key + '_path'] = (
            None if bundled is None else str((path / filename).resolve()))
    cfg = resolve_physics_config(data['physics'])
    track = load_track(path / 'track.toml')
    field = HeightFieldSpec(**recipe['heightfield'])
    assert_track_fits(track, field)
    specs = BikeSpecs(**data['geometry_and_suspension'])
    # The rider/leg resolution and model-compilation arguments mirror
    # RideSimulation's own defaults for the same recipe — crank-side legs stay
    # 'rigid' here exactly as they do for an unoverridden `legs=` there.
    rider = dataclasses.replace(
        resolve_rider(RiderSpecs(**data['rider']), include_rider=True,
                      default_variant=DEFAULT_RIDER_VARIANT),
        legs='rigid')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        specs=specs, solver=HorstLinkageSolver(specs), mode='ride', rider=rider,
        debug_markers=False, field=field, tyre_model=TyreConfig().model,
        crank_joint=False, gear_ratio=DrivetrainSpecs().gear_ratio,
        physics_config=cfg,
        mass_specs=BikeMassSpecs(**data['mass_budget'])))
    # Track raster before anything reads the field — the same write ordering
    # RideSimulation guarantees before its first forward pass.
    model.hfield_data[:] = build_field_data(track, field).reshape(-1)
    return _DisplayModel(model, mujoco.MjData(model), track, field,
                         float(recipe['start_x_m']))


class Playback:
    """Recorded qpos keyframes plus a display model — never a verified replay.

    ``verified`` is pinned ``False``: playback renders recorded states and
    cannot prove them. A checked ``ReplaySession`` report is the only artifact
    that may claim the episode was physically replayed.
    """

    verified = False

    def __init__(self, sim, times_s: np.ndarray, qpos: np.ndarray,
                 qvel: np.ndarray, duration_s: float, *, episode_id=None):
        self.sim = sim
        self.times_s = times_s
        self.qpos = qpos
        self.qvel = qvel
        self.duration_s = duration_s
        self.episode_id = episode_id
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


def _validate_frames(path, rows, model):
    """Shared recorded-frame checks for both container and legacy loaders."""
    label_prefix = f'{path}: '
    n_frames = len(rows)
    if n_frames < 2:
        raise ValueError(f'{path}: {n_frames} recorded frame(s); playback needs at least two')
    times_s, ends_s = np.empty(n_frames), np.empty(n_frames)
    qpos = np.empty((n_frames, model.nq))
    qvel = np.empty((n_frames, model.nv))
    for i, row in enumerate(rows):
        label = f'{label_prefix}frame {i} (interval_id={row.get("interval_id", "?")})'
        try:
            position = np.asarray(row['qpos'], dtype=float)
            velocity = np.asarray(row['qvel'], dtype=float)
            start_s, end_s = float(row['time_s']), float(row['interval_end_s'])
        except KeyError as exc:
            raise ValueError(f'{label} is missing {exc}') from None
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{label} has non-numeric fields') from exc
        if position.shape != (model.nq,):
            raise ValueError(f'{label} qpos length {position.size} != model.nq {model.nq}')
        if velocity.shape != (model.nv,):
            raise ValueError(f'{label} qvel length {velocity.size} != model.nv {model.nv}')
        if not (np.isfinite(position).all() and np.isfinite(velocity).all()):
            raise ValueError(f'{label} has nonfinite state')
        if not (np.isfinite(start_s) and np.isfinite(end_s)):
            raise ValueError(f'{label} has nonfinite time')
        times_s[i], ends_s[i] = start_s, end_s
        qpos[i], qvel[i] = position, velocity
    inverted = np.nonzero(ends_s <= times_s)[0]
    if inverted.size:
        i = int(inverted[0])
        raise ValueError(f'{label_prefix}frame {i} (interval_id={rows[i].get("interval_id", "?")}) '
                         'interval_end_s does not exceed time_s')
    regressed = np.nonzero(np.diff(times_s) <= 0.)[0]
    if regressed.size:
        i = int(regressed[0])+1
        raise ValueError(f'{label_prefix}frame {i} (interval_id={rows[i].get("interval_id", "?")}) '
                         'time_s is not strictly increasing')
    return times_s, qpos, qvel, float(ends_s[-1])


def _container_playback(path, episode_id):
    """Frames and the display model for a sealed DuckDB run container."""
    from bike_sim.recording.json_codec import thaw
    from bike_sim.recording.replay_input import load_playback_input
    from bike_sim.sim.research.replay import ENVELOPE_FILE, STRENGTH_FILE
    data = load_playback_input(path, episode_id=episode_id)
    owner = tempfile.TemporaryDirectory(prefix='bike-playback-input-')
    stage = Path(owner.name)
    try:
        for name in ('track.toml', ENVELOPE_FILE, STRENGTH_FILE):
            if name in data.assets:
                (stage / name).write_bytes(data.assets[name])
        sim = _display_model(stage, thaw(data.summary), thaw(data.recipe))
        playback = sim, data, owner
    except BaseException as error:
        try:
            owner.cleanup()
        except Exception as cleanup:
            error.add_note(f'playback input cleanup also failed: {cleanup}')
        raise
    return playback


def _legacy_playback(path, episode_id):
    """Frames and the display model for a fixed-file legacy directory."""
    if episode_id not in (None, 0) or isinstance(episode_id, bool):
        raise ValueError('legacy directories contain only episode 0')
    rows = _interval_rows(path)
    from bike_sim.sim.research.replay import validate_recording
    summary, manifest = validate_recording(path)
    return _display_model(path, summary, manifest), rows, None, 0


def load_playback(directory, *, episode_id=None) -> Playback:
    """Load recorded keyframes and a display model for visual playback.

    Portable inspection, not checked replay: the recorded frames are read
    through the container/legacy validators, and the model is compiled from
    the bundled authored config. No native extension is loaded and no
    execution identity is compared, so the result works across hosts for any
    supported recording — and is labeled ``verified=False`` forever.
    """
    path = Path(directory)
    # Same container detection as the checked adapter: a new database always
    # wins over legacy files and is never silently replaced by them.
    if any((path / name).exists() or (path / name).is_symlink()
           for name in ('manifest.json', 'run.duckdb', 'UNSEALED')):
        sim, data, owner = _container_playback(path, episode_id)
        rows, eid = data.frames, data.episode_id
    else:
        sim, rows, owner, eid = _legacy_playback(path, episode_id)
    times_s, qpos, qvel, duration_s = _validate_frames(path, rows, sim.model)
    playback = Playback(sim, times_s, qpos, qvel, duration_s, episode_id=eid)
    playback._assets_owner = owner
    return playback
