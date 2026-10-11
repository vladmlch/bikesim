"""Owned physical presentation values and isolated MuJoCo render replicas.

A replica is never an authoritative force/sensor source. Only integration state
and explicitly declared mutable model fields cross into it; kinematics are
recomputed without stepping, force evaluation, or sensor acquisition.
"""
from __future__ import annotations

from copy import copy
import math
from types import MappingProxyType

import mujoco
import numpy as np

from bike_sim.native.contracts import (
    FrameSnapshot, PhysicalViewState, _amend, _LazyMap,
)

MODEL_FIELDS = ('dof_frictionloss', 'site_pos', 'tendon_stiffness',
                'tendon_damping', 'tendon_lengthspring', 'tendon_range')


def _hud(sim):
    from bike_sim.sim.ride.hud import RideHUD
    hud = getattr(sim, '_presentation_hud', None)
    if hud is None:
        hud = RideHUD(sim.model)
        sim._presentation_hud = hud
    return hud


def make_physical_view(sim, *, requested_scale=1, achieved_rtf=None):
    """Capture the Python oracle's existing HUD/CSV values, including blanks."""
    hud = _hud(sim)
    row = hud._preview_log_row(sim)
    row['rtf'] = achieved_rtf
    row['requested_scale'] = requested_scale
    sample = sim.physical.sample
    joints = {}
    for index in range(sim.model.njnt):
        name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_JOINT, index)
        if name and name.startswith('rider_'):
            joints[name] = float(sim.data.qpos[sim.model.jnt_qposadr[index]])
    endpoint = dict(time_s=float(sim.time_s), position_m=float(sim.position_m),
        speed_mps=float(sim.speed_mps), pitch_rad=float(sim.pitch_rad),
        z_m=float(sim.data.xpos[sim._frame_body_id, 2]),
        torso_pitch_deg=hud._rider_body_pitch_deg(sim, 'rider_torso'),
        root_x_dofadr=int(sim.root_x_dofadr), rider_joint_angles_rad=joints,
        battery_energy_j=float(sim.physical.drive.battery.energy_j),
        balance_lost_at_m=None if sim.physical.balance_monitor.event is None else
            float(sim.physical.balance_monitor.event.position_m))
    return PhysicalViewState(channels={} if sample is None else sample.channels,
        drive_mode=sim.physics_config.drive_mode, sample=sample, endpoint=endpoint,
        preview_row=row, requested_scale=requested_scale, achieved_rtf=achieved_rtf)


def _terrain_fields(track, position_m):
    """The three road annotations at one position, exactly as the HUD row."""
    from bike_sim.sim.ride.hud import nearest_obstacle
    obstacle = nearest_obstacle(track, position_m)
    return dict(grade_pct=0. if track.grade_profile is None else
                100. * track.grade_profile.slope(position_m),
                obstacle='' if obstacle is None else obstacle[0],
                obstacle_distance_m='' if obstacle is None else float(obstacle[1]))


def present_view(view, *, requested_scale=1, achieved_rtf=None, track=None,
                 terrain=None):
    """Attach wall-clock values and immutable road annotations, never forces.

    ``terrain`` optionally supplies a precomputed ``_terrain_fields`` result;
    it is merged only when ``track`` is given.
    """
    row = dict(view.preview_row, rtf=achieved_rtf, requested_scale=requested_scale)
    if track is not None:
        row.update(_terrain_fields(track, view.endpoint['position_m'])
                   if terrain is None else terrain)
    # _amend skips __post_init__, so the row must be frozen explicitly. It is
    # depth-1 and already holds only immutable scalars, so a shallow proxy
    # preserves the read-only contract without a recursive walk.
    return _amend(view, preview_row=MappingProxyType(row),
                  requested_scale=requested_scale, achieved_rtf=achieved_rtf)


_BLANK_VIEW = PhysicalViewState()


def native_frame(value, sample):
    """Box the native envelope without borrowing mjData or writer scratch."""
    payload = value.view
    # sample.channels was frozen at publication and passes through. The view
    # payload, model_status and model_fields are fresh exclusively-owned
    # dictionaries per snapshot, so lazy read-only views own them without an
    # eager recursive freeze. first_failure keeps an eager freeze: equality
    # against tuples is part of its contract.
    view = _amend(_BLANK_VIEW,
        channels=_BLANK_VIEW.channels if sample is None else sample.channels,
        drive_mode=payload['drive_mode'], sample=sample,
        endpoint=_LazyMap(payload['endpoint']),
        preview_row=_LazyMap(payload['preview_row']))
    # Fresh capsule-owned vector per access; the readonly mark turns the
    # _owned_array check in __post_init__ into a pass-through.
    state = value.integration_state
    state.flags.writeable = False
    return FrameSnapshot(generation=int(value.generation), step=int(value.step),
        time_s=float(value.time_s), integration_state=state,
        latest_sample=sample, view=view, outcome=value.outcome,
        first_failure=value.first_failure,
        model_status=_LazyMap(value.model_status),
        model_fields=_LazyMap(payload['model_fields']))


def snapshot_python(sim, *, outcome=None):
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(sim.model, signature), dtype=np.float64)
    mujoco.mj_getState(sim.model, sim.data, state, signature)
    view = make_physical_view(sim)
    return FrameSnapshot(generation=sim.physical.generation, step=sim.steps,
        time_s=sim.time_s, integration_state=state, latest_sample=view.sample,
        view=view, outcome=outcome,
        first_failure=sim.physical.reference_monitor.first_failure,
        model_status=sim.physical.model_status.as_dict(),
        model_fields={name: np.array(getattr(sim.model, name), copy=True)
                      for name in MODEL_FIELDS})


def apply_frame(model, data, frame: FrameSnapshot) -> None:
    """Validate first, copy a complete frame, then update replica geometry only.

    Reference: supplied MuJoCo include/mujoco/{mjtype,mujoco}.h and
    src/engine/engine_support.c (mjSTATE_INTEGRATION / mj_setState).
    The caller owns this model/data pair; shape equality is not model identity.
    """
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    state = frame.integration_state
    width = mujoco.mj_stateSize(model, signature)
    if state.shape != (width,) or not np.isfinite(state).all():
        raise ValueError('render frame: incompatible or nonfinite integration state')
    if not math.isfinite(frame.time_s) or float(state[0]) != frame.time_s:
        raise ValueError('render frame time does not match integration state')
    if set(frame.model_fields) != set(MODEL_FIELDS):
        raise ValueError('render frame: incomplete mutable model fields')
    staged = []
    for name, value in frame.model_fields.items():
        if name not in MODEL_FIELDS:
            raise ValueError(f'render frame: unsupported model field {name}')
        destination = getattr(model, name)
        source = np.asarray(value, dtype=np.float64)
        if source.size != destination.size or not np.isfinite(source).all():
            raise ValueError(f'render frame: incompatible model field {name}')
        staged.append((destination, source.reshape(destination.shape)))
    for destination, source in staged:
        np.copyto(destination, source)
    mujoco.mj_setState(model, data, state, signature)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_camlight(model, data)
    mujoco.mj_tendon(model, data)


class RenderReplica:
    """The frontend retains the frame throughout sync; arrays outlive reset."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.frame = None

    def apply(self, frame):
        if self.frame is not None:
            if frame.generation < self.frame.generation or (
                    frame.generation == self.frame.generation and frame.step < self.frame.step):
                raise ValueError('render frame moved backwards without a generation change')
        apply_frame(self.model, self.data, frame)
        self.frame = frame


def environment_snapshot(env):
    if getattr(env, 'backend', 'python') == 'native':
        frame = env.snapshot()
        return _amend(frame, outcome=env.reason or frame.outcome)
    return snapshot_python(env.sim, outcome=env.reason)


def environment_replica(env):
    model = (env.make_render_model() if getattr(env, 'backend', 'python') == 'native'
             else copy(env.sim.model))
    replica = RenderReplica(model)
    replica.apply(environment_snapshot(env))
    return replica
