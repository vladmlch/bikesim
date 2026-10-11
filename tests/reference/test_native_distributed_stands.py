"""Native distributed-tire stands: the contact_manifold_rigs flat/step rigs
running on the native Stepper's distributed writer.

The loop mirrors validation/contact_manifold_rigs.py::_run_stand line for
line on the SAME wheel_stand models (hfield terrain, collision-disabled
wheel geoms): the native writer's ``tire_qfrc`` supplies the interval
force while the energy ledger reads native diagnostics, the native
``kinetic_energy``/``tire_stored_energy`` accessors and the MuJoCo state
exposed through the Stepper views — nothing is reconstructed from a
shared force matrix. The asserted bounds are the oracle rigs' own.
"""
import numpy as np
import mujoco
import pytest
from scipy.optimize import brentq

from native_loader import load_native
bike_native = load_native()

from bike_sim.physics.distributed_tire import flat_load
from bike_sim.validation.contact_manifold_rigs import (
    contact_case_vertices, wheel_stand)
from tools.native_config import _tire_section

DT_S = .000625
STATION_COUNT = 128
FLAT_LOAD_N = 600.
STEP_TERRAIN_DX_M = .005


def _stand(tmp_path, *, vertices=None, dt_s=DT_S, stations=STATION_COUNT):
    """Build wheel_stand and hand its model + minimal tire-only config to
    the native Stepper. The Python applier is used only for the oracle's
    own setup inputs (material density for the depth solve, config
    projection) — never for forces or state inside the loop."""
    model, data, tire = wheel_stand(dt_s, stations, vertices=vertices)
    path = tmp_path / 'stand.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    # Minimal valid Stepper config: only the tire writer section —
    # configured surface mode needs no surface map, and every other root
    # section is optional.
    config = {'schema': 2, 'tire': _tire_section(tire.config, None)}
    stepper = bike_native.Stepper(str(path), config)
    return model, tire, stepper


def _compress_depth(tire, load_n, radius=.37):
    """The oracle rigs' own initial compression: brentq on the analytic
    flat_load of the front wheel's density material."""
    return brentq(
        lambda x: float(flat_load(
            x, radius, tire.materials['front'].density, STATION_COUNT)[0])
        - load_n, 1e-9, .1, xtol=1e-14)


def _set_stand_state(stepper, qpos, qvel):
    stepper.set_state(list(map(float, qpos)), list(map(float, qvel)),
                      [], [0., 0., 0.], 0.)
    stepper.forward()


def _run_stand_native(stepper, load_n, duration_s, dt_s):
    """contact_manifold_rigs._run_stand with the native Stepper as the
    force source and its accessors as the ledger instruments."""
    count = round(duration_s / dt_s)
    if abs(count * dt_s - duration_s) > 1e-10 or count < 1:
        raise ValueError('stand duration must have an integer number of intervals')
    initial = (stepper.kinetic_energy() + stepper.tire_stored_energy()
               + load_n * float(np.asarray(stepper.qpos)[1]))
    scale = max(1., stepper.kinetic_energy() + stepper.tire_stored_energy())
    loss = 0.
    residual = 0.
    max_residual = 0.
    native_contacts = 0
    rows = []
    for i in range(count):
        force = np.asarray(stepper.tire_qfrc(dt_s), dtype=np.float64)
        front = stepper.tire_diagnostics()['front']
        snap = stepper.tire_snapshots()['front']
        rows.append({
            'time_s': i * dt_s, 'end_time_s': (i + 1) * dt_s,
            'normal_load_n': front['normal_load_n'],
            'position_m': float(np.asarray(stepper.qpos)[0]),
            'pitch_rate_rad_s': float(np.asarray(stepper.qvel)[2]),
            'patch_count': len(snap['patches']),
            'multi_support': bool(front['multi_support'])})
        loss += (stepper.tire_brush_loss_step_j()
                 + stepper.tire_radial_dissipation_power_w() * dt_s)
        applied = force
        applied[1] -= load_n
        stepper.set_inputs([], applied)
        stepper.step()
        stepper.forward()
        native_contacts = max(native_contacts, stepper.contact_count)
        qpos = np.asarray(stepper.qpos)
        qvel = np.asarray(stepper.qvel)
        if not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
            raise ArithmeticError('non-finite stand trajectory')
        current = (stepper.kinetic_energy() + stepper.tire_stored_energy()
                   + load_n * float(qpos[1]))
        residual = current - initial + loss
        max_residual = max(max_residual, abs(residual))
    return rows, {
        'energy_residual_j': residual,
        'energy_residual_ratio': abs(residual) / (scale + loss),
        'max_energy_residual_j': max_residual,
        'dissipation_j': loss,
        'native_wheel_contact_count': native_contacts,
        'final_speed_mps': float(np.asarray(stepper.qvel)[0])}


def test_distributed_flat_stand_native(tmp_path):
    """distributed_flat_rig's cell (dt=.000625, 128 stations, 600 N) on the
    native writer — the rig's own bounds."""
    model, tire, stepper = _stand(tmp_path)
    depth = _compress_depth(tire, FLAT_LOAD_N)
    qpos = np.asarray(stepper.qpos).copy()
    qpos[1] = -depth
    _set_stand_state(stepper, qpos, np.zeros(model.nv))
    rows, metrics = _run_stand_native(stepper, FLAT_LOAD_N, .05, DT_S)
    metrics.update(
        normal_force_n=float(np.mean([r['normal_load_n'] for r in rows])),
        deflection_m=depth,
        force_relative_error=abs(rows[-1]['normal_load_n'] - FLAT_LOAD_N)
        / FLAT_LOAD_N,
        station_count=STATION_COUNT)
    bounds = {'force_relative_error': (0., .02),
              'native_wheel_contact_count': (0., 0.),
              'energy_residual_ratio': (0., .02)}
    for name, (lo, hi) in bounds.items():
        assert lo <= metrics[name] <= hi, f'{name}={metrics[name]} ({metrics})'


def test_distributed_step_stand_native(tmp_path):
    """distributed_step_rig's cell (dt=.000625, 128 stations, dx=.005) on
    the native writer — energy and multi-patch bounds."""
    vertices = contact_case_vertices('step', STEP_TERRAIN_DX_M)
    model, tire, stepper = _stand(tmp_path, vertices=vertices)
    load_n = 600.
    depth = _compress_depth(tire, load_n)
    qpos = np.zeros(model.nq)
    qpos[0] = -.45
    qpos[1] = float(np.interp(-.45, vertices[:, 0], vertices[:, 1])) - depth
    qvel = np.zeros(model.nv)
    qvel[0] = 2.
    qvel[2] = 2. / .37
    _set_stand_state(stepper, qpos, qvel)
    rows, metrics = _run_stand_native(stepper, load_n, .4, DT_S)
    metrics.update(
        normal_impulse_ns=sum(r['normal_load_n'] * DT_S for r in rows),
        max_patch_count=max(r['patch_count'] for r in rows),
        multi_support_fraction=sum(r['multi_support'] for r in rows)
        / len(rows),
        station_count=STATION_COUNT, terrain_dx_m=STEP_TERRAIN_DX_M)
    bounds = {'native_wheel_contact_count': (0., 0.),
              'energy_residual_ratio': (0., .05),
              'max_patch_count': (2., float('inf'))}
    for name, (lo, hi) in bounds.items():
        assert lo <= metrics[name] <= hi, f'{name}={metrics[name]} ({metrics})'
