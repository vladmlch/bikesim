"""Explicit finite-contact research plant profile candidate factory and TOML writer.

Preserves the baseline welded preview profile completely unchanged and creates
an opt-in candidate with distributed tire materials, verified static compliance,
finite rider interface attachments (flat saddle and pedals, compliant spring grip),
and strict mechanical rider limits.
"""
from dataclasses import asdict, replace
from pathlib import Path
import json
import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.physics.distributed_tire import flat_load


def build_candidate(base, front, rear, front_material, rear_material, *,
                    compiled_radii_m, rider_power_limit_w, grip_pair_force_limit_n,
                    timestep_s=.000625):
    dt = scalar(timestep_s, 'timestep', positive=True)
    power = scalar(rider_power_limit_w, 'explicit mechanical rider budget', positive=True)
    grip = scalar(grip_pair_force_limit_n, 'explicit pair grip force limit', positive=True)
    if len(compiled_radii_m) != 2:
        raise ValueError('compiled front and rear radii are required')
    if 2 * dt > base.closure_time_constant_s:
        raise ValueError('dt must not silently alter the declared closure compliance')
    for c, m, radius in zip((front, rear), (front_material, rear_material), compiled_radii_m):
        radius = scalar(radius, 'compiled radius', positive=True)
        if abs(c.radius_m - radius) > 1e-6:
            raise ValueError('calibration radius does not match compiled wheel geometry')
        if m.fitting_dataset_id != c.dataset_id:
            raise ValueError('distributed material and radial record IDs differ')
        if m.valid_load_range_n[0] < c.material.valid_load_range_n[0] or \
                m.valid_load_range_n[1] > c.material.valid_load_range_n[1]:
            raise ValueError('distributed load domain exceeds the radial record')
        if m.valid_deflection_range_m[1] > c.material.deflection_m[-1]:
            raise ValueError('distributed deflection domain exceeds the radial record')
        d = np.asarray(c.material.deflection_m[1:])
        target = np.asarray(c.material.force_n[1:])
        actual = flat_load(d, radius, m.density)[0]
        if np.max(np.abs(actual - target) / np.maximum(target, 1.)) > .05:
            raise ValueError('distributed static response does not match radial record')
    tires = replace(base.tires, backend='distributed_2d_reference',
                    front=replace(base.tires.front, material=front.material),
                    rear=replace(base.tires.rear, material=rear.material),
                    distributed=replace(base.tires.distributed,
                                        front_material=front_material, rear_material=rear_material))
    rider = replace(base.articulated, pedal_attachment='flat', saddle_attachment='flat',
                    grip_attachment='spring', active_positive_power_limit_w=power,
                    grip_pair_force_limit_n=grip)
    drive = replace(base.drive,
                    shifting=replace(base.drive.shifting, upshift_slip_mode='magnitude'),
                    pedaling=replace(base.drive.pedaling, rollback_brake=False))
    return replace(base, physics_mode='physical', drive_mode='articulated_effort',
                   timestep_s=dt, tires=tires, articulated=rider, drive=drive,
                   seated_climb=replace(base.seated_climb, enabled=False))


def _toml_value(value):
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_toml_value(v) for v in value) + ']'
    if type(value) in (str, bool, int, float):
        return json.dumps(value, ensure_ascii=True, allow_nan=False)
    raise ValueError('unsupported physics configuration scalar')


def write_physics_toml(config, path: Path) -> None:
    lines = ['# Explicit candidate; not real-world safety validation.']

    def table(values, prefix=()):
        if prefix:
            lines.extend(['', '[' + '.'.join(prefix) + ']'])
        for key, value in values.items():
            if not key.isidentifier():
                raise ValueError('invalid configuration key')
            if value is not None and not isinstance(value, dict):
                lines.append(key + ' = ' + _toml_value(value))
        for key, value in values.items():
            if isinstance(value, dict):
                table(value, prefix + (key,))

    table(asdict(config))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError('candidate output must not overwrite an existing profile')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
