"""Research defaults, explicit CLI precedence and spatial mesh refinement."""
from dataclasses import asdict, replace
from pathlib import Path
import tomllib
from bike_sim.physics.checks import scalar
from bike_sim.physics.resolution import resolve_physics_config
from bike_sim.terrain.heightfield import HeightFieldSpec

# Only explicitly supplied physics flags override a TOML file. Policy setpoints
# (motor torque/limit, human request) remain a separate input surface.
PHYSICS_FLAGS = {
    'dt': ('timestep_s',),
    'initial_speed': ('initial_speed_mps',),
    'motor_max_torque': ('drive', 'assist', 'max_torque'),
    'motor_max_power': ('drive', 'assist', 'max_power'),
    'front_teeth': ('drive', 'gearing', 'front_teeth'),
    'rear_teeth': ('drive', 'gearing', 'rear_teeth'),
}


def resolve_research_physics(default_config, args):
    data = asdict(default_config)
    path = getattr(args, 'physics_config', None)
    if path is None:
        return default_config
    # Omitted material uses the class default. Do not recursively mix the linear
    # material's fields into a newly supplied tabulated constitutive law.
    for side in ('front', 'rear'):
        data['tires'][side].pop('material')
    with Path(path).open('rb') as stream:
        resolved = resolve_physics_config(data, tomllib.load(stream))
    overrides = {}
    for name in getattr(args, '_explicit_physics', ()):
        if name == 'initial_brake_demand':
            overrides.update(initial_front_brake=args.initial_brake_demand,
                             initial_rear_brake=args.initial_brake_demand)
            continue
        keys = PHYSICS_FLAGS[name]
        destination = overrides
        for key in keys[:-1]:
            destination = destination.setdefault(key, {})
        destination[keys[-1]] = getattr(args, name)
    return resolve_physics_config(asdict(resolved), overrides)


def research_field(track, resolution_m=.005):
    resolution = scalar(resolution_m, 'road resolution', positive=True)
    field = HeightFieldSpec.for_track(track)
    intervals = field.track_length_m/resolution
    count = round(intervals)
    if count < 2 or count > 1_000_000 or abs(intervals-count) > 1e-7:
        raise ValueError('road resolution must divide the field length into 2..1000000 intervals')
    return replace(field, ncol=count+1)
