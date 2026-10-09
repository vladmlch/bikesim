"""Research defaults, explicit CLI precedence and spatial mesh refinement."""
from dataclasses import asdict, replace
from pathlib import Path
import tomllib
from bike_sim.physics.checks import scalar
from bike_sim.physics.resolution import resolve_physics_config
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.research.environment import ResearchEnvironment
from bike_sim.terrain.heightfield import HeightFieldSpec

DEFAULT_TIME_STEPS = {'ideal_mid_drive': .0005, 'geometric_ideal_mid_drive': .0005,
                      'elastic_chain': .000125}

PHYSICS_FLAGS = {
    'dt': ('timestep_s',),
    'initial_speed': ('initial_speed_mps',),
    'motor_max_torque': ('drive', 'assist', 'max_torque'),
    'motor_max_power': ('drive', 'assist', 'max_power'),
    'front_teeth': ('drive', 'gearing', 'front_teeth'),
    'rear_teeth': ('drive', 'gearing', 'rear_teeth'),
    'human_torque': ('drive', 'human_torque_nm'),
    'transmission': ('drive', 'transmission_model'),
    'chain_stiffness': ('drive', 'chain_k_n_m'),
    'freehub_stiffness': ('drive', 'freehub_k_nm_rad'),
}


def resolve_research_physics(default_config, args):
    data = asdict(default_config)
    path = getattr(args, 'physics_config', None)
    overrides = {}
    if path is not None:
        from bike_sim.physics.resolution import resolve_config_paths
        with Path(path).open('rb') as stream:
            overrides = resolve_config_paths(tomllib.load(stream), Path(path).parent)
        for side in ('front', 'rear'):
            if 'material' in overrides.get('tires', {}).get(side, {}):
                data['tires'][side].pop('material')
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
    transmission = overrides.get('drive', {}).get('transmission_model',
                                                 default_config.drive.transmission_model)
    if ('timestep_s' not in overrides and transmission in DEFAULT_TIME_STEPS
            and transmission != default_config.drive.transmission_model):
        overrides['timestep_s'] = DEFAULT_TIME_STEPS[transmission]
    if 'closure_time_constant_s' not in overrides and 'timestep_s' in overrides:
        if overrides['timestep_s'] * 2.0 > data.get('closure_time_constant_s', 0.001):
            overrides['closure_time_constant_s'] = 0.0025
    return resolve_physics_config(data, overrides)


def research_field(track, resolution_m=.005):
    resolution = scalar(resolution_m, 'road resolution', positive=True)
    field = HeightFieldSpec.for_track(track)
    intervals = field.track_length_m/resolution
    count = round(intervals)
    if count < 2 or count > 1_000_000 or abs(intervals-count) > 1e-7:
        raise ValueError('road resolution must divide the field length into 2..1000000 intervals')
    return replace(field, ncol=count+1)


def build_environment(*, track, rider, physics_config, experiment, sensors,
                      road_resolution_m=.005, demand=None, rider_program=None, rider_behavior=None,
                      backend="python"):
    from bike_sim.sim.backend import require_backend
    require_backend(backend, physics_config, rider)
    sim = RideSimulation(track=track, rider=rider, physics_config=physics_config,
                         field=research_field(track, road_resolution_m))
    reference = ResearchEnvironment(sim, experiment, sensors, demand=demand,
                                    rider_program=rider_program, rider_behavior=rider_behavior)
    if backend == "python":
        return reference
    from bike_sim.native.research import create_native_research
    try:
        return create_native_research(reference)
    finally:
        # The native adapter owns copies, not this temporary setup owner.
        reference.close(discard_pending=True)
