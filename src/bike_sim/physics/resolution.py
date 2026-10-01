"""Strict TOML resolution: defaults < TOML < explicitly supplied CLI values."""
from dataclasses import fields
import copy
from pathlib import Path
import tomllib
from bike_sim.physics.model_config import SimulationPhysicsConfig, EndStopConfig
from bike_sim.physics.physical_config import (
    TireParameters,TireBackendConfig,PhysicalDriveConfig,AssistConfig,
    BatteryConfig,ResistanceConfig,ArticulatedConfig,PedalingConfig,ShiftingConfig,
)
from bike_sim.physics.tire import TireSpec
from bike_sim.physics.tire_curve import TabulatedTireSpec
from bike_sim.physics.distributed_tire import DistributedTireConfig, HingeDensity
from bike_sim.physics.chain import DrivetrainSpecs

CHILDREN={
    SimulationPhysicsConfig:{'end_stops':EndStopConfig,'tires':TireBackendConfig,
                             'drive':PhysicalDriveConfig,'resistance':ResistanceConfig,'articulated':ArticulatedConfig},
    TireBackendConfig:{'front':TireParameters,'rear':TireParameters,'distributed':DistributedTireConfig},
    DistributedTireConfig:{'density':HingeDensity},
    TireParameters:{'material':TireSpec},
    PhysicalDriveConfig:{'gearing':DrivetrainSpecs,'assist':AssistConfig,'battery':BatteryConfig,
                         'pedaling':PedalingConfig,'shifting':ShiftingConfig},
}


def _merge(base,override):
    result=copy.deepcopy(base)
    for key,value in override.items():
        if value is None:
            continue
        if isinstance(value,dict) and isinstance(result.get(key),dict):
            result[key]=_merge(result[key],value)
        else:
            result[key]=copy.deepcopy(value)
    return result


def _construct(cls,values):
    if not isinstance(values,dict):
        raise ValueError(f'{cls.__name__} requires a TOML table')
    if cls is TireSpec and ('deflection_m' in values or 'force_n' in values):
        cls = TabulatedTireSpec
    names={f.name for f in fields(cls)}
    unknown=set(values)-names
    if unknown:
        raise ValueError(f'unknown {cls.__name__} parameter(s): {", ".join(sorted(unknown))}')
    kwargs=copy.deepcopy(values)
    for name,child in CHILDREN.get(cls,{}).items():
        if name in kwargs:
            kwargs[name]=_construct(child,kwargs[name])
    try:
        return cls(**kwargs)
    except (TypeError,OverflowError) as exc:
        raise ValueError(f'invalid {cls.__name__}: {exc}') from exc


def resolve_physics_config(values=None,overrides=None):
    if values is None: values={}
    if overrides is None: overrides={}
    if not isinstance(values,dict) or not isinstance(overrides,dict):
        raise ValueError('configuration and overrides must be mappings')
    return _construct(SimulationPhysicsConfig,_merge(values,overrides))


def load_physics_config(path=None,overrides=None):
    values={}
    if path is not None:
        with Path(path).open('rb') as source:
            values=tomllib.load(source)
    values=resolve_config_paths(values,Path(path).parent) if path is not None else values
    return resolve_physics_config(values,overrides)


def resolve_config_paths(values, directory):
    """Resolve external model data relative to the TOML, never process cwd."""
    result=copy.deepcopy(values)
    path=result.get('articulated',{}).get('joint_envelope_path')
    if path is not None:
        file=Path(path)
        if not file.is_absolute():
            result['articulated']['joint_envelope_path']=str((Path(directory)/file).resolve())
    return result
