"""Reviewed field and coordinate catalog for run recordings (R08, R09).

Every canonical field is declared here with its DuckDB type, unit, physical
frame/reference, phase, authoritative source, optionality and measurement
semantics. Channel fields are inventoried from the schema-2 record the
runtime publishes (``PhysicalSample.channels`` in
``sim/ride/physical_runtime.py`` and ``native/src/runtime/samples.cpp``);
legacy flat-column names from ``sim/ride/physical_recorder.py`` /
``PhysicalSampleData::flat_columns`` survive only as aliases resolved to the
canonical field they mean — never as duplicated columns.

Model facts (joint/qpos/DOF maps) arrive through ``build_catalog`` so the
coordinate catalog and the indexed aliases reflect the actual model rather
than a guessed layout.
"""
from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

DTYPES = frozenset({
    'bigint', 'double', 'boolean', 'varchar',
    'bigint[]', 'double[]', 'varchar[]',
    'map<bigint>', 'map<double>', 'map<boolean>', 'map<varchar>',
    'map<double[]>', 'struct', 'struct[]', 'map<struct>',
})
PHASES = frozenset({
    'interval_start', 'interval', 'interval_end', 'control_period',
    'episode', 'n/a',
})
MEASUREMENTS = frozenset({
    'measured', 'forwarded', 'solved', 'derived', 'declared', 'diagnostic',
})
_UNITLESS = {'1', 'n/a', 'per_coordinate'}

_SRC_SAMPLES = 'native/src/runtime/samples.cpp::PhysicalSampleData::flat_columns'
_SRC_RECORDER = 'src/bike_sim/sim/ride/physical_recorder.py::PhysicalRecorder._row'
_SRC_RUNTIME = 'src/bike_sim/sim/ride/physical_runtime.py::PhysicalRuntime'
_SRC_CLOSE = (_SRC_RUNTIME + '._close_period')
_SRC_ADVANCE = (_SRC_RUNTIME + '._advance_physics')
_SRC_OBSERVE = 'src/bike_sim/sim/ride/physical_observations.py'


@dataclass(frozen=True)
class FieldSpec:
    """One canonical field inside a table (R09)."""

    key: str
    dtype: str
    unit: str
    phase: str
    optional: bool
    measurement: str
    source: str
    description: str
    frame: str = 'n/a'
    path: tuple = ()            # path inside the row tree; default (key,)
    aliases: tuple = ()         # legacy flat names this field replaces
    shape: tuple | None = None
    axes: tuple | None = None
    except_keys: tuple = ()     # map<struct>: members owned by sibling fields
    children: tuple = ()        # struct/struct[]/map<struct> child schema

    def __post_init__(self) -> None:
        if not isinstance(self.key, str) or not self.key:
            raise ValueError('field key must be a non-empty string')
        if self.dtype not in DTYPES:
            raise ValueError(f'{self.key}: unknown dtype {self.dtype!r}')
        if self.phase not in PHASES:
            raise ValueError(f'{self.key}: unknown phase {self.phase!r}')
        if self.measurement not in MEASUREMENTS:
            raise ValueError(
                f'{self.key}: unknown measurement {self.measurement!r}')
        if not isinstance(self.optional, bool):
            raise ValueError(f'{self.key}: optional must be a bool')
        if not self.unit or not self.source or not self.description:
            raise ValueError(
                f'{self.key}: unit, source and description are required')
        if self.shape is not None and self.axes is not None \
                and tuple(self.shape) != (len(self.axes),):
            raise ValueError(f'{self.key}: shape/axes mismatch')
        object.__setattr__(self, 'path',
                           tuple(self.path) if self.path else (self.key,))
        object.__setattr__(self, 'aliases', tuple(self.aliases))
        object.__setattr__(self, 'except_keys', tuple(self.except_keys))
        object.__setattr__(self, 'children', tuple(self.children))
        if self.dtype in ('struct', 'struct[]', 'map<struct>') \
                and not self.children:
            raise ValueError(f'{self.key}: structured dtype needs children')

    @property
    def resolved_path(self) -> tuple:
        return self.path

    def duckdb_type(self) -> str:
        scalar = {
            'bigint': 'BIGINT', 'double': 'DOUBLE', 'boolean': 'BOOLEAN',
            'varchar': 'VARCHAR', 'bigint[]': 'BIGINT[]',
            'double[]': 'DOUBLE[]', 'varchar[]': 'VARCHAR[]',
            'map<bigint>': 'MAP(VARCHAR, BIGINT)',
            'map<double>': 'MAP(VARCHAR, DOUBLE)',
            'map<boolean>': 'MAP(VARCHAR, BOOLEAN)',
            'map<varchar>': 'MAP(VARCHAR, VARCHAR)',
            'map<double[]>': 'MAP(VARCHAR, DOUBLE[])',
        }
        if self.dtype in scalar:
            return scalar[self.dtype]
        inner = ', '.join(f'{c.key} {c.duckdb_type()}' for c in self.children)
        struct = f'STRUCT<{inner}>'
        if self.dtype == 'struct':
            return struct
        if self.dtype == 'struct[]':
            return struct + '[]'
        if self.dtype == 'map<struct>':
            return f'MAP(VARCHAR, {struct})'
        raise ValueError(f'{self.key}: cannot render dtype {self.dtype!r}')


@dataclass(frozen=True)
class TableSpec:
    """A finite recording table generated from the reviewed field set."""

    name: str
    kind: str                   # 'physics' | 'forces' | 'channels'
    keys: tuple
    fields: tuple
    description: str
    source: str
    claims: frozenset = frozenset()   # owned top-level ``channels`` roots
    exempt_keys: tuple = ()           # row keys owned by other tables

    def __post_init__(self) -> None:
        if self.kind not in ('physics', 'forces', 'channels'):
            raise ValueError(f'{self.name}: unknown table kind {self.kind!r}')
        object.__setattr__(self, 'keys', tuple(self.keys))
        object.__setattr__(self, 'fields', tuple(self.fields))
        object.__setattr__(self, 'claims', frozenset(self.claims))
        object.__setattr__(self, 'exempt_keys', tuple(self.exempt_keys))
        names = [k.key for k in self.keys] + [f.key for f in self.fields]
        if len(names) != len(set(names)):
            raise ValueError(f'{self.name}: duplicate field keys')
        paths = [f.path for f in self.fields]
        if len(paths) != len(set(paths)):
            raise ValueError(f'{self.name}: duplicate field paths')
        containers = {'struct', 'struct[]', 'map<struct>'}
        for terminal in self.fields:
            depth = len(terminal.path)
            if any(len(p) > depth and p[:depth] == terminal.path
                   for p in paths):
                if not (terminal.dtype.startswith('map<')
                        or terminal.dtype in containers):
                    raise ValueError(
                        f'{self.name}: field {terminal.key} at '
                        f'{terminal.path} must be a container to share a '
                        'prefix with deeper fields')
        if self.kind == 'channels' and not self.claims:
            raise ValueError(f'{self.name}: channel table must claim roots')

    @property
    def required_fields(self) -> tuple:
        return tuple(f for f in self.fields if not f.optional)


@dataclass(frozen=True)
class JointCoords:
    """One model joint's coordinate block (qpos address and DOF address)."""

    name: str
    joint_type: str             # 'free' | 'ball' | 'hinge' | 'slide'
    qposadr: int
    dofadr: int


@dataclass(frozen=True)
class Coordinate:
    """One qpos or DOF index with its joint, member label and unit."""

    index: int
    kind: str                   # 'qpos' | 'dof'
    joint: str
    component: str              # member label ('' for scalar joints)
    dof_kind: str               # 'translation' | 'rotation' | 'quaternion'
    unit: str


@dataclass(frozen=True)
class CoordinateCatalog:
    """Model-specific qpos/DOF address, name and unit maps (R09)."""

    qpos: tuple
    dof: tuple

    def force_unit(self, dof_index: int) -> str:
        """Generalized-force unit for a DOF: N translational, N·m rotational."""
        coordinate = self.dof[dof_index]
        return {'translation': 'N', 'rotation': 'N·m',
                'quaternion': 'n/a'}[coordinate.dof_kind]

    def velocity_unit(self, dof_index: int) -> str:
        return self.dof[dof_index].unit


@dataclass(frozen=True)
class AliasSpec:
    """A legacy flat-column name resolved to a canonical field (R09)."""

    alias: str
    canonical: str              # 'table.field' (dot may index a member)
    index: int | None = None    # vector element when the alias names one
    parameter: str | None = None  # map member captured by a template
    note: str = ''


def _f(key, dtype, unit, phase, optional, measurement, source, description,
       **kwargs):
    return FieldSpec(key=key, dtype=dtype, unit=unit, phase=phase,
                     optional=optional, measurement=measurement,
                     source=source, description=description, **kwargs)


_EPISODE_KEY = _f('episode_id', 'bigint', 'n/a', 'n/a', False, 'declared',
                  'docs/superpowers/specs/2026-10-12-run-recording-design.md::R01',
                  'episode identity within the owning run')
_INTERVAL_KEY = _f('interval_id', 'bigint', 'n/a', 'interval', False,
                   'declared', _SRC_SAMPLES,
                   'accounted physics interval identifier, consecutive per '
                   'episode')
_COMPONENT_KEY = _f('component', 'varchar', 'n/a', 'interval', False,
                    'declared', _SRC_RUNTIME + '.apply_forces',
                    'named generalized-force component')


def _physics_table() -> TableSpec:
    fields = (
        _f('time_s', 'double', 's', 'interval_start', False, 'measured',
           _SRC_SAMPLES,
           'simulation time at the start of the interval; the incoming '
           '("prestep") timestamp', aliases=('interval_start_s',)),
        _f('interval_end_s', 'double', 's', 'interval_end', False, 'measured',
           _SRC_SAMPLES,
           'simulation time at the end of the interval', path=('end_time_s',)),
        _f('dt_s', 'double', 's', 'interval', False, 'derived',
           _SRC_SAMPLES,
           'interval duration end_time_s - time_s; always positive',
           aliases=('interval_dt_s',)),
        _f('qpos', 'double[]', 'per_coordinate', 'interval_start', False,
           'measured', _SRC_SAMPLES,
           'incoming generalized position vector at the interval start '
           '("prestep" state); per-element units and names follow the qpos '
           'coordinate map',
           frame='model qpos addresses (coordinate_catalog)'),
        _f('qvel', 'double[]', 'per_coordinate', 'interval_start', False,
           'measured', _SRC_SAMPLES,
           'incoming generalized velocity vector at the interval start '
           '("prestep" state); per-element units follow the DOF map',
           frame='model DOF addresses (coordinate_catalog)'),
    )
    return TableSpec('physics', 'physics', (_EPISODE_KEY, _INTERVAL_KEY),
                     fields,
                     'per-interval incoming state, interval times and '
                     'measurement provenance', _SRC_SAMPLES,
                     exempt_keys=('channels', 'forces', 'powers_w'))


def _forces_table() -> TableSpec:
    fields = (
        _f('qfrc', 'double[]', 'per_coordinate', 'interval', False, 'solved',
           _SRC_CLOSE,
           'component generalized-force vector applied across the interval; '
           'per-DOF units follow the DOF map (N translational, N·m '
           'rotational)', frame='generalized forces (coordinate_catalog)'),
        _f('power_w', 'double', 'W', 'interval', False, 'derived',
           _SRC_RECORDER,
           'scalar interval power of this component (qfrc @ incoming qvel); '
           'recorded from the source dot product, never recomputed from '
           'rounded values', aliases=('powers_w.{component}',
                                      'power_{component}_w',
                                      '{component}_power_w')),
    )
    return TableSpec('forces', 'forces',
                     (_EPISODE_KEY, _INTERVAL_KEY, _COMPONENT_KEY), fields,
                     'per-component generalized forces and interval powers',
                     _SRC_SAMPLES)


def _mass_table() -> TableSpec:
    mass_src = 'src/bike_sim/sim/ride/physical_energy.py::mass_observations'
    com_src = 'src/bike_sim/sim/ride/physical_energy.py::com_observations'
    members = (
        ('mass_kg', 'double', 'kg', False,
         'compiled total model mass', com_src, None),
        ('com_m', 'double[]', 'm', False,
         'compiled centre-of-mass position at the incoming state', com_src, ('x', 'y', 'z')),
        ('com_velocity_mps', 'double[]', 'm/s', True,
         'compiled centre-of-mass velocity (full measurement intervals '
         'only)', mass_src, ('x', 'y', 'z')),
        ('linear_momentum_kg_mps', 'double[]', 'kg·m/s', True,
         'spatial linear momentum (full measurement intervals only)',
         mass_src, ('x', 'y', 'z')),
        ('angular_momentum_kg_m2_s', 'double[]', 'kg·m²/s', True,
         'spatial angular momentum (full measurement intervals only)',
         mass_src, ('x', 'y', 'z')),
        ('kinetic_energy_j', 'double', 'J', True,
         'kinetic energy ½·qvelᵀ·M·qvel (full measurement intervals only)',
         mass_src, None),
        ('gravitational_energy_j', 'double', 'J', True,
         'gravitational potential energy (full measurement intervals only)',
         mass_src, None),
    )
    children = tuple(
        _f(key, dtype, unit, 'interval_start', optional, 'measured', source,
           description,
           shape=(3,) if axes else None, axes=axes)
        for key, dtype, unit, optional, description, source, axes in members)
    fields = tuple(
        _f(key, dtype, unit, 'interval_start', optional, 'measured', source,
           description, path=('mass', key),
           shape=(3,) if axes else None, axes=axes)
        for key, dtype, unit, optional, description, source, axes in members)
    fields += (
        _f('endpoint_mass', 'struct', 'n/a', 'interval_end', True,
           'forwarded', _SRC_CLOSE,
           'mass observations at the post-solve endpoint of a measured '
           'interval; forwarded from the period-closing measurement on '
           'unmeasured intervals', children=children,
           path=('endpoint_mass',)),
        _f('measurement_scope', 'varchar', 'n/a', 'interval', True,
           'derived', _SRC_ADVANCE,
           "which mass measurement ran: 'full' (mass_observations with "
           "energy/momentum) or 'com' (com_observations kinematic only)",
           path=('mass', 'measurement_scope')),
    )
    return TableSpec('channel_mass', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'compiled mass, centre of mass and momentum channels',
                     _SRC_ADVANCE, claims=frozenset({'mass', 'endpoint_mass'}))


def _energy_table() -> TableSpec:
    e = _SRC_CLOSE
    fields = (
        _f('mechanical_energy_j', 'double', 'J', 'interval_end', False,
           'forwarded', e,
           'mechanical energy of the last measured interval; on unmeasured '
           'intervals this is the forwarded period-closing measurement, not '
           'a fresh instantaneous value',
           path=('energy', 'mechanical_energy_j'),
           aliases=('mechanical_energy_j',)),
        _f('elastic_energy_j', 'map<double>', 'J', 'interval_end', True,
           'forwarded', e,
           'stored elastic energy by term (forwarded with the closing '
           'measurement when unmeasured)',
           path=('energy', 'elastic_energy_j')),
        _f('active_work_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative muscle + motor work over the episode',
           path=('energy', 'active_work_j')),
        _f('external_work_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative signed work of external/environment components',
           path=('energy', 'external_work_j')),
        _f('loss_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative material dissipation (dampers, brush, bearings)',
           path=('energy', 'loss_j')),
        _f('loss_step_j', 'double', 'J', 'interval', False, 'derived',
           _SRC_RUNTIME + '._loss_increment',
           'material dissipation of this interval',
           path=('energy', 'loss_step_j')),
        _f('solver_constraint_work_j', 'double', 'J', 'episode', False,
           'derived', e,
           'cumulative signed work of equality/limit constraint rows',
           path=('energy', 'solver_constraint_work_j')),
        _f('energy_scale_j', 'double', 'J', 'episode', False, 'derived',
           _SRC_RUNTIME + '.reset',
           'energy normalization scale fixed at episode initialization',
           path=('energy', 'energy_scale_j')),
        _f('muscle_signed_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative signed muscle (rider actuator + crank) work',
           path=('energy', 'muscle_signed_j')),
        _f('muscle_positive_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative positive muscle work', path=('energy', 'muscle_positive_j')),
        _f('motor_signed_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative signed motor work', path=('energy', 'motor_signed_j')),
        _f('motor_positive_j', 'double', 'J', 'episode', False, 'derived', e,
           'cumulative positive motor work', path=('energy', 'motor_positive_j')),
        _f('source_positive_work_j', 'double', 'J', 'episode', False,
           'derived', e,
           'muscle_positive_j + motor_positive_j reference for the '
           'constraint-work ratio', path=('energy', 'source_positive_work_j')),
        _f('constraint_signed_j', 'double', 'J', 'episode', False, 'derived',
           e, 'cumulative signed numerical constraint work',
           path=('energy', 'constraint_signed_j')),
        _f('constraint_absolute_j', 'double', 'J', 'episode', False,
           'derived', e,
           'cumulative absolute numerical constraint work',
           path=('energy', 'constraint_absolute_j')),
        _f('constraint_work_ratio', 'double', '1', 'episode', True,
           'derived', e,
           'constraint_absolute_j / source_positive_work_j; NULL when no '
           'positive source work exists yet (division is undefined, not '
           'zero)', path=('energy', 'constraint_work_ratio')),
        _f('constraint_work_ok', 'boolean', 'n/a', 'interval_end', False,
           'derived', e,
           'integral energy-closure verdict evaluated at the period-closing '
           'interval', path=('energy', 'constraint_work_ok')),
        _f('residual_j', 'double', 'J', 'interval_end', True, 'forwarded', e,
           'hybrid energy residual vs the initial datum; forwarded with the '
           'closing measurement and excluded from the validity verdict when '
           'unmeasured', path=('energy', 'residual_j'),
           aliases=('energy_residual_j',)),
        _f('electrical_work_j', 'double', 'J', 'episode', False, 'derived',
           e, 'cumulative electrical work drawn from the battery model',
           path=('energy', 'electrical_work_j')),
        _f('electrical_residual_j', 'double', 'J', 'episode', False,
           'derived', e,
           'battery bookkeeping residual (0 when the battery is disabled)',
           path=('energy', 'electrical_residual_j')),
        _f('measured', 'boolean', 'n/a', 'interval', False, 'declared',
           _SRC_CLOSE,
           'whether this row carries a fresh endpoint measurement '
           '(mechanical/elastic) or the forwarded period-closing values',
           path=('energy', 'measured')),
        _f('source_interval_id', 'bigint', 'n/a', 'interval', True,
           'declared', _SRC_CLOSE,
           'interval the forwarded measurement was taken on; equals '
           'interval_id when measured', path=('energy', 'source_interval_id')),
        _f('source_time_s', 'double', 's', 'interval', True, 'declared',
           _SRC_CLOSE,
           'start time of the interval the forwarded measurement belongs to',
           path=('energy', 'source_time_s')),
        _f('component_work_j', 'map<double>', 'J', 'interval', True,
           'derived', _SRC_CLOSE,
           'per-component work qfrc·qvel·dt of this interval, keyed by '
           'force component', path=('component_work_j',),
           aliases=('{component}_work_j',)),
    )
    return TableSpec('channel_energy', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'energy ledger, work accumulators and residual channels',
                     _SRC_CLOSE,
                     claims=frozenset({'energy', 'component_work_j'}))


def _sensors_table() -> TableSpec:
    s = _SRC_OBSERVE + '::sensor_channels'
    fields = (
        _f('frame_specific_force_body_mps2', 'double[]', 'm/s²',
           'interval_start', False, 'measured', s,
           'frame accelerometer specific force (proper acceleration) in the '
           'body frame, evaluated on the incoming state before the endpoint '
           'forward pass; gravity is not subtracted a second time',
           frame='body frame', shape=(3,), axes=('x', 'y', 'z'),
           path=('sensors', 'frame_specific_force_body_mps2')),
        _f('frame_gyro_body_rad_s', 'double[]', 'rad/s', 'interval_start',
           False, 'measured', s,
           'frame gyro angular rate in the body frame at the incoming state',
           frame='body frame', shape=(3,), axes=('x', 'y', 'z'),
           path=('sensors', 'frame_gyro_body_rad_s')),
        _f('encoder_front_wheel_rad_s', 'double', 'rad/s', 'interval_start',
           False, 'measured', s,
           'front wheel spin encoder rate from the incoming qvel',
           frame='front wheel spin joint',
           path=('sensors', 'encoders_rad_s', 'front_wheel')),
        _f('encoder_rear_wheel_rad_s', 'double', 'rad/s', 'interval_start',
           False, 'measured', s,
           'rear wheel spin encoder rate from the incoming qvel',
           frame='rear wheel spin joint',
           path=('sensors', 'encoders_rad_s', 'rear_wheel')),
        _f('encoder_crank_rad_s', 'double', 'rad/s', 'interval_start', False,
           'measured', s,
           'crank spin encoder rate from the incoming qvel',
           frame='crank shaft', path=('sensors', 'encoders_rad_s', 'crank')),
        _f('motor_torque_nm', 'double', 'N·m', 'interval_start', False,
           'measured', s,
           'motor torque sensed at the motor/mid-drive shaft, not the rear '
           'wheel', frame='mid-drive shaft', path=('sensors', 'motor_torque_nm')),
        _f('human_torque_nm', 'double', 'N·m', 'interval_start', False,
           'measured', s,
           'human torque sensed at the crank shaft (crank-side torque; '
           'wheel-side torque differs by the transmission ratio)',
           frame='crank shaft', path=('sensors', 'human_torque_nm')),
    )
    return TableSpec('channel_sensors', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'physical sensor channels: specific force, gyro, '
                     'encoders and shaft torque sensors', s,
                     claims=frozenset({'sensors'}))


def _tire_side() -> tuple:
    t = _SRC_OBSERVE + '::tire_channels'
    patch = (
        _f('point_m', 'double[]', 'm', 'interval', False, 'measured', t,
           'contact point position', shape=(3,), axes=('x', 'y', 'z')),
        _f('normal', 'double[]', '1', 'interval', False, 'measured', t,
           'contact unit normal (X-Z plane)', shape=(3,), axes=('x', 'y', 'z')),
        _f('normal_load_n', 'double', 'N', 'interval', False, 'measured', t,
           'normal load carried by this patch'),
        _f('tangent_force_n', 'double', 'N', 'interval', False, 'measured', t,
           'signed tangential (rolling/slip) force of this patch'),
        _f('slip_mps', 'double', 'm/s', 'interval', False, 'measured', t,
           'slip speed of this patch'),
        _f('world_force_n', 'double[]', 'N', 'interval', False, 'measured', t,
           'patch contact force in world coordinates',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('couple_world_nm', 'double[]', 'N·m', 'interval', True, 'measured',
           t, 'patch contact couple in world coordinates',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('source_geom', 'varchar', 'n/a', 'interval', False, 'declared', t,
           'physical contact source (terrain | catch_plane)'),
    )
    children = (
        _f('backend', 'varchar', 'n/a', 'interval', False, 'declared', t,
           'tire force backend that produced this snapshot'),
        _f('unloaded_radius_m', 'double', 'm', 'interval', False, 'declared',
           t, 'undeformed wheel radius'),
        _f('supports_multiple_contacts', 'boolean', 'n/a', 'interval', False,
           'declared', t, 'backend capability flag'),
        _f('outside_material_load_range', 'boolean', 'n/a', 'interval', False,
           'measured', t,
           'material model is extrapolating beyond its measured load range'),
        _f('outside_material_deflection_range', 'boolean', 'n/a', 'interval',
           True, 'measured', t,
           'deflection outside the material calibration range (distributed '
           'backend)'),
        _f('outside_profile_domain', 'boolean', 'n/a', 'interval', True,
           'measured', t, 'contact outside the profile domain'),
        _f('patches', 'struct[]', 'n/a', 'interval', False, 'measured', t,
           'individual contact patches with pointwise loads',
           children=patch),
        _f('geometric_contact', 'boolean', 'n/a', 'interval', False,
           'measured', t, 'wheel penetrates the surface geometrically'),
        _f('raw_contact', 'boolean', 'n/a', 'interval', False, 'measured', t,
           'working-road contact observed before controller filtering'),
        _f('controller_grounded', 'boolean', 'n/a', 'interval', False,
           'measured', t,
           'debounced grounded verdict the controllers see'),
        _f('multi_support', 'boolean', 'n/a', 'interval', False, 'measured',
           t, 'more than one support patch carries load'),
        _f('normal_load_n', 'double', 'N', 'interval', False, 'measured', t,
           'total normal load over all patches'),
        _f('world_force_n', 'double[]', 'N', 'interval', False, 'measured',
           t, 'resultant contact force in world coordinates',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('vertical_force_n', 'double', 'N', 'interval', False, 'measured',
           t, 'world-vertical component of the resultant contact force'),
        _f('normal_vertical_n', 'double', 'N', 'interval', False, 'measured',
           t, 'load projected on the world normal (support load)'),
        _f('tangent_force_n', 'double', 'N', 'interval', False, 'measured',
           t, 'resultant signed tangential force'),
        _f('effective_radius_m', 'double', 'm', 'interval', False, 'derived',
           t, 'load-weighted effective rolling radius'),
        _f('wheel_axis_m', 'double[]', 'm', 'interval', False, 'measured', t,
           'wheel spin axis point in world coordinates',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('wheel_axis_moment_nm', 'double', 'N·m', 'interval', False,
           'derived', t, 'contact moment about the wheel spin axis'),
        _f('omega_abs_rad_s', 'double', 'rad/s', 'interval_start', False,
           'measured', t, 'absolute wheel angular rate about its axis'),
        _f('omega_rel_rad_s', 'double', 'rad/s', 'interval_start', False,
           'measured', t, 'wheel spin joint rate relative to its carrier'),
        _f('slip_mps', 'double', 'm/s', 'interval', False, 'derived', t,
           'load-weighted patch slip speed'),
        _f('slip_ratio', 'double', '1', 'interval', False, 'derived', t,
           'dimensionless slip normalized by roll speed (floor 0.1 m/s)'),
        _f('tangent_center_speed_mps', 'double', 'm/s', 'interval_start',
           False, 'derived', t,
           'load-weighted rolling speed along the contact tangent'),
        _f('penetration_m', 'double', 'm', 'interval', True, 'measured', t,
           'signed radial overlap of the compliant tire model'),
        _f('normal_speed_mps', 'double', 'm/s', 'interval_start', True,
           'measured', t, 'approach speed along the contact normal'),
        _f('friction_coefficient', 'double', '1', 'interval', True,
           'measured', t, 'effective friction coefficient of the material'),
        _f('surface', 'varchar', 'n/a', 'interval', True, 'declared', t,
           'surface material identifier'),
        _f('branch_release_loss_j', 'double', 'J', 'interval', True,
           'derived', t, 'elastic energy released by a contact-branch '
           'transition this interval'),
        _f('brush_loss_j', 'double', 'J', 'interval', True, 'derived', t,
           'tire brush (tangential) dissipation of this interval'),
        _f('radial_energy_j', 'double', 'J', 'interval', True, 'derived', t,
           'stored radial contact energy'),
        _f('shear_energy_j', 'double', 'J', 'interval', True, 'derived', t,
           'stored tangential contact energy'),
        _f('capability_status', 'varchar', 'n/a', 'interval', True,
           'declared', t, 'backend qualification status'),
        _f('calibration_status', 'varchar', 'n/a', 'interval', True,
           'declared', t, 'material calibration status'),
        _f('station_count', 'bigint', 'n/a', 'interval', True, 'declared',
           t, 'number of circumference stations (distributed backend)'),
        _f('active_station_ids', 'bigint[]', 'n/a', 'interval', True,
           'measured', t, 'stations in geometric contact this interval'),
        _f('road_segment_ids', 'bigint[]', 'n/a', 'interval', True,
           'measured', t, 'road segments under the contact'),
        _f('road_projection_m', 'double[]', 'm', 'interval', True,
           'measured', t, 'wheel centre projected on the road',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('nonsmooth_station_count', 'bigint', 'n/a', 'interval', True,
           'measured', t, 'stations over nonsmooth road edges'),
    )
    return children


def _tires_table() -> TableSpec:
    children = _tire_side()
    t = _SRC_OBSERVE + '::tire_channels'
    fields = (
        _f('front', 'struct', 'n/a', 'interval', False, 'measured', t,
           'front wheel contact/kinematic channels at the incoming state',
           children=children, path=('tires', 'front')),
        _f('rear', 'struct', 'n/a', 'interval', False, 'measured', t,
           'rear wheel contact/kinematic channels at the incoming state',
           children=children, path=('tires', 'rear')),
    )
    return TableSpec('channel_tires', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'per-wheel tire contact and rolling channels',
                     t, claims=frozenset({'tires'}))


def _suspension_table() -> TableSpec:
    s = _SRC_ADVANCE
    fields = (
        _f('fork_travel_m', 'double', 'm', 'interval_start', False,
           'measured', s, 'fork compression travel at the incoming state',
           path=('suspension', 'fork_travel_m')),
        _f('shock_stroke_m', 'double', 'm', 'interval_start', False,
           'measured', s, 'rear shock stroke at the incoming state',
           path=('suspension', 'shock_stroke_m')),
        _f('fork_velocity_mps', 'double', 'm/s', 'interval_start', False,
           'measured', s, 'fork slide rate at the incoming state',
           path=('suspension', 'fork_velocity_mps')),
        _f('shock_velocity_mps', 'double', 'm/s', 'interval_start', False,
           'measured', s, 'shock stroke rate at the incoming state',
           path=('suspension', 'shock_velocity_mps')),
        _f('shock_solver_limit_power_w', 'double', 'W', 'interval', False,
           'solved', s,
           'power of the solved shock joint-limit constraint force',
           path=('suspension', 'shock_solver_limit_power_w')),
        _f('linkage_closure_max_m', 'double', 'm', 'interval_end', False,
           'solved', s,
           'maximum connect-equality positional residual of the suspension '
           'linkage (metre-denominated rows only)',
           path=('suspension', 'linkage_closure_max_m')),
        _f('generalized_force_components_n', 'map<double>', 'per_coordinate',
           'interval', False, 'solved', s,
           'suspension-related generalized-force components evaluated at '
           'their slide DOF (newtons)', path=('suspension',
                                              'generalized_force_components_n')),
    )
    return TableSpec('channel_suspension', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'fork/shock travel, rates and limit diagnostics',
                     s, claims=frozenset({'suspension'}))


def _drive_table() -> TableSpec:
    d = 'src/bike_sim/sim/ride/drivetrain_forces.py::DrivetrainForceApplier'
    dl = d + '._compute_components'
    ds = d + '.settle_actuation'
    fields = (
        _f('transmission_model', 'varchar', 'n/a', 'n/a', False, 'declared',
           dl, 'configured transmission model name',
           path=('drive', 'transmission_model')),
        _f('omits_suspension_coupling', 'boolean', 'n/a', 'n/a', False,
           'declared', dl,
           'ideal mid-drive model decouples the drivetrain from suspension',
           path=('drive', 'omits_suspension_coupling')),
        _f('chain_extension_m', 'double', 'm', 'interval_start', False,
           'measured', dl, 'chain stretch at the incoming state',
           path=('drive', 'chain_extension_m')),
        _f('chain_extension_rate_mps', 'double', 'm/s', 'interval_start',
           False, 'measured', dl, 'chain stretch rate',
           path=('drive', 'chain_extension_rate_mps')),
        _f('chain_tension_n', 'double', 'N', 'interval', False, 'measured',
           dl, 'chain tension at the incoming state',
           path=('drive', 'chain_tension_n')),
        _f('chain_energy_j', 'double', 'J', 'interval', False, 'derived', dl,
           'elastic chain energy', path=('drive', 'chain_energy_j')),
        _f('chain_dissipation_power_w', 'double', 'W', 'interval', False,
           'derived', dl, 'chain damping dissipation rate',
           path=('drive', 'chain_dissipation_power_w')),
        _f('chain_power_w', 'double', 'W', 'interval', False, 'derived',
           _SRC_ADVANCE, 'chain component interval power',
           path=('drive', 'chain_power_w')),
        _f('freehub_torque_nm', 'double', 'N·m', 'interval', False,
           'measured', dl, 'freehub transmitted torque',
           path=('drive', 'freehub_torque_nm')),
        _f('freehub_energy_j', 'double', 'J', 'interval', False, 'derived',
           dl, 'freehub spring energy', path=('drive', 'freehub_energy_j')),
        _f('freehub_engaged', 'boolean', 'n/a', 'interval', False, 'solved',
           ds, 'freehub carrying torque', path=('drive', 'freehub_engaged')),
        _f('freehub_deflection_rad', 'double', 'rad', 'interval_start',
           False, 'measured', dl, 'freehub spring wind-up',
           path=('drive', 'freehub_deflection_rad')),
        _f('freehub_dissipation_power_w', 'double', 'W', 'interval', False,
           'derived', dl, 'freehub overrun dissipation',
           path=('drive', 'freehub_dissipation_power_w')),
        _f('freehub_power_w', 'double', 'W', 'interval', False, 'derived',
           _SRC_ADVANCE,
           'freehub plus ideal-transmission interval power',
           path=('drive', 'freehub_power_w')),
        _f('cadence_rpm', 'double', 'rpm', 'interval_start', False,
           'measured', dl, 'crank cadence at the incoming state',
           path=('drive', 'cadence_rpm')),
        _f('crank_rad_s', 'double', 'rad/s', 'interval_start', False,
           'measured', dl, 'crank shaft angular rate',
           path=('drive', 'crank_rad_s'), frame='crank shaft'),
        _f('drive_shaft_rad_s', 'double', 'rad/s', 'interval_start', False,
           'measured', dl, 'motor/mid-drive shaft angular rate',
           path=('drive', 'drive_shaft_rad_s'), frame='mid-drive shaft'),
        _f('crank_phase_rad', 'double', 'rad', 'interval_start', False,
           'measured', _SRC_ADVANCE, 'crank angle at the incoming state',
           path=('drive', 'crank_phase_rad'), frame='crank shaft'),
        _f('human_torque_nm', 'double', 'N·m', 'interval', False, 'measured',
           dl,
           'human torque applied at the crank shaft (crank-side; wheel-side '
           'torque differs by the transmission ratio)',
           path=('drive', 'human_torque_nm'), frame='crank shaft'),
        _f('human_sensor_nm', 'double', 'N·m', 'interval_start', False,
           'measured', dl,
           'pedal torque as the crank-shaft sensor reports it (crank-side, '
           'not wheel-side torque); solved weld reaction under an '
           'articulated rider', path=('drive', 'human_sensor_nm'),
           frame='crank shaft'),
        _f('human_setpoint_nm', 'double', 'N·m', 'control_period', True,
           'declared', dl,
           'requested human crank-torque target of the applied control',
           path=('drive', 'human_setpoint_nm'), frame='crank shaft'),
        _f('human_command_nm', 'double', 'N·m', 'control_period', False,
           'declared', dl,
           'pedaling-effort mean request handed to the rider/drive policy',
           path=('drive', 'human_command_nm'), frame='crank shaft'),
        _f('human_active_power_w', 'double', 'W', 'interval', False,
           'derived', _SRC_ADVANCE,
           'sum of rider actuator interval powers',
           path=('drive', 'human_active_power_w')),
        _f('human_joint_power_w', 'double', 'W', 'interval', False,
           'derived', _SRC_ADVANCE,
           'rider actuator plus passive tissue damping interval power',
           path=('drive', 'human_joint_power_w')),
        _f('assist_demand_gated', 'boolean', 'n/a', 'interval', False,
           'diagnostic', dl,
           'assist request gated by braking/torque/cadence thresholds',
           path=('drive', 'assist_demand_gated')),
        _f('assist_sensor_nm', 'double', 'N·m', 'interval_start', False,
           'measured', dl,
           'crank torque the assist controller uses as input',
           path=('drive', 'assist_sensor_nm'), frame='crank shaft'),
        _f('assist_mode', 'varchar', 'n/a', 'interval', False, 'diagnostic',
           dl, 'assist controller mode/state name',
           path=('drive', 'assist_mode')),
        _f('assist_gain', 'double', '1', 'interval', True, 'diagnostic', dl,
           'dimensionless assist gain applied to the sensor torque',
           path=('drive', 'assist_gain')),
        _f('rider_mode', 'varchar', 'n/a', 'control_period', False,
           'diagnostic', dl,
           'pedaling state machine mode (e.g. pedaling, coasting)',
           path=('drive', 'rider_mode')),
        _f('coasting_reason', 'varchar', 'n/a', 'control_period', True,
           'diagnostic', dl,
           'why the pedaling policy is coasting; NULL while pedaling',
           path=('drive', 'coasting_reason')),
        _f('required_cadence_rpm', 'double', 'rpm', 'interval_start', False,
           'measured', dl,
           'cadence the wheel speed would require through the current gear',
           path=('drive', 'required_cadence_rpm')),
        _f('crank_target_phase_rad', 'double', 'rad', 'control_period', True,
           'declared', dl,
           'pedaling policy crank phase target (when phase tracking)',
           path=('drive', 'crank_target_phase_rad'), frame='crank shaft'),
        _f('crank_target_rate_rad_s', 'double', 'rad/s', 'control_period',
           True, 'declared', dl,
           'requested crank rate target (control or policy)',
           path=('drive', 'crank_target_rate_rad_s'), frame='crank shaft'),
        _f('motor_request_nm', 'double', 'N·m', 'interval', False,
           'diagnostic', dl,
           'assist policy torque request before ceilings and energy '
           'limiting', path=('drive', 'motor_request_nm'),
           frame='mid-drive shaft'),
        _f('motor_setpoint_nm', 'double', 'N·m', 'control_period', True,
           'declared', dl,
           'requested motor torque target of the applied control '
           '(external request; NULL selects configured assistance)',
           path=('drive', 'motor_setpoint_nm'), frame='mid-drive shaft'),
        _f('motor_limit_nm', 'double', 'N·m', 'control_period', True,
           'declared', dl,
           'immediate safety ceiling on motor torque (applied after the lag '
           'and before energy limiting); a ceiling caps the request, it is '
           'not the requested target itself',
           path=('drive', 'motor_limit_nm'), frame='mid-drive shaft'),
        _f('motor_limited_request_nm', 'double', 'N·m', 'interval', False,
           'diagnostic', dl,
           'request after the safety ceiling (safety_request)',
           path=('drive', 'motor_limited_request_nm'),
           frame='mid-drive shaft'),
        _f('motor_torque_nm', 'double', 'N·m', 'interval', False, 'solved',
           ds, 'motor torque actually delivered at the mid-drive shaft',
           path=('drive', 'motor_torque_nm'), frame='mid-drive shaft'),
        _f('motor_control_source', 'varchar', 'n/a', 'interval', False,
           'declared', dl,
           'assist | external_request: which demand produced the request',
           path=('drive', 'motor_control_source')),
        _f('safety_limited', 'boolean', 'n/a', 'interval', False,
           'diagnostic', dl,
           'the ceiling clamped this interval request',
           path=('drive', 'safety_limited')),
        _f('energy_limited', 'boolean', 'n/a', 'interval', False,
           'diagnostic', dl,
           'the battery power budget clamped this interval request',
           path=('drive', 'energy_limited')),
        _f('motor_enabled', 'boolean', 'n/a', 'interval', False, 'solved',
           ds, 'motor delivered nonzero torque this interval',
           path=('drive', 'motor_enabled')),
        _f('battery_empty', 'boolean', 'n/a', 'interval', False, 'solved',
           ds, 'battery energy exhausted', path=('drive', 'battery_empty')),
        _f('battery_energy_j', 'double', 'J', 'interval', True, 'solved',
           ds, 'remaining battery energy (battery-enabled drivetrains)',
           path=('drive', 'battery_energy_j')),
        _f('motor_shaft_power_w', 'double', 'W', 'interval', True,
           'derived', dl, 'mechanical shaft power of the motor',
           path=('drive', 'motor_shaft_power_w'), frame='mid-drive shaft'),
        _f('electrical_power_w', 'double', 'W', 'interval', True, 'solved',
           ds, 'settled electrical draw including losses',
           path=('drive', 'electrical_power_w')),
        _f('motor_freewheel_engaged', 'boolean', 'n/a', 'interval', False,
           'solved', ds, 'motor-side freewheel carrying torque',
           path=('drive', 'motor_freewheel_engaged')),
        _f('motor_freewheel_torque_nm', 'double', 'N·m', 'interval', False,
           'solved', ds, 'motor-side freewheel transmitted torque',
           path=('drive', 'motor_freewheel_torque_nm')),
        _f('motor_freewheel_dissipation_power_w', 'double', 'W', 'interval',
           False, 'derived', ds, 'motor freewheel overrun dissipation',
           path=('drive', 'motor_freewheel_dissipation_power_w')),
        _f('crank_clutch_torque_nm', 'double', 'N·m', 'interval', True,
           'solved', ds, 'crank clutch transmitted torque (when fitted)',
           path=('drive', 'crank_clutch_torque_nm')),
        _f('crank_clutch_engaged', 'boolean', 'n/a', 'interval', True,
           'solved', ds, 'crank clutch carrying torque',
           path=('drive', 'crank_clutch_engaged')),
        _f('crank_clutch_dissipation_power_w', 'double', 'W', 'interval',
           True, 'derived', ds, 'crank clutch catch dissipation',
           path=('drive', 'crank_clutch_dissipation_power_w')),
        _f('gear_front_teeth', 'bigint', 'n/a', 'n/a', False, 'declared',
           d + '._shift_diagnostics', 'chainring tooth count',
           path=('drive', 'gear_front_teeth')),
        _f('gear_rear_teeth', 'bigint', 'n/a', 'interval', False,
           'measured', d + '._shift_diagnostics', 'current sprocket teeth',
           path=('drive', 'gear_rear_teeth')),
        _f('gear_ratio', 'double', '1', 'interval', False, 'derived',
           d + '._shift_diagnostics', 'front/rear tooth ratio',
           path=('drive', 'gear_ratio')),
        _f('shift_active', 'boolean', 'n/a', 'interval', False,
           'diagnostic', d + '._shift_diagnostics',
           'a torque-cut shift is in progress', path=('drive', 'shift_active')),
        _f('shift_direction', 'varchar', 'n/a', 'interval', False,
           'diagnostic', d + '._shift_diagnostics',
           'up/down/none shift direction', path=('drive', 'shift_direction')),
        _f('shift_from_teeth', 'bigint', 'n/a', 'interval', False,
           'diagnostic', d + '._shift_diagnostics',
           'sprocket teeth before the in-progress shift',
           path=('drive', 'shift_from_teeth')),
        _f('shift_count', 'bigint', 'n/a', 'episode', False, 'diagnostic',
           d + '._shift_diagnostics', 'completed shifts this episode',
           path=('drive', 'shift_count')),
        _f('shift_time_s', 'double', 's', 'interval', True, 'diagnostic',
           d + '._shift_diagnostics', 'time of the latest shift',
           path=('drive', 'shift_time_s')),
        _f('shift_torque_factor', 'double', '1', 'interval', False,
           'diagnostic', d + '._shift_diagnostics',
           'torque scale during a shift cut', path=('drive', 'shift_torque_factor')),
        _f('front_brake_demand', 'double', '1', 'interval', False,
           'declared', _SRC_ADVANCE,
           'effective front brake demand applied this interval (includes a '
           'rollback hold)', path=('drive', 'front_brake_demand')),
        _f('rear_brake_demand', 'double', '1', 'interval', False,
           'declared', _SRC_ADVANCE,
           'effective rear brake demand applied this interval',
           path=('drive', 'rear_brake_demand')),
        _f('rollback_brake_demand', 'double', '1', 'interval', False,
           'declared', _SRC_RUNTIME + '._rollback_brake_demand',
           'hill-hold reflex contribution inside the effective demands',
           path=('drive', 'rollback_brake_demand')),
        _f('front_brake_power_w', 'double', 'W', 'interval', False,
           'solved', _SRC_ADVANCE, 'front static-brake interval power',
           path=('drive', 'front_brake_power_w')),
        _f('rear_brake_power_w', 'double', 'W', 'interval', False, 'solved',
           _SRC_ADVANCE, 'rear static-brake interval power',
           path=('drive', 'rear_brake_power_w')),
        _f('front_brake_torque_nm', 'double', 'N·m', 'interval', False,
           'solved', _SRC_ADVANCE,
           'front static-brake torque at the wheel spin DOF',
           path=('drive', 'front_brake_torque_nm'),
           frame='front wheel spin joint'),
        _f('rear_brake_torque_nm', 'double', 'N·m', 'interval', False,
           'solved', _SRC_ADVANCE,
           'rear static-brake torque at the wheel spin DOF',
           path=('drive', 'rear_brake_torque_nm'),
           frame='rear wheel spin joint'),
        _f('transmission_phi_m', 'double', 'm', 'interval', True, 'solved',
           ds, 'geometric transmission coordinate (geometric freehub)',
           path=('drive', 'transmission_phi_m')),
        _f('transmission_boundary_m', 'double', 'm', 'interval', True,
           'declared', ds, 'wrap boundary of the geometric freehub',
           path=('drive', 'transmission_boundary_m')),
        _f('transmission_gap_m', 'double', 'm', 'interval', True, 'solved',
           ds, 'distance to the wrap boundary',
           path=('drive', 'transmission_gap_m')),
        _f('transmission_tension_n', 'double', 'N', 'interval', True,
           'solved', ds, 'geometric transmission belt tension',
           path=('drive', 'transmission_tension_n')),
        _f('transmission_constraint_defect_m', 'double', 'm', 'interval',
           True, 'solved', ds, 'wrap constraint residual',
           path=('drive', 'transmission_constraint_defect_m')),
        _f('transmission_interval_work_j', 'double', 'J', 'interval', True,
           'solved', ds, 'wrap constraint signed work of this interval',
           path=('drive', 'transmission_interval_work_j')),
        _f('transmission_reaction_error_n', 'double', 'N', 'interval', True,
           'solved', ds, 'geometric reaction consistency error',
           path=('drive', 'transmission_reaction_error_n')),
        _f('transmission_reference_status', 'varchar', 'n/a', 'interval',
           True, 'declared', ds, 'geometric backend qualification marker',
           path=('drive', 'transmission_reference_status')),
        _f('shift_parameter_work_j', 'double', 'J', 'episode', True,
           'solved', ds, 'cumulative work attributed to shifting geometry',
           path=('drive', 'shift_parameter_work_j')),
        _f('shift_interval_constraint_work_j', 'double', 'J', 'interval',
           True, 'solved', ds, 'shift-step constraint work',
           path=('drive', 'shift_interval_constraint_work_j')),
        _f('shift_constraint_work_cumulative_j', 'double', 'J', 'episode',
           True, 'solved', ds, 'cumulative shift constraint work',
           path=('drive', 'shift_constraint_work_cumulative_j')),
    )
    return TableSpec('channel_drive', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'drivetrain, pedaling, assist, braking and shifting '
                     'channels', d, claims=frozenset({'drive'}))


def _posture_children() -> tuple:
    p = 'src/bike_sim/physics/rider_posture.py::RiderPosture'
    return (
        _f('torso_lean_rad', 'double', 'rad', 'control_period', False,
           'declared', p, 'forward torso lean request'),
        _f('pelvis_pitch_rad', 'double', 'rad', 'control_period', False,
           'declared', p, 'pelvis pitch request'),
        _f('pelvis_offset_m', 'double[]', 'm', 'control_period', True,
           'declared', p,
           'optional (forward, upward) hip offset; NULL keeps the '
           'support-following IK', shape=(2,), axes=('forward', 'up')),
        _f('use_saddle', 'boolean', 'n/a', 'control_period', False,
           'declared', p, 'requested saddle support (not the contact itself)'),
    )


def _control_table() -> TableSpec:
    c = 'src/bike_sim/sim/ride/control.py::RideControl'
    fields = (
        _f('motor_torque_nm', 'double', 'N·m', 'control_period', True,
           'declared', c,
           'requested motor torque target; NULL selects configured '
           'assistance', path=('control', 'motor_torque_nm'),
           frame='mid-drive shaft'),
        _f('motor_limit_nm', 'double', 'N·m', 'control_period', True,
           'declared', c,
           'requested immediate safety ceiling on motor torque (ceiling, '
           'not a target); NULL applies the default bound',
           path=('control', 'motor_limit_nm'), frame='mid-drive shaft'),
        _f('human_torque_nm', 'double', 'N·m', 'control_period', True,
           'declared', c,
           'requested human crank-torque target (crank-side torque)',
           path=('control', 'human_torque_nm'), frame='crank shaft'),
        _f('crank_target_rate_rad_s', 'double', 'rad/s', 'control_period',
           True, 'declared', c, 'requested crank rate target',
           path=('control', 'crank_target_rate_rad_s'),
           frame='crank shaft'),
        _f('posture', 'struct', 'n/a', 'control_period', True, 'declared', c,
           'requested rider posture offsets', children=_posture_children(),
           path=('control', 'posture')),
        _f('rider_enabled', 'boolean', 'n/a', 'control_period', False,
           'declared', c, 'rider torques enabled for this interval',
           path=('control', 'rider_enabled')),
    )
    return TableSpec('channel_control', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'applied control record effective over the interval',
                     _SRC_ADVANCE, claims=frozenset({'control'}))


def _model_status_table() -> TableSpec:
    m = 'src/bike_sim/sim/ride/model_status.py::ModelStatus'
    fields = (
        _f('model_status', 'struct', 'n/a', 'interval', False, 'diagnostic',
           m + '.as_dict',
           'episode-local applicability and numerical-quality status',
           children=(
               _f('model_valid', 'boolean', 'n/a', 'interval', False,
                  'diagnostic', m, 'no recorded model violations'),
               _f('first_model_violation', 'struct', 'n/a', 'interval', True,
                  'diagnostic', m, 'first recorded violation (sticky)',
                  children=(
                      _f('interval_id', 'bigint', 'n/a', 'interval', False,
                         'diagnostic', m, 'violating interval'),
                      _f('time_s', 'double', 's', 'interval', False,
                         'diagnostic', m, 'violation time'),
                      _f('reasons', 'varchar[]', 'n/a', 'interval', False,
                         'diagnostic', m, 'violation reason list'),
                  )),
               _f('model_violation_counts', 'map<double>', 'n/a', 'episode',
                  False, 'diagnostic', m, 'violation reason -> count'),
               _f('numerically_valid', 'varchar', 'n/a', 'interval', False,
                  'diagnostic', m,
                  "tri-state numeric verdict: 'true' | 'false' | "
                  "'not_evaluated' (a string in the source, kept verbatim)"),
               _f('calibration_status', 'varchar', 'n/a', 'interval', False,
                  'declared', m, 'model calibration status string'),
           ), path=('model_status',)),
        _f('contact_crash_cause', 'varchar', 'n/a', 'interval', True,
           'measured', _SRC_ADVANCE,
           'physical crash cause classified from contacts; NULL while no '
           'crash is observed', path=('contact_crash_cause',)),
        _f('attachment_violations', 'varchar[]', 'n/a', 'interval', True,
           'diagnostic', _SRC_CLOSE,
           'attachment-budget violation tags recorded for this interval',
           path=('attachment_violations',)),
    )
    return TableSpec('channel_model_status', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'model applicability, numerical quality and violation '
                     'channels', m,
                     claims=frozenset({'model_status', 'contact_crash_cause',
                                       'attachment_violations'}))


def _rider_table() -> TableSpec:
    rc = 'src/bike_sim/sim/ride/rider_contacts.py::RiderContactApplier'
    rctl = 'src/bike_sim/sim/ride/rider_control.py::ArticulatedRiderController'
    attachment = (
        _f('enabled', 'boolean', 'n/a', 'interval', True, 'declared', rc,
           'support surface enabled'),
        _f('in_platform', 'boolean', 'n/a', 'interval', True, 'measured',
           rc, 'contact point inside the support platform'),
        _f('reachable', 'boolean', 'n/a', 'interval', True, 'measured', rc,
           'grip within reach'),
        _f('normal_load_n', 'double', 'N', 'interval', True, 'measured', rc,
           'normal load on the support'),
        _f('gap_m', 'double', 'm', 'interval', True, 'measured', rc,
           'distance to the support platform'),
        _f('hand_gap_m', 'double', 'm', 'interval', True, 'measured', rc,
           'hand-to-grip distance'),
        _f('would_separate', 'boolean', 'n/a', 'interval', True, 'solved',
           rc, 'weld reaction is tensile: a real pad could not hold'),
        _f('would_slip', 'boolean', 'n/a', 'interval', True, 'solved', rc,
           'shear beyond the pad friction cone'),
        _f('tangent_n', 'double', 'N', 'interval', True, 'solved', rc,
           'weld shear along the pad tangent'),
        _f('vertical_force_on_rider_n', 'double', 'N', 'interval', True,
           'solved', rc, 'world-vertical force on the rider'),
        _f('tangent_force_n', 'double[]', 'N', 'interval', True, 'solved',
           rc, 'tangential force vector', shape=(3,), axes=('x', 'y', 'z')),
        _f('force_on_rider_n', 'double[]', 'N', 'interval', True, 'solved',
           rc, 'support force applied to the rider',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('force_on_bike_n', 'double[]', 'N', 'interval', True, 'solved',
           rc, 'equal-opposite support force applied to the bike',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('moment_about_rider_origin_nm', 'double[]', 'N·m', 'interval',
           True, 'solved', rc, 'support moment about the rider origin',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('point_m', 'double[]', 'm', 'interval', True, 'measured', rc,
           'support contact point', shape=(3,), axes=('x', 'y', 'z')),
        _f('patches', 'struct[]', 'n/a', 'interval', True, 'measured', rc,
           'per-patch pad contact detail',
           children=(
               _f('in_platform', 'boolean', 'n/a', 'interval', False,
                  'measured', rc, 'patch inside the pad footprint'),
               _f('normal_load_n', 'double', 'N', 'interval', False,
                  'measured', rc, 'patch normal load'),
               _f('gap_m', 'double', 'm', 'interval', False, 'measured', rc,
                  'patch distance to the pad'),
               _f('point_m', 'double[]', 'm', 'interval', False, 'measured',
                  rc, 'patch centre', shape=(3,), axes=('x', 'y', 'z')),
               _f('force_on_rider_n', 'double[]', 'N', 'interval', False,
                  'solved', rc, 'patch force on the rider',
                  shape=(3,), axes=('x', 'y', 'z')),
               _f('radial_energy_j', 'double', 'J', 'interval', False,
                  'derived', rc, 'patch stored normal energy'),
               _f('shear_energy_j', 'double', 'J', 'interval', False,
                  'derived', rc, 'patch stored shear energy'),
           )),
        _f('radial_energy_j', 'double', 'J', 'interval', True, 'derived',
           rc, 'stored normal energy'),
        _f('shear_energy_j', 'double', 'J', 'interval', True, 'derived', rc,
           'stored shear energy'),
        _f('elastic_energy_j', 'double', 'J', 'interval', True, 'derived',
           rc, 'stored elastic energy (grip variant)'),
        _f('relative_power_w', 'double', 'W', 'interval', True, 'derived',
           rc, 'support dissipation rate'),
        _f('overloaded', 'boolean', 'n/a', 'interval', True, 'diagnostic',
           rc, 'pair force beyond the grip limit'),
        _f('trial_pair_force_n', 'double', 'N', 'interval', True, 'solved',
           rc, 'summed grip pair force'),
        _f('pair_force_limit_n', 'double', 'N', 'n/a', True, 'declared', rc,
           'configured grip pair force limit'),
        _f('release_loss_j', 'double', 'J', 'interval', True, 'derived', rc,
           'energy released by a support release'),
        _f('shoulder_distance_m', 'double', 'm', 'interval', True,
           'measured', rc, 'shoulder-to-grip distance'),
        _f('arm_reach_m', 'double', 'm', 'n/a', True, 'declared', rc,
           'configured arm reach'),
    )
    weld = (
        _f('force_on_rider_n', 'double[]', 'N', 'interval', False, 'solved',
           rc + '.settle_welds', 'weld reaction on the rider',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('normal_n', 'double', 'N', 'interval', True, 'solved',
           rc + '.settle_welds',
           'weld reaction along the pad normal (negative = tensile)'),
        _f('tangent_n', 'double', 'N', 'interval', True, 'solved',
           rc + '.settle_welds', 'weld shear along the pad tangent'),
        _f('would_separate', 'boolean', 'n/a', 'interval', True, 'solved',
           rc + '.settle_welds', 'pad would lose contact'),
        _f('would_slip', 'boolean', 'n/a', 'interval', True, 'solved',
           rc + '.settle_welds', 'shear beyond the friction cone'),
    )
    terms = (
        _f('requested_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'controller torque request', ),
        _f('command_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'final commanded torque after limits'),
        _f('posture_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'posture-task torque share'),
        _f('pedaling_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'pedaling-task torque share'),
        _f('support_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'support-wrench torque share (legacy path)'),
        _f('tracking_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'tracking-inertia torque share (legacy path)'),
        _f('hip_posture_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'hip posture share (legacy path)'),
        _f('saturated', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'command clipped by capacity'),
        _f('solved_force_nm', 'double', 'N·m', 'interval', True, 'solved',
           rctl, 'solved actuator force at this joint'),
        _f('solved_active_nm', 'double', 'N·m', 'interval', True, 'solved',
           rctl, 'solved active muscle torque'),
        _f('solved_passive_nm', 'double', 'N·m', 'interval', True, 'solved',
           rctl, 'solved passive tissue damping torque'),
    )
    allocation = (
        _f('invalid_controller', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'allocation found no feasible command'),
        _f('feasible', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'allocation feasibility verdict'),
        _f('violation', 'double', '1', 'control_period', True, 'diagnostic',
           rctl, 'dimensionless infeasibility measure'),
        _f('crank_task_nm', 'double', 'N·m', 'control_period', True,
           'diagnostic', rctl, 'crank torque the allocation targeted',
           ),
        _f('crank_task_shortfall_nm', 'double', 'N·m', 'control_period',
           True, 'diagnostic', rctl, 'unmet crank-task torque'),
    )
    tracking = (
        _f('actual_phase_rad', 'double', 'rad', 'control_period', False,
           'measured', rctl, 'measured crank phase'),
        _f('target_phase_rad', 'double', 'rad', 'control_period', True,
           'declared', rctl, 'commanded crank phase target'),
        _f('phase_error_rad', 'double', 'rad', 'control_period', True,
           'derived', rctl, 'wrapped phase error'),
        _f('actual_rate_rad_s', 'double', 'rad/s', 'control_period', False,
           'measured', rctl, 'measured crank rate'),
        _f('target_rate_rad_s', 'double', 'rad/s', 'control_period', True,
           'declared', rctl, 'commanded crank rate target'),
    )
    goal_geometry = (
        _f('requested_compression_m', 'double', 'm', 'control_period', True,
           'diagnostic', rctl, 'sole-goal radial request'),
        _f('applied_compression_m', 'double', 'm', 'control_period', True,
           'diagnostic', rctl, 'sole-goal radial applied after projection'),
        _f('requested_shear_m', 'double', 'm', 'control_period', True,
           'diagnostic', rctl, 'sole-goal shear request'),
        _f('applied_shear_m', 'double', 'm', 'control_period', True,
           'diagnostic', rctl, 'sole-goal shear applied after projection'),
        _f('saturated', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'goal projection was limited'),
        _f('limiting_reasons', 'varchar[]', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'which bounds limited the goal'),
        _f('weld', 'boolean', 'n/a', 'control_period', True, 'diagnostic',
           rctl, 'goal came from a welded sole'),
        _f('recovery', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'goal came from pedal recovery'),
        _f('spindle', 'boolean', 'n/a', 'control_period', True, 'diagnostic',
           rctl, 'goal tracked the spindle'),
    )
    feet = (
        _f('actual_sole_position_m', 'double[]', 'm', 'control_period', True,
           'measured', rctl, 'measured sole site position',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('target_sole_position_m', 'double[]', 'm', 'control_period', True,
           'diagnostic', rctl, 'IK sole goal position',
           shape=(3,), axes=('x', 'y', 'z')),
        _f('ik_saturated', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'leg IK clipped at joint limits'),
        _f('recovery_stage', 'varchar', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'pedal recovery stage name'),
        _f('goal_geometry', 'struct', 'n/a', 'control_period', True,
           'diagnostic', rctl,
           'sole-goal projection diagnostics for this foot',
           children=goal_geometry),
    )
    stance = (
        _f('front', 'boolean', 'n/a', 'control_period', False, 'diagnostic',
           rctl, 'front foot is the stance leg'),
        _f('rear', 'boolean', 'n/a', 'control_period', False, 'diagnostic',
           rctl, 'rear foot is the stance leg'),
    )
    support_targets = (
        _f('stance', 'struct', 'n/a', 'control_period', True, 'diagnostic',
           rctl, 'per-side stance share', children=stance),
        _f('requested_vertical_forces_n', 'map<double>', 'N',
           'control_period', True, 'diagnostic', rctl,
           'vertical support force requests by support name'),
        _f('requested_pitch_moment_nm', 'double', 'N·m', 'control_period',
           True, 'diagnostic', rctl, 'requested pitch moment'),
        _f('feasible_pedal_force_on_bike_n', 'map<double[]>', 'N',
           'control_period', True, 'diagnostic', rctl,
           'feasible pedal force requests by side'),
        _f('posture', 'struct', 'n/a', 'control_period', True, 'declared',
           rctl, 'resolved posture command', children=_posture_children()),
        _f('coasting', 'boolean', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'coasting phase tracked'),
        _f('coasting_requested_crank_torque_nm', 'double', 'N·m',
           'control_period', True, 'diagnostic', rctl,
           'crank torque requested while coasting'),
        _f('crank_tracking', 'struct', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'crank phase/rate tracking terms',
           children=tracking),
        _f('feet', 'map<struct>', 'n/a', 'control_period', True,
           'diagnostic', rctl, 'per-foot sole targets and IK diagnostics',
           children=feet),
    )
    attachment_sample = (
        _f('kind', 'varchar', 'n/a', 'interval', False, 'declared',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'attachment kind (saddle, grip, pedal)'),
        _f('normal_n', 'double', 'N', 'interval', False, 'solved',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'wrench normal component'),
        _f('tangent_n', 'double', 'N', 'interval', False, 'solved',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'wrench tangent component'),
        _f('moment_nm', 'double', 'N·m', 'interval', False, 'solved',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'wrench moment'),
        _f('gap_m', 'double', 'm', 'interval', False, 'measured',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'measured gap to the support'),
        _f('pull_n', 'double', 'N', 'interval', False, 'solved',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'grip pull magnitude'),
        _f('half_patch_m', 'double', 'm', 'interval', False, 'declared',
           'src/bike_sim/physics/attachment_budget.py::AttachmentSample',
           'pad half extent for the budget check'),
    )
    balance = (
        _f('balance_lost', 'boolean', 'n/a', 'interval', False, 'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'low-speed balance loss is latched'),
        _f('low_speed_s', 'double', 's', 'episode', False, 'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'accumulated time below the balance floor'),
        _f('grace_left_s', 'double', 's', 'interval', False, 'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'grace period before balance loss is declared'),
        _f('balance_lost_at_m', 'double', 'm', 'episode', True, 'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'position where balance was lost'),
        _f('balance_lost_time_s', 'double', 's', 'episode', True,
           'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'time balance was lost'),
    )
    intent = (
        _f('posture', 'struct', 'n/a', 'control_period', True, 'declared',
           'src/bike_sim/physics/seated_climb.py::SeatedClimbIntent',
           'resolved posture intent', children=_posture_children()),
        _f('effort_ceiling_nm', 'double', 'N·m', 'control_period', True,
           'declared', 'src/bike_sim/physics/seated_climb.py::SeatedClimbIntent',
           'crank effort ceiling the policy resolved (ceiling on effort, '
           'not the target)', frame='crank shaft'),
        _f('inclination_rad', 'double', 'rad', 'control_period', True,
           'derived', _SRC_ADVANCE,
           'estimated road inclination behind the intent policy'),
    )
    fields = (
        _f('rider', 'map<struct>', 'n/a', 'interval', True, 'diagnostic',
           rc,
           'per-attachment contact diagnostics by support name '
           '(saddle/grips/pedals)', path=('rider',), children=attachment),
        _f('rider_welds', 'map<struct>', 'n/a', 'interval', True, 'solved',
           rc + '.settle_welds',
           'weld/connect reactions per support, solved at the incoming '
           'pose', path=('rider_welds',), children=weld,
           except_keys=('crank_torque_nm',)),
        _f('weld_crank_torque_nm', 'double', 'N·m', 'interval', True,
           'solved', rc + '.settle_welds',
           'crank torque delivered through the pedal welds (crank-side '
           'torque)', path=('rider_welds', 'crank_torque_nm'),
           frame='crank shaft'),
        _f('rider_control', 'map<struct>', 'n/a', 'control_period', True,
           'diagnostic', rctl,
           'per-joint controller term decomposition', path=('rider_control',),
           children=terms),
        _f('rider_allocation', 'struct', 'n/a', 'control_period', True,
           'diagnostic', rctl,
           'allocation feasibility and task diagnostics',
           path=('rider_allocation',), children=allocation),
        _f('rider_ik_saturation', 'map<boolean>', 'n/a', 'control_period',
           True, 'diagnostic', rctl,
           'IK saturation flags by limb group', path=('rider_ik_saturation',)),
        _f('rider_support_targets', 'struct', 'n/a', 'control_period', True,
           'diagnostic', rctl,
           'resolved support targets and tracking diagnostics',
           path=('rider_support_targets',), children=support_targets),
        _f('rider_intent', 'struct', 'n/a', 'control_period', True,
           'declared', _SRC_ADVANCE,
           'seated-climb resolved intent (posture + effort ceiling)',
           path=('rider_intent',), children=intent),
        _f('rider_joint_torques', 'map<double>', 'N·m', 'control_period',
           True, 'solved', _SRC_ADVANCE,
           'final joint torques by rider joint', path=('rider_joint_torques',)),
        _f('rider_joint_capacity_nm', 'map<double>', 'N·m', 'control_period',
           True, 'diagnostic', _SRC_ADVANCE,
           'directional torque capacity per rider joint',
           path=('rider_joint_capacity_nm',)),
        _f('rider_lean_limit_rad', 'double', 'rad', 'control_period', True,
           'declared', _SRC_ADVANCE,
           'active lean limit for the rider planner',
           path=('rider_lean_limit_rad',)),
        _f('rider_balance', 'struct', 'n/a', 'interval', True, 'diagnostic',
           'src/bike_sim/sim/ride/balance_monitor.py::BalanceMonitor.observation',
           'low-speed balance monitor observation',
           path=('rider_balance',), children=balance),
        _f('attachment_samples', 'map<struct>', 'n/a', 'interval', True,
           'solved', _SRC_CLOSE,
           'recovered attachment wrenches per support (period-close least '
           'squares)', path=('attachment_samples',),
           children=attachment_sample),
        _f('rider_active_request_nm', 'map<double>', 'N·m',
           'control_period', True, 'diagnostic', rctl,
           'per-joint activation-space torque request',
           path=('rider_active_request_nm',)),
        _f('rider_active_delivered_nm', 'map<double>', 'N·m', 'interval',
           True, 'solved', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'solved per-joint actuator torque', path=('rider_active_delivered_nm',)),
        _f('rider_joint_positive_power_w', 'map<double>', 'W', 'interval',
           True, 'derived', 'src/bike_sim/sim/ride/rider_effort.py::joint_effort_limits',
           'positive joint powers by joint', path=('rider_joint_positive_power_w',)),
        _f('rider_positive_power_w', 'double', 'W', 'interval', True,
           'derived', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'total positive muscle power', path=('rider_positive_power_w',)),
        _f('rider_passive_power_w', 'double', 'W', 'interval', True,
           'derived', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'passive tissue damping power', path=('rider_passive_power_w',)),
        _f('rider_positive_work_step_j', 'double', 'J', 'interval', True,
           'derived', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'positive muscle work of this interval',
           path=('rider_positive_work_step_j',)),
        _f('rider_passive_work_step_j', 'double', 'J', 'interval', True,
           'derived', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'passive damping work of this interval',
           path=('rider_passive_work_step_j',)),
        _f('rider_activation_saturated', 'boolean', 'n/a', 'control_period',
           True, 'diagnostic', rctl,
           'activation dynamics clipped this command',
           path=('rider_activation_saturated',)),
        _f('rider_strength_limited', 'varchar[]', 'n/a', 'interval', True,
           'diagnostic', rctl,
           'joints limited by the strength model',
           path=('rider_strength_limited',)),
        _f('rider_strength_violations', 'varchar[]', 'n/a', 'interval', True,
           'diagnostic', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'joints exceeding strength capacity',
           path=('rider_strength_violations',)),
        _f('rider_joint_power_violations', 'varchar[]', 'n/a', 'interval',
           True, 'diagnostic', 'src/bike_sim/sim/ride/rider_effort.py::joint_effort_limits',
           'joints exceeding the power limit',
           path=('rider_joint_power_violations',)),
        _f('rider_joint_speed_violations', 'varchar[]', 'n/a', 'interval',
           True, 'diagnostic', 'src/bike_sim/sim/ride/rider_effort.py::joint_effort_limits',
           'joints exceeding the speed limit',
           path=('rider_joint_speed_violations',)),
        _f('rider_effort_budget_exceeded', 'boolean', 'n/a', 'interval',
           True, 'diagnostic', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'positive power over the configured budget',
           path=('rider_effort_budget_exceeded',)),
        _f('rider_effort_observation', 'varchar', 'n/a', 'interval', True,
           'declared', 'src/bike_sim/sim/ride/period_buffer.py::_effort_observations',
           'effort measurement convention label',
           path=('rider_effort_observation',)),
    )
    return TableSpec('channel_rider', 'channels',
                     (_EPISODE_KEY, _INTERVAL_KEY), fields,
                     'articulated-rider contact, control and effort '
                     'channels (present only for articulated rider configs)',
                     rctl,
                     claims=frozenset({
                         'rider', 'rider_welds', 'rider_control',
                         'rider_allocation', 'rider_ik_saturation',
                         'rider_support_targets', 'rider_intent',
                         'rider_joint_torques', 'rider_joint_capacity_nm',
                         'rider_lean_limit_rad', 'rider_balance',
                         'attachment_samples', 'rider_active_request_nm',
                         'rider_active_delivered_nm',
                         'rider_joint_positive_power_w',
                         'rider_positive_power_w', 'rider_passive_power_w',
                         'rider_positive_work_step_j',
                         'rider_passive_work_step_j',
                         'rider_activation_saturated',
                         'rider_strength_limited',
                         'rider_strength_violations',
                         'rider_joint_power_violations',
                         'rider_joint_speed_violations',
                         'rider_effort_budget_exceeded',
                         'rider_effort_observation',
                     }))


_JOINT_LAYOUTS = {
    'free': (('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'),
             ('vx', 'vy', 'vz', 'wx', 'wy', 'wz'),
             ('translation',) * 3 + ('quaternion',) * 4,
             ('translation',) * 3 + ('rotation',) * 3),
    'ball': (('qw', 'qx', 'qy', 'qz'), ('wx', 'wy', 'wz'),
             ('quaternion',) * 4, ('rotation',) * 3),
    'hinge': (('',), ('',), ('rotation',), ('rotation',)),
    'slide': (('',), ('',), ('translation',), ('translation',)),
}
_QPOS_UNITS = {'translation': 'm', 'rotation': 'rad', 'quaternion': '1'}
_QVEL_UNITS = {'translation': 'm/s', 'rotation': 'rad/s'}


def _coordinate_catalog(joints) -> CoordinateCatalog:
    qpos, dof = [], []
    for joint in joints:
        if not isinstance(joint, JointCoords):
            raise ValueError('joints must be JointCoords values')
        layout = _JOINT_LAYOUTS.get(joint.joint_type)
        if layout is None:
            raise ValueError(
                f'{joint.name}: unknown joint_type {joint.joint_type!r}')
        qmembers, dmembers, qkinds, dkinds = layout
        for offset, (member, kind) in enumerate(zip(qmembers, qkinds)):
            qpos.append(Coordinate(index=joint.qposadr + offset, kind='qpos',
                                   joint=joint.name, component=member,
                                   dof_kind=kind, unit=_QPOS_UNITS[kind]))
        for offset, (member, kind) in enumerate(zip(dmembers, dkinds)):
            dof.append(Coordinate(index=joint.dofadr + offset, kind='dof',
                                  joint=joint.name, component=member,
                                  dof_kind=kind,
                                  unit=_QVEL_UNITS.get(kind, 'n/a')))
    for sequence, label in ((qpos, 'qpos'), (dof, 'dof')):
        indexes = [c.index for c in sequence]
        if indexes != list(range(len(indexes))):
            raise ValueError(
                f'{label} addresses must be dense from 0; got {indexes}')
    return CoordinateCatalog(tuple(qpos), tuple(dof))


class Catalog:
    """The reviewed field catalog plus model coordinate and alias maps."""

    def __init__(self, tables, coordinates, aliases):
        self._tables = tuple(tables)
        self._by_name = {spec.name: spec for spec in self._tables}
        if len(self._by_name) != len(self._tables):
            raise ValueError('duplicate table names')
        self.coordinates = coordinates
        self.aliases = tuple(aliases)
        roots = {}
        for spec in self._tables:
            for root in spec.claims:
                if root in roots:
                    raise ValueError(
                        f'channel root {root!r} claimed by both '
                        f'{roots[root]} and {spec.name}')
                roots[root] = spec.name
        self._channel_roots = roots

    def table(self, name: str) -> TableSpec:
        try:
            return self._by_name[name]
        except KeyError:
            raise KeyError(f'unknown recording table {name!r}') from None

    def table_order(self) -> tuple:
        return self._tables

    def channel_roots(self) -> frozenset:
        return frozenset(self._channel_roots)

    def channel_table(self, root: str) -> TableSpec:
        return self.table(self._channel_roots[root])

    def resolve_alias(self, name: str) -> AliasSpec | None:
        """Resolve a legacy flat-column name to its canonical field."""
        for spec in self.aliases:
            if spec.alias == name:
                return spec
        for spec in self.aliases:  # parameterized templates
            if '{' not in spec.alias:
                continue
            pattern = '^' + re.escape(spec.alias).replace(
                re.escape('{component}'), '(.+)') + '$'
            match = re.match(pattern, name)
            if match:
                return AliasSpec(alias=name, canonical=spec.canonical,
                                 index=spec.index,
                                 parameter=match.group(1), note=spec.note)
        if '.' in name:  # flattened channel path from the schema-2 tree
            return self._resolve_flat_path(name)
        return None

    def _resolve_flat_path(self, name: str) -> AliasSpec | None:
        head, _, rest = name.partition('.')
        spec = self._by_name.get(self._channel_roots.get(head, ''))
        if spec is None:
            return None
        return self._walk_alias(spec, (head,) + tuple(rest.split('.')),
                              name)

    def _walk_alias(self, spec, path, name) -> AliasSpec | None:
        for field in spec.fields:
            if field.path == path:
                return AliasSpec(alias=name,
                                 canonical=f'{spec.name}.{field.key}',
                                 note='flattened schema-2 channel leaf')
            if path[:len(field.path)] != field.path:
                continue
            rest = path[len(field.path):]
            if not rest:
                continue
            if field.dtype.startswith('map<') and len(rest) == 1:
                return AliasSpec(
                    alias=name, canonical=f'{spec.name}.{field.key}',
                    parameter=rest[0],
                    note='flattened schema-2 map member')
            if field.dtype in ('struct', 'struct[]', 'map<struct>'):
                child = self._walk_struct(field, rest, spec.name, name)
                if child is not None:
                    return child
        return None

    def _walk_struct(self, field, rest, table, name) -> AliasSpec | None:
        if field.dtype == 'map<struct>':
            # rest[0] is a dynamic map key; the schema sits one level deeper.
            member = rest[0]
            if len(rest) == 1:
                return None
            child = next((c for c in field.children if c.key == rest[1]),
                         None)
            if child is None or len(rest) != 2:
                return None
            return AliasSpec(
                alias=name,
                canonical=f'{table}.{field.key}.{member}.{child.key}',
                parameter=member,
                note='flattened schema-2 channel leaf')
        child = next((c for c in field.children if c.key == rest[0]), None)
        if child is None:
            return None
        if len(rest) == 1:
            return AliasSpec(
                alias=name,
                canonical=f'{table}.{field.key}.{child.key}',
                note='flattened schema-2 channel leaf')
        if child.dtype in ('struct', 'map<struct>'):
            return self._walk_struct(child, rest[1:],
                                     f'{table}.{field.key}', name)
        return None


_DEFAULT_SCALAR_ALIASES = {
    'x_m': ('qpos', 'root_x'),
    'speed_mps': ('dof', 'root_x'),
    'pitch_rad': ('qpos', 'root_pitch'),
}


def build_catalog(joints, *, scalar_aliases=None) -> Catalog:
    """Build the reviewed catalog on the model's actual joint map.

    ``joints`` are ``JointCoords`` values in model order (dense qpos and DOF
    addresses). ``scalar_aliases`` names the legacy scalar columns bound to
    specific coordinates, e.g. ``'x_m'`` -> ``('qpos', 'root_x')``; the
    default matches the physical runtime's root joints.
    """
    joints = tuple(joints)
    coordinates = _coordinate_catalog(joints)
    if scalar_aliases is None:
        alias_items = list(_DEFAULT_SCALAR_ALIASES.items())
    elif isinstance(scalar_aliases, Mapping):
        alias_items = list(scalar_aliases.items())
    else:
        alias_items = list(scalar_aliases)
    tables = (
        _physics_table(), _forces_table(), _mass_table(), _energy_table(),
        _sensors_table(), _drive_table(), _tires_table(),
        _suspension_table(), _control_table(), _rider_table(),
        _model_status_table(),
    )
    specs = []
    by_coord = {}
    for coordinate in coordinates.qpos + coordinates.dof:
        by_coord.setdefault((coordinate.kind, coordinate.joint),
                            []).append(coordinate)
    for alias, (kind, joint) in alias_items:
        if kind not in ('qpos', 'dof'):
            raise ValueError(
                f'scalar alias {alias!r} must bind a qpos or dof joint')
        matches = by_coord.get((kind, joint), [])
        if len(matches) != 1:
            raise ValueError(
                f'scalar alias {alias!r} needs exactly one {kind} coordinate '
                f'for joint {joint!r}, found {len(matches)}')
        canonical = 'physics.qpos' if kind == 'qpos' else 'physics.qvel'
        specs.append(AliasSpec(alias=alias, canonical=canonical,
                               index=matches[0].index,
                               note='legacy scalar column bound by meaning'))
    for i, coordinate in enumerate(coordinates.qpos):
        for alias in (f'qpos.{i}', f'prestep_qpos_{i}'):
            specs.append(AliasSpec(alias=alias, canonical='physics.qpos',
                                   index=i,
                                   note='flattened schema-2 element name'))
    for i, coordinate in enumerate(coordinates.dof):
        for alias in (f'qvel.{i}', f'prestep_qvel_{i}'):
            specs.append(AliasSpec(alias=alias, canonical='physics.qvel',
                                   index=i,
                                   note='flattened schema-2 element name'))
    specs += [
        AliasSpec('interval_start_s', 'physics.time_s',
                  note='legacy interval-start alias'),
        AliasSpec('interval_dt_s', 'physics.dt_s',
                  note='legacy interval-duration alias'),
        AliasSpec('compiled_mass_kg', 'channel_mass.mass_kg',
                  note='PhysicalRecorder compiled-mass column'),
        AliasSpec('kinetic_energy_j', 'channel_mass.kinetic_energy_j',
                  note='PhysicalRecorder mass energy column'),
        AliasSpec('gravitational_energy_j',
                  'channel_mass.gravitational_energy_j',
                  note='PhysicalRecorder mass energy column'),
        AliasSpec('mechanical_energy_j',
                  'channel_energy.mechanical_energy_j',
                  note='PhysicalRecorder energy column'),
        AliasSpec('energy_residual_j', 'channel_energy.residual_j',
                  note='PhysicalRecorder energy residual column'),
    ]
    for axis, index in (('x', 0), ('y', 1), ('z', 2)):
        specs.append(AliasSpec(f'compiled_com_{axis}_m',
                               'channel_mass.com_m', index=index,
                               note='PhysicalRecorder compiled-CoM column'))
    specs += [
        AliasSpec('powers_w.{component}', 'forces.power_w',
                  note='schema-2 power object member'),
        AliasSpec('power_{component}_w', 'forces.power_w',
                  note='PhysicalRecorder power column'),
        AliasSpec('{component}_power_w', 'forces.power_w',
                  note='PhysicalRecorder power column'),
        AliasSpec('{component}_work_j', 'channel_energy.component_work_j',
                  note='PhysicalRecorder work column'),
        AliasSpec('component_work_j.{component}',
                  'channel_energy.component_work_j',
                  note='schema-2 work object member'),
    ]
    return Catalog(tables, coordinates, specs)
