"""Shared synthetic fixtures for the run-recording contract tests.

``run_metadata`` carries the fixed identity of the recording contract tests;
``TreePhysicalSource`` converts hand-authored nested sample rows into typed
``TableBatch`` values strictly through the production field catalog and
encoder — never through the native adapter or a simulator.
"""
from datetime import datetime, timezone
from uuid import UUID

from bike_sim.recording.capture import RunCapture
from bike_sim.recording.catalog import JointCoords, build_catalog
from bike_sim.recording.contracts import EpisodeEvidence, RunMetadata
from bike_sim.recording.encoding import encode_rows, force_rows


def run_metadata():
    """Deterministic synthetic run identity for recording tests."""
    return RunMetadata(
        run_id=UUID('00000000-0000-0000-0000-000000000001'),
        started_at=datetime(2026, 10, 12, 0, 0, 0, tzinfo=timezone.utc),
        backend='native',
        mode='ride',
        values={'provenance': {
            'kind': 'synthetic',
            'author': 'tests/reference/_recording_support.py',
            'purpose': 'deterministic capture and catalog regression tests',
            'physical_source': 'none (hand-authored trees)',
        }},
    )


# A small planar joint set shaped like the supported physical model's scalar
# joints: joint name -> (type, qposadr, dofadr). nq == nv == 8.
_SYNTHETIC_JOINTS = (
    ('root_x', 'slide'),
    ('root_z', 'slide'),
    ('root_pitch', 'hinge'),
    ('front_wheel_spin', 'hinge'),
    ('rear_wheel_spin', 'hinge'),
    ('crank_spin', 'hinge'),
    ('fork_slide', 'slide'),
    ('shock_slide', 'slide'),
)


def synthetic_joints():
    return tuple(
        JointCoords(name=name, joint_type=joint_type, qposadr=i, dofadr=i)
        for i, (name, joint_type) in enumerate(_SYNTHETIC_JOINTS))


def support_catalog():
    """The production catalog specialized on the synthetic joint map."""
    return build_catalog(synthetic_joints())


def sample_row(interval_id=0, *, start=0.0, dt=0.001, qpos=None, qvel=None,
               forces=None, powers=None, channels=None):
    """One hand-authored canonical sample tree (see catalog.TableSpec kinds)."""
    n = len(_SYNTHETIC_JOINTS)
    return {
        'interval_id': interval_id,
        'time_s': start,
        'end_time_s': start + dt,
        'dt_s': dt,
        'qpos': [0.0] * n if qpos is None else list(qpos),
        'qvel': [0.0] * n if qvel is None else list(qvel),
        'forces': {} if forces is None else dict(forces),
        'powers_w': {} if powers is None else dict(powers),
        'channels': {} if channels is None else dict(channels),
    }


def energy_channels(**overrides):
    """A complete authored ``channels`` row centred on the energy domain."""
    channels = {
        'energy': {
            'mechanical_energy_j': 12.5,
            'elastic_energy_j': {'fork_spring': 0.25},
            'active_work_j': 1.5,
            'external_work_j': -0.5,
            'loss_j': 0.25,
            'loss_step_j': 0.001,
            'solver_constraint_work_j': 0.0,
            'energy_scale_j': 10.0,
            'muscle_signed_j': 1.0,
            'muscle_positive_j': 1.25,
            'motor_signed_j': 0.5,
            'motor_positive_j': 0.5,
            'source_positive_work_j': 1.75,
            'constraint_signed_j': 0.0,
            'constraint_absolute_j': 0.0,
            'constraint_work_ratio': 0.0,
            'constraint_work_ok': True,
            'residual_j': -0.001,
            'electrical_work_j': 0.5,
            'electrical_residual_j': 0.0,
            'measured': True,
            'source_interval_id': 0,
            'source_time_s': 0.001,
        },
        'component_work_j': {'chain': 0.1, 'tires': -0.2},
    }
    for key, value in overrides.items():
        if key == 'energy':
            channels['energy'].update(value)
        else:
            channels[key] = value
    return channels


def drive_channels(speed=0.0, **overrides):
    """A complete authored ``channels`` row centred on the ``drive`` root.

    Every ``channel_drive`` leaf appears — required ones always, optional
    ones too, so the authored root exercises both lanes of the
    optional-leaf encoding contract. ``speed`` perturbs the authored
    values deterministically. ``overrides`` mirrors ``energy_channels``:
    a ``drive`` mapping merges into the root, any other key becomes its
    own root.
    """
    drive = {
        'transmission_model': 'ideal_mid_drive',
        'omits_suspension_coupling': True,
        'chain_extension_m': 0.001 + speed,
        'chain_extension_rate_mps': 0.002 - speed,
        'chain_tension_n': 120.0 + speed,
        'chain_energy_j': 0.06 + speed,
        'chain_dissipation_power_w': -0.4 - speed,
        'chain_power_w': 12.0 - speed,
        'freehub_torque_nm': 8.0 + speed,
        'freehub_energy_j': 0.02,
        'freehub_engaged': speed < 0.5,
        'freehub_deflection_rad': -0.05 + speed,
        'freehub_dissipation_power_w': -0.0 if speed else 0.1,
        'freehub_power_w': 12.1,
        'cadence_rpm': 80.0 + speed,
        'crank_rad_s': 8.4 + speed,
        'drive_shaft_rad_s': 30.0 - speed,
        'crank_phase_rad': 1.25,
        'human_torque_nm': 20.0,
        'human_sensor_nm': 19.8 + speed,
        'human_setpoint_nm': 20.0,
        'human_command_nm': 20.0,
        'human_active_power_w': 168.0,
        'human_joint_power_w': 170.0 - speed,
        'assist_demand_gated': speed >= 0.5,
        'assist_sensor_nm': 19.7,
        'assist_mode': 'assist',
        'assist_gain': 1.0,
        'rider_mode': 'pedaling',
        'coasting_reason': 'downhill',
        'required_cadence_rpm': 72.0,
        'crank_target_phase_rad': 1.0,
        'crank_target_rate_rad_s': 8.0,
        'motor_request_nm': 6.5 + speed,
        'motor_setpoint_nm': 6.0,
        'motor_limit_nm': 9.0,
        'motor_limited_request_nm': 6.5,
        'motor_torque_nm': 6.0 + speed,
        'motor_control_source': 'assist',
        'safety_limited': False,
        'energy_limited': False,
        'motor_enabled': True,
        'battery_empty': False,
        'battery_energy_j': 4100.0 - speed,
        'motor_shaft_power_w': 150.0,
        'electrical_power_w': 151.5,
        'motor_freewheel_engaged': True,
        'motor_freewheel_torque_nm': 0.4,
        'motor_freewheel_dissipation_power_w': -0.05,
        'crank_clutch_torque_nm': 0.3,
        'crank_clutch_engaged': False,
        'crank_clutch_dissipation_power_w': 0.0,
        'gear_front_teeth': 44,
        'gear_rear_teeth': 16,
        'gear_ratio': 2.75,
        'shift_active': False,
        'shift_direction': 'none',
        'shift_from_teeth': 16,
        'shift_count': int(speed),
        'shift_time_s': 0.5 + speed,
        'shift_torque_factor': 1.0,
        'front_brake_demand': 0.0,
        'rear_brake_demand': 0.0,
        'rollback_brake_demand': 0.0,
        'front_brake_power_w': -0.0,
        'rear_brake_power_w': 0.0,
        'front_brake_torque_nm': 0.0,
        'rear_brake_torque_nm': 0.0,
        'transmission_phi_m': 0.02,
        'transmission_boundary_m': 0.05,
        'transmission_gap_m': 0.03,
        'transmission_tension_n': 40.0,
        'transmission_constraint_defect_m': 1e-9,
        'transmission_interval_work_j': 0.01,
        'transmission_reaction_error_n': 0.001,
        'transmission_reference_status': 'qualified',
        'shift_parameter_work_j': 0.0,
        'shift_interval_constraint_work_j': 0.0,
        'shift_constraint_work_cumulative_j': 0.0,
    }
    channels = {'drive': drive}
    for key, value in overrides.items():
        if key == 'drive':
            channels['drive'].update(value)
        else:
            channels[key] = value
    return channels


def sensors_channels(speed=0.0, **overrides):
    """A complete authored ``channels`` row centred on the ``sensors`` root.

    All seven ``channel_sensors`` leaves are required; ``encoders_rad_s``
    covers the depth-3 nested-leaf path. ``overrides`` merge like
    ``energy_channels``.
    """
    channels = {
        'sensors': {
            'frame_specific_force_body_mps2': [9.81 + speed, 0.0, -0.0],
            'frame_gyro_body_rad_s': [0.0, 0.01 * (speed + 1.0), 0.0],
            'encoders_rad_s': {
                'front_wheel': 4.0 + speed,
                'rear_wheel': 4.0 + speed,
                'crank': 1.4,
            },
            'motor_torque_nm': 12.0 + speed,
            'human_torque_nm': 20.0,
        },
    }
    for key, value in overrides.items():
        if key == 'sensors':
            channels['sensors'].update(value)
        else:
            channels[key] = value
    return channels


class TreePhysicalSource:
    """PhysicalSource over authored sample trees, encoded via the catalog.

    ``rows`` are canonical sample trees as produced by ``sample_row``. Every
    table in the bound catalog is emitted: ``physics`` and ``channel_*`` rows
    come straight from each sample, ``forces`` explodes the per-component
    generalized-force vectors into long rows. Conversion delegates to the
    production ``encoding.encode_rows``; nothing is flattened by hand here.
    """

    def __init__(self, rows=(), *, catalog=None):
        self._catalog = support_catalog() if catalog is None else catalog
        self._rows = tuple(rows)

    @property
    def catalog(self):
        return self._catalog

    @property
    def row_count(self):
        return len(self._rows)

    def _table_rows(self, spec):
        if spec.kind == 'physics':
            return [dict(row) for row in self._rows]
        if spec.kind == 'forces':
            return [row for sample in self._rows
                    for row in force_rows(sample)]
        if spec.kind == 'channels':
            return [{'interval_id': sample['interval_id'],
                     **{root: sample['channels'][root]
                        for root in spec.claims
                        if root in sample['channels']}}
                    for sample in self._rows]
        raise ValueError(f'unknown table kind {spec.kind!r}')

    def _check_channel_roots(self):
        claimed = self._catalog.channel_roots()
        for sample in self._rows:
            for root in sample['channels']:
                if root not in claimed:
                    from bike_sim.recording.encoding import UnknownFieldError
                    raise UnknownFieldError(
                        f'channels: unrecognized domain {root!r}')

    def tables(self, *, batch_rows):
        if type(batch_rows) is not int or batch_rows < 1:
            raise ValueError('batch_rows must be a positive integer')
        self._check_channel_roots()
        for spec in self._catalog.table_order():
            rows = self._table_rows(spec)
            for offset in range(0, max(len(rows), 1), batch_rows):
                yield encode_rows(spec, rows[offset:offset + batch_rows])


def episode(source, episode_id=0, *, generation=0, summary=None, assets=()):
    return EpisodeEvidence(
        episode_id=episode_id, generation=generation, physical=source,
        summary={} if summary is None else summary, assets=tuple(assets))


def value_case_rows(dt=1 / 8):
    """The reviewed optional-leaf and nonfinite matrix on ``energy.residual_j``.

    Intervals 0-2 hold a NaN, an explicit null and an absent leaf
    (``residual_j_present`` True, True, False); intervals 3-4 hold an
    explicitly null and an absent ``energy`` parent; intervals 5-8 hold
    +0.0, -0.0, +inf and -inf. Every interval is self-measured so the
    source/measurement provenance stays consistent. NaN and ±inf
    round-trip verbatim; the container's float compression may normalize
    the -0.0 sign bit on disk (layout-dependent), so signed-zero fidelity
    is an authored-level guarantee here, not a storage one.
    """
    leaves = (float('nan'), None, 'absent', 'parent-null', 'parent-absent',
              0., -0., float('inf'), -float('inf'))
    rows = []
    for i, value in enumerate(leaves):
        channels = energy_channels(energy={'measured': True,
                                           'source_interval_id': i,
                                           'source_time_s': (i + 1) * dt})
        if value == 'parent-absent':
            del channels['energy']
        elif value == 'parent-null':
            channels['energy'] = None
        elif value == 'absent':
            del channels['energy']['residual_j']
        else:
            channels['energy']['residual_j'] = value
        rows.append(sample_row(i, start=i * dt, dt=dt, channels=channels))
    return rows


def _with_forces(row):
    """Hand-computed per-component powers (``qfrc·qvel``) and work (``·dt``)."""
    n = len(_SYNTHETIC_JOINTS)
    qfrc = {'chain': [3.] + [0.] * (n - 1),
            'tires': [-1., 0.] + [0.] * (n - 2)}
    powers = {name: sum(f * v for f, v in zip(vector, row['qvel']))
              for name, vector in qfrc.items()}
    dt = row['dt_s']
    channels = dict(row['channels'])
    channels['component_work_j'] = {name: power * dt
                                    for name, power in powers.items()}
    return dict(row, forces=qfrc, powers_w=powers, channels=channels)


def _forwarded(rows):
    """Unmeasured rows forwarding the LAST interval's measurement metadata."""
    closing = rows[-1]
    out = []
    for row in rows:
        measured = row['interval_id'] == closing['interval_id']
        channels = dict(row['channels'],
                        energy=dict(row['channels'].get('energy') or {},
                                    measured=measured,
                                    source_interval_id=closing['interval_id'],
                                    source_time_s=closing['end_time_s']))
        out.append(dict(row, channels=channels))
    return out


def synthetic_capture(rows=None, *, episodes=1, summary=None, assets=(),
                      channels=None, forces=False, forwarded=False,
                      value_cases=False, events=(), metadata=None,
                      catalog=None, enabled=True):
    """A reviewed ``RunCapture`` over ``TreePhysicalSource`` sample trees.

    No simulator ever runs: ``rows`` are hand-authored canonical sample
    trees (see ``sample_row``); per-component powers/work are computed
    inline. Options cover the reviewed synthetic matrix:

    * ``episodes`` — a count sharing ``rows`` (``episodes=0`` seals
      nothing), or an explicit sequence of per-episode row iterables.
    * ``value_cases`` — replaces ``rows`` with the optional-leaf and
      nonfinite matrix of ``value_case_rows``.
    * ``forces`` — adds two force components whose ``powers_w`` and
      ``component_work_j`` are the hand-computed ``qfrc·qvel`` and
      ``qfrc·qvel·dt`` values.
    * ``forwarded`` — marks all but the last interval unmeasured, carrying
      the closing interval's ``source_interval_id``/``source_time_s``.
    * ``channels`` — a payload merged into every row's ``channels`` (or a
      callable returning it per row), for categorical/boolean coverage.
    * ``events`` — pre-built ``InputEvent`` values journaled in order.
    * ``summary`` — one mapping for every episode, or a per-episode
      sequence; ``assets``/``metadata`` forward verbatim.
    * ``catalog`` — a catalog override for every episode's
      ``TreePhysicalSource`` (e.g. a renamed joint map for
      coordinate-identity mismatch tests).
    """
    capture = RunCapture(run_metadata() if metadata is None else metadata,
                         enabled=enabled)
    if value_cases:
        per_episode = [value_case_rows()]
    elif type(episodes) is int:
        shared = list(() if rows is None else rows)
        per_episode = [shared] * episodes
    else:
        per_episode = [list(each) for each in episodes]

    def materialize(row):
        row = dict(row)
        if channels is not None:
            extra = channels(row) if callable(channels) else channels
            row['channels'] = {**row['channels'], **extra}
        return _with_forces(row) if forces else row

    for index, episode_rows in enumerate(per_episode):
        materialized = [materialize(row) for row in episode_rows]
        if forwarded:
            materialized = _forwarded(materialized)
        episode_summary = (summary[index] if isinstance(summary, (list, tuple))
                           else summary)
        capture.add_episode(episode(
            TreePhysicalSource(materialized, catalog=catalog), index,
            generation=index, summary=episode_summary, assets=assets))
    for event in events:
        capture.event(event)
    return capture
