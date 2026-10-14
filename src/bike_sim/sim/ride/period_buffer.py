"""Capture every solved interval; evaluate attachments together at period close."""
import copy
from dataclasses import asdict, dataclass, field
import numpy as np
from bike_sim.physics.attachment_budget import AttachmentSample
from bike_sim.physics.energy_ledger import step_work


@dataclass(frozen=True)
class RawStep:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: dict
    attachment_raw: dict
    actuator_force: np.ndarray
    qfrc_passive: np.ndarray
    contact_truth: dict
    metadata: dict = field(default_factory=dict)


class PeriodBuffer:
    def __init__(self, capacity: int):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('period buffer needs a positive capacity')
        self.capacity = capacity
        self._raws = []

    def push(self, raw: RawStep) -> None:
        if len(self._raws) >= self.capacity:
            raise RuntimeError('period buffer overflow: close the period first')
        if self._raws and raw.interval_id != self._raws[-1].interval_id+1:
            raise ValueError('period buffer requires consecutive intervals')
        self._raws.append(raw)

    @property
    def full(self) -> bool:
        return len(self._raws) >= self.capacity

    def drain(self):
        raws, self._raws = self._raws, []
        return raws


def _stack_wrenches(jacobians, qfrcs, *, full_rank=True):
    jac = np.stack(jacobians)
    target = np.stack(qfrcs)
    matrix = np.transpose(jac, (0, 2, 1))
    wrench = np.einsum('nrv,nv->nr', np.linalg.pinv(matrix, rcond=1e-12), target)
    explained = np.einsum('nvr,nr->nv', matrix, wrench)
    ok = np.all(np.isclose(explained, target, rtol=1e-8, atol=1e-8), axis=1)
    if full_rank:
        # Match lstsq's explicit rcond, rather than matrix_rank's default.
        singular = np.linalg.svd(matrix, compute_uv=False)
        rank = (singular > singular[:, :1]*1e-12).sum(axis=1)
        ok &= rank == jac.shape[1]
    return wrench, ok


def evaluate_attachments(raws, *, kinds=None, half_patch_m=None, gap_m=None):
    """Recover planar illustrations or the runtime's paired spatial body wrenches.

    Spatial body Jacobians can legitimately have null rotational directions;
    their scalar oracle checks explanation and in-plane force observability,
    not full spatial rank. Every original check and tolerance is retained.
    """
    from bike_sim.sim.ride.attachment_wrench import AttachmentRaw, decompose_wrench
    result = [dict() for _ in raws]
    names = sorted({name for raw in raws for name in raw.attachment_raw})
    for name in names:
        groups = {}
        for i, raw in enumerate(raws):
            if name not in raw.attachment_raw:
                continue
            item = raw.attachment_raw[name]
            key = ((item.rider_jac.shape, item.bike_jac.shape) if isinstance(item, AttachmentRaw)
                   else item[0].shape)
            groups.setdefault(key, []).append((i, item))
        for rows in groups.values():
            if isinstance(rows[0][1], AttachmentRaw):
                items = [item for _, item in rows]
                wr, ok_r = _stack_wrenches([a.rider_jac for a in items],
                    [a.rider_qfrc for a in items], full_rank=False)
                wb, ok_b = _stack_wrenches([a.bike_jac for a in items],
                    [a.bike_qfrc for a in items], full_rank=False)
                # Keep the scalar norm arithmetic and all original tolerances;
                # apply the identical elementwise checks across the group once.
                scales = np.array([max(1., float(np.linalg.norm(w[:3]))) for w in wr])
                valid = (ok_r & ok_b & np.array([a.observable for a in items])
                         & np.all(np.isclose(wb, -wr, rtol=1e-4,
                                            atol=1e-4*scales[:, None]), axis=1)
                         & np.all(np.abs(wr[:, [1, 3, 5]]) <= 1e-6*scales[:, None], axis=1))
                for (i, a), w, good in zip(rows, wr, valid):
                    if not good:
                        continue
                    result[i][name] = decompose_wrench(w, a.normal, a.kind,
                        rotational=a.rotational, half_patch_m=a.half_patch_m,
                        gap_m=a.gap_m, pull_direction=a.pull_direction)
            else:
                wrench, ok = _stack_wrenches([j for _, (j, _) in rows],
                                             [q for _, (_, q) in rows])
                for (i, _), w, good in zip(rows, wrench, ok):
                    if good:
                        kind = kinds[name]
                        result[i][name] = AttachmentSample(kind, float(w[1]), float(w[0]),
                            float(w[2]) if len(w) == 3 else 0., float(gap_m[name]),
                            pull_n=float(np.hypot(w[0], w[1])) if kind == 'grip' else 0.,
                            half_patch_m=float((half_patch_m or {}).get(name, 0.)))
    return result


@dataclass(frozen=True)
class PeriodReport:
    violations_by_step: tuple
    works: tuple
    first_failure: tuple | None
    attachments: tuple
    efforts: tuple


def _effort_observations(runtime, raws):
    """Vectorized solved powers with the original strength curves at every step."""
    c = runtime.rider_control
    if c is None:
        return [{} for _ in raws]
    names = tuple(c.joints)
    dofs = [c.joints[name][1] for name in names]
    aids = [c.joints[name][2] for name in names]
    delivered = np.stack([raw.actuator_force[aids] for raw in raws])
    speeds = np.stack([raw.qvel[dofs] for raw in raws])
    passive = np.stack([raw.qfrc_passive[dofs] for raw in raws])
    positive = np.maximum(delivered*speeds, 0.).sum(axis=1)
    damping_power = (passive*speeds).sum(axis=1)
    limit = c.config.active_positive_power_limit_w
    dt = runtime.control_clock.timestep_s
    result = []
    for i, raw in enumerate(raws):
        active = dict(zip(names, map(float, delivered[i])))
        violations = c.strength_violations(active, raw.qpos, raw.qvel)
        from bike_sim.sim.ride.rider_effort import joint_effort_limits
        joint_limits=joint_effort_limits(active,dict(zip(names,speeds[i])),c.config)
        result.append(dict(raw.metadata['effort_base'], **joint_limits,
            rider_active_delivered_nm=active, rider_positive_power_w=float(positive[i]),
            rider_passive_power_w=float(damping_power[i]),
            rider_positive_work_step_j=float(positive[i])*dt,
            rider_passive_work_step_j=float(damping_power[i])*dt,
            rider_strength_violations=violations,
            rider_effort_budget_exceeded=bool(limit is not None and positive[i] > limit+1e-9),
            rider_effort_observation='solved_actuator_force_at_incoming_interval'))
    return result


def evaluate_period(runtime, raws, budget):
    from bike_sim.physics.attachment_budget import attachment_violations
    attachments = evaluate_attachments(raws)
    efforts = _effort_observations(runtime, raws)
    violations, works = [], []
    first = None
    for raw, samples, effort in zip(raws, attachments, efforts):
        errors = list(raw.metadata.get('attachment_errors', ()))
        errors.extend(name+':unobservable_attachment_wrench'
                      for name in raw.attachment_raw if name not in samples)
        for name in raw.attachment_raw:
            if name in samples:
                errors.extend(f'{name}.{v}' for v in attachment_violations(samples[name], budget))
        c = runtime.rider_control
        if c is not None:
            errors.extend(f'rider_strength.{name}' for name in
                          effort['rider_strength_violations'])
            errors.extend(f'rider_power.{name}' for name in effort['rider_joint_power_violations'])
            errors.extend(f'rider_speed.{name}' for name in effort['rider_joint_speed_violations'])
            if effort['rider_effort_budget_exceeded']:
                errors.append('rider_power.positive')
            if raw.metadata.get('invalid_controller'):
                errors.append('rider_controller.infeasible')
        errors = tuple(errors)
        violations.append(errors)
        if errors and first is None:
            first = raw.end_time_s, errors
        components, v = raw.components, raw.qvel
        muscle = np.array([float(f@v) for n, f in components.items()
                           if n == 'human_crank' or n.startswith('act_rider_')])
        motor = float(components.get('mid_drive', np.zeros_like(v))@v)
        constraint = np.array(list(raw.metadata['numerical_constraint_power_w'].values()))
        works.append(step_work(muscle, motor, constraint, runtime.control_clock.timestep_s))
    return PeriodReport(tuple(violations), tuple(works), first, tuple(attachments), tuple(efforts))


_EXTERNAL_FORCE_NAMES = frozenset({'external', 'rear_drive', 'road_rolling',
                                   'aerodynamic', 'native_contact'})


def publish_close(rt, raws):
    """Evaluate a drained period and publish its samples onto ``rt``.

    ``rt`` is the runtime-like object that owns the close state: the real
    PhysicalRuntime on the synchronous path, or a worker-side replica of just
    the fields below when period evaluation runs off the stepping process.
    Nothing here touches live mjData, so the result is a pure function of the
    raws plus the carried state.

    Sets rt.completed_samples/sample/period_violations and returns the ordered
    ``(time_s, violations)`` failures for the caller's monitor to accept.
    """
    from bike_sim.physics.energy_ledger import constraint_work_ok
    from bike_sim.sim.ride.physical_samples import PhysicalSample
    report = evaluate_period(rt, raws, rt.cfg.articulated.attachment_budget())
    # The energy/momentum solve runs on full intervals only. A raw that
    # measured its state reports its own energy; an unmeasured raw carries
    # the closing measurement of its period. A crash or flush can drain a
    # partial period holding no measured interval at all -- then the
    # ledger keeps its previous datum instead of fabricating one.
    closing_total = closing_elastic = closing_source = None
    endpoint_mass = getattr(rt, '_last_endpoint_mass', None)
    for raw in reversed(raws):
        if raw.metadata['mechanical_energy_j'] is not None:
            closing_total = raw.metadata['mechanical_energy_j']
            closing_elastic = raw.metadata['elastic_energy_j']
            closing_source = (raw.interval_id, raw.end_time_s)
            endpoint_mass = raw.metadata['diagnostics']['endpoint_mass']
            break
    if closing_total is None:
        closing_total = rt.energy['mechanical_energy_j']
        closing_elastic = rt.energy['elastic_energy_j']
    rt._last_endpoint_mass = endpoint_mass
    # Provenance for every forwarded row in this period: the period's
    # closing measured interval when one exists, else the previous
    # period's closing measurement (None until the first measurement
    # ever commits — the catalog encodes that as explicit absence).
    source = closing_source if closing_source is not None else \
        getattr(rt, '_last_measurement_source', None)
    if closing_source is not None:
        rt._last_measurement_source = closing_source
    published = []
    failures = []
    rt.interval_constraints = {}
    for index, (raw, attachments, violations, work, effort) in enumerate(zip(
            raws, report.attachments, report.violations_by_step, report.works, report.efforts)):
        dt=rt.control_clock.timestep_s; v=raw.qvel
        components=raw.components
        loss_step=raw.metadata['loss_step_j']; rt.loss_j+=loss_step
        rt.muscle_signed_j+=work.muscle_signed_j
        rt.muscle_positive_j+=work.muscle_positive_j
        rt.motor_signed_j+=work.motor_signed_j
        rt.motor_positive_j+=work.motor_positive_j
        rt.constraint_absolute_j+=work.constraint_absolute_j
        rt.solver_work_j+=work.constraint_signed_j
        rt.active_work_j+=work.muscle_signed_j+work.motor_signed_j
        rt.external_work_j+=sum(float(components[n]@v)*dt for n in _EXTERNAL_FORCE_NAMES if n in components)
        rt.electrical_work_j+=raw.metadata['electrical_power_w']*dt
        source_positive = rt.muscle_positive_j+rt.motor_positive_j
        work_ok = constraint_work_ok(rt.constraint_absolute_j, source_positive)
        # This is an integral criterion, published at the closing physical
        # interval, including a partial terminal period. S8's zero-source
        # roundoff budget is absolute; it never grows with step count.
        if index == len(raws)-1 and not work_ok:
            violations += ('energy.constraint_work',)
        total=raw.metadata['mechanical_energy_j']
        elastic=raw.metadata['elastic_energy_j']
        # A measured row cites itself; a forwarded row cites the
        # interval its measurement came from (None only while no
        # measurement has ever committed).
        measured = total is not None
        if total is None:
            # Copy the forwarded closing elastic so each published sample
            # owns its dict instead of aliasing the period's shared one.
            total, elastic = (closing_total,
                              dict(closing_elastic) if isinstance(closing_elastic, dict) else closing_elastic)
        rt.energy={'mechanical_energy_j':total,'elastic_energy_j':elastic,
            'active_work_j':rt.active_work_j,'external_work_j':rt.external_work_j,
            'loss_j':rt.loss_j,'loss_step_j':loss_step,'solver_constraint_work_j':rt.solver_work_j,'energy_scale_j':rt.energy_scale_j,
            'muscle_signed_j':rt.muscle_signed_j,'muscle_positive_j':rt.muscle_positive_j,
            'motor_signed_j':rt.motor_signed_j,'motor_positive_j':rt.motor_positive_j,
            'source_positive_work_j':rt.muscle_positive_j+rt.motor_positive_j,
            'constraint_signed_j':rt.solver_work_j,'constraint_absolute_j':rt.constraint_absolute_j,
            'constraint_work_ratio':(rt.constraint_absolute_j/source_positive
                                     if source_positive > 0. else None),
            'constraint_work_ok':work_ok,
            'residual_j':total-rt.initial_energy_j-rt.active_work_j-rt.external_work_j+rt.loss_j,
            'electrical_work_j':rt.electrical_work_j,
            'electrical_residual_j':(rt.initial_battery_j-raw.metadata['battery_energy_j']-rt.electrical_work_j
                                    if rt.cfg.drive.battery.enabled else 0.),
            'measured':measured,
            'source_interval_id':raw.interval_id if measured else (None if source is None else source[0]),
            'source_time_s':raw.end_time_s if measured else (None if source is None else source[1])}
        rt.attachment_samples = attachments
        rt.attachment_errors = tuple(raw.metadata['attachment_errors']) + tuple(
            name+':unobservable_attachment_wrench' for name in raw.attachment_raw if name not in attachments)
        rt.step_violations = violations
        if rt.rider_contacts is not None:
            rt.rider_contacts.last_attachment_samples = attachments
            rt.rider_contacts.last_attachment_errors = rt.attachment_errors
        if rt.rider_control is not None:
            rt.rider_control.effort_diagnostics = dict(effort)
            rt.rider_control.last_terms = {name:dict(terms) for name, terms in
                                            raw.metadata['rider_control_terms'].items()}
            for name, (_, dof, aid) in rt.rider_control.joints.items():
                delivered = float(raw.actuator_force[aid])
                rt.rider_control.last_terms[name].update(solved_force_nm=delivered,
                    solved_active_nm=delivered, solved_passive_nm=float(raw.qfrc_passive[dof]))
        channels = dict(raw.metadata['channels'], **effort, energy=rt.energy,
                        attachment_violations=violations)
        full = raw.metadata['full'] or index == len(raws)-1
        if full:
            diagnostics = raw.metadata['diagnostics']
            endpoint_mass = diagnostics['endpoint_mass']
            if endpoint_mass is None:
                endpoint_mass = rt._last_endpoint_mass
            if isinstance(endpoint_mass, dict):
                endpoint_mass = dict(endpoint_mass)
            channels.update(rider=copy.deepcopy(diagnostics['rider']),
                rider_welds=copy.deepcopy(diagnostics['rider_welds']),
                endpoint_mass=endpoint_mass,
                rider_intent={} if diagnostics['rider_intent'] is None else
                    dict(asdict(diagnostics['rider_intent']), inclination_rad=diagnostics['inclination_rad']),
                rider_allocation={k:tuple(float(x) for x in value) if isinstance(value,np.ndarray) else value
                    for k,value in diagnostics['rider_allocation'].items() if not k.startswith('solution_')},
                rider_ik_saturation=copy.deepcopy(diagnostics['rider_ik_saturation']),
                rider_support_targets=copy.deepcopy(diagnostics['rider_support_targets']))
            channels['attachment_samples'] = {name:asdict(value) for name,value in attachments.items()}
            channels['component_work_j'] = {n:rt.history.work_j.get(n,0.)+float(f@v)*dt for n,f in components.items()}
            channels['rider_control'] = {} if rt.rider_control is None else copy.deepcopy(rt.rider_control.last_terms)
        # Energy quality judges a measurement; unmeasured intervals carry a
        # forwarded closing total whose hybrid residual must not feed the
        # (sticky) numerically_valid verdict.
        rt.model_status.observe(raw.interval_id,raw.time_s,
            channels if raw.metadata['mechanical_energy_j'] is not None else
            {k:v for k,v in channels.items() if k!='energy'})
        channels['model_status'] = rt.model_status.as_dict()
        sample = PhysicalSample(raw.interval_id,raw.time_s,raw.end_time_s,
                                raw.qpos,raw.qvel,components,channels)
        rt.history.add(sample)
        published.append(sample)
        rt.interval_constraints[raw.interval_id] = raw.metadata['constraint_snapshot']
        if violations:
            failures.append((raw.end_time_s,violations))
    rt.completed_samples = tuple(published)
    rt.sample = published[-1]
    rt.period_violations = tuple(failures)
    return failures
