"""Capture every solved interval; evaluate attachments together at period close."""
from dataclasses import dataclass, field
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
                for (i, a), w, bike, good in zip(rows, wr, wb, ok_r & ok_b):
                    if not good or not a.observable:
                        continue
                    scale = max(1., float(np.linalg.norm(w[:3])))
                    if not np.allclose(bike, -w, rtol=1e-4, atol=1e-4*scale):
                        continue
                    if not np.all(np.abs(w[[1, 3, 5]]) <= 1e-6*scale):
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
        result.append(dict(raw.metadata['effort_base'],
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
