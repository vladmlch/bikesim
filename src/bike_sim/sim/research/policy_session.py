"""Motor-policy lifecycle shared by interactive and unattended experiments."""
from hashlib import sha256
from importlib import import_module
import inspect
from pathlib import Path

from bike_sim.sim.ride.control import RideControl


def load_policy(reference: str):
    module_name, separator, factory_name = reference.partition(':')
    if not separator or not module_name or not factory_name:
        raise ValueError('policy must be module:factory')
    try:
        factory = getattr(import_module(module_name), factory_name)
        policy = factory()
    except (ImportError, AttributeError, TypeError) as error:
        raise ValueError(f'cannot load policy {reference}: {error}') from error
    if not callable(getattr(policy, 'reset', None)) or not callable(getattr(policy, 'act', None)):
        raise ValueError('policy must implement reset(seed) and act(observation, demand_nm)')
    return policy


def _metadata(policy, reference):
    try:
        source = inspect.getsourcefile(type(policy))
    except TypeError:
        source = None
    path = Path(source) if source is not None else None
    available = path is not None and path.is_file()
    return {
        'reference': reference,
        'class': f'{type(policy).__module__}.{type(policy).__qualname__}',
        'source_file': str(path.resolve()) if available else None,
        'source_sha256': sha256(path.read_bytes()).hexdigest() if available else None,
    }


class PolicySession:
    """One policy call per external interval, independent of wall-clock chunks."""
    def __init__(self, env, policy, *, reference=None):
        from collections import deque
        self.env, self.policy, self.reference = env, policy, reference
        self.operator_events = []
        self._queued = deque()
        self._paused = False
        self._brakes = (0., 0.)
        self._start_policy()

    def _start_policy(self):
        try:
            self.policy.reset(self.env.seed)
            metadata = _metadata(self.policy, self.reference)
        except Exception as error:
            self.env.fail_policy(error)
            raise
        self.env.run_metadata.update(policy=metadata,
            operator_events=self.operator_events, operator_intervention=False)

    @property
    def pending(self):
        return self.env.control_pending

    @property
    def paused(self):
        return self._paused

    @staticmethod
    def _validate_brakes(front, rear):
        from bike_sim.physics.checks import scalar
        values = tuple(scalar(value, 'brake demand', minimum=0.) for value in (front, rear))
        if any(value > 1. for value in values):
            raise ValueError('brake demand must lie in [0, 1]')
        return values

    def begin_advance(self, *, front_brake_demand=None, rear_brake_demand=None):
        if self.pending:
            raise RuntimeError('an external policy interval is already active')
        if self._paused or self.env.done:
            raise RuntimeError('cannot start a policy interval while paused or ended')
        front, rear = self._validate_brakes(
            self._brakes[0] if front_brake_demand is None else front_brake_demand,
            self._brakes[1] if rear_brake_demand is None else rear_brake_demand)
        try:
            command = self.policy.act(self.env.observation, self.env.demand_nm)
            if not isinstance(command, RideControl):
                raise ValueError('policy must return RideControl')
            if (command.human_torque_nm is not None or command.posture is not None
                    or command.crank_target_rate_rad_s is not None or not command.rider_enabled):
                raise ValueError('motor policy cannot own rider inputs')
        except Exception as error:
            self.env.fail_policy(error)
            raise
        self.env.begin_control(command, front_brake_demand=front, rear_brake_demand=rear)
        if front or rear:
            self.operator_events.append(dict(time_s=self.env.sim.time_s,
                front_brake_demand=front, rear_brake_demand=rear))
            self.env.run_metadata['operator_intervention'] = True

    def advance_pending(self, *, wall_budget_s=None, target_step=None):
        if not self.pending:
            raise RuntimeError('begin_advance is required before advance_pending')
        options = dict(wall_budget_s=wall_budget_s)
        if target_step is not None:
            options['target_step'] = target_step
        result = self.env.advance_control(**options)
        if result is not None:
            self._apply_queued()
        return result

    def advance(self, *, front_brake_demand=None, rear_brake_demand=None):
        self.begin_advance(front_brake_demand=front_brake_demand,
                           rear_brake_demand=rear_brake_demand)
        result = self.advance_pending()
        while result is None:
            result = self.advance_pending()
        return result

    def _operator(self, kind, value=None):
        if self.pending:
            self._queued.append((kind, value))
            return
        self._apply_operator(kind, value)

    def _apply_operator(self, kind, value):
        if kind == 'stop':
            self.env.stop()
        elif kind == 'pause':
            self._paused = value
        elif kind == 'brakes':
            self._brakes = value
        elif kind == 'reset':
            self._reset_now(value)
        else:
            raise ValueError(f'unknown operator event: {kind}')

    def _apply_queued(self):
        while self._queued:
            kind, value = self._queued.popleft()
            self._apply_operator(kind, value)

    def set_brakes(self, *, front_brake_demand=0., rear_brake_demand=0.):
        self._operator('brakes', self._validate_brakes(front_brake_demand, rear_brake_demand))

    def pause(self):
        self._operator('pause', True)

    def resume(self):
        self._operator('pause', False)

    def _reset_now(self, seed):
        observation = self.env.reset(seed=seed)
        self.operator_events.clear()
        self._paused, self._brakes = False, (0., 0.)
        self._start_policy()
        return observation

    def reset(self, *, seed=None):
        if seed is not None and (type(seed) is not int or seed < 0):
            raise ValueError('reset seed must be a nonnegative integer')
        if self.pending:
            self._queued.append(('reset', seed))
            return None
        self._queued.clear()
        return self._reset_now(seed)

    def stop(self):
        self._operator('stop')

    def save(self, directory, *, overwrite=False):
        previous = self.env.run_metadata['policy']
        current = _metadata(self.policy, self.reference)
        self.env.run_metadata['policy_source_changed_during_run'] = (
            current['source_sha256'] != previous['source_sha256'])
        return self.env.save(directory, overwrite=overwrite)


def episode_exit_code(env):
    if env.error is not None:
        return 3
    if not env.numerically_valid or not env.model_valid:
        return 2
    return 0
