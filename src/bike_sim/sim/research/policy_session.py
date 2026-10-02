"""Motor-policy lifecycle shared by interactive and unattended experiments."""
from dataclasses import replace
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
    def __init__(self, env, policy, *, reference=None):
        self.env = env
        self.policy = policy
        self.reference = reference
        self.operator_events = []
        self._start_policy()

    def _start_policy(self):
        self.policy.reset(self.env.seed)
        self.env.run_metadata.update(
            policy=_metadata(self.policy, self.reference),
            operator_events=self.operator_events, operator_intervention=False)

    def advance(self, *, front_brake_demand=0., rear_brake_demand=0.,
                crank_reposition=False):
        if self.env.done:
            raise RuntimeError('episode has ended')
        try:
            command = self.policy.act(self.env.observation, self.env.demand_nm)
            if not isinstance(command, RideControl):
                raise ValueError('policy must return RideControl')
            if (command.human_torque_nm is not None or command.posture is not None
                    or not command.rider_enabled):
                raise ValueError('motor policy cannot own rider inputs')
            if crank_reposition:
                command = replace(command, crank_reposition=True)
        except Exception as error:
            self.env.terminated = True
            self.env.reason = 'policy_error'
            self.env.error = f'{type(error).__name__}: {error}'
            raise
        if front_brake_demand or rear_brake_demand or crank_reposition:
            self.operator_events.append({
                'time_s': self.env.sim.time_s,
                'front_brake_demand': front_brake_demand,
                'rear_brake_demand': rear_brake_demand,
                'crank_reposition': bool(crank_reposition),
            })
            self.env.run_metadata['operator_intervention'] = True
        return self.env.step(command, front_brake_demand=front_brake_demand,
                             rear_brake_demand=rear_brake_demand)

    def reset(self, *, seed=None):
        observation = self.env.reset(seed=seed)
        self.operator_events.clear()
        self._start_policy()
        return observation

    def stop(self):
        if not self.env.done:
            self.env.truncated = True
            self.env.reason = 'operator_stop'

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
