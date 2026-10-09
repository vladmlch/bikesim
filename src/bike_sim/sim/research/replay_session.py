"""Incremental same-backend replay, with verification separate from rendering.

No policy is loaded. A budget/target yield resumes the same recorded request.
Only final comparisons can publish a successful report; close never completes
unverified work. Failed comparisons freeze the last verified control boundary.
"""
from __future__ import annotations

from copy import copy, deepcopy
from dataclasses import asdict, replace
import math
from numbers import Real
from pathlib import Path

import numpy as np

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_view import environment_snapshot
from bike_sim.sim.research.replay import (
    _check_runtime, _compare, _json_read, _rebuild, _rows, _state_error,
    integration_state, validate_recording,
)


class ReplaySession:
    def __init__(self, directory):
        self.directory = Path(directory)
        self._closed = False
        self._generation = 0
        self._manifest = None
        self._env = None
        self._install(self._load())

    def _load(self):
        summary, manifest = validate_recording(self.directory)
        if self._manifest is not None and manifest != self._manifest:
            raise ValueError('recording changed since replay session construction')
        commands = _rows(self.directory/'commands_requested.jsonl')
        transitions = _rows(self.directory/'transitions.jsonl')
        observations = _rows(self.directory/'observations.jsonl')
        applied = _rows(self.directory/'commands_applied.jsonl')
        if len(commands) != len(transitions) or len(observations) != len(commands)+1:
            raise ValueError('inconsistent recorded command/transition counts')
        for index, command in enumerate(commands):
            if (not isinstance(command, dict) or set(command) != {
                    'step', 'time_s', 'control', 'front_brake_demand', 'rear_brake_demand'} or
                    type(command['step']) is not int or command['step'] < 0 or
                    not isinstance(command['control'], dict)):
                raise ValueError(f'command {index}: invalid requested-command record')
        with np.load(self.directory/'states.npz', allow_pickle=False) as states:
            final_state = states['final'].copy()
        metrics = (_json_read(self.directory/'episode_metrics.json')
                   if manifest['schema_version'] == 2 else None)
        env = _rebuild(self.directory, summary, manifest)
        try:
            if summary['research'].get('rider_command_recording') != 'program_inputs':
                env.rider_program = None
            # This is recorded descriptive data, never a policy import or call.
            env.run_metadata['operator_intervention'] = summary['research'].get(
                'run_metadata', {}).get('operator_intervention', False)
            _compare(plain(asdict(env.observation)), observations[0], 'initial observation')
            frame = environment_snapshot(env)
        except BaseException:
            env.close(discard_pending=True)
            raise
        return env, summary, manifest, commands, transitions, observations, applied, final_state, metrics, frame

    def _install(self, loaded):
        (self._env, self._summary, self._manifest, self._commands, self._transitions,
         self._observations, self._applied, self._final_state, self._episode_metrics, frame) = loaded
        self._index = self._applied_index = 0
        self._frame = self._last_verified = replace(frame, generation=self._generation)
        self._done, self._error, self._report = False, None, None

    @property
    def backend(self):
        return self._env.backend

    @property
    def timestep_s(self):
        return self._env.dt_s

    @property
    def step(self):
        return self._frame.step

    @property
    def time_s(self):
        return self._frame.time_s

    @property
    def done(self):
        return self._done

    @property
    def error(self):
        return self._error

    @property
    def pending(self):
        return False if self._closed else self._env.control_pending

    @property
    def track(self):
        return self._env.sim.track

    def _require_open(self):
        if self._closed:
            raise RuntimeError('replay session is closed')

    def snapshot(self):
        self._require_open()
        return self._frame

    def make_render_model(self):
        self._require_open()
        return (self._env.make_render_model() if self.backend == 'native'
                else copy(self._env.sim.model))

    def _compare_applied_prefix(self):
        actual = plain(self._env.commands_applied)
        if len(actual) > len(self._applied):
            raise ValueError('applied commands: replay produced extra events')
        for index in range(self._applied_index, len(actual)):
            _compare(actual[index], self._applied[index], f'applied commands[{index}]')
        self._applied_index = len(actual)

    def _finish(self):
        env, summary = self._env, self._summary
        self._compare_applied_prefix()
        if self._applied_index != len(self._applied):
            raise ValueError('applied commands: replay omitted recorded events')
        recorded_reason = summary['outcome']['reason']
        reason = None if recorded_reason == 'not_finished' else recorded_reason
        if reason == 'operator_stop' and not env.done:
            env.stop()
        if reason is not None and not env.done:
            raise ValueError('outcome: replay did not reach the recorded terminal condition')
        _compare(env.reason, reason, 'outcome.reason')
        for name, actual in (('steps', env.sim.steps), ('time_s', env.sim.time_s),
                             ('position_m', env.sim.position_m)):
            if name in summary['outcome']:
                _compare(actual, summary['outcome'][name], 'outcome.'+name)
        _compare(plain(env.tracker.metrics), summary['research']['metrics'], 'event metrics')
        from bike_sim.sim.ride.physical_session import physical_summary
        actual_summary = physical_summary(env.sim, {}, recorded_reason)
        _compare(actual_summary['outcome'], summary['outcome'], 'outcome')
        research = summary['research']
        for name in ('terminated', 'truncated', 'numerically_valid', 'max_energy_residual_ratio'):
            if name in research:
                _compare(getattr(env, name), research[name], 'research.'+name)
        if 'model_status' in research:
            _compare(plain(env.sim.physical.model_status.as_dict()), research['model_status'], 'model status')
        if self._episode_metrics is not None:
            from bike_sim.sim.research.metrics import episode_metrics
            _compare(plain(episode_metrics(env)), self._episode_metrics, 'episode metrics')
        final_error = _state_error(integration_state(env.sim), self._final_state, 'final integration state')
        # A successful replay cannot conceal a source/binary edit during playback.
        _check_runtime(summary)
        if self._manifest['schema_version'] == 2:
            from bike_sim.native.artifact import execution_provenance
            if execution_provenance(self.backend) != self._manifest['execution']:
                raise ValueError('replay execution identity changed during playback')
        report = dict(passed=True, backend=self.backend,
            replayed_control_steps=self._index, physics_steps=env.sim.steps,
            final_state_max_abs_error=final_error, outcome=env.reason,
            numerically_valid=env.numerically_valid, model_valid=env.model_valid,
            source_sha256=env.metadata['model_source_sha256'],
            scope='Deterministic software replay under the recorded source/runtime, not physical calibration.')
        frame = replace(environment_snapshot(env), generation=self._generation)
        self._frame = self._last_verified = frame
        self._report, self._done = report, True

    def advance(self, wall_budget_s=None, *, target_step=None):
        self._require_open()
        if wall_budget_s is not None and (isinstance(wall_budget_s, bool) or
                not isinstance(wall_budget_s, Real) or not math.isfinite(wall_budget_s) or wall_budget_s < 0.):
            raise ValueError('wall_budget_s must be finite and nonnegative')
        if target_step is not None and (type(target_step) is not int or target_step < self.step):
            raise ValueError('target_step must be an integer at or after the committed step')
        if self._error is not None:
            raise RuntimeError(f'replay stopped after mismatch: {self._error}')
        if self._done:
            return True
        try:
            env = self._env
            if self._index == len(self._commands):
                self._finish()
                return True
            if not env.control_pending:
                command = self._commands[self._index]
                if env.done or command['step'] != env.sim.steps:
                    raise ValueError(f'command {self._index}: episode ended early or step does not align')
                _compare(env.sim.time_s, command['time_s'], f'command {self._index} time')
                value = dict(command['control'])
                if value.get('posture') is not None:
                    value['posture'] = RiderPosture(**value['posture'])
                env.begin_control(RideControl(**value),
                    front_brake_demand=command['front_brake_demand'],
                    rear_brake_demand=command['rear_brake_demand'])
                _compare(plain(env.commands_requested[-1]), command, f'command {self._index} request')
            options = dict(wall_budget_s=wall_budget_s)
            if target_step is not None:
                options['target_step'] = target_step
            result = env.advance_control(**options)
            if result is None:
                self._frame = replace(environment_snapshot(env), generation=self._generation)
                return False
            index = self._index
            _compare(plain(asdict(result)), self._transitions[index], f'transition {index}')
            _compare(plain(asdict(env.observation)), self._observations[index+1], f'observation {index+1}')
            self._compare_applied_prefix()
            self._index += 1
            if self._index == len(self._commands):
                # Final-state/metrics/outcome checks belong to the last boundary.
                # Do not replace the verified checkpoint until they all pass.
                self._finish()
            else:
                self._frame = self._last_verified = replace(
                    environment_snapshot(env), generation=self._generation)
            return self._done
        except Exception as error:
            self._error = f'{type(error).__name__}: {error}'
            self._frame = self._last_verified
            raise

    def restart(self):
        self._require_open()
        try:
            loaded = self._load()
            try:
                self._env.close(discard_pending=True)
            except BaseException:
                loaded[0].close(discard_pending=True)
                raise
            self._generation += 1
            self._install(loaded)
            return self.snapshot()
        except Exception as error:
            self._done, self._report = False, None
            self._error = f'{type(error).__name__}: {error}'
            self._frame = self._last_verified
            raise

    def report(self):
        if not self._done or self._report is None:
            raise RuntimeError('incomplete replay: final verification has not passed')
        return deepcopy(self._report)

    def close(self):
        if not self._closed:
            # In particular, never simulate the remainder of an early-closed replay.
            self._env.close(discard_pending=True)
            self._closed = True
