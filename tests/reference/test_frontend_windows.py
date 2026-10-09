"""Fake-window source tests: no OpenGL window and no physics are needed."""
from types import SimpleNamespace
import numpy as np
import pytest
from bike_sim.native.contracts import FrameSnapshot, PhysicalViewState
from bike_sim.sim.ride.physical_frontend import run_viewer
from bike_sim.sim.research.replay_viewer import run_replay_viewer


def frame(step=0, generation=1):
    return FrameSnapshot(generation=generation, step=step, time_s=step*.00125,
        integration_state=np.array([step*.00125]),
        view=PhysicalViewState(endpoint={'position_m': 0., 'z_m': 1.},
            preview_row={'time_s': step*.00125}), model_status={'model_valid': True})


class Clock:
    def __init__(self):
        self.now = 0.
    def monotonic(self):
        return self.now
    def sleep(self, dt):
        self.now += dt


class Window:
    def __init__(self, callback, keys):
        self.callback, self.keys = callback, keys
        self.calls = 0
    def is_running(self):
        self.calls += 1
        for key in self.keys.get(self.calls, ()):
            self.callback(key)
        return self.calls < 200
    def __enter__(self):
        return self
    def __exit__(self, *args):
        pass


class Presenter:
    def __init__(self, model, clock):
        self.clock = clock
        self.sync_times = []
        self.replica = SimpleNamespace(model=model, data=object(), frame=None, apply=lambda frame: None)
        self.camera = SimpleNamespace(reset_preset=lambda: None)
        self.hud = SimpleNamespace(preview_log_row=lambda view: dict(view.preview_row))
    def handle_key(self, key, viewer):
        return True
    def sync(self, viewer, frame):
        self.sync_times.append(self.clock.now)
    def print_hud(self, *args, **kwargs):
        pass


class Driver:
    dt_s = .00125
    backend = 'native'
    def __init__(self, clock):
        self.clock = clock
        self.step, self.time_s, self.generation = 0, 0., 1
        self.reason = None
        self.closed = False
        self.metadata = {'drive_mode': 'articulated_effort', 'configuration_sha256': '0'*64}
        self.track = SimpleNamespace(name='fake')
        self.events = []
    def snapshot(self):
        return frame(self.step, self.generation)
    def make_render_model(self):
        return object()
    def start(self):
        pass
    def advance(self, target, **kwargs):
        assert kwargs['wall_budget_s'] == .008
        self.clock.now += .003
        self.step = min(target, self.step+2)
        self.time_s = self.step*self.dt_s
    def flush(self):
        self.events.append(('flush', self.generation))
    def export(self, destination, *, reason):
        self.events.append(('export', self.generation))
        return {'outcome': {'reason': reason}, 'model_status': {'model_valid': True}}
    def reset(self):
        self.events.append(('reset', self.generation))
        self.generation += 1
        self.step, self.time_s, self.reason = 0, 0., None
        return self.snapshot()
    def close(self):
        self.closed = True


def patch_window(monkeypatch, module, clock, keys):
    import mujoco.viewer
    holder = []
    def presenter(model):
        result = Presenter(model, clock)
        holder.append(result)
        return result
    monkeypatch.setattr(module, 'FramePresenter', presenter)
    monkeypatch.setattr(module.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(module.time, 'sleep', clock.sleep)
    monkeypatch.setattr(mujoco.viewer, 'launch_passive',
        lambda model, data, *, key_callback, **kwargs: Window(key_callback, keys))
    return holder


def test_physical_window_bounds_rendering_and_exports_reset_generation(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    presenters = patch_window(monkeypatch, frontend, clock,
                              {1: [296], 15: [ord('R')], 30: [297], 60: [ord('Q')]})
    driver = Driver(clock)
    assert run_viewer(driver, out_root=tmp_path) == 0
    assert driver.closed and driver.dt_s == .00125
    assert driver.events[:3] == [('flush', 1), ('export', 1), ('reset', 1)]
    assert ('export', 2) in driver.events
    stamps = presenters[0].sync_times
    assert len(stamps) > 1
    assert all(b-a >= 1/60-1e-12 for a, b in zip(stamps, stamps[1:]))
    previews = list(tmp_path.rglob('preview.csv'))
    assert len(previews) == 1
    text = previews[0].read_text()
    assert 'requested_scale' in text and 'reset' in text


def test_window_construction_error_still_exports_and_closes(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    import mujoco.viewer
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {})
    def fail(*args, **kwargs):
        raise RuntimeError('window construction failed')
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', fail)
    driver = Driver(clock)
    with pytest.raises(RuntimeError, match='window construction'):
        run_viewer(driver, out_root=tmp_path)
    assert driver.closed
    assert driver.events == [('flush', 1), ('export', 1)]


class Replay:
    timestep_s = .00125
    backend = 'native'
    step, time_s = 0, 0.
    done, error, track = False, None, None
    def __init__(self):
        self.closed, self.report_calls = False, 0
    def snapshot(self):
        return frame(self.step)
    def make_render_model(self):
        return object()
    def advance(self, budget, *, target_step):
        self.step += 1
        self.time_s = self.step*self.timestep_s
        return False
    def report(self):
        self.report_calls += 1
        raise AssertionError('incomplete replay requested a success report')
    def close(self):
        self.closed = True


def test_replay_window_close_is_incomplete_not_success(monkeypatch):
    import bike_sim.sim.research.replay_viewer as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {20: [ord('Q')]})
    session = Replay()
    assert run_replay_viewer(session) == 2
    assert session.closed and session.report_calls == 0


def test_viewer_import_failure_still_closes_physical_and_replay_owners(tmp_path, monkeypatch):
    import builtins
    original_import = builtins.__import__
    def fail_viewer(name, *args, **kwargs):
        if name == 'mujoco.viewer':
            raise ImportError('viewer unavailable')
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', fail_viewer)
    driver, replay = Driver(Clock()), Replay()
    with pytest.raises(ImportError, match='viewer unavailable'):
        run_viewer(driver, out_root=tmp_path)
    assert driver.closed and driver.events == [('flush', 1), ('export', 1)]
    with pytest.raises(ImportError, match='viewer unavailable'):
        run_replay_viewer(replay)
    assert replay.closed and replay.report_calls == 0
