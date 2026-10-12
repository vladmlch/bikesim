"""Fake-window source tests: no OpenGL window and no physics are needed."""
from types import SimpleNamespace
import numpy as np
import pytest
from bike_sim.native.contracts import FrameSnapshot, PhysicalViewState
from bike_sim.recording.contracts import EpisodeEvidence, TableBatch
from bike_sim.sim.ride.physical_frontend import make_run_capture, run_viewer
from bike_sim.sim.research.replay_viewer import run_replay_viewer


def frame(step=0, generation=1):
    return FrameSnapshot(generation=generation, step=step, time_s=step*.00125,
        integration_state=np.array([step*.00125]),
        view=PhysicalViewState(endpoint={'position_m': 0., 'z_m': 1.},
            preview_row={'time_s': step*.00125, 'x_m': step*.5}),
        model_status={'model_valid': True})


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


class FakeEpisode:
    """Duck-typed OwnedNativeEpisode rows for the fake driver."""
    def __init__(self, generation, rows=()):
        self.generation = generation
        self._rows = list(rows)
    @property
    def row_count(self):
        return len(self._rows)
    def batch(self, start, limit):
        return SimpleNamespace(samples=self._rows[int(start):int(start)+int(limit)])


class FakePhysical:
    """Duck-typed PhysicalSource carrying its episode for the deriver."""
    def __init__(self, episode):
        self.episode = episode
    @property
    def row_count(self):
        return self.episode.row_count
    def tables(self, *, batch_rows):
        yield TableBatch('fake', {'interval_id': np.zeros(0)}, 0)


def row(interval_id, *, contact=False, brake=0.):
    """One minimal channel row for derived-transition coverage."""
    return SimpleNamespace(
        interval_id=interval_id, time_s=interval_id*.00125,
        end_time_s=(interval_id+1)*.00125,
        channels={'tires': {'front': {'raw_contact': contact},
                            'rear': {'raw_contact': contact}},
                  'drive': {'front_brake_demand': brake,
                            'rear_brake_demand': brake}})


class Driver:
    dt_s = .00125
    backend = 'native'
    strict = False
    def __init__(self, clock):
        self.clock = clock
        self.step, self.time_s, self.generation = 0, 0., 1
        self.reason = None
        self.closed = False
        self.metadata = {'drive_mode': 'articulated_effort', 'configuration_sha256': '0'*64}
        self.track = SimpleNamespace(name='fake')
        self.events = []
        self.episode_rows = [[]]
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
    def summary(self, reason=None):
        self.events.append(('summary', self.generation))
        return {'outcome': {'reason': reason}, 'model_status': {'model_valid': True},
                'first_failure': None}
    def seal_episode(self, episode_id, *, reason):
        self.events.append(('seal', self.generation, episode_id))
        rows = self.episode_rows[min(self.generation-1, len(self.episode_rows)-1)]
        episode = FakeEpisode(self.generation, rows)
        return EpisodeEvidence(
            episode_id=episode_id, generation=self.generation,
            physical=FakePhysical(episode),
            summary={'outcome': {'reason': reason}, 'first_failure': None,
                     'model_status': {'model_valid': True}},
            assets=())
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


def enabled_capture(driver, published):
    def publisher(capture, out_root):
        published.append((capture, out_root))
        from pathlib import Path
        path = Path(out_root)/'recording.marker'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('published')
        return path
    return make_run_capture(driver, enabled=True, mode='viewer',
                            publisher=publisher)


def test_physical_window_bounds_rendering_and_leaves_no_files_when_off(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    presenters = patch_window(monkeypatch, frontend, clock,
                              {1: [296], 15: [ord('R')], 30: [297], 60: [ord('Q')]})
    driver = Driver(clock)
    assert run_viewer(driver, out_root=tmp_path) == 0
    assert driver.closed and driver.dt_s == .00125
    # The reset seals then resets; the terminal generation seals once more.
    assert driver.events == [('flush', 1), ('summary', 1), ('reset', 1),
                             ('flush', 2), ('summary', 2)]
    stamps = presenters[0].sync_times
    assert len(stamps) > 1
    assert all(b-a >= 1/60-1e-12 for a, b in zip(stamps, stamps[1:]))
    # Recording off: no preview, report or per-generation export files.
    assert list(tmp_path.rglob('*')) == []


def test_physical_window_recording_publishes_once_and_writes_preview(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {10: [ord('R')], 30: [ord('Q')]})
    driver = Driver(clock)
    published = []
    assert run_viewer(driver, out_root=tmp_path,
                      capture=enabled_capture(driver, published)) == 0
    assert driver.closed
    assert driver.events == [('seal', 1, 0), ('reset', 1), ('seal', 2, 1)]
    assert len(published) == 1
    capture, out_root = published[0]
    episodes = list(capture.iter_episodes())
    assert [e.episode_id for e in episodes] == [0, 1]
    assert [e.generation for e in episodes] == [1, 2]
    previews = list(tmp_path.rglob('preview.csv'))
    assert len(previews) == 1
    text = previews[0].read_text()
    assert 'requested_scale' in text and 'reset' in text
    # The interactive route renders ride.html from the recorded preview rows.
    html = list(tmp_path.rglob('ride.html'))
    assert len(html) == 1 and html[0].parent == previews[0].parent
    assert list(tmp_path.rglob('recording.marker'))


def test_physical_window_skips_ride_html_when_plotly_is_absent(tmp_path, monkeypatch, capsys):
    import builtins
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {2: [ord('Q')]})
    original_import = builtins.__import__
    def no_plotly(name, *args, **kwargs):
        if name == 'bike_sim.viz.ride_plots' or name == 'plotly.graph_objects':
            raise ImportError('plotly not installed')
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', no_plotly)
    driver = Driver(clock)
    assert run_viewer(driver, out_root=tmp_path,
                      capture=enabled_capture(driver, [])) == 0
    assert driver.closed
    assert list(tmp_path.rglob('preview.csv'))
    assert not list(tmp_path.rglob('ride.html'))
    assert 'skipped ride.html' in capsys.readouterr().out


def test_window_construction_error_still_accounts_and_closes(tmp_path, monkeypatch):
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
    assert driver.events == [('flush', 1), ('summary', 1)]


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
    assert driver.closed and driver.events == [('flush', 1), ('summary', 1)]
    with pytest.raises(ImportError, match='viewer unavailable'):
        run_replay_viewer(replay)
    assert replay.closed and replay.report_calls == 0
