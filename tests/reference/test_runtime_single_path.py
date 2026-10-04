import inspect

import pytest
from types import SimpleNamespace
import numpy as np

from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
from bike_sim.sim.ride import viewer


def test_runtime_has_one_accounted_step_and_viewer_uses_it():
    assert not hasattr(PhysicalRuntime, '_step_preview')
    assert '_step_preview' not in inspect.getsource(PhysicalRuntime.step)
    assert 'preview_mode()' not in inspect.getsource(viewer.run_physical_viewer)


def test_preview_mode_cannot_select_an_unaccounted_path():
    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    with pytest.raises(RuntimeError, match='preview path removed'):
        with runtime.preview_mode():
            pass


def test_accounted_hud_displays_realtime_factor_without_repeated_failure(capsys):
    from bike_sim.sim.ride.hud import RideHUD
    hud = RideHUD.__new__(RideHUD)
    sample = SimpleNamespace(time_s=.005, qvel=np.array([1.]), channels={
        'tires':{'front':{'normal_load_n':100.}, 'rear':{'normal_load_n':200.}},
        'drive':{}, 'energy':{'residual_j':.1}, 'model_status':{'model_valid':False}})
    sim = SimpleNamespace(physics_config=SimpleNamespace(drive_mode='coast'), root_x_dofadr=0,
        physical=SimpleNamespace(sample=sample, interactive_preview=False,live_real_time_factor=.75,
            reference_monitor=SimpleNamespace(first_failure=(.001,('foot_front.friction',)))))
    text = hud.line(sim)
    assert 'RTF=0.75x' in text
    assert 'foot_front.friction' not in text
    hud.print_line(sim); hud.print_line(sim)
    out = capsys.readouterr().out
    assert out.count('first failure at t=0.001000s: foot_front.friction') == 1


def test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush(tmp_path,monkeypatch):
    import json
    from pathlib import Path
    from tools.measure_realtime import measure_realtime
    from bike_sim.cli import ride as ride_cli
    recorded = []
    monitor = SimpleNamespace(strict=True, first_failure=None)
    physical = SimpleNamespace(control_clock=SimpleNamespace(period_s=.005),
        set_record_decimation=recorded.append, reference_monitor=monitor,
        model_status=SimpleNamespace(as_dict=lambda: {'model_valid':True,'numerically_valid':False,
                                                     'calibration_status':'synthetic'}),
        flush=lambda: recorded.append('flush'))
    sim = SimpleNamespace(model=SimpleNamespace(opt=SimpleNamespace(timestep=.0005)),
        physical=physical,crash=None,position_m=0.,track=SimpleNamespace(length_m=100.),steps=0)
    def step():
        sim.steps+=1
    sim.step=step
    def build(args):
        assert args.resolved_physics.timestep_s == .0005 and args.decimate == 80
        return sim
    monkeypatch.setattr(ride_cli,'build_physical_simulation_from_args',build)
    root=Path(__file__).resolve().parents[2]
    report=measure_realtime(str(root/'examples/research/rough_uphill_extreme.toml'),
        str(root/'examples/research/viewer_physics_welded.toml'),.0015,out=tmp_path)
    assert sim.steps == report['steps'] == 3
    assert report['sim_seconds'] == .0015
    assert report['effective_timestep_s'] == .0005 and report['source_timestep_s'] == .00125
    assert report['controller_interval_s'] == .005 and report['record_decimation'] == 80
    assert report['model_valid'] is True and report['numerically_valid'] is False
    assert report['model_status']['calibration_status'] == 'synthetic'
    assert report['wall_seconds'] >= report['flush_wall_seconds'] >= 0.
    assert report['factor'] == report['sim_seconds']/report['wall_seconds']
    assert recorded == [80,'flush'] and monitor.strict is False
    assert json.loads((tmp_path/'realtime.json').read_text()) == report


def _fake_viewer_run(monkeypatch, tmp_path, *, fail=False, cleanup_fail=False):
    import itertools
    from contextlib import nullcontext
    import mujoco.viewer
    from bike_sim.sim import playground
    from bike_sim.sim.ride import physical_session
    from bike_sim.viz import ride_plots
    state=SimpleNamespace(open=False,pending=0,flushes=0,events=[])
    runtime=SimpleNamespace(generation=0,live_real_time_factor=None,sample=None,
        history=SimpleNamespace(duration_s=0.),reference_monitor=SimpleNamespace(first_failure=None),
        set_strict=lambda strict: state.events.append(('strict', strict)))
    def flush():
        state.flushes+=1
        if state.pending:
            runtime.history.duration_s=.0005
            runtime.sample=SimpleNamespace(interval_id=0)
            runtime.reference_monitor.first_failure=(.0005,('terminal.violation',))
            state.pending=0
        state.events.append(('flush',state.open))
        if cleanup_fail:
            raise RuntimeError('tail cleanup failed')
    runtime.flush=flush
    sim=SimpleNamespace(physical=runtime,model=SimpleNamespace(opt=SimpleNamespace(timestep=.0005)),
        data=SimpleNamespace(xpos=np.zeros((1,3))),track=SimpleNamespace(name='fake'),
        steps=0,time_s=0.,position_m=0.,_frame_body_id=0)
    def check_tail(event):
        assert runtime.sample is not None
        assert runtime.history.duration_s == .0005
        assert runtime.reference_monitor.first_failure == (.0005,('terminal.violation',))
        state.events.append((event,state.open))
    class Window:
        def __init__(self):self.running=iter((True,False))
        def __enter__(self):state.open=True;return self
        def __exit__(self,*args):state.open=False
        def is_running(self):return next(self.running)
        def lock(self):return nullcontext()
        def sync(self):pass
    class Log:
        def __init__(self,*args):pass
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def due(self,*args):return True
        def write(self,*args):check_tail('live_csv')
        def write_marker(self,*args):check_tail('marker')
    def describe():
        check_tail('announcement')
        return 'step cap'
    session=SimpleNamespace(outcome=None,handle_key=lambda key:None,
        camera=SimpleNamespace(reset_preset=lambda:None,update_viewer=lambda *a,**k:None),
        process_pending_keys=lambda:None,show_telemetry=True,
        print_hud=lambda:check_tail('hud'),
        hud=SimpleNamespace(preview_log_row=lambda sim:check_tail('row') or {}))
    def step():
        sim.steps=1;sim.time_s=.0005;state.pending=1
        if fail:raise RuntimeError('physics step failed')
        session.outcome=SimpleNamespace(describe=describe)
        return session.outcome
    session.step=step
    monkeypatch.setattr(viewer,'RideSession',lambda sim:session)
    monkeypatch.setattr(viewer,'RealTimePacer',lambda dt:SimpleNamespace(steps_for=lambda delta:1,reset=lambda:None))
    monkeypatch.setattr(playground,'ensure_macos_mjpython',lambda:None)
    monkeypatch.setattr(physical_session,'configuration_metadata',lambda sim:{})
    monkeypatch.setattr(physical_session,'physical_run_dir_name',lambda *a:'fake')
    monkeypatch.setattr(physical_session,'PhysicalLiveCsv',Log)
    monkeypatch.setattr(mujoco.viewer,'launch_passive',lambda *a,**k:Window())
    monkeypatch.setattr(ride_plots,'load_ride_csv',lambda path:{})
    monkeypatch.setattr(ride_plots,'plot_physical_ride_html',lambda *args:'fake.html')
    times=itertools.count(0.,.6)
    monkeypatch.setattr(viewer.time,'monotonic',lambda:next(times))
    monkeypatch.setattr(viewer.time,'sleep',lambda delay:None)
    return sim,state


def test_terminal_viewer_tail_is_accounted_before_announcement_hud_and_live_csv(monkeypatch,tmp_path):
    sim,state=_fake_viewer_run(monkeypatch,tmp_path)
    assert viewer.run_physical_viewer(sim,out_root=tmp_path) == 0
    assert state.events[0] == ('strict', False)
    assert ('flush',True) in state.events
    for event in ('announcement','hud','live_csv'):
        assert (event,True) in state.events
    assert state.pending == 0 and not state.open


@pytest.mark.parametrize('cleanup_fail',[False,True])
def test_exceptional_viewer_exit_flushes_tail_without_masking_original_error(monkeypatch,tmp_path,cleanup_fail):
    sim,state=_fake_viewer_run(monkeypatch,tmp_path,fail=True,cleanup_fail=cleanup_fail)
    with pytest.raises(RuntimeError,match='physics step failed') as failure:
        viewer.run_physical_viewer(sim,out_root=tmp_path)
    assert state.flushes >= 1 and state.pending == 0 and not state.open
    if cleanup_fail:
        assert any('tail cleanup failed' in note for note in failure.value.__notes__)
