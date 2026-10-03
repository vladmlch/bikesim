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


def test_accounted_hud_displays_realtime_factor_and_first_failure():
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
    assert 'foot_front.friction' in text and '0.001' in text


def test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush(tmp_path,monkeypatch):
    import json
    from pathlib import Path
    from tools.measure_realtime import measure_realtime
    from bike_sim.cli import ride as ride_cli
    recorded = []
    monitor = SimpleNamespace(strict=True, first_failure=None)
    physical = SimpleNamespace(control_clock=SimpleNamespace(period_s=.005),
        set_record_decimation=recorded.append, reference_monitor=monitor,
        model_status=SimpleNamespace(as_dict=lambda: {'model_valid':True}),
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
    assert report['wall_seconds'] >= report['flush_wall_seconds'] >= 0.
    assert report['factor'] == report['sim_seconds']/report['wall_seconds']
    assert recorded == [80,'flush'] and monitor.strict is False
    assert json.loads((tmp_path/'realtime.json').read_text()) == report
