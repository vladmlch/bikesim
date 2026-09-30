import inspect
import pytest
from bike_sim.validation.rider_replay import open_loop_schedule


def test_open_loop_uses_only_time_not_pitch_or_front_load():
    assert list(inspect.signature(open_loop_schedule).parameters)==['time_s']
    for t,value in [(0.,0.),(1.5,20.),(2.5,40.),(5.,0.)]:
        c=open_loop_schedule(t)
        assert c.motor_torque_nm==value
        assert c.human_torque_nm==0.


def test_reference_profile_keeps_physical_envelope_and_fixed_ratio():
    from bike_sim.physics.resolution import load_physics_config
    cfg=load_physics_config('examples/research/plant_reference_open_loop.toml')
    assert not cfg.pitch_assist
    assert not cfg.drive.shifting.enabled and not cfg.drive.pedaling.rollback_brake
    assert cfg.drive.gearing.front_teeth/cfg.drive.gearing.rear_teeth==pytest.approx(34/51)
    assert cfg.drive.assist.tau>0 and cfg.drive.assist.max_power>0

@pytest.mark.parametrize('ratio',[.5,34/51,.8])
def test_actual_shaft_and_ratio_are_not_applied_twice(ratio):
    from bike_sim.validation.plant_torque_rig import shaft_ratio_rig
    metrics,bounds=shaft_ratio_rig(.000625,ratio)
    assert metrics['delivered_crank_torque_nm']==pytest.approx(20.)
    assert all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())

def test_engine_freehub_transmits_nothing_on_overrun():
    from bike_sim.validation.plant_torque_rig import shaft_ratio_rig
    metrics,bounds=shaft_ratio_rig(.000625,overrun=True)
    assert all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())
