"""The selected phase is an initial condition, not a post-equilibrium correction."""
import numpy as np
from bike_sim.physics.drivetrain import DrivetrainSpecs
from bike_sim.terrain import get_preset
from bike_sim.sim import ride_sim


def test_nonzero_phase_does_not_overwrite_solved_state(monkeypatch):
    solved={};original=ride_sim.solve_static_equilibrium
    def capture(*args,**kwargs):
        result=original(*args,**kwargs)
        solved['qpos']=args[1].qpos.copy();solved['qvel']=args[1].qvel.copy()
        return result
    monkeypatch.setattr(ride_sim,'solve_static_equilibrium',capture)
    sim=ride_sim.RideSimulation(track=get_preset('flat'),target_speed_kmh=15.,drive_mode='pedal',
        assist='off',drivetrain=DrivetrainSpecs(crank_phase_deg=45.),legs='articulated')
    np.testing.assert_array_equal(sim.data.qpos,solved['qpos'])
    np.testing.assert_array_equal(sim.data.qvel,solved['qvel'])
    assert sim.equilibrium['residual_qacc']<=.05


def test_legacy_output_directory_includes_drive_settings():
    from bike_sim.cli.ride import run_dir_name
    track=get_preset('flat')
    a=run_dir_name(track,25.,None,drive_settings={'mode':'pedelec','phase_deg':0.})
    b=run_dir_name(track,25.,None,drive_settings={'mode':'pedelec','phase_deg':90.})
    c=run_dir_name(track,25.,None,drive_settings={'mode':'pedal','phase_deg':0.})
    assert len({a,b,c})==3
