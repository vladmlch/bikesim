import numpy as np
import pytest
import mujoco

from bike_sim.physics.drivetrain import DrivetrainSpecs
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


@pytest.fixture(scope="module")
def pedal_sim():
    return RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0,
        drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(), legs="articulated",
    )


def test_leg_drive_active(pedal_sim):
    assert pedal_sim.leg_drive is not None and pedal_sim.leg_drive.active


def test_weld_residual_near_zero(pedal_sim):
    for _ in range(4000):  # 2 s
        pedal_sim.step()
    m, d = pedal_sim.model, pedal_sim.data
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        sep = float(np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]))
        assert sep < 0.005, f"{side}: foot-pedal separation {sep*1000:.1f} mm"


def test_reset_at_nonzero_phase_consistent():
    sim = RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0, drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(crank_phase_deg=90.0), legs="articulated",
    )
    d = sim.data
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        assert np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]) < 0.002


def test_rigid_legs_still_pedal():
    sim = RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0, drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(), legs="rigid",
    )
    # The rider ceiling is 60 N.m at the crank -- ~26 N.m at the wheel through 32x14 --
    # so ~95 kg of bike+rider needs ~6 s to pass 3 m/s. Measured today: 1.79 m/s at 3 s,
    # 3.3 m/s at 6 s; the 7 s horizon keeps the same check with margin.
    for _ in range(14000):
        sim.step()
    assert sim.speed_mps > 3.0  # today's behaviour preserved
