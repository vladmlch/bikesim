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


def test_pedal_forces_sum_to_commanded_torque(pedal_sim):
    """
    The two legs' tangential forces times the crank arm equal `rider_torque_nm` at
    every crank phase: `rider_torque_nm` is already instantaneous (mean x
    ripple_shape) and `crank_torque_share` is a share of the *mean*, so the force
    map must divide the shape back out -- otherwise the legs deliver mean x shape^2,
    applying the ripple twice.
    """
    m, d = pedal_sim.model, pedal_sim.data
    drive = pedal_sim.leg_drive
    crank_adr = drive.crank_qposadr
    phase0 = float(d.qpos[crank_adr])
    try:
        for phase in (0.0, 0.6, 1.3, np.pi / 2, 2.2, np.pi, 4.0, 5.5):
            d.qpos[crank_adr] = phase
            drive.apply(m, d, 37.0)
            delivered = pedal_sim.crank_length_m * (
                drive.pedal_force_front_n + drive.pedal_force_rear_n
            )
            assert delivered == pytest.approx(37.0, rel=1e-6), (
                f"phase {phase}: legs deliver {delivered:.2f} N.m for a 37.0 N.m command"
            )
        # Depth 1 reaches zero ripple shape at the dead centres: commanded torque there
        # is 0 and the shares are 0 too -- the guard must produce 0, not NaN.
        drive.ripple_depth = 1.0
        d.qpos[crank_adr] = np.pi / 2
        drive.apply(m, d, 0.0)
        assert drive.pedal_force_front_n == 0.0 and drive.pedal_force_rear_n == 0.0
    finally:
        drive.ripple_depth = DrivetrainSpecs().ripple_depth
        d.qpos[crank_adr] = phase0


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


def test_visual_pedalling_drags_crank_and_legs():
    """
    `motor` + `visual_pedalling` is a shipped CLI mode with no `PedalDrivetrain`: the
    wheel actuator drives, the `chain_drive` equality drags `crank_spin` off the driven
    wheel, and the leg drive's zero-torque `apply` makes the welded feet follow the
    pedals. Exercises the `elif self.leg_drive.active:` reset branch and the motor-mode
    `leg_drive.apply(0.0)` path, which no other test touches.
    """
    sim = RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0,
        drive_mode="motor", visual_pedalling=True,
    )
    m, d = sim.model, sim.data
    assert sim.drivetrain is None  # motor mode: no pedalled drivetrain at all
    assert sim.leg_drive.active
    crank_dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "crank_spin")]
    wheel_dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rear_wheel_spin")]
    for _ in range(2000):  # 1 s
        sim.step()
    crank_radps = float(d.qvel[crank_dof])
    wheel_radps = float(d.qvel[wheel_dof])
    # The chain equality makes the crank run at wheel speed over the gear ratio; a
    # nonzero crank qvel is the difference between "dragged" and "parked".
    assert crank_radps > 0.5
    assert crank_radps == pytest.approx(
        wheel_radps / sim.drivetrain_specs.gear_ratio, rel=0.02
    )
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        sep = float(np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]))
        assert sep < 0.002, f"{side}: foot-pedal separation {sep * 1000:.1f} mm"
