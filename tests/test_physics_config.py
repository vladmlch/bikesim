"""Configuration and orchestration boundaries for physical ride mode."""

import mujoco
import pytest

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_physical_defaults_to_coast_without_external_pitch():
    cfg = SimulationPhysicsConfig(physics_mode="physical")
    assert cfg.drive_mode == "coast"
    assert cfg.pitch_assist is False


def test_external_pitch_is_rejected_in_physical():
    with pytest.raises(ValueError, match="pitch"):
        SimulationPhysicsConfig(physics_mode="physical", pitch_assist=True)


@pytest.mark.parametrize("mode", ["crank_effort", "articulated_effort"])
def test_effort_modes_cannot_silently_use_legacy_cruise(mode):
    with pytest.raises(NotImplementedError, match=mode):
        RideSimulation(physics_config=SimulationPhysicsConfig("physical", drive_mode=mode))


def test_invalid_physics_config_is_rejected():
    with pytest.raises(ValueError, match="physics_mode"):
        SimulationPhysicsConfig(physics_mode="unknown")
    with pytest.raises(ValueError, match="drive_mode"):
        SimulationPhysicsConfig(drive_mode="unknown")
    with pytest.raises(ValueError, match="timestep"):
        SimulationPhysicsConfig(timestep_s=0.0)


def test_explicit_timestep_reaches_ride_model():
    cfg = SimulationPhysicsConfig(physics_mode="physical", timestep_s=0.001)
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(mode="ride", rider="none", physics_config=cfg)
    )
    assert model.opt.timestep == pytest.approx(0.001)


def test_physical_coast_clears_drive_and_root_pitch_every_step():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    root_z_joint = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "root_z")
    sim.data.qpos[sim.model.jnt_qposadr[root_z_joint]] += 1.0
    sim.data.qpos[sim.root_pitch_qposadr] = 0.2
    sim.contact_query.reset()

    sim.data.ctrl[sim.drive_ctrl_adr] = 100.0
    sim.data.qfrc_applied[sim.stabilizer.pitch_dofadr] = 30.0
    sim.step()
    assert sim.contacts.airborne
    assert sim.data.ctrl[sim.drive_ctrl_adr] == 0.0
    assert sim.data.qfrc_applied[sim.stabilizer.pitch_dofadr] == 0.0
    assert sim.stabilizer.angular_impulse_nms == 0.0
    assert sim.cruise.torque_nm == 0.0


def test_implicit_legacy_and_physical_have_distinct_revisions():
    legacy = RideSimulation(track=get_preset("flat"), rider="none")
    physical = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    assert legacy.physics_config.physics_mode == "legacy"
    assert legacy.physics_revision != physical.physics_revision
    legacy_shock = legacy.controller.suspension_system.shock_damper
    physical_shock = physical.controller.suspension_system.shock_damper
    legacy_shock.set_clicks(lockout=True)
    physical_shock.set_clicks(lockout=True)
    assert legacy_shock.compute_damping_force(0.03, 20.0) == pytest.approx(480.0)
    assert legacy_shock.compute_damping_force(1.0, 65.0) == pytest.approx(
        legacy_shock.compute_damping_force(1.0, 20.0)
    )
    assert physical_shock.compute_damping_force(0.03, 20.0) == pytest.approx(450.0)
    assert physical_shock.compute_damping_force(1.0, 65.0) > physical_shock.compute_damping_force(1.0, 20.0)


def test_explicit_legacy_coast_disables_cruise_and_pitch_help():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(
            physics_mode="legacy", drive_mode="coast", pitch_assist=False
        ),
    )
    sim.data.ctrl[sim.drive_ctrl_adr] = 100.0
    sim.data.qfrc_applied[sim.stabilizer.pitch_dofadr] = 30.0
    sim.step()
    assert sim.data.ctrl[sim.drive_ctrl_adr] == 0.0
    assert sim.data.qfrc_applied[sim.stabilizer.pitch_dofadr] == 0.0
    assert sim.cruise.torque_nm == 0.0


def test_physical_pneumatic_reads_current_wheel_geometry():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        tyre=TyreConfig(model="pneumatic"),
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    z_joint = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "root_z")
    sim.data.qpos[sim.model.jnt_qposadr[z_joint]] += 1.0
    sim.step()
    assert sim.tyre_applier.front_outputs.normal_load_n == 0.0
    assert sim.tyre_applier.rear_outputs.normal_load_n == 0.0


def test_physical_detailed_tyre_rejects_incompatible_explicit_timestep():
    with pytest.raises(ValueError, match="detailed.*timestep_s"):
        RideSimulation(
            track=get_preset("flat"),
            rider="none",
            tyre=TyreConfig(model="pneumatic", tier="detailed"),
            physics_config=SimulationPhysicsConfig(
                physics_mode="physical", timestep_s=0.0005
            ),
        )


def test_physical_detailed_tyre_keeps_matching_explicit_timestep():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        tyre=TyreConfig(model="pneumatic", tier="detailed"),
        physics_config=SimulationPhysicsConfig(
            physics_mode="physical", timestep_s=0.00025
        ),
    )
    assert sim.model.opt.timestep == pytest.approx(0.00025)
