"""
Unit tests for 12-speed cassette and adaptive cadence auto-shifting.

Tests include:
- Low cadence under positive pedal demand triggers downshift (cog teeth increase).
- High cadence triggers upshift (cog teeth decrease).
- Shifting re-datums chain equality with zero residual error.
- Fixed gearing (auto_shift=False) preserves the specified single gear.
- Shift torque attenuation occurs during the shift cut window.
"""

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.drivetrain import (
    CASSETTE_12S_TEETH,
    RPM_PER_RADPS,
    DrivetrainSpecs,
)
from bike_sim.sim.ride.drivetrain import PedalDrivetrain


@pytest.fixture
def ride_model_data():
    xml = generate_mujoco_xml(mode="ride", include_rider=True, crank_joint=True)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    return model, data


def test_drivetrain_downshifts_at_low_cadence(ride_model_data):
    model, data = ride_model_data
    specs = DrivetrainSpecs(cog_teeth=14, auto_shift=True)
    dt = PedalDrivetrain(model, specs=specs, drive_mode="pedelec", assist_mode="turbo")
    dt.reset(model, data)

    # Set crank velocity to 50 RPM (< target_cadence_min_rpm = 65 RPM)
    data.qvel[dt.crank_dofadr] = 50.0 / RPM_PER_RADPS
    # Set wheel velocity matching current ratio (32/14)
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 14.0)

    cmd = dt.compute(model, data, wheel_demand_nm=30.0, speed_mps=5.0, rear_in_contact=True)

    # Should have downshifted to the next larger cog in CASSETTE_12S_TEETH: 16T
    assert dt.current_cog == 16
    assert cmd.gear_teeth == 16
    expected_ratio = 32.0 / 16.0
    assert model.eq_data[dt.eq_id, 1] == pytest.approx(1.0 / expected_ratio)


def test_drivetrain_upshifts_at_high_cadence(ride_model_data):
    model, data = ride_model_data
    specs = DrivetrainSpecs(cog_teeth=16, auto_shift=True)
    dt = PedalDrivetrain(model, specs=specs, drive_mode="pedelec", assist_mode="turbo")
    dt.reset(model, data)

    # Set crank velocity to 95 RPM (> target_cadence_max_rpm = 85 RPM)
    data.qvel[dt.crank_dofadr] = 95.0 / RPM_PER_RADPS
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 16.0)

    cmd = dt.compute(model, data, wheel_demand_nm=30.0, speed_mps=8.0, rear_in_contact=True)

    # Should have upshifted to the next smaller cog in CASSETTE_12S_TEETH: 14T
    assert dt.current_cog == 14
    assert cmd.gear_teeth == 14
    expected_ratio = 32.0 / 14.0
    assert model.eq_data[dt.eq_id, 1] == pytest.approx(1.0 / expected_ratio)


def test_shifting_redatums_chain_equality_with_zero_residual(ride_model_data):
    model, data = ride_model_data
    specs = DrivetrainSpecs(cog_teeth=14, auto_shift=True)
    dt = PedalDrivetrain(model, specs=specs, drive_mode="pedelec", assist_mode="turbo")
    dt.reset(model, data)

    # Move crank and wheel positions
    data.qpos[dt.crank_qposadr] = 1.5
    data.qpos[dt.wheel_qposadr] = 1.5 * (32.0 / 14.0)
    data.qvel[dt.crank_dofadr] = 40.0 / RPM_PER_RADPS
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 14.0)

    # Trigger shift
    dt.compute(model, data, wheel_demand_nm=30.0, speed_mps=4.0, rear_in_contact=True)

    # Chain residual should be virtually zero
    assert dt.chain_residual(model, data) == pytest.approx(0.0, abs=1e-6)
    # Crank velocity is updated to match wheel through the new ratio
    new_gear_ratio = 32.0 / dt.current_cog
    assert data.qvel[dt.crank_dofadr] == pytest.approx(float(data.qvel[dt.wheel_dofadr]) / new_gear_ratio)


def test_fixed_gearing_disables_autoshift(ride_model_data):
    model, data = ride_model_data
    specs = DrivetrainSpecs(cog_teeth=14, auto_shift=False)
    dt = PedalDrivetrain(model, specs=specs, drive_mode="pedelec", assist_mode="turbo")
    dt.reset(model, data)

    # Very low cadence that would normally trigger downshift
    data.qvel[dt.crank_dofadr] = 20.0 / RPM_PER_RADPS
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 14.0)

    cmd = dt.compute(model, data, wheel_demand_nm=30.0, speed_mps=2.0, rear_in_contact=True)

    assert dt.current_cog == 14
    assert cmd.gear_teeth == 14


def test_shift_torque_cut_attenuates_torque(ride_model_data):
    model, data = ride_model_data
    specs = DrivetrainSpecs(cog_teeth=14, auto_shift=True, ripple_depth=0.0)
    dt = PedalDrivetrain(model, specs=specs, drive_mode="pedal", assist_mode="off")
    dt.reset(model, data)

    # Establish baseline torque at 75 RPM (in band)
    data.qvel[dt.crank_dofadr] = 75.0 / RPM_PER_RADPS
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 14.0)
    normal_cmd = dt.compute(model, data, wheel_demand_nm=20.0, speed_mps=5.0, rear_in_contact=True)

    # Drop cadence to 50 RPM to trigger shift
    data.qvel[dt.crank_dofadr] = 50.0 / RPM_PER_RADPS
    data.qvel[dt.wheel_dofadr] = data.qvel[dt.crank_dofadr] * (32.0 / 14.0)
    shift_cmd = dt.compute(model, data, wheel_demand_nm=20.0, speed_mps=5.0, rear_in_contact=True)

    # Shift cut is active immediately after shift
    assert dt.shift_cut_timer_s > 0.0
    # Torque should be attenuated by 70% (i.e. roughly 30% of normal torque scaled by ratio)
    assert shift_cmd.rider_torque_nm < 0.5 * normal_cmd.rider_torque_nm
