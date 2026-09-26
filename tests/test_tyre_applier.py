"""Composition of carcass and brush kernels at the MuJoCo wheel boundary."""

import ast
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.tyre import (
    FRONT_TYRE,
    REAR_TYRE,
    TIERS,
    TyreConfig,
    TyreSpecs,
    crr_target,
)
from bike_sim.sim.ride.tyre.applier import TyreForceApplier
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride.tyre.model import PneumaticTyre
from bike_sim.terrain.surface import SurfaceMap

GROUND_Z_M = -0.3495
LOADED_ROAD_Z_M = GROUND_Z_M + 0.008


def _flat_road(z_m: float = LOADED_ROAD_Z_M) -> RoadProfile:
    x_m = np.arange(-2.0, 4.0 + 0.0025, 0.005)
    return RoadProfile.from_samples(x_m, np.full(x_m.size, z_m, dtype=float))


def _ride_model():
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(mode="ride", rider="none")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _applier(model, config: TyreConfig | None = None, road: RoadProfile | None = None):
    return TyreForceApplier(
        model,
        config if config is not None else TyreConfig(model="pneumatic", tier="fast"),
        road if road is not None else _flat_road(),
        SurfaceMap.uniform("asphalt"),
    )


@lru_cache(maxsize=16)
def _steady_rolling_crr(tyre: TyreSpecs, pressure_bar: float) -> float:
    """Measures the flat-road longitudinal drag at a solved 490.5 N support load."""
    spec = tyre.with_pressure(pressure_bar)
    pneumatic = PneumaticTyre(spec, TIERS["detailed"])
    road = _flat_road(0.0)
    surfaces = SurfaceMap.uniform("asphalt")
    radius_m = spec.outer_radius_mm / 1000.0
    speed_mps = 20.0 / 3.6
    compression_m = 0.008
    hub = np.asarray([0.0, 0.0, radius_m - compression_m])
    output = pneumatic.evaluate(
        hub, np.zeros(3), 0.0, road, surfaces, TIERS["detailed"].timestep_s
    )

    # A short Newton relaxation keeps the dynamic rolling load at the drum's 50 kg load;
    # material memory is preserved between the position corrections.
    for _ in range(4):
        for _ in range(3000):
            hub[2] = radius_m - compression_m
            effective_radius_m = radius_m - output.mean_deflection_m / 3.0
            output = pneumatic.evaluate(
                hub,
                np.asarray([speed_mps, 0.0, 0.0]),
                speed_mps / effective_radius_m,
                road,
                surfaces,
                TIERS["detailed"].timestep_s,
            )
        compression_m += (490.5 - output.support_n) / 80_000.0
    for _ in range(3000):
        hub[2] = radius_m - compression_m
        effective_radius_m = radius_m - output.mean_deflection_m / 3.0
        output = pneumatic.evaluate(
            hub,
            np.asarray([speed_mps, 0.0, 0.0]),
            speed_mps / effective_radius_m,
            road,
            surfaces,
            TIERS["detailed"].timestep_s,
        )
    return -float(output.force_world_n[0]) / output.normal_load_n


def test_pneumatic_tyre_outputs_radial_load_and_per_patch_brush_data():
    tyre = PneumaticTyre(FRONT_TYRE, TIERS["fast"])
    hub = np.asarray([0.8, 0.0, 0.0225])
    outputs = tyre.evaluate(
        hub,
        np.zeros(3),
        omega_forward_radps=0.0,
        road=_flat_road(),
        surface_map=SurfaceMap.uniform("asphalt"),
        dt_s=0.0005,
    )

    assert not outputs.airborne
    assert outputs.normal_load_n > 0.0
    assert outputs.support_n > 0.0
    assert outputs.contact_length_m > 0.0
    assert outputs.patches
    assert outputs.patches[0].surface_name == "asphalt"
    assert outputs.patches[0].contact_length_m == pytest.approx(outputs.contact_length_m)
    assert outputs.force_world_n[2] == pytest.approx(outputs.support_n)
    assert outputs.patches[0].tangential_force_n == pytest.approx(0.0)


def test_patch_tangential_moment_matches_the_effective_rolling_radius():
    pneumatic = PneumaticTyre(FRONT_TYRE, TIERS["fast"])
    road = _flat_road()
    surfaces = SurfaceMap.uniform("asphalt")
    radius_m = FRONT_TYRE.outer_radius_mm / 1000.0
    hub = np.asarray([0.8, 0.0, 0.0225])
    speed_mps = 4.0
    dt_s = 0.0005
    initial = pneumatic.evaluate(
        hub, np.zeros(3), 0.0, road, surfaces, dt_s
    )
    effective_radius_m = radius_m - initial.mean_deflection_m / 3.0
    omega_forward = 1.1 * speed_mps / effective_radius_m
    for _ in range(600):
        outputs = pneumatic.evaluate(
            hub,
            np.asarray([speed_mps, 0.0, 0.0]),
            omega_forward,
            road,
            surfaces,
            dt_s,
        )

    patch = outputs.patches[0]
    tangential_force_world_n = patch.tangential_force_n * patch.tangent_world
    moment_y_nm = np.cross(
        patch.centroid_world_m - hub, tangential_force_world_n
    )[1]
    expected_moment_y_nm = -patch.tangential_force_n * effective_radius_m
    assert patch.slip_ratio == pytest.approx(0.1, abs=1e-3)
    assert moment_y_nm == pytest.approx(expected_moment_y_nm, rel=0.02)


def test_tyre_force_applier_assigns_both_wrenches_and_clears_stale_rows():
    model, data = _ride_model()
    applier = _applier(model, TyreConfig(model="pneumatic", surface="wet"))
    front_id = applier.front_wheel.body_id
    rear_id = applier.rear_wheel.body_id
    data.xfrc_applied[front_id, :] = 123.0
    data.xfrc_applied[rear_id, :] = -456.0

    applier.apply(model, data)

    for body_id, outputs in (
        (front_id, applier.front_outputs),
        (rear_id, applier.rear_outputs),
    ):
        expected_torque = np.zeros(3)
        for patch in outputs.patches:
            expected_torque += np.cross(
                patch.centroid_world_m - data.xipos[body_id],
                patch.force_world_n,
            )
        assert data.xfrc_applied[body_id, :3] == pytest.approx(outputs.force_world_n)
        assert data.xfrc_applied[body_id, 3:] == pytest.approx(expected_torque)
        assert outputs.normal_load_n > 0.0
        assert outputs.patches[0].surface_name == "wet"

    applier.profile = _flat_road(GROUND_Z_M - 0.20)
    applier.apply(model, data)
    assert applier.front_outputs.airborne and applier.rear_outputs.airborne
    assert data.xfrc_applied[front_id, :] == pytest.approx(np.zeros(6))
    assert data.xfrc_applied[rear_id, :] == pytest.approx(np.zeros(6))

    data.xfrc_applied[front_id, :] = 7.0
    data.xfrc_applied[rear_id, :] = 8.0
    applier.reset(data)
    assert data.xfrc_applied[front_id, :] == pytest.approx(np.zeros(6))
    assert data.xfrc_applied[rear_id, :] == pytest.approx(np.zeros(6))


def test_absolute_wheel_velocity_includes_carrier_pitch():
    model, data = _ride_model()
    applier = _applier(model)
    pitch_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")
    pitch_dof = int(model.jnt_dofadr[pitch_joint])
    spin_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "front_wheel_spin")
    spin_dof = int(model.jnt_dofadr[spin_joint])
    data.qvel[pitch_dof] = 0.4
    data.qvel[spin_dof] = -5.0
    mujoco.mj_forward(model, data)

    velocity6 = np.zeros(6)
    mujoco.mj_objectVelocity(
        model,
        data,
        mujoco.mjtObj.mjOBJ_BODY,
        applier.front_wheel.body_id,
        velocity6,
        0,
    )
    applier.apply(model, data)

    assert applier.front_outputs.omega_forward_radps == pytest.approx(-velocity6[1])
    assert applier.front_outputs.hub_velocity_world_mps == pytest.approx(velocity6[3:6])
    assert applier.front_outputs.omega_forward_radps != pytest.approx(-data.qvel[spin_dof])


def test_detailed_tier_sets_the_runtime_timestep_but_fast_keeps_the_compiled_step():
    model_fast, _ = _ride_model()
    _applier(model_fast, TyreConfig(model="pneumatic", tier="fast"))
    assert model_fast.opt.timestep == pytest.approx(0.0005)

    model_detailed, _ = _ride_model()
    _applier(model_detailed, TyreConfig(model="pneumatic", tier="detailed"))
    assert model_detailed.opt.timestep == pytest.approx(0.00025)


def test_applier_rejects_sphere_config_and_mismatched_geometry():
    model, _ = _ride_model()
    with pytest.raises(ValueError, match="tyre_model='pneumatic'"):
        _applier(model, TyreConfig())

    wrong_front = TyreSpecs(
        **{
            **FRONT_TYRE.__dict__,
            "outer_radius_mm": FRONT_TYRE.outer_radius_mm + 1.0,
        }
    )
    wrong = TyreConfig(model="pneumatic", front=wrong_front)
    with pytest.raises(ValueError, match="does not match tyre radius"):
        _applier(model, wrong)


@pytest.mark.parametrize("tyre", [FRONT_TYRE, REAR_TYRE], ids=["front", "rear"])
def test_rolling_resistance_shortfall_is_filled_and_pressure_trend_matches(tyre):
    raw = _steady_rolling_crr(replace(tyre, tread_loss_crr=0.0), 1.5)
    target_reference = crr_target(tyre, 1.5)
    assert raw < 0.85 * target_reference
    assert raw + tyre.tread_loss_crr == pytest.approx(target_reference, rel=0.015)

    pressure_range = (1.3, 1.8)
    measured = [
        _steady_rolling_crr(tyre, pressure)
        for pressure in pressure_range
    ]
    targets = [crr_target(tyre, pressure) for pressure in pressure_range]
    assert measured == pytest.approx(targets, rel=0.15)
    exponent = np.log(measured[1] / measured[0]) / np.log(pressure_range[1] / pressure_range[0])
    assert -0.4 <= exponent <= -0.2


def test_tyremodel_kernel_does_not_import_mujoco():
    source = Path(__file__).resolve().parents[1] / "src/bike_sim/sim/ride/tyre/model.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    imported = {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported |= {
        node.module.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    assert "mujoco" not in imported
