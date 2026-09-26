"""
Tests for the pneumatic tyre's specifications (docs/RIDE.md sections 3.1 and 11.2).

The tyre's geometry must be the bike's own -- wheel radii from `BikeSpecs`, rim radii from
the compiled model -- or the tyre and the picture of it would disagree. The literature
targets must say what the contract says, and the configuration must reject what it cannot
run.
"""

import ast
from pathlib import Path

import mujoco
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics import tyre as tyre_module
from bike_sim.physics.tyre import (
    CONTACT_LENGTH_TARGETS_MM,
    CRR_PRESSURE_EXPONENT,
    DEFAULT_TYRE_MODEL,
    DEFAULT_TYRE_TIER,
    FRONT_TYRE,
    PRESSURE_MAX_BAR,
    PRESSURE_MIN_BAR,
    REAR_TYRE,
    RIM_STRIKE_FRACTION_BAND,
    TIERS,
    TyreConfig,
    TyreSpecs,
    clamp_pressure_bar,
    crr_target,
    static_stiffness_target_n_mm,
)


@pytest.fixture(scope="module")
def ride_model():
    return mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride", rider="none"))


def _geom_radius_mm(model, name):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert gid >= 0, name
    return 1000.0 * float(model.geom_size[gid][0])


def test_outer_radii_are_the_bikes_wheel_radii():
    specs = BikeSpecs()
    assert FRONT_TYRE.outer_radius_mm == specs.front_wheel_radius
    assert REAR_TYRE.outer_radius_mm == specs.rear_wheel_radius


def test_radii_match_the_compiled_model(ride_model):
    assert _geom_radius_mm(ride_model, "geom_front_rim") == pytest.approx(FRONT_TYRE.rim_radius_mm)
    assert _geom_radius_mm(ride_model, "geom_rear_rim") == pytest.approx(REAR_TYRE.rim_radius_mm)
    assert _geom_radius_mm(ride_model, "geom_front_contact") == pytest.approx(FRONT_TYRE.outer_radius_mm)
    assert _geom_radius_mm(ride_model, "geom_rear_contact") == pytest.approx(REAR_TYRE.outer_radius_mm)


def test_stock_tyres_and_pressures():
    assert "Magic Mary" in FRONT_TYRE.name and "Hans Dampf" in REAR_TYRE.name
    assert (FRONT_TYRE.pressure_bar, REAR_TYRE.pressure_bar) == (1.5, 1.7)
    assert (FRONT_TYRE.crr_reference, REAR_TYRE.crr_reference) == (0.011, 0.0103)


def test_rim_strike_deflection_is_46_mm_and_inside_the_literature_band():
    for t in (FRONT_TYRE, REAR_TYRE):
        assert t.tyre_height_mm == pytest.approx(52.0)
        assert t.rim_strike_deflection_mm == pytest.approx(46.0)
        lo, hi = RIM_STRIKE_FRACTION_BAND
        assert lo <= t.rim_strike_deflection_mm / t.section_height_mm <= hi, t.name


def test_static_stiffness_law():
    assert static_stiffness_target_n_mm(1.5) == pytest.approx(58.0)
    assert static_stiffness_target_n_mm(1.7) == pytest.approx(62.8)
    assert static_stiffness_target_n_mm(1.72) == pytest.approx(63.28)
    assert CONTACT_LENGTH_TARGETS_MM == {1.38: 133.0, 1.72: 122.0}


def test_crr_target_follows_the_pressure_law():
    assert crr_target(REAR_TYRE, 1.5) == pytest.approx(0.0103)
    assert crr_target(FRONT_TYRE, 1.5) == pytest.approx(0.011)
    assert crr_target(REAR_TYRE) == pytest.approx(0.0103 * (1.7 / 1.5) ** CRR_PRESSURE_EXPONENT)
    assert crr_target(REAR_TYRE, 3.0) < crr_target(REAR_TYRE, 1.5)


def test_with_pressure_copies():
    softer = REAR_TYRE.with_pressure(1.2)
    assert softer.pressure_bar == 1.2 and REAR_TYRE.pressure_bar == 1.7
    assert softer.pressure_pa == pytest.approx(1.2e5)


@pytest.mark.parametrize(
    "change, message",
    [
        (dict(pressure_bar=0.0), "pressure_bar must be positive"),
        (dict(rim_radius_mm=400.0), "not inside"),
        (dict(compressed_casing_mm=60.0), "leaves no travel"),
        (dict(area_factor=1.2), "at most 1"),
        (dict(loss_factor=-0.1), "must not be negative"),
        (dict(loss_factor=1.0), "below 1"),
    ],
)
def test_tyre_validation(change, message):
    base = dict(name="t", outer_radius_mm=372.0, rim_radius_mm=320.0, section_height_mm=57.0,
                pressure_bar=1.5, crr_reference=0.011)
    base.update(change)
    with pytest.raises(ValueError, match=message):
        TyreSpecs(**base)


def test_tiers_are_the_contracts():
    fast, detailed = TIERS["fast"], TIERS["detailed"]
    assert (fast.n_rays, fast.half_angle_deg, fast.timestep_s, fast.discretised_brush) == (64, 75.0, 0.0005, False)
    assert (detailed.n_rays, detailed.half_angle_deg, detailed.timestep_s, detailed.discretised_brush) == (
        256, 75.0, 0.00025, True)


def test_config_defaults_and_validation():
    cfg = TyreConfig()
    assert (cfg.model, cfg.tier) == (DEFAULT_TYRE_MODEL, DEFAULT_TYRE_TIER) == ("sphere", "fast")
    assert not cfg.pneumatic and cfg.surface is None
    assert TyreConfig(model="pneumatic", tier="detailed").tier_spec is TIERS["detailed"]
    assert TyreConfig(model="pneumatic", surface="wet").surface == "wet"
    for bad, message in ((dict(model="solid"), "unknown tyre model"), (dict(tier="ultra"), "unknown tyre tier"),
                         (dict(surface="ice"), "unknown surface")):
        with pytest.raises(ValueError, match=message):
            TyreConfig(**bad)


def test_config_with_pressures():
    cfg = TyreConfig(model="pneumatic").with_pressures(1.3, 1.9)
    assert (cfg.front.pressure_bar, cfg.rear.pressure_bar) == (1.3, 1.9)
    assert cfg.front.name == FRONT_TYRE.name


def test_pressure_clamp():
    assert clamp_pressure_bar(0.1) == PRESSURE_MIN_BAR
    assert clamp_pressure_bar(9.0) == PRESSURE_MAX_BAR
    assert clamp_pressure_bar(1.55) == 1.55


def test_physics_tyre_does_not_import_mujoco():
    tree = ast.parse(Path(tyre_module.__file__).read_text(encoding="utf-8"))
    imported = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import)
                for alias in node.names}
    imported |= {node.module.split(".")[0] for node in ast.walk(tree)
                 if isinstance(node, ast.ImportFrom) and node.module}
    assert "mujoco" not in imported
