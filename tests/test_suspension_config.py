"""Resolved suspension configuration and ride-mode override contracts."""

import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.suspension_config import build_suspension_components
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_public_spec_reaches_running_components():
    specs = BikeSpecs(
        fork_lsc=2,
        shock_hsc=4,
        shock_stiffness=90000.0,
        fork_travel=170.0,
        shock_stroke=60.0,
    )

    controller, coil = build_suspension_components(specs)

    assert controller.suspension_system.fork_damper.lsc_clicks == 2
    assert controller.suspension_system.shock_damper.hsc_clicks == 4
    assert coil.specs.rate_n_m == 90000.0
    assert coil.specs.stroke_mm == 60.0
    assert controller.suspension_system.fork_damper.total_travel_mm == 170.0


@pytest.mark.parametrize(
    ("field", "value", "damper", "attribute", "velocity", "travel"),
    [
        ("fork_hsc", 4, "fork_damper", "hsc_clicks", 0.5, 70.0),
        ("fork_lsc", 2, "fork_damper", "lsc_clicks", 0.05, 70.0),
        ("fork_rebound", 15, "fork_damper", "rebound_clicks", -0.5, 70.0),
        ("shock_hsc", 4, "shock_damper", "hsc_clicks", 0.5, 20.0),
        ("shock_lsc", 2, "shock_damper", "lsc_clicks", 0.05, 20.0),
        ("shock_rebound", 12, "shock_damper", "rebound_clicks", -0.5, 20.0),
        ("shock_hbo", 4, "shock_damper", "hbo_clicks", 0.5, 60.0),
        ("shock_lockout", True, "shock_damper", "lockout_firm", 0.5, 20.0),
    ],
)
def test_each_public_damper_setting_changes_its_force(
    field, value, damper, attribute, velocity, travel
):
    baseline, _ = build_suspension_components(BikeSpecs())
    tuned, _ = build_suspension_components(BikeSpecs(**{field: value}))
    normal_damper = getattr(baseline.suspension_system, damper)
    tuned_damper = getattr(tuned.suspension_system, damper)

    assert getattr(tuned_damper, attribute) == value
    assert tuned_damper.compute_damping_force(
        velocity, travel
    ) != pytest.approx(normal_damper.compute_damping_force(velocity, travel))


def test_factory_passes_air_settings_and_coil_preload():
    specs = BikeSpecs(fork_initial_psi=95.0, fork_air_tokens=3, shock_stiffness=90000.0)
    controller, coil = build_suspension_components(specs, preload_mm=3.0)
    baseline, _ = build_suspension_components(BikeSpecs())

    assert controller.air_spring.gauge_pressure_psi == 95.0
    assert controller.air_spring.num_tokens == 3
    assert controller.air_spring.compute_axial_force(80.0) != pytest.approx(
        baseline.air_spring.compute_axial_force(80.0)
    )
    assert coil.compute_spring_force(0.0) == pytest.approx(270.0)


def test_physical_hbo_zones_follow_travel_and_legacy_stays_absolute():
    physical = BikeSuspensionSystem(fork_travel_mm=170.0, shock_stroke_mm=60.0)
    legacy = BikeSuspensionSystem(
        fork_travel_mm=170.0, shock_stroke_mm=60.0, legacy_behavior=True
    )

    assert physical.fork_damper.hbo_start_mm == pytest.approx(150.0)
    assert physical.shock_damper.hbo_start_mm == pytest.approx(48.0)
    assert legacy.fork_damper.hbo_start_mm == pytest.approx(160.0)
    assert legacy.shock_damper.hbo_start_mm == pytest.approx(52.0)


@pytest.mark.parametrize(
    ("fork_travel", "shock_stroke"),
    [(0.0, 65.0), (float("nan"), 65.0), (19.0, 65.0), (180.0, 0.0), (180.0, float("inf"))],
)
def test_invalid_or_too_short_damper_travel_is_rejected(fork_travel, shock_stroke):
    with pytest.raises(ValueError, match="travel|stroke|zone"):
        BikeSuspensionSystem(fork_travel_mm=fork_travel, shock_stroke_mm=shock_stroke)


def test_legacy_short_travel_keeps_absolute_hbo_thresholds_without_rejection():
    legacy = BikeSuspensionSystem(
        fork_travel_mm=15.0, shock_stroke_mm=40.0, legacy_behavior=True
    )

    assert legacy.fork_damper.hbo_start_mm == 160.0
    assert legacy.shock_damper.hbo_start_mm == 52.0
    assert legacy.fork_damper.compute_damping_force(0.5, 14.0) > 0.0
    assert legacy.shock_damper.compute_damping_force(0.5, 39.0) > 0.0


@pytest.mark.parametrize(
    ("specs", "field", "expected"),
    [
        (BikeSpecs(fork_travel=170.0), "fork_travel", r"170\.0.*180\.0"),
        (BikeSpecs(shock_stroke=60.0), "shock_stroke", r"60\.0.*65\.0"),
        (BikeSpecs(front_wheel_radius=370.0), "front_wheel_radius", r"370\.0.*372\.0"),
        (BikeSpecs(shock_eye_to_eye=210.0), "shock_eye_to_eye", r"210\.0.*205\.0"),
    ],
)
def test_conflicting_controller_geometry_is_rejected_before_compile(specs, field, expected):
    controller, _ = build_suspension_components(BikeSpecs())

    with pytest.raises(ValueError, match=field + ".*" + expected):
        RideSimulation(
            track=get_preset("flat"),
            rider="none",
            specs=specs,
            controller=controller,
            physics_config=SimulationPhysicsConfig(physics_mode="physical"),
        )


def test_conflicting_damper_travel_is_rejected_even_with_matching_controller_specs():
    specs = BikeSpecs()
    controller, _ = build_suspension_components(specs)
    controller.suspension_system.fork_damper.total_travel_mm = 170.0

    with pytest.raises(ValueError, match=r"fork_travel.*180\.0.*170\.0"):
        RideSimulation(
            track=get_preset("flat"), rider="none", specs=specs,
            controller=controller,
            physics_config=SimulationPhysicsConfig(physics_mode="physical"),
        )


def test_nonfinite_controller_travel_is_rejected():
    specs = BikeSpecs()
    controller, _ = build_suspension_components(specs)
    controller.air_spring.specs.total_travel_mm = float("nan")

    with pytest.raises(ValueError, match="fork_travel"):
        RideSimulation(track=get_preset("flat"), rider="none", specs=specs, controller=controller)


@pytest.mark.parametrize("stroke", [float("nan"), float("inf"), -float("inf")])
def test_mutated_nonfinite_coil_stroke_is_rejected_before_model_build(monkeypatch, stroke):
    coil = CoilShock()
    coil.specs.stroke_mm = stroke
    monkeypatch.setattr(
        "bike_sim.sim.ride_sim.generate_mujoco_xml",
        lambda **kwargs: pytest.fail("model generation reached before coil validation"),
    )

    with pytest.raises(ValueError, match=r"CoilShock stroke_mm.*finite"):
        RideSimulation(track=get_preset("flat"), rider="none", coil_shock=coil)


def test_nonfinite_bike_stroke_is_rejected_before_model_build(monkeypatch):
    specs = BikeSpecs(shock_stroke=float("nan"))
    monkeypatch.setattr(
        "bike_sim.sim.ride_sim.generate_mujoco_xml",
        lambda **kwargs: pytest.fail("model generation reached before BikeSpecs validation"),
    )

    with pytest.raises(ValueError, match=r"BikeSpecs shock_stroke.*finite"):
        RideSimulation(track=get_preset("flat"), rider="none", specs=specs, coil_shock=CoilShock())


def test_compatible_explicit_overrides_keep_their_identity_and_tuning():
    specs = BikeSpecs()
    controller, _ = build_suspension_components(specs)
    controller.air_spring.set_pressure_psi(105.0)
    coil = CoilShock(CoilShockSpecs(rate_n_m=100000.0, stroke_mm=specs.shock_stroke))

    sim = RideSimulation(
        track=get_preset("flat"), rider="none", specs=specs,
        controller=controller, coil_shock=coil,
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )

    assert sim.controller is controller
    assert sim.applier.coil_shock is coil
    assert sim.controller.air_spring.gauge_pressure_psi == 105.0
    assert sim.applier.coil_shock.specs.rate_n_m == 100000.0


def test_physical_ride_uses_resolved_factory_and_legacy_keeps_old_dampers():
    specs = BikeSpecs(fork_lsc=2, shock_hsc=4)
    physical = RideSimulation(
        track=get_preset("flat"), rider="none", specs=specs,
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    legacy = RideSimulation(track=get_preset("flat"), rider="none", specs=specs)

    assert physical.controller.suspension_system.fork_damper.lsc_clicks == 2
    assert physical.controller.suspension_system.shock_damper.hsc_clicks == 4
    assert legacy.controller.suspension_system.fork_damper.lsc_clicks == 7
    assert legacy.controller.suspension_system.shock_damper.hsc_clicks == 2
    assert legacy.controller.suspension_system.shock_damper.legacy_behavior is True
    assert legacy.applier.coil_shock.legacy_behavior is True
