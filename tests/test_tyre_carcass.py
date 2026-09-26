"""Carcass-force calibration, material transport and rim-strike checks.

The calibration helper fits the literature stiffness and contact-length curves on a dense
flat-road radial quadrature, then the assertions exercise the production 256-ray tier.
The drop-sled model is a pure NumPy 1-DOF simulation, independent of MuJoCo integration.
"""

import ast
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import least_squares

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.tyre import (
    CONTACT_LENGTH_TARGETS_MM,
    DAMPING_RATIO_BAND,
    DROP_SLED_MASS_KG,
    DYNAMIC_STIFFNESS_RATIO_BAND,
    FRONT_TYRE,
    REAR_TYRE,
    RIM_STRIKE_FRACTION_BAND,
    TyreSpecs,
    static_stiffness_target_n_mm,
)
from bike_sim.sim.ride.tyre.carcass import (
    CarcassState,
    evaluate_carcass,
    material_deflection_rate_mps,
)
from bike_sim.sim.ride.tyre.geometry import RayHits, RayRing

RAY_HALF_ANGLE_RAD = np.radians(75.0)
CALIBRATION_RAYS = 1024
DROP_LOADS_N = (398.0, 438.0)
DROP_SLED_DT_S = 0.0005
DROP_SLED_DROP_HEIGHT_M = 0.010


def _static_response(
    tyre: TyreSpecs,
    pressure_bar: float,
    compression_m: float,
    *,
    n_rays: int = CALIBRATION_RAYS,
    area_factor: float | None = None,
    carcass_stiffness_n_mm2: float | None = None,
    contact_length_factor: float | None = None,
):
    """Flat-road static resultant and footprint for the radial-element law."""
    c_a = tyre.area_factor if area_factor is None else area_factor
    k_c = tyre.carcass_stiffness_n_mm2 if carcass_stiffness_n_mm2 is None else carcass_stiffness_n_mm2
    c_l = tyre.contact_length_factor if contact_length_factor is None else contact_length_factor
    radius_m = tyre.outer_radius_mm / 1000.0
    width_m = tyre.casing_width_mm / 1000.0
    theta = np.linspace(-RAY_HALF_ANGLE_RAD, RAY_HALF_ANGLE_RAD, n_rays)
    dtheta = theta[1] - theta[0]
    hub_height_m = radius_m - compression_m
    ray_r_m = hub_height_m / np.cos(theta)
    delta_m = np.maximum(radius_m - ray_r_m, 0.0)
    chord_m = 2.0 * np.sqrt(np.maximum(delta_m * (width_m - delta_m), 0.0))
    chord_m = np.minimum(chord_m, width_m)
    line_force_n_per_m = (
        c_a * pressure_bar * 100_000.0 * chord_m
        + k_c * 1_000_000.0 * delta_m
    )
    element_force_n = line_force_n_per_m * radius_m * dtheta
    vertical_force_n = float(np.sum(element_force_n * np.cos(theta)))
    pressure_force_n = float(np.sum(
        c_a * pressure_bar * 100_000.0 * chord_m * radius_m * dtheta * np.cos(theta)
    ))
    loaded = delta_m > 0.0
    if loaded.any():
        road_x_m = ray_r_m * np.sin(theta)
        contact_length_m = c_l * float(road_x_m[loaded][-1] - road_x_m[loaded][0])
    else:
        contact_length_m = 0.0
    share = pressure_force_n / vertical_force_n if vertical_force_n > 0.0 else 0.0
    return vertical_force_n, contact_length_m, share


def _sag_for_load_m(
    tyre: TyreSpecs,
    pressure_bar: float,
    load_n: float,
    *,
    n_rays: int = CALIBRATION_RAYS,
    area_factor: float | None = None,
    carcass_stiffness_n_mm2: float | None = None,
    contact_length_factor: float | None = None,
) -> float:
    """Inverts the static flat-road force law for a target wheel load."""
    low_m, high_m = 0.0, tyre.rim_strike_deflection_mm / 1000.0
    for _ in range(44):
        middle_m = 0.5 * (low_m + high_m)
        force_n, _, _ = _static_response(
            tyre, pressure_bar, middle_m, n_rays=n_rays, area_factor=area_factor,
            carcass_stiffness_n_mm2=carcass_stiffness_n_mm2,
            contact_length_factor=contact_length_factor,
        )
        if force_n < load_n:
            low_m = middle_m
        else:
            high_m = middle_m
    return 0.5 * (low_m + high_m)


@lru_cache(maxsize=2)
def _fit_literature_constants(tyre: TyreSpecs) -> tuple[float, float, float]:
    """Fits c_A, k_c and c_L to the stiffness and footprint literature targets."""
    pressures = np.linspace(1.0, 2.0, 5)

    def residual(parameters):
        c_a, k_c, c_l = parameters
        errors = []
        for pressure in pressures:
            sag_lo = _sag_for_load_m(
                tyre, pressure, DROP_LOADS_N[0], area_factor=c_a,
                carcass_stiffness_n_mm2=k_c, contact_length_factor=c_l,
            )
            sag_hi = _sag_for_load_m(
                tyre, pressure, DROP_LOADS_N[1], area_factor=c_a,
                carcass_stiffness_n_mm2=k_c, contact_length_factor=c_l,
            )
            stiffness_n_mm = (DROP_LOADS_N[1] - DROP_LOADS_N[0]) / (sag_hi - sag_lo) / 1000.0
            target_n_mm = static_stiffness_target_n_mm(pressure)
            errors.append((stiffness_n_mm - target_n_mm) / target_n_mm)
        for pressure, target_mm in CONTACT_LENGTH_TARGETS_MM.items():
            sag_m = _sag_for_load_m(
                tyre, pressure, 418.0, area_factor=c_a,
                carcass_stiffness_n_mm2=k_c, contact_length_factor=c_l,
            )
            _, length_m, _ = _static_response(
                tyre, pressure, sag_m, area_factor=c_a,
                carcass_stiffness_n_mm2=k_c, contact_length_factor=c_l,
            )
            errors.append((length_m * 1000.0 - target_mm) / target_mm)
        return np.asarray(errors)

    fit = least_squares(
        residual,
        x0=(tyre.area_factor, tyre.carcass_stiffness_n_mm2, tyre.contact_length_factor),
        bounds=((0.01, 0.0, 0.1), (1.0, 2.0, 1.0)),
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
        max_nfev=200,
    )
    assert fit.success, fit.message
    return tuple(float(value) for value in fit.x)


def _flat_hits(ring: RayRing, tyre: TyreSpecs, compression_m: float, x_m: float = 4.0) -> RayHits:
    """Exact radial-ray intersections with a flat road at z=0."""
    radius_m = tyre.outer_radius_mm / 1000.0
    hub_height_m = radius_m - compression_m
    cosine = -ring.uz
    ray_r_m = np.where(cosine > 0.0, hub_height_m / cosine, np.inf)
    hit = ray_r_m <= radius_m
    ray_r_m = np.where(hit, ray_r_m, np.inf)
    reach_m = np.minimum(ray_r_m, radius_m)
    return RayHits(
        r_m=ray_r_m,
        delta_m=np.maximum(radius_m - ray_r_m, 0.0),
        road_x_m=x_m + reach_m * ring.ux,
        road_z_m=np.zeros(ring.n, dtype=float),
        coverage_event=False,
        airborne=not bool(hit.any()),
    )


def _drop_sled(tyre: TyreSpecs, duration_s: float = 0.8):
    """Drops the 44 kg sled 10 mm onto the tyre and records its free oscillation."""
    ring = RayRing(256, RAY_HALF_ANGLE_RAD)
    mass_kg = DROP_SLED_MASS_KG
    pressure_bar = tyre.pressure_bar
    equilibrium_m = _sag_for_load_m(tyre, pressure_bar, mass_kg * 9.81, n_rays=1024)
    compression_m = -DROP_SLED_DROP_HEIGHT_M
    state = CarcassState(ring.n)
    result = evaluate_carcass(
        ring,
        _flat_hits(ring, tyre, compression_m),
        tyre,
        state,
        dt_s=DROP_SLED_DT_S,
    )
    velocity_mps = 0.0
    acceleration_mps2 = 9.81 - result.force_z_n / mass_kg
    count = int(round(duration_s / DROP_SLED_DT_S))
    times = [0.0]
    compressions = [compression_m]
    velocities = [velocity_mps]
    forces = [result.force_z_n]
    for step in range(1, count + 1):
        velocity_mps += acceleration_mps2 * DROP_SLED_DT_S
        compression_m += velocity_mps * DROP_SLED_DT_S
        result = evaluate_carcass(
            ring,
            _flat_hits(ring, tyre, compression_m),
            tyre,
            state,
            dt_s=DROP_SLED_DT_S,
        )
        acceleration_mps2 = 9.81 - result.force_z_n / mass_kg
        times.append(step * DROP_SLED_DT_S)
        compressions.append(compression_m)
        velocities.append(velocity_mps)
        forces.append(result.force_z_n)

    return equilibrium_m, np.asarray(times), np.asarray(compressions), np.asarray(velocities), np.asarray(forces)


@lru_cache(maxsize=8)
def _drop_sled_metrics(tyre: TyreSpecs):
    """Returns the dynamic/static stiffness ratio and mean first-three-cycle damping ratio."""
    equilibrium_m, times, compression_m, velocity_mps, force_n = _drop_sled(tyre)
    loading = (
        (times < 0.25)
        & (velocity_mps > 0.0)
        & (force_n >= 100.0)
        & (force_n <= 800.0)
    )
    dynamic_stiffness_n_m = float(np.polyfit(compression_m[loading], force_n[loading], 1)[0])
    sag_lo_m = _sag_for_load_m(tyre, tyre.pressure_bar, DROP_LOADS_N[0])
    sag_hi_m = _sag_for_load_m(tyre, tyre.pressure_bar, DROP_LOADS_N[1])
    static_stiffness_n_m = (DROP_LOADS_N[1] - DROP_LOADS_N[0]) / (sag_hi_m - sag_lo_m)
    dynamic_ratio = dynamic_stiffness_n_m / static_stiffness_n_m

    peaks = np.flatnonzero(
        (compression_m[1:-1] >= compression_m[:-2])
        & (compression_m[1:-1] > compression_m[2:])
    ) + 1
    peaks = peaks[(times[peaks] > 0.05) & (compression_m[peaks] > equilibrium_m + 0.0002)]
    amplitudes_m = compression_m[peaks] - equilibrium_m
    if amplitudes_m.size < 4:
        raise AssertionError(f"drop sled did not produce three rebound cycles for {tyre.name}")
    decrements = np.log(amplitudes_m[:-1] / amplitudes_m[1:])
    log_decrement = float(np.mean(decrements[:3]))
    damping_ratio = log_decrement / np.sqrt((2.0 * np.pi) ** 2 + log_decrement ** 2)
    return float(dynamic_ratio), float(damping_ratio)


@lru_cache(maxsize=4)
def _fit_rate_stiffening(tyre: TyreSpecs) -> float:
    """Fits the Maxwell fraction to the midpoint of the drop-test stiffness band."""
    target = 0.5 * sum(DYNAMIC_STIFFNESS_RATIO_BAND)
    fit = least_squares(
        lambda values: np.asarray([
            (_drop_sled_metrics(replace(tyre, rate_stiffening=float(values[0])))[0] - target) / target
        ]),
        x0=(tyre.rate_stiffening,),
        bounds=((0.01,), (0.8,)),
        diff_step=0.01,
        xtol=1e-6,
        ftol=1e-6,
        gtol=1e-6,
        max_nfev=20,
    )
    assert fit.success, fit.message
    return float(fit.x[0])


@lru_cache(maxsize=4)
def _fit_loss_factor(tyre: TyreSpecs) -> float:
    """Fits η to the midpoint of the measured damping-ratio band."""
    target = 0.5 * sum(DAMPING_RATIO_BAND)
    fit = least_squares(
        lambda values: np.asarray([
            (_drop_sled_metrics(replace(tyre, loss_factor=float(values[0])))[1] - target) / target
        ]),
        x0=(tyre.loss_factor,),
        bounds=((0.01,), (0.2,)),
        diff_step=0.01,
        xtol=1e-6,
        ftol=1e-6,
        gtol=1e-6,
        max_nfev=20,
    )
    assert fit.success, fit.message
    return float(fit.x[0])


@pytest.mark.parametrize("tyre", [FRONT_TYRE, REAR_TYRE], ids=["front", "rear"])
def test_fitted_constants_match_the_literature_and_are_frozen_in_specs(tyre):
    fitted = _fit_literature_constants(tyre)
    frozen = (tyre.area_factor, tyre.carcass_stiffness_n_mm2, tyre.contact_length_factor)
    assert frozen == pytest.approx(fitted, rel=0.01)

    # The detailed production resolution measures the target tangent stiffness around 418 N.
    for pressure in np.linspace(1.0, 2.0, 5):
        sag_lo = _sag_for_load_m(tyre, pressure, 398.0, n_rays=256)
        sag_hi = _sag_for_load_m(tyre, pressure, 438.0, n_rays=256)
        stiffness_n_mm = 40.0 / (sag_hi - sag_lo) / 1000.0
        target_n_mm = static_stiffness_target_n_mm(pressure)
        assert stiffness_n_mm == pytest.approx(target_n_mm, rel=0.15), (tyre.name, pressure)

    nominal_sag = _sag_for_load_m(tyre, tyre.pressure_bar, 418.0, n_rays=256)
    _, _, pressure_share = _static_response(
        tyre, tyre.pressure_bar, nominal_sag, n_rays=256
    )
    assert 0.70 <= pressure_share <= 1.00


@pytest.mark.parametrize("tyre", [FRONT_TYRE, REAR_TYRE], ids=["front", "rear"])
def test_rate_stiffening_and_loss_factor_are_fitted_in_the_drop_sled(tyre):
    fitted_rate = _fit_rate_stiffening(tyre)
    fitted_loss = _fit_loss_factor(tyre)
    assert tyre.rate_stiffening == pytest.approx(fitted_rate, abs=0.01)
    assert tyre.loss_factor == pytest.approx(fitted_loss, abs=0.005)


@pytest.mark.parametrize("tyre", [FRONT_TYRE, REAR_TYRE], ids=["front", "rear"])
def test_contact_length_targets_at_reference_load(tyre):
    for pressure, target_mm in CONTACT_LENGTH_TARGETS_MM.items():
        sag_m = _sag_for_load_m(tyre, pressure, 418.0, n_rays=256)
        _, length_m, _ = _static_response(tyre, pressure, sag_m, n_rays=256)
        assert length_m * 1000.0 == pytest.approx(target_mm, rel=0.15)


def test_first_carcass_step_is_static_and_zero_history_is_reproducible():
    ring = RayRing(64, RAY_HALF_ANGLE_RAD)
    hits = _flat_hits(ring, FRONT_TYRE, 0.008)
    state = CarcassState(ring.n)
    first = evaluate_carcass(ring, hits, FRONT_TYRE, state, dt_s=0.0005)
    second = evaluate_carcass(ring, hits, FRONT_TYRE, state, dt_s=0.0005)

    assert first.force_z_n > 0.0
    assert first.maxwell_force_n == pytest.approx(np.zeros(ring.n))
    assert first.hysteresis_force_n == pytest.approx(np.zeros(ring.n))
    assert first.force_z_n == pytest.approx(second.force_z_n)
    assert not first.rim_strike_active and first.rim_event is None

    state.reset()
    again = evaluate_carcass(ring, hits, FRONT_TYRE, state, dt_s=0.0005)
    assert again.total_element_force_n == pytest.approx(first.total_element_force_n)


def test_pressure_chord_saturates_at_the_casing_width():
    ring = RayRing(8, RAY_HALF_ANGLE_RAD)
    delta = np.asarray([0.0, 0.002, 0.008, 0.020, 0.030, 0.040, 0.046, 0.050])
    width_m = FRONT_TYRE.casing_width_mm / 1000.0
    hits = RayHits(
        r_m=np.ones(ring.n),
        delta_m=delta,
        road_x_m=np.arange(ring.n, dtype=float),
        road_z_m=np.zeros(ring.n, dtype=float),
        coverage_event=False,
        airborne=False,
    )
    result = evaluate_carcass(ring, hits, FRONT_TYRE, CarcassState(ring.n), dt_s=0.0005)
    ds_m = FRONT_TYRE.outer_radius_mm / 1000.0 * ring.dtheta_rad
    maximum_pressure_force_n = FRONT_TYRE.area_factor * FRONT_TYRE.pressure_bar * 100_000.0 * width_m * ds_m
    assert result.pressure_force_n[-3:] == pytest.approx(
        np.full(3, maximum_pressure_force_n)
    )


def test_material_rate_uses_upwind_transport_and_changes_sign_with_rotation():
    delta = np.asarray([0.0, 0.001, 0.002, 0.001, 0.0])
    dt_s, dtheta_rad, omega_radps = 0.01, 0.1, 1.0
    forward = material_deflection_rate_mps(
        delta, delta, omega_radps, dt_s, dtheta_rad
    )
    backward = material_deflection_rate_mps(
        delta, delta, -omega_radps, dt_s, dtheta_rad
    )
    assert forward[3] > 0.0 and forward[1] < 0.0
    assert backward[1] > 0.0 and backward[3] < 0.0
    assert material_deflection_rate_mps(delta, delta, 0.0, dt_s, dtheta_rad) == pytest.approx(
        np.zeros_like(delta)
    )


def test_steady_forward_rolling_shifts_the_centre_of_pressure_forward():
    ring = RayRing(256, RAY_HALF_ANGLE_RAD)
    hits = _flat_hits(ring, FRONT_TYRE, 0.008)
    state = CarcassState(ring.n)
    evaluate_carcass(ring, hits, FRONT_TYRE, state, dt_s=0.0005)
    for _ in range(600):
        result = evaluate_carcass(
            ring, hits, FRONT_TYRE, state, dt_s=0.0005, omega_radps=10.0, speed_mps=3.72
        )

    vertical_element_force = result.total_element_force_n * (-ring.uz)
    centre_x_m = float(np.dot(hits.road_x_m, vertical_element_force) / result.force_z_n)
    assert centre_x_m > 0.0
    assert result.force_x_n < 0.0


def test_rim_strike_threshold_energy_and_event_record():
    ring = RayRing(256, RAY_HALF_ANGLE_RAD)
    state = CarcassState(ring.n)
    below = FRONT_TYRE.rim_strike_deflection_mm / 1000.0 - 0.00002
    above = FRONT_TYRE.rim_strike_deflection_mm / 1000.0 + 0.00020

    before = evaluate_carcass(
        ring, _flat_hits(ring, FRONT_TYRE, below), FRONT_TYRE, state, dt_s=0.0005
    )
    assert not before.rim_strike_active
    struck = evaluate_carcass(
        ring,
        _flat_hits(ring, FRONT_TYRE, above, x_m=6.0),
        FRONT_TYRE,
        state,
        dt_s=0.0005,
        speed_mps=5.0,
    )
    assert struck.rim_strike_active
    assert struck.rim_energy_j > 0.0
    assert np.sum(struck.rim_force_n) > 0.0

    released = evaluate_carcass(
        ring,
        _flat_hits(ring, FRONT_TYRE, 0.040, x_m=6.1),
        FRONT_TYRE,
        state,
        dt_s=0.0005,
        speed_mps=4.0,
    )
    assert not released.rim_strike_active
    assert released.rim_event is not None
    assert released.rim_event.x_m == pytest.approx(6.0, abs=0.02)
    assert released.rim_event.speed_mps == pytest.approx(5.0)
    assert released.rim_event.peak_load_n > 0.0
    assert released.rim_event.peak_rim_force_n > 0.0
    assert released.rim_event.absorbed_energy_j == pytest.approx(struck.rim_energy_j)


@pytest.mark.parametrize("tyre, body_name", [(FRONT_TYRE, "front_wheel"), (REAR_TYRE, "rear_wheel")])
def test_stiffest_carcass_and_rim_patch_stay_inside_fast_step_budget(tyre, body_name):
    import mujoco

    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride", rider="none"))
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    assert body_id >= 0
    wheel_mass_kg = float(model.body_mass[body_id])

    # The 50 mm rim-patch cap is the one specified in RIDE.md §3.1.  Dynamic carcass
    # stiffness is included, so this uses the lightest compiled wheel as the limiting case.
    carcass_n_per_m = static_stiffness_target_n_mm(2.0) * 1000.0 * (1 + tyre.rate_stiffening)
    rim_n_per_m = tyre.rim_stiffness_n_mm2 * 50.0 * 1000.0
    omega_dt = np.sqrt((carcass_n_per_m + rim_n_per_m) / wheel_mass_kg) * 0.0005
    assert omega_dt <= 0.4, (body_name, wheel_mass_kg, carcass_n_per_m, rim_n_per_m, omega_dt)

    detailed_omega_dt = omega_dt * 0.5
    assert detailed_omega_dt <= 0.4


@pytest.mark.parametrize("tyre", [FRONT_TYRE, REAR_TYRE], ids=["front", "rear"])
def test_drop_sled_dynamic_stiffness_and_hysteresis_damping(tyre):
    ratio, damping_ratio = _drop_sled_metrics(tyre)
    assert DAMPING_RATIO_BAND[0] <= damping_ratio <= DAMPING_RATIO_BAND[1], damping_ratio
    assert DYNAMIC_STIFFNESS_RATIO_BAND[0] <= ratio <= DYNAMIC_STIFFNESS_RATIO_BAND[1], ratio


def test_rim_strike_threshold_is_inside_the_section_height_band():
    for tyre in (FRONT_TYRE, REAR_TYRE):
        fraction = tyre.rim_strike_deflection_mm / tyre.section_height_mm
        assert RIM_STRIKE_FRACTION_BAND[0] <= fraction <= RIM_STRIKE_FRACTION_BAND[1]


def test_carcass_kernel_does_not_import_mujoco():
    source = Path(__file__).resolve().parents[1] / "src/bike_sim/sim/ride/tyre/carcass.py"
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
