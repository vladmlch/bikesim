"""
Unit tests for Pneumatic Progressive Air Spring Fork Model.

Tests include:
- Thermodynamic volume and pressure computations (adiabatic law PV^1.4 = const).
- Transfer port equalization and zero breakaway force at top-out (0 mm).
- Volume spacer (token) displacement and progressive bottom-out ramp-up.
- Automatic air pressure (PSI) calibration for target static sag (30%).
- Integration with SuspensionPlayground simulation and live telemetry.
"""

import pytest
import numpy as np

from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.sim.playground import SuspensionPlayground


def test_air_spring_initialization_and_specs():
    """Verifies default AirSpringSpecs geometric dimensions and volume properties."""
    specs = AirSpringSpecs()
    assert specs.stanchion_inner_diam_mm == 24.0
    assert specs.total_travel_mm == 180.0
    assert specs.default_tokens == 2
    assert specs.max_tokens == 4
    assert specs.gamma == 1.40

    spring = ForkAirSpring(specs=specs)
    assert spring.num_tokens == 2
    assert spring.specs.piston_area_m2 > 0.0004  # ~4.52 cm^2
    assert spring.specs.token_volume_m3 == 8.0e-6  # 8 cm^3


def test_zero_breakaway_force_at_topout():
    """Verifies that equalized positive and negative chambers yield net zero breakaway force at x=0."""
    spring = ForkAirSpring(num_tokens=2, gauge_pressure_psi=85.0)
    f0 = spring.compute_axial_force(0.0)
    assert abs(f0) < 1e-4, f"Expected 0 N breakaway force at top-out, got {f0:.4f} N"

    p_pos, p_neg = spring.compute_pressures_pa(0.0)
    assert abs(p_pos - p_neg) < 1e-4, "Positive and negative pressures must equalize at top-out"


def test_adiabatic_polytropic_law_consistency():
    """Verifies that gas compression strictly follows P * V^gamma = const across stroke."""
    spring = ForkAirSpring(num_tokens=2, gauge_pressure_psi=85.0)
    gamma = spring.specs.gamma
    p_abs_0 = spring.abs_pressure_pa
    v_pos_0, v_neg_0 = spring.compute_volumes(0.0)

    const_pos = p_abs_0 * (v_pos_0 ** gamma)
    const_neg = p_abs_0 * (v_neg_0 ** gamma)

    for travel in [30.0, 54.0, 90.0, 135.0, 180.0]:
        v_pos, v_neg = spring.compute_volumes(travel)
        p_pos, p_neg = spring.compute_pressures_pa(travel)

        assert abs(p_pos * (v_pos ** gamma) - const_pos) / const_pos < 1e-5
        assert abs(p_neg * (v_neg ** gamma) - const_neg) / const_neg < 1e-5


def test_token_displacement_and_progressive_rampup():
    """Verifies that adding volume tokens increases end-stroke ramp-up and bottom-out force."""
    spring = ForkAirSpring()
    psi_calib = spring.calibrate_psi_for_sag(target_sag_mm=54.0, target_axial_force_n=322.2, num_tokens=2)

    forces_at_180 = []
    for tok in range(5):
        s = ForkAirSpring(num_tokens=tok, gauge_pressure_psi=psi_calib)
        f_180 = s.compute_axial_force(180.0)
        forces_at_180.append(f_180)

    # Each additional token must strictly increase bottom-out resistance
    for i in range(len(forces_at_180) - 1):
        assert forces_at_180[i + 1] > forces_at_180[i], (
            f"Token {i+1} force {forces_at_180[i+1]:.1f} N should be greater than token {i} force {forces_at_180[i]:.1f} N"
        )

    # 4 tokens should provide > 2x the bottom-out resistance of 0 tokens
    assert forces_at_180[4] > forces_at_180[0] * 2.0


def test_psi_sag_calibration():
    """Verifies that calibrate_psi_for_sag achieves exact target equilibrium load at sag."""
    spring = ForkAirSpring()
    target_sag = 54.0  # mm
    target_load = 322.2  # N

    for tok in [0, 1, 2, 3, 4]:
        calib_psi = spring.calibrate_psi_for_sag(target_sag_mm=target_sag, target_axial_force_n=target_load, num_tokens=tok)
        assert 60.0 <= calib_psi <= 110.0, f"Calibrated PSI {calib_psi:.1f} out of reasonable MTB range"

        actual_force = spring.compute_axial_force(target_sag, num_tokens=tok)
        assert abs(actual_force - target_load) < 0.5, (
            f"Expected {target_load} N at sag with {tok} tokens, got {actual_force:.2f} N"
        )


def test_playground_air_spring_integration(tmp_path):
    """Verifies that SuspensionPlayground initializes air spring, handles key events, and telemetry."""
    specs = BikeSpecs()
    pg = SuspensionPlayground(specs=specs)

    # Verify air spring exists
    assert hasattr(pg, "air_spring")
    assert pg.air_spring.num_tokens == 2

    # Check telemetry includes air metrics
    tel = pg.get_telemetry()
    assert "fork_air_psi" in tel
    assert "fork_tokens" in tel
    assert "fork_k_n_mm" in tel
    assert tel["fork_tokens"] == 2
    assert 70.0 <= tel["fork_air_psi"] <= 100.0

    # Key [ (decrease token)
    pg.handle_key(91)
    assert pg.air_spring.num_tokens == 1
    assert pg.get_telemetry()["fork_tokens"] == 1

    # Key ] (increase token)
    pg.handle_key(93)
    assert pg.air_spring.num_tokens == 2

    # Key + (increase PSI)
    psi_before = pg.air_spring.gauge_pressure_psi
    pg.handle_key(61)
    assert abs(pg.air_spring.gauge_pressure_psi - (psi_before + 2.0)) < 1e-4

    # Key - (decrease PSI)
    pg.handle_key(45)
    assert abs(pg.air_spring.gauge_pressure_psi - psi_before) < 1e-4


def test_volume_exhaustion_raises_error():
    """Verifies that attempting to compress beyond physical air chamber volume raises ValueError."""
    # Create an air spring with extremely short chamber where 180mm travel displaces more volume than exists
    tiny_specs = AirSpringSpecs(pos_chamber_length_mm=100.0, total_travel_mm=180.0)
    spring = ForkAirSpring(specs=tiny_specs, num_tokens=4)

    with pytest.raises(ValueError, match="Pneumatic volume exhausted"):
        spring.compute_volumes(travel_mm=150.0)
