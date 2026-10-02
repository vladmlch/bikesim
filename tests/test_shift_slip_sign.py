from dataclasses import replace
import pytest
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.shifting import CadenceShifter


@pytest.mark.parametrize('slip,allowed', [
    (-.8, False), (.8, False), (-.2, True), (.2, True),
    (-.5, True), (.5, True), (0., True), (None, True),
])
def test_new_gate_uses_slip_magnitude(slip, allowed):
    cfg = load_physics_config('examples/research/viewer_physics_welded.toml')
    shift = replace(cfg.drive.shifting, upshift_slip_mode='magnitude')
    shifter = CadenceShifter(cfg.drive.gearing, shift)
    assert shifter.update(120., 120., .01, rear_slip_mps=slip) is allowed
    assert shifter.rear_teeth == (45 if allowed else 51)


@pytest.mark.parametrize('slip,allowed', [(-.8, True), (.8, False)])
def test_legacy_profile_remains_reproducible(slip, allowed):
    cfg = load_physics_config('examples/research/viewer_physics_welded.toml')
    assert cfg.drive.shifting.upshift_slip_mode == 'legacy_signed'
    shifter = CadenceShifter(cfg.drive.gearing, cfg.drive.shifting)
    assert shifter.update(120., 120., .01, rear_slip_mps=slip) is allowed


def test_invalid_upshift_slip_mode_raises():
    from bike_sim.physics.physical_config import ShiftingConfig
    with pytest.raises(ValueError, match='unknown upshift slip mode'):
        ShiftingConfig(upshift_slip_mode='invalid')
