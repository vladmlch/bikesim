import pytest
from bike_sim.validation.rigs import radial_rig,wheel_inertia_rig,incline_rig,brake_rig,chain_locked_rig,chain_geometry_rig

@pytest.mark.parametrize('load',[100.,300.,600.,1000.])
def test_radial_engine_matches_analytic_deflection(load):
    result=radial_rig(load,.00025)
    assert result['deflection_m']==pytest.approx(load/130000.,rel=.02,abs=.0001)
    assert abs(result['speed_mps'])<1e-6

@pytest.mark.parametrize('dt',[.0005,.00025,.000125])
def test_isolated_inertia_and_transmission(dt):
    assert wheel_inertia_rig(dt)['acceleration_relative_error']<.001
    r=chain_locked_rig(dt)
    assert r['speed_ratio']==pytest.approx(r['expected_ratio'],rel=.001)
    assert r['torque_ratio']==pytest.approx(r['expected_ratio'],rel=.001)
    assert r['power_relative_error']<.005


def test_inclined_normal_is_not_vertical_load():
    r=incline_rig(.00025)
    assert r['normal_load_n']==pytest.approx(r['expected_normal_load_n'],rel=.005)
    assert r['vertical_with_stand_reaction_n']==pytest.approx(r['expected_weight_n'],rel=.005)
    assert r['normal_vertical_n']<r['normal_load_n']


def test_brake_holds_then_yields():
    r=brake_rig(.00025)
    assert abs(r['held_angle_rad'])<.001
    assert r['overload_speed_rad_s']>.5
    assert r['brake_work_j']<=0.


def test_chain_rigid_motion_and_virtual_work():
    r=chain_geometry_rig()
    assert abs(r['rigid_motion_extension_m'])<1e-12
    assert r['directional_relative_error']<1e-7


def test_real_linkage_chain_reaction_and_scratch_oracle():
    from bike_sim.validation.rigs import compiled_chain_motion_rig
    result=compiled_chain_motion_rig()
    assert result['compiled_jacobian_relative_error'] < 1e-7
    assert result['linkage_force_per_tension_m'] > 1e-5


def test_internal_rider_actuation_changes_limbs_but_preserves_total_momentum():
    from bike_sim.validation.rider_cases import airborne_internal_actuation
    metrics,criteria=airborne_internal_actuation(.0005)
    assert all(lo<=metrics[name]<=hi for name,(lo,hi) in criteria.items())
