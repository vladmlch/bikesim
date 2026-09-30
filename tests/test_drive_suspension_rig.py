import pytest
from bike_sim.validation.drive_suspension_rig import _fixture,drive_suspension_rig

@pytest.mark.parametrize('stroke',[.005,.015,.03])
def test_fixture_closure_at_declared_positions(stroke):
    m,d,drive,suspension,error=_fixture(.0003125,'geometric_ideal_mid_drive',stroke)
    assert error<1e-7
    assert d.qpos[m.joint('shock_stroke').qposadr[0]]==pytest.approx(stroke)
    assert d.time==0 and not m.geom_contype.any()

@pytest.mark.parametrize('mode',['ideal_mid_drive','geometric_ideal_mid_drive','elastic_chain'])
def test_rig_has_identical_delivered_torque_not_controller_request(mode):
    metrics,bounds=drive_suspension_rig(.0003125,mode,20.,duration_s=.005)
    assert metrics['mean_delivered_torque_nm']==pytest.approx(20.)
    assert all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())
