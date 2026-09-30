import pytest
from bike_sim.validation.contact_manifold_rigs import distributed_incline_rig
from bike_sim.validation.contact_resolution import contact_comparison,_events

@pytest.mark.parametrize('angle',[5.,15.,30.])
def test_distributed_incline_normal_and_complete_vertical_support(angle):
    metrics,bounds=distributed_incline_rig(.000625,128,angle)
    assert all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())
    assert metrics['normal_load_n']<metrics['total_vertical_support_n']

def test_comparison_does_not_hide_categorical_contact_loss():
    base=dict(normal_impulse_ns=100.,vertical_impulse_ns=100.,energy_residual_ratio=.001,
        native_wheel_contact_count=0,contact_events=[dict(state='loaded',time_s=0.)])
    changed={**base,'contact_events':base['contact_events']+[dict(state='unloaded',time_s=.2)]}
    assert not contact_comparison(base,changed)['passed']
