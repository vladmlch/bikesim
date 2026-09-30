import pytest
from bike_sim.validation.load_transfer import quasistatic_front_load,load_transfer_rig


def test_centered_half_weight_and_acceleration_unloads_front():
    rest=quasistatic_front_load(100.,1.2,.6,.9,0.)
    assert rest==pytest.approx(490.5)
    assert rest-quasistatic_front_load(100.,1.2,.6,.9,0.,1.)==pytest.approx(75.)


def test_negative_reference_is_not_clamped():
    assert quasistatic_front_load(100.,1.2,.2,1.,0.,3.)<0


@pytest.mark.parametrize('slope,x,h',[(0.,.6,.9),(.1,.5,.8),(-.1,.7,1.)])
def test_free_pitch_two_support_fixture(slope,x,h):
    metrics,bounds=load_transfer_rig(.000625,slope,x,h)
    assert metrics['hidden_stand_pitch_constraints']==0
    for name,(lo,hi) in bounds.items():assert lo<=metrics[name]<=hi
