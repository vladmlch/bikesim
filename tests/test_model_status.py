import pytest
from bike_sim.sim.research.validity import channel_violations, model_violations
from bike_sim.sim.ride.model_status import ModelStatus


def channels(**front):
    tire = dict(unloaded_radius_m=.35,penetration_m=.001,normal_load_n=400.,
                multi_support=False,supports_multiple_contacts=False,patches=())
    return {'tires':{'front':dict(tire,**front),'rear':dict(tire)}}


def test_compression_uses_unloaded_radius():
    assert 'front:tire_compression' in channel_violations(channels(penetration_m=.06))


def test_multisupport_requires_capability():
    assert 'front:multi_support' in channel_violations(channels(multi_support=True))
    assert not channel_violations(channels(multi_support=True,supports_multiple_contacts=True))


def test_missing_radius_is_not_silently_accepted():
    c=channels(); del c['tires']['front']['unloaded_radius_m']
    assert 'front:missing_radius' in channel_violations(c)


def test_once_per_interval_reset_and_no_mutation():
    import copy
    c=channels(multi_support=True); before=copy.deepcopy(c); status=ModelStatus()
    status.observe(0,0.,c)
    assert c==before
    assert not status.as_dict()['model_valid']
    assert status.as_dict()['numerically_valid']=='not_evaluated'
    with pytest.raises(ValueError):status.observe(0,0.,c)
    result=status.as_dict(); result['first_model_violation']['reasons'].clear()
    assert status.first['reasons']
    status.reset(); assert status.as_dict()['model_valid']
    status.observe(0,0.,channels())


def test_custom_threshold_and_catch_plane():
    assert not channel_violations(channels(penetration_m=.06),.2)
    assert 'front:catch_plane' in channel_violations(channels(patches=[{'source_geom':'catch_plane','normal_load_n':1.}]))
    with pytest.raises(ValueError):channel_violations(channels(),float('nan'))
