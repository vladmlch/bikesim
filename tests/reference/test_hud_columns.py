"""Physical HUD exposes arm effort, geometric lean and latched balance state."""
from types import SimpleNamespace

import numpy as np
import pytest

from bike_sim.sim.ride.hud import RideHUD


def _columns(balance, *, shoulder=20., shoulder_capacity=100.):
    channels = {'tires':{'front':{'normal_load_n':300.},'rear':{'normal_load_n':500.}},
        'drive':{},'rider':{},'rider_joint_torques':{'rider_shoulder_left':shoulder,'rider_elbow_left':-20.},
        'rider_joint_capacity_nm':{'rider_shoulder_left':shoulder_capacity,'rider_elbow_left':100.},
        'rider_lean_limit_rad':np.pi/6,'rider_balance':balance}
    sim = SimpleNamespace(physical=SimpleNamespace(live_real_time_factor=1.,
        sample=SimpleNamespace(channels=channels,time_s=1.,qvel=np.zeros(1))),
        physics_config=SimpleNamespace(drive_mode='articulated_effort'),root_x_dofadr=0,
        pitch_rad=0.,model=None)
    hud = RideHUD.__new__(RideHUD)
    return hud._physical_columns(sim)


@pytest.mark.parametrize('state,want,style', [
    ({'balance_lost':False,'low_speed_s':0.},'ok','green'),
    ({'balance_lost':False,'low_speed_s':.3},'low 0.3s','yellow'),
    ({'balance_lost':True,'low_speed_s':.5,'balance_lost_at_m':12.},'LOST@x=12 m','red'),
])
def test_arm_lean_limit_and_balance_columns(state,want,style):
    columns = _columns(state)
    text = ''.join(text for _,segments in columns for text,_ in segments)
    assert 'arm=' in text
    assert '/lim30°' in text
    assert 'BAL' in text
    assert want in text
    balance = dict(columns)['BAL']
    assert any(want in text and color==style for text,color in balance)
    labels = [name for name,_ in columns]
    assert labels.index('bar l/r') < labels.index('arm') < labels.index('saddle')
    assert labels.index('lean') < labels.index('BAL')


def test_arm_highlight_uses_published_directional_capacity():
    columns = _columns({'balance_lost':False,'low_speed_s':0.},shoulder=90.)
    assert any('sh' in text and style=='orange1' for text,style in dict(columns)['arm'])
    columns = _columns({'balance_lost':False,'low_speed_s':0.},shoulder=90.,shoulder_capacity=200.)
    assert not any(style=='orange1' for _,style in dict(columns)['arm'])
