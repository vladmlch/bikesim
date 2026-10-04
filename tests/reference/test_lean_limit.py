"""ROM changes must change the attainable geometric lean."""
from dataclasses import replace
from math import pi
from pathlib import Path
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.physics.rider_envelope import load_joint_envelopes


def _inputs():
    pose = geometry_pose(RiderSpecs(variant='articulated_planar'), BikeSpecs())
    envelopes = load_joint_envelopes(str(Path('examples/research/rider_joint_envelope_anatomical.json')))
    radius = float(np.linalg.norm(pose.pedal_front-pose.pedal_rear)/2)
    return pose, envelopes, radius


def test_lean_limit_is_geometric_and_elbow_rom_can_lower_it():
    from bike_sim.sim.ride.lean_limit import lean_limit_rad
    pose, envelopes, radius = _inputs()
    limit = lean_limit_rad(pose, envelopes, crank_m=radius, tdc_phase_rad=-pi/2)
    assert .1 < limit < .8
    tight = dict(envelopes)
    for side in ('left', 'right'):
        name = 'rider_elbow_'+side
        tight[name] = replace(envelopes[name], maximum_anatomical_rad=envelopes[name].neutral_anatomical_rad+.2)
    assert lean_limit_rad(pose, tight, crank_m=radius, tdc_phase_rad=-pi/2) < limit


def test_torso_rom_caps_the_limit():
    from bike_sim.sim.ride.lean_limit import lean_limit_rad
    pose, envelopes, radius = _inputs()
    original = envelopes['rider_torso_hinge']
    envelopes['rider_torso_hinge'] = replace(original, maximum_anatomical_rad=original.neutral_anatomical_rad+.3)
    assert lean_limit_rad(pose, envelopes, crank_m=radius, tdc_phase_rad=-pi/2) == pytest.approx(.3, abs=.001)
