"""Directional joint-torque strength curves for the articulated rider.

A Hill-type *joint moment* envelope, not a muscle model: each internal joint
has one isometric torque curve per torque direction (positive and negative q),
interpolated over the anatomical angle and scaled by a force-velocity factor.
Concentric (shortening) work loses capacity toward zero at ``vmax``; eccentric
work is bounded by ``eccentric_ratio`` and can never supply positive power.

The strength profile JSON is self-contained: per joint it declares the
q-to-anatomical coordinate convention (the same neutral+direction mapping as
``JointEnvelope``), the used anatomical range every curve must cover, and one
curve per direction with provenance and a ``verified`` flag. Profiles that
mark unverified data are loadable for engineering use but are rejected by
``require_verified=True`` -- the release gate, not the loader's leniency,
decides whether a profile is biomechanically validated.
"""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from bike_sim.physics.rider_envelope import JointEnvelope


@dataclass(frozen=True)
class TorqueCurve:
    angles_rad: tuple[float, ...]
    torques_nm: tuple[float, ...]
    vmax_rad_s: float
    hill_c: float
    eccentric_ratio: float
    source: str

    def __post_init__(self):
        if len(self.angles_rad) < 2 or len(self.angles_rad) != len(self.torques_nm):
            raise ValueError('strength curve requires paired angle/torque knots')
        values = (*self.angles_rad, *self.torques_nm, self.vmax_rad_s,
                  self.hill_c, self.eccentric_ratio)
        if not np.isfinite(values).all() or any(t < 0 for t in self.torques_nm):
            raise ValueError('invalid strength data')
        if any(b <= a for a, b in zip(self.angles_rad, self.angles_rad[1:])):
            raise ValueError('angles must increase')
        if self.vmax_rad_s <= 0 or self.hill_c <= 0 or self.eccentric_ratio < 1:
            raise ValueError('invalid force-velocity parameters')
        if not self.source.strip():
            raise ValueError('missing strength provenance')


def directional_capacity(curve: TorqueCurve, angle_rad: float,
                         velocity_rad_s: float, direction: int) -> float:
    """Bounded active torque magnitude for one joint direction, in N.m.

    ``velocity_rad_s`` is the joint's q-velocity; ``direction`` is the sign of
    the torque being evaluated (+1 or -1). Motion along the torque is
    concentric and loses capacity on the Hill factor; motion against it is
    eccentric and gains at most ``eccentric_ratio`` times the isometric value.
    """
    if direction not in (-1, 1) or not np.isfinite([angle_rad, velocity_rad_s]).all():
        raise ValueError('invalid joint state or direction')
    if not curve.angles_rad[0] <= angle_rad <= curve.angles_rad[-1]:
        raise ValueError('strength evaluation outside documented angle range')
    iso = float(np.interp(angle_rad, curve.angles_rad, curve.torques_nm))
    speed = direction * velocity_rad_s / curve.vmax_rad_s
    if speed >= 0:
        factor = max(0., (1. - speed) / (1. + speed / curve.hill_c))
    else:
        factor = 1. + (curve.eccentric_ratio - 1.) * (-speed) / (1. - speed)
    return iso * factor


def _torque_curve(payload, joint, direction_label):
    if not isinstance(payload, dict):
        raise ValueError(f'{joint}:{direction_label} strength curve must be an object')
    if set(payload) != {'angles_rad', 'torques_nm', 'vmax_rad_s', 'hill_c',
                        'eccentric_ratio', 'source', 'verified', 'label'}:
        raise ValueError(f'{joint}:{direction_label} strength curve has unknown or missing fields')
    if not isinstance(payload['label'], str) or not payload['label'].strip():
        raise ValueError(f'{joint}:{direction_label} anatomical direction label is required')
    if type(payload['verified']) is not bool:
        raise ValueError(f'{joint}:{direction_label} verified flag must be a boolean')
    try:
        curve = TorqueCurve(
            angles_rad=tuple(float(v) for v in payload['angles_rad']),
            torques_nm=tuple(float(v) for v in payload['torques_nm']),
            vmax_rad_s=float(payload['vmax_rad_s']),
            hill_c=float(payload['hill_c']),
            eccentric_ratio=float(payload['eccentric_ratio']),
            source=str(payload['source']))
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{joint}:{direction_label} invalid strength curve') from exc
    return curve, payload['verified']


def load_strength_profile(path: str, expected_joints: tuple[str, ...], *,
                          require_verified: bool) -> dict[str, dict[int, TorqueCurve]]:
    """Load one directional TorqueCurve pair per internal rider joint.

    The returned mapping is ``{joint_name: {+1: curve, -1: curve}}`` keyed by
    the sign of the q-torque the curve bounds. Every curve's knot range must
    cover the joint's declared ``used_anatomical_range_rad``; every joint must
    carry both directions. ``require_verified=True`` rejects any curve whose
    provenance is not confirmed, so unverified working data can never pass a
    validation gate by silence.
    """
    payload = json.loads(Path(path).read_text())
    if not isinstance(payload, dict) or payload.get('schema_version') != 1:
        raise ValueError('unknown strength profile schema')
    if payload.get('topology') != 'articulated_planar_two_arm':
        raise ValueError('strength profile names a different rider topology')
    if set(payload) != {'schema_version', 'topology', 'provenance', 'joints'}:
        raise ValueError('strength profile has unknown or missing fields')
    joints = payload['joints']
    if not isinstance(joints, dict) or not set(expected_joints) <= set(joints):
        raise ValueError('strength profile must cover the present internal rider joints')
    result = {}
    for name in expected_joints:
        entry = joints[name]
        if not isinstance(entry, dict) or set(entry) != {
                'coordinate', 'used_anatomical_range_rad', 'population',
                'scaling', 'directions'}:
            raise ValueError(f'{name}: unknown or missing strength entry fields')
        coordinate = load_strength_coordinate(entry, name)
        low, high = (float(v) for v in entry['used_anatomical_range_rad'])
        if not (np.isfinite([low, high]).all() and low < high):
            raise ValueError(f'{name}: invalid used anatomical range')
        if not coordinate.minimum_anatomical_rad <= coordinate.neutral_anatomical_rad <= coordinate.maximum_anatomical_rad:
            raise ValueError(f'{name}: neutral angle outside the used range')
        directions = entry['directions']
        if set(directions) != {'positive', 'negative'}:
            raise ValueError(f'{name}: exactly two torque directions are required')
        curves = {}
        for label, sign in (('positive', 1), ('negative', -1)):
            curve, verified = _torque_curve(directions[label], name, label)
            if curve.angles_rad[0] > low or curve.angles_rad[-1] < high:
                raise ValueError(f'{name}:{label} curve does not cover the used ROM')
            if require_verified and not verified:
                raise ValueError(f'{name}:{label} strength data is not verified')
            curves[sign] = curve
        result[name] = curves
    return result


def load_strength_coordinate(entry: dict, name: str) -> JointEnvelope:
    """The q->anatomical convention one strength entry declares for itself."""
    coordinate = entry['coordinate']
    if not isinstance(coordinate, dict) or set(coordinate) != {
            'neutral_anatomical_rad', 'direction'}:
        raise ValueError(f'{name}: invalid coordinate convention')
    low, high = entry['used_anatomical_range_rad']
    return JointEnvelope(
        neutral_anatomical_rad=float(coordinate['neutral_anatomical_rad']),
        direction=int(coordinate['direction']),
        minimum_anatomical_rad=float(low),
        maximum_anatomical_rad=float(high),
        provenance=str(entry['population']))


def load_strength_coordinates(path: str,
                              expected_joints: tuple[str, ...]) -> dict[str, JointEnvelope]:
    """Per-joint q->anatomical mapping carried inside the strength file."""
    payload = json.loads(Path(path).read_text())
    joints = payload['joints']
    if not set(expected_joints) <= set(joints):
        raise ValueError('strength profile must cover the present internal rider joints')
    return {name: load_strength_coordinate(joints[name], name)
            for name in expected_joints}
