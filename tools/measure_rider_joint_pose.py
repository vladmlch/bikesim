"""Measure the articulated_planar rider's q=0 joint geometry and emit an
anatomical joint-envelope JSON with verified bounds.

Coordinate conventions (all planar, X-Z, radians):

- torso   : forward lean of torso segment vs pelvis segment (+ = flexion)
- shoulder: arm elevation from hanging along the torso line (+ = elevation)
- elbow   : forearm deviation from the upper-arm line (+ = flexion)
- hip     : thigh deviation from the pelvis-down line (+ = flexion)
- knee    : shank deviation from the thigh line (+ = flexion; 0 = straight)
- ankle   : interior vertex angle shank->foot (+ = toward plantar pointe)

Each coordinate a satisfies a(q) = neutral + direction*q; neutral and
direction are measured on the compiled model by sweeping the hinge.
"""
import json
import numpy as np
import mujoco

from bike_sim.physics.resolution import load_physics_config
from bike_sim.mujoco.builder import generate_mujoco_xml

cfg = load_physics_config('examples/research/viewer_physics_fast.toml')
xml = generate_mujoco_xml(
    mode='ride', rider='articulated_planar',
    physics_config=cfg, crank_joint=True,
)
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)


def seg_dir(name):
    """Segment direction start->end in X-Z (geom axis is end->start)."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    axis = np.asarray(data.geom_xmat[gid]).reshape(3, 3)[:, 2]
    return -np.array([axis[0], axis[2]])


def signed_angle(a, b):
    return np.arctan2(a[1] * b[0] - a[0] * b[1], float(np.dot(a, b)))


SEG = {
    'pelvis': 'geom_rider_pelvis', 'torso': 'geom_rider_torso',
    'arm': 'geom_rider_upper_arm_pair', 'forearm': 'geom_rider_forearm_pair',
}
for side in ('front', 'rear'):
    for part in ('thigh', 'shank', 'foot'):
        SEG[f'{part}_{side}'] = f'geom_rider_{part}_{side}'

GEOMS = {k: None for k in SEG}


def refresh():
    mujoco.mj_kinematics(model, data)
    for k, g in SEG.items():
        GEOMS[k] = seg_dir(g)


def anatomical(name):
    p, t, arm, fo = GEOMS['pelvis'], GEOMS['torso'], GEOMS['arm'], GEOMS['forearm']
    if name == 'rider_torso_hinge':
        return signed_angle(p, t)
    if name == 'rider_shoulder':
        return signed_angle(arm, -t)
    if name == 'rider_elbow':
        return signed_angle(fo, arm)
    side = name.rsplit('_', 1)[1]
    th, sh, ft = GEOMS[f'thigh_{side}'], GEOMS[f'shank_{side}'], GEOMS[f'foot_{side}']
    if f'rider_hip_{side}' == name:
        return signed_angle(th, -p)
    if f'rider_knee_{side}' == name:
        return signed_angle(th, sh)
    if f'rider_ankle_{side}' == name:
        return signed_angle(-sh, ft)
    raise KeyError(name)


# anatomical bounds (radians) per joint name
BOUNDS = {
    'rider_torso_hinge': (-0.50, 1.30),    # trunk-on-pelvis flexion  -29..+74 deg
    'rider_shoulder': (-0.87, 2.96),       # elevation from hanging   -50..+170 deg
    'rider_elbow': (0.0, 2.62),            # flexion                  0..150 deg
    'rider_hip_front': (-0.35, 2.36),      # flexion                  -20..+135 deg
    'rider_knee_front': (0.0, 2.44),       # flexion, 0 = straight    0..140 deg
    'rider_ankle_front': (0.96, 3.10),     # vertex angle 55..177 deg (wraps at pi)
    'rider_hip_rear': (-0.35, 2.36),
    'rider_knee_rear': (0.0, 2.44),
    'rider_ankle_rear': (0.96, 3.10),
}

JOINTS = list(BOUNDS)
data.qpos[:] = 0.0
refresh()
qpos0 = data.qpos.copy()

result = {}
for name in JOINTS:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    qa = int(model.jnt_qposadr[jid])
    vals = {}
    for q in (-0.4, 0.0, 0.4):
        data.qpos[:] = qpos0
        data.qpos[qa] = q
        refresh()
        vals[q] = anatomical(name)
    wrap = lambda d: np.arctan2(np.sin(d), np.cos(d))
    slope = (wrap(vals[0.4] - vals[0.0]) - wrap(vals[-0.4] - vals[0.0])) / 0.8
    direction = int(np.sign(slope))
    assert abs(abs(slope) - 1.0) < 1e-6, f'{name}: non-unit slope {slope}'
    neutral = vals[0.0]
    lo, hi = BOUNDS[name]
    pairs = [(lo - neutral) / direction, (hi - neutral) / direction]
    q_lo, q_hi = min(pairs), max(pairs)
    # verify: put the hinge at each q bound and re-measure the anatomical angle
    for a_expect, q_expect in ((lo, (lo - neutral) / direction),
                               (hi, (hi - neutral) / direction)):
        data.qpos[:] = qpos0
        data.qpos[qa] = q_expect
        refresh()
        a_meas = anatomical(name)
        assert abs(a_meas - a_expect) < 1e-6, (name, a_meas, a_expect)
    result[name] = {
        'neutral_anatomical_rad': round(float(neutral), 6),
        'direction': direction,
        'minimum_anatomical_rad': lo,
        'maximum_anatomical_rad': hi,
    }
    print(f"{name:20s} neutral={neutral:+.4f} rad dir={direction:+d} "
          f"bounds=[{lo},{hi}] -> q_deg="
          f"[{float(np.degrees(q_lo)):+.2f},{float(np.degrees(q_hi)):+.2f}]")
    data.qpos[:] = qpos0

profile = {
    'schema_version': 1,
    'provenance': 'sagittal-plane goniometry bounds vs measured q=0 build pose; '
                  'adult ROM norms, not subject-measured',
    'joints': {
        name: {
            'neutral_anatomical_rad': r['neutral_anatomical_rad'],
            'direction': r['direction'],
            'minimum_anatomical_rad': r['minimum_anatomical_rad'],
            'maximum_anatomical_rad': r['maximum_anatomical_rad'],
            'provenance': 'standard adult ROM (AAOS goniometry) mapped to the '
                          'articulated_planar build pose; planar only',
        }
        for name, r in result.items()
    },
}
with open('examples/research/rider_joint_envelope_anatomical.json', 'w') as fh:
    json.dump(profile, fh, indent=2)
    fh.write('\n')
print('wrote examples/research/rider_joint_envelope_anatomical.json')
