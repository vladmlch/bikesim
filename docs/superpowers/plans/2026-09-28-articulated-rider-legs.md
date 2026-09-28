# Articulated Rider Legs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the rigid slide-mounted leg clusters with articulated, actuated legs (hip/knee/ankle hinges) welded to real pedal bodies, so pedalling looks real and physically drives the cranks.

**Architecture:** `physics/rider.py` gains a `legs="rigid"|"articulated"` variant on `RiderSpecs`/`solve_seated_pose`; `mujoco/` emits a thigh→shank→foot hinge chain per leg, pedal bodies on passive spindle hinges, and foot↔pedal `weld` equalities; a new `sim/ride/leg_drive.py` writer turns `PedalDrivetrain`'s rider-torque command into per-leg joint torques (Jacobian force map + low-gain impedance toward per-step IK), replacing `_follow_cranks`/`set_pedal_offsets` in the articulated variant.

**Tech Stack:** MuJoCo 3.12 (weld equalities, hinge chains), numpy, pytest. Run everything with `uv run`.

**Spec:** `docs/superpowers/specs/2026-09-28-articulated-rider-legs-design.md`

## Global Constraints

- Use `uv run` for all Python (`AGENTS.md`).
- Model is generated, not authored XML — builders in `src/bike_sim/mujoco/`, golden snapshots in `tests/golden/`.
- Rider geoms stay non-colliding (`contype="0" conaffinity="0"`).
- `legs` defaults to `"rigid"` everywhere so all existing goldens/behaviour are untouched; `articulated` is opt-in per mode (default in `bike-ride` pedal|pedelec).
- Deviation from spec §3.4, deliberate: joint torques are written to `qfrc_applied` (the `rider_forces.py` convention for rider forces), not motor actuators — no `ctrlrange` sizing needed, identical dynamics, one write path. All six leg DOFs are assigned every step.
- DOF accounting: rigid pedalled `nq=18` → articulated pedalled `nq=24` (−2 leg slides, +6 leg hinges, +2 pedal hinges, +1 `crank_spin` already counted).
- Weld datum: `weld` with no `relpose` captures the `qpos0` relative pose (verified on MuJoCo 3.12). Legs and pedals are built at the design pose (cranks horizontal, feet on pedals) → zero weld residual at `qpos0`.

---

### Task 1: Leg kinematics in the pose solver (`physics/rider.py`)

Pure-math foundation: the `LegChain` data a leg needs (design geometry, segment lengths, masses), plus the per-phase IK/Jacobian helpers the runtime controller calls. No MuJoCo.

**Files:**
- Modify: `src/bike_sim/physics/rider.py`
- Test: `tests/test_leg_kinematics.py` (new)

**Interfaces:**
- Produces (consumed by Tasks 2–5):
  - `RiderSpecs.legs: str = "rigid"` — new field; `RiderSpecs.__post_init__` validates `legs in ("rigid", "articulated")`.
  - `solve_seated_pose(specs, rider)` — unchanged signature; reads `rider.legs`. With `legs="articulated"`, `pose.bodies` omits the two leg `RiderBody` entries and `pose.leg_chains` is populated instead.
  - `SeatedPose.leg_chains: Tuple[LegChain, ...]` — `()` for rigid, two entries for articulated.
  - `LegChain` dataclass (frozen): `side: str` (`"front"`/`"rear"`), `lateral_y_m: float`, `crank_len_m: float`, `hip/knee/ankle/pedal: np.ndarray` (BB frame, design pose, y=0 sagittal — the lateral offset lives in `lateral_y_m`), `thigh_len_m`, `shank_len_m`, `thigh_mass_kg`, `shank_mass_kg`, `foot_mass_kg`, `pitch0_thigh_rad`, `pitch0_shank_rad`, `pitch0_foot_rad`.
  - `fk_leg(chain: LegChain, qpos: np.ndarray, pelvis_z_m: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]` — forward kinematics from hinge angles: `(knee, ankle, pedal_point)` in the sagittal plane, used by `leg_jacobian` (never re-solve IK for J).
  - `pedal_spindle_pos(phase_rad: float, lateral_y_m: float, crank_len_m: float) -> np.ndarray` — frame-frame `(L·cosφ, y, −L·sinφ)`.
  - `solve_leg_qpos(chain: LegChain, phase_rad: float, pelvis_z_m: float = 0.0) -> np.ndarray` — `(q_hip, q_knee, q_ankle)` hinge targets.
  - `solve_leg_joints(chain, phase_rad, pelvis_z_m=0.0) -> Tuple[np.ndarray, np.ndarray]` — `(knee_xz, ankle_xz)` for tests/telemetry.
  - `leg_jacobian(chain: LegChain, qpos: np.ndarray, pelvis_z_m: float = 0.0) -> np.ndarray` — `(3, 2)`; row i maps pedal-spindle force `(fx, fz)` to hinge torque i.
  - `crank_torque_share(phase_i_rad: float, depth: float) -> float` — per-leg share of `rider_torque_nm`; satisfies `share(φ) + share(φ+π) == ripple_shape(φ, depth)` exactly.

- [ ] **Step 1: Write the failing tests**

`tests/test_leg_kinematics.py`:

```python
import numpy as np
import pytest
from math import pi

from bike_sim.physics.drivetrain import ripple_shape
from bike_sim.physics.rider import (
    ANKLE_ABOVE_PEDAL_M, RiderSpecs, crank_torque_share, pedal_spindle_pos,
    solve_leg_qpos, solve_leg_joints,
)


@pytest.fixture(scope="module")
def pose():
    return RiderSpecs(variant="seated", legs="articulated").seated_pose()


def test_articulated_pose_has_leg_chains_no_leg_bodies(pose):
    assert len(pose.leg_chains) == 2
    names = {b.name for b in pose.bodies}
    assert names == {"rider_pelvis", "rider_torso", "rider_arms"}


def test_rigid_pose_unchanged():
    pose = RiderSpecs(variant="seated").seated_pose()  # legs defaults to "rigid"
    assert pose.leg_chains == ()
    assert {b.name for b in pose.bodies} == {
        "rider_pelvis", "rider_torso", "rider_arms", "rider_leg_front", "rider_leg_rear"}


def test_design_pose_is_zero_qpos(pose):
    # The model is built at phase 0 (horizontal cranks): IK must reproduce qpos=0,
    # which is what makes the weld datum consistent at build time.
    for chain in pose.leg_chains:
        q = solve_leg_qpos(chain, phase_rad=0.0)
        np.testing.assert_allclose(q, np.zeros(3), atol=1e-9)


def test_ankle_tracks_pedal_circle(pose):
    phases = np.linspace(0.0, 2.0 * pi, 73)
    for chain in pose.leg_chains:
        for phi in phases:
            # must never be unreachable through a revolution
            knee, ankle = solve_leg_joints(chain, phi)
            pedal = pedal_spindle_pos(phi, chain.lateral_y_m, chain.crank_len_m)
            np.testing.assert_allclose(
                [ankle[0], ankle[2]], [pedal[0], pedal[2] + ANKLE_ABOVE_PEDAL_M],
                atol=1e-9)


def test_knee_flexion_stays_in_human_range(pose):
    # 20-60 deg is the accepted BDC band (physics/rider.py); allow margin for the
    # top of the stroke but refuse folded-back or locked-out knees.
    for chain in pose.leg_chains:
        flexions = []
        for phi in np.linspace(0.0, 2.0 * pi, 37):
            knee, ankle = solve_leg_joints(chain, phi)
            hip = chain.hip
            v1, v2 = hip - knee, ankle - knee
            flexions.append(180.0 - np.degrees(
                np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))))
        assert min(flexions) > 15.0 and max(flexions) < 75.0


def test_pelvis_slide_moves_the_hip(pose):
    # The hip anchor rides the pelvis slide joint: raising the pelvis must move
    # the IK knee up (ankle stays welded to its pedal circle).
    chain = pose.leg_chains[0]
    knee0, _ = solve_leg_joints(chain, pi / 2, pelvis_z_m=0.0)
    knee1, _ = solve_leg_joints(chain, pi / 2, pelvis_z_m=0.03)
    assert knee1[2] > knee0[2]


def test_torque_shares_sum_to_ripple():
    for depth in (0.0, 0.5, 0.85, 1.0):
        for phi in np.linspace(0.0, 2.0 * pi, 65):
            total = crank_torque_share(phi, depth) + crank_torque_share(phi + pi, depth)
            assert total == pytest.approx(ripple_shape(phi, depth), abs=1e-12)
        # each leg averages half the mean torque over a revolution
        mean = np.mean([crank_torque_share(p, depth) for p in np.linspace(0, 2 * pi, 1001)])
        assert mean == pytest.approx(0.5, abs=1e-3)


def test_leg_masses_match_rigid_path(pose):
    # The articulated leg carries the same pedal-path mass the slide leg did.
    rigid = RiderSpecs(variant="seated").seated_pose()
    rigid_leg = rigid.body("rider_leg_front").mass
    chain = pose.leg_chains[0]
    assert chain.thigh_mass_kg + chain.shank_mass_kg + chain.foot_mass_kg == pytest.approx(rigid_leg)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_leg_kinematics.py -v`
Expected: FAIL — `legs`/`leg_chains`/`solve_leg_qpos`/`pedal_spindle_pos`/`crank_torque_share` do not exist.

- [ ] **Step 3: Implement**

In `src/bike_sim/physics/rider.py`:

a) `RiderSpecs`: add field `legs: str = "rigid"`; in `__post_init__` add
```python
if self.legs not in ("rigid", "articulated"):
    raise ValueError(f"legs must be 'rigid' or 'articulated', got {self.legs!r}")
```

b) New dataclass + helpers, appended before `solve_seated_pose`:

```python
@dataclass(frozen=True)
class LegChain:
    """One articulated leg: design-pose geometry, segment lengths and masses.

    All points are in the BB frame at the design pose (cranks horizontal) on the
    sagittal plane; the leg itself solves in a plane offset by ``lateral_y_m``.
    """
    side: str
    lateral_y_m: float
    hip: np.ndarray
    knee: np.ndarray
    ankle: np.ndarray
    pedal: np.ndarray
    thigh_len_m: float
    shank_len_m: float
    thigh_mass_kg: float
    shank_mass_kg: float
    foot_mass_kg: float
    pitch0_thigh_rad: float
    pitch0_shank_rad: float
    pitch0_foot_rad: float


def _pitch_xz(vec: np.ndarray) -> float:
    """Pitch angle of a sagittal-plane direction about +y: +x forward is 0,
    straight down is +pi/2 (R_y maps +x toward -z)."""
    return float(np.arctan2(-vec[2], vec[0]))


def pedal_spindle_pos(phase_rad: float, lateral_y_m: float, crank_len_m: float) -> np.ndarray:
    """Pedal spindle centre in the frame's coordinates at a crank phase."""
    return np.array([crank_len_m * cos(phase_rad), lateral_y_m, -crank_len_m * sin(phase_rad)])


def solve_leg_joints(chain: LegChain, phase_rad: float, pelvis_z_m: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """Knee and ankle points for one crank phase, in the leg's sagittal plane."""
    ankle = pedal_spindle_pos(phase_rad, chain.lateral_y_m, chain.crank_len_m).copy()
    ankle[2] += ANKLE_ABOVE_PEDAL_M
    hip = chain.hip + np.array([0.0, 0.0, pelvis_z_m])
    knee = _two_link_ik(hip, ankle, chain.thigh_len_m, chain.shank_len_m, prefer="+x")
    return knee, ankle


def solve_leg_qpos(chain: LegChain, phase_rad: float, pelvis_z_m: float = 0.0) -> np.ndarray:
    """Hinge-angle targets (hip, knee, ankle) relative to the design pose."""
    knee, ankle = solve_leg_joints(chain, phase_rad, pelvis_z_m)
    hip = chain.hip + np.array([0.0, 0.0, pelvis_z_m])
    pedal = pedal_spindle_pos(phase_rad, chain.lateral_y_m, chain.crank_len_m)
    pitch = (_pitch_xz(knee - hip), _pitch_xz(ankle - knee), _pitch_xz(pedal - ankle))
    pitch0 = (chain.pitch0_thigh_rad, chain.pitch0_shank_rad, chain.pitch0_foot_rad)
    q = np.empty(3)
    q[0] = pitch[0] - pitch0[0]
    q[1] = (pitch[1] - pitch0[1]) - q[0]
    q[2] = (pitch[2] - pitch0[2]) - (q[0] + q[1])
    return q


def fk_leg(chain: LegChain, qpos: np.ndarray, pelvis_z_m: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Forward kinematics from hinge angles: (knee, ankle, pedal_point), sagittal plane.

    Each hinge rotates its chain about +y relative to the design pose: the
    absolute pitch of segment i is ``pitch0_i + sum(qpos[:i+1])`` (qpos adds onto
    the design-pose pitch, since qpos=0 is the design pose).
    """
    hip = chain.hip + np.array([0.0, 0.0, pelvis_z_m])
    p_t = chain.pitch0_thigh_rad + qpos[0]
    knee = hip + chain.thigh_len_m * np.array([cos(p_t), 0.0, -sin(p_t)])
    p_s = chain.pitch0_shank_rad + qpos[0] + qpos[1]
    ankle = knee + chain.shank_len_m * np.array([cos(p_s), 0.0, -sin(p_s)])
    p_f = chain.pitch0_foot_rad + float(qpos.sum())
    foot_len = float(np.linalg.norm(chain.pedal - chain.ankle))
    pedal_pt = ankle + foot_len * np.array([cos(p_f), 0.0, -sin(p_f)])
    return knee, ankle, pedal_pt


def leg_jacobian(chain: LegChain, qpos: np.ndarray, pelvis_z_m: float = 0.0) -> np.ndarray:
    """(3, 2): pedal-spindle force (fx, fz) -> hinge torques about +y.

    Torque about +y of a force F applied at point p relative to joint centre j is
    tau = r_z * F_x - r_x * F_z for r = p - j. Joints in order: hip, knee, ankle.
    """
    hip = chain.hip + np.array([0.0, 0.0, pelvis_z_m])
    knee, ankle, p = fk_leg(chain, qpos, pelvis_z_m)
    J = np.empty((3, 2))
    for i, j in enumerate((hip, knee, ankle)):
        r = p - j
        J[i] = (r[2], -r[0])   # tau_i = r_z * F_x - r_x * F_z
    return J


def crank_torque_share(phase_rad: float, depth: float) -> float:
    """One leg's share of mean crank torque; front+rear sums to `ripple_shape`."""
    return 0.5 * (1.0 - depth) + depth * (pi / 2.0) * max(0.0, cos(phase_rad))
```

Important design note for the implementer:

- The pitch convention `_pitch_xz` matches the hinge semantics: a +y hinge rotation maps +x toward −z, so a segment pointing straight down reads `+pi/2`. `fk_leg` and `solve_leg_qpos` must use the same convention — the tests (design pose → qpos 0, ankle on the pedal circle) catch a flipped sign immediately.
- `solve_seated_pose`: after `masses = seated_path_masses(rider)` add the split — shank/foot mass by de Leva fractions (`f["shank"]` vs `f["foot"]` of `masses["shank_foot"]`); when `rider.legs == "articulated"`, drop the two `leg(...)` entries from `bodies` and instead build `leg_chains` (front/right is `lateral_y_m=-PEDAL_LATERAL_OFFSET_M` per `drivetrain.py`'s `("front", pedal_front, -1.0)` — import `PEDAL_LATERAL_OFFSET_M` from `geometry.cockpit`). `pitch0_*` = `_pitch_xz` of each design segment (thigh: knee−hip; shank: ankle−knee; foot: pedal−ankle). `SeatedPose` gains `leg_chains: Tuple[LegChain, ...] = ()`.
- `RiderSpecs.seated_pose()` signature unchanged — it reads `self.legs`.
- `compute_rider_centers_of_mass`: when `pose.leg_chains`, add per-segment entries (`rider_thigh_{side}` at thigh midpoint, `rider_shank_{side}`, `rider_foot_{side}`) so `physics/mass.py`'s CG table stays correct without touching it.

- [ ] **Step 4: Run tests**

Run: `uv run pytest tests/test_leg_kinematics.py tests/test_ride_model.py tests/test_ride_equilibrium.py -x -q`
Expected: new tests PASS; seated-rider tests unchanged (rigid default).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/physics/rider.py tests/test_leg_kinematics.py
git commit -m "Add articulated-leg variant to the seated pose solver"
```

---

### Task 2: MJCF build — leg chains, pedal bodies, welds (`mujoco/`)

**Files:**
- Modify: `src/bike_sim/mujoco/rider.py`, `src/bike_sim/mujoco/drivetrain.py`, `src/bike_sim/mujoco/frame.py`, `src/bike_sim/mujoco/builder.py`
- Test: `tests/test_ride_model.py` (extend), `tests/test_golden_baselines.py` (new golden)

**Interfaces:**
- Consumes: `SeatedPose.leg_chains`, `LegChain` fields (Task 1).
- Produces: joints `rider_{hip,knee,ankle}_{front,rear}` (hinge, axis `"0 1 0"`); bodies `pedal_{front,rear}` with hinges `pedal_spin_{front,rear}`; equality welds `weld_foot_{front,rear}` (`body1="rider_foot_*"`, `body2="pedal_*"`); `generate_mujoco_xml` accepts the legs variant only via `rider.legs` (no new top-level param — `RiderSpecs` carries it).

- [ ] **Step 1: Failing tests**

Append to `tests/test_ride_model.py`:

```python
@pytest.fixture(scope="module")
def articulated_ride_model():
    from bike_sim.physics.rider import RiderSpecs
    rider = RiderSpecs(variant="seated", legs="articulated")
    xml = generate_mujoco_xml(mode="ride", rider=rider, crank_joint=True)
    return mujoco.MjModel.from_xml_string(xml)


def test_articulated_leg_topology(articulated_ride_model):
    m = articulated_ride_model
    # 12 bike DOF + 3 rider slides + 6 leg hinges + 2 pedal hinges + crank_spin = 24
    assert m.nq == 24 and m.nv == 24
    for jname in ("rider_hip_front", "rider_knee_front", "rider_ankle_front",
                  "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear",
                  "pedal_spin_front", "pedal_spin_rear", "crank_spin"):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        assert jid >= 0, jname
        assert m.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
    body = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    assert m.body_parentid[body("rider_thigh_front")] == body("rider_pelvis")
    assert m.body_parentid[body("rider_shank_front")] == body("rider_thigh_front")
    assert m.body_parentid[body("rider_foot_front")] == body("rider_shank_front")
    assert m.body_parentid[body("pedal_front")] == body("crank")
    # leg slide bodies are gone
    for jname in ("rider_leg_front_z", "rider_leg_rear_z"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname) < 0


def test_foot_pedal_welds_present_and_satisfied(articulated_ride_model):
    m = articulated_ride_model
    welds = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, i)
             for i in range(m.neq) if m.eq_type[i] == mujoco.mjtEq.mjEQ_WELD}
    assert {"weld_foot_front", "weld_foot_rear"} <= welds
    # built pose is the weld datum: forward at qpos0 must put each foot's weld
    # site on its pedal site (~zero separation)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        sep = float(np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]))
        assert sep < 0.001, f"{side}: {sep * 1000:.2f} mm"
```

The weld residual is measured as site distance: `site_foot_{side}` at the foot's pedal-spindle point (emitted in `build_articulated_legs`) and `site_pedal_{side}` at the pedal-body origin (emitted in `build_bb_and_motor`) — both land in this task.

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_ride_model.py -k articulated -x -q`
Expected: FAIL (no `rider_hip_front` joint / `pedal_front` body / welds).

- [ ] **Step 3: Implement**

`mujoco/drivetrain.py::build_bb_and_motor` — when `crank_joint` is true, each pedal box moves onto its own body:

```python
pedal_body = ET.SubElement(crankset, "body", {
    "name": f"pedal_{side}",
    "pos": f"{pedal[0]:.6f} {y_arm:.6f} {pedal[2]:.6f}",
})
add_joint(pedal_body, f"pedal_spin_{side}", "hinge", axis="0 1 0", damping="0.005")
add_geom(pedal_body, f"geom_pedal_{side}", "box",
         pos=f"0 {y_pedal - y_arm:.6f} 0", size=..., mass=PEDAL_MASS_KG, ...)
add_site(pedal_body, f"site_pedal_{side}", pos="0 0 0", size="0.006")
```
(`y_pedal - y_arm` is `±0.040`.) The non-`crank_joint` branch is unchanged — welded crankset keeps rigid pedal geoms.

`mujoco/rider.py` — `build_seated_rider(frame, pose)`: unchanged emission of pelvis/torso/arms bodies; afterwards, `if pose.leg_chains: build_articulated_legs(elements["rider_pelvis"], pose)`. New:

```python
def build_articulated_legs(pelvis_el: ET.Element, pose: SeatedPose) -> None:
    """Thigh/shank/foot hinge chains hung off the pelvis at each pedal's plane."""
    for c in pose.leg_chains:
        # The pelvis body's attach point IS the hip centre (solve_seated_pose
        # sets attach=hip), so the thigh body sits at the hip, offset laterally
        # into its pedal plane.
        thigh = ET.SubElement(pelvis_el, "body", {
            "name": f"rider_thigh_{c.side}",
            "pos": _format_vec(np.array([0.0, c.lateral_y_m, 0.0]))})
        add_joint(thigh, f"rider_hip_{c.side}", "hinge", axis="0 1 0")
        add_geom(thigh, f"geom_rider_thigh_{c.side}", "capsule",
                 fromto=_format_fromto(np.zeros(3), c.knee - c.hip),
                 size=f"{SEGMENT_RADII_M['thigh']:.3f}", mass=f"{c.thigh_mass_kg:.6f}", ...)
        shank = ET.SubElement(thigh, "body", {"name": f"rider_shank_{c.side}", "pos": _format_vec(c.knee - c.hip)})
        add_joint(shank, f"rider_knee_{c.side}", "hinge", axis="0 1 0")
        add_geom(shank, ..., fromto=_format_fromto(np.zeros(3), c.ankle - c.knee), mass=c.shank_mass_kg, ...)
        foot = ET.SubElement(shank, "body", {"name": f"rider_foot_{c.side}", "pos": _format_vec(c.ankle - c.knee)})
        add_joint(foot, f"rider_ankle_{c.side}", "hinge", axis="0 1 0")
        add_geom(foot, f"geom_rider_foot_{c.side}", "capsule",
                 fromto=_format_fromto(np.zeros(3), c.pedal - c.ankle), size=..., mass=c.foot_mass_kg, ...)
        add_site(foot, f"site_foot_{c.side}", pos=_format_vec(c.pedal - c.ankle), size="0.006")
```

Details the implementer must handle:
- Pelvis attach point is `pose.hip` (pelvis `attach=hip`), so `pos` for the thigh body is `c.hip + (0, c.lateral_y_m, 0) - c.hip` = `(0, c.lateral_y_m, 0)` — verify from `pose.body("rider_pelvis").attach` rather than assuming.
- Segment radii come from `SEGMENT_RADII_M` (import already there? it's in `physics/rider.py` — import it).
- All leg geoms `contype="0" conaffinity="0"`, `material="mat_rider"`, `rgba=RIDER_RGBA_VISIBLE`.

Welds: in `builder.py`, after `build_equality_constraints(root, mode=mode)` / `build_chain_constraint`, add when `crank_joint and pose is not None and pose.leg_chains`:

```python
for c in pose.leg_chains:
    ET.SubElement(equality, "weld", {
        "name": f"weld_foot_{c.side}",
        "body1": f"rider_foot_{c.side}",
        "body2": f"pedal_{c.side}",
    })
```
(`equality = root.find("equality")` — it exists by then.)

`frame.py::build_frame_body` — no signature change needed: `build_seated_rider` dispatches internally on `pose.leg_chains`.

`builder.py::generate_mujoco_xml` — gate documented: `crank_joint=True` with a `legs="rigid"` seated rider keeps today's exact model (rigid comparison baseline); `legs="articulated"` without `crank_joint` is legal XML (chains hang off pelvis, pedals stay welded rigid geoms → feet weld has no pedal body, so **emit no welds** when `crank_joint=False`; the articulated chains then just hang — acceptable, but the sim only builds articulated when it can use them).

- [ ] **Step 4: Run tests + compile check**

Run: `uv run pytest tests/test_ride_model.py tests/test_golden_baselines.py -x -q`
Then compile-trace sanity:
```bash
uv run python -c "
import mujoco
from bike_sim.physics.rider import RiderSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
m = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride', rider=RiderSpecs(variant='seated', legs='articulated'), crank_joint=True))
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
print('nq', m.nq, 'warnings ok')
"
```
Expected: `nq 24`, no exceptions; existing goldens byte-identical (rigid default).

- [ ] **Step 5: New golden + commit**

Add `baseline_bike_ride_pedal.xml` (ride mode, `rider=RiderSpecs('seated', legs='articulated')`, `crank_joint=True`, default `gear_ratio`) to `tests/golden/` and a matching case in `test_golden_baselines.py`. Regenerate via a short script, inspect the diff is only the new file.

```bash
git add src/bike_sim/mujoco/ tests/test_ride_model.py tests/test_golden_baselines.py tests/golden/baseline_bike_ride_pedal.xml
git commit -m "Build articulated rider legs, pedal bodies and foot-pedal welds"
```

---

### Task 3: Leg force writer (`sim/ride/leg_drive.py`) + integration

The control core: per-step IK reference + Jacobian force mapping + impedance, written to `qfrc_applied`. Replaces the spring-offset mechanism for articulated legs.

**Files:**
- Create: `src/bike_sim/sim/ride/leg_drive.py`
- Modify: `src/bike_sim/sim/ride_sim.py`, `src/bike_sim/sim/ride/drivetrain.py`
- Test: `tests/test_leg_drive.py` (new)

**Interfaces:**
- Consumes: `SeatedPose.leg_chains`, `solve_leg_qpos`, `leg_jacobian`/`fk_leg`, `crank_torque_share`, `pedal_spindle_pos` (Task 1); joints/welds (Task 2); `PedalDrivetrain.command.rider_torque_nm`.
- Produces:
  - `LegDrive(model, pose, crank_len_m, ripple_depth)` — `None`-safe like `RiderForceApplier` (construct with `pose=None` or `pose.leg_chains==()` → `active=False`, all methods no-op).
  - `LegDrive.active -> bool`
  - `LegDrive.initialize(model, data, phase_rad)` — writes leg- and pedal-hinge `qpos`/`qvel` for a crank phase; called from `RideSimulation.reset`.
  - `LegDrive.apply(model, data, rider_torque_nm)` — per-step writer.
  - `LegDrive.pedal_force_{front,rear}_n -> float` — telemetry for the recorder.

- [ ] **Step 1: Failing tests** — `tests/test_leg_drive.py`:

```python
import numpy as np
import pytest
import mujoco

from bike_sim.physics.drivetrain import DrivetrainSpecs
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


@pytest.fixture(scope="module")
def pedal_sim():
    return RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0,
        drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(), legs="articulated",
    )


def test_leg_drive_active(pedal_sim):
    assert pedal_sim.leg_drive is not None and pedal_sim.leg_drive.active


def test_weld_residual_near_zero(pedal_sim):
    for _ in range(4000):  # 2 s
        pedal_sim.step()
    m, d = pedal_sim.model, pedal_sim.data
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        sep = float(np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]))
        assert sep < 0.005, f"{side}: foot-pedal separation {sep*1000:.1f} mm"


def test_reset_at_nonzero_phase_consistent():
    sim = RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0, drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(crank_phase_deg=90.0), legs="articulated",
    )
    d = sim.data
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        assert np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]) < 0.002


def test_rigid_legs_still_pedal():
    sim = RideSimulation(
        track=get_preset("flat"), target_speed_kmh=15.0, drive_mode="pedal", assist="off",
        drivetrain=DrivetrainSpecs(), legs="rigid",
    )
    for _ in range(6000):
        sim.step()
    assert sim.speed_mps > 3.0  # today's behaviour preserved
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run pytest tests/test_leg_drive.py -x -q`
Expected: FAIL — `RideSimulation` has no `legs` param.

- [ ] **Step 3: Implement** `sim/ride/leg_drive.py`:

```python
def _hinge_addrs(model: mujoco.MjModel, name: str) -> Tuple[int, int]:
    """(qposadr, dofadr) of a hinge joint; same shape as drivetrain.py's helper."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise ValueError(f"model has no joint '{name}'; articulated legs cannot be driven")
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


class LegDrive:
    """Drives the articulated legs: pedal force + impedance toward the IK pose.

    Torques go to `qfrc_applied` (the rider_forces.py convention): the six leg
    DOFs are assigned every step, zero included.
    """

    IMPEDANCE_KP = (50.0, 50.0, 20.0)   # N.m/rad, hip/knee/ankle
    IMPEDANCE_KD = (4.0, 4.0, 1.5)      # N.m.s/rad

    def __init__(self, model, pose, crank_len_m, ripple_depth):
        self.pose = pose
        self.crank_len_m = float(crank_len_m)
        self.ripple_depth = float(ripple_depth)
        self._legs = []   # per side: (chain, (qposadr x3), (dofadr x3))
        self._prev_q_ref = {}
        self._pelvis_qposadr = 0
        self._crank_qposadr = 0
        self._pedal_adrs = {}
        self.pedal_force_front_n = 0.0
        self.pedal_force_rear_n = 0.0
        if pose is None or not pose.leg_chains:
            return
        for chain in pose.leg_chains:
            adrs = [_hinge_addrs(model, f"rider_{j}_{chain.side}") for j in ("hip", "knee", "ankle")]
            qposadrs, dofadrs = zip(*adrs)
            self._legs.append((chain, list(qposadrs), list(dofadrs)))
        self._pelvis_qposadr = _hinge_addrs(model, "rider_pelvis_z")[0]
        self._crank_qposadr = _hinge_addrs(model, "crank_spin")[0]
        self._pedal_adrs = {c.side: _hinge_addrs(model, f"pedal_spin_{c.side}") for c in pose.leg_chains}

    @property
    def active(self) -> bool:
        return bool(self._legs)

    def initialize(self, model, data, phase_rad):
        """Legs and pedals to the IK pose for the crank's starting phase."""
        for chain, qposadrs, dofadrs in self._legs:
            data.qpos[qposadrs] = solve_leg_qpos(chain, phase_rad)
            data.qvel[dofadrs] = 0.0
            pq, pd = self._pedal_adrs[chain.side]
            data.qpos[pq] = -phase_rad   # platform level in frame coords
            data.qvel[pd] = 0.0
            self._prev_q_ref[chain.side] = solve_leg_qpos(chain, phase_rad)

    def apply(self, model, data, rider_torque_nm):
        if not self._legs:
            return
        dt = float(model.opt.timestep)
        phase = float(data.qpos[self._crank_qposadr])
        pelvis_z = float(data.qpos[self._pelvis_qposadr])
        for chain, qposadrs, dofadrs in self._legs:
            side_phase = phase + (0.0 if chain.side == "front" else np.pi)
            q_ref = solve_leg_qpos(chain, phase, pelvis_z)
            qd_ref = (q_ref - self._prev_q_ref.get(chain.side, q_ref)) / dt
            self._prev_q_ref[chain.side] = q_ref
            q = data.qpos[qposadrs]
            qd = data.qvel[dofadrs]
            # Tangential pedal force: this leg's torque share over the crank arm.
            # t_hat is d(pedal)/d(phase) normalized: (-sin, -cos) in (x, z) — at
            # phase 0 (front arm at 3 o'clock) it points straight down.
            share = crank_torque_share(side_phase, self.ripple_depth)
            f_t = share * rider_torque_nm / self.crank_len_m
            F = f_t * np.array([-sin(side_phase), -cos(side_phase)])
            tau = leg_jacobian(chain, q, pelvis_z) @ F  # (3,2) @ (2,)
            tau += np.multiply(self.IMPEDANCE_KP, q_ref - q) \
                 + np.multiply(self.IMPEDANCE_KD, qd_ref - qd)
            data.qfrc_applied[dofadrs] = tau
            setattr(self, f"pedal_force_{chain.side}_n", float(np.linalg.norm(F)))
```

Notes: `leg_jacobian` uses `fk_leg` at the *current* qpos (Task 1) — never re-solve IK for J. `data.qpos[list]`/`qvel[list]` fancy-index assignments work with mujoco ndarrays; if the binding rejects a list index, fall back to `np.asarray(dofadrs)` or a three-line loop.

`sim/ride/drivetrain.py::PedalDrivetrain.__init__` — add `legs_drive: bool = False`; in `compute`'s pedalling branch, `total = self.assist_torque_nm + (0.0 if self.legs_drive else rider_nm)` (rider torque now enters via legs; `command.rider_torque_nm`/`rider_power_w` unchanged — telemetry still sees the rider's contribution). Docstring updates.

`sim/ride_sim.py`:
- `__init__`: new params `legs: Optional[str] = None`, `visual_pedalling: bool = False`. Resolve rider with legs: `self.rider = dataclasses.replace(resolve_rider(...), legs=legs or ("articulated" if (pedalled or visual_pedalling) else "rigid"))`. `crank_joint = pedalled or visual_pedalling`. Construct `self.leg_drive = LegDrive(self.model, self.pose, self.crank_length_m, ripple_depth=self.drivetrain_specs.ripple_depth)`; `PedalDrivetrain(..., legs_drive=self.leg_drive.active)` in pedalled modes.
- `reset`: after `self.drivetrain.reset(...)`, `if self.leg_drive.active: self.leg_drive.initialize(self.model, self.data, float(self.data.qpos[crank_qposadr]))` — covers `--crank-phase`.
- `step`: delete `_follow_cranks` body's leg-offset write for articulated (keep for rigid — `set_pedal_offsets` no-ops harmlessly anyway; simplest is to keep the call as-is). After `self.drivetrain.compute(...)`: `self.leg_drive.apply(self.model, self.data, command.rider_torque_nm)`. When `self.drivetrain is None` but `self.leg_drive.active` (motor + `--visual-pedalling`): `self.leg_drive.apply(self.model, self.data, 0.0)` — chain equality drags the legs.

- [ ] **Step 4: Run tests**

Run: `uv run pytest tests/test_leg_drive.py tests/test_ride_model.py -x -q`
Expected: PASS; watch for solver instability (NaN → MuJoCo error or crash detector fires). If unstable: halve `IMPEDANCE_KP`, retest; if torque saturates weld loads, check `f_t` sign convention (front leg at φ=0 must push pedal *down*, i.e. `t_hat = (−sinφ, −cosφ)` at φ=0 → `(0,−1)` — already encoded).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/leg_drive.py src/bike_sim/sim/ride_sim.py src/bike_sim/sim/ride/drivetrain.py tests/test_leg_drive.py
git commit -m "Drive the cranks from the legs: Jacobian force map + impedance"
```

---

### Task 4: CLI flags, viewer wiring, docs

**Files:**
- Modify: `src/bike_sim/cli/ride.py`, `src/bike_sim/sim/ride/viewer.py`, `docs/RIDE.md` (§6/§7/§12 stale lines)
- Test: `tests/test_ride_cli.py` (extend)

**Interfaces:**
- Consumes: `RideSimulation(legs=..., visual_pedalling=...)` (Task 3).
- Produces: `--legs {articulated,rigid}` (default `articulated` in pedal|pedelec, `rigid` otherwise); `--visual-pedalling` (motor mode only; forces `crank_joint` + articulated legs at build).

- [ ] **Step 1: Failing tests** — in `tests/test_ride_cli.py` style:

```python
def test_legs_flag_defaults():
    args = parse_args(["--drive-mode", "pedal"])
    assert resolve_leg_config(args).legs == "articulated"
    args = parse_args(["--drive-mode", "motor"])
    assert resolve_leg_config(args).legs == "rigid"
    args = parse_args(["--drive-mode", "motor", "--visual-pedalling"])
    assert resolve_leg_config(args).legs == "articulated"

def test_visual_pedalling_requires_motor():
    args = parse_args(["--drive-mode", "pedal", "--visual-pedalling"])
    with pytest.raises(ValueError):
        resolve_leg_config(args)
```

- [ ] **Step 2: Implement**

`cli/ride.py`: add `--legs` (choices `articulated`/`rigid`, default `None`) and `--visual-pedalling` flags; a small `resolve_leg_config(args)` helper returning legs/visual_pedalling; thread into `RideSimulation` (`_headless`) and `run_interactive_ride` (`viewer.py`, pass-through kwargs). `resolve_rider` builds `RiderSpecs(..., legs=...)`.

`docs/RIDE.md`: fix the stale "no drivetrain model / no cadence" lines (§6 :705-ish, §12 item 6, §1 DOF table) — describe crank joint, chain equality, articulated legs in one honest paragraph each. Keep the tone of the existing doc.

- [ ] **Step 3: Run + commit**

Run: `uv run pytest tests/test_ride_cli.py -x -q`
```bash
git add src/bike_sim/cli/ride.py src/bike_sim/sim/ride/viewer.py docs/RIDE.md tests/test_ride_cli.py
git commit -m "bike-ride: --legs and --visual-pedalling flags"
```

---

### Task 5: Acceptance — `validate_pedals.py` + recorder/telemetry

**Files:**
- Modify: `tools/validate_pedals.py`, `src/bike_sim/sim/ride/recorder.py`
- Test: none new — this *is* the acceptance harness.

**Interfaces:**
- Consumes: `RideSimulation(legs=...)`, `LegDrive.pedal_force_*_n`, site separation handles (Task 2–3).

- [ ] **Step 1: Extend `_run`/`_pedal_probe`** with a `legs` parameter (default `"articulated"` — the acceptance bar is the new mechanism; rigid remains reachable for comparison).

- [ ] **Step 2: Fix check-2 expectations.** `implied_nm / commanded_nm` bounds `0.45..1.2` may need recalibration: leg impedance consumes part of the force budget. Run the probe, print the measured ratio, then set bounds from measurement (keep the ratio-style check; do not delete it).

- [ ] **Step 3: Recalibrate check 3 (bob).** The 2×-cadence bob now comes from real pedal-force pulses + leg-mass acceleration instead of spring offsets. Run and print both amplitudes; keep the assertion `pedalled_mm > motor_mm` but record the measured ratio in the comment. If it fails after honest retuning of impedance/frequency content, fall back per spec §5 (pelvis compliance) — do not weaken the check silently.

- [ ] **Step 4: New check 5 — feet stay welded.** During `_pedal_probe` also sample `|site_foot - site_pedal|` per leg per step; assert `max < 2 mm` for the whole traverse including the freewheel section of check 4.

- [ ] **Step 5: New check 6 — sustained load ("uphill").** In a pedal-mode flat run, set `sim.resistance.crr = 0.065` (≈5% grade load equivalent) and 20 km/h target; assert the run settles within 5% of target and `wheel_drive_torque_nm` settles ≈ `rider_torque_nm/gear_ratio` (legs delivering the torque, not bookkeeping). This is the literal "pedaling force matters under load" guarantee; a real sloped track is out of scope (terrain has no base grade).

- [ ] **Step 6: Recorder channels.** `pedal_load_front_n`/`pedal_load_rear_n` currently read `rider_forces` (zero under articulated legs). Point them at `sim.leg_drive.pedal_force_*_n` when `leg_drive.active`, else the old path — same columns, honest values.

- [ ] **Step 7: Run the harness**

```bash
uv run python tools/validate_pedals.py --quick
uv run pytest tests/ -x -q   # full suite
```
Expected: all checks pass; commit.

```bash
git add tools/validate_pedals.py src/bike_sim/sim/ride/recorder.py
git commit -m "Acceptance: weld-integrity, sustained-load and bob checks for leg drive"
```

---

### Task 6: Visual verification — render a pedal cycle

**Files:**
- Create (throwaway or `tools/`): `tools/render_pedals.py`
- Uses the repo's own conventions (`uv run`, offscreen `mujoco.Renderer`).

- [ ] **Step 1:** Write a short script that builds a pedal-mode `RideSimulation` on `flat`, steps ~1.5 crank revolutions, and renders a side-view strip (camera near y≈−1.5 m looking at the BB/rider) at ~8 phases to `output/pedal_check/pedal_*.png`. Small zoom on the legs.

- [ ] **Step 2:** Run it, **look at the images** (read them as image files), and iterate on what looks wrong (knee direction, foot pitch, pedal levelness). This task exists because the original complaint was visual — numbers passing is not the same as looking right.

- [ ] **Step 3:** Commit the script (keep or delete at user's preference; default keep — it's the eyeball harness).

```bash
git add tools/render_pedals.py
git commit -m "Add side-view pedal-cycle render tool"
```

---

## Execution notes

- Order matters: Task 1 → 2 → 3 are strictly sequential (each consumes the previous); Task 4 parallel-safe vs 5 only if different files are touched — they're not (both touch ride plumbing), so sequential too. Task 6 last.
- If `validate_pedals.py` check 1/2 bounds need recalibration under real leg dynamics, print measured values and set bounds from them — never widen bounds silently.
- Known hard spots flagged by the spec: closed-chain weld + torque control stability (mitigate: impedance floor, `solref` on welds if the solver complains, rigid fallback via `--legs rigid`), bob recalibration (fallback: pelvis compliance).
