"""
Rider Specification, Anthropometry and Seated Pose.

One source of truth for everything the simulation knows about the rider: how heavy and tall
they are, which model represents them, how their mass is split between the three interfaces
with the bike (saddle, pedals, handlebar), how stiff those interfaces are, and where every
body segment sits once the saddle has been set for their inseam.

Three rider variants exist:

- ``none``   -- no rider; the bike alone.
- ``lumped`` -- the original 80 kg rigid rider in a standing attack pose, three capsules lumped
  into the ``frame`` body (docs/RIDE.md section 7). Kept as the regression reference.
- ``seated`` -- a seated biodynamic rider: four lumped masses on vertical slide joints with
  spring-dampers to the saddle, the two pedals and the handlebar, plus a torso mass on the
  pelvis (docs/RIDE.md section 7). Ride mode only.

**Provenance.** Every number below is tagged. *Literature* values carry their source;
*derived* values are computed from a literature quantity and a model mass; *authored* values
are this repository's own and are the first thing to replace when a measurement turns up.
"""

from dataclasses import dataclass, field
from math import acos, cos, degrees, pi, radians, sin, sqrt
from typing import Dict, List, Optional, Tuple

import numpy as np

from bike_sim.geometry.cockpit import (
    MAX_EXPOSED_SEATPOST_M,
    MIN_EXPOSED_SEATPOST_M,
    PEDAL_LATERAL_OFFSET_M,
    SaddleGeometry,
    handlebar_grip_point,
    pedal_points,
    saddle_geometry,
    saddle_top_z_for_height,
)
from bike_sim.geometry.hardpoints import get_fixed_frame_points
from bike_sim.geometry.specs import BikeSpecs

GRAVITY_MPS2 = 9.81

RIDER_VARIANTS = ("none", "lumped", "seated", "articulated_planar")
DEFAULT_RIDER_VARIANT = "seated"

# --------------------------------------------------------------------------------------
# Anthropometry -- literature
# --------------------------------------------------------------------------------------

# de Leva, P. (1996). Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters.
# J. Biomechanics 29(9), 1223-1230, Table 4, males (73.0 kg, 1.741 m). Segment masses as
# fractions of body mass; they sum to 1.0000 with both limbs counted.
DE_LEVA_MASS_FRACTIONS: Dict[str, float] = {
    "head": 0.0694,
    "trunk_upper": 0.1596,   # suprasternale -> xyphion
    "trunk_middle": 0.1633,  # xyphion -> omphalion
    "trunk_lower": 0.1117,   # omphalion -> mid-hip: the pelvis
    "upper_arm": 0.0271,     # per limb
    "forearm": 0.0162,
    "hand": 0.0061,
    "thigh": 0.1416,
    "shank": 0.0433,
    "foot": 0.0137,
}

# Same table: segment lengths of the reference male, as fractions of stature, between the
# joint centres named in the comments.
DE_LEVA_REFERENCE_STATURE_M = 1.741
DE_LEVA_LENGTH_FRACTIONS: Dict[str, float] = {
    "trunk": 0.5155 / DE_LEVA_REFERENCE_STATURE_M,      # mid-shoulder -> mid-hip
    "upper_arm": 0.2817 / DE_LEVA_REFERENCE_STATURE_M,  # shoulder -> elbow joint centre
    "forearm": 0.2689 / DE_LEVA_REFERENCE_STATURE_M,    # elbow -> wrist joint centre
    "hand": 0.0862 / DE_LEVA_REFERENCE_STATURE_M,       # wrist -> 3rd metacarpale
    "thigh": 0.4222 / DE_LEVA_REFERENCE_STATURE_M,      # hip -> knee joint centre
    "shank": 0.4340 / DE_LEVA_REFERENCE_STATURE_M,      # knee -> lateral malleolus
    "head": 0.2033 / DE_LEVA_REFERENCE_STATURE_M,       # vertex -> mid-gonion
}

# Crotch height over stature, US Army ANSUR II male means (Gordon et al. 2014), rounded.
# Approximate; `RiderSpecs.inseam_m` overrides it.
INSEAM_STATURE_RATIO = 0.47

# LeMond: saddle height (BB centre to saddle top) = 0.883 x inseam. The standard road-fit rule.
LEMOND_SADDLE_HEIGHT_RATIO = 0.883

# --------------------------------------------------------------------------------------
# Seated pose -- authored
# --------------------------------------------------------------------------------------

# Hip joint centre above the saddle's top surface. The pelvis rests on the ischial
# tuberosities; the joint centre is a few centimetres above and in front of them.
HIP_ABOVE_SADDLE_M = 0.060

# Ankle joint above the pedal spindle: shoe sole, pedal body and the plantar-flexed foot of a
# rider pedalling toe-down. Calibrated so LeMond's saddle height gives ~30 degrees of knee
# flexion at bottom dead centre for the default rider, which is where fitters put it.
ANKLE_ABOVE_PEDAL_M = 0.115

# Elbow flexion the arms hold at the bar, and where along the hand the bar sits.
ELBOW_FLEXION_DEG = 15.0
GRIP_FROM_WRIST_FRACTION = 0.60

# Head sphere: radius, and neck length from the shoulder line to the sphere's underside.
HEAD_RADIUS_M = 0.110
NECK_LENGTH_M = 0.050

# Knee flexion at bottom dead centre the fit is accepted at. Outside it the saddle derived
# from the rider's inseam does not match the crank and the model refuses to build.
KNEE_FLEXION_BDC_MIN_DEG = 20.0
KNEE_FLEXION_BDC_MAX_DEG = 60.0

# Visual capsule radii, in metres.
SEGMENT_RADII_M: Dict[str, float] = {
    "torso": 0.110,
    "pelvis": 0.085,
    "thigh": 0.070,
    "shank": 0.055,
    "foot": 0.035,
    "upper_arm": 0.045,
    "forearm": 0.040,
}

# --------------------------------------------------------------------------------------
# Lumped (legacy) rider -- authored
# --------------------------------------------------------------------------------------

LUMPED_TORSO_HELMET_FRACTION = 55.0 / 80.0
LUMPED_LEGS_FRACTION = 18.0 / 80.0
LUMPED_ARMS_FRACTION = 7.0 / 80.0
LUMPED_COM_M: Dict[str, np.ndarray] = {
    "rider_torso": np.array([0.160, 0.0, 0.650]),
    "rider_legs": np.array([0.050, 0.0, 0.240]),
    "rider_arms": np.array([0.335, 0.0, 0.720]),
}


@dataclass
class RiderSpecs:
    """
    The rider: variant, size, mass split and interface dynamics.

    Attributes:
        variant: ``"none"``, ``"lumped"`` or ``"seated"``.
        legs: ``"rigid"`` (default) keeps the seated rider's two rigid capsule clusters on
            vertical slide joints; ``"articulated"`` replaces them with hip/knee/ankle
            chains whose feet weld to the pedal bodies. Only read by the seated variant.
        mass_kg: Total rider mass including helmet and kit.
        height_m: Stature; scales every de Leva segment length.
        inseam_m: Crotch height. ``None`` derives it from stature.
        helmet_mass_kg: Counted inside ``mass_kg``, placed on the head.
        saddle_share / pedal_share / bar_share: Static load split between the three
            interfaces, seated coasting. Literature: Carahalios (2015) measured 44 / 41 /
            15 % (saddle / bottom bracket / stem) at 2 W/kg on the hoods, shifting 5.2 pp
            from saddle and 3.3 pp from bars to pedals per W/kg; extrapolated to 0 W/kg that
            is ~54 / 34 / 12 %. Wilson et al. (2007): 49-52 % on the saddle at 125 W. Road
            posture; an upright MTB posture moves bar load to the saddle.
        torso_resonance_hz: Uncoupled resonance of the torso mass on its spine spring, the
            pelvis held. *Derived*: with the pelvis-to-saddle contact below (K8 / C8) the
            two-mass saddle path's apparent mass then peaks at 4.8 Hz with 1.6 x the static
            mass, which is where the seated human body's peak is measured -- 4-6 Hz, ~1.5 x
            (Fairley & Griffin 1989; Kumar & Saran 2019 measure 5 Hz; Stanczyk & Zuska 2015
            fit 4.5-5.5 Hz). `tests/test_ride_equilibrium.py` checks that coupled peak.
        leg_resonance_hz: Uncoupled resonance of each leg mass on its pedal. Authored; the
            measured values are Wang & Hull (1997) Table 2, which this repository could not
            read. Replace when available.
        arm_resonance_hz: Uncoupled resonance of the arm mass on the bar. Authored, as above.
        damping_ratio: Of every derived spring. Fitted seated-body models give 0.3-0.45
            (Wei & Griffin 1998 two-DOF: k1 = 42.9 kN/m, c1 = 721 N.s/m on 31.1 kg -> 0.31;
            Muksian & Nash via Turner 2024: 50 kN/m, 1 kN.s/m on 66 kg -> 0.28; Kumar &
            Saran 2019 fit 1.5-3.0 kN.s/m on 100-150 kN/m springs). 0.40 puts the torso's
            transmissibility peak at 1.8, against the 1.7 Kumar & Saran measured seat-to-head.
        saddle_interface_k_n_m / saddle_interface_c_ns_m: Stiffness and damping of the
            pelvis-to-saddle contact. Literature: Kumar & Saran (2019) Table 2, K8 / C8, the
            path beneath the pelvis of a subject seated upright on a hard seat.
    """

    variant: str = DEFAULT_RIDER_VARIANT
    legs: str = "rigid"
    mass_kg: float = 80.0
    height_m: float = 1.80
    inseam_m: Optional[float] = None
    helmet_mass_kg: float = 0.40

    saddle_share: float = 0.55
    pedal_share: float = 0.33
    bar_share: float = 0.12

    torso_resonance_hz: float = 7.0
    leg_resonance_hz: float = 5.0
    arm_resonance_hz: float = 4.0
    damping_ratio: float = 0.40
    saddle_interface_k_n_m: float = 101_000.0
    saddle_interface_c_ns_m: float = 2_762.0

    def __post_init__(self) -> None:
        if self.variant not in RIDER_VARIANTS:
            raise ValueError(f"rider variant must be one of {RIDER_VARIANTS}, got {self.variant!r}")
        if self.legs not in ("rigid", "articulated"):
            raise ValueError(f"legs must be 'rigid' or 'articulated', got {self.legs!r}")
        if self.mass_kg <= 0.0 and self.variant != "none":
            raise ValueError(f"rider mass must be positive, got {self.mass_kg}")
        if self.height_m <= 0.0:
            raise ValueError(f"rider height must be positive, got {self.height_m}")
        if self.inseam_m is not None and not 0.0 < self.inseam_m < self.height_m:
            raise ValueError(f"inseam {self.inseam_m} m must lie between 0 and the height {self.height_m} m")
        shares = self.saddle_share + self.pedal_share + self.bar_share
        if abs(shares - 1.0) > 1e-9:
            raise ValueError(f"saddle, pedal and bar shares must sum to 1, got {shares:.6f}")
        if self.helmet_mass_kg < 0.0 or self.helmet_mass_kg >= self.mass_kg:
            raise ValueError(f"helmet mass {self.helmet_mass_kg} kg must be non-negative and below the rider mass")

    # --- presence -------------------------------------------------------------------

    @property
    def present(self) -> bool:
        """Whether the rider adds mass to the system."""
        return self.variant != "none"

    @property
    def total_rider_mass(self) -> float:
        """Rider mass the system carries, in kg; zero for ``none``."""
        return float(self.mass_kg) if self.present else 0.0

    # --- size -------------------------------------------------------------------------

    @property
    def inseam(self) -> float:
        """Crotch height in metres, given or derived from stature."""
        return float(self.inseam_m) if self.inseam_m is not None else INSEAM_STATURE_RATIO * self.height_m

    @property
    def saddle_height_m(self) -> float:
        """LeMond saddle height for this rider: BB centre to saddle top, in metres."""
        return LEMOND_SADDLE_HEIGHT_RATIO * self.inseam

    def segment_length(self, name: str) -> float:
        """de Leva segment length for this stature, in metres."""
        return DE_LEVA_LENGTH_FRACTIONS[name] * self.height_m

    # --- lumped variant -----------------------------------------------------------------

    @property
    def torso_helmet_mass(self) -> float:
        """Lumped variant: torso and helmet capsule mass, 55/80 of the rider."""
        return LUMPED_TORSO_HELMET_FRACTION * self.mass_kg

    @property
    def legs_mass(self) -> float:
        """Lumped variant: legs capsule mass, 18/80 of the rider."""
        return LUMPED_LEGS_FRACTION * self.mass_kg

    @property
    def arms_mass(self) -> float:
        """Lumped variant: arms capsule mass, 7/80 of the rider."""
        return LUMPED_ARMS_FRACTION * self.mass_kg

    # --- seated variant -----------------------------------------------------------------

    def seated_pose(self, specs: Optional[BikeSpecs] = None) -> "SeatedPose":
        """
        Solves the seated pose on a bike.

        Args:
            specs: Bicycle geometry. Defaults to the shipped `BikeSpecs`.

        Raises:
            ValueError: If the variant is not ``seated``, or the rider does not fit the bike
                (see `solve_seated_pose`).
        """
        if self.variant != "seated":
            raise ValueError(f"seated_pose is only defined for the seated variant, not {self.variant!r}")
        return solve_seated_pose(specs if specs is not None else BikeSpecs(), self)

    # --- shared ---------------------------------------------------------------------------

    def compute_rider_centers_of_mass(
        self, specs: Optional[BikeSpecs] = None
    ) -> Dict[str, Tuple[np.ndarray, float]]:
        """
        Returns CoM position (m, BB frame) and mass (kg) per rider body.

        Lumped: the three capsule midpoints of the standing attack pose. Seated: one entry per
        MJCF rider body, at the mass-weighted centre of the capsules that body is built from --
        the same segment table the builder emits, so the analytic and compiled models cannot
        drift apart. Empty for ``none``.
        """
        if self.variant == "none":
            return {}
        if self.variant == "lumped":
            return {
                "rider_torso": (LUMPED_COM_M["rider_torso"].copy(), self.torso_helmet_mass),
                "rider_legs": (LUMPED_COM_M["rider_legs"].copy(), self.legs_mass),
                "rider_arms": (LUMPED_COM_M["rider_arms"].copy(), self.arms_mass),
            }
        if self.variant == "articulated_planar":
            from bike_sim.physics.rider_segments import geometry_pose, segment_masses
            pose = geometry_pose(self, specs if specs is not None else BikeSpecs())
            masses = segment_masses(self.mass_kg, self.helmet_mass_kg)
            centers = {
                "pelvis": pose.hip + np.array([0., 0., .06]),
                "torso": (pose.hip + pose.shoulder) / 2,
                "head": pose.head_center,
                "upper_arm_pair": (pose.shoulder + pose.elbow) / 2,
                "forearm_pair": (pose.elbow + pose.grip) / 2,
            }
            for side, sign in (("front", -1.), ("rear", 1.)):
                knee = getattr(pose, "knee_" + side)
                ankle = getattr(pose, "ankle_" + side)
                pedal = getattr(pose, "pedal_" + side)
                for part, point in (("thigh", (pose.hip + knee) / 2),
                                    ("shank", (knee + ankle) / 2),
                                    ("foot", (ankle + pedal) / 2)):
                    center = np.array(point, copy=True)
                    center[1] = sign * PEDAL_LATERAL_OFFSET_M
                    centers[part + "_" + side] = center
            return {"rider_" + key: (np.array(center, copy=True), masses[key])
                    for key, center in centers.items()}
        pose = self.seated_pose(specs)
        components = {body.name: (body.center_of_mass, body.mass) for body in pose.bodies}
        for chain in pose.leg_chains:
            # Articulated legs are MJCF segment bodies, not RiderBodys; add one table entry
            # per segment at its midpoint so the CG table still matches the compiled model.
            offset = np.array([0.0, chain.lateral_y_m, 0.0])
            components[f"rider_thigh_{chain.side}"] = (
                0.5 * (chain.hip + chain.knee) + offset, chain.thigh_mass_kg)
            components[f"rider_shank_{chain.side}"] = (
                0.5 * (chain.knee + chain.ankle) + offset, chain.shank_mass_kg)
            components[f"rider_foot_{chain.side}"] = (
                0.5 * (chain.ankle + chain.pedal) + offset, chain.foot_mass_kg)
        return components


# --------------------------------------------------------------------------------------
# Seated pose
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class RiderGeom:
    """One visual/mass capsule of a rider body, endpoints in the BB frame (metres)."""

    name: str
    kind: str  # "capsule" | "sphere"
    p1: np.ndarray
    p2: np.ndarray
    radius: float
    mass: float

    @property
    def center(self) -> np.ndarray:
        """Geometric centre, which for a uniform capsule or sphere is its centre of mass."""
        return 0.5 * (self.p1 + self.p2)


@dataclass(frozen=True)
class RiderBody:
    """
    One lumped mass of the seated rider and the spring-damper that carries it.

    Attributes:
        name: MJCF body name.
        parent: MJCF parent body name (``frame`` or another rider body).
        attach: Body origin in the BB frame (m). The slide joint acts along the frame's z here.
        joint: MJCF slide joint name.
        geoms: Capsules the body is built from; their masses sum to ``mass``.
        stiffness_n_m / damping_ns_m: Spring-damper between the body and its parent.
        supported_mass_kg: Mass the spring carries at rest -- this body plus any rider body
            stacked on it -- so the preload is ``supported_mass_kg * g``.
        unilateral: Whether the spring can only push (a saddle or a flat pedal) or also pull
            (a gripped bar, the spine).
        interface: Which bike interface the spring reacts against, for telemetry.
    """

    name: str
    parent: str
    attach: np.ndarray
    joint: str
    geoms: Tuple[RiderGeom, ...]
    stiffness_n_m: float
    damping_ns_m: float
    supported_mass_kg: float
    unilateral: bool
    interface: str

    @property
    def mass(self) -> float:
        """Body mass in kg: the sum of its geoms."""
        return float(sum(g.mass for g in self.geoms))

    @property
    def center_of_mass(self) -> np.ndarray:
        """Mass-weighted centre of the body's geoms, in the BB frame (m)."""
        total = self.mass
        return sum(g.mass * g.center for g in self.geoms) / total

    @property
    def preload_n(self) -> float:
        """Spring force at the design pose, in N."""
        return self.supported_mass_kg * GRAVITY_MPS2

    @property
    def preload_deflection_m(self) -> float:
        """Spring compression at the design pose, in m: the joint travel that unloads it."""
        return self.preload_n / self.stiffness_n_m


@dataclass(frozen=True)
class SeatedPose:
    """
    The solved seated pose: joint centres, saddle placement, masses and springs.

    All points are in the BB frame, metres, sagittal plane (y = 0).
    """

    saddle: SaddleGeometry
    hip: np.ndarray
    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    grip: np.ndarray
    head_center: np.ndarray
    knee_front: np.ndarray
    knee_rear: np.ndarray
    ankle_front: np.ndarray
    ankle_rear: np.ndarray
    pedal_front: np.ndarray
    pedal_rear: np.ndarray
    torso_lean_deg: float
    knee_flexion_front_deg: float
    knee_flexion_rear_deg: float
    knee_flexion_bdc_deg: float
    bodies: Tuple[RiderBody, ...]
    path_masses_kg: Dict[str, float] = field(default_factory=dict)
    leg_chains: Tuple["LegChain", ...] = ()

    def body(self, name: str) -> RiderBody:
        """Returns the rider body of that name."""
        for body in self.bodies:
            if body.name == name:
                return body
        raise KeyError(f"no rider body named {name!r}")

    @property
    def total_mass_kg(self) -> float:
        """Rider mass across all bodies, in kg -- plus the articulated leg chains,
        whose mass lives in ``leg_chains`` rather than in ``bodies``."""
        return float(
            sum(b.mass for b in self.bodies)
            + sum(c.thigh_mass_kg + c.shank_mass_kg + c.foot_mass_kg for c in self.leg_chains)
        )

    def interface_loads_n(self) -> Dict[str, float]:
        """Static load each interface carries at the design pose, in N.

        With articulated legs the ``pedals`` entry reads 0: leg weight reaches the
        pedals through the hip/knee/ankle joints and the foot-pedal weld, not a
        slide-spring preload (the LegDrive path owns pedal loading at runtime).
        """
        loads = {"saddle": 0.0, "pedals": 0.0, "bar": 0.0}
        for body in self.bodies:
            if body.parent == "frame":
                loads[body.interface] += body.preload_n
        return loads

    def interface_shares(self) -> Dict[str, float]:
        """Static load split between saddle, pedals and bar, as fractions of rider weight."""
        loads = self.interface_loads_n()
        weight = self.total_mass_kg * GRAVITY_MPS2
        return {k: v / weight for k, v in loads.items()}


def _two_link_ik(a: np.ndarray, b: np.ndarray, len_a: float, len_b: float, prefer: str) -> np.ndarray:
    """
    Middle joint of a two-link chain from ``a`` to ``b``, in the sagittal plane.

    Args:
        a: Proximal endpoint (m).
        b: Distal endpoint (m).
        len_a: Proximal link length (m).
        len_b: Distal link length (m).
        prefer: ``"+x"``, ``"-x"``, ``"+z"`` or ``"-z"``: which of the two solutions to take.

    Raises:
        ValueError: If the endpoints are out of reach or closer than the links allow.
    """
    d_vec = b - a
    d = float(np.hypot(d_vec[0], d_vec[2]))
    if d > len_a + len_b + 1e-12:
        raise ValueError(f"endpoints {d:.3f} m apart exceed the chain's {len_a + len_b:.3f} m reach")
    if d < abs(len_a - len_b) - 1e-12:
        raise ValueError(f"endpoints {d:.3f} m apart are closer than the chain's {abs(len_a - len_b):.3f} m minimum")
    # Distance along a->b of the joint's foot, and its height off the line.
    along = (len_a * len_a - len_b * len_b + d * d) / (2.0 * d)
    off = sqrt(max(0.0, len_a * len_a - along * along))
    u = np.array([d_vec[0], 0.0, d_vec[2]]) / d
    n = np.array([-u[2], 0.0, u[0]])  # +90 degrees in the x-z plane
    foot = a + along * u
    c1, c2 = foot + off * n, foot - off * n
    axis = {"+x": (0, 1.0), "-x": (0, -1.0), "+z": (2, 1.0), "-z": (2, -1.0)}[prefer]
    return c1 if axis[1] * c1[axis[0]] >= axis[1] * c2[axis[0]] else c2


def _flexion_deg(proximal: np.ndarray, joint: np.ndarray, distal: np.ndarray) -> float:
    """Flexion angle at ``joint``: 0 when the two links are collinear (straight)."""
    v1 = proximal - joint
    v2 = distal - joint
    cosang = float(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return 180.0 - degrees(acos(max(-1.0, min(1.0, cosang))))


def _spring(mass_kg: float, resonance_hz: float, damping_ratio: float) -> Tuple[float, float]:
    """Stiffness and damping that put ``mass_kg`` at ``resonance_hz`` with ``damping_ratio``."""
    k = mass_kg * (2.0 * pi * resonance_hz) ** 2
    c = 2.0 * damping_ratio * sqrt(k * mass_kg)
    return float(k), float(c)


def seated_path_masses(rider: RiderSpecs) -> Dict[str, float]:
    """
    Allocates the rider's anatomical segments to the saddle, pedal and bar load paths.

    The de Leva segments are assigned so the static split hits the rider's target shares: the
    arms path is the arms plus enough shoulder girdle to reach ``bar_share``; the leg paths are
    the shanks and feet plus enough thigh to reach ``pedal_share``; the saddle path is the rest,
    stacked as torso (upper and middle trunk, head, helmet) on pelvis (lower trunk, remaining
    thigh).

    Returns:
        Masses in kg: ``torso``, ``pelvis``, ``arms``, ``leg`` (each of two), plus the
        anatomical ``head`` (with helmet) and ``trunk_to_arms`` transfers for the builder.

    Raises:
        ValueError: If the shares cannot be met with non-negative segment masses.
    """
    f = DE_LEVA_MASS_FRACTIONS
    body = rider.mass_kg - rider.helmet_mass_kg
    head = f["head"] * body + rider.helmet_mass_kg
    trunk_upper_mid = (f["trunk_upper"] + f["trunk_middle"]) * body
    trunk_lower = f["trunk_lower"] * body
    arms = 2.0 * (f["upper_arm"] + f["forearm"] + f["hand"]) * body
    thighs = 2.0 * f["thigh"] * body
    shanks_feet = 2.0 * (f["shank"] + f["foot"]) * body

    bar_path = rider.bar_share * rider.mass_kg
    pedal_path = rider.pedal_share * rider.mass_kg
    saddle_path = rider.saddle_share * rider.mass_kg

    trunk_to_arms = bar_path - arms
    thigh_to_legs = pedal_path - shanks_feet
    if trunk_to_arms < 0.0:
        raise ValueError(
            f"bar share {rider.bar_share:.2f} puts {bar_path:.1f} kg on the bar, less than the arms' {arms:.1f} kg"
        )
    if thigh_to_legs < 0.0 or thigh_to_legs > thighs:
        raise ValueError(
            f"pedal share {rider.pedal_share:.2f} needs {thigh_to_legs:.1f} kg of thigh on the pedals; "
            f"the shanks and feet weigh {shanks_feet:.1f} kg and the thighs {thighs:.1f} kg"
        )
    torso = trunk_upper_mid + head - trunk_to_arms
    pelvis = saddle_path - torso
    if pelvis <= 0.0:
        raise ValueError(f"saddle share {rider.saddle_share:.2f} leaves {pelvis:.1f} kg for the pelvis")
    return {
        "torso": float(torso),
        "pelvis": float(pelvis),
        "arms": float(bar_path),
        "leg": float(pedal_path / 2.0),
        "head": float(head),
        "trunk_to_arms": float(trunk_to_arms),
        "thigh_to_legs": float(thigh_to_legs / 2.0),
        "shank_foot": float(shanks_feet / 2.0),
        "upper_arm_fraction": float(f["upper_arm"] / (f["upper_arm"] + f["forearm"] + f["hand"])),
    }


@dataclass(frozen=True)
class LegChain:
    """One articulated leg: design-pose geometry, segment lengths and masses.

    All points are in the BB frame at the design pose (cranks horizontal) on the
    sagittal plane; the leg itself solves in a plane offset by ``lateral_y_m``.
    ``crank_len_m`` is signed: ``+crank`` for the front leg (its crank arm points
    forward at phase 0), ``-crank`` for the rear leg (arm back), so that
    ``pedal_spindle_pos(phase, lateral_y_m, crank_len_m)`` tracks this leg's own
    pedal through a crank revolution.
    """

    side: str
    lateral_y_m: float
    crank_len_m: float
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


def solve_seated_pose(specs: BikeSpecs, rider: RiderSpecs) -> SeatedPose:
    """
    Places a seated rider on the bike.

    The saddle is set to LeMond's height for the rider's inseam; the hips sit above it; the
    feet stand on horizontal cranks; the torso leans until the arms, with a slightly bent
    elbow, reach the handlebar. Knees and elbows follow by two-link inverse kinematics.

    Args:
        specs: Bicycle geometry, including the crank length.
        rider: Rider size and mass split.

    Returns:
        The solved pose with its bodies and springs.

    Raises:
        ValueError: If the saddle leaves the post's usable range, the bar is out of the
            rider's reach, a pedal is out of the leg's reach, or the knee at bottom dead
            centre falls outside the accepted flexion window.
    """
    fixed = get_fixed_frame_points(specs)
    p9 = np.array(fixed["P9"], dtype=float) / 1000.0

    saddle = saddle_geometry(p9, saddle_top_z_for_height(p9, rider.saddle_height_m))
    if not MIN_EXPOSED_SEATPOST_M <= saddle.exposed_post_m <= MAX_EXPOSED_SEATPOST_M:
        raise ValueError(
            f"a {rider.height_m:.2f} m rider with a {rider.inseam:.3f} m inseam needs a "
            f"{rider.saddle_height_m:.3f} m saddle height, which exposes {saddle.exposed_post_m * 1000:.0f} mm of "
            f"seatpost; the frame accepts {MIN_EXPOSED_SEATPOST_M * 1000:.0f}-{MAX_EXPOSED_SEATPOST_M * 1000:.0f} mm. "
            f"Adjust --rider-height or --rider-inseam."
        )

    hip = saddle.top_center + np.array([0.0, 0.0, HIP_ABOVE_SADDLE_M])
    grip = handlebar_grip_point(specs)
    crank_m = specs.crank_length / 1000.0
    pedal_front, pedal_rear = pedal_points(crank_m)
    ankle_offset = np.array([0.0, 0.0, ANKLE_ABOVE_PEDAL_M])
    ankle_front, ankle_rear = pedal_front + ankle_offset, pedal_rear + ankle_offset

    l_thigh, l_shank = rider.segment_length("thigh"), rider.segment_length("shank")
    l_trunk = rider.segment_length("trunk")
    l_ua, l_fa, l_hand = (rider.segment_length(n) for n in ("upper_arm", "forearm", "hand"))

    # Legs: knee forward of the hip-ankle line.
    try:
        knee_front = _two_link_ik(hip, ankle_front, l_thigh, l_shank, prefer="+x")
        knee_rear = _two_link_ik(hip, ankle_rear, l_thigh, l_shank, prefer="+x")
    except ValueError as exc:
        raise ValueError(f"the rider's legs do not reach the pedals from the saddle: {exc}") from exc

    # Bottom dead centre check: crank straight down.
    ankle_bdc = np.array([0.0, 0.0, -crank_m]) + ankle_offset
    try:
        knee_bdc = _two_link_ik(hip, ankle_bdc, l_thigh, l_shank, prefer="+x")
    except ValueError as exc:
        raise ValueError(
            f"the leg does not reach the pedal at bottom dead centre ({exc}); the saddle at "
            f"{rider.saddle_height_m:.3f} m is too high for a {rider.height_m:.2f} m rider"
        ) from exc
    knee_flexion_bdc = _flexion_deg(hip, knee_bdc, ankle_bdc)
    if not KNEE_FLEXION_BDC_MIN_DEG <= knee_flexion_bdc <= KNEE_FLEXION_BDC_MAX_DEG:
        raise ValueError(
            f"knee flexion at bottom dead centre is {knee_flexion_bdc:.1f} degrees; the fit accepts "
            f"{KNEE_FLEXION_BDC_MIN_DEG:.0f}-{KNEE_FLEXION_BDC_MAX_DEG:.0f}. Saddle {rider.saddle_height_m:.3f} m "
            f"from a {rider.inseam:.3f} m inseam against {specs.crank_length:.0f} mm cranks; adjust --rider-height, "
            f"--rider-inseam or the crank length."
        )

    # Arms: the shoulder is where a trunk from the hip meets an arm from the bar.
    elbow_rad = radians(ELBOW_FLEXION_DEG)
    arm_chord = sqrt(l_ua * l_ua + l_fa * l_fa + 2.0 * l_ua * l_fa * cos(elbow_rad))
    arm_reach = arm_chord + GRIP_FROM_WRIST_FRACTION * l_hand
    try:
        shoulder = _two_link_ik(hip, grip, l_trunk, arm_reach, prefer="+z")
    except ValueError as exc:
        raise ValueError(f"the rider cannot reach the handlebar from the saddle: {exc}") from exc
    to_grip = grip - shoulder
    to_grip /= float(np.linalg.norm(to_grip))
    wrist = grip - GRIP_FROM_WRIST_FRACTION * l_hand * to_grip
    elbow = _two_link_ik(shoulder, wrist, l_ua, l_fa, prefer="-z")

    torso_axis = shoulder - hip
    torso_axis /= float(np.linalg.norm(torso_axis))
    torso_lean = degrees(acos(max(-1.0, min(1.0, float(torso_axis[2])))))
    head_center = shoulder + (NECK_LENGTH_M + HEAD_RADIUS_M) * torso_axis

    masses = seated_path_masses(rider)
    # Shank/foot split of each leg's shank_foot path mass, by de Leva fractions; only the
    # articulated legs consume it.
    shank_foot_fraction = DE_LEVA_MASS_FRACTIONS["shank"] + DE_LEVA_MASS_FRACTIONS["foot"]
    shank_mass = masses["shank_foot"] * DE_LEVA_MASS_FRACTIONS["shank"] / shank_foot_fraction
    foot_mass = masses["shank_foot"] * DE_LEVA_MASS_FRACTIONS["foot"] / shank_foot_fraction
    k_torso, c_torso = _spring(masses["torso"], rider.torso_resonance_hz, rider.damping_ratio)
    k_arms, c_arms = _spring(masses["arms"], rider.arm_resonance_hz, rider.damping_ratio)
    k_leg, c_leg = _spring(masses["leg"], rider.leg_resonance_hz, rider.damping_ratio)
    r = SEGMENT_RADII_M
    upper_arm_mass = masses["upper_arm_fraction"] * masses["arms"]

    pelvis_center = hip + np.array([-0.010, 0.0, 0.040])
    pelvis = RiderBody(
        name="rider_pelvis",
        parent="frame",
        attach=hip,
        joint="rider_pelvis_z",
        geoms=(
            RiderGeom(
                "geom_rider_pelvis", "capsule",
                pelvis_center + np.array([-0.060, 0.0, 0.0]), pelvis_center + np.array([0.060, 0.0, 0.0]),
                r["pelvis"], masses["pelvis"],
            ),
        ),
        stiffness_n_m=rider.saddle_interface_k_n_m,
        damping_ns_m=rider.saddle_interface_c_ns_m,
        supported_mass_kg=masses["pelvis"] + masses["torso"],
        unilateral=True,
        interface="saddle",
    )
    torso = RiderBody(
        name="rider_torso",
        parent="rider_pelvis",
        attach=hip,
        joint="rider_torso_z",
        geoms=(
            RiderGeom(
                "geom_rider_torso", "capsule",
                hip + 0.10 * torso_axis, shoulder, r["torso"], masses["torso"] - masses["head"],
            ),
            RiderGeom("geom_rider_head", "sphere", head_center, head_center, HEAD_RADIUS_M, masses["head"]),
        ),
        stiffness_n_m=k_torso,
        damping_ns_m=c_torso,
        supported_mass_kg=masses["torso"],
        unilateral=False,
        interface="saddle",
    )
    arms = RiderBody(
        name="rider_arms",
        parent="frame",
        attach=grip,
        joint="rider_arms_z",
        geoms=(
            RiderGeom("geom_rider_upper_arm", "capsule", shoulder, elbow, r["upper_arm"], upper_arm_mass),
            RiderGeom("geom_rider_forearm", "capsule", elbow, grip, r["forearm"], masses["arms"] - upper_arm_mass),
        ),
        stiffness_n_m=k_arms,
        damping_ns_m=c_arms,
        supported_mass_kg=masses["arms"],
        unilateral=False,
        interface="bar",
    )

    def leg(side: str, knee: np.ndarray, ankle: np.ndarray, pedal: np.ndarray) -> RiderBody:
        return RiderBody(
            name=f"rider_leg_{side}",
            parent="frame",
            attach=pedal,
            joint=f"rider_leg_{side}_z",
            geoms=(
                RiderGeom(f"geom_rider_thigh_{side}", "capsule", hip, knee, r["thigh"], masses["thigh_to_legs"]),
                RiderGeom(f"geom_rider_shank_{side}", "capsule", knee, ankle, r["shank"], masses["shank_foot"]),
                RiderGeom(f"geom_rider_foot_{side}", "capsule", ankle, pedal, r["foot"], 0.0),
            ),
            stiffness_n_m=k_leg,
            damping_ns_m=c_leg,
            supported_mass_kg=masses["leg"],
            unilateral=True,
            interface="pedals",
        )

    def leg_chain(side: str, knee: np.ndarray, ankle: np.ndarray, pedal: np.ndarray,
                  lateral_y_m: float, crank_len_m: float) -> LegChain:
        return LegChain(
            side=side,
            lateral_y_m=lateral_y_m,
            crank_len_m=crank_len_m,
            hip=hip,
            knee=knee,
            ankle=ankle,
            pedal=pedal,
            thigh_len_m=float(l_thigh),
            shank_len_m=float(l_shank),
            thigh_mass_kg=masses["thigh_to_legs"],
            shank_mass_kg=shank_mass,
            foot_mass_kg=foot_mass,
            pitch0_thigh_rad=_pitch_xz(knee - hip),
            pitch0_shank_rad=_pitch_xz(ankle - knee),
            pitch0_foot_rad=_pitch_xz(pedal - ankle),
        )

    # Front leg is the right one: its pedal sits at -y (`mujoco/drivetrain.py` pairs
    # "front" with lateral -1). Its crank arm leads at phase 0; the rear arm is back,
    # which a negative crank length expresses in `pedal_spindle_pos`.
    if rider.legs == "articulated":
        bodies = (pelvis, torso, arms)
        leg_chains = (
            leg_chain("front", knee_front, ankle_front, pedal_front, -PEDAL_LATERAL_OFFSET_M, crank_m),
            leg_chain("rear", knee_rear, ankle_rear, pedal_rear, PEDAL_LATERAL_OFFSET_M, -crank_m),
        )
    else:
        bodies = (
            pelvis,
            torso,
            arms,
            leg("front", knee_front, ankle_front, pedal_front),
            leg("rear", knee_rear, ankle_rear, pedal_rear),
        )
        leg_chains = ()
    return SeatedPose(
        saddle=saddle,
        hip=hip,
        shoulder=shoulder,
        elbow=elbow,
        wrist=wrist,
        grip=grip,
        head_center=head_center,
        knee_front=knee_front,
        knee_rear=knee_rear,
        ankle_front=ankle_front,
        ankle_rear=ankle_rear,
        pedal_front=pedal_front,
        pedal_rear=pedal_rear,
        torso_lean_deg=float(torso_lean),
        knee_flexion_front_deg=_flexion_deg(hip, knee_front, ankle_front),
        knee_flexion_rear_deg=_flexion_deg(hip, knee_rear, ankle_rear),
        knee_flexion_bdc_deg=float(knee_flexion_bdc),
        bodies=bodies,
        path_masses_kg={k: masses[k] for k in ("torso", "pelvis", "arms", "leg")},
        leg_chains=leg_chains,
    )


def saddle_path_apparent_mass(pose: SeatedPose, frequencies_hz: np.ndarray) -> np.ndarray:
    """
    Apparent mass of the saddle path -- pelvis on the saddle contact, torso on the pelvis --
    for a vertical base excitation, normalized by its static mass.

    This is the quantity the seated-body literature measures on a shaker (Fairley & Griffin
    1989; Kumar & Saran 2019): the force the seat has to exert per unit seat acceleration. It
    is 1.0 at low frequency and peaks near the body's first mode.

    Args:
        pose: Solved seated pose supplying the two masses and spring-dampers.
        frequencies_hz: Excitation frequencies.

    Returns:
        |F_seat| / (|a_seat| (m_pelvis + m_torso)) at each frequency.
    """
    pelvis = pose.body("rider_pelvis")
    torso = pose.body("rider_torso")
    m1, k1, c1 = pelvis.mass, pelvis.stiffness_n_m, pelvis.damping_ns_m
    m2, k2, c2 = torso.mass, torso.stiffness_n_m, torso.damping_ns_m
    s = 2j * np.pi * np.asarray(frequencies_hz, dtype=float)
    z1 = k1 + c1 * s
    z2 = k2 + c2 * s
    # Unit base displacement: pelvis and seat-force responses of the two-mass chain.
    denominator = m1 * s * s + z1 + z2 - z2 * z2 / (m2 * s * s + z2)
    x1 = z1 / denominator
    seat_force = z1 * (1.0 - x1)
    return np.abs(seat_force) / (np.abs(s) ** 2) / (m1 + m2)


def rider_from_args(variant: str, mass_kg: float = 80.0, height_m: float = 1.80,
                    inseam_m: Optional[float] = None) -> RiderSpecs:
    """Builds a `RiderSpecs` from command-line style values."""
    return RiderSpecs(variant=variant, mass_kg=float(mass_kg), height_m=float(height_m), inseam_m=inseam_m)


def resolve_rider(rider: Optional[object], include_rider: Optional[bool] = None,
                  default_variant: str = DEFAULT_RIDER_VARIANT) -> RiderSpecs:
    """
    Normalizes the ways a caller can name a rider into a `RiderSpecs`.

    Args:
        rider: A `RiderSpecs`, a variant name, or None.
        include_rider: Legacy flag. With ``rider`` None: False means ``none``; True means
            ``default_variant``.
        default_variant: Variant a bare ``True`` or a bare None resolves to.
    """
    if isinstance(rider, RiderSpecs):
        return rider
    if isinstance(rider, str):
        return RiderSpecs(variant=rider)
    if rider is not None:
        raise TypeError(f"rider must be a RiderSpecs, a variant name or None, got {type(rider).__name__}")
    if include_rider is False:
        return RiderSpecs(variant="none")
    return RiderSpecs(variant=default_variant)


__all__ = [
    "GRAVITY_MPS2",
    "RIDER_VARIANTS",
    "DEFAULT_RIDER_VARIANT",
    "DE_LEVA_MASS_FRACTIONS",
    "DE_LEVA_LENGTH_FRACTIONS",
    "DE_LEVA_REFERENCE_STATURE_M",
    "INSEAM_STATURE_RATIO",
    "LEMOND_SADDLE_HEIGHT_RATIO",
    "HIP_ABOVE_SADDLE_M",
    "ANKLE_ABOVE_PEDAL_M",
    "KNEE_FLEXION_BDC_MIN_DEG",
    "KNEE_FLEXION_BDC_MAX_DEG",
    "RiderSpecs",
    "RiderGeom",
    "RiderBody",
    "SeatedPose",
    "LegChain",
    "seated_path_masses",
    "solve_seated_pose",
    "pedal_spindle_pos",
    "solve_leg_joints",
    "solve_leg_qpos",
    "fk_leg",
    "leg_jacobian",
    "crank_torque_share",
    "saddle_path_apparent_mass",
    "rider_from_args",
    "resolve_rider",
]
