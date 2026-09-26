"""
MuJoCo Rider Geometry & Mass.

Two rider builders, for the two rider variants that carry mass (docs/RIDE.md section 7):

- `build_rider` -- the **lumped** rider: three capsules of an 80 kg rider in a neutral
  standing attack position, appended to the `frame` body so their mass is rigidly part of
  it. With `include_rider=False` the capsules are still emitted at zero mass and zero
  alpha, so the test stand's in-place toggle has stable geoms to write to.
- `build_seated_rider` -- the **seated** rider: one MuJoCo body per lumped mass of the
  biodynamic model, each on its own vertical slide joint, with the capsules the pose solver
  laid out. The spring-dampers that carry the bodies are *not* MJCF joint springs: they are
  written into `qfrc_applied` by `sim/ride/rider_forces.py`, because the saddle and pedal
  contacts are one-sided and the suspension forces already travel that path.

Every rider geom is non-colliding. A rider that hits the ground is a crash, and the crash
detector (`sim/ride/virtual_rider.py`) already decides that from the frame's attitude and
the handlebar; rider-ground contact would add solver load and nothing else.
"""

from typing import Optional
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.physics.rider import RiderSpecs, SeatedPose
from bike_sim.mujoco._xml_format import _format_fromto, _format_vec, add_geom, add_joint, add_site

# Accelerometer sites the seated rider's telemetry reads: on the chest, and on the pelvis.
SITE_RIDER_TORSO = "site_rider_torso"
SITE_RIDER_PELVIS = "site_rider_pelvis"

RIDER_RGBA_VISIBLE = "0.5 0.5 0.5 1.0"
RIDER_RGBA_HIDDEN = "0.5 0.5 0.5 0.0"


def build_rider(frame: ET.Element, include_rider: bool = True, rider: Optional[RiderSpecs] = None) -> None:
    """
    Appends the lumped rider's capsules (torso, legs, arms) to the frame body.

    Args:
        frame: The `frame` body element.
        include_rider: Whether the capsules carry mass and are drawn. False emits them at
            zero mass and zero alpha.
        rider: Rider masses. Defaults to the 80 kg reference rider.
    """
    rider = rider if rider is not None else RiderSpecs(variant="lumped")
    torso_sz = "0.13" if include_rider else "0.0001"
    legs_sz = "0.075" if include_rider else "0.0001"
    arms_sz = "0.055" if include_rider else "0.0001"
    torso_m = f"{rider.torso_helmet_mass:.1f}" if include_rider else "0.0"
    legs_m = f"{rider.legs_mass:.1f}" if include_rider else "0.0"
    arms_m = f"{rider.arms_mass:.1f}" if include_rider else "0.0"
    rider_rgba = RIDER_RGBA_VISIBLE if include_rider else RIDER_RGBA_HIDDEN

    # 1. Torso & Helmet (55.0 kg)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_torso",
            "type": "capsule",
            "fromto": "0.10 0 0.48 0.22 0 0.82",
            "size": torso_sz,
            "mass": torso_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    # 2. Rider Legs (18.0 kg, from pedals/BB to hips)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_legs",
            "type": "capsule",
            "fromto": "0.0 0 0.0 0.10 0 0.48",
            "size": legs_sz,
            "mass": legs_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    # 3. Rider Arms (7.0 kg, from shoulders to handlebar grips)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_arms",
            "type": "capsule",
            "fromto": "0.22 0 0.78 0.45 0 0.66",
            "size": arms_sz,
            "mass": arms_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )


def build_seated_rider(frame: ET.Element, pose: SeatedPose) -> None:
    """
    Appends the seated rider's bodies, slide joints, capsules and sites to the frame.

    Bodies whose parent is another rider body (the torso on the pelvis) are nested; the
    rest are children of `frame`. Body origins are the pose's attach points and every geom
    is re-expressed relative to its body, so the capsules land where the solver put them.

    Args:
        frame: The `frame` body element.
        pose: The solved seated pose.
    """
    elements = {"frame": frame}
    attach = {"frame": np.zeros(3)}
    # Parents precede children in `pose.bodies`; a nested body must find its parent built.
    for body in pose.bodies:
        if body.parent not in elements:
            raise ValueError(f"rider body {body.name!r} names parent {body.parent!r} before it is built")
        parent_el = elements[body.parent]
        rel_pos = body.attach - attach[body.parent]
        el = ET.SubElement(parent_el, "body", {"name": body.name, "pos": _format_vec(rel_pos)})
        # Along the frame's vertical; every rider body is a chain of pure translations from
        # `frame`, so the axis reads the same in each body's own frame.
        add_joint(el, body.joint, "slide", axis="0 0 1")
        for geom in body.geoms:
            p1 = geom.p1 - body.attach
            p2 = geom.p2 - body.attach
            if geom.kind == "sphere":
                add_geom(
                    el, geom.name, "sphere", pos=p1, size=f"{geom.radius:.3f}",
                    mass=f"{geom.mass:.6f}", rgba=RIDER_RGBA_VISIBLE, material="mat_rider",
                    contype="0", conaffinity="0",
                )
            else:
                add_geom(
                    el, geom.name, "capsule", fromto=_format_fromto(p1, p2), size=f"{geom.radius:.3f}",
                    mass=f"{geom.mass:.6f}", rgba=RIDER_RGBA_VISIBLE, material="mat_rider",
                    contype="0", conaffinity="0",
                )
        elements[body.name] = el
        attach[body.name] = body.attach

    torso = pose.body("rider_torso")
    add_site(
        elements["rider_torso"], SITE_RIDER_TORSO,
        pos=torso.center_of_mass - torso.attach, size="0.008", rgba="0.9 0.6 0.1 1.0",
    )
    pelvis = pose.body("rider_pelvis")
    add_site(
        elements["rider_pelvis"], SITE_RIDER_PELVIS,
        pos=pelvis.center_of_mass - pelvis.attach, size="0.008", rgba="0.9 0.6 0.1 1.0",
    )


__all__ = ["build_rider", "build_seated_rider", "SITE_RIDER_TORSO", "SITE_RIDER_PELVIS"]
