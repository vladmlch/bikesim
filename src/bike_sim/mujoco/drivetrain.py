"""
MuJoCo Drivetrain, Brakes, and Wheel Assembly Builder.

Provides modular builders for:
1. Bottom bracket shell, motor core, crankset, and pedals
2. Rear wheel, hub, rim, tire, cassette, and brake disc
3. Front and rear disc brake calipers and rotors
"""

import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.cockpit import (
    CRANK_ARM_LATERAL_OFFSET_M,
    PEDAL_HALF_SIZE_M,
    PEDAL_LATERAL_OFFSET_M,
    pedal_points,
)
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.mujoco._xml_format import _format_vec, add_geom, add_joint, add_site

# How `crank_pedals_mass` (0.85 kg) is split over the crankset. Authored: a 165 mm alloy arm
# is ~200 g, a platform pedal ~175 g, and the spindle with the chainring makes up the rest.
CRANK_ARM_MASS_KG = 0.20
PEDAL_MASS_KG = 0.175
CRANK_ARM_RADIUS_M = 0.012
# Bearing drag of the bottom bracket, N.m per rad/s. Small enough to be invisible against a
# 30 N.m pedal stroke, large enough that a disengaged crank settles instead of ringing.
CRANK_JOINT_DAMPING = 0.03
CRANK_SPINDLE_RADIUS_M = 0.016


def build_bb_and_motor(
    frame: ET.Element,
    mass_specs: BikeMassSpecs,
    crank_length_m: float = 0.165,
    crank_joint: bool = False,
) -> None:
    """
    Builds the bottom bracket shell, electric motor core, and crankset/pedals on the main frame.

    With `crank_joint` false the cranks are horizontal -- 3 and 9 o'clock, the coasting
    position -- and rigid with the frame, because a free crank with nothing driving it is an
    unactuated pendulum. With `crank_joint` true the spindle, arms and pedals move into a
    `crank` body on a `crank_spin` hinge: the pedalled drivetrain drives that hinge, the chain
    equality ties it to the rear wheel, and the drive torque's reaction lands on the frame
    through the bottom bracket the way a mid-drive's does.

    Args:
        frame: The `frame` body element.
        mass_specs: Component masses; `crank_pedals_mass` is spread over spindle, arms and pedals.
        crank_length_m: Crank arm length, spindle to pedal spindle, in metres.
        crank_joint: Whether to build the crankset on a rotating hinge.
    """
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_bb_shell",
            "type": "cylinder",
            "fromto": "0 -0.0365 0 0 0.0365 0",
            "size": "0.024",
            "mass": "0.30",
            "material": "mat_metal",
        },
    )
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_motor_core",
            "type": "cylinder",
            "fromto": "0.015 -0.055 0.020 0.045 0.055 0.035",
            "size": "0.045",
            "mass": f"{mass_specs.motor_mass:.2f}",
            "material": "mat_motor",
        },
    )
    spindle_mass = mass_specs.crank_pedals_mass - 2.0 * CRANK_ARM_MASS_KG - 2.0 * PEDAL_MASS_KG
    if spindle_mass <= 0.0:
        raise ValueError(
            f"crank_pedals_mass {mass_specs.crank_pedals_mass:.3f} kg does not cover two "
            f"{CRANK_ARM_MASS_KG:.3f} kg arms and two {PEDAL_MASS_KG:.3f} kg pedals"
        )
    # The crankset either stays welded to the frame, as it was before there was a
    # drivetrain, or hangs off its own hinge at the bottom bracket. Every geom below keeps
    # its coordinates either way: the bottom bracket is the frame's origin, so the crank
    # body sits at 0 0 0 and its children need no re-referencing.
    if crank_joint:
        crankset = ET.SubElement(frame, "body", {"name": "crank", "pos": "0 0 0"})
        ET.SubElement(
            crankset,
            "joint",
            {
                "name": "crank_spin",
                "type": "hinge",
                "pos": "0 0 0",
                "axis": "0 1 0",
                "damping": f"{CRANK_JOINT_DAMPING:.3f}",
            },
        )
    else:
        crankset = frame
    add_geom(
        crankset, "geom_crank_spindle", "cylinder",
        fromto=f"0 {-CRANK_ARM_LATERAL_OFFSET_M:.6f} 0 0 {CRANK_ARM_LATERAL_OFFSET_M:.6f} 0",
        size=f"{CRANK_SPINDLE_RADIUS_M:.3f}", mass=f"{spindle_mass:.3f}", material="mat_metal",
        contype="0", conaffinity="0",
    )
    pedal_front, pedal_rear = pedal_points(crank_length_m)
    # Right arm forward at 3 o'clock, left arm back at 9 o'clock. Neither the arms nor the
    # pedals collide: they are 350 mm off the road and would only ever meet it in a crash the
    # detector has already called.
    for side, pedal, lateral in (("front", pedal_front, -1.0), ("rear", pedal_rear, 1.0)):
        y_arm = lateral * CRANK_ARM_LATERAL_OFFSET_M
        y_pedal = lateral * PEDAL_LATERAL_OFFSET_M
        add_geom(
            crankset, f"geom_crank_arm_{side}", "capsule",
            fromto=f"0 {y_arm:.6f} 0 {pedal[0]:.6f} {y_arm:.6f} {pedal[2]:.6f}",
            size=f"{CRANK_ARM_RADIUS_M:.3f}", mass=f"{CRANK_ARM_MASS_KG:.3f}", material="mat_metal",
            contype="0", conaffinity="0",
        )
        if crank_joint:
            # The pedal rides on its own body at the crank arm's spindle end: a passive,
            # lightly damped hinge for the platform to level on (a real pedal bearing),
            # and the body a clipped-in foot welds to. The hinge axis is the spindle line,
            # so anchoring it at the arm end is the same rotation as anchoring it under the
            # platform. The platform box keeps its place under the foot, 40 mm outboard,
            # and `site_pedal` marks the platform centre -- the weld-datum probe the foot's
            # `site_foot` is checked against.
            pedal_body = ET.SubElement(
                crankset,
                "body",
                {"name": f"pedal_{side}", "pos": f"{pedal[0]:.6f} {y_arm:.6f} {pedal[2]:.6f}"},
            )
            add_joint(pedal_body, f"pedal_spin_{side}", "hinge", axis="0 1 0", damping="0.005")
            add_geom(
                pedal_body, f"geom_pedal_{side}", "box",
                pos=f"0 {y_pedal - y_arm:.6f} 0",
                size=" ".join(f"{s:.3f}" for s in PEDAL_HALF_SIZE_M),
                mass=f"{PEDAL_MASS_KG:.3f}", material="mat_metal",
                contype="0", conaffinity="0",
            )
            add_site(pedal_body, f"site_pedal_{side}", pos=f"0 {y_pedal - y_arm:.6f} 0", size="0.006")
        else:
            add_geom(
                crankset, f"geom_pedal_{side}", "box",
                pos=f"{pedal[0]:.6f} {y_pedal:.6f} {pedal[2]:.6f}",
                size=" ".join(f"{s:.3f}" for s in PEDAL_HALF_SIZE_M),
                mass=f"{PEDAL_MASS_KG:.3f}", material="mat_metal",
                contype="0", conaffinity="0",
            )


def build_chain_constraint(root: ET.Element, gear_ratio: float) -> None:
    """
    Ties the crank to the rear wheel with the chain, as a joint equality.

    The constraint is written `crank = wheel / ratio`: the wheel turns `ratio` times per
    crank revolution, so a positive crank torque arrives at the wheel divided by the ratio
    and the power balance closes. It is emitted **active**;
    the freewheel is expressed at runtime by clearing `eq_active`, which is what lets the
    cranks stop while the wheel keeps turning.

    This is deliberately not a chain *line*: the constraint transmits torque, not the tension
    of a chain running from chainring to cog, so the anti-squat that tension would produce is
    outside this model.

    Args:
        root: The MJCF root element.
        gear_ratio: Wheel revolutions per crank revolution.

    Raises:
        ValueError: If the ratio is not positive.
    """
    if gear_ratio <= 0.0:
        raise ValueError(f"gear_ratio must be positive, got {gear_ratio}")
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "chain_drive",
            "joint1": "crank_spin",
            "joint2": "rear_wheel_spin",
            "polycoef": f"0 {1.0 / gear_ratio:.9f} 0 0 0",
            "solref": "0.002 1",
        },
    )


def build_rear_wheel(
    seatstay: ET.Element,
    mode: str,
    p1_rel_p2: np.ndarray,
    rear_wheel_radius_m: float,
    tyre_model: str = "sphere",
) -> ET.Element:
    """
    Builds the rear wheel body (hub, rim, tire, cassette, brake rotor) mounted on the seatstay at P1 (Rear Axle).
    """
    rear_wheel = ET.SubElement(seatstay, "body", {"name": "rear_wheel", "pos": _format_vec(p1_rel_p2)})
    ET.SubElement(
        rear_wheel,
        "joint",
        {
            "name": "rear_wheel_spin",
            "type": "hinge",
            "pos": "0 0 0",
            "axis": "0 1 0",
            "damping": "0.01",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_hub",
            "type": "cylinder",
            "fromto": "0 -0.074 0 0 0.074 0",
            "size": "0.020",
            "mass": "0.30",
            "material": "mat_metal",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_rim",
            "type": "cylinder",
            "fromto": "0 -0.015 0 0 0.015 0",
            "size": "0.300",
            "mass": "0.40",
            "material": "mat_rim",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    tire_contype = "0" if mode == "ride" else "1"
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_tire",
            "type": "cylinder",
            "fromto": "0 -0.032 0 0 0.032 0",
            "size": f"{rear_wheel_radius_m:.6f}",
            "mass": "1.50",
            "material": "mat_tire",
            "contype": tire_contype,
            "conaffinity": tire_contype,
            "friction": "1.2 0.005 0.0001",
        },
    )
    if mode == "ride":
        # Sphere contact patch: see geom_front_contact in steering_fork.py for rationale.
        ET.SubElement(
            rear_wheel,
            "geom",
            {
                "name": "geom_rear_contact",
                "type": "sphere",
                "pos": "0 0 0",
                "size": f"{rear_wheel_radius_m:.6f}",
                "mass": "0",
                "condim": "3",
                "friction": "1.2 0.005 0.0001",
                "solref": "-130000 -800",
                "contype": "0" if tyre_model == "pneumatic" else "1",
                "conaffinity": "0" if tyre_model == "pneumatic" else "1",
                "rgba": "0.08 0.08 0.08 0",
            },
        )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_cassette",
            "type": "cylinder",
            "fromto": "0 -0.055 0 0 -0.035 0",
            "size": "0.095",
            "mass": "0.40",
            "material": "mat_metal",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_rotor",
            "type": "cylinder",
            "fromto": "0 0.038 0 0 0.040 0",
            "size": "0.1015",
            "mass": "0.20",
            "material": "mat_metal",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    return rear_wheel
