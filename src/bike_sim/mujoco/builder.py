"""
MuJoCo MJCF Model Builder and XML Generator.

Assembles the full bicycle simulation model from submodules and serializes clean,
prettified MuJoCo MJCF XML.
"""

from typing import Any, Dict, Optional
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.hardpoints import (
    compute_front_axle,
    compute_ground_z,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.physics.mass import BikeMassSpecs, compute_static_system_cg

from bike_sim.mujoco.assets import build_visual_and_assets
from bike_sim.mujoco.environment import build_environment
from bike_sim.mujoco.frame import build_frame_body
from bike_sim.mujoco.linkage import (
    build_contact_exclusions,
    build_equality_constraints,
    build_rear_linkage,
    build_steering_and_fork,
)
from bike_sim.mujoco.actuators import build_actuators
from bike_sim.mujoco.sensors import build_sensors
from bike_sim.mujoco.terrain import build_terrain
from bike_sim.terrain.heightfield import FIELD, HeightFieldSpec


def generate_mujoco_xml(
    specs: Optional[BikeSpecs] = None,
    solver: Optional[HorstLinkageSolver] = None,
    mode: str = "standard",
    mass_specs: Optional[BikeMassSpecs] = None,
    include_rider: bool = False,
    debug_markers: bool = False,
    field: Optional[HeightFieldSpec] = None,
) -> str:
    """
    Generates a complete, valid, high-fidelity MuJoCo MJCF XML model string.

    Args:
        specs: Bicycle geometric specifications.
        solver: Solved Horst-link kinematics solver instance.
        mode: Simulation mode ("standard", "stand", "playground", "ride").
        mass_specs: Physical component mass specifications.
        include_rider: Whether to include rider geometry and mass.
        debug_markers: Whether to include yellow debug joint markers.
        field: Ride-mode heightfield geometry. Defaults to the shipped field pinned by
            the golden baseline; a longer track passes a stretched field. Ignored
            outside ride mode.

    Returns:
        Formatted MJCF XML string ready for MuJoCo simulation.
    """
    if specs is None:
        specs = BikeSpecs()
    if solver is None:
        solver = HorstLinkageSolver(specs)
    if mass_specs is None:
        mass_specs = BikeMassSpecs()
    if field is None:
        field = FIELD

    # Compute uncompressed reference state (0 mm wheel travel)
    st0 = solver.solve_state_from_wheel_travel(0.0)
    fixed = get_fixed_frame_points(specs)
    trail_info = compute_trail(specs)
    cg_info = compute_static_system_cg(specs, mass_specs, solver=solver)
    cg_pos = cg_info["cg_pos_m"]

    # Hardpoints
    P10 = np.array(fixed["P10"], dtype=float) / 1000.0
    P_FA_raw = compute_front_axle(specs)

    ground_z_m = compute_ground_z(specs) / 1000.0

    root = ET.Element("mujoco", {"model": "enduro_bike_horst_yoke"})

    # 1. Compiler & Options
    ET.SubElement(
        root,
        "compiler",
        {
            "angle": "degree",
            "coordinate": "local",
            "inertiafromgeom": "auto",
            "autolimits": "true",
        },
    )
    if mode == "ride":
        timestep = "0.0005"
    elif mode in ("stand", "playground"):
        timestep = "0.001"
    else:
        timestep = "0.002"
    ET.SubElement(
        root,
        "option",
        {
            "gravity": "0 0 -9.81",
            "timestep": timestep,
            "integrator": "implicitfast",
            "iterations": "100",
            "tolerance": "1e-10",
        },
    )

    # 2. Visual & Assets
    build_visual_and_assets(root)

    # 3. Worldbody & Environment
    worldbody = ET.SubElement(root, "worldbody")
    build_environment(
        worldbody=worldbody,
        mode=mode,
        ground_z_m=ground_z_m,
        P10=P10,
        field=field,
    )
    if mode == "ride":
        build_terrain(root, worldbody, ground_z_m, spec=field)

    # 4. Frame & Attached Bodies
    frame = build_frame_body(
        worldbody=worldbody,
        mode=mode,
        specs=specs,
        mass_specs=mass_specs,
        fixed_points=fixed,
        cg_pos=cg_pos,
        include_rider=include_rider,
        debug_markers=debug_markers,
    )

    # 5. Steering, Fork & Front Wheel
    build_steering_and_fork(
        frame=frame,
        mode=mode,
        specs=specs,
        mass_specs=mass_specs,
        fixed_points=fixed,
        front_axle=P_FA_raw,
        debug_markers=debug_markers,
    )

    # 6. Rear Linkage, Damper & Rear Wheel
    build_rear_linkage(
        frame=frame,
        mode=mode,
        specs=specs,
        fixed_points=fixed,
        solved_points=st0,
        debug_markers=debug_markers,
    )

    # 7. Constraints & Collisions
    build_equality_constraints(root, mode=mode)
    build_contact_exclusions(root)

    # 8. Actuators & Sensors
    build_actuators(root, mode=mode)
    build_sensors(root, mode=mode)

    # Prettify XML
    xml_raw = ET.tostring(root, encoding="utf-8")
    parsed_dom = minidom.parseString(xml_raw)
    pretty_xml = parsed_dom.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")

    cleaned_lines = [line for line in pretty_xml.split("\n") if line.strip() != ""]
    return "\n".join(cleaned_lines) + "\n"
