"""
MuJoCo MJCF Model Builder and XML Generator.

Assembles the full bicycle simulation model from submodules and serializes clean,
prettified MuJoCo MJCF XML.
"""

from typing import Any, Dict, Optional, Union
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
from bike_sim.physics.tyre import TYRE_MODELS
from bike_sim.physics.mass import BikeMassSpecs, compute_static_system_cg
from bike_sim.physics.rider import RiderSpecs, resolve_rider

from bike_sim.mujoco.drivetrain import build_chain_constraint
from bike_sim.physics.drivetrain import DrivetrainSpecs
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


# Default single gear, 32x14: 25 km/h on the 352 mm rear wheel is 82 rpm at the cranks.
DEFAULT_GEAR_RATIO = DrivetrainSpecs().gear_ratio


def generate_mujoco_xml(
    specs: Optional[BikeSpecs] = None,
    solver: Optional[HorstLinkageSolver] = None,
    mode: str = "standard",
    mass_specs: Optional[BikeMassSpecs] = None,
    include_rider: bool = False,
    debug_markers: bool = False,
    field: Optional[HeightFieldSpec] = None,
    rider: Optional[Union[RiderSpecs, str]] = None,
    tyre_model: str = "sphere",
    crank_joint: bool = False,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> str:
    """
    Generates a complete, valid, high-fidelity MuJoCo MJCF XML model string.

    Args:
        specs: Bicycle geometric specifications.
        solver: Solved Horst-link kinematics solver instance.
        mode: Simulation mode ("standard", "stand", "playground", "ride").
        mass_specs: Physical component mass specifications.
        include_rider: Legacy switch for the lumped rider: True builds the 80 kg standing
            rider into `frame`, False builds the bike alone. Ignored when ``rider`` is given.
        debug_markers: Whether to include yellow debug joint markers.
        field: Ride-mode heightfield geometry. Defaults to the shipped field pinned by
            the golden baseline; a longer track passes a stretched field. Ignored
            outside ride mode.
        rider: The rider to build -- a `RiderSpecs` or a variant name (``none``, ``lumped``,
            ``seated``). The seated rider is ride-mode only; its ``legs`` field selects the
            rigid slide-mounted legs or articulated hip/knee/ankle chains hung off the
            pelvis. The centre-of-gravity site and the saddle height follow whichever
            rider is present.
        tyre_model: ``sphere`` keeps the existing wheel–road contacts; ``pneumatic`` disables
            only the two contact spheres so the ride-mode force applier can carry the wheels.
        crank_joint: Ride-mode only. Builds the crankset on a `crank_spin` hinge with a
            `pedal_*` body on each arm end, adds the `crank_drive` motor and ties the crank
            to the rear wheel with the chain equality. With an articulated-leg seated rider
            the feet are additionally welded to the pedal bodies; without `crank_joint`
            those welds do not exist (the pedals stay rigid frame geoms) and the leg
            chains just hang.
        gear_ratio: Wheel revolutions per crank revolution for that chain equality.

    Returns:
        Formatted MJCF XML string ready for MuJoCo simulation.

    Raises:
        ValueError: If the tyre model is unknown, a non-sphere model is requested outside
            ride mode, or a seated rider is requested outside ride mode or does not fit.
    """
    if crank_joint and mode != "ride":
        raise ValueError(f"the pedalled crankset is a ride-mode model; mode {mode!r} builds a rigid crankset")
    if tyre_model not in TYRE_MODELS:
        raise ValueError(f"unknown tyre model '{tyre_model}'; available: {', '.join(TYRE_MODELS)}")
    if tyre_model != "sphere" and mode != "ride":
        raise ValueError(f"tyre model {tyre_model!r} is ride-mode only")
    if specs is None:
        specs = BikeSpecs()
    if solver is None:
        solver = HorstLinkageSolver(specs)
    if mass_specs is None:
        mass_specs = BikeMassSpecs()
    if field is None:
        field = FIELD
    rider_specs = resolve_rider(rider, include_rider=include_rider, default_variant="lumped")
    if rider_specs.variant == "seated" and mode != "ride":
        raise ValueError(f"the seated rider is a ride-mode model; mode {mode!r} takes 'none' or 'lumped'")
    pose = rider_specs.seated_pose(specs) if rider_specs.variant == "seated" else None

    # Compute uncompressed reference state (0 mm wheel travel)
    st0 = solver.solve_state_from_wheel_travel(0.0)
    fixed = get_fixed_frame_points(specs)
    trail_info = compute_trail(specs)
    # The CG site marks the static centre of the whole system, rider included when present.
    cg_info = compute_static_system_cg(
        specs, mass_specs, solver=solver,
        rider_specs=rider_specs if rider_specs.present else None,
    )
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
        rider=rider_specs,
        pose=pose,
        debug_markers=debug_markers,
        crank_joint=crank_joint,
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
        tyre_model=tyre_model,
    )

    # 6. Rear Linkage, Damper & Rear Wheel
    build_rear_linkage(
        frame=frame,
        mode=mode,
        specs=specs,
        fixed_points=fixed,
        solved_points=st0,
        debug_markers=debug_markers,
        tyre_model=tyre_model,
    )

    # 7. Constraints & Collisions
    build_equality_constraints(root, mode=mode)
    if crank_joint:
        build_chain_constraint(root, gear_ratio=gear_ratio)
        if pose is not None and pose.leg_chains:
            # Clipless-pedal welds: each foot body is fixed to its pedal body. No
            # `relpose` means the weld datum is the relative pose at qpos0 -- the
            # design pose, which the pose solver makes self-consistent -- so the
            # constraints start residual-free. Pedal bodies only exist when
            # `crank_joint` builds them, so no welds are emitted without it; the
            # articulated chains then just hang off the pelvis. The 3 ms solref is
            # stiffer than the default 20 ms because this weld is the force path:
            # the downstroke drives several hundred newtons through it, and a soft
            # weld reads as millimetres of foot-pedal separation under load. 5 ms
            # held steady pedalling (~1.5 mm) but let the freewheel engagement kick
            # -- the crank re-spun to wheel speed while the feet are near-still --
            # bow the weld to ~4 mm transiently; 3 ms keeps that under 2 mm and is
            # still 6 timesteps of critically damped response.
            equality = root.find("equality")
            for chain in pose.leg_chains:
                ET.SubElement(
                    equality,
                    "weld",
                    {
                        "name": f"weld_foot_{chain.side}",
                        "body1": f"rider_foot_{chain.side}",
                        "body2": f"pedal_{chain.side}",
                        "solref": "0.003 1",
                    },
                )
    build_contact_exclusions(root)

    # 8. Actuators & Sensors
    build_actuators(root, mode=mode, crank_joint=crank_joint)
    build_sensors(root, mode=mode, seated_rider=(rider_specs.variant == "seated"))

    # Prettify XML
    xml_raw = ET.tostring(root, encoding="utf-8")
    parsed_dom = minidom.parseString(xml_raw)
    pretty_xml = parsed_dom.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")

    cleaned_lines = [line for line in pretty_xml.split("\n") if line.strip() != ""]
    return "\n".join(cleaned_lines) + "\n"
