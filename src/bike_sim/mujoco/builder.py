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
from bike_sim.physics.mass import BikeMassSpecs, compute_unloaded_analytic_system_cg
from bike_sim.physics.component_masses import assign_component_mass
from bike_sim.physics.model_config import SimulationPhysicsConfig
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
    *,
    physics_config: Optional[SimulationPhysicsConfig] = None,
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
        physics_config: Optional ride-mode physics settings, including the integration step.

    Returns:
        Formatted MJCF XML string ready for MuJoCo simulation.

    Raises:
        ValueError: If the tyre model is unknown, a non-sphere model is requested outside
            ride mode, or a seated rider is requested outside ride mode or does not fit.
    """
    physical = physics_config is not None and physics_config.physics_mode == "physical"
    if physical and mode != "ride":
        raise ValueError("physical configuration requires ride mode")
    if physical and tyre_model != "sphere":
        raise ValueError("legacy pneumatic tyre_model is not a physical backend; configure physics_config.tires")
    if physical:
        crank_joint = True
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
    physical_masses = physics_config is not None and physics_config.physics_mode == "physical"
    mass_registry: dict[str, list[ET.Element]] | None = {} if physical_masses else None
    if field is None:
        field = FIELD
    rider_specs = resolve_rider(rider, include_rider=include_rider, default_variant="lumped")
    if rider_specs.variant == "seated" and mode != "ride":
        raise ValueError(f"the seated rider is a ride-mode model; mode {mode!r} takes 'none' or 'lumped'")
    pose = rider_specs.seated_pose(specs) if rider_specs.variant == "seated" else None
    articulated_pose = None
    if rider_specs.variant == "articulated_planar":
        if not physical:
            raise ValueError("articulated_planar requires physical ride mode")
        from bike_sim.physics.rider_segments import geometry_pose
        articulated_pose = geometry_pose(rider_specs, specs)
    if physical and physics_config.drive_mode == "articulated_effort" and articulated_pose is None:
        raise ValueError("articulated_effort requires articulated_planar rider")
    if physical and pose is not None and pose.leg_chains:
        raise ValueError("physical seated rider supports rigid legs; use articulated_planar")

    # Compute uncompressed reference state (0 mm wheel travel)
    st0 = solver.solve_state_from_wheel_travel(0.0)
    fixed = get_fixed_frame_points(specs)
    trail_info = compute_trail(specs)
    # A nonzero initial offset keeps MuJoCo from compiling the site as a
    # same-frame shortcut; RideSimulation moves it to the compiled CoM at runtime.
    if physical_masses:
        cg_pos = np.array([0.001, 0.0, 0.0])
    else:
        cg_pos = compute_unloaded_analytic_system_cg(
            specs, mass_specs, solver=solver,
            rider_specs=rider_specs if rider_specs.present else None,
        )["cg_pos_m"]

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
        timestep = "0.0005" if physics_config is None else str(physics_config.timestep_s)
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
        pose=articulated_pose if articulated_pose is not None else pose,
        debug_markers=debug_markers,
        crank_joint=crank_joint,
        mass_registry=mass_registry,
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
        mass_registry=mass_registry,
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
        physics_config=physics_config,
        mass_registry=mass_registry,
        mass_specs=mass_specs,
    )

    if mass_registry is not None:
        expected_components = set(mass_specs.component_masses)
        if set(mass_registry) != expected_components:
            raise ValueError(f"mass registry mismatch: {expected_components ^ set(mass_registry)}")
        registered_parts = [part for group in mass_registry.values() for part in group]
        if len({id(part) for part in registered_parts}) != len(registered_parts):
            raise ValueError("a mass-bearing part belongs to multiple components")
        registered_geoms = [part for part in registered_parts if part.tag == "geom"]
        bike_geoms = {
            id(geom) for geom in frame.iter("geom")
            if float(geom.get("mass", "0")) > 0
            and not geom.get("name", "").startswith("geom_rider_")
        }
        if {id(geom) for geom in registered_geoms} != bike_geoms:
            raise ValueError("mass registry does not cover every bike geom exactly once")
        wheel_ids = {"front_wheel", "rear_wheel"}
        wheel_inertials = {
            id(inertial) for body in frame.iter("body")
            if body.get("name") in wheel_ids
            for inertial in body.findall("inertial")
        }
        if {id(part) for part in registered_parts if part.tag == "inertial"} != wheel_inertials:
            raise ValueError("wheel inertial registry mismatch")
        for component_id, parts in mass_registry.items():
            if component_id in wheel_ids:
                if (len(parts) != 1 or parts[0].tag != "inertial"
                        or float(parts[0].get("mass", "0")) != mass_specs.component_masses[component_id]):
                    raise ValueError(f"wheel component {component_id!r} must own one budgeted inertial")
            else:
                if any(part.tag != "geom" for part in parts):
                    raise ValueError(f"component {component_id!r} must own geoms")
                assign_component_mass(parts, mass_specs.component_masses[component_id])

    # 7. Constraints & Collisions
    build_equality_constraints(root, mode=mode)
    if crank_joint and not physical:
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

    if (physics_config is not None
            and (getattr(physics_config.articulated, 'pedal_attachment', 'flat') in ('weld', 'spindle')
                 or getattr(physics_config.articulated, 'saddle_attachment', 'flat') in ('weld', 'pin')
                 or getattr(physics_config.articulated, 'grip_attachment', 'spring') == 'connect')
            and not (physical and articulated_pose is not None)):
        raise ValueError(
            "non-flat rider attachments (weld/pin) require "
            "physics_mode='physical' and rider='articulated_planar'")

    if physical:
        from bike_sim.mujoco.physical_topology import finish_physical_topology
        finish_physical_topology(root, specs, mass_specs, physics_config)
        if articulated_pose is not None:
            from bike_sim.mujoco.articulated_rider import build_articulated_rider, add_rider_actuators
            from bike_sim.physics.rider_segments import segment_masses, split_paired_arm_masses
            for geom in list(frame.findall("geom")):
                if geom.get("name", "").startswith("geom_rider_"):
                    frame.remove(geom)
            from bike_sim.physics.rider_envelope import load_joint_envelopes
            envelope_path=physics_config.articulated.joint_envelope_path
            envelopes=None if envelope_path is None else load_joint_envelopes(envelope_path)
            build_articulated_rider(worldbody, articulated_pose,
                split_paired_arm_masses(segment_masses(rider_specs.mass_kg, rider_specs.helmet_mass_kg)),
                envelopes=envelopes,
                locked_joints=('rider_ankle_front', 'rider_ankle_rear')
                if physics_config.articulated.pedal_attachment == 'spindle' else ())
            add_rider_actuators(root, physics_config.articulated)
            solref = max(2. * physics_config.timestep_s,
                         physics_config.closure_time_constant_s)
            equality = root.find('equality')
            if physics_config.articulated.pedal_attachment == 'weld':
                assert equality is not None
                for side in ('front', 'rear'):
                    ET.SubElement(equality, 'weld', {
                        'name': f'weld_foot_{side}',
                        'body1': f'rider_foot_{side}',
                        'body2': f'pedal_{side}',
                        'solref': f'{solref:.17g} 1',
                    })
            if physics_config.articulated.pedal_attachment == 'spindle':
                from bike_sim.physics.rider import ANKLE_ABOVE_PEDAL_M
                assert equality is not None
                for side in ('front', 'rear'):
                    ET.SubElement(equality, 'connect', {
                        'name': f'connect_foot_{side}',
                        'body1': f'rider_foot_{side}',
                        'body2': f'pedal_{side}',
                        'anchor': f'0 0 {-ANKLE_ABOVE_PEDAL_M:.17g}',
                        'solref': f'{solref:.17g} 1',
                    })
            if physics_config.articulated.saddle_attachment == 'weld':
                assert equality is not None
                ET.SubElement(equality, 'weld', {
                    'name': 'weld_saddle',
                    'body1': 'rider_pelvis',
                    'body2': 'frame',
                    'solref': f'{solref:.17g} 1',
                })
            elif physics_config.articulated.saddle_attachment == 'pin':
                from bike_sim.mujoco.reference_rider import add_saddle_pin
                add_saddle_pin(root, solref)
            if physics_config.articulated.grip_attachment == 'connect':
                assert equality is not None
                # A `connect` pins each grip site to the bar point it already
                # occupies at qpos0: the hand can never leave the bar, but the
                # wrist keeps rotating and the torso keeps its lean-over-hands
                # DOF. A full `weld` here would freeze the whole arm+torso
                # loop rigid to the frame.
                for side in ('left', 'right'):
                    site = root.find(f".//site[@name='site_rider_grip_{side}']")
                    assert site is not None
                    ET.SubElement(equality, 'connect', {
                        'name': f'connect_grip_{side}',
                        'body1': f'rider_forearm_{side}',
                        'body2': 'steer',
                        'anchor': site.get('pos'),
                        'solref': f'{solref:.17g} 1',
                    })
            # Dedicated crash mask: no invisible rider/bike or rider/rider contacts.
            for name in ("geom_rider_head", "geom_rider_pelvis", "geom_rider_torso"):
                geom = root.find(f".//geom[@name='{name}']")
                geom.set("contype", "4")
                geom.set("conaffinity", "0")
            for name in ("terrain", "catch_plane"):
                geom = root.find(f".//geom[@name='{name}']")
                geom.set("conaffinity", str(int(geom.get("conaffinity", "1")) | 4))

    if physical:
        from bike_sim.mujoco.physical_topology import finalize_geometric_transmission
        finalize_geometric_transmission(root,physics_config)

    # Prettify XML
    xml_raw = ET.tostring(root, encoding="utf-8")
    parsed_dom = minidom.parseString(xml_raw)
    pretty_xml = parsed_dom.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")

    cleaned_lines = [line for line in pretty_xml.split("\n") if line.strip() != ""]
    return "\n".join(cleaned_lines) + "\n"
