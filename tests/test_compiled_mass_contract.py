"""Compiled bicycle mass and moving center-of-mass contracts."""

from dataclasses import fields
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_front_axle, get_fixed_frame_points
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.component_masses import assign_component_mass
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.mass_properties import compiled_center_of_mass, static_contact_loads
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride_sim import RideSimulation


MASS_FIELDS = tuple(field.name for field in fields(BikeMassSpecs) if field.name.endswith("_mass"))
PHYSICAL = SimulationPhysicsConfig(physics_mode="physical")


def _model(mass_specs=None):
    xml = generate_mujoco_xml(
        specs=BikeSpecs(), mode="ride", mass_specs=mass_specs,
        rider="none", physics_config=PHYSICAL,
    )
    return mujoco.MjModel.from_xml_string(xml)


def _geom_masses(mass, *, debug_markers=False):
    xml = generate_mujoco_xml(
        specs=BikeSpecs(), mode="ride", mass_specs=mass, rider="none",
        physics_config=PHYSICAL, debug_markers=debug_markers,
    )
    return {
        geom.get("name"): float(geom.get("mass"))
        for geom in ET.fromstring(xml).iter("geom") if geom.get("mass")
    }


def _wheel_inertial_masses(mass):
    xml = generate_mujoco_xml(
        specs=BikeSpecs(), mode="ride", mass_specs=mass, rider="none",
        physics_config=PHYSICAL,
    )
    root = ET.fromstring(xml)
    groups={'front_wheel':('front_wheel',),'rear_wheel':('rear_wheel','cassette'),
            'crank_pedals':('crank','pedal_front','pedal_rear')}
    return {key:sum(float(root.find(f".//body[@name='{name}']/inertial").get('mass'))
                    for name in names) for key,names in groups.items()}


@pytest.mark.parametrize("field", MASS_FIELDS)
def test_changing_component_mass_changes_compiled_total(field):
    mass = BikeMassSpecs()
    setattr(mass, field, getattr(mass, field) + 0.7)
    assert _model(mass).body_mass.sum() == pytest.approx(mass.total_bike_mass, abs=1e-8)


def test_default_physical_total_matches_budget():
    assert _model().body_mass.sum() == pytest.approx(BikeMassSpecs().total_bike_mass, abs=1e-8)


def test_tiny_positive_motor_budget_compiles_without_rounding_to_zero():
    mass = BikeMassSpecs(motor_mass=0.001)
    assert _geom_masses(mass)["geom_motor_core"] == pytest.approx(0.001, abs=1e-17)
    assert _model(mass).body_mass.sum() == pytest.approx(mass.total_bike_mass, abs=1e-8)


@pytest.mark.parametrize("field", MASS_FIELDS)
def test_tiny_positive_component_budget_keeps_its_mass_owner(field):
    mass = BikeMassSpecs()
    baseline = _geom_masses(mass)
    setattr(mass, field, 0.0001)
    tiny = _geom_masses(mass)
    assert _model(mass).body_mass.sum() == pytest.approx(mass.total_bike_mass, abs=1e-8)
    changed = [name for name in baseline if tiny[name] != baseline[name]]
    if field in ("front_wheel_mass", "rear_wheel_mass", "crank_pedals_mass"):
        wheel = field.removesuffix("_mass")
        assert changed == []
        assert _wheel_inertial_masses(mass)[wheel] == pytest.approx(0.0001, abs=1e-17)
        return
    assert changed
    scale = 0.0001 / getattr(BikeMassSpecs(), field)
    for name in changed:
        assert tiny[name] == pytest.approx(baseline[name] * scale, abs=1e-12)


def test_component_mass_and_distribution_provenance_is_synthetic():
    mass = BikeMassSpecs()
    assert mass.component_provenance == {
        component_id: {"mass": "synthetic", "distribution": "synthetic"}
        for component_id in mass.component_masses
    }
    assert len(mass.component_provenance) == 15


def test_lumped_rider_is_not_absorbed_into_frame_budget():
    xml = generate_mujoco_xml(
        specs=BikeSpecs(), mode="ride", mass_specs=BikeMassSpecs(),
        rider="lumped", physics_config=PHYSICAL,
    )
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.body_mass.sum() == pytest.approx(BikeMassSpecs().total_bike_mass + 80.0, abs=1e-8)


def test_all_fifteen_mass_fields_are_budget_components():
    assert len(MASS_FIELDS) == 15
    assert set(BikeMassSpecs().component_masses) == {name.removesuffix("_mass") for name in MASS_FIELDS}


@pytest.mark.parametrize("field", MASS_FIELDS)
def test_component_override_changes_only_its_own_mass_group(field):
    baseline = _geom_masses(BikeMassSpecs())
    changed = BikeMassSpecs()
    setattr(changed, field, getattr(changed, field) + 0.7)
    override = _geom_masses(changed)
    changed_geoms = [name for name in baseline if abs(override[name] - baseline[name]) > 1e-12]
    if field in ("front_wheel_mass", "rear_wheel_mass", "crank_pedals_mass"):
        wheel = field.removesuffix("_mass")
        inertial_deltas = {
            name: value - _wheel_inertial_masses(BikeMassSpecs())[name]
            for name, value in _wheel_inertial_masses(changed).items()
        }
        assert changed_geoms == []
        assert inertial_deltas[wheel] == pytest.approx(0.7, abs=1e-10)
        assert all(delta == 0 for key,delta in inertial_deltas.items() if key != wheel)
        return
    assert changed_geoms
    assert sum(override[name] - baseline[name] for name in changed_geoms) == pytest.approx(0.7, abs=1e-10)


def test_debug_geoms_remain_massless():
    masses = _geom_masses(BikeMassSpecs(), debug_markers=True)
    assert all(mass == 0 for name, mass in masses.items() if name.startswith("marker_") or name.endswith("_contact"))


def test_frame_structure_override_does_not_change_motor_or_battery_geom_mass():
    baseline = _geom_masses(BikeMassSpecs())
    changed = BikeMassSpecs(frame_structure_mass=4.2)
    override = _geom_masses(changed)
    for name in ("geom_motor_core", "geom_battery_pack"):
        assert override[name] == pytest.approx(baseline[name])


def test_root_translation_moves_compiled_com_without_changing_mass_or_inertia():
    model = _model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial_com = compiled_center_of_mass(model, data)
    body_mass = model.body_mass.copy()
    body_inertia = model.body_inertia.copy()
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
    data.qpos[model.jnt_qposadr[root]] += 1.0
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(compiled_center_of_mass(model, data), initial_com + [1, 0, 0], atol=1e-12)
    np.testing.assert_array_equal(model.body_mass, body_mass)
    np.testing.assert_array_equal(model.body_inertia, body_inertia)


def test_mass_override_preserves_geometry():
    specs = BikeSpecs()
    before = get_fixed_frame_points(specs)
    front = compute_front_axle(specs)
    linkage = HorstLinkageSolver(specs).solve_state_from_wheel_travel(0.0)
    _model(BikeMassSpecs(frame_structure_mass=4.2, rear_wheel_mass=3.5))
    for name, point in before.items():
        np.testing.assert_array_equal(get_fixed_frame_points(specs)[name], point)
    np.testing.assert_array_equal(compute_front_axle(specs), front)
    for name, point in linkage.items():
        np.testing.assert_array_equal(HorstLinkageSolver(specs).solve_state_from_wheel_travel(0.0)[name], point)


@pytest.mark.parametrize("total,weights", [(0, [1]), (-1, [1]), (float("nan"), [1]), (1, []), (1, [0]), (1, [-1]), (1, [float("inf")])])
def test_assign_component_mass_rejects_invalid_budget(total, weights):
    geoms = [ET.Element("geom", mass=str(weight)) for weight in weights]
    with pytest.raises(ValueError):
        assign_component_mass(geoms, total)


def test_assign_component_mass_preserves_relative_weights_and_precision():
    geoms = [ET.Element("geom", mass="1"), ET.Element("geom", mass="2")]
    assign_component_mass(geoms, 0.7)
    assert float(geoms[1].get("mass")) / float(geoms[0].get("mass")) == pytest.approx(2)
    assert sum(float(geom.get("mass")) for geom in geoms) == pytest.approx(0.7, abs=1e-16)


def test_compiled_com_rejects_zero_mass_model():
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="empty"/></worldbody></mujoco>')
    with pytest.raises(ValueError, match="physical mass"):
        compiled_center_of_mass(model, mujoco.MjData(model))


def test_ride_simulation_passes_mass_budget_and_marks_current_com():
    mass = BikeMassSpecs(frame_structure_mass=4.2)
    sim = RideSimulation(rider="none", mass_specs=mass, physics_config=PHYSICAL)
    assert sim.model.body_mass.sum() == pytest.approx(mass.total_bike_mass, abs=1e-8)
    cg_site = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "site_CG")
    np.testing.assert_allclose(sim.data.site_xpos[cg_site], compiled_center_of_mass(sim.model, sim.data), atol=1e-9)

    recorder = RideRecorder(sim)
    before_com = sim.data.site_xpos[cg_site].copy()
    recorder.record(sim)
    assert recorder.rows == 0  # A state alone is not a solved force interval.
    sim.step()
    recorder.record(sim)
    assert recorder.column("compiled_mass_kg")[0] == pytest.approx(mass.total_bike_mass)
    assert recorder.column("compiled_com_x_m")[0] == pytest.approx(before_com[0])


def test_static_load_uses_contact_locations_after_sag():
    sim = RideSimulation(rider="none", physics_config=PHYSICAL)
    front = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_front_contact")
    rear = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rear_contact")
    initial_front, initial_rear = static_contact_loads(sim.model, sim.data, front, rear)
    assert initial_front + initial_rear == pytest.approx(sim.model.body_mass.sum() * 9.81)
    for contact in sim.data.contact:
        if front in (contact.geom1, contact.geom2):
            contact.pos[0] += 0.1
    shifted_front, _ = static_contact_loads(sim.model, sim.data, front, rear)
    assert shifted_front < initial_front
