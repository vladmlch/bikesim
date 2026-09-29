"""Physical wheel mass, tensor, and fixed-axis response contracts."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.inertia import (
    FRONT_WHEEL_PROFILE,
    REAR_WHEEL_PROFILE,
    add_body_inertial,
    parallel_axis,
    ring_inertia,
)
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig


PHYSICAL = SimulationPhysicsConfig(physics_mode="physical")


def _wheel_xml(*, mass_specs=None, physics_config=PHYSICAL):
    return ET.fromstring(generate_mujoco_xml(
        specs=BikeSpecs(), mode="ride", rider="none", mass_specs=mass_specs,
        physics_config=physics_config,
    ))


def _body_tensor(model, name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body_id])
    rotation = rotation.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T


def test_ring_inertia_has_mass_at_the_rim():
    tensor = ring_inertia(1.0, 0.30, 0.35, 0.06)
    assert tensor[1, 1] == pytest.approx(0.10625)
    assert np.linalg.eigvalsh(tensor).min() > 0
    assert tensor[0, 0] + tensor[2, 2] >= tensor[1, 1]


def test_parallel_axis_uses_full_offset_vector():
    np.testing.assert_allclose(parallel_axis(2.0, (1.0, 2.0, 0.0)), [
        [8.0, -4.0, 0.0], [-4.0, 2.0, 0.0], [0.0, 0.0, 10.0],
    ])


@pytest.mark.parametrize("values", [
    (0.0, 0.30, 0.35, 0.06),
    (1.0, -0.01, 0.35, 0.06),
    (1.0, 0.35, 0.35, 0.06),
    (1.0, 0.30, 0.35, 0.0),
    (float("nan"), 0.30, 0.35, 0.06),
])
def test_ring_inertia_rejects_invalid_components(values):
    with pytest.raises(ValueError):
        ring_inertia(*values)


@pytest.mark.parametrize("mass,offset", [
    (-1.0, (0, 0, 0)), (1.0, (0, 0)), (1.0, (0, float("inf"), 0)),
])
def test_parallel_axis_rejects_invalid_inputs(mass, offset):
    with pytest.raises(ValueError):
        parallel_axis(mass, offset)


def test_add_body_inertial_emits_one_full_tensor_and_rejects_duplicate():
    body = ET.Element("body")
    tensor = np.diag((0.053425, 0.10625, 0.053425))
    add_body_inertial(body, 1.0, (0, 0, 0), tensor)
    inertial = body.find("inertial")
    assert inertial is not None
    assert inertial.get("fullinertia") == "0.053425 0.10625 0.053425 0 0 0"
    with pytest.raises(ValueError, match="already"):
        add_body_inertial(body, 1.0, (0, 0, 0), tensor)


def test_add_body_inertial_rejects_nonphysical_tensor():
    with pytest.raises(ValueError, match="triangle"):
        add_body_inertial(ET.Element("body"), 1.0, (0, 0, 0), np.diag((1.0, 1.0, 3.0)))


def test_fixed_axis_acceleration_uses_declared_inertia():
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option gravity="0 0 0"/>
      <worldbody><body><joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.053425 0.10625 0.053425"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qfrc_applied[0] = 1.0625
    mujoco.mj_forward(model, data)
    assert data.qacc[0] == pytest.approx(10.0, rel=1e-3)


@pytest.mark.parametrize("name,profile", [
    ("front_wheel", FRONT_WHEEL_PROFILE),
    ("rear_wheel", REAR_WHEEL_PROFILE),
])
def test_compiled_wheel_mass_com_and_tensor_match_selected_synthetic_profile(name, profile):
    mass_specs = BikeMassSpecs()
    budget = getattr(mass_specs, f"{name}_mass")
    component_masses = [budget * component.mass_fraction for component in profile]
    expected_mass = sum(component_masses)
    expected_com = sum(
        (mass * np.asarray(component.offset_m) for mass, component in zip(component_masses, profile)),
        np.zeros(3),
    ) / expected_mass
    expected_tensor = np.zeros((3, 3))
    for mass, component in zip(component_masses, profile):
        radial = component.inner_radius_m**2 + component.outer_radius_m**2
        transverse = mass * (3 * radial + component.width_m**2) / 12
        displacement = np.asarray(component.offset_m) - expected_com
        expected_tensor += np.diag((transverse, mass * radial / 2, transverse))
        expected_tensor += mass * (
            float(displacement @ displacement) * np.eye(3) - np.outer(displacement, displacement)
        )

    root = _wheel_xml(mass_specs=mass_specs)
    wheel = root.find(f".//body[@name='{name}']")
    assert wheel is not None
    inertials = wheel.findall("inertial")
    assert len(inertials) == 1
    assert expected_mass == pytest.approx(budget)
    share = .9 if name == 'rear_wheel' else 1.
    assert float(inertials[0].get("mass")) == pytest.approx(expected_mass*share)
    assert all(float(geom.get("mass")) == 0 for geom in wheel.findall("geom"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert model.body_mass[body_id] == pytest.approx(expected_mass*share)
    np.testing.assert_allclose(model.body_ipos[body_id], expected_com, rtol=0, atol=1e-12)
    combined_tensor=_body_tensor(model,name)
    if name=='rear_wheel':
        cassette=model.body('cassette').id
        assert model.body_parentid[body_id]==model.body_parentid[cassette]
        assert model.body_mass[cassette]==pytest.approx(.1*expected_mass)
        np.testing.assert_allclose(model.body_ipos[cassette],expected_com,atol=1e-12)
        combined_tensor+=_body_tensor(model,'cassette')
    # D1 transfers cassette inertia to a sibling, preserving the locked assembly.
    np.testing.assert_allclose(combined_tensor, expected_tensor, rtol=0, atol=1e-10)


@pytest.mark.parametrize("name", ["front_wheel", "rear_wheel"])
def test_decorative_wheel_geometry_does_not_change_compiled_inertia(name):
    root = _wheel_xml()
    original = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    wheel = root.find(f".//body[@name='{name}']")
    wheel.find(f"geom[@name='geom_{name.removesuffix('_wheel')}_rim']").set("size", "0.05")
    modified = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    np.testing.assert_allclose(_body_tensor(modified, name), _body_tensor(original, name), rtol=0, atol=1e-12)


def test_selected_wheel_inertia_sets_fixed_axis_energy_and_acceleration():
    wheel = _wheel_xml().find(".//body[@name='front_wheel']")
    inertial = ET.tostring(wheel.find("inertial"), encoding="unicode")
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><option gravity="0 0 0"><flag energy="enable"/></option>'
        '<worldbody><body><joint type="hinge" axis="0 1 0" damping="0"/>'
        f'{inertial}</body></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    data.qvel[0] = 4.0
    data.qfrc_applied[0] = 0.2173131 * 10.0
    mujoco.mj_forward(model, data)
    assert data.energy[1] == pytest.approx(0.5 * 0.2173131 * 4.0**2, rel=1e-10)
    assert data.qacc[0] == pytest.approx(10.0, rel=1e-10)


def test_legacy_wheels_retain_geom_masses_without_explicit_inertial():
    root = _wheel_xml(physics_config=SimulationPhysicsConfig())
    for name in ("front_wheel", "rear_wheel"):
        wheel = root.find(f".//body[@name='{name}']")
        assert wheel.find("inertial") is None
        assert sum(float(geom.get("mass")) for geom in wheel.findall("geom")) > 0


def test_physical_wheel_mass_override_scales_selected_tensor():
    baseline = mujoco.MjModel.from_xml_string(ET.tostring(_wheel_xml(), encoding="unicode"))
    mass = BikeMassSpecs(front_wheel_mass=4.8)
    doubled = mujoco.MjModel.from_xml_string(ET.tostring(_wheel_xml(mass_specs=mass), encoding="unicode"))
    np.testing.assert_allclose(_body_tensor(doubled, "front_wheel"), 2 * _body_tensor(baseline, "front_wheel"), rtol=0, atol=1e-10)
    np.testing.assert_allclose(_body_tensor(doubled, "rear_wheel"), _body_tensor(baseline, "rear_wheel"), rtol=0, atol=1e-10)
