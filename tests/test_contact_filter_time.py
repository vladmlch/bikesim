"""Timestamp-based controller contact gating stays separate from physical load."""

import math

import mujoco
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.contact_filter import GroundedFilter
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_filter_does_not_advance_when_read_twice():
    f = GroundedFilter(0.005)
    assert f.update(True, 0.0)
    assert f.update(False, 0.004)
    assert f.update(False, 0.004)
    assert not f.update(False, 0.006)


@pytest.mark.parametrize("hold_s", [-0.001, math.inf, -math.inf, math.nan])
def test_filter_rejects_invalid_hold(hold_s):
    with pytest.raises(ValueError, match="hold_s"):
        GroundedFilter(hold_s)


def test_filter_rejects_invalid_or_backward_time_and_reset_clears_history():
    f = GroundedFilter(0.005)
    assert f.update(True, 1.0)
    with pytest.raises(ValueError, match="non-finite timestamp"):
        f.update(False, math.nan)
    with pytest.raises(ValueError, match="backwards"):
        f.update(False, 0.999)
    assert f.update(False, 1.004)
    f.reset()
    assert not f.update(False, 0.0)
    assert f.update(True, 0.001)


@pytest.mark.parametrize("dt_s", [0.0005, 0.00025, 0.000125])
def test_hold_duration_agrees_within_one_step_at_three_time_steps(dt_s):
    f = GroundedFilter(0.005)
    assert f.update(True, 0.0)
    last_held_s = 0.0
    first_released_s = None
    for step in range(1, 42):
        time_s = step * dt_s
        if f.update(False, time_s):
            last_held_s = time_s
        else:
            first_released_s = time_s
            break
    assert first_released_s is not None
    assert 0.005 - dt_s <= last_held_s < 0.005
    assert 0.005 <= first_released_s < 0.005 + dt_s


def test_native_query_filters_only_working_road_contact_and_keeps_loads_raw():
    sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.0)
    model, data = sim.model, sim.data
    query = TerrainContactQuery(model)
    road = query.query(model, data)
    assert road.front_controller_grounded == road.front_snapshot.road_loaded_contact
    assert road.rear_controller_grounded == road.rear_snapshot.road_loaded_contact
    assert road.front_controller_grounded or road.rear_controller_grounded

    catch_plane = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "catch_plane")
    for index in range(data.ncon):
        contact = data.contact[index]
        if int(contact.geom1) in query.terrain_ids:
            contact.geom1 = catch_plane
        elif int(contact.geom2) in query.terrain_ids:
            contact.geom2 = catch_plane

    same_time = query.query(model, data)
    assert same_time.front_controller_grounded == road.front_controller_grounded
    assert same_time.rear_controller_grounded == road.rear_controller_grounded
    data.time = 0.006
    catch = query.query(model, data)
    assert not catch.front_snapshot.road_loaded_contact
    assert not catch.rear_snapshot.road_loaded_contact
    assert not catch.front_controller_grounded
    assert not catch.rear_controller_grounded
    assert catch.front_load_n == pytest.approx(road.front_load_n)
    assert catch.rear_load_n == pytest.approx(road.rear_load_n)
    assert catch.front_support_n == pytest.approx(road.front_support_n)
    assert catch.rear_support_n == pytest.approx(road.rear_support_n)

    query.reset()
    data.time = 0.0
    after_reset = query.query(model, data)
    assert not after_reset.front_controller_grounded
    assert not after_reset.rear_controller_grounded


def test_native_controller_gate_requires_more_than_one_newton_of_road_load():
    sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.0)
    model, data = sim.model, sim.data
    query = TerrainContactQuery(model)
    rear_row = next(
        data.contact[index] for index in range(data.ncon)
        if query.rear_id in (int(data.contact[index].geom1), int(data.contact[index].geom2))
        and (int(data.contact[index].geom1) in query.terrain_ids
             or int(data.contact[index].geom2) in query.terrain_ids)
    )
    data.efc_force[:] = 0.0
    data.efc_force[int(rear_row.efc_address)] = 0.5
    below_threshold = query.query(model, data)
    assert below_threshold.rear_snapshot.road_loaded_contact
    assert below_threshold.rear_snapshot.normal_load_n == pytest.approx(0.5)
    assert not below_threshold.rear_controller_grounded

    data.efc_force[int(rear_row.efc_address)] = 1.5
    data.time = 0.0005
    above_threshold = query.query(model, data)
    assert above_threshold.rear_snapshot.normal_load_n == pytest.approx(1.5)
    assert above_threshold.rear_controller_grounded


def test_physical_native_cruise_uses_controller_grounded_instead_of_held_load(monkeypatch):
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(
            physics_mode="physical", drive_mode="ideal_speed_control"
        ),
    )
    held_load = TerrainContacts(
        front_load_n=0.0, rear_load_n=600.0,
        front_support_n=0.0, rear_support_n=0.0,
        handlebar_load_n=0.0, rear_controller_grounded=False,
    )
    assert held_load.rear_in_contact  # Compatibility bridge still reports load.
    monkeypatch.setattr(sim.contact_query, "query", lambda model, data: held_load)
    sim.step()
    assert not sim.cruise.engaged
    assert sim.data.ctrl[sim.drive_ctrl_adr] == 0.0

    held_grounded = TerrainContacts(
        front_load_n=0.0, rear_load_n=0.0,
        front_support_n=0.0, rear_support_n=0.0,
        handlebar_load_n=0.0, rear_controller_grounded=True,
    )
    monkeypatch.setattr(sim.contact_query, "query", lambda model, data: held_grounded)
    sim.step()
    assert sim.cruise.engaged
    assert sim.data.ctrl[sim.drive_ctrl_adr] > 0.0
