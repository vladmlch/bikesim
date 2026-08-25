"""
Unit tests for the 2D MuJoCo Suspension Test Stand Playground.

Tests include:
- MJCF model generation and loading validation for standard and stand modes.
- Kinematic loop closure equality constraint tightness (< 0.5 mm residual).
- Full suspension travel range tracking (0 to 180 mm).
- Telemetry computations and consistency with analytical HorstLinkageSolver.
- Key controls, damper clicks, fork air spring tuning, and rider toggle logic.
- Camera view switching and low-pass filter smoothing.
"""

import pytest
import numpy as np
import mujoco

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import export_playground_models
from bike_sim.sim.playground import SuspensionPlayground


@pytest.fixture(scope="module")
def playground_models(tmp_path_factory):
    """Generates all playground models in a temporary directory."""
    temp_dir = tmp_path_factory.mktemp("models")
    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)
    models = export_playground_models(output_dir=temp_dir, specs=specs, solver=solver)
    return {
        "dir": temp_dir,
        "stand": str(temp_dir / "bike_playground_stand.xml"),
        "standard": str(temp_dir / "bike_model.xml"),
        "playground": str(temp_dir / "bike_playground.xml"),
        "specs": specs,
        "solver": solver,
    }


def test_mjcf_compilation_all_modes(playground_models):
    """Verifies that all generated MJCF XML models compile cleanly in MuJoCo."""
    for mode in ["stand", "standard", "playground"]:
        xml_path = playground_models[mode]
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        assert model.nbody >= 11, f"Expected >= 11 bodies in {mode} model"
        assert model.neq >= 2, f"Expected >= 2 equality constraints in {mode} model"
        assert model.njnt >= 6, f"Expected >= 6 joints in {mode} model"


def test_stand_mode_actuators_and_travel_sweep(playground_models):
    """Verifies that position servos in Stand mode can sweep travel 0..180mm with tight closure."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    for travel_target in [0.0, 45.0, 90.0, 135.0, 180.0]:
        pg.target_rear_travel_mm = travel_target
        pg.target_fork_travel_mm = travel_target

        for _ in range(400):
            pg.step()

        tel = pg.get_telemetry()

        # Check rear travel tracking within 8 mm (accounts for gravity sag settling)
        assert abs(tel["rear_travel_mm"] - travel_target) < 8.0, (
            f"Rear travel error too large at target {travel_target}: actual {tel['rear_travel_mm']:.2f}"
        )

        # Check loop closure tightness at P3 (< 0.5 mm)
        assert tel["p3_residual_mm"] < 0.5, (
            f"Loop closure residual at P3 too large: {tel['p3_residual_mm']:.4f} mm"
        )

        # Check shock stroke progressivity and bounds (0..65 mm)
        assert 0.0 <= tel["shock_stroke_mm"] <= 65.5, (
            f"Shock stroke out of bounds: {tel['shock_stroke_mm']:.2f} mm"
        )


def test_telemetry_metrics_and_forces(playground_models):
    """Verifies that telemetry calculations return physically consistent metrics."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    # Initial state
    tel0 = pg.get_telemetry()
    assert tel0["rear_travel_mm"] < 2.0
    assert tel0["fork_travel_mm"] < 1.0
    assert tel0["shock_stroke_mm"] < 1.5

    solver: HorstLinkageSolver = playground_models["solver"]
    lr_expected = solver.solve_state_from_wheel_travel(tel0["rear_travel_mm"])["leverage_ratio"]
    assert 3.3328 <= lr_expected <= 3.3495, (
        f"Solver leverage ratio {lr_expected:.6f} at {tel0['rear_travel_mm']:.3f} mm is "
        f"outside the photo-fitted top-out band (docs/reference/fitted_hardpoints.json)"
    )
    assert tel0["leverage_ratio"] == pytest.approx(lr_expected, rel=1e-6)

    # Compress rear wheel to 90 mm
    pg.target_rear_travel_mm = 90.0
    for _ in range(400):
        pg.step()

    tel90 = pg.get_telemetry()
    assert abs(tel90["rear_travel_mm"] - 90.0) < 3.0
    assert tel90["shock_stroke_mm"] > 25.0
    assert tel90["shock_force_n"] > 1500.0  # Spring force should increase


def test_key_handling(playground_models):
    """Verifies that key press events update target travel, auto-sweep, and damper clicks."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    # Key W (lift rear)
    pg.handle_key(87)
    assert pg.target_rear_travel_mm == 5.0

    # Key S (lower rear)
    pg.handle_key(83)
    assert pg.target_rear_travel_mm == 0.0

    # Key Up (compress fork)
    pg.handle_key(265)
    assert pg.target_fork_travel_mm == 5.0

    # Key Down (extend fork)
    pg.handle_key(264)
    assert pg.target_fork_travel_mm == 0.0

    # Space (toggle auto sweep)
    assert not pg.auto_sweep
    pg.handle_key(32)
    assert pg.auto_sweep
    pg.handle_key(32)
    assert not pg.auto_sweep


def test_stand_mode_survives_large_step_input(playground_models):
    """Verifies that large step changes in target do not destabilise the stand rig."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    for target in (45.0, 90.0, 120.0, 150.0, 180.0):
        pg.reset_state()
        pg.target_rear_travel_mm = target
        pg.target_fork_travel_mm = target

        prev_time = pg.data.time
        for i in range(400):
            pg.step()
            assert pg.data.time > prev_time, (
                f"Stand model diverged on a 0 -> {target:.0f} mm step and MuJoCo "
                f"auto-reset it at step {i} (t={pg.data.time:.4f} after t={prev_time:.4f})"
            )
            prev_time = pg.data.time

        tel = pg.get_telemetry()
        assert abs(tel["rear_travel_mm"] - target) < 8.0, (
            f"Rear travel failed to track a 0 -> {target:.0f} mm step: "
            f"{tel['rear_travel_mm']:.2f} mm"
        )
        assert tel["p3_residual_mm"] < 0.5, (
            f"Loop closure lost on a 0 -> {target:.0f} mm step: {tel['p3_residual_mm']:.4f} mm"
        )


def test_reset_simulation(playground_models):
    """Verifies that reset restores uncompressed state."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    pg.target_rear_travel_mm = 120.0
    pg.target_fork_travel_mm = 120.0
    prev_time = pg.data.time
    for i in range(200):
        pg.step()
        assert pg.data.time > prev_time, f"MuJoCo auto-reset the model at step {i}"
        prev_time = pg.data.time

    assert pg.get_telemetry()["rear_travel_mm"] > 100.0

    pg.reset_state()
    tel = pg.get_telemetry()
    assert pg.target_rear_travel_mm == 0.0
    assert pg.target_fork_travel_mm == 0.0
    assert tel["rear_travel_mm"] < 5.0


def test_default_stand_mode(playground_models):
    """Verifies that default playground mode is the interactive test stand with fixed frame."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    # Stand starts at zero target travel; auto-sweep is off until [Space]
    assert pg.target_rear_travel_mm == 0.0
    assert pg.target_fork_travel_mm == 0.0
    assert not pg.auto_sweep

    # Telemetry reports a fixed frame: no forward speed, no track position
    tel0 = pg.get_telemetry()
    assert tel0["speed_mps"] == 0.0
    assert tel0["speed_kmh"] == 0.0
    assert tel0["pos_x_m"] == 0.0
    assert "camera_mode" in tel0
    assert "rear_travel_mm" in tel0
    assert "fork_travel_mm" in tel0

    # Initial suspension travel is essentially zero (sag-only settle)
    assert tel0["rear_travel_mm"] < 5.0
    assert tel0["fork_travel_mm"] < 5.0

    # Step the simulation for 1 s: frame must stay fixed and travel must stay at zero
    for _ in range(500):
        pg.step()

    tel1 = pg.get_telemetry()
    assert tel1["pos_x_m"] == 0.0, f"Stand frame should not translate, got pos_x_m={tel1['pos_x_m']:.3f}"
    assert tel1["speed_mps"] == 0.0
    assert tel1["rear_travel_mm"] < 5.0
    assert tel1["fork_travel_mm"] < 5.0


def test_camera_manager_and_view_switching(playground_models):
    """Verifies camera mode switching, presets, and key handling."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    # Default camera is 2D Side View
    assert pg.camera_manager.active_mode == "2d"
    assert pg.get_telemetry()["camera_mode_id"] == "2d"

    # Switch to 3D via Key 2
    pg.handle_key(50)
    assert pg.camera_manager.active_mode == "3d"
    assert pg.get_telemetry()["camera_mode_id"] == "3d"

    # Switch to 2D via Key 1
    pg.handle_key(49)
    assert pg.camera_manager.active_mode == "2d"
    assert pg.get_telemetry()["camera_mode_id"] == "2d"

    # Cycle camera via Key C (67)
    pg.handle_key(67)
    assert pg.camera_manager.active_mode == "3d"
    pg.handle_key(99)  # lowercase 'c'
    assert pg.camera_manager.active_mode == "2d"


def test_camera_smoothing_and_reset(playground_models):
    """Verifies that CameraManager applies low-pass smoothing on Z and snaps instantly on reset."""
    from bike_sim.sim.camera import CameraManager

    cam = CameraManager(default_mode="2d", smoothing_alpha=0.1)

    class DummyCam:
        lookat = [0.0, 0.0, 0.0]
        azimuth = 0.0
        elevation = 0.0
        distance = 0.0

    class DummyViewer:
        cam = DummyCam()

    viewer = DummyViewer()

    # Frame 1: initial snap
    cam.update_viewer(viewer, bike_x=0.0, bike_z=0.0)
    assert np.isclose(viewer.cam.lookat[0], 0.15)
    assert np.isclose(viewer.cam.lookat[1], 0.0)
    assert np.isclose(viewer.cam.lookat[2], 0.25)

    # Frame 2: sudden jump in bike_z (e.g. 50mm bump)
    cam.update_viewer(viewer, bike_x=0.06, bike_z=0.05)
    # X follows directly with frame sync (0.06 + 0.15 = 0.21)
    assert np.isclose(viewer.cam.lookat[0], 0.21)
    # Z is smoothed: 0.25 + 0.1 * (0.30 - 0.25) = 0.255
    assert np.isclose(viewer.cam.lookat[2], 0.255)

    # Reset preset snaps immediately without lag
    cam.reset_preset()
    cam.update_viewer(viewer, bike_x=1.0, bike_z=0.05)
    assert np.isclose(viewer.cam.lookat[0], 1.15)
    assert np.isclose(viewer.cam.lookat[2], 0.30)


def test_zero_baseline_stiffness_in_xml():
    """Verifies that fork_travel and shock_stroke slide joints have 0 stiffness and damping in XML."""
    for mode in ["standard", "stand", "playground"]:
        xml_str = generate_mujoco_xml(mode=mode)
        model = mujoco.MjModel.from_xml_string(xml_str)

        jnt_fork = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fork_travel")
        jnt_shock = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")

        assert jnt_fork >= 0 and jnt_shock >= 0
        assert model.jnt_stiffness[jnt_fork] == 0.0
        assert model.dof_damping[model.jnt_dofadr[jnt_fork]] == 0.0
        assert model.jnt_stiffness[jnt_shock] == 0.0
        assert model.dof_damping[model.jnt_dofadr[jnt_shock]] == 0.0


def test_direct_in_memory_xml_generation():
    """Verifies that SuspensionPlayground generates and compiles models in memory dynamically."""
    custom_specs = BikeSpecs(fork_travel=160.0, shock_stroke=60.0)
    pg = SuspensionPlayground(specs=custom_specs)

    jnt_fork = mujoco.mj_name2id(pg.model, mujoco.mjtObj.mjOBJ_JOINT, "fork_travel")
    fork_range_max = pg.model.jnt_range[jnt_fork][1]
    assert np.isclose(fork_range_max, 0.160)


def test_rider_toggle_and_mass_telemetry():
    """Verifies that toggling the rider updates model mass, inertia, and telemetry between ~104.5 kg and ~24.5 kg."""
    pg = SuspensionPlayground(include_rider=False)
    tel_off = pg.get_telemetry()
    assert tel_off["rider_active"] is False
    assert 20.0 <= tel_off["total_mass_kg"] <= 28.0

    # Toggle rider on
    res = pg.toggle_rider()
    assert res is True
    tel_on = pg.get_telemetry()
    assert tel_on["rider_active"] is True
    assert 100.0 <= tel_on["total_mass_kg"] <= 110.0

    # Toggle rider back off
    res2 = pg.toggle_rider()
    assert res2 is False
    tel_off2 = pg.get_telemetry()
    assert tel_off2["rider_active"] is False
    assert 20.0 <= tel_off2["total_mass_kg"] <= 28.0


def test_rider_toggle_matches_freshly_compiled_frame_mass_properties():
    """Verifies that _apply_rider_in_place() matches fresh compiles of include_rider=True/False."""
    fresh_on = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="playground", include_rider=True))
    fresh_off = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="playground", include_rider=False))
    fid_fresh = mujoco.mj_name2id(fresh_on, mujoco.mjtObj.mjOBJ_BODY, "frame")

    pg = SuspensionPlayground(include_rider=False)
    fid = mujoco.mj_name2id(pg.model, mujoco.mjtObj.mjOBJ_BODY, "frame")

    # Starts OFF: matches a fresh include_rider=False compile.
    assert pg.model.body_mass[fid] == pytest.approx(fresh_off.body_mass[fid_fresh], abs=1e-6)
    assert np.allclose(pg.model.body_ipos[fid], fresh_off.body_ipos[fid_fresh], atol=1e-6)
    assert np.allclose(pg.model.body_inertia[fid], fresh_off.body_inertia[fid_fresh], atol=1e-6)

    # Toggle ON: matches fresh include_rider=True compile.
    pg.toggle_rider()
    assert pg.model.body_mass[fid] == pytest.approx(fresh_on.body_mass[fid_fresh], abs=1e-6)
    assert np.allclose(pg.model.body_ipos[fid], fresh_on.body_ipos[fid_fresh], atol=1e-6)
    assert np.allclose(pg.model.body_inertia[fid], fresh_on.body_inertia[fid_fresh], atol=1e-6)

    # Toggle back OFF: matches fresh include_rider=False compile again.
    pg.toggle_rider()
    assert pg.model.body_mass[fid] == pytest.approx(fresh_off.body_mass[fid_fresh], abs=1e-6)
    assert np.allclose(pg.model.body_ipos[fid], fresh_off.body_ipos[fid_fresh], atol=1e-6)
    assert np.allclose(pg.model.body_inertia[fid], fresh_off.body_inertia[fid_fresh], atol=1e-6)


def test_debug_marker_key_toggles_livery_without_touching_mass(playground_models):
    """Verifies [G] adds and removes the debug pivot markers and changes nothing physical."""
    pg = SuspensionPlayground(
        specs=playground_models["specs"],
    )

    assert not pg.debug_markers
    assert len(pg.marker_geom_ids) > 0
    assert all(pg.model.geom_rgba[gid, 3] == 0.0 for gid in pg.marker_geom_ids)
    mass_off = float(sum(pg.model.body_mass))

    pg.handle_key(71)  # G
    assert pg.debug_markers
    assert all(pg.model.geom_rgba[gid, 3] == 1.0 for gid in pg.marker_geom_ids)
    mass_on = float(sum(pg.model.body_mass))
    assert mass_on == pytest.approx(mass_off, abs=1e-9), (
        f"debug livery changed model mass by {mass_on - mass_off:.6e} kg - markers must be massless"
    )

    pg.handle_key(103)  # g
    assert not pg.debug_markers
    assert all(pg.model.geom_rgba[gid, 3] == 0.0 for gid in pg.marker_geom_ids)
    assert float(sum(pg.model.body_mass)) == pytest.approx(mass_off, abs=1e-9)
