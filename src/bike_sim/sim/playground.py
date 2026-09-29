"""
Interactive 2D Suspension Playground Runner for MuJoCo.

Provides a live, interactive 2D suspension test stand environment using the MuJoCo physics
engine with comprehensive RockShox damper and air spring controls, kinematic sweeps,
and real-time telemetry.
"""

import os
import queue
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.damper import (
    BikeSuspensionSystem,
    DamperClickConfig,
    DamperPreset,
)
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.camera import CameraManager
from bike_sim.sim.input_handler import PlaygroundInputHandler
from bike_sim.sim.telemetry import SimModelHandles, TelemetryProvider
from bike_sim.sim.hud import PlaygroundHUDManager


class SuspensionPlayground:
    """
    Manages the interactive MuJoCo bicycle suspension test stand simulation playground.
    """

    def __init__(
        self,
        specs: Optional[BikeSpecs] = None,
        include_rider: bool = False,
    ) -> None:
        self.specs = specs if specs is not None else BikeSpecs()
        self.solver = HorstLinkageSolver(self.specs)
        self.include_rider = include_rider

        # Pneumatic Air Spring Model
        self.air_specs = AirSpringSpecs(total_travel_mm=self.specs.fork_travel)
        self.air_spring = ForkAirSpring(
            specs=self.air_specs,
            num_tokens=self.specs.fork_air_tokens,
            gauge_pressure_psi=self.specs.fork_initial_psi,
        )

        # Hydrodynamic Suspension Damper System (RockShox ZEB Ultimate + Super Deluxe Ultimate)
        damper_cfg = DamperClickConfig(
            fork_hsc=self.specs.fork_hsc,
            fork_lsc=self.specs.fork_lsc,
            fork_rebound=self.specs.fork_rebound,
            shock_hsc=self.specs.shock_hsc,
            shock_lsc=self.specs.shock_lsc,
            shock_rebound=self.specs.shock_rebound,
            shock_hbo=self.specs.shock_hbo,
            shock_lockout=self.specs.shock_lockout,
            preset_name=DamperPreset.BASE.value,
        )
        self.suspension_system = BikeSuspensionSystem(click_config=damper_cfg, legacy_behavior=True)

        # Camera Perspective Manager
        self.camera_manager = CameraManager(default_mode="2d")

        # Telemetry provider & Input handler
        self._telemetry_provider = TelemetryProvider(
            specs=self.specs,
            solver=self.solver,
            air_spring=self.air_spring,
            suspension_system=self.suspension_system,
            camera_manager=self.camera_manager,
        )
        self._input_handler = PlaygroundInputHandler(self)

        # Target control positions for Stand mode
        self.target_rear_travel_mm = 0.0
        self.target_fork_travel_mm = 0.0

        # Auto sweep state (Stand mode)
        self.auto_sweep = False
        self.sweep_freq_hz = 0.4
        self.sweep_start_time = 0.0

        # Telemetry stream toggle
        self.show_telemetry = True

        # Debug livery toggle (pivot marker geoms)
        self.debug_markers = False

        # Passive viewer reference
        self.viewer: Optional[Any] = None

        # Thread-safety lock and key event queue
        self._key_queue: queue.Queue[int] = queue.Queue()
        self._lock = threading.RLock()

        # Cache compiled frame mass properties for rider toggling dynamically
        self._cache_frame_mass_properties()

        # Load active model
        self.load_model()

    def _cache_frame_mass_properties(self) -> None:
        """Pre-computes and caches frame body mass, CoM, and inertia for bike-only and rider-on configurations."""
        # Both variants are also what load_model needs, so keep the XML around:
        # regenerating it there costs a full kinematics-backed rebuild.
        self._xml_bike = generate_mujoco_xml(specs=self.specs, solver=self.solver, mode="playground", include_rider=False, debug_markers=True)
        self._xml_rider = generate_mujoco_xml(specs=self.specs, solver=self.solver, mode="playground", include_rider=True, debug_markers=True)

        m_bike = mujoco.MjModel.from_xml_string(self._xml_bike)
        m_rider = mujoco.MjModel.from_xml_string(self._xml_rider)

        fid_bike = mujoco.mj_name2id(m_bike, mujoco.mjtObj.mjOBJ_BODY, "frame")
        fid_rider = mujoco.mj_name2id(m_rider, mujoco.mjtObj.mjOBJ_BODY, "frame")
        
        self._frame_prop_bike = (
            float(m_bike.body_mass[fid_bike]),
            np.array(m_bike.body_ipos[fid_bike], copy=True),
            np.array(m_bike.body_inertia[fid_bike], copy=True),
        )
        self._frame_prop_rider = (
            float(m_rider.body_mass[fid_rider]),
            np.array(m_rider.body_ipos[fid_rider], copy=True),
            np.array(m_rider.body_inertia[fid_rider], copy=True),
        )

    def load_model(self) -> None:
        """Loads the MuJoCo test stand model in-memory."""
        # Compile a fresh model rather than reusing the cached ones: the loaded
        # model is mutated in place below (markers, rider livery).
        xml_str = self._xml_rider if self.include_rider else self._xml_bike
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # Cache handles for joints, sites, actuators, and markers.
        # Joint/site ids are read through self.handles; do not re-alias them here.
        self.handles = SimModelHandles.from_model(self.model)

        self.act_rear = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_travel_actuator")
        self.act_fork = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fork_travel_actuator")

        # Cache debug marker geom IDs and sizes
        self.marker_geom_ids = []
        self.marker_original_sizes = {}
        for gid in range(self.model.ngeom):
            gname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if gname and gname.startswith("marker_"):
                self.marker_geom_ids.append(gid)
                self.marker_original_sizes[gid] = np.array(self.model.geom_size[gid], copy=True)

        if self.handles.site_pra < 0:
            raise ValueError("model has no site_PRA; rear-axle telemetry cannot be referenced")

        if hasattr(self, "camera_manager") and self.camera_manager is not None:
            self.camera_manager.reset_preset()

        self._apply_debug_markers_in_place()
        self._apply_rider_in_place()
        self.reset_state()

    def _apply_debug_markers_in_place(self) -> None:
        """Toggles pivot marker livery in-place without model recreation."""
        if not hasattr(self, "marker_geom_ids"):
            return
        for gid in self.marker_geom_ids:
            if gid >= 0:
                if self.debug_markers:
                    self.model.geom_rgba[gid, 3] = 1.0
                    if gid in self.marker_original_sizes:
                        self.model.geom_size[gid] = self.marker_original_sizes[gid]
                else:
                    self.model.geom_rgba[gid, 3] = 0.0
                    self.model.geom_size[gid] = [0.0001, 0.0001, 0.0001]

    def _apply_rider_in_place(self) -> None:
        """Applies rider geometry, visibility, and mass properties in-place."""
        fid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "frame")
        if fid < 0:
            return

        gt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rider_torso")
        gl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rider_legs")
        ga = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rider_arms")

        mass, ipos, inertia = self._frame_prop_rider if self.include_rider else self._frame_prop_bike
        self.model.body_mass[fid] = mass
        self.model.body_ipos[fid] = ipos
        self.model.body_inertia[fid] = inertia

        if self.include_rider:
            if gt >= 0:
                self.model.geom_size[gt] = [0.13, 0.18027756, 0.0]
                self.model.geom_rgba[gt] = [0.5, 0.5, 0.5, 1.0]
            if gl >= 0:
                self.model.geom_size[gl] = [0.075, 0.24515301, 0.0]
                self.model.geom_rgba[gl] = [0.5, 0.5, 0.5, 1.0]
            if ga >= 0:
                self.model.geom_size[ga] = [0.055, 0.12971122, 0.0]
                self.model.geom_rgba[ga] = [0.5, 0.5, 0.5, 1.0]
        else:
            for gid in [gt, gl, ga]:
                if gid >= 0:
                    self.model.geom_rgba[gid, 3] = 0.0
                    self.model.geom_size[gid] = [0.0001, 0.0001, 0.0001]
                    self.model.geom_contype[gid] = 0
                    self.model.geom_conaffinity[gid] = 0

        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def toggle_rider(self) -> bool:
        """Toggles rider presence in-place."""
        self.include_rider = not self.include_rider
        if hasattr(self, "model") and hasattr(self, "data"):
            self._apply_rider_in_place()

        state_str = "ON (104.35 kg full system)" if self.include_rider else "OFF (24.35 kg bike only)"
        print(f"\n[KEY B] Rider Toggle -> {state_str}")
        return self.include_rider

    def toggle_debug_markers(self) -> bool:
        """Toggles the debug pivot-marker livery in-place."""
        self.debug_markers = not self.debug_markers
        self._apply_debug_markers_in_place()
        mujoco.mj_forward(self.model, self.data)
        print(f"\n[KEY G] Debug pivot markers: {'ON' if self.debug_markers else 'OFF'}")
        return self.debug_markers

    def reset_state(self) -> None:
        """Resets the simulation to initial uncompressed equilibrium on the test stand."""
        mujoco.mj_resetData(self.model, self.data)
        self.target_rear_travel_mm = 0.0
        self.target_fork_travel_mm = 0.0
        self.auto_sweep = False
        self.apply_stand_targets()
        mujoco.mj_forward(self.model, self.data)
        print("\n[RESET] Simulation reset to initial uncompressed test stand state.")

    def apply_stand_targets(self) -> None:
        """Applies actuator control values for Test Stand mode."""
        if self.act_fork >= 0:
            target_m = np.clip(self.target_fork_travel_mm / 1000.0, 0.0, 0.180)
            self.data.ctrl[self.act_fork] = target_m

        if self.act_rear >= 0:
            st = self.solver.solve_state_from_wheel_travel(self.target_rear_travel_mm)
            theta_cs_rad = float(st["theta_cs"])
            theta_cs_0 = float(self.solver.theta_cs_0)
            q_pivot = -(theta_cs_rad - theta_cs_0)
            self.data.ctrl[self.act_rear] = q_pivot

    def step(self) -> None:
        """Executes a single simulation step."""
        with self._lock:
            self.process_pending_keys()
            if self.auto_sweep:
                t = time.time() - self.sweep_start_time
                sweep_phase = (np.sin(2.0 * np.pi * self.sweep_freq_hz * t - np.pi / 2.0) + 1.0) / 2.0
                self.target_rear_travel_mm = float(sweep_phase * 180.0)
                self.target_fork_travel_mm = float(sweep_phase * 180.0)
            self.apply_stand_targets()
            mujoco.mj_step(self.model, self.data)

    def get_telemetry(self) -> Dict[str, Any]:
        """Computes live suspension telemetry and kinematics metrics."""
        with self._lock:
            return self._telemetry_provider.compute_telemetry(
                model=self.model,
                data=self.data,
                target_rear_travel_mm=self.target_rear_travel_mm,
                include_rider=self.include_rider,
                handles=self.handles,
            )

    def print_hud(self) -> None:
        """Prints a single-line live telemetry HUD to terminal."""
        tel = self.get_telemetry()
        PlaygroundHUDManager.print_hud(tel, auto_sweep=self.auto_sweep)

    def handle_key(self, keycode: int) -> None:
        """Enqueues keyboard event from GLFW viewer thread for thread-safe processing."""
        self._key_queue.put(keycode)
        if self.viewer is None:
            self.process_pending_keys()

    def process_pending_keys(self) -> None:
        """Processes all queued keyboard events safely on the physics simulation thread."""
        with self._lock:
            while not self._key_queue.empty():
                try:
                    keycode = self._key_queue.get_nowait()
                    self._handle_key_internal(keycode)
                except queue.Empty:
                    break

    def _handle_key_internal(self, keycode: int) -> None:
        """Handles keyboard events from MuJoCo GLFW viewer safely on the physics thread."""
        self._input_handler.handle_key(keycode)

    def print_help(self) -> None:
        """Displays interactive control help in terminal."""
        PlaygroundHUDManager.print_help()


def ensure_macos_mjpython() -> None:
    """
    On macOS, MuJoCo's GLFW interactive viewer requires running under `mjpython`
    on the main OS thread to handle the Cocoa NSApplication event loop.
    """
    if sys.platform != "darwin":
        return
    if getattr(sys, "_mjpython_checked", False):
        return
    sys._mjpython_checked = True

    is_mjpython = "mjpython" in sys.executable or os.environ.get("MUJOCO_MJPYTHON") == "1"
    if not is_mjpython:
        import shutil
        mjpython_path = shutil.which("mjpython")
        if mjpython_path:
            # mjpython dlopens the interpreter by @rpath; venvs (uv, pyenv) keep
            # libpython in base_prefix/lib which is not on the default fallback
            # path, so the re-exec dies with 'Library not loaded' otherwise.
            lib_dir = os.path.join(sys.base_prefix, "lib")
            version = f"{sys.version_info.major}.{sys.version_info.minor}"
            if os.path.exists(os.path.join(lib_dir, f"libpython{version}.dylib")):
                fallback = os.environ.get("DYLD_FALLBACK_LIBRARY_PATH")
                os.environ["DYLD_FALLBACK_LIBRARY_PATH"] = (
                    lib_dir if not fallback else f"{lib_dir}:{fallback}")
            cmd = [mjpython_path] + sys.argv
            print(f"[MuJoCo Stand] Re-launching under mjpython: {' '.join(cmd)}", flush=True)
            os.environ["MUJOCO_MJPYTHON"] = "1"
            os.execv(mjpython_path, cmd)


def run_interactive_playground(
    specs: Optional[BikeSpecs] = None,
    include_rider: bool = False,
) -> None:
    """
    Launches the live interactive MuJoCo suspension test stand viewer.
    """
    import mujoco.viewer

    ensure_macos_mjpython()

    pg = SuspensionPlayground(
        specs=specs,
        include_rider=include_rider,
    )

    print("\n" + "=" * 80)
    print("  ROCKSHOX TEST STAND - MUJOCO 2D SUSPENSION PLAYGROUND")
    print("=" * 80)
    pg.print_help()

    with mujoco.viewer.launch_passive(
        pg.model,
        pg.data,
        key_callback=pg.handle_key,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        pg.viewer = viewer
        # Request an instant preset snap on the first update_viewer call.
        pg.camera_manager.reset_preset()

        PHYSICS_DT = 0.002
        HUD_REFRESH_INTERVAL = 0.08
        last_step_time = time.time()
        last_hud_time = time.time()

        while viewer.is_running():
            now = time.time()
            elapsed = now - last_step_time

            if elapsed >= PHYSICS_DT:
                pg.step()
                last_step_time = now

                bb_pos = pg.data.site_xpos[pg.handles.site_bb] if pg.handles.site_bb >= 0 else (0.0, 0.0, 0.0)
                pg.camera_manager.update_viewer(viewer, bike_x=float(bb_pos[0]), bike_z=float(bb_pos[2]))
                viewer.sync()

                if pg.show_telemetry and (now - last_hud_time >= HUD_REFRESH_INTERVAL):
                    pg.print_hud()
                    last_hud_time = now

            time.sleep(0.0005)
