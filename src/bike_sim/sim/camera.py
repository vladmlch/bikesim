"""
Interactive MuJoCo Camera Perspective Manager.

Manages 2D Side Tracking and 3D Isometric Chase perspectives with instant snapping,
low-pass Z smoothing, and free mouse orbiting.
"""

from typing import TYPE_CHECKING, Any, Dict, Optional
import numpy as np

if TYPE_CHECKING:
    # Import-time only: mujoco.viewer pulls in glfw (~80 ms), which every
    # geometry/kinematics/export consumer would otherwise pay for.
    import mujoco.viewer


class CameraManager:
    """
    Manages interactive camera perspectives (2D Side Tracking & 3D Isometric Chase)
    with instant preset snapping, low-pass Z smoothing, and hybrid free mouse orbiting.
    """

    PRESETS: Dict[str, Dict[str, Any]] = {
        "2d": {
            "name": "2D Side Tracking",
            "azimuth": 90.0,
            "elevation": -5.0,
            "distance": 2.5,
            "lookat_offset": np.array([0.15, 0.0, 0.25]),
        },
        "3d": {
            "name": "3D Isometric Chase",
            "azimuth": 135.0,
            "elevation": -15.0,
            "distance": 3.2,
            "lookat_offset": np.array([0.20, 0.0, 0.30]),
        },
    }

    def __init__(self, default_mode: str = "2d", smoothing_alpha: float = 0.08) -> None:
        self.active_mode = default_mode if default_mode in self.PRESETS else "2d"
        self.needs_preset_reset = True
        self.smoothing_alpha = smoothing_alpha
        self.smoothed_lookat: Optional[np.ndarray] = None

    def set_mode(self, mode: str) -> str:
        """Sets the active camera mode and requests instant preset orientation snap."""
        if mode in self.PRESETS:
            self.active_mode = mode
            self.needs_preset_reset = True
        return self.active_mode

    def cycle_mode(self) -> str:
        """Cycles between available camera presets."""
        modes = list(self.PRESETS.keys())
        idx = (modes.index(self.active_mode) + 1) % len(modes)
        return self.set_mode(modes[idx])

    def reset_preset(self) -> None:
        """Forces the active camera orientation and distance to snap to preset defaults."""
        self.needs_preset_reset = True
        self.smoothed_lookat = None

    @property
    def mode_name(self) -> str:
        return self.PRESETS[self.active_mode]["name"]

    def update_viewer(self, viewer: "mujoco.viewer.Handle", bike_x: float, bike_z: float) -> None:
        """
        Updates camera center-of-interest (lookat) to smoothly track the bike,
        while applying instant preset snaps upon mode changes or allowing
        free interactive mouse orbiting.
        """
        preset = self.PRESETS[self.active_mode]
        offset = preset["lookat_offset"]

        target_lookat = np.array([
            bike_x + offset[0],
            offset[1],
            bike_z + offset[2],
        ])

        # Snap instantly upon mode reset or first frame
        if self.needs_preset_reset or self.smoothed_lookat is None:
            self.smoothed_lookat = target_lookat.copy()
            viewer.cam.azimuth = preset["azimuth"]
            viewer.cam.elevation = preset["elevation"]
            viewer.cam.distance = preset["distance"]
            self.needs_preset_reset = False
        else:
            # X tracks forward directly with sync; Z uses low-pass EMA filter
            alpha_z = self.smoothing_alpha
            self.smoothed_lookat[0] = target_lookat[0]
            self.smoothed_lookat[1] = target_lookat[1]
            self.smoothed_lookat[2] += alpha_z * (target_lookat[2] - self.smoothed_lookat[2])

        viewer.cam.lookat[0] = self.smoothed_lookat[0]
        viewer.cam.lookat[1] = self.smoothed_lookat[1]
        viewer.cam.lookat[2] = self.smoothed_lookat[2]
