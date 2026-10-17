"""
Recorded-Episode Playback Viewer.

Scrubs a saved episode's recorded trajectory through the passive MuJoCo viewer
instead of re-simulating it: each rendered frame writes the recorded `qpos` row
(interpolated by `Playback.pose_at`) and the nearest `qvel` row back into
`mjData`, then `mj_forward` recomputes the derived state, so the window shows
exactly what the episode contained.

**The playback cursor lives in a small shared namespace.** MuJoCo's passive
viewer calls the key callback from its own thread and hands it only a GLFW
keycode, so the callback mutates `_PlaybackState` and the render loop applies it
on its next frame -- the same queue-free division the ride viewer uses for
camera state. Seeking and rescaling speed are cursor edits, not physics writes,
so no locking beyond the viewer's own `lock()` around `mjData` is needed.

**Shift is polled, not delivered.** The callback signature carries no modifier
bits, so a fine seek asks GLFW directly whether a Shift key is held while the
event is being dispatched on the UI thread. When the context cannot be polled
the coarse step applies.
"""

import sys
import time
from typing import TYPE_CHECKING

import mujoco
import numpy as np

from bike_sim.sim.camera import CameraManager

if TYPE_CHECKING:
    # Import-time only: mujoco.viewer pulls in glfw, which every headless consumer of this
    # package would otherwise pay for.
    import mujoco.viewer

    from bike_sim.sim.research.playback import Playback

# Viewer refresh pacing, matching the ride viewer's idle loop.
FRAME_SLEEP_S = 0.001

# GLFW keycodes, the same code points `sim/ride/input.py` binds: letters in both
# cases, arrows and escape by their GLFW values.
_KEY_SPACE = 32
_KEY_ESC = 256
_KEY_LEFT = 263
_KEY_RIGHT = 262
_KEY_DOWN = 264
_KEY_UP = 265
_KEYS_R = (82, 114)
_KEYS_Q = (81, 113)

# Seek and speed steps the arrow keys take.
SEEK_STEP_S = 1.0
SEEK_STEP_FINE_S = 0.1
SPEED_FACTOR = 1.5

HELP_LINE = (
    "[bike-replay] keys: SPACE pause/resume; LEFT/RIGHT seek 1 s (Shift: 0.1 s); "
    "UP/DOWN playback speed; R restart; Q/ESC quit. See `bike-replay --help`."
)


class _PlaybackState:
    """Mutable playback cursor shared between the GLFW key callback and the render loop."""

    __slots__ = ("t_s", "speed", "paused", "stop_requested")

    def __init__(self, speed: float) -> None:
        self.t_s = 0.0
        self.speed = float(speed)
        self.paused = False
        self.stop_requested = False


def _shift_held() -> bool:
    """Polls whether a Shift key is down during a key event; False when GLFW cannot answer."""
    try:
        import glfw

        window = glfw.get_current_context()
        if window is None:
            return False
        return bool(
            glfw.get_key(window, glfw.KEY_LEFT_SHIFT)
            or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT)
        )
    except Exception:
        return False


def run_playback_viewer(
    playback: "Playback",
    *,
    speed: float = 1.0,
    loop: bool = False,
    fps: float = 60.0,
) -> int:
    """
    Plays a recorded episode in the passive MuJoCo viewer.

    Args:
        playback: Loaded episode playback (`bike_sim.sim.research.playback.Playback`).
        speed: Initial playback rate multiplier; `UP`/`DOWN` scale it by 1.5.
        loop: Wrap the cursor at the end instead of holding the last frame.
        fps: Viewer sync rate; the loop renders at most this many frames a second.

    Returns:
        0 once the window is closed normally.

    Raises:
        ValueError: If `fps` is not positive.
    """
    if fps <= 0.0:
        raise ValueError(f"fps must be positive, got {fps}")

    from bike_sim.viewer_runtime import load_viewer, passive_viewer
    # The caller has already rebuilt the recording. CLI preparation happens
    # before that rebuild; a direct caller must prepare its own launch first.
    viewer_api = load_viewer()

    sim = playback.sim
    model, data = sim.model, sim.data
    duration_s = float(playback.duration_s)
    times_s = playback.times_s

    camera = CameraManager(default_mode="2d")
    state = _PlaybackState(speed)

    def handle_key(keycode: int) -> None:
        if keycode == _KEY_SPACE:
            state.paused = not state.paused
        elif keycode == _KEY_LEFT:
            state.t_s -= SEEK_STEP_FINE_S if _shift_held() else SEEK_STEP_S
        elif keycode == _KEY_RIGHT:
            state.t_s += SEEK_STEP_FINE_S if _shift_held() else SEEK_STEP_S
        elif keycode == _KEY_UP:
            state.speed *= SPEED_FACTOR
        elif keycode == _KEY_DOWN:
            state.speed /= SPEED_FACTOR
        elif keycode in _KEYS_R:
            state.t_s = 0.0
        elif keycode in _KEYS_Q or keycode == _KEY_ESC:
            # The callback may arrive before launch_passive returns a handle.
            # Let the owning loop/context close it once on the Python thread.
            state.stop_requested = True

    print(HELP_LINE, file=sys.stderr)

    sync_interval_s = 1.0 / float(fps)
    with passive_viewer(viewer_api,
        model,
        data,
        key_callback=handle_key,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        camera.reset_preset()
        last_frame = time.monotonic()
        last_sync = last_frame

        while viewer.is_running() and not state.stop_requested:
            now = time.monotonic()
            elapsed, last_frame = now - last_frame, now

            if not state.paused:
                state.t_s += elapsed * state.speed
            if duration_s > 0.0:
                if loop:
                    state.t_s %= duration_s
                else:
                    state.t_s = min(max(state.t_s, 0.0), duration_s)
            else:
                state.t_s = 0.0

            if now - last_sync >= sync_interval_s:
                with viewer.lock():
                    data.qpos[:] = playback.pose_at(state.t_s)
                    # Rendering context only: the recorded row nearest the cursor is
                    # close enough -- no interpolation needed for qvel.
                    key = int(np.clip(np.searchsorted(times_s, state.t_s), 0, len(times_s) - 1))
                    data.qvel[:] = playback.qvel[key]
                    mujoco.mj_forward(model, data)
                    camera.update_viewer(
                        viewer,
                        bike_x=sim.position_m,
                        bike_z=float(data.xpos[sim._frame_body_id, 2]),
                    )
                viewer.sync()
                last_sync = now

            time.sleep(FRAME_SLEEP_S)

    return 0


def run_playback_dry_run(playback: "Playback", *, fps: float = 60.0) -> int:
    """
    Sweeps the recorded trajectory headlessly: `pose_at` plus `mj_forward` for
    every frame in `[0, duration]` at `fps` granularity, then one summary line.

    Args:
        playback: Loaded episode playback (`bike_sim.sim.research.playback.Playback`).
        fps: Frame granularity of the sweep.

    Returns:
        0 after the summary line is printed.

    Raises:
        ValueError: If `fps` is not positive.
    """
    if fps <= 0.0:
        raise ValueError(f"fps must be positive, got {fps}")

    sim = playback.sim
    duration_s = float(playback.duration_s)
    frames = int(duration_s * float(fps)) + 1
    for index in range(frames):
        sim.data.qpos[:] = playback.pose_at(index / float(fps))
        mujoco.mj_forward(sim.model, sim.data)
    print(f"frames={frames} duration={duration_s:.3f}s ok")
    return 0


__all__ = [
    "run_playback_viewer",
    "run_playback_dry_run",
    "HELP_LINE",
    "SEEK_STEP_S",
    "SEEK_STEP_FINE_S",
    "SPEED_FACTOR",
    "FRAME_SLEEP_S",
]
