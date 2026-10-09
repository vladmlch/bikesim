"""Presentation pacing in completed physics steps, never a physics timestep change."""
from __future__ import annotations

import math
from numbers import Integral, Real

SCALES = (1, 2, 4, 8)
COMPUTE_SLICE_S = .008
RENDER_INTERVAL_S = 1. / 60.


def _real(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite real number')
    return float(value)


def _step(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError('step must be a nonnegative integer')
    return int(value)


def _scale(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value not in SCALES:
        raise ValueError('scale must be one of 1, 2, 4, 8')
    return int(value)


class PlaybackClock:
    """Retain unfinished pacing debt when an advancement yields to the UI.

    Rebase after a generation change. A scale/pause change discards old debt;
    it must not apply a new scale retroactively to elapsed wall time.
    """

    def __init__(self, timestep_s: float, *, scale: int = 1):
        self.timestep_s = _real(timestep_s, 'timestep_s')
        if self.timestep_s <= 0.:
            raise ValueError('timestep_s must be positive')
        self.scale = _scale(scale)
        self.paused = False
        self._wall = None
        self._step = 0
        self._debt = 0.

    def rebase(self, now: float, *, step: int) -> None:
        now, step = _real(now, 'now'), _step(step)
        self._wall, self._step, self._debt = now, step, 0.

    def target_step(self, now: float, *, current_step: int) -> int:
        now, current_step = _real(now, 'now'), _step(current_step)
        if self._wall is None:
            self.rebase(now, step=current_step)
        if current_step < self._step:
            raise ValueError('completed step moved backwards; rebase after reset')
        completed = current_step - self._step
        debt = max(0., self._debt - completed * self.timestep_s)
        elapsed = max(0., now - self._wall)
        if self.paused:
            debt = 0.
        else:
            # At least one step also permits deliberately slow synthetic rigs.
            cap = max(.05 * self.scale, self.timestep_s)
            debt = min(debt + elapsed * self.scale, cap)
        self._debt, self._step = debt, current_step
        # A regressing wall clock must not count the same time twice later.
        self._wall = max(self._wall, now)
        return current_step + math.floor(debt / self.timestep_s)

    def set_scale(self, scale: int, *, now: float, step: int) -> None:
        scale = _scale(scale)
        now, step = _real(now, 'now'), _step(step)
        self.scale = scale
        self.rebase(now, step=step)

    def set_paused(self, paused: bool, *, now: float, step: int) -> None:
        if type(paused) is not bool:
            raise ValueError('paused must be bool')
        now, step = _real(now, 'now'), _step(step)
        if self.paused != paused:
            self.paused = paused
            self.rebase(now, step=step)


def speed_key(keycode: int, current_scale: int) -> int | None:
    index = SCALES.index(_scale(current_scale))
    if keycode == 295:  # GLFW_KEY_F6
        return SCALES[max(0, index - 1)]
    if keycode == 296:  # GLFW_KEY_F7
        return SCALES[min(len(SCALES) - 1, index + 1)]
    if keycode == 297:  # GLFW_KEY_F8
        return 1
    return None
