"""Common camera/HUD on a render-only model; never reads authoritative mjData."""
from __future__ import annotations

from bike_sim.sim.camera import CameraManager
from bike_sim.sim.ride.hud import RideHUD
from bike_sim.sim.ride.livery import ModelLivery
from bike_sim.sim.ride.physical_view import RenderReplica


class FramePresenter:
    def __init__(self, model):
        self.replica = RenderReplica(model)
        self.camera = CameraManager(default_mode='2d')
        self.hud = RideHUD(model)
        # Livery's marker operation needs only the model. Geometry construction
        # and suspension tuning never belong to the presentation replica.
        self.livery = ModelLivery(model, None, None)
        self.markers = False
        self.livery.set_markers(False)
        self.telemetry = True
        self._legend_generation = None
        self._failure = None

    def handle_key(self, key, viewer):
        from bike_sim.sim.ride.console import info
        if key in (ord('C'), ord('c')):
            self.camera.cycle_mode()
        elif key in (ord('1'), 321):
            self.camera.set_mode('2d')
        elif key in (ord('2'), 322):
            self.camera.set_mode('3d')
        elif key in (ord('T'), ord('t')):
            self.telemetry = not self.telemetry
        elif key in (ord('G'), ord('g')):
            self.markers = not self.markers
            with viewer.lock():
                self.livery.set_markers(self.markers)
            if not self.livery.has_markers:
                info('[KEY G] No marker geoms in this model.')
        else:
            return False
        return True

    def sync(self, viewer, frame):
        with viewer.lock():
            self.replica.apply(frame)
            endpoint = frame.view.endpoint
            self.camera.update_viewer(viewer, bike_x=endpoint['position_m'], bike_z=endpoint['z_m'])
        viewer.sync()

    def print_hud(self, frame, view, *, prefix=''):
        if not self.telemetry:
            return
        from bike_sim.sim.ride.console import console, event, info
        if self._legend_generation != frame.generation:
            self._legend_generation = frame.generation
            self._failure = None
            console.print(self.hud.physical_legend(view))
        if frame.first_failure is not None and frame.first_failure != self._failure:
            self._failure = frame.first_failure
            event(f'[FIRST FAILURE] {frame.first_failure}')
        console.print(self.hud.styled_line(view))
        factor = '-' if view.achieved_rtf is None else f'{view.achieved_rtf:.2f}'
        info(f'{prefix}playback={view.requested_scale}x achieved_rtf={factor}')


class AchievedRate:
    """Wall throughput, independent of requested playback scale."""
    def __init__(self, now, time_s):
        self.rebase(now, time_s)

    def rebase(self, now, time_s):
        self.wall, self.time_s, self.value = now, time_s, None

    def update(self, now, time_s):
        elapsed = now - self.wall
        if elapsed >= .5:
            self.value = (time_s - self.time_s) / elapsed
            self.wall, self.time_s = now, time_s
        return self.value
