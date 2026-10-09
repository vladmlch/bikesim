"""Visual checked replay: 0 passed, 1 mismatch, 2 closed before final checks."""
from __future__ import annotations

import json
from queue import Empty, SimpleQueue
import sys
import time

from bike_sim.sim.playback import PlaybackClock, COMPUTE_SLICE_S, RENDER_INTERVAL_S, speed_key
from bike_sim.sim.ride.physical_view import present_view
from bike_sim.sim.ride.presentation import AchievedRate, FramePresenter


def replay_exit_code(session):
    return 1 if session.error is not None else (0 if session.done else 2)


def run_replay_viewer(session, *, time_scale=1):
    try:
        import mujoco.viewer
        keys = SimpleQueue()
        clock = PlaybackClock(session.timestep_s, scale=time_scale)
        paused = stop = reported = False
        presenter = FramePresenter(session.make_render_model())
        presenter.replica.apply(session.snapshot())
        print(f'[bike-replay {session.backend}] Space pause | R restart | C/1/2 camera | '
              'T telemetry | G markers | Q/Escape exit | F6/F7/F8 speed', flush=True)
        with mujoco.viewer.launch_passive(presenter.replica.model, presenter.replica.data,
                key_callback=keys.put, show_left_ui=False, show_right_ui=False) as viewer:
            now = last_sync = last_hud = time.monotonic()
            rate = AchievedRate(now, session.time_s)
            clock.rebase(now, step=session.step)
            while viewer.is_running() and not stop:
                while True:
                    try:
                        key = keys.get_nowait()
                    except Empty:
                        break
                    now = time.monotonic()
                    scale = speed_key(key, clock.scale)
                    if scale is not None:
                        clock.set_scale(scale, now=now, step=session.step)
                        rate.rebase(now, session.time_s)
                    elif key == 32:
                        paused = not paused
                        clock.set_paused(paused, now=now, step=session.step)
                        rate.rebase(now, session.time_s)
                    elif key in (ord('R'), ord('r')):
                        try:
                            session.restart()
                        except Exception as error:
                            print(f'REPLAY ERROR: {error}', file=sys.stderr)
                        else:
                            paused = reported = False
                            now = time.monotonic()
                            clock.set_paused(False, now=now, step=session.step)
                            clock.rebase(now, step=session.step)
                            rate.rebase(now, session.time_s)
                    elif key in (ord('Q'), ord('q'), 256):
                        stop = True
                    else:
                        presenter.handle_key(key, viewer)
                now = time.monotonic()
                # On a mismatch step can roll back to the last verified frame.
                # Freeze the clock as well; it must not reject that diagnostic frame.
                if session.error is None:
                    target = clock.target_step(now, current_step=session.step)
                    if not stop and not paused and not session.done and target > session.step:
                        try:
                            session.advance(COMPUTE_SLICE_S, target_step=target)
                        except Exception as error:
                            clock.rebase(time.monotonic(), step=session.step)
                            rate.rebase(time.monotonic(), session.time_s)
                            print(f'REPLAY ERROR: {error}', file=sys.stderr)
                if session.done and not reported:
                    print(json.dumps(session.report(), indent=2, allow_nan=False), flush=True)
                    reported = True
                now = time.monotonic()
                factor = rate.update(now, session.time_s)
                if now-last_sync >= RENDER_INTERVAL_S:
                    frame = session.snapshot()
                    view = present_view(frame.view, requested_scale=clock.scale,
                                        achieved_rtf=factor, track=session.track)
                    # A mismatch may intentionally restore a prior verified step.
                    if session.error is not None:
                        presenter.replica.frame = None
                    presenter.sync(viewer, frame)
                    last_sync = now
                    if now-last_hud >= .5:
                        status = session.error or ('passed' if session.done else 'paused' if paused else 'checking')
                        presenter.print_hud(frame, view, prefix=f'[replay {status}] ')
                        last_hud = now
                time.sleep(.001)
        code = replay_exit_code(session)
        if code == 2:
            print(f'REPLAY INCOMPLETE: closed at physics step {session.step}; final verification not performed.',
                  file=sys.stderr)
        return code
    finally:
        session.close()
