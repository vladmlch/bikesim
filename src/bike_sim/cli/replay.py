"""Verify a saved research episode by physically replaying every recorded input.

With ``--view`` the recording is instead played back visually in the MuJoCo
viewer (see ``bike_sim.sim.ride.playback_viewer``); ``--view --dry-run`` sweeps
the trajectory headlessly without opening a window.
"""
import argparse
import json
from pathlib import Path
import sys
from bike_sim.sim.research.replay import replay_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--episode', type=int, default=None,
                        help='research episode ID; required for multi-episode containers')
    parser.add_argument('--viewer', action='store_true', help='show the same checked replay in a viewer')
    parser.add_argument('--time-scale', type=int, choices=(1, 2, 4, 8), default=None,
                        help='viewer playback speed; does not change recorded physics')
    parser.add_argument('--view', action='store_true',
                        help='play the recording in the MuJoCo viewer instead of re-verifying it')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='initial playback speed multiplier (with --view)')
    parser.add_argument('--loop', action='store_true',
                        help='wrap at the end instead of holding the last frame (with --view)')
    parser.add_argument('--fps', type=float, default=60.0,
                        help='viewer frame rate / dry-run frame granularity (with --view)')
    parser.add_argument('--dry-run', action='store_true',
                        help='with --view: sweep the trajectory headlessly; no window opens')
    args = parser.parse_args(argv)
    if args.time_scale is not None and not args.viewer:
        parser.error('--time-scale requires --viewer')
    if args.viewer and args.view:
        parser.error('--viewer and --view are mutually exclusive')
    if args.view:
        if args.fps <= 0.0:
            parser.error(f'--fps must be positive, got {args.fps}')
        try:
            from bike_sim.sim.research.playback import load_playback
            playback = load_playback(args.recording)
        except (ValueError, OSError) as exc:
            print(f'REPLAY ERROR: {exc}', file=sys.stderr)
            return 1
        if args.dry_run:
            from bike_sim.sim.ride.playback_viewer import run_playback_dry_run
            return run_playback_dry_run(playback, fps=args.fps)
        from bike_sim.sim.playground import ensure_macos_mjpython
        ensure_macos_mjpython()
        from bike_sim.sim.ride.playback_viewer import run_playback_viewer
        return run_playback_viewer(
            playback, speed=args.speed, loop=args.loop, fps=args.fps
        )
    try:
        if args.viewer:
            from bike_sim.sim.playground import ensure_macos_mjpython
            ensure_macos_mjpython()
            from bike_sim.sim.research.replay_session import ReplaySession
            from bike_sim.sim.research.replay_viewer import run_replay_viewer
            return run_replay_viewer(ReplaySession(args.recording, episode_id=args.episode),
                                     time_scale=1 if args.time_scale is None else args.time_scale)
        print(json.dumps(replay_episode(args.recording, episode_id=args.episode), indent=2, allow_nan=False))
        return 0
    except (ValueError, RuntimeError, ArithmeticError, OSError, KeyError, TypeError, ImportError) as exc:
        print(f'REPLAY ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
