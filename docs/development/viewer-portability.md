# Interactive viewer startup on macOS and Linux

This document describes the source changes for portability **T19**, delivered
in the 2026-10-17 second wave. The new regressions and GUI acceptance have not
been executed. Historical results in `docs/RIDE.md` do not qualify these changes.

## Platform preparation

On macOS the passive MuJoCo viewer needs the UI dispatcher installed by
`mjpython`. Application startup checks for that dispatcher before creating a
simulation, worker or recording session. When relaunch is required, it keeps
the active Python interpreter, original Python arguments (including `-m` and
interpreter flags), working directory and environment. The selected
`BIKE_NATIVE_BUILD_PATH` therefore remains the same. A missing launcher, failed
re-execution or unsuccessful relaunch is an explicit setup error.

The old `bike_sim.sim.playground.ensure_macos_mjpython` import remains available
as a compatibility delegate. An inherited environment variable or an executable
name alone is not proof that the MuJoCo UI dispatcher exists.

On Linux no `mjpython` lookup or re-execution takes place. Interactive startup
requires a display session; it checks `DISPLAY` / `WAYLAND_DISPLAY` and reports
an absent session before opening a window. A set display variable does not
prove that the display server, permissions, OpenGL driver or installed GLFW
backend works. GLFW initialization runs synchronously so a failure can be
reported by the caller rather than left in a background viewer thread.
Later asynchronous window, context or rendering failures after a successful
initialization remain dependent on MuJoCo's viewer implementation and require
desktop acceptance; this preparation does not establish that they cannot hang.

`WAYLAND_DISPLAY` permits trying the installed backend. It is not a claim that
every GLFW wheel supports native Wayland; an XWayland `DISPLAY` may be needed.
Offscreen MuJoCo backends do not replace an interactive desktop viewer.

## Owners and launch boundaries

| Entry point | Preparation boundary |
| --- | --- |
| `bike-playground` and `bike-sim --playground` | Before construction and output work for the interactive invocation |
| Interactive `bike-ride` | Before native preflight and simulation/driver construction |
| `bike-ride --research` | Before creation of the research session and workers |
| `bike-replay --viewer` | Before construction of the checked replay session |
| `bike-replay --view` | Before loading and presenting the recorded trajectory |
| Direct frontend with an existing driver/session | Validate the platform without re-executing; use its existing cleanup contract on failure |

A direct caller that already owns simulation or recording state must prepare
macOS launch before constructing that state. Re-executing from a low-level
viewer would discard state and bypass cleanup. Such viewers therefore report
an unprepared macOS invocation instead of relaunching it.

The compatibility `run_physical_viewer(sim)` entry retains its established
caller-ownership contract; it does not acquire ownership of an externally
created simulation merely to open a window.

Headless runs, track previews, help/listing commands and `bike-replay --view
--dry-run` do not request interactive preparation. Wave T18 owns complete
headless import isolation, which cannot be qualified from this snapshot.

## Failures and cleanup

Startup and render-loop exceptions retain their cause. The passive-window
context and the final physical-driver/replay-session close preserve an already
active exception when closing also fails; the secondary failure is attached
as an exception note. This includes `KeyboardInterrupt`. A close failure when
no earlier error is active still propagates.

Playback Q/Escape requests that the presentation loop stop. Its window is
closed by the owning context, including when a callback arrives during launch.
Preview-CSV publication and recording finalization keep their existing
behavior; their broader lifecycle acceptance belongs to T21.

Closing a checked-replay viewer early retains the existing incomplete result;
displaying a trajectory does not become checked replay. Recording and episode
accounting continue to use their existing frontend lifecycle.

## Acceptance still required

The following checks require a later execution-authorized session:

1. Focused launcher, fake-viewer and frontend cleanup regressions.
2. macOS desktop: script, console entry point and `python -m` routes; normal
   window close, interruption and restart; verify the selected native artifact
   remains unchanged across `mjpython` launch.
3. Linux desktop: working display, missing display, unavailable display and
   backend initialization failure; physical/research, playback and checked
   replay windows.
4. Both desktops: existing keyboard controls, final-frame behavior and owned
   resource cleanup on success and failure.
5. Reconcile the original T18 prerequisite evidence and T19 acceptance brief,
   which were not included in the supplied Repomix export.

No actual GUI, native build, recording round trip or supported-platform claim
is established by the source changes alone.

## Upstream contract references

- [MuJoCo Python passive viewer documentation](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)
- [MuJoCo viewer implementation](https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/viewer.py)
- [MuJoCo macOS launcher](https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/mjpython/mjpython.py)

These references were inspected on 2026-10-17 to establish launcher and passive
viewer behavior. The installed MuJoCo version on each acceptance host must
still be checked; upstream `main` is not a pinned local dependency.
