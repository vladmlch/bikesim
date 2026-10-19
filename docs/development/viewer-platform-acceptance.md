# Viewer platform acceptance

Status: **implementation and source review only; no execution or desktop qualification**.
This record belongs to portability T19. The 2026-10-17 wave-2 session was explicitly
prohibited from building, running tests, opening viewers or executing probes. Every
desktop row below is pending. Existing macOS or Linux build/test reports are not
evidence that one of these interactive sessions ran.

The contract is [portability design D7](../superpowers/specs/2026-10-14-linux-macos-portability-design.md)
and [T19](../superpowers/plans/2026-10-14-linux-macos-05-runtime.md).
The source snapshot supplies implementation evidence, not its original Git history
or a record of the user's desktop.

## Preserved entry points and ownership

| Mode | Actual entry and owner | Preserved behavior |
|---|---|---|
| Physical | `bike-ride --backend native` → `cli/ride.py` → `physical_frontend.run_viewer` | Launcher before driver construction; driver-owned independent render model and owning snapshots; fixed `dt_s`; rendering through `FramePresenter`. |
| Research | `bike-ride --research --backend native` → `run_ride_research` → `run_research_viewer` | Launcher before environment construction; policy windows keep their integration clock; reset/stop drains only an already accepted window; render replica has separate model/data. |
| Checked replay | `bike-replay RUN --viewer` → `ReplaySession` → `run_replay_viewer` | Launcher before replay owner construction; owning snapshots and isolated render model; early close returns 2, mismatch returns 1, completed verification returns 0. |
| Recorded playback | `bike-replay RUN --view` → `load_playback` → `run_playback_viewer` | Recorded pose interpolation, pause/seek/speed/end-frame hold; no checked-replay success claim. The existing legacy loader and pose-only model update remain unchanged. |
| Playground | `bike-playground` → `run_interactive_playground` | Existing test stand, controls and main-thread viewer entry; this is not the native physical/research runtime. |

`bike-research` is the headless research CLI in this snapshot. Use the
`bike-ride --research` command below for its interactive viewer.

`ensure_macos_mjpython()` retains its opening platform/sentinel guard. Linux returns
before launcher lookup, environment mutation or dylib inspection. Darwin keeps the
selected `mjpython` executable, exact argument order, `MUJOCO_MJPYTHON=1` marker and
conditional prepending of the base interpreter's `lib` directory to
`DYLD_FALLBACK_LIBRARY_PATH`. An already marked process never relaunches. A missing
launcher now identifies the required same-environment `mjpython`. Failed lookup or
`execv` restores the previous environment and clears the checked sentinel so a
caller can repair the environment and retry. Successful re-exec behavior is unchanged.

The playground and legacy ride entry points now consult this helper before importing
`mujoco.viewer`. The duplicate legacy physical async-worker setup before the launcher
is removed; its existing setup after the launcher is retained. Neither a `DISPLAY`
value nor a `WAYLAND_DISPLAY` value is treated as
a capability check: interactive setup still imports and invokes the real viewer,
and its actual library/context error propagates. No fallback renderer or automatic
system-package installation was added.

Source-supported ownership repairs in this wave:

- Physical and checked replay owners close after import, context-creation or loop
  failure. If close also fails, its error is attached to the original exception
  rather than hiding the actual viewer failure or interrupt. A close failure after
  an otherwise normal run still propagates.
- Research capture construction and session attachment are now inside the viewer's
  cleanup boundary. Either failure closes the environment with pending work discarded;
  a successfully constructed launch follows its existing finalization policy.
- Recorded-playback Q/Escape now requests stop. The render loop's context owns window
  teardown, including a key arriving before `launch_passive` has returned its handle.

These changes do not alter the integration timestep, numerical kernels, policy
selection, recording opt-in rules or final publication rules. The actual physical
frontend is included because it is the native CLI's consumer, despite being absent
from the old T19 file list.

## Scope of source evidence

Physical, research and checked replay presentation use the existing
`physical_view.apply_frame` path: state copy plus geometry updates, without
`mj_step` or `mj_forward`. Tests of their native owning snapshots already live in
`tests/reference/test_native_viewer_snapshot.py` and frontend behavior in
`tests/reference/test_frontend_windows.py`. These existing tests were not run here.

Legacy `--view` playback currently reconstructs its own playback model via
`replay.rebuild_environment` and calls `mj_forward` after copying recorded poses;
its source contract is distinct from the live native snapshot path. This wave does
not establish portable recording reconstruction, support for every recording schema
or cross-host playback. T22 must resolve and qualify those capabilities. Supply an
actually supported recording and retain any rejection as evidence; do not manufacture
a legacy export or suppress an execution-identity mismatch to make a desktop row pass.

## Required desktop matrix

The initial release Linux target is Ubuntu 24.04 x86_64. Linux aarch64 and macOS
Intel need separate rows if exercised; their results cannot replace the release
targets. X11 and a GLFW X11 window served by XWayland are separate Linux sessions.
The XWayland row does not qualify native Wayland, EGL or OSMesa.

| Row | Host/session | Mode | Requested scale | Status | Evidence |
|---|---|---|---|---|---|
| M-PH | Actual macOS Apple Silicon baseline; Cocoa | Physical native | 1 and 8 | pending | No desktop session performed |
| M-RS | Actual macOS Apple Silicon baseline; Cocoa | Research native | 1 and 8 | pending | No desktop session performed |
| M-CR | Actual macOS Apple Silicon baseline; Cocoa | Checked replay | 1 and 8 | pending | Needs a supported finalized T21 recording |
| M-PB | Actual macOS Apple Silicon baseline; Cocoa | Recorded playback | 1 and 8 | pending | Needs supported recorded poses; T22 compatibility pending |
| M-PG | Actual macOS Apple Silicon baseline; Cocoa | Playground | Existing stand pacing | pending | No desktop session performed |
| X-PH | Ubuntu 24.04 x86_64; X11 | Physical native | 1 and 8 | pending | No desktop session performed |
| X-RS | Ubuntu 24.04 x86_64; X11 | Research native | 1 and 8 | pending | No desktop session performed |
| X-CR | Ubuntu 24.04 x86_64; X11 | Checked replay | 1 and 8 | pending | Needs a supported finalized T21 recording |
| X-PB | Ubuntu 24.04 x86_64; X11 | Recorded playback | 1 and 8 | pending | Needs supported recorded poses; T22 compatibility pending |
| X-PG | Ubuntu 24.04 x86_64; X11 | Playground | Existing stand pacing | pending | No desktop session performed |
| W-PH | Ubuntu 24.04 x86_64; Wayland session via XWayland | Physical native | 1 and 8 | pending | No desktop session performed |
| W-RS | Ubuntu 24.04 x86_64; Wayland session via XWayland | Research native | 1 and 8 | pending | No desktop session performed |
| W-CR | Ubuntu 24.04 x86_64; Wayland session via XWayland | Checked replay | 1 and 8 | pending | Needs a supported finalized T21 recording |
| W-PB | Ubuntu 24.04 x86_64; Wayland session via XWayland | Recorded playback | 1 and 8 | pending | Needs supported recorded poses; T22 compatibility pending |
| W-PG | Ubuntu 24.04 x86_64; Wayland session via XWayland | Playground | Existing stand pacing | pending | No desktop session performed |

Each row needs a real window with exercised controls, visible complete geometry and
an orderly close. A returned handle, mocked window, no-display run or Xvfb session
alone is insufficient to mark visual quality passed.

## Future commands, not executed in this session

Use a host-local build that has already passed its required native gates. Set
`BIKE_NATIVE_BUILD_PATH` to its absolute directory, with a matching interpreter and
locked dependency environment. Do not reuse an extension from the other platform.
The [installation modes](install-modes.md) and
[dependency matrix](dependency-matrix.md) describe the existing environment route.
Record the actual GLFW, OpenGL loader, Mesa or proprietary graphics driver and
display-server packages installed on the Linux desktop; those packages are not
installed by the viewer.

### Physical and research

```sh
: "${BIKE_NATIVE_BUILD_PATH:?Set an absolute, qualified, host-local native build directory}"
export BIKE_NATIVE_BUILD_PATH
uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 8
uv run --frozen --group native bike-ride --research --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
uv run --frozen --group native bike-ride --research --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 8
```

For physical controls exercise C/1/2 cameras, T telemetry, G markers, Space brakes,
comma/period brake strength, R reset, F6/F7/F8 speed, and Q/Escape close. Physical
Space is a brake toggle, not pause. For research exercise Space pause/resume,
B brakes, R boundary-safe reset, C/1/2, T, G, F6/F7/F8 and Q/Escape.

On each backend/mode preserve the terminal frame, inspect it, reset, then close
after a terminal outcome. Repeat an open/close cycle and a separate Ctrl-C cycle.
Record the actual outcome and any pre-existing physics failure without relaxing
validity checks. Rendering should remain responsive at both requested scales.
The requested scale changes wall-clock pacing only: record achieved RTF from the
HUD/log and distinguish initialization/warmup from steady-state observations.
Do not report achieved 8x merely because `--time-scale 8` was requested.

The default commands record nothing. A separate T21 pass adds `--record --out`
and an explicit destination, resets at least twice, and records the final directory
printed at close. Completed episodes remain in RAM until launch finalization;
an opted-in preview CSV can already exist during the physical viewer session.
Do not equate that preview with a published finalized recording. Check that
recording-off launches create no persistence and recording-on launches publish
one final run with all reset-separated episodes. T19 does not mark T21 passed.

### Checked replay and recorded playback

`RUN` is the actual finalized recording directory, never the `--out` parent.
For a multi-episode checked replay, `EPISODE` must be a real episode ID. Keep the
execution identity intact. A foreign identity rejection is required behavior and
does not count as successful same-environment checked replay.

```sh
: "${RUN:?Set the actual finalized T21 recording directory}"
: "${EPISODE:?Set an episode ID present in the recording}"
uv run --frozen --group native bike-replay "$RUN" --episode "$EPISODE" --viewer --time-scale 1
uv run --frozen --group native bike-replay "$RUN" --episode "$EPISODE" --viewer --time-scale 8
```

Exercise checked replay's Space pause, R restart, C/1/2, T, G, F6/F7/F8 and
Q/Escape. Closing before final checks must remain incomplete (exit 2), not pass.
After completed verification, retain the report and inspect the held final frame.
Record a mismatch/error distinctly and preserve the last verified display frame.

For the existing supported recorded-pose playback format, use a separately
identified `PLAYBACK_RUN`. The current `--view` branch does not use the checked
replay `--episode` selector; T22 must qualify the intended container formats.

```sh
: "${PLAYBACK_RUN:?Set an existing recording supported by the pose playback loader}"
uv run --frozen --group native bike-replay "$PLAYBACK_RUN" --view --speed 1
uv run --frozen --group native bike-replay "$PLAYBACK_RUN" --view --speed 8
```

Exercise Space pause, left/right seek, Shift plus left/right fine seek, up/down
speed, R restart, end-frame hold, a separate `--loop` pass and Q/Escape. This is
unverified playback of recorded states. No physical replay report is produced.
Neither a playback-only result nor a `--view --dry-run` result closes the checked
replay gate.

### Playground

```sh
uv run --frozen --group native bike-playground
```

Exercise the printed stand controls, camera modes, rider/marker toggles, reset,
normal window close and a separate Ctrl-C run. Preserve the existing test-stand
pacing and visible geometry; do not treat it as native physics qualification.

### Automated controls and platform gates

These are proposed commands for a later authorized verification session:

```sh
uv run --frozen --group native python -m pytest tests/portability/test_viewer_launch_platform.py -q
uv run --frozen --group native python -m pytest tests/reference/test_frontend_windows.py -q
```

The new tests fake library launch and owner surfaces. They cover Linux early return,
Darwin original argv, marker/sentinel and conditional dylib fallback, failed-exec
rollback and retry, missing launcher, launcher-before-GL ordering, early playback
quit, physical/replay primary versus cleanup exceptions, normal close/incomplete
replay and research setup failure/success ownership. They do not load a native
extension, step a simulation, open a context or prove actual GUI availability.

After focused controls, run the profiles required by [AGENTS.md](../../AGENTS.md)
and [TESTING.md](../TESTING.md) for the integrated change. Serialize the native
gate on each machine. A later run should also exercise native snapshot controls
against the selected artifact and T21/T22 lifecycle and replay tests. None of
those gates was run for this source-only wave.

## Evidence record template

Copy this table for each real host/session and mode; attach actual logs and images
when available. Keep unperformed checks pending and failed checks failed.

| Field | Actual evidence to record |
|---|---|
| Matrix row, date, operator | Row ID plus local date/time and timezone |
| Source revision | Actual Git commit, dirty state, patch/source hash if applicable |
| OS and architecture | Actual macOS build or Linux distribution/kernel, CPU architecture |
| Desktop transport | Cocoa, X11, or Wayland compositor plus XWayland; identify actual window transport |
| GPU and graphics stack | GPU, driver version, OpenGL renderer/version, GLFW library/version, display transport |
| Interpreter and launcher | Exact Python executable/version, uv environment, resolved mjpython on macOS, observed re-exec/marker behavior |
| Native artifact | Absolute selected build, extension identity/hash, build context and numerics provider |
| Invocation | Exact argv, working directory and relevant environment; no credentials |
| Inputs | Config/track identities and actual replay run plus episode ID where applicable |
| Controls | Every key/control exercised, reset count, terminal hold, open/close and Ctrl-C results |
| Visual inspection | Bicycle/rider/terrain completeness, camera views, HUD, terminal frame and screenshots |
| Timing | Requested scale, fixed integration timestep, warmup, achieved RTF sample/range and source log |
| Recording | Off/no files or opted-in final run path, episode count, publication timing, preview distinction |
| Outcome | Actual exit status, pass/fail/pending per check, primary error and cleanup notes |
| Attachments | Exact log/screenshot paths and hashes; missing evidence stays pending |

T19 is verified only when its required real macOS and Linux X11 sessions and
automated controls have evidence. The separate XWayland row remains explicitly
qualified or unqualified on its own evidence. T29/T30 consume those records;
creating this matrix does not close their release gates.
