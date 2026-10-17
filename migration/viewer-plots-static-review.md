# T48–T50: implementation and static review

## Evidence status

This change was prepared under the user's explicit instruction **not to compile
or run the project, tools, examples or tests**. Only source inspection and file
edits were performed. The new test bodies have not been executed. No successful
build, red/green test cycle, numerical oracle comparison, rendered image review,
GUI run or timing measurement is claimed here. The implementation is available
for review; qualification and removal gates remain open.

## T48: independent render ownership

`runtime::RenderFrame` carries a copied `RuntimeSnapshot`, copied presentation
values and the compiled model SHA-256. `NativeRideRuntime::render_frame()` copies
the committed cache under one advancement guard, so the integration vector,
generation, endpoint, sample and mutable model fields refer to one boundary.
`NativeResearchRuntime` forwards the same owned boundary. Legacy ride and
playground snapshots now carry their own integration state and presentation
arrays; no mutable `mjModel*` or `mjData*` is exposed to the viewer.

`viewer/render_owner.cpp` loads an independent model from the verified owned MJB
image and allocates independent data. It validates the digest, generation,
step, time/state agreement, finite integration values and all six model-field
extents before changing its replica. The six arrays are friction loss, site
positions, tendon stiffness, tendon damping, tendon spring lengths and tendon
ranges. `render_fields.hpp` checks signed extent products and uses the existing
`model_access` C-buffer boundary helpers.

Applying a frame invokes only state restoration and MuJoCo's kinematics,
center-of-mass positioning, camera/light update and tendon geometry update.
It invokes no simulation step, forward dynamics, force writer, sensor
acquisition or policy. A rejected frame leaves the previous replica intact;
an engine failure poisons it. A different model digest requires construction of
a replacement owner, even if dimensions happen to agree.

GL resources initialize only when drawing a nonzero framebuffer. The event
thread owns the renderer, scene, GL context and GLFW window. Destruction frees
MuJoCo graphics before the window; headless plot and simulation paths do not
initialize GLFW. Main-thread checks include the native macOS main-thread query.

## T49: controls, lifecycle and presentation

`viewer/session.hpp` is the application seam. The generic loop accepts frame,
advance, terminal, reset, stop, brakes, pause, close-request, operator-event,
key-event, HUD and presented-frame callbacks. Its convenience entrypoints cover
physical rides, research policy sessions, legacy rides, playgrounds and both
replay modes. Recording/finalization remains owned by the calling application.

The loop keeps fixed simulation dt, separate render cadence and bounded 8 ms
advance budgets. Pacing debt is reduced by actual completed physics steps;
pause, time-scale changes, resets and inspection seeks rebase clocks. The
supported generic time scales are 1/2/4/8. Inspection uses its own arbitrary
cursor speed and configurable `--fps`, and therefore reports no achieved
physics RTF. Camera presets retain source side/chase angles, distances and
offsets, direct horizontal following and vertical smoothing.

Controls include cameras, telemetry, markers, help, quit, reset, physical/legacy
brake toggle and strength, research pause/brakes, legacy cruise/assist/tyre and
suspension tuning, and playground travel/sweep/rider/tuning. Physical parameter
edits produce the fixed-configuration message. Marker visibility changes only
the render model. Missing marker geometry receives an explicit message.

Research pause/stop/reset drain an already accepted control interval without
issuing another policy action. `PolicySession::reset()` remains the reset owner
and preserves its existing `ResearchRun` before/after hooks. Reset callbacks
seal the preceding episode before changing the runtime. `close_requested` is
checked while paused and while holding a terminal frame, so SIGINT can close
either state without adding a physics tick. User close invokes stop once.

The physical HUD consumes owned sample channels, including force arrows,
drivetrain, rider contacts/balance and tyre state. First failure details are
latched and printed once per generation. UTF-8 HUD arrows, degree signs and
Greek letters count as visible columns when aligning legends. Playground HUD
uses its actual telemetry. Terminal overlays retain failures and outcomes.

The replay CLI hookup was reviewed statically with the reader owner. Inspection
uses stored qpos interpolation and the right saved velocity keyframe, with no
`engine::forward`. Restart, seek and wrap advance a presentation generation.
Checked replay displays the last verified prefix after a mismatch; its new
generation makes the rollback explicit. Incomplete/error exits use diagnostics
rather than a success report. Closing before verification does not certify a
replay. Runtime behavior of these integrations is still unverified.

## T50: artifact generation

`viz/plot.hpp` defines typed series, panels, annotations and reports. Native
adapters cover bicycle geometry, leverage/stroke/axle curves, compressed
comparison, damper dynos, travel, shaft-velocity histograms, acceleration,
pneumatic tyres, track profiles and physical drivetrain/suspension reports.
Existing geometry, suspension, terrain, low-pass and metrics calculators supply
the data. CSV reading preserves quoted fields, embedded newlines and missing
numeric cells and rejects duplicate columns and malformed row widths.

Static SVG, PNG and HTML paths retain coordinates, units, panel titles, labels,
trace ordering, secondary axes, dashed/stepped traces, missing-point gaps and
annotations. Physical HTML contains five drivetrain panels and two
suspension/load panels, shift lines/cut regions, obstacle positions and
available reset/validity/termination evidence. Exports keep the canonical
legacy `travel.png`, `shaft_velocity.png`, `acceleration.png`, `profile.png`
and `tyres.png` names and physical `speed.png`, `normal_load.png`, `power.png`
and `energy_residual.png` names. Plot writes inspect the opened regular file
before truncation and reject leaf symlinks/devices/FIFOs.

PNG is available in headless builds through an internal bounded RGB rasterizer
and standards-based PNG encoder (CRC32, zlib stored DEFLATE blocks and Adler32).
It includes the exact UTF-8 semantic report in an iTXt chunk. Optional pinned
Cairo supplies antialiasing and font rendering through the same layout. The
dependency-free rasterizer uses original compact ASCII glyphs; non-ASCII text
has a visible fallback glyph, with exact labels retained in SVG, HTML and PNG
metadata. Pixel equality to Matplotlib or Cairo is not claimed.

HTML embeds a real installed local Plotly bundle when one is available. No
Plotly bundle was present in the uploaded repository or the editing environment
locations inspected. No upstream bundle, license or digest was fabricated.
The built-in first-party `native-html-v1` renderer provides offline SVG charts,
legend toggles, unified hover, pan, zoom, reset and SVG/JSON export. Its HTML
metadata identifies that renderer accurately. Neither path needs a CDN. Actual
Plotly packaging and exact source Plotly object/layout equivalence remain
separate qualification gates; the first-party path is a native fallback, not a
claim that a Plotly resource has been supplied.

## CLI integration fixes

`cli/ride.cpp` now retains legacy tyre suffixes and source-compatible drive
settings hashes in run directory names, native headless sag fitting, initial
suspension and rider/drive/tyre summary extras, tyre event history and the
measurement-window exclusion for shaft-velocity plots. Its legacy headless
loop observes Ctrl-C cooperatively and still produces the committed report.

Physical viewer preview capture is created only for explicit `--record`. It
consumes presented owned scalar rows, grade and nearest obstacle, model/numeric
validity and terminal outcome. The preview stream is RAM-retained alongside
the full recording and written as `preview.csv` after final publication; its
derived `ride.html` is generated unless plots are disabled. An output path by
itself creates no physical preview or recording. Headless physical full capture
retains its static reports without manufacturing viewer cadence samples.

Physical event timestamps use monotonic elapsed time from launch across all
episodes. Reset rebases only the episode wall-clock watchdog. Sealing and final
publication preserve an earlier failure if preview/plot cleanup also fails.

## Added, unrun regression bodies

| Test | Authored assertions |
|---|---|
| `test_t48.cpp` | independent model/data/frame ownership; six-field replica updates; time/digest/extent rejection; stale versus new generation; marker discovery; thread confinement; context-free cleanup |
| `test_t49.cpp` | fake-clock debt, partial completion, speed/pause/reset; achieved RTF; mode-specific controls; braking clamps; camera presets/smoothing; replay exit codes; Unicode HUD alignment |
| `test_t50.cpp` | escaped SVG/HTML labels; JSON gaps and series semantics; PNG signature/dimensions; invalid inputs; quoted CSV and malformed/header-only inputs; seven shifts; signed/deduplicated obstacles; physical panel structure and filenames |

These tests supplement rather than replace the mandatory source-reference
cases. Outstanding gates include the strict native build, executed T48/T49/T50,
source numerical/structural oracle comparisons, image and browser inspection,
GUI interaction/reset/failure/SIGINT tests, macOS and Linux window ownership,
HiDPI/minimized framebuffers, sustained pacing/performance, both Cairo/native
PNG paths, and installed offline resource discovery. No Python source was
deleted and no removal checkbox is closed by this note.
