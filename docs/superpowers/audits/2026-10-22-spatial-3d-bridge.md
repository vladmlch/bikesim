# W18 constrained bridge source handoff

Status: `source_written`, verification `pending V05`. No configure, build,
compiler, syntax check, test or simulation was run while writing this change.
All values below are unmeasured. W18 does not qualify or enable a runtime
capability; W25/W27 producers and the fresh constrained startup dependency must
exist before its positive integration can pass.

## Authored experiment

`native/tests/spatial/test_planar_bridge.cpp` runs the 2 s cases plus flat
static/0.1 m/s and integrated probe/reset/codec checks.
`native/tests/spatial/test_planar_bridge_long.cpp` runs the same three inputs and
actuator program for 10 s, with the `slow` label. Their private helper is
`native/tests/spatial/support/planar_bridge_cases.hpp`; it changes no frozen
production interface.

The paired fixture uses native `make_setup`, `settle` (through the production
`settle_context` owner), bootstrap sealing and `NativeRideRuntime`. The planar
side must install the existing pneumatic backend. The spatial side must own
`Pneumatic3dTireWriter` and a `ProfileExtrusion3d` made from the exact same track.
It cannot satisfy the test using an analytic stand-in, empty tyre state or an
unsupported-capability skip. W25/W27 are unresolved producers at this handoff;
normal construction failure remains a test failure.

The manifest fixes dt = 0.00125 s, control period = 0.005 s, start x = 2 m,
initial speed = 6 m/s, seed string `0`, no rider, fast tyres, track materials,
stock front/rear longitudinal parameter inputs and one piecewise-constant demand
trace. Both sides retain every interval synchronously. The same pressure,
radius, width and longitudinal relaxation input are checked on the actual
spatial writer. No coefficients are fitted from bridge output.

Weight is measured at execution from all compiled body masses and the model's
gravity vector. It is not an authored nominal bicycle/rider number. Each
retained sample must name the actual backend and contain finite world forces,
including the lateral lane. Every physics interval must be present and the
requested horizon must complete; early termination does not count as a shorter
successful experiment.

## Initial-state precondition requiring V05 review

The task requires identical initial physical state. Independent fresh settles
may produce different vertical positions or tyre deflections when the two laws
differ. This suite therefore emits the maximum difference over every named
common scalar joint's position and velocity, requires at most 1e-12, and requires
extra constrained coordinates to be zero. This is an equality precondition,
not a replacement for the V05 dynamics tolerances. It does not rewrite a saved
integration vector while leaving material history or the energy datum stale.

If that precondition fails, V05 must supply a reviewed common physical-state
initialization/capture path, or resolve the intended contract explicitly. The
measured difference belongs in the table below. It must not be set to zero in
the manifest or hidden by accepting two unrelated settled states. Fresh
constrained settling remains an independent required assertion.

## Metrics and state checks

Load RMS and wheel-axis torque impulse are measured separately for front and
rear wheels. The retained normal load and `wheel_axis_moment_nm` channels are
the observations; all integrals use their real interval times. Distance is the
committed endpoint position minus the measured initial root position. Raw
contact transition lists must have the same events and directions; an extra or
missing event receives a failing maximum delta, never zero. Filtered transition
timing, total grounded duration difference and observed hold duration are
reported independently against the actual configured grounded window.

The short suite checks actual writer state through the existing cold
`validation::RuntimeRigAccess::inspect` seam. Strict W27 codec/base64 round trips
use real committed bytes. Wrong identity and nonfinite material restores must
reject without changing those bytes. Repeated observational writer evaluations
must preserve committed state and force bytes. Runtime probes preserve model
identity, the entire integration vector, tyre bytes, step/time/outcome and the
canonical hold/brake/steer/outcome/bootstrap-phase inventory. Continuation after
codec roundtrip equals the uninterrupted sibling; reset equals fresh bootstrap
and its continuation, with only the expected generation increment.

The suite prints JSON containing physical metrics, measured weight, actual
coverage, terrain fingerprint, execution identity and source manifest hash.
Construction/precondition failures print an error when the case reaches that
boundary; they do not fabricate metrics. Replaying the generated fixture or
collecting evidence is explicitly deferred to V05.

## Per-channel worst cases

All cells marked `null` are pending observations, not zeros.

| Platform | Track | Duration | Weight N | Initial-state max error | Load RMS front/rear N | Distance error m | Torque impulse error front/rear N·m·s | Raw transition delta steps | Persistence error s | Outcome pair | Status |
|---|---|---:|---:|---:|---|---:|---|---|---|---|---|
| macos-arm64 | flat | 2 s | null | null | null | null | null | null | null | null | pending |
| macos-arm64 | rough_uphill_savage | 2 s | null | null | null | null | null | null | null | null | pending |
| macos-arm64 | alpine_rocky | 2 s | null | null | null | null | null | null | null | null | pending |
| macos-arm64 | flat | 10 s | null | null | null | null | null | null | null | null | pending |
| macos-arm64 | rough_uphill_savage | 10 s | null | null | null | null | null | null | null | null | pending |
| macos-arm64 | alpine_rocky | 10 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | flat | 2 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | rough_uphill_savage | 2 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | alpine_rocky | 2 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | flat | 10 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | rough_uphill_savage | 10 s | null | null | null | null | null | null | null | null | pending |
| linux-aarch64 | alpine_rocky | 10 s | null | null | null | null | null | null | null | null | pending |

| Additional acceptance | Threshold | macos-arm64 | linux-aarch64 |
|---|---|---|---|
| Static and 0.1 m/s flat total Fy | 1e-9 N | null | null |
| Static sum of loads vs measured weight | relative 1e-6 | null | null |
| Load RMS per wheel | 0.02 × measured weight | null | null |
| Distance | max(0.01 m, 0.01 × abs(planar distance)) | null | null |
| Torque impulse per wheel | max(0.01 N·m·s, 0.01 × abs(planar impulse)) | null | null |
| Raw contact transitions | at most 1 physics step | null | null |
| Persistence timing/duration | at most 1 configured window | null | null |
| Outcomes | identical | null | null |
| Probe/restore/reset/continuation | real producer, canonical bytes | null | null |
| Fresh constrained settle | real owner, no cache | null | null |
| Execution identity | captured from running binary | null | null |

The strict old planar goldens remain a separate V05 gate. This numeric bridge
does not qualify free-root or articulated-rider capability.
