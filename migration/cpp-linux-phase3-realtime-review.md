# Realtime measurement — independent source review

Date: 2026-10-21. Scope: completion Task 3.04's new runtime measurement,
report/options/provenance/command/driver sources, its reference tests,
`RealtimeMeasurement.cmake`, and the shared reference helper. The Python
`realtime_measurement.py`, `tools/measure_realtime.py`, physical driver and
recorder were the behavior references. Existing runtime, CLI, RunLimits,
provenance and CMake call sites were checked for interface consistency.

This is **source-supported review**, not reproduction. The user prohibited
all compiler/configure/build/test/application/container execution. Neither
timings nor successful linking, sanitizer gates, parity or performance were
measured. Writing test source does not supply that evidence.

## Findings fixed in source

| Finding | Source evidence and consequence | Change / regression source |
|---|---|---|
| Native crash detail leaked as the terminal reason | `PhysicalRideDriver.advance` normalizes native outcomes beginning with `crash` to `crash`. The new driver's initial `reason()` returned the raw outcome, changing the measurement prefix's frontend contract. | `realtime_driver.cpp::reason()` applies the same normalization. The complete outcome detail remains in the actual native snapshot. No simulated crash result was manufactured. |
| Whole-run wall cap was omitted | Python owns `sim.default_limits()`, resets its clock in `start()`, and checks `max_wall_clock_s` before calling native advance. An 8 ms slice limits each call, not a sequence of calls that makes no progress. | The new driver owns actual `bike::legacy::RunLimits::for_track`, uses its finish position and 300-second cap, resets a monotonic-clock origin in `start()`, and checks the cap before any new physics. A cold injectable clock drives a new real-driver regression without sleeping. |
| Retention-off CSV lost its schema | Both Python physical recorders emit `schema_version,time_s` with CSV CRLF when columns are empty, and write NaN numeric cells as empty fields. Native `make_columns` returns an empty map when no samples are retained; the initial exporter wrote only a newline. | Export now emits the original empty schema and NaN-cell spelling. The short CLI regression asserts exact `schema_version,time_s\r\n` bytes. Retention remains off. |

Modified source paths are `native/src/runtime/realtime_driver.cpp`,
`realtime_runner.hpp`, and
`native/tests/reference/test_measure_realtime.cpp`. The driver factory's
clock parameter defaults to the existing monotonic source, so production
callers retain their current call syntax.

## Termination and timing order

The reviewed driver validates target and slice budget before terminating a
call. It uses the actual committed runtime snapshot, recognizes an existing
outcome before the duration limit, and keeps an already established terminal
reason. It checks the whole-run cap before accepting another slice, including
when prior slices advanced zero steps. A successful native advance releases
samples; an exception leaves the authoritative committed boundary available
to the measurement wrapper. No independent fake step counter was added.

The new clock-control test constructs a real native driver at clock zero,
advances the injected clock past 300 seconds, and calls `start()` to exclude
setup. A zero-budget control must preserve its starting step and remain
unterminated. At another 300 seconds, an ordinary compute-slice request must
report `wall_clock_cap` without advancing physics. It then flushes/closes.
This is regression source only; no real or simulated clock test was run.

The measurement wrapper keeps setup, measured work and export separate. Its
measured interval encloses every advance call, boundary/loop overhead and
the final flush. It records actual completed-step deltas, attempts final
flush/export of a failed prefix, and keeps `per_step_latency` null rather
than deriving per-step percentiles from chunk timing. Warmups are separate
from the measured-run median. The existing finite/monotonic elapsed-clock
validation was not relaxed by these driver changes.

Strict and benchmark modes use the same CLI projection and physical inputs;
the monitor policy is changed on the resolved runtime request. Setup uses
real prepared terrain vertices, native execution identity, timestep and
duration-derived step count. Export consumes the existing retained column
and interval APIs without enabling retention to make output nonempty.

## API, linking and helper review

The driver's called APIs exist with matching source signatures:
`NativeRideRuntime::advance`, `snapshot`, `flush`, `release_samples`,
`recorded_columns`, `recorded_intervals`, `set_finish_position` and `close`;
CLI request resolution, preparation, launch metadata, physical assets and
physical summary use the existing application interfaces. The new RunLimits
dependency is already reachable in the declared target graph:

- `bike_realtime_measurement` publicly links `bike_cli`.
- `bike_cli` publicly links `bike_replay` and `bike_viz`.
- `bike_replay` exposes `bike_app` and `bike_numpy_io`;
  `bike_viz` exposes `bike_legacy`, terrain, IO and platform targets.
- The measurement executable links the measurement library and receives
  startup policy, MuJoCo build load-name rewrite and install relocation.
- Engine-dependent reference targets use the shared helper's `ENGINE`
  branch. The reference directory returns early when `BUILD_TESTING` is off.

Generated measurement paths are a private header, avoiding quoted public
compile definitions in the serialized compiler-check context. This review
found no remaining obvious missing signature or declared dependency among
the inspected paths. It cannot establish that the graph configures, links,
relocates or passes compiler-policy checks.

The parent change to `ControlClock::from_seconds` was also inspected:
nonfinite ratios and ratios at or above `2^63` are rejected before converting
the rounded value to `int64_t`. For the finite positive binary64 domain below
that boundary, representable spacing prevents rounding into an out-of-range
integer. Existing minimum-step and integral-period checks remain. This is a
source argument, not a reproduced sanitizer result.

## Remaining qualification

Future authorized work must build/link the intended root targets and execute
the CLI/report/error-path regressions; compare actual short native and
Python-schema reports; inspect real provider/compiler provenance; and
measure the prescribed warmup plus full runs on both platforms. No headless
RTF threshold, strict-mode validity, successful no-progress termination,
report identity stability, or macOS relocation is certified by this review.
