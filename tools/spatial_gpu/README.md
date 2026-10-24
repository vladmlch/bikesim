# Optional spatial GPU decision tools

**W55 source delivery; execution and measurements pending V11.** This directory
contains threshold helpers and their native contract test. It contains no GPU
kernel, GPU dependency, selected backend or performance result.

The CPU simulator remains the reference. Experiments belong exclusively under
`tools/spatial_gpu/`; product targets must never include, link, depend on or
install this directory. The root `BIKE_SPATIAL_GPU_TOOLS` option is a discovery
pointer, not a product build switch. This standalone CMake project refuses to
be included with `add_subdirectory`.

## Decision contract

| Helper | Necessary result |
|---|---|
| `latency_goal_met` | GPU section wall time ≤ half the best qualified CPU section time, with accuracy and determinism |
| `interactive_goal_met` | The latency criterion **and** full-ride GPU time ≤0.90× full-ride CPU time |
| `batch_goal_met` | Valid episodes/s ≥20× the best measured CPU process batch with the same physics |
| `offline_goal_met` | Complete input→artifact task at least three times faster with exact checks passed |

Every time/throughput input must be positive and finite. Thresholds are
inclusive; a zero, negative, NaN or infinite value is an invalid measurement,
including when an accuracy flag is false. The H1 section time includes dispatch,
upload, kernel, reduction, readback and synchronization needed by the next
sequential MuJoCo step. Calling `latency_goal_met` alone does not establish H1.

Passing a helper allows consideration of a separate design. It does not accept
that design, qualify physics or establish that an implementation fits its
engineering budget. H2 additionally needs a separately accepted implementation
scope. The [feasibility memo](../../docs/superpowers/audits/2026-10-22-spatial-3d-gpu-feasibility.md)
retains H1/H2/H3 evidence and decisions separately.

## Later V11 build and test

The following commands are provided for the later authorized verification
phase and were **not executed during W55**:

```sh
cmake -S tools/spatial_gpu -B build/spatial-gpu-tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/spatial-gpu-tools --target spatial_gpu_decision_test
ctest --test-dir build/spatial-gpu-tools --output-on-failure -R '^spatial\.gpu_decision$'
```

This graph uses C++23 and the existing `bt` implementation. That implementation
also contains manifest routines, so the target compiles the existing JSON and
SHA-256 sources locally to close their link dependencies. It does not link any
product library. The only external headers are the already acquired RapidJSON
tree pinned in `native/dependencies.lock.json`; its locked reader/document
digests are checked. The default location is `.deps/rapidjson-24b5e7a`. For an
existing relocated dependency tree, pass
`-DSPATIAL_GPU_RAPIDJSON_ROOT=/absolute/path/to/rapidjson-24b5e7a`. No network,
dependency acquisition, Python, MuJoCo or GPU SDK is needed by this test.

## Optional prototype location and budget

After V09 establishes the serial and best qualified CPU measurements, V11 may
create `tools/spatial_gpu/prototypes/<api>/` for the chosen host/API and retain
raw timings under `tools/spatial_gpu/evidence/<platform>/`. Keep original input
and artifact identities with the results. Neither path is a product hook.

The evidence budget is at most two engineering days. Without a usable GPU host,
record `unsupported-host` and stop that optional experiment. With a host, first
record device/driver/SDK and dated official API support sources, warm up, collect
at least 200 microtest repetitions, and compare distributions and whole valid
workloads against the best qualified CPU implementation. Report setup separately
from steady-state time and include dispatch/readback/sync in end-to-end cost.
Do not change dt, tyre laws, fixtures, precision, tolerances or dependency locks
to improve a result. Any positive decision requires a separate specification and
qualification plan; V10 completion remains independent of V11.
