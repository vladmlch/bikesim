# proto_native_bench — retained native benchmark

`bench.cpp` is the native implementation of the former `bike_native`
binding workload (native-safety runtime plan, tasks R2+R3). It replays one
recorded `mjData` snapshot through `mj_step` under a shared, deterministic
workload definition and emits a JSON report whose per-stage checksums the
CTest suite verifies. Every native engine call crosses the
`bike_native_engine` boundary (E1): MuJoCo warnings/fatals arrive as
recoverable `engine::EngineFailure` exceptions, and the CLI reports failures
and exits nonzero — it never takes MuJoCo's process-exit path.

> **Post-T63 status.** The paired Python twin (`bench.py`,
> `dump_model.py`, the `state_format` Python module and the pytest
> conformance harness) was removed at T63 with the interpreter runtime.
> The retained surface is the C++ target alone: the `native_bench`
> executable, this directory's `bench.cpp`/`state_format.hpp`, and the
> pinned artifact fixtures under `artifacts/`. The cross-language parity
> claims below are historical provenance for those fixtures — the current
> gate is the C++ CTest suite (`reference.native_benchmark`,
> `reference.native_state_bundle`, `reference.native_state_restore`,
> `reference.native_state_fuzz`, `reference.equilibrium_state_file`)
> against the committed artifacts and its own generated corpora.

## Build

The benchmark is a root-graph target (`add_executable(native_bench
tools/proto_native_bench/bench.cpp)`) built under the same strict ISO C++23
policy as every native target:

```bash
cmake --preset release
cmake --build build/native-release --target native_bench
```

The binary lands at `build/native-release/native_bench`. Sanitizer variants
are ordinary preset flavors — the target is reachable under the `asan`,
`rtsan` and `coverage` builds by configuring the corresponding preset.

## Run

```text
native_bench {bare,glue,heavy,operation_mix} [steps] [rewind]
             --artifacts <directory> [--emit-json <path>]
```

- `bare` — `mj_step` only, on a frozen replayed state (periodic state
  rewind).
- `glue` — bare + the engine-call surface the deleted Python glue used:
  forward passes, point Jacobians, `mj_name2id` lookups, eight
  `qfrc_applied` assemblies driven through the engine-projected
  `qfrc_actuator` mapping, and 100 channel writes per step.
- `heavy` — glue + representative telemetry/accounting volume: an `efc_*`
  row unpack, five attachment wrench solves over paired Jacobians, an energy
  ledger, per-step record churn, and a period-batch gradient-free solve.
- `operation_mix` — the retained R2 approximation mix. It exercises roughly
  the same engine surface but follows its own (deliberately different)
  arithmetic and carries **no parity claim**: its report is labeled
  `"comparison": "operation_mix"` and its numbers must not be quoted as
  conformance evidence.

`bare`, `glue`, and `heavy` remain the matched workload definitions that
the (deleted) paired run executed identically in both languages — same
warmup/rewind phases, stage order, operation counts, attachment Jacobians
and `1e-6 * I`-regularized 3x3 solve sequence, `qfrc_actuator` mapping and
SplitMix64-driven channels from a fixed seed.

`steps` (default 4000) is the timed iteration count and `rewind` (default
250) is the state-restore period; both must be integers in
`[1, 10000000]` and are parsed under a full-consumption contract — malformed
values, unknown modes, stray options, and extra positionals are errors
(nonzero exit, message on stderr). `-h` / `--help` prints usage;
`--selftest` prints the CRC64-ECMA check vector and exits.

`--artifacts` is required and names either:

- an **artifacts root** containing `current.json`, the pointer to one
  versioned bundle directory, or
- a **concrete bundle directory** that itself holds the four bundle members.

`--emit-json` writes the run report: workload definition, operation counts,
per-stage checksum contributions, solver outputs, provenance (language,
mujoco version, bundle CRCs, pointer-vs-direct resolution), and the total
checksum. The C++ CTest suite verifies this document.

Example against the tracked artifacts:

```bash
./build/native-release/native_bench glue 5 --artifacts tools/proto_native_bench/artifacts
```

## Bundle format (BIKEST02)

One bundle directory is one immutable generation of the benchmark inputs:

```text
model.mjb      compiled MjModel bytes
state.npz      float64 arrays: time(1,), qpos(nq), qvel(nv), act(na),
               ctrl(nu), qfrc_applied(nv), xfrc_applied(nbody,6),
               qacc_warmstart(nv)
state.bin      80-byte little-endian header + binary64 payload (below)
manifest.json  provenance: model dims, timestep, source config/track,
               warmup steps, generator commit, checksums, workload params
```

`state.bin` header (exactly 80 bytes, little-endian):

```text
offset  0  magic8         "BIKEST02"
offset  8  version        u32 = 2
offset 12  flags          u32 = 0
offset 16  nq, nv, na, nu, nbody   u64 each
offset 56  payload_bytes  u64
offset 64  model_crc64    u64  (CRC64-ECMA over model.mjb bytes)
offset 72  state_crc64    u64  (CRC64-ECMA over the payload bytes)
```

Payload is binary64 little-endian, in order:

```text
time(1), qpos(nq), qvel(nv), act(na), ctrl(nu), qfrc_applied(nv),
xfrc_applied flattened (6*nbody), qacc_warmstart(nv)
```

Required payload = `8 * (1 + nq + 3*nv + na + nu + 6*nbody)` bytes.

Reader contract (size floor, magic, version, flags, u64 field ranges vs the
signed mjtSize domain, dimension equality against the loaded model,
declared `payload_bytes` vs the recomputed requirement, actual file length
vs the requirement, payload CRC64, model CRC64, then per-section finiteness
during decode). Dimension equality does not excuse a checksum mismatch —
every check is independently mandatory. The C++ reader decodes fields
explicitly little-endian; it never casts a packed struct. The legacy
`0xBEA0` layout (R2) is retired: files starting with that magic are
rejected with a regeneration instruction.

CRC64-ECMA parameters: polynomial `0x42F0E1EBA9EA3693`, init 0, no
reflection, no final xor; check vector `crc64("123456789") ==
0x6C40DF5F0B497347` (asserted by `native_bench --selftest` and by the C++
suite). The checksums detect corruption and cross-generation mixing —
`state.bin` is bound to its own `model.mjb`; they are an integrity signal,
not a security guarantee.

## Generation and pointer publication

The committed fixtures under `artifacts/` were produced by the deleted
`dump_model.py` writer: it created a new versioned bundle directory under
the artifacts root, wrote the four members from one consistent `mjData`
snapshot, re-read and re-validated every file, and only then published
`current.json` atomically (write-temp + fsync + rename, plus a best-effort
directory fsync). That writer is gone; the bundle format is now read-only
input data and regenerating it requires a new authorized measurement tool
— the committed fixtures are immutable reference data.

Consumers resolve `current.json` exactly once. A malformed, oversized, or
path-escaping pointer is a loud error, never a fallback guess.

## Artifact tracking policy

`artifacts/` tracks the pinned fixture generations (`fixture-v2/`,
`fixture-v3/`, the `current.json` pointer) and the shared `model.mjb`,
which other test suites load directly as their compiled model (it predates
the bundle format and stays for them; bundle consumers always read the
bundle's own `model.mjb`, which `state.bin` is CRC-bound to). Ad-hoc run
output (`run*/`) is gitignored. The flat legacy `state.bin`/`state.npz`
were removed in R3 — they predate the bundle format and can no longer be
read.

## Numerical comparison scope

`--emit-json` reports are the verification surface. The retained C++ suite
checks the workload/operation-count blocks, stage checksums and the total
`checksum` against the committed expectations inside `rtol=1e-12,
atol=1e-12` (integer/scalar stages byte-equal), and `solve_outputs` same
shape, all finite, inside the same tolerance.

`state.bin` replay freezes applied forces (`qfrc_applied`/`xfrc_applied`
are re-staged, not recomputed), so trajectories are a workload fixture, not
a rollout; `steps/s`/`RTF` numbers remain comparative signals on the same
fixture.

## Supported workload domains

Domain preconditions are validated after model load, before any buffer is
sized or any modulo/body indexing runs:

| mode          | requirements |
|---------------|--------------|
| bare          | any model the engine accepts, including an empty model (`nbody == 1`, `nv == 0`) |
| glue          | `nq > 0`, `nv > 0`, `nu > 0`, `nbody > 1` |
| heavy         | glue's requirements plus `nbody >= 11` (attachment pairs resolve to bodies 1..5 and 6..10) and `nv >= 3` |
| operation_mix | same as heavy |

An unsupported model/mode combination exits nonzero with a descriptive
message rather than indexing past a missing body or dividing by a zero
dimension.

## Scope and honest limitations

- The matched workloads share a deterministic definition; the deleted
  paired run never claimed bit-identical output — only solver outputs and
  stage checksums inside the declared `1e-12` tolerance, with
  integer/scalar stages byte-equal.
- `operation_mix` approximates the *shape* of the binding's per-step work
  and is labeled as such; it is not parity evidence.
- All scratch is owned and model-sized (`nv`, `3*nv`, `6*nbody`); there is
  no fixed-size cap such as the prototype's 64-element Jacobian array.
- The Jacobian buffers `jacp`, `jacr`, and `jacp2` are independent — the
  prototype reused one buffer and truncated the second body's Jacobian.
- The CLI is intentionally minimal beyond the report: no statistical
  aggregation; measurement methodology is out of scope.

## Tests

The retained C++ suite generates its own fixtures (slide-joint chains sized
per case, synthetic BIKEST02 images, plus corrupt-model/corrupt-state
corpora) and exercises both the C++ state-format implementation and the
real `native_bench` executable from the same build:

```bash
ctest --test-dir build/native-release -R 'native_benchmark|native_state_bundle|native_state_restore|native_state_fuzz|equilibrium_state_file' --output-on-failure
```

The full native profile — configure, build, all static-analysis sweeps,
structural contracts and CTest — is:

```bash
bash tools/run_tests.sh native
```
