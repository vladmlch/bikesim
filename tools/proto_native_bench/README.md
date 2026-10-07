# proto_native_bench — prototype native benchmark

`bench.cpp` is the hardened C++ prototype of the bike_native binding workload
(native-safety runtime plan, task R2). It replays a recorded `mjData` snapshot
through `mj_step` under three selectable mixes. Every engine call crosses the
`bike_native_engine` boundary (E1): MuJoCo warnings/fatals arrive as
recoverable `engine::EngineFailure` exceptions, and the CLI reports failures
and exits nonzero — it never takes MuJoCo's process-exit path.

## Build

The benchmark is a plain CMake target inside the existing `native` project
(strict ISO C++23, extensions disabled, same warning/hardening/sanitizer
context as the extension targets):

```bash
cmake -S native -B native/build          # once; tools/run_tests.sh native does this too
cmake --build native/build --target native_bench
```

The binary lands at `native/build/native_bench` and the post-build step fixes
its `libmujoco` install name against the pinned Python wheel, the same recipe
the contract test binary uses.

Sanitizer variants are ordinary `native` build flavors, so the benchmark is
reachable under ASan/UBSan, RTSan, and coverage builds by configuring the
corresponding options (`-DNATIVE_SANITIZE=ON` etc.) — the target inherits the
module's flags.

## Run

```text
native_bench {bare,glue,heavy} [steps] [rewind] --artifacts <directory>
```

- `bare` — `mj_step` only, on a frozen replayed state (periodic state rewind).
- `glue` — bare + the engine-call surface of the Python glue: forward passes,
  point Jacobians, `mj_name2id` lookups, eight `qfrc_applied` assemblies, and
  100 channel writes per step.
- `heavy` — glue + representative telemetry/accounting volume: an `efc_*` row
  unpack, five attachment wrench solves over paired Jacobians, an energy
  ledger, per-step record churn, and a period-batch gradient-free solve.

`steps` (default 4000) is the timed iteration count and `rewind` (default
250) is the state-restore period; both must be integers in
`[1, 10000000]` and are parsed with `std::from_chars` under a
full-consumption contract — malformed values, unknown modes, stray options,
and extra positionals are errors (nonzero exit, message on stderr). `-h` /
`--help` prints usage. `--artifacts` is required and names a directory
containing `model.mjb` and `state.bin`.

Example against the tracked artifacts:

```bash
./native/build/native_bench bare 10 --artifacts tools/proto_native_bench/artifacts
./native/build/native_bench glue 5 --artifacts tools/proto_native_bench/artifacts
./native/build/native_bench heavy 3 --artifacts tools/proto_native_bench/artifacts
```

## Artifacts

`artifacts/` holds the tracked inputs (`model.mjb`, `state.bin`, `state.npz`)
produced by `dump_model.py`. The checked-in `bench` binary is the pre-CMake
prototype artifact and is superseded by the `native_bench` target.

`state.bin` is the legacy 0xBEA0 layout (`state_format.hpp`):

```text
i32 magic = 0xBEA0
i32 nq, i32 nv, i32 na, i32 nu, i32 nbody
f64 time
f64 qpos[nq] | qvel[nv] | act[na] | ctrl[nu]
f64 qfrc_applied[nv] | xfrc_applied[6*nbody]
```

The reader validates the header (magic, nonnegative counts) **and** dimension
equality against the loaded model before any storage is sized; the payload
must be exactly `8 * (1 + nq + nv + na + nu + nv + 6*nbody)` bytes — no
truncation, no trailing bytes — and every stored scalar must be finite.
Known limitations of this format: it carries no version, no checksum, and no
numeric tolerances metadata, so a structurally valid but semantically wrong
state still loads. Task R3 replaces it with the versioned bundle format; keep
the format reader confined to `state_format.hpp` until then. There is no
committed `state.bin` generator — the file is produced by the prototype
capture script, which is why tests generate fixtures themselves.

## Supported workload domains

Domain preconditions are validated after model load, before any buffer is
sized or any modulo/body indexing runs:

| mode  | requirements |
|-------|--------------|
| bare  | any model the engine accepts, including an empty model (`nbody == 1`, `nv == 0`) |
| glue  | `nq > 0`, `nv > 0`, `nu > 0`, `nbody > 1` |
| heavy | glue's requirements plus `nbody >= 11` (attachment pairs resolve to bodies 1..5 and 6..10) and `nv >= 3` |

An unsupported model/mode combination exits nonzero with a descriptive
message rather than indexing past a missing body or dividing by a zero
dimension.

## Scope and honest limitations

This is a **prototype operation mix**, not a port of the Python glue. The
heavy mode approximates the *shape* of the binding's per-step work (counts of
Jacobian calls, lookups, record churn) — it does not reproduce the glue's
exact arithmetic, and `bench.cpp` and `bench.py` differ in detail. Treat
`steps/s`/`RTF` numbers as comparative workload signals between `bare`,
`glue`, and `heavy` on the same fixture; do not quote them as Python-vs-C++
parity claims — the operation mixes intentionally differ. The `sink` value
keeps the optimizer honest; it is not a numerical oracle.

Other scope notes:

- All scratch is owned and model-sized (`nv`, `3*nv`, `6*nbody`); there is no
  fixed-size cap such as the prototype's 64-element Jacobian array.
- Nothing here measures or claims allocation-freedom for the Python API — the
  owned, model-sized scratch lives inside this executable only; the Python
  binding's allocation behavior is a separate contract.
- The Jacobian buffers `jacp`, `jacr`, and `jacp2` are independent — the
  prototype reused one buffer and truncated the second body's Jacobian.
- `state.bin` replay freezes applied forces (`qfrc_applied`/`xfrc_applied`
  are re-staged, not recomputed), so trajectories are a workload fixture, not
  a rollout.
- The CLI is intentionally minimal: there is no JSON/report output and no
  statistical aggregation — measurement methodology is out of scope here.

## Tests

The regression suite generates its own fixtures (a slide-joint chain sized
per case plus a corrupt-model/truncated-state corpus) and exercises the
selected build's `native_bench`:

```bash
uv run pytest tests/reference/test_native_benchmark.py -x -q
```

It honors `NATIVE_TEST_BUILD_PATH` / `NATIVE_TEST_BUILD_DIR` like every other
`test_native_*` module, so the same file covers the ASan build
(`NATIVE_TEST_BUILD_DIR=asan`). The full native profile — build, all static
analysis sweeps, CTest (which runs the three smoke invocations), contract
checks, and this test module — is:

```bash
bash tools/run_tests.sh native
```
