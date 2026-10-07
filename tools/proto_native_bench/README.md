# proto_native_bench — paired native benchmark

`bench.cpp` and `bench.py` are the paired C++/Python implementations of the
bike_native binding workload (native-safety runtime plan, tasks R2+R3). Both
replay one recorded `mjData` snapshot through `mj_step` under a shared,
deterministic workload definition, and both emit the same JSON report so the
operation mix and its checksums can be compared across languages. Every
native engine call crosses the `bike_native_engine` boundary (E1): MuJoCo
warnings/fatals arrive as recoverable `engine::EngineFailure` exceptions, and
the CLI reports failures and exits nonzero — it never takes MuJoCo's
process-exit path.

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
the contract test binary uses. The checked-in `bench` binary of the pre-CMake
prototype was removed in R3 — the CMake target is the only supported build.

Sanitizer variants are ordinary `native` build flavors, so the benchmark is
reachable under ASan/UBSan, RTSan, and coverage builds by configuring the
corresponding options (`-DNATIVE_SANITIZE=ON` etc.) — the target inherits the
module's flags.

## Run

```text
native_bench {bare,glue,heavy,operation_mix} [steps] [rewind]
             --artifacts <directory> [--emit-json <path>]
bench.py     {bare,glue,heavy,operation_mix} [steps] [rewind]
             --artifacts <directory> [--emit-json <path>]
```

- `bare` — `mj_step` only, on a frozen replayed state (periodic state
  rewind).
- `glue` — bare + the engine-call surface of the Python glue: forward passes,
  point Jacobians, `mj_name2id` lookups, eight `qfrc_applied` assemblies
  driven through the engine-projected `qfrc_actuator` mapping, and 100
  channel writes per step.
- `heavy` — glue + representative telemetry/accounting volume: an `efc_*` row
  unpack, five attachment wrench solves over paired Jacobians, an energy
  ledger, per-step record churn, and a period-batch gradient-free solve.
- `operation_mix` — the retained R2 approximation mix. It exercises roughly
  the same engine surface but follows its own (deliberately different)
  arithmetic and carries **no parity claim**: its report is labeled
  `"comparison": "operation_mix"` and its numbers must not be quoted as
  Python-vs-C++ conformance evidence.

`bare`, `glue`, and `heavy` are the matched workloads: the two languages run
the same warmup/rewind phases, the same stage order, the same operation
counts, the same attachment Jacobians and `1e-6 * I`-regularized 3x3 solve
sequence, the same `qfrc_actuator` actuator-to-velocity mapping (correct when
`nu > nv`), and the same SplitMix64-driven channels from a fixed seed.

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
checksum. This is the document the conformance test compares.

Example against the tracked artifacts:

```bash
./native/build/native_bench glue 5 --artifacts tools/proto_native_bench/artifacts
uv run python tools/proto_native_bench/bench.py glue 5 \
    --artifacts tools/proto_native_bench/artifacts
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

Reader contract (both languages, same order): size floor, magic, version,
flags, u64 field ranges vs the signed mjtSize domain, dimension equality
against the loaded model, declared `payload_bytes` vs the recomputed
requirement, actual file length vs the requirement, payload CRC64, model
CRC64, then per-section finiteness during decode. Dimension equality does not
excuse a checksum mismatch — every check is independently mandatory. The C++
reader decodes fields explicitly little-endian; it never casts a packed
struct. The legacy `0xBEA0` layout (R2) is retired: files starting with that
magic are rejected with a regeneration instruction.

CRC64-ECMA parameters: polynomial `0x42F0E1EBA9EA3693`, init 0, no
reflection, no final xor; check vector `crc64("123456789") ==
0x6C40DF5F0B497347` (asserted by `native_bench --selftest` and by the Python
suite). The checksums detect corruption and cross-generation mixing — `state.bin`
is bound to its own `model.mjb`; they are an integrity signal, not a
security guarantee.

## Generation and pointer publication

`dump_model.py` (or `state_format.write_bundle` directly) creates a NEW
versioned bundle directory under the artifacts root, writes the four members
from one consistent `mjData` snapshot, re-reads and re-validates every file,
and only then publishes `current.json` atomically (write-temp + fsync +
rename, plus a best-effort directory fsync). A failed generation removes the
partial directory and never touches the pointer, so a consumer can never
observe a mid-write bundle and files from different generations can never be
mixed. Bundle directories are immutable — reusing a name fails.

Consumers resolve `current.json` exactly once. A malformed, oversized, or
path-escaping pointer is a loud error, never a fallback guess.

Regenerate the tracked fixture with:

```bash
uv run python tools/proto_native_bench/dump_model.py --name fixture-v2
```

(see `dump_model.py --help` for track/config/warmup/duration overrides).

## Artifact tracking policy

`artifacts/` tracks exactly one fixture generation — currently
`fixture-v2/` plus the `current.json` that points at it — and the shared
`model.mjb`, which other test suites load directly as their compiled model
(it predates the bundle format and stays for them; bundle consumers always
read the bundle's own `model.mjb`, which `state.bin` is CRC-bound to).
Auto-generated bundle directories (`v2-*`) and ad-hoc run output (`run*/`)
are gitignored — a bundle is regenerable, so the repository does not
accumulate binary blobs. To swap the fixture: generate a new directory,
repoint `current.json`, commit both paths, and drop the old directory. The
flat legacy `state.bin`/`state.npz` were removed in R3 — they predate the
bundle format and can no longer be read.

## Numerical comparison scope

`--emit-json` reports are the conformance surface. For matched modes the
suite requires:

- identical `workload` and `operation_counts` blocks;
- `name_ids` and `trajectory_time` stage checksums exactly equal;
- every other stage checksum and the total `checksum` inside
  `rtol=1e-12, atol=1e-12`;
- `solve_outputs` — the regularized attachment/batch solve results — same
  shape, all finite, inside `rtol=1e-12, atol=1e-12`.

Remaining legitimate divergence is platform libm noise (for example the
energy ledger's BLAS ordering), bounded well inside the declared tolerance.
`operation_mix` is excluded from all of this by definition.

`state.bin` replay freezes applied forces (`qfrc_applied`/`xfrc_applied` are
re-staged, not recomputed), so trajectories are a workload fixture, not a
rollout; `steps/s`/`RTF` numbers remain comparative signals on the same
fixture, not cross-language timing parity — the *workload* is shared, the
wall clock is not.

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

- The matched workloads share a deterministic definition; they do not claim
  bit-identical output — only the solver outputs and stage checksums inside
  the declared `1e-12` tolerance, with integer/scalar stages byte-equal.
- `operation_mix` approximates the *shape* of the binding's per-step work and
  is labeled as such; it is not parity evidence.
- All scratch is owned and model-sized (`nv`, `3*nv`, `6*nbody`); there is no
  fixed-size cap such as the prototype's 64-element Jacobian array.
- The Jacobian buffers `jacp`, `jacr`, and `jacp2` are independent — the
  prototype reused one buffer and truncated the second body's Jacobian.
- Nothing here measures or claims allocation-freedom for the Python API —
  the owned, model-sized scratch lives inside this executable only.
- The CLI is intentionally minimal beyond the conformance report: no
  statistical aggregation; measurement methodology is out of scope.

## Tests

The regression suites generate their own fixtures (slide-joint chains sized
per case, synthetic BIKEST02 images, plus corrupt-model/corrupt-state
corpora) and exercise both the Python format module and the selected build's
`native_bench`:

```bash
uv run pytest tests/reference/test_native_benchmark.py \
              tests/reference/test_native_state_bundle.py -x -q
```

`test_native_state_bundle.py` covers the writer/reader/pointer contract in
Python (roundtrip, NPZ↔bin byte equality including the warmstart, header and
checksum corruption, truncation/trailing bytes, dimension and CRC mismatch,
mixed-generation and interrupted-publication rejection, and the CRC64 check
vector). `test_native_benchmark.py` covers the CLI, the C++ reader through
the binary, the artifact/pointer resolution, the emitted JSON, and the
paired Python/C++ matched-workload conformance.

The tests honor `NATIVE_TEST_BUILD_PATH` / `NATIVE_TEST_BUILD_DIR` like every
other `test_native_*` module, so the same files cover the ASan build
(`NATIVE_TEST_BUILD_DIR=asan`). The full native profile — build, all static
analysis sweeps, CTest (which runs the smoke invocations), contract checks,
and these test modules — is:

```bash
bash tools/run_tests.sh native
```
