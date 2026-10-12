# Native Runtime and Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:
> executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove known allocations from the owning native core and make benchmark safety and comparison claims
reproducible.

**Architecture:** Construction allocates writer scratch and transactional stage banks; core calls fill spans and typed
telemetry. Python convenience results allocate after leaving core regions. A CMake-owned benchmark validates model-sized
storage and consumes an atomically generated state bundle with explicit workload definitions.

**Tech Stack:** C++23, CMake/CTest, MuJoCo, Accelerate, pytest/NumPy through uv, allocation counters, ASan/UBSan, RTSan.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md), S8.

## Global Constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy
  behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

---

## File map

Create `native/src/writers/writer_types.hpp`, `native/tests/test_allocations.cpp`,
`tools/proto_native_bench/state_format.hpp`, `state_format.py`, and `README.md`; add
`tests/reference/test_native_benchmark.py`. Modify writers, binding boxing, the allocation test registry,
bench.cpp/bench.py/dump_model.py, CMake source manifest, and measurement scripts. This plan does not implement the P4
scheduler or claim the Python API is allocation-free.

Focused commands after writing regressions:

```bash
uv run cmake --build native/build --target native_contract_tests native_bench
uv run ctest --test-dir native/build --output-on-failure
uv run python -m pytest tests/reference/test_native_benchmark.py -q
bash tools/run_tests.sh native
bash tools/run_tests.sh full
```

Before implementation expect warm-core allocation counts, ASan bench overflow, and stale/missing bundle failures.
Afterwards require zero allocation in the declared warm core, clean sanitizers, complete compatible bundles, and finite
comparable workload outputs. Configure the existing NATIVE_RTSAN build and execute the same allocation/core path matrix
through V3's selected-build launcher; NATIVE_SANITIZE covers the malformed benchmark matrix.

### R1: Persistent scratch, typed telemetry, and fixed transaction banks

**Files:** Modify `native/src/writers/drivetrain.hpp/.cpp`, `suspension.hpp/.cpp`, `tire.hpp/.cpp`,
`resistance.hpp/.cpp`, `rider_forces.hpp/.cpp`, `brake.hpp/.cpp`, bindings, `drivetrain/pedaling.hpp/.cpp`, `rtsan.hpp`;
create writer types and allocation tests; modify `tools/measure_realtime.py`, `docs/TESTING.md`.

**Interfaces:**

```cpp
enum class ForceKind { human, motor, transmission, bearing, suspension, tire, rider, resistance };
struct ForceComponentView { ForceKind kind; std::span<const double> values; };
enum class CoreStatus { ok, invalid_input, pending_actuation, invalid_geometry, engine_failure };
```

Every writer gains `compute_into(const mjData*, std::span<double> out)` or `components_into` with its existing scalar
inputs and a span of persistent component views. Existing Python convenience APIs keep their returned shapes/names.
`DriveTelemetry` contains typed fields corresponding one-to-one to the existing diagnostic map; optional fields retain
their presence semantics. Two construction-sized stage banks hold policies, telemetry, controls, forces, and
transmission scratch; successful commit selects the completed bank without allocation.

- [x] Measure current allocations with isolated allocation-test executable, including drivetrain
  ideal/geometric/elastic, advance/probe/settle/reset/restore, both suspension modes, compliant_2d configured/track
  surface modes, engine-only native-reference contacts, rider and resistance. Separate construction, warmup, core and
  Python boxing. Add a native-only count regression targeting zero for the new core interfaces and an RTSan
  invalid-status control.

```cpp
fixture.warmup();
allocation_counter::reset();
auto status = fixture.run_core_tick();
auto count = allocation_counter::count();
require(status == CoreStatus::ok, "valid core tick failed");
require(count == 0, "warm core tick allocated");
```

Define allocation_counter in the isolated test executable alongside E4's allocation override; count
allocation/deallocation and aligned forms. Build all fixture snapshots and output buffers before resetting the counter.
Test repeated ticks and contacts changing within configured capacities.

- [x] Run count controls and retain the measured baseline (representative drivetrain54 is a starting observation, not a
  universal budget). Move rider out (nv), tire qfrc/tmp/effective-radius temporaries, suspension component vectors,
  resistance per-side row, and drivetrain angle scratch into construction-sized members. Fill existing capacity; use
  persistent string IDs/enums inside core. Remove allocating `components()` rebuilding from owning runtime calls.

```cpp
void RiderForcesWriter::compute_into(const mjData *data, std::span<double> out) {
    if (out.size() != static_cast<std::size_t>(nv_))
        throw std::invalid_argument("rider output width");
    std::ranges::fill(out, 0.0);
    accumulate_validated(data, out);
}
```

The checked convenience boundary above remains outside BIKE_NONBLOCKING. A prevalidated internal `try_compute_into`
returns CoreStatus, uses fixed error codes/storage, and contains no throw/string construction. Define
`accumulate_validated(const mjData*, std::span<double>)` by extracting the existing force arithmetic unchanged.

- [x] Replace E3/E4 candidate maps/strings with typed telemetry and fixed stage banks. Give each component a fixed
  name/ID and serialize at FFI export. Preserve insertion order and optional state fields in exported dictionaries. Use
  enum pedaling/shift states in core with string serialization at boundary; retain profile-less custom label storage
  outside the per-tick path. Avoid deep-copying probe state by computing into the inactive bank. Keep the exposed
  persistent prepared-storage identity stable by copying validated bank data into that storage at commit; bank selection
  does not change F4's generation contract.
- [x] Ensure required capacity is allocated before realtime entry; an unexpected extent/capacity change returns status
  and triggers an explicit outside-core rebuild or rejected operation. Engine error bridge returns fixed-buffer status.
  `engaged` allocating exceptions remain outside these regions. Mark only proved kernels/owning core tick, keeping
  convenience validation/boxing outside.
- [x] Run allocation path matrix, native/full golden tests, ASan/UBSan, and selected RTSan tests. Measure p50/p95/p99
  and hardening configuration on the same model/compiler and compare to recorded baseline. Review/commit when warm core
  paths allocate zero and valid outputs remain bitwise equal; identify construction/reset/restore paths intentionally
  outside realtime instead of claiming those are zero-allocation too.
  Status note: matrix/assertions green (core rows 0 alloc/0 freed; RTSan zero reports). p50/p95/p99 measured and
  recorded at `output/realtime-r1/realtime.json` with build provenance; a like-for-like baseline compare is only
  partially satisfiable — no qualified pre-R1 artifact exists (the 2026-10-03 audit used a different SHA, duration,
  and track, and reports realtime_qualified/model_valid false), so this run is retained as the baseline going
  forward. The run itself reports model_valid=false from the pre-existing rider/energy.monitor defects (same family
  as the 24 full-profile failures, reproduced at base fa7a7f0).

### R2: Build and harden the prototype benchmark

**Files:** Modify `tools/proto_native_bench/bench.cpp`, `native/CMakeLists.txt`, `tools/native_checks.py`; create
`tools/proto_native_bench/state_format.hpp`, `README.md`, `tests/reference/test_native_benchmark.py`; extend CTest.

**Interfaces:** CMake target `native_bench`; CLI
`native_bench {bare,glue,heavy} [steps] [rewind] --artifacts <directory>`. Parsing uses `std::from_chars` with
full-consumption checks and positive bounded steps/rewind. `load_state(path, const mjModel&) -> State` verifies
header/payload/model compatibility before allocation/copy. R3 replaces the legacy format reader with the versioned
bundle reader in the next independently reviewable task.

- [x] Test model dimensions nv0,1,33,65,128; nu0; no nonworld bodies; too few heavy attachment bodies; malformed
  steps/rewind, unknown mode, negative/zero rewind, excessive counts, missing/corrupt/truncated state, negative header
  counts, mismatched nq/nv/na/nu/nbody and unchecked makeData failure. Use generated temporary MJB/state files, not the
  tracked artifact as the only fixture.

```python
@pytest.mark.parametrize("rewind", ["0", "-1", "abc", "1x"])
def test_invalid_rewind_is_rejected(bench_executable, artifact_dir, rewind):
    result = subprocess.run([str(bench_executable), "glue", "2", rewind,
                             "--artifacts", str(artifact_dir)],
                            capture_output=True, text=True, check=False)
    assert result.returncode != 0
    assert "rewind" in result.stderr
```

Fixtures select `native_bench` from V3's selected build and generate a supported scalar model with actuators. ASan
controls use nv33 paired and nv65 glue to prove the fixed-array defects are removed.

- [x] Run failing cases under ASan/UBSan. Add the target to first-party source/context manifests, common warning
  options, ISO23/hardening, sanitizer builds, and analysis sweeps. Link E1 engine_call for fatal operations. Replace raw
  FILE/model/data handles with RAII, checking allocation and all read results.
- [x] Allocate srcv as eight nv-sized vectors, acc (nv), jacp/jacr (3 *nv), and separate jacp2 (3*nv) for paired
  Jacobians. Use checked size products and spans. Replace hardcoded 8-byte copies with typed std::copy after exact
  extent validation. Validate workload requirements before modulo/body access: glue requires nq>0,nv>0,nu>0,nbody>1;
  heavy additionally requires all attachment body IDs, or resolves a declared supported attachment list. Bare supports
  an empty model if engine permits it.

```cpp
std::vector<mjtNum> jacp(checked_product(3, nv, max_elements));
std::vector<mjtNum> jacp2(jacp.size());
std::vector<mjtNum> jacr(jacp.size());
```

Set `max_elements` from file/model representable byte extent and actual model dimensions; reject impossible byte-size
multiplication before vector construction. Do not impose the old64 cap.

- [x] Validate legacy header dimensions against loaded model before resizing. Require exact payload length and finite
  required scalars; zero-length optional act/ctrl buffers are valid when the mode permits them. Check every
  model/data/engine-call result through E1; benchmark CLI reports error and returns nonzero without abrupt engine exit.
- [x] Run benchmark regression, CTest, native profile, ASan/UBSan matrix and a valid bare/glue/heavy smoke. Update
  README with CMake build/run and workload limitations; remove obsolete C++17 shell recipe. Review/commit when no
  supported dimension corrupts memory and every unsupported workload fails before any kernel call.

### R3: One state bundle and honest comparable workloads

**Files:** Create `tools/proto_native_bench/state_format.py`; modify `state_format.hpp`, `dump_model.py`, `bench.cpp`,
`bench.py`, README and benchmark tests; remove tracked `tools/proto_native_bench/bench` after replacement build passes;
update benchmark artifact ignore/tracking policy.

**Interfaces:** Bundle directory contains `model.mjb`, `state.npz`, `state.bin`, and `manifest.json`. Generator writes a
new versioned directory, then atomically replaces a small `current.json` pointer; consumers resolve the pointer once, so
mixed-generation files cannot be read. `--artifacts` also accepts a concrete immutable bundle directory for tests.
Header is little-endian, 80 bytes: magic8, version u32, flags u32, nq/nv/na/nu/nbody each u64, payload_bytes u64,
model_crc64 u64, state_crc64 u64. Magic is `BIKEST02`, version2, flags0. Payload is binary64 time, qpos, qvel, act,
ctrl, qfrc_applied, flattened xfrc_applied, qacc_warmstart in that order.

CRC64-ECMA polynomial `0x42F0E1EBA9EA3693`, initial0, no reflection/final xor; test vector `123456789` gives
`0x6C40DF5F0B497347`. This detects corruption/mismatched bundles; it is not an authenticity/security guarantee.
Dimensions and exact payload length remain independently mandatory.

- [x] Write generator/reader roundtrip tests, all dimension mismatches, checksum corruption, unknown version/flags,
  truncated/trailing bytes, invalid little-endian fields, mixed model/state generation, and interrupted pointer
  publication. Compare npz/bin bytes for every array including warmstart. Old BEA0 state format fails with a
  regeneration instruction; R2's intermediate legacy support is retired explicitly.

```python
def test_bundle_arrays_match(tmp_path, scalar_model_and_data):
    model, data = scalar_model_and_data
    bundle = write_bundle(tmp_path, model, data)
    binary = read_state(bundle / "state.bin", bundle / "model.mjb")
    with np.load(bundle / "state.npz") as archive:
        for name in ("qpos", "qvel", "act", "ctrl", "qfrc_applied",
                     "xfrc_applied", "qacc_warmstart", "time"):
            assert binary[name].tobytes() == archive[name].tobytes()
```

Define `write_bundle(root: Path, model, data) -> Path`,
`read_state(state_path: Path, model_path: Path) -> dict[str,np.ndarray]` in state_format.py; C++ reader mirrors exact
fixed layout without casting a packed struct. Require `8*(1+nq+3*nv+na+nu+6*nbody)` payload bytes, checked for overflow.

- [x] Run tests showing dump_model currently omits state.bin. Implement both readers/writer and CRC algorithm with
  published test vector; generator captures one immutable snapshot, serializes all files, validates them, then publishes
  current.json. Restore warmstart and forces identically in both benches. Record model dimensions, timestep, source
  config/track, warmup steps, generator commit and checksums in manifest.
- [x] Write paired workload conformance tests: same warmup/rewind phases, accumulation counts and ordering, name list,
  actuator-to-velocity mapping, channels/checksum terms, attachment Jacobians, linear solves, regularization, ledger and
  batch schedule. Use deterministic shared workload JSON containing mode parameters and fixed seeds. `nu` may exceed nv:
  use actuator moment projection or an explicit supported mapping instead of slicing qvel[:nu] or modulo approximations.

For a comparable mode, use the same supported BLAS reduction/solve sequence in both languages, with `1e-6*I`
regularization for attachment normal matrices and the same rank/least-squares policy. Emit operation counts and checksum
contributions by stage. Require finite results and compare with a declared tolerance for solver outputs (`rtol=1e-12`,
`atol=1e-12`), while exact scalar-stage expectations stay byte-equal. Any workload whose operations cannot match is
named `operation_mix` and carries no parity/tax-isolation claim.

```python
assert np.isfinite(native_result["checksum"])
assert native_result["operation_counts"] == python_result["operation_counts"]
np.testing.assert_allclose(native_result["solve_outputs"], python_result["solve_outputs"],
                           rtol=1e-12, atol=1e-12)
```

- [x] Implement identical warmup/measurement boundaries, separate checksum accumulation from timed output serialization,
  and report compiler/hardening/model/bundle provenance. Keep baseline operation-mix results separate from matched
  bare/glue/heavy results. Test singular/near-singular attachment models return finite regularized results or explicit
  unsupported status, never a successful NaN sink.
- [x] Run bundle/workload tests, native profile, sanitizer benchmark controls and both-language smoke via
  `uv run python tools/proto_native_bench/bench.py`. Remove the platform binary with
  `git rm tools/proto_native_bench/bench` only at implementation time, after CMake replacement is reproducible.
  Review/commit when regenerated artifacts are compatible, pointer publication is atomic, checksums are finite, and
  documentation accurately scopes comparison claims.
