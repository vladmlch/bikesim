# GPU Golden Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Read `AGENTS.md` and `docs/agents/native-engineering.md` before any native task.

**Goal:** Verify that the Scenario 2 GPU path (warp-metal + mujoco_warp + MSL writers) computes the bike's tyre, suspension and resistance physics like the production CPU runtime, and let a human see every divergence in the native viewer.

**Architecture:** The production CPU runtime records golden runs (`bike-ride --record`). A native exporter turns a recording into a model bundle the Python GPU stack can load. A Python runner replays each golden run on `metal:0` in 0.1 s windows. At every window start it re-seats only `qpos`/`qvel` from the golden run. The GPU computes tyres, suspension and resistance itself; every other force component is injected from the recording. A report compares end-of-window states and per-component forces. `bike-replay --view --trajectory` shows the GPU trajectory with the CPU golden run as a translucent ghost, a Δ HUD and tyre-force arrows.

**Tech Stack:** C++23 native tree (CMake, CTest), MSL via `native/src/gpu/metal_batch_runner.mm`, Python through `uv` (`warp-metal` 1.17.0, `mujoco_warp` 3.11.0, `mujoco` 3.17.0 in `.venv-metal`; native libmujoco is 3.12.0), DuckDB (Python `duckdb` package to read recordings).

**Spec:** §Decisions below. They were settled in the 2026-11-02 grilling session; no separate spec file exists.

## Decisions (the spec)

| ID | Decision |
|---|---|
| D1 | The thing under test is the GPU path itself, not the bike simulation in general. |
| D2 | Golden = production native CPU recordings (`bike-ride --record`). `cpu_batch_runner` is only an auxiliary diagnostic. |
| D3 | Physics config: `examples/research/viewer_physics_distributed.toml`. Its tyre backend `distributed_2d_reference` is the one the GPU tyre kernel mirrors. The rider is attached exactly as in `welded` (saddle pin, spindle foot connects, grip connects); MuJoCo's solver enforces these on both sides. |
| D4 | Tracks: `flat` (built-in preset) and `examples/research/rough_uphill_extreme.toml`. |
| D5 | Timestep 0.00125 s on both sides. `nworld = 1`. Golden run length is the track end or 30 s, whichever comes first. |
| D6 | The GPU computes tyres, suspension and resistance. All other recorded force components (rider, drivetrain, motor, brakes, seated/rider interfaces, rider joint envelope, …) are injected from the recording at each step. |
| D7 | Comparison uses windows (default 0.1 s, configurable). At each window start only `qpos`/`qvel` are re-seated from the golden run. GPU writer internal state (bristles, freehub, damper) carries over from the previous window. A start-vs-end-of-window force metric shows whether that memory matters. If it does, adding writer state to recordings is a separate task. |
| D8 | The report has per-window Δ, worst windows located on the track, per-component force differences, the first window exceeding 1e-6, 1e-3 and 1e-2, and whole-run summary metrics from a free (unsynchronised) run. There is no pass/fail threshold; Task 20 of `2026-11-01-gpu-gate-compliance.md` chooses one from these numbers. |
| D9 | Visualisation is the native `bike-replay` with a new honest input: model + trajectory, no forged sealed container. It shows the ghost and Δ in one window plus tyre-force arrows. mp4 export is out of scope. |
| D10 | The GPU recorder is Python (mujoco_warp is Python-only) and lives under `tools/colab_gpu/golden/`. It is committed. The plan records in writing that `tools/colab_gpu/` survives T63. |
| D11 | Fix the GPU tyre parameters (radius, μ, density, station count) so they come from the recorded config. First run one baseline before the fix to prove the tool sees the mismatch. |
| D12 | A headless batch-identity check: `nworld = N` with identical input must reproduce world 0 bitwise. |
| D13 | Task 20 Step 3 of `2026-11-01-gpu-gate-compliance.md` is rewritten to use this tool. |
| Out | Out of scope: the pneumatic tyre on the GPU, rider/motor controllers on the GPU, the savage track, and comparison against production writers without injection. |

## Global Constraints

- Python always runs through `uv`. GPU tests use the warp-metal interpreter: `uv run --no-project --python .venv-metal/bin/python python -m pytest …`. CPU-only tests use `uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.17.0 python -m pytest …`.
- Native verification:
  - iterate on targeted CTest nodes;
  - the completion gate is `bash tools/run_tests.sh native` exiting 0, run once at the end;
  - GUI code also builds under `cmake --preset gui && cmake --build build/gui`.
- Never hand-forge a sealed recording container (`docs/development/viewer-platform-acceptance.md`: "do not manufacture a legacy export").
- Do not modify the existing benchmark kernels' behaviour: `bike_wp_writers_metal_kernel`, `bike_wp_writers_simd32_kernel` and the `bike_gpu_*` / `bike_metal_*` kernels. Add new entry points.
- Golden recordings and run outputs go under `output/gpu_golden/` (gitignored). They are never committed.
- FP32 on the GPU and FP64 on the CPU are expected to diverge slowly. Do not "fix" divergence by loosening tolerances inside writers.
- Commit subject style follows the repo log (`feat(...)`, `fix(...)`, `docs(...)`). Every commit ends with the attribution trailer required by the session.

## Facts this plan relies on (verified 2026-11-02 by reading code)

- `native/src/gpu/common.hpp:61`: `kMaxTreadStations = 64`. The CPU distributed config uses `station_count = 128` (`native/src/configuration/physical_config.hpp:70`).
- `native/src/gpu/tire_kernel.hpp:15-37`: the GPU defaults are knots `{0, .008, .018}`, stiffness `{55000, 85000, 140000}`, damping 420, radius 0.351, μ 0.95/0.85. The CPU `HingeDensity` defaults (`physical_config.hpp:53-57`) are knots `{0, .004, .012}`, stiffness `{300000, 0, 0}`, damping 2000. They differ completely.
- `native/src/gpu/batch_step.hpp:126-221`: `gpu_step_world_writers` zeroes and overwrites `qfrc` and `ctrl` and runs suspension → tyres → drivetrain+resistance → rider.
- Native recordings store, at every physics step, `physics(time_s, interval_end_s, dt_s, qpos, qvel)` and `forces(component, qfrc[nv], power_w)`. Component names come from `native/src/runtime/step.cpp:1275-1374` and include `tires`, `suspension`, `resistance`, `brake_hold`, `external`, `seated_interfaces`, `rider_interfaces` and `rider_joint_envelope`. `mjData.ctrl` and writer internal state are **not** recorded.
- `replay::load_playback` (`native/src/replay/playback.cpp:113`) takes the model from the `model.mjb` asset or rebuilds it from `recipe.json` via `reconstruct_model` (`playback.cpp:31-55`, MJCF from `model::generate_mjcf` + heightfield from `terrain::build_field_data`).
- The Python stack is mujoco 3.17.0 and native libmujoco is 3.12.0, so a native `model.mjb` cannot be assumed loadable in Python. The bundle therefore carries MJCF + `hfield_data.bin`, the same layout `tools/colab_gpu/model_assets.py:load_model_with_assets` already loads.
- The viewer frame loop is `native/src/viewer/ride.cpp:140-146` (`renderer->apply(frame)` → `set_overlay` → `draw`). `RenderOwner::draw` (`native/src/viewer/render_owner.cpp:111-139`) calls `mjv_updateScene` and then `mjr_render`.

## Known risks resolved by gates

- **R1 (Task 3 gate).** The CPU drives brakes, drive and rider through `ctrl` (actuators), may mutate model fields per step (`tendon_range` for the freehub, `RenderOwner` field list `render_owner.cpp:15-16`) and may toggle equalities. If recorded `forces` alone do not reproduce the CPU step, injection is unsound. The closure test proves or refutes this before any GPU work.
- **R2 (Task 8 gate).** If the GPU tyre law, not just its parameters, differs from `writers/distributed_tire.cpp`, a parameter fix is not enough. That becomes a re-port, a new plan.

## File structure

| Path | Responsibility |
|---|---|
| `tools/colab_gpu/golden/__init__.py` | Package marker. |
| `tools/colab_gpu/golden/recording.py` | Read a sealed native recording (physics + forces) into numpy. |
| `tools/colab_gpu/golden/bundle.py` | Load the exporter bundle into a `mujoco.MjModel`. |
| `tools/colab_gpu/golden/closure.py` | CPU FP64 closure gate: does `qfrc_applied = Σ recorded components` reproduce the next recorded state? |
| `tools/colab_gpu/golden/gpu_runner.py` | Windowed / free GPU replay on `metal:0` with injection. |
| `tools/colab_gpu/golden/trajectory.py` | Write the external trajectory directory read by `bike-replay --trajectory`. |
| `tools/colab_gpu/golden/compare.py` | Build `report.json` / `report.md` from golden + GPU run. |
| `tools/colab_gpu/golden/cli.py` | `record-goldens`, `closure`, `run`, `report` subcommands. |
| `tools/colab_gpu/golden/tests/*.py` | pytest suites (CPU-only and Metal-marked). |
| `native/src/replay/model_bundle.{hpp,cpp}` | Build and write the model bundle from a recording. |
| `native/src/tools/gpu_export_main.cpp` | `bike-gpu-export` executable. |
| `native/src/gpu/golden_params.hpp` | Host-side `GpuWorldConfig` from the recorded physics config (Task 8). |
| `native/src/gpu/batch_step.hpp` | + `gpu_step_world_writers_golden`. |
| `native/src/gpu/drivetrain_resistance_kernel.hpp` | + `gpu_write_resistance_only_qfrc`. |
| `native/src/gpu/common.hpp` | `kMaxTreadStations = 128`. |
| `native/src/gpu/metal_batch_runner.mm` | + `bike_wp_writers_golden_kernel`. |
| `native/src/replay/trajectory.{hpp,cpp}` | Load the external trajectory into `PlaybackData`. |
| `native/src/replay/golden_overlay.{hpp,cpp}` | Pure function: ghost pose, Δ text and arrows for a time. |
| `native/src/viewer/decorations.hpp` | GL-free `Decorations` value type. |
| `native/src/viewer/session.hpp`, `ride.cpp`, `render_owner.{hpp,cpp}` | Hook decorations into the frame loop and scene. |
| `native/src/cli/replay.cpp`, `argument_contracts.inc` | `--trajectory` flag. |

---

### Task 1: Golden recordings and the Python recording reader

**Files:**
- Create: `tools/colab_gpu/golden/__init__.py`, `tools/colab_gpu/golden/recording.py`, `tools/colab_gpu/golden/cli.py`, `tools/colab_gpu/golden/tests/__init__.py`, `tools/colab_gpu/golden/tests/conftest.py`, `tools/colab_gpu/golden/tests/test_recording.py`
- Modify: `docs/superpowers/plans/2026-10-20-cpp-linux-completion.md` (near OD-1 at `:55`, and T63 Step 1 at `:4775-4779`)

**Interfaces:**
- Produces:
  - `GoldenEpisode` (frozen dataclass) with fields:
    - `times_s: np.ndarray[(n,)]`, `ends_s: np.ndarray[(n,)]`, `dt_s: float`;
    - `qpos: np.ndarray[(n, nq)]`, `qvel: np.ndarray[(n, nv)]`;
    - `components: tuple[str, ...]` (sorted);
    - `qfrc: np.ndarray[(n, ncomp, nv)]`.
  - `load_golden(recording_dir: Path, episode_id: int = 0) -> GoldenEpisode`.
  - `GPU_COMPONENTS: tuple[str, str, str] = ("suspension", "tires", "resistance")`.
  - `injected_qfrc(ep: GoldenEpisode) -> np.ndarray[(n, nv)]`.
  - `component_qfrc(ep, name) -> np.ndarray[(n, nv)]`.

- [ ] **Step 1: Build the release binaries and record the two goldens**

```bash
cmake --workflow --preset release   # produces build/native-release/bin/bike-ride
mkdir -p output/gpu_golden
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_distributed.toml \
  --track flat --headless --duration 30 --check-policy strict --record --out output/gpu_golden/flat
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_distributed.toml \
  --track examples/research/rough_uphill_extreme.toml --headless --duration 30 --check-policy strict \
  --record --out output/gpu_golden/rough_uphill_extreme
```

Expected: each command exits 0 and prints a published recording directory (it contains `manifest.json` and `run.duckdb`). Record the exact published directory paths, the run end reason and the simulated duration in the task report.
- If `strict` aborts, rerun with `--check-policy diagnostic` and record the first failure reported by the run. A diagnostic recording is still a valid golden for this tool, because the tool compares the CPU's actual output.
- If `--out` is not the accepted flag name, read `native/src/cli/recording.cpp` for the output option and use that.

- [ ] **Step 2: Verify the goldens are readable by the existing playback path**

```bash
build/native-release/bin/bike-replay <published-flat-dir> --view --dry-run
build/native-release/bin/bike-replay <published-extreme-dir> --view --dry-run
```

Expected: `frames=… duration=…s ok` for both.

- [ ] **Step 3: Write the failing reader test**

`tools/colab_gpu/golden/tests/conftest.py`:

```python
"""Shared fixtures: golden recordings are local artifacts under output/gpu_golden/."""
from __future__ import annotations

import os
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]


def _published(track: str) -> Path:
    # The env var lets a run point at an explicit published directory.
    env = os.environ.get(f"BIKE_GOLDEN_{track.upper()}")
    if env:
        return Path(env)
    root = REPO_ROOT / "output" / "gpu_golden" / track
    candidates = sorted(p.parent for p in root.rglob("manifest.json"))
    if not candidates:
        pytest.skip(f"no golden recording under {root}; run Task 1 Step 1")
    return candidates[-1]


@pytest.fixture(scope="session")
def golden_flat_dir() -> Path:
    return _published("flat")


@pytest.fixture(scope="session")
def golden_extreme_dir() -> Path:
    return _published("rough_uphill_extreme")
```

`tools/colab_gpu/golden/tests/test_recording.py`:

```python
"""The reader returns every physics step with aligned force components."""
from __future__ import annotations

import numpy as np

from tools.colab_gpu.golden.recording import GPU_COMPONENTS, component_qfrc, injected_qfrc, load_golden


def test_reader_aligns_physics_and_forces(golden_flat_dir):
    """Every interval has qpos/qvel and one qfrc row per component; times step by dt."""
    ep = load_golden(golden_flat_dir)
    n = ep.times_s.shape[0]
    assert n > 100
    assert ep.qpos.shape[0] == n and ep.qvel.shape[0] == n
    assert ep.qfrc.shape == (n, len(ep.components), ep.qvel.shape[1])
    assert np.all(np.diff(ep.times_s) > 0)
    np.testing.assert_allclose(np.diff(ep.times_s), ep.dt_s, rtol=0, atol=1e-12)
    assert ep.dt_s == 0.00125


def test_reader_exposes_gpu_components(golden_flat_dir):
    """The three GPU-computed components exist and injection is the sum of all others."""
    ep = load_golden(golden_flat_dir)
    for name in GPU_COMPONENTS:
        assert name in ep.components, ep.components
    total = ep.qfrc.sum(axis=1)
    own = sum(component_qfrc(ep, name) for name in GPU_COMPONENTS)
    np.testing.assert_allclose(injected_qfrc(ep) + own, total, rtol=0, atol=1e-9)


def test_reader_rejects_unsealed(tmp_path):
    """A directory without a sealed manifest is refused, never guessed at."""
    import pytest

    (tmp_path / "run.duckdb").write_bytes(b"")
    with pytest.raises(ValueError, match="sealed"):
        load_golden(tmp_path)
```

- [ ] **Step 4: Run it to verify it fails**

Run: `uv run --no-project --with pytest --with numpy --with duckdb python -m pytest tools/colab_gpu/golden/tests/test_recording.py -q`
Expected: FAIL with `ModuleNotFoundError: tools.colab_gpu.golden.recording`.

- [ ] **Step 5: Implement the reader**

`tools/colab_gpu/golden/__init__.py`:

```python
"""Golden CPU recording → GPU (warp-metal + mujoco_warp) replay and comparison."""
```

`tools/colab_gpu/golden/recording.py`:

```python
"""Read a sealed native recording (docs/RECORDING.md) into numpy arrays.

Only `physics` and `forces` are read. The container seal is checked first:
manifest schema 1, sealed, and the run.duckdb SHA-256 recorded in it
(native/src/recording/reader.cpp:29-79 is the authoritative check).
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Components the GPU computes itself (D6). Everything else is injected.
GPU_COMPONENTS: tuple[str, str, str] = ("suspension", "tires", "resistance")


@dataclass(frozen=True)
class GoldenEpisode:
    times_s: np.ndarray
    ends_s: np.ndarray
    dt_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: tuple[str, ...]
    qfrc: np.ndarray  # (n, ncomp, nv), component order == self.components


def _require_sealed(recording_dir: Path) -> None:
    manifest_path = recording_dir / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError(f"{recording_dir}: not a sealed recording (no manifest.json)")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != 1 or manifest.get("sealed") is not True:
        raise ValueError(f"{recording_dir}: manifest is not a sealed schema-1 recording")
    expected = manifest.get("file_sha256")
    actual = hashlib.sha256((recording_dir / "run.duckdb").read_bytes()).hexdigest()
    if expected != actual:
        raise ValueError(f"{recording_dir}: run.duckdb SHA-256 differs from the sealed manifest")


def load_golden(recording_dir: Path, episode_id: int = 0) -> GoldenEpisode:
    import duckdb

    recording_dir = Path(recording_dir)
    _require_sealed(recording_dir)
    con = duckdb.connect(str(recording_dir / "run.duckdb"), read_only=True)
    try:
        rows = con.execute(
            "SELECT interval_id, time_s, interval_end_s, dt_s, qpos, qvel FROM physics "
            "WHERE episode_id = ? ORDER BY interval_id",
            [episode_id],
        ).fetchall()
        if not rows:
            raise ValueError(f"{recording_dir}: episode {episode_id} has no physics rows")
        interval_ids = np.array([r[0] for r in rows], dtype=np.int64)
        times = np.array([r[1] for r in rows], dtype=np.float64)
        ends = np.array([r[2] for r in rows], dtype=np.float64)
        dts = np.unique(np.array([r[3] for r in rows], dtype=np.float64))
        if dts.size != 1:
            raise ValueError(f"{recording_dir}: variable dt_s {dts}")
        qpos = np.array([r[4] for r in rows], dtype=np.float64)
        qvel = np.array([r[5] for r in rows], dtype=np.float64)

        frows = con.execute(
            "SELECT interval_id, component, qfrc FROM forces WHERE episode_id = ? "
            "ORDER BY interval_id, component",
            [episode_id],
        ).fetchall()
    finally:
        con.close()

    components = tuple(sorted({r[1] for r in frows}))
    col = {name: i for i, name in enumerate(components)}
    row_of = {int(iid): i for i, iid in enumerate(interval_ids)}
    nv = qvel.shape[1]
    qfrc = np.zeros((times.shape[0], len(components), nv), dtype=np.float64)
    seen = np.zeros((times.shape[0], len(components)), dtype=bool)
    for iid, name, vec in frows:
        i = row_of.get(int(iid))
        if i is None:
            raise ValueError(f"forces row for unknown interval {iid}")
        qfrc[i, col[name]] = np.asarray(vec, dtype=np.float64)
        seen[i, col[name]] = True
    # A component absent in an interval contributed nothing; keep zeros but
    # refuse an interval with no force rows at all (a reader bug, not physics).
    if not seen.any(axis=1).all():
        missing = int(np.flatnonzero(~seen.any(axis=1))[0])
        raise ValueError(f"interval index {missing} has no forces rows")
    return GoldenEpisode(times, ends, float(dts[0]), qpos, qvel, components, qfrc)


def component_qfrc(ep: GoldenEpisode, name: str) -> np.ndarray:
    return ep.qfrc[:, ep.components.index(name), :]


def injected_qfrc(ep: GoldenEpisode) -> np.ndarray:
    keep = [i for i, name in enumerate(ep.components) if name not in GPU_COMPONENTS]
    return ep.qfrc[:, keep, :].sum(axis=1)
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: the same command as Step 4.
Expected: 3 passed.
- If `manifest.get("file_sha256")` is not where the hash lives, read `native/src/recording/reader.cpp:29-79` and use the exact key path. Do not drop the check.
- If the qpos column comes back as a non-list type, convert it as `reader.cpp` does.
- Print `ep.components` in the task report.

- [ ] **Step 7: Record the T63 exemption**

In `docs/superpowers/plans/2026-10-20-cpp-linux-completion.md`, add after the OD-1 entry (`:55`):

```markdown
- **OD-colab (2026-11-02):** `tools/colab_gpu/**/*.py` is retained after T63. mujoco_warp and warp-metal are Python-only, so the GPU verification tools (`tools/colab_gpu/golden/`, plan `2026-11-02-gpu-golden-replay.md`) cannot be ported to C++. They run through `uv`, outside the CTest gate.
```

Change the T63 Step 1 deletion command (`:4775-4779`) from `git ls-files '*.py' | xargs git rm` to:

```bash
git ls-files '*.py' ':!:tools/colab_gpu/**' | xargs git rm
```

Change its goal check (`:4769`) to `git ls-files '*.py' ':!:tools/colab_gpu/**' | wc -l` = 0.

- [ ] **Step 8: Commit**

```bash
git add tools/colab_gpu/golden docs/superpowers/plans/2026-10-20-cpp-linux-completion.md
git commit -m "feat(colab_gpu): golden recording reader; keep colab_gpu through T63"
```

---

### Task 2: `bike-gpu-export` — model bundle from a recording

**Files:**
- Create: `native/src/replay/model_bundle.hpp`, `native/src/replay/model_bundle.cpp`, `native/src/tools/gpu_export_main.cpp`, `native/tests/integration/test_model_bundle.cpp`, `tools/colab_gpu/golden/bundle.py`, `tools/colab_gpu/golden/tests/test_bundle.py`
- Modify: `native/src/replay/playback.cpp:31-55` (`reconstruct_model` delegates to the bundle builder); `native/cmake/RecordingApplication.cmake` (target next to `bike-replay` at `:180`); test registration next to `test_replay_playback.cpp` (find it with `grep -rn test_replay_playback native/ CMakeLists.txt cmake/`)

**Interfaces:**
- Produces (C++):
  - `struct bike::replay::ModelBundle { std::string mjcf; std::vector<double> hfield_data; runtime::ModelImage image; int nq = 0, nv = 0; }`.
  - `ModelBundle build_model_bundle(const ReplayInput&)`.
  - `void write_model_bundle(const ModelBundle&, const std::filesystem::path& out_dir)`.
  - `ModelBundle export_model_bundle(const std::filesystem::path& recording, std::optional<std::int64_t> episode)`.
- Produces (bundle directory):
  - `model.xml`, `hfield_data.bin` (little-endian float64, `nhfielddata` values) and `model.mjb` (the exact image `load_playback` uses for this recording);
  - `bundle.json` with fields `model_sha256`, `nq`, `nv`, `nhfielddata`, `source_recording`, `mujoco_native_version`.
- Produces (Python): `load_bundle(bundle_dir: Path) -> tuple[mujoco.MjModel, dict]`.

- [ ] **Step 1: Write the failing native test**

`native/tests/integration/test_model_bundle.cpp` follows the structure of `test_replay_playback.cpp`: use `rr::record_container(tmp.path)` from `../reference/replay_reference_support.hpp` and the same `bt::require` helpers.

```cpp
// The exported bundle reproduces the playback model: its image is byte-equal
// to load_playback's, and recompiling its MJCF + heightfield gives the same
// nq/nv and heightfield raster.
#include "../reference/replay_reference_support.hpp"
#include "replay/model_bundle.hpp"
#include "replay/playback.hpp"
#include "model/builder.hpp"

namespace {
namespace rr = replay_reference;
namespace replay = bike::replay;

void bundle_matches_playback() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto data = replay::load_playback(publication.directory, 0);
    const auto bundle = replay::export_model_bundle(publication.directory, 0);
    bt::require(bundle.image.sha256 == data.model.sha256, "bundle image is the playback image");
    auto compiled = bike::model::compile_mjcf(bundle.mjcf);
    bt::require(compiled->nq == bundle.nq && compiled->nv == bundle.nv, "MJCF recompiles to the same dims");
    bt::require(static_cast<std::size_t>(compiled->nhfielddata) == bundle.hfield_data.size(),
        "heightfield raster size matches");
}

void write_creates_all_files() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto out = tmp.path / "bundle";
    replay::write_model_bundle(replay::export_model_bundle(publication.directory, 0), out);
    for (const char* name : {"model.xml", "hfield_data.bin", "model.mjb", "bundle.json"})
        bt::require(std::filesystem::is_regular_file(out / name), std::string("bundle has ") + name);
    bt::require(std::filesystem::file_size(out / "hfield_data.bin") % 8 == 0, "hfield is float64");
}
} // namespace

int main(int argc, char** argv) {
    return bt::run(argc, argv, {{"bundle_matches_playback", bundle_matches_playback},
                                {"write_creates_all_files", write_creates_all_files}});
}
```

Adapt `main`/`bt::run` to whatever runner `test_replay_playback.cpp` uses: copy its `main` verbatim and swap in the test table.

- [ ] **Step 2: Run it to verify it fails**

Run: `cmake --build build/native-release --target bike_test_model_bundle`
Expected: compile error, `replay/model_bundle.hpp` not found.

- [ ] **Step 3: Implement the bundle builder**

`native/src/replay/model_bundle.hpp`:

```cpp
#pragma once
#include "input.hpp"
#include "../runtime/runtime.hpp"
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bike::replay {
// Model artefacts for the Python GPU stack (mujoco 3.17), which cannot load a
// native 3.12 model.mjb: the MJCF text, the populated heightfield raster, and
// the exact playback image for the native viewer.
struct ModelBundle {
    std::string mjcf;
    std::vector<double> hfield_data;
    runtime::ModelImage image;
    int nq = 0, nv = 0;
};
[[nodiscard]] ModelBundle build_model_bundle(const ReplayInput&);
[[nodiscard]] ModelBundle export_model_bundle(const std::filesystem::path& recording,
                                              std::optional<std::int64_t> episode);
void write_model_bundle(const ModelBundle&, const std::filesystem::path& out_dir);
} // namespace bike::replay
```

`native/src/replay/model_bundle.cpp`:
- Move the body of `reconstruct_model` (`playback.cpp:31-55`) into `build_model_bundle`. Keep its exact checks and the float conversion of the raster.
- Additionally keep `mjcf = model::generate_mjcf(request->setup.model)`, copy the populated raster as doubles (`hfield_data[i] = model->hfield_data[i]` over `nhfielddata`), set `nq`/`nv`, and `image = initialization::capture_model_image(*model)`.
- Change `reconstruct_model` to `return build_model_bundle(input).image;`.
- `export_model_bundle` repeats `load_playback`'s container branch up to `ReplayInput input` (`playback.cpp:118-127`). Factor that branch into `ReplayInput load_replay_input(path, episode)` in `model_bundle.cpp` and call it from both places. Then:

```cpp
ModelBundle export_model_bundle(const std::filesystem::path& recording, std::optional<std::int64_t> episode) {
    const auto input = load_replay_input(recording, episode);
    auto bundle = build_model_bundle(input);
    // The viewer image must be exactly what load_playback serves, so a stored
    // model.mjb asset wins over the rebuilt image.
    if (const auto image = input.assets.find("model.mjb"); image != input.assets.end())
        bundle.image = {.mjb = image->second, .sha256 = io::sha256(image->second)};
    return bundle;
}
```

`write_model_bundle`:
- write the four files with `std::ofstream(..., std::ios::binary)` and check `good()` after each write;
- `hfield_data.bin` is the raw little-endian doubles (`static_assert(std::endian::native == std::endian::little)`);
- `bundle.json` is the five fields from the Interfaces block, written with the repo JSON writer used by `cli/export.cpp` (read it first and reuse its writer; do not hand-format JSON).

`native/src/tools/gpu_export_main.cpp`: `bike-gpu-export --recording DIR [--episode N] --out DIR`.
- Parse argv with the same helper `validate_main.cpp` uses.
- Call `export_model_bundle` then `write_model_bundle`.
- Print `bundle=<out> sha256=<sha>` and exit 0. On any exception print `error: <what>` and exit 1.
- CMake: copy the `bike-replay` `add_executable` block at `RecordingApplication.cmake:180` and rename it to `bike-gpu-export` with this source.

- [ ] **Step 4: Run the native test to verify it passes**

Run: `cmake --build build/native-release --target bike_test_model_bundle bike-gpu-export bike-replay && ctest --test-dir build/native-release -R model_bundle --output-on-failure`
Expected: PASS. Also rerun `ctest --test-dir build/native-release -R replay_playback` to confirm the refactor kept playback green.

- [ ] **Step 5: Export the two goldens and write the Python loader test**

```bash
for t in flat rough_uphill_extreme; do
  rec=$(dirname "$(find output/gpu_golden/$t -name manifest.json | sort | tail -1)")
  build/native-release/bin/bike-gpu-export --recording "$rec" --out output/gpu_golden/$t/bundle
done
```

`tools/colab_gpu/golden/tests/test_bundle.py`:

```python
"""The Python stack loads the exported bundle with the recording's dimensions."""
from __future__ import annotations

from tools.colab_gpu.golden.bundle import load_bundle
from tools.colab_gpu.golden.recording import load_golden
from tools.colab_gpu.golden.tests.conftest import REPO_ROOT


def test_bundle_dims_match_recording(golden_flat_dir):
    """nq/nv of the 3.17 model equal the recorded state widths."""
    model, meta = load_bundle(REPO_ROOT / "output" / "gpu_golden" / "flat" / "bundle")
    ep = load_golden(golden_flat_dir)
    assert model.nq == ep.qpos.shape[1] == meta["nq"]
    assert model.nv == ep.qvel.shape[1] == meta["nv"]
    assert model.nhfielddata == meta["nhfielddata"]
```

`tools/colab_gpu/golden/bundle.py`:

```python
"""Load a bike-gpu-export bundle into mujoco (Python 3.17 stack)."""
from __future__ import annotations

import json
from pathlib import Path

from tools.colab_gpu.model_assets import load_model_with_assets


def load_bundle(bundle_dir: Path):
    bundle_dir = Path(bundle_dir).resolve()
    meta = json.loads((bundle_dir / "bundle.json").read_text())
    # model_assets.load_model_with_assets reads hfield_data.bin next to the XML.
    model = load_model_with_assets(bundle_dir / "model.xml")
    if model.nq != meta["nq"] or model.nv != meta["nv"]:
        raise ValueError(f"bundle dims {model.nq}/{model.nv} differ from bundle.json {meta}")
    return model, meta
```

The bundle lives at the fixed path `output/gpu_golden/<track>/bundle` (Step 5 writes it there).

- [ ] **Step 6: Run the Python test**

Run: `uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.17.0 python -m pytest tools/colab_gpu/golden/tests/test_bundle.py -q`
Expected: PASS. If the MJCF references asset files (meshes or textures) that are not in the bundle, `load_model_with_assets` fails naming the file. Then extend `write_model_bundle` to copy the `FixedAssets` directory contents (`replay/rebuild.hpp:7`) into the bundle and rerun.

- [ ] **Step 7: Commit**

```bash
git add native/src/replay/model_bundle.* native/src/replay/playback.cpp native/src/tools/gpu_export_main.cpp \
  native/cmake/RecordingApplication.cmake native/tests/integration/test_model_bundle.cpp <test registration file> \
  tools/colab_gpu/golden/bundle.py tools/colab_gpu/golden/tests/test_bundle.py
git commit -m "feat(replay): bike-gpu-export writes an MJCF+heightfield model bundle from a recording"
```

---

### Task 3: Closure gate (CPU, FP64) — is injection sound?

**Files:**
- Create: `tools/colab_gpu/golden/closure.py`, `tools/colab_gpu/golden/tests/test_closure.py`
- Create: `docs/superpowers/audits/2026-11-02-gpu-golden-closure.md`

**Interfaces:**
- Consumes: `load_golden`, `load_bundle`.
- Produces: `closure_residuals(model, ep, start: int, count: int) -> ClosureResult`, where `ClosureResult` (dataclass) has fields `max_dqpos: np.ndarray[(count,)]`, `max_dqvel: np.ndarray[(count,)]` and `worst_dof: np.ndarray[(count,)]` of type int.

This task is a **gate**. It decides whether D6 can work at all.

- [ ] **Step 1: Write the test**

```python
"""Applying Σ recorded force components as qfrc_applied (ctrl = 0) to the
recorded incoming state reproduces the next recorded state."""
from __future__ import annotations

import numpy as np

from tools.colab_gpu.golden.bundle import load_bundle
from tools.colab_gpu.golden.closure import closure_residuals
from tools.colab_gpu.golden.recording import load_golden
from tools.colab_gpu.golden.tests.conftest import REPO_ROOT


def _check(rec_dir, track):
    model, _ = load_bundle(REPO_ROOT / "output" / "gpu_golden" / track / "bundle")
    ep = load_golden(rec_dir)
    count = min(1600, ep.times_s.shape[0] - 1)  # 2 s
    res = closure_residuals(model, ep, 0, count)
    print(f"{track}: max dqpos {res.max_dqpos.max():.3e} max dqvel {res.max_dqvel.max():.3e} "
          f"worst dof {np.bincount(res.worst_dof).argmax()}")
    # FP64, same MJCF: only solver warmstart (not recorded) and the mujoco
    # version can differ. 1e-6 m/s is ~1e4 above expected warmstart noise.
    assert res.max_dqvel.max() <= 1e-6


def test_closure_flat(golden_flat_dir):
    _check(golden_flat_dir, "flat")


def test_closure_extreme(golden_extreme_dir):
    _check(golden_extreme_dir, "rough_uphill_extreme")
```

- [ ] **Step 2: Implement**

```python
"""CPU FP64 closure: does qfrc_applied = Σ recorded components reproduce the CPU step?"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class ClosureResult:
    max_dqpos: np.ndarray
    max_dqvel: np.ndarray
    worst_dof: np.ndarray


def closure_residuals(model, ep, start: int, count: int) -> ClosureResult:
    d = mujoco.MjData(model)
    dq = np.empty(count)
    dv = np.empty(count)
    worst = np.empty(count, dtype=np.int64)
    for k, i in enumerate(range(start, start + count)):
        mujoco.mj_resetData(model, d)
        d.time = ep.times_s[i]
        d.qpos[:] = ep.qpos[i]
        d.qvel[:] = ep.qvel[i]
        d.ctrl[:] = 0.0
        d.qfrc_applied[:] = ep.qfrc[i].sum(axis=0)
        mujoco.mj_step(model, d)
        err_v = np.abs(d.qvel - ep.qvel[i + 1])
        dq[k] = np.max(np.abs(d.qpos - ep.qpos[i + 1]))
        dv[k] = err_v.max()
        worst[k] = int(err_v.argmax())
    return ClosureResult(dq, dv, worst)
```

- [ ] **Step 3: Run under mujoco 3.17 (the GPU stack) and 3.12 (native)**

```bash
uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.17.0 python -m pytest tools/colab_gpu/golden/tests/test_closure.py -q -s
uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.12.0 python -m pytest tools/colab_gpu/golden/tests/test_closure.py -q -s
```

Record both printed lines in the audit doc.

- [ ] **Step 4: Apply the gate**

The audit doc `docs/superpowers/audits/2026-11-02-gpu-golden-closure.md` gets a table: track × mujoco version → max dqpos, max dqvel, and the worst dof's joint name (`mujoco.mj_id2name(model, mjOBJ_JOINT, model.dof_jntid[dof])`).

- **Both versions pass:** continue to Task 4.
- **3.12 passes, 3.17 fails:** the cause is the mujoco version. Stop and report to the user; the GPU stack's mujoco version is a separate decision.
- **Both fail:**
  1. Look up the worst dof.
  2. If it is a crank, wheel, brake or rider joint, the residual is actuator force from `ctrl` (not recorded), the freehub `tendon_range` or `eq_active`.
  3. Write that conclusion with the numbers. **Stop and report to the user.** Recording `ctrl`/`tendon_range`/`eq_active` is a native recording change and needs its own plan.
  4. Do not continue to Task 4 with an unsound injection.

- [ ] **Step 5: Commit**

```bash
git add tools/colab_gpu/golden/closure.py tools/colab_gpu/golden/tests/test_closure.py docs/superpowers/audits/2026-11-02-gpu-golden-closure.md
git commit -m "test(colab_gpu): CPU closure gate for recorded-force injection"
```

---

### Task 4: GPU golden writer step (host C++)

**Files:**
- Modify: `native/src/gpu/common.hpp:61`, `native/src/gpu/drivetrain_resistance_kernel.hpp` (after `:350`), `native/src/gpu/batch_step.hpp` (after `:221`)
- Test: `native/tests/gpu/test_gpu_batch_step.cpp`

**Interfaces:**
- Produces:
  - `kGoldenComponents = 3`, with slot order `0 = suspension`, `1 = tires`, `2 = resistance` (matching `GPU_COMPONENTS`).
  - `struct GpuGoldenWheels { GpuWheelContactOutput front, rear; }`.
  - `gpu_write_resistance_only_qfrc(const GpuDrivetrainOutput&, const GpuModelIndices&, float* qfrc)`.
  - `gpu_step_world_writers_golden(float dt_s, const GpuModelIndices&, const GpuHeightfield1D&, const GpuWorldConfig&, GpuWorldState&, const float* qpos_w, const float* qvel_w, const float* xpos_w, const float* qfrc_inject_w, float* qfrc_w, float* components_w, GpuGoldenWheels* wheels)`. Here `qfrc_inject_w` and `components_w` (`3*nv`) and `wheels` may be null.

- [ ] **Step 1: Write the failing tests** (add to the test table in `test_gpu_batch_step.cpp`)

```cpp
// The golden step computes exactly suspension + tyres + resistance, adds the
// injected vector unchanged, writes no ctrl, and its per-component slots sum
// to the applied total.
void golden_step_components_sum_to_total() {
    GpuModelIndices idx{};
    GpuWorldConfig cfg{};
    gpu_init_default_world_config(cfg);
    GpuWorldState st{};
    std::vector<float> qpos(kMaxNq, 0.f), qvel(kMaxNv, 0.f), xpos(kMaxNbody * 3, 0.f);
    qvel[static_cast<std::size_t>(idx.root_x_dof)] = 5.f;           // aero + rolling act
    qpos[static_cast<std::size_t>(idx.fork_travel_qpos)] = .02f;      // suspension acts
    std::vector<float> inject(kMaxNv, 0.f);
    inject[static_cast<std::size_t>(idx.crank_spin_dof)] = 12.5f;
    std::vector<float> total(kMaxNv, -99.f), comps(3 * kMaxNv, -99.f);
    GpuGoldenWheels wheels{};
    const GpuHeightfield1D hf{};
    gpu_step_world_writers_golden(.00125f, idx, hf, cfg, st, qpos.data(), qvel.data(), xpos.data(),
        inject.data(), total.data(), comps.data(), &wheels);
    for (int i = 0; i < idx.nv; ++i) {
        const auto u = static_cast<std::size_t>(i);
        const float sum = comps[u] + comps[kMaxNv + u] + comps[2 * kMaxNv + u] + inject[u];
        require_near(total[u], sum, 1e-6f, "total = Σ components + inject");
    }
    require(comps[static_cast<std::size_t>(idx.fork_travel_dof)] != 0.f, "suspension slot is populated");
    require(comps[2 * kMaxNv + static_cast<std::size_t>(idx.root_x_dof)] < 0.f, "resistance opposes motion");
}

// Resistance-only writes aero/rolling/bearing and never brake, drive or crank torque.
void resistance_only_excludes_drive_and_brakes() {
    GpuModelIndices idx{};
    GpuDrivetrainOutput out{};
    out.aero_fx_n = -3.f; out.front_rolling_torque_nm = -1.f; out.rear_bearing_torque_nm = -.5f;
    out.front_brake_torque_nm = -100.f; out.rear_brake_torque_nm = -100.f;
    out.rear_drive_torque_nm = 50.f; out.crank_torque_nm = 30.f;
    std::vector<float> q(kMaxNv, 0.f);
    gpu_write_resistance_only_qfrc(out, idx, q.data());
    require_near(q[static_cast<std::size_t>(idx.front_wheel_spin_dof)], -1.f, 0.f, "front: rolling only");
    require_near(q[static_cast<std::size_t>(idx.rear_wheel_spin_dof)], -.5f, 0.f, "rear: bearing only");
    require_near(q[static_cast<std::size_t>(idx.crank_spin_dof)], 0.f, 0.f, "no crank torque");
}

// The CPU distributed config uses 128 stations; the state must hold them.
void tread_station_capacity_covers_cpu_default() {
    static_assert(kMaxTreadStations >= 128, "GPU must hold the CPU station_count");
}
```

Use the exact field names of `GpuDrivetrainOutput` from `drivetrain_resistance_kernel.hpp` (read the struct; the names above are the ones `gpu_write_drivetrain_and_resistance_qfrc` uses at `:319-350`).

- [ ] **Step 2: Run to verify they fail**

Run: `cmake --build build/native-release --target bike_test_gpu_batch_step`
Expected: compile errors for the missing functions and the `static_assert`.

- [ ] **Step 3: Implement**

`common.hpp:61` becomes `BIKE_CONSTEXPR int kMaxTreadStations = 128;`

`drivetrain_resistance_kernel.hpp`, after `gpu_write_drivetrain_and_resistance_qfrc(const GpuDrivetrainOutput&…)` (`:319-350`):

```cpp
// Golden replay: only the resistance part (aero, rolling, bearings). Brakes,
// rear drive and crank torque come from the recorded CPU components instead.
BIKE_HD void gpu_write_resistance_only_qfrc(
    const BIKE_THREAD GpuDrivetrainOutput &out,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_THREAD float *qfrc_world
) noexcept {
    if (idx.root_x_dof >= 0 && idx.root_x_dof < kMaxNv) qfrc_world[idx.root_x_dof] += out.aero_fx_n;
    if (idx.root_z_dof >= 0 && idx.root_z_dof < kMaxNv) qfrc_world[idx.root_z_dof] += out.aero_fz_n;
    if (idx.front_wheel_spin_dof >= 0 && idx.front_wheel_spin_dof < kMaxNv)
        qfrc_world[idx.front_wheel_spin_dof] += out.front_rolling_torque_nm + out.front_bearing_torque_nm;
    if (idx.rear_wheel_spin_dof >= 0 && idx.rear_wheel_spin_dof < kMaxNv)
        qfrc_world[idx.rear_wheel_spin_dof] += out.rear_rolling_torque_nm + out.rear_bearing_torque_nm;
}
```

`batch_step.hpp`, after `gpu_step_world_writers`:

```cpp
BIKE_CONSTEXPR int kGoldenComponents = 3; // 0 suspension, 1 tires, 2 resistance

struct GpuGoldenWheels {
    GpuWheelContactOutput front{};
    GpuWheelContactOutput rear{};
};

// Golden replay step (plan 2026-11-02-gpu-golden-replay D6): computes only the
// GPU-owned components, adds the recorded CPU components for everything else,
// and leaves ctrl and tendon_range untouched. components_w is kGoldenComponents
// rows of kMaxNv (row stride kMaxNv, so slots align with the host test).
BIKE_HD void gpu_step_world_writers_golden(
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD GpuHeightfield1D &hf,
    const BIKE_THREAD GpuWorldConfig &cfg,
    BIKE_THREAD GpuWorldState &st,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    const BIKE_THREAD float *xpos_w,
    const BIKE_THREAD float *qfrc_inject_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *components_w,
    BIKE_THREAD GpuGoldenWheels *wheels
) noexcept {
    if (idx.nv < 0 || idx.nv > kMaxNv || idx.nq < 0 || idx.nq > kMaxNq) return;
    float comp[kGoldenComponents][kMaxNv];
    for (int c = 0; c < kGoldenComponents; ++c)
        for (int i = 0; i < kMaxNv; ++i) comp[c][i] = 0.0f;

    gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_w, qvel_w, comp[0]);

    GpuWheelContactOutput front_out{};
    GpuWheelContactOutput rear_out{};
    gpu_write_both_wheels_qfrc(xpos_w, qvel_w, dt_s, cfg.front_tire, cfg.rear_tire, hf, idx,
        &st.front_wheel, &st.rear_wheel, comp[1], &front_out, &rear_out);

    // Demands are zero: only the resistance terms of the output are used.
    float scratch[kMaxNv];
    for (int i = 0; i < kMaxNv; ++i) scratch[i] = 0.0f;
    const GpuDrivetrainOutput dt_out = gpu_write_drivetrain_and_resistance_qfrc(
        cfg.resistance, cfg.drivetrain, idx, st.drivetrain, qpos_w, qvel_w,
        front_out.normal_load_n, rear_out.normal_load_n,
        front_out.effective_radius_m, rear_out.effective_radius_m,
        0.0f, 0.0f, 0.0f, scratch, false);
    gpu_write_resistance_only_qfrc(dt_out, idx, comp[2]);

    for (int i = 0; i < idx.nv; ++i) {
        const float inject = qfrc_inject_w != nullptr ? qfrc_inject_w[i] : 0.0f;
        qfrc_w[i] = comp[0][i] + comp[1][i] + comp[2][i] + inject;
    }
    if (components_w != nullptr)
        for (int c = 0; c < kGoldenComponents; ++c)
            for (int i = 0; i < kMaxNv; ++i) components_w[c * kMaxNv + i] = comp[c][i];
    if (wheels != nullptr) {
        wheels->front = front_out;
        wheels->rear = rear_out;
    }
}
```

The exact `gpu_write_drivetrain_and_resistance_qfrc` overload is the one `batch_step.hpp:190-206` calls (`… qfrc_w, include_rear_drive_torque`). Use the same argument order.

- [ ] **Step 4: Run all GPU host tests**

Run: `cmake --build build/native-release --target bike_test_gpu_batch_step bike_test_gpu_tire bike_test_gpu_batch_runners && ctest --test-dir build/native-release -R gpu --output-on-failure`
Expected:
- the new tests pass;
- every pre-existing GPU test still passes. Raising `kMaxTreadStations` must not change results for `num_stations ≤ 64`; if a test hard-codes a 64-station state size, update it to `kMaxTreadStations`.

- [ ] **Step 5: Commit**

```bash
git add native/src/gpu/common.hpp native/src/gpu/drivetrain_resistance_kernel.hpp native/src/gpu/batch_step.hpp native/tests/gpu/test_gpu_batch_step.cpp
git commit -m "feat(gpu): golden writer step with recorded-force injection; 128 tread stations"
```

---

### Task 5: Metal golden kernel, windowed GPU runner, batch identity

**Files:**
- Modify: `native/src/gpu/metal_batch_runner.mm` (after `bike_wp_writers_simd32_kernel`, ~`:745`; register the function name next to `:890`)
- Modify: `tools/colab_gpu/mjwarp_metal_sweep.py` (`WarpMetalWritersRuntime`: add `k_golden` and `launch_golden`)
- Create: `tools/colab_gpu/golden/gpu_runner.py`, `tools/colab_gpu/golden/tests/test_gpu_runner.py`

**Interfaces:**
- Consumes: `gpu_step_world_writers_golden`, `GpuGoldenWheels`, `kGoldenComponents`; `load_golden`, `injected_qfrc`, `load_bundle`.
- Produces:
  - `GpuRun` (dataclass) with fields:
    - `window_starts: np.ndarray[int]`, `window_ends: np.ndarray[int]`;
    - `end_qpos: np.ndarray[(nwin, nq)]`, `end_qvel: np.ndarray[(nwin, nv)]`;
    - `components: np.ndarray[(n_steps, 3, nv)]` (float64 copy of float32), `wheels: np.ndarray[(n_steps, 2, 4)]` (`contact_x`, `contact_z`, `fx`, `fz`);
    - `frame_steps: np.ndarray[int]`, `frame_qpos: np.ndarray[(nf, nq)]`, `frame_qvel`;
    - `mode: str`, `window_s: float`, `frame_stride: int`.
  - `run_golden_gpu(bundle_dir, recording_dir, *, window_s=0.1, frame_stride=10, mode="windowed", episode_id=0, nworld=1, config_bin=None) -> GpuRun`.
  - `check_batch_identity(run_n: GpuRunBatch) -> None` raises `AssertionError`.

- [ ] **Step 1: Add the Metal kernel**

```cpp
struct BikeWpGoldenArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const float *hf_heights;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *qfrc_inject;      // nworld * nv
    device float *qfrc_applied;           // nworld * nv
    device float *components;             // nworld * kGoldenComponents * kMaxNv
    device bike::gpu::GpuGoldenWheels *wheels; // nworld
    int nworld;
    float dt_s;
    int hf_num_samples;
    float hf_x_min;
    float hf_dx;
    int _pad;
};

// Golden replay (plan 2026-11-02-gpu-golden-replay): scalar per-world path,
// same thread-local copy-in/copy-out as bike_metal_step_writers_scalar_world.
kernel void bike_wp_writers_golden_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpGoldenArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) return;
    const bike::gpu::GpuModelIndices idx = *(args->idx);
    if (idx.nq < 0 || idx.nq > bike::gpu::kMaxNq || idx.nv < 0 || idx.nv > bike::gpu::kMaxNv ||
        idx.nbody < 0 || idx.nbody > bike::gpu::kMaxNbody) return;
    const float inv_dx = args->hf_dx > 0.0f ? (1.0f / args->hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{args->hf_heights, args->hf_num_samples, args->hf_x_min, args->hf_dx, inv_dx};
    const bike::gpu::GpuWorldConfig cfg = args->configs[0];
    bike::gpu::GpuWorldState st = args->states[w];
    float qpos_w[bike::gpu::kMaxNq], qvel_w[bike::gpu::kMaxNv], xpos_w[bike::gpu::kMaxNbody * 3];
    float inject_w[bike::gpu::kMaxNv], qfrc_w[bike::gpu::kMaxNv];
    float comp_w[bike::gpu::kGoldenComponents * bike::gpu::kMaxNv];
    bike::gpu::GpuGoldenWheels wheels_w{};
    const int nq = idx.nq, nv = idx.nv, nb3 = idx.nbody * 3;
    for (int i = 0; i < nq; ++i) qpos_w[i] = args->qpos[w * nq + i];
    for (int i = 0; i < nv; ++i) { qvel_w[i] = args->qvel[w * nv + i]; inject_w[i] = args->qfrc_inject[w * nv + i]; }
    for (int i = 0; i < nb3; ++i) xpos_w[i] = args->xpos[w * nb3 + i];
    bike::gpu::gpu_step_world_writers_golden(args->dt_s, idx, hf, cfg, st, qpos_w, qvel_w, xpos_w,
        inject_w, qfrc_w, comp_w, &wheels_w);
    for (int i = 0; i < nv; ++i) args->qfrc_applied[w * nv + i] = qfrc_w[i];
    const int stride = bike::gpu::kGoldenComponents * bike::gpu::kMaxNv;
    for (int i = 0; i < stride; ++i) args->components[w * stride + i] = comp_w[i];
    args->wheels[w] = wheels_w;
    args->states[w] = st;
}
```

Next to `:890`, add the pipeline lookup for `bike_wp_writers_golden_kernel` in the same way as `fn_wp1`, so the shader-source compile check covers it.

- [ ] **Step 2: Add the Python launcher**

In `tools/colab_gpu/mjwarp_metal_sweep.py`, add the ctypes mirror and the method to `WarpMetalWritersRuntime`:

```python
class _BikeWpGoldenArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64), ("hf_heights", ctypes.c_uint64), ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64), ("qpos", ctypes.c_uint64), ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64), ("qfrc_inject", ctypes.c_uint64), ("qfrc_applied", ctypes.c_uint64),
        ("components", ctypes.c_uint64), ("wheels", ctypes.c_uint64),
        ("nworld", ctypes.c_int32), ("dt_s", ctypes.c_float), ("hf_num_samples", ctypes.c_int32),
        ("hf_x_min", ctypes.c_float), ("hf_dx", ctypes.c_float), ("_pad", ctypes.c_int32),
    ]


_GOLDEN_PTR_OFFSETS = (ctypes.c_size_t * 11)(*(8 * i for i in range(11)))
```

In `__init__`:

```python
        self.k_golden = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_writers_golden_kernel")
        if not self.k_golden:
            raise RuntimeError("wp_metal_get_kernel(bike_wp_writers_golden_kernel) failed:\n"
                               + wp._src.context.runtime.get_error_string())
```

The method:

```python
    def launch_golden(self, *, nworld: int, dt_s: float, d_idx_ptr: int, d_hf_ptr: int,
                      hf_num_samples: int, hf_x_min: float, hf_dx: float, d_configs_ptr: int,
                      d_states_ptr: int, d_qpos_ptr: int, d_qvel_ptr: int, d_xpos_ptr: int,
                      d_inject_ptr: int, d_qfrc_ptr: int, d_components_ptr: int, d_wheels_ptr: int) -> None:
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpGoldenArgs(
            idx=d_idx_ptr, hf_heights=d_hf_ptr, configs=d_configs_ptr, states=d_states_ptr,
            qpos=d_qpos_ptr, qvel=d_qvel_ptr, xpos=d_xpos_ptr, qfrc_inject=d_inject_ptr,
            qfrc_applied=d_qfrc_ptr, components=d_components_ptr, wheels=d_wheels_ptr,
            nworld=int(nworld), dt_s=float(dt_s), hf_num_samples=int(hf_num_samples),
            hf_x_min=float(hf_x_min), hf_dx=float(hf_dx), _pad=0,
        )
        rc = self.core.wp_metal_launch_kernel(self.ordinal, self.k_golden, int(nworld), 64,
            ctypes.byref(bounds), ctypes.sizeof(bounds), ctypes.byref(args), ctypes.sizeof(args),
            _GOLDEN_PTR_OFFSETS, 11, 0)
        if rc != 0:
            raise RuntimeError("wp_metal_launch_kernel(bike_wp_writers_golden_kernel) failed: "
                               + wp._src.context.runtime.get_error_string())
```

- [ ] **Step 3: Write the failing GPU runner tests**

`tools/colab_gpu/golden/tests/test_gpu_runner.py`:

```python
"""Windowed golden replay on metal:0 (requires .venv-metal)."""
from __future__ import annotations

import numpy as np
import pytest

warp = pytest.importorskip("warp")
pytest.importorskip("warp_metal")

from tools.colab_gpu.golden.gpu_runner import run_golden_gpu
from tools.colab_gpu.golden.recording import load_golden
from tools.colab_gpu.golden.tests.conftest import REPO_ROOT

BUNDLE = REPO_ROOT / "output" / "gpu_golden" / "flat" / "bundle"


def test_windows_cover_run_and_are_finite(golden_flat_dir):
    """Windows tile the episode without gaps; every GPU output is finite."""
    ep = load_golden(golden_flat_dir)
    run = run_golden_gpu(BUNDLE, golden_flat_dir, window_s=0.1, frame_stride=10, max_steps=800)
    assert run.window_starts[0] == 0
    assert np.all(run.window_starts[1:] == run.window_ends[:-1])
    assert run.components.shape == (800, 3, ep.qvel.shape[1])
    assert np.isfinite(run.end_qpos).all() and np.isfinite(run.components).all()


def test_window_start_reseats_golden_state(golden_flat_dir):
    """With a one-step window the GPU starts each step from the golden state, so one-step
    error stays at FP32 scale and does not accumulate."""
    run = run_golden_gpu(BUNDLE, golden_flat_dir, window_s=0.00125, frame_stride=1, max_steps=200)
    ep = load_golden(golden_flat_dir)
    err = np.abs(run.end_qpos - ep.qpos[run.window_ends]).max(axis=1)
    assert err.max() < 1e-2  # one FP32 step; accumulation would grow it monotonically
    assert not np.all(np.diff(err) > 0)


def test_batch_identity(golden_flat_dir):
    """nworld = 4 with identical input reproduces world 0 bitwise (D12)."""
    run = run_golden_gpu(BUNDLE, golden_flat_dir, window_s=0.1, frame_stride=10, max_steps=400, nworld=4)
    for w in range(1, 4):
        assert np.array_equal(run.world_end_qpos[w], run.world_end_qpos[0])
        assert np.array_equal(run.world_components[w], run.world_components[0])
```

- [ ] **Step 4: Run to verify they fail**

Run: `uv run --no-project --python .venv-metal/bin/python python -m pytest tools/colab_gpu/golden/tests/test_gpu_runner.py -q`
Expected: FAIL with `ModuleNotFoundError: tools.colab_gpu.golden.gpu_runner`.

- [ ] **Step 5: Implement the runner**

```python
"""Golden replay of a CPU recording on metal:0 (plan 2026-11-02 D5-D7, D12).

Per step: copy the recorded injection row on-device → golden writers kernel →
mjwarp.step → device-copy components/wheels/frames into run-length buffers.
At each window start (mode="windowed") only qpos/qvel/time are re-seated from
the golden run. Writer state (bristles, freehub, damper) is never reset after
step 0 (D7). No host sync inside the step loop except window re-seats.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tools.colab_gpu.build_cuda_writers import DEFAULT_GPU_SRC_DIR, build_sweep_configs_bytes, resolve_gpu_model_indices
from tools.colab_gpu.golden.bundle import load_bundle
from tools.colab_gpu.golden.recording import injected_qfrc, load_golden
from tools.colab_gpu.metal_hybrid_sweep import compile_metal_batch_runner
from tools.colab_gpu.mjwarp_batch_sweep import _allocate_and_copy_device_bytes, _extract_heightfield_1d
from tools.colab_gpu.mjwarp_metal_sweep import WarpMetalWritersRuntime, _init_metal_device

K_COMP = 3
K_MAX_NV = 32  # native/src/gpu/common.hpp kMaxNv; components rows are kMaxNv wide
WHEEL_FLOATS = 8  # GpuWheelContactOutput: fx, fz, spin, load, radius, active(int), cx, cz


@dataclass
class GpuRun:
    mode: str
    window_s: float
    frame_stride: int
    window_starts: np.ndarray
    window_ends: np.ndarray
    end_qpos: np.ndarray
    end_qvel: np.ndarray
    components: np.ndarray
    wheels: np.ndarray
    frame_steps: np.ndarray
    frame_qpos: np.ndarray
    frame_qvel: np.ndarray
    world_end_qpos: np.ndarray
    world_components: np.ndarray


def _default_config_bytes(lib, nworld: int) -> bytes:
    ones = np.ones(1, dtype=np.float32)
    # Current GPU defaults (pre-Task-8 baseline): 32 stations, 1 rib.
    return build_sweep_configs_bytes(lib=lib, nworld=1, fork_psi_scale=ones, rebound_scale=ones,
                                     shock_rate_scale=ones, tire_stiff_scale=ones,
                                     num_stations=32, num_lateral_ribs=1)


def run_golden_gpu(bundle_dir: Path, recording_dir: Path, *, window_s: float = 0.1, frame_stride: int = 10,
                   mode: str = "windowed", episode_id: int = 0, nworld: int = 1,
                   config_bin: Path | None = None, max_steps: int | None = None) -> GpuRun:
    import mujoco_warp as mjwarp
    import warp as wp

    if mode not in ("windowed", "free"):
        raise ValueError(f"mode must be windowed or free, not {mode!r}")
    _init_metal_device()
    device = "metal:0"
    model, _ = load_bundle(bundle_dir)
    ep = load_golden(recording_dir, episode_id)
    if abs(float(model.opt.timestep) - ep.dt_s) > 1e-15:
        raise ValueError(f"bundle timestep {model.opt.timestep} != recording dt {ep.dt_s}")
    nq, nv = model.nq, model.nv
    n_steps = ep.times_s.shape[0] - 1
    if max_steps is not None:
        n_steps = min(n_steps, int(max_steps))
    win = max(1, int(round(window_s / ep.dt_s)))
    starts = np.arange(0, n_steps, win, dtype=np.int64)
    ends = np.minimum(starts + win, n_steps)

    lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    rt = WarpMetalWritersRuntime(lib)
    cfg_bytes = Path(config_bin).read_bytes() if config_bin else _default_config_bytes(lib, nworld)
    d_idx = _allocate_and_copy_device_bytes(resolve_gpu_model_indices(model), device=device)
    d_cfg = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_states = wp.zeros(nworld * int(lib.bike_gpu_sizeof_world_state()), dtype=wp.uint8, device=device)
    d_hf, hf_n, hf_x0, hf_dx = _extract_heightfield_1d(model, device=device)

    inject_host = np.tile(injected_qfrc(ep)[:n_steps].astype(np.float32), (1, nworld))  # (n, W*nv)
    d_inject_all = wp.array(inject_host.reshape(-1), dtype=wp.float32, device=device)
    d_inject = wp.zeros(nworld * nv, dtype=wp.float32, device=device)
    comp_w = nworld * K_COMP * K_MAX_NV
    d_comp = wp.zeros(comp_w, dtype=wp.float32, device=device)
    d_comp_all = wp.zeros(n_steps * comp_w, dtype=wp.float32, device=device)
    d_wheel = wp.zeros(nworld * 2 * WHEEL_FLOATS, dtype=wp.float32, device=device)
    d_wheel_all = wp.zeros(n_steps * nworld * 2 * WHEEL_FLOATS, dtype=wp.float32, device=device)

    m_wp = mjwarp.put_model(model)
    import mujoco
    d0 = mujoco.MjData(model)
    d0.qpos[:] = ep.qpos[0]
    d0.qvel[:] = ep.qvel[0]
    d0.time = ep.times_s[0]
    d_b = mjwarp.put_data(model, d0, nworld=nworld, nconmax=64, njmax=128)

    frame_steps: list[int] = []
    frame_q: list[np.ndarray] = []
    frame_v: list[np.ndarray] = []
    end_q = np.empty((starts.size, nworld, nq))
    end_v = np.empty((starts.size, nworld, nv))

    for k, (i0, i1) in enumerate(zip(starts, ends)):
        if k == 0 or mode == "windowed":
            d_b.qpos.assign(np.tile(ep.qpos[i0].astype(np.float32), (nworld, 1)))
            d_b.qvel.assign(np.tile(ep.qvel[i0].astype(np.float32), (nworld, 1)))
            d_b.time.assign(np.full(nworld, ep.times_s[i0], dtype=np.float32))
            mjwarp.forward(m_wp, d_b)  # refresh xpos for the tyre writer
            if k == 0:
                rt.launch_init_states(nworld=nworld, d_idx_ptr=int(d_idx.ptr), d_configs_ptr=int(d_cfg.ptr),
                                      per_world_config=0, d_states_ptr=int(d_states.ptr),
                                      d_qpos_ptr=int(d_b.qpos.ptr))
        for s in range(int(i0), int(i1)):
            wp.copy(d_inject, d_inject_all, dest_offset=0, src_offset=s * nworld * nv, count=nworld * nv)
            rt.launch_golden(nworld=nworld, dt_s=ep.dt_s, d_idx_ptr=int(d_idx.ptr), d_hf_ptr=int(d_hf.ptr),
                             hf_num_samples=hf_n, hf_x_min=hf_x0, hf_dx=hf_dx, d_configs_ptr=int(d_cfg.ptr),
                             d_states_ptr=int(d_states.ptr), d_qpos_ptr=int(d_b.qpos.ptr),
                             d_qvel_ptr=int(d_b.qvel.ptr), d_xpos_ptr=int(d_b.xpos.ptr),
                             d_inject_ptr=int(d_inject.ptr), d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
                             d_components_ptr=int(d_comp.ptr), d_wheels_ptr=int(d_wheel.ptr))
            mjwarp.step(m_wp, d_b)
            wp.copy(d_comp_all, d_comp, dest_offset=s * comp_w, src_offset=0, count=comp_w)
            wp.copy(d_wheel_all, d_wheel, dest_offset=s * nworld * 2 * WHEEL_FLOATS, src_offset=0,
                    count=nworld * 2 * WHEEL_FLOATS)
            if (s + 1 - i0) % frame_stride == 0 or s + 1 == i1:
                frame_steps.append(s + 1)
                frame_q.append(d_b.qpos.numpy()[0].astype(np.float64))
                frame_v.append(d_b.qvel.numpy()[0].astype(np.float64))
        end_q[k] = d_b.qpos.numpy().astype(np.float64)
        end_v[k] = d_b.qvel.numpy().astype(np.float64)

    wp.synchronize()
    comps = d_comp_all.numpy().reshape(n_steps, nworld, K_COMP, K_MAX_NV)[..., :nv].astype(np.float64)
    wheels_raw = d_wheel_all.numpy().reshape(n_steps, nworld, 2, WHEEL_FLOATS)
    # fx=0, fz=1, contact_x=6, contact_z=7 (GpuWheelContactOutput field order).
    wheels = wheels_raw[:, 0][..., [6, 7, 0, 1]].astype(np.float64)
    return GpuRun(mode, window_s, frame_stride, starts, ends, end_q[:, 0], end_v[:, 0], comps[:, 0], wheels,
                  np.asarray(frame_steps), np.asarray(frame_q), np.asarray(frame_v),
                  np.moveaxis(end_q, 1, 0), np.moveaxis(comps, 1, 0))
```

Notes for the implementer:
- `frame_q`/`frame_v` call `.numpy()`, which syncs. Frames are every `frame_stride` steps (default 10), so that is 10× fewer syncs than steps. If the 30 s run takes longer than 10 min, replace them with device copies into a preallocated frame buffer, exactly like `d_comp_all`.
- Check that `wp.copy` with `dest_offset`/`src_offset`/`count` is supported by warp 1.17 on `metal:0` with a two-line probe before relying on it. If it is not, upload the whole injection table once and add an `int step` field to `BikeWpGoldenArgs` that the kernel uses as the row offset into `qfrc_inject`, `components` and `wheels`.
- `GpuWheelContactOutput` holds an `int` (`active_stations`) at float index 5. Reading it as float32 is only used for indices 0, 1, 6 and 7; never interpret index 5.
- `d_b.time` dtype follows mujoco_warp's `Data.time`. Use its dtype instead of float32 if it differs.

- [ ] **Step 6: Run the GPU tests**

Run: the same command as Step 4.
Expected: 3 passed. Record the wall time of `test_windows_cover_run_and_are_finite`.

- [ ] **Step 7: Commit**

```bash
git add native/src/gpu/metal_batch_runner.mm tools/colab_gpu/mjwarp_metal_sweep.py tools/colab_gpu/golden/gpu_runner.py tools/colab_gpu/golden/tests/test_gpu_runner.py
git commit -m "feat(colab_gpu): windowed golden replay on metal:0 with recorded-force injection"
```

---

### Task 6: Comparison report

**Files:**
- Create: `tools/colab_gpu/golden/compare.py`, `tools/colab_gpu/golden/tests/test_compare.py`
- Modify: `tools/colab_gpu/golden/cli.py` (add `run` and `report` subcommands)

**Interfaces:**
- Consumes: `GoldenEpisode`, `GpuRun`, `component_qfrc`, `GPU_COMPONENTS`.
- Produces:
  - `LEVELS = (1e-6, 1e-3, 1e-2)`;
  - `build_report(model, ep: GoldenEpisode, windowed: GpuRun, free: GpuRun | None) -> dict`;
  - `write_report(report: dict, out_dir: Path) -> None`, which writes `report.json` and `report.md`.

- [ ] **Step 1: Write the failing tests** (synthetic data; no GPU needed)

```python
"""Report math on synthetic runs: a perfect GPU reports zero; an injected
error is located in the right window and component."""
from __future__ import annotations

import mujoco
import numpy as np

from tools.colab_gpu.golden.compare import LEVELS, build_report
from tools.colab_gpu.golden.gpu_runner import GpuRun
from tools.colab_gpu.golden.recording import GoldenEpisode

XML = """<mujoco><worldbody><body name="b"><joint name="root_x" type="slide" axis="1 0 0"/>
<geom size=".1"/></body></worldbody></mujoco>"""


def _episode(n=40):
    t = np.arange(n) * 0.00125
    q = np.linspace(0, 1, n)[:, None]
    comps = ("resistance", "rider_interfaces", "suspension", "tires")
    qfrc = np.ones((n, len(comps), 1))
    return GoldenEpisode(t, t + 0.00125, 0.00125, q, q.copy(), comps, qfrc)


def _run(ep, err_window=None, err_comp=None):
    starts = np.arange(0, 39, 10)
    ends = np.minimum(starts + 10, 39)
    end_q = ep.qpos[ends].copy()
    comps = np.stack([ep.qfrc[:39, ep.components.index(c)] for c in ("suspension", "tires", "resistance")], axis=1)
    if err_window is not None:
        end_q[err_window] += 0.05
        s0 = starts[err_window]
        comps[s0:s0 + 10, err_comp] += 2.0
    return GpuRun("windowed", 0.0125, 10, starts, ends, end_q, ep.qvel[ends], comps,
                  np.zeros((39, 2, 4)), ends, end_q, ep.qvel[ends], end_q[None], comps[None])


def test_perfect_gpu_reports_zero():
    """Identical data → zero Δ, no level crossed."""
    model = mujoco.MjModel.from_xml_string(XML)
    ep = _episode()
    rep = build_report(model, ep, _run(ep), None)
    assert rep["max_end_dqpos"] == 0.0
    assert all(v is None for v in rep["first_window_over"].values())


def test_error_is_located():
    """A 0.05 error in window 2 on the tyre slot is the worst window and blames tires."""
    model = mujoco.MjModel.from_xml_string(XML)
    ep = _episode()
    rep = build_report(model, ep, _run(ep, err_window=2, err_comp=1), None)
    worst = rep["worst_windows"][0]
    assert worst["window"] == 2 and worst["worst_dof"] == "root_x"
    assert worst["rel_rms"]["tires"] > worst["rel_rms"]["suspension"]
    assert rep["first_window_over"][str(LEVELS[1])] == 2
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.17.0 python -m pytest tools/colab_gpu/golden/tests/test_compare.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

```python
"""Golden-vs-GPU report (plan 2026-11-02 D8). No pass/fail threshold."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from tools.colab_gpu.golden.recording import GPU_COMPONENTS, component_qfrc

LEVELS = (1e-6, 1e-3, 1e-2)
_EPS = 1e-9


def _dof_names(model) -> list[str]:
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.dof_jntid[i])) or f"dof{i}"
            for i in range(model.nv)]


def _rel_rms(gpu: np.ndarray, cpu: np.ndarray) -> float:
    return float(np.sqrt(np.mean((gpu - cpu) ** 2)) / max(np.sqrt(np.mean(cpu ** 2)), _EPS))


def build_report(model, ep, windowed, free=None) -> dict:
    names = _dof_names(model)
    root_x = names.index("root_x") if "root_x" in names else 0
    golden_end = ep.qpos[windowed.window_ends]
    dq = np.abs(windowed.end_qpos - golden_end)
    per_window_max = dq.max(axis=1)
    cpu = {c: component_qfrc(ep, c) for c in GPU_COMPONENTS}
    windows = []
    warm_ratio = []
    for k, (i0, i1) in enumerate(zip(windowed.window_starts, windowed.window_ends)):
        rel = {c: _rel_rms(windowed.components[i0:i1, j], cpu[c][i0:i1]) for j, c in enumerate(GPU_COMPONENTS)}
        n = int(i1 - i0)
        head = max(1, n // 10)
        tail = max(1, n // 2)
        tyre = GPU_COMPONENTS.index("tires")
        h = _rel_rms(windowed.components[i0:i0 + head, tyre], cpu["tires"][i0:i0 + head])
        t = _rel_rms(windowed.components[i1 - tail:i1, tyre], cpu["tires"][i1 - tail:i1])
        warm_ratio.append(h / max(t, _EPS))
        windows.append({
            "window": k, "t_start_s": float(ep.times_s[i0]), "track_x_m": float(ep.qpos[i0, root_x]),
            "max_dqpos": float(per_window_max[k]), "worst_dof": names[int(dq[k].argmax())], "rel_rms": rel,
        })
    first_over = {}
    for level in LEVELS:
        idx = np.flatnonzero(per_window_max > level)
        first_over[str(level)] = int(idx[0]) if idx.size else None
    report = {
        "mode": windowed.mode, "window_s": windowed.window_s, "windows": windows,
        "worst_windows": sorted(windows, key=lambda w: w["max_dqpos"], reverse=True)[:10],
        "max_end_dqpos": float(per_window_max.max()),
        "first_window_over": first_over,
        # D7: >>1 means bristle/freehub memory carried across windows is wrong.
        "tyre_warmup_ratio_median": float(np.median(warm_ratio)),
    }
    if free is not None:
        last = int(free.window_ends[-1])
        report["free_run"] = {
            "final_root_x_gpu_m": float(free.end_qpos[-1, root_x]),
            "final_root_x_cpu_m": float(ep.qpos[last, root_x]),
            "mean_speed_gpu_mps": float(np.mean(free.frame_qvel[:, root_x])),
            "mean_speed_cpu_mps": float(np.mean(ep.qvel[:last, root_x])),
        }
    return report


def write_report(report: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    lines = [
        "# GPU golden replay report", "",
        f"mode `{report['mode']}`, window {report['window_s']} s, max end Δqpos {report['max_end_dqpos']:.3e}, "
        f"tyre warm-up ratio (median) {report['tyre_warmup_ratio_median']:.2f}", "",
        "| level | first window over |", "|---|---|",
        *[f"| {k} | {v} |" for k, v in report["first_window_over"].items()], "",
        "| window | t s | track x m | max Δqpos | worst dof | tires | suspension | resistance |",
        "|---|---|---|---|---|---|---|---|",
        *[f"| {w['window']} | {w['t_start_s']:.3f} | {w['track_x_m']:.2f} | {w['max_dqpos']:.3e} | {w['worst_dof']} | "
          f"{w['rel_rms']['tires']:.3f} | {w['rel_rms']['suspension']:.3f} | {w['rel_rms']['resistance']:.3f} |"
          for w in report["worst_windows"]],
    ]
    if "free_run" in report:
        lines += ["", "## Free run (no re-seat)", "", *[f"- {k}: {v:.3f}" for k, v in report["free_run"].items()]]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")
```

`cli.py` (`python -m tools.colab_gpu.golden.cli`):
- `run --bundle DIR --recording DIR --out DIR [--window-s 0.1] [--frame-stride 10] [--config-bin FILE] [--nworld 1]`. It runs the windowed run and the free run, saves `windowed.npz` and `free.npz` (every `GpuRun` field), writes the trajectory (Task 9 adds the writer call) and writes the report.
- `report --bundle DIR --recording DIR --out DIR` rebuilds the report from the saved npz files.
- `closure --bundle DIR --recording DIR` prints the Task 3 numbers.

Use `argparse` and keep each subcommand ≤ 25 lines by calling the module functions.

- [ ] **Step 4: Run the tests**

Run: the same command as Step 2.
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add tools/colab_gpu/golden/compare.py tools/colab_gpu/golden/cli.py tools/colab_gpu/golden/tests/test_compare.py
git commit -m "feat(colab_gpu): golden-vs-GPU window report"
```

---

### Task 7: Baseline before the tyre fix

**Files:**
- Create: `docs/superpowers/audits/2026-11-02-gpu-golden-baseline.md`

- [ ] **Step 1: Run both tracks with the current GPU defaults**

```bash
for t in flat rough_uphill_extreme; do
  rec=$(dirname "$(find output/gpu_golden/$t -name manifest.json | sort | tail -1)")
  uv run --no-project --python .venv-metal/bin/python python -m tools.colab_gpu.golden.cli run \
    --bundle output/gpu_golden/$t/bundle --recording "$rec" --out output/gpu_golden/$t/baseline
done
```

Expected: `report.md` and `report.json` in each `baseline/` directory.

- [ ] **Step 2: Write the baseline audit**

Copy both `report.md` tables into the audit doc. Answer, from the numbers only:
1. Is `tires` the component with the largest `rel_rms` in the worst windows? The tool is expected to see the D11 mismatch (radius 0.351 vs model, μ 0.95 vs 1.1, density `{55k,85k,140k}` vs `{300k,0,0}`, 32 vs 128 stations).
2. What is the tyre warm-up ratio? It answers the D7 question.
3. What do suspension and resistance show? They are independent findings.

If `tires` is **not** clearly the worst component, write that down. The tool failed to see a known mismatch, so investigate before Task 8. Do not proceed on the assumption that it worked.

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/audits/2026-11-02-gpu-golden-baseline.md
git commit -m "docs(audit): GPU golden replay baseline before tyre parameter fix"
```

---

### Task 8: GPU tyre parameters from the recorded config (D11)

**Files:**
- Create: `native/src/gpu/golden_params.hpp`, `native/tests/gpu/test_gpu_golden_params.cpp`
- Modify: `native/src/tools/gpu_export_main.cpp` (also write `gpu_world_config.bin`), `native/src/replay/model_bundle.{hpp,cpp}` (keep the request's `SimulationPhysicsConfig` in the bundle)
- Modify: `docs/superpowers/audits/2026-11-02-gpu-golden-baseline.md` (add a law-audit table)

**Interfaces:**
- Produces:
  - `bike::gpu::GpuWorldConfig golden_world_config(const configuration::SimulationPhysicsConfig&, const mjModel&)`, defined in `golden_params.hpp` (host only);
  - the bundle file `gpu_world_config.bin` (`sizeof(GpuWorldConfig)` raw bytes, `nworld = 1`), consumed by `run_golden_gpu(config_bin=…)`.

- [ ] **Step 1: Law audit (the R2 gate)**

Read `native/src/writers/distributed_tire.cpp` from the constructor (~`:190-260`) through `accumulate_validated` (`:587-740`), and `native/src/gpu/tire_kernel.hpp` from `:140` to the end. In the audit doc, write one table row per quantity the CPU law consumes, with columns: CPU source (file:line) | GPU field | same law? (yes/no + one line).

The quantities are at least:
- station count, angles and weights;
- hinge density knots, stiffness and damping;
- wheel radius (which geom or spec the CPU uses);
- friction μ and its Stribeck/material dependence;
- relaxation length and the tangential state law (CPU `xi` relaxation vs GPU bristle `kx`/`cx`);
- normal direction and terrain sampling.

**Gate:** if any row is "no" (a different law, not a different number), stop. Report to the user with the table. A re-port is a new plan. Do not continue Step 2.

- [ ] **Step 2: Write the failing parameter test**

```cpp
// golden_world_config copies every CPU tyre parameter the audit marked
// "same law", per wheel, from the recorded physics config and model.
#include "../../src/gpu/golden_params.hpp"
// … same standalone-test scaffolding as test_gpu_batch_step.cpp …
void density_and_stations_come_from_config() {
    bike::configuration::SimulationPhysicsConfig physics{};   // defaults: distributed station_count 128
    const char* xml = "<mujoco><worldbody><body><freejoint/><geom size='.1'/></body></worldbody></mujoco>";
    auto model = bike::model::compile_mjcf(xml);              // model is only read for radii in audited rows
    const auto cfg = bike::gpu::golden_world_config(physics, *model);
    const auto& d = physics.tires.distributed;               // confirm member path in config_types.hpp:266-345
    require(cfg.front_tire.num_stations == d.station_count, "front station_count");
    require(cfg.rear_tire.num_stations == d.station_count, "rear station_count");
    const auto fw = d.for_wheel("front");
    require(cfg.front_tire.density.num_knots == static_cast<int>(fw.density.knots_m.size()), "knot count");
    for (std::size_t i = 0; i < fw.density.knots_m.size(); ++i) {
        require_near(cfg.front_tire.density.knots_m[i], static_cast<float>(fw.density.knots_m[i]), 0.f, "knot");
        require_near(cfg.front_tire.density.stiffness_n_m[i], static_cast<float>(fw.density.stiffness_n_m[i]), 0.f, "k");
    }
    require_near(cfg.front_tire.density.damping_ns_m, static_cast<float>(fw.density.damping_ns_m), 0.f, "c");
}
```

Add one `require_near` per remaining "same law" row of the Step 1 table (radius, μ, relaxation length), each citing the CPU source line in its message. Each row of the audit table is exactly one assertion here and one assignment in Step 3.

- [ ] **Step 3: Implement `golden_world_config`**

```cpp
#pragma once
// Host-only: GpuWorldConfig for golden replay from the recorded physics config
// (plan 2026-11-02 D11). Every assignment cites its CPU source; the audit
// table in docs/superpowers/audits/2026-11-02-gpu-golden-baseline.md is the
// authority for which fields share a law.
#include "batch_step.hpp"
#include "../configuration/physical_config.hpp"
#include <mujoco/mujoco.h>
#include <stdexcept>

namespace bike::gpu {
inline void copy_density(const configuration::HingeDensity& src, GpuHingeDensity& dst) {
    if (src.knots_m.size() != src.stiffness_n_m.size() || src.knots_m.size() > kMaxHingeKnots)
        throw std::invalid_argument("golden_world_config: hinge density exceeds kMaxHingeKnots");
    dst.num_knots = static_cast<int>(src.knots_m.size());
    for (std::size_t i = 0; i < src.knots_m.size(); ++i) {
        dst.knots_m[i] = static_cast<float>(src.knots_m[i]);
        dst.stiffness_n_m[i] = static_cast<float>(src.stiffness_n_m[i]);
    }
    dst.damping_ns_m = static_cast<float>(src.damping_ns_m);
}

inline GpuWorldConfig golden_world_config(const configuration::SimulationPhysicsConfig& physics, const mjModel& model) {
    GpuWorldConfig cfg{};
    gpu_init_default_world_config(cfg);
    const auto& d = physics.tires.distributed;
    if (d.station_count > kMaxTreadStations)
        throw std::invalid_argument("golden_world_config: station_count exceeds kMaxTreadStations");
    for (const char* side : {"front", "rear"}) {
        auto& t = std::string_view(side) == "front" ? cfg.front_tire : cfg.rear_tire;
        t.num_stations = d.station_count;
        t.num_lateral_ribs = 1;
        copy_density(d.for_wheel(side).density, t.density);
        // Remaining "same law" rows from the audit table, one assignment each,
        // e.g. t.nominal_radius_m = <CPU source>; t.mu_static = <CPU source>;
    }
    gpu_init_suspension_preloads(cfg.suspension);
    (void)model;
    return cfg;
}
} // namespace bike::gpu
```

The commented line in the loop is replaced by the audited assignments before commit. The test from Step 2 fails until they exist, so the commit cannot land with the comment.

Exporter:
- `build_model_bundle` keeps `request->setup.model.physics` in `ModelBundle::physics` (`std::optional<configuration::SimulationPhysicsConfig>`);
- `bike-gpu-export` writes `gpu_world_config.bin` from `golden_world_config(*bundle.physics, *compiled_model)`;
- if `physics` is absent it fails with `error: recording carries no physics config`.

- [ ] **Step 4: Run native tests**

Run: `cmake --build build/native-release --target bike_test_gpu_golden_params bike-gpu-export && ctest --test-dir build/native-release -R "gpu|model_bundle" --output-on-failure`
Expected: PASS.

- [ ] **Step 5: Re-export, rerun and compare with the baseline**

```bash
for t in flat rough_uphill_extreme; do
  rec=$(dirname "$(find output/gpu_golden/$t -name manifest.json | sort | tail -1)")
  build/native-release/bin/bike-gpu-export --recording "$rec" --out output/gpu_golden/$t/bundle
  uv run --no-project --python .venv-metal/bin/python python -m tools.colab_gpu.golden.cli run \
    --bundle output/gpu_golden/$t/bundle --recording "$rec" --out output/gpu_golden/$t/fixed \
    --config-bin output/gpu_golden/$t/bundle/gpu_world_config.bin
done
```

Add a "after fix" section to the baseline audit with both tables side by side. Expected: tyre `rel_rms` drops in the worst windows. If it does not, record that as the next finding. Do not tune parameters by hand to make it drop.

- [ ] **Step 6: Commit**

```bash
git add native/src/gpu/golden_params.hpp native/tests/gpu/test_gpu_golden_params.cpp native/src/tools/gpu_export_main.cpp \
  native/src/replay/model_bundle.* <test registration> docs/superpowers/audits/2026-11-02-gpu-golden-baseline.md
git commit -m "fix(gpu): golden tyre parameters come from the recorded distributed config"
```

---

### Task 9: External trajectory input for `bike-replay --view`

**Files:**
- Create: `native/src/replay/trajectory.hpp`, `native/src/replay/trajectory.cpp`, `native/tests/integration/test_external_trajectory.cpp`, `tools/colab_gpu/golden/trajectory.py`, `tools/colab_gpu/golden/tests/test_trajectory.py`
- Modify: `native/src/replay/playback.cpp` (extract `fill_quaternion_starts`), `native/src/cli/replay.cpp`, `native/src/cli/argument_contracts.inc` (`bike-replay` options), `tools/colab_gpu/golden/cli.py` (`run` writes the trajectory)

**Interfaces:**
- Wire format, `trajectory.bin` (little-endian):

| offset | type | field |
|---|---|---|
| 0 | char[8] | magic `BGTRAJ01` |
| 8 | u32 | version = 1 |
| 12 | u32 | nq |
| 16 | u32 | nv |
| 20 | u32 | wheel_cols = 8 (front cx, cz, fx, fz, rear cx, cz, fx, fz) |
| 24 | u64 | frames |
| 32 | char[64] | lowercase hex SHA-256 of `model.mjb` |
| 96 | f64[frames × (2 + nq + nv + 8)] | rows: time_s, interval_end_s, qpos, qvel, wheel |

- The directory holds `trajectory.bin`, `model.mjb` (copied from the bundle) and `trajectory.json`. The JSON is human-only metadata; the native side ignores it.
- C++: `struct ExternalTrajectory { PlaybackData playback; std::vector<std::array<double, 8>> wheels; }` and `ExternalTrajectory load_external_trajectory(const std::filesystem::path& dir)`.
- Python: `write_trajectory(out_dir: Path, bundle_dir: Path, ep: GoldenEpisode, run: GpuRun) -> None`.
- CLI: `bike-replay GOLDEN_RECORDING --view --trajectory DIR`. The positional recording becomes the ghost (Task 10), and `--trajectory` is the shown trajectory. `--trajectory` without `--view` is an `ArgumentError`.

- [ ] **Step 1: Write the failing native test**

```cpp
// An external trajectory loads into PlaybackData that validate_playback
// accepts, refuses a model hash mismatch, and refuses truncated rows.
#include "../reference/replay_reference_support.hpp"
#include "io/hash.hpp"
#include "replay/playback.hpp"
#include "replay/trajectory.hpp"
#include <cstring>
#include <fstream>

namespace {
namespace rr = replay_reference;
namespace replay = bike::replay;

// Writes BGTRAJ01 exactly as tools/colab_gpu/golden/trajectory.py does.
void write_trajectory_file(const std::filesystem::path& dir, const bike::runtime::ModelImage& image,
                           const replay::PlaybackData& src, std::size_t frames, std::string sha_hex,
                           std::size_t drop_bytes = 0) {
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "model.mjb", std::ios::binary)
        .write(reinterpret_cast<const char*>(image.mjb.data()), static_cast<std::streamsize>(image.mjb.size()));
    const auto nq = static_cast<std::uint32_t>(src.qpos.front().size());
    const auto nv = static_cast<std::uint32_t>(src.qvel.front().size());
    std::string bytes = "BGTRAJ01";
    const auto put = [&](const auto& v) { bytes.append(reinterpret_cast<const char*>(&v), sizeof v); };
    put(std::uint32_t{1}); put(nq); put(nv); put(std::uint32_t{8}); put(static_cast<std::uint64_t>(frames));
    sha_hex.resize(64, '0'); bytes += sha_hex;
    for (std::size_t f = 0; f < frames; ++f) {
        put(src.times_s[f]); put(src.ends_s[f]);
        for (double q : src.qpos[f]) put(q);
        for (double v : src.qvel[f]) put(v);
        for (int w = 0; w < 8; ++w) put(static_cast<double>(w));
    }
    bytes.resize(bytes.size() - drop_bytes);
    std::ofstream(dir / "trajectory.bin", std::ios::binary).write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

void loads_valid_trajectory() {
    rr::Directory tmp;
    const auto src = replay::load_playback(rr::record_container(tmp.path).directory, 0);
    const std::size_t frames = std::min<std::size_t>(3, src.times_s.size());
    write_trajectory_file(tmp.path / "traj", src.model, src, frames, src.model.sha256);
    const auto ext = replay::load_external_trajectory(tmp.path / "traj");
    bt::require(ext.playback.times_s.size() == frames && ext.wheels.size() == frames, "frame counts");
    bt::require(ext.playback.duration_s == src.ends_s[frames - 1], "duration is the last end");
    bt::require(ext.playback.qpos[1] == src.qpos[1], "rows round-trip exactly");
    bt::require(ext.wheels[0][7] == 7.0, "wheel columns round-trip");
}

void rejects_model_hash_mismatch() {
    rr::Directory tmp;
    const auto src = replay::load_playback(rr::record_container(tmp.path).directory, 0);
    write_trajectory_file(tmp.path / "traj", src.model, src, 2, std::string(64, 'a'));
    bt::require_throws<bike::recording::RecordingError>(
        [&] { (void)replay::load_external_trajectory(tmp.path / "traj"); }, "trajectory model hash");
}

void rejects_truncated_rows() {
    rr::Directory tmp;
    const auto src = replay::load_playback(rr::record_container(tmp.path).directory, 0);
    write_trajectory_file(tmp.path / "traj", src.model, src, 2, src.model.sha256, 8);
    bt::require_throws<bike::recording::RecordingError>(
        [&] { (void)replay::load_external_trajectory(tmp.path / "traj"); }, "trajectory size");
}
} // namespace
```

Copy `main` from `test_replay_playback.cpp` and register these three functions. If the shared helpers have no `bt::require_throws`, use the throw-assertion helper `test_replay_playback.cpp` uses for its "refuse missing/empty inputs" cases (read that file's refusal tests).

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build/native-release --target bike_test_external_trajectory`
Expected: compile error, `replay/trajectory.hpp` not found.

- [ ] **Step 3: Implement the loader**

```cpp
#pragma once
#include "playback.hpp"
#include <array>
#include <filesystem>
#include <vector>

namespace bike::replay {
// Externally produced trajectory (GPU golden replay). Presentation only: it is
// not a recording, carries no seal and never certifies anything.
struct ExternalTrajectory {
    PlaybackData playback;
    std::vector<std::array<double, 8>> wheels;
};
[[nodiscard]] ExternalTrajectory load_external_trajectory(const std::filesystem::path& dir);
void fill_quaternion_starts(PlaybackData&, const mjModel&);
} // namespace bike::replay
```

`trajectory.cpp`:
- read `model.mjb` bytes; read `trajectory.bin` fully;
- check magic, version, `wheel_cols == 8`;
- check `size == 96 + frames * (2 + nq + nv + 8) * 8` with overflow-checked arithmetic (`model_access::checked_product` as in `playback.cpp:43`) → `RecordingError("trajectory size")`;
- check `io::sha256(mjb)` equals the header hex → `RecordingError("trajectory model hash")`;
- fill `playback.times_s`, `ends_s`, `qpos`, `qvel` and `wheels`;
- `playback.model = {.mjb, .sha256}`;
- `auto model = make_playback_model(playback); fill_quaternion_starts(playback, *model); playback.duration_s = playback.ends_s.back(); validate_playback(playback, nq, nv);`.

Move the quaternion loop at `playback.cpp:151-158` into `fill_quaternion_starts` and call it from `load_playback` as well.

`cli/replay.cpp::inspect`: replace `replay::PlaybackSession playback(args.text("recording"), episode);` with:

```cpp
    std::optional<replay::ExternalTrajectory> external;
    std::optional<replay::PlaybackData> ghost;
    if (args.has("trajectory")) {
        external = replay::load_external_trajectory(args.text("trajectory"));
        ghost = replay::load_playback(args.text("recording"), episode);
        if (ghost->model.sha256 != external->playback.model.sha256)
            throw ArgumentError("--trajectory model differs from the recording model");
    }
    replay::PlaybackSession playback = external
        ? replay::PlaybackSession(external->playback)
        : replay::PlaybackSession(args.text("recording"), episode);
```

`PlaybackSession` is non-movable. If the conditional initialisation does not compile, construct it through `std::optional<replay::PlaybackSession>` with `emplace` in the two branches.

In `replay_command`, add before dispatch: `if (args.has("trajectory") && !args.flag("view")) throw ArgumentError("--trajectory requires --view");`.

`argument_contracts.inc`: in the `"bike-replay"` options array, add an entry with the same fields as `--episode`: `"names": ["--trajectory"]`, `"dest": "trajectory"`, `"type": "Path"`, `"flag": false`, `"required": false`.

Python `trajectory.py`:

```python
"""Write the BGTRAJ01 directory read by `bike-replay --view --trajectory`."""
from __future__ import annotations

import hashlib
import json
import shutil
import struct
from pathlib import Path

import numpy as np

MAGIC = b"BGTRAJ01"


def write_trajectory(out_dir: Path, bundle_dir: Path, ep, run) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    mjb = (Path(bundle_dir) / "model.mjb").read_bytes()
    shutil.copyfile(Path(bundle_dir) / "model.mjb", out_dir / "model.mjb")
    sha = hashlib.sha256(mjb).hexdigest().encode("ascii")
    # Frame 0 is the golden start state; then the GPU frames.
    steps = np.concatenate([[0], run.frame_steps])
    qpos = np.vstack([ep.qpos[0], run.frame_qpos])
    qvel = np.vstack([ep.qvel[0], run.frame_qvel])
    times = ep.times_s[steps]
    ends = np.append(times[1:], times[-1] + ep.dt_s)
    wheel_rows = np.zeros((steps.size, 8))
    for r, s in enumerate(steps[1:], start=1):
        wheel_rows[r] = run.wheels[s - 1].reshape(-1)  # (2,4) → front cx,cz,fx,fz, rear cx,cz,fx,fz
    nq, nv = qpos.shape[1], qvel.shape[1]
    header = MAGIC + struct.pack("<IIIIQ", 1, nq, nv, 8, steps.size) + sha
    rows = np.hstack([times[:, None], ends[:, None], qpos, qvel, wheel_rows]).astype("<f8")
    (out_dir / "trajectory.bin").write_bytes(header + rows.tobytes())
    (out_dir / "trajectory.json").write_text(json.dumps({
        "format": "BGTRAJ01", "frames": int(steps.size), "nq": nq, "nv": nv, "mode": run.mode,
        "window_s": run.window_s, "frame_stride": run.frame_stride, "model_sha256": sha.decode(),
    }, indent=2))
```

Python test `test_trajectory.py`: write a 3-frame synthetic trajectory, parse it back with `struct`/`numpy`, and assert the header fields, the row count, `ends[-1] > times[-1]` and strictly increasing times.

- [ ] **Step 4: Run tests**

Run:
- `cmake --build build/native-release --target bike_test_external_trajectory bike-replay && ctest --test-dir build/native-release -R "external_trajectory|replay_playback" --output-on-failure`
- `uv run --no-project --with pytest --with numpy python -m pytest tools/colab_gpu/golden/tests/test_trajectory.py -q`

Expected: PASS.

- [ ] **Step 5: End-to-end dry run**

```bash
rec=$(dirname "$(find output/gpu_golden/flat -name manifest.json | sort | tail -1)")
build/native-release/bin/bike-replay "$rec" --view --dry-run --trajectory output/gpu_golden/flat/fixed/trajectory
```

Expected: `frames=… duration=…s ok`. Update `cli.py run` to call `write_trajectory(out/"trajectory", bundle, ep, windowed)` and rerun it once for `flat`.

- [ ] **Step 6: Commit**

```bash
git add native/src/replay/trajectory.* native/src/replay/playback.cpp native/src/cli/replay.cpp native/src/cli/argument_contracts.inc \
  native/tests/integration/test_external_trajectory.cpp <test registration> tools/colab_gpu/golden/trajectory.py \
  tools/colab_gpu/golden/tests/test_trajectory.py tools/colab_gpu/golden/cli.py
git commit -m "feat(replay): bike-replay --view --trajectory for external GPU trajectories"
```

---

### Task 10: Ghost, Δ HUD and tyre-force arrows in the viewer

**Files:**
- Create: `native/src/viewer/decorations.hpp`, `native/src/replay/golden_overlay.hpp`, `native/src/replay/golden_overlay.cpp`, `native/tests/integration/test_golden_overlay.cpp`
- Modify: `native/src/viewer/session.hpp` (`SessionHooks::decorations`), `native/src/viewer/ride.cpp:144-146`, `native/src/viewer/render_owner.hpp`/`.cpp` (`set_decorations` + draw), `native/src/cli/replay.cpp` (install the hook)

**Interfaces:**
- `viewer::Arrow { std::array<double, 3> from, to; std::array<float, 4> rgba; }`
- `viewer::Decorations { std::optional<std::vector<double>> ghost_qpos; std::vector<Arrow> arrows; std::string overlay; }`
- `viewer::Decorations replay::golden_decorations(const PlaybackData& shown, const PlaybackData& ghost, const std::vector<std::array<double, 8>>& wheels, const std::vector<std::string>& dof_names, double time_s, double& running_max)`.
- `RenderOwner::set_decorations(viewer::Decorations)`.

- [ ] **Step 1: Write the failing pure-function test**

```cpp
// golden_decorations: ghost pose is the recording's pose at the same time;
// Δ text names the worst dof and the running max; arrows start at the
// contact point and scale 0.0005 m per N; a no-contact wheel draws nothing.
#include "replay/golden_overlay.hpp"
#include "replay/playback.hpp"
#include "../reference/replay_reference_support.hpp"

namespace {
namespace replay = bike::replay;

replay::PlaybackData two_frames(double x1) {
    replay::PlaybackData d;
    d.times_s = {0.0, 0.1};
    d.ends_s = {0.1, 0.2};
    d.qpos = {{0.0, 0.0}, {x1, 0.0}};
    d.qvel = {{0.0, 0.0}, {0.0, 0.0}};
    d.duration_s = 0.2;
    return d;
}

void ghost_pose_and_delta() {
    const auto ghost = two_frames(1.0);
    const auto shown = two_frames(1.01);
    const std::vector<std::array<double, 8>> wheels(2, std::array<double, 8>{});
    double running_max = 0.0;
    const auto dec = replay::golden_decorations(shown, ghost, wheels, {"root_x", "root_z"}, 0.1, running_max);
    bt::require(dec.ghost_qpos && *dec.ghost_qpos == replay::pose_at(ghost, 0.1), "ghost is the recording pose");
    bt::require(std::abs(running_max - 0.01) < 1e-12, "running max tracks the worst delta");
    bt::require(dec.overlay.find("root_x") != std::string::npos, "overlay names the worst dof");
    bt::require(dec.overlay.find("1.000e-02") != std::string::npos, "overlay prints the delta");
    bt::require(dec.arrows.empty(), "no contact force, no arrows");
}

void arrows_from_contact_points() {
    const auto data = two_frames(1.0);
    std::array<double, 8> row{1.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<std::array<double, 8>> wheels{row, row};
    double running_max = 0.0;
    const auto dec = replay::golden_decorations(data, data, wheels, {"root_x", "root_z"}, 0.0, running_max);
    bt::require(dec.arrows.size() == 1, "only the front wheel is in contact");
    const auto& a = dec.arrows.front();
    bt::require(a.from == std::array<double, 3>{1.0, 0.0, 0.0}, "arrow starts at the contact point");
    bt::require(std::abs(a.to[2] - 0.5) < 1e-12 && a.to[0] == 1.0, "0.0005 m per N along the force");
}
} // namespace
```

Copy `main` from `test_replay_playback.cpp` and register both functions.

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build/native-release --target bike_test_golden_overlay`
Expected: compile error.

- [ ] **Step 3: Implement**

`decorations.hpp`, GL-free:

```cpp
#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace bike::viewer {
struct Arrow { std::array<double, 3> from{}, to{}; std::array<float, 4> rgba{1.f, .3f, .1f, 1.f}; };
// Extra scene content for one frame: a translucent ghost pose of the same
// model, force arrows, and text appended to the overlay. Presentation only.
struct Decorations {
    std::optional<std::vector<double>> ghost_qpos;
    std::vector<Arrow> arrows;
    std::string overlay;
};
} // namespace bike::viewer
```

`golden_overlay.cpp`:
- `ghost_qpos = pose_at(ghost, t)`;
- `shown_q = pose_at(shown, t)`;
- `d = |shown_q - ghost_q|`, `k = argmax`, `running_max = max(running_max, d[k])`;
- `overlay = std::format("GPU vs CPU  max|Δq| {:.3e} ({})  run max {:.3e}", d[k], dof_names[k], running_max)`;
- arrows: wheel row index = `upper_bound(shown.times_s, t) - 1`, clamped. For each wheel `(cx, cz, fx, fz)` build the arrow from `{cx, 0, cz}` to `{cx + 5e-4*fx, 0, cz + 5e-4*fz}`, front orange and rear blue. Skip it when `fx == fz == 0` (no contact).

`render_owner.hpp`: add `void set_decorations(Decorations);` and include `decorations.hpp`.

`render_owner.cpp`:
- Impl gains `std::unique_ptr<mjData, DataDelete> ghost; Decorations decorations;`.
- `set_decorations`:
  - `require_live()`;
  - if `ghost_qpos` is set, check its size `== model->nq` (otherwise `std::invalid_argument("ghost pose size")`) and that every value is finite;
  - allocate `ghost` once with `engine::make_data(model)`;
  - store the decorations.
- `draw()`, inside the invoke lambda after `mjv_updateScene`:

```cpp
        if (owner->decorations.ghost_qpos && owner->ghost) {
            auto* g = owner->ghost.get();
            for (int i = 0; i < owner->model->nq; ++i) g->qpos[i] = (*owner->decorations.ghost_qpos)[static_cast<std::size_t>(i)];
            mj_kinematics(owner->model.get(), g);
            mj_comPos(owner->model.get(), g);
            const int first = owner->scene.ngeom;
            mjv_addGeoms(owner->model.get(), g, &owner->option, nullptr, mjCAT_DYNAMIC, &owner->scene);
            for (int i = first; i < owner->scene.ngeom; ++i) owner->scene.geoms[i].rgba[3] *= .35f;
        }
        for (const auto& a : owner->decorations.arrows) {
            if (owner->scene.ngeom >= owner->scene.maxgeom) break;
            mjvGeom* geom = owner->scene.geoms + owner->scene.ngeom;
            mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, a.rgba.data());
            mjv_connector(geom, mjGEOM_ARROW, .01, a.from.data(), a.to.data());
            ++owner->scene.ngeom;
        }
```

Use the existing NOLINT/pointer-access conventions of this file for `qpos`/`geoms`: copy whatever `model_access::mutable_buffer` pattern `set_markers` uses instead of raw indexing if the analysis sweeps require it.

`session.hpp`: `std::function<Decorations(const runtime::RenderFrame&)> decorations;` in `SessionHooks`.

`ride.cpp:144-146` becomes:

```cpp
            renderer->apply(frame);
            Decorations decorations = hooks.decorations ? hooks.decorations(frame) : Decorations{};
            const std::string extra = decorations.overlay;
            renderer->set_decorations(std::move(decorations));
            renderer->set_overlay(overlay_text(frame,input,achieved,options.mode) + (extra.empty() ? "" : "\n" + extra));
```

`cli/replay.cpp::inspect`, when `external`: compute `dof_names` from `make_playback_model(external->playback)` (`mj_id2name(m, mjOBJ_JOINT, m->dof_jntid[i])`), keep `double running_max = 0`, and set:

```cpp
        hooks.decorations = [&](const runtime::RenderFrame&) {
            return replay::golden_decorations(external->playback, *ghost, external->wheels, dof_names, time, running_max);
        };
```

`hooks.reset` additionally sets `running_max = 0`.

- [ ] **Step 4: Run tests and both builds**

Run:
- `cmake --build build/native-release --target bike_test_golden_overlay bike-replay && ctest --test-dir build/native-release -R "golden_overlay|external_trajectory|replay_playback" --output-on-failure`
- `cmake --preset gui && cmake --build build/gui --target bike-replay`

Expected: tests pass; the GUI build compiles (draw code is only compiled under `BIKE_BUILD_GUI`).

- [ ] **Step 5: Manual acceptance (human)**

```bash
rec=$(dirname "$(find output/gpu_golden/rough_uphill_extreme -name manifest.json | sort | tail -1)")
build/gui/bin/bike-replay "$rec" --view --trajectory output/gpu_golden/rough_uphill_extreme/fixed/trajectory
```

The user confirms four things: a translucent ghost bike is visible, the HUD shows `GPU vs CPU max|Δq| …`, tyre-force arrows appear under the wheels, and arrows ←/→ seek. Record the user's confirmation, or the defect they report, in the task report. Do not mark this step done on the agent's own judgment.

- [ ] **Step 6: Commit**

```bash
git add native/src/viewer/decorations.hpp native/src/viewer/session.hpp native/src/viewer/ride.cpp native/src/viewer/render_owner.* \
  native/src/replay/golden_overlay.* native/src/cli/replay.cpp native/tests/integration/test_golden_overlay.cpp <test registration>
git commit -m "feat(viewer): golden replay ghost, delta HUD and tyre-force arrows"
```

---

### Task 11: Docs, Task 20 rewrite, final gate

**Files:**
- Modify: `tools/colab_gpu/README.md` (new section "Golden replay"), `docs/superpowers/plans/2026-11-01-gpu-gate-compliance.md` (Task 20 Step 3, `:428-434`), `docs/TESTING.md` (GPU kernel row: golden kernel + new tests)

- [ ] **Step 1: README section**

Add the end-to-end commands in order: record goldens (Task 1 Step 1) → `bike-gpu-export` → `cli closure` → `cli run` → `bike-replay … --view --trajectory`. Include the two interpreter commands from Global Constraints, and say what each report field means. Make the "No pass/fail threshold (D8)" sentence explicit.

- [ ] **Step 2: Rewrite Task 20 Step 3**

Replace the `/tmp/gc/T20/` harness text at `:428-434` with:

```markdown
- [ ] **Step 3:** Host-vs-Metal trajectory comparison uses the committed golden replay tool (plan `2026-11-02-gpu-golden-replay.md`): `python -m tools.colab_gpu.golden.cli run … --window-s 5` gives the 5 s `max|Δqpos|`; the threshold for this gate is chosen from the `report.json` of both tracks and written here with the numbers it came from.
```

- [ ] **Step 3: Final native gate**

Run: `bash tools/run_tests.sh native`
Expected: exit 0.
- On failure, compare against the documented baseline list (AGENTS.md "Verification speed"). Fix regressions this plan caused.
- Report every failure with its log path. Never claim completion on red.

- [ ] **Step 4: Full Python suites**

Run:
- `uv run --no-project --with pytest --with numpy --with duckdb --with mujoco==3.17.0 python -m pytest tools/colab_gpu/golden/tests -q` (the Metal suite skips itself via `importorskip("warp_metal")`)
- `uv run --no-project --python .venv-metal/bin/python python -m pytest tools/colab_gpu/golden/tests -q`

Expected: all pass. Report the counts and any skips with their reason.

- [ ] **Step 5: Commit**

```bash
git add tools/colab_gpu/README.md docs/superpowers/plans/2026-11-01-gpu-gate-compliance.md docs/TESTING.md
git commit -m "docs: golden replay workflow; Task 20 uses the golden replay tool"
```

---

## Stop conditions (summary)

| Where | Condition | Action |
|---|---|---|
| Task 1 Step 1 | A golden run cannot be recorded natively | Report the error; no workaround through Python recording. |
| Task 3 Step 4 | The closure residual is above 1e-6 m/s | Stop. Report the localisation (version vs actuator/tendon/equality). Needs a recording-change plan. |
| Task 7 Step 2 | The tool does not blame `tires` for the known mismatch | Investigate the tool before Task 8. |
| Task 8 Step 1 | Any tyre quantity uses a different law on the GPU | Stop. Report the audit table. A re-port is a new plan. |
| Task 10 Step 5 | The user reports a visual defect | Fix within Task 10, or record it as a follow-up with the user. |
