# Spatial 3D — Phase 2 (Track B): 3D Terrain Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phase 0 merged (contract 5.1 header `native/src/terrain/grid3d.hpp`). Tasks 1, 2, 4, 5 are independent of P1; Task 3 (builder hookup) and Task 6 need P1 Task 4 (builder `spatial` argument, `extrude.py`) merged. Keep P1's `Grid3D::from_profile_extrusion` working.

**Goal:** A seeded procedural 3D terrain generator in Python producing a `.terrain3d` grid (straight corridor ±1.5 m, 5 mm × 10 mm, side slope, 2D roughness, shoulder rocks, ruts, extruded legacy features, surface zones), a render heightfield, and a native `Grid3D` with bicubic sampling, analytic normals and a deterministic ray cast — all unit- and property-tested, with planar track extrusion byte-equal to the 1D profile along y = 0.

**Architecture:** `terrain/generator3d.py` composes `z(x, y) = h(x) + y·tan β(x) + r(x, y) + Σ features` on a NumPy grid, reusing `generator.py`'s longitudinal spec and `heightfield.py`'s profile sampling; `terrain/terrain3d.py` is the binary writer/reader (spec §6.2) shared with `extrude.py`; `terrain/render.py` downsamples to the MJCF hfield. Native `Grid3D` loads the bytes once (immutable, thread-safe), evaluates Catmull-Rom bicubic patches with analytic partial derivatives for normals, and casts rays with a 2D DDA over cells plus eight bisection steps (fixed count → deterministic).

**Tech Stack:** Python/NumPy for generation (`quick`); C++23 for `Grid3D` (`native`).

**Spec:** `docs/superpowers/specs/2026-10-14-spatial-3d-design.md` §6. **ADR:** 0002 Layers B, D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p2-terrain-3d`.
- `generator.py` (planar) output for every existing seed is unchanged — add a sha256 snapshot test of `build_track(spec, seed)` for seeds 0–4 before touching shared helpers.
- Terrain generation is off the tick-path and stays Python; the only native code is `Grid3D`.
- Grid storage dtype: `float32` only if the validation in Task 2 shows slope error < 0.1 % against `float64`; otherwise `float64`. Record the decision in the audit.
- `Grid3D` warm methods are `noexcept`, allocation-free, `const`; the ray cast has a provable upper bound on iterations.

---

## File Structure

| File | Responsibility |
|---|---|
| Create `src/bike_sim/terrain/terrain3d.py` | `Terrain3dFile` dataclass; `write(path)`, `read(path)`; sha256; validation |
| Create `src/bike_sim/terrain/generator3d.py` | `Track3dSpec`, `build_track3d(spec, seed) -> Terrain3dFile` |
| Create `src/bike_sim/terrain/render.py` | `render_hfield(file, dy_render=0.02, dx_render=0.02) -> (nrow, ncol, data)` |
| Modify `src/bike_sim/terrain/extrude.py` (from P1) | Uses `terrain3d.py` writer |
| Modify `src/bike_sim/terrain/research.py` | `Track3dSpec` resolution for `--track-file *_3d.toml` (regenerate from genspec, verify hash) |
| Create `examples/research/alpine_rocky_3d.toml`, `rough_uphill_savage_3d.toml` | Track files pointing to the genspecs |
| Modify `src/bike_sim/terrain/surface.py` | `SurfaceTable` with rectangles and ids |
| Modify `src/bike_sim/mujoco/terrain.py`, `builder.py` | hfield from `render.py` when a `.terrain3d` is given |
| Create `examples/research/alpine_rocky_3d_genspec.toml`, `rough_uphill_savage_3d_genspec.toml` | 3D track specs (seeded) |
| Modify `native/src/terrain/grid3d.{hpp,cpp}` | Implement load/sample/ray_cast; keep `from_profile_extrusion` |
| Create `native/tests/test_grid3d.cpp` | CTest: analytic surfaces, normals, ray cast bounds, determinism |
| Create `tests/test_terrain3d.py` | Python: writer/reader, generator reproducibility, features, extrusion equality |
| Create `tests/reference/test_native_grid3d.py` | Python↔C++ sample agreement on random points (1e-12), ray cast vs brute force |
| Create `tools/terrain3d_preview.py` | Plotly surface preview for review (optional, `uv run --with plotly`) |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p2.md` | Evidence |

---

### Task 1: `.terrain3d` writer/reader and `SurfaceTable`

**Files:** create `terrain/terrain3d.py`; modify `terrain/surface.py`; tests `tests/test_terrain3d.py`

**Interfaces:** spec §6.2 layout exactly; `Terrain3dFile(x0, y0, dx, dy, z: ndarray[ny, nx], dtype, surfaces: SurfaceTable)`; `sha256()`; `read` validates magic/version/shape and rejects NaN/inf.

- [ ] **Step 1 (failing tests):** round-trip equality (bitwise) for f32 and f64; corrupt magic rejected; `sha256` stable.
- [ ] **Step 2:** Implement with `numpy.tofile`/`frombuffer` and an explicit little-endian header (`struct`).
- [ ] **Step 3:** Commit `feat(terrain): .terrain3d binary format and surface table`.

### Task 2: Generator 3D

**Files:** create `terrain/generator3d.py`, the two `*_3d_genspec.toml`; tests in `tests/test_terrain3d.py`

**Interfaces:** `Track3dSpec` = `TrackSpec` (existing longitudinal fields) + `corridor_half_width_m = 1.5`, `dx = 0.005`, `dy = 0.010`, `bank_sections: int range`, `bank_max_deg = 15`, `roughness_ly_over_lx: (0.5, 1.0)`, `features_3d: {rock: {...}, rut: {...}}` with the ranges of spec §6.1; `build_track3d(spec, seed)` uses exactly one `np.random.default_rng(seed)`.

- [ ] **Step 1 (failing tests):** same `(spec, seed)` → bitwise identical grid; different seeds differ; the row at `y = 0` of a spec with no lateral terms equals `generator.py`'s 1D profile bitwise (shared helpers); bank sections respect `|β| ≤ 15°` and smoothstep transitions; a rock's centre offset lies in `[−0.12, 0.12]`; rut walls ≤ 45°; crest radius floor along x holds (`MIN_CONTACT_RADIUS_M = 0.45`); lead-in 4 m and lead-out 5 m flat laterally and longitudinally.
- [ ] **Step 2:** Implement; 2D roughness via FFT filtering of white noise with an anisotropic Gaussian kernel; features as superellipsoid bumps/grooves; surface zones as rectangles.
- [ ] **Step 3:** dtype study: compare slopes `∂z/∂x, ∂z/∂y` from f32 vs f64 grids on 5 seeds; record max relative error; choose dtype (default rule in Global Constraints).
- [ ] **Step 4:** Commit `feat(terrain): seeded 3D corridor generator with banks, 2D roughness, shoulder rocks and ruts`.

### Task 3: Render heightfield and builder hookup

**Files:** create `terrain/render.py`; modify `mujoco/terrain.py`, `mujoco/builder.py`; tests `tests/test_terrain3d.py`, `tests/test_builder_spatial.py`

- [ ] **Step 1 (failing tests):** `render_hfield` shape `(ceil(2W/dy_r)+1, ceil(L/dx_r)+1)`; values are area-averages of the fine grid (not point samples); the builder with a `.terrain3d` sets `nrow/ncol/size` accordingly and `radius_y = W`; planar builder output unchanged (sha256 snapshot from P1 Task 4).
- [ ] **Step 2:** Implement. Memory: render data as float32 (MuJoCo `hfield_data` is `float`).
- [ ] **Step 3:** Commit `feat(builder): render heightfield from 3D terrain grids`.

### Task 4: Native `Grid3D` — load and bicubic sample with analytic normals

**Files:** `native/src/terrain/grid3d.{hpp,cpp}`; `native/tests/test_grid3d.cpp`; `tests/reference/test_native_grid3d.py`

**Interfaces:** spec §5.1. Catmull-Rom tension 0.5; `normal = normalize(−∂z/∂x, −∂z/∂y, 1)`; border cells clamp indices; `inside = false` outside `[x0, x0+(nx−1)dx] × [y0, y0+(ny−1)dy]`.

- [ ] **Step 1 (failing tests, C++):** plane `z = a x + b y + c` reproduced exactly at grid nodes and to 1e-12 between nodes with exact normal; quadratic bowl sampled with the expected O(h⁴) error; normals continuous across cell boundaries (jump < 1e-9); `load` rejects bad magic/shape; `sample` is `noexcept` and allocation-free (RTSan build); **mirror rule (spec §10.5):** `sample(x, y)` on a grid mirrored in y equals `sample(x, −y)` **bitwise** — the basis polynomials are evaluated in the symmetric form, never as a one-sided Horner in `t`.
- [ ] **Step 2 (failing tests, Python):** a NumPy reference bicubic (`scipy.interpolate.RectBivariateSpline` is **not** Catmull-Rom — write the 16-coefficient reference in NumPy) agrees with native to 1e-12 on 10 000 random points of an `alpine_rocky_3d` grid.
- [ ] **Step 3:** Implement with precomputed per-cell access (no per-call allocation; 16 node reads + two 4-term Horner evaluations).
- [ ] **Step 4:** Commit `feat(native): Grid3D bicubic sampling with analytic normals`.

### Task 5: Native `Grid3D::ray_cast`

**Files:** as Task 4

**Interfaces:** `ray_cast(origin, dir, t_max)`: if `origin` is already below the surface, returns `hit = true, t = 0` (penetrating start — the tyre code treats this as full deflection); otherwise 2D DDA over cells along the projected ray; per cell, evaluate `f(t) = ray_z(t) − z(x(t), y(t))` at entry/exit; on sign change, 8 bisection steps (fixed), then one Newton correction using the analytic gradient, clamped to the bracket; returns point, normal, surface id. Iteration bound: `cells_crossed ≤ ceil(t_max/dx) + ceil(t_max/dy) + 2`.

- [ ] **Step 1 (failing tests):** vertical rays on the plane hit at the analytic `t`; oblique rays on the bowl agree with a brute-force fine march (1e-6); rays that never cross return `hit = false` after ≤ the bound (count instrumented in the test build); the same ray twice → identical bytes; a ray starting below returns `t = 0`; a mirrored ray on the mirrored grid → mirrored hit **bitwise**.
- [ ] **Step 2:** Implement; add `ray_cast_many(span<origins>, span<dirs>, span<RayHit> out)` as the batch entry the tyre writer and the worker pool will call (pure loop, no threading here).
- [ ] **Step 3:** Micro-benchmark `native_bench_grid3d` (CTest, not gated): ns per ray for 1 000 random rays on `alpine_rocky_3d`; record in the audit (expected O(100 ns)).
- [ ] **Step 4:** Commit `feat(native): deterministic Grid3D ray cast with bounded DDA`.

### Task 6: Extrusion equality and the two 3D reference tracks

**Files:** `terrain/extrude.py`, the genspec TOMLs, `tests/test_terrain3d.py`

- [ ] **Step 1:** Generate `alpine_rocky_3d` (seed 0) and `rough_uphill_savage_3d` (seed 0) into `examples/research/*.terrain3d`? **No** — the files are ≈ 70–145 MB; commit only the genspecs and sha256 of the generated files in `examples/research/terrain3d_manifest.json`; tests regenerate into `tmp_path` (≈ seconds) and compare the hash.
- [ ] **Step 2:** `extrude.py`: for every planar track in `examples/research/*.toml`, the extruded 3-row grid's middle row equals the 1D profile bitwise (test). `terrain/research.py` resolves `*_3d.toml` track files (regenerate from genspec into the cache dir, verify the manifest hash) so P3–P5 can run `--track-file examples/research/alpine_rocky_3d.toml`.
- [ ] **Step 3:** Commit `feat(terrain): 3D reference track specs with hash manifest and planar extrusion equality`.

### Task 7: Phase audit and gate

- [ ] **Step 1:** `bash tools/run_tests.sh native` and `quick` green; planar generator snapshot green; `tools/terrain3d_preview.py` renders both tracks (attach PNGs to the audit).
- [ ] **Step 2:** `docs/superpowers/audits/2026-10-14-spatial-3d-p2.md`: dtype decision, sample/ray-cast accuracy tables, ns/ray, memory sizes.
- [ ] **Step 3:** Merge; tag `spatial-3d-p2`.
