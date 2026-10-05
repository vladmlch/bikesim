# Task 5 — Tire writer port (stateful) — cpp-port P2

## Status

**Complete.** The native compliant-tire writer is bitwise-equal to the Python
`TireForceApplier` on every stored golden step — the full dual-run oracle:

```
set_state(state_*, time[k]) → forward → set_tire_state(names, tire_state[k])
→ tire_qfrc(dt)  ==  forces[k]['tires']  (bitwise, all 40 steps)
                ==  live TireForceApplier.compute_qfrc on restored _BrushState
```

and the committed brush states, contact snapshots, diagnostics and downstream
`resistance_components` match bitwise as well. **Zero ULP deviations.**

## Artifact timing resolution (the ruling's key concern)

`forces[k]['tires']` is produced inside `apply_forces` at step `k` **before**
`mj_step`, and `compute_qfrc` consumes the brush state committed by the
*previous* call. The artifact previously recorded `tire_state[k]` **after**
`runtime.step`, so row `k` held the state step `k+1` consumed — one row late.

`tools/golden_episode.py` now captures `_tire_state_row(runtime.tire)` **at
`spied_apply` entry** (before `orig_apply` runs), so `tire_state[k]` is exactly
the `_BrushState` pair `forces[k]` consumed — the same convention as
`state_qpos[k]`. With that encoding the ruling's per-stored-state oracle
holds literally, including row 0 (initial defaults: `xi=0`, all `None` → NaN).

## Files

New:
- `native/src/tyre/profile.hpp` / `profile.cpp` — `ProfileQuery`,
  `ProfileContact`, `compiled_profile_vertices` (heightfield raster → X-Z
  profile), plus the scalar primitives the contract needs:
  - `np_interp` — numpy's scalar `np.interp` path, including the **explicit
    `std::fma`** interpolation term (probed: mul+add lands ~1e3/24k rows 1 ULP
    off; fma matches exactly). `-ffp-contract=off` stays global.
  - `py_hypot` — CPython `math.hypot` (`vector_norm` scaled compensated
    squaring + differential correction), not libm hypot.
  - Same-libcall lowering: `cblas_ddot` for 1-D `@`/`np.dot`,
    `cblas_dgemv(RowMajor,NoTrans)` for `(k,2)@(2,)`,
    `cblas_dgemm(RowMajor,NoTrans,Trans)` for `local @ R.T`,
    `sqrt(ddot(v,v))` for `np.linalg.norm(v)`, seeded `0.0+p0+p1` accumulation
    for `einsum('ij,ij->i')`, elementwise `sqrt(x²+y²)` for `axis=1` norms.
- `native/src/writers/tire.hpp` / `tire.cpp` — `TireWriter`: the per-wheel
  `_BrushState` (`xi`, `tangent`, `point`, `segment`, `center`), the
  tangent-transport/branch-release logic, `effective_friction` (configured or
  `SurfaceMap.at(p.x)` capped by tire `mu`), `_brush_step`/`_normal_contact`,
  `ContactPatch`/`WheelContactSnapshot` validation, diagnostics, commit only
  after both wheels evaluate, once-per-timestamp clock.
- `tests/reference/test_native_tire.py` — 6 tests (2 slow dual-oracle, 4
  non-slow: config gates, key naming, set_state validation, happy-path clock).

Modified:
- `tools/golden_episode.py` — `tire_state` captured at `apply_forces` entry.
- `tools/native_config.py` — emits the `tire` section for `compliant_2d`
  (`TireSpec` only; `TabulatedTireSpec`/other backends rejected).
- `native/src/config.hpp` — `TireMaterial/TireParams/SurfaceSpec/
  SurfaceSection/SurfaceMap/TireConfig` + `tire_from_dict` (backend must be
  `compliant_2d`; surface_mode `configured`/`track`; full key-path errors).
- `native/src/stepper.{hpp,cpp}` — `tire_qfrc`, `set_tire_state`,
  `tire_state`, `tire_state_names`, `tire()` accessor; writer built when the
  `tire` section is present.
- `native/src/binding.cpp` — `set_tire_state`, `tire_state`,
  `tire_state_names` (prop), `tire_qfrc`, `tire_snapshots`,
  `tire_diagnostics`. `tire_snapshots()` dicts carry both schemas: the
  manifest digest keys (`time_s`, `interval_id`, `backend`,
  `geometric_contact`, `effective_radius_m`, `patches`) **and** the flat
  `side_input` keys (`patch_loads`, `patch_working`, `eff_radius`) so they
  feed `resistance_components` end-to-end.
- `native/CMakeLists.txt` — the two new TUs.

## Encoding contract

Canonical schema is 22 columns per `tire_state_names` (`{side}.{field}` for
`xi`, `tangent.0-2`, `point.0-2`, `segment`, `center.0-2`). All-NaN vector
triples decode to `None`; NaN `segment` decodes to `None`; NaN `xi` is a real
value (it reaches `_finite_result` exactly like Python). Artifact union rows
also carry leaf `*.tangent` columns (always `None`→NaN); they decode and are
ignored. Missing canonical columns are named in error messages before any
unexpected-column complaint.

## Verification

- `cd native && uv run cmake --build build` — clean, `-Werror` strict flags.
- `tools/check_native_frontends.sh` — GCC syntax sweep clean (ABI notes only).
- `pytest tests/reference/test_native_tire.py -m 'not slow'` — 4/4.
- `pytest tests/reference/test_native_tire.py -m slow` — 2/2 (40 steps each:
  golden forces + live oracle + evolved states + snapshots + resistance).
- `pytest tests -m 'not slow'` — **408 passed**.
- `pytest tests -m slow` — 41 passed / 24 failed; every `test_native_*` and
  `test_golden_episode*` test passed. The 24 failures are pre-existing
  environment-dependent physics-integration tests (equilibrium residual,
  pedelec acceptance, rider welds, realtime gates) — verified identical on the
  clean base `93e360f` with this work stashed (e.g.
  `test_flat_launch_delivers_crank_torque`: same 0.5875 ≥ 0.8 failure).

## Deliberate deviations / notes

- Python `ArithmeticError`/`TypeError` map to `std::overflow_error`/
  `std::invalid_argument` (nanobind → `OverflowError` ⊂ ArithmeticError /
  `ValueError`) — same exception families, not identical classes.
- A restored state with `tangent` but no `segment`/`point` is rejected with a
  named `invalid_argument` instead of Python's incidental `TypeError` on
  `int(None)` — malformed either way, explicit here.
- `set_tire_state` requires the canonical 22-column schema (artifact rows
  always carry it) and clears the timestamp clock — mirrors the test oracle's
  `last_time_s = None` after a restore.
- `effective_radius_m`/`sum(weights)` replicate CPython's compensated
  `sum()`; `round(time/dt)` is `nearbyint` under the default to-nearest-even
  mode; `min`/`max` are `std::min`/`std::max` (identical NaN/tie semantics).
- The ctor replays `TireSpec`/`TireParameters`/`TireBackendConfig`/
  `SurfaceSpec`/`SurfaceMap` `__post_init__` validation (including the
  `provenance.strip()` nonemptiness check) before `__init__`-body work, like
  Python's dataclass flow.
- `multi_support` is diagnostics-only; the flag path is exercised whenever the
  episode produces significant secondary supports, and every diagnostics field
  is bitwise-verified in the slow test.
