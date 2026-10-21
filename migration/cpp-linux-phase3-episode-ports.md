# Episode regressions — per-function source review

Date: 2026-10-21. Scope: the nine original functions in
`test_golden_episode.py`, `test_episode_compare.py`, and
`test_golden_episode_v2.py`, supporting completion Tasks 3.03 and 3.24b.
All nine now have named source successors in
`native/tests/reference/test_golden_episode_regressions.cpp`. The v1/v2
parametrization makes **10 concrete cases**. None has been compiled or run.
The user prohibited builds, compiler checks, configure, tests, applications,
containers and oracle capture; no successful verification is implied.

The parent integration registered `reference.golden_episode_regressions`
with `ENGINE`, `bike_golden`, and `golden;slow` labels (plus its migration
labels). This file does not change the ledgers or task checkboxes.

## Exact dispositions

| Original function | Source disposition | Preserved assertions |
|---|---|---|
| `test_golden_episode.py::test_capture_is_deterministic` | port / invariant; source implemented | Two fresh 50-step captures; channel schema and row count; exact typed rows and nested serialization rows; exact first failure. |
| `test_golden_episode.py::test_saved_artifact_roundtrips` | port / invariant; source implemented | Actual 50-step save/load; channel names, all flattened row leaves at zero tolerance, first failure and real `model.mjb`. |
| `test_episode_compare.py::test_replay_matches_golden_bitwise` | port / invariant; source implemented | Actual 50-step capture/replay and the original comparator defaults (`rtol=atol=1e-12`); exactly equal first failure. The Python name says bitwise, but its row assertion calls that default comparator. |
| `test_episode_compare.py::test_compare_episode_on_saved_artifact` | port / contract; source implemented | 50-step capture, save, load and complete replay/comparison entry point, including first failure. |
| `test_episode_compare.py::test_compare_reports_first_divergence` | port / contract; source implemented | Original `y=2` versus `2+1e-9` input; nonempty difference identifying step 0 and channel y. |
| `test_golden_episode_v2.py::test_v2_state_and_forces_recorded` | port / invariant; source implemented | Exact 40-step state and force dimensions; 40 tyre snapshots; `fork_spring`, `shock_coil`, `shock_damper`; ordered component fold compared bitwise with the independent live total at every step and DOF. |
| `test_golden_episode_v2.py::test_v2_roundtrip_and_determinism` | port / invariant; source implemented | Two fresh 40-step captures and save/load; all nine original state/force/tyre fields compare dtype, shape, order and bytes; component names keep their order. |
| `test_golden_episode_v2.py::test_exit_total_is_captured_independently_of_components` | port / invariant; source implemented | Zero-step total width; actual `+0.125` live force mutation after `apply_forces`; all three exit observations compared bitwise; last component fold must differ; actual mutated capture roundtrips losslessly. |
| `test_golden_episode_v2.py::test_load_old_artifact_without_exit_total` (`v1`, `v2`) | port / contract; both cases source implemented | The original one-slide real engine fixture; remove the same NPZ fields; missing total stays `[0,nv]`; v2 components stay `[2,1,nv]`, v1 components stay `[0,0,nv]`. |

Totals: **9 source functions implemented, 0 partial functions, 0 missing
functions, 0 drops** within these three files. This does not cover the other
files in Task 3.24b and does not close its gates. The two-platform episode
oracle test remains separate from these deterministic and format properties.

## Live-exit mutation stays inside the capture path

The original Python test monkeypatches `apply_forces` and writes to real
`mjData.qfrc_applied[0]` after accumulator assembly. A synthetic NPZ total
cannot establish that capture reads this live buffer. The native successor
therefore uses the internal `golden/capture_testing.hpp` entry point, not
the codec fixture.

`GoldenForceObservation` has private callback/context fields accessible only
to `golden::RuntimeAccess` and `runtime::PhysicalStep`. The callback executes
inside the existing opt-in observation branch, immediately after the real
`apply_forces` call and before the independent live buffer is copied. It
receives only a temporary bounded `span<double>` for generalized forces;
it cannot obtain model, controller, writer or arbitrary data pointers from
this interface. Normal advancement still passes a null observation, and
ordinary capture/replay leave the callback null.

The test allocates its full `3*nv` observation storage before capture, checks
that the actual model has a force DOF, validates each received extent/count,
and never retains the borrowed live span. The callback is `noexcept`; its
body performs the original one addition and copies into already allocated
storage. The component matrix is left untouched. The source test would
reject an implementation that silently reconstructed totals from that matrix.
It uses actual production advancement and save/load, and creates no oracle
fixture. Whether this regression passes remains unmeasured.

## Acquisition input corrections

`tests/fixtures/golden/episode/pinned_savage_40/freeze.py` identifies the real
Task 3.03, explicitly selects `--backend python --seed 0`, and keeps the
original 40 steps, 35 N m control, diagnostic policy and legacy capture.
`research_cli.parser` already defaults to Python and seed 0; the explicit
arguments preserve those defaults and match native `request.toml`'s
`seed_decimal="0"`. No seed was inferred from a generated trajectory.

The future capture's incidental CLI output now uses its own
`TemporaryDirectory`, replacing the shared `/tmp/g-py-out` path. The existing
`close_environment(env, primary)` lifecycle closes the environment before
temporary cleanup and preserves a primary capture/save exception if cleanup
also fails. The final oracle destination remains exclusive and refuses an
existing path.

## Remaining evidence

Actual Python artifacts for both `darwin-arm64/episode/pinned_savage_40` and
`linux-aarch64/episode/pinned_savage_40` are still missing. The native oracle
test fails explicitly if its platform artifact is absent; no values were
invented or populated from native output. Compilation, all ten new regression
cases, the platform-specific golden comparison, Python cross-reading of
native output, CLI acceptance and relocation checks remain unexecuted.
The infrastructure status and comparison contracts are recorded in
`cpp-linux-phase3-golden-review.md`.
