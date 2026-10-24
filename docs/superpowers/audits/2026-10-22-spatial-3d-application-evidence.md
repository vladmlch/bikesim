# W45–W49 application evidence handoff

Status: **implemented in source; verification pending V08**. No configure,
build, compilation, tests, application, installation or GUI was executed in
this write phase. This is an acquisition ledger, not a test report. `null`
hashes are deliberately unmeasured. Existing historical planar evidence is
not reused as evidence for the changed spatial product.

Commands below reference exact argv entries in
[`examples/spatial/examples_manifest.json`](../../../examples/spatial/examples_manifest.json).
`EXAMPLES` means the installed spatial example directory, `OUT` a fresh
destination and `RUN` the printed sealed noise2 research directory.

| Scenario | Command | Exit | Artifact hash | Platform |
| --- | --- | --- | --- | --- |
| Planar compatibility | manifest `planar`: `bike-ride --physics physical --duration 1 --headless --no-plots` | pending V08 | null — pending V08 | pending V08 |
| Constrained root | manifest `constrained` | pending V08 | null — pending V08 | pending V08 |
| Free-root straight | manifest `free-straight` | pending V08 | null — pending V08 | pending V08 |
| Cross-slope GUI | manifest `cross-slope` + C/1/2/R interaction | pending V08 | null — pending V08 | pending V08, real display |
| Recorded physical ride | manifest `record` | pending V08 | null — pending V08 | pending V08 |
| Noise2 research | manifest `noise2` | pending V08 | null — pending V08 | pending V08 |
| Inspect accepted container | manifest `inspect` | pending V08 | null — pending V08 | pending V08 |
| Checked recipe4 replay | manifest `checked` | pending V08 | null — pending V08 | pending V08 |
| Native report | manifest `report` | pending V08 | null — pending V08 | pending V08 |
| Process sweep | manifest `sweep`; workers 1/2/4 | pending V08 | null — pending V08 | pending V08 |
| Relocated installed product | `spatial.installed_product` / native install smoke | pending V08 | null — pending V08 | Linux/macOS pending V08 |
| Extracted binary package | package smoke includes installed-product script | pending V08 | null — pending V08 | Linux/macOS pending V08 |
| Incompatible identity | native helper prepares valid changed-identity container; inspect succeeds; checked reports incompatible-execution | pending V08 | null — pending V08 | pending V08 |
| CLI/parser/report structures | `spatial.cli`, `spatial.documented_examples`, `spatial.reports_batch` | pending V08 | null — pending V08 | pending V08 |
| Pose/render isolation | `spatial.viewer_pose` and display smoke | pending V08 | null — pending V08 | pending V08 |

## Required retained evidence

For each local lane record the exact argv and working directory, stdout/stderr,
exit, selected binary SHA-256, source/dependency/compiler/provider identity,
platform, and hashes of the sealed recording/report. Report expected failures
separately from successful application commands. Keep the terminal reason,
model status, hold release and actual recording extents. A real display is
mandatory for GUI evidence. The process regression compares keys and canonical
outcomes, not elapsed wall times; crash exit classification must remain visible.

The source-authored grid manifests have no binary hash until V08 generates and
reviews the asset. V07 still owns physical/calibration/causal acceptance and
V09 owns performance. No green results, fabricated platform identity or
cross-platform equivalence are asserted here.
