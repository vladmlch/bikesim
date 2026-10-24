# Failure diagnosis — Linux and macOS

## Spatial product — pending V08

The spatial application changes are implemented in source and unexecuted in
W45–W49. Preserve the exact selected binary/configuration/terrain/calibration
identity and the first diagnostic when qualifying them locally.

| Spatial symptom | Meaning and bounded action |
| --- | --- |
| `--terrain3d requires --spatial 3d` | The effective physics mode is planar. Select 3D explicitly or remove the terrain override. |
| Conflicting track/terrain options | Choose the typed terrain request; legacy profile extrusion is never inferred from a filename. |
| `profile_extrusion` capability rejection | C10/V04 must resolve the frozen material-registry limitation. Use a supported authored grid/genspec for the documented scenarios. |
| `recording schema 1 has no spatial channels` | The reader accepted a planar recording. Request its existing fields; no spatial zeros are synthesized. |
| `incompatible-execution` during checked replay | Preserve the accepted recording and use its exact numerical execution identity. Inspection/reporting may remain available; do not relax comparison or rename inspection as verified. |
| `N/A` attitude/contact/IMU | Inspect validity, per-component mask and source/delivery timestamps before interpreting the value. |
| Missing display | Run the explicit GUI gate on a real supported display. Headless application success does not cover it. |
| Missing installed examples/catalog/library | Retain install/package logs and inspect the moved prefix and existing rpaths/load names. Do not patch loader paths to a developer checkout. |

Scenario argv and the outstanding evidence are in [spatial-3d.md](../spatial-3d.md)
and its linked V08 ledger.

Preserve the original error and the selected build's
`native_check_context.json` and `bike_policy_report.json`. Configure owns
native dependency/toolchain/ABI diagnostics; there is no separate Python
doctor in the root workflow. Do not hide a failure with arbitrary library
symlinks, broad loader-path exports, weakened tests, changed physics
tolerances or a modified dependency lock.

| Symptom | Evidence | Bounded remediation |
| --- | --- | --- |
| Apple SDK option on Linux | Configure stderr and the selected `CMakeCache.txt` | Configure a fresh Linux build without `CMAKE_OSX_SYSROOT` or `SDKROOT`. Apple discovery must never run in the Linux lane. |
| Missing MuJoCo private symbol | Configure ABI-probe log naming `_mjPRIVATE_setTlsLogHandler`, `_mjPRIVATE_getGlobalLogHandler` or `mj_makeRawData` | Restore the exact MuJoCo 3.12.0 SDK from the lock's verified archive, then configure again. |
| Archive/member hash mismatch | Configure identifies the dependency and expected/actual SHA-256 | Reacquire the pinned archive explicitly or restore an intact offline package. Do not replace the lock with the bytes on disk. |
| Numerics record or ABI mismatch | `native/numerics.lock.json`, context `numerics`, original configure error | Restore the measured record and selected target's pinned provider. Generated records are updated only by their authorized measurement workflow. |
| Missing native executable | Missing `build/<preset>/bin/bike-*` or stale build cache | Use `cmake --workflow --preset <preset>` in the intended source checkout. Release lives at `build/native-release`. |
| Unresolved DSO | Original loader diagnostic; `readelf -d` on Linux or `otool -L` on Darwin | Rebuild with the selected dependency tree. For installation, rerun `install_smoke`; retain the complete `bin/lib/share` package layout. |
| Missing display | GLFW diagnostic and display/session environment | Use the application's headless mode or a real desktop. Linux Xvfb smoke is separate from GUI acceptance. |
| Unsupported sanitizer | Configure stderr and context `capabilities` | Select a qualified instrumenting compiler in its own build directory. Do not preload an unrelated runtime or run the release binary as a sanitizer control. |
| ASan executable cannot load its runtime | Linked runtime path and the selected compiler's resolved sanitizer image | Reconfigure the instrumented root build; its build rpath must point to the same runtime the compiler qualified. Native executables do not use the Python preload launcher. |
| Coverage tool mismatch | CMake policy probe `coverage_tool_resolution.json` and `coverage-format/` command/output logs | Set `LLVM_PROFDATA`/`LLVM_COV` only to tools compatible with the chosen compiler. The real compile/run/merge/export control must pass. |
| Empty coverage report | `build/coverage/profiles`, `coverage-binaries-<config>.txt`, CTest output | Build target `coverage`. It requires fresh profiles, successful tests and at least one `native/src/` file in the exported LCOV. |
| Strict invalid-controller stop | Run events, summary and first-failure diagnostic | Preserve the model's validity boundary and committed prefix. Do not relax tolerances to lengthen an episode. |
| Checked replay identity mismatch | Error names differing execution/provider/platform/source fields | Replay with the matching environment; use recorded-state inspection only when that is the intended operation. Do not edit recorded identity. |
| Offline dependency missing | `extract_deps.sh` identifies a missing archive or member | Use the source package for the same target platform. Offline extraction has no network fallback. |
| Transition Python SOABI/source-stamp mismatch | Retained binding loader error and binding context | Rebuild the binding graph with its selected CPython 3.13 environment. This does not affect selection of the standalone C++ executable. |

## Escalation evidence

Keep the exact command, source commit, selected preset, complete diagnostic,
exit code, failed test node names, context and per-sweep summaries/logs.
A failed sweep names its CMake target; rerun only that named target while
iterating. The release checklist retains the final broader gate evidence.
The source-only implementation session did not execute any command shown
as a future verification procedure.
