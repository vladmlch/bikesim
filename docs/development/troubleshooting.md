# Failure diagnosis — Linux and macOS

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
| Numerics record or BLAS/LAPACK provider-ABI mismatch | `native/numerics.lock.json`, context `numerics`, original configure or ABI-probe error naming the rejected provider, integer width or symbol spelling | Restore the measured record and the selected target's pinned provider — the NumPy wheel's OpenBLAS on Linux, system Accelerate on macOS ([numerical-profiles.md](../agents/native-numerical-profiles.md). A system-BLAS substitute or a different integer width is not a fix. Generated records are updated only by their authorized measurement workflow. |
| Missing native executable | Missing `build/<preset>/bin/bike-*` or stale build cache | Use `cmake --workflow --preset <preset>` in the intended source checkout. Release lives at `build/native-release`. |
| Missing extension artifact (transition era, removed at T63) | Binding-loader diagnostic naming the `bike_native` extension and the selected `NATIVE_TEST_BUILD_PATH` build directory | Historical diagnostic retained for reading pre-T63 logs: the retained binding build required `NATIVE_TEST_BUILD_PATH`/`BIKE_NATIVE_BUILD_PATH` to name a directory holding exactly one current-stamp artifact. The binding surface no longer exists; the equivalent post-T63 check is the produced executable in `build/<preset>/bin/`. |
| Unresolved DSO | Original loader diagnostic; `readelf -d` on Linux or `otool -L` on Darwin | Rebuild with the selected dependency tree. For installation, rerun `install_smoke`; retain the complete `bin/lib/share` package layout. |
| Missing display | GLFW diagnostic and display/session environment | Use the application's headless mode or a real desktop. Linux Xvfb smoke is separate from GUI acceptance. |
| Unsupported sanitizer | Configure stderr and context `capabilities` | Select a qualified instrumenting compiler in its own build directory. Do not preload an unrelated runtime or run the release binary as a sanitizer control. |
| ASan executable cannot load its runtime | Linked runtime path and the selected compiler's resolved sanitizer image | Reconfigure the instrumented root build; its build rpath must point to the same runtime the compiler qualified. Native executables do not use the Python preload launcher. |
| Coverage tool mismatch | CMake policy probe `coverage_tool_resolution.json` and `coverage-format/` command/output logs | Set `LLVM_PROFDATA`/`LLVM_COV` only to tools compatible with the chosen compiler. The real compile/run/merge/export control must pass. |
| Empty coverage report | `build/coverage/profiles`, `coverage-binaries-<config>.txt`, CTest output | Build target `coverage`. It requires fresh profiles, successful tests and at least one `native/src/` file in the exported LCOV. |
| Strict invalid-controller stop | Run events, summary and first-failure diagnostic (`invalid_controller` raw flag surfacing as `rider_controller.infeasible`) | Preserve the model's validity boundary and committed prefix. Do not relax tolerances to lengthen an episode. |
| Checked replay identity mismatch | Error names differing execution/provider/platform/source fields | Replay with the matching environment; use recorded-state inspection only when that is the intended operation. Do not edit recorded identity. |
| Offline dependency missing | `extract_deps.sh` identifies a missing archive or member | Use the source package for the same target platform. Offline extraction has no network fallback. |
| Slim external unresolved at destination | Offline bundle/install diagnostic naming the `slim_external` provider | Install the exact declared provider or rebuild a `full` bundle — a slim archive never vendors its externals, and a provider-version mismatch is not repaired by rewriting the destination's scientific stack. |
| Transition Python SOABI/source-stamp mismatch (transition era, removed at T63) | Retained binding loader error and binding context | Historical diagnostic retained for reading pre-T63 logs: the binding graph had to be rebuilt with its selected CPython 3.13 environment. The binding surface no longer exists; a post-T63 provider-identity mismatch surfaces at configure or checked-replay instead. |

## Escalation evidence

Keep the exact command, source commit, selected preset, complete diagnostic,
exit code, failed test node names, context and per-sweep summaries/logs.
A failed sweep names its CMake target; rerun only that named target while
iterating. The release checklist retains the final broader gate evidence.
The source-only implementation session did not execute any command shown
as a future verification procedure.
