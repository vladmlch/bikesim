# C++ Linux completion — Phase 2 source review

Date: 2026-10-21.

Scope: Phase 2 of
[`2026-10-20-cpp-linux-completion.md`](../docs/superpowers/plans/2026-10-20-cpp-linux-completion.md),
applied to the supplied Repomix snapshot. The user explicitly prohibited
compilation and execution. No CMake configure, build, CTest, pytest, native
checker, ShellCheck, container, application, GUI or numerical probe was run.
The changes and regression definitions below are source implementations;
they are not passing test results or platform qualification.

## Task coverage

| Task | Source result | Verification still required |
|---|---|---|
| 2.01 | The root graph uses `BikeDependencyLock.cmake` and resolved MuJoCo/DuckDB library paths. The root, startup, recording and migration target edges use the shared interfaces. | Darwin provenance and load-name parity; T03 and root CTest. |
| 2.02 | The root includes `BikePlatform` before `project()` and `BikeCompilerPolicy` afterwards, rejects cross-compilation, and applies the shared policy directly. Duplicate dispatch, policy options and hardening setup are removed. The root minimum CMake version is 3.27, matching the shared layer. | Effective compiler-command comparison and both-platform policy gates. |
| 2.03a | Both graphs use the existing schema-2 check-context/source-manifest emitter. Its `NATIVE_DIR` argument supports root paths; the root records checked dependencies and committed numerics with resolved provider paths. Root-only path metadata is validated before accepting explicit null Python fields. | Root T56/checker sweeps and binding-context parity. |
| 2.03b | Removed `native/cmake/CompilerPolicy.cmake` and `native/cmake/Dependencies.cmake`; active references use their shared successors. | Rebuild and inspect the newly generated source/build identities. |
| 2.04 | Dependency schema 2 selects platform objects, skips explicit null entries and rejects missing platform entries. The root and binding SDK consumers share verification. T03 scratch locks and negative cases use the new schema. | Platform selection and T03 on real dependency trees. |
| 2.05 | Added verified Linux aarch64 MuJoCo, DuckDB and NumPy provider pins. The explicit acquisition helper and Docker acquire stage prepare `.deps`; the container initializes its own tree without copying host dependencies or overwriting an existing tree. | Execute acquisition, build/provision the image and configure the real Linux root lane. |
| 2.06 | Linux `RECORD_ONLY` validates the measured glibc identity and maps recorded NumPy images to exact hash-checked files under the locked dependency root. File identities use null Apple build IDs; Darwin retains its existing identity probe. | Regenerate the real numerics records, then verify the no-Python Linux configure and provider ABI. |
| 2.07 | Prepared the root Linux graph, portable T03 loader audits and schema-2 checker integration. Removed a remaining root migration `-framework Accelerate` edge in favour of `Bike::Numerics`; added required dynamic-loader linkage. | The first actual Linux root configure/build/CTest and all resulting diagnostics. No unobserved build success is claimed. |
| 2.08 | All 11 installed CLIs receive relative install rpaths. Darwin rewrites only installed MuJoCo names and verifies/repairs installed signatures. Linux installs OpenBLAS and its bundled libgfortran. Added a moved-prefix install smoke and strict installed-provider identity support. | `install_smoke`, T03 focused-install compatibility, loader inspection and native runtime checks on both platforms. |
| 2.09 | GLFW extraction keeps the complete source tree; its two CMake entry files are pinned. Linux GUI configuration selects X11 and disables Wayland. | Actual GUI builds on macOS and Linux; host GLFW re-extraction when needed. |
| 2.10 | Added native-only `bike-ride --close-on-finish`, validation and shared physical/research viewer-option projection. Added T47 regressions and a bounded Xvfb smoke script. | T47 and the real GUI smoke. Xvfb never closes a desktop acceptance row. |
| 2.11 | Added `container.sh gate quick\|native\|full`, explicit Clang selection and existing-cache compiler checks. The command uses the existing common profile runner without adding a Linux skip list. | L3 Linux, measured counts/failing node IDs, qualification and the real repository commit/push workflow. |

The original task and checkpoint checkboxes remain unchanged. Task 2.07's
build/debug work, Checkpoint 2-A and the closing L3 run require execution
and cannot be completed under this session's restriction.

## Shared context and compiler policy

The root previously wrote a separate schema-1 check context. It now calls
`bike_write_check_context(NATIVE_DIR ...)`, which also owns the existing
source manifest. The root's first-party roots retain both `native/` and
`tools/proto_native_bench`; Python, nanobind and robin-map headers are not
required by that graph.

The actual C++ and Python context validators require a numerics object
with a nonempty backend. They do not require the additional machine
numerics fields anticipated by the plan's older emitter comment. No
synthetic probe transcript, build ID, evidence count or relaxed numerics
validation was introduced. The root projects the committed record and
substitutes the provider paths already verified by `BikeNumerics.cmake`.

The root-only `dependency_root`, `dependency_lock`,
`configure_source_dir` and `cmake` values must be valid existing paths;
`cmake` must be executable. Only a complete root context may carry
`python: null` and `python_executable: null`. Partial or null root path
metadata is rejected. Binding contexts retain their interpreter checks
and executable symlink spelling.

The policy comparison below comes from source, not generated compiler
commands:

| Policy surface | Source change and reason |
|---|---|
| Warning flags | Shared Clang candidates are support-probed; unsupported candidates are reported as omissions. `-Werror` is required separately. |
| Branch protection | The shared layer requires an AArch64 target and a successful compile/link capability probe before adding the flag. |
| Standard-library hardening | Existing libc++ hardening remains; Linux libstdc++ receives `_GLIBCXX_ASSERTIONS`. |
| glibc fortification | `_FORTIFY_SOURCE=3` applies to optimized configurations. |
| Darwin link header space | Header padding is supplied by the shared link policy. |
| Numerical and other hardening flags | Release `-O2`, `-ffp-contract=off`, stack protection and zero initialization remain requested in the source policy. |

Deleting or editing the CMake modules changes the inputs to
`native_source_sha256`. That future generated identity change is expected;
no generated identity was measured or patched by hand.

## Dependency evidence and acquisition

All eight Linux-selected dependency archives were downloaded and
inspected as inert data. Their archive SHA-256 values and all 17 selected
required-file SHA-256 values were compared with the lock. The exact URLs,
archive identities, required-file identities and acquisition instructions
are in [`native/dependencies.lock.json`](../native/dependencies.lock.json)
and [`native/dependencies.md`](../native/dependencies.md).

| Linux binary artifact | Version | Archive SHA-256 |
|---|---|---|
| MuJoCo manylinux aarch64 wheel | 3.12.0 | `267c3fc36b5a0536073557760e36c464da2ad9a744b047893db73fe273931ad1` |
| DuckDB `libduckdb-linux-arm64.zip` | 1.5.6 | `b72ed9f05003f5e9d2015f7ceada6416b377d9dd33169cdde3c9e33897856eee` |
| NumPy manylinux aarch64 wheel | 2.5.2 | `0aadf13b60048d501e05fa699efaf7734e2494f3498a4c2a5521d822640324f3` |

The remaining selected archives are toml++, RapidJSON, GLFW, Cairo and
Eigen. Original Darwin acquisition facts are preserved. Existing
platform-neutral pins remain, with only the two GLFW CMake-file identities
added to its consumed-file list.

Static ELF inspection of the NumPy wheel shows that
`libscipy_openblas64_-bb25d554.so` requires the renamed bundled
`libgfortran-e1b7dfc8-d8198c01.so.5.0.0` and carries `$ORIGIN` rpath.
Both files are acquired, verified and installed beside each other. This
inspection did not load or execute either library.

Archive inspection also established two extraction details: RapidJSON's
upstream `license.txt` is copied to the existing locked `LICENSE` path,
and DuckDB's complete initial MIT license block is extracted from its
pinned amalgamated header. The helper validates the resulting files.

The Docker build context contains exactly `Dockerfile`, `acquire_deps.sh`
and `dependencies.lock.json`. The acquire stage prepares `/opt/bike-deps`;
the checkout-owned container stages that into `/src/.deps` only when the
latter is absent. A pre-existing tree is preserved for lock verification.
The configure path does not download dependencies. The lane explicitly
installs both Make and Ninja because existing CMake regression fixtures
can select the default Unix Makefiles generator.

## Relocatable installation

Linux executables receive `$ORIGIN/../lib`; Darwin executables receive
`@loader_path/../lib`. Existing `OPTIONAL` application installs are
preserved, and the root's two focused test installs are also optional so
T03 can install a partial build. The installed MuJoCo/DuckDB files come
from the resolved lock entries.

Darwin installation changes MuJoCo load names and the copied library ID
to the appropriate `@rpath` name, leaving build-tree binaries and `.deps`
untouched. The install script verifies the resulting signature and uses
ad-hoc signing only when verification requires it. This is a source-level
repair path; no exit-137 launch failure or signing requirement was
observed in this session. Complete `@rpath/...` values are substituted as
one CMake template field to avoid adjacent `@...@` placeholder parsing.

The new smoke builds the complete graph, installs it, moves the prefix,
inspects installed runtime images and invokes each of the 11 CLI help
entry points with loader overrides unset. It rejects dependency-tree,
virtual-environment and original-prefix load paths. The test is registered
`RUN_SERIAL` because it builds its own tree. It has not been run.

The Linux root's provider identity API also supports relocation. It first
tries its original exact loaded-image identity. On a specific image
mismatch, it may use only the exact adjacent `bin/../lib/<recorded name>`
files with the compiled ILP64 policy, complete image list and matching
SHA-256 values. It rejects missing, modified, nonregular or symlinked
provider files and verifies that the validated files are actually loaded.
Binding and configure-probe code receive no such fallback. This addresses
an identity API path issue; it is not evidence that the existing CLI
simulation paths previously failed.

## Viewer and lane behaviour

`--close-on-finish` requires a positive finite `--duration` and an
interactive physical or research run. It is rejected for headless,
preview, settling and legacy use. Both actual viewer launch paths use the
same option projection. Research retains its runtime-owned duration and
pending control delivery. With the flag absent, the existing terminal
window hold remains.

`tools/linux/gui_smoke.sh` selects software GL, uses the existing example
files, requests one simulated second and places a 120-second outer limit
with a further 10-second forced-cleanup bound. Nonzero diagnostic and
timeout exits remain failures. No unmeasured physics diagnostic is
whitelisted. The captured Python CLI contract is unchanged.

The Linux gate exports `/usr/bin/clang++-18`,
`NATIVE_TEST_BUILD_PATH=/build/native-clang` and
`UV_PROJECT_ENVIRONMENT=/opt/bike-venv`. For native/full profiles it rejects
an existing root or binding CMake cache using a different compiler
spelling and preserves that cache for review. The existing common profile
runner supplies the actual gate and exit status.

## Regression source added or updated

- T03: platform-selected schema-2 fixtures, wrong-platform refusal,
  Linux shared-library stub and `ldd`/`readelf` audits; Darwin controls stay.
- T56: real root schema 2, complete root metadata, explicit null Python
  fields, invalid/partial path cases and binding-context strictness.
- T47: close-on-finish parsing, diagnostics, physical/research projection,
  default terminal hold and native help visibility.
- `native_installed_provider_tests`: moved prefix, absent original tree,
  exact policy/list/hash validation and missing/nonregular/symlink cases.
- Numerics portability: Linux record-only success without Python and
  reason-specific dependency, libc, hash and containment failures;
  existing Darwin controls remain.
- Root configure portability: host guards now admit the documented
  Darwin arm64 and Linux aarch64 root graphs. Darwin-only live/root
  provider-identity equality remains Darwin-only because Linux's locked
  dependency and virtual-environment paths intentionally differ.
- `install_smoke` and the bounded Xvfb script provide the planned future
  installation and GUI checks.

These are unexecuted regression definitions, not a new pass count.

## Remaining prerequisites and acceptance

The inherited [`native/numerics.lock.json`](../native/numerics.lock.json)
contains only the old four-operation Darwin entry. Phase 1's current
source requires eight operations and a real Linux entry. The generated
file remains byte-identical to the supplied input: inventing provider
symbols or measurements would invalidate the evidence model. The real
Darwin and Linux probes must regenerate it before qualified configuration.
The existing Phase-1 section of the
[evidence log](cpp-linux-completion-evidence.md) records that procedure.

The supplied archive also lacks the real branch objects/payload needed by
Phase 1 Task 1.03. No merge, commit, push or branch qualification is
claimed. Historical baseline and acceptance records remain unchanged.

Once execution is authorized in the actual repository, resolve those
prerequisites and follow the existing plan's focused L1 commands,
Checkpoint 2-A and L3 Linux. Record real source/image identities, commands,
exit codes and failing test IDs. Desktop viewer acceptance remains a
separate requirement. Until then, Phase 2 has a reviewable source delivery
and open execution gates.
