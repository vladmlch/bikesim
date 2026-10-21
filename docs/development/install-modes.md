# Native and transition installation modes

The standalone C++ application build starts at the repository root and
uses explicitly acquired native dependencies under `.deps`. Its root
presets never resolve an interpreter. The transition Python package and
binding graph remain in the tree until Phase 4's qualified deletion; they
are separate build routes.

## Choose an installation mode

| Mode | Contents | Build and verification |
| --- | --- | --- |
| Native binary archive | Installed `bin/`, runtime `lib/`, examples and provenance in `share/` | `package_smoke` and `install_smoke` on the target platform, then retain the complete directory when relocating |
| Native source archive | Source plus target-matched dependency archives under `.deps/_archives` | Offline extractor, root release workflow, native checks and an isolated offline rebuild |
| Native source development | Real checkout, extracted `.deps`, root CMake graph | `cmake --workflow --preset release`; other presets have separate build trees |
| Transition Python/oracle | Existing Python source, locked CPython 3.13 environment and optional binding graph | Retained until its two-platform captures and C++ successors have qualified; it is not the native distribution |

See [installation](../installation.md) for the exact acquisition,
extraction, CPack and network-isolated rehearsal commands. Version `0.0.0`
is the initial native migration version; no archive from this source-only
change is represented as an already qualified release.

## Native source setup

From a real source checkout, acquire or offline-extract the dependency
archives, then run:

```sh
cmake --workflow --preset release
bash tools/run_native_checks.sh build/native-release
```

On Linux the required lane is aarch64 inside the checkout's container:

```sh
bash tools/linux/container.sh sync
bash tools/linux/container.sh exec cmake --workflow --preset release
bash tools/linux/container.sh exec bash tools/run_native_checks.sh build/native-release
```

The release directory stays `build/native-release`; `debug`, `gui`,
`asan`, `rtsan` and `coverage` use `build/<preset>`. Do not copy a CMake
cache between machines, compilers or source paths. An unsupported
compiler, missing private MuJoCo ABI, wrong provider width or archive hash
fails configuration. There is no silent fallback to a system OpenBLAS,
a Python backend or an uninstrumented build.

## Configure diagnostics and artifact selection

Configure-time probes replace the native part of the former doctor
command. Inspect `native_check_context.json`, `bike_policy_report.json`
and the retained configure diagnostics in the selected root build tree.
The context records the actual compiler, SDK/platform and numerics
provider. A successful configure does not close CTest, performance,
package, offline or GUI gates.

Native applications run directly from that build's `bin/` directory:

```sh
build/native-release/bin/bike-ride --help
build/native-release/bin/bike-recording --help
```

Their identity comes from the linked native build and configured provider
record. Checked replay still requires the recording's execution identity
to match; recorded-state inspection does not prove physical replay.

## Transition binding/oracle setup

The Python files are retained solely until the staged migration has
captured and qualified their C++ successors. A text-only Repomix snapshot
cannot supply omitted binary assets, dependencies, executable metadata or
Git ancestry. Use the full source checkout for this transition route.

A binding build is explicitly configured from `native/`, with CPython 3.13
from the selected frozen environment. Select the interpreter and build
path consistently. `BIKE_NATIVE_BUILD_PATH` and `NATIVE_TEST_BUILD_PATH`
apply to those retained Python entry points; they do not choose which root
C++ executable a shell command invokes. The binding loader still checks
SOABI, selected artifact, source stamp and provider identity. Renaming or
copying an extension between operating systems does not satisfy those
checks.

The Python package wheel does not contain the `bike_native` extension.
The former full/slim offline Python installers remain available as
transition artifacts and are deleted only at the Phase 4 gate. They are
not installers for the native CPack distributions.

## Verification status

The Phase 3 changes were implemented and inspected as source with
compilation and execution prohibited. Actual preset builds, negative
sanitizer controls, coverage, install/package smokes and offline rebuilds
remain to be recorded in the
[release qualification checklist](../releases/python-free-qualification.md).
