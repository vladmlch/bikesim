# Native installation modes

The standalone C++ application build starts at the repository root and
uses explicitly acquired native dependencies under `.deps`. Its root
presets never resolve an interpreter. The transition Python package and
binding graph were deleted at T63; no interpreter route remains.

## Choose an installation mode

| Mode | Contents | Build and verification |
| --- | --- | --- |
| Native binary archive | Installed `bin/`, runtime `lib/`, examples and provenance in `share/` | `package_smoke` and `install_smoke` on the target platform, then retain the complete directory when relocating |
| Native source archive | Source plus target-matched dependency archives under `.deps/_archives` | Offline extractor, root release workflow, native checks and an isolated offline rebuild |
| Native source development | Real checkout, extracted `.deps`, root CMake graph | `cmake --workflow --preset release`; other presets have separate build trees |

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

On Linux the documented lane is aarch64 inside the checkout's container.
That lane is not yet qualified on this tree (see the
[release qualification checklist](../releases/python-free-qualification.md));
the recorded container contract is:

~~~sh
bash tools/linux/container.sh sync
bash tools/linux/container.sh exec cmake --workflow --preset release
bash tools/linux/container.sh exec bash tools/run_native_checks.sh build/native-release
~~~

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

## Historical: transition binding/oracle setup (removed at T63)

The retained-Python transition route was deleted at T63 with the runtime,
the frozen environment and the binding surface; the instructions that
followed here are historical only. During the transition a binding build
was configured from `native/` against CPython 3.13, `BIKE_NATIVE_BUILD_PATH`
and `NATIVE_TEST_BUILD_PATH` selected the extension artifact for the
retained Python entry points, and the binding loader checked SOABI, source
stamp and provider identity. The Python package wheel never contained the
`bike_native` extension, and the former full/slim offline Python installers
were transition artifacts — never installers for the native CPack
distributions. None of those mechanisms survive: the produced
`build/<preset>/bin/bike-*` executables are the only runnable surface and
carry their provider identity in the linked build record.

## Verification status

The Phase 3 changes were implemented and inspected as source with
compilation and execution prohibited. Actual preset builds, negative
sanitizer controls, coverage, install/package smokes and offline rebuilds
remain to be recorded in the
[release qualification checklist](../releases/python-free-qualification.md).
