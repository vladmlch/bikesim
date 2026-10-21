# Native installation and offline source distributions

The root build produces C++ applications for macOS arm64 and Linux aarch64.
Release qualification is recorded in
[the release checklist](releases/python-free-qualification.md). The new
Phase 3 packaging implementation has been inspected as source only; no
package, offline rebuild, compiler or runtime gate was executed in this change.

## Binary package

After a qualified release build, create its relocatable install archive:

```sh
cmake --workflow --preset release
ctest --test-dir build/native-release -R '^install_smoke$' --no-tests=error --output-on-failure
ctest --test-dir build/native-release -R '^package_smoke$' --no-tests=error --output-on-failure
cpack --config build/native-release/CPackConfig.cmake -G TGZ
```

The archive is `build/native-release/packages/bike-sim-0.0.0-<platform>.tar.gz`,
where `<platform>` is `darwin-arm64` or `linux-aarch64`. Version `0.0.0`
is the migration's initial project version; changing it is a release decision.
Extract the archive and run the programs in `bin/`. Keep `bin/`, `lib/`
and `share/` together when moving the directory. The install layer uses
`@loader_path/../lib` on Darwin and `$ORIGIN/../lib` on Linux. The package
contains the selected pinned runtime libraries and example inputs.

`package_smoke` exercises every installed `bike-* --help`, a bounded
headless ride, and the package audit with the checked-in no-Python policy.
Every nonzero ride exit remains a failure until a concrete diagnostic has
been measured and approved. A missing CLI, example, dependency or audit
policy cannot silently pass. `install_smoke` separately inspects the actual
loader paths after moving the install directory.

## Source package with offline dependency archives

```sh
cpack --config build/native-release/CPackSourceConfig.cmake -G TGZ
```

The output is `bike-sim-0.0.0-source.tar.gz` under the same package directory.
It includes the source snapshot and only the current platform's locked
archives under `.deps/_archives/`. The source distribution remains platform
specific even though its filename says `source`; retain it alongside the
corresponding binary package. Native source configuration verifies the
platform selection and rejects archives for a different lane.

CPack stages the source at package time and excludes `.git`, `.venv`,
extracted `.deps` trees, build outputs, `output`, `.worktrees`, `.scratch`,
`.work` and Python bytecode. It adds exactly the selected archive names.
The transition Python sources are retained until the separately gated
Phase 4 removal; their presence does not introduce interpreter execution
into the root configure/build/install path. Archive bytes are checked
against the dependency lock before staging, and package smoke rechecks
their digests after extracting the completed source archive.

### Explicit online acquisition

From a fresh checkout on the target platform, acquire the locked bytes
before configuring. This is the sole network acquisition step:

```sh
# macOS arm64; requires curl, jq, unzip, tar and shasum.
bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps darwin-arm64

# Linux aarch64, inside the qualified container; requires sha256sum.
bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps linux-aarch64
```

The Linux image's acquisition stage calls the same script. Both routes
delegate extraction to `tools/linux/extract_deps.sh`. No wheel is installed
or imported: wheel archives provide selected native headers, shared
libraries and license notices only. The extractor verifies the archive
digest before extraction and verifies every required member digest and
license before publishing the dependency tree. Existing extracted roots
are rejected so that a new acquisition cannot silently overwrite them.

### Offline preparation and rebuild

After extracting the source package, run from its root:

```sh
# On Linux aarch64:
bash tools/linux/extract_deps.sh native/dependencies.lock.json .deps linux-aarch64
cmake --workflow --preset release

# On macOS arm64, use its source package instead:
bash tools/linux/extract_deps.sh native/dependencies.lock.json .deps darwin-arm64
cmake --workflow --preset release
```

The extractor has no download branch. A missing or corrupted archive is an
error. It prepares MuJoCo and DuckDB SDK layouts, the complete GLFW tree,
header dependencies and Linux's pinned NumPy OpenBLAS plus libgfortran,
then writes each `BIKE_VERSION` marker consumed by configure. The numerics
record is frozen measured data; this route never regenerates it.

For the required Linux network-isolated rehearsal, use the explicit CI
lane from an ARM64 Linux host with Docker:

```sh
export BIKE_CI_LANE=offline-rebuild-linux
export BIKE_LINUX_CONTAINER=bike-offline-rehearsal
bash tools/ci_presets.sh release offline
```

It first builds and checks the selected source normally, then starts a
fresh `--network none --platform linux/arm64` container with the completed
package directory mounted read-only. Inside that container it extracts
one source archive, runs the offline extractor and runs the release
workflow. It does not reuse the checkout's extracted dependencies.

## Transition from full/slim Python bundles

The old `packaging/OFFLINE_README_full.md`, `OFFLINE_README_slim.md` and
installer scripts describe the retained Python distribution. They remain
until Phase 4 and are not entry points for these native archives.
Native binary packages carry runtime libraries; native source packages
carry the selected locked archives. There is no full/slim environment
installation or interpreter selection in the new path.
