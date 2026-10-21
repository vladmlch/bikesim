# Native dependency acquisition (`.deps/`)

The standalone Python-free build (`cmake -S . -B build/native-release`)
consumes dependency SDKs from a repository-root `.deps/` tree that is
populated **explicitly before configure**. Configure, build, and install make
no network calls and run no dependency code-generation Python;
`native/cmake/BikeDependencyLock.cmake` only verifies what is already on disk and
fails configure closed when anything is missing, wrong-versioned, or
hash-mismatched.

`.deps/` is gitignored on purpose: it holds multi-hundred-megabyte binary
SDKs. `native/dependencies.lock.json` (committed) is the integrity contract —
every entry records the acquisition URL, the retained archive's real SHA-256,
the license, and SHA-256 of every file the build consumes.

## Lock schema 2

`native/dependencies.lock.json` uses `schema_version: 2`. Version, license,
license paths, purpose and `extracted_root` are common metadata. A platform
neutral dependency retains its top-level `acquisition` and `required_files`.
A binary dependency stores those two objects below `platforms`:

```json
{
  "mujoco": {
    "version": "3.12.0",
    "extracted_root": "mujoco-3.12.0",
    "platforms": {
      "darwin-arm64": { "acquisition": {}, "required_files": {} },
      "linux-aarch64": { "acquisition": {}, "required_files": {} }
    }
  }
}
```

This excerpt illustrates placement only; the committed lock contains the
complete URLs, license metadata and real SHA-256 values. The platform key is
`lower(BIKE_TARGET_SYSTEM)-CMAKE_HOST_SYSTEM_PROCESSOR`, matching the numerics
record key. Selection has three distinct outcomes:

- An object selects and verifies only that platform's acquisition and files.
- JSON `null` means the dependency is not required on that platform: no tree,
  archive, variables or imported target are required. `numpy_openblas` uses
  this on Darwin, where Accelerate remains the provider.
- An absent key is a configure error. There is no fallback to another
  platform and no implicit skip.

Both graphs use `bike_dependency_lock_select` for this decision. The root
resolver and the binding graph's DuckDB/GLFW setup share the same archive,
consumed-file, version-marker and license-presence checks. Schema 1 is not
accepted. The original Darwin archive/file hashes are preserved; only the
lock's own identity changes with the schema and the added platform entries.

## Layout convention

```
.deps/
  _archives/                    # the downloaded archives, retained verbatim
    <archive-file>
  <extracted_root>/             # named in the lock, e.g. mujoco-3.12.0
    BIKE_VERSION                # one line: the lock's "version" string
    <license files>             # named in the lock's license_files[]
    <extracted payload>         # include/, lib/, etc.
```

- `BIKE_VERSION` (not `VERSION`): on case-insensitive filesystems a `VERSION`
  file at an include-rooted tree (Eigen's root is on the include path as
  `Eigen/…`) would shadow libc++'s `<version>` header. Always use
  `BIKE_VERSION`.
- `_archives/` retains the downloaded artifact so the recorded
  `archive_sha256` is checkable forever, and so extraction is reproducible
  without another fetch.
- `license_files` paths are relative to `.deps/`; `required_files` paths are
  relative to the entry's `extracted_root`.

## Per-dependency recipe

For each entry in `dependencies.lock.json` the recipe is identical:

1. Select the platform entry (or the platform-neutral entry), then download `acquisition.url` into `_archives/` under exactly the recorded
   `acquisition.archive` name.
2. Verify `shasum -a 256` of the archive equals `archive_sha256`.
3. Extract into `extracted_root` so the recorded `required_files` exist at
   their recorded relative paths.
4. Write `extracted_root/BIKE_VERSION` containing exactly `version`
   (no leading `v` unless the lock says so).
5. Place the license file(s) at the recorded `license_files` paths.
6. Re-verify each `required_files` SHA-256.

Concrete artifact notes:

| dep | version | source | layout after extraction |
|---|---|---|---|
| mujoco (Darwin) | 3.12.0 | `files.pythonhosted.org/…/mujoco-3.12.0-cp313-cp313-macosx_11_0_arm64.whl` | unzip the wheel; copy `mujoco/include/` and `mujoco/libmujoco.3.12.0.dylib` → `lib/`; `LICENSE`, `LICENSES_THIRD_PARTY.md` |
| duckdb (Darwin) | 1.5.6 | `github.com/duckdb/duckdb/releases/download/v1.5.6/libduckdb-osx-universal.zip` | `libduckdb.dylib` → `lib/`; `duckdb.h`, `duckdb.hpp` → `include/`; `LICENSE` |
| tomlplusplus | 3.4.0 | `github.com/marzer/tomlplusplus/archive/refs/tags/v3.4.0.tar.gz` | keep `include/` + `LICENSE` |
| rapidjson | 24b5e7a8…7102f | `github.com/Tencent/rapidjson/archive/24b5e7a8b27f42fa16b96fc70aade9106cf7102f.tar.gz` | keep `include/`; copy the upstream `license.txt` to the lock's `LICENSE` path |
| glfw | 3.4 | `github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip` | preserve the complete source tree, including `CMakeLists.txt`, `CMake/`, `src/`, `include/` and `LICENSE.md`, for `BIKE_BUILD_GUI=ON` |
| cairo | 1.18.4 | `cairographics.org/releases/cairo-1.18.4.tar.xz` | keep `src/` + `COPYING*` — headers/provenance only (see below) |
| eigen | 3.4.0 | `gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz` | keep `Eigen/`, `unsupported/` + `COPYING.MPL2` |

### Linux aarch64 artifacts

The committed URLs identify the glibc Linux aarch64 artifacts, not x86_64 or
musl builds. Acquisition retains the complete platform-neutral trees as well.

| Dependency | Retained archive | Extracted files |
|---|---|---|
| MuJoCo 3.12.0 | `mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl` | `mujoco/include/` → `include/`; `mujoco/libmujoco.so.3.12.0` → `lib/`; both license files from `mujoco-3.12.0.dist-info/licenses/` → root |
| DuckDB 1.5.6 | `libduckdb-linux-arm64-1.5.6.zip` (upstream asset: `libduckdb-linux-arm64.zip`) | `duckdb.h`, `duckdb.hpp` → `include/`; `libduckdb.so` → `lib/`; initial complete MIT block in the pinned `duckdb.hpp` → `LICENSE` |
| NumPy OpenBLAS 2.5.2 | `numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl` | `numpy.libs/libscipy_openblas64_-bb25d554.so` and `numpy.libs/libgfortran-e1b7dfc8-d8198c01.so.5.0.0` → `lib/`; `numpy-2.5.2.dist-info/licenses/LICENSE.txt` → root |

The NumPy wheel is only transport: its package files are never installed or
imported by the root graph. Static ELF inspection of the exact pinned image
shows that OpenBLAS requires the bundled, renamed `libgfortran` and already
carries `$ORIGIN` RPATH. Both images therefore travel together during
acquisition and installation. The bundled GCC runtime needs the ordinary
platform `libz`, `libm`, `libgcc_s` and glibc runtime. Its redistributable
license text is preserved verbatim in the wheel's combined `LICENSE.txt`.

The four archive hashes and consumed-file hashes were measured from upstream
bytes on 2026-10-21 without executing any downloaded artifact. The lock retains
the full values. The Linux DuckDB asset's SHA-256 also matches the digest from
[the official v1.5.6 release API](https://api.github.com/repos/duckdb/duckdb/releases/tags/v1.5.6).
This establishes acquisition provenance; it is not a successful build or a
runtime qualification result.

- **MuJoCo**: the wheel is purely the transport for the already-pinned
  3.12.0 binaries — the extracted dylib is byte-identical (SHA-256
  `9e772461…`) to the one the `.venv` package ships, so the standalone build
  links the same library without a Python edge. No interpreter, package, or
  wheel directory enters the build closure; nothing is "installed" from the
  wheel, only files copied out of the zip.
- **DuckDB**: the PyPI wheel statically links the engine into
  `_duckdb.so` and ships no C SDK, so the native SDK comes from the upstream
  release zip instead.
- **Cairo**: upstream build tooling (meson) requires Python, so the
  qualified Python-free workflow consumes a verified binary SDK rather than
  an upstream source build; this task locks the source archive for
  provenance and header access only (binary SDK lands at T50/T59).

### Recording and application options (T38–T51)

The recording/analyst targets consume the locked DuckDB C SDK directly.
`BIKE_BUILD_GUI=OFF` keeps the GLFW window code disabled and does not initialize
a display. Enabling it requires the full, pre-extracted GLFW source archive;
the earlier headers-only tree is insufficient. There is no download or
dependency extraction in CMake. The archive/header checks retain the locked
3.4 identity; full platform/installer qualification remains T58/T59.

Native PNG generation is available without another SDK. To select Cairo's
Unicode text renderer, explicitly provide `BIKE_CAIRO_LIBRARY`,
`BIKE_CAIRO_INCLUDE_DIR` and `BIKE_CAIRO_LIBRARY_SHA256`. The library fingerprint
and the locked 1.18.4 public header are checked; this optional SDK was neither
acquired nor executed in the source-only T38–T51 delivery. Its transitive
runtime libraries and redistribution licenses must be qualified at T59.

Offline HTML uses the native interactive renderer when a genuine local Plotly
bundle has not been supplied. `bike::viz::set_plotly_resource()` can select a
real local bundle; no CDN request, Python runtime or pretend Plotly bundle is
used. The renderer identity is explicit in the resulting HTML. This fallback
does not establish Plotly output parity; the plotting review records that gate.

## Explicit Linux acquisition

`tools/linux/acquire_deps.sh` is an operator/image-build acquisition step. It
reads the committed schema-2 lock, verifies each archive with `sha256sum`
before extracting it, checks every required file and license, writes
`BIKE_VERSION`, and publishes each staged tree only after validation. It runs
no downloaded code. Existing extracted dependency trees are not replaced;
use a fresh destination. A retained archive with a bad hash is an error and
is not silently downloaded again.

```bash
bash tools/linux/acquire_deps.sh native/dependencies.lock.json /path/to/new/deps linux-aarch64
```

The committed Linux Dockerfile uses this script in its explicit `acquire`
stage and copies the result to `/opt/bike-deps`. The container driver exposes
that tree as `/src/.deps` while preserving it across source synchronization.
Its image-build context contains only the Dockerfile, helper and lock, so
host `.venv`, SDKs and build trees are not sent into the image context.
No CMake configure/build/install rule calls acquisition.

GLFW extraction preserves the complete 3.4 archive, including the top-level
and `src/CMakeLists.txt`, CMake modules, headers and platform sources. The
lock now also verifies those two build files. Re-extract a historical
headers-only tree before GUI configure; the archive hash itself is unchanged.
On Linux, `RecordingApplication.cmake` explicitly selects X11 and disables
Wayland before adding GLFW. Xvfb smoke is separate from desktop acceptance.

## Record-only numerical provider

The Python-free graph consumes facts from `native/numerics.lock.json` and
never invokes Python to locate a provider. Linux requires the record's libc
to be `glibc` and the compiler policy's host glibc probe to agree. Each
recorded `numpy.libs/<file>` maps to the verified dependency's `lib/<file>`,
must identify the locked scipy-openblas image, remain within that dependency
root after symlink resolution, and match the record's SHA-256. System
OpenBLAS and site-packages locations are not substitutes. The lock resolver
exports `BIKE_DEP_numpy_openblas_LIBRARIES` for installing both provider and
bundled runtime; `Bike::NumericsProvider` imports only the provider itself.

The supplied snapshot's generated numerics record still needs the real
Phase-1 probe regeneration on both supported platforms. Its old Darwin
entry does not contain all eight required operations and it has no measured
Linux entry. This phase intentionally leaves that generated record intact;
the root graph fails closed on stale/missing facts. Regenerate through the
existing `tools/native_dependencies.py --write-numerics-record` workflow
when project execution is authorized, then run the planned platform gates.

## Verifying a populated tree without configuring

```bash
for d in .deps/*/; do echo "== $d $(cat "$d/BIKE_VERSION")"; done
shasum -a 256 .deps/_archives/*            # compare against the lock
shasum -a 256 .deps/mujoco-3.12.0/lib/libmujoco.3.12.0.dylib
# -> 9e7724614fb0b3f346758ff4158ef65e7c398f2caa3e9daab41ed210b4ab689c
```

`cmake -S . -B build/native-release` then re-verifies everything and fails
closed on any deviation — the negative cases (missing root, wrong
version, tampered file, missing TLS ABI and absent platform key) are covered by the `migration_T03`
CTest script.
