# Native dependency acquisition (`.deps/`)

The standalone Python-free build (`cmake -S . -B build/native-release`)
consumes dependency SDKs from a repository-root `.deps/` tree that is
populated **explicitly before configure**. Configure, build, and install make
no network calls and run no dependency code-generation Python;
`native/cmake/Dependencies.cmake` only verifies what is already on disk and
fails configure closed when anything is missing, wrong-versioned, or
hash-mismatched.

`.deps/` is gitignored on purpose: it holds multi-hundred-megabyte binary
SDKs. `native/dependencies.lock.json` (committed) is the integrity contract —
every entry records the acquisition URL, the retained archive's real SHA-256,
the license, and SHA-256 of every file the build consumes.

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

1. Download `acquisition.url` into `_archives/` under exactly the recorded
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
| mujoco | 3.12.0 | `files.pythonhosted.org/…/mujoco-3.12.0-cp314-cp314-macosx_11_0_arm64.whl` | unzip the wheel; copy `mujoco/include/` and `mujoco/libmujoco.3.12.0.dylib` → `lib/`; `LICENSE`, `LICENSES_THIRD_PARTY.md` |
| duckdb | 1.5.6 | `github.com/duckdb/duckdb/releases/download/v1.5.6/libduckdb-osx-universal.zip` | `libduckdb.dylib` → `lib/`; `duckdb.h`, `duckdb.hpp` → `include/`; `LICENSE` |
| tomlplusplus | 3.4.0 | `github.com/marzer/tomlplusplus/archive/refs/tags/v3.4.0.tar.gz` | keep `include/` + `LICENSE` |
| rapidjson | 24b5e7a8…7102f | `github.com/Tencent/rapidjson/archive/24b5e7a8b27f42fa16b96fc70aade9106cf7102f.tar.gz` | keep `include/` + `LICENSE` |
| glfw | 3.4 | `github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip` | preserve the complete source tree, including `CMakeLists.txt`, `CMake/`, `src/`, `include/` and `LICENSE.md`, for `BIKE_BUILD_GUI=ON` |
| cairo | 1.18.4 | `cairographics.org/releases/cairo-1.18.4.tar.xz` | keep `src/` + `COPYING*` — headers/provenance only (see below) |
| eigen | 3.4.0 | `gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz` | keep `Eigen/`, `unsupported/` + `COPYING.MPL2` |

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

## Why a document rather than an acquisition script

Acquisition is a deliberate, audited, one-time-per-machine act: the lock
records real hashes of exactly what was fetched, and reviewers check the
recipe table against `uv.lock`/upstream release pages. A script would make
silent re-fetching easy (defeating the "explicit, pre-acquired" contract the
build is meant to enforce), and per the no-network-at-configure rule it could
never run inside configure/build/install anyway. If a scripted form is ever
wanted it belongs in `tools/` as an operator-invoked command — never in the
build path.

## Verifying a populated tree without configuring

```bash
for d in .deps/*/; do echo "== $d $(cat "$d/BIKE_VERSION")"; done
shasum -a 256 .deps/_archives/*            # compare against the lock
shasum -a 256 .deps/mujoco-3.12.0/lib/libmujoco.3.12.0.dylib
# -> 9e7724614fb0b3f346758ff4158ef65e7c398f2caa3e9daab41ed210b4ab689c
```

`cmake -S . -B build/native-release` then re-verifies everything and fails
closed on any deviation — the four negative cases (missing root, wrong
version, tampered file, missing TLS ABI) are covered by the `migration_T03`
CTest script.
