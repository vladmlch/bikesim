# Numerical-provider profiles — the qualified BLAS/LAPACK ABI

Date: 2026-10-14. Task T06 of `docs/superpowers/plans/2026-10-14-linux-macos-master.md`.
Evidence producer: `tools/native_dependencies.py` (`discover_numpy_provider`,
inside the schema-1 dependency probe).
Consumer: `native/cmake/BikeNumerics.cmake` (`BIKE_NUMERICS_BACKEND`), then
`tools/dev.py doctor --numerics` surfaces the same record read-only.

A wrong BLAS/LAPACK integer width is memory corruption, not a rounding
error. The native build therefore binds BLAS/LAPACK **only** through the
qualified provider record — never `find_package(BLAS)`, never an assumed
symbol name, never a width inferred from pointer size or the fact that an
address exists.

## 1. Supported backends

| Backend | Platform | Provider | BLAS int | LAPACK int |
|---|---|---|---|---|
| `ACCELERATE` | macOS arm64 | System Accelerate.framework (dyld cache) | 32-bit legacy CBLAS | 64-bit `NEWLAPACK$ILP64` |
| `NUMPY_OPENBLAS` | Linux aarch64 | `numpy.libs/libscipy_openblas64_-*.so` inside the NumPy 2.5.2 wheel | 64-bit (ILP64) | 64-bit (ILP64) |

NumPy is pinned at exactly `2.5.2` (`REQUIRED_NUMPY_VERSION`); any other
version, a system/distribution NumPy, or a wheel built against a different
provider fails qualification with the remediation to run
`uv sync --locked --group native` (`tools/dev.py bootstrap`). There is no
silent fallback to whatever BLAS the host happens to have.

## 2. Provider identity rules

### Darwin — `apple-system` identity

Accelerate lives in the dyld shared cache; `/System/Library/Frameworks/
Accelerate.framework/Versions/A/{Accelerate,Frameworks/vecLib.framework/
Versions/A/libBLAS.dylib,libLAPACK.dylib}` are install names, not files on
disk. The record therefore carries:

- `identity_kind: "apple-system"`, `sha256: null` — a dyld-cache image is
  never hashed and an SDK stub is never substituted;
- `build_id`: `macos-<sw_vers build>+accelerate-<LC_UUID of the loaded
  umbrella image>+sdk-<xcrun sdk build>` — real build identifiers read from
  the running system, e.g.
  `macos-26A434+accelerate-a697e7b9-…-64b38af1ccaa+sdk-26A425`;
- `role: "blas"` on libBLAS.dylib and `role: "lapack"` on libLAPACK.dylib;
- the full loaded Accelerate image list under
  `provider_config.accelerate_images`.

`require_same_library` still applies wherever a real path exists: two
entries resolving to different real files are a mixed provider and fail.

### Linux — `file` identity

The provider must be a file the NumPy wheel itself shipped:

- the loaded BLAS image (from `threadpoolctl`) must appear in the installed
  distribution's RECORD (`importlib.metadata.distribution("numpy").files`,
  compared by resolved path) — a path outside the wheel is a system-BLAS
  substitution and fails;
- `identity_kind: "file"` with the real SHA-256 of the resolved file;
  the digest is additionally cross-checked against the wheel's own RECORD
  hash entry when present;
- `build_id: null` — the SHA-256 is the identity;
- one provider only: zero matches means nothing qualified, two or more
  NumPy-owned BLAS images is a duplicate-provider failure.

Observed on the aarch64 verification host:
`/opt/bike-venv/lib/python3.13/site-packages/numpy.libs/
libscipy_openblas64_-bb25d554.so`,
SHA-256 `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`.

## 3. Integer widths — evidence, not inference

Widths come from the provider's own build configuration and its documented
symbol ABI, never from `sizeof(void*)` or symbol existence:

- **NUMPY_OPENBLAS**: `numpy.__config__.show(mode="dicts")` Build
  Dependencies must name `scipy-openblas` for both BLAS and LAPACK, and the
  OpenBLAS build string must contain `USE64BITINT`; the loaded library's own
  `scipy_openblas_get_config64_` must repeat `USE64BITINT`. The exported
  `…64_` spellings are the documented ILP64 interface.
- **ACCELERATE**: the 32-bit width belongs to the *legacy* `cblas_*` entry
  points (qualified symbols `cblas_ddot`/`cblas_dgemv`/`cblas_dgemm` —
  `$NEWLAPACK`-decorated variants are a different ABI and are rejected in
  the `symbols` table), and the 64-bit width belongs to the documented
  `dgelsd$NEWLAPACK$ILP64` interface.

`blas_integer_bits=None`, a mixed-width record, an LP64 provider, or a
backend/width combination outside the qualified table are all hard
failures.

## 4. Symbol qualification

Candidate names are never assumed. For each of the four qualified
operations — `ddot`, `dgemv`, `dgemm`, `dgelsd` — the probe:

1. scans every `*.so` under the installed `numpy` package for undefined
   imports matching the operation stem and recognised suffixes
   (`scipy_*_64_`, `scipy_cblas_*64_`, `…$NEWLAPACK[$ILP64]`, plain `_`),
   on Linux via `nm -D --undefined-only`, on Darwin via `nm -u`;
2. reads the *provider's* export table — `nm -D --defined-only` on Linux,
   `dyld_info -exports` on the dyld-cache image paths on Darwin;
3. records the exact accepted spelling in `symbols` and the full observed
   import set per op in `numpy_imports`. Ambiguous or unexported sets fail.

Qualified spellings (aarch64/arm64, both recorded verbatim in the record):

| Op | NUMPY_OPENBLAS | ACCELERATE |
|---|---|---|
| ddot | `scipy_cblas_ddot64_` | `cblas_ddot` |
| dgemv | `scipy_cblas_dgemv64_` | `cblas_dgemv` |
| dgemm | `scipy_cblas_dgemm64_` | `cblas_dgemm` |
| dgelsd | `scipy_dgelsd_64_` | `dgelsd$NEWLAPACK$ILP64` |

Every name passes `checked_symbol` (`[A-Za-z_][A-Za-z0-9_$]*`, fullmatch)
before it can appear in generated C or a ctypes lookup; `BikeNumerics.cmake`
re-checks the same grammar and the backend spellings at configure time.

## 5. Compile/link/run qualification

Metadata is necessary but not sufficient — the provider ABI is then
*executed*, in a disposable subprocess generated by
`_numerics_abi_probe_source` with the record's widths and `__asm__`-labelled
exact symbols:

1. `cc -fsyntax-only -std=c11 -Wall -Wextra -Werror` — compile-only type
   check first;
2. compile+link against the provider (`-framework Accelerate`, or the
   resolved `libscipy_openblas64_*.so` with an rpath);
3. run controls: `ddot`, rectangular `dgemv` (3×4), rectangular `dgemm`
   (2×3 · 3×2), a rank-1-deficient `dgelsd` solve (min-norm `x=(0.6,1.2)`),
   and the `dgelsd` workspace query.

Any compile error, link error, crash, timeout, or wrong numeric result is a
hard provider failure. The compiler version and the `BIKE-ABI-OK` line are
kept under `provider_config.abi_check`.

## 6. Threading and provider configuration

- `threading`: `{implementation, num_threads}` — on Linux from
  `threadpoolctl` cross-checked against `scipy_openblas_get_num_threads64_`;
  on Darwin `num_threads` is **measured** (thread count while a fixed
  3000×3000 dgemm spins; the workload and method are recorded in
  `provider_config.thread_measure`) because Accelerate exposes no pool
  query and `VECLIB_MAXIMUM_THREADS` is read-only evidence, never mutated.
- `provider_config.openblas`: wheel RECORD path, OpenBLAS version/core/
  config string, `num_threads_reported`.
- `provider_config.wheel.tags`: the wheel's platform tags.
- `numpy_show_config` / `numpy_show_runtime`: the full NumPy config and
  runtime tables for review (captured so the probe's stdout stays JSON).

The probe records provider threading; it never changes it.

## 7. Import-order / interposition control

SciPy ships its *own* LP64 OpenBLAS (`scipy.libs/libscipy_openblas-*.so`).
The probe takes a bitwise baseline of dot + lstsq results under NumPy-only,
then imports `scipy.linalg`, exercises it, and re-verifies:

- the bitwise result is unchanged, and
- on Linux the NumPy provider is still the loaded one (SciPy's image may
  coexist in the pool — it just may not serve NumPy); on Darwin the
  Accelerate images remain.

A changed provider or changed result means interposition is live and the
environment fails qualification. The independence is additionally tested in
both orders — `numpy→scipy` and `scipy→numpy` — in fresh interpreters.

## 8. Unsupported-provider remediation

Qualification fails closed with a named error and the remediation to
install the locked wheel (`uv sync --locked --group native` /
`tools/dev.py bootstrap`) when NumPy is absent, is the wrong version, was
built against a non-Accelerate/non-scipy-openblas provider, has no file
records (editable/system install), loads a BLAS outside the wheel, or when
SciPy interposition moves the provider. `BIKE_NUMERICS_BACKEND` only ever
selects between already-qualified records — it cannot conjure one.
