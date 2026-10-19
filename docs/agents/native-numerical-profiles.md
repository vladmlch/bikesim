# Native numerical profiles

The native port mirrors the *local* NumPy/CPython oracle bit-for-bit.
That contract is platform-sensitive: NumPy lowers several operations to
different instructions per CPU/OS, and CPython's `math` module does not
always match the `np.*` ufunc of the same name. This file records which
lowering each qualified profile selects, where the dispatch is decided,
and what is not yet qualified. Task T11 produced this record; the
regression corpus lives in
`tests/reference/test_native_numeric_portability.py` and the standalone
probe in `tools/numeric_portability_probe.py` (schema 1 JSON, nonzero
exit on any bitwise mismatch).

Golden rules, unchanged:

* Oracles are the pinned NumPy 2.5.2 / local CPython — never the native
  implementation. Comparisons are `view(np.uint64)` bitwise, never
  tolerances.
* A NaN-payload guarantee is *local only*: the corpus asserts the
  payload behavior of the running interpreter's NumPy build and does
  not claim cross-dispatch stability.
* Dispatch follows the same capability/source guard upstream uses —
  never a bare `__linux__` and never a silent Python fallback.

## Qualified profiles

| profile | interpreter | compiler | provider | status |
|---|---|---|---|---|
| macOS arm64 | CPython 3.14.3 (AppleClang) | AppleClang (`native/build`) | Accelerate (system dyld cache) | qualified — zero mismatches |
| Linux aarch64 clang | CPython 3.13.16 | clang++-18 (`native/build-linux-clang`) | scipy-openblas ILP64 0.3.34.0.0 (neoversen1, 16 threads) | qualified — zero mismatches |
| Linux aarch64 gcc | CPython 3.13.16 | g++-13 (`native/build-linux-gcc`) | scipy-openblas ILP64 0.3.34.0.0 | qualified — zero mismatches |
| Linux x86_64 | — | — | — | **not qualified** (no lane; SSE2 min/max and x87/SSE libm paths differ from NEON) |

All three qualified artifacts pass the same 143-record probe corpus and
the reference pytest slice (`test_native_numeric_portability.py` +
`test_native_attachment_wrench.py`).

## Per-operation lowering decisions

### `np.tanh` → `numeric::np_tanh` (`numeric_numpy.hpp`)

NumPy's Highway `simd_tanh_f64` (lut18x16 polynomial,
`loops_hyperbolic.dispatch.cpp.src`) scalarized one lane at a time:
same table, same interval mapping, same evaluation order, same
saturating branch for `|x| >= 32`. `std::tanh` diverged on 29,048 of
2,000,000 probe values; the LUT port is bitwise-clean.

### `np.hypot` → `std::hypot` (`binding.cpp` `_debug_pneumatic_math`)

`npy_hypot` is the platform libm call; on both qualified libms the bits
match `np.hypot`. This is the `np.*` ufunc contract only — see
`math.hypot` below for the *different* CPython algorithm.

### `math.hypot` → `numeric::python_vector_norm` (`numeric_norm.hpp`)

CPython's `vector_norm` (DoubleLength compensated algorithm from
`mathmodule.c`), not `np.hypot`. The two-oracle distinction is
explicit: the `_debug_numeric_contract` hook name is `python_hypot`
and its test asserts against `math.hypot`, never `np.hypot`.

### `np.minimum` / `np.maximum` → `numeric::np_minimum` / `np_maximum` (`numeric_numpy.hpp`)

On `__aarch64__` NumPy's `scalar_min_d`/`scalar_max_d` and the
`npyv_minn/maxn` vector loop both lower to the NEON `fmin`/`fmax`
instructions (`loops_minmax.dispatch.c.src`). The instruction
semantics, verified bitwise against NumPy 2.5.2 on macOS arm64 and
Linux aarch64:

* A **signaling** NaN is returned quieted (quiet bit set, payload and
  sign preserved) and wins over a quiet NaN *in either operand
  position*: `np.minimum(qnan_a, snan_b)` is `quiet(snan_b)`, not the
  first operand. (Found during T11 — an earlier reading propagated the
  first NaN unconditionally and was caught by the corpus pair
  `(0x7ff80000000000aa, 0x7ff0000000000001)`.)
* Two NaNs of the same class resolve to the first operand —
  `(qnan_a, qnan_b) → qnan_a`, `(snan_a, snan_b) → quiet(snan_a)`.
* Zero ties: `fmin` keeps `-0.0`, `fmax` keeps `+0.0`.

This differs from NumPy's x86 SSE2 lowering (first-NaN wins) — the
port is correct for the qualified aarch64 profiles only.

### `np.sum` → `numeric::numpy_sum` (`numeric_sum.hpp`)

`np.sum` is the `add.reduce` loop seeded with the `+0.0` identity
folded over `DOUBLE_pairwise_sum` (`loops.c.src`): sequential for
`n < 8`, eight accumulators with tail fold for `n <= 128`, halving
recursion on a multiple of 8 beyond. `numpy_pairwise_sum` keeps the
raw inner loop (its `-0.0` seed); `numpy_sum` adds the identity fold
`0.0 + r` so `np.sum([-0.0])`/`np.sum([])` return `+0.0`. Production
call sites mirroring `np.sum` were moved off the raw helper;
`builtin sum` keeps `numeric::PythonSum` (CPython Neumaier).

### `np.sin` / `np.cos` → `numeric::np_sin` / `numeric::np_cos`; `math.sin` / `math.cos` → `numeric::math_sin` / `math_cos` (`numeric_sincos.hpp`)

f64 sin/cos use the scalar libm dispatch (`npy_sin`/`npy_cos`); the
wrappers also keep each call unfused — a volatile argument blocks the
compiler's `__sincos_stret` merge, which shifted results by 1 ulp.

**Apple arm64 only** (`__APPLE__ && __arm64__`): upstream applies
`WORKAROUND_APPLE_TRIG_BUG`
(`npy_math_internal.h.src:296-313`) — for non-finite input the
function returns `x - x` instead of calling libm. The AArch64 `fsub`
keeps the operand's NaN sign and payload (quieting sNaN), where Apple
libm canonicalizes to `+qNaN`. Mirrored under the identical
preprocessor guard inside `np_sin`/`np_cos`: `np.cos(0xfff8000000000000)`
is `0xfff8000000000000` on macOS, but `0x7ff8000000000000` via a plain
libm call. On Linux there is no workaround and the wrappers are plain
libm calls — matching `npy_sin`/`npy_cos` on that profile.

The two oracle families are NOT interchangeable on Apple arm64 for
non-finite input: CPython's `math.sin`/`math.cos` call libm directly,
so a negative-NaN angle returns `+qNaN` where `np.sin` returns the
sign-preserving `x - x`. Every sin/cos call mirrored from Python, paired
or lone, takes the family of its mirror (uniform rule, so each site is
audited against its oracle directly) — `np.*` mirrors
(`drivetrain/chain.cpp`'s `chain_center_gradient` normal,
`writers/distributed_tire.cpp`, `pneumatic/geometry.cpp`, the debug
binding) use `np_sin`/`np_cos`; `math.*` mirrors
(`drivetrain/chain.cpp`'s `unwrap`, `leg_loop.py`,
`rider_envelope.py`, `lean_limit.py`, scalar `rider_control.py` wraps,
`wheelie.py` — i.e. the `rider/spindle_*.cpp` and
`runtime/research_metrics.cpp` atan2 sites) use
`math_sin`/`math_cos` (`research_metrics.cpp`'s lone `cos(road_pitch)`
included). Bare `std::sin`/`std::cos` remains only where no Python
mirror exists: the viewer circle in `viz/geometry.cpp`.

### `np.interp` general table → `numeric::np_interp_segment` (`numeric_numpy.hpp`)

`arr_interp`'s interior expression `slope*(x - dx[j]) + dy[j]`
(`compiled_base.c`) is emitted once per wheel build; its multiply/add
contraction is a *wheel-build* property, not a platform property, and
the two qualified wheels disagree:

| profile | measured `np.interp` lowering | projected define |
|---|---|---|
| macOS arm64 (AppleClang wheel) | **contracted** — one fmadd rounding | `BIKE_NUMPY_INTERP_CONTRACTED=1` |
| Linux aarch64 (scipy-openblas GCC wheel) | **separate** — two roundings | `BIKE_NUMPY_INTERP_CONTRACTED=0` |

The dependency probe measures the installed `np.interp` against
`math.fma` and separate multiply/add on 4096 deterministic interior
queries (~1500 of them distinguish the two forms; the vote must be
unanimous) and records `numerics.interp.lowering` in the schema-1
document. `BikeNumerics.cmake` fails closed on any other value and
projects the define at directory scope so every first-party target
lowers `numeric::np_interp_eval` identically (the helper is an inline —
an inconsistent define would be an ODR-visible bitwise fork).
`np_interp_segment` additionally folds in the C source's
right-endpoint NaN retry and constant-segment fallback, so all four
mirrors — `biketyre::np_interp` (`tyre/profile.cpp`),
`np_interp_unit_grid` (`numeric_numpy.hpp`), `spindle_math.cpp`'s
`interp`, `drivetrain/motor.cpp`'s `interp` — share one implementation.
`_debug_interp` drives `biketyre::np_interp` itself; the probe corpus
covers random interior queries, exact knots, endpoint clamps,
infinities, constant segments, and non-finite `fp` lanes.

Contraction is **per-TU**, not per-wheel: the same Linux wheel that
keeps `np.interp` separate contracts `umath_linalg.cpp`'s `abs2`
residual loop (`res += el*el`) into fmadd — `least_squares.cpp` keeps
its `std::fma` unconditionally, verified bitwise on 3000
residual-bearing solves per Linux lane.

### `float * ndarray` / `np.multiply(scalar, array)` → `numeric::np_multiply_broadcast`

A NaN lane follows the same AArch64 precedence `np.maximum` documents
— an sNaN quiets and wins from either operand position, and a
same-class dual-NaN resolves to fmul operand 0's payload. Which
source operand the wheel's broadcast kernel loads into operand 0 is a
*wheel-build* property — the two qualified wheels disagree:

| profile | measured `nan_winner` | projected define |
|---|---|---|
| macOS arm64 (AppleClang wheel) | **scalar** — scalar in operand 0 | `BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=0` |
| Linux aarch64 (scipy-openblas GCC wheel) | **element** — array lane in operand 0 | `BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN=1` |

The helper walks operand-0 sNaN → operand-1 sNaN → operand-0 qNaN →
operand-1 qNaN → product for the projected order; the sNaN rule is
hardware-identical on every AArch64 lane, so `nan_winner` only has to
project the same-class tie. Finite lanes are the plain product. The
dependency probe measures `np.multiply(scalar, array64)` on 64
dual-NaN lanes through both the ufunc call and the `float * ndarray`
sugar (they must agree unanimously) and records
`numerics.multiply_broadcast`. `BikeNumerics.cmake` fails closed on
any other winner and projects the define at directory scope.
`pneumatic/carcass.cpp`'s `advance_unloaded` rescale is the current
production site.

### `a @ b` / `np.dot` / `x.dot(y)` 1-D → `blas::ddot` (`cblas_abi.hpp`)

NumPy's `DOUBLE_dot` (`arraytypes.c.src`) does not return the raw cblas
result: it seeds `double sum = 0.` and accumulates `sum +=
cblas_ddot(chunk)`. The literal addend canonicalizes a `-0.0` provider
fold to `+0.0` and quiets a signaling-NaN payload — `a @ b` on an
all-`±0` adversarial pair is `+0.0` where bare `cblas_ddot` returns
`-0.0`. `blas::ddot` therefore returns `0. + <provider ddot>` on both
backends; every production call site ports a `DOUBLE_dot` expression,
so the seed lives in the one wrapper rather than at ~30 call sites.
(The probe's 1-D oracle is `a @ b`, never `np.dot`: `np.dot` 1-D@1-D
takes a different kernel — `PyArray_InnerProduct`'s own loop — and
legitimately differs on the same inputs.)

### Rider fixed-size linear algebra → `rider::*` (`spindle_linalg.hpp`)

The spindle controller's packed (3,3)/(2,2) matvec/matmul mirrors and
`np.linalg.norm` all dispatch through the *provider* BLAS under NumPy,
whose accumulation order is provider-private — a scalar C++ loop is
not a portable stand-in:

| oracle expression | NumPy dispatch | `rider::` helper |
|---|---|---|
| `a @ x` (n,n) C-order | `dgemv` column-dot path (Accelerate: sequential rows; scipy-openblas: fused accumulation) | `mat_vec` / `mat_vec2` → `dgemv(RowMajor, NoTrans)` |
| `a.T @ x` (F-view) | `dgemv(RowMajor, Trans)` — fused on **both** providers | `mat_t_vec` / `mat_t_vec2` → `dgemv(RowMajor, Trans)` |
| `(a.T @ x)[[0,2]]` | same Trans call, lane slice | `mat_t_vec_xz` |
| `a @ b` | `dgemm(RowMajor, NoTrans, NoTrans)` | `mat_mul` |
| `a.T @ b` | `dgemm(RowMajor, Trans, NoTrans)` | `mat_t_mul` |
| `np.linalg.norm(v)` 1-D | `sqrt(x.dot(x))` → seeded `DOUBLE_dot` | `np_norm` → `sqrt(blas::ddot(v,v))` |

The non-transposed `M @ v` case is the one place the providers
*disagree* in kernel: Accelerate's NoTrans entry produces the
sequential multiply/add row fold while scipy-openblas fuses — which is
exactly why the implementation calls the provider instead of
emulating. `mat_t_vec2` additionally fixed a latent port bug: the
native lean-limit `arm_goal` evaluated `rotation.T @ goal` with plain
multiply/add where both wheels fuse — the helper's dgemv matches
bitwise on all lanes. `_debug_rider_linalg` drives the production
helpers on tightly C-contiguous operands; the probe corpus covers the
(3,3)/(2,2) ops, the xz slice, 1-D norms of lengths 0–8, and
adversarial IEEE values.

`y += A.T @ x` (in-place add of a matmul result) is **two** oracle
ufuncs — the product is rounded into a fresh vector, then the `+=`
rounds the sum. `dgemv(beta=1)` is not a substitute: scipy-openblas's
Trans kernel fuses the `beta*y` addend into the dot accumulator (one
rounding, ~40% of random lanes differ bitwise), while Accelerate's
happens to round first — a beta=1 port is silently Accelerate-only
correct. Every `+=`/accumulate site therefore calls `dgemv`/`dgemm`
with `beta=0` into scratch, then runs an elementwise `+=` loop (the
`writers/rider_contacts.cpp` pad and grip `qfrc += jrel.T @ f` sites
are the canonical example; `distributed_tire.cpp`/`tire.cpp`/
`pneumatic_tire.cpp` already used the scratch form).

### `np.sign`, `pyfloat_pow`, `blas::ddot`

Otherwise unchanged from earlier tasks; the corpus exercises
`np.maximum`-fold, `np.interp`, `**`, and the BLAS-backed dot paths
through the existing per-file tests.

## Provider contamination controls

`tools/numeric_portability_probe.py` records the interpreter,
`numpy.__config__`, `__cpu_features__`, threadpoolctl BLAS pools, the
selected artifact path + SHA-256, and every NumPy/SciPy/MuJoCo module
loaded before the extension. It was run in fresh processes with NumPy
alone, SciPy-first, and SciPy+MuJoCo-first import orders on all three
artifacts — 143/143 records equal in every order on every artifact
(schema-1 JSON under `output/numeric-portability/`).

## Effective FP flags

`native/CMakeLists.txt` keeps `-ffp-contract=off` project-wide:
explicit `std::fma` stays fused while ordinary multiply/add rounds
separately, matching the oracle's scalarized operation order. The one
site where the oracle's own contraction differs per wheel —
`np.interp`'s interior expression — is pinned through
`BIKE_NUMPY_INTERP_CONTRACTED` (measured by the dependency probe),
never left to a flag default. No `-ffast-math`/`ffinite-math-only` —
`x - x`, `std::signbit`, and NaN branches all keep IEEE semantics.

## Demonstrated-and-fixed divergences (permanent fixtures)

1. **fmin/fmax sNaN precedence** — `(qNaN, sNaN)` returned the qNaN
   instead of the quieted sNaN. Fixture: `[qnan, snan]`/`[snan, qnan]`
   rows in `test_minmax_signed_zero_and_nan_fixture`.
2. **np.sin/np.cos negative-NaN passthrough** — plain libm returned
   `+qNaN` for `0xfff8…`/`0xfff1…` inputs where `npy_sin`'s `x - x`
   workaround preserves sign and payload. Fixtures: `0xFFF8000000000000`
   and `0xFFF123456789ABCD` in `ADVERSARIAL_UINT64`, exercised by both
   sin/cos tests.
3. **np.sum `-0.0` identity** — the raw pairwise helper seeds `-0.0`;
   `np.sum` must return `+0.0`. Fixtures: length-0 and all-`-0.0`
   vectors at every `SUM_LENGTHS` length.
4. **np.interp per-wheel contraction** — all four mirrors hard-coded
   `std::fma` while the Linux wheel rounds the multiply and add
   separately. Fixtures: the `interp` records' `n{2,3,5,9,17}-random`
   cases plus the `test_interp_matches_np_interp_general_table` slice;
   the lowering is now probe-measured and CMake-projected.
5. **provider BLAS vs scalar emulation** — sequential/FMA scalar folds
   matched neither Accelerate's NoTrans row-matvec (sequential) nor
   scipy-openblas's fused accumulation across `M @ v`, `M.T @ v`,
   `M @ M`, `M.T @ M`, the 2-D variants, and `np.linalg.norm`'s
   `x.dot(x)` reduction; `rotation.T @ goal` in the lean-limit port was
   a latent wrong answer on both platforms. Fixtures: the
   `rider-*`/`blas-*` probe records and the
   `test_native_numeric_portability` linalg slice; the helpers now call
   `blas::dgemv`/`dgemm`/`ddot` in `spindle_linalg.hpp`.
6. **`a @ b` `+0.0` seed** — bare `cblas_ddot` returns `-0.0` on an
   all-`±0` adversarial pair where `DOUBLE_dot`'s `sum = 0.` seed
   yields `+0.0`. Fixture: the `blas-dot n3` probe record; `blas::ddot`
   now carries the seed.
7. **`+=` on a BLAS product fused under beta=1** —
   `writers/rider_contacts.cpp` accumulated `qfrc += jrel.T @ f` with
   `dgemv(beta=1)`, which Accelerate happened to evaluate
   round-then-add like the oracle's two ufuncs, while scipy-openblas
   fuses the addend into the dot accumulator (~40% lane drift). The
   pad and grip accumulates now produce the product with `beta=0`
   into scratch, then `+=` elementwise — identical to the oracle on
   every provider. Fixtures: `test_flat_contacts_qfrc_and_diagnostics_bitwise`
   and the multi-step state tests in
   `tests/reference/test_native_rider_contacts.py` (previously
   bitwise-red on Linux only).
8. **broadcast multiply NaN precedence per wheel** —
   `pneumatic/carcass.cpp` evaluated `float * ndarray`
   (`np.multiply(decay, advected)`) as a plain scalar loop; AArch64
   fmul resolves a NaN lane by class then position — an sNaN quiets
   and wins either side, a same-class dual-NaN takes operand 0 — and
   the ufunc's operand order differs per wheel build: the AppleClang
   wheel keeps the scalar in operand 0 (scalar NaN wins a same-class
   tie), the manylinux wheel's broadcast loop puts the array lane
   there (element NaN wins). The port called neither: the compiler's
   own operand choice decided. `numeric::np_multiply_broadcast` now
   walks the full precedence for the measured operand order via
   `BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN`, projected from the
   `numerics.multiply_broadcast` dependency record. Fixtures:
   `test_multiply_broadcast_nan_class_fixture` (every NaN-class
   cross-combination) and `test_carcass_advance_unloaded_validation`'s
   NaN-dt propagation case (previously bitwise-red on Linux only).

## Not qualified — do not claim

* **x86_64 Linux**: the SSE2 `max_sd`/`min_sd` scalar loop, the x87/SSE
  libm, and AVX2/FMA3 `simd_tanh_f64` lane order all differ from NEON.
  The aarch64 evidence says nothing about them. First x86_64 lane must
  re-run this suite plus the probe before any claim.
* **macOS Intel** and any non-pinned NumPy/CPython/libm combination.
