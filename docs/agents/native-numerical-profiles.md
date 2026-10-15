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

All three qualified artifacts pass the same 69-record probe corpus and
the 160-test pytest slice (`test_native_numeric_portability.py` +
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
sign-preserving `x - x`. Every same-operand sin/cos call site was
audited against its Python mirror — `np.*` mirrors
(`drivetrain/chain.cpp`, `writers/distributed_tire.cpp`,
`pneumatic/geometry.cpp`, the debug binding) use `np_sin`/`np_cos`;
`math.*` mirrors (`leg_loop.py`, `rider_envelope.py`, `lean_limit.py`,
scalar `rider_control.py` wraps, `wheelie.py` — i.e. the
`rider/spindle_*.cpp` and `runtime/research_metrics.cpp` sites) use
`math_sin`/`math_cos`. Lone calls with no fusion partner keep bare
`std::sin`/`std::cos`.

### `np.sign`, `np.interp` unit grid, `pyfloat_pow`, `blas::ddot`

Unchanged from earlier tasks; the corpus exercises `np.maximum`-fold,
`np.interp`, `**`, and the BLAS-backed dot paths through the existing
per-file tests.

## Provider contamination controls

`tools/numeric_portability_probe.py` records the interpreter,
`numpy.__config__`, `__cpu_features__`, threadpoolctl BLAS pools, the
selected artifact path + SHA-256, and every NumPy/SciPy/MuJoCo module
loaded before the extension. It was run in fresh processes with NumPy
alone, SciPy-first, and SciPy+MuJoCo-first import orders on all three
artifacts — 69/69 records equal in every order on every artifact
(schema-1 JSON under `output/numeric-portability/`).

## Effective FP flags

`native/CMakeLists.txt` keeps `-ffp-contract=off` project-wide:
explicit `std::fma` stays fused while ordinary multiply/add rounds
separately, matching the oracle's scalarized operation order. No
`-ffast-math`/`ffinite-math-only` — `x - x`, `std::signbit`, and NaN
branches all keep IEEE semantics.

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

## Not qualified — do not claim

* **x86_64 Linux**: the SSE2 `max_sd`/`min_sd` scalar loop, the x87/SSE
  libm, and AVX2/FMA3 `simd_tanh_f64` lane order all differ from NEON.
  The aarch64 evidence says nothing about them. First x86_64 lane must
  re-run this suite plus the probe before any claim.
* **macOS Intel** and any non-pinned NumPy/CPython/libm combination.
