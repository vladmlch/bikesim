# Native scientific dependency matrix — wheel qualification

Date: 2026-10-14. Task T02 of `docs/superpowers/plans/2026-10-14-linux-macos-master.md`.
Baseline ledger: `docs/superpowers/audits/2026-10-14-linux-macos-baseline.md`.

Scope: **PyPI dependencies only.** Every wheel listed here is a third-party
binary. It says nothing about `bike_native` — the extension is compiled
per-platform from `native/`; a manylinux-tagged MuJoCo wheel does not make an
Apple-built artifact portable, and vice versa.

## 1. Declared profile (`pyproject.toml`)

Core `[project.dependencies]` (unchanged floors/pins unless noted):

| Requirement | Role |
|---|---|
| `numpy>=1.24.0` | core floor; the *exact* pin lives in the native group |
| `mujoco==3.12.0` | exact pin (was `>=3.0.0`; aligned with the native pin in T02) |
| `scipy==1.17.0`, `duckdb==1.5.6`, `pyarrow==26.0.0` | exact pins (unchanged) |
| `numba>=0.68`, `matplotlib>=3.7.0`, `glfw>=2.10.0`, `pytest>=7.0.0`, `plotly>=5.24`, `rich>=15.0.0`, `pytz>=2026.5` | floors (unchanged) |

`[dependency-groups]`:

| Group | Entries | Enforcement |
|---|---|---|
| `native` | `mujoco==3.12.0`, `numpy==2.5.2`, `nanobind>=3.1.0`, `threadpoolctl>=3.6` | The `==` entries are the enforced map: `bike_sim.native.artifact.required_dependencies()` returns `{'mujoco': '3.12.0', 'numpy': '2.5.2'}` and `check_dependencies()` rejects a mismatch before the extension loads. `nanobind`/`threadpoolctl` are `>=` floors and are deliberately NOT in the enforced map — threadpoolctl is provider-discovery/diagnostics tooling, not physics. |
| `dev` | `hypothesis>=6.168.4`, `nanobind>=3.1.0` | unchanged |

`requires-python = ">=3.13"` — unchanged; the floor was not raised.

## 2. Locked/installed versions per lane

`uv.lock` (universal lock, revision 3) resolves the same versions on every
lane. Verified imports on each interpreter:

| Package | Locked | macOS host (CPython 3.14.3, arm64) | Linux container (CPython 3.13.16, aarch64) |
|---|---|---|---|
| numpy | 2.5.2 | 2.5.2 | 2.5.2 |
| scipy | 1.17.0 | 1.17.0 | 1.17.0 |
| mujoco | 3.12.0 | 3.12.0 | 3.12.0 |
| numba | 0.68.0 | 0.68.0 | 0.68.0 |
| duckdb | 1.5.6 | 1.5.6 | 1.5.6 |
| pyarrow | 26.0.0 | 26.0.0 | 26.0.0 |
| nanobind | 3.1.0 | 3.1.0 | 3.1.0 |
| threadpoolctl | 3.7.0 | 3.7.0 | 3.7.0 |
| hypothesis | 6.168.4 | 6.168.4 | 6.168.4 |
| matplotlib | 3.11.1 | 3.11.1 | 3.11.1 |
| glfw | 2.10.2 | 2.10.2 | 2.10.2 |

`required_dependencies()` and `check_dependencies()` return/pass
`{'mujoco': '3.12.0', 'numpy': '2.5.2'}` on both interpreters.

## 3. Wheel qualification method

Exported requirements (hash-pinned, all locked platforms' hashes included):

```sh
uv export --locked --group native --group dev --no-emit-project -o output/portability/native-requirements.txt
```

Downloads used an ephemeral pip helper (`uv run --frozen --group native
--with pip python -m pip download ...`) — a packaging tool, not an
application requirement. 38 wheels per lane (39 resolved distributions
minus `colorama`, correctly skipped by its `sys_platform == 'win32'`
marker).

- **macOS arm64** (native probe on the 3.14.3 host): `--only-binary=:all:` →
  `output/portability/wheelhouse/`
- **Linux aarch64** (in-container probe, Ubuntu 24.04, CPython 3.13.16):
  `--only-binary=:all:` → `output/portability/wheelhouse-linux-aarch64/`
- **Linux x86_64** (cross-probe from the macOS host; availability only,
  nothing installed): `--platform manylinux_2_28_x86_64 --platform
  manylinux_2_17_x86_64 --platform manylinux2014_x86_64 --python-version
  3.13 --only-binary=:all:` → `output/portability/wheelhouse-linux-x86_64/`

### x86_64 floor finding (recorded verbatim, not papered over)

The conservative probe `--platform manylinux_2_17_x86_64 --python-version
3.13 --only-binary=:all:` **fails**:

```text
ERROR: Could not find a version that satisfies the requirement contourpy==1.3.3 (from versions: 1.3.0, 1.3.1, 1.3.2)
ERROR: No matching distribution found for contourpy==1.3.3
```

This is a floor property, not a missing package: the locked dependency
intersection does not support glibc 2.17. Lowest published x86_64 cp313
tags: contourpy/numpy/scipy/mujoco/numba/llvmlite/pillow
`manylinux_2_27`, duckdb `manylinux_2_26`, **pyarrow `manylinux_2_28`**
(the binding constraint), kiwisolver/fonttools/matplotlib/hypothesis
`manylinux2014`/`manylinux_2_17`. ⇒ The x86_64 lane's qualification floor
is **glibc ≥ 2.28** (`manylinux_2_28_x86_64`), satisfied by the target
Ubuntu 24.04 image (observed glibc 2.39-0ubuntu8.9). No target claims
compatibility below glibc 2.28.

Probe caveat: pip's `--platform` matches wheel tags **literally** — it
does not expand a `manylinux_2_28` spec downward. The x86_64 probe must
therefore enumerate the tags the target accepts (2_28 + 2_17 + 2014);
with the bare `manylinux_2_28_x86_64` spec, `hypothesis==6.168.4`
(whose only Linux x86_64 wheel is `manylinux_2_17`) is falsely rejected:
`ERROR: No matching distribution found for hypothesis==6.168.4` even
though `requires_python` is `>=3.10` and a compatible wheel exists.

### macOS floor

The downloaded cp314 arm64 wheels carry `macosx_11_0` through
`macosx_14_0` tags; the highest is **scipy-1.17.0 → `macosx_14_0_arm64`**
(numpy-2.5.2 cp314 also carries `macosx_14_0`). The Python-dependency
floor on Apple Silicon is therefore macOS 14.0. This records what the
locked wheels already require; it does not change any `bike_native`
build-floor decision (later task's gate).

### Linux aarch64 floor

Same intersection: highest aarch64 tag is pyarrow's
`manylinux_2_28_aarch64` → glibc ≥ 2.28, satisfied by Ubuntu 24.04
(glibc 2.39) on aarch64.

## 4. Wheel manifests (exact filenames + sha256)

### macOS arm64 — `output/portability/wheelhouse/` (cp314, host 3.14.3)

```text
0f17b89f2a4eaaedc4f28c622998aa690564b3012a396a4ffad0821007fe03ba  absl_py-2.5.0-py3-none-any.whl
cbedb772ed74ff5be440fa8eee9bd49f64f6e3fc09436d9c7d8f1c287b121d77  contourpy-1.3.3-cp314-cp314-macosx_11_0_arm64.whl
85cef7cff222d8644161529808465972e51340599459b8ac3ccbac5a854e0d30  cycler-0.12.1-py3-none-any.whl
c79c6d222b1d015cde73b5139087186b00db65357fb4e2c94c2308fbbf465a72  duckdb-1.5.6-cp314-cp314-macosx_11_0_arm64.whl
b5df7341f54dbe1405a4450b2741207b4a8c279780402b45f87202b94dfc52b4  etils-1.14.0-py3-none-any.whl
fd1e3094f42d806d3d7c79162fc59e5910fcbe3a7360c385b8da969bc4493745  fonttools-4.63.0-cp314-cp314-macosx_10_15_universal2.whl
b57ddbafedfaef7018c1ecab32aa200a9d7ca26b77965f64e48b70061249d279  fsspec-2026.7.0-py3-none-any.whl
c000b8a5e5fb4374b2c18d12347255ccb92001b9bcf80ef74c56072acbb98fe3  glfw-2.10.2-py2.py3-none-macosx_11_0_arm64.whl
5df643acbe059e576778b579052d53a51e08be072d80b1bc207dff1519dcc73a  hypothesis-6.168.4-cp314-cp314-macosx_11_0_arm64.whl
f631c04d2c48c52b84d0d0549c99ff3859c98df65b3101406327ecc7d53fbf12  iniconfig-2.3.0-py3-none-any.whl
0cbe94b69b819209a62cb27bdfa5dc2a8977d8de2f89dfd97ba4f53ed3af754e  kiwisolver-1.5.0-cp314-cp314-macosx_11_0_arm64.whl
ecdc9fae295da8ac793578a27020515e24d970513143efa227e696582aeb16e6  llvmlite-0.50.0-cp314-cp314-macosx_12_0_arm64.whl
9f7ebbcd14fe59494226453aed97c1070d83f8d24b6fc3a3bcf9a38092641c4a  markdown_it_py-4.2.0-py3-none-any.whl
ac104be2768ffdd8655db9e71b768cbb45f2b9aa7b450cf1595e8f65d3822319  matplotlib-3.11.1-cp314-cp314-macosx_11_0_arm64.whl
84008a41e51615a49fc9966191ff91509e3c40b939176e643fd50a5c2196b8f8  mdurl-0.1.2-py3-none-any.whl
3e353625f3fc2c3c8147ceddb2dcda3597bfd08c1d001ef0637a2080cd49cf77  mujoco-3.12.0-cp314-cp314-macosx_11_0_arm64.whl
96984dbe8154ffb86e653b1b60aaeed6985c710c5e75ad0279eb87943c66292d  nanobind-3.1.0-py3-none-any.whl
29326d74f107c347fd1009bd58e38d9f7c7c5b51e6de97bc93dbc325d9038b54  narwhals-2.26.0-py3-none-any.whl
bfc890c9ca517823dfae0444595ef50d883ade9d3e17759d9a7650e5d128d950  numba-0.68.0-cp314-cp314-macosx_12_0_arm64.whl
9e9413326d726c2545bfa65d2c0876871e8d8386e77f992c1d426e180bbd4323  numpy-2.5.2-cp314-cp314-macosx_14_0_arm64.whl
d7193f7c8e4e93f444fde0262bf90af30e16fa0ad0ad44cb553c87339b23cd1c  packaging-26.3-py3-none-any.whl
e158cb00350dc278f3b91551101aa7d12415a66ebf2c91d8d5ac14e56ddd3ad0  pillow-12.3.0-cp314-cp314-macosx_11_0_arm64.whl
dbb7fa18afce40d0a8e80d1bf162eceb3faa0ce5a77fe741ad09a74cf78f53f3  plotly-7.1.0-py3-none-any.whl
e920276dd6813095e9377c0bc5566d94c932c33b27a3e3945d8389c374dd4746  pluggy-1.6.0-py3-none-any.whl
5780d487ff6c6ed7b42298609680d87fe0036e529a9dc2e1105364bce9697f50  pyarrow-26.0.0-cp314-cp314-macosx_12_0_arm64.whl
2363c69b61c4a97c838da3b130dcd6468f4848992b21a82f2a63ec34377137d9  pygments-2.21.0-py3-none-any.whl
794a943daced39300879e4e47bd94525280685f42dbb5a998d336cfff151d74f  pyopengl-3.1.10-py3-none-any.whl
850ba148bd908d7e2411587e247a1e4f0327839c40e2e5e6d05a007ecc69911d  pyparsing-3.3.2-py3-none-any.whl
37a86b45efb9a47a61a36449063e8e18d0cab3161329fc099eb21783169c4f0c  pytest-9.1.1-py3-none-any.whl
a8b2bc7bffae282281c8140a97d3aa9c14da0b136dfe83f850eea9a5f7470427  python_dateutil-2.9.0.post0-py2.py3-none-any.whl
e658af3757f9e26a9d25dd2aff38335acd92bc9104f890a894b2c1ba28311b03  pytz-2026.5-py2.py3-none-any.whl
33bd4ef74232fb73fe9279a257718407f169c09b78a87ad3d296f548e27de0bb  rich-15.0.0-py3-none-any.whl
f7df7941d71314e60a481e02d5ebcb3f0185b8d799c70d03d8258f6c80f3d467  scipy-1.17.0-cp314-cp314-macosx_14_0_arm64.whl
4721f391ed90541fddacab5acf947aa0d3dc7d27b2e1e8eda2be8970586c3274  six-1.17.0-py2.py3-none-any.whl
a163dcaede0f1c021485e957a39245190e74249897e2ae4b2aa38595db237ee0  sortedcontainers-2.4.0-py2.py3-none-any.whl
cd8b60b5641b45c67bbf73c64c843235fc2d8a480c87389f52f5dbee893b86be  threadpoolctl-3.7.0-py3-none-any.whl
481caa481374e813c1b176ada14e97f1f67a4539ce9cfeb3f350d78d6370c2e8  typing_extensions-4.16.0-py3-none-any.whl
25ad4e16390cd314347dd8f1de67a2ac538ae658ed4ab9db16029c07c188e97f  zipp-4.1.0-py3-none-any.whl
```

### Linux aarch64 — `output/portability/wheelhouse-linux-aarch64/` (cp313, container 3.13.16)

```text
0f17b89f2a4eaaedc4f28c622998aa690564b3012a396a4ffad0821007fe03ba  absl_py-2.5.0-py3-none-any.whl
348ac1f5d4f1d66d3322420f01d42e43122f43616e0f194fc1c9f5d830c5b286  contourpy-1.3.3-cp313-cp313-manylinux_2_26_aarch64.manylinux_2_28_aarch64.whl
85cef7cff222d8644161529808465972e51340599459b8ac3ccbac5a854e0d30  cycler-0.12.1-py3-none-any.whl
364992ba1089a2b327391cfcb68fd0bd0ce9090cf293baef861a0ba6847abfee  duckdb-1.5.6-cp313-cp313-manylinux_2_26_aarch64.manylinux_2_28_aarch64.whl
b5df7341f54dbe1405a4450b2741207b4a8c279780402b45f87202b94dfc52b4  etils-1.14.0-py3-none-any.whl
1e874792a8212b44583ea02189d9e693906b2f78b261f372f95d6c563210ac1d  fonttools-4.63.0-cp313-cp313-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl
b57ddbafedfaef7018c1ecab32aa200a9d7ca26b77965f64e48b70061249d279  fsspec-2026.7.0-py3-none-any.whl
7604425ec95665cc6a25c3ed7d07e9660d394a34472ceffaec0843c844e7b649  glfw-2.10.2-py2.py3-none-manylinux_2_28_aarch64.whl
17fce531c5812f4a1eee8915e1d48e2ac9b7ee67a0715b35c346557b16b14749  hypothesis-6.168.4-cp313-cp313-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
f631c04d2c48c52b84d0d0549c99ff3859c98df65b3101406327ecc7d53fbf12  iniconfig-2.3.0-py3-none-any.whl
0c50b89ffd3e1a911c69a1dd3de7173c0cd10b130f56222e57898683841e4f96  kiwisolver-1.5.0-cp313-cp313-manylinux_2_24_aarch64.manylinux_2_28_aarch64.whl
7dde9836d144c446a303b57b2dd906c35308411eb07f1279c1db581d3d774048  llvmlite-0.50.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
9f7ebbcd14fe59494226453aed97c1070d83f8d24b6fc3a3bcf9a38092641c4a  markdown_it_py-4.2.0-py3-none-any.whl
7389b77ed2ab0552f46d9a90b81b7b8e6dfcdc42adc36c37a0865799843e0e3e  matplotlib-3.11.1-cp313-cp313-manylinux_2_26_aarch64.manylinux_2_28_aarch64.whl
84008a41e51615a49fc9966191ff91509e3c40b939176e643fd50a5c2196b8f8  mdurl-0.1.2-py3-none-any.whl
267c3fc36b5a0536073557760e36c464da2ad9a744b047893db73fe273931ad1  mujoco-3.12.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
96984dbe8154ffb86e653b1b60aaeed6985c710c5e75ad0279eb87943c66292d  nanobind-3.1.0-py3-none-any.whl
29326d74f107c347fd1009bd58e38d9f7c7c5b51e6de97bc93dbc325d9038b54  narwhals-2.26.0-py3-none-any.whl
df6f881c5695f472873d0979bab54261959b3174b6c98a71f6f8a43c3e088985  numba-0.68.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
0aadf13b60048d501e05fa699efaf7734e2494f3498a4c2a5521d822640324f3  numpy-2.5.2-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
d7193f7c8e4e93f444fde0262bf90af30e16fa0ad0ad44cb553c87339b23cd1c  packaging-26.3-py3-none-any.whl
f7401aebd7f581d7f83a439d87d474999317ee099218e5ad25d125290990ba65  pillow-12.3.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
dbb7fa18afce40d0a8e80d1bf162eceb3faa0ce5a77fe741ad09a74cf78f53f3  plotly-7.1.0-py3-none-any.whl
e920276dd6813095e9377c0bc5566d94c932c33b27a3e3945d8389c374dd4746  pluggy-1.6.0-py3-none-any.whl
e3b190ba1d3d22a5a8758597f797111b77d433473744352a184a5ee0a42d672e  pyarrow-26.0.0-cp313-cp313-manylinux_2_28_aarch64.whl
2363c69b61c4a97c838da3b130dcd6468f4848992b21a82f2a63ec34377137d9  pygments-2.21.0-py3-none-any.whl
794a943daced39300879e4e47bd94525280685f42dbb5a998d336cfff151d74f  pyopengl-3.1.10-py3-none-any.whl
850ba148bd908d7e2411587e247a1e4f0327839c40e2e5e6d05a007ecc69911d  pyparsing-3.3.2-py3-none-any.whl
37a86b45efb9a47a61a36449063e8e18d0cab3161329fc099eb21783169c4f0c  pytest-9.1.1-py3-none-any.whl
a8b2bc7bffae282281c8140a97d3aa9c14da0b136dfe83f850eea9a5f7470427  python_dateutil-2.9.0.post0-py2.py3-none-any.whl
e658af3757f9e26a9d25dd2aff38335acd92bc9104f890a894b2c1ba28311b03  pytz-2026.5-py2.py3-none-any.whl
33bd4ef74232fb73fe9279a257718407f169c09b78a87ad3d296f548e27de0bb  rich-15.0.0-py3-none-any.whl
fe508b5690e9eaaa9467fc047f833af58f1152ae51a0d0aed67aa5801f4dd7d6  scipy-1.17.0-cp313-cp313-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl
4721f391ed90541fddacab5acf947aa0d3dc7d27b2e1e8eda2be8970586c3274  six-1.17.0-py2.py3-none-any.whl
a163dcaede0f1c021485e957a39245190e74249897e2ae4b2aa38595db237ee0  sortedcontainers-2.4.0-py2.py3-none-any.whl
cd8b60b5641b45c67bbf73c64c843235fc2d8a480c87389f52f5dbee893b86be  threadpoolctl-3.7.0-py3-none-any.whl
481caa481374e813c1b176ada14e97f1f67a4539ce9cfeb3f350d78d6370c2e8  typing_extensions-4.16.0-py3-none-any.whl
25ad4e16390cd314347dd8f1de67a2ac538ae658ed4ab9db16029c07c188e97f  zipp-4.1.0-py3-none-any.whl
```

### Linux x86_64 — `output/portability/wheelhouse-linux-x86_64/` (cp313 cross-probe)

```text
0f17b89f2a4eaaedc4f28c622998aa690564b3012a396a4ffad0821007fe03ba  absl_py-2.5.0-py3-none-any.whl
4debd64f124ca62069f313a9cb86656ff087786016d76927ae2cf37846b006c9  contourpy-1.3.3-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
85cef7cff222d8644161529808465972e51340599459b8ac3ccbac5a854e0d30  cycler-0.12.1-py3-none-any.whl
644f54ce99b3b61844bc9a3fe80e0aecb1ea4084b1fffc4396d1569db6111679  duckdb-1.5.6-cp313-cp313-manylinux_2_26_x86_64.manylinux_2_28_x86_64.whl
b5df7341f54dbe1405a4450b2741207b4a8c279780402b45f87202b94dfc52b4  etils-1.14.0-py3-none-any.whl
22135da48a348785c5e2d5d2d9d6bec5ed44adacbaeb9db12d9493bf6c6bfa68  fonttools-4.63.0-cp313-cp313-manylinux2014_x86_64.manylinux_2_17_x86_64.whl
b57ddbafedfaef7018c1ecab32aa200a9d7ca26b77965f64e48b70061249d279  fsspec-2026.7.0-py3-none-any.whl
b860de3ca0686182483f98f3ddd12e660acf25b3e0d521450ec9a3f999f72a65  glfw-2.10.2-py2.py3-none-manylinux_2_28_x86_64.whl
50d43c132ece8712c50ba20a2103b27693bdb098d0b3d2b451c3bf6837476655  hypothesis-6.168.4-cp313-cp313-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
f631c04d2c48c52b84d0d0549c99ff3859c98df65b3101406327ecc7d53fbf12  iniconfig-2.3.0-py3-none-any.whl
332b4f0145c30b5f5ad9374881133e5aa64320428a57c2c2b61e9d891a51c2f3  kiwisolver-1.5.0-cp313-cp313-manylinux2014_x86_64.manylinux_2_17_x86_64.whl
425845f415a06dc50db08db033c6b568e0d85c4937e932c605a4d49e1514b2da  llvmlite-0.50.0-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
9f7ebbcd14fe59494226453aed97c1070d83f8d24b6fc3a3bcf9a38092641c4a  markdown_it_py-4.2.0-py3-none-any.whl
b0a19dcf73406d3746d25a5ed42d713604c9a3e024d129b102852b0d941cb9f3  matplotlib-3.11.1-cp313-cp313-manylinux2014_x86_64.manylinux_2_17_x86_64.whl
84008a41e51615a49fc9966191ff91509e3c40b939176e643fd50a5c2196b8f8  mdurl-0.1.2-py3-none-any.whl
ce95e5a2dd1afcda3ae276b593ac53348d488d1a99c9b4205ef182164b90ba04  mujoco-3.12.0-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
96984dbe8154ffb86e653b1b60aaeed6985c710c5e75ad0279eb87943c66292d  nanobind-3.1.0-py3-none-any.whl
29326d74f107c347fd1009bd58e38d9f7c7c5b51e6de97bc93dbc325d9038b54  narwhals-2.26.0-py3-none-any.whl
be647fbc60c18c0323b34479f80173879654894eec58ad061f4b1901e294d854  numba-0.68.0-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
29b86ff8a6cc556b47ec6b64b194815cc80e6bf5eedcc6cddfd65318cb0b4eee  numpy-2.5.2-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
d7193f7c8e4e93f444fde0262bf90af30e16fa0ad0ad44cb553c87339b23cd1c  packaging-26.3-py3-none-any.whl
0847a763afefb695bc912d7c131e7e0632d4edc1d8698f58ddabec8e46b8b6d3  pillow-12.3.0-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
dbb7fa18afce40d0a8e80d1bf162eceb3faa0ce5a77fe741ad09a74cf78f53f3  plotly-7.1.0-py3-none-any.whl
e920276dd6813095e9377c0bc5566d94c932c33b27a3e3945d8389c374dd4746  pluggy-1.6.0-py3-none-any.whl
240bd18a7487f8767616a948a69dd4e740a8bc36a1c9da49e4dc9a32c5c2faed  pyarrow-26.0.0-cp313-cp313-manylinux_2_28_x86_64.whl
2363c69b61c4a97c838da3b130dcd6468f4848992b21a82f2a63ec34377137d9  pygments-2.21.0-py3-none-any.whl
794a943daced39300879e4e47bd94525280685f42dbb5a998d336cfff151d74f  pyopengl-3.1.10-py3-none-any.whl
850ba148bd908d7e2411587e247a1e4f0327839c40e2e5e6d05a007ecc69911d  pyparsing-3.3.2-py3-none-any.whl
37a86b45efb9a47a61a36449063e8e18d0cab3161329fc099eb21783169c4f0c  pytest-9.1.1-py3-none-any.whl
a8b2bc7bffae282281c8140a97d3aa9c14da0b136dfe83f850eea9a5f7470427  python_dateutil-2.9.0.post0-py2.py3-none-any.whl
e658af3757f9e26a9d25dd2aff38335acd92bc9104f890a894b2c1ba28311b03  pytz-2026.5-py2.py3-none-any.whl
33bd4ef74232fb73fe9279a257718407f169c09b78a87ad3d296f548e27de0bb  rich-15.0.0-py3-none-any.whl
6680f2dfd4f6182e7d6db161344537da644d1cf85cf293f015c60a17ecf08752  scipy-1.17.0-cp313-cp313-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
4721f391ed90541fddacab5acf947aa0d3dc7d27b2e1e8eda2be8970586c3274  six-1.17.0-py2.py3-none-any.whl
a163dcaede0f1c021485e957a39245190e74249897e2ae4b2aa38595db237ee0  sortedcontainers-2.4.0-py2.py3-none-any.whl
cd8b60b5641b45c67bbf73c64c843235fc2d8a480c87389f52f5dbee893b86be  threadpoolctl-3.7.0-py3-none-any.whl
481caa481374e813c1b176ada14e97f1f67a4539ce9cfeb3f350d78d6370c2e8  typing_extensions-4.16.0-py3-none-any.whl
25ad4e16390cd314347dd8f1de67a2ac538ae658ed4ab9db16029c07c188e97f  zipp-4.1.0-py3-none-any.whl
```

## 5. Gaps and follow-ups

- No package is missing from any of the three qualified targets. The
  only rejected candidate observed was contourpy at the *conservative*
  `manylinux_2_17` probe floor — resolved correctly at the documented
  intersection floor (§3).
- `colorama` has no wheel on any qualified target and needs none — it
  is Windows-only via environment marker.
- `bike_native` portability is a separate per-platform build problem;
  this matrix does not qualify it.
