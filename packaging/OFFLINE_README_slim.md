# Slim offline source bundles

Slim uses the same source, tool lock, target validation and payload checks
as [full mode](OFFLINE_README_full.md). It omits explicitly named scientific
wheels and records the exact external providers required at the destination.
It does not loosen the numerical profile or qualify a system installation.

T24 was implemented and reviewed statically on 2026-10-17. No assembler,
download, build, test or installer ran. T25 owns installation qualification.
The supplied Repomix omitted the production lock, binary assets and legacy
offline scripts; restore those real inputs as described in the full README.

## Assemble and inspect

Prepare the exact platform tool lock and selected local wheel directory
with `tools/prepare_offline_tools.py`, then run on the intended native host:

```sh
uv run --frozen --group native --no-sync python tools/offline_bundle.py \
  --target host --mode slim --native \
  --tool-lock packaging/offline-tools.json \
  --tool-wheelhouse /absolute/selected-tool-wheels \
  --output output/offline-bundles
```

The Python minor defaults to the authoring interpreter. An explicit
`--python 3.13` requires CPython 3.13 for native assembly.

Default omission candidates are numpy, scipy, numba, llvmlite and pyarrow.
The assembler first downloads and checks the entire exact closure, including
all target markers, extras and dependency edges. It then removes only the
selected wheels. Dependencies of an omitted provider remain bundled unless
also explicitly selected. Project, native-development and build/tool
dependencies cannot be omitted.

Replace the omission list with repeated options:

```sh
uv run --frozen --group native --no-sync python tools/offline_bundle.py \
  --target host --mode slim --native \
  --tool-wheelhouse /absolute/selected-tool-wheels \
  --slim-external numpy --slim-external scipy \
  --output output/offline-bundles
```

Each name must occur in the active frozen closure. Versions come from the
current lock and inspected wheels. Native retains NumPy 2.5.2, SciPy 1.17.0
and MuJoCo 3.12.0. SciPy 1.17.1, for example, cannot satisfy a SciPy 1.17.0
external-provider requirement.

Example output and read-only payload check:

```text
bike-sim-offline-py313-linux-x86_64-slim-native-source.tar.gz
```

```sh
uv run --frozen --group native --no-sync python tools/check_offline_sources.py \
  --bundle /absolute/extracted-slim-bundle --require-host
```

## External-provider contract

| `slim_external` field | Meaning |
| --- | --- |
| `distribution` | Exact normalized package name |
| `version` | Exact source-lock version |
| `qualification_profile` | `native-exact-provider-pending-T25`, or `python-exact-import-pending-T25` for the Python-only route |

`dependency-closure.json` retains original omitted wheel metadata, hashes
and edges as assembly evidence. The source checker compares that evidence
with the frozen export/lock and full closure. It cannot rehash absent wheel
bytes or prove which provider the destination interpreter imports.

T25 must verify every external version and actual import path, Python/ABI,
NumPy/MuJoCo provider and native numerical profile. Repeat checks after
environment creation because import precedence can change. A package
shadowed by a newly installed wheel is not reuse of the previously checked
system provider. A mismatch requires full mode or an explicitly corrected
provider environment; do not rewrite the system scientific stack.

Fresh offline qualification also requires empty caches, blocked network,
source/editable installation, local compilation, doctor/CTest/import and
recording checks, then relocation and rebuilding. Complete source metadata
is not evidence for these unrun gates.

## Legacy Python-only mode

The no-argument shell wrapper requests Linux x64 CPython 3.13 full plus
slim, with reconstructed slim name
`bike-sim-offline-py313-linux-x64-slim.tar.gz`. It declares
`native_source_build: false`; native recording and an extension are not
provided by this Python-only route.

The full README defines tool preparation, exact schema, source/asset
inclusion, hash conventions, host/target separation and the standalone
legacy Python wheel/source comparison.
