# Full offline source bundles

Schema 1 packages the actual source checkout and a frozen, target-matched
wheel closure. Native bundles contain source for editable installation and
local compilation. `native_source_build: true` describes the included
inputs. Installation, native execution and desktop acceptance remain
unqualified until their separate gates pass. T25 owns the installer and
fresh network-blocked installation qualification.

## Status and original inputs

This T24 implementation was written and reviewed statically on 2026-10-17.
No assembler, download, build, test, installer or viewer was run in that
session, as requested by the user.

The supplied Repomix snapshot omitted `uv.lock`, executable metadata,
required benchmark binaries, `packaging/` and the legacy offline scripts.
The wrapper/checker here reconstruct their documented compatibility
contract; exact parity with the absent legacy implementation is unverified.
Restore the real checkout's committed lock and original binary assets. Do
not create a new scientific lock to work around missing snapshot inputs.

## Contents and prerequisites

| Item | Contract |
| --- | --- |
| `project/` | Current tracked source plus approved assets; original file modes and SHA256 inventory |
| `wheelhouse/` | Every active frozen dependency and exact build/tool wheel, one per normalized distribution |
| `requirements.txt` | Unmodified hash-pinned export from the source `uv.lock` |
| `offline-tools.json` | Reviewed exact pins for uv, pip, setuptools, wheel and the full backend/tool closure |
| `manifest.json` | Schema 1 source identity, target/Python/tags, capabilities, hashes and prerequisites |
| `dependency-closure.json` | Checked wheel metadata, marker environment, extras and evaluated dependency edges |
| `assembly-report.json` | Commands/statuses, download evidence, source modes and `installation_qualification: pending-T25` |

CPython of the target minor must already exist. Native use additionally
requires CMake >=3.27, Ninja or Make, strict ISO C++23 compiler/stdlib, and
the OS-specific numerical/linker prerequisites. macOS uses its SDK and
Accelerate; Linux uses the exact NumPy scientific provider. Analysis,
sanitizer and coverage tools remain explicit system prerequisites; see
[`docs/TESTING.md`](../docs/TESTING.md). Viewers require OS graphics libraries
and a display. Neither source assembly nor a Python wheel qualifies them.

Native keeps MuJoCo 3.12.0, NumPy 2.5.2 and SciPy 1.17.0. Other exact
versions/hashes come from the current restored lock, not historical prose.
The documented Linux scientific-wheel floor is glibc 2.28. Host assembly
conservatively records the actual host glibc version and accepted tags;
it does not promise a locally built extension will run on an older host.
macOS deployment requirements are represented by accepted wheel tags.

## Prepare the supplemental tool lock

The project lock does not pin all PEP 517 tools or uv itself. Default
supplemental paths are `packaging/offline-tools.json` and the adjacent
`packaging/offline-tool-wheels/` directory.

Obtain an explicitly selected, previously qualified set of platform wheels
for uv, pip, setuptools, wheel and their actual transitive/backend closure
through your established online provisioning process. Do not select newer
versions implicitly. A tool dependency also present in `uv.lock`, such as
packaging, must use the same exact version and wheel hash.

Historical baseline lanes used macOS uv 0.12.9 and Linux uv 0.13.1; see
[`2026-10-14-linux-macos-baseline.md`](../docs/superpowers/audits/2026-10-14-linux-macos-baseline.md).
These are separate evidence, not one universal pin. Select the intended
lane's actual qualified tools. The assembler checks its invoking uv version
against the selected target uv wheel.

Prepare the lock from those local wheels:

```sh
uv run --frozen --group native --no-sync python tools/prepare_offline_tools.py \
  --target host --native \
  --wheelhouse /absolute/selected-tool-wheels \
  --output packaging/offline-tools.json
```

The preparer reads local bytes, validates tags, metadata, dependency edges
and the uv license, and writes a new lock. It does not resolve or upgrade
packages. It refuses to overwrite an existing lock; review a new output
when changing the profile. Omit `--python` to use the actual interpreter
minor, or specify `--python 3.13` while running under CPython 3.13.

Exact supplemental schema:

```text
schema_version: 1
target: {system, machine, python_minor, implementation,
         accepted_wheel_tags, libc_floor}
uv_version: EXACT_SELECTED_VERSION
wheels:
  - filename: EXACT_WHEEL_FILENAME.whl
    distribution: canonical-distribution-name
    version: EXACT_SELECTED_VERSION
    sha256: LOWERCASE_SHA256
    tags: [all expanded filename tags]
```

Keep the selected wheel directory available; the supplemental lock does
not authorize an unpinned tool download. The uv license remains inside its
hash-verified wheel, with license member hashes recorded in evidence.

## Native host assembly

Use the real Git checkout and an already provisioned frozen environment:

```sh
uv run --frozen --group native --no-sync python tools/offline_bundle.py \
  --target host --mode full --native \
  --tool-lock packaging/offline-tools.json \
  --tool-wheelhouse /absolute/selected-tool-wheels \
  --output output/offline-bundles
```

Native target OS, architecture and Python minor must equal the authoring
host. An actual aarch64 Linux `host` bundle does not qualify the x86_64
release target. Example native output:

```text
bike-sim-offline-py313-linux-x86_64-full-native-source.tar.gz
```

Assembly checks `uv lock --check --offline` before a frozen export. Dirty
source is preserved byte for byte, but changed dependency declarations
must still match the lock. Every application wheel is selected directly
from lock HTTPS URLs and verified against lock and export hashes. WHEEL,
METADATA, Requires-Python, target tags, extras and dependency edges are
checked. Source-only dependencies fail. Native assembly does not build a
project wheel or an extension.

Existing output is not overwritten. Failed staging retains an
`assembly-failure.json` under the output directory. Successful archives
exclude temporary build/cache directories.

## Source inclusion and binary inputs

The collector copies current tracked working-tree bytes and file modes.
Ordinary untracked files are excluded. Repeated `--asset relative/path`
options explicitly approve required assets within the source root;
committing reviewed implementation files is the normal route. Escaping
symlinks fail. Safe internal links become regular source files after both
the link path and resolved path pass checks.

Excluded content includes credentials/configuration directories, `.git`,
virtualenvs, caches, `native/build`, output/dist and generated native
binaries. Build markers detect arbitrarily named CMake/meson build trees
and virtualenvs. Real tests and binary fixtures remain eligible; neither
all `artifacts` directories nor all binary data extensions are discarded.

Native development requires the independent shared
`tools/proto_native_bench/artifacts/model.mjb` and the fixture selected by
`artifacts/current.json`. The selected fixture needs `manifest.json`,
`model.mjb`, `state.bin` and `state.npz`, with the recorded file checksums.
These required deterministic inputs are explicitly included even when
ignored by Git. Missing original inputs fail by name; they are never
regenerated. Literal CMake source/module/tool inputs are also checked.
Dynamic build inputs and compiler features remain T25 build gates.

## Inspect extracted payload

From an already provisioned source environment:

```sh
uv run --frozen --group native --no-sync python tools/check_offline_sources.py \
  --bundle /absolute/extracted-bundle --require-host
```

Omit `--require-host` to inspect another platform's bundle without running
its code. The checker validates hashes, tags, source/wheel inventory,
required native inputs, executable modes and recorded dependency closure.
`native_source_sha256` uses the existing `native_source_fingerprint()`
convention. `python_source_sha256` hashes sorted `src/` path/SHA256 rows
(`path`, NUL, hex hash, newline); it differs from the runtime recording
fingerprint of raw source bytes.

`payload-consistent` reports assembly integrity. T25 still needs fresh
Linux/macOS installation with empty caches and blocked network, local
compilation, doctor/CTest, selected native imports, opt-in recording and
relocation followed by rebuilding. Those results were not measured here.

## Python-only compatibility

`bash tools/make_offline_bundle.sh` with no arguments requests Linux x64
CPython 3.13 full and slim Python-only bundles, using reconstructed names:

```text
bike-sim-offline-py313-linux-x64.tar.gz
bike-sim-offline-py313-linux-x64-slim.tar.gz
```

Prepare a Linux-target supplemental lock/wheel directory for this route.
When authoring on another host, pip/backend wheels must also support that
host's tags, Python and dependency markers. Target uv is bundled and never
executed on the author; the existing host uv is version-checked. Ambiguous
platform-release or Python-patch-dependent cross-target markers require a
real target host.

This route builds the existing `py3-none-any` project wheel from a separate
source copy and checks its Python bytes. It declares
`native_source_build: false` and `development: false`; ordinary native
recording remains unavailable. Its standalone comparison is:

```sh
uv run --frozen --group native --no-sync python tools/check_offline_sources.py \
  --source /absolute/project --wheel /absolute/bike_sim-VERSION-py3-none-any.whl
```

That mode compares Python wheel/source only. The manifest bundle mode also
checks native inputs, frozen dependencies and assets.
