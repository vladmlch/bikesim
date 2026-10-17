# Offline bundle content contract

This is the portability T24 producer and content checker. The supplied snapshot
contained `tools/install_offline.py` and `tools/check_offline_sources.py`, but no
bundle maker, packaging schema, target catalog or install shell wrapper. The new
files define an explicit contract; they are not recovered versions of those
missing files. No bundle, installer, native build or test was run while preparing
this change.

## What the manifest establishes

Each ZIP contains `project/`, `wheelhouse/`, `requirements.txt` and
`bundle-manifest.json`. The root manifest uses
[`offline-bundle.schema.json`](offline-bundle.schema.json), version 1. Paths are
relative to the extracted root. It records:

- The **target** OS, architecture, CPython version and OS baseline separately
  from the machine/interpreter that **produced** the inventory.
- The project name, static version and `requires-python` declaration read from `project/pyproject.toml`, and a
  digest over all delivered project source files. No source commit is invented
  when a source export lacks git history.
- An exact inventory of regular files, byte sizes and SHA-256 digests. The
  manifest itself is excluded from its inventory to avoid a recursive digest.
- Supplied wheel identities, filename tags, byte hashes, and agreement between
  the wheel filename and its `METADATA`/`WHEEL` records.
- Runtime requirements from `project.dependencies` and build requirements from
  `build-system.requires` plus the `native` dependency group. These are source
  declarations, not a newly resolved lock or verified dependency closure.
- The distinction between full and slim input sets and any explicitly pinned
  packages expected in the receiving interpreter.

`qualification.installation`, `native_build`, `gui` and `binary_compatibility`
are always `not_run`. `requirements.closure` and
`requirements.wheel_dependency_metadata` are always `not_verified`. A valid
inventory does not change those statuses. Successful content checks print
`content_valid: true`; that field makes no runtime acceptance claim.

## Source authority

The source tree remains authoritative. Supply exactly one pure `bike_sim` wheel
whose version, Python files **and package resources** exactly match
`project/src/bike_sim`. The wheel exists to preserve the current installer
layout; it cannot override source content. A stale wheel, missing resource,
duplicate distribution or embedded native/build artifact in the project wheel
is rejected. Its installable payload is confined to `bike_sim/` and the expected
`.dist-info/` directory: `.data/purelib` or `.data/platlib` overrides and extra
top-level modules are rejected. Console, GUI and plugin entry points must agree
with the source tables, including letter case. `Requires-Python` must agree with
the source bound declarations.

For interpreter eligibility, T24 supports `>=major.minor` and `<major.minor`
bounds, optionally comma-separated (and an optional trailing `.0`). A source
floor of `>=3.14` cannot produce a CPython 3.13 bundle. Other forms, including
patch-specific restrictions, fail explicitly instead of guessing the target
patch version. General specifier resolution remains an installer concern.

Semantic equivalence of the project's `Requires-Dist` wheel metadata to source
dependencies is not checked by this stdlib-only producer. The source requirements
are recorded exactly, and `wheel_dependency_metadata: not_verified` makes this
remaining T25 check explicit. A same-version wheel with changed dependency
metadata must not be considered qualified merely because its code matches.

The separate legacy `compare_sources` interface continues comparing
Python files only, with its original three report keys.

Native extensions must be rebuilt for the receiving target. Source copying
excludes virtual environments, wheelhouses, native build directories, compiled
libraries/objects, interpreter caches, build/package caches, generated outputs,
and machine-local hidden directories. It retains source code, tests, dependency
declarations, documentation, examples and package resources. Source dot-configs
such as `.clangd`, `.clang-tidy`, `.clang-format`, `.editorconfig`,
`.gitattributes`, `.github`, `.gitignore` and `.python-version` remain eligible.
Symlinks in eligible sources
are rejected rather than dereferenced. The output must be outside the source
tree, and an existing archive is never replaced.

The source digest is SHA-256 of UTF-8 lines, sorted by source-relative path:
`relative/path=lowercase_file_sha256\n`. It covers all delivered files under
`project/`, without the `project/` prefix. ZIP entry ordering, timestamps and
permissions are normalized, so identical input bytes and producer identity
produce identical archives on the same compression implementation.

## Profiles and target limits

| Profile | Supplied wheels | Preinstalled Python packages | Meaning |
|---|---|---|---|
| `full` | Producer intends to supply the runtime/build dependency set and project wheel. | Must be empty. | T24 records the files and required dependency declarations; T25 must prove the complete closure installs offline. |
| `slim` | Project wheel and the explicitly supplied subset of dependencies. | At least one explicit `name==version` pin; duplicates or a package also supplied as a wheel are rejected. | T25 must prove that supplied wheels plus the receiving environment satisfy the complete closure. |

Full does not bundle an OS, Python interpreter, compiler, SDK, CMake or a virtual
environment. The current legacy installer still expects most scientific
packages to be present even when a local wheel is supplied; T24 does not change
that installation policy.

The Linux release target is **Ubuntu 24.04 x86_64, ordinary CPython 3.13**. An
aarch64 producer can inventory x86_64 wheel files but cannot qualify x86_64 by
doing so. Linux aarch64 and musllinux are not release targets of this contract.
macOS arm64 and x86_64 manifest targets support declarations for CPython 3.13
and 3.14, preserving the existing macOS interpreter surface. These targets remain
explicitly unqualified, and their deployment minimum is recorded as unspecified.

Wheel checks reject known OS, architecture, Python and ABI mismatches. For
example, `cp313t` is not accepted for ordinary CPython 3.13, while compatible
older CPython `abi3` tags are eligible. This is a filename/metadata exclusion
check: it does not inspect ELF/Mach-O dependencies, glibc symbols, CPU features,
macOS deployment requirements or system library availability. Those checks stay
`not_run`; accepted tags must never be presented as binary compatibility proof.
[`offline-targets.json`](offline-targets.json) lists the producer arguments and
explicit prerequisites. It is an example catalog, not a release manifest.

## Producer interface

The producer needs an existing Python 3.11+ interpreter for standard-library
`tomllib`. Target selection is independent of that producer interpreter. The wrapper uses `uv` without project
synchronization, network access or Python downloads. It does not build a wheel,
install dependencies, import the project, run CMake or invoke the installer.

Commands below are for a later, authorized packaging session. They were not run
as part of this change. Supply real locally prepared, target-specific wheels;
the examples do not imply the required wheels are present in this snapshot.

```bash
bash tools/make_offline_bundle.sh \
  --source . \
  --wheelhouse /path/to/local/ubuntu-24.04-x86_64-wheels \
  --target ubuntu-24.04-x86_64-cp313 \
  --profile full \
  --output ../bike-sim-offline-ubuntu-24.04-x86_64-full.zip
```

For slim, choose `--profile slim` and repeat `--preinstalled NAME==VERSION` for
each package intentionally omitted from the wheelhouse. Use the actual pinned
versions available to the receiver; the producer does not guess them. For
macOS, use `--target macos-arm64-cp313`, `macos-x86_64-cp313`,
`macos-arm64-cp314` or `macos-x86_64-cp314` and the matching local wheels.

The importable interfaces for the next delivery task are:

```python
create_bundle(source, wheelhouse, output, *, target, profile, preinstalled=())
validate_bundle(extracted_bundle_root)
```

Both return the manifest dictionary. Malformed/inconsistent content raises
`BundleError`, a `ValueError` subclass. File-system errors can also propagate.
Validation reads bytes only and rejects unexpected fields, unsafe or duplicate
paths, case/Unicode path collisions, symlinks, unknown files, changed hashes,
stale source wheels and a requirements file that differs from the exact local
wheel pins/hashes. File hashes provide consistency, not authenticity or a
publisher signature. No archive member or wheel is executed.

After extracting a bundle, its content can be checked with:

```bash
uv run --offline --no-project --no-python-downloads \
  python -B tools/check_offline_sources.py --bundle /path/to/extracted-bundle
```

The legacy command remains available:

```bash
uv run --offline --no-project --no-python-downloads \
  python -B tools/check_offline_sources.py \
  --source /path/to/project/src \
  --wheel /path/to/bike_sim-0.3.0-py3-none-any.whl
```

## Installer boundary and remaining acceptance

The existing installer now performs an additive content preflight when
`bundle-manifest.json` is present. It rejects full and non-Linux-target manifests
with a T25 diagnostic, and checks declared preinstalled pins before modifying an
environment. The no-manifest legacy path retains its old behavior.

The standalone checker and installer disable bytecode generation before loading
the sibling validator, so their own imports do not change a pristine bundle's
inventory. For `python -m tools.check_offline_sources`,
`python -m tools.install_offline` or other imports from inside an extracted
bundle, use `python -B` (or `PYTHONDONTWRITEBYTECODE=1`) from process startup:
module loaders can cache the entry module before its body executes.

The installer continues to require Linux x86_64 CPython 3.13. T24's guard checks
the declared target; it does not establish that the running Linux distribution,
libc, native provider or GUI stack meets that target. Source-first installation,
full/slim dependency closure, native building and all macOS installation paths
remain T25 work. The current snapshot also lacks the original portability T23
brief and completion evidence.

Inventory validation is a **pre-install** operation against the untouched
extracted directory. An environment or an install report later created inside
that directory changes its inventory; keep an untouched copy for repeat content
checks. Installer lifecycle acceptance must define its own post-install report
contract in T25.

`tests/test_offline_bundle.py` contains isolated regression sources for this
contract. Tests, producer execution, real full/slim archives and installation
acceptance are all pending execution on an authorized host. Do not mark the
original portability T24 verified until the required gates actually pass.
