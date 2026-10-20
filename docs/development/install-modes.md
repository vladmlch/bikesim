# Python and source-native installation modes

Native execution uses the Python package imported from a real source tree,
that tree's selected editable uv environment, and an explicitly built native
artifact. Importing an application module never bootstraps an environment,
configures CMake, compiles an extension, or substitutes the Python backend for
a requested native backend.

This document describes the T23 implementation contract. The changes were
reviewed statically in the 2026-10-17 source-snapshot session; the user prohibited
compilation and execution. The commands below are the subsequent verification
procedure, **not evidence that installation or platform qualification passed**.

## Choose an installation mode

| Mode | Contents and capabilities | Requirements and limits |
| --- | --- | --- |
| Reference Python package | Existing setuptools package and console scripts; Python reference execution and Python-side tools | A `py3-none-any` project wheel contains no `bike_native` extension. Its Python dependencies can include their own platform binaries. Use explicitly supported Python routes. |
| Source/editable native development | Python source, `native/CMakeLists.txt`, native sources/tools, and a native artifact built on this host | The frozen native environment, supported C++23 compiler, CMake 3.27 or newer, qualified MuJoCo/NumPy provider, and explicit configure/build. This is the supported native installation route. |
| Native-capable offline source bundle | A complete source tree and target-matched dependency/tool wheels; local editable installation and native compilation | T24/T25 own manifests, full/slim assembly, exact external dependency checks, and network-blocked installation qualification. A Python-only legacy bundle does not acquire native capability by its filename. |

Ordinary physical `bike-ride --record` requires the native backend. Python
research recording is a separate supported route; see [recording modes](../RECORDING.md).
Inspecting saved data does not prove that the current environment can perform
checked physical replay. The replay identity contract remains unchanged.

The source root is derived from the imported `bike_sim.native.artifact` module's
own layout. Both `pyproject.toml` and `native/CMakeLists.txt` must be there.
The loader never searches the current directory or arbitrary ancestors for
another checkout. A wheel-only environment reports its limitation even if a
different checkout happens to be the current working directory. Setting
`BIKE_NATIVE_BUILD_PATH` alone does not make that wheel a native installation:
use the real source tree and its editable environment.

## Source/editable setup

Use the real checkout or an extracted source bundle with all required files.
A Repomix text snapshot alone is not a runnable distribution: it does not prove
that ignored files, binary assets, executable bits or dependency wheels exist.
Python 3.13 or newer is required by `pyproject.toml`; platform release
qualification uses its explicitly selected interpreter and dependency profile.

From the source root, sync the committed environment and build explicitly:

```sh
uv sync --frozen --group native
uv run --frozen --group native --no-sync python tools/dev.py configure --build native/build/release --config Release
uv run --frozen --group native --no-sync python tools/dev.py build --build native/build/release
uv run --frozen --group native --no-sync python tools/dev.py doctor --build native/build/release --numerics
```

`tools/dev.py bootstrap` is the explicit convenience wrapper for
`uv sync --locked --group native --group dev`. It requires the committed lock
to remain current; it does not relock or upgrade dependencies. `--python` selects the interpreter
for that explicit synchronization. Its offline option accepts a local
wheelhouse, but the existence of that option is not T25 offline certification.

The native group pins MuJoCo **3.12.0** and NumPy **2.5.2**. The selected MuJoCo
image must also expose the project's private ABI symbols. macOS retains
Accelerate; Linux uses the qualified provider actually loaded by the selected
NumPy environment. An arbitrary system OpenBLAS or a newer MuJoCo package is
not an automatic replacement. Native compilation checks its own compiler,
library, ABI and numerical requirements.

Separate configurations and compilers need separate directories. For example,
replace the Release build above with `--build native/build/debug --config Debug`
when configuring Debug, and select that same directory for subsequent build,
doctor and tests. Existing macOS SDK selection and `mjpython` GUI launch rules
remain in force; installation does not qualify a live desktop session.

## Keep one selected environment

Without an override, every development wrapper exports an absolute
`UV_PROJECT_ENVIRONMENT` pointing at the source tree's `.venv`. An activated
unrelated venv does not select the project environment. Configure, build, test
and doctor must themselves run through that selected project interpreter.
Bootstrap is the explicit step that may create it.

An installer or developer may choose a different environment. Export its
absolute path **before the first uv command** and retain it for every step:

```sh
export UV_PROJECT_ENVIRONMENT="/absolute/path/to/bike-venv"
uv sync --frozen --group native
uv run --frozen --group native --no-sync python tools/dev.py configure --build native/build/release --config Release
uv run --frozen --group native --no-sync python tools/dev.py build --build native/build/release
uv run --frozen --group native --no-sync python tools/dev.py doctor --build native/build/release --numerics
```

A relative `UV_PROJECT_ENVIRONMENT` is rejected by the driver. Offline,
sanitizer and tool override variables are passed through unchanged. The
selected interpreter is passed as `Python_EXECUTABLE` at configure time; later
commands compare it with CMake's cached selection. Two venvs that symlink to
the same base CPython are distinct environments, even if their executable
files ultimately resolve to one binary.

`tools/dev.py` finds its project root from its own source path. It passes that
root as every subprocess's working directory. Relative `--build` arguments
are relative to that root, not the caller's directory. The outer `uv run`
should still be invoked from the intended source project so uv selects the
same project before the driver starts.

## Select application and test artifacts

The application defaults to the source tree's `native/build/release`.
An explicit application or test selector must be absolute:

```sh
export BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release"
export NATIVE_TEST_BUILD_PATH="$BIKE_NATIVE_BUILD_PATH"
uv run --frozen --group native --no-sync bike-ride --help
```

`tools/dev.py test --build native/build/release --profile native` sets both
selectors to one absolute path and delegates to the existing test entry point.
Use `quick`, `native` or `full` according to [verification rules](../../AGENTS.md).
The native/full profiles explicitly configure/build as part of their gate;
that behavior is distinct from application import and doctor.

The loader retains `importlib.machinery.EXTENSION_SUFFIXES` and requires exactly
one matching extension in the selected directory. It checks pinned dependencies,
the imported file path, and the extension's compiled native source stamp. A
pre-imported extension from another directory is an error: the loader does not
silently unload it or retry with a different binary. A cached matching module
still has its compiled source stamp checked.

## Doctor and relocated builds

`doctor` emits schema-1 JSON on stdout and human failure details on stderr.
Without `--build`, absence of a native extension is not a failure. `--numerics`
requests live numerical-provider inspection; `--build` requests the artifact,
build-context, runtime-library and numerical checks together.

The requested native checks cover:

- The selected interpreter, exact configured Python version and SOABI, and
  CMake's original source/build directories.
- Exactly one selected extension and live import from that file in a fresh
  process; compiled/current native source hashes and extension hash are exposed
  under `dependency_probe.details.native`.
- Existing recorded dependency files, the loaded MuJoCo 3.12.0 image, all three
  required private ABI symbols, and agreement between Python/native engine paths.
- The runtime numerical provider and its agreement with the compiled provider
  policy; a deleted, substituted or incompatible provider fails inspection.

Doctor performs file checks, version queries and a fresh runtime import. It
does **not** invoke the configure-time compile/link/run ABI probe in
`tools/native_dependencies.py`. Explicit CMake configuration owns that
qualification. A doctor pass cannot replace CTest, the native gate, a GUI
acceptance run, or a fully isolated offline installation.

T23 adds `python` (executable, version, SOABI and include directory),
`source_directory` and `build_directory` to `native_check_context.json`.
Its schema stays **2** and the original `python_executable` field is retained.
Existing runtime/test/sweep readers remain compatible with prior schema-2
contexts. Doctor requests explicit reconfiguration when an older context lacks
the new installation diagnostics; it does not invent missing evidence.

Build caches and virtualenvs retain absolute paths to their original source and
dependencies. Move or extract the **source**, choose/create the target's editable
environment, and configure a fresh build directory there. Do not reuse a copied
cache or move a venv as an installation method. The driver rejects mismatched
`CMAKE_HOME_DIRECTORY`, `CMAKE_CACHEFILE_DIR` or Python selection before invoking
a build. It never deletes an existing directory to repair the mismatch.

## Diagnose failures

| Diagnostic | Meaning and next step |
| --- | --- |
| Python-only installation has no native source checkout | Use the complete source tree and its own editable uv environment. A build path alone does not repair wheel-only layout. |
| No extension / more than one matching suffix | Explicitly configure/build the selected directory, or use a fresh unambiguous build directory. The message includes that directory and interpreter details. |
| Pre-imported module differs from the selected artifact | Start a new process with the intended selector. Do not remove a live extension from `sys.modules` to switch implementations. |
| Compiled/current source stamp differs | Explicitly rebuild the intended current sources; retain the source identity checks. |
| Configured interpreter, SOABI or source/build directory differs | Select the correct source environment; after relocation or an environment replacement, configure a fresh build directory. |
| Native dependency version mismatch | Restore the frozen native dependency group in the selected environment; do not relax its pins. |
| Native load failed / missing symbol / missing shared library | Read the original loader error retained as the exception cause, run doctor for the same absolute build, and restore/rebuild against the selected libraries. Do not mask it with arbitrary symlinks or broad library search paths. |
| Removed numerical provider | Restore the qualified provider and rebuild/restart in the selected environment; a mapped-but-deleted image is not valid identity evidence. |

Native import exceptions include selected build/artifact, `sys.executable`,
SOABI, operating system/architecture, and explicit doctor/configure/build
instructions. Dynamic-loader `ImportError`/`OSError` text and causes remain
available for diagnosis; there is no silent fallback or implicit retry.

## Verification still required for this change

The non-building regression suite covers fresh-process source/wheel-layout
selection, Python-only help/reference routing, missing/ambiguous artifacts,
relative selectors, cached-module conflicts, stale source stamps, dependency
version rejection, removed providers, preserved loader causes, custom venv
propagation, and copied-cache rejection. Its fixtures use synthetic artifacts;
they are not native binary or packaging qualification.

Once execution is authorized, run the focused tests in the selected frozen
environment, then the required native gate:

```sh
uv run --frozen --group native --no-sync python -m pytest tests/portability/test_install_modes.py tests/portability/test_dev_commands.py -q
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" bash tools/run_tests.sh native
```

Separately qualify a real fresh editable install; install the existing pure
Python wheel in a fresh environment and exercise help/reference/native-refusal
routes; relocate source and rebuild locally; reject a copied cache/venv; and
repeat valid/invalid loader controls on Linux and macOS. Record commands,
interpreter/compiler/provider versions, imported artifact paths and actual
results. T24/T25 own full/slim offline certification. None of these execution
gates was run in the static T23 session.
