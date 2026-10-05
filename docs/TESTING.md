# Running tests

Run these commands from the repository root:

```bash
bash tools/run_tests.sh          # defaults to quick
bash tools/run_tests.sh quick    # excludes tests marked slow
bash tools/run_tests.sh native   # native extension and golden episode checks
bash tools/run_tests.sh full     # includes slow physics and realtime episodes
```

Use `quick` for ordinary edits and the final fast regression check. Use `native`
for work on the C++ implementation and its Python bridge. Use `full` explicitly
when evaluating complete physical episodes or realtime acceptance. The launcher
resolves its repository root, so it also works when invoked from another directory.

All profiles run serially with `-q --durations=10`, forward additional arguments
to pytest, and return pytest's exit status. For example:

```bash
bash tools/run_tests.sh native --collect-only
bash tools/run_tests.sh quick -k config
bash tools/run_tests.sh full --collect-only
bash tools/run_tests.sh --help
```

An unknown profile returns exit status 2. For a targeted file or test node, invoke
pytest directly; the native profile always selects all native and golden files:

```bash
uv run python -m pytest tests/reference/test_native_suspension.py -q --durations=10
uv run python -m pytest tests/reference/test_native_suspension.py::test_suspension_components_bitwise -q
```

## Native prerequisite

Build the extension in an already configured `native/build` directory before
running native checks:

```bash
uv run cmake --build native/build -j4
```

The native profile prepends the absolute `native/build` directory to `PYTHONPATH`,
preserving an existing value. It imports `bike_native` and prints the imported
module's path before starting pytest. If import fails, the launcher exits with
the import command's failure status and pytest does not start. This makes a
missing or broken extension visible before native tests can be skipped.

For a direct native test invocation, set the same import path:

```bash
PYTHONPATH="$PWD/native/build${PYTHONPATH:+:$PYTHONPATH}" uv run python -m pytest tests/reference/test_native_suspension.py -q
```

## Timing and current acceptance

Prior measurements were approximately 2 seconds for `-m 'not slow'`, 6 seconds
for the six-module P2 focused run, and 478 seconds for the full suite on commit
`a216f36`. These are measurements from prior runs, not runtime promises.

The quick and P2 focused checks were green in those runs. The full run on
`a216f36` had 520 passes and 24 failures; passing quick or native checks does not
establish full physical or realtime acceptance. Full episodes are explicit
because they take minutes and realtime tests measure machine speed. Keep those
tests serial and evaluate their failures separately when full acceptance is
needed.
