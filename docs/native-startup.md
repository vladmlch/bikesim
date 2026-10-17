# Native startup

This document describes the native startup interfaces added for T16–T18 and
T20–T25: ordinary configuration and terrain inputs, native model generation,
physical equilibrium, an owned reset state, and bounded ride execution.

**Status: source-only implementation.** The code and examples were inspected
statically. CMake configuration, compilation, linking, tests, simulations and
project programs were **not run**, as requested for this implementation session.
The commands below are examples for later use with a qualified build; they are
not successful execution records. A Repomix extraction may omit binary `.f64`
oracle payloads and `.npz` pose files even when their manifests or referring TOML
files are present. Missing fixtures must be acquired and verified before the
corresponding acceptance tests can pass.

The implementation is registered in
[`native/cmake/NativeStartup.cmake`](../native/cmake/NativeStartup.cmake).
The application target is `bike-native-ride`, and the C++ application library is
`bike_app`. The ordinary preparation path constructs the model and initial state
in C++; it does not require a previously captured Python bootstrap. Removal of
the old implementation and final Python-free acceptance remain later migration
work.

## Current execution scope

The startup path performs these operations in order:

1. Resolve and validate physics, bike/rider and terrain inputs.
2. Generate MJCF, compile the model through the native engine boundary, and
   populate its normalized heightfield samples.
3. Solve physical equilibrium, restore an accepted equilibrium state, or prepare
   an explicit compatible initial-state transfer.
4. Establish the `t = 0` material, controller, velocity and energy state, then
   seal an owned `runtime::RuntimeBootstrap`.
5. Construct the existing `runtime::NativeRideRuntime`, which owns advancement,
   snapshots, reset and close.

Runtime support is checked on resolved values by
[`validate_native_supported`](../native/src/runtime/project_config.cpp).
The startup executable currently requires:

| Setting | Supported value |
|---|---|
| Model mode | `ride` |
| Physics mode | `physical` |
| Drive mode | `articulated_effort` |
| Rider variant | `articulated_planar` |
| Pitch assist | Disabled |
| Pedal / saddle / grip attachments | `spindle` / `pin` / `connect` |
| Transmission model | `ideal_mid_drive` |
| Motor clutch | Disabled |
| Reflected rotor inertia | `0.0` |
| Tire backend | `compliant_2d`, `distributed_2d_reference`, or `pneumatic` |

Compliant and distributed tires use `surface_mode = "track"`. Pneumatic tires
also accept `surface_mode = "configured"` with an explicit named pneumatic
surface. The compliant backend requires analytic front and rear tire material
specifications. The ordinary physics validators still apply to every backend.
Unsupported combinations fail during preparation.

The executable is a small startup and advancement entry point. Recording,
DuckDB output, replay files, reports, the viewer and the full application CLI
belong to T38–T51. Its JSON completion record is not a recording or an accepted
migration report.

## CLI examples for a later qualified build

These examples assume `bike-native-ride` is available on `PATH` and the working
directory is the repository root. They intentionally request a fresh equilibrium
so that they do not depend on a saved pose or an existing cache entry.

Prepare the default flat track and stop at the initialized state:

```sh
bike-native-ride \
  --physics-config examples/research/viewer_physics_welded.toml \
  --track flat --fresh --steps 0
```

Prepare an authored research track and request one simulated second:

```sh
bike-native-ride \
  --physics-config examples/research/viewer_physics_welded.toml \
  --track-config examples/research/rough_uphill.toml \
  --fresh --duration 1.0
```

Prepare the pneumatic profile on a flat track without opening its configured
scenario-specific saved equilibrium pose:

```sh
bike-native-ride \
  --physics-config examples/research/viewer_physics_pneumatic.toml \
  --track flat --fresh --steps 0
```

Use a schema-1 request document:

```sh
bike-native-ride --request request.toml --fresh --steps 0
```

`--steps 0` still performs model preparation and equilibrium. The ride's step or
duration limit applies after preparation; it does not shorten equilibrium.
`--max-equilibrium-steps N` sets a separate positive relaxation budget. Failure
to converge within that budget is an error, and no successful bootstrap is
sealed. The default equilibrium tolerance is `0.05` on the maximum absolute
generalized acceleration. The default step budget follows the source rule
`max(40000, round(80 / timestep_s))`.

### Options

The parser is in
[`native/src/app/native_ride_main.cpp`](../native/src/app/native_ride_main.cpp).

| Option | Meaning |
|---|---|
| `--physics-config FILE` | Load an ordinary physics TOML document. Required unless `--request` is supplied. |
| `--request FILE` | Load the assembly document described below. Exclusive with `--physics-config`, `--track`, `--track-config`, `--rider` and `--seed`. |
| `--track PRESET` | Select a built-in track; the direct-config default is `flat`. |
| `--track-config FILE` | Load an authored or generated track TOML document. Exclusive with `--track`. |
| `--rider NAME` | Direct-config rider selection; defaults to `articulated_planar`. The current runtime support gate requires that variant. |
| `--seed DECIMAL` | Nonnegative decimal integer string; the direct-config default is `0`. See the seed rules below. |
| `--initial-speed-kph VALUE` | Override the initial speed, converting km/h to m/s. This is allowed with either input form. |
| `--steps N` | Request a nonnegative target step, with `0` retaining the prepared state. |
| `--duration SECONDS` | Request a nonnegative finite simulated duration, rounded up to whole physics steps. Exclusive with `--steps`; defaults to one second when neither is supplied. |
| `--max-equilibrium-steps N` | Positive maximum relaxation step count. |
| `--fresh` | Ignore initial-state transfer and saved equilibrium input, and disable automatic cache reads and writes for this invocation. |
| `--cache-dir DIR` | Select the automatic equilibrium-cache directory; does not itself enable caching. |
| `--legacy-source-root DIR` | Select the source tree whose bytes identify an explicit historical compatibility input. See the provenance section below. |
| `--no-strict` | Disable the runtime's strict reference-monitor mode. Configuration validation and equilibrium acceptance remain in force. |
| `--help`, `-h` | Print usage. |

The built-in track names are `alpine_rocky`, `alpine_scree`, `climb_steps`,
`enduro_aggressive`, `flat`, `road_broken`, `road_smooth`, `road_worn`,
`single_edge` and `washboard_only`. The flat preset is 112 m long. Named presets
retain their source-defined recipes and seeds; `--seed` does not reseed a named
preset.

The executable applies a default `RideControl`, zero external front/rear brake
demands, and a finish position equal to the track length. Configuration-owned
control policies remain active. Advancement can stop before the requested target
when it reaches a terminal outcome or the track finish.

On successful preparation and advancement, standard output contains one JSON
object with `schema`, `track`, `step`, `time_s`, `reason`,
`equilibrium_residual_qacc`, and nullable `outcome`. It does not print the complete
preparation metadata. Exit status is `0` for ordinary completion, `3` for a
reported `crash:` outcome, and `2` for an exception. Exception diagnostics go to
standard error with the `bike-native-ride:` prefix.

## Request schema 1

The request reader is
[`native/src/initialization/request.cpp`](../native/src/initialization/request.cpp).
The following example assumes that `request.toml` is saved at the repository
root:

```toml
request_schema = 1
physics_file = "examples/research/viewer_physics_welded.toml"
track_file = "examples/research/rough_uphill.toml"
model_mode = "ride"
rider_variant = "articulated_planar"
speed_kph = 0.0
seed_decimal = "1234"
use_equilibrium_cache = false

# Optional: an explicit initial-state transfer or equilibrium-state file.
# initial_state_file = "poses/my_initial_state.json"

# Optional: partial physics overrides, merged before final validation.
[overrides]
initial_front_brake = 1.0
initial_rear_brake = 1.0
```

The eight fields before the optional sections are required. `request_schema`
must be integer `1`, `model_mode` must be `"ride"`, `speed_kph` must be finite,
and `seed_decimal` must be a nonempty string containing decimal digits only.
`use_equilibrium_cache` must be a boolean. `initial_state_file` and `overrides`
are optional. Unknown top-level keys fail. Bike geometry, field dimensions,
cache directory, legacy source root and equilibrium budget are not extra
schema-1 TOML keys; use the typed API or the corresponding CLI options.

Physics values are resolved in this order: load `physics_file`, merge
`overrides`, then apply the request's `speed_kph`, `use_equilibrium_cache` and
optional `initial_state_file`. Common CLI options such as
`--initial-speed-kph` and `--fresh` apply after the request values. The CLI passes
its fresh-start choice into the reader so that a discarded request
`initial_state_file` need not be opened.

### Relative paths

- `physics_file`, `track_file` and `initial_state_file` resolve relative to the
  directory containing the request document.
- Asset paths inside a physics document resolve relative to that physics
  document. For example, its rider envelope and strength JSON files stay beside
  the physics TOML when that directory is moved.
- Asset paths introduced under `[overrides]` resolve relative to the request
  document, which declares those overrides.
- Direct CLI paths, including `--cache-dir` and `--legacy-source-root`, resolve
  from the process's working directory when supplied as relative paths.

The CMake install rules place the executable under `bin` and preserve the
shipped example TOML/JSON/NPZ directory structure under
`share/bike_sim/examples`. They cannot supply binary files absent from the input
repository. Installed-layout execution has not been verified in this session.

### Generator seeds and materialized tracks

The setup reader uses `load_track_for_setup` to distinguish two cases:

- If the track document contains `[generator]`, the request's decimal seed
  overrides `generator.seed` before generator validation and generation.
- If the document contains only materialized geometry, the request seed is
  metadata. Explicit obstacle seeds and the existing geometry remain the track
  inputs.

Seed strings are not narrowed to 64 bits. The terrain TOML adapter also preserves
integer-valued `seed` fields beyond the signed 64-bit TOML parser range,
including valid hexadecimal, octal and binary spellings. This extension is
limited to seed fields; it does not widen every TOML integer field.

The standalone terrain API retains the historical contract:
`terrain::load_track(path, explicit_seed)` rejects a track without a generator.
The setup helper owns the metadata-only rule for materialized tracks.

## Saved state, cache and provenance

Without `--fresh`, an explicit initial-state transfer takes precedence over a
new equilibrium solve. The request reader recognizes a JSON transfer document
by its `contract_sha256` member and validates it through `PhysicalInitialState`.
The target must satisfy the physical model and initial support-road contract;
the target residual is measured during restoration. Otherwise,
`initial_state_file` selects the equilibrium-state reader, which accepts the
native versioned representation and supported historical NPZ input.

An explicit equilibrium file takes precedence over the automatic cache. It must
match the expected fingerprint, material layout and applicable model/provider
identity, and its acceleration residual is rechecked. Missing or incompatible
explicit state is an error. An absent or invalid automatic cache entry permits
a fresh solve. Optional cache publication failure does not turn a settled run
into a failed run.

Automatic cache storage is selected in this order:

1. `SetupRequest::cache_directory`, set by `--cache-dir` in the CLI.
2. `BIKE_SIM_EQUILIBRIUM_CACHE_DIR`, when nonempty.
3. The system temporary directory's `bike_sim_equilibrium` subdirectory.

The cache path uses the equilibrium fingerprint and the historical `.npz`
suffix; the reader identifies the actual supported content format. Cache and
state-file pointers are excluded from the physical fingerprint. Native startup
metadata also retains model, provider and source identities, resolved input
identities, equilibrium evidence and the sealed-state flag.

Native cache provenance always comes from the native build identity and retains
the complete typed track, provider and model identity.
`--legacy-source-root DIR`, or `SetupRequest::legacy_source_root`, separately
supplies historical source-byte provenance for old NPZ validation. That path
uses the source-compatible track dictionary and does not replace native cache
identity. Supply the same source-root boundary
used by the historical capture, normally `src/bike_sim` from the matching
checkout. This operation reads and hashes ordered `.py` paths and bytes; it does
not import or run those files. Selecting a legacy source root does not waive
fingerprint or model checks and does not establish compatibility with an
arbitrary old pose. Historical NPZ compatibility remains unqualified until the
matching payloads and comparisons are available.

The shipped pneumatic physics example refers to a saved pose for a specific
track. For a different track, or an extraction missing that pose, `--fresh`
explicitly requests independent native initialization. Other required inputs,
including physics, terrain and rider assets, must still be present and valid.

## C++ ownership API

Repository clients include `app/prepare.hpp` with `native/src` on their include
path and link `bike_app`. The public entry points are:

```cpp
bike::app::PreparedSimulation bike::app::prepare(
    const std::filesystem::path& request_toml);
bike::app::PreparedSimulation bike::app::prepare(
    bike::app::SetupRequest request);
std::unique_ptr<runtime::NativeRideRuntime> bike::app::make_ride(
    bike::app::PreparedSimulation prepared);
```

`PreparedSimulation` owns its `bootstrap` and `metadata`. Preparation must finish
equilibrium and the initial reset before it returns. Move the prepared value
into `make_ride`; copy any metadata needed separately before that move. The
returned runtime owns its state independently of the request and setup objects.
Its `reset()` restores the sealed bootstrap; it does not rerun terrain generation
or equilibrium.

For example, after compilation and runtime qualification:

```cpp
auto prepared = bike::app::prepare("request.toml");
auto metadata = prepared.metadata;
auto ride = bike::app::make_ride(std::move(prepared));
auto initial = ride->snapshot();
auto advanced = ride->advance(10, runtime::RideControl{}, 0.0, 0.0,
                             std::nullopt);
ride->reset();
ride->close();
```

Callers needing access between equilibrium and sealing use
[`SetupOwner`](../native/src/initialization/setup_owner.hpp):

```cpp
auto request = bike::initialization::read_setup_request("request.toml");
auto equilibrium_options = request.equilibrium;
auto owner = bike::initialization::make_setup(std::move(request));
auto equilibrium = bike::initialization::settle(*owner, equilibrium_options);
auto initial_state = owner->capture_initial_state();
auto bootstrap = owner->seal_bootstrap();
auto metadata = owner->metadata();
```

`make_setup` owns the generated model, data, writers and initialization context.
`settle` can complete the owner only once; sealing before completion or sealing
twice fails. Explicit initial-state capture and distributed-station projection
are `t = 0` operations. `build_equilibrium_evaluator` binds a setup request for sag
fitting and creates an independent candidate setup for each pressure/rate pair.

`read_setup_request(path, true)` skips the request's explicit
`initial_state_file` read. That parameter alone is not the complete fresh-start
policy: a typed caller must also disable `equilibrium_cache_enabled`, clear
`equilibrium_state_file` in the resolved physics, and clear `initial_state`.
The CLI's `--fresh` performs all of these steps.

Implementation entry points are
[`app/prepare.hpp`](../native/src/app/prepare.hpp),
[`app/prepare.cpp`](../native/src/app/prepare.cpp),
[`initialization/setup_owner.hpp`](../native/src/initialization/setup_owner.hpp)
and [`initialization/request.cpp`](../native/src/initialization/request.cpp).
Acceptance remains governed by the
[migration specification](superpowers/specs/2026-10-14-python-free-cpp.md),
[interface contracts](../migration/interface-contracts.md),
[native engineering rules](agents/native-engineering.md) and the task-specific
regressions. Documentation and static inspection do not close those gates.
