# bike-sim migration source audit

**Date:** 2026-10-14. **Method:** static source inspection, AST inventory, native/Python dependency review and source-based plan synthesis. No application code was executed. No build/test/performance result is being certified.

## Snapshot and extraction

Input: `repomix-output.xml(4).zip`. ZIP SHA-256: `caf5212ae0268a496490666fb12906fc3b8552adc1689be6313c75c1ce83abdc`.

Packed XML SHA-256: `0885327ee4035ff842d65741c05f61cd085010ecf11a5f9a33584908f0ca4424`.

All 1,075 extracted paths were checked for uniqueness, absolute paths and parent traversal before unmixing with the Repomix Unmixer script. The restored text was AST-parseable for all 459 Python files. The manifest hashes the restored file bytes; these are not Git blob IDs or proof of byte-identical omitted binary assets. The unmixing parser removes the Repomix wrapper newlines. T00 checks any final-newline difference against the real checkout before calling it a semantic source change.

| Category | Files | Physical lines |
|---|---:|---:|
| Application Python (`src`) | 284 | 56,422 |
| Test Python (`tests`) | 155 | 39,752 |
| Tool Python (`tools`) | 18 | 5,581 |
| Separate project test | 1 | 170 |
| Root conftest | 1 | 5 |
| **All Python** | **459** | **101,930** |
| C++ `.cpp` / `.hpp` across snapshot | 157 | 53,701 |

The full AST traversal indexes 1,611 `test_` functions/methods; 1,595 are top-level functions. This is not pytest collection or coverage. Binary fixtures, a real Git revision and `uv.lock` were not present in the supplied text snapshot and must be reconciled/acquired from the real checkout when needed. The complete text manifest, not this summary, defines what was extracted.

## Findings and source anchors

### A1. Native ride startup still depends on Python

The native constructor consumes an initialized Python simulation and later captures its t=0 frame/bootstrap. T22-T25 replace the setup path; T48 replaces initial presentation.

Source: `src/bike_sim/native/runtime.py:183-197`.

```text
 183 def create_native_ride(sim, *, strict: bool | None = None,
 184                        record_decimation: int | None = None,
 185                        retention: str | None = None,
 186                        directory=None) -> NativeRideDriver:
 187     """Capture a settled t=0 ride and build its independent native owner.
 188 
 189     ``record_decimation`` selects the energy/momentum measurement cadence
 190     (the measurement schedule). ``retention`` selects which committed
 191     samples the recording layer keeps: ``'off'`` none, ``'full'`` every
 192     interval, ``'legacy'`` the historical decimated projection (default).
 193     """
 194     physical = getattr(sim, 'physical', None)
 195     if physical is None:
 196         raise ValueError('create_native_ride requires a physical ride runtime')
 197     if strict is not None and not isinstance(strict, bool):
```

### A2. Research construction still needs a reference environment

The remaining work includes reference environment construction and noise/startup state capture, not only moving a viewer loop. T19/T34 own this boundary.

Source: `src/bike_sim/native/research.py:458-470`.

```text
 458 def create_native_research(reference: ResearchEnvironment, *, directory=None):
 459     """Capture a safe research t=0; do not replay startup noise or holding brakes."""
 460     if not isinstance(reference, ResearchEnvironment):
 461         raise ValueError('expected a fresh Python ResearchEnvironment as the setup reference')
 462     if reference.sim.steps != 0 or reference.sim.time_s != 0. or reference.control_pending or reference.done:
 463         raise ValueError('native research setup requires the initial external boundary')
 464     validate_supported(reference.sim.physics_config, reference.sim.rider)
 465     from bike_sim.native.artifact import load_native_extension, execution_provenance
 466     extension = load_native_extension()
 467     execution = execution_provenance('native', artifact=extension)
 468     if extension.runtime_identity()['native_source_sha256'] != execution['native_source_sha256']:
 469         raise ValueError('selected extension was not built from the current native source')
 470     tape = build_noise_tape(reference.sensor_config, seed=reference.seed,
```

### A3. Bootstrap capture is an initialization contract

Capture validates an already completed initial state. New startup uses a separate setup owner rather than circularly constructing a live runtime from a nonexistent bootstrap.

Source: `src/bike_sim/native/setup.py:342-360`.

```text
 342 def capture_bootstrap(sim, directory: Path) -> RuntimeBootstrap:
 343     """Freeze a freshly initialized physical ride into a RuntimeBootstrap.
 344 
 345     Requires t=0 on a completed initialization: ``sim.steps == 0``,
 346     ``data.time == 0`` and a finished physical reset (the equilibrium,
 347     settle and accounting baselines all exist). Bootstrap capture follows
 348     the frontend's initialization exactly once (design section 3.1); it is
 349     not a mid-run checkpoint format.
 350     """
 351     physical = getattr(sim, 'physical', None)
 352     if physical is None:
 353         raise ValueError('capture_bootstrap requires a physical ride runtime')
 354     validate_supported(sim.physics_config, sim.rider)
 355     if not physical.research_accounting_valid:
 356         raise ValueError('bootstrap capture requires a completed physical reset')
 357     if getattr(physical, 'initializing', False):
 358         raise ValueError('bootstrap capture requires a finished initialization')
 359     if sim.steps != 0 or float(sim.data.time) != 0.:
 360         raise ValueError(
```

### A4. Native build finds Python and pins an engine ABI

Standalone build discovery must replace interpreter/wheel paths while retaining the pinned engine ABI. macOS SDK and Accelerate usage are separately visible near the top and link targets.

Source: `native/CMakeLists.txt:160-178`.

```text
 160 find_package(Python 3.13 REQUIRED COMPONENTS Interpreter Development.Module)
 161 
 162 execute_process(COMMAND "${Python_EXECUTABLE}" -c "import mujoco; print(mujoco.__version__)"
 163   RESULT_VARIABLE _native_mujoco_version_result
 164   OUTPUT_VARIABLE _native_mujoco_version
 165   ERROR_VARIABLE _native_mujoco_version_error
 166   OUTPUT_STRIP_TRAILING_WHITESPACE)
 167 if(NOT _native_mujoco_version_result EQUAL 0 OR NOT _native_mujoco_version STREQUAL "3.12.0")
 168   message(FATAL_ERROR
 169     "bike_native requires the pinned MuJoCo 3.12.0 error ABI; configured Python '${Python_EXECUTABLE}' found '${_native_mujoco_version}': ${_native_mujoco_version_error}")
 170 endif()
 171 
 172 execute_process(COMMAND "${Python_EXECUTABLE}" -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))"
 173   OUTPUT_VARIABLE MJ_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
 174 execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
 175   OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
 176 list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
 177 find_package(nanobind CONFIG REQUIRED)
 178 
```

### A5. Native config owns nanobind objects

These are actual boundary dependencies inside C++ data records. Removing .py files alone does not produce a standalone native core.

Source: `native/src/runtime/config.hpp:44-71`.

```text
  44     std::optional<std::string> front_wheel_body;
  45     std::optional<std::string> rear_wheel_body;
  46     std::optional<std::string> com_marker_site;
  47     nanobind::object pose;  // owned dict copy or None
  48 };
  49 
  50 struct RuntimeConfig {
  51     double timestep_s = 0.;
  52     double control_period_s = 0.;
  53     int control_period_steps = 0;
  54     bool strict = false;
  55     // The expensive energy/momentum measurement cadence — historically the
  56     // `record_decimation` wire/argument value, now named for what it gates.
  57     int measurement_decimation = 0;
  58     RetentionMode record_retention = RetentionMode::legacy;
  59     MonitorConfig monitors;
  60     GeometryConfig geometry;
  61     nanobind::object writer_config;     // owned dict — the Stepper consumes it once
  62     nanobind::object rider_controller;  // owned dict — staged for the A2 controller
  63     nanobind::object rider_intent;      // owned dict — staged for the A2 scheduler
  64 };
  65 
  66 // Throws std::invalid_argument naming the full config path on any deviation
  67 // from the runtime_schema=1 envelope.
  68 [[nodiscard]] RuntimeConfig parse_runtime_config(nanobind::handle raw);
  69 
  70 // Deep ownership copy of a plain-data tree (dict/list/tuple/str/bool/
  71 // integral/real/ndarray/None). Immutable scalars are shared; containers and
```

### A6. Bootstrap records also contain Python objects

Move these fields to typed owned state and keep the existing validation/reset semantics.

Source: `native/src/runtime/bootstrap.hpp:58-73`.

```text
  58     drivetrain::DriveSnapshot drive;
  59     std::optional<std::pair<writers::RiderContactsState,
  60                             std::optional<writers::RiderContactsProbe>>>
  61         rider_contacts;
  62     // Owned plain-data trees staged for their A2/A3 native consumers.
  63     nanobind::object rider_controller;
  64     nanobind::object rider_intent;
  65     nanobind::object signals;
  66     nanobind::object runtime;
  67     // Scalars the runtime itself consumes, lifted out of `runtime`.
  68     std::int64_t step = 0;
  69     int generation = 0;
  70 };
  71 
  72 // Reads state.model_digest — validated presence + string type only. The
  73 // runtime compares it against sha256(model_path bytes) before loading.
```

### A7. Research includes a binding header

The research numerical implementation is substantially native, but its include closure is not independent of Python.

Source: `native/src/runtime/research.hpp:6-21`.

```text
   6 #include <memory>
   7 #include "programs.hpp"
   8 #include "recording.hpp"
   9 #include "research_metrics.hpp"
  10 #include "runtime_binding.hpp"
  11 #include "sensors.hpp"
  12 
  13 namespace runtime {
  14 enum class ResearchCaptureMode : std::uint8_t { off, full, legacy, verify };
  15 struct ResearchConfig {
  16     double timestep_s = 0., track_length_m = 0., start_position_m = 0.;
  17     double maximum_energy_residual_ratio = .05, wheelie_persistence_s = .02;
  18     std::int64_t control_steps = 0, delay_steps = 0, max_steps = 0, sensor_steps = 0;
  19     int record_decimation = 1;
  20     ResearchCaptureMode capture_mode = ResearchCaptureMode::legacy;
  21     bool stop_on_model_violation = true;
```

### A8. Column pieces already own native typed buffers

Reuse the typed column representation and move-only ownership; replace parsing/boxing/ingestion rather than rewrite the encoder as JSON rows.

Source: `native/src/runtime/encode.hpp:23-55`.

```text
  23 struct Piece {
  24     enum class Kind : std::uint8_t { scalar_d, scalar_i, scalar_b, scalar_s,
  25                       list_d, list_i, list_s, structure, map }
  26         kind = Kind::scalar_d;
  27     std::vector<double> d;
  28     std::vector<std::int64_t> i;
  29     std::vector<char> b;                    // bool payload
  30     std::vector<std::string> s;             // varchar payload / map keys
  31     std::vector<std::int32_t> offsets;      // list/struct[]/map member spans
  32     std::vector<char> valid;                // empty => all-valid
  33     std::map<std::string, Piece> children;  // struct members incl *_present
  34     // Struct member dispatch cache, parallel to the declaring plan node's
  35     // children order: slot i addresses children[child.name] and, for an
  36     // optional member, its *_present companion (nullptr when required).
  37     // structure_piece fills it once — the pointed-to children are std::map
  38     // nodes, so their addresses survive every move of this piece — and the
  39     // per-row converters index it instead of repeating the tree lookups.
  40     struct MemberSlot {
  41         Piece *member = nullptr;
  42         Piece *present = nullptr;
  43     };
  44     std::vector<MemberSlot> member_slots;
  45     std::unique_ptr<Piece> items;           // map values / struct[] elements
  46 
  47     Piece() = default;
  48     // Copying a structure piece would leave member_slots pointing at the
  49     // source's children; pieces are moved or emplaced, never copied.
  50     Piece(const Piece &) = delete;
  51     Piece &operator=(const Piece &) = delete;
  52     Piece(Piece &&) = default;
  53     Piece &operator=(Piece &&) = default;
  54     ~Piece() = default;
  55 };
```

### A9. Recording publication lives in Python

Port transaction/validation/hash/exclusive publication/manifest-last and retry behavior. The plan does not introduce streaming capture.

Source: `src/bike_sim/recording/storage.py:388-433`.

```text
 388 def write_recording(capture, out_root, *, batch_rows=BATCH_ROWS, _reporter=None):
 389     if not capture.enabled:
 390         return None
 391     if type(batch_rows) is not int or batch_rows < 1:
 392         raise ValueError('batch_rows must be a positive integer')
 393     root = Path(out_root)
 394     root.mkdir(parents=True, exist_ok=True)
 395     staging = Path(tempfile.mkdtemp(prefix='.run-recording-unsealed-', dir=root))
 396     (staging/'UNSEALED').write_text('A manifest is required. This directory is not a recording.\n', encoding='utf-8')
 397     destination = None
 398     try:
 399         counts = _database(capture, staging/DATABASE, batch_rows=batch_rows)
 400         report_status, report_error = 'complete', None
 401         try:
 402             (_report if _reporter is None else _reporter)(staging/DATABASE, staging/REPORT)
 403             if not (staging/REPORT).is_file():
 404                 raise RuntimeError('report generator produced no summary.md')
 405         except Exception as error:
 406             report_status, report_error = 'failed', f'{type(error).__name__}: {error}'
 407         payloads = [DATABASE] + ([REPORT] if report_status == 'complete' else [])
 408         for name in payloads:
 409             _sync_file(staging/name)
 410         manifest = dict(schema_version=SCHEMA_VERSION, sealed=True,
 411                         run_id=str(capture.metadata.run_id), database=DATABASE,
 412                         file_sha256={name: sha256_file(staging/name) for name in payloads},
 413                         row_counts=counts, report_status=report_status, report_error=report_error)
 414         candidate = staging/'sealed-manifest.json'
 415         candidate.write_text(dumps(manifest, indent=2)+'\n', encoding='utf-8')
 416         _sync_file(candidate)
 417         timestamp = capture.metadata.started_at.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
 418         # mkdir owns the name, not exists()+replace(). Even an empty collision
 419         # belongs to somebody else and is skipped, never overwritten.
 420         base = timestamp + '-' + str(capture.metadata.run_id)
 421         attempt = 0
 422         while True:
 423             name = base if not attempt else base + '-' + str(attempt)
 424             destination = root/name
 425             try:
 426                 destination.mkdir()
 427                 break
 428             except FileExistsError:
 429                 attempt += 1
 430         for name in payloads:
 431             # Hard links keep a retryable staging copy. Both directories are
 432             # created under the same parent/filesystem; link is exclusive.
 433             os.link(staging/name, destination/name)
```

### A10. Recording and replay version axes differ

Container schema 1 and replay contract 3 must not be conflated.

Source: `src/bike_sim/recording/schema.py:11-20`.

```text
  11 SCHEMA_VERSION = 1
  12 REPLAY_CONTRACT_VERSION = 3
  13 BATCH_ROWS = 4096
  14 DATABASE = 'run.duckdb'
  15 MANIFEST = 'manifest.json'
  16 REPORT = 'summary.md'
  17 
  18 
  19 def quote(name):
  20     if not isinstance(name, str) or not name or '\x00' in name:
```

### A11. Analyst exports are CSV and JSONL

The plan preserves actual export formats; it does not invent a requirement to replace nonexistent Parquet export.

Source: `src/bike_sim/cli/recording.py:34-52`.

```text
  34             command.add_argument('--fields', required=True)
  35         if name == 'sql':
  36             command.add_argument('--query', required=True)
  37         if name == 'export':
  38             command.add_argument('--format', choices=('csv', 'jsonl'), required=True)
  39             command.add_argument('--output', type=Path, required=True)
  40             command.add_argument('--table', default='telemetry', choices=(
  41                 'telemetry', 'legacy_telemetry', 'forces', 'commands_requested', 'commands_applied',
  42                 'observations', 'transitions', 'events', 'metrics'))
  43             command.add_argument('--limit', type=int, default=None,
  44                                  help='optional explicit export row limit; default exports all rows')
  45     command = commands.add_parser('compare')
  46     command.add_argument('left', type=Path)
  47     command.add_argument('right', type=Path)
  48     command.add_argument('--left-episode', type=int, required=True)
  49     command.add_argument('--right-episode', type=int, required=True)
  50     command.add_argument('--mode', choices=('exact', 'summary'), default='exact')
  51     command.add_argument('--fields')
  52     command.add_argument('--atol', type=float, default=1e-9)
```

### A12. Both native and full run native preflight

The shared launcher calls preflight_native for both profiles. Future runner/check migrations preserve that behavior.

Source: `tools/run_tests.sh:182-194`.

```text
 182 if [[ "$profile" == native || "$profile" == full ]]; then
 183   export PYTHONPATH="$REPO_ROOT/tests/reference${PYTHONPATH:+:$PYTHONPATH}"
 184   selected_build="$(native_uv python -c 'import os, sys; sys.path.insert(0, "tests/reference"); from native_loader import selected_build; print(selected_build(os.environ))')"
 185   export NATIVE_TEST_BUILD_PATH="$selected_build"
 186   preflight_native "$selected_build"
 187   pytest_args+=(-p native_test_reporter)
 188 fi
 189 
 190 if [[ "$profile" == native || "$profile" == full ]]; then
 191   run_with_native_runtime python -m pytest "${pytest_args[@]}" "$@"
 192 else
 193   exec uv run python -m pytest "${pytest_args[@]}" "$@"
 194 fi
```

### A13. Allocation accepts only remeasured feasible results

Preserve the exact objective, constraint order, SLSQP settings, restart loop and residual threshold. Optimizer success is not the acceptance condition.

Source: `src/bike_sim/physics/rider_allocation.py:106-136`.

```text
 106     def gradient(x):
 107         return x - target
 108 
 109     # SLSQP occasionally stalls on a line-search step with a nearly feasible
 110     # iterate; a warm restart from the returned point pushes through without
 111     # relaxing any residual tolerance.
 112     for _ in range(4):
 113         result = minimize(objective, x, jac=gradient,
 114             bounds=Bounds(lower, upper), constraints=constraints,
 115             method='SLSQP', options={'ftol': 1e-12, 'maxiter': 500})
 116         x = np.asarray(result.x, dtype=float)
 117         if not np.isfinite(x).all():
 118             break
 119         candidate = np.clip(x, lower, upper)
 120         residual = _residual(candidate, aeq, beq, g, h, extra_constraints, lower, upper)
 121         x = candidate
 122         if residual <= 1e-7:
 123             break
 124     if not np.isfinite(x).all():
 125         return Allocation(np.clip(np.nan_to_num(x), lower, upper), False, float('inf'))
 126     violation = _residual(x, aeq, beq, g, h, extra_constraints, lower, upper)
 127     return Allocation(x.copy(), bool(violation <= 1e-7), violation)
 128 
 129 
 130 def _as_old_style(constraint, start):
 131     """Avoid SciPy conversion while retaining equality and bound semantics."""
 132     if not isinstance(constraint, (LinearConstraint, NonlinearConstraint)):
 133         raise ValueError('unsupported allocation constraint')
 134     if isinstance(constraint, LinearConstraint):
 135         a = np.atleast_2d(np.asarray(constraint.A, dtype=float))
 136         fun = lambda x: a @ x
```

### A14. Rendering has declared mutable model fields

The render replica needs six explicit model fields as well as integration state; they are not reconstructed by stepping.

Source: `src/bike_sim/sim/ride/physical_view.py:20-24`.

```text
  20 MODEL_FIELDS = ('dof_frictionloss', 'site_pos', 'tendon_stiffness',
  21                 'tendon_damping', 'tendon_lengthspring', 'tendon_range')
  22 
  23 
  24 def _hud(sim):
```

### A15. Annular inertia is a full tensor

Preserve four physical inputs and a full 3x3 tensor; reducing this helper to a thin ring or diagonal-only parallel-axis shortcut changes behavior.

Source: `src/bike_sim/physics/inertia.py:32-52`.

```text
  32 def ring_inertia(mass_kg: float, inner_m: float, outer_m: float, width_m: float) -> np.ndarray:
  33     """Inertia of a uniform annular cylinder about its center, with axis Y."""
  34     if not all(math.isfinite(v) for v in (mass_kg, inner_m, outer_m, width_m)):
  35         raise ValueError("non-finite inertia input")
  36     if mass_kg <= 0 or inner_m < 0 or outer_m <= inner_m or width_m <= 0:
  37         raise ValueError("invalid annular cylinder")
  38     radial = inner_m**2 + outer_m**2
  39     transverse = mass_kg * (3 * radial + width_m**2) / 12
  40     return np.diag([transverse, mass_kg * radial / 2, transverse])
  41 
  42 
  43 def parallel_axis(mass_kg: float, offset_m: tuple[float, float, float] | np.ndarray) -> np.ndarray:
  44     """Move an inertia tensor through a displacement in body coordinates."""
  45     r = np.asarray(offset_m, dtype=float)
  46     if r.shape != (3,) or not np.isfinite(r).all() or not math.isfinite(mass_kg) or mass_kg < 0:
  47         raise ValueError("invalid parallel-axis input")
  48     return mass_kg * (float(r @ r) * np.eye(3) - np.outer(r, r))
  49 
  50 
  51 def wheel_body_inertia(
  52     mass_kg: float, components: tuple[WheelComponent, ...]
```

### A16. Native runtime initially patches t=0 presentation through Python

The native initialization work must produce the initial view/model fields so rendering does not retain a hidden Python dependency.

Source: `src/bike_sim/native/runtime.py:88-99`.

```text
  88         from bike_sim.sim.ride.physical_view import native_frame
  89         frame = native_frame(snap, _sample(snap.latest_sample))
  90         # Bootstrap restores the integration vector without mj_forward. The
  91         # already-settled setup owns the t=0 geometry/diagnostics; no extra
  92         # engine solve may be inserted merely to draw the initial frame.
  93         if frame.step == 0 and self._initial_frame is not None:
  94             frame = _amend(frame, view=self._initial_frame.view,
  95                            model_fields=self._initial_frame.model_fields)
  96         return frame
  97 
  98     def make_render_model(self):
  99         import mujoco
```

## What the audit does not claim

The counts do not prove that every Python line must be translated literally. A mirror can be retired after branch-level native parity; a Python-only adapter is replaced by an owned native boundary; a package marker can be retired after import/build edges are closed. Conversely, matching a native filename or a passing short scenario does not prove complete behavior.

No present-tense performance, numerical parity, platform qualification or successful-build claim is made here. Historical checked replay, exact random streams, solver/provider parity, binary fixture recovery and custom policy ports remain explicit implementation gates. Native dependency archive hashes for newly introduced libraries are acquired/verified in T03, not fabricated in this plan.

## Audit outputs

`migration/source-manifest.json`: all restored text paths/hashes. `migration/python-removal.json`: all 459 first-party Python paths and owners. `migration/file-work-units.json` and `migration/cards/`: declaration-level source cards. `migration/test-port-map.json`: qualified static test index. `migration/dependency-edges.json`: non-Python files mentioning interpreter/library/build edges; comments are intentionally distinguished from executable dependencies during T56/T62.
