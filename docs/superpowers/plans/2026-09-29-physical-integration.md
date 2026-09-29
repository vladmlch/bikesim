# Physical integration and release verification Implementation Plan

> Execution: inline in the supplied offline project. Existing phase plans and the
> physics-correctness specification remain authoritative.

**Goal:** Connect the supplied C5-F4 components to the public simulator and test
actual compiled dynamics, preserving the legacy branch.

**Architecture:** A PhysicalRuntime owns force producers, initialization and
interval accounting. The original orchestrator dispatches by physics mode.
Physical topology is finalized only after original mass-budget allocation.

**Tech Stack:** Python 3.13, bundled MuJoCo 3.12.0, NumPy, SciPy, pytest.

**Spec:** docs/superpowers/specs/2026-09-28-physics-correctness.md

## Global Constraints

- SI internally; planar X-Z model only.
- No root stabilization or post-step position/velocity corrections in physical.
- Preserve legacy outputs and isolate all physical topology changes.
- Synthetic parameters are not experimental calibration; no V2 claim.
- Keep uv.lock and declared dependencies unchanged. Record actual environment.
- Read scoped source ranges, not the complete source tree.

## Task 1: topology and initialization
Files: builder.py, rider.py, physical_topology.py, new runtime integration tests.
Run red: `PYTHONPATH=src python -m pytest tests/test_physical_integration.py -q`.
Connect explicit crank/cassette and independent rider builders, skip legacy chain
constraints, transfer masses once, initialize phase before static relaxation.
Compile all physical drive/rider combinations, check summed mass and no welds.

## Task 2: force ownership and observations
Files: sim/ride/physical_runtime.py, physical_equilibrium.py, ride_sim.py,
physical_samples.py, rider_control.py, engine integration tests.
Use fresh kinematics, one advance per interval, one accumulator, solved brake
rows and actuator force captured before any post-step forward call. Track actual
work every step; CSV decimation must not change it. Test virtual work, contact
release, low-friction wheelspin, passivity and conservation.

## Task 3: release interface
Files: cli/ride.py, sim/ride/physical_session.py, physics/configuration.py,
recorder/HUD/plots adapters and CLI tests.
Resolve defaults < TOML < explicit CLI. Hash complete resolved inputs and actual
terrain. Fit physical sag through compiled equilibrium. Export schema 2 only
for physical, with known limits and actual runtime versions.

## Task 4: acceptance and review
Files: validation/benchmarks.py, tools/validate_physics.py, regression tests,
verification/review.md. Execute the independent rigs on dt=0.0005/0.00025/0.000125,
then the full test suite and headless CLI runs. Review every changed file and
record unresolved criteria explicitly. Package source, patch and measured logs.
