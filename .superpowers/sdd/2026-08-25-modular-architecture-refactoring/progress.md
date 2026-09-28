# SDD ledger — plan: docs/superpowers/plans/2026-08-25-modular-architecture-refactoring.md

## Preflight Plan Scan
| Task Pair / Task | Prods vs Cons | Findings & Rulings |
|---|---|---|
| Task 1 (Phase 0: Output Isolation & Snapshots) | Snapshots `bike_model.xml`, `coordinates.json`, moves outputs to `output/` | Self-contained, establishes baseline |
| Task 2 (Phase 1: Package Skeleton) | Creates `src/bike_sim/`, configures `pyproject.toml` | Establishes package layout |
| Task 3 (Phase 2.1: Geometry) | Produces `BikeSpecs`, `hardpoints` | Consumed by Kinematics, Physics, MuJoCo |
| Task 4 (Phase 2.2: Kinematics) | Produces `HorstLinkageSolver`, `curves` | Consumes Geometry; consumed by MuJoCo, Viz |
| Task 5 (Phase 2.3: Physics & Config) | Produces `BikeMassSpecs`, `AirSpringModel`, `DamperModel`, `BikeConfig` | Consumes Geometry, Kinematics; consumed by MuJoCo |
| Task 6 (Phase 3: MuJoCo Subsystem) | Produces 8 sub-modules, `generate_mujoco_xml` | Consumes Geometry, Kinematics, Physics; snapshot equivalence checked |
| Task 7 (Phase 4: Simulation Subsystem) | Produces 4 sub-modules, `SuspensionPlayground` | Consumes MuJoCo, Geometry, Physics |
| Task 8 (Phase 5: Viz & CLI) | Produces `viz`, `cli` (`bike-sim`, `bike-playground`, `bike-export`) | Consumes all subpackages |
| Task 9 (Phase 6: Tests, Shims, Docs) | Updates all 8 test suites, `ARCHITECTURE.md` | Verifies full 95 tests pass, cleans root |

Preflight scan clean: No contradictory interfaces, dependency graph is strictly acyclic.
