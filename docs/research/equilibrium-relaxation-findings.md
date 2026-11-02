# Equilibrium relaxation: cycle vs kinetic damping

Measured 2026-11-03 on main3 (`aece48d7` plus the test-only follow-up `268db105`),
macOS arm64, `release` preset.

## The setting

Before t=0 the native setup searches for a rest pose: the bike and rider are
placed on the ground, the plant is stepped, and velocities are zeroed now and
then so the motion dies out. The search stops once every degree of freedom's
acceleration residual is at most the acceptance tolerance (0.05 by default,
`EquilibriumOptions::tolerance`).

When to zero velocities is selected by the physics config key
`equilibrium_relaxation` (top level of the physics TOML):

```toml
equilibrium_relaxation = "kinetic"   # default: "cycle"
```

- `cycle` (default, historical): zero velocities on a fixed 20 ms clock.
- `kinetic`: zero velocities once total kinetic energy has passed its peak
  (it dropped 2 % below the running peak), with segments bounded to
  [2 ms, 250 ms]. Implementation: `native/src/initialization/equilibrium.cpp`
  (`Segments`).

`kinetic` is opt-in. The default path and its result record are unchanged.

## Finding: the acceptance gate does not pin a unique rest pose

Both schedules pass the 0.05 gate on every shipped request fixture, but they
stop in different poses:

| request fixture | fork travel, mm (cycle / kinetic) | rear travel, mm | pitch, rad | steps | residual |
|---|---|---|---|---|---|
| `welded_flat` | 61.741 / 58.043 | 45.449 / 46.774 | 0.006812 / 0.003077 | 800 / 916 | 0.0060 / 0.0212 |
| `distributed_flat` | 60.879 / 57.971 | 45.742 / 46.528 | 0.005922 / 0.003150 | 800 / 943 | 0.0017 / 0.0415 |
| `pneumatic_flat` | 68.896 / 54.441 | 42.840 / 47.961 | 0.013720 / −0.001030 | 19040 / 916 | 0.0500 / 0.0491 |

A kinetic settle also rolls the bike about 2 cm forward (`settle_x_drift_m`
0.021–0.023 m) and takes 7–13 velocity resets.

Tightening the tolerance does not resolve this. Within the 64 000-step
budget, at 0.005 cycle converges on `welded_flat` and `distributed_flat`
only, and kinetic converges on no profile. At 0.0005 neither schedule
converges on any profile. The residual floor is held by degrees of freedom with no restoring force:
`rear_wheel_spin`, `front_wheel_spin`, `pedal_*_spin` and `crank_spin`, then
the rear linkage pivots (`rocker_frame_pivot`, `yoke_pivot`, `main_pivot`).

### What this means

- The starting pose of every ride is determined only to within a few
  millimetres of suspension travel (14 mm of fork travel on `pneumatic_flat`).
  Which pose you get depends on the path the search takes, not only on the
  physics.
- This is a property of the equilibrium search, not of the kinetic port.
  The Python test that came with the feature asserted the same rest within
  0.5 mm, but on a different configuration (compliant_2d tyres, lumped
  rider). On the native fixtures that does not hold, and the native test
  (`native/tests/initialization/test_setup.cpp`, `kinetic_relaxation`) does
  not assert it.

### Open directions (not done)

- Exclude the free spins (wheels, pedals, crank) from the stop criterion, or
  hold them, so the residual measures the supported pose only.
- Add a pose-convergence criterion (change in suspension travel and pitch
  between resets) next to the acceleration residual.
- Then re-measure whether cycle and kinetic agree, and only then decide
  whether `kinetic` should become the default.

## How the numbers were produced

A temporary experiment in `native/tests/initialization/test_setup.cpp` (not
committed) settled each request fixture with
`init::make_setup(request)` + `init::settle(*owner, options)` after:
`equilibrium_cache_enabled = false`, `equilibrium_state_file` reset,
`initial_speed_mps = 0`, `request.initial_state` reset, and
`equilibrium_relaxation` set to `"cycle"` or `"kinetic"`. It printed
`EquilibriumResult` fields for `options.tolerance` in {0.05, 0.005, 0.0005};
a failed settle prints the `EquilibriumFailure` message with the worst
accelerations.
