#!/usr/bin/env python3
"""
Pedalled Drivetrain Acceptance Checks.

Runs the three checks the pedalling work is judged against, plus the bob metric it exists to
produce. Everything that needs a compiled model is skipped automatically when MuJoCo is not
installed, so the torque arithmetic can still be verified anywhere.

    python tools/validate_pedals.py            # all checks on the default track
    python tools/validate_pedals.py --quick    # shorter traverse, same checks

**Check 1 -- the baseline is reproduced.** A `pedal` run with `--ripple-depth 0` asks the
crank for a constant torque, so the wheel should see what the ideal motor saw. It is
*not* bit-identical and cannot be: the crank is a real body with real inertia, and the chain
is a constraint the solver enforces. The check is therefore a tolerance on mean speed and on
mean wheel torque, and the tolerance is the result -- a number to quote, not a pass mark to
hide behind.

**Check 2 -- the gearing is not inverted.** Mean wheel torque divided by mean crank torque
must come out at the gear ratio. This is the check that catches a reciprocal ratio, a ratio
applied twice, or a polycoef written the wrong way round, all of which otherwise look like
"the bike is mysteriously slow".

**Check 3 -- the bob appears where the physics says it must.** Two power strokes per
revolution put the excitation at twice the cadence. The shock stroke must carry more
amplitude at that frequency in a pedalled run than in the motor run, which has no such
excitation at all.

**Check 4 -- the freewheel re-engages without an impulse.** The chain equality is a position
constraint, so every metre coasted with the chain open is angle the solver would have to
recover at engagement. This check watches the residual directly and watches the run stay
upright, because checks 1 to 3 average over a traverse and an averaged channel hides a
one-step blow-up: a run that goes over the bars at the first bump still produces a mean.
"""

import argparse
import importlib.util
import sys
from math import pi
from pathlib import Path

REPO_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(REPO_SRC))


def _load_pure(name: str, relative: str):
    """Loads one module by path, bypassing the package import chain and its dependencies."""
    spec = importlib.util.spec_from_file_location(name, REPO_SRC / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_torque_shape() -> int:
    """
    Verifies the ripple shape, the rider ceilings and the assist law without a model.

    Returns:
        Number of failures.
    """
    import numpy as np

    dt = _load_pure("pedal_physics", "bike_sim/physics/drivetrain.py")
    specs = dt.DrivetrainSpecs()
    failures = 0
    print("-- torque shape and limits ------------------------------------------------")

    phase = np.linspace(0.0, 2.0 * pi, 100_001)[:-1]
    for depth in (0.0, 0.5, 0.85, 1.0):
        values = np.array([dt.ripple_shape(p, depth) for p in phase])
        mean, floor, peak = values.mean(), values.min(), values.max()
        ok = abs(mean - 1.0) < 1e-6
        failures += not ok
        print(f"   depth {depth:4.2f}: mean {mean:.6f} {'OK' if ok else 'FAIL'}, "
              f"dead centre {100 * floor:5.1f} % of mean, peak {peak:.3f}x")

    # A mean the rider cannot hold is cut by whichever ceiling binds first.
    torque, limit = dt.limited_rider_torque(200.0, 0.5, specs)
    ok = limit == "torque" and abs(torque - specs.rider_torque_ceiling_nm) < 1e-9
    failures += not ok
    print(f"   at 5 rpm the torque ceiling binds: {torque:.1f} N.m via '{limit}' {'OK' if ok else 'FAIL'}")
    cadence_radps = 82.0 / dt.RPM_PER_RADPS
    torque, limit = dt.limited_rider_torque(200.0, cadence_radps, specs)
    expected = specs.rider_power_ceiling_w / cadence_radps
    ok = limit == "power" and abs(torque - expected) < 1e-9
    failures += not ok
    print(f"   at 82 rpm the power ceiling binds: {torque:.1f} N.m via '{limit}' {'OK' if ok else 'FAIL'} "
          f"({specs.rider_power_ceiling_w:.0f} W)")

    # The crank actuator must cover the worst case the ceilings allow.
    peak_crank = specs.rider_torque_ceiling_nm * dt.peak_shape(specs.ripple_depth) + specs.assist_torque_ceiling_nm
    ok = peak_crank <= dt.CRANK_TORQUE_CEILING_NM
    failures += not ok
    print(f"   worst-case crank torque {peak_crank:.1f} N.m vs {dt.CRANK_TORQUE_CEILING_NM:.0f} N.m "
          f"actuator ceiling {'OK' if ok else 'FAIL'}")

    # Cutoff taper, and the wheel-side consequence of each assist mode.
    for kmh, expected in ((20.0, 1.0), (24.0, 0.5), (26.0, 0.0)):
        got = dt.cutoff_factor(kmh / 3.6, specs)
        ok = abs(got - expected) < 1e-9
        failures += not ok
        print(f"   cutoff at {kmh:4.1f} km/h: support x{got:.2f} {'OK' if ok else 'FAIL'}")
    for mode, factor in dt.ASSIST_MODES.items():
        mean_wheel = 30.0 * (1.0 + factor) * specs.gear_ratio
        print(f"   {mode:6s} support x{factor:4.2f}: 30 N.m of legs -> {mean_wheel:6.1f} N.m at the wheel, "
              f"peak {mean_wheel * dt.peak_shape(specs.ripple_depth):6.1f}")
    return failures


def _run(track, speed_kmh, drive_mode, assist, ripple_depth, steps):
    """
    Runs one headless traverse and returns its recorder channels and summary inputs.

    Args:
        track: Track to ride.
        speed_kmh: Cruise target.
        drive_mode: ``motor``, ``pedal`` or ``pedelec``.
        assist: Assist mode for ``pedelec``.
        ripple_depth: Crank torque ripple depth.
        steps: Steps to run.

    Returns:
        Tuple of (channels dict, sample interval, the simulation).
    """
    import numpy as np

    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride.recorder import RideRecorder
    from bike_sim.sim.ride_sim import RideSimulation

    specs = None if drive_mode == "motor" else DrivetrainSpecs(ripple_depth=ripple_depth)
    sim = RideSimulation(
        track=track,
        target_speed_kmh=speed_kmh,
        drive_mode=drive_mode,
        assist=assist,
        drivetrain=specs if specs is not None else DrivetrainSpecs(),
    )
    recorder = RideRecorder(sim)
    for _ in range(steps):
        sim.step()
        recorder.record(sim)
    return recorder.columns(), recorder.sample_interval_s, sim


def _assert_upright(label: str, sim) -> int:
    """
    Reports whether a traverse finished on its wheels.

    Args:
        label: Name of the run, for the log line.
        sim: The simulation the run used.

    Returns:
        1 if the run crashed, 0 otherwise.
    """
    if sim.crash is None:
        return 0
    print(f"   FAIL: the {label} run crashed -- {sim.crash.cause} at "
          f"x = {sim.crash.position_m:.2f} m; the means below are of a crash, not a ride")
    return 1


def check_reengagement(quick: bool) -> int:
    """
    Verifies that opening and re-closing the chain leaves no position residual behind.

    Args:
        quick: Whether to use the shorter traverse.

    Returns:
        Number of failures.
    """
    import numpy as np

    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.terrain import DEFAULT_PRESET, get_preset

    sim = RideSimulation(
        track=get_preset(DEFAULT_PRESET),
        target_speed_kmh=20.0,
        drive_mode="pedelec",
        assist="tour",
        drivetrain=DrivetrainSpecs(),
    )
    steps = 4000 if quick else 12000
    residuals, speeds, engagements = [], [], 0
    was_engaged = True
    for _ in range(steps):
        sim.step()
        residuals.append(abs(sim.drivetrain.chain_residual(sim.model, sim.data)))
        speeds.append(sim.speed_mps)
        engaged = not sim.drivetrain.command.freewheel
        engagements += engaged and not was_engaged
        was_engaged = engaged
        if sim.crash is not None:
            break

    failures = 0
    worst = float(np.max(residuals))
    # One step of crank rotation at 20 km/h in 32x14 is 0.004 rad. A residual an order of
    # magnitude above that is the freewheel's accumulated angle, not integration error.
    ok = worst < 0.04
    failures += not ok
    print(f"   {engagements} re-engagements, worst chain residual {worst:.2e} rad "
          f"(limit 4.0e-02) {'OK' if ok else 'FAIL'}")

    slowest = float(np.min(speeds))
    ok = slowest > 0.0
    failures += not ok
    verdict = "OK" if ok else "FAIL: driven backwards, which is the engagement impulse"
    print(f"   lowest forward speed {slowest:.3f} m/s {verdict}")

    failures += _assert_upright("re-engagement", sim)
    if sim.crash is None:
        print("   run finished upright OK")
    return failures


def check_against_model(quick: bool) -> int:
    """
    Runs the three model-level checks.

    Args:
        quick: Whether to use the shorter traverse.

    Returns:
        Number of failures; zero when MuJoCo is unavailable and the checks are skipped.
    """
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("\n-- model checks -----------------------------------------------------------")
        print("   SKIPPED: MuJoCo is not installed in this environment. Run this script where")
        print("   `import mujoco` works to execute the baseline, gearing and bob checks.")
        return 0

    import numpy as np

    from bike_sim.sim.ride.metrics import pedal_bob_stats, sine_amplitude
    from bike_sim.terrain import DEFAULT_PRESET, get_preset

    track = get_preset(DEFAULT_PRESET)
    steps = 4000 if quick else 12000
    speed = 20.0
    failures = 0

    print("\n-- check 1: ripple depth 0 reproduces the motor baseline -------------------")
    motor, dt_s, motor_sim = _run(track, speed, "motor", "off", 0.0, steps)
    smooth, _, smooth_sim = _run(track, speed, "pedal", "off", 0.0, steps)
    failures += _assert_upright("motor baseline", motor_sim)
    failures += _assert_upright("smooth crank", smooth_sim)
    for name, tolerance in (("speed_mps", 0.02), ("drive_torque_nm", 0.10)):
        a, b = float(np.mean(motor[name])), float(np.mean(smooth[name]))
        rel = abs(b - a) / max(abs(a), 1e-9)
        ok = rel <= tolerance
        failures += not ok
        print(f"   mean {name}: motor {a:.4f} vs smooth crank {b:.4f} -> {100 * rel:.2f} % "
              f"(tolerance {100 * tolerance:.0f} %) {'OK' if ok else 'FAIL'}")

    print("\n-- check 2: the gearing is not inverted ------------------------------------")
    pedalled, _, sim = _run(track, speed, "pedal", "off", 0.85, steps)
    failures += _assert_upright("pedalled", sim)
    stats = pedal_bob_stats(pedalled, dt_s)
    ratio = sim.drivetrain.specs.gear_ratio
    ok = stats is not None and abs(stats.gear_ratio_check - ratio) / ratio <= 0.05
    failures += not ok
    print(f"   wheel torque / crank torque = {stats.gear_ratio_check:.4f} vs gearing {ratio:.4f} "
          f"{'OK' if ok else 'FAIL'}")

    print("\n-- check 3: bob appears at twice the cadence -------------------------------")
    bob_hz = stats.bob_frequency_hz
    pedalled_mm = sine_amplitude(pedalled["shock_stroke_mm"], bob_hz, dt_s)
    motor_mm = sine_amplitude(motor["shock_stroke_mm"], bob_hz, dt_s)
    ok = pedalled_mm > motor_mm
    failures += not ok
    print(f"   cadence {stats.mean_cadence_rpm:.1f} rpm -> {bob_hz:.2f} Hz; shock amplitude "
          f"pedalled {pedalled_mm:.3f} mm vs motor {motor_mm:.3f} mm {'OK' if ok else 'FAIL'}")

    print("\n-- check 4: the freewheel re-engages without an impulse --------------------")
    failures += check_reengagement(quick)

    print("\n-- assist sweep: what the mid-drive does to the bob ------------------------")
    for mode in ("off", "eco", "tour", "sport", "turbo"):
        channels, _, _ = _run(track, speed, "pedelec", mode, 0.85, steps)
        s = pedal_bob_stats(channels, dt_s)
        print(f"   {mode:6s}: cadence {s.mean_cadence_rpm:5.1f} rpm, legs {s.crank_torque_mean_nm:5.1f} N.m, "
              f"motor {s.assist_torque_mean_nm:5.1f} N.m, shock bob {s.shock_bob_amplitude_mm:.3f} mm, "
              f"freewheel {100 * s.freewheel_fraction:3.0f} %")
    return failures


def main(argv=None) -> int:
    """Runs the checks; returns the process exit code."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--quick", action="store_true", help="shorter traverse, same checks")
    args = parser.parse_args(argv)

    failures = check_torque_shape() + check_against_model(args.quick)
    print()
    print("ALL CHECKS PASSED" if failures == 0 else f"{failures} CHECK(S) FAILED")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
