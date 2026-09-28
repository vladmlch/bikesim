#!/usr/bin/env python3
"""
Pedalled Drivetrain Acceptance Checks.

Runs the three checks the pedalling work is judged against, plus the bob metric it exists to
produce. Everything that needs a compiled model is skipped automatically when MuJoCo is not
installed, so the torque arithmetic can still be verified anywhere.

    python tools/validate_pedals.py            # all checks on the flat track
    python tools/validate_pedals.py --quick    # shorter traverse, same checks

**Check 1 -- the regulated speed is still reached.** The launch cannot match the ideal
motor -- the rider's 60 N.m ceiling through 32x14 is only ~26 N.m at the wheel -- but the
closed loop must settle on the same cruise target and, once settled, deliver the same
wheel torque, because both runs are balancing the same rolling resistance.

**Check 2 -- the gearing is what the joints measure.** Wheel spin divided by crank spin must
come out at the gear ratio, which is what catches a reciprocal polycoef, a ratio applied
twice, or a ratio wired backwards. The commanded wheel torque is then checked end-to-end
against the chassis' own acceleration: when the demand mapping sent the ratio the wrong way
the telemetry still agreed with itself -- it was computed from the same wrong formula -- so
this check reads the state, not the command.

**Check 3 -- the bob appears where the physics says it must.** Two power strokes per
revolution put the excitation at twice the cadence. The shock stroke must carry more
amplitude at that frequency in a pedalled run than in the motor run, which has no such
excitation at all.

**Check 4 -- the freewheel re-engages without an impulse.** The chain equality is a position
constraint, so every metre coasted with the chain open is angle the solver would have to
recover at engagement. The overrun is forced deterministically: the cruise target drops
below the bike's speed, which a freewheel cannot satisfy, so the chain opens and the brakes
hold until the target is restored. The residual is read only while the chain is closed --
an open freewheel is *supposed* to accumulate angle, that is what the datum rewrite exists
for.
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

    # Cutoff taper, and the wheel-side consequence of each assist mode. The chain divides
    # crank torque by the gear ratio: 32 teeth driving 14 trades torque for wheel speed.
    for kmh, expected in ((20.0, 1.0), (24.0, 0.5), (26.0, 0.0)):
        got = dt.cutoff_factor(kmh / 3.6, specs)
        ok = abs(got - expected) < 1e-9
        failures += not ok
        print(f"   cutoff at {kmh:4.1f} km/h: support x{got:.2f} {'OK' if ok else 'FAIL'}")
    for mode, factor in dt.ASSIST_MODES.items():
        mean_wheel = 30.0 * (1.0 + factor) / specs.gear_ratio
        print(f"   {mode:6s} support x{factor:4.2f}: 30 N.m of legs -> {mean_wheel:6.1f} N.m at the wheel, "
              f"peak {mean_wheel * dt.peak_shape(specs.ripple_depth):6.1f}")
    return failures


def _run(track, speed_kmh, drive_mode, assist, ripple_depth, steps, specs=None):
    """
    Runs one headless traverse and returns its recorder channels and summary inputs.

    Args:
        track: Track to ride.
        speed_kmh: Cruise target.
        drive_mode: ``motor``, ``pedal`` or ``pedelec``.
        assist: Assist mode for ``pedelec``.
        ripple_depth: Crank torque ripple depth.
        steps: Steps to run.
        specs: Drivetrain limits; defaults to the shipped ones with ``ripple_depth``.

    Returns:
        Tuple of (channels dict, sample interval, the simulation).
    """
    import numpy as np

    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride.recorder import RideRecorder
    from bike_sim.sim.ride_sim import RideSimulation

    if specs is None:
        specs = DrivetrainSpecs(ripple_depth=ripple_depth)
    sim = RideSimulation(
        track=track,
        target_speed_kmh=speed_kmh,
        drive_mode=drive_mode,
        assist=assist,
        drivetrain=specs,
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


def _pedal_probe(track, speed_kmh: float, ripple_depth: float, steps: int):
    """
    Runs one pedalled traverse, sampling the joint state the recorder does not store.

    Args:
        track: Track to ride.
        speed_kmh: Cruise target.
        ripple_depth: Crank torque ripple depth.
        steps: Steps to run.

    Returns:
        Tuple of (the simulation, sample dict of per-step arrays).
    """
    import numpy as np

    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride_sim import RideSimulation

    sim = RideSimulation(
        track=track,
        target_speed_kmh=speed_kmh,
        drive_mode="pedal",
        assist="off",
        drivetrain=DrivetrainSpecs(ripple_depth=ripple_depth),
    )
    wheel_dof, crank_dof = sim.drivetrain.wheel_dofadr, sim.drivetrain.crank_dofadr
    keys = ("t", "v", "ww", "wc", "wheel_nm", "shock", "cadence", "engaged")
    samples = {k: [] for k in keys}
    for _ in range(steps):
        sim.step()
        command = sim.drivetrain.command
        samples["t"].append(float(sim.data.time))
        samples["v"].append(sim.speed_mps)
        samples["ww"].append(float(sim.data.qvel[wheel_dof]))
        samples["wc"].append(float(sim.data.qvel[crank_dof]))
        samples["wheel_nm"].append(sim.wheel_drive_torque_nm)
        samples["shock"].append(sim.shock_stroke_mm)
        samples["cadence"].append(command.cadence_rpm)
        samples["engaged"].append(not command.freewheel)
        if sim.crash is not None:
            break
    return sim, {k: np.asarray(v) for k, v in samples.items()}


def check_reengagement(quick: bool) -> int:
    """
    Verifies that opening and re-closing the chain leaves no position residual behind.

    The overrun is forced deterministically: midway through the traverse the cruise target
    drops below the bike's speed, which a freewheel cannot satisfy, so the chain opens and
    the brakes hold until the target is restored and the pawls catch again. The residual is
    read only while the chain is closed -- an open freewheel is supposed to accumulate
    angle, that is what the datum rewrite exists for.

    Args:
        quick: Whether to use the shorter traverse.

    Returns:
        Number of failures.
    """
    import numpy as np

    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.terrain import get_preset

    sim = RideSimulation(
        track=get_preset("flat"),
        target_speed_kmh=20.0,
        drive_mode="pedelec",
        assist="tour",
        drivetrain=DrivetrainSpecs(),
    )
    plan = ((20.0, 9000), (15.0, 3000), (20.0, 6000)) if quick else (
        (20.0, 16000), (15.0, 6000), (20.0, 10000)
    )
    residuals, speeds = [], []
    openings = engagements = 0
    was_engaged = True
    for target, steps in plan:
        sim.cruise.target_speed_kmh = target
        for _ in range(steps):
            sim.step()
            engaged = not sim.drivetrain.command.freewheel
            if engaged:
                residuals.append(abs(sim.drivetrain.chain_residual(sim.model, sim.data)))
            engagements += engaged and not was_engaged
            openings += (not engaged) and was_engaged
            was_engaged = engaged
            speeds.append(sim.speed_mps)
            if sim.crash is not None:
                break
        if sim.crash is not None:
            break

    failures = 0
    ok = openings >= 1 and engagements >= 1
    failures += not ok
    print(f"   {openings} openings, {engagements} re-engagements "
          f"{'OK' if ok else 'FAIL: the forced overrun never cycled the freewheel'}")

    # One step of crank rotation at 20 km/h in 32x14 is 0.004 rad. A residual an order of
    # magnitude above that is the freewheel's accumulated angle, not integration error.
    worst = float(np.max(residuals)) if residuals else 0.0
    ok = worst < 0.04
    failures += not ok
    print(f"   worst closed-chain residual {worst:.2e} rad (limit 4.0e-02) {'OK' if ok else 'FAIL'}")

    slowest = float(np.min(speeds)) if speeds else 0.0
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
    Runs the model-level checks.

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
    from bike_sim.terrain import get_preset

    # The drivetrain checks run on the flat track: obstacles knock speed off a rider who
    # is already at the human ceiling, and the checks are about the drivetrain, not the
    # terrain. 15 km/h is the bottom of the cruise band -- the 60 N.m rider through 32x14
    # (~26 N.m at the wheel) accelerates far slower than the ideal motor, so the pedalled
    # runs get the longer window and the comparisons read the settled tails.
    track = get_preset("flat")
    speed = 15.0
    motor_steps = 6000 if quick else 12000
    pedal_steps = 30000 if quick else 48000
    failures = 0

    print("\n-- check 1: the pedalled bike holds the same cruise as the motor -----------")
    motor, dt_s, motor_sim = _run(track, speed, "motor", "off", 0.0, motor_steps)
    smooth, _, smooth_sim = _run(track, speed, "pedal", "off", 0.0, pedal_steps)
    failures += _assert_upright("motor baseline", motor_sim)
    failures += _assert_upright("smooth crank", smooth_sim)
    a = float(np.mean(motor["speed_mps"][motor["speed_mps"].size // 2:]))
    b = float(np.mean(smooth["speed_mps"][smooth["speed_mps"].size // 2:]))
    rel = abs(b - a) / max(abs(a), 1e-9)
    ok = rel <= 0.05
    failures += not ok
    print(f"   settled mean speed_mps: motor {a:.4f} vs smooth crank {b:.4f} -> {100 * rel:.2f} % "
          f"(tolerance 5 %) {'OK' if ok else 'FAIL'}")
    # Holding the same speed asks for more wheel torque once a rider is pedalling: the
    # seated rider's legs ride the pedal circle and that bob is a real load (measured: the
    # same run with no rider settles near 1.7 N.m), on top of the bottom bracket's drag.
    # The overhead is a fraction, not a factor -- the inverted-ratio failure this guards
    # reads as x5.2.
    a = float(np.mean(motor["drive_torque_nm"][motor["drive_torque_nm"].size // 2:]))
    b = float(np.mean(smooth["drive_torque_nm"][smooth["drive_torque_nm"].size // 2:]))
    overhead = b / max(a, 1e-9)
    ok = 1.0 <= overhead <= 2.5
    failures += not ok
    print(f"   settled mean drive_torque_nm: motor {a:.4f} vs smooth crank {b:.4f} -> "
          f"pedalling overhead x{overhead:.2f} (bounds x1.0..x2.5) {'OK' if ok else 'FAIL'}")

    print("\n-- check 2: the gearing is what the joints measure --------------------------")
    ped_sim, ped = _pedal_probe(track, speed, 0.85, pedal_steps)
    failures += _assert_upright("pedalled", ped_sim)
    engaged = ped["engaged"] & (ped["wc"] > 0.2)
    ratio = ped_sim.drivetrain.specs.gear_ratio
    measured = float(np.median(ped["ww"][engaged] / ped["wc"][engaged]))
    ok = abs(measured - ratio) / ratio <= 0.05
    failures += not ok
    print(f"   wheel/crank speed ratio {measured:.4f} vs gearing {ratio:.4f} {'OK' if ok else 'FAIL'}")

    # End to end: the wheel torque implied by the chassis' own acceleration must match what
    # the drivetrain reports it is delivering. The fit runs over the ramp only -- once the
    # speed curve saturates against the rider's ceiling a whole-run slope reads low --
    # and rolling resistance with the leg-damper bob pull the implied figure a third or so
    # below the commanded one; a factor of two either way is a bookkeeping error, not
    # physics.
    ramp = engaged & (ped["v"] > 0.1) & (ped["v"] < 0.6 * speed / 3.6)
    accel = float(np.polyfit(ped["t"][ramp], ped["v"][ramp], 1)[0])
    mass_kg = float(ped_sim.model.body_mass.sum())
    implied_nm = mass_kg * accel * ped_sim.specs.rear_wheel_radius / 1000.0
    commanded_nm = float(np.mean(ped["wheel_nm"][ramp]))
    ok = commanded_nm > 0.0 and 0.45 <= implied_nm / commanded_nm <= 1.2
    failures += not ok
    print(f"   accel-implied wheel torque {implied_nm:6.1f} N.m vs commanded {commanded_nm:6.1f} N.m "
          f"(bounds x0.45..x1.2) {'OK' if ok else 'FAIL'}")

    print("\n-- check 3: bob appears at twice the cadence -------------------------------")
    tail = slice(2 * ped["t"].size // 3, None)
    cadence = float(np.mean(ped["cadence"][tail][ped["engaged"][tail]]))
    bob_hz = 2.0 * cadence / 60.0
    pedalled_mm = sine_amplitude(ped["shock"][tail], bob_hz, dt_s)
    motor_tail = slice(2 * motor["shock_stroke_mm"].size // 3, None)
    motor_mm = sine_amplitude(motor["shock_stroke_mm"][motor_tail], bob_hz, dt_s)
    ok = cadence > 20.0 and pedalled_mm > motor_mm
    failures += not ok
    print(f"   cadence {cadence:.1f} rpm -> {bob_hz:.2f} Hz; shock amplitude "
          f"pedalled {pedalled_mm:.3f} mm vs motor {motor_mm:.3f} mm {'OK' if ok else 'FAIL'}")

    print("\n-- check 4: the freewheel re-engages without an impulse --------------------")
    failures += check_reengagement(quick)

    print("\n-- assist sweep: what the mid-drive does to the bob ------------------------")
    sweep_steps = 8000 if quick else 16000
    for mode in ("off", "eco", "tour", "sport", "turbo"):
        channels, _, _ = _run(track, speed, "pedelec", mode, 0.85, sweep_steps)
        s = pedal_bob_stats(channels, dt_s)
        if s is None:
            continue
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
