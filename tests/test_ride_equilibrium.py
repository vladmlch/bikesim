"""
Unit tests for the ride-mode suspension force path and the static equilibrium solve.

Tests include:
- Convergence of the damped relaxation on the `flat` preset, and the error when it fails.
- Front and rear sag at the shipped spring defaults, against docs/RIDE.md section 9.
- The solved state being genuinely at rest, and staying there when stepped.
- Reproducibility: two solves of the same model return the same state.
- Cross-validation of the relaxation against a free dynamic settle.
- The force path's sign convention and its refusal to run on the wrong joint range.
"""

from typing import NamedTuple

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.sim.controllers import SuspensionController
from bike_sim.sim.equilibrium import (
    START_CLEARANCE_M,
    solve_static_equilibrium,
)
from bike_sim.sim.ride import SuspensionForceApplier
from bike_sim.terrain import build_field_data, get_preset

# docs/RIDE.md section 9 tabulates the sag the shipped defaults produce: 42.0 % front
# (75.6 mm of 180) and 25.3 % rear (45.5 mm of 180 wheel travel). Those are analytic
# predictions from the static axle loads (478.2 N front, 546.0 N rear), and the solved
# equilibrium of the multibody model sits below both of them, by 1.3 pp at the front and
# 2.6 pp at the rear, for two reasons the table does not model:
#
#   1. The table routes the whole axle load through the spring. In the model the unsprung
#      mass below each spring -- fork lowers and front wheel; chainstay, seatstay and rear
#      wheel -- is carried directly by the contact patch, about 34 N at the front and 42 N
#      at the rear.
#   2. The table evaluates at the undeflected centre of mass. Sagging 73 mm at the front
#      and 41 mm at the rear pitches the bike 1.14 degrees nose-down, which moves roughly
#      20 N forward: the solved contact loads are 498 N front and 525 N rear.
#
# Feeding the remaining *sprung* loads (464.5 N and 483.0 N) back into the repository's own
# analytic calculators reproduces the solved travels to the digit, so the force path agrees
# with the analytic model exactly and only the table's load assumption differs. The values
# below are therefore the solved equilibrium, with section 9's figures kept alongside.
SPEC_FRONT_SAG_PCT = 42.0
SPEC_REAR_SAG_PCT = 25.3
SOLVED_FRONT_SAG_PCT = 40.7
SOLVED_REAR_SAG_PCT = 22.7

# A minimal model whose fork coordinate runs the wrong way: increasing value means
# extension, so a resisting force would need the opposite sign.
_TOP_OUT_RANGE_XML = """
<mujoco>
  <worldbody>
    <body>
      <joint name="fork_travel" type="slide" axis="0 0 1" range="-0.180 0"/>
      <joint name="shock_stroke" type="slide" axis="1 0 0" range="0 0.065"/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
"""


class Rig(NamedTuple):
    """A compiled ride model with its track, force path and analytical solver."""

    specs: BikeSpecs
    solver: HorstLinkageSolver
    model: mujoco.MjModel
    data: mujoco.MjData
    applier: SuspensionForceApplier


def _qposadr(model: mujoco.MjModel, joint_name: str) -> int:
    """Returns the qpos address of a named joint."""
    return int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)])


def _build_rig() -> Rig:
    """Compiles the ride model on the `flat` preset with the shipped suspension defaults."""
    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(specs=specs, solver=solver, mode="ride", include_rider=True)
    )
    data = mujoco.MjData(model)
    model.hfield_data[:] = build_field_data(get_preset("flat")).reshape(-1)

    controller = SuspensionController(
        specs=specs,
        air_spring=ForkAirSpring(
            specs=AirSpringSpecs(total_travel_mm=specs.fork_travel),
            num_tokens=specs.fork_air_tokens,
            gauge_pressure_psi=specs.fork_initial_psi,
        ),
        suspension_system=BikeSuspensionSystem(),
    )
    applier = SuspensionForceApplier(model, controller, CoilShock())
    return Rig(specs=specs, solver=solver, model=model, data=data, applier=applier)


@pytest.fixture(scope="module")
def rig() -> Rig:
    """Builds the ride rig once for the whole module: compilation is the expensive part."""
    return _build_rig()


@pytest.fixture(scope="module")
def equilibrium(rig: Rig) -> dict:
    """Solves the static equilibrium once. Tests that mutate `rig.data` re-solve their own."""
    return solve_static_equilibrium(rig.model, rig.data, rig.applier, rig.solver)


def test_equilibrium_converges_on_flat(rig: Rig, equilibrium: dict):
    """The relaxation converges well inside the step cap, with both joints in range."""
    assert equilibrium["residual_qacc"] <= 0.05
    assert 0 < equilibrium["steps"] < 40000
    assert 0.0 < equilibrium["fork_travel_mm"] < rig.specs.fork_travel
    assert 0.0 < equilibrium["shock_stroke_mm"] < rig.specs.shock_stroke


def test_unconverged_solve_raises(rig: Rig):
    """Running out of steps raises: an unsettled state is never returned as equilibrium."""
    with pytest.raises(RuntimeError, match="did not converge"):
        solve_static_equilibrium(rig.model, rig.data, rig.applier, rig.solver, max_steps=400)


def test_front_sag_at_shipped_defaults(rig: Rig, equilibrium: dict):
    """Front sag matches the solved equilibrium, and section 9's table within 2 pp."""
    front_sag_pct = equilibrium["fork_travel_mm"] / rig.specs.fork_travel * 100.0
    assert front_sag_pct == pytest.approx(SOLVED_FRONT_SAG_PCT, abs=1.0)
    assert front_sag_pct == pytest.approx(SPEC_FRONT_SAG_PCT, abs=2.0)


def test_rear_sag_at_shipped_defaults(rig: Rig, equilibrium: dict):
    """
    Rear sag matches the solved equilibrium, 2.6 pp below section 9's analytic table.

    See the reconciliation at the top of this module: the deficit is the unsprung mass the
    contact patch carries plus the nose-down load transfer at sag, neither of which the
    table models. It is not the pre-existing spring-calibration mismatch (README Known
    Limitation #1) -- that is what puts the figure at 25.3 % instead of 30 % to begin with.
    """
    rear_sag_pct = equilibrium["rear_travel_mm"] / rig.specs.rear_wheel_travel * 100.0
    assert rear_sag_pct == pytest.approx(SOLVED_REAR_SAG_PCT, abs=1.0)
    assert rear_sag_pct < SPEC_REAR_SAG_PCT


def test_solved_state_is_at_rest(rig: Rig):
    """The solve leaves the bike at rest, and it stays put when the run starts."""
    result = solve_static_equilibrium(rig.model, rig.data, rig.applier, rig.solver)
    assert float(np.max(np.abs(rig.data.qvel))) == 0.0

    for _ in range(200):  # 0.1 s of ride-mode steps
        rig.applier.apply(rig.model, rig.data)
        mujoco.mj_step(rig.model, rig.data)

    assert float(np.max(np.abs(rig.data.qvel))) < 0.02
    fork_travel_mm = float(rig.data.qpos[rig.applier.fork_qposadr]) * 1000.0
    shock_stroke_mm = float(rig.data.qpos[rig.applier.shock_qposadr]) * 1000.0
    assert fork_travel_mm == pytest.approx(result["fork_travel_mm"], abs=0.2)
    assert shock_stroke_mm == pytest.approx(result["shock_stroke_mm"], abs=0.2)


def test_equilibrium_is_reproducible(rig: Rig, equilibrium: dict):
    """Solving twice on the same model returns the same state: the solve resets its own."""
    again = solve_static_equilibrium(rig.model, rig.data, rig.applier, rig.solver)
    assert again == equilibrium


def test_relaxation_matches_free_dynamic_settle(rig: Rig, equilibrium: dict):
    """
    A free settle under real damping reaches the same state as the damped relaxation.

    The relaxation discards kinetic energy every 20 ms, which is not physical motion; this
    is the independent check that its fixed point is the one the dynamics actually find.
    """
    mujoco.mj_resetData(rig.model, rig.data)
    rig.data.qpos[_qposadr(rig.model, "root_x")] = 2.0
    rig.data.qpos[_qposadr(rig.model, "root_z")] = START_CLEARANCE_M
    mujoco.mj_forward(rig.model, rig.data)

    for _ in range(6000):  # 3 s at the ride timestep
        rig.applier.apply(rig.model, rig.data)
        mujoco.mj_step(rig.model, rig.data)

    fork_travel_mm = float(rig.data.qpos[rig.applier.fork_qposadr]) * 1000.0
    shock_stroke_mm = float(rig.data.qpos[rig.applier.shock_qposadr]) * 1000.0
    assert fork_travel_mm == pytest.approx(equilibrium["fork_travel_mm"], abs=0.25)
    assert shock_stroke_mm == pytest.approx(equilibrium["shock_stroke_mm"], abs=0.1)


def test_force_path_resists_compression_at_both_ends(rig: Rig):
    """
    Both suspension forces resist compression, and no leverage ratio is applied.

    The generalized force on `shock_stroke` is the axial shock force itself: MuJoCo applies
    the linkage's leverage ratio -- around 3.0 here -- through the constraint Jacobian
    (docs/RIDE.md section 5), so an analytic ratio in the force path would count it twice.
    """
    applier = rig.applier
    mujoco.mj_resetData(rig.model, rig.data)
    rig.data.qpos[applier.fork_qposadr] = 0.050
    rig.data.qpos[applier.shock_qposadr] = 0.020
    mujoco.mj_forward(rig.model, rig.data)
    applier.apply(rig.model, rig.data)

    assert applier.fork_spring_n > 0.0
    assert applier.shock_spring_n > 0.0
    assert rig.data.qfrc_applied[applier.fork_dofadr] == pytest.approx(-applier.fork_total_n)
    assert rig.data.qfrc_applied[applier.shock_dofadr] == pytest.approx(-applier.shock_total_n)

    # At zero shaft velocity the dampers are silent, and the shock force is the axial
    # spring force alone: 20 mm of stroke below the bumper, with no leverage ratio on it.
    assert applier.fork_damper_n == pytest.approx(0.0)
    assert applier.shock_damper_n == pytest.approx(0.0)
    assert applier.shock_bumper_n == 0.0
    assert applier.shock_total_n == pytest.approx(applier.coil_shock.compute_axial_force(20.0))


@pytest.mark.parametrize("shaft_velocity_mps,expected_sign", [(0.5, 1.0), (-0.5, -1.0)])
def test_damping_opposes_shaft_motion(rig: Rig, shaft_velocity_mps: float, expected_sign: float):
    """Damping adds resistance while compressing and subtracts it while rebounding."""
    applier = rig.applier
    mujoco.mj_resetData(rig.model, rig.data)
    rig.data.qpos[applier.fork_qposadr] = 0.050
    rig.data.qpos[applier.shock_qposadr] = 0.020
    rig.data.qvel[applier.fork_dofadr] = shaft_velocity_mps
    rig.data.qvel[applier.shock_dofadr] = shaft_velocity_mps
    mujoco.mj_forward(rig.model, rig.data)
    applier.apply(rig.model, rig.data)

    assert np.sign(applier.fork_damper_n) == expected_sign
    assert np.sign(applier.shock_damper_n) == expected_sign
    assert applier.fork_total_n == pytest.approx(applier.fork_spring_n + applier.fork_damper_n)
    assert applier.shock_total_n == pytest.approx(
        applier.shock_spring_n + applier.shock_bumper_n + applier.shock_damper_n
    )


def test_force_path_rejects_a_non_compression_range():
    """A joint whose range does not start at zero is refused, not silently mis-signed."""
    model = mujoco.MjModel.from_xml_string(_TOP_OUT_RANGE_XML)
    controller = SuspensionController(
        specs=BikeSpecs(),
        air_spring=ForkAirSpring(),
        suspension_system=BikeSuspensionSystem(),
    )

    with pytest.raises(ValueError, match="fork_travel"):
        SuspensionForceApplier(model, controller, CoilShock())
