from dataclasses import asdict
import math
import numpy as np
import pytest
import _golden
from native_loader import load_native
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def _flat_control(control, demand_nm):
    """18-wide f8 row: the 6 optional/scalar control fields as (present, value)
    pairs, the posture's 4 fields the same way, then the demand pair.

    The C++ port's flatten_result emits the identical order.
    """
    def opt(value):
        return [float(value is not None), value if value is not None else 0.]
    p = control['posture']
    off = p['pelvis_offset_m'] if p is not None else None
    return (opt(control['motor_torque_nm']) + opt(control['motor_limit_nm'])
            + opt(control['human_torque_nm']) + opt(control['crank_target_rate_rad_s'])
            + [float(p is not None),
               p['torso_lean_rad'] if p is not None else 0.,
               p['pelvis_pitch_rad'] if p is not None else 0.,
               float(off is not None), off[0] if off is not None else 0.,
               off[1] if off is not None else 0.,
               float(p['use_saddle']) if p is not None else 0.,
               float(control['rider_enabled'])]
            + opt(demand_nm))


@pytest.mark.parametrize('offsets', [None, (0., 0.)])
@pytest.mark.parametrize('effort', [None, 0., 12.5])
def test_program_knots_optional_fields_and_association(native_module, assert_tree, offsets, effort):
    program = RiderProgram((RiderKeyframe(0., RiderPosture(pelvis_offset_m=offsets), effort),
        RiderKeyframe(.031, RiderPosture(.1, -.02, offsets, False),
            None if effort is None else effort+3.),
        RiderKeyframe(.067, RiderPosture(-.07, .03, offsets, True), effort)), reaction_delay_s=.004)
    demand = DemandProgram(((0., 1.125), (.017, 12.75), (.049, 2.0625)))
    command = RideControl(motor_torque_nm=None, motor_limit_nm=0.,
        human_torque_nm=4. if effort is None else None, crank_target_rate_rad_s=0., rider_enabled=False)
    times = [-.01, 0., .004, .0123, .021, .035, .071, 1.]
    times += [math.nextafter(.035, -math.inf), math.nextafter(.035, math.inf)]
    rows = []
    for time_s in times:
        result = native_module.research_program_at(program.to_dict(), demand.to_dict(), asdict(command), time_s)
        oracle = program.apply(command, time_s)
        assert_tree(result['control'], asdict(oracle), atol=0., rtol=0.)
        assert result['demand_nm'] == demand.at(time_s)
        assert result['control']['motor_torque_nm'] is None
        assert result['control']['motor_limit_nm'] == 0.
        rows += _flat_control(asdict(oracle), demand.at(time_s))
    _golden.record(np.asarray(rows).reshape(10, 18), 'program_at')


def test_no_program_preserves_none_and_explicit_zero(native_module, assert_tree):
    for command in (RideControl(), RideControl(0., 0., 0., 0., RiderPosture(), False)):
        actual = native_module.research_program_at(None, None, asdict(command), 0.)
        assert_tree(actual, dict(control=asdict(command), demand_nm=None), atol=0., rtol=0.)


def test_reject_mixed_effort_and_policy_posture(native_module):
    program = RiderProgram((RiderKeyframe(0., human_torque_nm=0.), RiderKeyframe(1., human_torque_nm=2.)))
    with pytest.raises(ValueError):
        native_module.research_program_at(program.to_dict(), None, asdict(RideControl(human_torque_nm=0.)), .5)
    with pytest.raises(ValueError):
        native_module.research_program_at(program.to_dict(), None, asdict(RideControl(posture=RiderPosture())), .5)
    malformed = program.to_dict()
    malformed['keyframes'][1]['human_torque_nm'] = None
    with pytest.raises(ValueError):
        native_module.research_program_at(malformed, None, asdict(RideControl()), .5)
