"""clone() of a drivetrain policy is independent of the original, like deepcopy."""
import copy
import numpy as np
import pytest


def _policies(tmp_path):
    # tools.golden_gate.make_env equivalent: the golden-profile env (pinned
    # spindle/pin/connect welded config on rough_uphill_savage, non-strict
    # monitor). The golden gate itself is not ported to this tree; this is the
    # same recipe test_pinned_topology/test_golden_episode already use here.
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '5', '--dt', '.00125', '--out', str(tmp_path/'out'),
        '--diagnostic-model-limits'])
    env = research_cli.make_environment(args)
    drive = env.sim.physical.drive
    return {'shifting': drive.shifting, 'pedaling': drive.pedaling, 'hub': drive.hub, 'assist': drive.assist}


def _mutable_leaves(obj):
    for name, value in vars(obj).items():
        if isinstance(value, (list, dict, set, np.ndarray)) or hasattr(value, '__dict__'):
            yield name, value


@pytest.mark.parametrize('name', ['shifting', 'pedaling', 'hub', 'assist'])
def test_clone_matches_deepcopy_and_shares_no_mutable_state(tmp_path, name):
    policy = _policies(tmp_path)[name]
    if policy is None:
        pytest.skip(f'{name} is not configured in the golden profile')
    cloned = policy.clone()
    reference = copy.deepcopy(policy)
    assert vars(cloned).keys() == vars(reference).keys()
    for attribute, value in _mutable_leaves(policy):
        assert getattr(cloned, attribute) is not value, f'{name}.{attribute} is shared with the clone'
